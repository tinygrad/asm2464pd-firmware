#!/usr/bin/env python3
"""Minimal PCIe bringup — binary search for minimum required register set."""

import ctypes, sys, time
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

SUPPORTED_DEVICES = [(0xADD1, 0x0001), (0x174C, 0x2463), (0x174C, 0x2464)]

class ASM2464PD:
    def __init__(self):
        self.dev = None
        self._wc = 0
        self._rc = 0
    def open(self):
        for vid, pid in SUPPORTED_DEVICES:
            try:
                self.dev = USB3(vid, pid, 0x81, 0x83, 0x02, 0x04, use_bot=True)
                print(f"Opened {vid:04X}:{pid:04X}")
                return
            except RuntimeError: pass
        raise RuntimeError("No device found")
    def close(self):
        if self.dev:
            libusb.libusb_release_interface(self.dev.handle, 0)
            libusb.libusb_close(self.dev.handle)
    def read8(self, addr):
        self._rc += 1
        buf = (ctypes.c_ubyte * 1)()
        ret = libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
        assert ret >= 0, f"read 0x{addr:04X} failed: {ret}"
        return buf[0]
    def write(self, addr, val):
        self._wc += 1
        ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write 0x{addr:04X}=0x{val:02X} failed: {ret}"
    def set_bits(self, addr, mask):
        self.write(addr, self.read8(addr) | mask)
    def clear_bits(self, addr, mask):
        self.write(addr, self.read8(addr) & ~mask & 0xFF)
    def rmw(self, addr, clear_mask, set_mask):
        self.write(addr, (self.read8(addr) & ~clear_mask | set_mask) & 0xFF)
    def bank1_write(self, addr, val):
        self._wc += 1
        ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val | (1 << 8), None, 0, 1000)
        assert ret >= 0
    def bank1_or_bits(self, addr, mask):
        self._rc += 1
        buf = (ctypes.c_ubyte * 1)()
        libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 1 << 8, buf, 1, 1000)
        self.bank1_write(addr, buf[0] | mask)


def timer_wait(dev, hi, lo, mode, timeout_s=2.0):
    dev.write(0xCC11, 0x04)
    dev.write(0xCC11, 0x02)
    div = dev.read8(0xCC10)
    dev.write(0xCC10, (div & 0xF8) | (mode & 0x07))
    dev.write(0xCC12, hi)
    dev.write(0xCC13, lo)
    dev.write(0xCC11, 0x01)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dev.read8(0xCC11) & 0x02: break
        time.sleep(0.01)
    dev.write(0xCC11, 0x02)


def phy_soft_reset(dev):
    dev.set_bits(0xCC37, 0x04)
    dev.write(0xCA70, 0x00)
    dev.write(0xE780, 0x00)
    dev.write(0xCC11, 0x04); dev.write(0xCC11, 0x02)
    div = dev.read8(0xCC10)
    dev.write(0xCC10, (div & 0xF8) | 0x02)
    dev.write(0xCC12, 0x00); dev.write(0xCC13, 0xC8)
    dev.write(0xCC11, 0x01)
    for _ in range(300):
        if dev.read8(0xE712) & 0x03: break
        if dev.read8(0xCC11) & 0x02: break
        time.sleep(0.01)
    dev.write(0xCC11, 0x04); dev.write(0xCC11, 0x02)
    dev.clear_bits(0xCC37, 0x04)


def link_controller_init(dev):
    dev.write(0xCC32, 0x01)
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | 0x04)
    dev.set_bits(0xC6A8, 0x01)
    dev.write(0xCC33, 0x04)
    dev.clear_bits(0xE324, 0x04)
    dev.clear_bits(0xCC3B, 0x01)
    dev.set_bits(0xE717, 0x01)
    dev.clear_bits(0xCC3E, 0x02)
    dev.clear_bits(0xCC3B, 0x02)
    dev.clear_bits(0xCC3B, 0x40)
    dev.set_bits(0xCC3B, 0x03)
    e716 = dev.read8(0xE716)
    dev.write(0xE716, (e716 & 0xFC) | 0x03)
    dev.clear_bits(0xCC3E, 0x01)
    dev.set_bits(0xCC39, 0x02)
    dev.clear_bits(0xCC3A, 0x02)
    dev.clear_bits(0xCC38, 0x02)
    ca06 = dev.read8(0xCA06)
    dev.write(0xCA06, (ca06 & 0x1F) | 0x60)
    dev.set_bits(0xCA81, 0x01)


def lane_config(dev, mask=0x0F):
    b402_saved = dev.read8(0xB402) & 0x02
    dev.clear_bits(0xB402, 0x02)
    dev.clear_bits(0xB402, 0x02)
    if mask == 0x0F:
        for step in [0x01, 0x03, 0x07, 0x0F]:
            b434 = dev.read8(0xB434)
            dev.write(0xB434, step | (b434 & 0xF0))
            timer_wait(dev, 0x00, 0xC7, 0x02)
    else:
        current = dev.read8(0xB434) & 0x0F
        counter = 0x01
        for _ in range(4):
            if current == mask: break
            new = current & (mask | (counter ^ 0x0F))
            current = new
            b434 = dev.read8(0xB434)
            dev.write(0xB434, new | (b434 & 0xF0))
            timer_wait(dev, 0x00, 0xC7, 0x02)
            counter = (counter << 1) & 0xFF
    if mask != 0x0F:
        dev.set_bits(0xB401, 0x01)
        b401 = dev.read8(0xB401)
        dev.write(0xB401, b401 & 0xFE)
    if b402_saved: dev.set_bits(0xB402, 0x02)
    b436 = dev.read8(0xB436)
    dev.write(0xB436, (b436 & 0xF0) | (mask & 0x0E))
    b404 = dev.read8(0xB404)
    upper = ((b404 & 0x0F) ^ 0x0F) << 4
    b436 = dev.read8(0xB436)
    dev.write(0xB436, (b436 & 0x0F) | (upper & 0xF0))


def pcie_phy_channel_config(dev, mode):
    dev.write(0x05AB, mode)
    for i in range(12): dev.write(0xB210 + i, 0x00)
    dev.write(0xB210, 0x40 if (mode & 0x01) else 0x00)
    dev.write(0xB213, 0x01); dev.write(0xB217, 0x0F); dev.write(0xB216, 0x20)
    dev.write(0xB218, dev.read8(0x05AC)); dev.write(0xB219, dev.read8(0x05AD))
    dev.write(0xB21A, dev.read8(0x05AE)); dev.write(0xB21B, dev.read8(0x05AF))
    dev.write(0xB296, 0x01); dev.write(0xB296, 0x02); dev.write(0xB296, 0x04)
    dev.write(0xB254, 0x0F)
    for _ in range(100):
        if dev.read8(0xB296) & 0x04: break
        time.sleep(0.001)
    dev.write(0xB296, 0x04)
    if mode & 0x01: return 0
    for _ in range(100):
        b296 = dev.read8(0xB296)
        if b296 & 0x02:
            dev.write(0xB296, 0x02)
            return 0
        if b296 & 0x01:
            dev.write(0xB296, 0x01)
            return 0xFE
        time.sleep(0.001)
    return 0xFE


def main():
    dev = ASM2464PD()
    dev.open()
    try:
        # SKIP Phase 1 entirely

        print("=== Phase 2: Power ===")
        # 3.3V
        c656 = dev.read8(0xC656)
        if not (c656 & 0x20):
            dev.write(0xC656, (c656 & 0xDF) | 0x20)
            dev.set_bits(0xC65B, 0x20)
        # 12V
        dev.set_bits(0xC659, 0x01)

        # SKIP Phase 3 (Tunnel) entirely

        print("=== Phase 4: PERST Deassert ===")
        dev.write(0xB455, 0x02); dev.write(0xB455, 0x04)
        dev.write(0xB2D5, 0x01); dev.write(0xB296, 0x08)
        # Config space write (0x00D00014)
        dev.write(0x05AC, 0x00); dev.write(0x05AD, 0xD0)
        dev.write(0x05AE, 0x00); dev.write(0x05AF, 0x14)
        dev.write(0xB220, 0x00); dev.write(0xB221, 0x46)
        dev.write(0xB222, 0x40); dev.write(0xB223, 0x01)
        pcie_phy_channel_config(dev, 1)
        # SKIP readiness poll (always fails anyway)
        # PERST deassert
        dev.clear_bits(0xB480, 0x01)
        print("  [PERST deasserted]")

        print("=== Phase 5: Gen3 Retraining ===")
        e710 = dev.read8(0xE710); dev.write(0xE710, (e710 & 0xE0) | 0x04)
        timer_wait(dev, 0x01, 0x2B, 0x04)
        dev.clear_bits(0xCA81, 0x01)
        ca06 = dev.read8(0xCA06); dev.write(0xCA06, (ca06 & 0x1F) | 0x20)
        dev.set_bits(0xB403, 0x01)
        b431 = dev.read8(0xB431); dev.write(0xB431, (b431 & 0xF0) | 0x0C)
        lane_config(dev, 0x0C)
        saved_e710 = dev.read8(0xE710) & 0x1F
        e710 = dev.read8(0xE710); dev.write(0xE710, (e710 & 0xE0) | 0x1F)
        saved_ca06 = dev.read8(0xCA06) & 0xE0
        dev.write(0xE751, 0x01)
        e764 = dev.read8(0xE764); dev.write(0xE764, (e764 & 0xF7) | 0x08)
        e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFB)
        e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFE)
        e764 = dev.read8(0xE764); dev.write(0xE764, (e764 & 0xFD) | 0x02)
        timer_wait(dev, 0x07, 0xCF, 0x01, timeout_s=5.0)
        rxpll = dev.read8(0xE762)
        link_ok = bool(rxpll & 0x10)
        print(f"  RXPLL=0x{rxpll:02X} link_ok={link_ok}")
        if link_ok:
            e764 = dev.read8(0xE764); dev.write(0xE764, (e764 & 0xFE) | 0x01)
            e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFD)
        else:
            e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xF7)
            e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFB)
            e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFE)
            e764 = dev.read8(0xE764); dev.write(0xE764, e764 & 0xFD)
        e710 = dev.read8(0xE710); dev.write(0xE710, (e710 & 0xE0) | saved_e710)
        ca06 = dev.read8(0xCA06); dev.write(0xCA06, (ca06 & 0x1F) | saved_ca06)
        dev.clear_bits(0xCA81, 0x01)
        dev.set_bits(0xC659, 0x01)
        timer_wait(dev, 0x03, 0xE7, 0x04, timeout_s=5.0)

        print("=== Phase 6: Post-Train ===")
        # bridge control — minimal
        dev.clear_bits(0xB430, 0x01)
        dev.bank1_or_bits(0x6025, 0x80)  # TLP routing enable
        # SKIP B481
        # B455 trigger
        dev.write(0xB455, 0x02); dev.write(0xB455, 0x04)
        dev.write(0xB2D5, 0x01); dev.write(0xB296, 0x08)
        for _ in range(200):
            if dev.read8(0xB455) & 0x02:
                dev.write(0xB455, 0x02); break
            time.sleep(0.005)

        # Status
        ltssm = dev.read8(0xB450)
        print(f"\nLTSSM=0x{ltssm:02X} {'UP' if ltssm in (0x48,0x78) else 'DOWN'}")
        print(f"Total: {dev._wc} writes, {dev._rc} reads")
        sys.exit(0 if ltssm in (0x48, 0x78) else 1)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
