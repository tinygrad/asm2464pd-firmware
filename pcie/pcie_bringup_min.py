#!/usr/bin/env python3
"""Minimal PCIe bringup for ASM2464PD."""

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
    def bank1_or_bits(self, addr, mask):
        self._rc += 1
        buf = (ctypes.c_ubyte * 1)()
        libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 1 << 8, buf, 1, 1000)
        self._wc += 1
        libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, (buf[0] | mask) | (1 << 8), None, 0, 1000)


def main():
    dev = ASM2464PD()
    dev.open()
    try:
        # verify firmware set B213
        assert dev.read8(0xB213) == 0x01, f"B213=0x{dev.read8(0xB213):02X}, firmware didn't set TLP_CTRL"

        # === Power + Deassert PERST# ===
        dev.set_bits(0xC656, 0x20)                       # 1. enable 3.3V
        dev.set_bits(0xC659, 0x01)                       # 2. enable 12V
        dev.clear_bits(0xB480, 0x01)                     # 3. deassert PERST#

        # === Gen3 link training ===
        dev.write(0xE764, (dev.read8(0xE764) & 0xF7) | 0x08)  # 5. PHY training prep (set bit 3)
        dev.write(0xE764, (dev.read8(0xE764) & 0xFD) | 0x02)  # 6. start training (set bit 1)
        print(hex(dev.read8(0xE764)))

        # === Wait for RXPLL lock ===
        """
        t0 = time.monotonic()
        for i in range(500):
            if dev.read8(0xE762) & 0x10: break
            time.sleep(0.01)
        else:
            raise RuntimeError(f"RXPLL never locked after {time.monotonic()-t0:.3f}s (E762=0x{dev.read8(0xE762):02X})")
        print(f"  RXPLL locked: {(time.monotonic()-t0)*1000:.1f}ms ({i+1} polls)")
        """

        dev.set_bits(0xB403, 0x01)                       # 4. enable tunnel
        # === Post-train (do before LTSSM poll so link stays stable) ===
        dev.clear_bits(0xB430, 0x01)                     # 7. clear tunnel link state
        dev.bank1_or_bits(0x6025, 0x80)                  # 8. TLP routing enable

        # === Wait for LTSSM Gen3 L0 (0x78) ===
        t0 = time.monotonic()
        for i in range(500):
            v = dev.read8(0xB450)
            if v == 0x78: break
            time.sleep(0.01)
        else:
            raise RuntimeError(f"LTSSM never reached Gen3 L0 after {time.monotonic()-t0:.3f}s (B450=0x{dev.read8(0xB450):02X})")
        print(f"  LTSSM Gen3 L0: {(time.monotonic()-t0)*1000:.1f}ms ({i+1} polls)")

        # === Status (reads don't count) ===
        print(f"\nTotal: {dev._wc} writes, {dev._rc} reads")
        print(f"\n=== PCIe Status ===")
        print(f"  LTSSM state (B450): 0x{dev.read8(0xB450):02X}")
        print(f"  Link detect (B455): 0x{dev.read8(0xB455):02X}")
        print(f"  PERST ctrl  (B480): 0x{dev.read8(0xB480):02X}")
        print(f"  12V enable  (C659): 0x{dev.read8(0xC659):02X}")
        print(f"  RXPLL       (E762): 0x{dev.read8(0xE762):02X}")
        print(f"  LINK        (E763): 0x{dev.read8(0xE763):02X}")
        print(f"  PHY timer   (E764): 0x{dev.read8(0xE764):02X}")
        print(f"  Tunnel link (B430): 0x{dev.read8(0xB430):02X}")
        print(f"  Tunnel ctrl (B403): 0x{dev.read8(0xB403):02X}")
        sys.exit(0)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
