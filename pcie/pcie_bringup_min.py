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
        # === Power ===
        dev.set_bits(0xC656, 0x20)                       # 1  REG_HDDPC_CTRL: enable PCIE_3V3 (bit 5)
        dev.set_bits(0xC65B, 0x20)                       # 2  REG_PHY_EXT_5B: PHY mode for 3.3V output
        dev.set_bits(0xC659, 0x01)                       # 3  REG_PCIE_LANE_CTRL: enable 12V (bit 0)

        # === PERST: CfgWr0 to 0x00D00014 ===
        dev.write(0xB455, 0x02)                          # 4  REG_PCIE_LTSSM_B455: clear link detect
        dev.write(0xB455, 0x04)                          # 5  REG_PCIE_LTSSM_B455: arm link detect
        dev.write(0xB2D5, 0x01)                          # 6  REG_PCIE_CTRL_B2D5: enable config routing
        dev.write(0xB296, 0x08)                          # 7  REG_PCIE_STATUS: reset TLP engine
        for i in range(12): dev.write(0xB210+i, 0x00)   # 8  clear TLP header (12 writes)
        dev.write(0xB210, 0x40)                          # 9  REG_PCIE_FMT_TYPE: CfgWr0
        dev.write(0xB213, 0x01)                          # 10 REG_PCIE_TLP_CTRL: 1 DW
        dev.write(0xB217, 0x0F)                          # 11 REG_PCIE_BYTE_EN: all 4 bytes
        dev.write(0xB216, 0x20)                          # 12 REG_PCIE_TLP_LENGTH
        dev.write(0xB218, 0x00)                          # 13 addr[7:0]
        dev.write(0xB219, 0xD0)                          # 14 addr[15:8]
        dev.write(0xB21A, 0x00)                          # 15 addr[23:16]
        dev.write(0xB21B, 0x14)                          # 16 addr[31:24]
        dev.write(0xB220, 0x00)                          # 17 data[0]
        dev.write(0xB221, 0x46)                          # 18 data[1]
        dev.write(0xB222, 0x40)                          # 19 data[2]
        dev.write(0xB223, 0x01)                          # 20 data[3]
        dev.write(0xB296, 0x01)                          # 21 clear error
        dev.write(0xB296, 0x02)                          # 22 clear completion
        dev.write(0xB296, 0x04)                          # 23 arm busy
        dev.write(0xB254, 0x0F)                          # 24 REG_PCIE_TRIGGER: execute
        for _ in range(100):                             #    poll busy
            if dev.read8(0xB296) & 0x04: break
            time.sleep(0.001)
        dev.write(0xB296, 0x04)                          # 25 clear busy

        # === PERST deassert ===
        dev.clear_bits(0xB480, 0x01)                     # 26 deassert PERST#

        # === Gen3 retraining ===
        dev.write(0xCA06, (dev.read8(0xCA06) & 0x1F) | 0x20)  # 27 REG_CPU_MODE_NEXT: Gen3
        dev.set_bits(0xB403, 0x01)                       # 28 REG_TUNNEL_CTRL_B403: enable
        # lane_config(0x0C) inlined
        dev.clear_bits(0xB402, 0x02)                     # 29 REG_PCIE_CTRL_B402: disable during reconfig
        dev.clear_bits(0xB402, 0x02)                     # 30 (redundant, matches stock)
        current = dev.read8(0xB434) & 0x0F               #    read current lane mask
        counter = 0x01
        for _ in range(4):                               # 31-34 progressive lane disable
            if current == 0x0C: break
            new = current & (0x0C | (counter ^ 0x0F))
            current = new
            b434 = dev.read8(0xB434)
            dev.write(0xB434, new | (b434 & 0xF0))
            time.sleep(0.01)
            counter = (counter << 1) & 0xFF
        dev.set_bits(0xB401, 0x01)                       # 35 REG_PCIE_TUNNEL_CTRL: pulse enable
        dev.write(0xB401, dev.read8(0xB401) & 0xFE)     # 36 REG_PCIE_TUNNEL_CTRL: pulse disable
        b436 = dev.read8(0xB436)
        dev.write(0xB436, (b436 & 0xF0) | (0x0C & 0x0E)) # 37 REG_PCIE_LANE_CONFIG: low nibble
        b404 = dev.read8(0xB404)
        upper = ((b404 & 0x0F) ^ 0x0F) << 4
        b436 = dev.read8(0xB436)
        dev.write(0xB436, (b436 & 0x0F) | (upper & 0xF0)) # 38 REG_PCIE_LANE_CONFIG: high nibble
        # end lane_config

        dev.write(0xE710, (dev.read8(0xE710) & 0xE0) | 0x1F)  # 39 REG_LINK_WIDTH: all lanes for EQ
        dev.write(0xE751, 0x01)                          # 40 REG_PHY_POLL: equalization mode
        # E764 training trigger
        dev.write(0xE764, (dev.read8(0xE764) & 0xF7) | 0x08)  # 41 set bit 3
        dev.write(0xE764, dev.read8(0xE764) & 0xFB)     # 42 clear bit 2
        dev.write(0xE764, dev.read8(0xE764) & 0xFE)     # 43 clear bit 0
        dev.write(0xE764, (dev.read8(0xE764) & 0xFD) | 0x02)  # 44 set bit 1 (start training)
        # poll RXPLL
        for _ in range(200):
            if dev.read8(0xE762) & 0x10: break
            time.sleep(0.01)

        # === Post-train ===
        dev.clear_bits(0xB430, 0x01)                     # 45 REG_TUNNEL_LINK_STATE: clear
        dev.bank1_or_bits(0x6025, 0x80)                  # 46 TLP routing enable
        dev.write(0xB455, 0x02)                          # 47 clear link detect
        dev.write(0xB455, 0x04)                          # 48 arm link detect
        dev.write(0xB2D5, 0x01)                          # 49 config routing
        dev.write(0xB296, 0x08)                          # 50 reset TLP engine
        for _ in range(200):                             #    poll link detect
            if dev.read8(0xB455) & 0x02:
                dev.write(0xB455, 0x02); break
            time.sleep(0.005)

        # === Status (reads don't count) ===
        print(f"\nTotal: {dev._wc} writes, {dev._rc} reads")
        ltssm = dev.read8(0xB450)
        print(f"\n=== PCIe Status ===")
        print(f"  LTSSM state (B450): 0x{ltssm:02X}  {'(L0!)' if ltssm in (0x48,0x78) else ''}")
        print(f"  Link width  (B22B): 0x{dev.read8(0xB22B):02X}")
        print(f"  Lane enable (B434): 0x{dev.read8(0xB434):02X}")
        print(f"  Link detect (B455): 0x{dev.read8(0xB455):02X}")
        print(f"  Link width  (E710): 0x{dev.read8(0xE710):02X}")
        print(f"  CPU mode    (CC30): 0x{dev.read8(0xCC30):02X}")
        print(f"  CPU next    (CA06): 0x{dev.read8(0xCA06):02X}")
        print(f"  PERST ctrl  (B480): 0x{dev.read8(0xB480):02X}  {'(asserted)' if dev.read8(0xB480) & 0x01 else '(deasserted)'}")
        print(f"  12V enable  (C659): 0x{dev.read8(0xC659):02X}  {'(on)' if dev.read8(0xC659) & 0x01 else '(off)'}")
        print(f"  RXPLL       (E762): 0x{dev.read8(0xE762):02X}")
        print(f"  PHY timer   (E764): 0x{dev.read8(0xE764):02X}")
        print(f"  Tunnel link (B430): 0x{dev.read8(0xB430):02X}")
        print(f"  Tunnel ctrl (B403): 0x{dev.read8(0xB403):02X}")
        print(f"  Link status: {'UP' if ltssm in (0x48,0x78) else 'DOWN'}")
        sys.exit(0 if ltssm in (0x48, 0x78) else 1)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
