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


def poll(dev, addr, expected, timeout_s=5.0, interval_s=0.01, mask=0xFF, name=None):
    """Poll register until (value & mask) == expected. Returns value. Raises on timeout."""
    label = name or f"0x{addr:04X}"
    t0 = time.monotonic()
    for i in range(int(timeout_s / interval_s)):
        v = dev.read8(addr)
        if (v & mask) == expected:
            print(f"  {label}: {(time.monotonic()-t0)*1000:.1f}ms ({i+1} polls, 0x{v:02X})")
            return v
        time.sleep(interval_s)
    raise RuntimeError(f"{label} timeout after {time.monotonic()-t0:.3f}s (0x{addr:04X}=0x{dev.read8(addr):02X}, wanted 0x{expected:02X} mask 0x{mask:02X})")


def main():
    dev = ASM2464PD()
    dev.open()
    try:
        # verify firmware set B213
        assert dev.read8(0xB213) == 0x01, f"B213=0x{dev.read8(0xB213):02X}, firmware didn't set TLP_CTRL"

        # === Power + Deassert PERST# ===
        dev.write(0xB480, 0x01)                          # 0. assert PERST#
        dev.write(0xC656, 0x20)                          # 1. enable 3.3V
        dev.write(0xC659, 0x01)                          # 2. enable 12V

        # === Gen3 link training ===
        dev.write(0xE764, 0x1E)                          # 4. training trigger
        poll(dev, 0xE762, 0x10, mask=0x10, name="RXPLL lock")

        # === Post-train ===
        dev.write(0xB430, 0x04)                          # 5. set tunnel link state
        dev.bank1_or_bits(0x6025, 0x80)                  # 6. TLP routing enable
        dev.write(0xB480, 0x00)                          # 3. deassert PERST#

        poll(dev, 0xB450, 0x78, name="LTSSM Gen3 L0")

        # === Status (reads don't count) ===
        print(f"\nTotal: {dev._wc} writes, {dev._rc} reads")
        print(f"\n=== PCIe Status ===")
        print(f"  LTSSM state (B450): 0x{dev.read8(0xB450):02X}")
        print(f"  Link detect (B455): 0x{dev.read8(0xB455):02X}")
        print(f"  PERST ctrl  (B480): 0x{dev.read8(0xB480):02X}")
        print(f"  3.3V enable (C656): 0x{dev.read8(0xC656):02X}")
        print(f"  12V enable  (C659): 0x{dev.read8(0xC659):02X}")
        print(f"  RXPLL       (E762): 0x{dev.read8(0xE762):02X}")
        print(f"  PHY timer   (E764): 0x{dev.read8(0xE764):02X}")
        print(f"  Tunnel link (B430): 0x{dev.read8(0xB430):02X}")
        print(f"  Tunnel ctrl (B403): 0x{dev.read8(0xB403):02X}")
        sys.exit(0)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
