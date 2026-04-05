#!/usr/bin/env python3
"""Replay stock firmware register init sequence.

Run after pcie_bringup.py gets LTSSM to 0x78.
Replays all writes from the stock trace post-train through NVMe init.
"""
import sys, time, ctypes
from pcie.pcie_probe import usb_open, usb_close, xdata_read, xdata_write
from tinygrad.runtime.autogen import libusb
from stock_replay import WRITES

def dpx_write(handle, addr, val):
    ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, addr & 0xFFFF, val | (1 << 8), None, 0, 1000)
    assert ret >= 0, f"DPX write 0x{addr:04X}=0x{val:02X} failed: {ret}"

def main():
    handle, ctx = usb_open()
    try:
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        print(f"LTSSM=0x{ltssm:02X}")
        if ltssm != 0x78:
            print("Need LTSSM=0x78, run pcie_bringup.py first")
            return 1

        print(f"Replaying {len(WRITES)} writes...")
        n = 0
        for addr, val, dpx in WRITES:
            if dpx:
                dpx_write(handle, addr, val)
            else:
                xdata_write(handle, addr, val)
            n += 1
            if n % 2000 == 0:
                print(f"  {n}/{len(WRITES)}...")

        print(f"Done: {n} writes")

        # Check state
        for addr in [0xC802, 0xC805, 0xC809, 0xC8D7, 0xC42A, 0xC412, 0xC428, 0xCEF3]:
            v = xdata_read(handle, addr, 1)[0]
            print(f"  0x{addr:04X} = 0x{v:02X}")
        return 0
    finally:
        usb_close(handle, ctx)

if __name__ == "__main__":
    sys.exit(main() or 0)
