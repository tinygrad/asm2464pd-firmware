#!/usr/bin/env python3
"""Scan all SFRs 0x80-0xFF: which one makes MOVX write to CODE space?"""
import ctypes, sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'pcie'))
from pcie_probe import usb_open, usb_close, xdata_read, xdata_write
from tinygrad.runtime.autogen import libusb

def sfr_test(handle, sfr_addr, sfr_val):
    """E7: test if SFR[sfr_addr]=sfr_val makes MOVX write CODE. Returns (hit, before, after, old_sfr)."""
    buf = (ctypes.c_ubyte * 4)()
    ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE7, sfr_addr, sfr_val, buf, 4, 2000)
    if ret < 0:
        return None
    return buf[0], buf[1], buf[2], buf[3]

time.sleep(1)
handle, ctx = usb_open()
print("Device opened")

# Skip dangerous SFRs: 0x81 (SP), 0xA8 (IE), 0xD0 (PSW), 0xE0 (ACC), 0xF0 (B)
skip = {0x81, 0xA8, 0xD0, 0xE0, 0xF0, 0x88}  # SP, IE, PSW, ACC, B, TCON

print("Scanning SFRs 0x80-0xFF with values 0x01, 0x02, 0x04, ... 0x80, 0xFF")
for sfr in range(0x80, 0x100):
    if sfr in skip:
        continue
    for val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]:
        try:
            result = sfr_test(handle, sfr, val)
        except:
            print(f"  SFR 0x{sfr:02X}=0x{val:02X}: device died, reopening...")
            try: usb_close(handle, ctx)
            except: pass
            time.sleep(2)
            handle, ctx = usb_open()
            break
        if result is None:
            break
        hit, before, after, old = result
        if hit:
            print(f"  *** SFR 0x{sfr:02X}=0x{val:02X}: CODE WRITE WORKS! before=0x{before:02X} after=0x{after:02X} old_sfr=0x{old:02X} ***")
        elif after != before:
            print(f"  SFR 0x{sfr:02X}=0x{val:02X}: CODE changed but not to 0xAA: before=0x{before:02X} after=0x{after:02X}")

print("\nDone!")
usb_close(handle, ctx)
