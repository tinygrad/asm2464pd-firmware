#!/usr/bin/env python3
"""
For each register C400-C420: write 0xFF, read back, restore.
The readback tells us the writable mask.
"""
import sys, ctypes
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0

# Skip C406 (trigger) and C42A (doorbell)
SKIP = {0xC406}

for addr in range(0xC400, 0xC420):
  if addr in SKIP:
    print(f"  0x{addr:04X}: SKIP (trigger)")
    continue
  orig = read8(addr)
  write(addr, 0xFF)
  rb = read8(addr)
  write(addr, orig)
  print(f"  0x{addr:04X}: orig=0x{orig:02X}  wrote=0xFF  readback=0x{rb:02X}  mask=0x{rb:02X}")
