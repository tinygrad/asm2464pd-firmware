#!/usr/bin/env python3
"""
Dump same registers on HANDMADE firmware after dma_explore priming.
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

def read_range(addr, n):
  return bytes([read8(addr+i) for i in range(n)])

print("Handmade firmware register dump after dma_explore:")
print("(only non-zero/non-0x55 shown)")
print()

for base_name, base_start, base_end in [
    ("C400", 0xC400, 0xC500),
    ("C800", 0xC800, 0xC900),
    ("CE00", 0xCE00, 0xCF00),
    ("9000", 0x9000, 0x9100),
    ("9100", 0x9100, 0x9200),
    ("9200", 0x9200, 0x9300),
    ("9300", 0x9300, 0x9400),
]:
    print(f"--- {base_name} ---")
    for addr in range(base_start, base_end):
        try:
            v = read8(addr)
            if v != 0x00 and v != 0x55:
                print(f"  0x{addr:04X} = 0x{v:02X}")
        except:
            print(f"  0x{addr:04X} = ERROR")
            break
    print()
