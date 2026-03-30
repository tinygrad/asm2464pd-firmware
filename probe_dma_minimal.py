#!/usr/bin/env python3
"""
Minimal C400 DMA test on handmade firmware.
Exactly replicating dma_explore.py logic.
"""
import sys, ctypes
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0, f"read8(0x{addr:04X}) failed: {ret}"
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

def read_range(addr, n):
  return bytes([read8(addr + i) for i in range(n)])

# Exactly dma_explore.dma_trigger
def dma_trigger(sectors=1):
  write(0xC420, 0x00)
  write(0xC421, 0x01)
  write(0xC422, 0x02)
  write(0xC423, 0x00)
  write(0xC424, 0x00)
  write(0xC425, 0x00)
  write(0xC426, 0x00)
  write(0xC427, sectors)
  write(0xC428, 0x30)
  write(0xC429, 0x00)
  write(0xC42A, 0x00)
  write(0xC42B, 0x00)
  write(0xC42C, 0x00)
  write(0xC42D, 0x00)
  write(0xC400, 1)
  write(0xC401, 1)
  write(0xC402, 1)
  write(0xC404, 1)
  write(0xC406, 1)

def dma_unlock():
  write(0xC42A, 0x01)

# Run exactly like dma_explore
print("Minimal DMA test (replicating dma_explore):\n")
for i in range(5):
  pat = bytes([0x10*i + j for j in range(16)]) + b'\x00' * 16
  dev._bulk_out(0x02, pat)
  x7 = read_range(0x7000, 4)
  dma_trigger()
  f = read_range(0xF000, 4)
  ok = f == pat[:4]
  print(f"  [{i}] 7000={' '.join(f'{b:02X}' for b in x7)} "
        f"F000={' '.join(f'{b:02X}' for b in f)} {'OK' if ok else 'FAIL'}")
  dma_unlock()

print("\nDone!")
