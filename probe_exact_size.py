#!/usr/bin/env python3
"""
Exact DMA byte count for various C403/C427 combinations.
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

def measure_dma_size(c403, c427, bulk_size=4096):
  """Send bulk_size bytes of 0xFF, clear SRAM, trigger, find boundary."""
  dev._bulk_out(0x02, bytes([0xFF]*bulk_size))
  
  # Clear enough SRAM
  clear_size = min(bulk_size + 0x100, 0x1000)
  for i in range(0, clear_size, 16):
    # Write 16 zeros at a time via E5 (faster)
    for j in range(16):
      write(0xF000 + i + j, 0x00)
  
  write(0xC403, c403)
  write(0xC427, c427)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  # Binary search for boundary
  lo, hi = 0, clear_size
  while lo < hi:
    mid = (lo + hi) // 2
    v = read8(0xF000 + mid)
    if v == 0xFF:
      lo = mid + 1
    else:
      hi = mid
  
  # lo = first non-FF byte = DMA size
  write(0xC42A, 0x01)
  write(0xC403, 0x00)
  return lo

# ============================================================
print("="*60)
print("DMA size for C403=0x03, various C427")
print("="*60)

for c427 in range(0, 9):
  sz = measure_dma_size(0x03, c427)
  print(f"  C403=0x03 C427={c427}: {sz} bytes (0x{sz:X})")

# ============================================================
print("\n" + "="*60)
print("DMA size for various C403, C427=1")
print("="*60)

for c403 in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0F, 0x10, 0x1F, 0x20, 0x3F]:
  sz = measure_dma_size(c403, 1)
  print(f"  C403=0x{c403:02X} C427=1: {sz} bytes (0x{sz:X})")

# ============================================================
print("\n" + "="*60)
print("DMA size for various C403, C427=2")
print("="*60)

for c403 in [0x00, 0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x0F, 0x10, 0x3F]:
  sz = measure_dma_size(c403, 2)
  print(f"  C403=0x{c403:02X} C427=2: {sz} bytes (0x{sz:X})")

print("\nDone!")
