#!/usr/bin/env python3
"""
Properly test ALL stock C5xx values on handmade firmware.
Check exact DMA byte count with binary search.
Also check what C509 does (Transfer Control in registers.h).
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

# Stock C5xx constants (always set on stock firmware after any write)
STOCK_C5_CONST = {
  0xC50C: 0x11, 0xC523: 0x01,
  0xC530: 0xE0, 0xC531: 0xE1, 0xC533: 0x01, 0xC534: 0x80,
  0xC536: 0x0C, 0xC537: 0x08, 0xC539: 0x1F, 0xC542: 0x01,
}

# Stock C5xx values for 4KB write
STOCK_C5_4KB = {
  **STOCK_C5_CONST,
  0xC516: 0x01, 0xC517: 0x01,
  0xC518: 0x08, 0xC519: 0x08,
  0xC51E: 0x01, 0xC51F: 0x01,
}

def measure_dma(label, pre_writes):
  """Send 1KB of FF, clear SRAM, set regs, trigger, binary search for size."""
  dev._bulk_out(0x02, bytes([0xFF]*1024))
  
  # Clear
  for off in range(0, 0x1000, 0x10):
    write(0xF000 + off, 0x00)
  
  # Apply pre-writes
  for addr, val in pre_writes.items():
    write(addr, val)
  
  # Full trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x08); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  
  # Binary search for DMA boundary
  lo, hi = 0, 0x1000
  while lo < hi:
    mid = (lo + hi) // 2
    v = read8(0xF000 + mid)
    if v == 0xFF:
      lo = mid + 1
    else:
      hi = mid
  
  write(0xC42A, 0x01)
  
  # Clean up
  for addr in pre_writes:
    write(addr, 0x00)
  
  print(f"  {label:50s}: {lo} bytes (0x{lo:X})")
  return lo

# ============================================================
print("="*60)
print("Baseline")
print("="*60)
measure_dma("no extra regs", {})
measure_dma("C403=0x04 only", {0xC403: 0x04})

# ============================================================
print("\n" + "="*60)
print("Stock C5xx constants only (no C403)")
print("="*60)
measure_dma("stock C5xx constants", STOCK_C5_CONST)

# ============================================================
print("\n" + "="*60)
print("Stock C5xx 4KB values (no C403)")
print("="*60)
measure_dma("stock C5xx 4KB values", STOCK_C5_4KB)

# ============================================================
print("\n" + "="*60)
print("Individual C5xx regs (no C403)")
print("="*60)
for addr, val in sorted(STOCK_C5_4KB.items()):
  measure_dma(f"0x{addr:04X}=0x{val:02X}", {addr: val})

# ============================================================
print("\n" + "="*60)
print("C5xx constants + C403")
print("="*60)
measure_dma("stock C5xx const + C403=4", {**STOCK_C5_CONST, 0xC403: 0x04})
measure_dma("stock C5xx 4KB + C403=4", {**STOCK_C5_4KB, 0xC403: 0x04})
measure_dma("stock C5xx 4KB + C403=8", {**STOCK_C5_4KB, 0xC403: 0x08})
measure_dma("stock C5xx 4KB + C403=0x10", {**STOCK_C5_4KB, 0xC403: 0x10})

# ============================================================
print("\n" + "="*60)
print("C509 (Transfer Control from registers.h)")
print("="*60)
for val in [0x01, 0x02, 0x04, 0x08, 0x10]:
  measure_dma(f"C509=0x{val:02X}", {0xC509: val})
  measure_dma(f"C509=0x{val:02X} + C403=4", {0xC509: val, 0xC403: 0x04})

print("\nDone!")
