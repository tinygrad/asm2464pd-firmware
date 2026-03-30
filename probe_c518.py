#!/usr/bin/env python3
"""
Test C518/C519 as DMA size control on handmade firmware.
Stock: C518=C519=size/256. 
Also set the constant C5xx regs from stock.
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

def read_range(addr, n):
  return bytes([read8(addr+i) for i in range(n)])

# Stock constant C5xx values
STOCK_C5 = {
  0xC50C: 0x11, 0xC523: 0x01,
  0xC530: 0xE0, 0xC531: 0xE1, 0xC533: 0x01, 0xC534: 0x80,
  0xC536: 0x0C, 0xC537: 0x08, 0xC539: 0x1F, 0xC542: 0x01,
}

def test(label, c518_val, extra_writes=None):
  """Send 4KB, set C518/C519, trigger, check how far DMA wrote."""
  # 4KB with unique per-page markers
  data = b''
  for page in range(16):
    data += bytes([0x10 + page] * 256)
  dev._bulk_out(0x02, data)
  
  # Clear F000-FFFF
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)
  
  # Apply extra writes
  if extra_writes:
    for addr, val in extra_writes.items():
      write(addr, val)
  
  # Set C518/C519
  write(0xC518, c518_val)
  write(0xC519, c518_val)
  
  # Full dma_explore trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x08); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  
  # Scan
  results = []
  last_hit = -1
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    expected = 0x10 + (off >> 8)
    if d == expected:
      results.append(f"F{off:03X}=OK")
      last_hit = off
    elif d != 0x00:
      results.append(f"F{off:03X}={d:02X}")
      last_hit = off
  
  dma_bytes = last_hit + 0x100 if last_hit >= 0 else 0
  print(f"  {label:45s}: {dma_bytes:5d}B  {' '.join(results[:8])}")
  write(0xC42A, 0x01)
  
  # Clean up
  write(0xC518, 0x00); write(0xC519, 0x00)
  if extra_writes:
    for addr in extra_writes:
      write(addr, 0x00)

# ============================================================
print("="*60)
print("C518/C519 alone")
print("="*60)
for val in [0x02, 0x04, 0x06, 0x08, 0x10]:
  test(f"C518=C519=0x{val:02X}", val)

# ============================================================
print("\n" + "="*60)
print("C518/C519 + stock C5xx constants")
print("="*60)
for val in [0x02, 0x04, 0x08, 0x10]:
  test(f"C518=C519=0x{val:02X} + stock C5xx", val, STOCK_C5)

# ============================================================
print("\n" + "="*60)
print("C518/C519 + C403")
print("="*60)
for val in [0x04, 0x08]:
  test(f"C518=0x{val:02X} + C403=0x04", val, {0xC403: 0x04})

# ============================================================
print("\n" + "="*60)
print("All stock C5xx + C403")
print("="*60)
for c518 in [0x04, 0x08, 0x10]:
  combined = {**STOCK_C5, 0xC403: 0x04}
  test(f"all stock C5xx + C403=4, C518=0x{c518:02X}", c518, combined)

print("\nDone!")
