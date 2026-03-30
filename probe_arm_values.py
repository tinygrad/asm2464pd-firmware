#!/usr/bin/env python3
"""
Explore all C400-C406 values for multi-sector DMA.
Stock writes C404=2 (not 1). Maybe other values matter.
Also try C405 which is R/W but untested.

Send 1KB, use C403=4 for size, try various arm combos.
Focus on getting F400 to have data (proof of >1KB DMA).
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
  return bytes([read8(addr + i) for i in range(n)])

def test(label, pre_writes, c403=0x04):
  """Send 2KB, configure, trigger, check F000-F800."""
  data = bytes([0xAA]*512 + [0xBB]*512 + [0xCC]*512 + [0xDD]*512)
  dev._bulk_out(0x02, data)
  
  # Clear targets
  for off in range(0, 0xA00, 0x200):
    write(0xF000 + off, 0x00)
  
  # Pre-writes (the registers we're testing)
  for addr, val in pre_writes:
    write(addr, val)
  
  write(0xC403, c403)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  results = []
  for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
    d = read8(0xF000 + off)
    results.append(f"F{off:03X}={d:02X}")
  
  # Check if we got past 1KB
  f400 = read8(0xF400)
  marker = "** >1KB **" if f400 in [0xCC, 0xBB, 0xAA] else ""
  print(f"  {label:30s}: {' '.join(results)} {marker}")
  
  write(0xC42A, 0x01)
  # Clean up pre_writes
  for addr, val in pre_writes:
    write(addr, 0x00)
  write(0xC403, 0x00)

# ============================================================
print("="*60)
print("C403 values (bigger than 4)")
print("="*60)

for c403 in [0x04, 0x05, 0x06, 0x07, 0x08, 0x10, 0x20, 0x40]:
  test(f"C403=0x{c403:02X}", [], c403=c403)

# ============================================================
print("\n" + "="*60)
print("C405 values with C403=4")
print("="*60)

for c405 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0xFF]:
  test(f"C405=0x{c405:02X}", [(0xC405, c405)])

# ============================================================
print("\n" + "="*60)
print("C404 values (stock writes 2)")
print("="*60)

for c404 in [0x01, 0x02, 0x03, 0x04, 0x08]:
  test(f"C404=0x{c404:02X}", [(0xC404, c404)])

# ============================================================
print("\n" + "="*60)
print("C400 values")
print("="*60)

for c400 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10]:
  test(f"C400=0x{c400:02X}", [(0xC400, c400)])

# ============================================================
print("\n" + "="*60)
print("C401 values")
print("="*60)

for c401 in [0x01, 0x02, 0x03, 0x04, 0x08]:
  test(f"C401=0x{c401:02X}", [(0xC401, c401)])

# ============================================================
print("\n" + "="*60)
print("C402 values")
print("="*60)

for c402 in [0x01, 0x02, 0x03, 0x04, 0x08]:
  test(f"C402=0x{c402:02X}", [(0xC402, c402)])

# ============================================================
print("\n" + "="*60)
print("C408/C409/C40C/C40D values")
print("="*60)

for reg in [0xC408, 0xC409, 0xC40C, 0xC40D]:
  for val in [0x01, 0x02, 0x04, 0x08]:
    test(f"0x{reg:04X}=0x{val:02X}", [(reg, val)])

# ============================================================
print("\n" + "="*60)
print("Combos: stock-like arm values")
print("="*60)

# Stock sets: C400=1, C401=1, C402=1, C404=2
test("stock-like arms", [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2)])
test("stock + C405=1", [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2),(0xC405,1)])
test("stock + C403=0x08", [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2)], c403=0x08)
test("stock + C403=0x10", [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2)], c403=0x10)
test("stock + C403=0xFF", [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2)], c403=0xFF)

print("\nDone!")
