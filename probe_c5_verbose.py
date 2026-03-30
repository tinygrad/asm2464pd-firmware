#!/usr/bin/env python3
"""
Verbose: show EVERY C5xx test result — did DMA fire? Where did it go?
Only test a few values per register to keep output manageable.
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

test_n = [0]

def test(reg, val):
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x23) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  
  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)
  
  # Set the reg BEFORE the descriptor
  write(reg, val)
  
  # Full trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  
  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")
  
  f0 = read8(0xF000)
  fired = f0 == marker or len(landed) > 0
  
  write(0xC42A, 0x01)
  write(reg, 0x00)
  
  return fired, landed

# Only test regs that stock has non-zero
STOCK_C5 = {
  0xC50C: 0x11,
  0xC516: 0x01, 0xC517: 0x01,
  0xC518: 0x08, 0xC519: 0x08,
  0xC51E: 0x01, 0xC51F: 0x01,
  0xC523: 0x01,
  0xC530: 0xE0, 0xC531: 0xE1, 0xC533: 0x01, 0xC534: 0x80,
  0xC536: 0x0C, 0xC537: 0x08, 0xC539: 0x1F,
  0xC542: 0x01,
}

print("Stock C5xx regs with stock values:")
for reg, val in sorted(STOCK_C5.items()):
  try:
    fired, landed = test(reg, val)
    locs = ', '.join(landed) if landed else 'nowhere'
    flag = " MOVED" if landed and landed[0] != 'F000' else (" NO_DMA" if not fired else "")
    print(f"  0x{reg:04X}=0x{val:02X}: {locs}{flag}")
  except:
    print(f"  0x{reg:04X}=0x{val:02X}: CRASH")
    break

# Also test C509 (documented as Transfer Control)
print("\nC509 (Transfer Control):")
for val in [0x01, 0x02, 0x04, 0x08]:
  fired, landed = test(0xC509, val)
  locs = ', '.join(landed) if landed else 'nowhere'
  print(f"  C509=0x{val:02X}: {locs}")

print("\nDone!")
