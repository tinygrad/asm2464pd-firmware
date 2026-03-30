#!/usr/bin/env python3
"""
Test every writable bit in C400-C419 WITH full known-good base state.
Base = dma_explore descriptor + arm bits.
For each bit: set base + one extra bit, trigger, scan.
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

# Known-good base state from dma_explore
BASE = {
  0xC420: 0x00, 0xC421: 0x01, 0xC422: 0x02, 0xC423: 0x00,
  0xC424: 0x00, 0xC425: 0x00, 0xC426: 0x00, 0xC427: 0x01,
  0xC428: 0x30, 0xC429: 0x00, 0xC42A: 0x00, 0xC42B: 0x00,
  0xC42C: 0x00, 0xC42D: 0x00,
  0xC400: 0x01, 0xC401: 0x01, 0xC402: 0x01, 0xC404: 0x01,
}

MASKS = {
  0xC400: 0x01, 0xC401: 0x07, 0xC402: 0xFF, 0xC403: 0xFF,
  0xC404: 0xFF, 0xC405: 0xFF,
  0xC408: 0xFF, 0xC409: 0xFF,
  0xC40C: 0xFF, 0xC40D: 0xFF,
  0xC412: 0x03, 0xC413: 0x3F, 0xC414: 0xBF, 0xC415: 0x3F,
  0xC416: 0xFF, 0xC417: 0xFF, 0xC418: 0xFF, 0xC419: 0xFF,
}

def apply_base():
  for addr, val in BASE.items():
    write(addr, val)

def test_bit(reg, bit):
  val = 1 << bit
  marker = (reg & 0xFF) ^ bit ^ 0x77
  
  dev._bulk_out(0x02, bytes([marker]*32))
  
  # Clear scan points
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)
  
  # Apply full base state
  apply_base()
  
  # Set the extra bit (OR with base if reg is in base)
  base_val = BASE.get(reg, 0x00)
  write(reg, base_val | val)
  
  # Trigger (C42A already 0x00 from base)
  write(0xC406, 0x01)
  
  # Scan
  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(off)
  
  f0 = read8(0xF000)
  dma_fired = len(landed) > 0
  
  write(0xC42A, 0x01)  # unlock
  
  return marker, f0, landed, dma_fired

# First verify base works
print("Verifying base state works...")
dev._bulk_out(0x02, bytes([0xEE]*32))
write(0xF000, 0x00)
apply_base()
write(0xC406, 0x01)
f0 = read8(0xF000)
print(f"  F000=0x{f0:02X} ({'OK' if f0==0xEE else 'FAIL'})")
write(0xC42A, 0x01)

print("\n" + "="*60)
print("Every writable bit with known-good base")
print("="*60)

for reg in sorted(MASKS.keys()):
  mask = MASKS[reg]
  for bit in range(8):
    if not (mask & (1 << bit)):
      continue
    # Skip bits that are already set in base
    base_val = BASE.get(reg, 0x00)
    if base_val & (1 << bit):
      continue
    try:
      marker, f0, landed, fired = test_bit(reg, bit)
      locs = ', '.join(f'F{off:03X}' for off in landed)
      
      flag = ""
      if not fired:
        flag = " NO_DMA"
      elif landed and landed[0] != 0:
        flag = " ** MOVED **"
      elif len(landed) != 1:
        flag = f" ({len(landed)} hits)"
      
      print(f"  0x{reg:04X} bit {bit} (=0x{1<<bit:02X}): {locs if locs else f'F000=0x{f0:02X}'}{flag}")
    except Exception as e:
      print(f"  0x{reg:04X} bit {bit} (=0x{1<<bit:02X}): CRASH ({e})")
      # Re-prime after crash
      try:
        write(0xC42A, 0x01)
      except:
        pass
      break

print("\nDone!")
