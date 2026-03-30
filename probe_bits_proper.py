#!/usr/bin/env python3
"""
For every writable bit in C400-C41F: set it, send fresh data, DMA, 
check F000 AND scan for where data actually went.
Properly verify DMA fired by checking F000 changed from cleared state.
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

# Writable masks from our probe
MASKS = {
  0xC400: 0x01, 0xC401: 0x07, 0xC402: 0xFF, 0xC403: 0xFF,
  0xC404: 0xFF, 0xC405: 0xFF,
  # C406 = trigger, skip
  0xC408: 0xFF, 0xC409: 0xFF,
  0xC40C: 0xFF, 0xC40D: 0xFF,
  0xC412: 0x03, 0xC413: 0x3F, 0xC414: 0xBF, 0xC415: 0x3F,
  0xC416: 0xFF, 0xC417: 0xFF, 0xC418: 0xFF, 0xC419: 0xFF,
}

def test_bit(reg, bit):
  """Set one bit, send unique data, trigger DMA, report what changed."""
  val = 1 << bit
  
  # Send unique data
  marker = (reg & 0xFF) ^ bit ^ 0x77
  dev._bulk_out(0x02, bytes([marker]*32))
  
  # Clear F000-FFFF scan points
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)
  
  # Set the bit
  write(reg, val)
  
  # Trigger
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  # Scan where marker landed
  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(off)
  
  # Also check F000 specifically
  f0 = read8(0xF000)
  dma_fired = f0 == marker or len(landed) > 0
  
  write(0xC42A, 0x01)
  write(reg, 0x00)
  
  return marker, f0, landed, dma_fired

print("="*60)
print("Every writable bit in C400-C419")
print("Format: reg.bit -> DMA? where?")
print("="*60)

for reg in sorted(MASKS.keys()):
  mask = MASKS[reg]
  for bit in range(8):
    if not (mask & (1 << bit)):
      continue
    try:
      marker, f0, landed, fired = test_bit(reg, bit)
      locs = ', '.join(f'F{off:03X}' for off in landed)
      if not locs:
        locs = f'F000=0x{f0:02X}(expect 0x{marker:02X})'
      
      # Highlight anything interesting
      flag = ""
      if not fired:
        flag = " NO_DMA"
      elif landed and landed[0] != 0:
        flag = " ** MOVED **"
      elif len(landed) > 4:
        flag = f" ({len(landed)} hits)"
      
      print(f"  0x{reg:04X} bit {bit} (=0x{1<<bit:02X}): {locs}{flag}")
    except:
      print(f"  0x{reg:04X} bit {bit} (=0x{1<<bit:02X}): CRASH")
      break

print("\nDone!")
