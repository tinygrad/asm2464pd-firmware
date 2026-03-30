#!/usr/bin/env python3
"""
Test clearing bits from the base state AND properly verify DMA freshness.
Each test:
  1. Clear F000 to 0x00
  2. Send FRESH unique data via bulk out
  3. Apply modified base state
  4. Trigger
  5. Check F000 == marker (proves fresh DMA, not stale)
  6. Scan F000-FFFF for marker (finds where it went)
"""
import sys, ctypes
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

test_counter = [0]

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0

# dma_explore base
BASE = {
  0xC420: 0x00, 0xC421: 0x01, 0xC422: 0x02, 0xC423: 0x00,
  0xC424: 0x00, 0xC425: 0x00, 0xC426: 0x00, 0xC427: 0x01,
  0xC428: 0x30, 0xC429: 0x00, 0xC42A: 0x00, 0xC42B: 0x00,
  0xC42C: 0x00, 0xC42D: 0x00,
  0xC400: 0x01, 0xC401: 0x01, 0xC402: 0x01, 0xC404: 0x01,
}

def test(label, overrides):
  """Apply BASE with overrides, trigger, check where unique marker landed."""
  test_counter[0] += 1
  marker = (test_counter[0] * 7 + 0x31) & 0xFF
  if marker in (0x00, 0x55): marker = 0x77
  
  # Fresh bulk out
  dev._bulk_out(0x02, bytes([marker]*32))
  
  # Clear scan points  
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)
  
  # Apply base with overrides
  state = dict(BASE)
  state.update(overrides)
  
  # Write ALL registers (including zeros for ones not in state)
  for addr in range(0xC400, 0xC420):
    if addr == 0xC406: continue  # trigger last
    val = state.get(addr, 0x00)
    write(addr, val)
  
  # Write descriptor
  for addr in range(0xC420, 0xC42E):
    val = state.get(addr, 0x00)
    write(addr, val)
  
  # Trigger
  write(0xC406, 0x01)
  
  # Scan
  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(off)
  
  f0 = read8(0xF000)
  fired = len(landed) > 0
  
  write(0xC42A, 0x01)
  
  locs = ', '.join(f'F{off:03X}' for off in landed)
  flag = ""
  if not fired: flag = " NO_DMA"
  elif landed[0] != 0: flag = " ** MOVED **"
  elif len(landed) > 1: flag = f" ({len(landed)} hits)"
  
  print(f"  {label:45s}: {locs if locs else 'nowhere'}{flag}")
  return fired, landed

# ============================================================
print("="*60)
print("Verify base works")
print("="*60)
test("full base (control)", {})

# ============================================================
print("\n" + "="*60)
print("Remove each base register one at a time")
print("="*60)
for reg in sorted(BASE.keys()):
  if reg == 0xC42A: continue  # doorbell must be 0
  test(f"remove 0x{reg:04X} (was 0x{BASE[reg]:02X})", {reg: 0x00})

# ============================================================
print("\n" + "="*60)
print("Set extra bits (not in base)")
print("="*60)

# These are all writable regs NOT in the base
for reg, vals in [
  (0xC403, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC405, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC408, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC409, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC40C, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC40D, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC412, [0x01, 0x02, 0x03]),
  (0xC413, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]),
  (0xC414, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]),
  (0xC415, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]),
  (0xC416, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC417, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC418, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
  (0xC419, [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]),
]:
  for val in vals:
    try:
      fired, landed = test(f"0x{reg:04X}=0x{val:02X}", {reg: val})
      if not fired or (landed and landed[0] != 0):
        break  # interesting, stop testing this reg
    except:
      print(f"  0x{reg:04X}=0x{val:02X}: CRASH")
      break

print("\nDone!")
