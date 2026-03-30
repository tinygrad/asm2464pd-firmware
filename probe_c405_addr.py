#!/usr/bin/env python3
"""
C403 = DMA size. C405 = DMA address?
Test C405 as page/offset selector.
Also test C402, C404, C408, C409, C40C, C40D as potential address regs.
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

def test(label, pre_writes):
  """Send AA, set regs, DMA, scan F000-FFFF for AA."""
  dev._bulk_out(0x02, bytes([0xAA]*32))
  
  # Clear every 0x100 in the window
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x55)
  
  for addr, val in pre_writes:
    write(addr, val)
  
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  found = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == 0xAA:
      found.append(f"F{off:03X}")
  
  write(0xC42A, 0x01)
  for addr, val in pre_writes:
    write(addr, 0x00)
  
  loc = ', '.join(found) if found else 'NOWHERE'
  moved = found and found[0] != 'F000'
  print(f"  {label:40s}: {loc} {'** MOVED **' if moved else ''}")
  return moved

# ============================================================
print("="*60)
print("Baseline")
print("="*60)
test("no extra regs", [])

# ============================================================
print("\n" + "="*60)
print("C405 sweep (potential address register)")
print("="*60)
for val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]:
  test(f"C405=0x{val:02X}", [(0xC405, val)])

# C405 with C403 (maybe both needed)
print("\n  C405 + C403:")
for c405 in [0x01, 0x02, 0x04, 0x08, 0x10]:
  for c403 in [0x01, 0x02, 0x04]:
    test(f"C403=0x{c403:02X} C405=0x{c405:02X}", [(0xC403, c403), (0xC405, c405)])

# ============================================================
print("\n" + "="*60)
print("C402 sweep (8-bit writable)")
print("="*60)
for val in [0x01, 0x02, 0x04, 0x08, 0x10]:
  test(f"C402=0x{val:02X}", [(0xC402, val)])

# C402 with C403
for c402 in [0x01, 0x02, 0x04]:
  test(f"C403=0x04 C402=0x{c402:02X}", [(0xC403, 0x04), (0xC402, c402)])

# ============================================================
print("\n" + "="*60)
print("C404 sweep (8-bit writable, stock writes 2)")
print("="*60)
for val in [0x01, 0x02, 0x04, 0x08, 0x10]:
  test(f"C404=0x{val:02X}", [(0xC404, val)])

for c404 in [0x01, 0x02, 0x04]:
  test(f"C403=0x04 C404=0x{c404:02X}", [(0xC403, 0x04), (0xC404, c404)])

# ============================================================
print("\n" + "="*60)
print("C408/C409 pair (8-bit each)")
print("="*60)
for val in [0x01, 0x02, 0x04]:
  test(f"C408=0x{val:02X}", [(0xC408, val)])
  test(f"C409=0x{val:02X}", [(0xC409, val)])
  test(f"C408=0x{val:02X} C409=0x{val:02X}", [(0xC408, val), (0xC409, val)])

# ============================================================
print("\n" + "="*60)
print("C40C/C40D pair (8-bit each)")
print("="*60)
for val in [0x01, 0x02, 0x04]:
  test(f"C40C=0x{val:02X}", [(0xC40C, val)])
  test(f"C40D=0x{val:02X}", [(0xC40D, val)])

# ============================================================
print("\n" + "="*60)
print("All R/W combos with C403")
print("="*60)
# C402, C404, C405, C408, C409, C40C, C40D all 8-bit
# Try each as address with C403=4 as size
for reg in [0xC402, 0xC404, 0xC405, 0xC408, 0xC409, 0xC40C, 0xC40D]:
  moved = test(f"C403=4 0x{reg:04X}=0x02", [(0xC403, 0x04), (reg, 0x02)])
  if moved:
    # Found it! Test more values
    for val in [0x01, 0x03, 0x04, 0x08, 0x10]:
      test(f"  C403=4 0x{reg:04X}=0x{val:02X}", [(0xC403, 0x04), (reg, val)])

print("\nDone!")
