#!/usr/bin/env python3
"""
C416-C419 might be the address, but need an enable bit.
Try every C401 value (0-7) + C416-C419 address.
Also try C412-C415 as enables (they have writable bits).
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

def test(label, c416_pci, extra_arm={}, extra_pre={}):
  global marker
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  # Set C416-C419 as BE PCI address (like CE10)
  write(0xC416, (c416_pci >> 24) & 0xFF)
  write(0xC417, (c416_pci >> 16) & 0xFF)
  write(0xC418, (c416_pci >> 8) & 0xFF)
  write(0xC419, c416_pci & 0xFF)

  # Extra pre-writes
  for addr, val in extra_pre.items():
    write(addr, val)

  # Descriptor
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)

  # Arm with overrides
  c400 = extra_arm.get(0xC400, 1)
  c401 = extra_arm.get(0xC401, 1)
  c402 = extra_arm.get(0xC402, 1)
  c404 = extra_arm.get(0xC404, 1)
  write(0xC400, c400); write(0xC401, c401); write(0xC402, c402); write(0xC404, c404)
  write(0xC406, 1)

  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")

  write(0xC42A, 0x01)
  write(0xC416, 0); write(0xC417, 0); write(0xC418, 0); write(0xC419, 0)
  for addr in extra_pre: write(addr, 0x00)

  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:55s}: {locs}{flag}")

# addr = PCI 0x200400 = F400
ADDR = 0x00200400

# ============================================================
print("="*60)
print("C401 values 0-7 with C416=0x200400")
print("="*60)

for c401 in range(8):
  test(f"C401={c401}", ADDR, {0xC401: c401})

# ============================================================
print("\n" + "="*60)
print("C412 enable bits with C416=0x200400")
print("="*60)

for c412 in [0x01, 0x02, 0x03]:
  test(f"C412=0x{c412:02X}", ADDR, extra_pre={0xC412: c412})
  test(f"C412=0x{c412:02X} + C401=2", ADDR, {0xC401: 2}, {0xC412: c412})
  test(f"C412=0x{c412:02X} + C401=3", ADDR, {0xC401: 3}, {0xC412: c412})

# ============================================================
print("\n" + "="*60)
print("C413 enable bits with C416=0x200400")
print("="*60)

for c413 in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]:
  test(f"C413=0x{c413:02X}", ADDR, extra_pre={0xC413: c413})

# ============================================================
print("\n" + "="*60)
print("C414/C415 enable bits with C416=0x200400")
print("="*60)

for c414 in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x80]:
  test(f"C414=0x{c414:02X}", ADDR, extra_pre={0xC414: c414})

for c415 in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]:
  test(f"C415=0x{c415:02X}", ADDR, extra_pre={0xC415: c415})

# ============================================================
print("\n" + "="*60)
print("Try LE address in C416-C419 (maybe it's LE not BE)")
print("="*60)

# LE: C416=LSB C419=MSB -> PCI 0x00200400: C416=0x00 C417=0x04 C418=0x20 C419=0x00
test("LE 0x200400, C401=1", 0, extra_pre={0xC416: 0x00, 0xC417: 0x04, 0xC418: 0x20, 0xC419: 0x00})
test("LE 0x200400, C401=2", 0, {0xC401: 2}, {0xC416: 0x00, 0xC417: 0x04, 0xC418: 0x20, 0xC419: 0x00})
test("LE 0x200400, C401=3", 0, {0xC401: 3}, {0xC416: 0x00, 0xC417: 0x04, 0xC418: 0x20, 0xC419: 0x00})
test("LE 0x200400, C401=5", 0, {0xC401: 5}, {0xC416: 0x00, 0xC417: 0x04, 0xC418: 0x20, 0xC419: 0x00})

# ============================================================
print("\n" + "="*60)
print("C402 as enable (8 bits writable)")
print("="*60)

for c402 in [0x02, 0x04, 0x08, 0x10, 0x20]:
  test(f"C402=0x{c402:02X}", ADDR, {0xC402: c402})

print("\nDone!")
