#!/usr/bin/env python3
"""
CE10-CE13 is writable on handmade firmware!
Set it to PCI 0x200400, trigger C400 DMA, check F400.
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

test_n = [0]

def test(label, pci_addr):
  """Set CE10-13 to pci_addr (BE), trigger DMA, scan for data."""
  test_n[0] += 1
  marker = (test_n[0] * 11 + 0x37) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  # Set CE10-13 (big-endian PCI address)
  write(0xCE10, (pci_addr >> 24) & 0xFF)
  write(0xCE11, (pci_addr >> 16) & 0xFF)
  write(0xCE12, (pci_addr >> 8) & 0xFF)
  write(0xCE13, (pci_addr >> 0) & 0xFF)

  # Verify it stuck
  ce10 = read_range(0xCE10, 4)

  # Full trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)

  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")

  write(0xC42A, 0x01)

  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:40s} CE10={ce10.hex()}: {locs}{flag}")

# ============================================================
print("="*60)
print("CE10-CE13 as DMA target address")
print("="*60)

test("PCI 0x200000 (default)", 0x00200000)
test("PCI 0x200100 (F100)", 0x00200100)
test("PCI 0x200200 (F200)", 0x00200200)
test("PCI 0x200400 (F400)", 0x00200400)
test("PCI 0x200800 (F800)", 0x00200800)
test("PCI 0x200F00 (FF00)", 0x00200F00)

# ============================================================
print("\n" + "="*60)
print("Sequential: write to F000, then F200, then F400")
print("="*60)

for i, target in enumerate([(0x00200000, "F000"), (0x00200200, "F200"), (0x00200400, "F400")]):
  pci, name = target
  marker = 0xA0 + i
  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    if i == 0: write(0xF000 + off, 0x00)  # only clear first time

  write(0xCE10, (pci >> 24) & 0xFF)
  write(0xCE11, (pci >> 16) & 0xFF)
  write(0xCE12, (pci >> 8) & 0xFF)
  write(0xCE13, (pci >> 0) & 0xFF)

  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  write(0xC42A, 0x01)

print("After 3 writes:")
for off in [0x000, 0x200, 0x400, 0x600]:
  d = read8(0xF000 + off)
  expected = 0xA0 + (off // 0x200) if off < 0x600 else 0
  ok = d == expected
  print(f"  F{off:03X}: 0x{d:02X} (expect 0x{expected:02X}) {'OK' if ok else ''}")

print("\nDone!")
