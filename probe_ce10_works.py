#!/usr/bin/env python3
"""
CE10 IS the DMA target address! Verify by writing to specific offsets.
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

def set_ce10(pci):
  write(0xCE10, (pci >> 24) & 0xFF)
  write(0xCE11, (pci >> 16) & 0xFF)
  write(0xCE12, (pci >> 8) & 0xFF)
  write(0xCE13, pci & 0xFF)

def dma_trigger():
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  write(0xC42A, 0x01)

# Clear SRAM
for off in range(0, 0x1000, 0x100):
  write(0xF000 + off, 0x00)

# ============================================================
print("="*60)
print("CE10 = DMA target address (verified!)")
print("="*60)

targets = [
  (0x00200000, "F000"),
  (0x00200100, "F100"),
  (0x00200200, "F200"),
  (0x00200400, "F400"),
  (0x00200800, "F800"),
]

for pci, name in targets:
  marker = (pci >> 8) & 0xFF | 0xA0
  dev._bulk_out(0x02, bytes([marker]*32))
  set_ce10(pci)
  dma_trigger()
  d = read8(0xF000 + (pci - 0x200000))
  print(f"  CE10=0x{pci:08X} ({name}): {name}=0x{d:02X} (expect 0x{marker:02X}) {'OK' if d==marker else 'FAIL'}")

# ============================================================
print("\n" + "="*60)
print("Sequential: fill F000, F200, F400 with different data")
print("="*60)

for off in range(0, 0x1000, 0x100):
  write(0xF000 + off, 0x00)

for i, pci in enumerate([0x200000, 0x200200, 0x200400, 0x200600, 0x200800]):
  marker = 0xA0 + i
  dev._bulk_out(0x02, bytes([marker]*32))
  set_ce10(pci)
  dma_trigger()

print("SRAM after 5 sequential DMAs:")
for off in range(0, 0xA00, 0x200):
  d = read8(0xF000 + off)
  expected = 0xA0 + (off // 0x200)
  ok = d == expected
  print(f"  F{off:03X}: 0x{d:02X} (expect 0x{expected:02X}) {'OK' if ok else 'FAIL'}")

# Reset CE10
set_ce10(0x00200000)
print("\nDone!")
