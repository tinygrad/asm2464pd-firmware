#!/usr/bin/env python3
"""
CE10 was stuck at 0x220000 and dma_explore FAILED (data not at F000).
This proves CE10 IS the address for C401=1 DMA.

Reset CE10, verify dma_explore works, then systematically test.
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

# Step 1: Check current CE10
ce10 = bytes([read8(0xCE10+i) for i in range(4)])
print(f"CE10 current: {ce10.hex()}")

# Step 2: Reset CE10 to 0x200000
set_ce10(0x00200000)
ce10 = bytes([read8(0xCE10+i) for i in range(4)])
print(f"CE10 after reset: {ce10.hex()}")

# Step 3: dma_explore style trigger — does it work now?
dev._bulk_out(0x02, bytes([0xEE]*32))
write(0xF000, 0x00)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
f0 = read8(0xF000)
print(f"F000 after C401=1 DMA: 0x{f0:02X} (expect EE) {'OK' if f0==0xEE else 'FAIL'}")
write(0xC42A, 0x01)

# Step 4: Set CE10 to 0x200400, C401=1, trigger
print(f"\n--- CE10=0x200400, C401=1 ---")
set_ce10(0x00200400)
dev._bulk_out(0x02, bytes([0xDD]*32))
write(0xF000, 0x00); write(0xF400, 0x00)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
f0 = read8(0xF000)
f4 = read8(0xF400)
print(f"F000=0x{f0:02X} F400=0x{f4:02X}")
print(f"Data went to: {'F000' if f0==0xDD else 'F400' if f4==0xDD else 'NEITHER'}")
write(0xC42A, 0x01)

# Step 5: Check CE10 after DMA
ce10 = bytes([read8(0xCE10+i) for i in range(4)])
print(f"CE10 after DMA: {ce10.hex()}")

# Step 6: Sequential test - set CE10, DMA, check
print(f"\n--- Sequential DMA with CE10 control ---")
for off in range(0, 0x1000, 0x100):
  write(0xF000 + off, 0x00)

for i, (pci, name) in enumerate([
  (0x200000, "F000"), (0x200100, "F100"), (0x200200, "F200"),
  (0x200400, "F400"), (0x200800, "F800")
]):
  marker = 0xA0 + i
  set_ce10(pci)
  dev._bulk_out(0x02, bytes([marker]*32))
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  write(0xC42A, 0x01)
  d = read8(0xF000 + (pci - 0x200000))
  print(f"  CE10=0x{pci:06X} -> {name}=0x{d:02X} (expect 0x{marker:02X}) {'OK' if d==marker else 'FAIL'}")

# Check all locations
print(f"\nFull SRAM scan:")
for off in range(0, 0x1000, 0x100):
  d = read8(0xF000 + off)
  if d != 0x00:
    print(f"  F{off:03X}: 0x{d:02X}")

set_ce10(0x00200000)
print("\nDone!")
