#!/usr/bin/env python3
"""
CECD changed between 1KB (0x02) and 4KB (0x08) writes = sector count.
CFCD mirrors it. Test if writing CECD before DMA trigger changes behavior.
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
  test_n[0] += 1
  marker = (test_n[0] * 13 + 0x41) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  for addr, val in pre_writes.items():
    write(addr, val)

  # Full trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x08); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)

  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")

  write(0xC42A, 0x01)
  for addr in pre_writes:
    write(addr, 0x00)

  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:50s}: {locs}{flag}")

test_n = [0]

# ============================================================
print("="*60)
print("CECD as DMA control")
print("="*60)

test("baseline", {})
test("CECD=0x01", {0xCECD: 0x01})
test("CECD=0x02", {0xCECD: 0x02})
test("CECD=0x04", {0xCECD: 0x04})
test("CECD=0x08", {0xCECD: 0x08})
test("CECD=0x10", {0xCECD: 0x10})
test("CECD=0x20", {0xCECD: 0x20})

# CECD + CE76 address
test("CECD=0x08 + CE76=0x200400", {0xCECD: 0x08, 0xCE76: 0x00, 0xCE77: 0x04, 0xCE78: 0x20, 0xCE79: 0x00})

# CFCD (mirror)
test("CFCD=0x08", {0xCFCD: 0x08})
test("CECD=0x08 + CFCD=0x08", {0xCECD: 0x08, 0xCFCD: 0x08})

# CECD + C403
test("CECD=0x08 + C403=0x04", {0xCECD: 0x08, 0xC403: 0x04})

# CECD + CEC0-CEC4 (stock values from dump)
test("CECD=8 + CEC0-4 stock", {0xCECD: 0x08, 0xCEC0: 0xE4, 0xCEC1: 0x01, 0xCEC2: 0x50, 0xCEC3: 0xCE, 0xCEC4: 0xC4})

# Read CECD after each test to see if it sticks
print(f"\n  CECD readback after write 0x08: ", end="")
write(0xCECD, 0x08)
print(f"0x{read8(0xCECD):02X}")

# CED0-CEDF area (near CECD)
print("\n  CEC0-CEDF dump:")
for addr in range(0xCEC0, 0xCEE0):
  v = read8(addr)
  if v: print(f"    0x{addr:04X} = 0x{v:02X}")

write(0xCECD, 0x00)

print("\nDone!")
