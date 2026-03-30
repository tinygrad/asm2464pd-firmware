#!/usr/bin/env python3
"""
C401=2 — earlier we found C401 bit 1 KILLS DMA. Maybe it switches
to the real DMA engine that uses CE10 as address.
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
marker = 0

def prep():
  global marker
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)

def set_ce10(pci):
  write(0xCE10, (pci >> 24) & 0xFF)
  write(0xCE11, (pci >> 16) & 0xFF)
  write(0xCE12, (pci >> 8) & 0xFF)
  write(0xCE13, pci & 0xFF)

def scan():
  landed = []
  for off in range(0, 0x1000, 0x100):
    if read8(0xF000 + off) == marker:
      landed.append(f"F{off:03X}")
  return landed

def report(label):
  landed = scan()
  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:55s}: {locs}{flag}")

# ============================================================
print("="*60)
print("C401=2 + CE10 targeting")
print("="*60)

# C401=1 is normal (dma_explore uses it). C401=2 killed DMA before.
# Maybe C401=2 switches to CE10-addressed mode?

prep(); set_ce10(0x00200400)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 2); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=2, CE10=0x200400")

prep(); set_ce10(0x00200000)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 2); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=2, CE10=0x200000 (default)")

prep(); set_ce10(0x00200400)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 3); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=3, CE10=0x200400")

# ============================================================
print("\n" + "="*60)
print("C401=2 + C403 + CE10")
print("="*60)

for c401 in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
  prep(); set_ce10(0x00200400)
  write(0xC403, 0x04)
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, c401); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  write(0xC42A, 0x01)
  write(0xC403, 0x00)
  report(f"C401=0x{c401:02X} + C403=4, CE10=0x200400")

# ============================================================
print("\n" + "="*60)
print("C401=2 only (no other arms)")
print("="*60)

prep(); set_ce10(0x00200400)
write(0xC42A, 0x00)
write(0xC401, 2)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=2 only, CE10=0x200400")

prep(); set_ce10(0x00200000)
write(0xC42A, 0x00)
write(0xC401, 2)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=2 only, CE10=0x200000")

# ============================================================
print("\n" + "="*60)
print("C401=2 + CE76 (LE target addr)")
print("="*60)

prep()
write(0xCE76, 0x00); write(0xCE77, 0x04); write(0xCE78, 0x20); write(0xCE79, 0x00)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 2); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
write(0xC42A, 0x01)
report("C401=2, CE76=0x200400")

# ============================================================
print("\n" + "="*60)
print("Try C401=2 with C403 sweep")
print("="*60)

for c403 in [0x00, 0x01, 0x02, 0x04]:
  prep(); set_ce10(0x00200400)
  write(0xC403, c403)
  write(0xC42A, 0x00)
  write(0xC401, 2)
  write(0xC406, 1)
  write(0xC42A, 0x01)
  write(0xC403, 0x00)
  report(f"C401=2 only + C403=0x{c403:02X}, CE10=0x200400")

set_ce10(0x00200000)
print("\nDone!")
