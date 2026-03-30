#!/usr/bin/env python3
"""
Part 2: C480-C4FF and CE00-CEFF writability scan.
Skip all dangerous registers including C47C/C47E which crashed before.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

SKIP = {
  0xC400, 0xC401, 0xC402, 0xC404, 0xC406,
  0xC42A, 0xC42C, 0xC42D,
  0xC47C, 0xC47E,  # crashed last time
  0xCE00, 0xCE88,
  0xCE3D, 0xCE3F,  # corrupted DMA state last time
}

print("="*60)
print("C480-C4FF writability scan")
print("="*60)

for addr in range(0xC480, 0xC500):
  if addr in SKIP:
    continue
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")

print("\n" + "="*60)
print("CE00-CE7F writability scan")
print("="*60)

for addr in range(0xCE00, 0xCE80):
  if addr in SKIP:
    continue
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")

print("\n" + "="*60)
print("CE80-CEFF writability scan")
print("="*60)

for addr in range(0xCE80, 0xCF00):
  if addr in SKIP:
    continue
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")

# Now test CE10 writability again
print("\n" + "="*60)
print("CE10-CE13 write test (careful)")
print("="*60)

for addr in [0xCE10, 0xCE11, 0xCE12, 0xCE13]:
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> {rw}")

# Verify
print("\n" + "="*60)
print("Verify scsi_write")
print("="*60)
import random
test = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test, lba=0)
after = ctrl.read(0xF000, 16)
print(f"  MATCH: {after == test[:16]}")

print("\nDone!")
