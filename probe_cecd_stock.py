#!/usr/bin/env python3
"""
On stock firmware: check CECD/CFCD writability and alias test.
Also: is C4xx really mirrored at C5xx? Check by writing to C500 area
and reading C400 area.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# CECD writable?
print("CECD writability on stock:")
orig = r(0xCECD)
w(0xCECD, 0xFF)
rb = r(0xCECD)
w(0xCECD, orig)
print(f"  orig=0x{orig:02X} wrote=0xFF read=0x{rb:02X}")

# CFCD
orig = r(0xCFCD)
w(0xCFCD, 0xFF)
rb = r(0xCFCD)
w(0xCFCD, orig)
print(f"  CFCD: orig=0x{orig:02X} wrote=0xFF read=0x{rb:02X}")

# Are CECD and CFCD aliases?
print("\nAlias test: write CECD=0x55, read CFCD:")
w(0xCECD, 0x55)
print(f"  CECD=0x{r(0xCECD):02X} CFCD=0x{r(0xCFCD):02X}")
w(0xCECD, 0x00)

# Is C4xx mirrored at C5xx?
print("\nC4xx/C5xx mirror test:")
# Read matching offsets
for offset in [0x00, 0x03, 0x12, 0x18, 0x20, 0x27, 0x30, 0x39, 0x42, 0x71, 0x73]:
  c4 = r(0xC400 + offset)
  c5 = r(0xC500 + offset)
  same = "SAME" if c4 == c5 else "DIFF"
  print(f"  C4{offset:02X}=0x{c4:02X}  C5{offset:02X}=0x{c5:02X}  {same}")

# Write to C5xx and check C4xx
print("\nWrite C518=0xAA, check C418:")
orig_c418 = r(0xC418)
orig_c518 = r(0xC518)
w(0xC518, 0xAA)
print(f"  C418=0x{r(0xC418):02X} C518=0x{r(0xC518):02X}")
w(0xC518, orig_c518)

# Write to C4xx and check C5xx
print("\nWrite C418=0xBB, check C518:")
w(0xC418, 0xBB)
print(f"  C418=0x{r(0xC418):02X} C518=0x{r(0xC518):02X}")
w(0xC418, orig_c418)

# Do a scsi_write, then check if C4xx and C5xx match
print("\nAfter 4KB scsi_write:")
ctrl.scsi_write(bytes(4096), lba=0)
for offset in [0x03, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x27, 0x30, 0x39, 0x42, 0x71, 0x73]:
  c4 = r(0xC400 + offset)
  c5 = r(0xC500 + offset)
  same = "SAME" if c4 == c5 else "DIFF"
  if c4 != 0 or c5 != 0:
    print(f"  C4{offset:02X}=0x{c4:02X}  C5{offset:02X}=0x{c5:02X}  {same}")

print("\nDone!")
