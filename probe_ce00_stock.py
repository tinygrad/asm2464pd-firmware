#!/usr/bin/env python3
"""
On STOCK firmware: test CE00 DMA with manual CE76-CE79 targeting.
After a scsi_write puts data at F000, can we re-trigger CE00
to copy to a different address?

Also test: after C400 DMA puts data at 7000/F000, can CE00
then move it to a different SRAM location?
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
print("="*60)
print("TEST 1: scsi_write then manual CE00 to different addr")
print("="*60)

# First, scsi_write to get data at F000
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")
print(f"  F200: {rn(0xF200, 4).hex()}")

# Now set CE76-79 to PCI 0x200200 (= F200 in XDATA window)
w(0xCE76, 0x00)
w(0xCE77, 0x02)  # 0x0200
w(0xCE78, 0x20)  # 0x20xxxx
w(0xCE79, 0x00)
print(f"  CE76-79: {rn(0xCE76, 4).hex()} (target PCI 0x200200)")

# Trigger CE00
print(f"  CE00 before: 0x{r(0xCE00):02X}")
w(0xCE00, 0x03)

# Poll CE00 for completion
import time
for i in range(100):
  v = r(0xCE00)
  if v == 0x00:
    print(f"  CE00 completed after {i} polls")
    break
else:
  print(f"  CE00 still 0x{v:02X} after 100 polls")

# Check F200 — did data arrive?
print(f"  F000: {rn(0xF000, 4).hex()} (should still be AA)")
print(f"  F200: {rn(0xF200, 4).hex()} (AA if CE00 copied here)")

# ============================================================
print("\n" + "="*60)
print("TEST 2: CE00 to address F400 (PCI 0x200400)")
print("="*60)

w(0xCE76, 0x00)
w(0xCE77, 0x04)  # 0x0400
w(0xCE78, 0x20)
w(0xCE79, 0x00)

w(0xCE00, 0x03)
for i in range(100):
  if r(0xCE00) == 0x00:
    print(f"  CE00 completed after {i} polls")
    break

print(f"  F400: {rn(0xF400, 4).hex()} (AA if CE00 copied)")

# ============================================================
print("\n" + "="*60)
print("TEST 3: Repeated CE00 to fill F200, F400, F600, F800")
print("="*60)

# First make sure F000 has data (from earlier scsi_write)
print(f"  F000: {rn(0xF000, 4).hex()}")

for offset in [0x200, 0x400, 0x600, 0x800]:
  pci_lo = offset & 0xFF
  pci_hi = (offset >> 8) & 0xFF
  w(0xCE76, pci_lo)
  w(0xCE77, pci_hi)
  w(0xCE78, 0x20)
  w(0xCE79, 0x00)
  
  w(0xCE00, 0x03)
  for i in range(200):
    if r(0xCE00) == 0x00:
      break
  
  d = rn(0xF000 + offset, 4)
  print(f"  F{offset:03X}: {d.hex()}")

# ============================================================
print("\n" + "="*60)
print("TEST 4: CE00 with unique data — change 0x7000 between CE00s")
print("="*60)

# Write BB to 0x7000 via E5
for i in range(512):
  w(0x7000 + i, 0xBB)

# CE00 to F200
w(0xCE76, 0x00); w(0xCE77, 0x02); w(0xCE78, 0x20); w(0xCE79, 0x00)
w(0xCE00, 0x03)
for i in range(200):
  if r(0xCE00) == 0x00: break

print(f"  F000: {rn(0xF000, 4).hex()} (should be AA)")
print(f"  F200: {rn(0xF200, 4).hex()} (BB if CE00 uses 0x7000 CPU data)")

# ============================================================
print("\n" + "="*60)
print("TEST 5: CE00 status registers during operation")
print("="*60)

print(f"  CE00: 0x{r(0xCE00):02X}")
print(f"  CE01: 0x{r(0xCE01):02X}")
print(f"  CE02: 0x{r(0xCE02):02X}")
print(f"  CE10-13: {rn(0xCE10, 4).hex()}")
print(f"  CE55: 0x{r(0xCE55):02X}")
print(f"  CE83: 0x{r(0xCE83):02X}")
print(f"  CE88: 0x{r(0xCE88):02X}")
print(f"  CE89: 0x{r(0xCE89):02X}")

print("\nDone!")
