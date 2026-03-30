#!/usr/bin/env python3
"""
Probe how the SRAM write pointer (CE10-CE13) controls DMA target.
Hypothesis: you can advance CE10-CE13 by writing 0x10000.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_target.py
"""
import sys, os, random, struct
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def dump_ptr(label):
  ce10 = rn(0xCE10, 4)
  ce76 = rn(0xCE76, 4)
  pci_be = struct.unpack('>I', bytes(ce10))[0]
  pci_le = struct.unpack('<I', bytes(ce76))[0]
  print(f"  {label}:")
  print(f"    CE10-13 (BE SRAM ptr): {' '.join(f'{b:02X}' for b in ce10)} -> PCI 0x{pci_be:08X}")
  print(f"    CE76-79 (LE buf addr): {' '.join(f'{b:02X}' for b in ce76)} -> PCI 0x{pci_le:08X}")

# ============================================================
# Test 1: baseline
# ============================================================
print("="*60)
print("TEST 1: Baseline SRAM pointer")
print("="*60)
dump_ptr("Current state")

# ============================================================
# Test 2: scsi_write lba=0, check pointer before/after
# ============================================================
print("\n" + "="*60)
print("TEST 2: scsi_write lba=0")
print("="*60)
dump_ptr("Before")
test = bytes([0xAA]*16) + bytes(512-16)
ctrl.scsi_write(test, lba=0)
dump_ptr("After scsi_write lba=0")
d = ctrl.read(0xF000, 4)
print(f"    F000: {d.hex()} (expect aa)")

# ============================================================
# Test 3: Try advancing CE10-13 by writing 0x10000
# CE10-13 is big-endian, so 0x00210000 would be:
#   CE10=0x00 CE11=0x21 CE12=0x00 CE13=0x00
# ============================================================
print("\n" + "="*60)
print("TEST 3: Advance SRAM pointer by writing CE10-13")
print("="*60)
dump_ptr("Before manual set")

# Try setting to 0x00210000 (advance by 0x10000 from 0x200000)
w(0xCE10, 0x00)
w(0xCE11, 0x21)
w(0xCE12, 0x00)
w(0xCE13, 0x00)
dump_ptr("After write CE10-13 = 0x00210000")

# Now scsi_write - does it go to the new address?
test3 = bytes([0xBB]*16) + bytes(512-16)
ctrl.scsi_write(test3, lba=0)
dump_ptr("After scsi_write lba=0 (with ptr=0x210000)")

# Check F000 (0x200000 window) - should NOT change if ptr moved
d_f000 = ctrl.read(0xF000, 4)
print(f"    F000: {d_f000.hex()} (should still be AA if ptr moved)")

# ============================================================
# Test 4: Try writing just CE12 to advance by 0x100 (256) in addr
# PCI 0x00200100 -> CE10=00 CE11=20 CE12=01 CE13=00
# ============================================================
print("\n" + "="*60)
print("TEST 4: Set SRAM pointer to 0x00200200 (offset +0x200)")
print("="*60)

# First fill F000 with known pattern
test4a = bytes([0x11]*512)
ctrl.scsi_write(test4a, lba=0)
d = ctrl.read(0xF000, 4)
print(f"    F000 before: {d.hex()}")
d = ctrl.read(0xF200, 4)
print(f"    F200 before: {d.hex()}")

dump_ptr("Before set")

# Set to 0x00200200
w(0xCE10, 0x00)
w(0xCE11, 0x20)
w(0xCE12, 0x02)
w(0xCE13, 0x00)
dump_ptr("After set to 0x00200200")

# Write different pattern
test4b = bytes([0x22]*16) + bytes(512-16)
ctrl.scsi_write(test4b, lba=0)
dump_ptr("After scsi_write (ptr=0x200200)")

# Check where data landed
d_f000 = ctrl.read(0xF000, 4)
d_f200 = ctrl.read(0xF200, 4)
print(f"    F000: {d_f000.hex()} (0x200000 - should be 11 if ptr worked)")
print(f"    F200: {d_f200.hex()} (0x200200 - should be 22 if ptr worked)")

# ============================================================
# Test 5: Check CE76-79 writability and if it also controls target
# ============================================================
print("\n" + "="*60)
print("TEST 5: CE76-79 vs CE10-13")
print("="*60)

# Reset CE10 back to 0x200000
w(0xCE10, 0x00)
w(0xCE11, 0x20)
w(0xCE12, 0x00)
w(0xCE13, 0x00)

# Set CE76-79 to 0x200400
w(0xCE76, 0x00)
w(0xCE77, 0x04)
w(0xCE78, 0x20)
w(0xCE79, 0x00)
dump_ptr("CE10=0x200000, CE76=0x200400")

test5 = bytes([0x33]*16) + bytes(512-16)
ctrl.scsi_write(test5, lba=0)
dump_ptr("After scsi_write")

d_f000 = ctrl.read(0xF000, 4)
d_f400 = ctrl.read(0xF400, 4)
print(f"    F000: {d_f000.hex()} (0x200000)")
print(f"    F400: {d_f400.hex()} (0x200400)")

# ============================================================
# Test 6: Does LBA affect the target at all?
# ============================================================
print("\n" + "="*60)
print("TEST 6: LBA=0 vs LBA=1 with default pointer")
print("="*60)

# Reset everything
w(0xCE10, 0x00)
w(0xCE11, 0x20)
w(0xCE12, 0x00)
w(0xCE13, 0x00)
w(0xCE76, 0x00)
w(0xCE77, 0x00)
w(0xCE78, 0x20)
w(0xCE79, 0x00)

# Write AA to lba=0
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
dump_ptr("After lba=0")
d0 = ctrl.read(0xF000, 4)
print(f"    F000: {d0.hex()}")

# Write BB to lba=1
ctrl.scsi_write(bytes([0xBB]*512), lba=1)
dump_ptr("After lba=1")
d0 = ctrl.read(0xF000, 4)
d1 = ctrl.read(0xF200, 4)
print(f"    F000: {d0.hex()} (lba=0 data)")
print(f"    F200: {d1.hex()} (lba=1 data?)")

print("\nDone!")
