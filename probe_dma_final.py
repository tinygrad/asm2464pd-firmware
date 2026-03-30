#!/usr/bin/env python3
"""
Final targeted DMA register probe: fix C473, test CE76 writability, 
check C487/C489 counters, and verify SCSI write still works.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_dma_final.py
"""
import sys, os, random
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
# Restore C473 if it was zeroed
# ============================================================
print("Restoring C473 to 0x66...")
w(0xC473, 0x66)
print(f"  C473 = 0x{r(0xC473):02X}")

# ============================================================
# Test CE76-CE79 writability
# ============================================================
print("\n" + "="*60)
print("TEST 1: CE76-CE79 writability")
print("="*60)

for addr in [0xCE76, 0xCE77, 0xCE78, 0xCE79]:
  orig = r(addr)
  w(addr, 0xAA)
  readback = r(addr)
  w(addr, orig)
  rw = "R/W" if readback == 0xAA else "R/O" if readback == orig else f"PARTIAL(0x{readback:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0xAA read=0x{readback:02X} -> {rw}")

# Write the SRAM target address for DMA
print("\n  Setting CE76-79 to PCI 0x00200000:")
w(0xCE76, 0x00)
w(0xCE77, 0x00)
w(0xCE78, 0x20)
w(0xCE79, 0x00)
ce = rn(0xCE76, 4)
print(f"  CE76-79 = {' '.join(f'{b:02X}' for b in ce)}")

# ============================================================  
# Test CE83 flow control bits
# ============================================================
print("\n" + "="*60)
print("TEST 2: CE83 flow control bits")
print("="*60)

orig = r(0xCE83)
print(f"  CE83 orig = 0x{orig:02X} (bits: {orig:08b})")

# Test clear bits 4-6 (& 0x8F)
w(0xCE83, orig & 0x8F)
readback = r(0xCE83)
print(f"  CE83 after &0x8F = 0x{readback:02X} (bits: {readback:08b})")
w(0xCE83, orig)  # restore

# ============================================================
# Test C487/C489 - are these transfer counters?
# ============================================================
print("\n" + "="*60)
print("TEST 3: C487/C489/C48A - DMA extended state")
print("="*60)

d = rn(0xC480, 16)
print(f"  C480-C48F: {' '.join(f'{b:02X}' for b in d)}")

# Do a scsi_write and check
test_data = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test_data, lba=0)
d2 = rn(0xC480, 16)
print(f"  After write: {' '.join(f'{b:02X}' for b in d2)}")

for i in range(16):
  if d[i] != d2[i]:
    print(f"  0x{0xC480+i:04X} changed: {d[i]:02X} -> {d2[i]:02X}")

# ============================================================
# Test C4F0-C4FF area
# ============================================================
print("\n" + "="*60)
print("TEST 4: C4F0-C4FF area")
print("="*60)

d = rn(0xC4E0, 32)
print(f"  C4E0-C4EF: {' '.join(f'{b:02X}' for b in d[:16])}")
print(f"  C4F0-C4FF: {' '.join(f'{b:02X}' for b in d[16:])}")

# Writability of C4EB-C4EF
for addr in [0xC4EB, 0xC4EC, 0xC4ED, 0xC4EE, 0xC4EF]:
  orig = r(addr)
  test_val = 0xAA if orig != 0xAA else 0x55
  w(addr, test_val)
  readback = r(addr)
  w(addr, orig)
  rw = "R/W" if readback == test_val else "R/O" if readback == orig else f"PARTIAL(0x{readback:02X})"
  if rw != "R/O" or orig != 0:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{test_val:02X} read=0x{readback:02X} -> {rw}")

# ============================================================
# Test CE00-CE09 area (DMA ctrl)
# ============================================================
print("\n" + "="*60)
print("TEST 5: CE00-CE0F DMA control detail")
print("="*60)
d = rn(0xCE00, 16)
print(f"  CE00-CE0F: {' '.join(f'{b:02X}' for b in d)}")

# CE00 writability
orig = r(0xCE00)
w(0xCE00, 0x03)  # trigger DMA (but nothing should happen without setup)
readback = r(0xCE00)
print(f"  CE00: wrote=0x03, read=0x{readback:02X} (should auto-clear to 0x00)")

# CE01 writability  
orig = r(0xCE01)
w(0xCE01, 0xAA)
readback = r(0xCE01)
w(0xCE01, orig)
print(f"  CE01: orig=0x{orig:02X}, wrote=0xAA, read=0x{readback:02X}")

# CE05 - always 0xFF
print(f"  CE05 = 0x{r(0xCE05):02X} (expected 0xFF)")

# ============================================================
# Test CE3C-CE3F 
# ============================================================
print("\n" + "="*60)
print("TEST 6: CE30-CE3F area")
print("="*60)
d = rn(0xCE30, 16)
print(f"  CE30-CE3F: {' '.join(f'{b:02X}' for b in d)}")

# ============================================================
# Test CE44-CE49 (the 50/05/55 pattern)
# ============================================================
print("\n" + "="*60)
print("TEST 7: CE44-CE49 pattern analysis")
print("="*60)
d = rn(0xCE44, 6)
print(f"  CE44-CE49: {' '.join(f'{b:02X}' for b in d)}")
print(f"  CE44-45 as LE u16: 0x{d[1]:02X}{d[0]:02X} = {d[1]*256+d[0]}")
print(f"  CE46-47-48 = 0x55,0x50,0x05 -> looks like a pattern/marker")

# ============================================================
# Test: Verify scsi_write still works after all our probing
# ============================================================
print("\n" + "="*60)
print("VERIFY: scsi_write still works")
print("="*60)

test = bytes(random.getrandbits(8) for _ in range(512))
before = ctrl.read(0xF000, 16)
ctrl.scsi_write(test, lba=0)
after = ctrl.read(0xF000, 16)
print(f"  Before: {before.hex()}")
print(f"  After:  {after.hex()}")
print(f"  Data:   {test[:16].hex()}")
print(f"  MATCH: {after == test[:16]}")

# ============================================================
# Summary: CE80-CE9F detailed writability
# ============================================================
print("\n" + "="*60)
print("TEST 8: CE80-CE9F full writability scan")
print("="*60)
for addr in range(0xCE80, 0xCEA0):
  orig = r(addr)
  test_val = 0xAA if orig != 0xAA else 0x55
  w(addr, test_val)
  readback = r(addr)
  w(addr, orig)
  rw = "R/W" if readback == test_val else "R/O" if readback == orig else f"PARTIAL(0x{readback:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

print("\nDone!")
