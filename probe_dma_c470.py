#!/usr/bin/env python3
"""
Focused probe on the C450-C47F and C4B0-C4BF areas which changed
significantly during scsi_write. Also test CE76-CE79 address auto-increment.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_dma_c470.py
"""
import sys, os, random
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
# Test 1: C478/C479 counter behavior - are these E4/E5 counters?
# ============================================================
print("="*60)
print("TEST 1: C478/C479 - command counters?")
print("="*60)

vals = []
for i in range(5):
  v78 = r(0xC478)
  v79 = r(0xC479)
  vals.append((v78, v79))
  print(f"  [{i}] C478=0x{v78:02X}  C479=0x{v79:02X}")

# Check if C478 increments with each E4 read
print(f"  -> C478 increments: {vals[1][0] - vals[0][0]}, {vals[2][0] - vals[1][0]}, {vals[3][0] - vals[2][0]}")
print(f"  -> C479 increments: {vals[1][1] - vals[0][1]}, {vals[2][1] - vals[1][1]}, {vals[3][1] - vals[2][1]}")

# ============================================================
# Test 2: C4B0-C4B5 - what are these? They changed with each scsi_write
# ============================================================
print("\n" + "="*60)
print("TEST 2: C4B0-C4B5 - SCSI command tracking?")
print("="*60)

d = rn(0xC4B0, 16)
print(f"  C4B0-C4BF: {' '.join(f'{b:02X}' for b in d)}")

# Do a scsi_write and check again
test_data = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test_data, lba=0)
d2 = rn(0xC4B0, 16)
print(f"  After write1: {' '.join(f'{b:02X}' for b in d2)}")

test_data2 = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test_data2, lba=0)
d3 = rn(0xC4B0, 16)
print(f"  After write2: {' '.join(f'{b:02X}' for b in d3)}")

# Identify which bytes change
for i in range(16):
  if d[i] != d2[i] or d2[i] != d3[i]:
    print(f"  0x{0xC4B0+i:04X} changed: {d[i]:02X} -> {d2[i]:02X} -> {d3[i]:02X}")

# ============================================================
# Test 3: C470 - what is this? It was 0x00 idle, 0xC0 after write
# ============================================================
print("\n" + "="*60)
print("TEST 3: C470 and neighbors")
print("="*60)
d = rn(0xC470, 16)
print(f"  C470-C47F: {' '.join(f'{b:02X}' for b in d)}")

# Is C470 writable?
orig = r(0xC470)
w(0xC470, 0xAA)
readback = r(0xC470)
w(0xC470, orig)
print(f"  C470: orig=0x{orig:02X}, wrote=0xAA, read=0x{readback:02X}")

# C471 writable?
orig = r(0xC471)
w(0xC471, 0x00)
readback = r(0xC471)
w(0xC471, orig)
print(f"  C471: orig=0x{orig:02X}, wrote=0x00, read=0x{readback:02X}")

# C473 writable?
orig = r(0xC473)
w(0xC473, 0x00)
readback = r(0xC473)
w(0xC473, orig)
print(f"  C473: orig=0x{orig:02X}, wrote=0x00, read=0x{readback:02X}")

# C47E writable?
orig = r(0xC47E)
w(0xC47E, 0x00)
readback = r(0xC47E)
w(0xC47E, orig)
print(f"  C47E: orig=0x{orig:02X}, wrote=0x00, read=0x{readback:02X}")

# ============================================================
# Test 4: CE76-CE79 after scsi_write - does it auto-increment?
# ============================================================
print("\n" + "="*60)
print("TEST 4: CE76-CE79 buffer address after writes")
print("="*60)

# Current state
ce76 = rn(0xCE76, 4)
print(f"  CE76-79 current: {' '.join(f'{b:02X}' for b in ce76)}")
print(f"           -> PCI address: 0x{ce76[3]:02X}{ce76[2]:02X}{ce76[1]:02X}{ce76[0]:02X}")

# After another write (1 sector)
ctrl.scsi_write(bytes(512), lba=0)
ce76 = rn(0xCE76, 4)
print(f"  CE76-79 after 1-sector write: {' '.join(f'{b:02X}' for b in ce76)}")
print(f"           -> PCI address: 0x{ce76[3]:02X}{ce76[2]:02X}{ce76[1]:02X}{ce76[0]:02X}")

# After 2-sector write
ctrl.scsi_write(bytes(1024), lba=0)
ce76 = rn(0xCE76, 4)
print(f"  CE76-79 after 2-sector write: {' '.join(f'{b:02X}' for b in ce76)}")
print(f"           -> PCI address: 0x{ce76[3]:02X}{ce76[2]:02X}{ce76[1]:02X}{ce76[0]:02X}")

# Also check CE10-CE13 (SRAM pointer)
ce10 = rn(0xCE10, 4)
print(f"  CE10-13 SRAM ptr: {' '.join(f'{b:02X}' for b in ce10)}")
print(f"           -> PCI address: 0x{ce10[0]:02X}{ce10[1]:02X}{ce10[2]:02X}{ce10[3]:02X} (big-endian)")

# ============================================================
# Test 5: CE20-CE22 always reads 50 CE 20 - what is this?
# ============================================================
print("\n" + "="*60)
print("TEST 5: CE20-CE23 constant pattern")
print("="*60)
d = rn(0xCE20, 8)
print(f"  CE20-27: {' '.join(f'{b:02X}' for b in d)}")
# 50 CE could be 0xCE50 reversed? A pointer to CE50?

# ============================================================
# Test 6: C430 PRP area - changed 00->01 after write
# ============================================================
print("\n" + "="*60)
print("TEST 6: C430-C44F PRP/Init area")
print("="*60)
d = rn(0xC430, 0x20)
print(f"  C430-C44F: {' '.join(f'{b:02X}' for b in d)}")
# Try writing C430
orig = r(0xC430)
w(0xC430, 0x00)
readback = r(0xC430)
w(0xC430, orig)
print(f"  C430: orig=0x{orig:02X}, wrote=0x00, read=0x{readback:02X}")

# ============================================================
# Test 7: C450-C46F detail with writability
# ============================================================
print("\n" + "="*60)
print("TEST 7: C450-C46F writability test")
print("="*60)
for addr in range(0xC450, 0xC470):
  orig = r(addr)
  if orig != 0:
    test_val = 0x00
  else:
    test_val = 0xAA
  w(addr, test_val)
  readback = r(addr)
  w(addr, orig)
  if orig != 0 or readback != 0:
    rw = "R/W" if readback == test_val else "R/O" if readback == orig else f"PARTIAL(0x{readback:02X})"
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{test_val:02X} read=0x{readback:02X} -> {rw}")

# ============================================================
# Test 8: Full C400-C420 area writability
# ============================================================
print("\n" + "="*60)
print("TEST 8: C400-C41F writability (CAREFUL - DMA control)")
print("="*60)
# Only read, don't write to trigger registers
for addr in range(0xC400, 0xC420):
  orig = r(addr)
  if orig != 0:
    print(f"  0x{addr:04X} = 0x{orig:02X}")

# Safe to test C410-C41F which are status registers
for addr in [0xC410, 0xC411, 0xC412, 0xC413, 0xC414, 0xC415, 0xC416, 0xC417, 0xC418, 0xC419]:
  orig = r(addr)
  test_val = 0xAA if orig != 0xAA else 0x55
  w(addr, test_val)
  readback = r(addr)
  w(addr, orig)
  rw = "R/W" if readback == test_val else "R/O" if readback == orig else f"PARTIAL(0x{readback:02X})"
  if rw != "R/O" or orig != 0:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{test_val:02X} read=0x{readback:02X} -> {rw}")

print("\nDone!")
