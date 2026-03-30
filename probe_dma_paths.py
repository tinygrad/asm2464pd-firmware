#!/usr/bin/env python3
"""
Test the C400 DMA path on STOCK firmware.
Stock firmware uses SCSI-based E4/E5 (not control transfers).

The C400 DMA path: bulk OUT -> 0x7000 -> C400 trigger -> SRAM at 0xF000
This is what dma_explore.py uses and what we want for handmade firmware.
"""
import sys, random
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, WriteOp, ReadOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
print("="*60)
print("Pre-DMA state (stock firmware)")
print("="*60)
print(f"  900B (MSC_CFG): 0x{r(0x900B):02X}")
print(f"  C42A (DOORBELL): 0x{r(0xC42A):02X}")
print(f"  C471 (QUEUE_BUSY): 0x{r(0xC471):02X}")
print(f"  F000 (SRAM): {rn(0xF000, 4).hex()}")

# Can we read 0x7000 on stock firmware?
try:
  buf = rn(0x7000, 4)
  print(f"  7000: {buf.hex()}")
except:
  print(f"  7000: read failed")

# ============================================================
print("\n" + "="*60)
print("C400 DMA: Set descriptor + arm + trigger via E5 writes")
print("="*60)

# First, do a scsi_write to put known data at F000
ctrl.scsi_write(bytes([0x11]*512), lba=0)
print(f"  F000 after scsi_write: {rn(0xF000, 4).hex()}")

# Now try to overwrite F000 via the C400 path
# But on stock firmware, we can't do bulk_out — it goes through MSC.
# The C400 path requires data at 0x7000 which comes from bulk OUT.
# On stock firmware, the only way to get data to 0x7000 is through
# the MSC engine (scsi_write).

# Let's check: after scsi_write, is 0x7000 accessible?
buf7 = rn(0x7000, 4)
print(f"  7000 after scsi_write: {buf7.hex()}")

# Try C400 DMA trigger to copy 0x7000 -> SRAM again
print("\n  Setting C420 descriptor...")
w(0xC420, 0x00)
w(0xC421, 0x01)
w(0xC422, 0x02)
w(0xC423, 0x00)
w(0xC424, 0x00)
w(0xC425, 0x00)
w(0xC426, 0x00)
w(0xC427, 0x01)  # 1 sector
w(0xC428, 0x30)
w(0xC429, 0x00)

# Write different data to 0x7000 via E5 (CPU write)
print("  Writing 0xDEAD to 0x7000 via E5...")
w(0x7000, 0xDE)
w(0x7001, 0xAD)
w(0x7002, 0xBE)
w(0x7003, 0xEF)
print(f"  7000 after E5 write: {rn(0x7000, 4).hex()}")

# Now arm and trigger
print("  Arming C400 DMA...")
w(0xC400, 1)
w(0xC401, 1)
w(0xC402, 1)
w(0xC404, 1)
print("  Triggering C406...")
w(0xC406, 1)

# Check result
f_after = rn(0xF000, 4)
print(f"  F000 after trigger: {f_after.hex()}")
print(f"  Expected deadbeef: {f_after == bytes([0xDE, 0xAD, 0xBE, 0xEF])}")

# Check 0x7000 locked?
buf7 = rn(0x7000, 4)
print(f"  7000 (locked?): {buf7.hex()}")

# Unlock
w(0xC42A, 0x01)
buf7 = rn(0x7000, 4)
print(f"  7000 after unlock: {buf7.hex()}")

# ============================================================
print("\n" + "="*60)
print("Test: Does CE00 path work with CPU-written 0x7000 data?")
print("="*60)

# CE00 docs say: "writing to 0x7000 via 8051 CPU does NOT set up DMA source correctly"
# But let's verify with C400 path

# Write known pattern to 0x7000
for i in range(16):
  w(0x7000 + i, 0xA0 + i)
print(f"  7000: {rn(0x7000, 16).hex()}")

# C400 trigger
w(0xC427, 0x01)
w(0xC400, 1)
w(0xC401, 1)
w(0xC402, 1)
w(0xC404, 1)
w(0xC406, 1)

f = rn(0xF000, 16)
print(f"  F000: {f.hex()}")
print(f"  Match: {f[:4] == bytes([0xA0, 0xA1, 0xA2, 0xA3])}")
w(0xC42A, 0x01)  # unlock

# ============================================================
print("\n" + "="*60)
print("Test: Can we do multiple C400 DMA transfers?")
print("="*60)

for i in range(5):
  for j in range(4):
    w(0x7000 + j, (i * 0x10 + j) & 0xFF)
  w(0xC427, 0x01)
  w(0xC400, 1); w(0xC401, 1); w(0xC402, 1); w(0xC404, 1); w(0xC406, 1)
  f = rn(0xF000, 4)
  expected = bytes([(i * 0x10 + j) & 0xFF for j in range(4)])
  print(f"  [{i}] F000={f.hex()} expected={expected.hex()} {'OK' if f == expected else 'FAIL'}")
  w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("Test: Minimum arm bits needed")
print("="*60)

def test_arm(label, c400, c401, c402, c404):
  w(0x7000, 0xBB); w(0x7001, 0xCC)
  w(0xC427, 0x01)
  if c400: w(0xC400, 1)
  if c401: w(0xC401, 1)
  if c402: w(0xC402, 1)
  if c404: w(0xC404, 1)
  w(0xC406, 1)
  f = rn(0xF000, 2)
  ok = f == bytes([0xBB, 0xCC])
  print(f"  {label}: F000={f.hex()} {'OK' if ok else 'FAIL'}")
  w(0xC42A, 0x01)

test_arm("all",         True,  True,  True,  True)
test_arm("no C400",     False, True,  True,  True)
test_arm("no C401",     True,  False, True,  True)
test_arm("no C402",     True,  True,  False, True)
test_arm("no C404",     True,  True,  True,  False)
test_arm("C406 only",   False, False, False, False)

print("\nDone!")
