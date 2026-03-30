#!/usr/bin/env python3
"""
On STOCK firmware: figure out how to advance the SRAM write pointer
between C400 DMA triggers using E5 register writes.

Stock scsi_write does: ScsiWriteOp -> CE6E reset
CE10 goes 0x200000 -> 0x204000 -> back to 0x200000 after CE6E reset.

But within a single scsi_write, the firmware internally advances through
SRAM. We need to figure out how to replicate this pointer advance
using just register writes (no SCSI commands), so the C400 DMA trigger
writes to successive SRAM addresses.

Plan:
  1. scsi_write some data so DMA state is hot
  2. Try writing CE10 directly (we know it's R/O, but recheck)
  3. Try CE6E/CE6F manipulation to advance pointer
  4. Try C400 DMA trigger, then CE6F++, then C400 again
  5. Check if C420-C427 descriptor fields advance the pointer
"""
import sys, random
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def ce10(): return rn(0xCE10, 4).hex()
def ce6e(): return rn(0xCE6E, 2).hex()

# ============================================================
print("="*60)
print("Baseline: CE10 pointer behavior with scsi_write")
print("="*60)

print(f"  CE10={ce10()} CE6E={ce6e()}")

# Small write
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
print(f"  After 512B ScsiWriteOp: CE10={ce10()} CE6E={ce6e()}")

# DON'T do the CE6E reset yet. Do another write.
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
print(f"  After 2nd ScsiWriteOp (no reset): CE10={ce10()} CE6E={ce6e()}")

# And another
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
print(f"  After 3rd ScsiWriteOp (no reset): CE10={ce10()} CE6E={ce6e()}")

# Check where data landed
print(f"  F000: {rn(0xF000, 4).hex()}")
print(f"  F200: {rn(0xF200, 4).hex()}")

# Reset
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
print(f"  After CE6E reset: CE10={ce10()}")

# ============================================================
print("\n" + "="*60)
print("TEST: Multiple ScsiWriteOps WITHOUT CE6E reset between")
print("  Does CE10 advance further with each?")
print("="*60)

# Write unique patterns without resetting
for i in range(4):
  pat = bytes([0x10*i + j for j in range(4)] + [0]*(512-4))
  ctrl.exec_ops([ScsiWriteOp(pat, lba=0)])
  print(f"  [{i}] CE10={ce10()} CE6E={ce6e()}")

# Check SRAM
print("\n  SRAM readback:")
for off in range(0, 0x1000, 0x200):
  d = rn(0xF000 + off, 4)
  print(f"    F{off:03X}: {d.hex()}")

# Reset
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("TEST: Write CE6F to advance pointer, then scsi_write")
print("="*60)

print(f"  CE10={ce10()}")

# Advance CE6F by 1
w(0xCE6F, 0x01)
print(f"  After CE6F=1: CE10={ce10()} CE6E={ce6e()}")

# Now scsi_write — where does it go?
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*512), lba=0)])
print(f"  After ScsiWriteOp: CE10={ce10()}")

print(f"  F000: {rn(0xF000, 4).hex()}")
print(f"  F200: {rn(0xF200, 4).hex()}")

ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("TEST: Directly write C400 descriptor and trigger via E5")
print("  on stock firmware (C400 DMA via SCSI E5 commands)")
print("="*60)

# First put known data at F000
ctrl.scsi_write(bytes([0x11]*512), lba=0)
print(f"  F000 after scsi_write: {rn(0xF000, 4).hex()}")

# Now try C400 DMA trigger via E5
# Write different data to 0x7000 via E5
w(0x7000, 0xDE); w(0x7001, 0xAD); w(0x7002, 0xBE); w(0x7003, 0xEF)
print(f"  7000: {rn(0x7000, 4).hex()}")

# Set descriptor and trigger
w(0xC427, 0x01)
w(0xC42A, 0x00)
w(0xC406, 0x01)

print(f"  F000 after C400 trigger: {rn(0xF000, 4).hex()}")
w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST: C400 trigger, advance CE10 via CE6F, C400 again")
print("="*60)

# 1st DMA: data AA to F000
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")

# Now advance CE10 via CE6F
w(0xCE6F, 0x01)
print(f"  CE10 after CE6F=1: {ce10()}")

# 2nd scsi_write with BB - does it go to the new CE10 location?
ctrl.exec_ops([ScsiWriteOp(bytes([0xBB]*512), lba=0)])
print(f"  CE10 after 2nd write: {ce10()}")

# Check both locations
print(f"  F000: {rn(0xF000, 4).hex()} (AA if BB went elsewhere)")
for off in [0x200, 0x400, 0x800, 0xF00]:
  d = rn(0xF000 + off, 4)
  if d != bytes([0xAA]*4) and d != bytes([0x55]*4):
    print(f"  F{off:03X}: {d.hex()} *** NEW DATA")
  else:
    print(f"  F{off:03X}: {d.hex()}")

ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("TEST: Increment CE6F multiple times, check CE10 each time")
print("="*60)

print(f"  CE10={ce10()}")
for i in range(1, 6):
  w(0xCE6F, i)
  print(f"  CE6F={i}: CE10={ce10()} CE6E={ce6e()}")

ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
print(f"  After reset: CE10={ce10()}")

# Write 1,2,3,4,5 to CE6F in sequence
for i in range(1, 6):
  w(0xCE6F, i)
  
print(f"  After writing 1..5: CE10={ce10()}")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("\nDone!")
