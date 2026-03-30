#!/usr/bin/env python3
"""
Safe probe: READ-ONLY experiments first, then scsi_writes.
No writability tests that could corrupt state.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)
def reset_ce6e(): ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

def do_write(sz):
  ctrl.exec_ops([ScsiWriteOp(bytes(sz), lba=0)])
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("PART A: Read-only observations")
print("="*60)

# C478/C479
vals = []
for i in range(6):
  v = rn(0xC478, 2)
  vals.append((v[0], v[1]))
print(f"C478/C479 across 6 reads:")
for i, (a, b) in enumerate(vals):
  print(f"  [{i}] C478=0x{a:02X} C479=0x{b:02X}")
print(f"  C478 delta per read: {vals[1][0]-vals[0][0]}")
print(f"  C479 delta per read: {vals[1][1]-vals[0][1]}")
print(f"  C479 = C478 - 1 always: {all(b == a-1 for a,b in vals)}")

# CE02 volatile
vals = [r(0xCE02) for _ in range(10)]
print(f"\nCE02 x10: {' '.join(f'{v:02X}' for v in vals)}")
print(f"  Alternates 0/1: {vals[1:] == [1-v for v in vals[:-1]] or 'mixed'}")

# CE20-27 constant
d = rn(0xCE20, 8)
print(f"\nCE20-27: {' '.join(f'{b:02X}' for b in d)}")

# CE3C-3F
d = rn(0xCE3C, 4)
print(f"CE3C-3F: {' '.join(f'{b:02X}' for b in d)}")

# CE44-49
d = rn(0xCE44, 6)
print(f"CE44-49: {' '.join(f'{b:02X}' for b in d)}")

# C470-7F
d = rn(0xC470, 16)
print(f"\nC470-7F: {' '.join(f'{b:02X}' for b in d)}")

# C480-8F
d = rn(0xC480, 16)
print(f"C480-8F: {' '.join(f'{b:02X}' for b in d)}")

# C4B0-BF
d = rn(0xC4B0, 16)
print(f"C4B0-BF: {' '.join(f'{b:02X}' for b in d)}")

# C4E0-FF
d = rn(0xC4E0, 32)
print(f"C4E0-EF: {' '.join(f'{b:02X}' for b in d[:16])}")
print(f"C4F0-FF: {' '.join(f'{b:02X}' for b in d[16:])}")

# CE86-8F
d = rn(0xCE86, 10)
print(f"\nCE86-8F: {' '.join(f'{b:02X}' for b in d)}")

# C410-1F
d = rn(0xC410, 16)
print(f"C410-1F: {' '.join(f'{b:02X}' for b in d)}")

# C450-6F
d = rn(0xC450, 32)
print(f"C450-5F: {' '.join(f'{b:02X}' for b in d[:16])}")
print(f"C460-6F: {' '.join(f'{b:02X}' for b in d[16:])}")

# ============================================================
print("\n" + "="*60)
print("PART B: Track changes across scsi_writes")
print("="*60)

# Snapshot key areas before/after each write
regs_to_watch = [
  (0xC410, 16, "C410-1F"),
  (0xC450, 32, "C450-6F"),
  (0xC470, 16, "C470-7F"),
  (0xC480, 16, "C480-8F"),
  (0xC4B0, 16, "C4B0-BF"),
  (0xC4E0, 32, "C4E0-FF"),
  (0xCE00, 16, "CE00-0F"),
  (0xCE10, 4,  "CE10-13"),
  (0xCE40, 16, "CE40-4F"),
  (0xCE50, 16, "CE50-5F"),
  (0xCE60, 16, "CE60-6F"),
  (0xCE70, 16, "CE70-7F"),
  (0xCE80, 16, "CE80-8F"),
]

def snap_all():
  result = {}
  for addr, sz, label in regs_to_watch:
    result[(addr, sz, label)] = rn(addr, sz)
  return result

def diff_snaps(before, after):
  changes = []
  for key in before:
    addr, sz, label = key
    d1, d2 = before[key], after[key]
    for i in range(sz):
      if d1[i] != d2[i]:
        changes.append((addr+i, d1[i], d2[i]))
  return changes

before = snap_all()

for sz_label, sz in [("512B", 512), ("4KB", 4096), ("16KB", 16384)]:
  print(f"\n--- After {sz_label} scsi_write ---")
  do_write(sz)
  after = snap_all()
  changes = diff_snaps(before, after)
  if changes:
    for addr, old, new in sorted(changes):
      print(f"  0x{addr:04X}: {old:02X} -> {new:02X}")
  else:
    print("  (no changes)")
  before = after

# ============================================================
print("\n" + "="*60)
print("PART C: CE6E values after various write sizes")
print("="*60)

for sz in [512, 1024, 2048, 4096, 8192, 16384, 32768, 65536]:
  reset_ce6e()
  ctrl.exec_ops([ScsiWriteOp(bytes(sz), lba=0)])
  ce6e = rn(0xCE6E, 2)
  ce10 = rn(0xCE10, 4)
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  print(f"  {sz:5d}B ({sz//512:3d} sectors): CE6E={ce6e.hex()} CE10={ce10.hex()}")
  reset_ce6e()

print("\nDone!")
