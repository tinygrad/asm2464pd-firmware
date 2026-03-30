#!/usr/bin/env python3
"""Deep probe part 2: C478 counters onwards."""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)
def reset_ce6e(): ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("EXPERIMENT 2: C478/C479 counters")
print("="*60)

v = rn(0xC478, 2)
print(f"  Start: C478={v[0]:02X} C479={v[1]:02X}")

_ = r(0xC412)  # 1 E4 read
v = rn(0xC478, 2)
print(f"  +1 E4: C478={v[0]:02X} C479={v[1]:02X}")

w(0xF000, 0x00)  # 1 E5 write
v = rn(0xC478, 2)
print(f"  +1 E5: C478={v[0]:02X} C479={v[1]:02X}")

# scsi_write
v_before = rn(0xC478, 2)
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
v_after = rn(0xC478, 2)
d78 = (v_after[0] - v_before[0]) & 0xFF
d79 = (v_after[1] - v_before[1]) & 0xFF
print(f"  scsi_write delta: C478+={d78} C479+={d79}")
reset_ce6e()

# writable?
orig = rn(0xC478, 2)
w(0xC478, 0x00); w(0xC479, 0x00)
rb = rn(0xC478, 2)
print(f"  Writable? orig={orig.hex()} wrote=0000 read={rb.hex()} -> {'R/W' if rb==b'\\x00\\x00' else 'R/O'}")

# ============================================================
print("\nEXPERIMENT 3: C4B0-C4B5")
print("="*60)

d = rn(0xC4B0, 6)
print(f"  Baseline: {' '.join(f'{b:02X}' for b in d)}")

for i in range(6):
  addr = 0xC4B0 + i
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv); rb = r(addr); w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

for i in range(5):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  d = rn(0xC4B0, 6)
  print(f"  write[{i}]: {' '.join(f'{b:02X}' for b in d)}")
  reset_ce6e()

# ============================================================
print("\nEXPERIMENT 4: C450-C46F queue area")
print("="*60)

d = rn(0xC450, 0x20)
for i in range(len(d)):
  if d[i] != 0:
    print(f"  0x{0xC450+i:04X} = 0x{d[i]:02X}")

# 3 more writes
for _ in range(3):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  reset_ce6e()
d2 = rn(0xC450, 0x20)
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
if changed:
  for i, old, new in changed: print(f"  0x{0xC450+i:04X}: {old:02X} -> {new:02X}")
else:
  print("  (no changes after 3 writes)")

print("\nDone!")
