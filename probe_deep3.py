#!/usr/bin/env python3
"""Deep probe part 3: CE area details."""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)
def reset_ce6e(): ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("EXPERIMENT 5: CE02 volatile")
print("="*60)
vals = [r(0xCE02) for _ in range(20)]
print(f"  CE02 x20: {' '.join(f'{v:02X}' for v in vals)}")

# ============================================================
print("\nEXPERIMENT 6: CE20-CE27")
print("="*60)
d = rn(0xCE20, 8)
print(f"  CE20-27: {' '.join(f'{b:02X}' for b in d)}")
# All R/O from earlier. 50 CE = 0xCE50 in LE.

# ============================================================
print("\nEXPERIMENT 7: CE3C-CE3F")
print("="*60)
d = rn(0xCE3C, 4)
print(f"  CE3C-3F: {' '.join(f'{b:02X}' for b in d)}")

# ============================================================
print("\nEXPERIMENT 8: CE44-CE49")
print("="*60)
d = rn(0xCE44, 6)
print(f"  CE44-49: {' '.join(f'{b:02X}' for b in d)}")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xCE44, 6)
print(f"  After 512B: {' '.join(f'{b:02X}' for b in d2)}")
reset_ce6e()
ctrl.exec_ops([ScsiWriteOp(bytes(4096), lba=0)])
d3 = rn(0xCE44, 6)
print(f"  After 4KB: {' '.join(f'{b:02X}' for b in d3)}")
reset_ce6e()

# ============================================================
print("\nEXPERIMENT 9: C412-C419")
print("="*60)
d = rn(0xC410, 16)
print(f"  C410-1F: {' '.join(f'{b:02X}' for b in d)}")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xC410, 16)
print(f"  After wr: {' '.join(f'{b:02X}' for b in d2)}")
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
for i, old, new in changed: print(f"    0x{0xC410+i:04X}: {old:02X} -> {new:02X}")
if not changed: print("    (no changes)")
reset_ce6e()

# ============================================================
print("\nEXPERIMENT 10: CE86-CE8F handshake")
print("="*60)
d = rn(0xCE86, 10)
print(f"  CE86-8F: {' '.join(f'{b:02X}' for b in d)}")
print(f"  CE89 x5: {' '.join(f'{r(0xCE89):02X}' for _ in range(5))}")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xCE86, 10)
print(f"  After wr: {' '.join(f'{b:02X}' for b in d2)}")
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
for i, old, new in changed: print(f"    0x{0xCE86+i:04X}: {old:02X} -> {new:02X}")
reset_ce6e()

# ============================================================
print("\nEXPERIMENT 11: C470-C47F")
print("="*60)
d = rn(0xC470, 16)
print(f"  C470-7F: {' '.join(f'{b:02X}' for b in d)}")
for i in range(16):
  if d[i] != 0:
    print(f"  0x{0xC470+i:04X} = 0x{d[i]:02X} ({d[i]:08b})")

# Writability of key regs
for addr in [0xC470, 0xC47B, 0xC47D, 0xC47E]:
  orig = r(addr)
  tv = 0x00 if orig else 0xAA
  w(addr, tv); rb = r(addr); w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

# ============================================================
print("\nEXPERIMENT 12: C487/C489/C48A")
print("="*60)
d = rn(0xC480, 16)
print(f"  C480-8F: {' '.join(f'{b:02X}' for b in d)}")
for i in range(3):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  d2 = rn(0xC487, 3)
  print(f"  wr[{i}]: C487={d2[0]:02X} C488={d2[1]:02X} C489={d2[2]:02X}")
  reset_ce6e()

for addr in [0xC487, 0xC489, 0xC48A]:
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv); rb = r(addr); w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

# ============================================================
print("\nEXPERIMENT 13: CE6E after various size writes (no reset between)")
print("="*60)
reset_ce6e()
for sz in [512, 512, 1024, 2048, 4096, 8192, 16384]:
  ctrl.exec_ops([ScsiWriteOp(bytes(sz), lba=0)])
  ce6e = rn(0xCE6E, 2)
  ce10 = rn(0xCE10, 4)
  print(f"  {sz:5d}B: CE6E={ce6e.hex()} CE10={ce10.hex()}")
  reset_ce6e()

print("\nDone!")
