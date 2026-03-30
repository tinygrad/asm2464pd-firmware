#!/usr/bin/env python3
"""
Deep probe of DMA registers. Systematic experiments on all undocumented areas.
"""
import sys, struct, time
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp, ReadOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def snap_ce6e(label):
  ce10 = rn(0xCE10, 4)
  ce6e = rn(0xCE6E, 2)
  print(f"  {label}: CE10={ce10.hex()} CE6E={ce6e.hex()}")

def reset_ce6e():
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("="*60)
print("EXPERIMENT 1: CE6E/CE6F precise behavior")
print("  What value of CE6E triggers the pointer move?")
print("="*60)

# Write CE6F=1 first, then various CE6E values
for ce6e_val in [0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x80]:
  reset_ce6e()
  w(0xCE6F, 0x01)  # set low byte first
  w(0xCE6E, ce6e_val)  # then write high byte (trigger?)
  snap_ce6e(f"CE6F=01 then CE6E={ce6e_val:02X}")

reset_ce6e()

# Write CE6E first, then CE6F
print("\n  Reverse order: CE6E first, then CE6F")
for ce6f_val in [0x00, 0x01, 0x02, 0x04]:
  reset_ce6e()
  w(0xCE6E, 0x00)
  w(0xCE6F, ce6f_val)
  snap_ce6e(f"CE6E=00 then CE6F={ce6f_val:02X}")

reset_ce6e()

# After a real scsi_write, what does CE6E read?
print("\n  CE6E after real scsi_write:")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
ce6e_after = rn(0xCE6E, 2)
snap_ce6e("after 512B write")
ctrl.exec_ops([ScsiWriteOp(bytes(4096), lba=0)])
snap_ce6e("after 4KB write")
ctrl.exec_ops([ScsiWriteOp(bytes(16384), lba=0)])
snap_ce6e("after 16KB write")
reset_ce6e()

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 2: C478/C479 - what do they count?")
print("="*60)

# Read them with no activity
v = rn(0xC478, 2)
print(f"  Baseline: C478={v[0]:02X} C479={v[1]:02X}")

# Do 1 E4 read (1 byte)
_ = r(0xC412)
v = rn(0xC478, 2)
print(f"  After 1 E4 read: C478={v[0]:02X} C479={v[1]:02X}")

# Do 1 E5 write
w(0xF000, 0x00)
v = rn(0xC478, 2)
print(f"  After 1 E5 write: C478={v[0]:02X} C479={v[1]:02X}")

# Do 5 E4 reads
for _ in range(5): r(0xC412)
v = rn(0xC478, 2)
print(f"  After 5 more E4 reads: C478={v[0]:02X} C479={v[1]:02X}")

# Do a scsi_write
v_before = rn(0xC478, 2)
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
v_after = rn(0xC478, 2)
print(f"  Before scsi_write: C478={v_before[0]:02X} C479={v_before[1]:02X}")
print(f"  After scsi_write:  C478={v_after[0]:02X} C479={v_after[1]:02X}")
print(f"  Delta C478={v_after[0]-v_before[0]} C479={v_after[1]-v_before[1]}")
reset_ce6e()

# Writability?
orig = rn(0xC478, 2)
w(0xC478, 0x00); w(0xC479, 0x00)
after = rn(0xC478, 2)
print(f"  Writable? orig={orig.hex()} wrote=0000 read={after.hex()}")

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 3: C4B0-C4B5 - SCSI command tracking")
print("="*60)

d = rn(0xC4B0, 6)
print(f"  Baseline: {' '.join(f'{b:02X}' for b in d)}")

# Writability
for i in range(6):
  addr = 0xC4B0 + i
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

# Track changes across multiple writes
print("\n  Tracking across writes:")
for i in range(5):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  d = rn(0xC4B0, 6)
  print(f"  [{i}] {' '.join(f'{b:02X}' for b in d)}")
  reset_ce6e()

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 4: C450-C46F queue status (appeared after scsi_write)")
print("="*60)

# Dump current state
d = rn(0xC450, 0x20)
print(f"  C450-C46F:")
for off in range(0, len(d), 16):
  addr = 0xC450 + off
  chunk = d[off:off+16]
  nz = [(off+i, chunk[i]) for i in range(16) if chunk[i] != 0]
  for i, v in nz:
    print(f"    0x{0xC450+i:04X} = 0x{v:02X}")

# Are these related to how many scsi_writes we've done?
print("\n  After 3 more writes:")
for i in range(3):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  reset_ce6e()
d2 = rn(0xC450, 0x20)
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
for i, old, new in changed:
  print(f"    0x{0xC450+i:04X}: {old:02X} -> {new:02X}")
if not changed:
  print("    (no changes)")

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 5: CE02 volatile register")
print("="*60)

# CE02 was volatile (toggled 00<->01 between reads)
vals = []
for i in range(20):
  vals.append(r(0xCE02))
print(f"  CE02 x20: {' '.join(f'{v:02X}' for v in vals)}")
print(f"  Pattern: alternates={len(set(vals))>1}")

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 6: CE20-CE22 constant pattern (50 CE 20)")
print("="*60)

d = rn(0xCE20, 8)
print(f"  CE20-CE27: {' '.join(f'{b:02X}' for b in d)}")
print(f"  CE20-CE21 as LE u16: 0x{d[1]:02X}{d[0]:02X} = {struct.unpack('<H', bytes(d[:2]))[0]}")
print(f"  Looks like a pointer to 0xCE50? (LE: 0x{d[1]:02X}{d[0]:02X})")

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 7: CE3C-CE3F area")
print("="*60)

d = rn(0xCE3C, 4)
print(f"  CE3C-CE3F: {' '.join(f'{b:02X}' for b in d)}")
# CE3C=01 R/O, CE3D=03 R/W, CE3E=00 R/W, CE3F=04 R/W (from earlier)

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 8: CE44-CE49 repeated pattern")
print("="*60)

d = rn(0xCE44, 6)
print(f"  CE44-CE49: {' '.join(f'{b:02X}' for b in d)}")
print(f"  CE44-45 = 0x{d[0]:02X}{d[1]:02X} (0x0550 = 1360)")
print(f"  CE46-48 = 0x{d[2]:02X}{d[3]:02X}{d[4]:02X}")
# Do they change after scsi_write?
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xCE44, 6)
print(f"  After write: {' '.join(f'{b:02X}' for b in d2)}")
reset_ce6e()
# After reset?
ctrl.exec_ops([ScsiWriteOp(bytes(4096), lba=0)])
d3 = rn(0xCE44, 6)
print(f"  After 4KB write: {' '.join(f'{b:02X}' for b in d3)}")
reset_ce6e()

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 9: C412-C419 control/status detail")
print("="*60)

d = rn(0xC410, 16)
print(f"  C410-C41F: {' '.join(f'{b:02X}' for b in d)}")

# Behavior across writes
print("  After scsi_write:")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xC410, 16)
print(f"  C410-C41F: {' '.join(f'{b:02X}' for b in d2)}")
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
for i, old, new in changed:
  print(f"    0x{0xC410+i:04X}: {old:02X} -> {new:02X}")
reset_ce6e()

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 10: CE86-CE8F handshake area detail")
print("="*60)

d = rn(0xCE86, 10)
print(f"  CE86-CE8F: {' '.join(f'{b:02X}' for b in d)}")

# Read CE89 multiple times (DMA state machine)
print("  CE89 x10:")
for i in range(10):
  print(f"    [{i}] CE89=0x{r(0xCE89):02X}")

# After scsi_write
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
d2 = rn(0xCE86, 10)
print(f"  After write: {' '.join(f'{b:02X}' for b in d2)}")
changed = [(i, d[i], d2[i]) for i in range(len(d)) if d[i] != d2[i]]
for i, old, new in changed:
  print(f"    0x{0xCE86+i:04X}: {old:02X} -> {new:02X}")
reset_ce6e()

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 11: C470-C47F detail")
print("="*60)

d = rn(0xC470, 16)
print(f"  C470-C47F: {' '.join(f'{b:02X}' for b in d)}")

# C470 went 0x00 -> 0xC0 after scsi_write. What bits?
print(f"  C470=0x{d[0]:02X} ({d[0]:08b})")
print(f"  C471=0x{d[1]:02X} ({d[1]:08b}) [QUEUE_BUSY]")
print(f"  C473=0x{d[3]:02X} ({d[3]:08b}) [LINK_PARAM]")
print(f"  C47E=0x{d[14]:02X} ({d[14]:08b})")

# Writability of C470
orig = r(0xC470)
w(0xC470, 0x00)
rb = r(0xC470)
w(0xC470, orig)
print(f"  C470 write test: orig=0x{orig:02X} wrote=0x00 read=0x{rb:02X} -> {'R/O' if rb == orig else 'R/W' if rb == 0 else f'PARTIAL(0x{rb:02X})'}")

# C47B, C47D, C47E
for addr in [0xC47B, 0xC47D, 0xC47E]:
  orig = r(addr)
  tv = 0x00 if orig else 0xAA
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

# ============================================================
print("\n" + "="*60)
print("EXPERIMENT 12: C487/C489/C48A")
print("="*60)

d = rn(0xC480, 16)
print(f"  C480-C48F: {' '.join(f'{b:02X}' for b in d)}")

# C487 changed 01->04 after write. C489 changed 01->00.
# More writes
for i in range(3):
  ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
  d2 = rn(0xC487, 3)
  print(f"  After write {i}: C487={d2[0]:02X} C488={d2[1]:02X} C489={d2[2]:02X}")
  reset_ce6e()

# Writability
for addr in [0xC487, 0xC489, 0xC48A]:
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)
  rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
  print(f"  0x{addr:04X}: orig=0x{orig:02X} -> {rw}")

print("\nDone!")
