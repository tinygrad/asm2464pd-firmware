#!/usr/bin/env python3
"""
Can we move CE10-CE13 by writing to it?
Previous test showed writing didn't stick, but maybe we need
to write in a specific order or use CE6E first.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def snap(label):
  ce10 = rn(0xCE10, 4)
  print(f"  {label}: CE10={ce10.hex()}")

# ============================================================
print("TEST 1: Direct write to CE10-CE13")
snap("baseline")

w(0xCE10, 0x00); w(0xCE11, 0x20); w(0xCE12, 0x10); w(0xCE13, 0x00)
snap("after write 0x00201000")

w(0xCE10, 0x00); w(0xCE11, 0x30); w(0xCE12, 0x00); w(0xCE13, 0x00)
snap("after write 0x00300000")

# ============================================================
print("\nTEST 2: Write CE6E first, then CE10")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after CE6E clear")

w(0xCE10, 0x00); w(0xCE11, 0x20); w(0xCE12, 0x10); w(0xCE13, 0x00)
snap("after write 0x00201000")

# ============================================================
print("\nTEST 3: Write via exec_ops (SCSI E5)")
ctrl.exec_ops([
  WriteOp(0xCE10, b'\x00'),
  WriteOp(0xCE11, b'\x20'),
  WriteOp(0xCE12, b'\x10'),
  WriteOp(0xCE13, b'\x00'),
])
snap("after exec_ops write 0x00201000")

# ============================================================
print("\nTEST 4: Write all 4 bytes at once")
ctrl.exec_ops([WriteOp(0xCE10, b'\x00\x21\x00\x00', ignore_cache=True)])
snap("after bulk write 0x00210000")

# ============================================================
print("\nTEST 5: Do scsi_write, then try to move pointer before CE6E reset")
ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
snap("after ScsiWriteOp (pointer advanced)")

# Now try to write it
w(0xCE10, 0x00); w(0xCE11, 0x20); w(0xCE12, 0x00); w(0xCE13, 0x00)
snap("after manual reset attempt")

# CE6E reset
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after CE6E reset")

# ============================================================
print("\nTEST 6: Write CE6E=something other than 0, then check")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x01\x00', ignore_cache=True)])
snap("after CE6E=0x0100")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x01', ignore_cache=True)])
snap("after CE6E=0x0001")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after CE6E=0x0000")

# ============================================================
print("\nTEST 7: scsi_write then move pointer, then scsi_write again")
# First write - data at F000
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*512), lba=0)])
snap("after 1st write")
d = rn(0xF000, 4)
print(f"  F000: {d.hex()}")

# DON'T reset CE6E - pointer should be at 0x200200
# Do another write - where does it go?
ctrl.exec_ops([ScsiWriteOp(bytes([0xBB]*512), lba=0)])
snap("after 2nd write (no CE6E reset)")
d0 = rn(0xF000, 4)
d2 = rn(0xF200, 4)
d4 = rn(0xF400, 4)
print(f"  F000: {d0.hex()}  F200: {d2.hex()}  F400: {d4.hex()}")

# Now reset
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after CE6E reset")

print("\nDone!")
