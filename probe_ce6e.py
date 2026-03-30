#!/usr/bin/env python3
"""
CE6E/CE6F controls the SRAM write pointer CE10-CE13.
Probe the exact relationship.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, WriteOp

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def snap(label):
  ce10 = rn(0xCE10, 4)
  ce6e = rn(0xCE6E, 2)
  print(f"  {label}: CE10={ce10.hex()} CE6E={ce6e.hex()}")

def reset():
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("TEST 1: CE6F values 0-8")
print("="*60)
reset()
snap("baseline")

for val in range(9):
  w(0xCE6F, val)
  snap(f"CE6F={val}")

reset()
snap("after reset")

# ============================================================
print("\nTEST 2: CE6E (high byte) values")
print("="*60)
reset()
for val in [0x01, 0x02, 0x03, 0x04, 0x10, 0xFF]:
  w(0xCE6E, val)
  snap(f"CE6E={val:02X}")
  reset()

# ============================================================
print("\nTEST 3: CE6F incremental - does it accumulate?")
print("="*60)
reset()
snap("start")
w(0xCE6F, 0x01)
snap("CE6F=1 (1st)")
w(0xCE6F, 0x01)
snap("CE6F=1 (2nd)")
w(0xCE6F, 0x01)
snap("CE6F=1 (3rd)")
w(0xCE6F, 0x01)
snap("CE6F=1 (4th)")

# ============================================================
print("\nTEST 4: CE6F=2, CE6F=4, CE6F=8")
print("="*60)
reset()
snap("start")
w(0xCE6F, 0x02)
snap("CE6F=2")
reset()
w(0xCE6F, 0x04)
snap("CE6F=4")
reset()
w(0xCE6F, 0x08)
snap("CE6F=8")
reset()
w(0xCE6F, 0x10)
snap("CE6F=0x10")
reset()
w(0xCE6F, 0x20)
snap("CE6F=0x20")
reset()
w(0xCE6F, 0x40)
snap("CE6F=0x40")
reset()
w(0xCE6F, 0x80)
snap("CE6F=0x80")
reset()

# ============================================================
print("\nTEST 5: CE6F=0xFF")
print("="*60)
w(0xCE6F, 0xFF)
snap("CE6F=0xFF")
reset()
snap("after reset")

# ============================================================
print("\nTEST 6: Write both CE6E+CE6F together")
print("="*60)
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x02', ignore_cache=True)])
snap("CE6E=0x0002")
reset()
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x04', ignore_cache=True)])
snap("CE6E=0x0004")
reset()
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x08', ignore_cache=True)])
snap("CE6E=0x0008")
reset()
ctrl.exec_ops([WriteOp(0xCE6E, b'\x03\x00', ignore_cache=True)])
snap("CE6E=0x0300")
reset()

print("\nDone!")
