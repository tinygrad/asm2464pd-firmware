#!/usr/bin/env python3
"""
Clean test: do two scsi_writes without CE6E reset between them.
See if second write appends after first in SRAM.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)
def ce10(): return rn(0xCE10, 4).hex()

# ============================================================
print("TEST 1: Two 512B writes, no CE6E reset between")
print("="*60)

# First write: AA
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*512), lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
# Skip CE6E reset!
print(f"  After 1st write: CE10={ce10()}")

# Second write: BB
ctrl.exec_ops([ScsiWriteOp(bytes([0xBB]*512), lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
print(f"  After 2nd write: CE10={ce10()}")

# Check SRAM
print("\n  SRAM:")
for off in range(0, 0x1000, 0x100):
  d = rn(0xF000 + off, 4)
  if d != bytes([0x55]*4):
    print(f"    F{off:03X}: {d.hex()}")

# Now reset
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
print(f"\n  After CE6E reset: CE10={ce10()}")

# ============================================================
print("\n\nTEST 2: 4KB write then check full SRAM window")
print("="*60)

# 4KB with unique pages
data = bytes()
for page in range(16):  # 16 x 256 = 4096
  data += bytes([page]*256)

ctrl.scsi_write(data, lba=0)

print("  SRAM after 4KB scsi_write:")
for off in range(0, 0x1000, 0x100):
  d = rn(0xF000 + off, 1)
  print(f"    F{off:03X}: 0x{d[0]:02X} (expect 0x{off//256:02X})")

# ============================================================
print("\n\nTEST 3: Two separate 4KB writes without CE6E reset")
print("="*60)

# First 4KB: pages 0x10-0x1F
data1 = bytes()
for page in range(16):
  data1 += bytes([0x10 + page]*256)
ctrl.exec_ops([ScsiWriteOp(data1, lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
print(f"  After 1st 4KB: CE10={ce10()}")

# NO CE6E reset — second 4KB: pages 0x20-0x2F  
data2 = bytes()
for page in range(16):
  data2 += bytes([0x20 + page]*256)
ctrl.exec_ops([ScsiWriteOp(data2, lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
print(f"  After 2nd 4KB: CE10={ce10()}")

# Check: does F000 show first or second write?
print("\n  SRAM readback:")
for off in range(0, 0x1000, 0x100):
  d = rn(0xF000 + off, 1)
  # First write was 0x10+page, second was 0x20+page
  is_first = d[0] == 0x10 + (off // 256)
  is_second = d[0] == 0x20 + (off // 256)
  marker = " <1st" if is_first else " <2nd" if is_second else ""
  print(f"    F{off:03X}: 0x{d[0]:02X}{marker}")

ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("\nDone!")
