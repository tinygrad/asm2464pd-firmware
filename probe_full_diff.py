#!/usr/bin/env python3
"""
Snapshot EVERY register in C400-C4FF and CE00-CEFF before and after
scsi_write of different sizes. Find what the stock firmware sets
differently for 4KB vs 512B.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)
def reset():
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

def snap_all():
  """Snapshot C400-C4FF and CE00-CEFF."""
  data = {}
  for base in range(0xC400, 0xC500, 0x40):
    d = rn(base, 0x40)
    for i in range(0x40):
      data[base + i] = d[i]
  for base in range(0xCE00, 0xCF00, 0x40):
    d = rn(base, 0x40)
    for i in range(0x40):
      data[base + i] = d[i]
  return data

# ============================================================
print("Snapshot idle state...")
idle = snap_all()

# ============================================================
print("512B scsi_write...")
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*512), lba=0)])
after_512 = snap_all()
reset()

print("4KB scsi_write...")
ctrl.exec_ops([ScsiWriteOp(bytes([0xBB]*4096), lba=0)])
after_4k = snap_all()
reset()

print("16KB scsi_write...")
ctrl.exec_ops([ScsiWriteOp(bytes([0xCC]*16384), lba=0)])
after_16k = snap_all()
reset()

# ============================================================
print("\n" + "="*60)
print("Registers that DIFFER between 512B and 4KB writes")
print("="*60)

for addr in sorted(after_512.keys()):
  v512 = after_512[addr]
  v4k = after_4k[addr]
  if v512 != v4k:
    vidl = idle[addr]
    v16k = after_16k[addr]
    print(f"  0x{addr:04X}: idle=0x{vidl:02X} 512B=0x{v512:02X} 4KB=0x{v4k:02X} 16KB=0x{v16k:02X}")

# ============================================================
print("\n" + "="*60)
print("Registers that changed from IDLE (any write)")
print("="*60)

for addr in sorted(idle.keys()):
  vidl = idle[addr]
  v512 = after_512[addr]
  v4k = after_4k[addr]
  v16k = after_16k[addr]
  if vidl != v512 or vidl != v4k or vidl != v16k:
    print(f"  0x{addr:04X}: idle=0x{vidl:02X} 512B=0x{v512:02X} 4KB=0x{v4k:02X} 16KB=0x{v16k:02X}")

print("\nDone!")
