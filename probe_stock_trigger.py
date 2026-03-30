#!/usr/bin/env python3
"""
On stock firmware: which register ACTUALLY triggers the DMA?
Read C400-C40F and CE00-CE0F during/after scsi_write.
Also: try triggering C8B8 (internal DMA engine) instead of C406.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp, ReadOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)

# Check what C400-C40F look like before/after
print("Before scsi_write:")
print(f"  C400-0F: {rn(0xC400, 16).hex()}")
print(f"  C8B0-BF: {rn(0xC8B0, 16).hex()}")
print(f"  C8D0-DF: {rn(0xC8D0, 16).hex()}")

ctrl.scsi_write(bytes([0xAA]*4096), lba=0)

print("\nAfter 4KB scsi_write:")
print(f"  C400-0F: {rn(0xC400, 16).hex()}")
print(f"  C8B0-BF: {rn(0xC8B0, 16).hex()}")
print(f"  C8D0-DF: {rn(0xC8D0, 16).hex()}")

# Interleave reads during scsi_write
print("\nInterleaved reads:")
results = ctrl.exec_ops([
  ReadOp(0xC400, 8),
  ReadOp(0xC8B0, 8),
  ReadOp(0xC8D0, 8),
  ScsiWriteOp(bytes([0xBB]*4096), lba=0),
  ReadOp(0xC400, 8),
  ReadOp(0xC8B0, 8),
  ReadOp(0xC8D0, 8),
])
labels = ["C400 pre", "C8B0 pre", "C8D0 pre", None, "C400 post", "C8B0 post", "C8D0 post"]
for i, res in enumerate(results):
  if res is not None and labels[i]:
    print(f"  {labels[i]}: {res.hex()}")

ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("\nDone!")
