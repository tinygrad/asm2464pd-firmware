#!/usr/bin/env python3
"""
On STOCK firmware: dump ALL regs after 1KB write vs 4KB write.
The difference tells us exactly which registers encode the transfer size/address.
"""
import sys, json
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def dump_all():
  result = {}
  for start, end in [(0x0000, 0x0100), (0xC000, 0xD000)]:
    for addr in range(start, end, 0x40):
      d = ctrl.read(addr, 0x40)
      for i in range(0x40):
        result[addr + i] = d[i]
  return result

# Do 1KB write
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*1024), lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
after_1k = dump_all()

# Do 4KB write
ctrl.exec_ops([ScsiWriteOp(bytes([0xBB]*4096), lba=0)])
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
after_4k = dump_all()

print("Registers different between 1KB and 4KB scsi_write:")
print(f"{'ADDR':>8s}  {'1KB':>5s}  {'4KB':>5s}")
print("-" * 25)

for addr in sorted(after_1k.keys()):
  v1 = after_1k[addr]
  v4 = after_4k[addr]
  if v1 != v4:
    print(f"  0x{addr:04X}   0x{v1:02X}   0x{v4:02X}")
