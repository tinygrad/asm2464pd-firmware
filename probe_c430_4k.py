#!/usr/bin/env python3
"""
Fresh reset, then 4KB scsi_write. Read C430 between each bulk.
Use exec_ops to interleave reads.
"""
import sys, struct
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ReadOp, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)

print(f"C430 before: 0x{r(0xC430):02X}")
print(f"CE10-13: {rn(0xCE10,4).hex()}")

print("\nDoing 4KB scsi_write (8 sectors, lba=0)...")
data = bytes(range(256)) * 16  # 4096 bytes, repeating pattern
ctrl.scsi_write(data, lba=0)

print(f"C430 after: 0x{r(0xC430):02X}")
print(f"CE10-13: {rn(0xCE10,4).hex()}")

# Read back from SRAM window to see where data went
print("\nSRAM readback (F000-FFFF):")
for off in range(0, 4096, 512):
  addr = 0xF000 + off
  d = rn(addr, 16)
  print(f"  0x{addr:04X}: {d.hex()}")

print("\nDone!")
