#!/usr/bin/env python3
"""
C5xx registers change with transfer size on stock firmware.
C518/C519: 0x01 for 1KB, 0x02 for 4KB.
Get full picture of C5xx across all sizes.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)
def reset():
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("C500-C54F after different size writes:")
print(f"{'SIZE':>6s}  C500-C50F                          C510-C51F                          C520-C52F                          C530-C53F                          C540-C54F")

for size in [512, 1024, 2048, 4096, 8192, 16384, 32768, 65536]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  chunks = []
  for base in range(0xC500, 0xC550, 0x10):
    d = rn(base, 0x10)
    chunks.append(' '.join(f'{b:02X}' for b in d))
  reset()
  
  # Just show non-zero bytes from each chunk
  print(f"{size:6d}  ", end="")
  for base_idx, base in enumerate(range(0xC500, 0xC550, 0x10)):
    d = rn(base, 0x10) if base_idx == 0 else bytes(16)  # already read above
  
  # Re-read properly
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  d = rn(0xC500, 0x50)
  nz = []
  for i in range(0x50):
    if d[i] != 0:
      nz.append(f"C5{i:02X}={d[i]:02X}")
  print(f"{size:6d}B: {' '.join(nz)}")
  reset()
