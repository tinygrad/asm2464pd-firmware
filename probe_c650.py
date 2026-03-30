#!/usr/bin/env python3
"""
Probe C650-C660 and C750-C760 areas. C656=0x20 looks like PCI address byte.
Check these across different write sizes.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()
def rn(addr, n): return ctrl.read(addr, n)
def reset():
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("C640-C670 and C740-C770 after different size writes:")
for size in [512, 1024, 4096, 16384, 65536]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  d6 = rn(0xC640, 0x30)
  d7 = rn(0xC740, 0x30)
  
  nz6 = [(i, d6[i]) for i in range(0x30) if d6[i] != 0]
  nz7 = [(i, d7[i]) for i in range(0x30) if d7[i] != 0]
  
  print(f"\n  {size}B:")
  print(f"    C640-C66F: {' '.join(f'C6{0x40+i:02X}={v:02X}' for i,v in nz6)}")
  print(f"    C740-C76F: {' '.join(f'C7{0x40+i:02X}={v:02X}' for i,v in nz7)}")
  reset()

# Specifically look at C654-C659 area
print("\n\nC650-C65F detail:")
for size in [512, 1024, 4096, 65536]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  d = rn(0xC650, 16)
  print(f"  {size:5d}B: {' '.join(f'{b:02X}' for b in d)}")
  reset()
