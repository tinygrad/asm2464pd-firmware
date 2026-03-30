#!/usr/bin/env python3
"""
Test large scsi_write and see how much SRAM gets filled.
If tinygrad writes 64KB via scsi_write, where does it all go?
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)

# Write 4KB of unique pattern
print("4KB scsi_write...")
data = bytes([i & 0xFF for i in range(4096)])
ctrl.scsi_write(data, lba=0)

# Check SRAM at various offsets
print("SRAM readback:")
for off in range(0, 4096, 256):
  addr = 0xF000 + off
  d = rn(addr, 4)
  expected = bytes([off & 0xFF, (off+1) & 0xFF, (off+2) & 0xFF, (off+3) & 0xFF])
  match = d == expected
  print(f"  0x{addr:04X}: {d.hex()} expected={expected.hex()} {'OK' if match else 'FAIL'}")

# The F000 window is only 4KB (F000-FFFF). What about data beyond 4KB?
# The SRAM is at PCI 0x200000. F000-FFFF maps to PCI 0x200000-0x200FFF.
# What about PCI 0x201000+? That's beyond the F000 window.
# Can we read it via the 0x8000 window? Or via PCIe TLP?

print("\n8000 window check (different SRAM region?):")
for off in range(0, 256, 64):
  d = rn(0x8000 + off, 4)
  print(f"  0x{0x8000+off:04X}: {d.hex()}")

# 64KB write
print("\n64KB scsi_write...")
big_data = bytes([(i >> 8) & 0xFF for i in range(65536)])  # each 256-byte page has same byte
ctrl.scsi_write(big_data, lba=0)

# Check F000 window
print("F000 window after 64KB write:")
for off in [0, 0x100, 0x200, 0x400, 0x800, 0xF00]:
  d = rn(0xF000 + off, 4)
  page = off >> 8
  expected = bytes([page]*4)
  print(f"  F{off:03X}: {d.hex()} (page {page}, expect {expected.hex()}) {'OK' if d==expected else 'FAIL'}")

print("\nDone!")
