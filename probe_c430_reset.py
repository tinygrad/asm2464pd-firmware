#!/usr/bin/env python3
"""
After fresh reset, watch C430 transition on first scsi_write.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]

# Fresh boot
print(f"C430 after boot: 0x{r(0xC430):02X}")
print(f"C431: 0x{r(0xC431):02X}  C432: 0x{r(0xC432):02X}  C433: 0x{r(0xC433):02X}")

# First E4 reads - does that change it?
for i in range(3):
  print(f"  read #{i}: C430=0x{r(0xC430):02X}")

# First scsi_write
print("\nDoing first scsi_write (512 bytes, lba=0)...")
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
print(f"C430 after 1st write: 0x{r(0xC430):02X}")

# Second
ctrl.scsi_write(bytes([0xBB]*512), lba=0)
print(f"C430 after 2nd write: 0x{r(0xC430):02X}")

print("Done!")
