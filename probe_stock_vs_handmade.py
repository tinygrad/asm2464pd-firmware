#!/usr/bin/env python3
"""
On STOCK firmware: dump EVERY register in C400-C4FF, CE00-CEFF, 
C800-C8FF, 9000-90FF, 9100-91FF after a successful 4KB scsi_write.
Compare with handmade firmware's register state.
Something configures the DMA engine that we're missing.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def rn(addr, n): return ctrl.read(addr, n)

# Do a 4KB scsi_write first
ctrl.scsi_write(bytes([0xAA]*4096), lba=0)

print("Stock firmware register dump after 4KB scsi_write:")
print("(only non-zero/non-0x55 shown)")
print()

for base_name, base_start, base_end in [
    ("C400", 0xC400, 0xC500),
    ("C800", 0xC800, 0xC900),
    ("CE00", 0xCE00, 0xCF00),
    ("9000", 0x9000, 0x9100),
    ("9100", 0x9100, 0x9200),
    ("9200", 0x9200, 0x9300),
    ("9300", 0x9300, 0x9400),
]:
    print(f"--- {base_name}-{base_name[0]}{(base_end-1)&0xFFF:03X} ---")
    for addr in range(base_start, base_end, 0x40):
        d = rn(addr, 0x40)
        for i in range(0x40):
            v = d[i]
            if v != 0x00 and v != 0x55:
                print(f"  0x{addr+i:04X} = 0x{v:02X}")
    print()
