#!/usr/bin/env python3
"""Dump stock registers after a 4KB scsi_write."""
import sys, json
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

# Do 4KB write
ctrl.scsi_write(bytes([0xAA]*4096), lba=0)

def read8(addr):
  try: return ctrl.read(addr, 1)[0]
  except: return -1

ranges = [
  (0x0000, 0x0800), (0x0800, 0x1000),
  (0x9000, 0x9400), (0xB200, 0xB300), (0xC000, 0xD000),
]

result = {}
for start, end in ranges:
  print(f"Reading 0x{start:04X}-0x{end:04X}...", file=sys.stderr)
  for addr in range(start, end):
    v = read8(addr)
    if v > 0:
      result[f"0x{addr:04X}"] = v

with open("dump_stock_after4k.json", 'w') as f:
  json.dump(result, f, indent=2, sort_keys=True)
print(f"Wrote {len(result)} non-zero regs", file=sys.stderr)
