#!/usr/bin/env python3
"""
Probe DMA registers (0xC400-0xC4FF, 0xCE00-0xCEFF) before and after scsi_write.
Uses E4/E5 vendor SCSI commands on stock firmware.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_dma_regs.py
"""
import sys, os, random, time
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

def read_reg(ctrl, addr):
  """Read single byte via E4."""
  return ctrl.read(addr, 1)[0]

def read_range(ctrl, start, length):
  """Read a range of bytes."""
  return ctrl.read(start, length)

def dump_regs(ctrl, name, ranges):
  """Dump register ranges with labels."""
  print(f"\n{'='*60}")
  print(f"  {name}")
  print(f"{'='*60}")
  for (start, end, label) in ranges:
    data = read_range(ctrl, start, end - start)
    print(f"\n  --- {label} (0x{start:04X}-0x{end-1:04X}) ---")
    for offset in range(0, len(data), 16):
      addr = start + offset
      chunk = data[offset:offset+16]
      hexstr = ' '.join(f'{b:02X}' for b in chunk)
      # Mark non-zero bytes
      markers = ''.join('*' if b != 0 else '.' for b in chunk)
      print(f"  0x{addr:04X}: {hexstr:<48s}  {markers}")

# Register ranges to probe
DMA_RANGES = [
  (0xC400, 0xC410, "DMA Arm/Trigger"),
  (0xC410, 0xC420, "DMA Control/Status"),
  (0xC420, 0xC430, "DMA Descriptor + Doorbell/MSC"),
  (0xC430, 0xC450, "NVMe PRP/Init/Queue"),
  (0xC450, 0xC480, "NVMe Queue Status/Busy"),
  (0xC480, 0xC4B0, "NVMe DMA Extended"),
  (0xC4B0, 0xC500, "NVMe SCSI Cmd/DMA Ctrl"),
]

SCSI_DMA_RANGES = [
  (0xCE00, 0xCE20, "SCSI DMA Ctrl + SRAM Ptr"),
  (0xCE20, 0xCE40, "SCSI DMA Config"),
  (0xCE40, 0xCE60, "SCSI DMA Params/Xfer/Compl"),
  (0xCE60, 0xCE80, "SCSI DMA Status/Addr"),
  (0xCE80, 0xCEA0, "SCSI Buf/Flow/Handshake"),
]

ALL_RANGES = DMA_RANGES + SCSI_DMA_RANGES

print("Connecting via ASM24Controller...")
ctrl = ASM24Controller()
print("Connected!")

# Phase 1: Baseline (idle state)
dump_regs(ctrl, "PHASE 1: IDLE BASELINE", ALL_RANGES)

# Phase 2: Read 0xF000 baseline
print(f"\n\n--- Reading 0xF000 baseline ---")
before = ctrl.read(0xF000, 32)
print(f"  0xF000: {before.hex()}")

# Phase 3: Snapshot just before scsi_write
dump_regs(ctrl, "PHASE 2: PRE-SCSI_WRITE (after read)", ALL_RANGES)

# Phase 4: Do the scsi_write
print(f"\n\n{'='*60}")
print(f"  PERFORMING scsi_write(512 bytes, lba=0)")
print(f"{'='*60}")
test_data = bytes(random.getrandbits(8) for _ in range(512))
print(f"  first 16: {test_data[:16].hex()}")
ctrl.scsi_write(test_data, lba=0)
print("  Done!")

# Phase 5: Snapshot after scsi_write
dump_regs(ctrl, "PHASE 3: POST-SCSI_WRITE", ALL_RANGES)

# Phase 6: Verify data arrived
print(f"\n\n--- Verifying 0xF000 ---")
after = ctrl.read(0xF000, 64)
print(f"  0xF000: {after.hex()}")
if after[:16] == test_data[:16]:
  print("  MATCH!")
else:
  print("  NO MATCH - checking other locations...")
  for addr in [0x7000, 0x8000]:
    d = ctrl.read(addr, 16)
    print(f"  0x{addr:04X}: {d.hex()}")

# Phase 7: Do a second scsi_write to see what changes
print(f"\n\n{'='*60}")
print(f"  PERFORMING second scsi_write(512 bytes, lba=0)")
print(f"{'='*60}")
test_data2 = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test_data2, lba=0)
print("  Done!")

dump_regs(ctrl, "PHASE 4: POST-SECOND-SCSI_WRITE", ALL_RANGES)

print("\n\nDone!")
