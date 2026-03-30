#!/usr/bin/env python3
"""
Stock firmware has MSC_CFG=0x00 and C430 latch triggers during scsi_write.
This means the C400 DMA engine IS being used by the stock firmware.

Track C400-C42D more carefully to understand how the stock firmware
configures and triggers C400 DMA during scsi_write.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp, ReadOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
print("="*60)
print("C400 descriptor state at boot (before any DMA)")
print("="*60)

d = rn(0xC400, 0x30)
print(f"  C400-C40F: {d[:16].hex()}")
print(f"  C410-C41F: {d[16:32].hex()}")
print(f"  C420-C42F: {d[32:48].hex()}")

# ============================================================
print("\n" + "="*60)
print("Interleaved reads of C400 area during scsi_write")
print("="*60)

# Read key C4xx regs before, between, and after scsi ops
results = ctrl.exec_ops([
  ReadOp(0xC400, 16),   # [0] C400-C40F before
  ReadOp(0xC420, 16),   # [1] C420-C42F before
  ReadOp(0xCE10, 4),    # [2] CE10 before
  ScsiWriteOp(bytes([0xAA]*4096), lba=0),
  ReadOp(0xC400, 16),   # [4] C400-C40F after
  ReadOp(0xC420, 16),   # [5] C420-C42F after
  ReadOp(0xCE10, 4),    # [6] CE10 after
  ReadOp(0xC430, 4),    # [7] C430 after
])

labels = {0: "C400 before", 1: "C420 before", 2: "CE10 before",
          4: "C400 after", 5: "C420 after", 6: "CE10 after", 7: "C430 after"}
for i, res in enumerate(results):
  if res is not None and i in labels:
    print(f"  {labels[i]}: {res.hex()}")

# cleanup
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("C427 (sector count) at various times")
print("="*60)

# Check C427 before/after writes of different sizes
for size in [512, 1024, 2048, 4096]:
  print(f"\n  --- {size}B scsi_write ---")
  c427_before = r(0xC427)
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  c427_after = r(0xC427)
  ce10 = rn(0xCE10, 4).hex()
  print(f"  C427 before=0x{c427_before:02X} after=0x{c427_after:02X} CE10={ce10}")
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("Full C420-C42F after different size writes")
print("="*60)

for size in [512, 4096, 16384, 65536]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  desc = rn(0xC420, 16)
  ce10 = rn(0xCE10, 4).hex()
  print(f"  {size:5d}B: C420={desc.hex()} CE10={ce10}")
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("C400-C406 arm/trigger state after scsi_write")
print("="*60)

ctrl.exec_ops([ScsiWriteOp(bytes(512), lba=0)])
for addr in [0xC400, 0xC401, 0xC402, 0xC404, 0xC406]:
  print(f"  0x{addr:04X}: 0x{r(addr):02X}")

ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("\n" + "="*60)
print("Can we manually set C427 and trigger C400 DMA for >1 sector?")
print("  Use a 4KB scsi_write, then re-trigger with C427=8")
print("="*60)

# First, do a 4KB scsi_write to get the DMA state right
ctrl.scsi_write(bytes([0x11]*4096), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")
print(f"  F800: {rn(0xF800, 4).hex()}")

# Now overwrite F000-F1FF with pattern via E5
for i in range(512):
  w(0xF000 + i, 0xEE)
print(f"  F000 after E5 overwrite: {rn(0xF000, 4).hex()}")

# Now write unique data to 0x7000 via E5
for i in range(4096):
  w(0x7000 + i, (i >> 8) & 0xFF)  # page number

# Set C427=8 (4KB = 8 sectors)
w(0xC427, 0x08)
# Trigger C400
w(0xC42A, 0x00)
w(0xC406, 0x01)

print(f"  F000 after C400 trigger: {rn(0xF000, 4).hex()}")
print(f"  F200 after C400 trigger: {rn(0xF200, 4).hex()}")
print(f"  F400 after C400 trigger: {rn(0xF400, 4).hex()}")
print(f"  F800 after C400 trigger: {rn(0xF800, 4).hex()}")
w(0xC42A, 0x01)

print("\nDone!")
