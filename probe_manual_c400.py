#!/usr/bin/env python3
"""
Can we manually trigger C400 DMA on stock firmware?
Stock firmware uses UAS SCSI, but can we also trigger C400
via E5 register writes after data is at 0x7000?

Plan:
  1. scsi_write to fill 0x7000 and get DMA state hot
  2. Overwrite F000 with known pattern (to distinguish new DMA)
  3. Manually set C427 and trigger C42A=0/C406=1
  4. Check if F000 gets the 0x7000 data
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
print("="*60)
print("TEST 1: scsi_write then manual C400 re-trigger")
print("="*60)

# scsi_write puts AA at F000
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
print(f"  F000 after scsi_write: {rn(0xF000, 4).hex()}")
print(f"  7000: {rn(0x7000, 4).hex()}")

# Overwrite F000 via E5 to clear it
w(0xF000, 0x00); w(0xF001, 0x00); w(0xF002, 0x00); w(0xF003, 0x00)
print(f"  F000 after clear: {rn(0xF000, 4).hex()}")

# Manually trigger C400 DMA — should re-copy 0x7000 -> F000
w(0xC427, 0x01)
w(0xC42A, 0x00)
w(0xC406, 0x01)
print(f"  F000 after manual C400 trigger: {rn(0xF000, 4).hex()}")
w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 2: scsi_write 4KB, clear F000, retrigger with C427=8")
print("="*60)

data = bytes([(i >> 8) & 0xFF for i in range(4096)])
ctrl.scsi_write(data, lba=0)
print(f"  F000: {rn(0xF000, 4).hex()} F800: {rn(0xF800, 4).hex()}")

# Clear F000-F003
w(0xF000, 0xDD); w(0xF001, 0xDD); w(0xF002, 0xDD); w(0xF003, 0xDD)
w(0xF800, 0xDD); w(0xF801, 0xDD)
print(f"  F000 cleared: {rn(0xF000, 4).hex()}")
print(f"  F800 cleared: {rn(0xF800, 4).hex()}")

# Retrigger with 8 sectors
w(0xC427, 0x08)
w(0xC42A, 0x00)
w(0xC406, 0x01)
print(f"  F000 after C427=8 trigger: {rn(0xF000, 4).hex()}")
print(f"  F200 after trigger: {rn(0xF200, 4).hex()}")
print(f"  F800 after trigger: {rn(0xF800, 4).hex()}")
w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 3: Set C427=2 after 512B write — does second sector appear?")
print("="*60)

# Write 1KB via scsi_write (2 sectors with unique data per sector)
data2 = bytes([0xBB]*512 + [0xCC]*512)
ctrl.scsi_write(data2, lba=0)
print(f"  F000: {rn(0xF000, 4).hex()} (expect BB)")
print(f"  F200: {rn(0xF200, 4).hex()} (expect CC)")

# Clear both
w(0xF000, 0x00); w(0xF001, 0x00); w(0xF002, 0x00); w(0xF003, 0x00)
w(0xF200, 0x00); w(0xF201, 0x00); w(0xF202, 0x00); w(0xF203, 0x00)

# Retrigger 1 sector only
w(0xC427, 0x01)
w(0xC42A, 0x00)
w(0xC406, 0x01)
print(f"  C427=1: F000={rn(0xF000, 4).hex()} F200={rn(0xF200, 4).hex()}")
w(0xC42A, 0x01)

# Clear again
w(0xF000, 0x00); w(0xF001, 0x00); w(0xF002, 0x00); w(0xF003, 0x00)

# Retrigger 2 sectors
w(0xC427, 0x02)
w(0xC42A, 0x00)
w(0xC406, 0x01)
print(f"  C427=2: F000={rn(0xF000, 4).hex()} F200={rn(0xF200, 4).hex()}")
w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 4: Does the descriptor (C420-C421) matter?")
print("="*60)

ctrl.scsi_write(bytes([0xEE]*4096), lba=0)
# Clear
for i in range(4):
  w(0xF000 + i, 0x00)
  w(0xF800 + i, 0x00)

# Try with stock descriptor values
print(f"  C420-C427 current: {rn(0xC420, 8).hex()}")

# Set C420=0, C421=0 (zero descriptor)
w(0xC420, 0x00); w(0xC421, 0x00)
w(0xC427, 0x08)
w(0xC42A, 0x00); w(0xC406, 0x01)
print(f"  C420=0,C421=0,C427=8: F000={rn(0xF000, 4).hex()} F800={rn(0xF800, 4).hex()}")
w(0xC42A, 0x01)

# Clear again
for i in range(4):
  w(0xF000 + i, 0x00)
  w(0xF800 + i, 0x00)

# Set C421=4 (as stock does for 4KB)
w(0xC420, 0x00); w(0xC421, 0x04)
w(0xC427, 0x08)
w(0xC42A, 0x00); w(0xC406, 0x01)
print(f"  C420=0,C421=4,C427=8: F000={rn(0xF000, 4).hex()} F800={rn(0xF800, 4).hex()}")
w(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 5: Multiple manual triggers — does pointer advance?")
print("="*60)

ctrl.scsi_write(bytes([0xFF]*4096), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")

# Clear F000, retrigger twice
w(0xF000, 0x00); w(0xF001, 0x00)
print(f"  CE10 before retrigger: {rn(0xCE10, 4).hex()}")

w(0xC427, 0x01)
w(0xC42A, 0x00); w(0xC406, 0x01)
print(f"  After 1st trigger: F000={rn(0xF000, 4).hex()} CE10={rn(0xCE10, 4).hex()}")
w(0xC42A, 0x01)

w(0xF000, 0x00); w(0xF001, 0x00)
w(0xC42A, 0x00); w(0xC406, 0x01)
print(f"  After 2nd trigger: F000={rn(0xF000, 4).hex()} CE10={rn(0xCE10, 4).hex()}")
w(0xC42A, 0x01)

print("\nDone!")
