#!/usr/bin/env python3
"""
What controls the SCSI DMA target address?

Candidates:
  - LBA field in SCSI WRITE(16) CDB
  - CE10-CE13 (BE SRAM write pointer)
  - CE76-CE79 (LE buf addr)

Test each by writing unique patterns and checking where they land.
"""
import sys, struct
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ReadOp, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def dump_sram(label, start=0xF000, length=2048):
  print(f"\n  {label} - SRAM readback:")
  for off in range(0, length, 512):
    addr = start + off
    d = rn(addr, 16)
    print(f"    0x{addr:04X}: {d.hex()}")

def dump_ptrs(label):
  ce10 = rn(0xCE10, 4)
  ce76 = rn(0xCE76, 4)
  print(f"  {label}: CE10={ce10.hex()} CE76={ce76.hex()}")

# ============================================================
# Test 1: lba=0 -> where does data go?
# ============================================================
print("="*60)
print("TEST 1: lba=0")
print("="*60)
dump_ptrs("Before")
ctrl.scsi_write(bytes([0x11]*512), lba=0)
dump_ptrs("After")
dump_sram("lba=0 result")

# ============================================================
# Test 2: lba=1 -> does it shift by 512?
# ============================================================
print("\n" + "="*60)
print("TEST 2: lba=1")
print("="*60)
dump_ptrs("Before")
ctrl.scsi_write(bytes([0x22]*512), lba=1)
dump_ptrs("After")
dump_sram("lba=1 result")

# ============================================================
# Test 3: lba=2
# ============================================================
print("\n" + "="*60)
print("TEST 3: lba=2")
print("="*60)
dump_ptrs("Before")
ctrl.scsi_write(bytes([0x33]*512), lba=2)
dump_ptrs("After")
dump_sram("lba=2 result")

# ============================================================
# Test 4: lba=0 again - does it overwrite F000?
# ============================================================
print("\n" + "="*60)
print("TEST 4: lba=0 again (overwrite test)")
print("="*60)
ctrl.scsi_write(bytes([0x44]*512), lba=0)
dump_sram("lba=0 overwrite result")

# ============================================================
# Test 5: manually set CE10-13 to 0x00200800, then lba=0
# Does CE10 or LBA win?
# ============================================================
print("\n" + "="*60)
print("TEST 5: Manual CE10 = 0x00200800, then lba=0")
print("="*60)
w(0xCE10, 0x00)
w(0xCE11, 0x20)
w(0xCE12, 0x08)
w(0xCE13, 0x00)
dump_ptrs("After manual CE10 set")

ctrl.scsi_write(bytes([0x55]*512), lba=0)
dump_ptrs("After write")

# Check both F000 (0x200000) and F800 (0x200800)
d_f000 = rn(0xF000, 4)
d_f800 = rn(0xF800, 4)
print(f"  F000: {d_f000.hex()} (0x200000)")
print(f"  F800: {d_f800.hex()} (0x200800)")

# ============================================================
# Test 6: manually set CE76-79 to 0x00200C00, then lba=0
# ============================================================
print("\n" + "="*60)
print("TEST 6: Manual CE76 = 0x00200C00, CE10 = default, lba=0")
print("="*60)
# Reset CE10 to default
w(0xCE10, 0x00)
w(0xCE11, 0x20)
w(0xCE12, 0x00)
w(0xCE13, 0x00)
# Set CE76 to 0x200C00
w(0xCE76, 0x00)
w(0xCE77, 0x0C)
w(0xCE78, 0x20)
w(0xCE79, 0x00)
dump_ptrs("After manual CE76 set")

ctrl.scsi_write(bytes([0x66]*512), lba=0)
dump_ptrs("After write")

d_f000 = rn(0xF000, 4)
d_fc00 = rn(0xFC00, 4)
print(f"  F000: {d_f000.hex()} (0x200000)")
print(f"  FC00: {d_fc00.hex()} (0x200C00)")

print("\nDone!")
