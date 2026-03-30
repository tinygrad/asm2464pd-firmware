#!/usr/bin/env python3
"""
Break out scsi_write into individual ops and read C430 + CE10 between each.
"""
import sys, struct
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ReadOp, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def snap(label):
  c430 = rn(0xC430, 4)
  ce10 = rn(0xCE10, 4)
  ce76 = rn(0xCE76, 4)
  ce6e = rn(0xCE6E, 2)
  print(f"  {label}: C430={c430.hex()} CE10={ce10.hex()} CE76={ce76.hex()} CE6E={ce6e.hex()}")

# 4KB write = 8 sectors
buf = bytes([0xAA]*512 + [0xBB]*512 + [0xCC]*512 + [0xDD]*512 +
            [0xEE]*512 + [0xFF]*512 + [0x11]*512 + [0x22]*512)
assert len(buf) == 4096

print("="*60)
print("Breaking out scsi_write(4096 bytes, lba=0)")
print("="*60)

snap("initial")

# Step 1: ScsiWriteOp (the actual 0x8A SCSI WRITE CDB + data)
print("\n--- Step 1: ScsiWriteOp ---")
ctrl.exec_ops([ScsiWriteOp(buf, lba=0)])
snap("after ScsiWriteOp")

# Step 2: WriteOp(0x171, 0xFF,0xFF,0xFF)
print("\n--- Step 2: WriteOp(0x171, ff ff ff) ---")
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
snap("after WriteOp 0x171")

# Step 3: WriteOp(0xCE6E, 0x00,0x00)
print("\n--- Step 3: WriteOp(0xCE6E, 00 00) ---")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after WriteOp CE6E")

# Verify data
print("\n--- SRAM readback ---")
for off in range(0, 4096, 512):
  addr = 0xF000 + off
  d = rn(addr, 4)
  print(f"  0x{addr:04X}: {d.hex()}")

# ============================================================
# Now do it again to see if C430 changes on second write
# ============================================================
print("\n" + "="*60)
print("Second write (different data)")
print("="*60)

buf2 = bytes([0x01]*4096)
snap("before 2nd write")

print("\n--- ScsiWriteOp ---")
ctrl.exec_ops([ScsiWriteOp(buf2, lba=0)])
snap("after ScsiWriteOp")

print("\n--- WriteOp(0x171) ---")
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
snap("after WriteOp 0x171")

print("\n--- WriteOp(CE6E) ---")
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
snap("after WriteOp CE6E")

# ============================================================
# Now try a big write (>16KB) which triggers CE40-43 clear
# ============================================================
print("\n" + "="*60)
print("Big write (64KB = 4 x 16KB chunks)")
print("="*60)

buf3 = bytes([0x77]*65536)
snap("before big write")

# Manually replicate scsi_write logic for >0x4000:
# buf is already 64KB, aligned to 0x10000
for i in range(0, len(buf3), 0x10000):
  chunk = buf3[i:i+0x10000]
  print(f"\n--- Chunk {i//0x10000}: ScsiWriteOp + WriteOp(0x171) ---")
  ctrl.exec_ops([ScsiWriteOp(chunk, lba=0), WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  snap(f"after chunk {i//0x10000}")

  print(f"--- Chunk {i//0x10000}: WriteOp(CE6E) ---")
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
  snap(f"after CE6E clear {i//0x10000}")

# CE40-43 clear (only for >0x4000)
print("\n--- CE40-43 clear ---")
for j in range(4):
  ctrl.exec_ops([WriteOp(0xCE40 + j, b'\x00', ignore_cache=True)])
snap("after CE40-43 clear")

print("\nDone!")
