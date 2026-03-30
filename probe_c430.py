#!/usr/bin/env python3
"""
Probe C430 between bulk transfers during large scsi_write.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_c430.py
"""
import sys, struct
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller, WriteOp, ReadOp, ScsiWriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)

# ============================================================
# Test 1: Read C430 baseline
# ============================================================
print("C430 baseline:", hex(r(0xC430)))
print("C431-C433:", rn(0xC430, 4).hex())

# ============================================================
# Test 2: Interleave C430 reads with 512-byte scsi_writes
# ============================================================
print("\n" + "="*60)
print("TEST: C430 between 512-byte scsi_writes")
print("="*60)

for i in range(8):
  v = r(0xC430)
  print(f"  [{i}] C430=0x{v:02X} (before write {i})")
  ctrl.scsi_write(bytes([i]*512), lba=0)
  v = r(0xC430)
  print(f"  [{i}] C430=0x{v:02X} (after write {i})")

# ============================================================
# Test 3: Big write (4KB = 8 sectors) and read C430 after
# ============================================================
print("\n" + "="*60)
print("TEST: C430 around 4KB write")
print("="*60)

v = r(0xC430)
print(f"  Before 4KB write: C430=0x{v:02X}")

ctrl.scsi_write(bytes(4096), lba=0)

v = r(0xC430)
print(f"  After 4KB write: C430=0x{v:02X}")

# ============================================================
# Test 4: Big write (16KB) and read C430 after
# ============================================================
print("\n" + "="*60)
print("TEST: C430 around 16KB write")
print("="*60)

v = r(0xC430)
print(f"  Before 16KB write: C430=0x{v:02X}")

ctrl.scsi_write(bytes(16384), lba=0)

v = r(0xC430)
print(f"  After 16KB write: C430=0x{v:02X}")

# ============================================================
# Test 5: Interleave reads of C430 WITHIN a batch with scsi_write
# Send: [ReadC430, ScsiWrite, ReadC430, ScsiWrite, ReadC430]
# ============================================================
print("\n" + "="*60)
print("TEST: C430 interleaved in exec_ops batch")
print("="*60)

results = ctrl.exec_ops([
  ReadOp(0xC430, 1),
  ScsiWriteOp(bytes([0xAA]*512), lba=0),
  ReadOp(0xC430, 1),
  ScsiWriteOp(bytes([0xBB]*512), lba=0),
  ReadOp(0xC430, 1),
  ScsiWriteOp(bytes([0xCC]*512), lba=0),
  ReadOp(0xC430, 1),
  ScsiWriteOp(bytes([0xDD]*512), lba=0),
  ReadOp(0xC430, 1),
])

for i, res in enumerate(results):
  if res is not None:
    print(f"  result[{i}] C430=0x{res[0]:02X}")

# ============================================================
# Test 6: Read C430-C433 between writes to see full 32-bit value
# ============================================================
print("\n" + "="*60)
print("TEST: C430-C433 interleaved with scsi_write")
print("="*60)

results = ctrl.exec_ops([
  ReadOp(0xC430, 4),
  ScsiWriteOp(bytes(512), lba=0),
  ReadOp(0xC430, 4),
  ScsiWriteOp(bytes(1024), lba=0),
  ReadOp(0xC430, 4),
  ScsiWriteOp(bytes(2048), lba=0),
  ReadOp(0xC430, 4),
  ScsiWriteOp(bytes(4096), lba=0),
  ReadOp(0xC430, 4),
])

for i, res in enumerate(results):
  if res is not None:
    print(f"  result[{i}] C430-C433={res.hex()}")

# ============================================================
# Test 7: Also read CE10-13 (SRAM ptr) interleaved
# ============================================================
print("\n" + "="*60)
print("TEST: CE10-13 + C430-C433 interleaved")
print("="*60)

results = ctrl.exec_ops([
  ReadOp(0xC430, 4), ReadOp(0xCE10, 4),
  ScsiWriteOp(bytes(512), lba=0),
  ReadOp(0xC430, 4), ReadOp(0xCE10, 4),
  ScsiWriteOp(bytes(512), lba=0),
  ReadOp(0xC430, 4), ReadOp(0xCE10, 4),
  ScsiWriteOp(bytes(512), lba=0),
  ReadOp(0xC430, 4), ReadOp(0xCE10, 4),
])

idx = 0
for i, res in enumerate(results):
  if res is not None:
    if len(res) == 4:
      val = struct.unpack('>I', res)[0]
      print(f"  result[{i}] = {res.hex()} (0x{val:08X})")

print("\nDone!")
