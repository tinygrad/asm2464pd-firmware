#!/usr/bin/env python3
"""
Test SCSI WRITE(16) against add1:0001 stock firmware using tinygrad's ASM24Controller.
Uses UAS (streams) mode, exactly like tinygrad does it.

Usage:
  PYTHONPATH=~/tinygrad python3 test_scsi_write.py
"""
import sys, os, random
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

print("Connecting via ASM24Controller (add1:0001, UAS mode)...")
ctrl = ASM24Controller()
print("Connected + initialized!")

# Read 0xf000 before
print("\n[1] Read 0xf000 baseline...")
before = ctrl.read(0xf000, 32)
print(f"  0xf000: {before.hex()}")

# Full scsi_write - exactly what tinygrad does for writes to 0xf000
print("\n[2] scsi_write(512 bytes, lba=0)...")
test_data = bytes(random.getrandbits(8) for _ in range(512))
print(f"  first 16: {test_data[:16].hex()}")
ctrl.scsi_write(test_data, lba=0)
print("  Done!")

# Check 0xf000
print("\n[3] Read 0xf000 after scsi_write...")
after = ctrl.read(0xf000, 64)
print(f"  0xf000: {after.hex()}")
if after[:32] == test_data[:32]:
    print("  MATCH at 0xf000!")
else:
    print("  No match at 0xf000, checking other locations...")
    for addr in [0x7000, 0x8000, 0xD800]:
        data = ctrl.read(addr, 32)
        print(f"  0x{addr:04X}: {data.hex()}")

print("\nDone!")
