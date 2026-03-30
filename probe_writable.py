#!/usr/bin/env python3
"""
Full writability scan of C400-C4FF and CE00-CEFF.
Skip dangerous registers (C42A doorbell, C42C MSC trigger, C406 DMA trigger).
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# Registers to SKIP (dangerous - can break USB/DMA state)
SKIP = {
  0xC400, 0xC401, 0xC402, 0xC404, 0xC406,  # DMA arm/trigger
  0xC42A,  # doorbell - BREAKS communication
  0xC42C, 0xC42D,  # MSC trigger
  0xCE00,  # DMA trigger
  0xCE88,  # handshake trigger
}

print("="*60)
print("Doorbell + MSC state (read-only)")
print("="*60)
print(f"  C42A (DOORBELL):   0x{r(0xC42A):02X}")
print(f"  C42C (MSC_CTRL):   0x{r(0xC42C):02X}")
print(f"  C42D (MSC_STATUS): 0x{r(0xC42D):02X}")
print(f"  C471 (QUEUE_BUSY): 0x{r(0xC471):02X}")
print(f"  900B (MSC_CFG):    0x{r(0x900B):02X}")

print("\n" + "="*60)
print("C400-C4FF writability scan")
print("="*60)

for addr in range(0xC400, 0xC500):
  if addr in SKIP:
    continue
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)  # restore
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL(mask=0x{rb & tv:02X})")

print("\n" + "="*60)
print("CE00-CEFF writability scan")
print("="*60)

for addr in range(0xCE00, 0xCF00):
  if addr in SKIP:
    continue
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)  # restore
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL(mask=0x{rb & tv:02X})")

# Verify still working
print("\n" + "="*60)
print("Verify scsi_write")
print("="*60)
import random
from tinygrad.runtime.support.usb import ScsiWriteOp, WriteOp
test = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test, lba=0)
after = ctrl.read(0xF000, 16)
print(f"  MATCH: {after == test[:16]}")

print("\nDone!")
