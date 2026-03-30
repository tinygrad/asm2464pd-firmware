#!/usr/bin/env python3
"""
Check NVME_DOORBELL (C42A) state and test if C4xx regs are writable
when doorbell bits are set. Also retry CE10 writes with different
doorbell/MSC states.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)
def reset_ce6e(): ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# ============================================================
print("="*60)
print("Current doorbell and MSC state")
print("="*60)
print(f"  C42A (DOORBELL):   0x{r(0xC42A):02X} ({r(0xC42A):08b})")
print(f"  C42C (MSC_CTRL):   0x{r(0xC42C):02X}")
print(f"  C42D (MSC_STATUS): 0x{r(0xC42D):02X}")
print(f"  C471 (QUEUE_BUSY): 0x{r(0xC471):02X}")
print(f"  900B (MSC_CFG):    0x{r(0x900B):02X}")
print(f"  C473 (LINK_PARAM): 0x{r(0xC473):02X}")

# ============================================================
print("\n" + "="*60)
print("C42A writability test")
print("="*60)
orig = r(0xC42A)
for val in [0x01, 0x1F, 0x20, 0x3F, 0xFF, 0x00]:
  w(0xC42A, val)
  rb = r(0xC42A)
  print(f"  wrote 0x{val:02X} -> read 0x{rb:02X}")
w(0xC42A, orig)

# ============================================================
print("\n" + "="*60)
print("CE10-CE13 write retry with doorbell bits set")
print("="*60)

ce10 = rn(0xCE10, 4)
print(f"  CE10 baseline: {ce10.hex()}")

# Try with doorbell ramp-up (like stock MSC engine does)
print("\n  With C42A doorbell ramp-up:")
w(0xC42A, 0x01)
w(0xC42A, 0x03)
w(0xC42A, 0x07)
w(0xC42A, 0x0F)
w(0xC42A, 0x1F)
print(f"  C42A = 0x{r(0xC42A):02X}")

w(0xCE10, 0x00); w(0xCE11, 0x20); w(0xCE12, 0x10); w(0xCE13, 0x00)
ce10 = rn(0xCE10, 4)
print(f"  CE10 after write: {ce10.hex()}")

# Clear doorbell
w(0xC42A, 0x00)

# Try with C42A bit 5 (link gate)
print("\n  With C42A bit 5 (link gate):")
w(0xC42A, 0x20)
w(0xCE10, 0x00); w(0xCE11, 0x21); w(0xCE12, 0x00); w(0xCE13, 0x00)
ce10 = rn(0xCE10, 4)
print(f"  CE10 after write: {ce10.hex()}")
w(0xC42A, 0x00)

# ============================================================
print("\n" + "="*60)
print("C4xx writability scan (all non-zero regs)")
print("="*60)

# Scan C400-C4FF for writable registers
for addr in range(0xC400, 0xC500):
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)  # restore
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig and rb != tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")

# ============================================================
print("\n" + "="*60)
print("CE00-CEFF writability scan")
print("="*60)

for addr in range(0xCE00, 0xCF00):
  orig = r(addr)
  tv = 0xAA if orig != 0xAA else 0x55
  w(addr, tv)
  rb = r(addr)
  w(addr, orig)  # restore
  if rb == tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
  elif rb != orig and rb != tv:
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")

# ============================================================
print("\n" + "="*60)
print("Verify scsi_write still works")
print("="*60)
import random
test = bytes(random.getrandbits(8) for _ in range(512))
ctrl.scsi_write(test, lba=0)
after = ctrl.read(0xF000, 16)
print(f"  Data:  {test[:16].hex()}")
print(f"  F000:  {after.hex()}")
print(f"  MATCH: {after == test[:16]}")

print("\nDone!")
