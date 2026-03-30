#!/usr/bin/env python3
"""
Check if stock firmware uses MSC engine, and trace all the key registers
that change during a scsi_write to understand the full DMA pipeline.
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
print("MSC and USB engine state on stock firmware")
print("="*60)

regs = [
  (0x900B, "MSC_CFG"),
  (0x9002, "USB_CONFIG"),
  (0x9005, "USB_CTRL"),
  (0x90E2, "USB_MODE"),
  (0x9091, "EP1_CFG1"),
  (0x9093, "EP2_CFG1"),
  (0x901A, "XFER_LEN"),
  (0x91C1, "EP_ARM"),
  (0x91D1, "INT_FLAGS"),
  (0x92C0, "POWER_ENABLE"),
  (0xC42A, "DOORBELL"),
  (0xC42C, "MSC_CTRL"),
  (0xC42D, "MSC_STATUS"),
  (0xC471, "QUEUE_BUSY"),
  (0xC473, "LINK_PARAM"),
  (0xCE00, "SCSI_DMA_CTRL"),
  (0xCE83, "BUF_FLOW"),
  (0xCE88, "HANDSHAKE"),
  (0xCE89, "DMA_STATE"),
]

for addr, name in regs:
  print(f"  0x{addr:04X} ({name:15s}): 0x{r(addr):02X}")

# ============================================================
print("\n" + "="*60)
print("Do a scsi_write and snapshot BEFORE and AFTER each step")
print("="*60)

# Snapshot function for key regs
def snap(label):
  vals = {}
  for addr in [0x900B, 0xC42A, 0xC42C, 0xC42D, 0xC471,
               0xCE00, 0xCE10, 0xCE11, 0xCE12, 0xCE13,
               0xCE55, 0xCE76, 0xCE77, 0xCE78, 0xCE79,
               0xCE83, 0xCE88, 0xCE89, 0xCE6E, 0xCE6F,
               0xC430]:
    vals[addr] = r(addr)
  msc = vals[0x900B]
  ce10 = f"{vals[0xCE10]:02X}{vals[0xCE11]:02X}{vals[0xCE12]:02X}{vals[0xCE13]:02X}"
  ce76 = f"{vals[0xCE79]:02X}{vals[0xCE78]:02X}{vals[0xCE77]:02X}{vals[0xCE76]:02X}"
  ce6e = f"{vals[0xCE6E]:02X}{vals[0xCE6F]:02X}"
  print(f"  {label}:")
  print(f"    MSC_CFG=0x{msc:02X} DOORBELL=0x{vals[0xC42A]:02X} MSC_CTRL=0x{vals[0xC42C]:02X} MSC_STATUS=0x{vals[0xC42D]:02X}")
  print(f"    CE10(SRAM)={ce10} CE76(buf)={ce76} CE6E={ce6e}")
  print(f"    CE00=0x{vals[0xCE00]:02X} CE55=0x{vals[0xCE55]:02X} CE83=0x{vals[0xCE83]:02X} CE88=0x{vals[0xCE88]:02X} CE89=0x{vals[0xCE89]:02X}")
  print(f"    C430=0x{vals[0xC430]:02X} C471=0x{vals[0xC471]:02X}")
  return vals

before = snap("IDLE")

# Step 1: ScsiWriteOp only (no cleanup)
ctrl.exec_ops([ScsiWriteOp(bytes([0xAA]*512), lba=0)])
after_scsi = snap("After ScsiWriteOp(512B)")

# Step 2: WriteOp(0x171)
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
after_171 = snap("After WriteOp(0x171)")

# Step 3: CE6E reset
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])
after_reset = snap("After CE6E reset")

# ============================================================
print("\n" + "="*60)
print("What changed between idle and after ScsiWriteOp?")
print("="*60)

for addr in before:
  if before[addr] != after_scsi[addr]:
    print(f"  0x{addr:04X}: 0x{before[addr]:02X} -> 0x{after_scsi[addr]:02X}")

# ============================================================
print("\n" + "="*60)
print("TEST: Can we toggle MSC_CFG (900B) on stock firmware?")
print("="*60)

orig = r(0x900B)
print(f"  900B orig: 0x{orig:02X}")

# Try writing different values
for val in [0x00, 0x01, 0x03, 0x07, 0x0F, 0xFF]:
  w(0x900B, val)
  rb = r(0x900B)
  print(f"  wrote 0x{val:02X} -> read 0x{rb:02X}")

w(0x900B, orig)  # restore

# ============================================================
print("\n" + "="*60)
print("TEST: Does scsi_write work after toggling MSC_CFG?")
print("="*60)

# Set MSC_CFG=0x07 (enable MSC)
w(0x900B, 0x07)
print(f"  900B after set 0x07: 0x{r(0x900B):02X}")

ctrl.scsi_write(bytes([0xBB]*512), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")

# Set back to 0x00
w(0x900B, 0x00)
print(f"  900B after set 0x00: 0x{r(0x900B):02X}")

ctrl.scsi_write(bytes([0xCC]*512), lba=0)
print(f"  F000: {rn(0xF000, 4).hex()}")

# ============================================================
print("\n" + "="*60)
print("TEST: What's the stock firmware 0x900B at different times?")
print("  Read it rapidly during scsi_write via interleaved ops")
print("="*60)

# Read 900B before, during (interleaved), and after
from tinygrad.runtime.support.usb import ReadOp

results = ctrl.exec_ops([
  ReadOp(0x900B, 1),
  ScsiWriteOp(bytes([0xDD]*512), lba=0),
  ReadOp(0x900B, 1),
])
for i, res in enumerate(results):
  if res is not None:
    print(f"  result[{i}] 900B=0x{res[0]:02X}")

ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("\nDone!")
