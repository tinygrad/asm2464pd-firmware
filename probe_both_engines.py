#!/usr/bin/env python3
"""
Trace BOTH DMA engines during scsi_write on stock firmware.
Goal: understand which parts of C400 and CE00 engines are used,
and how to target different SRAM addresses.
"""
import sys, random
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp, ReadOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

def snap(label):
  c400 = rn(0xC400, 8)   # arm/trigger
  c420 = rn(0xC420, 16)  # descriptor + doorbell + MSC
  c430 = rn(0xC430, 4)   # PRP/latch
  c470 = rn(0xC470, 2)   # dir + queue busy
  ce00 = rn(0xCE00, 4)   # DMA ctrl
  ce10 = rn(0xCE10, 4)   # SRAM ptr
  ce6e = rn(0xCE6E, 2)   # status
  ce76 = rn(0xCE76, 4)   # buf addr
  ce83 = r(0xCE83)
  ce88 = rn(0xCE88, 2)   # handshake
  print(f"  {label}:")
  print(f"    C400-07: {c400.hex()}")
  print(f"    C420-2F: {c420.hex()}")
  print(f"    C430-33: {c430.hex()}")
  print(f"    C470-71: {c470.hex()}")
  print(f"    CE00-03: {ce00.hex()}")
  print(f"    CE10-13 (SRAM ptr): {ce10.hex()}")
  print(f"    CE6E-6F (status): {ce6e.hex()}")
  print(f"    CE76-79 (buf addr): {ce76.hex()}")
  print(f"    CE83 (flow): 0x{ce83:02X}")
  print(f"    CE88-89 (handshake): {ce88.hex()}")

# ============================================================
print("="*60)
print("IDLE STATE")
print("="*60)
snap("idle")

# ============================================================
print("\n" + "="*60)
print("AFTER scsi_write(512B, lba=0)")
print("="*60)
ctrl.scsi_write(bytes([0xAA]*512), lba=0)
snap("after 512B lba=0")

# ============================================================
print("\n" + "="*60)
print("AFTER scsi_write(512B, lba=1)")
print("="*60)
ctrl.scsi_write(bytes([0xBB]*512), lba=1)
snap("after 512B lba=1")

# ============================================================
print("\n" + "="*60)
print("AFTER scsi_write(512B, lba=0x100)")
print("="*60)
ctrl.scsi_write(bytes([0xCC]*512), lba=0x100)
snap("after 512B lba=0x100")

# ============================================================
print("\n" + "="*60)
print("Check where data actually landed")
print("="*60)
# lba=0 should be F000, lba=1 should be F200(?), lba=0x100 should be ???
for off in [0x000, 0x100, 0x200, 0x400, 0x800]:
  d = rn(0xF000 + off, 4)
  print(f"  F{off:03X}: {d.hex()}")

# ============================================================
print("\n" + "="*60)
print("Now manually set CE76-79 and see if scsi_write uses it")
print("="*60)

# Set CE76-79 to PCI 0x00200100 (= F100 in XDATA)
w(0xCE76, 0x00)
w(0xCE77, 0x01)
w(0xCE78, 0x20)
w(0xCE79, 0x00)
print(f"  CE76-79 set to: {rn(0xCE76, 4).hex()}")

ctrl.scsi_write(bytes([0xDD]*512), lba=0)
snap("after write with CE76=0x200100")

print(f"\n  F000: {rn(0xF000, 4).hex()} (should be AA or DD)")
print(f"  F100: {rn(0xF100, 4).hex()} (DD if CE76 worked)")

# ============================================================
print("\n" + "="*60)
print("Interleave: read CE76-79 between scsi_write steps")
print("="*60)

# Reset CE76
w(0xCE76, 0x00); w(0xCE77, 0x00); w(0xCE78, 0x20); w(0xCE79, 0x00)

# Do a write, but read CE76 right after the ScsiWriteOp (before CE6E reset)
results = ctrl.exec_ops([
  ReadOp(0xCE76, 4),
  ScsiWriteOp(bytes([0xEE]*512), lba=0),
  ReadOp(0xCE76, 4),
  ReadOp(0xCE10, 4),
])
for i, res in enumerate(results):
  if res is not None:
    print(f"  result[{i}]: {res.hex()}")

# Do the CE6E reset
ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# Check CE76 after reset
print(f"  CE76-79 after CE6E reset: {rn(0xCE76, 4).hex()}")

# ============================================================
print("\n" + "="*60)
print("What does tinygrad init write to C422?")
print("="*60)
# From ASM24Controller.__init__:
# self.exec_ops([WriteOp(0x54b, b' '), WriteOp(0x54e, b'\x04'), 
#   WriteOp(0x5a8, b'\x02'), WriteOp(0x5f8, b'\x04'),
#   WriteOp(0x7ec, b'\x01\x00\x00\x00'), WriteOp(0xc422, b'\x02'), 
#   WriteOp(0x0, b'\x33')])
# 
# It writes C422=0x02! That's part of the C400 descriptor.
# Why does tinygrad init write to C422?
print(f"  C422: 0x{r(0xC422):02X}")

# What are the tinygrad init xdata writes?
for addr in [0x054B, 0x054E, 0x05A8, 0x05F8]:
  print(f"  0x{addr:04X}: 0x{r(addr):02X}")
print(f"  0x07EC-EF: {rn(0x07EC, 4).hex()}")

print("\nDone!")
