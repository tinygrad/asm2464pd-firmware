#!/usr/bin/env python3
"""
CC36=0x20 might be the F000 window base (SRAM page).
0x20 = PCI 0x200000 >> 16 perhaps?
Test: change CC36 and see if F000 shows different SRAM data.
Also look at nearby CC registers.
"""
import sys, ctypes
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0

def read_range(addr, n):
  return bytes([read8(addr + i) for i in range(n)])

# First, DMA some data to F000 so we have a known pattern
dev._bulk_out(0x02, bytes([0xDE, 0xAD, 0xBE, 0xEF] + [0]*28))
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)

print(f"  F000 baseline: {read_range(0xF000, 4).hex()} (should be deadbeef)")

# Now check CC36 and neighbors
print(f"\n  CC34: 0x{read8(0xCC34):02X}")
print(f"  CC35: 0x{read8(0xCC35):02X}")
print(f"  CC36: 0x{read8(0xCC36):02X}")
print(f"  CC37: 0x{read8(0xCC37):02X}")
print(f"  CC38: 0x{read8(0xCC38):02X}")

# Try changing CC36 — does F000 show different data?
print("\nChanging CC36:")
orig_cc36 = read8(0xCC36)

for val in [0x00, 0x21, 0x22, 0x30, 0x40]:
  write(0xCC36, val)
  f = read_range(0xF000, 4)
  cc36_rb = read8(0xCC36)
  print(f"  CC36=0x{val:02X} (readback=0x{cc36_rb:02X}): F000={f.hex()}")

# Restore
write(0xCC36, orig_cc36)
print(f"  Restored CC36=0x{read8(0xCC36):02X}: F000={read_range(0xF000, 4).hex()}")

# Check if CC36 is really writable
write(0xCC36, 0xAA)
rb = read8(0xCC36)
write(0xCC36, orig_cc36)
print(f"\n  CC36 write test: wrote 0xAA, read 0x{rb:02X}")

# Maybe the window is controlled by CC16/CC18/CC1C (also have 0x10 values)
print("\nCC16/CC18/CC1A/CC1C area:")
for addr in [0xCC16, 0xCC18, 0xCC1A, 0xCC1C, 0xCC1E, 0xCC20, 0xCC22]:
  print(f"  0x{addr:04X} = 0x{read8(addr):02X}")

print("\nDone!")
