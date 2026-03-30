#!/usr/bin/env python3
"""
Does the SRAM write pointer advance between C400 triggers?
Send 4KB bulk out, then do 4 x C403=4 triggers, see if each 
1KB lands at a different SRAM offset.
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

# Send 4KB with unique byte per 1KB page
data = bytes([0xAA]*1024 + [0xBB]*1024 + [0xCC]*1024 + [0xDD]*1024)
dev._bulk_out(0x02, data)

# Verify 0x7000 has all 4KB
print("7000 buffer after 4KB bulk out:")
print(f"  7000: {read_range(0x7000, 2).hex()} (expect AA)")
print(f"  7400: {read_range(0x7400, 2).hex()} (expect BB)")
print(f"  7800: {read_range(0x7800, 2).hex()} (expect CC)")
print(f"  7C00: {read_range(0x7C00, 2).hex()} (expect DD)")

# Clear SRAM F000-FFFF
print("\nClearing F000-F1000...")
for off in range(0, 0x1000, 0x200):
  write(0xF000 + off, 0x00)

# Trigger 1: C403=4 (1KB)
print("\n--- Trigger 1 (C403=4) ---")
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC406, 0x01)

for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
  d = read8(0xF000 + off)
  print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)  # unlock

# Trigger 2: another C403=4
print("\n--- Trigger 2 (C403=4, no new bulk out) ---")
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC406, 0x01)

for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
  d = read8(0xF000 + off)
  print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)

# Trigger 3
print("\n--- Trigger 3 ---")
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC406, 0x01)

for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
  d = read8(0xF000 + off)
  print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)

# Trigger 4
print("\n--- Trigger 4 ---")
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC406, 0x01)

for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
  d = read8(0xF000 + off)
  print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)
write(0xC403, 0x00)

print("\nDone!")
