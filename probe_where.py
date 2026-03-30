#!/usr/bin/env python3
"""
DMA is going somewhere other than F000! CE10 is probably still 0x200400.
Check CE10 and scan SRAM to find where data went.
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

# Check CE10
ce10 = bytes([read8(0xCE10+i) for i in range(4)])
print(f"CE10 = {ce10.hex()}")

# Send data and trigger
dev._bulk_out(0x02, bytes([0xEE]*32))
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

# Scan ALL of F000-FFFF
print("\nScan F000-FFFF for 0xEE:")
for off in range(0, 0x1000, 0x100):
  d = read8(0xF000 + off)
  if d == 0xEE:
    print(f"  F{off:03X}: 0x{d:02X} ** FOUND **")
  elif d != 0x55 and d != 0x00:
    print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)

# Now reset CE10 back to 0x200000 and try again
print("\nResetting CE10 to 0x200000...")
write(0xCE10, 0x00); write(0xCE11, 0x20); write(0xCE12, 0x00); write(0xCE13, 0x00)
ce10 = bytes([read8(0xCE10+i) for i in range(4)])
print(f"CE10 = {ce10.hex()}")

dev._bulk_out(0x02, bytes([0xDD]*32))
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

print(f"\nF000 = 0x{read8(0xF000):02X} (expect DD)")
write(0xC42A, 0x01)

print("\nDone!")
