#!/usr/bin/env python3
"""
Test every C5xx register for moving the DMA TARGET ADDRESS.
Send AA, clear F000-FFFF, set one C5xx reg, trigger, scan where AA landed.
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

test_n = [0]

def test(label, pre_writes):
  test_n[0] += 1
  marker = (test_n[0] * 13 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  
  dev._bulk_out(0x02, bytes([marker]*32))
  
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)
  
  for addr, val in pre_writes.items():
    write(addr, val)
  
  # Full known-good trigger
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  
  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(off)
  
  write(0xC42A, 0x01)
  for addr in pre_writes:
    write(addr, 0x00)
  
  moved = landed and landed[0] != 0
  if moved:
    locs = ', '.join(f'F{off:03X}' for off in landed)
    print(f"  {label:40s}: {locs} ** MOVED **")
  return moved

# Sweep every C5xx register with values that could be address-related
print("="*60)
print("Sweep C500-C5FF: looking for address control")
print("="*60)

for reg in range(0xC500, 0xC600):
  for val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]:
    try:
      moved = test(f"0x{reg:04X}=0x{val:02X}", {reg: val})
      if moved:
        # Found it! Test more values
        for v2 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x20]:
          test(f"  0x{reg:04X}=0x{v2:02X}", {reg: v2})
        break
    except:
      break

print("\nDone!")
