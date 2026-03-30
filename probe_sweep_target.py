#!/usr/bin/env python3
"""
Find what register moves the C400 DMA write target.
Send known data, DMA to F000, then write a register,
DMA again, check if second DMA went to a different address.
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

SKIP = {0xC406, 0xC42A, 0xC42C, 0xC42D}

def test_pointer_advance(reg, val):
  """
  1. bulk_out AA, DMA -> F000=AA
  2. Write reg=val
  3. bulk_out BB, DMA -> check if BB landed somewhere other than F000
  """
  # First DMA: AA to F000
  dev._bulk_out(0x02, bytes([0xAA]*32))
  write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
  
  # Clear F200-FBFF to detect new writes
  for off in [0x200, 0x400]:
    write(0xF000 + off, 0x55)
  
  # Write the register under test
  orig = read8(reg)
  write(reg, val)
  
  # Second DMA: BB
  dev._bulk_out(0x02, bytes([0xBB]*32))
  write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
  
  # Check: did BB go to F000 (overwrite) or somewhere else (advance)?
  f0 = read8(0xF000)
  f2 = read8(0xF200)
  f4 = read8(0xF400)
  
  write(reg, orig)  # restore
  
  # If F000 != BB, or F200/F400 == BB, pointer advanced
  advanced = (f0 != 0xBB) or (f2 == 0xBB) or (f4 == 0xBB)
  return f0, f2, f4, advanced

# ============================================================
print("="*60)
print("Sweep C400-C4FF: looking for pointer advance")
print("="*60)

for reg in range(0xC400, 0xC500):
  if reg in SKIP:
    continue
  for val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]:
    try:
      f0, f2, f4, advanced = test_pointer_advance(reg, val)
      if advanced:
        print(f"  0x{reg:04X}=0x{val:02X}: F000={f0:02X} F200={f2:02X} F400={f4:02X} ** ADVANCE **")
    except:
      # crashed, reset is needed but skip for now
      break

# ============================================================
print("\n" + "="*60)
print("Sweep CE00-CEFF: looking for pointer advance")
print("="*60)

SKIP_CE = {0xCE00, 0xCE88}

for reg in range(0xCE00, 0xCF00):
  if reg in SKIP_CE:
    continue
  for val in [0x01, 0x02, 0x04, 0x08]:
    try:
      f0, f2, f4, advanced = test_pointer_advance(reg, val)
      if advanced:
        print(f"  0x{reg:04X}=0x{val:02X}: F000={f0:02X} F200={f2:02X} F400={f4:02X} ** ADVANCE **")
    except:
      break

print("\nDone!")
