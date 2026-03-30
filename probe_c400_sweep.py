#!/usr/bin/env python3
"""
Sweep ALL values 0x00-0xFF for every register C400-C41F.
Looking for what makes F400 get data (>1KB DMA).
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

def test_reg(reg, val):
  """Send 2KB, set reg=val + C403=4, trigger, check F400."""
  data = bytes([0xAA]*1024 + [0xBB]*1024)
  dev._bulk_out(0x02, data)
  write(0xF400, 0x00)
  
  write(reg, val)
  write(0xC403, 0x04)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  f4 = read8(0xF400)
  
  write(0xC42A, 0x01)
  write(reg, 0x00)
  write(0xC403, 0x00)
  return f4

# Sweep each register with a few key values
print("="*60)
print("Sweep C400-C41F: looking for F400 != 0x00")
print("="*60)

SKIP = {0xC406}  # trigger — don't write random values

for reg in range(0xC400, 0xC420):
  if reg in SKIP:
    continue
  hits = []
  for val in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]:
    try:
      f4 = test_reg(reg, val)
      if f4 not in [0x00, 0x55]:
        hits.append(f"0x{val:02X}->F4={f4:02X}")
    except:
      hits.append(f"0x{val:02X}->CRASH")
      break
  if hits:
    print(f"  0x{reg:04X}: {', '.join(hits)}")
  else:
    print(f"  0x{reg:04X}: (no effect)")

print("\nDone!")
