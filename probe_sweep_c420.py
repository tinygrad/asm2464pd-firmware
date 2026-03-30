#!/usr/bin/env python3
"""
1. Sweep C420-C4FF for anything that makes F400 get data
2. Test if repeated C403 triggers advance the SRAM write pointer
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

SKIP = {0xC406, 0xC42A, 0xC42C, 0xC42D}

def test_reg(reg, val):
  data = bytes([0xAA]*1024 + [0xBB]*1024)
  dev._bulk_out(0x02, data)
  write(0xF400, 0x00)
  
  orig = read8(reg)
  write(reg, val)
  write(0xC403, 0x04)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  f4 = read8(0xF400)
  
  write(0xC42A, 0x01)
  write(reg, orig)
  write(0xC403, 0x00)
  return f4

# ============================================================
print("="*60)
print("Sweep C420-C4FF")
print("="*60)

for reg in range(0xC420, 0xC500):
  if reg in SKIP:
    continue
  hits = []
  for val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]:
    try:
      f4 = test_reg(reg, val)
      if f4 not in [0x00, 0x55]:
        hits.append(f"0x{val:02X}->F4={f4:02X}")
    except:
      hits.append(f"0x{val:02X}->CRASH")
      break
  if hits:
    print(f"  0x{reg:04X}: {', '.join(hits)}")

# ============================================================
print("\n" + "="*60)
print("TEST: Repeated C403 triggers — does SRAM pointer advance?")
print("="*60)

# Send 4KB
data = bytes([0x11]*1024 + [0x22]*1024 + [0x33]*1024 + [0x44]*1024)
dev._bulk_out(0x02, data)

# Clear F000-FFFF
for off in range(0, 0x1000, 0x400):
  write(0xF000 + off, 0x00)

# Trigger 1
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC406, 0x01)
print(f"  Trigger 1: F000={read8(0xF000):02X} F400={read8(0xF400):02X} F800={read8(0xF800):02X}")
# DON'T unlock, DON'T clear C403

# Trigger 2 (no unlock between)
write(0xC42A, 0x00)
write(0xC406, 0x01)
print(f"  Trigger 2: F000={read8(0xF000):02X} F400={read8(0xF400):02X} F800={read8(0xF800):02X}")

# Trigger 3
write(0xC42A, 0x00)
write(0xC406, 0x01)
print(f"  Trigger 3: F000={read8(0xF000):02X} F400={read8(0xF400):02X} F800={read8(0xF800):02X}")

write(0xC42A, 0x01)
write(0xC403, 0x00)

# Same but with unlock between
print("\n  With unlock between triggers:")
dev._bulk_out(0x02, data)
for off in range(0, 0x1000, 0x400):
  write(0xF000 + off, 0x00)

for i in range(4):
  write(0xC403, 0x04)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  f = [read8(0xF000 + off) for off in range(0, 0x1000, 0x400)]
  print(f"  Trigger {i+1}: F000={f[0]:02X} F400={f[1]:02X} F800={f[2]:02X} FC00={f[3]:02X}")
  write(0xC42A, 0x01)

write(0xC403, 0x00)

print("\nDone!")
