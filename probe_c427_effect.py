#!/usr/bin/env python3
"""
Does C427 actually do anything? Or is C403=0x03 always 2 sectors?
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

# Always send 2KB of unique data
data = bytes([0xAA]*512 + [0xBB]*512 + [0xCC]*512 + [0xDD]*512)

print("="*60)
print("C427 values with C403=0x03, always sending 2KB bulk OUT")
print("="*60)

for c427 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x00]:
  dev._bulk_out(0x02, data)
  
  # Clear targets
  for off in range(0, 0x800, 0x200):
    write(0xF000 + off, 0x00)
  
  write(0xC403, 0x03)
  write(0xC427, c427)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  results = []
  for off in range(0, 0x800, 0x200):
    d = read8(0xF000 + off)
    results.append(f"F{off:03X}=0x{d:02X}")
  
  print(f"  C427=0x{c427:02X}: {' '.join(results)}")
  write(0xC42A, 0x01)
  write(0xC403, 0x00)

# Now test: does C427 affect ANYTHING without C403?
print("\n" + "="*60)
print("C427 values WITHOUT C403 (C403=0x00)")
print("="*60)

for c427 in [0x01, 0x02, 0x04, 0x08]:
  dev._bulk_out(0x02, data)
  write(0xF000, 0x00)
  
  write(0xC427, c427)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  # Check how many bytes actually got DMAed
  results = []
  for off in [0x00, 0x08, 0x10, 0x18, 0x100, 0x1F0, 0x200]:
    d = read8(0xF000 + off)
    results.append(f"F{off:03X}=0x{d:02X}")
  
  print(f"  C427=0x{c427:02X}: {' '.join(results)}")
  write(0xC42A, 0x01)

# Specific test: C403=0, C427=1 — exactly how many bytes?
print("\n" + "="*60)
print("Exact byte count: C403=0, C427=1")
print("="*60)

dev._bulk_out(0x02, bytes([0xFF]*512))
# Clear F000-F210
for i in range(0x210):
  write(0xF000 + i, 0x00)

write(0xC427, 0x01)
write(0xC42A, 0x00)
write(0xC406, 0x01)

# Find boundary
last_ff = -1
for i in range(0x210):
  v = read8(0xF000 + i)
  if v == 0xFF:
    last_ff = i
  elif last_ff >= 0:
    print(f"  DMA ends at F{last_ff:03X} (0x{last_ff+1:X} bytes)")
    break

write(0xC42A, 0x01)

# Same for C403=3, C427=1
print("\n" + "="*60)
print("Exact byte count: C403=3, C427=1")
print("="*60)

dev._bulk_out(0x02, bytes([0xFF]*512))
for i in range(0x210):
  write(0xF000 + i, 0x00)

write(0xC403, 0x03)
write(0xC427, 0x01)
write(0xC42A, 0x00)
write(0xC406, 0x01)

last_ff = -1
for i in range(0x210):
  v = read8(0xF000 + i)
  if v == 0xFF:
    last_ff = i
  elif last_ff >= 0:
    print(f"  DMA ends at F{last_ff:03X} (0x{last_ff+1:X} bytes)")
    break

write(0xC42A, 0x01)
write(0xC403, 0x00)

# C403=3, C427=2
print("\nExact byte count: C403=3, C427=2")
dev._bulk_out(0x02, bytes([0xFF]*1024))
for i in range(0x410):
  write(0xF000 + i, 0x00)

write(0xC403, 0x03)
write(0xC427, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)

last_ff = -1
for i in range(0x410):
  v = read8(0xF000 + i)
  if v == 0xFF:
    last_ff = i
  elif last_ff >= 0:
    print(f"  DMA ends at F{last_ff:03X} (0x{last_ff+1:X} bytes)")
    break

write(0xC42A, 0x01)
write(0xC403, 0x00)

print("\nDone!")
