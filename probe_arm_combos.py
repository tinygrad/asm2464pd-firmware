#!/usr/bin/env python3
"""
Test different arm bit combinations for multi-sector C400 DMA.
Maybe certain arm bits enable multi-sector mode.
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

def test(label, c427, arms, c420=0x00, c421=0x01):
  """Send 1KB, set descriptor, arm, trigger, check F000 and F200."""
  data = bytes([0xAA]*512 + [0xBB]*512)
  dev._bulk_out(0x02, data)
  
  # Clear targets
  write(0xF000, 0x00); write(0xF001, 0x00)
  write(0xF200, 0x00); write(0xF201, 0x00)
  
  # Descriptor
  write(0xC420, c420); write(0xC421, c421); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, c427); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  
  # Arm
  for addr, val in arms:
    write(addr, val)
  
  # Trigger
  write(0xC406, 1)
  
  f0 = read_range(0xF000, 2)
  f2 = read_range(0xF200, 2)
  ok = f2 == bytes([0xBB, 0xBB])
  print(f"  {label}: F000={f0.hex()} F200={f2.hex()} {'** MULTI OK **' if ok else ''}")
  write(0xC42A, 0x01)
  return ok

# ============================================================
print("="*60)
print("Arm bit combinations with C427=2")
print("="*60)

# All standard arms
test("standard all",  2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1)])

# Try different values for C404 (stock writes 2, not 1)
test("C404=2",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,2)])
test("C404=3",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,3)])
test("C404=4",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,4)])

# Try different values for C400
test("C400=2",        2, [(0xC400,2),(0xC401,1),(0xC402,1),(0xC404,1)])
test("C400=3",        2, [(0xC400,3),(0xC401,1),(0xC402,1),(0xC404,1)])

# Try different values for C401
test("C401=2",        2, [(0xC400,1),(0xC401,2),(0xC402,1),(0xC404,1)])
test("C401=3",        2, [(0xC400,1),(0xC401,3),(0xC402,1),(0xC404,1)])

# Try different values for C402
test("C402=2",        2, [(0xC400,1),(0xC401,1),(0xC402,2),(0xC404,1)])
test("C402=3",        2, [(0xC400,1),(0xC401,1),(0xC402,3),(0xC404,1)])

# Try writing C403 and C405 (R/W regs we found)
test("C403=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC403,1),(0xC404,1)])
test("C405=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1),(0xC405,1)])
test("C403=1,C405=1", 2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC403,1),(0xC404,1),(0xC405,1)])

# Try the R/W regs C408, C409, C40C, C40D
test("C408=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1),(0xC408,1)])
test("C409=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1),(0xC409,1)])
test("C40C=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1),(0xC40C,1)])
test("C40D=1",        2, [(0xC400,1),(0xC401,1),(0xC402,1),(0xC404,1),(0xC40D,1)])

# ============================================================
print("\n" + "="*60)
print("Try different C428 values (queue cfg)")
print("="*60)

for c428 in [0x00, 0x08, 0x10, 0x20, 0x28, 0x30, 0x38]:
  dev._bulk_out(0x02, bytes([0xAA]*512 + [0xBB]*512))
  write(0xF000, 0x00); write(0xF200, 0x00)
  write(0xC427, 0x02); write(0xC428, c428)
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  f0 = read_range(0xF000, 2)
  f2 = read_range(0xF200, 2)
  ok = f2 == bytes([0xBB, 0xBB])
  print(f"  C428=0x{c428:02X}: F000={f0.hex()} F200={f2.hex()} {'** OK **' if ok else ''}")
  write(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("Try C412-C419 control registers (some writable)")
print("="*60)

# C414 is R/W, C416-C419 are R/W
for reg, val, label in [
  (0xC412, 0x03, "C412=0x03"),
  (0xC414, 0x80, "C414=0x80"),
  (0xC415, 0x01, "C415=0x01"),
  (0xC416, 0x01, "C416=0x01"),
  (0xC417, 0x01, "C417=0x01"),
  (0xC418, 0x01, "C418=0x01"),
  (0xC419, 0x01, "C419=0x01"),
]:
  dev._bulk_out(0x02, bytes([0xAA]*512 + [0xBB]*512))
  write(0xF000, 0x00); write(0xF200, 0x00)
  write(reg, val)
  write(0xC427, 0x02); write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  f0 = read_range(0xF000, 2)
  f2 = read_range(0xF200, 2)
  ok = f2 == bytes([0xBB, 0xBB])
  print(f"  {label}: F000={f0.hex()} F200={f2.hex()} {'** OK **' if ok else ''}")
  write(0xC42A, 0x01)
  write(reg, 0x00)  # restore

# ============================================================
print("\n" + "="*60)
print("Try setting C471 (queue busy)")
print("="*60)

write(0xC471, 0x01)
dev._bulk_out(0x02, bytes([0xAA]*512 + [0xBB]*512))
write(0xF000, 0x00); write(0xF200, 0x00)
write(0xC427, 0x02); write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
f0 = read_range(0xF000, 2)
f2 = read_range(0xF200, 2)
print(f"  C471=1: F000={f0.hex()} F200={f2.hex()}")
write(0xC42A, 0x01)

print("\nDone!")
