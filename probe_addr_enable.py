#!/usr/bin/env python3
"""
C403 enables the length. There's probably an address enable + address register.
C405 is R/W and adjacent to C403 — could be address enable.
C4EE/C4EF are documented as DMA addr lo/hi.

Test: set address registers + enable bits, then DMA, check if target moved.
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

def test(label, pre_writes):
  """Send AA, set regs, trigger, scan F000-FFFF for where AA landed."""
  dev._bulk_out(0x02, bytes([0xAA]*32))
  
  # Clear scan area
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x55)
  
  for addr, val in pre_writes:
    write(addr, val)
  
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  # Scan for where AA landed
  found = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == 0xAA:
      found.append(f"F{off:03X}")
  
  write(0xC42A, 0x01)
  for addr, val in pre_writes:
    write(addr, 0x00)
  
  target = ', '.join(found) if found else 'NOWHERE'
  moved = found and found[0] != 'F000'
  print(f"  {label:50s}: {target} {'** MOVED **' if moved else ''}")

# ============================================================
print("="*60)
print("C405 (addr enable?) + C4EE/C4EF (addr lo/hi)")
print("="*60)

# Just address, no enable
test("C4EE=0x01 C4EF=0x00 (addr=0x0100?)", [(0xC4EE, 0x01), (0xC4EF, 0x00)])
test("C4EE=0x02 C4EF=0x00", [(0xC4EE, 0x02), (0xC4EF, 0x00)])
test("C4EE=0x00 C4EF=0x01", [(0xC4EE, 0x00), (0xC4EF, 0x01)])

# C405 as enable + address
for c405 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10]:
  test(f"C405=0x{c405:02X} + C4EE=0x01", [(0xC405, c405), (0xC4EE, 0x01)])
  test(f"C405=0x{c405:02X} + C4EF=0x01", [(0xC405, c405), (0xC4EF, 0x01)])

# ============================================================
print("\n" + "="*60)
print("C403 as BOTH len+addr enable? + address in C4EE/C4EF")
print("="*60)

for c403 in [0x01, 0x02, 0x04]:
  test(f"C403=0x{c403:02X} + C4EE=0x02", [(0xC403, c403), (0xC4EE, 0x02)])
  test(f"C403=0x{c403:02X} + C4EF=0x02", [(0xC403, c403), (0xC4EF, 0x02)])

# ============================================================
print("\n" + "="*60)
print("C408/C409/C40C/C40D as addr enable + C4EE/C4EF")
print("="*60)

for en_reg in [0xC408, 0xC409, 0xC40C, 0xC40D]:
  test(f"0x{en_reg:04X}=0x01 + C4EE=0x02", [(en_reg, 0x01), (0xC4EE, 0x02)])

# ============================================================
print("\n" + "="*60)
print("Try CE76-CE79 with various enable bits")
print("="*60)

# CE76-79 = PCI 0x00200400 = F400
ce_addr = [(0xCE76, 0x00), (0xCE77, 0x04), (0xCE78, 0x20), (0xCE79, 0x00)]

test("CE76=F400 only", ce_addr)
test("CE76=F400 + C405=0x01", ce_addr + [(0xC405, 0x01)])
test("CE76=F400 + C405=0x02", ce_addr + [(0xC405, 0x02)])
test("CE76=F400 + C405=0x04", ce_addr + [(0xC405, 0x04)])
test("CE76=F400 + C403=0x01", ce_addr + [(0xC403, 0x01)])
test("CE76=F400 + C408=0x01", ce_addr + [(0xC408, 0x01)])
test("CE76=F400 + C40C=0x01", ce_addr + [(0xC40C, 0x01)])

# ============================================================
print("\n" + "="*60)
print("Address in C420 descriptor area?")
print("  C423-C426 unused in baseline, try as address bytes")
print("="*60)

# C423:C424 as 16-bit address offset from 0x200000?
# 0x0100 offset = F100, 0x0400 = F400
for c423 in [0x01, 0x02, 0x04]:
  test(f"C423=0x{c423:02X}", [(0xC423, c423)])
  test(f"C423=0x{c423:02X} + C405=0x01", [(0xC423, c423), (0xC405, 0x01)])
  test(f"C423=0x{c423:02X} + C403=0x01", [(0xC423, c423), (0xC403, 0x01)])

for c424 in [0x01, 0x02]:
  test(f"C424=0x{c424:02X}", [(0xC424, c424)])
  test(f"C424=0x{c424:02X} + C405=0x01", [(0xC424, c424), (0xC405, 0x01)])

# C425/C426
for c425 in [0x01, 0x02]:
  test(f"C425=0x{c425:02X} + C405=0x01", [(0xC425, c425), (0xC405, 0x01)])
for c426 in [0x01, 0x02]:
  test(f"C426=0x{c426:02X} + C405=0x01", [(0xC426, c426), (0xC405, 0x01)])

# ============================================================
print("\n" + "="*60)
print("C4ED (DMA ctrl) as enable + C4EE/C4EF as addr")
print("="*60)

for c4ed in [0x01, 0x02, 0x04, 0x08, 0x10, 0x80]:
  test(f"C4ED=0x{c4ed:02X} + C4EE=0x02", [(0xC4ED, c4ed), (0xC4EE, 0x02)])

print("\nDone!")
