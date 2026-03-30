#!/usr/bin/env python3
"""
C416-C419: four 8-bit R/W registers. Could be a 32-bit DMA target address.
Test setting them to PCI 0x00200400 (= F400) and see if DMA target moves.
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

def test(label, pre_writes):
  dev._bulk_out(0x02, bytes([0xAA]*32))
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x55)
  
  for addr, val in pre_writes:
    write(addr, val)
  
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  found = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == 0xAA:
      found.append(f"F{off:03X}")
  
  write(0xC42A, 0x01)
  for addr, val in pre_writes:
    write(addr, 0x00)
  
  loc = ', '.join(found) if found else 'NOWHERE'
  moved = found and found[0] != 'F000'
  print(f"  {label:50s}: {loc} {'** MOVED **' if moved else ''}")

# ============================================================
print("="*60)
print("Baseline")
print("="*60)
test("no extra regs", [])

# ============================================================
print("\n" + "="*60)
print("C416-C419 individually")
print("="*60)

for reg in [0xC416, 0xC417, 0xC418, 0xC419]:
  for val in [0x01, 0x02, 0x04, 0x10, 0x20, 0x80]:
    test(f"0x{reg:04X}=0x{val:02X}", [(reg, val)])

# ============================================================
print("\n" + "="*60)
print("C416-C419 as 32-bit LE PCI address (like CE76-CE79)")
print("  PCI 0x00200400 = F400")
print("="*60)

# LE: C416=LSB ... C419=MSB
test("LE: C416=00 C417=04 C418=20 C419=00", 
     [(0xC416, 0x00), (0xC417, 0x04), (0xC418, 0x20), (0xC419, 0x00)])

# BE: C416=MSB ... C419=LSB
test("BE: C416=00 C417=20 C418=04 C419=00",
     [(0xC416, 0x00), (0xC417, 0x20), (0xC418, 0x04), (0xC419, 0x00)])

# Maybe just offset from 0x200000?
test("C416=0x04 (offset 0x400?)", [(0xC416, 0x04)])
test("C417=0x04", [(0xC417, 0x04)])
test("C418=0x04", [(0xC418, 0x04)])
test("C419=0x04", [(0xC419, 0x04)])

# ============================================================
print("\n" + "="*60)
print("C416-C419 with C403 (maybe addr needs size enable)")
print("="*60)

test("C403=4 + C416=04", [(0xC403, 0x04), (0xC416, 0x04)])
test("C403=4 + C417=04", [(0xC403, 0x04), (0xC417, 0x04)])
test("C403=4 + C418=04", [(0xC403, 0x04), (0xC418, 0x04)])
test("C403=4 + C419=04", [(0xC403, 0x04), (0xC419, 0x04)])

test("C403=4 + C416=01", [(0xC403, 0x04), (0xC416, 0x01)])
test("C403=4 + C417=01", [(0xC403, 0x04), (0xC417, 0x01)])
test("C403=4 + C418=01", [(0xC403, 0x04), (0xC418, 0x01)])
test("C403=4 + C419=01", [(0xC403, 0x04), (0xC419, 0x01)])

# LE PCI with C403
test("C403=4 + LE PCI 0x200100",
     [(0xC403, 0x04), (0xC416, 0x00), (0xC417, 0x01), (0xC418, 0x20), (0xC419, 0x00)])
test("C403=4 + LE PCI 0x200400",
     [(0xC403, 0x04), (0xC416, 0x00), (0xC417, 0x04), (0xC418, 0x20), (0xC419, 0x00)])

# ============================================================
print("\n" + "="*60)
print("C416-C419 with C405 (maybe C405 is addr enable)")
print("="*60)

for c405 in [0x01, 0x02, 0x04, 0x08]:
  test(f"C405=0x{c405:02X} + C416=04",
       [(0xC405, c405), (0xC416, 0x04)])
  test(f"C405=0x{c405:02X} + LE PCI 0x200400",
       [(0xC405, c405), (0xC416, 0x00), (0xC417, 0x04), (0xC418, 0x20), (0xC419, 0x00)])

# ============================================================
print("\n" + "="*60)
print("C416-C419 with C403 + C405")
print("="*60)

for c405 in [0x01, 0x02, 0x04]:
  test(f"C403=4 C405=0x{c405:02X} + LE PCI 0x200400",
       [(0xC403, 0x04), (0xC405, c405), (0xC416, 0x00), (0xC417, 0x04), (0xC418, 0x20), (0xC419, 0x00)])

print("\nDone!")
