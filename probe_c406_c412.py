#!/usr/bin/env python3
"""
Explore C406-C412 area and all writable C4xx registers for multi-sector DMA.
Focus on finding what makes DMA write past 1KB.
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

def test(label, pre_writes, c403=0x04):
  data = bytes([0xAA]*1024 + [0xBB]*1024)
  dev._bulk_out(0x02, data)
  
  for off in [0x000, 0x200, 0x400, 0x600]:
    write(0xF000 + off, 0x00)
  
  for addr, val in pre_writes:
    write(addr, val)
  
  write(0xC403, c403)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  f0 = read8(0xF000)
  f2 = read8(0xF200)
  f4 = read8(0xF400)
  f6 = read8(0xF600)
  
  marker = ""
  if f4 != 0x00 and f4 != 0x55: marker = "** >1KB **"
  
  print(f"  {label:35s}: F000={f0:02X} F200={f2:02X} F400={f4:02X} F600={f6:02X} {marker}")
  
  write(0xC42A, 0x01)
  for addr, val in pre_writes:
    write(addr, 0x00)
  write(0xC403, 0x00)

# ============================================================
print("="*60)
print("Read C400-C41F current state")
print("="*60)
d = read_range(0xC400, 0x20)
for off in range(0, 0x20, 16):
  h = ' '.join(f'{b:02X}' for b in d[off:off+16])
  print(f"  C4{off:02X}: {h}")

# ============================================================
print("\n" + "="*60)
print("C407-C40F values (between trigger and control)")
print("="*60)

for addr in range(0xC407, 0xC410):
  for val in [0x01, 0x02, 0x04, 0x08]:
    test(f"0x{addr:04X}=0x{val:02X}", [(addr, val)])

# ============================================================
print("\n" + "="*60)
print("C410-C41F values")
print("="*60)

for addr in range(0xC410, 0xC420):
  for val in [0x01, 0x02, 0x04, 0x08]:
    test(f"0x{addr:04X}=0x{val:02X}", [(addr, val)])

# ============================================================
print("\n" + "="*60)
print("C403 bigger values (0x08, 0x10, 0x20, 0x40, 0x80, 0xFF)")
print("="*60)

for c403 in [0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]:
  test(f"C403=0x{c403:02X}", [], c403=c403)

# ============================================================
print("\n" + "="*60)
print("C405 values")
print("="*60)

for val in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x80, 0xFF]:
  test(f"C405=0x{val:02X}", [(0xC405, val)])

# ============================================================
print("\n" + "="*60)
print("Combos: C403 + other regs")
print("="*60)

test("C403=4 + C405=4", [(0xC405, 0x04)], c403=0x04)
test("C403=8 + C405=8", [(0xC405, 0x08)], c403=0x08)
test("C403=4 + C408=1", [(0xC408, 0x01)], c403=0x04)
test("C403=4 + C409=1", [(0xC409, 0x01)], c403=0x04)
test("C403=4 + C40C=1", [(0xC40C, 0x01)], c403=0x04)
test("C403=4 + C40D=1", [(0xC40D, 0x01)], c403=0x04)
test("C403=4 + C414=0x80", [(0xC414, 0x80)], c403=0x04)
test("C403=4 + C416=1", [(0xC416, 0x01)], c403=0x04)
test("C403=4 + C417=1", [(0xC417, 0x01)], c403=0x04)
test("C403=4 + C418=1", [(0xC418, 0x01)], c403=0x04)
test("C403=4 + C419=1", [(0xC419, 0x01)], c403=0x04)

# ============================================================
print("\n" + "="*60)
print("Non-LEN_EN writable regs in C420-C4FF")
print("="*60)

# These were found writable: C420-C427, C438-C44B, C4C0-C4C9, C4D8-C4DB
for addr, val, label in [
  (0xC420, 0x04, "C420=0x04 (xfer_hi)"),
  (0xC420, 0x08, "C420=0x08"),
  (0xC421, 0x04, "C421=0x04 (xfer_lo, stock 4KB value)"),
  (0xC421, 0x08, "C421=0x08"),
  (0xC421, 0x10, "C421=0x10"),
  (0xC427, 0x08, "C427=0x08 (sector count)"),
  (0xC427, 0x10, "C427=0x10"),
  (0xC438, 0x00, "C438=0x00 (init ctrl, default FF)"),
  (0xC448, 0x00, "C448=0x00 (init ctrl2, default FF)"),
  (0xC4C0, 0x08, "C4C0=0x08 (SCSI cmd buf)"),
]:
  test(label, [(addr, val)])

print("\nDone!")
