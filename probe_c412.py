#!/usr/bin/env python3
"""
Test C412 (DMA control/status) effect on DMA.
C412 is PARTIAL writable (mask 0x02 — only bit 1 sticks).
Stock firmware has C412=0x03 after first write.

Also test C413 (PARTIAL mask 0x2A), C414 (R/W), C415 (PARTIAL mask 0x2A).
These are set by stock firmware after first DMA and might be required
for proper multi-sector operation.
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
  if f4 not in [0x00, 0x55]: marker = "** >1KB **"
  
  print(f"  {label:40s}: F000={f0:02X} F200={f2:02X} F400={f4:02X} F600={f6:02X} {marker}")
  
  write(0xC42A, 0x01)
  for addr, val in pre_writes:
    write(addr, 0x00)
  write(0xC403, 0x00)

# ============================================================
print("="*60)
print("Current C410-C41F state")
print("="*60)
d = read_range(0xC410, 16)
print(f"  C410-C41F: {' '.join(f'{d[i]:02X}' for i in range(16))}")

# ============================================================
print("\n" + "="*60)
print("C412 values")
print("="*60)

test("baseline (no C412)", [])
test("C412=0x01 (WRITE_DIR only)", [(0xC412, 0x01)])
test("C412=0x02 (DMA_START only)", [(0xC412, 0x02)])
test("C412=0x03 (both, stock val)", [(0xC412, 0x03)])

# ============================================================
print("\n" + "="*60)
print("C414 values (stock=0x80 after first write)")
print("="*60)

test("C414=0x80 (stock val)", [(0xC414, 0x80)])
test("C414=0x40", [(0xC414, 0x40)])
test("C414=0xC0", [(0xC414, 0xC0)])
test("C414=0xFF", [(0xC414, 0xFF)])

# ============================================================
print("\n" + "="*60)
print("C415 values (stock=0x01)")
print("="*60)

test("C415=0x01", [(0xC415, 0x01)])
test("C415=0x02", [(0xC415, 0x02)])

# ============================================================
print("\n" + "="*60)
print("C413 values (stock=0x01, PARTIAL mask 0x2A)")
print("="*60)

test("C413=0x01", [(0xC413, 0x01)])
test("C413=0x02", [(0xC413, 0x02)])
test("C413=0x0A", [(0xC413, 0x0A)])
test("C413=0x2A", [(0xC413, 0x2A)])

# ============================================================
print("\n" + "="*60)
print("Stock combo: C412=0x03 + C413=0x01 + C414=0x80 + C415=0x01")
print("="*60)

test("all stock values", [(0xC412, 0x03), (0xC413, 0x01), (0xC414, 0x80), (0xC415, 0x01)])
test("stock + C403=0x08", [(0xC412, 0x03), (0xC413, 0x01), (0xC414, 0x80), (0xC415, 0x01)], c403=0x08)
test("stock + C403=0x10", [(0xC412, 0x03), (0xC413, 0x01), (0xC414, 0x80), (0xC415, 0x01)], c403=0x10)
test("stock + C403=0x20", [(0xC412, 0x03), (0xC413, 0x01), (0xC414, 0x80), (0xC415, 0x01)], c403=0x20)

# ============================================================
print("\n" + "="*60)
print("Does C412 BREAK the basic DMA? (regression test)")
print("="*60)

# Test: does writing C412=0x01 break single-sector DMA?
data = bytes([0xEE]*512)
dev._bulk_out(0x02, data)
write(0xF000, 0x00)
write(0xC412, 0x01)
write(0xC42A, 0x00)
write(0xC406, 0x01)
f0 = read8(0xF000)
print(f"  C412=0x01, no C403: F000=0x{f0:02X} ({'OK' if f0==0xEE else 'BROKEN'})")
write(0xC42A, 0x01)
write(0xC412, 0x00)

# Same with C412=0x02
dev._bulk_out(0x02, data)
write(0xF000, 0x00)
write(0xC412, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)
f0 = read8(0xF000)
print(f"  C412=0x02, no C403: F000=0x{f0:02X} ({'OK' if f0==0xEE else 'BROKEN'})")
write(0xC42A, 0x01)
write(0xC412, 0x00)

# C412=0x03
dev._bulk_out(0x02, data)
write(0xF000, 0x00)
write(0xC412, 0x03)
write(0xC42A, 0x00)
write(0xC406, 0x01)
f0 = read8(0xF000)
print(f"  C412=0x03, no C403: F000=0x{f0:02X} ({'OK' if f0==0xEE else 'BROKEN'})")
write(0xC42A, 0x01)
write(0xC412, 0x00)

print("\nDone!")
