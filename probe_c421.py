#!/usr/bin/env python3
"""
Test if setting C421 along with C427 enables multi-sector DMA.
Stock firmware sets C421=pages(?), C427=sectors for multi-sector transfers.
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

def dma_trigger_full(sectors, c421=None):
  """Full descriptor + trigger with given sector count."""
  write(0xC420, 0x00)
  write(0xC421, c421 if c421 is not None else 0x01)
  write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, sectors)
  write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)

def dma_unlock():
  write(0xC42A, 0x01)

# ============================================================
print("="*60)
print("Send 1KB bulk OUT, verify 0x7000 has both sectors")
print("="*60)

data = bytes([0xAA]*512 + [0xBB]*512)
dev._bulk_out(0x02, data)

print(f"  7000: {read_range(0x7000, 4).hex()} (expect AA)")
print(f"  7200: {read_range(0x7200, 4).hex()} (expect BB)")

# ============================================================
print("\n" + "="*60)
print("TEST: Various C421/C427 combos for 2-sector DMA")
print("="*60)

combos = [
  (0x01, 0x02, "C421=0x01, C427=0x02"),
  (0x02, 0x02, "C421=0x02, C427=0x02"),
  (0x01, 0x01, "C421=0x01, C427=0x01 (baseline 1 sector)"),
  (0x04, 0x02, "C421=0x04, C427=0x02"),
  (0x00, 0x02, "C421=0x00, C427=0x02"),
  (0x02, 0x01, "C421=0x02, C427=0x01"),
]

for c421, c427, label in combos:
  # Re-send data
  dev._bulk_out(0x02, data)
  
  # Clear F000, F200
  write(0xF000, 0x00); write(0xF001, 0x00)
  write(0xF200, 0x00); write(0xF201, 0x00)
  
  dma_trigger_full(c427, c421)
  
  f0 = read_range(0xF000, 2)
  f2 = read_range(0xF200, 2)
  
  print(f"  {label}: F000={f0.hex()} F200={f2.hex()}")
  dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST: Try different C420 values (xfer count hi)")
print("="*60)

for c420 in [0x00, 0x01, 0x02, 0x04, 0x08]:
  dev._bulk_out(0x02, data)
  write(0xF000, 0x00); write(0xF001, 0x00)
  write(0xF200, 0x00); write(0xF201, 0x00)
  
  write(0xC420, c420)
  write(0xC421, 0x01)
  write(0xC427, 0x02)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  f0 = read_range(0xF000, 2)
  f2 = read_range(0xF200, 2)
  print(f"  C420=0x{c420:02X}: F000={f0.hex()} F200={f2.hex()}")
  dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST: Use the exact stock values for 2-sector (1KB)")
print("  Stock 1KB: C420=0x00 C421=0x01 C427=0x02")
print("="*60)

dev._bulk_out(0x02, data)
write(0xF000, 0x00); write(0xF001, 0x00)
write(0xF200, 0x00); write(0xF201, 0x00)

# Exact stock descriptor
write(0xC420, 0x00)
write(0xC421, 0x01)  # stock uses 0x01 for 1KB
write(0xC422, 0x02)
write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
write(0xC426, 0x00)
write(0xC427, 0x02)  # 2 sectors
write(0xC428, 0x30)
write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
write(0xC42C, 0x00); write(0xC42D, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
write(0xC404, 1); write(0xC406, 1)

f0 = read_range(0xF000, 4)
f2 = read_range(0xF200, 4)
print(f"  F000: {f0.hex()} (expect AA)")
print(f"  F200: {f2.hex()} (expect BB if multi-sector works)")
dma_unlock()

# Also check: is 0x7000 locked after trigger?
buf = read_range(0x7000, 4)
print(f"  7000 after unlock: {buf.hex()}")

# ============================================================
print("\n" + "="*60)
print("TEST: Trigger C400 twice with different C427=1 — pointer advance?")
print("="*60)

# First trigger
dev._bulk_out(0x02, bytes([0x11]*512 + [0x22]*512))
write(0xF000, 0x00); write(0xF200, 0x00)

write(0xC427, 0x01)
write(0xC42A, 0x00); write(0xC406, 0x01)
f0_1 = read_range(0xF000, 2)
f2_1 = read_range(0xF200, 2)
print(f"  1st trigger C427=1: F000={f0_1.hex()} F200={f2_1.hex()}")

# DON'T unlock — try 2nd trigger immediately
write(0xC427, 0x01)
write(0xC42A, 0x00); write(0xC406, 0x01)
f0_2 = read_range(0xF000, 2)
f2_2 = read_range(0xF200, 2)
print(f"  2nd trigger (no unlock): F000={f0_2.hex()} F200={f2_2.hex()}")
dma_unlock()

print("\nDone!")
