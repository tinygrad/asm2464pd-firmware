#!/usr/bin/env python3
"""
C403 = DMA length enable. Without it, only 0x10 bytes get DMAed.
Test: C403=1 with C427=2 for multi-sector DMA.
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

def dma_unlock():
  write(0xC42A, 0x01)

def test_dma(label, sectors, use_c403=False, c403_val=1):
  # Send unique data per sector
  data = b''
  for s in range(sectors):
    data += bytes([0x10*s + (s+1)]*512)
  dev._bulk_out(0x02, data)
  
  # Clear targets
  for off in range(0, sectors*512, 512):
    write(0xF000 + off, 0x00)
    write(0xF001 + off, 0x00)
  
  # Full descriptor
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, sectors); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  
  if use_c403:
    write(0xC403, c403_val)
  
  # Arm and trigger
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)
  
  # Check results
  results = []
  for s in range(sectors):
    off = s * 0x200
    d = read_range(0xF000 + off, 2)
    expected = bytes([0x10*s + (s+1)]*2)
    ok = d == expected
    results.append(f"F{off:03X}={d.hex()}{'*' if ok else ''}")
  
  print(f"  {label}: {' '.join(results)}")
  dma_unlock()
  if use_c403:
    write(0xC403, 0x00)  # restore

# ============================================================
print("="*60)
print("TEST: C403 effect on DMA length")
print("="*60)

# Baseline without C403
test_dma("C427=1, no C403", 1, use_c403=False)
test_dma("C427=2, no C403", 2, use_c403=False)

# With C403=1
test_dma("C427=1, C403=1", 1, use_c403=True, c403_val=1)
test_dma("C427=2, C403=1", 2, use_c403=True, c403_val=1)
test_dma("C427=4, C403=1", 4, use_c403=True, c403_val=1)
test_dma("C427=8, C403=1", 8, use_c403=True, c403_val=1)

# ============================================================
print("\n" + "="*60)
print("TEST: Different C403 values")
print("="*60)

for c403 in [0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x80, 0xFF]:
  test_dma(f"C403=0x{c403:02X}, C427=2", 2, use_c403=True, c403_val=c403)

# ============================================================
print("\n" + "="*60)
print("TEST: Verify full 512 bytes per sector (not just 0x10)")
print("="*60)

# Send 512 bytes with unique pattern
data = bytes(range(256)) + bytes(range(256))
dev._bulk_out(0x02, data)

# Without C403
write(0xC427, 0x01); write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

print("  Without C403:")
print(f"    F000: {read_range(0xF000, 16).hex()}")
print(f"    F010: {read_range(0xF010, 16).hex()}")
print(f"    F020: {read_range(0xF020, 4).hex()}")
print(f"    F100: {read_range(0xF100, 4).hex()}")
dma_unlock()

# With C403=1
dev._bulk_out(0x02, data)
# Clear
for i in range(32): write(0xF000+i, 0x55)

write(0xC403, 0x01)
write(0xC427, 0x01); write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

print("\n  With C403=1:")
print(f"    F000: {read_range(0xF000, 16).hex()}")
print(f"    F010: {read_range(0xF010, 16).hex()}")
print(f"    F020: {read_range(0xF020, 4).hex()}")
print(f"    F100: {read_range(0xF100, 4).hex()}")
print(f"    F1F0: {read_range(0xF1F0, 4).hex()}")
dma_unlock()
write(0xC403, 0x00)

print("\nDone!")
