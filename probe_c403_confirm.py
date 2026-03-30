#!/usr/bin/env python3
"""
Confirm: C403 bit 1 enables multi-sector C400 DMA.
Test with 2, 4, 8 sectors and verify data at each offset.
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

def multi_sector_dma(sectors, c403=0x03):
  """Send unique data per sector, DMA with C403, verify."""
  # Build data: sector N has byte value N+1
  data = b''
  for s in range(sectors):
    data += bytes([s + 1]*512)
  
  dev._bulk_out(0x02, data)
  
  # Full descriptor with C403
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, sectors); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC403, c403)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)
  
  # Check each sector
  all_ok = True
  for s in range(sectors):
    off = s * 0x200
    d = read_range(0xF000 + off, 1)
    expected = s + 1
    ok = d[0] == expected
    if not ok: all_ok = False
    print(f"    F{off:03X}: 0x{d[0]:02X} (expect 0x{expected:02X}) {'OK' if ok else 'FAIL'}")
  
  write(0xC42A, 0x01)  # unlock
  write(0xC403, 0x00)  # restore
  return all_ok

# ============================================================
print("="*60)
print("2-sector DMA with C403=0x03")
print("="*60)
multi_sector_dma(2)

# ============================================================
print("\n" + "="*60)
print("4-sector DMA (2KB) with C403=0x03")
print("="*60)
multi_sector_dma(4)

# ============================================================
print("\n" + "="*60)
print("8-sector DMA (4KB) with C403=0x03")
print("="*60)
multi_sector_dma(8)

# ============================================================
print("\n" + "="*60)
print("Exact C403 bit test")
print("="*60)

for c403 in [0x01, 0x02, 0x03, 0x04]:
  data = bytes([0xAA]*512 + [0xBB]*512)
  dev._bulk_out(0x02, data)
  
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x02); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC403, c403)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)
  
  f0 = read_range(0xF000, 1)[0]
  f2 = read_range(0xF200, 1)[0]
  print(f"  C403=0x{c403:02X}: F000=0x{f0:02X} F200=0x{f2:02X} {'MULTI' if f2==0xBB else 'single'}")
  write(0xC42A, 0x01)
  write(0xC403, 0x00)

# ============================================================
print("\n" + "="*60)
print("Minimal trigger with C403: just C403 + C427 + C42A + C406?")
print("="*60)

data = bytes([0x11]*512 + [0x22]*512)
dev._bulk_out(0x02, data)

write(0xC403, 0x03)
write(0xC427, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)

f0 = read_range(0xF000, 1)[0]
f2 = read_range(0xF200, 1)[0]
print(f"  Minimal: F000=0x{f0:02X} F200=0x{f2:02X} {'MULTI' if f2==0x22 else 'single'}")
write(0xC42A, 0x01)
write(0xC403, 0x00)

print("\nDone!")
