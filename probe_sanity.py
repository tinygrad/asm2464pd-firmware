#!/usr/bin/env python3
"""
Sanity check on HANDMADE firmware:
1. Does dma_explore ACTUALLY write to F000? Or is F000 stale?
2. Write unique per-test data, CLEAR F000, trigger, verify marker.
3. Also check: does F000 = 0x7000? Maybe we're just reading 0x7000.
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
  return bytes([read8(addr+i) for i in range(n)])

# ============================================================
print("="*60)
print("TEST 1: Is F000 actually SRAM or just aliased to 7000?")
print("="*60)

# Write different values to F000 and 7000, check independently
write(0xF000, 0xAA)
write(0x7000, 0xBB)
f0 = read8(0xF000)
s0 = read8(0x7000)
print(f"  Wrote F000=AA, 7000=BB -> F000=0x{f0:02X} 7000=0x{s0:02X}")
print(f"  Same address? {f0 == s0}")

# ============================================================
print("\n" + "="*60)
print("TEST 2: Does the DMA actually change F000?")
print("="*60)

# Clear F000 completely
for i in range(16):
  write(0xF000+i, 0x00)
print(f"  F000 cleared: {read_range(0xF000, 4).hex()}")

# Bulk out unique data
dev._bulk_out(0x02, bytes([0xDE, 0xAD, 0xBE, 0xEF] + [0]*28))
print(f"  7000 after bulk: {read_range(0x7000, 4).hex()}")
print(f"  F000 before DMA: {read_range(0xF000, 4).hex()}")

# Full dma_explore trigger
for addr, val in [(0xC420,0),(0xC421,1),(0xC422,2),(0xC423,0),
                  (0xC424,0),(0xC425,0),(0xC426,0),(0xC427,1),
                  (0xC428,0x30),(0xC429,0),(0xC42A,0),(0xC42B,0),
                  (0xC42C,0),(0xC42D,0)]:
  write(addr, val)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

print(f"  F000 after DMA:  {read_range(0xF000, 4).hex()}")
print(f"  7000 after DMA:  {read_range(0x7000, 4).hex()}")
write(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 3: Can we write to F100 via CPU?")
print("="*60)

write(0xF100, 0xCC)
print(f"  F100 after write: 0x{read8(0xF100):02X}")
write(0xF100, 0x00)

# ============================================================
print("\n" + "="*60)
print("TEST 4: DMA with C403=4 — how many bytes EXACTLY?")
print("="*60)

# Send 2KB unique pattern
data = bytes([(i//4) & 0xFF for i in range(2048)])
dev._bulk_out(0x02, data)

# Clear F000-F7FF
for i in range(0, 0x800, 4):
  write(0xF000+i, 0x00)

# Full descriptor with C403=4
for addr, val in [(0xC420,0),(0xC421,1),(0xC422,2),(0xC423,0),
                  (0xC424,0),(0xC425,0),(0xC426,0),(0xC427,1),
                  (0xC428,0x30),(0xC429,0),(0xC42A,0),(0xC42B,0),
                  (0xC42C,0),(0xC42D,0)]:
  write(addr, val)
write(0xC403, 0x04)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

# Read every 0x100 offset
print("  DMA with C403=4:")
for off in [0x000, 0x100, 0x200, 0x300, 0x400, 0x500]:
  d = read_range(0xF000+off, 4)
  print(f"    F{off:03X}: {d.hex()}")

write(0xC42A, 0x01)
write(0xC403, 0x00)

# ============================================================
print("\n" + "="*60)
print("TEST 5: Does 7000 actually have >1KB from a 2KB bulk out?")
print("="*60)

data = bytes([0x11]*512 + [0x22]*512 + [0x33]*512 + [0x44]*512)
dev._bulk_out(0x02, data)

for off in [0x000, 0x200, 0x400, 0x600, 0x800]:
  d = read_range(0x7000+off, 4)
  print(f"  7{off:03X}: {d.hex()}")

# ============================================================
print("\n" + "="*60)  
print("TEST 6: DMA with C403=4 on the 2KB data — check what SRAM got")
print("="*60)

for i in range(0, 0x800, 4):
  write(0xF000+i, 0x00)

for addr, val in [(0xC420,0),(0xC421,1),(0xC422,2),(0xC423,0),
                  (0xC424,0),(0xC425,0),(0xC426,0),(0xC427,1),
                  (0xC428,0x30),(0xC429,0),(0xC42A,0),(0xC42B,0),
                  (0xC42C,0),(0xC42D,0)]:
  write(addr, val)
write(0xC403, 0x04)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

for off in [0x000, 0x100, 0x200, 0x300, 0x400, 0x500, 0x600]:
  d = read8(0xF000+off)
  print(f"  F{off:03X}: 0x{d:02X}")

write(0xC42A, 0x01)
write(0xC403, 0x00)

print("\nDone!")
