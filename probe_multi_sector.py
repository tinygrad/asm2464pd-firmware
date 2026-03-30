#!/usr/bin/env python3
"""
On HANDMADE firmware: test multi-sector C400 DMA.

The handmade firmware has 64-byte bulk EP max packet size.
But dma_explore sends 32-byte packets and it works for 1 sector.

Questions:
  1. Does C427>1 work when 0x7000 has >512 bytes from bulk OUT?
  2. How much data can accumulate at 0x7000 from multiple bulk OUTs?
  3. Can we send multiple bulk OUT packets and then DMA all at once?
"""
import sys, ctypes, time
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0, f"read8 failed: {ret}"
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0, f"write failed: {ret}"

def read_range(addr, n):
  return bytes([read8(addr + i) for i in range(n)])

# First prime with dma_explore sequence
def dma_explore_prime():
  """Run one full dma_explore cycle to initialize descriptor."""
  pat = bytes([0x55]*32)
  dev._bulk_out(0x02, pat)
  # Full descriptor
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)
  write(0xC42A, 0x01)

print("Priming DMA descriptor...")
dma_explore_prime()
print(f"  F000: {read_range(0xF000, 4).hex()} (should be 55)")

# ============================================================
print("\n" + "="*60)
print("TEST 1: Single bulk OUT (32 bytes), C427=1 — baseline")
print("="*60)

dev._bulk_out(0x02, bytes([0xAA]*32))
write(0xC427, 0x01)
write(0xC42A, 0x00); write(0xC406, 0x01)
print(f"  F000: {read_range(0xF000, 4).hex()}")
write(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 2: Multiple bulk OUTs, then C427=2 DMA")
print("  Send 2 x 512-byte bulk OUTs, then trigger 2-sector DMA")
print("="*60)

# Send 512 bytes of 0xBB
for i in range(8):  # 8 x 64 = 512 bytes
  dev._bulk_out(0x02, bytes([0xBB]*64))

# Check 0x7000
print(f"  7000: {read_range(0x7000, 4).hex()}")
print(f"  7200: {read_range(0x7200, 4).hex()}")

# Now send 512 bytes of 0xCC (total 1024 at 0x7000?)
for i in range(8):
  dev._bulk_out(0x02, bytes([0xCC]*64))

print(f"  7000: {read_range(0x7000, 4).hex()}")
print(f"  7200: {read_range(0x7200, 4).hex()}")

# DMA 2 sectors
write(0xC427, 0x02)
write(0xC42A, 0x00); write(0xC406, 0x01)
print(f"  F000: {read_range(0xF000, 4).hex()}")
print(f"  F200: {read_range(0xF200, 4).hex()}")
write(0xC42A, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 3: How much data accumulates at 0x7000?")
print("  Send increasing amounts and check 0x7000+offset")
print("="*60)

# Clear test: send 64 bytes unique pattern
dev._bulk_out(0x02, bytes(range(64)))
print(f"  After 1 pkt: 7000={read_range(0x7000, 4).hex()} 7040={read_range(0x7040, 4).hex()}")

dev._bulk_out(0x02, bytes(range(64, 128)))
print(f"  After 2 pkt: 7000={read_range(0x7000, 4).hex()} 7040={read_range(0x7040, 4).hex()}")

# Read more of 0x7000 buffer to see accumulation
print(f"  7000-7010: {read_range(0x7000, 16).hex()}")
print(f"  7040-7050: {read_range(0x7040, 16).hex()}")
print(f"  7080-7090: {read_range(0x7080, 16).hex()}")

# ============================================================
print("\n" + "="*60)
print("TEST 4: Large single bulk OUT")
print("  Try sending >64 bytes in one shot")
print("="*60)

# Try 512 bytes in one bulk transfer
try:
  dev._bulk_out(0x02, bytes([0xDD]*512))
  print(f"  After 512B bulk: 7000={read_range(0x7000, 4).hex()}")
  print(f"  7100={read_range(0x7100, 4).hex()}")
  print(f"  71FF={read_range(0x71FF, 1).hex()}")
except Exception as e:
  print(f"  512B bulk failed: {e}")

# Try 1024 bytes
try:
  dev._bulk_out(0x02, bytes([0xEE]*1024))
  print(f"  After 1024B bulk: 7000={read_range(0x7000, 4).hex()}")
  print(f"  7200={read_range(0x7200, 4).hex()}")
except Exception as e:
  print(f"  1024B bulk failed: {e}")

# ============================================================
print("\n" + "="*60)
print("TEST 5: DMA after large bulk OUT")
print("="*60)

# Send 1KB unique pattern
data = bytes([(i >> 2) & 0xFF for i in range(1024)])
try:
  dev._bulk_out(0x02, data)
  print(f"  7000: {read_range(0x7000, 4).hex()}")
  print(f"  7200: {read_range(0x7200, 4).hex()}")
  
  # DMA 2 sectors
  write(0xC427, 0x02)
  write(0xC42A, 0x00); write(0xC406, 0x01)
  
  print(f"  F000: {read_range(0xF000, 4).hex()} (expect {data[:4].hex()})")
  print(f"  F200: {read_range(0xF200, 4).hex()} (expect {data[512:516].hex()})")
  write(0xC42A, 0x01)
except Exception as e:
  print(f"  Failed: {e}")

print("\nDone!")
