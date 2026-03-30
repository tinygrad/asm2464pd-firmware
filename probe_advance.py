#!/usr/bin/env python3
"""
On handmade firmware: figure out how to advance the SRAM write pointer
so successive C400 DMAs write to increasing addresses.

Currently: every C400 DMA overwrites PCI 0x200000 (F000).
Need: 1st DMA -> 0x200000, 2nd -> 0x200200, 3rd -> 0x200400, etc.

Hypotheses:
  1. C427 (sector count) > 1 might auto-advance within one DMA
  2. NOT resetting C42A between DMAs might advance
  3. Some other register controls the C400 write pointer
  4. We need to use CE6E/CE6F to advance (like stock firmware)
"""
import sys, ctypes
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0, f"read8(0x{addr:04X}) failed: {ret}"
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

def read_range(addr, n):
  return bytes([read8(addr + i) for i in range(n)])

def dma_full():
  """Full descriptor + trigger (from dma_explore)."""
  write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
  write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
  write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
  write(0xC429, 0x00); write(0xC42A, 0x00); write(0xC42B, 0x00)
  write(0xC42C, 0x00); write(0xC42D, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1)
  write(0xC404, 1); write(0xC406, 1)

def dma_minimal():
  """Minimal trigger."""
  write(0xC42A, 0x00)
  write(0xC406, 0x01)

def dma_unlock():
  write(0xC42A, 0x01)

def check_sram():
  """Read first 4 bytes at each 512-byte boundary in F000-FFFF."""
  results = {}
  for off in range(0, 0x1000, 0x200):
    results[off] = read_range(0xF000 + off, 4)
  return results

# ============================================================
print("="*60)
print("TEST 1: Two DMAs with unlock between — both go to F000?")
print("="*60)

# First DMA: AA pattern
dev._bulk_out(0x02, bytes([0xAA]*32))
dma_full()
dma_unlock()

# Second DMA: BB pattern
dev._bulk_out(0x02, bytes([0xBB]*32))
dma_full()
dma_unlock()

sram = check_sram()
print(f"  F000: {sram[0].hex()} (expect BB — 2nd overwrote 1st)")
print(f"  F200: {sram[0x200].hex()} (expect 5555 — not written)")

# ============================================================
print("\n" + "="*60)
print("TEST 2: Two DMAs WITHOUT unlock between")
print("="*60)

dev._bulk_out(0x02, bytes([0xCC]*32))
dma_full()
# NO unlock!

dev._bulk_out(0x02, bytes([0xDD]*32))
# Can we even trigger again without unlock?
write(0xC42A, 0x00)
write(0xC406, 0x01)

sram = check_sram()
print(f"  F000: {sram[0].hex()}")
print(f"  F200: {sram[0x200].hex()}")
dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST 3: C427 sector count > 1")
print("  Set C427=2, send 1024 bytes, see if F200 gets data")
print("="*60)

# Fill 0x7000 with known pattern via CPU
for i in range(0x400):  # 1024 bytes
  write(0x7000 + i, i & 0xFF)

print(f"  7000: {read_range(0x7000, 4).hex()}")
print(f"  7200: {read_range(0x7200, 4).hex()}")

# DMA 2 sectors
write(0xC427, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)

sram = check_sram()
print(f"  F000: {sram[0].hex()} (first sector)")
print(f"  F200: {sram[0x200].hex()} (second sector — data here?)")
print(f"  F400: {sram[0x400].hex()} (should be empty)")
dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST 4: Bulk OUT larger packet, then multi-sector DMA")
print("  Send 1024 bytes via bulk, DMA 2 sectors")
print("="*60)

# Send larger bulk: 64 bytes at a time (handmade EP max)
for chunk in range(0, 1024, 64):
  pkt = bytes([(chunk + i) & 0xFF for i in range(64)])
  # Note: only the first bulk_out lands at 0x7000.
  # Subsequent ones might not work without re-arming.
  # But let's try:
  if chunk == 0:
    dev._bulk_out(0x02, pkt)
  # Can't do more without firmware support

# Check 0x7000
print(f"  7000: {read_range(0x7000, 8).hex()}")

write(0xC427, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)

sram = check_sram()
print(f"  F000: {sram[0].hex()}")
print(f"  F200: {sram[0x200].hex()}")
dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST 5: Repeated DMA with different C427 values")
print("="*60)

# Send unique data via CPU write to 0x7000
for i in range(0x800):
  write(0x7000 + i, (0x50 + (i >> 9)) & 0xFF)  # 0x50 for first 512, 0x51 for next

for sectors in [1, 2, 4, 8]:
  write(0xC427, sectors)
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  
  results = []
  for off in range(0, sectors * 512, 512):
    d = read_range(0xF000 + off, 1)
    results.append(f"F{off:03X}={d[0]:02X}")
  
  print(f"  C427={sectors}: {' '.join(results)}")
  dma_unlock()

# ============================================================
print("\n" + "="*60)
print("TEST 6: Does CE6F advance the C400 write pointer?")
print("  On stock firmware, writing CE6F!=0 advanced CE10.")
print("="*60)

# Put known data at 7000
for i in range(512):
  write(0x7000 + i, 0xEE)

# DMA to F000
write(0xC427, 0x01)
write(0xC42A, 0x00)
write(0xC406, 0x01)
dma_unlock()
print(f"  F000: {read_range(0xF000, 4).hex()} (expect EE)")

# Now try to advance pointer via CE6F
write(0xCE6F, 0x01)
print(f"  CE10 after CE6F=1: {read_range(0xCE10, 4).hex()}")

# DMA again with different data
for i in range(512):
  write(0x7000 + i, 0xFF)

write(0xC427, 0x01)
write(0xC42A, 0x00)
write(0xC406, 0x01)
dma_unlock()

print(f"  F000: {read_range(0xF000, 4).hex()} (EE if pointer advanced, FF if overwrote)")
print(f"  F200: {read_range(0xF200, 4).hex()} (FF if pointer advanced)")

# Reset
write(0xCE6E, 0x00)
write(0xCE6F, 0x00)

print("\nDone!")
