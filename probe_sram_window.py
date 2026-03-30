#!/usr/bin/env python3
"""
The C400 DMA always targets PCI 0x200000 (XDATA 0xF000 window).
But we need to target arbitrary SRAM addresses.

Options:
  1. Move the F000 window to a different PCI base address
  2. Use the 8000 window (also 4KB into SRAM)
  3. Find the actual DMA target address register

Let's probe the SRAM window configuration.
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

# First, check: does the 8000 window show the same data as F000?
print("="*60)
print("SRAM window comparison: 0x8000 vs 0xF000")
print("="*60)

f0 = read_range(0xF000, 16)
e0 = read_range(0x8000, 16)
print(f"  F000: {f0.hex()}")
print(f"  8000: {e0.hex()}")
print(f"  Same: {f0 == e0}")

# Check if writing to 8000 appears at F000
write(0x8000, 0xAA)
write(0x8001, 0xBB)
f0 = read_range(0xF000, 4)
e0 = read_range(0x8000, 4)
print(f"\n  After write 0x8000=0xAA,0xBB:")
print(f"  F000: {f0.hex()}")
print(f"  8000: {e0.hex()}")

# Check the SRAM window control registers
# From registers.h:
# 0x8000 = USB data buffer (4KB window into SRAM at PCI 0x00200000+)
# 0xF000 = NVMe data buffer (4KB window into SRAM at PCI 0x00200000+)
# They might have separate base address registers

# Look for window base address registers in C8xx area (DMA engine)
print("\n" + "="*60)
print("Searching for SRAM window base address registers")
print("="*60)

# C8B0-C8D9 is the "DMA mode, channel control" area
d = read_range(0xC8B0, 0x30)
print(f"  C8B0-C8DF: {' '.join(f'{b:02X}' for b in d)}")

# C900-C9FF area
d = read_range(0xC900, 0x40)
print(f"  C900-C93F:")
for off in range(0, 0x40, 16):
  chunk = d[off:off+16]
  nz = ' '.join(f'{b:02X}' for b in chunk)
  print(f"    0x{0xC900+off:04X}: {nz}")

# Check around CC00 (CPU mode area)
print("\nCC00-CC3F:")
d = read_range(0xCC00, 0x40)
for off in range(0, 0x40, 16):
  chunk = d[off:off+16]
  if any(b != 0 for b in chunk):
    nz = ' '.join(f'{b:02X}' for b in chunk)
    print(f"    0x{0xCC00+off:04X}: {nz}")

# The 8000 and F000 windows likely share the same PCI base.
# What if we use C400 DMA to write to offset > 0x1000 in SRAM?
# The DMA might write sequentially starting at 0x200000.
# If we do multiple sector DMA, sectors 2+ go to 0x200200, 0x200400...

print("\n" + "="*60)
print("TEST: Multi-sector DMA - does data go past F000+0x200?")
print("="*60)

# Write 2 sectors (1024 bytes) — data should fill F000-F3FF
write(0xC427, 0x02)  # 2 sectors
pat = bytes([0x10*i + j for j in range(16)] + [0]*16 for i in range(32))
# Actually need 1024 bytes of data in 0x7000
# bulk_out only sends 32 bytes. We need larger bulk out.
# For 2 sectors, we need 1024 bytes at 0x7000.
# The handmade firmware's bulk EP is 64 bytes max packet size.
# Let's try sending multiple bulk outs.

# First, fill 0x7000-0x71FF with known pattern via E5
for i in range(0, 512, 1):
  write(0x7000 + i, i & 0xFF)

print(f"  7000: {read_range(0x7000, 8).hex()}")
print(f"  7100: {read_range(0x7100, 8).hex()}")
print(f"  7200: {read_range(0x7200, 4).hex()}")

# Now DMA 2 sectors
write(0xC427, 0x02)
write(0xC42A, 0x00)
write(0xC406, 0x01)

print(f"  F000: {read_range(0xF000, 8).hex()}")
print(f"  F100: {read_range(0xF100, 8).hex()}")
print(f"  F200: {read_range(0xF200, 4).hex()}")
write(0xC42A, 0x01)

# Now do 4 sectors
write(0xC427, 0x04)
# fill 0x7000-0x7800
for i in range(0, 2048):
  write(0x7000 + i, (i + 0x80) & 0xFF)

write(0xC42A, 0x00)
write(0xC406, 0x01)
print(f"\n  4 sectors:")
print(f"  F000: {read_range(0xF000, 4).hex()}")
print(f"  F200: {read_range(0xF200, 4).hex()}")
print(f"  F400: {read_range(0xF400, 4).hex()}")
print(f"  F600: {read_range(0xF600, 4).hex()}")
print(f"  F800: {read_range(0xF800, 4).hex()}")
write(0xC42A, 0x01)

print("\nDone!")
