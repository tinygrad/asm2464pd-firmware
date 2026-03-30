#!/usr/bin/env python3
"""
Find how to change the C400 DMA target address.
The key insight: maybe C8B0-C8D9 DMA engine has channel config.
Or maybe we need to look at the 0x8000 window base address.
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

def dma_fire():
  write(0xC42A, 0x00)
  write(0xC406, 0x01)

def dma_unlock():
  write(0xC42A, 0x01)

# Step 1: Look at CC10-CC3F (CPU/memory config)
# CC2A=0x02 could be page shift. CC2E=0x03 interesting.
print("="*60)
print("CC00-CC3F area (memory/CPU config)")
print("="*60)
d = read_range(0xCC00, 0x40)
for off in range(0, 0x40, 16):
  chunk = d[off:off+16]
  nz = ' '.join(f'{b:02X}' for b in chunk)
  print(f"  0x{0xCC00+off:04X}: {nz}")

# Step 2: Can we remap the F000 window?
# Look for the F000 window base in registers
# From FUNCTIONALITY.md: the 0xF000 window is at PCI 0x00200000
# Maybe there's a register that holds the 0x0020 (high word)?
print("\n" + "="*60)
print("Looking for 0x20 in register space (SRAM base high byte)")
print("="*60)

# Check C8B0-C8DF
d = read_range(0xC8B0, 0x30)
for i in range(len(d)):
  if d[i] == 0x20:
    print(f"  C8{0xB0+i:02X} = 0x20")

# Check CC00-CC3F
d = read_range(0xCC00, 0x40)
for i in range(len(d)):
  if d[i] == 0x20:
    print(f"  CC{i:02X} = 0x20")

# Step 3: Instead of moving the DMA target, can we remap what 
# the 0xF000 XDATA window points to?
# The window maps 4KB of XDATA to SRAM. If we can change the
# SRAM base address of the window, we effectively change what
# the DMA "appears" to write to from the 8051's perspective.
# But we need the GPU to see it too...

# Step 4: Try the 0x8000 window - does it have a configurable base?
# Check if 9xxx registers control the 0x8000 window offset
print("\n" + "="*60)
print("0x8000 window vs 0xF000 window")
print("="*60)

# Write to different SRAM offsets via 8000 and read from F000
write(0x8000, 0x11)
write(0x8001, 0x22)
print(f"  8000: {read_range(0x8000, 4).hex()}")
print(f"  F000: {read_range(0xF000, 4).hex()}")

# Maybe the 8000 window base is at 0x200000 too but offset differently?
# Try: write to 0x8100 and check F100
write(0x8100, 0x33)
write(0x8101, 0x44)
print(f"  8100: {read_range(0x8100, 4).hex()}")
print(f"  F100: {read_range(0xF100, 4).hex()}")

# Step 5: The real answer might be simpler than we think.
# C400 DMA copies 0x7000 to SRAM. Maybe the DMA writes
# sequentially starting from wherever the SRAM pointer is.
# After dma_explore, CE10=0x200000. After each DMA,
# CE10 advances. Can we do multiple small DMAs to fill
# different SRAM offsets?
print("\n" + "="*60)
print("Multi-DMA: does CE10 advance on C400 path?")
print("="*60)

ce10 = read_range(0xCE10, 4)
print(f"  CE10 before: {ce10.hex()}")

# DMA 1
dev._bulk_out(0x02, bytes([0xAA]*32))
dma_fire()
ce10 = read_range(0xCE10, 4)
print(f"  CE10 after DMA 1: {ce10.hex()}")
dma_unlock()

# DMA 2
dev._bulk_out(0x02, bytes([0xBB]*32))
dma_fire()
ce10 = read_range(0xCE10, 4)
print(f"  CE10 after DMA 2: {ce10.hex()}")
dma_unlock()

print(f"  F000: {read_range(0xF000, 4).hex()} (expect AA)")
print(f"  F200: {read_range(0xF200, 4).hex()} (BB if pointer advanced)")

# Step 6: The SRAM has 6MB. The C400 DMA always targets base 0x200000.
# tinygrad reads via E4 from 0xF000 which is 0x200000.
# What if we just need to use C427 for multi-sector and accept
# that data always starts at 0x200000?
# The GPU reads from PCIe 0x200000+offset.
# We just need the 8051 to read it back at F000+offset.

# Actually, looking at it differently:
# The whole point is tinygrad writes data to SRAM, and the GPU 
# reads it via PCIe. The 8051 doesn't need to read past F000+FFF.
# The DMA target 0x200000 is fine. The question is whether
# subsequent DMA writes go to 0x200000 every time or advance.

print("\nDone!")
