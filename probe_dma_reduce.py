#!/usr/bin/env python3
"""
Find the MINIMUM register writes needed for C400 DMA.
Start from the full dma_explore sequence and remove registers one at a time.
"""
import sys, ctypes, random
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

def dma_unlock():
  write(0xC42A, 0x01)

def test_dma(label, desc_writes, arm_writes):
  """Test a DMA transfer with given descriptor and arm writes."""
  pat = bytes(random.getrandbits(8) for _ in range(32))
  dev._bulk_out(0x02, pat)
  for addr, val in desc_writes:
    write(addr, val)
  for addr, val in arm_writes:
    write(addr, val)
  f = read_range(0xF000, 4)
  ok = f == pat[:4]
  print(f"  {label}: {'OK' if ok else 'FAIL'}")
  dma_unlock()
  return ok

# Full descriptor (known working)
FULL_DESC = [
  (0xC420, 0x00), (0xC421, 0x01), (0xC422, 0x02), (0xC423, 0x00),
  (0xC424, 0x00), (0xC425, 0x00), (0xC426, 0x00), (0xC427, 0x01),
  (0xC428, 0x30), (0xC429, 0x00), (0xC42A, 0x00), (0xC42B, 0x00),
  (0xC42C, 0x00), (0xC42D, 0x00),
]
FULL_ARM = [
  (0xC400, 1), (0xC401, 1), (0xC402, 1), (0xC404, 1), (0xC406, 1),
]

# Verify full works
print("Full descriptor + arm:")
test_dma("full", FULL_DESC, FULL_ARM)

# Test removing each descriptor register
print("\nRemoving descriptor registers one at a time:")
for skip_idx in range(len(FULL_DESC)):
  addr, val = FULL_DESC[skip_idx]
  reduced = FULL_DESC[:skip_idx] + FULL_DESC[skip_idx+1:]
  test_dma(f"skip 0x{addr:04X}={val:02X}", reduced, FULL_ARM)

# Test removing arm bits
print("\nRemoving arm registers one at a time:")
for skip_idx in range(len(FULL_ARM)):
  addr, val = FULL_ARM[skip_idx]
  reduced = FULL_ARM[:skip_idx] + FULL_ARM[skip_idx+1:]
  test_dma(f"skip 0x{addr:04X}", FULL_DESC, reduced)

# Test minimal: just sector count + arm + trigger
print("\nMinimal combinations:")
test_dma("C427+C42A+arm+trigger",
  [(0xC427, 0x01), (0xC42A, 0x00)],
  [(0xC400, 1), (0xC401, 1), (0xC402, 1), (0xC404, 1), (0xC406, 1)])

test_dma("C427+arm+trigger (no C42A)",
  [(0xC427, 0x01)],
  [(0xC400, 1), (0xC401, 1), (0xC402, 1), (0xC404, 1), (0xC406, 1)])

test_dma("just C42A+arm+trigger",
  [(0xC42A, 0x00)],
  [(0xC400, 1), (0xC401, 1), (0xC402, 1), (0xC404, 1), (0xC406, 1)])

test_dma("just arm+trigger",
  [],
  [(0xC400, 1), (0xC401, 1), (0xC402, 1), (0xC404, 1), (0xC406, 1)])

# Test minimal arm
print("\nMinimal arm bits (with full descriptor):")
test_dma("C400+C406 only", FULL_DESC, [(0xC400, 1), (0xC406, 1)])
test_dma("C401+C406 only", FULL_DESC, [(0xC401, 1), (0xC406, 1)])
test_dma("C402+C406 only", FULL_DESC, [(0xC402, 1), (0xC406, 1)])
test_dma("C404+C406 only", FULL_DESC, [(0xC404, 1), (0xC406, 1)])
test_dma("C406 only", FULL_DESC, [(0xC406, 1)])

print("\nDone!")
