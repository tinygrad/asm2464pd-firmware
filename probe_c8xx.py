#!/usr/bin/env python3
"""
Set stock C8xx values on handmade firmware and test multi-sector DMA.
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

# Stock C8xx values that differ from handmade
STOCK_C8 = {
  0xC800: 0x05, 0xC801: 0x50, 0xC802: 0x04, 0xC805: 0x02,
  0xC807: 0x84, 0xC809: 0x2A, 0xC89D: 0xFF, 0xC8A2: 0x80,
  0xC8A4: 0x04, 0xC8A8: 0x01, 0xC8AA: 0x03, 0xC8AB: 0x01,
  0xC8AC: 0x07, 0xC8B2: 0xBC, 0xC8B3: 0x80, 0xC8B4: 0xFF,
  0xC8B5: 0xFF, 0xC8B6: 0x14,
}

def dma_test(label, pre_writes, sectors=2):
  """Send unique data, set regs, full DMA trigger, check results."""
  data = b''
  for s in range(sectors):
    data += bytes([0x10*s + (s+1)] * 512)
  dev._bulk_out(0x02, data[:1024])  # max 1KB bulk
  
  # Clear scan
  for off in range(0, min(sectors*512, 0x1000), 0x100):
    write(0xF000 + off, 0x00)
  
  # Apply pre-writes
  for addr, val in pre_writes.items():
    write(addr, val)
  
  # Full dma_explore descriptor
  for addr, val in [(0xC420,0),(0xC421,1),(0xC422,2),(0xC423,0),
                    (0xC424,0),(0xC425,0),(0xC426,0),(0xC427,sectors),
                    (0xC428,0x30),(0xC429,0),(0xC42A,0),(0xC42B,0),
                    (0xC42C,0),(0xC42D,0)]:
    write(addr, val)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  
  results = []
  for off in range(0, min(sectors*512, 0x1000), 0x100):
    d = read8(0xF000 + off)
    results.append(f"F{off:03X}={d:02X}")
  
  # Check if >1KB got written
  f4 = read8(0xF400) if sectors > 2 else 0
  flag = ""
  if sectors > 2 and f4 not in (0x00, 0x55): flag = " ** >1KB **"
  
  print(f"  {label:45s}: {' '.join(results)} {flag}")
  write(0xC42A, 0x01)

# ============================================================
print("="*60)
print("Baseline (no C8xx)")
print("="*60)
dma_test("no C8xx, 2 sectors", {}, 2)

# ============================================================
print("\n" + "="*60)
print("All stock C8xx values")
print("="*60)
dma_test("all stock C8xx, 2 sectors", STOCK_C8, 2)
dma_test("all stock C8xx, 4 sectors", STOCK_C8, 4)
dma_test("all stock C8xx, 8 sectors", STOCK_C8, 8)

# ============================================================
print("\n" + "="*60)
print("Individual C8xx groups")
print("="*60)

# C800-C809 group
g1 = {0xC800: 0x05, 0xC801: 0x50, 0xC802: 0x04, 0xC805: 0x02, 0xC807: 0x84, 0xC809: 0x2A}
dma_test("C800-C809 group, 2 sectors", g1, 2)

# C8A0-C8AC group
g2 = {0xC8A2: 0x80, 0xC8A4: 0x04, 0xC8A8: 0x01, 0xC8AA: 0x03, 0xC8AB: 0x01, 0xC8AC: 0x07}
dma_test("C8A0-C8AC group, 2 sectors", g2, 2)

# C8B2-C8B6 group (DMA engine core)
g3 = {0xC8B2: 0xBC, 0xC8B3: 0x80, 0xC8B4: 0xFF, 0xC8B5: 0xFF, 0xC8B6: 0x14}
dma_test("C8B2-C8B6 (DMA engine), 2 sectors", g3, 2)
dma_test("C8B2-C8B6 (DMA engine), 4 sectors", g3, 4)

# Just C89D
dma_test("C89D=0xFF, 2 sectors", {0xC89D: 0xFF}, 2)

# ============================================================
print("\n" + "="*60)
print("Stock C8xx + stock C4xx control regs")
print("="*60)

stock_c4 = {0xC412: 0x03, 0xC413: 0x01, 0xC414: 0x80, 0xC415: 0x01, 0xC471: 0x01, 0xC473: 0x66}
combined = {**STOCK_C8, **stock_c4}
dma_test("C8xx + C4xx stock, 2 sectors", combined, 2)
dma_test("C8xx + C4xx stock, 4 sectors", combined, 4)
dma_test("C8xx + C4xx stock, 8 sectors", combined, 8)

# ============================================================
print("\n" + "="*60)
print("Stock C8xx + C4xx + CE regs")
print("="*60)

stock_ce = {0xCE86: 0x08, 0xCE87: 0x11, 0xCE89: 0x03, 0xCE8A: 0x05, 0xCE8F: 0x01,
            0xCEB0: 0x01, 0xCEB3: 0x01, 0xCEC0: 0xE4, 0xCEC1: 0x40,
            0xCEC2: 0x50, 0xCEC3: 0xCE, 0xCEC4: 0xC0, 0xCECD: 0x08}
all_stock = {**STOCK_C8, **stock_c4, **stock_ce}
dma_test("C8+C4+CE stock, 4 sectors", all_stock, 4)
dma_test("C8+C4+CE stock, 8 sectors", all_stock, 8)

print("\nDone!")
