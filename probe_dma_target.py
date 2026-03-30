#!/usr/bin/env python3
"""
Make C400 DMA land at 0xF100 (PCI 0x200100) instead of 0xF000 (PCI 0x200000).

Candidates for controlling target address:
  - C420-C426 descriptor bytes (xfer count, LBA, count fields)
  - CE76-CE79 (SCSI buf addr, LE PCI address, R/W)
  - CE10-CE13 (SRAM write pointer, R/O — but maybe C400 path uses different regs)
  - Some C4xx register we haven't tried

Test each one.
"""
import sys, ctypes, random
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

def do_dma(pat=None):
  """Bulk out + DMA trigger. Returns pattern sent."""
  if pat is None:
    pat = bytes(random.getrandbits(8) for _ in range(32))
  dev._bulk_out(0x02, pat)
  dma_fire()
  dma_unlock()
  return pat

def check(addr, expected_4):
  """Check if addr matches first 4 bytes of expected."""
  actual = read_range(addr, 4)
  return actual == expected_4

# Baseline: confirm DMA goes to F000
print("="*60)
print("Baseline: DMA to F000")
print("="*60)
pat = do_dma()
print(f"  F000: {read_range(0xF000, 4).hex()} expected: {pat[:4].hex()} match: {check(0xF000, pat[:4])}")
print(f"  F100: {read_range(0xF100, 4).hex()}")

# ============================================================
print("\n" + "="*60)
print("TEST 1: C420-C426 descriptor bytes")
print("  C420=xfer_hi C421=xfer_lo C422=LBA_low C423-C426")
print("="*60)

# Try setting C422 (LBA_LOW) to shift the address
# C422 is currently 0x02. What if we set it to something else?
for c422_val in [0x00, 0x01, 0x03, 0x04, 0x10]:
  write(0xC422, c422_val)
  pat = do_dma()
  f000 = check(0xF000, pat[:4])
  f100 = check(0xF100, pat[:4])
  f200 = check(0xF200, pat[:4])
  print(f"  C422=0x{c422_val:02X}: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'} F200={'Y' if f200 else 'N'}")

# Reset C422
write(0xC422, 0x02)

# Try C420/C421 (xfer count)
print("\n  Varying C420 (xfer_hi):")
for val in [0x00, 0x01, 0x02, 0x04]:
  write(0xC420, val)
  pat = do_dma()
  f000 = check(0xF000, pat[:4])
  f100 = check(0xF100, pat[:4])
  print(f"  C420=0x{val:02X}: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")
write(0xC420, 0x00)

print("\n  Varying C421 (xfer_lo):")
for val in [0x00, 0x01, 0x02, 0x04]:
  write(0xC421, val)
  pat = do_dma()
  f000 = check(0xF000, pat[:4])
  f100 = check(0xF100, pat[:4])
  print(f"  C421=0x{val:02X}: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")
write(0xC421, 0x01)

# ============================================================
print("\n" + "="*60)
print("TEST 2: CE76-CE79 (SCSI buf addr)")
print("="*60)

# Set CE76-79 to PCI 0x00200100 = F100 in XDATA
# LE: CE76=0x00, CE77=0x01, CE78=0x20, CE79=0x00
write(0xCE76, 0x00)
write(0xCE77, 0x01)
write(0xCE78, 0x20)
write(0xCE79, 0x00)
print(f"  CE76-79 set to PCI 0x00200100")

pat = do_dma()
f000 = check(0xF000, pat[:4])
f100 = check(0xF100, pat[:4])
print(f"  F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")

# Reset
write(0xCE76, 0x00); write(0xCE77, 0x00); write(0xCE78, 0x20); write(0xCE79, 0x00)

# ============================================================
print("\n" + "="*60)
print("TEST 3: C423-C426 as address bytes")
print("  Maybe C423:C424:C425:C426 encode PCI target addr?")
print("="*60)

# Try C423=0x01 (maybe byte offset 0x100?)
for reg, val, label in [
  (0xC423, 0x01, "C423=0x01"),
  (0xC424, 0x01, "C424=0x01"),
  (0xC425, 0x01, "C425=0x01"),
  (0xC426, 0x01, "C426=0x01"),
]:
  write(reg, val)
  pat = do_dma()
  results = []
  for tgt in [0xF000, 0xF100, 0xF200, 0xF400, 0xF800]:
    if check(tgt, pat[:4]):
      results.append(f"0x{tgt:04X}")
  write(reg, 0x00)  # restore
  print(f"  {label}: landed at {', '.join(results) if results else 'NOWHERE'}")

# ============================================================
print("\n" + "="*60)
print("TEST 4: C428 queue config bits")
print("="*60)

# C428 default is 0x30. Writable mask is 0x28.
for val in [0x00, 0x08, 0x20, 0x28, 0x30, 0x38]:
  write(0xC428, val)
  pat = do_dma()
  f000 = check(0xF000, pat[:4])
  f100 = check(0xF100, pat[:4])
  print(f"  C428=0x{val:02X}: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")
write(0xC428, 0x30)

# ============================================================
print("\n" + "="*60)
print("TEST 5: C4ED-C4EF (NVMe DMA ctrl/addr)")
print("="*60)

print(f"  C4ED=0x{read8(0xC4ED):02X} C4EE=0x{read8(0xC4EE):02X} C4EF=0x{read8(0xC4EF):02X}")

# C4EE/C4EF are documented as DMA address low/high
# Try setting them to 0x0100 (offset 0x100 from SRAM base?)
write(0xC4EE, 0x00)  # low
write(0xC4EF, 0x01)  # high = 0x0100?
pat = do_dma()
f000 = check(0xF000, pat[:4])
f100 = check(0xF100, pat[:4])
print(f"  C4EE=0x00 C4EF=0x01: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")

write(0xC4EE, 0x01)
write(0xC4EF, 0x00)
pat = do_dma()
f000 = check(0xF000, pat[:4])
f100 = check(0xF100, pat[:4])
print(f"  C4EE=0x01 C4EF=0x00: F000={'Y' if f000 else 'N'} F100={'Y' if f100 else 'N'}")

# ============================================================
print("\n" + "="*60)
print("TEST 6: Wide scan of F000-FFFF after each tweak")
print("  Set C422 to various values, scan SRAM for data")
print("="*60)

for c422 in [0x00, 0x01, 0x02, 0x03, 0x04, 0x06, 0x08, 0x10, 0x20]:
  write(0xC422, c422)
  pat = do_dma()
  # Scan every 256 bytes
  found = []
  for off in range(0, 0x1000, 0x100):
    if check(0xF000 + off, pat[:4]):
      found.append(f"0x{0xF000+off:04X}")
  print(f"  C422=0x{c422:02X}: {', '.join(found) if found else 'not found in F000-FFFF'}")

write(0xC422, 0x02)

print("\nDone!")
