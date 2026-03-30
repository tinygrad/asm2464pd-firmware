#!/usr/bin/env python3
"""
Try CE00 DMA engine directly. Ignore all docs saying it needs MSC.
Put data at 0x7000 via bulk out, set CE76-79 target, trigger CE00=0x03.
Try every possible setup.
"""
import sys, ctypes, time
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

test_n = [0]

def test(label, pre_writes, target_pci=0x00200400):
  test_n[0] += 1
  marker = (test_n[0] * 13 + 0x41) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  # Put data at 0x7000 via bulk out
  dev._bulk_out(0x02, bytes([marker]*512))

  # Clear target areas
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  # Set CE76-79 target address (LE)
  write(0xCE76, (target_pci >> 0) & 0xFF)
  write(0xCE77, (target_pci >> 8) & 0xFF)
  write(0xCE78, (target_pci >> 16) & 0xFF)
  write(0xCE79, (target_pci >> 24) & 0xFF)

  # Apply pre-writes
  for addr, val in pre_writes:
    write(addr, val)

  # Trigger CE00
  write(0xCE00, 0x03)

  # Poll CE00 for completion (max 50 reads)
  for i in range(50):
    v = read8(0xCE00)
    if v == 0x00:
      break

  # Scan
  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")

  # Clean up CE76
  write(0xCE76, 0x00); write(0xCE77, 0x00); write(0xCE78, 0x20); write(0xCE79, 0x00)
  for addr, val in pre_writes:
    write(addr, 0x00)

  locs = ', '.join(landed) if landed else 'nowhere'
  moved = landed and landed[0] != 'F000'
  at_target = f"F{(target_pci - 0x200000):03X}" in landed if landed else False
  flag = " ** TARGET HIT **" if at_target else (" ** MOVED **" if moved else "")
  print(f"  {label:50s}: {locs}{flag}")
  return at_target

# ============================================================
print("="*60)
print("Raw CE00=0x03 trigger with CE76=0x200400")
print("="*60)

test("bare CE00 trigger", [])

# Set CE01 (DMA param)
test("CE01=0x01", [(0xCE01, 0x01)])
test("CE01=0x03", [(0xCE01, 0x03)])
test("CE01=0xC0", [(0xCE01, 0xC0)])

# CE55 (transfer count)
test("CE55=0x01", [(0xCE55, 0x01)])
test("CE55=0x02", [(0xCE55, 0x02)])

# CE83 (flow control)
test("CE83=0xF1", [(0xCE83, 0xF1)])
test("CE83=0x01", [(0xCE83, 0x01)])

# CE88/CE89 (handshake)
test("CE88=0x00 CE89=0x01", [(0xCE88, 0x00), (0xCE89, 0x01)])
test("CE88=0x00 CE89=0x03", [(0xCE88, 0x00), (0xCE89, 0x03)])
test("CE88=0x00 CE89=0x07", [(0xCE88, 0x00), (0xCE89, 0x07)])

# CE8A (sector ctrl)
test("CE8A=0x01", [(0xCE8A, 0x01)])
test("CE8A=0x05", [(0xCE8A, 0x05)])

# CECD (sector count on stock)
test("CECD=0x01", [(0xCECD, 0x01)])

# ============================================================
print("\n" + "="*60)
print("CE00 with various CE00 values (not just 0x03)")
print("="*60)

for ce00_val in [0x01, 0x02, 0x03, 0x04, 0x07, 0x0F, 0xFF]:
  test_n[0] += 1
  marker = (test_n[0] * 13 + 0x41) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  dev._bulk_out(0x02, bytes([marker]*512))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)
  write(0xCE76, 0x00); write(0xCE77, 0x04); write(0xCE78, 0x20); write(0xCE79, 0x00)
  write(0xCE00, ce00_val)
  for i in range(50):
    if read8(0xCE00) == 0: break
  landed = []
  for off in range(0, 0x1000, 0x200):
    if read8(0xF000 + off) == marker: landed.append(f"F{off:03X}")
  locs = ', '.join(landed) if landed else 'nowhere'
  print(f"  CE00=0x{ce00_val:02X}: {locs}")

# ============================================================
print("\n" + "="*60)
print("CE00 after C400 DMA (prime the buffer)")
print("="*60)

# First do a C400 DMA to get data at F000
dev._bulk_out(0x02, bytes([0xDD]*32))
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
print(f"  F000 after C400: {read8(0xF000):02X}")

# Now try CE00 to copy F000 data to F400
write(0xCE76, 0x00); write(0xCE77, 0x04); write(0xCE78, 0x20); write(0xCE79, 0x00)
write(0xF400, 0x00)
write(0xCE00, 0x03)
for i in range(50):
  if read8(0xCE00) == 0: break
print(f"  F400 after CE00: {read8(0xF400):02X} (DD=copied from F000)")

# ============================================================
print("\n" + "="*60)
print("Full stock CE setup + CE00 trigger")
print("="*60)

stock_ce = [
  (0xCE01, 0x00), (0xCE83, 0xF1),
  (0xCE86, 0x08), (0xCE87, 0x11), (0xCE89, 0x03),
  (0xCE8A, 0x05), (0xCE8F, 0x01),
  (0xCEB0, 0x01), (0xCEB3, 0x01),
  (0xCEC0, 0xE4), (0xCEC1, 0x01), (0xCEC2, 0x50), (0xCEC3, 0xCE), (0xCEC4, 0xC4),
]
test("full stock CE setup", stock_ce)
test("stock CE + CE55=1", stock_ce + [(0xCE55, 0x01)])
test("stock CE + CE70=1 CE71=2 CE73=8", stock_ce + [(0xCE70, 0x01), (0xCE71, 0x02), (0xCE73, 0x08)])

# Target F000 instead of F400
test("stock CE, target F000", stock_ce, target_pci=0x00200000)

print("\nDone!")
