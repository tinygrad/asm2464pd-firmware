#!/usr/bin/env python3
"""
Doorbell (C42A) might gate address register writes.
Stock firmware does a doorbell dance: ramp bits up, write config, ramp down.
Test: set C42A bits, then write address regs, then trigger.
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

test_n = [0]

def test(label, doorbell_seq, addr_writes):
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  # Doorbell sequence
  for val in doorbell_seq:
    write(0xC42A, val)

  # Write address regs while doorbell is set
  for addr, val in addr_writes:
    write(addr, val)

  # Now clear doorbell and trigger
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)

  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")

  write(0xC42A, 0x01)

  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:55s}: {locs}{flag}")

# Target: PCI 0x200400 via CE76-CE79
CE76_400 = [(0xCE76, 0x00), (0xCE77, 0x04), (0xCE78, 0x20), (0xCE79, 0x00)]
# Target via C416-C419
C416_400 = [(0xC416, 0x00), (0xC417, 0x04), (0xC418, 0x20), (0xC419, 0x00)]

# ============================================================
print("="*60)
print("Doorbell bit 5 (0x20) = link gate, then write address")
print("="*60)

test("baseline (no doorbell)", [], [])
test("C42A=0x20 then CE76=0x200400", [0x20], CE76_400)
test("C42A=0x20 then C416=0x200400", [0x20], C416_400)
test("C42A=0x20 then C4EE=4", [0x20], [(0xC4EE, 0x04)])

# ============================================================
print("\n" + "="*60)
print("Full doorbell ramp (stock sequence)")
print("="*60)

# Stock ramp: 0x01, 0x03, 0x07, 0x0F, 0x1F
ramp = [0x01, 0x03, 0x07, 0x0F, 0x1F]
test("ramp then CE76=0x200400", ramp, CE76_400)
test("ramp then C416=0x200400", ramp, C416_400)

# ============================================================
print("\n" + "="*60)
print("Every doorbell bit + CE76")
print("="*60)

for bit in range(6):
  val = 1 << bit
  test(f"C42A=0x{val:02X} then CE76=0x200400", [val], CE76_400)

# ============================================================
print("\n" + "="*60)
print("Doorbell + C403 + address")
print("="*60)

test("C42A=0x20 + C403=4 + CE76=0x200400", [0x20], [(0xC403, 0x04)] + CE76_400)
test("C42A=0x1F + C403=4 + CE76=0x200400", [0x1F], [(0xC403, 0x04)] + CE76_400)
test("C42A=0x3F + C403=4 + CE76=0x200400", [0x3F], [(0xC403, 0x04)] + CE76_400)

# ============================================================
print("\n" + "="*60)
print("Doorbell + C416-C419 + various enables")
print("="*60)

for db in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]:
  test(f"C42A=0x{db:02X} + C416=0x200400", [db], C416_400)

# ============================================================
print("\n" + "="*60)
print("Write addr BETWEEN doorbell set and clear")
print("  Doorbell=0x20, write CE76, doorbell=0x00, trigger")
print("="*60)

# Maybe the doorbell gates the latch
for db_val in [0x01, 0x03, 0x07, 0x0F, 0x1F, 0x20, 0x3F]:
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77

  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x200):
    write(0xF000 + off, 0x00)

  # Set doorbell
  write(0xC42A, db_val)
  # Write address
  write(0xCE76, 0x00); write(0xCE77, 0x04); write(0xCE78, 0x20); write(0xCE79, 0x00)
  # Verify it stuck
  ce76 = read8(0xCE76)
  ce77 = read8(0xCE77)
  # Clear doorbell + trigger
  write(0xC42A, 0x00)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)

  landed = []
  for off in range(0, 0x1000, 0x200):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")
  write(0xC42A, 0x01)
  write(0xCE76, 0x00); write(0xCE77, 0x00)

  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  C42A=0x{db_val:02X} CE77 stuck=0x{ce77:02X}: {locs}{flag}")

print("\nDone!")
