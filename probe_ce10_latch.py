#!/usr/bin/env python3
"""
CE10 is writable but doesn't affect C400 DMA target.
Maybe CE10 needs to be "latched" into the DMA engine.
Try: write CE10, then various CE triggers, then C400 DMA.
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
marker = 0

def prep():
  global marker
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)

def set_ce10(pci):
  write(0xCE10, (pci >> 24) & 0xFF)
  write(0xCE11, (pci >> 16) & 0xFF)
  write(0xCE12, (pci >> 8) & 0xFF)
  write(0xCE13, pci & 0xFF)

def trigger():
  write(0xC42A, 0x00)
  write(0xC406, 0x01)
  write(0xC42A, 0x01)

def scan():
  landed = []
  for off in range(0, 0x1000, 0x100):
    if read8(0xF000 + off) == marker:
      landed.append(f"F{off:03X}")
  return landed

def report(label):
  landed = scan()
  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:55s}: {locs}{flag}")

# ============================================================
print("="*60)
print("CE10 + CE6E latch (CE6E resets CE10 on stock)")
print("="*60)

# Write CE10, then write CE6E to "commit" it
prep(); set_ce10(0x00200400)
write(0xCE6E, 0x00); write(0xCE6F, 0x01)  # advance pointer
trigger(); report("CE10=400 -> CE6F=1 -> trigger")

prep(); set_ce10(0x00200400)
write(0xCE6E, 0x01); write(0xCE6F, 0x00)
trigger(); report("CE10=400 -> CE6E=1 -> trigger")

prep(); set_ce10(0x00200400)
write(0xCE6E, 0x00); write(0xCE6F, 0x00)  # reset
trigger(); report("CE10=400 -> CE6E=00 -> trigger")

# ============================================================
print("\n" + "="*60)
print("CE10 + CE00 (DMA trigger in CE engine)")
print("="*60)

prep(); set_ce10(0x00200400)
write(0xCE00, 0x03)  # CE DMA trigger
trigger(); report("CE10=400 -> CE00=03 -> C406 trigger")

prep(); set_ce10(0x00200400)
write(0xCE00, 0x01)
trigger(); report("CE10=400 -> CE00=01 -> C406 trigger")

prep(); set_ce10(0x00200400)
write(0xCE00, 0x02)
trigger(); report("CE10=400 -> CE00=02 -> C406 trigger")

# ============================================================
print("\n" + "="*60)
print("CE10 + CE88 handshake trigger")
print("="*60)

prep(); set_ce10(0x00200400)
write(0xCE88, 0x00)  # handshake trigger
trigger(); report("CE10=400 -> CE88=00 -> trigger")

prep(); set_ce10(0x00200400)
write(0xCE88, 0x01)
trigger(); report("CE10=400 -> CE88=01 -> trigger")

# ============================================================
print("\n" + "="*60)
print("Maybe CE10 IS the address but only for CE00 path")
print("  Try CE00=0x03 ONLY (no C400 trigger)")
print("="*60)

prep(); set_ce10(0x00200400)
write(0xCE76, 0x00); write(0xCE77, 0x04); write(0xCE78, 0x20); write(0xCE79, 0x00)
write(0xCE00, 0x03)
for i in range(50):
  if read8(0xCE00) == 0: break
report("CE10=400 + CE76=400 + CE00=03 only")

# ============================================================
print("\n" + "="*60)
print("C400 trigger THEN immediately read CE10")
print("  Does CE10 change after C400 DMA?")
print("="*60)

prep()
set_ce10(0x00200400)
print(f"  CE10 before trigger: {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")
write(0xC42A, 0x00); write(0xC406, 0x01)
print(f"  CE10 after trigger:  {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")
write(0xC42A, 0x01)
print(f"  CE10 after unlock:   {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")
report("CE10 tracking test")

# Same without pre-setting CE10
print("\n  Without pre-setting CE10:")
prep()
print(f"  CE10 before trigger: {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")
write(0xC42A, 0x00); write(0xC406, 0x01)
print(f"  CE10 after trigger:  {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")
write(0xC42A, 0x01)
print(f"  CE10 after unlock:   {read8(0xCE10):02X}{read8(0xCE11):02X}{read8(0xCE12):02X}{read8(0xCE13):02X}")

set_ce10(0x00200000)
print("\nDone!")
