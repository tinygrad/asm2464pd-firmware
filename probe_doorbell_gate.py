#!/usr/bin/env python3
"""
Doorbell bit 5 gates certain register writes.
Check: which registers ONLY accept writes when C42A bit 5 is set?
Compare writability with and without doorbell.

Focus on CE10-CE13 (SRAM pointer) which was R/O before.
Maybe it becomes writable with doorbell bit 5 set.
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

# ============================================================
print("="*60)
print("CE10-CE13 writability WITH doorbell bit 5")
print("="*60)

# Without doorbell
for addr in [0xCE10, 0xCE11, 0xCE12, 0xCE13]:
  orig = read8(addr)
  write(addr, 0xAA)
  rb = read8(addr)
  write(addr, orig)
  print(f"  0x{addr:04X} no doorbell: orig=0x{orig:02X} wrote=0xAA read=0x{rb:02X} {'R/W' if rb==0xAA else 'R/O'}")

# WITH doorbell bit 5
write(0xC42A, 0x20)
for addr in [0xCE10, 0xCE11, 0xCE12, 0xCE13]:
  orig = read8(addr)
  write(addr, 0xAA)
  rb = read8(addr)
  write(addr, orig)
  print(f"  0x{addr:04X} doorbell=20: orig=0x{orig:02X} wrote=0xAA read=0x{rb:02X} {'R/W!!' if rb==0xAA else 'R/O'}")
write(0xC42A, 0x00)

# ============================================================
print("\n" + "="*60)
print("Scan C400-C4FF: regs that change writability with doorbell")
print("="*60)

SKIP = {0xC406, 0xC42A}

for addr in range(0xC400, 0xC500):
  if addr in SKIP: continue
  # Without doorbell
  orig = read8(addr)
  write(addr, 0xAA if orig != 0xAA else 0x55)
  rb_no = read8(addr)
  write(addr, orig)

  # With doorbell bit 5
  write(0xC42A, 0x20)
  orig2 = read8(addr)
  write(addr, 0xAA if orig2 != 0xAA else 0x55)
  rb_yes = read8(addr)
  write(addr, orig2)
  write(0xC42A, 0x00)

  tv = 0xAA if orig != 0xAA else 0x55
  rw_no = rb_no == tv
  rw_yes = rb_yes == tv

  if rw_no != rw_yes:
    print(f"  0x{addr:04X}: no_db={'R/W' if rw_no else 'R/O'} db=20={'R/W' if rw_yes else 'R/O'} ** GATED **")

# ============================================================
print("\n" + "="*60)
print("Scan CE00-CEFF: regs that change writability with doorbell")
print("="*60)

SKIP_CE = {0xCE00, 0xCE88}

for addr in range(0xCE00, 0xCF00):
  if addr in SKIP_CE: continue
  try:
    orig = read8(addr)
    tv = 0xAA if orig != 0xAA else 0x55
    write(addr, tv)
    rb_no = read8(addr)
    write(addr, orig)

    write(0xC42A, 0x20)
    orig2 = read8(addr)
    write(addr, tv)
    rb_yes = read8(addr)
    write(addr, orig2)
    write(0xC42A, 0x00)

    rw_no = rb_no == tv
    rw_yes = rb_yes == tv

    if rw_no != rw_yes:
      print(f"  0x{addr:04X}: no_db={'R/W' if rw_no else 'R/O'} db=20={'R/W' if rw_yes else 'R/O'} ** GATED **")
  except:
    break

print("\nDone!")
