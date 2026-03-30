#!/usr/bin/env python3
"""
CE10 is writable. C403 controls DMA length.
Maybe C403 also enables CE10 as the target address?
Or maybe we need to write CE10 at a specific point in the sequence.
Try every ordering.
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

def scan():
  landed = []
  for off in range(0, 0x1000, 0x100):
    d = read8(0xF000 + off)
    if d == marker:
      landed.append(f"F{off:03X}")
  return landed

def set_ce10(pci):
  write(0xCE10, (pci >> 24) & 0xFF)
  write(0xCE11, (pci >> 16) & 0xFF)
  write(0xCE12, (pci >> 8) & 0xFF)
  write(0xCE13, pci & 0xFF)

def prep():
  global marker
  test_n[0] += 1
  marker = (test_n[0] * 7 + 0x31) & 0xFF
  if marker in (0, 0x55): marker = 0x77
  dev._bulk_out(0x02, bytes([marker]*32))
  for off in range(0, 0x1000, 0x100):
    write(0xF000 + off, 0x00)

def report(label, landed):
  moved = landed and landed[0] != 'F000'
  locs = ', '.join(landed) if landed else 'nowhere'
  flag = " ** MOVED **" if moved else ""
  print(f"  {label:55s}: {locs}{flag}")

marker = 0

# ============================================================
print("="*60)
print("Order 1: CE10 before trigger (what we tried before)")
print("="*60)
prep()
set_ce10(0x00200400)
write(0xC403, 0x04)
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
report("CE10 -> C403 -> trigger", scan())

# ============================================================
print("\nOrder 2: C403 before CE10")
prep()
write(0xC403, 0x04)
set_ce10(0x00200400)
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
report("C403 -> CE10 -> trigger", scan())

# ============================================================
print("\nOrder 3: CE10 between doorbell and trigger")
prep()
write(0xC403, 0x04)
write(0xC42A, 0x00)
set_ce10(0x00200400)
write(0xC406, 0x01)
write(0xC42A, 0x01)
report("C403 -> doorbell -> CE10 -> trigger", scan())

# ============================================================
print("\nOrder 4: CE10 after trigger, before unlock")
prep()
write(0xC403, 0x04)
write(0xC42A, 0x00); write(0xC406, 0x01)
set_ce10(0x00200400)
write(0xC42A, 0x01)
report("trigger -> CE10 -> unlock", scan())

# ============================================================
print("\nOrder 5: Two triggers — first to F000, set CE10, second trigger")
prep()
write(0xC403, 0x04)
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
# Now set CE10 and trigger again
set_ce10(0x00200400)
dev._bulk_out(0x02, bytes([marker]*32))
write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
report("trigger1 -> CE10 -> trigger2", scan())

# ============================================================  
print("\nOrder 6: Full descriptor + CE10")
prep()
write(0xC420, 0x00); write(0xC421, 0x01); write(0xC422, 0x02)
write(0xC423, 0x00); write(0xC424, 0x00); write(0xC425, 0x00)
write(0xC426, 0x00); write(0xC427, 0x01); write(0xC428, 0x30)
write(0xC429, 0x00); write(0xC42B, 0x00)
write(0xC42C, 0x00); write(0xC42D, 0x00)
set_ce10(0x00200400)
write(0xC403, 0x04)
write(0xC42A, 0x00)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)
write(0xC42A, 0x01)
report("full desc + CE10=0x200400 + C403=4", scan())

# ============================================================
print("\n" + "="*60)
print("Try C405 as address enable WITH CE10 set")
print("="*60)

for c405 in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20]:
  prep()
  set_ce10(0x00200400)
  write(0xC403, 0x04)
  write(0xC405, c405)
  write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
  write(0xC405, 0x00)
  report(f"CE10=0x200400 + C403=4 + C405=0x{c405:02X}", scan())

# ============================================================
print("\n" + "="*60)
print("Try CE10 with DIFFERENT C403 values")
print("="*60)

for c403 in [0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x20]:
  prep()
  set_ce10(0x00200400)
  write(0xC403, c403)
  write(0xC42A, 0x00); write(0xC406, 0x01); write(0xC42A, 0x01)
  report(f"CE10=0x200400 + C403=0x{c403:02X}", scan())

write(0xC403, 0x00)
set_ce10(0x00200000)

print("\nDone!")
