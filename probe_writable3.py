#!/usr/bin/env python3
"""
Safe one-at-a-time writability scan. Test each register individually
with a health check (read C412) after each to detect crashes early.
Print which register crashed.
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

SKIP = {
  0xC400, 0xC401, 0xC402, 0xC404, 0xC406,  # DMA arm/trigger
  0xC42A, 0xC42C, 0xC42D,  # doorbell/MSC
  0xC47C, 0xC47E,  # crashed
  0xCE00, 0xCE88,  # DMA triggers
  0xCE3D, 0xCE3F,  # corrupted DMA
}

ranges = [
  (0xC480, 0xC500, "C480-C4FF"),
  (0xCE00, 0xCE80, "CE00-CE7F"),
  (0xCE80, 0xCF00, "CE80-CEFF"),
]

for start, end, label in ranges:
  print(f"\n{label}:")
  for addr in range(start, end):
    if addr in SKIP:
      continue
    try:
      orig = r(addr)
      tv = 0xAA if orig != 0xAA else 0x55
      w(addr, tv)
      rb = r(addr)
      w(addr, orig)
      # health check
      _ = r(0xC471)
      if rb == tv:
        print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
      elif rb != orig:
        print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")
    except Exception as e:
      print(f"  0x{addr:04X}: CRASHED ({e})")
      break

# CE10 test
print("\nCE10-CE13:")
for addr in [0xCE10, 0xCE11, 0xCE12, 0xCE13]:
  try:
    orig = r(addr)
    tv = 0xAA if orig != 0xAA else 0x55
    w(addr, tv)
    rb = r(addr)
    w(addr, orig)
    rw = "R/W" if rb == tv else "R/O" if rb == orig else f"PARTIAL(0x{rb:02X})"
    print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> {rw}")
  except Exception as e:
    print(f"  0x{addr:04X}: CRASHED ({e})")

print("\nDone!")
