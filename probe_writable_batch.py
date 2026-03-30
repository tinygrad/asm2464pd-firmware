#!/usr/bin/env python3
"""
Writability scan in single-register batches.
Each register gets its own connection. If it crashes, we note it and move on.
Usage: python3 probe_writable_batch.py <start_addr> <end_addr>
"""
import sys, subprocess, time
sys.path.insert(0, "/home/geohot/tinygrad")

start = int(sys.argv[1], 16) if len(sys.argv) > 1 else 0xC480
end = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0xC500

SKIP = {
  0xC400, 0xC401, 0xC402, 0xC404, 0xC406,
  0xC42A, 0xC42C, 0xC42D,
  0xCE00, 0xCE88,
}

def reset():
  subprocess.run(["./ftdi_debug.py", "-rn"], capture_output=True, timeout=10)
  time.sleep(2.5)

def test_one(addr):
  """Test one register. Returns (orig, wrote, readback) or None on crash."""
  try:
    from tinygrad.runtime.support.usb import ASM24Controller
    ctrl = ASM24Controller()
    orig = ctrl.read(addr, 1)[0]
    tv = 0xAA if orig != 0xAA else 0x55
    ctrl.write(addr, bytes([tv]), ignore_cache=True)
    rb = ctrl.read(addr, 1)[0]
    ctrl.write(addr, bytes([orig]), ignore_cache=True)
    # health check
    _ = ctrl.read(0xC471, 1)[0]
    return (orig, tv, rb)
  except:
    return None

need_reset = True
for addr in range(start, end):
  if addr in SKIP:
    continue

  if need_reset:
    reset()
    need_reset = False

  result = test_one(addr)
  if result is None:
    print(f"  0x{addr:04X}: CRASHED")
    need_reset = True
  else:
    orig, tv, rb = result
    if rb == tv:
      print(f"  0x{addr:04X}: orig=0x{orig:02X} -> R/W")
    elif rb != orig:
      print(f"  0x{addr:04X}: orig=0x{orig:02X} wrote=0x{tv:02X} read=0x{rb:02X} -> PARTIAL")
    # else R/O, skip
