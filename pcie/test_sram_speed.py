#!/usr/bin/env python3
"""Fill the entire SRAM area via 0xF2 SRAM DMA across all slots.

0xF2 control message: wValue = sector_count, wIndex = slot_select (slot*4).
Each slot is a 0x10000 (64KB) chunk of internal SRAM.
Slot 0 is visible at XDATA 0xF000-0xFFFF (4KB window).

One arm + one 0x10000 bulk OUT fills an entire slot in a single transfer.

Usage:
  python3 pcie/test_sram_speed.py [--slots N] [--verify]
"""

import ctypes, struct, sys, time
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001
SECTOR_SIZE = 512
SLOT_SIZE = 0x4000  # 64KB per slot
SECTORS_PER_SLOT = SLOT_SIZE // SECTOR_SIZE

def ctrl_read(handle, addr, size=1):
  buf = (ctypes.c_ubyte * size)()
  ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
  assert ret >= 0, f"ctrl_read(0x{addr:04X}) failed: {ret}"
  return bytes(buf[:ret])

def sram_dma_arm(handle, sectors, slot):
  """Send 0xF2 control to init SRAM DMA engine and arm for bulk OUT."""
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF2, sectors, slot * 4, None, 0, 1000)
  assert ret >= 0, f"0xF2 arm failed (sectors={sectors}, slot={slot}): {ret}"

def main():
  num_slots = 4
  for arg in sys.argv[1:]:
    if arg.startswith("--slots"):
      num_slots = int(sys.argv[sys.argv.index(arg) + 1])

  dev = USB3(VID, PID, 0x81, 0x83, 0x02, 0x04, use_bot=True)
  handle = dev.handle

  total_bytes = num_slots * SLOT_SIZE
  print(f"Filling {num_slots} slots x {SLOT_SIZE // 1024}KB = {total_bytes // 1024}KB ({SECTORS_PER_SLOT} sectors/slot, 1 bulk OUT per slot)")

  t0 = time.monotonic()
  for slot in range(num_slots):
    print(f"slot {slot}")
    # Build 64KB payload: 4-byte header + pattern
    hdr = struct.pack('<HH', slot, 0)
    data = hdr + bytes([(slot + i) & 0xFF for i in range(SLOT_SIZE - 4)])
    assert len(data)%SECTOR_SIZE == 0
    sram_dma_arm(handle, sectors=len(data)//SECTOR_SIZE, slot=slot)
    dev._bulk_out(0x02, data)

  elapsed = time.monotonic() - t0
  speed = total_bytes / elapsed / 1024
  print(f"Write: {elapsed:.3f}s ({speed:.1f} KB/s)")

  print("\nVerifying slot 0 at 0xF000 (first 512B)...")
  expected = struct.pack('<HH', 0, 0) + bytes([i & 0xFF for i in range(SECTOR_SIZE - 4)])
  errors = 0
  for off in range(0, 512, 64):
    addr = 0xF000 + off
    got = ctrl_read(handle, addr, size=4)
    exp = expected[off:off+4]
    ok = got == exp
    if not ok: errors += 1
    print(f"  {'OK  ' if ok else 'FAIL'} 0x{addr:04X}: got {got.hex()} expected {exp.hex()}")

  if errors:
    print(f"\nFAIL: {errors} verification errors")
  else:
    print(f"\nPASS: slot 0 verified")
  return 1 if errors else 0

if __name__ == "__main__":
  sys.exit(main())
