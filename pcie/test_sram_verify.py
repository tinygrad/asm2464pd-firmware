#!/usr/bin/env python3
"""Boot the GPU via tinygrad, fill SRAM via 0xF2 bulk DMA, use GPU SDMA to
copy it into VRAM, then read that VRAM back via 0xF0 PCIe TLP reads and
verify every byte matches.

Usage:
  DEBUG=2 AM_RESET=1 GMMU=0 DEV=AMD AMD_IFACE=USB2 python3 pcie/test_sram_verify.py
"""

import os, sys, struct, ctypes, time
os.environ.setdefault("GMMU", "0")
os.environ.setdefault("DEV", "AMD")
os.environ.setdefault("AMD_IFACE", "USB2")

from tinygrad import Device
from tinygrad.runtime.ops_amd import AMDDevice, AMDCopyQueue
from tinygrad.runtime.support.memory import AddrSpace
from tinygrad.runtime.autogen import libusb

SRAM_EXPECTED = 0x80000  # 512KB, this is verified
SECTOR_SIZE = 512
SLOT_SIZE   = 0x4000   # 16KB per bulk transfer
SRAM_SIZE   = 0x80000  # 512KB total SRAM
NUM_SLOTS   = SRAM_SIZE // SLOT_SIZE
EP_OUT = 0x02

class BulkWriter:
  """Preallocated async bulk OUT writer for maximum throughput."""
  def __init__(self, ctx, handle, num_slots, slot_size):
    self.ctx, self.handle = ctx, handle
    self.num_slots, self.slot_size = num_slots, slot_size
    # Preallocate ctypes buffers and libusb transfers once
    self.bufs = [(ctypes.c_ubyte * slot_size)() for _ in range(num_slots)]
    self.transfers = []
    for i in range(num_slots):
      tr = libusb.libusb_alloc_transfer(0)
      tr.contents.dev_handle = handle
      tr.contents.endpoint = EP_OUT
      tr.contents.type = libusb.LIBUSB_TRANSFER_TYPE_BULK
      tr.contents.length = slot_size
      tr.contents.buffer = self.bufs[i]
      tr.contents.flags = 0
      tr.contents.timeout = 5000
      tr.contents.num_iso_packets = 0
      self.transfers.append(tr)

  def fill(self, slot, data):
    """Copy data into preallocated slot buffer."""
    ctypes.memmove(self.bufs[slot], data, len(data))

  def arm_and_submit(self, total_sectors, start_slot=0):
    """Single F2 arm + submit all transfers, wait for completion."""
    wIndex = (start_slot & 0xFF) | ((self.num_slots & 0xFF) << 8)
    ret = libusb.libusb_control_transfer(self.handle, 0x40, 0xF2, total_sectors, wIndex, None, 0, 1000)
    assert ret >= 0, f"0xF2 batch arm failed: {ret}"
    # Reset status sentinels and submit
    for tr in self.transfers:
      tr.contents.status = 0xFF
      rc = libusb.libusb_submit_transfer(tr)
      assert rc == 0, f"libusb_submit_transfer failed: {rc}"
    # Wait for all completions
    running = self.num_slots
    while running:
      libusb.libusb_handle_events(self.ctx)
      running = 0
      for tr in self.transfers:
        if tr.contents.status == 0xFF: running += 1
        elif tr.contents.status != libusb.LIBUSB_TRANSFER_COMPLETED:
          raise RuntimeError(f"bulk OUT failed: status={tr.contents.status}")

  def __del__(self):
    for tr in self.transfers:
      libusb.libusb_free_transfer(tr)

def make_pattern(slot, size):
  """Deterministic pattern: each byte = (slot * 37 + offset * 7 + 0x42) & 0xFF."""
  return bytes([(slot * 37 + i * 7 + 0x42) & 0xFF for i in range(size)])

def main():
  # 1. Boot the GPU
  print("Booting GPU...")
  dev = Device["AMD"]
  amd: AMDDevice = dev  # type: ignore
  iface = amd.iface
  pci_dev = iface.pci_dev
  usb2 = pci_dev.usb2
  handle = usb2.handle
  mm = iface.dev_impl.mm

  # 2. Fill all SRAM slots via 0xF2 + bulk OUT
  total_bytes = NUM_SLOTS * SLOT_SIZE
  total_sectors = total_bytes // SECTOR_SIZE
  print(f"Filling {NUM_SLOTS} slots x {SLOT_SIZE//1024}KB = {total_bytes//1024}KB via 0xF2 SRAM DMA")

  writer = BulkWriter(usb2.ctx, handle, NUM_SLOTS, SLOT_SIZE)
  for slot in range(NUM_SLOTS):
    writer.fill(slot, make_pattern(slot, SLOT_SIZE))

  t0 = time.monotonic()
  writer.arm_and_submit(total_sectors)
  elapsed = time.monotonic() - t0
  print(f"  Write done (batch): {elapsed:.3f}s ({total_bytes/elapsed/1024:.1f} KB/s)")

  # Spot-check slot 0 via xdata readback
  check = usb2.xdata_read(0xf000, 16)
  expected_first = make_pattern(0, 16)
  assert check == expected_first, f"spot-check failed: got {check.hex()}, expected {expected_first.hex()}"
  print(f"  Spot-check OK (slot 0 first 16B: {check.hex()})")

  # 3. Map each SRAM slot in GPU page tables and SDMA copy to VRAM
  #    Slots are at PCIe 0x200000 + slot * 0x10000, but only SLOT_SIZE bytes each.
  print(f"\nSDMA copying {NUM_SLOTS} slots to VRAM...")
  dst_mapping = mm.valloc(total_bytes, uncached=True)
  dst_va = dst_mapping.va_addr
  dst_paddr = dst_mapping.paddrs[0][0]
  print(f"  VRAM dst VA 0x{dst_va:012X}, paddr 0x{dst_paddr:08X}, size 0x{total_bytes:X}")

  t0 = time.monotonic()
  for slot in range(NUM_SLOTS):
    pcie_addr = 0x200000 + slot * SLOT_SIZE
    src_va = mm.alloc_vaddr(size=SLOT_SIZE)
    mm.map_range(src_va, SLOT_SIZE, [(pcie_addr, SLOT_SIZE)], aspace=AddrSpace.SYS, uncached=True)

    vram_off = slot * SLOT_SIZE
    AMDCopyQueue(amd) \
      .wait(amd.timeline_signal, amd.timeline_value - 1) \
      .copy(dst_va + vram_off, src_va, SLOT_SIZE) \
      .signal(amd.timeline_signal, amd.next_timeline()) \
      .submit(amd)
  amd.timeline_signal.wait(amd.timeline_value - 1)
  elapsed = time.monotonic() - t0
  print(f"  SDMA done: {elapsed:.3f}s ({total_bytes/elapsed/1024:.1f} KB/s)")

  # 4. Read VRAM back via PCIe BAR (0xF0 TLP reads)
  print(f"\nReading {total_bytes//1024}KB back from VRAM via PCIe BAR...")
  t0 = time.monotonic()
  view = pci_dev.map_bar(bar=iface.vram_bar, off=dst_paddr, size=total_bytes)
  got = bytes(view[0:total_bytes])
  elapsed = time.monotonic() - t0
  print(f"  Read done: {elapsed:.3f}s ({total_bytes/elapsed/1024:.1f} KB/s)")

  # 5. Verify every slot
  errors = 0
  for slot in range(NUM_SLOTS):
    expected = make_pattern(slot, SLOT_SIZE)
    off = slot * SLOT_SIZE
    chunk_got = got[off:off + SLOT_SIZE]
    if chunk_got != expected:
      # find first mismatch
      for j in range(SLOT_SIZE):
        if chunk_got[j] != expected[j]:
          errors += 1
          if errors <= 8:
            ctx = min(16, SLOT_SIZE - (j & ~0xF))
            a = j & ~0xF
            print(f"  FAIL slot {slot} @ 0x{j:04X}: got {chunk_got[a:a+ctx].hex()} expected {expected[a:a+ctx].hex()}")
          break
    else:
      if slot % 8 == 0:
        print(f"  slot {slot:2d}: OK")

  if errors:
    print(f"\nFAIL: {errors}/{NUM_SLOTS} slots have mismatches")
    return 1
  else:
    print(f"\nPASS: all {total_bytes} bytes across {NUM_SLOTS} slots verified (SRAM -> SDMA -> VRAM -> 0xF0 readback)")
    return 0

if __name__ == "__main__":
  sys.exit(main())
