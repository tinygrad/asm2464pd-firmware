#!/usr/bin/env python3
"""Boot the GPU via tinygrad, fill SRAM via 0xF2 bulk DMA, use GPU SDMA to
copy it into VRAM, then read that VRAM back via 0xF0 PCIe TLP reads and
verify every byte matches.

Usage:
  CUSTOM=1 DEBUG=2 AM_RESET=1 GMMU=0 DEV=AMD AMD_IFACE=USB python3 pcie/test_sram_verify.py
"""

import os, sys, struct, ctypes, time, types
os.environ.setdefault("CUSTOM", "1")
os.environ.setdefault("GMMU", "0")
os.environ.setdefault("DEV", "AMD")
os.environ.setdefault("AMD_IFACE", "USB")

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
EP_IN  = 0x81
CQ_PCIE_ADDR = 0x00822000  # NVMe completion queue region in SRAM

def sram_dma_arm(handle, total_sectors, start_slot=0, num_slots=1, bulk_in=False):
  """Arm SRAM DMA: wValue=sectors(16-bit)|direction, wIndex=start_slot|(num_slots<<8).
  bulk_in=True sets bit 15 of wValue → REG_NVME_CTRL_STATUS=0x02 (BULK IN)."""
  wValue = total_sectors & 0x7FFF
  if bulk_in: wValue |= 0x8000
  wIndex = (start_slot & 0xFF) | ((num_slots & 0xFF) << 8)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF2, wValue, wIndex, None, 0, 1000)
  assert ret >= 0, f"0xF2 arm failed: {ret}"

def bulk_out(handle, buf, size):
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, size, ctypes.byref(transferred), 5000)
  assert ret == 0, f"bulk OUT failed: {ret}"
  assert transferred.value == size, f"short write: {transferred.value}/{size}"

def _bulk_in(handle, buf, size, transferred, timeout=5000):
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, buf, size, ctypes.byref(transferred), timeout)
  assert ret == 0, f"bulk IN failed: {ret}"

def bulk_in(handle, size, timeout=5000):
  buf = (ctypes.c_ubyte * size)()
  transferred = ctypes.c_int()
  _bulk_in(handle, buf, size, transferred, timeout)
  return bytes(buf[:transferred.value])

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
  usb = pci_dev.usb
  handle = usb.usb.handle
  mm = iface.dev_impl.mm

  # 2. Fill all SRAM via 0xF2 + single bulk OUT
  total_bytes = NUM_SLOTS * SLOT_SIZE
  total_sectors = total_bytes // SECTOR_SIZE
  print(f"Filling {total_bytes//1024}KB via 0xF2 SRAM DMA (single bulk transfer)")

  buf = (ctypes.c_ubyte * total_bytes)()
  for slot in range(NUM_SLOTS):
    ctypes.memmove(ctypes.addressof(buf) + slot * SLOT_SIZE, make_pattern(slot, SLOT_SIZE), SLOT_SIZE)

  t0 = time.monotonic()
  sram_dma_arm(handle, total_sectors, start_slot=0, num_slots=NUM_SLOTS)
  bulk_out(handle, buf, total_bytes)
  elapsed = time.monotonic() - t0
  print(f"  Write done (batch): {elapsed:.3f}s ({total_bytes/elapsed/1024:.1f} KB/s)")

  # Spot-check slot 0 via xdata readback
  check = usb.read(0xf000, 16)
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
    src_buf, dst_buf = types.SimpleNamespace(va_addr=src_va), types.SimpleNamespace(va_addr=dst_va + vram_off)
    AMDCopyQueue(amd) \
      .wait(amd.timeline_signal, amd.timeline_value - 1) \
      .copy(dst_buf, src_buf, SLOT_SIZE) \
      .signal(amd.timeline_signal, amd.next_timeline()) \
      .submit(amd)
  amd.timeline_signal.wait(amd.timeline_value - 1)
  elapsed = time.monotonic() - t0
  print(f"  SDMA done: {elapsed:.3f}s ({total_bytes/elapsed/1024:.1f} KB/s)")

  # 4. Read VRAM back via 0xF0 streaming PCIe read (mode 2) + single bulk IN
  vram_pcie_addr = pci_dev.bar_info(iface.vram_bar)[0] + dst_paddr
  print(f"\nReading {total_bytes//1024}KB back from VRAM via 0xF0 mode 2 (addr 0x{vram_pcie_addr:X})...")
  ndwords = total_bytes // 4
  MRD64 = 0x20
  payload = struct.pack('<III', vram_pcie_addr & 0xFFFFFFFF, vram_pcie_addr >> 32, ndwords)
  f0_buf = (ctypes.c_ubyte * 12)(*payload)
  wval = MRD64 | (0x0F << 8)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 2, f0_buf, 12, 5000)
  assert ret == 12, f"F0 setup failed: {ret}"
  read_buf = (ctypes.c_ubyte * total_bytes)()
  transferred = ctypes.c_int()
  t0 = time.monotonic()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, read_buf, total_bytes, ctypes.byref(transferred), 30000)
  elapsed = time.monotonic() - t0
  assert ret == 0, f"bulk IN failed: {ret}"
  assert transferred.value == total_bytes, f"short read: {transferred.value}/{total_bytes}"
  got = bytes(read_buf)
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

  # =========================================================================
  # 6. DMA OUT test: VRAM → SRAM via GPU SDMA, then SRAM → host via BULK IN
  # =========================================================================
  print(f"\n{'='*60}")
  print("DMA OUT test: VRAM -> SRAM -> BULK IN")
  print(f"{'='*60}")

  # Use all 32 slots (512KB) for the DMA OUT test
  dma_out_slots = NUM_SLOTS
  dma_out_bytes = dma_out_slots * SLOT_SIZE
  dma_out_sectors = dma_out_bytes // SECTOR_SIZE

  # Fill VRAM with a known pattern for the DMA OUT test
  dma_out_pattern = bytes([(i * 13 + 0xAB) & 0xFF for i in range(dma_out_bytes)])
  dma_out_vram = mm.valloc(dma_out_bytes, uncached=True)
  dma_out_va = dma_out_vram.va_addr
  dma_out_paddr = dma_out_vram.paddrs[0][0]
  print(f"  VRAM src VA 0x{dma_out_va:012X}, paddr 0x{dma_out_paddr:08X}")

  # Write pattern into VRAM via 0xF0 streaming PCIe write (mode 1) + bulk OUT
  dma_out_pcie_addr = pci_dev.bar_info(iface.vram_bar)[0] + dma_out_paddr
  MWR64 = 0x60
  ndwords_out = dma_out_bytes // 4
  payload_out = struct.pack('<III', dma_out_pcie_addr & 0xFFFFFFFF, dma_out_pcie_addr >> 32, ndwords_out)
  f0_buf_out = (ctypes.c_ubyte * 12)(*payload_out)
  wval_out = MWR64 | (0x0F << 8)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval_out, 1, f0_buf_out, 12, 5000)
  assert ret == 12, f"F0 write setup failed: {ret}"
  out_buf = (ctypes.c_ubyte * dma_out_bytes)(*dma_out_pattern)
  out_transferred = ctypes.c_int()
  t0 = time.monotonic()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, out_buf, dma_out_bytes, ctypes.byref(out_transferred), 30000)
  elapsed = time.monotonic() - t0
  assert ret == 0, f"bulk OUT failed: {ret}"
  assert out_transferred.value == dma_out_bytes, f"short write: {out_transferred.value}/{dma_out_bytes}"
  print(f"  VRAM write done: {dma_out_bytes} bytes in {elapsed:.3f}s ({dma_out_bytes/elapsed/1024:.1f} KB/s)")

  # SDMA copy: VRAM → SRAM at PCIe 0x200000
  sram_pcie_addr = 0x200000
  sram_va = mm.alloc_vaddr(size=dma_out_bytes)
  mm.map_range(sram_va, dma_out_bytes, [(sram_pcie_addr, dma_out_bytes)], aspace=AddrSpace.SYS, uncached=True)

  print(f"  SDMA copying {dma_out_bytes//1024}KB from VRAM to SRAM @ PCIe 0x{sram_pcie_addr:X}...")
  src_ns = types.SimpleNamespace(va_addr=dma_out_va)
  dst_ns = types.SimpleNamespace(va_addr=sram_va)
  AMDCopyQueue(amd) \
    .wait(amd.timeline_signal, amd.timeline_value - 1) \
    .copy(dst_ns, src_ns, dma_out_bytes) \
    .signal(amd.timeline_signal, amd.next_timeline()) \
    .submit(amd)
  amd.timeline_signal.wait(amd.timeline_value - 1)
  print(f"  SDMA copy to SRAM done")

  # Spot-check SRAM via xdata readback (0xF000 window)
  check = usb.read(0xf000, 16)
  expected_first = dma_out_pattern[:16]
  print(f"  SRAM spot-check: got {check.hex()}, expected {expected_first.hex()}")
  assert check == expected_first, "SRAM spot-check failed after SDMA copy"

  # Build a proper 16-byte NVMe Completion Queue Entry (CQE):
  #   Bytes 0-3:   Command Specific Result (DW0)
  #   Bytes 4-7:   Reserved (DW1)
  #   Bytes 8-9:   SQ Head Pointer
  #   Bytes 10-11: SQ Identifier
  #   Bytes 12-13: Command Identifier
  #   Bytes 14-15: Status Field — bit 0 of byte 14 = Phase Tag
  cqe = bytearray(16)
  cqe[14] = 0x01   # Phase bit set (bit 0 of status field)

  # Prepare CQE in VRAM
  cq_vram = mm.valloc(0x1000, uncached=True)
  cq_va = cq_vram.va_addr
  cq_paddr = cq_vram.paddrs[0][0]
  cq_view = pci_dev.map_bar(bar=iface.vram_bar, off=cq_paddr, size=0x1000)
  cq_view[0:16] = cqe

  # Map the CQ PCIe address into GPU VA
  cq_dst_va = mm.alloc_vaddr(size=0x1000)
  mm.map_range(cq_dst_va, 0x1000, [(CQ_PCIE_ADDR, 0x1000)], aspace=AddrSpace.SYS, uncached=True)

  # Read 0xB800 before to see initial state
  cq_before = usb.read(0xB800, 16)
  print(f"  CQ @ 0xB800 before: {cq_before.hex()}")

  # Arm 0xF2 with BULK IN flag as close as possible to the bulk transfer
  print(f"  Arming 0xF2 for BULK IN: {dma_out_sectors} sectors, slot 0, {dma_out_slots} slots")
  sram_dma_arm(handle, dma_out_sectors, start_slot=0, num_slots=dma_out_slots, bulk_in=True)

  # Now SDMA the CQE to PCIe 0x00822000 — this unlocks the BULK IN
  print(f"  SDMA writing CQE to PCIe 0x{CQ_PCIE_ADDR:08X}...")
  cq_src_ns = types.SimpleNamespace(va_addr=cq_va)
  cq_dst_ns = types.SimpleNamespace(va_addr=cq_dst_va)
  AMDCopyQueue(amd) \
    .wait(amd.timeline_signal, amd.timeline_value - 1) \
    .copy(cq_dst_ns, cq_src_ns, 0x1000) \
    .signal(amd.timeline_signal, amd.next_timeline()) \
    .submit(amd)
  amd.timeline_signal.wait(amd.timeline_value - 1)
  print(f"  CQE sent")

  # Read 0xB800 after to verify CQE arrived
  cq_after = usb.read(0xB800, 16)
  print(f"  CQ @ 0xB800 after:  {cq_after.hex()}")

  # Perform BULK IN transfer — the CQE should have unlocked it
  # Pre-allocate buffer and transferred outside the timed section
  in_buf = (ctypes.c_ubyte * dma_out_bytes)()
  in_transferred = ctypes.c_int()
  print(f"  BULK IN: reading {dma_out_bytes} bytes from EP 0x{EP_IN:02X}...")
  t0 = time.monotonic()
  _bulk_in(handle, in_buf, dma_out_bytes, in_transferred, timeout=10000)
  elapsed = time.monotonic() - t0
  print(f"  BULK IN done: {in_transferred.value} bytes in {elapsed:.3f}s ({in_transferred.value/elapsed/1024:.1f} KB/s)")

  # Verify the data matches
  got_dma_out = bytes(in_buf[:in_transferred.value])
  dma_out_errors = 0
  for j in range(min(len(got_dma_out), dma_out_bytes)):
    if got_dma_out[j] != dma_out_pattern[j]:
      dma_out_errors += 1
      if dma_out_errors <= 8:
        ctx_start = j & ~0xF
        ctx_end = min(ctx_start + 16, dma_out_bytes)
        print(f"  FAIL @ 0x{j:04X}: got {got_dma_out[ctx_start:ctx_end].hex()} expected {dma_out_pattern[ctx_start:ctx_end].hex()}")
  if len(got_dma_out) != dma_out_bytes:
    print(f"  FAIL: got {len(got_dma_out)} bytes, expected {dma_out_bytes}")
    dma_out_errors += 1

  if dma_out_errors:
    print(f"\nDMA OUT FAIL: {dma_out_errors} errors")
    return 1
  else:
    print(f"\nDMA OUT PASS: {dma_out_bytes} bytes verified (VRAM -> SDMA -> SRAM -> BULK IN)")
    return 0

if __name__ == "__main__":
  sys.exit(main())
