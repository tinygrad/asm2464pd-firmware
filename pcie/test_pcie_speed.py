#!/usr/bin/env python3
"""Measure PCIe VRAM read/write speed via streaming bulk transfers.

0xF0 control message configures base addr + mode, then bulk streams raw dwords.
Firmware loops internally — one large bulk transfer per direction.
"""

import ctypes, struct, time, sys, array
from tinygrad.runtime.autogen import libusb
from pcie_probe import usb_open, usb_close, setup_bridges, assign_bars, resize_bars, xdata_read

SIZE = 1024 * 1024  # 1MB
GPU_BUS = 4
VRAM_BASE = 0x800000000
EP_OUT = 0x02
EP_IN = 0x81

MWR64 = 0x60
MRD64 = 0x20

def dma_setup(handle, addr, mode, ndwords=0):
  """0xF0: wValue = fmt_type|(be<<8), wIndex = mode. DATA_OUT = addr[4 LE] + addr_hi[4 LE] + value[4 LE]."""
  fmt = {0: 0, 1: MWR64, 2: MRD64}[mode]
  wval = fmt | (0x0F << 8)
  widx = mode & 0x03
  payload = struct.pack('<III', addr & 0xFFFFFFFF, addr >> 32, ndwords)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, widx, buf, 12, 5000)
  assert ret >= 0, f"F0 setup failed: {ret}"

def main():
  handle, ctx = usb_open()

  ltssm = xdata_read(handle, 0xB450, 1)[0]
  assert ltssm == 0x78, f"PCIe link not up (LTSSM=0x{ltssm:02X})"

  if "--setup" in sys.argv:
    gpu_bus = setup_bridges(handle, GPU_BUS)
    resize_bars(handle, gpu_bus)
    bars = assign_bars(handle, gpu_bus)
    vram_base = bars[0][0]
    print(f"VRAM BAR0: 0x{vram_base:X}")
  else:
    vram_base = VRAM_BASE

  total_dwords = SIZE // 4
  transferred = ctypes.c_int()

  # Build write data (little-endian dwords)
  data = array.array('I', range(total_dwords)).tobytes()

  print(f"\nWriting {SIZE // 1024} KB to VRAM...")
  dma_setup(handle, vram_base, 1, total_dwords)
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  t0 = time.monotonic()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), 30000)
  t_write = time.monotonic() - t0
  assert ret == 0, f"bulk write failed: {ret} (transferred {transferred.value})"

  print(f"  {t_write:.3f}s ({SIZE / t_write / 1024:.1f} KB/s)")

  print(f"Reading {SIZE // 1024} KB from VRAM...")
  dma_setup(handle, vram_base, 2, total_dwords)
  resp = (ctypes.c_ubyte * SIZE)()
  t0 = time.monotonic()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, SIZE, ctypes.byref(transferred), 30000)
  t_read = time.monotonic() - t0
  assert ret == 0, f"read failed: {ret} (transferred {transferred.value})"
  assert transferred.value == SIZE, f"short read: {transferred.value}/{SIZE}"
  result = list(struct.unpack(f'<{total_dwords}I', bytes(resp)))

  errors = 0
  for i in range(len(result)):
    if result[i] != i:
      if errors < 10:
        print(f"  MISMATCH at dword {i}: expected 0x{i:08X}, got 0x{result[i]:08X}")
      errors += 1
  print(f"  {t_read:.3f}s ({SIZE / t_read / 1024:.1f} KB/s)")

  if errors:
    print(f"\nFAIL: {errors}/{total_dwords} dwords mismatched")
  else:
    print(f"\nPASS: {total_dwords} dwords verified")

  usb_close(handle, ctx)
  return 1 if errors else 0

if __name__ == "__main__":
  sys.exit(main())
