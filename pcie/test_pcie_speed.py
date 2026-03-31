#!/usr/bin/env python3
"""Measure PCIe VRAM read/write speed via streaming bulk transfers.

0xF4 control message configures base addr + mode, then bulk streams raw dwords.
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
READ_CHUNK = 16  # 16 dwords = 64 bytes = 1 full bulk packet (no short packet termination)

def dma_setup(handle, addr, mode, count=0):
  """0xF4: wValue low = mode[1:0] | (count[5:0] << 2). DATA_OUT = addr[8 bytes LE]."""
  wval = (mode & 0x03) | ((count & 0x3F) << 2)
  payload = struct.pack('<II', addr & 0xFFFFFFFF, addr >> 32)
  buf = (ctypes.c_ubyte * 8)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF4, wval, 0, buf, 8, 5000)
  assert ret >= 0, f"F4 setup failed: {ret}"

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

  # Build write data (big-endian dwords)
  arr = array.array('I', range(total_dwords))
  if sys.byteorder == 'little':
    arr.byteswap()
  data = arr.tobytes()

  print(f"\nWriting {SIZE // 1024} KB to VRAM...")
  dma_setup(handle, vram_base, 1)
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  t0 = time.monotonic()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), 30000)
  t_write = time.monotonic() - t0
  assert ret == 0, f"bulk write failed: {ret} (transferred {transferred.value})"
  dma_setup(handle, 0, 0)
  print(f"  {t_write:.3f}s ({SIZE / t_write / 1024:.1f} KB/s)")

  print(f"Reading {SIZE // 1024} KB from VRAM...")
  nbytes = READ_CHUNK * 4
  dma_setup(handle, vram_base, 2, READ_CHUNK)
  resp = (ctypes.c_ubyte * nbytes)()
  errors = 0
  result = []
  t0 = time.monotonic()
  while len(result) < total_dwords:
    ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), 5000)
    assert ret == 0, f"read failed: {ret}"
    result.extend(struct.unpack(f'>{transferred.value // 4}I', bytes(resp[:transferred.value])))
  t_read = time.monotonic() - t0
  dma_setup(handle, 0, 0)
  for i in range(total_dwords):
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
