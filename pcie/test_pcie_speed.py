#!/usr/bin/env python3
"""Measure PCIe VRAM read/write speed via streaming bulk transfers.

0xF4 control message configures base addr + mode, then bulk streams raw dwords.
"""

import ctypes, struct, time, sys
from tinygrad.runtime.autogen import libusb
from pcie_probe import usb_open, usb_close, setup_bridges, assign_bars, resize_bars, xdata_read

SIZE = 1024 * 1024  # 1MB
GPU_BUS = 4
VRAM_BASE = 0x800000000
EP_OUT = 0x02
EP_IN = 0x81

READ_CHUNK = 31  # dwords per read trigger (31*4=124 bytes, avoids 128-byte ZLP)

def dma_setup(handle, addr, mode, count=0):
  """0xF4: wValue=addr[15:0], wIndex low = mode[1:0] | (count[5:0] << 2), wIndex high = addr[39:32]."""
  wval = addr & 0xFFFF
  widx_l = (mode & 0x03) | ((count & 0x3F) << 2)
  widx = widx_l | (((addr >> 32) & 0xFF) << 8)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF4, wval, widx, None, 0, 5000)
  assert ret >= 0, f"F4 setup failed: {ret}"

def stream_write_packets(handle, dwords):
  """Stream raw dwords as 512-byte bulk OUT packets (128 dwords each). Call dma_setup first."""
  data = struct.pack(f'>{len(dwords)}I', *dwords)
  transferred = ctypes.c_int()
  for off in range(0, len(data), 512):
    chunk = data[off:off+512]
    buf = (ctypes.c_ubyte * len(chunk))(*chunk)
    ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(chunk), ctypes.byref(transferred), 5000)
    assert ret == 0, f"bulk write failed: {ret}"

def stream_read(handle, addr, count):
  """Configure read mode, trigger via bulk OUT, receive READ_CHUNK dwords per bulk IN."""
  dma_setup(handle, addr, 2, READ_CHUNK)
  result = []
  transferred = ctypes.c_int()
  trigger = (ctypes.c_ubyte * 512)()
  nbytes = READ_CHUNK * 4
  while len(result) < count:
    ret = libusb.libusb_bulk_transfer(handle, EP_OUT, trigger, 512, ctypes.byref(transferred), 5000)
    assert ret == 0, f"read trigger failed: {ret}"
    resp = (ctypes.c_ubyte * nbytes)()
    ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), 5000)
    assert ret == 0, f"read data failed: {ret}"
    result.extend(struct.unpack(f'>{transferred.value // 4}I', bytes(resp[:transferred.value])))
  return result

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

  print(f"\nWriting {SIZE // 1024} KB to VRAM...")
  dma_setup(handle, vram_base, 1)
  t0 = time.monotonic()
  stream_write_packets(handle, list(range(total_dwords)))
  t_write = time.monotonic() - t0
  dma_setup(handle, 0, 0)
  print(f"  {t_write:.3f}s ({SIZE / t_write / 1024:.1f} KB/s)")

  print(f"Reading {SIZE // 1024} KB from VRAM...")
  errors = 0
  t0 = time.monotonic()
  vals = stream_read(handle, vram_base, total_dwords)
  t_read = time.monotonic() - t0
  for i, val in enumerate(vals[:total_dwords]):
    if val != i:
      if errors < 10:
        print(f"  MISMATCH at dword {i}: expected 0x{i:08X}, got 0x{val:08X}")
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
