#!/usr/bin/env python3
"""
SPI Flash programmer via 0xE4/0xE5 XDATA register pokes.

All flash operations done purely from the host by poking registers.
No firmware changes needed.

Register sequences matched from emulator trace (trace/flash).

Key discoveries:
  - CA81 bit 0 must be set for flash controller to work
  - CC33=0x04, C805=0x02 also needed for init
  - Address registers are: C8A1=addr[15:8], C8A2=addr[23:16], C8AB=addr[7:0]
  - DATA_LEN_HI=0x80 (32KB) needed for read DMA to execute
  - Flash read data lands at 0x7000, readable via E4
  - Flash write data comes from 0x7000, loaded via bulk OUT (ARM_OUT)

Usage:
  python3 e4_flash.py firmware.bin               # flash and verify
  python3 e4_flash.py --read 0x0000 256           # read flash
  python3 e4_flash.py --id                        # JEDEC ID
"""
import ctypes, time, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pcie'))
from tinygrad.runtime.autogen import libusb
from pcie_probe import xdata_read, xdata_write

VID, PID = 0xADD1, 0x0001
EP_OUT = 0x02

def flash_init(h):
  """Initialize flash controller — required before any SPI operation."""
  xdata_write(h, 0xCC33, 0x04)
  v = xdata_read(h, 0xCA81, 1)[0]
  xdata_write(h, 0xCA81, v | 0x01)
  xdata_write(h, 0xC805, 0x02)
  xdata_write(h, 0xC8A6, 0x04)  # SPI clock divider

def flash_poll_busy(h):
  for _ in range(100000):
    if not (xdata_read(h, 0xC8A9, 1)[0] & 0x01): return
  raise TimeoutError("Flash CSR busy timeout")

def flash_transaction(h, cmd, addr=0, data_len=0, addr_len=0x07, mode=0x00):
  """Run a flash transaction matching the stock bootloader sequence."""
  xdata_write(h, 0xC8AD, mode)
  xdata_write(h, 0xC8AE, 0x00)
  xdata_write(h, 0xC8AF, 0x00)
  xdata_write(h, 0xC8AA, cmd)
  xdata_write(h, 0xC8AC, addr_len)
  # Address: C8A1=addr[7:0], C8A2=addr[15:8], C8AB=addr[23:16]
  xdata_write(h, 0xC8A1, addr & 0xFF)
  xdata_write(h, 0xC8A2, (addr >> 8) & 0xFF)
  xdata_write(h, 0xC8AB, (addr >> 16) & 0xFF)
  xdata_write(h, 0xC8A3, data_len & 0xFF)
  xdata_write(h, 0xC8A4, (data_len >> 8) & 0xFF)
  xdata_write(h, 0xC8A9, 0x01)
  flash_poll_busy(h)
  xdata_write(h, 0xC8AD, 0x00)
  xdata_write(h, 0xC8AD, 0x00)
  xdata_write(h, 0xC8AD, 0x00)
  xdata_write(h, 0xC8AD, 0x00)

def flash_wren(h):
  """WREN — write enable. Does NOT use flash_transaction to avoid clobbering 0x7000 buffer."""
  xdata_write(h, 0xC8AD, 0x00)
  xdata_write(h, 0xC8AA, 0x06)
  xdata_write(h, 0xC8AC, 0x04)
  xdata_write(h, 0xC8A3, 0x00)
  xdata_write(h, 0xC8A4, 0x00)
  xdata_write(h, 0xC8A9, 0x01)
  flash_poll_busy(h)

def flash_read_status(h):
  flash_transaction(h, cmd=0x05, data_len=0x100, addr_len=0x04, mode=0x00)
  return xdata_read(h, 0x7000, 1)[0]

def flash_wait_wip(h, timeout=10.0):
  t0 = time.monotonic()
  while time.monotonic() - t0 < timeout:
    if not (flash_read_status(h) & 0x01): return
    time.sleep(0.01)
  raise TimeoutError("Flash WIP timeout")

def flash_jedec_id(h):
  flash_transaction(h, cmd=0x9F, data_len=3, addr_len=0x04, mode=0x00)
  return xdata_read(h, 0x7000, 3)

def flash_read(h, addr, size):
  """Read from SPI flash. Uses DATA_LEN=0xFF00 to get up to 255 valid bytes per transaction."""
  result = bytearray()
  offset = 0
  while offset < size:
    chunk = min(255, size - offset)
    flash_transaction(h, cmd=0x03, addr=addr + offset, data_len=0xFF00, addr_len=0x07, mode=0x00)
    result.extend(xdata_read(h, 0x7000, chunk))
    offset += chunk
  return bytes(result)

def flash_block_erase(h, addr):
  """Erase 64KB block."""
  assert (addr & 0xFFFF) == 0
  flash_wren(h)
  flash_transaction(h, cmd=0xD8, addr=addr, data_len=0, addr_len=0x07, mode=0x00)
  flash_wait_wip(h)

def flash_page_program(h, addr, size):
  """Write size bytes from 0x7000 buffer to flash."""
  flash_wren(h)
  flash_transaction(h, cmd=0x02, addr=addr, data_len=size, addr_len=0x07, mode=0x01)
  flash_wait_wip(h)

def bulk_out(h, data):
  """Send data to 0x7000 via bulk OUT."""
  xdata_write(h, 0x9094, 0x10)  # ARM_OUT
  buf = (ctypes.c_ubyte * len(data))(*data)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(h, EP_OUT, buf, len(data), ctypes.byref(transferred), 5000)
  assert ret == 0, f"Bulk OUT failed: {ret}"
  assert transferred.value == len(data), f"Short write: {transferred.value}/{len(data)}"
  time.sleep(0.001)

def flash_write(h, addr, firmware):
  """Write firmware to flash: erase blocks, then write pages via bulk OUT + page program."""
  total = len(firmware)
  # Erase needed 64KB blocks
  erase_end = (addr + total + 0xFFFF) & ~0xFFFF
  for block in range(addr & ~0xFFFF, erase_end, 0x10000):
    print(f"  Erasing block at 0x{block:05X}...")
    flash_block_erase(h, block)

  # Write in 128-byte chunks — 0x7000 buffer only reliably holds 128 bytes
  # (upper half gets overwritten by firmware code/ISR)
  written = 0
  while written < total:
    chunk_size = min(128, total - written)
    chunk = firmware[written:written + chunk_size]
    padded = chunk + bytes((-len(chunk)) % 4)
    bulk_out(h, padded)
    flash_page_program(h, addr + written, chunk_size)
    written += chunk_size
    pct = written * 100 // total
    print(f"\r  Writing: {written}/{total} ({pct}%)", end="", flush=True)
  print()

def flash_verify(h, addr, firmware):
  """Read back flash and compare."""
  total = len(firmware)
  errors = 0
  verified = 0
  while verified < total:
    chunk = min(255, total - verified)
    got = flash_read(h, addr + verified, chunk)
    expected = firmware[verified:verified + chunk]
    for i in range(min(len(got), len(expected))):
      if got[i] != expected[i]:
        if errors < 10:
          print(f"  MISMATCH at 0x{addr + verified + i:05X}: got 0x{got[i]:02X}, expected 0x{expected[i]:02X}")
        errors += 1
    verified += chunk
    pct = verified * 100 // total
    print(f"\r  Verifying: {verified}/{total} ({pct}%)", end="", flush=True)
  print()
  return errors

def main():
  ctx = ctypes.POINTER(libusb.libusb_context)()
  libusb.libusb_init(ctypes.byref(ctx))
  h = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
  assert h, "Device not found"
  libusb.libusb_claim_interface(h, 0)
  flash_init(h)

  if '--id' in sys.argv:
    jedec = flash_jedec_id(h)
    print(f"JEDEC ID: {jedec.hex()} (mfr=0x{jedec[0]:02X} type=0x{jedec[1]:02X} cap=0x{jedec[2]:02X})")
  elif '--read' in sys.argv:
    idx = sys.argv.index('--read')
    addr = int(sys.argv[idx + 1], 0)
    size = int(sys.argv[idx + 2], 0) if idx + 2 < len(sys.argv) else 256
    data = flash_read(h, addr, size)
    for off in range(0, len(data), 16):
      line = data[off:off+16]
      hex_str = ' '.join(f'{b:02X}' for b in line)
      ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in line)
      print(f"  {addr + off:05X}: {hex_str:<48s} {ascii_str}")
  elif len(sys.argv) >= 2 and not sys.argv[1].startswith('-'):
    fw_path = sys.argv[1]
    flash_addr = int(sys.argv[sys.argv.index('--addr') + 1], 0) if '--addr' in sys.argv else 0x0000
    with open(fw_path, 'rb') as f:
      fw = f.read()
    print(f"Firmware: {fw_path} ({len(fw)} bytes)")
    print(f"Flash address: 0x{flash_addr:05X}")
    print(f"\nWriting {len(fw)} bytes to flash at 0x{flash_addr:05X}...")
    t0 = time.monotonic()
    flash_write(h, flash_addr, fw)
    print(f"Write done in {time.monotonic() - t0:.2f}s")
    print(f"\nVerifying...")
    t0 = time.monotonic()
    errors = flash_verify(h, flash_addr, fw)
    if errors == 0:
      print(f"PASS: {len(fw)} bytes verified in {time.monotonic() - t0:.2f}s")
    else:
      print(f"FAIL: {errors} byte mismatches")
      sys.exit(1)
  else:
    print(f"Usage: {sys.argv[0]} <firmware.bin> [--addr 0xNNNN]")
    print(f"       {sys.argv[0]} --read <addr> [size]")
    print(f"       {sys.argv[0]} --id")

  libusb.libusb_release_interface(h, 0)
  libusb.libusb_close(h)
  libusb.libusb_exit(ctx)

if __name__ == '__main__':
  main()
