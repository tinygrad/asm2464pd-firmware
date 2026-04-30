#!/usr/bin/env python3
"""Raw SPI flash poker for the running handmade fw at ADD1:0001.

Writes a firmware image to flash starting at `--addr` (default 0x100,
the mask-ROM bootloader's load offset), preserving the chip's 256-byte
config header at flash 0. Useful when iterating on the handmade fw or
the bootstub without going through the FTDI bootloader strap.

Userfw flashing through the bootstub DFU goes through handmade/flash.py.
The SPI protocol primitives are imported from there."""

import os
import sys
import time
import random

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'handmade'))

from flash import (
  open_device, close_device, bulk_out,
  flash_init, flash_jedec_id, flash_read, flash_block_erase, flash_page_program,
  USERFW_VID, USERFW_PID, BOOTSTUB_VID, BOOTSTUB_PID,
)


def flash_write(h, addr, fw):
  """Erase covering blocks, save+restore config (0x000-0x0FF) if block 0
  is in range, then page-program `fw` at `addr`."""
  total = len(fw)
  config = None
  erase_start = addr & ~0xFFFF
  if erase_start == 0:
    print("  saving config (0x000-0x0FF)...")
    config = flash_read(h, 0, 256)

  erase_end = (addr + total + 0xFFFF) & ~0xFFFF
  for block in range(erase_start, erase_end, 0x10000):
    print(f"  erasing block at 0x{block:05X}...")
    flash_block_erase(h, block)

  if config is not None:
    print("  restoring config...")
    for off in range(0, 256, 128):
      chunk = config[off:off+128]
      if any(b != 0xFF for b in chunk):
        bulk_out(h, chunk)
        flash_page_program(h, off, 128)

  written = 0
  while written < total:
    chunk_size = min(128, total - written)
    chunk = fw[written:written + chunk_size]
    bulk_out(h, chunk + bytes((-len(chunk)) % 4))
    flash_page_program(h, addr + written, chunk_size)
    written += chunk_size
    print(f"\r  writing: {written}/{total} ({100*written//total}%)", end="", flush=True)
  print()


def flash_verify(h, addr, fw):
  total = len(fw)
  errors = 0
  verified = 0
  while verified < total:
    chunk = min(255, total - verified)
    got = flash_read(h, addr + verified, chunk)
    expected = fw[verified:verified + chunk]
    for i in range(min(len(got), len(expected))):
      if got[i] != expected[i]:
        if errors < 10:
          print(f"  mismatch at 0x{addr+verified+i:05X}: got 0x{got[i]:02X}, expected 0x{expected[i]:02X}")
        errors += 1
    verified += chunk
    print(f"\r  verifying: {verified}/{total} ({100*verified//total}%)", end="", flush=True)
  print()
  return errors


def flash_test(h):
  """Write a random pattern to flash 0x10000, read back, verify, then erase."""
  TEST_ADDR, TEST_SIZE = 0x10000, 4096
  print(f"flash test at 0x{TEST_ADDR:05X} ({TEST_SIZE} bytes)")

  rng = random.Random(42)
  pattern = bytes(rng.randint(0, 255) for _ in range(TEST_SIZE))

  print("  erasing...")
  flash_block_erase(h, TEST_ADDR)
  erased = flash_read(h, TEST_ADDR, 16)
  assert all(b == 0xFF for b in erased), f"erase failed: {erased.hex()}"

  print("  writing...")
  for off in range(0, TEST_SIZE, 128):
    chunk = pattern[off:off+128]
    bulk_out(h, chunk)
    flash_page_program(h, TEST_ADDR + off, 128)

  print("  verifying...")
  got = flash_read(h, TEST_ADDR, TEST_SIZE)
  errors = sum(1 for a, b in zip(got, pattern) if a != b)
  flash_block_erase(h, TEST_ADDR)
  print(f"  {'PASS' if errors == 0 else f'FAIL: {errors} mismatches'}")
  return errors


def main():
  vid, pid, h, ctx = open_device([(USERFW_VID, USERFW_PID), (BOOTSTUB_VID, BOOTSTUB_PID)])
  try:
    flash_init(h)

    if '--test' in sys.argv:
      sys.exit(1 if flash_test(h) else 0)

    if '--id' in sys.argv:
      jedec = flash_jedec_id(h)
      print(f"JEDEC ID: {jedec.hex()} (mfr=0x{jedec[0]:02X} type=0x{jedec[1]:02X} cap=0x{jedec[2]:02X})")
      return

    if '--read' in sys.argv:
      idx = sys.argv.index('--read')
      addr = int(sys.argv[idx + 1], 0)
      size = int(sys.argv[idx + 2], 0) if idx + 2 < len(sys.argv) else 256
      data = flash_read(h, addr, size)
      for off in range(0, len(data), 16):
        line = data[off:off+16]
        hexs = " ".join(f"{b:02X}" for b in line)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in line)
        print(f"  {addr + off:05X}: {hexs:<48s} {ascii_}")
      return

    if len(sys.argv) >= 2 and not sys.argv[1].startswith('-'):
      fw_path = sys.argv[1]
      flash_addr = int(sys.argv[sys.argv.index('--addr') + 1], 0) if '--addr' in sys.argv else 0x100
      with open(fw_path, 'rb') as f: fw = f.read()
      print(f"firmware: {fw_path} ({len(fw)} bytes) → flash 0x{flash_addr:05X}")
      t0 = time.monotonic()
      flash_write(h, flash_addr, fw)
      print(f"  write took {time.monotonic() - t0:.2f}s")
      t0 = time.monotonic()
      errors = flash_verify(h, flash_addr, fw)
      if errors: sys.exit(f"FAIL: {errors} mismatches")
      print(f"  verify took {time.monotonic() - t0:.2f}s — PASS")
      if '--no-reset' not in sys.argv:
        import subprocess
        subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), '..', 'ftdi_debug.py'), '-rn'],
                       capture_output=True)
        print("device rebooted with new firmware.")
      return

    print(f"usage: {sys.argv[0]} <firmware.bin> [--addr 0x100]")
    print(f"       {sys.argv[0]} --read <addr> [size]")
    print(f"       {sys.argv[0]} --id")
    print(f"       {sys.argv[0]} --test")
  finally:
    close_device(h, ctx)


if __name__ == '__main__':
  main()
