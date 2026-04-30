#!/usr/bin/env python3
"""Host-side flasher for the ASM2464PD bootstub + userfw.

Two device modes:
  - Bootstub DFU (ADD1:B007): vendor-class with one bulk OUT EP. We drive
    the SPI flash controller via 0xE4/0xE5 (XDATA r/w) + bulk OUT to fill
    the SPI page buffer at 0x7000.
  - Userfw (ADD1:0001) running our handmade userfw skeleton: sending the
    0xEC vendor command writes the DFU cookie and triggers CC31; XDATA
    survives the reset so the bootstub stays in DFU mode on next boot.

Importable as a library (scripts/e4_flash.py reuses these). When run as
a script, exposes `flash`, `read`, `enter-dfu`, `reboot`, and `id`.
"""

import argparse
import ctypes
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pcie'))

from tinygrad.runtime.autogen import libusb
from pcie_probe import xdata_read, xdata_write

USERFW_VID, USERFW_PID = 0xADD1, 0x0001
BOOTSTUB_VID, BOOTSTUB_PID = 0xADD1, 0xB007
EP_OUT = 0x02
PAGE = 128
USERFW_FLASH_OFFSET = 0x4000
CTRL_TIMEOUT_MS = 2000


# ===== USB =====================================================================

def open_device(vid_pid_list=((BOOTSTUB_VID, BOOTSTUB_PID), (USERFW_VID, USERFW_PID))):
  """Returns (vid, pid, handle, ctx). Raises if no device found."""
  ctx = ctypes.POINTER(libusb.libusb_context)()
  libusb.libusb_init(ctypes.byref(ctx))
  for vid, pid in vid_pid_list:
    h = libusb.libusb_open_device_with_vid_pid(ctx, vid, pid)
    if not h: continue
    if libusb.libusb_kernel_driver_active(h, 0) == 1:
      libusb.libusb_detach_kernel_driver(h, 0)
    libusb.libusb_claim_interface(h, 0)
    return vid, pid, h, ctx
  libusb.libusb_exit(ctx)
  raise RuntimeError(f"no device found ({'/'.join(f'{v:04X}:{p:04X}' for v,p in vid_pid_list)})")


def close_device(h, ctx):
  if h is not None:
    libusb.libusb_release_interface(h, 0)
    libusb.libusb_close(h)
  if ctx is not None:
    libusb.libusb_exit(ctx)


def bulk_out(h, data):
  """Send `data` to bulk EP 0x02 — lands at XDATA 0x7000 (the SPI page buffer)."""
  xdata_write(h, 0x9094, 0x10)            # ARM_OUT (REG_USB_EP_CFG2)
  buf = (ctypes.c_ubyte * len(data))(*data)
  transferred = ctypes.c_int()
  r = libusb.libusb_bulk_transfer(h, EP_OUT, buf, len(data),
                                  ctypes.byref(transferred), 5000)
  assert r == 0, f"bulk OUT failed: {r}"
  assert transferred.value == len(data), \
      f"short bulk OUT: {transferred.value}/{len(data)}"
  time.sleep(0.001)


# ===== SPI flash ===============================================================
# Register sequences matched from emulator trace (trace/flash). All
# operations are driven from the host by poking the chip's flash
# controller registers via 0xE4/0xE5 XDATA r/w.

def flash_poll_busy(h):
  for _ in range(100000):
    if not (xdata_read(h, 0xC8A9, 1)[0] & 0x01): return
  raise TimeoutError("flash CSR busy timeout")


def flash_transaction(h, cmd, addr=0, data_len=0, addr_len=0x07, mode=0x00):
  """Issue a flash command. addr_len: 0x04 = no address; 0x07 = 24-bit."""
  xdata_write(h, 0xC8AD, mode)
  xdata_write(h, 0xC8AE, 0x00)
  xdata_write(h, 0xC8AF, 0x00)
  xdata_write(h, 0xC8AA, cmd)
  xdata_write(h, 0xC8AC, addr_len)
  xdata_write(h, 0xC8A1, addr & 0xFF)
  xdata_write(h, 0xC8A2, (addr >> 8) & 0xFF)
  xdata_write(h, 0xC8AB, (addr >> 16) & 0xFF)
  xdata_write(h, 0xC8A3, (data_len >> 8) & 0xFF)
  xdata_write(h, 0xC8A4, data_len & 0xFF)
  xdata_write(h, 0xC8A9, 0x01)
  flash_poll_busy(h)
  for _ in range(4): xdata_write(h, 0xC8AD, 0x00)


def flash_wren(h):
  """Write Enable. Inlined here to avoid clobbering the 0x7000 buffer."""
  xdata_write(h, 0xC8AD, 0x00)
  xdata_write(h, 0xC8AA, 0x06)
  xdata_write(h, 0xC8AC, 0x04)
  xdata_write(h, 0xC8A3, 0x00)
  xdata_write(h, 0xC8A4, 0x00)
  xdata_write(h, 0xC8A9, 0x01)
  flash_poll_busy(h)


def flash_read_status(h):
  flash_transaction(h, cmd=0x05, data_len=1, addr_len=0x04, mode=0x00)
  return xdata_read(h, 0x7000, 1)[0]


def flash_wait_wip(h, timeout=10.0):
  t0 = time.monotonic()
  while time.monotonic() - t0 < timeout:
    if not (flash_read_status(h) & 0x01): return
    time.sleep(0.01)
  raise TimeoutError("flash WIP timeout")


def flash_init(h):
  """Initialize flash controller and clear block-protect bits."""
  xdata_write(h, 0xCC33, 0x04)
  v = xdata_read(h, 0xCA81, 1)[0]
  xdata_write(h, 0xCA81, v | 0x01)
  xdata_write(h, 0xC805, 0x02)
  xdata_write(h, 0xC8A6, 0x04)            # SPI clock divider
  for _ in range(5):
    flash_wren(h)
    bulk_out(h, bytes([0x00] * 4))
    flash_transaction(h, cmd=0x01, data_len=1, addr_len=0x04, mode=0x01)
    time.sleep(0.01)
    sr = flash_read_status(h)
    if not (sr & 0x1C): return
  raise RuntimeError(f"failed to clear block protect, SR=0x{sr:02X}")


def flash_jedec_id(h):
  flash_transaction(h, cmd=0x9F, data_len=3, addr_len=0x04, mode=0x00)
  return xdata_read(h, 0x7000, 3)


def flash_read(h, addr, size):
  """Read `size` bytes from flash. 4 KB DMA chunks, 64-byte E4 reads (USB2 cap)."""
  result = bytearray()
  while len(result) < size:
    want = min(4096, size - len(result))
    flash_transaction(h, cmd=0x03, addr=addr + len(result), data_len=max(4096, want),
                      addr_len=0x07, mode=0x00)
    read = 0
    while read < want:
      n = min(64, want - read)
      result.extend(xdata_read(h, 0x7000 + read, n))
      read += n
  return bytes(result)


def flash_block_erase(h, addr):
  """Erase a 64KB block."""
  assert (addr & 0xFFFF) == 0
  flash_wren(h)
  flash_transaction(h, cmd=0xD8, addr=addr, data_len=0, addr_len=0x07, mode=0x00)
  flash_wait_wip(h)


def flash_page_program(h, addr, size):
  """Program `size` bytes from the 0x7000 buffer (loaded via bulk_out) to flash."""
  flash_wren(h)
  flash_transaction(h, cmd=0x02, addr=addr, data_len=size, addr_len=0x07, mode=0x01)
  flash_wait_wip(h)


# ===== Image flashing ==========================================================

def flash_image(h, image, addr, preserve_before=True):
  """Erase the 64 KB blocks covering [addr, addr+len) and page-program `image`.

  If `preserve_before` is set and `addr` is past the start of its 64 KB
  block, the leading bytes of that block are read out before the erase
  and rewritten after — needed when userfw at 0x4000 lives in the same
  block as the bootstub at 0x100."""
  end = addr + len(image)
  erase_start = addr & ~0xFFFF
  erase_end   = (end + 0xFFFF) & ~0xFFFF

  preserved = b""
  if preserve_before and addr > erase_start:
    n = addr - erase_start
    print(f"  preserving 0x{erase_start:05X}-0x{addr-1:05X} ({n} bytes) across block erase")
    preserved = flash_read(h, erase_start, n)

  blocks = list(range(erase_start, erase_end, 0x10000))
  print(f"  erasing {len(blocks)} block(s) covering 0x{erase_start:05X}-0x{erase_end-1:05X}")
  for b in blocks: flash_block_erase(h, b)

  for off in range(0, len(preserved), PAGE):
    chunk = preserved[off:off + PAGE]
    if any(b != 0xFF for b in chunk):
      bulk_out(h, chunk + bytes((-len(chunk)) % 4))
      flash_page_program(h, erase_start + off, len(chunk))

  total, written, t0 = len(image), 0, time.monotonic()
  while written < total:
    chunk = image[written:written + PAGE]
    bulk_out(h, chunk + bytes((-len(chunk)) % 4))
    flash_page_program(h, addr + written, len(chunk))
    written += len(chunk)
    rate = written / (time.monotonic() - t0 + 1e-9)
    print(f"\r  writing: {written}/{total} ({100*written//total}%) {rate:.0f} B/s",
          end="", flush=True)
  print()


def verify_image(h, image, addr):
  got = flash_read(h, addr, len(image))
  errs = [i for i, (a, b) in enumerate(zip(got, image)) if a != b]
  for i in errs[:10]:
    print(f"    mismatch at 0x{addr+i:05X}: flash=0x{got[i]:02X} expected=0x{image[i]:02X}")
  return len(errs)


# ===== Bootstub control ========================================================

def request_enter_dfu(h):
  """Userfw 0xEC: write cookie + CC31. Errors are expected (device drops)."""
  libusb.libusb_control_transfer(h, 0x40, 0xEC, 0, 0, None, 0, CTRL_TIMEOUT_MS)


def reboot_via_cc31(h):
  print("  rebooting via CC31 (CPU reset)")
  try: xdata_write(h, 0xCC31, 0x01)
  except Exception: pass


def wait_for_dfu(timeout=10.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    try: return open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
    except RuntimeError: time.sleep(0.2)
  raise TimeoutError("bootstub never enumerated — check serial for [BS]/[DFU]")


# ===== CLI =====================================================================

def cmd_flash(args):
  with open(args.image, "rb") as f: image = f.read()
  if image[:4] != b"ASP2":
    sys.exit(f"error: {args.image} doesn't look like a packed userfw image")
  print(f"image: {args.image} ({len(image)} bytes)")

  vid, pid, h, ctx = open_device()
  try:
    if (vid, pid) == (USERFW_VID, USERFW_PID):
      print("device in app mode — sending 0xEC to enter DFU")
      request_enter_dfu(h)
      close_device(h, ctx)
      vid, pid, h, ctx = wait_for_dfu()
      print("bootstub up.")

    flash_init(h)
    print(f"  JEDEC ID: {flash_jedec_id(h).hex()}")
    flash_image(h, image, USERFW_FLASH_OFFSET)
    errors = verify_image(h, image, USERFW_FLASH_OFFSET)
    if errors: sys.exit(f"FAIL: {errors} byte mismatches")
    print("verify OK")
    if not args.no_reboot: reboot_via_cc31(h)
  finally:
    close_device(h, ctx)


def cmd_enter_dfu(args):
  vid, pid, h, ctx = open_device()
  if (vid, pid) == (BOOTSTUB_VID, BOOTSTUB_PID):
    print("already in DFU mode"); close_device(h, ctx); return
  try: request_enter_dfu(h)
  finally: close_device(h, ctx)
  _, _, h, ctx = wait_for_dfu()
  print("bootstub up.")
  close_device(h, ctx)


def cmd_reboot(args):
  _, _, h, ctx = open_device()
  try: reboot_via_cc31(h)
  finally: close_device(h, ctx)


def cmd_read(args):
  _, _, h, ctx = open_device()
  try:
    flash_init(h)
    data = flash_read(h, args.addr, args.size)
  finally: close_device(h, ctx)
  for off in range(0, len(data), 16):
    line = data[off:off + 16]
    hexs = " ".join(f"{b:02X}" for b in line)
    ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in line)
    print(f"  {args.addr + off:05X}: {hexs:<48s} {ascii_}")


def cmd_id(args):
  _, _, h, ctx = open_device()
  try:
    flash_init(h)
    jedec = flash_jedec_id(h)
  finally: close_device(h, ctx)
  print(f"JEDEC ID: {jedec.hex()} (mfr=0x{jedec[0]:02X} type=0x{jedec[1]:02X} cap=0x{jedec[2]:02X})")


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd")

  pf = sub.add_parser("flash", help="flash a userfw image (default if image given)")
  pf.add_argument("image")
  pf.add_argument("--no-reboot", action="store_true", help="skip the post-flash reboot")
  pf.set_defaults(func=cmd_flash)

  sub.add_parser("enter-dfu", help="ask running userfw to drop to bootstub DFU"
                 ).set_defaults(func=cmd_enter_dfu)
  sub.add_parser("reboot", help="CC31 reset — bootstub re-validates and boots userfw"
                 ).set_defaults(func=cmd_reboot)
  sub.add_parser("id", help="print SPI flash JEDEC ID"
                 ).set_defaults(func=cmd_id)

  pr = sub.add_parser("read", help="hex-dump SPI flash")
  pr.add_argument("addr", type=lambda s: int(s, 0))
  pr.add_argument("size", type=lambda s: int(s, 0), nargs="?", default=256)
  pr.set_defaults(func=cmd_read)

  argv = sys.argv[1:]
  if argv and not argv[0].startswith("-") \
     and argv[0] not in {"flash", "enter-dfu", "reboot", "read", "id"}:
    argv = ["flash", *argv]                 # `flash.py image.bin` shortcut

  args = p.parse_args(argv)
  if not args.cmd: p.print_help(); return 1
  args.func(args)
  return 0


if __name__ == "__main__":
  sys.exit(main())
