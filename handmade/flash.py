#!/usr/bin/env python3
"""Host-side flasher for the ASM2464PD bootstub.

Bootstub DFU is EP0-only (ADD1:B007). If the app is running (ADD1:0001),
the flasher asks it to reset into DFU before erasing, writing, verifying,
and rebooting the userfw image at SPI flash 0x4000.
"""

import argparse
import ctypes
import glob
import os
import struct
import sys
import time

from tinygrad.runtime.autogen import libusb

USERFW_VID,   USERFW_PID   = 0xADD1, 0x0001
BOOTSTUB_VID, BOOTSTUB_PID = 0xADD1, 0xB007

USERFW_FLASH_OFFSET = 0x4000
SECTOR_SIZE         = 0x1000
PAGE                = 64        # USB 2.0 EP0 max packet — see protocol comment in bootstub.c
PAGE_SS             = 512       # SuperSpeed EP0 max packet (bootstub bcdDevice >= 0x0002)
USERFW_HEADER_SIZE  = 0x40
USERFW_BODY_LIMIT   = 0xDC00
USERFW_IMAGE_END    = USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + USERFW_BODY_LIMIT
USERFW_ERASE_END    = (USERFW_IMAGE_END + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)

TIMEOUT_MS = 5000
FLASH_ADDR24_LIMIT = 0x1000000


def _range_ok(addr, length, end):
  return length > 0 and USERFW_FLASH_OFFSET <= addr <= end and length <= end - addr


def _addr24_range_ok(addr, length):
  return length > 0 and 0 <= addr < FLASH_ADDR24_LIMIT and length <= FLASH_ADDR24_LIMIT - addr


def _check(ret, what):
  if ret < 0:
    raise RuntimeError(f"{what} failed (libusb={ret})")


def count_add1_usb():
  """Number of ASM2464 (VID 0xADD1) USB devices currently on the host."""
  n = 0
  for d in glob.glob("/sys/bus/usb/devices/*"):
    try:
      if int(open(d + "/idVendor").read().strip(), 16) == 0xADD1:
        n += 1
    except OSError:
      pass
  return n


def assert_single_card():
  """Refuse to act when more than one ASM2464 card is on the host — the flasher
  selects by VID:PID only and cannot tell two cards apart, so guessing risks
  cross-flashing the wrong one. Update one card at a time."""
  n = count_add1_usb()
  if n > 1:
    sys.exit(f"error: {n} ADD1 USB devices present — refusing to guess which to flash. "
             "Update one card at a time (this flasher has no per-card selection).")


def open_device(vid_pid_list=((BOOTSTUB_VID, BOOTSTUB_PID), (USERFW_VID, USERFW_PID))):
  ctx = ctypes.POINTER(libusb.libusb_context)()
  _check(libusb.libusb_init(ctypes.byref(ctx)), "libusb_init")
  last_error = None
  for vid, pid in vid_pid_list:
    h = libusb.libusb_open_device_with_vid_pid(ctx, vid, pid)
    if not h: continue
    try:
      if libusb.libusb_kernel_driver_active(h, 0) == 1:
        _check(libusb.libusb_detach_kernel_driver(h, 0), f"detach kernel driver {vid:04X}:{pid:04X}")
      _check(libusb.libusb_claim_interface(h, 0), f"claim interface {vid:04X}:{pid:04X}")
      return vid, pid, h, ctx
    except RuntimeError as e:
      last_error = e
      libusb.libusb_close(h)
  libusb.libusb_exit(ctx)
  if last_error is not None:
    raise RuntimeError(str(last_error))
  raise RuntimeError(f"no device found ({'/'.join(f'{v:04X}:{p:04X}' for v,p in vid_pid_list)})")


def close_device(h, ctx):
  if h is not None:
    libusb.libusb_release_interface(h, 0)
    libusb.libusb_close(h)
  if ctx is not None:
    libusb.libusb_exit(ctx)


def _ctl(h, bm, bReq, wValue, wIndex, data, wLen, what, timeout=TIMEOUT_MS):
  buf = (ctypes.c_ubyte * wLen)(*data) if data else (ctypes.c_ubyte * wLen)()
  r = libusb.libusb_control_transfer(h, bm, bReq, wValue, wIndex, buf, wLen, timeout)
  if r < 0: raise IOError(f"{what}: control transfer failed (libusb={r})")
  if bm & 0x80:                                # IN: returned byte count
    return bytes(buf[:r])
  if r != wLen:                                # OUT: chip must have accepted everything
    raise IOError(f"{what}: short transfer ({r}/{wLen})")
  return None


def cmd_erase(h, addr, length):
  if (addr & (SECTOR_SIZE - 1)) or (length & (SECTOR_SIZE - 1)) or \
     not _range_ok(addr, length, USERFW_ERASE_END):
    raise ValueError(f"erase range outside userfw area: 0x{addr:05X}+0x{length:X}")
  # One 0xB0 erases every sector in the range inside a single control transfer;
  # a full-region erase on slow/aged NOR can far exceed the default 5s. Scale
  # the timeout to the sector count (a generous ~2s/sector).
  n_sectors = length // SECTOR_SIZE
  _ctl(h, 0x40, 0xB0, 0, 0, struct.pack("<II", addr, length), 8,
       f"erase 0x{addr:05X}+0x{length:X}", timeout=max(TIMEOUT_MS, 2000 * n_sectors))

def cmd_set_addr(h, addr):
  if not (0 <= addr < FLASH_ADDR24_LIMIT):
    raise ValueError(f"address outside 24-bit flash address space: 0x{addr:X}")
  _ctl(h, 0x40, 0xB1, addr & 0xFFFF, (addr >> 16) & 0xFF, None, 0,
       f"set_addr 0x{addr:05X}")

def cmd_read(h, length):
  out = bytearray()
  while len(out) < length:
    n = min(PAGE, length - len(out))
    out.extend(_ctl(h, 0xC0, 0xB3, 0, 0, None, n, f"read +{len(out)}"))
  return bytes(out)

def cmd_reboot(h):
  print("  rebooting (0xEB)")
  try: _ctl(h, 0x40, 0xEB, 0, 0, None, 0, "reboot")
  except IOError: pass        # the device disappears mid-status; ignore


def request_enter_dfu(h):
  """Ask the running app (USB function) to reset into bootstub DFU."""
  try: _ctl(h, 0x40, 0xEC, 0, 0, None, 0, "enter-dfu")
  except IOError: pass        # the device may disappear during status


def router_debugfs_regs(vid=0xADD1):
  """Path to the tiny USB4 router's debugfs regs file, or None. Used when the
  app runs as a USB4 router (the primary eGPU topology) and exposes no USB
  function to send 0xEC to."""
  for p in glob.glob("/sys/bus/thunderbolt/devices/*/vendor"):
    try:
      if int(open(p).read().strip(), 16) != vid:
        continue
    except (OSError, ValueError):
      continue
    name = os.path.basename(os.path.dirname(p))
    regs = f"/sys/kernel/debug/thunderbolt/{name}/regs"
    if os.path.exists(regs):
      return regs
  return None


def request_enter_dfu_router(regs):
  """Post the TINY_ENTER_DFU (0xEC) native router op via debugfs; the firmware
  acks it, sets the DFU cookie and CPU-resets. Needs root + thunderbolt
  debugfs mounted."""
  try:
    with open(regs, "w") as f: f.write("0x0019 0x00000000\n")
    with open(regs, "w") as f: f.write("0x001a 0x800000ec\n")
    return True
  except OSError as e:
    print(f"  router-op DFU entry failed ({e}); is debugfs mounted and are you root?")
    return False


def enter_dfu_from_app():
  """Drop a running app to bootstub DFU, whether it is a USB device (0xEC) or a
  USB4 router (native router op). No-op if already in DFU."""
  try:
    vid, pid, h, ctx = open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
    close_device(h, ctx)
    return True                                  # already in DFU
  except RuntimeError:
    pass
  try:
    vid, pid, h, ctx = open_device([(USERFW_VID, USERFW_PID)])
    try: request_enter_dfu(h)
    finally: close_device(h, ctx)
    return True
  except RuntimeError:
    pass
  regs = router_debugfs_regs()
  if regs is not None:
    print("  app is a USB4 router — posting native router-op DFU entry")
    return request_enter_dfu_router(regs)
  return False


def wait_for_dfu(timeout=10.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    try: return open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
    except RuntimeError: time.sleep(0.2)
  raise TimeoutError("bootstub never enumerated — check serial for [BS]/[DFU]")


def sysfs_usb_speed(vid, pid):
  """Speed (Mb/s) of the first vid:pid USB device in sysfs, or None."""
  for d in glob.glob("/sys/bus/usb/devices/*"):
    try:
      with open(d + "/idVendor") as f: v = int(f.read().strip(), 16)
      with open(d + "/idProduct") as f: p = int(f.read().strip(), 16)
      if v == vid and p == pid:
        with open(d + "/speed") as f: return int(float(f.read().strip()))
    except (OSError, ValueError):
      continue
  return None


def dfu_write_chunk():
  """Largest DFU write payload the enumerated bootstub supports: 512 bytes
  when it is at SuperSpeed and advertises bcdDevice >= 0x0002, else 64."""
  for d in glob.glob("/sys/bus/usb/devices/*"):
    try:
      with open(d + "/idVendor") as f: v = int(f.read().strip(), 16)
      with open(d + "/idProduct") as f: pr = int(f.read().strip(), 16)
      if v == BOOTSTUB_VID and pr == BOOTSTUB_PID:
        with open(d + "/bcdDevice") as f: bcd = int(f.read().strip(), 16)
        with open(d + "/speed") as f: spd = int(float(f.read().strip()))
        return PAGE_SS if (bcd >= 2 and spd >= 5000) else PAGE
    except (OSError, ValueError):
      continue
  return PAGE


def sysfs_usb4_router_present(vid=0xADD1):
  for d in glob.glob("/sys/bus/thunderbolt/devices/*/vendor"):
    try:
      with open(d) as f:
        if int(f.read().strip(), 16) == vid: return True
    except (OSError, ValueError):
      continue
  return False


def verify_app_link(max_bounces=2, timeout=240.0):
  """Wait for the app and make sure its USB link is SuperSpeed.

  A boot via the bootstub 0xEB reset occasionally comes up with the SS PHY
  in a state that only trains high speed; one extra trip through the
  bootstub DFU reliably retrains it. On USB4 hosts the app may come up as
  a USB4 router with no USB function - that counts as healthy too."""
  spd = None
  for bounce in range(max_bounces + 1):
    deadline = time.monotonic() + timeout
    spd = None
    while time.monotonic() < deadline:
      spd = sysfs_usb_speed(USERFW_VID, USERFW_PID)
      if spd is not None: break
      if sysfs_usb4_router_present():
        print("  app up as a USB4 router")
        return True
      time.sleep(0.25)
    if spd is None:
      print("  app did not enumerate")
      return False
    if spd >= 5000:
      print(f"  app up at {spd} Mb/s")
      return True
    if bounce == max_bounces: break
    print(f"  app at {spd} Mb/s - bouncing through DFU to retrain SuperSpeed")
    time.sleep(1.0)
    try:
      vid, pid, h, ctx = open_device([(USERFW_VID, USERFW_PID)])
      try: request_enter_dfu(h)
      finally: close_device(h, ctx)
      _, _, h, ctx = wait_for_dfu()
      try: cmd_reboot(h)
      finally: close_device(h, ctx)
    except (RuntimeError, TimeoutError) as e:
      print(f"  bounce failed: {e}")
      return False
  # Still high speed after the retrain bounces. On a genuine USB 2.0 host/hub
  # this is the correct, expected link and the app is fully functional — do
  # NOT fail the flash. (On an SS-capable host it would mean the retrain
  # didn't take, hence the warning.)
  print(f"  WARNING: app at {spd} Mb/s after retrain (USB 2.0 host? — app is up and usable)")
  return True


def flash_image(h, image, addr, verify=False):
  if addr % SECTOR_SIZE: sys.exit(f"image addr 0x{addr:X} not sector-aligned")
  if not _range_ok(addr, len(image), USERFW_IMAGE_END):
    sys.exit(f"image too large for userfw area: 0x{addr:05X}+0x{len(image):X}")
  erase_len = (len(image) + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
  if not _range_ok(addr, erase_len, USERFW_ERASE_END):
    sys.exit(f"erase range too large for userfw area: 0x{addr:05X}+0x{erase_len:X}")

  print(f"  erasing 0x{addr:05X}-0x{addr + erase_len - 1:05X} ({erase_len // SECTOR_SIZE} sectors)")
  cmd_erase(h, addr, erase_len)

  page = dfu_write_chunk()
  print(f"  writing {len(image)} bytes ({page}-byte packets)")
  cmd_set_addr(h, addr)
  t0 = time.monotonic()
  for off in range(0, len(image), page):
    chunk = image[off:off + page]
    _ctl(h, 0x40, 0xB2, 0, 0, chunk, len(chunk), f"write @0x{addr+off:05X}")
    rate = (off + len(chunk)) / (time.monotonic() - t0 + 1e-9)
    print(f"\r  {off + len(chunk)}/{len(image)} ({100 * (off + len(chunk)) // len(image)}%) {rate:.0f} B/s",
          end="", flush=True)
  print()

  if verify:
    # Optional: the bootstub CRC-checks the image on every boot anyway.
    print("  verifying")
    cmd_set_addr(h, addr)
    got = cmd_read(h, len(image))
    errs = [i for i, (a, b) in enumerate(zip(got, image)) if a != b]
    for i in errs[:10]:
      print(f"    mismatch at 0x{addr+i:05X}: flash=0x{got[i]:02X} expected=0x{image[i]:02X}")
    if errs: sys.exit(f"FAIL: {len(errs)} byte mismatches")
    print("verify OK")


def cmd_flash(args):
  assert_single_card()
  with open(args.image, "rb") as f: image = f.read()
  if image[:4] != b"A24F":
    sys.exit(f"error: {args.image} doesn't look like a packed userfw image")
  print(f"image: {args.image} ({len(image)} bytes)")

  # Converge to bootstub DFU first, handling app-as-USB-device (0xEC) and
  # app-as-USB4-router (native router op); no-op if already in DFU.
  if not enter_dfu_from_app():
    sys.exit("FAIL: no app or DFU device found to flash")
  _, _, h, ctx = wait_for_dfu()
  print("bootstub up.")
  try:
    flash_image(h, image, USERFW_FLASH_OFFSET, verify=args.verify)
    if not args.no_reboot: cmd_reboot(h)
  finally:
    close_device(h, ctx)
  if not args.no_reboot and not verify_app_link():
    sys.exit("FAIL: app link verification failed")


def cmd_enter_dfu(args):
  assert_single_card()
  if not enter_dfu_from_app():
    sys.exit("FAIL: no app or DFU device found")
  _, _, h, ctx = wait_for_dfu()
  print("bootstub up.")
  close_device(h, ctx)


def cmd_reboot_only(args):
  _, _, h, ctx = open_device()
  try: cmd_reboot(h)
  finally: close_device(h, ctx)
  if not verify_app_link():
    sys.exit("FAIL: app link verification failed")


def cmd_read_dump(args):
  if not _addr24_range_ok(args.addr, args.size):
    sys.exit(f"read range outside 24-bit flash address space: 0x{args.addr:X}+0x{args.size:X}")
  _, _, h, ctx = open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
  try:
    cmd_set_addr(h, args.addr)
    data = cmd_read(h, args.size)
  finally:
    close_device(h, ctx)
  for off in range(0, len(data), 16):
    line = data[off:off + 16]
    hexs  = " ".join(f"{b:02X}" for b in line)
    ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in line)
    print(f"  {args.addr + off:05X}: {hexs:<48s} {ascii_}")


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd")

  pf = sub.add_parser("flash", help="flash a userfw image (default if image given)")
  pf.add_argument("image")
  pf.add_argument("--no-reboot", action="store_true", help="skip the post-flash reboot")
  pf.add_argument("--verify", action="store_true",
                  help="read back and compare (the bootstub CRC-checks at boot regardless)")
  pf.set_defaults(func=cmd_flash)

  sub.add_parser("enter-dfu", help="ask running userfw to drop to bootstub DFU"
                 ).set_defaults(func=cmd_enter_dfu)
  sub.add_parser("reboot", help="0xEB reset; bootstub re-validates and boots userfw"
                 ).set_defaults(func=cmd_reboot_only)

  pr = sub.add_parser("read", help="hex-dump SPI flash")
  pr.add_argument("addr", type=lambda s: int(s, 0))
  pr.add_argument("size", type=lambda s: int(s, 0), nargs="?", default=256)
  pr.set_defaults(func=cmd_read_dump)

  argv = sys.argv[1:]
  if argv and not argv[0].startswith("-") \
     and argv[0] not in {"flash", "enter-dfu", "reboot", "read"}:
    argv = ["flash", *argv]

  args = p.parse_args(argv)
  if not args.cmd: p.print_help(); return 1
  args.func(args)
  return 0


if __name__ == "__main__":
  sys.exit(main())
