#!/usr/bin/env python3
"""Flash bootstub userfw over EP0 DFU."""

import argparse
import ctypes
from pathlib import Path
import struct
import sys
import time
import zlib

try:
  from tinygrad.runtime.autogen import libusb
except ModuleNotFoundError:
  libusb = None

USERFW_VID,   USERFW_PID   = 0xADD1, 0x0001
BOOTSTUB_VID, BOOTSTUB_PID = 0xADD1, 0xB007

USERFW_FLASH_OFFSET = 0x4000
SECTOR_SIZE         = 0x1000
PAGE                = 64        # USB 2.0 EP0 max packet — see protocol comment in bootstub.c
PAGE_SS             = 512       # SuperSpeed EP0 max packet (bootstub bcdDevice >= 0x0002)
USERFW_HEADER_SIZE  = 0x40
USERFW_BODY_LIMIT   = 0xD000
USERFW_IMAGE_END    = USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + USERFW_BODY_LIMIT
USERFW_ERASE_END    = (USERFW_IMAGE_END + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)

TIMEOUT_MS = 5000
FLASH_ADDR24_LIMIT = 0x1000000
DFU_PROTOCOL_VERSION = 1
BOOT_ABI_VERSION = 1
DFU_INFO = struct.Struct("<4sBBBBHHII")
DFU_STATUS = struct.Struct("<BBBB")

DFU_STATUS_NAMES = ("ok", "bad request", "range error", "USB DMA timeout",
                    "flash unlock failure", "flash erase failure",
                    "flash program failure", "flash read failure", "aborted")
USB_SYSFS = Path("/sys/bus/usb/devices")
TB_SYSFS = Path("/sys/bus/thunderbolt/devices")
TB_DEBUGFS = Path("/sys/kernel/debug/thunderbolt")


def _range_ok(addr, length, end):
  return length > 0 and USERFW_FLASH_OFFSET <= addr <= end and length <= end - addr


def _check(ret, what):
  if ret < 0:
    raise RuntimeError(f"{what} failed (libusb={ret})")


def read_hex(path):
  try:
    return int(path.read_text().strip(), 16)
  except (OSError, ValueError):
    return None


def count_add1_usb():
  return sum(read_hex(d / "idVendor") == USERFW_VID for d in USB_SYSFS.glob("*"))


def assert_single_card():
  n = count_add1_usb()
  if n > 1:
    sys.exit(f"error: {n} ADD1 USB devices present — refusing to guess which to flash. "
             "Update one card at a time (this flasher has no per-card selection).")


def open_device(vid_pid_list=((BOOTSTUB_VID, BOOTSTUB_PID), (USERFW_VID, USERFW_PID))):
  if libusb is None:
    raise RuntimeError("tinygrad.runtime.autogen.libusb is required for USB operations")
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
      return h, ctx
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
  if bm & 0x80:                                # IN: require the complete reply
    if r != wLen:
      raise IOError(f"{what}: short transfer ({r}/{wLen})")
    return bytes(buf[:r])
  if r != wLen:                                # OUT: chip must have accepted everything
    raise IOError(f"{what}: short transfer ({r}/{wLen})")
  return None


def validate_image(image, source="<image>"):
  def require(ok, message):
    if not ok: raise ValueError(f"{source}: {message}")
  require(len(image) >= USERFW_HEADER_SIZE, "truncated header")
  require(image[:4] == b"A24F", "bad image magic")
  require(image[0x1B] == BOOT_ABI_VERSION, f"unsupported boot ABI {image[0x1B]}")
  body_len, stored_crc = struct.unpack_from("<II", image, 0x1C)
  require(0 < body_len <= USERFW_BODY_LIMIT, f"invalid body length {body_len}")
  require(len(image) == USERFW_HEADER_SIZE + body_len, "length mismatch")
  require(not any(image[0x24:USERFW_HEADER_SIZE]), "nonzero reserved header bytes")
  crc = zlib.crc32(image[:0x20] + image[USERFW_HEADER_SIZE:]) & 0xFFFFFFFF
  require(crc == stored_crc, f"CRC mismatch ({stored_crc:08x} != {crc:08x})")
  require(_range_ok(USERFW_FLASH_OFFSET, len(image), USERFW_IMAGE_END),
          "image exceeds partition")
  return body_len


def cmd_get_info(h):
  fields = DFU_INFO.unpack(_ctl(h, 0xC0, 0xB4, 0, 0, None, DFU_INFO.size, "get_info"))
  magic, protocol, boot_abi, flags, reserved, max_write, sector, start, end = fields
  if magic != b"A24D": raise RuntimeError(f"bad DFU magic {magic!r}")
  if protocol != DFU_PROTOCOL_VERSION:
    raise RuntimeError(f"unsupported DFU protocol {protocol}")
  if boot_abi != BOOT_ABI_VERSION: raise RuntimeError(f"unsupported boot ABI {boot_abi}")
  expected = (reserved == 0 and flags & 1 and max_write in (PAGE, PAGE_SS) and
              sector == SECTOR_SIZE and start == USERFW_FLASH_OFFSET and
              end == USERFW_IMAGE_END)
  if not expected: raise RuntimeError(f"incompatible DFU geometry {fields[3:]}")
  return max_write


def dfu_error_context(h, error):
  try:
    protocol, status, last_op, pending_op = DFU_STATUS.unpack(
      _ctl(h, 0xC0, 0xB5, 0, 0, None, DFU_STATUS.size, "get_status"))
    name = DFU_STATUS_NAMES[status] if status < len(DFU_STATUS_NAMES) else status
    return IOError(f"{error}; bootstub={name}, last_op=0x{last_op:02X}, pending={pending_op}")
  except Exception as status_error:
    return IOError(f"{error}; status unavailable ({status_error})")


def cmd_erase(h, addr, length):
  if (addr & (SECTOR_SIZE - 1)) or (length & (SECTOR_SIZE - 1)) or \
     not _range_ok(addr, length, USERFW_ERASE_END):
    raise ValueError(f"erase range outside userfw area: 0x{addr:05X}+0x{length:X}")
  # A full-region erase can exceed the default control-transfer timeout.
  n_sectors = length // SECTOR_SIZE
  error = None
  for attempt in range(2):
    try:
      _ctl(h, 0x40, 0xB0, 0, 0, struct.pack("<II", addr, length), 8,
           f"erase 0x{addr:05X}+0x{length:X}", timeout=max(TIMEOUT_MS, 2000 * n_sectors))
      return
    except IOError as exc:
      error = exc
      if attempt == 0:
        print("  erase outcome unknown; safely retrying the same sectors")
  raise dfu_error_context(h, error)

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


def cmd_write(h, addr, data, retries=3):
  """Write one explicitly addressed packet; replaying identical NOR data is
  safe when the host lost only the status phase of the prior attempt."""
  error = None
  for attempt in range(retries):
    try:
      cmd_set_addr(h, addr)
      _ctl(h, 0x40, 0xB2, 0, 0, data, len(data), f"write @0x{addr:05X}")
      return
    except IOError as exc:
      error = exc
      if attempt + 1 < retries:
        try:
          _ctl(h, 0x40, 0xB6, 0, 0, None, 0, "abort")
        except Exception:
          pass
        print(f"\n  write @0x{addr:05X} outcome unknown; retrying identical data")
  raise dfu_error_context(h, error)

def cmd_reboot(h):
  print("  rebooting (0xEB)")
  try: _ctl(h, 0x40, 0xEB, 0, 0, None, 0, "reboot")
  except IOError: pass        # the device disappears mid-status; ignore


def request_enter_dfu(h):
  try: _ctl(h, 0x40, 0xEC, 0, 0, None, 0, "enter-dfu")
  except IOError: pass        # the device may disappear during status


def router_debugfs_regs(vid=0xADD1):
  routers = [p.parent.name for p in TB_SYSFS.glob("*/vendor") if read_hex(p) == vid]
  if len(routers) > 1:
    raise RuntimeError(f"{len(routers)} ADD1 USB4 routers present; refusing to guess")
  regs = TB_DEBUGFS / routers[0] / "regs" if routers else None
  return regs if regs is not None and regs.exists() else None


def request_enter_dfu_router(regs):
  try:
    with open(regs, "w") as f: f.write("0x0019 0x00000000\n")
    with open(regs, "w") as f: f.write("0x001a 0x800000ec\n")
    return True
  except OSError as e:
    print(f"  router-op DFU entry failed ({e}); is debugfs mounted and are you root?")
    return False


def enter_dfu_from_app():
  try:
    h, ctx = open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
    close_device(h, ctx)
    return True                                  # already in DFU
  except RuntimeError:
    pass
  try:
    h, ctx = open_device([(USERFW_VID, USERFW_PID)])
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


def wait_for_app(timeout=60.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    speed = sysfs_usb_speed(USERFW_VID, USERFW_PID)
    if speed is not None:
      return f"USB at {speed} Mb/s"
    if sysfs_usb4_router_present():
      return "USB4 router"
    time.sleep(0.2)
  raise TimeoutError("application never enumerated")


def test_app():
  h, ctx = open_device([(USERFW_VID, USERFW_PID)])
  payload = bytes(((i * 37) + 11) & 0xFF for i in range(512))
  try:
    _ctl(h, 0x40, 0xF2, 1, 0x0100, None, 0, "arm SRAM bulk OUT")
    buf, done = (ctypes.c_ubyte * len(payload))(*payload), ctypes.c_int()
    _check(libusb.libusb_bulk_transfer(h, 0x02, buf, len(buf), ctypes.byref(done), TIMEOUT_MS),
           "bulk OUT")
    if done.value != len(payload): raise IOError(f"short bulk OUT ({done.value}/{len(payload)})")
    got = b"".join(_ctl(h, 0xC0, 0xE4, 0xF000 + off, 0, None,
                        min(PAGE, len(payload)-off), "SRAM readback")
                   for off in range(0, len(payload), PAGE))
    if got != payload: raise IOError("bulk SRAM readback mismatch")
  finally:
    close_device(h, ctx)


def sysfs_usb_speed(vid, pid):
  for d in USB_SYSFS.glob("*"):
    if read_hex(d / "idVendor") == vid and read_hex(d / "idProduct") == pid:
      try:
        return int(float((d / "speed").read_text().strip()))
      except (OSError, ValueError):
        return None
  return None


def sysfs_usb4_router_present(vid=0xADD1):
  return any(read_hex(path) == vid for path in TB_SYSFS.glob("*/vendor"))


def flash_image(h, image, page, verify=False):
  addr = USERFW_FLASH_OFFSET
  if not _range_ok(addr, len(image), USERFW_IMAGE_END):
    sys.exit(f"image too large for userfw area: 0x{addr:05X}+0x{len(image):X}")
  erase_len = (len(image) + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
  if not _range_ok(addr, erase_len, USERFW_ERASE_END):
    sys.exit(f"erase range too large for userfw area: 0x{addr:05X}+0x{erase_len:X}")

  print(f"  erasing 0x{addr:05X}-0x{addr + erase_len - 1:05X} ({erase_len // SECTOR_SIZE} sectors)")
  cmd_erase(h, addr, erase_len)

  print(f"  writing {len(image)} bytes ({page}-byte packets)")
  t0 = time.monotonic()
  for off in range(0, len(image), page):
    chunk = image[off:off + page]
    cmd_write(h, addr + off, chunk)
    rate = (off + len(chunk)) / (time.monotonic() - t0 + 1e-9)
    print(f"\r  {off + len(chunk)}/{len(image)} ({100 * (off + len(chunk)) // len(image)}%) {rate:.0f} B/s",
          end="", flush=True)
  print()

  if verify:
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
  image = Path(args.image).read_bytes()
  try:
    validate_image(image, args.image)
  except ValueError as exc:
    sys.exit(f"error: {exc}")
  print(f"image: {args.image} ({len(image)} bytes)")

  if not enter_dfu_from_app():
    sys.exit("FAIL: no app or DFU device found to flash")
  h, ctx = wait_for_dfu()
  print("bootstub up.")
  try:
    page = cmd_get_info(h)
    print(f"  DFU v1, boot ABI 1, max write {page} bytes")
    flash_image(h, image, page, verify=args.verify)
    if not args.no_reboot: cmd_reboot(h)
  finally:
    close_device(h, ctx)
  if not args.no_reboot:
    print(f"application up: {wait_for_app()}")


def cmd_enter_dfu(args):
  assert_single_card()
  if not enter_dfu_from_app():
    sys.exit("FAIL: no app or DFU device found")
  h, ctx = wait_for_dfu()
  try:
    cmd_get_info(h)
    print("bootstub up: DFU v1, boot ABI 1.")
  finally:
    close_device(h, ctx)


def cmd_reboot_only(args):
  assert_single_card()
  h, ctx = open_device([(BOOTSTUB_VID, BOOTSTUB_PID)])
  try: cmd_reboot(h)
  finally: close_device(h, ctx)
  print(f"application up: {wait_for_app()}")


def cmd_wait_dfu(args):
  h, ctx = wait_for_dfu(args.timeout)
  try:
    page = cmd_get_info(h)
    print(f"bootstub up: DFU v1, boot ABI 1, max write {page} bytes")
  finally:
    close_device(h, ctx)


def cmd_wait_app(args):
  print(f"application up: {wait_for_app(args.timeout)}")
  if args.test:
    test_app()
    print("application control/bulk test passed")


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd", required=True)

  pf = sub.add_parser("flash", help="flash a userfw image")
  pf.add_argument("image")
  pf.add_argument("--no-reboot", action="store_true", help="skip the post-flash reboot")
  pf.add_argument("--verify", action="store_true",
                  help="read back and compare (the bootstub CRC-checks at boot regardless)")
  pf.set_defaults(func=cmd_flash)

  sub.add_parser("enter-dfu", help="ask running userfw to drop to bootstub DFU"
                 ).set_defaults(func=cmd_enter_dfu)
  sub.add_parser("reboot", help="0xEB reset; bootstub re-validates and boots userfw"
                 ).set_defaults(func=cmd_reboot_only)

  pwd = sub.add_parser("wait-dfu", help="wait for bootstub DFU and validate its protocol")
  pwd.add_argument("--timeout", type=float, default=30.0)
  pwd.set_defaults(func=cmd_wait_dfu)

  pwa = sub.add_parser("wait-app", help="wait for the application USB function or USB4 router")
  pwa.add_argument("--timeout", type=float, default=60.0)
  pwa.add_argument("--test", action="store_true", help="verify control and bulk data")
  pwa.set_defaults(func=cmd_wait_app)

  args = p.parse_args()
  args.func(args)
  return 0


if __name__ == "__main__":
  sys.exit(main())
