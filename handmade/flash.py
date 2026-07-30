#!/usr/bin/env python3
"""Flash bootstub userfw over EP0 DFU."""

import argparse
import ctypes
from pathlib import Path
import struct
import time

from tinygrad.runtime.autogen import libusb
from tinygrad.runtime.support.usb import USB3, checked

USERFW_ID = (0xADD1, 0x0001)
BOOTSTUB_ID = (0xADD1, 0xB007)
USERFW_FLASH_OFFSET = 0x4000
SECTOR_SIZE = 0x1000
PAGE = 64
PAGE_SS = 512
USERFW_IMAGE_END = 0x11040
USERFW_ERASE_END = (USERFW_IMAGE_END + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
TIMEOUT_MS = 5000
DFU_INFO = struct.Struct("<4sBBHHII")
USB_SYSFS = Path("/sys/bus/usb/devices")
TB_SYSFS = Path("/sys/bus/thunderbolt/devices")
TB_DEBUGFS = Path("/sys/kernel/debug/thunderbolt")
BOOT_ABI_VERSION = 1


def usb_nodes(vid, pid=None):
  return [p.parent for p in USB_SYSFS.glob("*/idVendor")
          if int(p.read_text(), 16) == vid and
          (pid is None or int((p.parent / "idProduct").read_text(), 16) == pid)]


def open_device(*ids):
  for vid, pid in ids:
    if h := libusb.libusb_open_device_with_vid_pid(USB3.ctx(), vid, pid):
      return h


def control(h, request, value=0, index=0, data=b"", read=0, timeout=TIMEOUT_MS):
  length = read or len(data)
  buf = (ctypes.c_ubyte * length)(*data)
  direction = 0xC0 if read else 0x40
  ret = checked(libusb.libusb_control_transfer)(
    h, direction, request, value, index, buf, length, timeout)
  assert ret == length, f"short control transfer: {ret}/{length}"
  return bytes(buf) if read else None


def disconnect_request(h, request):
  libusb.libusb_control_transfer(h, 0x40, request, 0, 0, None, 0, TIMEOUT_MS)


def get_info(h):
  info = DFU_INFO.unpack(control(h, 0xB4, read=DFU_INFO.size))
  magic, protocol, abi, page, sector, start, end = info
  valid = (magic == b"A24D" and protocol == 1 and abi == BOOT_ABI_VERSION and
           page in (PAGE, PAGE_SS) and sector == SECTOR_SIZE and
           start == USERFW_FLASH_OFFSET and end == USERFW_IMAGE_END)
  assert valid, f"incompatible DFU: {info}"
  return page


def set_addr(h, addr):
  assert 0 <= addr < 0x1000000, f"invalid flash address: 0x{addr:X}"
  control(h, 0xB1, value=addr & 0xFFFF, index=addr >> 16)


def router_regs():
  routers = [p.parent.name for p in TB_SYSFS.glob("*/vendor")
             if int(p.read_text(), 16) == USERFW_ID[0]]
  assert len(routers) <= 1, f"{len(routers)} ADD1 USB4 routers present"
  return TB_DEBUGFS / routers[0] / "regs" if routers else None


def enter_dfu():
  if h := open_device(BOOTSTUB_ID):
    libusb.libusb_close(h)
    return
  if h := open_device(USERFW_ID):
    disconnect_request(h, 0xEC)
    libusb.libusb_close(h)
    return
  if regs := router_regs():
    print("  entering DFU through the USB4 router")
    regs.write_text("0x0019 0x00000000\n")
    regs.write_text("0x001a 0x800000ec\n")
    return
  assert False, "no application or bootstub found"


def wait_dfu(timeout=30):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if h := open_device(BOOTSTUB_ID):
      return h
    time.sleep(0.2)
  raise TimeoutError("bootstub did not enumerate")


def wait_app(timeout=60):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if nodes := usb_nodes(*USERFW_ID):
      return f"USB at {int(float((nodes[0] / 'speed').read_text()))} Mb/s"
    if router_regs():
      return "USB4 router"
    time.sleep(0.2)
  raise TimeoutError("application did not enumerate")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("command", choices=("flash", "check"))
  parser.add_argument("image", nargs="?")
  parser.add_argument("--no-reboot", action="store_true")
  parser.add_argument("--verify", action="store_true")
  parser.add_argument("--timeout", type=float, default=60)
  args = parser.parse_args()

  assert len(usb_nodes(USERFW_ID[0])) <= 1, "multiple ADD1 USB devices present"
  if args.command == "flash":
    assert args.image, "flash requires an image"
    image = Path(args.image).read_bytes()
    assert 0 < len(image) <= USERFW_IMAGE_END - USERFW_FLASH_OFFSET
    enter_dfu()
    h = wait_dfu()
    page = get_info(h)
    print(f"bootstub up: {page}-byte writes")

    erase_len = (len(image) + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1)
    assert erase_len <= USERFW_ERASE_END - USERFW_FLASH_OFFSET
    print(f"  erasing {erase_len // SECTOR_SIZE} sectors")
    control(h, 0xB0, data=struct.pack("<II", USERFW_FLASH_OFFSET, erase_len),
            timeout=max(TIMEOUT_MS, 2000 * erase_len // SECTOR_SIZE))

    print(f"  writing {len(image)} bytes")
    for off in range(0, len(image), page):
      set_addr(h, USERFW_FLASH_OFFSET + off)
      control(h, 0xB2, data=image[off:off + page])

    if args.verify:
      set_addr(h, USERFW_FLASH_OFFSET)
      got = b"".join(control(h, 0xB3, read=min(PAGE, len(image) - off))
                     for off in range(0, len(image), PAGE))
      assert got == image, "flash verification failed"
      print("  verify OK")
    if not args.no_reboot:
      disconnect_request(h, 0xEB)
    libusb.libusb_close(h)
    if not args.no_reboot:
      print(f"application up: {wait_app()}")
  else:
    print(f"application up: {wait_app(args.timeout)}")
    enter_dfu()
    h = wait_dfu(args.timeout)
    print(f"bootstub up: {get_info(h)}-byte writes")
    disconnect_request(h, 0xEB)
    libusb.libusb_close(h)
    print(f"application up: {wait_app(args.timeout)}")
