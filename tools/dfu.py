#!/usr/bin/env python3
"""Host-side DFU flasher for the ASM2464PD bootstub.

Operates in two modes depending on which VID/PID is on the bus:

  - Bootstub DFU (ADD1:B007): vendor-class device with one bulk OUT EP.
    Same protocol as handmade/e4_flash.py — 0xE4 (XDATA read) + 0xE5 (XDATA
    write) over control transfers, plus bulk OUT to fill the SPI page
    buffer at 0x7000.

  - Userfw (ADD1:0001) running our handmade userfw skeleton: sends the 0xEC
    vendor command which sets the DFU cookie and triggers CC31. The chip's
    XDATA survives the reset, so the bootstub sees the cookie on its next
    boot and stays in DFU mode.

Wire format consumed: see tools/pack_userfw.py.

Usage:
  tools/dfu.py userfw_image.bin           # full flash + verify + boot
  tools/dfu.py --enter-dfu                # ask running userfw to drop to DFU
  tools/dfu.py --read 0x4000 256          # hex-dump SPI flash
  tools/dfu.py --jedec                    # SPI JEDEC ID
  tools/dfu.py --reboot                   # LJMP 0x0000 (re-run bootstub)
"""

import argparse
import ctypes
import os
import struct
import sys
import time

from tinygrad.runtime.autogen import libusb

# --- USB transport ---------------------------------------------------------

USERFW_VID, USERFW_PID = 0xADD1, 0x0001
BOOTSTUB_VID, BOOTSTUB_PID = 0xADD1, 0xB007

CTRL_TIMEOUT_MS = 2000


def _open(vid: int, pid: int):
    ctx = ctypes.POINTER(libusb.libusb_context)()
    libusb.libusb_init(ctypes.byref(ctx))
    h = libusb.libusb_open_device_with_vid_pid(ctx, vid, pid)
    if not h:
        libusb.libusb_exit(ctx)
        return None, None
    if libusb.libusb_kernel_driver_active(h, 0) == 1:
        libusb.libusb_detach_kernel_driver(h, 0)
    libusb.libusb_claim_interface(h, 0)
    return h, ctx


def _close(h, ctx):
    if h is not None:
        libusb.libusb_release_interface(h, 0)
        libusb.libusb_close(h)
    if ctx is not None:
        libusb.libusb_exit(ctx)


def find_device():
    """Returns ("dfu"|"app", handle, ctx) or (None, None, None)."""
    h, ctx = _open(BOOTSTUB_VID, BOOTSTUB_PID)
    if h: return "dfu", h, ctx
    h, ctx = _open(USERFW_VID, USERFW_PID)
    if h: return "app", h, ctx
    return None, None, None


def wait_for_dfu(timeout=10.0):
    """Poll for the bootstub VID/PID to appear after a CC31 reset."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        h, ctx = _open(BOOTSTUB_VID, BOOTSTUB_PID)
        if h: return h, ctx
        time.sleep(0.2)
    raise TimeoutError("bootstub never enumerated — check serial for [BS]/[DFU]")


def xdata_read(h, addr: int, n: int = 1) -> bytes:
    assert 0 < n <= 64, "0xE4 max 64 bytes per transfer"
    buf = (ctypes.c_ubyte * n)()
    r = libusb.libusb_control_transfer(h, 0xC0, 0xE4, addr, 0, buf, n, CTRL_TIMEOUT_MS)
    assert r >= 0, f"E4 read 0x{addr:04X} len {n} failed: {r}"
    return bytes(buf[:r])


def xdata_write(h, addr: int, val: int) -> None:
    r = libusb.libusb_control_transfer(h, 0x40, 0xE5, addr, val & 0xFF, None, 0, CTRL_TIMEOUT_MS)
    assert r >= 0, f"E5 write 0x{addr:04X}=0x{val:02X} failed: {r}"


EP_OUT = 0x02


def bulk_out(h, data: bytes) -> None:
    """Send `data` to bulk OUT EP 0x02 — lands at XDATA 0x7000 (the SPI page
    buffer). The bootstub exposes the same raw bulk OUT path that handmade fw
    uses, so this matches handmade/e4_flash.py byte-for-byte."""
    xdata_write(h, 0x9094, 0x10)               # ARM_OUT (REG_USB_EP_CFG2)
    buf = (ctypes.c_ubyte * len(data))(*data)
    transferred = ctypes.c_int()
    r = libusb.libusb_bulk_transfer(h, EP_OUT, buf, len(data),
                                    ctypes.byref(transferred), 5000)
    assert r == 0, f"bulk OUT failed: {r}"
    assert transferred.value == len(data), (
        f"short bulk OUT: {transferred.value}/{len(data)}")


def xdata_write_block(h, addr: int, data: bytes) -> None:
    """Load `data` at XDATA `addr`. For the SPI page buffer (0x7000) we use
    bulk OUT; everything else falls back to byte-at-a-time E5 writes."""
    if addr == FLASH_BUF:
        bulk_out(h, data)
    else:
        for i, b in enumerate(data):
            xdata_write(h, addr + i, b)


def vendor_jump(h, target: int) -> None:
    libusb.libusb_control_transfer(h, 0x40, 0xE7, target, 0, None, 0, CTRL_TIMEOUT_MS)


def request_enter_dfu(h) -> None:
    """Userfw 0xEC: writes cookie + CC31. Errors are expected (device drops)."""
    libusb.libusb_control_transfer(h, 0x40, 0xEC, 0, 0, None, 0, CTRL_TIMEOUT_MS)


# --- SPI flash controller -- driven entirely from host via XDATA pokes ----

FLASH_BUF        = 0x7000
REG_FLASH_MODE   = 0xC8AD
REG_BUF_OFF_HI   = 0xC8AE
REG_BUF_OFF_LO   = 0xC8AF
REG_FLASH_CMD    = 0xC8AA
REG_ADDR_LEN     = 0xC8AC
REG_ADDR_LO      = 0xC8A1
REG_ADDR_MD      = 0xC8A2
REG_ADDR_HI      = 0xC8AB
REG_DATALEN_HI   = 0xC8A3
REG_DATALEN_LO   = 0xC8A4
REG_FLASH_CSR    = 0xC8A9
REG_CC33         = 0xCC33
REG_CA81         = 0xCA81
REG_C805         = 0xC805
REG_FLASH_DIV    = 0xC8A6


def flash_poll_busy(h, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not (xdata_read(h, REG_FLASH_CSR, 1)[0] & 0x01): return
    raise TimeoutError("flash CSR busy timeout")


def flash_transaction(h, cmd: int, addr: int = 0, data_len: int = 0,
                      addr_len: int = 0x07, mode: int = 0x00) -> None:
    xdata_write(h, REG_FLASH_MODE, mode)
    xdata_write(h, REG_BUF_OFF_HI, 0)
    xdata_write(h, REG_BUF_OFF_LO, 0)
    xdata_write(h, REG_FLASH_CMD, cmd)
    xdata_write(h, REG_ADDR_LEN, addr_len)
    xdata_write(h, REG_ADDR_LO, addr & 0xFF)
    xdata_write(h, REG_ADDR_MD, (addr >> 8) & 0xFF)
    xdata_write(h, REG_ADDR_HI, (addr >> 16) & 0xFF)
    xdata_write(h, REG_DATALEN_HI, (data_len >> 8) & 0xFF)
    xdata_write(h, REG_DATALEN_LO, data_len & 0xFF)
    xdata_write(h, REG_FLASH_CSR, 0x01)
    flash_poll_busy(h)
    for _ in range(4): xdata_write(h, REG_FLASH_MODE, 0x00)


def flash_wren(h) -> None:
    xdata_write(h, REG_FLASH_MODE, 0x00)
    xdata_write(h, REG_FLASH_CMD, 0x06)
    xdata_write(h, REG_ADDR_LEN, 0x04)
    xdata_write(h, REG_DATALEN_HI, 0x00)
    xdata_write(h, REG_DATALEN_LO, 0x00)
    xdata_write(h, REG_FLASH_CSR, 0x01)
    flash_poll_busy(h)


def flash_read_status(h) -> int:
    flash_transaction(h, cmd=0x05, data_len=1, addr_len=0x04, mode=0x00)
    return xdata_read(h, FLASH_BUF, 1)[0]


def flash_wait_wip(h, timeout=10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not (flash_read_status(h) & 0x01): return
        time.sleep(0.005)
    raise TimeoutError("flash WIP timeout")


def flash_init(h) -> None:
    """Mirror handmade/e4_flash.py:flash_init — clears block-protect bits."""
    xdata_write(h, REG_CC33, 0x04)
    v = xdata_read(h, REG_CA81, 1)[0]
    xdata_write(h, REG_CA81, v | 0x01)
    xdata_write(h, REG_C805, 0x02)
    xdata_write(h, REG_FLASH_DIV, 0x04)

    sr = 0
    for _ in range(5):
        flash_wren(h)
        xdata_write_block(h, FLASH_BUF, b"\x00\x00\x00\x00")
        flash_transaction(h, cmd=0x01, data_len=1, addr_len=0x04, mode=0x01)
        time.sleep(0.01)
        sr = flash_read_status(h)
        if not (sr & 0x1C): return
    raise RuntimeError(f"failed to clear block protect, SR=0x{sr:02X}")


def flash_jedec_id(h) -> bytes:
    flash_transaction(h, cmd=0x9F, data_len=3, addr_len=0x04, mode=0x00)
    return xdata_read(h, FLASH_BUF, 3)


def flash_read(h, addr: int, size: int) -> bytes:
    out = bytearray()
    while len(out) < size:
        want = min(4096, size - len(out))
        dma_len = max(4096, want)
        flash_transaction(h, cmd=0x03, addr=addr + len(out), data_len=dma_len,
                          addr_len=0x07, mode=0x00)
        read = 0
        while read < want:
            n = min(64, want - read)
            out.extend(xdata_read(h, FLASH_BUF + read, n))
            read += n
    return bytes(out)


def flash_block_erase_64k(h, addr: int) -> None:
    assert (addr & 0xFFFF) == 0
    flash_wren(h)
    flash_transaction(h, cmd=0xD8, addr=addr, data_len=0, addr_len=0x07, mode=0x00)
    flash_wait_wip(h)


def flash_page_program(h, addr: int, size: int) -> None:
    flash_wren(h)
    flash_transaction(h, cmd=0x02, addr=addr, data_len=size, addr_len=0x07, mode=0x01)
    flash_wait_wip(h)


# --- high-level operations -------------------------------------------------

USERFW_FLASH_OFFSET = 0x4000
PAGE = 128


def _program_pages(h, base: int, data: bytes) -> None:
    """Page-program `data` at flash `base`. Skips all-0xFF pages (already erased)."""
    off = 0
    while off < len(data):
        chunk = data[off:off + PAGE]
        if not all(b == 0xFF for b in chunk):
            bulk_out(h, chunk)
            flash_page_program(h, base + off, len(chunk))
        off += len(chunk)


def flash_image(h, image: bytes, addr: int = USERFW_FLASH_OFFSET) -> None:
    """Erase the 64KB blocks covering [addr, addr+len), then page-program.

    If `addr` is past the start of its containing 64KB block, the leading bytes
    of that block are read out before the erase and rewritten after — needed
    when the userfw image lives in the same block as the bootstub itself."""
    end = addr + len(image)
    erase_start = addr & ~0xFFFF
    erase_end   = (end + 0xFFFF) & ~0xFFFF
    blocks = list(range(erase_start, erase_end, 0x10000))

    preserved = b""
    if addr > erase_start:
        n = addr - erase_start
        print(f"  preserving 0x{erase_start:05X}-0x{addr-1:05X} ({n} bytes) across block erase")
        preserved = flash_read(h, erase_start, n)

    print(f"  erasing {len(blocks)} block(s) covering 0x{erase_start:05X}-0x{erase_end-1:05X}")
    for b in blocks:
        flash_block_erase_64k(h, b)

    if preserved:
        _program_pages(h, erase_start, preserved)

    total = len(image)
    written = 0
    t0 = time.monotonic()
    while written < total:
        chunk = image[written:written + PAGE]
        bulk_out(h, chunk)
        flash_page_program(h, addr + written, len(chunk))
        written += len(chunk)
        pct = written * 100 // total
        elapsed = time.monotonic() - t0
        rate = written / elapsed if elapsed else 0
        print(f"\r  writing: {written}/{total} ({pct}%) {rate:.0f} B/s",
              end="", flush=True)
    print()


def verify_image(h, image: bytes, addr: int = USERFW_FLASH_OFFSET) -> int:
    total = len(image)
    got = bytearray()
    t0 = time.monotonic()
    while len(got) < total:
        n = min(4096, total - len(got))
        got.extend(flash_read(h, addr + len(got), n))
        pct = len(got) * 100 // total
        print(f"\r  verifying: {len(got)}/{total} ({pct}%)", end="", flush=True)
    print()
    print(f"  verify took {time.monotonic() - t0:.1f}s")
    errors = sum(1 for a, b in zip(got, image) if a != b)
    if errors:
        for i, (a, b) in enumerate(zip(got, image)):
            if a != b and errors > 0:
                print(f"    mismatch at 0x{addr+i:05X}: flash=0x{a:02X} expected=0x{b:02X}")
                errors -= 1
                if errors == 0: break
    return sum(1 for a, b in zip(got, image) if a != b)


def reboot_to_userfw(h) -> None:
    """Trigger a CC31 CPU reset. Bootstub starts fresh, validates the userfw
    image we just flashed, and LJMPs into it. CC31 is enough — the USB PHY
    stays connected through the reset so the host sees a re-enumeration as
    the new VID/PID once userfw's usb_init runs. (LJMP 0x0000 alone leaves
    the controller bound to whatever descriptors the bootstub last
    served.)"""
    print("  rebooting via CC31 (CPU reset)")
    try:
        xdata_write(h, 0xCC31, 0x01)
    except Exception:
        pass


# --- entry point -----------------------------------------------------------

def cmd_flash(args):
    with open(args.image, "rb") as f:
        image = f.read()
    if image[:4] != b"ASP2":
        sys.exit(f"error: {args.image} doesn't look like a packed userfw image (magic != 'ASP2')")
    print(f"image: {args.image} ({len(image)} bytes)")

    state, h, ctx = find_device()
    if state is None:
        sys.exit("error: no ASM2464PD device found (looked for ADD1:0001 and ADD1:B007)")

    try:
        if state == "app":
            print("device in app mode — sending 0xEC to enter DFU")
            request_enter_dfu(h)
            _close(h, ctx)
            h, ctx = wait_for_dfu()
            print("bootstub up.")

        flash_init(h)
        jedec = flash_jedec_id(h)
        print(f"  JEDEC ID: {jedec.hex()}")

        flash_image(h, image)

        errors = verify_image(h, image)
        if errors:
            sys.exit(f"FAIL: {errors} byte mismatches")
        print("verify OK")

        if not args.no_reboot:
            reboot_to_userfw(h)
    finally:
        _close(h, ctx)


def cmd_enter_dfu(args):
    state, h, ctx = find_device()
    if state == "dfu":
        print("already in DFU mode")
        _close(h, ctx); return
    if state is None:
        sys.exit("error: no device found")
    try:
        request_enter_dfu(h)
    finally:
        _close(h, ctx)
    h, ctx = wait_for_dfu()
    print("bootstub up.")
    _close(h, ctx)


def cmd_read(args):
    state, h, ctx = find_device()
    if state != "dfu":
        sys.exit("error: must be in DFU mode (run --enter-dfu first)")
    try:
        flash_init(h)
        data = flash_read(h, args.addr, args.size)
    finally:
        _close(h, ctx)
    for off in range(0, len(data), 16):
        line = data[off:off + 16]
        hexs = " ".join(f"{b:02X}" for b in line)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in line)
        print(f"  {args.addr + off:05X}: {hexs:<48s} {ascii_}")


def cmd_jedec(args):
    state, h, ctx = find_device()
    if state != "dfu":
        sys.exit("error: must be in DFU mode")
    try:
        flash_init(h)
        jedec = flash_jedec_id(h)
        print(f"JEDEC ID: {jedec.hex()} (mfr=0x{jedec[0]:02X} type=0x{jedec[1]:02X} cap=0x{jedec[2]:02X})")
    finally:
        _close(h, ctx)


def cmd_reboot(args):
    state, h, ctx = find_device()
    if state is None:
        sys.exit("error: no device found")
    try:
        reboot_to_userfw(h)
    finally:
        _close(h, ctx)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")

    pf = sub.add_parser("flash", help="flash userfw image (default if image given)")
    pf.add_argument("image")
    pf.add_argument("--no-reboot", action="store_true", help="skip the post-flash reboot")
    pf.set_defaults(func=cmd_flash)

    pd = sub.add_parser("enter-dfu", help="ask running userfw to drop to bootstub DFU")
    pd.set_defaults(func=cmd_enter_dfu)

    pr = sub.add_parser("read", help="hex-dump SPI flash")
    pr.add_argument("addr", type=lambda s: int(s, 0))
    pr.add_argument("size", type=lambda s: int(s, 0), nargs="?", default=256)
    pr.set_defaults(func=cmd_read)

    pj = sub.add_parser("jedec", help="print JEDEC ID")
    pj.set_defaults(func=cmd_jedec)

    pb = sub.add_parser("reboot", help="reboot via 0xE7 → 0x0000")
    pb.set_defaults(func=cmd_reboot)

    # Positional shortcut: dfu.py image.bin → flash image.bin.
    argv = sys.argv[1:]
    if argv and not argv[0].startswith("-") and argv[0] not in {"flash", "enter-dfu", "read", "jedec", "reboot"}:
        argv = ["flash", *argv]

    # Convenience flags that mirror subcommands.
    if argv and argv[0] == "--enter-dfu":  argv = ["enter-dfu", *argv[1:]]
    if argv and argv[0] == "--reboot":     argv = ["reboot", *argv[1:]]
    if argv and argv[0] == "--read":       argv = ["read", *argv[1:]]
    if argv and argv[0] == "--jedec":      argv = ["jedec", *argv[1:]]

    args = p.parse_args(argv)
    if not args.cmd:
        p.print_help()
        return 1
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
