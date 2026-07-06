#!/usr/bin/env python3
"""Flash firmware to ASM2464PD devices via bootstub DFU over USB.

Usage:
  python3 test_bootstub.py [--bootstub bootstub.bin] [--image firmware_image.bin] [--bus N]

Flow:
  1. Find ADD1:0001 device on the specified bus
  2. Send vendor request 0xF3 wValue=0xF4 to enter DFU mode
  3. Wait for B007:0001 (bootstub) to enumerate
  4. Erase userfw flash region
  5. Write firmware image in 64-byte chunks
  6. Send vendor request 0xEB to reset into new firmware
  7. Wait for ADD1:0001 to re-enumerate
  8. Read sideband 0xC0 to verify
"""
import argparse
import struct
import sys
import time
import usb.core
import usb.util

APP_VID = 0xADD1
APP_PID = 0x0001
BS_VID = 0xADD1
BS_PID = 0xB007

USERFW_FLASH_OFFSET = 0x4000
USERFW_HEADER_SIZE = 0x40
SECTOR_SIZE = 0x1000
USERFW_FLASH_END = USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + 0xDC10
USERFW_ERASE_END = ((USERFW_FLASH_END + (SECTOR_SIZE - 1)) & ~(SECTOR_SIZE - 1))


def find_device(vid, pid, bus=None, timeout=10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is not None:
            if bus is None or dev.bus == bus:
                return dev
        time.sleep(0.2)
    return None


def enter_dfu(dev):
    print("[dfu] entering DFU mode...")
    try:
        dev.ctrl_transfer(0x40, 0xF3, 0xF4, 0, None, timeout=1000)
    except Exception:
        pass  # device resets, transfer may fail
    usb.util.dispose_resources(dev)


def dfu_erase(dev, addr, length):
    data = struct.pack("<II", addr, length)
    dev.ctrl_transfer(0x40, 0xB0, 0, 0, data, timeout=5000)
    # wait for status IN
    dev.ctrl_transfer(0xC0, 0xB3, 0, 0, 0, timeout=5000)


def dfu_set_addr(dev, addr):
    dev.ctrl_transfer(0x40, 0xB1, addr & 0xFFFF, (addr >> 16) & 0xFF, None, timeout=1000)


def dfu_write(dev, data):
    dev.ctrl_transfer(0x40, 0xB2, len(data) & 0xFF, (len(data) >> 8) & 0xFF, data, timeout=5000)


def dfu_reset(dev):
    try:
        dev.ctrl_transfer(0x40, 0xEB, 0, 0, None, timeout=1000)
    except Exception:
        pass
    usb.util.dispose_resources(dev)


def flash_image(dev, image):
    # Erase
    erase_len = USERFW_ERASE_END - USERFW_FLASH_OFFSET
    print(f"[dfu] erasing 0x{USERFW_FLASH_OFFSET:x}-0x{USERFW_ERASE_END:x} ({erase_len} bytes)...")
    dfu_erase(dev, USERFW_FLASH_OFFSET, erase_len)
    print("[dfu] erase done")

    # Write
    addr = USERFW_FLASH_OFFSET
    total = len(image)
    written = 0
    while written < total:
        # Align to page boundary
        page_remaining = 0x100 - (addr & 0xFF)
        chunk_len = min(page_remaining, total - written, 64)
        chunk = image[written:written + chunk_len]

        dfu_set_addr(dev, addr)
        dfu_write(dev, chunk)

        addr += chunk_len
        written += chunk_len
        if written % 1024 == 0 or written == total:
            print(f"\r[dfu] writing: {written}/{total} bytes", end="", flush=True)
    print()

    # Reset into new firmware
    print("[dfu] resetting...")
    dfu_reset(dev)


def read_sideband(dev):
    try:
        data = dev.ctrl_transfer(0xC0, 0xC0, 0, 0, 4, timeout=2000)
        voltage = struct.unpack("<H", data[0:2])[0]
        current = struct.unpack("<h", data[2:4])[0]
        return voltage, current
    except Exception:
        return None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", required=True, help="firmware_image.bin to flash")
    ap.add_argument("--bus", type=int, default=None, help="USB bus number")
    ap.add_argument("--bootstub", default=None, help="bootstub.bin to flash first (optional)")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    print(f"[dfu] image: {len(image)} bytes")

    # Find app device
    print(f"[dfu] looking for app device (ADD1:0001){f' on bus {args.bus}' if args.bus else ''}...")
    dev = find_device(APP_VID, APP_PID, bus=args.bus, timeout=5)
    if dev is None:
        print("[dfu] app device not found, trying bootstub...")
        dev = find_device(BS_VID, BS_PID, bus=args.bus, timeout=2)
        if dev is None:
            print("[dfu] no device found")
            return 1
    else:
        # Enter DFU mode
        enter_dfu(dev)
        print("[dfu] waiting for bootstub...")
        dev = find_device(BS_VID, BS_PID, bus=args.bus, timeout=5)
        if dev is None:
            print("[dfu] bootstub did not enumerate")
            return 1

    print(f"[dfu] bootstub found on bus {dev.bus}")

    # Flash bootstub if requested
    if args.bootstub:
        with open(args.bootstub, "rb") as f:
            bs_image = f.read()
        print(f"[dfu] flashing bootstub ({len(bs_image)} bytes)...")
        # Bootstub goes to flash 0x0100 (via the mask ROM bootloader format)
        # For now, skip bootstub flashing via DFU — it needs to be flashed via FTDI
        print("[dfu] bootstub flashing via DFU not supported yet, use FTDI")

    # Flash userfw image
    flash_image(dev, image)

    # Wait for app to re-enumerate
    print("[dfu] waiting for app to re-enumerate...")
    dev = find_device(APP_VID, APP_PID, bus=args.bus, timeout=10)
    if dev is None:
        print("[dfu] app did not re-enumerate")
        return 1

    print(f"[dfu] app found on bus {dev.bus}")

    # Verify sideband
    v, i = read_sideband(dev)
    if v is not None:
        print(f"[dfu] sideband: {v}mV {i}mA")
    else:
        print("[dfu] sideband read failed")

    print("[dfu] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
