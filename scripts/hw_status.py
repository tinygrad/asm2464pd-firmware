#!/usr/bin/env python3
"""Poll the custom firmware's hw_status vendor control transfer.

  bRequest 0xC0 IN -> hw_status_t {
    uint16_t voltage_mv;  // INA231 bus voltage
    int16_t  current_ma;  // INA231 shunt current (signed)
  }
"""

import argparse
import struct
import sys
import time

import usb.core
import usb.util


SUPPORTED_CONTROLLERS = [
    (0xADD1, 0x0001),
]

HW_STATUS_REQ = 0xC0
HW_STATUS_LEN = 4


def find_device(vid, pid):
    if vid is not None and pid is not None:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            raise RuntimeError(f"Device {vid:04X}:{pid:04X} not found")
        return dev, vid, pid
    for cand_vid, cand_pid in SUPPORTED_CONTROLLERS:
        dev = usb.core.find(idVendor=cand_vid, idProduct=cand_pid)
        if dev is not None:
            return dev, cand_vid, cand_pid
    supported = ", ".join(f"{vid:04X}:{pid:04X}" for vid, pid in SUPPORTED_CONTROLLERS)
    raise RuntimeError(f"No ASM2464 device found (looked for {supported})")


def setup_device(dev):
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (NotImplementedError, usb.core.USBError):
        pass
    try:
        dev.set_configuration()
    except usb.core.USBError as exc:
        msg = str(exc).lower()
        if "busy" not in msg and "already" not in msg:
            raise


def read_hw_status(dev, timeout_ms=1000):
    data = bytes(dev.ctrl_transfer(0xC0, HW_STATUS_REQ, 0, 0, HW_STATUS_LEN, timeout=timeout_ms))
    if len(data) != HW_STATUS_LEN:
        raise RuntimeError(f"short hw_status read: expected {HW_STATUS_LEN}, got {len(data)}")
    voltage_mv, current_ma = struct.unpack("<Hh", data)
    return voltage_mv, current_ma


def fmt_timestamp(now=None):
    if now is None:
        now = time.time()
    whole = time.localtime(now)
    millis = int((now - int(now)) * 1000)
    return time.strftime("%H:%M:%S", whole) + f".{millis:03d}"


def main():
    parser = argparse.ArgumentParser(description="Poll ASM2464 hw_status (0xC0) over USB")
    parser.add_argument("--vid", type=lambda x: int(x, 0), help="USB vendor ID")
    parser.add_argument("--pid", type=lambda x: int(x, 0), help="USB product ID")
    parser.add_argument("--interval-ms", type=float, default=200.0, help="sample period in milliseconds")
    parser.add_argument("--count", type=int, help="number of samples to capture")
    args = parser.parse_args()

    if (args.vid is None) != (args.pid is None):
        parser.error("--vid and --pid must be supplied together")
    if args.interval_ms <= 0:
        parser.error("--interval-ms must be positive")

    dev, vid, pid = find_device(args.vid, args.pid)
    setup_device(dev)

    print(f"Device {vid:04X}:{pid:04X}", flush=True)
    print("time             V[V]    I[A]     P[W]", flush=True)
    print("------------  -------  ------  -------", flush=True)

    interval_s = args.interval_ms / 1000.0
    next_sample = time.monotonic()
    sample_count = 0
    try:
        while args.count is None or sample_count < args.count:
            now = time.monotonic()
            if now < next_sample:
                time.sleep(next_sample - now)

            wall_now = time.time()
            v_mv, i_ma = read_hw_status(dev)
            v = v_mv / 1000.0
            a = i_ma / 1000.0
            print(f"{fmt_timestamp(wall_now)}  {v:7.3f}  {a:+6.3f}  {v*a:+7.3f}", flush=True)

            sample_count += 1
            next_sample += interval_s
            if next_sample < time.monotonic():
                next_sample = time.monotonic()
    except KeyboardInterrupt:
        return 0
    finally:
        usb.util.dispose_resources(dev)
    return 0


if __name__ == "__main__":
    sys.exit(main())
