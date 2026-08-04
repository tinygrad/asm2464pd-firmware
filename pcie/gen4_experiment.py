#!/usr/bin/env python3
"""
Gen4 x4 link experiment via the UART MMIO proxy.

Brings the downstream link up with no B431 width cap, then probes PCIe
config space (internal switch downstream port via CfgRd0, GPU via CfgRd1)
and tries to trigger the Gen1 -> Gen4 speed change.

Usage: python3 pcie/gen4_experiment.py [--watch 10]
"""
import sys
import time
import argparse

sys.path.insert(0, "emulate")
from uart_proxy import UARTProxy

CFGRD0, CFGWR0, CFGRD1, CFGWR1 = 0x04, 0x44, 0x05, 0x45


def cfg_xfer(p, fmt, addr, wdata=None):
    """Run one config TLP. addr is B218:B21B big-endian. Returns (data_bytes, cpl_status) or None."""
    p.write(0xB210, fmt)
    p.write(0xB213, 0x01)
    p.write(0xB217, 0x0F)
    for i in range(4):
        p.write(0xB218 + i, (addr >> (8 * (3 - i))) & 0xFF)
    if wdata is not None:
        for i in range(4):
            p.write(0xB220 + i, (wdata >> (8 * i)) & 0xFF)
    p.write(0xB216, 0x20)
    p.write(0xB296, 0x01)
    p.write(0xB296, 0x02)
    p.write(0xB296, 0x04)
    p.write(0xB254, 0x0F)
    for _ in range(2000):
        s = p.read(0xB296)
        if s & 0x02:
            p.write(0xB296, 0x02)
            data = [p.read(0xB220 + i) for i in range(4)]
            return data, p.read(0xB22A)
        if s & 0x01:
            p.write(0xB296, 0x01)
            return None
    return None


def cfg_read(p, fmt, addr):
    r = cfg_xfer(p, fmt, addr)
    if r is None:
        return None
    data, st = r
    return int.from_bytes(bytes(data), "little"), st


def cfg_write(p, fmt, addr, val):
    return cfg_xfer(p, fmt, addr, wdata=val) is not None


def power_on_nocap(p):
    p.write(0xB480, p.read(0xB480) | 0x01)
    p.write(0xB403, 0x01)
    p.write(0xB430, p.read(0xB430) & 0xFE)
    p.write_dpx(0x6025, p.read_dpx(0x6025) | 0x80)
    p.write(0xC656, p.read(0xC656) | 0x20)
    time.sleep(0.2)
    p.write(0xCA81, p.read(0xCA81) & 0xFE)
    p.write(0xCA06, 0x21)
    p.write(0xE764, 0x1C)
    p.write(0xC659, p.read(0xC659) | 0x01)
    p.write(0xB298, p.read(0xB298) | 0x10)
    p.write(0xB480, p.read(0xB480) & 0xFE)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3 and not (p.read(0xE765) & 0x02):
        time.sleep(0.01)
    up = bool(p.read(0xE765) & 0x02)
    print("link:", "UP" if up else "DOWN",
          "info=0x%02X LTSSM=0x%02X" % (p.read_dpx(0x4092), p.read(0xB450)))
    return up


def watch_info(p, secs):
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < secs:
        info = p.read_dpx(0x4092)
        ltssm = p.read(0xB450)
        if (info, ltssm) != last:
            print("  t+%.2fs info=0x%02X LTSSM=0x%02X" % (time.monotonic() - t0, info, ltssm))
            last = (info, ltssm)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=float, default=8.0)
    args = ap.parse_args()
    with UARTProxy() as p:
        if not power_on_nocap(p):
            return
        print("== watch for speed retrain ==")
        watch_info(p, args.watch)


if __name__ == "__main__":
    main()
