#!/usr/bin/env python3
"""
Push the downstream PCIe link to Gen4 x4 via the UART MMIO proxy.

Flow (mirrors tinygrad's enumeration):
  1. power on with no B431 width cap (trains x4 at Gen1, retrains to Gen3)
  2. configure bridge bus numbers (bus0 -> 1, bus1 -> 2 = GPU)
  3. read bus-1 downstream port + GPU PCIe link caps
  4. optionally: write downstream port LNKCTL2 target speed = Gen4 and
     set LNKCTL retrain, watch bank1 0x4092 for the speed change

Config data byte order (verified): completion data registers B220-B223 hold
config bytes in REVERSE order, so value = int.from_bytes(d, 'big') with LE
field extraction. Writes take value big-endian into B220-B223.
"""
import sys
import time
import argparse

sys.path.insert(0, "emulate")
sys.path.insert(0, "pcie")
from uart_proxy import UARTProxy
from gen4_experiment import power_on_nocap

LNKCAP_OFF, LNKCTL_OFF, LNKSTA_OFF, LNKCTL2_OFF = 0x0C, 0x10, 0x12, 0x30


def cfg_xfer(p, bus, dev, fn, off, val=None, size=4):
    fmt = (0x44 if val is not None else 0x04) | (1 if bus > 0 else 0)
    addr = (bus << 24) | (dev << 19) | (fn << 16) | (off & 0xFFC)
    p.write(0xB210, fmt)
    p.write(0xB213, 0x01)
    p.write(0xB217, ((1 << size) - 1) << (off & 3))
    for i in range(4):
        p.write(0xB218 + i, (addr >> (8 * (3 - i))) & 0xFF)
    if val is not None:
        for i in range(4):
            p.write(0xB220 + i, (val >> (8 * (3 - i))) & 0xFF)
    p.write(0xB216, 0x20)
    p.write(0xB296, 0x01)
    p.write(0xB296, 0x02)
    p.write(0xB296, 0x04)
    p.write(0xB254, 0x0F)
    for _ in range(2000):
        s = p.read(0xB296)
        if s & 0x02:
            p.write(0xB296, 0x02)
            d = [p.read(0xB220 + i) for i in range(4)]
            return int.from_bytes(bytes(d), "big") >> (8 * (off & 3))
        if s & 0x01:
            p.write(0xB296, 0x01)
            return None
    return None


def cfg_read(p, bus, dev, fn, off, size=4, retries=8):
    for _ in range(retries):
        r = cfg_xfer(p, bus, dev, fn, off, size=size)
        if r is not None:
            return r
        time.sleep(0.002)
    return None


def cfg_write(p, bus, dev, fn, off, val, size=4):
    return cfg_xfer(p, bus, dev, fn, off, val=val, size=size)


def pcie_cap(p, bus, dev, fn):
    capp = cfg_read(p, bus, dev, fn, 0x34)
    if capp is None:
        return None
    ptr = capp & 0xFF
    for _ in range(16):
        if not ptr:
            return None
        hdr = cfg_read(p, bus, dev, fn, ptr)
        if hdr is None:
            return None
        if (hdr & 0xFF) == 0x10:
            return ptr
        ptr = (hdr >> 8) & 0xFF
    return None


def dump_link(p, bus, dev, fn, name):
    v = cfg_read(p, bus, dev, fn, 0)
    if v is None:
        print("%s: config read failed" % name)
        return None
    print("%s: vendor=0x%04X device=0x%04X" % (name, v & 0xFFFF, v >> 16))
    cap = pcie_cap(p, bus, dev, fn)
    if cap is None:
        print("  no PCIe cap found")
        return None
    lnkcap = cfg_read(p, bus, dev, fn, cap + LNKCAP_OFF)
    lnk = cfg_read(p, bus, dev, fn, cap + LNKCTL_OFF)  # LNKCTL(lo16) + LNKSTA(hi16)
    lnkctl2 = cfg_read(p, bus, dev, fn, cap + LNKCTL2_OFF)
    print("  PCIe cap @0x%02X: LNKCAP=0x%08X (max Gen%d x%d)" %
          (cap, lnkcap, lnkcap & 0xF, (lnkcap >> 4) & 0x3F))
    print("  LNKCTL=0x%04X LNKSTA=0x%04X (cur Gen%d x%d)" %
          (lnk & 0xFFFF, lnk >> 16, (lnk >> 16) & 0xF, (lnk >> 20) & 0x3F))
    print("  LNKCTL2=0x%08X (target Gen%d)" % (lnkctl2, lnkctl2 & 0xF))
    return cap


def setup_buses(p, gpu_bus=2):
    for bus in range(gpu_bus):
        buses = (0 << 0) | ((bus + 1) << 8) | (gpu_bus << 16)
        cfg_write(p, bus, 0, 0, 0x18, buses, size=4)


def watch_info(p, secs):
    t0 = time.monotonic()
    last = None
    best = 0
    while time.monotonic() - t0 < secs:
        info = p.read_dpx(0x4092)
        ltssm = p.read(0xB450)
        if (info, ltssm) != last:
            print("  t+%.2fs info=0x%02X (Gen%d x%d) LTSSM=0x%02X" %
                  (time.monotonic() - t0, info, info & 0xF, (info >> 4) & 0xF, ltssm))
            last = (info, ltssm)
        best = max(best, info & 0xF)
        time.sleep(0.02)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--retrain", action="store_true", help="write LNKCTL2=Gen4 + retrain link")
    ap.add_argument("--watch", type=float, default=6.0)
    args = ap.parse_args()

    with UARTProxy() as p:
        if not power_on_nocap(p):
            sys.exit(1)
        time.sleep(0.5)
        setup_buses(p, 2)
        cap1 = dump_link(p, 1, 0, 0, "downstream port (bus 1)")
        dump_link(p, 2, 0, 0, "GPU (bus 2)")
        if args.retrain and cap1 is not None:
            lnkctl2 = cfg_read(p, 1, 0, 0, cap1 + LNKCTL2_OFF)
            print("== LNKCTL2: 0x%08X -> target Gen4 ==" % lnkctl2)
            cfg_write(p, 1, 0, 0, cap1 + LNKCTL2_OFF, (lnkctl2 & ~0xF) | 0x4, size=2)
            lnk = cfg_read(p, 1, 0, 0, cap1 + LNKCTL_OFF)
            print("== set LNKCTL retrain bit (was 0x%04X) ==" % (lnk & 0xFFFF))
            cfg_write(p, 1, 0, 0, cap1 + LNKCTL_OFF, (lnk & 0xFFFF) | 0x20, size=2)
        print("== watch ==")
        watch_info(p, args.watch)


if __name__ == "__main__":
    main()
