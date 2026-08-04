#!/usr/bin/env python3
"""
PCIe link-training bisection experiments via the UART MMIO proxy.

From a cold (reset) state, applies a configurable subset of the stock
bring-up sequence and watches LTSSM (B450), link status (E765) and the
bank1 link-info reg (0x4092) to see if/when the downstream link trains.

Usage:
  ./ftdi_debug.py -rn    # cold start first!
  python3 pcie/train_experiment.py --steps power,cpcie
  python3 pcie/train_experiment.py --steps power,cpcie --skip b431
  python3 pcie/train_experiment.py --list
"""
import sys
import time
import argparse

sys.path.insert(0, "emulate")
from uart_proxy import UARTProxy


def step_power(p):
    """3.3V + 12V enable only."""
    p.write(0xC656, p.read(0xC656) | 0x20)


def step_phaseA(p):
    """PERST assert + tunnel link state clear + tunnel cfg."""
    p.write(0xB480, p.read(0xB480) | 0x01)
    p.write(0xB430, p.read(0xB430) & 0xFE)
    p.write(0xB298, p.read(0xB298) | 0x10)


def step_phaseB(p):
    """Tunnel adapter + switch/PHY plane config."""
    p.write(0xB401, 0x01)
    p.write(0xB401, 0x00)
    p.write(0xCA06, 0x61)
    for a, v in [(0xB410, 0x1B), (0xB411, 0x21), (0xB420, 0x1B), (0xB421, 0x21),
                 (0xB412, 0x24), (0xB413, 0x63), (0xB422, 0x24), (0xB423, 0x63),
                 (0xB415, 0x06), (0xB416, 0x04), (0xB417, 0x00), (0xB425, 0x06),
                 (0xB426, 0x04), (0xB427, 0x00), (0xB41A, 0x1B), (0xB41B, 0x21),
                 (0xB42A, 0x1B), (0xB42B, 0x21), (0xB418, 0x24), (0xB419, 0x63),
                 (0xB428, 0x24), (0xB429, 0x63)]:
        p.write(a, v)
    p.write_dpx(0x4084, 0x22)
    p.write_dpx(0x5084, 0x22)
    p.write(0xB401, 0x01)
    p.write(0xB482, p.read(0xB482) | 0xF1)
    p.write(0xB401, 0x00)
    p.write(0xB480, p.read(0xB480) | 0x01)
    p.write(0xB430, p.read(0xB430) & 0xFE)
    p.write(0xB298, p.read(0xB298) | 0x10)
    p.write_dpx(0x6043, 0x70)
    p.write_dpx(0x6025, p.read_dpx(0x6025) | 0x80)
    p.write(0xCA06, 0x61)
    p.write(0xB480, p.read(0xB480) | 0x01)
    p.write(0xC659, p.read(0xC659) & 0xFE)
    p.write(0xB402, 0x01)
    p.write(0xB436, 0xEE)
    p.write(0xB436, 0xEE)


def step_cpcie(p, skip=(), e764_mode="seq"):
    """Phase C: training trigger block (stock @283440-284482)."""
    if "ca81" not in skip:
        p.write(0xCA81, p.read(0xCA81) & 0xFE)
    if "ca06" not in skip:
        p.write(0xCA06, 0x21)
    if "b403" not in skip:
        p.write(0xB403, 0x01)
    if "b431" not in skip:
        p.write(0xB431, 0x0E)
    if "e764" not in skip:
        if e764_mode == "seq":      # stock: |=08, &=~04, |=02, |=01, &=~02
            p.write(0xE764, p.read(0xE764) | 0x08)
            p.write(0xE764, p.read(0xE764) & 0xFB)
            p.write(0xE764, p.read(0xE764) | 0x02)
            p.write(0xE764, p.read(0xE764) | 0x01)
            p.write(0xE764, p.read(0xE764) & 0xFD)
        elif e764_mode == "1c":     # handmade current: single write 0x1C
            p.write(0xE764, 0x1C)
        elif e764_mode == "19":     # single write of final value
            p.write(0xE764, 0x19)
    if "c659" not in skip:
        p.write(0xC659, p.read(0xC659) | 0x01)


STEPS = {
    "power": step_power,
    "phaseA": step_phaseA,
    "phaseB": step_phaseB,
    "cpcie": step_cpcie,
}


def watch(p, secs):
    t0 = time.monotonic()
    last = None
    trained_at = None
    while time.monotonic() - t0 < secs:
        b450 = p.read(0xB450)
        e765 = p.read(0xE765)
        info = p.read_dpx(0x4092)
        st = (b450, e765, info)
        if st != last:
            print(f"  t+{time.monotonic()-t0:5.2f}s LTSSM=0x{b450:02X} E765=0x{e765:02X} info=0x{info:02X}")
            last = st
        if e765 & 0x02 and trained_at is None:
            trained_at = time.monotonic() - t0
        time.sleep(0.02)
    return trained_at


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", default="power,cpcie", help="comma list of: power,phaseA,phaseB,cpcie")
    ap.add_argument("--skip", default="", help="comma list inside cpcie: ca81,ca06,b403,b431,e764,c659")
    ap.add_argument("--e764", default="seq", choices=["seq", "1c", "19"])
    ap.add_argument("--watch", type=float, default=5.0)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if args.list:
        print("steps:", ", ".join(STEPS))
        print("skip targets (cpcie): ca81, ca06, b403, b431, e764, c659")
        return

    skip = set(s for s in args.skip.split(",") if s)
    with UARTProxy() as p:
        print(f"cold: LTSSM=0x{p.read(0xB450):02X} E765=0x{p.read(0xE765):02X} info=0x{p.read_dpx(0x4092):02X} E764=0x{p.read(0xE764):02X}")
        for name in args.steps.split(","):
            fn = STEPS[name]
            print(f"== {name} ==")
            if name == "cpcie":
                fn(p, skip=skip, e764_mode=args.e764)
            else:
                fn(p)
        trained_at = watch(p, args.watch)
        b455 = p.read(0xB455)
        print(f"RESULT: {'TRAINED in %.2fs' % trained_at if trained_at is not None else 'NOT TRAINED'} "
              f"(B455=0x{b455:02X}, LTSSM=0x{p.read(0xB450):02X})")


if __name__ == "__main__":
    main()
