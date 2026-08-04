#!/usr/bin/env python3
"""
Gen4 experiment WITH the rxphy tuning applied, via the UART MMIO proxy.

Sequence: reset -> rxphy tuning (from handmade/src/pcie_tuning.h) ->
power on WITHOUT the B403 Gen3 cap -> watch for Gen4 training.

Usage: ./ftdi_debug.py -rn && python3 pcie/gen4_tuning_experiment.py
"""
import sys
import time

sys.path.insert(0, "emulate")
from uart_proxy import UARTProxy


def b1(p, addr, val):
    p.write_dpx(addr, val)


def preamble(p, lane, comp):
    b1(p, lane + 0x9B, 0x10)
    b1(p, lane + 0x9B, 0x90)
    b1(p, comp + 0x00, 0xC0)
    b1(p, comp + 0x04, 0x2B)
    b1(p, comp + 0x06, 0x41)
    b1(p, comp + 0x07, 0x81)
    b1(p, comp + 0x59, 0x41)
    b1(p, comp + 0x5A, 0x00)
    b1(p, comp + 0x0A, 0x11)
    b1(p, comp + 0x42, 0xBF)
    b1(p, comp + 0x05, 0x8A)


def stage0(p, lane):
    b1(p, lane + 0x64, 0x0F)
    b1(p, lane + 0xBF, 0xB0)
    b1(p, lane + 0xBF, 0x30)
    b1(p, lane + 0x67, 0xD0)


def stage1(p, lane):
    b1(p, lane + 0x40, 0x01)
    b1(p, lane + 0x01, 0x7C)
    b1(p, lane + 0x11, 0x7C)
    b1(p, lane + 0x21, 0xF0)
    b1(p, lane + 0x31, 0xF0)


def stage2(p, lane):
    b1(p, lane + 0x34, 0x07)
    b1(p, lane + 0x35, 0x6D)
    b1(p, lane + 0x26, 0x3A)
    b1(p, lane + 0x36, 0x57)
    b1(p, lane + 0x06, 0x73)
    b1(p, lane + 0x16, 0x73)
    b1(p, lane + 0x26, 0x7A)
    b1(p, lane + 0x36, 0x77)
    b1(p, lane + 0x37, 0x6F)
    b1(p, lane + 0x46, 0x74)
    b1(p, lane + 0x46, 0x64)
    b1(p, lane + 0x46, 0x44)
    b1(p, lane + 0x46, 0x04)
    b1(p, lane + 0x1D, 0x40)
    b1(p, lane + 0x2D, 0x40)
    b1(p, lane + 0x3D, 0x4A)
    b1(p, lane + 0x1A, 0x44)
    b1(p, lane + 0x3A, 0x4D)
    b1(p, lane + 0x1B, 0x4A)
    b1(p, lane + 0x3B, 0x4F)
    b1(p, lane + 0x0C, 0x17)
    b1(p, lane + 0x1C, 0x17)
    b1(p, lane + 0x02, 0xE8)
    b1(p, lane + 0x22, 0x30)
    b1(p, lane + 0x32, 0x30)
    b1(p, lane + 0x6C, 0x0C)
    b1(p, lane + 0x6C, 0x6C)


def comp_profile(p, comp):
    b1(p, comp + 0x20, 0x60)
    b1(p, comp + 0x20, 0x10)
    b1(p, comp + 0x21, 0xF2)
    b1(p, comp + 0x21, 0x42)
    b1(p, comp + 0x22, 0xC6)
    b1(p, comp + 0x22, 0x26)
    b1(p, comp + 0x23, 0xED)
    b1(p, comp + 0x23, 0xA9)
    b1(p, comp + 0x24, 0xEC)
    b1(p, comp + 0x24, 0xCC)
    b1(p, comp + 0x25, 0x7D)
    b1(p, comp + 0x25, 0x41)
    b1(p, comp + 0x26, 0xDF)
    b1(p, comp + 0x27, 0xDA)
    b1(p, comp + 0x27, 0xDE)
    b1(p, comp + 0x28, 0x20)
    b1(p, comp + 0x28, 0x00)
    b1(p, comp + 0x29, 0x61)
    b1(p, comp + 0x29, 0x31)
    b1(p, comp + 0x2A, 0xA4)
    b1(p, comp + 0x2A, 0x74)
    b1(p, comp + 0x2B, 0xE9)
    b1(p, comp + 0x2B, 0xC9)


def tail(p, lane, comp, t0b):
    b1(p, lane + 0x87, 0x10)
    b1(p, lane + 0x88, 0x08)
    b1(p, lane + 0x07, 0xEF)
    b1(p, lane + 0x17, 0xEF)
    b1(p, lane + 0x37, 0x4F)
    b1(p, lane + 0x12, 0xA8)
    b1(p, lane + 0x2C, 0x17)
    b1(p, comp + 0x26, 0xD1)
    b1(p, lane + 0x0B, t0b)
    b1(p, lane + 0x2A, 0x46)
    b1(p, lane + 0x0D, 0x5A)
    b1(p, lane + 0x1D, 0x50)
    b1(p, lane + 0x2D, 0x50)
    b1(p, lane + 0x3D, 0x5A)


def apply_tuning(p):
    lanes = [0x7800, 0x7900, 0x7A00, 0x7B00]
    comps = [0x6000, 0x6400, 0x6800, 0x6C00]
    t0bs = [0x56, 0x56, 0x5C, 0x5C]
    for l, c in zip(lanes, comps):
        preamble(p, l, c)
    for l in lanes:
        stage0(p, l)
    for l in lanes:
        stage1(p, l)
    for l, c in zip(lanes, comps):
        stage2(p, l)
        comp_profile(p, c)
        b1(p, l + 0x85, 0xA6)
    for l, c, t in zip(lanes, comps, t0bs):
        tail(p, l, c, t)


def rd32(p, a):
    d = [p.read_dpx(a + i) for i in range(4)]
    return d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)


def main():
    with UARTProxy() as p:
        v = rd32(p, 0x408C)
        print("cold: LNKCAP max Gen%d, B403=0x%02X" % (v & 0xF, p.read(0xB403)))
        print("applying rxphy tuning (~200 bank1 writes)...")
        apply_tuning(p)
        # power on WITHOUT B403 (the Gen3 cap)
        p.write(0xB480, p.read(0xB480) | 0x01)
        p.write(0xB430, p.read(0xB430) & 0xFE)
        p.write_dpx(0x6025, p.read_dpx(0x6025) | 0x80)
        p.write(0xC656, p.read(0xC656) | 0x20)
        time.sleep(0.2)
        p.write(0xCA81, p.read(0xCA81) & 0xFE)
        p.write(0xCA06, 0x21)
        v = rd32(p, 0x408C)
        print("pre-train: LNKCAP max Gen%d" % (v & 0xF))
        p.write(0xE764, 0x1C)
        p.write(0xC659, p.read(0xC659) | 0x01)
        best = 0
        for i in range(32):
            time.sleep(0.25)
            info = p.read_dpx(0x4092)
            lt = p.read(0xB450)
            e765 = p.read(0xE765)
            best = max(best, info & 0xF)
            print("t+%.2fs 0x4092=0x%02X (Gen%d x%d) LTSSM=0x%02X E765=0x%02X" %
                  (0.25 * (i + 1), info, info & 0xF, (info >> 4) & 0xF, lt, e765))
            if (e765 & 0x02) and (info & 0xF) == 4:
                print("*** Gen4 x%d achieved! ***" % ((info >> 4) & 0xF))
                break
        print("best speed seen: Gen%d" % best)


if __name__ == "__main__":
    main()
