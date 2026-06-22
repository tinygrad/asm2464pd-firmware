#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin LEAN E302 link-mode TIMELINE tracer (a066-entry, change-gated on E302).

Purpose: answer the one open question of the mode-1-persistence wall — does STOCK's
E302 HOLD mode-1 (0x97) through the connect window all the way to the bond commit (as
the handmade memory claims), or does it ALSO decay to mode-0 (0x83) during connect and
RE-ENTER mode-1 only at the bond? Handmade (in-tree fine-grained capture) holds 0x97
through every device connect code path, then E302 decays PASSIVELY to 0x83 in the idle
window after the dt=01 route reply, before the bond — no device write coincides. We need
stock's E302 timeline at the SAME a066 connect events to know if it's host-driven there too.

Hook: a066 INT1 ENTRY (the low-frequency connect/bond event handler — the SAME proven
cave mechanism as app/patch_stock_sfr.py, which enumerates the GPU with the hook live).
Change-gated on E302 itself so it emits ONE lean line per DISTINCT E302 value across the
whole connect->bond sequence (so the UART burst is tiny and non-intrusive).

Line: \r\n[e3 <ctr16> e302=<E302> 66=<SB66> A0=<SBA0> A1=<SBA1> 1407=<P1[0x1407]>]

Build:  python3 app/patch_stock_e302seq.py fw_tinygrad.bin /tmp/fw_e302.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_e302.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001
BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# a066 entry: `e4 f5 4d` = CLR A ; MOV 0x4d,A
H_A66_OFF = bank1_off(0xA066)
H_A66_OLD = bytes.fromhex("e4f54d")

CAVE = 0x6600
STR_E3 = 0x7000            # "\r\n[e3 "
LBL_E3 = 0x7010            # " e302="
LBL_66 = 0x7018            # " 66="
LBL_A0 = 0x7020            # " A0="
LBL_A1 = 0x7028            # " A1="
LBL_14 = 0x7030            # " 1407="

CTR_LO = 0x8830
CTR_HI = 0x8831
E3_LAST = 0x0B56           # last-logged E302 (validated free/persistent headroom cell)
E3_SEEN = 0x0B57

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    # DPX assumed 0. movx a,@dptr ; mov r7,a ; lcall puthex
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_dpx1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def emit_counter():
    code = bytearray()
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # read E302 (plain XDATA, DPX=0) -> B
    code += mov_dptr(0xE302) + b"\xe0"             # movx a,@dptr (E302)
    code += b"\xf5\xf0"                            # mov B,a

    # change gate: if SEEN==0 -> emit; else if (LAST ^ B)==0 -> skip.
    code += mov_dptr(E3_SEEN) + b"\xe0"            # A = SEEN
    jz_emit = len(code)
    code += b"\x60\x00"                            # jz emit
    code += mov_dptr(E3_LAST) + b"\xe0"            # A = LAST
    code += b"\x65\xf0"                            # xrl A,B
    jnz_emit = len(code)
    code += b"\x70\x00"                            # jnz emit
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (unchanged)

    emit = len(code)
    code[jz_emit + 1] = (emit - (jz_emit + 2)) & 0xFF
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    code += mov_dptr(E3_LAST) + b"\xe5\xf0\xf0"    # LAST = B (E302)
    code += mov_dptr(E3_SEEN) + b"\x74\x01\xf0"    # SEEN = 1

    code += puts_code(STR_E3)
    code += emit_counter()
    code += puts_code(LBL_E3); code += puthex_xdata(0xE302)
    code += puts_code(LBL_66); code += puthex_dpx1(0x2800 + 0x66)
    code += puts_code(LBL_A0); code += puthex_dpx1(0x2800 + 0xA0)
    code += puts_code(LBL_A1); code += puthex_dpx1(0x2800 + 0xA1)
    code += puts_code(LBL_14); code += puthex_dpx1(0x1407)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"    # ']'

    skip = len(code)
    skip_abs = CAVE + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_A66_OLD                              # replay CLR A ; MOV 0x4d,A
    code += b"\x22"                                # ret
    return bytes(code)


def wrap_body(body):
    return (len(body).to_bytes(4, "little") + body
            + bytes([0xA5, sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, "little"))


def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little")
        footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            if data[footer + 1] != (sum(body) & 0xFF):
                raise ValueError("wrapped checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("wrapped crc mismatch")
            return bytearray(body), True
    return bytearray(data), False


def write_cave(body, addr, data, name):
    end = addr + len(data)
    if body[addr:end] != bytes(len(data)):
        raise ValueError(f"{name} cave at 0x{addr:04x} not empty (len {len(data)})")
    body[addr:end] = data
    return end


def patch_site(body, off, old, cave, name):
    found = bytes(body[off:off + len(old)])
    if found != old:
        raise ValueError(f"{name} mismatch at 0x{off:05x}: {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR_E3, b"\r\n[e3 \x00", "e3 str")
    write_cave(body, LBL_E3, b" e302=\x00", "e302")
    write_cave(body, LBL_66, b" 66=\x00", "66")
    write_cave(body, LBL_A0, b" A0=\x00", "A0")
    write_cave(body, LBL_A1, b" A1=\x00", "A1")
    write_cave(body, LBL_14, b" 1407=\x00", "1407")
    hook = build_hook()
    if CAVE + len(hook) > STR_E3:
        raise ValueError(f"cave overruns strings (end 0x{CAVE + len(hook):04x})")
    write_cave(body, CAVE, hook, "e3 hook")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE, "a066")
    return len(hook)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    hlen = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    print(f"  a066-entry hook @0x{CAVE:04x} ({hlen} bytes), change-gated on E302")
    print("  line: [e3 <ctr> e302= 66= A0= A1= 1407=]")


if __name__ == "__main__":
    main()
