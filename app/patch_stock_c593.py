#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the c593 PCIe-DOWN ADAPTER ADVERTISE.

Hooks the state-4 call site `CODE_BANK1::b1b0: LCALL 0x05c0` (the banked stub that
tail-calls bank0 c593). The cave REPLAYS the LCALL (so c593 runs the real advertise),
then dumps the advertise INPUT/OUTPUT byte-for-byte so we can diff stock vs handmade:

  [C593 <ctr16> v2805=<plane2 0x2805 read = e916 input>
         072D=<bonded gate> 819=<lane-adv mask>
         |1334=<..> 1335=<..> 134D=<..> 1285=<..>
         |1206=<adapter-CS status> 1208=<..> 1210=<..> 1200=<..> 1203=<..>]

v2805 is read via the SAME R3=2 path c593's e916 uses (DPX=(R3-1)=1, DPTR=0x2805).
The 13xx/12xx adapter regs are page-1 (DPX=1). This is the advertise the host CM
reads over the tunnel to decide RESET vs ENABLE.

The hook fires once per state-4 c593 call (low frequency). Change-gated on the
counter is not needed (state-4 runs a handful of times), but we budget to 8 lines.

Build:
    python3 app/patch_stock_c593.py fw_tinygrad.bin /tmp/fw_c593.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_c593.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# Hook site: CODE_BANK1::b1b0 `LCALL 0x05c0` = 12 05 c0 (the c593 banked stub call).
H_OFF = bank1_off(0xB1B0)
H_OLD = bytes.fromhex("1205c0")

CAVE = 0x6A00          # roomy free slot below the 0x7000 strings
STR_C593 = 0x7000      # "\r\n[C593 "
SEP_A = 0x7010         # " 072D="
SEP_B = 0x7018         # " 819="
SEP_C = 0x7020         # "|1334="
SEP_D = 0x7028         # " 1335="
SEP_E = 0x7030         # " 134D="
SEP_F = 0x7038         # " 1285="
SEP_G = 0x7040         # "|1206="
SEP_H = 0x7048         # " 1208="
SEP_I = 0x7050         # " 1210="
SEP_J = 0x7058         # " 1200="
SEP_K = 0x7060         # " 1203="
STR_V = 0x7068         # " v2805="

CTR_LO = 0x8830
CTR_HI = 0x8831
BUDGET = 0x0B5B        # decrementing log budget (free 0x0B5x headroom cell)
SEEN = 0x0B5C
BUDGET_N = 0x08

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return (bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF])
            + lcall(UART_PUTS))


def puthex_xdata(addr):
    # DPX=0 plain XDATA read into R7, then print. Assumes DPX already 0.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_p1(addr):
    # DPX=1 paged read of page-1 `addr`, restore DPX=0, then print.
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


# Preserve everything (mid-state4 call).
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


def build_field_block():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    # v2805 = DPX=1 read of 0x2805 (the same physical cell e916 reads)
    code += puts_code(STR_V); code += puthex_p1(0x2805)
    code += puts_code(SEP_A); code += puthex_xdata(0x072D)
    code += puts_code(SEP_B); code += puthex_xdata(0x0819)
    code += puts_code(SEP_C); code += puthex_p1(0x1334)
    code += puts_code(SEP_D); code += puthex_p1(0x1335)
    code += puts_code(SEP_E); code += puthex_p1(0x134D)
    code += puts_code(SEP_F); code += puthex_p1(0x1285)
    code += puts_code(SEP_G); code += puthex_p1(0x1206)
    code += puts_code(SEP_H); code += puthex_p1(0x1208)
    code += puts_code(SEP_I); code += puthex_p1(0x1210)
    code += puts_code(SEP_J); code += puthex_p1(0x1200)
    code += puts_code(SEP_K); code += puthex_p1(0x1203)
    code += bytes([0x75, 0x93, 0x00])              # DPX=0 (restore)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_hook():
    code = bytearray()
    # 1) replay the LCALL 0x05c0 (run the real c593) FIRST
    code += lcall(0x05c0)
    # 2) save regs
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0

    # self-seed budget
    code += mov_dptr(SEEN) + b"\xe0"
    jnz_have = len(code)
    code += b"\x70\x00"
    code += mov_dptr(BUDGET) + bytes([0x74, BUDGET_N, 0xF0])
    code += mov_dptr(SEEN) + b"\x74\x01\xf0"
    have = len(code)
    code[jnz_have + 1] = (have - (jnz_have + 2)) & 0xFF

    # budget gate
    code += mov_dptr(BUDGET) + b"\xe0"
    jnz_emit = len(code)
    code += b"\x70\x00"
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"
    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    # EMIT: consume budget (A=BUDGET, DPTR=BUDGET)
    code += b"\x14\xf0"                             # dec a ; movx @dptr,a

    code += puts_code(STR_C593)
    code += emit_counter()
    code += build_field_block()

    skip = len(code)
    skip_abs = CAVE + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # restore + RET
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += b"\x22"                                 # ret (returns to b1b3)
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
                raise ValueError("checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("crc mismatch")
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
        raise ValueError(f"{name} site mismatch at body 0x{off:05x}: {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR_C593, b"\r\n[C593 \x00", "C593 str")
    write_cave(body, STR_V, b" v2805=\x00", "v")
    write_cave(body, SEP_A, b" 072D=\x00", "072D")
    write_cave(body, SEP_B, b" 819=\x00", "819")
    write_cave(body, SEP_C, b"|1334=\x00", "1334")
    write_cave(body, SEP_D, b" 1335=\x00", "1335")
    write_cave(body, SEP_E, b" 134D=\x00", "134D")
    write_cave(body, SEP_F, b" 1285=\x00", "1285")
    write_cave(body, SEP_G, b"|1206=\x00", "1206")
    write_cave(body, SEP_H, b" 1208=\x00", "1208")
    write_cave(body, SEP_I, b" 1210=\x00", "1210")
    write_cave(body, SEP_J, b" 1200=\x00", "1200")
    write_cave(body, SEP_K, b" 1203=\x00", "1203")
    hook = build_hook()
    write_cave(body, CAVE, hook, "c593 hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "b1b0")
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
    print(f"  c593 hook: site body 0x{H_OFF:05x} (LCALL 05c0) -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print("  dumps [C593 v2805 072D 819 |1334 1335 134D 1285 |1206 1208 1210 1200 1203] per state-4 c593")


if __name__ == "__main__":
    main()
