#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the PHY RATE / MODE state at the CL-walk.

GOAL (rate-tier divergence hunt, 2026-06-22): handmade's in-tree [CLW] diag reports
`pll=F4F4 rt=7272 clv=0C0D 779=3C3D E302=83` at the bond. The task hypothesis is that
handmade trains a HIGHER PHY rate tier (rt=7272 / pll=F4F4) than stock, which makes the
host offer cl_idx 0x0C (overshoot) instead of stock's 0x08, skipping the staggered 1->2
width walk. This tracer dumps the SAME registers on STOCK at the SAME hook site so we can
1:1 compare:
    rt  = C294 / C314   (PHY lane eq retrim, the [CLW] rt= field)
    pll = C2D0 / C350   (PHY lane lock status bit5=rate bit6=PLL, the [CLW] pll= field)
    E302 = PHY mode (bits4:5 lane config) -- stock=97(mode1) vs handmade=83(mode0)
    hd  = host partner pair 0x0779:0x077A (= the [CLW] 779= terminal)
    A   = SB[0xA0]:SB[0xA1] bond state
    plus C2D1/C351 lane status, C2EC/C30C? rate-source feeders, CA06 mode-next, E710 width.

HOOK SITE: same proven NON-INTRUSIVE af38-exit cave as app/patch_stock_txterm.py
(CODE_BANK1::b0af -- `e4 ff 02 d5 da` = CLR A ; MOV R7,A ; LJMP 0xd5da). At af38 exit the
SB walk state is current. Stock MUST still enumerate GPU 1002:7590 with this tracer.

GATE: budget-gated (first N af38 calls where SB[0xA0]==0x02, i.e. at/after CL0) so we
catch the bond-time PHY state without flooding. Falls back to a plain budget if A0 never
hits 0x02 (it does on stock).

Build:
    python3 app/patch_stock_phyrate.py fw_tinygrad.bin /tmp/fw_phyrate.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_phyrate.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# af38 exit @ CODE_BANK1::b0af
H_AF_OFF = bank1_off(0xB0AF)
H_AF_OLD = bytes.fromhex("e4ff02d5da")

SHARED_DUMP = 0x7000
STR_BASE = 0x7040

LABELS = [
    ("STR_PHY", b"\r\n[PHYR hd=\x00"),
    ("SEP_A",   b" A=\x00"),
    ("SEP_RT",  b" rt=\x00"),
    ("SEP_PLL", b" pll=\x00"),
    ("SEP_E302", b" E302=\x00"),
    ("SEP_E710", b" E710=\x00"),
    # width-event source registers (page-1 / DPX=1): 1407 evt, 1203 wpend, 124e, 128f, 1243
    ("SEP_1407", b" 1407=\x00"),
    ("SEP_1203", b" 1203=\x00"),
    ("SEP_124E", b" 124e=\x00"),
    ("SEP_128F", b" 128f=\x00"),
    ("SEP_1243", b" 1243=\x00"),
    ("SEP_1508", b" 1508=\x00"),
]

STR = {}
STR_TABLE = []
_p = STR_BASE
for _name, _bytes in LABELS:
    STR[_name] = _p
    STR_TABLE.append((_p, _bytes))
    globals()[_name] = _p
    _p += len(_bytes)
STR_END = _p

CAVE_AF = 0x6800

PHY_BUDGET = 0x0B5A
PHY_SEEN = 0x0B5B
PHY_BUDGET_N = 40

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


def puthex_xdata(addr):
    # plain XDATA read (DPX=0 assumed) -> R7 -> print
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_sb(off):
    return (
        bytes([0x75, 0x93, SB_DPX])
        + mov_dptr(0x2800 + (off & 0xFF)) + b"\xe0\xff"
        + bytes([0x75, 0x93, 0x00])
        + lcall(UART_PUTHEX)
    )


def puthex_p1(off):
    # page-1 (DPX=1) single-byte read at 0x1xxx, print, restore DPX=0.
    return (
        bytes([0x75, 0x93, SB_DPX])
        + mov_dptr(off) + b"\xe0\xff"
        + bytes([0x75, 0x93, 0x00])
        + lcall(UART_PUTHEX)
    )


def dump_block(start, count, dpx):
    return (
        bytes([0x75, 0x20, start & 0xFF])
        + bytes([0x75, 0x21, (start >> 8) & 0xFF])
        + bytes([0x75, 0x22, count & 0xFF])
        + bytes([0x74, dpx])
        + lcall(SHARED_DUMP)
    )


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x23, 0x93)


def build_shared_dump():
    code = bytearray()
    code += bytes([0xF5, 0x23])
    loop = len(code)
    code += bytes([0xE5, 0x23, 0xF5, 0x93])
    code += b"\xe5\x20\xf5\x82"
    code += b"\xe5\x21\xf5\x83"
    code += b"\xe0"
    code += bytes([0x75, 0x93, 0x00])
    code += b"\xff" + lcall(UART_PUTHEX)
    code += b"\x05\x20"
    code += b"\xe5\x20"
    jnz = len(code)
    code += b"\x70\x00"
    code += b"\x05\x21"
    no_carry = len(code)
    code[jnz + 1] = (no_carry - (jnz + 2)) & 0xFF
    djnz = len(code)
    code += b"\xd5\x22\x00"
    code[djnz + 2] = (loop - (djnz + 3)) & 0xFF
    code += b"\x22"
    return bytes(code)


def build_field_block():
    """DPX=0 on entry. Print the PHY-rate fields. hd value emitted first (label is
    '[PHYR hd=')."""
    code = bytearray()
    # hd = 0x0779:0x077A (plain XDATA)
    code += puthex_xdata(0x0779)
    code += puthex_xdata(0x077A)
    # A = SB[0xA0]:SB[0xA1] (paged)
    code += puts_code(SEP_A)
    code += dump_block(0x2800 + 0xA0, 0x02, SB_DPX)
    # rt = C294 / C314 (plain XDATA)
    code += puts_code(SEP_RT)
    code += puthex_xdata(0xC294)
    code += puthex_xdata(0xC314)
    # pll = C2D0 / C350
    code += puts_code(SEP_PLL)
    code += puthex_xdata(0xC2D0)
    code += puthex_xdata(0xC350)
    # E302 PHY mode
    code += puts_code(SEP_E302)
    code += puthex_xdata(0xE302)
    # E710 width
    code += puts_code(SEP_E710)
    code += puthex_xdata(0xE710)
    # width-event source regs (page-1 / DPX=1)
    code += puts_code(SEP_1407); code += puthex_p1(0x1407)
    code += puts_code(SEP_1203); code += puthex_p1(0x1203)
    code += puts_code(SEP_124E); code += puthex_p1(0x124E)
    code += puts_code(SEP_128F); code += puthex_p1(0x128F)
    code += puts_code(SEP_1243); code += puthex_p1(0x1243)
    code += puts_code(SEP_1508); code += puthex_p1(0x1508)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_af_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0

    # GATE: emit when P1[0x1407] != 0 (any link/tunnel/width event pending) -- this is
    # exactly the window the width-event (1407.0) would appear. page-1 (DPX=1) read.
    code += bytes([0x75, 0x93, SB_DPX])            # DPX=1
    code += mov_dptr(0x1407) + b"\xe0"             # movx a,@dptr (P1[0x1407])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    # if A == 0 -> skip (jz LJMP_SKIP). emit otherwise.
    jnz_emit_gate = len(code)
    code += b"\x70\x03"                            # jnz +3 (over the LJMP -> proceed)
    ljmp_skip_to_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (A==0)
    code[jnz_emit_gate + 1] = (len(code) - (jnz_emit_gate + 2)) & 0xFF

    # self-seed budget
    code += mov_dptr(PHY_SEEN) + b"\xe0"
    jnz_have_seen = len(code)
    code += b"\x70\x00"
    code += mov_dptr(PHY_BUDGET) + bytes([0x74, PHY_BUDGET_N, 0xF0])
    code += mov_dptr(PHY_SEEN) + b"\x74\x01\xf0"
    have_seen = len(code)
    code[jnz_have_seen + 1] = (have_seen - (jnz_have_seen + 2)) & 0xFF

    # budget gate
    code += mov_dptr(PHY_BUDGET) + b"\xe0"
    jnz_emit = len(code)
    code += b"\x70\x03"
    ljmp_skip_budget = len(code)
    code += b"\x02\x00\x00"

    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF
    code += b"\x14\xf0"                            # BUDGET--

    code += puts_code(STR_PHY)                      # "\r\n[PHYR hd="
    code += build_field_block()

    skip = len(code)
    skip_abs = CAVE_AF + skip
    code[ljmp_skip_to_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip_to_skip + 2] = skip_abs & 0xFF
    code[ljmp_skip_budget + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip_budget + 2] = skip_abs & 0xFF

    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += bytes([0x15, 0x81, 0x15, 0x81])
    code += H_AF_OLD
    return bytes(code)


def wrap_body(body):
    return (
        len(body).to_bytes(4, "little")
        + body
        + bytes([0xA5, sum(body) & 0xFF])
        + zlib.crc32(body).to_bytes(4, "little")
    )


def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little")
        footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            if data[footer + 1] != (sum(body) & 0xFF):
                raise ValueError("wrapped firmware checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("wrapped firmware crc mismatch")
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
        raise ValueError(
            f"{name} site mismatch at body 0x{off:05x}: found {found.hex()}, "
            f"expected {old.hex()}")
    repl = bytearray(lcall(cave))
    while len(repl) < len(old):
        repl += b"\x00"
    body[off:off + len(old)] = repl


def apply_patch(body):
    dump = build_shared_dump()
    write_cave(body, SHARED_DUMP, dump, "shared dump")
    for addr, data in STR_TABLE:
        write_cave(body, addr, data, f"str@0x{addr:04x}")
    af_hook = build_af_hook()
    if CAVE_AF + len(af_hook) > SHARED_DUMP:
        raise ValueError(f"af hook overruns SHARED_DUMP (len {len(af_hook)})")
    write_cave(body, CAVE_AF, af_hook, "af hook")
    patch_site(body, H_AF_OFF, H_AF_OLD, CAVE_AF, "af38")
    return [("phyr", H_AF_OFF, CAVE_AF, len(af_hook), H_AF_OLD)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    before = bytes(body)
    info = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)

    print(f"input:  {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    for name, off, cave, hlen, old in info:
        print(f"  {name}: site body 0x{off:05x}  hook 0x{cave:04x}..0x{cave + hlen:04x} ({hlen} B)")
    print("  GATE: SB[0xA0]==0x02 (at/after CL0), budget", PHY_BUDGET_N)
    print("  line: \\r\\n[PHYR hd=<779><77A> A=<A0><A1> rt=<C294><C314> pll=<C2D0><C350>"
          " d1=<C2D1><C351> rsrc=<C2EC><C36C> E302= CA06= E710= rdesc=<C2C9><C349>]")

    diffs = []
    i = 0
    n = len(before)
    while i < n:
        if before[i] != body[i]:
            j = i
            while j < n and before[j] != body[j]:
                j += 1
            diffs.append((i, j))
            i = j
        else:
            i += 1
    cave_regions = [
        (SHARED_DUMP, SHARED_DUMP + len(build_shared_dump())),
        (STR_BASE, STR_END),
        (CAVE_AF, CAVE_AF + info[0][3]),
        (H_AF_OFF, H_AF_OFF + len(H_AF_OLD)),
    ]

    def contained(region):
        s, e = region
        return any(cs <= s and e <= ce for cs, ce in cave_regions)

    all_ok = True
    for s, e in diffs:
        ok = contained((s, e))
        all_ok = all_ok and ok
        print(f"    [0x{s:05x}..0x{e:05x})  {e - s} bytes  {'ok' if ok else 'UNEXPECTED'}")
    print(f"  -> {'confined' if all_ok else 'UNEXPECTED CHANGES'}")


if __name__ == "__main__":
    main()
