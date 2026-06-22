#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the LINK/ROUTER-OP/CONNECT SFR BANK.

Purpose: dump the never-fully-diffed connect/bond-engine SFR set at the bond
(a066 change-gated on SB[0x66]), byte-comparable to handmade's [SFR ...] line
(handmade/src/sb_router.h CLW block). NON-INTRUSIVE: hooks ONLY the low-frequency
a066 INT1 handler (same proven cave mechanism as app/patch_stock_sball.py, which
enumerated the GPU with the hook live).

SFRs dumped (all DPX=0 XDATA reads; same order as handmade [SFR ...]):
  C805 C806 C809 C80A C8B0 C8C7 C8FF CA06 CA60 CA81 CA86
  CC30 CC36 CC37 CC3B E302 E40F E710 E716 E717 E763 E795 E7E3
  EA80 EA88 EA89 EA90 EC00 EC04 EC05 EC06 B402 B480

The hook fires at a066 ENTRY change-gated on SB[0x66] (paged XDATA 0x2866) so it
captures the commit frame (SB[0x66]=01) AND the pre-bond frame. Each line:
  \r\n[SFR <ctr16> C805=.. C806=.. ... B480=..]

Build:
    python3 app/patch_stock_sfr.py fw_tinygrad.bin /tmp/fw_sfr.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_sfr.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# a066 entry: `e4 f5 4d` = CLR A ; MOV 0x4d,A (2 instructions, 3 bytes).
H_A66_OFF = bank1_off(0xA066)
H_A66_OLD = bytes.fromhex("e4f54d")

CAVE_A66 = 0x6600          # roomy slot below the 0x7000 strings
STR_SFR = 0x7000           # "\r\n[SFR "
LBLS_BASE = 0x7010         # label strings packed sequentially from here

CTR_LO = 0x8830
CTR_HI = 0x8831
A66_LAST = 0x0B56
A66_SEEN = 0x0B57

SB_DPX = 0x01

# The SFR list: (label_text, xdata_addr). label_text includes the leading space.
SFRS = [
    (" C805=", 0xC805), (" C806=", 0xC806), (" C809=", 0xC809), (" C80A=", 0xC80A),
    (" AEC=", 0x0AEC), (" AED=", 0x0AED), (" L0=", 0x0750), (" L1=", 0x0751),
    (" C294=", 0xC294), (" C314=", 0xC314),
    (" C8B0=", 0xC8B0), (" C8C7=", 0xC8C7), (" C8FF=", 0xC8FF), (" CA06=", 0xCA06),
    (" CA60=", 0xCA60), (" CA81=", 0xCA81), (" CA86=", 0xCA86), (" CC30=", 0xCC30),
    (" CC36=", 0xCC36), (" CC37=", 0xCC37), (" CC3B=", 0xCC3B), (" E302=", 0xE302),
    (" E40F=", 0xE40F), (" E710=", 0xE710), (" E716=", 0xE716), (" E717=", 0xE717),
    (" E763=", 0xE763), (" E795=", 0xE795), (" E7E3=", 0xE7E3), (" EA80=", 0xEA80),
    (" EA88=", 0xEA88), (" EA89=", 0xEA89), (" EA90=", 0xEA90), (" EC00=", 0xEC00),
    (" EC04=", 0xEC04), (" EC05=", 0xEC05), (" EC06=", 0xEC06), (" B402=", 0xB402),
    (" B480=", 0xB480),
]

# PAGE-1 (DPX=1) adapter-config / width-event registers. These are the C80A.4 width-event
# SOURCE block (P1[0x1201] current-width walk, P1[0x1203].7 width-change pending, P1[0x1206/7]
# enable mask) plus the adapter-advertise (1334/1335/134D/1285), the lane-evt (1403-1408) and
# tunnel-evt (1503-1508) blocks, and the adapter-evt (1603/1604). Order matches the handmade
# in-tree [SFRP] dump so the two strings are byte-comparable.
P1_SFRS = [
    (" 1200=", 0x1200), (" 1201=", 0x1201), (" 1202=", 0x1202), (" 1203=", 0x1203),
    (" 1206=", 0x1206), (" 1207=", 0x1207), (" 1208=", 0x1208), (" 1210=", 0x1210),
    (" 1280=", 0x1280), (" 1285=", 0x1285), (" 128F=", 0x128F),
    (" 1334=", 0x1334), (" 1335=", 0x1335), (" 134D=", 0x134D),
    (" 1403=", 0x1403), (" 1404=", 0x1404), (" 1405=", 0x1405), (" 1406=", 0x1406),
    (" 1407=", 0x1407), (" 1408=", 0x1408),
    (" 1503=", 0x1503), (" 1504=", 0x1504), (" 1505=", 0x1505), (" 1506=", 0x1506),
    (" 1507=", 0x1507), (" 1508=", 0x1508),
    (" 1603=", 0x1603), (" 1604=", 0x1604),
]


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
    # DPX assumed 0. movx a,@dptr ; mov r7,a ; lcall puthex
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_xdata_dpx1(addr):
    # set DPX=1, read page-1, restore DPX=0. movx a,@dptr ; mov r7,a ; lcall puthex
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x23, 0x93)


def emit_counter():
    code = bytearray()
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


# Allocate the label strings sequentially and remember their addresses.
def alloc_labels():
    addrs = {}
    cur = LBLS_BASE
    for text, _ in SFRS + P1_SFRS:
        addrs[text] = cur
        cur += len(text) + 1     # +1 for NUL terminator
    return addrs, cur


LBL_ADDR, LBL_END = alloc_labels()


def build_sfr_field_block():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    for text, addr in SFRS:
        code += puts_code(LBL_ADDR[text])
        code += puthex_xdata(addr)
    # page-1 (DPX=1) adapter-config / width-event block
    for text, addr in P1_SFRS:
        code += puts_code(LBL_ADDR[text])
        code += puthex_xdata_dpx1(addr)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (restore)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_a66_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # change gate on SB[0x66] (paged XDATA 0x2866, DPX=1)
    code += bytes([0x75, 0x93, SB_DPX])
    code += mov_dptr(0x2800 + 0x66) + b"\xe0"
    code += b"\xf5\xf0"                            # mov B,a (SB66)
    code += bytes([0x75, 0x93, 0x00])
    code += mov_dptr(A66_SEEN) + b"\xe0"
    jz_emit = len(code)
    code += b"\x60\x00"
    code += mov_dptr(A66_LAST) + b"\xe0"
    code += b"\x65\xf0"
    jnz_emit = len(code)
    code += b"\x70\x00"
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"

    emit = len(code)
    code[jz_emit + 1] = (emit - (jz_emit + 2)) & 0xFF
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    code += mov_dptr(A66_LAST) + b"\xe5\xf0\xf0"
    code += mov_dptr(A66_SEEN) + b"\x74\x01\xf0"

    code += puts_code(STR_SFR)
    code += emit_counter()
    code += build_sfr_field_block()

    skip = len(code)
    skip_abs = CAVE_A66 + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_A66_OLD
    code += b"\x22"
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
            f"{name} site mismatch at body 0x{off:05x}: found {found.hex()}, expected {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR_SFR, b"\r\n[SFR \x00", "SFR str")
    for text in LBL_ADDR:
        write_cave(body, LBL_ADDR[text], text.encode() + b"\x00", text.strip())
    hook = build_a66_hook()
    cave_end = CAVE_A66 + len(hook)
    if cave_end > STR_SFR:
        raise ValueError(f"cave overruns into strings (end 0x{cave_end:04x} >= 0x{STR_SFR:04x})")
    write_cave(body, CAVE_A66, hook, "a66 hook")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE_A66, "a066")
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
    print(f"  a066 hook: site body 0x{H_A66_OFF:05x} -> lcall 0x{CAVE_A66:04x} ({hlen} bytes)")
    print(f"  labels 0x{LBLS_BASE:04x}-0x{LBL_END:04x}; cave end 0x{CAVE_A66 + hlen:04x}")
    print("  change-gated on SB[0x66]; dumps the link/router-op/connect SFR bank at the bond.")


if __name__ == "__main__":
    main()
