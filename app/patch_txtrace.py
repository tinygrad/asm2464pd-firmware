#!/usr/bin/env python3
"""
Patch fw_tinygrad.bin (stock OR handmade-stock-layout) to UART-dump the SB-transport
TX state at EVERY d5da TX-go, so the route-query (SB[0x15]==5) TX path can be diffed
stock-vs-handmade.

HOOK: d5da's SB[0x10]=1 TX-go write (bank1 logical d5f9: `79 10 74 01 12 0b e6`,
body offset 0x15564). We replace those 7 bytes with `lcall CAVE` + NOP fill; the cave
dumps the TX-relevant state, then REPLAYS the 7 bytes (R1=0x10; A=1; lcall 0be6 ->
SB[0x10]=1) and RETs, so the actual TX-go still happens and the poll proceeds normally.

Each pass prints one line:
  [TX p15=<SB15> p10=<SB10> p2c=<SB2C> p0c=<SB0C> p0d=<SB0D> p0e=<SB0E> p18=<SB18>
    p28=<SB28>|tx=<2900[0..7]>|rx=<2a00[0..7]>|719=<> 758=<> 6ed=<> 6ee=<> 775=<>
    776=<> 777=<>|ccd9=<> c80a=<> cc37=<> ca60=<>]
The route-query is the line with p15=05. (The d5da SB[0x10]=1 write hasn't happened
yet at the hook, so p10 shows the PRE-go value; p2c shows pre-go SB[0x2C].)

Addressing: ghidra.c == this image. body = data[4:-6]. Bank1 logical A -> body off
A - 0x8000 + 0xFF6B (verified: d5da -> 0x15545). Cave lives <0x8000 (flat both banks);
UART helpers 0x538D puts / 0x51C7 puthex are <0x8000 too. SB[off] = paged XDATA
0x2800+off (DPX=1). TX plane 0x2900, RX plane 0x2a00. XDATA 0x0xxx via DPX=0.

Build:
    python3 app/patch_txtrace.py fw_tinygrad.bin /tmp/fw_txtrace.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_txtrace.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# d5da SB[0x10]=1 TX-go write: `79 10 74 01 12 0b e6` (7 bytes) at bank1 d5f9.
HOOK_BODY = 0x15564
HOOK_OLD = bytes.fromhex("79107401120be6")  # 7 bytes

CAVE = 0x5E00
PREFIX_ADDR = 0x60B0
PREFIX = b"\r\n[TX p15=\x00"

# SB control-page bytes to print (DPX=1, 0x2800+off).
SB_OFFS = [0x15, 0x10, 0x2C, 0x0C, 0x0D, 0x0E, 0x18, 0x28]
# XDATA gating (DPX=0).
XD_REGS = [0x0719, 0x0758, 0x06ED, 0x06EE, 0x0775, 0x0776, 0x0777]
# HW regs (DPX=0).
HW_REGS = [0xCCD9, 0xC80A, 0xCC37, 0xCA60]


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_a_imm(val):
    return bytes([0x74, val & 0xFF])


def emit_char(ch):
    if isinstance(ch, str):
        ch = ord(ch)
    return mov_dptr(UART_TX) + mov_a_imm(ch) + b"\xf0"


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


def puthex_dpx0(addr):
    """DPX=0 plain XDATA read of `addr`, puthex it (restore DPX=0 already)."""
    return (
        bytes([0x75, 0x93, 0x00])               # mov DPX,#0
        + mov_dptr(addr) + b"\xe0\xff"           # movx a,@dptr ; mov r7,a
        + lcall(UART_PUTHEX)
    )


def puthex_dpx1(addr):
    """DPX=1 paged read of `addr` (SB/TX/RX plane), then restore DPX=0 and puthex."""
    return (
        bytes([0x75, 0x93, 0x01])               # mov DPX,#1
        + mov_dptr(addr) + b"\xe0"               # movx a,@dptr (paged)
        + bytes([0x75, 0x93, 0x00])              # mov DPX,#0
        + b"\xff" + lcall(UART_PUTHEX)           # mov r7,a ; lcall puthex
    )


def build_hook():
    code = bytearray()
    # Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX.
    PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
                0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)
    for d in PRESERVE:
        code += bytes([0xC0, d])

    code += puts_code(PREFIX_ADDR)               # "\r\n[TX p15="
    # SB control bytes (DPX=1). First is p15 (after the prefix's "p15=").
    first = True
    for off in SB_OFFS:
        if not first:
            # print the label " pXX="
            pass
        code += puthex_dpx1(0x2800 + off)
        first = False
        # labels for readability are emitted as literal chars below; to keep the
        # cave small we instead print a single space separator + tag via fixed text.
        code += emit_char(' ')

    code += emit_char('|')
    # TX plane 0x2900[0..7]
    for i in range(8):
        code += puthex_dpx1(0x2900 + i)
    code += emit_char('|')
    # RX plane 0x2a00[0..7]
    for i in range(8):
        code += puthex_dpx1(0x2A00 + i)
    code += emit_char('|')
    # XDATA gating (DPX=0)
    for a in XD_REGS:
        code += puthex_dpx0(a)
        code += emit_char(' ')
    code += emit_char('|')
    # HW regs (DPX=0)
    for a in HW_REGS:
        code += puthex_dpx0(a)
        code += emit_char(' ')
    code += emit_char(']')

    # restore DPX=0 for the safe pops, then pop
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed((0xE0, 0xF0, 0x82, 0x83, 0xD0,
                       0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)):
        code += bytes([0xD0, d])

    code += HOOK_OLD                             # replay R1=0x10;A=1;lcall 0be6 (SB[0x10]=1)
    code += b"\x22"                              # ret
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


def apply_patch(body):
    found = bytes(body[HOOK_BODY:HOOK_BODY + len(HOOK_OLD)])
    if found != HOOK_OLD:
        raise ValueError(
            f"hook site mismatch at body 0x{HOOK_BODY:05x}: found {found.hex()}, "
            f"expected {HOOK_OLD.hex()}"
        )
    hook = build_hook()
    repl = bytearray(lcall(CAVE))
    while len(repl) < len(HOOK_OLD):
        repl += b"\x00"
    body[HOOK_BODY:HOOK_BODY + len(HOOK_OLD)] = repl
    write_cave(body, CAVE, hook, "TX hook")
    write_cave(body, PREFIX_ADDR, PREFIX, "TX prefix")
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
    print(f"  hook: body 0x{HOOK_BODY:05x} -> lcall 0x{CAVE:04x} "
          f"(hook {hlen} bytes -> 0x{CAVE + hlen:04x})")
    print(f"  SB offs: {[hex(o) for o in SB_OFFS]}")
    print(f"  XD: {[hex(a) for a in XD_REGS]}  HW: {[hex(a) for a in HW_REGS]}")


if __name__ == "__main__":
    main()
