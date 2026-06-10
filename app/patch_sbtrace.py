#!/usr/bin/env python3
"""
Patch stock fw_tinygrad.bin to UART-dump the USB4 sideband (SB) transport page
+ the USB4 gating/status registers AT the [===SB Con===] connect transition.

This is the STOCK-side capture for the stock-vs-handmade SB-page diff. The SB
connect handler is bank1 a066 (sb_router_event_handler); it prints
"[===SB Con===]" via UART_PUTS(0x2056) at the bank1 site whose string-pointer
setup is `mov R3,#FF; mov R2,#0x20; mov R1,#0x56` at bank1 logical 0xA0ED-ish.
We code-cave-hook that 6-byte setup (body offset 0x12052) and, before replaying
it, dump:
  - the full SB page-1 block SB[0x00..0xFF] (DPX=1 paged XDATA at 0x2800+off),
  - the gating/status regs (DPX=0 plain XDATA):
        0x0AF1 C80A E302 0x09F9 0x09FA EC06 91C0 E318
The cave lives in the LOW SHARED code region (<0x8000), which is flat in both
banks, so the bank1 site's LCALL reaches it and the UART helpers (0x538D puts,
0x51C7 puthex, also <0x8000) are callable.

Addressing: ghidra.c == this image. body = data[4:-6]. Bank1 logical addr A maps
to body offset A - 0x8000 + 0xFF6B (verified: a0e7 -> 0x12052 = 7b ff 7a 20 79 56).

Build:
    python3 app/patch_sbtrace.py fw_tinygrad.bin /tmp/fw_sbtrace.bin

Output (one line, ~280 hex bytes) at the connect transition, format:
    [SBCON:<0xAF1><C80A><E302><09F9><09FA><EC06><91C0><E318>|<SB[0x00..0xFF] 256 bytes>]
followed by the stock's own [===SB Con===] print.
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_sbtrace.bin"

# Stock UART helpers (low shared region, <0x8000 -> flat in both banks).
UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# Bank1 site: the `mov R3,#FF; mov R2,#0x20; mov R1,#0x56` that loads the
# "[===SB Con===]" string pointer, immediately before LCALL UART_PUTS.
SB_CON_SITE_BODY = 0x12052
SB_CON_SITE_OLD = bytes.fromhex("7bff7a207956")  # 6 bytes

# Cave in the low shared zero region (flat both banks).
CAVE = 0x5E00
PREFIX_ADDR = 0x5F80
PREFIX = b"[SBCON:\x00"

# SB page-1 accessor: DPX=1, XDATA 0x2800+off (verified vs handmade SB_RD).
SB_DPX = 0x01
SB_BASE = 0x2800
SB_COUNT = 0x100  # full 256-byte page

# Gating / status regs (DPX=0 plain XDATA), dumped first.
GATING_REGS = [0x0AF1, 0xC80A, 0xE302, 0x09F9, 0x09FA, 0xEC06, 0x91C0, 0xE318]


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


def puthex_xdata(addr):
    # DPX=0 plain XDATA read + print.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF])
        + bytes([0x7A, (addr >> 8) & 0xFF])
        + bytes([0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


def dump_block_dpx(start, count, dpx):
    """Emit a loop printing `count` hex bytes from paged XDATA `start` at DPX=dpx.

    Sets DPX=dpx for the read, restores DPX=0 before each puthex (the print's
    movx to UART_TX is DPX-sensitive). Uses RAM 0x20 (lo), 0x21 (hi), 0x22 (cnt).
    """
    code = bytearray()
    code += bytes([0x75, 0x20, start & 0xFF])         # mov 0x20,#lo
    code += bytes([0x75, 0x21, (start >> 8) & 0xFF])  # mov 0x21,#hi
    code += bytes([0x75, 0x22, count & 0xFF])         # mov 0x22,#count

    loop = len(code)
    code += bytes([0x75, 0x93, dpx & 0xFF])           # mov DPX,#dpx   (paged)
    code += b"\xe5\x20\xf5\x82"                        # mov a,0x20 ; mov DPL,a
    code += b"\xe5\x21\xf5\x83"                        # mov a,0x21 ; mov DPH,a
    code += b"\xe0"                                    # movx a,@dptr  (paged read)
    code += bytes([0x75, 0x93, 0x00])                 # mov DPX,#0     (restore)
    code += b"\xff" + lcall(UART_PUTHEX)              # mov r7,a ; lcall puthex
    code += b"\x05\x20"                               # inc 0x20
    code += b"\xe5\x20"                               # mov a,0x20
    jnz = len(code)
    code += b"\x70\x00"                               # jnz no_carry
    code += b"\x05\x21"                               # inc 0x21
    no_carry = len(code)
    code[jnz + 1] = (no_carry - (jnz + 2)) & 0xFF
    djnz = len(code)
    code += b"\xd5\x22\x00"                           # djnz 0x22, loop
    code[djnz + 2] = (loop - (djnz + 3)) & 0xFF
    return bytes(code)


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX, dump scratch.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x93)


def build_hook():
    code = bytearray()
    for direct in PRESERVE:
        code += bytes([0xC0, direct])                 # push direct

    code += puts_code(PREFIX_ADDR)
    for addr in GATING_REGS:
        code += puthex_xdata(addr)
    code += emit_char('|')
    code += dump_block_dpx(SB_BASE, SB_COUNT, SB_DPX)
    code += emit_char(']')

    for direct in reversed(PRESERVE):
        code += bytes([0xD0, direct])                 # pop direct

    code += SB_CON_SITE_OLD                           # replay replaced bytes
    code += b"\x22"                                   # ret
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
            checksum = data[footer + 1]
            crc = int.from_bytes(data[footer + 2:footer + 6], "little")
            if checksum != (sum(body) & 0xFF):
                raise ValueError("wrapped firmware checksum mismatch")
            if crc != zlib.crc32(body):
                raise ValueError("wrapped firmware crc mismatch")
            return bytearray(body), True
    return bytearray(data), False


def write_cave(body, addr, data, name):
    end = addr + len(data)
    if body[addr:end] != bytes(len(data)):
        raise ValueError(f"{name} cave at 0x{addr:04x} is not empty (len {len(data)})")
    body[addr:end] = data
    return end


def apply_patch(body):
    found = bytes(body[SB_CON_SITE_BODY:SB_CON_SITE_BODY + len(SB_CON_SITE_OLD)])
    if found != SB_CON_SITE_OLD:
        raise ValueError(
            f"SBCON site mismatch at body 0x{SB_CON_SITE_BODY:05x}: found {found.hex()}, "
            f"expected {SB_CON_SITE_OLD.hex()}"
        )
    hook = build_hook()
    # Replace the 6-byte string-ptr setup with `lcall CAVE` (3) + NOP fill (3).
    repl = bytearray(lcall(CAVE))
    while len(repl) < len(SB_CON_SITE_OLD):
        repl += b"\x00"
    body[SB_CON_SITE_BODY:SB_CON_SITE_BODY + len(SB_CON_SITE_OLD)] = repl
    write_cave(body, CAVE, hook, "SBCON hook")
    write_cave(body, PREFIX_ADDR, PREFIX, "SBCON prefix")
    return [("SBCON", SB_CON_SITE_BODY, CAVE, len(hook))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    info = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)

    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    for name, site, cave, hlen in info:
        print(f"  {name}: site body 0x{site:05x} -> lcall 0x{cave:04x} "
              f"(hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print(f"  gating regs: {', '.join(hex(r) for r in GATING_REGS)}")
    print(f"  SB page: DPX={SB_DPX} 0x{SB_BASE:04x}..0x{SB_BASE + SB_COUNT - 1:04x} "
          f"({SB_COUNT} bytes)")


if __name__ == "__main__":
    main()
