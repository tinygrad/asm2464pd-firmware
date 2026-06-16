#!/usr/bin/env python3
"""
Patch stock fw_tinygrad.bin to UART-dump the wide CDR/PLL state (lock + full eye-margin
register block) sampled in the super-loop, logged ON CHANGE. Companion to patch_phylock.py.

The CDR margin regs C2D2/C2D9/C2DA/C352/C359/C35A are sample/hold registers that are only
CLOCKED after the PHY trains (E764>=0x19); reading them pre-train hangs the XDATA bus. So
this tracer GATES those reads on E764>=0x19 (prints "00" for them otherwise). The lock regs
C2D0/C350 + lanes + 0779 are always safe (and stay out of the change-signature, which uses
only always-clocked regs).

HOOK: same as patch_phylock -- super-loop top 0x2FC0 `mov dptr,#0x0AE2` (EA already 0).

LINE:  [W:<a0><a1><0779><C2D0><C350><E764><C2D2><C2D9><C2DA><C352><C359><C35A>]
  a0/a1 = SB[0xA0]/[0xA1] lane state; C2D0/C350 bit6=PLL-lock bit4=full-CDR-lock; the last
  six are the per-lane eye margins (00 until E764>=0x19). Compare stock's locked margins to
  the handmade's to confirm whether the handmade CDR physically locks.

Build:  python3 app/patch_phywide.py fw_tinygrad.bin /tmp/fw_phywide.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_phywide.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

HOOK_SITE_BODY = 0x2FC0
HOOK_SITE_OLD = bytes.fromhex("900ae2")  # mov dptr,#0x0AE2

CAVE = 0x5E00
PREFIX_ADDR = 0x5FC0
PREFIX = b"[W:\x00"

SIG_ADDR = 0x0BFE

# Always-clocked regs (printed every change, safe anytime).
ALWAYS_REGS = [0x0779, 0xC2D0, 0xC350, 0xE764]
# CDR eye-margin sample/hold regs: ONLY clocked post-train -> read gated on E764>=0x19.
MARGIN_REGS = [0xC2D2, 0xC2D9, 0xC2DA, 0xC352, 0xC359, 0xC35A]

SB_DPX = 0x01
SB_LANE = [0x28A0, 0x28A1]                     # SB[0xA0], SB[0xA1]

# Change-signature reads ONLY always-clocked regs (never C2xx) so early boot never hangs.
SIG_PLAIN = [0xC8FF, 0xE302, 0x0779]


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
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_sb(addr):
    return (
        bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
        + mov_dptr(addr) + b"\xe0\xff"         # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])            # mov DPX,#0
        + lcall(UART_PUTHEX)
    )


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF])
        + bytes([0x7A, (addr >> 8) & 0xFF])
        + bytes([0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


SIG_SCRATCH = 0x22
TMP_SCRATCH = 0x23

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x93, SIG_SCRATCH, TMP_SCRATCH)


def build_signature():
    """sig = xor(SIG_PLAIN) ^ SB[0xA0] ^ SB[0xA1] into 0x22 (only always-clocked regs)."""
    code = bytearray()
    first, rest = SIG_PLAIN[0], SIG_PLAIN[1:]
    code += mov_dptr(first) + b"\xe0" + bytes([0xF5, SIG_SCRATCH])
    for addr in rest:
        code += mov_dptr(addr) + b"\xe0"
        code += bytes([0x65, SIG_SCRATCH])
        code += bytes([0xF5, SIG_SCRATCH])
    for addr in SB_LANE:
        code += bytes([0x75, 0x93, SB_DPX])
        code += mov_dptr(addr) + b"\xe0"
        code += bytes([0x75, 0x93, 0x00])
        code += bytes([0x65, SIG_SCRATCH])
        code += bytes([0xF5, SIG_SCRATCH])
    return bytes(code)


def build_print():
    """[W:<a0><a1><0779><C2D0><C350><E764><margins...>] -- margins zeroed unless E764>=0x19."""
    code = bytearray()
    code += puts_code(PREFIX_ADDR)
    for addr in SB_LANE:                       # a0, a1 (DPX=1)
        code += puthex_sb(addr)
    for addr in ALWAYS_REGS:                   # 0779, C2D0, C350, E764 (DPX=0, always clocked)
        code += puthex_xdata(addr)
    margins = bytearray()
    for addr in MARGIN_REGS:
        margins += puthex_xdata(addr)
    zeros = bytearray()
    for _ in range(len(MARGIN_REGS)):
        zeros += bytes([0x7F, 0x00]) + lcall(UART_PUTHEX)   # mov r7,#0 ; lcall puthex -> "00"
    code += mov_dptr(0xE764) + b"\xe0"          # movx a,@dptr (E764); DPX already 0
    code += b"\xc3"                              # clr C
    code += bytes([0x94, 0x19])                  # subb a,#0x19 -> C set iff E764 < 0x19
    code += bytes([0x50, (len(zeros) + 2) & 0xFF])   # jnc PRINT_MARGINS (E764>=0x19)
    code += bytes(zeros)
    code += bytes([0x80, len(margins) & 0xFF])   # sjmp AFTER (over margins)
    code += bytes(margins)
    code += emit_char(']')
    code += emit_char('\r')
    code += emit_char('\n')
    return bytes(code)


def build_hook():
    code = bytearray()
    for direct in PRESERVE:
        code += bytes([0xC0, direct])

    code += bytes([0x75, 0x93, 0x00])                      # mov DPX,#0
    code += build_signature()                              # -> sig in 0x22

    code += mov_dptr(SIG_ADDR) + b"\xe0"                   # movx a,@dptr (old sig)
    code += bytes([0xB5, SIG_SCRATCH, 0x00])               # cjne a,0x22,neq ; rel filled below
    skip_branch = len(code) - 1

    print_code = build_print()
    store_sig = mov_dptr(SIG_ADDR) + bytes([0xE5, SIG_SCRATCH]) + b"\xf0"

    eq_sjmp = b"\x80\x00"
    neq = bytearray()
    neq += store_sig
    neq += print_code

    code[skip_branch] = len(eq_sjmp) & 0xFF
    code += eq_sjmp
    eq_sjmp_pos = len(code) - 2
    code += neq

    done = len(code)
    code[eq_sjmp_pos + 1] = (done - (eq_sjmp_pos + 2)) & 0xFF

    for direct in reversed(PRESERVE):
        code += bytes([0xD0, direct])

    code += HOOK_SITE_OLD
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
    found = bytes(body[HOOK_SITE_BODY:HOOK_SITE_BODY + len(HOOK_SITE_OLD)])
    if found != HOOK_SITE_OLD:
        raise ValueError(
            f"hook site mismatch at body 0x{HOOK_SITE_BODY:05x}: found {found.hex()}, "
            f"expected {HOOK_SITE_OLD.hex()}"
        )
    hook = build_hook()
    repl = bytearray(lcall(CAVE))
    while len(repl) < len(HOOK_SITE_OLD):
        repl += b"\x00"
    if len(repl) != len(HOOK_SITE_OLD):
        raise ValueError("replacement length mismatch")
    if CAVE + len(hook) > PREFIX_ADDR:
        raise ValueError(f"hook ({len(hook)} bytes) overruns PREFIX_ADDR 0x{PREFIX_ADDR:04x}")
    body[HOOK_SITE_BODY:HOOK_SITE_BODY + len(HOOK_SITE_OLD)] = repl
    write_cave(body, CAVE, hook, "phywide hook")
    write_cave(body, PREFIX_ADDR, PREFIX, "phywide prefix")
    return [("PHYWIDE", HOOK_SITE_BODY, CAVE, len(hook))]


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
    print("  line: [W:<a0><a1><0779><C2D0><C350><E764><C2D2><C2D9><C2DA><C352><C359><C35A>]  (margins gated E764>=0x19)")


if __name__ == "__main__":
    main()
