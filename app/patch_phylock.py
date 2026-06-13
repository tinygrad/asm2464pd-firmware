#!/usr/bin/env python3
"""
Patch stock fw_tinygrad.bin to UART-dump a TIMELINE of the USB4 PHY RXPLL-lock /
CL-snap state, sampled in the super-loop and logged ONLY ON CHANGE (no flood).

WHY: the handmade fw reaches state-5 with E764=0x19 (PHY train sequence complete)
but E762==0 throughout (E762.4 RXPLL-ready never holds) and the state-5 CL-walker's
snap byte 0x0779+lane never gets bit7 set -> the host never posts the CL response and
the lanes stay SB[0xA0]/[0xA1]==0x07. Hypothesis: stock HOLDS the RXPLL lock (E762.4)
and so gets a populated 0x0779 (bit7 set), while the handmade does not (suspect the
d436 lane-ramp PHY settles + b8db CDR-margin we simplified). This trace captures, on
STOCK, the timeline of E762 (RXPLL) + 0x0779/0x077A (CL snap) + C2D0/C350 (PLL lock)
alongside the rate/mode/lane state, to confirm whether stock holds E762.4 lock and
populates 0x0779 -- the host-vs-fw discriminator before we invest in the d436/b8db
faithful completion.

HOOK SITE: identical to patch_lanetrace.py -- main_boot_and_superloop @0x2FB4 body
top `mov dptr,#0x0AE2` (3 bytes) at 0x2FC0, right after `clr EA` (interrupts already
disabled -> atomic DPX-paged reads). We replace it with `lcall CAVE`; the cave samples,
prints only on a changed signature, then replays `mov dptr,#0x0AE2` and returns.

CHANGE SIGNATURE (1 byte @ XDATA 0x0BFE, a free working-RAM byte):
  C8FF ^ E302 ^ E762 ^ SB[0xA0] ^ SB[0xA1] ^ 0x0779 ^ 0x077A
so a new line prints whenever the rate, link-mode, RXPLL-ready, lanes, OR the CL snap
changes -> a compact timeline of the lock + snap evolution.

STOCK'S OWN PRINTS ARE KEPT (only the super-loop top is touched), so [SB Con]/[SB P0x]/
[PcieTunnel-*]/[*** USB4 Gen3 x2 ***] interleave with [P:] lines for time-alignment.

OUTPUT LINE (hex, DPX-restored before each print):
    [P:<C8FF><E302><E762><SBa0><SBa1><0779><077A><E764><C2D0><C350>]
       rate  mode  rxpll lane  lane  snap0 snap1 train pll   pll
  Key reads: E762.4 (RXPLL-ready) should be SET if stock holds lock; 0779 bit7 SET +
  bit4 CLEAR is the CL-ready snap that drives the walker's ea7c CL0 emit; C2D0/C350
  bit6 are the b8db PLL-lock poll bits.

Addressing: ghidra.c == this image. body = data[4:-6] (wrapped). Cave/prefix live in
the low shared zero region (<0x8000, flat both banks). SB[off] = paged XDATA 0x2800+off
(DPX=1); plain XDATA via DPX=0.

Build:
    python3 app/patch_phylock.py fw_tinygrad.bin /tmp/fw_phylock.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_phylock.bin"

# Stock UART helpers (low shared region, <0x8000 -> flat in both banks).
UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# Super-loop body top: `mov dptr,#0x0AE2` (3 bytes) right after `clr EA`.
HOOK_SITE_BODY = 0x2FC0
HOOK_SITE_OLD = bytes.fromhex("900ae2")  # mov dptr,#0x0AE2

# Caves in the low shared zero region (flat both banks), <0x8000.
CAVE = 0x5E00
PREFIX_ADDR = 0x5FC0
PREFIX = b"[P:\x00"

# Persistent change-signature byte (free working-RAM byte, plain XDATA DPX=0).
SIG_ADDR = 0x0BFE

# Plain-XDATA (DPX=0) registers logged, in print order.
#   C8FF lane-rate, E302 link-mode, E762 RXPLL-ready, then (SB lane bytes),
#   0x0779/0x077A CL snap, E764 train, C2D0/C350 PLL-lock.
PLAIN_PRE = [0xC8FF, 0xE302, 0xE762]          # before the SB lane bytes
PLAIN_POST = [0x0779, 0x077A, 0xE764, 0xC2D0, 0xC350]

# SB page-1 lane-state bytes (DPX=1, XDATA 0x2800+off).
SB_DPX = 0x01
SB_LANE = [0x28A0, 0x28A1]                     # SB[0xA0], SB[0xA1]

# XDATA addrs folded into the change-signature (plain DPX=0).
SIG_PLAIN = [0xC8FF, 0xE302, 0xE762, 0x0779, 0x077A]


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
    # DPX=0 plain XDATA read into R7, then print.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_sb(addr):
    # DPX=1 paged read, restore DPX=0, then print (the print's movx is DPX-sensitive).
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


# Scratch IRAM bytes used only inside the hook (saved/restored).
SIG_SCRATCH = 0x22
TMP_SCRATCH = 0x23

# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the scratch bytes we touch.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x93, SIG_SCRATCH, TMP_SCRATCH)


def build_signature():
    """sig = xor(SIG_PLAIN) ^ SB[0xA0] ^ SB[0xA1] into 0x22."""
    code = bytearray()
    first, rest = SIG_PLAIN[0], SIG_PLAIN[1:]
    code += mov_dptr(first) + b"\xe0" + bytes([0xF5, SIG_SCRATCH])
    for addr in rest:
        code += mov_dptr(addr) + b"\xe0"                   # movx a,@dptr
        code += bytes([0x65, SIG_SCRATCH])                 # xrl a,0x22
        code += bytes([0xF5, SIG_SCRATCH])                 # mov 0x22,a
    for addr in SB_LANE:
        code += bytes([0x75, 0x93, SB_DPX])                # mov DPX,#1
        code += mov_dptr(addr) + b"\xe0"                   # movx a,@dptr (paged)
        code += bytes([0x75, 0x93, 0x00])                  # mov DPX,#0
        code += bytes([0x65, SIG_SCRATCH])                 # xrl a,0x22
        code += bytes([0xF5, SIG_SCRATCH])                 # mov 0x22,a
    return bytes(code)


def build_print():
    """Emit the [P:...] line: PLAIN_PRE, SB lane bytes, PLAIN_POST."""
    code = bytearray()
    code += puts_code(PREFIX_ADDR)
    for addr in PLAIN_PRE:
        code += puthex_xdata(addr)
    for addr in SB_LANE:
        code += puthex_sb(addr)
    for addr in PLAIN_POST:
        code += puthex_xdata(addr)
    code += emit_char(']')
    code += emit_char('\r')
    code += emit_char('\n')
    return bytes(code)


def build_hook():
    code = bytearray()
    for direct in PRESERVE:
        code += bytes([0xC0, direct])                      # push direct

    # DPX is preserved above; ensure DPX=0 for the plain reads.
    code += bytes([0x75, 0x93, 0x00])                      # mov DPX,#0

    code += build_signature()                              # -> sig in 0x22

    # Compare sig (0x22) to stored 0x0BFE. If equal -> skip the print.
    code += mov_dptr(SIG_ADDR) + b"\xe0"                   # movx a,@dptr (old sig)
    code += bytes([0xB5, SIG_SCRATCH, 0x00])               # cjne a,0x22,neq ; rel filled below
    skip_branch = len(code) - 1                            # location of the cjne rel byte

    print_code = build_print()
    store_sig = mov_dptr(SIG_ADDR) + bytes([0xE5, SIG_SCRATCH]) + b"\xf0"

    eq_sjmp = b"\x80\x00"                                   # sjmp DONE (rel filled later)
    neq = bytearray()
    neq += store_sig
    neq += print_code

    code[skip_branch] = len(eq_sjmp) & 0xFF                # rel over the sjmp
    code += eq_sjmp
    eq_sjmp_pos = len(code) - 2
    code += neq

    done = len(code)
    code[eq_sjmp_pos + 1] = (done - (eq_sjmp_pos + 2)) & 0xFF

    for direct in reversed(PRESERVE):
        code += bytes([0xD0, direct])                      # pop direct

    code += HOOK_SITE_OLD                                  # replay mov dptr,#0x0AE2
    code += b"\x22"                                        # ret
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
    write_cave(body, CAVE, hook, "phylock hook")
    write_cave(body, PREFIX_ADDR, PREFIX, "phylock prefix")
    return [("PHYLOCK", HOOK_SITE_BODY, CAVE, len(hook))]


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
    print(f"  sig byte: XDATA 0x{SIG_ADDR:04x} (log-on-change)")
    print(f"  line: [P:<C8FF><E302><E762><SBa0><SBa1><0779><077A><E764><C2D0><C350>]")


if __name__ == "__main__":
    main()
