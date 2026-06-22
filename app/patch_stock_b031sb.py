#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for THE DECISIVE b031-then-SB-read experiment.

QUESTION (the fork that decides the whole route=1-responder investigation):
  When stock runs bank0 b031 (the SB-transport / in-band control-adapter REINIT) inside
  the width-event/bond ISR, does stock's OWN next SB register READ succeed immediately
  -- with NO host follow-up packet -- or does it stall (like handmade, which HARD-HANGS
  on the next SB_RD after porting b031)?

  If stock's SB reads work right after b031 (no intervening host TLP) -> handmade's hang
  is a TRANSCRIPTION/CONTEXT bug (verdict A).
  If stock ALSO stalls until a host packet -> genuinely host-gated (verdict B).

HOOK SITES (both fire on the discrete bond/transport-up event, a handful of times):
  1) CODE:b031 ENTRY (bank0, flat file_off = ghidra + 0x4). Tag [B031in].
     Dumps the SB-read engine state the INSTANT b031 begins (before any of its writes):
     a live SB read (SB[0xA0]) + the descriptor-engine regs e5b0 will touch
     (P12[0x03]/[0x8F]/[0x90], i.e. P1[0x1203]/[0x128F]/[0x1290]) + EC06/EA90/E302.
     Displaced head = `7b 02 7a 14` (MOV R3,#2 ; MOV R2,#0x14 -- 2 instr, 4 bytes).
  2) CODE_BANK1::e530 (bank1, file_off = ghidra + 0x7f6f). This is the instruction
     RIGHT AFTER e52d's `LCALL 0x05d4` (= b031) returns, BEFORE e52d's b7a4/eb0a/3578
     follow-ups and BEFORE any host packet. Tag [AFb031]. This is the decisive point:
       * does the LIVE SB read here return (cave prints the byte + a sentinel '!' AFTER
         the read) -> if we see the sentinel, stock's SB read SURVIVED b031.
       * dumps the same SB/descriptor-engine regs to compare pre/post b031.
     Displaced head = `e4 ff 12 05 98` (CLR A ; MOV R7,A ; LCALL 0x0598 -- 5 bytes).

DECISIVE READOUT:
  If the UART shows `[B031in ...]` then `[AFb031 ...sb=<val>!...]` (sentinel '!' present)
  AND stock still enumerates the GPU (host trace / lspci), then stock's SB read engine is
  ALIVE right after b031 with no host follow-up -> the handmade hang is device-side
  (transcription/context), verdict A. If `[AFb031` never appears or truncates before the
  sentinel, the read stalled -> verdict B (or stock never reaches e52d, which itself is
  informative: e52d is host-gated on P1[0x0109]).

NON-INTRUSIVENESS: both hooks fire only on the rare bond/transport-up path (NOT the hot
8000 walker), each caves PUSH/POP all touched regs (mid-ISR safe), and the b031 hook is a
strict pass-through (read-only, no state change). The e530 hook likewise only reads.

Build:
    python3 app/patch_stock_b031sb.py fw_tinygrad.bin /tmp/fw_b031sb.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_b031sb.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# THIS fw_tinygrad.bin is a WRAPPED image: 4-byte header (LE body length) + body +
# 10-byte footer (0xA5 magic + 1-byte checksum + 4-byte CRC32). The boot/flash validates
# the footer, so ANY raw byte change without recomputing it bricks boot (reboot loop /
# `\QQQQ` UART garbage). We unwrap -> patch in BODY space -> rewrap (checksum+CRC fixed).
# In body space, GHIDRA addresses map directly:
#   bank0 (flat code): body_off = ghidra_addr
#   bank1 (CODE_BANK1): body_off = ghidra_addr + 0x7F6B
def bank1_off(addr):
    return addr + 0x7F6B


# ---- Hook sites (BODY offsets) ----
H_E530_OFF = bank1_off(0xE530)
H_E530_OLD = bytes.fromhex("e4ff120598")   # CLR A ; MOV R7,A ; LCALL 0x0598 (5 bytes)

# ---- Caves + strings in the low shared zero region (<0x8000, flat in both banks). ----
SHARED_DUMP = 0x7000
STR_B031 = 0x7040          # "\r\n[B031in"
STR_AF = 0x7050            # "\r\n[AFb031"
LBL_SB = 0x7060            # " sb="
LBL_1203 = 0x7068          # " 1203="
LBL_128F = 0x7074          # " 128F="
LBL_1290 = 0x7080          # " 1290="
LBL_EC06 = 0x708C          # " EC06="
LBL_EA90 = 0x7098          # " EA90="
LBL_E302 = 0x70A4          # " E302="

CAVE_B031 = 0x6800
CAVE_AF = 0x6C00

SB_DPX = 0x01

STRINGS = {
    STR_B031: "\r\n[B031in",
    STR_AF: "\r\n[AFb031",
    LBL_SB: " sb=",
    LBL_1203: " 1203=",
    LBL_128F: " 128F=",
    LBL_1290: " 1290=",
    LBL_EC06: " EC06=",
    LBL_EA90: " EA90=",
    LBL_E302: " E302=",
}


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    # DPX=0 plain XDATA read into R7, then print. Assumes DPX already 0.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_p1(addr):
    # DPX=1 paged read of page-1 reg `addr`, restore DPX=0, then print.
    return (
        bytes([0x75, 0x93, SB_DPX])
        + mov_dptr(addr) + b"\xe0\xff"
        + bytes([0x75, 0x93, 0x00])
        + lcall(UART_PUTHEX)
    )


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX. Both hooks run mid-ISR / mid-call.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def field_block(with_sentinel):
    """Shared field dump: a live SB read (SB[0xA0]) + descriptor-engine regs + transport
    state. If with_sentinel, emit '!' immediately AFTER the SB read returns -- proving the
    read did not stall."""
    code = bytearray()
    code += puts_code(LBL_SB)
    code += puthex_p1(0x2800 + 0xA0)               # live SB read (SB[0xA0]) via DPX=1 paged
    if with_sentinel:
        # '!' printed only if the SB read above completed and returned here.
        code += mov_dptr(UART_TX) + b"\x74\x21\xf0"   # '!'
    code += puts_code(LBL_1203); code += puthex_p1(0x1203)   # e5b0 writes P12[0x03]
    code += puts_code(LBL_128F); code += puthex_p1(0x128F)   # e5b0 writes P12[0x8F]
    code += puts_code(LBL_1290); code += puthex_p1(0x1290)   # e5b0 RMW P12[0x90]
    code += puts_code(LBL_EC06); code += puthex_xdata(0xEC06)
    code += puts_code(LBL_EA90); code += puthex_xdata(0xEA90)
    code += puts_code(LBL_E302); code += puthex_xdata(0xE302)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_hook(str_addr, with_sentinel, old_bytes):
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += puts_code(str_addr)
    code += field_block(with_sentinel)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += old_bytes                              # replay displaced head
    code += b"\x22"                                # ret
    return bytes(code)


def wrap_body(body):
    return (
        len(body).to_bytes(4, "little")
        + bytes(body)
        + bytes([0xA5, sum(body) & 0xFF])
        + zlib.crc32(bytes(body)).to_bytes(4, "little")
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile", nargs="?", default=str(DEFAULT_IN))
    ap.add_argument("outfile", nargs="?", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    raw = bytearray(Path(args.infile).read_bytes())
    body, wrapped = unwrap_image(raw)   # patch in BODY space; rewrap recomputes the footer

    # Sanity: confirm the displaced e530 head matches (catches a wrong-binary / wrong-offset).
    assert body[H_E530_OFF:H_E530_OFF + len(H_E530_OLD)] == H_E530_OLD, (
        f"e530 head mismatch @body 0x{H_E530_OFF:x}: {body[H_E530_OFF:H_E530_OFF+5].hex()}")

    # Strings + cave live in the zero padding at body 0x6800-0x7100 (ghidra addr == body off).
    for addr, s in STRINGS.items():
        b = s.encode() + b"\x00"
        assert not any(body[addr:addr + len(b)]), f"string region 0x{addr:x} not empty"
        body[addr:addr + len(b)] = b

    # Cave. Hook ONLY e530 (inside e52d, bond-gated on P1[0x0109]; never runs at boot). e530 is
    # RIGHT AFTER b031 returns, before any e52d follow-up and before any host packet -- the
    # decisive "SB read after b031, no intervening host TLP" point. (A b031-ENTRY hook is NOT
    # used: b031 also runs at BOOT via d894 with the UART not fully up -> bricks boot.)
    af_hook = build_hook(STR_AF, True, H_E530_OLD)
    assert not any(body[CAVE_AF:CAVE_AF + len(af_hook)]), "AF cave not empty"
    body[CAVE_AF:CAVE_AF + len(af_hook)] = af_hook
    assert CAVE_AF + len(af_hook) <= SHARED_DUMP, "AF cave overruns string region"

    # Patch hook site: LCALL cave + NOP pad to cover the displaced head length.
    ins = lcall(CAVE_AF) + b"\x00" * (len(H_E530_OLD) - 3)
    assert len(ins) == len(H_E530_OLD)
    body[H_E530_OFF:H_E530_OFF + len(ins)] = ins

    out = wrap_body(body) if wrapped else bytes(body)
    Path(args.outfile).write_bytes(out)
    print(f"wrote {args.outfile} ({len(out)} bytes, wrapped={wrapped})  "
          f"crc32={zlib.crc32(out) & 0xffffffff:08x}")
    print(f"  e530 hook @body 0x{H_E530_OFF:x}  cave 0x{CAVE_AF:x} ({len(af_hook)} B)")
    print(f"  (b031-entry hook NOT used: it also fires at boot via d894 -> bricks)")


if __name__ == "__main__":
    main()
