#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the LANE-ENABLE inputs at the entry of
the connection-routing setup function.

GOAL: dump the lane-enable inputs at the moment stock decides lane enablement, i.e.
at the ENTRY of cm_conn_routing_setup @ CODE_BANK1::a869 -- the function that
computes XDATA[0x0819] (the lane-present mask) from XDATA[0x077A] and XDATA[0x081A].
We want the runtime values of those bytes on real hardware.

HOOK SITE: the entry of CODE_BANK1::a869 (body off via A - 0x8000 + 0xFF6B, exactly
as clbond does for 8000). The head (Ghidra-verified) is:
    a869  12 e391    LCALL 0xe391
    a86c  90 0776    MOV   DPTR,#0x776
= 6 bytes `12 E3 91 90 07 76` = 2 WHOLE instructions. We displace those 6 bytes with
`LCALL CAVE` + 3 NOP (mirrors clbond's 6-byte head displacement), dump, then REPLAY
the 6 displaced bytes verbatim and RET so the function runs normally.

CHANGE-GATE: the cave reads XDATA[0x0819] (lane-present mask) each pass and emits a
line ONLY when it changed vs the last-seen value, kept in scratch XDATA 0x0B56 with a
seen-flag in 0x0B57 (the SAME free scratch cells clbond uses -- safe to reuse, the two
hooks never run concurrently). All gated reads are plain DPX=0 XDATA (0x06xx/0x07xx/
0x08xx), so no DPX juggling is needed.

LINE FORMAT (hex, exactly as requested):
  \r\n[cr 77A=<0x077A> 81A=<0x081A> 819=<0x0819> 778=<0x0778> 779=<0x0779>
       77B=<0x077B> 77C=<0x077C> 77D=<0x077D> ED=<0x06ED> 776=<0x0776>]

Uses the same uart_puts (0x538D) / uart_puthex (0x51C7) helpers clbond uses.

Build:
    python3 app/patch_crtrace.py fw_tinygrad.bin /tmp/fw_crtrace.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = Path("/tmp/fw_crtrace.bin")

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# Hook: CODE_BANK1::a869 entry. Displace the 6-byte head (2 instructions).
H_OFF = bank1_off(0xA869)
H_OLD = bytes.fromhex("12e391900776")   # LCALL 0xe391 ; MOV DPTR,#0x776

# Cave in the low shared zero region (<0x8000, flat in both banks).
CAVE = 0x6800
STR_CR = 0x7000            # "\r\n[cr 77A="
LBL_81A = 0x7010          # " 81A="
LBL_819 = 0x7018          # " 819="
LBL_778 = 0x7020          # " 778="
LBL_779 = 0x7028          # " 779="
LBL_77B = 0x7030          # " 77B="
LBL_77C = 0x7038          # " 77C="
LBL_77D = 0x7040          # " 77D="
LBL_ED = 0x7048           # " ED="
LBL_776 = 0x7050          # " 776="

# Scratch XDATA (DPX=0), same free cells clbond reuses (0x0B56/0x0B57 genuinely free
# on stock per the handmade 0x0B5x headroom layout).
LAST_819 = 0x0B56
SEEN = 0x0B57


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
    # DPX=0 plain XDATA read into R7, then print. Assumes DPX already 0.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def build_field_block():
    """The per-line field block. DPX is 0 throughout (all plain XDATA reads)."""
    code = bytearray()
    # The "\r\n[cr 77A=" prefix string already names the first field, so emit 77A's
    # hex right after the prefix is printed (done in build_hook before this block).
    # Here we emit the remaining labelled fields in order.
    code += puts_code(LBL_81A); code += puthex_xdata(0x081A)
    code += puts_code(LBL_819); code += puthex_xdata(0x0819)
    code += puts_code(LBL_778); code += puthex_xdata(0x0778)
    code += puts_code(LBL_779); code += puthex_xdata(0x0779)
    code += puts_code(LBL_77B); code += puthex_xdata(0x077B)
    code += puts_code(LBL_77C); code += puthex_xdata(0x077C)
    code += puts_code(LBL_77D); code += puthex_xdata(0x077D)
    code += puts_code(LBL_ED); code += puthex_xdata(0x06ED)
    code += puts_code(LBL_776); code += puthex_xdata(0x0776)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_hook():
    """push preserve; DPX=0; CHANGE-GATE on XDATA[0x0819]; if changed emit the line +
    update last-seen; pop; replay displaced bytes; ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- change gate. Read XDATA[0x0819] into SFR B (0xF0, bank-independent) so the
    # later xrl comparison is robust vs PSW RS1:RS0.
    code += mov_dptr(0x0819) + b"\xe0"             # movx a,@dptr (0x0819)
    code += b"\xf5\xf0"                            # mov B,a      (B = cur 0x0819)
    # if SEEN==0 -> force-emit (first line). Else compare cur (B) vs LAST_819.
    # The emit body is >127 bytes, so a no-change skip must be an LJMP (not SJMP).
    code += mov_dptr(SEEN) + b"\xe0"               # movx a,@dptr (SEEN)
    jz_emit = len(code)
    code += b"\x60\x00"                            # jz EMIT  (SEEN==0 -> first time)
    code += mov_dptr(LAST_819) + b"\xe0"          # movx a,@dptr (LAST_819) -> A
    code += b"\x65\xf0"                            # xrl A,B   (A ^= cur)
    jnz_emit = len(code)
    code += b"\x70\x00"                            # jnz EMIT  (changed -> over LJMP)
    # no change -> LJMP SKIP (restore). 16-bit target, always in range.
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP ; placeholder addr

    emit = len(code)
    # patch the short forward jumps to EMIT (just past the LJMP)
    code[jz_emit + 1] = (emit - (jz_emit + 2)) & 0xFF
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    # --- EMIT: update last-seen (LAST_819 = B), mark seen, print the line.
    code += mov_dptr(LAST_819) + b"\xe5\xf0\xf0"   # mov a,B ; movx @dptr,a (LAST_819=cur)
    code += mov_dptr(SEEN) + b"\x74\x01\xf0"       # SEEN=1

    code += puts_code(STR_CR)                       # "\r\n[cr 77A="
    code += puthex_xdata(0x077A)                    # 77A value
    code += build_field_block()

    skip = len(code)
    # LJMP SKIP absolute target = CAVE base + offset of the SKIP label.
    skip_abs = CAVE + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # --- restore + replay (SKIP label) ---
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += H_OLD                                  # replay LCALL 0xe391 ; MOV DPTR,#0x776
    code += b"\x22"                                # ret
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
    write_cave(body, STR_CR, b"\r\n[cr 77A=\x00", "cr str")
    write_cave(body, LBL_81A, b" 81A=\x00", "81A")
    write_cave(body, LBL_819, b" 819=\x00", "819")
    write_cave(body, LBL_778, b" 778=\x00", "778")
    write_cave(body, LBL_779, b" 779=\x00", "779")
    write_cave(body, LBL_77B, b" 77B=\x00", "77B")
    write_cave(body, LBL_77C, b" 77C=\x00", "77C")
    write_cave(body, LBL_77D, b" 77D=\x00", "77D")
    write_cave(body, LBL_ED, b" ED=\x00", "ED")
    write_cave(body, LBL_776, b" 776=\x00", "776")

    hook = build_hook()
    write_cave(body, CAVE, hook, "cr hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "a869")
    return [("cr", H_OFF, CAVE, len(hook))]


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
    for name, off, cave, hlen in info:
        print(f"  {name}: site body 0x{off:05x} -> lcall 0x{cave:04x} "
              f"(hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print(f"  displaced head (replayed verbatim): {H_OLD.hex()}")
    print("  fields: 77A 81A 819 778 779 77B 77C 77D ED 776  (all plain DPX=0 XDATA)")
    print("  change-gated on XDATA[0x0819]; hooked at cm_conn_routing_setup "
          "CODE_BANK1::a869 entry")
    print("  scratch: LAST_819=0x0B56 SEEN=0x0B57 (reused clbond cells)")


if __name__ == "__main__":
    main()
