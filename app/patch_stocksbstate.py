#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the device-side SB-transport state at
the two moments that matter for the handmade-vs-stock USB4-engage diff:

  (1) ROUTE-QUERY SEND  -- stock d5da's SB[0x10]=1 TX-go (body 0x15564, the 7-byte
      `79 10 74 01 12 0b e6`, same hook as app/patch_txtrace.py). Tagged [p15 ...].
      This is the route-query the host samples; we want stock's PRE-ENGAGE state.

  (2) eaac ENTRY  -- CODE_BANK1::eaac (body 0x16a17, `90 07 75` MOV DPTR,#0x775).
      eaac is the block-copy that fills 0x0777 from the host's posted routing
      descriptor in SB-plane-2 (0x2a00). It runs ONLY when the host has actually
      posted, so [E ...] firing AT ALL == host engaged. We dump it BEFORE the copy
      so 0x2a00 / SB[0x18] / SB[0x28] are the HOST descriptor source bytes.

BOTH lines dump the SAME register set as handmade's [TX] pre-go dump
(handmade/src/usb4_lanebond.h ~line 134), PLUS the link/PHY regs E302/E751/E764/
C8FF, so stock's pre-engage state is byte-comparable against handmade's stuck:
  SB15=05 SB10=00 SB2C=03 SB0C=08 SB0D=00 SB0E=00 SB18=00 SB28=00
  | SBTX=0C03000000000000 | 0x2a00=all-zero
  | 0719=00 0758=10 06ED=03 06EE=00 0775=00 0776=01 0777=55
  | CCD9=00 C80A=20 CC37=2C CA60=56
  | P1[0109]=72 SB[D8]=00 SB[3A-3D]=24,03,06,00
  | lanes SB[A0]=07 SB[A1]=07   (extras: E302 E751 E764 C8FF)

A free-running 16-bit counter (XDATA 0x8830:0x8831 -- free handmade-IRAM headroom
region, but on STOCK this XDATA is unused scratch) is printed first on every line
so the ORDER of route-queries vs eaac fires is visible.

ADDRESSING (ghidra.c == this image; body=data[4:-6]):
  bank1 logical A -> body off  A - 0x8000 + 0xFF6B   (verified vs a0e7 anchor).
  SB[off]   = DPX=1 paged XDATA 0x2800+off      (handmade SB_RD).
  SBTX[i]   = DPX=1 paged XDATA 0x2900+i        (handmade SBTX_RD).
  0x2a00[i] = DPX=1 paged XDATA 0x2a00+i        (handmade plane-2 read).
  P1[off]   = DPX=1 paged XDATA 0x0000+off      (handmade P1_RD; so P1[0x0109]
              is DPX=1 at 0x0109, NOT DPX=0).
  PR(addr)  = DPX=0 plain XDATA                 (gating + HW + link regs).
Caves live in the low shared zero region (<0x8000), flat in both banks; the
UART helpers 0x538D puts / 0x51C7 puthex are <0x8000 too.

LINE FORMATS (hex, no spaces inside groups except where shown):
  \r\n[p15 <ctr16> S=<15><10><2C><0C><0D><0E><18><28>|TX=<2900[0..7]>|
       2a=<2a00[0..7]>|X=<0719><0758><06ED><06EE><0775><0776><0777>|
       H=<CCD9><C80A><CC37><CA60>|D=<P1.0109><SBD8><SB3A><SB3B><SB3C><SB3D>|
       L=<SBA0><SBA1><E302><E751><E764><C8FF>]
  \r\n[E  <ctr16> S=...|TX=...|2a=...|X=...|H=...|D=...|L=...]
(identical field set; the only difference is the tag p15 vs E, so the two hooks
diff line-for-line.)

Build:
    python3 app/patch_stocksbstate.py fw_tinygrad.bin /tmp/fw_stocksbstate.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_stocksbstate.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook sites (body offsets) + original bytes (replaced + replayed). --------
# 1) d5da SB[0x10]=1 TX-go write: `79 10 74 01 12 0b e6` (7 bytes), body 0x15564.
H_TX_OFF = 0x15564
H_TX_OLD = bytes.fromhex("79107401120be6")
# 2) eaac entry: MOV DPTR,#0x775 (3 bytes).
H_EAAC_OFF = bank1_off(0xEAAC)
H_EAAC_OLD = bytes.fromhex("900775")

# ---- Caves in the low shared zero region (0x5E00..0x8000, flat both banks). ---
SHARED_DUMP = 0x5E00         # paged-block dump subroutine (~34 B)
STR_P15 = 0x5E40             # "\r\n[p15 "
STR_E = 0x5E50               # "\r\n[E  "
STR_S = 0x5E58               # "|S="  (after counter)  -> we inline the bars
SEP_TX = 0x5E60              # "|TX="
SEP_2A = 0x5E68              # "|2a="
SEP_X = 0x5E70               # "|X="
SEP_H = 0x5E78               # "|H="
SEP_D = 0x5E80               # "|D="
SEP_L = 0x5E88               # "|L="
CAVE_TX = 0x5F00
CAVE_EAAC = 0x6200

# Free-running counter (XDATA, DPX=0). Stock leaves 0x8830/0x8831 unused; this is
# scratch SRAM. Incremented once per line so route-query/eaac ordering is visible.
CTR_LO = 0x8830
CTR_HI = 0x8831

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
    # DPX=0 plain XDATA read into R7, then print. Assumes DPX already 0.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_sb(addr):
    # DPX=1 paged read of XDATA `addr` (page-1 plane), restore DPX=0, then print.
    return (
        bytes([0x75, 0x93, SB_DPX])               # mov DPX,#1
        + mov_dptr(addr) + b"\xe0\xff"            # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])               # mov DPX,#0
        + lcall(UART_PUTHEX)
    )


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the dump scratch (0x20/0x21/0x22/0x23).
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x23, 0x93)


def build_shared_dump():
    """Print R(0x22) hex bytes from paged XDATA (DPH:DPL = 0x21:0x20) at plane A.
    Re-asserts the plane each iteration (puthex restores DPX=0). Plane saved in
    0x23. Caller sets 0x20/0x21/0x22 + A=plane, then LCALLs here.
    """
    code = bytearray()
    code += bytes([0xF5, 0x23])                    # mov 0x23,a   (save plane)
    loop = len(code)
    code += bytes([0xE5, 0x23, 0xF5, 0x93])        # mov a,0x23 ; mov DPX,a
    code += b"\xe5\x20\xf5\x82"                     # mov a,0x20 ; mov DPL,a
    code += b"\xe5\x21\xf5\x83"                     # mov a,0x21 ; mov DPH,a
    code += b"\xe0"                                 # movx a,@dptr   (paged read)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0     (restore)
    code += b"\xff" + lcall(UART_PUTHEX)           # mov r7,a ; lcall puthex
    code += b"\x05\x20"                             # inc 0x20
    code += b"\xe5\x20"                             # mov a,0x20
    jnz = len(code)
    code += b"\x70\x00"                             # jnz no_carry
    code += b"\x05\x21"                             # inc 0x21
    no_carry = len(code)
    code[jnz + 1] = (no_carry - (jnz + 2)) & 0xFF
    djnz = len(code)
    code += b"\xd5\x22\x00"                         # djnz 0x22, loop
    code[djnz + 2] = (loop - (djnz + 3)) & 0xFF
    code += b"\x22"                                 # ret
    return bytes(code)


def dump_block(start, count, dpx):
    """Call the shared dump subroutine for `count` bytes from `start` at `dpx`."""
    return (
        bytes([0x75, 0x20, start & 0xFF])          # mov 0x20,#lo
        + bytes([0x75, 0x21, (start >> 8) & 0xFF]) # mov 0x21,#hi
        + bytes([0x75, 0x22, count & 0xFF])        # mov 0x22,#count
        + bytes([0x74, dpx])                       # mov a,#dpx
        + lcall(SHARED_DUMP)
    )


def emit_counter():
    """ctr16++ then print it (hi then lo). DPX assumed 0."""
    code = bytearray()
    # inc 16-bit counter at CTR_LO/CTR_HI
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"      # movx a,@dptr ; inc a ; movx @dptr,a
    code += b"\x70\x05"                             # jnz +5 (skip hi inc if lo!=0)
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"      # inc hi
    # print hi then lo
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


# The common field block (same for both hooks). DPX is 0 on entry/return.
def build_field_block():
    code = bytearray()
    # "|S=" then SB control bytes (DPX=1)
    code += puts_code(STR_S)
    for off in (0x15, 0x10, 0x2C, 0x0C, 0x0D, 0x0E, 0x18, 0x28):
        code += puthex_sb(0x2800 + off)
    # "|TX=" SBTX plane 0x2900[0..7]
    code += puts_code(SEP_TX)
    code += dump_block(0x2900, 0x08, SB_DPX)
    # "|2a=" plane-2 0x2a00[0..7] (host descriptor source)
    code += puts_code(SEP_2A)
    code += dump_block(0x2A00, 0x08, SB_DPX)
    # "|X=" FSM/gating XDATA (DPX=0)
    code += puts_code(SEP_X)
    for a in (0x0719, 0x0758, 0x06ED, 0x06EE, 0x0775, 0x0776, 0x0777):
        code += puthex_xdata(a)
    # "|H=" HW regs (DPX=0)
    code += puts_code(SEP_H)
    for a in (0xCCD9, 0xC80A, 0xCC37, 0xCA60):
        code += puthex_xdata(a)
    # "|D=" diag: P1[0x0109] (DPX=1!), SB[0xD8], SB[0x3A..0x3D] (DPX=1)
    code += puts_code(SEP_D)
    code += puthex_sb(0x0109)            # P1[0x0109] = DPX=1 at 0x0109
    code += puthex_sb(0x2800 + 0xD8)
    for off in (0x3A, 0x3B, 0x3C, 0x3D):
        code += puthex_sb(0x2800 + off)
    # "|L=" link/lane: SB[0xA0],SB[0xA1] (DPX=1) then E302/E751/E764/C8FF (DPX=0)
    code += puts_code(SEP_L)
    code += puthex_sb(0x2800 + 0xA0)
    code += puthex_sb(0x2800 + 0xA1)
    for a in (0xE302, 0xE751, 0xE764, 0xC8FF):
        code += puthex_xdata(a)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def _wrap_hook(prefix_str_addr, old_bytes):
    """push preserve; DPX=0; \r\n[tag <ctr>; field block; pop; replay old; ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += puts_code(prefix_str_addr)             # "\r\n[p15 " or "\r\n[E  "
    code += emit_counter()
    code += build_field_block()
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += old_bytes                              # replay replaced bytes
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
    write_cave(body, SHARED_DUMP, build_shared_dump(), "shared dump")
    write_cave(body, STR_P15, b"\r\n[p15 \x00", "p15 str")
    write_cave(body, STR_E, b"\r\n[E  \x00", "E str")
    write_cave(body, STR_S, b"|S=\x00", "S")
    write_cave(body, SEP_TX, b"|TX=\x00", "TX")
    write_cave(body, SEP_2A, b"|2a=\x00", "2a")
    write_cave(body, SEP_X, b"|X=\x00", "X")
    write_cave(body, SEP_H, b"|H=\x00", "H")
    write_cave(body, SEP_D, b"|D=\x00", "D")
    write_cave(body, SEP_L, b"|L=\x00", "L")

    hooks = [
        ("p15", H_TX_OFF, H_TX_OLD, CAVE_TX, STR_P15),
        ("eaac", H_EAAC_OFF, H_EAAC_OLD, CAVE_EAAC, STR_E),
    ]
    info = []
    for name, off, old, cave, prefix in hooks:
        hook = _wrap_hook(prefix, old)
        write_cave(body, cave, hook, f"{name} hook")
        patch_site(body, off, old, cave, name)
        info.append((name, off, cave, len(hook)))
    return info


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
    print("  fields: |S=15/10/2C/0C/0D/0E/18/28 |TX=2900[0..7] |2a=2a00[0..7] "
          "|X=0719/0758/06ED/06EE/0775/0776/0777 |H=CCD9/C80A/CC37/CA60 "
          "|D=P1.0109/SBD8/SB3A-3D |L=SBA0/SBA1/E302/E751/E764/C8FF")


if __name__ == "__main__":
    main()
