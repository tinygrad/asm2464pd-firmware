#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin tracer for the USB4 af38 SB-descriptor RESPONSE engine.

PURPOSE (vs app/patch_stock_cfg.py, which hooks a066/eaac):
  We need the ORDERED (desc_type -> device-reply) sequence that af38 produces so we
  can diff it against the handmade af38. The eaac hook reads a FIXED plane 0x2a00
  and fires at eaac-ENTRY (before af38 builds its 0x2900 reply), so it can NOT show
  the device reply, and it dumps the WRONG plane when the active port is 1 (rx plane
  = 0x2b00). This tracer hooks af38's EXIT instead, where:
    - intmem 0x50 (DAT_INTMEM_50) = the desc_type af38 just read from its rx plane
    - intmem 0x51 / 0x52         = desc_len / desc_dir(0x80)
    - plane 0x2900[0..7]         = the FULLY-BUILT device reply (the af38 TX)
    - SB[0x15]                   = the TX trigger byte (sb_tx_command_desc)
    - XDATA 0x06F1               = sb_active_plane_port (which rx plane it used)

HOOK SITE (bank1; body off = addr - 0x8000 + 0xFF6B):
  CODE_BANK1::b0af -- the af38 epilogue tail, AFTER the SB[0x15]/0x2900 reply is
  fully written. Original bytes `e4 ff 02 d5 da` = CLR A ; MOV R7,A ; LJMP 0xd5da
  (tail-call into u4lb_d5da with R7=0). We overwrite the first 3 bytes `e4 ff 02`
  with `LCALL CAVE`; bytes `d5 da` stay in place but are DEAD because the cave does
  NOT return -- it replays `CLR A ; MOV R7,A` then `LJMP 0xd5da` itself.
  body off 0x1301a. Displaced head = `e4 ff 02`.

NON-INTRUSIVENESS:
  af38 fires a few times per descriptor (not per hot-walker-pass), and we BUDGET the
  logger (~100 lines) so the UART burst is bounded. af38 runs MID-INTERRUPT, so the
  cave PUSHes/POPs every register it touches (ACC,B,DPL,DPH,PSW,R0-R7,DPX + the dump
  scratch 0x20-0x23) and the intmem cells it READS (0x50-0x53) are only read, never
  written. Stock must still enumerate the GPU (1002:7590) with this tracer.

LINE FORMAT (hex, budgeted ~100 in order):
  \r\n[af <ctr16> ty=<i50> ln=<i51> dr=<i52> po=<06F1> 15=<SB15>
       |tx=<plane 0x2900[0..7], DPX=1 paged>]
  ty   = desc_type af38 processed (the host-posted descriptor type).
  tx   = device reply af38 built on plane 0x2900 (bytes [0..7]).
  Diff target: stock's (ty -> tx) sequence vs handmade's.

Build:
    python3 app/patch_stock_af38.py fw_tinygrad.bin /tmp/fw_stockaf38.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_af38.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook site (body offset) + original bytes (replaced + replayed). ----------
# af38 epilogue tail b0af: `e4 ff 02` = CLR A ; MOV R7,A  (then LJMP 0xd5da at b0b1).
H_AF_OFF = bank1_off(0xB0AF)          # 0x1701a
H_AF_OLD = bytes.fromhex("e4ff02")
AF_TAIL_LJMP = 0xD5DA                 # the LJMP target the displaced LJMP went to

# ---- Caves in the low shared zero region (<0x8000, flat in both banks). --------
SHARED_DUMP = 0x7000       # paged/plain block-dump subroutine
STR_AF = 0x7040            # "\r\n[af "
LBL_TY = 0x7048            # " ty="
LBL_LN = 0x7050            # " ln="
LBL_DR = 0x7058            # " dr="
LBL_PO = 0x7060            # " po="
LBL_15 = 0x7068            # " 15="
SEP_TX = 0x7070            # "|tx="

CAVE_AF = 0x6800           # af38-exit hook

# Free-running 16-bit counter (XDATA, DPX=0; stock leaves 0x8830/0x8831 unused).
CTR_LO = 0x8830
CTR_HI = 0x8831

# Budget + change-gate state (0x0B5x headroom block, validated free/persistent).
AF_SEEN = 0x0B57           # budget-initialized flag
AF_BUDGET = 0x0B58         # remaining budget counter
AF_LAST = 0x0B53           # last-logged signature (ty ^ tx[4])
AF_BUDGET_INIT = 200       # log this many DISTINCT af38 exits, then go silent

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def ljmp(addr):
    return bytes([0x02, (addr >> 8) & 0xFF, addr & 0xFF])


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


def puthex_intmem(off):
    # direct read of internal RAM 0x50..0x53 into R7, then print. DPX irrelevant.
    return bytes([0xE5, off, 0xFF]) + lcall(UART_PUTHEX)   # mov a,off ; mov r7,a


# Two-tier preserve. af38 is a HOT path (called per CL-walk step); saving/restoring
# 18 regs on EVERY call breaks the bond timing (the host abandons the bond). So the
# LIGHT tier (just the gate's working regs) runs on every call; the FULL tier (R0-R7
# + dump scratch, needed only by the puts/puthex/dump_block emit code) is pushed ONLY
# when a line is actually emitted. The displaced tail `CLR A;MOV R7,A;LJMP d5da` only
# needs ACC/R7 well-defined (set in REPLAY), so PSW/DPL/DPH/DPX/B must round-trip.
PRESERVE_LIGHT = (0xE0, 0xF0, 0x82, 0x83, 0xD0, 0x93)              # ACC,B,DPL,DPH,PSW,DPX
PRESERVE_FULL = (0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,   # R0-R7
                 0x20, 0x21, 0x22, 0x23)                           # dump scratch


def build_shared_dump():
    """Print R(0x22) hex bytes from paged XDATA (DPH:DPL = 0x21:0x20) at plane A.
    Re-asserts the plane each iteration (puthex restores DPX=0). Plane saved in
    0x23. Caller sets 0x20/0x21/0x22 + A=plane, then LCALLs here."""
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
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"      # movx a,@dptr ; inc a ; movx @dptr,a
    code += b"\x70\x05"                             # jnz +5 (skip hi inc if lo!=0)
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"      # inc hi
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


def build_af_field_block():
    """The [af] per-line field block. DPX is 0 on entry/return."""
    code = bytearray()
    # " ty="/" ln="/" dr=" -- intmem 0x50/0x51/0x52 (desc_type/len/dir)
    code += puts_code(LBL_TY); code += puthex_intmem(0x50)
    code += puts_code(LBL_LN); code += puthex_intmem(0x51)
    code += puts_code(LBL_DR); code += puthex_intmem(0x52)
    # " po=" active port 0x06F1 (DPX=0)
    code += puts_code(LBL_PO); code += puthex_xdata(0x06F1)
    # " 15=" SB[0x15] TX trigger (DPX=1 SB)
    code += puts_code(LBL_15); code += puthex_sb(0x2800 + 0x15)
    # "|tx=" device reply plane 0x2900[0..7] (DPX=1 paged)
    code += puts_code(SEP_TX)
    code += dump_block(0x2900, 0x08, SB_DPX)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_af_hook():
    """push preserve; DPX=0; BUDGETED logger; pop; replay `CLR A ; MOV R7,A`;
    LJMP 0xd5da (the displaced tail-call -- the cave never returns).

    Replay note: the displaced 3 bytes are `e4 ff 02` = CLR A ; MOV R7,A ; <0x02>,
    where 0x02 was the LJMP opcode whose operand (d5da) STAYS in place at b0b2. So
    we replay only `e4 ff` then issue a full `LJMP 0xd5da` ourselves -- never RET."""
    code = bytearray()
    # ---- ULTRA-LIGHT pre-gate: read desc_type (intmem 0x50, direct, no DPX/paged)
    # and bail with ZERO state touched for the HOT CL-walk types 0x0C and 0x0D. The
    # 0D CL-walk is the bond-timing-critical hot path; engaging the hook there (even
    # lightly) made the host abandon the bond in ~18 runs. So we skip BOTH 0x0C and
    # 0x0D and only log the post-bond CM command descriptors (08/09/01/12). Only
    # ACC+PSW are saved here (the CJNE/SUBB clobber both). ----
    code += bytes([0xC0, 0xE0, 0xC0, 0xD0])        # push ACC ; push PSW
    code += bytes([0xE5, 0x50])                    # mov a,0x50  (desc_type)
    # if (desc_type == 0x0C) -> skip
    cjne_0c = len(code)
    code += bytes([0xB4, 0x0C, 0x00])              # cjne a,#0x0C, chk0d  (-> over LJMP)
    ljmp_skip_pre = len(code)
    code += b"\x02\x00\x00"                        # ljmp REPLAY_PRE (==0x0C: skip)
    chk0d = len(code)
    code[cjne_0c + 2] = (chk0d - (cjne_0c + 3)) & 0xFF
    # if (desc_type == 0x0D) -> skip  (A still = desc_type; cjne preserves A)
    cjne_0d = len(code)
    code += bytes([0xB4, 0x0D, 0x00])              # cjne a,#0x0D, proceed
    ljmp_skip_pre2 = len(code)
    code += b"\x02\x00\x00"                        # ljmp REPLAY_PRE (==0x0D: skip)
    proceed = len(code)
    code[cjne_0d + 2] = (proceed - (cjne_0d + 3)) & 0xFF

    # ---- LIGHT tier: remaining gate save (only reached for non-0C descriptors). ----
    for d in (0xF0, 0x82, 0x83, 0x93):             # push B,DPL,DPH,DPX (ACC/PSW already)
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # budget init (once): if AF_SEEN==0 -> AF_BUDGET=INIT, AF_SEEN=1.
    code += mov_dptr(AF_SEEN) + b"\xe0"            # movx a,@dptr (SEEN)
    jnz_have = len(code)
    code += b"\x70\x00"
    code += mov_dptr(AF_BUDGET) + bytes([0x74, AF_BUDGET_INIT, 0xF0])
    code += mov_dptr(AF_SEEN) + b"\x74\x01\xf0"
    have = len(code)
    code[jnz_have + 1] = (have - (jnz_have + 2)) & 0xFF

    # CHANGE-GATE: sig = intmem0x50(desc_type) ^ plane2900[4](TX CL byte). If sig ==
    # AF_LAST -> skip (no budget spend, no line) -> collapses repetitive non-0C polls.
    code += bytes([0x85, 0x50, 0xF0])              # mov B,0x50  (B = desc_type)
    code += bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
    code += mov_dptr(0x2904) + b"\xe0"             # movx a,@dptr (plane 0x2900[4])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += b"\x62\xf0"                            # xrl B,a   (B = ty ^ tx4)
    code += mov_dptr(AF_LAST) + b"\xe0"            # movx a,@dptr (AF_LAST) -> A
    code += b"\x65\xf0"                            # xrl A,B   (A = LAST ^ sig)
    jnz_changed = len(code)
    code += b"\x70\x00"                            # jnz CHANGED
    ljmp_skip0 = len(code)
    code += b"\x02\x00\x00"                        # ljmp REPLAY (unchanged -> skip)
    changed = len(code)
    code[jnz_changed + 1] = (changed - (jnz_changed + 2)) & 0xFF
    code += mov_dptr(AF_LAST) + b"\xe5\xf0\xf0"    # AF_LAST = sig (B)

    # budget check
    code += mov_dptr(AF_BUDGET) + b"\xe0"          # A = BUDGET
    jnz_emit = len(code)
    code += b"\x70\x00"
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp REPLAY
    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF
    code += b"\x14\xf0"                            # dec a ; movx @dptr,a (BUDGET--)

    # ---- FULL tier: push R0-R7 + dump scratch ONLY on the emit path. ----
    for d in PRESERVE_FULL:
        code += bytes([0xC0, d])                   # push direct
    code += puts_code(STR_AF)
    code += emit_counter()
    code += build_af_field_block()
    for d in reversed(PRESERVE_FULL):
        code += bytes([0xD0, d])                   # pop direct

    # ---- REPLAY (LIGHT skip target): pop DPX,DPH,DPL,B then PSW,ACC. The pre-gate's
    # 0x0C path uses REPLAY_PRE instead (only ACC+PSW on its stack). ----
    skip = len(code)
    skip_abs = CAVE_AF + skip
    for jp in (ljmp_skip0, ljmp_skip):
        code[jp + 1] = (skip_abs >> 8) & 0xFF
        code[jp + 2] = skip_abs & 0xFF
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in (0x93, 0x83, 0x82, 0xF0):             # pop DPX,DPH,DPL,B (reverse of light push)
        code += bytes([0xD0, d])
    # fall through into the shared PSW/ACC pop + tail.
    code += bytes([0xD0, 0xD0, 0xD0, 0xE0])        # pop PSW ; pop ACC
    tail_after_acc = len(code)
    code += b"\xe4\xff"                            # CLR A ; MOV R7,A  (d5da arg)
    code += ljmp(AF_TAIL_LJMP)                     # LJMP 0xd5da  (never returns)

    # ---- REPLAY_PRE (0x0C/0x0D pre-gate skip targets): only ACC+PSW were pushed. Pop
    # them and jump to the shared CLR A;MOV R7,A;LJMP d5da tail. ----
    pre_abs = CAVE_AF + len(code)
    for jp in (ljmp_skip_pre, ljmp_skip_pre2):
        code[jp + 1] = (pre_abs >> 8) & 0xFF
        code[jp + 2] = pre_abs & 0xFF
    code += bytes([0xD0, 0xD0, 0xD0, 0xE0])        # pop PSW ; pop ACC
    # LJMP to the shared tail (CLR A;MOV R7,A;LJMP d5da) at tail_after_acc.
    code += ljmp(CAVE_AF + tail_after_acc)
    return bytes(code)


# ---- wrap / unwrap / patch plumbing ------------------------------------------
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
    write_cave(body, STR_AF, b"\r\n[af \x00", "af str")
    write_cave(body, LBL_TY, b" ty=\x00", "ty")
    write_cave(body, LBL_LN, b" ln=\x00", "ln")
    write_cave(body, LBL_DR, b" dr=\x00", "dr")
    write_cave(body, LBL_PO, b" po=\x00", "po")
    write_cave(body, LBL_15, b" 15=\x00", "15")
    write_cave(body, SEP_TX, b"|tx=\x00", "tx")

    af_hook = build_af_hook()
    write_cave(body, CAVE_AF, af_hook, "af hook")
    patch_site(body, H_AF_OFF, H_AF_OLD, CAVE_AF, "af38-exit")

    return [("af", H_AF_OFF, CAVE_AF, len(af_hook), H_AF_OLD)]


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
    for name, off, cave, hlen, old in info:
        print(f"  {name}: site body 0x{off:05x} (displaced {old.hex()}) -> "
              f"lcall 0x{cave:04x} (hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print(f"  [af ] BUDGETED ({AF_BUDGET_INIT} af38 exits, in order); "
          "hooked at CODE_BANK1::b0af (af38 epilogue, after SB[0x15]/0x2900 reply)")
    print("        fields: ty=intmem0x50(desc_type) ln=0x51 dr=0x52 po=06F1 "
          "15=SB15 |tx=2900[0..7](device reply)")
    print("  NON-INTRUSIVE: budgeted; stock must still enumerate the GPU (1002:7590).")


if __name__ == "__main__":
    main()
