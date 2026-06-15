#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the POST-CL0 LANE-BOND phase.

GOAL: capture the byte-level evolution of the device's lane-bond state as the
host advances SB[0xA0] (lane0) / SB[0xA1] (lane1) through 0x07 -> 0x01 -> 0x02(CL0),
so it can be diffed against the handmade firmware which STALLS at SB[0xA0]=0x01,
SB[0xA1]=0x01 (never reaches CL0=0x02) and never prints "Lane Bonded".

HOOK SITE: the entry of the state-5 lane-bond walker CODE_BANK1::8000 (body off
0x0FF6B; bank1 logical A -> body = A - 0x8000 + 0xFF6B). 8000 is dispatched ONLY
from e672 for FSM-state 5 with XDATA[0x0718]==4 (the live lane-bond walker pass),
so the hook fires exactly once per walker pass during lane bond -- the right cadence.
We displace the 6-byte head `E4 F5 21 90 21 AD` (CLR A;MOV 0x21,A;MOV DPTR,#0x21AD =
3 clean instructions) with `LCALL CAVE` + 3 NOP, dump, then REPLAY the 6 bytes and
RET so the walker runs normally.

CHANGE-GATE: the cave reads SB[0xA0] each pass and emits a line ONLY when it changed
vs the last-seen value (kept in SFR B during the pass, persisted in scratch XDATA
0x0B56 with a seen-flag in 0x0B57). The skip-when-unchanged branch is an LJMP (the
emit body is >127 B, out of SJMP range). Mirrors the handmade [s5] change-gated diag
(usb4_lanebond.h u4lb_s5_diag). A0-only gating keeps the line count tiny (~3/cycle).

CAVEAT (HW-observed): hooking the entry of the TIMING-CRITICAL state-5 walker adds
per-pass push/pop+SB-read latency (the walker iterates thousands of times/sec). Stock
still drives SB[0xA0] through 07->01->02(CL0) every cycle and the trace captures it
byte-accurate, but the added latency delays the post-CL0 tunnel link-up enough to trip
the device's own [Abrt 1] retry, so the INSTRUMENTED run loops 07->01->02->Abrt->07...
instead of latching Lane Bonded. Flash PLAIN fw_tinygrad.bin to see the full bond
(Lx:CL0 02 -> Lane Bonded -> PcieLinkUp -> *** PCIE Gen04 x04 *** -> Bus#). The
per-transition device state in the instrumented trace is identical to the bonding
cycle's (same CL-walk; the abort only affects the post-CL0 tunnel step, not A0).

LINE FORMAT (LITE, hex, no inner spaces except the group bars):
  \r\n[clb <ctr16> A0=<SBA0> A1=<SBA1> 66=<SB66> 9E=<SB9E>
       |6x=<SB6A><SB6B><SB6C><SB6D>
       |hd=<0779><077A><077B><077C><077D>
       |w1C=<081C><081D>]
(FULL block -- set LITE=False -- adds hd=0777..077F, w1C=081C..081F, st=0759..075C +
718/ED/775, and tx=2900[0..5]; use it only at a non-timing-critical site.)
Where SB[off] = DPX=1 paged XDATA 0x2800+off (handmade SB_RD); 0x2900[i] = DPX=1
paged XDATA 0x2900+i (SBTX); 0x07xx/0x08xx = DPX=0 plain XDATA.

Field meanings for the diff vs handmade u4lb_s5_diag:
  A0/A1   = SB[0xA0]/SB[0xA1] host-advanced lane FSM (07->01->02 CL0). THE watched bytes.
  66/9E   = SB[0x66] (Lane-Bonded gate, .0 host-set) / SB[0x9E] lane-event strobe.
  6x      = SB[0x6A..0x6D] = the CL config the device writes (handmade L1:CL0 01F5 = 6A/6B).
  hd      = host_desc 0x0777..0x077F (eaac mirror): 0779/077A = the route/CL snap bytes,
            077B..077D = the host route-ID tail the device needs.
  w1C     = work shadow 0x081C..0x081F (af38 echoes work[0x081C+lane] into the SB TX plane).
  st      = lb_loop1_state[0/1] (0x0759/0x075A) + lb_loop2_state[0/1] (0x075B/0x075C) =
            the per-lane CL-walk FSM cells; 718 = walker variant gate; ED = 0x06ED FSM state;
            775 = route-query response flag.
  tx      = SBTX plane 0x2900[0..5] = the device's outgoing descriptor this pass.

Build:
    python3 app/patch_clbond_trace.py fw_tinygrad.bin /tmp/fw_clbond.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_clbond.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# Hook: CODE_BANK1::8000 entry. Displace the 6-byte head (3 instructions).
H_OFF = bank1_off(0x8000)
H_OLD = bytes.fromhex("e4f5219021ad")   # CLR A;MOV 0x21,A;MOV DPTR,#0x21AD

# Caves in the low shared zero region (<0x8000, flat in both banks). The main hook
# is large (~0.5 KB of straight-line dump code), so it gets a roomy 0x6800-0x6FFF
# slot; the subroutine + strings live at 0x7000+.
CAVE = 0x6800
SHARED_DUMP = 0x7000       # paged/plain block-dump subroutine
STR_CLB = 0x7040           # "\r\n[clb "
SEP_6X = 0x7050            # "|6x="
SEP_HD = 0x7058            # "|hd="
SEP_W1C = 0x7060           # "|w1C="
SEP_ST = 0x7068            # "|st="
SEP_TX = 0x7070            # "|tx="
LBL_A0 = 0x7078            # " A0="
LBL_A1 = 0x7080            # " A1="
LBL_66 = 0x7088            # " 66="
LBL_9E = 0x7090            # " 9E="
LBL_718 = 0x7098           # " 718="
LBL_ED = 0x70A0            # " ED="
LBL_775 = 0x70A8           # " 775="
SEP_CF = 0x70B0            # "|cf=" lb_cap_field(0x072E)+sb_lane_flip(0x073E) dump

# Scratch XDATA (DPX=0). The 0x8830 region turned out NOT to be free on stock (the
# change-gate state did not persist there -> the gate fired every pass). Moved to the
# 0x0B5x headroom block which is genuinely unused on stock (free cells per the handmade
# layout: 0x0B53/0x0B54/0x0B56/0x0B57). counter at 0x0B53/0x0B54; last-A0 + seen flag
# at 0x0B56/0x0B57.
CTR_LO = 0x0B53
CTR_HI = 0x0B54
LAST_A0 = 0x0B56
LAST_A1 = 0x0B58   # unused (gate is A0-only) but kept distinct
SEEN = 0x0B57

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


# LITE mode trims the per-line payload (and the caller gates on SB[0xA0] ONLY) so the
# UART burst in the timing-critical state-5 walker path does not stall the CL-walk and
# trip the host's [Abrt 1] (observed: the FULL block aborts every bond; the lite block
# lets stock complete Lane Bonded). LITE keeps exactly the bytes that change at the
# 0x01->0x02 transition: A0/A1, 66/9E (bond gate), 6A-6D (CL cfg), hd snap+route-ID
# (0779..077D), w1C[0..1] (af38 echo shadow).
LITE = True


def build_field_block():
    """The per-line field block. DPX is 0 on entry/return."""
    code = bytearray()
    # " A0="/" A1=" lane FSM (DPX=1 SB)
    code += puts_code(LBL_A0); code += puthex_sb(0x2800 + 0xA0)
    code += puts_code(LBL_A1); code += puthex_sb(0x2800 + 0xA1)
    # " 66="/" 9E=" Lane-Bonded gate + lane-event strobe (DPX=1 SB)
    code += puts_code(LBL_66); code += puthex_sb(0x2800 + 0x66)
    code += puts_code(LBL_9E); code += puthex_sb(0x2800 + 0x9E)
    # "|6x=" SB[0x6A..0x6D] CL config (DPX=1 SB)
    code += puts_code(SEP_6X)
    for off in (0x6A, 0x6B, 0x6C, 0x6D):
        code += puthex_sb(0x2800 + off)
    if LITE:
        # "|hd=" host_desc snap+route-ID only (0x0779..0x077D, 5 bytes)
        code += puts_code(SEP_HD)
        code += dump_block(0x0779, 0x05, 0x00)
        # "|w1C=" work shadow 0x081C..0x081D (2 bytes)
        code += puts_code(SEP_W1C)
        code += dump_block(0x081C, 0x02, 0x00)
        # "|cf=" lb_cap_field 0x072E..0x073D (16) + sb_lane_flip 0x073E..0x074D (16) = 32 bytes
        code += puts_code(SEP_CF)
        code += dump_block(0x072E, 0x20, 0x00)
        code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
        return bytes(code)
    # --- FULL block (use for a non-timing-critical / post-bond capture) ---
    # "|hd=" host_desc 0x0777..0x077F (DPX=0)
    code += puts_code(SEP_HD)
    code += dump_block(0x0777, 0x09, 0x00)
    # "|w1C=" work shadow 0x081C..0x081F (DPX=0)
    code += puts_code(SEP_W1C)
    code += dump_block(0x081C, 0x04, 0x00)
    # "|st=" lb states 0x0759..0x075C + 718/ED/775 (DPX=0)
    code += puts_code(SEP_ST)
    code += dump_block(0x0759, 0x04, 0x00)
    code += puts_code(LBL_718); code += puthex_xdata(0x0718)
    code += puts_code(LBL_ED); code += puthex_xdata(0x06ED)
    code += puts_code(LBL_775); code += puthex_xdata(0x0775)
    # "|tx=" SBTX plane 0x2900[0..5] (DPX=1)
    code += puts_code(SEP_TX)
    code += dump_block(0x2900, 0x06, SB_DPX)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_hook():
    """push preserve; DPX=0; CHANGE-GATE on SB[0xA0]/SB[0xA1]; if changed emit the
    line + update last-seen; pop; replay displaced bytes; ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- change gate. Keep SB[0xA0] in SFR B (0xF0, register-bank-INDEPENDENT) so the
    # comparison via XRL A,B is robust regardless of PSW RS1:RS0 (the earlier CJNE A,R6
    # encoding used direct addr 0x06 which only equals R6 in bank 0 -> mis-gated, emitted
    # every pass). SB read needs DPX=1; XDATA scratch reads need DPX=0.
    code += bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
    code += mov_dptr(0x2800 + 0xA0) + b"\xe0"      # movx a,@dptr  (SBA0)
    code += b"\xf5\xf0"                            # mov B,a       (B = SBA0, bank-safe)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    # CHANGE GATE: fire on SB[0xA0] change (A0 carries lane0 07->01->02; A1 stays 07 on
    # single-lane bonds). Gating on A0-only keeps the print count low so the UART burst
    # does not stall the timing-critical CL-walk (A1 is still captured in the line).
    # if SEEN==0 -> force-emit (first line). Else compare A0 (B) vs LAST_A0.
    # The emit body (field block) is >127 bytes, so a no-change skip CANNOT use SJMP
    # (8-bit signed range) -- it must be an LJMP. So: short-jump OVER an `LJMP SKIP`
    # when we DO want to emit; otherwise fall into the LJMP that skips the whole body.
    code += mov_dptr(SEEN) + b"\xe0"               # movx a,@dptr (SEEN)
    jz_emit = len(code)
    code += b"\x60\x00"                            # jz EMIT  (SEEN==0 -> first time)
    code += mov_dptr(LAST_A0) + b"\xe0"            # movx a,@dptr (LAST_A0) -> A
    code += b"\x65\xf0"                            # xrl A,B   (A ^= SBA0)
    jnz_emit = len(code)
    code += b"\x70\x00"                            # jnz EMIT  (A0 changed -> over LJMP)
    # no change -> LJMP SKIP (restore). 16-bit target, always in range.
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP ; placeholder addr

    emit = len(code)
    # patch the short forward jumps to EMIT (just past the LJMP)
    code[jz_emit + 1] = (emit - (jz_emit + 2)) & 0xFF
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    # --- EMIT: update last-seen (LAST_A0 = B = SBA0), mark seen, print the line.
    code += mov_dptr(LAST_A0) + b"\xe5\xf0\xf0"    # mov a,B ; movx @dptr,a (LAST_A0=SBA0)
    code += mov_dptr(SEEN) + b"\x74\x01\xf0"       # SEEN=1

    code += puts_code(STR_CLB)                      # "\r\n[clb "
    code += emit_counter()
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
    code += H_OLD                                  # replay CLR A;MOV 0x21,A;MOV DPTR,#0x21AD
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
    write_cave(body, STR_CLB, b"\r\n[clb \x00", "clb str")
    write_cave(body, SEP_6X, b"|6x=\x00", "6x")
    write_cave(body, SEP_HD, b"|hd=\x00", "hd")
    write_cave(body, SEP_W1C, b"|w1C=\x00", "w1C")
    write_cave(body, SEP_ST, b"|st=\x00", "st")
    write_cave(body, SEP_TX, b"|tx=\x00", "tx")
    write_cave(body, LBL_A0, b" A0=\x00", "A0")
    write_cave(body, LBL_A1, b" A1=\x00", "A1")
    write_cave(body, LBL_66, b" 66=\x00", "66")
    write_cave(body, LBL_9E, b" 9E=\x00", "9E")
    write_cave(body, LBL_718, b" 718=\x00", "718")
    write_cave(body, LBL_ED, b" ED=\x00", "ED")
    write_cave(body, LBL_775, b" 775=\x00", "775")
    write_cave(body, SEP_CF, b"|cf=\x00", "cf")

    hook = build_hook()
    write_cave(body, CAVE, hook, "clb hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "8000")
    return [("clb", H_OFF, CAVE, len(hook))]


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
    if LITE:
        print("  fields(LITE): A0=SBA0 A1=SBA1 66=SB66 9E=SB9E |6x=SB6A-6D "
              "|hd=0779..077D |w1C=081C-081D")
    else:
        print("  fields(FULL): A0=SBA0 A1=SBA1 66=SB66 9E=SB9E |6x=SB6A-6D "
              "|hd=0777..077F |w1C=081C..081F |st=0759..075C 718 ED 775 |tx=2900[0..5]")
    print("  change-gated on SB[0xA0]; hooked at walker CODE_BANK1::8000 entry")
    print("  NOTE: hooking the hot state-5 walker adds per-pass latency that trips the")
    print("  device's post-CL0 [Abrt 1] retry -- the A0 07->01->02 progression is still")
    print("  captured byte-accurate each retry; flash PLAIN stock to see the full bond.")


if __name__ == "__main__":
    main()
