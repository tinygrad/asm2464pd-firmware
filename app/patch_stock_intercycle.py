#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the USB4 LANE-BOND FINALIZE phase --
NON-INTRUSIVE: hooks ONLY low-frequency INTERRUPT/EVENT handlers, so the bond
still completes (stock must still print "Lane Bonded" and "[PcieTunnel-Deassert]").

WHY THIS EXISTS (vs app/patch_clbond_trace.py):
  patch_clbond_trace.py hooks the HOT state-5 walker CODE_BANK1::8000, which is
  called thousands of times/sec. Its per-pass push/pop + SB-read + UART latency
  delays the post-CL0 tunnel link-up enough to trip the device's own [Abrt 1]
  retry -- the INSTRUMENTED run loops 07->01->02->Abrt->07... and NEVER latches
  Lane Bonded. This tracer instead hooks two LOW-FREQUENCY handlers that fire only
  on discrete lane events (a few times per bond), so the added latency is
  negligible and the bond runs to completion. Each hook is additionally
  change-gated so it emits at most one line per meaningful state change.

HOOK SITES (both bank1; body off = addr - 0x8000 + 0xFF6B):
  1) CODE_BANK1::a066 ENTRY -- the SB connect / lane-bond INT1 handler. Fires on
     discrete SB lane events (connect, CL0, bond, abort). This is where SB[0x66]
     (.0 = Lane-Bonded host-set, .2 = Abr2) is processed. Tagged [a66 ...],
     change-gated on SB[0x66] (paged XDATA 0x2866).
     body off 0x11fd1. Displaced head = `e4 f5 4d`
       (CLR A ; MOV 0x4d,A -- 2 clean instructions, 3 bytes).
  2) CODE_BANK1::eaac ENTRY -- the block-copy that fills XDATA[0x0777..] from the
     host-posted routing descriptor in SB plane-2 (paged XDATA 0x2a00). This is
     where 0x077A is mirrored from the host. Tagged [ea ...], change-gated on
     XDATA[0x077A].
     body off 0x16a17. Displaced head = `90 07 75`
       (MOV DPTR,#0x775 -- 1 clean instruction, 3 bytes).
  Both displaced regions are EXACTLY 3 bytes = LCALL size, so no NOP padding; the
  cave replays the displaced bytes byte-exact then RETs.

NON-INTRUSIVENESS GUARANTEES:
  - Hooks fire only a handful of times per bond (interrupt/event handlers), NOT
    per walker pass -- the dominant latency source (the hot 8000 path) is
    untouched.
  - Each cave change-gates: a066 on SB[0x66], eaac on XDATA[0x077A]; a line is
    emitted only when that watched byte changed vs the last-seen value. So even
    if a handler is re-entered, the UART burst happens at most once per change.
  - Both a066 and eaac run MID-INTERRUPT, so each cave PUSHes/POPs ACC,B,DPL,DPH,
    PSW,R0-R7,DPX and the dump scratch (0x20-0x23). The interrupt handler's
    register state is preserved exactly -- corrupting it would break the bond.

LINE FORMATS (hex, no inner spaces except the group bars; kept SHORT -- UART is
the main latency source):
  [a66] (change-gated on SB[0x66]; SB[off] = DPX=1 paged XDATA 0x2800+off;
         0x07xx/0x08xx = DPX=0 plain XDATA):
    \r\n[a66 <ctr16> 66=<SB66> 9E=<SB9E> A0=<SBA0> A1=<SBA1> D4=<SBD4>
         64=<SB64> 2C=<SB2C> 2D=<SB2D>
         |hd=<0779><077A><077B>
         |cl=<0761><0762>
         |19=<0819><081A>]
  [ea] (change-gated on XDATA[0x077A]):
    \r\n[ea  <ctr16> 77A=<077A> 779=<0779>
         |2a=<plane 0x2a00[0..7], DPX=1 paged>
         |6f0=<06F0 active-port>]

FIELD MEANINGS (diff target = stock walks 66.0->Lane-Bonded; handmade stalls):
  66/9E   = SB[0x66] Lane-Bonded gate (.0 host-set, .2 Abr2) / SB[0x9E] lane-event
            strobe. 66 is the a066 change-gate -- the bond-finalize watched byte.
  A0/A1   = SB[0xA0]/SB[0xA1] host-advanced lane FSM (07->01->02 CL0).
  D4/64   = SB[0xD4] (.5 set by a066 on bond confirm) / SB[0x64] (.0 confirm input).
  2C/2D   = SB[0x2C]/SB[0x2D] route/status bytes.
  hd      = host_desc 0x0779/0x077A/0x077B (eaac mirror of the host route/CL snap).
  cl      = lb CL cells 0x0761/0x0762.
  19      = lane-advertise mask 0x0819/0x081A (0x0819.0/.1 = lane0/lane1 advertised).
  77A/779 = the eaac mirror bytes themselves (077A is the eaac change-gate).
  2a      = SB plane-2 0x2a00[0..7] = the host-posted descriptor eaac copies from.
  6f0     = 0x06F0 active-port selector (which plane the descriptor was read from).

SCRATCH XDATA (DPX=0):
  Counter  0x8830/0x8831  -- free-running 16-bit, stock leaves these unused (same
                            cells as app/patch_stocksbstate.py).
  a066 gate 0x0B56 (last SB[0x66]) / 0x0B57 (seen flag) -- the 0x0B5x headroom
                            block that app/patch_clbond_trace.py validated as
                            genuinely free / persistent on STOCK.
  eaac gate 0x0B58 (last XDATA[0x077A]) / 0x0B59 (seen flag).

Build:
    python3 app/patch_finalize_trace.py fw_tinygrad.bin /tmp/fw_finalize.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_finalize.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook sites (body offsets) + original bytes (replaced + replayed). --------
# 1) a066 entry: `e4 f5 4d` = CLR A ; MOV 0x4d,A (2 instructions, 3 bytes).
H_A66_OFF = bank1_off(0xA066)        # 0x11fd1
H_A66_OLD = bytes.fromhex("e4f54d")
# 2) eaac entry: `90 07 75` = MOV DPTR,#0x775 (1 instruction, 3 bytes).
H_EAAC_OFF = bank1_off(0xEAAC)       # 0x16a17
H_EAAC_OLD = bytes.fromhex("900775")

# ---- Caves in the low shared zero region (<0x8000, flat in both banks). The UART
# helpers 0x538D puts / 0x51C7 puthex are <0x8000 too. The shared dump subroutine +
# strings live at 0x7000+; the two hooks each get a roomy slot below it.
SHARED_DUMP = 0x7000       # paged/plain block-dump subroutine (~34 B)
STR_A66 = 0x7040           # "\r\n[a66 "
STR_EA = 0x7050            # "\r\n[ea  "
LBL_66 = 0x7058            # " 66="
LBL_9E = 0x7060            # " 9E="
LBL_A0 = 0x7068            # " A0="
LBL_A1 = 0x7070            # " A1="
LBL_D4 = 0x7078            # " D4="
LBL_64 = 0x7080            # " 64="
LBL_2C = 0x7088            # " 2C="
LBL_2D = 0x7090            # " 2D="
SEP_HD = 0x7098            # "|hd="
SEP_CL = 0x70A0            # "|cl="
SEP_19 = 0x70A8            # "|19="
LBL_77A = 0x70B0           # " 77A="
LBL_779 = 0x70B8           # " 779="
SEP_2A = 0x70C0            # "|2a="
SEP_6F0 = 0x70C8           # "|6f0="

CAVE_A66 = 0x6800          # a066 hook (the larger of the two)
CAVE_EA = 0x6C00           # eaac hook

# Free-running 16-bit counter (XDATA, DPX=0). Stock leaves 0x8830/0x8831 unused;
# this is scratch SRAM (same cells as app/patch_stocksbstate.py). Shared by both
# hooks so the ORDER of a066 vs eaac fires is visible.
CTR_LO = 0x8830
CTR_HI = 0x8831

# Change-gate state. The 0x0B5x headroom block is genuinely unused / persistent on
# STOCK (validated by app/patch_clbond_trace.py, which uses 0x0B53/0x0B54/0x0B56/
# 0x0B57). a066 gate at 0x0B56/0x0B57; eaac gate at 0x0B58/0x0B59 -- distinct cells
# so the two gates never clobber each other.
A66_LAST = 0x0B56          # last-seen SB[0x66]
A66_SEEN = 0x0B57          # a066 seen flag
EA_LAST = 0x0B58           # last-seen XDATA[0x077A] (unused in postbond budget variant)
EA_SEEN = 0x0B59           # eaac seeded flag (0 -> seed budget on first call)
EA_BUDGET = 0x0B5A         # eaac decrementing budget (postbond variant)
EA_BUDGET_N = 0xFF         # intercycle: log the first 255 eaac calls (span both CL0 cycles)

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
# a066/eaac run MID-INTERRUPT, so every register the cave touches must be restored.
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


# ---- a066 hook --------------------------------------------------------------
def build_a66_field_block():
    """The [a66] per-line field block. DPX is 0 on entry/return."""
    code = bytearray()
    # " 66="/" 9E=" bond gate + lane-event strobe (DPX=1 SB)
    code += puts_code(LBL_66); code += puthex_sb(0x2800 + 0x66)
    code += puts_code(LBL_9E); code += puthex_sb(0x2800 + 0x9E)
    # " A0="/" A1=" lane FSM (DPX=1 SB)
    code += puts_code(LBL_A0); code += puthex_sb(0x2800 + 0xA0)
    code += puts_code(LBL_A1); code += puthex_sb(0x2800 + 0xA1)
    # " D4="/" 64=" bond confirm out/in (DPX=1 SB)
    code += puts_code(LBL_D4); code += puthex_sb(0x2800 + 0xD4)
    code += puts_code(LBL_64); code += puthex_sb(0x2800 + 0x64)
    # " 2C="/" 2D=" route/status (DPX=1 SB)
    code += puts_code(LBL_2C); code += puthex_sb(0x2800 + 0x2C)
    code += puts_code(LBL_2D); code += puthex_sb(0x2800 + 0x2D)
    # "|hd=" host_desc 0x0779..0x077B (3 bytes, DPX=0)
    code += puts_code(SEP_HD)
    code += dump_block(0x0779, 0x03, 0x00)
    # "|cl=" lb CL cells 0x0761..0x0762 (2 bytes, DPX=0)
    code += puts_code(SEP_CL)
    code += dump_block(0x0761, 0x02, 0x00)
    # "|19=" lane-advertise mask 0x0819..0x081A (2 bytes, DPX=0)
    code += puts_code(SEP_19)
    code += dump_block(0x0819, 0x02, 0x00)
    # |CFG (reuse "|19=" marker) then the 5 connect-time config-space ranges (DPX=1), 0x20 bytes each,
    # order 0x1200,0x1400,0x1500,0x1600,0x1800 -> diff against handmade [CFGxxxx=] (160 hex chars).
    code += puts_code(SEP_19)
    code += dump_block(0x1200, 0x20, 0x01)
    code += dump_block(0x1400, 0x20, 0x01)
    code += dump_block(0x1500, 0x20, 0x01)
    code += dump_block(0x1600, 0x20, 0x01)
    code += dump_block(0x1800, 0x20, 0x01)
    # transport state (DPX=0): E302 link-mode, EA90 in-band-staging, EC06 router-op evt, c80a INT src
    code += puts_code(SEP_19)
    code += puthex_xdata(0xE302)
    code += puthex_xdata(0xEA90)
    code += puthex_xdata(0xEC06)
    code += puthex_xdata(0xC80A)
    # link-mode/width state: CA06 (mode-next), C00E, E710 (width), and the width-event flag P1[0x1203]
    code += puthex_xdata(0xCA06)
    code += puthex_xdata(0xC00E)
    code += puthex_xdata(0xE710)
    code += puthex_sb(0x1203)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_a66_hook():
    """push preserve; DPX=0; CHANGE-GATE on SB[0x66]; if changed emit the line +
    update last-seen; pop; replay displaced bytes; ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- change gate. Keep SB[0x66] in SFR B (0xF0, register-bank-INDEPENDENT) so
    # the compare via XRL A,B is robust regardless of PSW RS1:RS0. SB read needs
    # DPX=1; XDATA scratch reads need DPX=0.
    code += bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
    code += mov_dptr(0x2800 + 0x66) + b"\xe0"      # movx a,@dptr  (SB66)
    code += b"\xf5\xf0"                            # mov B,a       (B = SB66, bank-safe)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    # if SEEN==0 -> force-emit (first line). Else compare SB66 (B) vs A66_LAST.
    # The emit body is >127 bytes, so the no-change skip CANNOT use SJMP -- it must
    # be an LJMP. So: short-jump OVER an `LJMP SKIP` when we DO want to emit;
    # otherwise fall into the LJMP that skips the whole body.
    code += mov_dptr(A66_SEEN) + b"\xe0"           # movx a,@dptr (SEEN)
    jz_emit = len(code)
    code += b"\x60\x00"                            # jz EMIT  (SEEN==0 -> first time)
    code += mov_dptr(A66_LAST) + b"\xe0"           # movx a,@dptr (LAST) -> A
    code += b"\x65\xf0"                            # xrl A,B   (A ^= SB66)
    jnz_emit = len(code)
    code += b"\x70\x00"                            # jnz EMIT  (66 changed -> over LJMP)
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP ; placeholder addr

    emit = len(code)
    code[jz_emit + 1] = (emit - (jz_emit + 2)) & 0xFF
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    # --- EMIT: update last-seen (A66_LAST = B = SB66), mark seen, print the line.
    code += mov_dptr(A66_LAST) + b"\xe5\xf0\xf0"   # mov a,B ; movx @dptr,a (LAST=SB66)
    code += mov_dptr(A66_SEEN) + b"\x74\x01\xf0"   # SEEN=1

    code += puts_code(STR_A66)                      # "\r\n[a66 "
    code += emit_counter()
    code += build_a66_field_block()

    skip = len(code)
    skip_abs = CAVE_A66 + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # --- restore + replay (SKIP label) ---
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += H_A66_OLD                              # replay CLR A ; MOV 0x4d,A
    code += b"\x22"                                # ret
    return bytes(code)


# ---- eaac hook --------------------------------------------------------------
def build_ea_field_block():
    """The [ea] per-line field block. DPX is 0 on entry/return."""
    code = bytearray()
    # " 77A="/" 779=" eaac mirror bytes (DPX=0)
    code += puts_code(LBL_77A); code += puthex_xdata(0x077A)
    code += puts_code(LBL_779); code += puthex_xdata(0x0779)
    # "|2a=" host descriptor source plane 0x2a00[0..7] (DPX=1 paged)
    code += puts_code(SEP_2A)
    code += dump_block(0x2A00, 0x08, SB_DPX)
    # "|6f0=" active-port selector 0x06F0 (DPX=0)
    code += puts_code(SEP_6F0)
    code += puthex_xdata(0x06F0)
    # SB lane-confirm regs the host reads to decide the pairing (DPX=1 SB 0x2800+off),
    # matched to handmade's [ea] |9E=..64=..A=..D4=..66=..cl= for a per-frame diff.
    code += puts_code(LBL_9E); code += puthex_sb(0x2800 + 0x9E)
    code += puts_code(LBL_64); code += puthex_sb(0x2800 + 0x64)
    code += puts_code(LBL_A0); code += puthex_sb(0x2800 + 0xA0)
    code += puthex_sb(0x2800 + 0xA1)
    code += puts_code(LBL_D4); code += puthex_sb(0x2800 + 0xD4)
    code += puts_code(LBL_66); code += puthex_sb(0x2800 + 0x66)
    # device cl cells 0x0761/0x0762/0x0775 (DPX=0)
    code += puts_code(SEP_CL); code += dump_block(0x0761, 0x02, 0x00); code += puthex_xdata(0x0775)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_ea_hook():
    """POSTBOND VARIANT: log EVERY eaac call (in counter order) via a SELF-SEEDING
    DECREMENTING BUDGET, so the full ordered desc_type stream is captured incl the
    0x08 (cm8) descriptors that the original change-on-0x077A gate misses.

    Gate logic (DPX=0):
      seen = XDATA[EA_SEEN]
      if seen == 0:                    # first ever call -> seed budget
          XDATA[EA_BUDGET] = EA_BUDGET_N
          XDATA[EA_SEEN]   = 1
      budget = XDATA[EA_BUDGET]
      if budget == 0: skip             # budget exhausted -> silent
      XDATA[EA_BUDGET] = budget - 1    # consume one
      emit line

    This logs the FIRST EA_BUDGET_N eaac calls unconditionally, in counter order.
    The bond/connect descriptors (incl the post-bond cm8 0x08s) all arrive within
    the first few dozen eaac calls, so a budget of ~96 captures the whole window.

    NOTE: eaac OVERWRITES 0x077A during its copy, so we read 0x077A / |2a= BEFORE the
    displaced MOV DPTR,#0x775 runs -- i.e. the line captures the host-posted value
    from this pass's source plane, which is exactly the watched evolution."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- INTERCYCLE VARIANT: NO arm gate. Log EVERY eaac call from the very first
    # one (counter-ordered, budget EA_BUDGET_N) so the FULL 2a-plane route walk is
    # captured across BOTH CL0 cycles: the 1st cycle (route walks to 303C), the bond
    # moment, and the 2nd cycle (303C -> 3C3D done byte). This is the inter-cycle
    # transition the post-bond-armed variant skipped. ljmp_arm_skip is unused here.
    ljmp_arm_skip = None

    # --- self-seed: if EA_SEEN==0 then EA_BUDGET=EA_BUDGET_N, EA_SEEN=1.
    code += mov_dptr(EA_SEEN) + b"\xe0"            # movx a,@dptr (SEEN)
    jnz_have_seen = len(code)
    code += b"\x70\x00"                            # jnz HAVE_SEEN (already seeded)
    code += mov_dptr(EA_BUDGET) + bytes([0x74, EA_BUDGET_N, 0xF0])  # BUDGET=N
    code += mov_dptr(EA_SEEN) + b"\x74\x01\xf0"    # SEEN=1
    have_seen = len(code)
    code[jnz_have_seen + 1] = (have_seen - (jnz_have_seen + 2)) & 0xFF

    # --- budget gate: budget = XDATA[EA_BUDGET]; if 0 -> SKIP; else dec + emit.
    code += mov_dptr(EA_BUDGET) + b"\xe0"          # movx a,@dptr (BUDGET) -> A
    jnz_emit = len(code)
    code += b"\x70\x00"                            # jnz EMIT (budget!=0)
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (budget==0)

    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF

    # --- EMIT: consume one budget unit (A still = BUDGET; DPTR still = EA_BUDGET).
    code += b"\x14\xf0"                            # dec a ; movx @dptr,a (BUDGET--)

    code += puts_code(STR_EA)                       # "\r\n[ea  "
    code += emit_counter()
    code += build_ea_field_block()

    skip = len(code)
    skip_abs = CAVE_EA + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    # (intercycle variant: no arm-gate ljmp to patch)
    if ljmp_arm_skip is not None:
        code[ljmp_arm_skip + 1] = (skip_abs >> 8) & 0xFF
        code[ljmp_arm_skip + 2] = skip_abs & 0xFF

    # --- restore + replay (SKIP label) ---
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += H_EAAC_OLD                             # replay MOV DPTR,#0x775
    code += b"\x22"                                # ret
    return bytes(code)


# ---- wrap / unwrap / patch plumbing (identical to the clbond template) -------
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
        repl += b"\x00"                            # NOP pad (none needed: 3==3)
    body[off:off + len(old)] = repl


def apply_patch(body):
    write_cave(body, SHARED_DUMP, build_shared_dump(), "shared dump")
    write_cave(body, STR_A66, b"\r\n[a66 \x00", "a66 str")
    write_cave(body, STR_EA, b"\r\n[ea  \x00", "ea str")
    write_cave(body, LBL_66, b" 66=\x00", "66")
    write_cave(body, LBL_9E, b" 9E=\x00", "9E")
    write_cave(body, LBL_A0, b" A0=\x00", "A0")
    write_cave(body, LBL_A1, b" A1=\x00", "A1")
    write_cave(body, LBL_D4, b" D4=\x00", "D4")
    write_cave(body, LBL_64, b" 64=\x00", "64")
    write_cave(body, LBL_2C, b" 2C=\x00", "2C")
    write_cave(body, LBL_2D, b" 2D=\x00", "2D")
    write_cave(body, SEP_HD, b"|hd=\x00", "hd")
    write_cave(body, SEP_CL, b"|cl=\x00", "cl")
    write_cave(body, SEP_19, b"|19=\x00", "19")
    write_cave(body, LBL_77A, b" 77A=\x00", "77A")
    write_cave(body, LBL_779, b" 779=\x00", "779")
    write_cave(body, SEP_2A, b"|2a=\x00", "2a")
    write_cave(body, SEP_6F0, b"|6f0=\x00", "6f0")

    a66_hook = build_a66_hook()
    write_cave(body, CAVE_A66, a66_hook, "a66 hook")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE_A66, "a066")

    ea_hook = build_ea_hook()
    write_cave(body, CAVE_EA, ea_hook, "ea hook")
    patch_site(body, H_EAAC_OFF, H_EAAC_OLD, CAVE_EA, "eaac")

    return [
        ("a66", H_A66_OFF, CAVE_A66, len(a66_hook), H_A66_OLD),
        ("ea", H_EAAC_OFF, CAVE_EA, len(ea_hook), H_EAAC_OLD),
    ]


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
    print("  [a66] change-gated on SB[0x66]; hooked at CODE_BANK1::a066 entry")
    print("        fields: 66=SB66 9E=SB9E A0=SBA0 A1=SBA1 D4=SBD4 64=SB64 "
          "2C=SB2C 2D=SB2D |hd=0779..077B |cl=0761,0762 |19=0819,081A")
    print("  [ea ] change-gated on XDATA[0x077A]; hooked at CODE_BANK1::eaac entry")
    print("        fields: 77A=077A 779=0779 |2a=2a00[0..7] |6f0=06F0")
    print("  NON-INTRUSIVE: only low-frequency INT/event handlers hooked (NOT the")
    print("  hot 8000 walker) -- stock should still print Lane Bonded + "
          "[PcieTunnel-Deassert].")


if __name__ == "__main__":
    main()
