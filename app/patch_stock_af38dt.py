#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the af38 SB-TX COMMIT -- dumps the FULL
af38-moment SB / SB-TX state at the moment af38 stages the SB-TX plane, just before
the LJMP d5da commit. MINIMAL + NON-INTRUSIVE.

WHY THIS EXISTS:
  We want stock's full af38-moment state to byte-diff against the HANDMADE [a5] diag
  (sb_router.h ~line 158). This tracer emits the SAME fields, SAME order, so the two
  lines compare directly:
      \\r\\n[af <ctr16> 2C=<SB2C> 0C=<SB0C> 15=<SB15> 18=<SB18>|TX=<2900[0..7]>|
        6A=<SB6A><SB6B><SB6C><SB6D> 84=<SB84><SB85> 9E=<SB9E> 66=<SB66>
        A=<SBA0><SBA1> po=<06F0> g=<0AB3> or=<C2C3><C343>]
  SB[off] = paged XDATA 0x2800+off (DPX=1). TX = SBTX plane 0x2900[0..7] (DPX=1).
  po/g/or = XDATA 0x06F0/0x0AB3/0xC2C3/0xC343 plain (DPX=0).

NON-INTRUSIVE RATIONALE (same proven pattern as app/patch_finalize_trace.py, which
  hooked a066+eaac and did NOT break the bond -- "Lane Bonded" still printed):
  - We hook the af38 EXIT (one site), and the emit is CHANGE-GATED on SBTX[5]
    (XDATA 0x2905, paged). So at most one UART burst per meaningful per-lane change,
    NOT per af38 call. The hot state-5 walker (CODE_BANK1::8000) is UNTOUCHED -- that
    is the path whose added latency trips the [Abrt 1] retry; we stay out of it.
  - af38 runs MID-INTERRUPT, so the cave PUSHes/POPs every register it can clobber
    (ACC,B,DPL,DPH,PSW,R0-R7,DPX + dump scratch 0x20-0x23). The handler's register
    state is preserved exactly -- corrupting it would break the bond.
  - The emit body is TINY (counter + 3 fields) to minimise UART time on the path.

HOOK SITE (bank1; body off = addr - 0x8000 + 0xFF6B):
  CODE_BANK1::b0af -- af38 tail, the 3-instruction exit sequence:
      b0af  e4        CLR A
      b0b0  ff        MOV R7,A
      b0b1  02 d5da   LJMP 0xd5da        ; <- the SB-TX commit
  At this point the full SBTX plane 0x2900 (incl bytes 4/5) is STAGED.
  Displaced head = `e4 ff 02 d5 da` (5 bytes, 3 whole instructions, clean boundary).
  No xref lands inside b0af..b0b3 (verified), so displacing all 5 bytes is safe.
  body off 0x1301a.

  REPLAY NOTE: the last displaced instruction is `LJMP 0xd5da` (an unconditional
  jump = af38's natural exit). The cave replays `CLR A ; MOV R7,A ; LJMP 0xd5da`
  verbatim -- so the cave exits via that LJMP and needs NO trailing RET. The hook is
  an LCALL into the cave; because we leave via LJMP (not RET), the cave first balances
  the LCALL's 2-byte return address off the stack with two `DEC SP` (after the
  PUSH/POP block) -- otherwise it would leak 2 bytes per af38 call and eventually
  overflow the stack. Then control leaves via the replayed LJMP d5da exactly as af38
  would have.

LINE FORMAT (hex, no inner spaces; change-gated on SBTX[5] = XDATA 0x2905 DPX=1):
    \r\n[af <ctr16> 2C=<SB2C> 0C=<SB0C> 15=<SB15> 18=<SB18>|TX=<2900[0..7]>|
       6A=<SB6A><SB6B><SB6C><SB6D> 84=<SB84><SB85> 9E=<SB9E> 66=<SB66>
       A=<SBA0><SBA1> po=<06F0> g=<0AB3> or=<C2C3><C343>]
  ctr16    = free-running 16-bit counter (hi then lo), XDATA 0x8830:0x8831 DPX=0.
  SBxx     = SB register at paged XDATA 0x2800+xx (DPX=1 paged); printed via the
             single-byte paged read helper puthex_sb(off).
  TX       = SBTX plane 0x2900[0..7] (8 bytes, DPX=1 paged) via dump_block.
  po       = XDATA[0x06F0] active-port selector (DPX=0 plain).
  g        = XDATA[0x0AB3] phy_lane_gate (DPX=0 plain, full byte).
  or       = XDATA[0xC2C3]<<XDATA[0xC343] orientation regs (DPX=0 plain, full
             bytes -- read bit0 yourself).
  These fields/order MIRROR the handmade [a5] diag (sb_router.h ~158-170) so the two
  traces diff byte-for-byte.

SCRATCH XDATA (all verified stock-unused: no DPTR/MOVX refs in fw_tinygrad.bin):
  Counter   0x8830 / 0x8831 -- free-running 16-bit (same cells as
                              app/patch_stocksbstate.py / patch_finalize_trace.py).
  Gate      0x0B5A (last SBTX[5]) / 0x0B5B (seen flag) -- the 0x0B5x headroom block
                              (neighbours 0x0B53..0x0B59 used by the sibling tracers;
                              0x0B5A/0x0B5B are free).

Build:
    python3 app/patch_af38tx.py fw_tinygrad.bin /tmp/fw_af38tx.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_af38tx.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook site (body offset) + displaced bytes (replaced + replayed). ---------
# af38 exit @ CODE_BANK1::b0af: `e4 ff 02 d5 da` = CLR A ; MOV R7,A ; LJMP 0xd5da
# (3 instructions, 5 bytes, clean boundary; final LJMP is af38's exit).
H_AF_OFF = bank1_off(0xB0AF)         # 0x1301a
H_AF_OLD = bytes.fromhex("e4ff02d5da")

# ---- Caves in the low shared zero region (<0x8000, flat in both banks). The UART
# helpers 0x538D puts / 0x51C7 puthex are <0x8000 too.
SHARED_DUMP = 0x7000       # paged/plain block-dump subroutine (~34 B)

# Label strings packed CONTIGUOUSLY after the shared-dump subroutine, starting at
# STR_BASE. Addresses are assigned by the allocator below (no overlap with SHARED_DUMP
# and no spill past the next landmark). The af38 hook cave (CAVE_AF, 0x6800) sits BELOW
# SHARED_DUMP, leaving 0x800 bytes for the hook body before 0x7000.
STR_BASE = 0x7040          # first label string (after the ~34 B dump subroutine)

# Each label string (NUL-terminated). Order matches the emit order in the field block.
LABELS = [
    ("STR_AF", b"\r\n[af \x00"),   # line head
    ("SEP_2C", b" 2C=\x00"),
    ("SEP_0C", b" 0C=\x00"),
    ("SEP_15", b" 15=\x00"),
    ("SEP_18", b" 18=\x00"),
    ("SEP_TX", b"|TX=\x00"),
    ("SEP_6A", b"|6A=\x00"),
    ("SEP_84", b" 84=\x00"),
    ("SEP_9E", b" 9E=\x00"),
    ("SEP_66", b" 66=\x00"),
    ("SEP_A",  b" A=\x00"),
    ("SEP_PO", b" po=\x00"),
    ("SEP_G",  b" g=\x00"),
    ("SEP_OR", b" or=\x00"),
    ("SEP_RA", b"|2a=\x00"),
    ("SEP_RB", b"|2b=\x00"),
]

# Assign each label a contiguous address; expose as module-level names + a (addr,bytes)
# table for the cave-writer / verification.
STR = {}            # name -> addr
STR_TABLE = []      # [(addr, bytes)]
_p = STR_BASE
for _name, _bytes in LABELS:
    STR[_name] = _p
    STR_TABLE.append((_p, _bytes))
    globals()[_name] = _p
    _p += len(_bytes)
STR_END = _p        # one past the last string byte

CAVE_AF = 0x6800           # af38 exit hook

# Free-running 16-bit counter (XDATA, DPX=0). Stock leaves 0x8830/0x8831 unused.
CTR_LO = 0x8830
CTR_HI = 0x8831

# Change-gate state (0x0B5x headroom, verified free on STOCK).
AF_LAST = 0x0B5A           # last-seen SBTX[5] (XDATA 0x2905)
AF_SEEN = 0x0B5B           # seen flag

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


def puthex_sb(off):
    # Single-byte SB-register read from the paged 0x2800 plane (DPX=1), print it, then
    # restore DPX=0. Mirrors the inline paged-read pattern used in the change-gate.
    # UART_PUTHEX itself leaves DPX=0, but we restore it explicitly before the read so
    # back-to-back puthex_sb() calls are each self-contained. Assumes DPX=0 on entry.
    return (
        bytes([0x75, 0x93, SB_DPX])                # mov DPX,#1
        + mov_dptr(0x2800 + (off & 0xFF)) + b"\xe0\xff"   # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])                # mov DPX,#0  (restore before puthex)
        + lcall(UART_PUTHEX)
    )


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the dump scratch (0x20/0x21/0x22/0x23).
# af38 runs MID-INTERRUPT, so every register the cave touches must be restored.
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


# ---- af38 exit hook ---------------------------------------------------------
def build_af_field_block():
    """The [af] per-line field block. DPX is 0 on entry/return. Fields/order MIRROR the
    handmade [a5] diag (sb_router.h ~158-170):
        2C=<SB2C> 0C=<SB0C> 15=<SB15> 18=<SB18>|TX=<2900[0..7]>|
        6A=<SB6A..6D> 84=<SB84><SB85> 9E=<SB9E> 66=<SB66> A=<SBA0><SBA1>
        po=<06F0> g=<0AB3> or=<C2C3><C343>"""
    code = bytearray()
    # SB registers (paged 0x2800 plane, DPX=1 per read via puthex_sb).
    code += puts_code(SEP_2C); code += puthex_sb(0x2C)
    code += puts_code(SEP_0C); code += puthex_sb(0x0C)
    code += puts_code(SEP_15); code += puthex_sb(0x15)
    code += puts_code(SEP_18); code += puthex_sb(0x18)
    # "|TX=" SBTX plane 0x2900[0..7] (8 bytes, DPX=1 paged) via the dump subroutine.
    code += puts_code(SEP_TX)
    code += dump_block(0x2900 + 0x00, 0x08, SB_DPX)
    # "|6A=" SB[0x6A..0x6D] (4 bytes, paged). Use dump_block on the 0x2800 plane.
    code += puts_code(SEP_6A)
    code += dump_block(0x2800 + 0x6A, 0x04, SB_DPX)
    # " 84=" SB[0x84],SB[0x85] (paged).
    code += puts_code(SEP_84)
    code += dump_block(0x2800 + 0x84, 0x02, SB_DPX)
    # " 9E=" SB[0x9E] ; " 66=" SB[0x66] (paged singles).
    code += puts_code(SEP_9E); code += puthex_sb(0x9E)
    code += puts_code(SEP_66); code += puthex_sb(0x66)
    # " A=" SB[0xA0],SB[0xA1] (paged).
    code += puts_code(SEP_A)
    code += dump_block(0x2800 + 0xA0, 0x02, SB_DPX)
    # " po=" active-port selector 0x06F0 (DPX=0 plain)
    code += puts_code(SEP_PO)
    code += puthex_xdata(0x06F0)
    # " g=" phy_lane_gate 0x0AB3 (DPX=0 plain)
    code += puts_code(SEP_G)
    code += puthex_xdata(0x0AB3)
    # " or=" orientation regs 0xC2C3 then 0xC343 (DPX=0 plain, full bytes)
    code += puts_code(SEP_OR)
    code += puthex_xdata(0xC2C3)
    code += puthex_xdata(0xC343)
    # "|2a=" / "|2b=" host RX descriptor planes 0x2a00[0..3] / 0x2b00[0..3] (byte[0]=desc_type).
    # af38 reads the plane selected by the port (0/1 => 2a/2b); dump both so desc_type is visible
    # regardless of which port this af38 call served.
    code += puts_code(SEP_RA)
    code += dump_block(0x2A00, 0x04, SB_DPX)
    code += puts_code(SEP_RB)
    code += dump_block(0x2B00, 0x04, SB_DPX)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_af_hook():
    """push preserve; DPX=0; CHANGE-GATE on SBTX[5] (XDATA 0x2905, paged); if changed
    emit the line + update last-seen; pop; replay displaced bytes; LJMP d5da (no RET --
    the replayed tail's own LJMP is af38's exit)."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- BUDGET gate (af38dt variant): log EVERY af38 call in counter order until the
    # budget is exhausted, so the FULL ordered desc-type sequence is captured (the
    # change-gate-on-SBTX5 sibling MISSES same-SBTX5 calls). AF_SEEN(0x0B5B)=init flag;
    # AF_LAST(0x0B5A)=remaining budget. First call: seed budget. Then: spend-or-skip.
    BUDGET = 0x50                                  # ~80 af38 calls logged in order
    code += mov_dptr(AF_SEEN) + b"\xe0"            # movx a,@dptr (init flag)
    jnz_have = len(code)
    code += b"\x70\x00"                            # jnz HAVE (already initialized)
    code += mov_dptr(AF_SEEN) + bytes([0x74, 0x01, 0xF0])   # SEEN=1
    code += mov_dptr(AF_LAST) + bytes([0x74, BUDGET, 0xF0]) # AF_LAST=BUDGET
    sjmp_emit = len(code)
    code += b"\x80\x00"                            # sjmp EMIT
    have = len(code)
    code[jnz_have + 1] = (have - (jnz_have + 2)) & 0xFF
    code += mov_dptr(AF_LAST) + b"\xe0"            # movx a,@dptr (budget) -> A
    jnz_spend = len(code)
    code += b"\x70\x00"                            # jnz SPEND (budget>0)
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (budget==0)
    spend = len(code)
    code[jnz_spend + 1] = (spend - (jnz_spend + 2)) & 0xFF
    code += b"\x14"                                # dec a
    code += mov_dptr(AF_LAST) + b"\xf0"           # movx @dptr,a (AF_LAST--)

    emit = len(code)
    code[sjmp_emit + 1] = (emit - (sjmp_emit + 2)) & 0xFF

    # --- EMIT: print the line.

    code += puts_code(STR_AF)                       # "\r\n[af "
    code += emit_counter()
    code += build_af_field_block()

    skip = len(code)
    skip_abs = CAVE_AF + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # --- restore + replay (SKIP label). DPX=0 for safe pops, pop all preserved regs,
    # then replay the displaced 3 instructions. The replayed `LJMP d5da` IS the exit;
    # no trailing RET.
    #
    # STACK BALANCE: the hook entered via LCALL (which pushed a 2-byte return addr).
    # We exit via the replayed `LJMP d5da` (NOT a RET), so that 2-byte return addr is
    # still on the stack and must be reclaimed or it leaks every af38 call (-> stack
    # grows -> brick). After popping all preserved regs, the SP sits exactly above the
    # LCALL return addr, so two `DEC SP` discard it cleanly before the LJMP. (None of
    # CLR A / MOV R7,A / LJMP touch SP, so the order is safe.)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += bytes([0x15, 0x81, 0x15, 0x81])        # dec SP ; dec SP (drop LCALL retaddr)
    code += H_AF_OLD                               # replay CLR A ; MOV R7,A ; LJMP d5da
    return bytes(code)


# ---- wrap / unwrap / patch plumbing (identical to the finalize template) ------
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
        repl += b"\x00"                            # NOP pad displaced bytes 4,5
    body[off:off + len(old)] = repl


def apply_patch(body):
    dump = build_shared_dump()
    # --- layout asserts: dump subroutine must not run into the string block, the string
    # block must not spill past 0x8000, and the af38 hook must fit below SHARED_DUMP.
    if SHARED_DUMP + len(dump) > STR_BASE:
        raise ValueError(
            f"shared-dump ({len(dump)} B) at 0x{SHARED_DUMP:04x} overruns STR_BASE "
            f"0x{STR_BASE:04x}")
    if STR_END > 0x8000:
        raise ValueError(f"string block spills past 0x8000 (ends 0x{STR_END:04x})")

    write_cave(body, SHARED_DUMP, dump, "shared dump")
    for addr, data in STR_TABLE:
        write_cave(body, addr, data, f"str@0x{addr:04x}")

    af_hook = build_af_hook()
    if CAVE_AF + len(af_hook) > SHARED_DUMP:
        raise ValueError(
            f"af hook ({len(af_hook)} B) at 0x{CAVE_AF:04x} overruns SHARED_DUMP "
            f"0x{SHARED_DUMP:04x}")
    write_cave(body, CAVE_AF, af_hook, "af hook")
    patch_site(body, H_AF_OFF, H_AF_OLD, CAVE_AF, "af38")

    return [("af", H_AF_OFF, CAVE_AF, len(af_hook), H_AF_OLD)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)

    # snapshot for the changed-bytes summary
    before = bytes(body)

    info = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)

    # ---- summary -----------------------------------------------------------
    print(f"input:  {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    for name, off, cave, hlen, old in info:
        repl = lcall(cave) + b"\x00" * (len(old) - 3)
        print(f"  {name}: site body 0x{off:05x}  displaced {old.hex()} -> "
              f"{repl.hex()}  (lcall 0x{cave:04x}; hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print("  [af] change-gated on SBTX[5] (XDATA 0x2905 paged); hooked at "
          "CODE_BANK1::b0af (af38 exit, just before LJMP d5da)")
    print(f"       strings 0x{STR_BASE:04x}..0x{STR_END:04x} ({STR_END - STR_BASE} B); "
          f"hook 0x{CAVE_AF:04x}..0x{CAVE_AF + info[0][3]:04x} (< SHARED_DUMP 0x{SHARED_DUMP:04x})")
    print("       line: \\r\\n[af <ctr16> 2C=<SB2C> 0C=<SB0C> 15=<SB15> 18=<SB18>|"
          "TX=<2900[0..7]>|6A=<SB6A><SB6B><SB6C><SB6D> 84=<SB84><SB85> 9E=<SB9E> "
          "66=<SB66> A=<SBA0><SBA1> po=<06F0> g=<0AB3> or=<C2C3><C343>]")
    print("  NON-INTRUSIVE: single low-rate exit hook, change-gated, hot 8000 walker")
    print("  untouched -- stock should still print Lane Bonded.")

    # ---- verification: only the hook site + caves changed ------------------
    diffs = []
    i = 0
    n = len(before)
    while i < n:
        if before[i] != body[i]:
            j = i
            while j < n and before[j] != body[j]:
                j += 1
            diffs.append((i, j))
            i = j
        else:
            i += 1
    # expected changed regions: each cave region + the hook site
    cave_regions = [
        (SHARED_DUMP, SHARED_DUMP + len(build_shared_dump())),
        (STR_BASE, STR_END),                   # the contiguous string block
        (CAVE_AF, CAVE_AF + info[0][3]),
        (H_AF_OFF, H_AF_OFF + len(H_AF_OLD)),
    ]

    def contained(region):
        s, e = region
        return any(cs <= s and e <= ce for cs, ce in cave_regions)

    print("  changed byte-regions (body):")
    all_ok = True
    for s, e in diffs:
        ok = contained((s, e))
        all_ok = all_ok and ok
        tag = "ok" if ok else "UNEXPECTED"
        print(f"    [0x{s:05x}..0x{e:05x})  {e - s} bytes  {tag}")
    print(f"  -> {'all changes confined to hook site + caves' if all_ok else 'UNEXPECTED CHANGES PRESENT'}")

    # ---- re-validate the output round-trips (A5 magic + checksum + CRC) -----
    rt, rt_wrapped = unwrap_image(out)
    body_csum = sum(rt) & 0xFF
    if wrapped:
        footer = 4 + len(rt)
        stored_csum = out[footer + 1]
        stored_crc = int.from_bytes(out[footer + 2:footer + 6], "little")
        crc_ok = stored_crc == zlib.crc32(bytes(rt))
        csum_ok = stored_csum == body_csum
        magic_ok = out[footer] == 0xA5
        print(f"  round-trip: A5 magic={'ok' if magic_ok else 'BAD'} "
              f"checksum={'ok' if csum_ok else 'BAD'} "
              f"(0x{stored_csum:02x}) zlib-crc={'ok' if crc_ok else 'BAD'} "
              f"(0x{stored_crc:08x})")
    else:
        print("  round-trip: unwrapped image (raw, no footer)")


if __name__ == "__main__":
    main()
