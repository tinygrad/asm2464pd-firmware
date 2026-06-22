#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin SUB-FRAME TIMING tracer (2026-06-20).

GOAL: measure the cycle-level CADENCE of the USB4 sideband partner-walk on STOCK,
using the chip's free-running HW counter CCE4:CCE5 (XDATA 0xCCE4/0xCCE5, the same
counter the cb10 state-5 walker reads for its throttle), so it can be diffed against
the on-device HANDMADE measurement (sb_router.h tmring + main.c [TMR] dump). Every
frame-level observable (all 256 SB regs at the commit frame, the 4CC stream, the
af38/eaac content) is byte-identical stock-vs-handmade; the localized divergence is
the SUB-FRAME timing of the partner-walk. This is the first stock cadence capture.

DESIGN (NON-INTRUSIVE — per-frame work is a few XDATA writes, NO per-frame UART):
  Hook eaac entry + af38 entry: read CCE4:CCE5, append a 4-byte record
  [tag, stage, cce_hi, cce_lo] to a RING in stock scratch XDATA (0x88xx page).
    tag  1 = eaac (host-post arrival), 2 = af38 (device-TX dispatch)
    stage = XDATA[0x0779] (eaac walk stage) / XDATA[0x0777] (af38 desc_type mirror)
  Hook a066 entry: when SB[0x66].0 (Lane-Bonded) becomes set (the COMMIT), DUMP the
  whole ring ONCE over UART (the dump is a one-shot burst, AFTER the timed walk, so
  it does not perturb the cadence being measured). This mirrors handmade's [TMR]
  dump at TunnelUp:HELD.

This reuses the proven cave framework of app/patch_stock_intercycle.py (same three
verified hook sites + wrap/unwrap plumbing); stock enumerates the GPU with the
intercycle hooks, so these strictly-lighter hooks are non-intrusive too.

Hook sites (bank1; body off = addr - 0x8000 + 0xFF6B):
  eaac entry CODE_BANK1::EAAC  body 0x16a17  displaced `90 07 75` (MOV DPTR,#0x775)
  af38 entry CODE_BANK1::AF38  body 0x15a4d  displaced `90 07 52` (MOV DPTR,#0x752)
  a066 entry CODE_BANK1::A066  body 0x11fd1  displaced `e4 f5 4d` (CLR A; MOV 0x4d,A)

Build:
    python3 app/patch_stock_timing.py fw_tinygrad.bin /tmp/fw_timing.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_timing.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook sites (body offsets) + displaced (replaced + replayed) bytes. --------
H_A66_OFF = bank1_off(0xA066)        # 0x11fd1
H_A66_OLD = bytes.fromhex("e4f54d")  # CLR A ; MOV 0x4d,A
H_EAAC_OFF = bank1_off(0xEAAC)       # 0x16a17
H_EAAC_OLD = bytes.fromhex("900775") # MOV DPTR,#0x775
H_AF38_OFF = bank1_off(0xAF38)       # 0x15a4d
H_AF38_OLD = bytes.fromhex("900752") # MOV DPTR,#0x752

# ---- Caves in the low shared zero region (<0x8000, flat in both banks). --------
SHARED_DUMP = 0x7000       # ring-dump subroutine (~264 B, ends ~0x7108)
STR_TMR = 0x7180           # "\r\n[TMR n="
STR_T = 0x7190             # "\r\nT "
STR_END = 0x71A0           # "\r\n[TMRend]\r\n"

CAVE_PUT = 0x6700          # shared "append record to ring" subroutine
CAVE_EA = 0x6800           # eaac hook
CAVE_AF = 0x6900           # af38 hook
CAVE_A66 = 0x6A00          # a066 hook (does the ring dump on commit)

# ---- HW free-running counter (XDATA, DPX=0) ----
CCE_HI = 0xCCE4
CCE_LO = 0xCCE5

# ---- Ring + state in stock scratch XDATA (0x88xx page; stock leaves it unused —
# the intercycle/sbstate patches use 0x8830/0x8831 as free scratch). ----
RING_BASE = 0x8900         # 250 entries * 4 bytes = 1000 bytes (0x8900-0x8ce7)
RING_CAP = 250
RING_IDX = 0x88F0          # next-entry index (0..RING_CAP)
RING_ARMED = 0x88F1        # 1 once the walk started (first eaac seen)
A66_LAST = 0x88F2          # last-seen SB[0x66] (commit change-gate)
A66_SEEN = 0x88F3          # a066 seen flag
SCR_TAG = 0x88F4           # arg passing to CAVE_PUT: tag
RING_WRAP = 0x88F5         # circular-ring wrapped flag (1 = ring full, idx is oldest)

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    # DPX=0 plain XDATA read into R7, then print. Assumes DPX already 0.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX. Hooks run MID-INTERRUPT.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def mov_a_xdata(addr):
    """A = XDATA[addr] (DPX=0). Clobbers DPTR — call only when DPTR is free."""
    return mov_dptr(addr) + b"\xe0"


def mov_xdata_imm(addr, val):
    """XDATA[addr] = #val (DPX=0)."""
    return mov_dptr(addr) + bytes([0x74, val & 0xFF, 0xF0])


# ---- shared CAVE_PUT: append [tag@SCR_TAG, stage@A, CCE_HI, CCE_LO] to the ring. ----
# Caller passes the stage byte in A and has stored tag at SCR_TAG. DPX must be 0.
# The HW counter CCE4:CCE5 (0xCCE4/0xCCE5) is sampled HERE, as close to the event as
# possible. The four payload bytes are first staged into scratch (0x88FA..0x88FD) so
# the ring-pointer math (which clobbers A/B/DPTR) can't disturb them; then copied to
# the ring slot. Caller preserves all regs.
SCR_PUT_TAG = 0x88FA
SCR_PUT_STG = 0x88FB
SCR_PUT_CHI = 0x88FC
SCR_PUT_CLO = 0x88FD


def build_put():
    code = bytearray()
    # stage (in A) -> scratch
    code += mov_dptr(SCR_PUT_STG) + b"\xf0"          # movx @dptr,a
    # tag (SCR_TAG) -> scratch
    code += mov_a_xdata(SCR_TAG) + mov_dptr(SCR_PUT_TAG) + b"\xf0"
    # CCE4 (hi) -> scratch
    code += mov_a_xdata(CCE_HI) + mov_dptr(SCR_PUT_CHI) + b"\xf0"
    # CCE5 (lo) -> scratch
    code += mov_a_xdata(CCE_LO) + mov_dptr(SCR_PUT_CLO) + b"\xf0"
    # idx = XDATA[RING_IDX]; if idx >= RING_CAP -> ret (ring full).
    code += mov_a_xdata(RING_IDX)                     # a = idx
    # CIRCULAR: if idx >= CAP -> wrap idx=0 and set RING_WRAP=1 (keep the LAST CAP
    # records so the commit-adjacent frames always survive).
    code += bytes([0xFF])                            # mov r7,a (idx)
    code += bytes([0xC3, 0x94, RING_CAP & 0xFF])     # clr c ; subb a,#CAP
    jc = len(code)
    code += b"\x40\x00"                              # jc not_full (idx<CAP)
    # wrap: idx=0 ; RING_WRAP=1 ; r7=0
    code += mov_dptr(RING_IDX) + b"\x74\x00\xf0"     # idx=0
    code += mov_dptr(RING_WRAP) + b"\x74\x01\xf0"    # wrap=1
    code += bytes([0x7F, 0x00])                      # mov r7,#0
    not_full = len(code)
    code[jc + 1] = (not_full - (jc + 2)) & 0xFF
    # DPTR = RING_BASE + idx*4   (idx in R7; idx*4 via MUL)
    code += bytes([0xEF, 0x75, 0xF0, 0x04, 0xA4])    # mov a,r7 ; mov b,#4 ; mul ab -> A=lo,B=hi
    code += bytes([0xF5, 0x82])                      # DPL = lo
    code += bytes([0xE5, 0xF0, 0x24, (RING_BASE >> 8) & 0xFF, 0xF5, 0x83])  # DPH = hi + 0x89
    # write the 4 staged bytes (advance DPTR each). Read each via a SECOND dptr load
    # would clobber the write-dptr; instead push the write-dptr addr is awkward — use
    # R0/R1? Simpler: read staged byte into A by re-loading dptr each time is wrong.
    # Use: keep write-DPTR; load each staged byte through a far-pointer using a saved
    # copy of the slot DPTR in R0:R1, restoring DPTR after each scratch read.
    code += bytes([0xA8, 0x82, 0xA9, 0x83])          # mov r0,DPL ; mov r1,DPH (save slot ptr)
    for sc in (SCR_PUT_TAG, SCR_PUT_STG, SCR_PUT_CHI, SCR_PUT_CLO):
        code += mov_a_xdata(sc)                      # a = staged byte (clobbers DPTR)
        code += bytes([0x88, 0x82, 0x89, 0x83])      # mov DPL,r0 ; mov DPH,r1 (restore slot)
        code += b"\xf0"                              # movx @dptr,a
        code += bytes([0xA3, 0xA8, 0x82, 0xA9, 0x83])# inc dptr ; r0=DPL ; r1=DPH (advance saved)
    # idx++
    code += mov_dptr(RING_IDX) + b"\xe0\x04\xf0"     # a=idx ; inc ; movx
    code += b"\x22"                                  # ret
    return bytes(code)


# ---- eaac hook: arm + RESET the ring at the start of each partner-walk (stage
# 0x00 seen after a non-0x00 last stage), then append a tag=1 record. The reset
# discards pre-walk af38 noise so the ring always holds the MOST RECENT complete
# walk up to the commit; a066 dumps it. LAST_STAGE tracks the prior eaac stage. ----
LAST_STAGE = 0x88F8        # last eaac stage (walk-start detector)


def build_ea_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    # stage = XDATA[0x0779] -> B (preserve across the reset logic)
    code += mov_a_xdata(0x0779) + b"\xf5\xf0"      # a=stage ; B=stage
    # walk-start? if stage==0 AND LAST_STAGE!=0 -> reset idx=0 + armed=1.
    code += b"\xe5\xf0"                            # a = stage(B)
    jnz_notzero = len(code)
    code += b"\x70\x00"                            # jnz not_zero (stage!=0 -> just record)
    # stage==0: check LAST_STAGE
    code += mov_a_xdata(LAST_STAGE)
    jz_norst = len(code)
    code += b"\x60\x00"                            # jz no_reset (LAST==0 -> already in walk)
    # reset: idx=0, armed=1, wrap=0
    code += mov_dptr(RING_IDX) + b"\x74\x00\xf0"   # idx=0
    code += mov_dptr(RING_ARMED) + b"\x74\x01\xf0" # armed=1
    code += mov_dptr(RING_WRAP) + b"\x74\x00\xf0"  # wrap=0
    norst = len(code)
    code[jz_norst + 1] = (norst - (jz_norst + 2)) & 0xFF
    notzero = len(code)
    code[jnz_notzero + 1] = (notzero - (jnz_notzero + 2)) & 0xFF
    # LAST_STAGE = stage(B)
    code += mov_dptr(LAST_STAGE) + b"\xe5\xf0\xf0" # LAST_STAGE = B
    # if not armed, skip recording (pre-first-walk eaac with stage!=0)
    code += mov_a_xdata(RING_ARMED)
    jnz_rec = len(code)
    code += b"\x70\x00"                            # jnz record
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp skip
    record = len(code)
    code[jnz_rec + 1] = (record - (jnz_rec + 2)) & 0xFF
    # tag=1 ; A=stage(B) ; put
    code += mov_dptr(SCR_TAG) + b"\x74\x01\xf0"    # SCR_TAG=1
    code += b"\xe5\xf0"                            # a = stage(B)
    code += lcall(CAVE_PUT)
    skip = len(code)
    skip_abs = CAVE_EA + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_EAAC_OLD                             # replay MOV DPTR,#0x775
    code += b"\x22"
    return bytes(code)


# ---- af38 hook: if armed, append a tag=2 record with stage=XDATA[0x0777]. ----
def build_af_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    # if armed==0 skip
    code += mov_dptr(RING_ARMED) + b"\xe0"         # a = armed
    jnz = len(code)
    code += b"\x70\x00"                            # jnz do
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp skip
    do = len(code)
    code[jnz + 1] = (do - (jnz + 2)) & 0xFF
    # tag=2
    code += mov_dptr(SCR_TAG) + b"\x74\x02\xf0"    # SCR_TAG=2
    # A = stage = XDATA[0x0777] (last desc_type mirror)
    code += mov_a_xdata(0x0777)
    code += lcall(CAVE_PUT)
    skip = len(code)
    skip_abs = CAVE_AF + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_AF38_OLD                             # replay MOV DPTR,#0x752
    code += b"\x22"
    return bytes(code)


def mov_xdata_a(addr):
    """XDATA[addr] = A (DPX=0), preserving A. Uses B as temp (B free in dump ctx)."""
    return bytes([0xF5, 0xF0]) + mov_dptr(addr) + bytes([0xE5, 0xF0, 0xF0])


# ---- ring dump subroutine: print each [tag,stage,cce_hi,cce_lo] record RAW.
# One line per record: "\r\nT <H/D> <stage> c=<cce_hi><cce_lo>" — the host computes
# the per-frame deltas in Python (CCE counts DOWN; delta=prev-cur). k lives in XDATA
# scratch (DMP_K) because the UART helpers clobber R7; the slot pointer is rebuilt
# from k each iteration (RING_BASE + k*4) into R0:R1, robust to helper clobbers.
DMP_K = 0x88F9


DMP_N = 0x88FE             # dump count = wrap ? CAP : idx


def build_shared_dump():
    code = bytearray()
    # DMP_N = wrap ? CAP : idx  (physical slots to emit; Python reorders via w/i)
    code += mov_a_xdata(RING_WRAP)
    jnz_wrap = len(code)
    code += b"\x70\x00"                              # jnz set_cap
    code += mov_a_xdata(RING_IDX)                    # a = idx
    code += b"\x80\x02"                              # sjmp store
    setcap = len(code)
    code[jnz_wrap + 1] = (setcap - (jnz_wrap + 2)) & 0xFF
    code += bytes([0x74, RING_CAP & 0xFF])           # a = #CAP
    code += mov_dptr(DMP_N) + b"\xf0"                # DMP_N = a
    # header "[TMR n=<DMP_N>]" + " w=<wrap> i=<idx>"
    code += puts_code(STR_TMR)
    code += puthex_xdata(DMP_N)
    code += puts_code(STR_W)
    code += puthex_xdata(RING_WRAP)
    code += puts_code(STR_I)
    code += puthex_xdata(RING_IDX)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"      # ']'
    code += mov_xdata_imm(DMP_K, 0)                  # k = 0
    loop = len(code)
    # if k >= DMP_N -> done.  A = DMP_N - k ; jz done
    code += mov_a_xdata(DMP_K) + b"\xff"             # r7 = k
    code += mov_a_xdata(DMP_N)                       # a = DMP_N
    code += bytes([0xC3, 0x9F])                      # clr c ; subb a,r7  -> a = idx-k
    jnz_cont = len(code)
    code += b"\x70\x00"                              # jnz cont
    ljmp_done = len(code)
    code += b"\x02\x00\x00"                          # ljmp done
    cont = len(code)
    code[jnz_cont + 1] = (cont - (jnz_cont + 2)) & 0xFF
    # slot ptr R0:R1 = RING_BASE + k*4
    code += mov_a_xdata(DMP_K)                       # a = k
    code += bytes([0x75, 0xF0, 0x04, 0xA4])          # mov b,#4 ; mul ab -> A=lo,B=hi
    code += bytes([0xF8])                            # mov r0,a (lo)
    code += bytes([0xE5, 0xF0, 0x24, (RING_BASE >> 8) & 0xFF, 0xF9])  # a=b ; add #hi ; mov r1,a
    # read 4 record bytes via DPTR=R1:R0, staging into 0x88FA..0x88FD
    for i, sc in enumerate((SCR_PUT_TAG, SCR_PUT_STG, SCR_PUT_CHI, SCR_PUT_CLO)):
        code += bytes([0x88, 0x82, 0x89, 0x83])      # DPL=r0 ; DPH=r1
        code += b"\xa3" * i                          # inc dptr i times
        code += b"\xe0"                              # a = record byte
        code += mov_xdata_a(sc)                      # stage it (clobbers DPTR; R0/R1 kept)
    # "\r\nT "
    code += puts_code(STR_T)
    # tag letter: 1->'H' else 'D'
    code += mov_a_xdata(SCR_PUT_TAG)
    code += bytes([0xB4, 0x01, 0x00])                # cjne a,#1,emit_d (rel filled)
    cjne = len(code) - 1
    code += mov_dptr(UART_TX) + b"\x74\x48\xf0"       # 'H'
    code += b"\x80\x00"                               # sjmp over 'D'
    sjmp_idx = len(code) - 1
    code[cjne] = (len(code) - (cjne + 1)) & 0xFF
    code += mov_dptr(UART_TX) + b"\x74\x44\xf0"       # 'D'
    code[sjmp_idx] = (len(code) - (sjmp_idx + 1)) & 0xFF
    code += mov_dptr(UART_TX) + b"\x74\x20\xf0"       # ' '
    code += puthex_xdata(SCR_PUT_STG)                 # stage
    code += puts_code(STR_C)                          # " c="
    code += puthex_xdata(SCR_PUT_CHI)
    code += puthex_xdata(SCR_PUT_CLO)
    # k++
    code += mov_a_xdata(DMP_K) + b"\x04" + mov_xdata_a(DMP_K)  # a=k ; inc a ; store
    # ljmp loop
    tgt = SHARED_DUMP + loop
    code += b"\x02" + bytes([(tgt >> 8) & 0xFF, tgt & 0xFF])
    done = len(code)
    code[ljmp_done + 1] = ((SHARED_DUMP + done) >> 8) & 0xFF
    code[ljmp_done + 2] = (SHARED_DUMP + done) & 0xFF
    code += puts_code(STR_END)
    code += b"\x22"                                   # ret
    return bytes(code)


STR_C = 0x71B0
STR_W = 0x71B8             # " w="
STR_I = 0x71C0            # " i="


# ---- a066 hook: change-gate on SB[0x66]; when SB[0x66].0 set & changed -> dump. ----
def build_a66_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    # read SB[0x66] (DPX=1) into B
    code += bytes([0x75, 0x93, SB_DPX])
    code += mov_dptr(0x2866) + b"\xe0"             # a = SB66
    code += b"\xf5\xf0"                            # B = SB66
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    # compare with A66_LAST; if same -> skip. else update + (if .0 set) dump.
    code += mov_a_xdata(A66_LAST)
    code += b"\x65\xf0"                            # xrl a,B
    jnz_chg = len(code)
    code += b"\x70\x00"                            # jnz changed
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp skip
    changed = len(code)
    code[jnz_chg + 1] = (changed - (jnz_chg + 2)) & 0xFF
    # A66_LAST = B
    code += mov_dptr(A66_LAST) + b"\xe5\xf0\xf0"   # LAST = SB66
    # if (SB66 & 1) -> dump (only on the COMMIT, SB66.0=Lane-Bonded set)
    code += b"\xe5\xf0"                            # a = SB66 (B)
    code += b"\x54\x01"                            # anl a,#1
    jnz_dump = len(code)
    code += b"\x70\x00"                            # jnz dodump
    ljmp_skip2 = len(code)
    code += b"\x02\x00\x00"                        # ljmp skip
    dodump = len(code)
    code[jnz_dump + 1] = (dodump - (jnz_dump + 2)) & 0xFF
    code += lcall(SHARED_DUMP)
    # RE-ARM (idx=0) after dump so EACH SB[0x66].0 edge dumps its own walk; the
    # GPU-committing walk is the LAST dump (followed by Lane Bonded / PCIE Gen in
    # the UART). Lets us distinguish committing vs retry cycles.
    code += mov_dptr(RING_IDX) + b"\x74\x00\xf0"   # idx=0
    code += mov_dptr(RING_ARMED) + b"\x74\x01\xf0" # armed=1
    skip = len(code)
    skip_abs = CAVE_A66 + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    code[ljmp_skip2 + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip2 + 2] = skip_abs & 0xFF
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_A66_OLD                              # replay CLR A ; MOV 0x4d,A
    code += b"\x22"
    return bytes(code)


# ---- wrap / unwrap / patch plumbing (identical to the intercycle template) -----
def wrap_body(body):
    return (len(body).to_bytes(4, "little") + body
            + bytes([0xA5, sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, "little"))


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
        raise ValueError(f"{name} site mismatch at body 0x{off:05x}: found {found.hex()}, expected {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, SHARED_DUMP, build_shared_dump(), "shared dump")
    write_cave(body, STR_TMR, b"\r\n[TMR n=\x00", "tmr str")
    write_cave(body, STR_T, b"\r\nT \x00", "t str")
    write_cave(body, STR_C, b" c=\x00", "c str")
    write_cave(body, STR_W, b" w=\x00", "w str")
    write_cave(body, STR_I, b" i=\x00", "i str")
    write_cave(body, STR_END, b"\r\n[TMRend]\r\n\x00", "end str")
    write_cave(body, CAVE_PUT, build_put(), "put")
    ea = build_ea_hook(); write_cave(body, CAVE_EA, ea, "ea hook")
    af = build_af_hook(); write_cave(body, CAVE_AF, af, "af hook")
    a66 = build_a66_hook(); write_cave(body, CAVE_A66, a66, "a66 hook")
    patch_site(body, H_EAAC_OFF, H_EAAC_OLD, CAVE_EA, "eaac")
    patch_site(body, H_AF38_OFF, H_AF38_OLD, CAVE_AF, "af38")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE_A66, "a066")
    return [("ea", H_EAAC_OFF, CAVE_EA, len(ea)),
            ("af", H_AF38_OFF, CAVE_AF, len(af)),
            ("a66", H_A66_OFF, CAVE_A66, len(a66))]


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
        print(f"  {name}: site body 0x{off:05x} -> lcall 0x{cave:04x} (hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print("  eaac/af38 entry -> append [tag,stage,CCE4,CCE5] to ring 0x8900 (480B)")
    print("  a066 entry: on SB[0x66].0 commit -> dump ring [TMR ...]")


if __name__ == "__main__":
    main()
