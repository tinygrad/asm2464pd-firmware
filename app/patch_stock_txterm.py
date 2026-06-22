#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the COMMIT-TERMINAL SBTX REPLY.

THE FINAL DEVICE-SIDE CHECK (2026-06-20). The host walks a lane-pairing partner
search `20,20 -> AD,AD -> 2D,2D -> A8,A8 -> 28,28` (the host-posted 0x0779/0x077A
pair). At the matched `28,28` terminal STOCK COMMITS (host posts 3x -> SB[0x66]=01
-> Lane Bonded -> GPU) while HANDMADE OVERSHOOTS (`28,28 -> AC,2C -> 2C,2C` stuck).
The device IMAGE (2a/2b planes + SB regs) at `28,28` is byte-IDENTICAL stock-vs-
handmade (proven by app/patch_stock_intercycle.py). The ONE device-controlled signal
NOT yet byte/time-aligned to the `28,28` terminal is the device's SBTX (0x2900) af38
REPLY -- the host advances its partner search on the device's REPLY, not on the route
it posts.

This tracer hooks the af38 EXIT (where the SBTX 0x2900 plane is fully staged) and, on
every af38 call where the host-posted partner pair is the COMMIT TERMINAL
(XDATA[0x0779]==XDATA[0x077A]==0x28), dumps the FULL SBTX 0x2900[0..7] reply + the
trigger (desc_type=SBTX[0] proxy via 0x2a00[0] + 2a/2b planes) + the SB lane regs.
This compares 1:1 with the HANDMADE [TXT] logger in sb_router.h.

NON-INTRUSIVE: single low-rate exit hook, BUDGET-gated (not per-call), hot 8000 walker
untouched. Stock MUST still print Lane Bonded + enumerate GPU 1002:7590 with this
tracer (proves non-intrusive). Derived byte-for-byte from app/patch_af38tx.py; the
ONLY change is the gate: BUDGET-gated on the 0x0779==0x077A==0x28 terminal instead of
change-gated on SBTX[5], plus the 2a/2b plane fields.

HOOK SITE (bank1; body off = addr - 0x8000 + 0xFF6B):
  CODE_BANK1::b0af -- af38 tail, the 3-instruction exit sequence:
      b0af  e4        CLR A
      b0b0  ff        MOV R7,A
      b0b1  02 d5da   LJMP 0xd5da        ; <- the SB-TX commit
  At this point the full SBTX plane 0x2900 (incl bytes 4/5) is STAGED.
  Displaced head = `e4 ff 02 d5 da` (5 bytes, 3 whole instructions, clean boundary).
  body off 0x1301a. The cave exits via the replayed LJMP d5da (no RET); it first
  reclaims the LCALL's 2-byte return addr with two `DEC SP`.

LINE FORMAT (hex, no inner spaces; budget-gated on 0x0779==0x077A==0x28):
    \r\n[TXT dt=<2a00[0]> hd=<0779><077A> po=<06F0>|TX=<2900[0..7]>|
       2a=<2a00[0..7]>|2b=<2b00[0..7]> 9E=<SB9E> 64=<SB64> A=<SBA0><SBA1>
       D4=<SBD4> 66=<SB66>]
  dt   = desc_type proxy = 0x2a00[0] host descriptor type (DPX=1 paged).
  hd   = host-posted partner pair XDATA[0x0779]:[0x077A] (DPX=0 plain) -- the gate.
  po   = XDATA[0x06F0] active-port selector (DPX=0 plain).
  TX   = SBTX plane 0x2900[0..7] (the device's af38 REPLY, DPX=1 paged) -- THE TARGET.
  2a   = SB plane-2 0x2a00[0..7] (the host-posted descriptor, DPX=1 paged).
  2b   = SB plane-2 0x2b00[0..7] (DPX=1 paged).
  SBxx = SB register at paged XDATA 0x2800+xx (DPX=1 paged).

SCRATCH XDATA (all verified stock-unused: no DPTR/MOVX refs in fw_tinygrad.bin):
  Counter   0x8830 / 0x8831 -- free-running 16-bit.
  Budget    0x0B5A (decrementing budget) / 0x0B5B (seeded flag) -- 0x0B5x headroom.

Build:
    python3 app/patch_stock_txterm.py fw_tinygrad.bin /tmp/fw_txterm.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_txterm.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook site (body offset) + displaced bytes (replaced + replayed). ---------
# af38 exit @ CODE_BANK1::b0af: `e4 ff 02 d5 da` = CLR A ; MOV R7,A ; LJMP 0xd5da
H_AF_OFF = bank1_off(0xB0AF)         # 0x1301a
H_AF_OLD = bytes.fromhex("e4ff02d5da")

# ---- Caves in the low shared zero region (<0x8000, flat in both banks).
SHARED_DUMP = 0x7000       # paged/plain block-dump subroutine (~34 B)
STR_BASE = 0x7040          # first label string (after the ~34 B dump subroutine)

LABELS = [
    ("STR_TXT", b"\r\n[TXT dt=\x00"),  # line head
    ("SEP_HD", b" hd=\x00"),
    ("SEP_PO", b" po=\x00"),
    ("SEP_TX", b"|TX=\x00"),
    ("SEP_2A", b"|2a=\x00"),
    ("SEP_2B", b"|2b=\x00"),
    ("SEP_9E", b" 9E=\x00"),
    ("SEP_64", b" 64=\x00"),
    ("SEP_A",  b" A=\x00"),
    ("SEP_D4", b" D4=\x00"),
    ("SEP_66", b" 66=\x00"),
]

STR = {}
STR_TABLE = []
_p = STR_BASE
for _name, _bytes in LABELS:
    STR[_name] = _p
    STR_TABLE.append((_p, _bytes))
    globals()[_name] = _p
    _p += len(_bytes)
STR_END = _p

CAVE_AF = 0x6800           # af38 exit hook

CTR_LO = 0x8830
CTR_HI = 0x8831

# Budget gate (0x0B5x headroom, verified free on STOCK).
AF_BUDGET = 0x0B5A         # decrementing budget
AF_SEEN = 0x0B5B           # seeded flag (0 -> seed budget on first call)
AF_BUDGET_N = 40           # log the first 40 terminal frames (plenty for the bond window)

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


def puthex_sb(off):
    # Single-byte SB-register read from the paged 0x2800 plane (DPX=1), print, restore.
    return (
        bytes([0x75, 0x93, SB_DPX])                # mov DPX,#1
        + mov_dptr(0x2800 + (off & 0xFF)) + b"\xe0\xff"   # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])                # mov DPX,#0  (restore before puthex)
        + lcall(UART_PUTHEX)
    )


def puthex_paged(addr):
    # Single-byte read from an arbitrary paged plane (DPX=1), print, restore DPX=0.
    return (
        bytes([0x75, 0x93, SB_DPX])
        + mov_dptr(addr) + b"\xe0\xff"
        + bytes([0x75, 0x93, 0x00])
        + lcall(UART_PUTHEX)
    )


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the dump scratch (0x20/0x21/0x22/0x23).
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x23, 0x93)


def build_shared_dump():
    """Print R(0x22) hex bytes from paged XDATA (DPH:DPL = 0x21:0x20) at plane A."""
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
    return (
        bytes([0x75, 0x20, start & 0xFF])
        + bytes([0x75, 0x21, (start >> 8) & 0xFF])
        + bytes([0x75, 0x22, count & 0xFF])
        + bytes([0x74, dpx])
        + lcall(SHARED_DUMP)
    )


def emit_counter():
    code = bytearray()
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


# ---- af38 exit field block --------------------------------------------------
def build_txt_field_block():
    """The [TXT] per-line field block. DPX is 0 on entry/return. Mirrors the HANDMADE
    [TXT] logger (sb_router.h): dt=<2a00[0]> already emitted by the line head label
    being "[TXT dt="; this block emits the value then the rest."""
    code = bytearray()
    # dt value = desc_type proxy = host descriptor type 0x2a00[0] (paged).
    code += puthex_paged(0x2A00)
    # " hd=" host-posted partner pair 0x0779:0x077A (DPX=0 plain) -- the gate values.
    code += puts_code(SEP_HD)
    code += puthex_xdata(0x0779)
    code += puthex_xdata(0x077A)
    # " po=" active-port selector 0x06F0 (DPX=0 plain)
    code += puts_code(SEP_PO)
    code += puthex_xdata(0x06F0)
    # "|TX=" SBTX plane 0x2900[0..7] (8 bytes, DPX=1 paged) -- THE TARGET (af38 reply).
    code += puts_code(SEP_TX)
    code += dump_block(0x2900 + 0x00, 0x08, SB_DPX)
    # "|2a=" host descriptor plane 0x2a00[0..7] (DPX=1 paged).
    code += puts_code(SEP_2A)
    code += dump_block(0x2A00 + 0x00, 0x08, SB_DPX)
    # "|2b=" plane 0x2b00[0..7] (DPX=1 paged).
    code += puts_code(SEP_2B)
    code += dump_block(0x2B00 + 0x00, 0x08, SB_DPX)
    # SB lane regs (paged 0x2800 plane).
    code += puts_code(SEP_9E); code += puthex_sb(0x9E)
    code += puts_code(SEP_64); code += puthex_sb(0x64)
    code += puts_code(SEP_A)
    code += dump_block(0x2800 + 0xA0, 0x02, SB_DPX)
    code += puts_code(SEP_D4); code += puthex_sb(0xD4)
    code += puts_code(SEP_66); code += puthex_sb(0x66)
    # close
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    return bytes(code)


def build_af_hook():
    """push preserve; DPX=0; GATE: only emit when XDATA[0x0779]==XDATA[0x077A]==0x28
    (the commit terminal) AND the self-seeding budget is non-zero; pop; reclaim the
    LCALL retaddr; replay displaced bytes; LJMP d5da (no RET)."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0

    # --- TERMINAL GATE: read XDATA[0x0779] and XDATA[0x077A]; if either != 0x28 -> SKIP.
    # 0x0779/0x077A are plain XDATA (DPX=0). The emit body is >127 bytes, so cjne's 1-byte
    # signed rel cannot reach SKIP directly -- each cjne (a SHORT != jump) targets a local
    # `LJMP SKIP` placed right after the gate; the equal-case (==0x28) falls THROUGH the
    # cjne to the next instruction, and a final SJMP hops OVER the LJMP into the budget code.
    code += mov_dptr(0x0779) + b"\xe0"             # movx a,@dptr (0x0779)
    code += b"\xb4\x28\x00"                        # cjne a,#0x28, LJMP_SKIP (!=0x28)
    cjne1 = len(code) - 1
    code += mov_dptr(0x077A) + b"\xe0"             # movx a,@dptr (0x077A)
    code += b"\xb4\x28\x00"                        # cjne a,#0x28, LJMP_SKIP (!=0x28)
    cjne2 = len(code) - 1
    # both == 0x28 -> hop over the LJMP into the budget code.
    code += b"\x80\x03"                            # sjmp +3 (over the 3-byte LJMP)
    ljmp_skip_to_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP  (placeholder; patched below)
    # both cjne rels target this LJMP (= jump to ljmp_skip_to_skip opcode).
    code[cjne1] = (ljmp_skip_to_skip - (cjne1 + 1)) & 0xFF
    code[cjne2] = (ljmp_skip_to_skip - (cjne2 + 1)) & 0xFF

    # --- self-seed budget: if AF_SEEN==0 then AF_BUDGET=AF_BUDGET_N, AF_SEEN=1.
    code += mov_dptr(AF_SEEN) + b"\xe0"            # movx a,@dptr (SEEN)
    jnz_have_seen = len(code)
    code += b"\x70\x00"                            # jnz HAVE_SEEN
    code += mov_dptr(AF_BUDGET) + bytes([0x74, AF_BUDGET_N, 0xF0])  # BUDGET=N
    code += mov_dptr(AF_SEEN) + b"\x74\x01\xf0"    # SEEN=1
    have_seen = len(code)
    code[jnz_have_seen + 1] = (have_seen - (jnz_have_seen + 2)) & 0xFF

    # --- budget gate: budget = XDATA[AF_BUDGET]; if 0 -> LJMP SKIP; else dec + emit.
    code += mov_dptr(AF_BUDGET) + b"\xe0"          # movx a,@dptr (BUDGET) -> A
    jnz_emit = len(code)
    code += b"\x70\x03"                            # jnz EMIT (budget!=0; +3 over the ljmp)
    ljmp_skip_budget = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (budget==0; placeholder)

    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF
    # --- EMIT: consume one budget unit (A still = BUDGET; DPTR still = AF_BUDGET).
    code += b"\x14\xf0"                            # dec a ; movx @dptr,a (BUDGET--)

    code += puts_code(STR_TXT)                      # "\r\n[TXT dt="
    # The field block's first action prints the dt value (2a00[0]) right after the label,
    # giving "[TXT dt=<val> hd=...". No free-running counter (keep the line short).
    code += build_txt_field_block()

    # SKIP target = end of EMIT body.
    skip = len(code)
    skip_abs = CAVE_AF + skip
    # patch the two LJMP SKIP absolute targets.
    code[ljmp_skip_to_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip_to_skip + 2] = skip_abs & 0xFF
    code[ljmp_skip_budget + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip_budget + 2] = skip_abs & 0xFF

    # --- restore + replay (SKIP label). DPX=0 for safe pops, pop, reclaim LCALL
    # retaddr (2 DEC SP), replay displaced 3 instructions (LJMP d5da IS the exit).
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += bytes([0x15, 0x81, 0x15, 0x81])        # dec SP ; dec SP (drop LCALL retaddr)
    code += H_AF_OLD                               # replay CLR A ; MOV R7,A ; LJMP d5da
    return bytes(code)


# ---- wrap / unwrap / patch plumbing -----------------------------------------
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

    return [("txt", H_AF_OFF, CAVE_AF, len(af_hook), H_AF_OLD)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    before = bytes(body)
    info = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)

    print(f"input:  {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    for name, off, cave, hlen, old in info:
        repl = lcall(cave) + b"\x00" * (len(old) - 3)
        print(f"  {name}: site body 0x{off:05x}  displaced {old.hex()} -> "
              f"{repl.hex()}  (lcall 0x{cave:04x}; hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print("  [TXT] BUDGET-gated on XDATA[0x0779]==XDATA[0x077A]==0x28 (commit terminal);")
    print("        hooked at CODE_BANK1::b0af (af38 exit, SBTX 0x2900 fully staged).")
    print(f"        strings 0x{STR_BASE:04x}..0x{STR_END:04x} ({STR_END - STR_BASE} B); "
          f"hook 0x{CAVE_AF:04x}..0x{CAVE_AF + info[0][3]:04x} (< SHARED_DUMP 0x{SHARED_DUMP:04x})")
    print("        line: \\r\\n[TXT dt=<2a00[0]> hd=<0779><077A> po=<06F0>|"
          "TX=<2900[0..7]>|2a=<2a00[0..7]>|2b=<2b00[0..7]> 9E= 64= A=<A0><A1> D4= 66=]")
    print("  NON-INTRUSIVE: single low-rate exit hook, budget-gated, hot 8000 walker")
    print("  untouched -- stock MUST still print Lane Bonded + enumerate GPU 1002:7590.")

    # verification: only the hook site + caves changed.
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
    cave_regions = [
        (SHARED_DUMP, SHARED_DUMP + len(build_shared_dump())),
        (STR_BASE, STR_END),
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
        print(f"    [0x{s:05x}..0x{e:05x})  {e - s} bytes  {'ok' if ok else 'UNEXPECTED'}")
    print(f"  -> {'all changes confined to hook site + caves' if all_ok else 'UNEXPECTED CHANGES'}")

    rt, _ = unwrap_image(out)
    if wrapped:
        footer = 4 + len(rt)
        stored_csum = out[footer + 1]
        stored_crc = int.from_bytes(out[footer + 2:footer + 6], "little")
        crc_ok = stored_crc == zlib.crc32(bytes(rt))
        csum_ok = stored_csum == (sum(rt) & 0xFF)
        magic_ok = out[footer] == 0xA5
        print(f"  round-trip: A5 magic={'ok' if magic_ok else 'BAD'} "
              f"checksum={'ok' if csum_ok else 'BAD'} zlib-crc={'ok' if crc_ok else 'BAD'}")


if __name__ == "__main__":
    main()
