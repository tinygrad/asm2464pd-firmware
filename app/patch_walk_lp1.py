#!/usr/bin/env python3
"""
STOCK/HANDMADE fw_tinygrad.bin code-cave tracer for the LOOP1 WIDTH-FINALIZE signal.

VANTAGE: the state-5 walker ENTRY (CODE_BANK1::8000, head `e4 f5 21` = CLR A; MOV 0x21,A).
This is the DECISIVE vantage (a066 fires POST-walk and is misleading). Fires on EVERY
walker pass; budget-gated to 24 prints so the bond window stays lean (the 16th agent proved
this is non-intrusive: stock still enumerates GPU 1002:7590 with it).

WHY: the prior 16 agents all chased LOOP2 (the cl-walk: 0779/077A -> work[0x1E]/[0x1F]).
Handmade reaches the BYTE-IDENTICAL (D,D) LOOP2 terminal as stock yet the host rejects it.
The acceptance gate is a SEPARATE signal: LOOP1's width-finalize. Ghidra-confirmed:
  - LP1 state 0x70 (8262) reads snap = 985b = XDATA[0x077B+lane]  (077B lane0 / 077C lane1,
    HOST-POSTED via eaac). Gate: (snap & 0xC0)==0xC0 AND (snap & 0xF)==(work[0x1C+lane] & 0xF).
  - On match it arms lb_width_pairA[2*lane] = ee57() = CCE4:CCE5 (the live HW width counter).
  - The device TX for desc_type 0x0D copies work_buf[0x1C..] (af38 rom21a1[0x0D]=0x1C) -> the
    host echoes its acceptance back into 077B/077C.
So work[0x1C]/[0x1D] is the device's LOOP1 TX shadow and 077B/077C is the host's LOOP1 echo.
These are the NEVER-COMPARED bytes. Capture both at the walker entry, diff stock vs handmade.

LINE FORMAT:
  \r\n[lp1 <ctr16> 779=<0779><077A> 77B=<077B><077C> 759=<0759><075A>
       w1C=<081C><081D><081E><081F> CCE=<CCE4><CCE5>
       6A=<6A><6B><6C><6D> A=<A0><A1> 66=<66>
       adv=<1334><1335><1285><134D> 1407=<1407> 1203=<1203> 1201=<1201>]

CHANGE-GATE: budget(24) + a composite signature over the LOOP1+LOOP2 cells so every distinct
LOOP1/LOOP2 state prints once:
  sig = 0779 ^ 077A ^ 077B ^ 077C ^ 0759 ^ 075A ^ 081C ^ 081D ^ 081E ^ 081F ^ 66

Build:  python3 app/patch_walk_lp1.py <in.bin> <out.bin>
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_lp1.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# WALKER hook: the 8000 walker ENTRY (`e4 f5 21` = CLR A; MOV 0x21,A). Fires on EVERY walker pass.
H_OFF = bank1_off(0x8000)
H_OLD = bytes.fromhex("e4f521")

CAVE = 0x6400             # roomy slot below the 0x7000 strings (bigger line -> bigger cave)
STR_TAG = 0x7000          # "\r\n[lp1 "
LBLS_BASE = 0x7010

CTR_LO = 0x8830
CTR_HI = 0x8831
SIG_LAST = 0x0B56        # repurposed as remaining-print budget
SIG_SEEN = 0x0B57        # first-call seed flag
SIG_PREV = 0x0B59        # last composite signature (for change-gate within budget)

SB_DPX = 0x01

# (label, addr, dpx). P1[] regs are page-1 (DPX=1); plain XDATA (CCE4/CCE5/077x/081x) are DPX=0;
# SB[] are page-1 SB plane (0x2800+off, DPX=1).
FIELDS = [
    # LOOP2 cl-walk pair (host-posted) — the prior chase, kept for cross-reference.
    (" 779=", 0x0779, 0), ("", 0x077A, 0),
    # LOOP1 host-posted width-finalize echo — THE never-compared signal.
    (" 77B=", 0x077B, 0), ("", 0x077C, 0),
    # LOOP1 / LOOP2 per-lane FSM state cells.
    (" 759=", 0x0759, 0), ("", 0x075A, 0),
    (" 75b=", 0x075B, 0), ("", 0x075C, 0),
    # device TX shadow: work[0x1C]/[0x1D] = LOOP1 (desc 0x0D), work[0x1E]/[0x1F] = LOOP2 (cl-walk).
    (" w1C=", 0x081C, 0), ("", 0x081D, 0), ("", 0x081E, 0), ("", 0x081F, 0),
    # the live HW width counter ee57 reads into lb_width_pairA.
    (" CCE=", 0xCCE4, 0), ("", 0xCCE5, 0),
    # the latched width pairA the settle compares (0x076C lane0 / 0x0770 lane1).
    (" pA=", 0x076C, 0), ("", 0x076D, 0), ("", 0x0770, 0), ("", 0x0771, 0),
    # per-lane cl_cfg the host reads.
    (" 6A=", 0x2800 + 0x6A, 1), ("", 0x2800 + 0x6B, 1), ("", 0x2800 + 0x6C, 1), ("", 0x2800 + 0x6D, 1),
    # lane FSM + bond gate.
    (" A=", 0x2800 + 0xA0, 1), ("", 0x2800 + 0xA1, 1),
    (" 66=", 0x2800 + 0x66, 1), (" 9E=", 0x2800 + 0x9E, 1),
    # width-advertise page-1 regs (the c593 advertise) + width-event flags.
    (" adv=", 0x1334, 1), ("", 0x1335, 1), ("", 0x1285, 1), ("", 0x134D, 1),
    (" 1407=", 0x1407, 1), (" 1203=", 0x1203, 1), (" 1201=", 0x1201, 1),
    # lane-advertise mask + cap.
    (" 819=", 0x0819, 0), ("", 0x081A, 0),
]

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_dpx0(addr):
    return bytes([0x75, 0x93, 0x00]) + mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_dpx1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


def emit_counter():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += mov_dptr(CTR_HI) + b"\xe0\xff" + lcall(UART_PUTHEX)
    code += mov_dptr(CTR_LO) + b"\xe0\xff" + lcall(UART_PUTHEX)
    return bytes(code)


def alloc_labels():
    addrs = {}
    cur = LBLS_BASE
    for idx, (text, _, _) in enumerate(FIELDS):
        if text == "":
            continue
        addrs[idx] = cur
        cur += len(text) + 1
    return addrs, cur


LBL_ADDR, LBL_END = alloc_labels()


def build_field_block():
    code = bytearray()
    for idx, (text, addr, dpx) in enumerate(FIELDS):
        if text != "":
            code += puts_code(LBL_ADDR[idx])
        code += puthex_dpx1(addr) if dpx else puthex_dpx0(addr)
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(UART_TX) + b"\x74\x5d\xf0"   # ']'
    return bytes(code)


def build_sig_into_acc():
    """Composite sig over LOOP1+LOOP2 cells. Uses B (0xF0) as scratch. DPX=0 throughout."""
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(0x0779) + b"\xe0\xf5\xf0"   # B = 0779
    for a in (0x077A, 0x077B, 0x077C, 0x0759, 0x075A, 0x081C, 0x081D, 0x081E, 0x081F):
        code += mov_dptr(a) + b"\xe0\x62\xf0"                                # B ^= [a]
    # SB[0x66] (DPX=1) ^= into B
    code += bytes([0x75, 0x93, SB_DPX]) + mov_dptr(0x2800 + 0x66) + b"\xe0\x62\xf0"
    code += bytes([0x75, 0x93, 0x00])
    code += b"\xe5\xf0"                                                      # A = B
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])

    # first-call seed: SIG_SEEN=0 at boot -> seed budget=24.
    code += mov_dptr(SIG_SEEN) + b"\xe0"             # a = SIG_SEEN
    jnz_have = len(code)
    code += b"\x70\x00"                              # jnz already-seeded
    code += mov_dptr(SIG_SEEN) + b"\x74\x01\xf0"     # SIG_SEEN = 1
    code += mov_dptr(SIG_LAST) + b"\x74\x50\xf0"     # budget = 80
    code += mov_dptr(SIG_PREV) + b"\x74\x00\xf0"     # SIG_PREV = 0
    have = len(code)
    code[jnz_have + 1] = (have - (jnz_have + 2)) & 0xFF

    # budget gate
    code += mov_dptr(SIG_LAST) + b"\xe0"             # a = budget
    jnz_bud = len(code)
    code += b"\x70\x00"                              # jnz budget>0
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"
    after_bud = len(code)
    code[jnz_bud + 1] = (after_bud - (jnz_bud + 2)) & 0xFF

    # change gate: compute sig, compare to SIG_PREV; if equal -> skip (don't waste budget)
    code += build_sig_into_acc()
    code += b"\xfe"                                  # mov r6,a  (sig)
    code += mov_dptr(SIG_PREV) + b"\xe0"             # a = SIG_PREV
    code += b"\xb5\x06\x00"                          # cjne a,r6,<emit>  (changed -> emit)
    ljmp_skip2 = len(code)
    code += b"\x02\x00\x00"                          # equal -> skip
    emit = len(code)
    # patch cjne rel (byte at ljmp_skip2-1) to jump to emit
    code[ljmp_skip2 - 1] = (emit - ljmp_skip2) & 0xFF

    # emit: store sig, decrement budget, print
    code += b"\xee"                                  # mov a,r6
    code += mov_dptr(SIG_PREV) + b"\xf0"             # SIG_PREV = sig
    code += mov_dptr(SIG_LAST) + b"\xe0\x14\xf0"     # budget--
    code += puts_code(STR_TAG)
    code += emit_counter()
    code += build_field_block()

    skip = len(code)
    skip_abs = CAVE + skip
    # both skip branches land at the tail
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    code[ljmp_skip2 + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip2 + 2] = skip_abs & 0xFF

    # tail: restore + replay displaced head + ret
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_OLD
    code += b"\x22"
    return bytes(code)


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
                raise ValueError("checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("crc mismatch")
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
        raise ValueError(f"{name} site mismatch at 0x{off:05x}: {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR_TAG, b"\r\n[lp1 \x00", "tag")
    for idx, addr in LBL_ADDR.items():
        text = FIELDS[idx][0]
        write_cave(body, addr, text.encode() + b"\x00", text.strip())
    hook = build_hook()
    if CAVE + len(hook) > STR_TAG:
        raise ValueError(f"cave overruns strings (end 0x{CAVE + len(hook):04x})")
    write_cave(body, CAVE, hook, "hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "walker8000")
    return len(hook)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    hlen = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    print(f"  walker8000 hook: site 0x{H_OFF:05x} -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print(f"  labels 0x{LBLS_BASE:04x}-0x{LBL_END:04x}; cave end 0x{CAVE + hlen:04x}")
    print("  budget=24, change-gated on LOOP1+LOOP2 sig; dumps 779/77B/759/w1C/CCE/pA/6A/A/66/adv")


if __name__ == "__main__":
    main()
