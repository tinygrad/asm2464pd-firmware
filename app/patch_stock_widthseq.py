#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the LANE-BOND WIDTH-TRANSITION TIMELINE.

GOAL: prove (or refute) the "staggered 1->2 width transition" hypothesis. The claim
is that stock brings lane0 to CL0 FIRST (single-lane / 1-lane state), the host
registers a 1-lane bond, THEN lane1 comes to CL0 -> a real PHY 1->2 WIDTH TRANSITION
that raises P1[0x1407].0 / P1[0x1203].7. Handmade brings BOTH lanes up at the same
sample (no 1->2 transition) so the width event never fires.

To SEE the sequence we must sample the width/lane state CONTINUOUSLY through the
connect->bond, NOT only when C80A.4 fires (that's the OUTCOME we want the lead-up to).

HOOK SITE (bank1; body off = addr - 0x8000 + 0xFF6B):
  CODE_BANK1::a066 ENTRY -- the SB connect / lane-bond INT1 handler. Fires on every
  discrete SB lane event (connect, per-lane CL0, bond, abort) -- i.e. once per
  meaningful lane-state change -- so it samples the whole bond timeline at the
  natural event cadence WITHOUT the hot-walker latency that breaks the bond.
  body off 0x11fd1. Displaced head = `e4 f5 4d` (CLR A ; MOV 0x4d,A -- 2 clean
  instructions, 3 bytes = LCALL size).

CHANGE-GATE: a composite 1-byte signature
    sig = (E710 & 0x0F) ^ (P1[0x1201] << 4) ^ P1[0x1407] ^ (SBA0 << 2) ^ (SBA1 << 5)
samples to a scratch byte; the line is emitted only when sig != last_sig. This is a
WIDTH-SEQUENCE gate (vs af38seq's SB[0x66] gate which only fires at the very end),
so every distinct (width,lane) state stock passes through prints exactly once.

LINE FORMAT (E710/CA06/E302 = DPX=0 plain XDATA; P1[] = DPX=1 page-1; SB[] = DPX=1
SB plane 0x2800+off):
  \r\n[wseq <ctr16> E710=.. 1201=.. 1203=.. 1407=.. 1603=.. CA06=.. E302=..
        A0=<SBA0> A1=<SBA1> 9E=<SB9E> 66=<SB66>]

  E710     = link width (does it pass 02=1-lane before 04=2-lane?)
  1201     = current-width walk (0->1->2?)         1203 = .7 width-change pending
  1407     = .0 width event (THE bit c105 reads)   1603 = .0/.1 adapter evt
  CA06/E302= PHY link-mode/rate                    A0/A1 = per-lane FSM (07->01->02)
  9E/66    = SB lane-event strobe / Lane-Bonded gate

Build:
    python3 app/patch_stock_widthseq.py fw_tinygrad.bin /tmp/fw_wseq.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_wseq.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# a066 entry: `e4 f5 4d` = CLR A ; MOV 0x4d,A
H_A66_OFF = bank1_off(0xA066)
H_A66_OLD = bytes.fromhex("e4f54d")

CAVE = 0x6500             # roomy slot below the 0x7000 strings
STR_TAG = 0x7000          # "\r\n[wseq "
LBLS_BASE = 0x7010

CTR_LO = 0x8830
CTR_HI = 0x8831
SIG_LAST = 0x0B56        # last composite signature
SIG_SEEN = 0x0B57        # seen flag

SB_DPX = 0x01

# (label, addr, dpx). P1[] regs are page-1 (DPX=1); E710/CA06/E302 are plain XDATA (DPX=0);
# SB[] are page-1 SB plane (0x2800+off, DPX=1).
FIELDS = [
    (" E710=", 0xE710, 0), (" 1201=", 0x1201, 1), (" 1203=", 0x1203, 1),
    (" 1407=", 0x1407, 1),
    (" 819=", 0x0819, 0), (" 81A=", 0x081A, 0), (" 77A=", 0x077A, 0),
    (" CA06=", 0xCA06, 0), (" E302=", 0xE302, 0),
    (" A0=", 0x2800 + 0xA0, 1), (" A1=", 0x2800 + 0xA1, 1),
    (" 9E=", 0x2800 + 0x9E, 1), (" 66=", 0x2800 + 0x66, 1),
    # route-plane descriptor bytes [2:6] for port0 (2a) and port1 (2b) — the partner-walk
    # terminal. Stock=303C asymmetric (lane0=30/lane1=3C) vs handmade=3C3C symmetric is the
    # open lead. Capture them at EVERY distinct width/lane state including the finalize.
    (" 2a=", 0x2A02, 1), ("", 0x2A03, 1), ("", 0x2A04, 1), ("", 0x2A05, 1),
    (" 2b=", 0x2B02, 1), ("", 0x2B03, 1), ("", 0x2B04, 1), ("", 0x2B05, 1),
    # per-lane CL config SB[0x6A-0x6D] (lane0=6A:6B / lane1=6C:6D) — the device-presented cl_cfg
    # the host reads to choose the partner cl_idx. Stock asymmetric (lane0 cleared / lane1 0103)
    # vs handmade symmetric is the documented lead. + device TX shadow work_buf[0x1C-0x1F].
    (" 6A=", 0x2800 + 0x6A, 1), ("", 0x2800 + 0x6B, 1), ("", 0x2800 + 0x6C, 1), ("", 0x2800 + 0x6D, 1),
    (" w1C=", 0x081C, 0), ("", 0x081D, 0), ("", 0x081E, 0), ("", 0x081F, 0),
    # THE cl_idx source the state-5 walker reads: u4_host_desc[0x2+lane] = XDATA[0x0779+lane].
    # lane0 cl_idx = 0779&0x0F, lane1 cl_idx = 077A&0x0F. clv pair = (0779&F, 077A&F).
    # Plus the e461 push in-flight token (0x0719) and the route-query fresh-desc flag (0x0775).
    (" 779=", 0x0779, 0), ("", 0x077A, 0),
    (" 719=", 0x0719, 0), (" 775=", 0x0775, 0),
    # LOOP2 per-lane state cells (0x075B lane0 / 0x075C lane1) — the CL-walk FSM state.
    (" 75b=", 0x075B, 0), ("", 0x075C, 0),
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
    """Compute the composite signature into ACC. Uses B (0xF0) as scratch."""
    code = bytearray()
    # E710 (DPX=0) & 0x0F -> B
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(0xE710) + b"\xe0\x54\x0f\xf5\xf0"
    # P1[0x1201] (DPX=1) << 4 -> swap & 0xF0, XRL into B
    code += bytes([0x75, 0x93, SB_DPX]) + mov_dptr(0x1201) + b"\xe0\xc4\x54\xf0\x62\xf0"  # swap a;anl a,#f0;xrl 0xf0,a
    # P1[0x1407] (DPX=1) XRL into B
    code += mov_dptr(0x1407) + b"\xe0\x62\xf0"
    # SB[A0] (DPX=1) << 2 -> rl rl, XRL into B
    code += mov_dptr(0x2800 + 0xA0) + b"\xe0\x23\x23\x62\xf0"
    # SB[A1] (DPX=1) << 5 -> swap then rl (x16 then x2 = x32), XRL into B
    code += mov_dptr(0x2800 + 0xA1) + b"\xe0\xc4\x23\x62\xf0"
    # 0x0819 lane-advertise mask (DPX=0) -- XRL raw into B so any 819 change emits a line
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(0x0819) + b"\xe0\x62\xf0"
    # per-lane TX shadow work_buf[0x1E]/[0x1F] (DPX=0) -- the cl_idx walk; XRL raw so every
    # per-lane cl_idx step emits a line (this is the partner-walk we want to trace).
    code += mov_dptr(0x081E) + b"\xe0\x62\xf0"
    code += mov_dptr(0x081F) + b"\xe0\x62\xf0"
    # THE host-posted cl_idx source 0x0779/0x077A (DPX=0) -- XRL raw so every change in the
    # per-lane host descriptor (the clv pair) emits a line. This is the byte the walker echoes.
    code += mov_dptr(0x0779) + b"\xe0\x62\xf0"
    code += mov_dptr(0x077A) + b"\xe0\x62\xf0"
    code += bytes([0x75, 0x93, 0x00])
    # ACC = B (the signature)
    code += b"\xe5\xf0"
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])

    # --- change gate: compute sig in ACC, compare to SIG_LAST (only if SIG_SEEN) ---
    code += build_sig_into_acc()
    code += b"\xfe"                                  # mov r6,a   (save sig)
    code += mov_dptr(SIG_SEEN) + b"\xe0"             # a = SIG_SEEN
    jz_first = len(code)
    code += b"\x60\x00"                              # jz first-time -> skip compare, always emit
    code += mov_dptr(SIG_LAST) + b"\xe0"             # a = SIG_LAST
    code += b"\xb5\x06\x00"                          # cjne a,r6,<emit>  (changed -> emit)
    # unchanged -> skip to tail
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"

    emit = len(code)
    code[jz_first + 1] = (emit - (jz_first + 2)) & 0xFF
    code[ljmp_skip - 3 - 1] = 0  # placeholder (overwritten below for cjne rel)
    # patch the cjne rel8 (3rd byte of the cjne at offset emit-... ) -- recompute:
    # cjne is at (ljmp_skip-3); its rel byte is at ljmp_skip-1
    code[ljmp_skip - 1] = (emit - ljmp_skip) & 0xFF

    # --- emit branch: store sig, mark seen, print the line ---
    code += b"\xee"                                   # mov a,r6
    code += mov_dptr(SIG_LAST) + b"\xf0"              # SIG_LAST = sig
    code += mov_dptr(SIG_SEEN) + b"\x74\x01\xf0"      # SIG_SEEN = 1
    code += puts_code(STR_TAG)
    code += emit_counter()
    code += build_field_block()

    skip = len(code)
    skip_abs = CAVE + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # --- tail: restore + replay displaced head + ret ---
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_A66_OLD
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
    write_cave(body, STR_TAG, b"\r\n[wseq \x00", "tag")
    for idx, addr in LBL_ADDR.items():
        text = FIELDS[idx][0]
        write_cave(body, addr, text.encode() + b"\x00", text.strip())
    hook = build_hook()
    if CAVE + len(hook) > STR_TAG:
        raise ValueError(f"cave overruns strings (end 0x{CAVE + len(hook):04x})")
    write_cave(body, CAVE, hook, "hook")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE, "a066")
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
    print(f"  a066 hook: site 0x{H_A66_OFF:05x} -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print(f"  labels 0x{LBLS_BASE:04x}-0x{LBL_END:04x}; cave end 0x{CAVE + hlen:04x}")
    print("  change-gated on composite width sig; dumps E710/1201/1203/1407/1603/CA06/E302/A0/A1/9E/66")


if __name__ == "__main__":
    main()
