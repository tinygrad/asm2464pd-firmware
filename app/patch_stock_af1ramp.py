#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the sb_assert (FUN_CODE_a3f5) E710 WIDTH-RAMP gate.

The lead: handmade NEVER sets u4_connect_gate (XDATA 0x0AF1) bit4, so the E710 ramp
  if (0x0AF1 & 0x10) { ... REG_LINK_WIDTH_E710 = (E710 & 0xE0) | 0x1F; }
is skipped -> E710 stays 0x04 -> no width-change event -> no GPU. Stock's sw883f_case_ee
(host cmd 0x0AD2=3 / 0x0AD1=0xEE) writes 0x0AF1=0x59 (bit4 set). This tracer confirms,
on STOCK during the live connect, whether 0x0AF1 bit4 is SET at the ramp gate and whether
E710 actually ramps 0x04 -> 0x1F.

HOOK SITE (bank0, flat <0x8000 addressable):
  0x0A4CA  the `MOV DPTR,#0x0AF1` (90 0a f1) right before the `MOVX A / JNB ACC.4` ramp gate
           in FUN_CODE_a3f5 / sb_assert. Displaced head = `90 0a f1` (3 bytes = LCALL size).
  Cave prints E710(pre) and 0x0AF1, replays `90 0a f1` (restore DPTR for the original MOVX), RETs.

LINE: \r\n[ramp <ctr> af1=.. e710=..]

Build: python3 app/patch_stock_af1ramp.py fw_tinygrad.bin /tmp/fw_af1ramp.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_af1ramp.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# sb_assert ramp gate: MOV DPTR,#0x0af1 right before MOVX A / JNB ACC.4 / E710 ramp.
# Offset is into the UNWRAPPED body (fw_tinygrad.bin has a 4-byte length header).
H_OFF = 0x0A4C9
H_OLD = bytes.fromhex("900af1")   # MOV DPTR,#0x0af1

CAVE = 0x6600
STR_TAG = 0x7000           # "\r\n[ramp "
LBLS_BASE = 0x7020

CTR_LO = 0x8830
CTR_HI = 0x8831

# (label, addr, dpx). E710 plain XDATA (DPX=0); 0x0AF1 plain XDATA (DPX=0).
FIELDS = [
    (" af1=", 0x0AF1, 0),
    (" e710=", 0xE710, 0),
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
    for text, _, _ in FIELDS:
        addrs[text] = cur
        cur += len(text) + 1
    return addrs, cur


LBL_ADDR, LBL_END = alloc_labels()


def build_field_block():
    code = bytearray()
    for text, addr, dpx in FIELDS:
        code += puts_code(LBL_ADDR[text])
        code += puthex_dpx0(addr)
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(UART_TX) + b"\x74\x5d\xf0"   # ']'
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += puts_code(STR_TAG)
    code += emit_counter()
    code += build_field_block()
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_OLD          # replay MOV DPTR,#0x0af1 (restore DPTR for original MOVX A)
    code += b"\x22"        # ret
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
    write_cave(body, STR_TAG, b"\r\n[ramp \x00", "tag")
    for text in LBL_ADDR:
        write_cave(body, LBL_ADDR[text], text.encode() + b"\x00", text.strip())
    hook = build_hook()
    if CAVE + len(hook) > STR_TAG:
        raise ValueError(f"cave overruns strings (end 0x{CAVE + len(hook):04x})")
    write_cave(body, CAVE, hook, "hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "ramp_gate")
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
    print(f"  ramp_gate hook: site 0x{H_OFF:05x} -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print(f"  fires every sb_assert E710-ramp gate; dumps 0x0AF1 + E710(pre-ramp)")


if __name__ == "__main__":
    main()
