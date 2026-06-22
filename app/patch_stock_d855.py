#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the PCIe-TUNNEL EVENT dispatch (d855).

Hooks CODE_BANK1::d855 ENTRY. Each call the host has posted a tunnel-event bitmask in
P1[0x1508]; d855 dispatches the highest set bit (Enable/DisPath/UPS_Rst_Deassert/Assert).
This tracer dumps, IN ORDER, the 1508 bits the host posts + the adapter width/event state
so we can see WHETHER and WHEN stock gets 1508.4 (Enable) -- the leg handmade never sees:

  [D855 <ctr16> 1508=<bits> 1407=<lane-evt> 1201=<width> 1203=<wpend> 1206=<adapter-CS>
         1335=<advertise> 9FA=<route_mode>]

The cave replays the displaced d855 head bytes then RETs into d855+3 so dispatch still runs.

Build:
    python3 app/patch_stock_d855.py fw_tinygrad.bin /tmp/fw_d855.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_d855.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001
BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# d855 entry: `12 d1 72` = LCALL 0xd172 (1 instruction, 3 bytes -> clean replay).
H_OFF = bank1_off(0xD855)
H_OLD = bytes.fromhex("12d172")

CAVE = 0x6A00
STR = 0x7000          # "\r\n[D855 "
L_1407 = 0x7010
L_1201 = 0x7018
L_1203 = 0x7020
L_1206 = 0x7028
L_1335 = 0x7030
L_9FA = 0x7038

CTR_LO = 0x8830
CTR_HI = 0x8831
BUDGET = 0x0B5B
SEEN = 0x0B5C
BUDGET_N = 0x18       # log first 24 d855 calls

SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return (bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF])
            + lcall(UART_PUTS))


def puthex_xdata(addr):
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_p1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def emit_counter():
    code = bytearray()
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


def build_fields():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])
    # 1508 is the value the [D855 ] string already implies; print it first (after counter)
    code += puthex_p1(0x1508)
    code += puts_code(L_1407); code += puthex_p1(0x1407)
    code += puts_code(L_1201); code += puthex_p1(0x1201)
    code += puts_code(L_1203); code += puthex_p1(0x1203)
    code += puts_code(L_1206); code += puthex_p1(0x1206)
    code += puts_code(L_1335); code += puthex_p1(0x1335)
    code += puts_code(L_9FA); code += puthex_xdata(0x09FA)
    code += bytes([0x75, 0x93, 0x00])
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])
    # self-seed
    code += mov_dptr(SEEN) + b"\xe0"
    jnz_have = len(code)
    code += b"\x70\x00"
    code += mov_dptr(BUDGET) + bytes([0x74, BUDGET_N, 0xF0])
    code += mov_dptr(SEEN) + b"\x74\x01\xf0"
    have = len(code)
    code[jnz_have + 1] = (have - (jnz_have + 2)) & 0xFF
    # budget gate
    code += mov_dptr(BUDGET) + b"\xe0"
    jnz_emit = len(code)
    code += b"\x70\x00"
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"
    emit = len(code)
    code[jnz_emit + 1] = (emit - (jnz_emit + 2)) & 0xFF
    code += b"\x14\xf0"
    code += puts_code(STR)
    code += emit_counter()
    code += build_fields()
    skip = len(code)
    skip_abs = CAVE + skip
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_OLD                                  # replay LCALL 0xd172
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
        raise ValueError(f"{name} site mismatch at body 0x{off:05x}: {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR, b"\r\n[D855 1508=\x00", "str")
    write_cave(body, L_1407, b" 1407=\x00", "1407")
    write_cave(body, L_1201, b" 1201=\x00", "1201")
    write_cave(body, L_1203, b" 1203=\x00", "1203")
    write_cave(body, L_1206, b" 1206=\x00", "1206")
    write_cave(body, L_1335, b" 1335=\x00", "1335")
    write_cave(body, L_9FA, b" 9FA=\x00", "9FA")
    hook = build_hook()
    write_cave(body, CAVE, hook, "d855 hook")
    patch_site(body, H_OFF, H_OLD, CAVE, "d855")
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
    print(f"  d855 hook: site body 0x{H_OFF:05x} -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print("  dumps [D855 1508 1407 1201 1203 1206 1335 9FA] per tunnel-event dispatch")


if __name__ == "__main__":
    main()
