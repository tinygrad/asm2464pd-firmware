#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the e76b UPS_Rst_Deassert GATE.

The handmade wall: in e76b the heavy block (downstream PCIe link bring-up: C659/PERST
re-drive + "PcieLinkUp") is gated `MOV A,R7; JZ skip` where R7 = ee94()'s return =
P1[0x1243]. On HW handmade reads P1[0x1243]==0 -> the gate is CLOSED -> the downstream
PCIe link never comes up -> the host never sees link-up -> never posts the tunnel
ENABLE (1508.4) -> width never walks -> no GPU.

This tracer captures STOCK's gate value to confirm stock reads NONZERO (so the divergence
is "what makes P1[0x1243] nonzero", i.e. the ee94/e890 descriptor-engine commit state).

HOOK SITE (bank1; body off = addr - 0x8000 + 0xFF6B):
  CODE_BANK1::e76b ENTRY -- displaced head `12 ee 94` (LCALL 0xee94, 3 bytes). The cave
  replays LCALL ee94 (so the gate is computed exactly as normal), dumps R7 (=gate=P1[0x1243])
  + the surrounding PCIe-link regs, then LJMPs to e76e (the original MOV A,R7 that the body
  branches on). R7 is preserved across the dump (saved/restored), so the JZ at e76f still sees
  the true gate.

LINE: \r\n[e76b 1243=<R7> 1267=<P1.1267> C659=<C659> B402=<B402> 1508=<P1.1508>]

Build:
    python3 app/patch_stock_e76b.py fw_tinygrad.bin /tmp/fw_e76b.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_e76b.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001
BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


EE94 = 0xEE94               # LCALL target (ghidra addr; resolved at runtime)
H_OFF = bank1_off(0xE76B)
H_OLD = bytes.fromhex("12ee94")    # LCALL 0xee94
RETURN_GH = 0xE76E                  # MOV A,R7 (the original next instruction)

CAVE = 0x6A00
STR = 0x7000          # "\r\n[e76b "
L_1267 = 0x7010
L_C659 = 0x7018
L_B402 = 0x7020
L_1508 = 0x7028

SB_DPX = 0x01


def lcall(a):
    return bytes([0x12, (a >> 8) & 0xFF, a & 0xFF])


def ljmp(a):
    return bytes([0x02, (a >> 8) & 0xFF, a & 0xFF])


def mov_dptr(a):
    return bytes([0x90, (a >> 8) & 0xFF, a & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_r7():
    # R7 already holds the value; just print it.
    return lcall(UART_PUTHEX)


def puthex_dpx0(addr):
    return bytes([0x75, 0x93, 0x00]) + mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_dpx1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


def putc(ch):
    return mov_dptr(UART_TX) + bytes([0x74, ch, 0xF0])


# Preserve everything EXCEPT R7 (R7 is the gate the JZ needs; we save it ourselves and keep it).
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x93)


def build_hook():
    code = bytearray()
    code += lcall(EE94)              # replay: compute the gate -> R7 = P1[0x1243]
    code += bytes([0xC0, 0x07])      # PUSH R7 (save the gate before the dump clobbers regs)
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])
    code += puts_code(STR)           # "\r\n[e76b "
    # 1243 (the gate) is on the stack top-but-one; re-read it: easiest is to read R7 from stack.
    # Simpler: the gate == P1[0x1243]; just read it fresh (DPX=1).
    code += puthex_dpx1(0x1243)
    code += puts_code(L_1267); code += puthex_dpx1(0x1267)
    code += puts_code(L_C659); code += puthex_dpx0(0xC659)
    code += puts_code(L_B402); code += puthex_dpx0(0xB402)
    code += puts_code(L_1508); code += puthex_dpx1(0x1508)
    code += putc(ord(']'))
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += bytes([0xD0, 0x07])      # POP R7 (restore the gate for the JZ at e76f)
    code += ljmp(RETURN_GH)          # back to MOV A,R7 ; JZ
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


def apply_patch(body):
    write_cave(body, STR, b"\r\n[e76b 1243=\x00", "tag")
    write_cave(body, L_1267, b" 1267=\x00", "1267")
    write_cave(body, L_C659, b" C659=\x00", "C659")
    write_cave(body, L_B402, b" B402=\x00", "B402")
    write_cave(body, L_1508, b" 1508=\x00", "1508")
    hook = build_hook()
    write_cave(body, CAVE, hook, "hook")
    found = bytes(body[H_OFF:H_OFF + len(H_OLD)])
    if found != H_OLD:
        raise ValueError(f"e76b site mismatch at 0x{H_OFF:05x}: {found.hex()} != {H_OLD.hex()}")
    body[H_OFF:H_OFF + len(H_OLD)] = lcall(CAVE)
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
    print(f"  e76b hook: site 0x{H_OFF:05x} -> lcall 0x{CAVE:04x} ({hlen} bytes); returns to e76e")
    print("  dumps P1[0x1243] gate + 1267/C659/B402/1508 at each UPS_Rst_Deassert")


if __name__ == "__main__":
    main()
