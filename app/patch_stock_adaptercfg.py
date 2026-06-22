#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the PCIe-TUNNEL ADAPTER CONFIG (TASK A).

Hooks the `LCALL 0xc8db` inside pcie_tunnel_adapter_enable_b401 (stock cd6c, ghidra
addr cd73). The cave first replays the LCALL c8db (so stock builds B410-B42B from
0x0A52-0x0A55 exactly as normal), THEN dumps the resulting register/xdata values:

  [ACFG A52=.. A53=.. A54=.. A55=.. 7E=.. | B410=.. B411=.. B412=.. B413=.. B415=..
         B418=.. B419=.. B41A=.. B41B=.. B425=..]

This reveals the REAL adapter-config bytes stock advertises on THIS board, and whether
the SPI config blob is valid (0x707E==0xA5). The handmade fw never seeds 0x0A52-0x0A55
(no SPI-shadow load) so it reads 0x55 -> garbage adapter; this gives the values to seed.

FILE-OFFSET CONVENTION (verified on fw_tinygrad.bin):
  * fw_tinygrad.bin is a WRAPPED image: 4-byte body_len LE + body + 0xA5 + csum + crc32.
  * After unwrap_image() strips the 4-byte header, BODY offset == ghidra CODE address.
  * LCALL TARGETS also use the ghidra CODE address directly (resolved at runtime).

Build:
    python3 app/patch_stock_adaptercfg.py fw_tinygrad.bin /tmp/fw_acfg.bin
"""
import argparse, zlib
from pathlib import Path

UART_PUTS   = 0x538D     # ghidra addr (LCALL target)
UART_PUTHEX = 0x51C7     # ghidra addr (LCALL target)
C8DB        = 0xc8db     # ghidra addr (LCALL target = the real adapter-config builder)
FILE_DELTA  = 4          # file_off = ghidra_addr + 4

# Hook site: ghidra cd73 `LCALL 0xc8db` (12 c8 db). In the unwrapped body, offset == ghidra.
H_OFF  = 0xcd73
H_OLD  = bytes.fromhex("12c8db")

# Cave + string (ghidra addrs; file offsets = +4). 0x6500/0x6480 unused on stock.
STR_GH = 0x6480
CAVE_GH = 0x6500

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)

def lcall(a):    return bytes([0x12, (a >> 8) & 0xFF, a & 0xFF])
def mov_dptr(a): return bytes([0x90, (a >> 8) & 0xFF, a & 0xFF])

def puts_code(addr_gh):
    # r5:r4:r3 = far ptr to string? stock uart_puts takes ptr in r3(=bank?)/dptr.
    # The keystone patcher used: 7B FF 7A hi 79 lo  then LCALL UART_PUTS.
    return bytes([0x7B, 0xFF, 0x7A, (addr_gh >> 8) & 0xFF, 0x79, addr_gh & 0xFF]) + lcall(UART_PUTS)

def puthex_xdata(addr):
    # DPX=0 plain xdata read addr -> r7 -> puthex
    return (bytes([0x75, 0x93, 0x00]) + mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX))

def putc(ch):
    # write a literal char to UART_TX 0xC001 (DPTR), via A
    return mov_dptr(0xC001) + bytes([0x74, ch, 0xF0])

def build_hook():
    code = bytearray()
    # 1) replay the original LCALL c8db FIRST so the adapter config is built
    code += lcall(C8DB)
    # 2) save regs
    for d in PRESERVE: code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    code += puts_code(STR_GH)                       # "\r\n[ACFG "
    # 0x0A52..0x0A55, 0x707E (sources)
    for a in (0x0A52, 0x0A53, 0x0A54, 0x0A55, 0x707E):
        code += puthex_xdata(a) + putc(0x20)        # value + space
    code += putc(ord('|')) + putc(0x20)
    # the resulting adapter registers
    for a in (0xB410, 0xB411, 0xB412, 0xB413, 0xB415,
              0xB418, 0xB419, 0xB41A, 0xB41B, 0xB425):
        code += puthex_xdata(a) + putc(0x20)
    code += putc(ord(']'))
    # restore
    for d in reversed(PRESERVE): code += bytes([0xD0, d])
    code += b"\x22"                                  # ret  (returns to cd76)
    return bytes(code)

def wrap_body(body):
    return (len(body).to_bytes(4, "little") + body
            + bytes([0xA5, sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, "little"))

def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little"); footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            if data[footer + 1] != (sum(body) & 0xFF): raise ValueError("checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("crc mismatch")
            return bytearray(body), True
    return bytearray(data), False

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()
    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)

    str_off  = STR_GH       # body offset == ghidra addr after unwrap
    cave_off = CAVE_GH

    s = b"\r\n[ACFG \x00"
    if body[str_off:str_off+len(s)] != bytes(len(s)):
        raise ValueError("STR cave not empty @0x%05x" % str_off)
    body[str_off:str_off+len(s)] = s

    hook = build_hook()
    if body[cave_off:cave_off+len(hook)] != bytes(len(hook)):
        raise ValueError("CAVE not empty @0x%05x (len %d)" % (cave_off, len(hook)))
    body[cave_off:cave_off+len(hook)] = hook

    found = bytes(body[H_OFF:H_OFF+len(H_OLD)])
    if found != H_OLD:
        raise ValueError(f"site mismatch @0x{H_OFF:05x}: {found.hex()} != {H_OLD.hex()}")
    body[H_OFF:H_OFF+len(H_OLD)] = lcall(CAVE_GH)

    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} (wrapped={wrapped}); output: {args.output} ({len(out)} bytes)")
    print(f"  ACFG hook: cd73 LCALL c8db (file 0x{H_OFF:05x}) -> lcall 0x{CAVE_GH:04x} "
          f"(cave file 0x{cave_off:05x}, {len(hook)} bytes)")
    print("  dumps [ACFG A52 A53 A54 A55 7E | B410 B411 B412 B413 B415 B418 B419 B41A B41B B425]")

if __name__ == "__main__":
    main()
