#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the GOAL-2 KEYSTONE READBACK.

Hooks the END of stock's sb_block_init (bank1 bb37) -- specifically the
`mov r1,#0xba; mov a,#0x3f; lcall 9838` SB[0xBA]=0x3F write at bb37+ (logical 0xBC45,
the LAST write in the init body, AFTER all the keystone SB writes:
  SB[0x81]=0x08  (bb9e-bba2),  SB[0x66]=0x20 (via 9945/9950 @bbb5-bbc6),
  SB[0x9E]=0x20  (via 994e     @bbdd-bbef)).
The hook dumps SB[0x81]/[0x66]/[0x9E]/[0x2C]/[0x2D]/[0xC9] (paged DPX=1 0x2800+off)
as `[KBS 81=.. 66=.. 9E=.. 2C=.. 2D=.. C9=..]`, then replays the SB[0xBA] write.

This is the STOCK counterpart of the handmade [KB ..] dump (handmade/src/sb.h end of
sb_block_init). The discriminator:
  * if stock ALSO reads back 0 for 81/66/9E -> they are write-only strobes (handmade's
    readback-0 is a red herring).
  * if stock reads back 08/20/20 but handmade reads 0 -> the handmade writes are NOT
    landing (a real HW addressing bug).

Apply ON TOP of an already-patched (or stock) image; caves 0x6480/0x6500 are unused by
patch_stocksbstate.py (which ends at 0x642F).

Build:
    python3 app/patch_stockkeystone.py fw_tinygrad_stocksbstate.bin /tmp/fw_kb.bin
  or directly on stock:
    python3 app/patch_stockkeystone.py fw_tinygrad.bin /tmp/fw_kb.bin
"""
import argparse, zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
BANK1_K = 0xFF6B
SB_DPX = 0x01

def b1(addr): return addr - 0x8000 + BANK1_K
def lcall(a): return bytes([0x12, (a >> 8) & 0xFF, a & 0xFF])
def mov_dptr(a): return bytes([0x90, (a >> 8) & 0xFF, a & 0xFF])

# Hook site: bb37+ logical 0xBC45 = `79 ba 74 3f 12 98 38` (7 bytes).
H_OFF = b1(0xBC45)
H_OLD = bytes.fromhex("79ba743f129838")

STR_KB = 0x6480            # "\r\n[KBS "
SEP_SP = 0x6490           # " "  -- use inline space writes instead
CAVE_KB = 0x6500

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)

def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)

def puthex_sb(off):
    # DPX=1 paged read 0x2800+off -> r7 -> puthex ; restore DPX=0
    a = 0x2800 + off
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(a) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))

def build_hook():
    code = bytearray()
    for d in PRESERVE: code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    code += puts_code(STR_KB)                       # "\r\n[KBS "
    # dump 81/66/9E/2C/2D/C9 each prefixed by nothing (fixed positions, like handmade [KB])
    for off in (0x81, 0x66, 0x9E, 0x2C, 0x2D, 0xC9):
        code += puthex_sb(off)
    # ']'
    code += mov_dptr(0xC001) + b"\x74\x5d\xf0"      # write ']' to UART_TX 0xC001
    for d in reversed(PRESERVE): code += bytes([0xD0, d])
    code += H_OLD                                    # replay SB[0xBA]=0x3F write
    code += b"\x22"                                  # ret
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
    # string
    s = b"\r\n[KBS \x00"
    if body[STR_KB:STR_KB+len(s)] != bytes(len(s)):
        raise ValueError("STR_KB cave not empty")
    body[STR_KB:STR_KB+len(s)] = s
    hook = build_hook()
    if body[CAVE_KB:CAVE_KB+len(hook)] != bytes(len(hook)):
        raise ValueError("CAVE_KB not empty (len %d)" % len(hook))
    body[CAVE_KB:CAVE_KB+len(hook)] = hook
    found = bytes(body[H_OFF:H_OFF+len(H_OLD)])
    if found != H_OLD:
        raise ValueError(f"site mismatch @0x{H_OFF:05x}: {found.hex()} != {H_OLD.hex()}")
    repl = bytearray(lcall(CAVE_KB))
    while len(repl) < len(H_OLD): repl += b"\x00"
    body[H_OFF:H_OFF+len(H_OLD)] = repl
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} (wrapped={wrapped}); output: {args.output} ({len(out)} bytes)")
    print(f"  KB hook: site bb37+ 0xBC45 (body 0x{H_OFF:05x}) -> lcall 0x{CAVE_KB:04x} "
          f"(hook {len(hook)} bytes)")
    print("  dumps [KBS 81 66 9E 2C 2D C9] right after stock bb37 keystone writes")

if __name__ == "__main__":
    main()
