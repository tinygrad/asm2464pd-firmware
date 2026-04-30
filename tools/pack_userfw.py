#!/usr/bin/env python3
"""Pack userfw .bin files into the bootstub-consumable wire image.

Wire format:
  +0x00  magic       'A','S','P','2'  (4 bytes)
  +0x04  version     u32 LE           (4 bytes)
  +0x08  common_len  u32 LE           (4 bytes)
  +0x0C  bank0_len   u32 LE           (4 bytes)
  +0x10  bank1_len   u32 LE           (4 bytes)
  +0x14  reserved    (12 bytes, zeros)
  +0x20  crc32       u32 LE — over header[0:0x20] || body  (4 bytes)
  +0x24  reserved    (28 bytes, zeros)
  +0x40  body — common_len + bank0_len + bank1_len bytes

The bootstub copies common to CODE 0x4000-0x7FFF, bank0 to CODE
0x8000-0xFFFF with PSBANK=0, bank1 to CODE 0x8000-0xFFFF with PSBANK=1.
"""
import argparse
import struct
import sys
import zlib

MAGIC      = b"ASP2"
HDR_SIZE   = 0x40
HASH_OFF   = 0x20
COMMON_MAX = 0x4000   # 16 KB
BANK_MAX   = 0x8000   # 32 KB


def read_optional(path: str | None) -> bytes:
    if not path: return b""
    with open(path, "rb") as f: return f.read()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--common", help=".bin to load at CODE 0x4000 (≤16 KB)")
    ap.add_argument("--bank0",  help=".bin to load at CODE 0x8000 with PSBANK=0 (≤32 KB)")
    ap.add_argument("--bank1",  help=".bin to load at CODE 0x8000 with PSBANK=1 (≤32 KB)")
    ap.add_argument("-v", "--version", type=lambda s: int(s, 0), default=1,
                    help="Image version (default: 1)")
    ap.add_argument("-o", "--output", required=True, help="Output image file")
    args = ap.parse_args()

    if not (args.common or args.bank0 or args.bank1):
        sys.exit("error: at least one of --common/--bank0/--bank1 is required")

    common = read_optional(args.common)
    bank0  = read_optional(args.bank0)
    bank1  = read_optional(args.bank1)

    if len(common) > COMMON_MAX:
        sys.exit(f"error: common too big ({len(common)} > {COMMON_MAX})")
    if len(bank0) > BANK_MAX:
        sys.exit(f"error: bank0 too big ({len(bank0)} > {BANK_MAX})")
    if len(bank1) > BANK_MAX:
        sys.exit(f"error: bank1 too big ({len(bank1)} > {BANK_MAX})")

    body = common + bank0 + bank1

    pre = (MAGIC
           + struct.pack("<I", args.version)
           + struct.pack("<I", len(common))
           + struct.pack("<I", len(bank0))
           + struct.pack("<I", len(bank1))
           + b"\x00" * 12)
    assert len(pre) == HASH_OFF

    crc   = zlib.crc32(pre + body) & 0xFFFFFFFF
    image = pre + struct.pack("<I", crc) + b"\x00" * (HDR_SIZE - HASH_OFF - 4) + body

    with open(args.output, "wb") as f:
        f.write(image)

    print(f"image: {args.output}")
    print(f"  common: {len(common):>6} bytes  (max {COMMON_MAX})")
    print(f"  bank0:  {len(bank0):>6} bytes  (max {BANK_MAX})")
    print(f"  bank1:  {len(bank1):>6} bytes  (max {BANK_MAX})")
    print(f"  total:  {len(image):>6} bytes  (header {HDR_SIZE} + body {len(body)})")
    print(f"  crc32:  {crc:08x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
