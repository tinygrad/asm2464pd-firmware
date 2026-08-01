#!/usr/bin/env python3
"""Pack a bootstub userfw image."""

import argparse
from pathlib import Path
import struct
import zlib


MAGIC = b"A24F"
HDR_SIZE = 0x40
CRC_OFF = 9
BODY_MAX = 0xD000
BOOTSTUB_ABI_VERSION = 1


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--common", required=True)
  parser.add_argument("-o", "--output", required=True)
  args = parser.parse_args()

  body = Path(args.common).read_bytes()
  if not 0 < len(body) <= BODY_MAX:
    raise ValueError(f"invalid image size ({len(body)}, maximum {BODY_MAX})")

  pre = MAGIC + bytes([BOOTSTUB_ABI_VERSION]) + struct.pack("<I", len(body))
  if len(pre) != CRC_OFF:
    raise RuntimeError("invalid user firmware header layout")

  crc = zlib.crc32(pre + body) & 0xFFFFFFFF
  image = pre + struct.pack("<I", crc)
  image += b"\x00" * (HDR_SIZE - CRC_OFF - 4) + body

  Path(args.output).write_bytes(image)

  print(f"{args.output}: {len(body)}/{BODY_MAX} bytes, crc32 {crc:08x}")
