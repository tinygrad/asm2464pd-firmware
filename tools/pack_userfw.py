#!/usr/bin/env python3
"""Pack a bootstub userfw image."""

import argparse
from pathlib import Path
import struct
import subprocess
import zlib


MAGIC = b"A24F"
HDR_SIZE = 0x40
HASH_OFF = 0x20
BODY_MAX = 0xD000
GITVERSION_SIZE = 23
BOOT_ABI_VERSION = 1


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--common", required=True)
  parser.add_argument("-o", "--output", required=True)
  args = parser.parse_args()

  body = Path(args.common).read_bytes()
  repo = Path(__file__).resolve().parents[1]
  commit = subprocess.check_output(
    ["git", "rev-parse", "--short=8", "HEAD"], cwd=repo, text=True).strip()
  dirty = subprocess.check_output(
    ["git", "status", "--porcelain", "--untracked-files=no"], cwd=repo, text=True)
  gitversion = f"{commit}-{'DIRTY' if dirty else 'CLEAN'}".encode()
  assert len(body) <= BODY_MAX, f"image too big ({len(body)} > {BODY_MAX})"
  assert len(gitversion) < GITVERSION_SIZE, \
         f"gitversion too long ({len(gitversion)} >= {GITVERSION_SIZE})"

  pre = (
    MAGIC
    + gitversion
    + b"\x00" * (GITVERSION_SIZE - len(gitversion))
    + bytes([BOOT_ABI_VERSION])
    + struct.pack("<I", len(body))
  )
  assert len(pre) == HASH_OFF

  crc = zlib.crc32(pre + body) & 0xFFFFFFFF
  image = pre + struct.pack("<I", crc)
  image += b"\x00" * (HDR_SIZE - HASH_OFF - 4) + body

  Path(args.output).write_bytes(image)

  print(f"{args.output}: {gitversion.decode()}, {len(body)}/{BODY_MAX} bytes, crc32 {crc:08x}")
