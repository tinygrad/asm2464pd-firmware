#!/usr/bin/env python3
"""Pack a bootstub userfw image."""

import argparse
from pathlib import Path
import struct
import subprocess


MAGIC = b"A24F"
HDR_SIZE = 0x3F
HASH_OFF = 0x1F
BODY_MAX = 0xD000
GITVERSION_SIZE = 23
CHECKSUM_SEED = 0xA52464F1


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--common", required=True)
  parser.add_argument("-o", "--output", required=True)
  args = parser.parse_args()

  body = Path(args.common).read_bytes()
  repo = Path(__file__).resolve().parents[1]
  commit = subprocess.check_output(["git", "rev-parse", "--short=8", "HEAD"], cwd=repo, text=True).strip()
  dirty = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=repo, text=True)
  gitversion = f"{commit}-{'DIRTY' if dirty else 'CLEAN'}".encode()
  assert len(body) <= BODY_MAX, f"image too big ({len(body)} > {BODY_MAX})"
  assert len(gitversion) < GITVERSION_SIZE, f"gitversion too long ({len(gitversion)} >= {GITVERSION_SIZE})"

  pre = (
    MAGIC
    + gitversion
    + b"\x00" * (GITVERSION_SIZE - len(gitversion))
    + struct.pack("<I", len(body))
  )
  assert len(pre) == HASH_OFF

  checksum = CHECKSUM_SEED
  for i, b in enumerate(pre + body):
    checksum ^= b << (8 * (i & 3))
  image = pre + struct.pack("<I", checksum)
  image += b"\x00" * (HDR_SIZE - HASH_OFF - 4) + body

  Path(args.output).write_bytes(image)

  print(f"{args.output}: {gitversion.decode()}, {len(body)}/{BODY_MAX} bytes, checksum {checksum:08x}")
