#!/usr/bin/env python3
"""Pack a bootstub userfw image."""

import argparse
from pathlib import Path
import struct
import subprocess
import sys
import zlib


MAGIC = b"A24F"
HDR_SIZE = 0x40
HASH_OFF = 0x20
BODY_MAX = 0xD000  # CODE 0x3000-0xFFFF
GITVERSION_SIZE = 23
BOOT_ABI_VERSION = 1


def git(repo: Path, *args):
  try:
    return subprocess.check_output(["git", *args], cwd=repo, text=True).strip()
  except (subprocess.CalledProcessError, FileNotFoundError):
    return None


def get_gitversion() -> str:
  repo = Path(__file__).resolve().parents[1]
  commit = git(repo, "rev-parse", "--short=8", "HEAD") or "unknown"
  status = git(repo, "status", "--porcelain", "--untracked-files=no")
  return commit + ("-CLEAN" if status == "" else "-DIRTY")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--common", required=True,
                      help=".bin to load at CODE 0x3000 (<=52 KB)")
  parser.add_argument("-o", "--output", required=True, help="output image file")
  args = parser.parse_args()

  body = Path(args.common).read_bytes()
  gitversion = get_gitversion().encode("ascii")

  if len(body) > BODY_MAX:
    raise SystemExit(f"error: image too big ({len(body)} > {BODY_MAX})")
  if len(gitversion) >= GITVERSION_SIZE:
    raise SystemExit(f"error: gitversion too long ({len(gitversion)} >= {GITVERSION_SIZE})")

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

  print(f"image: {args.output}")
  print(f"  version: {gitversion.decode('ascii')}")
  print(f"  body:   {len(body):>6} bytes  (max {BODY_MAX})")
  print(f"  total:  {len(image):>6} bytes  (header {HDR_SIZE} + body {len(body)})")
  print(f"  crc32:  {crc:08x}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
