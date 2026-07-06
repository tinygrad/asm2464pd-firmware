#!/usr/bin/env python3
"""Pack userfw .bin files into the bootstub-consumable wire image.

Wire format:
  +0x00  magic       'A','2','4','F'  (4 bytes)
  +0x04  gitversion  short git hash plus -CLEAN/-DIRTY, NUL-padded  (24 bytes)
  +0x1C  body_len    u32 LE           (4 bytes)
  +0x20  crc32       u32 LE — over header[0:0x20] || body  (4 bytes)
  +0x24  reserved    (28 bytes, zeros)
  +0x40  body        copied to CODE 0x2400
"""
import argparse
from pathlib import Path
import struct
import subprocess
import sys
import zlib

MAGIC      = b"A24F"
HDR_SIZE   = 0x40
HASH_OFF   = 0x20
BODY_MAX   = 0xDC10   # CODE 0x2400-0xFFFF (matches USERFW_BODY_LIMIT in bootstub.c)
GITVERSION_SIZE = 24


def read_file(path: str) -> bytes:
    with open(path, "rb") as f: return f.read()


def git_is_dirty(repo: Path) -> bool:
    try:
        status = subprocess.check_output(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=repo, encoding="utf8")
    except (subprocess.CalledProcessError, FileNotFoundError):
        return True
    return bool(status.strip())


def get_gitversion() -> str:
    repo = Path(__file__).resolve().parents[1]
    try:
        git = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=repo, encoding="utf8").strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        git = "unknown"
    return git + ("-DIRTY" if git_is_dirty(repo) else "-CLEAN")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--common", required=True, help=".bin to load at CODE 0x2400 (≤55 KB)")
    ap.add_argument("-o", "--output", required=True, help="Output image file")
    args = ap.parse_args()

    body = read_file(args.common)
    gitversion = get_gitversion().encode("ascii")

    if len(body) > BODY_MAX:
        sys.exit(f"error: image too big ({len(body)} > {BODY_MAX})")
    if len(gitversion) >= GITVERSION_SIZE:
        sys.exit(f"error: gitversion too long ({len(gitversion)} >= {GITVERSION_SIZE})")

    pre = (MAGIC
           + gitversion + b"\x00" * (GITVERSION_SIZE - len(gitversion))
           + struct.pack("<I", len(body))
           )
    assert len(pre) == HASH_OFF

    crc   = zlib.crc32(pre + body) & 0xFFFFFFFF
    image = pre + struct.pack("<I", crc) + b"\x00" * (HDR_SIZE - HASH_OFF - 4) + body

    with open(args.output, "wb") as f:
        f.write(image)

    print(f"image: {args.output}")
    print(f"  version: {gitversion.decode('ascii')}")
    print(f"  body:   {len(body):>6} bytes  (max {BODY_MAX})")
    print(f"  total:  {len(image):>6} bytes  (header {HDR_SIZE} + body {len(body)})")
    print(f"  crc32:  {crc:08x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
