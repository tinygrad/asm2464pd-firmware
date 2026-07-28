#!/usr/bin/env python3
"""Check the immutable bootstub/application linker contract."""

import re
import sys
from pathlib import Path

BASE, BODY_MAX, XRAM_END = 0x3000, 0xD000, 0x5FF8
VECTORS = (3, 0xB, 0x13, 0x1B, 0x23, 0x2B)
# Fixed SDCC 4.2 startup skeleton; growth means unsupported writable initializers.
INIT_MAX = dict(GSINIT=3, GSFINAL=3, GSINIT0=3, GSINIT1=0, GSINIT2=10,
                GSINIT3=0, GSINIT4=6, GSINIT5=0, XINIT=0, XISEG=0)


def fail(msg):
  raise SystemExit(f"layout error: {msg}")


def ihx(path):
  mem = {}
  for line in Path(path).read_text().splitlines():
    if line[7:9] != "00": continue
    n, addr = int(line[1:3], 16), int(line[3:7], 16)
    mem.update((addr + i, b) for i, b in enumerate(bytes.fromhex(line[9:9 + 2*n])))
  if not mem: fail(f"{path}: empty image")
  return mem


def ljmp(mem, addr):
  if mem.get(addr) != 2: fail(f"missing LJMP at 0x{addr:04X}")
  return mem[addr + 1] << 8 | mem[addr + 2]


def areas(path):
  text = Path(path).read_text()
  found = {}
  for name, start, size in re.findall(
      r"^(\w+)\s+([0-9A-Fa-f]{8})\s+([0-9A-Fa-f]{8})\s+=", text, re.M):
    found[name] = max(found.get(name, (0, 0)), (int(start, 16), int(size, 16)))
  return found


def check_map(path):
  a = areas(path)
  bad = [f"{name}=0x{a.get(name, (0, 0))[1]:X}" for name, limit in INIT_MAX.items()
         if a.get(name, (0, 0))[1] > limit]
  if bad: fail(f"{path}: startup skips initialization sections: {', '.join(bad)}")
  start, size = a.get("XSEG", (0, 0))
  if size and start + size > XRAM_END:
    fail(f"{path}: XSEG overlaps reserved XRAM at 0x{XRAM_END:04X}")


def check_stack(path):
  m = re.search(r"Stack starts at: .* with (\d+) bytes available", Path(path).read_text())
  if not m or int(m.group(1)) < 64: fail(f"{path}: less than 64 stack bytes available")


def main(argv):
  if len(argv) != 8:
    fail("expected boot.ihx boot.map boot.mem app.ihx app.map app.mem app.bin")
  bihx, bmap, bmem, aihx, amap, amem, abin = argv[1:]
  boot, app, body = ihx(bihx), ihx(aihx), Path(abin).read_bytes()
  if min(boot) != 0 or max(boot) >= BASE:
    fail(f"bootstub range 0x{min(boot):04X}-0x{max(boot):04X}")
  if min(app) != BASE or max(app) > 0xFFFF:
    fail(f"application range 0x{min(app):04X}-0x{max(app):04X}")
  for off in VECTORS:
    if ljmp(boot, off) != BASE + off: fail(f"bad bootstub vector 0x{off:02X}")
  if not BASE <= ljmp(app, BASE) < BASE + 0x100: fail("bad application reset vector")
  if not body or len(body) > BODY_MAX or len(body) != max(app) - BASE + 1:
    fail(f"bad application body size {len(body)}")
  for path in (bmap, amap): check_map(path)
  for path in (bmem, amem): check_stack(path)
  print(f"layout OK: bootstub through 0x{max(boot):04X}, "
        f"application {len(body)}/{BODY_MAX} bytes")


if __name__ == "__main__":
  main(sys.argv)
