#!/usr/bin/env python3
"""
Dump every XDATA register 0x0000-0xFFFF to a file.
Usage: python3 dump_all_regs.py <output_file>
"""
import sys, json
sys.path.insert(0, "/home/geohot/tinygrad")

mode = sys.argv[1] if len(sys.argv) > 1 else "stock"

if mode == "handmade":
  import ctypes
  from tinygrad.runtime.support.usb import USB3
  from tinygrad.runtime.autogen import libusb
  dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)
  def read8(addr):
    buf = (ctypes.c_ubyte * 1)()
    ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
    if ret < 0: return -1
    return buf[0]
else:
  from tinygrad.runtime.support.usb import ASM24Controller
  ctrl = ASM24Controller()
  def read8(addr):
    try:
      return ctrl.read(addr, 1)[0]
    except:
      return -1

# Dump interesting ranges (skip RAM areas that will just have random data)
ranges = [
  (0x0000, 0x0800),   # low XDATA / config
  (0x0800, 0x1000),   # more config
  # skip 0x1000-0x6FFF (RAM)
  # skip 0x7000-0x7FFF (bulk buffer)  
  # skip 0x8000-0xBFFF (SRAM windows / banked)
  (0x9000, 0x9400),   # USB engine
  (0xB200, 0xB300),   # PCIe
  (0xC000, 0xD000),   # DMA / NVMe / CE
  # skip 0xD000-0xEFFF (EP buffers)
  # skip 0xF000-0xFFFF (SRAM window)
]

result = {}
for start, end in ranges:
  print(f"Reading 0x{start:04X}-0x{end:04X}...", file=sys.stderr)
  for addr in range(start, end):
    v = read8(addr)
    if v > 0:  # only store non-zero
      result[f"0x{addr:04X}"] = v

outfile = f"dump_{mode}.json"
with open(outfile, 'w') as f:
  json.dump(result, f, indent=2, sort_keys=True)
print(f"Wrote {len(result)} non-zero regs to {outfile}", file=sys.stderr)
