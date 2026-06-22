#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer for the C80A.4 WIDTH/ADAPTER-EVENT handler
(c105 = usb4_sec_adapter_link_event_c80a4). Logs, every time the device SERVICES
C80A.4, the page-1 width-event source registers + the lane FSM, so we can see the
EXACT HW state when stock raises the width event vs handmade (which never raises it).

This is the CL0->commit gate: in stock the chain is
  C80A.4 -> c105 reads P1[0x1407].0 (width evt) / .3 (tunnel evt -> bank1_d855 ->
  PcieTunnel-Deassert) ; P1[0x1603].0 -> sets pd_cm_dispatch_sel=0x69 (lets the
  router-op path proceed) -> host posts router-op -> SB[0x66]=01 (Lane Bonded) -> GPU.
Handmade reaches CL0 with P1[0x1407]=00 / P1[0x1203]=00 / C80A.4=0 but P1[0x1508]=06
(tunnel events pending). This tracer captures what stock's P1[0x1407]/1203/1508/1603
look like when c105 actually fires.

HOOK SITE (bank0, flat <0x8000 so addressable from a low cave):
  CODE:c105 ENTRY -- the C80A.4 handler. Displaced head = `12 bc de`
    (LCALL 0xbcde = usb4_rd_evt_p1_1407, 1 instruction, 3 bytes = LCALL size).
  The cave PUSH/POPs the full register set (c105 runs mid-INT1), prints the line,
  replays the displaced LCALL 0xbcde, then RETs.

LINE FORMAT (ctr16 then the page-1 evt regs DPX=1, then C80A/E710/SB-A0/A1 DPX-mixed):
  \r\n[c80a4 <ctr> 1407=.. 1203=.. 1603=.. 1508=.. 128F=.. 1201=.. |C80A=.. E710=..
        A0=<SBA0> A1=<SBA1>]

Build:
    python3 app/patch_stock_c80a4.py fw_tinygrad.bin /tmp/fw_c80a4.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_c80a4.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# c105 is bank0 / flat (addr < 0x8000 region is shared); body offset == file offset.
H_C105_OFF = 0xC105
H_C105_OLD = bytes.fromhex("12bcde")   # LCALL 0xbcde (usb4_rd_evt_p1_1407)

CAVE = 0x6600
STR_TAG = 0x7000           # "\r\n[c80a4 "
LBLS_BASE = 0x7020

CTR_LO = 0x8830
CTR_HI = 0x8831

SB_DPX = 0x01

# (label, addr, dpx). P1[] regs are page-1 (DPX=1); C80A/E710 are plain XDATA (DPX=0);
# SB[A0]/[A1] are page-1 SB plane (0x2800+off, DPX=1).
FIELDS = [
    (" 1407=", 0x1407, 1), (" 1508=", 0x1508, 1),
    (" 1404=", 0x1404, 1), (" 1405=", 0x1405, 1), (" 1406=", 0x1406, 1),
    (" 1411=", 0x1411, 1), (" 1415=", 0x1415, 1),
    (" 1504=", 0x1504, 1), (" 1505=", 0x1505, 1), (" 1506=", 0x1506, 1),
    (" 1507=", 0x1507, 1), (" 1511=", 0x1511, 1),
    (" |C80A=", 0xC80A, 0), (" A0=", 0x2800 + 0xA0, 1),
]

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_dpx0(addr):
    return bytes([0x75, 0x93, 0x00]) + mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_dpx1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


def emit_counter():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += mov_dptr(CTR_HI) + b"\xe0\xff" + lcall(UART_PUTHEX)
    code += mov_dptr(CTR_LO) + b"\xe0\xff" + lcall(UART_PUTHEX)
    return bytes(code)


def alloc_labels():
    addrs = {}
    cur = LBLS_BASE
    for text, _, _ in FIELDS:
        addrs[text] = cur
        cur += len(text) + 1
    return addrs, cur


LBL_ADDR, LBL_END = alloc_labels()


def build_field_block():
    code = bytearray()
    for text, addr, dpx in FIELDS:
        code += puts_code(LBL_ADDR[text])
        code += puthex_dpx1(addr) if dpx else puthex_dpx0(addr)
    code += bytes([0x75, 0x93, 0x00]) + mov_dptr(UART_TX) + b"\x74\x5d\xf0"   # ']'
    return bytes(code)


def build_hook():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += puts_code(STR_TAG)
    code += emit_counter()
    code += build_field_block()
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_C105_OLD          # replay LCALL 0xbcde
    code += b"\x22"             # ret
    return bytes(code)


def wrap_body(body):
    return (len(body).to_bytes(4, "little") + body
            + bytes([0xA5, sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, "little"))


def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little")
        footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            if data[footer + 1] != (sum(body) & 0xFF):
                raise ValueError("checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("crc mismatch")
            return bytearray(body), True
    return bytearray(data), False


def write_cave(body, addr, data, name):
    end = addr + len(data)
    if body[addr:end] != bytes(len(data)):
        raise ValueError(f"{name} cave at 0x{addr:04x} not empty (len {len(data)})")
    body[addr:end] = data
    return end


def patch_site(body, off, old, cave, name):
    found = bytes(body[off:off + len(old)])
    if found != old:
        raise ValueError(f"{name} site mismatch at 0x{off:05x}: {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, STR_TAG, b"\r\n[c80a4 \x00", "tag")
    for text in LBL_ADDR:
        write_cave(body, LBL_ADDR[text], text.encode() + b"\x00", text.strip())
    hook = build_hook()
    if CAVE + len(hook) > STR_TAG:
        raise ValueError(f"cave overruns strings (end 0x{CAVE + len(hook):04x})")
    write_cave(body, CAVE, hook, "hook")
    patch_site(body, H_C105_OFF, H_C105_OLD, CAVE, "c105")
    return len(hook)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    hlen = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    print(f"  c105 hook: site 0x{H_C105_OFF:04x} -> lcall 0x{CAVE:04x} ({hlen} bytes)")
    print(f"  labels 0x{LBLS_BASE:04x}-0x{LBL_END:04x}; cave end 0x{CAVE + hlen:04x}")
    print("  fires on every C80A.4 service; dumps P1[0x1407/1203/1603/1508/128F/1201] + C80A/E710/A0/A1")


if __name__ == "__main__":
    main()
