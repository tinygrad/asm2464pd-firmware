#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer: does the C0A5 router-op mailbox fire
during the in-band ROUTER_CS read that enumerates the device?

DECISIVE TEST for project_inband_routercs_read_wall: the host's first config
read to route=1 (offset=0, ROUTER_CS) is served by the device. The open
question is WHETHER it is served by the C0A5 firmware mailbox (EC06.0 -> C0A5,
opcode 0x50 read) or purely by HW config space. If C0A5 fires during the
successful enumeration read, the read IS firmware-served and the handmade fix
target is making EC06.0 fire (the HW transport-RX staging the packet).

HOOK: CODE_BANK1::c0a5 ENTRY. c0a5 is only reached from the INT1 demux when
EC06.0 is set, so it is already low-frequency -- safe to print every entry.
Displaced head = `90 ea 90` (MOV DPTR,#0xea90, 3 bytes = LCALL size).

Prints, on each c0a5 entry:
  \r\n[C0 e90=<EA90> st=<0xB02 state> op=<0xB03 opcode> e80=<EA80> e81=<EA81>]

Non-intrusive: runs mid-interrupt; PUSH/POPs all touched regs; replays the
displaced head then RETs so c0a5 runs normally afterward.
"""
import sys
import zlib

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7

BANK1_K = 0xFF6B
def bank1_off(addr):
    return addr - 0x8000 + BANK1_K

H_C0A5_OFF = bank1_off(0xC0A5)
H_C0A5_OLD = bytes.fromhex("90ea90")   # MOV DPTR,#0xea90

CAVE = 0x6800            # roomy empty slot below the 0x7000 string area
STR_C0 = 0x7040          # "\r\n[C0 "
LBL_E90 = 0x7050         # "e90="
LBL_ST = 0x7058          # " st="
LBL_OP = 0x7060          # " op="
LBL_E80 = 0x7068         # " e80="
LBL_E81 = 0x7070         # " e81="
STR_END = 0x7078         # "]\x00"

# Preserve everything the cave touches (mid-interrupt).
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    # DPX assumed 0 on entry; these are all plain-XDATA regs (EA8x/EAxx/0x0Bxx).
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def build_hook():
    code = bytearray()
    # mov DPX,#0 (cave reads plain XDATA only)
    code += bytes([0x75, 0x93, 0x00])
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += puts_code(STR_C0)
    code += puts_code(LBL_E90); code += puthex_xdata(0xEA90)
    code += puts_code(LBL_ST);  code += puthex_xdata(0x0B02)
    code += puts_code(LBL_OP);  code += puthex_xdata(0x0B03)
    code += puts_code(LBL_E80); code += puthex_xdata(0xEA80)
    code += puts_code(LBL_E81); code += puthex_xdata(0xEA81)
    code += puts_code(STR_END)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += H_C0A5_OLD                             # replay MOV DPTR,#0xea90
    code += b"\x22"                                # ret
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
        raise ValueError(f"{name} site mismatch at 0x{off:05x}: found {found.hex()} != {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "fw_tinygrad.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "fw_c0a5.bin"
    data = bytearray(open(src, "rb").read())
    body, was_wrapped = unwrap_image(data)

    write_cave(body, STR_C0, b"\r\n[C0 \x00", "C0 str")
    write_cave(body, LBL_E90, b"e90=\x00", "e90")
    write_cave(body, LBL_ST, b" st=\x00", "st")
    write_cave(body, LBL_OP, b" op=\x00", "op")
    write_cave(body, LBL_E80, b" e80=\x00", "e80")
    write_cave(body, LBL_E81, b" e81=\x00", "e81")
    write_cave(body, STR_END, b"]\x00", "end")

    hook = build_hook()
    write_cave(body, CAVE, hook, "c0a5 hook")
    patch_site(body, H_C0A5_OFF, H_C0A5_OLD, CAVE, "c0a5")

    out = wrap_body(body) if was_wrapped else bytes(body)
    open(dst, "wb").write(out)
    print(f"[patch_stock_c0a5] {src} -> {dst}  ({len(out)} bytes, wrapped={was_wrapped})")
    print(f"  hook @CODE_BANK1::c0a5 (body 0x{H_C0A5_OFF:05x}) -> cave 0x{CAVE:04x} ({len(hook)} B)")
    print(f"  prints [C0 e90=.. st=.. op=.. e80=.. e81=..] on each mailbox entry")


if __name__ == "__main__":
    main()
