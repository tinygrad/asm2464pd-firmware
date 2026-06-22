#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer: DOES the CM 4CC-command dispatcher (cm_command_dispatch
@CODE_BANK1::d283) — and in particular cm_RXCM (@CODE_BANK1::cc86, the primary-lane ORIENTATION
COMMIT: phy_lane_gate=1 + C2C3.0/C343.0) — actually EXECUTE during stock's successful USB4 bond?

WHY THIS EXISTS (the dynamic-execution gap):
  Handmade reaches A=0202 (both lanes CL0) but the host never posts SB[0x66]=01 (Lane Bonded).
  The af38 read-branch in STOCK, on a host descriptor of type-8 ("cm8"), calls cm_command_dispatch()
  which matches the 4-char command at XDATA[0x0810] against cm_cmd_table and jumps to a cm_<KEY>
  handler. The "RXCM" key -> cm_RXCM = the per-lane orientation commit handmade OMITS (it only
  prints [af38-A:cm8]).  Prior agents checked the af38 desc-type STREAM and SB STATE but NEVER
  instrumented the CM-command dispatch itself, so whether stock runs cm_RXCM during the bond is
  UNVERIFIED.  This tracer answers it directly.

HOOKS (both bank1; body off = addr-0x8000+0xFF6B):
  1) cm_command_dispatch ENTRY @CODE_BANK1::d283.  Displaced head = `90 08 10` (MOV DPTR,#0x810,
     3 bytes = LCALL size).  At entry XDATA[0x0810..0x0813] holds the incoming 4CC command.
     Prints:  \r\n[CMD <0810><0811><0812><0813> 9f8=<09f8>]
     (9f8 = the gate that, if !=0, sends it to the default path instead of the keyed dispatch.)
  2) cm_RXCM ENTRY @CODE_BANK1::cc86.  Displaced head = `90 c6 db` (MOV DPTR,#0xc6db, 3 bytes).
     Prints:  \r\n[RXCM 6db=<C6DB> A0A1=<2840.SB? no> ab3=<0AB3>]  -- before it commits, so ab3/or
     are the PRE-commit values; the POST values show up on the next af38 [af ... g= or=] line.
     Prints:  \r\n[RXCM 6db=<C6DB> 814=<0814><0815> ab3=<0AB3>]

Both sites are LOW-FREQUENCY (only when the host posts a CM command), so printing every entry is
safe — matches the c0a5 tracer rationale.  Runs mid-interrupt: PUSH/POPs every touched reg, sets
DPX=0 (all logged regs are plain XDATA), replays the displaced head, RETs.
"""
import sys
import zlib

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7

BANK1_K = 0xFF6B
def bank1_off(addr):
    return addr - 0x8000 + BANK1_K

H_CMD_OFF = bank1_off(0xD283)
H_CMD_OLD = bytes.fromhex("900810")    # MOV DPTR,#0x810
H_RXCM_OFF = bank1_off(0xCC86)
H_RXCM_OLD = bytes.fromhex("90c6db")   # MOV DPTR,#0xc6db

CAVE_CMD = 0x6800        # roomy empty slot below the 0x7000 string area
CAVE_RXCM = 0x6900

STR_CMD = 0x7040         # "\r\n[CMD "
LBL_9F8 = 0x7050         # " 9f8="
STR_RXCM = 0x7058        # "\r\n[RXCM "
LBL_814 = 0x7068         # " 814="
LBL_AB3 = 0x7070         # " ab3="
STR_END = 0x7078         # "]\x00"

PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def build_cmd_hook():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])
    code += puts_code(STR_CMD)
    code += puthex_xdata(0x0810); code += puthex_xdata(0x0811)
    code += puthex_xdata(0x0812); code += puthex_xdata(0x0813)
    code += puts_code(LBL_9F8); code += puthex_xdata(0x09F8)
    code += puts_code(STR_END)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_CMD_OLD                              # replay MOV DPTR,#0x810
    code += b"\x22"
    return bytes(code)


def build_rxcm_hook():
    code = bytearray()
    code += bytes([0x75, 0x93, 0x00])
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])
    code += puts_code(STR_RXCM)
    code += puthex_xdata(0xC6DB)
    code += puts_code(LBL_814); code += puthex_xdata(0x0814); code += puthex_xdata(0x0815)
    code += puts_code(LBL_AB3); code += puthex_xdata(0x0AB3)
    code += puts_code(STR_END)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_RXCM_OLD                             # replay MOV DPTR,#0xc6db
    code += b"\x22"
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
    dst = sys.argv[2] if len(sys.argv) > 2 else "fw_cmcmd.bin"
    data = bytearray(open(src, "rb").read())
    body, was_wrapped = unwrap_image(data)

    write_cave(body, STR_CMD, b"\r\n[CMD \x00", "CMD str")
    write_cave(body, LBL_9F8, b" 9f8=\x00", "9f8")
    write_cave(body, STR_RXCM, b"\r\n[RXCM \x00", "RXCM str")
    write_cave(body, LBL_814, b" 814=\x00", "814")
    write_cave(body, LBL_AB3, b" ab3=\x00", "ab3")
    write_cave(body, STR_END, b"]\x00", "end")

    hook_cmd = build_cmd_hook()
    write_cave(body, CAVE_CMD, hook_cmd, "cmd hook")
    patch_site(body, H_CMD_OFF, H_CMD_OLD, CAVE_CMD, "cm_command_dispatch")

    hook_rxcm = build_rxcm_hook()
    write_cave(body, CAVE_RXCM, hook_rxcm, "rxcm hook")
    patch_site(body, H_RXCM_OFF, H_RXCM_OLD, CAVE_RXCM, "cm_RXCM")

    out = wrap_body(body) if was_wrapped else bytes(body)
    open(dst, "wb").write(out)
    print(f"[patch_stock_cmcmd] {src} -> {dst}  ({len(out)} bytes, wrapped={was_wrapped})")
    print(f"  hook @cm_command_dispatch d283 (body 0x{H_CMD_OFF:05x}) -> cave 0x{CAVE_CMD:04x} ({len(hook_cmd)} B)")
    print(f"  hook @cm_RXCM cc86 (body 0x{H_RXCM_OFF:05x}) -> cave 0x{CAVE_RXCM:04x} ({len(hook_rxcm)} B)")
    print(f"  prints [CMD <4cc> 9f8=..] on every CM command + [RXCM 6db=.. 814=.. ab3=..] on every orientation commit")


if __name__ == "__main__":
    main()
