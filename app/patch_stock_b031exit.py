#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave tracer — b031 EXIT SB-read probe (THE decisive experiment).

QUESTION: when stock runs bank0 b031 (the SB-transport / in-band control-adapter REINIT),
does stock's OWN next SB register READ succeed immediately AT b031's exit, with NO host
follow-up packet -- or does it stall (handmade HARD-HANGS on the next SB_RD after b031)?

WHY b031 EXIT (not e52d/e530): b031 is reached on stock's enumerating path via THREE
callers -- d894 (boot superloop init), da9f (sb_phy_link_bringup), and e52d (bond ISR).
A prior probe at e52d/e530 NEVER fired during a full GPU enumeration -> stock does NOT
bring the responder up via e52d at bond-complete; it uses the d894/da9f path. Hooking
b031's LAST instruction (b101 `LJMP 0xe74e`) catches EVERY b031 invocation regardless of
caller, and dumps the SB-read-engine state the instant b031's writes (incl e5b0's
descriptor reset) have all landed.

HOOK SITE (bank0, body off = ghidra addr; this fw is a WRAPPED image, 4B header + body +
10B A5/checksum/CRC footer -- we unwrap, patch BODY space, rewrap so boot accepts it):
  CODE:b101  `02 e7 4e` (LJMP 0xe74e) -> LCALL cave. Cave: push-preserve; live SB read
  (SB[0xA0]) with a sentinel '!' printed AFTER the read returns (proves the read did NOT
  stall); dump descriptor-engine regs P12[0x03]/[0x8F]/[0x90] + EC06/EA90/E302; pop;
  then `LJMP 0xe74e` (replay b031's real tail-jump). Tag [B031x].

DECISIVE READOUT:
  * `[B031x sb=<val>!...]` printed (sentinel '!' present) AND stock enumerates the GPU
    -> stock's SB read SURVIVES b031 with no host follow-up -> the handmade hang is a
    device-side TRANSCRIPTION/CONTEXT bug (VERDICT A).
  * `[B031x` truncates before the '!' / never appears while bonding -> the read stalled
    -> host-gated (VERDICT B). (If it only appears at BOOT and stock still enumerates,
    that still proves b031's SB read works standalone -> verdict A.)

Build: python3 app/patch_stock_b031exit.py fw_tinygrad.bin /tmp/fw_b031exit.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_b031exit.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# Body-space (after unwrap): bank0 ghidra addr == body offset; bank1 = ghidra + 0x7F6B.
def bank1_off(addr):
    return addr + 0x7F6B


H_B101_OFF = 0xB101
H_B101_OLD = bytes.fromhex("02e74e")       # LJMP 0xe74e (b031's tail-jump)
E74E = 0xE74E

# c4aa (tunnel_pcie_status_banner entry, the [*** PCIE Gen ... ***] printer) -- a reliable
# POST-bond / post-tunnel / post-UART point. Hooked to print the b031 invocation counter so
# we learn whether b031 ran AT ALL (incl at boot, pre-UART) even though [B031x] never prints
# live. The banner fires once per tunnel bring-up.
H_C4AA_OFF = bank1_off(0xC4AA)
H_C4AA_OLD = bytes.fromhex("7bff7a38")     # MOV R3,#0xff ; MOV R2,#0x38 (4 bytes)
CAVE_C4AA = 0x6800
STR_PCTR = 0x70B0          # "\r\n[b031#="

# Caves + strings in the zero padding at body 0x6800-0x7100 (ghidra addr == body off).
STR_B031X = 0x7040         # "\r\n[B031x"
LBL_SB = 0x7050            # " sb="
LBL_1203 = 0x7058          # " 1203="
LBL_128F = 0x7064          # " 128F="
LBL_1290 = 0x7070          # " 1290="
LBL_EC06 = 0x707C          # " EC06="
LBL_EA90 = 0x7088          # " EA90="
LBL_E302 = 0x7094          # " E302="
LBL_CTR = 0x70A0           # " #="

CAVE = 0x6C00
SB_DPX = 0x01
CTR = 0x8830               # free-running 16-bit invocation counter (scratch SRAM)

STRINGS = {
    STR_B031X: "\r\n[B031x",
    LBL_SB: " sb=", LBL_1203: " 1203=", LBL_128F: " 128F=", LBL_1290: " 1290=",
    LBL_EC06: " EC06=", LBL_EA90: " EA90=", LBL_E302: " E302=", LBL_CTR: " #=",
    STR_PCTR: "\r\n[b031#=",
}


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def ljmp(addr):
    return bytes([0x02, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF]) + lcall(UART_PUTS)


def puthex_xdata(addr):
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_p1(addr):
    return (bytes([0x75, 0x93, SB_DPX]) + mov_dptr(addr) + b"\xe0\xff"
            + bytes([0x75, 0x93, 0x00]) + lcall(UART_PUTHEX))


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x93)


def build_cave():
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += puts_code(STR_B031X)
    # invocation counter (hi then lo)
    code += puts_code(LBL_CTR)
    code += mov_dptr(CTR) + b"\xe0\x04\xf0"        # inc lo
    code += b"\x70\x05" + mov_dptr(CTR + 1) + b"\xe0\x04\xf0"  # carry -> inc hi
    code += puthex_xdata(CTR + 1) + puthex_xdata(CTR)
    # live SB read + sentinel
    code += puts_code(LBL_SB)
    code += puthex_p1(0x2800 + 0xA0)               # SB[0xA0] via DPX=1 paged read
    code += mov_dptr(UART_TX) + b"\x74\x21\xf0"     # '!' (printed only if the read returned)
    # descriptor-engine regs e5b0 touches + transport state
    code += puts_code(LBL_1203); code += puthex_p1(0x1203)
    code += puts_code(LBL_128F); code += puthex_p1(0x128F)
    code += puts_code(LBL_1290); code += puthex_p1(0x1290)
    code += puts_code(LBL_EC06); code += puthex_xdata(0xEC06)
    code += puts_code(LBL_EA90); code += puthex_xdata(0xEA90)
    code += puts_code(LBL_E302); code += puthex_xdata(0xE302)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0 (safe pops)
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += ljmp(E74E)                             # replay b031's tail-jump LJMP 0xe74e
    return bytes(code)


def build_c4aa_cave():
    """[Pend Int] entry hook: print the b031 invocation counter (CTR), then replay the
    displaced head + RET. Change-gated would hide repeats; we print every call (cheap, the
    counter is what matters -- a nonzero count proves b031 ran, at boot pre-UART)."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0
    code += puts_code(STR_PCTR)
    code += puthex_xdata(CTR + 1) + puthex_xdata(CTR)   # b031 call count (hi,lo)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'
    code += bytes([0x75, 0x93, 0x00])
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_C4AA_OLD                             # replay MOV R3,#0xff ; MOV R2,#0x20
    code += b"\x22"                                # ret
    return bytes(code)


def wrap_body(body):
    return (len(body).to_bytes(4, "little") + bytes(body)
            + bytes([0xA5, sum(body) & 0xFF]) + zlib.crc32(bytes(body)).to_bytes(4, "little"))


def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little")
        footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            if data[footer + 1] != (sum(body) & 0xFF):
                raise ValueError("wrapped firmware checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
                raise ValueError("wrapped firmware crc mismatch")
            return bytearray(body), True
    return bytearray(data), False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile", nargs="?", default=str(DEFAULT_IN))
    ap.add_argument("outfile", nargs="?", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    body, wrapped = unwrap_image(bytearray(Path(args.infile).read_bytes()))

    assert body[H_B101_OFF:H_B101_OFF + 3] == H_B101_OLD, (
        f"b101 head mismatch @body 0x{H_B101_OFF:x}: {body[H_B101_OFF:H_B101_OFF+3].hex()}")

    for addr, s in STRINGS.items():
        b = s.encode() + b"\x00"
        assert not any(body[addr:addr + len(b)]), f"string 0x{addr:x} not empty"
        body[addr:addr + len(b)] = b

    cave = build_cave()
    assert not any(body[CAVE:CAVE + len(cave)]), "cave not empty"
    assert CAVE + len(cave) <= 0x7000, "cave overruns string region"
    body[CAVE:CAVE + len(cave)] = cave
    body[H_B101_OFF:H_B101_OFF + 3] = lcall(CAVE)   # LJMP 0xe74e -> LCALL cave (cave re-LJMPs e74e)

    # c4aa banner hook: print the b031 call counter.
    assert body[H_C4AA_OFF:H_C4AA_OFF + 4] == H_C4AA_OLD, (
        f"c4aa head mismatch @body 0x{H_C4AA_OFF:x}: {body[H_C4AA_OFF:H_C4AA_OFF+4].hex()}")
    cave2 = build_c4aa_cave()
    assert not any(body[CAVE_C4AA:CAVE_C4AA + len(cave2)]), "c4aa cave not empty"
    assert CAVE_C4AA + len(cave2) <= CAVE, "c4aa cave overruns b031 cave"
    body[CAVE_C4AA:CAVE_C4AA + len(cave2)] = cave2
    body[H_C4AA_OFF:H_C4AA_OFF + 4] = lcall(CAVE_C4AA) + b"\x00"   # 4-byte head -> LCALL + NOP

    out = wrap_body(body) if wrapped else bytes(body)
    Path(args.outfile).write_bytes(out)
    print(f"wrote {args.outfile} ({len(out)} bytes, wrapped={wrapped})  crc32={zlib.crc32(out) & 0xffffffff:08x}")
    print(f"  b031 EXIT hook @body 0x{H_B101_OFF:x} (LJMP->LCALL 0x{CAVE:x}), cave {len(cave)} B")
    print(f"  c4aa hook @body 0x{H_C4AA_OFF:x} (LCALL 0x{CAVE_C4AA:x}), cave {len(cave2)} B")


if __name__ == "__main__":
    main()
