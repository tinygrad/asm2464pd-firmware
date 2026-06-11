#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin code-cave trace of the USB4 SB CONNECT-DESCRIPTOR window.

THE QUESTION (handmade wall, project_usb4_tunnel.md SESSION 2026-06-11k):
  Handmade's USB4 connect path is now FAITHFUL and the FSM reaches state-3
  ([ConnRout]) but STALLS at the `0x0777 == 0x0C` gate because the Intel MTL TB4
  host posts NO SB connect descriptor to handmade:
      SB[0x28]=00 (descriptor-valid), SB[0x18]=00 (connect descriptor),
      SB-plane-2 [0x2a00..] = all zeros.
  The eaac block-copy that fills 0x0777 is reached from cd3f only when the host
  drives the SB[0x28].3 transport edge (sb_transport_substate_poll) and the
  descriptor passes cd3f's gate (SB[0x28].4 + SB[0x18] connect bits).
  But STOCK reaches the GPU on this EXACT host -> stock DOES obtain the
  descriptor. This patch TRACES STOCK to see HOW: does the host post the
  descriptor (under a trigger/handshake handmade doesn't fire), or does stock
  write SB[0x28]/the SB-plane-2 descriptor device-side?

FOUR CODE-CAVE HOOKS (bank1; ghidra.c == this image; bank1 logical addr A ->
body offset A - 0x8000 + 0xFF6B; verified vs the a0e7 [===SB Con===] anchor):

  1) a066 [===SB Con===] PRINT SITE  (body 0x12052, the 6-byte R3/R2/R1 string
     setup `7b ff 7a 20 79 56`, same anchor as patch_sbtrace.py).
     = state BEFORE the descriptor is processed, at the connect instant.
     Dumps: the connect/descriptor SB regs SB[0x18/19], SB[0x28/29],
     SB[0x2A/2B], SB[0x2C/2D]; SB-plane-2 [0x2a00..0x2a0F] + [0x2b00..0x2b0F];
     the FSM walk 0x0777/0x0775/0x0758/0x06ED/0x06EE/0x06F0; and the router-op /
     SB-transport mailbox CE88/CE89/EA80/EA81/EA90/EC06/EC04.

  2) cd3f ENTRY  (body 0x14caa, `12 ed d9` LCALL 0xedd9).
     cd3f = the per-port connect-descriptor READER, called from
     sb_transport_substate_poll on the SB[0x28].3 / SB[0x2A].3 transport edge.
     This fires WHEN stock reads/relays the descriptor. Dumps the same connect
     SB regs + 0x06F0 (port) + the mailbox, so we see what the descriptor reads
     return AT the read instant and whether a host router-op preceded it.

  3) eaac ENTRY  (body 0x16a17, `90 07 75` MOV DPTR,#0x775).
     eaac = the SOLE writer of 0x0777 (block-copy 0x40 bytes from SB-plane-2 ->
     0x0777). Reached from cd3f when `(SB[0x18]&0x60)!=0x60 && (SB[0x18]&1)==0`.
     If this hook fires AT ALL on stock, the host posted a descriptor. Dumps the
     SB-plane-2 source candidates [0x2a00..0x2a0F]/[0x2b00..0x2b0F] (the BYTES
     that get copied to 0x0777) + 0x06F0 (port) + 0x0775.

  4) a7de ENTRY  (body 0x12749, `90 07 58` MOV DPTR,#0x758).
     a7de = cm_conn_routing_setup, the [ConnRout] state-3 FSM that GATES on
     0x0777==0x0C. Dumps 0x0758 (FSM state)/0x0777/0x0775/0x06ED + 0x0765/0x0766
     (connect-present) + SB[0x28]/SB[0x18], so we see whether 0x0777 reaches 0x0C
     and whether SB[0x28].4 is set when stock confirms.

STOCK'S OWN PRINTS ARE KEPT (the hooks replay the replaced bytes), so
[SB Init]/[===SB Con===]/[SB P0x]/[ConnRout]/[PcieTunnel-*] interleave with the
[D0:]/[D1:]/[D2:]/[D3:] trace lines for time-alignment.

LINE FORMATS (hex; DPX restored to 0 before each print):
  [D0:<SB18><SB19><SB28><SB29><SB2A><SB2B><SB2C><SB2D>|<P2a:16><P2b:16>|
      <0777><0775><0758><06ED><06EE><06F0>|<CE88><CE89><EA80><EA81><EA90><EC06><EC04>]
  [D1:<SB18><SB19><SB28><SB29><SB2A><SB2B><SB2C><SB2D>|06F0=<06F0>|
      <CE88><CE89><EA80><EA81><EA90><EC06><EC04>]
  [D2:P2a<0x2a00..0x2a0F:16>|P2b<0x2b00..0x2b0F:16>|06F0=<06F0> 0775=<0775>]
  [D3:758=<0758> 777=<0777> 775=<0775> 6ED=<06ED> 765=<0765> 766=<0766>
      SB28=<SB28> SB18=<SB18>]

Build:
    python3 app/patch_descrtrace.py fw_tinygrad.bin /tmp/fw_descrtrace.bin

FINDINGS (stock run, 2026-06-11, GPU 1002:7590 + tbt router 1-1 CONFIRMED UP):
  D0 @[===SB Con===]:  SB-plane-2[0x2a00..]=ALL ZEROS, 0x0777=0x55 -> IDENTICAL
     to handmade's stuck state. The descriptor is NOT present at connect.
  D1 cd3f reads: the host's connect descriptor (the per-port SB-page field cd3f
     reads; my SB[0x18] dump tracks it) climbs 00 -> 05 -> 0x63 across successive
     reads, purely via the SB SIDEBAND TRANSPORT. The router-op / SB-transport
     MAILBOX (CE88/CE89/EA80/EA81/EA90/EC06/EC04) is ALL ZERO the WHOLE run ->
     there is NO router-op exchange. The host posts the descriptor over sideband.
  D2 eaac block-copy: SB-plane-2[0x2a00]=0x0C (then 0x0D...) -> NON-ZERO. eaac
     FIRES 157x on stock (vs 0x on handmade). Plane-2 is populated DEVICE-SIDE.
  D3 a7de FSM: 0x0777 walks 0x55 -> 0x0C; gate passes; [ConnRout]; 0x0758 -> 0.

  VERDICT: the missing piece is a DEVICE-SIDE descriptor writer handmade DROPPED.
  cd3f dispatches the host descriptor to THREE branches; handmade implements only
  ebb5 ((0x752&0x60)==0x60 -> 0x0765=1) and eaac ((0x752&1)==0 -> 0x0777 copy),
  but DROPS the third: CODE_BANK1::af38 ((0x752&1)!=0, bit6+bits1-4 set) which
  WRITES the SB-plane-2 descriptor (r3_write_dispatch to the "2" plane 0x2900/
  0x2902 + the per-byte copy loop) from the host's SB[0x18] descriptor data. With
  af38 absent, plane-2 stays zero on handmade -> eaac copies zeros -> 0x0777=0x55
  -> state-3 stall. The host DOES post (over sideband, no mailbox); stock then
  TRANSLATES that into the plane-2 descriptor via af38; handmade omits af38.
  FIX = port CODE_BANK1::af38 into handmade's cd3f dispatch (the (0x752&1)!=0
  branch), faithfully (r3-plane "2" accessor at 0x2900/0x2a00 + the descriptor
  copy loops), so plane-2 0x2a00 -> 0x0C and eaac then yields 0x0777=0x0C.
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_descrtrace.bin"

# Stock UART helpers (low shared region, <0x8000 -> flat in both banks).
UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# Bank1 file-offset mapping: body_off = addr - 0x8000 + K (K verified 0xFF6B).
BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


# ---- Hook sites (body offsets) + the original bytes we replace + replay. -----
# 1) a066 [===SB Con===] string-ptr setup (6 bytes), same as patch_sbtrace.
H_A066_OFF = 0x12052
H_A066_OLD = bytes.fromhex("7bff7a207956")
# 2) cd3f entry: LCALL 0xedd9 (3 bytes).
H_CD3F_OFF = bank1_off(0xCD3F)
H_CD3F_OLD = bytes.fromhex("12edd9")
# 3) eaac entry: MOV DPTR,#0x775 (3 bytes).
H_EAAC_OFF = bank1_off(0xEAAC)
H_EAAC_OLD = bytes.fromhex("900775")
# 4) a7de entry: MOV DPTR,#0x758 (3 bytes).
H_A7DE_OFF = bank1_off(0xA7DE)
H_A7DE_OLD = bytes.fromhex("900758")

# ---- Caves (low shared zero region 0x5E00..0x8000 = 8.7KB, flat both banks). --
SHARED_DUMP = 0x5E00         # shared paged-block dump subroutine (~34 B)
# Prefix strings.
STR_D0 = 0x5E40
STR_D1 = 0x5E48
STR_D2 = 0x5E50
STR_D3 = 0x5E58
STR_BAR = 0x5E60             # "|"
STR_P2A = 0x5E62            # "P2a"
STR_P2B = 0x5E66            # "P2b"
# Hooks: a066 ~344 B, cd3f ~264 B, eaac ~132 B, a7de ~155 B. 0x200 spacing each.
CAVE_A066 = 0x5F00
CAVE_CD3F = 0x6100
CAVE_EAAC = 0x6300
CAVE_A7DE = 0x6500

# SB page-1 accessor: DPX=1, XDATA 0x2800+off (verified vs handmade SB_RD).
SB_DPX = 0x01


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_a_imm(val):
    return bytes([0x74, val & 0xFF])


def emit_char(ch):
    if isinstance(ch, str):
        ch = ord(ch)
    return mov_dptr(UART_TX) + mov_a_imm(ch) + b"\xf0"


def puthex_xdata(addr):
    # DPX=0 plain XDATA read into R7, then print.
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


def puthex_sb(addr):
    # DPX=1 paged read of XDATA 0x2800+off, restore DPX=0, then print.
    return (
        bytes([0x75, 0x93, SB_DPX])               # mov DPX,#1
        + mov_dptr(addr) + b"\xe0\xff"            # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])               # mov DPX,#0
        + lcall(UART_PUTHEX)
    )


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF])
        + bytes([0x7A, (addr >> 8) & 0xFF])
        + bytes([0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the dump scratch (0x20/0x21/0x22).
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x93)


def build_shared_dump():
    """Subroutine: print R3 hex bytes from paged XDATA (DPH:DPL = 0x21:0x20) at
    DPX = (acc on entry); restores DPX=0 before each puthex (the print's movx to
    UART_TX is DPX-sensitive). Count in 0x22. Caller sets 0x20/0x21/0x22 + A=DPX.
    """
    code = bytearray()
    code += bytes([0xF5, 0x93])                    # mov DPX,a      (set plane)
    loop = len(code)
    code += bytes([0x75, 0x93, 0x00])              # placeholder; we re-set below
    # We must re-assert the plane each iteration because puthex restores DPX=0.
    # Rework: store the plane in 0x23.
    code = bytearray()
    code += bytes([0xF5, 0x23])                    # mov 0x23,a     (save plane)
    loop = len(code)
    code += bytes([0xE5, 0x23, 0xF5, 0x93])        # mov a,0x23 ; mov DPX,a
    code += b"\xe5\x20\xf5\x82"                     # mov a,0x20 ; mov DPL,a
    code += b"\xe5\x21\xf5\x83"                     # mov a,0x21 ; mov DPH,a
    code += b"\xe0"                                 # movx a,@dptr   (paged read)
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0     (restore)
    code += b"\xff" + lcall(UART_PUTHEX)           # mov r7,a ; lcall puthex
    code += b"\x05\x20"                             # inc 0x20
    code += b"\xe5\x20"                             # mov a,0x20
    jnz = len(code)
    code += b"\x70\x00"                             # jnz no_carry
    code += b"\x05\x21"                             # inc 0x21
    no_carry = len(code)
    code[jnz + 1] = (no_carry - (jnz + 2)) & 0xFF
    djnz = len(code)
    code += b"\xd5\x22\x00"                         # djnz 0x22, loop
    code[djnz + 2] = (loop - (djnz + 3)) & 0xFF
    code += b"\x22"                                 # ret
    return bytes(code)


def dump_block(start, count, dpx):
    """Call the shared dump subroutine for `count` bytes from `start` at `dpx`."""
    return (
        bytes([0x75, 0x20, start & 0xFF])          # mov 0x20,#lo
        + bytes([0x75, 0x21, (start >> 8) & 0xFF]) # mov 0x21,#hi
        + bytes([0x75, 0x22, count & 0xFF])        # mov 0x22,#count
        + mov_a_imm(dpx)                           # mov a,#dpx
        + lcall(SHARED_DUMP)
    )


# Note: PRESERVE includes 0x23 implicitly? No -> add it so the shared dump's
# plane-save byte is preserved.
PRESERVE = PRESERVE + (0x23,)


def _wrap_hook(body_payload, old_bytes):
    """Push preserve, run payload, pop preserve, replay old bytes, ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])                   # push direct
    code += bytes([0x75, 0x93, 0x00])              # mov DPX,#0  (plain reads)
    code += body_payload
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])                   # pop direct
    code += old_bytes                              # replay replaced bytes
    code += b"\x22"                                # ret
    return bytes(code)


# Connect/descriptor SB regs (page-1, DPX=1) dumped by D0 + D1.
SB_CONN = [0x2818, 0x2819, 0x2828, 0x2829, 0x282A, 0x282B, 0x282C, 0x282D]
# Router-op / SB-transport mailbox (plain XDATA, DPX=0).
MAILBOX = [0xCE88, 0xCE89, 0xEA80, 0xEA81, 0xEA90, 0xEC06, 0xEC04]


def build_a066_payload():
    """[D0: SB conn regs | P2a/P2b 16 each | FSM | mailbox]."""
    code = bytearray()
    code += puts_code(STR_D0)
    for a in SB_CONN:
        code += puthex_sb(a)
    code += puts_code(STR_BAR)
    code += dump_block(0x2A00, 0x10, SB_DPX)       # SB-plane-2 port0
    code += dump_block(0x2B00, 0x10, SB_DPX)       # SB-plane-2 port1
    code += puts_code(STR_BAR)
    for a in (0x0777, 0x0775, 0x0758, 0x06ED, 0x06EE, 0x06F0):
        code += puthex_xdata(a)
    code += puts_code(STR_BAR)
    for a in MAILBOX:
        code += puthex_xdata(a)
    code += emit_char(']') + emit_char('\r') + emit_char('\n')
    return bytes(code)


def build_cd3f_payload():
    """[D1: SB conn regs | 06F0 | mailbox] - at the descriptor READ instant."""
    code = bytearray()
    code += puts_code(STR_D1)
    for a in SB_CONN:
        code += puthex_sb(a)
    code += puts_code(STR_BAR)
    code += puthex_xdata(0x06F0)
    code += puts_code(STR_BAR)
    for a in MAILBOX:
        code += puthex_xdata(a)
    code += emit_char(']') + emit_char('\r') + emit_char('\n')
    return bytes(code)


def build_eaac_payload():
    """[D2: P2a 16 | P2b 16 | 06F0 0775] - the BLOCK-COPY source bytes."""
    code = bytearray()
    code += puts_code(STR_D2)
    code += dump_block(0x2A00, 0x10, SB_DPX)
    code += puts_code(STR_BAR)
    code += dump_block(0x2B00, 0x10, SB_DPX)
    code += puts_code(STR_BAR)
    code += puthex_xdata(0x06F0)
    code += puthex_xdata(0x0775)
    code += emit_char(']') + emit_char('\r') + emit_char('\n')
    return bytes(code)


def build_a7de_payload():
    """[D3: FSM gate state + connect-present + SB28/SB18]."""
    code = bytearray()
    code += puts_code(STR_D3)
    for a in (0x0758, 0x0777, 0x0775, 0x06ED, 0x0765, 0x0766):
        code += puthex_xdata(a)
    code += puts_code(STR_BAR)
    code += puthex_sb(0x2828)                       # SB[0x28]
    code += puthex_sb(0x2818)                       # SB[0x18]
    code += emit_char(']') + emit_char('\r') + emit_char('\n')
    return bytes(code)


def wrap_body(body):
    return (
        len(body).to_bytes(4, "little")
        + body
        + bytes([0xA5, sum(body) & 0xFF])
        + zlib.crc32(body).to_bytes(4, "little")
    )


def unwrap_image(data):
    if len(data) >= 10:
        body_len = int.from_bytes(data[:4], "little")
        footer = 4 + body_len
        if body_len + 10 == len(data) and data[footer] == 0xA5:
            body = data[4:footer]
            checksum = data[footer + 1]
            crc = int.from_bytes(data[footer + 2:footer + 6], "little")
            if checksum != (sum(body) & 0xFF):
                raise ValueError("wrapped firmware checksum mismatch")
            if crc != zlib.crc32(body):
                raise ValueError("wrapped firmware crc mismatch")
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
        raise ValueError(
            f"{name} site mismatch at body 0x{off:05x}: found {found.hex()}, "
            f"expected {old.hex()}")
    repl = bytearray(lcall(cave))
    while len(repl) < len(old):
        repl += b"\x00"                            # NOP-pad to original length
    body[off:off + len(old)] = repl


def apply_patch(body):
    # Shared dump subroutine + prefix strings.
    write_cave(body, SHARED_DUMP, build_shared_dump(), "shared dump")
    write_cave(body, STR_D0, b"[D0:\x00", "D0")
    write_cave(body, STR_D1, b"[D1:\x00", "D1")
    write_cave(body, STR_D2, b"[D2:\x00", "D2")
    write_cave(body, STR_D3, b"[D3:\x00", "D3")
    write_cave(body, STR_BAR, b"|\x00", "bar")
    write_cave(body, STR_P2A, b"P2a\x00", "p2a")
    write_cave(body, STR_P2B, b"P2b\x00", "p2b")

    hooks = [
        ("a066", H_A066_OFF, H_A066_OLD, CAVE_A066, build_a066_payload()),
        ("cd3f", H_CD3F_OFF, H_CD3F_OLD, CAVE_CD3F, build_cd3f_payload()),
        ("eaac", H_EAAC_OFF, H_EAAC_OLD, CAVE_EAAC, build_eaac_payload()),
        ("a7de", H_A7DE_OFF, H_A7DE_OLD, CAVE_A7DE, build_a7de_payload()),
    ]
    info = []
    for name, off, old, cave, payload in hooks:
        hook = _wrap_hook(payload, old)
        write_cave(body, cave, hook, f"{name} hook")
        patch_site(body, off, old, cave, name)
        info.append((name, off, cave, len(hook)))
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    info = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)

    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    for name, off, cave, hlen in info:
        print(f"  {name}: site body 0x{off:05x} -> lcall 0x{cave:04x} "
              f"(hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print("  D0 a066[===SB Con===]: SB18/19/28/29/2A/2B/2C/2D | P2a P2b | "
          "0777/0775/0758/06ED/06EE/06F0 | mailbox")
    print("  D1 cd3f(descriptor read): SB conn regs | 06F0 | mailbox")
    print("  D2 eaac(block-copy src):  P2a P2b | 06F0 0775")
    print("  D3 a7de(ConnRout FSM):    0758/0777/0775/06ED/0765/0766 | SB28 SB18")


if __name__ == "__main__":
    main()
