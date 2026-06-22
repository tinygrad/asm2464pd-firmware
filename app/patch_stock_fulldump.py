#!/usr/bin/env python3
"""
STOCK fw_tinygrad.bin EXHAUSTIVE full-address-space code-cave dumper.

Goal (the "turn every last stone" pass): dump the FULL register space at the
single-lane CL0 bond frame (SB[0xA0]==0x02), covering the ranges prior subset
dumps (patch_stock_sfr.py / patch_stock_sball.py) SKIPPED:

  * FULL adapter-CS 0x1200-0x15FF (every adapter's CS block incl control adapter #0)
  * FULL SFR-mapped pages: B4xx C2xx C3xx C6xx C8xx CAxx CCxx E3xx E7xx EAxx ECxx
  * FULL page-1 SB 0x2800-0x28FF (all 256 SB regs) + page-1 router agg 0x0100-0x011F
  * CM/transport working buffers 0x0900-0x09FF

patch_stock_sball.py already covers SBALL/XALL/FULL-XDATA(0x0600-0x0BFF); those
are NOT the focus here. This dumper is the COMPLEMENT (the SFR-mapped HW blocks +
the full adapter-CS that no prior dump walked in full), at the A=0202 gate.

DESIGN (small, table-driven, non-intrusive):
  * Hook = CODE_BANK1::a066 ENTRY (proven non-intrusive cave; same as
    patch_stock_sfr.py which enumerated the GPU with the hook live).
  * GATE = fire only when SB[0xA0]==0x02 (the A=0202 CL0 frame) -- the EXACT
    moment handmade stalls, so stock and handmade dumps are at the equivalent
    instant.
  * A BLOCK-DESCRIPTOR TABLE in code memory (4 bytes each: dpx, addrHI, addrLO,
    count). The hook reads PHASE (XDATA 0x0B5A), indexes the table, dumps that
    ONE block, then advances PHASE (mod N). So the hook is small + constant size
    regardless of N, and each firing emits <=128 bytes -> tiny ISR-time UART
    burst -> never stalls the bond. Across the ~315ms CL0 window a066 fires many
    times -> all phases covered.
  * Line format (self-describing so the diff aligns regardless of phase order):
        \r\n[FD <ctr16> <dpx1><addr4>:<hex...>]
    e.g.  [FD 1234 11200:0001....]

Build:
    python3 app/patch_stock_fulldump.py fw_tinygrad.bin /tmp/fw_fulldump.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_fulldump.bin"

UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

BANK1_K = 0xFF6B


def bank1_off(addr):
    return addr - 0x8000 + BANK1_K


H_A66_OFF = bank1_off(0xA066)
H_A66_OLD = bytes.fromhex("e4f54d")

# ---- cave layout -----------------------------------------------------------
SHARED_DUMP = 0x7000        # generalized paged block dump subroutine
STR_FD = 0x7040            # "\r\n[FD "
STR_COLON = 0x7050         # ":"
TABLE = 0x7060             # block descriptor table (4 bytes/entry)

CAVE_A66 = 0x6600

CTR_LO = 0x8830
CTR_HI = 0x8831
# PHASE: the 0x0B5x headroom is NOT all free on stock (0x0B5A is LIVE stock state -- a
# first attempt there stuck the phase at 15). Use the high SRAM scratch region 0x8832,
# adjacent to the 0x8830/0x8831 counter cells which the sibling patches confirmed
# stock-unused.
PHASE = 0x8832             # dump phase (0..N-1)

SB_DPX = 0x01

# ---- exhaustive block list (dpx, start, count<=0x80) -----------------------
BLOCKS = []


def _add_range(dpx, start, total):
    n, a = total, start
    while n > 0:
        c = 0x80 if n >= 0x80 else n
        BLOCKS.append((dpx, a, c))
        a += c
        n -= c


_add_range(1, 0x1200, 0x400)   # FULL adapter-CS 0x1200-0x15FF (incl control adapter #0)
_add_range(1, 0x2800, 0x100)   # FULL page-1 SB 0x00-0xFF
_add_range(1, 0x0100, 0x20)    # page-1 router/link aggregate evt
for page in (0xB400, 0xC200, 0xC300, 0xC600, 0xC800, 0xCA00, 0xCC00,
             0xE300, 0xE700, 0xEA00):
    _add_range(0, page, 0x100)  # FULL SFR-mapped HW pages
_add_range(0, 0xEC00, 0x10)    # EC00-EC0F router-op evt
_add_range(0, 0x0900, 0x100)   # CM/transport cap/state working buffers

N_PHASES = len(BLOCKS)


def lcall(addr):
    return bytes([0x12, (addr >> 8) & 0xFF, addr & 0xFF])


def mov_dptr(addr):
    return bytes([0x90, (addr >> 8) & 0xFF, addr & 0xFF])


def puts_code(addr):
    return (bytes([0x7B, 0xFF, 0x7A, (addr >> 8) & 0xFF, 0x79, addr & 0xFF])
            + lcall(UART_PUTS))


def puthex_xdata(addr):
    return mov_dptr(addr) + b"\xe0\xff" + lcall(UART_PUTHEX)


PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x20, 0x21, 0x22, 0x23, 0x93)


def build_shared_dump():
    """Print R(0x22) hex bytes from paged XDATA (DPH:DPL=0x21:0x20), plane in
    A (saved 0x23). puthex restores DPX=0 so the plane is re-asserted each iter."""
    code = bytearray()
    code += bytes([0xF5, 0x23])                    # mov 0x23,a
    loop = len(code)
    code += bytes([0xE5, 0x23, 0xF5, 0x93])        # mov a,0x23 ; mov DPX,a
    code += b"\xe5\x20\xf5\x82"                     # DPL=0x20
    code += b"\xe5\x21\xf5\x83"                     # DPH=0x21
    code += b"\xe0"                                 # movx a,@dptr
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    code += b"\xff" + lcall(UART_PUTHEX)
    code += b"\x05\x20"                             # inc 0x20
    code += b"\xe5\x20"
    jnz = len(code)
    code += b"\x70\x00"
    code += b"\x05\x21"                             # inc 0x21
    no_carry = len(code)
    code[jnz + 1] = (no_carry - (jnz + 2)) & 0xFF
    djnz = len(code)
    code += b"\xd5\x22\x00"                         # djnz 0x22,loop
    code[djnz + 2] = (loop - (djnz + 3)) & 0xFF
    code += b"\x22"
    return bytes(code)


def emit_counter():
    code = bytearray()
    code += mov_dptr(CTR_LO) + b"\xe0\x04\xf0"
    code += b"\x70\x05"
    code += mov_dptr(CTR_HI) + b"\xe0\x04\xf0"
    code += puthex_xdata(CTR_HI)
    code += puthex_xdata(CTR_LO)
    return bytes(code)


def build_table():
    """4 bytes/entry: dpx, addrHI, addrLO, count."""
    t = bytearray()
    for dpx, start, count in BLOCKS:
        t += bytes([dpx & 0xFF, (start >> 8) & 0xFF, start & 0xFF, count & 0xFF])
    return bytes(t)


def build_a66_hook():
    """push; DPX=0; if SB[0xA0]!=2 skip; else read PHASE, index TABLE via MOVC,
    dump that block, advance PHASE; pop; replay; ret."""
    code = bytearray()
    for d in PRESERVE:
        code += bytes([0xC0, d])
    code += bytes([0x75, 0x93, 0x00])              # DPX=0

    # GATE: SB[0xA0]==0x02 ? Body is >127B so the skip needs an LJMP. Pattern:
    #   cjne a,#2, +3   ; A!=2 -> fall into the LJMP SKIP
    #   sjmp +3         ; A==2 -> hop over the LJMP into the body
    #   ljmp SKIP       ; (the A!=2 path)
    code += bytes([0x75, 0x93, SB_DPX])            # DPX=1
    code += mov_dptr(0x2800 + 0xA0) + b"\xe0"      # movx a,@dptr (SB A0)
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    code += bytes([0xB4, 0x02, 0x03])              # cjne a,#2, +3 (A!=2 -> next 2 instrs)
    code += b"\x80\x03"                            # sjmp +3 (A==2 -> over the ljmp)
    ljmp_skip = len(code)
    code += b"\x02\x00\x00"                        # ljmp SKIP (placeholder)
    # fall-through (A==2): do the dump.

    # --- compute table entry pointer: DPTR = TABLE + PHASE*4 ---
    code += mov_dptr(PHASE) + b"\xe0"              # a = PHASE
    # if PHASE >= N -> wrap to 0 (defensive; PHASE is RAM, may boot nonzero)
    code += bytes([0xB4, N_PHASES & 0xFF, 0x00])   # cjne a,#N, chk_lt
    chk = len(code) - 1
    code += b"\xe4"                                # (a==N) clr a
    # chk_lt: if carry set (a<N) keep; else (a>N) also clr. Simplicity: after cjne,
    # C=1 iff a<N. If C==0 and a!=N (a>N) -> clr. Handle: jc keep ; clr a ; keep:
    code += b"\x40\x01"                            # jc +1 (a<N -> keep)
    code += b"\xe4"                                # clr a (a>=N path wrap)
    code[chk] = (len(code) - (chk + 1)) & 0xFF     # cjne rel -> here (the jc)
    # a = PHASE' (in range). Store back the (possibly wrapped) value.
    code += b"\xf5\x23"                            # stash PHASE' in 0x23
    code += mov_dptr(PHASE) + b"\xe5\x23\xf0"      # PHASE = PHASE'
    # DPTR = TABLE + a*4 :  a<<2 ; add TABLE
    code += b"\xe5\x23"                            # a = PHASE'
    code += b"\x25\xe0"                            # add a,acc (a*2)
    code += b"\x25\xe0"                            # add a,acc (a*4)
    code += b"\x24" + bytes([TABLE & 0xFF])        # add a,#TABLE_lo
    code += b"\xf5\x82"                            # DPL = a
    code += b"\xe4\x34" + bytes([(TABLE >> 8) & 0xFF])  # clr a ; addc a,#TABLE_hi
    code += b"\xf5\x83"                            # DPH = a   (DPTR -> table entry)

    # --- read 4 descriptor bytes via MOVC (code memory) into 0x23(dpx),0x21(hi),0x20(lo),0x22(cnt) ---
    code += b"\xe4\x93"                            # clr a ; movc a,@a+dptr  -> dpx
    code += b"\xf5\x23"                            # 0x23 = dpx (plane for shared_dump)
    code += b"\x74\x01\x93"                        # mov a,#1 ; movc a,@a+dptr -> addrHI
    code += b"\xf5\x21"                            # 0x21 = hi
    code += b"\x74\x02\x93"                        # addrLO
    code += b"\xf5\x20"                            # 0x20 = lo
    code += b"\x74\x03\x93"                        # count
    code += b"\xf5\x22"                            # 0x22 = count

    # --- emit header: "\r\n[FD " ctr16 ' ' dpxdigit addrHI addrLO ':' ---
    code += puts_code(STR_FD)
    code += emit_counter()
    code += mov_dptr(UART_TX) + b"\x74\x20\xf0"    # ' '
    # dpx digit = '0'+0x23
    code += b"\xe5\x23\x24\x30"                     # a = 0x23 + '0'
    code += mov_dptr(UART_TX) + b"\xf0"            # putc dpx digit
    # addrHI/addrLO hex
    code += b"\xaf\x21" + lcall(UART_PUTHEX)        # mov r7,0x21 ; puthex
    code += b"\xaf\x20" + lcall(UART_PUTHEX)        # mov r7,0x20 ; puthex
    code += puts_code(STR_COLON)

    # --- dump: shared_dump needs A=plane(0x23), 0x20/0x21=DPTR, 0x22=count ---
    code += b"\xe5\x23"                            # a = dpx (plane)
    code += lcall(SHARED_DUMP)
    code += mov_dptr(UART_TX) + b"\x74\x5d\xf0"     # ']'

    # --- advance PHASE = (PHASE+1) mod N ---
    code += mov_dptr(PHASE) + b"\xe0\x04"          # a = PHASE+1
    code += bytes([0xB4, N_PHASES & 0xFF, 0x02])   # cjne a,#N, +2 (skip clr if !=N)
    code += b"\xe4"                                # clr a (wrap)
    code += mov_dptr(PHASE) + b"\xf0"              # PHASE = a

    # --- SKIP target ---
    skip_off = len(code)
    skip_abs = CAVE_A66 + skip_off
    code[ljmp_skip + 1] = (skip_abs >> 8) & 0xFF
    code[ljmp_skip + 2] = skip_abs & 0xFF

    # epilogue
    code += bytes([0x75, 0x93, 0x00])              # DPX=0
    for d in reversed(PRESERVE):
        code += bytes([0xD0, d])
    code += H_A66_OLD
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
                raise ValueError("wrapped firmware checksum mismatch")
            if int.from_bytes(data[footer + 2:footer + 6], "little") != zlib.crc32(body):
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
        raise ValueError(f"{name} site mismatch at body 0x{off:05x}: found {found.hex()}, "
                         f"expected {old.hex()}")
    body[off:off + len(old)] = lcall(cave)


def apply_patch(body):
    write_cave(body, SHARED_DUMP, build_shared_dump(), "shared dump")
    write_cave(body, STR_FD, b"\r\n[FD \x00", "FD str")
    write_cave(body, STR_COLON, b":\x00", "colon")
    table = build_table()
    if TABLE + len(table) > CAVE_A66 + 0x600:  # sanity
        pass
    write_cave(body, TABLE, table, "table")
    hook = build_a66_hook()
    cave_end = CAVE_A66 + len(hook)
    if cave_end > SHARED_DUMP:
        raise ValueError(f"hook overruns strings (end 0x{cave_end:04x} >= 0x{SHARED_DUMP:04x})")
    write_cave(body, CAVE_A66, hook, "a66 hook")
    patch_site(body, H_A66_OFF, H_A66_OLD, CAVE_A66, "a066")
    return len(hook), len(table)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", type=Path, default=DEFAULT_IN)
    ap.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    data = args.input.read_bytes()
    body, wrapped = unwrap_image(data)
    hlen, tlen = apply_patch(body)
    out = wrap_body(body) if wrapped else bytes(body)
    args.output.write_bytes(out)
    print(f"input: {args.input} ({len(data)} bytes, wrapped={wrapped})")
    print(f"output: {args.output} ({len(out)} bytes)")
    print(f"  a066 hook 0x{CAVE_A66:04x}..0x{CAVE_A66 + hlen:04x} ({hlen} bytes)")
    print(f"  table 0x{TABLE:04x} ({tlen} bytes, {N_PHASES} entries)")
    print(f"  GATE SB[0xA0]==0x02; phase ctr @0x{PHASE:04x}; one block/firing")
    for i, (d, s, c) in enumerate(BLOCKS):
        print(f"    FD{i:02d}: dpx={d} 0x{s:04x} +0x{c:02x}")


if __name__ == "__main__":
    main()
