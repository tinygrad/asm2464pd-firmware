#!/usr/bin/env python3
"""
Patch stock fw_tinygrad.bin to UART-dump a TIMELINE of the USB4 lane-rate / link
train state, sampled in the super-loop and logged ONLY ON CHANGE so the train
window is captured without flooding the UART.

WHY: handmade fw caps the USB4 PHY at Gen2 (C8FF==4): lanes stuck at SB[0xA0]/
[0xA1]==0x07 (never CL0==2), E751 (USB4 link-arm, written only by bank0_8a89 when
C8FF>=6) never arms, E302 (link-mode (>>4)&3) reaches mode2 only transiently then
collapses to mode0. C8FF/E710/CA06 are READ-ONLY PHY/host/cable-negotiated status
(8/8 image-wide accesses are MOVX reads) so the firmware cannot WRITE the rate.
Stock's UART prints "[*** USB4 Gen3 x2 ***]" so stock likely DOES reach Gen3. This
patch captures stock's C8FF / E751 / E302 / lane-state TIMELINE through its
SUCCESSFUL train so we can confirm whether stock climbs 4->6 and HOW/WHEN, and
settle whether Gen3/E751 is REQUIRED (-> handmade must reach it) or a red herring.

HOOK SITE (the timeline source):
  main_boot_and_superloop @0x2FB4. The super-loop body top is:
      0x2FBE: c2 af            clr  EA               (interrupts already disabled)
      0x2FC0: 90 0a e2         mov  dptr,#0x0AE2     <-- hooked
      0x2FC3: e0               movx a,@dptr
  We replace the 3-byte `mov dptr,#0x0AE2` at 0x2FC0 with `lcall CAVE`. The cave
  samples the registers, prints a line only when the change-signature differs from
  the previous iteration, then replays `mov dptr,#0x0AE2` and returns. EA is already
  0 at this point (the loop clears it just above), so the sampler runs with
  interrupts disabled -> the DPX-paged SB reads are atomic wrt the PD/USB4 ISRs.

CHANGE DETECTION: a 1-byte signature = C8FF ^ E302 ^ SB[0xA0] ^ SB[0xA1] ^ E751 ^
0x0AA0 is compared against XDATA 0x0BFE (a free byte in the working-RAM page; the
firmware references 0x0Bxx only up to 0x0B44 + 0x0BE0, never 0x0BFE/0x0BFF). On the
first iteration 0x0BFE is 0 so the first non-trivial state always prints. This
yields one line per DISTINCT (rate/mode/lane/arm) state -> a compact timeline.

STOCK'S OWN PRINTS ARE KEPT (this patch touches only the super-loop top, not the
print sites), so [SB Init]/[===SB Con===]/[SB P0x]/[PcieTunnel-*]/[*** USB4 Gen3
x2 ***]/[*** PCIE Gen0x x0x ***] interleave with [T:] lines for time-alignment.

OUTPUT LINE (hex, DPX-restored before each print):
    [T:<C8FF><E751><E302><SBa0><SBa1><0AA0><E710><CA06><E764><C80A>]
       rate  arm   mode  lane  lane  gate  rate  rate  link  conn
  (C8FF: 4=Gen2,5=mid,>=6=Gen3; E302 link-mode=(>>4)&3; SB[0xA0/A1] 0x07->CL0=2;
   0x0AA0=0x0B when Gen3 gate armed; C80A.5 set = host SB-connect.)

Addressing: ghidra.c == this image. body = data[4:-6]; ghidra addr == body offset.
UART helpers (0x538D puts, 0x51C7 puthex) + the cave (0x5E00) all live <0x8000 ->
flat in both banks, callable from the bank0 super-loop site.

Build:
    python3 app/patch_lanetrace.py fw_tinygrad.bin /tmp/fw_lanetrace.bin
"""

import argparse
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN = PROJECT_ROOT / "fw_tinygrad.bin"
DEFAULT_OUT = PROJECT_ROOT / "fw_tinygrad_lanetrace.bin"

# Stock UART helpers (low shared region, <0x8000 -> flat in both banks).
UART_PUTS = 0x538D
UART_PUTHEX = 0x51C7
UART_TX = 0xC001

# Super-loop body top: `mov dptr,#0x0AE2` (3 bytes) right after `clr EA`.
HOOK_SITE_BODY = 0x2FC0
HOOK_SITE_OLD = bytes.fromhex("900ae2")  # mov dptr,#0x0AE2

# Caves in the low shared zero region (flat both banks), <0x8000.
CAVE = 0x5E00
PREFIX_ADDR = 0x5FC0
PREFIX = b"[T:\x00"

# Persistent change-signature byte (free working-RAM byte, plain XDATA DPX=0).
SIG_ADDR = 0x0BFE

# Plain-XDATA (DPX=0) registers logged, in print order.
#   C8FF lane-rate, E751 link-arm, E302 link-mode, then (SB lane bytes inserted),
#   0x0AA0 Gen3 gate, E710 rate, CA06 rate, E764 link-mode-en, C80A connect.
PLAIN_PRE = [0xC8FF, 0xE751, 0xE302]          # before the SB lane bytes
PLAIN_POST = [0x0AA0, 0xE710, 0xCA06, 0xE764, 0xC80A]

# SB page-1 lane-state bytes (DPX=1, XDATA 0x2800+off).
SB_DPX = 0x01
SB_LANE = [0x28A0, 0x28A1]                     # SB[0xA0], SB[0xA1]


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


def read_xdata_to_r(addr, rdir):
    # mov dptr,#addr ; movx a,@dptr ; mov rdir,a   (DPX must already be 0)
    return mov_dptr(addr) + b"\xe0" + bytes([0xF5, rdir])


def read_sb_to_r(addr, rdir):
    # DPX=1 paged read of XDATA 0x2800+off into direct rdir, restore DPX=0.
    return (
        bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
        + mov_dptr(addr) + b"\xe0"              # mov dptr,#addr ; movx a,@dptr
        + bytes([0x75, 0x93, 0x00])            # mov DPX,#0
        + bytes([0xF5, rdir])                  # mov rdir,a
    )


def puthex_sb(addr):
    # DPX=1 paged read, restore DPX=0, then print (the print's movx is DPX-sensitive).
    return (
        bytes([0x75, 0x93, SB_DPX])            # mov DPX,#1
        + mov_dptr(addr) + b"\xe0\xff"         # movx a,@dptr ; mov r7,a
        + bytes([0x75, 0x93, 0x00])            # mov DPX,#0
        + lcall(UART_PUTHEX)
    )


def puts_code(addr):
    return (
        bytes([0x7B, 0xFF])
        + bytes([0x7A, (addr >> 8) & 0xFF])
        + bytes([0x79, addr & 0xFF])
        + lcall(UART_PUTS)
    )


# Scratch IRAM bytes used only inside the hook (saved/restored). 0x22 holds the
# freshly-computed signature; 0x23 a temp.
SIG_SCRATCH = 0x22
TMP_SCRATCH = 0x23

# Preserve ACC,B,DPL,DPH,PSW,R0-R7,DPX + the scratch bytes we touch.
PRESERVE = (0xE0, 0xF0, 0x82, 0x83, 0xD0,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x93, SIG_SCRATCH, TMP_SCRATCH)


def build_signature():
    """Compute sig = C8FF ^ E302 ^ SB[0xA0] ^ SB[0xA1] ^ E751 ^ 0x0AA0 into 0x22."""
    code = bytearray()
    # start sig = C8FF
    code += mov_dptr(0xC8FF) + b"\xe0" + bytes([0xF5, SIG_SCRATCH])
    for addr in (0xE302, 0xE751, 0x0AA0):
        code += mov_dptr(addr) + b"\xe0"                   # movx a,@dptr
        code += bytes([0x65, SIG_SCRATCH])                 # xrl a,0x22
        code += bytes([0xF5, SIG_SCRATCH])                 # mov 0x22,a
    # SB lane bytes (DPX=1)
    for addr in SB_LANE:
        code += bytes([0x75, 0x93, SB_DPX])                # mov DPX,#1
        code += mov_dptr(addr) + b"\xe0"                   # movx a,@dptr (paged)
        code += bytes([0x75, 0x93, 0x00])                  # mov DPX,#0
        code += bytes([0x65, SIG_SCRATCH])                 # xrl a,0x22
        code += bytes([0xF5, SIG_SCRATCH])                 # mov 0x22,a
    return bytes(code)


def build_print():
    """Emit the [T:...] line: PLAIN_PRE, SB lane bytes, PLAIN_POST."""
    code = bytearray()
    code += puts_code(PREFIX_ADDR)
    for addr in PLAIN_PRE:
        code += puthex_xdata(addr)
    for addr in SB_LANE:
        code += puthex_sb(addr)
    for addr in PLAIN_POST:
        code += puthex_xdata(addr)
    code += emit_char(']')
    code += emit_char('\r')
    code += emit_char('\n')
    return bytes(code)


def build_hook():
    code = bytearray()
    for direct in PRESERVE:
        code += bytes([0xC0, direct])                      # push direct

    # DPX is preserved above; ensure DPX=0 for the plain reads.
    code += bytes([0x75, 0x93, 0x00])                      # mov DPX,#0

    code += build_signature()                              # -> sig in 0x22

    # Compare sig (0x22) to stored 0x0BFE. If equal -> skip the print.
    code += mov_dptr(SIG_ADDR) + b"\xe0"                   # movx a,@dptr (old sig)
    code += bytes([0xB5, SIG_SCRATCH, 0x00])               # cjne a,0x22,neq ; rel filled below
    skip_branch = len(code) - 1                            # location of the cjne rel byte

    # EQUAL path: fall through to the pop/replay/ret (no print).
    # We jump over the print block. Compute after building print.
    print_code = build_print()

    # store new sig: mov dptr,#SIG_ADDR ; mov a,0x22 ; movx @dptr,a
    store_sig = mov_dptr(SIG_ADDR) + bytes([0xE5, SIG_SCRATCH]) + b"\xf0"

    # Layout:
    #   cjne a,0x22, NEQ           (3 bytes; rel = forward to NEQ)
    #   sjmp DONE                  (2 bytes)  [equal -> skip print]
    # NEQ:
    #   store_sig
    #   print_code
    # DONE:
    #   pops / replay / ret
    eq_sjmp = b"\x80\x00"                                   # sjmp DONE (rel filled later)
    neq = bytearray()
    neq += store_sig
    neq += print_code

    # cjne rel: branch taken (not-equal) jumps to NEQ which is right after the
    # eq_sjmp. rel = len(eq_sjmp) measured from end of cjne. cjne already in code.
    code[skip_branch] = len(eq_sjmp) & 0xFF                # rel over the sjmp
    code += eq_sjmp
    eq_sjmp_pos = len(code) - 2
    code += neq

    # DONE target = current end (the pops). Patch eq_sjmp rel.
    done = len(code)
    code[eq_sjmp_pos + 1] = (done - (eq_sjmp_pos + 2)) & 0xFF

    for direct in reversed(PRESERVE):
        code += bytes([0xD0, direct])                      # pop direct

    code += HOOK_SITE_OLD                                  # replay mov dptr,#0x0AE2
    code += b"\x22"                                        # ret
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
        raise ValueError(f"{name} cave at 0x{addr:04x} is not empty (len {len(data)})")
    body[addr:end] = data
    return end


def apply_patch(body):
    found = bytes(body[HOOK_SITE_BODY:HOOK_SITE_BODY + len(HOOK_SITE_OLD)])
    if found != HOOK_SITE_OLD:
        raise ValueError(
            f"hook site mismatch at body 0x{HOOK_SITE_BODY:05x}: found {found.hex()}, "
            f"expected {HOOK_SITE_OLD.hex()}"
        )
    hook = build_hook()
    repl = bytearray(lcall(CAVE))
    while len(repl) < len(HOOK_SITE_OLD):
        repl += b"\x00"
    if len(repl) != len(HOOK_SITE_OLD):
        raise ValueError("replacement length mismatch")
    body[HOOK_SITE_BODY:HOOK_SITE_BODY + len(HOOK_SITE_OLD)] = repl
    write_cave(body, CAVE, hook, "lanetrace hook")
    write_cave(body, PREFIX_ADDR, PREFIX, "lanetrace prefix")
    return [("LANETRACE", HOOK_SITE_BODY, CAVE, len(hook))]


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
    for name, site, cave, hlen in info:
        print(f"  {name}: site body 0x{site:05x} -> lcall 0x{cave:04x} "
              f"(hook {hlen} bytes -> 0x{cave + hlen:04x})")
    print(f"  sig byte: XDATA 0x{SIG_ADDR:04x} (log-on-change)")
    print(f"  line: [T:<C8FF><E751><E302><SBa0><SBa1><0AA0><E710><CA06><E764><C80A>]")


if __name__ == "__main__":
    main()
