#!/usr/bin/env python3
"""Replay register reads/writes from an emulator trace via USB control transfers (0xE4/0xE5).

Parses a trace file and replays all operations using the same vendor
control-transfer interface as dma_explore.py.  Writes are sent to hardware;
reads are verified against the expected trace value and shown in green (match)
or red (mismatch).  Supports cycle-range filtering and a register blacklist.

Usage:
    python3 replay_trace.py <trace_file> [--after CYCLE] [--before CYCLE]
                                          [--blacklist REG1,REG2,...]
                                          [--dry-run] [--delay MS]

Examples:
    # Replay after cycle 813676, skipping 0x9091/0x9092
    python3 replay_trace.py trace/usb2_dma --after 813676 \\
        --blacklist 0x9092,0x9091

    # Dry-run to see what would be replayed
    python3 replay_trace.py trace/usb2_dma --after 813676 --before 900000 \\
        --blacklist 0x9092,0x9091 --dry-run
"""

import argparse, ctypes, re, struct, sys, time
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

GREEN = "\033[32m"
RED   = "\033[31m"
RESET = "\033[0m"


class Dev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04)

    def read8(self, addr):
        buf = (ctypes.c_ubyte * 1)()
        ret = libusb.libusb_control_transfer(self.usb.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
        assert ret >= 0, f"read(0x{addr:04X}) failed: {ret}"
        return buf[0]

    def write(self, addr, val):
        ret = libusb.libusb_control_transfer(self.usb.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

    def scsi_write(self, data):
        cdb = struct.pack('>BBQIBB', 0x8A, 0, 0, len(data) // 512, 0, 0)
        self.usb.send_batch([cdb], idata=[0], odata=[data])


def parse_ops(filename, after=None, before=None):
    """Yield (cycle, op, addr, val) for every Read/Write in the trace within the cycle range."""
    line_re = re.compile(
        r'\[\s*(\d+)\]\s+PC=0x[0-9A-Fa-f]+\s+(Read|Write)\s+0x([0-9A-Fa-f]+)\s+=\s+0x([0-9A-Fa-f]+)')
    with open(filename) as f:
        for line in f:
            m = line_re.search(line)
            if not m:
                continue
            cycle = int(m.group(1))
            if after is not None and cycle <= after:
                continue
            if before is not None and cycle >= before:
                continue
            op = m.group(2)
            addr = int(m.group(3), 16)
            val = int(m.group(4), 16)
            yield cycle, op, addr, val


def main():
    parser = argparse.ArgumentParser(description="Replay register reads/writes from an emulator trace")
    parser.add_argument("trace", help="Path to trace file")
    parser.add_argument("--after", type=str, default=None,
                        help="Only replay ops after this cycle number (exclusive). "
                             "Use START-END to specify a range (e.g. 813676-889507)")
    parser.add_argument("--before", type=int, default=None,
                        help="Only replay ops before this cycle number (exclusive)")
    parser.add_argument("--blacklist", type=str, default="",
                        help="Comma-separated list of register addresses to skip (e.g. 0x9091,0x9092)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print ops without sending them to hardware")
    parser.add_argument("--delay", type=float, default=0,
                        help="Delay in milliseconds between ops")
    args = parser.parse_args()

    # Parse --after (supports single value or START-END range)
    after = None
    if args.after is not None:
        if '-' in args.after:
            parts = args.after.split('-', 1)
            after = int(parts[0])
            if args.before is None:
                args.before = int(parts[1])
        else:
            after = int(args.after)
    args.after = after

    # Parse blacklist
    blacklist = set()
    if args.blacklist:
        for tok in args.blacklist.split(","):
            tok = tok.strip()
            if tok:
                blacklist.add(int(tok, 0))

    # Collect ops
    all_ops = list(parse_ops(args.trace, after=args.after, before=args.before))

    # Filter blacklisted and buffer region 0xD800-0xDFFF
    ops = [(cy, op, addr, val) for cy, op, addr, val in all_ops
           if addr not in blacklist and not (0xD800 <= addr < 0xE000)]

    total_reads = sum(1 for _, op, _, _ in ops if op == "Read")
    total_writes = sum(1 for _, op, _, _ in ops if op == "Write")
    skipped = len(all_ops) - len(ops)
    print(f"Trace: {args.trace}")
    print(f"Range: after={args.after}  before={args.before}")
    print(f"Blacklist: {', '.join(f'0x{a:04X}' for a in sorted(blacklist))}" if blacklist else "Blacklist: (none)")
    print(f"Total ops in range: {len(all_ops)}")
    print(f"Blacklisted (skipped): {skipped}")
    print(f"Ops to replay: {len(ops)} ({total_reads} reads, {total_writes} writes)")
    print()

    if not ops:
        print("Nothing to replay.")
        return

    if not args.dry_run:
        dev = Dev()

    delay_s = args.delay / 1000.0
    read_match = 0
    read_mismatch = 0

    for i, (cycle, op, addr, val) in enumerate(ops):
        if args.dry_run:
            tag = "Read " if op == "Read" else "Write"
            print(f"  [{cycle:>10}] {tag} 0x{addr:04X} = 0x{val:02X}")
        else:
            if op == "Write":
                print(f"  [{i+1:>5}/{len(ops)}] [{cycle:>10}] Write 0x{addr:04X} = 0x{val:02X}", end="")
                try:
                    dev.write(addr, val)
                    print("  OK")
                except AssertionError as e:
                    print(f"  FAIL: {e}")
                    sys.exit(1)
            else:
                actual = dev.read8(addr)
                if actual == val:
                    read_match += 1
                    color = GREEN
                    status = "MATCH"
                else:
                    read_mismatch += 1
                    color = RED
                    status = f"MISMATCH got 0x{actual:02X}"
                print(f"  {color}[{i+1:>5}/{len(ops)}] [{cycle:>10}] Read  0x{addr:04X} = 0x{val:02X}  {status}{RESET}")
            if delay_s > 0:
                time.sleep(delay_s)

    print()
    if args.dry_run:
        print("Dry run complete.")
    else:
        print(f"Replay complete. {total_writes} writes, {total_reads} reads "
              f"({GREEN}{read_match} match{RESET}, {RED}{read_mismatch} mismatch{RESET}).")

        # Send a SCSI WRITE(16) via UAS
        test_data = bytes(range(256)) + bytes(range(256))
        print(f"\nSending SCSI WRITE(16) via UAS ({len(test_data)} bytes)...", end="")
        try:
            dev.scsi_write(test_data)
            print("  done.")
        except Exception as e:
            print(f"  {e}")

        # Check if the data showed up at 0xF000
        print("Checking 0xF000:")
        for off in range(0, 64, 16):
            vals = [dev.read8(0xF000 + off + i) for i in range(16)]
            hex_str = ' '.join(f'{v:02X}' for v in vals)
            ascii_str = ''.join(chr(v) if 32 <= v < 127 else '.' for v in vals)
            print(f"  F{off:03X}: {hex_str}  |{ascii_str}|")


if __name__ == "__main__":
    main()
