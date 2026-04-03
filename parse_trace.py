#!/usr/bin/env python3
"""Parse MMIO traces and print every register write cleanly.

No filtering — every write to any address is shown.
Read-only polling loops are collapsed into a single POLL line.
Decoded overlays for USB setup packets, SCSI commands, CSW, interrupts,
PCIe tunnel, and DMA engine events are shown inline.

Usage: python3 parse_trace.py trace/good_dma_with_delay
       python3 parse_trace.py --no-color trace/good_dma_with_delay
"""
import sys, re, os
from collections import OrderedDict

# ─── ANSI Color Codes ────────────────────────────────────────────────────────

def _supports_color():
    if os.environ.get("NO_COLOR"):
        return False
    if os.environ.get("FORCE_COLOR"):
        return True
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

USE_COLOR = _supports_color()

class C:
    RESET     = "\033[0m"
    BOLD      = "\033[1m"
    DIM       = "\033[2m"
    USB       = "\033[1;36m"   # Bold cyan
    BULK      = "\033[1;33m"   # Bold yellow
    IRQ       = "\033[1;31m"   # Bold red
    NVME      = "\033[1;35m"   # Bold magenta
    DMA       = "\033[1;34m"   # Bold blue
    PCIE      = "\033[0;32m"   # Green
    MSC       = "\033[1;33m"   # Bold yellow
    CSW       = "\033[0;33m"   # Yellow
    INIT      = "\033[0;37m"   # White
    SCSI      = "\033[0;36m"   # Cyan
    UAS       = "\033[1;32m"   # Bold green
    POLL      = "\033[2;33m"   # Dim yellow
    CYCLE     = "\033[2;37m"   # Dim white
    HEADER    = "\033[1;4;37m" # Bold underline white
    WRITE     = "\033[0m"      # Default for generic writes

    @classmethod
    def disable(cls):
        for attr in dir(cls):
            if attr.isupper() and isinstance(getattr(cls, attr), str):
                setattr(cls, attr, "")

if not USE_COLOR:
    C.disable()

CATEGORY_COLORS = {
    "LINK":   C.USB,    "CTRL":   C.USB,    "USB":   C.USB,
    "PERIPH": C.USB,
    "INT":    C.IRQ,    "IRQ":    C.IRQ,    "RETI":  C.IRQ,
    "BULK":   C.BULK,
    "MSC":    C.MSC,    "CSW":    C.CSW,
    "NVME":   C.NVME,
    "DMA":    C.DMA,    "SDMA":   C.DMA,
    "SCSI":   C.SCSI,
    "UAS":    C.UAS,
    "PCIE":   C.PCIE,
    "INIT":   C.INIT,
    "POLL":   C.POLL,
    "W":      C.WRITE,
    "BUF":    C.CSW,
}

# ─── USB standard request names ──────────────────────────────────────────────

USB_REQUESTS = {
    0x00: "GET_STATUS", 0x01: "CLEAR_FEATURE", 0x03: "SET_FEATURE",
    0x05: "SET_ADDRESS", 0x06: "GET_DESCRIPTOR", 0x07: "SET_DESCRIPTOR",
    0x08: "GET_CONFIGURATION", 0x09: "SET_CONFIGURATION",
    0x0A: "GET_INTERFACE", 0x0B: "SET_INTERFACE",
    0x30: "SET_SEL", 0x31: "SET_ISOCH_DELAY",
}

DESC_TYPES = {
    0x01: "DEVICE", 0x02: "CONFIGURATION", 0x03: "STRING",
    0x04: "INTERFACE", 0x05: "ENDPOINT", 0x06: "DEVICE_QUALIFIER",
    0x07: "OTHER_SPEED", 0x0F: "BOS", 0x10: "DEVICE_CAPABILITY",
    0x30: "SS_EP_COMPANION",
}

SCSI_OPCODES = {
    0x00: "TEST_UNIT_READY", 0x03: "REQUEST_SENSE", 0x12: "INQUIRY",
    0x1A: "MODE_SENSE_6", 0x1E: "PREVENT_ALLOW", 0x25: "READ_CAPACITY_10",
    0x28: "READ_10", 0x2A: "WRITE_10", 0x88: "READ_16", 0x8A: "WRITE_16",
    0xA0: "REPORT_LUNS",
    0xE4: "VENDOR_READ_REG", 0xE5: "VENDOR_WRITE_REG",
}

USB_SPEEDS = {0: "Full", 1: "High", 2: "SuperSpeed", 3: "SS+"}

def decode_setup(bmreq, breq, wval_l, wval_h, widx_l, widx_h, wlen_l, wlen_h):
    direction = "IN" if bmreq & 0x80 else "OUT"
    req_type = ["Std", "Class", "Vendor", "Rsvd"][(bmreq >> 5) & 3]
    recip = ["Dev", "Iface", "EP", "Other"][min(bmreq & 0x1F, 3)]
    wValue = wval_l | (wval_h << 8)
    wIndex = widx_l | (widx_h << 8)
    wLength = wlen_l | (wlen_h << 8)
    req_name = USB_REQUESTS.get(breq, f"REQ(0x{breq:02X})")

    extra = ""
    if breq == 0x06:
        desc_name = DESC_TYPES.get(wval_h, f"0x{wval_h:02X}")
        extra = f" desc={desc_name}[{wval_l}]"
    elif breq == 0x05:
        extra = f" addr={wValue}"
    elif breq == 0x09:
        extra = f" config={wValue}"
    elif breq == 0x0B:
        extra = f" alt={wValue} iface={wIndex}"

    return f"{direction} {req_type}/{recip} {req_name} wVal=0x{wValue:04X} wIdx=0x{wIndex:04X} wLen={wLength}{extra}"

SCSI_DIR = {
    0x00: None, 0x03: "IN", 0x12: "IN", 0x1A: "IN", 0x1E: None,
    0x25: "IN", 0x28: "IN", 0x2A: "OUT", 0x88: "IN", 0x8A: "OUT", 0xA0: "IN",
    0xE4: "IN", 0xE5: None,
}

def decode_scsi(opcode, cdb):
    name = SCSI_OPCODES.get(opcode, f"0x{opcode:02X}")
    direction = SCSI_DIR.get(opcode, "?")
    ep = ""
    if direction == "IN":
        ep = " bulk_in(EP1)"
    elif direction == "OUT":
        ep = " bulk_out(EP2)"

    if opcode == 0xE4:
        size = cdb.get(1, 0)
        addr = (cdb.get(2, 0) << 16) | (cdb.get(3, 0) << 8) | cdb.get(4, 0)
        return f"{name} size={size} addr=0x{addr:06X}{ep}"
    elif opcode == 0xE5:
        value = cdb.get(1, 0)
        addr = (cdb.get(2, 0) << 16) | (cdb.get(3, 0) << 8) | cdb.get(4, 0)
        return f"{name} val=0x{value:02X} addr=0x{addr:06X}"
    elif opcode in (0x28, 0x2A):
        lba = (cdb.get(2,0)<<24)|(cdb.get(3,0)<<16)|(cdb.get(4,0)<<8)|cdb.get(5,0)
        cnt = (cdb.get(7,0)<<8)|cdb.get(8,0)
        return f"{name} LBA=0x{lba:08X} cnt={cnt} len={cnt*512}{ep}"
    elif opcode in (0x88, 0x8A):
        lba = 0
        for i in range(2, 10): lba = (lba << 8) | cdb.get(i, 0)
        cnt = (cdb.get(10,0)<<24)|(cdb.get(11,0)<<16)|(cdb.get(12,0)<<8)|cdb.get(13,0)
        return f"{name} LBA=0x{lba:016X} cnt={cnt} len={cnt*512}{ep}"
    if ep:
        return f"{name}{ep}"
    return name

# ─── Polling detection ───────────────────────────────────────────────────────

POLL_THRESHOLD = 4

class ReadRun:
    __slots__ = ('reads', 'start_cycle')

    def __init__(self):
        self.reads = []
        self.start_cycle = 0

    def add(self, cycle, addr, val):
        if not self.reads:
            self.start_cycle = cycle
        self.reads.append((cycle, addr, val))

    def flush(self, events):
        if len(self.reads) < POLL_THRESHOLD:
            self.reads.clear()
            return
        by_addr = OrderedDict()
        for cy, a, v in self.reads:
            if a not in by_addr:
                by_addr[a] = {'count': 0, 'first_cycle': cy, 'first_val': v,
                              'last_val': v, 'vals': set()}
            rec = by_addr[a]
            rec['count'] += 1
            rec['last_val'] = v
            rec['vals'].add(v)
        parts = []
        for a, rec in by_addr.items():
            vs = rec['vals']
            if len(vs) == 1:
                vstr = f"0x{rec['first_val']:02X}"
            else:
                vstr = f"0x{rec['first_val']:02X}..0x{rec['last_val']:02X}"
            parts.append(f"{a:04X}={vstr} x{rec['count']}")
        total = len(self.reads)
        msg = "  ".join(parts)
        events.append((self.start_cycle, "POLL",
                        f"{total} reads: {msg}"))
        self.reads.clear()

    def __len__(self):
        return len(self.reads)

def _addr_color(addr):
    """Pick a color based on address range."""
    if 0x9000 <= addr <= 0x93FF: return C.USB
    if 0xC400 <= addr <= 0xC5FF: return C.NVME
    if 0xCE00 <= addr <= 0xCEFF: return C.SCSI
    if 0xB400 <= addr <= 0xB4FF: return C.PCIE
    if 0xC800 <= addr <= 0xC8FF: return C.DMA
    if 0xD800 <= addr <= 0xDFFF: return C.CSW
    return C.WRITE

def parse_trace(filename):
    line_re = re.compile(
        r'\[\s*(\d+)\]\s+PC=0x([0-9A-Fa-f]+)\s+(Read|Write)\s+0x([0-9A-Fa-f]+)\s+=\s+0x([0-9A-Fa-f]+)(?:\s+(\S.*))?')
    interrupt_re = re.compile(r'\[PROXY\] >>> INTERRUPT mask=0x(\w+) \((\w+)\)')
    reti_re = re.compile(r'\[EMU\] RETI')

    events = []
    read_run = ReadRun()

    # Control transfer state
    setup_bytes = {}
    setup_start_cycle = None

    # SCSI command state
    scsi_base = None
    scsi_opcode = None
    scsi_opcode_cycle = 0
    scsi_cdb = {}

    # MSC hardware-handled SCSI detection
    cbw_received_cycle = 0       # cycle of last CBW_RECEIVED (9101=0x40)
    msc_dma_state = 0            # last CE89 value after CBW
    msc_xfer_cnt = 0             # CE55 value (sector count)
    msc_cbw_tag = 0              # CBW tag for correlation
    msc_detected = False         # set when CDB is read from 0x912A (software-handled)

    # UAS protocol state
    uas_tag_c47a = 0             # last C47A value (current UAS command tag)
    uas_tag_hi = 0               # CEB2 (tag high byte)
    uas_tag_lo = 0               # CEB3 (tag low byte)
    uas_cmd_iu = {}              # per-slot Command IU bytes: {slot: {offset: val}}
    uas_cmd_iu_cycle = {}        # per-slot first read cycle
    uas_mode = False             # set True when UAS mode detected (900D writes)

    # Track link status
    last_usb_speed = None

    # Track D800 region writes
    d800_zero_run = 0
    d800_last_pc = None

    # DMA state
    dma_addr_lo = 0
    dma_addr_hi = 0

    # D800-DFFF buffer accumulator
    buf_writes = []       # list of (addr, val)
    buf_start_cycle = 0

    def flush_buf():
        nonlocal buf_writes
        if not buf_writes:
            return
        data = buf_writes
        buf_writes = []
        count = len(data)
        base_addr = data[0][0]
        # Check if all zeros
        all_zero = all(v == 0 for _, v in data)
        if all_zero:
            events.append((buf_start_cycle, "BUF",
                           f"D800 buf: {count}x 0x00", 0xD800))
        else:
            # Print hex dump, compact
            hex_bytes = " ".join(f"{v:02X}" for _, v in data)
            # Try to decode as ASCII where printable
            ascii_str = "".join(chr(v) if 0x20 <= v < 0x7F else "." for _, v in data)
            events.append((buf_start_cycle, "BUF",
                           f"D800+0x{base_addr - 0xD800:03X} [{count}]: {hex_bytes}  |{ascii_str}|",
                           0xD800))

    uas_scsi_tag = 0  # tag associated with current SCSI command

    def flush_scsi():
        nonlocal scsi_opcode, scsi_cdb, scsi_base, uas_scsi_tag
        if scsi_opcode is not None:
            msg = decode_scsi(scsi_opcode, scsi_cdb)
            if uas_mode and uas_scsi_tag > 0:
                msg = f"[tag={uas_scsi_tag}] {msg}"
            events.append((scsi_opcode_cycle, "BULK", msg))
            scsi_opcode = None
            scsi_cdb = {}
            scsi_base = None
            uas_scsi_tag = 0

    with open(filename) as f:
        last_cycle = 0
        for line in f:
            # Check for interrupt signals (from proxy)
            im = interrupt_re.search(line)
            if im:
                read_run.flush(events)
                events.append((last_cycle, "IRQ", f">>> {im.group(2)} (mask=0x{im.group(1)})"))
                continue

            if reti_re.search(line):
                read_run.flush(events)
                events.append((last_cycle, "RETI", "return from interrupt"))
                continue

            m = line_re.search(line)
            if not m:
                continue

            cycle = int(m.group(1))
            pc = int(m.group(2), 16)
            op = m.group(3)
            addr = int(m.group(4), 16)
            val = int(m.group(5), 16)
            reg_name = m.group(6) or f"0x{addr:04X}"
            last_cycle = cycle

            is_read = (op == "Read")
            is_write = (op == "Write")

            # ── Polling: accumulate reads, flush on write ─────────────────
            if is_read:
                flush_buf()
                read_run.add(cycle, addr, val)
            else:
                read_run.flush(events)

            # ═══════════════════════════════════════════════════════════════
            # WRITES
            # ═══════════════════════════════════════════════════════════════
            if is_write and 0xD800 <= addr <= 0xDFFF:
                # Accumulate buffer writes; flush if non-consecutive address
                if buf_writes and addr != buf_writes[-1][0] + 1:
                    flush_buf()
                if not buf_writes:
                    buf_start_cycle = cycle
                buf_writes.append((addr, val))
            elif is_write:
                flush_buf()
                events.append((cycle, "W", f"{addr:04X}=0x{val:02X}  {reg_name}", addr))

            # ─── Decoded overlays ─────────────────────────────────────────

            # USB link speed change
            if addr == 0x9100 and is_read and val != 0:
                speed = val & 0x03
                if speed != last_usb_speed:
                    last_usb_speed = speed
                    events.append((cycle, "LINK", f"USB {USB_SPEEDS.get(speed, '?')} detected"))

            # USB setup packet decode
            if addr == 0x9104 and is_read:
                setup_bytes = {0x9104: val}
                setup_start_cycle = cycle
            elif 0x9105 <= addr <= 0x910B and is_read and setup_start_cycle:
                setup_bytes[addr] = val
                if len(setup_bytes) == 8:
                    msg = decode_setup(*(setup_bytes[0x9104 + i] for i in range(8)))
                    events.append((setup_start_cycle, "CTRL", msg))
                    setup_start_cycle = None

            # Interrupt source decode (C802)
            if addr == 0xC802 and is_read and val:
                sources = []
                if val & 0x01: sources.append("USB")
                if val & 0x02: sources.append("TIMER")
                if val & 0x04: sources.append("MSC/NVMe")
                if val & 0x08: sources.append("DMA")
                events.append((cycle, "INT", f"C802=0x{val:02X} [{'+'.join(sources)}]"))

            # USB periph status decode
            if addr == 0x9101 and is_read and val:
                sources = []
                if val & 0x01: sources.append("BUS_RESET")
                if val & 0x02: sources.append("CONTROL")
                if val & 0x04: sources.append("BULK_DATA")
                if val & 0x08: sources.append("BULK_REQ")
                if val & 0x10: sources.append("EP_COMPLETE")
                if val & 0x20: sources.append("SUSPEND")
                if val & 0x40: sources.append("CBW_RECEIVED")
                events.append((cycle, "PERIPH", f"9101=0x{val:02X} [{'+'.join(sources)}]"))

            # CSW signature
            if addr == 0xD800 and is_write and val == 0x55:
                events.append((cycle, "CSW", "D800='U' (CSW sig start)"))
            if addr == 0xD803 and is_write and val == 0x53:
                events.append((cycle, "CSW", "D800-D803='USBS' (CSW sig complete)"))

            # CSW status write
            if 0xD800 <= addr <= 0xD80F and is_write:
                if pc == d800_last_pc and val == 0x00:
                    d800_zero_run += 1
                else:
                    d800_zero_run = 0
                d800_last_pc = pc

            if addr == 0xD80C and is_write:
                if d800_zero_run < 4:
                    flush_scsi()
                    status = {0: "PASS", 1: "FAIL", 2: "PHASE_ERR"}.get(val, f"0x{val:02X}")
                    events.append((cycle, "CSW", f"STATUS={status}"))

            # NVMe DMA address tracking
            if addr == 0xC4EE and is_read:
                dma_addr_lo = val
            if addr == 0xC4EF and is_read:
                dma_addr_hi = val

            # DMA trigger: write to 0x90A1+idx
            if is_write and 0x90A1 <= addr <= 0x90AB and val == 0x01:
                stream = addr - 0x90A1
                dma_addr = (dma_addr_hi << 8) | dma_addr_lo
                events.append((cycle, "SDMA", f"stream={stream} dma_addr=0x{dma_addr:04X}"))

            # ── UAS protocol decoding ─────────────────────────────────────
            # C47A: current UAS command tag (hardware-parsed from Command IU)
            if addr == 0xC47A and is_read and val != 0xFF and val != 0x00:
                uas_tag_c47a = val

            # CE88: handshake write — the value IS the stream_id/tag
            if addr == 0xCE88 and is_write:
                uas_tag_c47a = val  # track for annotating next SCSI command
                if val > 0:
                    uas_mode = True
                    events.append((cycle, "UAS",
                                   f"CE88 handshake stream_id={val} (tag={val})"))

            # CEB2/CEB3: next command tag (high/low bytes)
            if addr == 0xCEB2 and is_read:
                uas_tag_hi = val
            if addr == 0xCEB3 and is_read:
                uas_tag_lo = val

            # 900D: UAS response stream configuration
            if addr == 0x900D and is_write and uas_mode:
                events.append((cycle, "UAS",
                               f"response stream={val}"))
            if addr == 0x900F and is_write and uas_mode:
                events.append((cycle, "UAS",
                               f"response stream2={val}"))

            # D0xx Command IU buffer reads (slot = (addr-0xD000)//0x20)
            # Byte layout: [0]=IU type, [2:3]=tag, [16:31]=CDB
            if is_read and 0xD000 <= addr <= 0xD1FF:
                slot = (addr - 0xD000) // 0x20
                offset = (addr - 0xD000) % 0x20
                if slot not in uas_cmd_iu:
                    uas_cmd_iu[slot] = {}
                    uas_cmd_iu_cycle[slot] = cycle
                uas_cmd_iu[slot][offset] = val

            # MSC hardware-handled SCSI command detection:
            # Track CBW_RECEIVED events
            if addr == 0x9101 and is_read and (val & 0x40):
                if cbw_received_cycle == 0:  # first read of this CBW batch
                    cbw_received_cycle = cycle
                    msc_detected = False

            # CE88 handshake write after CBW — start tracking MSC DMA
            if addr == 0xCE88 and is_write and cbw_received_cycle:
                msc_dma_state = 0
                msc_xfer_cnt = 0

            # CE89 DMA state read after CBW handshake
            if addr == 0xCE89 and is_read and cbw_received_cycle:
                msc_dma_state = val

            # CE55 transfer sector count
            if addr == 0xCE55 and is_read and cbw_received_cycle:
                msc_xfer_cnt = val

            # CBW tag read (0x911F) — MSC is building CSW.
            # If no CDB read from 0x912A happened, this was hardware-handled.
            if addr == 0x911F and is_read and cbw_received_cycle and not msc_detected:
                if msc_dma_state in (0x01, 0x05):
                    flush_scsi()
                    if msc_dma_state == 0x05:
                        opcode = 0x2A  # WRITE_10
                    else:
                        opcode = 0x28  # READ_10
                    name = SCSI_OPCODES.get(opcode, f"0x{opcode:02X}")
                    direction = SCSI_DIR.get(opcode, "?")
                    ep = " bulk_in(EP1)" if direction == "IN" else " bulk_out(EP2)" if direction == "OUT" else ""
                    events.append((cbw_received_cycle, "BULK",
                                   f"{name} cnt={msc_xfer_cnt} len={msc_xfer_cnt*512} (hw-handled){ep}"))
                cbw_received_cycle = 0

            # SCSI opcode read from D0xx command buffer (stock firmware, PC=0x3CBF)
            # In UAS mode, CDB starts at offset 0x10 within each 0x20-byte Command IU slot
            if pc == 0x3CBF and is_read and 0xD000 <= addr <= 0xD1FF:
                page_offset = addr & 0x0F
                if page_offset == 0x00:
                    flush_scsi()
                    scsi_base = addr
                    scsi_opcode = val
                    scsi_opcode_cycle = cycle
                    scsi_cdb = {0: val}
                    # Use the current UAS tag from CE88/C47A
                    uas_scsi_tag = uas_tag_c47a

            # CDB param reads from D0xx buffer (stock firmware)
            if is_read and scsi_base and scsi_base >= 0xD000 and 0xD000 <= addr <= 0xD1FF:
                offset = addr - scsi_base
                if 1 <= offset <= 15 and (cycle - scsi_opcode_cycle) < 50000:
                    scsi_cdb[offset] = val

            # SCSI opcode read from CBW register 0x912A (REG_USB_CBWCB_0)
            if is_read and addr == 0x912A:
                flush_scsi()
                scsi_base = 0x912A
                scsi_opcode = val
                scsi_opcode_cycle = cycle
                scsi_cdb = {0: val}
                msc_detected = True  # mark that CDB was read by firmware
                cbw_received_cycle = 0  # reset — firmware handles this one

            # CDB param reads from CBW registers 0x912B-0x9139
            if is_read and scsi_base == 0x912A and 0x912B <= addr <= 0x9139:
                offset = addr - 0x912A
                if 1 <= offset <= 15 and (cycle - scsi_opcode_cycle) < 200000:
                    scsi_cdb[offset] = val

    read_run.flush(events)
    flush_buf()
    flush_scsi()

    # ─── Sort by cycle ────────────────────────────────────────────────────────
    events.sort(key=lambda x: x[0])

    # ─── Normalize events to uniform tuples: (cycle, etype, msg, addr) ──────
    norm = []
    for ev in events:
        if len(ev) == 4 and ev[1] in ("W", "BUF"):
            norm.append(ev)  # (cycle, "W"/"BUF", msg, addr)
        else:
            norm.append((ev[0], ev[1], ev[2], 0))

    # ─── Collapse repeating blocks ───────────────────────────────────────────
    # Build a signature for each event (type + msg, ignoring cycle/addr).
    # For POLL lines, strip the leading count so "11 reads: ..." and
    # "9 reads: ..." with same addresses still match.
    def _sig(ev):
        etype, msg = ev[1], ev[2]
        if etype == "POLL":
            idx = msg.find("reads: ")
            return ("POLL", msg[idx:] if idx >= 0 else msg)
        return (etype, msg)

    sigs = [_sig(e) for e in norm]
    n = len(norm)

    # Greedy: try block sizes from 1..MAX_BLOCK. A block starting at i
    # repeats if the same signature sequence appears consecutively.
    MAX_BLOCK = 20
    collapsed = []  # list of (events_in_block, repeat_count)
    i = 0
    while i < n:
        best_blen = 1
        best_reps = 1
        # Try increasing block lengths, pick the one that collapses most
        for blen in range(1, min(MAX_BLOCK + 1, (n - i) // 2 + 1)):
            pat = sigs[i:i+blen]
            reps = 1
            j = i + blen
            while j + blen <= n and sigs[j:j+blen] == pat:
                reps += 1
                j += blen
            if reps > 5 and blen * reps > best_blen * best_reps:
                best_blen = blen
                best_reps = reps
        collapsed.append((norm[i:i+best_blen], best_reps))
        i += best_blen * best_reps

    # ─── Print ────────────────────────────────────────────────────────────────
    print(f"{C.HEADER}{'─' * 80}{C.RESET}")
    print(f"{C.HEADER}  ASM2464PD Trace — all writes, loops collapsed{C.RESET}")
    print(f"{C.HEADER}{'─' * 80}{C.RESET}")
    total_lines = sum(len(blk) + (1 if reps > 1 else 0) for blk, reps in collapsed)
    print(f"  {C.DIM}File: {filename}  |  Events: {n}  |  Lines: {total_lines}{C.RESET}")
    print(f"{C.HEADER}{'─' * 80}{C.RESET}")
    print()

    for block, reps in collapsed:
        if reps > 1:
            print(f"{C.POLL}  ┌── repeated {reps}x ──{C.RESET}")
        for cycle, etype, msg, addr in block:
            cat = etype.strip()
            if cat == "W":
                color = _addr_color(addr)
            else:
                color = CATEGORY_COLORS.get(cat, C.RESET)
            prefix = "  │ " if reps > 1 else ""
            print(f"{C.CYCLE}[{cycle:>10}]{C.RESET} "
                  f"{C.POLL}{prefix}{C.RESET}"
                  f"{color}[{etype:>5}]{C.RESET} "
                  f"{color}{msg}{C.RESET}")
        if reps > 1:
            print(f"{C.POLL}  └── end x{reps} ──{C.RESET}")

def main():
    args = sys.argv[1:]
    if "--no-color" in args:
        C.disable()
        args.remove("--no-color")
    if not args:
        print(f"Usage: {sys.argv[0]} [--no-color] <trace_file>")
        sys.exit(1)
    parse_trace(args[0])

if __name__ == "__main__":
    main()
