#!/usr/bin/env python3
"""
diff_trace.py -- Pure-emulation MMIO access-pattern differential.

Runs the firmware in PURE emulation (no real chip, no proxy, no real-time
constraint) against a DETERMINISTIC MOCK MMIO backend that is IDENTICAL for
both firmwares.  Logs EVERY MMIO access (PC, enclosing Ghidra function name,
R/W, address, value, sequence #).

The whole point: the mock does NOT need to be physically accurate.  It must be
IDENTICAL for stock and handmade.  Then any place where, given the SAME mock
responses, handmade reads/writes a DIFFERENT register, runs a DIFFERENT
handler, or takes a DIFFERENT branch than stock -- THAT divergence is the
code-path difference a sampled register-state diff cannot see.

Usage:
    python3 emulate/diff_trace.py FW.bin --max-inst N --out trace.txt
    python3 emulate/diff_trace.py --compare STOCK.bin HANDMADE.bin --max-inst N

STEADY-STATE MODE (--steady, 2026-06-22):
    python3 emulate/diff_trace.py --compare STOCK.bin HANDMADE.bin --steady [--tail-frac 0.12]

    Seeds POST-BOND state IDENTICALLY for both firmwares (SB[0xA0]/[0xA1]=02,
    B481 lane-count=2, CD31 timer, E716=03, 0x06EC connect-consequence gate=1,
    CA81 bit0 clear, P1 config pre-width-event) and models the DPX=1 page-1 plane
    (SB block @0x2800+off, P1 config) as a dedicated reactive mock so BOTH
    superloops run their steady-state connect/lane-advance path.  Then it diffs
    the per-iteration access SET: which (rw,addr) does STOCK re-touch every
    iteration that HANDMADE does NOT (and vice-versa)?  This exposes the
    EXECUTION-PATH divergence a sampled register-state diff cannot see.

    KEY FINDING (2026-06-22): stock re-runs sb_channel_connect_service (c35b:
    C620/C655/C65A connect-config writer, driven off CD31) EVERY superloop
    iteration via bank0_c7a5->bank0_d7cd; handmade ran it only ONCE at boot.
    Ported as sb_connect_service_reservice_d7cd (sb_router.h) -> the C620/C655/
    C65A re-program moved from ONLY-STOCK to COMMON in the steady-state diff.
    HW-validated bond-safe (CL0/CL0 holds, no Abr2) but the width event still
    does not fire (remaining stock-only set = the USB3/NVMe enum SM handmade
    omits in USB4 mode).  See project_inband_routercs_read_wall.md.
"""
import sys, os, json, argparse
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from cpu import CPU8051
from memory import Memory


# ---------------------------------------------------------------------------
# Ghidra symbol map  (PC -> enclosing function name, per address space)
# ---------------------------------------------------------------------------
class SymbolMap:
    def __init__(self, path):
        self.bank0 = []   # list of (entry, end, name) sorted by entry
        self.bank1 = []
        if os.path.exists(path):
            funcs = json.load(open(path))
            for f in funcs:
                t = (f['entry'], f['end'], f['name'])
                if f['space'] == 'CODE_BANK1':
                    self.bank1.append(t)
                else:
                    self.bank0.append(t)
            self.bank0.sort()
            self.bank1.sort()
        self._e0 = [t[0] for t in self.bank0]
        self._e1 = [t[0] for t in self.bank1]

    def name(self, pc, bank):
        import bisect
        if bank & 1 and pc >= 0x8000:
            tbl, entries = self.bank1, self._e1
        else:
            tbl, entries = self.bank0, self._e0
        i = bisect.bisect_right(entries, pc) - 1
        if 0 <= i < len(tbl):
            entry, end, nm = tbl[i]
            if entry <= pc <= end:
                return nm
        return '?'


# ---------------------------------------------------------------------------
# Deterministic MOCK MMIO backend  (IDENTICAL for both firmwares)
# ---------------------------------------------------------------------------
class MockMMIO:
    """
    Reactive, deterministic mock for all XDATA >= mock_lo.

    Design rules (so it is IDENTICAL for both builds):
      * value table seeded once; reads return current value
      * status/handshake/poll registers: a per-address "auto" rule advances the
        value after N reads so poll-loops EXIT (firmware PROGRESSES).  The rule
        is keyed ONLY on (address, read-count) -- never on which firmware.
      * writes update the table (so a written-then-read reg is coherent)

    STEADY-STATE EXTENSION (2026-06-22):
      * page1{}  -- the DPX=1 plane (SB block @ 0x2800+off, P1 config @ 0x12xx/
                    0x14xx, etc.).  In pure emulation w/ no proxy the firmware's
                    DPX page-1 reads otherwise ALIAS page-0 RAM; we give it a
                    dedicated reactive plane seeded with POST-BOND values so the
                    superloop's connect/lane-advance path runs identically on
                    both firmwares.
      * force0{} -- page-0 XDATA cells the HW/host drives that the firmware can't
                    itself set in pure emulation (e.g. 0x06EC connect-consequence
                    gate).  A read of a force0 cell ALWAYS returns the forced
                    value (overriding firmware writes), modelling post-bond.
    """

    def __init__(self, log, steady=False):
        self.v = {}                 # addr -> current byte value (page-0, >=mock_lo)
        self.read_count = {}        # addr -> times read
        self.write_count = {}       # addr -> times written
        self.log = log              # callable(rw, addr, val)
        self.steady = steady
        self.page1 = {}             # DPX=1 plane: off -> value
        self.page1_rc = {}          # DPX=1 read counts
        self.page1_auto = {}        # DPX=1 auto-advance rules: off -> (after_n, newval)
        self.force0 = {}            # page-0 RAM (<mock_lo) forced cells (override firmware)
        self.force_mmio = {}        # page-0 MMIO (>=mock_lo) forced reads (override firmware writes)
        self._seed()
        if steady:
            self._seed_steady()

    def _seed(self):
        v = self.v
        # captured stock register VALUES (from memory docs / tracers)
        v[0xE302] = 0x97     # stable mode-1
        v[0xCA06] = 0x01
        v[0xE710] = 0x04
        v[0xE40F] = 0x01     # PD event type bit0 = Source_Cap
        v[0xE410] = 0x00
        # UART: TX FIFO always drained -> (TFBF & 0x1F) == 0x10 so uart_putc
        # busy-waits exit immediately.  IDENTICAL for both firmwares (real UART
        # drains far faster than the CPU fills it).
        v[0xC006] = 0x10     # UART TX FIFO bytes free

        # addr -> (after_n_reads, new_value).  Identical for both builds.
        self.auto = {
            0x1238: (3, 0x00),
        }

        # CC11 = TIMER0_CSR: a PHY-command GO/DONE handshake register.
        #   write 0x01 (GO)  -> the PHY command "completes": reads show bit1 DONE
        #   write 0x02/0x04  -> clear DONE / clear event
        # Modeled identically for both firmwares so phy_cc10_cmd_wait() loops
        # (`while(!((CC11>>1)&1)){}`) terminate the same way on both.
        self.v[0xCC11] = 0x00

        # E712 = LINK_STATUS: PHY link up.  After the firmware has run the PHY
        # bringup commands, the link comes up.  We signal it after a handful of
        # reads so the `while(!(E712 & 3))` poll exits identically on both.
        self.auto[0xE712] = (4, 0x01)   # bit0 = link up
        # C6B3 = PHY_EXT_B3 ready gate: stock waits `while(!(C6B3 & 0x30))`.
        self.auto[0xC6B3] = (4, 0x30)

        # Self-clearing GO strobes: firmware writes 0x01 (GO) then polls for the
        # bit to CLEAR (HW auto-clears on completion).  Identical for both.
        #   C8A9 = FLASH_CSR, CC89 = XFER_DMA_CMD
        self.self_clear = {0xC8A9, 0xCC89, 0xC8B8}
        for a in self.self_clear:
            self.v[a] = 0x00

        # GO/DONE timer/counter regs with the CC11 layout (write 0x01=GO ->
        # read bit1 DONE; write 0x02/0x04 = clear).  CC17 = TIMER1_CSR.
        self.go_done = {0xCC11, 0xCC17}
        for a in self.go_done:
            self.v[a] = 0x00

        # I2C_CSR (C875): write 0x01 = GO, read bit6 = DONE, write 0xFF = clear.
        self.i2c_csr = 0xC875
        self.v[0xC875] = 0x00

    def _seed_steady(self):
        """POST-BOND steady-state seed -- IDENTICAL for both firmwares.

        Models the connect+bond having occurred so BOTH superloops run their
        steady-state connect/lane-advance path.  Everything here is fixed and
        keyed only on (addr), never on which firmware.
        """
        # ---- page-0 forced cells (HW/host-driven; firmware can't set in pure emu) ----
        # 0x06EC u4_conn_consequence_done = 1: the connect-consequence gate that
        #   both superloops test (stock: (u4_mode_flag&0x83)&&0x06EC; handmade same).
        self.force0[0x06EC] = 0x01
        # 0x09F9 u4_mode_flag: the USB4-enable gate the INT1 demux tests `& 0x83`
        #   in BOTH firmwares (stock int1_isr_orchestrator, handmade int1_isr).
        #   In pure emulation w/o the host PD negotiation stock never latches it,
        #   so its EC06.0 -> c0a5 branch is never reached.  Post-bond reality is
        #   USB4 mode ON -> force 0x87 (tunnel route + VDM-ACK) IDENTICALLY for both
        #   so BOTH run their EC06.0 router-op responder on the injected route=1.
        self.force0[0x09F9] = 0x87
        # 0x06ED FSM state -- leave firmware-managed (advances via cb10).  Not forced.
        # 0x09FA u4_route_mode: tunnel route mode.  stock c7a5 gate tests &0x81.
        #   In handmade it's set by firmware; don't force (page-0 RAM, firmware owns).

        # ---- page-0 MMIO post-bond values (>=mock_lo already handled by self.v) ----
        # B481 lane-count latch: 2 lanes present (bond complete).  c7a5/d7cd read it,
        #   write 0xFF, then xdata[0xb2f]=lanecount-1.
        self.v[0xB481] = 0x02
        # CD31 CPU timer CSR: drives the 0x0B30 connect-state recompute in d7cd/c7a5.
        #   (bit0==0)||(bit1==1) -> b30=1 else b30=2.  Seed 0x02 -> bit1 set -> b30=1.
        self.v[0xCD31] = 0x02
        # CA81 CPU-ctrl: d7cd gate `if (CA81&1)!=1 proceed`.  HW-observed post-bond
        #   value is 0x0E (bit0 CLEAR -> gate passes).  FORCE-READ it (the firmware
        #   writes CA81=01 during boot which would falsely gate the reservice OUT in
        #   emulation).  force_mmio overrides reads of a page-0 MMIO (>=mock_lo) reg.
        self.force_mmio[0xCA81] = 0x0E
        self.v[0xCA81] = 0x0E
        # E716 link-event status: stock superloop gate FUN_CODE_541f tests E716&3.
        #   Post-bond, link events pending -> 0x03 keeps the dee3/480c handlers live.
        self.v[0xE716] = 0x03

        # ---- DPX=1 page-1 plane (SB block @ 0x2800+off, P1 config) ----
        p1 = self.page1
        # SB[0xA0]/[0xA1] lane-bond state = 0x02 (both lanes CL0 = bonded).
        p1[0x28A0] = 0x02
        p1[0x28A1] = 0x02
        # SB[0x66]=01 9E=03 D4=B7 64=03 2C=F1 2D=F5 -- stock a066 post-bond image.
        p1[0x2866] = 0x01
        p1[0x289E] = 0x03
        p1[0x28D4] = 0xB7
        p1[0x2864] = 0x03
        p1[0x282C] = 0xF1
        p1[0x282D] = 0xF5
        # SB[0x26] event status -- keep 0 so the router-op deferred path doesn't fire
        #   spuriously (we want the CONNECT-SERVICE steady path, not event handling).
        p1[0x2826] = 0x00
        # SB[0x24] transport edge -- arbitrary stable value.
        p1[0x2824] = 0x01
        # P1 config-space (page-1 0x12xx/0x14xx): the WIDTH-event registers.
        #   Seed them to the PRE-width-event value (the wall): 1201=00, 1407=00,
        #   1203=00.  We are studying whether the steady-state re-service makes
        #   these WALK -- so they must start at the wall, identical for both.
        p1[0x1201] = 0x00
        p1[0x1203] = 0x00
        p1[0x1407] = 0x00
        # E710 link width = 0x04 (final width reached) on page-1? No -- E710 is a
        #   direct SFR-mapped MMIO (page-0 >=mock_lo), already seeded in self.v.

    def seed_route1_mailbox(self):
        """Seed the inbound CM router-op mailbox with the EXACT route=1 ROUTER_CS_0
        config-read TLP the TB4 host posts (per host-trace), IDENTICAL for both fw.

        Stock c0a5 (CODE_BANK1::c0a5) reads:
          EA90 == 0x5A     -> magic gate (else jmp out, no ack)
          0x0B02 (state)   -> RMBOX_IDLE on first packet
          EA80             -> generation/path opcode; 0xE2 = CONFIG read/write path
          EA81             -> sub-opcode 0x50=READ (ROUTER_CFG_READ) / 0x51=WRITE
          EA82..EA85       -> 32-bit config-space address (route/adapter/offset)
        For a route=1 ROUTER_CS_0 (vid/did) read the host sends:
          gen=0xE2 (config), sub=0x50 (read), addr=route1, offset 0 (ROUTER_CS_0).
        We model the address bytes from the route=1 in-band read (route=1, config=0x2,
        port=0, offset=0).  The exact address encoding does not matter for the
        responder DIFFERENTIAL (it is identical for both fw); what matters is that
        BOTH firmwares see the SAME inbound mailbox and we compare what each DOES.
        """
        # EC06 (REG_NVME_EVENT_STATUS) bit0 = NVMe/router-op event pending.
        self.force_mmio[0xEC06] = 0x01
        self.v[0xEC06] = 0x01
        # The inbound mailbox.  force_mmio so a firmware ACK-write (EC04=1) or a
        # readback does not erase the host-posted bytes mid-dispatch.
        mbox = {
            0xEA90: 0x5A,   # magic gate
            0xEA80: 0xE2,   # CONFIG generation/path opcode (the read/write path)
            0xEA81: 0x50,   # ROUTER_CFG_READ sub-opcode
            0xEA82: 0x01,   # config-space addr byte0  (route=1)
            0xEA83: 0x00,   # offset 0 = ROUTER_CS_0 (vid/did)
            0xEA84: 0x00,
            0xEA85: 0x00,
        }
        for a, val in mbox.items():
            self.v[a] = val
            # EA90/EA80/EA81/EA82-85 are host-owned for the duration; keep them
            # readable.  EA90 the firmware WRITES (0xA5 ack) at the end -> do NOT
            # force EA90 (we want to SEE the ack write).  Force the read-only ones.
            if a != 0xEA90:
                self.force_mmio[a] = val

    def read_page1(self, off):
        """DPX=1 plane read (reactive, deterministic, identical for both fw)."""
        off &= 0xFFFF
        n = self.page1_rc.get(off, 0)
        self.page1_rc[off] = n + 1
        if off in self.page1_auto:
            after_n, newval = self.page1_auto[off]
            if n + 1 >= after_n:
                self.page1[off] = newval
        val = self.page1.get(off, 0x00)
        if self.log:
            self.log('R', 0x10000 | off, val)   # tag page-1 with 0x10000 bit
        return val

    def write_page1(self, off, value):
        off &= 0xFFFF
        value &= 0xFF
        self.page1[off] = value
        if self.log:
            self.log('W', 0x10000 | off, value)

    def read(self, addr):
        addr &= 0xFFFF
        n = self.read_count.get(addr, 0)
        self.read_count[addr] = n + 1
        if addr in self.force_mmio:        # steady-state forced read (HW-owned)
            val = self.force_mmio[addr]
            if self.log:
                self.log('R', addr, val)
            return val
        if addr in self.auto:
            after_n, newval = self.auto[addr]
            if n + 1 >= after_n:
                self.v[addr] = newval
        val = self.v.get(addr, 0x00)
        if self.log:
            self.log('R', addr, val)
        return val

    def write(self, addr, value):
        addr &= 0xFFFF
        value &= 0xFF
        self.write_count[addr] = self.write_count.get(addr, 0) + 1

        # GO/DONE timer model (CC11/CC17 layout), identical for both firmwares
        if addr in self.go_done:
            if value & 0x01:        # GO -> command completes, set DONE (bit1)
                self.v[addr] = 0x02
            elif value & (0x02 | 0x04):   # clear DONE / clear event
                self.v[addr] = 0x00
            else:
                self.v[addr] = value
            if self.log:
                self.log('W', addr, value)
            return

        # Self-clearing GO strobe: write 0x01 (GO) immediately auto-clears
        # (HW completes the flash/DMA op instantly in the mock).
        if addr in self.self_clear:
            self.v[addr] = 0x00     # already "done" -> wait-for-clear exits
            if self.log:
                self.log('W', addr, value)
            return

        # I2C_CSR: write GO -> DONE bit6 set; write 0xFF -> clear.
        if addr == self.i2c_csr:
            if value == 0xFF:
                self.v[addr] = 0x00
            elif value & 0x01:
                self.v[addr] = 0x40   # bit6 DONE
            else:
                self.v[addr] = value
            if self.log:
                self.log('W', addr, value)
            return

        self.v[addr] = value
        if self.log:
            self.log('W', addr, value)


# ---------------------------------------------------------------------------
# Trace runner
# ---------------------------------------------------------------------------
class TraceRunner:
    def __init__(self, fw_path, symmap, max_inst, mock_lo=0x6000,
                 ram_log_lo=None, ram_log_hi=None, steady=False):
        self.mem = Memory()
        self.sym = symmap
        self.max_inst = max_inst
        self.events = []
        self.seq = 0
        self.mock_lo = mock_lo
        self.ram_log_lo = ram_log_lo
        self.ram_log_hi = ram_log_hi
        self.steady = steady
        self.note = ''

        self.mock = MockMMIO(log=self._log_mmio_pending, steady=steady)

        # DPX-aware XDATA front-end.  When DPX (SFR 0x93) != 0 the firmware's
        # XDATA access targets the page-1 plane (SB block / P1 config).  In pure
        # emulation w/ no proxy the base Memory aliases that to page-0 RAM; we
        # intercept here so page-1 goes to the mock's reactive page-1 plane and
        # is captured in the trace.  Identical for both firmwares.
        base_rd, base_wr = self.mem.read_xdata, self.mem.write_xdata

        def rd_x(addr):
            dpx = self.mem.sfr[0x93 - 0x80]
            if dpx:
                return self.mock.read_page1(addr & 0xFFFF)
            if self.steady and (addr & 0xFFFF) in self.mock.force0:
                a = addr & 0xFFFF
                val = self.mock.force0[a]
                self._record('R', a, val)   # tag forced page-0 cell
                return val
            return base_rd(addr)

        def wr_x(addr, value):
            dpx = self.mem.sfr[0x93 - 0x80]
            if dpx:
                self.mock.write_page1(addr & 0xFFFF, value)
                return
            if self.steady and (addr & 0xFFFF) in self.mock.force0:
                # swallow firmware writes to forced cells (HW owns them)
                return
            base_wr(addr, value)

        self.cpu = CPU8051(
            read_code=self.mem.read_code,
            read_xdata=rd_x,
            write_xdata=wr_x,
            read_idata=self.mem.read_idata,
            write_idata=self.mem.write_idata,
            read_sfr=self.mem.read_sfr,
            write_sfr=self.mem.write_sfr,
            read_bit=self.mem.read_bit,
            write_bit=self.mem.write_bit,
            trace=False,
        )

        self._load(fw_path)
        self._install_hooks()

    def _load(self, path):
        with open(path, 'rb') as f:
            data = f.read()
        if len(data) >= 10:
            hl = int.from_bytes(data[:4], 'little')
            if hl == len(data) - 10:
                data = data[4:-6]
        self.mem.load_firmware(data)

    def _curfn(self):
        bank = self.mem.read_sfr(0x96) & 1
        return bank, self.sym.name(self.cpu.pc, bank)

    def _record(self, rw, addr, val):
        bank, fn = self._curfn()
        self.seq += 1
        self.events.append({
            'seq': self.seq, 'pc': self.cpu.pc, 'bank': bank,
            'fn': fn, 'rw': rw, 'addr': addr, 'val': val,
        })

    def _log_mmio_pending(self, rw, addr, val):
        self._record(rw, addr, val)

    def _install_hooks(self):
        m = self.mem
        mock = self.mock

        def rd(addr):
            return mock.read(addr)

        def wr(addr, value):
            mock.write(addr, value)

        for a in range(self.mock_lo, 0x10000):
            m.xdata_read_hooks[a] = rd
            m.xdata_write_hooks[a] = wr

        if self.ram_log_lo is not None:
            lo, hi = self.ram_log_lo, self.ram_log_hi
            backing = m.xdata

            def ram_rd(addr, _bk=backing):
                val = _bk[addr]
                self._record('r', addr, val)
                return val

            def ram_wr(addr, value, _bk=backing):
                _bk[addr] = value & 0xFF
                self._record('w', addr, value & 0xFF)

            for a in range(lo, hi):
                m.xdata_read_hooks[a] = ram_rd
                m.xdata_write_hooks[a] = ram_wr

    def reset(self):
        self.mem.reset()
        self.cpu.reset()
        self.mem.write_sfr(0x81, 0x07)

    def run(self):
        self.reset()
        cpu = self.cpu
        n = 0
        last_pc = -1
        stuck = 0
        while n < self.max_inst:
            pc = cpu.pc
            if pc == last_pc:
                stuck += 1
                if stuck > 500000:
                    self.note = f'STUCK@{pc:04X}'
                    break
            else:
                stuck = 0
                last_pc = pc
            try:
                cpu.step()
            except Exception as e:
                self.note = f'EXC@{pc:04X}:{e}'
                break
            n += 1
            if cpu.halted:
                self.note = f'HALT@{cpu.pc:04X}'
                break
        self.inst_run = n
        return self.events

    def inject_route1(self, isr_entry, isr_bank, settle_inst, isr_max_inst=200000):
        """Drive the firmware to post-bond steady state, then INJECT the route=1
        ROUTER_CS read into the mock mailbox and SYNTHETICALLY CALL the INT1 ISR
        body so the firmware's REAL EC06.0 -> c0a5 responder runs.  Capture ONLY
        the ISR's MMIO access events (self.events is cleared at injection).

        isr_entry/isr_bank: the firmware's INT1 ISR body entry + code bank (DPX).
          stock:    0x4486 bank0 (int1_isr_orchestrator; routes EC06.0 -> bank1 c0a5)
          handmade: 0xBEB9 bank0 (int1_isr -> usb4_int_demux -> cm_routerop_mailbox)
        settle_inst: instructions of boot/superloop to run before injecting.

        Returns (settle_events, isr_events) where isr_events is the responder trace.
        """
        self.reset()
        cpu = self.cpu
        # --- 1. settle: run boot + superloop to steady state (events kept) ---
        n = 0
        last_pc = -1
        stuck = 0
        while n < settle_inst:
            pc = cpu.pc
            if pc == last_pc:
                stuck += 1
                if stuck > 500000:
                    break
            else:
                stuck = 0
                last_pc = pc
            try:
                cpu.step()
            except Exception as e:
                self.note = f'SETTLE_EXC@{pc:04X}:{e}'
                break
            n += 1
        settle_events = list(self.events)

        # --- 2. inject: seed the route=1 mailbox + EC06.0 ---
        self.mock.seed_route1_mailbox()
        # mark the boundary; new events from here are the ISR responder trace
        boundary = len(self.events)

        # --- 3. synthetic CALL of the INT1 ISR body (faithful dispatch entry) ---
        SENTINEL = 0xFFEE
        cpu.in_interrupt = True          # block re-entrant interrupt machinery
        # ISR runs with DPX(0x93)=0 (XDATA page-0) as the real ISR prologue sets
        self.mem.write_sfr(0x93, 0x00)
        # set the CODE bank (SFR 0x96) for the ISR entry
        self.mem.write_sfr(0x96, isr_bank & 0xFF)
        # push the sentinel return address (LCALL order: low first, high on top)
        cpu.push(SENTINEL & 0xFF)
        cpu.push((SENTINEL >> 8) & 0xFF)
        cpu.pc = isr_entry & 0xFFFF

        m = 0
        ret_pc = None
        while m < isr_max_inst:
            if cpu.pc == SENTINEL:
                ret_pc = SENTINEL
                break
            try:
                cpu.step()
            except Exception as e:
                self.note = (self.note + ' | ' if self.note else '') + f'ISR_EXC@{cpu.pc:04X}:{e}'
                break
            m += 1
        self.isr_inst = m
        self.isr_returned = (ret_pc == SENTINEL)
        isr_events = self.events[boundary:]
        return settle_events, isr_events


def addr_str(addr):
    if addr & 0x10000:
        off = addr & 0xFFFF
        if 0x2800 <= off <= 0x28FF:
            return f"SB[{off-0x2800:02X}]"
        if 0x2900 <= off <= 0x29FF:
            return f"SB2[{off-0x2900:02X}]"
        return f"P1[{off:04X}]"
    return f"{addr:04X}"


def fmt_event(e, with_pc=True):
    rw = e['rw']
    a = addr_str(e['addr'])
    if with_pc:
        return f"{rw} {a} pc={e['pc']:04X}.{e['bank']} {e['fn']}"
    return f"{rw} {a} {e['fn']}"


# ---------------------------------------------------------------------------
# Steady-state loop detection + per-iteration access-set extraction
# ---------------------------------------------------------------------------
def detect_steady_window(events, tail_frac=0.25):
    """Return (start_idx, end_idx) of the steady-state tail of the trace.

    Heuristic: the superloop, once reached, revisits a small recurring set of
    (rw,addr,pc) tuples.  We take the last `tail_frac` of the events as the
    steady window (boot is front-loaded; by the tail both firmwares are looping
    or, for handmade, stuck in a flood).  Callers also get the recurring SET.
    """
    n = len(events)
    if n == 0:
        return 0, 0
    start = int(n * (1.0 - tail_frac))
    return start, n


def access_set(events, lo, hi):
    """SET of (rw, addr) touched in events[lo:hi] + per-(rw,addr) count + which
    Ghidra fn touched it (first seen)."""
    s = {}
    for e in events[lo:hi]:
        key = (e['rw'], e['addr'])
        if key not in s:
            s[key] = {'count': 0, 'fn': e['fn'], 'pc': e['pc']}
        s[key]['count'] += 1
    return s


def steady_diff(s_events, h_events, tail_frac=0.25):
    """Diff the steady-state per-iteration access SETS of stock vs handmade."""
    sl, sh = detect_steady_window(s_events, tail_frac)
    hl, hh = detect_steady_window(h_events, tail_frac)
    sset = access_set(s_events, sl, sh)
    hset = access_set(h_events, hl, hh)
    skeys, hkeys = set(sset), set(hset)
    only_stock = sorted(skeys - hkeys, key=lambda k: (k[1], k[0]))
    only_hand = sorted(hkeys - skeys, key=lambda k: (k[1], k[0]))
    common = sorted(skeys & hkeys, key=lambda k: (k[1], k[0]))
    return sset, hset, only_stock, only_hand, common


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('firmware', nargs='?')
    ap.add_argument('--compare', nargs=2, metavar=('STOCK', 'HANDMADE'))
    ap.add_argument('--max-inst', type=int, default=3000000)
    ap.add_argument('--out', default=None)
    ap.add_argument('--symbols', default=str(Path(__file__).parent / 'ghidra_funcs.json'))
    ap.add_argument('--ram-log', default=None,
                    help='LO-HI hex RAM window to also log (e.g. 2800-2c00)')
    ap.add_argument('--mock-lo', default='6000')
    ap.add_argument('--ctx', type=int, default=10)
    ap.add_argument('--steady', action='store_true',
                    help='seed POST-BOND state (both fw) + diff the steady-state access SET')
    ap.add_argument('--tail-frac', type=float, default=0.25)
    ap.add_argument('--inject-route1', action='store_true',
                    help='post-bond steady state, then INJECT a route=1 ROUTER_CS read '
                         'into the mailbox + CALL each fw INT1 ISR; diff the c0a5 responder')
    ap.add_argument('--settle-inst', type=int, default=2000000,
                    help='instructions to settle into steady state before injecting')
    ap.add_argument('--stock-isr', default='4486:0',
                    help='stock INT1 ISR body entry:bank (hex:dec)')
    ap.add_argument('--hand-isr', default='BEB9:0',
                    help='handmade INT1 ISR body entry:bank (hex:dec)')
    args = ap.parse_args()

    symmap = SymbolMap(args.symbols)
    ram_lo = ram_hi = None
    if args.ram_log:
        lo, hi = args.ram_log.split('-')
        ram_lo, ram_hi = int(lo, 16), int(hi, 16)
    mock_lo = int(args.mock_lo, 16)

    def run_one(fw):
        tr = TraceRunner(fw, symmap, args.max_inst, mock_lo=mock_lo,
                         ram_log_lo=ram_lo, ram_log_hi=ram_hi, steady=args.steady)
        tr.run()
        return tr

    if args.compare and args.inject_route1:
        # --- ROUTE=1 ROUTER_CS-READ RESPONDER DIFFERENTIAL -------------------
        def parse_isr(spec):
            a, b = spec.split(':')
            return int(a, 16), int(b)
        s_entry, s_bank = parse_isr(args.stock_isr)
        h_entry, h_bank = parse_isr(args.hand_isr)

        st = TraceRunner(args.compare[0], symmap, args.max_inst, mock_lo=mock_lo,
                         ram_log_lo=ram_lo, ram_log_hi=ram_hi, steady=True)
        s_settle, s_isr = st.inject_route1(s_entry, s_bank, args.settle_inst)
        ht = TraceRunner(args.compare[1], symmap, args.max_inst, mock_lo=mock_lo,
                         ram_log_lo=ram_lo, ram_log_hi=ram_hi, steady=True)
        h_settle, h_isr = ht.inject_route1(h_entry, h_bank, args.settle_inst)

        print(f"===== ROUTE=1 ROUTER_CS-READ RESPONDER DIFFERENTIAL =====")
        print(f"injected mailbox: EA90=5A EA80=E2(CONFIG) EA81=50(READ) EA82=01(route1) "
              f"EA83-85=00(ROUTER_CS_0) EC06.0=1\n")
        print(f"STOCK    ISR @{s_entry:04X}.{s_bank}: ran {st.isr_inst} inst, "
              f"returned={st.isr_returned}, {len(s_isr)} MMIO events {st.note}")
        print(f"HANDMADE ISR @{h_entry:04X}.{h_bank}: ran {ht.isr_inst} inst, "
              f"returned={ht.isr_returned}, {len(h_isr)} MMIO events {ht.note}\n")

        def isr_summary(label, evs):
            print(f"  --- {label} responder MMIO sequence ({len(evs)} events) ---")
            for e in evs:
                print(f"    {fmt_event(e)} = {e['val']:02X}")
            print()
        isr_summary("STOCK", s_isr)
        isr_summary("HANDMADE", h_isr)

        # key responder signals: did each TOUCH the send-path registers?
        KEY = {0xEA90: 'EA90 ack', 0xEA80: 'EA80 opcode', 0xEA81: 'EA81 subop',
               0xEA82: 'EA82 addr', 0xC805: 'C805 reply-trig', 0xC8B0: 'C8B0 DMA-arm',
               0xC8B1: 'C8B1', 0xC8B8: 'C8B8 DMA-go', 0x0B02: '0B02 mbox-state',
               0x0B03: '0B03 opcode', 0x0B04: '0B04 cfg-addr', 0x0B0A: '0B0A limit',
               0xEC04: 'EC04 evt-ack', 0xEC06: 'EC06 evt-status'}

        def keymap(evs):
            d = {}
            for e in evs:
                a = e['addr'] & 0xFFFF if not (e['addr'] & 0x10000) else e['addr']
                if a in KEY:
                    d.setdefault(a, []).append((e['rw'], e['val']))
            return d
        sk, hk = keymap(s_isr), keymap(h_isr)
        print("  --- KEY RESPONDER REGISTER differential (stock vs handmade) ---")
        for a in sorted(KEY):
            sv = sk.get(a)
            hv = hk.get(a)
            mark = '  ' if (sv == hv) else '!!'
            print(f"  {mark} {KEY[a]:16s}  stock={sv}   hand={hv}")

        if args.out:
            with open(args.out + '.stock-isr', 'w') as f:
                for e in s_isr:
                    f.write(fmt_event(e) + f" = {e['val']:02X}\n")
            with open(args.out + '.handmade-isr', 'w') as f:
                for e in h_isr:
                    f.write(fmt_event(e) + f" = {e['val']:02X}\n")
            print(f"\nwrote {args.out}.stock-isr and {args.out}.handmade-isr")
        return

    if args.compare:
        s = run_one(args.compare[0])
        h = run_one(args.compare[1])
        print(f"STOCK    {args.compare[0]}: {len(s.events)} MMIO events, {s.inst_run} inst {s.note}")
        print(f"HANDMADE {args.compare[1]}: {len(h.events)} MMIO events, {h.inst_run} inst {h.note}")

        if args.steady:
            sset, hset, only_s, only_h, common = steady_diff(s.events, h.events, args.tail_frac)
            print(f"\n===== STEADY-STATE ACCESS-SET DIFFERENTIAL (tail {args.tail_frac:.0%}) =====")
            print(f"  stock steady set: {len(sset)} distinct (rw,addr)   "
                  f"handmade: {len(hset)}   common: {len(common)}")
            print(f"\n  --- ONLY STOCK re-touches in steady state (handmade LACKS these) ---")
            for (rw, addr) in only_s:
                info = sset[(rw, addr)]
                print(f"    {rw} {addr_str(addr):10s} x{info['count']:<6d} "
                      f"pc={info['pc']:04X} {info['fn']}")
            print(f"\n  --- ONLY HANDMADE touches in steady state (stock does NOT) ---")
            for (rw, addr) in only_h:
                info = hset[(rw, addr)]
                print(f"    {rw} {addr_str(addr):10s} x{info['count']:<6d} "
                      f"pc={info['pc']:04X} {info['fn']}")
            print(f"\n  --- COMMON (both re-touch in steady state) ---")
            for (rw, addr) in common:
                si, hi_ = sset[(rw, addr)], hset[(rw, addr)]
                print(f"    {rw} {addr_str(addr):10s} stock x{si['count']:<6d} "
                      f"hand x{hi_['count']:<6d}  stock-fn={si['fn']}")
            if args.out:
                with open(args.out + '.stock', 'w') as f:
                    for e in s.events:
                        f.write(fmt_event(e) + f" = {e['val']:02X}\n")
                with open(args.out + '.handmade', 'w') as f:
                    for e in h.events:
                        f.write(fmt_event(e) + f" = {e['val']:02X}\n")
                print(f"\nwrote {args.out}.stock and {args.out}.handmade")
            return

        def norm(ev):
            return [(e['rw'], e['addr'], e['fn']) for e in ev]
        ns, nh = norm(s.events), norm(h.events)
        i = 0
        while i < len(ns) and i < len(nh) and ns[i] == nh[i]:
            i += 1
        print(f"\nFirst access-pattern divergence at index {i}:")
        ctx = args.ctx
        print("  --- STOCK context ---")
        for j in range(max(0, i - ctx), min(len(s.events), i + ctx + 1)):
            mark = '>>' if j == i else '  '
            print(f"  {mark} [{j}] {fmt_event(s.events[j])} = {s.events[j]['val']:02X}")
        print("  --- HANDMADE context ---")
        for j in range(max(0, i - ctx), min(len(h.events), i + ctx + 1)):
            mark = '>>' if j == i else '  '
            print(f"  {mark} [{j}] {fmt_event(h.events[j])} = {h.events[j]['val']:02X}")
        if args.out:
            with open(args.out + '.stock', 'w') as f:
                for e in s.events:
                    f.write(fmt_event(e) + f" = {e['val']:02X}\n")
            with open(args.out + '.handmade', 'w') as f:
                for e in h.events:
                    f.write(fmt_event(e) + f" = {e['val']:02X}\n")
            print(f"\nwrote {args.out}.stock and {args.out}.handmade")
    else:
        tr = run_one(args.firmware)
        print(f"{args.firmware}: {len(tr.events)} MMIO events, {tr.inst_run} inst {tr.note}")
        if args.out:
            with open(args.out, 'w') as f:
                for e in tr.events:
                    f.write(fmt_event(e) + f" = {e['val']:02X}\n")
            print(f"wrote {args.out}")
        else:
            for e in tr.events[:200]:
                print(fmt_event(e) + f" = {e['val']:02X}")


if __name__ == '__main__':
    main()
