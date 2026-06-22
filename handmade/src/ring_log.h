#ifndef RING_LOG_H
#define RING_LOG_H
/*
 * ring_log.h — on-board HIGH-FREQUENCY XDATA ring-buffer logger for the real-time USB4
 * connect -> lane-bond -> P1[0x1407].0 (link-WIDTH event) timeline.
 *
 * WHY: the change-gated UART diags in the super-loop are too SLOW (UART putc spins) and SAMPLE
 * too SPARSELY to catch sub-millisecond transitions in the mode-negotiation window. UART during
 * the window also alters timing. The proxy/emulator is ~1700x too slow to enter the real-time
 * window at all. This logger snapshots a fixed register set into a fast XDATA ring at full chip
 * speed (NO UART during the window), then dumps the whole ring over UART AFTER the window
 * (bond reached / 1407.0 fired / timeout). It is the on-board, full-speed version of the MMIO
 * trace the proxy could not run.
 *
 * BUFFER LOCATION (RE-confirmed free, 2026-06-22 agent a09198bf): chip-CM XRAM is 0x0000-0x0BFF.
 * The handmade fw allocates ZERO __at cells below 0x0600, and SDCC's XSEG is at 0x0B60. The
 * region 0x0274-0x043F is unused by both stock and handmade (no xrefs / no data items / no
 * computed base descends into it — all computed bases index upward from >=0x0440). We take
 * 0x0280-0x043F = 448 bytes = 28 entries of 16 bytes. This is chip-CM RAM (safe to read at
 * state-5 too, though the connect->bond window is BEFORE the PCIe tunnel powers on anyway).
 *
 * ENTRY = 16 bytes (CHANGE-GATED — see rl_log; only DISTINCT states get a slot, each with a dwell):
 *   [0]  dwell    (# consecutive identical samples collapsed into this entry, capped 0xFF)
 *   [1]  src      (1=ISR usb4_int_demux top, 2=super-loop walker, 3=c105 width-event handler)
 *   [2]  E302     PHY mode      (XDATA, cheap)
 *   [3]  CA06     mode advertise(XDATA, cheap)
 *   [4]  E710     link width    (XDATA, cheap)
 *   [5]  C8FF     lane rate     (XDATA, cheap)
 *   [6]  C294     PHY rate A    (XDATA, cheap)
 *   [7]  C314     PHY rate B    (XDATA, cheap)
 *   [8]  C80A     SFR int src   (XDATA, cheap)  -- EXCLUDED from the change-gate (flickers); OR-accum'd
 *   [9]  0x076B   CL-cfg lane1 RX-status hi (XDATA, cheap) = stock 6A-hi (03 vs handmade transient C1)
 *   [10] SB[0xA0] lane0 state   (P1_RD)  -- 0x02 == CL0 bond
 *   [11] SB[0xA1] lane1 state   (P1_RD)
 *   [12] P1[0x1407] link-event  (P1_RD)  *** THE TARGET: .0 = (PCIe-down adapter) WIDTH event ***
 *   [13] P1[0x1203] width-pending latch (P1_RD)  -- .7 raised by HW
 *   [14] SB[0x9E]   bond strobe  (P1_RD)  -- stock pulses 0x03 at the bond; handmade also does
 *   [15] 0x0759     walker lane-state ladder (XDATA, cheap)
 *
 * The 5 P1_RD reads (each = DPX=1; MOVX; DPX=0) are the only expensive part; a LEAN flag (rl_lean)
 * skips them when only the cheap mode-negotiation set is wanted (smaller timing footprint).
 *
 * LIFECYCLE:
 *   rl_arm()       — call on the connect edge: zero the ring, armed=1, reset idx/seq/dumped.
 *   rl_log(src)    — call on a tight cadence in the window (ISR demux src=1, c105 src=3, super-loop
 *                    src=2). No-op unless armed. CHANGE-GATED: a duplicate of the previous entry only
 *                    bumps its dwell; a distinct state advances the 28-slot ring. This keeps the FULL
 *                    distinct-state transition history (not just the noisy steady-state tail).
 *   rl_stop_dump() — call at 1407.0 / bond+hold / timeout: armed=0, then dump the ring over UART.
 *                    One-shot via rl_dumped so it is poll-safe from the super-loop.
 *
 * RESULT (2026-06-22, this technique): the high-res timeline is byte-IDENTICAL handmade-vs-stock
 * through the whole connect->bond window (E302 33->97->83, CA06 61->01, E710=04, the CL walk to the
 * 0C0C/6A=01030103/779=3C3C terminal, A0/A1->02, SB[0x9E] pulse 03). P1[0x1407].0 / P1[0x1203].7 /
 * P1[0x1201] NEVER fire in handmade. STOCK raises them ~8 events AFTER the bond — specifically after
 * [PcieTunnel-Deassert]->[PcieLinkUp]->[Enable]->[PCIE Gen04 x04]: 1407.0 is the PCIe-DOWN ADAPTER
 * width event raised when the TUNNELED PCIe link trains, NOT a USB4-lane-bond width event. The host
 * gates the PcieTunnel-Enable (P1[0x1508].4) on a successful in-band tunnel config read (route=1),
 * which handmade's dead in-band responder fails (-110). So the wall is the in-band config responder,
 * and the lane-bond / USB4-width half of the timeline has NO firmware-visible divergence at all.
 */

#define RL_BASE      0x0280u
#define RL_ENTRYSZ   16u
#define RL_NENT      28u           /* 28 * 16 = 448 bytes -> 0x0280..0x043F */

/* Header lives in the SDCC-managed XDATA area (auto-allocated, not __at) so it can't alias the
 * ring or any chip cell. */
static volatile __xdata uint8_t rl_armed;     /* 1 while logging the window */
static volatile __xdata uint8_t rl_idx;       /* next entry index (0..RL_NENT-1) */
static volatile __xdata uint8_t rl_wrapped;   /* set once idx wraps (ring full at least once) */
static volatile __xdata uint16_t rl_seq;      /* free-running snapshot counter */
static volatile __xdata uint8_t rl_dumped;    /* one-shot dump guard */
static volatile __xdata uint8_t rl_lean;      /* 1 = skip the 5 P1_RD reads (cheap-only entries) */

static void rl_arm(void) {
  static __xdata uint16_t a;
  for (a = RL_BASE; a < RL_BASE + RL_ENTRYSZ * RL_NENT; a++) XDATA_REG8(a) = 0;
  rl_idx = 0; rl_wrapped = 0; rl_seq = 0; rl_dumped = 0; rl_armed = 1;
}

/* Snapshot the register set, CHANGE-GATED: only advance the ring when the register bytes [2..15]
 * differ from the previous entry. Identical consecutive snapshots just bump byte[0] (dwell count,
 * capped 0xFF) of the current entry. This is the key fix over a flat ring: the connect->bond window
 * spends most of its time in a steady walker spin, which would otherwise flood + overwrite the ring
 * with duplicates and lose the actual transitions. Change-gating keeps the FULL distinct-state
 * transition history in 28 slots, each tagged with how long it dwelled.
 *
 * Entry: [0]=dwell, [1]=src(of the FIRST sample of this distinct state), [2..15]=registers.
 * Cheap XDATA reads first; the 5 page-1 P1_RD reads last (skipped when rl_lean). DPX-clean on entry
 * (int1_isr sets DPX=0 before usb4_int_demux; the super-loop runs DPX=0). P1_RD self-restores DPX=0. */
/* Candidate-snapshot scratch in XDATA (not an IRAM local — keeps DSEG within the tight 8051 stack
 * budget; see crt0.s). Single-writer: ISR rl_log calls can't nest, and the super-loop rl_log call
 * runs with EA disabled, so there is no concurrent writer to this buffer. */
static __xdata uint8_t rl_buf[14];
static void rl_log(uint8_t src) {
  static __xdata uint16_t base;
  static __xdata uint8_t k, same;
  if (!rl_armed) return;
  rl_buf[0]  = XDATA_REG8V(0xE302);
  rl_buf[1]  = XDATA_REG8V(0xCA06);
  rl_buf[2]  = XDATA_REG8V(0xE710);
  rl_buf[3]  = XDATA_REG8V(0xC8FF);
  rl_buf[4]  = XDATA_REG8V(0xC294);
  rl_buf[5]  = XDATA_REG8V(0xC314);
  rl_buf[6]  = XDATA_REG8V(0xC80A);
  rl_buf[7]  = XDATA_REG8V(0x076B);       /* CL-cfg lane1 RX-status hi (= stock 6A-hi: 03 vs hm C1) */
  if (!rl_lean) {
    rl_buf[8]  = P1_RD(0x2800u + 0xA0);   /* SB[0xA0] */
    rl_buf[9]  = P1_RD(0x2800u + 0xA1);   /* SB[0xA1] */
    rl_buf[10] = P1_RD(0x1407);           /* link-event (THE target) */
    rl_buf[11] = P1_RD(0x1203);           /* width-pending latch */
    rl_buf[12] = P1_RD(0x2800u + 0x9E);   /* SB[0x9E] bond strobe (stock 03 at bond; hm 00) */
  } else {
    rl_buf[8] = 0; rl_buf[9] = 0; rl_buf[10] = 0; rl_buf[11] = 0; rl_buf[12] = 0;
  }
  rl_buf[13] = XDATA_REG8V(0x0759);       /* walker lane-state ladder */

  rl_seq++;

  /* Compare to the previously-written entry (if any). If the GATED regs are identical, just bump its
   * dwell. The comparison EXCLUDES only rl_buf[6]=C80A: C80A.5/.4 are W1C'd by the ISR so the bit
   * flickers 20<->00 every pass and would otherwise mark every pass as a "new state", flooding +
   * overwriting the ring with noise and losing the real connect->bond progression. C80A is still
   * RECORDED (OR-accumulated) in each entry so a brief C80A.4 pulse during a dwell is not lost; and a
   * C80A.4 width event also changes P1[0x1407] (a gated reg) so a true event still lands a fresh entry.
   * All other bytes (incl. 076B lane1-RX-status and SB[0x9E] bond strobe) DO gate. */
  if (rl_seq != 1 || rl_wrapped) {     /* at least one entry written before */
    static __xdata uint16_t pbase;
    static __xdata uint8_t prev, d;
    prev = (uint8_t)((rl_idx == 0) ? (RL_NENT - 1) : (rl_idx - 1));
    pbase = RL_BASE + (uint16_t)prev * RL_ENTRYSZ;
    same = 1;
    for (k = 0; k < 14; k++) {
      if (k == 6) continue;                                           /* skip C80A in the gate */
      if (XDATA_REG8(pbase + 2 + k) != rl_buf[k]) { same = 0; break; }
    }
    if (same) {
      d = XDATA_REG8(pbase + 0);
      if (d < 0xFF) XDATA_REG8(pbase + 0) = (uint8_t)(d + 1);
      XDATA_REG8(pbase + 2 + 6) = (uint8_t)(XDATA_REG8(pbase + 2 + 6) | rl_buf[6]);  /* OR-accum C80A */
      return;
    }
  }

  /* New distinct state -> write a fresh entry and advance. */
  base = RL_BASE + (uint16_t)rl_idx * RL_ENTRYSZ;
  XDATA_REG8(base + 0) = 1;     /* dwell starts at 1 */
  XDATA_REG8(base + 1) = src;
  for (k = 0; k < 14; k++) XDATA_REG8(base + 2 + k) = rl_buf[k];

  rl_idx++;
  if (rl_idx >= RL_NENT) { rl_idx = 0; rl_wrapped = 1; }
}

static void rl_puthex(uint8_t v) {
  static __code const char hx[] = "0123456789ABCDEF";
  uart_putc(hx[v >> 4]); uart_putc(hx[v & 0x0F]);
}

/* Dump the whole ring in chronological order, oldest first. One header line + one line/entry.
 * Columns match the entry layout above so a stock-vs-handmade diff aligns by seq + src. */
static void rl_dump(void) {
  static __xdata uint8_t n, i, e, k;
  static __xdata uint16_t base;
  uart_puts("\r\n[RL DUMP n=");
  n = rl_wrapped ? (uint8_t)RL_NENT : rl_idx;
  rl_puthex(n);
  uart_puts(" cols=dwell:src E302 CA06 E710 C8FF C294 C314 C80A 6Bhi A0 A1 1407 1203 SB9E 759]\r\n");
  /* oldest entry: if wrapped, it's at rl_idx; else at 0. */
  for (i = 0; i < n; i++) {
    e = rl_wrapped ? (uint8_t)((rl_idx + i) % RL_NENT) : i;
    base = RL_BASE + (uint16_t)e * RL_ENTRYSZ;
    uart_puts("[RL ");
    rl_puthex(XDATA_REG8(base + 0)); uart_putc(':');   /* dwell */
    rl_puthex(XDATA_REG8(base + 1)); uart_putc(' ');   /* src */
    for (k = 2; k < 16; k++) { rl_puthex(XDATA_REG8(base + k)); uart_putc(' '); }
    uart_puts("]\r\n");
  }
  uart_puts("[RL END]\r\n");
}

/* One-shot stop + dump (poll-safe from the super-loop). */
static void rl_stop_dump(void) {
  if (rl_dumped) return;
  rl_dumped = 1;
  rl_armed = 0;
  rl_dump();
}

#endif /* RING_LOG_H */
