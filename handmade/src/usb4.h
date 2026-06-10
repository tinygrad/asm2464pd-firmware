#ifndef USB4_H
#define USB4_H
/*
 * USB4 mode-establishment glue — faithful transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) post-Enter_USB connect path + the INT1 USB4 event demux.
 * Ghidra body offset == fw_tinygrad offset. Included AFTER vdm.h (so PR(a), uart_*, the PD
 * helpers and `DPX` SFR are in scope).
 *
 * CONTEXT (HW-proven, see handmade/USB4_RE.md + memory project_usb4_vdm_responder):
 *   PD contract + Discover_Identity + Enter_USB all work; the device ACCEPTs Enter_USB. The
 *   stock UART then prints:  [Enter_USB 4][Connect_U4][flp=01][SB Init][SB P03] ... and the
 *   host begins USB4 lane training (E302 link-mode (E302>>4)&3 -> >=2). The captured AMD/TB4
 *   host goes Discover_Identity -> Enter_USB DIRECTLY (NO EnterMode VDM), so the SB bring-up is
 *   driven entirely by the Enter_USB Accept setting 0x07BA=1 -> usb4_connect_decide @0xA0A7 tail
 *   -> [Connect_U4] -> usb4_connect_u4 @0xA3F5.
 *
 * usb4_connect_u4 @0xA3F5 has two parts:
 *   (A) a HIGH-CONFIDENCE bank0 head: direct XDATA RMW on E716/CA81/CA06 (gated 0x0AF1.0) and
 *       the 0x09FA/0x09FB route-mode latch from 0x09F9&3 / 0x09F4. Transcribed verbatim below.
 *   (B) the bank1 tail: b230 (sb_lane_flip_init) + bb37 (sb_block_init) reached via trampolines
 *       0x0606/0x060b, preceded by the a48c..a516 intermediate tunnel/PCIe path-state RMWs. This
 *       was previously left un-implemented at the RE boundary; it is now resolved via the
 *       high-confidence `sideband` decode + the byte-exact ROM tables (CODE 0x213d/0x21d4) and
 *       implemented verbatim in sb.h. Running it is the M1 keystone: it is the SB block the host
 *       CM polls, so the upstream PHY sets E302 in response. See handmade/USB4_TUNNEL_PLAN.md §2.
 * ===========================================================================
 */

/* usb4_connect_u4 @0xA3F5 — bank0 head (part A) + the sb_assert() bank1 tail (part B, sb.h). */
static void usb4_connect_u4(void) {
  /* RE-AUDIT #3/D(c): 0x0AF1.0 is a DYNAMIC connect-time gate (NOT the static boot 0). Both this
   * inner a3f5 body AND c9a8/bank0_8a89 require it set before the connect runs. Stock seeds it 0 at
   * boot (92C5) and sets it on the connect path; set it here on connect entry. */
  PR(0x0AF1) = PR(0x0AF1) | 0x01;
  /* a3f5: gated on 0x0AF1.0 — link/route control RMW (direct bank0 XDATA, high confidence). */
  if (PR(0x0AF1) & 0x01) {
    PR(0xE716) = (PR(0xE716) & 0xFC) | 0x03;     /* a3fc */
    PR(0xCA81) = PR(0xCA81) & 0xFE;              /* a405 */
    PR(0xCA06) = (PR(0xCA06) & 0x1F) | 0x60;     /* a40c */
  }
  /* a424: 0x07BA gate (set by Enter_USB Accept). Latch the route mode from 0x09F9 low 2 bits.
   * NOTE: the stock a415-a421 pre-gate helpers (dd42(0)/e7c1(1)/e0d9(0)) and the fuller a48c tail
   * were tried and REGRESSED E302 training on HW (mode 3 -> mode 0). The simpler baseline tail
   * trains E302 to mode 3, so the extra PHY pre-stage is kept OUT. */
  if (PR(0x07BA) != 0) {
    /* a42a: latch route mode from 0x09F9&3. RE-AUDIT #3/D(b): do NOT clobber 0x09FA.2 (the c9a8
     * connect-gate bit set on the route entry); preserve it across the route latch. (Stock's a42a
     * writes the low 2 bits via a masked store; bit2 is owned by the a522/link-event path.) */
    PR(0x09FA) = (PR(0x09FA) & 0x04) | (PR(0x09F9) & 0x03);
    if (PR(0x09F4) == 0x03) {                    /* a434: DP-alt sub-case */
      if (PR(0x07BE) == 0) { PR(0x09FA) = (PR(0x09FA) & 0x04) | 2; PR(0x09FB) = 1; }
      else                 { PR(0x09FA) = (PR(0x09FA) & 0x04) | 1; PR(0x09FB) = 2; }
    }
    if (PR(0x09FA) & 0x02) {                     /* a45a: 0x09FA.1 lane-1 route mode */
      PR(0xE716) = PR(0xE716) & 0xFC;
      PR(0xE716) = (PR(0xE716) & 0xFC) | 0x03;
      SB_WR(0xD8, 0x02);                          /* SB[0xD8]=2 (page1 0x128D8) */
    }
    /* a48c..a51e: intermediate tunnel/PCIe path-state, then b230 + sb_block_init (sb.h). */
    sb_assert();
  }
}

/* ====================================================================================
 * INT1 USB4 event demux (orchestrator int1_isr_orchestrator @0x4486, USB4 branch).
 * Gated by (0x09F9 & 0x83). Each source's handler is a bank1/heavy-helper body that is NOT
 * statically recoverable; here we ONLY (a) log which sources fire and (b) apply the
 * DOCUMENTED-safe W1C acks (EC04=1, E763 W1C) so a fired source is observable without being
 * left perpetually asserted. We do NOT fabricate the bank1 handler register sequences.
 * usb4_int_log lets a single super-loop print show whether the host ever drives USB4 events
 * after the corrected Enter_USB/Connect_U4 state — the key diagnostic for the stopping point.
 * ==================================================================================== */

/* Sticky bitmap of USB4 INT sources seen (printed from the super-loop, not the ISR, to keep the
 * ISR short): bit0=C80A.5 SB, bit1=C80A.4 evt, bit2=EC06.0 routerop, bit3=C80A.0-3 tunnel. */
static volatile uint8_t usb4_int_seen = 0;

/* M2: the SB-router event handler (a066) is implemented in sb_router.h and W1C-acks the C80A.5
 * source, so the int1 demux no longer needs the M1 one-shot latch (it can service every ISR
 * without storming). sb_router_event_handler is forward-declared here (defined in sb_router.h,
 * included before usb4.h). */
static void sb_router_event_handler(void);

/* Sticky accumulator of every C80A value seen in the ISR (catches a transient C80A.5). */
static volatile uint8_t c80a_acc = 0;

/* Called from int1_isr after PD-RX, gated by (0x09F9 & 0x83). Reactive W1C-ack + forward. */
static void usb4_int_demux(void) {
  uint8_t c80a = PR(0xC80A);
  c80a_acc |= c80a;
  if (c80a & 0x20) {                         /* C80A.5 -> SB router/connect (bank1 0xA066) */
    usb4_int_seen |= 0x01;
    sb_router_event_handler();               /* M2: a066 — per-channel connect + lane-bond + W1C */
  }
  if (c80a & 0x10) usb4_int_seen |= 0x02;   /* C80A.4 -> USB4 adapter/link event (bank0 0xC105) */
  if (PR(0xEC06) & 0x01) {                   /* EC06.0 -> router-op mailbox (bank1 0xC0A5) */
    usb4_int_seen |= 0x04;
    PR(0xEC04) = 1;                          /* documented ack (orchestrator @0x4486) */
  }
  if (c80a & 0x0F) {                         /* C80A.0-3 -> PCIe-tunnel link events (bank1 0xE911) */
    usb4_int_seen |= 0x08;
    { uint8_t e763 = PR(0xE763);             /* documented W1C: bit2->0x04, bit3->0x08 */
      if (e763 & 0x04) PR(0xE763) = 0x04;
      if (e763 & 0x08) PR(0xE763) = 0x08; }
  }
}

#endif /* USB4_H */
