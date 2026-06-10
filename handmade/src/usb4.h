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
 * ============================ HONEST RE BOUNDARY ============================
 * usb4_connect_u4 @0xA3F5 has two parts:
 *   (A) a HIGH-CONFIDENCE bank0 head: direct XDATA RMW on E716/CA81/CA06 (gated 0x0AF1.0) and
 *       the 0x09FA/0x09FB route-mode latch from 0x09F9&3 / 0x09F4. Transcribed verbatim below.
 *   (B) a LOW-CONFIDENCE bank1 tail: b230 (lane-flip @bank1 0xB230) + sb_block_init (@bank1
 *       0xBB37, ~40 RMW writes to the SB transport block at page-1 DPX=1 0x012800) reached via
 *       the bank-switch trampoline 0x0606/0x060b, PLUS ~10 intermediate bank1 helper calls
 *       (0x0575/0x0624/0xbc8f/0xbcd7/0xbcc4/0xbc91 ...) that program B4xx/CA06 path state before
 *       the SB init. Those bank1 bodies are NOT statically recoverable byte-accurately: they go
 *       through context-dependent generic-pointer helpers (func_96c7/func_e0d9/func_9685/...)
 *       whose SB-block addressing could not be pinned down. Per the project rule "no speculative
 *       HW fixes — if a value is un-RE-able, STOP and report rather than guess", part (B) is
 *       intentionally NOT reproduced here (writing guessed SB-block pokes could mis-train the
 *       sideband). usb4_connect_u4 runs the faithful part (A) + logs, so we can observe on HW
 *       whether the host raises any USB4 interrupt (C80A.5/EC06.0) once the device state matches
 *       stock through [Connect_U4]. See the deliverable report for the exact stopping point.
 * ===========================================================================
 */

/* ---- SB (sideband) transport block: page-1 (DPX=1), base 0x012800. SB[off] via DPX=1. ---- */
/* (Reserved for a future faithful sb_block_init once the bank1 helpers are resolved on HW.) */

/* usb4_connect_u4 @0xA3F5 — recoverable bank0 head (part A). The bank1 SB/tunnel tail (part B)
 * is deliberately omitted (un-RE-able; see HONEST RE BOUNDARY above). */
static void usb4_connect_u4(void) {
  /* a3f5: gated on 0x0AF1.0 — link/route control RMW (direct bank0 XDATA, high confidence). */
  if (PR(0x0AF1) & 0x01) {
    PR(0xE716) = (PR(0xE716) & 0xFC) | 0x03;     /* a3fc */
    PR(0xCA81) = PR(0xCA81) & 0xFE;              /* a405 */
    PR(0xCA06) = (PR(0xCA06) & 0x1F) | 0x60;     /* a40c */
  }
  /* a424: 0x07BA gate (set by Enter_USB Accept). Latch the route mode from 0x09F9 low 2 bits. */
  if (PR(0x07BA) != 0) {
    PR(0x09FA) = PR(0x09F9) & 0x03;              /* a42a */
    if (PR(0x09F4) == 0x03) {                    /* a434: DP-alt sub-case */
      if (PR(0x07BE) == 0) { PR(0x09FA) = 2; PR(0x09FB) = 1; }
      else                 { PR(0x09FA) = 1; PR(0x09FB) = 2; }
    }
    /* a45a..: the remainder (E716 RMW on 0x09FA.1, the bank1 SB/tunnel bring-up at 0xA48C-0xA51E
     * incl. b230 + sb_block_init) is the un-RE-able bank1 tail — NOT reproduced (see boundary). */
  }
  /* No SB-block writes emitted: stopping at the RE boundary rather than guessing. */
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

/* Called from int1_isr after PD-RX, gated by (0x09F9 & 0x83). Observe + safe-ack only. */
static void usb4_int_demux(void) {
  uint8_t c80a = PR(0xC80A);
  if (c80a & 0x20) {                 /* C80A.5 -> SB router/connect (bank1 0xA066) */
    usb4_int_seen |= 0x01;
  }
  if (c80a & 0x10) {                 /* C80A.4 -> USB4 adapter/link event (bank0 0xC105) */
    usb4_int_seen |= 0x02;
  }
  if (PR(0xEC06) & 0x01) {           /* EC06.0 -> router-op mailbox (bank1 0xC0A5) */
    usb4_int_seen |= 0x04;
    PR(0xEC04) = 1;                  /* documented ack (orchestrator @0x4486) */
  }
  if (c80a & 0x0F) {                 /* C80A.0-3 -> PCIe-tunnel link events (bank1 0xE911) */
    usb4_int_seen |= 0x08;
    /* documented W1C of the tunnel link-event reg (orchestrator/E911 RE): bit2->0x04, bit3->0x08 */
    { uint8_t e763 = PR(0xE763);
      if (e763 & 0x04) PR(0xE763) = 0x04;
      if (e763 & 0x08) PR(0xE763) = 0x08; }
  }
}

#endif /* USB4_H */
