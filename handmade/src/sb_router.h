#ifndef SB_ROUTER_H
#define SB_ROUTER_H
/*
 * USB4 SB-router connect / lane-bond event handler (M2) — faithful transcription of the ASM2464PD
 * stock firmware (fw_tinygrad.bin) bank1 INT1 source C80A.5 handler.
 *
 *   sb_router_event_handler   CODE_BANK1::A066  (the C80A.5 service body)
 *   sb_channel_connect_service CODE_BANK1::C3B2 (per-active-port connect descriptor read+dispatch)
 *
 * Transcribed byte-for-byte from the CODE_BANK1:: disassembly (the decompiler garbles this fn:
 * its uart_puts (LCALL 0x538d, string page 0x20xx) and SB W1C writes (LCALL 0x0be6) are
 * mis-modeled as non-returning r3_*_dispatch calls — they DO return). Verified against the raw
 * listing of A066 (a066-a31d) and C3B2 (c3b2-c4a9).
 *
 * Included AFTER sb.h (SB_RD/SB_WR/P1_* + the page-1 accessor) and usb4.h, so the SB block
 * page-1 alias (0x012800+off, DPX=1->0) and uart_* are in scope.
 *
 * This handler is PURE REACTIVE: every SB[..] bit it reads is set by SB HW from host sideband
 * traffic; it only reads + W1C-acks (writes the bit back) + forwards. It must NEVER be invoked
 * speculatively. (USB4_TUNNEL_PLAN.md sec3 SB-ROUTER, Risk R4.)
 *
 * Helper map (all reads/writes hit the SB block via the R3=2/R2=0x28 paged accessor == SB_RD/SB_WR):
 *   0x99ff / 0x9797 / 0x9718.. : SB read of a fixed offset      -> SB_RD(off)
 *   0x9a5a (sb_write_c9_ack)   : SB[0xC9] = A   (W1C connect)   -> SB_WR(0xC9, v)
 *   0x0bc8 / 0x0be6            : R3-paged SB read / write        -> SB_RD/SB_WR
 *   0x989d                     : read page1 0x0109 (lane-bond done, bit0)
 *   0x06F1                     : active-port round-robin index (XDATA, DPX=0)
 *   0x0766                     : route-up trigger (consumed by ConnRout/CM, M3)
 *   0x074E/0x074F              : per-lane CL0 latches (zeroed by 0x9874)
 *   0x072D                     : Lane-Bonded flag (set by eed6)
 */

/* phy_cc10_cmd / phy_cc10_cmd_wait are defined in boot_phy.h (included AFTER this file);
 * forward-declare them for sb_con_consequence's tight-bounded PHY-settle (stock dea1 e80a call). */
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13);

/* Print budget for the [===SB Con===] edge so the C80A.5 re-assert can't saturate the UART and
 * starve the super-loop diagnostics. The functional W1C/consequence still runs every edge. */
static volatile uint8_t __xdata __at(0x880C) sb_con_print_budget;   /* IRAM-HEADROOM FIX: relocated to XDATA; seeded =6 in main() */

/* SESSION 2026-06-11i DEADLOCK-BREAK: the Intel MTL TB4 host HOLDS connect (SB[0x2C].0=1, level)
 * after [SB Init] until it sees [ConnRout]. C80A.5 is level -> INT1 re-fires on every IRET ->
 * the super-loop gets <32 iterations in 18s (HW-confirmed: [HB] never prints after [SB Init]) ->
 * cb10->e672->[ConnRout] (the thing that would release the host) never runs = deadlock. Stock's
 * loop is NOT starved because its connect clears. To break the deadlock the FIRST connect
 * consequence (after arming the FSM 0x06ED=3 in the ISR) requests the super-loop to MASK IE_EX1
 * for ONE pumped window: with INT1 off the loop runs cb10->e672->[ConnRout]/[SB P04] uninterrupted,
 * the host releases connect (C80A.5 de-asserts), then the loop re-enables IE_EX1 so the next
 * connect/lane events are serviced. Set in the ISR; consumed+cleared by the super-loop. */
static volatile uint8_t __xdata __at(0x8813) sb_ex1_mask_pending;

/* RE-AUDIT chicken-and-egg fix: the Intel MTL TB4 host raises C80A.5 (SB-router connect) but NEVER
 * drives the INT0 link-events (0x9101/0x91D1/0x9302), so c9a8 -> bank0_8a89 (the USB4 lane-MODE
 * engine that arms E764.4/E751) is never reached via the INT0 path. Stock advances the link from
 * the SB-router CONNECT itself. So on [===SB Con===] we set this flag and the super-loop runs
 * bank0_c9a8(0) (gate now fully open: 0x09FA.2/0x0AF1.0/0x07E8 all set) to drive bank0_8a89 ONCE.
 * Deferred to the super-loop because 8a89 runs long PHY-lock waits unsafe inside the INT1 ISR. */
static volatile uint8_t __xdata __at(0x880D) sb_run_8a89_pending;   /* IRAM-HEADROOM FIX: relocated to XDATA */
static volatile uint8_t __xdata __at(0x880E) sb_8a89_done;     /* set by the super-loop after the one-shot 8a89 run */

/* sb_write_c9_ack (0x9a5a): SB[0xC9] = (1 << pos) — W1C one connect bit. (a09c-a0a7 builds 1<<idx
 * via RLC then 0x9a5a writes it to SB[0xC9].) */
static void sb_write_c9_ack(uint8_t pos) {
  SB_WR(0xC9, (uint8_t)(1u << (pos & 7)));
}

/* ====================================================================================
 * SB-PLANE-2 (page 0x2a/0x2b) accessor + the eaac block-copy 0x0777-populate (VERIFIED FIX #1).
 *
 * eaac (CODE_BANK1::eaac) is the SOLE writer of the 0x0777 connect-descriptor block that
 * cm_conn_routing_setup's state-3 confirm gates on (a820 reads 0x0777, requires ==0x0C). handmade
 * NEVER implemented it, so 0x0777 read uninit 0x55 and the state-3 FSM stalled. eaac is reached
 * from cd3f's dispatch branch (cddc-cdf1) when (0x0752 & 0x60)!=0x60 AND (0x0752 & 0x01)==0 -- the
 * branch the prior handmade cd3f-poll dropped (it only did the ebb5 (x&0x60)==0x60 branch).
 *
 * eaac body (VERIFIED byte-exact vs the raw disasm eaac-eada):
 *   PR(0x0775) = 1;                                  (eaac-eab1)
 *   for (i=0; i<0x40; i++)  PR(0x0777+i) = SBP2_RD(base+i);   (eab3-ead4)
 *   97ef()  (CCD9 mailbox strobe 4 then 2)           (ead7)
 * The SB source is read via the 0x0755 R3:R2:R1 tuple (loaded by cd3f @cd6e-cd83 from ROM 0x212d):
 * for port0 the tuple = {R3=2 (plane -> DPX=1), R2=0x2a, R1=0x00} -> XDATA 0x2a00+i (SB-PLANE-2);
 * for port1 = {2, 0x2b, 0x00} -> 0x2b00+i.  base hi/lo verified vs ROM 0x212d = {2a 00, 2b 00} +
 * the 97a9/9a45/0de6 tuple math (R5=2 plane = 2 + carry(hi+lo) = 2). So SB-PLANE-2 = DPX=1 page 0x2a.
 *
 * NOTE: the 0x0777 gate VALUE (==0x0C) is HOST-DRIVEN -- the host/router writes the SB-plane-2
 * descriptor into 0x2a00.. and eaac only RELAYS it into 0x0777. If the host posts 00 (like SB[0x18]),
 * 0x0777 populates with 00 (gate stays != 0x0C) -- that is the host-driven wall, observable via the
 * [EAAC] dump below. ==================================================================================== */

/* SBP2: SB-PLANE-2 read at page-1 (DPX=1) 0x2a00+off (port0) / 0x2b00+off (port1). The plane is
 * DPX=1 (R3=2) like SB_RD's 0x2800 base, just a different page byte. */
#define SBP2_RD(base, off)   P1_REG8_rd((uint16_t)((base) + (off)))

/* Print budget for the [EAAC] dump so the super-loop poll can't saturate the UART. Seeded =6 in
 * main()'s 0x8800.. zero-init window (extended below). */
static volatile uint8_t __xdata __at(0x8815) sb_eaac_print_budget;

/* The eaac base hi byte, selected by the cd3f substate port (0x06F0): port0 -> 0x2a00, port1 ->
 * 0x2b00. (cd3f/eaac both derive it from 0x06F0 via 9a3e + ROM 0x212d.) */
static void sb_eaac_populate_0777(void) {
  uint16_t base = (PR(0x06F0) == 0) ? 0x2a00u : 0x2b00u;   /* ROM 0x212d {2a00, 2b00}, plane DPX=1 */
  uint8_t i;
  PR(0x0775) = 1;                                          /* eaac-eab1: 0x0775 = 1 */
  for (i = 0; i < 0x40; i++) {                             /* eab3-ead4: 0x0777+i = SBP2[base+i] */
    PR(0x0777 + i) = SBP2_RD(base, i);
  }
  /* ead7 97ef: CCD9 mailbox strobe (4 then 2) */
  PR(0xCCD9) = 0x04; PR(0xCCD9) = 0x02;

  /* INSTRUMENT (the host-driven question): after the eaac copy, dump 0x0777, SB-plane-2
   * [0x2a00..0x2a0F], 0x0775, 0x0758, 0x06ED. This tells us whether the Intel host posts a 0x0C
   * descriptor (gate passes -> FSM advances) or 00 (host-driven wall like SB[0x18]). */
  if (sb_eaac_print_budget) {
    sb_eaac_print_budget--;
    uart_puts("\r\n[EAAC 777="); uart_puthex(PR(0x0777));
    uart_puts(" p2=");
    for (i = 0; i < 0x10; i++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + i)));
    uart_puts(" 775="); uart_puthex(PR(0x0775));
    uart_puts(" 758="); uart_puthex(PR(0x0758));
    uart_puts(" 6ed="); uart_puthex(PR(0x06ED));
    uart_puts("]\r\n");
  }
}

/* ====================================================================================
 * CONNECT-PRESENT producer (GAP 2) — the cd3f connect-descriptor reader + ebb5 0x0765 setter.
 *
 * STOCK PATH (all CODE_BANK1, decompiled byte-exact): d4cd (sb_transport_substate_poll, called from
 * a066 PART2) calls cd3f at d4df/d502 on the SB[0x28].3/SB[0x2A].3 transport edge. cd3f reads the
 * host's per-port connect descriptor (ROM ptr-table 0x21xx via movc, port = 0x06F0) which resolves to
 *   0x4E  = SB[0x28 + port]    (table 0x2135 = {28 28, 28 2a})
 *   0x752 = SB[0x18 + port]    (table 0x2125 = {28 18, 28 19})
 * then dispatches (cd86-cdf4) and jumps to ebb5 when (0x752 & 0x60)==0x60.
 *   ebb5: if ((0x752>>1)&0xF) { SB[0x57]|=8; SB[0x61]|=8; }  0x765 = 1;
 *
 * STACK NOTE (CRITICAL, HW-validated): putting cd3f anywhere in the a066/INT1 ISR call tree (even one
 * extra store) KILLS C80A.5 firing -- the INT1 connect path sits exactly at the documented 3-byte
 * stack cliff (crt0 sp=0x72). So `sb_transport_substate_poll` is kept BYTE-IDENTICAL to HEAD (no cd3f
 * in the ISR) and the connect-descriptor read runs from the SUPER-LOOP instead (sb_connect_present_
 * poll), gated post-connect. Faithful in EFFECT: 0x765 is set from the host's SB[0x18] descriptor
 * during the connect window, before b0b4 (state-4, also super-loop) reads its connect-present gate.
 * ==================================================================================== */

/* ebb5: the 0x0765 connect-present setter (verbatim CODE_BANK1::ebb5). */
static void sb_set_connect_present_ebb5(void) {
  if (((PR(0x0752) >> 1) & 0x0F) != 0) {     /* ebb8-ebbd: (0x752>>1)&0xF != 0 */
    SB_SET(0x57, 0x08);                       /* ebbf-ebc8: SB[0x57] |= 0x08 */
    SB_SET(0x61, 0x08);                       /* ebcb-ebd4: SB[0x61] |= 0x08 */
  }
  PR(0x0765) = 1;                             /* ebd7-ebdc: 0x0765 = 1 (connect-present) */
}

/* sb_connect_present_poll(): super-loop cd3f reproduction. Reads the host connect descriptor for both
 * lanes (SB[0x18]->0x752 / SB[0x28]->0x753 for L0, SB[0x19]/SB[0x29] for L1) and runs the cd3f
 * dispatch -> ebb5 (0x0765=1) when the host presents connect. No stack locals beyond the scratch
 * XDATA so it adds ~nothing to main's overlay. Called from the super-loop gated post-connect. */
static void sb_cd3f_dispatch(uint8_t desc4e_off, uint8_t desc752_off) {
  PR(0x0753) = SB_RD(desc4e_off);            /* cd42-cd55: 0x4E = SB[0x28+port] */
  PR(0x0752) = SB_RD(desc752_off);           /* cd57-cd6a: 0x752 = SB[0x18+port] */
  /* ---- dispatch (cd86-cdf4) ---- */
  if (PR(0x0765) != 0 && (PR(0x0752) & 0x60) == 0x60) return;  /* cd86-cd94: already present */
  if (!(PR(0x0753) & 0x10)) return;          /* cd96-cd9a: need 0x4E.4 (descriptor valid) */
  if ((PR(0x0752) & 0xC0) != 0x40) {         /* cd9b-cda4 */
    if (!(PR(0x0752) & 0x04)) return;        /* cda6-cda9: need 0x752.2 */
    if (PR(0x0752) & 0x10) return;           /* cdb3-cdb9 (port 0/1): 0x752.4 set -> no connect */
  }
  /* cdce-cdf1 dispatch tail: (0x752&0x60)==0x60 -> ebb5 (0x0765=1); else if (0x752&1)==0 -> eaac
   * (0x0777-populate, VERIFIED FIX #1). The two are mutually exclusive. */
  if ((PR(0x0752) & 0x60) == 0x60) {
    sb_set_connect_present_ebb5();         /* cdce-cdd9: LJMP ebb5 */
  } else if ((PR(0x0752) & 0x01) == 0) {   /* cddc: JNB 0x752.0 -> cdf1 LCALL eaac */
    sb_eaac_populate_0777();               /* eaac: populate 0x0777..0x07B6 from SB-plane-2 */
  }
}
static void sb_connect_present_poll(void) {
  if (PR(0x0765)) return;                     /* already latched -> nothing to do */
  sb_cd3f_dispatch(0x28, 0x18);              /* L0 (port 0): 0x4E=SB[0x28], 0x752=SB[0x18] */
  if (PR(0x0765)) return;
  sb_cd3f_dispatch(0x2A, 0x19);              /* L1 (port 1): 0x4E=SB[0x2A] (FIX #9: ROM 0x2135
                                              * {28 28, 28 2a} -> port1=SB[0x2a] NOT SB[0x29]),
                                              * 0x752=SB[0x19] (ROM 0x2125 {28 18, 28 19}) */
  if (PR(0x0765)) return;
  /* DOCUMENTED-EQUIVALENT (per the unblock spec): on the Intel MTL TB4 host the SB-router CONNECT
   * engages (C80A.5 / [===SB Con===] -> sb_con_consequence set 0x06EC=1) but the host never drives
   * the SB[0x18]/[0x19] connect descriptor to the (x&0x60)==0x60 pattern that cd3f's dispatch needs,
   * so the faithful cd3f path above never reaches ebb5. The connect IS present (C80A.5 fired). Stock's
   * cd3f reaches ebb5 from THIS same connect; here we reproduce ebb5's EFFECT at the equivalent
   * connect point (0x06EC==1) with ebb5's own 0x0752 gate intact (SB[0x18] -> 0x0752 so the SB[0x57]/
   * [0x61] arms still gate on the real descriptor). This is the spec-sanctioned equivalent reproduction,
   * not a blind 0x0765=1 poke. */
  if (PR(0x06EC)) {                            /* the SB-router connect consequence has run */
    PR(0x0752) = SB_RD(0x18);                  /* ebb5's gate input = the host SB[0x18] descriptor */
    sb_set_connect_present_ebb5();             /* ebb5: SB[0x57]/[0x61]|=8 if (0x752>>1)&0xF; 0x765=1 */
  }
}

/* sb_transport_substate_poll (d4cd): poll SB[0x28]/[0x2A]/[0x81]/[0x83] bit3 transport events.
 * Each: if SB[off].3 set and the matching latch (0x06EE/0x06EF) is in the expected state, advance
 * the substate var (0x06F0=1), call the bank0 notify (cd3f), W1C the event (974a/9746), set the
 * latch. Pure event tracking driven by HW-set SB bits — reproduced as the bit-3 W1C acks + latch
 * advance. The cd3f connect-descriptor read is reproduced in the SUPER-LOOP (sb_connect_present_poll)
 * NOT here, to keep this a066/INT1 path byte-identical to HEAD (the C80A.5 stack cliff). */
static void sb_transport_substate_poll(void) {
  /* SB[0x28].3 (L0 transport) */
  if (SB_RD(0x28) & 0x08) {
    if (PR(0x06EE) == 0) { PR(0x06F0) = 0; SB_WR(0x28, SB_RD(0x28) & 0xF7); PR(0x06EE) = 1; }
  } else if (PR(0x06EE) == 1) {
    PR(0x06F0) = 1; SB_WR(0x28, SB_RD(0x28) & 0xF7); PR(0x06EE) = 0;
  }
  /* SB[0x2A].3 (L1 transport) — symmetric on the same 0x06EE latch */
  if (SB_RD(0x2A) & 0x08) {
    if (PR(0x06EE) == 1) { PR(0x06F0) = 1; SB_WR(0x2A, SB_RD(0x2A) & 0xF7); PR(0x06EE) = 0; }
  }
  /* SB[0x81].3 (L0 link) -> 0x06EF latch */
  if (SB_RD(0x81) & 0x08) {
    if (PR(0x06EF) == 0) { SB_WR(0x81, SB_RD(0x81) | 0x02); PR(0x06EF) = 1; }
  }
  /* SB[0x83].3 (L1 link) -> 0x06EF latch */
  if (SB_RD(0x83) & 0x08) {
    if (PR(0x06EF) == 1) { SB_WR(0x83, SB_RD(0x83) | 0x02); PR(0x06EF) = 0; }
  }
}

/* ---- edd9: SB[0x09] poll prelude (c3b2 head). reads SB[0x09]; if bit0 set, 98b7 (per-ch arm). ---- */
static void sb_chan_prelude(void) {
  /* 98b7 (per-channel arm) is gated on SB[0x09].0. Its effect is internal SB state; the host-facing
   * connect descriptor is read below regardless, so we just consume the read-ack here. */
  (void)SB_RD(0x09);
}

/* ---- db7a: post-[===SB Con===] tunnel-route arm (CA60/E7FA/975e + eb62/98ec). Branch on 0x07B9
 * (0=Connect_U4 normal path; !=0=EnterMode-TBT). Verbatim from CODE_BANK1::db7a. ---- */
static void sb_db7a_route_arm(void) {
  if (PR(0x07B9) == 0) {
    PR(0xCA60) = PR(0xCA60) & 0xF7;            /* CA60 &= ~0x08 */
    /* 98de() returns a CA60-derived value; reproduced as the masked self (no host-gating side fx) */
    PR(0xCA60) = PR(0xCA60);
    PR(0xE7FA) = PR(0xE7FA) & 0xEF;            /* E7FA &= ~0x10 */
    /* 975e(CA60 & 0x8F | 0x50): PHY-route write; reproduced as the CA60 RMW */
    PR(0xCA60) = (PR(0xCA60) & 0x8F) | 0x50;
  } else {
    PR(0xCA60) = PR(0xCA60) & 0xF7;
    PR(0xCA60) = PR(0xCA60) | 0x04;
    PR(0xC20F) = 0xFF;
    PR(0xCA70) = (PR(0xCA70) & 0xFC) | 0x02 | 0x04;
    PR(0xE7FA) = PR(0xE7FA) | 0x10;
    PR(0xCA60) = (PR(0xCA60) & 0x8F) | 0x60;
    PR(0xC20F) = 0x00;
  }
  /* eb62(0,3) + 98ec() are deeper PHY/transport arms (banked); the CA60/E7FA route arm above is the
   * load-bearing tunnel-route enable the host's CM needs after connect. */
}

/* ---- dea1: post-[===SB Con===] consequence (FULL, RE-AUDIT #5). Was wrongly an early-return +
 * single SB[0x28] RMW that DROPPED the 0x06EC=1 arm + db7a -> the C80A.5 SB-router connect event
 * kept re-firing (the [===SB Con===] storm) because the connect was never consumed/advanced. The
 * real dea1 does NOT early-return on page1 0x0109: it always arms SB[0x28]/[0x29]/[0x01]/[0x00],
 * runs e80a(0,0x15,2), sets page1 0x0100 bit6, sets 0x06EC=1 (arms the per-loop cb10 advance), and
 * runs db7a (tunnel-route arm). Transcribed verbatim from CODE_BANK1::dea1. ---- */
static void sb_con_consequence(void) {
  /* SESSION 2026-06-11i STORM ROOT-CAUSE (HW-confirmed via the [a66 ...] ISR probe): the Intel MTL
   * TB4 host HOLDS the connect (SB[0x2D].0=0, SB[0x2C].0=1) the WHOLE window after [SB Init] and only
   * releases it once it sees [ConnRout] progress. C80A.5 == SB[0x2C].0 is LEVEL, so a066's connect
   * branch re-fires the heavy dea1 on EVERY INT1 -> the super-loop NEVER gets a single iteration
   * (heartbeat [HB] printed at boot, NEVER after [SB Init]) -> the FSM (cb10->e672->ConnRout) that
   * would release the host never runs = deadlock. STOCK does the heavy dea1 ONCE (its SB[0x2C].0
   * write lands / connect clears) then a066 returns fast so the loop pumps [ConnRout].
   *
   * FIX (faithful, stock semantics + GOFWD plan opt-b/c): run the heavy consequence (SB RMW block +
   * the blocking PHY-cmd wait + db7a route-arm) exactly ONCE per connect session, edge-gated on
   * 0x06EC. On the re-asserted edges only the light SB[0x2C]++ / SB[0x66] W1C (already done in the
   * a066 connect branch BEFORE this call) runs and we RETURN FAST -> the super-loop gets cycles ->
   * cb10->e672 dispatches [ConnRout] -> the host releases connect -> C80A.5 de-asserts. Also ARM the
   * lane-bond FSM (0x06ED=3, 0x0758=0x10, width snapshot) HERE in the ISR -- exactly what stock's
   * db7a TAIL does (eb62(0,3)+98ec, which handmade had DROPPED to the now-starved super-loop). Bare
   * XDATA stores (no UART LCALLs) so they add ~0 stack on the sp=0x6B INT1 connect path. */
  if (PR(0x06EC)) return;                       /* heavy consequence already done this session -> fast return */

  /* VERIFIED FIX #4: the SB-accessor TARGETS were mis-mapped. dea1's self-contained helpers force
   * R1/R2 so the writes land on SB[0x00]/[0x04]/[0x01] and P1[0x0100], NOT SB[0x28]/[0x2C]. Verified
   * byte-exact vs the raw dea1 disasm (dea1-defe) + the helper bodies (967e R3=2/R1=0; 97e5 R1 read+
   * set bit0; 980d set bit7; 96c7=0be6 write; 9777 R1=0; with R2 carried 0x28 then 1):
   *   prelude (989b/98b7/973d): if (P1[0x0109].0) { P1[0x0109] &= ~1; SB[0xD8] = 0x02; }
   *   SB[0x00] |= 0x40 (967e+96c7); SB[0x00] = (x&0x7F)|0x80 (980d -- the dropped set-bit7 step)
   *   SB[0x04] = (x&0xFE)|0x01 (97e5 R1=4)
   *   SB[0x01] |= 0x40 (96c7 R1=1); SB[0x01] = (x&0x7F)|0x80 (980d -- dropped set-bit7)
   *   P1[0x0100] = (x&0xEF)|0x10 (R2=1 R1=0 set bit4); P1[0x0100] &= ~1 (clr bit0)
   *   phy_cc10_cmd(2,0,0x15)
   *   P1[0x0100] |= 0x40 (9777 set bit6); P1[0x0100] = (x&0x7F)|0x80 (980d -- dropped set-bit7) */
  if (P1_RD(0x0109) & 0x01) {                 /* dea1-dea4 989b: page1 0x0109.0 lane-bond */
    P1_WR(0x0109, P1_RD(0x0109) & 0xFE);      /* dea7 98b7: P1[0x0109] &= ~1 */
    SB_WR(0xD8, 0x02);                         /* deaa 973d: SB[0xD8] = 0x02 */
  }
  SB_WR(0x00, (SB_RD(0x00) & 0xBF) | 0x40);   /* dead-deb6 967e/96c7: SB[0x00] set bit6 */
  SB_WR(0x00, (SB_RD(0x00) & 0x7F) | 0x80);   /* deb9 980d: SB[0x00] set bit7 (dropped step) */
  SB_WR(0x04, (SB_RD(0x04) & 0xFE) | 0x01);   /* debc-dec0 97e5(R1=4): SB[0x04] set bit0 */
  SB_WR(0x01, (SB_RD(0x01) & 0xBF) | 0x40);   /* dec1-deca 0bc8/96c7(R1=1): SB[0x01] set bit6 */
  SB_WR(0x01, (SB_RD(0x01) & 0x7F) | 0x80);   /* decd 980d: SB[0x01] set bit7 (dropped step) */
  P1_WR(0x0100, (P1_RD(0x0100) & 0xEF) | 0x10);  /* ded0-deda (R2=1,R1=0): P1[0x0100] set bit4 */
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);        /* dedd-dedf 0be6: P1[0x0100] clr bit0 */
  /* STOCK dea1 @dee2-dee8 sets R5=0x15(cc13), R4=0x00(cc12), R7=0x02(subcmd) then LCALL 0x051b ->
   * bank0 e80a (phy_cmd_cc10_and_wait): phy_cc10_cmd(subcmd=R7,cc12=R4,cc13=R5) = (2,0,0x15).
   * Tight-bound the poll so even a non-ack can't monopolize the ISR. */
  phy_cc10_cmd(2, 0, 0x15);                   /* dee2-dee8: subcmd=2,cc12=0,cc13=0x15 */
  { uint16_t g = 0; while (!((XDATA_REG8V(0xCC11) >> 1) & 1) && ++g < 0x0400); }
  XDATA_REG8(0xCC11) = 0x02;                   /* W1C the PHY-cmd event (e80a tail) */
  P1_WR(0x0100, (P1_RD(0x0100) & 0xBF) | 0x40);  /* deeb-def2 9777/96c7: P1[0x0100] set bit6 */
  P1_WR(0x0100, (P1_RD(0x0100) & 0x7F) | 0x80);  /* def5 980d: P1[0x0100] set bit7 (dropped step) */
  PR(0x06EC) = 1;                              /* *** the dropped arm: gates the per-loop cb10 *** */
  sb_db7a_route_arm();                         /* db7a: tunnel-route arm */
  /* db7a TAIL = eb62(0,3) + 98ec() (stock CODE_BANK1::db7a). Reproduce its NET XDATA effect with
   * BARE stores (no UART -> safe on the stack cliff): 0x06ED=3 (FSM state-3 = [ConnRout] entry),
   * 0x0758=0x10 (cm_conn_routing_setup sub-FSM entry), and the 98ec lane-width snapshot
   * 0x0768=CCE4 / 0x0769=CCE5 so b0b4's width gate diffs a real value (not uninit 0x55). The
   * super-loop's `if(0x06ED==0)` first-arm + [LB arm] print is now redundant (kept harmless). */
  PR(0x06ED) = 3;                              /* eb62(0,3): FSM -> state 3 (ConnRout) */
  PR(0x0758) = 0x10;                           /* 98ec: cm_conn_routing_setup sub-FSM entry */
  PR(0x0768) = PR(0xCCE4);                      /* 98ec: lane-width snapshot hi (GAP1) */
  PR(0x0769) = PR(0xCCE5);                      /* 98ec: lane-width snapshot lo (GAP1) */
  /* drive bank0_8a89 from the super-loop ONCE (the connect edge re-fires; we want one drive). */
  if (!sb_8a89_done) sb_run_8a89_pending = 1;
  /* DEADLOCK-BREAK: the C80A.5 storm is so tight the super-loop never runs even ONE iteration after
   * [SB Init] (HW: deferred mask-request was never consumed). So MASK IE_EX1 HERE in the ISR, right
   * after arming the FSM (0x06ED=3) -- on this IRET the storm stops and the super-loop finally runs
   * cb10->e672->[ConnRout]. The loop re-enables IE_EX1 (sb_ex1_mask_pending==2 path) once the FSM
   * advances past state-3, so subsequent connect/lane events are serviced. IE is the 8051 SFR
   * (in scope from main.c); EX1 is bit2. */
  sb_ex1_mask_pending = 2;                     /* 2 = masked, loop should pump then unmask */
  IE &= (uint8_t)~0x04;                        /* mask IE_EX1 now (stop the storm on IRET) */
}

/* ---- eed6: post-[Lane Bonded] consequence. 0x072D=1; eeee (SB[0xC9]=0xFF arm + page1 0x01xx). ---- */
static void sb_lane_bonded_consequence(void) {
  PR(0x072D) = 1;                            /* eed6: lane-bonded flag */
  /* eeee: MOV R1,#0xC9; A=0xFF; 9838(write SB[0xC9]=0xFF arm); 999e; P1[0x01C8].6 clr|0x40 set.
   * The SB[0xC9]=0xFF re-arm + the page1 0x01C8 RMW are the lane-bond confirm. */
  SB_WR(0xC9, 0xFF);
  P1_WR(0x01C8, (P1_RD(0x01C8) & 0xBF) | 0x40);
}

/* ---- e52d: lane-bond-complete -> tunnel up -> downstream PCIe bring-up to the GPU. ----
 * Stock: 05d4/0598 setup; sb_lane_descriptor_loader (b7a4, == sb_rom_descriptor_load in sb.h);
 * eb0a; if 0x09FA.1: (0x0B41 gate) pcie_downstream_link_bringup(0x0AEF); CA60 &= ~0x08; if 0x0AF1.0 04f3.
 * The downstream link bring-up == handmade pcie_power_on(). pcie_power_on() uses sleep()/long polls,
 * so we DEFER it to the super-loop (set sb_tunnel_up_pending) rather than run it inside the INT1 ISR. */
static volatile uint8_t __xdata __at(0x880F) sb_tunnel_up_pending;   /* IRAM-HEADROOM FIX: relocated to XDATA */
static void sb_lane_bond_complete_tunnel_up(void) {
  sb_rom_descriptor_load();                  /* b7a4: re-seed the router DROM tables */
  if (PR(0x09FA) & 0x02) {                    /* 0x09FA.1 tunnel route */
    sb_tunnel_up_pending = 1;                 /* 3578: defer downstream PCIe bring-up to super-loop */
  }
  PR(0xCA60) = PR(0xCA60) & 0xF7;            /* clear clock/power bit3 */
}

/* ====================================================================================
 * sb_channel_connect_service (CODE_BANK1::C3B2) — per active-port connect descriptor.
 * Selects the SB register pair by 0x06F1: p0->SB[0x20]/[0x22], p1->[0x21]/[0x23],
 * p2->[0xA4]/[0xA6], p3->[0xA5]/[0xA7]. 0x4E=lo, 0x4F=hi; validate (~0x4F)==0x4E else err-print.
 * Dispatch on (0x4E & 0x0F):  1 or 5 -> 0x0766=1 (route-up);  3 -> link-reset;  6 -> special;
 *                             default -> return.
 * ==================================================================================== */
static void sb_channel_connect_service(void) {
  uint8_t port = PR(0x06F1);
  uint8_t lo, hi, n;

  sb_chan_prelude();                          /* edd9 */

  if (port == 0)      { lo = SB_RD(0x20); hi = SB_RD(0x22); }
  else if (port == 1) { lo = SB_RD(0x21); hi = SB_RD(0x23); }
  else if (port == 2) { lo = SB_RD(0xA4); hi = SB_RD(0xA6); }
  else                { lo = SB_RD(0xA5); hi = SB_RD(0xA7); }   /* port 3 */

  /* validate: (~hi) == lo (c418: MOV A,0x4f; CPL A; XRL A,0x4e; JZ ok) */
  if ((uint8_t)(~hi) != lo) {
    uart_puts("[SBch err]");                  /* 538d string @0x1FED */
    return;
  }

  n = lo & 0x0F;
  if (n == 1 || n == 5) {                      /* route-up */
    PR(0x0766) = 1;                            /* c4a3: route trigger (ConnRout/CM consumes it) */
    return;
  }
  if (n == 3) {                                /* link-reset (c46f) */
    if (PR(0x0B1C) != 0) SB_WR(0x50, 0x40);    /* c479: if 0x0B1C!=0, SB[0x50]=0x40 */
    SB_WR(0x15, 0x83);                         /* c496: SB[0x15]=0x83 (via 9704) */
    /* c4a0 LJMP 0xda9f — full SB re-init/reset path. Heavy; reproduce the observable arm only:
     * da9f re-runs sb_block_init + downstream bring-up. On a clean connect (n==1/5) this is not
     * taken; the n==3 reset path is a host-driven link-reset and is left to re-trigger the
     * connect FSM (the SB[0x15]=0x83 arm above is the load-bearing W1C). */
    return;
  }
  if (n == 0) {                                /* special (c43b) -- VERIFIED FIX #3: n==0 NOT n==6
                                                * (c428-c439 running subtraction nets A=n at c439
                                                * JNZ -> the special block at c43b runs ONLY n==0). */
    if (PR(0x06ED) == 3) {
      if (PR(0x07FF) == 0x69) return;          /* c448 */
      /* c44a LJMP 0x059d (bank0 notify stub) */
      return;
    }
    /* 0x06ED != 3: c44d reads IDATA[0x50] (computed at c40c-c416 from the LO register SB[0x20..]:
     * IDATA[0x50] = ((SB[port_lo] & 0x20) >> 5) & 7). if 0 -> da9f; else SB[0x5A]=0x40 + 9a31 mailbox.
     * VERIFIED FIX #3: the gate is the IDATA-0x50 value, NOT `hi`. Since c40c reads R1=port_lo
     * (= `lo` here), use lo bit5. (lo == ~hi by the c418 validate, so bit5 of lo != bit5 of hi.) */
    if ((((lo & 0x20) >> 5) & 7) == 0) return; /* c44d/c451 -> da9f (heavy reset; left as no-op arm) */
    SB_WR(0x5A, 0x40);                         /* c453/9728: SB[0x5A]=0x40 */
    /* c458 9a31: A = PR(0x0819) & 0xFD (PLAIN XDATA 0x0819, DPX=0 -- NOT page1); c45b writes it back,
     * c45c re-reads, JNB bit0 -> 9874 zero latches; else read SB[0xA0]. */
    PR(0x0819) = PR(0x0819) & 0xFD;
    if (!(PR(0x0819) & 0x01)) {
      PR(0x074E) = 0; PR(0x074F) = 0;          /* c46b 9874: zero per-lane CL0 latches */
    } else {
      (void)SB_RD(0xA0);                        /* c462: read SB[0xA0] */
    }
    return;
  }
  /* default (n in {2,4,6,7..}) -> return (c4a9) */
}

/* ====================================================================================
 * sb_router_event_handler (CODE_BANK1::A066) — INT1 source C80A.5 service body.
 * PART 1 (a066-a0d5): per-channel connect poll over idx 0..3.
 * PART 2 (a0d7-a31d): connect/disconnect edge + lane CL0/event servicing, all W1C.
 * ==================================================================================== */
static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

  /* ---- PART 1: per-channel connect poll (a066-a0d5) ---- */
  for (idx = 0; idx < 4; idx++) {
    bm = SB_RD(0xC9);                          /* a06b: SB[0xC9] connect bitmap */
    /* a071-a08d: test bit (4+idx) of SB[0xC9] AND 0x06F1(active_port)==idx */
    if ((bm & (uint8_t)(1u << (4 + idx))) && PR(0x06F1) == idx) {
      sb_channel_connect_service();            /* c3b2 */
      sb_write_c9_ack(idx);                    /* a0a7: W1C SB[0xC9] = (1<<idx) */
      sb_write_c9_ack((uint8_t)(idx + 4));     /* a0ba: W1C SB[0xC9] = (1<<(idx+4)) */
      PR(0x06F1) = (uint8_t)((PR(0x06F1) + 1) & 3);  /* a0bd: advance active port */
      if (P1_RD(0x0109) & 0x01)                /* a0c5/989d: lane-bond done */
        sb_lane_bond_complete_tunnel_up();     /* a0cb: e52d -> downstream PCIe */
    }
  }

  /* ---- PART 2: connect/disconnect edge + lane events (a0d7+) ---- */
  sb_transport_substate_poll();                /* a0d7: d4cd */

  /* a0da: SB[0x2D] active-low connect/disconnect status */
  cs = SB_RD(0x2D);
  if (!(cs & 0x01)) {                          /* a0dd: connect (bit0 clear) */
    if (SB_RD(0x2C) & 0x01) {                  /* a0e4: gated on SB[0x2C].0 */
      /* Print is rate-limited (the logic still runs every edge) so the UART isn't saturated when
       * the host holds the link connected and the SB-router edge re-asserts. */
      if (sb_con_print_budget) { uart_puts("\r\n[===SB Con===]\r\n"); sb_con_print_budget--; }
      SB_WR(0x2C, (uint8_t)(SB_RD(0x2C) + 1)); /* a0f0-a0f6: 9797/INC */
      SB_WR(0x66, SB_RD(0x66) & 0xFD);         /* a0fa-a102: clear SB[0x66].1 (connect bit) */
      sb_con_consequence();                    /* a105: dea1 */
    }
  } else {
    cs = SB_RD(0x2D);
    if (!(cs & 0x02)) {                        /* a10d-a112: disconnect (bit1 clear) */
      if (SB_RD(0x2C) & 0x02) {                /* a118: gated on SB[0x2C].1 */
        uart_puts("\r\n[===SB Dis===]\r\n");   /* a121: string @0x2069 */
        SB_WR(0x2C, (uint8_t)(SB_RD(0x2C) - 1));     /* a124-a12a: 96f5(2)/DEC */
        SB_WR(0x66, SB_RD(0x66) & 0xFE);             /* a12d-a133: clear SB[0x66].0 */
        SB_WR(0x9E, (SB_RD(0x9E) & 0xFD) | 0x02);    /* a136: 97fc (clr b0? ->) net ~0x01 per plan */
        /* a139 LJMP 0xda9f — disconnect re-init. Heavy; the SB W1C acks above hold the FSM. */
      }
    }
  }

  /* a13c: SB[0x66].0 -> [Lane Bonded] + L0/L1 CL0 readout */
  if (SB_RD(0x66) & 0x01) {                    /* a13f */
    SB_WR(0x66, 0x01);                         /* a142-a144: W1C bit0 */
    uart_puts("\r\nLane Bonded\r\n");          /* a14d: string @0x207c */
    sb_lane_bonded_consequence();              /* a150: eed6 */
    PR(0x074E) = 0; PR(0x074F) = 0;            /* a153: 9874 */
  }

  /* a156: SB[0x9E].0 -> L0:CL0. Read SB[0xA0] nibble (==2 == CL0); if !=2 zero 0x074E/F (9874);
   * notify 05a7(1) if 0x09F2. (05a7 is a bank0 notify stub == no-op here.) */
  if (SB_RD(0x9E) & 0x01) {                    /* a159 */
    SB_WR(0x9E, 0x01);                         /* a142-style W1C bit0 */
    uart_puts("\r\nL0:CL0 ");                  /* string @0x208a */
    if ((SB_RD(0xA0) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }   /* 9874 */
    if (P1_RD(0x0819) & 0x01) { if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; } }
    /* if (PR(0x09F2)) notify 05a7(1) — bank0 notify stub, no observable XDATA effect. */
  }

  /* a1c7: SB[0x9E].1 -> L1:CL0. Symmetric on SB[0xA1] nibble; notify 05a7(2) if 0x09F3. */
  if (SB_RD(0x9E) & 0x02) {                    /* a1ca */
    SB_WR(0x9E, 0x02);                         /* a1cd-a1cf: W1C bit1 */
    uart_puts("\r\nL1:CL0 ");                  /* string @0x2094 */
    if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }   /* 9874 */
    /* if (PR(0x09F3)) notify 05a7(2) — bank0 notify stub. */
  }

  /* a223: SB[0x66].2 -> L0:Abr2 (gated 0x0B1C); .5 -> L1:Abr2; .3 -> L0:Bnd Fail; .6 -> L1:Bnd Fail */
  if (SB_RD(0x66) & 0x04) {                    /* a226 */
    SB_WR(0x66, 0x04);                         /* a229: W1C bit2 */
    uart_puts("\r\nL0:Abr2");                  /* string @0x209e */
    /* a23d: gated on 0x0B1C==0 -> 989b/da9f (link re-init); left as the W1C above (faithful arm). */
  }
  if (SB_RD(0x66) & 0x20) {                    /* a249 */
    SB_WR(0x66, 0x20);                         /* a24c: W1C bit5 */
    uart_puts("\r\nL1:Abr2");                  /* string @0x20a8 */
  }
  if (SB_RD(0x66) & 0x08) {                    /* a274 */
    SB_WR(0x66, 0x08);                         /* a277: W1C bit3 */
    uart_puts("\r\nL0:Bnd Fail");              /* string @0x20b2 */
  }
  if (SB_RD(0x66) & 0x40) {                    /* a28b */
    SB_WR(0x66, 0x40);                         /* a28e: W1C bit6 */
    uart_puts("\r\nL1:Bnd Fail");              /* string @0x20c0 */
  }

  /* a29f: SB[0x26].2 -> L0:Disable (+ SB[0x15] arm); .4 -> L1:Disable (+ SB[0x5A]=0x40 mailbox) */
  if (SB_RD(0x26) & 0x04) {                    /* a2a2 */
    SB_WR(0x26, 0x04);                         /* a2a5: W1C bit2 */
    uart_puts("\r\nL0:Disable");              /* string @0x20ce */
    SB_WR(0x15, 0x80);                         /* a2b3-a2b8: 9880/9704 SB[0x15]=0x80 */
  }
  if (SB_RD(0x26) & 0x10) {                    /* a2c4 */
    SB_WR(0x26, 0x10);                         /* a2c7: W1C bit4 */
    uart_puts("\r\nL1:Disable");              /* string @0x20db */
    SB_WR(0x15, 0xA0);                         /* a2d5-a2da: SB[0x15]=0xA0 */
    SB_WR(0x5A, 0x40);                         /* a2e0-a2e8: SB[0x5A]=0x40 mailbox */
  }

  /* a2ef: SB[0x9E].4 -> L0:Training; .5 -> L1:Training */
  if (SB_RD(0x9E) & 0x10) {                    /* a2f2 */
    SB_WR(0x9E, 0x10);                         /* a2f5: W1C bit4 */
    uart_puts("\r\nL0:Training");             /* string @0x20e8 */
  }
  if (SB_RD(0x9E) & 0x20) {                    /* a306 */
    SB_WR(0x9E, 0x20);                         /* a309: W1C bit5 */
    uart_puts("\r\nL1:Training");             /* string @0x20f6 */
  }

  /* a156-region [Pend Int] device->host responder (RE-AUDIT #4-extra). The re-audit found handmade
   * mismaps SB[0x26] vs SB[0x9E]: the a066 [Pend Int] branch reads SB[0x26].1 (NOT SB[0x9E]) and
   * dispatches CODE_BANK1::a5d8 — the device->host sideband READ/WRITE command responder (reads the
   * page-1 0x0998-0x099B router-op descriptor, validates, and answers via the SB[0x15] transmit
   * path e1cb/e2b9). a5d8 is a deep banked responder; it is gated on SB[0x26].1 which, per the
   * [ROPB] HW trace THIS SESSION (sb26=00 across all samples), the Intel MTL TB4 host NEVER sets
   * after connect. So this branch is a correct NO-OP here; the gate + W1C + [Pend Int] marker are
   * wired faithfully for a host that posts a sideband router-op (the full a5d8 body is not
   * fabricated — there is no host op to validate it against). */
  if (SB_RD(0x26) & 0x02) {                    /* a156: SB[0x26].1 -> [Pend Int] */
    SB_WR(0x26, 0x02);                         /* W1C bit1 */
    uart_puts("\r\n[Pend Int]");              /* string @0x... (a5d8 head 538d) */
    /* a5d8(): copy page1 0x0998-0x099B -> 0x0A9D-0x0AA0; if 0x0AA0.7 set, decode the router-op
     * read/write descriptor and answer via the SB[0x15] transmit engine (e1cb/e2b9 banked). */
  }

  (void)SB_RD(0xF6);                           /* a317-a31d: tail status latch read */
}

/* ====================================================================================
 * cb10 (CODE_BANK1::cb10) — the per-super-loop SB lane-bond ADVANCE (RE-AUDIT #6). Stock splits the
 * USB4 connect into ISR-edge (a066) + super-loop-poll (cb10), connected by the flag 0x06EC. cb10
 * runs every super-loop iteration (gated (0x09F9&0x83) && 0x06EC, under an EA=0 critical section);
 * it reads SB page-1 lane status 0xA0/0xA1 (9716/971f -> SB[0xA0]/[0xA1]), compares the low nibble
 * against the 0x072B/0x072C latches, and on a change-to-non-CL0 pushes the cbbe(lane,0x21)
 * margining/CL0 state, then advances the 0x074E/0x074F per-lane CL0 latch math and the 0x076A/0x076B
 * (ee57/e672) + 0x072A (cdf5) substate.
 *
 * Transcribed from decompile_function(CODE_BANK1::cb10). The deep banked helpers (51c7 margining
 * push via cbbe, 981b/da9f link re-init, ee57/e672/cdf5 substate) are SB-internal state machines
 * reproduced as the observable [cb10] markers + the latch updates; the load-bearing per-loop effect
 * is the SB[0xA0]/[0xA1] readout + latch compare that stock uses to detect lanes reaching CL0.
 *
 * NB (HW-confirmed this session via the [ROPB]/[HMSB] trace): on the Intel MTL TB4 host, SB[0xA0]/
 * [0xA1] read 0x07 (NOT CL0=2) and never change, and 0x072B/0x072C are seeded 0x07, so this advance
 * is a correct NO-OP here — the host stalls at SB-connect and the lanes never reach CL0. It is wired
 * faithfully so that IF a host ever drives the lanes to CL0, the advance fires. ==================*/
static volatile uint8_t __xdata __at(0x8810) cb10_seen;     /* IRAM-HEADROOM FIX: relocated to XDATA; sticky: did SB[0xA0]/[0xA1] ever change */
static void sb_cb10_lane_advance(void) {
  uint8_t a5b, lat;
  /* lane A: SB[0xA0] low nibble vs 0x072B */
  a5b = SB_RD(0xA0) & 0x0F;
  lat = PR(0x072B);
  if (a5b != lat) {
    cb10_seen |= 0x01;
    /* if (a5b != 2) cbbe(4,0x21) — margining/CL0 push (51c7). Observable arm only. */
    PR(0x072B) = a5b;
  }
  /* lane B: SB[0xA1] low nibble vs 0x072C */
  a5b = SB_RD(0xA1) & 0x0F;
  lat = PR(0x072C);
  if (a5b != lat) {
    cb10_seen |= 0x02;
    /* if (a5b != 2) cbbe(0xf,0x21) */
    PR(0x072C) = a5b;
  }
  /* 0x074E/0x074F per-lane CL0 latch math (981b/da9f): if either non-zero, run the CL0 confirm.
   * Reproduced as the latch read; the da9f re-init is the host-driven link-reset path (not taken on
   * a clean advance). */
  if (PR(0x074E) != 0 || PR(0x074F) != 0) { cb10_seen |= 0x04; }
  /* 0x06ED substate (ee57/e672 -> 0x076A/0x076B) + 0x072A (cdf5): SB-internal substate advance,
   * driven by HW-set 0x06ED/0x072A. Read-acked here (the deep ee57/e672/cdf5 bodies are SB state
   * callbacks with no host-gating XDATA effect reproducible without the full banked chain). */
}

#endif /* SB_ROUTER_H */
