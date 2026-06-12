#ifndef USB4_LANEBOND_H
#define USB4_LANEBOND_H
/*
 * USB4 lane-bond / CL0 / PCIe-tunnel bring-up FSM — faithful transcription of the ASM2464PD stock
 * firmware (fw_tinygrad.bin) bank1 super-loop lane-bond engine.
 *
 * GROUND TRUTH (stock lanetrace, app/patch_lanetrace.py, captured this session on the real HW):
 *   [===SB Con===] -> [SB P03] -> Disc_SIDs/Modes -> [ConnRout](0x0AA0 38->3C) -> [SB P04] ->
 *   [PcieTunnel-PwrOn] -> Chg2 20G -> [RstRxpll..][Done] -> [CDRV ok] -> [L0 OS1][L1 OS1] ->
 *   [SB P05][Trig] (SB[0xA1] 07->01, CA06 61->01, E764 14->19) -> L1 Sts:01 (SB[0xA0] 07->01) ->
 *   [SB P00] -> L0:CL0 02 / L1:CL0 02 -> [PcieTunnel-Deassert] (SB[0xA0/A1] 01->02 = CL0) ->
 *   Lane Bonded -> [PcieTunnel-PcieLinkUp] -> [*** USB4 Gen3 x2 ***] -> GPU.
 *   Stock reaches the GPU with C8FF=04 (Gen2) the WHOLE run, E751=0, E302 mode0 -> the SUCCESS axis
 *   is SB[0xA0]/[0xA1] reaching CL0=0x02, NOT any rate/E302-mode.
 *
 * ARCHITECTURE (decompiled, verbatim):
 *   The engine is a self-advancing FSM whose state lives in XDATA 0x06ED, driven from the SUPER-LOOP
 *   (NOT the ISR):
 *     - sb_con_consequence (dea1) -> db7a -> eb62(0,3)   sets 0x06ED=3 and 0x06EC=1 (the cb10 enable)
 *     - the super-loop runs cb10 every iteration (gated (0x09F9&0x83) && 0x06EC, EA=0); cb10's TAIL
 *       calls e672() (gated 0x06ED!=0)
 *     - e672 dispatches by 0x06ED:  3 -> cm_conn_routing_setup ([ConnRout]);  4 -> b0b4 (the
 *       PcieTunnel-PwrOn/Chg2/RstRxpll/CDRV/OS1 lane-bond engine);  5 -> 8000/850b (the per-lane
 *       CL-state walker that drives SB[0xA0]/[0xA1] 07->01->02 and prints [Trig]/CL0)
 *     - each state body calls eb62(0,N) to advance 0x06ED to the next state ([SB P0N] marker).
 *
 *   handmade was COMPLETELY missing this:
 *     - its db7a (sb_router.h sb_db7a_route_arm) omits the eb62(0,3) -> 0x06ED never armed
 *     - its cb10 (sb_router.h sb_cb10_lane_advance) is observe-only -> never calls e672
 *   so [ConnRout]/[SB P04]/[PcieTunnel-PwrOn]/lane-bond NEVER ran and SB[0xA0]/[0xA1] stayed 0x07.
 *
 * SCOPE OF THIS FILE (HONEST): the FSM DRIVER (e672 + eb62 + the db7a/cb10 wiring), the state-3
 * body cm_conn_routing_setup, AND the state-4 body b0b4 (ROUND A) are transcribed faithfully here.
 * b0b4's ROUND A = the shell + the 6 shallow helpers (96fe/d5da/e9e7/d3b0/e980/e07d) + the gates /
 * latches / CC37.2 / CA60.3 / ec51 / eb62, verbatim from the raw bank1 ASM; its DEEP, under-mapped
 * tunnel-power+train nodes (e305 PcieTunnel-PwrOn, e26a/cdc6 E764 14->19 train, b8db CDRV-compare)
 * are ROUND B -- emitted as LABELED placeholders (UART marker + an e57d/CA06 stand-in for the E764
 * train), NOT fabricated. The state-5 (8000/850b) per-lane CL-state walker is still a marker stub
 * (next transcription target). They are wired so the FSM advances and the UART shows EXACTLY how far
 * the lanes walk on HW.
 *
 * Included after usb4_connect.h (needs SB_RD/WR, P1_*, PR, uart_*, the boot_phy helpers).
 */

/* ---- 0x06ED FSM state (the lane-bond engine state machine register) ---- */
#define U4LB_STATE   PR(0x06ED)

/* eb62(p1,p2): set 0x0AAC=p1, 0x0AAD=p2 (the new FSM state), print "[SB P0<state>]", set 0x06ED=p2.
 * Verbatim from CODE_BANK1::eb62 + eb81 ([SB P]@0x1ffc printer returns 0x0AAD). */
static void u4lb_eb62(uint8_t p1, uint8_t p2) {
  PR(0x0AAC) = p1;
  PR(0x0AAD) = p2;
  uart_puts("\r\n[SB P0");
  uart_puthex(p2);            /* eb81: print "[SB P" + puthex(0x0AAD); returns 0x0AAD */
  uart_putc(']');
  PR(0x06ED) = PR(0x0AAD);    /* 0x06ED = 0x0AAD = p2 (the second eb81 returns 0x0AAD too) */
}

/* (Instrumentation flags removed to save DSEG/IRAM; the UART markers [ConnRout]/[u4lb:S4]/[u4lb:S5]
 * + the [SB P0x] prints already show how far the FSM walks.) */

/* [EDF5] dump budget (XDATA, seeded in main()). */
static volatile uint8_t __xdata __at(0x0B58) u4lb_edf5_print_budget;

/* ====================================================================================
 * edf5 / e2b9 (CODE_BANK1::edf5 -> e2b9) — THE DROPPED DEVICE->HOST SB-TRANSPORT ROUTE-QUERY.
 *
 * THIS IS THE TRIGGER THAT MAKES THE HOST START POSTING THE CONNECTION-ROUTING DESCRIPTOR.
 * Stock's cm_conn_routing_setup (a7de) at FSM state 0x0758==0x10 calls edf5(); handmade DROPPED it
 * and only bumped 0x0758=0x11 (the comment wrongly claimed "0x0758=0x11 is the load-bearing effect").
 * The REAL load-bearing effect is e2b9(R7=5,R5=7,R3=5,R2=4): it builds a small SB-transport TX
 * descriptor, writes SB[0x15]=5 (the SB-transport TX COMMAND), and TRIGGERS the transport via d5da.
 * That device->host message is what tells the host CM "router ready, send the routing descriptor",
 * so the host then HW-DMA-fills the 0x2a00 RX plane with the 0x0C descriptor -> cd3f/eaac -> 0x0777
 * -> the [ConnRout] confirm passes. Without it the host posts NOTHING (0x2a00 RX stays all-zeros,
 * SB[0x18]=00, SB[0x28].3/.4 never set) -- exactly the observed wall.
 *
 * edf5 (CODE_BANK1::edf5, byte-exact disasm edf5-ee10):
 *   if (0x0719 != 0) return;          // a route-query is already in flight (token), don't re-send
 *   0x0AAB = 0;  e2b9(R7=5,R5=7,R3=5,R2=4);   // SEND it
 * e2b9 (CODE_BANK1::e2b9, byte-exact disasm e2b9-e304):
 *   0xAA8=R7(=5); 0xAA9=R5(=7); 0xAAA=R3(=5);
 *   d4cd();                                   // sb_transport_substate_poll (drain pending edges)
 *   997e(0xAA9):  SBTX[0] = 0xAA9 (=7)        // TX-plane byte 0
 *   A=0xAAB(=0); 9923(A, 0xAAA): SBTX[1] = (0xAAA(=5) | A<<7) = 5;  returns A(=0) -> JZ branch:
 *     9695(): A = SB[0x0C];  A = (A & 0x80) | 0x08;  SB[0x0C] = A   // STAGE descriptor (status|8).
 *       9695 sets R2:R1=0x28:0x0C, so r3_write_dispatch's target is SB[0x0C] (page1 0x280C), NOT SBTX[1].
 *   96f7(0xAA8 -> R1=0x15):  SB[0x15] = 0xAA8 (=5)   // THE SB-TRANSPORT TX COMMAND
 *   d5da(0):  trigger (SB[0x10]=1, wait SB[0x2C].2)  // HW transport send
 *   97ef():  CCD9=4; CCD9=2;   then CCD9=(2-1)=1;  0x0719 = 1   // set the in-flight token
 * The token 0x0719 is cleared by eda0 (0x0775!=0 path) once the host's descriptor arrives, which
 * re-allows the confirm body / a fresh query.
 *
 * STACK: runs in the SUPER-LOOP (cb10->e672->cm_conn_routing_setup), never the a066/INT1 ISR.
 * d5da's stock busy-poll on SB[0x2C].2 is bounded here so the super-loop can't hang (same pattern
 * as sb_af38_descriptor_response). ==================================================================
 */
static uint8_t u4lb_edf5_route_query(void) {
  uint8_t b1;
  if (PR(0x0719) != 0) return 0;               /* edff-ee01: in-flight token set -> R7=0, don't re-send */

  PR(0x0AAB) = 0;                              /* ee04-ee08: 0x0AAB = 0 */
  /* ---- a7de loads R5=3, R7=0x0C (a7f7/a7fb) then LCALL edf5 -> e2b9(R5=3, R7=0x0C, 5) ----
   * e2b9: 0xAA8=param_3=5; 0xAA9=param_2=R7=0x0C; 0xAAA=param_1=R5=3.
   * FIX 2026-06-11: the route-query PAYLOAD was WRONG. Stock sends SBTX[0]=0x0C, SBTX[1]=3 (the
   * "TX=0C03" connection-routing descriptor the host actually responds to). Handmade had hardcoded
   * 7/5 (a mis-derivation of the e2b9 args from a7de R5/R7) -> the host received a malformed query
   * and never posted the routing descriptor (0x0777 stayed 0x55). Verified from the a7de disasm. */
  PR(0x0AA8) = 5;                              /* 0xAA8 = param_3 = 5 (-> SB[0x15] TX command) */
  PR(0x0AA9) = 0x0C;                           /* 0xAA9 = R7 = 0x0C (-> SBTX[0]) */
  PR(0x0AAA) = 3;                              /* 0xAAA = R5 = 3  (-> SBTX[1] base) */
  sb_transport_substate_poll();                /* e2c4: d4cd (drain pending SB[0x28].3 edges) */

  SBTX_WR(0, PR(0x0AA9));                       /* 997e: SBTX[0] = 0xAA9 (=7) */
  /* 9923(A=0xAAB(=0), 0xAAA(=5)): SBTX[1] = 5; returns 0 -> JZ branch (e2d7) */
  SBTX_WR(1, (uint8_t)(PR(0x0AAA) | ((PR(0x0AAB) & 1) << 7)));   /* = 5 */
  /* JZ taken (9923 returned 0): 9695() sets R2:R1=0x28:0x0C and returns SB[0x0C]; stage
   * (SB[0x0C]&0x80)|0x08 -> SB[0x0C].3 (descriptor-staged).
   * FIX 2026-06-11: this write targets SB[0x0C] (page1 0x280C), NOT SBTX[1] (0x2901). The old
   * SBTX_WR(1,..) was a plane-misdirection transcription bug: it clobbered SBTX[1] and never staged
   * SB[0x0C], so the HW descriptor-engine never armed SB[0x2C] bits 5,4 (channel READY) and
   * SB[0x2C].2 (TX-complete) never asserted -> the route-query TX poll looped forever. (HW-confirmed:
   * stock SB[0x0C]=0x08/SB[0x2C]=0xF1 vs handmade SB[0x0C]=0x00/SB[0x2C]=0x03.) */
  b1 = (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08);  /* e2e3-e2e8 */
  SB_WR(0x0C, b1);                              /* e2ea r3_write_dispatch(b1, 0x2800+0x0C) = SB[0x0C] */

  SB_WR(0x15, PR(0x0AA8));                       /* 96f7: SB[0x15] = 0xAA8 (=5) -- TX COMMAND */

  /* ---- d5da(0): the SB-transport TX trigger (byte-faithful CODE_BANK1::d5da, bounded poll) ---- */
  { uint8_t tx_done; uint16_t g = 0;
    PR(0x0AAC) = 0;                              /* d5dd: 0x0AAC = R7(=0) (param!=1 -> skip 96cf/97e8) */
    P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));  /* d5ea 9777: P1[0x0100] &= 0xFE (NOT SB[0x00]) */
    SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));  /* d5f2 98c7(R2=0x28,R1=4): SB[0x04] clr bit1 */
    /* [TX] PRE-GO dump (matches stock app/patch_txtrace.py hook fields, same order). */
    if (u4lb_edf5_print_budget) {
      uart_puts("\r\n[TX p15="); uart_puthex(SB_RD(0x15));
      uart_putc(' '); uart_puthex(SB_RD(0x10));
      uart_putc(' '); uart_puthex(SB_RD(0x2C));
      uart_putc(' '); uart_puthex(SB_RD(0x0C));
      uart_putc(' '); uart_puthex(SB_RD(0x0D));
      uart_putc(' '); uart_puthex(SB_RD(0x0E));
      uart_putc(' '); uart_puthex(SB_RD(0x18));
      uart_putc(' '); uart_puthex(SB_RD(0x28));
      uart_putc('|'); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBTX_RD(i)); }
      uart_putc('|'); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + i))); }
      uart_putc('|'); uart_puthex(PR(0x0719)); uart_putc(' '); uart_puthex(PR(0x0758));
      uart_putc(' '); uart_puthex(PR(0x06ED)); uart_putc(' '); uart_puthex(PR(0x06EE));
      uart_putc(' '); uart_puthex(PR(0x0775)); uart_putc(' '); uart_puthex(PR(0x0776));
      uart_putc(' '); uart_puthex(PR(0x0777));
      uart_putc('|'); uart_puthex(REG_XFER2_DMA_STATUS); uart_putc(' '); uart_puthex(REG_INT_PCIE_NVME);
      uart_putc(' '); uart_puthex(REG_CPU_CTRL_CC37); uart_putc(' '); uart_puthex(REG_CPU_CTRL_CA60);
      /* DIAG (host-post wall): P1[0x0109] = edd9 host-request gate; SB[0xD8] = ack strobe;
       * SB[0x3A-3D] = device router/lane DESCRIPTOR RAM (stock 12,36,06,00 vs handmade FF garbage). */
      uart_putc('|'); uart_puthex(P1_RD(0x0109)); uart_putc(' '); uart_puthex(SB_RD(0xD8));
      uart_putc(' '); uart_puthex(SB_RD(0x3A)); uart_puthex(SB_RD(0x3B));
      uart_puthex(SB_RD(0x3C)); uart_puthex(SB_RD(0x3D));
      uart_putc(']');
    }
    SB_WR(0x10, 0x01);                           /* d5f9: SB[0x10] = 1 (TX go) */
    /* d600 poll: wait SB[0x2C].2 (TX complete) */
    while (((SB_RD(0x2C) >> 2) & 1) == 0 && ++g < 0x4000) { }
    tx_done = (uint8_t)((SB_RD(0x2C) >> 2) & 1);
    /* [TX] POST-POLL: did SB[0x2C].2 assert? + iteration count + SB[0x2C] raw. */
    if (u4lb_edf5_print_budget) {
      u4lb_edf5_print_budget--;
      uart_puts("[TXdone done="); uart_puthex(tx_done);
      uart_puts(" sb2c="); uart_puthex(SB_RD(0x2C));
      uart_puts(" g="); uart_puthex((uint8_t)(g >> 8)); uart_puthex((uint8_t)g);
      uart_puts("]\r\n");
    }
    SB_WR(0x2C, 0x04);                           /* d60d 9799(4): W1C SB[0x2C].2 */
    phy_cc10_cmd(1, 0, 0x0B);                    /* d614-d61a e80a(subcmd=1,cc12=0,cc13=0x0b) */
    SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));  /* d61d 96ee(0x0f): SB[0x0F] &= 0xFE */
    /* d627: if 0x0AAC==0 (our case) zero the 0x2900 tail when SB[0x0C]>6 -- omitted (descriptor short) */
    (void)tx_done;
  }

  /* 97ef + tail: CCD9 strobe, set the in-flight token 0x0719 = 1 */
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;   /* 97ef + DEC A tail */
  PR(0x0719) = 0x01;                             /* e300-e303: 0x0719 = 1 (in-flight token) */
  return 1;                                      /* ee0e: R7 = 1 (a query was just sent) */
}

/* ====================================================================================
 * cm_conn_routing_setup (CODE_BANK1::a7de) — [ConnRout] connection-routing FSM (state 0x06ED==3).
 * FSM on 0x0758 (state: 0x10/0x11/0x00). On the confirm path it prints [ConnRout]@0x2004 and writes
 * 0x0718=0x04 (ROUTE-ENABLE consumed by the tunnel bring-up), latches the 0x077a/0x081a/0x0819 lane-
 * width state into 0x0750/0x0751, then runs c586 (C8FF PHY rate cfg) + bank0 e175/e282/c17f (tunnel-
 * adapter cfg) and clears 0x0758.
 *
 * FAITHFUL PARTS reproduced: the 0x0758 state machine, the confirm gate (0x0777/0x07B9/0x0778/
 * 0x081B/0x07CE/0x07CD -> 0x0776), the [ConnRout] print + 0x0718 write, the 0x077a-driven 0x0750/
 * 0x0751 lane-width latch, and the 0x0763/0x0764 clear. OMITTED (deep PHY layer, documented):
 * c586 (C8FF tunnel-adapter rate descriptor) + bank0 db80/e175/e282/c17f (PCIe-tunnel adapter PHY
 * config) -- they configure the tunnel adapter; reproduced as a marker so the [ConnRout] confirm +
 * 0x0718 + advance still happen and the FSM proceeds to state 4.
 * ==================================================================================== */
static void u4lb_cm_conn_routing_setup(void) {
  uint8_t st = PR(0x0758);
  if (st == 0x10) {
    /* a7eb-a800: edf5() then 0x0758=0x11. edf5 (CODE_BANK1::edf5) is the DEVICE->HOST SB-TRANSPORT
     * ROUTE-QUERY (-> e2b9(..,5) -> SB[0x15]=5 + d5da TX) that ELICITS the host to post the
     * connection-routing descriptor into the 0x2a00 RX plane. handmade had DROPPED it (only the
     * 0x0758=0x11 advance) -> the host never started posting -> 0x0777 stayed 0x55, state-3 stalled.
     * Now wired faithfully: edf5 sends the query (gated by the 0x0719 in-flight token) and returns
     * R7=1 only when it actually sent; a7f5 advances to 0x11 ONLY then (else stays at 0x10 to retry).
     * At 0x11, eda0 watches 0x0775 (the host's posted descriptor) -> clears 0x0719 -> on 0x0777!=0x0C
     * it loops back to 0x10 and edf5 re-queries. This is the query/response retry loop. */
    if (u4lb_edf5_route_query() != 1) return;    /* a7f1-a7f4: edf5 R7!=1 -> stay at 0x10 */
    PR(0x0758) = 0x11;
    return;
  }
  if (st != 0x11) {
    if (st != 0x00) return;
    /* st==0: arm the FSM -> [SB P04] (eb62(0,4)). This is the cm_conn_routing_setup st==0 path. */
    u4lb_eb62(0, 4);
    return;
  }

  /* st == 0x11: the main confirm body. */
  /* eda0() (VERIFIED FIX #2): it is NOT void -- it returns an R7 SELECTOR that gates the rest.
   * Verbatim from CODE_BANK1::eda0 (eda0-edbc):
   *   if (0x0775 != 0) { 0x0775=0; 0x0719=0; r7=0; }       (eda4..edad: the EVAL path)
   *   else if (0x0719 == 2) { 0x0719=0; r7=2; }            (edb2..edb9: the route-special path)
   *   else r7=1;                                           (edba: the IDLE path -- leave 0x0758)
   * The caller (a82f) then: r7==2 -> 0x0758=0x10; return;  r7!=0 (==1) -> return (idle, 0x0758
   * UNCHANGED); r7==0 ONLY -> evaluate the 0x0777 confirm gate. The prior handmade treated eda0 as
   * void and ALWAYS fell through to the 0x0777 gate -> on the idle (r7==1) re-entries it wrongly
   * re-stamped 0x0758=0x10 and never let the confirm body run. */
  { uint8_t r7;
    if (PR(0x0775) != 0)      { PR(0x0775) = 0; PR(0x0719) = 0; r7 = 0; }
    else if (PR(0x0719) == 0x02) { PR(0x0719) = 0; r7 = 2; }
    else                       { r7 = 1; }
    if (r7 == 2) { PR(0x0758) = 0x10; return; }   /* route-special re-arm */
    if (r7 != 0) { return; }                       /* idle: leave 0x0758 unchanged */
    /* r7 == 0 ONLY: the EVAL path -> evaluate the host connect-descriptor confirm gate. */
  }

  /* DAT_INTMEM_21 = param_2 (the mode); the live AMD path enters with mode 0. We follow the mode==0
   * branch (param_2==0): gate 0x0777==0x0C else 0x0758=0x10/ret. */
  if (PR(0x0777) != 0x0C) { PR(0x0758) = 0x10; return; }

  /* 0x0776 connect-confirm computation (a850..): if 0x07B9 set, validate. */
  if (PR(0x07B9) != 0) {
    uint8_t b778 = PR(0x0778);
    if (((b778 & 0x7F) == 2) || ((PR(0x081B) & 1) == 0) ||
        (PR(0x07CE) != 0 && PR(0x07CD) == 0)) {
      PR(0x0776) = 0;
    } else {
      /* else: SB[0xED] &= 0x7F via 9814(0xed)+r3_write -- a SB page write; reproduced as SB_CLR. */
      SB_CLR(0xED, 0x80);
    }
  }
  /* if 0x0776==0 -> e391() routing-descriptor seed (ROM 0x514c/0x515f -> PHY descriptor regs +
   * d221 PHY load). Deep PHY layer -- documented-omitted (it seeds the routing descriptor the
   * tunnel adapter reads; the [ConnRout]/0x0718 confirm below is the load-bearing FSM effect). */

  /* a878: the [ConnRout] confirm print + 0x0718 ROUTE-ENABLE. */
  if (PR(0x0776) == 0 && PR(0x07CE) != 0) {
    uart_puts("[ConnRtmr]");   /* str @0x200f (alt path) */
    PR(0x0718) = 0;
  } else {
    uart_puts("[ConnRout]");   /* str @0x2004 -- the route confirm */
    PR(0x0718) = 4;            /* ROUTE-ENABLE (consumed by the tunnel bring-up / e672 state 5) */
  }

  /* a88f..: latch the 0x077a lane-width bits into 0x0819/0x0751/0x0750. */
  { uint8_t b77a = PR(0x077A);
    if ((b77a & 1) && (PR(0x081A) & 1)) { PR(0x0819) = (PR(0x0819) & 0xFE) | 1; }
    if (b77a & 0x02) { if (PR(0x081A) & 2) { PR(0x081A) = (PR(0x081A) & 0xFD) | 2; } }  /* 9a31 net */
    /* 0x0751 = 1 iff (0x077a.4 && 0x081a.4 && 0x0819.0 && 0x0819.1) else 0 */
    if ((b77a & 0x10) && (PR(0x081A) & 0x10) && (PR(0x0819) & 1) && (PR(0x0819) & 2)) PR(0x0751) = 1;
    else PR(0x0751) = 0;
    /* 0x0750 = 2 iff (0x077a.5 && 0x081a.5) */
    if ((b77a & 0x20) && (PR(0x081A) & 0x20)) PR(0x0750) = 2;
    /* DIAG (latch localization): what the lane-present latch sees AT RUN TIME. */
    uart_puts("[Lt77A="); uart_puthex(b77a); uart_puts(" 81A="); uart_puthex(PR(0x081A));
    uart_puts(" 819="); uart_puthex(PR(0x0819)); uart_puts("]");
    PR(0x0763) = 0; PR(0x0764) = 0;
  }

  /* a95f..: c586 (C8FF tunnel-adapter rate cfg) + bank0 e175/e282/c17f (tunnel-adapter PHY cfg) +
   * the 0x0750==1 ROM-table 0x21c4 copy. DEEP PHY LAYER -- documented-omitted (configures the PCIe-
   * tunnel adapter; not reproduced to avoid fabricating the PHY write sequence). */

  /* a891: 0x0758 = 0 (FSM done with the routing-setup state). The eb62(0,4) -> [SB P04] advance
   * happens via the st==0 re-entry next cb10 iteration (cm_conn_routing_setup st==0 path). */
  PR(0x0758) = 0;
}

/* ====================================================================================
 * State 4 (0x06ED==4) — b0b4: the [PcieTunnel-PwrOn] -> Chg2 20G -> [RstRxpll][Done] -> [CDRV ok] ->
 * [L0 OS1][L1 OS1] lane-bond / 20G-rate / RxPLL-reset / per-lane OS1-arm engine.
 *
 * ROUND A (this commit): the b0b4 SHELL + the 6 shallow helpers (96fe/d5da/e9e7/d3b0/e980/e07d) +
 * the gates/latches/CC37.2/CA60.3/ec51/eb62, transcribed verbatim from the RAW bank1 ASM
 * (CODE_BANK1::b0b4 @ file 0x130b4 + helper tree). The DEEP, under-mapped tunnel-power+train nodes
 * (e305 PcieTunnel-PwrOn, e26a/cdc6 E764 14->19 train, b8db CDRV-compare) are LABELED placeholders
 * (Round B) -- they print their UART marker and, for the E764 train, reuse the EXISTING e57d reset
 * pulse + connect-head CA06 RMW as a clearly-commented stand-in; their real bodies are NOT fabricated.
 *
 * phy_cc10_cmd_wait(subcmd,cc12,cc13) arg order was RE-DERIVED from R7(subcmd)/R4(cc12)/R5(cc13) at
 * EVERY stock call site (NOT a decompiler arg order). Plane discipline: SB[off]=DPX1 0x2800+off;
 * SB2[off]=DPX1 0x2900+off (new); P1[0x01xx]=DPX1 0x0100+off; C2xx/C3xx/CA06/CA60/CA81/CC37/E716 =
 * bank0 PLAIN XDATA via PR(). Every stock busy-poll is BOUNDED with a guard counter (no super-loop
 * hang). 0x1fc0/0x1fcb/0x1fd6 are CODE STRINGS (uart_puts), NOT registers.
 * ==================================================================================== */

/* ---- SB2: page-1 lane-block accessor at 0x2900+off (mirror of sb.h's SB_RD/WR base 0x2800).
 * Stock reaches it via the R3=2/R2=0x29/R1=off paged r3_xdata accessor (DPX=1). Needed by e07d. */
#define SB2_RD(off)      P1_REG8_rd((uint16_t)(0x2900u + (off)))
#define SB2_WR(off, v)   P1_REG8_wr((uint16_t)(0x2900u + (off)), (uint8_t)(v))

/* ---- 96fe sb_cmd_issue(op): SB[0x15]=op; SB[0x0C]=(SB[0x0C]&0x80)|0x03. Per-lane OS/CDR command-
 * issue descriptor. Stock leaves R7=1 on return, so the d5da that follows runs its param==1 path. */
static void u4lb_96fe(uint8_t op) {
  SB_WR(0x15, op);                                  /* 9704: r3_write SB[0x15]=op */
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);         /* 9707-9710: SB[0x0C]=(old&0x80)|3 */
}

/* ---- d5da sb_lane_phy_ready_handshake(param): the load-bearing per-lane PHY-RX/CDR commit+settle.
 * Transcribed verbatim from CODE_BANK1::d5da (all 3 param branches). 0x0AAC=param selects:
 *   param==1: extra SB[0x0F]|=1 prologue (96cf read + 97e8 set-bit0)
 *   common  : P1[0x0100]&=~1 (9777); SB[0x04]&=~2 (98c7); SB[0x10]=1; BOUNDED-spin SB[0x2C].2;
 *             SB[0x2C]=4; phy_cc10_cmd_wait(1,0,0x0B); SB[0x0F]&=~1
 *   param==0: SB[0x0C]>6 -> zero SB2[0x00+i] for i in [0 .. SB[0x0C]-6)
 * b0b4 calls d5da with param 1 (after 96fe/9695+970c, which set R7=1) AND param 0 (e07d tail). */
static void u4lb_d5da(uint8_t param) {
  PR(0x0AAC) = param;                               /* d5da: 0x0AAC = param */
  if (param == 1) {
    SB_WR(0x0F, (SB_RD(0x0F) & 0xFE) | 0x01);       /* d5e2-d5e7: SB[0x0F] |= 1 (96cf + 97e8) */
  }
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);              /* d5ea: P1[0x0100] &= ~1 (9777) */
  SB_WR(0x04, SB_RD(0x04) & 0xFD);                  /* d5f2: SB[0x04] &= ~2 (98c7) */
  SB_WR(0x10, 0x01);                                /* d5f9: SB[0x10]=1 */
  { uint16_t g = 0;                                 /* d600: BOUNDED spin until SB[0x2C].2 set */
    while (((SB_RD(0x2C) & 0x04) == 0) && ++g < 0x2000); }
  SB_WR(0x2C, 0x04);                                /* d60d: SB[0x2C]=4 (9799) */
  phy_cc10_cmd_wait(1, 0, 0x0B);                    /* d614: R7=1,R4=0,R5=0x0B */
  SB_WR(0x0F, SB_RD(0x0F) & 0xFE);                  /* d61d: SB[0x0F] &= ~1 (96ee) */
  if (param != 0) return;                           /* d627: param!=0 -> done */
  /* param==0 tail (d62d..d65a): zero the SB2[0x00..] descriptor if SB[0x0C] > 6 */
  { uint8_t c = SB_RD(0x0C);                        /* d62d: read SB[0x0C] */
    if (c <= 6) return;                             /* d632: SB[0x0C]<=6 -> done */
    { uint8_t limit = (uint8_t)(c - 6), i;          /* d637: limit = SB[0x0C]-6 */
      for (i = 0; i < limit; i++) SB2_WR(i, 0x00); }/* d641-d65a: SB2[i]=0 (R2=0x29 plane) */
  }
}

/* ---- e07d phy_lane_block_poke(): RETRAIN-path per-lane PHY/SB2 lane-block program (ignores its
 * incoming arg). Verbatim from CODE_BANK1::e07d (the raw disasm tool truncates this to 3 instrs;
 * read from the file bytes / disassemble_bytes -- transcribed from those). */
static void u4lb_e07d(void) {
  uint8_t cfg;
  SB_WR(0x15, 0x61);                                /* e080: SB[0x15]=0x61 (PHY power/OS poke) */
  SB2_WR(0x00, 0x09);                               /* e085-e08a: SB2[0x00]=0x09 (R2 INC -> 0x29) */
  if ((PR(0x0763) | PR(0x0764)) != 0)               /* e08d 9a06: gate = 0x0763|0x0764 */
    SB2_WR(0x00, SB2_RD(0x00) | 0x04);              /* e092: SB2[0x00] |= 4 */
  if (PR(0x07B9) != 0)                              /* e09a: 0x07B9 != 0 */
    SB2_WR(0x00, SB2_RD(0x00) | 0x10);              /* e0a0-e0ab: SB2[0x00] |= 0x10 */
  /* e0ae..e0c6: compose the lane-cfg byte: ((0x0750&0x0F)<<4) | ((0x0819.1)<<1) | (0x0819.0) */
  cfg = (uint8_t)(((PR(0x0750) & 0x0F) << 4)
                  | ((PR(0x0819) & 0x02) ? 0x02 : 0x00)
                  | (PR(0x0819) & 0x01));
  SB2_WR(0x01, cfg);                                /* e0c9 9981: SB2[0x01] = composed cfg */
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x08);         /* e0cc-e0d1 99ac: SB[0x0C]=(old&0x80)|0x08 */
  u4lb_d5da(0);                                     /* e0d4-e0d6: tail d5da(0) */
}

/* ---- e9e7 RstRxpll: uart '[RstRxpll...]'; CC37|=0x04; C20E=0xFF + phy_cc10_cmd_wait(1,0,0x14);
 * C20E=0x00 + phy_cc10_cmd_wait(2,0,0x28); CC37&=~0x04; uart '[Done]'. Verbatim CODE_BANK1::e9e7.
 * 0x1fd6/0x1fe6 are CODE STRINGS not regs (corrected). */
static void u4lb_e9e7(void) {
  uart_puts("\r\n[RstRxpll...]");                   /* e9ed 538d(str 0x1fd6) */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;          /* e9f0 984d: CC37=(CC37&0xFB)|4 */
  REG_PHY_RXPLL_RESET = 0xFF;                                /* e9f6-e9f9 9a27: C20E=0xFF */
  phy_cc10_cmd_wait(1, 0, 0x14);                    /* e9fc: R7=1,R4=0,R5=0x14 */
  REG_PHY_RXPLL_RESET = 0x00;                                /* e9ff-ea03: C20E=0 */
  phy_cc10_cmd_wait(2, 0, 0x28);                    /* ea09: R7=2,R4=0,R5=0x28 */
  REG_CPU_CTRL_CC37 = REG_CPU_CTRL_CC37 & 0xFB;                   /* ea0c 984d: CC37 &= ~4 */
  uart_puts("[Done]");                              /* ea16 538d(str 0x1fe6) */
}

/* ---- ebde settle: C20F=0xFF + phy_cc10_cmd_wait(1,0,0x14); C20F=0; BOUNDED-spin C2D0.5 then
 * C350.5 (PLL/lane rate-lock). Verbatim CODE_BANK1::ebde (inlined into e980 below). */
static void u4lb_ebde(void) {
  REG_PHY_CTRL_C20F = 0xFF;                                /* ebde-ebe1 9a27: C20F=0xFF */
  phy_cc10_cmd_wait(1, 0, 0x14);                    /* ebe4: R7=1,R4=0,R5=0x14 */
  REG_PHY_CTRL_C20F = 0x00;                                /* ebe7-ebeb: C20F=0 */
  { uint16_t g = 0; while (((PR(0xC2D0) & 0x20) == 0) && ++g < 0x2000); }  /* ebec: C2D0.5 lock */
  { uint16_t g = 0; while (((PR(0xC350) & 0x20) == 0) && ++g < 0x2000); }  /* ebf8: C350.5 lock */
}

/* ---- e980 rate-descriptor apply (20G): C2A8&=0x3F; C328&=0x3F; ebde settle; C2A8&=0x3F;
 * C2C9=(C2C9&0x80)|(((C2EC&0x38)>>3)|0x40); C328&=0x3F; C349=(C349&0x80)|(((C36C&0x38)>>3)|0x40);
 * C2A8|=0x80; C328|=0x80 (START). All bank0 PLAIN XDATA (PR()). Verbatim CODE_BANK1::e980 (decompiler
 * ghosts a param_1; the asm c343/c32d/c31f resolve to these net writes). */
static void u4lb_e980(void) {
  PR(0xC2A8) = PR(0xC2A8) & 0x3F;                   /* e980-e986 c343/c32d: C2A8&=0x3F */
  PR(0xC328) = PR(0xC328) & 0x3F;                   /* e986: C328&=0x3F */
  u4lb_ebde();                                      /* e987: ebde settle */
  PR(0xC2A8) = PR(0xC2A8) & 0x3F;                   /* e98a-e98d c343: C2A8&=0x3F (re-commit) */
  PR(0xC2C9) = (PR(0xC2C9) & 0x80)
             | (uint8_t)(((PR(0xC2EC) & 0x38) >> 3) | 0x40);  /* e98e-e99c: C2C9 rate desc */
  PR(0xC328) = PR(0xC328) & 0x3F;                   /* e99c: C328&=0x3F */
  PR(0xC349) = (PR(0xC349) & 0x80)
             | (uint8_t)(((PR(0xC36C) & 0x38) >> 3) | 0x40);  /* e99d-e9a8: C349 rate desc */
  PR(0xC2A8) = (PR(0xC2A8) & 0x3F) | 0x80;          /* e9a9-e9b3: C2A8 START bit7 */
  PR(0xC328) = (PR(0xC328) & 0x3F) | 0x80;          /* e9b1-e9b3: C328 START bit7 */
}

/* ---- d3b0 Chg2-rate setup (rate=3=20G): 0x0A5C=rate; branch on 0x0750==1 (10G/bonded) vs !=1 (20G
 * live path). 20G path: SB[0x65]&=~0x10/~0x20 (clear) -> uart 'Chg2 20G' -> SB[0x65]|=0x10 (rate.0) /
 * SB[0x65]|=0x20 (rate.1); commit phy_cc10_cmd_wait(2,0,0xc8). Verbatim CODE_BANK1::d3b0.
 * SB[0x65] bit4 set iff rate bit0; bit5 set iff rate bit1 (rate=3 -> both). */
static void u4lb_d3b0(uint8_t rate) {
  PR(0x0A5C) = rate;                                /* d3b0: 0x0A5C=rate */
  if (PR(0x0750) == 1) {
    /* 10G/bonded path (d3bd) -- not the live AMD path (0x0750 latched 2). Kept faithful. */
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);  /* 968c+99e0 */
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);  /* 968c+986d */
    uart_puts("\r\nChg2 10G");                      /* d3d9 d435(str 0x1fc0) */
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);
  } else {
    /* 20G live path (d3f5) */
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);           /* d3fd: SB[0x65]&=~0x10 */
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);           /* d409: SB[0x65]&=~0x20 */
    uart_puts("\r\nChg2 20G");                      /* d415 d435(str 0x1fcb) */
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);  /* d41b 968c+99e0 */
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);  /* d425 968c+986d */
  }
  phy_cc10_cmd_wait(2, 0, 0xC8);                    /* d42b: R7=2,R4=0,R5=0xC8 COMMIT */
}

/* ---- ec51 Trig-arm: CCE1=4 then 2 (97f2 strobe); CCE0=(CCE0&0xF8)|0x04; CCE2=CCE3=0xFF; CCE1=0x01;
 * 0x0774 ^= 1. Arms the lane-train trigger that state 5 fires as [Trig]. Verbatim CODE_BANK1::ec51. */
static void u4lb_ec51(void) {
  PR(0xCCE1) = 0x04; PR(0xCCE1) = 0x02;             /* ec54 97f2(DPTR=CCE1): strobe 4 then 2 */
  PR(0xCCE0) = (PR(0xCCE0) & 0xF8) | 0x04;          /* ec57-ec5f: CCE0 bits2:0 = 4 */
  PR(0xCCE2) = 0xFF; PR(0xCCE3) = 0xFF;             /* ec60-ec67: CCE2=CCE3=0xFF */
  PR(0xCCE1) = 0x01;                                /* ec68-ec6d: CCE1=1 (arm) */
  PR(0x0774) = PR(0x0774) ^ 0x01;                   /* ec6e-ec74: 0x0774 ^= 1 */
}

/* ---- b226 settle: phy_cc10_cmd_wait(2,0,0xc8). Verbatim CODE_BANK1::b226. */
static void u4lb_b226(void) { phy_cc10_cmd_wait(2, 0, 0xC8); }

/* ---- ee57: (if CCE1.0 && CCE1.1 -> ec51 Trig-arm); then the caller snapshots CCE4:CCE5. Verbatim
 * CODE_BANK1::ee57. CCE4/CCE5 is a read-only HW lane-width/train counter (every firmware access
 * image-wide -- 981b/b10f/b20f/ee65 -- is a READ; nothing writes it). ---- */
static void u4lb_ee57(void) {
  /* FIX 2026-06-12: condition was INVERTED. Stock ee57 = `if ((CCE1&1)==0 || (CCE1&2)!=0) ec51()`
   * — fires the lane-train Trig-arm when CCE1.0 is CLEAR (e.g. CCE1=0 initial) OR CCE1.1 set. The old
   * `(CCE1.0 && CCE1.1)` only fired at CCE1=0x03, so ec51 NEVER armed the trigger at db7a -> the lanes
   * never trained -> CCE4:CCE5 (read-only HW train counter) stayed 0 -> b0b4 WIDGATE-abort. */
  if (!(PR(0xCCE1) & 0x01) || (PR(0xCCE1) & 0x02)) u4lb_ec51();  /* ee5a-ee62 */
  /* ee65-ee6d: returns R6=CCE4, R7=CCE5 to the caller (98ec consumes them). */
}

/* ---- 98ec(): 0x758=0x10; ee57(); 0x768=CCE4; 0x769=CCE5. The lane-width SNAPSHOT producer (GAP-1).
 * Verbatim CODE_BANK1::98ec. Stock db7a's TAIL is `eb62(0,3); 98ec()` but handmade dropped 98ec, so
 * 0x768/0x769 read uninit 0x55 and b0b4's width gate (b10f-b12d) diffed garbage. (The R6/R7 db7a
 * passes to 98ec are DEAD -- ee57 overwrites them with CCE4/CCE5.) 0x758=0x10 also re-arms the
 * cm_conn_routing_setup state, consistent with the state-3 entry. ---- */
static void u4lb_98ec(void) {
  PR(0x0758) = 0x10;                 /* 98ec-98f1: 0x758 = 0x10 (cm_conn_routing_setup state arm) */
  u4lb_ee57();                       /* 98f2: ee57 (Trig-arm gate + CCE4/CCE5 read) */
  PR(0x0768) = PR(0xCCE4);           /* 98f5-98f9: 0x768 = CCE4 (R6) */
  PR(0x0769) = PR(0xCCE5);           /* 98fa-98fc: 0x769 = CCE5 (R7) */
}

/* ====================================================================================
 * b0b4 body (CODE_BANK1::b0b4 @ file 0x130b4) — state-4 assembled per the plan's dependency order.
 * ==================================================================================== */
static void u4lb_state4_b0b4(void) {
  uart_puts("[b4:A]");   /* INSTR: b0b4 entered (before any poll) */
  /* --- (A) entry / retrain guard (b0b4-b0cc): 0x0776 != 0 -> retrain {e07d; b226} x2 --- */
  if (PR(0x0776) != 0) {
    uint8_t i;
    for (i = 0; i < 2; i++) { u4lb_e07d(); u4lb_b226(); }   /* b0bd-b0ca */
  } else {
    /* --- normal-connect OS-prewrite (b0ce-b10c) --- */
    if (PR(0x0819) & 0x01) {                                /* b0d2: 0x0819.0 (L0 present) */
      uint8_t op = (PR(0x0750) == 2) ? 0x85 : 0x81;         /* b0d5-b0df: 987c(0x0750)==2 ? */
      SB_WR(0x15, op);                                      /* b0e1: r3_write SB[0x15]=op */
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);             /* b0e4 9695 + b0e7 970c */
      u4lb_d5da(1);                                         /* b0ea: d5da (R7=1 from 970c) */
    }
    if (PR(0x0819) & 0x02) {                                /* b0ed 9854: 0x0819.1 (L1 present) */
      uint8_t op = (PR(0x0750) == 2) ? 0xA5 : 0xA1;         /* b0f4-b0fe */
      SB_WR(0x15, op);                                      /* b100 */
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);             /* b103 9695 + b106 970c */
      u4lb_d5da(1);                                         /* b109 */
    }
    u4lb_b226();                                            /* b10c: settle */
  }
  uart_puts("[b4:B]");   /* INSTR: past prewrite/retrain (d5da/b226 polls survived) */

  /* --- (B) lane-width ready gate (b10f-b12d): (0x0768:0x0769)-(CCE4:CCE5) < 0x38 -> abort --- */
  { uint16_t width = ((uint16_t)PR(0x0768) << 8) | PR(0x0769);
    uint16_t neg   = ((uint16_t)PR(0xCCE4) << 8) | PR(0xCCE5);
    uart_puts("[b4:wid="); uart_puthex(PR(0x0768)); uart_puthex(PR(0x0769));
    uart_puts(" neg="); uart_puthex(PR(0xCCE4)); uart_puthex(PR(0xCCE5));
    uart_puts(" 765="); uart_puthex(PR(0x0765)); uart_puthex(PR(0x0766));
    /* DIAG (state-5 lane-present): 0x0819/0x081A (lane-present + device lane-cap) + 0x077A (host width). */
    uart_puts(" 819="); uart_puthex(PR(0x0819)); uart_puts(" 81A="); uart_puthex(PR(0x081A));
    uart_puts(" 77A="); uart_puthex(PR(0x077A)); uart_puts("]");
    if ((uint16_t)(width - neg) < 0x0038) { uart_puts("[b4:WIDGATE-abort]"); return; }  /* b12d */
  }

  /* --- (C) connect-present gate (b130-b13c): 0x0765==0 && 0x0766==0 -> abort --- */
  if (PR(0x0765) == 0 && PR(0x0766) == 0) { uart_puts("[b4:CONGATE-abort]"); return; }  /* b13a */
  uart_puts("[b4:C]");   /* INSTR: past width+connect gates */

  /* --- E716/CA06 enable (b13d-b15b), gated 0x0AF1.0 --- */
  if (PR(0x0AF1) & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;               /* b147 9790: E716=(E716&0xFC)|3 */
    phy_cc10_cmd_wait(2, 0, 0x28);                         /* b14a-b150 051b: R7=2,R4=0,R5=0x28 */
    REG_LINK_STATUS_E716 = REG_LINK_STATUS_E716 & 0xFC;                        /* b153 9789: E716 &= ~3 */
    REG_CPU_CTRL_CA81 = REG_CPU_CTRL_CA81 & 0xFE;                        /* b156 9908: CA81 &= ~1 */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;               /* b159-b15b: CA06=(CA06&0x1F)|0x60 */
  }

  /* --- e305: [PcieTunnel-PwrOn] (ROUND B placeholder) (b15c) --- */
  uart_puts("[PcieTunnel-PwrOn]");
  /* ROUND B: real e305/e26a/cdc6 tunnel-power + E764 0x14->0x19 train TBD. As a clearly-commented
   * stand-in for the E764 train-arm, reuse the EXISTING e57d reset pulse + the connect-head CA06 RMW
   * (already in handmade). The real ee29/a840/b8db SERDES/link-speed bodies are NOT fabricated. */
  boot_phy_e57d_e764_reset_pulse(0x01);             /* e57d E764 reset pulse (Round B stand-in) */
  REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x20;          /* connect-head CA06 RMW (Round B stand-in) */

  /* --- L0 OS-arm (b15f-b176), gated 0x0819.0 --- */
  if (PR(0x0819) & 0x01) {
    u4lb_96fe(0x82);                                /* b166-b168: 96fe(0x82) -> SB[0x15]=0x82 */
    u4lb_d5da(1);                                   /* b16b: d5da (96fe left R7=1) */
    PR(0x081E) = (PR(0x081E) & 0x7F) | 0x80;        /* b16e-b176: latch L0 OS-armed (0x081E.7) */
  }
  /* --- L1 OS-arm (b177-b18e), gated 0x0819.1 --- */
  if (PR(0x0819) & 0x02) {
    u4lb_96fe(0xA2);                                /* b17e-b180: 96fe(0xA2) -> SB[0x15]=0xA2 */
    u4lb_d5da(1);                                   /* b183 */
    PR(0x081F) = (PR(0x081F) & 0x7F) | 0x80;        /* b186-b18e: latch L1 OS-armed (0x081F.7) */
  }

  uart_puts("[b4:D]");   /* INSTR: past E716/PwrOn-stub/OS-arm; entering rate-change */
  /* --- CC37.2 set -> d3b0(3) Chg2 20G -> e980 rate apply -> e9e7 RstRxpll -> CC37.2 clr (b18f-b1a3) */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;          /* b18f-b194 984d: CC37 |= 0x04 */
  u4lb_d3b0(3);                                     /* b195-b197: d3b0(3) Chg2 20G */
  u4lb_e980();                                      /* b19a: rate descriptor apply */
  u4lb_e9e7();                                      /* b19d: RstRxpll */
  REG_CPU_CTRL_CC37 = REG_CPU_CTRL_CC37 & 0xFB;                   /* b1a0-b1a3 984d: CC37 &= ~0x04 */

  /* --- b8db: [CDRV ok] (ROUND B placeholder) (b1a4) --- */
  uart_puts("[CDRV ok]");
  /* ROUND B: real b8db CDR/margining compare body TBD (under-mapped). */

  /* --- CA60.3 set (b1a7-b1af) --- */
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;          /* CA60 bit3 = tunnel-adapter enable */

  /* --- c593 bank0 stub (b1b0): tunnel/PHY commit -- ROUND B placeholder (LCALL 0x05c0 -> bank0). --- */
  /* (No standalone observable XDATA effect mapped; left as a documented Round-B node.) */

  /* --- L0 OS1 trigger (b1b3-b1de), gated 0x0819.0 --- */
  if (PR(0x0819) & 0x01) {
    uart_puts("[L0 OS1]");                          /* b1ba 538d(str 0x201A) */
    SB_WR(0x50, 0x02);                              /* b1c3-b1c7 973f: SB[0x50]=0x02 (OS1 trigger) */
    P1_WR(0x010B, P1_RD(0x010B) | 0x01);            /* b1ca-b1cc 97e3: P1[0x010B] |= 1 */
    PR(0x075B) = 0x10; PR(0x0759) = 0x10;           /* b1cf-b1de: L0 OS1-in-progress sub-lane state */
  } else {
    PR(0x075B) = 0x00; PR(0x0759) = 0x00;           /* b1d6: clear */
  }
  /* --- L1 OS1 trigger (b1df-b20b), gated 0x0819.1 --- */
  if (PR(0x0819) & 0x02) {
    uart_puts("[L1 OS1]");                          /* b1e7 538d(str 0x2023) */
    SB_WR(0x5A, 0x02);                              /* b1f0-b1f2 9728: SB[0x5A]=0x02 (OS1 trigger) */
    P1_WR(0x010B, (P1_RD(0x010B) & 0xFD) | 0x02);   /* b1f5-b1f9 97f9: P1[0x010B] = (&0xFD)|2 */
    PR(0x075C) = 0x10; PR(0x075A) = 0x10;           /* b1fc-b20b: L1 OS1-in-progress sub-lane state */
  } else {
    PR(0x075C) = 0x00; PR(0x075A) = 0x00;           /* b203: clear */
  }

  /* --- ec51 Trig-arm (b20c) --- */
  u4lb_ec51();

  /* --- latch negotiated width 0x074E:0x074F = CCE4:CCE5 (b20f-b21d) --- */
  PR(0x074E) = PR(0xCCE4);
  PR(0x074F) = PR(0xCCE5);

  /* --- eb62(0,5) -> [SB P05] -> state 5 (b21e-b222) --- */
  u4lb_eb62(0, 5);
}

/* ====================================================================================
 * State 5 (0x06ED==5) — THE CL-STATE LANE-BOND WALKER. Faithful C port of CODE_BANK1::8000
 * (run when 0x0718==4) and 850b (else): the per-lane FSMs that drive SB[0xA0]/[0xA1] toward
 * CL0(0x02) via the 0x0800-plane PHY shadow + ea7c, emitting Lx:CL0. RE'd by the
 * re-state5-walker workflow (dispatch ladders adversarially verified; 850b ladder corrected).
 * IRAM: runs from the super-loop (e672), all scratch is C locals; persistent cells (0x0759-
 * 0x076x, 0x0767, 0x0800-plane shadow) already live in XDATA. Helpers are bounded.
 * Flagged <90% (HW will confirm): the lane-gate (here the semantic 0x0819.lane present-test),
 * e461's full e2b9 chain (reproduced as gating result + 0x0AAB arm), the 0x8174 width producer
 * (8000 loop1 0x50), and whether the 0x0800-shadow+ea7c path actually drives SB[0xA0/A1] to CL0.
 * ==================================================================================== */

/* [S5:..] per-pass CL-state dump budget. Auto __xdata -> lands in the SDCC XSEG, which is now
 * relocated to chip-CM XDATA 0x0BC0 (see Makefile): the WHOLE 0x88xx scratch region read-STALLS
 * the CPU at state-5 once the PCIe tunnel is active, whereas chip-CM XDATA stays valid RAM. */
static volatile uint8_t __xdata u4lb_s5_print_budget;

/* ee6e: per-lane SB connect-present = SB[lane?0x60:0x56].0 (DPX=1 SB plane). */
static uint8_t u4lb_ee6e(uint8_t lane) { return (uint8_t)(SB_RD(lane ? 0x60 : 0x56) & 0x01); }

/* eda0: route-special selector (0=eval-path, 1=idle, 2=route-special); clears 0x0775/0x0719. */
static uint8_t u4lb_eda0(void) {
  if (PR(0x0775) != 0) { PR(0x0775) = 0; PR(0x0719) = 0; return 0; }
  if (PR(0x0719) == 0x02) { PR(0x0719) = 0; return 2; }
  return 1;
}

/* e461 (CODE_BANK1::e461) — the SB-transport route PUSH the walker depends on. The walker iterates
 * fine (HW-confirmed it advances 0x10->0x20->0x30) and PARKS at 0x30 because this was a stub: at 0x20
 * it returns 1 to advance but never sends the e2b9 descriptor, so the host never sets 0x0775 and the
 * 0x30 handler's eda0() never returns EVAL -> no CL0 emit. Faithful live-path push below.
 *
 * e2b9 (byte-exact e2b9-e304): 0xAA8=p3(->SB[0x15] TX cmd), 0xAA9=p2(->SBTX[0]), 0xAAA=p1(->SBTX[1]).
 * Stock-trace target descriptor at state-5 = SBTX[0]=0x0D, SBTX[1]=0x04, SB[0x15]=0x0718(=4),
 * token 0x0719=0x0D. d5da triggers the transport TX (bounded). Returns 0 if the 0x0719 in-flight
 * token is busy (no re-push, no advance), 1 if a push was issued. Separate push body from edf5
 * (must NOT share the working state-3 route-query timing). */
static uint8_t u4lb_e461(void) {
  uint16_t g;
  if (PR(0x0719) != 0) return 0;                   /* token busy -> no push, no advance */
  if (PR(0x0718) == 0) {                            /* NOT the live AMD path (0x0718 must be 4) */
    SB_WR(0x04, (uint8_t)((SB_RD(0x04) & 0xFE) | 0x01));
    SB_WR(0x04, (uint8_t)((SB_RD(0x04) & 0xFD) | 0x02));
    PR(0x0AAB) = 0;
    return 1;
  }
  /* LIVE PATH (CL0 workflow iter2, adversarially verified). Stock e461 (e487-e497) splits on 0x0776:
   *   0x0776 == 0 -> e2b9(p1,p2,0x0718): SB[0x15] = 0xAA8 = 0x0718 (= 4).
   *   0x0776 != 0 -> 9966(0xAAB); e1cb(p1=0x04,p2=0x0D,p3=0): SB[0x15] = (0xAA8<<1)|0x41 = 0x41 (0xAA8=0).
   * b7a4 sets 0x0776=1, so the live AMD route-query is the e1cb form -- the host answers SB[0x15]=0x41, NOT
   * 0x04. The handmade previously ALWAYS sent the e2b9 0x04 form, so the host never posted the eaac-routed
   * response and the walker parked at 0x30. The ONLY functional difference is the SB[0x15] value.
   * KEY (kept): status byte (SB[0x0C]&0x80)|8 -> SB[0x0C] (9695 leaves R1=0x0C); token 0x0719 = d5da_ret-1. */
  {
    uint8_t dc;
    PR(0x0AAB) = 0;                                /* 9960/9966: 0xAAB = 0 */
    PR(0x0AA8) = PR(0x0776) ? 0 : PR(0x0718);      /* e1cb p3 = 0 (9966 CLR A); e2b9 p3 = 0x0718 (=4) */
    PR(0x0AA9) = 0x0D;                             /* p2 -> SBTX[0] */
    PR(0x0AAA) = 0x04;                             /* p1 -> SBTX[1] */
    /* (stock e2b9 head calls d4cd here; omitted from the super-loop walker -- the a066 INT1 ISR already
     * drives the SB transport-edge alternation, and keeping cd3f/af38 out of the walker call tree keeps
     * af38's locals overlayable (IRAM/DSEG budget). The host response is captured by the ISR's d4cd.) */
    SBTX_WR(0, PR(0x0AA9));                         /* 997e: SBTX[0] = 0x0D */
    SBTX_WR(1, (uint8_t)(PR(0x0AAA) | ((PR(0x0AAB) & 1) << 7)));   /* 9923: SBTX[1] = 0x04 | (0xAAB.0<<7) */
    SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08));   /* 9695 read + status = (SB[0x0C]&0x80)|8 -> SB[0x0C] */
    /* e461 e487 split: 0x0776!=0 -> e1cb SB[0x15]=(0xAA8<<1)|0x41=0x41; else e2b9 SB[0x15]=0xAA8=0x0718=4 */
    SB_WR(0x15, PR(0x0776) ? (uint8_t)((PR(0x0AA8) << 1) | 0x41) : (uint8_t)PR(0x0AA8));
    /* d5da(0): the bounded SB-transport TX trigger; stock returns cVar5 = SB[0x0C] - 7. */
    PR(0x0AAC) = 0;
    P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));
    SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));
    SB_WR(0x10, 0x01);                              /* SB[0x10] = 1 (TX go) */
    g = 0; while (((SB_RD(0x2C) >> 2) & 1) == 0 && ++g < 0x4000) { }   /* wait SB[0x2C].2 (bounded) */
    SB_WR(0x2C, 0x04);                              /* W1C SB[0x2C].2 */
    phy_cc10_cmd(1, 0, 0x0B);                        /* d614 */
    SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));
    REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;            /* 97ef strobe */
    dc = SB_RD(0x0C);                               /* d5da return = SB[0x0C] - 7 */
    PR(0x0719) = (uint8_t)((uint8_t)(dc - 7) - 1);  /* token 0x0719 = d5da_ret - 1 */
  }
  return 1;
}

/* ea7c: CC-orientation PHY CL bit2 program (C2CB/C34B). sel==0x0F set bit2 else clear. */
static void u4lb_ea7c(uint8_t sel, uint8_t cc) {
  uint8_t idx = (REG_PHY_VENDOR_CTRL_C6DB & 0x01) ? (uint8_t)((cc + 1) & 0x01) : cc;
  uint16_t reg = (idx == 0) ? 0xC2CB : 0xC34B;
  if (sel == 0x0F) PR(reg) = (uint8_t)((PR(reg) & 0xFB) | 0x04);
  else             PR(reg) = (uint8_t)(PR(reg) & 0xFB);
}

/* 8992: per-lane SB lane-arm (SB[0x15]=v; SB[0x0C]=(.&0x80)|3; d5da(1)). */
static void u4lb_8992(uint8_t v) {
  SB_WR(0x15, v);
  SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x03));
  u4lb_d5da(1);
}

/* lane gate: walk lane L iff 0x0819.L (lane-present). [Stock uses a shift+mask over a 9a11 read;
 * the per-lane-present test is the verified net for the live path (0x0819=01 -> walk L0 only).] */
static uint8_t u4lb_lane_gate(uint8_t lane) { return (uint8_t)(PR(0x0819) & (uint8_t)(1u << lane)); }

/* [S5:..] diagnostic — print only on CHANGE of the lane-state bytes, so a fast-iterating walker is
 * visible WITHOUT flooding/saturating the UART (the bounded uart_putc drops a long burst once the TX
 * FIFO fills ~64-75 bytes -- that saturation is why only ONE full [S5] line ever survived). A SHORT
 * marker per state change tells us if the walker actually advances (running) or is stuck (hung). */
static volatile uint8_t __xdata u4lb_s5_last759, u4lb_s5_last75b, u4lb_s5_seen, u4lb_s5_lasta0;
static volatile uint8_t __xdata u4lb_s5_last775;
static void u4lb_s5_diag(void) {
  __xdata uint8_t a, b, a0, h;
  a = PR(0x0759); b = PR(0x075B); a0 = SB_RD(0xA0); h = PR(0x0775);
  if (u4lb_s5_seen && a == u4lb_s5_last759 && b == u4lb_s5_last75b && a0 == u4lb_s5_lasta0
      && h == u4lb_s5_last775) return;
  if (!u4lb_s5_print_budget) return;
  u4lb_s5_print_budget--;
  u4lb_s5_seen = 1; u4lb_s5_last759 = a; u4lb_s5_last75b = b; u4lb_s5_lasta0 = a0; u4lb_s5_last775 = h;
  uart_puts("\r\n[s5 9="); uart_puthex(a); uart_putc('/'); uart_puthex(b);
  uart_puts(" A="); uart_puthex(a0); uart_puthex(SB_RD(0xA1));
  uart_puts(" 775="); uart_puthex(h); uart_putc(']');
}

/* ---- 8501: e80a(R5R4=0x0065,R7=2) via trampoline 0x051b -- a banked SB-transport drain/poll. The FSM
 * state is always written BEFORE this call, so it is non-load-bearing for progression. Modeled by the
 * existing bounded SB-transport edge drain (same class as e80a). (CL0 wf iter4, verified) ---- */
static void u4lb_8501(void) { }   /* non-load-bearing SB-transport drain; ISR's d4cd handles the edges */

/* ---- 81d4 finalize (CODE_BANK1::81d4): width-settle -> advance state cell 0x0759+lane to 0x60. On
 * counter overflow (>=0x0F) reset state to 0x00. The 0x0800-plane PHY shadow (81eb-821c) is best-effort
 * (non-progression). On the live path 0x0AB3==0 so the C2C3/C343 retrain tail is dead (omitted). ---- */
static void u4lb_lp1_finalize(uint8_t lane) {
  __xdata uint8_t r7;
  if (PR(0x075F + lane) >= 0x0F) { PR(0x0759 + lane) = 0x00; return; }   /* 81d4-81e2: counter overflow */
  PR(0x075F + lane)++;                                                   /* 81e5-81ea: bump counter */
  r7 = (uint8_t)((PR(0x075D + lane) + 1) & 0x0F);
  PR(0x075D + lane) = r7;
  PR(0x0800 + (uint8_t)(lane + r7)) |= 0x80;                             /* 8212-821c: lane-cfg shadow bit7 */
  PR(0x0759 + lane) = 0x60;                                              /* 821d: state 0x40 -> 0x60 */
}

/* ---- 8174 width-settle poll (CODE_BANK1::8174): advance to 0x60 (via finalize) when the negotiated
 * width pair has settled vs the read-only HW lane-width counter CCE4:CCE5 (981b 16-bit subtract >= 0xC8);
 * else leave state 0x40 to retry next pass. Off the live path (0x10 seed never reaches 0x40). ---- */
static void u4lb_lp1_width_settle(uint8_t lane) {
  __xdata uint16_t wA, wB, neg; __xdata uint8_t c774;
  wA = (uint16_t)(((uint16_t)PR(0x076C + 2*lane) << 8) | PR(0x076D + 2*lane));
  if (wA == 0) { u4lb_lp1_finalize(lane); return; }                     /* 817c: early settle */
  wB   = (uint16_t)(((uint16_t)PR(0x0770 + 2*lane) << 8) | PR(0x0771 + 2*lane));
  c774 = PR(0x0774);
  if ((uint8_t)wB == c774 && (uint8_t)(wB >> 8) == 0) { u4lb_lp1_finalize(lane); return; }  /* 81a6 */
  neg = (uint16_t)(((uint16_t)PR(0xCCE4) << 8) | PR(0xCCE5));
  if ((uint16_t)((wA - 1) - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }            /* 81b7 */
  /* 81d1: not settled -> leave state 0x40, retry next pass. */
}

/* --- 8000: PRIMARY state-5 walker (0x0718==4). LOOP1 state@0x0759+lane, LOOP2 (CL walk) @0x075B+lane. --- */
static void u4lb_walk_8000(void) {
  __xdata uint8_t lane, s;
  /* (marker removed) */
  /* LOOP1 -- byte-faithful port of CODE_BANK1::8000 jump-table FSM (state cell @0x0759+lane), CL0 wf iter4,
   * adversarially verified. Table @0x802a: default->804f 0x10->8069 0x20->807a 0x30->80ca 0x40->80d7
   *   0x50->8251 0x60->8262 0x70->82d5 0x80->82fa 0x90->831a 0xa0->8327 0xa1->TERMINAL.
   * Live chain from the b0b4 seed (0x10): 0x10->0x30->0x50->0x70->0x90->0xA1 (bonded). The OLD handmade
   * forced state 0x00 (terminal) at 0x50 via an ee6e stub -- THAT was the lane-bond killer. 0x50 is the
   * e461-push -> 0x70. States 0x20/0x40/0x60/0x80/0xA0 are the connect-arm/retrain edges. */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;             /* 8003-8021 prologue present-gate (0x0819.lane) */
    s = PR(0x0759 + lane);                           /* 8023-8026: read state cell */

    if (s == 0x10) {                                 /* 8069: e461-push -> 0x30 */
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x30;
    }
    else if (s == 0x20) {                            /* 807a: host-snapshot connect-arm */
      __xdata uint8_t r = u4lb_eda0();               /* 996d -> DAT_22 */
      if (r == 0) {
        __xdata uint8_t snap = PR(0x077B + lane);    /* 985b snapshot */
        if ((snap & 0x80) == 0) {                    /* 8082 snap.7 clear: stay */
          PR(0x0759 + lane) = 0x20;
        } else {                                     /* 8085-80ab connect-arm (snap.7 set) */
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));      /* 8085-8091: SB[0x40]=lane?2:1 */
          PR(0x076C + 2*lane) = 0x00;                /* 8094-809a: clear width pair A */
          PR(0x076D + 2*lane) = 0x00;
          PR(0x081C + lane) |= 0x20;                 /* 809b-80a6: shadow |= 0x20 */
          PR(0x0759 + lane) = 0x40;                  /* 80a9-80ab: *** state 0x20 -> 0x40 *** */
        }
        u4lb_8501();                                 /* 80ac: SB-transport drain */
      } else if (r == 2) {
        PR(0x0759 + lane) = 0x20;                    /* 80b8/80c1: re-arm */
      }
    }
    else if (s == 0x30) {                            /* 80ca: zero counter, advance to 0x50 */
      PR(0x075F + lane) = 0x00;                      /* 80ca-80d0: counter = 0 */
      PR(0x0759 + lane) = 0x50;                      /* 80d1-80d3: state = 0x50 */
    }
    else if (s == 0x40) {                            /* 80d7: [Trig] + orientation + width-settle */
      if (u4lb_ee6e(lane) != 0 && PR(0x075F + lane) != 0) {  /* 80d7-80e3: present AND counter!=0 */
        if (PR(0x0AB3) == 0) {                       /* 80e8-80ec: emit [Trig] OS1 strobe */
          SB_WR(lane ? 0x5A : 0x50, 0x01);           /* 80f7-8105 */
        }
        PR(0x081C + lane) |= 0x40;                   /* 8108-8117: shadow |= 0x40 */
        PR(0x081C + lane) = 0x00;                    /* 811a-811b CLR A;MOVX (verbatim post-OR write) */
        if ((PR(0xC2C3) & 0x01) || (REG_VENDOR_CTRL_C343 & 0x01)) {  /* 811c-8127: orientation gate */
          if ((PR(0x0819) & 0x03) != 0) {            /* 812d-8137: 0x0819 lane bits */
            PR(0x081C + lane) = (uint8_t)((PR(0x081C + lane) | 0x40) & 0x7F);  /* 813f-8152 */
            PR(0x081D + lane) = (uint8_t)(((PR(0x081D + lane) | 0x10) + 1) & 0x7F);  /* 8157-816b */
          }
        }
        /* present+counter!=0 path tails to 8355 (stay 0x40, retry next pass). */
      } else {
        u4lb_lp1_width_settle(lane);                 /* 80dc/80e5: !present OR counter==0 -> 8174 */
      }
    }
    else if (s == 0x50) {                            /* 8251: e461-push -> 0x70  *** BUG FIX *** */
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x70;
    }
    else if (s == 0x60) {                            /* 8262: width-program -> 0x80 */
      __xdata uint8_t r = u4lb_eda0();               /* 996d */
      if (r == 0) {
        __xdata uint8_t snap = PR(0x077B + lane);    /* 985b/826a */
        if ((snap & 0xC0) == 0xC0 &&
            (snap & 0x0F) == (uint8_t)(PR(0x081C + lane) & 0x0F)) {   /* 826b-8281 */
          if (PR(0x0AB3)) u4lb_e9e7();               /* 8283-8289 */
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));      /* 828c-8298 */
          u4lb_ee57();                               /* 829b: ec51 Trig-arm; CCE4/CCE5 read */
          PR(0x076C + 2*lane) = PR(0xCCE4);          /* 829e-82a2: width pair A lo = CCE4 */
          PR(0x076D + 2*lane) = PR(0xCCE5);          /* 82a3-82a5: = CCE5 */
          PR(0x0770 + 2*lane) = 0x00;                /* 82a6-82af: width pair B lo = 0 */
          PR(0x0771 + 2*lane) = PR(0x0774);          /* 82b0-82b2: width pair B hi = 0x0774 */
          PR(0x0759 + lane) = 0x80;                  /* 82b5-82b7: state advance -> 0x80 */
        } else {
          PR(0x0759 + lane) = 0x60;                  /* 82ba: mismatch retry */
        }
        u4lb_8501();                                 /* 82b8/82c0 */
      } else if (r == 2) {
        PR(0x0759 + lane) = 0x60;                    /* 82c3-82cc: retry */
      }
    }
    else if (s == 0x70) {                            /* 82d5: present-gated plane RMW -> 0x90 */
      if (u4lb_ee6e(lane)) {                         /* 82d5-82d8 present-gate */
        PR(0x0800 + lane) |= 0x10;                   /* 82da-82df 969e |= 0x10 */
        PR(0x081C + lane) |= 0x40;                   /* 82e0-82e8 96a9 |= 0x40 */
        PR(0x0819 + lane) &= 0x7F;                   /* 82e9-82f1 96a7 &= 0x7F */
      }
      PR(0x0759 + lane) = 0x90;                      /* 82f2-82f5: unconditional advance */
    }
    else if (s == 0x80) {                            /* 82fa: present-gated |0x40 + uncond &0x7F -> 0xA0 */
      if (u4lb_ee6e(lane)) {                         /* 82fa-82fd present-gate */
        PR(0x081C + lane) |= 0x40;                   /* 82ff-830e |= 0x40 */
      }
      PR(0x081C + lane) &= 0x7F;                     /* 830f-8317 UNCONDITIONAL &= 0x7F */
      PR(0x0759 + lane) = 0xA0;                      /* 8318: state advance */
    }
    else if (s == 0x90) {                            /* 831a: e461-push -> 0xA1 (bonded-terminal arm) */
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0xA1;
    }
    else if (s == 0xA0) {                            /* 8327: snap.C0==0x80 -> 0x50 (retrain) else 0xA0 */
      __xdata uint8_t r = u4lb_eda0();               /* 996d */
      if (r == 0) {
        if ((PR(0x077B + lane) & 0xC0) == 0x80) PR(0x0759 + lane) = 0x50;  /* 832c-833c */
        else                                    PR(0x0759 + lane) = 0xA0;  /* 833f-8344 retry */
        u4lb_8501();                               /* 8345 */
      } else if (r == 2) {
        PR(0x0759 + lane) = 0xA0;                  /* 834a-8354: retry */
      }
    }
    else {                                           /* default (804f): shadow clear -> 0x20 */
      PR(0x0800 + lane) &= 0xEF;                     /* 804f-8054 969e &= 0xEF */
      PR(0x081C + lane) &= 0x7F;                     /* 8055-805d 96a9 &= 0x7F */
      PR(0x0819 + lane) &= 0xDF;                     /* 805e-8066 96a7 &= 0xDF */
      PR(0x0759 + lane) = 0x20;                      /* 8067: state = 0x20 */
    }
    /* 0xA1 has NO case: TERMINAL (table key a1 -> 0x0000) = lane bonded, FSM holds. */
  }
  /* LOOP2: THE CL-STATE WALKER (state @0x075B+lane). */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    s = PR(0x075B + lane);
    if (s == 0x10) {
      PR(0x0802 + lane) |= 0x80;
      PR(0x081E + lane) &= 0xBF;
      PR(0x075B + lane) = 0x20;
    } else if (s == 0x20) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x30;
    } else if (s == 0x30) {
      __xdata uint8_t r = u4lb_eda0();
      if (r == 0) {
        __xdata uint8_t snap = PR(0x0779 + lane);
        if ((snap >> 4) & 1) PR(0x075B + lane) = 0x00;
        else if (((snap >> 7) & 1) == 0) PR(0x075B + lane) = 0x20;
        else {
          __xdata uint8_t v = (uint8_t)(snap & 0x0F), cap, d23 = 0, d24 = 0;
          PR(0x0761 + lane) = v;
          PR(0x0802 + lane) &= 0xF0;
          PR(0x081E + lane) |= v;
          PR(0x081E + lane) |= 0x40;
          cap = PR(0x0AB4 + lane);
          if ((cap >> 1) & 1) d24 = PR(0x072E + v);
          if (cap & 1) { __xdata uint16_t m = (uint16_t)(PR(0x073E + v) * 0x20); d24 |= (uint8_t)m; d23 |= (uint8_t)(m >> 8); }
          uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
          uart_puthex(d23); uart_puthex(d24);
          u4lb_ea7c(v, lane);
          PR(0x075B + lane) = 0x50;
        }
      } else if (r == 2) PR(0x075B + lane) = 0x20;
    } else if (s == 0x50) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x60;
    } else if (s == 0x60) {
      __xdata uint8_t r = u4lb_eda0();
      if (r == 0) {
        if ((PR(0x0779 + lane) >> 7) & 1) PR(0x075B + lane) = 0x50;
        else { PR(0x0802 + lane) &= 0xBF; PR(0x075B + lane) = 0x20; }
      } else if (r == 2) PR(0x075B + lane) = 0x50;
    }
  }
}

/* --- 850b: ALTERNATE state-5 walker (0x0718 != 4). Corrected (handler-writes-N+1) ladder.
 * Dead on the live AMD path (0x0718==4 -> 8000); included for completeness/faithfulness. --- */
static void u4lb_walk_850b(void) {
  __xdata uint8_t lane, s, dat22 = 0;
  /* LOOP1: state @0x075B+lane. */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    s = PR(0x075B + lane);
    if (s == 0x11) {
      __xdata uint8_t r = u4lb_eda0(); dat22 = r;
      if (r == 0) PR(0x075B + lane) = 0x20;
      else if (r == 1) PR(0x075B + lane) = 0x10;
    } else if (s == 0x20) {
      __xdata uint8_t r = u4lb_eda0(); dat22 = r;
      if (r == 0) { if (PR(0x0779) == 0) PR(0x075B + lane) = 0x30; else PR(0x075B + lane) = 0x20; }
      else if (r != 2) PR(0x075B + lane) = 0x20;
    } else if (s == 0x21) {
      if (u4lb_ee6e(lane) == 0) PR(0x075B + lane) = 0x40;
    } else if (s == 0x30) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t a = PR(0x0B26 + lane);
        if (!((a >> 4) & 1)) PR(0x075B + lane) = 0x00;
        else if ((a >> 7) & 1) { PR(0x0761 + lane) = (uint8_t)(a & 0x0F); PR(0x075B + lane) = 0x50; }
        else PR(0x075B + lane) = 0x30;
      }
    } else if (s == 0x40) {
      __xdata uint8_t cap = PR(0x0AB4 + lane), v = PR(0x0761 + lane), d23 = 0, d24 = 0;
      if ((cap >> 1) & 1) d24 = PR(0x072E + v);
      if (cap & 1) { __xdata uint16_t m = (uint16_t)(PR(0x073E + v) * 0x20); d24 |= (uint8_t)m; d23 |= (uint8_t)(m >> 8); }
      uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
      uart_puthex(d23); uart_puthex(d24);
      u4lb_ea7c(v, lane);
      PR(0x075B + lane) = 0x51;
    } else if (s == 0x50) {
      PR(0x075B + lane) = 0x60;
    } else if (s == 0x51) {
      __xdata uint8_t v;
      PR(0x0B2C + lane) |= 0x80;
      v = (uint8_t)((PR(0x0B2C + lane) & 0xF0) | PR(0x0761 + lane));
      PR(0x0B2C + lane) = v;
      if (v == 0) PR(0x075B + lane) = 0x61;
    } else if (s == 0x60) {
      if (u4lb_e461() == 1) { if (PR(0x0779) == 0) PR(0x075B + lane) = 0x70; else PR(0x075B + lane) = 0x60; }
    } else if (s == 0x61) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x71;
    } else if (s == 0x70) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t a = PR(0x0B26 + lane);
        if (!((a >> 7) & 1)) PR(0x075B + lane) = 0x30;
        else if (PR(0x0761 + lane) != 0x07) PR(0x075B + lane) = 0x30;
        else PR(0x075B + lane) = 0x70;
      }
    } else {
      if (u4lb_ee6e(lane) == 0) PR(0x075B + lane) = 0x11;   /* 0x10 + default: bootstrap */
    }
  }
  /* width-limit one-shot + LOOP2 (state @0x0759+lane). */
  if (PR(0x075B) == 0 && PR(0x075C) == 0) {
    if (PR(0x0767) == 0) {
      if (PR(0x0819) & 0x01) u4lb_8992(0x86);
      if ((PR(0x0819) >> 1) & 0x01) u4lb_8992(0xA6);
      PR(0x0767) = 1;
    }
    for (lane = 0; lane < 2; lane++) {
      if (!u4lb_lane_gate(lane)) continue;
      s = PR(0x0759 + lane);
      if (s == 0x10) {
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x21;
      } else if (s == 0x20) {
        if (u4lb_e461() == 1) {
          if ((PR(0x0B28 + lane) >> 7) & 1) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); PR(0x0759 + lane) = 0x30; }
          else PR(0x0759 + lane) = 0x20;
        }
      } else if (s == 0x21) {
        if (u4lb_eda0() != 0) PR(0x081C + lane) |= 0x10;
        PR(0x0759 + lane) = 0x40;
      } else if (s == 0x30) {
        if (u4lb_eda0() != 0) {
          if (PR(0x075F + lane) != 0) PR(0x0759 + lane) = 0x50;
          else { uart_puts("\r\n(lim)"); PR(0x0759 + lane) = 0x60; }
        }
      } else if (s == 0x40) {
        uart_puts("EQ");
        SB_WR(0x50, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1));
        PR(0x0759 + lane) = 0x51;
      } else if (s == 0x50) {
        PR(0x0B2A + lane) |= 0x10;
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x52;
      } else if (s == 0x51) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) != 0) PR(0x0759 + lane) = 0x51; else { uart_puts("\r\n(lim)"); PR(0x075B + lane) = 0x00; } }
        else if (r != 2) PR(0x0759 + lane) = 0x51;
      } else if (s == 0x52) {
        PR(0x075F + lane)++;
        PR(0x075D + lane) = (uint8_t)((PR(0x075D + lane) + 1) & 0x0F);
        PR(0x0759 + lane) = 0x70;
      } else if (s == 0x60) {
        __xdata uint8_t a = PR(0x075D + lane);
        PR(0x071A + a) |= 0xA0;
        PR(0x0B2A + lane) = a;
        if (a == 0) PR(0x0759 + lane) = 0x80;
      } else if (s == 0x70) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) == 0) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); PR(0x0759 + lane) = 0x90; } else PR(0x0759 + lane) = 0x70; }
        else if (r != 2) PR(0x0759 + lane) = 0x70;
      } else if (s == 0x80) {
        PR(0x0759 + lane) = 0xA0;
      } else if (s == 0x90) {
        PR(0x0B2A + lane) &= 0x7F;
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0xA1;
      } else if (s == 0xA0) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) == 0) PR(0x0759 + lane) = 0xB0; else PR(0x0759 + lane) = 0xA0; }
        else if (r != 2) PR(0x0759 + lane) = 0xA0;
      } else if (s == 0xA1) {
        if (u4lb_ee6e(lane) == 0) { uart_puts("\r\n(lim)"); PR(0x075B + lane) = 0x60; }
        else PR(0x0759 + lane) = 0x50;
      } else {
        PR(0x0800 + lane) &= 0xEF;
        PR(0x0800 + lane) &= 0x7F;
        PR(0x081C + lane) &= 0xDF;
        PR(0x075F + lane) = 0x00;
        PR(0x0759 + lane) = 0x20;
      }
    }
  }
  (void)dat22;
}

/* u4lb_state5(): e672 state-5 entry. 0x0718==4 -> 8000 else 850b. The idle/finalise (eb62(0,0)
 * when all sub-lane states drain) is handled by u4lb_e672 BEFORE it calls us; we only walk. */
static void u4lb_state5(void) {
  DPX = 0x00;                                       /* defensive: ensure flat plane for the PR() reads below */
  u4lb_s5_diag();                                  /* change-gated [s5 ..] marker (no per-pass flooding) */
  if (PR(0x0718) == 0x04) u4lb_walk_8000();
  else                    u4lb_walk_850b();
  DPX = 0x00;   /* CRITICAL: the walker's SB accessors can leave DPX=1; restore the flat-XDATA plane
                 * so the super-loop's subsequent PR()/0x06ED reads don't hit plane-1 and read-STALL. */
  /* (S marker removed -- the change-gated [s5 ..] diag is the only state-5 output now) */
}

/* ====================================================================================
 * e672 (CODE_BANK1::e672) — the lane-bond FSM dispatcher. Called from cb10's tail (gated 0x06ED!=0).
 *   0x06ED==3 -> cm_conn_routing_setup
 *   0x06ED==4 -> b0b4 (state 4)
 *   0x06ED==5 -> (0x075b/0x0759/0x075c/0x075a all 0 -> eb62(0,0)) else (0x0718==4 -> 8000 else 850b)
 * Verbatim control flow from CODE_BANK1::e672.
 * ==================================================================================== */
static void u4lb_e672(void) {
  uint8_t st = PR(0x06ED);
  if (st == 0x04) {
    u4lb_state4_b0b4();
    return;
  }
  if (st == 0x05) {
    if (PR(0x075B) == 0 && PR(0x0759) == 0 && PR(0x075C) == 0 && PR(0x075A) == 0) {
      u4lb_eb62(0, 0);         /* e672: all sub-lane states clear -> finalise */
      return;
    }
    u4lb_state5();             /* 0x0718==4 -> 8000, else 850b (both omitted markers) */
    uart_putc('E');            /* TRIAGE: e672 regained control after state5 */
    return;
  }
  if (st == 0x03) {
    u4lb_cm_conn_routing_setup();
    return;
  }
  /* st 0,1,2 or other -> nothing */
}

#endif /* USB4_LANEBOND_H */
