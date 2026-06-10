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
 * SCOPE OF THIS FILE (HONEST): the FSM DRIVER (e672 + eb62 + the db7a/cb10 wiring) and the state-3
 * body cm_conn_routing_setup are transcribed faithfully here. The state-4 (b0b4) and state-5
 * (8000/850b) bodies are the deep PHY per-lane CL-state walkers: they descend through ~4 layers of
 * undocumented PHY / SB / tunnel-adapter register accessors (b0b4->e9e7/d3b0/e980/b8db/8000->
 * 96a7/9a11/982b/97c9/0c7a/ea7c... and bank0 a2c1/a2c2/a37b/ce23...). Transcribing those verbatim is
 * a multi-hundred-function effort and is NOT reproduced here (doing it partially would FABRICATE the
 * exact PHY write sequence that drives SB[0xA0]->CL0 -- the project's repeated failure mode). They
 * are wired as state markers (u4lb_state4/state5) so the FSM advances and the UART shows EXACTLY how
 * far the lanes walk on HW, pinpointing the next transcription target.
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
    /* a800: edf5(0,3) sub-confirm then 0x0758=0x11. edf5 gated on 0x0719==0 -> e2b9(..,5) banked
     * sideband op; reproduced as the state advance (the load-bearing effect is 0x0758=0x11). */
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
  /* eda0(): 0x0775/0x0719 housekeeping (clears 0x0775->0x0719=0; if 0x0719==2 ->0). */
  if (PR(0x0775) != 0) { PR(0x0775) = 0; PR(0x0719) = 0; }
  else if (PR(0x0719) == 0x02) { PR(0x0719) = 0; }

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
 * [L0 OS1][L1 OS1] lane-bond engine. This descends through e9e7 (RstRxpll), d3b0 (Chg2), e980, b8db
 * (CDRV margining compare), e07d/e305 + ~25 PHY accessors. DEEP PHY LAYER -- NOT transcribed (would
 * fabricate the SB[0x81]/[0xa1] OS1 + CDR writes that move the lanes). Wired as a marker that advances
 * to state 5 so the FSM walk is observable; if the lanes don't move past this, THIS is the next
 * transcription target (b0b4 @CODE_BANK1::b0b4 + its helper tree).
 * ==================================================================================== */
static void u4lb_state4_b0b4(void) {
  uart_puts("[u4lb:S4 b0b4 OMITTED]");
  u4lb_eb62(0, 5);             /* b222: eb62(0,5) -> [SB P05] -> state 5 */
}

/* ====================================================================================
 * State 5 (0x06ED==5) — e672 runs 8000 (if 0x0718==4) else 850b: the per-lane CL-state walker that
 * drives SB[0xA0]/[0xA1] 0x07->...->CL0=0x02 and prints [Trig]/L0:CL0/L1:CL0. DEEP PHY LAYER (8000
 * @CODE_BANK1::8000 / 850b @CODE_BANK1::850b -- the ~150-line lane FSM with the func_0def movc
 * dispatch). NOT transcribed. Wired as a marker that finalises to state 0.
 * ==================================================================================== */
static void u4lb_state5(void) {
  uart_puts("[u4lb:S5 8000/850b OMITTED]");
  u4lb_eb62(0, 0);             /* finalise -> [SB P00] -> FSM idle */
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
    return;
  }
  if (st == 0x03) {
    u4lb_cm_conn_routing_setup();
    return;
  }
  /* st 0,1,2 or other -> nothing */
}

#endif /* USB4_LANEBOND_H */
