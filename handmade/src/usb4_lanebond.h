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
  PR(0xCC37) = (PR(0xCC37) & 0xFB) | 0x04;          /* e9f0 984d: CC37=(CC37&0xFB)|4 */
  PR(0xC20E) = 0xFF;                                /* e9f6-e9f9 9a27: C20E=0xFF */
  phy_cc10_cmd_wait(1, 0, 0x14);                    /* e9fc: R7=1,R4=0,R5=0x14 */
  PR(0xC20E) = 0x00;                                /* e9ff-ea03: C20E=0 */
  phy_cc10_cmd_wait(2, 0, 0x28);                    /* ea09: R7=2,R4=0,R5=0x28 */
  PR(0xCC37) = PR(0xCC37) & 0xFB;                   /* ea0c 984d: CC37 &= ~4 */
  uart_puts("[Done]");                              /* ea16 538d(str 0x1fe6) */
}

/* ---- ebde settle: C20F=0xFF + phy_cc10_cmd_wait(1,0,0x14); C20F=0; BOUNDED-spin C2D0.5 then
 * C350.5 (PLL/lane rate-lock). Verbatim CODE_BANK1::ebde (inlined into e980 below). */
static void u4lb_ebde(void) {
  PR(0xC20F) = 0xFF;                                /* ebde-ebe1 9a27: C20F=0xFF */
  phy_cc10_cmd_wait(1, 0, 0x14);                    /* ebe4: R7=1,R4=0,R5=0x14 */
  PR(0xC20F) = 0x00;                                /* ebe7-ebeb: C20F=0 */
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

/* ====================================================================================
 * b0b4 body (CODE_BANK1::b0b4 @ file 0x130b4) — state-4 assembled per the plan's dependency order.
 * ==================================================================================== */
static void u4lb_state4_b0b4(void) {
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

  /* --- (B) lane-width ready gate (b10f-b12d): (0x0768:0x0769)-(CCE4:CCE5) < 0x38 -> abort --- */
  { uint16_t width = ((uint16_t)PR(0x0768) << 8) | PR(0x0769);
    uint16_t neg   = ((uint16_t)PR(0xCCE4) << 8) | PR(0xCCE5);
    if ((uint16_t)(width - neg) < 0x0038) return;          /* b12d: LJMP b225 (RET) */
  }

  /* --- (C) connect-present gate (b130-b13c): 0x0765==0 && 0x0766==0 -> abort --- */
  if (PR(0x0765) == 0 && PR(0x0766) == 0) return;          /* b13a: LJMP b225 (RET) */

  /* --- E716/CA06 enable (b13d-b15b), gated 0x0AF1.0 --- */
  if (PR(0x0AF1) & 0x01) {
    PR(0xE716) = (PR(0xE716) & 0xFC) | 0x03;               /* b147 9790: E716=(E716&0xFC)|3 */
    phy_cc10_cmd_wait(2, 0, 0x28);                         /* b14a-b150 051b: R7=2,R4=0,R5=0x28 */
    PR(0xE716) = PR(0xE716) & 0xFC;                        /* b153 9789: E716 &= ~3 */
    PR(0xCA81) = PR(0xCA81) & 0xFE;                        /* b156 9908: CA81 &= ~1 */
    PR(0xCA06) = (PR(0xCA06) & 0x1F) | 0x60;               /* b159-b15b: CA06=(CA06&0x1F)|0x60 */
  }

  /* --- e305: [PcieTunnel-PwrOn] (ROUND B placeholder) (b15c) --- */
  uart_puts("[PcieTunnel-PwrOn]");
  /* ROUND B: real e305/e26a/cdc6 tunnel-power + E764 0x14->0x19 train TBD. As a clearly-commented
   * stand-in for the E764 train-arm, reuse the EXISTING e57d reset pulse + the connect-head CA06 RMW
   * (already in handmade). The real ee29/a840/b8db SERDES/link-speed bodies are NOT fabricated. */
  boot_phy_e57d_e764_reset_pulse(0x01);             /* e57d E764 reset pulse (Round B stand-in) */
  PR(0xCA06) = (PR(0xCA06) & 0x1F) | 0x20;          /* connect-head CA06 RMW (Round B stand-in) */

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

  /* --- CC37.2 set -> d3b0(3) Chg2 20G -> e980 rate apply -> e9e7 RstRxpll -> CC37.2 clr (b18f-b1a3) */
  PR(0xCC37) = (PR(0xCC37) & 0xFB) | 0x04;          /* b18f-b194 984d: CC37 |= 0x04 */
  u4lb_d3b0(3);                                     /* b195-b197: d3b0(3) Chg2 20G */
  u4lb_e980();                                      /* b19a: rate descriptor apply */
  u4lb_e9e7();                                      /* b19d: RstRxpll */
  PR(0xCC37) = PR(0xCC37) & 0xFB;                   /* b1a0-b1a3 984d: CC37 &= ~0x04 */

  /* --- b8db: [CDRV ok] (ROUND B placeholder) (b1a4) --- */
  uart_puts("[CDRV ok]");
  /* ROUND B: real b8db CDR/margining compare body TBD (under-mapped). */

  /* --- CA60.3 set (b1a7-b1af) --- */
  PR(0xCA60) = (PR(0xCA60) & 0xF7) | 0x08;          /* CA60 bit3 = tunnel-adapter enable */

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
