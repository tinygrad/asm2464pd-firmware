#ifndef USB4_LANEBOND_H
#define USB4_LANEBOND_H
/*
 * USB4 lane-bond / CL0 / PCIe-tunnel bring-up FSM. State lives in XDATA 0x06ED and is driven from
 * the super-loop via e672: 3 -> connection-routing setup, 4 -> b0b4 tunnel power-on / lane-bond,
 * 5 -> the per-lane CL-state walker (8000/850b). Include after usb4_connect.h.
 */

#define U4LB_STATE   PR(0x06ED)

/* eb62(p1,p2): set the new FSM state, print "[SB P0<state>]", and store it into 0x06ED. */
static void u4lb_eb62(uint8_t state_lo, uint8_t state) {
  PR(0x0AAC) = state_lo;
  PR(0x0AAD) = state;
  uart_puts("\r\n[SB P0");
  uart_puthex(state);
  uart_putc(']');
  PR(0x06ED) = PR(0x0AAD);
}

/* [EDF5] dump budget (XDATA, seeded in main()). */
static volatile uint8_t __xdata __at(0x0B58) u4lb_edf5_print_budget;

/* edf5 -> e2b9: the device->host SB-transport route-query that prompts the host to post the
 * connection-routing descriptor. Gated by the 0x0719 in-flight token; d5da's poll is bounded so the
 * super-loop can't hang. Returns 1 only when a query was actually sent. */
static uint8_t u4lb_edf5_route_query(void) {
  uint8_t staged;
  if (PR(0x0719) != 0) return 0;

  PR(0x0AAB) = 0;
  PR(0x0AA8) = 5;
  PR(0x0AA9) = 0x0C;
  PR(0x0AAA) = 3;
  sb_d4cd_transport_edges();

  SBTX_WR(0, PR(0x0AA9));
  SBTX_WR(1, (uint8_t)(PR(0x0AAA) | ((PR(0x0AAB) & 1) << 7)));
  staged = (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08);
  SB_WR(0x0C, staged);

  SB_WR(0x15, PR(0x0AA8));

  /* d5da(0): the bounded SB-transport TX trigger. */
  { uint8_t tx_done; uint16_t g = 0;
    PR(0x0AAC) = 0;
    P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));
    SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));
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
      uart_putc('|'); uart_puthex(P1_RD(0x0109)); uart_putc(' '); uart_puthex(SB_RD(0xD8));
      uart_putc(' '); uart_puthex(SB_RD(0x3A)); uart_puthex(SB_RD(0x3B));
      uart_puthex(SB_RD(0x3C)); uart_puthex(SB_RD(0x3D));
      uart_putc(']');
    }
    SB_WR(0x10, 0x01);
    while (((SB_RD(0x2C) >> 2) & 1) == 0 && ++g < 0x4000) { }
    tx_done = (uint8_t)((SB_RD(0x2C) >> 2) & 1);
    if (u4lb_edf5_print_budget) {
      u4lb_edf5_print_budget--;
      uart_puts("[TXdone done="); uart_puthex(tx_done);
      uart_puts(" sb2c="); uart_puthex(SB_RD(0x2C));
      uart_puts(" g="); uart_puthex((uint8_t)(g >> 8)); uart_puthex((uint8_t)g);
      uart_puts("]\r\n");
    }
    SB_WR(0x2C, 0x04);
    phy_cc10_cmd(1, 0, 0x0B);
    SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));
    (void)tx_done;
  }

  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
  PR(0x0719) = 0x01;
  return 1;
}

/* cm_conn_routing_setup — [ConnRout] connection-routing FSM (state 0x06ED==3). Runs the 0x0758
 * sub-FSM (0x10/0x11/0x00), evaluates the host connect-descriptor confirm gate, prints [ConnRout]
 * and sets 0x0718 ROUTE-ENABLE, latches the lane-width state, and advances toward state 4. */

/* e391 LUT-seed ROM tables: per-descriptor-type width LUT (-> XDATA[0x06F2+i]) and the af38
 * BRANCH-A presence gate (-> XDATA[0x0705+i]). */
static __code const uint8_t u4lb_width_lut_514c[0x13] = {
  0x04,0x04,0x00,0x04,0x04,0x00,0x00,0x00, 0x04,0x04,0x01,0x00, 0x03,0x04,0x00,0x04, 0x00,0x00,0x10
};
static __code const uint8_t u4lb_branchA_gate_515f[0x13] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x01
};

static void u4lb_cm_conn_routing_setup(void) {
  uint8_t state = PR(0x0758);
  if (state == 0x10) {
    if (u4lb_edf5_route_query() != 1) return;
    PR(0x0758) = 0x11;
    return;
  }
  if (state != 0x11) {
    if (state != 0x00) return;
    u4lb_eb62(0, 4);
    return;
  }

  /* state == 0x11: the main confirm body. eda0 returns a selector: 0 = eval the confirm gate,
   * 1 = idle (leave 0x0758), 2 = route-special re-arm. */
  { uint8_t selector;
    if (PR(0x0775) != 0)      { PR(0x0775) = 0; PR(0x0719) = 0; selector = 0; }
    else if (PR(0x0719) == 0x02) { PR(0x0719) = 0; selector = 2; }
    else                       { selector = 1; }
    if (selector == 2) { PR(0x0758) = 0x10; return; }
    if (selector != 0) { return; }
  }

  /* mode==0 path: gate on the host connect descriptor 0x0777==0x0C. */
  if (PR(0x0777) != 0x0C) { PR(0x0758) = 0x10; return; }

  { static __xdata uint8_t conn_routing_diag_budget = 6;
    if (conn_routing_diag_budget) { conn_routing_diag_budget--;
      uart_puts("\r\n[cr B9="); uart_puthex(PR(0x07B9)); uart_puts(" 778="); uart_puthex(PR(0x0778));
      uart_puts(" 81B="); uart_puthex(PR(0x081B)); uart_puts(" CE="); uart_puthex(PR(0x07CE));
      uart_puts(" CD="); uart_puthex(PR(0x07CD)); uart_puts(" 776="); uart_puthex(PR(0x0776)); uart_putc(']'); } }

  /* 0x0776 connect-confirm computation. */
  if (PR(0x07B9) != 0) {
    uint8_t host_status = PR(0x0778);
    if (((host_status & 0x7F) == 2) || ((PR(0x081B) & 1) == 0) ||
        (PR(0x07CE) != 0 && PR(0x07CD) == 0)) {
      PR(0x0776) = 0;
    } else {
      SB_CLR(0xED, 0x80);
    }
  }
  /* e391 width-LUT seed (gated 0x0776==0): the per-descriptor LUT af38 ORs into SBTX[1]. */
  if (PR(0x0776) == 0) {
    uint8_t i;
    for (i = 0; i < 0x13; i++) {
      PR((uint16_t)(0x06F2u + i)) = u4lb_width_lut_514c[i];
      PR((uint16_t)(0x0705u + i)) = u4lb_branchA_gate_515f[i];
    }
  }

  /* [ConnRout] confirm print + 0x0718 ROUTE-ENABLE. */
  if (PR(0x0776) == 0 && PR(0x07CE) != 0) {
    uart_puts("[ConnRtmr]");
    PR(0x0718) = 0;
  } else {
    uart_puts("[ConnRout]");
    PR(0x0718) = 4;
  }

  /* Latch the 0x077a lane-width bits into 0x0819/0x0751/0x0750. */
  { uint8_t host_width = PR(0x077A);
    if ((host_width & 1) && (PR(0x081A) & 1)) { PR(0x0819) = (PR(0x0819) & 0xFE) | 1; }
    if ((host_width & 0x02) && (PR(0x081A) & 2)) { PR(0x0819) = (PR(0x0819) & 0xFD) | 2; }
    if ((host_width & 0x10) && (PR(0x081A) & 0x10) && (PR(0x0819) & 1) && (PR(0x0819) & 2)) PR(0x0751) = 1;
    else PR(0x0751) = 0;
    if ((host_width & 0x20) && (PR(0x081A) & 0x20)) PR(0x0750) = 2;
    uart_puts("[Lt77A="); uart_puthex(host_width); uart_puts(" 81A="); uart_puthex(PR(0x081A));
    uart_puts(" 819="); uart_puthex(PR(0x0819)); uart_puts("]");
    PR(0x0763) = 0; PR(0x0764) = 0;
  }

  /* c586: negotiated-rate descriptor (SB[0x6A-0x6D]/[0x74-0x75]) + Gen2 lane-eq retrim. */
  {
    uint16_t rate = (uint16_t)((uint16_t)PR(0x0749) * 0x20);
    uint8_t rate_hi = (uint8_t)(rate >> 8);
    uint8_t rate_lo = (uint8_t)((rate & 0xFF) | PR(0x0739));
    SB_WR(0x6A, rate_hi); SB_WR(0x6B, rate_lo);
    SB_WR(0x6C, rate_hi); SB_WR(0x6D, rate_lo);
    SB_WR(0x74, 0x00); SB_WR(0x75, (uint8_t)((PR(0x0750) == 2) ? 0x1F : 0x0F));
    if (REG_LANE_RATE_C8FF == 0x04 && PR(0x07BA) == 0) {
      PR(0xC294) = (uint8_t)((PR(0xC294) & 0xF0) | 0x03);
      PR(0xC293) = (uint8_t)((PR(0xC293) & 0xFC) | 0x02);
      PR(0xC314) = (uint8_t)((PR(0xC314) & 0xF0) | 0x03);
      PR(0xC313) = (uint8_t)((PR(0xC313) & 0xFC) | 0x02);
    }
    if (PR(0x0750) == 1) {
      PR(0xC2C5) = (uint8_t)((PR(0xC2C5) & 0xF0) | 0x0F);
      PR(0xC345) = (uint8_t)((PR(0xC345) & 0xF0) | 0x0F);
    }
  }

  PR(0x0758) = 0;
}

/* State 4 (0x06ED==4) — b0b4: [PcieTunnel-PwrOn] -> Chg2 20G -> [RstRxpll] -> [CDRV ok] ->
 * [L0/L1 OS1] lane-bond / 20G-rate / RxPLL-reset / per-lane OS1-arm engine, plus its helpers. Every
 * busy-poll is bounded. SB[off]=0x2800+off, SB2[off]=0x2900+off, P1[0x01xx]=0x0100+off; the C2xx/C3xx/
 * CAxx/CCxx/E716 registers are plain XDATA via PR(). */

/* SB2: page-1 lane-block accessor at 0x2900+off (mirror of SB_RD/WR base 0x2800). */
#define SB2_RD(off)      P1_REG8_rd((uint16_t)(0x2900u + (off)))
#define SB2_WR(off, v)   P1_REG8_wr((uint16_t)(0x2900u + (off)), (uint8_t)(v))

/* 96fe: per-lane OS/CDR command-issue descriptor (SB[0x15]=op; SB[0x0C]=(.&0x80)|3). */
static void u4lb_96fe(uint8_t op) {
  SB_WR(0x15, op);
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
}

/* d5da: the per-lane PHY-RX/CDR commit + settle handshake. param==1 adds the SB[0x0F]|=1 prologue;
 * param==0 also zeroes the SB2 descriptor tail when SB[0x0C] > 6. */
static void u4lb_d5da(uint8_t param) {
  PR(0x0AAC) = param;
  if (param == 1) {
    SB_WR(0x0F, (SB_RD(0x0F) & 0xFE) | 0x01);
  }
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);
  SB_WR(0x04, SB_RD(0x04) & 0xFD);
  uart_puts("[Td="); uart_puthex(SB_RD(0x15)); uart_putc(':'); uart_puthex(SBTX_RD(0)); uart_puthex(SBTX_RD(1)); uart_putc(']');
  SB_WR(0x10, 0x01);
  { uint16_t g = 0;
    while (((SB_RD(0x2C) & 0x04) == 0) && ++g < 0x2000); }
  SB_WR(0x2C, 0x04);
  phy_cc10_cmd_wait(1, 0, 0x0B);
  SB_WR(0x0F, SB_RD(0x0F) & 0xFE);
  if (param != 0) return;
  { uint8_t count = SB_RD(0x0C);
    if (count <= 6) return;
    { uint8_t limit = (uint8_t)(count - 6), i;
      for (i = 0; i < limit; i++) SB2_WR(i, 0x00); }
  }
}

/* e07d: retrain-path per-lane PHY/SB2 lane-block program. */
static void u4lb_e07d(void) {
  uint8_t cfg;
  SB_WR(0x15, 0x61);
  SB2_WR(0x00, 0x09);
  if ((PR(0x0763) | PR(0x0764)) != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x04);
  if (PR(0x07B9) != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x10);
  cfg = (uint8_t)(((PR(0x0750) & 0x0F) << 4)
                  | ((PR(0x0819) & 0x02) ? 0x02 : 0x00)
                  | (PR(0x0819) & 0x01));
  SB2_WR(0x01, cfg);
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x08);
  u4lb_d5da(0);
}

/* e9e7: RstRxpll — reset the RX PLL via C20E + the two CC10 settles. */
static void u4lb_e9e7(void) {
  uart_puts("\r\n[RstRxpll...]");
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  REG_PHY_RXPLL_RESET = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_RXPLL_RESET = 0x00;
  phy_cc10_cmd_wait(2, 0, 0x28);
  REG_CPU_CTRL_CC37 &= 0xFB;
  uart_puts("[Done]");
}

/* ebde: rate-lock settle — pulse C20F then spin (bounded) for the C2D0.5 / C350.5 lock bits. */
static void u4lb_ebde(void) {
  REG_PHY_CTRL_C20F = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_CTRL_C20F = 0x00;
  { uint16_t g = 0; while (((PR(0xC2D0) & 0x20) == 0) && ++g < 0x2000); }
  { uint16_t g = 0; while (((PR(0xC350) & 0x20) == 0) && ++g < 0x2000); }
}

/* e980: 20G rate-descriptor apply (C2A8/C328 + C2C9/C349 rate fields + START bit7). */
static void u4lb_e980(void) {
  PR(0xC2A8) &= 0x3F;
  PR(0xC328) &= 0x3F;
  u4lb_ebde();
  PR(0xC2A8) &= 0x3F;
  PR(0xC2C9) = (PR(0xC2C9) & 0x80)
             | (uint8_t)(((PR(0xC2EC) & 0x38) >> 3) | 0x40);
  PR(0xC328) &= 0x3F;
  PR(0xC349) = (PR(0xC349) & 0x80)
             | (uint8_t)(((PR(0xC36C) & 0x38) >> 3) | 0x40);
  PR(0xC2A8) = (PR(0xC2A8) & 0x3F) | 0x80;
  PR(0xC328) = (PR(0xC328) & 0x3F) | 0x80;
}

/* d3b0: Chg2 rate setup (rate=3=20G). SB[0x65] bit4=rate.0, bit5=rate.1; commit via CC10. */
static void u4lb_d3b0(uint8_t rate) {
  PR(0x0A5C) = rate;
  if (PR(0x0750) == 1) {
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);
    uart_puts("\r\nChg2 10G");
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);
  } else {
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);
    uart_puts("\r\nChg2 20G");
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);
  }
  phy_cc10_cmd_wait(2, 0, 0xC8);
}

/* ec51: Trig-arm — arm the lane-train trigger (CCE0/CCE1/CCE2/CCE3) that state 5 fires as [Trig]. */
static void u4lb_ec51(void) {
  REG_LANE_TRAIN_ARM = 0x04; REG_LANE_TRAIN_ARM = 0x02;
  REG_LANE_TRAIN_CTRL = (REG_LANE_TRAIN_CTRL & 0xF8) | 0x04;
  REG_LANE_TRAIN_MASK_LO = 0xFF; REG_LANE_TRAIN_MASK_HI = 0xFF;
  REG_LANE_TRAIN_ARM = 0x01;
  PR(0x0774) ^= 0x01;
}

/* b226: CC10 settle. */
static void u4lb_b226(void) { phy_cc10_cmd_wait(2, 0, 0xC8); }

/* ee57: fire ec51 Trig-arm when CCE1.0 is clear or CCE1.1 is set; the caller then reads the
 * read-only CCE4:CCE5 lane-width counter. */
static void u4lb_ee57(void) {
  if (!(REG_LANE_TRAIN_ARM & 0x01) || (REG_LANE_TRAIN_ARM & 0x02)) u4lb_ec51();
}

/* 98ec: lane-width snapshot producer — arm 0x0758=0x10, run ee57, latch CCE4:CCE5 into 0x768:0x769. */
static void u4lb_98ec(void) {
  PR(0x0758) = 0x10;
  u4lb_ee57();
  PR(0x0768) = REG_LANE_WIDTH_CNT_HI;
  PR(0x0769) = REG_LANE_WIDTH_CNT_LO;
}

/* State-4 PCIe-tunnel power / PHY bring-up (e305 -> ee29 -> ed44 -> df61 / a840 / c593 / b8db).
 * Every 0x1xxx/0x4xxx/0x5xxx/0x6xxx/0x7xxx address is a plane-2 access via P1_RD/P1_WR; bank0
 * CA06/CA81/C659/B40x/B43x/CCxx/E764 are plain XDATA via PR(). */

/* d195: P1[0x7104] = (P1[0x7104] & 0xBF) | 0x40. */
static void u4lb_d195(void) {
  P1_WR(0x7104, (uint8_t)((P1_RD(0x7104) & 0xBF) | 0x40));
}

/* d1d3(hi): returns (P1[hi:0x8D] & 0xF3) | 8. The high byte carries from the prior df61 access. */
static uint8_t u4lb_d1d3(uint16_t base_hi) {
  return (uint8_t)((P1_RD((uint16_t)((base_hi & 0xFF00) | 0x8D)) & 0xF3) | 0x08);
}

/* df61: the plane-2 PHY lane-block program. The 0x7041 read is a dead read whose discarded value
 * leaves the access pointer at 0x7041 for the following write. */
static void u4lb_df61(void) {
  uint8_t v;
  u4lb_d195();
  P1_WR(0x1808, 0x00);
  v = (uint8_t)((P1_RD(0x1835) & 0xFE) | 0x01);
  P1_WR(0x1835, v);
  (void)P1_RD(0x7041);
  P1_WR(0x7041, (uint8_t)(v | 0x40));
  P1_WR(0x6043, 0x70);
  P1_WR(0x6025, (uint8_t)((P1_RD(0x6025) & 0x7F) | 0x80));
  P1_WR(0x508F, 0x01);
  P1_WR(0x508D, u4lb_d1d3(0x5000));
  P1_WR(0x5204, (uint8_t)(P1_RD(0x5204) & 0xFE));
  P1_WR(0x5204, (uint8_t)(P1_RD(0x5204) & 0xFD));
  P1_WR(0x408D, u4lb_d1d3(0x4000));
}

/* ed44: B401/B402 tunnel-link strobe, then df61. */
static void u4lb_ed44(void) {
  PR(0xB401) = (uint8_t)((PR(0xB401) & 0xFE) | 0x01);
  PR(0xB401) = (uint8_t)((PR(0xB401) & 0xFD) | 0x02);
  PR(0xB401) &= 0xFE;
  PR(0xB401) &= 0xFD;
  PR(0xB402) = (uint8_t)((PR(0xB402) & 0xF7) | 0x08);
  PR(0xB402) &= 0xFD;
  u4lb_df61();
}

/* e74e: 0x0B1B=0; CCF8 &= ~0x10; CCF9=4; CCF9=2. */
static void u4lb_e74e(void) {
  PR(0x0B1B) = 0;
  REG_CPU_EXT_CTRL &= 0xEF;
  REG_CPU_EXT_STATUS = 0x04;
  REG_CPU_EXT_STATUS = 0x02;
}

/* ee29: C659&=~1; B402&=~1; ed44; e74e; 0x0B42=0; 0x0B43=0. */
static void u4lb_ee29(void) {
  REG_PCIE_LANE_CTRL_C659 &= 0xFE;
  REG_PCIE_CTRL_B402 &= 0xFE;
  u4lb_ed44();
  u4lb_e74e();
  PR(0x0B42) = 0;
  PR(0x0B43) = 0;
}

/* d702: CC10-mailbox lane-mask bit-distributor (plane-2 0x78AF..0x7BAF slot bit7 = mask.slot). */
static void u4lb_d702(uint8_t newmask) {
  __xdata uint8_t slot;
  slot = (uint8_t)(P1_RD(0x78AF) & 0x7F);
  P1_WR(0x78AF, (uint8_t)(((newmask & 0x01) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x79AF) & 0x7F);
  P1_WR(0x79AF, (uint8_t)(((newmask & 0x02) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x7AAF) & 0x7F);
  P1_WR(0x7AAF, (uint8_t)(((newmask & 0x04) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x7BAF) & 0x7F);
  P1_WR(0x7BAF, (uint8_t)(((newmask & 0x08) ? 0x80 : 0x00) | slot));
}

/* c089: the 4-round B434 lane ramp toward target, with a d702 + CC10 settle each round. */
static void u4lb_c089_lane_ramp(uint8_t target) {
  __xdata uint8_t curmask = (uint8_t)(PR(0xB434) & 0x0F);
  __xdata uint8_t roundbit = 0x01;
  __xdata uint8_t round = 0;
  do {
    __xdata uint8_t newmask;
    if (target < 0x0F) {
      if (curmask == target) return;
      newmask = (uint8_t)((uint8_t)(target | (uint8_t)(roundbit ^ 0x0F)) & curmask);
    } else {
      if (curmask == 0x0F) return;
      newmask = (uint8_t)(roundbit | curmask);
    }
    curmask = newmask;
    PR(0xB434) = (uint8_t)(newmask | (uint8_t)(PR(0xB434) & 0xF0));
    u4lb_d702(newmask);
    phy_cc10_cmd_wait(2, 0, 0xC7);
    roundbit = (uint8_t)(roundbit << 1);
    round++;
  } while (round < 4);
}

/* d436: PCIe-tunnel link-width config — bracket B402.1, run the lane ramp, strobe B401.0, set B436. */
static void u4lb_d436(uint8_t mask) {
  __xdata uint8_t saved_b402_1 = (uint8_t)(PR(0xB402) & 0x02);
  PR(0xB402) = (uint8_t)(PR(0xB402) & 0xFD);
  u4lb_c089_lane_ramp(mask);
  if (mask != 0x0F) {
    PR(0xB401) = (uint8_t)((PR(0xB401) & 0xFE) | 0x01);
    PR(0xB401) = (uint8_t)(PR(0xB401) & 0xFE);
  }
  if (saved_b402_1 != 0) PR(0xB402) = (uint8_t)((PR(0xB402) & 0xFD) | 0x02);
  PR(0xB436) = (uint8_t)((PR(0xB436) & 0xF0) | (uint8_t)(mask & 0x0E));
  PR(0xB436) = (uint8_t)((PR(0xB436) & 0x0F) | (uint8_t)(((uint8_t)(PR(0xB404) & 0x0F) ^ 0x0F) << 4));
}

/* a840 gen->speed table, indexed by 0x0A5D (gen). */
static __code const uint8_t u4lb_a840_speed_38cc[8] = { 0x02,0x01,0x03,0x01,0x03,0x01,0x03,0x02 };

/* a840: PCIe link-speed/width config (B403/B431 + d436 width). gen=0x0AEC, lane=0x0AED. */
static void u4lb_a840(uint8_t param) {
  uint8_t gen = PR(0x0AEC);
  uint8_t lane = PR(0x0AED);
  uint8_t usb4;
  uint8_t width_code;
  REG_CPU_CTRL_CA81 &= 0xFE;
  if (gen == 3 && lane == 3) {
    if ((PR(0x09FA) & 0x81) != 0) {
      uint8_t idx;
      PR(0x0A5D) = gen; idx = (uint8_t)(PR(0x0A5D) & 0x03);
      gen = u4lb_a840_speed_38cc[(uint8_t)(idx << 1)];
      lane = u4lb_a840_speed_38cc[(uint8_t)((idx << 1) + 1)];
      if (PR(0x07B9) != 0) lane = 1;
    } else {
      static __code const uint8_t t5d24[5] = { 0x00, 0x00, 0x02, 0x02, 0x02 };
      static __code const uint8_t t5d29[5] = { 0x01, 0x00, 0x00, 0x01, 0x02 };
      PR(0x0A5D) = gen;
      gen = t5d24[PR(0x0A5D)]; lane = t5d29[PR(0x0A5D)];
    }
  }
  usb4 = (uint8_t)(PR(0x09FA) & 0x81);
  if (usb4 == 0) {
    if (gen >= 3) {
      REG_CPU_MODE_NEXT &= 0x1F;
    } else if (lane < 2) {
      REG_CPU_MODE_NEXT = (uint8_t)((REG_CPU_MODE_NEXT & 0x1F) | 0x20);
    } else {
      REG_CPU_MODE_NEXT &= 0x1F;
    }
  } else {
    if (gen == 3) {
      REG_CPU_MODE_NEXT &= 0x1F;
    }
  }
  if (gen < 3) {
    PR(0xB403) = (uint8_t)((PR(0xB403) & 0xFE) | 0x01);
    P1_WR(0x40B0, (uint8_t)((uint8_t)(gen + 1) | (uint8_t)(P1_RD(0x40B0) & 0xF0)));
  } else {
    PR(0xB403) &= 0xFE;
    P1_WR(0x40B0, (uint8_t)((P1_RD(0x40B0) & 0xF0) | 0x04));
  }
  width_code = 0;
  if (lane < 3) {
    if (lane == 1) width_code = 0x0C;
    else if (lane == 0) width_code = 0x0E;
  }
  PR(0x0A5C) = width_code;
  PR(0xB431) = (uint8_t)((PR(0xB431) & 0xF0) | width_code);
  u4lb_d436(width_code);
  if ((uint8_t)(PR(0x09FA) & 0x81) != 0) u4lb_ed44();
  (void)param;
}

/* e305: state-4 PcieTunnel power-on prologue (gated (0x09FA & 0x81)): conditional CA06 mode-next
 * select, ee29, B402 &= ~2, a840. The E764 train and HDDPC strobe follow inline in b0b4. */
static void u4lb_e305(uint8_t param) {
  if ((PR(0x09FA) & 0x81) == 0) return;
  if ((PR(0x0AEC) == 3) ||
      (((PR(0x0750) != 2) || (PR(0x0751) != 0)) && (PR(0x0750) != 1))) {
    REG_CPU_MODE_NEXT &= 0x1F;
  } else {
    REG_CPU_MODE_NEXT = (uint8_t)((REG_CPU_MODE_NEXT & 0x1F) | 0x20);
  }
  u4lb_ee29();
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_a840(param);
}

/* c593: bank0 tunnel/PHY commit. e916 returns the plane-2 0x2805 read that seeds the 0x1335 RMWs. */
static uint8_t u4lb_e916(void) { return P1_RD(0x2805); }
static void u4lb_c593(void) {
  uint8_t v;
  PR(0xCCB0) = (uint8_t)((PR(0xCCB0) & 0xF8) | 0x05);
  PR(0xCCB2) = 0x00;
  PR(0xCCB3) = 0xC8;
  P1_WR(0x134D, 0x04);
  P1_WR(0x1334, 0x02);
  P1_WR(0x1335, 0x02);
  v = u4lb_e916();
  P1_WR(0x1335, (uint8_t)((v & 0xFE) | 0x01));
  if (PR(0x072D) == 0) {
    v = u4lb_e916();
    P1_WR(0x1335, (uint8_t)(v & 0xFD));
  } else {
    P1_WR(0x1335, (uint8_t)((P1_RD(0x1335) & 0xFD) | 0x02));
  }
  v = u4lb_e916();
  P1_WR(0x1335, (uint8_t)((v & 0xFB) | 0x04));
  P1_WR(0x1334, (uint8_t)((P1_RD(0x1334) & 0x7F) | 0x80));
  P1_WR(0x1285, (uint8_t)((P1_RD(0x1285) & 0x0F) | 0x30));
}

/* b8db: CDR/PLL validate loop. A prologue early-returns in the already-locked cases and otherwise
 * sets the per-lane margin window (lo/hi = CDR phase; lo52:lo54 / hi52:hi54 = a 16-bit eye margin),
 * then a bounded (<=10) loop polls bit6 PLL-lock plus the full CDR-margin compare and fires e9e7
 * (RxPLL reset) on any miss. The caller discards the return. */
static __xdata uint8_t b8db_lo, b8db_hi, b8db_lo52, b8db_hi52, b8db_lo54, b8db_hi54;
static void u4lb_b8db(void) {
  __xdata uint8_t phase0, phase1, margin0_lo, margin0_hi, margin1_lo, margin1_hi;
  uint8_t iter;
  if ((P1_RD(0x0000) & 0x02) == 0) {
    if ((PR(0x92F8) & 0x0C) == 0) return;
    b8db_lo = 0x01; b8db_hi = 0x28;
    b8db_lo52 = 0x01; b8db_hi52 = 0x3D; b8db_lo54 = 0x01; b8db_hi54 = 0x43;
  } else if (PR(0x0750) == 1) {
    if (PR(0xC297) & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x28;
    if (PR(0x07BA) == 0) { b8db_lo52=0x01; b8db_hi52=0x47; b8db_lo54=0x01; b8db_hi54=0x4D; }
    else { b8db_lo52=0x01; b8db_hi52=0x3D; b8db_lo54=0x01; b8db_hi54=0x43; }
  } else {
    if (PR(0xC2A7) & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x20;
    if (PR(0x07BA) != 0) { b8db_lo52=0x01; b8db_hi52=0x3E; b8db_lo54=0x01; b8db_hi54=0x42; }
    else { b8db_lo52=0x01; b8db_hi52=0x48; b8db_lo54=0x01; b8db_hi54=0x4C; }
  }
  for (iter = 0; iter < 10; iter++) {
    phase0 = PR(0xC2D2) & 0x3F; margin0_lo = PR(0xC2D9); margin0_hi = PR(0xC2DA);
    phase1 = PR(0xC352) & 0x3F; margin1_lo = PR(0xC359); margin1_hi = PR(0xC35A);
    if ((PR(0xC2D0) & 0x40) && (PR(0xC350) & 0x40) &&
        phase0 >= b8db_lo && phase0 <= b8db_hi && phase1 >= b8db_lo && phase1 <= b8db_hi &&
        (uint8_t)(b8db_lo52 - (margin0_hi <  b8db_lo54         ? 1 : 0)) <= margin0_lo &&
        margin0_lo <  (uint8_t)(b8db_hi52 - (margin0_hi < (uint8_t)(b8db_hi54 + 1) ? 1 : 0)) &&
        (uint8_t)(b8db_lo52 - (margin1_hi <  b8db_lo54         ? 1 : 0)) <= margin1_lo &&
        margin1_lo <  (uint8_t)(b8db_hi52 - (margin1_hi < (uint8_t)(b8db_hi54 + 1) ? 1 : 0)))
      return;
    u4lb_e9e7();
  }
}

/* b0b4 body — state-4, assembled in dependency order. */
static void u4lb_state4_b0b4(void) {
  uart_puts("[b4:A]");
  /* (A) entry / retrain guard: 0x0776 != 0 -> retrain {e07d; b226} x2 */
  if (PR(0x0776) != 0) {
    uint8_t i;
    for (i = 0; i < 2; i++) { u4lb_e07d(); u4lb_b226(); }
  } else {
    /* normal-connect OS-prewrite */
    if (PR(0x0819) & 0x01) {
      uint8_t op = (PR(0x0750) == 2) ? 0x85 : 0x81;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    if (PR(0x0819) & 0x02) {
      uint8_t op = (PR(0x0750) == 2) ? 0xA5 : 0xA1;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    u4lb_b226();
  }
  uart_puts("[b4:B]");

  /* (B) lane-width ready gate: (0x0768:0x0769) - (CCE4:CCE5) < 0x38 -> abort */
  { uint16_t width = ((uint16_t)PR(0x0768) << 8) | PR(0x0769);
    uint16_t neg   = ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
    uart_puts("[b4:wid="); uart_puthex(PR(0x0768)); uart_puthex(PR(0x0769));
    uart_puts(" neg="); uart_puthex(REG_LANE_WIDTH_CNT_HI); uart_puthex(REG_LANE_WIDTH_CNT_LO);
    uart_puts(" 765="); uart_puthex(PR(0x0765)); uart_puthex(PR(0x0766));
    uart_puts(" 819="); uart_puthex(PR(0x0819)); uart_puts(" 81A="); uart_puthex(PR(0x081A));
    uart_puts(" 77A="); uart_puthex(PR(0x077A)); uart_puts("]");
    if ((uint16_t)(width - neg) < 0x0038) { uart_puts("[b4:WIDGATE-abort]"); return; }
  }

  /* (C) connect-present gate: 0x0765==0 && 0x0766==0 -> abort */
  if (PR(0x0765) == 0 && PR(0x0766) == 0) { uart_puts("[b4:CONGATE-abort]"); return; }
  uart_puts("[b4:C]");

  /* E716/CA06 enable, gated 0x0AF1.0 */
  if (PR(0x0AF1) & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    phy_cc10_cmd_wait(2, 0, 0x28);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    REG_CPU_CTRL_CA81 &= 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  }

  /* e305: [PcieTunnel-PwrOn] + the cdc6 E764 0x14->0x19 PHY train (E764=0x19 is the precondition for
   * the host to move SB[0xA0]/[0xA1] 07->01->02), then the e26a HDDPC strobe. */
  uart_puts("[PcieTunnel-PwrOn]");
  u4lb_e305(1);
  if (PR(0x09FA) & 0x81) {
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xF7) | 0x08;
    REG_PHY_TIMER_CTRL_E764 &= 0xFB;
    REG_PHY_TIMER_CTRL_E764 &= 0xFE;
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xFD) | 0x02;
    phy_cc10_cmd_wait(1, 7, 0xCF);
    if (PR(0xE762) & 0x10) {
      REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xFE) | 0x01;
      REG_PHY_TIMER_CTRL_E764 &= 0xFD;
      PR(0x06E9) = 0;
    } else {
      REG_PHY_TIMER_CTRL_E764 &= 0xF7;
      REG_PHY_TIMER_CTRL_E764 &= 0xFB;
      REG_PHY_TIMER_CTRL_E764 &= 0xFE;
      REG_PHY_TIMER_CTRL_E764 &= 0xFD;
      PR(0x06E9) = 1;
    }
    REG_HDDPC_CTRL = (uint8_t)((REG_HDDPC_CTRL & 0xDF) | 0x20);
  }

  /* L0 OS-arm, gated 0x0819.0 */
  if (PR(0x0819) & 0x01) {
    u4lb_96fe(0x82);
    u4lb_d5da(1);
    PR(0x081E) = (PR(0x081E) & 0x7F) | 0x80;
  }
  /* L1 OS-arm, gated 0x0819.1 */
  if (PR(0x0819) & 0x02) {
    u4lb_96fe(0xA2);
    u4lb_d5da(1);
    PR(0x081F) = (PR(0x081F) & 0x7F) | 0x80;
  }

  uart_puts("[b4:D c2="); uart_puthex(PR(0xC2D0)); uart_puthex(PR(0xC350)); uart_putc(']');
  /* CC37.2 set -> d3b0(3) Chg2 20G -> e980 rate apply -> e9e7 RstRxpll -> CC37.2 clr */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  u4lb_d3b0(3);
  u4lb_e980();
  u4lb_e9e7();
  uart_puts("[c2@rstpll="); uart_puthex(PR(0xC2D0)); uart_puthex(PR(0xC350)); uart_putc(']');
  REG_CPU_CTRL_CC37 &= 0xFB;

  /* b8db: [CDRV ok] CDR/PLL validate loop */
  uart_puts("[CDRV ok]");
  u4lb_b8db();
  uart_puts("[c2@b8db="); uart_puthex(PR(0xC2D0)); uart_puthex(PR(0xC350)); uart_putc(']');

  /* CA60.3 set (tunnel-adapter enable) */
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;

  /* c593: bank0 tunnel/PHY commit */
  u4lb_c593();

  /* L0 OS1 trigger, gated 0x0819.0 */
  if (PR(0x0819) & 0x01) {
    uart_puts("[L0 OS1]");
    SB_WR(0x50, 0x02);
    P1_WR(0x010B, P1_RD(0x010B) | 0x01);
    PR(0x075B) = 0x10; PR(0x0759) = 0x10;
  } else {
    PR(0x075B) = 0x00; PR(0x0759) = 0x00;
  }
  /* L1 OS1 trigger, gated 0x0819.1 */
  if (PR(0x0819) & 0x02) {
    uart_puts("[L1 OS1]");
    SB_WR(0x5A, 0x02);
    P1_WR(0x010B, (P1_RD(0x010B) & 0xFD) | 0x02);
    PR(0x075C) = 0x10; PR(0x075A) = 0x10;
  } else {
    PR(0x075C) = 0x00; PR(0x075A) = 0x00;
  }

  /* ec51 Trig-arm */
  u4lb_ec51();
  uart_puts("[c2@ec51="); uart_puthex(PR(0xC2D0)); uart_puthex(PR(0xC350));
  uart_puts(" d1="); uart_puthex(PR(0xC2D1)); uart_puthex(PR(0xC351));
  uart_puts(" ab6="); uart_puthex(PR(0x0AB6)); uart_putc(']');

  /* latch negotiated width 0x074E:0x074F = CCE4:CCE5 */
  PR(0x074E) = REG_LANE_WIDTH_CNT_HI;
  PR(0x074F) = REG_LANE_WIDTH_CNT_LO;

  /* eb62(0,5) -> [SB P05] -> state 5 */
  u4lb_eb62(0, 5);
}

/* State 5 (0x06ED==5) — the CL-state lane-bond walker (8000 when 0x0718==4, else 850b): the
 * per-lane FSMs that drive SB[0xA0]/[0xA1] toward CL0(0x02) via the 0x0800-plane shadow + ea7c,
 * emitting Lx:CL0. Runs from the super-loop; all helpers are bounded. */

/* [S5:..] per-pass CL-state dump budget. */
static volatile uint8_t __xdata u4lb_s5_print_budget;

/* ee6e: per-lane SB connect-present = SB[lane?0x60:0x56].0. */
static uint8_t u4lb_ee6e(uint8_t lane) { return (uint8_t)(SB_RD(lane ? 0x60 : 0x56) & 0x01); }

/* eda0: route-special selector (0=eval-path, 1=idle, 2=route-special); clears 0x0775/0x0719. */
static uint8_t u4lb_eda0(void) {
  if (PR(0x0775) != 0) { PR(0x0775) = 0; PR(0x0719) = 0; return 0; }
  if (PR(0x0719) == 0x02) { PR(0x0719) = 0; return 2; }
  return 1;
}

/* e461: the SB-transport route push the walker depends on. Gated by the 0x0719 in-flight token;
 * the live path (0x0718==4) builds the e2b9/e1cb descriptor (SB[0x15] = 0x41 when 0x0776 set, else
 * 0x0718) and triggers the bounded TX. Returns 1 only when a push was issued. */
static uint8_t u4lb_e461(void) {
  uint16_t g;
  if (PR(0x0719) != 0) return 0;
  if (PR(0x0718) == 0) {
    SB_WR(0x04, (uint8_t)((SB_RD(0x04) & 0xFE) | 0x01));
    SB_WR(0x04, (uint8_t)((SB_RD(0x04) & 0xFD) | 0x02));
    PR(0x0AAB) = 0;
    return 1;
  }
  {
    PR(0x0AAB) = 0;
    PR(0x0AA8) = PR(0x0776) ? 0 : PR(0x0718);
    PR(0x0AA9) = 0x0D;
    PR(0x0AAA) = 0x04;
    sb_d4cd_transport_edges();
    SBTX_WR(0, PR(0x0AA9));
    SBTX_WR(1, (uint8_t)(PR(0x0AAA) | ((PR(0x0AAB) & 1) << 7)));
    SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08));
    SB_WR(0x15, PR(0x0776) ? (uint8_t)((PR(0x0AA8) << 1) | 0x41) : (uint8_t)PR(0x0AA8));
    PR(0x0AAC) = 0;
    P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));
    SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));
    uart_puts("[Te="); uart_puthex(SB_RD(0x15)); uart_putc(':'); uart_puthex(SBTX_RD(0)); uart_puthex(SBTX_RD(1));
    uart_puts(" 2c="); uart_puthex(SB_RD(0x2C)); uart_puts(" 0c="); uart_puthex(SB_RD(0x0C));
    uart_puts(" 18="); uart_puthex(SB_RD(0x18)); uart_puts(" 26="); uart_puthex(SB_RD(0x26));
    uart_puts(" rx="); { uint8_t i; for (i = 0; i < 6; i++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + i))); } uart_putc(']');
    SB_WR(0x10, 0x01);
    g = 0; while (((SB_RD(0x2C) >> 2) & 1) == 0 && ++g < 0x4000) { }
    SB_WR(0x2C, 0x04);
    phy_cc10_cmd(1, 0, 0x0B);
    SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));
    REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
    (void)SB_RD(0x0C);
    REG_XFER2_DMA_STATUS = 0x01;
    PR(0x0719) = 0x01;
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

/* lane gate: walk lane L iff 0x0819.L (lane-present). */
static uint8_t u4lb_lane_gate(uint8_t lane) { return (uint8_t)(PR(0x0819) & (uint8_t)(1u << lane)); }

/* [s5 ..] diagnostic — print only when a watched lane-state byte changes, to avoid saturating the
 * UART TX FIFO on a fast-iterating walker. */
static volatile uint8_t __xdata u4lb_s5_last759, u4lb_s5_last75b, u4lb_s5_seen, u4lb_s5_lasta0;
static volatile uint8_t __xdata u4lb_s5_last775, u4lb_s5_lastaf38, u4lb_s5_lasttx;
static void u4lb_s5_diag(void) {
  __xdata uint8_t state0, state1, sba0, host_desc, af, tx;
  state0 = PR(0x0759); state1 = PR(0x075B); sba0 = SB_RD(0xA0); host_desc = PR(0x0775);
  af = af38_t50; tx = SBTX_RD(4);
  if (u4lb_s5_seen && state0 == u4lb_s5_last759 && state1 == u4lb_s5_last75b && sba0 == u4lb_s5_lasta0
      && host_desc == u4lb_s5_last775 && af == u4lb_s5_lastaf38 && tx == u4lb_s5_lasttx) return;
  u4lb_s5_lastaf38 = af; u4lb_s5_lasttx = tx;
  if (!u4lb_s5_print_budget) return;
  u4lb_s5_print_budget--;
  u4lb_s5_seen = 1; u4lb_s5_last759 = state0; u4lb_s5_last75b = state1; u4lb_s5_lasta0 = sba0; u4lb_s5_last775 = host_desc;
  uart_puts("\r\n[s5 9="); uart_puthex(state0); uart_putc('/'); uart_puthex(state1);
  uart_puts(" A="); uart_puthex(sba0); uart_puthex(SB_RD(0xA1));
  uart_puts(" 775="); uart_puthex(host_desc);
  uart_puts(" E764="); uart_puthex(REG_PHY_TIMER_CTRL_E764); uart_puts(" E762="); uart_puthex(PR(0xE762));
  uart_puts(" ED="); uart_puthex(PR(0x06ED));
  uart_puts(" snap="); uart_puthex(PR(0x0779)); uart_puthex(PR(0x077A));
  uart_puts(" pll="); uart_puthex(PR(0xC2D0)); uart_puthex(PR(0xC350));
  uart_puts(" TX="); { uint8_t i; for (i = 0; i < 6; i++) uart_puthex(SBTX_RD(i)); }
  uart_puts(" w81C="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(PR((uint16_t)(0x081Cu + i))); }
  uart_puts(" 776="); uart_puthex(PR(0x0776));
  uart_puts(" af38["); uart_puthex(af38_t50); uart_putc('/'); uart_puthex(af38_t53); uart_putc('/'); uart_puthex(af38_t51); uart_putc('/'); uart_puthex(af38_t4f); uart_putc('/'); uart_puthex(af38_t6f0); uart_putc(']');
  uart_puts(" 6A="); uart_puthex(SB_RD(0x6A)); uart_puthex(SB_RD(0x6B)); uart_puthex(SB_RD(0x6C)); uart_puthex(SB_RD(0x6D));
  uart_putc(']');
}

/* 8501: a banked SB-transport drain/poll; non-load-bearing for progression (the ISR's d4cd handles
 * the edges), so it is a no-op here. */
static void u4lb_8501(void) { }

/* 81d4 finalize: width-settle -> advance state cell 0x0759+lane to 0x60; on counter overflow
 * (>=0x0F) reset to 0x00. Composes the device TX[2:3] CL-walk value from work[0x071A+walk_idx]. */
static void u4lb_lp1_finalize(uint8_t lane) {
  __xdata uint8_t walk_idx;
  if (PR(0x075F + lane) >= 0x0F) { PR(0x0759 + lane) = 0x00; return; }
  PR(0x075F + lane)++;
  walk_idx = (uint8_t)((PR(0x075D + lane) + 1) & 0x0F);
  PR(0x075D + lane) = walk_idx;
  PR(0x081C + lane) = (uint8_t)(PR((uint16_t)(0x071A + walk_idx)) | 0x10);
  PR(0x0800 + (uint8_t)(lane + walk_idx)) |= 0x80;
  PR(0x0759 + lane) = 0x60;
}

/* 8174 width-settle poll: advance to 0x60 (via finalize) when the negotiated width pair has settled
 * vs the read-only CCE4:CCE5 counter; else leave state 0x40 to retry. */
static void u4lb_lp1_width_settle(uint8_t lane) {
  __xdata uint16_t widthA, widthB, neg; __xdata uint8_t train_tgl;
  widthA = (uint16_t)(((uint16_t)PR(0x076C + 2*lane) << 8) | PR(0x076D + 2*lane));
  if (widthA == 0) { u4lb_lp1_finalize(lane); return; }
  widthB   = (uint16_t)(((uint16_t)PR(0x0770 + 2*lane) << 8) | PR(0x0771 + 2*lane));
  train_tgl = PR(0x0774);
  if ((uint8_t)widthB == train_tgl && (uint8_t)(widthB >> 8) == 0) { u4lb_lp1_finalize(lane); return; }
  neg = (uint16_t)(((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO);
  if ((uint16_t)((widthA - 1) - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
}

/* 8000: primary state-5 walker (0x0718==4). LOOP1 state@0x0759+lane (connect-arm/retrain edges),
 * LOOP2 state@0x075B+lane (the CL-state walk). Live LOOP1 chain from the b0b4 seed:
 * 0x10->0x30->0x50->0x70->0x90->0xA1 (bonded); 0xA1 is terminal. */
static void u4lb_walk_8000(void) {
  __xdata uint8_t lane, state;
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = PR(0x0759 + lane);

    if (state == 0x10) {
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x30;
    }
    else if (state == 0x20) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = PR(0x077B + lane);
        if ((snap & 0x80) == 0) {
          PR(0x0759 + lane) = 0x20;
        } else {
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          PR(0x076C + 2*lane) = 0x00;
          PR(0x076D + 2*lane) = 0x00;
          PR(0x081C + lane) |= 0x20;
          PR(0x0759 + lane) = 0x40;
        }
        u4lb_8501();
      } else if (selector == 2) {
        PR(0x0759 + lane) = 0x20;
      }
    }
    else if (state == 0x30) {
      PR(0x075F + lane) = 0x00;
      PR(0x0759 + lane) = 0x50;
    }
    else if (state == 0x40) {
      if (u4lb_ee6e(lane) != 0 && PR(0x075F + lane) != 0) {
        if (PR(0x0AB3) == 0) {
          SB_WR(lane ? 0x5A : 0x50, 0x01);
        }
        PR(0x081C + lane) |= 0x40;
        PR(0x081C + lane) = 0x00;
        if ((REG_PHY_ORIENT_C2C3 & 0x01) || (REG_VENDOR_CTRL_C343 & 0x01)) {
          if ((PR(0x0819) & 0x03) != 0) {
            PR(0x081C + lane) = (uint8_t)((PR(0x081C + lane) | 0x40) & 0x7F);
            PR(0x081D + lane) = (uint8_t)(((PR(0x081D + lane) | 0x10) + 1) & 0x7F);
          }
        }
      } else {
        u4lb_lp1_width_settle(lane);
      }
    }
    else if (state == 0x50) {
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x70;
    }
    else if (state == 0x60) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = PR(0x077B + lane);
        if ((snap & 0xC0) == 0xC0 &&
            (snap & 0x0F) == (uint8_t)(PR(0x081C + lane) & 0x0F)) {
          if (PR(0x0AB3)) u4lb_e9e7();
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          u4lb_ee57();
          PR(0x076C + 2*lane) = REG_LANE_WIDTH_CNT_HI;
          PR(0x076D + 2*lane) = REG_LANE_WIDTH_CNT_LO;
          PR(0x0770 + 2*lane) = 0x00;
          PR(0x0771 + 2*lane) = PR(0x0774);
          PR(0x0759 + lane) = 0x80;
        } else {
          PR(0x0759 + lane) = 0x60;
        }
        u4lb_8501();
      } else if (selector == 2) {
        PR(0x0759 + lane) = 0x60;
      }
    }
    else if (state == 0x70) {
      if (u4lb_ee6e(lane)) {
        PR(0x081C + lane) |= 0x10;
        PR(0x081C + lane) |= 0x40;
        PR(0x0819 + lane) &= 0x7F;
      }
      PR(0x0759 + lane) = 0x90;
    }
    else if (state == 0x80) {
      if (u4lb_ee6e(lane)) {
        PR(0x081C + lane) |= 0x40;
      }
      PR(0x081C + lane) &= 0x7F;
      PR(0x0759 + lane) = 0xA0;
    }
    else if (state == 0x90) {
      if (u4lb_e461() == 1) PR(0x0759 + lane) = 0xA1;
    }
    else if (state == 0xA0) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((PR(0x077B + lane) & 0xC0) == 0x80) PR(0x0759 + lane) = 0x50;
        else                                    PR(0x0759 + lane) = 0xA0;
        u4lb_8501();
      } else if (selector == 2) {
        PR(0x0759 + lane) = 0xA0;
      }
    }
    else {
      PR(0x081C + lane) &= 0xEF;
      PR(0x081C + lane) &= 0x7F;
      PR(0x0819 + lane) &= 0xDF;
      PR(0x0759 + lane) = 0x20;
    }
  }
  /* LOOP2: the CL-state walker (state @0x075B+lane). */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = PR(0x075B + lane);
    if (state == 0x10) {
      PR(0x081E + lane) |= 0x80;
      PR(0x081E + lane) &= 0xBF;
      PR(0x075B + lane) = 0x20;
    } else if (state == 0x20) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x30;
    } else if (state == 0x30) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = PR(0x0779 + lane);
        if ((snap >> 4) & 1) PR(0x075B + lane) = 0x00;
        else if (((snap >> 7) & 1) == 0) PR(0x075B + lane) = 0x20;
        else {
          __xdata uint8_t cl_idx = (uint8_t)(snap & 0x0F), cap, cl_cfg_hi = 0, cl_cfg_lo = 0;
          PR(0x0761 + lane) = cl_idx;
          PR(0x081E + lane) &= 0xF0;
          PR(0x081E + lane) |= cl_idx;
          PR(0x081E + lane) |= 0x40;
          cap = PR(0x0AB4 + lane);
          if ((cap >> 1) & 1) cl_cfg_lo = PR(0x072E + cl_idx);
          if (cap & 1) { __xdata uint16_t m = (uint16_t)(PR(0x073E + cl_idx) * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
          uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
          uart_puthex(cl_cfg_hi); uart_puthex(cl_cfg_lo);
          SB_WR(0x6A + 2 * lane, cl_cfg_hi);
          SB_WR(0x6B + 2 * lane, cl_cfg_lo);
          u4lb_ea7c(cl_idx, lane);
          PR(0x075B + lane) = 0x50;
        }
      } else if (selector == 2) PR(0x075B + lane) = 0x20;
    } else if (state == 0x50) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x60;
    } else if (state == 0x60) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((PR(0x0779 + lane) >> 7) & 1) PR(0x075B + lane) = 0x50;
        else { PR(0x081E + lane) &= 0xBF; PR(0x075B + lane) = 0x20; }
      } else if (selector == 2) PR(0x075B + lane) = 0x50;
    }
  }
}

/* 850b: alternate state-5 walker (0x0718 != 4). Dead on the live AMD path; kept for completeness. */
static void u4lb_walk_850b(void) {
  __xdata uint8_t lane, state, selector = 0;
  /* LOOP1: state @0x075B+lane. */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = PR(0x075B + lane);
    if (state == 0x11) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) PR(0x075B + lane) = 0x20;
      else if (r == 1) PR(0x075B + lane) = 0x10;
    } else if (state == 0x20) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) { if (PR(0x0779) == 0) PR(0x075B + lane) = 0x30; else PR(0x075B + lane) = 0x20; }
      else if (r != 2) PR(0x075B + lane) = 0x20;
    } else if (state == 0x21) {
      if (u4lb_ee6e(lane) == 0) PR(0x075B + lane) = 0x40;
    } else if (state == 0x30) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = PR(0x0B26 + lane);
        if (!((snap >> 4) & 1)) PR(0x075B + lane) = 0x00;
        else if ((snap >> 7) & 1) { PR(0x0761 + lane) = (uint8_t)(snap & 0x0F); PR(0x075B + lane) = 0x50; }
        else PR(0x075B + lane) = 0x30;
      }
    } else if (state == 0x40) {
      __xdata uint8_t cap = PR(0x0AB4 + lane), cl_idx = PR(0x0761 + lane), cl_cfg_hi = 0, cl_cfg_lo = 0;
      if ((cap >> 1) & 1) cl_cfg_lo = PR(0x072E + cl_idx);
      if (cap & 1) { __xdata uint16_t m = (uint16_t)(PR(0x073E + cl_idx) * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
      uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
      uart_puthex(cl_cfg_hi); uart_puthex(cl_cfg_lo);
      u4lb_ea7c(cl_idx, lane);
      PR(0x075B + lane) = 0x51;
    } else if (state == 0x50) {
      PR(0x075B + lane) = 0x60;
    } else if (state == 0x51) {
      __xdata uint8_t v;
      PR(0x0B2C + lane) |= 0x80;
      v = (uint8_t)((PR(0x0B2C + lane) & 0xF0) | PR(0x0761 + lane));
      PR(0x0B2C + lane) = v;
      if (v == 0) PR(0x075B + lane) = 0x61;
    } else if (state == 0x60) {
      if (u4lb_e461() == 1) { if (PR(0x0779) == 0) PR(0x075B + lane) = 0x70; else PR(0x075B + lane) = 0x60; }
    } else if (state == 0x61) {
      if (u4lb_e461() == 1) PR(0x075B + lane) = 0x71;
    } else if (state == 0x70) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = PR(0x0B26 + lane);
        if (!((snap >> 7) & 1)) PR(0x075B + lane) = 0x30;
        else if (PR(0x0761 + lane) != 0x07) PR(0x075B + lane) = 0x30;
        else PR(0x075B + lane) = 0x70;
      }
    } else {
      if (u4lb_ee6e(lane) == 0) PR(0x075B + lane) = 0x11;
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
      state = PR(0x0759 + lane);
      if (state == 0x10) {
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x21;
      } else if (state == 0x20) {
        if (u4lb_e461() == 1) {
          if ((PR(0x0B28 + lane) >> 7) & 1) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); PR(0x0759 + lane) = 0x30; }
          else PR(0x0759 + lane) = 0x20;
        }
      } else if (state == 0x21) {
        if (u4lb_eda0() != 0) PR(0x081C + lane) |= 0x10;
        PR(0x0759 + lane) = 0x40;
      } else if (state == 0x30) {
        if (u4lb_eda0() != 0) {
          if (PR(0x075F + lane) != 0) PR(0x0759 + lane) = 0x50;
          else { uart_puts("\r\n(lim)"); PR(0x0759 + lane) = 0x60; }
        }
      } else if (state == 0x40) {
        uart_puts("EQ");
        SB_WR(0x50, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1));
        PR(0x0759 + lane) = 0x51;
      } else if (state == 0x50) {
        PR(0x0B2A + lane) |= 0x10;
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0x52;
      } else if (state == 0x51) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) != 0) PR(0x0759 + lane) = 0x51; else { uart_puts("\r\n(lim)"); PR(0x075B + lane) = 0x00; } }
        else if (r != 2) PR(0x0759 + lane) = 0x51;
      } else if (state == 0x52) {
        PR(0x075F + lane)++;
        PR(0x075D + lane) = (uint8_t)((PR(0x075D + lane) + 1) & 0x0F);
        PR(0x0759 + lane) = 0x70;
      } else if (state == 0x60) {
        __xdata uint8_t walk_idx = PR(0x075D + lane);
        PR(0x071A + walk_idx) |= 0xA0;
        PR(0x0B2A + lane) = walk_idx;
        if (walk_idx == 0) PR(0x0759 + lane) = 0x80;
      } else if (state == 0x70) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) == 0) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); PR(0x0759 + lane) = 0x90; } else PR(0x0759 + lane) = 0x70; }
        else if (r != 2) PR(0x0759 + lane) = 0x70;
      } else if (state == 0x80) {
        PR(0x0759 + lane) = 0xA0;
      } else if (state == 0x90) {
        PR(0x0B2A + lane) &= 0x7F;
        if (u4lb_e461() == 1) PR(0x0759 + lane) = 0xA1;
      } else if (state == 0xA0) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (PR(0x0779) == 0) PR(0x0759 + lane) = 0xB0; else PR(0x0759 + lane) = 0xA0; }
        else if (r != 2) PR(0x0759 + lane) = 0xA0;
      } else if (state == 0xA1) {
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
  (void)selector;
}

/* u4lb_state5(): e672 state-5 entry. 0x0718==4 -> 8000 else 850b. */
static void u4lb_state5(void) {
  DPX = 0x00;
  u4lb_s5_diag();
  if (PR(0x0718) == 0x04) u4lb_walk_8000();
  else                    u4lb_walk_850b();
  DPX = 0x00;
}

/* e672 — the lane-bond FSM dispatcher, called from cb10's tail (gated 0x06ED!=0).
 *   3 -> cm_conn_routing_setup; 4 -> b0b4; 5 -> finalise when all sub-lane states clear, else the walker. */
static void u4lb_e672(void) {
  uint8_t state = PR(0x06ED);
  if (state == 0x04) {
    u4lb_state4_b0b4();
    return;
  }
  if (state == 0x05) {
    if (PR(0x075B) == 0 && PR(0x0759) == 0 && PR(0x075C) == 0 && PR(0x075A) == 0) {
      u4lb_eb62(0, 0);
      return;
    }
    u4lb_state5();
    return;
  }
  if (state == 0x03) {
    u4lb_cm_conn_routing_setup();
    return;
  }
}

#endif /* USB4_LANEBOND_H */
