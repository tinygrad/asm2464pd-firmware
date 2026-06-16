#ifndef USB4_LANEBOND_H
#define USB4_LANEBOND_H
/*
 * USB4 lane-bond / CL0 / PCIe-tunnel bring-up FSM. State lives in XDATA 0x06ED and is driven from
 * the super-loop via e672: 3 -> connection-routing setup, 4 -> b0b4 tunnel power-on / lane-bond,
 * 5 -> the per-lane CL-state walker (8000/850b). Include after usb4_connect.h.
 */

#define U4LB_STATE   u4_fsm_state

/* eb62(p1,p2): set the new FSM state, print "[SB P0<state>]", and store it into 0x06ED. */
static void u4lb_eb62(uint8_t state_lo, uint8_t state) {
  sb_tx_go_param = state_lo;
  sb_fsm_state = state;
  uart_puts("\r\n[SB P0");
  uart_puthex(state);
  uart_putc(']');
  u4_fsm_state = sb_fsm_state;
}

/* [EDF5] dump budget (XDATA, seeded in main()). */
static volatile uint8_t __xdata __at(0x0B58) u4lb_edf5_print_budget;

/* edf5 -> e2b9: the device->host SB-transport route-query that prompts the host to post the
 * connection-routing descriptor. Gated by the 0x0719 in-flight token; d5da's poll is bounded so the
 * super-loop can't hang. Returns 1 only when a query was actually sent. */
static uint8_t u4lb_edf5_route_query(void) {
  uint8_t staged;
  if (e461_inflight_token != 0) return 0;

  sb_tx_flag = 0;
  sb_tx_cmd = 5;
  sb_tx_byte0 = 0x0C;
  sb_tx_byte1 = 3;
  sb_d4cd_transport_edges();

  SBTX_WR(0, sb_tx_byte0);
  SBTX_WR(1, (uint8_t)(sb_tx_byte1 | ((sb_tx_flag & 1) << 7)));
  staged = (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08);
  SB_WR(0x0C, staged);

  SB_WR(0x15, sb_tx_cmd);

  /* d5da(0): the bounded SB-transport TX trigger. */
  { uint8_t tx_done; uint16_t g = 0;
    sb_tx_go_param = 0;
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
      uart_putc('|'); uart_puthex(e461_inflight_token); uart_putc(' '); uart_puthex(cm_conn_routing_substate);
      uart_putc(' '); uart_puthex(u4_fsm_state); uart_putc(' '); uart_puthex(sb_transport_edge_toggle);
      uart_putc(' '); uart_puthex(u4_route_query_response); uart_putc(' '); uart_puthex(u4_coldboot_seed_gate);
      uart_putc(' '); uart_puthex(u4_host_desc[0x0]);
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
  e461_inflight_token = 0x01;
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
  uint8_t state = cm_conn_routing_substate;
  if (state == 0x10) {
    if (u4lb_edf5_route_query() != 1) return;
    cm_conn_routing_substate = 0x11;
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
    if (u4_route_query_response != 0)      { u4_route_query_response = 0; e461_inflight_token = 0; selector = 0; }
    else if (e461_inflight_token == 0x02) { e461_inflight_token = 0; selector = 2; }
    else                       { selector = 1; }
    if (selector == 2) { cm_conn_routing_substate = 0x10; return; }
    if (selector != 0) { return; }
  }

  /* mode==0 path: gate on the host connect descriptor 0x0777==0x0C. */
  if (u4_host_desc[0x0] != 0x0C) { cm_conn_routing_substate = 0x10; return; }

  { static __xdata uint8_t conn_routing_diag_budget = 6;
    if (conn_routing_diag_budget) { conn_routing_diag_budget--;
      uart_puts("\r\n[cr B9="); uart_puthex(u4_connect_route_latch); uart_puts(" 778="); uart_puthex(u4_host_desc[0x1]);
      uart_puts(" 81B="); uart_puthex(u4_work_buf[0x1B]); uart_puts(" CE="); uart_puthex(u4_confirm_input_ce);
      uart_puts(" CD="); uart_puthex(u4_confirm_input_cd); uart_puts(" 776="); uart_puthex(u4_coldboot_seed_gate); uart_putc(']'); } }

  /* 0x0776 connect-confirm computation. */
  if (u4_connect_route_latch != 0) {
    uint8_t host_status = u4_host_desc[0x1];
    if (((host_status & 0x7F) == 2) || ((u4_work_buf[0x1B] & 1) == 0) ||
        (u4_confirm_input_ce != 0 && u4_confirm_input_cd == 0)) {
      u4_coldboot_seed_gate = 0;
    } else {
      SB_CLR(0xED, 0x80);
    }
  }
  /* e391 width-LUT seed (gated 0x0776==0): the per-descriptor LUT af38 ORs into SBTX[1]. */
  if (u4_coldboot_seed_gate == 0) {
    uint8_t i;
    for (i = 0; i < 0x13; i++) {            /* e391 LOOP1: 0x06F2+i / 0x0705+i width+gate LUT */
      sb_width_lut[(uint16_t)(0x0 + i)] = u4lb_width_lut_514c[i];
      sb_branchA_gate[(uint16_t)(0x0 + i)] = u4lb_branchA_gate_515f[i];
    }
    /* e391 LOOP2 (e3c3-e3d3): d221 zeroes XDATA[0x0B26..0x0B2D] = the CL-walk shadows, on EVERY
     * conn-routing seed. Latent on cold boot (SDCC zeroes XDATA) but the live AMD path is a connect
     * STORM: on re-connect the walker would otherwise read STALE lb_cl_status/lb_eq_status/
     * lb_loop2_scratch/lb_cl0_width and poison the snap&0x80/&0x10 CL-walk gates. */
    for (i = 0; i < 2; i++) {
      lb_cl_status[i] = 0; lb_eq_status[i] = 0; lb_loop2_scratch[i] = 0; lb_cl0_width[i] = 0;
    }
  }

  /* [ConnRout] confirm print + 0x0718 ROUTE-ENABLE. */
  if (u4_coldboot_seed_gate == 0 && u4_confirm_input_ce != 0) {
    uart_puts("[ConnRtmr]");
    u4_route_enable_latch = 0;
  } else {
    uart_puts("[ConnRout]");
    u4_route_enable_latch = 4;
  }

  /* Latch the 0x077a lane-width bits into 0x0819/0x0751/0x0750. */
  { uint8_t host_width = u4_host_desc[0x3];
    if ((host_width & 1) && (u4_work_buf[0x1A] & 1)) { u4_work_buf[0x19] = (u4_work_buf[0x19] & 0xFE) | 1; }
    if ((host_width & 0x02) && (u4_work_buf[0x1A] & 2)) { u4_work_buf[0x19] = (u4_work_buf[0x19] & 0xFD) | 2; }
    if ((host_width & 0x10) && (u4_work_buf[0x1A] & 0x10) && (u4_work_buf[0x19] & 1) && (u4_work_buf[0x19] & 2)) lb_lane_width_latch1 = 1;
    else lb_lane_width_latch1 = 0;
    if ((host_width & 0x20) && (u4_work_buf[0x1A] & 0x20)) lb_lane_width_latch0 = 2;
    uart_puts("[Lt77A="); uart_puthex(host_width); uart_puts(" 81A="); uart_puthex(u4_work_buf[0x1A]);
    uart_puts(" 819="); uart_puthex(u4_work_buf[0x19]); uart_puts("]");
    u4_phy_gate_a = 0; u4_phy_gate_b = 0;
  }

  /* c586: negotiated-rate descriptor (SB[0x6A-0x6D]/[0x74-0x75]) + Gen2 lane-eq retrim. */
  {
    uint16_t rate = (uint16_t)((uint16_t)sb_lane_flip[0xB] * 0x20);
    uint8_t rate_hi = (uint8_t)(rate >> 8);
    uint8_t rate_lo = (uint8_t)((rate & 0xFF) | lb_cap_field[0xB]);
    SB_WR(0x6A, rate_hi); SB_WR(0x6B, rate_lo);
    SB_WR(0x6C, rate_hi); SB_WR(0x6D, rate_lo);
    SB_WR(0x74, 0x00); SB_WR(0x75, (uint8_t)((lb_lane_width_latch0 == 2) ? 0x1F : 0x0F));
    if (REG_LANE_RATE_C8FF == 0x04) {
      if (u4_enter_usb_accepted != 0) {
        /* c2c6 (stock c5d2->c5db RETs here): force the advertised PHY rate-field low nibble to 0x02.
         * Omitting this left C294/C314 at the boot default 0x07 (usb4_irq C2E0 RMW(0xF0,0x07)) -> the
         * host trained/granted the higher-rate CL index 0x0D instead of stock's 0x0C. */
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x02);
      } else {
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x03);
        REG_PHY_LANEA_C293 = (uint8_t)((REG_PHY_LANEA_C293 & 0xFC) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x03);
        REG_PHY_LANEB_C313 = (uint8_t)((REG_PHY_LANEB_C313 & 0xFC) | 0x02);
      }
    }
    if (lb_lane_width_latch0 == 1) {
      REG_PHY_LANEA_C2C5 = (uint8_t)((REG_PHY_LANEA_C2C5 & 0xF0) | 0x0F);
      REG_PHY_LANEB_C345 = (uint8_t)((REG_PHY_LANEB_C345 & 0xF0) | 0x0F);
    }
  }

  cm_conn_routing_substate = 0;
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
  sb_tx_go_param = param;
  if (param == 1) {
    SB_WR(0x0F, (SB_RD(0x0F) & 0xFE) | 0x01);
  }
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);
  SB_WR(0x04, SB_RD(0x04) & 0xFD);
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
  if ((u4_phy_gate_a | u4_phy_gate_b) != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x04);
  if (u4_connect_route_latch != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x10);
  cfg = (uint8_t)(((lb_lane_width_latch0 & 0x0F) << 4)
                  | ((u4_work_buf[0x19] & 0x02) ? 0x02 : 0x00)
                  | (u4_work_buf[0x19] & 0x01));
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

/* cdc6 E764 RX-PLL train (stock CODE:cdc6): ramp E764 (set bit3, clear 2/1/0, set bit1), cc10 settle,
 * then poll E762.4 (the RX-PLL "trained/ready" latch). If set -> finish E764->0x19 (set bit0, clear
 * bit1) and clear the busy flag 0x06E9; else clear E764 bits 3-0 and leave busy=1. Re-driven per
 * state-5 walker pass while busy so E762.4 can latch under the host's LIVE lane-training stimulus
 * (stock runs it in state-4; the handmade got E762=00 once and never re-drove it -> the host's lane
 * adapters loop Training/Bonding and never reach CL0). */
static void u4lb_e764_rxpll_train(void) {
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xF7) | 0x08);
  REG_PHY_TIMER_CTRL_E764 &= 0xFB;
  REG_PHY_TIMER_CTRL_E764 &= 0xFE;
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFD) | 0x02);
  phy_cc10_cmd_wait(1, 7, 0xCF);
  if (REG_PHY_RXPLL_STATUS & 0x10) {
    REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFE) | 0x01);
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
    phy_rxpll_train_busy = 0;
  } else {
    REG_PHY_TIMER_CTRL_E764 &= 0xF7;
    REG_PHY_TIMER_CTRL_E764 &= 0xFB;
    REG_PHY_TIMER_CTRL_E764 &= 0xFE;
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
    phy_rxpll_train_busy = 1;
  }
}

/* ebde: rate-lock settle — pulse C20F then spin (bounded) for the C2D0.5 / C350.5 lock bits. */
static void u4lb_ebde(void) {
  REG_PHY_CTRL_C20F = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_CTRL_C20F = 0x00;
  { uint16_t g = 0; while (((REG_PHY_LANEA_LOCK_C2D0 & 0x20) == 0) && ++g < 0x2000); }
  { uint16_t g = 0; while (((REG_PHY_LANEB_LOCK_C350 & 0x20) == 0) && ++g < 0x2000); }
}

/* e980: 20G rate-descriptor apply (C2A8/C328 + C2C9/C349 rate fields + START bit7). */
static void u4lb_e980(void) {
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  u4lb_ebde();
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEA_RATE_DESC_C2C9 = (REG_PHY_LANEA_RATE_DESC_C2C9 & 0x80)
             | (uint8_t)(((REG_PHY_LANEA_RATE_SRC_C2EC & 0x38) >> 3) | 0x40);
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  REG_PHY_LANEB_RATE_DESC_C349 = (REG_PHY_LANEB_RATE_DESC_C349 & 0x80)
             | (uint8_t)(((REG_PHY_LANEB_RATE_SRC_C36C & 0x38) >> 3) | 0x40);
  REG_PHY_LANEA_RATE_START_C2A8 = (REG_PHY_LANEA_RATE_START_C2A8 & 0x3F) | 0x80;
  REG_PHY_LANEB_RATE_START_C328 = (REG_PHY_LANEB_RATE_START_C328 & 0x3F) | 0x80;
}

/* d3b0: Chg2 rate setup (rate=3=20G). SB[0x65] bit4=rate.0, bit5=rate.1; commit via CC10. */
static void u4lb_d3b0(uint8_t rate) {
  u4lb_width_rate_code = rate;
  if (lb_lane_width_latch0 == 1) {
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
  u4_lane_train_trigger ^= 0x01;
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
  cm_conn_routing_substate = 0x10;
  u4lb_ee57();
  lb_lane_width_cnt_hi = REG_LANE_WIDTH_CNT_HI;
  lb_lane_width_cnt_lo = REG_LANE_WIDTH_CNT_LO;
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
  REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01);
  REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFD) | 0x02);
  REG_PCIE_TUNNEL_CTRL &= 0xFE;
  REG_PCIE_TUNNEL_CTRL &= 0xFD;
  REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xF7) | 0x08);
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_df61();
}

/* e74e: 0x0B1B=0; CCF8 &= ~0x10; CCF9=4; CCF9=2. */
static void u4lb_e74e(void) {
  cc_subdemux_src = 0;
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
  __xdata uint8_t curmask = (uint8_t)(REG_PCIE_LINK_STATE & 0x0F);
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
    REG_PCIE_LINK_STATE = (uint8_t)(newmask | (uint8_t)(REG_PCIE_LINK_STATE & 0xF0));
    u4lb_d702(newmask);
    phy_cc10_cmd_wait(2, 0, 0xC7);
    roundbit = (uint8_t)(roundbit << 1);
    round++;
  } while (round < 4);
}

/* d436: PCIe-tunnel link-width config — bracket B402.1, run the lane ramp, strobe B401.0, set B436. */
static void u4lb_d436(uint8_t mask) {
  __xdata uint8_t saved_b402_1 = (uint8_t)(REG_PCIE_CTRL_B402 & 0x02);
  REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
  u4lb_c089_lane_ramp(mask);
  if (mask != 0x0F) {
    REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01);
    REG_PCIE_TUNNEL_CTRL = (uint8_t)(REG_PCIE_TUNNEL_CTRL & 0xFE);
  }
  if (saved_b402_1 != 0) REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFD) | 0x02);
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0xF0) | (uint8_t)(mask & 0x0E));
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0x0F) | (uint8_t)(((uint8_t)(REG_PCIE_LINK_PARAM_B404 & 0x0F) ^ 0x0F) << 4));
}

/* a840 gen->speed table, indexed by 0x0A5D (gen). */
static __code const uint8_t u4lb_a840_speed_38cc[8] = { 0x02,0x01,0x03,0x01,0x03,0x01,0x03,0x02 };

/* a840: PCIe link-speed/width config (B403/B431 + d436 width). gen=0x0AEC, lane=0x0AED. */
static void u4lb_a840(uint8_t param) {
  uint8_t gen = u4_link_gen;
  uint8_t lane = u4_link_lane;
  uint8_t usb4;
  uint8_t width_code;
  REG_CPU_CTRL_CA81 &= 0xFE;
  if (gen == 3 && lane == 3) {
    if ((u4_route_mode & 0x81) != 0) {
      uint8_t idx;
      u4lb_gen_index = gen; idx = (uint8_t)(u4lb_gen_index & 0x03);
      gen = u4lb_a840_speed_38cc[(uint8_t)(idx << 1)];
      lane = u4lb_a840_speed_38cc[(uint8_t)((idx << 1) + 1)];
      if (u4_connect_route_latch != 0) lane = 1;
    } else {
      static __code const uint8_t t5d24[5] = { 0x00, 0x00, 0x02, 0x02, 0x02 };
      static __code const uint8_t t5d29[5] = { 0x01, 0x00, 0x00, 0x01, 0x02 };
      u4lb_gen_index = gen;
      gen = t5d24[u4lb_gen_index]; lane = t5d29[u4lb_gen_index];
    }
  }
  usb4 = (uint8_t)(u4_route_mode & 0x81);
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
    REG_TUNNEL_CTRL_B403 = (uint8_t)((REG_TUNNEL_CTRL_B403 & 0xFE) | 0x01);
    P1_WR(0x40B0, (uint8_t)((uint8_t)(gen + 1) | (uint8_t)(P1_RD(0x40B0) & 0xF0)));
  } else {
    REG_TUNNEL_CTRL_B403 &= 0xFE;
    P1_WR(0x40B0, (uint8_t)((P1_RD(0x40B0) & 0xF0) | 0x04));
  }
  width_code = 0;
  if (lane < 3) {
    if (lane == 1) width_code = 0x0C;
    else if (lane == 0) width_code = 0x0E;
  }
  u4lb_width_rate_code = width_code;
  REG_TUNNEL_LINK_STATUS = (uint8_t)((REG_TUNNEL_LINK_STATUS & 0xF0) | width_code);
  u4lb_d436(width_code);
  if ((uint8_t)(u4_route_mode & 0x81) != 0) u4lb_ed44();
  (void)param;
}

/* e305: state-4 PcieTunnel power-on prologue (gated (0x09FA & 0x81)): conditional CA06 mode-next
 * select, ee29, B402 &= ~2, a840. The E764 train and HDDPC strobe follow inline in b0b4. */
static void u4lb_e305(uint8_t param) {
  if ((u4_route_mode & 0x81) == 0) return;
  if ((u4_link_gen == 3) ||
      (((lb_lane_width_latch0 != 2) || (lb_lane_width_latch1 != 0)) && (lb_lane_width_latch0 != 1))) {
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
  REG_TUNNEL_PHY_CFG_CCB0 = (uint8_t)((REG_TUNNEL_PHY_CFG_CCB0 & 0xF8) | 0x05);
  REG_TUNNEL_PHY_CFG_CCB2 = 0x00;
  REG_TUNNEL_PHY_TIMER_CCB3 = 0xC8;
  P1_WR(0x134D, 0x04);
  P1_WR(0x1334, 0x02);
  P1_WR(0x1335, 0x02);
  v = u4lb_e916();
  P1_WR(0x1335, (uint8_t)((v & 0xFE) | 0x01));
  if (lb_lane_bonded_flag == 0) {
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
  } else if (lb_lane_width_latch0 == 1) {
    if (REG_PHY_LANEA_LOCK_C297 & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x28;
    if (u4_enter_usb_accepted == 0) { b8db_lo52=0x01; b8db_hi52=0x47; b8db_lo54=0x01; b8db_hi54=0x4D; }
    else { b8db_lo52=0x01; b8db_hi52=0x3D; b8db_lo54=0x01; b8db_hi54=0x43; }
  } else {
    if (REG_PHY_LANEA_LOCK_C2A7 & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x20;
    if (u4_enter_usb_accepted != 0) { b8db_lo52=0x01; b8db_hi52=0x3E; b8db_lo54=0x01; b8db_hi54=0x42; }
    else { b8db_lo52=0x01; b8db_hi52=0x48; b8db_lo54=0x01; b8db_hi54=0x4C; }
  }
  for (iter = 0; iter < 10; iter++) {
    phase0 = REG_PHY_LANEA_MARGIN_PHASE_C2D2 & 0x3F; margin0_lo = REG_PHY_LANEA_MARGIN_EYE_C2D9; margin0_hi = REG_PHY_LANEA_MARGIN_EYE_C2DA;
    phase1 = REG_PHY_LANEB_MARGIN_PHASE_C352 & 0x3F; margin1_lo = REG_PHY_LANEB_MARGIN_EYE_C359; margin1_hi = REG_PHY_LANEB_MARGIN_EYE_C35A;
    if ((REG_PHY_LANEA_LOCK_C2D0 & 0x40) && (REG_PHY_LANEB_LOCK_C350 & 0x40) &&
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
  if (u4_coldboot_seed_gate != 0) {
    uint8_t i;
    for (i = 0; i < 2; i++) { u4lb_e07d(); u4lb_b226(); }
  } else {
    /* normal-connect OS-prewrite */
    if (u4_work_buf[0x19] & 0x01) {
      uint8_t op = (lb_lane_width_latch0 == 2) ? 0x85 : 0x81;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    if (u4_work_buf[0x19] & 0x02) {
      uint8_t op = (lb_lane_width_latch0 == 2) ? 0xA5 : 0xA1;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    u4lb_b226();
  }
  uart_puts("[b4:B]");

  /* (B) lane-width ready gate: (0x0768:0x0769) - (CCE4:CCE5) < 0x38 -> abort */
  { uint16_t width = ((uint16_t)lb_lane_width_cnt_hi << 8) | lb_lane_width_cnt_lo;
    uint16_t neg   = ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
    uart_puts("[b4:wid="); uart_puthex(lb_lane_width_cnt_hi); uart_puthex(lb_lane_width_cnt_lo);
    uart_puts(" neg="); uart_puthex(REG_LANE_WIDTH_CNT_HI); uart_puthex(REG_LANE_WIDTH_CNT_LO);
    uart_puts(" 765="); uart_puthex(sb_connect_present); uart_puthex(sb_route_up_trigger);
    uart_puts(" 819="); uart_puthex(u4_work_buf[0x19]); uart_puts(" 81A="); uart_puthex(u4_work_buf[0x1A]);
    uart_puts(" 77A="); uart_puthex(u4_host_desc[0x3]); uart_puts("]");
    if ((uint16_t)(width - neg) < 0x0038) { uart_puts("[b4:WIDGATE-abort]"); return; }
  }

  /* (C) connect-present gate: 0x0765==0 && 0x0766==0 -> abort */
  if (sb_connect_present == 0 && sb_route_up_trigger == 0) { uart_puts("[b4:CONGATE-abort]"); return; }
  uart_puts("[b4:C]");

  /* E716/CA06 enable, gated 0x0AF1.0 */
  if (u4_connect_gate & 0x01) {
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
  if (u4_route_mode & 0x81) {
    u4lb_e764_rxpll_train();   /* cdc6 E764 RX-PLL train (now re-driven per state-5 pass while busy) */
    REG_HDDPC_CTRL = (uint8_t)((REG_HDDPC_CTRL & 0xDF) | 0x20);
  }

  /* L0 OS-arm, gated 0x0819.0 */
  if (u4_work_buf[0x19] & 0x01) {
    u4lb_96fe(0x82);
    u4lb_d5da(1);
    u4_work_buf[0x1E] = (u4_work_buf[0x1E] & 0x7F) | 0x80;
  }
  /* L1 OS-arm, gated 0x0819.1 */
  if (u4_work_buf[0x19] & 0x02) {
    u4lb_96fe(0xA2);
    u4lb_d5da(1);
    u4_work_buf[0x1F] = (u4_work_buf[0x1F] & 0x7F) | 0x80;
  }

  uart_puts("[b4:D c2="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350); uart_putc(']');
  /* CC37.2 set -> d3b0(3) Chg2 20G -> e980 rate apply -> e9e7 RstRxpll -> CC37.2 clr */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  u4lb_d3b0(3);
  u4lb_e980();
  u4lb_e9e7();
  uart_puts("[c2@rstpll="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350); uart_putc(']');
  REG_CPU_CTRL_CC37 &= 0xFB;

  /* b8db: [CDRV ok] CDR/PLL validate loop */
  uart_puts("[CDRV ok]");
  u4lb_b8db();
  uart_puts("[c2@b8db="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350); uart_putc(']');

  /* CA60.3 set (tunnel-adapter enable) */
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;

  /* c593: bank0 tunnel/PHY commit */
  u4lb_c593();

  /* L0 OS1 trigger, gated 0x0819.0 */
  if (u4_work_buf[0x19] & 0x01) {
    uart_puts("[L0 OS1]");
    SB_WR(0x50, 0x02);
    P1_WR(0x010B, P1_RD(0x010B) | 0x01);
    lb_loop2_state[0x0] = 0x10; lb_loop1_state[0x0] = 0x10;
  } else {
    lb_loop2_state[0x0] = 0x00; lb_loop1_state[0x0] = 0x00;
  }
  /* L1 OS1 trigger, gated 0x0819.1 */
  if (u4_work_buf[0x19] & 0x02) {
    uart_puts("[L1 OS1]");
    SB_WR(0x5A, 0x02);
    P1_WR(0x010B, (P1_RD(0x010B) & 0xFD) | 0x02);
    lb_loop2_state[0x1] = 0x10; lb_loop1_state[0x1] = 0x10;
  } else {
    lb_loop2_state[0x1] = 0x00; lb_loop1_state[0x1] = 0x00;
  }

  /* ec51 Trig-arm */
  u4lb_ec51();
  uart_puts("[c2@ec51="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350);
  uart_puts(" d1="); uart_puthex(REG_PHY_LANEA_STATUS_C2D1); uart_puthex(REG_PHY_LANEB_STATUS_C351);
  uart_puts(" ab6="); uart_puthex(phy_cdr_arm_mask); uart_putc(']');

  /* latch negotiated width 0x074E:0x074F = CCE4:CCE5 */
  lb_laneA_cl0_latch = REG_LANE_WIDTH_CNT_HI;
  lb_laneB_cl0_latch = REG_LANE_WIDTH_CNT_LO;

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
  if (u4_route_query_response != 0) { u4_route_query_response = 0; e461_inflight_token = 0; return 0; }
  if (e461_inflight_token == 0x02) { e461_inflight_token = 0; return 2; }
  return 1;
}

/* e1cb/e2b9: the SB-transport descriptor builder (CODE_BANK1::e1cb is the 0x0776!=0 live AMD path,
 * e2b9 the 0x0776==0 path). Byte-true transcription:
 *   sb_tx_cmd  (0x0AA8) = byte0   sb_tx_byte0 (0x0AA9) = byte1   sb_tx_byte1 (0x0AAA) = flag3
 *   d4cd(); 997e -> SBTX[0]=byte1; 9923 -> SBTX[1]=flag3|((0xAAB.0)<<7);
 *   SB[0x0C] form: (0xAAB!=0) ? (byte1+8)|(SB0C&0x80) : (SB0C&0x80)|0x08;
 *   e1cb: SB[0x15] = (byte0<<1)|0x41 (via 972a);  e2b9: SB[0x15] = byte0 (via 96f7);
 *   d5da(0); 0x0719 = 0xFF (inflight token). */
static void u4lb_e1cb_e2b9(uint8_t is_e1cb) {
  uint8_t aab = sb_tx_flag;          /* XDATA[0x0AAB] */
  uint8_t form;
  sb_d4cd_transport_edges();         /* d4cd */
  SBTX_WR(0, sb_tx_byte0);           /* 997e: SBTX[0] = XDATA[0x0AA9] (=byte1) */
  SBTX_WR(1, (uint8_t)(sb_tx_byte1 | ((aab & 1) << 7)));  /* 9923: SBTX[1] = XDATA[0x0AAA] | ((0xAAB.0)<<7) */
  if (aab != 0)
    form = (uint8_t)(((sb_tx_byte0 + 8) & 0xFF) | (SB_RD(0x0C) & 0x80));  /* 99ac: (byte1+8)|SB0C.7 */
  else
    form = (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08);                        /* 9695: (SB0C.7)|0x08 */
  SB_WR(0x0C, form);
  if (is_e1cb)
    SB_WR(0x15, (uint8_t)((sb_tx_cmd << 1) | 0x41));   /* e1cb 972a: SB[0x15]=(byte0<<1)|0x41 */
  else
    SB_WR(0x15, sb_tx_cmd);                            /* e2b9 96f7: SB[0x15]=byte0 */
  u4lb_d5da(0);                                        /* d5da(0) TX trigger */
  /* 97ef tail: CCD9 strobe (4,2) then DEC A=1 -> CCD9=1 and 0x0719=1 (inflight token). */
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
  e461_inflight_token = 0x01;                          /* 0x0719 = 1 */
}

/* e461: the SB-transport route push the walker depends on (CODE_BANK1::e461). Gated by the 0x0719
 * in-flight token. Live AMD path: 0x0718(route_enable)=4 -> e487; 0x0776=1 -> e1cb with byte0=0,
 * byte1=0x0D, flag3=4 (9966 seeds R5=0x0D,R3=4,R7=0). Returns 1 only when a push was issued. */
static uint8_t u4lb_e461(void) {
  if (e461_inflight_token != 0) return 0;            /* XDATA[0x0719] in-flight */
  if (u4_route_enable_latch == 0) {
    /* 0x0718==0: 9960 seeds byte0=route|1, R5=0x0D, R3=4, 0xAAB=0; then e2b9. (Dead on AMD.) */
    sb_tx_flag = 0;                                  /* 9966: XDATA[0x0AAB]=0 */
    sb_tx_cmd  = (uint8_t)(u4_route_enable_latch | 0x01);  /* 9960: byte0 = route|1 = 1 */
    sb_tx_byte0 = 0x0D;                              /* R5 */
    sb_tx_byte1 = 0x04;                              /* R3 */
    u4lb_e1cb_e2b9(0);                               /* e2b9 */
    return 1;
  }
  if (u4_coldboot_seed_gate != 0) {
    /* 0x0718!=0 && 0x0776!=0 -> e1cb (THE live AMD CL-walk push). */
    sb_tx_flag = 0;                                  /* 9966: XDATA[0x0AAB]=0 */
    sb_tx_cmd  = 0x00;                               /* 9966 returns A=0 -> byte0 = 0 */
    sb_tx_byte0 = 0x0D;                              /* 9966: R5 = 0x0D */
    sb_tx_byte1 = 0x04;                              /* 9966: R3 = 0x04 */
    u4lb_e1cb_e2b9(1);                               /* e1cb */
    return 1;
  }
  /* 0x0718!=0 && 0x0776==0 -> e499: 9960(0x0718) + e2b9. */
  sb_tx_flag = 0;
  sb_tx_cmd  = (uint8_t)(u4_route_enable_latch | 0x01);  /* 9960: byte0 = route|1 */
  sb_tx_byte0 = 0x0D;
  sb_tx_byte1 = 0x04;
  u4lb_e1cb_e2b9(0);                                 /* e2b9 */
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

/* lane gate: walk lane L iff 0x0819.L (lane-present mask = 1<<lane). */
/* Stock 8000 head (8006-801e): ROM[0x21AD]=0x19 -> 96a7 reads XDATA[0x0819]=work[0x19] (=R5);
 * 9a11 sets A=1, R0 = (lane_counter 0x21)+1; then `SJMP 8015` jumps straight to `DJNZ R0,8010`
 * which executes the RLC-left body (R0-1) times (the initial SJMP skips one body pass). Net shift
 * of A=1 is therefore `lane`, NOT `lane+1`: lane0 mask = 1<<0 = 0x01 (bit0), lane1 mask = 1<<1 =
 * 0x02 (bit1). (801e ANL/ORL/JNZ then gates on work[0x19] & mask.) With work[0x19]=0x03 BOTH lanes
 * walk -> the stock 2-lane bond (work[0x081C]=work[0x081D]=0x7B, host posts 2487 lane0=24/lane1=87,
 * SB[0xA0]=SB[0xA1]->0x02). The earlier 1<<(lane+1) read mis-counted the DJNZ-after-SJMP loop and
 * forced a lane0-only walk (work[0x081D]=0x00) so the host never 2-lane bonded. HW-confirmed via the
 * stock |w= work-shadow dump (w=7B7B... 2-lane) vs the lane0-only handmade (w=7B00...). */
static uint8_t u4lb_lane_gate(uint8_t lane) { return (uint8_t)(u4_work_buf[0x19] & (uint8_t)(1u << lane)); }

/* [s5 ..] diagnostic — print only when a watched lane-state byte changes, to avoid saturating the
 * UART TX FIFO on a fast-iterating walker. */
static volatile uint8_t __xdata u4lb_s5_last759, u4lb_s5_last75b, u4lb_s5_seen, u4lb_s5_lasta0;
static volatile uint8_t __xdata u4lb_s5_last775, u4lb_s5_lastaf38, u4lb_s5_lasttx;
static void u4lb_s5_diag(void) {
  __xdata uint8_t state0, state1, sba0, host_desc, af, tx;
  state0 = lb_loop1_state[0x0]; state1 = lb_loop2_state[0x0]; sba0 = SB_RD(0xA0); host_desc = u4_route_query_response;
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
  uart_puts(" 719="); uart_puthex(e461_inflight_token);
  uart_puts(" E764="); uart_puthex(REG_PHY_TIMER_CTRL_E764); uart_puts(" E762="); uart_puthex(REG_PHY_RXPLL_STATUS);
  uart_puts(" 6E9="); uart_puthex(phy_rxpll_train_busy);
  uart_puts(" ED="); uart_puthex(u4_fsm_state);
  uart_puts(" hd="); { uint8_t i; for (i = 2; i < 7; i++) uart_puthex(u4_host_desc[i]); }
  uart_puts(" snap="); uart_puthex(u4_host_desc[0x2]); uart_puthex(u4_host_desc[0x3]);
  uart_puts(" pll="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350);
  uart_puts(" TX="); { uint8_t i; for (i = 0; i < 6; i++) uart_puthex(SBTX_RD(i)); }
  uart_puts(" w81C="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(u4_work_buf[(uint16_t)(0x1C + i)]); }
  uart_puts(" 776="); uart_puthex(u4_coldboot_seed_gate);
  uart_puts(" af38["); uart_puthex(af38_t50); uart_putc('/'); uart_puthex(af38_t53); uart_putc('/'); uart_puthex(af38_t51); uart_putc('/'); uart_puthex(af38_t4f); uart_putc('/'); uart_puthex(af38_t6f0); uart_putc(']');
  uart_puts(" 6A="); uart_puthex(SB_RD(0x6A)); uart_puthex(SB_RD(0x6B)); uart_puthex(SB_RD(0x6C)); uart_puthex(SB_RD(0x6D));
  uart_putc(']');
}

/* 8501: a banked SB-transport drain/poll; non-load-bearing for progression (the ISR's d4cd handles
 * the edges), so it is a no-op here. */
static void u4lb_8501(void) { }

/* 81d4 finalize: width-settle -> advance state cell 0x0759+lane to 0x60; on counter overflow
 * (>=0x0F) reset to 0x00. Composes the device TX[2:3] CL-walk value: the low nibble walks the
 * lane-descriptor ROM (sb_lane_desc) while the high nibble of work[0x081C+lane] is PRESERVED, then
 * bit7 is latched (81f8 969e read, 81fb ANL #0xf0, 8208 ORL, 8211 write; 8215 96a7 |0x80 write). */
static void u4lb_lp1_finalize(uint8_t lane) {
  __xdata uint8_t walk_idx;
  if (lb_settle_counter[lane] >= 0x0F) { lb_loop1_state[lane] = 0x00; return; }
  lb_settle_counter[lane]++;
  walk_idx = (uint8_t)((lb_lane_desc_idx[lane] + 1) & 0x0F);
  lb_lane_desc_idx[lane] = walk_idx;
  u4_work_buf[0x1C + lane] = (uint8_t)((u4_work_buf[0x1C + lane] & 0xF0) | sb_lane_desc[(uint16_t)(0x0 + walk_idx)]);
  u4_work_buf[0x1C + lane] |= 0x80;
  lb_loop1_state[lane] = 0x60;
}

/* 8174 width-settle poll: advance to 0x60 (via finalize) when the negotiated width pair has settled
 * vs the read-only CCE4:CCE5 counter; else leave state 0x40 to retry. */
static void u4lb_lp1_width_settle(uint8_t lane) {
  __xdata uint16_t widthA, widthB, neg; __xdata uint8_t train_tgl;
  widthA = (uint16_t)(((uint16_t)lb_width_pairA[2*lane] << 8) | lb_width_pairA[0x1 + 2*lane]);
  if (widthA == 0) { u4lb_lp1_finalize(lane); return; }
  widthB   = (uint16_t)(((uint16_t)lb_width_pairB[2*lane] << 8) | lb_width_pairB[0x1 + 2*lane]);
  train_tgl = u4_lane_train_trigger;
  if ((uint8_t)widthB == train_tgl && (uint8_t)(widthB >> 8) == 0) { u4lb_lp1_finalize(lane); return; }
  neg = (uint16_t)(((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO);
  if ((uint16_t)((widthA - 1) - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
}

/* stock 8000: the e461-bearing dispatches (84f3/84fa @8069/8251/831a/83bd/84a3) advance on the
 * e461 PUSH RESULT. 84f3/84fa = `LCALL e461; MOV A,R7; XRL #1; RET`. e461 CLOBBERS R7 -- it sets
 * R7=#0 when the 0x0719 in-flight token is set (e467, early RET) and R7=#1 on every push (e4a3),
 * so 84f3 returns push^1 and the caller's JZ advances exactly when e461 ISSUED A PUSH. That is
 * `if (u4lb_e461()==1)`. (The Ghidra decompile `return param_1^1` is an ARTIFACT: param_1 maps to
 * R7, which e461 overwrites -- verified in stock_ghidra_export.c e461 @50792 + disasm e467/e4a3.)
 * A prior session mis-read this as a 1<<lane lane-status gate, which killed lane1 (mask 0x02 != 1);
 * reverted to the byte-true e461-result gate. */

/* 8000: primary state-5 walker (0x0718==4). LOOP1 state@0x0759+lane (connect-arm/retrain edges),
 * LOOP2 state@0x075B+lane (the CL-state walk). LOOP1 dispatches through the 0def jump table @802a
 * = [target_hi,target_lo,match]: 0x10->804f 0x20->8069 0x30->807a 0x40->80ca 0x50->80d7 0x60->8251
 * 0x70->8262 0x80->82d5 0x90->82fa 0xA0->831a 0xA1->8327 default->8355. Linear chain from the b0b4
 * seed (0x10): 0x10->0x20->0x30->0x40->0x50(finalize: one TX[2:3] walk step)->0x60->0x70->0x80->
 * 0x90->0xA0->0xA1; the host re-train (8327 snap&0xC0==0x80) re-enters 0x50 for each walk step. */
static void u4lb_walk_8000(void) {
  __xdata uint8_t lane, state;
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop1_state[lane];

    if (state == 0x10) {
      /* 804f: width-settle init (3x mask on work[0x081C+lane]) -> 0x20 (unconditional, no e461). */
      u4_work_buf[0x1C + lane] &= 0xEF;
      u4_work_buf[0x1C + lane] &= 0x7F;
      u4_work_buf[0x1C + lane] &= 0xDF;
      lb_loop1_state[lane] = 0x20;
    }
    else if (state == 0x20) {
      /* 8069 LCALL 84f3: advance on the e461 push result (R7=1 push / 0 in-flight). */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x30;
    }
    else if (state == 0x30) {
      /* 807a: route-special selector; on snap.7 set, arm the lane (SB[0x40]) + work[0x1C]|=0x20
       * -> 0x40, else stay 0x20. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0x80) == 0) {
          lb_loop1_state[lane] = 0x20;
        } else {
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          lb_width_pairA[2*lane] = 0x00;
          lb_width_pairA[0x1 + 2*lane] = 0x00;
          u4_work_buf[0x1C + lane] |= 0x20;
          lb_loop1_state[lane] = 0x40;
        }
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = 0x20;
      }
    }
    else if (state == 0x40) {
      /* 80ca: clear width-settle counter -> 0x50. */
      lb_settle_counter[lane] = 0x00;
      lb_loop1_state[lane] = 0x50;
    }
    else if (state == 0x50) {
      /* 80d7: first entry (counter==0) -> width-settle/finalize = the TX[2:3] CL-walk step;
       * re-entry (ee6e && counter) -> work[0x1C]|=0x10|=0x40, reset state cell to 0x00. */
      if (u4lb_ee6e(lane) != 0 && lb_settle_counter[lane] != 0) {
        if (phy_lane_gate == 0) {
          SB_WR(lane ? 0x5A : 0x50, 0x01);
        }
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        lb_loop1_state[lane] = 0x00;
        if ((REG_PHY_ORIENT_C2C3 & 0x01) || (REG_VENDOR_CTRL_C343 & 0x01)) {
          if ((u4_work_buf[0x19] & 0x03) != 0) {
            /* 813c-816b: work_buf[0x1C] and [0x1D] are LANE-INDEPENDENT here (9997=0x1C, 9916=0x1D,
             * no lane add), each |=0x10 |=0x40 &=0x7F. The handmade's `+lane`, the dropped |0x10/|0x40,
             * and the bogus data `+1` (a stock index-arith artifact) were all transcription bugs. */
            u4_work_buf[0x1C] |= 0x10; u4_work_buf[0x1C] |= 0x40; u4_work_buf[0x1C] &= 0x7F;
            u4_work_buf[0x1D] |= 0x10; u4_work_buf[0x1D] |= 0x40; u4_work_buf[0x1D] &= 0x7F;
          }
        }
      } else {
        u4lb_lp1_width_settle(lane);
      }
    }
    else if (state == 0x60) {
      /* 8251 LCALL 84f3: advance on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x70;
    }
    else if (state == 0x70) {
      /* 8262: snap&0xC0==0xC0 + low-nibble match -> arm width pairs -> 0x80, else stay 0x60. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0xC0) == 0xC0 &&
            (snap & 0x0F) == (uint8_t)(u4_work_buf[0x1C + lane] & 0x0F)) {
          if (phy_lane_gate) u4lb_e9e7();
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          u4lb_ee57();
          lb_width_pairA[2*lane] = REG_LANE_WIDTH_CNT_HI;
          lb_width_pairA[0x1 + 2*lane] = REG_LANE_WIDTH_CNT_LO;
          lb_width_pairB[2*lane] = 0x00;
          lb_width_pairB[0x1 + 2*lane] = u4_lane_train_trigger;
          lb_loop1_state[lane] = 0x80;
        } else {
          lb_loop1_state[lane] = 0x60;
        }
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = 0x60;
      }
    }
    else if (state == 0x80) {
      /* 82d5: ee6e -> work[0x1C] |=0x10 |=0x40 &=0x7F (all gated) -> 0x90. */
      if (u4lb_ee6e(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        u4_work_buf[0x1C + lane] &= 0x7F;
      }
      lb_loop1_state[lane] = 0x90;
    }
    else if (state == 0x90) {
      /* 82fa: ee6e -> work[0x1C] |=0x10 |=0x40; then work[0x1C] &=0x7F (unconditional) -> 0xA0. */
      if (u4lb_ee6e(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
      }
      u4_work_buf[0x1C + lane] &= 0x7F;
      lb_loop1_state[lane] = 0xA0;
    }
    else if (state == 0xA0) {
      /* 831a LCALL 84fa: advance to 0xA1 (lane bonded) on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = 0xA1;
    }
    else if (state == 0xA1) {
      /* 8327: bonded monitor; host re-train request (snap&0xC0==0x80) re-enters 0x50, else 0xA0. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((u4_host_desc[0x4 + lane] & 0xC0) == 0x80) lb_loop1_state[lane] = 0x50;
        else                                    lb_loop1_state[lane] = 0xA0;
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = 0xA0;
      }
    }
    /* default (state 0x00 / unmatched): stock 8355 tail is a no-op (lane parked). */
  }
  /* LOOP2: the CL-state walker (state @0x075B+lane). */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == 0x10) {
      u4_work_buf[0x1E + lane] |= 0x80;
      u4_work_buf[0x1E + lane] &= 0xBF;
      lb_loop2_state[lane] = 0x20;
    } else if (state == 0x20) {
      /* 83bd LCALL 84fa: advance on the e461 push result (waits here while 0x0719 in-flight, which
       * is the intended one-push-at-a-time serialization -- the cycle is unblocked by the host ack
       * clearing 0x0719 via eda0, not by forcing the advance). */
      if (u4lb_e461() == 1) lb_loop2_state[lane] = 0x30;
    } else if (state == 0x30) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x2 + lane];
        if ((snap >> 4) & 1) lb_loop2_state[lane] = 0x00;
        else if (((snap >> 7) & 1) == 0) lb_loop2_state[lane] = 0x20;
        else {
          __xdata uint8_t cl_idx = (uint8_t)(snap & 0x0F), cap, cl_cfg_hi = 0, cl_cfg_lo = 0;
          lb_cl_value[lane] = cl_idx;
          u4_work_buf[0x1E + lane] &= 0xF0;
          u4_work_buf[0x1E + lane] |= cl_idx;
          u4_work_buf[0x1E + lane] |= 0x40;
          cap = phy_lane_cap[lane];
          if ((cap >> 1) & 1) cl_cfg_lo = lb_cap_field[cl_idx];
          if (cap & 1) { __xdata uint16_t m = (uint16_t)(sb_lane_flip[cl_idx] * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
          uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
          uart_puthex(cl_cfg_hi); uart_puthex(cl_cfg_lo);
          SB_WR(0x6A + 2 * lane, cl_cfg_hi);
          SB_WR(0x6B + 2 * lane, cl_cfg_lo);
          u4lb_ea7c(cl_idx, lane);
          lb_loop2_state[lane] = 0x50;
        }
      } else if (selector == 2) lb_loop2_state[lane] = 0x20;
    } else if (state == 0x50) {
      /* 84a3 LCALL e461; MOV A,R7; XRL #1; JNZ stay: advance on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop2_state[lane] = 0x60;
    } else if (state == 0x60) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((u4_host_desc[0x2 + lane] >> 7) & 1) lb_loop2_state[lane] = 0x50;
        else { u4_work_buf[0x1E + lane] &= 0xBF; lb_loop2_state[lane] = 0x20; }
      } else if (selector == 2) lb_loop2_state[lane] = 0x50;
    }
  }
}

/* 850b: alternate state-5 walker (0x0718 != 4). Dead on the live AMD path; kept for completeness. */
static void u4lb_walk_850b(void) {
  __xdata uint8_t lane, state, selector = 0;
  /* LOOP1: state @0x075B+lane. */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == 0x11) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) lb_loop2_state[lane] = 0x20;
      else if (r == 1) lb_loop2_state[lane] = 0x10;
    } else if (state == 0x20) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x30; else lb_loop2_state[lane] = 0x20; }
      else if (r != 2) lb_loop2_state[lane] = 0x20;
    } else if (state == 0x21) {
      if (u4lb_ee6e(lane) == 0) lb_loop2_state[lane] = 0x40;
    } else if (state == 0x30) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 4) & 1)) lb_loop2_state[lane] = 0x00;
        else if ((snap >> 7) & 1) { lb_cl_value[lane] = (uint8_t)(snap & 0x0F); lb_loop2_state[lane] = 0x50; }
        else lb_loop2_state[lane] = 0x30;
      }
    } else if (state == 0x40) {
      __xdata uint8_t cap = phy_lane_cap[lane], cl_idx = lb_cl_value[lane], cl_cfg_hi = 0, cl_cfg_lo = 0;
      if ((cap >> 1) & 1) cl_cfg_lo = lb_cap_field[cl_idx];
      if (cap & 1) { __xdata uint16_t m = (uint16_t)(sb_lane_flip[cl_idx] * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
      uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
      uart_puthex(cl_cfg_hi); uart_puthex(cl_cfg_lo);
      u4lb_ea7c(cl_idx, lane);
      lb_loop2_state[lane] = 0x51;
    } else if (state == 0x50) {
      lb_loop2_state[lane] = 0x60;
    } else if (state == 0x51) {
      __xdata uint8_t v;
      lb_cl0_width[lane] |= 0x80;
      v = (uint8_t)((lb_cl0_width[lane] & 0xF0) | lb_cl_value[lane]);
      lb_cl0_width[lane] = v;
      if (v == 0) lb_loop2_state[lane] = 0x61;
    } else if (state == 0x60) {
      if (u4lb_e461() == 1) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x70; else lb_loop2_state[lane] = 0x60; }
    } else if (state == 0x61) {
      if (u4lb_e461() == 1) lb_loop2_state[lane] = 0x71;
    } else if (state == 0x70) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 7) & 1)) lb_loop2_state[lane] = 0x30;
        else if (lb_cl_value[lane] != 0x07) lb_loop2_state[lane] = 0x30;
        else lb_loop2_state[lane] = 0x70;
      }
    } else {
      if (u4lb_ee6e(lane) == 0) lb_loop2_state[lane] = 0x11;
    }
  }
  /* width-limit one-shot + LOOP2 (state @0x0759+lane). */
  if (lb_loop2_state[0x0] == 0 && lb_loop2_state[0x1] == 0) {
    if (lb_walk_oneshot_flag == 0) {
      if (u4_work_buf[0x19] & 0x01) u4lb_8992(0x86);
      if ((u4_work_buf[0x19] >> 1) & 0x01) u4lb_8992(0xA6);
      lb_walk_oneshot_flag = 1;
    }
    for (lane = 0; lane < 2; lane++) {
      if (!u4lb_lane_gate(lane)) continue;
      state = lb_loop1_state[lane];
      if (state == 0x10) {
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x21;
      } else if (state == 0x20) {
        if (u4lb_e461() == 1) {
          if ((lb_eq_status[lane] >> 7) & 1) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x30; }
          else lb_loop1_state[lane] = 0x20;
        }
      } else if (state == 0x21) {
        if (u4lb_eda0() != 0) u4_work_buf[0x1C + lane] |= 0x10;
        lb_loop1_state[lane] = 0x40;
      } else if (state == 0x30) {
        if (u4lb_eda0() != 0) {
          if (lb_settle_counter[lane] != 0) lb_loop1_state[lane] = 0x50;
          else { uart_puts("\r\n(lim)"); lb_loop1_state[lane] = 0x60; }
        }
      } else if (state == 0x40) {
        uart_puts("EQ");
        SB_WR(0x50, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1));
        lb_loop1_state[lane] = 0x51;
      } else if (state == 0x50) {
        lb_loop2_scratch[lane] |= 0x10;
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x52;
      } else if (state == 0x51) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] != 0) lb_loop1_state[lane] = 0x51; else { uart_puts("\r\n(lim)"); lb_loop2_state[lane] = 0x00; } }
        else if (r != 2) lb_loop1_state[lane] = 0x51;
      } else if (state == 0x52) {
        lb_settle_counter[lane]++;
        lb_lane_desc_idx[lane] = (uint8_t)((lb_lane_desc_idx[lane] + 1) & 0x0F);
        lb_loop1_state[lane] = 0x70;
      } else if (state == 0x60) {
        __xdata uint8_t walk_idx = lb_lane_desc_idx[lane];
        sb_lane_desc[walk_idx] |= 0xA0;
        lb_loop2_scratch[lane] = walk_idx;
        if (walk_idx == 0) lb_loop1_state[lane] = 0x80;
      } else if (state == 0x70) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] == 0) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x90; } else lb_loop1_state[lane] = 0x70; }
        else if (r != 2) lb_loop1_state[lane] = 0x70;
      } else if (state == 0x80) {
        lb_loop1_state[lane] = 0xA0;
      } else if (state == 0x90) {
        lb_loop2_scratch[lane] &= 0x7F;
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0xA1;
      } else if (state == 0xA0) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop1_state[lane] = 0xB0; else lb_loop1_state[lane] = 0xA0; }
        else if (r != 2) lb_loop1_state[lane] = 0xA0;
      } else if (state == 0xA1) {
        if (u4lb_ee6e(lane) == 0) { uart_puts("\r\n(lim)"); lb_loop2_state[lane] = 0x60; }
        else lb_loop1_state[lane] = 0x50;
      } else {
        u4_work_buf[lane] &= 0xEF;
        u4_work_buf[lane] &= 0x7F;
        u4_work_buf[0x1C + lane] &= 0xDF;
        lb_settle_counter[lane] = 0x00;
        lb_loop1_state[lane] = 0x20;
      }
    }
  }
  (void)selector;
}

/* u4lb_state5(): e672 state-5 entry. 0x0718==4 -> 8000 else 850b. */
static void u4lb_state5(void) {
  DPX = 0x00;
  u4lb_s5_diag();
  if (u4_route_enable_latch == 0x04) u4lb_walk_8000();
  else                    u4lb_walk_850b();
  DPX = 0x00;
}

/* e672 — the lane-bond FSM dispatcher, called from cb10's tail (gated 0x06ED!=0).
 *   3 -> cm_conn_routing_setup; 4 -> b0b4; 5 -> finalise when all sub-lane states clear, else the walker. */
static void u4lb_e672(void) {
  uint8_t state = u4_fsm_state;
  if (state == 0x04) {
    u4lb_state4_b0b4();
    return;
  }
  if (state == 0x05) {
    if (lb_loop2_state[0x0] == 0 && lb_loop1_state[0x0] == 0 && lb_loop2_state[0x1] == 0 && lb_loop1_state[0x1] == 0) {
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
