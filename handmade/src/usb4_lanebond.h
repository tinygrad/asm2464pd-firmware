#ifndef USB4_LANEBOND_H
#define USB4_LANEBOND_H

static void u4lb_set_fsm_state(u4_fsm_state_t state) {  /* eb62 */
  uart_puts("\r\n[SB P0");
  uart_puthex(state);
  uart_putc(']');
  u4_sb.state = state;
}

static uint8_t u4lb_send_route_query(void) {  /* edf5 */
  if (u4_sb.routerop_push_token != 0) return 0;

  sb_tx_flag = 0;
  sb_tx_cmd = 5;
  sb_tx_byte0 = 0x0C;
  sb_tx_byte1 = 3;
  sb_routerop_tx(0);
  return 1;
}

static void u4lb_desc_engine_arm(void) {  /* a2c2 */
  P12_WR(DE_CTRL, (uint8_t)((P12_RD(DE_CTRL) & 0xC0) | 0x01));
  P12_WR(DE_CTRL, (uint8_t)((P12_RD(DE_CTRL) & 0x3F) | 0x40));
  P12_WR(DE_OPCODE, 0xD2);
  P12_WR(DE_COMMIT, (uint8_t)(P12_RD(DE_COMMIT) & 0xE0));
}

static void u4lb_reg_set_bit7(uint8_t cur) {  /* a310 */
  P12_WR(cur, (uint8_t)((P12_RD(cur) & 0x3F) | 0x80));
}

static void u4lb_desc_engine_clear(uint8_t commit_hi2) {  /* e711 */
  (void)commit_hi2;
  P12_WR(DE_CTRL, (uint8_t)(P12_RD(DE_CTRL) & 0xC0));
  DE_WR_CLEAR();
}
static void u4lb_desc_commit_noset(uint8_t ctrl_low6) {  /* e890 */
  (void)ctrl_low6;
  P12_WR(DE_COMMIT, (uint8_t)(P12_RD(DE_COMMIT) & 0x7F));
  P12_WR(DE_KICK, 0x01);
  { uint16_t g; for (g = 0; (P12_RD(DE_KICK) & 0x01) && g < 0x2000; g++) { } }
  u4lb_desc_engine_clear(0);
}
static void u4lb_desc_commit_reg35(uint8_t val) {  /* e7f8 */
  P12_WR(DE_CTRL, val);
  (void)u4c_sb_desc_commit();
}
static void u4lb_desc_apply_lanemask(void) {  /* e00c */
  uint8_t m = (uint8_t)((u4lb_desc0_lanemask != 0) ? 1 : 0);
  uint8_t a;
  if (u4lb_desc1 != 0) m |= 2;
  if (u4lb_desc2 != 0) m |= 4;
  if (u4lb_desc3 != 0) m |= 8;
  P12_WR(DE_LANESEL, (uint8_t)((P12_RD(DE_LANESEL) & 0xF0) | 0x0F));
  a = (uint8_t)((P12_RD(DE_CTRL) & 0x3F) | 0x80); P12_WR(DE_CTRL, a);
  P12_WR(DE_OPCODE, (uint8_t)((a & 0xF0) | m));
}
static void u4lb_desc_emit_lanes(uint8_t ctrl_bits, uint8_t set_lanes) {  /* ce23 */
  uint8_t ctrl_low6 = (uint8_t)(ctrl_bits & 0x3F);
  uint8_t r6, lane_mask, r7;
  r6 = (uint8_t)(P12_RD(DE_CTRL) & 0x3F);
  u4lb_desc_commit_noset(ctrl_low6);
  if (set_lanes != 0) {
    r7 = P12_RD(DE_RD(0));
    P12_WR(DE_WR(0), (uint8_t)(u4lb_desc0_lanemask | r7)); r7 = P12_RD(DE_RD(1));
    P12_WR(DE_WR(1), (uint8_t)(u4lb_desc1          | r7)); r7 = P12_RD(DE_RD(2));
    P12_WR(DE_WR(2), (uint8_t)(u4lb_desc2          | r7)); r7 = P12_RD(DE_RD(3));
    lane_mask   = (uint8_t)(u4lb_desc3          | r7);
  } else {
    r7 = P12_RD(DE_RD(0));
    P12_WR(DE_WR(0), (uint8_t)((uint8_t)~u4lb_desc0_lanemask & r7)); r7 = P12_RD(DE_RD(1));
    P12_WR(DE_WR(1), (uint8_t)((uint8_t)~u4lb_desc1          & r7)); r7 = P12_RD(DE_RD(2));
    P12_WR(DE_WR(2), (uint8_t)((uint8_t)~u4lb_desc2          & r7)); r7 = P12_RD(DE_RD(3));
    lane_mask   = (uint8_t)((uint8_t)~u4lb_desc3          & r7);
  }
  P12_WR(DE_WR(3), lane_mask);
  u4lb_desc_apply_lanemask();
  u4lb_desc_commit_reg35((uint8_t)((P12_RD(DE_CTRL) & 0xC0) | r6));
  u4lb_desc0_lanemask = 0; u4lb_desc1 = 0; u4lb_desc2 = 0; u4lb_desc3 = 0;
}
static void u4lb_desc_emit_routerop(void) {  /* c3ce */
  uint8_t pass;
  u4_cfg.routerop_desc1 = u4lb_desc0_lanemask;
  u4_cfg.routerop_desc2 = u4lb_desc1;
  u4_cfg.routerop_desc3 = u4lb_desc2;
  u4_cfg.msg_type       = u4lb_desc3;
  for (pass = 1; pass <= 2; pass++) {
    (void)P12_RD(DE_CTRL);
    ENG_DESC_WR_SELF40(0x35, (uint8_t)((P12_RD(DE_CTRL) & 0xC0) | pass));
    u4lb_desc0_lanemask = u4_cfg.routerop_desc1; P12_WR(DE_WR(0), u4_cfg.routerop_desc1);
    u4lb_desc1          = u4_cfg.routerop_desc2; P12_WR(DE_WR(1), u4_cfg.routerop_desc2);
    u4lb_desc2          = u4_cfg.routerop_desc3; P12_WR(DE_WR(2), u4_cfg.routerop_desc3);
    u4lb_desc3          = u4_cfg.msg_type;       P12_WR(DE_WR(3), u4_cfg.msg_type);
    if ((uint8_t)(u4lb_desc_set_mode ^ 2) == 0) u4c_sb_desc_commit();
    else                                       u4lb_desc_emit_lanes((uint8_t)(u4lb_desc_set_mode ^ 2), u4lb_desc_set_mode);
  }
}
static void u4lb_emit_lane0_desc(uint8_t set_lanes) {  /* e175 */
  if (set_lanes != 0) u4lb_lane_active_flags |= 0x01;
  u4lb_desc_engine_arm();
  u4lb_desc1 = 1;
  u4lb_desc_emit_lanes(1, set_lanes);
  u4lb_reg_set_bit7(0x35);
  ENG_DESC_WR_RDBACK(0x36, 0x8D);
  P12_WR(DE_COMMIT, (uint8_t)((P12_RD(DE_COMMIT) & 0xE0) | 0x01));
  u4lb_desc1 = 0x10;
  u4lb_desc_emit_lanes(0x10, set_lanes);
}
static void u4lb_emit_lane1_desc(uint8_t set_lanes) {  /* e282 */
  if (set_lanes != 0) u4lb_lane_active_flags |= 0x02;
  u4lb_desc_engine_arm();
  u4lb_desc1 = 2;
  u4lb_desc_emit_lanes(2, set_lanes);
  u4lb_reg_set_bit7(0x35);
  ENG_DESC_WR_CLR(0x36, 0x97);
  u4lb_desc1 = 0x02;
  u4lb_desc_emit_lanes(0x02, set_lanes);
}
static void u4lb_emit_route_desc(void) {  /* c17f */
  uint8_t desc0;
  (void)P12_RD(DE_LANESEL);
  P12_WR(DE_LANESEL, 0x0C);
  ENG_DESC_WR_CLR(0x35, 0x04);
  u4lb_desc2 = 0x20; u4lb_desc3 = 0x04; u4lb_desc_set_mode = 0x02;
  u4lb_desc_emit_routerop();
  (void)P12_RD(DE_LANESEL);
  P12_WR(DE_LANESEL, 0x0C);
  ENG_DESC_WR_CLR(0x35, 0x80);
  desc0 = 0x18;
  if (u4_work_buf[WB_LANE_CAP] & 0x20) desc0 |= 0x04;
  if (u4_cfg.cap20g_gate1)          desc0 |= 0x20;
  u4lb_desc2 = desc0;
  u4lb_desc3 = 0x00;
  u4lb_desc_set_mode = 0x02;
  u4lb_desc_emit_routerop();
}

static void u4lb_cm_conn_routing_setup(void) {
  connrt_substate_t state = u4_sb.conn_routing_substate;
  if (state == CONNRT_ARM_ROUTE_QUERY) {
    if (u4lb_send_route_query() != 1) return;
    u4_sb.conn_routing_substate = CONNRT_AWAIT_RESULT;
    return;
  }
  if (state != CONNRT_AWAIT_RESULT) {
    if (state != CONNRT_PRINT_STATUS) return;
    u4lb_set_fsm_state(U4FSM_LANE_TRAIN);
    return;
  }

  { uint8_t selector;
    if (u4_sb.route_query_response != 0)      { u4_sb.route_query_response = 0; u4_sb.routerop_push_token = 0; selector = 0; }
    else if (u4_sb.routerop_push_token == 0x02) { u4_sb.routerop_push_token = 0; selector = 2; }
    else                       { selector = 1; }
    if (selector == 2) { u4_sb.conn_routing_substate = CONNRT_ARM_ROUTE_QUERY; return; }
    if (selector != 0) { return; }
  }

  if (u4_host_desc[HD_TYPE] != 0x0C) { u4_sb.conn_routing_substate = CONNRT_ARM_ROUTE_QUERY; return; }

  if (u4_pd.connect_route_latch != 0) {
    uint8_t host_status = u4_host_desc[HD_STATUS];
    if (((host_status & 0x7F) == 2) || ((u4_work_buf[0x1B] & 1) == 0) ||
        (u4_pd.confirm_input_ce != 0 && u4_pd.confirm_input_cd == 0)) {
      u4_sb.coldboot_seed_gate = 0;
    } else {
      SB_CLR(SB_ROUTE_GATE, 0x80);
    }
  }
  if (u4_sb.coldboot_seed_gate == 0) {
    mem_copy(sb_width_lut, width_lut_rom, 0x13);
    mem_copy(sb_branchA_gate, branchA_gate_rom, 0x13);
    { uint8_t i; for (i = 0; i < 2; i++) {
      lb_cl_status[i] = 0; lb_eq_status[i] = 0; lb_loop2_scratch[i] = 0; lb_cl0_width[i] = 0;
    } }
  }

  if (u4_sb.coldboot_seed_gate == 0 && u4_pd.confirm_input_ce != 0) {
    uart_puts("[ConnRtmr]");
    u4_sb.route_enable_latch = 0;
  } else {
    uart_puts("[ConnRout]");
    u4_sb.route_enable_latch = 4;
  }

  { uint8_t host_width = u4_host_desc[HD_WIDTH];
    if ((host_width & 1) && (u4_work_buf[WB_LANE_CAP] & 1)) { u4_work_buf[WB_LANE_EN] = (u4_work_buf[WB_LANE_EN] & 0xFE) | 1; }
    if ((host_width & 0x02) && (u4_work_buf[WB_LANE_CAP] & 2)) { u4_work_buf[WB_LANE_EN] = (u4_work_buf[WB_LANE_EN] & 0xFD) | 2; }
    if ((host_width & 0x10) && (u4_work_buf[WB_LANE_CAP] & 0x10) && (u4_work_buf[WB_LANE_EN] & 1) && (u4_work_buf[WB_LANE_EN] & 2)) u4_sb.lane_width_latch1 = 1;
    else u4_sb.lane_width_latch1 = 0;
    if ((host_width & 0x20) && (u4_work_buf[WB_LANE_CAP] & 0x20)) u4_sb.lane_width_latch0 = 2;

    u4_sb.phy_gate_a = 0; u4_sb.phy_gate_b = 0;

    if (u4_sb.lane_width_latch0 == 2) {
      if ((host_width & 0x80) && (u4_work_buf[WB_LANE_CAP] & 0x80)) u4_sb.phy_gate_b = 1;
    } else {
      uint8_t set_a = 0;
      if (u4_pd.enter_usb_accepted != 0 &&
          (host_width & 0x40) &&
          (u4_work_buf[WB_LANE_CAP] & 0x40)) {
        set_a = 1;
      } else if (u4_pd.connect_route_latch != 0) {
        if ((host_width & 0x40) ||
            (u4_work_buf[WB_LANE_CAP] & 0x40)) {
          set_a = 1;
        }
      }
      if (set_a) u4_sb.phy_gate_a = 1;
    }

    {
      uint8_t rate = u4_sb.phy_gate_a;
      uint8_t mask;
      u4_cfg.lb_width_rate_code = u4_sb.phy_gate_a;
      mask = (uint8_t)(u4lb_lane_active_flags
                       | (u4_sb.phy_gate_b ? 0x10 : 0x00)
                       | (u4_sb.phy_gate_a ? 0x08 : 0x00));
      u4lb_lane_active_flags = mask; u4lb_desc_engine_arm();
      u4lb_desc1 = 0x08;
      u4lb_desc_emit_lanes(rate, rate);
      u4lb_desc_engine_arm();
      u4lb_desc1 = 0x10;
      u4lb_desc_emit_lanes(rate, rate);
    }

    if ((u4_sb.phy_gate_a | u4_sb.phy_gate_b) != 0) {
      uint8_t rb = (uint8_t)((SB_RD(SB_RATE_STROBE) & 0xBF) | 0x40);
      SB_WR(SB_RATE_STROBE, rb); rb = SB_RD(SB_RATE_STROBE);
      SB_WR(SB_RATE_STROBE, (uint8_t)((rb & 0x7F) | 0x80));
    } else {
      uint8_t rb = (uint8_t)(SB_RD(SB_RATE_STROBE) & 0xBF);
      SB_WR(SB_RATE_STROBE, rb); rb = SB_RD(SB_RATE_STROBE);
      SB_WR(SB_RATE_STROBE, (uint8_t)(rb & 0x7F));
    }

    if ((u4_connect_gate & 0x01) && (u4_cfg.route_mode & 0x02)) {
      REG_LINK_STATUS_E716 = (uint8_t)((REG_LINK_STATUS_E716 & 0xFC) | 0x03);
    }
  }

  {
    uint16_t rate = (uint16_t)((uint16_t)sb_lane_flip[0xB] * 0x20);
    uint8_t rate_hi = (uint8_t)(rate >> 8);
    uint8_t rate_lo = (uint8_t)((rate & 0xFF) | lb_cap_field[0xB]);
    SB_WR(SB_RATE_HI(0), rate_hi); SB_WR(SB_RATE_LO(0), rate_lo);
    SB_WR(SB_RATE_HI(1), rate_hi); SB_WR(SB_RATE_LO(1), rate_lo);
    SB_WR(SB_WIDTH_LO, 0x00); SB_WR(SB_WIDTH_HI, (uint8_t)((u4_sb.lane_width_latch0 == 2) ? 0x1F : 0x0F));
    if (REG_LANE_RATE_C8FF == 0x04) {
      if (u4_pd.enter_usb_accepted != 0) {
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x02);
      } else {
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x03);
        REG_PHY_LANEA_C293 = (uint8_t)((REG_PHY_LANEA_C293 & 0xFC) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x03);
        REG_PHY_LANEB_C313 = (uint8_t)((REG_PHY_LANEB_C313 & 0xFC) | 0x02);
      }
    }
    if (u4_pd.enter_usb_accepted == 0 && u4_sb.lane_width_latch0 == 1) {
      REG_PHY_LANEA_C2C5 = (uint8_t)((REG_PHY_LANEA_C2C5 & 0xF0) | 0x0F);
      REG_PHY_LANEB_C345 = (uint8_t)((REG_PHY_LANEB_C345 & 0xF0) | 0x0F);
    }
  }

  u4lb_emit_lane0_desc(u4_sb.lane_width_latch1);
  u4lb_emit_lane1_desc(u4_pd.connect_route_latch);
  u4lb_emit_route_desc();

  u4_sb.conn_routing_substate = CONNRT_PRINT_STATUS;
}

static void u4lb_sb_op_kick(uint8_t op) {  /* 96fe */
  SB_WR(SB_DESC_CMD, op);
  SB_WR(SB_DESC_COUNT_GO, (SB_RD(SB_DESC_COUNT_GO) & 0x80) | 0x03);
}

static void u4lb_sb_op_run_drain(uint8_t param) {  /* d5da */
  if (param == 1) {
    SB_WR(SB_OP_CTRL, (SB_RD(SB_OP_CTRL) & 0xFE) | 0x01);
  }
  P1_WR(P1_LANE_FLIP(0), P1_RD(P1_LANE_FLIP(0)) & 0xFE);
  SB_WR(SB_ADP0_EN, SB_RD(SB_ADP0_EN) & 0xFD);
  SB_WR(SB_OP_TRIGGER, 0x01);
  while ((SB_RD(SB_CONNECT_EVENT) & 0x04) == 0) { }
  SB_WR(SB_CONNECT_EVENT, 0x04);
  phy_cc10_cmd_wait(1, 0, 0x0B);
  SB_WR(SB_OP_CTRL, SB_RD(SB_OP_CTRL) & 0xFE);
  if (param != 0) return;
  { uint8_t count = SB_RD(SB_DESC_COUNT_GO);
    if (count <= 6) return;
    { uint8_t limit = (uint8_t)(count - 6), i;
      for (i = 0; i < limit; i++) SBTX_WR(i, 0x00); }
  }
}

static void u4lb_sb_send_lane_cfg(void) {  /* e07d */
  uint8_t cfg;
  SB_WR(SB_DESC_CMD, 0x61);
  SBTX_WR(SBTX_DESC_TYPE, 0x09);
  if ((u4_sb.phy_gate_a | u4_sb.phy_gate_b) != 0)
    SBTX_WR(SBTX_DESC_TYPE, SBTX_RD(SBTX_DESC_TYPE) | 0x04);
  if (u4_pd.connect_route_latch != 0)
    SBTX_WR(SBTX_DESC_TYPE, SBTX_RD(SBTX_DESC_TYPE) | 0x10);
  cfg = (uint8_t)(((u4_sb.lane_width_latch0 & 0x0F) << 4)
                  | ((u4_work_buf[WB_LANE_EN] & 0x02) ? 0x02 : 0x00)
                  | (u4_work_buf[WB_LANE_EN] & 0x01));
  SBTX_WR(SBTX_DESC_DIR, cfg);
  SB_WR(SB_DESC_COUNT_GO, (SB_RD(SB_DESC_COUNT_GO) & 0x80) | 0x08);
  u4lb_sb_op_run_drain(0);
}

static void u4lb_rxpll_reset_pulse(void) {  /* e9e7 */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  REG_PHY_RXPLL_RESET = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_RXPLL_RESET = 0x00;
  phy_cc10_cmd_wait(2, 0, 0x28);
  REG_CPU_CTRL_CC37 &= 0xFB;
}

static void u4lb_rxpll_train(void) {  /* e764 */
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xF7) | 0x08);
  REG_PHY_TIMER_CTRL_E764 &= 0xFB;
  REG_PHY_TIMER_CTRL_E764 &= 0xFE;
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFD) | 0x02);
  phy_cc10_cmd_wait(1, 7, 0xCF);
  if (REG_PHY_RXPLL_STATUS & 0x10) {
    REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFE) | 0x01);
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
  } else {
    REG_PHY_TIMER_CTRL_E764 &= 0xF7;
    REG_PHY_TIMER_CTRL_E764 &= 0xFB;
    REG_PHY_TIMER_CTRL_E764 &= 0xFE;
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
  }
}

static void u4lb_phy_ctrl_pulse_lock(void) {  /* ebde */
  REG_PHY_CTRL_C20F = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_CTRL_C20F = 0x00;
  { uint16_t g = 0; while (((REG_PHY_LANEA_LOCK_C2D0 & 0x20) == 0) && ++g < 0x2000); }
  { uint16_t g = 0; while (((REG_PHY_LANEB_LOCK_C350 & 0x20) == 0) && ++g < 0x2000); }
}

static void u4lb_phy_rate_start(void) {  /* e980 */
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  u4lb_phy_ctrl_pulse_lock();
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEA_RATE_DESC_C2C9 = (REG_PHY_LANEA_RATE_DESC_C2C9 & 0x80)
             | (uint8_t)(((REG_PHY_LANEA_RATE_SRC_C2EC & 0x38) >> 3) | 0x40);
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  REG_PHY_LANEB_RATE_DESC_C349 = (REG_PHY_LANEB_RATE_DESC_C349 & 0x80)
             | (uint8_t)(((REG_PHY_LANEB_RATE_SRC_C36C & 0x38) >> 3) | 0x40);
  REG_PHY_LANEA_RATE_START_C2A8 = (REG_PHY_LANEA_RATE_START_C2A8 & 0x3F) | 0x80;
  REG_PHY_LANEB_RATE_START_C328 = (REG_PHY_LANEB_RATE_START_C328 & 0x3F) | 0x80;
}

static void u4lb_sb_rate_strobe(uint8_t rate) {  /* d3b0 */
  u4_cfg.lb_width_rate_code = rate;
  if (u4_sb.lane_width_latch0 == 1) {
    if (rate & 0x01) SB_WR(SB_RATE_STROBE, (SB_RD(SB_RATE_STROBE) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(SB_RATE_STROBE, (SB_RD(SB_RATE_STROBE) & 0xDF) | 0x20);
    if (rate & 0x01) SB_WR(SB_RATE_STROBE, SB_RD(SB_RATE_STROBE) & 0xEF);
    if (rate & 0x02) SB_WR(SB_RATE_STROBE, SB_RD(SB_RATE_STROBE) & 0xDF);
  } else {
    if (rate & 0x01) SB_WR(SB_RATE_STROBE, SB_RD(SB_RATE_STROBE) & 0xEF);
    if (rate & 0x02) SB_WR(SB_RATE_STROBE, SB_RD(SB_RATE_STROBE) & 0xDF);
    if (rate & 0x01) SB_WR(SB_RATE_STROBE, (SB_RD(SB_RATE_STROBE) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(SB_RATE_STROBE, (SB_RD(SB_RATE_STROBE) & 0xDF) | 0x20);
  }
  phy_cc10_cmd_wait(2, 0, 0xC8);
}

static void u4lb_lane_train_arm(void) {  /* ec51 */
  REG_LANE_TRAIN_ARM = 0x04; REG_LANE_TRAIN_ARM = 0x02;
  REG_LANE_TRAIN_CTRL = (REG_LANE_TRAIN_CTRL & 0xF8) | 0x04;
  REG_LANE_TRAIN_MASK_LO = 0xFF; REG_LANE_TRAIN_MASK_HI = 0xFF;
  REG_LANE_TRAIN_ARM = 0x01;
  u4_sb.lane_train_trigger ^= 0x01;
}

static void u4lb_phy_settle_wait(void) { phy_cc10_cmd_wait(2, 0, 0xC8); }  /* b226 */

static uint16_t u4lb_read_lane_width_cnt(void) {  /* ee57 */
  if (!(REG_LANE_TRAIN_ARM & 0x01) || (REG_LANE_TRAIN_ARM & 0x02)) u4lb_lane_train_arm();
  return ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
}

static void u4lb_conn_rout_restart(void) {  /* 98ec */
  u4_sb.conn_routing_substate = CONNRT_ARM_ROUTE_QUERY;
  u4lb_read_lane_width_cnt();
  u4_sb.lane_width_cnt_hi = 0;
  u4_sb.lane_width_cnt_lo = 3;
}

static void u4lb_p1_7104_set40(void) {  /* d195 */
  P1_WR(P1_PCIE_PHY_7104, (uint8_t)((P1_RD(P1_PCIE_PHY_7104) & 0xBF) | 0x40));
}

static uint8_t u4lb_phy_8d_set08(uint16_t base_hi) {  /* d1d3 */
  return (uint8_t)((P1_RD((uint16_t)((base_hi & 0xFF00) | 0x8D)) & 0xF3) | 0x08);
}

static void u4lb_pcie_link_bringup(void) {  /* df61 */
  uint8_t v;
  u4lb_p1_7104_set40();
  P1_WR(P1_RXPLL_CFG_1808, 0x00);
  v = (uint8_t)((P1_RD(P1_PCIE_LINK_1835) & 0xFE) | 0x01);
  P1_WR(P1_PCIE_LINK_1835, v);
  (void)P1_RD(P1_PCIE_PHY_7041);
  P1_WR(P1_PCIE_PHY_7041, (uint8_t)(v | 0x40));
  P1_WR(P1_TUNNEL_CFG_6043, 0x70);
  P1_WR(P1_TUNNEL_CFG_6025, (uint8_t)((P1_RD(P1_TUNNEL_CFG_6025) & 0x7F) | 0x80));
  P1_WR(P1_PCIE_PHY_508F, 0x01);
  P1_WR(P1_PCIE_PHY_508D, u4lb_phy_8d_set08(0x5000));
  P1_WR(P1_PCIE_PHY_5204, (uint8_t)(P1_RD(P1_PCIE_PHY_5204) & 0xFE));
  P1_WR(P1_PCIE_PHY_5204, (uint8_t)(P1_RD(P1_PCIE_PHY_5204) & 0xFD));
  P1_WR(P1_PCIE_PHY_408D, u4lb_phy_8d_set08(0x4000));
}

static void u4lb_pcie_tunnel_reset(void) {  /* ed44 */
  REG_PCIE_TUNNEL_CTRL |= 0x01;
  REG_PCIE_TUNNEL_CTRL |= 0x02;
  REG_PCIE_TUNNEL_CTRL &= ~0x01;
  REG_PCIE_TUNNEL_CTRL &= ~0x02;
  REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xF7) | 0x08);
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_pcie_link_bringup();
}

static void u4lb_cpu_ext_kick(void) {  /* e74e */
  u4_p12.cc_subdemux_src = 0;
  REG_CPU_EXT_CTRL &= 0xEF;
  REG_CPU_EXT_STATUS = 0x04;
  REG_CPU_EXT_STATUS = 0x02;
}

static void u4lb_emit_link_reinit_desc(uint8_t v) {  /* e06b */
  u4_cfg.routerop_desc2 = v;
  P12_WR(DE_CTRL, (uint8_t)((P12_RD(DE_CTRL) & 0x3F) | 0x80));
  u4lb_desc0_lanemask = 1;
  u4lb_desc_emit_lanes(v, v);
  u4_p12.link_reinit_gate = (uint8_t)(u4_cfg.routerop_desc2 != 0 ? 1 : 0);
}

static void u4lb_transport_reinit(uint8_t skip_lane) {  /* b031 */
  uint8_t a;

  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xFE));
  P12_WR(DE_TRANSPORT(1), (uint8_t)((P12_RD(DE_TRANSPORT(1)) & 0xEF) | 0x10));
  P12_WR(DE_TRANSPORT(0), (uint8_t)(P12_RD(DE_TRANSPORT(0)) & 0xEF));
  P12_WR(DE_TRANSPORT(2), 0x02);
  P12_WR(DE_TRANSPORT(0), (uint8_t)(P12_RD(DE_TRANSPORT(0)) & 0xDF));
  P12_WR(DE_TRANSPORT(2), 0x04);
  P12_WR(DE_TRANSPORT_TRIG, 0x01);
  a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802) & 0xFD); P1_WR(P1_XPORT_RESET_1802, a); a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802) & 0xFB);
  P1_WR(P1_XPORT_RESET_1802, a); a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802) & 0xF7); P1_WR(P1_XPORT_RESET_1802, a);
  a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802) & 0xEF); P1_WR(P1_XPORT_RESET_1802, a);
  if (!skip_lane) {
    a = (uint8_t)((P1_RD(P1_XPORT_LANE_EVT_1404) & 0xFD) | 0x02);
    a = (uint8_t)((a & 0xEF) | 0x10); P1_WR(P1_XPORT_LANE_EVT_1404, a);
  }
  P1_WR(P1_XPORT_TRIG_1511, 0x01);
  if (!skip_lane) {
    a = (uint8_t)(P1_RD(P1_XPORT_LANE_EVT_1404) & 0xFD); P1_WR(P1_XPORT_LANE_EVT_1404, a);
    a = (uint8_t)(P1_RD(P1_XPORT_LANE_EVT_1404) & 0xEF); P1_WR(P1_XPORT_LANE_EVT_1404, a);
  }
  a = (uint8_t)((P1_RD(P1_XPORT_RESET_1802) & 0xFD) | 0x02);
  a = (uint8_t)(((a & 0xFB) | 0x04)); P1_WR(P1_XPORT_RESET_1802, a); a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802));
  a = (uint8_t)(((a & 0xF7) | 0x08)); P1_WR(P1_XPORT_RESET_1802, a); a = (uint8_t)(P1_RD(P1_XPORT_RESET_1802));
  a = (uint8_t)((a & 0xEF) | 0x10); P1_WR(P1_XPORT_RESET_1802, a);
  if (!skip_lane) {
    a = (uint8_t)(P1_RD(P1_XPORT_LANE_EVT_1405) & 0xFE); P1_WR(P1_XPORT_LANE_EVT_1405, a);
    a = (uint8_t)(P1_RD(P1_XPORT_LANE_EVT_1405) | 0x01); P1_WR(P1_XPORT_LANE_EVT_1405, a);
  }
  P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x08);

  P12_WR(DE_CTRL, (uint8_t)(P12_RD(DE_CTRL) & 0xC0));
  DE_WR_CLEAR();
  u4lb_desc0_lanemask = 0; u4lb_desc1 = 0; u4lb_desc2 = 0; u4lb_desc3 = 0;
  u4c_desc_engine_reset();
  u4c_seed_workbuf();
  sb_rom_descriptor_load();
  sb_eng_data_init();
  u4c_descriptor_load_stock();
  u4c_router_cfg_seed_usb4();
  u4lb_emit_route_desc();
  u4c_desc_edge_engine();
  u4c_desc_edge_clear();
  u4c_desc_seq_commit();
  u4c_transport_reg_reinit();
  if (!skip_lane) {
    u4lb_emit_link_reinit_desc(0);
  }
  u4lb_cpu_ext_kick();
}

static void u4lb_pcie_tunnel_pwroff(void) {  /* ee29 */
  REG_PCIE_LANE_CTRL_C659 &= 0xFE;
  REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
  u4lb_pcie_tunnel_reset();
  u4lb_cpu_ext_kick();
  PR(0x0B42) = 0;
  PR(0x0B43) = 0;
}

static void u4lb_p1_desc_engine_arm(void) {  /* d149 */
  P1_WR(P1_DESC_CTRL_1235, (uint8_t)((P1_RD(P1_DESC_CTRL_1235) & 0xC0) | 0x03));
  P1_WR(P1_DESC_CTRL_1235, (uint8_t)((P1_RD(P1_DESC_CTRL_1235) & 0x3F) | 0x40));
  P1_WR(P1_DESC_CMD_1236, 0x09);
  P1_WR(P1_DESC_COMMIT_1237, (uint8_t)(P1_RD(P1_DESC_COMMIT_1237) & 0xE0));
}

static uint8_t u4lb_p1_desc_result(void) { return P1_RD(P1_DESC_RESULT_1243); }  /* d1dd */

static uint8_t u4lb_p1_desc_query(uint8_t arg) {  /* ee94 */
  u4lb_p1_desc_engine_arm();
  u4lb_desc_commit_noset(arg);
  return u4lb_p1_desc_result();
}
static void u4lb_save_b402_clr(void) {  /* e84d */
  u4_boot.pcie_ctrl_shadow = (uint8_t)(REG_PCIE_CTRL_B402 & 0x02);
  u4_boot.pcie_ctrl_shadow = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
}

static void u4lb_pcie_tunnel_pwron(void) {  /* e76b */
  uint8_t gate = u4lb_p1_desc_query(0x04);
  if (gate != 0) {
    uint8_t v;
    u4lb_save_b402_clr();
    v = (uint8_t)(P1_RD(P1_PCIE_PHY_7041) & 0xBF);
    P1_WR(P1_PCIE_PHY_7041, v);
    phy_cc10_cmd_wait(1, 0, 0xCF);
    u4lb_p1_7104_set40();
    if (!(REG_PCIE_LANE_CTRL_C659 & 0x01)) {
      REG_PCIE_LANE_CTRL_C659 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
      REG_PCIE_LANE_CTRL_C659 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFE) | 0x01);
    }
    if (u4_boot.pcie_ctrl_shadow) u4_boot.pcie_ctrl_shadow = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFD) | 0x02);
  }
}

static void u4lb_rxpll_cfg_trigger(void) {  /* e9b5 */
  P1_WR(P1_RXPLL_CFG_1808, P1_RD(P1_RXPLL_CFG_1808));
  REG_PHY_RXPLL_CFG_B = 0xFF;
  REG_PHY_RXPLL_CFG_A = (uint8_t)((REG_PHY_RXPLL_CFG_A & 0xFB) | 0x04);
  REG_PHY_RXPLL_CFG_B = (uint8_t)(REG_PHY_RXPLL_CFG_B & 0xFB);
  REG_PHY_RXPLL_CFG_A = (uint8_t)((REG_PHY_RXPLL_CFG_A & 0xF7) | 0x08);
  REG_PHY_RXPLL_CFG_B = (uint8_t)(REG_PHY_RXPLL_CFG_B & 0xF7);
  REG_PHY_RXPLL_TRIGGER = 0x04;
  REG_PHY_RXPLL_TRIGGER = 0x08;
}
static void u4lb_tunnel_pwron_train(void);  /* e26a */

/* PCIe-tunnel arm strobe: latch CPU_EXT, write the CCFA/CCFB tunnel config, kick CPU_EXT_STATUS. */
static void u4lb_pcie_tunnel_arm(void) {
  REG_CPU_EXT_CTRL = (uint8_t)((REG_CPU_EXT_CTRL & 0xF8) | 0x02);
  XDATA_REG8(0xCCFA) = 0x03;
  XDATA_REG8(0xCCFB) = 0xE7;
  REG_CPU_EXT_STATUS = 0x01;
}

static void u4lb_pcie_link_enable(void) {  /* e4ea */
  if (!(REG_PCIE_LANE_CTRL_C659 & 0x01)) {
    u4lb_tunnel_pwron_train();
    REG_PCIE_LANE_CTRL_C659 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
  }
  u4lb_cpu_ext_kick();
  u4lb_pcie_tunnel_arm();
  REG_PCIE_PERST_CTRL &= 0xF0;
  u4lb_rxpll_cfg_trigger();
}

static void u4lb_phy_mode3_seed(void) {  /* e0d9 */
  REG_PHY_RXPLL_RESET = 0x00; REG_PHY_CTRL_C20F = 0x00; REG_PHY_CDR_SEED_C210 = 0x00;
  REG_PHY_RXPLL_RESET = 0x26; REG_PHY_CTRL_C20F = 0x26;
  REG_PHY_CDR_SEED_C214 = 0x00; REG_PHY_CDR_SEED_C215 = 0x00; REG_PHY_CDR_SEED_C216 = 0x00;
  REG_PHY_CDR_SEED_C214 = 0x26;
}
static void u4lb_link_phy_reconfig(void) {  /* d90e */
  P1_SET(P1_LINK_PHY_CFG_1267, 0x02);
  P1_CLR(P1_LINK_PHY_CFG_1267, 0x04);
  P1_SET(P1_LINK_PHY_CFG_1267, 0x08);
  if (u4_connect_gate & 0x02) {
    u4lb_phy_mode3_seed();
  }
  if (u4_cfg.route_mode & 0x81) {
    u4lb_rxpll_cfg_trigger();
    REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
    REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
  }
  u4lb_cpu_ext_kick();
  u4lb_pcie_tunnel_arm();
}

static void u4lb_tunnel_event_dispatch(uint8_t assert) {  /* d855 */
  uint8_t p1508 = P1_RD(P1_USB4_TUNNEL_EVENT_STATUS_1508);
  if (p1508 & 0x10) {
    if (u4_cfg.route_mode & 0x81) { P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x10); u4lb_pcie_link_enable(); }
  } else if (p1508 & 0x08) {
    if (u4_cfg.route_mode & 0x81) { P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x08); u4lb_pcie_tunnel_pwroff(); }
  } else if (p1508 & 0x04) {
    P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x04);
    u4lb_pcie_tunnel_pwron();
  } else if (p1508 & 0x02) {
    P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x02);
    (void)u4lb_p1_desc_query(0x02);
    if (assert) {
      uint8_t v;
      REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
      v = (uint8_t)(P1_RD(P1_PCIE_PHY_7041) | 0x40);
      P1_WR(P1_PCIE_PHY_7041, v);
      phy_cc10_cmd_wait(1, 0, 0xCF);
      REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
    }
  }
}

static void u4lb_tunnel_pwron_train(void) {
  if ((u4_cfg.route_mode & 0x81) != 0) {
    u4lb_rxpll_train();
    REG_HDDPC_CTRL = (uint8_t)((REG_HDDPC_CTRL & 0xDF) | 0x20);
  }
  uart_puts("[PwrOn]");
}

static void u4lb_pcie_lane_slot_mask(uint8_t newmask) {  /* d702 */
  uint8_t i, slot;
  for (i = 0; i < 4; i++) {
    slot = P1_RD(P1_PCIE_LANE_SLOT(i)) & 0x7F;
    P1_WR(P1_PCIE_LANE_SLOT(i), (uint8_t)(((newmask & (1u << i)) ? 0x80 : 0x00) | slot));
  }
}

static void u4lb_pcie_lane_ramp(uint8_t target) {  /* c089 */
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
    u4lb_pcie_lane_slot_mask(newmask);
    phy_cc10_cmd_wait(2, 0, 0xC7);
    roundbit = (uint8_t)(roundbit << 1);
    round++;
  } while (round < 4);
}

static void u4lb_pcie_set_link_width(uint8_t mask) {  /* d436 */
  __xdata uint8_t saved_b402_1 = (uint8_t)(REG_PCIE_CTRL_B402 & 0x02);
  REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
  u4lb_pcie_lane_ramp(mask);
  if (mask != 0x0F) {
    REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01);
    REG_PCIE_TUNNEL_CTRL = (uint8_t)(REG_PCIE_TUNNEL_CTRL & 0xFE);
  }
  if (saved_b402_1 != 0) REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFD) | 0x02);
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0xF0) | (uint8_t)(mask & 0x0E));
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0x0F) | (uint8_t)(((uint8_t)(REG_PCIE_LINK_PARAM_B404 & 0x0F) ^ 0x0F) << 4));
}

static __code const uint8_t u4lb_usb4_gen_lane_lut[8] = { 0x02,0x01,0x03,0x01,0x03,0x01,0x03,0x02 };  /* a840/38cc */

static void u4lb_set_link_gen_width(uint8_t param) {  /* a840 */
  uint8_t gen = u4_link_gen;
  uint8_t lane = u4_link_lane;
  uint8_t usb4;
  uint8_t width_code;
  REG_CPU_CTRL_CA81 &= 0xFE;
  if (gen == 3 && lane == 3) {
    if ((u4_cfg.route_mode & 0x81) != 0) {
      uint8_t idx;
      u4_cfg.lb_gen_index = gen; idx = (uint8_t)(u4_cfg.lb_gen_index & 0x03);
      gen = u4lb_usb4_gen_lane_lut[(uint8_t)(idx << 1)];
      lane = u4lb_usb4_gen_lane_lut[(uint8_t)((idx << 1) + 1)];
      if (u4_pd.connect_route_latch != 0) lane = 1;
    } else {
      static __code const uint8_t u4lb_nonu4_gen_lut[5] = { 0x00, 0x00, 0x02, 0x02, 0x02 };  /* 5d24 */
      static __code const uint8_t u4lb_nonu4_lane_lut[5] = { 0x01, 0x00, 0x00, 0x01, 0x02 };  /* 5d29 */
      u4_cfg.lb_gen_index = gen;
      gen = u4lb_nonu4_gen_lut[u4_cfg.lb_gen_index]; lane = u4lb_nonu4_lane_lut[u4_cfg.lb_gen_index];
    }
  }
  usb4 = (uint8_t)(u4_cfg.route_mode & 0x81);
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
    P1_WR(P1_LINK_GEN_40B0, (uint8_t)((uint8_t)(gen + 1) | (uint8_t)(P1_RD(P1_LINK_GEN_40B0) & 0xF0)));
  } else {
    REG_TUNNEL_CTRL_B403 &= 0xFE;
    P1_WR(P1_LINK_GEN_40B0, (uint8_t)((P1_RD(P1_LINK_GEN_40B0) & 0xF0) | 0x04));
  }
  width_code = 0;
  if (lane < 3) {
    if (lane == 1) width_code = 0x0C;
    else if (lane == 0) width_code = 0x0E;
  }
  u4_cfg.lb_width_rate_code = width_code;
  REG_TUNNEL_LINK_STATUS = (uint8_t)((REG_TUNNEL_LINK_STATUS & 0xF0) | width_code);
  u4lb_pcie_set_link_width(width_code);
  if ((uint8_t)(u4_cfg.route_mode & 0x81) != 0) u4lb_pcie_tunnel_reset();
  (void)param;
}

static void u4lb_pcie_tunnel_setup(uint8_t param) {  /* e305 */
  if ((u4_cfg.route_mode & 0x81) == 0) return;
  if ((u4_link_gen == 3) ||
      (((u4_sb.lane_width_latch0 != 2) || (u4_sb.lane_width_latch1 != 0)) && (u4_sb.lane_width_latch0 != 1))) {
    REG_CPU_MODE_NEXT &= 0x1F;
  } else {
    REG_CPU_MODE_NEXT = (uint8_t)((REG_CPU_MODE_NEXT & 0x1F) | 0x20);
  }
  u4lb_pcie_tunnel_pwroff();
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_set_link_gen_width(param);
  u4lb_tunnel_pwron_train();
}

static uint8_t u4lb_read_p1_2805(void) { return P1_RD(P1_TUNNEL_PHY_2805); }  /* e916 */
static void u4lb_tunnel_phy_finalize(void) {  /* c593 */
  uint8_t v;
  REG_TUNNEL_PHY_CFG_CCB0 = (uint8_t)((REG_TUNNEL_PHY_CFG_CCB0 & 0xF8) | 0x05);
  REG_TUNNEL_PHY_CFG_CCB2 = 0x00;
  REG_TUNNEL_PHY_TIMER_CCB3 = 0xC8;
  P1_WR(P1_TUNNEL_PHY_134D, 0x04);
  P1_WR(P1_TUNNEL_PHY_CTRL_1334, 0x02);
  P1_WR(P1_TUNNEL_PHY_CFG_1335, 0x02);
  v = u4lb_read_p1_2805();
  P1_WR(P1_TUNNEL_PHY_CFG_1335, (uint8_t)((v & 0xFE) | 0x01));
  if (u4_sb.lane_bonded_flag == 0) {
    v = u4lb_read_p1_2805();
    P1_WR(P1_TUNNEL_PHY_CFG_1335, (uint8_t)(v & 0xFD));
  } else {
    P1_WR(P1_TUNNEL_PHY_CFG_1335, (uint8_t)((P1_RD(P1_TUNNEL_PHY_CFG_1335) & 0xFD) | 0x02));
  }
  v = u4lb_read_p1_2805();
  P1_WR(P1_TUNNEL_PHY_CFG_1335, (uint8_t)((v & 0xFB) | 0x04));
  P1_WR(P1_TUNNEL_PHY_CTRL_1334, (uint8_t)((P1_RD(P1_TUNNEL_PHY_CTRL_1334) & 0x7F) | 0x80));
  P1_WR(P1_TUNNEL_PHY_1285, (uint8_t)((P1_RD(P1_TUNNEL_PHY_1285) & 0x0F) | 0x30));
  P1_WR(P1_TUNNEL_PHY_CFG_1335, 0x02);
  P1_WR(P1_ADP_LINK_CFG_1206, 0x58);
}

static __xdata uint8_t eye_phase_min, eye_phase_max, eye_lo_min, eye_lo_limit, eye_hi_min, eye_hi_limit;
static void u4lb_phy_eye_margin_wait(void) {  /* b8db */
  __xdata uint8_t phase0, phase1, margin0_lo, margin0_hi, margin1_lo, margin1_hi;
  uint8_t iter;
  if ((P1_RD(P1_PORT_CTRL_0000) & 0x02) == 0) {
    if ((REG_POWER_STATUS_92F8 & 0x0C) == 0) return;
    eye_phase_min = 0x01; eye_phase_max = 0x28;
    eye_lo_min = 0x01; eye_lo_limit = 0x3D; eye_hi_min = 0x01; eye_hi_limit = 0x43;
  } else if (u4_sb.lane_width_latch0 == 1) {
    if (REG_PHY_LANEA_LOCK_C297 & 0x20) return;
    eye_phase_min = 0x01; eye_phase_max = 0x28;
    if (u4_pd.enter_usb_accepted == 0) { eye_lo_min=0x01; eye_lo_limit=0x47; eye_hi_min=0x01; eye_hi_limit=0x4D; }
    else { eye_lo_min=0x01; eye_lo_limit=0x3D; eye_hi_min=0x01; eye_hi_limit=0x43; }
  } else {
    if (REG_PHY_LANEA_LOCK_C2A7 & 0x20) return;
    eye_phase_min = 0x01; eye_phase_max = 0x20;
    if (u4_pd.enter_usb_accepted != 0) { eye_lo_min=0x01; eye_lo_limit=0x3E; eye_hi_min=0x01; eye_hi_limit=0x42; }
    else { eye_lo_min=0x01; eye_lo_limit=0x48; eye_hi_min=0x01; eye_hi_limit=0x4C; }
  }
  for (iter = 0; iter < 10; iter++) {
    phase0 = REG_PHY_LANEA_MARGIN_PHASE_C2D2 & 0x3F; margin0_lo = REG_PHY_LANEA_MARGIN_EYE_C2D9; margin0_hi = REG_PHY_LANEA_MARGIN_EYE_C2DA;
    phase1 = REG_PHY_LANEB_MARGIN_PHASE_C352 & 0x3F; margin1_lo = REG_PHY_LANEB_MARGIN_EYE_C359; margin1_hi = REG_PHY_LANEB_MARGIN_EYE_C35A;
    if ((REG_PHY_LANEA_LOCK_C2D0 & 0x40) && (REG_PHY_LANEB_LOCK_C350 & 0x40) &&
        phase0 >= eye_phase_min && phase0 <= eye_phase_max && phase1 >= eye_phase_min && phase1 <= eye_phase_max &&
        (uint8_t)(eye_lo_min - (margin0_hi <  eye_hi_min         ? 1 : 0)) <= margin0_lo &&
        margin0_lo <  (uint8_t)(eye_lo_limit - (margin0_hi < (uint8_t)(eye_hi_limit + 1) ? 1 : 0)) &&
        (uint8_t)(eye_lo_min - (margin1_hi <  eye_hi_min         ? 1 : 0)) <= margin1_lo &&
        margin1_lo <  (uint8_t)(eye_lo_limit - (margin1_hi < (uint8_t)(eye_hi_limit + 1) ? 1 : 0)))
      return;
    u4lb_rxpll_reset_pulse();
  }
}

static void u4lb_state_lane_train(void) {  /* b0b4 */
  if (u4_sb.coldboot_seed_gate != 0) {
    uint8_t i;
    for (i = 0; i < 2; i++) { u4lb_sb_send_lane_cfg(); u4lb_phy_settle_wait(); }
  } else {
    { uint8_t lane; for (lane = 0; lane < 2; lane++) {
      if (u4_work_buf[WB_LANE_EN] & (1 << lane)) {
        uint8_t op = (u4_sb.lane_width_latch0 == 2) ? (lane == 0 ? 0x85 : 0xA5) : (lane == 0 ? 0x81 : 0xA1);
        u4lb_sb_op_kick(op);
        u4lb_sb_op_run_drain(1);
      }
    } }
    u4lb_phy_settle_wait();
  }
  { uint16_t width = ((uint16_t)u4_sb.lane_width_cnt_hi << 8) | u4_sb.lane_width_cnt_lo;
    uint16_t neg   = ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
    if ((uint16_t)(width - neg) < 0x0038) { uart_puts("[b4:WIDGATE-abort]"); return; }
  }

  if (u4_sb.connect_present == 0 && u4_sb.route_up_trigger == 0) { uart_puts("[b4:CONGATE-abort]"); return; }

  if (u4_connect_gate & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    phy_cc10_cmd_wait(2, 0, 0x28);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    REG_CPU_CTRL_CA81 &= 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  }

  u4lb_pcie_tunnel_setup(1);

  { uint8_t lane; for (lane = 0; lane < 2; lane++) {
    if (u4_work_buf[WB_LANE_EN] & (1 << lane)) {
      u4lb_sb_op_kick((uint8_t)(lane == 0 ? 0x82 : 0xA2));
      u4lb_sb_op_run_drain(1);
      u4_work_buf[0x1E + lane] = (u4_work_buf[0x1E + lane] & 0x7F) | 0x80;
    }
  } }

  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  u4lb_sb_rate_strobe(3);
  u4lb_phy_rate_start();
  u4lb_rxpll_reset_pulse();
  REG_CPU_CTRL_CC37 &= 0xFB;

  u4lb_phy_eye_margin_wait();

  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;

  u4lb_tunnel_phy_finalize();

  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xF6));
  P1_WR(P1_USB4_TUNNEL_EVENT_MASK_1507, (uint8_t)(P1_RD(P1_USB4_TUNNEL_EVENT_MASK_1507) & 0x61));

  { uint8_t lane; for (lane = 0; lane < 2; lane++) {
    if (u4_work_buf[WB_LANE_EN] & (1 << lane)) {
      SB_WR(lane == 0 ? SB_LINK_REINIT_50 : SB_LINK_REINIT_5A, 0x02);
      P1_WR(P1_LANE_EN_010B, (uint8_t)((P1_RD(P1_LANE_EN_010B) & ~(1 << lane)) | (1 << lane)));
      lb_loop2_state[lane] = LP2_CL_INIT; lb_loop1_state[lane] = LP1_WIDTH_INIT;
    } else {
      lb_loop2_state[lane] = LP2_CL_IDLE; lb_loop1_state[lane] = LP1_PARKED;
    }
  } }

  u4lb_lane_train_arm();

  u4lb_set_fsm_state(U4FSM_LANE_BOND);
}

static uint8_t u4lb_lane_present(uint8_t lane) { return (uint8_t)(SB_RD(lane ? 0x60 : 0x56) & 0x01); }  /* ee6e */

static uint8_t u4lb_routerop_poll(void) {  /* eda0 */
  if (u4_sb.route_query_response != 0) { u4_sb.route_query_response = 0; u4_sb.routerop_push_token = 0; return 0; }
  if (u4_sb.routerop_push_token == 0x02) { u4_sb.routerop_push_token = 0; return 2; }
  return 1;
}

static uint8_t u4lb_send_routerop(void) {  /* e461 */
  uint8_t is_e1cb;
  if (u4_sb.routerop_push_token != 0) return 0;
  if (u4_sb.route_enable_latch == 0) {
    is_e1cb = 0;
  } else if (u4_sb.coldboot_seed_gate != 0) {
    is_e1cb = 1;
  } else {
    is_e1cb = 0;
  }
  sb_tx_flag = 0;
  sb_tx_cmd  = is_e1cb ? 0x00 : (uint8_t)(u4_sb.route_enable_latch | 0x01);
  sb_tx_byte0 = 0x0D;
  sb_tx_byte1 = 0x04;
  sb_routerop_tx(is_e1cb);
  return 1;
}

static void u4lb_phy_cl_orient_strobe(uint8_t sel, uint8_t cc) {  /* ea7c */
  uint8_t idx = (REG_PHY_VENDOR_CTRL_C6DB & 0x01) ? (uint8_t)((cc + 1) & 0x01) : cc;
  uint16_t reg = (idx == 0) ? 0xC2CB : 0xC34B;
  if (sel == 0x0F) PR(reg) = (uint8_t)((PR(reg) & 0xFB) | 0x04);
  else             PR(reg) = (uint8_t)(PR(reg) & 0xFB);
}

static void u4lb_sb_op_kick_drain(uint8_t v) {  /* 8992 */
  u4lb_sb_op_kick(v);
  u4lb_sb_op_run_drain(1);
}

static uint8_t u4lb_lane_gate(uint8_t lane) { return (uint8_t)(u4_work_buf[WB_LANE_EN] & (uint8_t)(1u << lane)); }

static void u4lb_routerop_settle(void) { phy_cc10_cmd_wait(2, 0, 0x65); }  /* 8501 */

static void u4lb_lp1_finalize(uint8_t lane) {
  __xdata uint8_t walk_idx;
  if (lb_settle_counter[lane] >= 0x10) { lb_loop1_state[lane] = LP1_PARKED; return; }
  lb_settle_counter[lane]++;
  walk_idx = (uint8_t)((lb_lane_desc_idx[lane] + 1) & 0x0F);
  lb_lane_desc_idx[lane] = walk_idx;
  u4_work_buf[0x1C + lane] = (uint8_t)((u4_work_buf[0x1C + lane] & 0xF0) | sb_lane_desc[(uint16_t)(walk_idx)]);
  u4_work_buf[0x1C + lane] |= 0x80;
  lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
}

static void u4lb_lp1_width_settle(uint8_t lane) {
  __xdata uint16_t widthA, neg;
  widthA = (uint16_t)(((uint16_t)lb_width_pairA[2*lane] << 8) | lb_width_pairA[0x1 + 2*lane]);
  if (widthA == 0) { u4lb_lp1_finalize(lane); return; }
  neg = (uint16_t)(((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO);
  if (lb_width_pairB[2*lane] == 0 && lb_width_pairB[0x1 + 2*lane] == u4_sb.lane_train_trigger) {
    if ((uint16_t)(widthA - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
    return;
  }
  if ((uint16_t)((widthA - 1) - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
}

/*
 * Active CL-walker: runs both lanes' loop1 state machine (lb_loop1_state[],
 * LP1_* states) on the route_enable_latch==0x04 path. Walks each lane from
 * width-init through lane-present detect, width settle, and CL0 bond, pushing
 * per-lane CL descriptors over sideband. Selected by u4lb_walk() below.
 */
static void u4lb_walk_route_active(void) {  /* 8000 */
  __xdata uint8_t lane, state;
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop1_state[lane];

    if (state == LP1_WIDTH_INIT) {
      u4_work_buf[0x1C + lane] &= 0xEF;
      u4_work_buf[0x1C + lane] &= 0x7F;
      u4_work_buf[0x1C + lane] &= 0xDF;
      lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
    }
    else if (state == LP1_ARM_WAIT_PUSH) {
      if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = LP1_LANE_PRESENT_SEL;
    }
    else if (state == LP1_LANE_PRESENT_SEL) {
      __xdata uint8_t selector = u4lb_routerop_poll();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0x80) == 0) {
          lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
        } else {
          SB_WR(SB_LANESEL, (uint8_t)(lane ? 2 : 1));
          lb_width_pairA[2*lane] = 0x00;
          lb_width_pairA[0x1 + 2*lane] = 0x00;
          u4_work_buf[0x1C + lane] |= 0x20;
          lb_loop1_state[lane] = LP1_SETTLE_CLEAR;
        }
        u4lb_routerop_settle();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
      }
    }
    else if (state == LP1_SETTLE_CLEAR) {
      lb_settle_counter[lane] = 0x00;
      lb_loop1_state[lane] = LP1_WIDTH_SETTLE_WALK;
    }
    else if (state == LP1_WIDTH_SETTLE_WALK) {
      if (u4lb_lane_present(lane) != 0 && lb_settle_counter[lane] != 0) {
        if (phy_lane_gate == 0) {
          SB_WR(lane ? 0x5A : 0x50, 0x01);
        }
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        lb_loop1_state[lane] = LP1_PARKED;
        if ((REG_PHY_ORIENT_C2C3 & 0x01) || (REG_VENDOR_CTRL_C343 & 0x01)) {
          if ((u4_work_buf[WB_LANE_EN] & 0x03) != 0) {
            u4_work_buf[0x1C] |= 0x10; u4_work_buf[0x1C] |= 0x40; u4_work_buf[0x1C] &= 0x7F;
            u4_work_buf[0x1D] |= 0x10; u4_work_buf[0x1D] |= 0x40; u4_work_buf[0x1D] &= 0x7F;
            lb_loop1_state[0x0] = LP1_PARKED; lb_loop1_state[0x1] = LP1_PARKED;
          }
        }
      } else {
        u4lb_lp1_width_settle(lane);
      }
    }
    else if (state == LP1_BOND_WAIT_PUSH) {
      if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = LP1_WIDTH_LATCH_SEL;
    }
    else if (state == LP1_WIDTH_LATCH_SEL) {
      __xdata uint8_t selector = u4lb_routerop_poll();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0xC0) == 0xC0 &&
            (snap & 0x0F) == (uint8_t)(u4_work_buf[0x1C + lane] & 0x0F)) {
          if (phy_lane_gate) u4lb_rxpll_reset_pulse();
          SB_WR(SB_LANESEL, (uint8_t)(lane ? 2 : 1));
          { __xdata uint16_t w = u4lb_read_lane_width_cnt();
            lb_width_pairA[2*lane] = (uint8_t)(w >> 8);
            lb_width_pairA[0x1 + 2*lane] = (uint8_t)w; }
          lb_width_pairB[2*lane] = 0x00;
          lb_width_pairB[0x1 + 2*lane] = u4_sb.lane_train_trigger;
          lb_loop1_state[lane] = LP1_FINALIZE_A;
        } else {
          lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
        }
        u4lb_routerop_settle();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
      }
    }
    else if (state == LP1_FINALIZE_A) {
      if (u4lb_lane_present(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        u4_work_buf[0x1C + lane] &= 0x7F;
      }
      lb_loop1_state[lane] = LP1_FINALIZE_B;
    }
    else if (state == LP1_FINALIZE_B) {
      if (u4lb_lane_present(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
      }
      u4_work_buf[0x1C + lane] &= 0x7F;
      lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
    }
    else if (state == LP1_BOND_WAIT_ACK) {
      if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = LP1_BONDED_MONITOR;
    }
    else if (state == LP1_BONDED_MONITOR) {
      __xdata uint8_t selector = u4lb_routerop_poll();
      if (selector == 0) {
        if ((u4_host_desc[0x4 + lane] & 0xC0) == 0x80) lb_loop1_state[lane] = LP1_WIDTH_SETTLE_WALK;
        else                                    lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
        u4lb_routerop_settle();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
      }
    }
  }
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == LP2_CL_INIT) {
      u4_work_buf[0x1E + lane] |= 0x80;
      u4_work_buf[0x1E + lane] &= 0xBF;
      lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
    } else if (state == LP2_CL_PUSH_WAIT) {
      if (u4lb_send_routerop() == 1) lb_loop2_state[lane] = LP2_CL_EVAL;
    } else if (state == LP2_CL_EVAL) {
      __xdata uint8_t selector = u4lb_routerop_poll();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x2 + lane];
        if ((snap >> 4) & 1) {
          lb_loop2_state[lane] = LP2_CL_IDLE;
        }
        else if (((snap >> 7) & 1) == 0) lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
        else {
          __xdata uint8_t cl_idx = (uint8_t)(snap & 0x0F), cap, cl_cfg_hi = 0, cl_cfg_lo = 0;
          lb_cl_value[lane] = cl_idx;
          u4_work_buf[0x1E + lane] &= 0xF0;
          u4_work_buf[0x1E + lane] |= cl_idx;
          u4_work_buf[0x1E + lane] |= 0x40;
          cap = u4_phy.lane_cap[lane];
          if ((cap >> 1) & 1) cl_cfg_lo = lb_cap_field[cl_idx];
          if (cap & 1) { __xdata uint16_t m = (uint16_t)(sb_lane_flip[cl_idx] * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
          SB_WR(SB_RATE_HI(lane), cl_cfg_hi);
          SB_WR(SB_RATE_LO(lane), cl_cfg_lo);
          u4lb_phy_cl_orient_strobe(cl_idx, lane);
          lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
        }
        u4lb_routerop_settle();
      } else if (selector == 2) lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
    } else if (state == LP2_CL_BOND_WAIT) {
      if (u4lb_send_routerop() == 1) lb_loop2_state[lane] = LP2_CL_BOND_MON;
    } else if (state == LP2_CL_BOND_MON) {
      __xdata uint8_t selector = u4lb_routerop_poll();
      if (selector == 0) {
        if ((u4_host_desc[0x2 + lane] >> 7) & 1) lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
        else { u4_work_buf[0x1E + lane] &= 0xBF; lb_loop2_state[lane] = LP2_CL_PUSH_WAIT; }
        u4lb_routerop_settle();
      } else if (selector == 2) lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
    }
  }
}

/*
 * Passive CL-walker: runs both lanes' loop2 state machine (lb_loop2_state[])
 * on the non-route_enable_latch==0x04 path, advancing each lane by polling the
 * router-op result and the lane-present/CL0 sideband status. Distinct machine
 * from the active walker (ROM 850b); its states are kept as bare hex because
 * the individual state meanings are not established from the decompilation.
 */
static void u4lb_walk_route_passive(void) {  /* 850b */
  __xdata uint8_t lane, state;
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == 0x11) {
      __xdata uint8_t r = u4lb_routerop_poll();
      if (r == 0) lb_loop2_state[lane] = 0x20;
      else if (r == 1) lb_loop2_state[lane] = 0x10;
    } else if (state == 0x20) {
      __xdata uint8_t r = u4lb_routerop_poll();
      if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x30; else lb_loop2_state[lane] = 0x20; }
      else if (r != 2) lb_loop2_state[lane] = 0x20;
    } else if (state == 0x21) {
      if (u4lb_lane_present(lane) == 0) lb_loop2_state[lane] = 0x40;
    } else if (state == 0x30) {
      if (u4lb_send_routerop() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 4) & 1)) lb_loop2_state[lane] = 0x00;
        else if ((snap >> 7) & 1) { lb_cl_value[lane] = (uint8_t)(snap & 0x0F); lb_loop2_state[lane] = 0x50; }
        else lb_loop2_state[lane] = 0x30;
      }
    } else if (state == 0x40) {
      u4lb_phy_cl_orient_strobe(lb_cl_value[lane], lane);
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
      if (u4lb_send_routerop() == 1) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x70; else lb_loop2_state[lane] = 0x60; }
    } else if (state == 0x61) {
      if (u4lb_send_routerop() == 1) lb_loop2_state[lane] = 0x71;
    } else if (state == 0x70) {
      if (u4lb_send_routerop() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 7) & 1)) lb_loop2_state[lane] = 0x30;
        else if (lb_cl_value[lane] != 0x07) lb_loop2_state[lane] = 0x30;
        else lb_loop2_state[lane] = 0x70;
      }
    } else {
      if (u4lb_lane_present(lane) == 0) lb_loop2_state[lane] = 0x11;
    }
  }
  if (lb_loop2_state[0x0] == 0 && lb_loop2_state[0x1] == 0) {
    if (u4_sb.walk_oneshot_flag == 0) {
      if (u4_work_buf[WB_LANE_EN] & 0x01) u4lb_sb_op_kick_drain(0x86);
      if ((u4_work_buf[WB_LANE_EN] >> 1) & 0x01) u4lb_sb_op_kick_drain(0xA6);
      u4_sb.walk_oneshot_flag = 1;
    }
    for (lane = 0; lane < 2; lane++) {
      if (!u4lb_lane_gate(lane)) continue;
      state = lb_loop1_state[lane];
      if (state == 0x10) {
        if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = 0x21;
      } else if (state == 0x20) {
        if (u4lb_send_routerop() == 1) {
          if ((lb_eq_status[lane] >> 7) & 1) { SB_WR(SB_LANESEL, (uint8_t)(u4lb_lane_present(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x30; }
          else lb_loop1_state[lane] = 0x20;
        }
      } else if (state == 0x21) {
        if (u4lb_routerop_poll() != 0) u4_work_buf[0x1C + lane] |= 0x10;
        lb_loop1_state[lane] = 0x40;
      } else if (state == 0x30) {
        if (u4lb_routerop_poll() != 0) {
          if (lb_settle_counter[lane] != 0) lb_loop1_state[lane] = 0x50;
          else lb_loop1_state[lane] = 0x60;
        }
      } else if (state == 0x40) {
        SB_WR(SB_LINK_REINIT_50, (uint8_t)(u4lb_lane_present(lane) ? 2 : 1));
        lb_loop1_state[lane] = 0x51;
      } else if (state == 0x50) {
        lb_loop2_scratch[lane] |= 0x10;
        if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = 0x52;
      } else if (state == 0x51) {
        __xdata uint8_t r = u4lb_routerop_poll();
        if (r == 0) { if (u4_host_desc[0x2] != 0) lb_loop1_state[lane] = 0x51; else lb_loop2_state[lane] = 0x00; }
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
        __xdata uint8_t r = u4lb_routerop_poll();
        if (r == 0) { if (u4_host_desc[0x2] == 0) { SB_WR(SB_LANESEL, (uint8_t)(u4lb_lane_present(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x90; } else lb_loop1_state[lane] = 0x70; }
        else if (r != 2) lb_loop1_state[lane] = 0x70;
      } else if (state == 0x80) {
        lb_loop1_state[lane] = 0xA0;
      } else if (state == 0x90) {
        lb_loop2_scratch[lane] &= 0x7F;
        if (u4lb_send_routerop() == 1) lb_loop1_state[lane] = 0xA1;
      } else if (state == 0xA0) {
        __xdata uint8_t r = u4lb_routerop_poll();
        if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop1_state[lane] = 0xB0; else lb_loop1_state[lane] = 0xA0; }
        else if (r != 2) lb_loop1_state[lane] = 0xA0;
      } else if (state == 0xA1) {
        if (u4lb_lane_present(lane) == 0) lb_loop2_state[lane] = 0x60;
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
}

static void u4lb_state_lane_bond(void) {
  DPX = 0x00;
  if (u4_sb.route_enable_latch == 0x04) u4lb_walk_route_active();
  else                    u4lb_walk_route_passive();
  DPX = 0x00;
}

static void u4lb_fsm_step(void) {  /* e672 */
  u4_fsm_state_t state = u4_sb.state;
  if (state == U4FSM_LANE_TRAIN) {
    u4lb_state_lane_train();
    return;
  }
  if (state == U4FSM_LANE_BOND) {
    if (lb_loop2_state[0x0] == 0 && lb_loop1_state[0x0] == 0 && lb_loop2_state[0x1] == 0 && lb_loop1_state[0x1] == 0) {
      u4lb_set_fsm_state(U4FSM_IDLE);
      return;
    }
    u4lb_state_lane_bond();
    return;
  }
  if (state == U4FSM_CONN_ROUT) {
    u4lb_cm_conn_routing_setup();
    return;
  }
}

#endif
