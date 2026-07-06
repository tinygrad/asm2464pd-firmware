#ifndef SB_H
#define SB_H


static void boot_phy_set_link_mode(uint8_t param);  /* dd42 */

static uint8_t P1_REG8_rd(uint16_t off) {
  uint8_t v;
  DPX = 0x01;
  v = XDATA_REG8V(off);
  DPX = 0x00;
  return v;
}
static void P1_REG8_wr(uint16_t off, uint8_t v) {
  DPX = 0x01;
  XDATA_REG8V(off) = v;
  DPX = 0x00;
}
#define P1_RD(off)      P1_REG8_rd((uint16_t)(off))
#define P1_WR(off, v)   P1_REG8_wr((uint16_t)(off), (uint8_t)(v))
#define P1_CLR(off, m)  P1_WR((off), P1_RD(off) & (uint8_t)~(m))
#define P1_SET(off, m)  P1_WR((off), P1_RD(off) | (uint8_t)(m))

#define SB_RD(off)      P1_REG8_rd((uint16_t)(0x2800u + (off)))
#define SB_WR(off, v)   P1_REG8_wr((uint16_t)(0x2800u + (off)), (uint8_t)(v))
#define SB_CLR(off, m)  SB_WR((off), SB_RD(off) & (uint8_t)~(m))
#define SB_SET(off, m)  SB_WR((off), SB_RD(off) | (uint8_t)(m))
#define SB_W1C(off, m)  do { if (SB_RD(off) & (m)) SB_WR(off, (m)); } while (0)
#define SB_W1C_PRINT(off, m, msg) do { if (SB_RD(off) & (m)) { SB_WR(off, (m)); uart_puts(msg); } } while (0)
#define SB_EDGE_ACK(off) do { SB_WR(off, 0x10); SB_WR(off, 0x20); SB_WR(off, 0x40); SB_WR(off, 0x08); } while (0)
#define DE_WR_CLEAR() do { P12_WR(DE_WR(0), 0x00); P12_WR(DE_WR(1), 0x00); P12_WR(DE_WR(2), 0x00); P12_WR(DE_WR(3), 0x00); } while (0)

#define P12_RD(off)     P1_REG8_rd((uint16_t)(0x1200u + (off)))
#define P12_WR(off, v)  P1_REG8_wr((uint16_t)(0x1200u + (off)), (uint8_t)(v))

static void u4lb_desc_emit_lanes(uint8_t ctrl_bits, uint8_t set_lanes);  /* ce23 */

static __code const uint8_t sb_lane_rate_desc[0x10] = {
  0x00,0x06,0x0B,0x0E,0x13,0x00,0x05,0x0A, 0x0E,0x11,0x00,0x05,0x08,0x0D,0x00,0x04
};

static __code const uint8_t sb_cap_field_desc[0x10] = {
  0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01, 0x01,0x01,0x03,0x03,0x03,0x03,0x07,0x02
};

static void sb_lane_flip_init(void) {
  uint8_t flip = REG_PHY_VENDOR_CTRL_C6DB & 0x01;

  P1_CLR(P1_LANE_FLIP(0), 0x10);
  P1_CLR(P1_LANE_FLIP(0), 0x40);
  P1_CLR(P1_LANE_FLIP(0), 0x80);

  if (u4_pd.enter_usb_accepted != 0 || u4_pd.connect_route_latch != 0) P1_CLR(P1_LANE_FLIP(0), 0x01);
  else                                    P1_SET(P1_LANE_FLIP(0), 0x01);

  if (flip) {
    P1_SET(P1_LANE_FLIP(2), 0x03);
    P1_SET(P1_LANE_FLIP(1), 0x03);
  } else {
    P1_CLR(P1_LANE_FLIP(2), 0x03);
    P1_CLR(P1_LANE_FLIP(1), 0x03);
  }

  if (u4_pd.enter_usb_accepted != 0 || u4_pd.connect_route_latch != 0) {
    P1_WR(P1_LANE_FLIP(1), (P1_RD(P1_LANE_FLIP(1)) & 0xEF) | 0x10);
    P1_CLR(P1_LANE_FLIP(1), 0x20);
    P1_SET(P1_LANE_FLIP(1), 0x80);
    REG_LINK_MODE_CTRL &= ~0x03;
  } else {
    P1_CLR(P1_LANE_FLIP(1), 0x10);
    P1_CLR(P1_LANE_FLIP(1), 0x80);
    REG_LINK_MODE_CTRL |= 0x03;
  }

  SB_WR(SB_LANE_CFG_D1, (SB_RD(SB_LANE_CFG_D1) & 0xEF) | 0x10);
  SB_WR(SB_PHY_CFG_49, 0xA0);
  u4_sb.conn_consequence_done = 0;

  SB_WR(SB_CFG_94, 0x02); SB_WR(SB_CFG_95, 0x71); SB_WR(SB_CFG_96, 0x00);
  SB_WR(SB_CFG_98, 0x3E); SB_WR(SB_CFG_99, 0x80);
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  REG_XFER2_DMA_CTRL &= 0xEF;
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_XFER2_DMA_CTRL = (REG_XFER2_DMA_CTRL & 0xF8) | 0x04;
  REG_XFER2_DMA_ADDR_LO = 0x00;
  REG_XFER2_DMA_ADDR_HI = 200;
  SB_CLR(SB_EVENT_CLR_CF, 0x01);
  SB_WR(SB_MASK_53, 0xFF);
  SB_WR(SB_MASK_5D, 0xFF);
  SB_CLR(SB_EVENT_CLR_27, 0x01);
  SB_WR(SB_CONNECT_STATE, (SB_RD(SB_CONNECT_STATE) & 0xFD) | 0x02);
  SB_CLR(SB_CONNECT_EVENT, 0x01);
  REG_INT_CTRL = (REG_INT_CTRL & 0xF7) | 0x08;
  SB_CLR(SB_EVENT_CLR_67, 0x40);
  mem_set(lb_cap_field, 0, 0x10);
  if (REG_LANE_RATE_C8FF == 0x04) {
    mem_copy(sb_lane_flip, sb_lane_rate_desc, 0x10);
    mem_copy(lb_cap_field, sb_cap_field_desc, 0x10);
  }
}

static __code const uint8_t drom_identity[0x64] = {
  0x4C,0x17,0x00,0x00,0x64,0x24,0x00,0x00, 0x41,0x50,0x50,0x20,0x45,0x4D,0x20,0x20,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0xD3,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};
static __code const uint8_t usb4_drom_window[0x7A] = {
  0x38,0xC0,0xF8,0x0B,0x03,0x03,0x03,0x4C, 0x17,0xC3,0x47,0xD5,0xF1,0x01,0x6D,0x00,
  0xD1,0xAD,0x01,0x00,0x01,0x01,0x02,0xC4, 0x08,0x81,0x80,0x02,0x00,0x00,0x00,0x00,
  0x08,0x82,0x90,0x01,0x00,0x00,0x00,0x00, 0x0B,0x83,0x20,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x17,0x01,0x74,0x69,0x6E, 0x79,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x1E,0x02,0x63,0x75,0x73,0x74,
  0x6F,0x6D,0x20,0x76,0x30,0x2E,0x31,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x03,0x08,0x00,0x0F,0x09,0x10,0x04,0xD1,
  0xAD,0x01,0x00,0x01,0x00,0x66,0x06,0x00, 0xC0,0x5A
};
static __code const uint8_t usb4_router_cfg_seed[] = {
  0x06, 0x00,0x00,0x00,0x03,
  0x09, 0x00,0x00,0x00,0x00,
  0x0A, 0x00,0x00,0x00,0x00,
  0x0B, 0x00,0x00,0x1E,0x02,
  0x0C, 0x63,0x75,0x73,0x74,
  0x0D, 0x6F,0x6D,0x20,0x76,
  0x0E, 0x30,0x2E,0x31,0x00,
  0x0F, 0x00,0x00,0x00,0x00,
  0x10, 0x00,0x00,0x00,0x00,
  0x11, 0x00,0x00,0x00,0x00,
  0x12, 0x00,0x00,0x00,0x00,
  0x13, 0x03,0x08,0x00,0x0F,
  0x14, 0x09,0x10,0x04,0xD1,
  0x15, 0xAD,0x01,0x00,0x01,
  0x16, 0x00,0x66,0x06,0x00,
  0x17, 0xC0,0x5A,0x00,0x00,
  0x18, 0x00,0x00,0x00,0x00,
  0x19, 0x00,0x00,0x00,0x00,
  0x1A, 0x25,0x00,0x00,0x02
};
static __code const uint8_t lane_descriptor_rom[0x10] = {
  0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0C,0x0C, 0x0C,0x0C,0x0C,0x07,0x07,0x07,0x07,0x07
};
static __code const uint8_t width_lut_rom[0x13] = {
  0x04,0x04,0x00,0x04,0x04,0x00,0x00,0x00, 0x04,0x04,0x01,0x00, 0x03,0x04,0x00,0x04, 0x00,0x00,0x10
};
static __code const uint8_t branchA_gate_rom[0x13] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x01
};

static void sb_rom_descriptor_load(void) {
  mem_copy((void __xdata *)P1_USB4_DROM_SHADOW_0240, usb4_drom_window, sizeof(usb4_drom_window));
  mem_copy(u4_work_buf, drom_identity, 0x64);
  u4_work_buf[0x4] = u4_cfg.product_pid_lo;
  u4_work_buf[0x5] = u4_cfg.product_pid_hi;
  if (!(u4_cfg.mode_flag & MODE_FLAG_VDM_ACK)) u4_work_buf[0x1B] &= ~0x02;
  mem_copy(sb_lane_desc, lane_descriptor_rom, 0x10);
  mem_copy(sb_width_lut, width_lut_rom, 0x13);
  mem_copy(sb_branchA_gate, branchA_gate_rom, 0x13);
  if (u4_cfg.cap20g_gate0) {
    u4_work_buf[WB_LANE_CAP] = (uint8_t)((u4_work_buf[WB_LANE_CAP] & 0xDF) | 0x20);
    if ((u4_pd.enter_usb_accepted != 0 && u4_pd.eudo_mode_confirm < 3) ||
        (u4_pd.connect_route_latch != 0 && (u4_pd.confirm_input_cf == 1 || u4_pd.confirm_input_cf == 2))) {
      u4_work_buf[WB_LANE_CAP] &= 0xDF;
    }
  }
  if (u4_cfg.cap20g_gate1 == 0) u4_work_buf[WB_LANE_CAP] &= 0xED;

  u4_sb.lane_width_latch0 = 1; u4_sb.route_query_response = 0; u4_sb.connect_present = 0; u4_sb.route_up_trigger = 0; u4_sb.walk_oneshot_flag = 0;
  u4_sb.state = U4FSM_IDLE; u4_sb.conn_routing_substate = CONNRT_PRINT_STATUS; lb_lane_desc_idx[0x0] = 0x0F; lb_lane_desc_idx[0x1] = 0x0F; u4_sb.coldboot_seed_gate = 1;
  u4_sb.routerop_resp_armed = 0; u4_sb.lane_bonded_flag = 0;
  SB_WR(SB_CONNECT_EVENT, 0x04);
  SB_WR(SB_CONN_EDGE(0), 0x08);
  SB_WR(SB_CONN_EDGE(1), 0x08);
  { uint8_t b; for (b = 1; b; b <<= 1) SB_WR(SB_PORT_SVC, b); }
  SB_WR(SB_BOND_EVENT, BOND_EVT_L0_FAIL); SB_WR(SB_BOND_EVENT, BOND_EVT_L1_FAIL);
  u4_sb.transport_edge_toggle = SB_RD(SB_TRANSPORT_STAT) & 0x01;
  u4_sb.link_edge_toggle = SB_RD(SB_LINK_EDGE_STAT) & 0x01;
  u4_sb.active_port_rr = (uint8_t)((SB_RD(SB_TRANSPORT_STAT) & 0x06) >> 1);
  u4_sb.routerop_push_token = 0;
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  SB_CLR(SB_PEER_CL0, 0x20);
  SB_WR(SB_DESC_CFG_8F, (SB_RD(SB_DESC_CFG_8F) & 0xEF) | 0x10);
}

static void sb_block_init(void) {
  uart_puts("[SB Init]");

  REG_PHY_RXPLL_RESET = 0; REG_PHY_CTRL_C20F = 0; REG_PHY_CDR_SEED_C210 = 0;
  REG_PHY_CDR_SEED_C211 = 0; REG_PHY_CDR_SEED_C212 = 0;
  REG_PHY_CDR_SEED_C214 = 0; REG_PHY_CDR_SEED_C215 = 0; REG_PHY_CDR_SEED_C216 = 0;
  REG_PHY_CDR_SEED_C217 = 0;

  P1_CLR(P1_LANE_FLIP(0), 0x10);
  P1_CLR(P1_LANE_FLIP(0), 0x40);
  P1_CLR(P1_LANE_FLIP(0), 0x80);
  P1_CLR(P1_LANE_FLIP(0), 0x01);

  SB_WR(SB_CONNECT_EVENT, 0x01); SB_WR(SB_CONNECT_EVENT, 0x02);
  SB_WR(SB_ROUTEROP_EVENT, 0x02);
  SB_WR(SB_BOND_EVENT, BOND_EVT_BONDED);
  SB_CLR(SB_CONNECT_STATE, 0x01); SB_WR(SB_CONNECT_STATE, (SB_RD(SB_CONNECT_STATE) & 0xFD) | 0x02);
  SB_CLR(SB_EVENT_CLR_29, 0x08);
  SB_CLR(SB_EVENT_CLR_2B, 0x08);
  SB_CLR(SB_EVENT_CLR_C8, 0xF0);
  SB_CLR(SB_EVENT_CLR_27, 0x02);
  SB_CLR(SB_EVENT_CLR_67, 0x81);
  SB_WR(SB_LINK_EDGE(0), 0x08);
  SB_WR(SB_LINK_EDGE(1), 0x08);
  SB_CLR(SB_EVENT_CLR_82, 0x08);
  SB_CLR(SB_EVENT_CLR_84, 0x08);
  SB_WR(SB_CL0_EVENT, CL0_EVT_L0); SB_WR(SB_CL0_EVENT, CL0_EVT_L1);
  SB_WR(SB_BOND_EVENT, BOND_EVT_L0_ABR2); SB_WR(SB_BOND_EVENT, BOND_EVT_L1_ABR2);
  SB_CLR(SB_EVENT_CLR_9F, 0x03);
  SB_CLR(SB_EVENT_CLR_67, 0x24);
  SB_WR(SB_CL0_EVENT, CL0_EVT_L0_TRAIN); SB_WR(SB_CL0_EVENT, CL0_EVT_L1_TRAIN);
  SB_CLR(SB_EVENT_CLR_9F, 0x30);
  u4_sb.conn_consequence_done = 0;

  sb_rom_descriptor_load();

  SB_WR(SB_ADP1_CTRL, (SB_RD(SB_ADP1_CTRL) & 0xBF) | 0x40);
  SB_WR(SB_ADP1_CTRL, (SB_RD(SB_ADP1_CTRL) & 0x7F) | 0x80);
  SB_CLR(SB_EVENT_CLR_C4, 0x02);
  SB_CLR(SB_EVENT_CLR_27, 0x14);

  REG_PHY_CONFIG &= ~0x08;
  REG_PHY_LANEA_C2C4 = (REG_PHY_LANEA_C2C4 & 0xBF) | 0x40;
  REG_PHY_LANEA_C2DC &= 0xC0;
  REG_PHY_LANEB_C344 = (REG_PHY_LANEB_C344 & 0xBF) | 0x40;
  REG_PHY_LANEB_C35C &= 0xC0;

  phy_lane_gate = 0; u4_phy.lane_cap[0x0] = 3; u4_phy.lane_cap[0x1] = 3;
  REG_PHY_ORIENT_C2C3 &= 0xFE; REG_PHY_ORIENT_C2C3 &= 0xFD;
  REG_PHY_ORIENT_C2C3 = (REG_PHY_ORIENT_C2C3 & 0xC3) | 0x1C; REG_PHY_ORIENT_C2C3 &= 0xBF;
  REG_PHY_LANEA_CDR_C2CB &= 0xFB;
  REG_VENDOR_CTRL_C343 &= 0xFE; REG_VENDOR_CTRL_C343 &= 0xFD;
  REG_VENDOR_CTRL_C343 = (REG_VENDOR_CTRL_C343 & 0xC3) | 0x1C; REG_VENDOR_CTRL_C343 &= 0xBF;
  REG_PHY_LANEB_CDR_C34B &= 0xFB;
  REG_PHY_LINK_CTRL_C21C = (REG_PHY_LINK_CTRL_C21C & 0xBF) | 0x40;
  REG_PHY_LINK_CTRL_C208 &= 0xBF;
  REG_PHY_ORIENT_C2C3 &= 0x7F;
  REG_VENDOR_CTRL_C343 &= 0x7F;
  SB_CLR(SB_PHY_CTRL_1D, 0x02);

  SB_WR(SB_KEYSTONE_BA, 0x3F);
  SB_WR(SB_KEYSTONE_BD, 0x3F);

}

#define ENG_DESC_WR_HI80(cur, v) do { \
  P12_WR((cur), (uint8_t)(v)); \
  P12_WR((uint8_t)((cur) + 1), (uint8_t)((P12_RD((uint8_t)((cur) + 1)) & 0x3F) | 0x80)); \
} while (0)  /* a30c */
#define ENG_DESC_WR_LO0F(cur, v) do { \
  P12_WR((cur), (uint8_t)(((v) & 0xF0) | 0x0F)); \
  P12_WR((uint8_t)((cur) + 1), (uint8_t)((P12_RD((uint8_t)((cur) + 1)) & 0x3F) | 0x80)); \
} while (0)  /* a308 */
#define ENG_DESC_WR_CLR(cur, v) do { \
  P12_WR((cur), (uint8_t)(v)); \
  P12_WR((uint8_t)((cur) + 1), (uint8_t)(P12_RD((uint8_t)((cur) + 1)) & 0xE0)); \
} while (0)  /* a2df */
#define ENG_DESC_WR_STROBE(cur, v) do { \
  P12_WR((cur), (uint8_t)(v)); \
  P12_WR((uint8_t)((cur) + 1), (uint8_t)((P12_RD((uint8_t)((cur) + 1)) & 0xC0) | 0x04)); \
  P12_WR((uint8_t)((cur) + 1), (uint8_t)((P12_RD((uint8_t)((cur) + 1)) & 0x3F) | 0x40)); \
} while (0)  /* a31c */
#define ENG_DESC_WR_RDBACK(cur, v) do { \
  P12_WR((cur), (uint8_t)(v)); \
  (void)P12_RD((uint8_t)((cur) + 1)); \
} while (0)  /* a348 */
#define ENG_DESC_WR_SELF40(cur, v) do { \
  P12_WR((cur), (uint8_t)(v)); \
  P12_WR((cur), (uint8_t)((P12_RD(cur) & 0x3F) | 0x40)); \
} while (0)  /* a327 */
#define ENG_DESC_WR_STROBE_RET(cur, v) \
  (P12_WR((cur), (uint8_t)(v)), (uint8_t)((P12_RD((uint8_t)((cur) + 1)) & 0xC0) | 0x04))  /* a31c */

static uint8_t u4c_sb_desc_commit(void) {
  uint8_t commit; uint16_t g;
  commit = (uint8_t)((P12_RD(DE_COMMIT) & 0x7F) | 0x80);
  P12_WR(DE_COMMIT, commit);
  P12_WR(DE_KICK, 0x01);
  for (g = 0; (P12_RD(DE_KICK) & 0x01) && g < 0x2000; g++) { }
  P12_WR(DE_CTRL, (uint8_t)(P12_RD(DE_CTRL) & 0xC0));
  DE_WR_CLEAR();
  return 0x00;
}

static void u4c_seed_workbuf(void) {  /* e8d6 */
  mem_set((void __xdata *)0x0994, 0, 0x50);
  XDATA_REG8V(0x0995) = 0x06;
  XDATA_REG8V(0x09DD) = 0x20;
  XDATA_REG8V(0x09DC) = 0x10;
  XDATA_REG8V(0x09E2) = 0x04;
  XDATA_REG8V(0x09E0) = 0x06;
}

static void u4c_desc_edge_engine(void) {  /* d4c8 */
  uint8_t v, commit_a, e;
  static __code const uint8_t edge_args[2][3] = {{0x40, 0x08, 0x09}, {0x04, 0x02, 0x05}};

  v = (uint8_t)((P12_RD(DE_LANESEL) & 0xF0) | 0x02);
  ENG_DESC_WR_RDBACK(0x34, v);
  ENG_DESC_WR_SELF40(0x35, (uint8_t)((P12_RD(DE_CTRL) & 0xC0) | 0x03));
  ENG_DESC_WR_CLR(0x36, 0x05);

  for (e = 0; e < 2; e++) {
    P12_WR(DE_WR(1), edge_args[e][0]); commit_a = u4c_sb_desc_commit();
    (void)ENG_DESC_WR_STROBE_RET(0x3D, (uint8_t)((commit_a & 0xF0) | edge_args[e][1]));
    ENG_DESC_WR_CLR(0x3F, edge_args[e][2]);
  }

  P12_WR(DE_WR(1), 0x40); (void)u4c_sb_desc_commit();
}

static void u4c_desc_edge_clear(void) {  /* e4d2 */
  uint8_t a;
  a = (uint8_t)((P12_RD(DE_LANESEL) & 0xF0) | 0x04);
  P12_WR(DE_LANESEL, a);
  a = P12_RD(DE_CTRL);
  P12_WR(DE_CTRL, (uint8_t)(a & 0x3F));
  ENG_DESC_WR_CLR(0x36, 0x00);
  P12_WR(DE_WR(2), 0x04);
  u4c_sb_desc_commit();
}

static void u4c_desc_block_cc(void) {  /* a2eb */
  P12_WR(DE_WR(0), 0xCC);
  P12_WR(DE_WR(1), 0xCC);
  P12_WR(DE_WR(2), 0x08);
  (void)u4c_sb_desc_commit();
}

static void u4c_desc_block_66(void) {  /* a365/a3d2 */
  P12_WR(DE_WR(0), 0x66);
  P12_WR(DE_WR(1), 0x66);
  P12_WR(DE_WR(2), 0x7B);
}

static void u4c_desc_seq_commit(void) {  /* cbf8 */
  uint8_t resid, pass;

  resid = 0x00;
  ENG_DESC_WR_SELF40(0x34, (uint8_t)(((((resid & 0xF0) | 0x0F) & 0xC0) | 0x01)));
  ENG_DESC_WR_CLR(0x35, 0x41);
  u4c_desc_block_cc();

  for (pass = 1; pass <= 2; pass++) {
    ENG_DESC_WR_SELF40(0x3D, pass);
    ENG_DESC_WR_CLR(0x3E, 0x42);
    u4c_desc_block_66();
    P12_WR(DE_WR(3), 0x01);
    (void)u4c_sb_desc_commit();

    if (pass == 1) {
      ENG_DESC_WR_SELF40(0x3D, 0x02);
      ENG_DESC_WR_CLR(0x3E, 0x41);
      u4c_desc_block_cc();
    }
  }
}

static __code const uint8_t u4c_crc32_table[256][4] = {  /* 5466 */
  {0x00,0x00,0x00,0x00},{0xF2,0x6B,0x83,0x03},{0xE1,0x3B,0x70,0xF7},{0x13,0x50,0xF3,0xF4},
  {0xC7,0x9A,0x97,0x1F},{0x35,0xF1,0x14,0x1C},{0x26,0xA1,0xE7,0xE8},{0xD4,0xCA,0x64,0xEB},
  {0x8A,0xD9,0x58,0xCF},{0x78,0xB2,0xDB,0xCC},{0x6B,0xE2,0x28,0x38},{0x99,0x89,0xAB,0x3B},
  {0x4D,0x43,0xCF,0xD0},{0xBF,0x28,0x4C,0xD3},{0xAC,0x78,0xBF,0x27},{0x5E,0x13,0x3C,0x24},
  {0x10,0x5E,0xC7,0x6F},{0xE2,0x35,0x44,0x6C},{0xF1,0x65,0xB7,0x98},{0x03,0x0E,0x34,0x9B},
  {0xD7,0xC4,0x50,0x70},{0x25,0xAF,0xD3,0x73},{0x36,0xFF,0x20,0x87},{0xC4,0x94,0xA3,0x84},
  {0x9A,0x87,0x9F,0xA0},{0x68,0xEC,0x1C,0xA3},{0x7B,0xBC,0xEF,0x57},{0x89,0xD7,0x6C,0x54},
  {0x5D,0x1D,0x08,0xBF},{0xAF,0x76,0x8B,0xBC},{0xBC,0x26,0x78,0x48},{0x4E,0x4D,0xFB,0x4B},
  {0x20,0xBD,0x8E,0xDE},{0xD2,0xD6,0x0D,0xDD},{0xC1,0x86,0xFE,0x29},{0x33,0xED,0x7D,0x2A},
  {0xE7,0x27,0x19,0xC1},{0x15,0x4C,0x9A,0xC2},{0x06,0x1C,0x69,0x36},{0xF4,0x77,0xEA,0x35},
  {0xAA,0x64,0xD6,0x11},{0x58,0x0F,0x55,0x12},{0x4B,0x5F,0xA6,0xE6},{0xB9,0x34,0x25,0xE5},
  {0x6D,0xFE,0x41,0x0E},{0x9F,0x95,0xC2,0x0D},{0x8C,0xC5,0x31,0xF9},{0x7E,0xAE,0xB2,0xFA},
  {0x30,0xE3,0x49,0xB1},{0xC2,0x88,0xCA,0xB2},{0xD1,0xD8,0x39,0x46},{0x23,0xB3,0xBA,0x45},
  {0xF7,0x79,0xDE,0xAE},{0x05,0x12,0x5D,0xAD},{0x16,0x42,0xAE,0x59},{0xE4,0x29,0x2D,0x5A},
  {0xBA,0x3A,0x11,0x7E},{0x48,0x51,0x92,0x7D},{0x5B,0x01,0x61,0x89},{0xA9,0x6A,0xE2,0x8A},
  {0x7D,0xA0,0x86,0x61},{0x8F,0xCB,0x05,0x62},{0x9C,0x9B,0xF6,0x96},{0x6E,0xF0,0x75,0x95},
  {0x41,0x7B,0x1D,0xBC},{0xB3,0x10,0x9E,0xBF},{0xA0,0x40,0x6D,0x4B},{0x52,0x2B,0xEE,0x48},
  {0x86,0xE1,0x8A,0xA3},{0x74,0x8A,0x09,0xA0},{0x67,0xDA,0xFA,0x54},{0x95,0xB1,0x79,0x57},
  {0xCB,0xA2,0x45,0x73},{0x39,0xC9,0xC6,0x70},{0x2A,0x99,0x35,0x84},{0xD8,0xF2,0xB6,0x87},
  {0x0C,0x38,0xD2,0x6C},{0xFE,0x53,0x51,0x6F},{0xED,0x03,0xA2,0x9B},{0x1F,0x68,0x21,0x98},
  {0x51,0x25,0xDA,0xD3},{0xA3,0x4E,0x59,0xD0},{0xB0,0x1E,0xAA,0x24},{0x42,0x75,0x29,0x27},
  {0x96,0xBF,0x4D,0xCC},{0x64,0xD4,0xCE,0xCF},{0x77,0x84,0x3D,0x3B},{0x85,0xEF,0xBE,0x38},
  {0xDB,0xFC,0x82,0x1C},{0x29,0x97,0x01,0x1F},{0x3A,0xC7,0xF2,0xEB},{0xC8,0xAC,0x71,0xE8},
  {0x1C,0x66,0x15,0x03},{0xEE,0x0D,0x96,0x00},{0xFD,0x5D,0x65,0xF4},{0x0F,0x36,0xE6,0xF7},
  {0x61,0xC6,0x93,0x62},{0x93,0xAD,0x10,0x61},{0x80,0xFD,0xE3,0x95},{0x72,0x96,0x60,0x96},
  {0xA6,0x5C,0x04,0x7D},{0x54,0x37,0x87,0x7E},{0x47,0x67,0x74,0x8A},{0xB5,0x0C,0xF7,0x89},
  {0xEB,0x1F,0xCB,0xAD},{0x19,0x74,0x48,0xAE},{0x0A,0x24,0xBB,0x5A},{0xF8,0x4F,0x38,0x59},
  {0x2C,0x85,0x5C,0xB2},{0xDE,0xEE,0xDF,0xB1},{0xCD,0xBE,0x2C,0x45},{0x3F,0xD5,0xAF,0x46},
  {0x71,0x98,0x54,0x0D},{0x83,0xF3,0xD7,0x0E},{0x90,0xA3,0x24,0xFA},{0x62,0xC8,0xA7,0xF9},
  {0xB6,0x02,0xC3,0x12},{0x44,0x69,0x40,0x11},{0x57,0x39,0xB3,0xE5},{0xA5,0x52,0x30,0xE6},
  {0xFB,0x41,0x0C,0xC2},{0x09,0x2A,0x8F,0xC1},{0x1A,0x7A,0x7C,0x35},{0xE8,0x11,0xFF,0x36},
  {0x3C,0xDB,0x9B,0xDD},{0xCE,0xB0,0x18,0xDE},{0xDD,0xE0,0xEB,0x2A},{0x2F,0x8B,0x68,0x29},
  {0x82,0xF6,0x3B,0x78},{0x70,0x9D,0xB8,0x7B},{0x63,0xCD,0x4B,0x8F},{0x91,0xA6,0xC8,0x8C},
  {0x45,0x6C,0xAC,0x67},{0xB7,0x07,0x2F,0x64},{0xA4,0x57,0xDC,0x90},{0x56,0x3C,0x5F,0x93},
  {0x08,0x2F,0x63,0xB7},{0xFA,0x44,0xE0,0xB4},{0xE9,0x14,0x13,0x40},{0x1B,0x7F,0x90,0x43},
  {0xCF,0xB5,0xF4,0xA8},{0x3D,0xDE,0x77,0xAB},{0x2E,0x8E,0x84,0x5F},{0xDC,0xE5,0x07,0x5C},
  {0x92,0xA8,0xFC,0x17},{0x60,0xC3,0x7F,0x14},{0x73,0x93,0x8C,0xE0},{0x81,0xF8,0x0F,0xE3},
  {0x55,0x32,0x6B,0x08},{0xA7,0x59,0xE8,0x0B},{0xB4,0x09,0x1B,0xFF},{0x46,0x62,0x98,0xFC},
  {0x18,0x71,0xA4,0xD8},{0xEA,0x1A,0x27,0xDB},{0xF9,0x4A,0xD4,0x2F},{0x0B,0x21,0x57,0x2C},
  {0xDF,0xEB,0x33,0xC7},{0x2D,0x80,0xB0,0xC4},{0x3E,0xD0,0x43,0x30},{0xCC,0xBB,0xC0,0x33},
  {0xA2,0x4B,0xB5,0xA6},{0x50,0x20,0x36,0xA5},{0x43,0x70,0xC5,0x51},{0xB1,0x1B,0x46,0x52},
  {0x65,0xD1,0x22,0xB9},{0x97,0xBA,0xA1,0xBA},{0x84,0xEA,0x52,0x4E},{0x76,0x81,0xD1,0x4D},
  {0x28,0x92,0xED,0x69},{0xDA,0xF9,0x6E,0x6A},{0xC9,0xA9,0x9D,0x9E},{0x3B,0xC2,0x1E,0x9D},
  {0xEF,0x08,0x7A,0x76},{0x1D,0x63,0xF9,0x75},{0x0E,0x33,0x0A,0x81},{0xFC,0x58,0x89,0x82},
  {0xB2,0x15,0x72,0xC9},{0x40,0x7E,0xF1,0xCA},{0x53,0x2E,0x02,0x3E},{0xA1,0x45,0x81,0x3D},
  {0x75,0x8F,0xE5,0xD6},{0x87,0xE4,0x66,0xD5},{0x94,0xB4,0x95,0x21},{0x66,0xDF,0x16,0x22},
  {0x38,0xCC,0x2A,0x06},{0xCA,0xA7,0xA9,0x05},{0xD9,0xF7,0x5A,0xF1},{0x2B,0x9C,0xD9,0xF2},
  {0xFF,0x56,0xBD,0x19},{0x0D,0x3D,0x3E,0x1A},{0x1E,0x6D,0xCD,0xEE},{0xEC,0x06,0x4E,0xED},
  {0xC3,0x8D,0x26,0xC4},{0x31,0xE6,0xA5,0xC7},{0x22,0xB6,0x56,0x33},{0xD0,0xDD,0xD5,0x30},
  {0x04,0x17,0xB1,0xDB},{0xF6,0x7C,0x32,0xD8},{0xE5,0x2C,0xC1,0x2C},{0x17,0x47,0x42,0x2F},
  {0x49,0x54,0x7E,0x0B},{0xBB,0x3F,0xFD,0x08},{0xA8,0x6F,0x0E,0xFC},{0x5A,0x04,0x8D,0xFF},
  {0x8E,0xCE,0xE9,0x14},{0x7C,0xA5,0x6A,0x17},{0x6F,0xF5,0x99,0xE3},{0x9D,0x9E,0x1A,0xE0},
  {0xD3,0xD3,0xE1,0xAB},{0x21,0xB8,0x62,0xA8},{0x32,0xE8,0x91,0x5C},{0xC0,0x83,0x12,0x5F},
  {0x14,0x49,0x76,0xB4},{0xE6,0x22,0xF5,0xB7},{0xF5,0x72,0x06,0x43},{0x07,0x19,0x85,0x40},
  {0x59,0x0A,0xB9,0x64},{0xAB,0x61,0x3A,0x67},{0xB8,0x31,0xC9,0x93},{0x4A,0x5A,0x4A,0x90},
  {0x9E,0x90,0x2E,0x7B},{0x6C,0xFB,0xAD,0x78},{0x7F,0xAB,0x5E,0x8C},{0x8D,0xC0,0xDD,0x8F},
  {0xE3,0x30,0xA8,0x1A},{0x11,0x5B,0x2B,0x19},{0x02,0x0B,0xD8,0xED},{0xF0,0x60,0x5B,0xEE},
  {0x24,0xAA,0x3F,0x05},{0xD6,0xC1,0xBC,0x06},{0xC5,0x91,0x4F,0xF2},{0x37,0xFA,0xCC,0xF1},
  {0x69,0xE9,0xF0,0xD5},{0x9B,0x82,0x73,0xD6},{0x88,0xD2,0x80,0x22},{0x7A,0xB9,0x03,0x21},
  {0xAE,0x73,0x67,0xCA},{0x5C,0x18,0xE4,0xC9},{0x4F,0x48,0x17,0x3D},{0xBD,0x23,0x94,0x3E},
  {0xF3,0x6E,0x6F,0x75},{0x01,0x05,0xEC,0x76},{0x12,0x55,0x1F,0x82},{0xE0,0x3E,0x9C,0x81},
  {0x34,0xF4,0xF8,0x6A},{0xC6,0x9F,0x7B,0x69},{0xD5,0xCF,0x88,0x9D},{0x27,0xA4,0x0B,0x9E},
  {0x79,0xB7,0x37,0xBA},{0x8B,0xDC,0xB4,0xB9},{0x98,0x8C,0x47,0x4D},{0x6A,0xE7,0xC4,0x4E},
  {0xBE,0x2D,0xA0,0xA5},{0x4C,0x46,0x23,0xA6},{0x5F,0x16,0xD0,0x52},{0xAD,0x7D,0x53,0x51},
};

static void u4c_transport_reg_reinit(void) {  /* dcb4 */
  uint8_t idx;

  P12_WR(DE_TRANSPORT(2), 0x01);
  P12_WR(DE_TRANSPORT(3), 0x08);
  P12_WR(DE_TRANSPORT(0), (uint8_t)(P12_RD(DE_TRANSPORT(0)) & 0xF7));
  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xFE));
  REG_INT_CTRL = (uint8_t)((REG_INT_CTRL & 0xFD) | 0x02);
  P12_WR(DE_TRANSPORT(1), (uint8_t)(P12_RD(DE_TRANSPORT(1)) & 0xBF));
  P12_WR(DE_ENG_RESET_7A, (uint8_t)((P12_RD(DE_ENG_RESET_7A) & 0xFE) | 0x01));
  PR(0x023F) = 0;
  PR(0x0443) = 0; PR(0x0444) = 0; PR(0x0445) = 0;
  PR(0x0448) = 0;
  PR(0x0446) = 0;
  PR(0x0449) = 0; PR(0x044A) = 0;

  {
    static __xdata uint8_t acc[4];
    static __xdata uint8_t ci, climit;
    acc[0] = 0xFF; acc[1] = 0xFF; acc[2] = 0xFF; acc[3] = 0xFF;
    climit = PR(0x024E);
    for (ci = 0; ci < climit; ci++) {
      idx = (uint8_t)(acc[3] ^ PR((uint16_t)(0x024D + ci)));
      acc[3] = acc[2]; acc[2] = acc[1]; acc[1] = acc[0]; acc[0] = 0x00;
      acc[0] ^= u4c_crc32_table[idx][0];
      acc[1] ^= u4c_crc32_table[idx][1];
      acc[2] ^= u4c_crc32_table[idx][2];
      acc[3] ^= u4c_crc32_table[idx][3];
    }
    PR(0x0249) = (uint8_t)~acc[3];
    PR(0x024A) = (uint8_t)~acc[2];
    PR(0x024B) = (uint8_t)~acc[1];
    PR(0x024C) = (uint8_t)~acc[0];
  }
}

static void u4c_set_sb1c_usb_mode(void) {  /* edbd */
  if (u4_pd.enter_usb_accepted == 0) SB_SET(SB_USB_MODE, 0x01);
  else                 SB_CLR(SB_USB_MODE, 0x01);
}

static void u4c_desc_engine_reset(void) {  /* e5b0 */
  P12_WR(DE_TRANSPORT(0), (uint8_t)(P12_RD(DE_TRANSPORT(0)) & 0xFE));
  P12_WR(DE_ENG_RESET_03, 0x80);
  P12_WR(DE_ENG_RESET_90, (uint8_t)(P12_RD(DE_ENG_RESET_90) & 0xFB));
  P12_WR(DE_ENG_RESET_8F, 0x80);
  P12_WR(DE_ENG_RESET_90, (uint8_t)(P12_RD(DE_ENG_RESET_90) & 0xDF));
  P12_WR(DE_ENG_RESET_8F, 0x20);
}

static void u4c_lane_width_desc(uint8_t width_byte) {  /* ccb3 */
  uint8_t gate;
  (void)P12_RD(DE_LANESEL);
  ENG_DESC_WR_HI80(0x34, (uint8_t)((width_byte & 0xF0) | 0x0E));
  ENG_DESC_WR_CLR(0x35, 0x01);
  P12_WR(DE_WR(0), 0x00); P12_WR(DE_WR(1), 0x01);
  P12_WR(DE_WR(2), 0x01); P12_WR(DE_WR(3), 0x00);
  u4c_sb_desc_commit();
  gate = u4_cfg.lane_gate_sel;
  if (!(gate & 0x01)) {
    (void)P12_RD(DE_LANESEL);
    ENG_DESC_WR_STROBE(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    ENG_DESC_WR_CLR(0x35, 0x02);
    u4c_sb_desc_commit();
  }
  gate = u4_cfg.lane_gate_sel;
  if (!((gate >> 1) & 0x01)) {
    (void)P12_RD(DE_LANESEL);
    ENG_DESC_WR_RDBACK(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    ENG_DESC_WR_SELF40(0x35, (uint8_t)((gate & 0xC0) | 0x03));
    ENG_DESC_WR_CLR(0x35, 0x02);
    u4c_sb_desc_commit();
  }
}

static void sb_eng_data_init(void) {  /* bbc7 */
  uint8_t i;
  for (i = 0; i < 9; i++) P12_WR((uint8_t)(DE_ENG_DATA_BASE + i), 0x00);

  XDATA_REG8V(0x09F9) = 0x87;
  XDATA_REG8V(0x09FB) = 0x03;  /* stock lane-gate status: both optional ccb3 descriptors disabled */
  XDATA_REG8V(0x0A57) = u4_cfg.product_pid_lo;
  XDATA_REG8V(0x0A58) = u4_cfg.product_pid_hi;

  u4_p12.data_hi = 0x17; u4_p12.data_lo = 0x4C;
  u4_p12.p12_desc_a0 = 0x03; u4_p12.p12_desc_a1 = 0x03;
  u4_p12.p12_desc_b0 = 0xC0; u4_p12.p12_desc_b1 = 0xF8;
  u4_p12.p12_desc_b2 = 0x0B; u4_p12.p12_desc_b3 = 0x03;

  XDATA_REG8V(0x0B13) = u4_p12.p12_desc_b0;
  XDATA_REG8V(0x0B14) = u4_p12.p12_desc_b1;
  XDATA_REG8V(0x0B15) = u4_p12.p12_desc_b2;
  XDATA_REG8V(0x0B16) = u4_p12.p12_desc_b3;
  XDATA_REG8V(0x0B17) = u4_p12.p12_desc_a0;
  XDATA_REG8V(0x0B18) = u4_p12.p12_desc_a1;
  XDATA_REG8V(0x0B19) = u4_p12.data_lo;
  XDATA_REG8V(0x0B1A) = u4_p12.data_hi;
}

static void u4c_identity_desc_emit(uint8_t width_byte) {  /* c270 */
  (void)P12_RD(DE_LANESEL);
  ENG_DESC_WR_LO0F(0x34, width_byte);
  ENG_DESC_WR_CLR(0x35, 0x00);
  P12_WR(DE_WR(0), u4_p12.data_lo);
  P12_WR(DE_WR(1), u4_p12.data_hi);
  P12_WR(DE_WR(2), u4_cfg.product_pid_lo);
  P12_WR(DE_WR(3), u4_cfg.product_pid_hi);
  (void)u4c_sb_desc_commit();
  (void)P12_RD(DE_LANESEL);
  ENG_DESC_WR_LO0F(0x34, u4_cfg.product_pid_hi);
  ENG_DESC_WR_CLR(0x35, 0x07);
  P12_WR(DE_WR(0), u4_p12.p12_desc_a0);
  P12_WR(DE_WR(1), u4_p12.p12_desc_a1);
  P12_WR(DE_WR(2), u4_p12.data_lo);
  P12_WR(DE_WR(3), u4_p12.data_hi);
  (void)u4c_sb_desc_commit();
  (void)P12_RD(DE_LANESEL);
  ENG_DESC_WR_LO0F(0x34, u4_p12.data_hi);
  ENG_DESC_WR_CLR(0x35, 0x08);
  P12_WR(DE_WR(0), u4_p12.p12_desc_b0);
  P12_WR(DE_WR(1), u4_p12.p12_desc_b1);
  P12_WR(DE_WR(2), u4_p12.p12_desc_b2);
  P12_WR(DE_WR(3), u4_p12.p12_desc_b3);
  (void)u4c_sb_desc_commit();
}

/*
 * USB4 router config-space (CS_25) access primitives, ported from stock ROM.
 * The host reads/writes router config dwords through the page-1 0x1234-0x1243
 * register file; u4c_cs_ptr is the current CS register index the wr/rd helpers
 * operate on. The u4c_cs_a3xx/e7f8 steps encode individual protocol fields and
 * keep their ROM addresses in the name for cross-referencing the stock decomp.
 */
static volatile uint8_t __xdata u4c_cs_ptr;

static uint8_t u4c_cs_read34(void) {
  u4c_cs_ptr = 0x34;
  return P12_RD(DE_LANESEL);
}
static uint8_t u4c_cs_read35(void) {
  u4c_cs_ptr = 0x35;
  return P12_RD(DE_CTRL);
}
static void u4c_cs_wr(uint8_t v) {
  P12_WR(u4c_cs_ptr, v);
}
static uint8_t u4c_cs_rd(void) {
  return P12_RD(u4c_cs_ptr);
}
static uint8_t u4c_cs_a308(uint8_t a) {
  a = (uint8_t)((a & 0xF0) | 0x0F);
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  a = (uint8_t)((u4c_cs_rd() & 0x3F) | 0x80);
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  return a;
}
static uint8_t u4c_cs_a30c(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  a = (uint8_t)((u4c_cs_rd() & 0x3F) | 0x80);
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  return a;
}
static uint8_t u4c_cs_a2df(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  a = (uint8_t)(u4c_cs_rd() & 0xE0);
  u4c_cs_wr(a);
  return a;
}
static uint8_t u4c_cs_a2de(uint8_t a) {
  u4c_cs_ptr++;
  return u4c_cs_a2df(a);
}
static uint8_t u4c_cs_commit_read34(void) {
  u4c_sb_desc_commit();
  return u4c_cs_read34();
}
static uint8_t u4c_cs_a2f9(uint8_t a) {
  u4c_cs_wr(a);
  return u4c_cs_commit_read34();
}
static uint8_t u4c_cs_a2f8(uint8_t a) {
  u4c_cs_ptr++;
  return u4c_cs_a2f9(a);
}
static uint8_t u4c_cs_a31c(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  a = (uint8_t)((u4c_cs_rd() & 0xC0) | 0x04);
  u4c_cs_wr(a);
  a = (uint8_t)((u4c_cs_rd() & 0x3F) | 0x40);
  u4c_cs_wr(a);
  return a;
}
static uint8_t u4c_cs_a327(uint8_t a) {
  u4c_cs_wr(a);
  a = (uint8_t)((u4c_cs_rd() & 0x3F) | 0x40);
  u4c_cs_wr(a);
  return a;
}
static uint8_t u4c_cs_a348(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  return u4c_cs_rd();
}
static uint8_t u4c_cs_a369(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  return a;
}
static uint8_t u4c_cs_a3cb(void) {
  uint8_t a = XDATA_REG8V(0x0B19);
  u4c_cs_wr(a);
  return a;
}
static uint8_t u4c_cs_a3d4(uint8_t a) {
  u4c_cs_wr(a);
  u4c_cs_ptr++;
  return 0x01;
}
static void u4c_cs_e7f8(uint8_t a) {
  u4c_cs_wr(a);
  u4c_sb_desc_commit();
}
static void u4c_router_cfg_dword_write(uint8_t idx, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  uint8_t a;
  a = u4c_cs_read34();
  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(idx);
  u4c_cs_ptr = 0x3C;
  u4c_cs_wr(b0);
  u4c_cs_ptr++;
  u4c_cs_wr(b1);
  u4c_cs_ptr++;
  u4c_cs_wr(b2);
  u4c_cs_ptr++;
  u4c_cs_wr(b3);
  u4c_sb_desc_commit();
}
static uint32_t u4c_router_cfg_dword_read(uint8_t idx) {
  uint8_t a, b0, b1, b2, b3;
  uint16_t g;
  a = u4c_cs_read34();
  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(idx);
  P12_WR(DE_COMMIT, (uint8_t)(P12_RD(DE_COMMIT) & 0x7F));
  P12_WR(DE_KICK, 0x01);
  for (g = 0; (P12_RD(DE_KICK) & 0x01) && g < 0x2000; g++) { }
  b0 = P12_RD(DE_RD(0));
  b1 = P12_RD(DE_RD(1));
  b2 = P12_RD(DE_RD(2));
  b3 = P12_RD(DE_RD(3));
  P12_WR(DE_CTRL, (uint8_t)(P12_RD(DE_CTRL) & 0xC0));
  DE_WR_CLEAR();
  return ((uint32_t)b0) | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}
static void u4c_router_cfg_u32_write(uint8_t idx, uint32_t v) {
  u4c_router_cfg_dword_write(idx,
                             (uint8_t)v,
                             (uint8_t)(v >> 8),
                             (uint8_t)(v >> 16),
                             (uint8_t)(v >> 24));
}
static uint8_t u4c_drom_byte(uint16_t off) {
  if (off >= sizeof(usb4_drom_window)) return 0x00;
  return usb4_drom_window[off];
}
static void u4c_native_drom_read(uint32_t metadata) {
  uint8_t i, dwords;
  uint16_t byte_off;

  dwords = (uint8_t)((metadata >> 15) & 0x1F);
  if (dwords == 0 || dwords > 16) dwords = 16;
  byte_off = (uint16_t)(((metadata >> 2) & 0x1FFF) << 2);
  for (i = 0; i < dwords; i++) {
    uint16_t off = (uint16_t)(byte_off + ((uint16_t)i << 2));
    u4c_router_cfg_dword_write((uint8_t)(0x09 + i),
                               u4c_drom_byte(off),
                               u4c_drom_byte((uint16_t)(off + 1)),
                               u4c_drom_byte((uint16_t)(off + 2)),
                               u4c_drom_byte((uint16_t)(off + 3)));
  }
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_OPCODE, USB4_ROUTER_OP_DROM_READ);
}
static void u4c_native_buffer_alloc(void) {
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_DATA0, ((uint32_t)32 << 16) | 0x0001UL);
  u4c_router_cfg_u32_write((uint8_t)(USB4_ROUTER_CS_DATA0 + 1), ((uint32_t)1 << 16) | 0x0002UL);
  u4c_router_cfg_u32_write((uint8_t)(USB4_ROUTER_CS_DATA0 + 2), ((uint32_t)1 << 16) | 0x0003UL);
  u4c_router_cfg_u32_write((uint8_t)(USB4_ROUTER_CS_DATA0 + 3), ((uint32_t)32 << 16) | 0x0004UL);
  u4c_router_cfg_u32_write((uint8_t)(USB4_ROUTER_CS_DATA0 + 4), ((uint32_t)32 << 16) | 0x0005UL);
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_METADATA, 0x00000005UL);
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_OPCODE, USB4_ROUTER_OP_BUFFER_ALLOC);
}
static void u4c_native_hw_status(void) {
  static __xdata uint16_t shunt_raw, bus_raw;
  uint16_t voltage_mv;
  int16_t current_ma;

  shunt_raw = 0;
  bus_raw = 0;
  (void)ina231_read_u16(INA231_REG_SHUNT, &shunt_raw);
  (void)ina231_read_u16(INA231_REG_BUS, &bus_raw);
  voltage_mv = (uint16_t)(((uint32_t)bus_raw * 125) / 100);
  current_ma = (int16_t)(((int32_t)(int16_t)shunt_raw * 2500) / INA231_SHUNT_UOHM);

  u4c_router_cfg_u32_write(USB4_ROUTER_CS_DATA0,
                           ((uint32_t)(uint16_t)current_ma << 16) | voltage_mv);
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_METADATA, 0x00000001UL);
  u4c_router_cfg_u32_write(USB4_ROUTER_CS_OPCODE, USB4_ROUTER_OP_TINY_HW_STATUS);
}
static void u4c_native_routerop_service(void) {
  uint32_t op, metadata;
  if (!u4_boot.sb_asserted) return;
  op = u4c_router_cfg_dword_read(USB4_ROUTER_CS_OPCODE);
  if (!(op & USB4_ROUTER_OP_OV)) return;

  metadata = u4c_router_cfg_dword_read(USB4_ROUTER_CS_METADATA);
  switch ((uint8_t)op) {
  case USB4_ROUTER_OP_DROM_READ:
    u4c_native_drom_read(metadata);
    break;
  case USB4_ROUTER_OP_NVM_SECTOR_SIZE:
    u4c_router_cfg_u32_write(USB4_ROUTER_CS_METADATA, 0x00000000UL);
    u4c_router_cfg_u32_write(USB4_ROUTER_CS_OPCODE, 0x02000000UL | USB4_ROUTER_OP_NVM_SECTOR_SIZE);
    break;
  case USB4_ROUTER_OP_BUFFER_ALLOC:
    u4c_native_buffer_alloc();
    break;
  case USB4_ROUTER_OP_TINY_HW_STATUS:
    u4c_native_hw_status();
    break;
  default:
    u4c_router_cfg_u32_write(USB4_ROUTER_CS_OPCODE, (op & 0x0000FFFFUL) | USB4_ROUTER_OP_UNSUPPORTED);
    break;
  }
}
static void u4c_router_cfg_seed_usb4(void) {
  uint8_t base;
  for (base = 0; base < sizeof(usb4_router_cfg_seed); base = (uint8_t)(base + 5)) {
    u4c_router_cfg_dword_write(usb4_router_cfg_seed[base],
                               usb4_router_cfg_seed[(uint8_t)(base + 1)],
                               usb4_router_cfg_seed[(uint8_t)(base + 2)],
                               usb4_router_cfg_seed[(uint8_t)(base + 3)],
                               usb4_router_cfg_seed[(uint8_t)(base + 4)]);
  }
}
static void u4c_cs_ce20(uint16_t desc_addr, uint8_t val) {
  XDATA_REG8V(desc_addr) = val;
  u4lb_desc0_lanemask = XDATA_REG8V(0x0B34);
  u4lb_desc1 = XDATA_REG8V(0x0B35);
  u4lb_desc2 = XDATA_REG8V(0x0B36);
  u4lb_desc3 = XDATA_REG8V(0x0B37);
  u4lb_desc_emit_lanes((uint8_t)(P12_RD(DE_CTRL) & 0x3F), 1);
  mem_set((void __xdata *)0x0B34, 0, 4);
}

static void u4c_identity_desc_emit_stock(void) {  /* c270 */
  uint8_t a;
  a = u4c_cs_read34();
  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x00);
  u4c_cs_ptr = 0x3C;
  u4c_cs_a3cb();
  a = XDATA_REG8V(0x0B1A); u4c_cs_ptr++; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0A57); u4c_cs_ptr++; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0A58); a = u4c_cs_a2f8(a);

  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x07);
  a = XDATA_REG8V(0x0B17); u4c_cs_ptr = 0x3C; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0B18); u4c_cs_ptr++; u4c_cs_wr(a);
  u4c_cs_ptr++;
  u4c_cs_a3cb();
  a = XDATA_REG8V(0x0B1A); a = u4c_cs_a2f8(a);

  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x08);
  a = XDATA_REG8V(0x0B13); u4c_cs_ptr = 0x3C; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0B14); u4c_cs_ptr++; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0B15); u4c_cs_ptr++; u4c_cs_wr(a);
  a = XDATA_REG8V(0x0B16); u4c_cs_ptr++; u4c_cs_wr(a);
  u4c_sb_desc_commit();
}

static void u4c_lane_width_desc_stock(void) {  /* ccb3 */
  uint8_t a;
  a = u4c_cs_read34();
  a = (uint8_t)((a & 0xF0) | 0x0E);
  u4c_cs_a30c(a);
  u4c_cs_a2df(0x01);
  u4c_cs_ptr = 0x3C;
  a = 0x00; u4c_cs_wr(a);
  u4c_cs_ptr++;
  a = 0x01; u4c_cs_a369(a);
  a = 0x00; u4c_cs_wr(a);
  u4c_sb_desc_commit();

  a = XDATA_REG8V(0x09FB);
  if (!(a & 0x01)) {
    a = u4c_cs_read34();
    a = (uint8_t)((a & 0xF0) | 0x07);
    u4c_cs_a31c(a);
    u4c_cs_ptr++;
    u4c_cs_a2df(0x02);
    u4c_sb_desc_commit();
  }
  a = XDATA_REG8V(0x09FB);
  if (!(a & 0x02)) {
    a = u4c_cs_read34();
    a = (uint8_t)((a & 0xF0) | 0x07);
    a = u4c_cs_a348(a);
    a = (uint8_t)((a & 0xC0) | 0x03);
    u4c_cs_a327(a);
    u4c_cs_ptr++;
    u4c_cs_a2df(0x02);
    u4c_sb_desc_commit();
  }
}

static void u4c_descriptor_load_stock(void) {  /* b779 */
  uint8_t a;
  u4c_identity_desc_emit_stock();
  u4c_lane_width_desc_stock();

  if (!(XDATA_REG8V(0x09F9) & 0x80)) {
    a = u4c_cs_read35();
    u4c_cs_wr((uint8_t)((a & 0x3F) | 0x80));
    u4c_cs_a2de(0x06);
    u4c_cs_ce20(0x0B34, 0x02);
  }

  u4c_cs_ptr = 0x35;
  a = u4c_cs_rd();
  u4c_cs_wr((uint8_t)((a & 0x3F) | 0x80));
  u4c_cs_ptr++;
  u4c_cs_a2df(0x20);
  u4c_cs_ce20(0x0B37, 0x40);

  a = u4c_cs_read34();
  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x41);
  u4c_cs_ptr = 0x3C;
  u4c_cs_wr(0x01);
  a = u4c_cs_a2f8(0x40);

  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x44);
  u4c_cs_ptr = 0x3C;
  u4c_cs_wr(0x8A);
  u4c_cs_ptr = 0x3E;
  a = u4c_cs_a2f9(0x03);

  a = u4c_cs_a308(a);
  (void)a;
  u4c_cs_a2df(0x4C);
  u4c_cs_ptr = 0x3C;
  u4c_cs_wr(0x00);
  u4c_cs_ptr++;
  u4c_cs_wr(0x03);
  u4c_cs_ptr++;
  u4c_cs_wr(0x00);
  a = u4c_cs_a2f8(0x00);
  a = (uint8_t)((a & 0xF0) | 0x04);
  u4c_cs_a30c(a);
  a = u4c_cs_a348(0x97);
  a = (uint8_t)((a & 0xE0) | 0x01);
  u4c_cs_wr(a);
  u4c_cs_ptr = 0x3E;
  u4c_cs_e7f8(0x04);
  u4c_cs_ptr = 0x06;
  a = u4c_cs_rd();
  a = (uint8_t)(a & 0xDF);
  u4c_cs_a3d4(a);
  u4c_cs_wr(0x01);
}

static void u4c_route_mode_regs(void) {  /* d556 */
  if (u4_cfg.mode_flag & MODE_FLAG_VDM_ACK) {
    PR(0x0247) = u4_p12.data_lo; PR(0x0248) = u4_p12.data_hi;
  }
}

static void u4c_pcie_tunnel_ramp_tail(void) {  /* bcd7 */
  if (u4_cfg.route_mode & 0x81) {
    REG_CPU_MODE_NEXT &= 0xEF;
    { uint8_t width = 0x0F;
      REG_PCIE_LINK_STATE = width; REG_PCIE_LINK_STATE_HI_B435 = width; REG_PCIE_LANE_CONFIG = width; REG_PCIE_LANE_CONFIG_HI_B437 = width;
    }
    if (u4_connect_gate & 0x10) {
      u4c_uart_drain_wait();
      REG_CPU_MODE &= 0xFE;
      REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    }
    REG_TUNNEL_LINK_STATE = (REG_TUNNEL_LINK_STATE & 0xFE) | 0x01;
  }
}

static void sb_assert(void) {
  u4c_set_sb1c_usb_mode();
  P1_SET(P1_PORT_CTRL_0000, 0x02);
  boot_phy_set_link_mode((u4_cfg.route_mode & 0x02) ? 2 : 1);

  u4c_pcie_tunnel_ramp_tail();

  { uint8_t route_flags = u4_cfg.route_mode;
    if (route_flags & 0x02) {
      REG_USB_CTRL_924C = (REG_USB_CTRL_924C & 0xF7) | 0x08;
    }
  }

  u4_cfg.lane_gate_sel = 0x02;
  u4c_lane_width_desc(0x81);
  sb_eng_data_init();
  u4c_identity_desc_emit(0x81);
  u4c_route_mode_regs();
  u4_pd.cm_dispatch_sel = 0;
  sb_lane_flip_init();
  sb_block_init();
  u4_boot.sb_asserted = 1;
  uart_puts("[SBdone]\n");
}

static void phy_cc11_ack(void);
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
#define phy_cc10_cmd(subcmd, cc12, cc13) do { \
  phy_cc11_ack(); \
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | ((subcmd) & 0x07); \
  REG_TIMER0_THRESHOLD_HI = (cc12); \
  REG_TIMER0_THRESHOLD_LO = (cc13); \
  REG_TIMER0_CSR = 0x01; \
} while (0)
static void u4c_link_mode_apply(uint8_t mode);  /* 8a89 */
static void u4lb_transport_reinit(uint8_t skip_lane);  /* b031 */
static void u4lb_pcie_tunnel_setup(uint8_t param);  /* e305 */
static void u4lb_pcie_link_enable(void);  /* e4ea */
static void boot_phy_set_bit0(uint16_t addr);  /* bceb */
static void sb_channel_connect_service(void);
static void u4lb_set_fsm_state(u4_fsm_state_t state);  /* eb62 */
static void u4lb_conn_rout_restart(void);  /* 98ec */
static void u4lb_sb_op_run_drain(uint8_t param);  /* d5da */

static void sb_write_c9_ack(uint8_t pos) {
  SB_WR(SB_PORT_SVC, (uint8_t)(1u << (pos & 7)));
}

#define SBP2_BASE()          ((uint16_t)(0x2A00u + ((uint16_t)(u4_sb.active_plane_port & 3) << 8)))
#define SBP2_RD(off)         P1_REG8_rd((uint16_t)(SBP2_BASE() + (off)))

#define SBTX_RD(off)         P1_REG8_rd((uint16_t)(0x2900u + (off)))
#define SBTX_WR(off, v)      P1_REG8_wr((uint16_t)(0x2900u + (off)), (uint8_t)(v))

static void sb_copy_host_route_desc(void) {  /* eaac */
  uint8_t i;
  u4_sb.route_query_response = 1;
  for (i = 0; i < 0x40; i++) {
    u4_host_desc[i] = SBP2_RD(i);
  }
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
}

static __code const uint8_t sb_desc_field_offset[0x13] = {  /* 21a1 */
  0x00,0x04,0xFF,0x08,0x0C,0xFF,0xFF,0xFF, 0x10,0x14,0x18,0xFF, 0x19,0x1C,0xFF,0x20, 0xFF,0xFF,0x24
};

static void sb_descriptor_response(void) {  /* af38 */
  uint8_t  desc_type, desc_len, desc_dir, raw_byte1, i, status5, width;
  uint8_t  desc = u4_sb.connect_descriptor;
  uint8_t  status_off = (u4_sb.active_plane_port == 0) ? 0x0D : 0x0E;

  u4_sb.tx_command_desc = (uint8_t)(desc & 0xDE);

  desc_type = SBP2_RD(SBP2_DESC_TYPE);
  raw_byte1 = SBP2_RD(SBP2_DESC_LEN);
  desc_len  = (uint8_t)(raw_byte1 & 0x7F);
  desc_dir  = (uint8_t)(raw_byte1 & 0x80);

  SBTX_WR(SBTX_DESC_TYPE, desc_type);
  SBTX_WR(SBTX_DESC_DIR, desc_dir);

  if (desc_type < 0x12) {
    width = sb_width_lut[(uint16_t)(desc_type)];
    SBTX_WR(SBTX_DESC_DIR, (uint8_t)(SBTX_RD(SBTX_DESC_DIR) | width));
  }

  status5 = (uint8_t)((SB_RD(status_off) & 0x7F) - 5);
  if (desc_dir != 0) {
    u4_sb.sb_desc_resp_len = 1;
    SBTX_WR(SBTX_DESC_BODY, 0);
    if (desc_type < 0x12 &&
        sb_width_lut[(uint16_t)(desc_type)] != 0 &&
        desc_len == status5 &&
        sb_branchA_gate[(uint16_t)(desc_type)] != 0 &&
        desc_len <= sb_width_lut[(uint16_t)(desc_type)]) {
      for (i = 0; i < desc_len; i++)
        u4_work_buf[(uint16_t)(uint8_t)(sb_desc_field_offset[desc_type] + i)] = SBP2_RD((uint8_t)(2 + i));
    } else {
      SBTX_WR(SBTX_DESC_DIR, (uint8_t)(SBTX_RD(SBTX_DESC_DIR) & 0x80));
      SBTX_WR(SBTX_DESC_BODY, 1);
    }
  } else {
    u4_sb.sb_desc_resp_len = desc_len;
    if (desc_type < 0x12 &&
        (width = sb_width_lut[(uint16_t)(desc_type)]) != 0 &&
        status5 == 0) {
      if ((uint8_t)(width + 1) <= u4_sb.sb_desc_resp_len) u4_sb.sb_desc_resp_len = width;
      for (i = 0; i < u4_sb.sb_desc_resp_len; i++)
        SBTX_WR((uint8_t)(2 + i), u4_work_buf[(uint16_t)(uint8_t)(sb_desc_field_offset[desc_type] + i)]);
    } else {
      SBTX_WR(SBTX_DESC_DIR, (uint8_t)(SBTX_RD(SBTX_DESC_DIR) & 0x80));
      u4_sb.sb_desc_resp_len = 0;
    }
  }

  {
    uint8_t status;
    (void)SB_RD(SB_DESC_COUNT_GO);
    status = (uint8_t)((u4_sb.sb_desc_resp_len + 8) | (SB_RD(SB_DESC_COUNT_GO) & 0x80));
    SB_WR(SB_DESC_COUNT_GO, status);
    SB_WR(SB_DESC_CMD, u4_sb.tx_command_desc);
  }

  u4lb_sb_op_run_drain(0);
}

static void sb_set_connect_present(void) {  /* ebb5 */
  if (((u4_sb.connect_descriptor >> 1) & 0x0F) != 0) {
    SB_SET(SB_CONNECT_PRESENT_57, 0x08);
    SB_SET(SB_CONNECT_PRESENT_61, 0x08);
  }
  u4_sb.connect_present = 1;
}

static void sb_rx_route_ack(void) {  /* edd9 */
  if (P1_RD(P1_ROUTE_ACK) & 0x01) {
    P1_WR(P1_ROUTE_ACK, P1_RD(P1_ROUTE_ACK) & 0xFE);
    SB_WR(SB_ROUTE_ACK, 0x02);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    u4lb_set_fsm_state(U4FSM_CONN_ROUT);
    u4lb_conn_rout_restart();
  }
}
static void sb_service_transport_edges(void);  /* d4cd */
static void sb_connect_desc_dispatch(uint8_t desc4e_off, uint8_t desc752_off) {  /* cd3f */
  uint8_t desc4e;
  uint8_t desc752;
  uint8_t port = u4_sb.active_plane_port;
  sb_rx_route_ack();
  desc4e = SB_RD(desc4e_off);
  u4_sb.connect_descriptor = SB_RD(desc752_off);
  desc752 = u4_sb.connect_descriptor;
  if (u4_sb.connect_present != 0 && (desc752 & 0x60) == 0x60) return;
  if (!(desc4e & 0x10)) return;
  if ((desc752 & 0xC0) != 0x40) {
    if (!(desc752 & 0x04)) return;
    if (port == 0 || port == 1) {
      if (desc752 & 0x10) return;
    } else if (port == 2 || port == 3) {
      if (!(desc752 & 0x10)) return;
    }
  }
  if ((desc752 & 0x60) == 0x60) {
    sb_set_connect_present();
  } else if ((desc752 & 0x01) == 0) {
    sb_copy_host_route_desc();
  } else {
    if (((desc752 & 0x40) == 0) || (((desc752 >> 1) & 0x0F) == 0)) {
      sb_descriptor_response();
    }
  }
}
static void sb_service_transport_edges(void) {
  static __code const uint16_t edge_regs[4] = {SB_CONN_EDGE(0), SB_CONN_EDGE(1), SB_LINK_EDGE(0), SB_LINK_EDGE(1)};
  uint8_t i;
  for (i = 0; i < 4; i++) {
    if (SB_RD(edge_regs[i]) & 0x08) {
      uint8_t toggle = (i < 2) ? u4_sb.transport_edge_toggle : u4_sb.link_edge_toggle;
      if (toggle == (i & 1)) {
        static __code const uint8_t dispatch_a[4] = {0x28, 0x2A, 0x81, 0x83};
        static __code const uint8_t dispatch_b[4] = {0x18, 0x19, 0x08, 0x09};
        u4_sb.active_plane_port = i;
        sb_connect_desc_dispatch(dispatch_a[i], dispatch_b[i]);
        SB_EDGE_ACK(edge_regs[i]);
        if (i < 2) u4_sb.transport_edge_toggle = (uint8_t)(1 - (i & 1));
        else       u4_sb.link_edge_toggle = (uint8_t)(1 - (i & 1));
      }
    }
  }
}
static void sb_route_arm(void) {  /* db7a */
  if (u4_pd.connect_route_latch == 0) {
    REG_CPU_CTRL_CA60 &= 0xF7;
    REG_CPU_CTRL_CA60 = REG_CPU_CTRL_CA60;
    REG_PHY_LINK_TRIGGER &= 0xEF;
    REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0x8F) | 0x50;
  } else {
    REG_CPU_CTRL_CA60 &= 0xF7;
    REG_CPU_CTRL_CA60 |= 0x04;
    REG_PHY_CTRL_C20F = 0xFF;
    REG_CPU_CTRL_CA70 = (REG_CPU_CTRL_CA70 & 0xFC) | 0x02 | 0x04;
    REG_PHY_LINK_TRIGGER |= 0x10;
    REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0x8F) | 0x60;
    REG_PHY_CTRL_C20F = 0x00;
  }
  u4lb_set_fsm_state(U4FSM_CONN_ROUT);
  u4lb_conn_rout_restart();
}

static void sb_con_consequence(void) {
  if (u4_sb.conn_consequence_done) return;

  if (P1_RD(P1_ROUTE_ACK) & 0x01) {
    P1_WR(P1_ROUTE_ACK, P1_RD(P1_ROUTE_ACK) & 0xFE);
    SB_WR(SB_ROUTE_ACK, 0x02);
  }
  SB_WR(SB_ADP0_CTRL, (SB_RD(SB_ADP0_CTRL) & 0xBF) | 0x40);
  SB_WR(SB_ADP0_CTRL, (SB_RD(SB_ADP0_CTRL) & 0x7F) | 0x80);
  SB_WR(SB_ADP0_EN, (SB_RD(SB_ADP0_EN) & 0xFE) | 0x01);
  SB_WR(SB_ADP1_CTRL, (SB_RD(SB_ADP1_CTRL) & 0xBF) | 0x40);
  SB_WR(SB_ADP1_CTRL, (SB_RD(SB_ADP1_CTRL) & 0x7F) | 0x80);
  P1_WR(P1_LANE_FLIP(0), (P1_RD(P1_LANE_FLIP(0)) & 0xEF) | 0x10);
  P1_WR(P1_LANE_FLIP(0), P1_RD(P1_LANE_FLIP(0)) & 0xFE);
  phy_cc10_cmd(2, 0, 0x15);
  { uint16_t g = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++g < 0x0400); }
  REG_TIMER0_CSR = 0x02;
  P1_WR(P1_LANE_FLIP(0), (P1_RD(P1_LANE_FLIP(0)) & 0xBF) | 0x40);
  P1_WR(P1_LANE_FLIP(0), (P1_RD(P1_LANE_FLIP(0)) & 0x7F) | 0x80);
  u4_sb.conn_consequence_done = 1;
  sb_route_arm();
}

static void u4lb_tunnel_phy_finalize(void);  /* c593 */
static void sb_lane_bonded_consequence(void) {
  u4_sb.lane_bonded_flag = 1;
  SB_WR(SB_PORT_SVC, 0xFF);
  u4_sb.active_port_rr = (uint8_t)((SB_RD(SB_TRANSPORT_STAT) & 0x06) >> 1);
  u4lb_tunnel_phy_finalize();
}

static void sb_phy_connect_config(void) {  /* c35b */
  uint8_t s = XDATA_REG8(0x0B30);
  if (s == 0x02) {
    REG_GPIO_CTRL_0 = (uint8_t)((REG_GPIO_CTRL_0 & 0xE0) | 0x05);
    REG_GPIO_CTRL_0 = (uint8_t)((REG_GPIO_CTRL_0 & 0xF7) | 0x08);
  } else {
    if (s == 0x01) REG_PHY_CFG_C655 = (uint8_t)(REG_PHY_CFG_C655 & 0xFE);
    else           REG_PHY_CFG_C655 = (uint8_t)((REG_PHY_CFG_C655 & 0xFE) | 0x01);
    REG_GPIO_CTRL_0 = (uint8_t)(REG_GPIO_CTRL_0 & 0xE0);
    REG_PHY_CFG_C65A = (uint8_t)((REG_PHY_CFG_C65A & 0xFE) | 0x01);
  }
  if (XDATA_REG8(0x0AE4) == 0) {
    uint8_t s1 = XDATA_REG8(0x0B31);
    if (s1 == 0x02) {
      XDATA_REG8(0xC623) = (uint8_t)((XDATA_REG8(0xC623) & 0xE0) | 0x05);
      REG_PHY_CFG_C65A = (uint8_t)((REG_PHY_CFG_C65A & 0xF7) | 0x08);
    } else {
      if (s1 == 0x01) REG_PHY_CFG_C655 = (uint8_t)(REG_PHY_CFG_C655 & 0xF7);
      else            REG_PHY_CFG_C655 = (uint8_t)((REG_PHY_CFG_C655 & 0xF7) | 0x08);
      XDATA_REG8(0xC623) = (uint8_t)(XDATA_REG8(0xC623) & 0xE0);
      REG_PHY_CFG_C65A = (uint8_t)((REG_PHY_CFG_C65A & 0xF7) | 0x08);
    }
  }
}

static void u4lb_phy_connect_dma_kick(void) {  /* e96c */
  XDATA_REG8(0x0B30) = 0; XDATA_REG8(0x0B31) = 0;
  XDATA_REG8(0x0B32) = 0; XDATA_REG8(0x0B33) = 0;
  REG_CPU_TIMER_CTRL_CD31 = 0x04;
  REG_CPU_TIMER_CTRL_CD31 = 0x02;
  REG_PHY_DMA_CMD_CD30 = (uint8_t)((REG_PHY_DMA_CMD_CD30 & 0xF8) | 0x05);
  REG_PHY_DMA_ADDR_LO = 0x00;
  REG_PHY_DMA_ADDR_HI = 0xC7;
  REG_TIMER_CFG_CC2A = (uint8_t)((REG_TIMER_CFG_CC2A & 0xF8) | 0x04);
  REG_TIMER_CFG_CC2C = 0xC7;
  REG_TIMER_CFG_CC2D = 0xC7;
  sb_phy_connect_config();
}

static void sb_connect_reservice(void) {  /* d7cd */
  uint8_t r7;
  if ((REG_CPU_CTRL_CA81 & 0x01) == 0x01) return;
  r7 = REG_PCIE_LINK_CTRL_B481;
  REG_PCIE_LINK_CTRL_B481 = 0xFF;
  if (u4_p12.reinit_pending == 0) {
    uint8_t lc = (uint8_t)(r7 & 0x03);
    if (lc != 0) {
      REG_CPU_TIMER_CTRL_CD31 = 0x04;
      REG_CPU_TIMER_CTRL_CD31 = 0x02;
      u4_p12.reinit_pending = (uint8_t)(lc - 1);
    }
    { uint8_t cd31 = REG_CPU_TIMER_CTRL_CD31;
      if (((cd31 & 0x01) == 0) || ((cd31 >> 1) & 0x01))
        XDATA_REG8(0x0B30) = 1;
      else
        XDATA_REG8(0x0B30) = 2; }
  } else {
    XDATA_REG8(0x0B30) = 0;
  }
  sb_phy_connect_config();
}

static void sb_lane_bond_complete_tunnel_up(void) {
  if (u4_boot.tunnel_up_done) return;
  u4_boot.tunnel_up_done = 1;

  sb_rom_descriptor_load();
  REG_CPU_CTRL_CA60 &= 0xF7;
  if (XDATA_REG8V(0x0AF1) & 0x01) {
    u4c_link_mode_apply(2);
  }
}

static void sb_pcie_tunnel_setup_if_cl0(void) {
  if (u4_boot.tunnel_up_done) return;
  if (((SB_RD(SB_LANE_CL(0)) & 0x0F) != 2) || ((SB_RD(SB_LANE_CL(1)) & 0x0F) != 2)) return;

  u4_boot.tunnel_up_done = 1;
  u4lb_pcie_tunnel_setup(1);
  u4lb_pcie_link_enable();
  REG_PCIE_LANE_CTRL_C659 |= PCIE_LANE_CTRL_ENABLE;
  REG_PHY_TIMER_CTRL_E764 = PHY_TIMER_PCIE_TRAIN_USB4;
}

static void sb_channel_connect_service(void) {
  uint8_t port = u4_sb.active_port_rr;
  uint8_t lo, hi, n;

  sb_rx_route_ack();

  if (port == 0)      { lo = SB_RD(SB_CH_ROUTE_LO(0)); hi = SB_RD(SB_CH_ROUTE_HI(0)); }
  else if (port == 1) { lo = SB_RD(SB_CH_ROUTE_LO(1)); hi = SB_RD(SB_CH_ROUTE_HI(1)); }
  else if (port == 2) { lo = SB_RD(SB_CH2_ROUTE_LO(0)); hi = SB_RD(SB_CH2_ROUTE_HI(0)); }
  else                { lo = SB_RD(SB_CH2_ROUTE_LO(1)); hi = SB_RD(SB_CH2_ROUTE_HI(1)); }

  if ((uint8_t)(~hi) != lo) {
    uart_puts("[SBch err]");
    return;
  }

  n = lo & 0x0F;
  if (n == 1 || n == 5) {
    u4_sb.route_up_trigger = 1;
    return;
  }
  if (n == 3) {
    if (u4_p12.link_reinit_gate != 0) {
      SB_WR(SB_LINK_REINIT_50, 0x40);
      SB_WR(SB_LINK_REINIT_5A, 0x40);
      P1_WR(P1_ROUTE_ACK, (uint8_t)(P1_RD(P1_ROUTE_ACK) | 0x01));
    }
    SB_WR(SB_DESC_CMD, 0x83);
    SB_WR(SB_DESC_COUNT_GO, (uint8_t)((SB_RD(SB_DESC_COUNT_GO) & 0x80) | 0x03));
    u4lb_sb_op_run_drain(1);
    return;
  }
  if (n == 0) {
    if (u4_sb.state == U4FSM_CONN_ROUT) return;  /* both cm_dispatch_sel arms returned */
    if ((((lo & 0x20) >> 5) & 7) == 0) return;
    SB_WR(SB_LINK_REINIT_5A, 0x40);
    u4_work_buf[WB_LANE_EN] &= 0xFD;
    if (u4_work_buf[WB_LANE_EN] & 0x01) {
      (void)SB_RD(SB_LANE_CL(0));
    }
    return;
  }
}

static void sb_routerop_tx(uint8_t is_e1cb) {  /* a5d8 */
  sb_service_transport_edges();
  SBTX_WR(SBTX_DESC_TYPE, sb_tx_byte0);
  SBTX_WR(SBTX_DESC_DIR, (uint8_t)(sb_tx_byte1 | ((sb_tx_flag & 1) << 7)));
  if (sb_tx_flag == 0)
    SB_WR(SB_DESC_COUNT_GO, (uint8_t)((SB_RD(SB_DESC_COUNT_GO) & 0x80) | 0x08));
  else
    SB_WR(SB_DESC_COUNT_GO, (uint8_t)(((sb_tx_byte0 + 8) & 0xFF) | (SB_RD(SB_DESC_COUNT_GO) & 0x80)));
  SB_WR(SB_DESC_CMD, is_e1cb ? (uint8_t)((sb_tx_cmd << 1) | 0x41) : (uint8_t)sb_tx_cmd);
  u4lb_sb_op_run_drain(0);
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
  u4_sb.routerop_push_token = 0x01;
}

static void sb_routerop_pending(void) {  /* a5d8 */
  static __xdata uint8_t opcode_byte, opcode, i, op_idx, len;

  u4_cfg.routerop_desc0 = u4_rop_hdr.hdr0;
  u4_cfg.routerop_desc1 = u4_rop_hdr.hdr1;
  u4_cfg.routerop_desc2 = u4_rop_hdr.hdr2;
  u4_cfg.routerop_desc3 = u4_rop_hdr.hdr3;

  if (!(u4_cfg.routerop_desc3 & 0x80)) return;

  u4_routerop_op_lo = u4_cfg.routerop_desc0;
  u4_routerop_op_len = u4_cfg.routerop_desc1;
  opcode_byte = u4_cfg.routerop_desc2;
  u4_routerop_opcode = (uint8_t)(opcode_byte & 7);
  u4_routerop_svid_hi = (uint8_t)(opcode_byte >> 4);
  u4_routerop_flag = (uint8_t)(u4_cfg.routerop_desc3 & 1);
  u4_routerop_port = sb_desc_field_offset[u4_routerop_op_lo];

  opcode = u4_routerop_opcode;
  if (opcode != 0) {
    if (opcode != 1 && opcode != 2) return;
    u4_sb.routerop_resp_armed = 1;
    if (u4_routerop_flag != 0) {
      for (i = 0; i < u4_routerop_op_len && i != 0x40; i += 4) {
        SBTX_WR(i + 2, sb_routerop_body[i]);
        SBTX_WR(i + 3, sb_routerop_body[0x1 + i]);
        SBTX_WR(i + 4, sb_routerop_body[0x2 + i]);
        SBTX_WR(i + 5, sb_routerop_body[0x3 + i]);
      }
    }
    if (u4_routerop_opcode == 1) {
      sb_tx_flag = u4_routerop_flag;
      sb_tx_cmd = (uint8_t)(u4_sb.route_enable_latch | 0x01);
      sb_tx_byte0 = u4_routerop_op_lo;
      sb_tx_byte1 = u4_routerop_op_len;
      sb_routerop_tx(0);
    } else {
      sb_tx_flag = u4_routerop_flag;
      sb_tx_cmd = u4_routerop_svid_hi;
      sb_tx_byte0 = u4_routerop_op_lo;
      sb_tx_byte1 = u4_routerop_op_len;
      sb_routerop_tx(1);
    }
    return;
  }

  if (u4_routerop_flag != 0) {
    op_idx = u4_routerop_op_lo;
    if (op_idx < 0x12 &&
        (len = sb_width_lut[(uint16_t)op_idx]) != 0 &&
        sb_branchA_gate[(uint16_t)op_idx] != 0 &&
        u4_routerop_op_len <= len) {
      for (i = 0; i < u4_routerop_op_len; i++) {
        u4_work_buf[(uint8_t)(u4_routerop_port + i)] = sb_routerop_body[i];
      }
      u4_cfg.routerop_desc1 = len;
    } else {
      uart_puts("\r\n[RdCmdErr]");
      u4_cfg.routerop_desc1 = 0;
      u4_cfg.routerop_desc3 = (uint8_t)(u4_cfg.routerop_desc3 | 0x04);
    }
  } else {
    op_idx = u4_routerop_op_lo;
    if (op_idx < 0x12 &&
        (len = sb_width_lut[(uint16_t)op_idx]) != 0) {
      if (u4_routerop_op_len > len) u4_routerop_op_len = len;
      for (i = 0; i < u4_routerop_op_len; i++) {
        sb_routerop_body[i] = u4_work_buf[(uint8_t)(u4_routerop_port + i)];
      }
      u4_cfg.routerop_desc1 = len;
    } else {
      uart_puts("\r\n[WrCmdErr]");
      u4_cfg.routerop_desc1 = 0;
      u4_cfg.routerop_desc3 = (uint8_t)(u4_cfg.routerop_desc3 | 0x04);
    }
  }

  u4_cfg.routerop_desc3 = (uint8_t)(u4_cfg.routerop_desc3 & 0x7F);
  u4_rop_hdr.hdr0 = u4_cfg.routerop_desc0;
  u4_rop_hdr.hdr1 = u4_cfg.routerop_desc1;
  u4_rop_hdr.hdr2 = u4_cfg.routerop_desc2;
  u4_rop_hdr.hdr3 = u4_cfg.routerop_desc3;
  SB_WR(SB_ROUTEROP_COMMIT, 0x01);
}

static void sb_routerop_response(uint8_t r) {  /* cdf5 */
  uint8_t hdr1_new, hdr3_orig, w3, len;
  if (r == 1) return;
  u4_sb.routerop_resp_armed = 0;
  hdr1_new  = (uint8_t)(u4_host_desc[HD_STATUS] & 0x7F);
  hdr3_orig = u4_rop_hdr.hdr3;
  if (r == 2) {
    w3 = (uint8_t)(hdr3_orig | 0x02);
  } else {
    w3 = (uint8_t)(hdr3_orig & 0xFB);
    if ((hdr3_orig & 0x01) == 0) {
      if (hdr1_new == 0) w3 = (uint8_t)(w3 | 0x04);
      len = hdr1_new; if (len > 0x40) len = 0x40;
      mem_copy(sb_routerop_body, &u4_host_desc[0x2], len);
    } else {
      if (u4_host_desc[0x2] != 0) w3 = (uint8_t)(w3 | 0x04);
    }
  }
  w3 = (uint8_t)(w3 & 0x7F);
  u4_rop_hdr.hdr1 = hdr1_new;
  u4_rop_hdr.hdr3 = w3;
  SB_WR(SB_ROUTEROP_COMMIT, 0x01);
}

static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

  for (idx = 0; idx < 4; idx++) {
    bm = SB_RD(SB_PORT_SVC);
    if ((bm & (uint8_t)(1u << (4 + idx))) && u4_sb.active_port_rr == idx) {
      sb_channel_connect_service();
      sb_write_c9_ack(idx);
      sb_write_c9_ack((uint8_t)(idx + 4));
      u4_sb.active_port_rr = (uint8_t)((u4_sb.active_port_rr + 1) & 3);
      if (P1_RD(P1_ROUTE_ACK) & 0x01)
        sb_lane_bond_complete_tunnel_up();
    }
  }

  sb_service_transport_edges();

  cs = SB_RD(SB_CONNECT_STATE);
  if (!(cs & 0x01)) {
    if (SB_RD(SB_CONNECT_EVENT) & 0x01) {
      SB_WR(SB_CONNECT_EVENT, 0x01);
      SB_WR(SB_CONNECT_EVENT, 0x02);
      { uint8_t w = (uint8_t)((SB_RD(SB_CONNECT_STATE) & 0xFE) | 0x01);
        SB_WR(SB_CONNECT_STATE, w);
        SB_WR(SB_CONNECT_STATE, (uint8_t)(SB_RD(SB_CONNECT_STATE) & 0xFD)); }
      sb_con_consequence();
    }
  } else {
    cs = SB_RD(SB_CONNECT_STATE);
    if (!(cs & 0x02)) {
      if (SB_RD(SB_CONNECT_EVENT) & 0x02) {
        uart_puts("\r\n[===SB Dis===]\r\n");
        SB_WR(SB_CONNECT_EVENT, 0x02); SB_WR(SB_CONNECT_EVENT, 0x01);
        SB_WR(SB_CONNECT_STATE, (uint8_t)(SB_RD(SB_CONNECT_STATE) & 0xFE));
        SB_WR(SB_CONNECT_STATE, (uint8_t)((SB_RD(SB_CONNECT_STATE) & 0xFD) | 0x02));
      }
    }
  }

  if (SB_RD(SB_BOND_EVENT) & BOND_EVT_BONDED) {
    SB_WR(SB_BOND_EVENT, BOND_EVT_BONDED);
    uart_puts("\r\nLane Bonded\r\n");
    sb_lane_bonded_consequence();
  }

  if (SB_RD(SB_ROUTEROP_EVENT) & ROUTEROP_EVT_PENDING) {
    sb_routerop_pending();
    SB_WR(SB_ROUTEROP_EVENT, 0x02);
  }

  { uint8_t lane;
    for (lane = 0; lane < 2; lane++) {
      uint8_t bit = (uint8_t)(1u << lane);
      if (!(SB_RD(SB_CL0_EVENT) & bit)) continue;
      SB_WR(SB_CL0_EVENT, bit);
      uart_puts(lane == 0 ? "\r\nL0:CL0 " : "\r\nL1:CL0 ");
      uart_puthex(SB_RD(SB_LANE_CL(lane)) & 0x0F);
      SB_WR(SB_CL0_ACK, (uint8_t)((SB_RD(SB_CL0_ACK) & ~bit) | bit));
      if (!(u4_work_buf[WB_LANE_EN] & (uint8_t)(1u << (lane ^ 1))) || ((SB_RD(SB_LANE_CL(lane ^ 1)) & 0x0F) == 2))
        SB_WR(SB_PEER_CL0, (uint8_t)((SB_RD(SB_PEER_CL0) & 0xDF) | 0x20));
    }
  }

  sb_pcie_tunnel_setup_if_cl0();

  SB_W1C_PRINT(SB_BOND_EVENT, BOND_EVT_L0_ABR2, "\r\nL0:Abr2");
  SB_W1C_PRINT(SB_BOND_EVENT, BOND_EVT_L1_ABR2, "\r\nL1:Abr2");
  SB_W1C_PRINT(SB_BOND_EVENT, BOND_EVT_L0_FAIL, "\r\nL0:Bnd Fail");
  SB_W1C_PRINT(SB_BOND_EVENT, BOND_EVT_L1_FAIL, "\r\nL1:Bnd Fail");

  if (SB_RD(SB_ROUTEROP_EVENT) & ROUTEROP_EVT_L0_DIS) {
    SB_WR(SB_ROUTEROP_EVENT, ROUTEROP_EVT_L0_DIS);
    uart_puts("\r\nL0:Disable");
    SB_WR(SB_DESC_CMD, 0x80);
  }
  if (SB_RD(SB_ROUTEROP_EVENT) & ROUTEROP_EVT_L1_DIS) {
    SB_WR(SB_ROUTEROP_EVENT, ROUTEROP_EVT_L1_DIS);
    uart_puts("\r\nL1:Disable");
    SB_WR(SB_DESC_CMD, 0xA0);
    SB_WR(SB_LINK_REINIT_5A, 0x40);
  }

  SB_W1C(SB_CL0_EVENT, CL0_EVT_L0_TRAIN);
  SB_W1C(SB_CL0_EVENT, CL0_EVT_L1_TRAIN);

  (void)SB_RD(SB_EVENT_CLEAR_F6);
}


#endif
