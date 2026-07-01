#ifndef USB4_BOOT_H
#define USB4_BOOT_H

#if HANDMADE_USB4_MODE_FLAGS
static uint8_t boot_mode_flags_usb4_policy(void) {
  if (!(REG_FLASH_READY_STATUS & FLASH_READY_USB4_MODE)) return HANDMADE_USB3_MODE_FLAGS;
  return HANDMADE_USB4_MODE_FLAGS;
}

static void usb4_state_prepare(void) {
  // Type-C SBU + PHY config + SB-block enable; powers the sideband transport.
  boot_phy_bringup_early();

  // Seed the lane-engine link width/mode and the USB4 tunnel state flags.
  u4_phy_state_seed();

  u4_cfg.dp_alt_mode = 3;
  u4_cfg.cap20g_gate0 = 1; u4_cfg.cap20g_gate1 = 1;
  u4_cfg.sb_desc_profile = 3; u4_cfg.lane_gate_sel = 3;
  u4_cfg.product_pid_lo = 0x63;
  u4_cfg.product_pid_hi = 0x24;

  u4_cfg.tunnel_cfg_hi = 0x21;
  u4_cfg.tunnel_cfg_lo = 0x1B;
  u4_cfg.tunnel_cfg_mode = 0x63;
  u4_cfg.tunnel_credits = 0x24;

  pcie_tunnel_adapter_enable();

  // Stock seeds the sideband connect-service path at boot and then refreshes it in the main loop.
  u4lb_phy_connect_dma_kick();

  { uint8_t z; for (z = 0; z < U4_ROUTEROP_MBOX_CLEAR_LEN; z++) U4_XDATA_BYTES(u4_routerop_mbox_state)[z] = 0; }
  u4_sb.active_port_rr = 0;
  u4_sb.route_up_trigger = 0; u4_sb.lane_bonded_flag = 0;
  u4_sb.transport_edge_toggle = 0; u4_sb.link_edge_toggle = 0; u4_sb.active_plane_port = 0;
  u4_sb.conn_consequence_done = 0; u4_sb.state = U4FSM_IDLE;
  u4_sb.conn_routing_substate = CONNRT_PRINT_STATUS; lb_loop1_state[0] = LP1_PARKED; lb_loop1_state[1] = LP1_PARKED;
  lb_loop2_state[0] = LP2_CL_IDLE; lb_loop2_state[1] = LP2_CL_IDLE; u4_sb.route_enable_latch = 0;
  u4_sb.connect_present = 0; u4_sb.lane_width_cnt_hi = 0; u4_sb.lane_width_cnt_lo = 0;
  u4_sb.connect_descriptor = 0; u4_sb.tx_command_desc = 0;
  u4_pd.connect_oneshot_suppress = 0;

  u4_boot.pcie_ctrl_shadow = 0;
  u4_boot.pd_seen = 0;
  u4_boot.sb_asserted = 0;
  u4_boot.tunnel_up_done = 0;
}

static void usb4_policy_enable(void) {
  usb_pipe_engine_init();
  usb4_phy_arm();
  pd_keystone_init();
  usb4_phy_rx_arm();
  usb4_routerop_init();

  REG_INT_ENABLE = (uint8_t)((REG_INT_ENABLE & 0xBF) | 0x40);

  P1_WR(0x0000, (uint8_t)(P1_RD(0x0000) & 0xFD));
  REG_INT_CTRL = (uint8_t)((REG_INT_CTRL & 0xFD) | 0x02);
  u4lb_transport_reinit(0);
  P1_WR(P1_USB4_BOOT_TAIL_CTRL_1602, (uint8_t)(P1_RD(P1_USB4_BOOT_TAIL_CTRL_1602) & 0xFE));
  P1_WR(P1_USB4_BOOT_TAIL_EVENT_1603, 0x01);
  P1_WR(P1_USB4_BOOT_TAIL_CTRL_1602, (uint8_t)(P1_RD(P1_USB4_BOOT_TAIL_CTRL_1602) & 0xFD));
  P1_WR(P1_USB4_BOOT_TAIL_EVENT_1603, 0x02);
  P1_WR(P1_USB4_CFG_ENABLE_121E, (uint8_t)(P1_RD(P1_USB4_CFG_ENABLE_121E) | 0x01));
}

static void usb4_fallback_to_usb3(void) {
  uart_puts("[USB4 fallback]\n");
  usb4_skip_magic0 = 0xA5;
  usb4_skip_magic1 = 0x5A;
  REG_CPU_RESET = CPU_RESET_TRIGGER;
  while (1) { }
}

#define USB4_INT1_BODY() do { \
  uint8_t saved_dpx = DPX; \
  DPX = 0x00; \
  if ((u4_cfg.mode_flag & 0x83) && (REG_INT_SYSTEM & 0x01)) cc_pd_timer_tick(); \
  if (REG_CPU_EXEC_STATUS_2 & 0x04) { REG_CPU_EXEC_STATUS_2 = 0x04; } \
  if ((u4_cfg.mode_flag & 0x83) && (REG_INT_PCIE_NVME & 0x40)) pd_rx_isr(); \
  if (u4_cfg.mode_flag & 0x83) usb4_int_demux(); \
  if (REG_INT_SYSTEM & 0x10) { } \
  DPX = saved_dpx; \
} while (0)

#define USB4_SELECT_BOOT_MODE(reset_fallback) do { \
  if (usb4_skip_magic0 == 0xA5 && usb4_skip_magic1 == 0x5A) { \
    usb4_skip_magic0 = 0; \
    usb4_skip_magic1 = 0; \
    u4_cfg.mode_flag = HANDMADE_USB3_MODE_FLAGS; \
    (reset_fallback) = 1; \
  } else { \
    u4_cfg.mode_flag = boot_mode_flags_usb4_policy(); \
  } \
  uart_puts("[Mode "); \
  uart_puthex(u4_cfg.mode_flag); \
  uart_puts(" S"); \
  uart_puthex(REG_FLASH_READY_STATUS); \
  uart_puts(" C"); \
  uart_puthex(REG_FLASH_BUF_BYTE(0)); \
  uart_puts("]\n"); \
} while (0)

#define USB4_REINIT_USB3_AFTER_RESET_FALLBACK() do { \
  usb_pipe_engine_init(); \
  REG_CPU_MODE = CPU_MODE_USB3; \
  REG_CPU_MODE_NEXT &= 0x1F; \
  REG_CPU_CTRL_CA81 &= 0xFE; \
  boot_phy_set_link_mode(0); \
  boot_phy_lane_power(0x0F); \
  boot_phy_set_lane_width(0x0F); \
  u4lb_pcie_set_link_width(PCIE_LINK_WIDTH_x2); \
} while (0)

#else

#define USB4_INT1_BODY() do { uart_puts("[int1]\n"); } while (0)

#endif

#endif
