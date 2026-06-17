#ifndef USB4_H
#define USB4_H
/*
 * USB4 mode-establishment glue: post-Enter_USB connect path + the INT1 USB4 event demux.
 * Include AFTER vdm.h (needs PR(a), uart_*, PD helpers, DPX SFR).
 */

/* PHY descriptor seed (mode 4 seeds the PHY trim registers). */
static void u4c_e0d9(uint8_t mode) {
  if (mode == 4) {
    REG_PHY_RXPLL_RESET = 0x3E; REG_PHY_CTRL_C20F = 0x08; REG_PHY_CDR_SEED_C210 = 0x08; REG_PHY_CDR_SEED_C211 = 0x2E; REG_PHY_CDR_SEED_C212 = 0x3E;
    REG_PHY_CDR_SEED_C214 = 0x00; REG_PHY_CDR_SEED_C215 = 0x20; REG_PHY_CDR_SEED_C216 = 0x00; REG_PHY_CDR_SEED_C217 = 0x3F;
  }
}

/* Timer-enable gate: mode 1 clears bit1; else conditionally sets bit1 per 0x0AF1.4. */
static void u4c_e7c1(uint8_t mode) {
  if (mode == 1) { REG_TIMER_ENABLE_B &= 0xFD; REG_TIMER_ENABLE_A &= 0xFD; }
  else if (u4_connect_gate & 0x10) { REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xFD) | 0x02; REG_TIMER_ENABLE_A = (REG_TIMER_ENABLE_A & 0xFD) | 0x02; }
}

/* Post-Enter_USB connect path: drives sideband bring-up so the host CM trains the lanes. */
static void usb4_connect_u4(void) {
  u4_connect_gate |= 0x01;
  if (u4_connect_gate & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    REG_CPU_CTRL_CA81 &= 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  }
  boot_phy_dd42(0);
  u4c_e7c1(1);
  u4c_e0d9(0);
  if (u4_enter_usb_accepted == 0) {
    if (u4_connect_route_latch == 0) return;
    u4_route_mode = 0x81;
    u4_lane_gate_sel = 0x02;
  } else {
    u4_route_mode = (u4_route_mode & 0x04) | (u4_mode_flag & 0x03);
    if (u4_dp_alt_mode == 0x03) {
      if (pd_usb3_fallback_flag == 0) { u4_route_mode = (u4_route_mode & 0x04) | 2; u4_lane_gate_sel = 1; }
      else                 { u4_route_mode = (u4_route_mode & 0x04) | 1; u4_lane_gate_sel = 2; }
    }
    if (u4_route_mode & 0x02) {
      REG_LINK_STATUS_E716 &= 0xFC;
      REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
      SB_WR(0xD8, 0x02);
    }
  }
  sb_assert();
}

/* INT1 USB4 event demux: services the USB4 INT sources and applies their W1C acks. */

/* Sticky bitmap of USB4 INT sources seen (bit0=SB, bit1=evt, bit2=routerop, bit3=tunnel). */
static volatile uint8_t __xdata __at(0x0B49) usb4_int_seen;

static void sb_router_event_handler(void);

/* Sticky accumulator of every C80A value seen in the ISR. */
static volatile uint8_t __xdata __at(0x0B4A) c80a_acc;

/* Config-space router-op dispatcher: replies to host-posted EA90 router-ops and acks them. */
static void cm_routerop_mailbox(void) {
  if (REG_SYS_CTRL_EA90 != 0x5A) return;
  {
    rmbox_state_t state = u4_routerop_mbox_state;
    if (state == RMBOX_IDLE) {
      u4_routerop_mbox_opcode = REG_ROUTEROP_OPCODE_EA80;
    } else if (state == RMBOX_MULTIPKT_1) {
      if (u4_routerop_mbox_opcode == 0xE2) {
        u4_routerop_mbox_state = RMBOX_IDLE;
        REG_SYS_CTRL_EA90 = 0xA5;
        return;
      }
      u4_routerop_mbox_state = RMBOX_IDLE;
    } else if (state == RMBOX_MULTIPKT_2) {
      if (u4_routerop_mbox_opcode == 0xE3) {
        u4_routerop_mbox_state = RMBOX_IDLE;
        REG_SYS_CTRL_EA90 = 0xA5;
        return;
      }
      u4_routerop_mbox_state = RMBOX_IDLE;
    }
  }
}

/* Called from int1_isr after PD-RX: acks and forwards each fired USB4 INT source. */
static void usb4_int_demux(void) {
  uint8_t int_sources = REG_INT_PCIE_NVME;
  c80a_acc |= int_sources;
  if (int_sources & 0x20) {
    usb4_int_seen |= 0x01;
    sb_router_event_handler();
  }
  if (int_sources & 0x10) usb4_int_seen |= 0x02;
  if (REG_NVME_EVENT_STATUS & 0x01) {
    usb4_int_seen |= 0x04;
    REG_NVME_EVENT_ACK = 1;
    cm_routerop_mailbox();
  }
  if (int_sources & 0x0F) {
    usb4_int_seen |= 0x08;
    { uint8_t tunnel_status = REG_PHY_RXPLL_TRIGGER;
      if (tunnel_status & 0x04) REG_PHY_RXPLL_TRIGGER = 0x04;
      if (tunnel_status & 0x08) REG_PHY_RXPLL_TRIGGER = 0x08; }
  }
}

#endif /* USB4_H */
