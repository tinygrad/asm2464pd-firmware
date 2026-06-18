#ifndef SB_ROUTER_H
#define SB_ROUTER_H
/*
 * USB4 SB-router: connect/disconnect, lane-bond, and the SB-transport descriptor engine.
 * Included after sb.h and usb4.h.
 */

static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void u4lb_eb62(uint8_t state_lo, u4_fsm_state_t state);
static void u4lb_98ec(uint8_t hi, uint8_t lo);
static void u4lb_d5da(uint8_t param);

/* Print budget for the [===SB Con===] edge so the connect re-assert can't saturate the UART. */
static volatile uint8_t __xdata __at(0x0B4C) sb_con_print_budget;

/* Run bank0_8a89 (the USB4 lane-MODE engine) once from the super-loop after connect. */
static volatile uint8_t __xdata __at(0x0B4D) sb_run_8a89_pending;
static volatile uint8_t __xdata __at(0x0B4E) sb_8a89_done;

/* W1C one connect bit in SB[0xC9]. */
static void sb_write_c9_ack(uint8_t pos) {
  SB_WR(0xC9, (uint8_t)(1u << (pos & 7)));
}

/* SB-PLANE-2 RX read (DPX=1) 0x2a00+off (port0) / 0x2b00+off (port1): the HW-DMA-filled
 * SB-transport RX descriptor plane. */
#define SBP2_RD(base, off)   P1_REG8_rd((uint16_t)((base) + (off)))

/* SB-PLANE TX response plane (DPX=1) 0x2900: the SB-transport TX descriptor the device builds. */
#define SBTX_RD(off)         P1_REG8_rd((uint16_t)(0x2900u + (off)))
#define SBTX_WR(off, v)      P1_REG8_wr((uint16_t)(0x2900u + (off)), (uint8_t)(v))

/* Per-port SB-RX descriptor plane base, selected by the cd3f port (0x06F0): connect edges use
 * port 0/1 (0x2a00/0x2b00); link edges use port 2/3 (0x2c00/0x2d00). */
static __code const uint16_t sb_rxplane_212d[4] = { 0x2a00u, 0x2b00u, 0x2c00u, 0x2d00u };

/* eaac: copy the host connect descriptor from SB-plane-2 into the 0x0777 block that the
 * state-3 routing confirm gates on. */
static void sb_eaac_populate_0777(void) {
  uint16_t rx_plane = sb_rxplane_212d[sb_active_plane_port & 3];
  uint8_t i;
  u4_route_query_response = 1;
  for (i = 0; i < 0x40; i++) {
    u4_host_desc[i] = SBP2_RD(rx_plane, i);
  }
  { static __xdata uint8_t dump_budget = 40;
    if (dump_budget) { dump_budget--; uart_puts("[eaac b="); uart_puthex((uint8_t)(rx_plane >> 8)); uart_puts(" 777="); uart_puthex(u4_host_desc[0x0]);
      uart_puthex(u4_host_desc[0x1]); uart_puthex(u4_host_desc[0x2]); uart_puthex(u4_host_desc[0x3]); uart_putc(']'); } }
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
}

/* Per-descriptor-type work-buffer base offset (ROM 0x21A1). 0xFF = invalid type. */
static __code const uint8_t sb_af38_rom21a1[0x13] = {
  0x00,0x04,0xFF,0x08,0x0C,0xFF,0xFF,0xFF, 0x10,0x14,0x18,0xFF, 0x19,0x1C,0xFF,0x20, 0xFF,0xFF,0x24
};

/* af38 trace latches, printed by [s5]. */
static volatile uint8_t __xdata __at(0x0B40) af38_t50;
static volatile uint8_t __xdata __at(0x0B41) af38_t53;
static volatile uint8_t __xdata __at(0x0B42) af38_t51;
static volatile uint8_t __xdata __at(0x0B43) af38_t4f;
static volatile uint8_t __xdata __at(0x0B44) af38_t6f0;

/* State-3 af38 TX diag budget. */
static volatile uint8_t __xdata __at(0x0B53) af38_s3_budget;

/* af38: read the host connect descriptor from the RX plane, build the device->host TX response
 * into the 0x2900 plane, and trigger the SB-transport TX. */
static void sb_af38_descriptor_response(void) {
  uint16_t rx_plane = sb_rxplane_212d[sb_active_plane_port & 3];
  uint8_t  desc_type, desc_len, desc_dir, work_off = 0, raw_byte1, i, status5, width;
  uint8_t  desc = sb_connect_descriptor;
  uint8_t  status_off = (sb_active_plane_port == 0) ? 0x0D : 0x0E;

  sb_tx_command_desc = (uint8_t)(desc & 0xDE);

  desc_type = SBP2_RD(rx_plane, 0);
  raw_byte1 = SBP2_RD(rx_plane, 1);
  desc_len  = (uint8_t)(raw_byte1 & 0x7F);
  desc_dir  = (uint8_t)(raw_byte1 & 0x80);

  SBTX_WR(0, desc_type);
  SBTX_WR(1, desc_dir);

  if (desc_type < 0x13) {
    width = sb_width_lut[(uint16_t)(0x0 + desc_type)];
    SBTX_WR(1, (uint8_t)(SBTX_RD(1) | width));
    work_off = sb_af38_rom21a1[desc_type];
  }

  status5 = (uint8_t)((SB_RD(status_off) & 0x7F) - 5);
  if (u4_fsm_state == 5) { af38_t50 = desc_type; af38_t53 = work_off; af38_t51 = desc_len; af38_t4f = status5; af38_t6f0 = sb_active_plane_port; }
  if (desc_dir != 0) {
    sb_af38_copy_len = 1;
    SBTX_WR(2, 0);
    if (desc_type < 0x13 &&
        sb_width_lut[(uint16_t)(0x0 + desc_type)] != 0 &&
        desc_len == status5 &&
        sb_branchA_gate[(uint16_t)(0x0 + desc_type)] != 0 &&
        desc_len <= sb_width_lut[(uint16_t)(0x0 + desc_type)]) {
      for (i = 0; i < desc_len; i++)
        u4_work_buf[(uint16_t)(0x0 + (uint8_t)(work_off + i))] = SBP2_RD(rx_plane, (uint8_t)(2 + i));
      if (desc_type == 8) { uart_puts("\r\n[af38-A:cm8]"); }
    } else {
      SBTX_WR(1, (uint8_t)(SBTX_RD(1) & 0x80));
      SBTX_WR(2, 1);
    }
  } else {
    sb_af38_copy_len = desc_len;
    if (desc_type < 0x13 &&
        (width = sb_width_lut[(uint16_t)(0x0 + desc_type)]) != 0 &&
        status5 == 0) {
      if ((uint8_t)(width + 1) <= sb_af38_copy_len) sb_af38_copy_len = width;
      for (i = 0; i < sb_af38_copy_len; i++)
        SBTX_WR((uint8_t)(2 + i), u4_work_buf[(uint16_t)(0x0 + (uint8_t)(work_off + i))]);
    } else {
      SBTX_WR(1, (uint8_t)(SBTX_RD(1) & 0x80));
      sb_af38_copy_len = 0;
    }
  }

  {
    uint8_t status;
    (void)SB_RD(0x0C);
    status = (uint8_t)((sb_af38_copy_len + 8) | (SB_RD(0x0C) & 0x80));
    SB_WR(0x0C, status);
    SB_WR(0x15, sb_tx_command_desc);
  }

  if (u4_fsm_state == 3 && af38_s3_budget) {
    af38_s3_budget--;
    uart_puts("\r\n[a3 S="); uart_puthex(SB_RD(0x15)); uart_puthex(SB_RD(0x10));
    uart_puthex(SB_RD(0x2C)); uart_puthex(SB_RD(0x0C)); uart_puthex(SB_RD(0x0D));
    uart_puthex(SB_RD(0x0E)); uart_puthex(SB_RD(0x18)); uart_puthex(SB_RD(0x28));
    uart_puts("|TX="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBTX_RD(i)); }
    uart_puts("|2a="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBP2_RD(rx_plane, i)); }
    uart_puts("|X="); uart_puthex(u4_fsm_state); uart_puthex(sb_active_plane_port);
    uart_puthex(sb_connect_descriptor); uart_puthex(u4_host_desc[0x0]);
    uart_puts(" 50="); uart_puthex(desc_type); uart_puts(" 51="); uart_puthex(desc_len);
    uart_puts(" 4f="); uart_puthex(status5); uart_putc(']');
  }
  u4lb_d5da(0);
}

/* ebb5: the 0x0765 connect-present setter. */
static void sb_set_connect_present_ebb5(void) {
  if (((sb_connect_descriptor >> 1) & 0x0F) != 0) {
    SB_SET(0x57, 0x08);
    SB_SET(0x61, 0x08);
  }
  sb_connect_present = 1;
}

/* edd9: cd3f's first action on every transport edge: the device->host receive-ACK. */
static void sb_edd9_receive_ack(void) {
  if (P1_RD(0x0109) & 0x01) {
    P1_WR(0x0109, P1_RD(0x0109) & 0xFE);
    SB_WR(0xD8, 0x02);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    u4lb_eb62(0, U4FSM_CONN_ROUT);
    u4lb_98ec(0, 3);
    uart_puts("\r\n[edd9]");
  }
}
static void sb_d4cd_transport_edges(void);
/* cd3f: read the host descriptor, run the connect dispatch (ebb5 / eaac / af38). port: 0/1 =
 * transport edges, 2/3 = link edges (the 0x752.4 gate flips accordingly). */
static void sb_cd3f_dispatch(uint8_t desc4e_off, uint8_t desc752_off) {
  uint8_t desc4e;
  uint8_t desc752;
  uint8_t port = sb_active_plane_port;
  sb_edd9_receive_ack();
  desc4e = SB_RD(desc4e_off);
  sb_connect_descriptor = SB_RD(desc752_off);
  desc752 = sb_connect_descriptor;
#if 0
  uart_puts("[cd p="); uart_puthex(port); uart_puts(" 752="); uart_puthex(desc752);
  uart_puts(" 4e="); uart_puthex(desc4e); uart_puts(" 765="); uart_puthex(sb_connect_present); uart_putc(']');
#endif
  if (sb_connect_present != 0 && (desc752 & 0x60) == 0x60) return;
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
    sb_set_connect_present_ebb5();
  } else if ((desc752 & 0x01) == 0) {
    sb_eaac_populate_0777();
  } else {
    if (((desc752 & 0x40) == 0) || (((desc752 >> 1) & 0x0F) == 0)) {
      sb_af38_descriptor_response();
    }
  }
}
/* d4cd: per-edge transport/link dispatch. Sets 0x06F0 to the port atomically before each cd3f
 * call so eaac/af38 read the matching plane; W1C-acks the edge and toggles the ping-pong latch. */
static void sb_d4cd_transport_edges(void) {
  if (SB_RD(0x28) & 0x08) {
    if (sb_transport_edge_toggle == 0) {
      sb_active_plane_port = 0; sb_cd3f_dispatch(0x28, 0x18);
      SB_WR(0x28, 0x10); SB_WR(0x28, 0x20); SB_WR(0x28, 0x40); SB_WR(0x28, 0x08);
      sb_transport_edge_toggle = 1;
    }
  }
  if (SB_RD(0x2A) & 0x08) {
    if (sb_transport_edge_toggle == 1) {
      sb_active_plane_port = 1; sb_cd3f_dispatch(0x2A, 0x19);
      SB_WR(0x2A, 0x10); SB_WR(0x2A, 0x20); SB_WR(0x2A, 0x40); SB_WR(0x2A, 0x08);
      sb_transport_edge_toggle = 0;
    }
  }
  if (SB_RD(0x81) & 0x08) {
    if (sb_link_edge_toggle == 0) {
      sb_active_plane_port = 2; sb_cd3f_dispatch(0x81, 0x08);
      SB_WR(0x81, 0x10); SB_WR(0x81, 0x20); SB_WR(0x81, 0x40); SB_WR(0x81, 0x08);
      sb_link_edge_toggle = 1;
    }
  }
  if (SB_RD(0x83) & 0x08) {
    if (sb_link_edge_toggle == 1) {
      sb_active_plane_port = 3; sb_cd3f_dispatch(0x83, 0x09);
      SB_WR(0x83, 0x10); SB_WR(0x83, 0x20); SB_WR(0x83, 0x40); SB_WR(0x83, 0x08);
      sb_link_edge_toggle = 0;
    }
  }
}
/* No-op stub: all edges are processed by sb_d4cd_transport_edges. */
static void sb_transport_substate_poll(void) { }

/* edd9 prelude: consume the SB[0x09] read-ack. */
static void sb_chan_prelude(void) {
  (void)SB_RD(0x09);
}

/* db7a: post-connect tunnel-route arm. Branch on 0x07B9 (0 = Connect_U4; !=0 = EnterMode-TBT). */
static void sb_db7a_route_arm(void) {
  if (u4_connect_route_latch == 0) {
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
  u4lb_eb62(0, U4FSM_CONN_ROUT);
  u4lb_98ec(0, 3);
}

/* dea1: post-connect consequence. Runs the heavy SB/PHY arm once per connect session (gated on
 * 0x06EC), arms the lane-bond FSM, and runs db7a. */
static void sb_con_consequence(void) {
  if (u4_conn_consequence_done) return;

  if (P1_RD(0x0109) & 0x01) {
    P1_WR(0x0109, P1_RD(0x0109) & 0xFE);
    SB_WR(0xD8, 0x02);
  }
  SB_WR(0x00, (SB_RD(0x00) & 0xBF) | 0x40);
  SB_WR(0x00, (SB_RD(0x00) & 0x7F) | 0x80);
  SB_WR(0x04, (SB_RD(0x04) & 0xFE) | 0x01);
  SB_WR(0x01, (SB_RD(0x01) & 0xBF) | 0x40);
  SB_WR(0x01, (SB_RD(0x01) & 0x7F) | 0x80);
  P1_WR(0x0100, (P1_RD(0x0100) & 0xEF) | 0x10);
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);
  phy_cc10_cmd(2, 0, 0x15);
  { uint16_t g = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++g < 0x0400); }
  REG_TIMER0_CSR = 0x02;
  P1_WR(0x0100, (P1_RD(0x0100) & 0xBF) | 0x40);
  P1_WR(0x0100, (P1_RD(0x0100) & 0x7F) | 0x80);
  u4_conn_consequence_done = 1;
  sb_db7a_route_arm();
  if (!(REG_LANE_TRAIN_ARM & 0x01) || (REG_LANE_TRAIN_ARM & 0x02)) {
    REG_LANE_TRAIN_ARM = 0x04; REG_LANE_TRAIN_ARM = 0x02;
    REG_LANE_TRAIN_CTRL = (REG_LANE_TRAIN_CTRL & 0xF8) | 0x04;
    REG_LANE_TRAIN_MASK_LO = 0xFF; REG_LANE_TRAIN_MASK_HI = 0xFF;
    REG_LANE_TRAIN_ARM = 0x01;
    u4_lane_train_trigger ^= 0x01;
  }
  if (!sb_8a89_done) sb_run_8a89_pending = 1;
}

/* eed6: post-[Lane Bonded] consequence. Byte-true: lb_lane_bonded_flag=1; sb_init_and_read_field_6f1
 * (SB[0xC9]=0xFF W1C + sb_active_port_rr = (SB[0x24]&6)>>1 re-latch); u4lb_c593() — the POST-BOND
 * tunnel-PHY commit, which (flag==1 branch) SETS plane-2 P1[0x1335] bit1 = the adapter advertise the
 * host CM needs to start reading the router config space. (u4lb_c593 is defined in usb4_lanebond.h,
 * included after this file -> forward-declared.) The prior P1_WR(0x01C8,...) was fabricated; dropped. */
static void u4lb_c593(void);
static void sb_lane_bonded_consequence(void) {
  lb_lane_bonded_flag = 1;
  SB_WR(0xC9, 0xFF);
  sb_active_port_rr = (uint8_t)((SB_RD(0x24) & 0x06) >> 1);
  u4lb_c593();
}

/* e52d: lane-bond complete -> tunnel up -> downstream PCIe bring-up (deferred to super-loop). */
static volatile uint8_t __xdata __at(0x0B4F) sb_tunnel_up_pending;
static void sb_lane_bond_complete_tunnel_up(void) {
  sb_rom_descriptor_load();
  if (u4_route_mode & 0x02) {
    sb_tunnel_up_pending = 1;
  }
  REG_CPU_CTRL_CA60 &= 0xF7;
}

/* c3b2: per-active-port connect descriptor read + dispatch. Selects the SB register pair by
 * 0x06F1, validates (~hi)==lo, dispatches on (lo & 0x0F). */
static void sb_channel_connect_service(void) {
  uint8_t port = sb_active_port_rr;
  uint8_t lo, hi, n;

  sb_edd9_receive_ack();

  if (port == 0)      { lo = SB_RD(0x20); hi = SB_RD(0x22); }
  else if (port == 1) { lo = SB_RD(0x21); hi = SB_RD(0x23); }
  else if (port == 2) { lo = SB_RD(0xA4); hi = SB_RD(0xA6); }
  else                { lo = SB_RD(0xA5); hi = SB_RD(0xA7); }

  if ((uint8_t)(~hi) != lo) {
    uart_puts("[SBch err]");
    return;
  }

  n = lo & 0x0F;
  if (n == 1 || n == 5) {
    sb_route_up_trigger = 1;
    return;
  }
  if (n == 3) {
    if (sb_link_reinit_gate != 0) {
      SB_WR(0x50, 0x40);
      SB_WR(0x5A, 0x40);
      P1_WR(0x0109, (uint8_t)(P1_RD(0x0109) | 0x01));
    }
    SB_WR(0x15, 0x83);
    SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x03));
    u4lb_d5da(1);
    return;
  }
  if (n == 0) {
    if (u4_fsm_state == 3) {
      if (pd_cm_dispatch_sel == 0x69) return;
      return;
    }
    if ((((lo & 0x20) >> 5) & 7) == 0) return;
    SB_WR(0x5A, 0x40);
    u4_work_buf[0x19] &= 0xFD;
    if (!(u4_work_buf[0x19] & 0x01)) {
      lb_laneA_cl0_latch = 0; lb_laneB_cl0_latch = 0;
    } else {
      (void)SB_RD(0xA0);
    }
    return;
  }
}

/* lane_port_map_a (ROM 0x21A1), indexed by 0x0AA2 -> 0x0AA6. 0xFF = invalid/reserved slot. */
static __code const uint8_t lane_port_map_a[19] = {
  0x00,0x04,0xFF,0x08,0x0C,0xFF,0xFF,0xFF,0x10,0x14,0x18,0xFF,0x19,0x1C,0xFF,0x20,0xFF,0xFF,0x24
};

static void sb_clear_cl0_width_latches(void) {
  lb_laneA_cl0_latch = 0;
  lb_laneB_cl0_latch = 0;
}

static void sb_set_d4_peer_cl0(void) {
  SB_WR(0xD4, (uint8_t)((SB_RD(0xD4) & 0xDF) | 0x20));
}

/* e1cb/e2b9: the SB-transport TX answer push. is_e1cb=1 -> READ/route answer (aa4==2);
 * is_e1cb=0 -> WRITE answer (aa4==1). */
static void sb_a5d8_tx(uint8_t is_e1cb) {
  sb_d4cd_transport_edges();   /* stock e1cb@e1d6 / e2b9@e2c4 both LCALL d4cd FIRST (drain the host's
                                * just-posted RX descriptor -> eaac/af38 re-sync host_desc before the
                                * answer). The byte-true sibling u4lb_e1cb_e2b9 (usb4_lanebond.h:773)
                                * does this; this transcription wrongly called the no-op stub. */
  SBTX_WR(0, sb_tx_byte0);
  SBTX_WR(1, (uint8_t)(sb_tx_byte1 | ((sb_tx_flag & 1) << 7)));
  if (sb_tx_flag == 0)
    SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08));
  else
    SB_WR(0x0C, (uint8_t)(((sb_tx_byte0 + 8) & 0xFF) | (SB_RD(0x0C) & 0x80)));
  SB_WR(0x15, is_e1cb ? (uint8_t)((sb_tx_cmd << 1) | 0x41) : (uint8_t)sb_tx_cmd);
  u4lb_d5da(0);
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
  e461_inflight_token = 0x01;
}

/* a5d8: the [Pend Int] device->host router-op responder. */
static void sb_a5d8_pend_int(void) {
  static __xdata uint8_t opcode_byte, opcode, i, op_idx, len;

  uart_puts("\r\n[Pend Int]");

  u4_routerop_desc0 = sb_routerop_hdr0;
  u4_routerop_desc1 = sb_routerop_hdr1;
  u4_routerop_desc2 = sb_routerop_hdr2;
  u4_routerop_desc3 = sb_routerop_hdr3;

  if (!(u4_routerop_desc3 & 0x80)) return;

  u4_routerop_op_lo = u4_routerop_desc0;
  u4_routerop_op_len = u4_routerop_desc1;
  opcode_byte = u4_routerop_desc2;
  u4_routerop_opcode = (uint8_t)(opcode_byte & 7);
  u4_routerop_svid_hi = (uint8_t)(opcode_byte >> 4);
  u4_routerop_flag = (uint8_t)(u4_routerop_desc3 & 1);
  u4_routerop_port = lane_port_map_a[u4_routerop_op_lo];

  opcode = u4_routerop_opcode;
  if (opcode != 0) {
    if (opcode != 1 && opcode != 2) return;
    sb_cdf5_substate_arm = 1;
    if (u4_routerop_flag != 0) {
      pd_msg_type = 0;
      while (pd_msg_type < u4_routerop_op_len && pd_msg_type != 0x40) {
        i = pd_msg_type;
        SBTX_WR(i + 2, sb_routerop_body[i]);
        SBTX_WR(i + 3, sb_routerop_body[0x1 + i]);
        SBTX_WR(i + 4, sb_routerop_body[0x2 + i]);
        SBTX_WR(i + 5, sb_routerop_body[0x3 + i]);
        pd_msg_type = (uint8_t)(i + 4);
      }
    }
    if (u4_routerop_opcode == 1) {
      sb_tx_flag = u4_routerop_flag;
      sb_tx_cmd = (uint8_t)(u4_route_enable_latch | 0x01);
      sb_tx_byte0 = u4_routerop_op_lo;
      sb_tx_byte1 = u4_routerop_op_len;
      sb_a5d8_tx(0);
    } else {
      sb_tx_flag = u4_routerop_flag;
      sb_tx_cmd = u4_routerop_svid_hi;
      sb_tx_byte0 = u4_routerop_op_lo;
      sb_tx_byte1 = u4_routerop_op_len;
      sb_a5d8_tx(1);
    }
    return;
  }

  if (u4_routerop_flag != 0) {
    op_idx = u4_routerop_op_lo;
    if (op_idx < 0x12 &&
        (len = sb_width_lut[(uint16_t)op_idx]) != 0 &&
        sb_branchA_gate[(uint16_t)op_idx] != 0 &&
        u4_routerop_op_len <= len) {
      pd_msg_type = 0;
      while (pd_msg_type < u4_routerop_op_len) {
        i = pd_msg_type;
        u4_work_buf[(uint8_t)(u4_routerop_port + i)] = sb_routerop_body[i];
        pd_msg_type = (uint8_t)(i + 1);
      }
      u4_routerop_desc1 = len;
      if (op_idx == 8) {
        uart_puts("[a5d8:cm8]");
      }
    } else {
      uart_puts("\r\n[RdCmdErr]");
      u4_routerop_desc1 = 0;
      u4_routerop_desc3 = (uint8_t)(u4_routerop_desc3 | 0x04);
    }
  } else {
    op_idx = u4_routerop_op_lo;
    if (op_idx < 0x12 &&
        (len = sb_width_lut[(uint16_t)op_idx]) != 0) {
      if (u4_routerop_op_len > len) u4_routerop_op_len = len;
      pd_msg_type = 0;
      while (pd_msg_type < u4_routerop_op_len) {
        i = pd_msg_type;
        sb_routerop_body[i] = u4_work_buf[(uint8_t)(u4_routerop_port + i)];
        pd_msg_type = (uint8_t)(i + 1);
      }
      u4_routerop_desc1 = len;
    } else {
      uart_puts("\r\n[WrCmdErr]");
      u4_routerop_desc1 = 0;
      u4_routerop_desc3 = (uint8_t)(u4_routerop_desc3 | 0x04);
    }
  }

  u4_routerop_desc3 = (uint8_t)(u4_routerop_desc3 & 0x7F);
  sb_routerop_hdr0 = u4_routerop_desc0;
  sb_routerop_hdr1 = u4_routerop_desc1;
  sb_routerop_hdr2 = u4_routerop_desc2;
  sb_routerop_hdr3 = u4_routerop_desc3;
  SB_WR(0x06, 0x01);
}

/* cdf5: the DEFERRED device->host router-op CONFIG-READ RESPONSE (CODE_BANK1::cdf5..cea9, byte-verified).
 * Stock cb10's super-loop tail runs this whenever the opcode-1/2 router-op (sb_a5d8_pend_int above)
 * armed 0x072A. It builds the lane-config response header (0x0998-0x099B; only hdr1/hdr3 change --
 * hdr0/hdr2 are snapshot/round-trip no-ops) + body (0x099C+) and TRANSMITS it via the SB-transport
 * plane (SB_WR(0x06,0x01) == stock 9934-pack + r3_write_dispatch into bank-2 0x2806, the same strobe
 * the opcode-0 a719 tail uses). THIS IS THE WIRE EVENT THE TB4 HOST BLOCKS ON: without it the host's
 * router-op config-read goes unanswered, the lane-config snap plateaus at 2D2D, and SB[0xA0]/[0xA1]
 * stay 0x07 (bond-ready) instead of advancing to 0x02 (CL0 bonded). `r` = the eda0 selector taken at
 * the call site (0/2 => respond; 1 => nothing this pass, arm stays set so it retries). */
static void sb_cdf5_routerop_response(uint8_t r) {
  uint8_t hdr1_new, hdr3_orig, w3, i, len;
  if (r == 1) return;                                  /* cdff: RET when eda0 selector == 1 */
  sb_cdf5_substate_arm = 0;                            /* ce02-ce06: clear the 0x072A one-shot */
  hdr1_new  = (uint8_t)(u4_host_desc[0x1] & 0x7F);     /* ce27-ce30: a5d = XDATA[0x0778] & 0x7F */
  hdr3_orig = sb_routerop_hdr3;                        /* a5f initial = 0x099B */
  if (r == 2) {                                        /* ce31-ce3c: R5==2 */
    w3 = (uint8_t)(hdr3_orig | 0x02);
  } else {                                             /* ce3e-ce87 */
    w3 = (uint8_t)(hdr3_orig & 0xFB);                  /* ce42: clear bit2 */
    if ((hdr3_orig & 0x01) == 0) {                     /* ce45: bit0 of ORIGINAL hdr3 clear -> body copy */
      if (hdr1_new == 0) w3 = (uint8_t)(w3 | 0x04);    /* ce57-ce63: if (0x0778&0x7F)==0 -> |4 */
      len = hdr1_new; if (len > 0x40) len = 0x40;      /* ce64-ce6e: cap body length at 0x40 */
      for (i = 0; i < len; i++)
        sb_routerop_body[i] = u4_host_desc[(uint8_t)(0x2 + i)];  /* ce72-ce87: 0x099C+i = 0x0779+i */
    } else {                                           /* ce48-ce55: bit0 of ORIGINAL hdr3 set */
      if (u4_host_desc[0x2] != 0) w3 = (uint8_t)(w3 | 0x04);     /* if XDATA[0x0779] != 0 -> |4 */
    }
  }
  w3 = (uint8_t)(w3 & 0x7F);                            /* ce89-ce90: clear bit7 */
  sb_routerop_hdr1 = hdr1_new;                          /* ce99-cea0: 0x0999 = a5d (hdr0/hdr2 unchanged) */
  sb_routerop_hdr3 = w3;                                /* cea1+9934: 0x099B = w3 */
  { static __xdata uint8_t cdf5_dbg = 0;
    if (cdf5_dbg) { cdf5_dbg--; uart_puts("\r\n[cdf5 r="); uart_puthex(r); uart_puts(" h1="); uart_puthex(hdr1_new);
      uart_puts(" w3="); uart_puthex(w3); uart_putc(']'); } }
  SB_WR(0x06, 0x01);                                   /* cea4 9934 + cea7 0be6 r3_write_dispatch: SB-transport TX */
}

/* Fallback set when SB[0x26].1 ([Pend Int]) is observed outside the stock a066 position. */
static volatile uint8_t __xdata __at(0x0B55) sb_pend_int_pending;

/* a066: INT1 source C80A.5 service body. PART 1 = per-channel connect poll; PART 2 =
 * connect/disconnect edge + lane CL0/event servicing (all W1C). */
static volatile uint8_t a066_dbg_budget = 0;
static volatile uint8_t a066_dbg_ret_budget = 0;
static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

  if (a066_dbg_budget && u4_fsm_state == 5) {
    a066_dbg_budget--;
    uart_puts("\r\n[a66 2d="); uart_puthex(SB_RD(0x2D));
    uart_puts(" 2c="); uart_puthex(SB_RD(0x2C));
    uart_puts(" 66="); uart_puthex(SB_RD(0x66));
    uart_puts(" 9e="); uart_puthex(SB_RD(0x9E));
    uart_puts(" 26="); uart_puthex(SB_RD(0x26));
    uart_puts(" c9="); uart_puthex(SB_RD(0xC9));
    uart_puts(" 109="); uart_puthex(P1_RD(0x0109));
    uart_putc(']');
  }

  for (idx = 0; idx < 4; idx++) {
    bm = SB_RD(0xC9);
    if ((bm & (uint8_t)(1u << (4 + idx))) && sb_active_port_rr == idx) {
      sb_channel_connect_service();
      sb_write_c9_ack(idx);
      sb_write_c9_ack((uint8_t)(idx + 4));
      sb_active_port_rr = (uint8_t)((sb_active_port_rr + 1) & 3);
      if (P1_RD(0x0109) & 0x01)
        sb_lane_bond_complete_tunnel_up();
    }
  }

  sb_d4cd_transport_edges();
  sb_transport_substate_poll();

  cs = SB_RD(0x2D);
  if (!(cs & 0x01)) {
    if (SB_RD(0x2C) & 0x01) {
      if (sb_con_print_budget) { uart_puts("\r\n[===SB Con===]\r\n"); sb_con_print_budget--; }
      SB_WR(0x2C, 0x01);
      SB_WR(0x2C, 0x02);
      { uint8_t w = (uint8_t)((SB_RD(0x2D) & 0xFE) | 0x01);
        SB_WR(0x2D, w);
        SB_WR(0x2D, (uint8_t)(SB_RD(0x2D) & 0xFD)); }
      sb_con_consequence();
    }
  } else {
    cs = SB_RD(0x2D);
    if (!(cs & 0x02)) {
      if (SB_RD(0x2C) & 0x02) {
        uart_puts("\r\n[===SB Dis===]\r\n");
        SB_WR(0x2C, 0x02); SB_WR(0x2C, 0x01);
        SB_WR(0x2D, (uint8_t)(SB_RD(0x2D) & 0xFE));
        SB_WR(0x2D, (uint8_t)((SB_RD(0x2D) & 0xFD) | 0x02));
      }
    }
  }

  if (SB_RD(0x66) & 0x01) {
    SB_WR(0x66, 0x01);
    uart_puts("\r\nLane Bonded\r\n");
    sb_lane_bonded_consequence();
    sb_clear_cl0_width_latches();
  }

  if (SB_RD(0x26) & 0x02) {
    sb_a5d8_pend_int();
    SB_WR(0x26, 0x02);
  }

  if (SB_RD(0x9E) & 0x01) {
    SB_WR(0x9E, 0x01);
    uart_puts("\r\nL0:CL0 ");
    uart_puthex(SB_RD(0xA0) & 0x0F);
    SB_WR(0x64, (uint8_t)((SB_RD(0x64) & 0xFE) | 0x01));
    if (!(u4_work_buf[0x19] & 0x02) || ((SB_RD(0xA1) & 0x0F) == 2)) {
      sb_clear_cl0_width_latches();
      sb_set_d4_peer_cl0();
    }
  }

  if (SB_RD(0x9E) & 0x02) {
    SB_WR(0x9E, 0x02);
    uart_puts("\r\nL1:CL0 ");
    uart_puthex(SB_RD(0xA1) & 0x0F);
    SB_WR(0x64, (uint8_t)((SB_RD(0x64) & 0xFD) | 0x02));
    if (!(u4_work_buf[0x19] & 0x01) || ((SB_RD(0xA0) & 0x0F) == 2)) {
      sb_clear_cl0_width_latches();
      sb_set_d4_peer_cl0();
    }
  }

  if (SB_RD(0x66) & 0x04) {
    SB_WR(0x66, 0x04);
    uart_puts("\r\nL0:Abr2");
  }
  if (SB_RD(0x66) & 0x20) {
    SB_WR(0x66, 0x20);
    uart_puts("\r\nL1:Abr2");
  }
  if (SB_RD(0x66) & 0x08) {
    SB_WR(0x66, 0x08);
    uart_puts("\r\nL0:Bnd Fail");
  }
  if (SB_RD(0x66) & 0x40) {
    SB_WR(0x66, 0x40);
    uart_puts("\r\nL1:Bnd Fail");
  }

  if (SB_RD(0x26) & 0x04) {
    SB_WR(0x26, 0x04);
    uart_puts("\r\nL0:Disable");
    SB_WR(0x15, 0x80);
  }
  if (SB_RD(0x26) & 0x10) {
    SB_WR(0x26, 0x10);
    uart_puts("\r\nL1:Disable");
    SB_WR(0x15, 0xA0);
    SB_WR(0x5A, 0x40);
  }

  if (SB_RD(0x9E) & 0x10) {
    SB_WR(0x9E, 0x10);
    uart_puts("\r\nL0:Training");
  }
  if (SB_RD(0x9E) & 0x20) {
    SB_WR(0x9E, 0x20);
    uart_puts("\r\nL1:Training");
  }

  (void)SB_RD(0xF6);
  if (a066_dbg_ret_budget && u4_fsm_state == 5) { a066_dbg_ret_budget--; uart_putc('@'); }
}

/* cb10: per-super-loop SB lane-bond advance. Reads SB[0xA0]/[0xA1] low nibble, compares against
 * the 0x072B/0x072C latches, and on a change updates the latch (NO-OP on a host that stalls). */
static volatile uint8_t __xdata __at(0x0B50) cb10_seen;
static void sb_cb10_lane_advance(void) {
  uint8_t nibble, lat;
  nibble = SB_RD(0xA0) & 0x0F;
  lat = lb_laneA_cl_latch;
  if (nibble != lat) {
    cb10_seen |= 0x01;
    lb_laneA_cl_latch = nibble;
  }
  nibble = SB_RD(0xA1) & 0x0F;
  lat = lb_laneB_cl_latch;
  if (nibble != lat) {
    cb10_seen |= 0x02;
    lb_laneB_cl_latch = nibble;
  }
  if (lb_laneA_cl0_latch != 0 || lb_laneB_cl0_latch != 0) { cb10_seen |= 0x04; }
}

#endif /* SB_ROUTER_H */
