#ifndef SB_ROUTER_H
#define SB_ROUTER_H
/*
 * USB4 SB-router: connect/disconnect, lane-bond, and the SB-transport descriptor engine.
 * Included after sb.h and usb4.h.
 */

static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13);

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
  uint16_t rx_plane = sb_rxplane_212d[PR(0x06F0) & 3];
  uint8_t i;
  PR(0x0775) = 1;
  for (i = 0; i < 0x40; i++) {
    PR(0x0777 + i) = SBP2_RD(rx_plane, i);
  }
  { static __xdata uint8_t dump_budget = 40;
    if (dump_budget) { dump_budget--; uart_puts("[eaac b="); uart_puthex((uint8_t)(rx_plane >> 8)); uart_puts(" 777="); uart_puthex(PR(0x0777));
      uart_puthex(PR(0x0778)); uart_puthex(PR(0x0779)); uart_puthex(PR(0x077A)); uart_putc(']'); } }
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
  uint16_t rx_plane = sb_rxplane_212d[PR(0x06F0) & 3];
  uint8_t  desc_type, desc_len, desc_dir, work_off = 0, raw_byte1, i, status5, width;
  uint8_t  desc = PR(0x0752);
  uint8_t  status_off = (PR(0x06F0) == 0) ? 0x0D : 0x0E;

  PR(0x0753) = (uint8_t)(desc & 0xDE);

  desc_type = SBP2_RD(rx_plane, 0);
  raw_byte1 = SBP2_RD(rx_plane, 1);
  desc_len  = (uint8_t)(raw_byte1 & 0x7F);
  desc_dir  = (uint8_t)(raw_byte1 & 0x80);

  SBTX_WR(0, desc_type);
  SBTX_WR(1, desc_dir);

  if (desc_type < 0x13) {
    width = PR((uint16_t)(0x06F2u + desc_type));
    SBTX_WR(1, (uint8_t)(SBTX_RD(1) | width));
    work_off = sb_af38_rom21a1[desc_type];
  }

  status5 = (uint8_t)((SB_RD(status_off) & 0x7F) - 5);
  if (PR(0x06ED) == 5) { af38_t50 = desc_type; af38_t53 = work_off; af38_t51 = desc_len; af38_t4f = status5; af38_t6f0 = PR(0x06F0); }
  if (desc_dir != 0) {
    PR(0x0754) = 1;
    SBTX_WR(2, 0);
    if (desc_type < 0x13 &&
        PR((uint16_t)(0x06F2u + desc_type)) != 0 &&
        desc_len == status5 &&
        PR((uint16_t)(0x0705u + desc_type)) != 0 &&
        desc_len <= PR((uint16_t)(0x06F2u + desc_type))) {
      for (i = 0; i < desc_len; i++)
        PR((uint16_t)(0x0800u + (uint8_t)(work_off + i))) = SBP2_RD(rx_plane, (uint8_t)(2 + i));
      if (desc_type == 8) { uart_puts("\r\n[af38-A:cm8]"); }
    } else {
      SBTX_WR(1, (uint8_t)(SBTX_RD(1) & 0x80));
      SBTX_WR(2, 1);
    }
  } else {
    PR(0x0754) = desc_len;
    if (desc_type < 0x13 &&
        (width = PR((uint16_t)(0x06F2u + desc_type))) != 0 &&
        status5 == 0) {
      if ((uint8_t)(width + 1) <= PR(0x0754)) PR(0x0754) = width;
      for (i = 0; i < PR(0x0754); i++)
        SBTX_WR((uint8_t)(2 + i), PR((uint16_t)(0x0800u + (uint8_t)(work_off + i))));
    } else {
      SBTX_WR(1, (uint8_t)(SBTX_RD(1) & 0x80));
      PR(0x0754) = 0;
    }
  }

  {
    uint8_t status;
    (void)SB_RD(0x0C);
    status = (uint8_t)((PR(0x0754) + 8) | (SB_RD(0x0C) & 0x80));
    SB_WR(0x0C, status);
    SB_WR(0x15, PR(0x0753));
  }

  if (PR(0x06ED) == 3 && af38_s3_budget) {
    af38_s3_budget--;
    uart_puts("\r\n[a3 S="); uart_puthex(SB_RD(0x15)); uart_puthex(SB_RD(0x10));
    uart_puthex(SB_RD(0x2C)); uart_puthex(SB_RD(0x0C)); uart_puthex(SB_RD(0x0D));
    uart_puthex(SB_RD(0x0E)); uart_puthex(SB_RD(0x18)); uart_puthex(SB_RD(0x28));
    uart_puts("|TX="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBTX_RD(i)); }
    uart_puts("|2a="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBP2_RD(rx_plane, i)); }
    uart_puts("|X="); uart_puthex(PR(0x06ED)); uart_puthex(PR(0x06F0));
    uart_puthex(PR(0x0752)); uart_puthex(PR(0x0777));
    uart_puts(" 50="); uart_puthex(desc_type); uart_puts(" 51="); uart_puthex(desc_len);
    uart_puts(" 4f="); uart_puthex(status5); uart_putc(']');
  }
  PR(0x0AAC) = 0;
  P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));
  SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));
  SB_WR(0x10, 0x01);
  { uint16_t spin = 0; while (((SB_RD(0x2C) >> 2) & 1) == 0 && ++spin < 0x4000) { } }
  SB_WR(0x2C, 0x04);
  phy_cc10_cmd(1, 0, 0x0B);
  SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));
}

/* ebb5: the 0x0765 connect-present setter. */
static void sb_set_connect_present_ebb5(void) {
  if (((PR(0x0752) >> 1) & 0x0F) != 0) {
    SB_SET(0x57, 0x08);
    SB_SET(0x61, 0x08);
  }
  PR(0x0765) = 1;
}

/* edd9: cd3f's first action on every transport edge: the device->host receive-ACK. */
static void sb_edd9_receive_ack(void) {
  if (P1_RD(0x0109) & 0x01) {
    P1_WR(0x0109, P1_RD(0x0109) & 0xFE);
    SB_WR(0xD8, 0x02);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    uart_puts("\r\n[edd9]");
  }
}
static void sb_d4cd_transport_edges(void);
/* cd3f: read the host descriptor, run the connect dispatch (ebb5 / eaac / af38). port: 0/1 =
 * transport edges, 2/3 = link edges (the 0x752.4 gate flips accordingly). */
static void sb_cd3f_dispatch(uint8_t desc4e_off, uint8_t desc752_off) {
  uint8_t desc4e;
  uint8_t desc752;
  uint8_t port = PR(0x06F0);
  sb_edd9_receive_ack();
  desc4e = SB_RD(desc4e_off);
  PR(0x0752) = SB_RD(desc752_off);
  desc752 = PR(0x0752);
  { static __xdata uint8_t dispatch_budget = 16;
    if (dispatch_budget) { dispatch_budget--; uart_puts("[cd p="); uart_puthex(port); uart_puts(" 752="); uart_puthex(desc752);
      uart_puts(" 4e="); uart_puthex(desc4e); uart_puts(" 765="); uart_puthex(PR(0x0765)); uart_putc(']'); } }
  if (PR(0x0765) != 0 && (desc752 & 0x60) == 0x60) return;
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
/* Super-loop connect-present producer: reproduce ebb5's effect at the connect point. */
static void sb_connect_present_poll(void) {
  if (PR(0x0765)) return;
  if (PR(0x06EC)) {
    PR(0x0752) = SB_RD(0x18);
    sb_set_connect_present_ebb5();
  }
}

/* d4cd: per-edge transport/link dispatch. Sets 0x06F0 to the port atomically before each cd3f
 * call so eaac/af38 read the matching plane; W1C-acks the edge and toggles the ping-pong latch. */
static void sb_d4cd_transport_edges(void) {
  if (SB_RD(0x28) & 0x08) {
    if (PR(0x06EE) == 0) {
      PR(0x06F0) = 0; sb_cd3f_dispatch(0x28, 0x18);
      SB_WR(0x28, 0x10); SB_WR(0x28, 0x20); SB_WR(0x28, 0x40); SB_WR(0x28, 0x08);
      PR(0x06EE) = 1;
    }
  }
  if (SB_RD(0x2A) & 0x08) {
    if (PR(0x06EE) == 1) {
      PR(0x06F0) = 1; sb_cd3f_dispatch(0x2A, 0x19);
      SB_WR(0x2A, 0x10); SB_WR(0x2A, 0x20); SB_WR(0x2A, 0x40); SB_WR(0x2A, 0x08);
      PR(0x06EE) = 0;
    }
  }
  if (SB_RD(0x81) & 0x08) {
    if (PR(0x06EF) == 0) {
      PR(0x06F0) = 2; sb_cd3f_dispatch(0x81, 0x08);
      SB_WR(0x81, 0x10); SB_WR(0x81, 0x20); SB_WR(0x81, 0x40); SB_WR(0x81, 0x08);
      PR(0x06EF) = 1;
    }
  }
  if (SB_RD(0x83) & 0x08) {
    if (PR(0x06EF) == 1) {
      PR(0x06F0) = 3; sb_cd3f_dispatch(0x83, 0x09);
      SB_WR(0x83, 0x10); SB_WR(0x83, 0x20); SB_WR(0x83, 0x40); SB_WR(0x83, 0x08);
      PR(0x06EF) = 0;
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
  if (PR(0x07B9) == 0) {
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
}

/* dea1: post-connect consequence. Runs the heavy SB/PHY arm once per connect session (gated on
 * 0x06EC), arms the lane-bond FSM, and runs db7a. */
static void sb_con_consequence(void) {
  if (PR(0x06EC)) return;

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
  PR(0x06EC) = 1;
  sb_db7a_route_arm();
  PR(0x06ED) = 3;
  PR(0x0758) = 0x10;
  if (!(REG_LANE_TRAIN_ARM & 0x01) || (REG_LANE_TRAIN_ARM & 0x02)) {
    REG_LANE_TRAIN_ARM = 0x04; REG_LANE_TRAIN_ARM = 0x02;
    REG_LANE_TRAIN_CTRL = (REG_LANE_TRAIN_CTRL & 0xF8) | 0x04;
    REG_LANE_TRAIN_MASK_LO = 0xFF; REG_LANE_TRAIN_MASK_HI = 0xFF;
    REG_LANE_TRAIN_ARM = 0x01;
    PR(0x0774) ^= 0x01;
  }
  PR(0x0768) = REG_LANE_WIDTH_CNT_HI;
  PR(0x0769) = REG_LANE_WIDTH_CNT_LO;
  if (!sb_8a89_done) sb_run_8a89_pending = 1;
}

/* eed6: post-[Lane Bonded] consequence (0x072D=1 + the SB[0xC9]/page1 0x01C8 confirm). */
static void sb_lane_bonded_consequence(void) {
  PR(0x072D) = 1;
  SB_WR(0xC9, 0xFF);
  P1_WR(0x01C8, (P1_RD(0x01C8) & 0xBF) | 0x40);
}

/* e52d: lane-bond complete -> tunnel up -> downstream PCIe bring-up (deferred to super-loop). */
static volatile uint8_t __xdata __at(0x0B4F) sb_tunnel_up_pending;
static void sb_lane_bond_complete_tunnel_up(void) {
  sb_rom_descriptor_load();
  if (PR(0x09FA) & 0x02) {
    sb_tunnel_up_pending = 1;
  }
  REG_CPU_CTRL_CA60 &= 0xF7;
}

/* c3b2: per-active-port connect descriptor read + dispatch. Selects the SB register pair by
 * 0x06F1, validates (~hi)==lo, dispatches on (lo & 0x0F). */
static void sb_channel_connect_service(void) {
  uint8_t port = PR(0x06F1);
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
    PR(0x0766) = 1;
    return;
  }
  if (n == 3) {
    if (PR(0x0B1C) != 0) SB_WR(0x50, 0x40);
    SB_WR(0x15, 0x83);
    return;
  }
  if (n == 0) {
    if (PR(0x06ED) == 3) {
      if (PR(0x07FF) == 0x69) return;
      return;
    }
    if ((((lo & 0x20) >> 5) & 7) == 0) return;
    SB_WR(0x5A, 0x40);
    PR(0x0819) &= 0xFD;
    if (!(PR(0x0819) & 0x01)) {
      PR(0x074E) = 0; PR(0x074F) = 0;
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

/* e1cb/e2b9: the SB-transport TX answer push. is_e1cb=1 -> READ/route answer (aa4==2);
 * is_e1cb=0 -> WRITE answer (aa4==1). */
static void sb_a5d8_tx(uint8_t is_e1cb) {
  static __xdata uint16_t g;
  sb_transport_substate_poll();
  SBTX_WR(0, PR(0x0AA9));
  SBTX_WR(1, (uint8_t)(PR(0x0AAA) | ((PR(0x0AAB) & 1) << 7)));
  if (PR(0x0AAB) == 0)
    SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08));
  else
    SB_WR(0x0C, (uint8_t)(((PR(0x0AA9) + 8) & 0xFF) | (SB_RD(0x0C) & 0x80)));
  SB_WR(0x15, is_e1cb ? (uint8_t)((PR(0x0AA8) << 1) | 0x41) : (uint8_t)PR(0x0AA8));
  PR(0x0AAC) = 0;
  P1_WR(0x0100, (uint8_t)(P1_RD(0x0100) & 0xFE));
  SB_WR(0x04, (uint8_t)(SB_RD(0x04) & 0xFD));
  SB_WR(0x10, 0x01);
  g = 0; while (((SB_RD(0x2C) & 0x04) == 0) && ++g < 0x4000) { }
  SB_WR(0x2C, 0x04);
  phy_cc10_cmd(1, 0, 0x0B);
  SB_WR(0x0F, (uint8_t)(SB_RD(0x0F) & 0xFE));
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  REG_XFER2_DMA_STATUS = 0x01;
  PR(0x0719) = 0x01;
}

/* a5d8: the [Pend Int] device->host router-op responder. Super-loop only (stack cliff). */
static void sb_a5d8_pend_int(void) {
  static __xdata uint8_t opcode_byte, opcode, i, op_idx, len;

  uart_puts("\r\n[Pend Int]");

  PR(0x0A9D) = PR(0x0998);
  PR(0x0A9E) = PR(0x0999);
  PR(0x0A9F) = PR(0x099A);
  PR(0x0AA0) = PR(0x099B);

  if (!(PR(0x0AA0) & 0x80)) return;

  PR(0x0AA2) = PR(0x0A9D);
  PR(0x0AA3) = PR(0x0A9E);
  opcode_byte = PR(0x0A9F);
  PR(0x0AA4) = (uint8_t)(opcode_byte & 7);
  PR(0x0AA7) = (uint8_t)(opcode_byte >> 4);
  PR(0x0AA5) = (uint8_t)(PR(0x0AA0) & 1);
  PR(0x0AA6) = lane_port_map_a[PR(0x0AA2)];

  opcode = PR(0x0AA4);
  if (opcode != 0) {
    if (opcode != 1 && opcode != 2) return;
    PR(0x072A) = 1;
    if (PR(0x0AA5) != 0) {
      PR(0x0AA1) = 0;
      while (PR(0x0AA1) < PR(0x0AA3) && PR(0x0AA1) != 0x40) {
        i = PR(0x0AA1);
        SBTX_WR(i + 2, PR(0x099C + i));
        SBTX_WR(i + 3, PR(0x099D + i));
        SBTX_WR(i + 4, PR(0x099E + i));
        SBTX_WR(i + 5, PR(0x099F + i));
        PR(0x0AA1) = (uint8_t)(i + 4);
      }
    }
    if (PR(0x0AA4) == 1) {
      PR(0x0AAB) = PR(0x0AA5);
      PR(0x0AA8) = (uint8_t)(PR(0x0718) | 0x01);
      PR(0x0AA9) = PR(0x0AA2);
      PR(0x0AAA) = PR(0x0AA3);
      sb_a5d8_tx(0);
    } else {
      PR(0x0AAB) = PR(0x0AA5);
      PR(0x0AA8) = PR(0x0AA7);
      PR(0x0AA9) = PR(0x0AA2);
      PR(0x0AAA) = PR(0x0AA3);
      sb_a5d8_tx(1);
    }
    return;
  }

  if (PR(0x0AA5) != 0) {
    op_idx = PR(0x0AA2);
    if (op_idx < 0x12 &&
        (len = PR(0x0600 + (uint8_t)(op_idx - 0x0E))) != 0 &&
        PR(0x0700 + (uint8_t)(op_idx + 5)) != 0 &&
        PR(0x0AA3) < len) {
      PR(0x0AA1) = 0;
      while (PR(0x0AA1) < PR(0x0AA3)) {
        i = PR(0x0AA1);
        PR(0x0800 + (uint8_t)(PR(0x0AA6) + i)) = PR(0x099C + i);
        PR(0x0AA1) = (uint8_t)(i + 1);
      }
      PR(0x0A9E) = PR(0x0600 + (uint8_t)(op_idx - 0x0E));
      if (op_idx == 8) {
        uart_puts("[a5d8:cm8]");
      }
    } else {
      uart_puts("\r\n[RdCmdErr]");
      PR(0x0A9E) = 0;
      PR(0x0AA0) = (uint8_t)(PR(0x0AA0) | 0x04);
    }
  } else {
    op_idx = PR(0x0AA2);
    if (op_idx < 0x12 &&
        (len = PR(0x0600 + (uint8_t)(op_idx - 0x0E))) != 0) {
      if (PR(0x0AA3) >= len) PR(0x0AA3) = len;
      PR(0x0AA1) = 0;
      while (PR(0x0AA1) < PR(0x0AA3)) {
        i = PR(0x0AA1);
        PR(0x099C + i) = PR(0x0800 + (uint8_t)(PR(0x0AA6) + i));
        PR(0x0AA1) = (uint8_t)(i + 1);
      }
      PR(0x0A9E) = PR(0x0600 + (uint8_t)(op_idx - 0x0E));
    } else {
      uart_puts("\r\n[WrCmdErr]");
      PR(0x0A9E) = 0;
      PR(0x0AA0) = (uint8_t)(PR(0x0AA0) | 0x04);
    }
  }

  PR(0x0AA0) = (uint8_t)(PR(0x0AA0) & 0x7F);
  PR(0x0998) = PR(0x0A9D);
  PR(0x0999) = PR(0x0A9E);
  PR(0x099A) = PR(0x0A9F);
  PR(0x099B) = PR(0x0AA0);
  SB_WR(0x06, 0x01);
}

/* Set by the a066 ISR when it observes SB[0x26].1 ([Pend Int]); the body runs in the super-loop. */
static volatile uint8_t __xdata __at(0x0B55) sb_pend_int_pending;

/* a066: INT1 source C80A.5 service body. PART 1 = per-channel connect poll; PART 2 =
 * connect/disconnect edge + lane CL0/event servicing (all W1C). */
static volatile uint8_t a066_dbg_budget = 30;
static volatile uint8_t a066_dbg_ret_budget = 30;
static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

  if (a066_dbg_budget && PR(0x06ED) == 5) {
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
    if ((bm & (uint8_t)(1u << (4 + idx))) && PR(0x06F1) == idx) {
      sb_channel_connect_service();
      sb_write_c9_ack(idx);
      sb_write_c9_ack((uint8_t)(idx + 4));
      PR(0x06F1) = (uint8_t)((PR(0x06F1) + 1) & 3);
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
    PR(0x074E) = 0; PR(0x074F) = 0;
  }

  if (SB_RD(0x9E) & 0x01) {
    SB_WR(0x9E, 0x01);
    uart_puts("\r\nL0:CL0 ");
    if ((SB_RD(0xA0) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }
    if (P1_RD(0x0819) & 0x01) { if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; } }
  }

  if (SB_RD(0x9E) & 0x02) {
    SB_WR(0x9E, 0x02);
    uart_puts("\r\nL1:CL0 ");
    if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }
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

  if (SB_RD(0x26) & 0x02) {
    sb_pend_int_pending = 1;
  }

  (void)SB_RD(0xF6);
  if (a066_dbg_ret_budget && PR(0x06ED) == 5) { a066_dbg_ret_budget--; uart_putc('@'); }
}

/* cb10: per-super-loop SB lane-bond advance. Reads SB[0xA0]/[0xA1] low nibble, compares against
 * the 0x072B/0x072C latches, and on a change updates the latch (NO-OP on a host that stalls). */
static volatile uint8_t __xdata __at(0x0B50) cb10_seen;
static void sb_cb10_lane_advance(void) {
  uint8_t nibble, lat;
  nibble = SB_RD(0xA0) & 0x0F;
  lat = PR(0x072B);
  if (nibble != lat) {
    cb10_seen |= 0x01;
    PR(0x072B) = nibble;
  }
  nibble = SB_RD(0xA1) & 0x0F;
  lat = PR(0x072C);
  if (nibble != lat) {
    cb10_seen |= 0x02;
    PR(0x072C) = nibble;
  }
  if (PR(0x074E) != 0 || PR(0x074F) != 0) { cb10_seen |= 0x04; }
}

#endif /* SB_ROUTER_H */
