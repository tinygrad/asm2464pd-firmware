#ifndef SB_ROUTER_H
#define SB_ROUTER_H
/*
 * USB4 SB-router: connect/disconnect, lane-bond, and the SB-transport descriptor engine.
 * Included after sb.h and usb4.h.
 */

static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
/* fwd: bank0_8a89 (the U2/U3/U4 lane-MODE engine, defined in usb4_connect.h) — called inline at
 * bond-complete with desc0=2 (U4 branch), matching stock e52d tail @CODE_BANK1::e56b. */
static void bank0_8a89(uint8_t mode);
/* fwd: e52d transport-up sub-fns defined in usb4_lanebond.h / main.c (both included AFTER this
 * header). The FULL byte-true e52d in sb_lane_bond_complete_tunnel_up runs these in stock order. */
static void u4lb_b031_transport_reinit(uint8_t skip_lane);  /* bank0 b031 (usb4_lanebond.h) */
static void u4lb_e06b(uint8_t v);                 /* bank0 e06b (usb4_lanebond.h) */
static void pcie_power_on(void);                  /* bank0 0x3578 pcie_downstream_link_bringup (main.c) */
static void boot_phy_bceb_set0(uint16_t addr);    /* boot_phy.h (included after this header) */
static void sb_channel_connect_service(void);     /* defined below; used by u4lb_e96c (e52d) */
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
/* State-5 (post-bond) af38 TX diag budget. */
static volatile uint8_t __xdata __at(0x0B54) af38_s5_budget;

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

  if (desc_type < 0x12) {   /* stock af6f: SUBB A,#0x12; JNC (skip if >=0x12) -> body runs iff <0x12 */
    width = sb_width_lut[(uint16_t)(0x0 + desc_type)];
    SBTX_WR(1, (uint8_t)(SBTX_RD(1) | width));
    work_off = sb_af38_rom21a1[desc_type];
  }

  status5 = (uint8_t)((SB_RD(status_off) & 0x7F) - 5);
  if (u4_fsm_state == 5) { af38_t50 = desc_type; af38_t53 = work_off; af38_t51 = desc_len; af38_t4f = status5; af38_t6f0 = sb_active_plane_port; }
  if (desc_dir != 0) {
    sb_af38_copy_len = 1;
    SBTX_WR(2, 0);
    if (desc_type < 0x12 &&   /* stock 99b5: SUBB A,#0x12 (branch-A path, JNC at afbc) */
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
    if (desc_type < 0x12 &&   /* stock 99b5: SUBB A,#0x12 (else path, JNC at b043) */
        (width = sb_width_lut[(uint16_t)(0x0 + desc_type)]) != 0 &&
        status5 == 0) {
      if ((uint8_t)(width + 1) <= sb_af38_copy_len) sb_af38_copy_len = width;
      for (i = 0; i < sb_af38_copy_len; i++)
        SBTX_WR((uint8_t)(2 + i), u4_work_buf[(uint16_t)(0x0 + (uint8_t)(work_off + i))]);
      /* WIDTH-STAGING ATTEMPTED + REVERTED (2026-06-19): advertising lane1 ABSENT (TX[3]/[5]=0) until
         lane0 CL0 STALLED the bond like the CL-walk gate — the device cannot unilaterally stage the bond;
         the host needs the full 2-lane advertisement. Host-coupled. */
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

  /* CMD diag (2026-06-20): the SB-transport command/status at each af38 call, to compare handmade's
   * values at the CL terminal vs stock's commit terminal (stock: 0C=0C 15=40 18=40 A=0101). The 2a
   * route bytes [2:4] identify the terminal (stock reaches 3C3D; handmade tops out at 2CAC). */
  { static __xdata uint8_t cmdbud = 60;
    if (cmdbud) { cmdbud--;
      uart_puts("\r\n[cmd dt="); uart_puthex(desc_type);
      uart_puts(" 0C="); uart_puthex(SB_RD(0x0C)); uart_puts(" 15="); uart_puthex(SB_RD(0x15));
      uart_puts(" 18="); uart_puthex(SB_RD(0x18)); uart_puts(" 28="); uart_puthex(SB_RD(0x28));
      uart_puts(" po="); uart_puthex(sb_active_plane_port);
      uart_puts(" 6F0="); uart_puthex(XDATA_REG8V(0x06F0)); uart_puthex(XDATA_REG8V(0x06F1));
      uart_puts(" 2a="); uart_puthex(P1_RD(0x2A02)); uart_puthex(P1_RD(0x2A03));
      uart_puthex(P1_RD(0x2A04)); uart_puthex(P1_RD(0x2A05));
      uart_puts(" 2b="); uart_puthex(P1_RD(0x2B02)); uart_puthex(P1_RD(0x2B03));
      uart_puthex(P1_RD(0x2B04)); uart_puthex(P1_RD(0x2B05)); uart_putc(']'); } }

  /* AF38SEQ logger (2026-06-20, captured then reverted — RE agent): the ordered host desc-type
   * sequence is IDENTICAL to stock (0C -> 01 -> 0D x N CL-walk; NO 05/08 in EITHER af38 stream).
   * Handmade host walks the SAME CL grant 2487, device replies the SAME TX walk
   * (8080->ABAB->7B7B->CDCD->8DCD->8D8D->C8C8->8888->CC88->8CCC->8C8C->808C). DIVERGENCE is NOT a
   * desc-type: stock host posts the TERMINAL 2a=0D04 3C3D ("3C done" byte) -> SB[0xA0]=02 -> bond;
   * handmade host tops out at 2CAC/2C2C/A02C and never posts 3C3D. Per-lane bytes settle 8B8B in
   * handmade vs stock's two planes carrying DISTINCT staggered partner indices. Not a presentation
   * byte the device controls here — the host withholds the 3C terminal. Logger removed. */

  /* DESC-TYPE+TX logger (2026-06-20, captured then reverted): handmade af38 desc-types = 01 + 0D (CL walk).
     Type-01 reply = TX 0104 6324 0000 (BYTE-TRUE to stock — NOT the cm8 divergence). The 0D CL-walk replies
     cycle TX[4:6] through 8C8C/8D8D/8888/CDCD/CC88/8CC0/88C8 etc. (handmade's walker, mostly lockstep) vs
     stock's STAGGERED 8DCD (per reference_stock_bond_groundtruth). So the cm8 trigger is the 0D CL-walk
     reply staggering, not type-01. (Re-add this logger to compare handmade's 0D-reply sequence to stock's.) */

  if (u4_fsm_state == 3 && af38_s3_budget) {
    af38_s3_budget--;
    /* comprehensive state-3 dump, field set byte-matched to stock app/patch_stocksbstate.py [p15]
     * so the two diff line-for-line. SB_RD=page-1 0x2800+off; P1_RD=page-1 0x0000+off; PR=plain XDATA. */
    { uint8_t i;
      uart_puts("\r\n[a3 S="); uart_puthex(SB_RD(0x15)); uart_puthex(SB_RD(0x10));
      uart_puthex(SB_RD(0x2C)); uart_puthex(SB_RD(0x0C)); uart_puthex(SB_RD(0x0D));
      uart_puthex(SB_RD(0x0E)); uart_puthex(SB_RD(0x18)); uart_puthex(SB_RD(0x28));
      uart_puts("|TX="); for (i = 0; i < 8; i++) uart_puthex(SBTX_RD(i));
      uart_puts("|2a="); for (i = 0; i < 8; i++) uart_puthex(P1_RD((uint16_t)(0x2A00 + i)));
      uart_puts("|2b="); for (i = 0; i < 8; i++) uart_puthex(P1_RD((uint16_t)(0x2B00 + i)));
      uart_puts("|W="); for (i = 0; i < 4; i++) uart_puthex(u4_work_buf[(uint16_t)(0x1C + i)]);
      uart_puts("|X="); uart_puthex(PR(0x0719)); uart_puthex(PR(0x0758)); uart_puthex(PR(0x06ED));
      uart_puthex(PR(0x06EE)); uart_puthex(PR(0x0775)); uart_puthex(PR(0x0776)); uart_puthex(PR(0x0777));
      uart_puts("|H="); uart_puthex(PR(0xCCD9)); uart_puthex(PR(0xC80A)); uart_puthex(PR(0xCC37)); uart_puthex(PR(0xCA60));
      uart_puts("|D="); uart_puthex(P1_RD(0x0109)); uart_puthex(SB_RD(0xD8));
      for (i = 0x3A; i <= 0x3D; i++) uart_puthex(SB_RD(i));
      uart_puts("|L="); uart_puthex(SB_RD(0xA0)); uart_puthex(SB_RD(0xA1));
      uart_puthex(PR(0xE302)); uart_puthex(PR(0xE751)); uart_puthex(PR(0xE764)); uart_puthex(PR(0xC8FF));
      uart_puts("|19="); uart_puthex(u4_work_buf[0x19]); uart_puthex(u4_work_buf[0x1A]);
      uart_puts("|hd="); uart_puthex(u4_host_desc[0x0]); uart_puthex(desc_type);
      uart_puthex(desc_len); uart_puthex(status5); uart_putc(']'); }
  }
  if (u4_fsm_state == 5 && af38_s5_budget && SB_RD(0x18) != 0) {
    af38_s5_budget--;
    uart_puts("\r\n[a5 18="); uart_puthex(SB_RD(0x18)); uart_puts(" 0C="); uart_puthex(SB_RD(0x0C));
    uart_puts(" 15="); uart_puthex(SB_RD(0x15)); uart_puts(" 2C="); uart_puthex(SB_RD(0x2C));
    uart_puts("|TX="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(SBTX_RD(i)); }
    uart_puts("|6A="); { uint8_t i; for (i = 0; i < 4; i++) uart_puthex(SB_RD((uint8_t)(0x6A + i))); }
    uart_puts(" 84="); uart_puthex(SB_RD(0x84)); uart_puthex(SB_RD(0x85));
    uart_puts(" 9E="); uart_puthex(SB_RD(0x9E)); uart_puts(" 66="); uart_puthex(SB_RD(0x66));
    uart_puts(" A="); uart_puthex(SB_RD(0xA0)); uart_puthex(SB_RD(0xA1));
    /* read-only orientation/per-lane probe (zero risk): which port this af38 serves (po), the
     * phy_lane_gate (expect 00, == stock e34b pre-bond), the two primary-orientation bits c2c3.0/c343.0,
     * the per-lane arm strobes SB[0x50]/SB[0x5A], and the per-desc staging offset wo. Diff lane0 vs lane1. */
    uart_puts(" po="); uart_puthex(sb_active_plane_port);
    uart_puts(" g="); uart_puthex(phy_lane_gate);
    uart_puts(" or="); uart_puthex((uint8_t)(PR(0xC2C3) & 1)); uart_puthex((uint8_t)(PR(0xC343) & 1));
    uart_puts(" 5x="); uart_puthex(SB_RD(0x50)); uart_puthex(SB_RD(0x5A));
    uart_puts(" wo="); uart_puthex(work_off);
    /* comprehensive state-5 fields (byte-matched to stock patch_stocksbstate.py [E]) for the diff. */
    uart_puts("|2a="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(P1_RD((uint16_t)(0x2A00 + i))); }
    uart_puts("|2b="); { uint8_t i; for (i = 0; i < 8; i++) uart_puthex(P1_RD((uint16_t)(0x2B00 + i))); }
    uart_puts("|W="); { uint8_t i; for (i = 0; i < 4; i++) uart_puthex(u4_work_buf[(uint16_t)(0x1C + i)]); }
    uart_puts("|X="); uart_puthex(PR(0x0719)); uart_puthex(PR(0x0758)); uart_puthex(PR(0x06ED));
    uart_puthex(PR(0x06EE)); uart_puthex(PR(0x0775)); uart_puthex(PR(0x0776)); uart_puthex(PR(0x0777));
    uart_puts("|H="); uart_puthex(PR(0xCCD9)); uart_puthex(PR(0xCC37)); uart_puthex(PR(0xCA60));
    uart_puts("|19="); uart_puthex(u4_work_buf[0x19]); uart_puthex(u4_work_buf[0x1A]);
    uart_puts("|l2="); uart_puthex(lb_loop2_state[0]); uart_puthex(lb_loop2_state[1]);
    uart_puts(" l1="); uart_puthex(lb_loop1_state[0]); uart_puthex(lb_loop1_state[1]);
    uart_puts(" cv="); uart_puthex(lb_cl_value[0]); uart_puthex(lb_cl_value[1]);
    uart_puts(" 719="); uart_puthex(PR(0x0719)); uart_putc(']');
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
  /* *** CONNECT-TIME LANE-TRAIN RE-ARM REMOVED — byte-true to stock dea1 (2026-06-22) ***
   * Stock sb_con_consequence (dea1 @CODE_BANK1::dea1) ENDS at `u4_conn_consequence_done=1; LJMP db7a`
   * (disasm verified: defd MOVX@0x6EC; defe LJMP 0xdb7a) — it does NOT re-arm lane training here. The
   * removed CCE0-CCE3 lane-train RESTART strobe was an inlined copy of stock u4lb_lane_train_ctrl_
   * something (ec51); stock calls ec51 ONLY from state-4 b0b4 (CODE_BANK1::b20c) and the ee57 throttle
   * helper (ee62) — NEVER from the connect-event consequence. This is a byte-true correctness fix; the
   * strobe still runs at the correct time via u4lb_ec51 in state-4 b0b4 (usb4_lanebond.h, [c2@ec51]) +
   * the ee57 helper. Bond-safe (verified CL0/CL0). NOTE (HW-proven this session): removing it does NOT
   * change E302 — the "E302 mode-1 decays early" theory is REFUTED: STOCK's E302 ALSO decays
   * 0x97->0x93->0x83 at [ConnRout] BEFORE the bond (app/patch_stock_e302seq.py, A0=07/66=00/1407=00),
   * and stock raises 1407.0 + bonds with E302=0x83. E302 mode is NOT the width-event gate. See memory
   * project_inband_routercs_read_wall.md (2026-06-22 E302-premise-refuted section). */
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

/* e3b7 (bank0 CODE:e3b7), called from e52d with R7=2: CC17=4;CC17=2 (xdata_write_seq); since
 * (2&1)==0 SKIP the 0x92C4 clear; since (2>>1&1)==1 -> B480 |= 1 (boot_phy_bceb_set0) + c2e6(...,0)
 * which with its last arg 0 only runs pcie_link_up_check_b432_e765 and returns. Byte-true. */
static void u4lb_e3b7(uint8_t r7) {
  XDATA_REG8(0xCC17) = 0x04;
  XDATA_REG8(0xCC17) = 0x02;
  if (r7 & 0x01) {                          /* (param&1): clear 0x92C4 bit0 (NOT taken for r7=2) */
    REG_POWER_MISC_CTRL &= 0xFE;
  }
  if ((r7 >> 1) & 0x01) {                   /* (param>>1&1): taken for r7=2 */
    boot_phy_bceb_set0(0xB480);             /* B480 |= 1 */
    /* c2e6(...,0): last arg 0 -> only pcie_link_up_check_b432_e765 (= (B432&7)==7 && (E765&2)),
     * result discarded; main loop skipped. Inlined (cm_link_up_check is in cm_tunnel.h, included
     * after this header). Side-effect-free read. */
    (void)((REG_POWER_CTRL_B432 & 0x07) == 0x07 && (REG_SYS_CTRL_E765 & 0x02) != 0);
  }
}

/* c35b (bank0 CODE:c35b-c3cd): the REAL stock sb_channel_connect_service -- the SB PHY/GPIO connect-
 * config writer (C620/C655/C65A/C623), gated on the connect-service state 0x0B30 / 0x0B31 / 0x0AE4.
 * Stock's bank0_de16 (= u4lb_e96c below) calls THIS at boot.  NAMING NOTE: handmade's existing
 * `sb_channel_connect_service` (defined further down) is actually a port of stock's c3b2 SB-descriptor
 * DISPATCHER, NOT this c35b writer -- u4lb_e96c previously called that wrong function, so C620/C655/C65A
 * were never programmed at boot.  Found via emulate/diff_trace.py (2026-06-22): stock writes C620/C655/
 * C65A 43500x in its main loop; handmade 0x.  e730(addr) = `*addr=(*addr&0xE0)|5`. */
static void sb_phy_connect_config_c35b(void) {
  uint8_t s = XDATA_REG8(0x0B30);
  if (s == 0x02) {
    REG_GPIO_CTRL_0 = (uint8_t)((REG_GPIO_CTRL_0 & 0xE0) | 0x05);   /* e730(0xC620) */
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
      XDATA_REG8(0xC623) = (uint8_t)((XDATA_REG8(0xC623) & 0xE0) | 0x05);  /* e730(0xC623) */
      REG_PHY_CFG_C65A = (uint8_t)((REG_PHY_CFG_C65A & 0xF7) | 0x08);
    } else {
      if (s1 == 0x01) REG_PHY_CFG_C655 = (uint8_t)(REG_PHY_CFG_C655 & 0xF7);
      else            REG_PHY_CFG_C655 = (uint8_t)((REG_PHY_CFG_C655 & 0xF7) | 0x08);
      XDATA_REG8(0xC623) = (uint8_t)(XDATA_REG8(0xC623) & 0xE0);
      REG_PHY_CFG_C65A = (uint8_t)((REG_PHY_CFG_C65A & 0xF7) | 0x08);
    }
  }
}

/* e96c (bank0 CODE:e96c): zero 0x0B30-0x0B33; e726 (CD31=4;CD31=2); CD30=(CD30&0xF8)|5;
 * CD32=0,CD33=0xC7 (PHY DMA addr=0xC700); CC2A=(CC2A&0xF8)|4; CC2C=CC2D=0xC7; then the c35b
 * SB-PHY connect-config (NOT the c3b2 descriptor dispatcher). Byte-true. */
static void u4lb_e96c(void) {
  XDATA_REG8(0x0B30) = 0; XDATA_REG8(0x0B31) = 0;
  XDATA_REG8(0x0B32) = 0; XDATA_REG8(0x0B33) = 0;
  REG_CPU_TIMER_CTRL_CD31 = 0x04;           /* e726 */
  REG_CPU_TIMER_CTRL_CD31 = 0x02;
  REG_PHY_DMA_CMD_CD30 = (uint8_t)((REG_PHY_DMA_CMD_CD30 & 0xF8) | 0x05);
  REG_PHY_DMA_ADDR_LO = 0x00;
  REG_PHY_DMA_ADDR_HI = 0xC7;
  REG_TIMER_CFG_CC2A = (uint8_t)((REG_TIMER_CFG_CC2A & 0xF8) | 0x04);
  REG_TIMER_CFG_CC2C = 0xC7;
  REG_TIMER_CFG_CC2D = 0xC7;
  sb_phy_connect_config_c35b();
}

/* d7cd (bank0 CODE:d7cd-d80f): the STEADY-STATE connect-service RE-RUN.  Stock calls this EVERY
 * superloop iteration (via bank0_c7a5_stub_0507, unconditional in main_boot_and_superloop @2f80) to
 * re-program the SB PHY/GPIO connect path (C620/C655/C65A via c35b) from the live B481 lane-count +
 * CD31 timer.  handmade ran u4lb_e96c (-> c35b) only ONCE at boot and NEVER re-serviced it.
 *
 * EMULATION DIFFERENTIAL (emulate/diff_trace.py --steady, 2026-06-22): stock re-touches
 * C620/C655/C65A (R+W) + CD31 + the 0x0B30 recompute EVERY steady-state iteration (x1624 in a 15%
 * tail); handmade re-touches NONE of them.  This is the documented steady-state execution-path gap:
 * the connect/link state is not continuously re-driven, so the 1->2 width transition that raises
 * P1[0x1407].0 is never re-evaluated.  Byte-true to the d7cd disassembly:
 *   if ((CA81 & 1) != 1) {
 *     r7 = B481;  B481 = 0xFF;                         (latch + clear lane-count)
 *     if (u4_reinit_pending(0x0B2F) == 0) {
 *       if ((r7 & 3) != 0) { e726(); 0x0B2F = (r7&3) - 1; }   (NOTE: writes 0x0B2F = lanecount-1)
 *       if ((CD31&1)==0 || (CD31>>1&1)==1) 0x0B30 = 1; else 0x0B30 = 2;
 *     } else 0x0B30 = 0;
 *     c35b();                                          (= sb_phy_connect_config_c35b: C620/C655/C65A)
 *   }
 * BOND-SAFETY: in the bonded steady state 0x0B30 settles (lanecount-1=1 -> reinit_pending!=0 ->
 * 0x0B30=0 branch) and the c35b writes are IDEMPOTENT (write-back the same value), matching stock.
 * The B481=0xFF latch + 0x0B2F dual-use are reproduced exactly so the state machine tracks stock. */
static void sb_connect_service_reservice_d7cd(void) {
  uint8_t r7;
  if ((REG_CPU_CTRL_CA81 & 0x01) == 0x01) return;        /* d7d1 JB CA81.0 -> ret */
  r7 = REG_PCIE_LINK_CTRL_B481;                           /* d7d7 read lane-count */
  REG_PCIE_LINK_CTRL_B481 = 0xFF;                         /* d7db latch/clear */
  if (u4_reinit_pending == 0) {                           /* d7df read 0x0B2F; d7e0 JNZ */
    uint8_t lc = (uint8_t)(r7 & 0x03);
    if (lc != 0) {
      REG_CPU_TIMER_CTRL_CD31 = 0x04;                     /* e726 */
      REG_CPU_TIMER_CTRL_CD31 = 0x02;
      u4_reinit_pending = (uint8_t)(lc - 1);              /* d7ea DEC A; 0x0B2F = lanecount-1 */
    }
    { uint8_t cd31 = REG_CPU_TIMER_CTRL_CD31;
      if (((cd31 & 0x01) == 0) || ((cd31 >> 1) & 0x01))   /* d7f0/d7f4 */
        XDATA_REG8(0x0B30) = 1;
      else
        XDATA_REG8(0x0B30) = 2; }
  } else {
    XDATA_REG8(0x0B30) = 0;                                /* d807 reinit-pending branch */
  }
  sb_phy_connect_config_c35b();                            /* d80c LCALL c35b */
}

/* e52d: lane-bond complete -> device transport/PCIe-downstream bring-up. FIRED FROM THE a066 ISR
 * PART1 (gated XDATA P1[0x0109]&1, host-INDEPENDENT) — see sb_router_event_handler.
 *
 * *** FULL BYTE-TRUE e52d (CODE_BANK1::e52d), 2026-06-20 ***  Prior port did ONLY rom_load + CA60 +
 * 8a89 — the entire transport-up was GUTTED. Stock order (disasm e52d-e56e):
 *   b031()                              [05d4]  SB-transport / in-band control-adapter REINIT (re-arms EC06)
 *   e06b(0)                             [0598]  re-apply lane mask -> PHY 0x3F
 *   sb_lane_descriptor_loader(p3,p4)    [b7a4]  = sb_rom_descriptor_load (DROM/lane desc reload)
 *   sb_enter_sleep(..)                  [eb0a]  tunnel power-state (gated route_mode&0x81); see note below
 *   if (XDATA[0x09FA].1) {                       (= u4_route_mode.1, the tunnel route mode)
 *     if (af38_t53) e3b7(p2,p3,p4, R7=2) [04da]  CC17 seq + B480|1 + link-up check
 *     R7 = XDATA[0x0AEF];  pcie_downstream_link_bringup(p3,p4) [3578]  = pcie_power_on (downstream link)
 *     e96c()                            [04fd]  PHY-DMA/timer config + sb_channel_connect_service
 *     sb_clr_7e8_set_b2f()              [9a1c]  0x07E8=0; 0x0B2F=1
 *   }
 *   CA60 &= 0xF7
 *   if (XDATA[0x0AF1].0) 8a89(2)         [04f3]  U4 lane-MODE commit
 *
 * *** HW VERDICT (2026-06-20): the FULL byte-true e52d was assembled + tested on the BOARD (run
 * from main()'s deferred super-loop since this ISR gate P1[0x0109]&1 NEVER fires in handmade — the
 * host withholds the bond commit). RESULT: b031's SB-transport REINIT (the e52d FIRST step) RE-DRIVES
 * the just-bonded lanes -> L0:Abr2/L1:Abr2 loop, even in stock's byte-true order, even WITHOUT the
 * pcie block. [b031 EN ec06=00] -> b031 does NOT raise EC06 either. pcie_power_on alone (no b031)
 * keeps the bond stable + reaches PCIe-78-CONNECTED but 1203=00/c80a=20 (no width event, transport
 * still down, config read -110). So the e52d transport-up CANNOT be fired device-side at handmade's
 * pre-commit CL0 state — stock only runs it AFTER the host posts 0x0109 (bond committed). The byte-
 * true ORDER does not save it; b031 is the disturber. ***
 * *** HW VERDICT UPDATE (2026-06-22): the responder-lever was executed — arm sb_link_reinit_gate=1 +
 * set P1[0x0109].0 at the STABLE bond (main.c tup_arm_mode harness) -> this handler FIRES ([TUP! 109=73]
 * reproduced). b031 runs to completion BUT poisons the SB read engine (e5b0 descriptor reset) -> the
 * device super-loop HANGS on the next SB read (hard hang, not Abr2). EC06 never fires. The bond IS
 * stable in the host's ~414ms route=1 window (host oracle: tb_cfg_get_upstream_port(route=1) -> -110)
 * but the device can neither answer nor bring up the responder. See project_b031_not_ec06_gate. ***
 * The DEFAULT body below runs rom_load + CA60 + 8a89(2) (bond-safe); b031/e06b are gated behind
 * tup_b031_enable (default 0) — invoked only when explicitly re-testing the lever. Byte-true ports of
 * b031 (u4lb_b031_transport_reinit) + e06b (u4lb_e06b) live in usb4_lanebond.h as reference. */
static volatile uint8_t __xdata __at(0x0B4F) sb_tunnel_up_pending;
/* 2026-06-22 ROUTE=1 RESPONDER ARM: the host-ftrace lever. e52d is the in-band config RESPONDER
 * bring-up; it answers the host's post-bond route=1 TB_CFG read (TB_CFG_SWITCH offset 0). It runs ONLY
 * when P1[0x0109].0 is set (the n==3 connect-descriptor branch, gated sb_link_reinit_gate). Stock arms
 * that gate via a522->e06b(1) (the width event). handmade never gets a522, so we arm the gate manually
 * at the STABLE bond (A=0202) — then the next n==3 descriptor sets P1[0x0109] and THIS handler fires.
 * It runs the FULL byte-true e52d incl. b031 (the transport reinit that raises EC06 so the C0A5
 * router-op mailbox can answer route=1). Gated one-shot via tup_e52d_done. */
static volatile uint8_t __xdata __at(0x0B57) tup_e52d_done;
static volatile uint8_t __xdata __at(0x0B59) tup_dbg_budget;
static volatile uint8_t __xdata __at(0x0B5A) tup_b031_enable;   /* 0=skip b031 (bond-safe), 1=run it */
static volatile uint8_t __xdata __at(0x0B5B) tup_arm_mode;      /* 0=detect-only,1=set109,2=run e52d */
static void sb_lane_bond_complete_tunnel_up(void) {
  if (tup_e52d_done) return;
  tup_e52d_done = 1;

  if (tup_dbg_budget) { tup_dbg_budget--;
    uart_puts("\r\n[TUP! 109="); uart_puthex(P1_RD(0x0109));
    uart_puts(" ec06="); uart_puthex(XDATA_REG8V(0xEC06)); uart_putc(']'); }

  /* b031 OMITTED (test): b031 at the bond HANGS the SB-transport read interface (device super-loop
   * stops after e52d; even transport-only b031 kills SB reads). Run e52d WITHOUT b031 to check the
   * bond holds + whether the host posts route=1 / EC06 fires on its own. */
  if (tup_b031_enable) u4lb_b031_transport_reinit(1);
  sb_rom_descriptor_load();                          /* [b7a4] sb_lane_descriptor_loader (DROM/lane re-seed) */
  if (u4_route_mode & 0x02) {                        /* if (XDATA[0x09FA].1) — the tunnel-route block */
    sb_tunnel_up_pending = 1;                        /* defer pcie bring-up to super-loop (bond-safe) */
  }
  REG_CPU_CTRL_CA60 &= 0xF7;
  /* stock e52d tail @CODE_BANK1::e56b: `8a89(0, desc0=2)` (U4 lane-MODE branch) gated XDATA[0x0AF1].0. */
  if (XDATA_REG8V(0x0AF1) & 0x01) {
    bank0_8a89(2);                                  /* U4 branch (desc0=2) */
  }

  if (tup_dbg_budget != 0xFF) {
    uart_puts("\r\n[TUP done ec06="); uart_puthex(XDATA_REG8V(0xEC06));
    uart_puts(" ea90="); uart_puthex(REG_SYS_CTRL_EA90); uart_putc(']'); }
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
    { static __xdata uint8_t ob = 6;       /* CL-WALK diag. **2026-06-22 PREMISE CORRECTED via direct
       * STATE-5 WALKER hook (CODE_BANK1::8000 entry, /tmp/patch_walk.py — stock still enumerates):**
       * STOCK's cl-walk terminal is SYMMETRIC clv=DD (BOTH lanes cl_idx D, w1C[1E:1F]=8D8D, 6A=01A301A3,
       * C2C3/C343=1C1C). The earlier "stock=0C0D consecutive" was an a066-hook SAMPLING ARTIFACT (a066
       * fires POST-walk; sampled 6A at a transient — gave 010301A3 one run, 01030103 another). HANDMADE
       * TRANSIENTLY reaches the EXACT stock terminal (1E=8D8D, 6A=01A301A3) but the host REJECTS it and
       * KEEPS walking the cl_idx (D->8->C->0 cycling) -> the lanes desync chasing the moving target
       * (clv 080C/000C/0000). cl_idx is a pure verbatim ECHO of the host (host_desc[0x2+lane]&0xF,
       * usb4_lanebond.h:1593); the cl_cfg compose + tables (sb_cap_field_desc/sb_lane_rate_desc) are
       * byte-correct (produce 01A3 for cl_idx D exactly like stock). So the bond gate is NOT the cl-walk
       * pair (device presents stock-identical 6A/1E at the D-terminal) — it is the LOOP1 WIDTH-FINALIZE /
       * SB[0x66] / 1407.0 width-event the host reads to ACCEPT the (D,D) bond. DO NOT chase consecutive
       * (C,D) (wrong target) or the e461 read-race (refuted) — chase why the host rejects handmade's
       * byte-identical (D,D). See memory reference_stock_bond_groundtruth (2026-06-22). */
      if (ob) { ob--;
        uart_puts("\r\n[CLW 779="); uart_puthex(u4_host_desc[0x02]); uart_puthex(u4_host_desc[0x03]);
        uart_puts(" L1=");  uart_puthex(lb_loop1_state[0]); uart_puthex(lb_loop1_state[1]);
        uart_puts(" L2=");  uart_puthex(lb_loop2_state[0]); uart_puthex(lb_loop2_state[1]);
        uart_puts(" clv="); uart_puthex(lb_cl_value[0]); uart_puthex(lb_cl_value[1]);
        uart_puts(" 6A=");  uart_puthex(SB_RD(0x6A)); uart_puthex(SB_RD(0x6B));
        uart_puthex(SB_RD(0x6C)); uart_puthex(SB_RD(0x6D));
        uart_puts(" A=");   uart_puthex(SB_RD(0xA0)); uart_puthex(SB_RD(0xA1));
        uart_puts(" 77BC="); uart_puthex(u4_host_desc[0x04]); uart_puthex(u4_host_desc[0x05]);
        uart_puts(" w1C="); uart_puthex(u4_work_buf[0x1C]); uart_puthex(u4_work_buf[0x1D]);
        uart_puthex(u4_work_buf[0x1E]); uart_puthex(u4_work_buf[0x1F]);
        /* PHY lock/rate AT CL0 — is the high-speed link actually up, or only the sideband? */
        uart_puts(" pll="); uart_puthex(REG_PHY_LANEA_LOCK_C2D0); uart_puthex(REG_PHY_LANEB_LOCK_C350);
        uart_puts(" rt="); uart_puthex(REG_PHY_LANEA_C294); uart_puthex(REG_PHY_LANEB_C314);
        uart_puts(" e710="); uart_puthex(REG_LINK_WIDTH_E710); uart_puts(" 6db="); uart_puthex(REG_PHY_VENDOR_CTRL_C6DB);
        uart_putc(']'); } }
    /* *** COMPREHENSIVE LINK/ROUTER-OP/CONNECT SFR DUMP at the bond (2026-06-20) ***
     * The never-fully-diffed SFR bank. Order EXACTLY matches app/patch_stock_sfr.py so the two
     * [SFR=...] strings are byte-comparable. Fires at A=0202 (both lanes CL0), budget 4. */
    { static __xdata uint8_t sfrb = 4;
      if (sfrb) { sfrb--;
        uart_puts("\r\n[SFR C805="); uart_puthex(XDATA_REG8V(0xC805));
        uart_puts(" C806="); uart_puthex(XDATA_REG8V(0xC806));
        uart_puts(" C809="); uart_puthex(XDATA_REG8V(0xC809));
        uart_puts(" C80A="); uart_puthex(XDATA_REG8V(0xC80A));
        uart_puts(" C8B0="); uart_puthex(XDATA_REG8V(0xC8B0));
        uart_puts(" C8C7="); uart_puthex(XDATA_REG8V(0xC8C7));
        uart_puts(" C8FF="); uart_puthex(XDATA_REG8V(0xC8FF));
        uart_puts(" CA06="); uart_puthex(XDATA_REG8V(0xCA06));
        uart_puts(" CA60="); uart_puthex(XDATA_REG8V(0xCA60));
        uart_puts(" CA81="); uart_puthex(XDATA_REG8V(0xCA81));
        uart_puts(" CA86="); uart_puthex(XDATA_REG8V(0xCA86));
        uart_puts(" CC30="); uart_puthex(XDATA_REG8V(0xCC30));
        uart_puts(" CC36="); uart_puthex(XDATA_REG8V(0xCC36));
        uart_puts(" CC37="); uart_puthex(XDATA_REG8V(0xCC37));
        uart_puts(" CC3B="); uart_puthex(XDATA_REG8V(0xCC3B));
        uart_puts(" E302="); uart_puthex(XDATA_REG8V(0xE302));
        uart_puts(" E40F="); uart_puthex(XDATA_REG8V(0xE40F));
        uart_puts(" E710="); uart_puthex(XDATA_REG8V(0xE710));
        uart_puts(" E716="); uart_puthex(XDATA_REG8V(0xE716));
        uart_puts(" E717="); uart_puthex(XDATA_REG8V(0xE717));
        uart_puts(" E763="); uart_puthex(XDATA_REG8V(0xE763));
        uart_puts(" E795="); uart_puthex(XDATA_REG8V(0xE795));
        uart_puts(" E7E3="); uart_puthex(XDATA_REG8V(0xE7E3));
        uart_puts(" EA80="); uart_puthex(XDATA_REG8V(0xEA80));
        uart_puts(" EA88="); uart_puthex(XDATA_REG8V(0xEA88));
        uart_puts(" EA89="); uart_puthex(XDATA_REG8V(0xEA89));
        uart_puts(" EA90="); uart_puthex(XDATA_REG8V(0xEA90));
        uart_puts(" EC00="); uart_puthex(XDATA_REG8V(0xEC00));
        uart_puts(" EC04="); uart_puthex(XDATA_REG8V(0xEC04));
        uart_puts(" EC05="); uart_puthex(XDATA_REG8V(0xEC05));
        uart_puts(" EC06="); uart_puthex(XDATA_REG8V(0xEC06));
        uart_puts(" B402="); uart_puthex(XDATA_REG8V(0xB402));
        uart_puts(" B480="); uart_puthex(XDATA_REG8V(0xB480));
        /* WIDTH-EVENT page-1 source registers (DPX=1): the C80A.4 gate chain. P1[0x1407].0 /
         * P1[0x1203].7 are HW-set on a real 1->2 width transition; P1[0x1603].0/.1 are the
         * adapter-event evt0/evt1; P1[0x1508] bits feed bank1_d855 (PcieTunnel-Deassert). */
        uart_puts(" 1407="); uart_puthex(P1_RD(0x1407));
        uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
        uart_puts(" 1603="); uart_puthex(P1_RD(0x1603));
        uart_puts(" 1508="); uart_puthex(P1_RD(0x1508));
        uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
        uart_puts(" 128F="); uart_puthex(P1_RD(0x128F));
        /* tunnel-event AGGREGATION MASK (the C80A.4 gate): stock=1406:0x16/1507:0x61,
         * pre-fix handmade=0x1F/0x7F; P1[0x1406].3 masks the tunnel evt -> P1[0x1407].3. */
        uart_puts(" 1406="); uart_puthex(P1_RD(0x1406));
        uart_puts(" 1507="); uart_puthex(P1_RD(0x1507));
        uart_putc(']'); } }
    /* PAGE-1 ADAPTER-CONFIG / WIDTH-EVENT block — byte-comparable with app/patch_stock_sfr.py's
     * [SFR ... ] page-1 tail. The width-event SOURCE (1200/1201/1203/1206/1207) + advertise
     * (1334/1335/134D/1285) + lane/tunnel/adapter evt blocks. Budget 2 (UART burst). */
    { static __xdata uint8_t sfrpb = 2;
      if (sfrpb) { sfrpb--;
        uart_puts("\r\n[SFRP 1200="); uart_puthex(P1_RD(0x1200));
        uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
        uart_puts(" 1202="); uart_puthex(P1_RD(0x1202));
        uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
        uart_puts(" 1206="); uart_puthex(P1_RD(0x1206));
        uart_puts(" 1207="); uart_puthex(P1_RD(0x1207));
        uart_puts(" 1208="); uart_puthex(P1_RD(0x1208));
        uart_puts(" 1210="); uart_puthex(P1_RD(0x1210));
        uart_puts(" 1280="); uart_puthex(P1_RD(0x1280));
        uart_puts(" 1285="); uart_puthex(P1_RD(0x1285));
        uart_puts(" 128F="); uart_puthex(P1_RD(0x128F));
        uart_puts(" 1334="); uart_puthex(P1_RD(0x1334));
        uart_puts(" 1335="); uart_puthex(P1_RD(0x1335));
        uart_puts(" 134D="); uart_puthex(P1_RD(0x134D));
        uart_puts(" 1403="); uart_puthex(P1_RD(0x1403));
        uart_puts(" 1404="); uart_puthex(P1_RD(0x1404));
        uart_puts(" 1405="); uart_puthex(P1_RD(0x1405));
        uart_puts(" 1406="); uart_puthex(P1_RD(0x1406));
        uart_puts(" 1407="); uart_puthex(P1_RD(0x1407));
        uart_puts(" 1408="); uart_puthex(P1_RD(0x1408));
        uart_puts(" 1503="); uart_puthex(P1_RD(0x1503));
        uart_puts(" 1504="); uart_puthex(P1_RD(0x1504));
        uart_puts(" 1505="); uart_puthex(P1_RD(0x1505));
        uart_puts(" 1506="); uart_puthex(P1_RD(0x1506));
        uart_puts(" 1507="); uart_puthex(P1_RD(0x1507));
        uart_puts(" 1508="); uart_puthex(P1_RD(0x1508));
        uart_puts(" 1603="); uart_puthex(P1_RD(0x1603));
        uart_puts(" 1604="); uart_puthex(P1_RD(0x1604));
        uart_putc(']'); } }
  }

  if (SB_RD(0x66) & 0x04) {
    SB_WR(0x66, 0x04);
    uart_puts("\r\nL0:Abr2");
    { static __xdata uint8_t ab0 = 5;   /* lane-SM state AT the abort (pre-CL0), budgeted */
      if (ab0) { ab0--;
        uart_puts(" A="); uart_puthex(SB_RD(0xA0)); uart_puthex(SB_RD(0xA1));
        uart_puts(" 9E="); uart_puthex(SB_RD(0x9E)); uart_puts(" 64="); uart_puthex(SB_RD(0x64));
        uart_puts(" D4="); uart_puthex(SB_RD(0xD4)); uart_puts(" 2C="); uart_puthex(SB_RD(0x2C));
        uart_puts(" 779="); uart_puthex(u4_host_desc[0x02]); uart_puthex(u4_host_desc[0x03]);
        uart_puts(" clv="); uart_puthex(lb_cl_value[0]); uart_puthex(lb_cl_value[1]);
        uart_puts(" 19="); uart_puthex(u4_work_buf[0x19]);
        uart_puts(" w1C="); uart_puthex(u4_work_buf[0x1C]); uart_puthex(u4_work_buf[0x1D]);
        uart_puthex(u4_work_buf[0x1E]); uart_puthex(u4_work_buf[0x1F]);
        uart_puts(" fsm="); uart_puthex(PR(0x0AAD)); uart_putc(']'); } }
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
