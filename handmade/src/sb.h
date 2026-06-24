#ifndef SB_H
#define SB_H
/*
 * USB4 sideband-assert keystone: bring up the SB block so the host trains USB4 lanes.
 */

static void boot_phy_dd42(uint8_t param);

/* page-1 / sideband accessors (DPX=1 window, restore DPX=0) */
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
/* page-1 RMW helpers (clear/set) on the 0x010000 plane */
#define P1_CLR(off, m)  P1_WR((off), P1_RD(off) & (uint8_t)~(m))
#define P1_SET(off, m)  P1_WR((off), P1_RD(off) | (uint8_t)(m))

/* SB block lives at page-1 0x012800 + off. */
#define SB_RD(off)      P1_REG8_rd((uint16_t)(0x2800u + (off)))
#define SB_WR(off, v)   P1_REG8_wr((uint16_t)(0x2800u + (off)), (uint8_t)(v))
#define SB_CLR(off, m)  SB_WR((off), SB_RD(off) & (uint8_t)~(m))
#define SB_SET(off, m)  SB_WR((off), SB_RD(off) | (uint8_t)(m))

/* SB descriptor-engine plane: page-1 0x011200 + off (channel-READY commit plane). */
#define P12_RD(off)     P1_REG8_rd((uint16_t)(0x1200u + (off)))
#define P12_WR(off, v)  P1_REG8_wr((uint16_t)(0x1200u + (off)), (uint8_t)(v))

/* Lane-rate SB descriptor ROM table (used when C8FF==4). */
static __code const uint8_t sb_lane_rate_desc[0x10] = {
  0x00,0x06,0x0B,0x0E,0x13,0x00,0x05,0x0A, 0x0E,0x11,0x00,0x05,0x08,0x0D,0x00,0x04
};

/* per-CL cap-field source (stock ROM CODE:0x0a92), seeded into lb_cap_field[0x072E] alongside
 * sb_lane_flip. c586/cm_SBER/the LOOP2 CL-emit OR this into the CL config low byte
 * (cl_cfg_lo |= lb_cap_field[cl_idx]); leaving it uninitialised reads the 0x55 XDATA poison and the
 * device emits e.g. CL0 01F5 instead of stock 0103 -> host refuses CL0 (SB[0xA1] stalls at 0x01). */
static __code const uint8_t sb_cap_field_desc[0x10] = {
  0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01, 0x01,0x01,0x03,0x03,0x03,0x03,0x07,0x02
};

/* Runs FIRST: orientation/connect lane map + SB-PHY-RX descriptor + mailbox strobe. */
static void sb_lane_flip_init(void) {
  uint8_t flip = REG_PHY_VENDOR_CTRL_C6DB & 0x01;
  uart_puts("[flp=");
  uart_puthex(flip);
  uart_puts("]");

  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);

  if (u4_enter_usb_accepted != 0 || u4_connect_route_latch != 0) P1_CLR(0x0100, 0x01);
  else                                    P1_SET(0x0100, 0x01);

  if (flip) {
    P1_SET(0x0102, 0x03);
    P1_SET(0x0101, 0x03);
  } else {
    P1_CLR(0x0102, 0x03);
    P1_CLR(0x0101, 0x03);
  }

  if (u4_enter_usb_accepted != 0 || u4_connect_route_latch != 0) {
    P1_WR(0x0101, (P1_RD(0x0101) & 0xEF) | 0x10);
    P1_CLR(0x0101, 0x20);
    P1_SET(0x0101, 0x80);
    REG_LINK_MODE_CTRL &= ~0x03;
  } else {
    P1_CLR(0x0101, 0x10);
    P1_CLR(0x0101, 0x80);
    REG_LINK_MODE_CTRL |= 0x03;
  }

  SB_WR(0xD1, (SB_RD(0xD1) & 0xEF) | 0x10);
  SB_WR(0x49, 0xA0);
  u4_conn_consequence_done = 0;

  SB_WR(0x94, 0x02); SB_WR(0x95, 0x71); SB_WR(0x96, 0x00);
  SB_WR(0x98, 0x3E); SB_WR(0x99, 0x80);
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  REG_XFER2_DMA_CTRL &= 0xEF;
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_XFER2_DMA_CTRL = (REG_XFER2_DMA_CTRL & 0xF8) | 0x04;
  REG_XFER2_DMA_ADDR_LO = 0x00;
  REG_XFER2_DMA_ADDR_HI = 200;
  SB_CLR(0xCF, 0x01);
  SB_WR(0x53, 0xFF);
  SB_WR(0x5D, 0xFF);
  SB_CLR(0x27, 0x01);
  SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);
  SB_CLR(0x2C, 0x01);
  REG_INT_CTRL = (REG_INT_CTRL & 0xF7) | 0x08;
  SB_CLR(0x67, 0x40);
  lb_laneA_cl_latch = 0x07; lb_laneB_cl_latch = 0x07;
  /* clear the cap-field (stock boot-clears XDATA; the handmade does not -> 0x55 poison) then seed
   * it + the lane-flip table from ROM when the link is the Gen3 (C8FF==0x04) rate. */
  { uint8_t k; for (k = 0; k < 0x10; k++) lb_cap_field[(uint16_t)(0x0 + k)] = 0; }
  if (REG_LANE_RATE_C8FF == 0x04) {
    uint8_t k;
    for (k = 0; k < 0x10; k++) {
      sb_lane_flip[(uint16_t)(0x0 + k)] = sb_lane_rate_desc[k];
      lb_cap_field[(uint16_t)(0x0 + k)] = sb_cap_field_desc[k];
    }
  }
}

/* ROM descriptor loader: copies router/DROM identity tables into staging XDATA + PID latches. */
static __code const uint8_t drom_identity[0x64] = {
  0x4C,0x17,0x00,0x00,0x64,0x24,0x00,0x00, 0x41,0x50,0x50,0x20,0x45,0x4D,0x20,0x20,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0xD3,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};
static __code const uint8_t lane_descriptor[0x10] = {
  0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0C,0x0C, 0x0C,0x0C,0x0C,0x07,0x07,0x07,0x07,0x07
};
/* af38 connect-descriptor width LUT (0x06F2+i) + BRANCH-A presence gate (0x0705+i). */
static __code const uint8_t width_lut[0x13] = {
  0x04,0x04,0x00,0x04,0x04,0x00,0x00,0x00, 0x04,0x04,0x01,0x00, 0x03,0x04,0x00,0x04, 0x00,0x00,0x10
};
static __code const uint8_t branchA_gate[0x13] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x01
};

static void sb_rom_descriptor_load(void) {
  uint8_t i;
  for (i = 0; i < 0x64; i++) u4_work_buf[i] = drom_identity[i];
  u4_work_buf[0x4] = pd_product_pid_lo;
  u4_work_buf[0x5] = pd_product_pid_hi;
  if (!(u4_mode_flag & 0x80)) u4_work_buf[0x1B] &= ~0x02;
  for (i = 0; i < 0x10; i++) sb_lane_desc[i] = lane_descriptor[i];
  for (i = 0; i < 0x13; i++) {
    sb_width_lut[(uint16_t)(0x0 + i)] = width_lut[i];
    sb_branchA_gate[(uint16_t)(0x0 + i)] = branchA_gate[i];
  }
  if (u4_cap20g_gate0) {
    u4_work_buf[0x1A] = (uint8_t)((u4_work_buf[0x1A] & 0xDF) | 0x20);
    if ((u4_enter_usb_accepted != 0 && u4_route_confirm_07cc < 3) ||
        (u4_connect_route_latch != 0 && (u4_confirm_input_cf == 1 || u4_confirm_input_cf == 2))) {
      u4_work_buf[0x1A] &= 0xDF;
    }
  }
  if (u4_cap20g_gate1 == 0) u4_work_buf[0x1A] &= 0xED;

  lb_lane_width_latch0 = 1; u4_route_query_response = 0; sb_connect_present = 0; sb_route_up_trigger = 0; lb_walk_oneshot_flag = 0;
  u4_fsm_state = U4FSM_IDLE; cm_conn_routing_substate = CONNRT_PRINT_STATUS; lb_lane_desc_idx[0x0] = 0x0F; lb_lane_desc_idx[0x1] = 0x0F; u4_coldboot_seed_gate = 1;
  sb_cdf5_substate_arm = 0; lb_lane_bonded_flag = 0;
  SB_WR(0x2C, 0x04);
  SB_WR(0x28, 0x08);
  SB_WR(0x2A, 0x08);
  SB_WR(0xC9, 0x01); SB_WR(0xC9, 0x02);
  SB_WR(0xC9, 0x04); SB_WR(0xC9, 0x08);
  SB_WR(0xC9, 0x10); SB_WR(0xC9, 0x20);
  SB_WR(0xC9, 0x40); SB_WR(0xC9, 0x80);
  SB_WR(0x66, 0x08); SB_WR(0x66, 0x40);
  sb_transport_edge_toggle = SB_RD(0x24) & 0x01;
  sb_link_edge_toggle = SB_RD(0x80) & 0x01;
  sb_active_port_rr = (uint8_t)((SB_RD(0x24) & 0x06) >> 1);
  e461_inflight_token = 0;
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  lb_laneA_cl0_latch = 0; lb_laneB_cl0_latch = 0;
  SB_CLR(0xD4, 0x20);
  SB_WR(0x8F, (SB_RD(0x8F) & 0xEF) | 0x10);
}

/* Runs SECOND: SB-block RMWs + PHY descriptor seed + ROM tables + PHY-reg config. */
static void sb_block_init(void) {
  uart_puts("[SB Init]");

  REG_PHY_RXPLL_RESET = 0; REG_PHY_CTRL_C20F = 0; REG_PHY_CDR_SEED_C210 = 0;
  REG_PHY_CDR_SEED_C211 = 0; REG_PHY_CDR_SEED_C212 = 0;
  REG_PHY_CDR_SEED_C214 = 0; REG_PHY_CDR_SEED_C215 = 0; REG_PHY_CDR_SEED_C216 = 0;
  REG_PHY_CDR_SEED_C217 = 0;

  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);
  P1_CLR(0x0100, 0x01);

  SB_WR(0x2C, 0x01); SB_WR(0x2C, 0x02);
  SB_WR(0x26, 0x02);
  SB_WR(0x66, 0x01);
  SB_CLR(0x2D, 0x01); SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);
  SB_CLR(0x29, 0x08);
  SB_CLR(0x2B, 0x08);
  SB_CLR(0xC8, 0xF0);   /* stock bb80: 96b2 clears SB[0xC8] bits 4,5,6 + anl 0x7f clears bit7 = clear 0xF0.
                         * Was 0x90 (bits 4,7 only) — left bits 5,6 set, mis-arming 2 of 4 SB-transport
                         * channels so SB[0x2C].4/.5 never latched (0xC1 vs stock 0xF1) -> host never
                         * advanced SB[0x18] 05->63 (route-commit) -> no 2-lane bond. */
  SB_CLR(0x27, 0x02);
  SB_CLR(0x67, 0x81);
  SB_WR(0x81, 0x08);
  SB_WR(0x83, 0x08);
  SB_CLR(0x82, 0x08);
  SB_CLR(0x84, 0x08);
  SB_WR(0x9E, 0x01); SB_WR(0x9E, 0x02);
  SB_WR(0x66, 0x04); SB_WR(0x66, 0x20);
  SB_CLR(0x9F, 0x03);
  SB_CLR(0x67, 0x24);
  SB_WR(0x9E, 0x10); SB_WR(0x9E, 0x20);
  SB_CLR(0x9F, 0x30);
  u4_conn_consequence_done = 0;

  sb_rom_descriptor_load();

  SB_WR(0x01, (SB_RD(0x01) & 0xBF) | 0x40);
  SB_WR(0x01, (SB_RD(0x01) & 0x7F) | 0x80);
  SB_CLR(0xC4, 0x02);
  SB_CLR(0x27, 0x14);

  /* PHY bank0 RMW (DPX=0) */
  REG_PHY_CONFIG &= ~0x08;
  REG_PHY_LANEA_C2C4 = (REG_PHY_LANEA_C2C4 & 0xBF) | 0x40;
  REG_PHY_LANEA_C2DC &= 0xC0;
  REG_PHY_LANEB_C344 = (REG_PHY_LANEB_C344 & 0xBF) | 0x40;
  REG_PHY_LANEB_C35C &= 0xC0;

  /* PHY lane config */
  phy_lane_gate = 0; phy_lane_cap[0x0] = 3; phy_lane_cap[0x1] = 3; phy_cdr_arm_mask = 0;
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
  SB_CLR(0x1D, 0x02);

  SB_WR(0xBA, 0x3F);
  SB_WR(0xBD, 0x3F);

}

/* Program the PCIe tunnel link width across 4 lanes. */
static void sb_pcie_width_ramp(uint8_t width) {
  REG_PCIE_LINK_STATE = width; REG_PCIE_LINK_STATE_HI_B435 = width; REG_PCIE_LANE_CONFIG = width; REG_PCIE_LANE_CONFIG_HI_B437 = width;
}

/*
 * Descriptor engine on the 0x12 plane: control primitives RMW ENGINE[0x34..0x36], data bytes land
 * at ENGINE[0x3C..0x3F], commit arms SB[0x2C].4-7 (channel-READY).
 */
static void eng_a30c(uint8_t cur, uint8_t v) {
  P12_WR(cur, v); P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x80));
}
static void eng_a308(uint8_t cur, uint8_t v) {
  P12_WR(cur, (uint8_t)((v & 0xF0) | 0x0F));
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x80));
}
static void eng_a2df(uint8_t cur, uint8_t v) {
  P12_WR(cur, v); P12_WR((uint8_t)(cur + 1), (uint8_t)(P12_RD((uint8_t)(cur + 1)) & 0xE0));
}
static void eng_a31c(uint8_t cur, uint8_t v) {
  P12_WR(cur, v);
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0xC0) | 0x04));
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x40));
}
static void eng_a348(uint8_t cur, uint8_t v) {
  P12_WR(cur, v); (void)P12_RD((uint8_t)(cur + 1));
}
static void eng_a327(uint8_t cur, uint8_t v) {
  P12_WR(cur, v); P12_WR(cur, (uint8_t)((P12_RD(cur) & 0x3F) | 0x40));
}

/* Commit the staged descriptor (GO + kick + bounded poll + cleanup); arms SB[0x2C].4-7.
 * RETURNS the commit's threaded A. Disasm of the stock commit tail (CODE:e7fb->e83d->e711):
 *   e7fb: R1=0x37; A=RMW(0x37,(A&0x7F)|0x80); write 0x37=A          -> LJMP e83d
 *   e83d: write 0x38=1; poll (read 0x38, JB bit0 -> spin)           -> LJMP e711
 *   e711: A=read 0x35; A&=0xC0; write 0x35=A;
 *         CLR A; a367: write 0x3C=0,write 0x3D=0 (R1->0x3E);
 *         write 0x3E=A(=0); INC R1; LJMP 0be6: write 0x3F=A(=0)  [TAIL]
 * The tail is 0be6 (R3=2 -> MOVX @DPTR,A; RET), which leaves A UNCHANGED = the last value
 * written = 0x00 (from CLR A). So the commit RETURNS A = 0x00.
 * The descriptor engines (d4c8/e4d2/cbf8) thread this: e.g. d4c8 does
 *   a2f9(write 0x3D=0x40; commit) -> A(=0x00); A=(A&0xF0)|0x08 (=0x08); a31c(0x3D,A).
 * Returning 0x00 makes that threading byte-true (was VOID -> callers fed stale/garbage A). */
static uint8_t u4c_sb_desc_commit(void) {
  uint8_t commit; uint16_t g;
  commit = (uint8_t)((P12_RD(0x37) & 0x7F) | 0x80);
  P12_WR(0x37, commit);
  P12_WR(0x38, 0x01);
  for (g = 0; (P12_RD(0x38) & 0x01) && g < 0x2000; g++) { }
  /* e711: reg0x35 = (LIVE reg35 & 0xC0);
   * then CLR A; LCALL a367 -> a367 writes [0x3C]=0; INC R1->0x3D; writes [0x3D]=0; INC R1->0x3E; RET
   *   (a367 returns with R1=0x3E; the 0be6/0adc write helper does NOT auto-increment R1).
   * e711 TAIL then: e71f LCALL 0be6 writes [0x3E]=0; e722 INC R1->0x3F; e723 LJMP 0be6 writes [0x3F]=0.
   * => stock e711 zeros ALL FOUR descriptor bytes 0x3C/0x3D/0x3E/0x3F at every commit.
   * The d4c8 engine re-stages only 0x3D between back-to-back commits and
   *  relies on e711 having re-zeroed 0x3E/0x3F each commit; without it, STALE 0x3E/0x3F leak into the
   *  next SB-transport/lane-config descriptor the host CM reads.) Returns A=0 (CLR A through 0be6). */
  P12_WR(0x35, (uint8_t)(P12_RD(0x35) & 0xC0));
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x00); P12_WR(0x3E, 0x00); P12_WR(0x3F, 0x00);
  return 0x00;  /* commit's threaded A (CLR-A tail through 0be6) */
}

/* ==== b031 transport-reinit ENGINES (Phase-3 port; see B031_TRANSPORT_REINIT_PORT.md) ==== */

/* bank1 e8d6 = control/transport WORKING-BUFFER seed (PLAIN XDATA, DPX=0).
 * Zero 0x0994..0x09E3 (0x50 bytes) then 5 seeds. Disasm-verified byte-true.
 * MUST run after u4c_e5b0() and before sb_rom_descriptor_load()/u4lb_c17f(). */
static void u4c_e8d6_seed_workbuf(void) {
  uint16_t a;
  for (a = 0x0994; a != 0x09E4; a++) {
    XDATA_REG8V(a) = 0;
  }
  XDATA_REG8V(0x0995) = 0x06;
  XDATA_REG8V(0x09DD) = 0x20;   /* == u4lb_lane_active_flags */
  XDATA_REG8V(0x09DC) = 0x10;
  XDATA_REG8V(0x09E2) = 0x04;
  XDATA_REG8V(0x09E0) = 0x06;
}

/* a31c (CODE:a31c) byte-true: write[cur]=v; INC R1; A=read[cur+1]&0xC0|0x04; RETURN A (NO write-back).
 * The shared eng_a31c (sb.h above) ADDS two spurious write-backs to cur+1 (a left-over from an earlier
 * mis-port). d4c8 discards a31c's A but the cur+1 WRITES are observable, so use the exact primitive
 * here to stay byte-true. Leaves R1 at cur+1 (callers that INC after must account for that — d4c8 does). */
static uint8_t eng_a31c_exact(uint8_t cur, uint8_t v) {
  P12_WR(cur, v);
  return (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0xC0) | 0x04);
}

/* CODE:d4c8 transport-edge descriptor ENGINE (NOT the bank1 d4cd event poll;
 * renamed to avoid the sb_d4cd_transport_edges collision). P12 engine plane.
 * R1 thread (a2ff sets R1=0x34):
 *   a348@0x34 -> R1=0x35 ; a327@0x35 (R1 unch) ; INC->0x36 ; a2df@0x36 -> R1=0x37
 *   R1=0x3D ; a2f9@0x3D (commit -> A=0) ; a31c@0x3D using (A&0xF0)|8 -> R1=0x3E ; INC->0x3F ; a2df@0x3F -> 0x40
 *   R1=0x3D ; a2f9@0x3D (commit -> A=0) ; a31c@0x3D using (A&0xF0)|2 -> R1=0x3E ; INC->0x3F ; a2df@0x3F -> 0x40
 *   R1=0x3D ; write 0x3D=0x40 (plain) ; commit. */
static void u4c_d4c8_edge_engine(void) {
  uint8_t v, commit_a;

  /* d4c8: a2ff(R1=0x34); A=read[0x34]&0xF0|2; a348(0x34,A) -> R1=0x35 */
  v = (uint8_t)((P12_RD(0x34) & 0xF0) | 0x02);
  eng_a348(0x34, v);
  /* d4d2: A=read[0x35]&0xC0|3; a327(0x35,A) (R1 stays 0x35) */
  eng_a327(0x35, (uint8_t)((P12_RD(0x35) & 0xC0) | 0x03));
  /* d4d9: INC R1 -> 0x36 ; A=5 ; a2df(0x36,5) -> R1=0x37 */
  eng_a2df(0x36, 0x05);

  /* d4df: R1=0x3D ; A=0x40 ; a2f9(write 0x3D=0x40; commit -> A) */
  P12_WR(0x3D, 0x40); commit_a = u4c_sb_desc_commit();
  /* d4e6: A=(commit_a&0xF0)|8 ; a31c(0x3D,A) -> R1=0x3E */
  (void)eng_a31c_exact(0x3D, (uint8_t)((commit_a & 0xF0) | 0x08));
  /* d4ed: INC R1 -> 0x3F ; A=9 ; a2df(0x3F,9) -> R1=0x40 */
  eng_a2df(0x3F, 0x09);

  /* d4f3: R1=0x3D ; A=4 ; a2f9(write 0x3D=4; commit -> A) */
  P12_WR(0x3D, 0x04); commit_a = u4c_sb_desc_commit();
  /* d4fa: A=(commit_a&0xF0)|2 ; a31c(0x3D,A) -> R1=0x3E */
  (void)eng_a31c_exact(0x3D, (uint8_t)((commit_a & 0xF0) | 0x02));
  /* d501: INC R1 -> 0x3F ; A=5 ; a2df(0x3F,5) -> R1=0x40 */
  eng_a2df(0x3F, 0x05);

  /* d507: R1=0x3D ; A=0x40 ; 0be6 (plain write 0x3D=0x40) ; LJMP e7fb (commit) */
  P12_WR(0x3D, 0x40); (void)u4c_sb_desc_commit();
}

/* CODE:e4d2 edge descriptor. P12 engine plane. Byte-true to CODE:e4d2 (disasm-verified):
 *   a2ff           -> R1=0x34 (A = residual ACC from prior commit; high nibble only, == reg[0x34] hi)
 *   A=(A&0xF0)|0x04
 *   a348(0x34,A)   -> write[0x34]=A; INC R1->0x35; A=read[0x35]
 *   A&=0x3F; write[R1=0x35]=A (plain 0be6)
 *   INC R1->0x36; A=0
 *   a2df @0x36     -> write[0x36]=0; INC R1->0x37; A=read[0x37]; A&=0xE0; write[0x37]=A
 *   R1=0x3E; write[0x3E]=0x04 (plain 0be6)
 *   LJMP e7fb (commit)
 * NOTE the old port was triple-wrong: a348 readback ignored, &0x3F written to 0x34 not 0x35,
 * and a2df threaded at 0x35 not 0x36. */
static void u4c_e4d2_edge(void) {
  uint8_t a;
  /* a2ff residual-A high nibble is reg[0x34] hi (d4c8 last wrote 0x34 = resid&0xF0|0x02). */
  a = (uint8_t)((P12_RD(0x34) & 0xF0) | 0x04);
  /* a348(0x34,a): write 0x34, INC->0x35, A=read[0x35] */
  P12_WR(0x34, a);
  a = P12_RD(0x35);
  /* A&=0x3F; write[0x35]=A */
  P12_WR(0x35, (uint8_t)(a & 0x3F));
  /* INC->0x36; A=0; a2df @0x36 */
  eng_a2df(0x36, 0x00);
  /* R1=0x3E; write[0x3E]=4 */
  P12_WR(0x3E, 0x04);
  u4c_sb_desc_commit();
}

/* a2eb (CODE:a2eb) byte-true: R1=0x3c; write[0x3c]=0xCC; INC->0x3d; write[0x3d]=0xCC (A still 0xCC);
 *   A=0x08; INC->0x3e; falls into a2f9: write[0x3e]=0x08; commit. Leaves R1=0x3d, A=0 (commit). */
static void u4c_a2eb_block(void) {
  P12_WR(0x3C, 0xCC);
  P12_WR(0x3D, 0xCC);
  P12_WR(0x3E, 0x08);
  (void)u4c_sb_desc_commit();
}

/* a365 (CODE:a365) byte-true: A=0x66; R1=0x3c; write[0x3c]=0x66; INC->0x3d; write[0x3d]=0x66 (A still
 *   0x66); INC->0x3e; RET. Leaves R1=0x3e, does NOT touch 0x3e/0x3f.
 * a3d2 (CODE:a3d2) byte-true: write[R1=0x3e]=0x7B; INC->0x3f; A=0x01; RET. */
static void u4c_a365_a3d2_block(void) {
  P12_WR(0x3C, 0x66);
  P12_WR(0x3D, 0x66);
  P12_WR(0x3E, 0x7B);   /* a3d2 writes 0x7B at R1=0x3e (a365 left R1=0x3e) */
  /* a3d2 returns A=0x01; the caller's following a2f9/0be6 writes that 0x01 to 0x3f */
}

/* CODE:cbf8 == phy_cc10_cmd: a2ff/a344/a327/a2df + a2eb/a365/a3d2 descriptor-COMMIT engine. P12 plane.
 * The real engine is 2 rounds (a327 OR-mask 0x1 then 0x2); each round threads the a2eb/a365/a3d2
 * data blocks AND relies
 * on the embedded commit (a2eb / a2f9) leaving R1=0x3d, so the a327 AFTER each commit writes to 0x3d
 * (NOT 0x35). Disasm-traced register/value sequence (R1 tracked through the commit's R1=0x3d exit):
 *   R1=0x34: a327@0x34 (resid&0xC0|1) ; INC->0x35 ; a2df@0x35=0x41 (INC->0x36)
 *   a2eb: 0x3c=CC 0x3d=CC 0x3e=08 + commit -> R1=0x3d
 *   a327@0x3d (0x0F&0xC0|1=0x01) ; INC->0x3e ; a2df@0x3e=0x42 (INC->0x3f)
 *   a365/a3d2: 0x3c=66 0x3d=66 0x3e=7B ; a2f9 writes 0x3f=0x01 + commit -> R1=0x3d
 *   --- ROUND 2 (OR-mask 0x2) mirrors with a2eb in the MIDDLE slot ---
 *   a327@0x3d (0x02) ; INC->0x3e ; a2df@0x3e=0x41 (INC->0x3f)
 *   a2eb: 0x3c=CC 0x3d=CC 0x3e=08 + commit -> R1=0x3d
 *   a327@0x3d (0x02) ; INC->0x3e ; a2df@0x3e=0x42 (INC->0x3f)
 *   a365/a3d2: 0x3c=66 0x3d=66 0x3e=7B ; final 0be6 0x3f=0x01 + commit. */
static void u4c_cbf8_commit(void) {
  uint8_t resid;

  /* a2ff: R1=0x34 (A=residual ACC from the prior commit/engine = 0 in practice). a344=(A&0xF0)|0xF. */
  resid = 0x00;  /* a2ff leaves A=resid; callers (e5b0/d4c8 tail) enter with A=0 from the last commit */
  eng_a327(0x34, (uint8_t)(((((resid & 0xF0) | 0x0F) & 0xC0) | 0x01)));
  eng_a2df(0x35, 0x41);
  u4c_a2eb_block();                       /* 0x3c=CC 0x3d=CC 0x3e=08 + commit; R1->0x3d */

  /* a344 on A=0 (commit return) -> 0x0F; (&0xC0)|1 = 0x01; a327 @0x3d */
  eng_a327(0x3D, 0x01);
  eng_a2df(0x3E, 0x42);
  u4c_a365_a3d2_block();                   /* 0x3c=66 0x3d=66 0x3e=7B ; a3d2 A=0x01 */
  P12_WR(0x3F, 0x01);                      /* a2f9: write 0x3f=0x01 */
  (void)u4c_sb_desc_commit();              /* commit; R1->0x3d */

  /* --- ROUND 2 (OR-mask 0x2) --- */
  eng_a327(0x3D, 0x02);
  eng_a2df(0x3E, 0x41);
  u4c_a2eb_block();                        /* 0x3c=CC 0x3d=CC 0x3e=08 + commit; R1->0x3d */

  eng_a327(0x3D, 0x02);
  eng_a2df(0x3E, 0x42);
  u4c_a365_a3d2_block();                    /* 0x3c=66 0x3d=66 0x3e=7B ; a3d2 A=0x01 */
  P12_WR(0x3F, 0x01);                       /* final 0be6: write 0x3f=0x01 */
  (void)u4c_sb_desc_commit();
}

/* CRC32 table for the dcb4 -> d31e SB-TX checksum engine.
 * Byte-true copy of stock ROM CODE:0x5466 (256 entries x 4 bytes, big-endian per
 * entry relative to the 0x0AA8-MSB accumulator orientation). NON-standard poly
 * (entry[1]=0xF26B8303), so the table MUST be embedded verbatim. */
static __code const uint8_t u4c_crc32_tab_5466[256][4] = {
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

/* bank1 dcb4 leaf (CODE_BANK1::dcb4) + its LJMP 0xd31e tail (the SB-TX CRC32 engine).
 *
 * The prelude (regs) is byte-true to CODE_BANK1::dcb4. dcb4 ends `LJMP 0xd31e`, a
 * TAIL CALL that (being in bank1) lands at CODE_BANK1::d31e (NOT the bank0 d30b/CCF8
 * clock setter the address overlaps). d31e computes a reflected CRC32 over the
 * descriptor bytes XDATA[0x024D + i] for i=0..XDATA[0x024E)-1, using the 0x5466
 * table, then stores the (inverted) result LSB->MSB to XDATA[0x0249..0x024C] and
 * leaves it also at 0x0AA8..0x0AAB. There is NO R7 parameter: d31e is a 0-arg
 * tail call; the loop limit comes from XDATA[0x024E]. (The task's "R7 1/2/3 ->
 * CCFx" was the bank0 d30b mislabel; the bank1 LJMP target is this CRC engine.)
 *
 * Accumulator layout (matches stock R4:R5:R6:R7 stored at 0x0AA8..0x0AAB):
 *   0x0AA8 = MSB ... 0x0AAB = LSB. crc>>8 and (crc^data)&0xFF use the LSB(0x0AAB).
 * NOTE: 0x0AA8-0x0AAB alias sb_tx_cmd/byte0/byte1/flag and 0x0AAC/0x0AAD alias
 * sb_tx_go_param/sb_fsm_state; stock reuses these same cells as CRC scratch and
 * clobbers them here (rebuilt before the next real SB-TX). Byte-true. */
static void u4c_dcb4_transport_reg_reinit(void) {
  uint8_t idx;
  /* accumulator lives in XDATA 0x0AA8(MSB)..0x0AAB(LSB), exactly like stock R4:R5:R6:R7 */

  /* ---- dcb4 prelude (byte-true CODE_BANK1::dcb4) ---- */
  P12_WR(0x4E, 0x01);
  P12_WR(0x4F, 0x08);
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xF7));
  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xFE));
  PR(0xC809) = (uint8_t)((PR(0xC809) & 0xFD) | 0x02);
  P12_WR(0x4D, (uint8_t)(P12_RD(0x4D) & 0xBF));
  P12_WR(0x7A, (uint8_t)((P12_RD(0x7A) & 0xFE) | 0x01));
  PR(0x023F) = 0;
  PR(0x0443) = 0; PR(0x0444) = 0; PR(0x0445) = 0;
  PR(0x0448) = 0;
  PR(0x0446) = 0;
  PR(0x0449) = 0; PR(0x044A) = 0;

  /* ---- d31e CRC32 engine (LJMP 0xd31e tail) ----
   * STOCK uses XDATA[0x0AA8..0x0AAD] as the CRC accumulator(0x0AA8 MSB..0x0AAB LSB)/index(0x0AAC)/
   * limit(0x0AAD) scratch. In HANDMADE those cells alias the LIVE SB-transport state
   * (sb_tx_cmd/byte0/byte1/flag @0x0AA8-0x0AAB, sb_tx_go_param @0x0AAC, sb_fsm_state @0x0AAD) — a
   * handmade XDATA LAYOUT COLLISION. Running the stock scratch verbatim corrupts sb_fsm_state ->
   * the lane SM raises SB[0x66].2 "Abr2" instead of .0 "Lane Bonded" and the bond never finalizes.
   * The CRC scratch is INTERNAL (only the result XDATA[0x0249..0x024C] is observable), so compute it
   * in LOCAL vars: byte-true RESULT, no collision. (c0=MSB..c3=LSB == stock 0x0AA8..0x0AAB.) */
  {
    /* function-static XDATA scratch (linker-allocated free XDATA): byte-true CRC, zero IRAM, and
     * crucially NOT 0x0AA8-0x0AAD so it cannot clobber the live sb_tx/sb_fsm cells. acc[0]=MSB
     * (stock 0x0AA8) .. acc[3]=LSB (stock 0x0AAB). */
    static __xdata uint8_t acc[4];
    static __xdata uint8_t ci, climit;
    acc[0] = 0xFF; acc[1] = 0xFF; acc[2] = 0xFF; acc[3] = 0xFF;   /* CRC init 0xFFFFFFFF */
    climit = PR(0x024E);                                          /* d326: loop limit = XDATA[0x024E] */
    for (ci = 0; ci < climit; ci++) {
      idx = (uint8_t)(acc[3] ^ PR((uint16_t)(0x024D + ci)));      /* index = LSB ^ data[0x024D+ci] */
      acc[3] = acc[2]; acc[2] = acc[1]; acc[1] = acc[0]; acc[0] = 0x00;  /* crc >>= 8 (MSB-first) */
      acc[0] ^= u4c_crc32_tab_5466[idx][0];                      /* ^= tab[idx] (0=MSB..3=LSB) */
      acc[1] ^= u4c_crc32_tab_5466[idx][1];
      acc[2] ^= u4c_crc32_tab_5466[idx][2];
      acc[3] ^= u4c_crc32_tab_5466[idx][3];
    }
    PR(0x0249) = (uint8_t)~acc[3];     /* d37b final XOR + d38f..d3ae result LSB..MSB -> 0x0249..0x024C */
    PR(0x024A) = (uint8_t)~acc[2];
    PR(0x024B) = (uint8_t)~acc[1];
    PR(0x024C) = (uint8_t)~acc[0];
  }
}

/* SB[0x1C].0 = !connect. */
static void u4c_edbd(void) {
  if (u4_enter_usb_accepted == 0) SB_SET(0x1C, 0x01);
  else                 SB_CLR(0x1C, 0x01);
}

/* Engine pre-config before ccb3/c270 program descriptors. */
static void u4c_e5b0(void) {
  sb_descr_engine_scratch = 0;
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xFE));
  P12_WR(0x03, 0x80);
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xFB));
  P12_WR(0x8F, 0x80);
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xDF));
  P12_WR(0x8F, 0x20);
  sb_notify_flag0 = 0; sb_notify_flag1 = 0; sb_notify_flag2 = 0; sb_notify_flag3 = 0;
}

/* Lane-config descriptor: main descriptor + two 0x09FB-gated sub-descriptors. */
static void u4c_ccb3(uint8_t width_byte) {
  uint8_t gate;
  (void)P12_RD(0x34);
  eng_a30c(0x34, (uint8_t)((width_byte & 0xF0) | 0x0E));
  eng_a2df(0x35, 0x01);
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x01);
  P12_WR(0x3E, 0x01); P12_WR(0x3F, 0x00);
  u4c_sb_desc_commit();
  gate = u4_lane_gate_sel;
  if (!(gate & 0x01)) {
    (void)P12_RD(0x34);
    eng_a31c(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    eng_a2df(0x35, 0x02);
    u4c_sb_desc_commit();
  }
  gate = u4_lane_gate_sel;
  if (!((gate >> 1) & 0x01)) {
    (void)P12_RD(0x34);
    eng_a348(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    eng_a327(0x35, (uint8_t)((gate & 0xC0) | 0x03));
    eng_a2df(0x35, 0x02);
    u4c_sb_desc_commit();
  }
}

/* bbc7 (stock CODE:bbc7): populate the sb_eng_data* DROM descriptor cells (0x0B12-0x0B1A) the host CM
 * reads back via u4c_c270's three sub-descriptors. Stock runs this in the cap-apply tail (bank0_8d77).
 * Handmade NEVER wrote these cells -> the connect DROM advertised 0x00 where stock advertises VID 0x174C
 * (sb_eng_data_lo/hi = 0x4C/0x17) -> the host CM's descriptor read-back diverged. bc70/bcb8 = nibble-swap helpers
 * over the SPI-blob bytes 0x0A3C-0x0A41; with NO SPI blob (our case, byte-true to fw_tinygrad.bin where
 * 0x707e!=0xA5) those inputs are 0, so every nibble-derived field computes to 0 and only the hardcoded
 * VID (0x4C/0x17) and the runtime u4lb_width_rate_code (3c_b) are non-zero. Byte-true to stock. */
/* nibble accumulator (stock xdata_00a5b) + the bc70/bcb8 helpers kept in __xdata to avoid the IRAM
 * ceiling (this call tree is reachable from the connect path; SDCC can't overlay IRAM locals here). */
static volatile uint8_t __xdata bbc7_a5b;
static uint8_t bbc7_bc70(uint16_t addr) {
  uint8_t code = XDATA_REG8(addr);
  uint8_t hi = (uint8_t)(code >> 4);
  u4lb_width_rate_code = (uint8_t)(((uint8_t)(hi | (uint8_t)(code << 4))) ^ hi);
  bbc7_a5b = (uint8_t)((uint8_t)(bbc7_a5b << 4) | hi);
  return bbc7_a5b;
}
static uint8_t bbc7_bcb8(uint16_t addr) {
  return (uint8_t)(u4lb_width_rate_code | (uint8_t)(XDATA_REG8(addr) >> 4));
}
static void sb_eng_data_init_bbc7(void) {
  uint8_t i;
  /* zero the 9 SB engine descriptor bytes 0x12-0x1A (stock r3_write_dispatch loop) */
  for (i = 0; i < 9; i++) P12_WR((uint8_t)(0x12 + i), 0x00);
  /* xdata_00b12: width-class marker keyed on 0x09F7 */
  i = XDATA_REG8V(0x09F7);
  XDATA_REG8(0x0B12) = (i == 0) ? 0x0C : (i == 1) ? 0x1C : 0x00;
  sb_eng_data_hi = 0x17; sb_eng_data_lo = 0x4C;            /* TBT VID 0x174C */
  bbc7_a5b = XDATA_REG8(0x0A41);
  sb_eng_data3d_a = bbc7_bc70(0x0A40);
  sb_eng_data3c_a = bbc7_bcb8(0x0A3F);
  bbc7_a5b = 0;
  sb_eng_data3f_b = bbc7_bc70(0x0A3E);
  sb_eng_data3e_b = bbc7_bcb8(0x0A3D);
  bbc7_a5b = 0;
  sb_eng_data3d_b = bbc7_bc70(0x0A3C);
  sb_eng_data3c_b = u4lb_width_rate_code;
}

/* DROM PID descriptors: three descriptors the host CM reads back. */
static void u4c_c270(uint8_t width_byte) {
  (void)P12_RD(0x34);
  eng_a308(0x34, width_byte);
  eng_a2df(0x35, 0x00);
  P12_WR(0x3C, sb_eng_data_lo);
  P12_WR(0x3D, sb_eng_data_hi);
  P12_WR(0x3E, pd_product_pid_lo);
  P12_WR(0x3F, pd_product_pid_hi);
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);
  eng_a308(0x34, pd_product_pid_hi);
  eng_a2df(0x35, 0x07);
  P12_WR(0x3C, sb_eng_data3c_a);
  P12_WR(0x3D, sb_eng_data3d_a);
  P12_WR(0x3E, sb_eng_data_lo);
  P12_WR(0x3F, sb_eng_data_hi);
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);
  eng_a308(0x34, sb_eng_data_hi);
  eng_a2df(0x35, 0x08);
  P12_WR(0x3C, sb_eng_data3c_b);
  P12_WR(0x3D, sb_eng_data3d_b);
  P12_WR(0x3E, sb_eng_data3e_b);
  P12_WR(0x3F, sb_eng_data3f_b);
  u4c_sb_desc_commit();
}

/* Per-route descriptor latch. */
static void u4c_d556(void) {
  if (u4_route_mode & 0x02) {
    PR(0x0250) = 0x02; PR(0x0251) = 0xC3;
  }
  if (u4_mode_flag & 0x80) {
    PR(0x0247) = sb_eng_data_lo; PR(0x0248) = sb_eng_data_hi;
  }
}

/* Tunnel/lane-rate train path: width ramp + bounded PHY-lock wait + E710 rate latch + link-up. */
static void u4c_bcd7_tail(void) {
  if (u4_route_mode & 0x81) {
    REG_CPU_MODE_NEXT &= 0xEF;
    sb_pcie_width_ramp(0x0F);
    if (u4_connect_gate & 0x10) {
      uint16_t g = 0;
      while (((REG_UART_TFBF & 0x1F) != 0x10) && ++g < 0x0800);
      g = 0;
      while (((REG_UART_STATUS & 0x07) != 0x00) && ++g < 0x0800);
      REG_CPU_MODE &= 0xFE;
      REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    }
    REG_TUNNEL_LINK_STATE = (REG_TUNNEL_LINK_STATE & 0xFE) | 0x01;
  }
}

/* Set once sb_assert has run so the super-loop can switch to the post-SB E302-poll diagnostic. */
static volatile uint8_t __xdata __at(0x0B4B) sb_asserted;

/* SB-assert entry: lane-flip init FIRST, then block init, then post-SB diagnostics. */
static void sb_assert(void) {
  u4c_edbd();
  P1_SET(0x0000, 0x02);
  u4c_e5b0();
  boot_phy_dd42((u4_route_mode & 0x02) ? 2 : 1);

  u4c_bcd7_tail();

  { uint8_t route_flags = u4_route_mode;
    if (route_flags & 0x02) {
      PR(0x924C) = (PR(0x924C) & 0xF7) | 0x08;
    }
  }

  u4_lane_gate_sel = 0x02;
  u4c_ccb3(0x81);
  sb_eng_data_init_bbc7();   /* stock cap-apply tail: seed sb_eng_data* (VID 0x174C etc.) BEFORE the
                              * c270 DROM build so the host CM reads back stock-true descriptors */
  u4c_c270(0x81);
  u4c_d556();
  pd_cm_dispatch_sel = 0;
  sb_lane_flip_init();
  sb_block_init();
  sb_asserted = 1;
  uart_puts("[SBdone]\n");
}


/*=== SB Router / Event Handling ===*/
/*
 * USB4 SB-router: connect/disconnect, lane-bond, and the SB-transport descriptor engine.
 */

static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13);
/* fwd: bank0_8a89 (the U2/U3/U4 lane-MODE engine, defined in usb4.h) — called inline at
 * bond-complete with desc0=2 (U4 branch), matching stock e52d tail @CODE_BANK1::e56b. */
static void bank0_8a89(uint8_t mode);
/* fwd: e52d transport-up sub-fns defined in usb4_lanebond.h / main.c (both included AFTER this
 * header). The FULL byte-true e52d in sb_lane_bond_complete_tunnel_up runs these in stock order. */
static void u4lb_b031_transport_reinit(uint8_t skip_lane);  /* bank0 b031 (usb4_lanebond.h) */
static void boot_phy_bceb_set0(uint16_t addr);    /* usb4.h (included after this header) */
static void sb_channel_connect_service(void);     /* defined below; used by u4lb_e96c (e52d) */
static void u4lb_eb62(uint8_t state_lo, u4_fsm_state_t state);
static void u4lb_98ec(uint8_t hi, uint8_t lo);
static void u4lb_d5da(uint8_t param);

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
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
}

/* Per-descriptor-type work-buffer base offset (ROM 0x21A1). 0xFF = invalid type. */
static __code const uint8_t sb_af38_rom21a1[0x13] = {
  0x00,0x04,0xFF,0x08,0x0C,0xFF,0xFF,0xFF, 0x10,0x14,0x18,0xFF, 0x19,0x1C,0xFF,0x20, 0xFF,0xFF,0x24
};

static volatile uint8_t __xdata __at(0x0B44) pcie_ctrl_b402_bit1_save;

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
    /* c2e6(...,0): last arg 0 -> only pcie_link_up_check_b432_e765; result discarded. */
    (void)((REG_POWER_CTRL_B432 & 0x07) == 0x07 && (REG_SYS_CTRL_E765 & 0x02) != 0);
  }
}

/* c35b (bank0 CODE:c35b-c3cd): the REAL stock sb_channel_connect_service -- the SB PHY/GPIO connect-
 * config writer (C620/C655/C65A/C623), gated on the connect-service state 0x0B30 / 0x0B31 / 0x0AE4.
 * Stock's bank0_de16 (= u4lb_e96c below) calls THIS at boot.  NAMING NOTE: handmade's existing
 * `sb_channel_connect_service` (defined further down) is actually a port of stock's c3b2 SB-descriptor
 * DISPATCHER, NOT this c35b writer -- u4lb_e96c previously called that wrong function, so C620/C655/C65A
 * were never programmed at boot. e730(addr) = `*addr=(*addr&0xE0)|5`. */
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

/* d7cd (bank0 CODE:d7cd-d80f): steady-state sibling of the boot-time e96c call. Stock re-programs
 * the SB PHY/GPIO connect path from the live B481 lane count and CD31 timer:
 *   if ((CA81 & 1) != 1) {
 *     r7 = B481;  B481 = 0xFF;                         (latch + clear lane-count)
 *     if (u4_reinit_pending(0x0B2F) == 0) {
 *       if ((r7 & 3) != 0) { e726(); 0x0B2F = (r7&3) - 1; }   (NOTE: writes 0x0B2F = lanecount-1)
 *       if ((CD31&1)==0 || (CD31>>1&1)==1) 0x0B30 = 1; else 0x0B30 = 2;
 *     } else 0x0B30 = 0;
 *     c35b();                                          (= sb_phy_connect_config_c35b: C620/C655/C65A)
 *   } */
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

/* e52d: lane-bond complete -> DROM/lane descriptor reload and lane-mode tail. */
static volatile uint8_t __xdata __at(0x0B57) tup_e52d_done;
static void sb_lane_bond_complete_tunnel_up(void) {
  if (tup_e52d_done) return;
  tup_e52d_done = 1;

  sb_rom_descriptor_load();                          /* [b7a4] sb_lane_descriptor_loader (DROM/lane re-seed) */
  REG_CPU_CTRL_CA60 &= 0xF7;
  /* stock e52d tail @CODE_BANK1::e56b: `8a89(0, desc0=2)` (U4 lane-MODE branch) gated XDATA[0x0AF1].0. */
  if (XDATA_REG8V(0x0AF1) & 0x01) {
    bank0_8a89(2);                                  /* U4 branch (desc0=2) */
  }
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

/* a5d8: device->host router-op responder. */
static void sb_a5d8_pend_int(void) {
  static __xdata uint8_t opcode_byte, opcode, i, op_idx, len;

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
  SB_WR(0x06, 0x01);                                   /* cea4 9934 + cea7 0be6 r3_write_dispatch: SB-transport TX */
}

/* a066: INT1 source C80A.5 service body. PART 1 = per-channel connect poll; PART 2 =
 * connect/disconnect edge + lane CL0/event servicing (all W1C). */
static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

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

  cs = SB_RD(0x2D);
  if (!(cs & 0x01)) {
    if (SB_RD(0x2C) & 0x01) {
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
}

/* cb10: per-super-loop SB lane-bond advance. */
static void sb_cb10_lane_advance(void) {
  uint8_t nibble, lat;
  nibble = SB_RD(0xA0) & 0x0F;
  lat = lb_laneA_cl_latch;
  if (nibble != lat) {
    lb_laneA_cl_latch = nibble;
  }
  nibble = SB_RD(0xA1) & 0x0F;
  lat = lb_laneB_cl_latch;
  if (nibble != lat) {
    lb_laneB_cl_latch = nibble;
  }
}

#endif /* SB_H */
