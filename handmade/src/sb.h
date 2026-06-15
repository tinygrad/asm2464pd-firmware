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

/* Runs FIRST: orientation/connect lane map + SB-PHY-RX descriptor + mailbox strobe. */
static void sb_lane_flip_init(void) {
  uint8_t flip = REG_PHY_VENDOR_CTRL_C6DB & 0x01;
  uart_puts("[flp=");
  uart_puthex(flip);
  uart_puts("]");

  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);

  if (PR(0x07BA) != 0 || PR(0x07B9) != 0) P1_CLR(0x0100, 0x01);
  else                                    P1_SET(0x0100, 0x01);

  if (flip) {
    P1_SET(0x0102, 0x03);
    P1_SET(0x0101, 0x03);
  } else {
    P1_CLR(0x0102, 0x03);
    P1_CLR(0x0101, 0x03);
  }

  if (PR(0x07BA) != 0 || PR(0x07B9) != 0) {
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
  PR(0x06EC) = 0;

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
  PR(0x072B) = 0x07; PR(0x072C) = 0x07;
  if (REG_LANE_RATE_C8FF == 0x04) {
    uint8_t k;
    for (k = 0; k < 0x10; k++) PR((uint16_t)(0x073E + k)) = sb_lane_rate_desc[k];
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
  for (i = 0; i < 0x64; i++) PR(0x0800 + i) = drom_identity[i];
  PR(0x0804) = PR(0x0A57);
  PR(0x0805) = PR(0x0A58);
  if (!(PR(0x09F9) & 0x80)) PR(0x081B) &= ~0x02;
  for (i = 0; i < 0x10; i++) PR(0x071A + i) = lane_descriptor[i];
  for (i = 0; i < 0x13; i++) {
    PR((uint16_t)(0x06F2u + i)) = width_lut[i];
    PR((uint16_t)(0x0705u + i)) = branchA_gate[i];
  }
  if (PR(0x09F5)) PR(0x081A) = (uint8_t)((PR(0x081A) & 0xDF) | 0x20);
  if (PR(0x09F6) == 0) PR(0x081A) &= 0xED;

  PR(0x0750) = 1; PR(0x0775) = 0; PR(0x0765) = 0; PR(0x0766) = 0; PR(0x0767) = 0;
  PR(0x06ED) = 0; PR(0x0758) = 0; PR(0x075D) = 0x0F; PR(0x075E) = 0x0F; PR(0x0776) = 1;
  PR(0x072A) = 0; PR(0x072D) = 0;
  SB_WR(0x2C, 0x04);
  SB_WR(0x28, 0x08);
  SB_WR(0x2A, 0x08);
  SB_WR(0xC9, 0x01); SB_WR(0xC9, 0x02);
  SB_WR(0xC9, 0x04); SB_WR(0xC9, 0x08);
  SB_WR(0xC9, 0x10); SB_WR(0xC9, 0x20);
  SB_WR(0xC9, 0x40); SB_WR(0xC9, 0x80);
  SB_WR(0x66, 0x08); SB_WR(0x66, 0x40);
  PR(0x06EE) = SB_RD(0x24) & 0x01;
  PR(0x06EF) = SB_RD(0x80) & 0x01;
  PR(0x06F1) = (uint8_t)((SB_RD(0x24) & 0x06) >> 1);
  PR(0x0719) = 0;
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
  PR(0x074E) = 0; PR(0x074F) = 0;
  SB_CLR(0xD4, 0x20);
  SB_WR(0x8F, (SB_RD(0x8F) & 0xEF) | 0x10);
}

/* Runs SECOND: SB-block RMWs + PHY descriptor seed + ROM tables + PHY-reg config. */
static void sb_block_init(void) {
  uart_puts("[SB Init]");

  REG_PHY_RXPLL_RESET = 0; REG_PHY_CTRL_C20F = 0; PR(0xC210) = 0;
  PR(0xC211) = 0; PR(0xC212) = 0;
  PR(0xC214) = 0; PR(0xC215) = 0; PR(0xC216) = 0;
  PR(0xC217) = 0;

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
  SB_CLR(0xC8, 0x90);
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
  PR(0x06EC) = 0;

  sb_rom_descriptor_load();

  SB_WR(0x01, (SB_RD(0x01) & 0xBF) | 0x40);
  SB_WR(0x01, (SB_RD(0x01) & 0x7F) | 0x80);
  SB_CLR(0xC4, 0x02);
  SB_CLR(0x27, 0x14);

  /* PHY bank0 RMW (DPX=0) */
  REG_PHY_CONFIG &= ~0x08;
  PR(0xC2C4) = (PR(0xC2C4) & 0xBF) | 0x40;
  PR(0xC2DC) &= 0xC0;
  PR(0xC344) = (PR(0xC344) & 0xBF) | 0x40;
  PR(0xC35C) &= 0xC0;

  /* PHY lane config */
  PR(0x0AB3) = 0; PR(0x0AB4) = 3; PR(0x0AB5) = 3; PR(0x0AB6) = 0;
  REG_PHY_ORIENT_C2C3 &= 0xFE; REG_PHY_ORIENT_C2C3 &= 0xFD;
  REG_PHY_ORIENT_C2C3 = (REG_PHY_ORIENT_C2C3 & 0xC3) | 0x1C; REG_PHY_ORIENT_C2C3 &= 0xBF;
  PR(0xC2CB) &= 0xFB;
  REG_VENDOR_CTRL_C343 &= 0xFE; REG_VENDOR_CTRL_C343 &= 0xFD;
  REG_VENDOR_CTRL_C343 = (REG_VENDOR_CTRL_C343 & 0xC3) | 0x1C; REG_VENDOR_CTRL_C343 &= 0xBF;
  PR(0xC34B) &= 0xFB;
  PR(0xC21C) = (PR(0xC21C) & 0xBF) | 0x40;
  REG_PHY_LINK_CTRL_C208 &= 0xBF;
  REG_PHY_ORIENT_C2C3 &= 0x7F;
  REG_VENDOR_CTRL_C343 &= 0x7F;
  SB_CLR(0x1D, 0x02);

  SB_WR(0xBA, 0x3F);
  SB_WR(0xBD, 0x3F);

  uart_puts("[KB 81="); uart_puthex(SB_RD(0x81));
  uart_puts(" 66=");    uart_puthex(SB_RD(0x66));
  uart_puts(" 9E=");    uart_puthex(SB_RD(0x9E));
  uart_puts(" 2C=");    uart_puthex(SB_RD(0x2C));
  uart_puts(" 2D=");    uart_puthex(SB_RD(0x2D));
  uart_puts(" C9=");    uart_puthex(SB_RD(0xC9)); uart_putc(']');
}

/* Program the PCIe tunnel link width across 4 lanes. */
static void sb_pcie_width_ramp(uint8_t width) {
  PR(0xB434) = width; PR(0xB435) = width; PR(0xB436) = width; PR(0xB437) = width;
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

/* Commit the staged descriptor (GO + kick + bounded poll + cleanup); arms SB[0x2C].4-7. */
static void u4c_sb_desc_commit(void) {
  uint8_t commit; uint16_t g;
  commit = (uint8_t)((P12_RD(0x37) & 0x7F) | 0x80);
  P12_WR(0x37, commit);
  P12_WR(0x38, 0x01);
  for (g = 0; (P12_RD(0x38) & 0x01) && g < 0x2000; g++) { }
  (void)P12_RD(0x35);
  P12_WR(0x35, (uint8_t)(commit & 0xC0));
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x00);
  P12_WR(0x35, 0x00); P12_WR(0x36, 0x00);
}

/* SB[0x1C].0 = !connect. */
static void u4c_edbd(void) {
  if (PR(0x07BA) == 0) SB_SET(0x1C, 0x01);
  else                 SB_CLR(0x1C, 0x01);
}

/* Engine pre-config before ccb3/c270 program descriptors. */
static void u4c_e5b0(void) {
  PR(0x097A) = 0;
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xFE));
  P12_WR(0x03, 0x80);
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xFB));
  P12_WR(0x8F, 0x80);
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xDF));
  P12_WR(0x8F, 0x20);
  PR(0x09F0) = 0; PR(0x09F1) = 0; PR(0x09F2) = 0; PR(0x09F3) = 0;
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
  gate = PR(0x09FB);
  if (!(gate & 0x01)) {
    (void)P12_RD(0x34);
    eng_a31c(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    eng_a2df(0x35, 0x02);
    u4c_sb_desc_commit();
  }
  gate = PR(0x09FB);
  if (!((gate >> 1) & 0x01)) {
    (void)P12_RD(0x34);
    eng_a348(0x34, (uint8_t)((gate & 0xF0) | 0x07));
    eng_a327(0x35, (uint8_t)((gate & 0xC0) | 0x03));
    eng_a2df(0x35, 0x02);
    u4c_sb_desc_commit();
  }
}

/* DROM PID descriptors: three descriptors the host CM reads back. */
static void u4c_c270(uint8_t width_byte) {
  (void)P12_RD(0x34);
  eng_a308(0x34, width_byte);
  eng_a2df(0x35, 0x00);
  P12_WR(0x3C, PR(0x0B19));
  P12_WR(0x3D, PR(0x0B1A));
  P12_WR(0x3E, PR(0x0A57));
  P12_WR(0x3F, PR(0x0A58));
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);
  eng_a308(0x34, PR(0x0A58));
  eng_a2df(0x35, 0x07);
  P12_WR(0x3C, PR(0x0B17));
  P12_WR(0x3D, PR(0x0B18));
  P12_WR(0x3E, PR(0x0B19));
  P12_WR(0x3F, PR(0x0B1A));
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);
  eng_a308(0x34, PR(0x0B1A));
  eng_a2df(0x35, 0x08);
  P12_WR(0x3C, PR(0x0B13));
  P12_WR(0x3D, PR(0x0B14));
  P12_WR(0x3E, PR(0x0B15));
  P12_WR(0x3F, PR(0x0B16));
  u4c_sb_desc_commit();
}

/* Per-route descriptor latch. */
static void u4c_d556(void) {
  if (PR(0x09FA) & 0x02) {
    PR(0x0250) = 0x02; PR(0x0251) = 0xC3;
  }
  if (PR(0x09F9) & 0x80) {
    PR(0x0247) = PR(0x0B19); PR(0x0248) = PR(0x0B1A);
  }
}

/* Tunnel/lane-rate train path: width ramp + bounded PHY-lock wait + E710 rate latch + link-up. */
static void u4c_bcd7_tail(void) {
  if (PR(0x09FA) & 0x81) {
    REG_CPU_MODE_NEXT &= 0xEF;
    sb_pcie_width_ramp(0x0F);
    if (PR(0x0AF1) & 0x10) {
      uint16_t g = 0;
      while (((PR(0xC006) & 0x1F) != 0x10) && ++g < 0x0800);
      g = 0;
      while (((PR(0xC00E) & 0x07) != 0x00) && ++g < 0x0800);
      REG_CPU_MODE &= 0xFE;
      REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    }
    PR(0xB430) = (PR(0xB430) & 0xFE) | 0x01;
  }
}

/* Set once sb_assert has run so the super-loop can switch to the post-SB E302-poll diagnostic. */
static volatile uint8_t __xdata __at(0x0B4B) sb_asserted;

/* SB-assert entry: lane-flip init FIRST, then block init, then post-SB diagnostics. */
static void sb_assert(void) {
  u4c_edbd();
  P1_SET(0x0000, 0x02);
  u4c_e5b0();
  boot_phy_dd42((PR(0x09FA) & 0x02) ? 2 : 1);

  u4c_bcd7_tail();

  { uint8_t route_flags = PR(0x09FA);
    if (route_flags & 0x02) {
      PR(0x924C) = (PR(0x924C) & 0xF7) | 0x08;
    }
  }

  PR(0x09FB) = 0x02;
  u4c_ccb3(0x81);
  u4c_c270(0x81);
  u4c_d556();
  PR(0x07FF) = 0;
  sb_lane_flip_init();
  sb_block_init();
  sb_asserted = 1;
  uart_puts("[SBdone e302=");
  uart_puthex(REG_PHY_MODE_E302);
  uart_puts(" sb81=");  uart_puthex(SB_RD(0x81));
  uart_puts(" sb66=");  uart_puthex(SB_RD(0x66));
  uart_puts(" sb9e=");  uart_puthex(SB_RD(0x9E));
  uart_puts(" sb01=");  uart_puthex(SB_RD(0x01));
  uart_puts(" sb2d=");  uart_puthex(SB_RD(0x2D));
  uart_puts(" sbc9=");  uart_puthex(SB_RD(0xC9));
  uart_puts(" sb2c=");  uart_puthex(SB_RD(0x2C));
  uart_puts("]");

  uart_puts("[HMSB:");
  uart_puthex(PR(0x0AF1)); uart_puthex(REG_INT_PCIE_NVME); uart_puthex(REG_PHY_MODE_E302);
  uart_puthex(PR(0x09F9)); uart_puthex(PR(0x09FA)); uart_puthex(REG_NVME_EVENT_STATUS);
  uart_puthex(REG_USB_PHY_CTRL_91C0); uart_puthex(REG_PHY_COMPLETION_E318);
  uart_putc('|');
  { uint16_t off; for (off = 0; off < 0x100; off++) uart_puthex(SB_RD(off)); }
  uart_puts("]\n");
}

#endif /* SB_H */
