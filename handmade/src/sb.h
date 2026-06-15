#ifndef SB_H
#define SB_H
/*
 * USB4 sideband-assert keystone — transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) bank1 tail of usb4_connect_u4 @0xA3F5. Ghidra body offset == fw_tinygrad
 * offset; bank1 functions live in the CODE_BANK1 overlay (b230 sb_lane_flip_init, bb37
 * sb_block_init, b7a4 ROM-descriptor loader, e0d9 PHY descriptor seed, e34b PHY lane cfg).
 *
 * Running sb_lane_flip_init (b230) then sb_block_init (bb37) to completion gives the host an SB
 * block to train against, so the upstream USB4 PHY sets E302 ((E302>>4)&3 -> 2/3). The firmware
 * never writes E302 — it is a consequence of asserting the sideband.
 *
 * Included AFTER usb4.h forward-declares these (so PR(a)=XDATA_REG8V(a), uart_*, DPX SFR scope).
 *
 * SB / page-1 access model (equivalent of the stock R3=2/R2=0x28/R1=off paged accessor
 * 0x0ac1/0x0adc which sets DPX=R3-1=1): every SB[off] access is page-1 XDATA at 0x2800+off, i.e.
 *   read : DPX=1; v = XDATA_REG8V(0x2800+off); DPX=0;
 *   write: DPX=1; XDATA_REG8V(0x2800+off) = v; DPX=0;
 * P1[off] is page-1 XDATA at 0x0000+off (the 0x010000 plane). RMW = read(DPX=1), modify,
 * write(DPX=1), then DPX=0. int1_isr forces DPX=0 on entry and restores it on exit, and this
 * runs synchronously inside that ISR window, so local DPX=1->0 toggling per access is safe.
 */

/* boot_phy.h is included AFTER sb.h/usb4.h; forward-declare the dd42 PHY-descriptor latch the
 * usb4_connect_u4 tail uses (definition in boot_phy.h, byte-identical to stock dd42 @0xdd42). */
static void boot_phy_dd42(uint8_t param);

/* ---- page-1 / sideband accessors (DPX=1 window, restore DPX=0) ---- */
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

/* SB descriptor-ENGINE plane: page-1 0x011200 + off (stock cursor a2ff latches R2=0x12). This is
 * the control/commit plane the HW arms SB[0x2C] (channel-READY) from — NOT the 0x2800 SB readback
 * plane. Stock a480 -> ccb3/c270 program descriptors here; committing the engine
 * (ENGINE[0x37].7=1 + ENGINE[0x38] kick/poll) arms SB[0x2C].4-7. */
#define P12_RD(off)     P1_REG8_rd((uint16_t)(0x1200u + (off)))
#define P12_WR(off, v)  P1_REG8_wr((uint16_t)(0x1200u + (off)), (uint8_t)(v))

/* ROM table CODE 0x21B4[0x10] — the C8FF==4 lane-rate SB descriptor (b230 tail). byte-exact. */
static __code const uint8_t sb_flip_rom_21b4[0x10] = {
  0x00,0x06,0x0B,0x0E,0x13,0x00,0x05,0x0A, 0x0E,0x11,0x00,0x05,0x08,0x0D,0x00,0x04
};

/* ============================ sb_lane_flip_init (bank1 b230) ============================
 * Runs FIRST. Reads cable orientation C6DB.0 (flip), clears P1[0x0100] bits 4/6/7, sets/clears
 * P1[0x0100] bit0 by connect state, programs the orientation-dependent SB lane map, arms/clears
 * the E7FC lane-bond gate by connect state, then SB[0xd1].4=1, SB[0x49]=0xA0, XDATA(0x06EC)=0.
 * Then the b307-b39e tail (SB-PHY-RX descriptor + C801.4/C809.3 unmask + CCD mailbox strobe). */
static void sb_lane_flip_init(void) {
  uint8_t flip = REG_PHY_VENDOR_CTRL_C6DB & 0x01;
  uart_puts("[flp=");
  uart_puthex(flip);
  uart_puts("]");

  /* page-1 0x010100: clear bits 4,6,7 */
  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);

  /* connect -> clear bit0; no-connect -> set bit0 */
  if (PR(0x07BA) != 0 || PR(0x07B9) != 0) P1_CLR(0x0100, 0x01);
  else                                    P1_SET(0x0100, 0x01);

  /* orientation-dependent lane map on the P1[0x0101]/[0x0102] plane (bank1 b230 @0x131ec, base
   * 0xFF6F; r3 paged accessor = DPX=1 at XDATA 0x0101/0x0102). flip(C6DB.0=1) -> set b0,b1;
   * straight -> clear b0,b1. */
  if (flip) {                 /* C6DB.0 == 1 */
    P1_SET(0x0102, 0x03);     /* set b0,b1 */
    P1_SET(0x0101, 0x03);     /* set b0,b1 */
  } else {                    /* straight */
    P1_CLR(0x0102, 0x03);     /* clear b0,b1 */
    P1_CLR(0x0101, 0x03);     /* clear b0,b1 */
  }

  /* connect-state lane-bond map on P1[0x0101] (b22f/b247). connect: set b4, clr b5, set b7 +
   * E7FC &= ~3; no-connect: clr b4, clr b7 + E7FC |= 3. */
  if (PR(0x07BA) != 0 || PR(0x07B9) != 0) {
    P1_WR(0x0101, (P1_RD(0x0101) & 0xEF) | 0x10);   /* 0x9685+0x96b7: set b4, clr b5 */
    P1_CLR(0x0101, 0x20);                            /* 0x96b7: clr b5 */
    P1_SET(0x0101, 0x80);                            /* 0x980d: set b7 */
    REG_LINK_MODE_CTRL &= ~0x03;                /* connect -> clear bond gate */
  } else {
    P1_CLR(0x0101, 0x10);                            /* 0x96b2: clr b4 */
    P1_CLR(0x0101, 0x80);                            /* clr b7 */
    REG_LINK_MODE_CTRL |= 0x03;                 /* no connect -> set bond gate */
  }

  SB_WR(0xD1, (SB_RD(0xD1) & 0xEF) | 0x10);     /* b4=1 */
  SB_WR(0x49, 0xA0);
  PR(0x06EC) = 0;

  /* ---- b307-b39e tail: SB-PHY-RX descriptor + C801.4/C809.3 SB-PHY-RX unmask +
   * CCD8/CCDA/CCDB=200/CCD9 mailbox strobe. b230 writes SB 0x94..0x99 here. ---- */
  /* SB 0x94 PHY-RX descriptor (0d59 -> stock SB[0x94-0x96] = 02 71 00) */
  SB_WR(0x94, 0x02); SB_WR(0x95, 0x71); SB_WR(0x96, 0x00);
  /* SB 0x98/0x99 PHY-RX descriptor (0c7a A=0x3e B=0x80 -> stock SB[0x98]=3E SB[0x99]=80) */
  SB_WR(0x98, 0x3E); SB_WR(0x99, 0x80);
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;          /* 97ef: CCD9 mailbox strobe (4 then 2) */
  REG_XFER2_DMA_CTRL &= 0xEF;                /* CCD8 &= ~0x10 */
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;       /* C801.4 SET (SB-PHY RX unmask) */
  REG_XFER2_DMA_CTRL = (REG_XFER2_DMA_CTRL & 0xF8) | 0x04;       /* CCD8 bits2:0 = 4 */
  REG_XFER2_DMA_ADDR_LO = 0x00;
  REG_XFER2_DMA_ADDR_HI = 200;                              /* 0xC8 */
  SB_CLR(0xCF, 0x01);                            /* 98b7(0xCF): SB[0xCF] &= ~1 */
  SB_WR(0x53, 0xFF);                             /* SB[0x53]=0xFF */
  SB_WR(0x5D, 0xFF);                             /* SB[0x5D]=0xFF */
  SB_CLR(0x27, 0x01);                            /* 98b7(0x27): SB[0x27] &= ~1 */
  SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);      /* 9958(0x2D)+97fc: SB[0x2D] b1 set */
  SB_CLR(0x2C, 0x01);                            /* 9945(0x2C): SB[0x2C] &= ~1 */
  REG_INT_CTRL = (REG_INT_CTRL & 0xF7) | 0x08;       /* C809.3 SET (SB-PHY RX/INT unmask) */
  SB_CLR(0x67, 0x40);                            /* SB[0x67] &= ~0x40 */
  PR(0x072B) = 0x07; PR(0x072C) = 0x07;          /* lane-state latches */
  /* if C8FF==4 (lane-rate gate): ROM-copy CODE 0x21b4[0x10] -> plain XDATA 0x073E..0x074D (DPX=0).
   * Destination is CONCAT11(0x07, 0x3E+i), NOT SB[0x3E] (page1 0x283E). */
  if (REG_LANE_RATE_C8FF == 0x04) {
    uint8_t k;
    for (k = 0; k < 0x10; k++) PR((uint16_t)(0x073E + k)) = sb_flip_rom_21b4[k];
  }
}

/* ============================ ROM descriptor loader (bank1 b7a4) ============================
 * Copies the router/DROM identity ROM tables into staging XDATA, then latches a few PID/mode
 * bytes. The tables are DATA the host CM reads back, so they are copied byte-exact:
 *   CODE 0x213d[0x64] -> XDATA 0x0800..0x0863 (VID 0x174C, PID 0x2464, 'APP EM  ', D3 03 @0x1A)
 *   CODE 0x21d4[0x10] -> XDATA 0x07xx lane descriptor
 * Both live in common code space (ghidra offset == file offset == handmade __code addr). */
static __code const uint8_t sb_drom_213d[0x64] = {
  0x4C,0x17,0x00,0x00,0x64,0x24,0x00,0x00, 0x41,0x50,0x50,0x20,0x45,0x4D,0x20,0x20,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0xD3,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};
static __code const uint8_t sb_lane_desc_21d4[0x10] = {
  0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0C,0x0C, 0x0C,0x0C,0x0C,0x07,0x07,0x07,0x07,0x07
};
/* e391 width-LUT seed tables (CODE 0x514c / 0x515f, read_memory byte-exact). STOCK CAPTURE (2026-06-15)
 * proved e391 runs ONCE at COLD BOOT (before UART; gated 0x0776==0 which only holds at first boot before
 * the loader sets it to 1) seeding XDATA[0x06F2..0x0704]; the values persist into state-3 (06FE=0x03,
 * 06FF=0x04). af38 ORs PR(0x06F2+dat50) into SBTX[1] (the connect descriptor's sub-byte the host validates).
 * The handmade's gated e391 (usb4_lanebond.h) runs in the super-loop AFTER the a066-ISR af38, so the
 * state-3 af38 read the UNSEEDED 0x55 -> SBTX[1]=0x55 (vs stock 0x03). Seed the LUT HERE at SB init. */
static __code const uint8_t sb_width_lut_514c[0x13] = {
  0x04,0x04,0x00,0x04,0x04,0x00,0x00,0x00, 0x04,0x04,0x01,0x00, 0x03,0x04,0x00,0x04, 0x00,0x00,0x10
};
static __code const uint8_t sb_branchA_gate_515f[0x13] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x01
};

static void sb_rom_descriptor_load(void) {
  uint8_t i;
  for (i = 0; i < 0x64; i++) PR(0x0800 + i) = sb_drom_213d[i];
  /* latch PID/bcd from PD RAM into the DROM product fields */
  PR(0x0804) = PR(0x0A57);
  PR(0x0805) = PR(0x0A58);
  /* if not USB4-cap (0x09F9.7 clear): clear DROM[0x1B].1 */
  if (!(PR(0x09F9) & 0x80)) PR(0x081B) &= ~0x02;
  /* PHASE-DIFF FIX (byte-true, self-verified): stock b7e4 `MOV A,#0x1a; ADD A,idx; LCALL 99d8` + 99d8
   * sets DPH=0x07 -> dest = 0x0700+(0x1A+i) = 0x071A+i, NOT 0x0700+i. The old base left 0x071A..0x0729
   * (the 81d4 walk-value source work[0x071A+r7]) zero AND clobbered 0x0700..0x070F. */
  for (i = 0; i < 0x10; i++) PR(0x071A + i) = sb_lane_desc_21d4[i];
  /* STOCK-CAPTURE FIX: seed the af38 width LUT (0x06F2+i) + BRANCH-A presence gate (0x0705+i) at SB init,
   * mirroring stock's cold-boot e391 (the only 0x06FE/0x0705 writer). Without this the state-3 af38 SBTX[1]
   * for the 0x0C connect descriptor was the uninit 0x55 instead of 0x03, so the host never escalated 05->63. */
  for (i = 0; i < 0x13; i++) {
    PR((uint16_t)(0x06F2u + i)) = sb_width_lut_514c[i];
    PR((uint16_t)(0x0705u + i)) = sb_branchA_gate_515f[i];
  }
  /* b7a4 0x081A.5 (20G lane-present bit): if 0x09F5 set, SET bit5.
   * NOTE: stock also has a conditional clear-back here (clear bit5 if
   *   (0x07BA!=0 && 0x07CC<3) || (0x07B9!=0 && 0x07CF in {1,2})) that is OMITTED until 0x07CC is
   * faithfully initialized — it currently fires non-deterministically and destabilizes 20G connect.
   * TODO(0x07CC): RE the stock writers of 0x07CC/0x07BA before b7a4, then restore the clear-back. */
  if (PR(0x09F5)) PR(0x081A) = (uint8_t)((PR(0x081A) & 0xDF) | 0x20);
  /* if 0x09F6 == 0: DROM[0x1A] &= 0xED */
  if (PR(0x09F6) == 0) PR(0x081A) &= 0xED;

  /* ---- b83b-b8d8 tail: connect-state init + the SB[0xC9] walking-1 connect-detect ARM +
   * SB[0x28]/[0x2A]/[0x2C]/[0x66] + 0x6EE/0x6EF latch + CCD9 strobe + SB[0xD4]/[0x8F]. a066 only
   * services a channel when SB[0xC9] bit(4+idx) is set, so this gates the C80A.5 SB-router service. */
  PR(0x0750) = 1; PR(0x0775) = 0; PR(0x0765) = 0; PR(0x0766) = 0; PR(0x0767) = 0;
  PR(0x06ED) = 0; PR(0x0758) = 0; PR(0x075D) = 0x0F; PR(0x075E) = 0x0F; PR(0x0776) = 1;
  PR(0x072A) = 0; PR(0x072D) = 0;
  SB_WR(0x2C, 0x04);                              /* 96f5(4): SB[0x2C]=4 */
  SB_WR(0x28, 0x08);                              /* SB[0x28]=8 */
  SB_WR(0x2A, 0x08);                              /* SB[0x2A]=8 */
  /* SB[0xC9] walking-1 connect-detect arm: 1,2,4,8,0x10,0x20,0x40,0x80 (9945/9756/994e + direct) */
  SB_WR(0xC9, 0x01); SB_WR(0xC9, 0x02);          /* 9945 */
  SB_WR(0xC9, 0x04); SB_WR(0xC9, 0x08);          /* 9756 */
  SB_WR(0xC9, 0x10); SB_WR(0xC9, 0x20);          /* 994e */
  SB_WR(0xC9, 0x40); SB_WR(0xC9, 0x80);          /* direct */
  SB_WR(0x66, 0x08); SB_WR(0x66, 0x40);          /* SB[0x66]=8 then 0x40 */
  PR(0x06EE) = SB_RD(0x24) & 0x01;               /* 0x6EE = SB[0x24].0 */
  PR(0x06EF) = SB_RD(0x80) & 0x01;               /* 0x6EF = SB[0x80].0 */
  PR(0x06F1) = (uint8_t)((SB_RD(0x24) & 0x06) >> 1);  /* PHASE-DIFF FIX: b8b7 LCALL 999e = 0x06F1 =
                                                       * (SB[0x24]&6)>>1 (the active-PORT latch). a066
                                                       * services a connect channel only when 0x06F1==idx. */
  PR(0x0719) = 0;
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;          /* 97ef: CCD9 strobe */
  PR(0x074E) = 0; PR(0x074F) = 0;                /* 9874: zero per-lane CL0 latches */
  SB_CLR(0xD4, 0x20);                            /* SB[0xD4] &= ~0x20 */
  SB_WR(0x8F, (SB_RD(0x8F) & 0xEF) | 0x10);      /* SB[0x8F] = (&0xEF)|0x10 */
}

/* ============================ sb_block_init (bank1 bb37) ============================
 * Runs SECOND. ~25 SB-block RMWs + PHY descriptor seed + ROM tables + PHY-reg RMW. This is the
 * SB transport bring-up the host CM polls (SB[0x81]=0x08, SB[0x01].6/.7, SB[0x66]=0x20,
 * SB[0x9e]=0x20, the lane map) to begin USB4 lane training. (C2DC &= 0xC0, final SB[0x66]=0x20,
 * SB[0x9e]=0x20; SB[0x15]/SB[0xC9] are NOT seeded here.) */
static void sb_block_init(void) {
  uart_puts("[SB Init]");

  /* e0d9 PHY descriptor seed: stock bb37 calls e0d9(0) (CODE_BANK1::e0d9 e11d branch) =
   * c306(0xC20E) zeroes C20E/C20F/C210 + c307(0,0xC214) zeroes C214/C215/C216; C211/C212/C217
   * are NOT written (left at default). */
  REG_PHY_RXPLL_RESET = 0; REG_PHY_CTRL_C20F = 0; PR(0xC210) = 0;   /* c306(0xC20E) */
  PR(0xC211) = 0; PR(0xC212) = 0;                  /* PHASE-DIFF FIX: e0d9 R7=0/e11d path also zeroes these */
  PR(0xC214) = 0; PR(0xC215) = 0; PR(0xC216) = 0;   /* c307(0, 0xC214) */
  PR(0xC217) = 0;                                  /* PHASE-DIFF FIX: e11d zeroes C217 too (C213 untouched) */

  /* page-1 0x010100 head: clear bits 0,4,6,7 (net &= ~0xD1) */
  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);
  P1_CLR(0x0100, 0x01);

  /* --- SB block writes (DPX=1) --- */
  /* FAITHFUL: stock bb37 @bb57 calls func_9797 (which hardcodes R1=0x2c, A=0x01 -> SB[0x2C]=0x01),
   * then bb5a INC A / bb5b LCALL 0be6 -> SB[0x2C]=0x02. So stock bb37 DOES write SB[0x2C]=0x01;0x02
   * here, BEFORE the bb5e SB[0x26] write. This is NOT spurious. (A prior session wrongly deleted it,
   * misreading func_9797 as not touching SB[0x2C].) SB[0x2C].1 reads back 0xF1 in the stock trace not
   * because stock avoids setting it, but because the HW/host auto-clears bit1 once the descriptor is
   * consumed — stock sets bit1 in THREE places (bb57, b230@b35d, a066@a0f2) yet the trace is always
   * 0xF1. The handmade 0xF3 is a SYMPTOM of the host never advancing (SB[0x18] stuck at 0x04), not the
   * cause. Do NOT delete this write. */
  SB_WR(0x2C, 0x01); SB_WR(0x2C, 0x02);          /* bb57-bb5b: func_9797(0x2C<-1) + inc + 0be6(0x2C<-2) */
  SB_WR(0x26, 0x02);
  SB_WR(0x66, 0x01);                              /* transient (overridden below) */
  SB_CLR(0x2D, 0x01); SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);  /* net b0=0, b1=1 */
  SB_CLR(0x29, 0x08);
  SB_CLR(0x2B, 0x08);
  SB_CLR(0xC8, 0x90);                             /* PHASE-DIFF FIX: stock bb80 96b2(clr bit4)+anl 0x7f = clr bits 4,7 ONLY (not 5,6) */
  SB_CLR(0x27, 0x02);
  SB_CLR(0x67, 0x81);                             /* clr bits 0,7 */
  SB_WR(0x81, 0x08);
  SB_WR(0x83, 0x08);
  SB_CLR(0x82, 0x08);
  SB_CLR(0x84, 0x08);
  SB_WR(0x9E, 0x01); SB_WR(0x9E, 0x02);          /* transient (overridden below) */
  SB_WR(0x66, 0x04); SB_WR(0x66, 0x20);          /* FINAL SB[0x66]=0x20 */
  SB_CLR(0x9F, 0x03);
  SB_CLR(0x67, 0x24);                             /* clr bits 2,5 */
  SB_WR(0x9E, 0x10); SB_WR(0x9E, 0x20);          /* FINAL SB[0x9e]=0x20 */
  SB_CLR(0x9F, 0x30);
  PR(0x06EC) = 0;

  /* b7a4 ROM descriptor loader (byte-exact ROM tables + latches) */
  sb_rom_descriptor_load();

  SB_WR(0x01, (SB_RD(0x01) & 0xBF) | 0x40);      /* set bit6 */
  SB_WR(0x01, (SB_RD(0x01) & 0x7F) | 0x80);      /* set bit7 (net bits6,7 set) */
  SB_CLR(0xC4, 0x02);
  SB_CLR(0x27, 0x14);                             /* clr bits 2,4 */

  /* --- PHY bank0 RMW (DPX=0) --- */
  REG_PHY_CONFIG &= ~0x08;
  PR(0xC2C4) = (PR(0xC2C4) & 0xBF) | 0x40;
  PR(0xC2DC) &= 0xC0;
  PR(0xC344) = (PR(0xC344) & 0xBF) | 0x40;
  PR(0xC35C) &= 0xC0;

  /* e34b PHY lane cfg (bc42). Helpers: b70d(a)=&FE;&FD;(&C3)|1C;&BF ; b796(a)=(&BF)|40 ;
   * b73b: C2C3&=7F;C343&=7F. Verbatim from bank1 e34b (e34b..e38e). */
  PR(0x0AB3) = 0; PR(0x0AB4) = 3; PR(0x0AB5) = 3; PR(0x0AB6) = 0;
  /* b70d(C2C3) */
  REG_PHY_ORIENT_C2C3 &= 0xFE; REG_PHY_ORIENT_C2C3 &= 0xFD;
  REG_PHY_ORIENT_C2C3 = (REG_PHY_ORIENT_C2C3 & 0xC3) | 0x1C; REG_PHY_ORIENT_C2C3 &= 0xBF;
  PR(0xC2CB) &= 0xFB;                 /* e35f C2CB &= ~0x04 */
  /* b70d(C343) */
  REG_VENDOR_CTRL_C343 &= 0xFE; REG_VENDOR_CTRL_C343 &= 0xFD;
  REG_VENDOR_CTRL_C343 = (REG_VENDOR_CTRL_C343 & 0xC3) | 0x1C; REG_VENDOR_CTRL_C343 &= 0xBF;
  PR(0xC34B) &= 0xFB;                 /* e36c C34B &= ~0x04 */
  PR(0xC21C) = (PR(0xC21C) & 0xBF) | 0x40;        /* e373 b796(C21C) */
  REG_PHY_LINK_CTRL_C208 &= 0xBF;                 /* e379 C208 &= ~0x40 */
  REG_PHY_ORIENT_C2C3 &= 0x7F;                 /* e380 b73b: C2C3 &= 0x7F */
  REG_VENDOR_CTRL_C343 &= 0x7F;                 /* e380 b73b: C343 &= 0x7F */
  SB_CLR(0x1D, 0x02);                             /* e387 SB[0x1D] &= ~0x02 (DPX=1) */

  /* --- bb45 tail (bc45..bc5d): SB[0xBA]/[0xBD] + 0x09F7 sub-block --- */
  SB_WR(0xBA, 0x3F);                               /* bc45: SB[0xBA]=0x3F (via 0x9838) */
  SB_WR(0xBD, 0x3F);                               /* bc4c: SB[0xBD]=0x3F (A retained = 0x3F) */
  /* bc51: if (0x09F7 < 2) lcall func_05a2 (trampoline -> bank1 0xC523 connect-state helper).
   * Not transcribed: it does NOT touch the SB-page target bytes (0xBA/0xBD set above), and 0x09F7
   * is >=2 on the happy connect path. Documented omission. */

  /* [KB] GOAL-2 KEYSTONE READBACK: immediately after the [SB Init] keystone writes, read back the SB
   * cells we just wrote (SB[0x81]=0x08, SB[0x66]=0x20, SB[0x9E]=0x20) plus SB[0x2C]/[0x2D]/[0xC9].
   * If these read the written value -> the writes LAND (real cells). If they read 0/garbage -> the
   * writes are write-only strobes (or not landing). Same readback added to stock for the discriminator. */
  uart_puts("[KB 81="); uart_puthex(SB_RD(0x81));
  uart_puts(" 66=");    uart_puthex(SB_RD(0x66));
  uart_puts(" 9E=");    uart_puthex(SB_RD(0x9E));
  uart_puts(" 2C=");    uart_puthex(SB_RD(0x2C));
  uart_puts(" 2D=");    uart_puthex(SB_RD(0x2D));
  uart_puts(" C9=");    uart_puthex(SB_RD(0xC9)); uart_putc(']');
}

/* d436: program B434 lane-ramp x4 + B436. Stock ramps B434 up to `width` (0xF) across 4 lanes. */
static void sb_pcie_width_ramp(uint8_t width) {
  PR(0xB434) = width; PR(0xB435) = width; PR(0xB436) = width; PR(0xB437) = width;
}

/* ====================================================================================
 * usb4_connect_u4 TAIL (a48c-a516) — faithful transcription. The lane/tunnel-train sub-blocks
 * the host CM needs after connect. Stock addr per block below.
 * ====================================================================================
 *
 * SB descriptor-engine: stock ccb3/c270 program the descriptor via a CURSOR-based write engine on
 * the 0x12 ENGINE plane (physical 0x01_1200), NOT the 0x2800 SB plane. The cursor a2ff
 * = {R1=0x34;R3=2;R2=0x12; read 0x011234} latches R2=0x12 so every subsequent r3 write inherits the
 * engine plane. Control prims (a30c/a308/a2df/a31c/a348/a327) RMW ENGINE[0x34..0x36]; data bytes
 * land at ENGINE[0x3C..0x3F]; COMMIT = e7fb (ENGINE[0x37].7=1 GO) + e83d (ENGINE[0x38]=1 kick, poll
 * bit0, e711 cleanup). The COMMIT arms SB[0x2C].4-7 (channel-READY). */

/* engine control primitives (verbatim CODE:a30c/a308/a2df/a31c/a348/a327; cursor offset passed in). */
static void eng_a30c(uint8_t cur, uint8_t v) {           /* [cur]=v ; [cur+1]=(rd&0x3F)|0x80 */
  P12_WR(cur, v); P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x80));
}
static void eng_a308(uint8_t cur, uint8_t v) {           /* [cur]=(v&0xF0)|0xF ; [cur+1]=(rd&0x3F)|0x80 */
  P12_WR(cur, (uint8_t)((v & 0xF0) | 0x0F));
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x80));
}
static void eng_a2df(uint8_t cur, uint8_t v) {           /* [cur]=v ; [cur+1]=rd&0xE0 */
  P12_WR(cur, v); P12_WR((uint8_t)(cur + 1), (uint8_t)(P12_RD((uint8_t)(cur + 1)) & 0xE0));
}
static void eng_a31c(uint8_t cur, uint8_t v) {           /* [cur]=v ; [cur+1]=(rd&0xC0)|4 ; [cur+1]=(rd&0x3F)|0x40 */
  P12_WR(cur, v);
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0xC0) | 0x04));
  P12_WR((uint8_t)(cur + 1), (uint8_t)((P12_RD((uint8_t)(cur + 1)) & 0x3F) | 0x40));
}
static void eng_a348(uint8_t cur, uint8_t v) {           /* [cur]=v ; dummy read [cur+1] */
  P12_WR(cur, v); (void)P12_RD((uint8_t)(cur + 1));
}
static void eng_a327(uint8_t cur, uint8_t v) {           /* [cur]=v ; [cur]=(rd&0x3F)|0x40 */
  P12_WR(cur, v); P12_WR(cur, (uint8_t)((P12_RD(cur) & 0x3F) | 0x40));
}

/* COMMIT (verbatim e7fb -> e83d -> e711): set ENGINE[0x37].7 (GO), kick ENGINE[0x38]=1, poll bit0
 * (bounded), then the e711 cleanup. This arms SB[0x2C].4-7. */
static void u4c_sb_desc_commit(void) {
  uint8_t cval; uint16_t g;
  cval = (uint8_t)((P12_RD(0x37) & 0x7F) | 0x80);    /* e7fb/a33d: read ENGINE[0x37], set bit7 */
  P12_WR(0x37, cval);                                 /* e7fb: GO */
  P12_WR(0x38, 0x01);                                 /* e83d/a38b: kick */
  for (g = 0; (P12_RD(0x38) & 0x01) && g < 0x2000; g++) { }  /* e83d/a336: poll bit0 (bounded) */
  (void)P12_RD(0x35);                                 /* e711/a301: read ENGINE[0x35] */
  P12_WR(0x35, (uint8_t)(cval & 0xC0));               /* e711: ENGINE[0x35] = cval & 0xC0 */
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x00);             /* e711/a367(0): ENGINE[0x3C]=[0x3D]=0 */
  P12_WR(0x35, 0x00); P12_WR(0x36, 0x00);             /* e711: ENGINE[0x35]=[0x36]=0 */
}

/* edbd (bank1 @CODE_BANK1::edbd): SB[0x1C] (page1 0x281C) bit0 = !connect.
 *   if (0x07BA==0) SB[0x1C] |= 1; else SB[0x1C] &= ~1. */
static void u4c_edbd(void) {
  if (PR(0x07BA) == 0) SB_SET(0x1C, 0x01);
  else                 SB_CLR(0x1C, 0x01);
}

/* e5b0 (CODE_BANK1::e5b0) — engine pre-config on the 0x12 ENGINE plane (cursor c208 =
 * {R3=2;R2=0x12;LJMP 0bc8} latches R2=0x12). Walk (verbatim disasm): ENGINE[0x4C]&=~1, [0x03]=0x80,
 * [0x90]&=~4, [0x8F]=0x80, [0x90]&=~0x20, [0x8F]=0x20. (No 0x91 write.) Sets up the engine BEFORE
 * ccb3/c270 program descriptors. */
static void u4c_e5b0(void) {
  PR(0x097A) = 0;
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xFE));   /* c208: ENGINE[0x4C] &= ~1 */
  P12_WR(0x03, 0x80);                              /* ENGINE[0x03] = 0x80 */
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xFB));    /* ENGINE[0x90] &= ~4 */
  P12_WR(0x8F, 0x80);                              /* DEC R1->0x8F: ENGINE[0x8F] = 0x80 */
  P12_WR(0x90, (uint8_t)(P12_RD(0x90) & 0xDF));    /* INC R1->0x90: ENGINE[0x90] &= ~0x20 */
  P12_WR(0x8F, 0x20);                              /* DEC R1->0x8F: ENGINE[0x8F] = 0x20 */
  PR(0x09F0) = 0; PR(0x09F1) = 0; PR(0x09F2) = 0; PR(0x09F3) = 0;
}

/* ccb3 (CODE:ccb3) lane-config descriptor, verbatim on the 0x12 ENGINE plane. p1 = a480's width byte
 * (= 0x09FA, high nibble masked). Main descriptor (always) + two 0x09FB-gated sub-descriptors. */
static void u4c_ccb3(uint8_t p1) {
  uint8_t t;
  (void)P12_RD(0x34);                              /* a2ff: cursor reset+read ENGINE[0x1234] */
  eng_a30c(0x34, (uint8_t)((p1 & 0xF0) | 0x0E));   /* a30c(p1&0xF0|0xE): ENGINE[0x34],[0x35] */
  eng_a2df(0x35, 0x01);                            /* a2df(1): ENGINE[0x35]=1,[0x36] */
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x01);          /* data: [0x3C]=0 ; a369(1): [0x3D]=1,[0x3E]=1 */
  P12_WR(0x3E, 0x01); P12_WR(0x3F, 0x00);          /* [0x3F]=0 */
  u4c_sb_desc_commit();                            /* ccd1 e7fb */
  t = PR(0x09FB);
  if (!(t & 0x01)) {                               /* ccd8 JB 0x9FB.0 -> sub-descriptor 1 */
    (void)P12_RD(0x34);                            /* a2ff */
    eng_a31c(0x34, (uint8_t)((t & 0xF0) | 0x07));  /* a31c((0x9FB&0xF0)|7): [0x34],[0x35] */
    eng_a2df(0x35, 0x02);                          /* a2df(2): [0x35]=2,[0x36] */
    u4c_sb_desc_commit();                          /* cceb e7fb */
  }
  t = PR(0x09FB);
  if (!((t >> 1) & 0x01)) {                        /* ccf2 JB 0x9FB.1 -> sub-descriptor 2 */
    (void)P12_RD(0x34);                            /* a2ff */
    eng_a348(0x34, (uint8_t)((t & 0xF0) | 0x07));  /* a348((0x9FB&0xF0)|7): [0x34], read [0x35] */
    eng_a327(0x35, (uint8_t)((t & 0xC0) | 0x03));  /* a327((0x9FB&0xC0)|3): [0x35] RMW */
    eng_a2df(0x35, 0x02);                          /* a2df(2): [0x35]=2,[0x36] */
    u4c_sb_desc_commit();                          /* cd0c e7fb */
  }
}

/* c270 (CODE:c270) DROM PID descriptors, verbatim on the 0x12 ENGINE plane. Three descriptors; each
 * = a308(width) + a2df(slot) control, four data bytes ENGINE[0x3C..0x3F], commit. 0x0A57/0x0A58 are
 * the DROM product-ID the host CM reads back. p1 = a480's width byte. */
static void u4c_c270(uint8_t p1) {
  /* --- descriptor 1 (slot 0) --- */
  (void)P12_RD(0x34);                /* a2ff */
  eng_a308(0x34, p1);                /* a308(p1) */
  eng_a2df(0x35, 0x00);              /* a2df(0) */
  P12_WR(0x3C, PR(0x0B19));          /* a3cb: ENGINE[0x3C] = XDATA[0x0B19] */
  P12_WR(0x3D, PR(0x0B1A));
  P12_WR(0x3E, PR(0x0A57));          /* PID low */
  P12_WR(0x3F, PR(0x0A58));          /* a2f8: ENGINE[0x3F] = PID high */
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);                /* a2f8 tail: re-read cursor */
  /* --- descriptor 2 (slot 7) --- */
  eng_a308(0x34, PR(0x0A58));        /* a308(0x0A58) */
  eng_a2df(0x35, 0x07);              /* a2df(7) */
  P12_WR(0x3C, PR(0x0B17));
  P12_WR(0x3D, PR(0x0B18));
  P12_WR(0x3E, PR(0x0B19));          /* a3cb */
  P12_WR(0x3F, PR(0x0B1A));          /* a2f8 */
  u4c_sb_desc_commit();
  (void)P12_RD(0x34);
  /* --- descriptor 3 (slot 8) --- */
  eng_a308(0x34, PR(0x0B1A));        /* a308(0x0B1A) */
  eng_a2df(0x35, 0x08);              /* a2df(8) */
  P12_WR(0x3C, PR(0x0B13));
  P12_WR(0x3D, PR(0x0B14));
  P12_WR(0x3E, PR(0x0B15));
  P12_WR(0x3F, PR(0x0B16));
  u4c_sb_desc_commit();              /* c2e3 e7fb */
}

/* d556 (bank1 @CODE_BANK1::d556): per-route descriptor copy.
 *   if 0x09FA.1: 0x0250=2, 0x0251=0xC3, d31e() commit;
 *   if 0x09F9.7: copy 0x0B19/0x0B1A -> 0x0247/0x0248, ed63(); if 0x09FA.7: copy 0x0250..0x0253
 *     into the staged descriptor + d31e(). Reproduced as the resolved XDATA latches (d31e/ed63 are
 *     descriptor-engine commit strobes). */
static void u4c_d556(void) {
  if (PR(0x09FA) & 0x02) {
    PR(0x0250) = 0x02; PR(0x0251) = 0xC3;
  }
  if (PR(0x09F9) & 0x80) {
    PR(0x0247) = PR(0x0B19); PR(0x0248) = PR(0x0B1A);
  }
}

/* ============================ a48c..a516 tail (bcd7 gate) ============================
 * Verbatim from usb4_connect_u4 @0xa48c-0xa516, gated on 0x09FA & 0x81 (FUN_CODE_bcd7). The
 * tunnel/lane-rate train path: width ramp + e7ae PHY-lock wait (gated 0x0AF1.4) + E710 rate latch +
 * ee82 (B430|=1 tunnel link-up). bank0_e7ae waits C006[4:0]==0x10 then C00E[2:0]==0; bounded here so
 * a never-locking PHY cannot hang the connect path. */
static void u4c_bcd7_tail(void) {
  /* bcc4: r3_read(0x1504) & 0xF3, then r3_write |= 2 — an SB[0x04]-region descriptor RMW handled by
   * the engine; the load-bearing XDATA writes follow. */
  if (PR(0x09FA) & 0x81) {              /* a48f: bcd7() != 0  (0x09FA & 0x81) */
    REG_CPU_MODE_NEXT &= 0xEF;     /* a4a6: CA06 &= ~0x10 */
    sb_pcie_width_ramp(0x0F);           /* a4ac: pcie_tunnel_link_width_config_b434_b436(0xF) = d436 */
    if (PR(0x0AF1) & 0x10) {            /* a4cd: JNB 0x0AF1.4 -> skip e7ae block */
      uint16_t g = 0;
      while (((PR(0xC006) & 0x1F) != 0x10) && ++g < 0x0800);   /* bank0_e7ae C006 lock (bounded) */
      g = 0;
      while (((PR(0xC00E) & 0x07) != 0x00) && ++g < 0x0800);   /* bank0_e7ae C00E lock (bounded) */
      REG_CPU_MODE &= 0xFE;                            /* CC30 &= ~1 */
      REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;                   /* E710 rate latch low5 = 0x1F */
    }
    PR(0xB430) = (PR(0xB430) & 0xFE) | 0x01;   /* ee82: B430 |= 1 (tunnel link-up bit) */
  }
}

/* SB-assert-done gate: set once sb_assert has run so the super-loop can switch to the post-SB
 * E302-poll diagnostic. Relocated to XDATA (IRAM headroom). */
static volatile uint8_t __xdata __at(0x0B4B) sb_asserted;

/* ============================ SB-assert entry (usb4_connect_u4 tail) ============================
 * The synchronous tail reached at 0xa51b/0xa51e after the route latch, gated on 0x07BA!=0.
 * Order: intermediate path-state -> sb_lane_flip_init (b230) FIRST -> sb_block_init (bb37). */
static void sb_assert(void) {
  /* --- usb4_connect_u4 tail (a460-a51e), faithful order from the decompile --- */
  u4c_edbd();                   /* a460: edbd  -> SB[0x1C].0 by connect */
  P1_SET(0x0000, 0x02);         /* a463: bc8f()&0xFD|2 -> page1[0x0000] |= 2 (r3_write masked) */
  u4c_e5b0();                   /* a469: e5b0  -> SB[0x4C]/[0x90]/[0x8F]/[0x91] + clr 0x09F0-F3 */
  /* a46f: dd42(uVar1) where uVar1 = (0x09FA.1)?2:1 */
  boot_phy_dd42((PR(0x09FA) & 0x02) ? 2 : 1);

  u4c_bcd7_tail();              /* a48c..a4dc: the tunnel/lane-rate train path (e7ae/E710/ee82) */

  /* a4e8..a50a: the 0x09FA.1 lane-1 descriptor + 0x924C bit3 */
  { uint8_t r9fa = PR(0x09FA);
    if (r9fa & 0x02) {
      /* if route is NOT tunnel (0x09FA & 0x81 == 0): an extra SB descriptor write (bcc4|8) */
      PR(0x924C) = (PR(0x924C) & 0xF7) | 0x08;   /* a4fe: 0x924C bit3 set */
    }
  }

  PR(0x09FB) = 0x02;            /* a480 head: 0x09FB=2 (ccb3 sub-descriptor lane-gate selector) */
  u4c_ccb3(0x81);               /* a50e: ccb3 -> ENGINE(0x12)-plane lane descriptor + commit (ARMS SB[0x2C]).
                                 * p1=0x81 = a480's 0x09FA width byte (high nibble 0x80 used by a30c). */
  u4c_c270(0x81);               /* a511: c270 -> ENGINE(0x12)-plane DROM PID descriptors + commit */
  u4c_d556();                   /* a514: d556  -> per-route descriptor latch */
  PR(0x07FF) = 0;               /* a517: 0x07FF=0 (stock clears just before b230) */
  sb_lane_flip_init();          /* a51b: b230 — FIRST */
  sb_block_init();              /* a51e: bb37 — SECOND */
  sb_asserted = 1;              /* SB-assert-done gate (see super-loop) */
  /* INT1 stays ENABLED here: the C80A.5 SB-router source is W1C-acked by sb_router_event_handler
   * (sb_router.h) and the SB-connect handshake must run in the ISR to advance the link, so do NOT
   * mask EX1. */
  /* Immediate (ISR-context) E302 marker so we capture the post-SB link mode even if the super-loop's
   * sleep() hangs on the shared CC10 timer mailbox (R-timer hazard). */
  uart_puts("[SBdone e302=");
  uart_puthex(REG_PHY_MODE_E302);
  /* SB write-landed self-readback (diagnostic): read back the keystone bits we just wrote —
   * SB[0x81]==0x08, SB[0x66]==0x20, SB[0x9E]==0x20, SB[0x01] bits6,7 set, SB[0xC9] host connect. */
  uart_puts(" sb81=");  uart_puthex(SB_RD(0x81));
  uart_puts(" sb66=");  uart_puthex(SB_RD(0x66));
  uart_puts(" sb9e=");  uart_puthex(SB_RD(0x9E));
  uart_puts(" sb01=");  uart_puthex(SB_RD(0x01));
  uart_puts(" sb2d=");  uart_puthex(SB_RD(0x2D));
  uart_puts(" sbc9=");  uart_puthex(SB_RD(0xC9));
  uart_puts(" sb2c=");  uart_puthex(SB_RD(0x2C));   /* did the commit arm SB[0x2C].4-7? */
  uart_puts("]");

  /* Full SB-page + gating-reg snapshot for the stock-vs-handmade diff. Format mirrors the stock
   * patch_sbtrace.py [SBCON:...] line: gating regs in the same order, then '|', then the full
   * 256-byte SB page-1 block SB[0x00..0xFF] (DPX=1 paged XDATA at 0x2800+off). */
  uart_puts("[HMSB:");
  uart_puthex(PR(0x0AF1)); uart_puthex(REG_INT_PCIE_NVME); uart_puthex(REG_PHY_MODE_E302);
  uart_puthex(PR(0x09F9)); uart_puthex(PR(0x09FA)); uart_puthex(REG_NVME_EVENT_STATUS);
  uart_puthex(REG_USB_PHY_CTRL_91C0); uart_puthex(REG_PHY_COMPLETION_E318);
  uart_putc('|');
  { uint16_t off; for (off = 0; off < 0x100; off++) uart_puthex(SB_RD(off)); }
  uart_puts("]\n");
}

#endif /* SB_H */
