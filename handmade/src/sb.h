#ifndef SB_H
#define SB_H
/*
 * USB4 SIDEBAND-assert keystone — faithful transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) bank1 tail of usb4_connect_u4 @0xA3F5. Ghidra body offset == fw_tinygrad
 * offset; bank1 functions live in the CODE_BANK1 overlay (b230 sb_lane_flip_init, bb37
 * sb_block_init, b7a4 ROM-descriptor loader, e0d9 PHY descriptor seed, e34b PHY lane cfg).
 *
 * Implements handmade/USB4_TUNNEL_PLAN.md §2 (verbatim sideband assert) + §3 SB-INIT phase
 * (the a48c..a516 intermediate bank1 path-state RMWs). THIS IS THE M1 KEYSTONE: running
 * sb_lane_flip_init (b230) then sb_block_init (bb37) to completion is what gives the host an SB
 * block to train against, so the upstream USB4 PHY sets E302 ((E302>>4)&3 -> 2/3). The firmware
 * NEVER writes E302 — it is a consequence of asserting the sideband, not a precondition.
 *
 * Included AFTER usb4.h forward-declares these (so PR(a)=XDATA_REG8V(a), uart_*, DPX SFR scope).
 *
 * SB / page-1 access model (proven equivalent of the stock R3=2/R2=0x28/R1=off paged accessor
 * 0x0ac1/0x0adc which sets DPX=R3-1=1): every SB[off] access is page-1 XDATA at 0x2800+off, i.e.
 *   read : DPX=1; v = XDATA_REG8V(0x2800+off); DPX=0;
 *   write: DPX=1; XDATA_REG8V(0x2800+off) = v; DPX=0;
 * P1[off] is page-1 XDATA at 0x0000+off (the 0x010000 plane). RMW = read(DPX=1), modify,
 * write(DPX=1), then DPX=0. The int1_isr forces DPX=0 on entry and restores it on exit, and this
 * runs synchronously inside that ISR window, so local DPX=1->0 toggling per access is safe.
 */

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
/* page-1 RMW helpers (clear/set/masked-set) on the 0x010000 plane */
#define P1_CLR(off, m)  P1_WR((off), P1_RD(off) & (uint8_t)~(m))
#define P1_SET(off, m)  P1_WR((off), P1_RD(off) | (uint8_t)(m))

/* SB block lives at page-1 0x012800 + off. */
#define SB_RD(off)      P1_REG8_rd((uint16_t)(0x2800u + (off)))
#define SB_WR(off, v)   P1_REG8_wr((uint16_t)(0x2800u + (off)), (uint8_t)(v))
#define SB_CLR(off, m)  SB_WR((off), SB_RD(off) & (uint8_t)~(m))
#define SB_SET(off, m)  SB_WR((off), SB_RD(off) | (uint8_t)(m))

/* ROM table CODE 0x21B4[0x10] — the C8FF==4 lane-rate SB[0x3e..0x4d] descriptor (b230 tail).
 * read_memory byte-exact. */
static __code const uint8_t sb_flip_rom_21b4[0x10] = {
  0x00,0x06,0x0B,0x0E,0x13,0x00,0x05,0x0A, 0x0E,0x11,0x00,0x05,0x08,0x0D,0x00,0x04
};

/* ============================ sb_lane_flip_init (bank1 b230) ============================
 * Runs FIRST. Reads cable orientation C6DB.0 (flip), prints [flp=N], clears P1[0x0100] bits
 * 4/6/7, sets/clears P1[0x0100] bit0 by connect state, programs the orientation-dependent SB
 * lane map (SB[0x01]/SB[0x02]), arms/clears the E7FC lane-bond gate by connect state, then
 * SB[0xd1].4=1, SB[0x49]=0xA0, XDATA(0x06EC)=0. Then the b307-b39e tail (SB-PHY-RX descriptor +
 * C801.4/C809.3 UNMASK + CCD mailbox strobe). Verbatim per §2 + RE-AUDIT breakthrough A. */
static void sb_lane_flip_init(void) {
  uint8_t flip = PR(0xC6DB) & 0x01;
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

  /* orientation-dependent lane map. AUDIT FIX I11 (byte-exact from bank1 b230 @0x131ec, base
   * 0xFF6F): stock writes the orientation map on the P1[0x0101]/[0x0102] plane (R3=2,R2=1,R1={1,2}
   * via the r3_xdata paged accessor = DPX=1 at XDATA 0x0101/0x0102), NOT SB[0x01]/[0x02] (which is
   * DPX=1 at 0x2801/0x2802). The prior handmade code targeted the WRONG plane -> the SB-PHY RX was
   * enabled on the wrong SBU pins and never locked. Sense (verified via helpers 0x9958=set b0,
   * 0x97fc=set b1, 0x96c7=write+reread): flip(C6DB.0=1) -> set b0,b1; straight -> clear b0,b1. */
  if (flip) {                 /* C6DB.0 == 1 */
    P1_SET(0x0102, 0x03);     /* set b0,b1 */
    P1_SET(0x0101, 0x03);     /* set b0,b1 */
  } else {                    /* straight */
    P1_CLR(0x0102, 0x03);     /* clear b0,b1 */
    P1_CLR(0x0101, 0x03);     /* clear b0,b1 */
  }

  /* connect-state lane-bond map on P1[0x0101] (b22f/b247). connect: set b4, clr b5, set b7 +
   * E7FC &= ~3; no-connect: clr b4, clr b7 + E7FC |= 3. (Was wrongly on SB[0x01].) */
  if (PR(0x07BA) != 0 || PR(0x07B9) != 0) {
    P1_WR(0x0101, (P1_RD(0x0101) & 0xEF) | 0x10);   /* 0x9685+0x96b7: set b4, clr b5 */
    P1_CLR(0x0101, 0x20);                            /* 0x96b7: clr b5 */
    P1_SET(0x0101, 0x80);                            /* 0x980d: set b7 */
    PR(0xE7FC) = PR(0xE7FC) & ~0x03;                /* connect -> clear bond gate */
  } else {
    P1_CLR(0x0101, 0x10);                            /* 0x96b2: clr b4 */
    P1_CLR(0x0101, 0x80);                            /* clr b7 */
    PR(0xE7FC) = PR(0xE7FC) | 0x03;                 /* no connect -> set bond gate */
  }

  SB_WR(0xD1, (SB_RD(0xD1) & 0xEF) | 0x10);     /* b4=1 */
  SB_WR(0x49, 0xA0);
  PR(0x06EC) = 0;

  /* ---- b307-b39e tail (RE-AUDIT #2/#5 + breakthrough A): the dropped SB-PHY-RX descriptor +
   * the C801.4 / C809.3 SB-PHY-RX UNMASK + the CCD8/CCDA/CCDB=200/CCD9 mailbox strobe. handmade
   * dropped this whole tail -> the SB-PHY RX interrupt SOURCE stayed MASKED -> the host could drive
   * all the sideband it wanted and C80A.5 could never fire (matches c80aACC=00 every run). The old
   * usb4_irq.h comment claiming SB 0x94..0x99 are "HW-latched" is WRONG: b230 writes them here. */
  /* SB 0x94 PHY-RX descriptor (0d59 -> stock SB[0x94-0x96] = 02 71 00) */
  SB_WR(0x94, 0x02); SB_WR(0x95, 0x71); SB_WR(0x96, 0x00);
  /* SB 0x98/0x99 PHY-RX descriptor (0c7a A=0x3e B=0x80 -> stock SB[0x98]=3E SB[0x99]=80) */
  SB_WR(0x98, 0x3E); SB_WR(0x99, 0x80);
  PR(0xCCD9) = 0x04; PR(0xCCD9) = 0x02;          /* 97ef: CCD9 mailbox strobe (4 then 2) */
  PR(0xCCD8) = PR(0xCCD8) & 0xEF;                /* CCD8 &= ~0x10 */
  PR(0xC801) = (PR(0xC801) & 0xEF) | 0x10;       /* *** C801.4 SET (SB-PHY RX unmask) *** */
  PR(0xCCD8) = (PR(0xCCD8) & 0xF8) | 0x04;       /* CCD8 bits2:0 = 4 */
  PR(0xCCDA) = 0x00;
  PR(0xCCDB) = 200;                              /* 0xC8 */
  SB_CLR(0xCF, 0x01);                            /* 98b7(0xCF): SB[0xCF] &= ~1 */
  SB_WR(0x53, 0xFF);                             /* SB[0x53]=0xFF */
  SB_WR(0x5D, 0xFF);                             /* SB[0x5D]=0xFF */
  SB_CLR(0x27, 0x01);                            /* 98b7(0x27): SB[0x27] &= ~1 */
  SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);      /* 9958(0x2D)+97fc: SB[0x2D] b1 set */
  SB_CLR(0x2C, 0x01);                            /* 9945(0x2C): SB[0x2C] &= ~1 */
  PR(0xC809) = (PR(0xC809) & 0xF7) | 0x08;       /* *** C809.3 SET (SB-PHY RX/INT unmask) *** */
  SB_CLR(0x67, 0x40);                            /* SB[0x67] &= ~0x40 */
  PR(0x072B) = 0x07; PR(0x072C) = 0x07;          /* lane-state latches */
  /* if C8FF==4 (lane-rate gate): ROM-copy CODE 0x21b4[0x10] -> SB[0x3e..0x4d] */
  if (PR(0xC8FF) == 0x04) {
    uint8_t k;
    for (k = 0; k < 0x10; k++) SB_WR((uint8_t)(0x3E + k), sb_flip_rom_21b4[k]);
  }
}

/* ============================ ROM descriptor loader (bank1 b7a4) ============================
 * Copies the router/DROM identity ROM tables into staging XDATA, then latches a few PID/mode
 * bytes. The tables are DATA the host CM reads back, so they are copied byte-exact (Risk R2):
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

static void sb_rom_descriptor_load(void) {
  uint8_t i;
  for (i = 0; i < 0x64; i++) PR(0x0800 + i) = sb_drom_213d[i];
  /* latch PID/bcd from PD RAM into the DROM product fields */
  PR(0x0804) = PR(0x0A57);
  PR(0x0805) = PR(0x0A58);
  /* if not USB4-cap (0x09F9.7 clear): clear DROM[0x1B].1 */
  if (!(PR(0x09F9) & 0x80)) PR(0x081B) = PR(0x081B) & ~0x02;
  for (i = 0; i < 0x10; i++) PR(0x0700 + i) = sb_lane_desc_21d4[i];
  /* if 0x09F5 set: DROM[0x1A].5 = 1 (the richer stock guard is gated on 0x07BA/0x07CC/0x07B9/0x07CF
   * connect substate -- reproduce the leading set; the conditional clear-back is a no-op on the
   * happy connect path). */
  if (PR(0x09F5)) PR(0x081A) = (PR(0x081A) & 0xDF) | 0x20;
  /* if 0x09F6 == 0: DROM[0x1A] &= 0xED */
  if (PR(0x09F6) == 0) PR(0x081A) = PR(0x081A) & 0xED;

  /* ---- b83b-b8d8 tail (RE-AUDIT breakthrough A + #5): connect-state init + the SB[0xC9]
   * walking-1 connect-detect ARM + SB[0x28]/[0x2A]/[0x2C]/[0x66] + 0x6EE/0x6EF latch + CCD9 strobe
   * + SB[0xD4]/[0x8F]. a066 only services a channel when SB[0xC9] bit(4+idx) is SET; handmade
   * never armed it, so the C80A.5 SB-router service was never gated open. */
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
  PR(0x0719) = 0;
  PR(0xCCD9) = 0x04; PR(0xCCD9) = 0x02;          /* 97ef: CCD9 strobe */
  PR(0x074E) = 0; PR(0x074F) = 0;                /* 9874: zero per-lane CL0 latches */
  SB_CLR(0xD4, 0x20);                            /* SB[0xD4] &= ~0x20 */
  SB_WR(0x8F, (SB_RD(0x8F) & 0xEF) | 0x10);      /* SB[0x8F] = (&0xEF)|0x10 */
}

/* ============================ sb_block_init (bank1 bb37) ============================
 * Runs SECOND. ~25 SB-block RMWs + PHY descriptor seed + ROM tables + PHY-reg RMW. This is the
 * SB transport bring-up the host CM polls (SB[0x81]=0x08, SB[0x01].6/.7, SB[0x66]=0x20,
 * SB[0x9e]=0x20, the lane map) to begin USB4 lane training. Verbatim per §2 (the high-confidence
 * `sideband` decode: C2DC &= 0xC0, final SB[0x66]=0x20, SB[0x9e]=0x20; SB[0x15]/SB[0xC9] are NOT
 * seeded here). */
static void sb_block_init(void) {
  uart_puts("[SB Init]");

  /* e0d9 PHY descriptor seed (DPX=0 plain XDATA) */
  PR(0xC20E) = 0x3E; PR(0xC20F) = 0x08; PR(0xC210) = 0x08; PR(0xC211) = 0x2E; PR(0xC212) = 0x3E;
  PR(0xC214) = 0x00; PR(0xC215) = 0x20; PR(0xC216) = 0x00; PR(0xC217) = 0x3F;

  /* page-1 0x010100 head: clear bits 0,4,6,7 (net &= ~0xD1) */
  P1_CLR(0x0100, 0x10);
  P1_CLR(0x0100, 0x40);
  P1_CLR(0x0100, 0x80);
  P1_CLR(0x0100, 0x01);

  /* --- SB block writes (DPX=1) --- */
  SB_WR(0x2C, 0x01); SB_WR(0x2C, 0x02);          /* net SB[0x2c]=2 */
  SB_WR(0x26, 0x02);
  SB_WR(0x66, 0x01);                              /* transient (overridden below) */
  SB_CLR(0x2D, 0x01); SB_WR(0x2D, (SB_RD(0x2D) & 0xFD) | 0x02);  /* net b0=0, b1=1 */
  SB_CLR(0x29, 0x08);
  SB_CLR(0x2B, 0x08);
  SB_CLR(0xC8, 0xF0);                             /* clr bits 4,5,6,7 */
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
  PR(0xC233) = PR(0xC233) & ~0x08;
  PR(0xC2C4) = (PR(0xC2C4) & 0xBF) | 0x40;
  PR(0xC2DC) = PR(0xC2DC) & 0xC0;
  PR(0xC344) = (PR(0xC344) & 0xBF) | 0x40;
  PR(0xC35C) = PR(0xC35C) & 0xC0;

  /* e34b PHY lane cfg (bc42). Helpers: b70d(a)=&FE;&FD;(&C3)|1C;&BF ; b796(a)=(&BF)|40 ;
   * b73b: C2C3&=7F;C343&=7F. Transcribed VERBATIM from bank1 e34b (e34b..e38e). */
  PR(0x0AB3) = 0; PR(0x0AB4) = 3; PR(0x0AB5) = 3; PR(0x0AB6) = 0;
  /* b70d(C2C3) */
  PR(0xC2C3) = PR(0xC2C3) & 0xFE; PR(0xC2C3) = PR(0xC2C3) & 0xFD;
  PR(0xC2C3) = (PR(0xC2C3) & 0xC3) | 0x1C; PR(0xC2C3) = PR(0xC2C3) & 0xBF;
  PR(0xC2CB) = PR(0xC2CB) & 0xFB;                 /* e35f C2CB &= ~0x04 */
  /* b70d(C343) */
  PR(0xC343) = PR(0xC343) & 0xFE; PR(0xC343) = PR(0xC343) & 0xFD;
  PR(0xC343) = (PR(0xC343) & 0xC3) | 0x1C; PR(0xC343) = PR(0xC343) & 0xBF;
  PR(0xC34B) = PR(0xC34B) & 0xFB;                 /* e36c C34B &= ~0x04 */
  PR(0xC21C) = (PR(0xC21C) & 0xBF) | 0x40;        /* e373 b796(C21C) */
  PR(0xC208) = PR(0xC208) & 0xBF;                 /* e379 C208 &= ~0x40 */
  PR(0xC2C3) = PR(0xC2C3) & 0x7F;                 /* e380 b73b: C2C3 &= 0x7F */
  PR(0xC343) = PR(0xC343) & 0x7F;                 /* e380 b73b: C343 &= 0x7F */
  SB_CLR(0x1D, 0x02);                             /* e387 SB[0x1D] &= ~0x02 (DPX=1) */

  /* --- bb45 tail (bc45..bc5d): the truncated SB[0xBA]/[0xBD] + 0x09F7 sub-block --- */
  SB_WR(0xBA, 0x3F);                               /* bc45: SB[0xBA]=0x3F (via 0x9838) */
  SB_WR(0xBD, 0x3F);                               /* bc4c: SB[0xBD]=0x3F (A retained = 0x3F) */
  /* bc51: if (0x09F7 < 2) lcall func_05a2 (trampoline -> bank1 0xC523, a connect-state helper).
   * Not transcribed: it does NOT touch the SB-page target bytes (0xBA/0xBD already set above), and
   * 0x09F7 is the PD/USB4 sub-state counter which is >=2 on the happy connect path. Left as a
   * documented omission rather than fabricating the C523 body. */
}

/* d436: program B434 lane-ramp x4 + B436. Stock ramps B434 up to `width` (0xF) across 4 lanes. */
static void sb_pcie_width_ramp(uint8_t width) {
  PR(0xB434) = width; PR(0xB435) = width; PR(0xB436) = width; PR(0xB437) = width;
}

/* ============================ intermediate path-state (a48c..a516) ============================
 * The bank1 helper chain in usb4_connect_u4's tail that runs BEFORE b230. This is the BASELINE
 * subset that TRAINS E302 to mode 3 on HW. The fuller stock tail (edbd/e5b0/dd42(route)/bcc4/
 * e7ae PHY-lock wait/ccb3/c270/d556 + pre-gate dd42(0)/e7c1(1)/e0d9(0)) was transcribed and tried
 * and it REGRESSED E302 to mode 0 (the e7ae C006/C00E wait + extra PHY writes disturb the training
 * window) — so it is intentionally kept OUT. d436 ramps the downstream PCIe width; ee82 sets the
 * tunnel link-up bit. */
static void sb_connect_path_state(void) {
  /* bc8f: page1[0x0000] |= 0x02 */
  P1_SET(0x0000, 0x02);
  /* tunnel route (0x09FA & 0x81): tunnel/PCIe path bring-up */
  if (PR(0x09FA) & 0x81) {
    PR(0xCA06) = PR(0xCA06) & ~0x10;             /* clear bit4 */
    P1_CLR(0x0000, 0x04);                         /* bc91: page1[0x0000] &= ~0x04 */
    sb_pcie_width_ramp(0x0F);                      /* d436(0x0F) */
    if (PR(0x0AF1) & 0x10) {                       /* gated 0x0AF1.4 */
      PR(0xE710) = (PR(0xE710) & 0xE0) | 0x1F;
      PR(0xCC30) = PR(0xCC30) & ~0x01;
    }
    PR(0xB430) = PR(0xB430) | 0x01;               /* ee82: tunnel link-up bit */
    PR(0x924C) = (PR(0x924C) & 0xF7) | 0x08;
    P1_SET(0x0000, 0x04);                          /* page1[0x0000] |= 0x04 */
  }
}

/* M1' diagnostic: set once the SB assert has run, so the super-loop can switch to the post-SB
 * E302-poll diagnostic (the un-W1C'd C80A.5 SB-router source storms INT1 and starves the loop
 * until the a066/M2 handler exists; the diagnostic masks EX1 to read the trained E302). */
static volatile uint8_t sb_asserted = 0;

/* ============================ SB-assert entry (usb4_connect_u4 tail) ============================
 * The synchronous tail reached at 0xa51b/0xa51e after the route latch, gated on 0x07BA!=0.
 * Order: intermediate path-state -> sb_lane_flip_init (b230) FIRST -> sb_block_init (bb37). */
static void sb_assert(void) {
  sb_connect_path_state();      /* a48c..a516 intermediate tunnel/PCIe path state */
  PR(0x07FF) = 0;               /* 0x07FF=0 (stock clears just before b230) */
  sb_lane_flip_init();          /* b230 — FIRST */
  sb_block_init();              /* bb37 — SECOND */
  sb_asserted = 1;              /* SB-assert-done gate (see super-loop) */
  /* M2: INT1 stays ENABLED here. The C80A.5 SB-router source is now W1C-acked by
   * sb_router_event_handler (sb_router.h), so it no longer storms the CPU — the SB-connect
   * handshake must run in the ISR to advance the link, so do NOT mask EX1. (M1 masked it as a
   * diagnostic to read E302 in a busy loop; that is removed.) */
  /* Immediate (ISR-context, single-read) E302 marker so we capture the post-SB link mode even if
   * the super-loop's sleep() hangs on the shared CC10 timer mailbox (R-timer hazard). */
  uart_puts("[SBdone e302=");
  uart_puthex(PR(0xE302));
  /* SB write-landed self-readback (diagnostic): the page-1 0x2800 alias for the SB block is
   * RE-asserted, not HW-confirmed. Read back the keystone bits we just wrote: SB[0x81]==0x08,
   * SB[0x66]==0x20, SB[0x9E]==0x20, SB[0x01] bits6,7 set, SB[0xC9] (host connect bits). */
  uart_puts(" sb81=");  uart_puthex(SB_RD(0x81));
  uart_puts(" sb66=");  uart_puthex(SB_RD(0x66));
  uart_puts(" sb9e=");  uart_puthex(SB_RD(0x9E));
  uart_puts(" sb01=");  uart_puthex(SB_RD(0x01));
  uart_puts(" sb2d=");  uart_puthex(SB_RD(0x2D));
  uart_puts(" sbc9=");  uart_puthex(SB_RD(0xC9));
  uart_puts("]");

  /* Full SB-page + gating-reg snapshot for the STOCK-vs-handmade diff. Format mirrors the stock
   * patch_sbtrace.py [SBCON:...] line EXACTLY: gating regs in the SAME order, then '|', then the
   * full 256-byte SB page-1 block SB[0x00..0xFF] (DPX=1 paged XDATA at 0x2800+off). Taken right
   * after our SB-assert (the logical equivalent of stock's [===SB Con===] transition). */
  uart_puts("[HMSB:");
  uart_puthex(PR(0x0AF1)); uart_puthex(PR(0xC80A)); uart_puthex(PR(0xE302));
  uart_puthex(PR(0x09F9)); uart_puthex(PR(0x09FA)); uart_puthex(PR(0xEC06));
  uart_puthex(PR(0x91C0)); uart_puthex(PR(0xE318));
  uart_putc('|');
  { uint16_t off; for (off = 0; off < 0x100; off++) uart_puthex(SB_RD(off)); }
  uart_puts("]\n");
}

#endif /* SB_H */
