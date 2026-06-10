#ifndef USB4_IRQ_H
#define USB4_IRQ_H
/*
 * USB4 SB-transport / router interrupt arming — faithful transcription of the bank1 tail of
 * init_sys_flags @0x4BE6 (the C806/C80A/EC06 interrupt-enable group) that handmade's
 * pd_int1_enable_group (pd.h) only PARTIALLY transcribed.
 *
 * init_sys_flags @0x4BE6 does:  0x07F0-0x07F5 timer consts; CC35&=~1; C801=(&0xEF)|0x10;
 *   C800=(&0xFB)|4; CA60=(&0xF8)|6; CA60=(&0xF7)|8; 5418(C800)=C800|=1; CC3B=(&0xFD)|2;
 *   cb37 (bank0); ef24 (bank1: db0d+8e31); ef1e (bank1: d0ac+9a63).
 * pd_int1_enable_group already does the C801/C800/CA60/C800|=1 part. THIS file adds the missing
 * CC35/CC3B and the bank1 ef24/ef1e SB-transport arming, which enable the SB-PHY RX path that
 * detects the host's sideband connect packets and raises C80A.5 (the SB-router event the M2
 * handler services). Without it the SB block is powered but never signals connect.
 *
 *   ef24 = db0d + 8e31  (CODE_BANK1):
 *     db0d: C21B=(C21B&0x3F)|0xC0 ; C202=(C202&0xF7)|0x08 ; read page-0x12 reg 0x62 (discard)
 *     8e31: E741=(E741&0xF8)|0x03 ; E741=(E741&0xC7)|0x28 ; E742=(E742&0xFC)|0x03 ;
 *           E741=(E741&0x3F)|0x80 ; E742&=~0x08 ; CC43=(CC43&0x1F)|0x80 ;
 *           c390(C21F): C21F=(C21F&0xFB)|0x04 ; SB[0x49]=0xA0  (page-0x28 reg 0x49)
 *   ef1e = d0ac + 9a63  (CODE_BANK1):
 *     d0ac: read page-0x78 reg 0x9b (discard)
 *     9a63: read page-0x78 regs 0x40,0x41,0x34 ; 0x7834=(0x7834&0x8F)|0x60 (9661 RMW) ;
 *           read page-0x78 reg 0x20 (discard)
 *
 * Included AFTER sb.h (SB_WR + P1_*) and pd.h (PR()). All page-0x12/0x28/0x78 accesses are the
 * R3=2/R2=page/R1=off paged accessor == DPX=1 XDATA at 0x{page}{off} (same model as SB_WR but a
 * different page byte). Plain C2xx/E7xx/CCxx are bank0 XDATA (DPX=0).
 */

/* page-N (DPX=1) XDATA RMW helper for the PHY register banks (page 0x12 / 0x78). */
static uint8_t PG_RD(uint16_t addr) {
  uint8_t v; DPX = 0x01; v = XDATA_REG8V(addr); DPX = 0x00; return v;
}
static void PG_WR(uint16_t addr, uint8_t v) {
  DPX = 0x01; XDATA_REG8V(addr) = v; DPX = 0x00;
}

#include "usb4_rx_table.h"   /* u4rx_tab[][4]: the full ef1e SB-PHY RX-lane arm RMW table */

/* C2xx/C3xx PHY-config helper RMWs, transcribed VERBATIM from the bank1 helper functions that
 * 8E31 LCALLs. Each operates on a single XDATA reg `a` (the value 8E31 loaded into DPTR).
 *   c390:(&FB)|04  c34a:(&8F)|70  c2f8:(&FE)|01  c351:(&FD)|02  c358:(&1F)|60  c2f1:(&F0)|0B
 *   c2ff:(&F3)|04  c2bf:(&FC)|02  c35f:(&E0)|0A  c366:(&E0)|03  c36d:(&E0)|08  c2e0:(&F0)|07
 *   c2e7:(&F0)|0F  c374:(&E0)|11  c382:(&F0)|0D  c389:(&1F)|40  c37b:(&0F)|80
 *   c343: A=(C2A8&3F)[ret]   c328: A=(C328&3F)[ret, used at 8efe via c32d]
 *   c335(a): a=(&0F)|E0 ; (a+1)=(&0F)|70
 *   c30e(a): a&=FE; a&=FD; a&=FB; a&=F7
 *   c2d9(a): a=(&0F)|60 ; (a+1)=(&F0)|07
 *   c397(a): a=(&F1)|0E ; (a+1)=0
 *   c3a1(a): (a+1)=ACC ; (a+2)=0     [stores the prior ACC at a+1, zeroes a+2] */
#define RMW(a, m, o)   PR(a) = (PR(a) & (uint8_t)(m)) | (uint8_t)(o)
#define C390(a)        RMW((a), 0xFB, 0x04)
#define C34A(a)        RMW((a), 0x8F, 0x70)
#define C2F8(a)        RMW((a), 0xFE, 0x01)
#define C351(a)        RMW((a), 0xFD, 0x02)
#define C358(a)        RMW((a), 0x1F, 0x60)
#define C2F1(a)        RMW((a), 0xF0, 0x0B)
#define C2FF(a)        RMW((a), 0xF3, 0x04)
#define C2BF(a)        RMW((a), 0xFC, 0x02)
#define C35F(a)        RMW((a), 0xE0, 0x0A)
#define C366(a)        RMW((a), 0xE0, 0x03)
#define C36D(a)        RMW((a), 0xE0, 0x08)
#define C2E0(a)        RMW((a), 0xF0, 0x07)
#define C2E7(a)        RMW((a), 0xF0, 0x0F)
#define C374(a)        RMW((a), 0xE0, 0x11)
#define C382(a)        RMW((a), 0xF0, 0x0D)
#define C389(a)        RMW((a), 0x1F, 0x40)
#define C37B(a)        RMW((a), 0x0F, 0x80)
static void C335(uint16_t a) { RMW(a, 0x0F, 0xE0); RMW(a + 1, 0x0F, 0x70); }
static void C30E(uint16_t a) { PR(a) &= 0xFE; PR(a) &= 0xFD; PR(a) &= 0xFB; PR(a) &= 0xF7; }
static void C2D9(uint16_t a) { RMW(a, 0x0F, 0x60); RMW(a + 1, 0xF0, 0x07); }
static void C397(uint16_t a) { RMW(a, 0xF1, 0x0E); PR(a + 1) = 0; }

/* Lane-A and lane-B share the same body except for the base register; 8E31 mirrors lane A (c2xx)
 * into lane B (c3xx). The lane body is a tail block (8e6f..8f96 for A, 8f04..8f96 for B) plus a
 * second tail (8ffd.. for A, 90c3.. for B) — but they are NOT a clean function, so 8E31 is
 * transcribed straight-line below to stay byte-faithful to the stock instruction order. */

/* 8e31 (full PHY-RX descriptor config, 0x8e31..0x9259 in bank1). Configures the dual PHY lanes
 * (c2xx = lane A, c3xx = lane B) + page-0x93 + page-0x28 (SB) lane regs.
 * CORRECTION (RE-AUDIT breakthrough A): SB[0x94..0x99] are NOT "HW-latched / nothing writes them".
 * The b230 sb_lane_flip_init b307-b39e tail writes them EXPLICITLY (0d59 SB[0x94]=02/0x95=71/0x96=00,
 * 0c7a SB[0x98]=3E/0x99=80) -- handmade had dropped that whole tail, which is now restored in
 * sb.h::sb_lane_flip_init. This 8e31 block configures the underlying PHY lanes; both are needed. */
static void usb4_phy_rx_descriptor_8e31(void) {
  /* 8e31 head: E741/E742/CC43/C21F */
  PR(0xE741) = (PR(0xE741) & 0xF8) | 0x03;
  PR(0xE741) = (PR(0xE741) & 0xC7) | 0x28;
  PR(0xE742) = (PR(0xE742) & 0xFC) | 0x03;
  PR(0xE741) = (PR(0xE741) & 0x3F) | 0x80;
  PR(0xE742) = PR(0xE742) & 0xF7;
  PR(0xCC43) = (PR(0xCC43) & 0x1F) | 0x80;
  C390(0xC21F);                                  /* 8e5e */
  SB_WR(0x49, 0xA0);                              /* 8e64 */

  /* --- lane A (c2xx) cfg block 1: 8e6f..8f03 --- */
  PR(0xC2A8) = (PR(0xC2A8) & 0x3F) | 0x40;        /* 8e6f c343->A=(C2A8&3F); orl 40; movx */
  C34A(0xC2C5);                                   /* 8e75 */
  PR(0xC2A1) = (PR(0xC2A1) & 0x9F) | 0x60;        /* 8e7b */
  C2F8(0xC28C); C2F8(0xC29C); C2F8(0xC2AC);      /* 8e84/8e8a/8e90 */
  PR(0xC2BC) = PR(0xC2BC) & 0xFE;                 /* 8e96 */
  PR(0xC28C) = PR(0xC28C) & 0xFD;                 /* 8e9d */
  C351(0xC29C); C351(0xC2AC);                     /* 8ea4/8eaa */
  PR(0xC2BC) = PR(0xC2BC) & 0xFD;                 /* 8eb0 */
  PR(0xC2C3) = (PR(0xC2C3) & 0xC3) | 0x1C;        /* 8eb7 */
  PR(0xC2C9) = (PR(0xC2C9) & 0x80) | 0x41;        /* 8ec0 */
  C335(0xC2A5);                                   /* 8ec9 */
  C30E(0xC2CA);                                   /* 8ecf */
  C358(0xC287);                                   /* 8ed5 */
  C34A(0xC294);                                   /* 8edb */
  C358(0xC2A2);                                   /* 8ee1 */
  C2F1(0xC2C5);                                   /* 8ee7 */
  C2FF(0xC293);                                   /* 8eed */
  C2BF(0xC2CE);                                   /* 8ef3 */
  PR(0xC2CE) = (PR(0xC2CE) & 0xE3) | 0x14;        /* 8ef9; 8efe c32d writes C2CE then reads C328 */
  PR(0xC328) = (PR(0xC328) & 0x3F) | 0x40;        /* 8efe..8f03: C328=(C328&3F)|40 */

  /* --- lane B (c3xx) cfg block 1: 8f04..8f96 --- */
  C34A(0xC345);                                   /* 8f04 */
  PR(0xC321) = (PR(0xC321) & 0x9F) | 0x60;        /* 8f0a */
  C2F8(0xC30C); C2F8(0xC31C); C2F8(0xC32C);      /* 8f13/8f19/8f1f */
  PR(0xC33C) = PR(0xC33C) & 0xFE;                 /* 8f25 */
  PR(0xC30C) = PR(0xC30C) & 0xFD;                 /* 8f2c */
  C351(0xC31C); C351(0xC32C);                     /* 8f33/8f39 */
  PR(0xC33C) = PR(0xC33C) & 0xFD;                 /* 8f3f */
  PR(0xC343) = (PR(0xC343) & 0xC3) | 0x1C;        /* 8f46 */
  PR(0xC349) = (PR(0xC349) & 0x80) | 0x41;        /* 8f4f */
  C335(0xC325);                                   /* 8f58 */
  C30E(0xC34A);                                   /* 8f5e */
  C358(0xC307);                                   /* 8f64 */
  C34A(0xC314);                                   /* 8f6a */
  C358(0xC322);                                   /* 8f70 */
  C2F1(0xC345);                                   /* 8f76 */
  C2FF(0xC313);                                   /* 8f7c */
  C2BF(0xC34E);                                   /* 8f82 */
  PR(0xC34E) = (PR(0xC34E) & 0xE3) | 0x14;        /* 8f88 */
  PR(0xC21D) = (PR(0xC21D) & 0x3F) | 0x80;        /* 8f8e */

  /* page-0x93 clears: 8f97..8fa9 */
  PR(0x9316) = 0; PR(0x9317) = 0;
  PR(0x931A) = 0; PR(0x931B) = 0;
  PR(0x9322) = 0; PR(0x9323) = 0;

  /* --- lane A (c2xx) cfg block 2: 8faa..9063 --- */
  PR(0xC290) = PR(0xC290) & 0x9F;                 /* 8faa */
  PR(0xC2A0) = PR(0xC2A0) & 0x9F;                 /* 8fb1 */
  C35F(0xC282);                                   /* 8fb8 */
  PR(0xC292) = (PR(0xC292) & 0xE0) | 0x09;        /* 8fbe */
  C35F(0xC2A2);                                   /* 8fc7 */
  C366(0xC290);                                   /* 8fcd */
  C366(0xC2A0);                                   /* 8fd3 */
  C36D(0xC291);                                   /* 8fd9 */
  C36D(0xC2A1);                                   /* 8fdf */
  PR(0xC2DB) = (PR(0xC2DB) & 0xE0) | 0x1B;        /* 8fe5 */
  PR(0xC284) = (PR(0xC284) & 0xF0) | 0x05;        /* 8fee */
  C2E0(0xC294);                                   /* 8ffa */
  C2E7(0xC285);                                   /* 9000 */
  PR(0xC295) = (PR(0xC295) & 0xF0) | 0x0C;        /* 9003 */
  C2E7(0xC2A5);                                   /* 900c */
  C2D9(0xC285);                                   /* 9012 c2d9(C285)->C285/C286 */
  C2E7(0xC296);                                   /* 9018 */
  C374(0xC2A7);                                   /* 9021 */
  PR(0xC28B) = (PR(0xC28B) & 0xC0) | 0x0A;        /* 9024 */
  PR(0xC284) = (PR(0xC284) & 0x8F) | 0x40;        /* 902d */
  PR(0xC2A4) = PR(0xC2A4) & 0x8F;                 /* 9036 */
  PR(0xC289) = (PR(0xC289) & 0x0F) | 0x90;        /* 903d */
  C37B(0xC299);                                   /* 9046 */
  C37B(0xC2A9);                                   /* 904c */
  PR(0xC282) = (PR(0xC282) & 0x1F) | 0xA0;        /* 9052 */
  PR(0xC292) = (PR(0xC292) & 0x1F) | 0x20;        /* 905b */
  C382(0xC2C6);                                   /* 9064 */
  C397(0xC2CC);                                   /* 906d c397(C2CC)->C2CC/C2CD */

  /* --- lane B (c3xx) cfg block 2: 9070..9135 --- */
  PR(0xC310) = PR(0xC310) & 0x9F;                 /* 9070 */
  PR(0xC320) = PR(0xC320) & 0x9F;                 /* 9077 */
  C35F(0xC302);                                   /* 907e */
  PR(0xC312) = (PR(0xC312) & 0xE0) | 0x09;        /* 9084 */
  C35F(0xC322);                                   /* 908d */
  C366(0xC310);                                   /* 9093 */
  C366(0xC320);                                   /* 9099 */
  C36D(0xC311);                                   /* 909f */
  C36D(0xC321);                                   /* 90a5 */
  PR(0xC35B) = (PR(0xC35B) & 0xE0) | 0x1B;        /* 90ab */
  PR(0xC304) = (PR(0xC304) & 0xF0) | 0x05;        /* 90b4 */
  C2E0(0xC314);                                   /* 90bd */
  C2E7(0xC305);                                   /* 90c3 */
  PR(0xC315) = (PR(0xC315) & 0xF0) | 0x0C;        /* 90c9 */
  C2E7(0xC325);                                   /* 90d2 */
  C2D9(0xC305);                                   /* 90d8 c2d9(C305)->C305/C306 */
  C2E7(0xC316);                                   /* 90de */
  C374(0xC327);                                   /* 90e4 */
  PR(0xC30B) = (PR(0xC30B) & 0xC0) | 0x0A;        /* 90ea */
  PR(0xC304) = (PR(0xC304) & 0x8F) | 0x40;        /* 90f3 */
  PR(0xC324) = PR(0xC324) & 0x8F;                 /* 90fc */
  PR(0xC309) = (PR(0xC309) & 0x0F) | 0x90;        /* 9103 */
  C37B(0xC319);                                   /* 910c */
  C37B(0xC329);                                   /* 9112 */
  PR(0xC302) = (PR(0xC302) & 0x1F) | 0xA0;        /* 9118 */
  PR(0xC312) = (PR(0xC312) & 0x1F) | 0x20;        /* 9121 */
  C382(0xC346);                                   /* 912a */
  C397(0xC34C);                                   /* 9133 c397(C34C)->C34C/C34D */

  /* page-0x93 descriptor seed: 9136..9168 (emulated byte-exact incl. the c3a1 zero-fill helper) */
  PR(0x9310) = 0x01;                              /* 9136 */
  PR(0x9311) = 0x60; PR(0x9312) = 0x00;          /* 913b c3a1(0x60) */
  PR(0x9313) = 0xE3;                              /* 9140 */
  PR(0x9314) = 0x01;                              /* 9143 */
  PR(0x9315) = 0x60;                              /* 9147 */
  PR(0x9318) = 0x01;                              /* 914b */
  PR(0x9319) = 0x60;                              /* 9151 */
  PR(0x931C) = 0x00;                              /* 9155 */
  PR(0x931D) = 0x03; PR(0x931E) = 0x00;          /* 915a c3a1(0x03) */
  PR(0x931F) = 0xE0;                              /* 915f */
  PR(0x9320) = 0x00;                              /* 9162 */
  PR(0x9321) = 0xE3;                              /* 9165 */

  /* --- lane A (c2xx) cfg block 3: 9169..91e3 --- */
  C2FF(0xC2A3);                                   /* 9169 */
  C2FF(0xC323);                                   /* 916f (lane B reg, interleaved by stock) */
  C389(0xC297);                                   /* 9175 */
  PR(0xC29A) = (PR(0xC29A) & 0xF0) | 0x0E;        /* 917b */
  C389(0xC2A7);                                   /* 9184 */
  PR(0xC2AB) = PR(0xC2AB) & 0xC0;                 /* 918a */
  C389(0xC317);                                   /* 9191 (lane B) */
  PR(0xC31A) = (PR(0xC31A) & 0xF0) | 0x0E;        /* 9197 */
  C389(0xC327);                                   /* 91a0 (lane B) */
  PR(0xC32B) = PR(0xC32B) & 0xC0;                 /* 91a6 */
  C382(0xC2AA);                                   /* 91ad */
  PR(0xC297) = (PR(0xC297) & 0xE0) | 0x10;        /* 91b3 */
  PR(0xC293) = (PR(0xC293) & 0xFC) | 0x01;        /* 91bc */
  C2FF(0xC283);                                   /* 91c5 */
  C2F1(0xC2A6);                                   /* 91cb */
  C2E0(0xC2A4);                                   /* 91d1 */
  C2BF(0xC2A3);                                   /* 91d7 */
  PR(0xC29B) = PR(0xC29B) & 0xC0;                 /* 91dd */

  /* --- lane B (c3xx) cfg block 3: 91e4..921a --- */
  C382(0xC32A);                                   /* 91e4 */
  PR(0xC317) = (PR(0xC317) & 0xE0) | 0x10;        /* 91ea */
  PR(0xC313) = (PR(0xC313) & 0xFC) | 0x01;        /* 91f3 */
  C2FF(0xC303);                                   /* 91fc */
  C2F1(0xC326);                                   /* 9202 */
  C2E0(0xC324);                                   /* 9208 */
  C2BF(0xC323);                                   /* 920e */
  PR(0xC31B) = PR(0xC31B) & 0xC0;                 /* 9214 */

  /* 921b: if (C8FF >= 5) extra lane-rate cfg */
  if (PR(0xC8FF) >= 0x05) {
    PR(0xC294) = (PR(0xC294) & 0xF0) | 0x06;      /* 9224 */
    C374(0xC297);                                  /* 922d */
    PR(0xC314) = (PR(0xC314) & 0xF0) | 0x06;      /* 9233 */
    C374(0xC317);                                  /* 923c */
  }
  /* 9242: if (C8FF >= 6) further cfg */
  if (PR(0xC8FF) >= 0x06) {
    PR(0xC283) = PR(0xC283) & 0xF3;               /* 924b */
    PR(0xC303) = PR(0xC303) & 0xF3;               /* 9252 */
  }
  /* 9259: ret */
}

/* db0d (full): page-0x12[0x62] RMW + SB[0xED]/[0xCE]/[0x1C]/[0x1D] + C20B/C22F.
 * Produces the stock SB[0x1C]=0xC2 (|=0x80,0x40,0x02 from 0). Transcribed VERBATIM from bank1. */
static void usb4_irq_db0d(void) {
  PR(0xC21B) = (PR(0xC21B) & 0x3F) | 0xC0;        /* db0d */
  PR(0xC202) = (PR(0xC202) & 0xF7) | 0x08;        /* db16 */
  PG_WR(0x1262, PG_RD(0x1262) & 0xEF);            /* db1f page-0x12[0x62] &= 0xEF (NOT discard) */
  SB_WR(0xED, (SB_RD(0xED) & 0xBF) | 0x40);       /* db2d SB[0xED]=(&0xBF)|0x40 */
  SB_WR(0xCE, SB_RD(0xCE) & 0xFE);                /* db3b SB[0xCE] &= 0xFE */
  SB_SET(0x1C, 0x80);                              /* db45 SB[0x1C] |= 0x80 */
  SB_SET(0x1C, 0x40);                              /* db4f SB[0x1C] |= 0x40 */
  SB_SET(0x1C, 0x02);                              /* db57 SB[0x1C] |= 0x02  => 0xC2 */
  PR(0xC20B) = PR(0xC20B) & 0x7F;                 /* db5f C20B &= 0x7F */
  SB_WR(0x1D, SB_RD(0x1D) & 0xFE);                /* db66 inc r1 -> SB[0x1D] &= 0xFE */
  C390(0xC22F);                                    /* db72 c390(C22F): (&FB)|04 */
  PR(0xC22F) = PR(0xC22F) & 0xBF;                 /* db75 C22F &= 0xBF */
}

/* ef1e (d0ac + 9a63): the SB-PHY 4-lane RX arm. AUDIT FIX T2: the prior handmade version did ONLY
 * a single 0x7834 RMW; the real ef1e is ~324 paged RMWs across PHY pages 0x78-0x7b + descriptor
 * pages 0x60/0x64/0x68/0x6c (the dual-bank RX equalizer/rate config that lets the lanes lock onto
 * the host's sideband). Reproduced byte-exact via the u4rx_tab table (usb4_rx_table.h), applied as
 * read-modify-write. This is the SB-PHY RX lane enable the host's connect packets need to be seen.*/
static void usb4_irq_ef1e(void) {
  uint16_t i;
  for (i = 0; i < (uint16_t)(sizeof(u4rx_tab) / 4); i++) {
    uint16_t a = ((uint16_t)u4rx_tab[i][0] << 8) | u4rx_tab[i][1];
    PG_WR(a, (uint8_t)((PG_RD(a) & u4rx_tab[i][2]) | u4rx_tab[i][3]));
  }
}

/* ef24 = db0d + 8e31 (CODE_BANK1: `ef24: lcall db0d; ljmp 8e31`). Faithful order. */
static void usb4_irq_ef24(void) {
  usb4_irq_db0d();
  usb4_phy_rx_descriptor_8e31();
}

/* The missing init_sys_flags arming (CC35/CC3B + ef24 + ef1e). Call once at boot after
 * pd_int1_enable_group (pd_keystone_init). */
static void usb4_irq_arm(void) {
  PR(0xCC35) = PR(0xCC35) & 0xFE;                 /* CC35 &= ~1 */
  PR(0xCC3B) = (PR(0xCC3B) & 0xFD) | 0x02;        /* CC3B = (&0xFD)|2 */
  usb4_irq_ef24();
  usb4_irq_ef1e();
}

#endif /* USB4_IRQ_H */
