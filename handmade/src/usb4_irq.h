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

/* ef24 (db0d + 8e31): SB-PHY RX / router interrupt arm + PHY descriptor. */
static void usb4_irq_ef24(void) {
  /* db0d */
  PR(0xC21B) = (PR(0xC21B) & 0x3F) | 0xC0;
  PR(0xC202) = (PR(0xC202) & 0xF7) | 0x08;
  (void)PG_RD(0x1262);                          /* read page-0x12 reg 0x62 (discard) */
  /* 8e31 */
  PR(0xE741) = (PR(0xE741) & 0xF8) | 0x03;
  PR(0xE741) = (PR(0xE741) & 0xC7) | 0x28;
  PR(0xE742) = (PR(0xE742) & 0xFC) | 0x03;
  PR(0xE741) = (PR(0xE741) & 0x3F) | 0x80;
  PR(0xE742) = PR(0xE742) & 0xF7;
  PR(0xCC43) = (PR(0xCC43) & 0x1F) | 0x80;
  PR(0xC21F) = (PR(0xC21F) & 0xFB) | 0x04;       /* c390 */
  SB_WR(0x49, 0xA0);                              /* page-0x28 SB[0x49]=0xA0 */
}

/* ef1e (d0ac + 9a63): SB-PHY page-0x78 RX-lane arm. */
static void usb4_irq_ef1e(void) {
  (void)PG_RD(0x789B);                            /* d0ac: read page-0x78 reg 0x9b (discard) */
  (void)PG_RD(0x7840);                            /* 9a63: read 0x40 */
  (void)PG_RD(0x7841);                            /* read 0x41 */
  (void)PG_RD(0x7834);                            /* read 0x34 */
  PG_WR(0x7834, (PG_RD(0x7834) & 0x8F) | 0x60);   /* 9661: 0x7834 RMW */
  (void)PG_RD(0x7820);                            /* read 0x20 (discard) */
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
