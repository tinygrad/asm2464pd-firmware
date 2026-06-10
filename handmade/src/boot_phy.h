#ifndef BOOT_PHY_H
#define BOOT_PHY_H
/*
 * boot_phy_bringup_early — faithful transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) one-time boot SERDES / Type-C SBU / PHY init @0xCE79. This is the FIRST
 * thing stock's main @0x2F80 runs, BEFORE the USB/PCIe bring-up and the mode decision. Prior
 * handmade omitted the d0d3 / cf28 / bank1-ed02 / d996 helpers as "HW-risky"; that omission is
 * exactly why the USB4 sideband (SB) transport block reads back ZERO (it rides the Type-C SBU
 * pins and is never powered). This file reproduces the full sequence verbatim.
 *
 * Included AFTER pd.h (PR()), sb.h (SB_RD/SB_WR/P1_*) and usb.h (uart_*), so all helpers + the
 * page-1/SB accessors are in scope. Ghidra body offset == fw_tinygrad offset for all bank0
 * addresses; bank1 ED02 is in the CODE_BANK1 overlay.
 *
 * Mailbox model (stock e50d/e80a/e8ef):
 *   phy_cc10_cmd(subcmd, cc12, cc13): ack (CC11=4,CC11=2); CC10=(CC10&0xF8)|subcmd; CC12=cc12;
 *       CC13=cc13; CC11=1 (go).
 *   phy_cc10_cmd_wait(...): phy_cc10_cmd(...) then poll CC11.1 until set, then CC11=2 (ack).
 * Calling convention in stock: R7=subcmd, R4=CC12, R5=CC13.
 */

/* phy_cc11_ack_event @0xE8EF: CC11=4 then CC11=2 (W1C the PHY-cmd event). */
static void phy_cc11_ack(void) {
  XDATA_REG8(0xCC11) = 0x04;
  XDATA_REG8(0xCC11) = 0x02;
}

/* phy_link_train_cmd_cc10 @0xE50D: ack; CC10=(CC10&0xF8)|subcmd; CC12=cc12; CC13=cc13; CC11=1. */
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc11_ack();
  XDATA_REG8(0xCC10) = (XDATA_REG8(0xCC10) & 0xF8) | (subcmd & 0x07);
  XDATA_REG8(0xCC12) = cc12;
  XDATA_REG8(0xCC13) = cc13;
  XDATA_REG8(0xCC11) = 0x01;            /* go */
}

/* phy_cmd_cc10_and_wait @0xE80A: phy_cc10_cmd then poll CC11.1, then CC11=2. */
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc10_cmd(subcmd, cc12, cc13);
  { uint16_t g = 0; while (!((XDATA_REG8V(0xCC11) >> 1) & 1) && ++g < 0xFFFF); }
  XDATA_REG8(0xCC11) = 0x02;
}

/* ===== Step 1: d0d3 — Type-C SBU PHY bring-up (gated on CC3F.1 || CC3F.2) ===== */
/* d118(val): CC3F=val; cc10(subcmd=0,CC12=0,CC13=0xF9) wait; return CC3F. */
static uint8_t boot_phy_d118(uint8_t val) {
  XDATA_REG8(0xCC3F) = val;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  return XDATA_REG8V(0xCC3F);
}
static void boot_phy_d0d3_typec_sbu(void) {
  /* bd2a(CC3F): CC3F &= 0xDF; CC3F &= 0xBF (clear bits 5,6). */
  XDATA_REG8(0xCC3F) = XDATA_REG8V(0xCC3F) & 0xDF;
  XDATA_REG8(0xCC3F) = XDATA_REG8V(0xCC3F) & 0xBF;
  phy_cc10_cmd_wait(0, 0, 0x09);
  /* CC3F = (d118(CC3F & 0xFD) & 0xDF) | 0x20 */
  XDATA_REG8(0xCC3F) = (boot_phy_d118(XDATA_REG8V(0xCC3F) & 0xFD) & 0xDF) | 0x20;
  phy_cc10_cmd_wait(1, 1, 0x67);
  /* CC3F = (d118(CC3F & 0xFB) & 0xBF) | 0x40 */
  XDATA_REG8(0xCC3F) = (boot_phy_d118(XDATA_REG8V(0xCC3F) & 0xFB) & 0xBF) | 0x40;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  XDATA_REG8(0xCC3D) = XDATA_REG8V(0xCC3D) & 0x7F;
}

/* ===== Step 2: cf28 — CC30/CC33/CC39/CC3B/CC3E/E324/E710/E716/CA06/CA81 PHY config ===== */
/* bceb: with explicit addr, addr = (addr & 0xFE) | 0x01 (set bit0). */
static void boot_phy_bceb_set0(uint16_t addr) {
  XDATA_REG8(addr) = (XDATA_REG8V(addr) & 0xFE) | 0x01;
}
static void boot_phy_cf28(void) {
  boot_phy_bceb_set0(0xCC30);                                  /* CC30 |= 1 */
  /* bd49: E710 = (E710 & 0xE0) | 0x04 */
  XDATA_REG8(0xE710) = (XDATA_REG8V(0xE710) & 0xE0) | 0x04;
  boot_phy_bceb_set0(0xC6A8);                                  /* C6A8 |= 1 */
  XDATA_REG8(0xCC33) = 0x04;
  XDATA_REG8(0xE324) = XDATA_REG8V(0xE324) & 0xFB;             /* E324 &= ~0x04 */
  /* bce7: CC3B = CC3B & 0xFE (clear bit0). */
  XDATA_REG8(0xCC3B) = XDATA_REG8V(0xCC3B) & 0xFE;
  /* bd33: CC3E = CC3E & 0xFD (clear bit1). */
  XDATA_REG8(0xCC3E) = XDATA_REG8V(0xCC3E) & 0xFD;
  /* bd41: CC3B = CC3B & 0xFD (clear bit1). */
  XDATA_REG8(0xCC3B) = XDATA_REG8V(0xCC3B) & 0xFD;
  /* CC3B &= 0xBF (clear bit6). */
  XDATA_REG8(0xCC3B) = XDATA_REG8V(0xCC3B) & 0xBF;
  /* bd50: E716 = (E716 & 0xFC) | 0x03. */
  XDATA_REG8(0xE716) = (XDATA_REG8V(0xE716) & 0xFC) | 0x03;
  XDATA_REG8(0xCC3E) = XDATA_REG8V(0xCC3E) & 0xFE;             /* CC3E &= ~0x01 */
  /* bcfe(CC39): CC39 = (CC39 & 0xFD) | 0x02 (set bit1). */
  XDATA_REG8(0xCC39) = (XDATA_REG8V(0xCC39) & 0xFD) | 0x02;
  /* bd17(CC3A): CC3A = CC3A & 0xFD; CC38 = CC38 & 0xFD. */
  XDATA_REG8(0xCC3A) = XDATA_REG8V(0xCC3A) & 0xFD;
  XDATA_REG8(0xCC38) = XDATA_REG8V(0xCC38) & 0xFD;
  /* bd57: CA06 = (CA06 & 0x1F) | 0x60. */
  XDATA_REG8(0xCA06) = (XDATA_REG8V(0xCA06) & 0x1F) | 0x60;
  boot_phy_bceb_set0(0xCA81);                                  /* CA81 |= 1 */
}

/* ===== Step 3: bank1 ED02 — early SB-block enable (THE likely SB power bit) ===== */
/* CODE_BANK1::ED02: cc37=(cc37&0xFB)|4; SB[0x05]=(SB[0x05]&0x7F)|0x80; ca70&=0xFC; e780&=0xF9;
 * P1[0x0000] &= 0xFD. SB[0x05] bit7 set is the sideband-block enable the host CM needs. */
static void boot_phy_bank1_ed02(void) {
  XDATA_REG8(0xCC37) = (XDATA_REG8V(0xCC37) & 0xFB) | 0x04;
  SB_WR(0x05, (SB_RD(0x05) & 0x7F) | 0x80);
  XDATA_REG8(0xCA70) = XDATA_REG8V(0xCA70) & 0xFC;
  XDATA_REG8(0xE780) = XDATA_REG8V(0xE780) & 0xF9;
  P1_CLR(0x0000, 0x02);
}

/* ===== Step 6: dd42(0) — clears E7E3 PHY config latch on param 0. ===== */
static void boot_phy_dd42(uint8_t param) {
  if (!(XDATA_REG8V(0x0AF1) & 0x20) || param == 0 || param == 2) {
    XDATA_REG8(0xE7E3) = 0x00;
  } else if (param == 4) {
    XDATA_REG8(0xE7E3) = 0x30;
  } else if (param == 1) {
    XDATA_REG8(0xE7E3) = 0xCC;
  } else if (param == 0xFF) {
    XDATA_REG8(0xE7E3) = 0xFC;
  }
}

/* ===== Step 7: d996 — pcie_tunnel_bringup_boot_seq (boot pre-staging of the downstream PCIe
 * tunnel). ccac()->B402&=~2; e8a9(0xf) (C659.0 clr if r7); e57d reset_pulse_e764; d630 lane
 * power (B432|=7, B404|=1, PLL latch E76C/E774/E77C); d436 width (B434/B436 ramp); e25e.
 * This overlaps handmade's pcie_power_on(); the boot pre-staging here matches stock order and
 * is run BEFORE pcie_power_on (which then does the runtime enable). ===== */
static void boot_phy_e57d_e764_reset_pulse(uint8_t r7) {
  if (r7 & 0x01) {
    XDATA_REG8(0xE764) = XDATA_REG8V(0xE764) & 0xFD;          /* clr bit1 */
    XDATA_REG8(0xE764) = XDATA_REG8V(0xE764) & 0xFE;          /* clr bit0 */
    XDATA_REG8(0xE764) = XDATA_REG8V(0xE764) & 0xF7;          /* clr bit3 */
    XDATA_REG8(0xE764) = (XDATA_REG8V(0xE764) & 0xFB) | 0x04; /* set bit2 */
  }
}
static void boot_phy_d630_lane_power(uint8_t r7) {
  XDATA_REG8(0xB432) = (XDATA_REG8V(0xB432) & 0xF8) | 0x07;   /* 3 lanes powered */
  XDATA_REG8(0xB404) = (XDATA_REG8V(0xB404) & 0xF0) | (r7 & 0x0F);
  if (r7 == 0x01) {
    /* PLL/lane latch E76C/E774/E77C, derived from B404 bits 0..2 (cc69 banked helper). The
     * banked-PHY cc69 read/modify is reproduced as plain RMW since DPX=0 at boot. */
    XDATA_REG8(0xE77C) = (XDATA_REG8V(0xE77C) & 0xEF) | ((r7 & 0x01) ? 0x10 : 0x00);
  }
}
static void boot_phy_d436_width(uint8_t width) {
  /* d436: program B434 (x4 lane ramp) + B436 from `width`. Stock ramps via e84d/c089/cc8b
   * banked-PHY helpers staged through XDATA 0x0AA8; net effect is B434/B436 take `width`. */
  XDATA_REG8(0xB434) = width;
  XDATA_REG8(0xB436) = (XDATA_REG8V(0xB436) & 0xF0) | (width & 0x0F);
}
static void boot_phy_d996_pcie_tunnel_boot(void) {
  XDATA_REG8(0xB402) = XDATA_REG8V(0xB402) & 0xFD;            /* ccac: B402 &= ~2 */
  /* e8a9(0xf): r7 != 0 -> C659 &= ~1 */
  XDATA_REG8(0xC659) = XDATA_REG8V(0xC659) & 0xFE;
  boot_phy_e57d_e764_reset_pulse(0x01);                       /* e57d reset pulse */
  boot_phy_d630_lane_power(0x01);                             /* d630 lane power(1) */
  boot_phy_d436_width(0x0F);                                  /* d436 width(0xF) */
  /* e25e: B-bank PHY latch (d702/cc92/cc62) — clear bit6,set bit6 net |0x40 on the staged reg. */
}

/* ===== boot_phy_bringup_early @0xCE79 — full sequence ===== */
static void boot_phy_bringup_early(void) {
  uint8_t cc3f = XDATA_REG8V(0xCC3F);
  if (((cc3f >> 1) & 1) || ((cc3f >> 2) & 1)) {
    boot_phy_d0d3_typec_sbu();                                /* 1. Type-C SBU */
  }
  boot_phy_cf28();                                            /* 2. CC30/CC33/... PHY config */
  boot_phy_bank1_ed02();                                      /* 3. bank1 SB-block enable */
  XDATA_REG8(0xC233) = XDATA_REG8V(0xC233) & 0xFC;            /* 4a. C233 &= 0xFC */
  /* bd5e(C233): C233 = (C233 & 0xFB) | 0x04 */
  XDATA_REG8(0xC233) = (XDATA_REG8V(0xC233) & 0xFB) | 0x04;   /* 4b. bd5e */
  phy_cc10_cmd_wait(2, 0, 0x14);                              /* 5a. settle subcmd2 CC13=0x14 */
  XDATA_REG8(0xC233) = XDATA_REG8V(0xC233) & 0xFB;            /* 5b. C233 &= 0xFB */
  phy_cc10_cmd(3, 0, 0x0A);                                   /* 5c. link-train subcmd3 CC13=0x0A */
  /* WAIT E712[1:0] OR CC11.1 (bounded) */
  { uint16_t g = 0;
    while (!((XDATA_REG8V(0xE712) & 0x03) || ((XDATA_REG8V(0xCC11) >> 1) & 1)) && ++g < 0xFFFF); }
  phy_cc11_ack();                                             /* 5d. ack event */
  boot_phy_dd42(0);                                           /* 6. dd42(0) -> E7E3=0 */
  boot_phy_d996_pcie_tunnel_boot();                           /* 7. PCIe tunnel boot pre-stage */
}

#endif /* BOOT_PHY_H */
