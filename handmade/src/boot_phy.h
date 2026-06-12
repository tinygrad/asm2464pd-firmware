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
  REG_TIMER0_CSR = 0x04;
  REG_TIMER0_CSR = 0x02;
}

/* phy_link_train_cmd_cc10 @0xE50D: ack; CC10=(CC10&0xF8)|subcmd; CC12=cc12; CC13=cc13; CC11=1. */
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc11_ack();
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | (subcmd & 0x07);
  REG_TIMER0_THRESHOLD_HI = cc12;
  REG_TIMER0_THRESHOLD_LO = cc13;
  REG_TIMER0_CSR = 0x01;            /* go */
}

/* phy_cmd_cc10_and_wait @0xE80A: phy_cc10_cmd then poll CC11.1, then CC11=2. */
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc10_cmd(subcmd, cc12, cc13);
  { uint16_t g = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++g < 0xFFFF); }
  REG_TIMER0_CSR = 0x02;
}

/* ===== Step 1: d0d3 — Type-C SBU PHY bring-up (gated on CC3F.1 || CC3F.2) ===== */
/* d118(val): CC3F=val; cc10(subcmd=0,CC12=0,CC13=0xF9) wait; return CC3F. */
static uint8_t boot_phy_d118(uint8_t val) {
  REG_LTSSM_CTRL = val;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  return REG_LTSSM_CTRL;
}
static void boot_phy_d0d3_typec_sbu(void) {
  /* bd2a(CC3F): CC3F &= 0xDF; CC3F &= 0xBF (clear bits 5,6). */
  REG_LTSSM_CTRL = REG_LTSSM_CTRL & 0xDF;
  REG_LTSSM_CTRL = REG_LTSSM_CTRL & 0xBF;
  phy_cc10_cmd_wait(0, 0, 0x09);
  /* CC3F = (d118(CC3F & 0xFD) & 0xDF) | 0x20 */
  REG_LTSSM_CTRL = (boot_phy_d118(REG_LTSSM_CTRL & 0xFD) & 0xDF) | 0x20;
  phy_cc10_cmd_wait(1, 1, 0x67);
  /* CC3F = (d118(CC3F & 0xFB) & 0xBF) | 0x40 */
  REG_LTSSM_CTRL = (boot_phy_d118(REG_LTSSM_CTRL & 0xFB) & 0xBF) | 0x40;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  REG_LTSSM_STATE = REG_LTSSM_STATE & 0x7F;
}

/* ===== Step 2: cf28 — CC30/CC33/CC39/CC3B/CC3E/E324/E710/E716/CA06/CA81 PHY config ===== */
/* bceb: with explicit addr, addr = (addr & 0xFE) | 0x01 (set bit0). */
static void boot_phy_bceb_set0(uint16_t addr) {
  XDATA_REG8(addr) = (XDATA_REG8V(addr) & 0xFE) | 0x01;
}
static void boot_phy_cf28(void) {
  boot_phy_bceb_set0(0xCC30);                                  /* CC30 |= 1 */
  /* bd49: E710 = (E710 & 0xE0) | 0x04 */
  REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;
  boot_phy_bceb_set0(0xC6A8);                                  /* C6A8 |= 1 */
  REG_CPU_EXEC_STATUS_2 = 0x04;
  REG_LINK_CTRL_E324 = REG_LINK_CTRL_E324 & 0xFB;             /* E324 &= ~0x04 */
  /* bce7: CC3B = CC3B & 0xFE (clear bit0). */
  REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFE;
  /* bd33: CC3E = CC3E & 0xFD (clear bit1). */
  REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFD;
  /* bd41: CC3B = CC3B & 0xFD (clear bit1). */
  REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFD;
  /* CC3B &= 0xBF (clear bit6). */
  REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xBF;
  /* bd50: E716 = (E716 & 0xFC) | 0x03. */
  REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
  REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFE;             /* CC3E &= ~0x01 */
  /* bcfe(CC39): CC39 = (CC39 & 0xFD) | 0x02 (set bit1). */
  REG_TIMER_CTRL_CC39 = (REG_TIMER_CTRL_CC39 & 0xFD) | 0x02;
  /* bd17(CC3A): CC3A = CC3A & 0xFD; CC38 = CC38 & 0xFD. */
  REG_TIMER_ENABLE_B = REG_TIMER_ENABLE_B & 0xFD;
  REG_TIMER_ENABLE_A = REG_TIMER_ENABLE_A & 0xFD;
  /* bd57: CA06 = (CA06 & 0x1F) | 0x60. */
  REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  boot_phy_bceb_set0(0xCA81);                                  /* CA81 |= 1 */
}

/* ===== Step 3: bank1 ED02 — early SB-block enable (THE likely SB power bit) ===== */
/* CODE_BANK1::ED02: cc37=(cc37&0xFB)|4; SB[0x05]=(SB[0x05]&0x7F)|0x80; ca70&=0xFC; e780&=0xF9;
 * P1[0x0000] &= 0xFD. SB[0x05] bit7 set is the sideband-block enable the host CM needs. */
static void boot_phy_bank1_ed02(void) {
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  SB_WR(0x05, (SB_RD(0x05) & 0x7F) | 0x80);
  REG_CPU_CTRL_CA70 = REG_CPU_CTRL_CA70 & 0xFC;
  REG_SYS_CTRL_E780 = REG_SYS_CTRL_E780 & 0xF9;
  P1_CLR(0x0000, 0x02);
}

/* ===== Step 6: dd42(0) — clears E7E3 PHY config latch on param 0. ===== */
static void boot_phy_dd42(uint8_t param) {
  if (!(XDATA_REG8V(0x0AF1) & 0x20) || param == 0 || param == 2) {
    REG_PHY_LINK_CTRL = 0x00;
  } else if (param == 4) {
    REG_PHY_LINK_CTRL = 0x30;
  } else if (param == 1) {
    REG_PHY_LINK_CTRL = 0xCC;
  } else if (param == 0xFF) {
    REG_PHY_LINK_CTRL = 0xFC;
  }
}

/* ===== Step 7: d996 — pcie_tunnel_bringup_boot_seq (boot pre-staging of the downstream PCIe
 * tunnel). ccac()->B402&=~2; e8a9(0xf) (C659.0 clr if r7); e57d reset_pulse_e764; d630 lane
 * power (B432|=7, B404|=1, PLL latch E76C/E774/E77C); d436 width (B434/B436 ramp); e25e.
 * This overlaps handmade's pcie_power_on(); the boot pre-staging here matches stock order and
 * is run BEFORE pcie_power_on (which then does the runtime enable). ===== */
static void boot_phy_e57d_e764_reset_pulse(uint8_t r7) {
  if (r7 & 0x01) {
    REG_PHY_TIMER_CTRL_E764 = REG_PHY_TIMER_CTRL_E764 & 0xFD;          /* clr bit1 */
    REG_PHY_TIMER_CTRL_E764 = REG_PHY_TIMER_CTRL_E764 & 0xFE;          /* clr bit0 */
    REG_PHY_TIMER_CTRL_E764 = REG_PHY_TIMER_CTRL_E764 & 0xF7;          /* clr bit3 */
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xFB) | 0x04; /* set bit2 */
  }
}
static void boot_phy_d630_lane_power(uint8_t r7) {
  XDATA_REG8(0xB432) = (XDATA_REG8V(0xB432) & 0xF8) | 0x07;   /* 3 lanes powered */
  XDATA_REG8(0xB404) = (XDATA_REG8V(0xB404) & 0xF0) | (r7 & 0x0F);
  if (r7 == 0x01) {
    /* PLL/lane latch E76C/E774/E77C, derived from B404 bits 0..2 (cc69 banked helper). The
     * banked-PHY cc69 read/modify is reproduced as plain RMW since DPX=0 at boot. */
    REG_SYS_CTRL_E77C = (REG_SYS_CTRL_E77C & 0xEF) | ((r7 & 0x01) ? 0x10 : 0x00);
  }
}
static void boot_phy_d436_width(uint8_t width) {
  /* d436: program B434 (x4 lane ramp) + B436 from `width`. Stock ramps via e84d/c089/cc8b
   * banked-PHY helpers staged through XDATA 0x0AA8; net effect is B434/B436 take `width`. */
  XDATA_REG8(0xB434) = width;
  XDATA_REG8(0xB436) = (XDATA_REG8V(0xB436) & 0xF0) | (width & 0x0F);
}
static void boot_phy_d996_pcie_tunnel_boot(void) {
  REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;            /* ccac: B402 &= ~2 */
  /* e8a9(0xf): r7 != 0 -> C659 &= ~1 */
  REG_PCIE_LANE_CTRL_C659 = REG_PCIE_LANE_CTRL_C659 & 0xFE;
  boot_phy_e57d_e764_reset_pulse(0x01);                       /* e57d reset pulse */
  boot_phy_d630_lane_power(0x01);                             /* d630 lane power(1) */
  boot_phy_d436_width(0x0F);                                  /* d436 width(0xF) */
  /* e25e: B-bank PHY latch (d702/cc92/cc62) — clear bit6,set bit6 net |0x40 on the staged reg. */
}

/* ===== boot_phy_bringup_early @0xCE79 — full sequence ===== */
static void boot_phy_bringup_early(void) {
  uint8_t cc3f = REG_LTSSM_CTRL;
  if (((cc3f >> 1) & 1) || ((cc3f >> 2) & 1)) {
    boot_phy_d0d3_typec_sbu();                                /* 1. Type-C SBU */
  }
  boot_phy_cf28();                                            /* 2. CC30/CC33/... PHY config */
  boot_phy_bank1_ed02();                                      /* 3. bank1 SB-block enable */
  REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFC;            /* 4a. C233 &= 0xFC */
  /* bd5e(C233): C233 = (C233 & 0xFB) | 0x04 */
  REG_PHY_CONFIG = (REG_PHY_CONFIG & 0xFB) | 0x04;   /* 4b. bd5e */
  phy_cc10_cmd_wait(2, 0, 0x14);                              /* 5a. settle subcmd2 CC13=0x14 */
  REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFB;            /* 5b. C233 &= 0xFB */
  phy_cc10_cmd(3, 0, 0x0A);                                   /* 5c. link-train subcmd3 CC13=0x0A */
  /* WAIT E712[1:0] OR CC11.1 (bounded) */
  { uint16_t g = 0;
    while (!((REG_LINK_STATUS_E712 & 0x03) || ((REG_TIMER0_CSR >> 1) & 1)) && ++g < 0xFFFF); }
  phy_cc11_ack();                                             /* 5d. ack event */
  boot_phy_dd42(0);                                           /* 6. dd42(0) -> E7E3=0 */
  boot_phy_d996_pcie_tunnel_boot();                           /* 7. PCIe tunnel boot pre-stage */
}

/* bank0 92C5 RAM-state seed (audit O6/S4). Stock's boot step 4e: seeds the lane-engine link
 * WIDTH/MODE/state RAM the USB4 tunnel reads. Faithful to the decompiled 92C5 HEAD + the non-OTP
 * (no 0x5A magic at 0x707E) LAB_94c5 tail. NOTE vs audit: the head seeds 0x0AE3..0x0AE8/0x0AF0 to
 * *1* (NOT zero as the audit guessed) and 0x0AEB..0x0AEF to *3*; 0x0AE9=0x0F (width), 0x0AEE=3
 * (mode). With 0x0AE6=1 the tail sets 0x0AF1=0 (matches stock); 0x0AE4=1 -> C65A &= 0xF7; 0x0AE3=1
 * & 0x0AE6=1 -> CC35 &= 0xFB; 0x905F &= 0xEF. The OTP-override branch (0x707E=='Z') is absent on
 * this part so it is not reproduced. Run at boot before the SB path (stock runs it in boot_hw_init
 * step 4e, before the USB4 fork). */
static void bank0_92c5_seed(void) {
  XDATA_REG8(0x0213) = 0;
  XDATA_REG8(0x0AEA) = 1;
  XDATA_REG8(0x0AE3) = 1;
  XDATA_REG8(0x0AE4) = 1;
  XDATA_REG8(0x0AF0) = 1;
  XDATA_REG8(0x0AE5) = 1;
  XDATA_REG8(0x0AE6) = 1;
  XDATA_REG8(0x0AE7) = 1;
  XDATA_REG8(0x0AE8) = 1;
  XDATA_REG8(0x0AE9) = 0x0F;   /* link WIDTH */
  XDATA_REG8(0x0AEE) = 3;      /* link MODE */
  XDATA_REG8(0x0AEF) = 3;
  XDATA_REG8(0x0AEB) = 3;
  XDATA_REG8(0x0AEC) = 3;
  XDATA_REG8(0x0AED) = 3;
  XDATA_REG8(0x0A83) = 0;
  /* non-OTP LAB_94c5 tail (resolved with the seeded values above) */
  XDATA_REG8(0x0AEB) = XDATA_REG8V(0x0AEB) | 0x01;              /* 0x0AEB |= 1 (stays 3) */
  XDATA_REG8(0x0AF1) = 0x00;                                    /* 0x0AE6!=0 -> 0x0AF1 = 0 */
  REG_PHY_CFG_C65A = REG_PHY_CFG_C65A & 0xF7;              /* 0x0AE4!=0 -> C65A &= 0xF7 */
  REG_CPU_EXEC_STATUS_3 = REG_CPU_EXEC_STATUS_3 & 0xFB;              /* 0x0AE3,0x0AE6!=0 -> CC35 &= 0xFB */
  REG_USB_EP_CTRL_905F = REG_USB_EP_CTRL_905F & 0xEF;             /* 0x905F &= 0xEF */
}

#endif /* BOOT_PHY_H */
