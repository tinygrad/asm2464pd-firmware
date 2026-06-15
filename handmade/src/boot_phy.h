#ifndef BOOT_PHY_H
#define BOOT_PHY_H
/*
 * One-time boot SERDES / Type-C SBU / PHY init. Powers the Type-C SBU pins the
 * USB4 sideband transport rides on. Include AFTER pd.h, sb.h and usb.h.
 */

/* W1C the PHY-cmd event. */
static void phy_cc11_ack(void) {
  REG_TIMER0_CSR = 0x04;
  REG_TIMER0_CSR = 0x02;
}

/* Issue a PHY command: load subcmd/args into the mailbox and set go. */
static void phy_cc10_cmd(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc11_ack();
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | (subcmd & 0x07);
  REG_TIMER0_THRESHOLD_HI = cc12;
  REG_TIMER0_THRESHOLD_LO = cc13;
  REG_TIMER0_CSR = 0x01;
}

/* Issue a PHY command then poll for completion and ack it. */
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13) {
  phy_cc10_cmd(subcmd, cc12, cc13);
  { uint16_t spin = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++spin < 0xFFFF); }
  REG_TIMER0_CSR = 0x02;
}

/* Write LTSSM ctrl, run a PHY settle command, return the ctrl readback. */
static uint8_t boot_phy_d118(uint8_t ctrl) {
  REG_LTSSM_CTRL = ctrl;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  return REG_LTSSM_CTRL;
}
/* Type-C SBU PHY bring-up. */
static void boot_phy_d0d3_typec_sbu(void) {
  REG_LTSSM_CTRL &= 0xDF;
  REG_LTSSM_CTRL &= 0xBF;
  phy_cc10_cmd_wait(0, 0, 0x09);
  REG_LTSSM_CTRL = (boot_phy_d118(REG_LTSSM_CTRL & 0xFD) & 0xDF) | 0x20;
  phy_cc10_cmd_wait(1, 1, 0x67);
  REG_LTSSM_CTRL = (boot_phy_d118(REG_LTSSM_CTRL & 0xFB) & 0xBF) | 0x40;
  phy_cc10_cmd_wait(0, 0, 0xF9);
  REG_LTSSM_STATE &= 0x7F;
}

/* Set bit0 of an XDATA register. */
static void boot_phy_bceb_set0(uint16_t addr) {
  XDATA_REG8(addr) = (XDATA_REG8V(addr) & 0xFE) | 0x01;
}
/* PHY config block: program the CC3x/E3xx/E71x link registers. */
static void boot_phy_cf28(void) {
  boot_phy_bceb_set0(0xCC30);
  REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;
  boot_phy_bceb_set0(0xC6A8);
  REG_CPU_EXEC_STATUS_2 = 0x04;
  REG_LINK_CTRL_E324 &= 0xFB;
  REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFE;
  REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFD;
  REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFD;
  REG_TIMER_CTRL_CC3B &= 0xBF;
  REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
  REG_CPU_CTRL_CC3E &= 0xFE;
  REG_TIMER_CTRL_CC39 = (REG_TIMER_CTRL_CC39 & 0xFD) | 0x02;
  REG_TIMER_ENABLE_B = REG_TIMER_ENABLE_B & 0xFD;
  REG_TIMER_ENABLE_A &= 0xFD;
  REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  boot_phy_bceb_set0(0xCA81);
}

/* Early SB-block enable: SB[0x05] bit7 powers the sideband block. */
static void boot_phy_bank1_ed02(void) {
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  SB_WR(0x05, (SB_RD(0x05) & 0x7F) | 0x80);
  REG_CPU_CTRL_CA70 &= 0xFC;
  REG_SYS_CTRL_E780 &= 0xF9;
  P1_CLR(0x0000, 0x02);
}

/* Program the E7E3 PHY config latch from a mode selector. */
static void boot_phy_dd42(uint8_t mode) {
  if (!(XDATA_REG8V(0x0AF1) & 0x20) || mode == 0 || mode == 2) {
    REG_PHY_LINK_CTRL = 0x00;
  } else if (mode == 4) {
    REG_PHY_LINK_CTRL = 0x30;
  } else if (mode == 1) {
    REG_PHY_LINK_CTRL = 0xCC;
  } else if (mode == 0xFF) {
    REG_PHY_LINK_CTRL = 0xFC;
  }
}

/* PCIe-tunnel PHY timer reset pulse. */
static void boot_phy_e57d_e764_reset_pulse(uint8_t enable) {
  if (enable & 0x01) {
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
    REG_PHY_TIMER_CTRL_E764 &= 0xFE;
    REG_PHY_TIMER_CTRL_E764 &= 0xF7;
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xFB) | 0x04;
  }
}
/* Power up the PCIe-tunnel lanes and latch the PLL. */
static void boot_phy_d630_lane_power(uint8_t enable) {
  XDATA_REG8(0xB432) = (XDATA_REG8V(0xB432) & 0xF8) | 0x07;
  XDATA_REG8(0xB404) = (XDATA_REG8V(0xB404) & 0xF0) | (enable & 0x0F);
  if (enable == 0x01) {
    REG_SYS_CTRL_E77C = (REG_SYS_CTRL_E77C & 0xEF) | ((enable & 0x01) ? 0x10 : 0x00);
  }
}
/* Program the PCIe-tunnel lane width. */
static void boot_phy_d436_width(uint8_t width) {
  XDATA_REG8(0xB434) = width;
  XDATA_REG8(0xB436) = (XDATA_REG8V(0xB436) & 0xF0) | (width & 0x0F);
}
/* Boot pre-staging of the downstream PCIe tunnel (runs before runtime power-on). */
static void boot_phy_d996_pcie_tunnel_boot(void) {
  REG_PCIE_CTRL_B402 &= 0xFD;
  REG_PCIE_LANE_CTRL_C659 &= 0xFE;
  boot_phy_e57d_e764_reset_pulse(0x01);
  boot_phy_d630_lane_power(0x01);
  boot_phy_d436_width(0x0F);
}

/* Full early PHY bring-up sequence. */
static void boot_phy_bringup_early(void) {
  uint8_t ltssm = REG_LTSSM_CTRL;
  if (((ltssm >> 1) & 1) || ((ltssm >> 2) & 1)) {
    boot_phy_d0d3_typec_sbu();
  }
  boot_phy_cf28();
  boot_phy_bank1_ed02();
  REG_PHY_CONFIG &= 0xFC;
  REG_PHY_CONFIG = (REG_PHY_CONFIG & 0xFB) | 0x04;
  phy_cc10_cmd_wait(2, 0, 0x14);
  REG_PHY_CONFIG &= 0xFB;
  phy_cc10_cmd(3, 0, 0x0A);
  { uint16_t spin = 0;
    while (!((REG_LINK_STATUS_E712 & 0x03) || ((REG_TIMER0_CSR >> 1) & 1)) && ++spin < 0xFFFF); }
  phy_cc11_ack();
  boot_phy_dd42(0);
  boot_phy_d996_pcie_tunnel_boot();
}

/* Seed the lane-engine link WIDTH/MODE/state RAM the USB4 tunnel reads (non-OTP path). */
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
  XDATA_REG8(0x0AE9) = 0x0F;
  XDATA_REG8(0x0AEE) = 3;
  XDATA_REG8(0x0AEF) = 3;
  XDATA_REG8(0x0AEB) = 3;
  XDATA_REG8(0x0AEC) = 3;
  XDATA_REG8(0x0AED) = 3;
  XDATA_REG8(0x0A83) = 0;
  XDATA_REG8(0x0AEB) = XDATA_REG8V(0x0AEB) | 0x01;
  XDATA_REG8(0x0AF1) = 0x00;
  REG_PHY_CFG_C65A &= 0xF7;
  REG_CPU_EXEC_STATUS_3 &= 0xFB;
  REG_USB_EP_CTRL_905F &= 0xEF;
}

#endif /* BOOT_PHY_H */
