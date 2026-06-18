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
  while (!((REG_TIMER0_CSR >> 1) & 1)) { }
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
    /* d630 byte-true (cc69 returns R7==1 -> ACC.1/.2/.3 of 1 are all 0 -> every bit written is 0):
     * stock CLEARS E76C.4, E774.4, AND E77C.4. Handmade had omitted the first two and wrongly SET
     * E77C.4 (these are PCIe-tunnel PLL-latch bits; a mis-latched PLL never reaches a clean lock). */
    XDATA_REG8(0xE76C) = (uint8_t)(XDATA_REG8V(0xE76C) & 0xEF);   /* E76C.4 = 0 */
    XDATA_REG8(0xE774) = (uint8_t)(XDATA_REG8V(0xE774) & 0xEF);   /* E774.4 = 0 */
    REG_SYS_CTRL_E77C  = (uint8_t)(REG_SYS_CTRL_E77C & 0xEF);     /* E77C.4 = 0 (was wrongly set to 1) */
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
  /* One-shot OTP/strap probe: stock 92c5 reads fuses ONLY if 0x707E==0x5A (+ checksum 0x707F).
   * If fused, it overrides lane mask (0x0AE9), lane/gen (u4_link_lane/0x0AEE), width (0x086C-71)
   * from 0x707A/0x707B/0x707D. Handmade omits this -> if the board is fused, our defaults diverge. */
  { uart_puts("\r\n[OTP 7E="); uart_puthex(XDATA_REG8V(0x707E));
    uart_puts(" 7F="); uart_puthex(XDATA_REG8V(0x707F));
    uart_puts(" 7A="); uart_puthex(XDATA_REG8V(0x707A));
    uart_puts(" 7B="); uart_puthex(XDATA_REG8V(0x707B));
    uart_puts(" 7D="); uart_puthex(XDATA_REG8V(0x707D));
    uart_puts(" 74="); uart_puthex(XDATA_REG8V(0x7074)); uart_puthex(XDATA_REG8V(0x7075));
    uart_puthex(XDATA_REG8V(0x7076)); uart_puthex(XDATA_REG8V(0x7077));
    uart_puthex(XDATA_REG8V(0x7078)); uart_puthex(XDATA_REG8V(0x7079)); uart_putc(']'); }
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

/* c8db: PCIe-tunnel adapter capability/path config (B410-B42B) from 0x0A52/53/54/55.
 * cc75(addr,hi,lo): [addr]=lo,[addr+1]=hi.  cc80(addr): [addr]=06,[addr+1]=04,[addr+2]=00. */
static void pcie_tunnel_adapter_config_b410(void) {
  uint8_t lo = u4_tunnel_cfg_lo, hi = u4_tunnel_cfg_hi, mode = u4_tunnel_cfg_mode, cred = u4_tunnel_credits;
  REG_TUNNEL_CFG_A_LO = lo;                              /* B410 = [0A53] */
  XDATA_REG8(0xB411) = hi;                               /* B411 = [0A52] */
  REG_TUNNEL_DATA_LO = lo; XDATA_REG8(0xB421) = hi;      /* cc75(B420,hi,lo) */
  REG_TUNNEL_CREDITS = cred;                             /* B412 = [0A55] */
  REG_TUNNEL_CFG_MODE = mode;                            /* B413 = [0A54] */
  XDATA_REG8(0xB422) = lo;                               /* B422 = [0A53] */
  XDATA_REG8(0xB423) = mode;                             /* B423 = [0A54] */
  XDATA_REG8(0xB415) = 6; XDATA_REG8(0xB416) = 4; XDATA_REG8(0xB417) = 0;  /* cc80(B415) */
  XDATA_REG8(0xB425) = 6; XDATA_REG8(0xB426) = 4; XDATA_REG8(0xB427) = 0;  /* cc80(B425) */
  XDATA_REG8(0xB41A) = lo;                               /* B41A = [0A53] */
  XDATA_REG8(0xB41B) = hi;                               /* B41B = [0A52] */
  XDATA_REG8(0xB42A) = lo; XDATA_REG8(0xB42B) = hi;      /* cc75(B42A,hi,lo) */
  REG_TUNNEL_PATH_CREDITS = cred;                        /* B418 = [0A55] */
  XDATA_REG8(0xB419) = mode;                             /* B419 = [0A54] */
  XDATA_REG8(0xB428) = lo;                               /* B428 = [0A53] */
  XDATA_REG8(0xB429) = mode;                             /* B429 = [0A54] */
}

/* cd6c: PCIe-over-USB4 TUNNEL ADAPTER ENABLE (master tunnel-up). Byte-true to stock CODE:cd6c-cdc5.
 * Stock runs this in boot_hw_init_main step 6k so the host CM can discover a PCIe-down adapter and
 * tunnel PCIe to the GPU; it is re-run at runtime via bank0_c00d on tunnel-up. */
static void pcie_tunnel_adapter_enable_b401(void) {
  REG_CPU_MODE_NEXT &= 0xEF;                             /* CA06 &= ~0x10 */
  pcie_tunnel_adapter_config_b410();
  P1_WR(0x4084, 0x22);                                   /* port0 PHY cfg mailbox */
  P1_WR(0x5084, 0x22);                                   /* port1 PHY cfg mailbox (val 0x22, NOT 0x50) */
  REG_PCIE_TUNNEL_CTRL |= 0x01;                          /* B401 |= 1 (TUNNEL MASTER ENABLE) */
  REG_TUNNEL_ADAPTER_MODE |= 0x01;                       /* B482 |= 1 */
  REG_TUNNEL_ADAPTER_MODE = (uint8_t)((REG_TUNNEL_ADAPTER_MODE & 0x0F) | 0xF0);
  REG_PCIE_TUNNEL_CTRL &= 0xFE;                          /* B401 &= ~1 ... */
  REG_PCIE_PERST_CTRL = (uint8_t)((REG_PCIE_PERST_CTRL & 0xFE) | 0x01);  /* ...then B480 |= 1 (PERST) */
  REG_TUNNEL_LINK_STATE &= 0xFE;                         /* B430 &= ~1 (link down during cfg) */
  REG_PCIE_TUNNEL_CFG = (uint8_t)((REG_PCIE_TUNNEL_CFG & 0xEF) | 0x10);  /* B298 TLP-routing bit4 */
  P1_WR(0x6043, 0x70);
  P1_WR(0x6025, (uint8_t)((P1_RD(0x6025) & 0x7F) | 0x80));
}

#endif /* BOOT_PHY_H */
