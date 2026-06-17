#ifndef USB4_IRQ_H
#define USB4_IRQ_H
/*
 * USB4 SB-transport / router interrupt arming (init_sys_flags bank1 tail not covered by
 * pd_int1_enable_group): enables the SB-PHY RX path that detects host sideband connect packets.
 * Included AFTER sb.h and pd.h.
 */

/* page-N (DPX=1) XDATA RMW helper for the PHY register banks. */
static uint8_t PG_RD(uint16_t addr) {
  uint8_t v; DPX = 0x01; v = XDATA_REG8V(addr); DPX = 0x00; return v;
}
static void PG_WR(uint16_t addr, uint8_t v) {
  DPX = 0x01; XDATA_REG8V(addr) = v; DPX = 0x00;
}

#include "usb4_rx_table.h"

/* PHY-config helper RMWs; each operates on a single XDATA reg. */
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
static void C335(uint16_t reg) { RMW(reg, 0x0F, 0xE0); RMW(reg + 1, 0x0F, 0x70); }
static void C30E(uint16_t reg) { PR(reg) &= 0xFE; PR(reg) &= 0xFD; PR(reg) &= 0xFB; PR(reg) &= 0xF7; }
static void C2D9(uint16_t reg) { RMW(reg, 0x0F, 0x60); RMW(reg + 1, 0xF0, 0x07); }
static void C397(uint16_t reg) { RMW(reg, 0xF1, 0x0E); PR(reg + 1) = 0; }

/* Full PHY-RX descriptor config: dual PHY lanes + page-0x93 + SB lane regs. */
static void usb4_phy_rx_descriptor_8e31(void) {
  REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0xF8) | 0x03;
  REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0xC7) | 0x28;
  REG_PHY_PLL_CFG = (REG_PHY_PLL_CFG & 0xFC) | 0x03;
  REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0x3F) | 0x80;
  REG_PHY_PLL_CFG &= 0xF7;
  REG_CPU_CLK_CFG = (REG_CPU_CLK_CFG & 0x1F) | 0x80;
  C390(0xC21F);
  SB_WR(0x49, 0xA0);

  REG_PHY_LANEA_RATE_START_C2A8 = (REG_PHY_LANEA_RATE_START_C2A8 & 0x3F) | 0x40;
  C34A(0xC2C5);
  REG_PHY_LANEA_C2A1 = (REG_PHY_LANEA_C2A1 & 0x9F) | 0x60;
  C2F8(0xC28C); C2F8(0xC29C); C2F8(0xC2AC);
  REG_PHY_LANEA_C2BC &= 0xFE;
  REG_PHY_LANEA_C28C &= 0xFD;
  C351(0xC29C); C351(0xC2AC);
  REG_PHY_LANEA_C2BC &= 0xFD;
  REG_PHY_ORIENT_C2C3 = (REG_PHY_ORIENT_C2C3 & 0xC3) | 0x1C;
  REG_PHY_LANEA_RATE_DESC_C2C9 = (REG_PHY_LANEA_RATE_DESC_C2C9 & 0x80) | 0x41;
  C335(0xC2A5);
  C30E(0xC2CA);
  C358(0xC287);
  C34A(0xC294);
  C358(0xC2A2);
  C2F1(0xC2C5);
  C2FF(0xC293);
  C2BF(0xC2CE);
  REG_PHY_LANEA_C2CE = (REG_PHY_LANEA_C2CE & 0xE3) | 0x14;
  REG_PHY_LANEB_RATE_START_C328 = (REG_PHY_LANEB_RATE_START_C328 & 0x3F) | 0x40;

  C34A(0xC345);
  REG_PHY_LANEB_C321 = (REG_PHY_LANEB_C321 & 0x9F) | 0x60;
  C2F8(0xC30C); C2F8(0xC31C); C2F8(0xC32C);
  REG_PHY_LANEB_C33C &= 0xFE;
  REG_PHY_LANEB_C30C &= 0xFD;
  C351(0xC31C); C351(0xC32C);
  REG_PHY_LANEB_C33C &= 0xFD;
  REG_VENDOR_CTRL_C343 = (REG_VENDOR_CTRL_C343 & 0xC3) | 0x1C;
  REG_PHY_LANEB_RATE_DESC_C349 = (REG_PHY_LANEB_RATE_DESC_C349 & 0x80) | 0x41;
  C335(0xC325);
  C30E(0xC34A);
  C358(0xC307);
  C34A(0xC314);
  C358(0xC322);
  C2F1(0xC345);
  C2FF(0xC313);
  C2BF(0xC34E);
  REG_PHY_LANEB_C34E = (REG_PHY_LANEB_C34E & 0xE3) | 0x14;
  REG_PHY_LINK_CTRL_C21D = (REG_PHY_LINK_CTRL_C21D & 0x3F) | 0x80;

  REG_BUF_DESC_STAT0_HI = 0; REG_BUF_DESC_STAT0_LO = 0;
  REG_BUF_DESC_STAT1_HI = 0; REG_BUF_DESC_STAT1_LO = 0;
  REG_BUF_DESC_STAT2_HI = 0; REG_BUF_DESC_STAT2_LO = 0;

  REG_PHY_LANEA_C290 &= 0x9F;
  REG_PHY_LANEA_C2A0 &= 0x9F;
  C35F(0xC282);
  REG_PHY_LANEA_C292 = (REG_PHY_LANEA_C292 & 0xE0) | 0x09;
  C35F(0xC2A2);
  C366(0xC290);
  C366(0xC2A0);
  C36D(0xC291);
  C36D(0xC2A1);
  REG_PHY_LANEA_C2DB = (REG_PHY_LANEA_C2DB & 0xE0) | 0x1B;
  REG_PHY_STATUS = (REG_PHY_STATUS & 0xF0) | 0x05;
  C2E0(0xC294);
  C2E7(0xC285);
  REG_PHY_LANEA_C295 = (REG_PHY_LANEA_C295 & 0xF0) | 0x0C;
  C2E7(0xC2A5);
  C2D9(0xC285);
  C2E7(0xC296);
  C374(0xC2A7);
  REG_PHY_LANEA_C28B = (REG_PHY_LANEA_C28B & 0xC0) | 0x0A;
  REG_PHY_STATUS = (REG_PHY_STATUS & 0x8F) | 0x40;
  REG_PHY_LANEA_C2A4 &= 0x8F;
  REG_PHY_LANEA_C289 = (REG_PHY_LANEA_C289 & 0x0F) | 0x90;
  C37B(0xC299);
  C37B(0xC2A9);
  REG_PHY_LANEA_C282 = (REG_PHY_LANEA_C282 & 0x1F) | 0xA0;
  REG_PHY_LANEA_C292 = (REG_PHY_LANEA_C292 & 0x1F) | 0x20;
  C382(0xC2C6);
  C397(0xC2CC);

  REG_PHY_LANEB_C310 &= 0x9F;
  REG_PHY_LANEB_C320 &= 0x9F;
  C35F(0xC302);
  REG_PHY_LANEB_C312 = (REG_PHY_LANEB_C312 & 0xE0) | 0x09;
  C35F(0xC322);
  C366(0xC310);
  C366(0xC320);
  C36D(0xC311);
  C36D(0xC321);
  REG_PHY_LANEB_C35B = (REG_PHY_LANEB_C35B & 0xE0) | 0x1B;
  REG_PHY_LANEB_C304 = (REG_PHY_LANEB_C304 & 0xF0) | 0x05;
  C2E0(0xC314);
  C2E7(0xC305);
  REG_PHY_LANEB_C315 = (REG_PHY_LANEB_C315 & 0xF0) | 0x0C;
  C2E7(0xC325);
  C2D9(0xC305);
  C2E7(0xC316);
  C374(0xC327);
  REG_PHY_LANEB_C30B = (REG_PHY_LANEB_C30B & 0xC0) | 0x0A;
  REG_PHY_LANEB_C304 = (REG_PHY_LANEB_C304 & 0x8F) | 0x40;
  REG_PHY_LANEB_C324 &= 0x8F;
  REG_PHY_LANEB_C309 = (REG_PHY_LANEB_C309 & 0x0F) | 0x90;
  C37B(0xC319);
  C37B(0xC329);
  REG_PHY_LANEB_C302 = (REG_PHY_LANEB_C302 & 0x1F) | 0xA0;
  REG_PHY_LANEB_C312 = (REG_PHY_LANEB_C312 & 0x1F) | 0x20;
  C382(0xC346);
  C397(0xC34C);

  REG_BUF_DESC_BASE0_HI = 0x01;
  REG_BUF_DESC_BASE0_LO = 0x60; REG_BUF_DESC_SIZE0_HI = 0x00;
  REG_BUF_DESC_SIZE0_LO = 0xE3;
  REG_BUF_DESC_BASE1_HI = 0x01;
  REG_BUF_DESC_BASE1_LO = 0x60;
  REG_BUF_DESC_BASE2_HI = 0x01;
  REG_BUF_DESC_BASE2_LO = 0x60;
  REG_BUF_DESC_CFG0_HI = 0x00;
  REG_BUF_DESC_CFG0_LO = 0x03; REG_BUF_DESC_CFG1_HI = 0x00;
  REG_BUF_DESC_CFG1_LO = 0xE0;
  REG_BUF_DESC_CFG2_HI = 0x00;
  REG_BUF_DESC_CFG2_LO = 0xE3;

  C2FF(0xC2A3);
  C2FF(0xC323);
  C389(0xC297);
  REG_PHY_LANEA_C29A = (REG_PHY_LANEA_C29A & 0xF0) | 0x0E;
  C389(0xC2A7);
  REG_PHY_LANEA_C2AB &= 0xC0;
  C389(0xC317);
  REG_PHY_LANEB_C31A = (REG_PHY_LANEB_C31A & 0xF0) | 0x0E;
  C389(0xC327);
  REG_PHY_LANEB_C32B &= 0xC0;
  C382(0xC2AA);
  REG_PHY_LANEA_LOCK_C297 = (REG_PHY_LANEA_LOCK_C297 & 0xE0) | 0x10;
  REG_PHY_LANEA_C293 = (REG_PHY_LANEA_C293 & 0xFC) | 0x01;
  C2FF(0xC283);
  C2F1(0xC2A6);
  C2E0(0xC2A4);
  C2BF(0xC2A3);
  REG_PHY_LANEA_C29B &= 0xC0;

  C382(0xC32A);
  REG_PHY_LANEB_LOCK_C317 = (REG_PHY_LANEB_LOCK_C317 & 0xE0) | 0x10;
  REG_PHY_LANEB_C313 = (REG_PHY_LANEB_C313 & 0xFC) | 0x01;
  C2FF(0xC303);
  C2F1(0xC326);
  C2E0(0xC324);
  C2BF(0xC323);
  REG_PHY_LANEB_C31B &= 0xC0;

  if (REG_LANE_RATE_C8FF >= 0x05) {
    REG_PHY_LANEA_C294 = (REG_PHY_LANEA_C294 & 0xF0) | 0x06;
    C374(0xC297);
    REG_PHY_LANEB_C314 = (REG_PHY_LANEB_C314 & 0xF0) | 0x06;
    C374(0xC317);
  }
  if (REG_LANE_RATE_C8FF >= 0x06) {
    REG_PHY_LANEA_C283 &= 0xF3;
    REG_PHY_LANEB_C303 &= 0xF3;
  }
}

/* PHY link/SB sideband setup producing SB[0x1C]=0xC2. */
static void usb4_irq_db0d(void) {
  REG_PHY_LINK_CTRL_C21B = (REG_PHY_LINK_CTRL_C21B & 0x3F) | 0xC0;
  REG_LINK_CTRL = (REG_LINK_CTRL & 0xF7) | 0x08;
  PG_WR(0x1262, PG_RD(0x1262) & 0xEF);
  SB_WR(0xED, (SB_RD(0xED) & 0xBF) | 0x40);
  SB_WR(0xCE, SB_RD(0xCE) & 0xFE);
  SB_SET(0x1C, 0x80);
  SB_SET(0x1C, 0x40);
  SB_SET(0x1C, 0x02);
  REG_PHY_LINK_CTRL_C20B &= 0x7F;
  SB_WR(0x1D, SB_RD(0x1D) & 0xFE);
  C390(0xC22F);
  REG_PHY_SERDES_C22F &= 0xBF;
}

/* SB-PHY 4-lane RX arm: applies the paged RX equalizer/rate RMW table byte-exact. */
static void usb4_irq_ef1e(void) {
  uint16_t i;
  for (i = 0; i < (uint16_t)(sizeof(u4rx_tab) / 4); i++) {
    uint16_t addr = ((uint16_t)u4rx_tab[i][0] << 8) | u4rx_tab[i][1];
    PG_WR(addr, (uint8_t)((PG_RD(addr) & u4rx_tab[i][2]) | u4rx_tab[i][3]));
  }
}

/* PHY link setup + RX descriptor config. */
static void usb4_irq_ef24(void) {
  usb4_irq_db0d();
  usb4_phy_rx_descriptor_8e31();
}

/* init_sys_flags arming not covered by pd_int1_enable_group; call once at boot. */
static void usb4_irq_arm(void) {
  REG_CPU_EXEC_STATUS_3 &= 0xFE;
  REG_TIMER_CTRL_CC3B = (REG_TIMER_CTRL_CC3B & 0xFD) | 0x02;
  usb4_irq_ef24();
  usb4_irq_ef1e();
}

/* phy_cc10_cmd_wait is defined in boot_phy.h (included after this file). */
static void phy_cc10_cmd_wait(uint8_t subcmd, uint8_t cc12, uint8_t cc13);

/* USB4 CM router-op RX-enable: enables the router-op engine and SB-transport RX. */
static void usb4_routerop_init(void) {
  REG_ROUTEROP_ENGINE_CTRL_EC00 &= 0xFE;
  phy_cc10_cmd_wait(0, 9, 0);
  REG_ROUTEROP_ENGINE_CTRL_EC00 = (REG_ROUTEROP_ENGINE_CTRL_EC00 & 0xFE) | 0x01;
  REG_ROUTEROP_SPEED_LO_EA88 = 100;
  REG_ROUTEROP_SPEED_HI_EA89 = 0x24;
  REG_NVME_EVENT_ACK = 1;
  REG_ROUTEROP_CFG_EC05 &= 0xFE;
  REG_INT_DMA_CTRL &= 0xBF;
  REG_INT_DMA_CTRL = (REG_INT_DMA_CTRL & 0x7F) | 0x80;
  u4_routerop_mbox_state = RMBOX_IDLE;
}

#endif /* USB4_IRQ_H */
