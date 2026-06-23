#ifndef USB4_CONNECT_H
#define USB4_CONNECT_H
/*
 * USB4 PHY / lane-mode bring-up engine. Include after usb4.h and before sb_router.h.
 */

/* single-register read-modify-write bit helpers */
#define U4C_BD23(a)   PR(a) = (PR(a) & 0xDF) | 0x20
#define U4C_BD3A(a)   PR(a) = (PR(a) & 0xBF) | 0x40
#define U4C_BD65(a)   PR(a) = (PR(a) & 0x7F) | 0x80
#define U4C_BCFE(a)   PR(a) = (PR(a) & 0xFD) | 0x02
#define U4C_BCEB(a)   PR(a) = (PR(a) & 0xFE) | 0x01
#define U4C_BD5E(a)   PR(a) = (PR(a) & 0xFB) | 0x04
static void u4c_bd2a(uint16_t a) { PR(a) &= 0xDF; PR(a) &= 0xBF; }
static void u4c_bcf2(void) { REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xFD) | 0x02; REG_TIMER_ENABLE_A = (REG_TIMER_ENABLE_A & 0xFD) | 0x02; }
static void u4c_bd41(void) { REG_TIMER_CTRL_CC3B &= 0xFD; }
static void u4c_bd14(void) { REG_TIMER_ENABLE_B &= 0xFD; REG_TIMER_ENABLE_A &= 0xFD; }

/* pump the link controller, then return the current link-mode byte */
static uint8_t u4c_bd6c(void) {
  REG_CPU_LINK_CTRL_CA00 = (REG_CPU_LINK_CTRL_CA00 & 0xC0) | 0x07;
  REG_CPU_LINK_GO_CA0A = 0x02;
  return u4_routerop_desc0;
}

/* bounded PHY-lock wait */
static void u4c_e7ae_bounded(void) {
  uint16_t g = 0;
  while (((REG_UART_TFBF & 0x1F) != 0x10) && ++g < 0x0800);
  g = 0;
  while (((REG_UART_STATUS & 0x07) != 0x00) && ++g < 0x0800);
}

/* set when the engine has run; read from the super-loop */
static volatile uint8_t __xdata __at(0x0B51) bank0_8a89_entered;

/* USB4 lane-mode bring-up engine. mode: 0, 1=USB3.2-tunnel, 2=USB4. */
static void bank0_8a89(uint8_t mode) {
  uint8_t config;
  uint8_t link_mode;
  uint8_t descr_arg;

  bank0_8a89_entered = 1;
  u4_routerop_desc0 = mode;
  uart_puts("[8a89:");
  uart_puthex(mode);
  uart_putc(']');

  /* pick the config byte by the lane-rate latch */
  if (REG_LANE_RATE_C8FF < 0x06) {
    u4_routerop_desc3 = 0x0A;
  } else {
    u4_routerop_desc3 = 0x0B;
    U4C_BCEB(0xCC37);
    U4C_BCEB(0xCC36);
    REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xF7) | 0x08;
  }

  link_mode = u4_routerop_desc0;
  if (link_mode < 3) {
    boot_phy_dd42((uint8_t)(link_mode - 3));
    REG_PHY_TIMER_CTRL_E764 &= 0xEF;
    u4c_e7ae_bounded();
    U4C_BCEB(0xCA81);

    /* pre-arm latch of the link rate into 0x0A9F / 0x0A9E */
    u4_routerop_desc2 = REG_LINK_WIDTH_E710 & 0x1F;
    u4_routerop_desc2 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    u4_routerop_desc1 = REG_CPU_MODE_NEXT >> 5;
    u4_routerop_desc1 = (REG_CPU_MODE_NEXT & 0x1F) | 0x80;

    config = u4_routerop_desc3;
    if (config & 0x01) {
      U4C_BD23(0xE40B);
      U4C_BD23(0xC698);
      u4c_bd14();
      U4C_BCEB(0xCAC4);
      REG_PHY_POLL_E751 = 0x01;
      U4C_BD65(0xE313);
      U4C_BCFE(0xE413);
    }

    /* first mode dispatch */
    if (u4_routerop_desc0 == 0x02) {
      if (config & 0x02) {
        u4_routerop_desc0 = (REG_LINK_STATUS_E716 & 0xFC);
        U4C_BCFE(0xCC3E);
        REG_LINK_CTRL_E717 &= 0xFE;
      }
      u4c_e7ae_bounded();
      P1_WR(0x011F, 0x01);
      if (config & 0x02) u4c_bcf2();
    } else if (u4_routerop_desc0 == 0x01) {
      if (config & 0x02) u4c_bd41();
      u4c_e7ae_bounded();
      REG_POWER_DOMAIN = 0x01;
      if (config & 0x02) { U4C_BD3A(0xCC3B); U4C_BD5E(0xCC37); }
    } else {
      u4c_e7ae_bounded();
      REG_USB_EP_CTRL_91D0 = 0x01;
    }

    phy_cc10_cmd_wait(0, 0x27, 2);
    u4_connect_oneshot_suppress = 0x01;

    if (u4c_bd6c() != 0 && (config & 0x08)) {
      U4C_BCFE(0xCC3F);
      U4C_BD5E(0xCC3F);
      u4c_bd2a(0xCC3F);
      U4C_BD65(0xCC3D);
    }

    /* link-up service loop: pump PD-RX while the lanes come up */
    { uint32_t guard = 0;
      while (u4c_bd6c() != 0 && ++guard < 200000UL) {
        if (REG_INT_PCIE_NVME & 0x40) pd_rx_isr();
        if ((u4_link_busy == 0 && (REG_CPU_EXEC_STATUS_2 >> 2 & 1)) || u4_connect_oneshot_suppress == 0) break;
      }
    }

    if ((config & 0x08) && ((int8_t)REG_LTSSM_STATE < 0)) {
      boot_phy_d0d3_typec_sbu();
    }
    config = u4_routerop_desc3;
    if (config & 0x01) {
      REG_CMD_ARM_CAC4 &= 0xFE;
      REG_CMD_CONFIG &= 0xDF;
      u4c_bd2a(0xC698);
      REG_CMD_LINK_ARM_E313 &= 0x7F;
      REG_CMD_CFG_E413 &= 0xFD;
    }

    /* enable link-mode and re-latch the final link rate */
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xEF) | 0x10;
    u4_routerop_desc2 |= (REG_LINK_WIDTH_E710 & 0xE0);
    u4_routerop_desc1 = (REG_CPU_MODE_NEXT & 0x1F) | (uint8_t)(u4_routerop_desc1 << 5);
    REG_CPU_CTRL_CA81 &= 0xFE;

    /* second mode dispatch selecting the descriptor arg */
    if (u4_routerop_desc0 == 0x02) {
      if (config & 0x02) {
        u4_routerop_desc0 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
        u4_routerop_desc1 = (REG_CPU_CTRL_CC3E & 0xFD);
        U4C_BCEB(0xCA81);
        u4c_bd14();
      }
      descr_arg = (u4_route_mode >> 1 & 1) ? 2 : 1;
    } else if (u4_routerop_desc0 == 0x01) {
      if (config & 0x02) {
        { uint16_t g = 0; while (!(REG_CPU_LINK_DONE_CD4E & 1) && ++g < 0x4000);
          g = 0; while (!(REG_CPU_LINK_DONE_CD4E & 2) && ++g < 0x4000); }
        { uint16_t g = 0;
          while (!((REG_LINK_STATUS_E712 & 1) || (REG_LINK_STATUS_E712 & 2)) && ++g < 0x4000); }
        REG_TIMER_CTRL_CC3B &= 0xBF;
        phy_cc10_cmd_wait(0, 0x13, 2);
        REG_CPU_CTRL_CC37 &= 0xFB;
        U4C_BCFE(0xCC3B);
      }
      descr_arg = 4;
    } else {
      descr_arg = 4;
    }
    boot_phy_dd42(descr_arg);

    /* drive the connect unless the one-shot suppress is set */
    if (u4_connect_oneshot_suppress == 0) {
      uint8_t cur_mode = u4_routerop_desc0;
      if (cur_mode == 0) {
        REG_CPU_CTRL_CC3E = (REG_CPU_CTRL_CC3E & 0xFD) | 0x02;
        REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;
        REG_CPU_CTRL_CC36 &= 0xDF;
      }
      U4C_BD23(0x92C4);
      if (cur_mode == 0) {
        REG_POWER_MISC_CTRL = (REG_CPU_CTRL_CC3E & 0xFD);
        U4C_BD23(0xCC36);
      }
      REG_POWER_MISC_CTRL &= 0xDF;
      usb4_connect_u4();
    }
    u4_connect_oneshot_suppress = 0x00;

    /* On link-complete, ack the tunnel/link transition. */
    if (u4_link_busy == 0 && (REG_CPU_EXEC_STATUS_2 >> 2 & 1)) {
      REG_CPU_EXEC_STATUS_2 = 0x04;
    }
    uart_puts("[8a89done]");
  }
}

/* host-link-event connect dispatcher; runs the lane-mode engine when the connect gate passes */
static void bank0_c9a8(uint8_t mode) {
  u4_lane_mode_arg = mode;
  if (u4_route_mode & 0x04) {
    if ((u4_connect_gate & 0x01) &&
        (u4_connect_gate_e8 != 0 || u4_connect_gate_eb != 0)) {
      REG_PHY_CFG_C6A8 &= 0xFE;
      bank0_8a89(u4_lane_mode_arg);
    }
    u4_connect_gate_e8 = 0x00;
    u4_reinit_pending = 0x01;
    return;
  }
}

#endif /* USB4_CONNECT_H */
