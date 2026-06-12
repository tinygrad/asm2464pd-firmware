#ifndef USB4_CONNECT_H
#define USB4_CONNECT_H
/*
 * bank0_8a89 @0x8A89 — USB4 PHY / lane-MODE bring-up engine. Faithful transcription
 * of the ASM2464PD stock firmware (fw_tinygrad.bin). Arms E764.4 (LINK-MODE-ENABLE)
 * and E751 (USB4 link arm), latches the E710/CA06 link-rate into 0x0A9F/0x0A9E, writes
 * the per-mode E7E3 descriptor via dd42(), and runs a dynamic service loop that pumps
 * PD-RX while bringing up the lanes (waiting on CC33.2 / 0x07ED).
 *
 * Driven per host link-event from the INT0 demux (0e5b -> e94d/e952 -> c9a8 -> 8a89).
 * Include AFTER usb4.h and BEFORE sb_router.h.
 *
 * Stock helper map (byte-exact):
 *   bd49() = E710 & 0xE0          bd57() = CA06 & 0x1F        bd50() = E716 & 0xFC
 *   bd33() = CC3E & 0xFD          bceb(a): a |= 1             bcfe(a): a = (&0xFD)|2
 *   bd23(a): a = (&0xDF)|0x20     bd3a(a): a = (&0xBF)|0x40   bd65(a): a = (&0x7F)|0x80
 *   bd2a(a): a &= 0xDF; a &= 0xBF                             bd5e(a): a = (&0xFB)|0x04
 *   bce7(v,a): *a = v; E717 = (E717&0xFE)|1
 *   bcf2(): CC3A=(&0xFD)|2; CC38=(&0xFD)|2   bd41(): CC3B &= 0xFD   bd14(): CC3A&=0xFD; CC38&=0xFD
 *   8d6e(): uart_puts; e7ae() PHY-lock busy wait (bounded here)
 *   bd6c(): dcd4(0x0A9D); return 0x0A9D       dcd4(m): CA00=(&0xC0)|7; CA0A=2; CA0D/CA0E poll
 *   af5e() == pd_rx_isr()   dd42() == boot_phy_dd42()   cd10() == bank0_cd10 (handoff)
 */

/* tiny single-reg RMW helpers from bank0_8a89's helper block (verbatim) */
#define U4C_BD23(a)   PR(a) = (PR(a) & 0xDF) | 0x20      /* set bit5 */
#define U4C_BD3A(a)   PR(a) = (PR(a) & 0xBF) | 0x40      /* set bit6 */
#define U4C_BD65(a)   PR(a) = (PR(a) & 0x7F) | 0x80      /* set bit7 */
#define U4C_BCFE(a)   PR(a) = (PR(a) & 0xFD) | 0x02      /* set bit1 */
#define U4C_BCEB(a)   PR(a) = (PR(a) & 0xFE) | 0x01      /* set bit0 */
#define U4C_BD5E(a)   PR(a) = (PR(a) & 0xFB) | 0x04      /* set bit2 */
static void u4c_bd2a(uint16_t a) { PR(a) &= 0xDF; PR(a) &= 0xBF; }   /* clear bits 5,6 */
static void u4c_bcf2(void) { REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xFD) | 0x02; REG_TIMER_ENABLE_A = (REG_TIMER_ENABLE_A & 0xFD) | 0x02; }
static void u4c_bd41(void) { REG_TIMER_CTRL_CC3B &= 0xFD; }
static void u4c_bd14(void) { REG_TIMER_ENABLE_B &= 0xFD; REG_TIMER_ENABLE_A &= 0xFD; }

/* bd6c: pump the link controller (dcd4 = CA00=(&0xC0)|7; CA0A=2) then return mode byte 0x0A9D. */
static uint8_t u4c_bd6c(void) {
  PR(0xCA00) = (PR(0xCA00) & 0xC0) | 0x07;       /* dcd4 */
  PR(0xCA0A) = 0x02;
  return PR(0x0A9D);
}

/* e7ae: PHY-lock wait — C006[4:0]==0x10 then C00E[2:0]==0 (bounded so it can't hang the ISR). */
static void u4c_e7ae_bounded(void) {
  uint16_t g = 0;
  while (((PR(0xC006) & 0x1F) != 0x10) && ++g < 0x0800);
  g = 0;
  while (((PR(0xC00E) & 0x07) != 0x00) && ++g < 0x0800);
}

/* indicates the engine ran (instrumentation, read from the super-loop). XDATA scratch, seeded in main(). */
static volatile uint8_t __xdata __at(0x0B51) bank0_8a89_entered;

/* bank0_8a89 @0x8A89. param = USB4 link mode (0=?,1=USB3.2-tunnel,2=USB4). */
static void bank0_8a89(uint8_t mode) {
  uint8_t aa0;          /* 0x0AA0 config byte */
  uint8_t a9d;          /* 0x0A9D mode (mutated by the dispatch) */
  uint8_t uVar2;        /* the dd42 descriptor arg for the second dispatch */

  bank0_8a89_entered = 1;
  PR(0x0A9D) = mode;                              /* xdata_00a9d = param */
  uart_puts("[8a89:");
  uart_puthex(mode);
  uart_putc(']');

  /* head: pick 0x0AA0 config by the lane-rate latch C8FF */
  if (REG_LANE_RATE_C8FF < 0x06) {
    PR(0x0AA0) = 0x0A;
  } else {
    PR(0x0AA0) = 0x0B;
    U4C_BCEB(0xCC37);                             /* bceb(cc37): set bit0 */
    U4C_BCEB(0xCC36);                             /* bceb(cc36): set bit0 */
    REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xF7) | 0x08;      /* CC3A bit3 set */
  }

  a9d = PR(0x0A9D);
  if (a9d < 3) {                                  /* the WHOLE engine gates on mode < 3 */
    boot_phy_dd42((uint8_t)(a9d - 3));            /* dd42(0x0A9D - 3) -> E7E3 per-mode */
    REG_PHY_TIMER_CTRL_E764 &= 0xEF;                           /* E764 LINK-MODE clear (head) */
    u4c_e7ae_bounded();                           /* e7ae PHY-lock wait */
    U4C_BCEB(0xCA81);                             /* bceb(CA81): set bit0 */

    /* latch link-rate into 0x0A9F / 0x0A9E (pre-arm):
     *   0x0A9F = (E710 & 0xE0) | 0x1F   [bd49()]
     *   0x0A9E = (CA06 & 0x1F) | 0x80   [bd57()] */
    PR(0x0A9F) = REG_LINK_WIDTH_E710 & 0x1F;
    PR(0x0A9F) = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    PR(0x0A9E) = REG_CPU_MODE_NEXT >> 5;
    PR(0x0A9E) = (REG_CPU_MODE_NEXT & 0x1F) | 0x80;

    aa0 = PR(0x0AA0);
    if (aa0 & 0x01) {                             /* 0x0AA0.0 -> arm E751 (USB4 link arm) */
      U4C_BD23(0xE40B);                            /* bd23(E40B): set bit5 */
      U4C_BD23(0xC698);                            /* bd23(C698): set bit5 */
      u4c_bd14();                                  /* bd14: CC3A/CC38 &=0xFD */
      U4C_BCEB(0xCAC4);                            /* bceb(CAC4) */
      REG_PHY_POLL_E751 = 0x01;                           /* E751 = 1 (USB4 LINK ARM) */
      U4C_BD65(0xE313);                            /* bd65(E313): set bit7 */
      U4C_BCFE(0xE413);                            /* bcfe(E413): set bit1 */
    }

    /* FIRST mode dispatch */
    if (PR(0x0A9D) == 0x02) {
      if (aa0 & 0x02) {                            /* 0x0AA0.1 */
        PR(0x0A9D) = (REG_LINK_STATUS_E716 & 0xFC);          /* 0x0A9D = bd50() */
        U4C_BCFE(0xCC3E);                          /* bcfe(CC3E) */
        REG_LINK_CTRL_E717 &= 0xFE;
      }
      u4c_e7ae_bounded();                          /* 8d6e(0xff) */
      P1_WR(0x011F, 0x01);                          /* page1[0x011F]=1 */
      if (aa0 & 0x02) u4c_bcf2();                  /* bcf2(): CC3A/CC38 bit1 set */
    } else if (PR(0x0A9D) == 0x01) {
      if (aa0 & 0x02) u4c_bd41();                  /* bd41(): CC3B &= 0xFD */
      u4c_e7ae_bounded();                          /* 8d6e(4) */
      REG_POWER_DOMAIN = 0x01;
      if (aa0 & 0x02) { U4C_BD3A(0xCC3B); U4C_BD5E(0xCC37); }   /* bd3a(CC3B); bd5e(CC37) */
    } else {                                       /* mode 0 */
      u4c_e7ae_bounded();                          /* 538d(9)+e7ae */
      REG_USB_EP_CTRL_91D0 = 0x01;
    }

    phy_cc10_cmd_wait(0, 0x27, 2);                 /* phy_cmd_cc10_and_wait(0,0x27,2) */
    PR(0x07ED) = 0x01;                             /* one-shot suppress (a17c path) */

    /* d0d3/cc3f conditional before the wait loop */
    if (u4c_bd6c() != 0 && (aa0 & 0x08)) {          /* bd6c()!=0 && 0x0AA0.3 */
      U4C_BCFE(0xCC3F);                            /* bcfe(CC3F) */
      U4C_BD5E(0xCC3F);                            /* bd5e(CC3F) */
      u4c_bd2a(0xCC3F);                            /* bd2a(): clear bits5,6 */
      U4C_BD65(0xCC3D);                            /* bd65(CC3D) */
    }

    /* dynamic link-up service loop: pump PD-RX while bringing up lanes */
    { uint32_t guard = 0;
      while (u4c_bd6c() != 0 && ++guard < 200000UL) {
        if (REG_INT_PCIE_NVME & 0x40) pd_rx_isr();         /* C80A.6 -> af5e() == pd_rx_isr() */
        /* break on link-complete (0x0AE3==0 && CC33.2) OR connect-aborted (0x07ED==0) */
        if ((PR(0x0AE3) == 0 && (REG_CPU_EXEC_STATUS_2 >> 2 & 1)) || PR(0x07ED) == 0) break;
      }
    }

    /* tail */
    if ((aa0 & 0x08) && ((int8_t)REG_LTSSM_STATE < 0)) {  /* 0x0AA0.3 && CC3D.7 */
      boot_phy_d0d3_typec_sbu();                    /* d0d3 Type-C SBU re-arm */
    }
    aa0 = PR(0x0AA0);
    if (aa0 & 0x01) {                               /* 0x0AA0.0 teardown */
      PR(0xCAC4) &= 0xFE;
      REG_CMD_CONFIG &= 0xDF;
      u4c_bd2a(0xC698);                             /* bd2a(C698): clear bits5,6 */
      PR(0xE313) &= 0x7F;
      REG_CMD_CFG_E413 &= 0xFD;
    }

    /* E764 bit4 = LINK-MODE-ENABLE */
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xEF) | 0x10;
    /* re-latch link rate into 0x0A9F/0x0A9E (final) */
    PR(0x0A9F) |= (REG_LINK_WIDTH_E710 & 0xE0);  /* bVar5 | bd49() */
    PR(0x0A9E) = (REG_CPU_MODE_NEXT & 0x1F) | (uint8_t)(PR(0x0A9E) << 5);  /* bd57() | (0x0A9E<<5) */
    REG_CPU_CTRL_CA81 &= 0xFE;

    /* SECOND mode dispatch -> dd42(uVar2) */
    if (PR(0x0A9D) == 0x02) {
      if (aa0 & 0x02) {
        PR(0x0A9D) = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;    /* bd50() | 3 */
        PR(0x0A9E) = (REG_CPU_CTRL_CC3E & 0xFD);           /* (0x0A9D+1 store) bd33() */
        U4C_BCEB(0xCA81);                           /* bceb() */
        u4c_bd14();                                 /* bd14 */
      }
      /* e0d9(0) PHY descriptor seed (already ran in sb_block_init); only uVar2 selection matters here */
      uVar2 = (PR(0x09FA) >> 1 & 1) ? 2 : 1;
    } else if (PR(0x0A9D) == 0x01) {
      if (aa0 & 0x02) {
        /* wait CD4E.0 then CD4E.1, then E712[1:0] (bounded) */
        { uint16_t g = 0; while (!(PR(0xCD4E) & 1) && ++g < 0x4000);
          g = 0; while (!(PR(0xCD4E) & 2) && ++g < 0x4000); }
        { uint16_t g = 0;
          while (!((REG_LINK_STATUS_E712 & 1) || (REG_LINK_STATUS_E712 & 2)) && ++g < 0x4000); }
        REG_TIMER_CTRL_CC3B &= 0xBF;
        phy_cc10_cmd_wait(0, 0x13, 2);              /* phy_cmd_cc10_and_wait(0,0x13,2) */
        REG_CPU_CTRL_CC37 &= 0xFB;
        U4C_BCFE(0xCC3B);
      }
      uVar2 = 4;
    } else {
      uVar2 = 4;
    }
    boot_phy_dd42(uVar2);                            /* dd42(uVar2) -> E7E3 per-mode */

    /* tail: drive the connect (usb4_connect_u4) unless the one-shot suppress is set */
    if (PR(0x07ED) == 0) {
      uint8_t cv = PR(0x0A9D);
      if (cv == 0) {
        REG_CPU_CTRL_CC3E = (REG_CPU_CTRL_CC3E & 0xFD) | 0x02;     /* bce7: CC3E bit1 + E717.0 */
        REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;
        REG_CPU_CTRL_CC36 &= 0xDF;
      }
      U4C_BD23(0x92C4);                              /* bd23(92C4): set bit5 */
      if (cv == 0) {
        REG_POWER_MISC_CTRL = (REG_CPU_CTRL_CC3E & 0xFD);            /* store bd33() */
        U4C_BD23(0xCC36);
      }
      REG_POWER_MISC_CTRL &= 0xDF;
      usb4_connect_u4();                             /* drive SB/tunnel connect */
    }
    PR(0x07ED) = 0x00;

    /* final: if link complete -> CC33 ack + cd10 (downstream PCIe handoff). Stock cd10 spins
     * forever; we defer the downstream bring-up to the super-loop (sb_tunnel_up_pending) and return. */
    if (PR(0x0AE3) == 0 && (REG_CPU_EXEC_STATUS_2 >> 2 & 1)) {
      REG_CPU_EXEC_STATUS_2 = 0x04;
      sb_tunnel_up_pending = 1;                      /* cd10: defer downstream PCIe bring-up */
    }
    uart_puts("[8a89done]");
  }
}

/* FUN_CODE_c9a8 @0xC9A8 — host-link-event connect dispatcher (e94d=c9a8(0), e952=c9a8(1)).
 * Gate: (0x09FA.2) && (0x0AF1.0) && (0x07E8 || 0x07EB) -> C6A8 &= ~1; bank0_8a89(arg).
 * The heavy banked eGPU LTSSM/PERST bring-up (3578) is deferred to the super-loop via
 * sb_tunnel_up_pending instead of running the long banked poll inside the ISR. */
static void bank0_c9a8(uint8_t arg) {
  PR(0x0A7D) = arg;                                /* mode for 8a89 */
  if (PR(0x09FA) & 0x04) {                          /* 0x09FA.2 */
    /* defer downstream PCIe bring-up (e3b7(3) B480 PERST clear + 3578) */
    sb_tunnel_up_pending = 1;
    if ((PR(0x0AF1) & 0x01) &&
        (PR(0x07E8) != 0 || PR(0x07EB) != 0)) {     /* the c9a8 connect gate */
      REG_PHY_CFG_C6A8 &= 0xFE;                            /* C6A8 &= ~1 */
      bank0_8a89(PR(0x0A7D));                       /* run the lane-MODE engine */
    }
    PR(0x07E8) = 0x00;
    PR(0x0B2F) = 0x01;
    return;
  }
  /* 0x09FA.2 clear: secondary 0x09FA.1 path -> e3b7(1) (deferred). */
}

#endif /* USB4_CONNECT_H */
