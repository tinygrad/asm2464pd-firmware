#ifndef USB4_CONNECT_H
#define USB4_CONNECT_H
/*
 * bank0_8a89 — the USB4 PHY / lane-MODE bring-up engine (THE missing dynamic engine).
 * Faithful transcription of the ASM2464PD stock firmware (fw_tinygrad.bin) @0x8A89.
 *
 * This is the ONLY function in the whole USB4 path that:
 *   - arms E764 bit4 = LINK-MODE-ENABLE  (E764 = (E764 & 0xEF) | 0x10)   [the keystone bit]
 *   - arms E751 = 1                      (USB4 link arm, inside if(0x0AA0&1))
 *   - latches the E710/CA06 link-rate into 0x0A9F / 0x0A9E
 *   - writes the per-mode E7E3 descriptor via dd42()
 *   - runs the dynamic in-loop service `while(bd6c()){ if(C80A.6) pd_rx_isr(); ... }` that pumps
 *     PD-RX WHILE bringing up the lanes, and waits on link-completion (CC33.2 / 0x07ED).
 *
 * handmade reproduced a byte-faithful STATIC SB block (sb.h) but NEVER ran this engine, so the
 * link stayed mode0 and the TB4 host never saw a trained USB4 link -> C80A.5 never fired. It is
 * meant to be RE-DRIVEN per host link-event from the INT0 demux (0e5b -> e94d/e952 -> c9a8 ->
 * 8a89). usb4_connect_u4 (the inner a3f5 RMW + sb_assert) is called ONLY from inside this wrapper
 * (8a89 tail @0x8d49) and from the faithful Enter_USB data-msg path (a17f, vdm.h) -- never
 * standalone from a VDM Accept.
 *
 * Included AFTER usb4.h (usb4_connect_u4 fwd-decl, SB_* / P1_* from sb.h, PR(), uart_*, the boot_phy
 * helpers e7ae/d0d3/phy_cc10_cmd_wait, and pd_rx_isr from pd.h) and BEFORE sb_router.h.
 *
 * Helper map (all byte-exact, decompiled from stock):
 *   bd49() = E710 & 0xE0          bd57() = CA06 & 0x1F        bd50() = E716 & 0xFC
 *   bd33() = CC3E & 0xFD          bceb(a): a |= 1             bcfe(a): a = (&0xFD)|2  (set bit1)
 *   bd23(a): a = (&0xDF)|0x20     bd3a(a): a = (&0xBF)|0x40   bd65(a): a = (&0x7F)|0x80
 *   bd2a(a): a &= 0xDF; a &= 0xBF                             bd5e(a): a = (&0xFB)|0x04
 *   bce7(v,a): *a = v; E717 = (E717&0xFE)|1
 *   bcf2(): CC3A=(&0xFD)|2; CC38=(&0xFD)|2     bd41(): CC3B &= 0xFD     bd14(): CC3A&=0xFD; CC38&=0xFD
 *   8d6e(): uart_puts; e7ae()  (the e7ae C006/C00E PHY-lock busy wait, BOUNDED here)
 *   bd6c(): dcd4(0x0A9D); return 0x0A9D       dcd4(m): CA00=(&0xC0)|7; CA0A=2; (CA0D/CA0E poll, no SE)
 *   af5e() == pd_rx_isr()   dd42() == boot_phy_dd42()   cd10() == bank0_cd10 (handoff, never returns)
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

/* bd6c: pump the link controller (dcd4) then return the mode byte 0x0A9D. Loop continues while
 * 0x0A9D != 0. dcd4 = CA00=(&0xC0)|7; CA0A=2 (+ a CA0D/CA0E status poll that has no side effect in
 * the decompile -> reproduced as the two writes). */
static uint8_t u4c_bd6c(void) {
  PR(0xCA00) = (PR(0xCA00) & 0xC0) | 0x07;       /* dcd4 */
  PR(0xCA0A) = 0x02;
  return PR(0x0A9D);
}

/* 8d6e: print + e7ae PHY-lock wait. e7ae waits C006[4:0]==0x10 then C00E[2:0]==0 (BOUNDED so a
 * never-locking PHY can't hang the ISR -- handmade bounds every stock spin). */
static void u4c_e7ae_bounded(void) {
  uint16_t g = 0;
  while (((PR(0xC006) & 0x1F) != 0x10) && ++g < 0x0800);
  g = 0;
  while (((PR(0xC00E) & 0x07) != 0x00) && ++g < 0x0800);
}

/* indicates the engine ran (instrumentation, read from the super-loop)
 * IRAM-HEADROOM FIX: relocated to XDATA scratch (0x8800..); seeded in main(). */
static volatile uint8_t __xdata __at(0x0B51) bank0_8a89_entered;

/* ==================================================================================== */
/* bank0_8a89 @0x8A89 — verbatim. param = USB4 link mode (0=?,1=USB3.2-tunnel,2=USB4). */
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
  if (PR(0xC8FF) < 0x06) {
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

    /* latch link-rate into 0x0A9F / 0x0A9E (pre-arm). Net (per decompile):
     *   0x0A9F = (E710 & 0xE0) | 0x1F   [bd49()=E710&0xE0, then |0x1F]
     *   0x0A9E = (CA06 & 0x1F) | 0x80   [bd57()=CA06&0x1F, then |0x80] */
    PR(0x0A9F) = REG_LINK_WIDTH_E710 & 0x1F;
    PR(0x0A9F) = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
    PR(0x0A9E) = REG_CPU_MODE_NEXT >> 5;
    PR(0x0A9E) = (REG_CPU_MODE_NEXT & 0x1F) | 0x80;

    aa0 = PR(0x0AA0);
    if (aa0 & 0x01) {                             /* 0x0AA0.0 -> arm E751 (USB4 link arm) */
      U4C_BD23(0xE40B);                            /* bd23(E40B): set bit5 */
      U4C_BD23(0xC698);                            /* bd23(C698): set bit5 */
      u4c_bd14();                                  /* bd3a()? -> stock bd3a()/bd14: CC3A/CC38 &=0xFD */
      U4C_BCEB(0xCAC4);                            /* bceb(CAC4) */
      REG_PHY_POLL_E751 = 0x01;                           /* *** E751 = 1 (USB4 LINK ARM) *** */
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
      u4c_e7ae_bounded();                          /* 8d6e(0xff): print + e7ae */
      P1_WR(0x011F, 0x01);                          /* r3_write_dispatch(val=1,0x011f,R3=2): page1[0x011F]=1 */
      if (aa0 & 0x02) u4c_bcf2();                  /* bcf2(): CC3A/CC38 bit1 set */
    } else if (PR(0x0A9D) == 0x01) {
      if (aa0 & 0x02) u4c_bd41();                  /* bd41(): CC3B &= 0xFD */
      u4c_e7ae_bounded();                          /* 8d6e(4): print + e7ae */
      REG_POWER_DOMAIN = 0x01;
      if (aa0 & 0x02) { U4C_BD3A(0xCC3B); U4C_BD5E(0xCC37); }   /* bd3a(CC3B); bd5e(CC37) */
    } else {                                       /* mode 0 */
      u4c_e7ae_bounded();                          /* 538d(9)+e7ae */
      REG_USB_EP_CTRL_91D0 = 0x01;
    }

    phy_cc10_cmd_wait(0, 0x27, 2);                 /* phy_cmd_cc10_and_wait(0,0x27,2) */
    PR(0x07ED) = 0x01;                             /* one-shot suppress (a17c path) */

    /* the d0d3/cc3f conditional BEFORE the wait loop */
    if (u4c_bd6c() != 0 && (aa0 & 0x08)) {          /* bd6c()!=0 && 0x0AA0.3 */
      U4C_BCFE(0xCC3F);                            /* bcfe(CC3F) */
      U4C_BD5E(0xCC3F);                            /* bd5e()? stock bd5e on a staged reg */
      u4c_bd2a(0xCC3F);                            /* bd2a(): clear bits5,6 */
      U4C_BD65(0xCC3D);                            /* bd65(CC3D) */
    }

    /* *** the DYNAMIC link-up service loop (pumps PD-RX while bringing up lanes) *** */
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

    /* *** E764 bit4 = LINK-MODE-ENABLE *** (the keystone write) */
    REG_PHY_TIMER_CTRL_E764 = (REG_PHY_TIMER_CTRL_E764 & 0xEF) | 0x10;
    /* re-latch link rate into 0x0A9F/0x0A9E (final) */
    PR(0x0A9F) = PR(0x0A9F) | (REG_LINK_WIDTH_E710 & 0xE0);  /* bVar5 | bd49() */
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
      /* e0d9(0) -- PHY descriptor seed (bank1, e0d9_stub). Reproduced as no-op arm (the SB e0d9
       * seed already ran in sb_block_init); the load-bearing effect here is uVar2 selection. */
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
        REG_CPU_CTRL_CC3E = (REG_CPU_CTRL_CC3E & 0xFD) | 0x02;     /* bce7(bd33()|2 -> store): CC3E bit1 + E717.0 */
        REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;
        REG_CPU_CTRL_CC36 &= 0xDF;
      }
      U4C_BD23(0x92C4);                              /* bd23(92C4): set bit5 */
      if (cv == 0) {
        REG_POWER_MISC_CTRL = (REG_CPU_CTRL_CC3E & 0xFD);            /* store bd33() */
        U4C_BD23(0xCC36);
      }
      REG_POWER_MISC_CTRL &= 0xDF;
      usb4_connect_u4();                             /* *** drive SB/tunnel connect *** */
    }
    PR(0x07ED) = 0x00;

    /* final: if link complete -> CC33 ack + cd10 (downstream PCIe handoff). Stock cd10 brings up
     * the downstream PCIe link to the GPU then SPINS FOREVER (it is the terminal hand-off state).
     * handmade can't spin inside the connect path, so we DEFER the downstream bring-up to the
     * super-loop (sb_tunnel_up_pending -> pcie_power_on) and return -- the link-mode is already
     * armed (E764.4/E751) which is the load-bearing effect the host needs. */
    if (PR(0x0AE3) == 0 && (REG_CPU_EXEC_STATUS_2 >> 2 & 1)) {
      REG_CPU_EXEC_STATUS_2 = 0x04;
      sb_tunnel_up_pending = 1;                      /* cd10: defer downstream PCIe bring-up */
    }
    uart_puts("[8a89done]");
  }
}

/* ====================================================================================
 * FUN_CODE_c9a8 @0xC9A8 — the host-link-event connect dispatcher (e94d=c9a8(0), e952=c9a8(1)).
 * Gate: (0x09FA.2) && (0x0AF1.0) && (0x07E8 || 0x07EB).  When open:
 *   if (0x0B41) e3b7(3); pcie_downstream_link_bringup(0x0AEF); e96c(); then if gated -> 545c();
 *   C6A8 &= ~1; bank0_8a89(arg).   tail: 0x07E8=0; 0x0B2F=1.
 * pcie_downstream_link_bringup(0x0AEF)/3578 is the heavy banked eGPU LTSSM/PERST path; handmade
 * runs the equivalent (pcie_power_on) from the super-loop via sb_tunnel_up_pending, so we set that
 * pending flag instead of running the long banked poll inside the ISR.
 * ==================================================================================== */
static void bank0_c9a8(uint8_t arg) {
  PR(0x0A7D) = arg;                                /* xdata_00a7d = param (mode for 8a89) */
  if (PR(0x09FA) & 0x04) {                          /* 0x09FA.2 */
    /* if (0x0B41) bank0_e3b7(3) -- B480 PERST clear; deferred via pcie path below. */
    /* pcie_downstream_link_bringup(0x0AEF) + e96c(): defer the downstream PCIe bring-up. */
    sb_tunnel_up_pending = 1;
    if ((PR(0x0AF1) & 0x01) &&
        (PR(0x07E8) != 0 || PR(0x07EB) != 0)) {     /* the c9a8 connect gate */
      REG_PHY_CFG_C6A8 &= 0xFE;                            /* C6A8 &= ~1 (545c side then this) */
      bank0_8a89(PR(0x0A7D));                       /* *** run the lane-MODE engine *** */
    }
    PR(0x07E8) = 0x00;
    PR(0x0B2F) = 0x01;
    return;
  }
  /* 0x09FA.2 clear: secondary 0x09FA.1 path -> e3b7(1) (deferred). */
}

#endif /* USB4_CONNECT_H */
