#ifndef USB4_H
#define USB4_H
/*
 * USB4 bring-up: boot PHY setup, mode entry, router-op/IRQ handling,
 * and lane-mode connect glue. Include after sb.h.
 */

/*=== Boot PHY / Tunnel Adapter Setup ===*/

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
  /* Stock cf4d->bce7: after CC3B&=0xFE, bce7 ALSO sets E717 = (E717 & 0xFE) | 1 (link-ctrl enable
   * bit0). Handmade had dropped this E717 strobe (only did the CC3B write). Restored byte-true. */
  REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;
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
  XDATA_REG8(0x0AE3) = 1;
  XDATA_REG8(0x0AEC) = 3;
  XDATA_REG8(0x0AED) = 3;
  XDATA_REG8(0x0AF1) = 0x00;
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

/*=== USB4 Mode / Router-Op Handling ===*/

/*
 * USB4 mode-establishment glue: post-Enter_USB connect path + the INT1 USB4 event demux.
 * Include AFTER vdm.h (needs PR(a), uart_*, PD helpers, DPX SFR).
 */

/* PHY descriptor seed (mode 4 seeds the PHY trim registers). */
static void u4c_e0d9(uint8_t mode) {
  if (mode == 4) {
    REG_PHY_RXPLL_RESET = 0x3E; REG_PHY_CTRL_C20F = 0x08; REG_PHY_CDR_SEED_C210 = 0x08; REG_PHY_CDR_SEED_C211 = 0x2E; REG_PHY_CDR_SEED_C212 = 0x3E;
    REG_PHY_CDR_SEED_C214 = 0x00; REG_PHY_CDR_SEED_C215 = 0x20; REG_PHY_CDR_SEED_C216 = 0x00; REG_PHY_CDR_SEED_C217 = 0x3F;
  }
}

/* Timer-enable gate: mode 1 clears bit1; else conditionally sets bit1 per 0x0AF1.4. */
static void u4c_e7c1(uint8_t mode) {
  if (mode == 1) { REG_TIMER_ENABLE_B &= 0xFD; REG_TIMER_ENABLE_A &= 0xFD; }
  else if (u4_connect_gate & 0x10) { REG_TIMER_ENABLE_B = (REG_TIMER_ENABLE_B & 0xFD) | 0x02; REG_TIMER_ENABLE_A = (REG_TIMER_ENABLE_A & 0xFD) | 0x02; }
}

/* Post-Enter_USB connect path: drives sideband bring-up so the host CM trains the lanes. */
static void usb4_connect_u4(void) {
  u4_connect_gate |= 0x01;
  if (u4_connect_gate & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    REG_CPU_CTRL_CA81 &= 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;   /* byte-true; CA06 mode-3 req (recomputed down later) */
  }
  boot_phy_dd42(0);
  u4c_e7c1(1);
  u4c_e0d9(0);
  if (u4_enter_usb_accepted == 0) {
    if (u4_connect_route_latch == 0) return;
    u4_route_mode = 0x81;
    u4_lane_gate_sel = 0x02;
  } else {
    /* stock sb_lane_flip_init a433: PLAIN write 0x09FA = 0x09F9 & 3 (NO bit2 preservation). The prior
     * `(u4_route_mode & 0x04) | ...` preserved a stale bit2 (from the connect_decide 0x09FA=4 pre-write)
     * -> 0x09FA latched 0x07/0x06 instead of stock's 0x01. a444/a450 likewise write plain 1/2. */
    u4_route_mode = (u4_mode_flag & 0x03);
    if (u4_dp_alt_mode == 0x03) {
      if (pd_usb3_fallback_flag == 0) { u4_route_mode = 2; u4_lane_gate_sel = 1; }
      else                 { u4_route_mode = 1; u4_lane_gate_sel = 2; }
    }
    if (u4_route_mode & 0x02) {
      REG_LINK_STATUS_E716 &= 0xFC;
      REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
      SB_WR(0xD8, 0x02);
    }
  }
  sb_assert();
}

/* INT1 USB4 event demux: services the USB4 INT sources and applies their W1C acks. */

static void sb_router_event_handler(void);

/* c0a5 = cm_routerop_mailbox (CODE_BANK1::c0a5): inbound host router-op responder.
 * EC06.0 fires when the host posts an in-band router-op into the EA80 mailbox. The E2 CONFIG path
 * latches EA82-85, reads the router config-space image from the SPI-flash shadow, DMA-sends the
 * response on tunnel TX channel 0xEA, and acks EA90=0xA5. */

/* IRAM-free scratch: this responder runs in the INT1 ISR call tree where SDCC cannot overlay
 * function locals -> they consume scarce DSEG (the handmade build sits at the IRAM ceiling). So the
 * responder routes ALL working storage through file-scope __xdata statics, no multi-byte stack locals. */
static volatile __xdata uint16_t u4rop_w16;   /* shared 16-bit accumulator */
static volatile __xdata uint16_t u4rop_g;     /* bounded-wait guard */
static volatile __xdata uint8_t  u4rop_op;    /* latched EA80 opcode */

/* ced6: returns 1 if (limit < addr) (stock CY/borrow -> error status). 32-bit LE compare. */
static uint8_t u4rop_underflow(void) {
  if (u4_rop_limit[3] != u4_rop_cfg_addr[3]) return u4_rop_limit[3] < u4_rop_cfg_addr[3];
  if (u4_rop_limit[2] != u4_rop_cfg_addr[2]) return u4_rop_limit[2] < u4_rop_cfg_addr[2];
  if (u4_rop_limit[1] != u4_rop_cfg_addr[1]) return u4_rop_limit[1] < u4_rop_cfg_addr[1];
  if (u4_rop_limit[0] != u4_rop_cfg_addr[0]) return u4_rop_limit[0] < u4_rop_cfg_addr[0];
  return 0;
}

/* d945 = cm_routerop_send_read_resp — build+arm+DMA-send the config-space READ response (byte-true).
 * Config-shadow scratch RELOCATED off stock 0x0AAD/0x0AB1/0x0AB2 (those alias live SB/PHY cells). */
static void u4rop_send_read_resp(void) {
  u4_rop_dir = 1;                                /* stock idata 0x4E = 1 */
  if (u4rop_underflow()) u4_rop_xfer_len = 0x80; /* ced6: addr>limit -> error status */
  else                   u4_rop_xfer_len = (uint8_t)(u4_rop_limit[3] - u4_rop_cfg_addr[3]); /* cf11 */
  REG_I2C_DMA_ENABLE = (REG_I2C_DMA_ENABLE & 0xF9) | 0x02;   /* cf35: C805 reply trigger */

  /* d95a-d977: flash config-shadow pointer = cf23(limit) + (dir?+2:+0); low-16 + carry. */
  u4_rop_shadow_ptr[2] = u4_rop_limit[2];
  u4_rop_shadow_ptr[3] = u4_rop_limit[3];
  u4rop_w16 = (uint16_t)((((uint16_t)u4_rop_limit[1] << 8) | u4_rop_limit[0]) + (u4_rop_dir ? 3 : 1));
  u4_rop_shadow_ptr[0] = (uint8_t)(u4rop_w16 & 0xFF);
  u4_rop_shadow_ptr[1] = (uint8_t)(u4rop_w16 >> 8);

  u4_rop_resp_hdr[0] = 0x00;                       /* stock 0x0AB1 = 0 */
  u4_rop_resp_hdr[1] = u4_rop_xfer_len;            /* stock 0x0AB2 = transfer length */

  /* d987 (-> bank0_be02 R7=3): SPI flash READ (cmd 0x03) of the config-space shadow. Byte-true. */
  REG_FLASH_MODE = REG_FLASH_MODE & 0xFE;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = (REG_FLASH_BUF_OFFSET_HI & 0xFC);
  REG_FLASH_CMD = 0x03;
  REG_FLASH_ADDR_LO = u4_rop_shadow_ptr[0];
  REG_FLASH_ADDR_MD = u4_rop_shadow_ptr[1];
  REG_FLASH_ADDR_HI = u4_rop_shadow_ptr[2];
  REG_FLASH_DATA_PAGE_CNT = 0;
  REG_FLASH_DATA_BYTE_OFS = u4_rop_resp_hdr[0];
  REG_FLASH_CSR = 0x01;
  for (u4rop_g = 0; (REG_FLASH_CSR & 0x01) && u4rop_g < 0x4000; u4rop_g++) { }
  REG_FLASH_MODE = REG_FLASH_MODE & 0xEF;
  REG_FLASH_MODE = REG_FLASH_MODE & 0xDF;
  REG_FLASH_MODE = REG_FLASH_MODE & 0xBF;
  REG_FLASH_MODE = REG_FLASH_MODE & 0x7F;

  /* cf3f: limit += len (advance the running cursor). */
  u4rop_w16 = (uint16_t)((((uint16_t)u4_rop_limit[1] << 8) | u4_rop_limit[0]) + u4_rop_xfer_len);
  u4_rop_limit[0] = (uint8_t)(u4rop_w16 & 0xFF);
  u4_rop_limit[1] = (uint8_t)(u4rop_w16 >> 8);

  /* d995-d9ba: DMA the response out the tunnel TX channel 0xEA. Byte-true. */
  REG_DMA_MODE      = 0x70;                         /* C8B0 */
  XDATA_REG8(0xC8B1) = 0x00;
  REG_DMA_CHAN_AUX   = 0xEA;                        /* C8B2 = router-op TX channel */
  REG_DMA_CHAN_AUX1  = 0x00;                        /* C8B3 */
  REG_DMA_XFER_CNT_HI = (uint8_t)(u4_rop_xfer_len ? 0xFE : 0xFF);  /* C8B4 */
  REG_DMA_XFER_CNT_LO = (uint8_t)(u4_rop_xfer_len - 1);            /* C8B5 (cec4) */
  XDATA_REG8(0xC8B6) = 0x10;                        /* cec4 */
  REG_DMA_TRIGGER = 0x01;                           /* C8B8 GO */
  for (u4rop_g = 0; (REG_DMA_TRIGGER & 0x01) && u4rop_g < 0x4000; u4rop_g++) { }
}

/* Config-space router-op dispatcher: byte-true port of CODE_BANK1::c0a5 (E2 CONFIG-READ path). */
static void cm_routerop_mailbox(void) {
  if (REG_SYS_CTRL_EA90 != 0x5A) return;       /* magic gate (c0a9) */

  if (u4_routerop_mbox_state == RMBOX_IDLE) {
    u4rop_op = REG_ROUTEROP_OPCODE_EA80;       /* c0b9: latch EA80 -> 0x0B03 */
    u4_routerop_mbox_opcode = u4rop_op;
    if (u4rop_op == 0xE2) {                    /* c0ef: CONFIG read/write path (route=1 ROUTER_CS) */
      if (REG_ROUTEROP_CFG_EA81 == 0x50 || REG_ROUTEROP_CFG_EA81 == 0x51) {  /* cf4c: READ/WRITE */
        /* ceef: latch direction + copy the 4-byte config addr EA82-85 -> u4_rop_cfg_addr. */
        u4_rop_dir = (uint8_t)(REG_ROUTEROP_CFG_EA81 & 0x01);
        u4_rop_cfg_addr[0] = XDATA_REG8V(0xEA82);
        u4_rop_cfg_addr[1] = XDATA_REG8V(0xEA83);
        u4_rop_cfg_addr[2] = XDATA_REG8V(0xEA84);
        u4_rop_cfg_addr[3] = XDATA_REG8V(0xEA85);
        u4_rop_limit[0] = u4_rop_cfg_addr[3];  /* cf2e: seed limit = splat-4 of addr-high */
        u4_rop_limit[1] = u4_rop_cfg_addr[3];
        u4_rop_limit[2] = u4_rop_cfg_addr[3];
        u4_rop_limit[3] = u4_rop_cfg_addr[3];
        u4rop_send_read_resp();                /* d945: build+arm+DMA-send the READ response */
        /* ceab: more to send? -> MULTIPKT_1 continuation. */
        if (!u4rop_underflow() &&
            (u4_rop_limit[0] != u4_rop_cfg_addr[0] || u4_rop_limit[1] != u4_rop_cfg_addr[1] ||
             u4_rop_limit[2] != u4_rop_cfg_addr[2] || u4_rop_limit[3] != u4_rop_cfg_addr[3]))
          u4_routerop_mbox_state = RMBOX_MULTIPKT_1;
      }
    }
    REG_SYS_CTRL_EA90 = 0xA5;                   /* c1a2: ack the host (response ready) */
    return;
  }

  if (u4_routerop_mbox_state == RMBOX_MULTIPKT_1) {
    if (u4_routerop_mbox_opcode == 0xE2) {      /* CONFIG-read continuation */
      u4rop_send_read_resp();
      if (u4rop_underflow()) u4_routerop_mbox_state = RMBOX_IDLE;
      REG_SYS_CTRL_EA90 = 0xA5;
      return;
    }
    u4_routerop_mbox_state = RMBOX_IDLE;
  } else if (u4_routerop_mbox_state == RMBOX_MULTIPKT_2) {
    if (u4_routerop_mbox_opcode == 0xE3) {
      u4_routerop_mbox_state = RMBOX_IDLE;
      REG_SYS_CTRL_EA90 = 0xA5;
      return;
    }
    u4_routerop_mbox_state = RMBOX_IDLE;
  }
}

/* c105 = usb4_sec_adapter_link_event_c80a4 (INT1 C80A.4 handler). Stock c105 demux:
 *   1) rd P1[0x1407]; if .0 (WIDTH evt)  -> a522 link-width service (W1C P1[0x1203].7; 0x09FA|=4)
 *   2) rd P1[0x1407]; if .3 (TUNNEL evt) -> bank1 d855: W1C-loop P1[0x1508] bits 4/3/2/1, each
 *        bit dispatches a tunnel-event service (e4ea Enable / ee29 DisPath / e76b UPS_Rst_Deassert).
 *   3) rd P1[0x1603]; if .0 (evt0) -> W1C P1[0x1603]=1; if 0x09FA.1: u4_entered_usb_mode=1 + ca0d;
 *        e74e (CC re-arm); pd_cm_dispatch_sel(0x07FF)=0x69; return.
 *   4) else if P1[0x1603].1 (evt1) -> W1C P1[0x1603]=2; deeper reconfig (data/state dependent).
 */
static void u4lb_e74e(void);                                   /* defined in usb4_lanebond.h (incl. after) */
static void u4lb_a310(uint8_t cur);                             /* descriptor-engine helper (usb4_lanebond.h) */
static void u4lb_e890(uint8_t ctrl_low6);                       /* descriptor-engine commit (usb4_lanebond.h) */
static void u4lb_d855(uint8_t heavy);                          /* tunnel-event dispatch (usb4_lanebond.h) */
static void u4lb_d90e_link_phy_reconfig(void);                 /* a522 width-event PHY reconfig + Deassert */

static void usb4_sec_adapter_link_event_c105(void) {
  uint8_t p1407 = P1_RD(P1_USB4_ADP_EVENT_STATUS_1407);
  uint8_t p1508 = P1_RD(P1_USB4_TUNNEL_EVENT_STATUS_1508);
  uint8_t p1603 = P1_RD(P1_USB4_BOOT_TAIL_EVENT_1603);

  /* (1) WIDTH event: W1C P1[0x1203].7 + flag USB4-ready (a522-lite, link-safe). */
  if (p1407 & 0x01) {
    if (P1_RD(P1_USB4_WIDTH_EVENT_1203) & 0x80) {
      P1_WR(P1_USB4_WIDTH_EVENT_1203, 0x80);       /* W1C width-change pending */
      if ((u4_connect_gate & 0x10) && (u4_route_mode & 0x81)) u4_route_mode |= 0x04;
    }
    /* a522/a578 sub-leg: lane-width-set event -> descriptor commit -> PHY reconfig. */
    if (P1_RD(0x124E) & 0x02) {
      P1_WR(0x124E, 0x02);                 /* W1C the lane-width-set sub-event */
      u4lb_a310(0x35);
      eng_a2df(0x36, 0x03);
      u4lb_e890(0x03);
      if (P1_RD(0x1243) & 0x80) u4lb_d90e_link_phy_reconfig();
    }
  }

  /* (2) TUNNEL event: run the stock d855 leg dispatch. The c105 path passes heavy=0 so the
   * 1508.1 assert leg only W1C-acks and runs ee94; the destructive PERST re-drive is not taken. */
  if (p1407 & 0x08) {
    u4lb_d855(0);
  }

  /* (3) adapter evt0 -> W1C + the 0x69 dispatch token (link-safe, NO C659/PERST). */
  if (p1603 & 0x01) {
    P1_WR(P1_USB4_BOOT_TAIL_EVENT_1603, 0x01);
    if (u4_route_mode & 0x02) {
      if (REG_POWER_STATUS & 0x40) u4_entered_usb_mode = 1;
      u4lb_e74e();
      pd_cm_dispatch_sel = 0x69;
    }
  } else if (p1603 & 0x02) {
    P1_WR(P1_USB4_BOOT_TAIL_EVENT_1603, 0x02);     /* W1C evt1; deeper reconfig omitted */
  }
}

/* Called from int1_isr after PD-RX: acks and forwards each fired USB4 INT source. */
static void usb4_int_demux(void) {
  uint8_t int_sources = REG_INT_PCIE_NVME;
  if (int_sources & 0x20) {
    sb_router_event_handler();
  }
  if (int_sources & 0x10) {                /* C80A.4 = secondary adapter/link event (c105) */
    usb4_sec_adapter_link_event_c105();
  }
  if (REG_NVME_EVENT_STATUS & 0x01) {
    REG_NVME_EVENT_ACK = 1;
    cm_routerop_mailbox();
  }
  if (int_sources & 0x0F) {
    { uint8_t tunnel_status = REG_PHY_RXPLL_TRIGGER;
      if (tunnel_status & 0x04) REG_PHY_RXPLL_TRIGGER = 0x04;
      if (tunnel_status & 0x08) REG_PHY_RXPLL_TRIGGER = 0x08; }
  }
}

/*=== USB4 RX Equalizer Table ===*/

/*
 * SB-PHY RX-lane arm table. Each row = {page, reg, AND-mask, OR-mask} for a
 * paged-XDATA read-modify-write: v = PG_RD(page<<8|reg); PG_WR(..., (v & AND) | OR).
 */

static __code const uint8_t u4rx_tab[324][4] = {
  {0x78,0x9b,0xcf,0x10}, {0x78,0x9b,0x7f,0x80}, {0x60,0x00,0x3f,0xc0}, {0x60,0x04,0xff,0x01},
  {0x60,0x06,0x0f,0x40}, {0x60,0x07,0x80,0x01}, {0x60,0x59,0x03,0x40}, {0x60,0x5a,0xe0,0x00},
  {0x60,0x0a,0xff,0x11}, {0x60,0x42,0xff,0x03}, {0x60,0x05,0xf9,0x02}, {0x79,0x9b,0xcf,0x10},
  {0x79,0x9b,0x7f,0x80}, {0x64,0x00,0x3f,0xc0}, {0x64,0x04,0xff,0x01}, {0x64,0x06,0x0f,0x40},
  {0x64,0x07,0x80,0x01}, {0x64,0x59,0x03,0x40}, {0x64,0x5a,0xe0,0x00}, {0x64,0x0a,0xff,0x11},
  {0x64,0x42,0xff,0x03}, {0x64,0x05,0xf9,0x02}, {0x7a,0x9b,0xcf,0x10}, {0x7a,0x9b,0x7f,0x80},
  {0x68,0x00,0x3f,0xc0}, {0x68,0x04,0xff,0x01}, {0x68,0x06,0x0f,0x40}, {0x68,0x07,0x80,0x01},
  {0x68,0x59,0x03,0x40}, {0x68,0x5a,0xe0,0x00}, {0x68,0x0a,0xff,0x11}, {0x68,0x42,0xff,0x03},
  {0x68,0x05,0xf9,0x02}, {0x7b,0x9b,0xcf,0x10}, {0x7b,0x9b,0x7f,0x80}, {0x6c,0x00,0x3f,0xc0},
  {0x6c,0x04,0xff,0x01}, {0x6c,0x06,0x0f,0x40}, {0x6c,0x07,0x80,0x01}, {0x6c,0x59,0x03,0x40},
  {0x6c,0x5a,0xe0,0x00}, {0x6c,0x0a,0xff,0x11}, {0x6c,0x42,0xff,0x03}, {0x6c,0x05,0xf9,0x02},
  {0x78,0x64,0xf3,0x0c}, {0x78,0xbf,0xbf,0x00}, {0x78,0xbf,0x7f,0x00}, {0x78,0x67,0x3f,0xc0},
  {0x79,0x64,0xf3,0x0c}, {0x79,0xbf,0xbf,0x00}, {0x79,0xbf,0x7f,0x00}, {0x79,0x67,0x3f,0xc0},
  {0x7a,0x64,0xf3,0x0c}, {0x7a,0xbf,0xbf,0x00}, {0x7a,0xbf,0x7f,0x00}, {0x7a,0x67,0x3f,0xc0},
  {0x7b,0x64,0xf3,0x0c}, {0x7b,0xbf,0xbf,0x00}, {0x7b,0xbf,0x7f,0x00}, {0x7b,0x67,0x3f,0xc0},
  {0x78,0x40,0xf7,0x00}, {0x78,0x01,0xe0,0x1c}, {0x78,0x11,0xe0,0x1c}, {0x78,0x21,0xe0,0x10},
  {0x78,0x31,0xe0,0x10}, {0x79,0x40,0xf7,0x00}, {0x79,0x01,0xe0,0x1c}, {0x79,0x11,0xe0,0x1c},
  {0x79,0x21,0xe0,0x10}, {0x79,0x31,0xe0,0x10}, {0x7a,0x40,0xf7,0x00}, {0x7a,0x01,0xe0,0x1c},
  {0x7a,0x11,0xe0,0x1c}, {0x7a,0x21,0xe0,0x10}, {0x7a,0x31,0xe0,0x10}, {0x7b,0x40,0xf7,0x00},
  {0x7b,0x01,0xe0,0x1c}, {0x7b,0x11,0xe0,0x1c}, {0x7b,0x21,0xe0,0x10}, {0x7b,0x31,0xe0,0x10},
  {0x78,0x34,0xf0,0x07}, {0x78,0x35,0xf0,0x0d}, {0x78,0x26,0xf0,0x0a}, {0x78,0x36,0xf0,0x07},
  {0x78,0x06,0x0f,0x70}, {0x78,0x16,0x0f,0x70}, {0x78,0x26,0x0f,0x70}, {0x78,0x36,0x0f,0x70},
  {0x78,0x37,0xe0,0x0f}, {0x78,0x46,0xf7,0x00}, {0x78,0x46,0xef,0x00}, {0x78,0x46,0xdf,0x00},
  {0x78,0x46,0xbf,0x00}, {0x78,0x1d,0xf0,0x00}, {0x78,0x2d,0xf0,0x00}, {0x78,0x3d,0xf0,0x0a},
  {0x78,0x1a,0xf0,0x04}, {0x78,0x3a,0xf0,0x0d}, {0x78,0x1b,0xc0,0x0a}, {0x78,0x3b,0xc0,0x0f},
  {0x78,0x0c,0xf8,0x07}, {0x78,0x1c,0xf8,0x07}, {0x78,0x02,0x1f,0xe0}, {0x78,0x22,0x1f,0x20},
  {0x78,0x32,0x1f,0x20}, {0x78,0x6c,0xf1,0x0c}, {0x78,0x6c,0x8f,0x60}, {0x60,0x20,0xf0,0x00},
  {0x60,0x20,0x0f,0x10}, {0x60,0x21,0xf0,0x02}, {0x60,0x21,0x0f,0x40}, {0x60,0x22,0xe0,0x06},
  {0x60,0x22,0x1f,0x20}, {0x60,0x23,0xfc,0x01}, {0x60,0x23,0x83,0x28}, {0x60,0x24,0xe0,0x0c},
  {0x60,0x24,0x1f,0xc0}, {0x60,0x25,0xfc,0x01}, {0x60,0x25,0x83,0x40}, {0x60,0x26,0x1f,0xc0},
  {0x60,0x27,0xfc,0x02}, {0x60,0x27,0x83,0x5c}, {0x60,0x28,0xf0,0x00}, {0x60,0x28,0x0f,0x00},
  {0x60,0x29,0xf0,0x01}, {0x60,0x29,0x0f,0x30}, {0x60,0x2a,0xf0,0x04}, {0x60,0x2a,0x0f,0x70},
  {0x60,0x2b,0xf0,0x09}, {0x60,0x2b,0x0f,0xc0}, {0x78,0x85,0xfb,0x04}, {0x79,0x34,0xf0,0x07},
  {0x79,0x35,0xf0,0x0d}, {0x79,0x26,0xf0,0x0a}, {0x79,0x36,0xf0,0x07}, {0x79,0x06,0x0f,0x70},
  {0x79,0x16,0x0f,0x70}, {0x79,0x26,0x0f,0x70}, {0x79,0x36,0x0f,0x70}, {0x79,0x37,0xe0,0x0f},
  {0x79,0x46,0xf7,0x00}, {0x79,0x46,0xef,0x00}, {0x79,0x46,0xdf,0x00}, {0x79,0x46,0xbf,0x00},
  {0x79,0x1d,0xf0,0x00}, {0x79,0x2d,0xf0,0x00}, {0x79,0x3d,0xf0,0x0a}, {0x79,0x1a,0xf0,0x04},
  {0x79,0x3a,0xf0,0x0d}, {0x79,0x1b,0xc0,0x0a}, {0x79,0x3b,0xc0,0x0f}, {0x79,0x0c,0xf8,0x07},
  {0x79,0x1c,0xf8,0x07}, {0x79,0x02,0x1f,0xe0}, {0x79,0x22,0x1f,0x20}, {0x79,0x32,0x1f,0x20},
  {0x79,0x6c,0xf1,0x0c}, {0x79,0x6c,0x8f,0x60}, {0x64,0x20,0xf0,0x00}, {0x64,0x20,0x0f,0x10},
  {0x64,0x21,0xf0,0x02}, {0x64,0x21,0x0f,0x40}, {0x64,0x22,0xe0,0x06}, {0x64,0x22,0x1f,0x20},
  {0x64,0x23,0xfc,0x01}, {0x64,0x23,0x83,0x28}, {0x64,0x24,0xe0,0x0c}, {0x64,0x24,0x1f,0xc0},
  {0x64,0x25,0xfc,0x01}, {0x64,0x25,0x83,0x40}, {0x64,0x26,0x1f,0xc0}, {0x64,0x27,0xfc,0x02},
  {0x64,0x27,0x83,0x5c}, {0x64,0x28,0xf0,0x00}, {0x64,0x28,0x0f,0x00}, {0x64,0x29,0xf0,0x01},
  {0x64,0x29,0x0f,0x30}, {0x64,0x2a,0xf0,0x04}, {0x64,0x2a,0x0f,0x70}, {0x64,0x2b,0xf0,0x09},
  {0x64,0x2b,0x0f,0xc0}, {0x79,0x85,0xfb,0x04}, {0x7a,0x34,0xf0,0x07}, {0x7a,0x35,0xf0,0x0d},
  {0x7a,0x26,0xf0,0x0a}, {0x7a,0x36,0xf0,0x07}, {0x7a,0x06,0x0f,0x70}, {0x7a,0x16,0x0f,0x70},
  {0x7a,0x26,0x0f,0x70}, {0x7a,0x36,0x0f,0x70}, {0x7a,0x37,0xe0,0x0f}, {0x7a,0x46,0xf7,0x00},
  {0x7a,0x46,0xef,0x00}, {0x7a,0x46,0xdf,0x00}, {0x7a,0x46,0xbf,0x00}, {0x7a,0x1d,0xf0,0x00},
  {0x7a,0x2d,0xf0,0x00}, {0x7a,0x3d,0xf0,0x0a}, {0x7a,0x1a,0xf0,0x04}, {0x7a,0x3a,0xf0,0x0d},
  {0x7a,0x1b,0xc0,0x0a}, {0x7a,0x3b,0xc0,0x0f}, {0x7a,0x0c,0xf8,0x07}, {0x7a,0x1c,0xf8,0x07},
  {0x7a,0x02,0x1f,0xe0}, {0x7a,0x22,0x1f,0x20}, {0x7a,0x32,0x1f,0x20}, {0x7a,0x6c,0xf1,0x0c},
  {0x7a,0x6c,0x8f,0x60}, {0x68,0x20,0xf0,0x00}, {0x68,0x20,0x0f,0x10}, {0x68,0x21,0xf0,0x02},
  {0x68,0x21,0x0f,0x40}, {0x68,0x22,0xe0,0x06}, {0x68,0x22,0x1f,0x20}, {0x68,0x23,0xfc,0x01},
  {0x68,0x23,0x83,0x28}, {0x68,0x24,0xe0,0x0c}, {0x68,0x24,0x1f,0xc0}, {0x68,0x25,0xfc,0x01},
  {0x68,0x25,0x83,0x40}, {0x68,0x26,0x1f,0xc0}, {0x68,0x27,0xfc,0x02}, {0x68,0x27,0x83,0x5c},
  {0x68,0x28,0xf0,0x00}, {0x68,0x28,0x0f,0x00}, {0x68,0x29,0xf0,0x01}, {0x68,0x29,0x0f,0x30},
  {0x68,0x2a,0xf0,0x04}, {0x68,0x2a,0x0f,0x70}, {0x68,0x2b,0xf0,0x09}, {0x68,0x2b,0x0f,0xc0},
  {0x7a,0x85,0xfb,0x04}, {0x7b,0x34,0xf0,0x07}, {0x7b,0x35,0xf0,0x0d}, {0x7b,0x26,0xf0,0x0a},
  {0x7b,0x36,0xf0,0x07}, {0x7b,0x06,0x0f,0x70}, {0x7b,0x16,0x0f,0x70}, {0x7b,0x26,0x0f,0x70},
  {0x7b,0x36,0x0f,0x70}, {0x7b,0x37,0xe0,0x0f}, {0x7b,0x46,0xf7,0x00}, {0x7b,0x46,0xef,0x00},
  {0x7b,0x46,0xdf,0x00}, {0x7b,0x46,0xbf,0x00}, {0x7b,0x1d,0xf0,0x00}, {0x7b,0x2d,0xf0,0x00},
  {0x7b,0x3d,0xf0,0x0a}, {0x7b,0x1a,0xf0,0x04}, {0x7b,0x3a,0xf0,0x0d}, {0x7b,0x1b,0xc0,0x0a},
  {0x7b,0x3b,0xc0,0x0f}, {0x7b,0x0c,0xf8,0x07}, {0x7b,0x1c,0xf8,0x07}, {0x7b,0x02,0x1f,0xe0},
  {0x7b,0x22,0x1f,0x20}, {0x7b,0x32,0x1f,0x20}, {0x7b,0x6c,0xf1,0x0c}, {0x7b,0x6c,0x8f,0x60},
  {0x6c,0x20,0xf0,0x00}, {0x6c,0x20,0x0f,0x10}, {0x6c,0x21,0xf0,0x02}, {0x6c,0x21,0x0f,0x40},
  {0x6c,0x22,0xe0,0x06}, {0x6c,0x22,0x1f,0x20}, {0x6c,0x23,0xfc,0x01}, {0x6c,0x23,0x83,0x28},
  {0x6c,0x24,0xe0,0x0c}, {0x6c,0x24,0x1f,0xc0}, {0x6c,0x25,0xfc,0x01}, {0x6c,0x25,0x83,0x40},
  {0x6c,0x26,0x1f,0xc0}, {0x6c,0x27,0xfc,0x02}, {0x6c,0x27,0x83,0x5c}, {0x6c,0x28,0xf0,0x00},
  {0x6c,0x28,0x0f,0x00}, {0x6c,0x29,0xf0,0x01}, {0x6c,0x29,0x0f,0x30}, {0x6c,0x2a,0xf0,0x04},
  {0x6c,0x2a,0x0f,0x70}, {0x6c,0x2b,0xf0,0x09}, {0x6c,0x2b,0x0f,0xc0}, {0x7b,0x85,0xfb,0x04},
  {0x78,0x87,0xe0,0x10}, {0x78,0x88,0xe0,0x08}, {0x78,0x07,0x1f,0xe0}, {0x78,0x17,0x1f,0xe0},
  {0x78,0x37,0x1f,0x40}, {0x78,0x12,0x1f,0xa0}, {0x78,0x2c,0xf8,0x07}, {0x60,0x26,0xe0,0x11},
  {0x79,0x87,0xe0,0x10}, {0x79,0x88,0xe0,0x08}, {0x79,0x07,0x1f,0xe0}, {0x79,0x17,0x1f,0xe0},
  {0x79,0x37,0x1f,0x40}, {0x79,0x12,0x1f,0xa0}, {0x79,0x2c,0xf8,0x07}, {0x64,0x26,0xe0,0x11},
  {0x7a,0x87,0xe0,0x10}, {0x7a,0x88,0xe0,0x08}, {0x7a,0x07,0x1f,0xe0}, {0x7a,0x17,0x1f,0xe0},
  {0x7a,0x37,0x1f,0x40}, {0x7a,0x12,0x1f,0xa0}, {0x7a,0x2c,0xf8,0x07}, {0x68,0x26,0xe0,0x11},
  {0x7b,0x87,0xe0,0x10}, {0x7b,0x88,0xe0,0x08}, {0x7b,0x07,0x1f,0xe0}, {0x7b,0x17,0x1f,0xe0},
  {0x7b,0x37,0x1f,0x40}, {0x7b,0x12,0x1f,0xa0}, {0x7b,0x2c,0xf8,0x07}, {0x6c,0x26,0xe0,0x11},
  {0x78,0x0b,0xc0,0x16}, {0x78,0x2a,0xf0,0x06}, {0x79,0x0b,0xc0,0x16}, {0x79,0x2a,0xf0,0x06},
  {0x7a,0x0b,0xc0,0x1c}, {0x7a,0x2a,0xf0,0x06}, {0x7b,0x0b,0xc0,0x1c}, {0x7b,0x2a,0xf0,0x06},
};

/*=== USB4 IRQ / Router-Op Init ===*/

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

  /* Stock 8e6f-8e74 (CROSS-REGISTER): C21F = (C2A8 & 0x3F) | 0x40 — c343() reads the lane-A
   * rate-START register C2A8 (read-only here) and the LOW 6 bits are committed (with bit6 set)
   * into the PHY link-control register C21F. Handmade previously wrote C2A8 itself (wrong target)
   * and never set C21F's final value -> lane-A PHY link-ctrl mis-seeded. */
  REG_PHY_LINK_CTRL_C21F = (REG_PHY_LANEA_RATE_START_C2A8 & 0x3F) | 0x40;
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
  /* Stock 8ef9-8f03 (CROSS-REGISTER, via c32d): C2CE first gets the transient (C2CE & 0xE3)|0x14,
   * then c32d reads the lane-B rate-START register C328 (read-only here) and the caller commits
   * C2CE = (C328 & 0x3F) | 0x40. Handmade previously left C2CE at the transient value and wrote
   * C328 itself (stock never writes C328) -> lane-A C2CE CDR-config + lane-B rate-START mis-seeded. */
  REG_PHY_LANEA_C2CE = (REG_PHY_LANEA_C2CE & 0xE3) | 0x14;        /* stock c32d transient write */
  REG_PHY_LANEA_C2CE = (REG_PHY_LANEB_RATE_START_C328 & 0x3F) | 0x40;  /* stock 8f03 final commit */

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
  REG_TIMER_CTRL_CC3B = (REG_TIMER_CTRL_CC3B & 0xFD) | 0x03;
  usb4_irq_ef24();
  usb4_irq_ef1e();
}

/* USB4 CM router-op RX-enable: enables the router-op engine and SB-transport RX. */
static void usb4_routerop_init(void) {
  REG_ROUTEROP_ENGINE_CTRL_EC00 &= 0xFE;
  phy_cc10_cmd_wait(0, 0, 9);   /* stock e56f: CC12=0(R4), CC13=9(R5) */
  REG_ROUTEROP_ENGINE_CTRL_EC00 = (REG_ROUTEROP_ENGINE_CTRL_EC00 & 0xFE) | 0x01;
  REG_ROUTEROP_SPEED_LO_EA88 = 100;
  REG_ROUTEROP_SPEED_HI_EA89 = 0x24;
  REG_NVME_EVENT_ACK = 1;
  REG_ROUTEROP_CFG_EC05 &= 0xFE;
  REG_INT_DMA_CTRL &= 0xBF;
  REG_INT_DMA_CTRL = (REG_INT_DMA_CTRL & 0x7F) | 0x80;
  u4_routerop_mbox_state = RMBOX_IDLE;
}

/*=== USB4 Lane-Mode Connect Engine ===*/

/*
 * USB4 PHY / lane-mode bring-up engine.
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

#endif /* USB4_H */
