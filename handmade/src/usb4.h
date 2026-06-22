#ifndef USB4_H
#define USB4_H
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

/* Sticky bitmap of USB4 INT sources seen (bit0=SB, bit1=evt, bit2=routerop, bit3=tunnel). */
static volatile uint8_t __xdata __at(0x0B49) usb4_int_seen;

static void sb_router_event_handler(void);

/* Sticky accumulator of every C80A value seen in the ISR. */
static volatile uint8_t __xdata __at(0x0B4A) c80a_acc;

/* ===========================================================================================
 * c0a5 = cm_routerop_mailbox (CODE_BANK1::c0a5) — the INBOUND host router-op RESPONDER.
 *
 * EC06.0 (REG_NVME_EVENT_STATUS) fires when the host's USB4 Connection-Manager posts an in-band
 * router-op (e.g. the route=1 ROUTER_CS_0 vid/did read the host issues during enumeration). The
 * inbound TLP lands in the EA80-mailbox: EA90==0x5A magic, EA80=generation/path opcode (0xE2 =
 * CONFIG read/write path), EA81=sub-opcode (0x50 READ / 0x51 WRITE), EA82-85 = 32-bit config addr.
 *
 * c0a5 must (a) read EA81 sub-opcode, (b) latch EA82-85 -> the config target, (c) READ our router's
 * config-space image (from the SPI-flash config shadow) and (d) DMA the response out the tunnel TX
 * channel (0xEA) + ack EA90=0xA5. The DECISIVE differential (emulate/diff_trace.py --inject-route1,
 * 2026-06-22) proved the OLD handmade c0a5 was a STUB: it latched EA80 then RETURNED — it never read
 * EA81/EA82, never set C805 (reply trigger), never armed C8B0/C8B8 (DMA send), never wrote EA90=0xA5
 * (ack). Stock ran the full 56-MMIO-event send; handmade ran 11 and stopped. This is the byte-true
 * port of the stock E2 CONFIG-READ send path (d945 + helpers), with the config-shadow scratch
 * RELOCATED off stock's 0x0AAD/0x0AB1/0x0AB2 (those alias handmade's live sb_fsm_state/phy cells).
 * ========================================================================================== */

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
  { static __xdata uint8_t rop_bdg = 30;
    if (rop_bdg) { rop_bdg--; uart_puts("[rop ea80="); uart_puthex(REG_ROUTEROP_OPCODE_EA80);
      uart_puts(" ea81="); uart_puthex(REG_ROUTEROP_CFG_EA81);
      uart_puts(" st="); uart_puthex((uint8_t)u4_routerop_mbox_state); uart_putc(']'); } }

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

/* ===========================================================================================
 * c105 = usb4_sec_adapter_link_event_c80a4 (INT1 C80A.4 handler) — MINIMAL TUNNEL+WIDTH PORT.
 *
 * Stock c105 (CODE:c105) demux (byte-true, see Ghidra):
 *   1) rd P1[0x1407]; if .0 (WIDTH evt)  -> a522 link-width service (W1C P1[0x1203].7; 0x09FA|=4)
 *   2) rd P1[0x1407]; if .3 (TUNNEL evt) -> bank1 d855: W1C-loop P1[0x1508] bits 4/3/2/1, each
 *        bit dispatches a tunnel-event service (e4ea Enable / ee29 DisPath / e76b UPS_Rst_Deassert).
 *   3) rd P1[0x1603]; if .0 (evt0) -> W1C P1[0x1603]=1; if 0x09FA.1: u4_entered_usb_mode=1 + ca0d;
 *        e74e (CC re-arm); pd_cm_dispatch_sel(0x07FF)=0x69; return.
 *   4) else if P1[0x1603].1 (evt1) -> W1C P1[0x1603]=2; deeper reconfig (data/state dependent).
 *
 * TUNNEL LEGS (2026-06-22): d855's e4ea(Enable)/e76b(UPS_Rst_Deassert)/ee29(DisPath)/Assert legs
 * are now PORTED (u4lb_d855 in usb4_lanebond.h) and HW-VERIFIED SAFE — at handmade's post-CL0 they
 * run the C659/PERST/PHY re-drive WITHOUT triggering Abr2 (the bond stays at L1:CL0 across runs),
 * exactly as stock does here. Previously this port plain-W1C-drained P1[0x1508] WITHOUT the legs, so
 * the tunnel PCIe link never re-drove and the PCIe-DOWN adapter width never trained (P1[0x1201]
 * stuck 0). On HW handmade now fires C80A.4 -> c105 -> d855 with P1[0x1508]=0x06 (bits 1,2 =
 * UPS_Rst Assert/Deassert) and runs those legs cleanly. REMAINING WALL (upstream, not this handler):
 * the host requests RESET (1508.1/.2), never ENABLE (1508.4) — so e4ea never runs and the width
 * still doesn't walk; the host won't post the Enable until it accepts the adapter config it reads
 * over the tunnel (adapter-CS P1[0x1335]=07 vs stock 02 / P1[0x1206]=78 vs 58 still diverge). */
static volatile uint8_t __xdata __at(0x0B56) c105_fire_bdg;   /* free cell per MEMORY headroom map */
static void u4lb_e74e(void);                                   /* defined in usb4_lanebond.h (incl. after) */
static void u4lb_a310(uint8_t cur);                             /* descriptor-engine helper (usb4_lanebond.h) */
static void u4lb_e890(uint8_t ctrl_low6);                       /* descriptor-engine commit (usb4_lanebond.h) */
static void u4lb_d855(uint8_t heavy);                          /* tunnel-event dispatch (usb4_lanebond.h) */
static void u4lb_d90e_link_phy_reconfig(void);                 /* a522 width-event PHY reconfig + Deassert */

static void usb4_sec_adapter_link_event_c105(void) {
  uint8_t p1407 = P1_RD(0x1407);
  uint8_t p1508 = P1_RD(0x1508);
  uint8_t p1603 = P1_RD(0x1603);

  rl_log(3);   /* ring-log: c105 width-event handler entry (real-time 1407.0 snapshot) */

  if (c105_fire_bdg) { c105_fire_bdg--;
    uart_puts("\r\n[c105 1407="); uart_puthex(p1407);
    uart_puts(" 1508="); uart_puthex(p1508);
    uart_puts(" 1603="); uart_puthex(p1603);
    uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
    uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
    uart_puts(" 9FA="); uart_puthex(u4_route_mode); uart_putc(']'); }

  /* (1) WIDTH event: W1C P1[0x1203].7 + flag USB4-ready (a522-lite, link-safe). */
  if (p1407 & 0x01) {
    if (P1_RD(0x1203) & 0x80) {
      P1_WR(0x1203, 0x80);                 /* W1C width-change pending */
      if ((u4_connect_gate & 0x10) && (u4_route_mode & 0x81)) u4_route_mode |= 0x04;
    }
    /* a522 a578 sub-leg: P1[0x124e].1 width-set -> ... -> P1[0x1243].7 -> d90e PHY reconfig +
     * [PcieTunnel-Deassert]. This is the device-side wire event the TB4 host waits for pre-commit. */
    if (P1_RD(0x124E) & 0x02) {
      P1_WR(0x124E, 0x02);                 /* W1C the lane-width-set sub-event */
      u4lb_a310(0x35);
      eng_a2df(0x36, 0x03);
      u4lb_e890(0x03);
      if (P1_RD(0x1243) & 0x80) u4lb_d90e_link_phy_reconfig();
    }
  }

  /* (2) TUNNEL event (P1[0x1407].3): run the stock d855 leg dispatch — this DRIVES the tunnel-side
   * PCIe link (PERST deassert + PHY RXPLL re-trigger) so the PCIe-DOWN adapter trains its width
   * (P1[0x1201] 0->1->2) and raises the width event. Previously handmade plain-W1C-drained 1508
   * WITHOUT the legs, so the link never re-drove and the width never walked -> no GPU.
   *
   * assert=0 (2026-06-22, HW-PROVEN): the stock d855 1508.1 UPS_Rst_Assert heavy block (B402/PHY/
   * PERST re-drive that prints "UPS_Rst_Assert]") is gated `if (param_5==0) return;` and on the
   * c105->d855 path STOCK NEVER RUNS IT (d855 trace: stock prints PwrOn, Deassert, PcieLinkUp,
   * Enable, UPS_Rst_Deassert — but NEVER UPS_Rst_Assert; param_5 is an uninit reg that is 0 here).
   * Handmade previously passed assert=1, ran the heavy block, and it DROPPED THE LANE BOND
   * (A=0202->0707) on the FIRST reset round — so handmade re-bonded forever and never survived to
   * the host's Enable round. With assert=0 the leg does only W1C(1508.1)+ee94 (non-destructive),
   * matching stock, so the bond stays up across the RESET rounds and the host can post Enable. */
  if (p1407 & 0x08) {
    u4lb_d855(0);
  }

  /* (3) adapter evt0 -> W1C + the 0x69 dispatch token (link-safe, NO C659/PERST). */
  if (p1603 & 0x01) {
    P1_WR(0x1603, 0x01);
    if (u4_route_mode & 0x02) {
      if (REG_POWER_STATUS & 0x40) u4_entered_usb_mode = 1;
      u4lb_e74e();
      pd_cm_dispatch_sel = 0x69;
    }
  } else if (p1603 & 0x02) {
    P1_WR(0x1603, 0x02);                   /* W1C evt1; deeper reconfig omitted (data-dependent) */
  }
}

/* Called from int1_isr after PD-RX: acks and forwards each fired USB4 INT source. */
static void usb4_int_demux(void) {
  uint8_t int_sources = REG_INT_PCIE_NVME;
  c80a_acc |= int_sources;
  rl_log(1);   /* ring-log: every USB4 INT1 demux pass (real-time connect->bond ISR timeline) */
  if (int_sources & 0x20) {
    usb4_int_seen |= 0x01;
    sb_router_event_handler();
  }
  if (int_sources & 0x10) {                /* C80A.4 = secondary adapter/link event (c105) */
    usb4_int_seen |= 0x02;
    usb4_sec_adapter_link_event_c105();
  }
  if (REG_NVME_EVENT_STATUS & 0x01) {
    usb4_int_seen |= 0x04;
    { static __xdata uint8_t ec06_bdg = 40;   /* gate-independent: does the host post router-ops post-bond? */
      if (ec06_bdg) { ec06_bdg--; uart_puts("\r\n[EC06 ea90="); uart_puthex(REG_SYS_CTRL_EA90);
        uart_puts(" ea80="); uart_puthex(REG_ROUTEROP_OPCODE_EA80);
        uart_puts(" ea81="); uart_puthex(REG_ROUTEROP_CFG_EA81); uart_putc(']'); } }
    REG_NVME_EVENT_ACK = 1;
    cm_routerop_mailbox();
  }
  if (int_sources & 0x0F) {
    usb4_int_seen |= 0x08;
    { uint8_t tunnel_status = REG_PHY_RXPLL_TRIGGER;
      if (tunnel_status & 0x04) REG_PHY_RXPLL_TRIGGER = 0x04;
      if (tunnel_status & 0x08) REG_PHY_RXPLL_TRIGGER = 0x08; }
  }
}

#endif /* USB4_H */
