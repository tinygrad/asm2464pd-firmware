#ifndef CM_TUNNEL_H
#define CM_TUNNEL_H
/* ============================================================================
 * cm_tunnel.h — USB4 Connection-Manager PCIe-tunnel bring-up (STAGE 1+2).
 *
 * Byte-true port of the stock CM tunnel arm + link-up core (see CM_TUNNEL_DESIGN.md):
 *   cm_arm_c00d              <- CODE:c00d
 *   cm_pcie_link_init_state  <- CODE:39e4  (scalar resets; DMA-engine parts TODO -> Stage 4)
 *   cm_pcie_link_step_machine<- CODE:9037  (phases 1-4; per-port adapter walk TODO -> Stage 3)
 *   callees e8a9/e8d9/e5cb/e2a6/e30e/e6fc/e8e4/92bb (all disasm-verified)
 *
 * Include from main.c AFTER usb4_lanebond.h (uses u4lb_a840 / u4lb_e764_rxpll_train).
 * The 0x0A59 super-loop state machine that drives this lives in main.c (CL0-guarded).
 *
 * !! HW WRITES (all gated behind the main-loop CL0 bond-safety guard): PERST(B480),
 *    CA06, B401 tunnel-master, RXPLL/E764, B432/E765 poll, C656/C65B/C659. None fire
 *    until BOTH lanes read CL0 (SB[A0]&0x0F==2 && SB[A1]&0x0F==2). !!
 * ========================================================================== */

/* --- forward decls (mutual recursion e6fc/e8e4/92bb -> c00d) --- */
static void cm_arm_c00d(void);
static void cm_pcie_link_init_state(void);

/* e8a9: C659 &= ~1 (clear lane-ctrl bit0). */
static void cm_e8a9_c659(void) {
  REG_PCIE_LANE_CTRL_C659 = (uint8_t)(REG_PCIE_LANE_CTRL_C659 & 0xFE);
}
/* e8d9: C659 = (C659 & ~1) | 1 (set lane-ctrl bit0). Called with arg.0 set. */
static void cm_e8d9_c659(void) {
  REG_PCIE_LANE_CTRL_C659 = (uint8_t)((REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01);
}
/* e5cb hddpc_ctrl_phy_logic: if !(C656.5) { 0x06E6=1; C656=(C656&0xDF)|0x20; C65B=(C65B&0xDF)|0x20; } */
static void cm_hddpc_phy_logic(void) {
  if ((REG_HDDPC_CTRL & 0x20) == 0) {
    XDATA_REG8V(0x06E6) = 1;
    REG_HDDPC_CTRL = (uint8_t)((REG_HDDPC_CTRL & 0xDF) | 0x20);
    XDATA_REG8(0xC65B) = (uint8_t)((XDATA_REG8V(0xC65B) & 0xDF) | 0x20);
  }
}
/* e2a6 link-up check: (0x07EF==0) && (B432&7)==7 && (E765&2). */
static uint8_t cm_link_up_check(void) {
  if (XDATA_REG8V(0x07EF) != 0) return 0;
  if ((REG_POWER_CTRL_B432 & 0x07) == 0x07 && (REG_SYS_CTRL_E765 & 0x02) != 0) return 1;
  return 0;
}
/* e30e timer4 tick gate: if CC5D.0 ret0; elif CC5D.1 {CC5D=4;CC5D=2;ret1} else {CC5D=4;CC5D=1;ret0}. */
static uint8_t cm_timer4_csr_fired(void) {
  if (REG_TIMER4_CSR & 0x01) return 0;
  if (REG_TIMER4_CSR & 0x02) { REG_TIMER4_CSR = 0x04; REG_TIMER4_CSR = 0x02; return 1; }
  REG_TIMER4_CSR = 0x04; REG_TIMER4_CSR = 0x01; return 0;
}
/* e6fc restart: C656 &= ~0x20; e764 reset pulse(0x0F); 0x06E6=1; c00d(). */
static void cm_e6fc_restart(void) {
  REG_HDDPC_CTRL = (uint8_t)(REG_HDDPC_CTRL & 0xDF);
  boot_phy_e57d_e764_reset_pulse(0x0F);
  XDATA_REG8V(0x06E6) = 1;
  cm_arm_c00d();
}
/* e8e4 settle: u4_reinit_pending=0; hddpc_phy_logic(); c00d(). */
static void cm_e8e4_settle(void) {
  u4_reinit_pending = 0;
  cm_hddpc_phy_logic();
  cm_arm_c00d();
}
/* 92bb tunnel-link-bringup-start: 0x06E6=1; c00d(). */
static void cm_tunnel_link_bringup_start(void) {
  XDATA_REG8V(0x06E6) = 1;
  cm_arm_c00d();
}

/* 39e4: DMA/descriptor RAM + page-04 table + router-op mailbox reset.
 * STAGE 1+2: scalar resets only (byte-true). The DMA-descriptor-engine fills
 * (175d/16c3/15ef) are TODO-gated (Stage 4) — MEMORY warns the bulk-DMA engine
 * breaks SPI READ DMA; link-up-only does not need them. */
static void cm_pcie_link_init_state(void) {
  uint8_t i;
  XDATA_REG8V(0x044B) = 0; XDATA_REG8V(0x0000) = 0;
  REG_DMA_STATUS2 = (uint8_t)(REG_DMA_STATUS2 & 0xF3);   /* C8D8 &= ~0x0C */
  REG_DMA_STATUS2 = (uint8_t)(REG_DMA_STATUS2 & 0xFE);   /* C8D8 &= ~1 */
  REG_DMA_CTRL    = 0;                                   /* C8D7 = 0 */
  REG_DMA_CTRL    = (uint8_t)(REG_DMA_CTRL & 0xF3);
  REG_DMA_CTRL    = (uint8_t)(REG_DMA_CTRL & 0xFE);
  XDATA_REG8V(0x0579) = 0; XDATA_REG8V(0x0464) = 0;
  for (i = 0; i < 4; i++) {
    XDATA_REG8V(0x044E + i) = 0; XDATA_REG8V(0x0452 + i) = 0;
    XDATA_REG8V(0x0456 + i) = 0; XDATA_REG8V(0x045A + i) = 0; XDATA_REG8V(0x0466 + i) = 0;
  }
  for (i = 0; i < 4; i++) XDATA_REG8V(0x057A + i) = 0;
  /* TODO(Stage 4): DMA-descriptor-engine fills + the router-op mailbox 0xFF-clear loop. */
  XDATA_REG8V(0x044D) = 0;
}

/* c00d: the one-shot CM arm. Sets 0x05B4=0x10 (LTSSM trigger) so the step machine
 * runs the link core. Gated by 0x06E6 (arm request). */
static void cm_arm_c00d(void) {
  uint16_t p;
  if (XDATA_REG8V(0x06E6) == 0) return;
  XDATA_REG8V(0x06E6) = 0; XDATA_REG8V(0x06E7) = 1; XDATA_REG8V(0x06E8) = 1;
  XDATA_REG8V(0x05A7) = 0; XDATA_REG8V(0x06EB) = 0;
  XDATA_REG8V(0x05AC) = 0; XDATA_REG8V(0x05AD) = 0; XDATA_REG8V(0x0AF8) = 0;
  REG_PCIE_TUNNEL_CTRL = (uint8_t)(REG_PCIE_TUNNEL_CTRL | 0x01);   /* B401 |= 1 */
  REG_PCIE_TUNNEL_CTRL = (uint8_t)(REG_PCIE_TUNNEL_CTRL & 0xFE);   /* B401 &= ~1 */
  pcie_tunnel_adapter_enable_b401();                              /* cd6c */
  REG_CPU_MODE_NEXT = (uint8_t)(REG_CPU_MODE_NEXT & 0xEF);                 /* CA06 &= ~0x10 */
  REG_PCIE_PERST_CTRL = (uint8_t)((REG_PCIE_PERST_CTRL & 0xFE) | 0x01);    /* B480 PERST assert */
  cm_e8a9_c659();                                                /* C659 &= ~1 */
  boot_phy_d436_width(0x0F);
  XDATA_REG8V(0x06E5) = 0;
  for (p = 0x05B3; p != 0x06E5; p++) XDATA_REG8V(p) = 0;          /* per-port array clear (0x132 B) */
  for (p = 0x05A8; p != 0x05AC; p++) XDATA_REG8V(p) = 0;
  cm_pcie_link_init_state();
  XDATA_REG8V(0x05B4) = 0x10;                                     /* arm the link core */
}

/* ============================================================================
 * B220 PCIe-config-TLP mailbox engine (stock c1f9 + sub-helpers). Issues a PCIe
 * config read/write TLP over the tunnel to the downstream device (the GPU) and
 * returns the result u32 in XDATA[0xB220..0xB223]. This is the core of the
 * per-port adapter walk. cfg address = 0x00D000{token} in XDATA[0x05AF..0x05B2].
 * ========================================================================== */
static void cm_mbox_stage_clear(void) { uint8_t i; for (i = 0; i < 0x0C; i++) XDATA_REG8(0xB210 + i) = 0; }
static void cm_mbox_kick_999d(void) {
  XDATA_REG8(0xB296) = 1; XDATA_REG8(0xB296) = 2; XDATA_REG8(0xB296) = 4; XDATA_REG8(0xB254) = 0x0F;
}
static uint8_t cm_mbox_busy_99eb(void) { return (uint8_t)((XDATA_REG8(0xB296) & 0x04) ? 1 : 0); }
static void cm_mbox_ack_9a95(void) { XDATA_REG8(0xB296) = 4; }
static uint8_t cm_mbox_9a74(void) { XDATA_REG8(0xB296) = 2; return XDATA_REG8(0xB22C); }
/* e762: set cfg addr = 0x00D000{token} -> XDATA[0x05AF..0x05B2] BE. */
static void cm_cfg_addr(uint8_t token) {
  XDATA_REG8V(0x05AF) = 0x00; XDATA_REG8V(0x05B0) = 0xD0;
  XDATA_REG8V(0x05B1) = 0x00; XDATA_REG8V(0x05B2) = token;
}
/* c1f9: cfg-mailbox engine. dir from XDATA[0x05AE]; addr u32 from XDATA[0x05AF..0x05B2].
 * Read result lands at XDATA[0xB220..0xB223]. Returns 0 ok / 0xFE busy-err / 0xFF mismatch. */
static uint8_t cm_c1f9(void) {
  uint8_t a;
  cm_mbox_stage_clear();
  XDATA_REG8(0xB210) = (uint8_t)(XDATA_REG8V(0x05AE) ? 0x40 : 0x00);
  XDATA_REG8(0xB213) = 1;
  XDATA_REG8(0xB217) = 0x0F; XDATA_REG8(0xB216) = 0x20;
  XDATA_REG8(0xB218) = XDATA_REG8V(0x05AF); XDATA_REG8(0xB219) = XDATA_REG8V(0x05B0);
  XDATA_REG8(0xB21A) = XDATA_REG8V(0x05B1); XDATA_REG8(0xB21B) = XDATA_REG8V(0x05B2);
  cm_mbox_kick_999d();
  { uint16_t g = 0; while (cm_mbox_busy_99eb() == 0 && ++g < 0x4000) { } }   /* bounded busy-wait */
  cm_mbox_ack_9a95();
  if (XDATA_REG8V(0x05AE)) return 0;                       /* write -> done */
  { uint16_t g = 0; for (;;) { a = (uint8_t)((XDATA_REG8(0xB296) & 0x02) >> 1); if (a) break;
      if (XDATA_REG8(0xB296) & 1) { XDATA_REG8(0xB296) = 1; return 0xFE; } if (++g >= 0x4000) return 0xFE; } }
  if (cm_mbox_9a74() != 0) return 0xFF;
  if (XDATA_REG8(0xB22D) != 0) return 0xFF;
  if (XDATA_REG8(0xB22B) != 0x04) return 0xFF;
  return 0;
}
static uint8_t cm_cfg_read_commit(void) { XDATA_REG8V(0x05AE) = 0; return cm_c1f9(); }

/* 9037 cm_pcie_link_step_machine: phases 1-4 (restart / timed / link-core / link-up poll).
 * Returns 0 = still working; nonzero = done/error (caller -> SM state 2). The per-port
 * adapter walk (link-up success path) is STAGE 3 — here we latch link-up and report success. */
static uint8_t cm_pcie_link_step_machine(void) {
  uint8_t step, up;
  XDATA_REG8V(0x0B39) = (uint8_t)(XDATA_REG8V(0x0B39) + 1);

  if (XDATA_REG8V(0x0B3B) != 0) { XDATA_REG8V(0x0B3B) = 0; XDATA_REG8V(0x0B3A) = 1; cm_e6fc_restart(); }
  if (XDATA_REG8V(0x0B3A) != 0) {
    step = XDATA_REG8V(0x0B39);
    if (step == 0x15) { cm_e8e4_settle(); return step; }
    if (step < 0x18) return (uint8_t)(step - 0x18);
    XDATA_REG8V(0x0B3A) = 0;
  }

  /* LTSSM bring-up core — BYTE-EXACT (disasm CODE:9074-909c). */
  if (XDATA_REG8V(0x05B4) == 0x10) {
    phy_cc10_cmd_wait(4, 1, 0x2B);   /* CC10=4, CC12=1, CC13=0x2B */
    u4lb_a840(0);                    /* pcie_link_speed_config_b403 (param ignored) */
    u4lb_e764_rxpll_train();         /* e764 RXPLL retrain */
    cm_e8d9_c659();                  /* C659 |= 1 */
    XDATA_REG8V(0x05B4) = 0;
    phy_cc10_cmd_wait(4, 3, 0xE7);   /* CC10=4, CC12=3, CC13=0xE7 */
  }

  up = cm_link_up_check();
  if (up == 0) {
    step = XDATA_REG8V(0x0B39);
    if (step == 0x0A)      XDATA_REG8V(0x0B3B) = 1;       /* request restart */
    else if (step == 0x05) cm_tunnel_link_bringup_start();
    step = XDATA_REG8V(0x0B39);
    if (step < 0x1D) return (uint8_t)(step - 0x1D);
    XDATA_REG8V(0x06E8) = 0;                              /* timeout */
    return 0;
  }

  /* LINK UP. Validate the B220 config-TLP mailbox engine by reading the downstream
   * PCIe config space offset 0 (vendor/device ID). If it returns the AMD GPU id
   * (1002/7590), the tunnel carries PCIe TLPs to the GPU and the engine works — the
   * foundation for the full per-port walk (89db/Phase-A) that flips host PCIeDn->L0. */
  { uint16_t g; uint8_t rc;
    REG_PCIE_PERST_CTRL = (uint8_t)(REG_PCIE_PERST_CTRL & 0xFE);   /* deassert PERST (c00d asserted it) */
    for (g = 0; g < 0x8000; g++) { __asm nop __endasm; }           /* settle for link */
    /* ROUTE PRIME (c5ff subset): bind the 0x00D000xx ECAM aperture to the GPU cfg space by pushing
     * the route descriptor word 0x01D000D0 to the B220 DATA FIFO with FMT=0x40 (write/prime variant),
     * then trigger. Without this the aperture reads 0 (no bus route to the GPU). */
    XDATA_REG8V(0x0A5F) = 0x00; XDATA_REG8V(0x0A60) = 0x00;
    XDATA_REG8V(0x0A61) = 0x00; XDATA_REG8V(0x0A62) = 0x02;        /* cfg-target scratch = 0x02000000 */
    cm_cfg_addr(0x00);                                            /* 0x05AF..0x05B2 = 00 D0 00 00 */
    XDATA_REG8(0xB220) = 0xD0; XDATA_REG8(0xB221) = 0x00;          /* route word 0x01D000D0 (LE) */
    XDATA_REG8(0xB222) = 0xD0; XDATA_REG8(0xB223) = 0x01;
    XDATA_REG8V(0x05AE) = 1; (void)cm_c1f9(); XDATA_REG8V(0x05AE) = 0;  /* FMT=0x40 push+trigger */
    /* ECAM MemRd (FMT=0x00) of 0x00D00000 = GPU cfg dword0. id LE: vid=B221:B220, did=B223:B222. */
    cm_cfg_addr(0x00); rc = cm_cfg_read_commit();
    uart_puts("\r\n[CFGrd rc="); uart_puthex(rc);
    uart_puts(" id="); uart_puthex(XDATA_REG8(0xB221)); uart_puthex(XDATA_REG8(0xB220));
    uart_putc(' '); uart_puthex(XDATA_REG8(0xB223)); uart_puthex(XDATA_REG8(0xB222));
    uart_puts(" 22A="); uart_puthex(XDATA_REG8(0xB22A));
    uart_puts(" ltssm="); uart_puthex(REG_PCIE_LTSSM_STATE); uart_putc(']'); }
  XDATA_REG8V(0x044B) = XDATA_REG8V(0x06E5);
  return 1;
}

#endif /* CM_TUNNEL_H */
