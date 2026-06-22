#ifndef PD_H
#define PD_H
/*
 * USB-PD / Type-C CC-controller driver: PD attach + power-contract bring-up.
 * Touches the PD engine (E4xx), CC controller (CCxx), interrupt-enable group and PD state RAM.
 */
#include "types.h"
#include "registers.h"

#define PR(a) XDATA_REG8V(a)

/* Set once the host engages PD (a PD message is received). */
static volatile uint8_t __xdata __at(0x0B45) pd_seen;

static void pd_rx_message_dispatch(void);
/* Set if any CC-controller command wait timed out. */
static volatile uint8_t __xdata __at(0x0B46) pd_cc_timeout;

#define PD_WAIT_LIMIT 0x4000u
/* Bounded poll until (*reg & mask) matches set (1=wait-for-set, 0=wait-for-clear). */
static void pd_wait(volatile __xdata uint8_t *reg, uint8_t mask, uint8_t set) {
  uint16_t iters = 0;
  if (set) { while (!(*reg & mask) && ++iters < PD_WAIT_LIMIT); }
  else     { while ( (*reg & mask) && ++iters < PD_WAIT_LIMIT); }
  if (iters >= PD_WAIT_LIMIT) pd_cc_timeout = 1;
}

/* RDO/CRC timing constants for the PD engine. */
static void pd_da51(void) {
  REG_CMD_CONFIG = (REG_CMD_CONFIG & 0x7F) | 0x80;
  if (REG_CMD_CONFIG & 0x80) {
    REG_CMD_CFG_E401 = (REG_CMD_CFG_E401 & 0xF8) | 0x04;
    REG_CMD_CFG_E401 = (REG_CMD_CFG_E401 & 0x07) | 0xB0;
    REG_CMD_CFG_E406 = (REG_CMD_CFG_E406 & 0xF0) | 0x06;
    REG_CMD_CFG_E406 = (REG_CMD_CFG_E406 & 0x0F) | 0xA0;
    REG_CMD_CFG_E407 = (REG_CMD_CFG_E407 & 0xE0) | 0x15;
    REG_CMD_CFG_E408 = (REG_CMD_CFG_E408 & 0xE0) | 0x1C;
  }
}

/* Program CC Rp/Rd termination and arm the PD command/RX engine so the host sees a sink attach. */
static void cc_pd_phy_term_init(void) {
  REG_CMD_CONFIG = (REG_CMD_CONFIG & 0xBF) | 0x40;
  REG_CMD_CFG_E40A = 0x0F;
  REG_CMD_CFG_E413 &= 0xFE;
  REG_CMD_CFG_E413 &= 0xFD;
  REG_CMD_CTRL_E400 &= 0x7F;
  REG_XFER_DMA_CTRL &= 0xF8; REG_XFER_DMA_ADDR_LO = 0;
  REG_XFER_DMA_ADDR_HI = 0x0A; REG_XFER_DMA_CMD = 0x01;
  pd_wait(&REG_XFER_DMA_CMD, 0x02, 1);
  REG_XFER_DMA_CMD = 0x02;
  REG_CMD_CONFIG = (REG_CMD_CONFIG & 0xFE) | 0x01;
  REG_XFER_DMA_CTRL &= 0xF8; REG_XFER_DMA_ADDR_LO = 0;
  REG_XFER_DMA_ADDR_HI = 0x3C; REG_XFER_DMA_CMD = 0x01;
  pd_wait(&REG_XFER_DMA_CMD, 0x02, 1);
  REG_XFER_DMA_CMD = 0x02;
  pd_wait(&REG_CMD_STATUS_E402, 0x08, 0);
  REG_CMD_CTRL_E409 &= 0xFE;
  REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xBF) | 0x40;
  REG_CMD_TRIGGER = 0x40;
  REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x06;
  REG_CMD_CTRL_E400 = (REG_CMD_CTRL_E400 & 0xBF) | 0x40;
  REG_CMD_CFG_E411 = 0xA1;
  REG_CMD_CFG_E412 = 0x79;
  REG_CMD_CTRL_E400 = (REG_CMD_CTRL_E400 & 0xC3) | 0x3C;
  REG_CMD_CTRL_E409 &= 0x7F;
  REG_INT_CTRL = (REG_INT_CTRL & 0xDF) | 0x20;
  pd_da51();
  REG_CMD_CFG_E40E = 0x8A;
  REG_CMD_CTRL_E400 = (REG_CMD_CTRL_E400 & 0x7F) | 0x80;
  REG_CMD_CONFIG &= 0xFE;
  REG_PD_CTRL_E66A &= 0xEF;
  REG_CMD_CFG_E40D = 0x28;
  REG_CMD_CFG_E413 = (REG_CMD_CFG_E413 & 0x8F) | 0x60;
  REG_CMD_ARM_CAC4 &= 0xFE;
  REG_CMD_CONFIG &= 0xDF;
  REG_PHY_LINK_ARM_C698 &= 0xDF;
}

/* Clear and enable the CC attach/role event sources. */
static void cc_ctrl_enable_events(void) {
  REG_CPU_INT_CTRL = 0x04; REG_CPU_INT_CTRL = 0x02;
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_CPU_CTRL_CC80 &= 0xEF;
  REG_CPU_CTRL_CC80 = (REG_CPU_CTRL_CC80 & 0xF8) | 0x03;
  REG_XFER_DMA_CFG = 0x04; REG_XFER_DMA_CFG = 0x02;
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_CPU_DMA_READY &= 0xEF;
  REG_CPU_DMA_READY = (REG_CPU_DMA_READY & 0xF8) | 0x04;
}

/* Reset the PD policy-engine state block, set substate=init, seed timers, enable CC events. */
static void pd_internal_state_init(void) {
  uart_puts("[InternalPD_StateInit]");
  pd_tx_staged_pending = 0; pd_contract_state = 0;
  pd_tx_msgid_counter = 0; pd_tx_msg_len = 0;
  pd_selected_pdo_idx = 0;
  pd_pdo_selection_valid = 0;
  pd_rx_num_data_obj = 0;
  pd_rx_slot_idx = 0;
  pd_state_07e3 = 0;
  pd_msg_substate = 1;
  pd_rx_slot_mask = (REG_CMD_CTRL_E400 & 0x40) ? 0x10 : 0x01;
  if (pd_softreset_pending == 0) pd_sop_field = 2;
  pd_softreset_pending = 0; pd_hardreset_done = 0;
  u4_connect_route_latch = 0; u4_enter_usb_accepted = 0;
  u4_route_confirm_07cc = 0;
  pd_state_07cb = 0;
  u4_confirm_input_cd = 0; u4_confirm_input_ce = 0; u4_confirm_input_cf = 0;
  u4_connect_pending = 0;
  pd_bist_mode = 0;
  pd_usb3_fallback_flag = 0;
  pd_role_state = 0;
  cc_ctrl_enable_events();
  pd_timer_e = 5; pd_timer_f = 0; pd_timer_g = 0;
  pd_timer_a = 1; pd_timer_b = 0x2C; pd_timer_c = 0; pd_timer_d = 0x64;
}

/* Transmit a USB-PD HARD RESET to force the host to re-send Source_Cap. No-op once USB4 is up. */
static void pd_drive_hard_reset(void) {
  uint8_t link_mode = (REG_PHY_MODE_E302 & 0x30) >> 4;
  uint16_t guard;
  uart_puts("[CC_state=");
  uart_puthex(link_mode);
  if (link_mode == 3) {
    uart_puts("][CCOpen_neednt_HardRst]");
    return;
  }
  uart_puts("][Drive_HardRst]");
  { uint8_t i; for (i = 0; i < 0x20; i++) PR(0xE420 + i) = 0; }
  pd_internal_state_init();
  REG_PHY_EVENT_E40F = 0xFF; REG_PHY_INT_STATUS_E410 = 0xFF;
  REG_CMD_CONFIG &= ~0x0E;
  REG_XFER_DMA_CTRL = (REG_XFER_DMA_CTRL & 0xF8) | 0x02;
  REG_XFER_DMA_ADDR_LO = 0; REG_XFER_DMA_ADDR_HI = 0xC7; REG_XFER_DMA_CMD = 0x01;
  pd_wait(&REG_XFER_DMA_CMD, 0x02, 1);
  REG_XFER_DMA_CMD = 0x02;
  REG_CMD_CONFIG |= 0x0E;
  REG_CMD_CTRL_E403 = 0x00; REG_CMD_CFG_E404 = 0x40;
  REG_CMD_CFG_E405 = (REG_CMD_CFG_E405 & 0xF8) | 0x05;
  REG_CMD_STATUS_E402 = (REG_CMD_STATUS_E402 & 0x1F) | 0x20;
  for (guard = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && guard < 0x4000; guard++);
  REG_CMD_BUSY_STATUS |= 0x01;
  for (guard = 0; (REG_CMD_BUSY_STATUS & 0x01) && guard < 0x4000; guard++);
  pd_hardreset_done = 1;
}

/* Route the C806/C80A PD/system interrupt aggregate to the 8051 EX1 line so INT1 fires. */
static void pd_int1_enable_group(void) {
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_INT_STATUS_C800 = (REG_INT_STATUS_C800 & 0xFB) | 0x04;
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF8) | 0x06;
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;
  REG_INT_STATUS_C800 |= 0x01;
}

/* Top-level keystone bring-up: enable INT1, force USB4 mode, init PD PHY + state. */
static void pd_keystone_init(void) {
  pd_int1_enable_group();
  u4_mode_flag = 0x87;  /* 0x87 = USB4 tunnel route + VDM-ACK */
  cc_pd_phy_term_init();
  pd_internal_state_init();
}

/* PD-interrupt handler: priority demux over the PD RX event registers E40F/E410, W1C-acking each. */
static void pd_rx_isr(void) {
  uint8_t e40f = REG_PHY_EVENT_E40F;
  uart_puts("\n[PD_int:");
  uart_puthex(e40f);
  uart_putc(':');
  uart_puthex(REG_PHY_INT_STATUS_E410);
  uart_putc(']');
  if (e40f & 0x80) {                 /* Soft_Rst_Int */
    uart_puts("[Soft_Rst_Int]");
    REG_PHY_EVENT_E40F = 0x80;
  } else if (e40f & 0x01) {          /* message received */
    REG_PHY_EVENT_E40F = 0x01;
    pd_seen = 1;
    pd_rx_message_dispatch();
  } else if (e40f & 0x20) {          /* Hard_Rst_Int */
    REG_PHY_EVENT_E40F = 0x20;
    uart_puts("[Hard_Rst_Int]");
  } else {
    uint8_t e410 = REG_PHY_INT_STATUS_E410;
    if      (e410 & 0x01) REG_PHY_INT_STATUS_E410 = 0x01;
    else if (e410 & 0x08) REG_PHY_INT_STATUS_E410 = 0x08;
    else if (e410 & 0x10) REG_PHY_INT_STATUS_E410 = 0x10;
    else if (e410 & 0x20) REG_PHY_INT_STATUS_E410 = 0x20;
    else if (e410 & 0x40) REG_PHY_INT_STATUS_E410 = 0x40;
    else if (e410 & 0x80) REG_PHY_INT_STATUS_E410 = 0x80;
  }
  if (REG_DEBUG_STATUS_E314 & 0x01) REG_DEBUG_STATUS_E314 = 0x01;
}

/* USB4 mode-entry latch the host waits on (defined in vdm.h, #included after pd.h). */
static uint8_t usb4_mode_entry_commit(void);

/* CC23.1 re-init / SB-reconnect event. */
static void cc_cc23_reinit_event(void) {
  u4_connect_gate_e8 = 0x00;
  u4_reinit_pending = 0x01;
}

/* Type-C error-recovery (diagnostic print only). */
static void cc_state_full_reset(void) {
  uart_puts("[Error_Recovery]\n");
}

/* CC81 |= 4 then drive a hard reset. */
static void pd_cc81_hard_reset_4(void) {
  REG_CPU_INT_CTRL = 0x04;
  pd_drive_hard_reset();
}

/* Enqueue a PD control message. */
static void pd_queue_ctrl_msg(uint8_t code) {
  u4_routerop_op_len = code;
  REG_PHY_LINK_CTRL = 0x00;
}

/* CC99 default branch. */
static void cc_cc99_default_event(void) {
}

/* CCF9.1 sub-demux on 0x0A9D (copied from 0x0B1B). */
static void cc_ccf9_subdemux(void) {
  u4_routerop_desc0 = cc_subdemux_src;
}

/* INT1 timer-tick PD/USB4 policy-engine tick: services the 6 CC per-channel event regs (bit1=event). */
/* tick_seen counts tick entries; cc_hit is a bitmask of which CC channels ever showed bit1. */
static volatile uint8_t __xdata __at(0x0B47) tick_seen;
static volatile uint8_t __xdata __at(0x0B48) cc_hit;

static void cc_pd_timer_tick(void) {
  tick_seen++;
  if (REG_TIMER3_CSR & 0x02) cc_hit |= 0x01;
  if (REG_CPU_INT_CTRL & 0x02) cc_hit |= 0x02;
  if (REG_CPU_DMA_INT & 0x02) cc_hit |= 0x04;
  if (REG_XFER_DMA_CFG & 0x02) cc_hit |= 0x08;
  if (REG_XFER2_DMA_STATUS & 0x02) cc_hit |= 0x10;
  if (REG_CPU_EXT_STATUS & 0x02) cc_hit |= 0x20;
  if (REG_TIMER3_CSR & 0x02) {                 /* CC23.1: re-init / SB-reconnect */
    cc_cc23_reinit_event();
    REG_TIMER3_CSR = 0x02;
  }
  if (REG_CPU_INT_CTRL & 0x02) {                 /* CC81.1: CC attach/detach */
    uint8_t substate = pd_msg_substate;
    if (substate == 0x0E || substate == 0x0D) {      /* Data_Reset / Enter_USB pending */
      REG_CPU_INT_CTRL = 0x02;
      if (pd_role_state != 0) pd_queue_ctrl_msg(0x3B);
      cc_state_full_reset();
    } else {
      pd_cc81_hard_reset_4();
      REG_CPU_INT_CTRL = 0x02;
    }
  }
  if (REG_CPU_DMA_INT & 0x02) {                 /* CC91.1: 1s sender-response timeout -> commit USB4 mode */
    REG_CPU_DMA_INT = 0x02;
    uart_puts("[1 sec time out]\n");
    u4_connect_pending = 0x01;
    u4_route_mode = 0x04;
    u4_entered_usb_mode = usb4_mode_entry_commit();
  }
  if (REG_XFER_DMA_CFG & 0x02) {                 /* CC99.1: role-dependent reset */
    uint8_t role = pd_role_state;
    if (role == 0x02) { pd_queue_ctrl_msg(0x3C); pd_drive_hard_reset(); }
    else if (role == 0x03) { pd_queue_ctrl_msg(0xFF); }
    else { cc_cc99_default_event(); REG_XFER_DMA_CFG = 0x02; }
  }
  if (REG_XFER2_DMA_STATUS & 0x02) {                 /* CCD9.1 */
    REG_XFER2_DMA_STATUS = 0x02;
    e461_inflight_token = 0x02;
  }
  if (REG_CPU_EXT_STATUS & 0x02) {                 /* CCF9.1 */
    REG_CPU_EXT_STATUS = 0x02;
    cc_ccf9_subdemux();
  }
}

#endif /* PD_H */
