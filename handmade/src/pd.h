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

#define PD_WAIT_LIMIT 0x4000u
/* Bounded poll until (*reg & mask) matches set (1=wait-for-set, 0=wait-for-clear). */
static void pd_wait(volatile __xdata uint8_t *reg, uint8_t mask, uint8_t set) {
  uint16_t iters = 0;
  if (set) { while (!(*reg & mask) && ++iters < PD_WAIT_LIMIT); }
  else     { while ( (*reg & mask) && ++iters < PD_WAIT_LIMIT); }
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

#ifndef HANDMADE_USB4_MODE_FLAGS
#define HANDMADE_USB4_MODE_FLAGS 0x87u
#endif

/* Top-level keystone bring-up: enable INT1, set the USB4 mode policy, init PD PHY + state. */
static void pd_keystone_init(void) {
  pd_int1_enable_group();
  u4_mode_flag = HANDMADE_USB4_MODE_FLAGS;  /* 0x87 = USB4 tunnel route + VDM-ACK */
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
static void cc_pd_timer_tick(void) {
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

/*=== PD Message Dispatcher ===*/

/*
 * USB-PD message dispatcher: PD policy-engine RX path.
 * RX buffer base = 0xE440 + 0x20*slot; TX buffer = 0xE420-0xE43F.
 */

static void pd_tx_commit_engine(void);
static void pd_rx_nak_send(void);
static void pd_ctrl_goodcrc(void);
static void vdm_tx_dispatch(void);
static void pd_handle_enter_usb(void);

/* Stash the 16-bit RX buffer pointer for the current RX slot and return it. */
static uint16_t pd_rx_ptr(void) {
  uint8_t slot = pd_rx_slot_idx;
  uint16_t base = (uint16_t)(0xE440u + (uint16_t)(0x20u * slot));
  pd_rx_ptr_hi = (uint8_t)(base >> 8);
  pd_rx_ptr_lo = (uint8_t)(base & 0xFF);
  return base;
}
/* Re-read the stowed RX buffer pointer. */
static uint16_t pd_rx_ptr_get(void) {
  return ((uint16_t)pd_rx_ptr_hi << 8) | pd_rx_ptr_lo;
}

/* Send the message staged in E420-E43F and bump the TX MessageID. */
static void pd_tx_commit_engine(void) {
  { uint16_t guard; for (guard = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && guard < 0x4000; guard++); }
  REG_CMD_CTRL_E403 = pd_tx_msg_len;
  REG_CMD_BUSY_STATUS = (REG_CMD_BUSY_STATUS & 0xFE) | 0x01;
  { uint16_t guard; for (guard = 0; (REG_CMD_BUSY_STATUS & 0x01) && guard < 0x4000; guard++); }
  pd_tx_msgid_counter = (pd_tx_msgid_counter + 1) & 7;
  pd_tx_staged_pending = 0;
}

/* W1C clear a CC event register (write 4 then 2). */
static void pd_cc_event_clear(uint16_t reg) {
  PR(reg) = 0x04;
  PR(reg) = 0x02;
}
/* Clear the CC attach event (CC81) before a state transition. */
static void pd_e933_clear_cc81(void) { pd_cc_event_clear(0xCC81); }

/* Arm the CC sender-response / PS-transition timer. */
static void pd_arm_cc_timer(uint8_t threshold_hi, uint8_t threshold_lo) {
  REG_CPU_CTRL_CC82 = threshold_hi;
  REG_CPU_CTRL_CC83 = threshold_lo;
  pd_cc_event_clear(0xCC81);
  REG_CPU_INT_CTRL = 0x01;
}

/* Build the PD header in E420/E421: SOP type, NumDataObjects, MessageType. */
static void pd_tx_set_sop_header(uint8_t nobj, uint8_t msgtype) {
  uint8_t sop = pd_sop_field;
  if (sop == 2 || sop == 3) REG_CMD_TRIGGER = 0x80; else REG_CMD_TRIGGER = 0x40;
  REG_CMD_CFG_E405 &= 0xF8;
  REG_CMD_MODE_E421 = (uint8_t)(((nobj & 7) << 4) | ((uint8_t)(pd_tx_msgid_counter << 1) & 0x0E));
  REG_CMD_TRIGGER = (uint8_t)((REG_CMD_TRIGGER & 0xC0) | msgtype);
}

/* Zero the PD TX message buffer E420-E43F. */
static void pd_tx_buf_clear(void) {
  uint8_t i;
  for (i = 0; i < 0x20; i++) PR(0xE420 + i) = 0;
}

/* Select PDO[0] (vSafe5V fixed supply) from the received Source_Cap. */
static void pd_select_pdo_from_source_cap(void) {
  uint16_t pdo0 = (uint16_t)(0xE442u + (uint16_t)(0x20u * pd_rx_slot_idx));
  pd_op_current_lo = PR(pdo0 + 0);
  pd_op_current_hi = PR(pdo0 + 1) & 0x03;
  pd_selected_pdo_idx = 0;
  pd_pdo_selection_valid = 1;
}

/* Build a Request (header + Fixed RDO) and send it, then arm SenderResponse. */
static void pd_build_send_request_rdo(void) {
  uint8_t cur_hi = pd_op_current_hi;
  uint8_t cur_lo = pd_op_current_lo;

  pd_tx_buf_clear();
  pd_tx_set_sop_header(1, 2);

  REG_CMD_PARAM = cur_lo;
  REG_CMD_STATUS = cur_hi;
  REG_CMD_STATUS |= (uint8_t)(cur_lo << 2);
  REG_CMD_ISSUE = (uint8_t)((cur_hi << 2) & 0x0C);
  REG_CMD_ISSUE |= (uint8_t)(cur_lo >> 6);
  REG_CMD_TAG = 1;
  REG_CMD_TAG |= (uint8_t)(((pd_selected_pdo_idx + 1) & 7) << 4);
  REG_CMD_TAG |= 2;
  REG_CMD_TAG &= 0xFE;

  pd_tx_msg_len = 6;
  pd_msg_substate = 3;

  REG_TIMER0_DIV &= 0xF7;
  pd_cc_event_clear(0xCC11);
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | 0x03;
  REG_TIMER0_THRESHOLD_HI = 0;
  REG_TIMER0_THRESHOLD_LO = 0x28;
  REG_TIMER0_CSR = 0x01;
  { uint16_t guard = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++guard < PD_WAIT_LIMIT);
    (void)guard; }
  REG_TIMER0_CSR = 0x02;

  pd_tx_commit_engine();
  pd_arm_cc_timer(2, 0x30);
}

/* ---------- CONTROL-message handlers (NumObj==0) ---------- */

/* GoodCRC: advance the RX slot index; commit any staged pending TX. */
static void pd_ctrl_goodcrc(void) {
  pd_rx_slot_idx = (uint8_t)((pd_rx_slot_idx + 1) & (uint8_t)(pd_rx_slot_mask - 1));
  if (pd_tx_staged_pending != 0) pd_tx_commit_engine();
}

/* Accept: on substate 3 advance to 4 and arm the PS_RDY timer. */
static void pd_ctrl_accept(void) {
  uint8_t substate;
  uart_puts("[Accept]");
  substate = pd_msg_substate;
  if (substate == 3) {
    pd_e933_clear_cc81();
    pd_msg_substate = 4;
    pd_arm_cc_timer(0x10, 0x27);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0) {
    if (pd_softreset_pending != 0) { pd_softreset_pending = 0; }
    pd_e933_clear_cc81();
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    pd_e933_clear_cc81();
    pd_msg_substate = 0x0D;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* PS_RDY: on substate 4, decode the contract voltage into 0x07B8. */
static void pd_ctrl_ps_rdy(void) {
  uint8_t volt_hi, volt_lo;
  uart_puts("[PS_RDY]");
  if (pd_msg_substate != 4) { pd_ctrl_goodcrc(); return; }
  pd_e933_clear_cc81();

  volt_hi = pd_decoded_voltage_hi;
  volt_lo = pd_decoded_voltage_lo;
  if (volt_hi == 1 && volt_lo == 0x2C) {
    uart_puts("[5V3A]");
    pd_contract_state = 3;
  } else {
    uint16_t volt = ((uint16_t)volt_hi << 8) | volt_lo;
    if (volt >= 0x012C || volt_hi >= 1) {
      uart_puts("[5V1.5A]");
      pd_contract_state = 2;
    } else if (volt < 0x0096) {
      pd_contract_state = 1;
    } else {
      pd_contract_state = 4;
    }
  }
  pd_softreset_pending = 0;
  pd_hardreset_done = 0;
  pd_ctrl_goodcrc();
}

/* Reject: arm the timer on substate 3, or latch Data_Reset on 0x0E. */
static void pd_ctrl_reject(void) {
  uint8_t substate;
  uart_puts("[Reject]");
  substate = pd_msg_substate;
  if (substate == 3) {
    pd_e933_clear_cc81();
    if (pd_contract_state == 0) {
    }
    pd_arm_cc_timer(0, 0);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    pd_msg_substate = 0x0E;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* Wait: on substate 3 with an active contract, arm the CC timer and ack. */
static void pd_ctrl_wait(void) {
  if (pd_msg_substate != 3) { pd_ctrl_goodcrc(); return; }
  pd_e933_clear_cc81();
  if (pd_contract_state != 0) {
    pd_arm_cc_timer(0xD0, 0x07);
    { uint16_t guard = 0; while (!(REG_CPU_INT_CTRL & 0x02) && ++guard < PD_WAIT_LIMIT); }
    REG_CPU_INT_CTRL = 0x02;
    pd_ctrl_goodcrc();
  } else {
    pd_ctrl_goodcrc();
  }
}

/* Stage and send a 2-byte control NAK response, then advance the RX slot. */
static void pd_rx_nak_send(void) {
  if (pd_sop_field == 1) {
    pd_tx_set_sop_header(0, 1);
  } else {
    REG_CMD_CFG_E405 &= 0xF8;
  }
  pd_tx_msg_len = 2;
  pd_tx_commit_engine();
  pd_rx_slot_idx = (uint8_t)((pd_rx_slot_idx + 1) & (uint8_t)(pd_rx_slot_mask - 1));
}

/* Soft_Reset: reset both MessageID counters and reply Accept with MessageID=0. */
static void pd_ctrl_soft_reset(void) {
  uart_puts("[Soft_Reset]");
  pd_tx_msgid_counter = 0;
  pd_rx_slot_idx = 0;
  pd_tx_staged_pending = 0;
  pd_e933_clear_cc81();

  pd_tx_buf_clear();
  pd_tx_set_sop_header(0, 3);
  pd_tx_msg_len = 2;

  REG_CMD_MODE_E421 &= 0xF1;

  pd_tx_commit_engine();

  pd_msg_substate = 1;
}

/* Dispatch a CONTROL message by MessageType. */
static void pd_dispatch_control(uint8_t msgtype) {
  switch (msgtype) {
    case 0x01: pd_ctrl_goodcrc();    break;
    case 0x03: pd_ctrl_accept();     break;
    case 0x04: pd_ctrl_reject();     break;
    case 0x06: pd_ctrl_ps_rdy();     break;
    case 0x0C: pd_ctrl_wait();       break;
    case 0x0D: pd_ctrl_soft_reset(); break;
    default:   pd_rx_nak_send();  break;
  }
}

/* ---------- DATA-message handlers (NumObj>0) ---------- */

/* Dispatch a DATA message by MessageType. */
static void pd_dispatch_data(uint8_t msgtype) {
  if (msgtype == 0x01) {
    uint8_t sop;
    uart_puts("[Source_Cap]");
    sop = pd_sop_field;
    if (sop == 2 || sop == 3) {
      REG_CMD_TRIGGER = 0x80;
      REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x04;
    }
    {
      uint8_t substate = pd_msg_substate;
      if (substate != 1 && substate != 5 && substate != 3 && substate != 6) {
        return;
      }
    }
    pd_e933_clear_cc81();
    pd_select_pdo_from_source_cap();
    pd_msg_substate = 2;
    pd_build_send_request_rdo();
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype == 0x03) {
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype >= 2 && msgtype < 8) {
    pd_rx_nak_send();
    return;
  }
  if (msgtype == 0x08) {
    uart_puts("[Enter_USB]");
    pd_handle_enter_usb();
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype == 0x0F) {
    uart_puts("[VDM]");
    vdm_tx_dispatch();
    pd_ctrl_goodcrc();
    return;
  }
  pd_rx_nak_send();
}

/* Entry point: parse the RX header and dispatch CONTROL vs DATA. */
static void pd_rx_message_dispatch(void) {
  uint8_t e40f = REG_PHY_EVENT_E40F;
  uint8_t hdr0, hdr1, sop, ext_bit;
  uint16_t base;

  if ((e40f & 0x80) || (e40f & 0x20)) return;

  base = pd_rx_ptr();
  hdr1 = PR(base + 1);
  hdr0 = PR(base + 0);

  pd_rx_num_data_obj = (uint8_t)((hdr1 >> 4) & 7);
  u4_routerop_op_lo = (uint8_t)((hdr1 >> 1) & 7);
  pd_msg_type = (uint8_t)(hdr0 & 0x1F);
  sop = (uint8_t)(hdr0 >> 6);
  pd_sop_field = sop;

  uart_puts("[D");
  uart_puthex(pd_rx_slot_idx); uart_putc(' ');
  uart_puthex(hdr0); uart_puthex(hdr1); uart_putc(' ');
  uart_puthex(pd_rx_num_data_obj); uart_puthex(pd_msg_type); uart_putc(']');

  ext_bit = (uint8_t)((PR(base + 1) >> 7) & 1);
  if (pd_bist_mode != 0) return;

  if (ext_bit != 0 && sop == 0) {
    pd_rx_nak_send();
    return;
  }

  if (pd_rx_num_data_obj == 0) {
    pd_dispatch_control(pd_msg_type);
  } else {
    pd_dispatch_data(pd_msg_type);
  }
}

/*=== USB4 / Thunderbolt VDM Responder ===*/

/*
 * USB-PD Structured VDM responder + USB4/Thunderbolt mode entry.
 * Included after pd_dispatch.h (PR, uart_*, PD TX engine helpers in scope).
 */

static void usb4_connect_u4(void);

#define VDM_VID_LO        0x4C
#define VDM_VID_HI        0x17
#define VDM_TBT_SVID_LO   0x87
#define VDM_TBT_SVID_HI   0x80

/* Build the VDM-header VDO into the command registers. */
static void pd_vdm_hdr_build(uint8_t cmdtype, uint8_t cmd) {
  REG_CMD_PARAM = (uint8_t)((((uint8_t)(cmdtype << 6)) | cmd) & 0xCF);
  REG_CMD_STATUS = (pd_sop_field == 1) ? 0x80 : 0xA8;
  REG_CMD_ISSUE = 0x00;
  REG_CMD_TAG = 0xFF;
}

/* NAK echoing the received SVID. */
static void vdm_nak(uint8_t cmd, uint8_t svid_lo, uint8_t svid_hi) {
  pd_tx_set_sop_header(1, 0x0F);
  pd_vdm_hdr_build(2, cmd);
  REG_CMD_ISSUE = svid_lo;
  REG_CMD_TAG = svid_hi;
  pd_tx_msg_len = 6;
}

/* Discover_Identity responder: build the ID ACK VDO chain. */
static void vdm_build_discover_id(void) {
  uint8_t sop = pd_sop_field;
  uint8_t mode_bits, gen_bits;

  pd_tx_set_sop_header((sop == 2) ? 5 : 4, 0x0F);
  pd_vdm_hdr_build(1, 1);

  REG_CMD_LBA_0 = VDM_VID_LO;
  REG_CMD_LBA_1 = VDM_VID_HI;
  REG_CMD_LBA_2 = (sop == 2) ? 0x40 : 0x00;
  if (pd_role_state == 0 && (u4_mode_flag & 0x80))
    REG_CMD_LBA_3 = 0x54;
  else
    REG_CMD_LBA_3 = 0x50;

  REG_CMD_COUNT_LOW = 0; REG_CMD_COUNT_HIGH = 0; REG_CMD_LENGTH_LOW = 0; REG_CMD_LENGTH_HIGH = 0; REG_CMD_RESP_TAG = 0;

  REG_CMD_CTRL = pd_product_pid_lo;
  REG_CMD_TIMEOUT = pd_product_pid_hi;

  if (sop == 2) {
    mode_bits = u4_mode_flag;
    gen_bits = mode_bits & 0x03;
    sb_tx_cmd = (gen_bits != 0) ? 3 : 2;
    if (mode_bits & 0x80) sb_tx_cmd |= 0x08;
    if (pd_role_state == 0) REG_CMD_PARAM_L = sb_tx_cmd;
    else                 REG_CMD_PARAM_L = 2;
    REG_CMD_PARAM_H = 0;
    REG_CMD_EXT_PARAM_0 = 0x80;
    if (pd_role_state == 0 && gen_bits != 0) REG_CMD_EXT_PARAM_1 = 0x6D;
    else                                  REG_CMD_EXT_PARAM_1 = 0x65;
  }

  pd_tx_msg_len = (sop == 2) ? 0x16 : 0x12;
}

/* Discover_SVIDs responder: ACK with SVID 0x8087, else NAK. */
static void vdm_build_discover_sids(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  if (((uint8_t)~rx_svid_hi | rx_svid_lo) == 0 && (u4_mode_flag & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);
    pd_vdm_hdr_build(1, 2);
    REG_CMD_LBA_0 = 0x00;
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = VDM_TBT_SVID_LO;
    REG_CMD_LBA_3 = VDM_TBT_SVID_HI;
    pd_tx_msg_len = 0x0A;
    return;
  }
  vdm_nak(2, rx_svid_lo, rx_svid_hi);
}

/* Discover_Modes responder: ACK TBT3 mode for SVID 0x8087, else NAK. */
static void vdm_build_discover_modes(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  if (rx_svid_hi == 0x80 && rx_svid_lo == 0x87 && (u4_mode_flag & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);
    pd_vdm_hdr_build(1, 3);
    REG_CMD_ISSUE = VDM_TBT_SVID_LO;
    REG_CMD_TAG = VDM_TBT_SVID_HI;
    REG_CMD_LBA_0 = 0x01;
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = 0x00;
    REG_CMD_LBA_3 = 0x00;
    pd_tx_msg_len = 0x0A;
    return;
  }
  vdm_nak(3, rx_svid_lo, rx_svid_hi);
}

/* Device-side USB4 mode-entry latch. */
static uint8_t usb4_mode_entry_commit(void) {
  uint8_t mode_flags = u4_mode_flag;
  if (mode_flags & 0x40) {
    u4_mode_entry_class = 3;
    u4_mode_entry_param = 1;
    PR(0x92E1) = 0x10;
    REG_USB_INT_MASK_9090 &= 0x7F;
    return 4;
  }
  u4_mode_entry_class = 1;
  u4_mode_entry_param = ((mode_flags & 0x81) == 0) ? 0x0D : 0x05;
  return 1;
}

/* EnterMode responder: enter TBT alt-mode for SVID 0x8087, else generic ACK. */
static void vdm_handle_enter_mode(uint8_t objpos, uint8_t svid_lo, uint8_t svid_hi) {
  uint16_t vdo0 = (uint16_t)(pd_rx_ptr_get() + 2);
  uint8_t enter_mode_flags = PR(vdo0 + 6);

  sb_tx_cmd = svid_lo;
  sb_tx_byte0 = svid_hi;
  sb_tx_byte1 = objpos;
  u4_confirm_input_cd = (uint8_t)(enter_mode_flags >> 7);
  u4_confirm_input_ce = (uint8_t)((enter_mode_flags & 0x40) >> 6);
  u4_confirm_input_cf = (uint8_t)(enter_mode_flags & 0x07);

  if (svid_lo == 0x87 && svid_hi == 0x80 && (u4_mode_flag & 0x80) && pd_role_state == 0) {
    pd_tx_set_sop_header(1, 0x0F);
    pd_vdm_hdr_build(1, 4);
    pd_tx_msg_len = 6;
    u4_connect_route_latch = 1;
    u4_connect_pending = 1;
    uart_puts("[Enter_TBT]");
    return;
  }
  pd_tx_set_sop_header(1, 0x0F);
  pd_vdm_hdr_build(2, 4);
  REG_CMD_STATUS |= (sb_tx_byte1 & 0x07);
  REG_CMD_ISSUE = svid_lo;
  REG_CMD_TAG = svid_hi;
  pd_tx_msg_len = 6;
}

/* Enter_USB Data Message handler: latch USB4 mode, Accept, or Reject. */
static void pd_handle_enter_usb(void) {
  uint16_t base = pd_rx_ptr();
  uint16_t vdo0 = (uint16_t)(base + 2);
  uint8_t eudo1 = PR(vdo0 + 1);
  uint8_t eudo2 = PR(vdo0 + 2);
  uint8_t eudo3 = PR(vdo0 + 3);
  uint8_t cable_cur = (uint8_t)((eudo1 & 0x20) >> 5);
  uint8_t mode = (uint8_t)((eudo3 & 0x70) >> 4);
  uint8_t mode_flags;

  u4_routerop_flag = (uint8_t)((eudo1 & 0x40) >> 6);
  u4_routerop_opcode = (uint8_t)(eudo1 >> 7);
  pd_usb3_fallback_flag = (uint8_t)(u4_routerop_opcode & 1);
  u4_routerop_op_len = (uint8_t)((eudo2 & 0x06) >> 1);
  pd_state_07cb = (uint8_t)((eudo2 & 0x18) >> 3);
  u4_route_confirm_07cc = (uint8_t)(eudo2 >> 5);
  u4_routerop_port = mode;
  pd_tx_buf_clear();

  mode_flags = u4_mode_flag;
  if ((mode_flags & 0x03) == 0) {
    REG_CMD_CFG_E405 &= 0xF8;
    u4_route_mode = 4;
    usb4_mode_entry_commit();
    u4_entered_usb_mode = mode;
  } else if (mode == 2 && pd_role_state == 0) {
    pd_tx_set_sop_header(0, 3);
    if (cable_cur) {
      u4_connect_pending = 1;
      uart_puts("[Enter_USB 4]");
      u4_enter_usb_accepted = 1;
      u4_route_mode |= 0x04;
      u4_connect_gate_e8 = 1;
    }
  } else {
    pd_tx_set_sop_header(0, 4);
  }

  pd_tx_msg_len = 2;
  pd_tx_commit_engine();

  if (u4_enter_usb_accepted != 0) {
    uart_puts("[Connect_U4]");
    if (u4_connect_oneshot_suppress != 0) {
      u4_connect_oneshot_suppress = 0;
    } else {
      usb4_connect_u4();
    }
  }
}

/* Drive the command interface (opcode 3 = PD-message TX) then commit the buffer. */
static void vdm_tx_strobe_commit(void) {
  REG_XFER_DMA_CTRL = (REG_XFER_DMA_CTRL & 0xF8) | 0x03;
  REG_XFER_DMA_ADDR_LO = 0;
  REG_XFER_DMA_ADDR_HI = 0x50;
  REG_XFER_DMA_CMD = 0x01;
  { uint16_t poll = 0; while (!((REG_XFER_DMA_CMD >> 1) & 1) && ++poll < PD_WAIT_LIMIT);
    (void)poll; }
  REG_XFER_DMA_CMD = 0x02;
  if (REG_PHY_EVENT_E40F & 0x01) return;
  pd_tx_commit_engine();
}

/* Structured VDM command dispatch: parse RX VDO0, respond per command, strobe+commit. */
static void vdm_tx_dispatch(void) {
  uint16_t base = pd_rx_ptr();
  uint16_t vdo0 = (uint16_t)(base + 2);
  uint8_t cmd, objpos, svid_lo, svid_hi;

  cmd     = PR(vdo0 + 0) & 0x1F;
  objpos  = PR(vdo0 + 1) & 0x07;
  svid_lo = PR(vdo0 + 2);
  svid_hi = PR(vdo0 + 3);
  u4_routerop_flag = cmd;
  u4_routerop_op_len = objpos;
  u4_routerop_port = svid_lo;
  u4_routerop_svid_hi = svid_hi;

  if (pd_role_state != 0) {
    vdm_nak(cmd, svid_lo, svid_hi);
    vdm_tx_strobe_commit();
    return;
  }

  pd_tx_buf_clear();

  switch (cmd) {
    case 0x01:
      uart_puts("[Disc_ID]");
      vdm_build_discover_id();
      break;
    case 0x02:
      uart_puts("[Disc_SVIDs]");
      vdm_build_discover_sids(svid_lo, svid_hi);
      break;
    case 0x03:
      uart_puts("[Disc_Modes]");
      vdm_build_discover_modes(svid_lo, svid_hi);
      break;
    case 0x04:
      uart_puts("[EnterMode]");
      vdm_handle_enter_mode(objpos, svid_lo, svid_hi);
      break;
    default:
      vdm_nak(cmd, svid_lo, svid_hi);
      break;
  }

  vdm_tx_strobe_commit();
}

#endif /* PD_H */
