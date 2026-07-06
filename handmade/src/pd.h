#ifndef PD_H
#define PD_H
/*
 * USB-PD / Type-C CC-controller driver: PD attach + power-contract bring-up.
 * Touches the PD engine (E4xx), CC controller (CCxx), interrupt-enable group and PD state RAM.
 */
#include "types.h"
#include "registers.h"

#define PR(a) XDATA_REG8V(a)

static void pd_rx_message_dispatch(void);
static void pd_tx_buf_clear(void);

#define PD_WAIT_LIMIT 0x4000u

/* USB-PD Control-Message types (header NumDataObjects == 0). */
#define PD_CTRL_GOODCRC      0x01
#define PD_CTRL_ACCEPT       0x03
#define PD_CTRL_REJECT       0x04
#define PD_CTRL_PS_RDY       0x06
#define PD_CTRL_WAIT         0x0C
#define PD_CTRL_SOFT_RESET   0x0D

/* USB-PD Data-Message types (header NumDataObjects > 0). */
#define PD_DATA_SOURCE_CAP   0x01
#define PD_DATA_BIST         0x03
#define PD_DATA_ENTER_USB    0x08
#define PD_DATA_VENDOR       0x0F   /* Vendor_Defined (Structured VDM) */

/* Structured VDM command codes and header Command Type field. */
#define VDM_CMD_DISCOVER_ID     0x01
#define VDM_CMD_DISCOVER_SVIDS  0x02
#define VDM_CMD_DISCOVER_MODES  0x03
#define VDM_CMD_ENTER_MODE      0x04
#define VDM_CMDT_ACK  1
#define VDM_CMDT_NAK  2

/* PD message mailbox (E4xx): one 0x20-byte slot per message.
 * TX staging buffer at 0xE420; RX ring of slots from 0xE440. */
#define PD_TX_BASE    0xE420u
#define PD_RX_BASE    0xE440u
#define PD_MSG_STRIDE 0x20u   /* bytes per message slot (also the TX buffer size) */
/* Bounded poll until (*reg & mask) matches set (1=wait-for-set, 0=wait-for-clear). */
static void pd_wait(volatile __xdata uint8_t *reg, uint8_t mask, uint8_t set) {
  uint16_t iters = 0;
  if (set) { while (!(*reg & mask) && ++iters < PD_WAIT_LIMIT); }
  else     { while ( (*reg & mask) && ++iters < PD_WAIT_LIMIT); }
}

/* Bounded wait for the PD command engine to go idle (no pending op, not busy). */
static void pd_wait_engine_idle(void) {
  uint16_t guard;
  for (guard = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && guard < 0x4000; guard++);
}

/* RDO/CRC timing constants for the PD engine. */
static void pd_cmd_engine_cfg(void) {  /* da51 */
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
static void pd_xfer_dma_run(uint8_t addr_hi) {
  REG_XFER_DMA_CTRL &= 0xF8; REG_XFER_DMA_ADDR_LO = 0;
  REG_XFER_DMA_ADDR_HI = addr_hi; REG_XFER_DMA_CMD = 0x01;
  pd_wait(&REG_XFER_DMA_CMD, 0x02, 1);
  REG_XFER_DMA_CMD = 0x02;
}
static void cc_pd_phy_term_init(void) {
  REG_CMD_CONFIG = (REG_CMD_CONFIG & 0xBF) | 0x40;
  REG_CMD_CFG_E40A = 0x0F;
  REG_CMD_CFG_E413 &= 0xFE;
  REG_CMD_CFG_E413 &= 0xFD;
  REG_CMD_CTRL_E400 &= 0x7F;
  pd_xfer_dma_run(0x0A);
  REG_CMD_CONFIG = (REG_CMD_CONFIG & 0xFE) | 0x01;
  pd_xfer_dma_run(0x3C);
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
  pd_cmd_engine_cfg();
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
  u4_pd.contract_state = 0;
  u4_pd.tx_msgid_counter = 0; u4_pd.tx_msg_len = 0;
  u4_pd.rx_num_data_obj = 0;
  u4_pd.rx_slot_idx = 0;
  u4_pd.msg_substate = 1;
  u4_pd.rx_slot_mask = (REG_CMD_CTRL_E400 & 0x40) ? 0x10 : 0x01;
  if (u4_pd.softreset_pending == 0) u4_pd.sop_field = 2;
  u4_pd.softreset_pending = 0;
  u4_pd.connect_route_latch = 0; u4_pd.enter_usb_accepted = 0;
  u4_pd.eudo_mode_confirm = 0;
  u4_pd.confirm_input_cd = 0; u4_pd.confirm_input_ce = 0; u4_pd.confirm_input_cf = 0;
  u4_pd.bist_mode = 0;
  u4_pd.usb3_fallback_flag = 0;
  u4_pd.role_state = 0;
  cc_ctrl_enable_events();
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
  pd_tx_buf_clear();
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
  pd_wait_engine_idle();
  REG_CMD_BUSY_STATUS |= 0x01;
  for (guard = 0; (REG_CMD_BUSY_STATUS & 0x01) && guard < 0x4000; guard++);
}

/* Route the C806/C80A PD/system interrupt aggregate to the 8051 EX1 line so INT1 fires. */
static void pd_int1_enable_group(void) {
  REG_INT_ENABLE = (REG_INT_ENABLE & 0xEF) | 0x10;
  REG_INT_STATUS_C800 = (REG_INT_STATUS_C800 & 0xFB) | 0x04;
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF8) | 0x06;
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;
  REG_INT_STATUS_C800 |= 0x01;
  /* Timer/Link Power Control strobe (stock fw pd_int1_enable_group). */
  REG_TIMER_CTRL_CC3B |= 0x01;
  REG_TIMER_CTRL_CC3B = (uint8_t)((REG_TIMER_CTRL_CC3B & 0xFD) | 0x02);
}

/* Top-level keystone bring-up: enable INT1, set the USB4 mode policy, init PD PHY + state. */
static void pd_keystone_init(void) {
  pd_int1_enable_group();
  u4_cfg.mode_flag = USB4_MODE_FLAGS;
  cc_pd_phy_term_init();
  pd_internal_state_init();
  /* Arm the 1s DMA timer once at boot (stock fw func_e352). */
  REG_CPU_DMA_CTRL_CC90 = (uint8_t)(REG_CPU_DMA_CTRL_CC90 | 0x05);
  REG_CPU_DMA_DATA_LO = 0x00;
  REG_CPU_DMA_DATA_HI = 0xC8;
  REG_CPU_DMA_INT = 0x01;
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
    u4_boot.pd_seen = 1;
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
static void cc_sb_reconnect_event(void) {  /* cc23 */
  u4_pd.enter_usb_reinit_gate = 0x00;
  u4_p12.reinit_pending = 0x01;
}

/* Type-C error-recovery (diagnostic print only). */
/* CC81 |= 4 then drive a hard reset. */
static void pd_attach_hard_reset(void) {  /* cc81 */
  REG_CPU_INT_CTRL = 0x04;
  pd_drive_hard_reset();
}

/* Enqueue a PD control message. */
static void pd_queue_ctrl_msg(uint8_t code) {
  u4_routerop_op_len = code;
  REG_PHY_LINK_CTRL = 0x00;
}

/* CC99 default branch. */
static void cc_role_reset_default(void) {  /* cc99 */
}

/* CCF9.1 sub-demux on 0x0A9D (copied from 0x0B1B). */
/* INT1 timer-tick PD/USB4 policy-engine tick: services the 6 CC per-channel event regs (bit1=event). */
static void cc_pd_timer_tick(void) {
  if (REG_TIMER3_CSR & 0x02) {                 /* CC23.1: re-init / SB-reconnect */
    cc_sb_reconnect_event();
    REG_TIMER3_CSR = 0x02;
  }
  if (REG_CPU_INT_CTRL & 0x02) {                 /* CC81.1: CC attach/detach */
    uint8_t substate = u4_pd.msg_substate;
    if (substate == 0x0E || substate == 0x0D) {      /* Data_Reset / Enter_USB pending */
      REG_CPU_INT_CTRL = 0x02;
      if (u4_pd.role_state != 0) pd_queue_ctrl_msg(0x3B);
      uart_puts("[Error_Recovery]\n");
    } else {
      pd_attach_hard_reset();
      REG_CPU_INT_CTRL = 0x02;
    }
  }
  if (REG_CPU_DMA_INT & 0x02) {                 /* CC91.1: 1s sender-response timeout -> commit USB4 mode */
    REG_CPU_DMA_INT = 0x02;
    uart_puts("[1 sec time out]\n");
    u4_cfg.route_mode = 0x04;
    u4_entered_usb_mode = usb4_mode_entry_commit();
  }
  if (REG_XFER_DMA_CFG & 0x02) {                 /* CC99.1: role-dependent reset */
    uint8_t role = u4_pd.role_state;
    if (role == 0x02) { pd_queue_ctrl_msg(0x3C); pd_drive_hard_reset(); }
    else if (role == 0x03) { pd_queue_ctrl_msg(0xFF); }
    else { cc_role_reset_default(); REG_XFER_DMA_CFG = 0x02; }
  }
  if (REG_XFER2_DMA_STATUS & 0x02) {                 /* CCD9.1 */
    REG_XFER2_DMA_STATUS = 0x02;
    u4_sb.routerop_push_token = 0x02;
  }
  if (REG_CPU_EXT_STATUS & 0x02) {                 /* CCF9.1 */
    REG_CPU_EXT_STATUS = 0x02;
    u4_cfg.routerop_desc0 = u4_p12.cc_subdemux_src;
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

/* Return the 16-bit RX buffer pointer for the current RX slot. */
static uint16_t pd_rx_ptr(void) {
  uint8_t slot = u4_pd.rx_slot_idx;
  return (uint16_t)(PD_RX_BASE + (uint16_t)(PD_MSG_STRIDE * slot));
}

/* Send the message staged in E420-E43F and bump the TX MessageID. */
static void pd_tx_commit_engine(void) {
  pd_wait_engine_idle();
  REG_CMD_CTRL_E403 = u4_pd.tx_msg_len;
  REG_CMD_BUSY_STATUS = (REG_CMD_BUSY_STATUS & 0xFE) | 0x01;
  { uint16_t guard; for (guard = 0; (REG_CMD_BUSY_STATUS & 0x01) && guard < 0x4000; guard++); }
  u4_pd.tx_msgid_counter = (u4_pd.tx_msgid_counter + 1) & 7;
}

/* W1C clear a CC event register (write 4 then 2). */
static void pd_cc_event_clear(uint16_t reg) {
  PR(reg) = 0x04;
  PR(reg) = 0x02;
}
/* Clear the CC attach event (CC81) before a state transition. */
static void pd_clear_cc81_attach(void) { pd_cc_event_clear(0xCC81); }  /* e933 */

/* Arm the CC sender-response / PS-transition timer. */
static void pd_arm_cc_timer(uint16_t threshold) {
  REG_CPU_CTRL_CC82 = (uint8_t)(threshold >> 8);
  REG_CPU_CTRL_CC83 = (uint8_t)threshold;
  pd_cc_event_clear(0xCC81);
  REG_CPU_INT_CTRL = 0x01;
}

/* Build the PD header in E420/E421: SOP type, NumDataObjects, MessageType. */
static void pd_tx_set_sop_header(uint8_t nobj, uint8_t msgtype) {
  uint8_t sop = u4_pd.sop_field;
  if (sop == 2 || sop == 3) REG_CMD_TRIGGER = 0x80; else REG_CMD_TRIGGER = 0x40;
  REG_CMD_CFG_E405 &= 0xF8;
  REG_CMD_MODE_E421 = (uint8_t)(((nobj & 7) << 4) | ((uint8_t)(u4_pd.tx_msgid_counter << 1) & 0x0E));
  REG_CMD_TRIGGER = (uint8_t)((REG_CMD_TRIGGER & 0xC0) | msgtype);
}

/* Zero the PD TX message buffer E420-E43F. */
static void pd_tx_buf_clear(void) {
  mem_set((void __xdata *)PD_TX_BASE, 0, PD_MSG_STRIDE);
}

/* Build a Request (header + Fixed RDO) and send it, then arm SenderResponse. */
static void pd_build_send_request_rdo(void) {
  uint16_t pdo0 = (uint16_t)(PD_RX_BASE + 2u + (uint16_t)(PD_MSG_STRIDE * u4_pd.rx_slot_idx));
  uint8_t cur_lo = PR(pdo0 + 0);
  uint8_t cur_hi = PR(pdo0 + 1) & 0x03;

  pd_tx_buf_clear();
  pd_tx_set_sop_header(1, 2);

  REG_CMD_PARAM = cur_lo;
  REG_CMD_STATUS = cur_hi;
  REG_CMD_STATUS |= (uint8_t)(cur_lo << 2);
  REG_CMD_ISSUE = (uint8_t)((cur_hi << 2) & 0x0C);
  REG_CMD_ISSUE |= (uint8_t)(cur_lo >> 6);
  REG_CMD_TAG = 1;
  REG_CMD_TAG |= 0x10;
  REG_CMD_TAG |= 2;
  REG_CMD_TAG &= 0xFE;

  u4_pd.tx_msg_len = 6;
  u4_pd.msg_substate = 3;

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
  pd_arm_cc_timer(0x0230);
}

/* ---------- CONTROL-message handlers (NumObj==0) ---------- */

/* GoodCRC: advance the RX slot index. */
static void pd_ctrl_goodcrc(void) {
  u4_pd.rx_slot_idx = (uint8_t)((u4_pd.rx_slot_idx + 1) & (uint8_t)(u4_pd.rx_slot_mask - 1));
}

/* Accept: on substate 3 advance to 4 and arm the PS_RDY timer. */
static void pd_ctrl_accept(void) {
  uint8_t substate;
  uart_puts("[Accept]");
  substate = u4_pd.msg_substate;
  if (substate == 3) {
    pd_clear_cc81_attach();
    u4_pd.msg_substate = 4;
    pd_arm_cc_timer(0x1027);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0) {
    if (u4_pd.softreset_pending != 0) { u4_pd.softreset_pending = 0; }
    pd_clear_cc81_attach();
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    pd_clear_cc81_attach();
    u4_pd.msg_substate = 0x0D;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* PS_RDY: on substate 4, decode the contract voltage into 0x07B8. */
static void pd_ctrl_ps_rdy(void) {
  uint8_t volt_hi, volt_lo;
  uart_puts("[PS_RDY]");
  if (u4_pd.msg_substate != 4) { pd_ctrl_goodcrc(); return; }
  pd_clear_cc81_attach();

  volt_hi = u4_pd.decoded_voltage_hi;
  volt_lo = u4_pd.decoded_voltage_lo;
  if (volt_hi == 1 && volt_lo == 0x2C) {
    uart_puts("[5V3A]");
    u4_pd.contract_state = 3;
  } else {
    uint16_t volt = ((uint16_t)volt_hi << 8) | volt_lo;
    if (volt >= 0x012C || volt_hi >= 1) {
      uart_puts("[5V1.5A]");
      u4_pd.contract_state = 2;
    } else if (volt < 0x0096) {
      u4_pd.contract_state = 1;
    } else {
      u4_pd.contract_state = 4;
    }
  }
  u4_pd.softreset_pending = 0;
  pd_ctrl_goodcrc();
}

/* Reject: arm the timer on substate 3, or latch Data_Reset on 0x0E. */
static void pd_ctrl_reject(void) {
  uint8_t substate;
  uart_puts("[Reject]");
  substate = u4_pd.msg_substate;
  if (substate == 3) {
    pd_clear_cc81_attach();
    pd_arm_cc_timer(0);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    u4_pd.msg_substate = 0x0E;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* Wait: on substate 3 with an active contract, arm the CC timer and ack. */
static void pd_ctrl_wait(void) {
  if (u4_pd.msg_substate != 3) { pd_ctrl_goodcrc(); return; }
  pd_clear_cc81_attach();
  if (u4_pd.contract_state != 0) {
    pd_arm_cc_timer(0xD007);
    { uint16_t guard = 0; while (!(REG_CPU_INT_CTRL & 0x02) && ++guard < PD_WAIT_LIMIT); }
    REG_CPU_INT_CTRL = 0x02;
  }
  pd_ctrl_goodcrc();
}

/* Stage and send a 2-byte control NAK response, then advance the RX slot. */
static void pd_rx_nak_send(void) {
  if (u4_pd.sop_field == 1) {
    pd_tx_set_sop_header(0, 1);
  } else {
    REG_CMD_CFG_E405 &= 0xF8;
  }
  u4_pd.tx_msg_len = 2;
  pd_tx_commit_engine();
  u4_pd.rx_slot_idx = (uint8_t)((u4_pd.rx_slot_idx + 1) & (uint8_t)(u4_pd.rx_slot_mask - 1));
}

/* Soft_Reset: reset both MessageID counters and reply Accept with MessageID=0. */
static void pd_ctrl_soft_reset(void) {
  uart_puts("[Soft_Reset]");
  u4_pd.tx_msgid_counter = 0;
  u4_pd.rx_slot_idx = 0;
  pd_clear_cc81_attach();

  pd_tx_buf_clear();
  pd_tx_set_sop_header(0, 3);
  u4_pd.tx_msg_len = 2;

  REG_CMD_MODE_E421 &= 0xF1;

  pd_tx_commit_engine();

  u4_pd.msg_substate = 1;
}

/* Dispatch a CONTROL message by MessageType. */
static void pd_dispatch_control(uint8_t msgtype) {
  switch (msgtype) {
    case PD_CTRL_GOODCRC:    pd_ctrl_goodcrc();    break;
    case PD_CTRL_ACCEPT:     pd_ctrl_accept();     break;
    case PD_CTRL_REJECT:     pd_ctrl_reject();     break;
    case PD_CTRL_PS_RDY:     pd_ctrl_ps_rdy();     break;
    case PD_CTRL_WAIT:       pd_ctrl_wait();       break;
    case PD_CTRL_SOFT_RESET: pd_ctrl_soft_reset(); break;
    default:                 pd_rx_nak_send();     break;
  }
}

/* ---------- DATA-message handlers (NumObj>0) ---------- */

/* Dispatch a DATA message by MessageType. */
static void pd_dispatch_data(uint8_t msgtype) {
  if (msgtype == PD_DATA_SOURCE_CAP) {
    uint8_t sop;
    uart_puts("[Source_Cap]");
    sop = u4_pd.sop_field;
    if (sop == 2 || sop == 3) {
      REG_CMD_TRIGGER = 0x80;
      REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x04;
    }
    {
      uint8_t substate = u4_pd.msg_substate;
      if (substate != 1 && substate != 5 && substate != 3 && substate != 6) {
        return;
      }
    }
    pd_clear_cc81_attach();
    u4_pd.msg_substate = 2;
    pd_build_send_request_rdo();
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype == PD_DATA_BIST) {
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype >= 2 && msgtype < 8) {   /* Request/Sink_Cap/… data messages we NAK */
    pd_rx_nak_send();
    return;
  }
  if (msgtype == PD_DATA_ENTER_USB) {
    uart_puts("[Enter_USB]");
    pd_handle_enter_usb();
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype == PD_DATA_VENDOR) {
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

  u4_pd.rx_num_data_obj = (uint8_t)((hdr1 >> 4) & 7);
  u4_routerop_op_lo = (uint8_t)((hdr1 >> 1) & 7);
  u4_cfg.msg_type = (uint8_t)(hdr0 & 0x1F);
  sop = (uint8_t)(hdr0 >> 6);
  u4_pd.sop_field = sop;

  ext_bit = (uint8_t)((PR(base + 1) >> 7) & 1);
  if (u4_pd.bist_mode != 0) return;

  if (ext_bit != 0 && sop == 0) {
    pd_rx_nak_send();
    return;
  }

  if (u4_pd.rx_num_data_obj == 0) {
    pd_dispatch_control(u4_cfg.msg_type);
  } else {
    pd_dispatch_data(u4_cfg.msg_type);
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
  REG_CMD_STATUS = (u4_pd.sop_field == 1) ? 0x80 : 0xA8;
  REG_CMD_ISSUE = 0x00;
  REG_CMD_TAG = 0xFF;
}

/* NAK echoing the received SVID. */
static void vdm_nak(void) {
  pd_tx_set_sop_header(1, 0x0F);
  pd_vdm_hdr_build(2, u4_routerop_flag);
  REG_CMD_ISSUE = u4_routerop_port;
  REG_CMD_TAG = u4_routerop_svid_hi;
  u4_pd.tx_msg_len = 6;
}

/* Discover_Identity responder: build the ID ACK VDO chain. */
static void vdm_build_discover_id(void) {
  uint8_t sop = u4_pd.sop_field;
  uint8_t mode_bits, gen_bits;

  pd_tx_set_sop_header((sop == 2) ? 5 : 4, 0x0F);
  pd_vdm_hdr_build(1, 1);

  REG_CMD_LBA_0 = VDM_VID_LO;
  REG_CMD_LBA_1 = VDM_VID_HI;
  REG_CMD_LBA_2 = (sop == 2) ? 0x40 : 0x00;
  if (u4_pd.role_state == 0 && (u4_cfg.mode_flag & MODE_FLAG_VDM_ACK))
    REG_CMD_LBA_3 = 0x54;
  else
    REG_CMD_LBA_3 = 0x50;

  REG_CMD_COUNT_LOW = 0; REG_CMD_COUNT_HIGH = 0; REG_CMD_LENGTH_LOW = 0; REG_CMD_LENGTH_HIGH = 0; REG_CMD_RESP_TAG = 0;

  REG_CMD_CTRL = u4_cfg.product_pid_lo;
  REG_CMD_TIMEOUT = u4_cfg.product_pid_hi;

  if (sop == 2) {
    mode_bits = u4_cfg.mode_flag;
    gen_bits = mode_bits & 0x03;
    sb_tx_cmd = (gen_bits != 0) ? 3 : 2;
    if (mode_bits & 0x80) sb_tx_cmd |= 0x08;
    if (u4_pd.role_state == 0) REG_CMD_PARAM_L = sb_tx_cmd;
    else                 REG_CMD_PARAM_L = 2;
    REG_CMD_PARAM_H = 0;
    REG_CMD_EXT_PARAM_0 = 0x80;
    if (u4_pd.role_state == 0 && gen_bits != 0) REG_CMD_EXT_PARAM_1 = 0x6D;
    else                                  REG_CMD_EXT_PARAM_1 = 0x65;
  }

  u4_pd.tx_msg_len = (sop == 2) ? 0x16 : 0x12;
}

/* Discover_SVIDs responder: ACK with SVID 0x8087, else NAK. */
static void vdm_build_discover_svids(void) {
  if (((uint8_t)~u4_routerop_svid_hi | u4_routerop_port) == 0 && (u4_cfg.mode_flag & MODE_FLAG_VDM_ACK)) {
    pd_tx_set_sop_header(2, 0x0F);
    pd_vdm_hdr_build(1, 2);
    REG_CMD_LBA_0 = 0x00;
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = VDM_TBT_SVID_LO;
    REG_CMD_LBA_3 = VDM_TBT_SVID_HI;
    u4_pd.tx_msg_len = 0x0A;
    return;
  }
  vdm_nak();
}

/* Discover_Modes responder: ACK TBT3 mode for SVID 0x8087, else NAK. */
static void vdm_build_discover_modes(void) {
  if (u4_routerop_svid_hi == 0x80 && u4_routerop_port == 0x87 && (u4_cfg.mode_flag & MODE_FLAG_VDM_ACK)) {
    pd_tx_set_sop_header(2, 0x0F);
    pd_vdm_hdr_build(1, 3);
    REG_CMD_ISSUE = VDM_TBT_SVID_LO;
    REG_CMD_TAG = VDM_TBT_SVID_HI;
    REG_CMD_LBA_0 = 0x01;
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = 0x00;
    REG_CMD_LBA_3 = 0x00;
    u4_pd.tx_msg_len = 0x0A;
    return;
  }
  vdm_nak();
}

/* Device-side USB4 mode-entry latch. */
static uint8_t usb4_mode_entry_commit(void) {
  uint8_t mode_flags = u4_cfg.mode_flag;
  if (mode_flags & MODE_FLAG_FULL_COMMIT) {
    u4_mode_entry_class = 3;
    u4_mode_entry_param = 1;
    REG_POWER_EVENT_92E1 = 0x10;
    REG_USB_INT_MASK_9090 &= 0x7F;
    return 4;
  }
  u4_mode_entry_class = 1;
  u4_mode_entry_param = ((mode_flags & (MODE_FLAG_VDM_ACK | MODE_FLAG_ROUTE_MASK)) == 0) ? 0x0D : 0x05;
  return 1;
}

/* EnterMode responder: enter TBT alt-mode for SVID 0x8087, else generic ACK. */
static void vdm_handle_enter_mode(void) {
  uint16_t vdo0 = (uint16_t)(pd_rx_ptr() + 2);
  uint8_t enter_mode_flags = PR(vdo0 + 6);

  sb_tx_cmd = u4_routerop_port;
  sb_tx_byte0 = u4_routerop_svid_hi;
  sb_tx_byte1 = u4_routerop_op_len;
  u4_pd.confirm_input_cd = (uint8_t)(enter_mode_flags >> 7);
  u4_pd.confirm_input_ce = (uint8_t)((enter_mode_flags & MODE_FLAG_FULL_COMMIT) >> 6);
  u4_pd.confirm_input_cf = (uint8_t)(enter_mode_flags & 0x07);

  if (u4_routerop_port == 0x87 && u4_routerop_svid_hi == 0x80 && (u4_cfg.mode_flag & MODE_FLAG_VDM_ACK) && u4_pd.role_state == 0) {
    pd_tx_set_sop_header(1, 0x0F);
    pd_vdm_hdr_build(1, 4);
    u4_pd.tx_msg_len = 6;
    u4_pd.connect_route_latch = 1;
    uart_puts("[Enter_TBT]");
    return;
  }
  pd_tx_set_sop_header(1, 0x0F);
  pd_vdm_hdr_build(2, 4);
  REG_CMD_STATUS |= (sb_tx_byte1 & 0x07);
  REG_CMD_ISSUE = u4_routerop_port;
  REG_CMD_TAG = u4_routerop_svid_hi;
  u4_pd.tx_msg_len = 6;
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
  u4_pd.usb3_fallback_flag = (uint8_t)(u4_routerop_opcode & 1);
  u4_routerop_op_len = (uint8_t)((eudo2 & 0x06) >> 1);
  u4_pd.eudo_mode_confirm = (uint8_t)(eudo2 >> 5);
  u4_routerop_port = mode;
  pd_tx_buf_clear();

  mode_flags = u4_cfg.mode_flag;
  if ((mode_flags & MODE_FLAG_ROUTE_MASK) == 0) {
    REG_CMD_CFG_E405 &= 0xF8;
    u4_cfg.route_mode = 4;
    usb4_mode_entry_commit();
    u4_entered_usb_mode = mode;
  } else if (mode == 2 && u4_pd.role_state == 0) {
    pd_tx_set_sop_header(0, 3);
    if (cable_cur) {
      uart_puts("[Enter_USB 4]");
      u4_pd.enter_usb_accepted = 1;
      u4_cfg.route_mode |= 0x04;
      u4_pd.enter_usb_reinit_gate = 1;
    }
  } else {
    pd_tx_set_sop_header(0, 4);
  }

  u4_pd.tx_msg_len = 2;
  pd_tx_commit_engine();

  if (u4_pd.enter_usb_accepted != 0) {
    uart_puts("[Connect_U4]");
    if (u4_pd.connect_oneshot_suppress != 0) {
      u4_pd.connect_oneshot_suppress = 0;
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

  if (u4_pd.role_state != 0) {
    vdm_nak();
    vdm_tx_strobe_commit();
    return;
  }

  pd_tx_buf_clear();

  switch (cmd) {
    case VDM_CMD_DISCOVER_ID:
      uart_puts("[Disc_ID]");
      vdm_build_discover_id();
      break;
    case VDM_CMD_DISCOVER_SVIDS:
      uart_puts("[Disc_SVIDs]");
      vdm_build_discover_svids();
      break;
    case VDM_CMD_DISCOVER_MODES:
      uart_puts("[Disc_Modes]");
      u4_pd.usb3_fallback_flag = 1;
      vdm_build_discover_modes();
      break;
    case VDM_CMD_ENTER_MODE:
      uart_puts("[EnterMode]");
      vdm_handle_enter_mode();
      break;
    default:
      vdm_nak();
      break;
  }

  vdm_tx_strobe_commit();
}

#endif /* PD_H */
