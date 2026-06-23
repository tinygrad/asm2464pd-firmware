#ifndef PD_DISPATCH_H
#define PD_DISPATCH_H
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

#endif /* PD_DISPATCH_H */
