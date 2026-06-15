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
  uint8_t slot = PR(0x07C1);
  uint16_t base = (uint16_t)(0xE440u + (uint16_t)(0x20u * slot));
  PR(0x07BF) = (uint8_t)(base >> 8);
  PR(0x07C0) = (uint8_t)(base & 0xFF);
  return base;
}
/* Re-read the stowed RX buffer pointer. */
static uint16_t pd_rx_ptr_get(void) {
  return ((uint16_t)PR(0x07BF) << 8) | PR(0x07C0);
}

/* Send the message staged in E420-E43F and bump the TX MessageID. */
static void pd_tx_commit_engine(void) {
  { uint16_t guard; for (guard = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && guard < 0x4000; guard++); }
  REG_CMD_CTRL_E403 = PR(0x07C4);
  REG_CMD_BUSY_STATUS = (REG_CMD_BUSY_STATUS & 0xFE) | 0x01;
  { uint16_t guard; for (guard = 0; (REG_CMD_BUSY_STATUS & 0x01) && guard < 0x4000; guard++); }
  PR(0x07C3) = (PR(0x07C3) + 1) & 7;
  PR(0x07B7) = 0;
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
  uint8_t sop = PR(0x07CA);
  if (sop == 2 || sop == 3) REG_CMD_TRIGGER = 0x80; else REG_CMD_TRIGGER = 0x40;
  REG_CMD_CFG_E405 &= 0xF8;
  REG_CMD_MODE_E421 = (uint8_t)(((nobj & 7) << 4) | ((uint8_t)(PR(0x07C3) << 1) & 0x0E));
  REG_CMD_TRIGGER = (uint8_t)((REG_CMD_TRIGGER & 0xC0) | msgtype);
}

/* Zero the PD TX message buffer E420-E43F. */
static void pd_tx_buf_clear(void) {
  uint8_t i;
  for (i = 0; i < 0x20; i++) PR(0xE420 + i) = 0;
}

/* Select PDO[0] (vSafe5V fixed supply) from the received Source_Cap. */
static void pd_select_pdo_from_source_cap(void) {
  uint16_t pdo0 = (uint16_t)(0xE442u + (uint16_t)(0x20u * PR(0x07C1)));
  PR(0x07D4) = PR(pdo0 + 0);
  PR(0x07D3) = PR(pdo0 + 1) & 0x03;
  PR(0x07C7) = 0;
  PR(0x07C5) = 1;
}

/* Build a Request (header + Fixed RDO) and send it, then arm SenderResponse. */
static void pd_build_send_request_rdo(void) {
  uint8_t cur_hi = PR(0x07D3);
  uint8_t cur_lo = PR(0x07D4);

  pd_tx_buf_clear();
  pd_tx_set_sop_header(1, 2);

  REG_CMD_PARAM = cur_lo;
  REG_CMD_STATUS = cur_hi;
  REG_CMD_STATUS |= (uint8_t)(cur_lo << 2);
  REG_CMD_ISSUE = (uint8_t)((cur_hi << 2) & 0x0C);
  REG_CMD_ISSUE |= (uint8_t)(cur_lo >> 6);
  REG_CMD_TAG = 1;
  REG_CMD_TAG |= (uint8_t)(((PR(0x07C7) + 1) & 7) << 4);
  REG_CMD_TAG |= 2;
  REG_CMD_TAG &= 0xFE;

  PR(0x07C4) = 6;
  PR(0x07BD) = 3;

  REG_TIMER0_DIV &= 0xF7;
  pd_cc_event_clear(0xCC11);
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | 0x03;
  REG_TIMER0_THRESHOLD_HI = 0;
  REG_TIMER0_THRESHOLD_LO = 0x28;
  REG_TIMER0_CSR = 0x01;
  { uint16_t guard = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++guard < PD_WAIT_LIMIT);
    if (guard >= PD_WAIT_LIMIT) pd_cc_timeout = 1; }
  REG_TIMER0_CSR = 0x02;

  pd_tx_commit_engine();
  pd_arm_cc_timer(2, 0x30);
}

/* ---------- CONTROL-message handlers (NumObj==0) ---------- */

/* GoodCRC: advance the RX slot index; commit any staged pending TX. */
static void pd_ctrl_goodcrc(void) {
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
  if (PR(0x07B7) != 0) pd_tx_commit_engine();
}

/* Accept: on substate 3 advance to 4 and arm the PS_RDY timer. */
static void pd_ctrl_accept(void) {
  uint8_t substate;
  uart_puts("[Accept]");
  substate = PR(0x07BD);
  if (substate == 3) {
    pd_e933_clear_cc81();
    PR(0x07BD) = 4;
    pd_arm_cc_timer(0x10, 0x27);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0) {
    if (PR(0x07DE) != 0) { PR(0x07DE) = 0; }
    pd_e933_clear_cc81();
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    pd_e933_clear_cc81();
    PR(0x07BD) = 0x0D;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* PS_RDY: on substate 4, decode the contract voltage into 0x07B8. */
static void pd_ctrl_ps_rdy(void) {
  uint8_t volt_hi, volt_lo;
  uart_puts("[PS_RDY]");
  if (PR(0x07BD) != 4) { pd_ctrl_goodcrc(); return; }
  pd_e933_clear_cc81();

  volt_hi = PR(0x07D6);
  volt_lo = PR(0x07D7);
  if (volt_hi == 1 && volt_lo == 0x2C) {
    uart_puts("[5V3A]");
    PR(0x07B8) = 3;
  } else {
    uint16_t volt = ((uint16_t)volt_hi << 8) | volt_lo;
    if (volt >= 0x012C || volt_hi >= 1) {
      uart_puts("[5V1.5A]");
      PR(0x07B8) = 2;
    } else if (volt < 0x0096) {
      PR(0x07B8) = 1;
    } else {
      PR(0x07B8) = 4;
    }
  }
  PR(0x07DE) = 0;
  PR(0x07DF) = 0;
  pd_ctrl_goodcrc();
}

/* Reject: arm the timer on substate 3, or latch Data_Reset on 0x0E. */
static void pd_ctrl_reject(void) {
  uint8_t substate;
  uart_puts("[Reject]");
  substate = PR(0x07BD);
  if (substate == 3) {
    pd_e933_clear_cc81();
    if (PR(0x07B8) == 0) {
    }
    pd_arm_cc_timer(0, 0);
    pd_ctrl_goodcrc();
    return;
  }
  if (substate == 0x0E) {
    PR(0x07BD) = 0x0E;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* Wait: on substate 3 with an active contract, arm the CC timer and ack. */
static void pd_ctrl_wait(void) {
  if (PR(0x07BD) != 3) { pd_ctrl_goodcrc(); return; }
  pd_e933_clear_cc81();
  if (PR(0x07B8) != 0) {
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
  if (PR(0x07CA) == 1) {
    pd_tx_set_sop_header(0, 1);
  } else {
    REG_CMD_CFG_E405 &= 0xF8;
  }
  PR(0x07C4) = 2;
  pd_tx_commit_engine();
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
}

/* Soft_Reset: reset both MessageID counters and reply Accept with MessageID=0. */
static void pd_ctrl_soft_reset(void) {
  uart_puts("[Soft_Reset]");
  PR(0x07C3) = 0;
  PR(0x07C1) = 0;
  PR(0x07B7) = 0;
  pd_e933_clear_cc81();

  pd_tx_buf_clear();
  pd_tx_set_sop_header(0, 3);
  PR(0x07C4) = 2;

  REG_CMD_MODE_E421 &= 0xF1;

  pd_tx_commit_engine();

  PR(0x07BD) = 1;
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
    sop = PR(0x07CA);
    if (sop == 2 || sop == 3) {
      REG_CMD_TRIGGER = 0x80;
      REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x04;
    }
    {
      uint8_t substate = PR(0x07BD);
      if (substate != 1 && substate != 5 && substate != 3 && substate != 6) {
        return;
      }
    }
    pd_e933_clear_cc81();
    pd_select_pdo_from_source_cap();
    PR(0x07BD) = 2;
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

  PR(0x07C2) = (uint8_t)((hdr1 >> 4) & 7);
  PR(0x0AA2) = (uint8_t)((hdr1 >> 1) & 7);
  PR(0x0AA1) = (uint8_t)(hdr0 & 0x1F);
  sop = (uint8_t)(hdr0 >> 6);
  PR(0x07CA) = sop;

  uart_puts("[D");
  uart_puthex(PR(0x07C1)); uart_putc(' ');
  uart_puthex(hdr0); uart_puthex(hdr1); uart_putc(' ');
  uart_puthex(PR(0x07C2)); uart_puthex(PR(0x0AA1)); uart_putc(']');

  ext_bit = (uint8_t)((PR(base + 1) >> 7) & 1);
  if (PR(0x07C8) != 0) return;

  if (ext_bit != 0 && sop == 0) {
    pd_rx_nak_send();
    return;
  }

  if (PR(0x07C2) == 0) {
    pd_dispatch_control(PR(0x0AA1));
  } else {
    pd_dispatch_data(PR(0x0AA1));
  }
}

#endif /* PD_DISPATCH_H */
