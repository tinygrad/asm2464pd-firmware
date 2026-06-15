#ifndef VDM_H
#define VDM_H
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
  sb_tx_cmd = svid_lo;
  sb_tx_byte0 = svid_hi;
  sb_tx_byte1 = objpos;

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
  uint8_t cable_cur = (uint8_t)((PR(vdo0 + 1) & 0x20) >> 5);
  uint8_t mode = (uint8_t)((PR(vdo0 + 3) & 0x70) >> 4);
  uint8_t mode_flags;

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
    if (poll >= PD_WAIT_LIMIT) pd_cc_timeout = 1; }
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

#endif /* VDM_H */
