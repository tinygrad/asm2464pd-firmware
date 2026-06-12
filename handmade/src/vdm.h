#ifndef VDM_H
#define VDM_H
/*
 * USB-PD Structured VDM responder + USB4/Thunderbolt mode entry (stock fw_tinygrad.bin).
 * Included AFTER pd_dispatch.h: PR(a)=XDATA_REG8V(a), uart_* and the PD TX engine helpers
 * (pd_tx_set_sop_header / pd_tx_commit_engine / pd_tx_buf_clear / pd_ctrl_goodcrc) are in scope.
 *
 * Pure VDM responder. After the power contract the host runs:
 *   Discover_Identity -> ACK (USB4-capable, VID 0x174C)
 *   Discover_SVIDs    -> ACK (SVID 0x8087 Intel Thunderbolt)
 *   Discover_Modes    -> ACK (TBT3 mode 0x00000001)
 *   EnterMode / Enter_USB -> accept; USB4 mode entry latches.
 *
 * Stock anchors:
 *   vdm_tx_dispatch               @0x9AC4
 *   vdm_build_discover_id         @0xAA36
 *   vdm_build_discover_sids       @0xDDAD
 *   vdm_build_discover_modes      @0xD852
 *   vdm_handle_enter_mode         @0xB966
 *   pd_handle_enter_usb           @0xA036  (connect decide tail @0xA0A7)
 *   usb4_mode_entry_commit        @0xD78A  (92E1=0x10 latch)
 *   pd_vdm_hdr_build              @0xE120  (VDM header VDO E422:E425)
 *   pd_tx_set_sop_header          @0xDD12  [pd_dispatch.h]
 *   pd_tx_commit_engine           @0xE1C6  [pd_dispatch.h]
 *
 * VDO TX buffer layout: the Structured-VDM header is the first VDO slot E422:E425 (same bytes
 * the Request RDO used). Response VDOs follow at E426, E42A, E42E(/E430), E432, ... (4-byte aligned).
 * PD message length 0x07C4 = 2 (PD header) + 4*NumVDOs.
 *
 * VDM TX uses the CC88/CC89/CC8A/CC8B command lane (stock @0x9BBF), NOT the CC10 lane the Request
 * RDO drives: CC88=(CC88&0xF8)|3 opcode-3, CC8A=0, CC8B=0x50 param, CC89=1 go / poll .1 / =2 ack,
 * then pd_tx_commit_engine() transmits the staged E420.. buffer.
 */

/* usb4_connect_u4 @0xA3F5 — post-Enter_USB connect handler (defined in usb4.h, included after). */
static void usb4_connect_u4(void);

/* VDM identity constants */
#define VDM_VID_LO        0x4C   /* 0x174C low byte  */
#define VDM_VID_HI        0x17   /* 0x174C high byte */
#define VDM_TBT_SVID_LO   0x87   /* 0x8087 low byte (Intel Thunderbolt) */
#define VDM_TBT_SVID_HI   0x80   /* 0x8087 high byte */

/* pd_vdm_hdr_build @0xE120 — build the VDM-header VDO in E422:E425.
 * cmdtype in E422 bits 7:6, cmd in bits 4:0 (mask 0xCF). E423 = structured-VDM byte
 * (0x80 SOP, 0xA8 SOP'). E424/E425 = SVID placeholder, overwritten by caller.
 * cmdtype: 0=REQ,1=ACK,2=NAK,3=BUSY.  cmd: 1=Disc_ID,2=Disc_SVIDs,3=Disc_Modes,4=EnterMode. */
static void pd_vdm_hdr_build(uint8_t cmdtype, uint8_t cmd) {
  REG_CMD_PARAM = (uint8_t)((((uint8_t)(cmdtype << 6)) | cmd) & 0xCF);
  REG_CMD_STATUS = (PR(0x07CA) == 1) ? 0x80 : 0xA8;
  REG_CMD_ISSUE = 0x00;
  REG_CMD_TAG = 0xFF;
}

/* VDM NAK (dd0e/95a0/95a2) — NAK echoing the received SVID, length 0x07C4 = 6 (1 VDM-hdr VDO). */
static void vdm_nak(uint8_t cmd, uint8_t svid_lo, uint8_t svid_hi) {
  pd_tx_set_sop_header(1, 0x0F);   /* dd0e: 1 VDO, MsgType 0x0F (VDM) */
  pd_vdm_hdr_build(2, cmd);        /* 95a2: cmdtype 2 = NAK */
  REG_CMD_ISSUE = svid_lo;            /* echo received SVID lo */
  REG_CMD_TAG = svid_hi;            /* echo received SVID hi */
  PR(0x07C4) = 6;                  /* 2 PD header + 4 VDM-hdr VDO */
}

/* Discover_Identity responder @0xAA36 — ID ACK VDO chain.
 * 5 VDOs / 0x07C4=0x16 for USB4 (SOP==2), 4 VDOs / 0x12 for USB3. */
static void vdm_build_discover_id(void) {
  uint8_t sop = PR(0x07CA);
  uint8_t f9, g23;

  /* PD header: 5 VDOs for USB4 (SOP==2), 4 for USB3. MsgType 0x0F. */
  pd_tx_set_sop_header((sop == 2) ? 5 : 4, 0x0F);
  pd_vdm_hdr_build(1, 1);          /* ACK, Discover_Identity */

  /* ID Header VDO (E426-E429): VID 0x174C, modal/product-type bits, USB4-cap flag. */
  REG_CMD_LBA_0 = VDM_VID_LO;         /* 0x4C */
  REG_CMD_LBA_1 = VDM_VID_HI;         /* 0x17  -> VID 0x174C */
  REG_CMD_LBA_2 = (sop == 2) ? 0x40 : 0x00;
  if (PR(0x07BC) == 0 && (PR(0x09F9) & 0x80))
    REG_CMD_LBA_3 = 0x54;             /* USB4-capable */
  else
    REG_CMD_LBA_3 = 0x50;

  /* Cert Stat VDO (E42A-E42D) + E42E cleared (stock 95F9 clears E42A..E42E). */
  REG_CMD_COUNT_LOW = 0; REG_CMD_COUNT_HIGH = 0; REG_CMD_LENGTH_LOW = 0; REG_CMD_LENGTH_HIGH = 0; REG_CMD_RESP_TAG = 0;

  /* Product VDO (E430:E431 = PID/bcdDevice from PD RAM 0x0A57/0x0A58). */
  REG_CMD_CTRL = PR(0x0A57);
  REG_CMD_TIMEOUT = PR(0x0A58);

  /* Product-Type VDO(s) from 0x09F9 mode/gen bits — USB4 only. */
  if (sop == 2) {
    f9 = PR(0x09F9);
    g23 = f9 & 0x03;
    PR(0x0AA8) = (g23 != 0) ? 3 : 2;
    if (f9 & 0x80) PR(0x0AA8) |= 0x08;
    if (PR(0x07BC) == 0) REG_CMD_PARAM_L = PR(0x0AA8);
    else                 REG_CMD_PARAM_L = 2;
    REG_CMD_PARAM_H = 0;
    REG_CMD_EXT_PARAM_0 = 0x80;
    if (PR(0x07BC) == 0 && g23 != 0) REG_CMD_EXT_PARAM_1 = 0x6D;
    else                             REG_CMD_EXT_PARAM_1 = 0x65;
  }

  PR(0x07C4) = (sop == 2) ? 0x16 : 0x12;
}

/* Discover_SVIDs responder @0xDDAD — ACK only when received SVID == PD SID 0xFF00 and 0x09F9.7.
 * Advertises SVID 0x8087 (Intel TBT) in the low half, 0x0000 terminator high half; 0x07C4=0x0A. */
static void vdm_build_discover_sids(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  /* ddb0-ddb8: ACK iff (~hi | lo)==0 (hi==0xFF AND lo==0x00, PD SID 0xFF00) AND 0x09F9.7 (USB4-cap). */
  if (((uint8_t)~rx_svid_hi | rx_svid_lo) == 0 && (PR(0x09F9) & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);   /* 9730: 2 VDOs, MsgType 0x0F */
    pd_vdm_hdr_build(1, 2);          /* ACK, Discover_SVIDs */
    REG_CMD_LBA_0 = 0x00;               /* SVID-list VDO: hi half terminator */
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = VDM_TBT_SVID_LO;    /* 0x87 */
    REG_CMD_LBA_3 = VDM_TBT_SVID_HI;    /* 0x80  -> SVID 0x8087 */
    PR(0x07C4) = 0x0A;
    return;
  }
  vdm_nak(2, rx_svid_lo, rx_svid_hi);
}

/* Discover_Modes responder @0xD852 — ACK when received SVID == 0x8087 and 0x09F9.7.
 * Echoes SVID in E424:E425; Mode VDO E426=1 (TBT3 mode 0x00000001); 0x07C4=0x0A. */
static void vdm_build_discover_modes(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  if (rx_svid_hi == 0x80 && rx_svid_lo == 0x87 && (PR(0x09F9) & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);   /* 9730 */
    pd_vdm_hdr_build(1, 3);          /* ACK, Discover_Modes */
    REG_CMD_ISSUE = VDM_TBT_SVID_LO;    /* echo SVID 0x8087 */
    REG_CMD_TAG = VDM_TBT_SVID_HI;
    REG_CMD_LBA_0 = 0x01;               /* Mode VDO = TBT3 mode 0x00000001 */
    REG_CMD_LBA_1 = 0x00;
    REG_CMD_LBA_2 = 0x00;
    REG_CMD_LBA_3 = 0x00;
    PR(0x07C4) = 0x0A;
    return;
  }
  vdm_nak(3, rx_svid_lo, rx_svid_hi);
}

/* usb4_mode_entry_commit @0xD78A — device-side USB4 mode-entry latch.
 * 0x09F9.6 set: 0x0ACD=3, 0x0ACE=1, 92E1=0x10 (mode-entry trigger), clear 9090.7 (USB int mask).
 * Else: 0x0ACD=1; 0x0ACE = 0x0D if (0x09F9 & 0x81)==0 else 5. */
static uint8_t usb4_mode_entry_commit(void) {
  uint8_t f9 = PR(0x09F9);
  if (f9 & 0x40) {
    PR(0x0ACD) = 3;
    PR(0x0ACE) = 1;
    PR(0x92E1) = 0x10;                 /* USB4 mode-entry latch */
    REG_USB_INT_MASK_9090 &= 0x7F;    /* clear USB global int mask bit7 */
    return 4;                          /* d7b1: R7=4 (USB4 mode) */
  }
  PR(0x0ACD) = 1;
  PR(0x0ACE) = ((f9 & 0x81) == 0) ? 0x0D : 0x05;
  return 1;                            /* d7ca: R7=1 */
}

/* EnterMode responder @0xB966 — SVID 0x8087 + obj-pos 1 + 0x09F9.7 + 0x07BC==0 enters TBT alt-mode
 * (0x07B9=1, 0x07BB=1, [Enter_TBT]); otherwise a generic EnterMode ACK with obj-pos in E423. */
static void vdm_handle_enter_mode(uint8_t objpos, uint8_t svid_lo, uint8_t svid_hi) {
  PR(0x0AA8) = svid_lo;
  PR(0x0AA9) = svid_hi;
  PR(0x0AAA) = objpos;

  if (svid_lo == 0x87 && svid_hi == 0x80 && (PR(0x09F9) & 0x80) && PR(0x07BC) == 0) {
    /* TBT alt-mode entry. Stock: e93a (clear CC91 event) then pd_vdm_hdr_build(1,4) ACK. */
    pd_tx_set_sop_header(1, 0x0F);
    pd_vdm_hdr_build(1, 4);            /* ACK, EnterMode */
    PR(0x07C4) = 6;                    /* 1 VDM-hdr VDO */
    PR(0x07B9) = 1;                    /* EnterMode TBT */
    PR(0x07BB) = 1;                    /* connect-pending (Connect_U4 trigger) */
    uart_puts("[Enter_TBT]");
    return;
  }
  /* Non-TBT or wrong state: plain EnterMode ACK echoing SVID, obj-pos in E423. */
  pd_tx_set_sop_header(1, 0x0F);
  pd_vdm_hdr_build(2, 4);
  REG_CMD_STATUS |= (PR(0x0AAA) & 0x07);
  REG_CMD_ISSUE = svid_lo;
  REG_CMD_TAG = svid_hi;
  PR(0x07C4) = 6;
}

/* pd_handle_enter_usb @0xA036 -> usb4_connect_decide @0xA0A7 — Enter_USB Data Message.
 * Parses the EUDO USB-Mode field (0=USB2,1=USB3.2,2=USB4) and cable-current bit, then either
 * latches USB4 mode entry (default), Accepts USB4 + flags connect-pending, or rejects. */
static void pd_handle_enter_usb(void) {
  uint16_t base = pd_rx_ptr();
  uint16_t vdo0 = (uint16_t)(base + 2);
  /* EUDO (little-endian in RX buffer): VDO0[1].5 = cable-current (0xA03F-0xA046);
   * VDO0[3] bits 6:4 = USB-Mode -> 0x0AA6 (0xA0A2-0xA0A6: 0=USB2,1=USB3.2,2=USB4).
   * EUDO 0x2487E000 -> VDO0[3]=0x24 -> mode=2=USB4. */
  uint8_t cable_cur = (uint8_t)((PR(vdo0 + 1) & 0x20) >> 5);
  uint8_t mode = (uint8_t)((PR(vdo0 + 3) & 0x70) >> 4);
  uint8_t f9;

  PR(0x0AA6) = mode;
  pd_tx_buf_clear();                              /* e73a */

  f9 = PR(0x09F9);
  if ((f9 & 0x03) == 0) {
    /* USB4-capable default state: latch mode entry, control reply length 2. */
    REG_CMD_CFG_E405 &= 0xF8;                           /* 9664 */
    PR(0x09FA) = 4;
    usb4_mode_entry_commit();                     /* d78a: 92E1=0x10 if 0x09F9.6 */
    PR(0x0AE2) = mode;
  } else if (mode == 2 && PR(0x07BC) == 0) {
    /* USB4 requested + not in alt-mode: Accept and flag connect-pending.
     * Stock @0xA108-0xA11F: Accept header (dd12), then iff cable_cur set 0x07BB, print
     * [Enter_USB 4], set 0x07BA (Connect_U4 gate consumed by the tail below). */
    pd_tx_set_sop_header(0, 3);                   /* dd12: 0 VDO, MsgType 3 = Accept */
    if (cable_cur) {
      PR(0x07BB) = 1;                             /* connect-pending */
      uart_puts("[Enter_USB 4]");
      PR(0x07BA) = 1;                             /* a11f: Connect_U4 gate */
      /* Arm the c9a8 connect-dispatch gate that stock sets via the C80A.4 a522 link-width path
       * (0x09FA|=4); the Intel host never reaches it with handmade, so mirror it here.
       * 0x07E8/0x0AF1.0 are also set on the USB4 route entry (pd.h) + connect (usb4.h). */
      PR(0x09FA) |= 0x04;             /* 0x09FA.2 -> c9a8 gate bit (a522 mirror) */
      PR(0x07E8) = 1;                             /* c9a8 gate term */
    }
  } else {
    /* Reject (mode not USB4 / wrong state). */
    pd_tx_set_sop_header(0, 4);                   /* dd12: MsgType 4 = Reject */
  }

  PR(0x07C4) = 2;                                 /* 95e4: control reply length */
  pd_tx_commit_engine();                          /* e1c6 */

  /* Stock tail @0xA167: after Accept, if the Connect_U4 gate (0x07BA) is set, print [Connect_U4]
   * and (when 0x07ED==0) run usb4_connect_u4 @0xA3F5 (SB b230 flip + sb_block_init + tunnel route).
   * This is the PD->USB4 handoff. */
  if (PR(0x07BA) != 0) {
    uart_puts("[Connect_U4]");
    if (PR(0x07ED) != 0) {
      PR(0x07ED) = 0;                             /* a17c: one-shot suppress, then return */
    } else {
      usb4_connect_u4();                          /* a17f -> 0xA3F5 */
    }
  }
}

/* VDM TX CC strobe + commit (stock @0x9BBF tail) — drive the CC88/CC89 command interface
 * (opcode 3 = PD-message TX), then commit the staged buffer. Same strobe the keystone uses. */
static void vdm_tx_strobe_commit(void) {
  REG_XFER_DMA_CTRL = (REG_XFER_DMA_CTRL & 0xF8) | 0x03;     /* 9627: opcode 3 */
  REG_XFER_DMA_ADDR_LO = 0;                               /* 9627 */
  REG_XFER_DMA_ADDR_HI = 0x50;                            /* 955e: timeout/param */
  REG_XFER_DMA_CMD = 0x01;                            /* 955e: go */
  { uint16_t g = 0; while (!((REG_XFER_DMA_CMD >> 1) & 1) && ++g < PD_WAIT_LIMIT);
    if (g >= PD_WAIT_LIMIT) pd_cc_timeout = 1; }
  REG_XFER_DMA_CMD = 0x02;                            /* 964f: ack */
  if (REG_PHY_EVENT_E40F & 0x01) return;                /* RX pending -> abort (stock 0x9be2) */
  pd_tx_commit_engine();                        /* e1c6: transmit */
}

/* vdm_tx_dispatch @0x9AC4 — Structured VDM command dispatch (responder).
 * Parses the received VDM header (cmd/objpos/svid) from RX VDO0, and while 0x07BC==0 (not yet in
 * alt-mode) dispatches on cmd; otherwise NAKs. Each path strobes + commits the staged reply. */
static void vdm_tx_dispatch(void) {
  uint16_t base = pd_rx_ptr();            /* 9566/96d4: RX base = 0xE440 + 0x20*0x07C1 */
  uint16_t vdo0 = (uint16_t)(base + 2);   /* first data object (VDM header) */
  uint8_t cmd, objpos, svid_lo, svid_hi;

  cmd     = PR(vdo0 + 0) & 0x1F;          /* 0xAA5: VDM command (1..6) */
  objpos  = PR(vdo0 + 1) & 0x07;          /* 0xAA3: object position */
  svid_lo = PR(vdo0 + 2);                 /* 0xAA6: received SVID lo */
  svid_hi = PR(vdo0 + 3);                 /* 0xAA7: received SVID hi */
  PR(0x0AA5) = cmd;
  PR(0x0AA3) = objpos;
  PR(0x0AA6) = svid_lo;
  PR(0x0AA7) = svid_hi;

  /* Stock gate: only respond to discovery while 0x07BC==0 (not yet in alt-mode); else NAK. */
  if (PR(0x07BC) != 0) {
    vdm_nak(cmd, svid_lo, svid_hi);
    vdm_tx_strobe_commit();
    return;
  }

  pd_tx_buf_clear();                      /* e73a: clear TX VDO buffer before builder writes */

  switch (cmd) {
    case 0x01:                              /* Discover_Identity */
      uart_puts("[Disc_ID]");
      vdm_build_discover_id();
      break;
    case 0x02:                              /* Discover_SVIDs */
      uart_puts("[Disc_SVIDs]");
      vdm_build_discover_sids(svid_lo, svid_hi);
      break;
    case 0x03:                              /* Discover_Modes */
      uart_puts("[Disc_Modes]");
      vdm_build_discover_modes(svid_lo, svid_hi);
      break;
    case 0x04:                              /* EnterMode */
      uart_puts("[EnterMode]");
      vdm_handle_enter_mode(objpos, svid_lo, svid_hi);
      break;
    default:                                /* ExitMode/Attention/unknown -> NAK */
      vdm_nak(cmd, svid_lo, svid_hi);
      break;
  }

  vdm_tx_strobe_commit();
}

#endif /* VDM_H */
