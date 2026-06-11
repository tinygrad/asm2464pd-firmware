#ifndef VDM_H
#define VDM_H
/*
 * USB-PD Structured VDM responder + USB4/Thunderbolt mode entry — faithful transcription of the
 * ASM2464PD stock firmware (fw_tinygrad.bin). Ghidra body offset == fw_tinygrad offset.
 *
 * Included AFTER pd_dispatch.h, so PR(a)=XDATA_REG8V(a), uart_* and the PD TX engine helpers
 * (pd_tx_set_sop_header / pd_tx_commit_engine / pd_tx_buf_clear / pd_ctrl_goodcrc) are in scope.
 *
 * The device is a PURE VDM RESPONDER. After the PD power contract, the host runs:
 *   Discover_Identity -> ACK (USB4-capable, VID 0x174C)
 *   Discover_SVIDs    -> ACK (SVID 0x8087 Intel Thunderbolt)
 *   Discover_Modes    -> ACK (TBT3 mode 0x00000001)
 *   EnterMode / Enter_USB -> device accepts; USB4 mode entry latches.
 *
 * STOCK ANCHORS (verified via disassemble_bytes/decompile_function):
 *   vdm_tx_dispatch                @0x9AC4  — VDM command dispatch (responder)
 *   vdm_build_discover_id_resp     @0xAA36  — Discover_Identity ACK
 *   vdm_build_discover_sids_resp   @0xDDAD  — Discover_SVIDs ACK
 *   vdm_build_discover_modes_resp  @0xD852  — Discover_Modes ACK
 *   vdm_handle_enter_mode          @0xB966  — EnterMode (SVID 0x8087, pos 1)
 *   pd_handle_enter_usb            @0xA036  — Enter_USB Data Message
 *   usb4_connect_decide            @0xA0A7  — Enter_USB connect decision (shared tail)
 *   usb4_mode_entry_commit         @0xD78A  — device-side mode-entry latch (92E1=0x10)
 *   pd_vdm_hdr_build               @0xE120  — VDM header VDO builder (E422:E425)
 *   pd_tx_set_sop_header           @0xDD12  — PD header (NumObj/MsgType) builder  [reused, pd_dispatch.h]
 *   pd_tx_commit_engine            @0xE1C6  — TX commit                           [reused, pd_dispatch.h]
 *
 * VDO TX BUFFER LAYOUT (verified by disasm of 0xAA36/0xDDAD/0xD852/0xE120):
 *   The Structured-VDM Header occupies the FIRST VDO slot E422:E425 (same physical bytes the
 *   Request RDO used). Response VDOs follow at E426, E42A, E42E(/E430), E432, ... (4-byte aligned).
 *   pd_vdm_hdr_build(cmdtype, cmd):  [@0xE120]
 *     E422 = (((cmdtype<<6) | cmd) & 0xCF)   ; VDM-hdr byte0: CmdType (7:6) | Command (4:0)
 *     E423 = (SOP==1) ? 0x80 : 0xA8          ; structured-VDM bit + version
 *     E424 = 0, E425 = 0xFF                   ; SVID placeholder (caller overwrites)
 *   cmdtype: 0=REQ, 1=ACK, 2=NAK, 3=BUSY.  cmd: 1=Disc_ID,2=Disc_SVIDs,3=Disc_Modes,4=EnterMode.
 *
 * PD MESSAGE LENGTH 0x07C4 = 2 (PD header) + 4*NumVDOs:
 *   Discover_ID  USB4 = 0x16 (5 VDOs), USB3 = 0x12 (4 VDOs)
 *   Discover_SVIDs / Discover_Modes  = 0x0A (2 VDOs)
 *
 * IDENTITY CONSTANTS (from stock):
 *   VID 0x174C (ASMedia/USB-IF), ID-header VDO bytes E426=0x4C,E427=0x17
 *   SVID 0x8087 (Intel Thunderbolt): bytes lo=0x87, hi=0x80
 *   TBT3 mode 0x00000001
 *
 * CC-CONTROLLER TX STROBE — NOTE THE DEVIATION FROM THE REQUEST PATH:
 *   The Request RDO (pd_build_send_request_rdo) drives the CC10/CC11/CC12/CC13 PHY command lane.
 *   The VDM TX (stock @0x9BBF) drives a DIFFERENT lane: CC88/CC89/CC8A/CC8B (the same command
 *   interface the keystone hard-reset/term-init uses). Verified by disasm of 0x9BBF:
 *     CC88 = (CC88 & 0xF8) | 3   (PD-message-TX opcode 3)
 *     CC8A = 0
 *     CC8B = 0x50                (timeout/param)
 *     CC89 = 1                   (go); poll CC89.1 (done); CC89 = 2 (ack)
 *   then pd_tx_commit_engine() transmits the staged E420.. buffer.
 */

/* usb4_connect_u4 @0xA3F5 — post-Enter_USB connect handler (defined in usb4.h, included after). */
static void usb4_connect_u4(void);

/* VDM identity constants */
#define VDM_VID_LO        0x4C   /* 0x174C low byte  */
#define VDM_VID_HI        0x17   /* 0x174C high byte */
#define VDM_TBT_SVID_LO   0x87   /* 0x8087 low byte (Intel Thunderbolt) */
#define VDM_TBT_SVID_HI   0x80   /* 0x8087 high byte */

/* ---------- pd_vdm_hdr_build @0xE120 — build the VDM-header VDO in E422:E425 ---------- */
/* cmdtype lands in E422 bits 7:6, cmd in bits 4:0 (mask 0xCF). E423 = structured-VDM byte
 * (0x80 SOP, 0xA8 SOP'). E424/E425 = SVID placeholder (0x00/0xFF) overwritten by the caller. */
static void pd_vdm_hdr_build(uint8_t cmdtype, uint8_t cmd) {
  PR(0xE422) = (uint8_t)((((uint8_t)(cmdtype << 6)) | cmd) & 0xCF);
  PR(0xE423) = (PR(0x07CA) == 1) ? 0x80 : 0xA8;
  PR(0xE424) = 0x00;
  PR(0xE425) = 0xFF;
}

/* ---------- VDM NAK (dd0e/95a0/95a2) ----------
 * Build a NAK (cmdtype 2) echoing the received SVID, length 0x07C4 = 6 (1 VDM-hdr VDO).
 * dd0e seeds the SOP/header (pd_tx_set_sop_header(1,0xF)); 95a2 builds the VDM hdr + SVID echo. */
static void vdm_nak(uint8_t cmd, uint8_t svid_lo, uint8_t svid_hi) {
  pd_tx_set_sop_header(1, 0x0F);   /* dd0e: 1 VDO, MsgType 0x0F (VDM) */
  pd_vdm_hdr_build(2, cmd);        /* 95a2: cmdtype 2 = NAK */
  PR(0xE424) = svid_lo;            /* echo received SVID lo */
  PR(0xE425) = svid_hi;            /* echo received SVID hi */
  PR(0x07C4) = 6;                  /* 2 PD header + 4 VDM-hdr VDO */
}

/* ---------- Discover_Identity responder @0xAA36 ----------
 * Builds the ID ACK VDO chain. SOP header = pd_tx_set_sop_header(5|4, 0xF) (5 VDOs for USB4,
 * 4 for USB3). VDM hdr = pd_vdm_hdr_build(1=ACK, 1=Disc_ID). VDOs:
 *   E426/E427 = 0x4C/0x17        ID Header VDO low half = VID 0x174C
 *   E428      = 0x40 (USB4 SOP==2) else 0     ID Header VDO high half (modal/product-type bits)
 *   E429      = 0x54 (USB4-capable: 0x7BC==0 && 0x09F9 bit7) else 0x50
 *   E42A      = (95F9 clears E42A-E42E)        Cert Stat VDO = 0
 *   E430/E431 = 0x0A57/0x0A58                  Product VDO (PID/bcdDevice from RAM)
 *   USB4 only (SOP==2): Product-Type VDOs E432-E435 from 0x09F9 (mode/gen bits, E434=0x80,
 *                       E435=0x6D if Gen-bits set else 0x65)
 *   0x07C4 = 0x16 (USB4) / 0x12 (USB3) */
static void vdm_build_discover_id(void) {
  uint8_t sop = PR(0x07CA);
  uint8_t f9, g23;

  /* PD header: 5 VDOs for USB4 (SOP==2), 4 for USB3. MsgType 0x0F. */
  pd_tx_set_sop_header((sop == 2) ? 5 : 4, 0x0F);
  pd_vdm_hdr_build(1, 1);          /* ACK, Discover_Identity */

  /* ID Header VDO (E426-E429) */
  PR(0xE426) = VDM_VID_LO;         /* 0x4C */
  PR(0xE427) = VDM_VID_HI;         /* 0x17  -> VID 0x174C */
  PR(0xE428) = (sop == 2) ? 0x40 : 0x00;
  if (PR(0x07BC) == 0 && (PR(0x09F9) & 0x80))
    PR(0xE429) = 0x54;             /* USB4-capable */
  else
    PR(0xE429) = 0x50;

  /* Cert Stat VDO (E42A-E42D) + E42E cleared (stock 95F9 clears E42A..E42E). */
  PR(0xE42A) = 0; PR(0xE42B) = 0; PR(0xE42C) = 0; PR(0xE42D) = 0; PR(0xE42E) = 0;

  /* Product VDO (E430:E431 = PID/bcdDevice bytes from PD RAM 0x0A57/0x0A58). */
  PR(0xE430) = PR(0x0A57);
  PR(0xE431) = PR(0x0A58);

  /* Product-Type VDO(s) — USB4 only. */
  if (sop == 2) {
    f9 = PR(0x09F9);
    g23 = f9 & 0x03;
    PR(0x0AA8) = (g23 != 0) ? 3 : 2;
    if (f9 & 0x80) PR(0x0AA8) |= 0x08;
    if (PR(0x07BC) == 0) PR(0xE432) = PR(0x0AA8);
    else                 PR(0xE432) = 2;
    PR(0xE433) = 0;
    PR(0xE434) = 0x80;
    if (PR(0x07BC) == 0 && g23 != 0) PR(0xE435) = 0x6D;
    else                             PR(0xE435) = 0x65;
  }

  PR(0x07C4) = (sop == 2) ? 0x16 : 0x12;
}

/* ---------- Discover_SVIDs responder @0xDDAD ----------
 * Only ACKs when the received SVID is the PD SID 0xFF00 terminator and 0x09F9 bit7 (USB4 enabled).
 * The host's post-[SB Con] Discover_SVIDs carries SVID 0xFF00 (the PD-defined SID terminator:
 * hi byte 0xFF, lo byte 0x00); stock vdm_build_discover_sids_resp@0xDDAD ACKs exactly that.
 * Advertises ONE SVID: 0x8087 (Intel TBT) in the low half, 0x0000 terminator in the high half.
 *   E426/E427 = 0/0 (terminator hi half), E428/E429 = 0x87/0x80 (SVID 0x8087)
 *   0x07C4 = 0x0A (2 VDOs). Else NAK. */
static void vdm_build_discover_sids(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  /* Stock @0xddb0-ddb8: 96ae loads A=IRAM[0x07] (hi, =rx_svid_hi @0xAA7) / R2=IRAM[0x06]
   * (lo, =rx_svid_lo @0xAA6); guard = CPL A; ORL A,R2; JNZ NAK -> ACK iff (~hi | lo)==0,
   * i.e. hi==0xFF AND lo==0x00 (PD SID 0xFF00), AND 0x09F9.7 (USB4-cap, the 2nd ACK gate). */
  if (((uint8_t)~rx_svid_hi | rx_svid_lo) == 0 && (PR(0x09F9) & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);   /* 9730: 2 VDOs, MsgType 0x0F */
    pd_vdm_hdr_build(1, 2);          /* ACK, Discover_SVIDs */
    PR(0xE426) = 0x00;               /* SVID-list VDO: hi half terminator */
    PR(0xE427) = 0x00;
    PR(0xE428) = VDM_TBT_SVID_LO;    /* 0x87 */
    PR(0xE429) = VDM_TBT_SVID_HI;    /* 0x80  -> SVID 0x8087 */
    PR(0x07C4) = 0x0A;
    return;
  }
  vdm_nak(2, rx_svid_lo, rx_svid_hi);
}

/* ---------- Discover_Modes responder @0xD852 ----------
 * Guard: received SVID == 0x8087 (hi^0x80==0 && lo^0x87==0) && 0x09F9 bit7. Echoes SVID in the
 * VDM header E424:E425 = 0x87/0x80; Mode VDO E426=1 (TBT3 mode 0x00000001), E427/E428/E429 = 0.
 *   0x07C4 = 0x0A (2 VDOs). Else NAK. */
static void vdm_build_discover_modes(uint8_t rx_svid_lo, uint8_t rx_svid_hi) {
  if (rx_svid_hi == 0x80 && rx_svid_lo == 0x87 && (PR(0x09F9) & 0x80)) {
    pd_tx_set_sop_header(2, 0x0F);   /* 9730 */
    pd_vdm_hdr_build(1, 3);          /* ACK, Discover_Modes */
    PR(0xE424) = VDM_TBT_SVID_LO;    /* echo SVID 0x8087 */
    PR(0xE425) = VDM_TBT_SVID_HI;
    PR(0xE426) = 0x01;               /* Mode VDO = TBT3 mode 0x00000001 */
    PR(0xE427) = 0x00;
    PR(0xE428) = 0x00;
    PR(0xE429) = 0x00;
    PR(0x07C4) = 0x0A;
    return;
  }
  vdm_nak(3, rx_svid_lo, rx_svid_hi);
}

/* ---------- usb4_mode_entry_commit @0xD78A — device-side USB4 mode-entry latch ----------
 * If 0x09F9 bit6: 0x0ACD=3, 0x0ACE=1, 92E1=0x10 (USB SW-DMA / mode-entry trigger), clear 9090 bit7
 * (USB global int mask). Else: 0x0ACD=1; 0x0ACE = 0x0D if (0x09F9 & 0x81)==0 else 5.
 * NOTE: bba8(0x92C2) in stock is a banked helper (USB engine kick); reproduced as the 92C2 RMW it
 * performs is not on the contract-critical path — left as the direct latch writes per RE. */
static uint8_t usb4_mode_entry_commit(void) {
  uint8_t f9 = PR(0x09F9);
  if (f9 & 0x40) {
    PR(0x0ACD) = 3;
    PR(0x0ACE) = 1;
    PR(0x92E1) = 0x10;                 /* USB4 mode-entry latch */
    PR(0x9090) = PR(0x9090) & 0x7F;    /* clear USB global int mask bit7 */
    return 4;                          /* stock d7b1: R7=4 (USB4 mode) */
  }
  PR(0x0ACD) = 1;
  PR(0x0ACE) = ((f9 & 0x81) == 0) ? 0x0D : 0x05;
  return 1;                            /* stock d7ca: R7=1 */
}

/* ---------- EnterMode responder @0xB966 ----------
 * If target SVID==0x8087 && obj-pos==1 && 0x09F9 bit7 && 0x07BC==0: enter Thunderbolt alt-mode:
 * set 0x07B9=1 (EnterMode TBT) and 0x07BB=1 (connect-pending), print [Enter_TBT]. Otherwise build
 * a generic EnterMode ACK (pd_vdm_hdr_build(2-VDO? -> 1 VDO ACK), E423 |= obj-pos). */
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
  PR(0xE423) = PR(0xE423) | (PR(0x0AAA) & 0x07);
  PR(0xE424) = svid_lo;
  PR(0xE425) = svid_hi;
  PR(0x07C4) = 6;
}

/* ---------- pd_handle_enter_usb @0xA036 -> usb4_connect_decide @0xA0A7 ----------
 * Enter_USB Data Message. Parse the Enter_USB Data Object (VDO0): USB-Mode field (bits 6:4 of
 * VDO0 byte1) -> 0x0AA6 (0=USB2,1=USB3.2,2=USB4); cable-current bit (VDO0 byte1 bit5) -> bVar3.
 * Then usb4_connect_decide:
 *   - If (0x09F9 & 3)==0 (USB4-capable default): 9664 control-header, 0x09FA=4,
 *     usb4_mode_entry_commit(), 0x0AE2=mode. (latch USB4 mode entry, no Accept VDO.)
 *   - Else if mode==2 (USB4) && 0x07BC==0: pd_tx_set_sop_header(0,3) = Accept, 0x07BB=1,
 *     print [Enter_USB4]. (cable-current set -> 0x07BB=1.)
 *   - Else reject variants.
 * Tail: 95e4 (0x07C4=2), pd_tx_commit_engine. (This handler sends its own control reply via the
 * Source_Cap-path engine, NOT the CC88 VDM strobe.) */
static void pd_handle_enter_usb(void) {
  uint16_t base = pd_rx_ptr();
  uint16_t vdo0 = (uint16_t)(base + 2);
  /* Enter_USB Data Object (EUDO), little-endian in the RX buffer:
   *   VDO0[1] bit5 = cable-current flag (stock 0xA03F-0xA046: byte[VDO0+1] & 0x20).
   *   VDO0[3] bits 6:4 = USB-Mode field -> 0x0AA6 (stock 0xA0A2-0xA0A6: byte[VDO0+3] & 0x70 >>4).
   *     0=USB2, 1=USB3.2, 2=USB4.  EUDO 0x2487E000 -> VDO0[3]=0x24 -> mode=2=USB4. */
  uint8_t cable_cur = (uint8_t)((PR(vdo0 + 1) & 0x20) >> 5);
  uint8_t mode = (uint8_t)((PR(vdo0 + 3) & 0x70) >> 4);
  uint8_t f9;

  PR(0x0AA6) = mode;
  pd_tx_buf_clear();                              /* e73a (clear TX before staging the reply) */

  f9 = PR(0x09F9);
  if ((f9 & 0x03) == 0) {
    /* USB4-capable default state: latch mode entry, control reply length 2. */
    PR(0xE405) &= 0xF8;                           /* 9664 */
    PR(0x09FA) = 4;
    usb4_mode_entry_commit();                     /* d78a: 92E1=0x10 if 0x09F9.6 */
    PR(0x0AE2) = mode;
  } else if (mode == 2 && PR(0x07BC) == 0) {
    /* USB4 requested + not in alt-mode: Accept and flag connect-pending.
     * Stock @0xA108-0xA11F: build Accept header (dd12), then iff cable_cur:
     *   0x07BB=1 (connect-pending), print "[Enter_USB 4]"@0x229e, **0x07BA=1** (Connect_U4 gate).
     * The prior handmade transcription set only 0x07BB and dropped 0x07BA — so the tail's
     * `if (0x07BA) -> [Connect_U4] -> usb4_connect_u4()` never fired. 0x07BA is REQUIRED. */
    pd_tx_set_sop_header(0, 3);                   /* dd12: 0 VDO, MsgType 3 = Accept */
    if (cable_cur) {
      PR(0x07BB) = 1;                             /* connect-pending */
      uart_puts("[Enter_USB 4]");
      PR(0x07BA) = 1;                             /* a11f: Connect_U4 gate (WAS MISSING) */
      /* RE-AUDIT #3/D(a): on the live AMD/TB4 mode==2 route the c9a8 connect dispatcher's gate
       * (0x09FA.2 && 0x0AF1.0 && (0x07E8||0x07EB)) is CLEAR -> bank0_8a89 can never be driven from
       * a host link-event. Stock's setter is the C80A.4 a522 link-width path (0x09FA|=4), which the
       * Intel host never reaches with handmade. Mirror it here so the link-event entry is armed.
       * 0x07E8/0x0AF1.0 are set on the USB4 route entry (pd.h) + connect (usb4.h). */
      PR(0x09FA) = PR(0x09FA) | 0x04;             /* 0x09FA.2 -> c9a8 gate bit (a522 mirror) */
      PR(0x07E8) = 1;                             /* D(d): USB4 route entry -> c9a8 gate term */
    }
  } else {
    /* Reject (mode not USB4 / wrong state). */
    pd_tx_set_sop_header(0, 4);                   /* dd12: MsgType 4 = Reject */
  }

  PR(0x07C4) = 2;                                 /* 95e4: control reply length */
  pd_tx_commit_engine();                          /* e1c6 */

  /* Stock tail @0xA167: after sending Accept, if the Connect_U4 gate (0x07BA) is set, print
   * "[Connect_U4]" and (when 0x07ED==0) run the USB4 connect handler @0xA3F5, which drives the
   * sideband (b230 flip + sb_block_init) + tunnel route setup. This is the PD->USB4 handoff. */
  if (PR(0x07BA) != 0) {
    uart_puts("[Connect_U4]");
    if (PR(0x07ED) != 0) {
      PR(0x07ED) = 0;                             /* a17c: one-shot suppress, then return */
    } else {
      usb4_connect_u4();                          /* a17f -> 0xA3F5 (SB/tunnel bring-up) */
    }
  }
}

/* ---------- VDM TX CC strobe + commit (stock @0x9BBF tail) ----------
 * Drive the CC88/CC89 command interface (opcode 3 = PD-message TX), then commit the staged buffer.
 * This is the SAME strobe the keystone uses, NOT the CC10 lane the Request uses. */
static void vdm_tx_strobe_commit(void) {
  PR(0xCC88) = (PR(0xCC88) & 0xF8) | 0x03;     /* 9627: opcode 3 */
  PR(0xCC8A) = 0;                               /* 9627 */
  PR(0xCC8B) = 0x50;                            /* 955e: timeout/param */
  PR(0xCC89) = 0x01;                            /* 955e: go */
  { uint16_t g = 0; while (!((PR(0xCC89) >> 1) & 1) && ++g < PD_WAIT_LIMIT);
    if (g >= PD_WAIT_LIMIT) pd_cc_timeout = 1; }
  PR(0xCC89) = 0x02;                            /* 964f: ack */
  if (PR(0xE40F) & 0x01) return;                /* RX pending -> abort (stock 0x9be2) */
  pd_tx_commit_engine();                        /* e1c6: transmit */
}

/* ---------- vdm_tx_dispatch @0x9AC4 — Structured VDM command dispatch (responder) ----------
 * Parses the received VDM header from the RX buffer (VDO0 at RX base+2 = 0xE440+0x20*0x07C1 + 2):
 *   cmd      = RX_VDO0[0] & 0x1F      -> 0x0AA5
 *   objpos   = RX_VDO0[1] & 0x07      -> 0x0AA3   (header byte1 bits 2:0)
 *   svid     = RX_VDO0[2], RX_VDO0[3] -> 0x0AA6 (lo), 0x0AA7 (hi)
 * Then (only when 0x07BC==0, i.e. not yet in alt-mode) dispatches on cmd. Each builder clears the
 * TX buffer (e73a) implicitly here, builds its VDOs, then we strobe + commit. */
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

  /* Stock gate: only respond to discovery while 0x07BC==0 (not yet in alt-mode). The >=2 path
   * (already in alt-mode) and the ==1 path are out-of-scope tunnel states; we NAK-on-default. */
  if (PR(0x07BC) != 0) {
    vdm_nak(cmd, svid_lo, svid_hi);
    vdm_tx_strobe_commit();
    return;
  }

  /* Clear the TX VDO buffer (e73a) before the builder writes (stock 96ae does this). */
  pd_tx_buf_clear();

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
