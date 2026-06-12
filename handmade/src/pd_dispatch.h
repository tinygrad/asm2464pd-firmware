#ifndef PD_DISPATCH_H
#define PD_DISPATCH_H
/*
 * USB-PD message dispatcher — transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) PD policy-engine RX path. Ghidra body offset == fw_tinygrad offset.
 *
 * Included AFTER pd.h, so PR(a)=XDATA_REG8V(a), uart_putc/uart_puts and the PD state RAM
 * (0x07xx / 0x0AAx) are all in scope.
 *
 *   E40F.0 (msg received) -> pd_rx_message_dispatch @0x83D6
 *      decode header (NumObj/MsgID/MsgType/SOP)
 *      NumObj>0, MsgType==1 (Source_Cap):
 *         pd_select_pdo_from_source_cap @0xABF5   -> pick PDO, 0x07C5=1
 *         0x07BD = 2
 *         pd_build_send_request_rdo  @0xACD4       -> stage Request RDO in E420-E425, CC TX strobe
 *            pd_tx_commit_engine     @0xE1C6       -> E403<-0x07C4, E41C|=1, wait, bump MsgID
 *      NumObj==0 (CONTROL): switch(MsgType) -> Accept/Reject/PS_RDY/Wait/GoodCRC ...
 *
 * RX/TX BUFFER ADDRESSING (disasm of 0x96d4/0x96e1 and the 0x83D6 header reads):
 *   RX message buffer base = 0xE440 + 0x20*xdata[0x07C1]   (idx = RX slot)
 *     0x96d4: stows the 16-bit pointer in 0x07BF(hi)/0x07C0(lo); idx=0 -> 0xE440.
 *     0x96e1: loads 0x07BF/0x07C0 into a DPTR and returns it.
 *     0x971e: returns low byte of the PDO/data area = base+2.
 *   Header (little-endian in the HW RX buffer):
 *     hdr[0] = RX[base+0]: MsgType = hdr[0] & 0x1F ; SOP-field 0x07CA = hdr[0] >> 6
 *     hdr[1] = RX[base+1]: NumDataObjects = (hdr[1]>>4)&7 ; MessageID = (hdr[1]>>1)&7
 *   Data objects (PDOs / RDO) start at base+2 = 0xE442 (4 bytes each).
 *
 *   TX message buffer = 0xE420-0xE43F:
 *     E420 = SOP/control + header byte0   (0x40=SOP, 0x80=SOP'/cable)
 *     E421 = header byte1 (NumObj<<4 | ...)
 *     E422..E425 = first data object (the Request RDO)
 *     E403 = TX byte-length descriptor (loaded from 0x07C4 at commit time)
 *     E41C bit0 = TX go/busy doorbell.
 */

/* Forward declarations (mutual recursion between handlers and the common send tail). */
static void pd_tx_commit_engine(void);
static void pd_rx_nak_send(void);
static void pd_ctrl_goodcrc(void);
/* Structured-VDM responder + Enter_USB handler (defined in vdm.h, #included after pd_dispatch.h). */
static void vdm_tx_dispatch(void);
static void pd_handle_enter_usb(void);

/* ---------- low-level RX-pointer helpers (0x96d4 / 0x96e1 / 0x971e) ---------- */

/* pd_rx_ptr @0x96d4: stash the 16-bit RX buffer pointer (hi:lo) in 0x07BF/0x07C0.
 * Base for slot idx = 0xE440 + 0x20*idx (idx = 0x07C1). */
static uint16_t pd_rx_ptr(void) {
  uint8_t idx = PR(0x07C1);
  uint16_t base = (uint16_t)(0xE440u + (uint16_t)(0x20u * idx));
  PR(0x07BF) = (uint8_t)(base >> 8);
  PR(0x07C0) = (uint8_t)(base & 0xFF);
  return base;
}
/* re-read the stowed pointer (0x96e1) */
static uint16_t pd_rx_ptr_get(void) {
  return ((uint16_t)PR(0x07BF) << 8) | PR(0x07C0);
}

/* ---------- TX engine commit (pd_tx_commit_engine @0xE1C6) ----------
 * Shared "send the message staged in E420-E43F" path (14 stock callers):
 *   - 0xE09A idle wait until engine idle
 *   - E403 <- 0x07C4 (TX byte-length descriptor)
 *   - 0x9605: E41C = (E41C & 0xFE) | 1   (TX go)
 *   - poll E41C bit0 until HW clears it (TX accepted/done)
 *   - 0x07C3 = (0x07C3 + 1) & 7   (bump local TX MessageID counter)
 *   - 0x07B7 = 0
 */
static void pd_tx_commit_engine(void) {
  { uint16_t g; for (g = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && g < 0x4000; g++); }  /* 0xE09A idle wait */
  REG_CMD_CTRL_E403 = PR(0x07C4);                     /* TX length descriptor */
  REG_CMD_BUSY_STATUS = (REG_CMD_BUSY_STATUS & 0xFE) | 0x01;     /* 0x9605: trigger TX */
  { uint16_t g; for (g = 0; (REG_CMD_BUSY_STATUS & 0x01) && g < 0x4000; g++); }   /* wait TX accepted */
  PR(0x07C3) = (PR(0x07C3) + 1) & 7;           /* bump TX MessageID */
  PR(0x07B7) = 0;
}

/* CC-event W1C clear (0x95c2): write 4 then 2 to a CC event reg pair (e.g. CC81/CC99). */
static void pd_cc_event_clear(uint16_t reg) {
  PR(reg) = 0x04;
  PR(reg) = 0x02;
}
/* e933: clear the CC attach event (CC81) before a state transition. */
static void pd_e933_clear_cc81(void) { pd_cc_event_clear(0xCC81); }

/* e81b(p1,p2): arm the CC sender-response / PS-transition timer:
 *   CC82 = p1 ; CC83 = p2 ; CC81 W1C then CC81 = 1 (go). */
static void pd_arm_cc_timer(uint8_t p1, uint8_t p2) {
  REG_CPU_CTRL_CC82 = p1;
  REG_CPU_CTRL_CC83 = p2;
  pd_cc_event_clear(0xCC81);
  REG_CPU_INT_CTRL = 0x01;                /* CC81 go */
}

/* ---------- TX header builder (pd_tx_set_sop_header @0xDD12) ----------
 * nobj = number of data objects.
 *   - read SOP 0x07CA; E420 = (SOP==2||3) ? 0x80 : 0x40   (SOP'/cable vs SOP)
 *   - 0x9635: E405 &= 0xF8 ; E421 = (nobj & 7) << 4   (NumDataObjects into byte1 bits6:4)
 *   - 0x96f7: E420 = (E420 & 0xC0) | (msgtype)   (keep SOP-type bits, OR in MessageType) */
static void pd_tx_set_sop_header(uint8_t nobj, uint8_t msgtype) {
  uint8_t sop = PR(0x07CA);
  /* dd12: E420 bits 7:6 select the SpecRev/SOP type (0x40=SOP, 0x80=SOP'/cable). */
  if (sop == 2 || sop == 3) REG_CMD_TRIGGER = 0x80; else REG_CMD_TRIGGER = 0x40;
  REG_CMD_CFG_E405 &= 0xF8;                               /* 9635 */
  /* PD header byte1 (E421): NumDataObjects in bits 6:4, TX MessageID in bits 3:1.
   * The host reads the header NumObj field directly (NOT derived from the TX length). */
  REG_CMD_MODE_E421 = (uint8_t)(((nobj & 7) << 4) | ((uint8_t)(PR(0x07C3) << 1) & 0x0E));
  /* PD header byte0 (E420): SpecRev in bits 7:6 (kept), MessageType in bits 4:0. */
  REG_CMD_TRIGGER = (uint8_t)((REG_CMD_TRIGGER & 0xC0) | msgtype);
}

/* e73a: zero the PD TX message buffer E420-E43F (32 bytes). */
static void pd_tx_buf_clear(void) {
  uint8_t i;
  for (i = 0; i < 0x20; i++) PR(0xE420 + i) = 0;
}

/* ---------- PDO selection (pd_select_pdo_from_source_cap @0xABF5) ----------
 * PDO[0] of any Source_Cap is the guaranteed vSafe5V fixed supply, so we request object
 * position 1 at its offered current. 0x07D3/0x07D4 hold the selected PDO's operating-current
 * bytes that pd_build_send_request_rdo packs into the RDO. */
static void pd_select_pdo_from_source_cap(void) {
  /* RX data objects start at header+2: base = 0xE440 + 0x20*0x07C1, so PDO[0] is at 0xE442.
   * Fixed-supply max-operating-current is bits 9:0 (10mA units), little-endian. */
  uint16_t pdo0 = (uint16_t)(0xE442u + (uint16_t)(0x20u * PR(0x07C1)));
  PR(0x07D4) = PR(pdo0 + 0);          /* operating current bits 7:0 */
  PR(0x07D3) = PR(pdo0 + 1) & 0x03;   /* operating current bits 9:8 */
  PR(0x07C7) = 0;                     /* PDO index 0 -> object position 1 */
  PR(0x07C5) = 1;                     /* selection valid */
}

/* ---------- Request builder + send (pd_build_send_request_rdo @0xACD4) ----------
 * Stages the Request message (header + RDO) in E420-E425, sets PD substate 0x07BD=3
 * (waiting Accept/PS_RDY), strobes the CC controller PD-message TX command (CC10 opcode 3),
 * then commits the engine.
 *
 * RDO packing (PD Fixed RDO, object position = 0x07C7+1):
 *   E422 = 0x07D4                                    (op-current low byte)
 *   E423 = 0x07D3 | (0x07D4 << 2)                    (current bits packed)
 *   E424 = ((0x07D3 << 2) & 0x0C) | (0x07D4 >> 6)
 *   E425 = 1; |= ((0x07C7+1)&7)<<4 (object position) ; |= 2 ; &= 0xFE
 */
static void pd_build_send_request_rdo(void) {
  uint8_t d3 = PR(0x07D3);           /* operating current bits 9:8 */
  uint8_t d4 = PR(0x07D4);           /* operating current bits 7:0 */

  pd_tx_buf_clear();                 /* e73a: clear TX buf 0xE420-0xE43F */
  pd_tx_set_sop_header(1, 2);        /* 1 data object, MessageType 2 = Request */

  /* Fixed RDO bit packing per stock @0xACD4 (object position = 0x07C7+1). */
  REG_CMD_PARAM = d4;
  REG_CMD_STATUS = d3;
  REG_CMD_STATUS |= (uint8_t)(d4 << 2);
  REG_CMD_ISSUE = (uint8_t)((d3 << 2) & 0x0C);
  REG_CMD_ISSUE |= (uint8_t)(d4 >> 6);
  REG_CMD_TAG = 1;
  REG_CMD_TAG |= (uint8_t)(((PR(0x07C7) + 1) & 7) << 4);   /* object position */
  REG_CMD_TAG |= 2;
  REG_CMD_TAG &= 0xFE;

  PR(0x07C4) = 6;                    /* 95af: TX length = 2 header + 4 RDO */
  PR(0x07BD) = 3;                    /* substate: Request sent, waiting Accept/PS_RDY */

  /* CC controller: issue the PD-message TX command (opcode 3 in CC10[2:0]). */
  REG_TIMER0_DIV &= 0xF7;
  pd_cc_event_clear(0xCC11);                       /* 95c2(CC11): W1C the PHY-cmd event */
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | 0x03;         /* opcode 3 = transmit PD message */
  REG_TIMER0_THRESHOLD_HI = 0;
  REG_TIMER0_THRESHOLD_LO = 0x28;
  REG_TIMER0_CSR = 0x01;                               /* go */
  { uint16_t g = 0; while (!((REG_TIMER0_CSR >> 1) & 1) && ++g < PD_WAIT_LIMIT);
    if (g >= PD_WAIT_LIMIT) pd_cc_timeout = 1; }   /* wait done (flag timeout) */
  REG_TIMER0_CSR = 0x02;                               /* W1C ack */

  pd_tx_commit_engine();                           /* transmit (E403<-0x07C4, E41C|=1) */
  pd_arm_cc_timer(2, 0x30);                        /* arm SenderResponse timer */
}

/* ====================================================================================
 * CONTROL-message handlers (NumObj==0). Stock dispatches via a movc jump-table at
 * CODE:0x850A [hi,lo,key]; here a clean switch on MessageType. Stock table keys:
 *   key1=GoodCRC@0x870B  key3=Accept@0x8550  key4=Reject@0x8641  key5@0x8693
 *   key6=PS_RDY@0x8591   key8=Get_Sink_Cap@0x8686  key9=DR_Swap@0x86F3  keyC=Wait@0x860E
 *   keyE=Data_Reset@0x869A  keyF=Data_Reset_Complete@0x86AA  key10=Not_Supported@0x86BE
 *   key16@0x86E9  key18@0x86FC   (all other keys + default -> 0x871A pd_rx_nak_send)
 * 0x0D Soft_Reset is a handmade addition (no stock key); un-ported keys fold into default NAK. */

/* GoodCRC @0x870B: 96bf advance RX slot (0x07C1=(0x07C1+1)&(0x07D5-1)); if a pending TX is
 * staged (0x07B7!=0) commit it. */
static void pd_ctrl_goodcrc(void) {
  /* 96bf: rotate RX slot index 0x07C1 = (0x07C1 + 1) & (0x07D5 - 1) */
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
  if (PR(0x07B7) != 0) pd_tx_commit_engine();
}

/* Accept @0x8550: read substate 0x07BD.
 *   ==3 (Request sent): e933 clear; 0x07BD=4; arm PS_RDY timer; send-tail. */
static void pd_ctrl_accept(void) {
  uint8_t s;
  uart_puts("[Accept]");
  s = PR(0x07BD);
  if (s == 3) {
    pd_e933_clear_cc81();
    PR(0x07BD) = 4;
    pd_arm_cc_timer(0x10, 0x27);   /* 865f: e81b(0x10,0x27) arm PS_RDY timer, then 870b tail */
    pd_ctrl_goodcrc();
    return;
  }
  if (s == 0) {                /* 0x07DE==0 path: ignore-and-reset (e933 + 9684) */
    if (PR(0x07DE) != 0) { PR(0x07DE) = 0; }
    pd_e933_clear_cc81();
    pd_ctrl_goodcrc();
    return;
  }
  if (s == 0x0E) {             /* Data_Reset pending: e933, 0x07BD=0x0D, fall to reject-tail */
    pd_e933_clear_cc81();
    PR(0x07BD) = 0x0D;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();           /* 0x8717 NAK tail */
}

/* PS_RDY @0x8591: require substate 0x07BD==4 (Accept received). e933 clear.
 *   If decoded V (0x07D6==1 && 0x07D7==0x2C): "[5V3A]", 0x07B8=3.
 *   Else compare V against {0x2C..0x96} window: in-range -> "[5V1.5A]" 0x07B8=2;
 *        below -> 0x07B8=1 ; above -> 0x07B8=4. 0x07DE/0x07DF cleared. */
static void pd_ctrl_ps_rdy(void) {
  uint8_t v_hi, v_lo;
  uart_puts("[PS_RDY]");
  if (PR(0x07BD) != 4) { pd_ctrl_goodcrc(); return; }   /* != 4 -> 0x8717 NAK */
  pd_e933_clear_cc81();

  v_hi = PR(0x07D6);
  v_lo = PR(0x07D7);
  if (v_hi == 1 && v_lo == 0x2C) {
    uart_puts("[5V3A]");                 /* PD contract level 3 */
    PR(0x07B8) = 3;
  } else {
    /* 16-bit compare (v_hi:v_lo) against 0x012C and 0x0096 windows (stock SUBB chain). */
    uint16_t v = ((uint16_t)v_hi << 8) | v_lo;
    if (v >= 0x012C || v_hi >= 1) {
      uart_puts("[5V1.5A]");
      PR(0x07B8) = 2;
    } else if (v < 0x0096) {
      PR(0x07B8) = 1;
    } else {
      PR(0x07B8) = 4;
    }
  }
  PR(0x07DE) = 0;
  PR(0x07DF) = 0;
  pd_ctrl_goodcrc();
}

/* Reject @0x8641: substate 0x07BD.
 *   ==3: e933; if contract 0x07B8!=0 arm timer (e81b) else reset (9687); send-tail.
 *   ==0x0E: Data_Reset pending -> Type-C Error Recovery (cc_state_full_reset @0xD676). */
static void pd_ctrl_reject(void) {
  uint8_t s;
  uart_puts("[Reject]");
  s = PR(0x07BD);
  if (s == 3) {
    pd_e933_clear_cc81();
    if (PR(0x07B8) == 0) {
      /* 9687 reset path then e81b tail */
    }
    pd_arm_cc_timer(0, 0);   /* e81b tail (params from R6/R7 context) */
    pd_ctrl_goodcrc();
    return;
  }
  if (s == 0x0E) {
    /* Data_Reset pending -> cc_state_full_reset @0xD676 (out of scope; stock SJMP $ while HW
     * re-enumerates). TODO next milestone: cc_state_full_reset() */
    PR(0x07BD) = 0x0E;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* Wait @0x860E: require substate 0x07BD==3. e933. If contract 0x07B8!=0: arm CC timer
 * e81b(0xD0,0x07), wait CC81 attach event, ack CC81=2, send-tail. Else reset (9684). */
static void pd_ctrl_wait(void) {
  if (PR(0x07BD) != 3) { pd_ctrl_goodcrc(); return; }   /* != 3 -> 0x8717 NAK */
  pd_e933_clear_cc81();
  if (PR(0x07B8) != 0) {
    pd_arm_cc_timer(0xD0, 0x07);
    { uint16_t g = 0; while (!(REG_CPU_INT_CTRL & 0x02) && ++g < PD_WAIT_LIMIT); }  /* JNB CC81.1 */
    REG_CPU_INT_CTRL = 0x02;                                  /* W1C ack */
    pd_ctrl_goodcrc();
  } else {
    pd_ctrl_goodcrc();                                  /* 9684 reset tail */
  }
}

/* ---------- common send tails ----------
 * 0x871a: stage a 2-byte control GoodCRC/NAK response and commit. SOP(0x07CA)==1 -> dd12
 * header; else 9664/965d (clear E405 low). Then 95e4 (0x07C4=2), commit, 96bf (advance MsgID). */
static void pd_rx_nak_send(void) {
  if (PR(0x07CA) == 1) {
    /* control SOP path: header with 0 data objects. TODO next milestone: exact 871a reply type. */
    pd_tx_set_sop_header(0, 1);
  } else {
    REG_CMD_CFG_E405 &= 0xF8;        /* 9664/965d: clear E405 low for non-SOP control reply */
  }
  PR(0x07C4) = 2;                    /* 95e4: control message = 2 header bytes, no data */
  pd_tx_commit_engine();
  /* 96bf: advance expected RX MessageID slot */
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
}

/* ---------- Soft_Reset (control MsgType 0x0D) ----------
 * Handmade addition (stock table has no 0x0D key). The host boots mid-PD and rejects our
 * Request with a Soft_Reset when its expected device MessageID has drifted from our reset-to-0
 * counter. Per USB-PD §6.8.1: reset BOTH MessageID counters for this SOP, then reply Accept.
 *
 * Built from stock Accept primitive 0x95da (clear E420-E43F; pd_tx_set_sop_header(0,3)
 * MessageType 3 = Accept; 0x07C4=2) + pd_tx_commit_engine. The inbound Soft_Reset's GoodCRC
 * is auto-sent by the PD HW engine. */
static void pd_ctrl_soft_reset(void) {
  uart_puts("[Soft_Reset]");
  /* §6.8.1: reset MessageIDCounter (TX 0x07C3) and expected RX slot/MessageID (0x07C1)
   * BEFORE building the Accept so it goes out with MessageID=0. */
  PR(0x07C3) = 0;                 /* TX MessageID counter -> 0 */
  PR(0x07C1) = 0;                 /* RX slot / expected MessageID -> 0 */
  PR(0x07B7) = 0;                 /* no pending staged TX */
  pd_e933_clear_cc81();           /* e933: clear the CC attach event before replying */

  /* 0x95da: build the Accept control message (header MsgType 3, 0 data objects, len 2). */
  pd_tx_buf_clear();              /* e73a */
  pd_tx_set_sop_header(0, 3);     /* dd12: 0 data objects, MessageType 3 = Accept */
  PR(0x07C4) = 2;                 /* 95af-style: control length = 2 header bytes */

  /* §6.8.1: the Accept MUST carry wire MessageID=0. E421 bits 3:1 are the wire MessageID;
   * force them to 0. */
  REG_CMD_MODE_E421 &= 0xF1;

  pd_tx_commit_engine();          /* e1c6: transmit Accept with MessageID=0 (bumps 0x07C3 0->1) */

  /* Leave 0x07C3=1 so the device's next message (the Request after the host re-sends
   * Source_Cap) carries MessageID 1, per the restarted AtomicMessage Sequence. */

  /* Return to the ready-to-negotiate substate (0x07BD in {1,5,3,6}); 1 == init/ready. */
  PR(0x07BD) = 1;
}

/* CONTROL dispatch — switch over the 0x850A jump table. */
static void pd_dispatch_control(uint8_t msgtype) {
  switch (msgtype) {
    case 0x01: pd_ctrl_goodcrc();    break;   /* GoodCRC    */
    case 0x03: pd_ctrl_accept();     break;   /* Accept     */
    case 0x04: pd_ctrl_reject();     break;   /* Reject     */
    case 0x06: pd_ctrl_ps_rdy();     break;   /* PS_RDY     */
    case 0x0C: pd_ctrl_wait();       break;   /* Wait       */
    case 0x0D: pd_ctrl_soft_reset(); break;   /* Soft_Reset */
    /* 0x02 Reserved, 0x05 Ping, 0x07 Get_Source_Cap, 0x08 Get_Sink_Cap, 0x0A..0x0B PR/DR_Swap,
     * 0x0E Soft_Reset, 0x0F..0x10 — stock NAKs/handles via @0x8706/869x; out of scope */
    default:   pd_rx_nak_send();  break;   /* TODO next milestone: full control set */
  }
}

/* ====================================================================================
 * DATA-message handlers (NumObj>0). Out-of-scope ones (BIST/Enter_USB/VDM) print + NAK with
 * a TODO; the Source_Cap path is fully implemented.
 * ==================================================================================== */

/* 874c(lo,hi): print CODE string then return SOP (0x07CA). Source_Cap prints "[Source_Cap]". */

/* DATA dispatch */
static void pd_dispatch_data(uint8_t msgtype) {
  if (msgtype == 0x01) {
    /* Source_Cap @0x84b3. If SOP in {2,3} (cable) seed E420=0x80 via 9713. Require substate
     * 0x07BD in {1,5,3,6} else thunk_dc2d()+return. */
    uint8_t sop;
    uart_puts("[Source_Cap]");
    sop = PR(0x07CA);
    if (sop == 2 || sop == 3) {
      /* 0x84b3: 9713(0x80) writes E420=0x80; 0x84b8: E409 = (E409 & 0xF1) | 4. */
      REG_CMD_TRIGGER = 0x80;             /* 9713(0x80): seed TX control byte */
      REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x04;
    }
    {
      uint8_t s = PR(0x07BD);
      if (s != 1 && s != 5 && s != 3 && s != 6) {
        return;   /* thunk_dc2d: ignore Source_Cap (wrong substate) */
      }
    }
    pd_e933_clear_cc81();                 /* e933 */
    pd_select_pdo_from_source_cap();      /* 0xABF5 */
    PR(0x07BD) = 2;                       /* substate: building Request */
    pd_build_send_request_rdo();          /* 0xACD4 -> sends Request + commits engine */
    pd_ctrl_goodcrc();                    /* 870b tail */
    return;
  }
  if (msgtype == 0x03) {
    /* BIST data message. dc65 sets 0x07C8=1 (BIST mode) when RX[1]&0xF0==0x80.
     * TODO next milestone: dc65() BIST handler. */
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype >= 2 && msgtype < 8) {
    pd_rx_nak_send();   /* 0x02 Request (we are sink), 0x04..0x07: NAK (962e + 871a) */
    return;
  }
  if (msgtype == 0x08) {
    /* Enter_USB Data Message (PD3.1) @0x84E8: pd_handle_enter_usb @0xA036 sends Accept and
     * latches USB4 mode entry (vdm.h). We accept regardless of the substate gate. */
    uart_puts("[Enter_USB]");
    pd_handle_enter_usb();               /* a036 -> usb4_connect_decide -> Accept + mode-entry */
    pd_ctrl_goodcrc();                   /* 870b tail */
    return;
  }
  if (msgtype == 0x0F) {
    /* Structured VDM @0x848E: vdm_tx_dispatch @0x9AC4 builds + sends ACK/NAK (CC88 strobe +
     * pd_tx_commit_engine), then 870b GoodCRC tail. */
    uart_puts("[VDM]");
    vdm_tx_dispatch();                   /* 9ac4 — Discover_Identity/SVIDs/Modes/EnterMode */
    pd_ctrl_goodcrc();                   /* 870b tail */
    return;
  }
  pd_rx_nak_send();                      /* default 871a */
}

/* ====================================================================================
 * pd_rx_message_dispatch @0x83D6 — entry point (called from pd_rx_isr after E40F.0 W1C).
 * ==================================================================================== */
static void pd_rx_message_dispatch(void) {
  uint8_t e40f = REG_PHY_EVENT_E40F;
  uint8_t hdr0, hdr1, sop, ext_bit;
  uint16_t base;

  /* Gate: if E40F.7 (Soft_Rst) OR E40F.5 (Hard_Rst) set, bail (those are handled in the ISR). */
  if ((e40f & 0x80) || (e40f & 0x20)) return;

  /* Stage RX pointer (base = 0xE440 + 0x20*0x07C1) and parse the 2-byte header. */
  base = pd_rx_ptr();
  hdr1 = PR(base + 1);
  hdr0 = PR(base + 0);

  PR(0x07C2) = (uint8_t)((hdr1 >> 4) & 7);   /* NumDataObjects */
  PR(0x0AA2) = (uint8_t)((hdr1 >> 1) & 7);   /* MessageID */
  PR(0x0AA1) = (uint8_t)(hdr0 & 0x1F);       /* MessageType */
  sop = (uint8_t)(hdr0 >> 6);
  PR(0x07CA) = sop;                          /* SOP/port field */

  /* DEBUG: slot, raw header bytes, NumObj, MsgType */
  uart_puts("[D");
  uart_puthex(PR(0x07C1)); uart_putc(' ');
  uart_puthex(hdr0); uart_puthex(hdr1); uart_putc(' ');
  uart_puthex(PR(0x07C2)); uart_puthex(PR(0x0AA1)); uart_putc(']');

  /* GoodCRC / extended-message-id retry gate (957c/95ec). ext_bit = bit7 of RX[base+1]
   * (Extended header bit); the retry sub-path runs only when 0x07C8 clear AND ext_bit set AND
   * SOP==0. Source_Cap has ext_bit==0 and falls through to the NumObj dispatch. */
  ext_bit = (uint8_t)((PR(base + 1) >> 7) & 1);
  if (PR(0x07C8) != 0) return;               /* 0x07C8 set (BIST/chunk) -> stock LJMP 0x8739 ret */

  if (ext_bit != 0 && sop == 0) {
    /* Extended/chunked message-id retry path (956a/95eb/96b7/9703/95b6). Out of scope —
     * Source_Cap/Request never take it. TODO next milestone: chunked extended msgs. */
    pd_rx_nak_send();
    return;
  }

  if (PR(0x07C2) == 0) {
    pd_dispatch_control(PR(0x0AA1));         /* CONTROL message */
  } else {
    pd_dispatch_data(PR(0x0AA1));            /* DATA message */
  }
}

#endif /* PD_DISPATCH_H */
