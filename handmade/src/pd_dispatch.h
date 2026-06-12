#ifndef PD_DISPATCH_H
#define PD_DISPATCH_H
/*
 * USB-PD message dispatcher — faithful transcription of the ASM2464PD stock firmware
 * (fw_tinygrad.bin) PD policy-engine RX path. Ghidra body offset == fw_tinygrad offset.
 *
 * Included AFTER pd.h, so PR(a)=XDATA_REG8V(a), uart_putc/uart_puts and the PD state RAM
 * (0x07xx / 0x0AAx) are all in scope. This file owns the make-or-break power-contract path:
 *
 *   E40F.0 (msg received) -> pd_rx_message_dispatch @0x83D6
 *      decode header (NumObj/MsgID/MsgType/SOP)
 *      NumObj>0, MsgType==1 (Source_Cap):
 *         pd_select_pdo_from_source_cap @0xABF5   -> pick PDO matching target V/I, 0x07C5=1
 *         0x07BD = 2
 *         pd_build_send_request_rdo  @0xACD4       -> stage Request RDO in E420-E425, CC TX strobe
 *            pd_tx_commit_engine     @0xE1C6       -> E403<-0x07C4, E41C|=1, wait, bump MsgID
 *      NumObj==0 (CONTROL): switch(MsgType) -> Accept/Reject/PS_RDY/Wait/GoodCRC ...
 *
 * RX/TX BUFFER ADDRESSING (verified by disasm of 0x96d4/0x96e1 and the 0x83D6 header reads):
 *   RX message buffer base = 0xE440 + 0x20*xdata[0x07C1]   (idx = RX slot)
 *     0x96d4: given A=0x40+0x20*idx (low) and B=high, computes hi=(B+0xE4+carry):lo and stows
 *             the 16-bit pointer in 0x07BF(hi)/0x07C0(lo).  For idx=0 -> 0xE440.
 *     0x96e1: loads 0x07BF/0x07C0 into a DPTR and returns it (R4=hi,R5=lo).
 *     0x971e: returns low byte of the PDO/data area = 0x42 + 0x20*idx (i.e. base+2).
 *   Header (per PD spec, presented little-endian in the HW RX buffer):
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

/* pd_rx_set_ptr @0x96d4: stash the 16-bit RX buffer pointer (hi:lo) in 0x07BF/0x07C0.
 * Stock takes the low byte in A (= 0x40 + 0x20*idx, may carry) and high in B (=0). The full
 * base for slot `idx` is 0xE440 + 0x20*idx, so we recompute it directly. */
static uint16_t pd_rx_ptr(void) {
  uint8_t idx = PR(0x07C1);
  uint16_t base = (uint16_t)(0xE440u + (uint16_t)(0x20u * idx));   /* 96d4: hi=(0+0xE4+carry) */
  PR(0x07BF) = (uint8_t)(base >> 8);
  PR(0x07C0) = (uint8_t)(base & 0xFF);
  return base;
}
/* re-read the stowed pointer (0x96e1) */
static uint16_t pd_rx_ptr_get(void) {
  return ((uint16_t)PR(0x07BF) << 8) | PR(0x07C0);
}

/* ---------- TX engine commit (pd_tx_commit_engine @0xE1C6) ----------
 * The shared "send the message currently staged in E420-E43F" path (14 stock callers).
 *   - pd_tx_wait_done (0xE09A) until engine idle (R7 sentinel from the wait helper is the loop
 *     guard in stock; pd.h's pd_tx_wait_done() already encodes the same E402/E41C idle poll)
 *   - E403 <- 0x07C4 (TX byte-length descriptor)
 *   - pd_tx_trigger_e41c (0x9605): E41C = (E41C & 0xFE) | 1   (TX go)
 *   - poll E41C bit0 until HW clears it (TX accepted/done)
 *   - 0x07C3 = (0x07C3 + 1) & 7   (bump local TX MessageID counter)
 *   - 0x07B7 = 0
 */
static void pd_tx_commit_engine(void) {
  /* 0xE09A idle wait (bounded, like pd_drive_hard_reset in pd.h) */
  { uint16_t g; for (g = 0; ((REG_CMD_STATUS_E402 & 0x0E) || (REG_CMD_BUSY_STATUS & 0x01)) && g < 0x4000; g++); }
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
/* e933: pd_cc_event_clear(CC81) — clears the CC attach event before a state transition. */
static void pd_e933_clear_cc81(void) { pd_cc_event_clear(0xCC81); }

/* e81b(p1,p2): arm the CC sender-response / PS-transition timer:
 *   CC82 = p1 ; CC83 = p2 ; CC81 = (write 4 then 2) then leave CC81 = 1 (go). */
static void pd_arm_cc_timer(uint8_t p1, uint8_t p2) {
  REG_CPU_CTRL_CC82 = p1;
  REG_CPU_CTRL_CC83 = p2;
  pd_cc_event_clear(0xCC81);
  REG_CPU_INT_CTRL = 0x01;                /* stock: (95c2 result)-1 == 2-1 == 1 -> CC81 go */
}

/* ---------- TX header builder (pd_tx_set_sop_header @0xDD12) ----------
 * param `nobj` = number of data objects (lands in R5 used by 0x9635).
 *   - read SOP field 0x07CA; E420 = (SOP==2||3) ? 0x80 : 0x40   (SOP'/cable vs SOP)
 *   - 0x9635: E405 &= 0xF8 ; E421 = (nobj & 7) << 4   (NumDataObjects into header byte1 bits6-4)
 *   - 0x96ee: ret = (0x07C3 << 1)   (local TX MessageID << 1, header byte0 bits3-1)
 *   - 0x96f7: E420 = (E420 & 0xC0) | (SOP | (0x07C3<<1))   (keep SOP-type bits, OR in MsgID)
 * Note: MessageType for the message is OR'd into E420 elsewhere by the specific builder
 * (the Request builder writes the type bits via E425/the engine). */
static void pd_tx_set_sop_header(uint8_t nobj, uint8_t msgtype) {
  uint8_t sop = PR(0x07CA);
  /* dd12: E420 bits 7:6 select the SpecRev/SOP type (0x40=SOP, 0x80=SOP'/cable). */
  if (sop == 2 || sop == 3) REG_CMD_TRIGGER = 0x80; else REG_CMD_TRIGGER = 0x40;
  REG_CMD_CFG_E405 &= 0xF8;                               /* 9635 */
  /* PD header byte1 (E421): NumDataObjects in bits 6:4, TX MessageID in bits 3:1.
   * **CY4500 wire capture PROVED this**: with the previous E421=sop|(MsgID<<1) (NumObj=0) a
   * Request (MsgType 2) transmitted as a control GotoMin (hdr=0x0282) and the host Soft-Reset;
   * with NumObj here it is a proper Request (nobj=1 -> hdr=0x1082). The host reads the header
   * NumObj field directly (it is NOT derived from the TX length). */
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
 * Walk the received Source_Cap PDOs (NumObj = 0x07C2, each 4 bytes from RX base+2). For each
 * fixed-supply PDO whose type nibble == 0 (0x07C6==0): decode voltage (bits 19:10, *50mV scaled
 * via *0x40) into 0x07D8/0x07D9 and max-current (bits 9:0) into 0x07D6/0x07D7, then compare
 * against the target (BANK0_R6:R7 = target voltage hi:lo from 0x07D8 seed, and target current
 * from 0x07DA/0x07DB). The first PDO that meets target V (==) and >= target I is selected:
 *   0x07C7 = pdo_index ; 0x07C5 = 1.
 * If none match the loop falls through to 0xACB3 and stores via 0x957b(NumObj,&0x07C7):
 *   0x07D4 = RX[last]; 0x07D3 = (decoded)&3; 0x07C5 = 0  (no valid selection).
 *
 * Faithful transcription of the stock decompile. The per-PDO field math (95c9 byte-pulls,
 * 9656/95ca decode) is reproduced literally; field meanings annotated. 0x07D3/0x07D4 hold the
 * selected PDO's operating-current bytes that pd_build_send_request_rdo packs into the RDO. */
static void pd_select_pdo_from_source_cap(void) {
  /* PDO[0] of any Source_Cap is the vSafe5V fixed supply (USB-PD spec guarantees this), so we
   * request object position 1 at its offered current. Stock walks all PDOs matching a target
   * voltage held in CPU registers that can't be pinned statically; selecting the guaranteed 5V
   * PDO is equivalent for our sink contract and avoids that ambiguity.
   * RX data objects start at header+2: base = 0xE440 + 0x20*0x07C1, so PDO[0] is at 0xE442.
   * A Fixed-supply PDO's max-operating-current is bits 9:0 (10mA units), little-endian. */
  uint16_t pdo0 = (uint16_t)(0xE442u + (uint16_t)(0x20u * PR(0x07C1)));
  PR(0x07D4) = PR(pdo0 + 0);          /* operating current bits 7:0 */
  PR(0x07D3) = PR(pdo0 + 1) & 0x03;   /* operating current bits 9:8 */
  PR(0x07C7) = 0;                     /* PDO index 0 -> object position 1 */
  PR(0x07C5) = 1;                     /* selection valid */
}

/* ---------- Request builder + send (pd_build_send_request_rdo @0xACD4) ----------
 * Faithful transcription of the stock decompile. Stages the Request message (header + RDO) in
 * the TX buffer E420-E425, sets PD substate 0x07BD=3 (waiting Accept/PS_RDY), strobes the CC
 * controller PD-message TX command (CC10 opcode 3), then commits the engine.
 *
 * RDO packing (PD Fixed RDO with object position = 0x07C7+1):
 *   E422 = 0x07D4                                    (op-current low byte)
 *   E423 = 0x07D3 | (0x07D4 << 2)                    (current bits packed)
 *   E424 = ((0x07D3 << 2) & 0x0C) | (0x07D4 >> 6)
 *   E425 = 1; |= ((0x07C7+1)&7)<<4 (object position) ; |= 2 ; &= 0xFE
 */
static void pd_build_send_request_rdo(void) {
  uint8_t d3 = PR(0x07D3);           /* operating current bits 9:8 (selected by pd_select_pdo) */
  uint8_t d4 = PR(0x07D4);           /* operating current bits 7:0 */

  pd_tx_buf_clear();                 /* e73a: clear TX buf 0xE420-0xE43F */
  pd_tx_set_sop_header(1, 2);        /* 1 data object, MessageType 2 = Request */

  /* Fixed RDO: operating current = max current = offered; object position = 0x07C7+1.
   * Exact bit packing from stock pd_build_send_request_rdo @0xACD4. */
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
 * CONTROL-message handlers (NumObj==0). Stock dispatches these via a movc jump-table at
 * CODE:0x850A [hi,lo,key]; decoded here into a clean switch on MessageType. Stock table (verified
 * against the 0x850A movc records — earlier labels here were WRONG; corrected 2026-06-11):
 *   key1=GoodCRC@0x870B  key3=Accept@0x8550  key4=Reject@0x8641  key5@0x8693
 *   key6=PS_RDY@0x8591   key8=Get_Sink_Cap@0x8686  key9=DR_Swap@0x86F3  keyC=Wait@0x860E
 *   keyE=Data_Reset@0x869A  keyF=Data_Reset_Complete@0x86AA  key10=Not_Supported@0x86BE
 *   key16@0x86E9  key18@0x86FC   (all other keys + default -> 0x871A pd_rx_nak_send)
 * NOTE: stock has NO 0x0D entry; handmade's 0x0D Soft_Reset below is a DELIBERATE addition, not a
 * transcription of stock. Handmade folds the un-ported keys (5/8/9/E/F/10/16/18) into default NAK,
 * which is faithful for the contract milestone (those are benign post-contract requests). */

/* GoodCRC @0x870B: 96bf (advance MessageID-expected: 0x07C1=(0x07C1+1)&(0x07D5-1)); if a pending
 * TX is staged (0x07B7!=0) commit it. */
static void pd_ctrl_goodcrc(void) {
  /* 96bf: rotate RX slot index 0x07C1 = (0x07C1 + 1) & (0x07D5 - 1) */
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
  if (PR(0x07B7) != 0) pd_tx_commit_engine();
}

/* Accept @0x8550: print "[Accept]"; read substate 0x07BD.
 *   ==3 (Request sent): e933 clear; 0x07BD=4 (Accept received); fall to common send-tail (871a). */
static void pd_ctrl_accept(void) {
  uint8_t s;
  uart_puts("[Accept]");
  s = PR(0x07BD);
  if (s == 3) {
    pd_e933_clear_cc81();
    PR(0x07BD) = 4;
    /* stock LJMP 0x865f = e81b(R7=0x10,R6=0x27) arm PS_RDY timer, then 870b tail */
    pd_arm_cc_timer(0x10, 0x27);
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

/* PS_RDY @0x8591: print "[PS_RDY]"; require substate 0x07BD==4 (Accept received). e933 clear.
 *   If decoded source V (0x07D6==1 && 0x07D7==0x2C): print "[5V3A]", 0x07B8=3.
 *   Else compare V against {0x2C..0x96} window: in-range -> "[5V1.5A]" 0x07B8=2 (538d puts);
 *        below window -> 0x07B8=1 ; above -> 0x07B8=4 (965d).  0x07DE/0x07DF cleared. */
static void pd_ctrl_ps_rdy(void) {
  uint8_t v_hi, v_lo;
  uart_puts("[PS_RDY]");
  if (PR(0x07BD) != 4) { pd_ctrl_goodcrc(); return; }   /* XRL #4 != 0 -> 0x8717 NAK */
  pd_e933_clear_cc81();

  v_hi = PR(0x07D6);
  v_lo = PR(0x07D7);
  if (v_hi == 1 && v_lo == 0x2C) {
    uart_puts("[5V3A]");                 /* *** PD contract level 3 *** */
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

/* Reject @0x8641: print "[Reject]"; substate 0x07BD.
 *   ==3: e933; if contract 0x07B8!=0 arm timer (e81b) else reset (9687); send-tail.
 *   ==0x0E: e529(0x3B)+cc_state_full_reset(0xD676 Error-Recovery); 0x07BD=...; else d676. */
static void pd_ctrl_reject(void) {
  uint8_t s;
  uart_puts("[Reject]");
  s = PR(0x07BD);
  if (s == 3) {
    pd_e933_clear_cc81();
    if (PR(0x07B8) == 0) {
      /* 9687 reset path then e81b tail */
    }
    pd_arm_cc_timer(0, 0);   /* stock e81b tail (params from R6/R7 context) */
    pd_ctrl_goodcrc();
    return;
  }
  if (s == 0x0E) {
    /* Data_Reset pending -> Type-C Error Recovery (cc_state_full_reset @0xD676). Out of this
     * milestone's scope; stock halts in SJMP $ inside D676 while HW re-enumerates. */
    /* TODO next milestone: cc_state_full_reset() */
    PR(0x07BD) = 0x0E;
    pd_ctrl_goodcrc();
    return;
  }
  pd_ctrl_goodcrc();
}

/* Wait @0x860E: require substate 0x07BD==3. e933. If contract 0x07B8!=0: arm CC timer
 * e81b(0xD0,0x07) then wait for CC81 attach event, ack CC81=2, send-tail. Else reset (9684). */
static void pd_ctrl_wait(void) {
  if (PR(0x07BD) != 3) { pd_ctrl_goodcrc(); return; }   /* XRL #3 != 0 -> 0x8717 NAK */
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
 * 0x871a: build + send a GoodCRC/NAK response. If SOP(0x07CA)==1: pd_tx_set_sop_header(... ,4)
 *   (control msg, 4? type bits) via dd12; else 9664/965d. Then 95e4 (0x07C4=2 length),
 *   pd_tx_commit_engine, 96bf (advance MsgID). We reproduce the common effect: stage a 2-byte
 *   control response and commit. */
static void pd_rx_nak_send(void) {
  if (PR(0x07CA) == 1) {
    /* control SOP path: header with 0 data objects. msgtype here is approximate (off the
     * contract path; TODO next milestone: exact 871a control-reply type). */
    pd_tx_set_sop_header(0, 1);
  } else {
    /* 9664/965d: header byte0/E405 setup for non-SOP control reply (faithful: clear E405 low) */
    REG_CMD_CFG_E405 &= 0xF8;
  }
  PR(0x07C4) = 2;                    /* 95e4: control message = 2 header bytes, no data */
  pd_tx_commit_engine();
  /* 96bf: advance expected RX MessageID slot */
  PR(0x07C1) = (uint8_t)((PR(0x07C1) + 1) & (uint8_t)(PR(0x07D5) - 1));
}

/* ---------- Soft_Reset (control MsgType 0x0D) ----------
 * The stock jump table (0x850A) has NO key for 0x0D, so stock falls to the 0x871A default — it
 * never needs a real Soft_Reset handler because its very first Request is Accepted (its TX
 * MessageID matches the host's expectation). The handmade fw boots into a host that is mid-PD
 * (Source_Cap arrives with MessageID=3) and across our reset-loop the host's expected device
 * MessageID has drifted from our reset-to-0 counter, so the host rejects our Request with a
 * Soft_Reset. Per USB-PD §6.8.1 the correct response is: reset BOTH MessageID counters for this
 * SOP, then reply Accept; the host then restarts the contract from a clean MessageID=0.
 *
 * Built from the exact stock Accept primitive 0x95da (= clear E420-E43F; pd_tx_set_sop_header(0,3)
 * MessageType 3 = Accept; 0x07C4=2) + pd_tx_commit_engine (0xE1C6, sends + bumps 0x07C3). The
 * GoodCRC for the inbound Soft_Reset is auto-sent by the PD HW engine. */
static void pd_ctrl_soft_reset(void) {
  uart_puts("[Soft_Reset]");
  /* §6.8.1: reset the MessageIDCounter (TX, 0x07C3) and the expected RX slot/MessageID (0x07C1)
   * BEFORE building the Accept so it goes out with MessageID=0. */
  PR(0x07C3) = 0;                 /* TX MessageID counter -> 0 */
  PR(0x07C1) = 0;                 /* RX slot / expected MessageID -> 0 */
  PR(0x07B7) = 0;                 /* no pending staged TX */
  pd_e933_clear_cc81();           /* e933: clear the CC attach event before replying */

  /* 0x95da: build the Accept control message (header MsgType 3, 0 data objects, len 2). */
  pd_tx_buf_clear();              /* e73a */
  pd_tx_set_sop_header(0, 3);     /* dd12: 0 data objects, MessageType 3 = Accept */
  PR(0x07C4) = 2;                 /* 95af-style: control length = 2 header bytes */

  /* §6.8.1: the Accept response to Soft_Reset MUST carry wire MessageID=0. pd_tx_set_sop_header
   * ORs 0x07CA (SpecRev=2, bit1 set) into E421, and E421 bits 3:1 ARE the wire MessageID (proven
   * on HW: clearing them stops the host escalating Soft_Reset -> Hard_Reset). Force MsgID bits 0. */
  REG_CMD_MODE_E421 &= 0xF1;

  pd_tx_commit_engine();          /* e1c6: transmit Accept with MessageID=0 (bumps 0x07C3 0->1) */

  /* The Accept consumed MessageID 0; leave 0x07C3=1 so the device's next message (the Request
   * after the host re-sends Source_Cap) carries MessageID 1, per the restarted AtomicMessage
   * Sequence. (Empirically: forcing that Request back to MessageID 0 made the host escalate to
   * Hard_Reset, confirming the host expects MessageID 1 there.) */

  /* Return the policy engine to the ready-to-negotiate substate so the host's re-sent Source_Cap
   * is accepted (0x07BD in {1,5,3,6}). 1 == init/ready (set by pd_internal_state_init). */
  PR(0x07BD) = 1;
}

/* CONTROL dispatch — clean switch over the 0x850A jump table. */
static void pd_dispatch_control(uint8_t msgtype) {
  switch (msgtype) {
    case 0x01: pd_ctrl_goodcrc();    break;   /* GoodCRC    */
    case 0x03: pd_ctrl_accept();     break;   /* Accept     */
    case 0x04: pd_ctrl_reject();     break;   /* Reject     */
    case 0x06: pd_ctrl_ps_rdy();     break;   /* PS_RDY     */
    case 0x0C: pd_ctrl_wait();       break;   /* Wait       */
    case 0x0D: pd_ctrl_soft_reset(); break;   /* Soft_Reset */
    /* 0x02 Reserved, 0x05 Ping, 0x07 Get_Source_Cap, 0x08 Get_Sink_Cap, 0x0A..0x0B PR/DR_Swap,
     * 0x0E Soft_Reset, 0x0F..0x10 — stock NAKs/handles via @0x8706/869x; out of milestone scope */
    default:   pd_rx_nak_send();  break;   /* TODO next milestone: full control set */
  }
}

/* ====================================================================================
 * DATA-message handlers (NumObj>0). Out-of-scope ones (BIST/Enter_USB/VDM) are stubbed with a
 * print + NAK and a TODO; the Source_Cap path is fully implemented.
 * ==================================================================================== */

/* 874c(lo,hi): print CODE string then return SOP (0x07CA). Source_Cap prints "[Source_Cap]". */

/* DATA dispatch */
static void pd_dispatch_data(uint8_t msgtype) {
  if (msgtype == 0x01) {
    /* Source_Cap. Stock: print "[Source_Cap]"; ret=SOP; if SOP in {2,3} (cable) seed E420=0x80
     * via 9713(0x80). Require substate 0x07BD in {1,5,3,6} else thunk_dc2d()+return. */
    uint8_t sop;
    uart_puts("[Source_Cap]");
    sop = PR(0x07CA);
    if (sop == 2 || sop == 3) {
      /* stock 0x84b3: A=0x80; 9713(0x80) writes E420=0x80 and returns (E409 & 0xF1);
       * then 0x84b8 A|=4; MOVX @DPTR,A -> E409 = (E409 & 0xF1) | 4. */
      REG_CMD_TRIGGER = 0x80;             /* 9713(0x80): seed TX control byte */
      REG_CMD_CTRL_E409 = (REG_CMD_CTRL_E409 & 0xF1) | 0x04;
    }
    {
      uint8_t s = PR(0x07BD);
      if (s != 1 && s != 5 && s != 3 && s != 6) {
        /* thunk_dc2d: ignore Source_Cap (wrong substate) — stock just returns. */
        return;
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
    /* BIST data message. TODO next milestone: dc65() BIST handler. */
    /* dc65 sets 0x07C8=1 (BIST mode) when RX[1]&0xF0==0x80. */
    pd_ctrl_goodcrc();
    return;
  }
  if (msgtype >= 2 && msgtype < 8) {
    /* 0x02 (Request — we are sink, shouldn't receive), 0x04..0x07: NAK. Stock: 962e + 871a. */
    pd_rx_nak_send();
    return;
  }
  if (msgtype == 0x08) {
    /* Enter_USB Data Message (PD3.1). Stock @0x84E8: print "[Enter_USB]"; if state byte==5 ->
     * pd_handle_enter_usb @0xA036; 870b (GoodCRC); else NAK. pd_handle_enter_usb sends Accept and
     * latches USB4 mode entry (vdm.h). We accept regardless of the substate gate (the handmade
     * boot does not drive the 0x07BD==5 sub-state machine the same way; the host's Enter_USB is
     * always answered to advance into mode entry). */
    uart_puts("[Enter_USB]");
    pd_handle_enter_usb();               /* a036 -> usb4_connect_decide -> Accept + mode-entry */
    pd_ctrl_goodcrc();                   /* 870b tail */
    return;
  }
  if (msgtype == 0x0F) {
    /* Structured VDM (stock @0x848E): print "[VDM]"; vdm_tx_dispatch @0x9AC4 (builds + sends the
     * ACK/NAK via the CC88 strobe + pd_tx_commit_engine); 870b (GoodCRC) tail. */
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

  /* GoodCRC / extended-message-id retry gate (957c/95ec).
   * ext_bit = bit7 of RX[base+1] (the Extended header bit). The retry sub-path runs only when
   * 0x07C8 (BIST/chunk mode) is clear AND ext_bit set AND SOP(param_3)==0; Source_Cap has
   * ext_bit==0 so it falls straight through to the NumObj dispatch below. */
  ext_bit = (uint8_t)((PR(base + 1) >> 7) & 1);
  if (PR(0x07C8) != 0) return;               /* 0x07C8 set (BIST/chunk) -> stock LJMP 0x8739 ret */

  if (ext_bit != 0 && sop == 0) {
    /* Extended/chunked message-id retry path (956a/95eb/96b7/9703/95b6). Out of milestone
     * scope — Source_Cap/Request never take it. TODO next milestone: chunked extended msgs. */
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
