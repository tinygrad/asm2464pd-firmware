#ifndef PD_H
#define PD_H
/*
 * USB-PD / Type-C CC-controller keystone driver.
 *
 * Faithful reimplementation of the ASM2464PD stock firmware (fw_tinygrad.bin) PD attach +
 * power-contract bring-up, reverse-engineered from the Ghidra decompilation. See
 * handmade/USB4_RE.md for the full phase reference. Every function cites the stock body
 * offset it mirrors (Ghidra addr == fw_tinygrad body offset).
 *
 * Goal of this first milestone: get the TB4 host to engage USB-PD with the handmade fw —
 * i.e. make the PD-interrupt source (C80A bit6) fire and the host send a Source_Cap. Basic
 * PD power negotiation runs over the Type-C CC lines and is independent of USB4 data mode,
 * so this path is exercised before any PHY/VDM/tunnel work. Success = "[PD_int:" on UART.
 *
 * Additive to the USB3 firmware: touches only the PD engine (E4xx), CC controller (CCxx),
 * the interrupt-enable group (C800/C801/CA60/C809) and PD state RAM (0x07xx) — none of which
 * the USB3 device path uses.
 */
#include "types.h"
#include "registers.h"

/* Volatile MMIO accessor (polling loops must not be optimised away). */
#define PR(a) XDATA_REG8V(a)

/* Set once the host actually engages PD (a PD message is received). */
static volatile uint8_t pd_seen = 0;

/* PD message dispatcher (defined in pd_dispatch.h, #included after pd.h). */
static void pd_rx_message_dispatch(void);
/* Set if any CC-controller command wait timed out (the CC engine isn't responding — most
 * likely the boot-time Type-C/CC PHY setup is still missing). Surfaced on the status line. */
static volatile uint8_t pd_cc_timeout = 0;

#define PD_WAIT_LIMIT 0x4000u
/* Bounded poll: spin until (PR(reg) & mask) matches `set` (1=wait-for-set, 0=wait-for-clear),
 * giving up after PD_WAIT_LIMIT iterations and flagging the timeout. */
static void pd_wait(uint16_t reg, uint8_t mask, uint8_t set) {
  uint16_t g = 0;
  if (set) { while (!(PR(reg) & mask) && ++g < PD_WAIT_LIMIT); }
  else     { while ( (PR(reg) & mask) && ++g < PD_WAIT_LIMIT); }
  if (g >= PD_WAIT_LIMIT) pd_cc_timeout = 1;
}

/* ---- PD command/RX engine (0xE4xx) ---- */
/* E400 engine ctrl: bit7=enable, (&0xC3)|0x3C=CC drive/termination, bit6=CC orientation     */
/* E402 status: bits1-3 busy, (&0x1F)|0x20=start TX                                           */
/* E403/E404/E405 TX command descriptor                                                       */
/* E409 mode; E40A=0x0F; E40B sub-block enables (bits1-3 = RX/TX enable mask 0x0E)            */
/* E40D=0x28; E40E=0x8A; E411=0xA1/E412=0x79 Rp/Rd CC comparator thresholds (REQUIRED)        */
/* E413 RX/TX enable (&0x8F)|0x60; E40F RX-event status (bit0=msg, W1C); E410 RX-event 2      */
/* E41C TX trigger/busy (bit0); E420.. PD TX message buffer (0xE420-0xE43F)                   */
/* E401/E406/E407/E408 RDO/CRC timing constants (set by pd_da51)                              */

/* ---- CC controller command interface (0xCCxx) ---- */
/* CC88 low3=opcode (2=hard-reset-TX); CC89 bit0=go / bit1=done / write2=ack; CC8A param;     */
/* CC8B value/timeout. CC80/CC98 event-enable; CC81 attach-edge / CC99 role event (bit1, W1C) */

/* ---- PD state RAM (stock globals; free in handmade — SDCC owns only 0x0000-0x000F) ---- */
/* 0x07BC PD role/contract state; 0x07BD PD msg substate (1=init); 0x07D5 from E400.6;        */
/* 0x07DA..0x07E2 PD timers; 0x07DF hard-reset-done flag                                       */

/* ---- interrupt routing / global mode ---- */
/* C800 sys int status/enable (bit0,bit2); C801 int enable (bit4); CA60 clk/pwr int           */
/* C809 PD interrupt enable (bit5 = 0x20, REQUIRED); 0x09F9 USB4 mode flag (1=USB4)           */

/* da51: RDO/CRC timing constants for the PD engine. */
static void pd_da51(void) {
  PR(0xE40B) = (PR(0xE40B) & 0x7F) | 0x80;
  if (PR(0xE40B) & 0x80) {
    PR(0xE401) = (PR(0xE401) & 0xF8) | 0x04;
    PR(0xE401) = (PR(0xE401) & 0x07) | 0xB0;   /* -> 0xB4 */
    PR(0xE406) = (PR(0xE406) & 0xF0) | 0x06;
    PR(0xE406) = (PR(0xE406) & 0x0F) | 0xA0;   /* -> 0xA6 */
    PR(0xE407) = (PR(0xE407) & 0xE0) | 0x15;
    PR(0xE408) = (PR(0xE408) & 0xE0) | 0x1C;
  }
}

/* cc_pd_phy_term_init @0xAE87 — program CC Rp/Rd termination + arm the PD command/RX engine.
 * This is what makes the host see a PD-capable Type-C sink attach and lets the device RX. */
static void cc_pd_phy_term_init(void) {
  PR(0xE40B) = (PR(0xE40B) & 0xBF) | 0x40;
  PR(0xE40A) = 0x0F;
  PR(0xE413) &= 0xFE;
  PR(0xE413) &= 0xFD;
  PR(0xE400) &= 0x7F;                         /* engine off during reconfig */
  /* CC cmd: opcode 0, CC8A=0, CC8B=0x0A, go/wait/ack */
  PR(0xCC88) &= 0xF8; PR(0xCC8A) = 0;
  PR(0xCC8B) = 0x0A; PR(0xCC89) = 0x01;
  pd_wait(0xCC89, 0x02, 1);
  PR(0xCC89) = 0x02;
  PR(0xE40B) = (PR(0xE40B) & 0xFE) | 0x01;
  /* CC cmd: opcode 0, CC8A=0, CC8B=0x3C, go/wait/ack */
  PR(0xCC88) &= 0xF8; PR(0xCC8A) = 0;
  PR(0xCC8B) = 0x3C; PR(0xCC89) = 0x01;
  pd_wait(0xCC89, 0x02, 1);
  PR(0xCC89) = 0x02;
  pd_wait(0xE402, 0x08, 0);                   /* wait engine idle */
  PR(0xE409) &= 0xFE;
  PR(0xE409) = (PR(0xE409) & 0xBF) | 0x40;
  PR(0xE420) = 0x40;                          /* 9713(0x40): seed TX buf[0] */
  PR(0xE409) = (PR(0xE409) & 0xF1) | 0x06;
  PR(0xE400) = (PR(0xE400) & 0xBF) | 0x40;    /* CC orientation bit6 */
  PR(0xE411) = 0xA1;                          /* Rp/Rd comparator thresholds (REQUIRED) */
  PR(0xE412) = 0x79;
  PR(0xE400) = (PR(0xE400) & 0xC3) | 0x3C;    /* enable CC drive/termination (host sees attach) */
  PR(0xE409) &= 0x7F;
  PR(0xC809) = (PR(0xC809) & 0xDF) | 0x20;    /* enable PD interrupt source (REQUIRED) */
  pd_da51();
  PR(0xE40E) = 0x8A;
  PR(0xE400) = (PR(0xE400) & 0x7F) | 0x80;    /* PD engine enable (REQUIRED) */
  PR(0xE40B) &= 0xFE;
  PR(0xE66A) &= 0xEF;
  PR(0xE40D) = 0x28;
  PR(0xE413) = (PR(0xE413) & 0x8F) | 0x60;    /* RX+TX enable (REQUIRED) */
  PR(0xCAC4) &= 0xFE;
  PR(0xE40B) &= 0xDF;
  PR(0xC698) &= 0xDF;
}

/* cc_ctrl_enable_cc81_cc98_events @0xE330 — clear + enable the CC attach/role event sources. */
static void cc_ctrl_enable_events(void) {
  PR(0xCC81) = 0x04; PR(0xCC81) = 0x02;
  PR(0xC801) = (PR(0xC801) & 0xEF) | 0x10;
  PR(0xCC80) &= 0xEF;
  PR(0xCC80) = (PR(0xCC80) & 0xF8) | 0x03;
  PR(0xCC99) = 0x04; PR(0xCC99) = 0x02;
  PR(0xC801) = (PR(0xC801) & 0xEF) | 0x10;
  PR(0xCC98) &= 0xEF;
  PR(0xCC98) = (PR(0xCC98) & 0xF8) | 0x04;
}

/* pd_internal_state_init / InternalPD_StateInit @0xB8C3 — reset the PD policy-engine state
 * block, set substate=init (0x07BD=1), seed timers, enable CC events. */
static void pd_internal_state_init(void) {
  uart_puts("[InternalPD_StateInit]");
  PR(0x07B7) = 0; PR(0x07B8) = 0;
  PR(0x07C3) = 0; PR(0x07C4) = 0;
  PR(0x07C7) = 0;
  PR(0x07C5) = 0;
  PR(0x07C2) = 0;
  PR(0x07C1) = 0;
  PR(0x07E3) = 0;
  PR(0x07BD) = 1;
  PR(0x07D5) = (PR(0xE400) & 0x40) ? 0x10 : 0x01;
  if (PR(0x07DE) == 0) PR(0x07CA) = 2;
  PR(0x07DE) = 0; PR(0x07DF) = 0;
  PR(0x07B9) = 0; PR(0x07BA) = 0;
  PR(0x07CC) = 0;
  PR(0x07CB) = 0;
  PR(0x07CD) = 0; PR(0x07CE) = 0; PR(0x07CF) = 0;
  PR(0x07BB) = 0;
  PR(0x07C8) = 0;
  PR(0x07BE) = 0;
  PR(0x07BC) = 0;
  cc_ctrl_enable_events();                    /* e330 */
  PR(0x07E0) = 5; PR(0x07E1) = 0; PR(0x07E2) = 0;
  PR(0x07DA) = 1; PR(0x07DB) = 0x2C; PR(0x07DC) = 0; PR(0x07DD) = 0x64;
}

/* pd_drive_hard_reset / Drive_HardRst @0xBE8B — transmit a USB-PD HARD RESET ordered set to
 * force the host to drop its PD contract and re-send Source_Cap. No-op (print only) once USB4
 * is already established ((E302>>4)&3 == 3). */
static void pd_drive_hard_reset(void) {
  uint8_t lm = (PR(0xE302) & 0x30) >> 4;
  uint16_t g;
  uart_puts("[CC_state=");
  uart_puthex(lm);
  if (lm == 3) {
    uart_puts("][CCOpen_neednt_HardRst]");
    return;
  }
  uart_puts("][Drive_HardRst]");
  { uint8_t i; for (i = 0; i < 0x20; i++) PR(0xE420 + i) = 0; }   /* e73a: clear TX buf */
  pd_internal_state_init();                                       /* b8c3 */
  /* 9536: arm CC-controller HARD-RESET-TX opcode */
  PR(0xE40F) = 0xFF; PR(0xE410) = 0xFF;
  PR(0xE40B) &= ~0x0E;
  PR(0xCC88) = (PR(0xCC88) & 0xF8) | 0x02;
  PR(0xCC8A) = 0; PR(0xCC8B) = 0xC7; PR(0xCC89) = 0x01;
  pd_wait(0xCC89, 0x02, 1);
  /* 9584: ack + re-enable PD events */
  PR(0xCC89) = 0x02;
  PR(0xE40B) |= 0x0E;
  /* build + start the PD TX command */
  PR(0xE403) = 0x00; PR(0xE404) = 0x40;
  PR(0xE405) = (PR(0xE405) & 0xF8) | 0x05;
  PR(0xE402) = (PR(0xE402) & 0x1F) | 0x20;
  /* e09a: wait engine idle (bounded) */
  for (g = 0; ((PR(0xE402) & 0x0E) || (PR(0xE41C) & 0x01)) && g < 0x4000; g++);
  /* 9605: trigger TX */
  PR(0xE41C) |= 0x01;
  /* wait TX accepted (HW clears E41C bit0), bounded */
  for (g = 0; (PR(0xE41C) & 0x01) && g < 0x4000; g++);
  PR(0x07DF) = 1;
}

/* init_sys_flags @0x4BE6 (subset) — route the C806/C80A PD/system interrupt aggregate to the
 * 8051 EX1 line so INT1 actually fires. Without this the PD-RX ISR never runs. */
static void pd_int1_enable_group(void) {
  PR(0xC801) = (PR(0xC801) & 0xEF) | 0x10;
  PR(0xC800) = (PR(0xC800) & 0xFB) | 0x04;
  PR(0xCA60) = (PR(0xCA60) & 0xF8) | 0x06;
  PR(0xCA60) = (PR(0xCA60) & 0xF7) | 0x08;
  PR(0xC800) |= 0x01;
}

/* Top-level keystone bring-up. Mirrors pd_cc_attach_term_setup @0xBAA0 (minus the boot
 * mode-decision, which we short-circuit by forcing the USB4 mode flag for now). */
static void pd_keystone_init(void) {
  pd_int1_enable_group();
  /* 0x09F9 runtime mode flag. bit7 = VDM-ACK enable (Discover and EnterMode ACK gates); bits1:0
   * = route -> 0x09FA. 0x87 = VDM-ACK + route 3 (USB4 tunnel): (0x09FA&0x81)!=0 selects the
   * tunnel route and 0x09FA.1 fires SB[0xD8]=2 + E716 reflip in usb4_connect_u4. (Was 0x01, which
   * clears bit7 -> all VDM NAK and mis-selects route 1.) See USB4_TUNNEL_PLAN.md sec 4. */
  PR(0x09F9) = 0x87;           /* VDM-ACK + USB4 tunnel route (TODO: real mode-decision @0xB1CB) */
  cc_pd_phy_term_init();
  pd_internal_state_init();
}

/* pd_rx_isr — the C80A.6 PD-interrupt handler body @0xAF5E. A priority demux over the PD RX
 * event registers E40F/E410: EACH event bit must be W1C-acked or the interrupt re-fires
 * forever. E40F.7=Soft_Rst_Int, E40F.0=message received, E40F.5=Hard_Rst_Int; E410.x are
 * secondary PD engine events. Ends by servicing the E314 PHY-completion bit.
 * (Full PD message decode + power contract — the 0x83d6 dispatcher — is the next milestone.) */
static void pd_rx_isr(void) {
  uint8_t e40f = PR(0xE40F);
  uart_puts("\n[PD_int:");
  uart_puthex(e40f);
  uart_putc(':');
  uart_puthex(PR(0xE410));
  uart_putc(']');
  if (e40f & 0x80) {                 /* Soft_Rst_Int (dfdc) */
    uart_puts("[Soft_Rst_Int]");
    PR(0xE40F) = 0x80;
  } else if (e40f & 0x01) {          /* message received */
    PR(0xE40F) = 0x01;
    pd_seen = 1;                     /* host is doing PD — the keystone is cracked */
    pd_rx_message_dispatch();        /* @0x83d6 — Source_Cap -> Request -> power contract */
  } else if (e40f & 0x20) {          /* Hard_Rst_Int (e419) */
    PR(0xE40F) = 0x20;
    uart_puts("[Hard_Rst_Int]");
  } else {
    uint8_t e410 = PR(0xE410);
    if      (e410 & 0x01) PR(0xE410) = 0x01;
    else if (e410 & 0x08) PR(0xE410) = 0x08;
    else if (e410 & 0x10) PR(0xE410) = 0x10;
    else if (e410 & 0x20) PR(0xE410) = 0x20;   /* e876 */
    else if (e410 & 0x40) PR(0xE410) = 0x40;   /* e439 */
    else if (e410 & 0x80) PR(0xE410) = 0x80;
  }
  if (PR(0xE314) & 0x01) PR(0xE314) = 0x01;    /* E314 PHY-completion W1C */
}

/* ====================================================================================
 * cc_pd_timer_tick @0xB4BA — the C806.0 INT1 timer-tick = PD/USB4 policy-engine tick.
 * Forward declarations for usb4_mode_entry_commit (vdm.h, #included AFTER pd.h) and the
 * deeper PD-state helpers. usb4_mode_entry_commit is the USB4 mode-entry latch the host
 * waits on (CC91.1 -> it). pd_drive_hard_reset is defined above.
 * ==================================================================================== */
static uint8_t usb4_mode_entry_commit(void);          /* vdm.h @0xD78A; returns mode in R7 */

/* cc_cc23_reinit_event @0xE3D8 — CC23.1 re-init / SB-reconnect event. Stock body chains into
 * banked helpers (e3b7 if 0x0B41, 3578(0x0AEE), d810 link re-prep) then 0x07E8=0, 0x0B2F=1.
 * The 3578/d810 chain is a large banked re-prep path (CM-owned); transcribe the two verified
 * XDATA seeds and NOTE the banked chain. W1C of CC23 is done by the caller. */
static void cc_cc23_reinit_event(void) {
  /* NOTE: stock also calls 3578(0x0AEE)+d810 (banked link re-prep) before these writes; not
   * transcribed (deep/banked, CM-owned). The verified tail state-seeds are kept. */
  PR(0x07E8) = 0x00;                                  /* e3f2 */
  PR(0x0B2F) = 0x01;                                  /* e3f7 */
}

/* cc_state_full_reset @0xD676 — Type-C error-recovery. Stock body is a single banked uart_puts
 * (R3:R2:R1=0xFF:0x23:0x4B -> string "[Error_Recovery]" via the r3_read_dispatch print routine
 * @0x538d->0x0bc8); it is a diagnostic print, NOT a HW register op. */
static void cc_state_full_reset(void) {
  uart_puts("[Error_Recovery]\n");                    /* d676: 538d print of 0x234B string */
}

/* pd_cc81_hard_reset_4 @0xE90B — CC81 |= 4 then pd_drive_hard_reset (LJMP 0xBE8B). Verbatim. */
static void pd_cc81_hard_reset_4(void) {
  PR(0xCC81) = 0x04;                                  /* e910 (MOV #4, not RMW per disasm) */
  pd_drive_hard_reset();                              /* e911 LJMP 0xBE8B */
}

/* pd_queue_ctrl_msg @0xE529 — enqueue a PD control message (code in `code`). Stock: 0x0AA3=code;
 * dd42(0); if e6d2() (mailbox build/send, banked: e396/0dc5/d17a) returns nonzero then
 * 0x7000=0x0AA3 + e478() (banked send: 0d84/b851/b104). The e6d2/e478 mailbox path is deep and
 * banked (CM/PD TX engine); transcribe the two verified XDATA writes (0x0AA3 record + dd42(0))
 * and NOTE the banked send. The timer-tick's MUST-BE-CORRECT branches (CC91 commit, CC81
 * hard/full reset) do not depend on this deep path; e529 is reached only when 0x07BC!=0. */
static void pd_queue_ctrl_msg(uint8_t code) {
  PR(0x0AA3) = code;                                  /* e52d: record ctrl-msg code */
  PR(0xE7E3) = 0x00;                                  /* e530: dd42(0) with param 0 -> E7E3=0 */
  /* NOTE: stock e533 e6d2()/e541 e478() PD-TX mailbox send not transcribed (deep banked). */
}

/* cc_cc99_default_event @0xE883 — CC99 default branch. Stock: e73a (clear PD TX buf), then
 * 95e1(R5=0,R7=0x10) + LJMP e1c6 (PD-TX engine drain). Both are banked PD-TX helpers; the
 * channel's required side effect for the timer-tick is the CC99 W1C ack (done by the caller).
 * NOTE the banked TX drain. */
static void cc_cc99_default_event(void) {
  /* NOTE: stock e73a (TX buf clear) + 95e1/e1c6 (banked PD-TX drain) not transcribed. The CC99
   * W1C ack is performed by the caller so the source can't storm. */
}

/* cc_ccf9_subdemux @0xDF79 — CCF9.1 sub-demux on 0x0A9D (copied from 0x0B1B). Stock: e74e then
 * dispatch on 0x0A9D: ==1 -> e374; ==2 && 0x07FF==0x69 -> e545; ==3 -> 0x055c. All banked CM
 * sub-handlers. Transcribe the verified 0x0A9D<-0x0B1B copy and NOTE the banked dispatch. */
static void cc_ccf9_subdemux(void) {
  PR(0x0A9D) = PR(0x0B1B);                             /* df7c-df80 */
  /* NOTE: stock e74e + (0x0A9D==1->e374 / ==2&&0x07FF==0x69->e545 / ==3->055c) banked CM
   * sub-handlers not transcribed. */
}

/* cc_pd_timer_tick @0xB4BA — INT1 C806.0 timer-tick = PD/USB4 policy-engine tick. Services 6
 * Type-C/CC controller per-channel event regs (CC23/CC81/CC91/CC99/CCD9/CCF9), bit1=event,
 * W1C by writing 2. CC91.1 -> usb4_mode_entry_commit (USB4 mode-entry latch the host waits on);
 * CC81.1 -> hard/full reset re-elicitation of Source_Cap. Verbatim from stock disasm @0xB4BA. */
/* Instrumentation (decision-rule diagnostic): tick_seen counts C806.0 timer-tick entries;
 * cc_hit is a bitmask of which CC channels ever showed bit1 (1=CC23,2=CC81,4=CC91,8=CC99,
 * 0x10=CCD9,0x20=CCF9). Surfaced on the [U ...] status line. */
static volatile uint8_t tick_seen = 0;
static volatile uint8_t cc_hit = 0;

static void cc_pd_timer_tick(void) {
  tick_seen++;
  if (PR(0xCC23) & 0x02) cc_hit |= 0x01;
  if (PR(0xCC81) & 0x02) cc_hit |= 0x02;
  if (PR(0xCC91) & 0x02) cc_hit |= 0x04;
  if (PR(0xCC99) & 0x02) cc_hit |= 0x08;
  if (PR(0xCCD9) & 0x02) cc_hit |= 0x10;
  if (PR(0xCCF9) & 0x02) cc_hit |= 0x20;
  if (PR(0xCC23) & 0x02) {                 /* CC23.1: re-init / SB-reconnect */
    cc_cc23_reinit_event();                /* 0xE3D8 */
    PR(0xCC23) = 0x02;
  }
  if (PR(0xCC81) & 0x02) {                 /* CC81.1: CC attach/detach, branch on substate 0x07BD */
    uint8_t sub = PR(0x07BD);
    if (sub == 0x0E || sub == 0x0D) {      /* Data_Reset / Enter_USB pending */
      PR(0xCC81) = 0x02;
      if (PR(0x07BC) != 0) pd_queue_ctrl_msg(0x3B);   /* 0xE529 queue Data_Reset */
      cc_state_full_reset();               /* 0xD676 Type-C error recovery / full reset */
    } else {
      pd_cc81_hard_reset_4();              /* 0xE90B: CC81|=4 then pd_drive_hard_reset() */
      PR(0xCC81) = 0x02;
    }
  }
  if (PR(0xCC91) & 0x02) {                 /* CC91.1: 1s sender-response timeout -> COMMIT USB4 mode */
    PR(0xCC91) = 0x02;
    uart_puts("[1 sec time out]\n");       /* stock string @0x53F8 */
    PR(0x07BB) = 0x01;
    PR(0x09FA) = 0x04;
    PR(0x0AE2) = usb4_mode_entry_commit(); /* 0xD78A returns mode in R7 */
  }
  if (PR(0xCC99) & 0x02) {                 /* CC99.1: role-dependent reset */
    uint8_t role = PR(0x07BC);
    if (role == 0x02) { pd_queue_ctrl_msg(0x3C); pd_drive_hard_reset(); }   /* 0xBE8B */
    else if (role == 0x03) { pd_queue_ctrl_msg(0xFF); }
    else { cc_cc99_default_event(); PR(0xCC99) = 0x02; }                    /* 0xE883 */
  }
  if (PR(0xCCD9) & 0x02) {                 /* CCD9.1 */
    PR(0xCCD9) = 0x02;
    PR(0x0719) = 0x02;                     /* disasm: 0x0719 = A (=2) */
  }
  if (PR(0xCCF9) & 0x02) {                 /* CCF9.1 */
    PR(0xCCF9) = 0x02;
    cc_ccf9_subdemux();                    /* 0xDF79 */
  }
}

#endif /* PD_H */
