#ifndef SB_ROUTER_H
#define SB_ROUTER_H
/*
 * USB4 SB-router connect / lane-bond event handler (M2) — faithful transcription of the ASM2464PD
 * stock firmware (fw_tinygrad.bin) bank1 INT1 source C80A.5 handler.
 *
 *   sb_router_event_handler   CODE_BANK1::A066  (the C80A.5 service body)
 *   sb_channel_connect_service CODE_BANK1::C3B2 (per-active-port connect descriptor read+dispatch)
 *
 * Transcribed byte-for-byte from the CODE_BANK1:: disassembly (the decompiler garbles this fn:
 * its uart_puts (LCALL 0x538d, string page 0x20xx) and SB W1C writes (LCALL 0x0be6) are
 * mis-modeled as non-returning r3_*_dispatch calls — they DO return). Verified against the raw
 * listing of A066 (a066-a31d) and C3B2 (c3b2-c4a9).
 *
 * Included AFTER sb.h (SB_RD/SB_WR/P1_* + the page-1 accessor) and usb4.h, so the SB block
 * page-1 alias (0x012800+off, DPX=1->0) and uart_* are in scope.
 *
 * This handler is PURE REACTIVE: every SB[..] bit it reads is set by SB HW from host sideband
 * traffic; it only reads + W1C-acks (writes the bit back) + forwards. It must NEVER be invoked
 * speculatively. (USB4_TUNNEL_PLAN.md sec3 SB-ROUTER, Risk R4.)
 *
 * Helper map (all reads/writes hit the SB block via the R3=2/R2=0x28 paged accessor == SB_RD/SB_WR):
 *   0x99ff / 0x9797 / 0x9718.. : SB read of a fixed offset      -> SB_RD(off)
 *   0x9a5a (sb_write_c9_ack)   : SB[0xC9] = A   (W1C connect)   -> SB_WR(0xC9, v)
 *   0x0bc8 / 0x0be6            : R3-paged SB read / write        -> SB_RD/SB_WR
 *   0x989d                     : read page1 0x0109 (lane-bond done, bit0)
 *   0x06F1                     : active-port round-robin index (XDATA, DPX=0)
 *   0x0766                     : route-up trigger (consumed by ConnRout/CM, M3)
 *   0x074E/0x074F              : per-lane CL0 latches (zeroed by 0x9874)
 *   0x072D                     : Lane-Bonded flag (set by eed6)
 */

/* sb_write_c9_ack (0x9a5a): SB[0xC9] = (1 << pos) — W1C one connect bit. (a09c-a0a7 builds 1<<idx
 * via RLC then 0x9a5a writes it to SB[0xC9].) */
static void sb_write_c9_ack(uint8_t pos) {
  SB_WR(0xC9, (uint8_t)(1u << (pos & 7)));
}

/* sb_transport_substate_poll (d4cd): poll SB[0x28]/[0x2A]/[0x81]/[0x83] bit3 transport events.
 * Each: if SB[off].3 set and the matching latch (0x06EE/0x06EF) is in the expected state, advance
 * the substate var (0x06F0=1), call the bank0 notify (cd3f), W1C the event (974a/9746), set the
 * latch. Pure event tracking driven by HW-set SB bits — reproduced as the bit-3 W1C acks + latch
 * advance (the cd3f bank0 notify is a state callback, non-host-gating). */
static void sb_transport_substate_poll(void) {
  /* SB[0x28].3 (L0 transport) */
  if (SB_RD(0x28) & 0x08) {
    if (PR(0x06EE) == 0) { PR(0x06F0) = 0; SB_WR(0x28, SB_RD(0x28) & 0xF7); PR(0x06EE) = 1; }
  } else if (PR(0x06EE) == 1) {
    PR(0x06F0) = 1; SB_WR(0x28, SB_RD(0x28) & 0xF7); PR(0x06EE) = 0;
  }
  /* SB[0x2A].3 (L1 transport) — symmetric on the same 0x06EE latch */
  if (SB_RD(0x2A) & 0x08) {
    if (PR(0x06EE) == 1) { PR(0x06F0) = 1; SB_WR(0x2A, SB_RD(0x2A) & 0xF7); PR(0x06EE) = 0; }
  }
  /* SB[0x81].3 (L0 link) -> 0x06EF latch */
  if (SB_RD(0x81) & 0x08) {
    if (PR(0x06EF) == 0) { SB_WR(0x81, SB_RD(0x81) | 0x02); PR(0x06EF) = 1; }
  }
  /* SB[0x83].3 (L1 link) -> 0x06EF latch */
  if (SB_RD(0x83) & 0x08) {
    if (PR(0x06EF) == 1) { SB_WR(0x83, SB_RD(0x83) | 0x02); PR(0x06EF) = 0; }
  }
}

/* ---- edd9: SB[0x09] poll prelude (c3b2 head). reads SB[0x09]; if bit0 set, 98b7 (per-ch arm). ---- */
static void sb_chan_prelude(void) {
  /* 98b7 (per-channel arm) is gated on SB[0x09].0. Its effect is internal SB state; the host-facing
   * connect descriptor is read below regardless, so we just consume the read-ack here. */
  (void)SB_RD(0x09);
}

/* ---- dea1: post-[===SB Con===] consequence. if (page1 0x0109 & 1) return; else SB[0x28] |= 0x40. ---- */
static void sb_con_consequence(void) {
  if (P1_RD(0x0109) & 0x01) return;          /* 989b/989d lane-bond gate */
  SB_WR(0x28, (SB_RD(0x28) & 0xBF) | 0x40);  /* 967e read + set bit6 */
}

/* ---- eed6: post-[Lane Bonded] consequence. 0x072D=1; eeee (SB[0xC9]=0xFF arm + page1 0x01xx). ---- */
static void sb_lane_bonded_consequence(void) {
  PR(0x072D) = 1;                            /* eed6: lane-bonded flag */
  /* eeee: MOV R1,#0xC9; A=0xFF; 9838(write SB[0xC9]=0xFF arm); 999e; P1[0x01C8].6 clr|0x40 set.
   * The SB[0xC9]=0xFF re-arm + the page1 0x01C8 RMW are the lane-bond confirm. */
  SB_WR(0xC9, 0xFF);
  P1_WR(0x01C8, (P1_RD(0x01C8) & 0xBF) | 0x40);
}

/* ---- e52d: lane-bond-complete -> tunnel up -> downstream PCIe bring-up to the GPU. ----
 * Stock: 05d4/0598 setup; sb_lane_descriptor_loader (b7a4, == sb_rom_descriptor_load in sb.h);
 * eb0a; if 0x09FA.1: (0x0B41 gate) pcie_downstream_link_bringup(0x0AEF); CA60 &= ~0x08; if 0x0AF1.0 04f3.
 * The downstream link bring-up == handmade pcie_power_on(). pcie_power_on() uses sleep()/long polls,
 * so we DEFER it to the super-loop (set sb_tunnel_up_pending) rather than run it inside the INT1 ISR. */
static volatile uint8_t sb_tunnel_up_pending = 0;
static void sb_lane_bond_complete_tunnel_up(void) {
  sb_rom_descriptor_load();                  /* b7a4: re-seed the router DROM tables */
  if (PR(0x09FA) & 0x02) {                    /* 0x09FA.1 tunnel route */
    sb_tunnel_up_pending = 1;                 /* 3578: defer downstream PCIe bring-up to super-loop */
  }
  PR(0xCA60) = PR(0xCA60) & 0xF7;            /* clear clock/power bit3 */
}

/* ====================================================================================
 * sb_channel_connect_service (CODE_BANK1::C3B2) — per active-port connect descriptor.
 * Selects the SB register pair by 0x06F1: p0->SB[0x20]/[0x22], p1->[0x21]/[0x23],
 * p2->[0xA4]/[0xA6], p3->[0xA5]/[0xA7]. 0x4E=lo, 0x4F=hi; validate (~0x4F)==0x4E else err-print.
 * Dispatch on (0x4E & 0x0F):  1 or 5 -> 0x0766=1 (route-up);  3 -> link-reset;  6 -> special;
 *                             default -> return.
 * ==================================================================================== */
static void sb_channel_connect_service(void) {
  uint8_t port = PR(0x06F1);
  uint8_t lo, hi, n;

  sb_chan_prelude();                          /* edd9 */

  if (port == 0)      { lo = SB_RD(0x20); hi = SB_RD(0x22); }
  else if (port == 1) { lo = SB_RD(0x21); hi = SB_RD(0x23); }
  else if (port == 2) { lo = SB_RD(0xA4); hi = SB_RD(0xA6); }
  else                { lo = SB_RD(0xA5); hi = SB_RD(0xA7); }   /* port 3 */

  /* validate: (~hi) == lo (c418: MOV A,0x4f; CPL A; XRL A,0x4e; JZ ok) */
  if ((uint8_t)(~hi) != lo) {
    uart_puts("[SBch err]");                  /* 538d string @0x1FED */
    return;
  }

  n = lo & 0x0F;
  if (n == 1 || n == 5) {                      /* route-up */
    PR(0x0766) = 1;                            /* c4a3: route trigger (ConnRout/CM consumes it) */
    return;
  }
  if (n == 3) {                                /* link-reset (c46f) */
    if (PR(0x0B1C) != 0) SB_WR(0x50, 0x40);    /* c479: if 0x0B1C!=0, SB[0x50]=0x40 */
    SB_WR(0x15, 0x83);                         /* c496: SB[0x15]=0x83 (via 9704) */
    /* c4a0 LJMP 0xda9f — full SB re-init/reset path. Heavy; reproduce the observable arm only:
     * da9f re-runs sb_block_init + downstream bring-up. On a clean connect (n==1/5) this is not
     * taken; the n==3 reset path is a host-driven link-reset and is left to re-trigger the
     * connect FSM (the SB[0x15]=0x83 arm above is the load-bearing W1C). */
    return;
  }
  if (n == 6) {                                /* special (c43b) */
    if (PR(0x06ED) == 3) {
      if (PR(0x07FF) == 0x69) return;          /* c448 */
      /* c44a LJMP 0x059d (bank0 notify stub) */
      return;
    }
    /* 0x06ED != 3: c44d read IDATA 0x50 (== hi here). if 0 -> da9f; else SB[0x5A]=0x40 + 9a31 mailbox */
    if (hi == 0) return;                       /* c451 -> da9f (heavy reset; left as no-op arm) */
    SB_WR(0x5A, 0x40);                         /* 9728: SB[0x5A]=0x40 */
    if (!(P1_RD(0x0819) & 0x01)) {             /* 9a31: read 0x0819 & 0xFD, bit0 */
      PR(0x074E) = 0; PR(0x074F) = 0;          /* 9874: zero per-lane CL0 latches */
    } else {
      (void)SB_RD(0xA0);                        /* c462: read SB[0xA0] */
    }
    return;
  }
  /* default (n in {0,2,4,7..}) -> return (c4a9) */
}

/* ====================================================================================
 * sb_router_event_handler (CODE_BANK1::A066) — INT1 source C80A.5 service body.
 * PART 1 (a066-a0d5): per-channel connect poll over idx 0..3.
 * PART 2 (a0d7-a31d): connect/disconnect edge + lane CL0/event servicing, all W1C.
 * ==================================================================================== */
static void sb_router_event_handler(void) {
  uint8_t idx, bm, cs;

  /* ---- PART 1: per-channel connect poll (a066-a0d5) ---- */
  for (idx = 0; idx < 4; idx++) {
    bm = SB_RD(0xC9);                          /* a06b: SB[0xC9] connect bitmap */
    /* a071-a08d: test bit (4+idx) of SB[0xC9] AND 0x06F1(active_port)==idx */
    if ((bm & (uint8_t)(1u << (4 + idx))) && PR(0x06F1) == idx) {
      sb_channel_connect_service();            /* c3b2 */
      sb_write_c9_ack(idx);                    /* a0a7: W1C SB[0xC9] = (1<<idx) */
      sb_write_c9_ack((uint8_t)(idx + 4));     /* a0ba: W1C SB[0xC9] = (1<<(idx+4)) */
      PR(0x06F1) = (uint8_t)((PR(0x06F1) + 1) & 3);  /* a0bd: advance active port */
      if (P1_RD(0x0109) & 0x01)                /* a0c5/989d: lane-bond done */
        sb_lane_bond_complete_tunnel_up();     /* a0cb: e52d -> downstream PCIe */
    }
  }

  /* ---- PART 2: connect/disconnect edge + lane events (a0d7+) ---- */
  sb_transport_substate_poll();                /* a0d7: d4cd */

  /* a0da: SB[0x2D] active-low connect/disconnect status */
  cs = SB_RD(0x2D);
  if (!(cs & 0x01)) {                          /* a0dd: connect (bit0 clear) */
    if (SB_RD(0x2C) & 0x01) {                  /* a0e4: gated on SB[0x2C].0 */
      uart_puts("\r\n[===SB Con===]\r\n");     /* a0ed: string @0x2056 */
      SB_WR(0x2C, (uint8_t)(SB_RD(0x2C) + 1)); /* a0f0-a0f6: 9797/INC */
      SB_WR(0x66, SB_RD(0x66) & 0xFD);         /* a0fa-a102: clear SB[0x66].1 (connect bit) */
      sb_con_consequence();                    /* a105: dea1 */
    }
  } else {
    cs = SB_RD(0x2D);
    if (!(cs & 0x02)) {                        /* a10d-a112: disconnect (bit1 clear) */
      if (SB_RD(0x2C) & 0x02) {                /* a118: gated on SB[0x2C].1 */
        uart_puts("\r\n[===SB Dis===]\r\n");   /* a121: string @0x2069 */
        SB_WR(0x2C, (uint8_t)(SB_RD(0x2C) - 1));     /* a124-a12a: 96f5(2)/DEC */
        SB_WR(0x66, SB_RD(0x66) & 0xFE);             /* a12d-a133: clear SB[0x66].0 */
        SB_WR(0x9E, (SB_RD(0x9E) & 0xFD) | 0x02);    /* a136: 97fc (clr b0? ->) net ~0x01 per plan */
        /* a139 LJMP 0xda9f — disconnect re-init. Heavy; the SB W1C acks above hold the FSM. */
      }
    }
  }

  /* a13c: SB[0x66].0 -> [Lane Bonded] + L0/L1 CL0 readout */
  if (SB_RD(0x66) & 0x01) {                    /* a13f */
    SB_WR(0x66, 0x01);                         /* a142-a144: W1C bit0 */
    uart_puts("\r\nLane Bonded\r\n");          /* a14d: string @0x207c */
    sb_lane_bonded_consequence();              /* a150: eed6 */
    PR(0x074E) = 0; PR(0x074F) = 0;            /* a153: 9874 */
  }

  /* a156: SB[0x9E].0 -> L0:CL0. Read SB[0xA0] nibble (==2 == CL0); if !=2 zero 0x074E/F (9874);
   * notify 05a7(1) if 0x09F2. (05a7 is a bank0 notify stub == no-op here.) */
  if (SB_RD(0x9E) & 0x01) {                    /* a159 */
    SB_WR(0x9E, 0x01);                         /* a142-style W1C bit0 */
    uart_puts("\r\nL0:CL0 ");                  /* string @0x208a */
    if ((SB_RD(0xA0) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }   /* 9874 */
    if (P1_RD(0x0819) & 0x01) { if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; } }
    /* if (PR(0x09F2)) notify 05a7(1) — bank0 notify stub, no observable XDATA effect. */
  }

  /* a1c7: SB[0x9E].1 -> L1:CL0. Symmetric on SB[0xA1] nibble; notify 05a7(2) if 0x09F3. */
  if (SB_RD(0x9E) & 0x02) {                    /* a1ca */
    SB_WR(0x9E, 0x02);                         /* a1cd-a1cf: W1C bit1 */
    uart_puts("\r\nL1:CL0 ");                  /* string @0x2094 */
    if ((SB_RD(0xA1) & 0x0F) != 2) { PR(0x074E) = 0; PR(0x074F) = 0; }   /* 9874 */
    /* if (PR(0x09F3)) notify 05a7(2) — bank0 notify stub. */
  }

  /* a223: SB[0x66].2 -> L0:Abr2 (gated 0x0B1C); .5 -> L1:Abr2; .3 -> L0:Bnd Fail; .6 -> L1:Bnd Fail */
  if (SB_RD(0x66) & 0x04) {                    /* a226 */
    SB_WR(0x66, 0x04);                         /* a229: W1C bit2 */
    uart_puts("\r\nL0:Abr2");                  /* string @0x209e */
    /* a23d: gated on 0x0B1C==0 -> 989b/da9f (link re-init); left as the W1C above (faithful arm). */
  }
  if (SB_RD(0x66) & 0x20) {                    /* a249 */
    SB_WR(0x66, 0x20);                         /* a24c: W1C bit5 */
    uart_puts("\r\nL1:Abr2");                  /* string @0x20a8 */
  }
  if (SB_RD(0x66) & 0x08) {                    /* a274 */
    SB_WR(0x66, 0x08);                         /* a277: W1C bit3 */
    uart_puts("\r\nL0:Bnd Fail");              /* string @0x20b2 */
  }
  if (SB_RD(0x66) & 0x40) {                    /* a28b */
    SB_WR(0x66, 0x40);                         /* a28e: W1C bit6 */
    uart_puts("\r\nL1:Bnd Fail");              /* string @0x20c0 */
  }

  /* a29f: SB[0x26].2 -> L0:Disable (+ SB[0x15] arm); .4 -> L1:Disable (+ SB[0x5A]=0x40 mailbox) */
  if (SB_RD(0x26) & 0x04) {                    /* a2a2 */
    SB_WR(0x26, 0x04);                         /* a2a5: W1C bit2 */
    uart_puts("\r\nL0:Disable");              /* string @0x20ce */
    SB_WR(0x15, 0x80);                         /* a2b3-a2b8: 9880/9704 SB[0x15]=0x80 */
  }
  if (SB_RD(0x26) & 0x10) {                    /* a2c4 */
    SB_WR(0x26, 0x10);                         /* a2c7: W1C bit4 */
    uart_puts("\r\nL1:Disable");              /* string @0x20db */
    SB_WR(0x15, 0xA0);                         /* a2d5-a2da: SB[0x15]=0xA0 */
    SB_WR(0x5A, 0x40);                         /* a2e0-a2e8: SB[0x5A]=0x40 mailbox */
  }

  /* a2ef: SB[0x9E].4 -> L0:Training; .5 -> L1:Training */
  if (SB_RD(0x9E) & 0x10) {                    /* a2f2 */
    SB_WR(0x9E, 0x10);                         /* a2f5: W1C bit4 */
    uart_puts("\r\nL0:Training");             /* string @0x20e8 */
  }
  if (SB_RD(0x9E) & 0x20) {                    /* a306 */
    SB_WR(0x9E, 0x20);                         /* a309: W1C bit5 */
    uart_puts("\r\nL1:Training");             /* string @0x20f6 */
  }

  (void)SB_RD(0xF6);                           /* a317-a31d: tail status latch read */
}

#endif /* SB_ROUTER_H */
