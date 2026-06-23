#ifndef USB4_LANEBOND_H
#define USB4_LANEBOND_H
/*
 * USB4 lane-bond / CL0 / PCIe-tunnel bring-up FSM. State lives in XDATA 0x06ED and is driven from
 * the super-loop via e672: 3 -> connection-routing setup, 4 -> b0b4 tunnel power-on / lane-bond,
 * 5 -> the per-lane CL-state walker (8000/850b). Include after usb4_connect.h.
 */

#define U4LB_STATE   u4_fsm_state

static void u4lb_e1cb_e2b9(uint8_t is_e1cb);

/* eb62(p1,p2): set the new FSM state, print "[SB P0<state>]", and store it into 0x06ED. */
static void u4lb_eb62(uint8_t state_lo, u4_fsm_state_t state) {
  sb_tx_go_param = state_lo;
  sb_fsm_state = state;
  uart_puts("\r\n[SB P0");
  uart_puthex(state);
  uart_putc(']');
  u4_fsm_state = sb_fsm_state;
}

/* edf5 -> e2b9: the device->host SB-transport route-query that prompts the host to post the
 * connection-routing descriptor. Gated by the 0x0719 in-flight token; d5da's poll is bounded so the
 * super-loop can't hang. Returns 1 only when a query was actually sent. */
static uint8_t u4lb_edf5_route_query(void) {
  if (e461_inflight_token != 0) return 0;

  sb_tx_flag = 0;
  sb_tx_cmd = 5;
  sb_tx_byte0 = 0x0C;
  sb_tx_byte1 = 3;
  u4lb_e1cb_e2b9(0);
  return 1;
}

/* cm_conn_routing_setup — [ConnRout] connection-routing FSM (state 0x06ED==3). Runs the 0x0758
 * sub-FSM (0x10/0x11/0x00), evaluates the host connect-descriptor confirm gate, prints [ConnRout]
 * and sets 0x0718 ROUTE-ENABLE, latches the lane-width state, and advances toward state 4. */

/* e391 LUT-seed ROM tables: per-descriptor-type width LUT (-> XDATA[0x06F2+i]) and the af38
 * BRANCH-A presence gate (-> XDATA[0x0705+i]). */
static __code const uint8_t u4lb_width_lut_514c[0x13] = {
  0x04,0x04,0x00,0x04,0x04,0x00,0x00,0x00, 0x04,0x04,0x01,0x00, 0x03,0x04,0x00,0x04, 0x00,0x00,0x10
};
static __code const uint8_t u4lb_branchA_gate_515f[0x13] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x01
};

/* ===== OMITTED conn-routing tail: router-op lane-width descriptors (e175/e282/c17f) =====
 * Byte-true port of the stock P12 descriptor-engine chain that advertises lane1's width so the host
 * grants a 2-lane CL0 bond (stock cm_conn_routing_setup a9ca/a9d2/a9d5). r3_write_dispatch(v,0x12xx,2)
 * == P12_WR(xx,v) (DPX=1, 0x12xx); REUSES the existing sb.h P12 engine. HW deep-PHY engine writes
 * (reg 0x3C-0x3F lane-enable mask + commit GO/busy-poll on 0x37/0x38) reproduced IN FULL. */

/* a2c2: engine reg 0x35/0x36/0x37 preamble RMW. */
static void u4lb_a2c2(void) {
  P12_WR(0x35, (uint8_t)((P12_RD(0x35) & 0xC0) | 0x01));
  P12_WR(0x35, (uint8_t)((P12_RD(0x35) & 0x3F) | 0x40));
  P12_WR(0x36, 0xD2);
  P12_WR(0x37, (uint8_t)(P12_RD(0x37) & 0xE0));
}

/* a310 (r3_reg_set_bit7_mask3f): reg[cur] = (reg[cur] & 0x3F) | 0x80. */
static void u4lb_a310(uint8_t cur) {
  P12_WR(cur, (uint8_t)((P12_RD(cur) & 0x3F) | 0x80));
}

/* e711 cleanup tail (CODE:e711): reg0x35=(LIVE reg35 & 0xC0); reg0x3C/0x3D/0x3E/0x3F = 0. */
static void u4lb_e711_tail(uint8_t commit_hi2) {
  (void)commit_hi2;   /* stock uses LIVE reg35, not the commit value */
  P12_WR(0x35, (uint8_t)(P12_RD(0x35) & 0xC0));
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x00); P12_WR(0x3E, 0x00); P12_WR(0x3F, 0x00);
}
/* e83d: commit GO + bounded busy-poll on reg 0x38.0. */
static void u4lb_engine_go(void) {
  uint16_t g;
  P12_WR(0x38, 0x01);
  for (g = 0; (P12_RD(0x38) & 0x01) && g < 0x2000; g++) { }
}
/* e7fb byte-true commit (CODE:e7fb->e83d->e711): GO + e711 (zero 0x3C-0x3F). Local copy so the
 * SHARED u4c_sb_desc_commit (used by u4c_ccb3) is untouched. */
static void u4lb_sb_desc_commit(void) {
  uint8_t commit = (uint8_t)((P12_RD(0x37) & 0x7F) | 0x80);
  P12_WR(0x37, commit);
  u4lb_engine_go();
  u4lb_e711_tail(commit);
}
/* e890: select sub-block (reg 0x37 &= 0x7F) + GO + e711 tail. */
static void u4lb_e890(uint8_t ctrl_low6) {
  uint8_t commit = (uint8_t)(P12_RD(0x37) & 0x7F);
  (void)ctrl_low6;
  P12_WR(0x37, commit);
  u4lb_engine_go();
  u4lb_e711_tail(commit);
}
/* e7f8 (cursor-relative; in ce23 R1=0x35): write val to reg 0x35, commit reg0x37 + GO + tail. */
static void u4lb_e7f8(uint8_t val) {
  uint8_t commit;
  P12_WR(0x35, val);
  commit = (uint8_t)((P12_RD(0x37) & 0x7F) | 0x80);
  P12_WR(0x37, commit);
  u4lb_engine_go();
  u4lb_e711_tail(commit);
}
/* e00c (CODE:e00c): fold 0x0B34..0x0B37 nonzero into 4-bit m; a2ff prelude (reg0x34/0x35 RMW) then
 * write reg0x36 = (a2ff_A & 0xF0) | m. (R1=0x36 after a2ff.) */
static void u4lb_e00c(void) {
  uint8_t m = (uint8_t)((u4lb_b34_lanemask != 0) ? 1 : 0);
  uint8_t a;
  if (u4lb_b35 != 0) m |= 2;
  if (u4lb_b36 != 0) m |= 4;
  if (u4lb_b37 != 0) m |= 8;
  P12_WR(0x34, (uint8_t)((P12_RD(0x34) & 0xF0) | 0x0F));      /* a2ff reg0x34 */
  a = (uint8_t)((P12_RD(0x35) & 0x3F) | 0x80); P12_WR(0x35, a); /* a2ff reg0x35, A=a */
  P12_WR(0x36, (uint8_t)((a & 0xF0) | m));                    /* e00c reg0x36 */
}
/* ce23 (link_apply_lane_mask_reg3f): assemble per-lane enable mask into engine reg 0x3F via the
 * fold 0x3C..0x3E (status regs 0x40/0x41/0x42/0x43, one per stage), commit value -> reg0x35. */
static void u4lb_ce23(uint8_t ctrl_bits, uint8_t set_lanes) {
  uint8_t ctrl_low6 = (uint8_t)(ctrl_bits & 0x3F);
  uint8_t r6, lane_mask, r7;
  r6 = (uint8_t)(P12_RD(0x35) & 0x3F);   /* a334 at entry: R6 = LIVE reg35 & 0x3F (ctrl_bits destroyed) */
  u4lb_e890(ctrl_low6);
  if (set_lanes != 0) {
    r7 = P12_RD(0x40);
    P12_WR(0x3C, (uint8_t)(u4lb_b34_lanemask | r7)); r7 = P12_RD(0x41);
    P12_WR(0x3D, (uint8_t)(u4lb_b35          | r7)); r7 = P12_RD(0x42);
    P12_WR(0x3E, (uint8_t)(u4lb_b36          | r7)); r7 = P12_RD(0x43);
    lane_mask   = (uint8_t)(u4lb_b37          | r7);
  } else {
    r7 = P12_RD(0x40);
    P12_WR(0x3C, (uint8_t)((uint8_t)~u4lb_b34_lanemask & r7)); r7 = P12_RD(0x41);
    P12_WR(0x3D, (uint8_t)((uint8_t)~u4lb_b35          & r7)); r7 = P12_RD(0x42);
    P12_WR(0x3E, (uint8_t)((uint8_t)~u4lb_b36          & r7)); r7 = P12_RD(0x43);
    lane_mask   = (uint8_t)((uint8_t)~u4lb_b37          & r7);
  }
  P12_WR(0x3F, lane_mask);
  u4lb_e00c();
  u4lb_e7f8((uint8_t)((P12_RD(0x35) & 0xC0) | r6));   /* ce6d: (LIVE reg35 & 0xC0)|r6 -> reg0x35 */
  u4lb_b34_lanemask = 0; u4lb_b35 = 0; u4lb_b36 = 0; u4lb_b37 = 0;
}
/* a37b: 0x0B35 = n; ce23(n, set_lanes). */
static void u4lb_a37b(uint8_t n, uint8_t set_lanes) {
  u4lb_b35 = n;
  u4lb_ce23(n, set_lanes);
}
/* c3ce: 2-pass router-op width-descriptor push (engine regs 0x3C-0x3F). set_lanes(0x0B38)==2 here so
 * every pass commits via u4c_sb_desc_commit (the ce23 arm is dead on this path). */
static void u4lb_c3ce(void) {
  uint8_t pass;
  u4_routerop_desc1 = u4lb_b34_lanemask;
  u4_routerop_desc2 = u4lb_b35;
  u4_routerop_desc3 = u4lb_b36;
  pd_msg_type       = u4lb_b37;
  for (pass = 1; pass <= 2; pass++) {
    (void)P12_RD(0x35);
    eng_a327(0x35, (uint8_t)((P12_RD(0x35) & 0xC0) | pass));
    u4lb_b34_lanemask = u4_routerop_desc1; P12_WR(0x3C, u4_routerop_desc1);
    u4lb_b35          = u4_routerop_desc2; P12_WR(0x3D, u4_routerop_desc2);
    u4lb_b36          = u4_routerop_desc3; P12_WR(0x3E, u4_routerop_desc3);
    u4lb_b37          = pd_msg_type;       P12_WR(0x3F, pd_msg_type);
    if ((uint8_t)(u4lb_b38_setlanes ^ 2) == 0) u4lb_sb_desc_commit();   /* byte-true e711 (local) */
    else                                       u4lb_ce23((uint8_t)(u4lb_b38_setlanes ^ 2), u4lb_b38_setlanes);
  }
}
/* e175: router-op lane-width descriptor (set_lanes = lb_lane_width_latch1). */
static void u4lb_e175(uint8_t set_lanes) {
  if (set_lanes != 0) u4lb_lane_active_flags |= 0x01;
  u4lb_a2c2();
  u4lb_a37b(1, set_lanes);
  u4lb_a310(0x35);
  eng_a348(0x36, 0x8D);
  P12_WR(0x37, (uint8_t)((P12_RD(0x37) & 0xE0) | 0x01));
  u4lb_b35 = 0x10;
  u4lb_ce23(0x10, set_lanes);
}
/* e282: router-op connect-route descriptor (set_lanes = u4_connect_route_latch). */
static void u4lb_e282(uint8_t set_lanes) {
  if (set_lanes != 0) u4lb_lane_active_flags |= 0x02;
  u4lb_a2c2();
  u4lb_a37b(2, set_lanes);
  u4lb_a310(0x35);
  eng_a2df(0x36, 0x97);
  u4lb_b35 = 0x02;
  u4lb_ce23(0x02, set_lanes);
}
/* c17f: push two router-op width descriptors via c3ce (desc #1 fixed, desc #2 = lane1 WIDTH). */
static void u4lb_c17f(void) {
  uint8_t desc0;
  (void)P12_RD(0x34);
  P12_WR(0x34, 0x0C);
  eng_a2df(0x35, 0x04);
  u4lb_b36 = 0x20; u4lb_b37 = 0x04; u4lb_b38_setlanes = 0x02;
  u4lb_c3ce();
  (void)P12_RD(0x34);
  P12_WR(0x34, 0x0C);
  eng_a2df(0x35, 0x80);
  desc0 = 0x18;
  if (u4_work_buf[0x1A] & 0x20) desc0 |= 0x04;
  if (u4_cap20g_gate1)          desc0 |= 0x20;
  u4lb_b36 = desc0;
  u4lb_b37 = 0x00;   /* = XDATA[0x0B12], 0 on handmade */
  u4lb_b38_setlanes = 0x02;
  u4lb_c3ce();
  /* stock 0x0B12-gated deep-PHY tail is DEAD (0x0B12==0); omitted byte-true. */
}

static void u4lb_cm_conn_routing_setup(void) {
  connrt_substate_t state = cm_conn_routing_substate;
  if (state == CONNRT_ARM_ROUTE_QUERY) {
    if (u4lb_edf5_route_query() != 1) return;
    cm_conn_routing_substate = CONNRT_AWAIT_RESULT;
    return;
  }
  if (state != CONNRT_AWAIT_RESULT) {
    if (state != CONNRT_PRINT_STATUS) return;
    u4lb_eb62(0, U4FSM_LANE_TRAIN);
    return;
  }

  /* state == 0x11: the main confirm body. eda0 returns a selector: 0 = eval the confirm gate,
   * 1 = idle (leave 0x0758), 2 = route-special re-arm. */
  { uint8_t selector;
    if (u4_route_query_response != 0)      { u4_route_query_response = 0; e461_inflight_token = 0; selector = 0; }
    else if (e461_inflight_token == 0x02) { e461_inflight_token = 0; selector = 2; }
    else                       { selector = 1; }
    if (selector == 2) { cm_conn_routing_substate = CONNRT_ARM_ROUTE_QUERY; return; }
    if (selector != 0) { return; }
  }

  /* mode==0 path: gate on the host connect descriptor 0x0777==0x0C. */
  if (u4_host_desc[0x0] != 0x0C) { cm_conn_routing_substate = CONNRT_ARM_ROUTE_QUERY; return; }

  /* 0x0776 connect-confirm computation. */
  if (u4_connect_route_latch != 0) {
    uint8_t host_status = u4_host_desc[0x1];
    if (((host_status & 0x7F) == 2) || ((u4_work_buf[0x1B] & 1) == 0) ||
        (u4_confirm_input_ce != 0 && u4_confirm_input_cd == 0)) {
      u4_coldboot_seed_gate = 0;
    } else {
      SB_CLR(0xED, 0x80);
    }
  }
  /* e391 width-LUT seed (gated 0x0776==0): the per-descriptor LUT af38 ORs into SBTX[1]. */
  if (u4_coldboot_seed_gate == 0) {
    uint8_t i;
    for (i = 0; i < 0x13; i++) {            /* e391 LOOP1: 0x06F2+i / 0x0705+i width+gate LUT */
      sb_width_lut[(uint16_t)(0x0 + i)] = u4lb_width_lut_514c[i];
      sb_branchA_gate[(uint16_t)(0x0 + i)] = u4lb_branchA_gate_515f[i];
    }
    /* e391 LOOP2 (e3c3-e3d3): d221 zeroes XDATA[0x0B26..0x0B2D] = the CL-walk shadows, on EVERY
     * conn-routing seed. Latent on cold boot (SDCC zeroes XDATA) but the live AMD path is a connect
     * STORM: on re-connect the walker would otherwise read STALE lb_cl_status/lb_eq_status/
     * lb_loop2_scratch/lb_cl0_width and poison the snap&0x80/&0x10 CL-walk gates. */
    for (i = 0; i < 2; i++) {
      lb_cl_status[i] = 0; lb_eq_status[i] = 0; lb_loop2_scratch[i] = 0; lb_cl0_width[i] = 0;
    }
  }

  /* [ConnRout] confirm print + 0x0718 ROUTE-ENABLE. */
  if (u4_coldboot_seed_gate == 0 && u4_confirm_input_ce != 0) {
    uart_puts("[ConnRtmr]");
    u4_route_enable_latch = 0;
  } else {
    uart_puts("[ConnRout]");
    u4_route_enable_latch = 4;
  }

  /* Latch the 0x077a lane-width bits into 0x0819/0x0751/0x0750. */
  { uint8_t host_width = u4_host_desc[0x3];
    if ((host_width & 1) && (u4_work_buf[0x1A] & 1)) { u4_work_buf[0x19] = (u4_work_buf[0x19] & 0xFE) | 1; }
    /* Lane1 enable gated on host advertising the 2nd lane (077A.1) AND lane1 cap (081A.1). With the
     * byte-true cap20g_gate1=0 (main.c, stock blob[0x59].7=0 on this board), 081A=0xE1 (bit1 clear)
     * so this branch stays off -> 0x0819=0x01 (lane0-first) naturally, matching stock. */
    if ((host_width & 0x02) && (u4_work_buf[0x1A] & 2)) { u4_work_buf[0x19] = (u4_work_buf[0x19] & 0xFD) | 2; }
    if ((host_width & 0x10) && (u4_work_buf[0x1A] & 0x10) && (u4_work_buf[0x19] & 1) && (u4_work_buf[0x19] & 2)) lb_lane_width_latch1 = 1;
    else lb_lane_width_latch1 = 0;
    if ((host_width & 0x20) && (u4_work_buf[0x1A] & 0x20)) lb_lane_width_latch0 = 2;
    uart_puts("[Lt77A="); uart_puthex(host_width); uart_puts(" 81A="); uart_puthex(u4_work_buf[0x1A]);
    uart_puts(" 819="); uart_puthex(u4_work_buf[0x19]); uart_puts("]");

    /* ===== a912-a9c2 deep-PHY lane-capability setup (was OMITTED) =====
     * Byte-true port of stock cm_conn_routing_setup CODE_BANK1::a912-a9c2. On the LIVE 2-lane path
     * (latch0==2, 077A.7 & 081A.7 set) this sets 0x0764=1 (gate_b), runs the db80 reg3f/0x09DD lane-
     * mask program, sets SB[0x65] bit7|bit6, then RMWs the link-status reg E716 lo2->0b11. Without it
     * both lanes stay symmetric (gate_a|gate_b==0 -> the state-4 e07d SB2[0]|=0x04 lane-select bit
     * never sets) and the host withholds the CL0 grant. */

    /* a90b-a911: clear both gates (then the sub-FSM sets EXACTLY ONE). */
    u4_phy_gate_a = 0; u4_phy_gate_b = 0;

    /* a912-a95e sub-FSM: set exactly one of gate_a(0x0763)/gate_b(0x0764), or neither.
     * host_width == u4_host_desc[0x3] (=0x077A) is still in scope. 081A == u4_work_buf[0x1A].
     * 99f3() == ((081A >> 6) & 1). */
    if (lb_lane_width_latch0 == 2) {
      /* latch0==2: gate_b=1 iff (077A.7 && 081A.7); else neither. */
      if ((host_width & 0x80) && (u4_work_buf[0x1A] & 0x80)) u4_phy_gate_b = 1;
    } else {
      uint8_t set_a = 0;
      if (u4_enter_usb_accepted != 0 &&            /* 0x07BA != 0 */
          (host_width & 0x40) &&                   /* 077A.6 */
          (u4_work_buf[0x1A] & 0x40)) {            /* 081A.6 (== 99f3()) */
        set_a = 1;
      } else if (u4_connect_route_latch != 0) {    /* 0x07B9 != 0 */
        if ((host_width & 0x40) ||                  /* 077A.6 */
            (u4_work_buf[0x1A] & 0x40)) {           /* 081A.6 (== 99f3()) */
          set_a = 1;
        }
      }
      if (set_a) u4_phy_gate_a = 1;
    }

    /* a967: LCALL db80(R7=gate_a, R5=gate_b) via trampoline 0x05e3. db80 byte-true:
     *   0x0A5C(width_rate_code) = gate_a;
     *   mask = 0x09DD | (gate_b?0x10:0) | (gate_a?0x08:0); a2c1(mask) = {0x09DD=mask; a2c2();}
     *   0x0B35 = 8; ce23(rate,rate); a2c2(); 0x0B35 = 0x10(a3e2); ce23(rate,rate).
     * a2c1 == {u4lb_lane_active_flags = v; u4lb_a2c2();}; a3e2 == {u4lb_b35 = 0x10;}. */
    {
      uint8_t rate = u4_phy_gate_a;
      uint8_t mask;
      u4lb_width_rate_code = u4_phy_gate_a;
      mask = (uint8_t)(u4lb_lane_active_flags
                       | (u4_phy_gate_b ? 0x10 : 0x00)
                       | (u4_phy_gate_a ? 0x08 : 0x00));
      u4lb_lane_active_flags = mask; u4lb_a2c2();        /* a2c1(mask, 0x09DD) */
      u4lb_b35 = 0x08;
      u4lb_ce23(rate, rate);                              /* link_apply_lane_mask_reg3f */
      u4lb_a2c2();
      u4lb_b35 = 0x10;                                    /* a3e2 */
      u4lb_ce23(rate, rate);                              /* link_apply_lane_mask_reg3f (2nd) */
    }

    /* a96a-a98a: SB[0x65] RMW gated on 9a06()==(gate_a|gate_b).
     *   nonzero -> SB[0x65] = (SB[0x65] & 0xBF) | 0x40 (write+readback via 96c7), then
     *              final = (readback & 0x7F) | 0x80, written via 0be6 (bit7=1, bit6=1).
     *   zero    -> SB[0x65] = SB[0x65] & 0x7F (96c2 clears bit6 first), bit7 cleared. */
    if ((u4_phy_gate_a | u4_phy_gate_b) != 0) {
      uint8_t rb = (uint8_t)((SB_RD(0x65) & 0xBF) | 0x40);
      SB_WR(0x65, rb); rb = SB_RD(0x65);                  /* 96c7: write-then-readback */
      SB_WR(0x65, (uint8_t)((rb & 0x7F) | 0x80));
    } else {
      uint8_t rb = (uint8_t)(SB_RD(0x65) & 0xBF);         /* 96c2: clear bit6, write, readback */
      SB_WR(0x65, rb); rb = SB_RD(0x65);
      SB_WR(0x65, (uint8_t)(rb & 0x7F));
    }

    /* a98d-a9ab latch0==1 sb_lane_desc(0x21c4) reseed: does NOT fire on the live latch0==2 path; omitted. */

    /* a9ae-a9c2: if (u4_connect_gate.0 && u4_route_mode.1) -> E716 lo2 = 0b11.
     * Stock target CODE_BANK1::9790 == sb_rmw_set_bits01: E716 = (E716 & 0xFC) | 0x03 (verified;
     * NOT the bank0 d02a/B220 router-op engine). */
    if ((u4_connect_gate & 0x01) && (u4_route_mode & 0x02)) {
      REG_LINK_STATUS_E716 = (uint8_t)((REG_LINK_STATUS_E716 & 0xFC) | 0x03);
    }
  }

  /* c586: negotiated-rate descriptor (SB[0x6A-0x6D]/[0x74-0x75]) + Gen2 lane-eq retrim. */
  {
    uint16_t rate = (uint16_t)((uint16_t)sb_lane_flip[0xB] * 0x20);
    uint8_t rate_hi = (uint8_t)(rate >> 8);
    uint8_t rate_lo = (uint8_t)((rate & 0xFF) | lb_cap_field[0xB]);
    SB_WR(0x6A, rate_hi); SB_WR(0x6B, rate_lo);
    SB_WR(0x6C, rate_hi); SB_WR(0x6D, rate_lo);
    SB_WR(0x74, 0x00); SB_WR(0x75, (uint8_t)((lb_lane_width_latch0 == 2) ? 0x1F : 0x0F));
    if (REG_LANE_RATE_C8FF == 0x04) {
      if (u4_enter_usb_accepted != 0) {
        /* c2c6 (stock c5d2->c5db RETs here): force the advertised PHY rate-field low nibble to 0x02.
         * Omitting this left C294/C314 at the boot default 0x07 (usb4_irq C2E0 RMW(0xF0,0x07)) -> the
         * host trained/granted the higher-rate CL index 0x0D instead of stock's 0x0C. */
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x02);
      } else {
        REG_PHY_LANEA_C294 = (uint8_t)((REG_PHY_LANEA_C294 & 0xF0) | 0x03);
        REG_PHY_LANEA_C293 = (uint8_t)((REG_PHY_LANEA_C293 & 0xFC) | 0x02);
        REG_PHY_LANEB_C314 = (uint8_t)((REG_PHY_LANEB_C314 & 0xF0) | 0x03);
        REG_PHY_LANEB_C313 = (uint8_t)((REG_PHY_LANEB_C313 & 0xFC) | 0x02);
      }
    }
    /* c586 byte-true: stock RETs after c2c6() on the enter_usb!=0 path (C8FF==4), so the C2C5/C345
     * EQ-trim runs ONLY when enter_usb==0. Handmade was doing it unconditionally -> extra per-lane EQ
     * writes on the LIVE Enter_USB bond path that flatten the lanes. Gate on enter_usb==0. */
    if (u4_enter_usb_accepted == 0 && lb_lane_width_latch0 == 1) {
      REG_PHY_LANEA_C2C5 = (uint8_t)((REG_PHY_LANEA_C2C5 & 0xF0) | 0x0F);
      REG_PHY_LANEB_C345 = (uint8_t)((REG_PHY_LANEB_C345 & 0xF0) | 0x0F);
    }
  }

  /* a9ca/a9d2/a9d5: the OMITTED router-op lane-width descriptors — advertise lane1 width so the host
   * grants the 2-lane CL0 bond (without these, per-lane status 0x077A stays 0x00 / lane1 never bonds). */
  u4lb_e175(lb_lane_width_latch1);     /* a9ca: R7 = XDATA[0x0751] */
  u4lb_e282(u4_connect_route_latch);   /* a9d2: R7 = XDATA[0x07B9] */
  u4lb_c17f();                         /* a9d5 */

  cm_conn_routing_substate = CONNRT_PRINT_STATUS;
}

/* State 4 (0x06ED==4) — lane-bond / 20G-rate / RxPLL-reset / per-lane OS1-arm engine, plus its
 * helpers. Every busy-poll is bounded. SB[off]=0x2800+off, SB2[off]=0x2900+off, P1[0x01xx]=0x0100+off; the C2xx/C3xx/
 * CAxx/CCxx/E716 registers are plain XDATA via PR(). */

/* SB2: page-1 lane-block accessor at 0x2900+off (mirror of SB_RD/WR base 0x2800). */
#define SB2_RD(off)      P1_REG8_rd((uint16_t)(0x2900u + (off)))
#define SB2_WR(off, v)   P1_REG8_wr((uint16_t)(0x2900u + (off)), (uint8_t)(v))

/* 96fe: per-lane OS/CDR command-issue descriptor (SB[0x15]=op; SB[0x0C]=(.&0x80)|3). */
static void u4lb_96fe(uint8_t op) {
  SB_WR(0x15, op);
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
}

/* d5da: the per-lane PHY-RX/CDR commit + settle handshake. param==1 adds the SB[0x0F]|=1 prologue;
 * param==0 also zeroes the SB2 descriptor tail when SB[0x0C] > 6. */
static void u4lb_d5da(uint8_t param) {
  sb_tx_go_param = param;
  if (param == 1) {
    SB_WR(0x0F, (SB_RD(0x0F) & 0xFE) | 0x01);
  }
  P1_WR(0x0100, P1_RD(0x0100) & 0xFE);
  SB_WR(0x04, SB_RD(0x04) & 0xFD);
  SB_WR(0x10, 0x01);
  while ((SB_RD(0x2C) & 0x04) == 0) { }
  SB_WR(0x2C, 0x04);
  phy_cc10_cmd_wait(1, 0, 0x0B);
  SB_WR(0x0F, SB_RD(0x0F) & 0xFE);
  if (param != 0) return;
  { uint8_t count = SB_RD(0x0C);
    if (count <= 6) return;
    { uint8_t limit = (uint8_t)(count - 6), i;
      for (i = 0; i < limit; i++) SB2_WR(i, 0x00); }
  }
}

/* e07d: retrain-path per-lane PHY/SB2 lane-block program. */
static void u4lb_e07d(void) {
  uint8_t cfg;
  SB_WR(0x15, 0x61);
  SB2_WR(0x00, 0x09);
  if ((u4_phy_gate_a | u4_phy_gate_b) != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x04);
  if (u4_connect_route_latch != 0)
    SB2_WR(0x00, SB2_RD(0x00) | 0x10);
  cfg = (uint8_t)(((lb_lane_width_latch0 & 0x0F) << 4)
                  | ((u4_work_buf[0x19] & 0x02) ? 0x02 : 0x00)
                  | (u4_work_buf[0x19] & 0x01));
  SB2_WR(0x01, cfg);
  SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x08);
  u4lb_d5da(0);
}

/* e9e7: RstRxpll — reset the RX PLL via C20E + the two CC10 settles. */
static void u4lb_e9e7(void) {
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  REG_PHY_RXPLL_RESET = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_RXPLL_RESET = 0x00;
  phy_cc10_cmd_wait(2, 0, 0x28);
  REG_CPU_CTRL_CC37 &= 0xFB;               /* ea0c: stock calls 984d, which returns CC37 & ~0x04, then writes it back. */
}

/* cdc6 E764 RX-PLL train (stock CODE:cdc6): ramp E764 (set bit3, clear 2/1/0, set bit1), cc10 settle,
 * then poll E762.4 (the RX-PLL "trained/ready" latch). If set -> finish E764->0x19 (set bit0, clear
 * bit1) and clear the busy flag 0x06E9; else clear E764 bits 3-0 and leave busy=1. Re-driven per
 * state-5 walker pass while busy so E762.4 can latch under the host's LIVE lane-training stimulus
 * (stock runs it in state-4; the handmade got E762=00 once and never re-drove it -> the host's lane
 * adapters loop Training/Bonding and never reach CL0). */
static void u4lb_e764_rxpll_train(void) {
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xF7) | 0x08);
  REG_PHY_TIMER_CTRL_E764 &= 0xFB;
  REG_PHY_TIMER_CTRL_E764 &= 0xFE;
  REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFD) | 0x02);
  phy_cc10_cmd_wait(1, 7, 0xCF);
  if (REG_PHY_RXPLL_STATUS & 0x10) {
    REG_PHY_TIMER_CTRL_E764 = (uint8_t)((REG_PHY_TIMER_CTRL_E764 & 0xFE) | 0x01);
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
    phy_rxpll_train_busy = 0;
  } else {
    REG_PHY_TIMER_CTRL_E764 &= 0xF7;
    REG_PHY_TIMER_CTRL_E764 &= 0xFB;
    REG_PHY_TIMER_CTRL_E764 &= 0xFE;
    REG_PHY_TIMER_CTRL_E764 &= 0xFD;
    phy_rxpll_train_busy = 1;
  }
}

/* ebde: rate-lock settle — pulse C20F then spin (bounded) for the C2D0.5 / C350.5 lock bits. */
static void u4lb_ebde(void) {
  REG_PHY_CTRL_C20F = 0xFF;
  phy_cc10_cmd_wait(1, 0, 0x14);
  REG_PHY_CTRL_C20F = 0x00;
  { uint16_t g = 0; while (((REG_PHY_LANEA_LOCK_C2D0 & 0x20) == 0) && ++g < 0x2000); }
  { uint16_t g = 0; while (((REG_PHY_LANEB_LOCK_C350 & 0x20) == 0) && ++g < 0x2000); }
}

/* e980: 20G rate-descriptor apply (C2A8/C328 + C2C9/C349 rate fields + START bit7). */
static void u4lb_e980(void) {
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  u4lb_ebde();
  REG_PHY_LANEA_RATE_START_C2A8 &= 0x3F;
  REG_PHY_LANEA_RATE_DESC_C2C9 = (REG_PHY_LANEA_RATE_DESC_C2C9 & 0x80)
             | (uint8_t)(((REG_PHY_LANEA_RATE_SRC_C2EC & 0x38) >> 3) | 0x40);
  REG_PHY_LANEB_RATE_START_C328 &= 0x3F;
  REG_PHY_LANEB_RATE_DESC_C349 = (REG_PHY_LANEB_RATE_DESC_C349 & 0x80)
             | (uint8_t)(((REG_PHY_LANEB_RATE_SRC_C36C & 0x38) >> 3) | 0x40);
  REG_PHY_LANEA_RATE_START_C2A8 = (REG_PHY_LANEA_RATE_START_C2A8 & 0x3F) | 0x80;
  REG_PHY_LANEB_RATE_START_C328 = (REG_PHY_LANEB_RATE_START_C328 & 0x3F) | 0x80;
}

/* d3b0: Chg2 rate setup (rate=3=20G). SB[0x65] bit4=rate.0, bit5=rate.1; commit via CC10. */
static void u4lb_d3b0(uint8_t rate) {
  u4lb_width_rate_code = rate;
  if (lb_lane_width_latch0 == 1) {
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);
  } else {
    if (rate & 0x01) SB_WR(0x65, SB_RD(0x65) & 0xEF);
    if (rate & 0x02) SB_WR(0x65, SB_RD(0x65) & 0xDF);
    if (rate & 0x01) SB_WR(0x65, (SB_RD(0x65) & 0xEF) | 0x10);
    if (rate & 0x02) SB_WR(0x65, (SB_RD(0x65) & 0xDF) | 0x20);
  }
  phy_cc10_cmd_wait(2, 0, 0xC8);
}

/* ec51: Trig-arm — arm the lane-train trigger (CCE0/CCE1/CCE2/CCE3) that state 5 fires as [Trig]. */
static void u4lb_ec51(void) {
  REG_LANE_TRAIN_ARM = 0x04; REG_LANE_TRAIN_ARM = 0x02;
  REG_LANE_TRAIN_CTRL = (REG_LANE_TRAIN_CTRL & 0xF8) | 0x04;
  REG_LANE_TRAIN_MASK_LO = 0xFF; REG_LANE_TRAIN_MASK_HI = 0xFF;
  REG_LANE_TRAIN_ARM = 0x01;
  u4_lane_train_trigger ^= 0x01;
}

/* b226: CC10 settle. */
static void u4lb_b226(void) { phy_cc10_cmd_wait(2, 0, 0xC8); }

/* ee57: fire ec51 Trig-arm when CCE1.0 is clear or CCE1.1 is set, then read CCE4:CCE5.
 * Stock returns those live counter bytes in R6:R7; cb10 uses them for the state-5 throttle. */
static uint16_t u4lb_ee57(void) {
  if (!(REG_LANE_TRAIN_ARM & 0x01) || (REG_LANE_TRAIN_ARM & 0x02)) u4lb_ec51();
  return ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
}

/* 98ec: arm connection-routing substate, run ee57, then store caller-provided R6:R7 into
 * 0x0768:0x0769. Stock db7a/edd9 both call this as 98ec(0,3). */
static void u4lb_98ec(uint8_t hi, uint8_t lo) {
  cm_conn_routing_substate = CONNRT_ARM_ROUTE_QUERY;
  u4lb_ee57();
  lb_lane_width_cnt_hi = hi;
  lb_lane_width_cnt_lo = lo;
}

/* State-4 PCIe-tunnel power / PHY bring-up (e305 -> ee29 -> ed44 -> df61 / a840 / c593 / b8db).
 * Every 0x1xxx/0x4xxx/0x5xxx/0x6xxx/0x7xxx address is a plane-2 access via P1_RD/P1_WR; bank0
 * CA06/CA81/C659/B40x/B43x/CCxx/E764 are plain XDATA via PR(). */

/* d195: P1[0x7104] = (P1[0x7104] & 0xBF) | 0x40. */
static void u4lb_d195(void) {
  P1_WR(0x7104, (uint8_t)((P1_RD(0x7104) & 0xBF) | 0x40));
}

/* d1d3(hi): returns (P1[hi:0x8D] & 0xF3) | 8. The high byte carries from the prior df61 access. */
static uint8_t u4lb_d1d3(uint16_t base_hi) {
  return (uint8_t)((P1_RD((uint16_t)((base_hi & 0xFF00) | 0x8D)) & 0xF3) | 0x08);
}

/* df61: the plane-2 PHY lane-block program. The 0x7041 read is a dead read whose discarded value
 * leaves the access pointer at 0x7041 for the following write. */
static void u4lb_df61(void) {
  uint8_t v;
  u4lb_d195();
  P1_WR(0x1808, 0x00);
  v = (uint8_t)((P1_RD(0x1835) & 0xFE) | 0x01);
  P1_WR(0x1835, v);
  (void)P1_RD(0x7041);
  P1_WR(0x7041, (uint8_t)(v | 0x40));
  P1_WR(0x6043, 0x70);
  P1_WR(0x6025, (uint8_t)((P1_RD(0x6025) & 0x7F) | 0x80));
  P1_WR(0x508F, 0x01);
  P1_WR(0x508D, u4lb_d1d3(0x5000));
  P1_WR(0x5204, (uint8_t)(P1_RD(0x5204) & 0xFE));
  P1_WR(0x5204, (uint8_t)(P1_RD(0x5204) & 0xFD));
  P1_WR(0x408D, u4lb_d1d3(0x4000));
}

/* ed44: B401/B402 tunnel-link strobe, then df61. */
static void u4lb_ed44(void) {
  REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01);
  REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFD) | 0x02);
  REG_PCIE_TUNNEL_CTRL &= 0xFE;
  REG_PCIE_TUNNEL_CTRL &= 0xFD;
  REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xF7) | 0x08);
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_df61();
}

/* e74e: 0x0B1B=0; CCF8 &= ~0x10; CCF9=4; CCF9=2. */
static void u4lb_e74e(void) {
  cc_subdemux_src = 0;
  REG_CPU_EXT_CTRL &= 0xEF;
  REG_CPU_EXT_STATUS = 0x04;
  REG_CPU_EXT_STATUS = 0x02;
}

/* bank0 e06b (CODE:e06b), byte-true. v=arg (R7):
 *   XDATA[0x0A9F](u4_routerop_desc2) = v;
 *   bc57: P1[0x1235] = (P1[0x1235] & 0x3F) | 0x80   (set engine-commit bit7, R1=0x35);
 *   XDATA[0x0B34] = 1;
 *   link_apply_lane_mask_reg3f(v, 0x12, 2, v)  == u4lb_ce23(v, v)  (re-apply lane mask -> PHY reg 0x3F);
 *   sb_link_reinit_gate(0x0B1C) = (XDATA[0x0A9F] != 0) ? 1 : 0;
 * STOCK calls e06b(1) from a522 (the WIDTH-event service) -> sb_link_reinit_gate=1 -> the next n==3
 * connect descriptor sets P1[0x0109].0 -> the a066 ISR runs e52d (bond-complete transport-up).
 * e06b(0) is called inside e52d/b031 and RESETS the gate to 0 (since v=0 -> 0x0A9F=0). So the gate
 * is the a522 width-event's arming output; handmade never gets a522, so we must arm 0x0B1C manually. */
static void u4lb_e06b(uint8_t v) {
  u4_routerop_desc2 = v;                                   /* XDATA[0x0A9F] = v */
  P12_WR(0x35, (uint8_t)((P12_RD(0x35) & 0x3F) | 0x80));  /* bc57: P1[0x1235] commit-bit (R1=0x35) */
  u4lb_b34_lanemask = 1;                                   /* XDATA[0x0B34] = 1 */
  u4lb_ce23(v, v);                                         /* link_apply_lane_mask_reg3f(v,...) */
  sb_link_reinit_gate = (uint8_t)(u4_routerop_desc2 != 0 ? 1 : 0);
}

/* bank0 b031 (CODE:b031) — the SB-transport / in-band control-adapter REINIT that e52d runs FIRST at
 * bond-complete. Disasm-true (CODE:b031-b101). All prelude regs are R3=2/DPX=1 page-1 banked:
 *   R2=0x12 -> P12_*(0x..) (= P1[0x12xx]);  R2=0x14/0x15/0x18 -> P1[0x14xx/0x15xx/0x18xx].
 * Helper semantics (disasm): a35f(A)=write[cur]=A,readback->A ; a358=read[cur],A=(A&0xFD)|2 ;
 *   a3db(A)=A=(A&0xEF)|0x10,write[cur]=A.
 * Tail chain (b0de-b101): e711(standalone engine reset) -> e8cd(zero 0x0B34..0x0B37) -> e5b0 ->
 *   e8d6(workbuf seed) -> b779(rom desc load) -> c17f -> d4c8 -> e4d2 -> cbf8 -> dcb4 -> e06b(0) -> e74e.
 * Engine bodies (e5b0/e8d6/d4c8/e4d2/cbf8/dcb4/c17f/e74e/rom_load) are byte-true in tree (sb.h /
 * usb4_lanebond.h). */
/* skip_lane: when nonzero, OMIT the lane/link-disturbing pieces (the P1[0x1404]/[0x1405] LINK-adapter
 * RMWs + the e06b(0) lane-mask reg3f CLEAR-path re-apply) that re-drive the just-bonded lanes -> Abr2.
 * Keeps the transport/control-adapter DESCRIPTOR channel arm (P1[0x124x]/[0x1258]/[0x1802]/[0x1508]
 * adapter cfg + e5b0/e8d6/rom_load/c17f/d4c8/e4d2/cbf8/dcb4) — the part that brings the in-band config
 * responder live (so HW can raise EC06.0 on a route=1 read). For testing the Abr2 separation. */
static void u4lb_b031_transport_reinit(uint8_t skip_lane) {
  uint8_t a;

  /* b031: P1[0x1406] &= 0xFE */
  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xFE));
  /* b03f: read P1[0x124d]; a3db -> P1[0x124d] = (A&0xEF)|0x10 */
  P12_WR(0x4D, (uint8_t)((P12_RD(0x4D) & 0xEF) | 0x10));
  /* b049: DEC R1 -> 0x124c; read; &0xEF; write */
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xEF));
  /* b052: P1[0x124e] = 0x02 */
  P12_WR(0x4E, 0x02);
  /* b059: read P1[0x124c]; &0xDF; write */
  P12_WR(0x4C, (uint8_t)(P12_RD(0x4C) & 0xDF));
  /* b063: P1[0x124e] = 0x04 */
  P12_WR(0x4E, 0x04);
  /* b06a: P1[0x1258] = 0x01 */
  P12_WR(0x58, 0x01);
  /* b071: P1[0x1802] staged clears of bits1,2,3,4 (a35f write+readback between masks) */
  a = (uint8_t)(P1_RD(0x1802) & 0xFD); P1_WR(0x1802, a); a = (uint8_t)(P1_RD(0x1802) & 0xFB);
  P1_WR(0x1802, a); a = (uint8_t)(P1_RD(0x1802) & 0xF7); P1_WR(0x1802, a);
  a = (uint8_t)(P1_RD(0x1802) & 0xEF); P1_WR(0x1802, a);
  if (!skip_lane) {
    /* b08c: a358(read P1[0x1404], A=(A&0xFD)|2); a3db(A=(A&0xEF)|0x10; write P1[0x1404]) */
    a = (uint8_t)((P1_RD(0x1404) & 0xFD) | 0x02);
    a = (uint8_t)((a & 0xEF) | 0x10); P1_WR(0x1404, a);
  }
  /* b096: INC R2 -> P1[0x1511] = 0x01 */
  P1_WR(0x1511, 0x01);
  if (!skip_lane) {
    /* b09e: DEC R2 -> P1[0x1404]; read; &0xFD; a35f; &0xEF; write */
    a = (uint8_t)(P1_RD(0x1404) & 0xFD); P1_WR(0x1404, a);
    a = (uint8_t)(P1_RD(0x1404) & 0xEF); P1_WR(0x1404, a);
  }
  /* b0ae: a358(read P1[0x1802], (A&0xFD)|2); &0xFB; |0x04; a35f; &0xF7; |0x08; a35f; a3db(write P1[0x1802]) */
  a = (uint8_t)((P1_RD(0x1802) & 0xFD) | 0x02);
  a = (uint8_t)(((a & 0xFB) | 0x04)); P1_WR(0x1802, a); a = (uint8_t)(P1_RD(0x1802));
  a = (uint8_t)(((a & 0xF7) | 0x08)); P1_WR(0x1802, a); a = (uint8_t)(P1_RD(0x1802));
  a = (uint8_t)((a & 0xEF) | 0x10); P1_WR(0x1802, a);
  if (!skip_lane) {
    /* b0c6: R2=0x14, INC R1 -> P1[0x1405]; read; &0xFE; a35f; |0x01; write */
    a = (uint8_t)(P1_RD(0x1405) & 0xFE); P1_WR(0x1405, a);
    a = (uint8_t)(P1_RD(0x1405) | 0x01); P1_WR(0x1405, a);
  }
  /* b0d6: INC R2 -> P1[0x1508] = 0x08 */
  P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x08);

  /* ---- tail (b0de-b101) ---- */
  /* e711 standalone engine reset: preserve P1[0x1235].7:6 and clear the 0x123C..0x123F engine latch. */
  P12_WR(0x35, (uint8_t)(P12_RD(0x35) & 0xC0));
  P12_WR(0x3C, 0x00); P12_WR(0x3D, 0x00);
  P12_WR(0x3E, 0x00); P12_WR(0x3F, 0x00);
  /* e8cd: zero XDATA 0x0B34..0x0B37 */
  u4lb_b34_lanemask = 0; u4lb_b35 = 0; u4lb_b36 = 0; u4lb_b37 = 0;
  u4c_e5b0();                  /* e5b0 descriptor-engine pre-config */
  u4c_e8d6_seed_workbuf();     /* e8d6 working-buffer seed (0x0994-0x09E3) */
  sb_rom_descriptor_load();    /* b779 DROM/lane desc load */
  u4lb_c17f();                 /* c17f width descriptors */
  u4c_d4c8_edge_engine();      /* d4c8 transport-edge engine */
  u4c_e4d2_edge();             /* e4d2 edge descriptor */
  u4c_cbf8_commit();           /* cbf8 2-round commit */
  u4c_dcb4_transport_reg_reinit(); /* dcb4 + d31e CRC engine */
  if (!skip_lane) {
    u4lb_e06b(0);              /* e06b(0): lane-mask reg3f re-apply (DISTURBS lanes) */
  }
  u4lb_e74e();                 /* e74e: CCF8/CCF9 demux reset */
}

/* ee29: stock runs bank0_e8a9(0x0f), then d185 which RELOADS DPTR=0xB402, reads B402, ANDs 0xFE,
 * RETs (DPTR still 0xB402); the following MOVX @DPTR,A (ee31) writes the result BACK TO B402
 * (B402 &= 0xFE, clears B402.0) — it does NOT write CA06. The prior transcription wrongly wrote
 * CA06, leaving B402.0 set AND clobbering the e305-computed CA06 mode-next (which feeds a840). */
static void u4lb_ee29(void) {
  REG_PCIE_LANE_CTRL_C659 &= 0xFE;
  REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);   /* d185+ee31: B402 &= 0xFE (clear B402.0) */
  u4lb_ed44();
  u4lb_e74e();
  PR(0x0B42) = 0;
  PR(0x0B43) = 0;
}

/* ===========================================================================================
 * d855 PCIe-tunnel EVENT dispatch (the C80A.4 -> c105 -> d855 leg handmade previously OMITTED).
 * RE: bank1 d855 reads P1[0x1508] (d172) and dispatches the HIGHEST set bit:
 *   1508.4 -> e4ea (Enable)            [d17b(0x10) ack]
 *   1508.3 -> ee29 (DisPath)           [d17b(0x08) ack]   (u4lb_ee29 already ported)
 *   1508.2 -> e76b (UPS_Rst_Deassert)  [W1C 1508=0x04, then e76b]
 *   1508.1 -> ee94 + B402 PERST        [W1C 1508=0x02]   (UPS_Rst_Assert)
 * These legs DRIVE the tunnel-side PCIe link (PERST deassert + PHY RXPLL re-trigger), which is what
 * makes the PCIe-DOWN adapter train its width (P1[0x1201] 0->1->2) and raise the width event
 * (P1[0x1203].7 -> P1[0x1407].0). Handmade's old c105 plain-W1C-drained 1508 WITHOUT running these
 * legs -> the link never re-drove -> width never walked -> no GPU. Ported byte-true to stock.
 * =========================================================================================== */

/* d149: P1[0x1235] lane-mask reprogram (RMW &0xC0|3 then &0x3F|0x40), P1[0x1236]=9, P1[0x1237]&=0xE0. */
static void u4lb_d149(void) {
  P1_WR(0x1235, (uint8_t)((P1_RD(0x1235) & 0xC0) | 0x03));
  P1_WR(0x1235, (uint8_t)((P1_RD(0x1235) & 0x3F) | 0x40));
  P1_WR(0x1236, 0x09);
  P1_WR(0x1237, (uint8_t)(P1_RD(0x1237) & 0xE0));
}

/* d1dd: read P1[0x1243] -> this value is ee94's return, the e76b/assert heavy-block gate. */
static uint8_t u4lb_d1dd(void) { return P1_RD(0x1243); }

/* ee94(arg): d149 lane-mask reprogram, e890(arg) sub-block commit, d1dd -> returns P1[0x1243]. */
static uint8_t u4lb_ee94(uint8_t arg) {
  u4lb_d149();
  u4lb_e890(arg);
  return u4lb_d1dd();
}
/* e84d/e85c: save and restore B402.1 around the tunnel reset leg. */
static void u4lb_e84d(void) {
  pcie_ctrl_b402_bit1_save = (uint8_t)(REG_PCIE_CTRL_B402 & 0x02);
  PR(0x0B44) = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
}
static void u4lb_e85c(void) {
  if (pcie_ctrl_b402_bit1_save) PR(0x0B44) = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFD) | 0x02);
}

/* e76b: UPS_Rst_Deassert leg (byte-true to bank1 e76b). ee94(4) returns P1[0x1243], the stock body
 * gate for this leg. Early reset events at A0/A1=07/07 W1C 1508.2 but do not print Deassert; the
 * post-CL0 Deassert comes from the width-event d8d5/d90e path. */
static void u4lb_e76b(void) {
  uint8_t gate = u4lb_ee94(0x04);
  if (gate != 0) {
    uint8_t v;
    u4lb_e84d();
    v = (uint8_t)(P1_RD(0x7041) & 0xBF);                 /* d1af read + ANL 0xBF */
    P1_WR(0x7041, v);                                    /* d1c9: write back to 0x7041 */
    phy_cc10_cmd_wait(1, 0, 0xCF);                       /* phy_cmd_cc10_and_wait */
    u4lb_d195();
    if (!(REG_PCIE_LANE_CTRL_C659 & 0x01)) {
      REG_PCIE_LANE_CTRL_C659 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);   /* d185 -> C659 */
      REG_PCIE_LANE_CTRL_C659 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFE) | 0x01);
    }
    u4lb_e85c();
  }
}

/* e4ea: Enable leg. if C659.0==0: e26a(0x81,1)+C659 write+print Deassert; d30b(3) [CPU-ext clk];
 * clear B480 PERST bits 0-3; e9b5 PHY RXPLL re-trigger; print Enable. The B480 PERST deassert +
 * e9b5 RXPLL trigger DRIVES the tunnel PCIe link up -> adapter width trains. */
static void u4lb_e9b5(void) {
  /* e9b5: C808 strobe (d1a8), then RXPLL cfg A/B bit2/3 toggles + RXPLL trigger 4/8. */
  P1_WR(0x1808, P1_RD(0x1808));                          /* d1a8(0xC808)->plane2 0x1808 dead RMW */
  REG_PHY_RXPLL_CFG_B = 0xFF;
  REG_PHY_RXPLL_CFG_A = (uint8_t)((REG_PHY_RXPLL_CFG_A & 0xFB) | 0x04);
  REG_PHY_RXPLL_CFG_B = (uint8_t)(REG_PHY_RXPLL_CFG_B & 0xFB);
  REG_PHY_RXPLL_CFG_A = (uint8_t)((REG_PHY_RXPLL_CFG_A & 0xF7) | 0x08);
  REG_PHY_RXPLL_CFG_B = (uint8_t)(REG_PHY_RXPLL_CFG_B & 0xF7);
  REG_PHY_RXPLL_TRIGGER = 0x04;
  REG_PHY_RXPLL_TRIGGER = 0x08;
}
static void u4lb_e26a_pwr(uint8_t mode, uint8_t arg);    /* defined below */
static void u4lb_e4ea(void) {
  if (!(REG_PCIE_LANE_CTRL_C659 & 0x01)) {
    u4lb_e26a_pwr(1, 1);                                 /* e26a(0x81,1) -> PwrOn envelope */
    REG_PCIE_LANE_CTRL_C659 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);   /* d185 -> C659 */
  }
  /* d30b(3): CPU-ext clock divider for tunnel — handmade inlines the param==3 branch:
   * CPU_EXT_CTRL = (&0xF8)|2; CCFA=3; CCFB=0xE7; CPU_EXT_STATUS=1. (e74e prologue handled by caller.) */
  u4lb_e74e();
  REG_CPU_EXT_CTRL = (uint8_t)((REG_CPU_EXT_CTRL & 0xF8) | 0x02);
  XDATA_REG8(0xCCFA) = 0x03;
  XDATA_REG8(0xCCFB) = 0xE7;
  REG_CPU_EXT_STATUS = 0x01;
  REG_PCIE_PERST_CTRL = (uint8_t)(REG_PCIE_PERST_CTRL & 0xFE);     /* B480 PERST clear bits 0-3 */
  REG_PCIE_PERST_CTRL = (uint8_t)(REG_PCIE_PERST_CTRL & 0xFD);
  REG_PCIE_PERST_CTRL = (uint8_t)(REG_PCIE_PERST_CTRL & 0xFB);
  REG_PCIE_PERST_CTRL = (uint8_t)(REG_PCIE_PERST_CTRL & 0xF7);
  u4lb_e9b5();
}

/* d90e = usb4_link_phy_reconfig (BANK0, byte-true). The WIDTH-event PHY reconfig that emits the
 * pre-commit tunnel-deassert. Stock reaches it via a522 (P1[0x1407].0 width event) -> a578
 * sub-leg -> d90e. Steps (CODE:d90e):
 *   1) rd SB-engine P1[0x1267]; RMW: (&0xFD|2) write, then (&0xFB) write, then (&0xF7|8) write
 *      (set bit1=lane-width-go, clear bit2, set bit3=commit on the descriptor-engine plane).
 *   2) if u4_connect_gate.1: e0d9(3) PHY CDR mode-3 seed (C20E/C20F=0x26, C214=0x26, zeros around).
 *   3) if (u4_route_mode & 0x81): e9b5() RXPLL re-trigger; B402 &= 0xFE; B402 &= 0xFD.
 *   4) d30b(1): e74e + CPU-ext clock (CPU_EXT_CTRL(&0xF8)|2, CCFA=3, CCFB=0xE7, CPU_EXT_STATUS=1).
 * The e9b5 RXPLL re-trigger + B402 PERST clears are the SAME link-safe ops e4ea runs (HW-proven not
 * to drop the bond at handmade's bonded state). The Deassert is the wire event the TB4 host waits
 * for before posting its post-CL0 router-op + committing SB[0x66]=01 (Lane Bonded). */
static void u4lb_e0d9_mode3(void) {
  /* e0d9(3): bVar1 = 3 & 3 = 3 (nonzero) -> the bVar1!=0 branch:
   *   zero3(C20E) [C20E=C20F=C210=0]; C20E=0x26; C20F=0x26; zero3(C214); C214=0x26. */
  REG_PHY_RXPLL_RESET = 0x00; REG_PHY_CTRL_C20F = 0x00; REG_PHY_CDR_SEED_C210 = 0x00;
  REG_PHY_RXPLL_RESET = 0x26; REG_PHY_CTRL_C20F = 0x26;
  REG_PHY_CDR_SEED_C214 = 0x00; REG_PHY_CDR_SEED_C215 = 0x00; REG_PHY_CDR_SEED_C216 = 0x00;
  REG_PHY_CDR_SEED_C214 = 0x26;
}
static void u4lb_d90e_link_phy_reconfig(void) {
  uint8_t v = P1_RD(0x1267);
  P1_WR(0x1267, (uint8_t)((v & 0xFD) | 0x02));               /* set bit1 */
  v = P1_RD(0x1267);
  P1_WR(0x1267, (uint8_t)(v & 0xFB));                        /* clear bit2 */
  v = P1_RD(0x1267);
  P1_WR(0x1267, (uint8_t)((v & 0xF7) | 0x08));               /* set bit3 (commit) */
  if (u4_connect_gate & 0x02) {
    u4lb_e0d9_mode3();
  }
  if (u4_route_mode & 0x81) {
    u4lb_e9b5();                                             /* RXPLL re-trigger */
    REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);
    REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
  }
  /* d30b(1): e74e + CPU-ext clock (same envelope e4ea runs inline). */
  u4lb_e74e();
  REG_CPU_EXT_CTRL = (uint8_t)((REG_CPU_EXT_CTRL & 0xF8) | 0x02);
  XDATA_REG8(0xCCFA) = 0x03;
  XDATA_REG8(0xCCFB) = 0xE7;
  REG_CPU_EXT_STATUS = 0x01;
}

/* d855: tunnel-event dispatch on P1[0x1508] highest set bit (byte-true to bank1 d855). assert=1
 * runs the 1508.1 UPS_Rst_Assert heavy block (stock's param_5 gate); the bit2/bit4/bit3 legs are
 * always taken when their bit is set (gated internally on u4_route_mode / ee94 return). */
static void u4lb_d855(uint8_t assert) {
  uint8_t p1508 = P1_RD(P1_USB4_TUNNEL_EVENT_STATUS_1508);
  if (p1508 & 0x10) {                       /* 1508.4 -> Enable (e4ea), gated u4_route_mode.0/.7 */
    if (u4_route_mode & 0x81) { P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x10); u4lb_e4ea(); }
  } else if (p1508 & 0x08) {                 /* 1508.3 -> DisPath (ee29), gated u4_route_mode.0/.7 */
    if (u4_route_mode & 0x81) { P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x08); u4lb_ee29(); }
  } else if (p1508 & 0x04) {                 /* 1508.2 -> UPS_Rst_Deassert (e76b) */
    P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x04);
    u4lb_e76b();
  } else if (p1508 & 0x02) {                 /* 1508.1 -> UPS_Rst_Assert */
    P1_WR(P1_USB4_TUNNEL_EVENT_STATUS_1508, 0x02);
    (void)u4lb_ee94(0x02);
    if (assert) {
      uint8_t v;
      REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);   /* B402 &= 0xFD */
      v = (uint8_t)(P1_RD(0x7041) | 0x40);                /* d1af read + ORL 0x40 */
      P1_WR(0x7041, v);                                   /* d1c9: write back to 0x7041 */
      phy_cc10_cmd_wait(1, 0, 0xCF);
      REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFE);   /* d185 -> B402 */
    }
  }
}

/* e26a(1,1): stock calls cdc6(1), then sets C656.5 inside the e305 power-on envelope. */
static void u4lb_e26a_pwr(uint8_t mode, uint8_t arg) {
  if ((u4_route_mode & 0x81) != 0) {
    if (mode == 1) {
      if (arg & 1) u4lb_e764_rxpll_train();
      else phy_cc10_cmd_wait(1, 7, 0xCF);
    }
    REG_HDDPC_CTRL = (uint8_t)((REG_HDDPC_CTRL & 0xDF) | (uint8_t)((mode & 1) << 5));
  }
  uart_puts(mode == 1 ? "[PwrOn]" : "[Pwroff]");
}

/* d702: CC10-mailbox lane-mask bit-distributor (plane-2 0x78AF..0x7BAF slot bit7 = mask.slot). */
static void u4lb_d702(uint8_t newmask) {
  __xdata uint8_t slot;
  slot = (uint8_t)(P1_RD(0x78AF) & 0x7F);
  P1_WR(0x78AF, (uint8_t)(((newmask & 0x01) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x79AF) & 0x7F);
  P1_WR(0x79AF, (uint8_t)(((newmask & 0x02) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x7AAF) & 0x7F);
  P1_WR(0x7AAF, (uint8_t)(((newmask & 0x04) ? 0x80 : 0x00) | slot));
  slot = (uint8_t)(P1_RD(0x7BAF) & 0x7F);
  P1_WR(0x7BAF, (uint8_t)(((newmask & 0x08) ? 0x80 : 0x00) | slot));
}

/* c089: the 4-round B434 lane ramp toward target, with a d702 + CC10 settle each round. */
static void u4lb_c089_lane_ramp(uint8_t target) {
  __xdata uint8_t curmask = (uint8_t)(REG_PCIE_LINK_STATE & 0x0F);
  __xdata uint8_t roundbit = 0x01;
  __xdata uint8_t round = 0;
  do {
    __xdata uint8_t newmask;
    if (target < 0x0F) {
      if (curmask == target) return;
      newmask = (uint8_t)((uint8_t)(target | (uint8_t)(roundbit ^ 0x0F)) & curmask);
    } else {
      if (curmask == 0x0F) return;
      newmask = (uint8_t)(roundbit | curmask);
    }
    curmask = newmask;
    REG_PCIE_LINK_STATE = (uint8_t)(newmask | (uint8_t)(REG_PCIE_LINK_STATE & 0xF0));
    u4lb_d702(newmask);
    phy_cc10_cmd_wait(2, 0, 0xC7);
    roundbit = (uint8_t)(roundbit << 1);
    round++;
  } while (round < 4);
}

/* d436: PCIe-tunnel link-width config — bracket B402.1, run the lane ramp, strobe B401.0, set B436. */
static void u4lb_d436(uint8_t mask) {
  __xdata uint8_t saved_b402_1 = (uint8_t)(REG_PCIE_CTRL_B402 & 0x02);
  REG_PCIE_CTRL_B402 = (uint8_t)(REG_PCIE_CTRL_B402 & 0xFD);
  u4lb_c089_lane_ramp(mask);
  if (mask != 0x0F) {
    REG_PCIE_TUNNEL_CTRL = (uint8_t)((REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01);
    REG_PCIE_TUNNEL_CTRL = (uint8_t)(REG_PCIE_TUNNEL_CTRL & 0xFE);
  }
  if (saved_b402_1 != 0) REG_PCIE_CTRL_B402 = (uint8_t)((REG_PCIE_CTRL_B402 & 0xFD) | 0x02);
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0xF0) | (uint8_t)(mask & 0x0E));
  REG_PCIE_LANE_CONFIG = (uint8_t)((REG_PCIE_LANE_CONFIG & 0x0F) | (uint8_t)(((uint8_t)(REG_PCIE_LINK_PARAM_B404 & 0x0F) ^ 0x0F) << 4));
}

/* a840 gen->speed table, indexed by 0x0A5D (gen). */
static __code const uint8_t u4lb_a840_speed_38cc[8] = { 0x02,0x01,0x03,0x01,0x03,0x01,0x03,0x02 };

/* a840: PCIe link-speed/width config (B403/B431 + d436 width). gen=0x0AEC, lane=0x0AED. */
static void u4lb_a840(uint8_t param) {
  uint8_t gen = u4_link_gen;
  uint8_t lane = u4_link_lane;
  uint8_t usb4;
  uint8_t width_code;
  REG_CPU_CTRL_CA81 &= 0xFE;
  if (gen == 3 && lane == 3) {
    if ((u4_route_mode & 0x81) != 0) {
      uint8_t idx;
      u4lb_gen_index = gen; idx = (uint8_t)(u4lb_gen_index & 0x03);
      gen = u4lb_a840_speed_38cc[(uint8_t)(idx << 1)];
      lane = u4lb_a840_speed_38cc[(uint8_t)((idx << 1) + 1)];
      if (u4_connect_route_latch != 0) lane = 1;
    } else {
      static __code const uint8_t t5d24[5] = { 0x00, 0x00, 0x02, 0x02, 0x02 };
      static __code const uint8_t t5d29[5] = { 0x01, 0x00, 0x00, 0x01, 0x02 };
      u4lb_gen_index = gen;
      gen = t5d24[u4lb_gen_index]; lane = t5d29[u4lb_gen_index];
    }
  }
  usb4 = (uint8_t)(u4_route_mode & 0x81);
  if (usb4 == 0) {
    if (gen >= 3) {
      REG_CPU_MODE_NEXT &= 0x1F;
    } else if (lane < 2) {
      REG_CPU_MODE_NEXT = (uint8_t)((REG_CPU_MODE_NEXT & 0x1F) | 0x20);
    } else {
      REG_CPU_MODE_NEXT &= 0x1F;
    }
  } else {
    if (gen == 3) {
      REG_CPU_MODE_NEXT &= 0x1F;
    }
  }
  if (gen < 3) {
    REG_TUNNEL_CTRL_B403 = (uint8_t)((REG_TUNNEL_CTRL_B403 & 0xFE) | 0x01);
    P1_WR(0x40B0, (uint8_t)((uint8_t)(gen + 1) | (uint8_t)(P1_RD(0x40B0) & 0xF0)));
  } else {
    REG_TUNNEL_CTRL_B403 &= 0xFE;
    P1_WR(0x40B0, (uint8_t)((P1_RD(0x40B0) & 0xF0) | 0x04));
  }
  width_code = 0;
  if (lane < 3) {
    if (lane == 1) width_code = 0x0C;
    else if (lane == 0) width_code = 0x0E;
  }
  u4lb_width_rate_code = width_code;
  REG_TUNNEL_LINK_STATUS = (uint8_t)((REG_TUNNEL_LINK_STATUS & 0xF0) | width_code);
  u4lb_d436(width_code);
  if ((uint8_t)(u4_route_mode & 0x81) != 0) u4lb_ed44();
  (void)param;
}

/* e305: state-4 PcieTunnel power-on prologue (gated (0x09FA & 0x81)): conditional CA06 mode-next
 * select, ee29, B402 &= ~2, a840, e26a(1,1). */
static void u4lb_e305(uint8_t param) {
  if ((u4_route_mode & 0x81) == 0) return;
  if ((u4_link_gen == 3) ||
      (((lb_lane_width_latch0 != 2) || (lb_lane_width_latch1 != 0)) && (lb_lane_width_latch0 != 1))) {
    REG_CPU_MODE_NEXT &= 0x1F;
  } else {
    REG_CPU_MODE_NEXT = (uint8_t)((REG_CPU_MODE_NEXT & 0x1F) | 0x20);
  }
  u4lb_ee29();
  REG_PCIE_CTRL_B402 &= 0xFD;
  u4lb_a840(param);
  u4lb_e26a_pwr(1, 1);
}

/* c593: bank0 tunnel/PHY commit. e916 returns the plane-2 0x2805 read that seeds the 0x1335 RMWs. */
static uint8_t u4lb_e916(void) { return P1_RD(0x2805); }
static void u4lb_c593(void) {
  uint8_t v;
  REG_TUNNEL_PHY_CFG_CCB0 = (uint8_t)((REG_TUNNEL_PHY_CFG_CCB0 & 0xF8) | 0x05);
  REG_TUNNEL_PHY_CFG_CCB2 = 0x00;
  REG_TUNNEL_PHY_TIMER_CCB3 = 0xC8;
  P1_WR(0x134D, 0x04);
  P1_WR(0x1334, 0x02);
  P1_WR(0x1335, 0x02);
  v = u4lb_e916();
  P1_WR(0x1335, (uint8_t)((v & 0xFE) | 0x01));
  if (lb_lane_bonded_flag == 0) {
    v = u4lb_e916();
    P1_WR(0x1335, (uint8_t)(v & 0xFD));
  } else {
    P1_WR(0x1335, (uint8_t)((P1_RD(0x1335) & 0xFD) | 0x02));
  }
  v = u4lb_e916();
  P1_WR(0x1335, (uint8_t)((v & 0xFB) | 0x04));
  P1_WR(0x1334, (uint8_t)((P1_RD(0x1334) & 0x7F) | 0x80));
  P1_WR(0x1285, (uint8_t)((P1_RD(0x1285) & 0x0F) | 0x30));
  P1_WR(0x1335, 0x02);
  P1_WR(0x1206, 0x58);
}

/* b8db: CDR/PLL validate loop. A prologue early-returns in the already-locked cases and otherwise
 * sets the per-lane margin window (lo/hi = CDR phase; lo52:lo54 / hi52:hi54 = a 16-bit eye margin),
 * then a bounded (<=10) loop polls bit6 PLL-lock plus the full CDR-margin compare and fires e9e7
 * (RxPLL reset) on any miss. The caller discards the return. */
static __xdata uint8_t b8db_lo, b8db_hi, b8db_lo52, b8db_hi52, b8db_lo54, b8db_hi54;
static void u4lb_b8db(void) {
  __xdata uint8_t phase0, phase1, margin0_lo, margin0_hi, margin1_lo, margin1_hi;
  uint8_t iter;
  if ((P1_RD(0x0000) & 0x02) == 0) {
    if ((PR(0x92F8) & 0x0C) == 0) return;
    b8db_lo = 0x01; b8db_hi = 0x28;
    b8db_lo52 = 0x01; b8db_hi52 = 0x3D; b8db_lo54 = 0x01; b8db_hi54 = 0x43;
  } else if (lb_lane_width_latch0 == 1) {
    if (REG_PHY_LANEA_LOCK_C297 & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x28;
    if (u4_enter_usb_accepted == 0) { b8db_lo52=0x01; b8db_hi52=0x47; b8db_lo54=0x01; b8db_hi54=0x4D; }
    else { b8db_lo52=0x01; b8db_hi52=0x3D; b8db_lo54=0x01; b8db_hi54=0x43; }
  } else {
    if (REG_PHY_LANEA_LOCK_C2A7 & 0x20) return;
    b8db_lo = 0x01; b8db_hi = 0x20;
    if (u4_enter_usb_accepted != 0) { b8db_lo52=0x01; b8db_hi52=0x3E; b8db_lo54=0x01; b8db_hi54=0x42; }
    else { b8db_lo52=0x01; b8db_hi52=0x48; b8db_lo54=0x01; b8db_hi54=0x4C; }
  }
  for (iter = 0; iter < 10; iter++) {
    phase0 = REG_PHY_LANEA_MARGIN_PHASE_C2D2 & 0x3F; margin0_lo = REG_PHY_LANEA_MARGIN_EYE_C2D9; margin0_hi = REG_PHY_LANEA_MARGIN_EYE_C2DA;
    phase1 = REG_PHY_LANEB_MARGIN_PHASE_C352 & 0x3F; margin1_lo = REG_PHY_LANEB_MARGIN_EYE_C359; margin1_hi = REG_PHY_LANEB_MARGIN_EYE_C35A;
    if ((REG_PHY_LANEA_LOCK_C2D0 & 0x40) && (REG_PHY_LANEB_LOCK_C350 & 0x40) &&
        phase0 >= b8db_lo && phase0 <= b8db_hi && phase1 >= b8db_lo && phase1 <= b8db_hi &&
        (uint8_t)(b8db_lo52 - (margin0_hi <  b8db_lo54         ? 1 : 0)) <= margin0_lo &&
        margin0_lo <  (uint8_t)(b8db_hi52 - (margin0_hi < (uint8_t)(b8db_hi54 + 1) ? 1 : 0)) &&
        (uint8_t)(b8db_lo52 - (margin1_hi <  b8db_lo54         ? 1 : 0)) <= margin1_lo &&
        margin1_lo <  (uint8_t)(b8db_hi52 - (margin1_hi < (uint8_t)(b8db_hi54 + 1) ? 1 : 0)))
      return;
    u4lb_e9e7();
  }
}

/* b0b4 body — state-4, assembled in dependency order. */
static void u4lb_state4_b0b4(void) {
  /* (A) entry / retrain guard: 0x0776 != 0 -> retrain {e07d; b226} x2 */
  if (u4_coldboot_seed_gate != 0) {
    uint8_t i;
    for (i = 0; i < 2; i++) { u4lb_e07d(); u4lb_b226(); }
  } else {
    /* normal-connect OS-prewrite */
    if (u4_work_buf[0x19] & 0x01) {
      uint8_t op = (lb_lane_width_latch0 == 2) ? 0x85 : 0x81;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    if (u4_work_buf[0x19] & 0x02) {
      uint8_t op = (lb_lane_width_latch0 == 2) ? 0xA5 : 0xA1;
      SB_WR(0x15, op);
      SB_WR(0x0C, (SB_RD(0x0C) & 0x80) | 0x03);
      u4lb_d5da(1);
    }
    u4lb_b226();
  }
  /* (B) lane-width ready gate: (0x0768:0x0769) - (CCE4:CCE5) < 0x38 -> abort */
  { uint16_t width = ((uint16_t)lb_lane_width_cnt_hi << 8) | lb_lane_width_cnt_lo;
    uint16_t neg   = ((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO;
    if ((uint16_t)(width - neg) < 0x0038) { uart_puts("[b4:WIDGATE-abort]"); return; }
  }

  /* (C) connect-present gate: 0x0765==0 && 0x0766==0 -> abort */
  if (sb_connect_present == 0 && sb_route_up_trigger == 0) { uart_puts("[b4:CONGATE-abort]"); return; }

  /* E716/CA06 enable, gated 0x0AF1.0 */
  if (u4_connect_gate & 0x01) {
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    phy_cc10_cmd_wait(2, 0, 0x28);
    REG_LINK_STATUS_E716 &= 0xFC;
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;
    REG_CPU_CTRL_CA81 &= 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
  }

  /* e305 includes stock e26a(1,1): cdc6(1) RXPLL train and the HDDPC C656.5 power-on strobe. */
  u4lb_e305(1);

  /* L0 OS-arm, gated 0x0819.0 */
  if (u4_work_buf[0x19] & 0x01) {
    u4lb_96fe(0x82);
    u4lb_d5da(1);
    u4_work_buf[0x1E] = (u4_work_buf[0x1E] & 0x7F) | 0x80;
  }
  /* L1 OS-arm, gated 0x0819.1 */
  if (u4_work_buf[0x19] & 0x02) {
    u4lb_96fe(0xA2);
    u4lb_d5da(1);
    u4_work_buf[0x1F] = (u4_work_buf[0x1F] & 0x7F) | 0x80;
  }

  /* CC37.2 SET (b18f/b192) -> d3b0(3) Chg2 20G -> e980 rate apply -> e9e7 RstRxpll;
   * stock e9e7 and the b1a0 984d call both clear CC37.2 before b8db CDR/PLL validate. */
  REG_CPU_CTRL_CC37 = (REG_CPU_CTRL_CC37 & 0xFB) | 0x04;
  u4lb_d3b0(3);
  u4lb_e980();
  u4lb_e9e7();
  REG_CPU_CTRL_CC37 &= 0xFB;   /* b1a0: 984d returns CC37 & ~0x04, then b1a3 writes it before b8db. */

  /* b8db: CDR/PLL validate loop */
  u4lb_b8db();

  /* CA60.3 set (tunnel-adapter enable) */
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;

  /* c593: bank0 tunnel/PHY commit */
  u4lb_c593();

  /* Unmask the adapter/tunnel events that aggregate into C80A.4. Stock reaches this mask state
   * through e72e/b031; without it, pending P1[0x1508] tunnel events do not call c105. */
  P1_WR(P1_USB4_ADP_EVENT_MASK_1406, (uint8_t)(P1_RD(P1_USB4_ADP_EVENT_MASK_1406) & 0xF6));     /* clear bits 0,3 -> 0x1F becomes 0x16 */
  P1_WR(P1_USB4_TUNNEL_EVENT_MASK_1507, (uint8_t)(P1_RD(P1_USB4_TUNNEL_EVENT_MASK_1507) & 0x61)); /* clear bits 1-4 -> 0x7F becomes 0x61 */

  /* L0 OS1 trigger, gated 0x0819.0 */
  if (u4_work_buf[0x19] & 0x01) {
    SB_WR(0x50, 0x02);
    P1_WR(0x010B, P1_RD(0x010B) | 0x01);
    lb_loop2_state[0x0] = LP2_CL_INIT; lb_loop1_state[0x0] = LP1_WIDTH_INIT;
  } else {
    lb_loop2_state[0x0] = LP2_CL_IDLE; lb_loop1_state[0x0] = LP1_PARKED;
  }
  /* L1 OS1 trigger, gated 0x0819.1 */
  if (u4_work_buf[0x19] & 0x02) {
    SB_WR(0x5A, 0x02);
    P1_WR(0x010B, (P1_RD(0x010B) & 0xFD) | 0x02);
    lb_loop2_state[0x1] = LP2_CL_INIT; lb_loop1_state[0x1] = LP1_WIDTH_INIT;
  } else {
    lb_loop2_state[0x1] = LP2_CL_IDLE; lb_loop1_state[0x1] = LP1_PARKED;
  }

  /* ec51 Trig-arm */
  u4lb_ec51();

  /* latch negotiated width 0x074E:0x074F = CCE4:CCE5 */
  lb_laneA_cl0_latch = REG_LANE_WIDTH_CNT_HI;
  lb_laneB_cl0_latch = REG_LANE_WIDTH_CNT_LO;

  /* eb62(0,5) -> [SB P05] -> state 5 */
  u4lb_eb62(0, U4FSM_LANE_BOND);
}

/* State 5 (0x06ED==5) — the CL-state lane-bond walker (8000 when 0x0718==4, else 850b): the
 * per-lane FSMs that drive SB[0xA0]/[0xA1] toward CL0(0x02) via the 0x0800-plane shadow + ea7c,
 * emitting Lx:CL0. Runs from the super-loop; all helpers are bounded. */

/* ee6e: per-lane SB connect-present = SB[lane?0x60:0x56].0. */
static uint8_t u4lb_ee6e(uint8_t lane) { return (uint8_t)(SB_RD(lane ? 0x60 : 0x56) & 0x01); }

/* eda0: route-special selector (0=eval-path, 1=idle, 2=route-special); clears 0x0775/0x0719. */
static uint8_t u4lb_eda0(void) {
  if (u4_route_query_response != 0) { u4_route_query_response = 0; e461_inflight_token = 0; return 0; }
  if (e461_inflight_token == 0x02) { e461_inflight_token = 0; return 2; }
  return 1;
}

/* e1cb/e2b9: the SB-transport descriptor builder (CODE_BANK1::e1cb is the 0x0776!=0 live AMD path,
 * e2b9 the 0x0776==0 path). Byte-true transcription:
 *   sb_tx_cmd  (0x0AA8) = byte0   sb_tx_byte0 (0x0AA9) = byte1   sb_tx_byte1 (0x0AAA) = flag3
 *   d4cd(); 997e -> SBTX[0]=byte1; 9923 -> SBTX[1]=flag3|((0xAAB.0)<<7);
 *   SB[0x0C] form: (0xAAB!=0) ? (byte1+8)|(SB0C&0x80) : (SB0C&0x80)|0x08;
 *   e1cb: SB[0x15] = (byte0<<1)|0x41 (via 972a);  e2b9: SB[0x15] = byte0 (via 96f7);
 *   d5da(0); 0x0719 = 1 (inflight token). */
static void u4lb_e1cb_e2b9(uint8_t is_e1cb) {
  uint8_t aab;                       /* XDATA[0x0AAB] */
  uint8_t form;
  sb_d4cd_transport_edges();         /* d4cd */
  aab = sb_tx_flag;                  /* stock reads 0x0AAB after d4cd */
  SBTX_WR(0, sb_tx_byte0);           /* 997e: SBTX[0] = XDATA[0x0AA9] (=byte1) */
  SBTX_WR(1, (uint8_t)(sb_tx_byte1 | ((aab & 1) << 7)));  /* 9923: SBTX[1] = XDATA[0x0AAA] | ((0xAAB.0)<<7) */
  if (aab != 0)
    form = (uint8_t)(((sb_tx_byte0 + 8) & 0xFF) | (SB_RD(0x0C) & 0x80));  /* 99ac: (byte1+8)|SB0C.7 */
  else
    form = (uint8_t)((SB_RD(0x0C) & 0x80) | 0x08);                        /* 9695: (SB0C.7)|0x08 */
  SB_WR(0x0C, form);
  if (is_e1cb)
    SB_WR(0x15, (uint8_t)((sb_tx_cmd << 1) | 0x41));   /* e1cb 972a: SB[0x15]=(byte0<<1)|0x41 */
  else
    SB_WR(0x15, sb_tx_cmd);                            /* e2b9 96f7: SB[0x15]=byte0 */
  u4lb_d5da(0);                                        /* d5da(0) TX trigger */
  /* 97ef tail: CCD9 strobe (4,2); handmade keeps the legacy 1 strobe because removing it stalls TX completion. */
  REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02; REG_XFER2_DMA_STATUS = 0x01;
  e461_inflight_token = 0x01;                          /* 0x0719 = 1 */
}

/* e461: the SB-transport route push the walker depends on (CODE_BANK1::e461). Gated by the 0x0719
 * in-flight token. Live AMD path: 0x0718(route_enable)=4 -> e487; 0x0776=1 -> e1cb with byte0=0,
 * byte1=0x0D, flag3=4 (9966 seeds R5=0x0D,R3=4,R7=0). Returns 1 only when a push was issued. */
static uint8_t u4lb_e461(void) {
  if (e461_inflight_token != 0) return 0;            /* XDATA[0x0719] in-flight */
  if (u4_route_enable_latch == 0) {
    /* 0x0718==0: 9960 seeds byte0=route|1, R5=0x0D, R3=4, 0xAAB=0; then e2b9. (Dead on AMD.) */
    sb_tx_flag = 0;                                  /* 9966: XDATA[0x0AAB]=0 */
    sb_tx_cmd  = (uint8_t)(u4_route_enable_latch | 0x01);  /* 9960: byte0 = route|1 = 1 */
    sb_tx_byte0 = 0x0D;                              /* R5 */
    sb_tx_byte1 = 0x04;                              /* R3 */
    u4lb_e1cb_e2b9(0);                               /* e2b9 */
    return 1;
  }
  if (u4_coldboot_seed_gate != 0) {
    /* 0x0718!=0 && 0x0776!=0 -> e1cb (THE live AMD CL-walk push). */
    sb_tx_flag = 0;                                  /* 9966: XDATA[0x0AAB]=0 */
    sb_tx_cmd  = 0x00;                               /* 9966 returns A=0 -> byte0 = 0 */
    sb_tx_byte0 = 0x0D;                              /* 9966: R5 = 0x0D */
    sb_tx_byte1 = 0x04;                              /* 9966: R3 = 0x04 */
    u4lb_e1cb_e2b9(1);                               /* e1cb */
    return 1;
  }
  /* 0x0718!=0 && 0x0776==0 -> e499: 9960(0x0718) + e2b9. */
  sb_tx_flag = 0;
  sb_tx_cmd  = (uint8_t)(u4_route_enable_latch | 0x01);  /* 9960: byte0 = route|1 */
  sb_tx_byte0 = 0x0D;
  sb_tx_byte1 = 0x04;
  u4lb_e1cb_e2b9(0);                                 /* e2b9 */
  return 1;
}

/* ea7c: CC-orientation PHY CL bit2 program (C2CB/C34B). sel==0x0F set bit2 else clear. */
static void u4lb_ea7c(uint8_t sel, uint8_t cc) {
  uint8_t idx = (REG_PHY_VENDOR_CTRL_C6DB & 0x01) ? (uint8_t)((cc + 1) & 0x01) : cc;
  uint16_t reg = (idx == 0) ? 0xC2CB : 0xC34B;
  if (sel == 0x0F) PR(reg) = (uint8_t)((PR(reg) & 0xFB) | 0x04);
  else             PR(reg) = (uint8_t)(PR(reg) & 0xFB);
}

/* 8992: per-lane SB lane-arm (SB[0x15]=v; SB[0x0C]=(.&0x80)|3; d5da(1)). */
static void u4lb_8992(uint8_t v) {
  SB_WR(0x15, v);
  SB_WR(0x0C, (uint8_t)((SB_RD(0x0C) & 0x80) | 0x03));
  u4lb_d5da(1);
}

/* lane gate: walk lane L iff 0x0819.L (lane-present mask = 1<<lane). */
/* Stock 8000 head (8006-801e): ROM[0x21AD]=0x19 -> 96a7 reads XDATA[0x0819]=work[0x19] (=R5);
 * 9a11 sets A=1, R0 = (lane_counter 0x21)+1; then `SJMP 8015` jumps straight to `DJNZ R0,8010`
 * which executes the RLC-left body (R0-1) times (the initial SJMP skips one body pass). Net shift
 * of A=1 is therefore `lane`, NOT `lane+1`: lane0 mask = 1<<0 = 0x01 (bit0), lane1 mask = 1<<1 =
 * 0x02 (bit1). (801e ANL/ORL/JNZ then gates on work[0x19] & mask.) With work[0x19]=0x03 BOTH lanes
 * walk -> the stock 2-lane bond (work[0x081C]=work[0x081D]=0x7B, host posts 2487 lane0=24/lane1=87,
 * SB[0xA0]=SB[0xA1]->0x02). The earlier 1<<(lane+1) read mis-counted the DJNZ-after-SJMP loop and
 * forced a lane0-only walk (work[0x081D]=0x00) so the host never 2-lane bonded. */
static uint8_t u4lb_lane_gate(uint8_t lane) { return (uint8_t)(u4_work_buf[0x19] & (uint8_t)(1u << lane)); }

/* 8501: a synchronous PHY CC10-mailbox command+wait/ack (NOT a no-op). Stock 8501 -> e50d/e8ef/e80a:
 * e8ef CC11=4 then 2 (reset); e50d CC10=(CC10&0xF8)|0x02 (opcode 2), CC12=0x00, CC13=0x65, CC11=1
 * (trigger); e80a busy-waits JNB CC11.1 then CC11=2 (ack). = phy_cc10_cmd_wait(2,0,0x65).
 * CC13=0x65 = R5: stock 8501 does `MOV R5,#0x65`; e50d's `MOV R7,0x05` is a DIRECT-ADDRESS operand
 * (SFR 0x05 == R5 of bank0), so R7:=R5=0x65 -> CC13. (Prior transcription misread it as immediate #5.)
 * Fired by stock after each LOOP1/LOOP2 selector==0 step to commit the staged cl_cfg to the PHY. */
static void u4lb_8501(void) { phy_cc10_cmd_wait(2, 0, 0x65); }

/* 81d4 finalize: width-settle -> advance state cell 0x0759+lane to 0x60; on counter overflow
 * (>=0x10) reset to 0x00. Composes the device TX[2:3] CL-walk value into work[0x081C+lane]: the low
 * nibble walks the lane-descriptor table (sb_lane_desc) with the high nibble PRESERVED, then bit7 is
 * latched into the SAME cell. Both writes target work[0x1C+lane] (verified byte-true vs Ghidra):
 *   8211  (9900, A=R6=0x1C+lane):  work[0x1C+lane] = (work[0x1C+lane]&0xF0) | sb_lane_desc[walk_idx]
 *   8212-821a (A=lane+R7, R7=ROM[0x21ae]=0x1C; 96a7 read @0x0800+0x1C+lane; |0x80; 96d6 write back):
 *            work[0x1C+lane] |= 0x80           <-- SAME cell as write#1 (R7 is 0x1C, not walk_idx).
 * NOTE: this is NOT a per-cl_idx "advertise slot" write -- the 0x8B/0xCB bit6 on 0x077B is HOST-driven
 * (eaac DMA-copies it verbatim from the RX plane; the device only TESTS (snap&0xC0)==0xC0 at 8262).
 * The device earns bit6 upstream via the primary-lane orientation commit (cm_RXCM_handler cc86:
 * C2C3.0/C343.0 + phy_lane_gate=0x0AB3), which handmade still omits. */
static void u4lb_lp1_finalize(uint8_t lane) {
  __xdata uint8_t walk_idx;
  if (lb_settle_counter[lane] >= 0x10) { lb_loop1_state[lane] = LP1_PARKED; return; }
  lb_settle_counter[lane]++;
  walk_idx = (uint8_t)((lb_lane_desc_idx[lane] + 1) & 0x0F);
  lb_lane_desc_idx[lane] = walk_idx;
  u4_work_buf[0x1C + lane] = (uint8_t)((u4_work_buf[0x1C + lane] & 0xF0) | sb_lane_desc[(uint16_t)(0x0 + walk_idx)]);
  /* 8212-821a write#2: `MOV A,0x21(lane); ADD A,R7; 96a7(ADD#0)->DPTR=0x0800+A; |0x80; 96d6 MOVX@DPTR`.
   * R7 here is 0x1C (set by 969e@81f8 MOV R7,A, A=ROM[0x21ae]=0x1C) -- NOT walk_idx (that R7 from 81f2
   * was clobbered by 969e). So target = work[0x1C+lane], the SAME cell as write#1. Verified byte-true
   * vs the CODE_BANK1::8212-821a + 96a7/96d6 disassembly. A prior "advertise-slot" edit that wrote
   * work[lane+walk_idx] was a misread of R7 and a REGRESSION (scribbled 0x80 into the DROM shadow). */
  u4_work_buf[0x1C + lane] |= 0x80;
  lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
}

/* 8174 width-settle poll: advance to 0x60 (via finalize) when the negotiated width pair has settled
 * vs the read-only CCE4:CCE5 counter; else leave state 0x40 to retry. */
static void u4lb_lp1_width_settle(uint8_t lane) {
  __xdata uint16_t widthA, neg;
  widthA = (uint16_t)(((uint16_t)lb_width_pairA[2*lane] << 8) | lb_width_pairA[0x1 + 2*lane]);
  if (widthA == 0) { u4lb_lp1_finalize(lane); return; }
  neg = (uint16_t)(((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO);
  if (lb_width_pairB[2*lane] == 0 && lb_width_pairB[0x1 + 2*lane] == u4_lane_train_trigger) {
    if ((uint16_t)(widthA - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
    return;
  }
  if ((uint16_t)((widthA - 1) - neg) >= 0x00C8) { u4lb_lp1_finalize(lane); return; }
}

/* stock 8000: the e461-bearing dispatches (84f3/84fa @8069/8251/831a/83bd/84a3) advance on the
 * e461 PUSH RESULT. 84f3/84fa = `LCALL e461; MOV A,R7; XRL #1; RET`. e461 CLOBBERS R7 -- it sets
 * R7=#0 when the 0x0719 in-flight token is set (e467, early RET) and R7=#1 on every push (e4a3),
 * so 84f3 returns push^1 and the caller's JZ advances exactly when e461 ISSUED A PUSH. That is
 * `if (u4lb_e461()==1)`. (The Ghidra decompile `return param_1^1` is an ARTIFACT: param_1 maps to
	 * R7, which e461 overwrites -- verified against the e461/e467/e4a3 disassembly.)
 * A prior session mis-read this as a 1<<lane lane-status gate, which killed lane1 (mask 0x02 != 1);
 * reverted to the byte-true e461-result gate. */

/* 8000: primary state-5 walker (0x0718==4). LOOP1 state@0x0759+lane (connect-arm/retrain edges),
 * LOOP2 state@0x075B+lane (the CL-state walk). LOOP1 dispatches through the 0def jump table @802a
 * = [target_hi,target_lo,match]: 0x10->804f 0x20->8069 0x30->807a 0x40->80ca 0x50->80d7 0x60->8251
 * 0x70->8262 0x80->82d5 0x90->82fa 0xA0->831a 0xA1->8327 default->8355. Linear chain from the b0b4
 * seed (0x10): 0x10->0x20->0x30->0x40->0x50(finalize: one TX[2:3] walk step)->0x60->0x70->0x80->
 * 0x90->0xA0->0xA1; the host re-train (8327 snap&0xC0==0x80) re-enters 0x50 for each walk step. */
static void u4lb_walk_8000(void) {
  __xdata uint8_t lane, state;
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop1_state[lane];

    if (state == LP1_WIDTH_INIT) {
      /* 804f: width-settle init (3x mask on work[0x081C+lane]) -> 0x20 (unconditional, no e461). */
      u4_work_buf[0x1C + lane] &= 0xEF;
      u4_work_buf[0x1C + lane] &= 0x7F;
      u4_work_buf[0x1C + lane] &= 0xDF;
      lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
    }
    else if (state == LP1_ARM_WAIT_PUSH) {
      /* 8069 LCALL 84f3: advance on the e461 push result (R7=1 push / 0 in-flight). */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = LP1_LANE_PRESENT_SEL;
    }
    else if (state == LP1_LANE_PRESENT_SEL) {
      /* 807a: route-special selector; on snap.7 set, arm the lane (SB[0x40]) + work[0x1C]|=0x20
       * -> 0x40, else stay 0x20. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0x80) == 0) {
          lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
        } else {
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          lb_width_pairA[2*lane] = 0x00;
          lb_width_pairA[0x1 + 2*lane] = 0x00;
          u4_work_buf[0x1C + lane] |= 0x20;
          lb_loop1_state[lane] = LP1_SETTLE_CLEAR;
        }
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_ARM_WAIT_PUSH;
      }
    }
    else if (state == LP1_SETTLE_CLEAR) {
      /* 80ca: clear width-settle counter -> 0x50. */
      lb_settle_counter[lane] = 0x00;
      lb_loop1_state[lane] = LP1_WIDTH_SETTLE_WALK;
    }
    else if (state == LP1_WIDTH_SETTLE_WALK) {
      /* 80d7: first entry (counter==0) -> width-settle/finalize = the TX[2:3] CL-walk step;
       * re-entry (ee6e && counter) -> work[0x1C]|=0x10|=0x40, reset state cell to 0x00. */
      if (u4lb_ee6e(lane) != 0 && lb_settle_counter[lane] != 0) {
        if (phy_lane_gate == 0) {
          SB_WR(lane ? 0x5A : 0x50, 0x01);
        }
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        lb_loop1_state[lane] = LP1_PARKED;
        if ((REG_PHY_ORIENT_C2C3 & 0x01) || (REG_VENDOR_CTRL_C343 & 0x01)) {
          if ((u4_work_buf[0x19] & 0x03) != 0) {
            /* 813c-816b: work_buf[0x1C] and [0x1D] are LANE-INDEPENDENT here (9997=0x1C, 9916=0x1D,
             * no lane add), each |=0x10 |=0x40 &=0x7F, and both LOOP1 state cells are cleared. */
            u4_work_buf[0x1C] |= 0x10; u4_work_buf[0x1C] |= 0x40; u4_work_buf[0x1C] &= 0x7F;
            u4_work_buf[0x1D] |= 0x10; u4_work_buf[0x1D] |= 0x40; u4_work_buf[0x1D] &= 0x7F;
            lb_loop1_state[0x0] = LP1_PARKED; lb_loop1_state[0x1] = LP1_PARKED;
          }
        }
      } else {
        u4lb_lp1_width_settle(lane);
      }
    }
    else if (state == LP1_BOND_WAIT_PUSH) {
      /* 8251 LCALL 84f3: advance on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = LP1_WIDTH_LATCH_SEL;
    }
    else if (state == LP1_WIDTH_LATCH_SEL) {
      /* 8262: snap&0xC0==0xC0 + low-nibble match -> arm width pairs -> 0x80, else stay 0x60. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x4 + lane];
        if ((snap & 0xC0) == 0xC0 &&
            (snap & 0x0F) == (uint8_t)(u4_work_buf[0x1C + lane] & 0x0F)) {
          if (phy_lane_gate) u4lb_e9e7();
          SB_WR(0x40, (uint8_t)(lane ? 2 : 1));
          /* 829b-82a5: pairA = the CCE4:CCE5 value RETURNED by ee57 (captured at ee57's internal read,
           * ee68-ee6b), NOT a fresh re-read — the live HW counter ticks between, skewing the settle. */
          { __xdata uint16_t w = u4lb_ee57();
            lb_width_pairA[2*lane] = (uint8_t)(w >> 8);
            lb_width_pairA[0x1 + 2*lane] = (uint8_t)w; }
          lb_width_pairB[2*lane] = 0x00;                         /* 82ae/82af pairB[2*lane]=0 */
          lb_width_pairB[0x1 + 2*lane] = u4_lane_train_trigger;  /* 82b1-82b2: 96d6 (MOVX @DPTR,A) writes
            * the trigger to pairB[1]; 96d7 then reloads DPTR=0x0759+lane (state cell), and 82b5-82b7
            * (#0x80) writes the STATE TRANSITION to LP1_FINALIZE_A (next line), NOT pairB. So pairB[1] =
            * trigger is byte-true. (A prior session + my own first read both misattributed the #0x80 to
            * pairB; disasm of 96d6/96d7 corrected it.) */
          lb_loop1_state[lane] = LP1_FINALIZE_A;
        } else {
          lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
        }
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_BOND_WAIT_PUSH;
      }
    }
    else if (state == LP1_FINALIZE_A) {
      /* 82d5: ee6e -> work[0x1C] |=0x10 |=0x40 &=0x7F (all gated) -> 0x90. */
      if (u4lb_ee6e(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
        u4_work_buf[0x1C + lane] &= 0x7F;
      }
      lb_loop1_state[lane] = LP1_FINALIZE_B;
    }
    else if (state == LP1_FINALIZE_B) {
      /* 82fa: ee6e -> work[0x1C] |=0x10 |=0x40; then work[0x1C] &=0x7F (unconditional) -> 0xA0. */
      if (u4lb_ee6e(lane)) {
        u4_work_buf[0x1C + lane] |= 0x10;
        u4_work_buf[0x1C + lane] |= 0x40;
      }
      u4_work_buf[0x1C + lane] &= 0x7F;
      lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
    }
    else if (state == LP1_BOND_WAIT_ACK) {
      /* 831a LCALL 84fa: advance to 0xA1 (lane bonded) on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop1_state[lane] = LP1_BONDED_MONITOR;
    }
    else if (state == LP1_BONDED_MONITOR) {
      /* 8327: bonded monitor; host re-train request (snap&0xC0==0x80) re-enters 0x50, else 0xA0. */
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((u4_host_desc[0x4 + lane] & 0xC0) == 0x80) lb_loop1_state[lane] = LP1_WIDTH_SETTLE_WALK;
        else                                    lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
        u4lb_8501();
      } else if (selector == 2) {
        lb_loop1_state[lane] = LP1_BOND_WAIT_ACK;
      }
    }
    /* default (state 0x00 / unmatched): stock 8355 tail is a no-op (lane parked). */
  }
  /* LOOP2: the CL-state walker (state @0x075B+lane). */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == LP2_CL_INIT) {
      u4_work_buf[0x1E + lane] |= 0x80;
      u4_work_buf[0x1E + lane] &= 0xBF;
      lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
    } else if (state == LP2_CL_PUSH_WAIT) {
      /* 83bd LCALL 84fa: advance on the e461 push result (waits here while 0x0719 in-flight, which
       * is the intended one-push-at-a-time serialization -- the cycle is unblocked by the host ack
       * clearing 0x0719 via eda0, not by forcing the advance). */
      if (u4lb_e461() == 1) lb_loop2_state[lane] = LP2_CL_EVAL;
    } else if (state == LP2_CL_EVAL) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        __xdata uint8_t snap = u4_host_desc[0x2 + lane];
        if ((snap >> 4) & 1) {
          lb_loop2_state[lane] = LP2_CL_IDLE;
        }
        else if (((snap >> 7) & 1) == 0) lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
        else {
          __xdata uint8_t cl_idx = (uint8_t)(snap & 0x0F), cap, cl_cfg_hi = 0, cl_cfg_lo = 0;
          lb_cl_value[lane] = cl_idx;
          u4_work_buf[0x1E + lane] &= 0xF0;
          u4_work_buf[0x1E + lane] |= cl_idx;
          u4_work_buf[0x1E + lane] |= 0x40;
          cap = phy_lane_cap[lane];
          if ((cap >> 1) & 1) cl_cfg_lo = lb_cap_field[cl_idx];
          if (cap & 1) { __xdata uint16_t m = (uint16_t)(sb_lane_flip[cl_idx] * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
          SB_WR(0x6A + 2 * lane, cl_cfg_hi);
          SB_WR(0x6B + 2 * lane, cl_cfg_lo);
          u4lb_ea7c(cl_idx, lane);
          lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
        }
        u4lb_8501();   /* 83e2/83ef/8493 -> 84d6 LCALL 8501: PHY commit after each 0x30 selector==0 step */
      } else if (selector == 2) lb_loop2_state[lane] = LP2_CL_PUSH_WAIT;
    } else if (state == LP2_CL_BOND_WAIT) {
      /* 84a3 LCALL e461; MOV A,R7; XRL #1; JNZ stay: advance on the e461 push result. */
      if (u4lb_e461() == 1) lb_loop2_state[lane] = LP2_CL_BOND_MON;
    } else if (state == LP2_CL_BOND_MON) {
      __xdata uint8_t selector = u4lb_eda0();
      if (selector == 0) {
        if ((u4_host_desc[0x2 + lane] >> 7) & 1) lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
        else { u4_work_buf[0x1E + lane] &= 0xBF; lb_loop2_state[lane] = LP2_CL_PUSH_WAIT; }
        u4lb_8501();   /* 84c4/84d0 -> 84d6 LCALL 8501: PHY commit after each 0x60 selector==0 step */
      } else if (selector == 2) lb_loop2_state[lane] = LP2_CL_BOND_WAIT;
    }
  }
}

/* 850b: alternate state-5 walker (0x0718 != 4). Dead on the live AMD path; kept for completeness. */
static void u4lb_walk_850b(void) {
  __xdata uint8_t lane, state, selector = 0;
  /* LOOP1: state @0x075B+lane. */
  for (lane = 0; lane < 2; lane++) {
    if (!u4lb_lane_gate(lane)) continue;
    state = lb_loop2_state[lane];
    if (state == 0x11) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) lb_loop2_state[lane] = 0x20;
      else if (r == 1) lb_loop2_state[lane] = 0x10;
    } else if (state == 0x20) {
      __xdata uint8_t r = u4lb_eda0(); selector = r;
      if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x30; else lb_loop2_state[lane] = 0x20; }
      else if (r != 2) lb_loop2_state[lane] = 0x20;
    } else if (state == 0x21) {
      if (u4lb_ee6e(lane) == 0) lb_loop2_state[lane] = 0x40;
    } else if (state == 0x30) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 4) & 1)) lb_loop2_state[lane] = 0x00;
        else if ((snap >> 7) & 1) { lb_cl_value[lane] = (uint8_t)(snap & 0x0F); lb_loop2_state[lane] = 0x50; }
        else lb_loop2_state[lane] = 0x30;
      }
    } else if (state == 0x40) {
      __xdata uint8_t cap = phy_lane_cap[lane], cl_idx = lb_cl_value[lane], cl_cfg_hi = 0, cl_cfg_lo = 0;
      if ((cap >> 1) & 1) cl_cfg_lo = lb_cap_field[cl_idx];
      if (cap & 1) { __xdata uint16_t m = (uint16_t)(sb_lane_flip[cl_idx] * 0x20); cl_cfg_lo |= (uint8_t)m; cl_cfg_hi |= (uint8_t)(m >> 8); }
      uart_puts(lane ? "\r\nL1:CL0 " : "\r\nL0:CL0 ");
      uart_puthex(cl_cfg_hi); uart_puthex(cl_cfg_lo);
      u4lb_ea7c(cl_idx, lane);
      lb_loop2_state[lane] = 0x51;
    } else if (state == 0x50) {
      lb_loop2_state[lane] = 0x60;
    } else if (state == 0x51) {
      __xdata uint8_t v;
      lb_cl0_width[lane] |= 0x80;
      v = (uint8_t)((lb_cl0_width[lane] & 0xF0) | lb_cl_value[lane]);
      lb_cl0_width[lane] = v;
      if (v == 0) lb_loop2_state[lane] = 0x61;
    } else if (state == 0x60) {
      if (u4lb_e461() == 1) { if (u4_host_desc[0x2] == 0) lb_loop2_state[lane] = 0x70; else lb_loop2_state[lane] = 0x60; }
    } else if (state == 0x61) {
      if (u4lb_e461() == 1) lb_loop2_state[lane] = 0x71;
    } else if (state == 0x70) {
      if (u4lb_e461() == 1) {
        __xdata uint8_t snap = lb_cl_status[lane];
        if (!((snap >> 7) & 1)) lb_loop2_state[lane] = 0x30;
        else if (lb_cl_value[lane] != 0x07) lb_loop2_state[lane] = 0x30;
        else lb_loop2_state[lane] = 0x70;
      }
    } else {
      if (u4lb_ee6e(lane) == 0) lb_loop2_state[lane] = 0x11;
    }
  }
  /* width-limit one-shot + LOOP2 (state @0x0759+lane). */
  if (lb_loop2_state[0x0] == 0 && lb_loop2_state[0x1] == 0) {
    if (lb_walk_oneshot_flag == 0) {
      if (u4_work_buf[0x19] & 0x01) u4lb_8992(0x86);
      if ((u4_work_buf[0x19] >> 1) & 0x01) u4lb_8992(0xA6);
      lb_walk_oneshot_flag = 1;
    }
    for (lane = 0; lane < 2; lane++) {
      if (!u4lb_lane_gate(lane)) continue;
      state = lb_loop1_state[lane];
      if (state == 0x10) {
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x21;
      } else if (state == 0x20) {
        if (u4lb_e461() == 1) {
          if ((lb_eq_status[lane] >> 7) & 1) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x30; }
          else lb_loop1_state[lane] = 0x20;
        }
      } else if (state == 0x21) {
        if (u4lb_eda0() != 0) u4_work_buf[0x1C + lane] |= 0x10;
        lb_loop1_state[lane] = 0x40;
      } else if (state == 0x30) {
        if (u4lb_eda0() != 0) {
          if (lb_settle_counter[lane] != 0) lb_loop1_state[lane] = 0x50;
          else { uart_puts("\r\n(lim)"); lb_loop1_state[lane] = 0x60; }
        }
      } else if (state == 0x40) {
        uart_puts("EQ");
        SB_WR(0x50, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1));
        lb_loop1_state[lane] = 0x51;
      } else if (state == 0x50) {
        lb_loop2_scratch[lane] |= 0x10;
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0x52;
      } else if (state == 0x51) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] != 0) lb_loop1_state[lane] = 0x51; else { uart_puts("\r\n(lim)"); lb_loop2_state[lane] = 0x00; } }
        else if (r != 2) lb_loop1_state[lane] = 0x51;
      } else if (state == 0x52) {
        lb_settle_counter[lane]++;
        lb_lane_desc_idx[lane] = (uint8_t)((lb_lane_desc_idx[lane] + 1) & 0x0F);
        lb_loop1_state[lane] = 0x70;
      } else if (state == 0x60) {
        __xdata uint8_t walk_idx = lb_lane_desc_idx[lane];
        sb_lane_desc[walk_idx] |= 0xA0;
        lb_loop2_scratch[lane] = walk_idx;
        if (walk_idx == 0) lb_loop1_state[lane] = 0x80;
      } else if (state == 0x70) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] == 0) { SB_WR(0x40, (uint8_t)(u4lb_ee6e(lane) ? 2 : 1)); lb_loop1_state[lane] = 0x90; } else lb_loop1_state[lane] = 0x70; }
        else if (r != 2) lb_loop1_state[lane] = 0x70;
      } else if (state == 0x80) {
        lb_loop1_state[lane] = 0xA0;
      } else if (state == 0x90) {
        lb_loop2_scratch[lane] &= 0x7F;
        if (u4lb_e461() == 1) lb_loop1_state[lane] = 0xA1;
      } else if (state == 0xA0) {
        __xdata uint8_t r = u4lb_eda0();
        if (r == 0) { if (u4_host_desc[0x2] == 0) lb_loop1_state[lane] = 0xB0; else lb_loop1_state[lane] = 0xA0; }
        else if (r != 2) lb_loop1_state[lane] = 0xA0;
      } else if (state == 0xA1) {
        if (u4lb_ee6e(lane) == 0) { uart_puts("\r\n(lim)"); lb_loop2_state[lane] = 0x60; }
        else lb_loop1_state[lane] = 0x50;
      } else {
        u4_work_buf[lane] &= 0xEF;
        u4_work_buf[lane] &= 0x7F;
        u4_work_buf[0x1C + lane] &= 0xDF;
        lb_settle_counter[lane] = 0x00;
        lb_loop1_state[lane] = 0x20;
      }
    }
  }
  (void)selector;
}

/* u4lb_state5(): e672 state-5 entry. 0x0718==4 -> 8000 else 850b. */
static void u4lb_state5(void) {
  DPX = 0x00;
  if (u4_route_enable_latch == 0x04) u4lb_walk_8000();
  else                    u4lb_walk_850b();
  DPX = 0x00;
}

/* e672 — the lane-bond FSM dispatcher, called from cb10's tail (gated 0x06ED!=0).
 *   3 -> cm_conn_routing_setup; 4 -> b0b4; 5 -> finalise when all sub-lane states clear, else the walker. */
static void u4lb_e672(void) {
  u4_fsm_state_t state = u4_fsm_state;
  if (state == U4FSM_LANE_TRAIN) {
    u4lb_state4_b0b4();
    return;
  }
  if (state == U4FSM_LANE_BOND) {
    if (lb_loop2_state[0x0] == 0 && lb_loop1_state[0x0] == 0 && lb_loop2_state[0x1] == 0 && lb_loop1_state[0x1] == 0) {
      u4lb_eb62(0, U4FSM_IDLE);
      return;
    }
    u4lb_state5();
    return;
  }
  if (state == U4FSM_CONN_ROUT) {
    u4lb_cm_conn_routing_setup();
    return;
  }
}

#endif /* USB4_LANEBOND_H */
