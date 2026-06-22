# b031 transport-reinit port — VERIFIED root cause + pseudocode (Phase-3, 2026-06-19)

Ground-up RE (wf_cb37fa99, Ghidra-verified). **ROOT CAUSE (verified):** handmade's bond-complete handler
`sb_lane_bond_complete_tunnel_up` (sb_router.h:322 = stock `CODE_BANK1::e52d`) SKIPS the entire `bank0 b031`
SB-transport / in-band control-adapter REINIT that stock runs as e52d's FIRST act. Consequence: the
control-adapter descriptor channel is never re-armed at bond-complete → **HW never raises EC06.0** → no
in-band Router-CS control packet ever lands → host "no switch found"; AND the link never finalizes/holds
(stock's `3578` downstream finalize → internal state 0x0F is replaced by the simplified `pcie_power_on`).

## EC06 path — RESOLVED (the two prior REs were each half-right)
EC06.0 (REG_NVME_EVENT_STATUS.0) is **HW-RAISED** (single xref: int1_isr @CODE:44e0 reads it as a status bit)
when an inbound in-band router-op packet lands in the EA80/EA9x aperture (gate EA90==0x5A); the Router-CS read
is then **FIRMWARE-SERVED** by c0a5 (movc @c0c5: E2 cfg-read → d945 → reply C805|=0x02 / C8B0<-0xEA). So
EC06=0 is NOT a missing e56f arm (boot-once @2fa8, correctly ported); HW never raises it because b031 never
brings the control channel up at bond-complete.

## Stock e52d call order (byte-true)
STOCK e52d (CODE_BANK1::e52d = sb_lane_bond_complete_tunnel_up), byte-true call/write order:

  e52d LCALL 0x05d4 -> bank0 CODE:b031  [the FULL SB-TRANSPORT-ENGINE REINIT, see below]
  e530 CLR A; e531 MOV R7,A; e532 LCALL 0x0598 -> bank0 CODE:e06b(R7=0)   [link_apply_lane_mask_reg3f + xdata_00b34=1 + sets 0xB1C from 0xA9F]
  e535 LCALL 0xb7a4 -> CODE_BANK1::sb_lane_descriptor_loader  [FULL DROM/lane re-seed + FSM RESET, see correction]
  e538 LCALL 0xeb0a -> CODE_BANK1::sb_enter_sleep  [conditional on d17e(): ee29/e26a(1,0)/d436(.,1,0xF)/u4_reinit_pending=1/d7cd/print "Sleep]"/e7ae]
  e53b MOV DPTR,#0x9fa; e53e MOVX A,@DPTR  [read XDATA 0x09FA = u4_route_mode]
  e53f JNB ACC.1,0xe55b   [GATE: skip the whole PCIe-downstream block unless 0x09FA bit1 set]
    e542 MOV DPTR,#0xb41; e545 MOVX A,@DPTR (read 0x0B41 = af38_t53/work_off)
    e546 JZ 0xe54d  [if af38_t53!=0:]
      e548 MOV R7,#2; e54a LCALL 0x04da -> bank0 CODE:e3b7(...,2)
    e54d MOV DPTR,#0xaef; e550 MOVX A,@DPTR (read 0x0AEF); e551 MOV R7,A
    e552 LCALL 0x3578 -> CODE:pcie_downstream_link_bringup(R7=0x0AEF)  [the REAL stock downstream-PCIe-to-GPU bring-up: B455=2,B455=4,B2D5=1,B296=8; cfg-TLP read @B220=0x1404600; B480&=~1 PERST deassert; poll B455.1; W1C B455=2; e8a9(1)/d436(0xF) -> state 0x0F linked x4]
    e555 LCALL 0x04fd -> bank0 CODE:e96c
    e558 LCALL 0x9a1c -> CODE_BANK1::sb_clr_7e8_set_b2f  [u4_connect_gate_e8(0x07E8)=0; u4_reinit_pending(0x0B2F)=1]
  e55b MOV DPTR,#0xca60; e55e MOVX A; e55f ANL A,#0xf7; e561 MOVX @DPTR  [REG_CPU_CTRL_CA60 &= ~0x08, clear tunnel clock/power bit]
  e562 MOV DPTR,#0xaf1; e565 MOVX A,@DPTR; e566 JNB ACC.0,0xe56e  [if 0x0AF1.0:]
    e569 MOV R7,#2; e56b LCALL 0x04f3 -> bank0 CODE:8a89(...,2)
  e56e RET

INSIDE bank0 b031 (the transport-engine reinit, executed FIRST every bond-complete) — r3_read/write_dispatch are SB-register (bank2) accesses: SB[0x14]&=~1; a3db(SB[0x12]); SB[0x12-1]&=~0x10; SB[0x4E]=2; SB[0x4C]&=~0x20; SB[0x4E]=4; SB[0x58]=1; SB[0x18] &=~0x02,&=~0x04,&=~0x08,&=~0x10 (4 staged writes via a35f); a358/a3db(SB[0x14]); SB[0x11]=1; SB[0x04]&=~0x02,&=~0x10; SB[0x18]|=0x04,|=0x08; SB[0x14]&=~1 then |=1; SB[0x08]=8; then the ENGINE CHAIN: FUN_CODE_e711 (intmem 0x35 = a301/a367 + r3 writes); clear_xdata_0b34_block4(); bank1_e5b0 (SB-descriptor engine); bank1_e8d6 (working-buffer seed 0x0994-0x09E3); sb_rom_descriptor_load; bank0_c17f; sb_d4cd_transport_edges; FUN_CODE_e4d2; phy_cc10_cmd; bank1_dcb4; bank0_e06b(0); u4lb_e74e.

## What makes the link HOLD (L2 finalize)
The theory's "TB_PORT_UP = a full transport reinit must run at bond-complete" is CORRECT in spirit and verified. The writes/latches that finalize and HOLD the bonded link, in stock e52d, are:

1. bank0 b031 (e52d's FIRST act, LCALL 0x05d4) is the transport-layer (in-band control-channel) reinit. It rebuilds the SB-transport descriptor engine via the chain e711 -> e5b0 (descriptor engine) -> e8d6 (working-buffer seed 0x0994-0x09E3) -> sb_rom_descriptor_load -> c17f -> sb_d4cd_transport_edges -> e4d2 -> phy_cc10_cmd -> dcb4 -> e06b(0) -> e74e. This is what brings the Transport/Control adapter (EC06) live: it re-arms the descriptor planes and the SB[0x18]/[0x14]/[0x08]/[0x58] transport-channel enables so the router-op mailbox (EC06.0) can carry in-band control traffic. THIS is the EC06=0 root cause — handmade never runs it.

2. e06b(0) (LCALL 0x0598, called BOTH standalone in e52d and again at the tail of b031): runs link_apply_lane_mask_reg3f(0x12) and sets xdata_0B34=1, plus 0xB1C from 0xA9F. This re-applies the lane mask to REG3F so the bonded lanes stay enabled.

3. pcie_downstream_link_bringup@3578 (LCALL 0x3578, gated 0x09FA.1): the actual downstream-link finalize that the host's "link is NOT up (state:1)" is waiting on — B455 LTSSM speed config, B220 cfg-TLP, B480 PERST deassert, poll-to-link, transition to internal state 0x0F (linked x4). This is the stock equivalent that holds the tunnel.

4. CA60 &= ~0x08 (the only write the handmade keeps) is the LAST step, not the mechanism.

So: yes, the bonded link is held by the e52d->b031 transport reinit (control channel/EC06) PLUS the e52d->3578 downstream finalize, both of which handmade omits. The "flaky, doesn't HOLD / state:1 retry" is consistent with handmade leaving the transport engine un-reinited after bond.

## L3 transport bring-up (the engine tail)
BYTE-TRUE stock bond-complete transport bring-up. The REAL bond handler is CODE_BANK1::e52d (the bank0 CODE:e529 same-named func is a mislabeled PD-queue stub — ignore it). bank1 e52d calls, in order: bank0_b031_stub_05d4 (-> bank0_b031, the transport engine), bank0_e06b_stub_0598(0), sb_lane_descriptor_loader, sb_enter_sleep, then (if u4_route_mode.1) af38_t53-gated e3b7, pcie_downstream_link_bringup, e96c, sb_clr_7e8_set_b2f; finally CA60 &= 0xF7 and (u4_connect_gate.1) 8a89.

The transport ENGINE is bank0_b031 (NOT e52d directly — theory had the nesting slightly off). b031 does a long r3_write_dispatch (banked-register, bank=2/page-1 control-adapter) descriptor program: clears 0x1406.0, a3db(0x4d), clears 0x12xx bit4, writes 0x4e=2, clears 0x4c bit5, 0x4e=4, 0x58=1, walks 0x1802 clearing bits 1/2/3/4, a358/a3db(0x14), 0x11=1, 0x04 clr bits1/4, 0x18 set bits2/3, 0x14 set bit0, 0x08=8. THEN the TAIL callees:
 - FUN_CODE_e711(8,...): reg0x35=(LIVE&0xC0); zero reg 0x3C/0x3D/0x3E/0x3F (clears the descriptor-engine commit regs).
 - clear_xdata_0b34_block4().
 - bank1_e5b0 (SB-descriptor ENGINE): scratch 0x097A=0; P12[0x4C]&=0xFE; P12[0x03]=0x80; P12[0x90]&=0xFB; P12[0x8F]=0x80; P12[0x90]&=0xDF; P12[0x8F]=0x20; clears notify flags 0x09F0-0x09F3.
 - bank1_e8d6 (working-buffer SEED): loops r3_write_dispatch(0) over XDATA 0x0994..0x09E3 (bank-adjusted), then 0x0995=6, 0x09DD=0x20, 0x09DC=0x10, 0x09E2=4, 0x09E0=6.
 - sb_rom_descriptor_load.
 - bank0_c17f: a2ff/a3eb then eng_a2df(4); seeds 0x0B36=0x20/0x0B37=4/0x0B38=2; c3ce commit; second pass eng_a2df(0x80), builds u4_routerop_desc0=0x18 (|4 if 0x081A.5, |0x20 if cap20g_gate1), 0x0B36=desc0, c3ce commit; if 0x0B12 a2c1(0x09DD|4)/0x0B35=4/lane-mask reg3f.
 - sb_d4cd_transport_edges: a2ff; eng_a348(|2)/a327(|3)/a2df(5); a2f9(0x40 @0x3D)/a31c(|8)/a2df(9); a2f9(4)/a31c(|2)/a2df(5); write 0x3D=0x40 + u4c_sb_desc_commit.
 - FUN_CODE_e4d2: a2ff; a348(|4); write &0x3F; a2df(0); write 0x3E=4 + commit.
 - phy_cc10_cmd (== the cbf8 transport-COMMIT a2ff/a344/a327/a2df engine): two descriptor-commit rounds via a344/a327/a2df/a2eb/a365/a3d2, final r3_write_dispatch + u4c_sb_desc_commit.
 - bank1_dcb4_stub_05f2, bank0_e06b(0), u4lb_e74e (0x0B1B=0; CCF8&=~0x10; CCF9=4; CCF9=2).

REGISTERS THAT ACTIVATE THE CONTROL-ADAPTER RX (so packets land in EA80/EA9x and raise EC06.0): the router-op engine is armed by bank1_e56f (stub 0x0494): EC00.0 toggled 0->disable, phy_cc10(0,0,9) settle, EC00.0=1 (ENABLE engine); EA88=100/EA89=0x24 (speed/gen 0x2464); EC04=1 (NVME_EVENT_ACK); EC05.0 cleared; C807 (INT_DMA_CTRL) bit6 cleared then bit7 set; mbox_state=IDLE. The aperture gate is EA90==0x5A. The RX itself only delivers packets once b031's page-1/P12 control-adapter descriptor channel above is live; e56f only arms the mailbox parser.

## Theory corrections (from Ghidra)
Corrections to the L2 theory from Ghidra ground truth:

1. THEORY: "e52d's first act is bank0_b031 whose TAIL = e711". WRONG. e711 is NOT b031's tail — e711 is an EARLY call inside b031 (FUN_CODE_e711 = intmem-0x35 a301/a367 setup). b031's actual TAIL chain is: e711, clear_0b34_block4, e5b0, e8d6, sb_rom_descriptor_load, c17f, d4cd, e4d2, phy_cc10_cmd, dcb4, e06b(0), e74e. (The theory's list of engine pieces e5b0/e8d6/rom_load/c17f/d4cd/e4d2/dcb4/e06b/e74e is otherwise correct, and they ARE the transport reinit — that part of the theory holds.)

2. THEORY implies sb_lane_descriptor_loader (b7a4) is a link-finalizer/holder. CORRECTION: b7a4 is a FULL FSM RESET, not a finalize-and-hold. It re-copies the DROM identity table (CODE 0x213d[0x64]->XDATA 0x0800), re-seeds the lane descriptor (0x21d4[0x10]), and RESETS state: u4_fsm_state=U4FSM_IDLE, sb_connect_present=0, sb_route_up_trigger=0, lb_lane_bonded_flag=0, sb_cdf5_substate_arm=0, plus SB[0x28]=8/SB[0x2A]=8/SB[0xC9] re-init/SB[0x66] etc. It re-arms the channel for the NEXT phase rather than "holding" the current bond. It is still required (handmade skips it), but its ROLE is re-arm-for-transport, not hold.

3. THEORY treats sb_enter_sleep (eb0a) as an unconditional reinit. CORRECTION: eb0a only does work if FUN_CODE_BANK1__d17e() returns nonzero; otherwise it's a no-op. Low priority.

4. THEORY: "handmade e52d calls ONLY sb_rom_descriptor_load + sets pending + clears CA60.8". CONFIRMED EXACTLY (sb_router.h:322-328).

5. THEORY: eed6/c593 might be part of the wall. CORRECTION: eed6 AND c593 are faithfully ported in handmade (byte-true to stock eed6 and bank0 CODE:c593). NOTE there are TWO c593's in stock: bank0 CODE:c593 (the P1[0x1335] tunnel-PHY commit that eed6 calls via stub 0x05c0, = handmade u4lb_c593, CORRECT) vs CODE_BANK1::c586 (a different PHY lane-rate commit, NOT the one eed6 calls). The handmade comment at sb_router.h:307-311 claiming c593 "SETS plane-2 P1[0x1335] bit1 = the adapter advertise the host CM needs" is approximately right (it does write P1[0x1335]), so eed6 is NOT the bug. The bug is isolated to e52d's missing b031 transport-engine reinit (-> EC06=0) and missing pcie_downstream finalize@3578.

NET: The theory's central L2->L3-seam claim is CORRECT and verified — handmade skips the entire bond-complete transport-engine reinit (b031) so the in-band control channel never comes up (EC06=0) and the link doesn't hold. The fix is to port stock e52d's b031 (and e06b(0)) transport reinit at bond-complete, not just sb_rom_descriptor_load in isolation.

CORRECTIONS to the L3 theory (from Ghidra, evidence-based):
1) FUNCTION ATTRIBUTION: The bond-complete handler is CODE_BANK1::e52d (bank1), NOT CODE:e52d. The bank0 function at CODE:e529 that Ghidra also names sb_lane_bond_complete_tunnel_up is a DIFFERENT, mislabeled PD-queue stub (boot_phy_dd42/e6d2/e478) — do not use it.
2) NESTING: e52d does NOT directly call e5b0/e8d6/cbf8/sb_rom_descriptor_load/c17f/d4cd/e4d2/e711. Its FIRST act is bank0_b031 (via stub 0x05d4), and bank0_b031 is the function whose TAIL calls e711, e5b0, e8d6, sb_rom_descriptor_load, c17f, d4cd, e4d2, cc10(=cbf8 a2ff/a344 engine), dcb4, e06b, e74e. e52d additionally calls e06b, sb_lane_descriptor_loader, sb_enter_sleep, pcie_downstream bring-up (route_mode.1), sb_clr_7e8_set_b2f, CA60&=0xF7, 8a89. Theory's chain was right; only the call nesting (b031 wraps the transport tail) needed fixing.
3) cbf8 == phy_cc10_cmd in Ghidra (the a2ff/a344/a327/a2df descriptor-commit engine) — confirmed, theory correct.
4) EC06 PATH (the part the two prior REs disagreed on): it is HYBRID. EC06.0 is HW-raised (single xref, the int1_isr reads it as a status bit); the host's Router-CS read is firmware-served by c0a5. Neither "pure silicon" nor "pure firmware" was right.
5) e56f (router-op arm) is NOT in the bond path and is NOT the fix — it is boot-once (@2fa8), already correctly ported in handmade (main.c:562). The memory note "refuted: re-arm e56f" is consistent: re-arming a boot-once engine does nothing; the real gap is HW never raising EC06.0 due to the missing b031 transport bring-up.
6) THE ACTIONABLE ROOT CAUSE (matches the theory's L2->L3 seam): port bank0_b031 (especially the e8d6 0x0994-0x09E3 working-buffer seed, e5b0 engine, c17f/d4cd/e4d2/cc10 descriptor commits, e711 zero-tail) into the handmade bond-complete handler at sb_router.h:322, plus sb_lane_descriptor_loader/sb_enter_sleep/sb_clr_7e8_set_b2f. The handmade e5b0 body is already correct (sb.h:309) but is only run at connect (sb_assert) — it must also run post-bond inside the b031 reinit, and e8d6 must be added (currently absent). Files: stock bank0_b031 @CODE:b031, bank1 e52d @CODE_BANK1::e52d; handmade /home/batman/asm2464pd-firmware/handmade/src/sb_router.h:322 + sb.h:309/288.

## Already in handmade (REUSE) vs MUST PORT
REUSE (byte-true bodies already present): u4c_e5b0 (sb.h:309 = bank1 e5b0 engine pre-config); u4c_sb_desc_commit (sb.h:288, with e711's engine-reg zero embedded); eng_a348/a327/a30c/a308/a31c/a2df helpers (sb.h); sb_rom_descriptor_load (sb.h:138 = b779); u4lb_c17f (usb4_lanebond.h:196 = c17f b36/b37/b38 + c3ce); u4lb_e74e (usb4_lanebond.h:619 = e74e CCF8/CCF9); u4lb_c3ce / e00c / a2c1 lane-mask path; the link_apply_lane_mask reg3f helper (usb4_lanebond.h:330+). The route_mode.1 gate, sb_tunnel_up_pending, and CA60&=0xF7 in handmade e52d (sb_router.h:322) are correct/kept. eed6 -> u4lb_c593 bond consequence (sb_router.h:313) is faithfully ported (NOT a divergence). usb4_routerop_init (e56f) is boot-once at main.c:562 (correct, not the fix). A deferred pcie_power_on() (main.c:91/239/535/705) exists as a SIMPLIFIED substitute for 3578.

MUST PORT (absent or wrong-context): bank0 b031 itself (the SB-register re-arm prelude + the standalone engine-tail orchestration) — ENTIRELY ABSENT at bond-complete. e8d6 working-buffer seed (zero 0x0994-0x09E3 + 5 seed bytes) — NO writer exists anywhere in handmade (grep-confirmed). bank0 d4c8 transport-edge ENGINE (distinct from the handmade bank1 d4cd poll that shares the name). e4d2 edge descriptor. cbf8/phy_cc10_cmd 2-round commit. clear_xdata_0b34_block4 (trivial). e06b(0) (the lane-mask reg3f re-apply + 0x0B34=1 + 0x0B1C, called twice). e711 as a STANDALONE engine-reset (handmade only has it embedded inside u4c_sb_desc_commit). sb_lane_descriptor_loader/b7a4 (full FSM re-arm) and sb_clr_7e8_set_b2f (0x07E8=0;0x0B2F=1) at bond-complete. sb_enter_sleep/eb0a (mostly no-op; low pri). Optionally the faithful 3578 downstream finalize to replace deferred pcie_power_on.

## PSEUDOCODE
// ============================================================================
// (1) REPLACE handmade sb_lane_bond_complete_tunnel_up (sb_router.h:322)
//     to mirror stock CODE_BANK1::e52d in byte-true call order.
// ============================================================================
sb_lane_bond_complete_tunnel_up():            // stock e52d
    b031_transport_reinit()                   // e52d LCALL 0x05d4  -> bank0 b031  [NEW, port below]
    e06b_apply_lane_mask(0)                   // e52d LCALL 0x0598  -> e06b(0)     [NEW helper below]
    sb_lane_descriptor_loader()               // e52d LCALL 0xb7a4  -> b7a4 FULL FSM RE-ARM [port/verify]
    sb_enter_sleep()                          // e52d LCALL 0xeb0a  -> eb0a (gated d17e(); usually no-op) [stub ok]
    if (u4_route_mode & 0x02) {               // e52d MOV 0x9FA; JNB ACC.1  (route_mode.1 gate)
        if (af38_t53 /*0x0B41*/ != 0)
            e3b7(... , 2)                      // e52d LCALL 0x04da (af38_t53-gated) [low pri]
        // OPTION A (faithful): port stock downstream finalize:
        pcie_downstream_link_bringup(0x0AEF)  // e52d LCALL 0x3578  [see (2) — real L3 finalize]
        e96c()                                // e52d LCALL 0x04fd  [tail housekeeping]
        sb_clr_7e8_set_b2f()                  // e52d LCALL 0x9a1c: 0x07E8=0; 0x0B2F=1
        // OPTION B (keep deferred): leave sb_tunnel_up_pending=1 so main.c runs pcie_power_on(),
        //   BUT this is NOT byte-true to 3578 and is the suspected reason the link doesn't reach
        //   internal state 0x0F. Prefer A once b031 fires EC06.
    }
    REG_CPU_CTRL_CA60 &= 0xF7;                 // e55b: clear tunnel clock/power bit (handmade keeps this)
    if (u4_connect_gate /*0x0AF1*/ & 0x01)
        8a89(... , 2)                          // e562-e56b gated 0x0AF1.0 [low pri]

// ============================================================================
// (2) NEW HELPER: b031_transport_reinit()  (stock bank0 CODE:b031)
//     ALL SB[..] accesses are bank=2 / page-1 control-adapter registers via
//     r3_read/r3_write dispatch (handmade has these as P12_* / SB bank-2 ops).
//     register = (value, regaddr, bank=2)
// ============================================================================
b031_transport_reinit():
    SBb2[0x14] &= ~0x01;                       // r3: clear bit0
    a3db(SBb2[0x12]);                          // read-modify helper on 0x12 (existing a3db semantics)
    SBb2[0x11 /*0x12-1*/] &= ~0x10;
    SBb2[0x4E] = 0x02;
    SBb2[0x4C] &= ~0x20;
    SBb2[0x4E] = 0x04;
    SBb2[0x58] = 0x01;
    // 0x18: 4 staged a35f writes clearing bits 1,2,3,4 in sequence:
    t = SBb2[0x18] & ~0x02; a35f(t,0x18);
    t &= ~0x04;             a35f(t,0x18);
    t &= ~0x08;             a35f(t,0x18);
    t &= ~0x10;             SBb2[0x18] = t;
    a358(0x14); a3db(SBb2[0x14]);
    SBb2[0x11] = 0x01;
    SBb2[0x04] = (SBb2[0x04] & ~0x02) & ~0x10;  // via a35f then write
    // 0x18: set bits 2,3:
    t = SBb2[0x18] & ~0x04 | 0x04; a35f(t,0x18);
    t = t & ~0x08 | 0x08;          a3db(t,0x18);
    SBb2[0x14] = (SBb2[0x14] & ~0x01) | 0x01;   // clear then set bit0
    SBb2[0x08] = 0x08;
    // ---- ENGINE TAIL (the part that brings the control adapter / EC06 live): ----
    e711_zero_engine_regs():                    // FUN_CODE_e711: a301(); reg0x35=(LIVE&0xC0); a367(); reg0x35=0; reg0x36=0
        SBb2[0x35] = SBb2[0x35] & 0xC0;          //   (handmade already embeds this in u4c_sb_desc_commit;
        SBb2[0x36] = 0;                          //    here it must run standalone as the engine reset)
    clear_xdata_0b34_block4():                   // 0x0B34=0;0x0B35=0;0x0B36=0;0x0B37=0
    u4c_e5b0();                                  // EXISTS sb.h:309 — descriptor-engine pre-config (REUSE)
    e8d6_seed_workbuf():                         // *** NEW, ABSENT in handmade ***
        for i in 0..0x4F: XDATA[0x0994+i] = 0;   //   zero 0x0994..0x09E3 (via r3_write_dispatch page=1, A=0)
        XDATA[0x0995] = 0x06;
        XDATA[0x09DD] = 0x20;
        XDATA[0x09DC] = 0x10;
        XDATA[0x09E2] = 0x04;
        XDATA[0x09E0] = 0x06;
    sb_rom_descriptor_load();                    // EXISTS sb.h:138 (REUSE)
    u4lb_c17f();                                 // EXISTS usb4_lanebond.h:196 (REUSE — b36/b37/b38 + c3ce x2)
    d4c8_transport_edge_engine():                // *** NEW *** (NOT the handmade sb_d4cd poll!):
        a2ff(); t=desc&0xF0|2; eng_a348(t,..); eng_a327(t&0xC0|3,..); eng_a2df(5,..);
        a2f9(0x40 @reg0x3D); eng_a31c(|8); eng_a2df(9,..);
        a2f9(0x04 @reg0x3D); eng_a31c(|2); eng_a2df(5,..);
        SBb2[0x3D]=0x40; u4c_sb_desc_commit();
    e4d2_edge():                                 // *** NEW ***:
        a2ff(); t=desc&0xF0|4; eng_a348(t,..); SBb2[reg]&=0x3F; eng_a2df(0,..);
        SBb2[0x3E]=0x04; u4c_sb_desc_commit();
    cbf8_phy_cc10_commit():                       // *** NEW *** (== phy_cc10_cmd a2ff/a344/a327/a2df engine):
        a2ff(); a344(..); eng_a327(|1); eng_a2df(0x41,..); a2eb(); a344(..); eng_a327(|1);
        eng_a2df(0x42,..); a365(); b=a3d2(); a2f9(b,..); a344(b,..); eng_a327(b&0xC0|2,..);
        eng_a2df(0x41,..); a2eb(); a344(..); eng_a327(|2); eng_a2df(0x42,..);
        a365(); b=a3d2(); SBb2[addr]=b; u4c_sb_desc_commit();
    dcb4();                                       // bank1 dcb4 (port/verify — minor)
    e06b_apply_lane_mask(0);                       // SECOND e06b(0) (tail of b031)
    u4lb_e74e();                                   // EXISTS usb4_lanebond.h:619 (REUSE: cc_subdemux_src=0; CCF8&=~0x10; CCF9=4; CCF9=2)

// ============================================================================
//   e06b_apply_lane_mask(v)  (stock bank0 e06b) — NEW small helper:
// ============================================================================
e06b_apply_lane_mask(v):
    u4_routerop_desc2 = v;
    bc57(intmem_0x35, 0x12, bank=2);              // applies reg0x35/0x12
    XDATA[0x0B34] = 1;
    link_apply_lane_mask_reg3f(v, 0x12, 2, v);    // re-apply lane mask to REG3F (handmade has reg3f helper)
    XDATA[0x0B1C] = (XDATA[0x0A9F] != 0) ? 1 : 0;

// ============================================================================
// (3) ORDERING / PLACEMENT CONSTRAINTS
// ============================================================================
// - b031 MUST run as e52d's FIRST act, BEFORE any downstream PCIe bring-up. The control-adapter
//   descriptor planes it re-arms are what let HW raise EC06.0; if pcie bring-up runs first the
//   control channel is still dead.
// - e8d6 zero-then-seed MUST run AFTER u4c_e5b0 pre-config and BEFORE sb_rom_descriptor_load/c17f
//   (the commits read 0x09DD/0x09E0/0x09E2). Order inside b031 is: e711 -> clear_0b34 -> e5b0 ->
//   e8d6 -> rom_load -> c17f -> d4c8 -> e4d2 -> cbf8 -> dcb4 -> e06b(0) -> e74e. Do NOT reorder.
// - CAUTION: 0x0998-0x099B (sb_routerop_hdr0-3) and 0x099C+ (sb_routerop_body) OVERLAP the e8d6
//   0x0994-0x09E3 window. e8d6 zeroes them at bond-complete; that is correct stock behavior (the
//   router-op header/body are runtime-rebuilt later by ce72/ce99 on the FIRST router-op). Verify no
//   handmade code reads 0x0998-0x099B between bond-complete and the first router-op.
// - The handmade name `sb_d4cd_transport_edges` (sb_router.h:216) is the bank1 d4cd EVENT POLL and
//   must NOT be reused for b031's d4c8 descriptor-edge ENGINE. Add the d4c8 engine under a new name
//   (e.g. u4c_d4c8_edge_engine) to avoid the collision.
// - CA60 &= ~0x08 stays LAST (it is the final step, not the mechanism).
// - For the downstream finalize, prefer porting 3578 (B455/B220-cfgTLP=0x1404600/PERST/poll/state
//   0x0F) over the deferred pcie_power_on() once EC06 fires; pcie_power_on lacks B455 LTSSM staging,
//   the B220 cfg-TLP, and the e8a9/d436 -> state 0x0F width finalize that makes the link HOLD.

## HW checks (per layer)
L2/control-adapter (b031 + e8d6): EC06 (REG_NVME_EVENT_STATUS.0) should transition 0->1 for the first time after bond-complete — it is HW-raised only once the b031 page-1/P12 control-adapter descriptor channel is live and an in-band router-op packet lands in the EA80/EA9x aperture (gate EA90==0x5A). Watch int1_isr @CODE:44e0 path firing -> c0a5 mailbox. If EC06 stays 0, b031's SB[0x18]/[0x14]/[0x08]/[0x58] re-arm or the e8d6 seed didn't take. Add a one-shot UART print on first EC06.0==1 (change-gated, not per-loop, to avoid UART saturation). L2-hold: device UART should show [SB P00] CL0 stay bonded (SB[0xA0]/[0xA1] low-nibble hold 0x02, route-ID 0104 6324) WITHOUT the cycle-variable drop; host ctl.c dyndbg should stop printing "link is NOT up (state:1), retrying" / "failed to reach TB_PORT_UP". L3-downstream (3578 or pcie_power_on): internal link state should reach 0x0F (stock linked x4) rather than only LTSSM 0x78; host should then emit 0xE8 PcieTunnel-Deassert -> device UART [PcieTunnel-Deassert] -> Lane Bonded -> PcieLinkUp -> Enable. L4: host driver reads "USB4 Switch:174c:2463" -> "new device found 1-1"; final success = 1002:7590 GPU in lspci.

## Remaining uncertainties
1) e8d6's loop write uses r3_write_dispatch with the "bank/page=1" arg (LCALL 0x0be6, R3=1) rather than a plain MOVX — verified the EFFECT is XDATA 0x0994+i=0 (the tail 5 writes are plain MOVX to those same XDATA cells), but if r3_write_dispatch page=1 routes 0x0994 to a different physical plane than the plain MOVX seed bytes, a naive handmade `XDATA[0x0994+i]=0` loop could target the wrong page. Verify the dispatch page semantics for the 0x09xx range before porting (likely identical to plain XDATA here, but confirm on HW). 2) Whether OPTION-A (port 3578) vs OPTION-B (keep deferred pcie_power_on) is needed for HOLD: the theory's strong claim is b031 alone fixes EC06 and that 3578-vs-pcie_power_on is the separate "doesn't reach state 0x0F" issue. It is plausible that once EC06 fires and the host gets a live control channel, the existing pcie_power_on suffices; recommend porting b031 FIRST and observing EC06+hold before also porting 3578. 3) b031's exact SB[0x04]/[0x18] staged-write order (a35f vs a3db RMW nuances) is transcribed from decompile; recommend a disassembly diff of CODE:b031 against the ported C to guarantee byte-true write count/order (the decompiler's CONCAT11 pointer arithmetic obscures a few reg targets). 4) sb_lane_descriptor_loader/b7a4 is a FULL FSM reset (resets u4_fsm_state, sb_connect_present, lb_lane_bonded_flag, etc.) — porting it at bond-complete is stock-faithful but could re-arm state that the handmade super-loop assumes is stable; watch for a bond-then-immediately-reset oscillation and gate/verify against stock if it appears. 5) e56f re-arm and eef9 etc. were already refuted; this analysis does not re-propose them.

## Address-collision notes (XDATA buffer placement — user-flagged)
- e8d6 working buffer 0x0994-0x09E3 OVERLAPS handmade sb_routerop_hdr0-3 (0x0998-0x099B) + body (0x099C). e8d6
  zeroing them at bond-complete is CORRECT stock behavior (router-op hdr/body are runtime-rebuilt on the first
  router-op). 0x09DD=0x20 is the correct seed of what handmade calls u4lb_lane_active_flags. So 0x09xx is NOT a
  conflict — it's the missing seed.
- 0x0AAD COLLISION: stock d945 stages the Router-CS READ RESPONSE there; handmade declared sb_fsm_state @0x0AAD.
  This bites ONLY when porting the c0a5/d945 response path (a LATER step, after EC06 fires) — relocate sb_fsm_state
  off 0x0AAD before porting d945. Not relevant to b031 itself.
- NAME COLLISION: handmade `sb_d4cd_transport_edges` (sb_router.h:216) is the bank1 d4cd EVENT POLL, NOT stock
  b031's `d4c8` descriptor-edge ENGINE. Port d4c8 under a new name.

---
## 2026-06-19 PORT STATUS — b031 in place + RUNNING, but EC06 still 0 (descriptor-engine transcription)
- b031 ported (sb.h engines + usb4_lanebond.h u4lb_b031_transport_reinit) and CALLED at the UNIVERSAL
  bond-complete point (main.c deferred tunnel-up block, before pcie_power_on) so it runs once per bond
  regardless of which path set sb_tunnel_up_pending (handmade has THREE: e52d sb_router.h:598 +
  c9a8/8a89 shortcuts usb4_connect.h:186/196). Confirmed via `[b031 EN ec06=..]` UART marker.
- HW: b031 runs every bond (`[b031 EN ec06=00]`), bond completes (A=0202, 779=3C3D), PCIe 78 — but
  EC06 STILL 0, host STILL "no switch found". So the b031 prelude is fine but the descriptor-engine
  tail (d4c8/e4d2/cbf8) is NOT byte-true.
- ROOT of the remaining gap = the eng-helper R1/A threading (DISASSEMBLED, definitive):
  - a2ff = `R1=0x34` (just sets R1; the following ANL A uses a fresh P12_RD(0x34) in the handmade idiom).
  - a348 = write[R1]; INC R1; read[R1] -> A.   (R1 ENDS +1; A = readback of new R1)
  - a327 = write[R1]; read[R1]; A=(A&0x3F)|0x40; write[R1].   (R1 UNCHANGED)
  - a2df = write[R1]; INC R1; read[R1]; A=(A&0xE0); write[R1].   (R1 ENDS +1)
  - a2f9 = write[R1]; LJMP e7fb(=u4c_sb_desc_commit).   (R1 UNCHANGED; A = commit's return)
  - a31c = write[R1]; INC R1; read[R1]; A=(A&0xC0)|0x04.   (R1 ENDS +1; returns A, no final write)
  - a30c = write[R1]; INC R1.   (R1 ENDS +1)
- d4c8 correct R1 thread (R1 from a2ff=0x34): a348@0x34->R1=0x35; a327@0x35; INC->0x36; a2df@0x36->R1=0x37;
  R1=0x3D; a2f9@0x3D; a31c@0x3D->R1=0x3E; INC->0x3F; a2df@0x3F->0x40; R1=0x3D; a2f9@0x3D; a31c@0x3D->0x3E;
  INC->0x3F; a2df@0x3F->0x40; R1=0x3D; write 0x3D=0x40; commit(e7fb).
  => MY PORTED d4c8 BUGS: a2df were @0x35/@0x3E/@0x3E, must be @0x36/@0x3F/@0x3F. Also the a31c input
  value uses the COMMIT's return A, not the literal a2f9 value (I used (0x40&0xF0)|8 — needs the threaded A).
- e4d2 + cbf8 need the SAME R1/A re-thread (cbf8 is the dense 2-round; a2df/a31c indices + commit-A all
  thread). dcb4's LJMP 0xd31e SB-TX/CRC engine is STILL STUBBED (the other EC06 suspect).
- NEXT (focused): re-transcribe d4c8/e4d2/cbf8 byte-true using the eng semantics above + DISASM-DIFF each
  against CODE:d4c8/e4d2/cbf8; then port dcb4->d31e. Verify HW = `[b031 EN ec06=..]` -> EC06 fires (the
  existing [EC06 ea90=..] diag in usb4.h) -> host reads "USB4 Switch: 174c:2463".

---
## 2026-06-22 ENGINE BYTE-TRUE COMPLETED + b031 PROVEN STRUCTURALLY-DOWNSTREAM (this session)
Re-disassembled CODE:d4c8/e4d2/cbf8 + every eng-helper (a2ff/a344/a327/a2df/a31c/a2f9/a2eb/a365/a3d2/
e711/e7fb) and DIFFED against the in-tree ports. Findings + fixes (sb.h):
- **d4c8 / e4d2 = byte-true** (confirmed against Ghidra decompile+disasm; the in-tree ports w/
  eng_a31c_exact are correct). No change needed.
- **u4c_sb_desc_commit (e711) BUG FIXED (LIVE PATH):** stock e711 zeros ONLY 0x3C and 0x3D
  (a367: R1=0x3c, write 0; INC->0x3d, write 0; ends — it does NOT touch 0x3E/0x3F). The in-tree
  commit ALSO zeroed 0x3E/0x3F, which poisons the next descriptor's 0x3E/0x3F whenever an engine
  commits without re-writing them (e.g. u4c_ccb3's 2nd/3rd sub-descriptors on the LIVE connect path
  via sb_assert->u4c_c270/d556). Removed the 0x3E/0x3F zeroing -> byte-true. e711 also leaves R1=0x3d
  on exit (load-bearing for cbf8 inter-block R1 drift).
- **u4c_cbf8_commit REWRITTEN byte-true:** the prior 4-round plain-write port was WRONG. Real cbf8 is
  a 2-round a2ff/a344/a327/a2df engine threading the a2eb (0x3c=CC 0x3d=CC 0x3e=08 +commit) and
  a365/a3d2 (0x3c=66 0x3d=66 0x3e=7B; a2f9 0x3f=01) data blocks, with each embedded commit leaving
  R1=0x3d so the FOLLOWING a327 writes 0x3d (NOT 0x35). Full R1-tracked trace in sb.h. Added
  u4c_a2eb_block / u4c_a365_a3d2_block helpers.
- **HW-TESTED (test.sh 30s):** the commit fix does NOT regress the bond (L0:CL0 02 / L1:CL0 02 /
  A=0202; host d1[CL0 CL0 ...] 128 samples). GPU still ABSENT; SFR dump at the bond:
  `C80A=00 EC06=00 EA90=00 C8C7=00 E302=83 CA06=01` -> the width event / in-band transport never wakes.
- **DECISIVE STRUCTURAL FINDING (closes the b031-as-trigger theory):** e52d's ONLY caller is
  sb_router_event_handler_M2 @CODE_BANK1::a0cb, gated `JNB ACC.0` on 989d() = `P1[0x0109].0`
  (R2=1,R1=9,R3=2 -> EXTMEM 0x10109). So in STOCK TOO, b031 (e52d's first act) runs ONLY after the
  host sets P1[0x0109].0 = the host's bond-COMMIT signal. The handmade gate (sb_router.h:723
  `if (P1_RD(0x0109)&1)`) is BYTE-TRUE. => b031 is structurally DOWNSTREAM of the host commit in both
  fw; it CANNOT cause the commit/width-event. Wiring the (now byte-true) b031 into e52d would be DEAD
  on handmade (the gate P1[0x0109].0 is host-withheld, never set -> the `66=00` wall). Forcing b031
  before the commit destabilizes the bond (Abr2, per prior sessions) and still doesn't raise C80A.4.
- **NET:** the descriptor-engine transcription is now byte-true (the task's deliverable), but completing
  it does NOT wake the transport — the wall is the host-withheld P1[0x0109].0 / C80A.4 width-event
  commit (consistent with project_inband_routercs_read_wall's ~25-agent host-coupled verdict), NOT an
  incomplete device-side engine. dcb4->d31e remains stubbed but is moot (same host-gated path). Engines
  kept as byte-true reference (linker-stripped = 0 binary cost); commit fix kept (live, non-regressing).
  Binary 0b4f093c (was fdb0d114; delta = the commit-fix removing 2 P12_WR on the live ccb3 path).
