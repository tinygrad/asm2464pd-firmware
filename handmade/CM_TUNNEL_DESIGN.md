# CM PCIe-tunnel layer — high-level pseudocode (stock), to port byte-correct

Goal: after the lane bond (CL0 CL0), drive the USB4 CM PCIe-tunnel so the host's
PCIe-Down adapter goes `Disabled -> L0` and the AMD GPU (1002:7590) enumerates.
The handmade omits this ENTIRE layer. Below is the stock structure (Ghidra-verified)
to reproduce byte-for-byte.

Go-signal: host `tbadapters` PCIeDn column `Disabled -> L0`; `tbtunnels` populates.

================================================================================
## 0. SUPER-LOOP CM-arm state machine  (stock main_boot_and_superloop 0x2fc0-0x301c)
================================================================================
```
// at TOP of while(1), gated: u4_entered_usb_mode(0x0AE2) not in {0, 0x10}
if (cm_sm_state /*0x0A59*/ == 0) {                 // DECIDE
    if (XDATA[0x0AE8]==0 && u4_route_mode/*0x09FA*/==4) {
        cm_sm_state   = 1;
        cm_step       = 0;        // 0x0B39
        cm_global_mode= 0xFF;     // 0x0002
        cm_arm_req    = 1;        // 0x06E6
        cm_e8e4_settle();         // = LCALL 0x04e9 (e8e4 -> c00d arm)
    } else cm_sm_state = 2;
}
if (cm_sm_state == 1) {                            // POLL
    if (cm_timer4_csr_fired()) {                   // e30e throttle (cc5d.1)
        r = cm_pcie_link_step_machine();           // 9037
        if (r != 0) { cm_sm_state = 2; cm_arm_req = 1; cm_arm_c00d(); }  // 0x045d
    }
}
// cm_sm_state==2 -> USB3-fallback (ca0d); not needed for the GPU path.
```
HANDMADE GUARD (deviation, mandatory): only allow state 0->1 when BOTH lanes are
CL0 already — `(SB[A0]&0x0F)==2 && (SB[A1]&0x0F)==2` — so c00d's PERST/RXPLL
re-drive never fires mid-bond. Hold in state 0 (do NOT fall to 2) until then.

================================================================================
## 1. cm_arm_c00d   (stock CODE:c00d) — the one-shot arm
================================================================================
```
if (cm_arm_req/*0x06E6*/ == 0) return;
cm_arm_req=0; XDATA[0x06E7]=1; cm_done/*0x06E8*/=1;
XDATA[0x05A7]=0; XDATA[0x06EB]=0; XDATA[0x05AC]=0; XDATA[0x05AD]=0; XDATA[0x0AF8]=0;
B401 |= 1; B401 &= ~1;                              // bit0 toggle
pcie_tunnel_adapter_enable_b401();                 // cd6c (EXISTS in boot_phy.h)
CA06 &= ~0x10;  B480 = (B480&~1)|1;                // PERST assert
e8a9(): C659 &= ~1;
boot_phy_d436_width(0x0F);                          // EXISTS
cm_substate/*0x06E5*/ = 0;
for (p=0x05B3; p!=0x06E5; p++) XDATA[p]=0;          // 0x132-byte per-port array clear
for (p=0x05A8; p!=0x05AC; p++) XDATA[p]=0;          // 4-byte
cm_pcie_link_init_state();                          // 39e4 (scalar parts; DMA TODO)
cm_ltssm_trigger/*0x05B4*/ = 0x10;                  // arm the link core
```

================================================================================
## 2. cm_pcie_link_init_state   (stock CODE:39e4) — RAM/descriptor reset
================================================================================
```
XDATA[0x044B]=0; XDATA[0x0000]=0;
C8D8 &= ~0x0C; C8D8 &= ~1; C8D7=0; C8D7 &= ~0x0C; C8D7 &= ~1;  // DMA scalar resets
XDATA[0x0579]=0; XDATA[0x0464]=0;
// TODO(later): DMA-descriptor-engine fills (175d/16c3/15ef) — MEMORY: breaks SPI DMA
for(i=0..3){ XDATA[0x044E+i]=0; [0x0452+i]=0; [0x0456+i]=0; [0x045A+i]=0; [0x0466+i]=0; }
for(i=0..3) XDATA[0x057A+i]=0;
while (op_len < (opcode<0x20 ? 0xFF:0)) { descRAM[opcode+0xB7]=0xFF; op_len++; }  // router-op mbox clear
XDATA[0x044D]=0;
```

================================================================================
## 3. cm_pcie_link_step_machine   (stock CODE:9037) — polled, per timer-4 tick
================================================================================
returns: 0 = still working; nonzero = done/error (caller -> state 2)
```
cm_step/*0x0B39*/++;

// --- restart-request phase ---
if (cm_restart_req/*0x0B3B*/) { cm_restart_req=0; cm_timed_phase/*0x0B3A*/=1; cm_e6fc_restart(); }
if (cm_timed_phase) {
    if (cm_step==0x15) { cm_e8e4_settle(); return cm_step; }
    if (cm_step <0x18) return (cm_step-0x18);    // still settling
    cm_timed_phase=0;
}

// --- LTSSM bring-up core (BYTE-EXACT, disasm 9074-909c) ---
if (cm_ltssm_trigger/*0x05B4*/ == 0x10) {
    phy_cc10_cmd_wait(4, 1, 0x2B);    // CC10=4,CC12=1,CC13=0x2B
    u4lb_a840(<R7 leftover>);          // pcie_link_speed_config_b403
    u4lb_e764_rxpll_train();           // R7=1
    cm_e8d9_c659();                    // C659 |= 1
    cm_ltssm_trigger = 0;
    phy_cc10_cmd_wait(4, 3, 0xE7);    // CC10=4,CC12=3,CC13=0xE7
}

// --- link-up poll ---
up = cm_link_up_check();              // e2a6: (0x07EF==0)&&(B432&7)==7&&(E765&2)
if (!up) {
    if (cm_step==0x0A) cm_restart_req=1;
    else if (cm_step==0x05) cm_tunnel_link_bringup_start();   // 92bb: 0x06E6=1; c00d
    if (cm_step < 0x1D) return (cm_step-0x1D);
    cm_done=0; return 0;              // timeout
}

// --- link UP: per-port adapter walk (Section 4) ---
if (cm_substate/*0x06E5*/ == 0) { ...per-port INIT (c5ff/a183/c874/c6d3, retry)... }
...per-port STATE WALK (Section 4)...
cm_status_latch/*0x044B*/ = cm_substate;
```

================================================================================
## 4. Per-port adapter walk  (stock 9037, the 0x06E5!=0 branch)  [STAGE 3]
================================================================================
Port array @ XDATA 0x00A8, indexed by INTMEM 0x21; per-port state byte cycles:
```
for each port p (0..cm_substate-1):
   load state = portArr[0xA8+p]
   switch(state):
     0x0F: done/skip
     0x02: e1ee();  if (R7!=0xFF) portArr = R7                 // status latch
     0x13: r=status_read(); if(r==0) portArr=0x12
     0x05: r=status_read(); if(r==0){ da13(); pcie_link_init_state(); c44e(3); portArr=0 }
     0x12: ab16();  portArr = (R7==0)? 0x03 : 0x0F
     0x03: e6a7();  if (R7==1){ portArr=1; 89db();
                       if (port==0) c44e(1),slot=0x0F else slot=4;
                       if (all ports==R7count){ status_latch=substate; e788(); dma_setup(); return } }
              else if (R7==0xFF) tunnel_link_bringup_start();
// then a 2nd pass counts ports in {0x0F,0x01} == BANK0_R7 -> done (0x06E8=0)
```
Callees to port byte-true (Stage 3): c5ff, a183(link_init_iter_scratch), c874, c6d3,
da13, ab16, e6a7, e788, pcie_link_init_state, set_intmem24_after_status_read(92b3),
e1ee, and the port-array helpers c441/c444/c445/c44e/c44f/c451/c496/c4a9, 89db(bank1).

================================================================================
## 5. small callees — byte-true (verified)
================================================================================
```
cm_phy ...      -> phy_cc10_cmd_wait(subcmd,cc12,cc13)  [EXISTS boot_phy.h]
e8a9            -> C659 &= ~1
e8d9            -> C659 = (C659&~1)|1            (arg.0 set; called with R7=1)
e5cb (hddpc)    -> if(!(C656&0x20)){ 0x06E6=1; C656|=0x20; C65B|=0x20; }
e6fc (restart)  -> C656 &= ~0x20; boot_phy_e57d_e764_reset_pulse(0x0F); 0x06E6=1; c00d();
e8e4 (settle)   -> u4_reinit_pending/*0x0B2F*/=0; e5cb(); c00d();
92bb (tun-start)-> 0x06E6=1; c00d();
e2a6 (link-up)  -> if(0x07EF) ret 0; return (B432&7)==7 && (E765&2);
e30e (timer4)   -> if(CC5D&1) ret0; if(CC5D&2){CC5D=4;CC5D=2;ret1} else {CC5D=4;CC5D=1;ret0}
```
EXISTS in handmade (reuse): pcie_tunnel_adapter_enable_b401, boot_phy_d436_width,
boot_phy_e57d_e764_reset_pulse, u4lb_a840, u4lb_e764_rxpll_train, phy_cc10_cmd_wait,
u4_route_mode(0x09FA), u4_entered_usb_mode(0x0AE2), u4_reinit_pending(0x0B2F),
u4_routerop_op_len(0x0AA3), u4_routerop_opcode(0x0AA4).

State cells to declare (all alias-checked FREE): 0x0002,0x044B-D,0x0464,0x0579,
0x05A6-A7,0x05A8[4],0x05AC-AD,0x05B4,0x06E5-E8,0x06EB,0x07EF,0x0A59,0x0B39-3C.

================================================================================
## Stages (smallest-first, each HW-testable; bond-safe)
================================================================================
S1: state vars + cm_arm_c00d + cm_pcie_link_init_state(scalar) + small callees. NOT wired. (build+bond OK)
S2: 9037 phases 1-4 (link-core + link-up poll, NO per-port walk) + super-loop a59 SM (CL0-guarded).
    Go-signal: [CMpoll] shows link-up (B432&7==7,E765&2), bond survives.
S3: the per-port adapter walk (Section 4) + its callees. Go-signal: host PCIeDn Disabled->L0.
S4: the 39e4 DMA-descriptor fills (only if needed) + the router-op CONFIG responder (c0a5).
    Go-signal: tbtunnels populates; 1002:7590 in lspci.
