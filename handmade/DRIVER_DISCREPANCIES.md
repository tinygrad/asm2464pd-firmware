# Handmade Driver Discrepancies — diff vs stock (Phase 2)

Cross-reference of `HANDMADE_DRIVER_MAP.md` against `STOCK_DRIVER_MAP.md`,
produced 2026-06-19 (9-mapper + completeness-critic workflow, Ghidra-verified).
**Completeness verdict: `minor-gaps`** — coverage is complete at the
whole-handler level; every known HIGH-structural wall is captured; the resolved
NVMe boundary is applied with **zero misclassifications**. Severity downgrades
and dedup from the critic are applied below.

**Totals:** 43 discrepancies — 6 HIGH · 18 MEDIUM · 19 LOW.

## Severity / kind legend
- **HIGH** = whole missing handler / dispatch / structural wall on the GPU path.
- **MEDIUM** = stub / partial / wrong-arm / omitted secondary path.
- **LOW** = byte-divergence / cosmetic / intentional handmade-only choice.

---

## THE WALL — prioritized HIGH-structural set (fix order)

The GPU tunnel stalls because the host's PCIe-tunnel config transport is never
serviced. In dependency order:

1. **`routerop-mailbox-stub`** — `cm_routerop_mailbox` (`usb4.h:63`) is a 🔶 stub:
   it acks `EA90` but never runs the `c0a5` movc dispatch (E0..E8), builds no
   reply (`C805|=0x02`), pushes no `C8B0<-0xEA`. → host posts `EC06=0`. **Root wall.**
2. **`missing-E2-E3-cfg-rw`** — the config R/W engine (`c0ef`/`c119` → `d945`/`cf5d`
   + working buffer `0x0B04/0x0B0A/0x0B08`) is absent. Without it the host's
   tunneled CfgRd/CfgWr get no completion → it stops enumerating the device router.
3. **`missing-E8-tunnel-reset`** — `c15f → e4a6` (`CC31.0` HW reset → `eec7`
   `[PcieTunnel-Deassert]` → `B480&=~0x0F` PERST deassert) is absent. This is the
   op that brings the GPU out of PERST; never reachable while #1 is a stub.
4. **`c5ff-dealias-not-lifted`** — the 4× CfgWr bridge de-alias exists only as
   DEBUG `cm_route_test` (3 of 4 writes, hardcoded bus0/dev0). Lift it into a
   GPU-path adb0 one-shot (see resolved boundary). *Necessary-but-not-sufficient.*
5. **`lp1-finalize-bit6-orientation-omitted`** — state-5 lane-bond CL-terminal /
   lane-orientation diverges (`usb4_lanebond.h:1102-1117`); the bond's final
   lane0 cl_idx-0 terminal is the residual §4 gap (host withholds `SB[A0]`).
6. **`eea5-cdr-rearm-missing`** — the superloop CDR/RX-PLL re-arm pass that feeds
   lane training is absent (`bank1_eea5`); may underlie #5's flakiness.

> **NVMe-INT-source caveat (critic):** the GPU router-op mailbox is delivered on
> the **NVMe-event INT source** (`int1_isr` → `usb4_int_demux` gates the EC06 path
> on `REG_NVME_EVENT_STATUS&0x01`). This is correct vs stock (`EC06.0→c0a5`) but
> means a future "mask the NVMe INT, it's NVMe-only" change would silently kill
> the GPU router-op path. Do **not** mask it.

---

## Full discrepancy table

| Sev | Kind | ID | Subsystem | Stock | Handmade | Summary |
|---|---|---|---|---|---|---|
| HIGH | stub | `routerop-mailbox-stub` | 2. ISR Handlers + 5. CM-Tunnel / Router-ops | cm_routerop_mailbox (CODE_BANK1::c0a5, trampol | usb4.h:63-88 ; handmade/src/usb4.h:63 | Stock c0a5 is the CM router-op config-TLP mailbox: gate EA90==0x5A, state 0x0B02, movc dispatch over opcodes E0..E8 (… |
| HIGH | missing-handler | `eea5-cdr-rearm-missing` | 3. Superloop | bank1_eea5 CODE_BANK1::eea5 | absent | Stock superloop runs eea5 (CDR/RX-PLL re-arm, 'RHMG') as a cb10-companion pass feeding lane training when phy_cdr_arm… |
| HIGH | partial | `lp1-finalize-bit6-orientation-omitted` | 4. USB4 State Machines | 8000/81d4 finalize CODE_BANK1::8211-821a depen | handmade/src/usb4_lanebond.h:1102-1117 | The LOOP1 81d4 finalize tests (snap&0xC0)==0xC0 at 0x70 but bit6 on host_desc 0x077B is HOST-driven and only earned u… |
| HIGH | partial | `c5ff-dealias-not-lifted` | 5. CM-Tunnel / Router-ops | route de-alias / bridge bus-num @ CODE:c5ff (4 | handmade/src/cm_tunnel.h:253 (cm_route_t | c5ff's 4x CfgWr-over-tunnel route de-alias is not ported as a live GPU-path one-shot. Only the DEBUG cm_route_test ex… |
| HIGH | missing-handler | `missing-E2-E3-cfg-rw` | 5. CM-Tunnel / Router-ops | routerop_op_E2_cfgread @ CODE_BANK1::c0ef / ro | absent | The router config-space READ (E2) and WRITE (E3) handlers plus their send-response (d945/cf5d), bounds (ceab), addr-c… |
| HIGH | missing-handler | `missing-E8-tunnel-reset` | 5. CM-Tunnel / Router-ops | routerop_op_E8_tunnelreset/tunnel_routerop_lin | absent | The 0xE8 PcieTunnel-Deassert/Enable op is entirely missing. e4a6 (C656&=~0x20, CA06&=~1, CC31.0=1 assert+spin, eec7 C… |
| MEDIUM | partial | `de16-d127-d894-dma-omitted` | 1. Boot / Init | bank0_de16 CODE:de16 / bank0_d127 CODE:d127 /  | absent | Stock boot_hw_init_main runs de16 (PHY-DMA cfg + sb_channel_connect_service), d127 (PCIe-DMA ring SIZE/BUF/DOORBELL/C… |
| MEDIUM | missing-handler | `eef9-int-enable-missing` | 1. Boot / Init | bank1_eef9 CODE:eef9 | absent | Stock boot calls bank1_eef9 which does INT_ENABLE\|=0x40 (bank1/USB4 INT group bit6). Handmade has no equivalent; usb… |
| MEDIUM | partial | `no-spi-fuse-override` | 1. Boot / Init | bank0_92c5_seed CODE:92c5 / usb4_cap_apply_09f | boot_phy.h:151 / main.c:496-516 | Stock 92c5 overrides lane-mask(0x0AE9)/lane-gen(0x0AEE)/width(0x086C-71) from OTP fuses when 0x707E==0x5A; 8d77 overr… |
| MEDIUM | missing-handler | `missing-c105-secadapter` | 2. ISR Handlers | usb4_sec_adapter_link_event_c80a4 (CODE:c105,  | usb4.h:98 | Stock services C80A.4 via c105 (secondary USB4 adapter / link-width events, page-1 1407/1603, linkwidth recovery + CC… |
| MEDIUM | missing-handler | `missing-cd10-cc33` | 2. ISR Handlers | bank0_cd10 (CODE:cd10, via int1_isr_orchestrat | absent | Stock int1_isr dispatches CC33.2 (W1C=4) to cd10 for PCIe-downstream link bringup/reset (CC31 spin = controlled soft-… |
| MEDIUM | partial | `tunnel-branch-w1c-only` | 2. ISR Handlers | tunnel_link_event_e763 (CODE_BANK1::e911, tram | usb4.h:108-113 | Stock C80A.0-3 branch calls e911: E763.2 -> [PcieTunnel-PcieLinkUp]+d17e, E763.3 -> PcieLinkDn, plus W1C. Handmade on… |
| MEDIUM | missing-handler | `dee3-lane-advance-drain-missing` | 3. Superloop | bank0_dee3 CODE:dee3 | absent | Stock 541f-gated dee3 drains sb_cb10_lane_advance: ==10 spin, ==0x0b path (cb23/cadf) + clears entered_usb_mode. Clas… |
| MEDIUM | partial | `pend-int-flag-never-set` | 3. Superloop | a5d8 CODE_BANK1::a5d8 (deferred responder gate | sb_router.h:552 / main.c:670 | sb_pend_int_pending (sb_router.h:552, __at 0x0B55) is never assigned 1 anywhere in the tree (only cleared at main.c:6… |
| MEDIUM | partial | `af38-sbtx1-width-residual` | 4. USB4 State Machines | sb_af38_descriptor_response CODE_BANK1::af38 | handmade/src/sb_router.h:72 (SBTX[1] at  | af38 builds SBTX[1] = desc_dir \| sb_width_lut[desc_type]. In THIS tree sb_width_lut@0x06F2 is correctly boot-seeded … |
| MEDIUM | partial | `e391-0776-gate-live-fire` | 4. USB4 State Machines | cm_conn_routing_setup CODE_BANK1::a7de/a869 (e | handmade/src/usb4_lanebond.h:250-273 | e391 width-LUT seed is ported byte-true but only fires when u4_coldboot_seed_gate(0x0776)==0. The seed is what gives … |
| MEDIUM | partial | `stale-cap20g-comment-0819` | 4. USB4 State Machines | cm_conn_routing_setup CODE_BANK1::a7de (0x0819 | handmade/src/usb4_lanebond.h:288-290 | The comment block asserts cap20g_gate1=0 -> 081A=0xE1 -> 0x0819=0x01 'matching stock'. This is FALSE and contradicts … |
| MEDIUM | stub | `substate-poll-noop-stub` | 4. USB4 State Machines | sb_transport_substate_poll CODE_BANK1::d4cd (s | handmade/src/sb_router.h:247 | sb_transport_substate_poll is a no-op stub; the comment claims all edges are handled by sb_d4cd_transport_edges. Stoc… |
| MEDIUM | partial | `adb0-engine-dead` | 5. CM-Tunnel / Router-ops | cm_adb0_tlp @ CODE:adb0 | handmade/src/cm_tunnel.h:174 | The general config-TLP issuer cm_adb0_tlp (and cm_c1f9, cm_e89d_read, cm_e91d_write, the cm_mbox_* helpers) are porte… |
| MEDIUM | missing-handler | `reply-trigger-missing` | 5. CM-Tunnel / Router-ops | cm_routerop_reply_trigger @ CODE_BANK1::cf35 ( | absent | Handmade acks router-ops by writing only EA90=0xA5; it never asserts the C805 send-response bit (cf35) nor arms the C… |
| MEDIUM | omitted-state-arm | `working-buffer-absent` | 5. CM-Tunnel / Router-ops | router-op working buffer 0x0B04/0x0B0A/0x0B08  | absent | The router-op working-buffer state machine (64-bit addr 0x0B04 vs limit 0x0B0A, write cursor 0x0B08/09, len=min(limit… |
| MEDIUM | missing-handler | `vdm-cmd5-enter-usb4-ack-absent` | 6. PD / VDM | vdm_enter_usb4_ack_sb_init CODE:ca71 (dispatch | vdm.h:233 (vdm_tx_dispatch switch; cmd5  | Stock 9ac4 routes VDM cmd 5 (Enter_USB4) to ca71 which ACKs and hands off to the USB4 SB/lane subsystem (sb_lane_flip… |
| MEDIUM | partial | `missing-b220-cap-read-x4-width` | 7. Local PCIe (device→GPU downstream LTSSM) | pcie_downstream_link_bringup CODE:3578 (B220=0 | main.c:91 | pcie_power_on omits the stock B220 PCIe-config-TLP capability read (0x1404600) and the d436 x4 lane-width finalize th… |
| MEDIUM | missing-handler | `tunnel-link-setup-8000-absent` | 7. Local PCIe (device→GPU downstream LTSSM) | pcie_tunnel_link_setup CODE:8000 (bank0) | absent | The bank0 tunnel data-path setup CODE:8000 (B220 config-TLP, B230/B234/B240/B244/B246 link params, read negotiated wi… |
| LOW | wrong-dispatch | `mode-flag-hardcoded-no-b1cb` | 1. Boot / Init | boot_usb4_vs_usb3_mode_decision CODE:b1cb | main.c:496 | Stock derives 0x09F9 through 4c40 default(0x04)->8d77 SPI map->b1cb (91C0&0x18)==0x10 decision. Handmade hard-sets 0x… |
| LOW | partial | `int0-nvme-tail-omitted` | 2. ISR Handlers | int0_isr body (CODE:0e5b) | main.c:374-459 | Stock int0_isr body has a C806.5 CPU-link + NVMe-queue service loop tail (5442/52a7/3419/180d/1196/488f/4784/49e9/3e8… |
| LOW | handmade-only | `deferred-8a89-nonstock-fallback` | 3. Superloop | no stock superloop counterpart (c9a8/8a89 re-e | main.c:680 | The deferred-8a89 block drives bank0_c9a8(0) once after a synthetic fsm_stall>=6 heuristic and no bond. In-tree comme… |
| LOW | missing-handler | `e2ec-541f-postconnect-reinit-missing` | 3. Superloop | bank0_e2ec CODE:e2ec / E716 guard CODE:541f | absent | Stock superloop runs e2ec (USB4 post-connect reinit, e869/e95f) under ae9!=0x0f, and 541f as the E716&3 link-up guard… |
| LOW | wrong-dispatch | `pd-tx-commit-relocated` | 3. Superloop | pd_tx_commit_engine CODE:e1c6 | pd_dispatch.h:108/117,vdm.h:185 (not mai | Stock commits staged PD TX as a per-pass superloop check on pd_tx_staged_pending!=0. Handmade calls pd_tx_commit_engi… |
| LOW | byte-divergence | `af38-plane-status-readsource` | 4. USB4 State Machines | sb_af38_descriptor_response CODE_BANK1::af38 ( | handmade/src/sb_router.h:76,94 (status_o | Stock af38 reads the per-port status from absolute page-1 0x2280D (port0) / 0x2280E (port1). Handmade reads SB(0x0D)/… |
| LOW | partial | `cb10-thinned-vs-stock` | 4. USB4 State Machines | CODE_BANK1::cb10 (full periodic tick: 9716/sb_ | handmade/src/sb_router.h:702 + main.c:63 | Stock cb10 is one function owning the SB[A0/A1] latch compare + ee57 throttle + e672 dispatch + cdf5 tail. Handmade s… |
| LOW | partial | `e391-loop2-deepphy-omitted` | 4. USB4 State Machines | cm_init_routing_tables CODE_BANK1::e391 (loop2 | handmade/src/sb.h:145 (sb_rom_descriptor | Stock e391 has TWO loops: loop1 seeds the width-LUT (0x06F2) + branch-A gate (0x0705) — ported into sb_rom_descriptor… |
| LOW | byte-divergence | `lp1-width-settle-clr-replaced-by-park` | 4. USB4 State Machines | 8000 LOOP1 0x50 step CODE_BANK1::811a (CLR wor | handmade/src/usb4_lanebond.h:1192-1218 | The stock 811a CLR of work_buf[0x1C+lane] (re-zeroing at 0x50 every tick) is deliberately replaced by an LP1 park, be… |
| LOW | misclassification | `c00d-9037-dead-correct` | 5. CM-Tunnel / Router-ops | cm_arm_c00d @ CODE:c00d / cm_pcie_link_step_ma | handmade/src/cm_tunnel.h:95 / :283 / :75 | The 9037/c00d CM step-machine + PERST arm is present (byte-true) but intentionally DEAD/uninvoked (removed from the l… |
| LOW | partial | `enter-mode-no-sb-handoff` | 6. PD / VDM | vdm_handle_enter_mode CODE:b966 + 9ac4 tail (c | vdm.h:117 vdm_handle_enter_mode (only la | Stock invokes sb_lane_flip_init after an Enter_Mode (cmd4) ACK (the cmd==4\|\|5 tail in 9ac4). Handmade vdm_handle_en… |
| LOW | missing-handler | `vdm-cmd6-exit-absent` | 6. PD / VDM | VDM Exit handler CODE:e973 (dispatched from vd | vdm.h:233 (cmd6 falls to default vdm_nak | Stock 9ac4 handles VDM cmd 6 (Exit_Mode) via e973 (and returns without the post-commit lane_flip). Handmade NAKs cmd … |
| LOW | stub | `cm-arm-c00d-deadweight` | 7. Local PCIe (device→GPU downstream LTSSM) | pcie_tunnel_link_bringup_start CODE:92bb -> ba | cm_tunnel.h:66 / cm_tunnel.h:95 | cm_tunnel_link_bringup_start (92bb) and cm_arm_c00d (c00d) are ported but drive the NVMe-harmful CM step-machine (B48… |
| LOW | byte-divergence | `divergent-linkup-gate` | 7. Local PCIe (device→GPU downstream LTSSM) | pcie_link_up_check_b432_e765 CODE:e2a6 | main.c:106 / cm_tunnel.h:41 | Two coexisting link-up criteria test different registers. pcie_power_on (main.c:106-119) gates CONNECTED on REG_PCIE_… |
| LOW | handmade-only | `pcie-poweron-replaces-3578` | 7. Local PCIe (device→GPU downstream LTSSM) | pcie_downstream_link_bringup CODE:3578 | main.c:91 | pcie_power_on is a custom downstream bring-up that wholesale replaces stock 3578. Stock ramps B455 LTSSM speed (2 the… |
| LOW | missing-handler | `link-mode-finalize-ca0d-absent` | 8. USB Device | bank0_ca0d CODE:ca0d | absent | Stock ca0d latches entered_usb_mode=0x10 to finalize USB link-mode. No direct handmade equivalent in the USB-device s… |
| LOW | partial | `ss-link-event-policy-0003-partial` | 8. USB Device | usb_ss_link_event_policy_0003 CODE:4532 | main.c:407-435 / main.c:381-389 | Stock's deferred 0x0003-bitmap link-event policy (b3→pcie-down bringup, b4→USB4 lane train, b6→TypeC recovery, b2→SS … |
| LOW | missing-handler | `ss-link-train-engine-missing` | 8. USB Device | usb_ss_link_train_engine CODE:9c2b | absent | Stock SS link-train engine (cc10 train via 91D1.3, SS-FAIL→force USB2) has no handmade port. int0_isr only reproduces… |
| LOW | handmade-only | `vendor-bulk-replaces-msc` | 8. USB Device | usb_msc_init CODE:4904 (and 4784/180d/3419/119 | main.c:348 handle_usb_bulk_data / main.c | The entire stock USB-MSC/NVMe storage data-path is intentionally replaced by a vendor BULK-EP interface (tinygrad). S… |

---

## Cross-section reconciliation (completeness critic)

Consistency fixes applied to the map (the gaps were in cross-section consistency, not coverage):

- GAP-1 width-LUT SEED IS DOUBLE-MAPPED AND DESCRIBED INCONSISTENTLY (most important cross-section issue). §1 and §4-FSM-TICK say the e391 seed lives ONLY in usb4_lanebond.h:260-265, connect-time-gated on 0x0776==0. §4-SB-EVENT says it was RESTRUCTURED to a boot-UNCONDITIONAL seed in sb_rom_descriptor_load (sb.h:145-148) that 'replaced' the gated copy. In the actual tree BOTH coexist: sb.h:146-147 seeds sb_width_lut/sb_branchA_gate unconditionally at boot from width_lut[]/branchA_gate[] (sb.h:131-136), AND usb4_lanebond.h:263-264 re-seeds the same arrays connect-time-gated from u4lb_width_lut_514c[]/u4lb_branchA_gate_515f[] (usb4_lanebond.h:50-55). I verified the two source tables are byte-identical (both ROM 0x514c/0x515f, idx0x0C=0x03), so the data is consistent — but the structural mapping is duplicated and three sections disagree on which copy is canonical.
- CONSEQUENCE OF THE DOUBLE-SEED: §4-FSM-TICK's HIGH discrepancy e391-0776-gate-live-fire (worry: 'if the 0x0776 gate never opens, af38 emits SBTX[1]=0x55 poison') is largely MOOT because the sb.h boot-unconditional seed already populates sb_width_lut before any connect, so af38 reads 0x03 regardless of the gate. §4-FSM-TICK and §1 did not account for the sb.h boot seed; §4-SB-EVENT did but mis-stated it as a replacement. The severity of e391-0776-gate-live-fire should be downgraded.
- sb_channel_connect_service has CONFLICTING stock addresses: §3 maps it to stock CODE:c7a5 (connect-channel poll); §4-SB-EVENT maps it to stock CODE_BANK1::c3b2 (a066 PART-1 per-port read). The handmade body (sb_router.h:332: edd9 ack + per-port SB[0x20/0x22]/[0xA4/0xA6] read + (~hi)==lo validate + (lo&0x0F) dispatch) matches the c3b2/a066-PART1 description, not c7a5. §3's c7a5 attribution is the looser one.
- sb_router_event_handler stock-counterpart conflict: §2 and §4-SB-EVENT map it to CODE_BANK1::a066; §3 maps it to 'CODE:c7a5 (sb_channel_connect_service) + cb10 conn-service'. a066 is the correct INT1 C80A.5 body; §3 conflated the handler with its callee c7a5.
- sb_cb10_lane_advance stock counterpart is given three slightly different scopes: §3 'cb10 monitor head only', §4-FSM-TICK 'CODE_BANK1::cb10 (full)' then notes the throttle/dispatch are hoisted to main.c, §4-SB-EVENT 'cb10 SB tail'. Consistent in spirit (cb10 is decomposed across sb_router.h + main.c) but the row-level stock-ref differs per section.
- cm_routerop_mailbox is mapped THREE times (§2 row, §5 row, plus §2/§5 state-machine rows) — all consistently 🔶 stub, but the per-section opcode detail differs: §2 says it acks E2/E3; §5 says it acks E2 in MULTIPKT_1 and E3 in MULTIPKT_2. Source (usb4.h:72-86) confirms §5's per-state split is the precise one.

**NVMe-classification audit:** NONE found that are wrong. The resolved boundary is applied correctly across §1/§3/§5/§7. Verified in Ghidra: c5ff (CODE:c5ff) is the SHARED route de-alias — it calls 9ee5(cap-walk)/d956(bus-num CfgWr)/e91d(route-bind CfgWr)/9a7f(mem-win)/writes B220 and sets per-port 0x05B4=2; §5 correctly classifies it SHARED-lift and flags it as NOT lifted into a live one-shot (only the DEBUG cm_route_test exists). a183 (CODE:a183) is genuinely NVMe-ONLY: a 32-iteration (DAT_INTMEM_25 0..0x1f) per-port enroll loop with db45 class-read + 0x06E5 per-port table + 0x05B4=2 marker — §5's NVMe-ONLY tag holds. e91d/d956/9ee5 are correctly SHARED (route-bind primitives reused by both c5ff SHARED and a183 NVMe).

---

## Per-item detail — HIGH & MEDIUM (description + fix)

### HIGH

#### `routerop-mailbox-stub` — stub (2. ISR Handlers + 5. CM-Tunnel / Router-ops)
- **Stock:** cm_routerop_mailbox (CODE_BANK1::c0a5, trampoline 0x0499)
- **Handmade:** usb4.h:63-88 ; handmade/src/usb4.h:63
- **What:** Stock c0a5 is the CM router-op config-TLP mailbox: gate EA90==0x5A, state 0x0B02, movc dispatch over opcodes E0..E8 (CfgRd/CfgWr over the tunnel, tunnel reset via e4a6), reply via C805|=0x02 with C8B0<-0xEA arming. Handmade cm_routerop_mailbox only latches EA80 into u4_routerop_mbox_opcode in IDLE and emits a bare EA90=0xA5 ack for E2/E3 continuation packets in MULTIPKT_1/2. NO movc dispatch table, NO per-opcode E0..E8 work, NO CfgRd/CfgWr, NO C805|=0x02 reply. This is the host-visibility gate (host posts EC06=0, never 0xE8) noted in section 0.
- **Fix:** Port the full c0a5 movc-table dispatcher (func_0def table, opcodes E0..E8, e4a6 tunnel-reset, d945/ceab/e21b/e2b9 helpers) and the C805|=0x02 / C8B0<-0xEA reply path so the device actually answers host router-ops. Lift c5ff's CfgWr de-alias into this engine per the resolved boundary.

#### `eea5-cdr-rearm-missing` — missing-handler (3. Superloop)
- **Stock:** bank1_eea5 CODE_BANK1::eea5
- **Handmade:** absent
- **What:** Stock superloop runs eea5 (CDR/RX-PLL re-arm, 'RHMG') as a cb10-companion pass feeding lane training when phy_cdr_arm_mask!=0. It is part of the §3 GPU-PATH minimal loop set. Handmade has NO eea5 equivalent anywhere (grep eea5 = empty). Lane training may rely on this CDR re-drive.
- **Fix:** Decompile CODE_BANK1::eea5, port it as a superloop pass (or into the FSM-advance block) gated on its phy_cdr_arm_mask condition; verify it is not already folded into u4lb_e672's state-5 path before adding.

#### `lp1-finalize-bit6-orientation-omitted` — partial (4. USB4 State Machines)
- **Stock:** 8000/81d4 finalize CODE_BANK1::8211-821a depends on cm_RXCM_handler cc86 (C2C3.0/C343.0 + phy_lane_gate 0x0AB3)
- **Handmade:** handmade/src/usb4_lanebond.h:1102-1117
- **What:** The LOOP1 81d4 finalize tests (snap&0xC0)==0xC0 at 0x70 but bit6 on host_desc 0x077B is HOST-driven and only earned upstream via the primary-lane orientation commit (cm_RXCM cc86: C2C3.0/C343.0 + phy_lane_gate=0x0AB3) which the comment notes 'handmade still omits'. Without it the device never gets bit6, so the 8262 width-latch select never advances, blocking LOOP1 0x60->0x70->...->A0->A1 and thus the bond.
- **Fix:** Port the cm_RXCM_handler cc86 primary-lane orientation commit (C2C3.0/C343.0 set + phy_lane_gate@0x0AB3) into the SB-EVENT path (§5/cm_tunnel) so the device earns bit6 in the host descriptor; verify LOOP1 reaches A1 and LOOP2 reaches 0x60 with SB[A0/A1]=0x02 on the wire.

#### `c5ff-dealias-not-lifted` — partial (5. CM-Tunnel / Router-ops)
- **Stock:** route de-alias / bridge bus-num @ CODE:c5ff (4x CfgWr via adb0: d956/9ee5/9a7f + 0x05B4=2)
- **Handmade:** handmade/src/cm_tunnel.h:253 (cm_route_test, DEBUG)
- **What:** c5ff's 4x CfgWr-over-tunnel route de-alias is not ported as a live GPU-path one-shot. Only the DEBUG cm_route_test exists, which replicates 3 of the 4 CfgWr with hardcoded bus0/dev0, performs an extra ECAM MemRd probe, and is uninvoked. Per the resolved boundary this de-alias is necessary (PERST-free, touches no PHY) but currently dead.
- **Fix:** Lift c5ff's 4 CfgWr (reg6 bus-num, reg1 cmd, reg8 mem-win 0x00D00000 aperture, 4th cap-derived 0x40010000 keyed off the 9ee5 cap-walk) into a live one-shot driven by the already-ported cm_adb0_tlp engine, plus the per-port 0x05B4=2 done-marker. Drop the 9037/c00d/PERST path. Drive it post-bond once the router-op mailbox is real.

#### `missing-E2-E3-cfg-rw` — missing-handler (5. CM-Tunnel / Router-ops)
- **Stock:** routerop_op_E2_cfgread @ CODE_BANK1::c0ef / routerop_op_E3_cfgwrite @ CODE_BANK1::c119 (+d945/cf5d/ceab/ceef/cf4c)
- **Handmade:** absent
- **What:** The router config-space READ (E2) and WRITE (E3) handlers plus their send-response (d945/cf5d), bounds (ceab), addr-copy (ceef) and sub-opcode (cf4c) helpers are all missing. These let the host CM read/write the tunneled router/device config space; the handmade mailbox only fakes their state transition and ACKs.
- **Fix:** Port c0ef/c119 and the d945/cf5d/ceab/ceef/cf4c/cf35 helper set; back them with the multipacket working buffer (0x0B04 addr / 0x0B0A limit / 0x0B08 write cursor) so multi-packet reads/writes continue across IDLE->MULTIPKT_1/2.

#### `missing-E8-tunnel-reset` — missing-handler (5. CM-Tunnel / Router-ops)
- **Stock:** routerop_op_E8_tunnelreset/tunnel_routerop_link_reset @ CODE_BANK1::c15f -> e4a6 (+eec7,e4ea)
- **Handmade:** absent
- **What:** The 0xE8 PcieTunnel-Deassert/Enable op is entirely missing. e4a6 (C656&=~0x20, CA06&=~1, CC31.0=1 assert+spin, eec7 C659.0 PERST-deassert, B480&=~0x0F) is THE trigger that brings the downstream GPU out of PERST. Without it the GPU stays in reset even after the tunnel link trains.
- **Fix:** Port c15f/e4a6/eec7/e4ea byte-true into bank1 and wire them as the E8 arm of the c0a5 dispatcher. Handmade already has the constituent C659 helpers (cm_e8a9/cm_e8d9_c659) to reuse for the PERST-deassert bit.

### MEDIUM

#### `de16-d127-d894-dma-omitted` — partial (1. Boot / Init)
- **Stock:** bank0_de16 CODE:de16 / bank0_d127 CODE:d127 / bank0_d894 CODE:d894
- **Handmade:** absent
- **What:** Stock boot_hw_init_main runs de16 (PHY-DMA cfg + sb_channel_connect_service), d127 (PCIe-DMA ring SIZE/BUF/DOORBELL/CEF2-3/B281), and the USB4-fork d894 (bank-2 adapter cluster incl 0x121E.0). None are ported. These are SHARED?-uncertain for the GPU tunnel DMA/connect-service path.
- **Fix:** Decompile de16/d127/d894 in Ghidra to determine which registers feed the PCIe-tunnel TLP/DMA path vs NVMe-only; port the tunnel-relevant subset (especially d894 0x121E.0 and de16 sb_channel_connect_service).

#### `eef9-int-enable-missing` — missing-handler (1. Boot / Init)
- **Stock:** bank1_eef9 CODE:eef9
- **Handmade:** absent
- **What:** Stock boot calls bank1_eef9 which does INT_ENABLE|=0x40 (bank1/USB4 INT group bit6). Handmade has no equivalent; usb4_irq_arm (usb4_irq.h:242) arms CC3B+ef24+ef1e but never sets the 0x40 bank1 INT-group enable bit. If that bit gates the bank1 a066/c0a5/e911 USB4 INT1 handlers, they would never fire.
- **Fix:** Confirm via Ghidra which SFR/bit eef9 sets and whether it gates the USB4 INT1 dispatch; if so, add the INT_ENABLE|=0x40 write to usb4_irq_arm or pd_int1_enable_group.

#### `no-spi-fuse-override` — partial (1. Boot / Init)
- **Stock:** bank0_92c5_seed CODE:92c5 / usb4_cap_apply_09f9 CODE:8d77
- **Handmade:** boot_phy.h:151 / main.c:496-516
- **What:** Stock 92c5 overrides lane-mask(0x0AE9)/lane-gen(0x0AEE)/width(0x086C-71) from OTP fuses when 0x707E==0x5A; 8d77 overrides cap/mode_flag/0x0A52-55 from the SPI cap-blob when 0x707E==0xA5. Handmade hardcodes all of these from a captured stock trace and only prints the fuse/blob bytes (boot_phy.h:155-162). On an OTP/strap-fused or differently-provisioned board the handmade defaults silently diverge from stock.
- **Fix:** Acceptable for the current board; document the dependency. If targeting other boards, port the 0x707E magic + checksum gate and apply the fuse/blob overrides.

#### `missing-c105-secadapter` — missing-handler (2. ISR Handlers)
- **Stock:** usb4_sec_adapter_link_event_c80a4 (CODE:c105, trampoline 0x0593)
- **Handmade:** usb4.h:98
- **What:** Stock services C80A.4 via c105 (secondary USB4 adapter / link-width events, page-1 1407/1603, linkwidth recovery + CC re-arm). Handmade usb4_int_demux only sets usb4_int_seen|=0x02 for int_sources&0x10 and drops the event — no handler.
- **Fix:** Port c105 as the C80A.4 handler (SHARED infra). At minimum replicate its W1C ack and link-width recovery; required if the host raises secondary-adapter/link-width events during tunnel bring-up.

#### `missing-cd10-cc33` — missing-handler (2. ISR Handlers)
- **Stock:** bank0_cd10 (CODE:cd10, via int1_isr_orchestrator 0x4486 trampoline 0x0390)
- **Handmade:** absent
- **What:** Stock int1_isr dispatches CC33.2 (W1C=4) to cd10 for PCIe-downstream link bringup/reset (CC31 spin = controlled soft-reset). Handmade int1_isr (main.c:462-474) has NO CC33 read or branch. Per the resolved CM-tunnel boundary this ISR-time path is replaced by the custom superloop pcie_power_on(), so it is intentionally absent, but there is no equivalent ISR-driven downstream reset.
- **Fix:** Confirm pcie_power_on() fully subsumes cd10's CC33.2 downstream bringup; if any host-initiated CC33.2 event must be serviced at ISR time, add a gated CC33.2 W1C+handler. Otherwise document as deliberately-omitted (SHARED, replaced by pcie_power_on).

#### `tunnel-branch-w1c-only` — partial (2. ISR Handlers)
- **Stock:** tunnel_link_event_e763 (CODE_BANK1::e911, trampoline 0x0570)
- **Handmade:** usb4.h:108-113
- **What:** Stock C80A.0-3 branch calls e911: E763.2 -> [PcieTunnel-PcieLinkUp]+d17e, E763.3 -> PcieLinkDn, plus W1C. Handmade only W1Cs REG_PHY_RXPLL_TRIGGER bits .2/.3 (REG=E763) and runs no link-up/link-down processing (no d17e). Tunnel-link-up notifications are swallowed.
- **Fix:** Port e911's E763.2 PcieLinkUp path (d17e processing) into the int_sources&0x0F branch so a tunnel-link-up event is actually consumed, not just acked. GPU-PATH relevant.

#### `dee3-lane-advance-drain-missing` — missing-handler (3. Superloop)
- **Stock:** bank0_dee3 CODE:dee3
- **Handmade:** absent
- **What:** Stock 541f-gated dee3 drains sb_cb10_lane_advance: ==10 spin, ==0x0b path (cb23/cadf) + clears entered_usb_mode. Classified GPU-PATH in §3. No handmade equivalent; the handmade sb_cb10_lane_advance (sb_router.h:702) only latches CL nibbles, it does not perform the dee3 drain/connect-drop handling.
- **Fix:** Decompile CODE:dee3 + CODE:541f; port the drain (==10/==0x0b cb23/cadf + entered_usb_mode clear) into the superloop guarded by REG_LINK_STATUS_E716&3, or confirm the lane-advance drop case cannot occur on the GPU path before omitting.

#### `pend-int-flag-never-set` — partial (3. Superloop)
- **Stock:** a5d8 CODE_BANK1::a5d8 (deferred responder gate)
- **Handmade:** sb_router.h:552 / main.c:670
- **What:** sb_pend_int_pending (sb_router.h:552, __at 0x0B55) is never assigned 1 anywhere in the tree (only cleared at main.c:671). The superloop a5d8 [Pend Int] responder therefore only runs on the live SB[0x26]&0x02 poll, never via the intended ISR-deferred flag. The deferral mechanism is half-wired.
- **Fix:** Either set sb_pend_int_pending=1 from the INT1 SB router-op handler (a5d8 trigger) so the responder is reliably deferred, or remove the dead flag and document that the SB[0x26].1 poll is the sole trigger. Verify no router-op response is being missed between ISR edges.

#### `af38-sbtx1-width-residual` — partial (4. USB4 State Machines)
- **Stock:** sb_af38_descriptor_response CODE_BANK1::af38
- **Handmade:** handmade/src/sb_router.h:72 (SBTX[1] at :89-90)
- **What:** af38 builds SBTX[1] = desc_dir | sb_width_lut[desc_type]. In THIS tree sb_width_lut@0x06F2 is correctly boot-seeded by sb_rom_descriptor_load (sb.h:145-148) so for the route descriptor (idx 0x0C) the width = 0x03, fixing the historical 0x55-poison half of GAP-1. BUT the ground-truth captures still show the device not emitting the stock 2nd-af38 TX '0104 6324' and SB[0x0C] not advancing 08->0B->0C; this residual is a downstream lane-bond/CL-walk staging issue, not the width seed. Stock af38 reads the width via FUN_976e(desc_type-0xe) (offset form) whereas handmade indexes sb_width_lut[desc_type] (direct 0..0x12); confirm the two index conventions land on the same ROM 0x514c byte (idx 0x0C=0x03) for ALL descriptor types, not just 0x0C.
- **Fix:** Verify on-rig that with the boot-seed live, the first route af38 now emits 0C03 (TX[1]=0x03). If TX is correct but SB[0x0C] still stalls / 2nd af38 emits 0000, chase the SB[0x0C] descriptor-staging advance in the §4 FSM-TICK (a7de/0x0758 substate) + §5 CL-walk, not af38. Also re-confirm the 976e(-0xe) vs [desc_type] indexing equivalence across desc types via Ghidra 976e.

#### `e391-0776-gate-live-fire` — partial (4. USB4 State Machines)
- **Stock:** cm_conn_routing_setup CODE_BANK1::a7de/a869 (e391 caller-site gate 0x0776==0)
- **Handmade:** handmade/src/usb4_lanebond.h:250-273
- **What:** e391 width-LUT seed is ported byte-true but only fires when u4_coldboot_seed_gate(0x0776)==0. The seed is what gives af38 SBTX[1]=0x03 instead of uninit 0x55 (GAP-1). If the 0x0776-clear path (the 07B9/081B.0/07CE/07CD confirm at lines 250-258) does not evaluate to clearing 0x0776 on the LIVE Connect_U4 path, the seed never runs and the first af38 still emits 0C55, stalling state-3.
- **Fix:** Instrument the [cr ..] diag (already present, line 242-247) to confirm 0x0776 actually reaches 0 on the live connect; verify u4_confirm_input_ce/cd/cf and 081B.0 inputs are set by the connect-descriptor path so the e391 gate opens. If it never opens, trace the 0x0776 writers (b7a4 sets=1, the 07B9!=0 confirm a835-a857 clears) to find why the clear is skipped.

#### `stale-cap20g-comment-0819` — partial (4. USB4 State Machines)
- **Stock:** cm_conn_routing_setup CODE_BANK1::a7de (0x0819 lane-advertise)
- **Handmade:** handmade/src/usb4_lanebond.h:288-290
- **What:** The comment block asserts cap20g_gate1=0 -> 081A=0xE1 -> 0x0819=0x01 'matching stock'. This is FALSE and contradicts (a) the live code main.c:498 which sets 0x09F6=1, and (b) HW ground truth (stock advertises both lanes, 0x0819=0x03). The CODE at line 290 is correct (ORs lane1 in when 081A.1 set), only the comment is wrong/misleading.
- **Fix:** Correct the usb4_lanebond.h:288-290 comment to state cap20g_gate1=1 (main.c:498), 081A keeps bit1, 0x0819 reaches 0x03 (2-lane) when host advertises lane1 (077A.1); lane0 still bonds first. Remove the false 'lane0-first naturally / matching stock' claim.

#### `substate-poll-noop-stub` — stub (4. USB4 State Machines)
- **Stock:** sb_transport_substate_poll CODE_BANK1::d4cd (substate-poll tail)
- **Handmade:** handmade/src/sb_router.h:247
- **What:** sb_transport_substate_poll is a no-op stub; the comment claims all edges are handled by sb_d4cd_transport_edges. Stock d4cd is documented (stock map row d4cd) as a poller that polls SB[0x28/0x2A/0x81/0x83] bit.3 edges AND drives 0x06EE/06EF/06F0 with a d54c sub-path. The handmade sb_d4cd_transport_edges sets 0x06F0 and acks edges but the d54c branch and the 0x06EE/0x06EF drive may be incompletely reproduced.
- **Fix:** Diff stock CODE_BANK1::d4cd against sb_d4cd_transport_edges to confirm the d54c sub-call and 0x06EE/0x06EF updates are not lost. If the consolidated d4cd covers them, delete the stub for clarity; if not, port the missing substate-poll/d54c logic.

#### `adb0-engine-dead` — partial (5. CM-Tunnel / Router-ops)
- **Stock:** cm_adb0_tlp @ CODE:adb0
- **Handmade:** handmade/src/cm_tunnel.h:174
- **What:** The general config-TLP issuer cm_adb0_tlp (and cm_c1f9, cm_e89d_read, cm_e91d_write, the cm_mbox_* helpers) are ported byte-true and LIVE-capable but have no live caller (only the DEAD cm_topo_walk/cm_route_test/cm_dbg_cfgrd debug functions and the DEAD c00d/9037 machine). The validated config-TLP engine is sitting idle.
- **Fix:** Keep the engine; once the router-op mailbox is real, drive cm_adb0_tlp from (a) the E2/E3 router-op config R/W handlers and (b) the lifted c5ff de-alias one-shot. No code change to adb0 itself needed.

#### `reply-trigger-missing` — missing-handler (5. CM-Tunnel / Router-ops)
- **Stock:** cm_routerop_reply_trigger @ CODE_BANK1::cf35 (C805=(C805&0xF9)|0x02) + C8B0<-0xEA DMA push
- **Handmade:** absent
- **What:** Handmade acks router-ops by writing only EA90=0xA5; it never asserts the C805 send-response bit (cf35) nor arms the C8B0<-0xEA DMA stream push that actually transmits the reply TLP back to the host CM. So no router-op response ever reaches the host even when state advances.
- **Fix:** Port cf35 and the C8B0=0xEA DMA-push arming and call them from each opcode handler's completion, as the stock c0a5 does after d945/cf5d build the response.

#### `working-buffer-absent` — omitted-state-arm (5. CM-Tunnel / Router-ops)
- **Stock:** router-op working buffer 0x0B04/0x0B0A/0x0B08 (ceef/ceab)
- **Handmade:** absent
- **What:** The router-op working-buffer state machine (64-bit addr 0x0B04 vs limit 0x0B0A, write cursor 0x0B08/09, len=min(limit-addr,7)) is not implemented. The handmade MULTIPKT_1/2 arms have no payload buffer, so even a partial dispatch could not produce a correct multi-packet reply.
- **Fix:** Implement ceef (copy addr/payload from EAxx mailbox) and ceab (bounds compare -> continuation vs completion) and the 0x0B04/0x0B0A/0x0B08 working buffer as part of the E2/E3 port.

#### `vdm-cmd5-enter-usb4-ack-absent` — missing-handler (6. PD / VDM)
- **Stock:** vdm_enter_usb4_ack_sb_init CODE:ca71 (dispatched from vdm_tx_strobe_commit CODE:9ac4 cmd==5)
- **Handmade:** vdm.h:233 (vdm_tx_dispatch switch; cmd5 falls to default vdm_nak vdm.h:251)
- **What:** Stock 9ac4 routes VDM cmd 5 (Enter_USB4) to ca71 which ACKs and hands off to the USB4 SB/lane subsystem (sb_lane_flip_init + sb_block_init). Handmade vdm_tx_dispatch only handles cmd 1-4; cmd 5 hits the default arm and is NAK'd. The handmade design instead bridges to USB4 via the PD3.1 Enter_USB Data Message path (pd_handle_enter_usb -> usb4_connect_u4), which the captured wire trace shows the host actually uses (Enter_USB Accept reached). Latent gap: a host that drives TBT entry via VDM Enter_USB4 rather than Enter_USB would be NAK'd and never bridge.
- **Fix:** Add a `case 0x05:` arm to vdm.h:233 that builds the Enter_USB4 ACK (pd_tx_set_sop_header(1,0x0F)+pd_vdm_hdr_build(1,5), msg_len=6) and on success invokes the SB hand-off (sb_lane_flip_init / the same bring-up usb4_connect_u4 performs), mirroring ca71; gate the post-commit `[Connect TBT]` sb_lane_flip_init on cmd==4||cmd==5 with the u4_connect_oneshot_suppress check as stock 9ac4 does. Verify on the wire only if a host is seen issuing VDM Enter_USB4; the current Enter_USB path is the validated bridge so this is a robustness/parity fix, not the live blocker.

#### `missing-b220-cap-read-x4-width` — partial (7. Local PCIe (device→GPU downstream LTSSM))
- **Stock:** pcie_downstream_link_bringup CODE:3578 (B220=0x1404600 cap read + bank0_d436 0xF x4 width)
- **Handmade:** main.c:91
- **What:** pcie_power_on omits the stock B220 PCIe-config-TLP capability read (0x1404600) and the d436 x4 lane-width finalize that stock 3578 performs after PERST deassert. Handmade trains the downstream link at x2 only (REG_TUNNEL_LINK_STATUS=PCIE_LINK_WIDTH_x2=0x0C) and never runs the cap-derived width ramp, so the GPU link may come up narrower/slower than stock. d436 is present elsewhere (cm_arm_c00d:107) but cm_arm_c00d is DEAD on the GPU path.
- **Fix:** If the trained width/gen is below target after CONNECTED, port the stock 3578 tail: after B480 PERST deassert and B450==0x78, do the B220 cap read and a PERST-free d436(0xF) x4 width apply on the GPU path (do NOT route through 9037/c00d). Confirm via REG_PHY_PCIE_LINK_INFO (4092) gen/lane nibbles already printed in pcie_power_on.

#### `tunnel-link-setup-8000-absent` — missing-handler (7. Local PCIe (device→GPU downstream LTSSM))
- **Stock:** pcie_tunnel_link_setup CODE:8000 (bank0)
- **Handmade:** absent
- **What:** The bank0 tunnel data-path setup CODE:8000 (B220 config-TLP, B230/B234/B240/B244/B246 link params, read negotiated width from 0x8000-0x800B SB scratch, set 0x05B4=2) has no single ported counterpart. Handmade has the B220 mailbox engine in cm_tunnel.h but not this tunnel-link-setup unit that latches negotiated width and the 0x05B4=2 marker. Note 0x8000 here is the BANK0 fn, distinct from CODE_BANK1::8000 u4lb_lane_gate.
- **Fix:** Audit whether the negotiated-width latch (0x8000-0x800B scratch) and 0x05B4=2 marker are needed for the host CM to see the PCIe-down adapter as ready. If the host stalls waiting on the tunnel-link-setup completion marker, port CODE:8000's B23x/B24x link-param writes + width latch into a GPU-path one-shot (PERST-free), alongside the c5ff de-alias lift noted in section 5.

---

## LOW (compact)

- **`mode-flag-hardcoded-no-b1cb`** (wrong-dispatch, 1. Boot / Init): Stock derives 0x09F9 through 4c40 default(0x04)->8d77 SPI map->b1cb (91C0&0x18)==0x10 decision. Handmade hard-sets 0x09F9=0x87 unconditionally, bypassing the runtime USB4-vs-USB3 mode decision; the USB3 fallback branch (usb_init_controller, usb_phy_tune, pcie_power_off/on at main.c:525-536) is therefore effectively dead unless 0x87 is later cleared. — *Fix:* Intentional for forcing USB4; fine as long as the goal is USB4-only. Note the non-USB4 fork is dead code under this hardcode.
- **`int0-nvme-tail-omitted`** (partial, 2. ISR Handlers): Stock int0_isr body has a C806.5 CPU-link + NVMe-queue service loop tail (5442/52a7/3419/180d/1196/488f/4784/49e9/3e81). Handmade int0_isr handles peripheral EP/bulk/CBW + 91D1 + 9300/01/02 buffer-cfg + MSC ack, but omits the NVMe-queue service tail. The SS-fail branch is gated to NOT drop to USB2 in USB4 mode (main.c:418). — *Fix:* NVMe-queue tail is NVMe-ONLY and correctly omitted; keep. Verify the 91D1/9300/9302 buffer-cfg demux fully covers the USB4-router link-event variants stock services here. Primarily a §8 concern.
- **`deferred-8a89-nonstock-fallback`** (handmade-only, 3. Superloop): The deferred-8a89 block drives bank0_c9a8(0) once after a synthetic fsm_stall>=6 heuristic and no bond. In-tree comment notes a batch-5 HW test found disabling it NEUTRAL. This is a non-stock fallback that may mask the real FSM-stall root cause and is classified SHARED? (uncertain). — *Fix:* Keep but flag: once the eea5/dee3 gaps are closed, re-test with this block removed; if the FSM no longer stalls it should be deleted to stay byte-true to stock (stock has no fsm_stall fallback).
- **`e2ec-541f-postconnect-reinit-missing`** (missing-handler, 3. Superloop): Stock superloop runs e2ec (USB4 post-connect reinit, e869/e95f) under ae9!=0x0f, and 541f as the E716&3 link-up guard for the dee3/d3a2 cluster. Neither is present in handmade; the link-up gating is instead implicit in pcie_power_on's both-lanes-CL0 check. — *Fix:* Confirm e2ec's post-connect reinit (e869/e95f) is not required for GPU tunnel stability; if the tunnel stalls post-connect, port e2ec. Low priority unless a post-connect re-init symptom appears.
- **`pd-tx-commit-relocated`** (wrong-dispatch, 3. Superloop): Stock commits staged PD TX as a per-pass superloop check on pd_tx_staged_pending!=0. Handmade calls pd_tx_commit_engine inline from the PD/VDM dispatchers, not from the superloop. Functionally equivalent for synchronous TX but loses the deferred-commit timing if a stage is set outside a dispatcher. — *Fix:* Verify no code path stages pd_tx_staged_pending without immediately committing; if so, add the superloop pass `if(pd_tx_staged_pending) pd_tx_commit_engine();` to match stock e1c6 dispatch.
- **`af38-plane-status-readsource`** (byte-divergence, 4. USB4 State Machines): Stock af38 reads the per-port status from absolute page-1 0x2280D (port0) / 0x2280E (port1). Handmade reads SB(0x0D)/SB(0x0E) (i.e. 0x2800+0x0D = 0x2280D) — addresses match. Plane RX descriptor read uses sb_rxplane_212d[]={2a00,2b00,2c00,2d00}; stock af38's body-copy reads read_0x755 (the 0x0755 staged descriptor pointer) not a fixed plane base. Confirm sb_rxplane_212d indexing matches the stock read_0x755 plane for the body-copy branch. — *Fix:* Confirm via Ghidra that read_0x755 resolves to the same plane base that sb_rxplane_212d[port] yields for the active port; the status reads (0x2280D/E) already match. Low risk — likely equivalent.
- **`cb10-thinned-vs-stock`** (partial, 4. USB4 State Machines): Stock cb10 is one function owning the SB[A0/A1] latch compare + ee57 throttle + e672 dispatch + cdf5 tail. Handmade splits it: sb_cb10_lane_advance() does only the SB[A0/A1] nibble-latch compare (plus a non-stock cb10_seen diag accumulator), while the throttle/e672/cdf5 are inlined in the main super-loop. Behaviorally equivalent but structurally diverged; the cb10_seen byte and the IE_EA bracketing are handmade-only. — *Fix:* Leave as-is (functionally byte-true); document that cb10 is decomposed across sb_router.h + main.c. Optionally fold back into a single u4lb_cb10() for a cleaner stock correspondence once the bond closes.
- **`e391-loop2-deepphy-omitted`** (partial, 4. USB4 State Machines): Stock e391 has TWO loops: loop1 seeds the width-LUT (0x06F2) + branch-A gate (0x0705) — ported into sb_rom_descriptor_load; loop2 (i=0..7) zeroes XDATA[0x0B26+i] (lb_cl_status deep-PHY shadow). The handmade reproduces only loop1; the loop2 zero-init is not present in sb_rom_descriptor_load. — *Fix:* Per MEMORY loop2 is 'omit-able' (deep-PHY). Leave omitted unless the 850b walker (which reads lb_cl_status@0x0B26) is ever live on the AMD path — it is not (state-5 walker A/8000 is the live path). If 0x0B26 reads stale, add the 8-byte zero at the end of sb_rom_descriptor_load.
- **`lp1-width-settle-clr-replaced-by-park`** (byte-divergence, 4. USB4 State Machines): The stock 811a CLR of work_buf[0x1C+lane] (re-zeroing at 0x50 every tick) is deliberately replaced by an LP1 park, because the byte-true CLR was HW-confirmed to clobber the host-written 7B7B route-id and break the bond. This is an intentional, documented divergence from byte-true (a NON-stock behavior chosen to preserve host state). — *Fix:* Keep the park (HW-validated as necessary) but flag it as an explicit non-byte-true divergence; revisit only if the upstream descriptor staging is changed such that the stock CLR no longer clobbers 7B7B.
- **`c00d-9037-dead-correct`** (misclassification, 5. CM-Tunnel / Router-ops): The 9037/c00d CM step-machine + PERST arm is present (byte-true) but intentionally DEAD/uninvoked (removed from the live path 2026-06-18 because its B480 PERST-assert/C659-clear/d436 re-drive raced pcie_power_on's downstream LTSSM). This is correct per the resolved boundary (NVMe-only and harmful on the GPU path) and should NOT be re-wired; flagged only so it is not misclassified as a missing handler. — *Fix:* Leave dead. Do not re-invoke c00d/9037 on the GPU path. The live downstream-link bring-up is pcie_power_on() (main.c, mapped in section 7). Optionally strip these dead functions once the router-op engine is proven.
- **`enter-mode-no-sb-handoff`** (partial, 6. PD / VDM): Stock invokes sb_lane_flip_init after an Enter_Mode (cmd4) ACK (the cmd==4||5 tail in 9ac4). Handmade vdm_handle_enter_mode only sets u4_connect_route_latch/u4_connect_pending and prints [Enter_TBT]; the SB hand-off is deferred to the Enter_USB path (pd_handle_enter_usb -> usb4_connect_u4). This is consistent with the working Enter_USB bridge design, so it is intentional, but the cmd4 lane-flip trigger is structurally absent vs stock. — *Fix:* No change required while the Enter_USB bridge is the live path; document the design choice. If parity with stock TBT-mode entry is desired, wire the cmd4/cmd5 post-commit sb_lane_flip_init in vdm_tx_dispatch's strobe tail (gated by u4_connect_oneshot_suppress) as stock 9ac4 does.
- **`vdm-cmd6-exit-absent`** (missing-handler, 6. PD / VDM): Stock 9ac4 handles VDM cmd 6 (Exit_Mode) via e973 (and returns without the post-commit lane_flip). Handmade NAKs cmd 6. Not on the GPU bring-up path (Exit is teardown), so low impact, but a parity gap. — *Fix:* Add a `case 0x06:` arm that builds the Exit ACK and returns before the lane-flip tail, matching e973; low priority — only affects mode teardown, not enumeration.
- **`cm-arm-c00d-deadweight`** (stub, 7. Local PCIe (device→GPU downstream LTSSM)): cm_tunnel_link_bringup_start (92bb) and cm_arm_c00d (c00d) are ported but drive the NVMe-harmful CM step-machine (B480 PERST + C659 + d436 re-drive) that races the downstream LTSSM. Per the resolved boundary they are DEAD/uninvoked on the GPU path (removed from the live super-loop 2026-06-18). They remain in the tree as code that does nothing useful for GPU enumeration. — *Fix:* Leave uninvoked (correct). Ensure no GPU-path caller reaches cm_arm_c00d; the live path must use pcie_power_on only. Consider a comment/guard documenting that c00d is NVMe-only and must not be wired to the bond->tunnel-up trigger.
- **`divergent-linkup-gate`** (byte-divergence, 7. Local PCIe (device→GPU downstream LTSSM)): Two coexisting link-up criteria test different registers. pcie_power_on (main.c:106-119) gates CONNECTED on REG_PCIE_LTSSM_STATE (B450)==0x78 plus E765.2 only in the printed string, whereas the ported stock criterion cm_link_up_check (e2a6, cm_tunnel.h:41) uses (B432&7)==7 && E765.2 and is gated on 0x07EF==0. pcie_power_on does not call cm_link_up_check; the two could disagree about when the link is up. — *Fix:* Decide on one authority. Since e2a6 is the stock-true criterion (and cm_link_up_check is already ported byte-true), consider having pcie_power_on's success condition also require cm_link_up_check()==1 (B432&7==7 && E765.2 && 0x07EF==0) rather than relying solely on B450==0x78, to match stock semantics.
- **`pcie-poweron-replaces-3578`** (handmade-only, 7. Local PCIe (device→GPU downstream LTSSM)): pcie_power_on is a custom downstream bring-up that wholesale replaces stock 3578. Stock ramps B455 LTSSM speed (2 then 4), enables B2D5/B296, reads device caps via B220 config-TLP (0x1404600), deasserts PERST, polls B455.1 train-done, applies d436 x4 width, returns 0x0F. Handmade instead does B431=x2 width, B403=1, a B480 PERST assert/deassert cycle, B298 TLP-routing, C656/C659 rails, then polls B450==0x78 CONNECTED. This is the intended boundary (stock 3578 is reachable only via the NVMe-harmful 9037/c00d PERST-race), so it is a deliberate replacement, not a bug. — *Fix:* Keep as the GPU-path mechanism. No change needed; documented as resolved boundary. Verify only that it consistently reaches 0x78 (MEMORY says it does).
- **`link-mode-finalize-ca0d-absent`** (missing-handler, 8. USB Device): Stock ca0d latches entered_usb_mode=0x10 to finalize USB link-mode. No direct handmade equivalent in the USB-device subsystem; mode-entry/commit is handled in pd/usb4 (usb4_mode_entry_commit 0102, 92E1=0x10). Likely covered elsewhere but not in §8. — *Fix:* Confirm the 0x10 entered-usb-mode latch is set by the USB4 mode-entry-commit path (cross-section). If a downstream consumer reads the §8 latch and finds it unset, port the ca0d write.
- **`ss-link-event-policy-0003-partial`** (partial, 8. USB Device): Stock's deferred 0x0003-bitmap link-event policy (b3→pcie-down bringup, b4→USB4 lane train, b6→TypeC recovery, b2→SS re-init+mode re-decide) is only partially reproduced: int0 handles 9302.2 and 9300 SS events inline (→bank0_c9a8, SS-FAIL fallback) but does not implement the full deferred 0x0003 demux. The USB4 lane-train (b4) and pcie-down bringup (b3) are instead driven by the USB4/SB subsystems, so the omission is GPU-path-neutral. — *Fix:* Verify the USB4 lane-train + pcie-down-bringup that stock's 4532 b3/b4 would have armed are fully covered by the sb_router/usb4_lanebond subsystems (they are, per the GPU-tunnel chain). No port needed unless a 0x0003 event is observed unhandled.
- **`ss-link-train-engine-missing`** (missing-handler, 8. USB Device): Stock SS link-train engine (cc10 train via 91D1.3, SS-FAIL→force USB2) has no handmade port. int0_isr only reproduces the SS-FAIL→USB2 fallback POLICY inline (main.c:416-429), not the train engine. In USB4 mode the fallback is suppressed (correct), so this is harmless for the GPU path but means a genuine SS link-train retry is absent for any USB3 storage fallback. — *Fix:* No action needed for GPU path (USB4 mode skips SS bring-up). If USB3 fallback is ever required, port 9c2b's cc10 train loop. Document as intentional non-port.
- **`vendor-bulk-replaces-msc`** (handmade-only, 8. USB Device): The entire stock USB-MSC/NVMe storage data-path is intentionally replaced by a vendor BULK-EP interface (tinygrad). SET_CONFIG writes REG_USB_MSC_CFG=0 to bypass MSC; bulk completion streams via pcie_read_chunk/pcie_write_chunk. This is by design, not a regression — flagged only to record the deliberate divergence. — *Fix:* No fix. Keep the stock MSC/NVMe storage fns omitted (omitted-NVMe). Ensure the vendor 0xF0/0xF2/0xF3 TLP/DMA hooks remain the host's tunneled-config driver for the GPU path.
