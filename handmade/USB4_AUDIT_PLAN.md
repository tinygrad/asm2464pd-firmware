<!-- Fidelity+interference audit (wf_e455e7e4) of handmade USB4 impl vs stock disassembly. THE action plan. Apply phased per Section 6; do not omit/cherry-pick. -->

# PRIORITIZED ACTION PLAN — make the TB4 host negotiate USB4 (sustain E302 / raise C80A.5 / reach GPU)

Verified against source: `int1_isr` (main.c:463-476) reads only C80A — no C806, no timer-tick, no CC33.2, no C806.4. `sleep()` (main.c:29-40) writes CC10/CC11/CC12/CC13, which `pd_dispatch.h:190-198` proves is the PHY/PD command mailbox (registers.h at `/home/batman/asm2464pd-firmware/src/include/registers.h` mislabels them `REG_TIMER0_*`). `pcie_power_off()`/`pcie_power_on()`/`usb_phy_tune()` all run unconditionally at boot (main.c:497,503,506) before the USB4 arm. `cc_pd_timer_tick`/`B4BA` is entirely absent. `usb4_mode_entry_commit`@D78A exists (vdm.h:185) but is reachable only via Enter_USB, never via the CC91 attach edge.

---

## 1. TOP SUSPECTS (ranked by likelihood of unblocking the host)

**S1 — MISSING INT SOURCE: the C806.0 timer-tick `cc_pd_timer_tick`@0xB4BA is not serviced at all.** [isr_superloop OMISSION/high @0x44a3]
This is the single most likely blocker. The stock orchestrator @0x4486 runs B4BA FIRST and UNGATED. B4BA is the PD/USB4 *policy-engine* tick that drives: the re-elicitation Hard-Reset loop (CC81.1 reads 0x07BD substate → e90b hard-reset / d676 full-reset), and crucially **CC91.1 → 0x07BB=1; 0x09FA=4; usb4_mode_entry_commit@D78A; 0x0AE2=R7** — the USB4 mode-entry latch driven by the controller attach edge. handmade's `int1_isr` never reads C806, so none of CC23/CC81/CC91/CC99/CCD9/CCF9 are serviced. The CM/attach handshake stalls exactly at the observed symptom (no sustained C80A.5, E302 collapses). Without the CC91→D78A path the device never commits mode entry on the controller's terms, so the host never proceeds to lane training. **Highest value, low cost.**

**S2 — INTERFERENCE: `sleep()` is the PHY/PD mailbox (CC10-CC13), and it runs on the boot path.** [interference INTERFERENCE/high; boot INTERFERENCE/high]
Every `sleep()` issues `CC11=4;CC11=2` (e8ef ack) + `CC10=(&0xF8)|4` (subcmd-4 — the SAME link-up-arm subcmd `usb4_phy_arm` uses) + garbage CC12/CC13 + `CC11=1` (go), then spins on CC11.1 (the PHY done bit). `pcie_power_on()` fires up to 20 of these (main.c:117) right after `boot_phy_bringup_early` configured the PHY and before `usb4_phy_arm`/`pd_keystone_init`. This actively corrupts the PHY/PD link engine each boot and can collapse a just-trained E302. **High value, low cost** (gate the calls; retarget sleep to Timer1 CC16-CC19).

**S3 — INTERFERENCE: unconditional `pcie_power_off()` + `pcie_power_on()` clobber tunnel state the CM owns.** [interference INTERFERENCE/high ×2]
Boot pre-stages the tunnel (d996 inside boot_phy), then `pcie_power_off()` tears it down, then `pcie_power_on()` re-asserts a hardcoded version — three conflicting passes over B480/B430/B431/E764/B298/6025/C656/C659, the exact registers the USB4 CM/SB-lane-bond path (e52d→0x3625) owns. Stock touches these ONLY from the CM path, never at boot. This corrupts the state the host CM polls. **High value, low cost** (gate behind `!(0x09F9&0x83)`).

**S4 — MISSING BOOT STEP: 0x92C5 RAM-state seed never runs → mode-decision flags are garbage.** [boot OMISSION/high @0x92C5]
0x0AE3/0x0AE4/0x0AE5/0x0AE8/0x0AE9/0x0AEA/0x0AEE/0x0AF0/0x0213 are uninitialised. d436 reads 0x0AE5, boot_hw_init reads 0x0AE3, the super-loop reads 0x0AE8/0x0AE9, and 0x0AE9=0x0F (link width) + 0x0AEE=3 (mode) are load-bearing for the tunnel width. Running the width/mode logic against garbage is a strong candidate for "link comes up wrong." **High value, low cost.**

**S5 — MISSING BOOT STEP: 0xD894 never runs → C809 bit1 (USB4-mode PD interrupt enable) never set.** [boot OMISSION/high @0xD894]
Stock's USB4 branch sets `C809=(&0xFD)|0x02`. handmade only sets C809 bit5 (0x20) in cc_pd_phy_term_init. The bit1 PD interrupt the USB4 mode relies on is missing, which can starve the very interrupts the timer-tick (S1) and demux consume. **High value, low cost.**

**S6 — TRUNCATED SB-PHY RX config in `usb4_irq_ef24`/`usb4_irq_ef1e`.** [pd_engine TRUNCATION/high 8E31, 9A63; OMISSION/high DB0D]
The entire C2xx SB-PHY config block of 8E31 (post-`SB[0x49]=0xA0`), all page-0x78 lane RMWs of 9A63 (only 0x7834 done), and the page-0x12/page-0x28 writes of db0d are dropped. This is exactly the SB-PHY RX path; if it's half-configured the host's sideband connect packets are never detected → C80A.5 can't fire. **High value, higher cost** (needs bank1 disasm).

**S7 — TRUNCATED SB connect tail: `usb4_connect_u4`@A3F5 pre-gate + path-state chain dropped.** [vdm_connect TRUNCATION/high A415, A48C; sb TRUNCATIONs b230/bb37/b7a4]
The unconditional a415 pre-gate (dd42(0)/e7c1(1)/e0d9 PHY-descriptor seed) and the a48c tail (edbd/e5b0/c270 DROM PID latch/d556/EE82 tunnel link-up) are absent; sb.h re-implemented a guessed subset with fabricated d436/d8-plane addressing. The device asserts a partial, mis-addressed sideband — consistent with the host never training. **High value but high risk of churn**; do AFTER S1-S5 prove the interrupt/boot layer is sound, because mis-RE'd bank1 here is itself a regression source (the team already saw e0d9/e7ae "regress E302" — see S-note below).

**S8 — INTERFERENCE: `usb_phy_tune()` writes C280/C300 (zero stock references) unconditionally at boot.** [interference INTERFERENCE/medium]
Pure fabricated SERDES tuning on undocumented regs, run in USB4 mode. Lower likelihood of being THE blocker but trivially gateable. **Medium-low value, trivial cost.**

> **Critical caveat on S7/e7ae:** the prior "e7ae regressed E302" observation was with the pre-stage run UNGATED/out-of-order. In stock, e7ae (C006/C00E PHY-lock wait) is GATED by `0x0AF1.4` (JNB 0xe4 @a4cd) and the e0d9 seed is on the *separate* unconditional a415 pre-gate. The blanket removal was over-broad. Do not re-add e7ae ungated; reinstate the gated structure. [vdm_connect INTERFERENCE/medium a4cd]

---

## 2. OMISSIONS TO RESTORE ("do as stock does")

| # | Stock site | What's missing | Fix |
|---|---|---|---|
| O1 | 0x44a3→B4BA | **C806.0 timer-tick** (CC23/CC81/CC91/CC99/CCD9/CCF9 service) | In `int1_isr`, BEFORE C80A.6 and UNGATED: `if (PR(0xC806)&0x01) cc_pd_timer_tick();`. Transcribe B4BA verbatim; CC91.1 must do 0x07BB=1; 0x09FA=4; `usb4_mode_entry_commit()`; 0x0AE2=ret. CC81.1 reads 0x07BD → hard/full reset. W1C each channel =2. |
| O2 | 0x44ad→CD10 | **CC33.2 link-state service** | UNGATED: `if (PR(0xCC33)&0x04){ PR(0xCC33)=0x04; cc33_linkstate_service(); }`. At minimum the CC33=0x04 W1C so the source can't latch. |
| O3 | 0x44ed | **0x0AF1.5-gated E7E3 &=~0xC0** + call to router-op C0A5 | In EC06.0 branch: `if(PR(0x0AF1)&0x20){PR(0xE7E3)&=0xBF;PR(0xE7E3)&=0x7F;}` then call bank1 C0A5. |
| O4 | 0x44d6→C105 | **C80A.4 adapter handler** (currently only logs) | Call/transcribe bank0 C105 (acks r3_write 1/2); at minimum its W1C ack so C80A.4 can't storm. |
| O5 | 0x450d→EF4E | **C806.4 handler** (bank1 EF4E) | `if(PR(0xC806)&0x10) usb4_c806_4_handler();` at ISR tail; at minimum ack. |
| O6 | 0x92C5 | **RAM-state seed** | Before any reader: zero 0x0213,0x0AE3,0x0AE4,0x0AF0,0x0AE5-0x0AE8; set 0x0AEA=1, 0x0AE9=0x0F, 0x0AEE=3. |
| O7 | 0xD894 | **C809 bit1 PD-int enable** + bc8f/bcb1 banked clears + b031 | `C809=(&0xFD)|0x02` in the USB4 branch; decode bc8f/bcb1/b031. |
| O8 | 0x2F85→5418 | **CC32 |= 1** (stock's first HW write) | Top of `main()`: `PR(0xCC32)=(PR(0xCC32)&0xFE)|1;`. |
| O9 | 0x5284 | **Analog/SERDES bias cfg** | `C65B=(&0xF7)|0x08; C656&=~0x20; C65B=(&0xDF)|0x20; C62D=(&0xE0)|0x07;` right after boot_phy_bringup_early. |
| O10 | 0x4BE6 head | **0x07F0-0x07F5 const block** (0x24,0x04,0x17,0x85,0,0) + **CC3B |= 1** + **cb37** | Add const writes; add `CC3B|=0x01` before the `(&0xFD)|2`; transcribe cb37 (it's bank1, not bank0 — header label is wrong). |
| O11 | 0xE14B | **C8A6=0x04** + **0x7000 |= 0x1C** (+ b8b9/b820/b833/b881 RAM seed) | Reproduce concrete writes; decode b8xx. |
| O12 | 0xE56F | **bank1 USB4 router-init** (gated 0x09F9&0x81) | Disasm e56f at file_off `e56f-0x8000+0xFF6F`; run before sb_lane_flip in USB4 branch. |
| O13 | 0x8601→965d | **0x07BD=5** on every PS_RDY path | `PR(0x07BD)=5` in pd_ctrl_ps_rdy before clearing 0x07DE/0x07DF. |
| O14 | 0x9BEC | **EnterMode(cmd==4/5)→usb4_connect_u4** post-commit handoff | After e1c6 commit: `if(PR(0x0AA5)==4||==5){if(PR(0x07ED)){PR(0x07ED)=0;return;}else usb4_connect_u4();}`. |
| O15 | 0xA424/a477 | **0x07BA==0 / 0x07B9!=0 alt branch** | `else if(PR(0x07B9)){PR(0x09FA)=0x81;PR(0x09FB)=2;sb_assert();}`. |
| O16 | 0xA415 | **Unconditional pre-gate** dd42(0)/e7c1(1)→bd14/e0d9 seed | Re-add in order (the e0d9 PHY-descriptor seed the host CM reads). |

---

## 3. TRUNCATIONS TO COMPLETE (incomplete copies of stock)

| # | handmade fn | Stock body | What to recover |
|---|---|---|---|
| T1 | `usb4_irq_ef24` (8E31 half) | 0x8E31→RET | Full C2xx SB-PHY config block after SB[0x49]=0xA0: c343|=0x40, C2C5/c34a, C2A1=(&0x9F)|0x60, C28C/C29C/C2AC/C2BC seq, C2C3=(&0xC3)|0x1C, C2C9=(&0x80)|0x41, C2A5/C2CA/C287/C294/C2A2. Expand c343/c34a/c2f8/c351/c335/c30e/c358. |
| T2 | `usb4_irq_ef1e` (9A63 half) | 0x9A63→RET | All page-0x78 0x9403 write RMWs (≥8), not just 0x7834. Expand 9388/9403/940a/9267/9661/9668/9386. |
| T3 | `usb4_irq_ef24` (db0d half) | 0xDB0D→DB79 | Replace discard `PG_RD(0x1262)` with `&=0xEF` writeback; add page-0x28[0xED]=(&0xBF)|0x40, [0xCE]&=0xFE, [0x1C]|=0x80, |=0x40/|=0x02 pair, C20B&=0x7F, &=0xFE, C22F=(&0xFB)|0x04 then &=0xBF. |
| T4 | `sb_lane_flip_init` (b230) | b230→b3a5 (~165 bytes dropped) | After 0x06EC=0: CCD8/C801/CCDA/CCDB writes, SB[0xCF]/[0x53]/[0x5D]/[0x27]/[0x2D] RMWs, C809|=8, SB[0x67] clr b3/b6, 0x072B/0x072C=7, C8FF==4-gated copy of CODE 0x21b4[0x10] → 0x073E..0x074D. |
| T5 | `sb_block_init` (bb37) | bb45→bc5d | Add SB[0xBA]=0x3F, SB[0xBD]=0x3F, 0x09F7<2 tail; add e34b PHY RMWs (b70d C2C3/C343, b796 C21C, b73b C2C3/C343). |
| T6 | `sb_rom_descriptor_load` (b7a4) | b805→b86b+ | The 0x081A orientation RMWs and connect-state seed: 0x0750=1, 0x0765-67=0, 0x06ED=0, 0x075D=0x0F (width), 0x0776=1, 0x072A/0x072D=0. |
| T7 | `usb4_connect_u4` tail (vdm/usb4/sb) | a48c→a51e | edbd/e5b0/dd42(route)/ccb3/**c270 (DROM PID latch 0x0A57/0x0A58)**/d556/**EE82 (B430|=1 tunnel link-up)**. Verify each against bank1 disasm. |
| T8 | `usb4_mode_entry_commit` (d78a) | d79e→d7a1 | 0x92C2/bba8 USB-engine kick; return R7 (4 if 0x09F9.6 else 1) and store to 0x0AE2 (not raw `mode`). |
| T9 | `boot_phy_d996` (d996) | d9af→d9d2 | e25e PHY latch then 0x7041&=~0x40, 0x1507=(&0xFB)|0x04, 0x1507=(&0xFD)|0x02. |
| T10 | super-loop (2FB4) | full polled step machine | At minimum drain PD-TX via E1C6 and the 0x0A59/CM step LCALLs each iteration (keep E302 diag prints). |
| T11 | `pd_rx_nak_send` (871A) | 871A tail | SOP==1→pd_tx_set_sop_header(0,**4**) (not 1); SOP!=1→dd12(0,0x10) then 0x07BD=5. |

---

## 4. INTERFERENCE TO REMOVE / GUARD

| # | handmade code | Conflict | Fix |
|---|---|---|---|
| I1 | `sleep()` main.c:29-40 | Writes CC10-CC13 PHY/PD mailbox (subcmd-4 + GO) | Retarget to Timer1 CC16-CC19 (stock d47f: CC16=(&0xF8)|4 DIV, CC18=lo, CC19=hi, CC17 CSR/expired). Edit registers.h `REG_TIMER0_*`→CC16-19 or add REG_TIMER1_*. Never call on USB4 path until fixed. |
| I2 | `pcie_power_off()`+`pcie_power_on()` main.c:503,506 | Clobber tunnel regs the CM owns; 20× bogus sleep() PHY cmds | Gate both behind `!(PR(0x09F9)&0x83)`. Keep the legitimate USB4 trigger at main.c:625-629 (sb_tunnel_up_pending→bringup). |
| I3 | `usb_phy_tune()` main.c:497 | C280/C300 — zero stock references | Gate behind `!(PR(0x09F9)&0x83)` (USB3-only) or remove from USB4 path. |
| I4 | boot order overall | d996 pre-stage → power_off → power_on = 3 conflicting passes | Per I2: let d996 stand alone; defer real tunnel-up to SB-lane-bond. |
| I5 | `pd_select_pdo` + RDO | 0x07DA/0x07DB uninit → stock builder reloads garbage op-current | Write 0x07DA=0x07D4, 0x07DB=0x07D3 before pd_build_send_request_rdo (or port the real PDO loop @0xABF5). |
| I6 | `pd_ctrl_ps_rdy` voltage classifier | 0x07B8 codes 2↔4 SWAPPED + spurious `v_hi>=1` | `if(v>=0x012C)0x07B8=4; elif(v>=0x0096)0x07B8=2; else 1`. |
| I7 | `pd_arm_cc_timer` Accept/Wait args byte-swapped | Wrong sender-response/PS timers | Accept→(0x27,0x10); Wait→(0x07,0xD0). |
| I8 | `pd_ctrl_soft_reset` (invented 0x0D handler) | Resets 0x07C1/0x07C3 → desyncs host MsgID expectation | Verify host even sends Soft_Reset; if MsgID drift is the real need, fix at source (don't zero 0x07C1/0x07C3 mid-PD on boot). Route 0x0D to default 0x871A as stock does. |
| I9 | `sb_pcie_width_ramp` (d436) | Fabricated B434-B437 block-write | Replace with real d436 (c089 lane ramp + B436/B404 RMW; no B434). |
| I10 | `boot_phy_d436_width` | Writes B434 (stock never does) | Remove B434 write; derive B436 nibble from 0x0AA8&0x0E + B404. |
| I11 | sb.h flip/straight lane map | Operates on SB[0x01]/[0x02], stock uses page-1 P1[0x0101]/[0x0102], opposite sense | Re-address to P1 plane; flip⇒set b0,b1; straight⇒clear. |
| I12 | `sb_block_init` e0d9 inline | Inlined R7==4 branch (writes 9 PHY regs); stock calls e0d9(0)=zero C20E/F/210 only | Replace with C20E=C20F=C210=0. |
| I13 | a066 connect/disconnect edge | Fabricated SB[0x66]/[0x9E] writes; omits SB[0x2D] RMW + dea1/da9f | connect: SB[0x2C]=2, SB[0x2D] set b0/clr b1, dea1; disconnect: SB[0x2C]=1, SB[0x2D] clr b0/set b1, da9f. Drop SB[0x66]/[0x9E]. |

---

## 5. FULL STOCK BOOT CALL-ORDER (reproduce wholesale)

```
1.  CC32 |= 1                          (FUN_5418 @ 0x2F85)               [O8]
2.  boot_phy_bringup_early @0xCE79      (d0d3 SBU, cf28, ed02, C233, cc10 settle, d996 tunnel pre-stage)  [OK, but complete d996 T9]
3.  boot_analog_cfg @0x5284             (C65B/C656/C65B/C62D)            [O9]
4.  boot_hw_init_main @0x4FB6:
      4a. 0x5305 (→4c40, e795 latch, 0x07F6=1)
      4b. e597
      4c. e14b   (C8A6=4; 0x7000|=0x1C; b8xx RAM seed)                  [O11]
      4d. init_sys_flags @0x4BE6  — FULL:
            0x07F0-0x07F5 const (0x24,0x04,0x17,0x85,0,0)               [O10]
            CC35 &= 0xFE
            C801=(&0xEF)|0x10; C800=(&0xFB)|4; CA60=(&0xF8)|6; CA60=(&0xF7)|8; C800|=1
            CC3B |= 1; CC3B=(&0xFD)|2                                   [O10: bit0 missing]
            cb37 (bank1 connect-state init: 0x0A5B/0x072C/0x074E)       [O10]
            ef24 (db0d + 8E31 SB-PHY config)                           [T1,T3]
            ef1e (d0ac + 9A63 page-0x78 lane arm)                      [T2]
      4e. 92C5   (RAM-state seed 0x0AE3/0x0AE5/0x0AE9=0x0F/0x0AEE=3...)  [O6]
      4f. 8D77   (0x09F4-0x09F8 USB4 cap seed)
5.  boot_usb4_vs_usb3_mode_decision @0xB1CB:
      usb_pipe_engine_init (PIPE cfg) + cc10(subcmd4,CC12=1,CC13=0x8F) arm + E318.4 wait
      read m=(91C0&0x18)>>3; if m==2 && 0x09F9==4 → bbb6, set 0x09FA   (NOT hardcode 0x87)  [boot DEVIATION/high]
6.  if (0x09F9 & 0x83)  USB4 branch:
      e56f   (bank1 router/CM init, gated 0x09F9&0x81)                 [O12]
      b230   (sb_lane_flip_init — full)                               [T4]
      d894   (bc8f clr; C809|=0x02 PD-int; b031; bcb1 clr)            [O7]
      baa0   (== pd attach / pd_keystone_init equivalent)
7.  super-loop @0x2FB4  (polled CM/PD step machine: E1C6, C7A5, CB10, EEA5, 0x0A59 selector, CC17.1 service)  [T10]
```
Note: PIPE+arm (B1CB) runs AFTER init_sys_flags/92C5/8D77/analog — handmade runs the arm far too early and pcie_power_on (with sleep) before it.

---

## 6. CONCRETE IMPLEMENTATION SEQUENCE (cheapest-first, each HW-observable)

Each step is independently testable on the wire (UART E302/C80A prints + CY4500). Apply and flash one at a time; do NOT batch — several of these can mask or unmask each other.

**Phase A — stop the active sabotage (cheapest, highest expected unblock, no bank1 RE):**
1. **Fix `sleep()`** → retarget to Timer1 CC16-CC19 in registers.h (I1). Observable: PHY mailbox no longer perturbed; E302 should stop collapsing during boot/super-loop delays.
2. **Gate `pcie_power_off`/`pcie_power_on` + `usb_phy_tune` behind `!(0x09F9&0x83)`** (I2,I3,I4). Observable: tunnel regs (B480/B430/E764) stay as boot_phy left them; no 20× subcmd-4 storm. Keep the deferred sb_tunnel_up_pending trigger.

**Phase B — restore the interrupt engine (the keystone, no bank1 RE for the core):**
3. **Add UNGATED C806.0 timer-tick to `int1_isr` + implement `cc_pd_timer_tick`@B4BA** (O1). Wire CC91.1→0x07BB=1/0x09FA=4/`usb4_mode_entry_commit()`/0x0AE2, and CC81.1→hard/full reset. W1C all six channels. Also add the CC33.2 ack (O2), C80A.4 ack/C105 (O4), C806.4 ack (O5), and the EC06.0 E7E3 clear (O3). Observable: CC91/CC81 edges now drive mode commit + re-elicitation; watch for sustained C80A.5 / E302 holding.

**Phase C — fix the boot state the engine reads:**
4. **Add 0x92C5 RAM-state seed** (O6) before boot_phy/d436 and before the fork. Observable: width=0x0F, mode=3 now valid where stock has them.
5. **Add CC32|=1 (O8), 0x5284 analog (O9), the 0x07F0 block + CC3B|=1 + cb37 (O10), C809|=0x02 / D894 (O7), e14b C8A6/0x7000 (O11)** and **reorder main() to the §5 sequence**. Replace the hardcoded `0x09F9=0x87` with the B1CB mode latch (read 91C0&0x18). Observable: USB4-mode classification now real; C809.1 set.

**Phase D — complete the SB-PHY RX + connect tail (requires bank1 disasm — highest churn, do last):**
6. **Complete T1/T2/T3** (ef24/ef1e SB-PHY RX) — the path that lets the host's sideband connect raise C80A.5. Disasm via `emulate/disasm8051.py` at `file_off = addr - 0x8000 + 0xFF6F`.
7. **Complete T4/T5/T6** (b230/bb37/b7a4) and **fix sb.h plane/sense bugs I11/I12/I13**.
8. **Restore `usb4_connect_u4` a415 pre-gate (O16) + a48c tail T7** with e7ae correctly GATED on 0x0AF1.4 (not ungated), plus EnterMode→connect handoff O14/O15. Fix the contract-path PD bugs I5/I6/I7 in parallel (cheap, independent).

**SINGLE HIGHEST-VALUE EXPERIMENT TO RUN NEXT:**
Do **Phase A (steps 1-2) and Phase B step 3 together as one flash**, then capture UART + CY4500. Rationale: S1 (missing timer-tick) and S2/S3 (sleep/power_on mailbox corruption) are mutually reinforcing — the timer-tick can't drive the attach handshake if `sleep()`/`pcie_power_on` keep corrupting the PHY mailbox it depends on, and conversely a clean mailbox is useless if nothing services the CC91 attach edge. Fixing the interrupt SOURCE while simultaneously removing the interference that poisons the mailbox it feeds is the minimal combination most likely to produce the first sustained C80A.5 + E302 hold. **Decision rule:** if E302 now sustains but C80A.5 still never fires → the blocker is the truncated SB-PHY RX (Phase D step 6, T1/T2/T3). If C80A.5 fires but the GPU isn't reached → the blocker is the connect-tail truncation (Phase D step 8, T7: c270 DROM latch + EE82 tunnel link-up).

Key file paths: `/home/batman/asm2464pd-firmware/handmade/src/main.c` (ISR, boot order, sleep), `/home/batman/asm2464pd-firmware/src/include/registers.h` (CC10-13 mislabel, Timer1 retarget), `/home/batman/asm2464pd-firmware/handmade/src/pd.h` (0x09F9=0x87, init_sys_flags subset), `/home/batman/asm2464pd-firmware/handmade/src/usb4_irq.h` (ef24/ef1e truncations), `/home/batman/asm2464pd-firmware/handmade/src/sb.h` (b230/bb37/b7a4, plane bugs), `/home/batman/asm2464pd-firmware/handmade/src/vdm.h`+`usb4.h` (connect_u4 tail), `/home/batman/asm2464pd-firmware/handmade/src/pd_dispatch.h` (PS_RDY/RDO/timer-arg bugs). Bank1 disasm: `emulate/disasm8051.py` at `file_off = addr - 0x8000 + 0xFF6F`.