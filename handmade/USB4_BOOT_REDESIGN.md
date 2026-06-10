<!-- bank1-RE-enabled boot redesign (wf_6fb150d1): present USB4, suppress USB2 fallback, train E302. Implement cheapest-first per Section 5. -->

# Handmade ASM2464PD Boot/Upstream-Link Redesign: Present USB4 + Train E302

## Verification notes (read first — two load-bearing facts I confirmed against the tree, not just the reports)

- **CC10–CC13 is NOT a timer in this chip — it is the PHY/PD command mailbox, and handmade already drives it.** `handmade/src/include` resolves to `/home/batman/asm2464pd-firmware/src/include/registers.h`, where `0xCC10/11/12/13` are *mislabeled* `REG_TIMER0_DIV/CSR/THRESHOLD_HI/LO`. But `pd_dispatch.h:190-198` already uses those exact addresses as the PHY command mailbox for PD-message TX (`CC10=(&0xF8)|3; CC11=1; poll CC11.1; CC11=2`), which is byte-for-byte the `phy_link_train_cmd_cc10`/`phy_cc11_ack_event` protocol the reports describe (subcmd in `CC10[2:0]`, `CC11`=trigger/done/ack). **This means handmade's `sleep()` in `main.c` writes the same CC10/CC11/CC12/CC13 hardware** (`REG_TIMER0_*`). That is a real resource conflict (see Risks) and confirms the mailbox is reachable.
- **The PD/VDM/Enter_USB/SB path is already wired** (`vdm.h:268/281-286`, `usb4.h:30-49`, `sb.h`): Enter_USB Accept sets `0x07BA=1`, prints `[Connect_U4]`, and calls `usb4_connect_u4 -> sb_lane_flip_init(b230) + sb_block_init(bb37)`. So the trigger_chain report's "add the SB tail" work is **done**. What is missing is the **boot-side USB4 presentation** so the host gets that far without first SS_FAILing to USB2.

**Conflict flagged:** the four reports unanimously agree E302 is HW-set and there is **no runtime fw CC10 lane-train** on the happy path (trigger_chain is authoritative; cc10_phy's "subcmd 5 per-rate train" is actually PD-TX, as trigger_chain proves via the `df47` caller tree → `pd_queue_ctrl_msg @0xE529`). I adopt the **negative result**: do **not** add a runtime CC10 subcmd-4/5 lane-train. The keystone is boot presentation + the already-present SB assert.

---

## 1. Why E302 doesn't train today, and what trains it

**Precise mechanism (two independent failure points, both in `main.c`):**

1. **Boot presents a USB3 *device*, not a USB4 *upstream PHY*.** `main()` calls `usb_init_controller(0)` (`usb.h:166`) unconditionally. That routine does `REG_POWER_STATUS(0x92C2) &= ~POWER_STATUS_USB_PATH; REG_USB_CONFIG(0x9002)=0xE0; REG_USB_EP0_CFG(0x9005)=0xF0; REG_USB_EP_MGMT(0x905E)=0; REG_BUF_CFG_9303=0x33` — i.e. it brings up the SuperSpeed **device** descriptor/EP engine and *clears* `92C2.6`. On a TB4/AMD host the SS device link-trains and **SS_FAILs** (the host wants a USB4 router, not a USB3 device).
2. **The SS_FAIL handler force-drops to USB2 and kills USB4.** `int0_isr`'s `USB_PERIPH_LINK_EVENT` branch (`main.c`, `[USB2 fallback]`) does on `BUF_CFG_9300_SS_FAIL`: `is_usb2=1; REG_CPU_MODE(0xCC30)=CPU_MODE_USB2(0x00); REG_USB_PHY_CTRL_91C0=0x10`. This is exactly the stock `usb_ss_link_train_engine @0x9C2B` USB2-fallback (`91C0|=2; CC30; E710`), **but stock only ever reaches it via the `0x0A59==2` USB-device super-loop branch, which USB4 mode never takes** (usb2_avoid: master gate @0x2FD1, `0x0A59=1` when `0x0AE8==0 && 0x09FA==4`). Forcing `CC30=USB2 / 91C0=0x10` makes the host see a USB2-HS device and abandon USB4 lane training.
3. **The upstream USB4 PHY is never armed.** Handmade never runs the stock boot arm `boot_usb4_vs_usb3_mode_decision @0xB1CB` (PIPE-engine program + pulse `91C0.0` + `cc10(subcmd=4, CC12=1, CC13=0x8F)` + wait `E318.4`), nor `boot_phy_bringup_early @0xCE79`. So `(91C0&0x18)` never reaches `0x10`, `E318.4` never asserts, and the upstream lane engine that ultimately sets `E302` is never live (cc10_phy ROOT CAUSE; boot_fork Q4).

**What trains E302 (the actual chain):** E302 is read-only HW-set (all 4 sites are reads: `a6ef/b394/be8b/bf04`). It climbs to `(E302>>4)&3 == 3` when, **after** the device ACCEPTs PD Enter_USB[USB4] and **asserts the USB4 sideband** (`b230 sb_lane_flip_init` + `bb37 sb_block_init`: `SB[0x81]=0x08`, `SB[0x01].6/.7`, `SB[0x66]=0x20`, `SB[0x9e]=0x20`, lane map per `C6DB.0`, `E7FC` bond gate), the **host's connection manager** polls those SB bits and trains the upstream lanes. There is **no device-side CC10 lane-train on the happy path** (trigger_chain NEGATIVE result: the only runtime CC10-subcmd-4 is the bank1 `0xADA9` handler reached *only* via the host-command LJMP table `@0xD2F3`, RHMG/RSMG/RXCM/SBER/TXCM — do not self-invoke it). The "runtime trigger" is the SB assert, which handmade already emits. The **boot prerequisite** that lets the host get to Enter_USB without SS_FAILing first is the B1CB PIPE-arm + suppressing the USB2 fallback.

**Net:** E302 stays 0 because (a) handmade presents/forces USB2 before the host can attempt USB4, and (b) the upstream PHY (`E318.4`/B1CB arm) is never brought up. The SB assert (which is present) has nothing to train against.

---

## 2. The boot redesign — present USB4, not USB3

### Keep / replace / reorder in `main()`

| Current call | Action | Why |
|---|---|---|
| `REG_UART_LCR`, `flash_init()` | **KEEP** (first) | UART + OTP serial. Unchanged. |
| `usb_phy_tune()` (C280/C300 SERDES lane tune) | **KEEP** | Lane analog tune; needed for both USB3 and USB4. |
| PCIe TLP consts + `pcie_apply_x2_rxphy_tuning()` + `pcie_power_off()` | **KEEP** | Direct-PCIe path for tinygrad; unrelated to upstream link. |
| `pcie_power_on()` | **KEEP** (but see Risks — consider deferring) | Downstream GPU bring-up for tinygrad. |
| **`usb_init_controller(0)`** | **REPLACE with mode fork** (below) | This is the USB3-device bring-up that SS_FAILs → USB2. |
| `pd_keystone_init()` | **KEEP**, but **fix the 0x09F9 seed** (below) | Arms PD engine + Type-C attach. |
| 0x0B02-0x0B1F / 0x06F1 / 0x07ED zero-init | **KEEP** | M0 prereqs; correct. |
| `IE = …` | **KEEP** (after the fork) | |
| super-loop | **KEEP**, extend M1 check | |

### New boot order (concrete)

```
flash_init();
usb_phy_tune();
... PCIe consts, tuning, power_off, power_on ...      // unchanged (tinygrad path)

usb_pipe_engine_init();      // NEW: stock B1CB PIPE config, UNCONDITIONAL (both modes)
usb4_phy_arm();              // NEW: the cc10(subcmd=4) upstream arm + E318.4 wait + DECIDE
pd_keystone_init();          // KEEP (sets 0x09F9 = 0x87 — fixed)

// fork:
if (XDATA_REG8V(0x09F9) & 0x83) {     // USB4 mode (0x87) — do NOT bring up a USB3 device
    usb4_boot_init();                 // NEW: e56f/b230/d894/baa0 equivalents already in sb.h/pd.h
} else {
    usb_init_controller(0);           // USB3 host: keep tinygrad SS bring-up unchanged
}

... 0x0B02/0x06F1/0x07ED zero-init, IE=, i2c, super-loop ...
```

### `usb_pipe_engine_init()` — stock B1CB PIPE config, run UNCONDITIONALLY (cite `boot_usb4_vs_usb3_mode_decision @0xB1CB`)

All three of boot_fork, cc10_phy, usb2_avoid agree this exact set is identical for USB3 and USB4 and is the only USB-engine programming stock does at boot. Copy verbatim:

```
REG_POWER_ENABLE(0x92C0)   = (REG_POWER_ENABLE & 0x7F) | 0x80;
REG_USB_PHY_CTRL_91D1      = 0x0F;
REG_BUF_CFG_9300           = 0x0C;
REG_BUF_CFG_9301           = 0xC0;
REG_BUF_CFG_9302           = 0xBF;
XDATA_REG8(0x9091)         = 0x1F;        // (REG_USB_CTRL_PHASE addr; raw write, boot-time)
REG_USB_EP_CFG1(0x9093)    = 0x0F;
REG_USB_PHY_CTRL_91C1      = 0xF0;
REG_BUF_CFG_9303           = 0x33;
REG_BUF_CFG_9304           = 0x3F;
REG_BUF_CFG_9305           = 0x40;
REG_USB_CONFIG(0x9002)     = 0xE0;        // == USB_CONFIG_MSC_INIT (handmade already uses this)
REG_USB_EP0_CFG(0x9005)    = 0xF0;
REG_USB_MODE(0x90E2)       = 0x01;
REG_USB_EP_MGMT(0x905E)   &= ~0x01;
REG_USB_MSC_CTRL(0xC42C)   = 0x01;        // NOTE: C42C/C42D are MSC regs here, NOT a "USB4 cfg"
REG_USB_MSC_STATUS(0xC42D)&= ~0x01;
REG_USB_PHY_CTRL_91C3     &= ~0x20;
REG_USB_PHY_CTRL_91C0     |= 0x01;        // pulse 91C0.0
REG_USB_PHY_CTRL_91C0     &= ~0x01;
```

> **Conflict to flag:** the B1CB list writes `C42C=1; C42D.0=0` (boot_fork/cc10_phy). In handmade's registers.h those addresses are `REG_USB_MSC_CTRL/STATUS`. Whether B1CB's `C42C` is the MSC trigger or a DMA-engine strap is unverified — copy the value verbatim but do not assume the MSC semantics. Also note handmade already does `905E`/`9303` in `usb_init_controller`; consolidating them here is fine.

### `usb4_phy_arm()` — the upstream USB4 PHY link arm (cite `phy_link_train_cmd_cc10 @0xE50D`, arm site `B1CB/b249`)

This is the missing keystone arm. Use the **same CC10 mailbox handmade already drives in `pd_dispatch.h`**:

```
// ack any prior event (phy_cc11_ack_event @0xE8EF):
REG_TIMER0_CSR(0xCC11) = 0x04;            // CC11=4
REG_TIMER0_CSR(0xCC11) = 0x02;            // CC11=2
// issue subcmd=4, CC12=0x01, CC13=0x8F (b249):
REG_TIMER0_DIV(0xCC10) = (REG_TIMER0_DIV & 0xF8) | 0x04;
REG_TIMER0_THRESHOLD_HI(0xCC12) = 0x01;
REG_TIMER0_THRESHOLD_LO(0xCC13) = 0x8F;
REG_TIMER0_CSR(0xCC11) = 0x01;            // go
// WAIT (b252): E318.4 (PHY link-up) OR CC11.1 (done), bounded:
{ uint16_t g=0; while(!((REG_PHY_COMPLETION_E318 & 0x10) || (REG_TIMER0_CSR & 0x02)) && ++g<0xFFFF); }
// ack (E8EF):
REG_TIMER0_CSR(0xCC11) = 0x04;
REG_TIMER0_CSR(0xCC11) = 0x02;
```

(I deliberately reuse handmade's existing `REG_TIMER0_*` names since they ARE the CC10 mailbox; rename them to `REG_PHY_CMD_*` when you touch registers.h — see Risks R-timer.) **Values are verbatim** from the reports (subcmd=4, CC12=1, CC13=0x8F); do not invent. `0x8F` is an opaque HW selector — copy as-is (cc10_phy OPEN).

**Optional but recommended (cite `boot_phy_bringup_early @0xCE79`):** precede `usb4_phy_arm()` with the early settle `cc10(subcmd=2,CC12=0,CC13=0x14)` then `cc10(subcmd=3,CC12=0,CC13=0x0A)` + wait `E712[1:0]`. boot_fork/cc10_phy both list this as a likely precondition for `(91C0&0x18)==0x10`. Add it only if M1 fails without it (it touches Type-C/SBU which the PD path may already cover).

### 0x09F9 fix in `pd_keystone_init()` (cite `usb4_cap_apply_09f9 @0x8D77`)

boot_fork is explicit and the other reports concur: the runtime VDM/route gate must be **0x87**, not 1.
```
PR(0x09F9) = 0x87;   // bit7=VDM-ACK enable, bit6 CLEAR (normal USB4, no captive latch), bits2:0=mode3
// optionally seed: 0x09F4=3,0x09F5=1,0x09F6=1,0x09F7=3,0x09F8=1,0x09FB=3 (as 8D77 does)
```
`0x87` makes `(0x09F9&0x83)!=0` (USB4 handlers reachable AND the boot fork takes the USB4 branch), bit7 set (VDMs ACK — your VDM responder needs this), and `0x09FA=0x09F9&3=3` (tunnel route). **Check `pd.h:211-215` — the comment there mentions a prior `0x01`; set the actual write to `0x87`.** Forcing 1 NAKs all Discover/EnterMode VDMs (boot_fork Q3).

### `usb4_boot_init()` — the USB4-only boot block (cite main fork `@0x2F97`, gated `if (0x09F9 & 0x83)`)

Stock runs only `e56f` (router init) / `b230` (SB lane-flip) / `d894` (`bc8f&0xFD` RMW) / `baa0` (`pd_cc_attach_term_setup`). **None brings up a USB device.** `b230` and the PD attach (`baa0`) are already in handmade (`sb.h` / `pd.h` — `pd_keystone_init` already does the attach-term). So this block is largely satisfied by `pd_keystone_init()`; the only addition is the router-init `e56f` (bank1) if M1 needs it — defer until M1 baseline is measured.

### USB2-fallback suppression (cite handmade `int0_isr` `[USB2 fallback]` vs stock `ca0d @0xCA0D`)

Guard the SS_FAIL branch so it never fires in USB4 mode. In `int0_isr`'s `USB_PERIPH_LINK_EVENT`:
```
if (ep & BUF_CFG_9300_SS_FAIL) {
    if (!(XDATA_REG8V(0x09F9) & 0x83)) {     // ONLY fall back when NOT in USB4 mode
        is_usb2 = 1;
        REG_CPU_MODE = CPU_MODE_USB2;
        REG_USB_PHY_CTRL_91C0 = 0x10;
    }
    // else: W1C-ack the SS event (REG_BUF_CFG_9300 = ep below) but do NOT drop to USB2
}
```
This matches usb2_avoid's "GUARD all USB2-fallback writes with `if (!(0x09F9 & 0x83))`" and boot_fork's DO-NOT list (no `CC30=USB2`, no `91C0=0x10` in USB4 mode). The `REG_BUF_CFG_9300 = ep` W1C-ack stays (safe; stock `d916` acks without CC30).

---

## 3. Post-Enter_USB lane-train step handmade must add

**None at the CC10 level — and this is the load-bearing conclusion (trigger_chain, high conf, corroborated by USB4_TUNNEL_PLAN §4).** There is **no software CC10 lane-train** on the Enter_USB happy path. `usb4_connect_decide @0xA0A7` issues zero CC10 ops; its annotation says "E302 link-mode is set by HARDWARE; fw only polls it." The one runtime CC10-subcmd-4 (bank1 `0xADA9`) is a host-CM command handler behind the `0xD2F3` LJMP table — **do not self-invoke it** (driving it out-of-band desyncs the PHY engine).

What handmade must add post-Enter_USB is **already added**: run `usb4_connect_u4 @0xA3F5` to completion **including** the synchronous SB tail `b230 + bb37` (present in `usb4.h:30-49` → `sb.h`). Verify on HW that `[flp=NN][SB Init]` prints. The SB block IS the trigger; the host then trains the lanes and HW sets `E302`. If you observe `[Connect_U4]` but no `[SB Init]`, the bug is the `0x07BA`/`0x07ED` gating or an `0x0AF1` sub-block gate (R3), not a missing CC10 call.

(`df47`/`d916` subcmd-5 are PD-TX-over-CC10, NOT lane train — confirmed via the `pd_queue_ctrl_msg @0xE529` caller tree. The cc10_phy report's "subcmd 5 per-rate train" label is **wrong**; trigger_chain overrides it.)

---

## 4. Milestone M1' and fallback diagnostic

**M1' = E302 trains: `(REG_PHY_MODE_E302(0xE302) >> 4) & 3 >= 2`.**

The super-loop already prints E302 (`[U <E302>:<C80A>:<EC06>:...]`). Add an explicit pass print:
```
uint8_t e302 = XDATA_REG8V(0xE302);
if (((e302 >> 4) & 3) >= 2) uart_puts("[*** USB4 TRAINED ***]\n");
```
**Pass sequence on UART:** `[Connect_U4]` → `[flp=NN]` → `[SB Init]` → within ~ms the `[U …]` line shows E302 high nibble go `0x0_ → 0x2_/0x3_` and `C80A` bit5 (SB-router, `0x20`) sets. This matches the captured stock order (trigger_chain: `Enter_USB 4 → Connect_U4 → [flp] → [SB Init] → *** USB4 Gen3 x2 ***`).

**Fallback diagnostic (if E302 stays 0) — dump in this order:**
1. **Did SB assert run?** Confirm `[flp=NN][SB Init]` printed. If `[Connect_U4]` but no `[SB Init]`: `0x07BA`/`0x07ED` gate or `0x0AF1` sub-block gate. Dump `0x07BA`, `0x07ED`, `0x0AF1`, `0x09F4` (vendor read 0xE4).
2. **Is the PHY arm up?** Dump `E318` (expect bit4 set after `usb4_phy_arm`) and `91C0` (expect `(91C0&0x18)==0x10` if the host plugged USB4). If `E318.4`=0, the B1CB arm didn't link — add the `boot_phy_bringup_early` settle (§2 optional) and re-check.
3. **Did we accidentally drop to USB2?** Dump `is_usb2`, `CC30` (must NOT be 0x00 in USB4 mode), `91C0` (must NOT be 0x10), `9300` (SS_FAIL bit2). If any USB2 toggle is set, the suppression guard didn't take.
4. **Mode gate correct?** Dump `0x09F9` (must be `0x87`), `0x09FA` (must be `3`). If `0x09F9` lost bit7, VDMs are NAKing → host never reaches Enter_USB.
5. **Did the host raise SB?** Dump `C80A` bit5 and `usb4_int_seen`. If the host never raises C80A.5 after `[SB Init]`, the SB block bits/ROM tables are wrong (R1/R2) — compare SB page-1 `0x01xx`/DROM `0x213d` against a stock MMIO trace.

---

## 5. Risks + implementation order

**Risks:**

- **R-timer (HIGH, must fix first):** `sleep()` in `main.c` writes `REG_TIMER0_DIV/CSR/THRESHOLD` = the **same `CC10/CC11/CC12/CC13` mailbox** the PHY arm and PD-TX use. Every `sleep(500)` in the super-loop issues a CC10 "command" (`CC10[2:0]=mode 0x04`, `CC11=1`) — colliding with `pd_dispatch.h`'s PD-TX (CC10 subcmd 3) and any PHY arm. **Action:** confirm on HW whether `CC10` is genuinely a dual timer/PHY mailbox or two distinct blocks (the byte protocol is identical, which is suspicious). If shared: (a) serialize — never `sleep()` between a CC10 PHY/PD issue and its `CC11.1` ack; (b) consider replacing `sleep()` with a different timer or a busy NOP loop. This is the single highest-risk interaction and must be resolved before adding `usb4_phy_arm()`. **Do not proceed to the arm until the timer/mailbox aliasing is confirmed by HW.**
- **R-USB3 regression (tinygrad):** the whole point is that the mode fork keeps `usb_init_controller(0)` for USB3 hosts. Risk: the `0x09F9 & 0x83` test mis-classifies a real USB3 host as USB4 and skips the device bring-up. Mitigation: the boot fork mirrors stock's `(91C0&0x18)==0x10 && 0x09F9==4` decision (boot_fork OPEN) — only force USB4 when the host actually presents a USB4 plug; otherwise fall through to `usb_init_controller`. Test `DEV=USB` (no AMD) on a plain USB3 host first.
- **R-PCIe regression:** `pcie_power_on()` is unchanged and orthogonal, but commit `c3e537b` notes PCIe enum was broken before. Keep `pcie_power_on()` exactly as-is; do not let the USB4 PIPE config touch B4xx/E764.
- **R-host-timing (host-interactive):** `E318.4` vs `CC11.1` race after the arm, and whether the host trains from `[SB Init]` alone or needs a CM router-op, are not statically provable (cc10_phy/trigger_chain OPEN). Bounded waits everywhere; never spin forever.
- **R-PIPE-double-program:** `usb_pipe_engine_init` and the old `usb_init_controller` both write `9002/9005/905E/9303`. Make sure only ONE runs per boot (the fork guarantees this).

**Implementation order (so USB3+tinygrad never breaks):**

1. **Resolve R-timer** (HW): confirm CC10 timer/mailbox aliasing. Gate everything else on this.
2. **Guard the USB2 fallback** in `int0_isr` with `if (!(0x09F9 & 0x83))`. *Low risk, reversible.* Re-run tinygrad (`DEV=USB+AMD` falls to USB2 today; with `0x09F9=0x87` it now won't, so verify the USB3-host case `DEV=USB` still falls back correctly when `0x09F9` is NOT USB4).
3. **Fix `0x09F9 = 0x87`** in `pd_keystone_init`. Verify VDM ACKs on the wire (CY4500) still pass.
4. **Add the boot fork** (`if (0x09F9 & 0x83) usb4_boot_init(); else usb_init_controller(0);`) — replace the unconditional `usb_init_controller(0)`. At this stage measure M1' baseline: with SB already asserting and USB2 suppressed, **does E302 climb without the PHY arm?** (boot_fork/usb2_avoid OPEN — this is the cheapest experiment; the SB assert may be sufficient.)
5. **Only if step 4 doesn't train E302:** add `usb_pipe_engine_init()` (B1CB PIPE config) + `usb4_phy_arm()` (cc10 subcmd-4 + E318.4 wait). Re-measure.
6. **Only if still not training:** add `boot_phy_bringup_early` settle (§2 optional) and/or the `e56f` router-init. Use the §4 diagnostic dump to localize.

Each step is independently UART-observable; do not stack. The fork in step 4 is what protects tinygrad: a USB3 host (`0x09F9` not `0x83`) always reaches the unchanged `usb_init_controller(0)` + USB2 fallback path.

**Files to touch:** `/home/batman/asm2464pd-firmware/handmade/src/main.c` (boot fork + ISR guard + M1 print), `/home/batman/asm2464pd-firmware/handmade/src/pd.h` (0x09F9=0x87 in `pd_keystone_init`), `/home/batman/asm2464pd-firmware/handmade/src/usb.h` (new `usb_pipe_engine_init` + `usb4_phy_arm`, or a new `usb4_boot.h`), `/home/batman/asm2464pd-firmware/src/include/registers.h` (rename `REG_TIMER0_*` at 0xCC10-13 to `REG_PHY_CMD_*`; add `E318` bit4 = PHY-link-up). The SB/connect tail in `usb4.h`/`sb.h` needs **no change** — it is already the correct E302 trigger.