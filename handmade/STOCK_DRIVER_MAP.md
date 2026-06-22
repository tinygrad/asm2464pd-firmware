# Stock ASM2464PD Driver Map (fw_tinygrad.bin) — high-level architecture

This document is a subsystem-by-subsystem map of the **stock** ASM2464PD firmware
(`fw_tinygrad.bin`), reverse-engineered from Ghidra (with handmade symbol names
ported) and cross-checked against the handmade C re-implementation in
`handmade/src/`. Its purpose is to give the handmade port a single architectural
reference: what every boot step, ISR, superloop handler, and state machine does,
and — critically — **which parts are required for the PCIe-over-USB4 GPU tunnel
vs. which are NVMe-storage-only and can be omitted**.

> **All addresses are stock Ghidra addresses** (valid `fw_tinygrad.bin` body
> offsets). `CODE:` = bank0/common flat space (`0x0000–0xFF69`);
> `CODE_BANK1::` = the banked bank1 space (decompile via the
> `CODE_BANK1::<addr>` prefix). Symbol names are the handmade-ported names.

## Classification legend

| Class | Meaning |
|---|---|
| **GPU-PATH** | Directly required to bring up the GPU PCIe tunnel (lane bond, SB transport, router-op mailbox, tunnel adapter, tunnel-link events). |
| **SHARED** | Infrastructure used by **both** the GPU tunnel and NVMe storage (clock/PHY/SB/PCIe-tunnel link engine, config-TLP engine, CM arm/init). Must be kept. |
| **SHARED?** | Tagged SHARED but **needs a second look** before relying on it / omitting it — its gate or dependency may have been satisfied only by the NVMe path. Re-verify against handmade. |
| **NVMe-ONLY (OMIT)** | Storage data-path / NVMe-enclosure enumeration. Not on the GPU path; documented but omittable. Several of these are exactly what handmade dropped (the `c00d`-vs-`pcie_power_on` PERST race). |
| **PD** | USB-PD / Type-C / Structured-VDM / Enter_USB handshake. Precedes and is required for USB4, but is its own subsystem. |
| **USB-DEV** | USB device enumeration / EP machinery / SS link train. Storage-transport adjacent, not GPU. |
| **HOUSEKEEPING** | crt0, timers, unused vectors, version bytes, UART console. |

---

## 1. Boot / Init

One-time boot runs with interrupts **disabled**, in a fixed order, then `main`
makes the USB4-vs-USB3 mode decision, runs the USB4-only init fork, enables
interrupts, and enters the superloop.

### Entry points

| Addr | Name | Role | Class |
|---|---|---|---|
| `CODE:0000` | RESET | crt0: zero IRAM `0xFF..0x00`, SP=`0x72`, call `0x030a`, run cstartup table @`0x0648` (XDATA/SFR seed via `r3_write_dispatch`), → main | HOUSEKEEPING |
| `CODE:0003` | int0_isr | INT0/EX0 USB-device EP + PCIe-link + NVMe-queue ISR | USB-DEV |
| `CODE:000b` | TIMER0 | Timer0 vector (stub) | HOUSEKEEPING |
| `CODE:0013` | int1_isr | INT1/EX1 USB4/PD/CM ISR | SHARED |
| `CODE:2f80` | main_boot_and_superloop | boot init → mode decision → USB4 fork → enable ints → superloop | SHARED |

### Functions (ordered boot chain)

| Addr | Name | Role | Class | Calls |
|---|---|---|---|---|
| `CODE:2f80` | main_boot_and_superloop | Boot orchestrator. ORDERED: (1) `0x0A59=0`; (2) set `CC32.0`; (3) `boot_phy_bringup_early`; (4) `hddpc_phy_init`; (5) `e971` stub; (6) `boot_hw_init_main`; (7) `boot_usb4_vs_usb3_mode_decision`. Then USB4-only fork on `(u4_mode_flag&0x83)`. | SHARED | ce79, 5284, 4fb6, b1cb, e56f, b230, d894, baa0 |
| `CODE:ce79` | boot_phy_bringup_early | **FIRST main HW action (step 3). SERDES/Type-C-SBU/PHY bring-up KEYSTONE** — powers the sideband transport on SBU pins. Without it SB reads 0 and E302 never trains. | SHARED | d0d3, cf28, ed02, cc10-wait, cc11-ack, dd42, d996 |
| `CODE:5284` | hddpc_phy_init | Step 4. HDDPC/PHY analog cfg (PHY_EXT_5B/2D, HDDPC_CTRL). | SHARED | (direct SFR) |
| `CODE:4fb6` | boot_hw_init_main | Step 6 master HW init chain (see notes for full order). WAIT on `C6B3&0x30` PHY/clock-ready gate, then tunnel-adapter-enable + DMA cfg + tail. | SHARED | 5305, e597, e14b, 4be6, 92c5, 8d77, de16, eef9, cd6c, d127, bf8e |
| `CODE:5305` | boot_ramstate_reset_and_analogcfg | `4c40` RAM reset; sample E795 strap → `d6bc` analog; `7F6=1`. | SHARED | 4c40, d6bc |
| `CODE:4c40` | boot_ramstate_reset | Zero PD/USB RAM. **KEY defaults: `0x09F9=0x04`, `0x09FA=0x04`, `0x09FB=0x00`** (the 'USB4-capable' default cap — NOT from OTP/flash). | SHARED | r3_write_dispatch |
| `CODE:e597` | e597 UART/clk init | UART_FCR/LCR, CPU_CTRL_CA2E.0 — console enable. | HOUSEKEEPING | (direct SFR) |
| `CODE:e14b` | bank0_e14b SPI flash read engine | FLASH_DIV=4; SPI engine setup; reads flash header @`0x7000`. Path used by `8d77`/`92c5` blob loads. | SHARED | b8b9, b820, be02, b833, b881, e3f9, df47 |
| `CODE:4be6` | pd_int1_enable_group | INT1 enable group + fwver `7F0..7F5 = 24 04 17 85 00 00`; arms USB4/CM INT1 sources (`ef24`/`ef1e`). | SHARED | cb37, ef24, ef1e |
| `CODE:92c5` | bank0_92c5_seed | Link-default seeder (`u4_link_busy=1`, gen=3, lane=3); SPI blob override; PHY enable gates. | SHARED | e957, ece1, e5fe, dbbb |
| `CODE:8d77` | usb4_cap_apply_09f9 | SPI-cap-blob loader → runtime cap. Seeds `0x09F4=3/F5=1/F6=1(cap20g_gate1)/F7=3/F8=1`; maps `dp_alt_mode`→`u4_mode_flag` (3→0x87,2→0x06,1→0xC1,…). If `E795.5==0` forces `u4_mode_flag=4`. | SHARED | e957, dace/daeb/daf5, bbc7, 9d90 |
| `CODE:de16` | bank0_de16 | PHY-DMA cfg + timer + `sb_channel_connect_service`. | SHARED | e726, sb_channel_connect_service |
| `CODE:eef9` | bank1_eef9 | `INT_ENABLE |= 0x40` (bank1/USB4 INT group bit6). | SHARED | (none) |
| `CODE:cd6c` | pcie_tunnel_adapter_enable_b401 | PCIe-over-USB4 **TUNNEL ADAPTER ENABLE**. `CA06&=~0x10`; config B410-B42B; `B401|=1` (TUNNEL MASTER EN); `B480|=1` PERST; `B298.4` TLP-routing-en. | SHARED | c8db, r3_write_dispatch, 99e0 |
| `CODE:d127` | bank0_d127 | PCIe-DMA engine ring config (SIZE/BUF, DOORBELL, CEF2/CEF3, B281). | SHARED | (direct SFR) |
| `CODE:bf8e` | bank0_bf8e tail-init | Final tail: clear connect-state RAM; timers; `c00d` CM-arm/tunnel-power-on; `usb_msc_init`. | SHARED | c6a8, d47f, e19e, c00d, 4904 |
| `CODE:e19e` | init_timer2_and_timer4 | TIMER2/4 div + threshold boot base. | HOUSEKEEPING | e677 |
| `CODE:b1cb` | boot_usb4_vs_usb3_mode_decision | Step 7 PIPE-engine block + USB link arm. **DECISION: `(91C0&0x18)==0x10 && 0x09F9==4` → `0x09F9=1`(USB4) else `=2`(USB3)**. | SHARED | usb4_phy_set_enable, e214, cc10, cc11-ack, bbb6 |
| `CODE_BANK1::e56f` | bank1_e56f router-op engine init | USB4-only fork (`u4_mode_flag&0x81`). Arms the C0A5 router-op mailbox path (EC00, EA88/89 speed, EC05). | SHARED | cc10-wait stub |
| `CODE:d894` | bank0_d894 | USB4-only fork. Seeds INT/PCIe-tunnel adapter registers (bank-2 cluster incl `0x121E.0`). | SHARED | bc8f, b031, r3 dispatch |
| `CODE:baa0` | pd_cc_attach_term_setup | USB4-only fork. Type-C term + PD entry gated on `E795.5`; dispatch on flash cmd byte `0x7000`. | PD | ae87, b8c3, dd42, e6e7, pd_arm_cc_timer |
| `CODE:c00d` | bank0_c00d CM-arm/tunnel-power-on | PCIe-tunnel power-on (gated `0x6E6`). **Also runs from the superloop NVMe branch — its PERST/C659/width re-drive is the race handmade removed.** | SHARED | cd6c, 99e0, e8a9, d436, 39e4 |
| `CODE:4904` | usb_msc_init | NVMe/USB-MSC BOT init: `'USBS'` CSW sig @`0xD800`, MSC EPs, NVME_DOORBELL clears. | NVMe-ONLY | set/write helpers, c4b3 |

### State machines

| Name | State var | States / transitions | Class |
|---|---|---|---|
| u4_mode_flag (USB4 vs USB3 cap) | XDATA `0x09F9` | `0x04`=default (4c40) → `8d77` may set `0x87/06/85/C1` → `b1cb` overrides to `1`(USB4)/`2`(USB3) only if still `4`. Gate `(&0x83)!=0` enables USB4 init + USB4 INT1 handlers. | SHARED |
| u4_route_mode | XDATA `0x09FA` | `0x04`=USB4-ready sentinel; superloop polls `==4` to gate the USB4 connect path. | SHARED |
| USB-enum / superloop service (a59) | XDATA `0x0A59` | `0`=idle, `1`=USB4-connect armed (timer4 + 9037 NVMe walk + c00d), `2`=USB3/fallback (ca0d). | SHARED |
| c00d one-shot | XDATA `0x06E6` | `1`=arm pending (set by main head, boot_hw_init_main, post-9037), `0`=consumed. | SHARED |
| MSC BOT phase | IRAM `0x6A` | `0`=idle, `0x0B`=command-block ready → run `3419`/`180d` CBW/CSW. | NVMe-ONLY |

**Full boot order (Ghidra-confirmed):** RESET(crt0) → main: `0x0A59=0` → `CC32.0`
→ `boot_phy_bringup_early(ce79)` SBU/PHY keystone → `hddpc_phy_init(5284)` →
`e971` stub → `boot_hw_init_main(4fb6)` = [`5305`→`4c40` RAM reset+`0x09F9=4`,
E795→`d6bc`] → [`e597` UART] → [`e14b` SPI] → [`4be6` INT1 arm + fwver] →
[`92c5` link-default + SPI override] → [`8d77` cap-blob → mode_flag/0A4x] →
[`de16` PHY-DMA/SB-connect] → [`eef9` INT_ENABLE|=0x40] → **WAIT `C6B3&0x30`** →
[`cd6c` tunnel-adapter-enable] → `0x6E6=1` → [`d127` PCIe-DMA] → [`bf8e` tail:
timers + `c00d` + `usb_msc_init`] → `boot_usb4_vs_usb3_mode_decision(b1cb)`.

---

## 2. ISR Handlers

Only **two** interrupts are wired: INT0 (`0x0003`→`0x0e5b`, USB-device) and INT1
(`0x0013`→`0x4486`, USB4/PD/CM orchestrator). Slots `0x0B`/`0x1B`/`0x23`
(Timer0/Timer1/UART) have **no LJMP installed** — those interrupts are unused;
UART is polled.

### Entry points

| Addr | Name | Role | Class |
|---|---|---|---|
| `CODE:0000` | RESET | LJMP `0x431a` (C startup). Not an ISR. | HOUSEKEEPING |
| `CODE:0003` → `0x0e5b` | int0_isr | USB-device/peripheral EP + USB PHY (91D1) + buffer-cfg (9300/01/02) + NVMe-queue/MSC tail. | USB-DEV |
| `CODE:0013` → `0x4486` | int1_isr_orchestrator | TOP-LEVEL USB4/PD/CC/tunnel demux. **THE critical GPU-path ISR.** | SHARED |

### INT1 dispatch order (from `0x4486`)

| Trigger | Handler (trampoline → body) | Role | Class |
|---|---|---|---|
| C806.0 | `0x0520` → b4ba cc_pd_timer_tick | CC/PD timer-tick (CC23/81/91/99/D9/F9) | PD |
| CC33.2 (W1C=4 first) | `0x0390` → cd10 | PCIe-downstream link bringup/reset; CC31 spin | SHARED |
| C80A.6 | `0x052f` → af5e pd_rx_isr | PD message RX dispatch → `83d6` | PD |
| **C80A.5** (gated `0x09F9&0x83`) | `0x061a` → BANK1 a066 | **SB-router connect / lane-bond → tunnel** | GPU-PATH |
| C80A.4 (gated) | `0x0593` → c105 | Secondary USB4 adapter/link-width events | SHARED |
| **EC06.0** (gated; EC04=1 ack) | `0x0499` → BANK1 c0a5 | **CM router-op mailbox (config-TLP, tunnel reset)** | SHARED |
| **C80A.0-3** (gated) | `0x0570` → BANK1 e911 | **PCIe-tunnel link UP/DOWN (E763.2/.3)** | GPU-PATH |
| C806.4 | `0x0642` → BANK1 ef4e | Reserved/unused — empty RET stub | HOUSEKEEPING |

### Functions

| Addr | Name | Role | Class | Calls |
|---|---|---|---|---|
| `CODE:4486` | int1_isr_orchestrator | INT1 demux (order above). Saves/restores regs, RETI. | SHARED | b4ba, cd10, af5e, a066, c105, c0a5, e911, ef4e |
| `CODE:0e5b` | int0_isr body | USB-device ISR: EP walk (`5a6a`/`5442`), buffer-cfg events, USB PHY 91D1, USB3 ctrl-EP, USB-mode bringup; tail = C806.5 CPU-link + NVMe-queue service loop. | USB-DEV | 5442, 52a7, 3419, 180d, 1196, 488f, 4784, 49e9, 3e81 |
| `CODE:b4ba` | cc_pd_timer_tick | CC/PD timer-tick: 6 Type-C event regs. CC81.1 attach/detach; CC91.1 1s timeout (→ `0x09FA=4` + USB4 commit); CC99.1 role-dep. | PD | e3d8, d676, e90b, be8b, d78a, df79 |
| `CODE:af5e` | pd_rx_isr | PD-RX: `[PD_int:E40F:E410]`; E40F.0 msg-rx → `83d6`; .5 hard-rst; .7 soft-rst; E314 PHY-completion acks. | PD | 83d6, dfdc, e419, e439 |
| `CODE_BANK1::a066` | sb_router_event_handler_M2 | SB-router connect/lane-bond. Part1 per-channel connect (SB[0xC9]) + lane-bond-complete tunnel kick. Part2 `d4cd` poll + SB[0x2C/2D/66/9E/A0/A1/26] service. | GPU-PATH | c3b2, e52d, d4cd, sb_phy_link_bringup |
| `CODE:c105` | usb4_sec_adapter_link_event_c80a4 | Secondary adapter demux (page-1 1407/1603). Linkwidth recovery, CC re-arm. | SHARED | a522, d855, ca0d, e74e, bc88 |
| `CODE_BANK1::c0a5` | cm_routerop_mailbox | CM router-op mailbox. Gate EA90==0x5A; state `0x0B02`; movc dispatch E0..E8; reply via C805\|=0x02. Host-driven config-TLP/CfgRd/CfgWr over the tunnel. | SHARED | func_0def, d945, ceab, e21b, e4a6, e2b9 |
| `CODE_BANK1::e911` | tunnel_link_event_e763 | E763.2 → `[PcieTunnel-PcieLinkUp]` + d17e; E763.3 → `[PcieTunnel-PcieLinkDn]`. Print + W1C only. | GPU-PATH | ef03, d17e, sprint |
| `CODE:cd10` | bank0_cd10 | PCIe-downstream link bringup/reset (CC33.2). Ends in intentional infinite spin after `CC31.0` = controlled soft-reset (not a hang). | SHARED | e3b7, pcie_downstream_link_bringup, e8b5, e8a9, e26a |
| `CODE_BANK1::ef4e` | bank1_ef4e | C806.4 source — bare RET (no-op in stock). | HOUSEKEEPING | (none) |

### State machines

| Name | State var | States | Class |
|---|---|---|---|
| usb4_routerop_mbox_state | `0x0B02` | `0` IDLE (latch EA80, movc dispatch); `1` MULTIPKT_1 (E2 read cont); `2` MULTIPKT_2 (E3 write cont). | SHARED |
| PD role/contract + msg substate | `0x07BC` / `0x07BD` | `0x07BD`: `0x0D/0x0E` Data_Reset/Enter_USB pending; other → hard reset. `0x07BC`: role states driving CC99. Set by dispatcher `83d6`, **read** by ISR. | PD |
| SB active-port round-robin | `0x06F1` | `0..3` index selecting which SB channel's C9 connect event is serviced; advances `(+1)&3`. | GPU-PATH |

> **Why PD completes even when USB4 never starts:** `af5e` (PD-RX) and `b4ba`
> (timer-tick) run **regardless** of the `0x09F9&0x83` USB4 gate. The known wall:
> on handmade fw the host never RAISES C80A.5, so `a066` is starved.

---

## 3. Superloop

Stock superloop = `main_boot_and_superloop @ CODE:2f80`. Outer/inner do-while;
inner repeats until INTMEM `0x6A==0x0B` (USB bus-reset). EA is toggled around
each critical section.

### Per-pass dispatch order

| Trigger | Handler | Role | Class |
|---|---|---|---|
| top, EA-wrapped | USB-enum SM on a59/ae8 → `9037`/`c00d`/`ca0d` | NVMe CM bring-up + USB link-mode finalize | NVMe-ONLY |
| always | `c7a5` (gen/lanes printer → `sb_channel_connect_service`) | periodic SB connect service + banner | SHARED |
| `(09F9&0x83) && 0x06EC!=0` | `cb10` lane-bond tick, then `eea5` CDR re-arm | **USB4 lane-train/CL-walk/lane-bond FSM — the GPU tunnel engine** | GPU-PATH |
| `pd_tx_staged_pending != 0` | `e1c6` pd_tx_commit_engine | commit a staged PD TX (Source_Cap/Request/Accept/VDM) | PD |
| `541f` (E716&3 link up) | `dee3` | SB lane-advance drain / connect-drop (cb23/cadf) | GPU-PATH |
| `541f` | `480c` | NVMe MSC/DMA service | NVMe-ONLY |
| `541f` | `d3a2` | runtime tunnel PERST deassert + c2e6 | SHARED |
| `ae9 != 0x0f` | `e2ec` | USB4 post-connect reinit / housekeeping | SHARED |
| inner exit `0x6A==0x0B` | timer2-gated `9037`/`e677`; then `3419`(SS)/`180d`(USB2) | USB device re-enum + storage CBW | NVMe-ONLY |

### Functions

| Addr | Name | Role | Class |
|---|---|---|---|
| `CODE_BANK1::cb10` | u4lb_link_monitor_tick | **GPU-PATH CORE per-pass tick.** Guard `(09F9&0x83) && 0x06EC!=0`. Samples L0/L1 CL, abort timer vs `0x1388`, throttles on `ee57`, calls `u4lb_state_dispatch` (state-3/4/5 FSM), then `cdf5` if `0x072A!=0`. Drives SB[0xA0/A1]→`0x02`. | GPU-PATH |
| `CODE_BANK1::eea5` | bank1_eea5 (CDR re-arm 'RHMG') | PHY CDR/RX-PLL re-drive feeding lane training (when `phy_cdr_arm_mask!=0`). | GPU-PATH |
| `CODE:e1c6` | pd_tx_commit_engine | PD TX commit (Source_Cap/Request/Accept/VDM); msgid&7. | PD |
| `CODE:541f` | E716 link-up guard | `REG_LINK_STATUS_E716 & 3`; gates `dee3`/`480c`/`d3a2`. | SHARED |
| `CODE:dee3` | bank0_dee3 | `sb_cb10_lane_advance` drain: `==10` spin, `==0x0b` cb23/cadf + clear entered_usb_mode. | GPU-PATH |
| `CODE:480c` | FUN_CODE_480c | NVMe MSC/DMA service (488f/3e81, queue walk, 180d). | NVMe-ONLY |
| `CODE:d3a2` | bank0_d3a2 (tunnel PERST deassert) | Runtime PCIe-tunnel completion: deassert downstream PERST# (analog of handmade `pcie_power_on` deassert). **Gates (Phase-2 corrected):** internal entry `E716&3`; PERST-clear path `param4 && 0x06EB && PERST_CTRL.0`; inner c2e6 `0x06EB.1 && u4_connect_gate.3` (**NOT** `0x0AF1.3`); then `PERST_CTRL&=~1`. **Superloop call-site gate** `af38_t53!=0 && TIMER1_CSR.1` (writes TIMER1_CSR=2 first). KEEP for GPU path. | SHARED |
| `CODE:e2ec` | bank0_e2ec | USB4 post-connect reinit (e869/e95f). | SHARED |
| `CODE:9037` | cm_pcie_link_step_machine | Polled NVMe CM PCIe-link bring-up + per-port enroll walk (array @`0x00A8`). HW-proven harmful when raced with GPU power-on. | NVMe-ONLY |
| `CODE:c00d` | bank0_c00d (CM arm) | Arms the 9037 NVMe CM (PERST/C659/width re-drive + `0x05B4=0x10`). **REMOVED from handmade** (PERST collides with `pcie_power_on`, stalls GPU LTSSM at 0x01). | NVMe-ONLY |
| `CODE:e8e4` | bank0_e8e4 | CM re-arm wrapper → c00d. | NVMe-ONLY |
| `CODE:ca0d` | bank0_ca0d (mode-finalize) | USB link-mode finalize (latches `entered_usb_mode=0x10`). | USB-DEV |
| `CODE:c7a5` | gen/lanes printer + sb_channel_connect_service | Always-run: housekeeping + **SB connect channel poll** + `[PCIE Gen/lanes]` banner. | SHARED |
| `CODE:e677` | bank0_e677 (timer rearm) | Rearm TIMER2/4 (outer-loop housekeeping). | HOUSEKEEPING |
| `CODE:3419` | FUN_CODE_3419 (USB-SS enum/MSC) | SuperSpeed enum + SCSI/MSC CBW→DMA. Sets `0x06E6=1` (arms c00d). | NVMe-ONLY |
| `CODE:180d` | FUN_CODE_180d (USB2 enum + SCSI) | Legacy MSC SCSI/CBW + NVMe-param/DMA walk. | NVMe-ONLY |

### State machines

| Name | State var | States | Class |
|---|---|---|---|
| USB-enum / device-bringup (a59) | `0x0A59` | `0` fresh → `1` (route_mode==4: CM/NVMe bring-up via 9037, on done → `2` + `0x06E6=1` + c00d) / `2` link-mode finalize → ca0d. | NVMe-ONLY |
| USB4 lane-bond FSM (06ED) | `0x06ED` | Gate to enter cb10: `0x06EC!=0`. `0`=idle (monitor only); `!=0` → `u4lb_state_dispatch` state-3/4/5. State 5 done → `0`. | GPU-PATH |
| CM PCIe-link step machine | `0x0B39`/`0x06E5`/`0x05B4` | step++; `0x05B4==0x10` runs cc10/b403/e764 link core; per-port array @`0x00A8` states `0F/02/13/05/12/03`. | NVMe-ONLY |
| USB device MSC/CBW (0x6A) | IRAM `0x6A` | inner loop until `0x0B` (bus-reset/CBW-pending); 3419/180d set phase. | NVMe-ONLY |

> **GPU-PATH minimal loop set** = `cb10` (06ED FSM, gated 06EC) + `eea5` (CDR) +
> `dee3` (lane-advance) + `d3a2` (tunnel PERST deassert) +
> `c7a5`/`sb_channel_connect_service` + the INT1 SB/router/tunnel handlers
> (a066/c0a5/e911). Everything on the a59 branch (9037/c00d/e8e4) + 480c/3419/180d
> is NVMe-ONLY and is exactly what handmade dropped.

---

## 4. USB4 State Machines (lane-bond / PHY / SB-transport)

**All functions here are GPU-PATH or SHARED — no NVMe-only functions.** Two
driver chains feed the FSMs and share state via XDATA:

- **SB EVENT path:** INT1 `4486`@`44d3` → `061a` → `a066` → `d4cd` → `cd3f` →
  {`eaac`/`af38`/`ebb5`}.
- **FSM-TICK path:** superloop `2f80` → `061f` → `cb10` → `e672` →
  {`a7de`/`b0b4`/`8000`|`850b`} + `cdf5`.

### Functions

| Addr | Name | Role | Class | Calls |
|---|---|---|---|---|
| `CODE_BANK1::cb10` | u4lb_link_monitor_tick | Periodic FSM tick (see §3). | GPU-PATH | 9716, sb_read_a1, e672, cdf5, da9f |
| `CODE_BANK1::e672` | u4lb_state_dispatch | **THE lane-bond FSM dispatcher.** Reads `0x06ED`: `3`→a7de, `4`→b0b4, `5`→(0x0718==4 → 8000 else 850b). | GPU-PATH | a7de, b0b4, 8000, 850b |
| `CODE_BANK1::a7de` | u4lb_cm_conn_routing_setup | **STATE 3 `[ConnRout]`.** Inner `0x0758` substate. On confirm gates `0x0777==0x0C`, writes `0x0718=0x04` ROUTE-ENABLE, updates `0x0819/081A` lane-advertise from `0x077A`, seeds width latches via e391. | GPU-PATH | 9814, **e391**, 99f3, 9a31, eda0, edf5, eb62, r3_write_dispatch, sprint |
| `CODE_BANK1::e391` | cm_init_routing_tables (width-LUT seeder) | **GAP-1 ROOT (Phase-2 added row).** Called from a7de gated `0x0776==0` (gate is at the a7de/a869 CALL-SITE, not inside e391). loop1 (i=0..0x12, 19 ent): ROM `CODE:0x514c[i]`→XDATA[`0x06F2`+i] width-LUT **+** ROM `CODE:0x515f[i]`→XDATA[`0x0705`+i] branch-A gate; loop2 (i=0..7) zeroes XDATA[`0x0B26`+i] (deep-PHY, omit-able). af38 SBTX[1] = LUT[`0x06F2`+idx]; **unseeded → TX[1]=uninit 0x55** (stock 0x03). | GPU-PATH | (ROM copy + d221) |
| `CODE_BANK1::b0b4` | u4lb_state4_b0b4 | **STATE 4 (PCIe-tunnel PwrOn / lane-training).** Per enabled lane (`0x0819.0/.1`) SB opcode `0x81/0x85` (lane0) / `0xA1/0xA5` (lane1); counter≥0x38 + link present → reset RX-PLL, `[L0/L1 OS1]`, latch 075B/0759/075C/075A, snapshot to 074E/074F, → state 5. | GPU-PATH | e07d, b226, d5da, e305, rst_rx_pll, e980 |
| `CODE_BANK1::8000` | u4lb_lane_gate | **STATE 5 walker A (`0x0718==4`, live AMD path).** Two ladders over 2 ports: LOOP1 @0x0759/075A, LOOP2 @0x075B/075C. CL-state walk `0x10→0x20→0x30→0x50→0x60→bond`; CL-walk pushes (`e461` at 0x50). | GPU-PATH | read_page08_byte, c51_switch_dispatch(0def), e461, 84fa |
| `CODE_BANK1::850b` | u4lb_walk_850b | STATE 5 walker B (`0x0718!=4`). Dead on live AMD path. NOTE: its 0x30 reads `lb_cl_status@0x0B26` not `0x0779`. | GPU-PATH | d20d/d278, c51_switch_dispatch, 8992 |
| `CODE_BANK1::d4cd` | sb_transport_substate_poll | SB-transport poller. Polls SB[0x28/0x2A/0x81/0x83] bit.3 edges; sets `sb_active_plane_port`=0/1 (plane 0x2900 vs 0x2A00); drives `0x06EE/06EF/06F0`. Edge → `cd3f` + W1C ack / `d54c`. | GPU-PATH | cd3f, 9746/974a, d54c |
| `CODE_BANK1::cd3f` | sb_cd3f_dispatch | SB descriptor-event ROUTER. cmd→0x4e, status→0x752, payload→0x755. Dispatch: `(status&0x60)==0x60`→ebb5; `status.0==0`→eaac; `status.0==1 & (bit6\|nibble)`→af38. | GPU-PATH | eaac, af38, ebb5, edd9 |
| `CODE_BANK1::af38` | sb_af38_descriptor_response | **SB-transport descriptor RESPONSE engine.** Builds TX in plane 0x2900/0x2A00: `header[0]=cmd&0xDE`, `[1]=976e width-LUT(0x06F2+idx)`. **Emits the `0C03`/`0104-6324` route-descriptor TX the host needs.** | GPU-PATH | r3 dispatch, 976e, cm_command_dispatch, d5da |
| `CODE_BANK1::eaac` | sb_eaac_populate_0777 | Copy 0x40 bytes of host connect descriptor → `0x0777..0x07B6` (route gate `0x0777==0x0C`). | GPU-PATH | read_0x755, r3_read_dispatch |
| `CODE_BANK1::cdf5` | sb_cdf5_routerop_response | Router-op response builder (cb10 tail, `0x072A!=0`): assemble 4-byte hdr + 0x40-byte width payload, write back. | GPU-PATH | eda0, r3_write_dispatch |
| `CODE_BANK1::ebb5` | sb_set_connect_present_ebb5 | cd3f branch `(status&0x60)==0x60`: set connect-present `0x0765`. | GPU-PATH | sb helpers |

### State machines

| Name | State var | States / transitions | Class |
|---|---|---|---|
| U4LB top FSM | `0x06ED` | `0`=IDLE; `3`=CONN_ROUT→a7de; `4`=LANE_TRAIN→b0b4; `5`=LANE_BOND→8000\|850b. `3→4→5→0`. (Values 1/2 don't exist.) Set via `eb62` (prints `[SB P0<state>]`). | GPU-PATH |
| CONN-ROUT inner substate | `0x0758` | `0x10` ARM_ROUTE_QUERY (edf5); `0x11` AWAIT_RESULT (gate `0x0777==0x0C`, set `0x0718=4`); `0x00` PRINT_STATUS (terminal → state 4). | GPU-PATH |
| State-5 LOOP1 lane FSM | `0x0759`(L0)/`0x075A`(L1) | `00`PARKED→`10`WIDTH_INIT→`20`ARM_WAIT_PUSH→`30`LANE_PRESENT_SEL→`40`SETTLE_CLEAR→`50`WIDTH_SETTLE_WALK→`60`BOND_WAIT_PUSH→`70`WIDTH_LATCH_SEL→`80`/`90` FINALIZE→`A0`BOND_WAIT_ACK→`A1`BONDED_MONITOR. | GPU-PATH |
| State-5 LOOP2 CL-state FSM | `0x075B`(L0)/`0x075C`(L1) | `00`CL_IDLE→`10`CL_INIT→`20`CL_PUSH_WAIT→`30`CL_EVAL (reads `0x0779`)→`50`CL_BOND_WAIT (e461 push)→`60`CL_BOND_MON. **Drives SB[0xA0/A1] 0x07→0x02 + host CL-grant.** | GPU-PATH |
| State-5 routing FSM (850b) | `0x0718` + reused `0x075B` | dead on live AMD path; `0x30` reads `lb_cl_status@0x0B26` (≠ LOOP2's 0x30). | GPU-PATH |
| SB-transport edge/plane | `0x06F0` (+ 0x06EE/06EF) | `0`→plane 0x2900 / read 0x2280D; `1`→plane 0x2A00 / read 0x2280E. d4cd flips on SB[0x28/0x2A] bit.3. Round-robin connect channel = `0x06F1`. | GPU-PATH |

> **KNOWN-BUG context (from memory):** live divergence is `af38` SBTX[1] reading
> uninit `0x55` from `0x06FE` because the `e391` width-LUT seed (gated `0x0776==0`)
> never fires on the live Connect_U4 path → TX=`0C55` instead of stock `0C03`.
> Also `0x0819` lane-advertise depends on `cap20g_gate1(0x09F6)` (a7de sets
> `0x0819=0x03` both lanes vs `0x01`).

---

## 5. CM-Tunnel / Router-ops — **GPU-CRITICAL**

The host CM drives the GPU tunnel through the **router-op mailbox** (EC06.0 →
`c0a5`). The most important op is **`0xE8` (PcieTunnel-Deassert/Enable)** which
HW-resets then deasserts B480 PERST bits0-3 — bringing the downstream PCIe link
(the GPU) out of reset.

### Dispatch chain (confirmed from `c0a5` jump-table @`CODE_BANK1::c0c5`)

`EC06.0 → int1(4486) ack EC04=1 → tramp 0499 → c0a5`. Gate `EA90==0x5A`, latch
`EA80→0x0B03`, movc table `{E0:c0de, E1:c0e8, E2:c0ef, E3:c119, E4:c151, E5:c158,
E8:c15f, default:c1ae}`. `EA81` = read/write sub-opcode (`0x50`/`0x51`). Reply =
`cf35` (`C805=(C805&0xF9)|0x02`) + `C8B0<-0xEA` DMA push; `EA90<-0xA5` on done.

### Functions

| Addr | Name | Role | Class | Calls |
|---|---|---|---|---|
| `CODE_BANK1::c0a5` | cm_routerop_mailbox | Router-op dispatcher / state machine. | GPU-PATH | 0def, d945, cf5d, ceab |
| `CODE:0def` | c51_switch_dispatch | Keil C51 movc switch helper (table after LCALL @c0c5). | SHARED | (indirect JMP) |
| `CODE_BANK1::c0ef` | routerop_op_E2_cfgread | **E2 = router CONFIG-SPACE READ.** cf4c gate, ceef addr, d945 send-read-resp, ceab bounds; set `0x0B02=1`. | GPU-PATH | cf4c, ceef, d945, ceab |
| `CODE_BANK1::c119` | routerop_op_E3_cfgwrite | **E3 = router CONFIG-SPACE/PATH WRITE.** stage 0x0B08/09, cf5d send-write-resp; set `0x0B02=2`. | GPU-PATH | cf4c, ceef, cf5d, ceab |
| `CODE_BANK1::c15f` | routerop_op_E8_tunnelreset | **E8 = PCIe-TUNNEL RESET/ENABLE.** LCALL `e4a6` — the host's `[PcieTunnel-Deassert]/[Enable]`. THE trigger that brings the GPU out of reset. | GPU-PATH | e4a6 |
| `CODE_BANK1::e4a6` | tunnel_routerop_link_reset | PCIe-tunnel link RESET+re-enable. `C656&=~0x20`, `CA06&=~1`, `CC31.0=1` HW RESET ASSERT + spin; post-reset → e26a/d185 + `eec7` PERST-deassert; `B480&=~0x0F` deassert; `[PcieTunnel-Enable]`. | GPU-PATH | eec7, e26a, d185, e9b5 |
| `CODE_BANK1::eec7` | bank1_eec7 | PERST-DEASSERT helper: e8d9 (`C659.0`) + `[PcieTunnel-Deassert]`. | GPU-PATH | e8d9, ef03, sprint |
| `CODE_BANK1::e4ea` | tunnel_routerop_enable_tail | Post-reset Enable tail: power-on + `B480` PERST bits0-3 deassert + `[PcieTunnel-Enable]`. | GPU-PATH | e26a, d185, eec7, e9b5 |
| `CODE_BANK1::d945` | cm_routerop_send_read_resp | Build+arm router config-space READ response (bounds, hdr status/code=3, DMA stream `C8B0=0xEA`). | GPU-PATH | ced6, cf11, cf35, be02 |
| `CODE_BANK1::cf5d` | cm_routerop_send_write_resp | Build+arm router config-space WRITE response (DMA stream write data, advance 0x0B08). | GPU-PATH | ced6, cf11, cf35, ceab, dbf5 |
| `CODE_BANK1::ceab` | cm_routerop_addr_in_bounds | Compare 64-bit addr `0x0B04` vs limit `0x0B0A` → continuation vs completion. | GPU-PATH | cmp_neq_u32 |
| `CODE_BANK1::ceef` | cm_routerop_copy_addr_from_mbox | Copy 64-bit addr/payload from EAxx mailbox → `0x0B04`/`0x0B0A`. | GPU-PATH | (mailbox reads) |
| `CODE_BANK1::cf4c` | cm_routerop_is_read_opcode | Test EA81 sub-opcode read(0x50)/write(0x51). | GPU-PATH | (EA81) |
| `CODE_BANK1::cf35` | cm_routerop_reply_trigger | `C805 = (C805&0xF9)\|0x02` (send-response bit). | GPU-PATH | (C805 RMW) |
| `CODE_BANK1::e21b` | routerop_op_E5_cfgop | E5 single-reg config read/write (EA82 0x50/0x51). | GPU-PATH | cf53, r3_write_dispatch |
| `CODE_BANK1::d6dc` | routerop_op_E4_blockcfg | E4 router config BLOCK read/write loop (EA81=len). | GPU-PATH | cf53, add_u32, r3_read_dispatch |
| `CODE_BANK1::e2b9` | sb_issue_transport_cmd | routerop connect-fill helper (stage 0xAA8/9/A, poll d4cd, stream via 96f7→d5da). | SHARED | d4cd, 9923, 96f7, d5da |
| `CODE:adb0` | cm_adb0_tlp | **GENERAL config-TLP issuer** (CfgRd/CfgWr FMT 04/05/44/45 by mailbox IRAM 0x60-65). Result B220-B223. On link-down → `0x06EA=0xFE, 0x06E6=1, c00d`. | SHARED | 9a53, 999d, 99eb, 9a95, c00d |
| `CODE:c1f9` | cm_c1f9 | Predecessor cfg/ECAM-MemRd engine (dir 0x05AE, addr 0x05AF-0x05B2, 0x00D000 aperture). | SHARED | 9a53/999d/99eb/9a95, e762 |
| `CODE:c5ff` | route de-alias / bridge bus-num | **BOUNDARY RESOLVED (see §NVMe note).** 4× CfgWr-over-tunnel via adb0: reg6 bus-num (from `0x0A5F`=0x02000000 window, d956), reg1 cmd, reg8 mem-win `0x00D00000` aperture (9a7f), 4th cap-derived `0x40010000` (keyed off 9ee5 cap-walk); then per-port `0x05B4=2` done-marker. **Touches NO PERST/PHY.** Only caller=9037; reaching it needs c00d (`0x05B4==0x10`)=the PERST race. → **LIFT the de-alias, drop 9037.** | SHARED (lift) | e77a, dde2, **9ee5**, **d956**, **e91d**, 9a7f, 9930, 994c/e, 0dc5 |
| `CODE:a183` | NVMe class-01:08:02 enroll | Per-port device-enroll: CfgRd class, record NVMe storage devices into per-port table. | NVMe-ONLY | d02a, dde2, db45 |
| `CODE:c00d` | cm_arm_c00d | CM downstream-link ARM (gated `0x06E6`): B401 pulse, cd6c, B480 PERST assert, 39e4 init, `0x05B4=0x10`. | SHARED | cd6c, 39e4, e8a9, d436 |
| `CODE:39e4` | cm_pcie_link_init_state | DMA/descriptor-RAM + page-04 + router-op mailbox reset. | SHARED | (DMA regs, array clears) |
| `CODE:cd6c` | pcie_tunnel_adapter_enable | TUNNEL ADAPTER enable: `CA06&=~0x10`, config B410-B42B, `B401.0`. **Callers (Phase-2): c00d + boot-stub 0462 → single-tag SHARED** (canonical; §1 `..._b401` is the same fn). Handmade LIFTS it to a direct boot call (main.c:522). | SHARED | c8db, CA06, B401 |
| `CODE:e2a6` | cm_link_up_check | `(0x07EF==0) && (B432&7)==7 && (E765&2)` = downstream link-UP PASS gate. | SHARED | (B432, E765) |
| `CODE_BANK1::e911` | tunnel_link_event_e763 | INT1 tunnel link-event (E763.2/.3 W1C + print). | GPU-PATH | ef03, sprint, d17e |

### State machines

| Name | State var | States | Class |
|---|---|---|---|
| router-op mailbox | `0x0B02` | `0`=IDLE (latch EA80→0x0B03, c0c5 dispatch); `1`=MULTIPKT_1 (E2 read cont); `2`=MULTIPKT_2 (E3 write cont). Gate `EA90==0x5A`→`0xA5`. | GPU-PATH |
| router-op working buffer | `0x0B04`(addr)/`0x0B0A`(limit) | addr from EA82+ (ceef); len=min(limit-addr,7); 0x0B08/09=write cursor. | GPU-PATH |
| CM downstream-link step (9037) | `0x05B4`+`0x0B39`+`0x0B3A/B` | `0x05B4==0x10` runs LTSSM core; e2a6 link-up poll; per-port walk c5ff/a183. | SHARED |
| CM arm one-shot (c00d) | `0x06E6` | `!=0` → consume, B401/cd6c/B480 PERST/39e4/`0x05B4=0x10`. Re-armed by adb0/e6fc on TLP error. | SHARED |

> **The open GPU wall (per memory `project_cm_walk_host_enable_open`):** the host
> only posts `0xE8` `[PcieTunnel-Deassert]` **after** it discovers+enables a
> PCIe-DOWN adapter. That needs the device to advertise/enable the tunnel adapter
> (`cd6c` → `c8db` B410-B42B + `B430|=1` + `B401.0`) **and** the host to write the
> adapter-CS page-1 enable regs (`0x1208/0x1210/0x1334`). Currently handmade
> enumerates the router but never enables the PCIe-down adapter → no `0xE8` →
> `e4a6` never runs → GPU stays in PERST. **The adapter advertise/enable is the
> immediate predecessor to chase.**

---

## 6. PD / VDM (USB-PD / Structured-VDM / Enter_USB — precedes USB4)

Entire subsystem is **PD**. It is on the GPU path *indirectly*: the device must
PROMPT the host (Hard Reset), reply to Source_Cap with a Request RDO, ACK
Discover_Identity (VID `0x174C`) / SVIDs / Modes (TBT SVID `0x8087`), then ACK
Enter_Mode/Enter_USB4 → which **hands off to the USB4 SB/lane subsystem**.

### Functions

| Addr | Name | Role | Class |
|---|---|---|---|
| `CODE:83d6` | pd_dispatch_data | **PD RX MESSAGE DISPATCHER** (live ISR path). Switch on `0xAA1` #data-obj: `1`=Source_Cap, `3`=BIST, `8`=Enter_USB (only if type==5), `0xF`=VDM, else NAK. | PD |
| `CODE:8406` | pd_dispatch_control | Near-identical sibling (control/alternate ctx); shares Source_Cap/Enter_USB/VDM. | PD |
| `CODE:abf5` | pd_select_pdo_from_source_cap | Source_Caps handler: select operating PDO → op-current 0x7DA/0x7DB. `[Source_Cap]`. | PD |
| `CODE:acd4` | pd_build_send_request_rdo | Build Request(RDO) → E420-E425, set substate `0x7BD=3`, strobe CC TX (CC10/12/13/11). | PD |
| `CODE:a036` | pd_handle_enter_usb | Enter_USB (PD3.1) handler. Parse EUDO (mode @0xAA6). mode==2 & role==0 → Accept, set accepted=1, `connect_gate_e8=1`, `[Enter_USB 4]` → `[Connect_U4]` `sb_lane_flip_init`. | PD |
| `CODE:9ac4` | vdm_tx_strobe_commit | VDM dispatcher. cmd 1=Discover_ID, 2=SVIDs, 3=Modes, 4=Enter_Mode, 5=Enter_USB4-ACK, 6=Exit. **TX via CC88 (NOT CC10).** cmd 4/5 accept → `[Connect TBT]` sb_lane_flip_init. | PD |
| `CODE:aa36` | vdm_build_discover_id | DISCOVER_IDENTITY ACK: **VID 0x174C** (LBA0=0x4C, LBA1=0x17), TBT bits when SOP'. | PD |
| `CODE:ddad` | vdm_build_discover_sids | DISCOVER_SVIDs ACK: **TBT SVID 0x8087**. | PD |
| `CODE:d852` | vdm_build_discover_modes | DISCOVER_MODES ACK: TBT3 mode VDO for SVID 0x8087. | PD |
| `CODE:b966` | vdm_handle_enter_mode | ENTER_MODE ACK (SVID 0x8087, objpos1, role0): latch `connect_route_latch=1`, `connect_pending=1`, `[Enter_Mode_Suc]`. | PD |
| `CODE:ca71` | vdm_enter_usb4_ack_sb_init | **ENTER_USB4 (cmd5) ACK + SB hand-off** — calls `bank1_lane_flip_init_stub` + `sb_block_init_stub`. THE PD→USB4 bridge. | PD |
| `CODE:dd0e` | vdm_nak | VDM NAK builder (echo SVID, cmdtype 2). | PD |
| `CODE:be8b` | pd_drive_hard_reset (Drive_HardRst KEYSTONE) | TX HARD-RESET to force host to re-send Source_Cap. **NO-OP once E302 link-mode==3** (`[CCOpen_neednt_HardRst]`). The device's host-prompt mechanism. | PD |
| `CODE:b4ba` | cc_pd_timer_tick | INT1 policy timer-tick (CC23/81/91/99/D9/F9). CC91.1=1s timeout → `route_mode=4` + USB4 commit. | PD |
| `CODE:ae87` | cc_pd_phy_term_init | PD PHY/CC term (Rp/Rd), RDO/CRC timing, arm PD engine. | PD |
| `CODE:b8c3` | pd_internal_state_init | Reset PD policy-engine block: `0x7BD=1`, clear latches. `[InternalPD_StateInit]`. | PD |
| `CODE:0102` | usb4_mode_entry_commit | Device-side USB4/TB mode-entry latch: write `92E1=0x10`, mask `9090.7`; return 4(USB4)/1(USB3). The latch the host waits on. | PD |

### State machines

| Name | State var | States / transitions | Class |
|---|---|---|---|
| PD policy-engine substate | XDATA `0x07BD` | `0`=idle; `1`=INIT (b8c3, ready for Source_Cap); `2`=Source_Cap+PDO selected; `3`=Request sent, awaiting Accept/PS_RDY; `5/6`=valid-contract; `0x0D`=Enter_USB pending, `0x0E`=Data_Reset pending. `1 →Source_Cap→ 2 →send Request→ 3`. Bad-state → soft-reset → 1. Hard-reset → state-init → 1. | PD |
| USB4 connect-decision latches | accepted / connect_pending / connect_route_latch / route_mode / mode_flag(0x09F9) | Enter_USB Accept → accepted=1, route_mode=4, gate_e8=1. Enter_Mode ACK → route_latch=1; Enter_USB4-ACK consumes it → `bank1_lane_flip_init_stub`+`sb_block_init_stub`. accepted!=0 → `[Connect_U4]/[Connect TBT]` → sb_lane_flip_init. | PD |

### Dispatch table (PD RX)

| Trigger | Handler | Role |
|---|---|---|
| E40F.0 msg received | pd_dispatch_data@83d6 (switch on 0xAA1) | Main PD RX demux |
| 0xAA1==1 Source_Cap | abf5 + acd4 | Reply Request(RDO); substate 1→2→3 |
| 0xAA1==8 Enter_USB & type==5 | a036 | Enter_USB Accept/Reject + USB4 latch |
| 0xAA1==0xF VDM | 9ac4 (switch on VDM cmd) | Structured-VDM responder |
| VDM 1/2/3/4/5 | aa36/ddad/d852/b966/ca71 | Identity/SVIDs/Modes/Enter_Mode/Enter_USB4 |
| CC91.1 1s timeout | usb4_mode_entry_commit | Timeout fallback → commit USB4 (route_mode=4) |

> **Gotchas:** (a) VDM TX uses **CC88** not CC10. (b) EUDO mode = VDO0 byte3
> bits6:4 → `0x0AA6` (0=USB2,1=USB3.2,2=USB4). (c) VID=`0x174C`, TBT SVID=`0x8087`.
> (d) Drive_HardRst is a NO-OP once E302 link-mode==3 — it must run when !=3.
> (e) `bank0_8a89` (suspend/resume PHY loop) is a SECONDARY caller of `pd_rx_isr`
> (tagged SHARED?).

---

## 7. Local PCIe (device→GPU downstream LTSSM)

The LOCAL device-side downstream PCIe regs that bring up the GPU link over the
tunnel.

| Addr | Name | Role | Class | Calls |
|---|---|---|---|---|
| `CODE:3578` | pcie_downstream_link_bringup | LOCAL device→GPU LTSSM bring-up: B455 speed, B2D5/B296 enable, B220 config-TLP caps, **deassert PERST (B480&=~1)**, poll B455.1 train-done, d436 x4 width; returns `0x0F` linked. | GPU-PATH | 05a8, cc10, 519e, e8a9, d436 |
| `CODE:8000` | pcie_tunnel_link_setup | BANK0 tunnel data-path setup: B220 config-TLP, B230/B234/B240/B244/B246 link params, read neg'd width from 0x8000-0x800B SB scratch, `0x05B4=2`. **DISTINCT from `CODE_BANK1::8000` u4lb_lane_gate (§4) — different fn, never cross-wire; always bank-prefix.** | GPU-PATH | 9ee5, 994c, e91d, b220, d02a, e762 |
| `CODE:92bb` | pcie_tunnel_link_bringup_start | `0x06E6=1` + `bank0_c00d` (tunnel power-on re-entry). | GPU-PATH | c00d |
| `CODE:d996` | boot_phy_d996_pcie_tunnel_boot | BOOT pre-stage downstream tunnel datapath: B402.1 clr, e8a9, e764 reset pulse, B432/B404 lane power, B434/B436 width, e25e finalize. | SHARED | b402, e8a9, e764, d630, d436, e25e |
| `CODE:e2a6` | pcie_link_up_check_b432_e765 | **`(B432&7)==7 && (E765&2)` = the `0x78` CONNECTED criterion.** | GPU-PATH | (B432, E765) |
| `CODE:99e0` | pcie_write_xdata_assert_perst | `B480 = (B480&0xFE)\|1` (downstream PERST assert). | GPU-PATH | (B480) |
| `CODE:b402` | sb_pcie_width_ramp | PCIe lane-width/speed ramp (0x0AD5-0x0AE1, a666 query, a647/a704 apply). | SHARED | a666, a704, a647 |
| `CODE:39e4` | pcie_link_init_state | DMA/descriptor-RAM + page-04 + mailbox reset on (re)init. | SHARED | C8D7/C8D8, 175d |
| `CODE:c5ff` | config-TLP de-alias/probe | See §5/§NVMe — de-alias = 4× CfgWr to the type-1 bridge (bus-num/cmd/mem-win + cap-derived) over the adb0 tunnel mailbox; `0x05B4=2` marker. **LIFT to GPU-path adb0 caller; do NOT reach via 9037/c00d (PERST race).** | SHARED (lift) | e77a, dde2, 9ee5, d956, e91d |

**LOCAL device→GPU PERST/PHY regs:** `B480`(PERST), `B455`(LTSSM speed),
`B432/B404`(lane power), `B434/B436`(width), `C656/CA06/CC31`(router-op link
reset @e4a6), `E764/E765`(PHY train/link-up).

**GPU/PCIe-tunnel entry chain:** `boot_phy_bringup_early(ce79)` → `d996` pre-stage
→ [USB4 connect] → `pcie_tunnel_link_bringup_start(92bb)` → `bank0_c00d` power-on
→ `pcie_tunnel_adapter_enable_b401(cd6c)` + `pcie_link_init_state(39e4)` → arm
`0x05B4=0x10` → `9037` step machine → `pcie_downstream_link_bringup(3578)`
LTSSM/PERST → `pcie_link_up_check_b432_e765` → `tunnel_link_event_e763`
(PcieLinkUp).

---

## 8. USB Device

| Addr | Name | Role | Class |
|---|---|---|---|
| `CODE:0e5b` | int0_isr body | USB-device/peripheral ISR (EP/PHY/buffer + NVMe-queue tail). | USB-DEV |
| `CODE:9c2b` | usb_ss_link_train_engine | USB SS link-train (from int0 via 91D1.3): cc10 train, on SS-FAIL force USB2; not reached in USB4 mode. | USB-DEV |
| `CODE:4532` | usb_ss_link_event_policy_0003 | Deferred SS link-event policy on latched `0x0003` bitmap: b3→pcie_downstream_link_bringup, b4→USB4 lane train, b6→Type-C err recovery, b2→SS re-init + re-run mode decision. | USB-DEV |
| `CODE:4904` | usb_msc_init | USB MSC engine init (900B/C42A, 'USBS' CSW, NVMe doorbell). | USB-DEV / NVMe |
| `CODE:ca0d` | bank0_ca0d | USB link-mode finalize (latch entered_usb_mode=0x10). | USB-DEV |

In USB4 mode (`0x0A59==1`) the SS device is NOT armed → the USB2/USB3
device-enum + MSC paths are largely dormant.

---

## NVMe — OMITTED FOR NOW (documented for reference)

> **WARNING — re-verify SHARED? items before omitting.** The config-TLP machinery
> (`adb0`/`c1f9`/`c5ff`/`9ee5`/`d956`/`b220`/`e91d`) and the `9037` step machine
> are **SHARED** infrastructure used by both NVMe enroll *and* the GPU tunnel.
> Only the class-enroll bodies and storage command/queue/MSC functions are
> NVMe-ONLY.
>
> **✅ c5ff BOUNDARY — RESOLVED (Phase 2, Ghidra-verified). The self-contradiction
> ("c5ff on the GPU critical path" vs "drop the whole 9037 branch") is settled:**
> c5ff's de-alias is just **4× CfgWr-over-tunnel** (reg6 bus-num from the
> `0x0A5F`=0x02000000 window via `d956`; reg1 cmd; reg8 mem-window `0x00D00000`
> aperture via `9a7f`; a 4th cap-derived `0x40010000` keyed off the `9ee5` cap-walk)
> **+** the per-port `0x05B4=2` done-marker — all issued through the same
> **adb0 (B210-B296) mailbox** the handmade `cm_adb0_tlp` already implements.
> **c5ff touches NO PERST/C659/B480/PHY.** BUT its ONLY caller is `9037`, and reaching
> the call-site (`CODE:90ce`) requires `c00d` to set `0x05B4==0x10` (gate @9071/9075) —
> i.e. the full NVMe-arm + 9037 link-core, whose **B480-PERST / C659 / d436-width /
> e764-RXPLL / B403 re-drive is the HW-proven race** that strands the GPU LTSSM at 0x01
> (dead+removed in handmade). **RESOLUTION → DROP 9037 (NVMe); LIFT c5ff's CfgWr de-alias
> into a GPU-path caller** driven by the handmade adb0 engine, fired ONCE post-bond +
> post-tunnel-link-up (after `pcie_power_on`, main.c:700-705) — PERST-free, no race.
> Route-bind PRIMITIVES `e91d`/`d956`/`9ee5` are **[SHARED]** (a183 NVMe-enroll calls them
> too); a183's outer 0x20-device class-scan/enroll is **NVMe-ONLY**. **CAVEAT:** the de-alias
> is **necessary-but-not-sufficient** — it only makes the DEVICE's own tunneled config
> coherent. The actual HOST-visibility gate (per memory `project_cm_walk_host_enable_open`)
> remains the **stubbed C0A5 router-op mailbox** (`usb4.h:63` — host posts `EC06=0` → never
> `0xE8` PcieTunnel-Deassert). **Implement BOTH:** the lifted c5ff de-alias AND the c0a5
> E2/E3/E8 mailbox.
>
> Also re-verify **`d3a2`** (runtime tunnel PERST deassert — needed for ANY downstream PCIe
> link incl. the GPU): real gate is `0x06EB.1 && u4_connect_gate.3` (**NOT** `0x0AF1.3`),
> superloop call-site `af38_t53 && TIMER1_CSR.1`; **KEEP** for the GPU path.

| Addr | Name | Why omittable |
|---|---|---|
| `CODE:4904` | usb_msc_init | USB MSC BOT + NVMe-doorbell ('USBS' CSW @0xD800, MSC EPs). Pure storage data-path. Last call in `bf8e` — its absence must not disturb the preceding `c00d`/tunnel init. |
| `CODE:1196` | nvme_queue_service / NVMe CQ worker | NVMe IOSQ/IOCQ drain + `C47A=0xFF` ack from int0 NVME_QUEUE_BUSY loop. |
| `CODE:488f` | nvme_completion_handler | NVMe completion-queue handler (link-status bit1). |
| `CODE:49e9` | nvme_link_event_handler | NVMe link/admin event handler. |
| `CODE:3e81` | nvme_handler_3e81 | NVMe handler when USB host connected. |
| `CODE:4784` | nvme/MSC service 4784 | USB-MSC / NVMe bridge (BOT/UAS→NVMe). |
| `CODE:9037` | cm_pcie_link_step_machine | NVMe CM PCIe-link + per-port enroll walk. HW-proven harmful when raced with GPU power-on. Handmade omits it. |
| `CODE:c00d` | bank0_c00d / cm_arm_c00d | Arms 9037 NVMe CM (PERST/C659/width re-drive). **REMOVED from handmade** (PERST collides with `pcie_power_on`, stalls GPU LTSSM at 0x01). |
| `CODE:e8e4` | bank0_e8e4 | CM re-arm wrapper → c00d (only via a59 NVMe branch). |
| `CODE:480c` | FUN_CODE_480c | NVMe-link (488f/3e81) + storage command-queue/DMA dispatcher (calls 180d). |
| `CODE:3419` | FUN_CODE_3419 | USB SS enum + SCSI/MSC CBW→DMA. Sets `0x06E6` to arm NVMe CM. Handmade replaces with vendor bulk EP. |
| `CODE:180d` | FUN_CODE_180d | USB2 MSC SCSI + NVMe-param/DMA walk. |
| `CODE:a183` | link_init_iter_scratch_a183 | Per-port class-01:08:02 enroll. **NOTE: shares route-bind primitives (d956/e91d) with c5ff — only the class-match enroll body is NVMe-specific.** |
| `CODE_BANK1::89db` | bank1_89db | NVMe storage command builder: IOSQ entry (0x0540-0x054E) + LBA/COUNT/DMA regs C422/C423/C424/C404. |
| `CODE:0xa000` | NVME_IOSQ_BASE | NVMe IOSQ 4KB window (ASQ 0xB000/ACQ 0xB100, data 0xF000). Document layout, omit. |

---

## GPU-TUNNEL CRITICAL PATH

The ordered minimal chain (init + handlers + state machines) needed to get from
PD contract to GPU-in-`lspci`:

1. **PD contract (PD subsystem).** Device prompts host: `pd_drive_hard_reset(be8b)`
   (NO-OP if E302==3) → host sends Source_Cap → `pd_rx_isr(af5e)` →
   `pd_dispatch_data(83d6)` → `pd_select_pdo_from_source_cap(abf5)` +
   `pd_build_send_request_rdo(acd4)` (substate `0x7BD` 1→2→3).
2. **VDM + Enter_USB4 (PD subsystem).** ACK Discover_Identity (VID `0x174C`,
   `aa36`) / SVIDs (`ddad`) / Modes (`d852`) / Enter_Mode (`b966`) →
   `vdm_enter_usb4_ack_sb_init(ca71)` / `pd_handle_enter_usb(a036)` → set
   `accepted`/`route_latch` → `[Connect_U4]` → `sb_lane_flip_init` +
   `usb4_mode_entry_commit(0102)` (`92E1=0x10`). USB4 gate `0x09F9&0x83` set.
3. **USB4 lane bond (USB4 SM subsystem).** SB transport powered by
   `boot_phy_bringup_early(ce79)`. INT1 `C80A.5 → a066` (`d4cd`→`cd3f`→
   `eaac`/`af38`) populates `0x0777==0x0C` + emits route-descriptor TX
   (`0C03`/`0104-6324`). Superloop `cb10 → e672`: state-3 `a7de` (ROUTE-ENABLE
   `0x0718=4`, `0x0819` lane-advertise) → state-4 `b0b4` (tunnel pwr-on, lane
   train, `[L0/L1 OS1]`) → state-5 `8000` CL-walk → **SB[0xA0/A1] 0x07→0x02 =
   BOND** + host CL-grant.
4. **SB/router transport up.** `tunnel_link_event_e763(e911)` posts
   `[PcieTunnel-PcieLinkUp]`; router enumerates as `1-0:1.1`.
5. **Host posts router-ops (EC06).** INT1 `EC06.0 → c0a5` mailbox: host CM
   `0xE2`/`0xE3` config read/write (`c0ef`/`c119` → `d945`/`cf5d`) — device router
   config space is enumerated.
6. **PCIe-Down adapter advertised/enabled.** Device advertises/enables the tunnel
   adapter: `cd6c` → `c8db` (B410-B42B + `B430|=1` + `B401.0`). **Host must then
   write adapter-CS page-1 enable regs (`0x1208/0x1210/0x1334`).** *(This is the
   current open wall — see §5.)*
7. **Tunnel deassert (host posts `0xE8`).** `c0a5 → c15f → e4a6` (`CC31.0` HW
   reset → `eec7` `[PcieTunnel-Deassert]` → `B480&=~0x0F` PERST deassert →
   `[PcieTunnel-Enable]`).
8. **Downstream PCIe link to GPU.** `pcie_downstream_link_bringup(3578)` LTSSM +
   `pcie_link_up_check_b432_e765(e2a6)` (`(B432&7)==7 && E765.2` = `0x78`
   CONNECTED). The bridge-route de-alias (`c5ff`'s 4× CfgWr, **LIFTED** out of 9037 and
   replayed via the adb0 mailbox — see §NVMe note) makes a tunneled CfgRd/MemRd return the
   real **AMD GPU 1002:7590** instead of the 5555:5555 route-echo. *Necessary-but-not-sufficient*
   for HOST lspci visibility — the host still gates on the `c0a5`/`0xE8` adapter-deassert (step 7).

**Loop/ISR minimal keep-set:** `cb10`+`eea5`+`dee3`+`d3a2`+`c7a5`/sb-connect-service
(superloop); `a066`+`c0a5`+`e911` (INT1 GPU); `af5e`+`b4ba`+`e1c6` (PD);
`boot_phy_bringup_early`+`cd6c`+`d127`+`b1cb`+`e56f` (boot). Drop the entire a59
NVMe branch (`9037`/`c00d`/`e8e4`) + `480c`/`3419`/`180d` + `usb_msc_init` —
**EXCEPT lift `c5ff`'s 4× CfgWr de-alias out of 9037** and replay it via the adb0
mailbox in a GPU-path one-shot (PERST-free); keep the SHARED primitives `e91d`/`d956`/`9ee5`.

---

## OPEN QUESTIONS / GAPS NOTED

1. **PCIe-Down adapter never enabled (THE wall).** Host enumerates the router
   (`1-0:1.1`) but never enables a PCIe-DOWN adapter → never posts `0xE8` → `e4a6`
   never runs → GPU stays in PERST. Need to instrument STOCK to capture what it
   writes between `[PcieTunnel-PwrOn]` and `[PcieTunnel-Deassert]`, and what
   triggers the host's first `0xE8`. Adapter-CS page-1 writes (`0x1208/0x1210/
   0x1334`) tested 2× with no host reaction.
2. **`af38` SBTX[1] = 0x55 (uninit).** The `e391` width-LUT seed (gated
   `0x0776==0`) never fires on the live Connect_U4 path → TX=`0C55` vs stock
   `0C03`. ROM `0x514c[0x0C]=0x03`. Necessary-but-maybe-not-sufficient: a prior
   force-seed gave TX=`0D04` exactly like stock "yet host still posted 0000" —
   the `SB[0x0C]` non-advance (00 vs stock 08→0B→0C) is the other half.
3. **`0x0819` lane-advertise mask / `cap20g_gate1` (`0x09F6`).** Stock advertises
   BOTH lanes (`0x0819=0x03`); the `cap20g_gate1=0` "fix" is WRONG (forces
   lane0-only `0x01`, strands `077A=0x00`, route-id `3C00` instead of `303C`, no
   bond). Set `0x09F6=1` (stock default) so lane1 slot populates → terminal
   route-id `303C` → host raises `SB[0x9E].0` → bond.
4. **`SHARED?` items — RESOLVED/updated (Phase 2):** `c5ff` route de-alias = **[SHARED], LIFT
   out of 9037** (see §NVMe note); shares `d956`/`e91d`/`9ee5` ([SHARED] primitives) with NVMe
   enroll `a183` (NVMe-ONLY). `d3a2` (runtime tunnel PERST deassert) — **KEEP**; real gate is
   `0x06EB.1 && u4_connect_gate.3` (NOT `0x0AF1.3`), call-site `af38_t53 && TIMER1_CSR.1`.
   `bank0_8a89` (suspend/resume PHY loop that drains PD during resume) — still SHARED?, re-verify.
5. **State-5 walker port.** The `8000` CL-walker (live AMD path, `0x0718==4`) is
   the engine that drives `SB[0xA0/A1] 0x07→0x02`. LOOP2 (CL-state) `0x30` reads
   `0x0779`; the dead `850b` walker's `0x30` reads `0x0B26` — do not confuse the
   two when porting.
6. **Not yet decompiled (low priority):** `e788` (port-walk commit), `FUN_CODE_180d`
   (NVMe queue drain detail), `c874`/`c6d3` (per-port post-init),
   `e25e` (downstream tunnel finalize). **CORRECTION (Phase 2):** `d2a1` is **NOT** a
   `cm_cmd_table` — `0xd2a1` is mid-instruction (the `#0x7f` operand of `CJNE A,#0x7f` at
   `CODE:d2a0`) inside the real code fn `FUN_CODE_d26f`; it has no xrefs because it is not a
   symbol. Fold into `d26f`; the `cm_<KEY>` (RHMG/RDCP/…) dispatch table is elsewhere/unconfirmed.

---

### Source files (handmade ports / RE references)

- `/home/batman/asm2464pd-firmware/handmade/src/main.c` — handmade superloop
- `/home/batman/asm2464pd-firmware/handmade/src/usb4_lanebond.h` — cb10/e672/af38 port
- `/home/batman/asm2464pd-firmware/handmade/src/sb_router.h` — SB transport port
- `/home/batman/asm2464pd-firmware/handmade/src/cm_tunnel.h` — 9037/adb0 CM port
- `/home/batman/asm2464pd-firmware/handmade/src/pd.h`, `vdm.h` — PD/VDM port
- `/home/batman/asm2464pd-firmware/handmade/src/boot_phy.h` — boot PHY port
- `/home/batman/asm2464pd-firmware/handmade/USB4_RE.md` — RE notes
- `/home/batman/asm2464pd-firmware/src/include/registers.h` — register map
