# Handmade ASM2464PD Driver Map (handmade/src/*) — vs stock

Phase-2 companion to `STOCK_DRIVER_MAP.md`. Maps what the **handmade** firmware
(`handmade/src/main.c` + headers) actually implements, in the SAME subsystem
structure as the stock map, with a per-function **Status** vs its stock
counterpart. Produced 2026-06-19 by a 9-mapper + completeness-critic workflow
(Ghidra-cross-checked). The structural diff lives in `DRIVER_DISCREPANCIES.md`.

> Addresses in the **Stock counterpart** column are stock Ghidra addresses
> (`CODE:` = bank0/common flat; `CODE_BANK1::` = banked). Handmade refs are
> `file:line` in `handmade/src/`.

## Class legend
GPU-PATH · SHARED · SHARED? (uncertain, re-verify) · NVMe-ONLY · PD · USB-DEV · HOUSEKEEPING · ➕ handmade-only

## Status legend
| Glyph | Meaning |
|---|---|
| ✅ | ported byte-true to stock |
| ⚠️ | partial — ported but incomplete / known-diverged |
| 🔶 | stub — present but does nothing real |
| ❌ | missing — stock has it, handmade does not |
| ➕ | handmade-only — no stock counterpart (custom mechanism / diagnostics) |
| 🚫 | omitted-NVMe — intentionally not ported (NVMe storage path) |

---

## Top-level architecture & divergences

Handmade is a **GPU-tunnel-focused subset** of the stock USB4-NVMe-enclosure
firmware. Major architectural divergences from stock:

1. **No SPI-cap-blob pipeline.** Stock `4c40`→`92c5`→`8d77` reads the cap/mode
   from flash. Handmade has **no SPI shadow load**: it hard-sets `0x09F9=0x87`
   and seeds the cap-gates (`0x09F5-F8/FB`), adapter-cfg (`0x0A52-55`) and PID
   (`0x0A57/58`) from a captured stock wire-trace (`main.c:496-516`). Correct on
   this board; **diverges if the board is OTP/strap-fused**.
2. **Vendor bulk EP replaces USB-MSC/NVMe storage.** The entire stock storage
   data-path (`usb_msc_init`/`3419`/`180d`/`1196`/`488f`/`3e81`) is dropped and
   replaced by a tinygrad vendor-bulk interface (`handle_usb_bulk_data`, vendor
   control reqs 0xC0/0xE4/0xE5/0xF0/0xF2/0xF3). → 🚫 omitted-NVMe + ➕ handmade-only.
3. **`pcie_power_on()` replaces the stock `c00d`/`3578` runtime tunnel-up.** The
   stock CM step-machine `9037` + its `c00d` PERST/PHY re-drive arm is **dead**
   in handmade (removed 2026-06-18 — it races the downstream LTSSM → stuck 0x01).
   Handmade trains the downstream link with a custom `pcie_power_on` one-shot
   after the lane bond. → ➕ handmade-only (PERST-race-free design).
4. **The CM router-op mailbox (`c0a5`) is a STUB.** `cm_routerop_mailbox`
   (`usb4.h:63`) acks `EA90` but never dispatches the host's `E2`/`E3`/`E8`
   config/tunnel ops → host posts `EC06=0` → never `0xE8` → GPU stays in PERST.
   **This is the dominant remaining wall.**

---

## GPU-tunnel critical path — handmade status (mirrors STOCK_DRIVER_MAP §"GPU-TUNNEL CRITICAL PATH")

| # | Step | Handmade status |
|---|---|---|
| 1 | PD contract (hard-reset prompt → Source_Cap → Request RDO) | ✅ working on the wire |
| 2 | VDM + Enter_USB4 (Discover_ID/SVIDs/Modes/Enter_Mode/Enter_USB4) | ✅ working; ⚠️ cmd5 Enter_USB4-ACK SB hand-off (`ca71`) thin (see §6) |
| 3 | USB4 lane bond (SB transport → state-3/4/5 → SB[A0/A1]=0x02) | ⚠️ reaches bond but **CL-terminal/lane-orientation diverges** (`lp1-finalize-bit6`, §4) |
| 4 | SB/router transport up; router enumerates `1-0:1.1` | ✅ host enumerates the host router |
| 5 | Host posts router-ops (`EC06`) → `c0a5` E2/E3 cfg | ❌ **mailbox is a STUB** — host posts `EC06=0` |
| 6 | PCIe-Down adapter advertised/enabled (`cd6c`) | ✅ adapter advertised at boot; ❌ host never enables it (consequence of #5) |
| 7 | Tunnel deassert (host posts `0xE8` → `e4a6` PERST deassert) | ❌ **`E8`/`e4a6` missing** — `0xE8` never posted |
| 8 | Downstream PCIe to GPU + route de-alias (`c5ff`) | ⚠️ `pcie_power_on` trains local link to `0x78`; ❌ **c5ff de-alias not lifted** (DEBUG `cm_route_test` only) |

**Net:** the chain stalls at step 5 — the stubbed router-op mailbox — with the
step-3 CL-terminal and step-8 c5ff-de-alias as the next two GPU-path gaps.

---
## 1. Boot / Init

Handmade boot is a flat `main()` (main.c:476-616) that replaces the stock orchestrator `main_boot_and_superloop(2f80)` + `boot_hw_init_main(4fb6)` chain. It cherry-picks the GPU/USB4-relevant stock steps as direct calls and **drops the SPI-cap-blob path** (stock `8d77`/`92c5` SPI override), instead hard-coding `0x09F9=0x87` and seeding adapter-cfg `0x0A52-55` + PID `0x0A57/58` from captured stock SPI bytes. The NVMe tail (`usb_msc_init 4904`, `c00d` boot CM-arm) is intentionally omitted; the superloop CM machine was removed 2026-06-18.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `main` (main.c:476) | `main_boot_and_superloop` `CODE:2f80` | Boot orchestrator: flash→PHY-keystone→link-seed→cap-seed→tunnel-adapter→pipe/PHY arm→PD→IRQ arm→routerop→XDATA seed→IE enable→superloop | SHARED | ⚠️ |
| `int1_isr` (main.c:462) | `int1_isr` `CODE:0013` | INT1/EX1 PD/USB4/CM aggregate ISR (demux to `usb4_int_demux`, `pd_rx_isr`, `cc_pd_timer_tick`) | SHARED | ⚠️ |
| `flash_init` (flash.h:7) | part of `bank0_e14b` `CODE:e14b` (FLASH_DIV=4) | SPI flash controller bring-up: C806.2 ack, AUX-status W1C, FLASH_DIV=4 | SHARED | ⚠️ |
| `flash_cmd`/`flash_read_otp` (flash.h:25,52) | `e14b` flash-read engine `CODE:e14b` (b8b9/b820/be02…) | SPI cmd issue + OTP serial read for USB enumeration | SHARED | ⚠️ |
| `boot_phy_bringup_early` (boot_phy.h:131) | `boot_phy_bringup_early` `CODE:ce79` | **SBU/PHY KEYSTONE** — Type-C SBU + PHY config + SB-block enable + PCIe-tunnel pre-stage | SHARED | ✅ |
| `boot_phy_d0d3_typec_sbu` (boot_phy.h:37) | `d0d3` (via ce79) | Type-C SBU PHY bring-up (CC10 mailbox sequence) | SHARED | ✅ |
| `boot_phy_cf28` (boot_phy.h:53) | `cf28` (via ce79) | CC3x/E3xx/E71x link-register PHY config block | SHARED | ✅ |
| `boot_phy_bank1_ed02` (boot_phy.h:73) | `ed02` (via ce79) | SB[0x05].7 sideband-block power enable | SHARED | ✅ |
| `boot_phy_dd42` (boot_phy.h:82) | `dd42` (via ce79) | E7E3 PHY-config latch from mode selector | SHARED | ✅ |
| `boot_phy_d996_pcie_tunnel_boot` (boot_phy.h:122) | `d996` (via ce79) | Downstream PCIe-tunnel pre-stage (B402/C659/E764/lane-pwr/width) | SHARED | ✅ |
| `boot_phy_d630_lane_power` (boot_phy.h:104) | `d630` | PCIe-tunnel lane power + PLL-latch clears (E76C/E774/E77C.4) | SHARED | ✅ |
| `boot_phy_d436_width` (boot_phy.h:117) | `d436` | PCIe-tunnel lane-width program (B434/B436) | SHARED | ✅ |
| `bank0_92c5_seed` (boot_phy.h:151) | `bank0_92c5_seed` `CODE:92c5` | Lane-engine WIDTH/MODE/state RAM seed (0x0AE3-0xAF1, gen=3 lane=3) | SHARED | ⚠️ |
| `pcie_tunnel_adapter_config_b410` (boot_phy.h:188) | `c8db` (via cd6c) | PCIe-tunnel adapter cap/path cfg B410-B42B from 0x0A52-55 | SHARED | ✅ |
| `pcie_tunnel_adapter_enable_b401` (boot_phy.h:211) | `pcie_tunnel_adapter_enable_b401` `CODE:cd6c` | PCIe-over-USB4 TUNNEL MASTER ENABLE (B401/B480-PERST/B298 TLP-route) | SHARED | ✅ |
| `usb_pipe_engine_init` (usb.h:181) | PIPE-engine block of `boot_usb4_vs_usb3_mode_decision` `CODE:b1cb` | USB PIPE/PHY engine bring-up (POWER_EN, 91Cx, 93xx, USB_MODE) | SHARED | ⚠️ |
| `usb4_phy_arm` (usb.h:224) | CC10 link-arm in `b1cb` | Arm upstream USB4 PHY link-up via CC10 mailbox (subcmd 4, wait E318.4) | SHARED | ⚠️ |
| `usb_phy_tune`/`usb_serdes_tune_lane` (usb.h:161) | `usb_phy_tune` (non-USB4 fork) | SERDES lane tune C280/C300 — **gated to non-USB4 only** | USB-DEV | ⚠️ |
| `usb_init_controller` (usb.h:166) | USB3 device bring-up (non-USB4 fork) | USB device controller init — **skipped in USB4 mode** | USB-DEV | ⚠️ |
| `pd_keystone_init` (pd.h:158) | `4be6` (INT1 arm) + `8d77` (mode) + `baa0` (CC term/PD) | INT1 enable group + force `u4_mode_flag=0x87` + PD PHY term + PD state init | SHARED | ⚠️ |
| `pd_int1_enable_group` (pd.h, via keystone) | `pd_int1_enable_group` `CODE:4be6` | INT1 enable group + fwver `7F0..7F5` + ef24/ef1e USB4/CM arm | SHARED | ⚠️ |
| `usb4_irq_arm` (usb4_irq.h:242) | `init_sys_flags`/ef24/ef1e portion of `4be6` | Arm USB4 SB-transport/router INT1 sources (CC3B, ef24, ef1e) | SHARED | ⚠️ |
| `usb4_routerop_init` (usb4_irq.h:253) | `bank1_e56f` `CODE_BANK1::e56f` | C0A5 router-op mailbox RX-enable (EC00, EA88/89 speed 0x2464, EC05, C807) | SHARED | ⚠️ |
| `pcie_power_off` (main.c:82) | (non-USB4 fork PCIe pwr-down) | Hold downstream device in reset + remove rails — non-USB4 only | ➕ | ➕ |
| `pcie_power_on` (main.c:91) | replaces `c00d`/`3578` runtime tunnel-up | Custom downstream LTSSM tunnel-up (PERST deassert + train to 0x78) — PERST/PHY-race-free design | ➕ | ➕ |
| `i2c_init` (i2c.h:34) | absent | I2C peripheral bring-up for INA231 debug shunt | HOUSEKEEPING | ➕ |
| `ina231_init` (i2c.h:101) | absent | INA231 power-monitor init (1 Hz debug loop) | HOUSEKEEPING | ➕ |
| crt0 `__sdcc_program_startup` (crt0.s:35) | RESET `CODE:0000` | Zero IRAM, SP=0x7F, DPX=0, →main | HOUSEKEEPING | ⚠️ |
| absent | `boot_ramstate_reset` `CODE:4c40` | Zero PD/USB RAM + defaults `0x09F9=0x04/0x09FA=0x04/0x09FB=0x00` | SHARED | ❌ |
| absent | `usb4_cap_apply_09f9` `CODE:8d77` | SPI-cap-blob loader → runtime cap/mode_flag; replaced by hardcoded 0x87 + 0x09F5-FB seeds | SHARED | ❌ |
| absent | `hddpc_phy_init` `CODE:5284` | HDDPC/PHY analog cfg (PHY_EXT_5B/2D) | SHARED? | ❌ |
| absent | `boot_ramstate_reset_and_analogcfg` `CODE:5305` | E795 strap sample → d6bc analog; 0x7F6=1 | SHARED? | ❌ |
| absent | `e597` UART/clk init `CODE:e597` | UART FCR/LCR console enable (handmade does this inline in main head) | HOUSEKEEPING | ❌ |
| absent | `bank0_de16` `CODE:de16` | PHY-DMA cfg + timer + sb_channel_connect_service | SHARED? | ❌ |
| absent | `bank1_eef9` `CODE:eef9` | INT_ENABLE\|=0x40 (bank1 USB4 INT group bit6) | SHARED | ❌ |
| absent | `bank0_d127` `CODE:d127` | PCIe-DMA engine ring config (SIZE/BUF/DOORBELL/CEF2-3/B281) | SHARED? | ❌ |
| absent | `bank0_bf8e` tail-init `CODE:bf8e` | Tail: connect-state clear + timers + c00d + usb_msc_init | SHARED? | ❌ |
| absent | `init_timer2_and_timer4` `CODE:e19e` | TIMER2/4 div + threshold | HOUSEKEEPING | ❌ |
| absent | `bank0_d894` `CODE:d894` | USB4 fork: INT/PCIe-tunnel adapter regs (0x121E.0 bank-2 cluster) | SHARED? | ❌ |
| absent | `bank0_c00d` `CODE:c00d` | Boot CM-arm / tunnel-power-on (PERST/C659 re-drive) | NVMe-ONLY | 🚫 |
| absent | `usb_msc_init` `CODE:4904` | NVMe/USB-MSC BOT init ('USBS' CSW, MSC EPs) | NVMe-ONLY | 🚫 |

### State machines

| Name | State var | Stock states / transitions | Handmade |
|---|---|---|---|
| u4_mode_flag (USB4 vs USB3 cap) | XDATA `0x09F9` | `0x04`(4c40 default)→`8d77` SPI maps to `0x87/06/85/C1`→`b1cb` overrides to 1/2 if still 4 | **Hard-set `0x87`** (main.c:496); no 4c40 default, no 8d77 SPI path, no b1cb 1/2 override. Gate `(&0x83)`/`(&0x81)` used identically downstream. |
| cap20g_gate1 | XDATA `0x09F6` | seeded 1 by 8d77 head; SPI `0x07059.7` may clear | Hard-set `1` (main.c:498) matching stock default |
| USB-enum / superloop (a59) | XDATA `0x0A59` | 0=idle,1=USB4-connect(9037+c00d),2=USB3 | Seeded `0` (main.c:608); NVMe `1` arm + `c00d` REMOVED |
| c00d one-shot | XDATA `0x06E6` | 1=arm pending,0=consumed | Seeded `0` (main.c:606); CM machine dead |

**Diff note:** Handmade preserves the GPU-critical SBU/PHY keystone (`boot_phy_bringup_early`/ce79) byte-true and lifts `pcie_tunnel_adapter_enable_b401`/cd6c to a direct boot call (stock runs it inside `boot_hw_init_main` step 6k), so the PCIe-down tunnel adapter is advertised. The major architectural divergence is the elimination of the SPI-cap-blob pipeline (`4c40`→`92c5`→`8d77`): there is **no SPI shadow load**, so the cap (`0x09F9`), cap-gates (`0x09F5-F8/FB`), adapter-cfg (`0x0A52-55`) and PID (`0x0A57/58`) are all hardcoded from a captured stock wire-trace rather than read from flash. This is correct on the current board but **diverges if the board is OTP/strap-fused** — bank0_92c5_seed (boot_phy.h:151-162) explicitly omits the `0x707E==0x5A` fuse-override probe and only prints the fuse bytes, so a fused board's lane-mask/gen/width would not match stock. Several stock infra steps are dropped: `bank1_eef9` (INT_ENABLE|=0x40 bank1 group) is ❌ missing — verify the bank1 USB4 INT group is armed elsewhere (usb4_irq_arm covers ef24/ef1e but not eef9's 0x40 bit); `bank0_de16` (PHY-DMA + sb_channel_connect_service) and `bank0_d127`/`d894` (PCIe-DMA ring + bank-2 0x121E adapter cluster) are SHARED?-uncertain omissions that may matter for tunnel DMA. The NVMe layer (`c00d` boot-arm, `usb_msc_init`) is correctly 🚫 omitted per the resolved CM-tunnel boundary, and the custom `pcie_power_on()` (➕) replaces the harmful stock `c00d`/`3578` PERST-re-drive path.

---
## 2. ISR Handlers

Only **two** interrupt vectors are wired in handmade, matching stock's two-vector design: `int0_isr` (`__interrupt(0)`, USB-device) and `int1_isr` (`__interrupt(1)`, USB4/PD/CM orchestrator). Timer0/Timer1/UART vectors are unused (UART polled), same as stock.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `int1_isr` (main.c:462) | `int1_isr_orchestrator` (CODE:4486) | TOP-LEVEL INT1 demux: C806.0→tick, C80A.6→pd_rx, EX-gated usb4_int_demux, C806.4 ack-only | SHARED | ⚠️ partial (missing cd10 + c105 branches; tunnel branch is W1C-only) |
| `cc_pd_timer_tick` (called main.c:466; defined in cc/pd hdr) | `cc_pd_timer_tick` (CODE:b4ba via 0x0520) | CC/PD timer-tick on C806.0 (`REG_INT_SYSTEM&0x01`) | PD | ✅ ported-byte-true (wired) |
| `pd_rx_isr` (called main.c:469; defined in pd.h) | `pd_rx_isr` (CODE:af5e via 0x052f) | PD-RX msg dispatch on C80A.6 (`REG_INT_PCIE_NVME&0x40`) | PD | ✅ ported-byte-true (wired) |
| `usb4_int_demux` (usb4.h:91, called main.c:471) | (no single stock fn — stock inlines branches a066/c105/c0a5/e911 in 4486) | USB4 INT source demux gated `0x09F9&0x83`; reads C80A (`REG_INT_PCIE_NVME`) | GPU-PATH | ➕ handmade-only (consolidates the gated stock branches) |
| `sb_router_event_handler` (sb_router.h:558; called usb4.h:96 on C80A.5/bit0x20) | `sb_router_event_handler_M2` (CODE_BANK1::a066 via 0x061a) | SB-router connect / lane-bond → tunnel | GPU-PATH | ⚠️ partial (wired; body diverges per §1/§5) |
| `cm_routerop_mailbox` (usb4.h:63; called usb4.h:106 on EC06.0/`REG_NVME_EVENT_STATUS&0x01`) | `cm_routerop_mailbox` (CODE_BANK1::c0a5 via 0x0499) | CM router-op config-TLP mailbox; gate EA90==0x5A; state 0x0B02 | SHARED | 🔶 stub (gates EA90==0x5A, only latches opcode + bare A5-ack on E2/E3 continuation; NO movc E0..E8 dispatch, NO CfgRd/CfgWr, NO C805\|=0x02 reply) |
| tunnel-bit W1C block (usb4.h:108-113, on C80A.0-3/`int_sources&0x0F`) | `tunnel_link_event_e763` (CODE_BANK1::e911 via 0x0570) | PCIe-tunnel link UP/DOWN E763.2/.3 | GPU-PATH | ⚠️ partial (W1Cs REG_PHY_RXPLL_TRIGGER .2/.3 only; NO PcieLinkUp print / d17e processing) |
| C806.4 branch (main.c:472, `REG_INT_SYSTEM&0x10`) | `bank1_ef4e` (CODE_BANK1::ef4e via 0x0642) | C806.4 source — empty stub | HOUSEKEEPING | ✅ ported-byte-true (no-op, matches stock bare RET) |
| _absent_ | `bank0_cd10` (CODE:cd10 via 0x0390) | **PCIe-downstream link bringup/reset on CC33.2** (W1C=4, CC31 spin) | SHARED | ❌ missing (no CC33.2 branch in int1_isr) |
| _absent_ | `usb4_sec_adapter_link_event_c80a4` (CODE:c105 via 0x0593) | Secondary USB4 adapter/link-width events on C80A.4 | SHARED | ❌ missing (demux only acks bit 0x10 into `usb4_int_seen`, no handler) |
| `int0_isr` (main.c:374) | `int0_isr body` (CODE:0e5b via CODE:0003) | USB-device ISR: peripheral EP/bulk/CBW + USB-PHY 91D1 + buffer-cfg 9300/01/02 + MSC tail | USB-DEV | ⚠️ partial (§8 territory; SS-fail USB2-fallback gated off in USB4 mode; NVMe-queue tail omitted) |

### State machines

| Name | State var | States | Class | Status |
|---|---|---|---|---|
| `u4_routerop_mbox_state` (usb4.h, RMBOX_IDLE/MULTIPKT_1/MULTIPKT_2) | `usb4_routerop_mbox_state` @0x0B02 | IDLE latches EA80 opcode; MULTIPKT_1/2 only bare-ack E2/E3 then return IDLE | SHARED | 🔶 stub (no movc dispatch table, no per-opcode E0..E8 work) |
| `usb4_int_seen` sticky bitmap (usb4.h:55) | (no stock equiv) | bit0=SB,bit1=evt(0x10),bit2=routerop,bit3=tunnel | HOUSEKEEPING | ➕ handmade-only (diagnostic accumulator) |
| SB active-port round-robin (sb_router.h `0x06F1`) | `0x06F1` | 0..3 SB channel index for C9 connect service | GPU-PATH | (see §1/§5) |

### diff note

The handmade INT1 dispatch preserves the **PD-critical** order — `cc_pd_timer_tick` (C806.0) and `pd_rx_isr` (C80A.6) run unconditionally, exactly like stock — which is why PD completes even when USB4 stalls. The USB4-gated branches (`0x09F9&0x83`) are folded into the single `usb4_int_demux`, which correctly reads C80A (`REG_INT_PCIE_NVME`) and routes bit0x20→`sb_router_event_handler` (=stock a066, GPU-PATH, WIRED). However three stock INT1 branches are **degraded or missing**, and two of them are GPU/tunnel-relevant: (1) **`cd10` (CC33.2 PCIe-downstream bringup) is entirely absent** — there is no CC33 read in int1_isr at all; in handmade the downstream link is driven by the custom superloop `pcie_power_on()` instead, so this is by-design under the resolved CM-tunnel boundary, but it means the ISR-time downstream-reset path does not exist. (2) **`c105` (C80A.4 secondary-adapter / link-width) is absent** — the demux merely sets `usb4_int_seen|=0x02` and drops the event; this is SHARED infra (link-width recovery / CC re-arm) and its omission is a real gap. (3) The **C80A.0-3 tunnel branch is W1C-only** — it acks `REG_PHY_RXPLL_TRIGGER` (E763) bits .2/.3 but never runs `e911`'s `[PcieTunnel-PcieLinkUp]` + `d17e` processing, so tunnel-link-up events are silently swallowed. Most consequential for GPU visibility: **`cm_routerop_mailbox` is a stub** — it gates on `EA90==0x5A` and only echoes an `A5` ack for E2/E3 continuation packets, with NO movc opcode dispatch (E0..E8), NO CfgRd/CfgWr-over-tunnel, and NO `C805|=0x02` reply arming. This is the §0-noted host-visibility gate: the host posts `EC06=0` (never 0xE8) partly because nothing here ever services/answers a real router-op. `int0_isr` is §8 USB-device territory but is the second of the two ISR vectors; its USB4-relevant divergence is the deliberate SS-fail-no-USB2-drop gate (main.c:418) and the omitted NVMe-queue service tail.

---
## 3. Superloop

Stock superloop = `main_boot_and_superloop @ CODE:2f80` (outer/inner do-while, inner until INTMEM `0x6A==0x0B`). The handmade superloop is `main()`'s `while(1)` at **main.c:621–821**. It is a *flattened, inlined* re-implementation: stock's monolithic per-pass dispatch table (the a59 NVMe SM, `c7a5`, `cb10`, `e1c6`, the `541f`-gated `dee3`/`480c`/`d3a2`, `e2ec`) is replaced by **(a)** an INT1-ISR-driven SB/router/tunnel path (the c7a5/cb10-FSM/dee3 work moved *into* the INT1 handler `sb_router_event_handler` and the deferred responders), and **(b)** a small set of superloop-resident deferred blocks. The entire a59 NVMe branch was dropped.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `main` while(1) body (main.c:621) | `main_boot_and_superloop` `CODE:2f80` | Superloop shell (outer loop only; no inner `0x6A==0x0B` bus-reset loop) | SHARED | ⚠️ |
| FSM-advance block (main.c:633–667), gate `(09F9&0x83)&&06EC` | `CODE_BANK1::cb10` u4lb_link_monitor_tick | Per-pass GPU-tunnel tick: lane-latch + state-3/4/5 dispatch + ee57 throttle + cdf5 tail. Stock's single `cb10` is here *inlined* as 4 calls. | GPU-PATH | ⚠️ |
| `sb_cb10_lane_advance` (sb_router.h:702) | `CODE_BANK1::cb10` (monitor head only) | SB[0xA0/0xA1] CL-nibble latch + cb10_seen diag; the *monitor* half of stock cb10. Does NOT itself dispatch the FSM. | GPU-PATH | ⚠️ |
| `u4lb_e672` (usb4_lanebond.h:1477) | `CODE_BANK1::e672` | The real lane-bond state-3/4/5 dispatcher (stock cb10's tail call, gated 06ED!=0). | GPU-PATH | ⚠️ |
| `u4lb_ee57` (usb4_lanebond.h:560) | `CODE_BANK1::ee57` | Live CCE4:CCE5 counter read for the state-5 walker throttle (R6:R7 in stock). | GPU-PATH | ✅ |
| `sb_cdf5_routerop_response` (sb_router.h:523) | `CODE_BANK1::cdf5` | cb10-tail deferred router-op CONFIG-READ response (the lane-config TX the host blocks on), gated `072A`. | GPU-PATH | ⚠️ |
| `u4lb_eda0` (usb4_lanebond.h:967) | `CODE_BANK1::eda0` | 0x0775/0x0719 token-clear; yields the cdf5 build/skip selector. | GPU-PATH | ✅ |
| `sb_a5d8_pend_int` (sb_router.h:415) | `CODE_BANK1::a5d8` | `[Pend Int]` device→host router-op responder; deferred off INT1 to keep stack shallow. | GPU-PATH | ⚠️ |
| `sb_router_event_handler` (sb_router.h:558) | `CODE_BANK1::a066` (INT1 body; stock's superloop `c7a5` connect-service is subsumed here) | SB connect/lane-bond service — driven from the **INT1 ISR** (usb4.h:96), not the superloop. Calls `sb_channel_connect_service`. *(Stock-ref reconciled: a066, not c7a5 — see §2/§4.)* | GPU-PATH | ⚠️ |
| `sb_channel_connect_service` (sb_router.h:332) | `CODE_BANK1::c3b2` (= a066 PART-1; stock superloop `c7a5` calls into this) | Per-port SB connect descriptor service (n==1/3/5/0 arms). *(Stock-ref reconciled: c3b2, not c7a5 — see §4B.)* | GPU-PATH | ✅ |
| deferred-8a89 block (main.c:680–694) → `bank0_c9a8` (usb4_connect.h:193) | `CODE:8a89`/`c9a8` (re-entry; not a stock superloop call) | NON-STOCK fallback: drive bank0_c9a8(0) once after `fsm_stall>=6` and no bond. | SHARED? | ➕ |
| deferred tunnel-up block (main.c:700–705) → `pcie_power_on` (main.c:91) | replaces `CODE:d3a2`/`c00d`/`3578` | Fire downstream PCIe bring-up once both lanes reach CL0 (SB[A0]&A1 nibble==2). | GPU-PATH | ➕ |
| `pd_drive_hard_reset` kick (main.c:817–820) | (custom milestone-1 PD prompt) | Periodic Hard-Reset until `pd_seen`; not a stock superloop element. | PD | ➕ |
| E302/TICK/M2/LANE/S4/P2 diag blocks (main.c:713–816) | absent | Heartbeat/training telemetry. | HOUSEKEEPING | ➕ |
| absent | `CODE_BANK1::eea5` bank1_eea5 (CDR re-arm 'RHMG') | PHY CDR/RX-PLL re-drive feeding lane training (stock cb10-pass companion). | GPU-PATH | ❌ |
| absent | `CODE:dee3` bank0_dee3 | `sb_cb10_lane_advance` drain: `==10` spin / `==0x0b` cb23/cadf + clear entered_usb_mode. | GPU-PATH | ❌ |
| absent | `CODE:d3a2` bank0_d3a2 (tunnel PERST deassert) | Runtime tunnel completion / downstream PERST# deassert + c2e6, `541f`-gated. Functionally REPLACED by `pcie_power_on`. | SHARED | ❌ |
| absent | `CODE:541f` E716 link-up guard | `REG_LINK_STATUS_E716 & 3` gate for dee3/480c/d3a2. | SHARED | ❌ |
| absent | `CODE:e2ec` bank0_e2ec | USB4 post-connect reinit (e869/e95f). | SHARED | ❌ |
| absent | `CODE:e1c6` pd_tx_commit_engine (superloop pass) | Stock commits staged PD-TX in the superloop on `pd_tx_staged_pending`; handmade commits it *inline* from pd_dispatch.h/vdm.h instead — not a superloop pass. | PD | ⚠️ |
| absent | `CODE:9037` cm_pcie_link_step_machine | Polled NVMe CM PCIe-link bring-up + per-port enroll walk. | NVMe-ONLY | 🚫 |
| absent | `CODE:c00d` bank0_c00d (CM arm) | Arms 9037 (PERST/C659/width re-drive + 0x05B4=0x10). Removed (races pcie_power_on). | NVMe-ONLY | 🚫 |
| absent | `CODE:e8e4` bank0_e8e4 | CM re-arm wrapper → c00d. | NVMe-ONLY | 🚫 |
| absent | `CODE:ca0d` bank0_ca0d (mode-finalize) | USB link-mode finalize (latches entered_usb_mode=0x10). | USB-DEV | ❌ |
| absent | `CODE:e677` bank0_e677 (timer rearm) | Rearm TIMER2/4 outer-loop housekeeping. | HOUSEKEEPING | ❌ |
| absent | `CODE:480c` FUN_CODE_480c | NVMe MSC/DMA service. | NVMe-ONLY | 🚫 |
| absent | `CODE:3419` FUN_CODE_3419 (USB-SS enum/MSC) | SuperSpeed enum + SCSI/MSC CBW→DMA (sets 0x06E6=1). | NVMe-ONLY | 🚫 |
| absent | `CODE:180d` FUN_CODE_180d (USB2 enum + SCSI) | Legacy MSC SCSI/CBW + NVMe DMA walk. | NVMe-ONLY | 🚫 |

### State machines

| Name | State var | Handmade location | States | Class | Status |
|---|---|---|---|---|---|
| USB4 lane-bond FSM | `0x06ED` | main.c:633 gate `06EC!=0` → `u4lb_e672` | 0=idle (monitor only) ; 3/4/5 via e672 ; state-5 done → 0 | GPU-PATH | ⚠️ |
| cb10 state-5 walker throttle | `0x076A:0x076B` snap vs ee57 | main.c:646–658 | advance when `snap - live >= 3` | GPU-PATH | ✅ |
| cdf5 router-op sub-state | `0x072A` (sb_cdf5_substate_arm) | main.c:662 | one-shot arm; selector from eda0 | GPU-PATH | ⚠️ |
| USB-enum / device-bringup (a59) | `0x0A59` | DROPPED (seeded=0 main.c:608) | — | NVMe-ONLY | 🚫 |
| CM PCIe-link step machine | `0x0B39`/`0x06E5`/`0x05B4` | DROPPED (cells seeded idle main.c:600–609) | — | NVMe-ONLY | 🚫 |
| USB device MSC/CBW inner loop | IRAM `0x6A==0x0B` | absent (no inner loop) | — | NVMe-ONLY | 🚫 |

### diff note

The handmade superloop is structurally inverted vs stock: stock runs `c7a5` (connect service) and `cb10` (lane FSM) as **always-run superloop passes**, whereas handmade runs the connect-service half via the **INT1 ISR** (`sb_router_event_handler` → `sb_channel_connect_service`, usb4.h:96) and only keeps the cb10-FSM tick in the superloop (main.c:633), inlined as `sb_cb10_lane_advance` (monitor) + `u4lb_e672` (state-3/4/5 dispatch) + `u4lb_ee57` throttle + `sb_cdf5_routerop_response` tail — these four are byte-true to stock cb10's internal sequence. The entire **a59 NVMe branch** (9037/c00d/e8e4/480c/3419/180d) is correctly dropped (🚫 omitted-NVMe / removed 2026-06-18), and the boundary-resolution confirms this is *correct* (c00d's PERST re-drive races pcie_power_on). **No GPU-PATH function was silently lost on the a59 branch itself** — cb10/dee3/d3a2/c7a5 are GPU/SHARED and live outside a59. However three GPU/SHARED stock superloop passes are genuinely **MISSING**: `eea5` (CDR re-arm — ❌, not ported anywhere; this is a real lane-training feeder per the §3 minimal-loop set), `dee3` (lane-advance drain `==10`/`==0x0b` — ❌, no equivalent), and `e2ec`/`541f` (post-connect reinit + link-up guard — ❌). `d3a2` (runtime tunnel PERST deassert) is intentionally **replaced** by the ➕ handmade-only `pcie_power_on` one-shot (gated on both lanes CL0), consistent with the resolved boundary. Two non-stock additions to flag: the **deferred-8a89 fallback** (main.c:680, ➕ SHARED? — drives bank0_c9a8 after a synthetic `fsm_stall>=6`, no stock analog) and the dead `sb_pend_int_pending` flag (sb_router.h:552) which is **never set to 1**, so the main.c:670 a5d8 responder only fires on the live `SB[0x26]&0x02` poll — a latent half-wired deferral. `pd_tx_commit_engine` (stock e1c6 superloop pass) is committed inline from the PD/VDM dispatchers instead of the superloop (⚠️ relocated, functionally OK).

---
## 4. USB4 State Machines (lane-bond / PHY / SB-transport)
> **Part 4A — Lane-bond FSM (FSM-TICK path: cb10→e672→{a7de/b0b4/8000|850b}+cdf5)**
**Scope:** the FSM-TICK chain `cb10 → e672 → {a7de/b0b4/8000|850b} + cdf5`. All functions here are **GPU-PATH** (or SHARED PHY infra); no NVMe-only code in this section. State vars live in `usb4_state.h` (a pure `__at` header — no functions); the FSM bodies are in `usb4_lanebond.h`; the periodic driver (cb10) + cdf5 response live in `sb_router.h`/`main.c`.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `sb_cb10_lane_advance` (sb_router.h:702) | `CODE_BANK1::cb10` | Periodic SB lane-bond advance: reads SB[0xA0]/[0xA1] nibble vs 0x72B/0x72C latches. Handmade is a thinned-down version; the **e672 dispatch + ee57 throttle that stock cb10 owns is hoisted into the main.c super-loop** (main.c:633-666). | GPU-PATH | ⚠️ partial |
| main.c:646-658 (cb10 throttle+dispatch inline) | `CODE_BANK1::cb10` (cb8d-cbb3 throttle) | ee57 CCE4:CCE5 delta-≥3 throttle then `u4lb_e672()`; re-samples ee57 into 0x76A/0x76B. Byte-true throttle math. | GPU-PATH | ✅ ported-byte-true |
| `u4lb_e672` (usb4_lanebond.h:1477) | `CODE_BANK1::e672` | **THE lane-bond FSM dispatcher.** 3→a7de, 4→b0b4, 5→(all-lanes-clear→IDLE; else 0x0718==4→8000 else 850b). Verified vs Ghidra e672 (same 4-cell all-clear guard 0759/075A/075B/075C). | GPU-PATH | ✅ ported-byte-true |
| `u4lb_cm_conn_routing_setup` (usb4_lanebond.h:216) | `CODE_BANK1::a7de` | **STATE 3 [ConnRout].** 0x0758 sub-FSM (0x10/0x11/0x00); confirm gate 0x0777==0x0C; sets 0x0718=4 ROUTE-ENABLE; 0x077A→0x0819/0751/0750 lane-advertise; e391 seed; a912-a9c2 deep-PHY gate; c586 rate-desc; e175/e282/c17f router-op width tail. | GPU-PATH | ⚠️ partial |
| `u4lb_cm_conn_routing_setup` e391 inline (usb4_lanebond.h:260-273) | `CODE_BANK1::e391` | **GAP-1 width-LUT seed.** LOOP1 ROM 0x514c→sb_width_lut[0x06F2+i] + ROM 0x515f→sb_branchA_gate[0x0705+i]; LOOP2 zeroes CL-walk shadows (0xB26+). ROM tables byte-exact (idx 0x0C=0x03). **Gate is `0x0776==0` and the prior caller-site gate-never-fires bug is closed** (see diff note). | GPU-PATH | ✅ ported-byte-true |
| `u4lb_edf5_route_query`/`u4lb_e1cb_e2b9` (usb4_lanebond.h:29,980) | `CODE_BANK1::edf5`/`e2b9`/`e1cb` | State-3 ARM substate device→host route-query push (0x0719 in-flight gate). | GPU-PATH | ✅ ported-byte-true |
| `u4lb_e175`/`u4lb_e282`/`u4lb_c17f`/`u4lb_ce23`/`u4lb_c3ce`/`u4lb_e00c`/`u4lb_a2c2`/`u4lb_a310`/`u4lb_a37b`/`u4lb_a840`-engine helpers (usb4_lanebond.h:64-214) | `CODE_BANK1::e175/e282/c17f/ce23/c3ce/e00c/a2c2/...` | a9ca/a9d2/a9d5 router-op lane-width descriptor tail (advertise lane1 width via the P12 descriptor engine) so host grants 2-lane CL0. Previously OMITTED; now ported. | GPU-PATH | ✅ ported-byte-true |
| `u4lb_state4_b0b4` (usb4_lanebond.h:836) | `CODE_BANK1::b0b4` | **STATE 4** [PcieTunnel-PwrOn]: retrain guard, width-ready gate (Δ<0x38 abort), connect-present gate, e305 pwr-on, L0/L1 OS-arm (0x82/0xA2), Chg2-20G/RstRxpll/b8db CDR-validate, c593, [L0/L1 OS1], seeds LOOP1/LOOP2=0x10, ec51 Trig-arm, latch 074E/074F, →state 5. | GPU-PATH | ✅ ported-byte-true |
| state-4 helpers `e305/ee29/ed44/df61/a840/d436/c089/d702/b8db/e980/d3b0/e9e7/e764/ebde/e07d/d5da/c593/e26a/96fe/b226` (usb4_lanebond.h:422-833) | `CODE_BANK1::e305/ee29/ed44/df61/a840/d436/c089/d702/b8db/e980/d3b0/e9e7/cdc6/ebde/e07d/d5da/c593/e26a` | PCIe-tunnel pwr-on / 20G-rate / RX-PLL-reset / CDR-margin validate / lane-ramp engine. SHARED PHY infra reachable only via the GPU lane-bond path here. | SHARED | ✅ ported-byte-true |
| `u4lb_ec51` (usb4_lanebond.h:547) / `u4lb_ee57` (usb4_lanebond.h:560) | `CODE_BANK1::ec51`/`ee57` | Lane-train Trig-arm (CCE0-3) + ee57 throttle: fire ec51 when CCE1.0 clear / CCE1.1 set, return CCE4:CCE5. | GPU-PATH | ✅ ported-byte-true |
| `u4lb_walk_8000` (usb4_lanebond.h:1150) + `u4lb_lp1_finalize`/`u4lb_lp1_width_settle`/`u4lb_lane_gate`/`u4lb_e461`/`u4lb_eda0`/`u4lb_ee6e`/`u4lb_8501`/`u4lb_ea7c`/`u4lb_8992` | `CODE_BANK1::8000` (+ 81d4/8174/e461/eda0/ee6e/8501/ea7c/8992) | **STATE 5 walker A** (0x0718==4, live AMD path). LOOP1 @0759/075A (0x10→…→A0/A1) + LOOP2 @075B/075C CL-walk (0x10→20→30→50→60). e461 push at the 0x20/0x60/0xA0/0x50 wait states; drives SB[0xA0/A1] 0x07→0x02 bond. (Ghidra symbol-port names 8000 `u4lb_lane_gate`.) | GPU-PATH | ⚠️ partial |
| `u4lb_walk_850b` (usb4_lanebond.h:1339) | `CODE_BANK1::850b` | STATE 5 walker B (0x0718!=4). **Dead on the live AMD path** (0x0718 always==4 there). Its 0x30 reads `lb_cl_status@0x0B26` not 0x0779. | GPU-PATH | ✅ ported-byte-true |
| `sb_cdf5_routerop_response` (sb_router.h:523) | `CODE_BANK1::cdf5` | cb10-tail deferred router-op CONFIG-READ response (hdr1/hdr3 + 0x40 body, TX via SB[0x06]=1). Gated by 0x072A arm + eda0 selector. The wire event the host blocks on for CL0. | GPU-PATH | ✅ ported-byte-true |
| `u4lb_eb62` (usb4_lanebond.h:14) | `CODE_BANK1::eb62` | FSM-state setter: print "[SB P0n]", store 0x06ED + mirror 0x0AAD. | GPU-PATH | ✅ ported-byte-true |
| `u4lb_98ec` (usb4_lanebond.h:567) | `CODE_BANK1::98ec` | arm 0x0758=0x10 + ee57 + store R6:R7→0x768:0x769 (db7a/edd9 call as 98ec(0,3)). | GPU-PATH | ✅ ported-byte-true |
| `u4lb_s5_diag` (usb4_lanebond.h:1066) + `[edf5]`/`[cr ..]`/`[b4:..]`/`[Lt77A..]` budgets | absent | On-change UART diagnostics for the bond window (deliberately lean). | HOUSEKEEPING | ➕ handmade-only |
| absent | `CODE_BANK1::d4cd`,`cd3f`,`af38`,`eaac`,`ebb5` (SB-EVENT path) | These five SB-transport/descriptor fns belong to the **SB-EVENT chain** (`4486→061a→a066→d4cd→cd3f→{eaac/af38/ebb5}`), not FSM-TICK — mapped in §2 ISR / §5. Listed here only as cross-ref; they ARE present in handmade (sb_router.h). | GPU-PATH | ✅ ported (other section) |

### State machines

| Name | State var | States / transitions | Class | Status |
|---|---|---|---|---|
| U4LB top FSM | `0x06ED` u4_fsm_state | 0=IDLE; 3=CONN_ROUT→a7de; 4=LANE_TRAIN→b0b4; 5=LANE_BOND→8000\|850b. 3→4→5→0. (1/2 absent.) | GPU-PATH | ✅ |
| CONN-ROUT inner | `0x0758` cm_conn_routing_substate | 0x10 ARM_ROUTE_QUERY(edf5); 0x11 AWAIT_RESULT(gate 0x0777==0x0C, set 0x0718=4); 0x00 PRINT_STATUS→state4. | GPU-PATH | ✅ |
| State-5 LOOP1 | `0x0759`(L0)/`0x075A`(L1) lb_loop1_state | 00→10→20→30→40→50→60→70→80→90→A0→A1; host-retrain (8327 snap&0xC0==0x80) re-enters 0x50. | GPU-PATH | ⚠️ |
| State-5 LOOP2 (CL-walk) | `0x075B`(L0)/`0x075C`(L1) lb_loop2_state | 00→10→20→30(reads 0x0779, prime bond decision)→50→60. Drives SB[0xA0/A1] 0x07→0x02. | GPU-PATH | ⚠️ |
| State-5 routing FSM (850b) | `0x0718` + reused 0x075B | dead on live AMD path. | GPU-PATH | ✅ |

### diff note

The FSM-TICK skeleton is **fully ported and byte-true at the dispatch/throttle level**: e672 matches stock's dispatch ladder (confirmed via Ghidra `CODE_BANK1::e672`, identical 4-cell all-lanes-clear guard); the ee57 CCE4:CCE5 delta-≥3 throttle that stock keeps inside cb10 is hoisted into main.c:646-658 but is arithmetically byte-true. State-3 e391 width-LUT seed is now PORTED with byte-exact ROM tables (`u4lb_width_lut_514c` idx 0x0C=0x03, idx 0x0D=0x04) under the `0x0776==0` gate, closing the GAP-1 `af38 SBTX[1]=0x55` divergence — provided the 0x0776-clear path (usb4_lanebond.h:250-258) actually fires on the live connect (0x07B9/0x081B.0/0x07CE/0x07CD inputs). **DUAL-SEED RECONCILIATION (critic):** this connect-gated copy is belt-and-suspenders — `sb_width_lut` is ALSO seeded **boot-unconditionally** in `sb_rom_descriptor_load` (sb.h:146, the canonical copy, §4B), byte-identical tables, so `af38 SBTX[1]=0x03` holds REGARDLESS of whether this gate fires. **GAP-1's width-seed half is therefore closed in this tree independent of the 0x0776 gate**; the residual is the lane-bond CL-terminal, not the seed. The cap20g question is **resolved correctly in code**: main.c:498 sets `0x09F6=1` (the stock bank0_8d77 default), so `081A` keeps bit1 and cm_conn_routing_setup (usb4_lanebond.h:290) ORs lane1 into `0x0819`, reaching **0x0819=0x03 (both lanes)** when the host advertises lane1 (077A.1). **HOWEVER the in-tree comment block at usb4_lanebond.h:288-289 still claims `cap20g_gate1=0 → 081A=0xE1 → 0x0819=0x01 matching stock`, which directly CONTRADICTS both the live code (gate1=1) and the HW ground truth (MEMORY: stock advertises 0x03)** — a stale, misleading comment that should be corrected. The walkers (8000/850b) are present and byte-mapped, but the live bond is still not closing per the project memory (host withholds SB[A0]=0x02): the residual divergence is upstream of these FSMs — the **stubbed c0a5 router-op mailbox (§5) and the missing c5ff-style route de-alias**, plus the documented primary-lane orientation commit (cm_RXCM cc86: C2C3.0/C343.0 + phy_lane_gate@0x0AB3) that the 8000 walker's 81d4 finalize depends on (usb4_lanebond.h:1102-1104) — hence LOOP1/LOOP2 carry a ⚠️ partial despite byte-true transcription.
> **Part 4B — SB transport (SB-EVENT path: a066→d4cd→cd3f→{eaac/af38/ebb5})**
The SB-EVENT INT1 chain (stock `4486`@`44d3`→`061a`→`a066`→`d4cd`→`cd3f`→{`eaac`/`af38`/`ebb5`}) is **fully present** in handmade `sb_router.h`, driven from `sb_router_event_handler` (a066). The stock GAP-1 e391 width-LUT seeder is **restructured**: instead of a connect-time, `0x0776==0`-gated copy (stock a7de→e391), the handmade seeds `sb_width_lut`@0x06F2 + `sb_branchA_gate`@0x0705 **unconditionally at boot** inside `sb_rom_descriptor_load` (sb.h:145-148, byte-identical to ROM `0x514c`/`0x515f`). So af38 SBTX[1] now reads the correct width (idx 0x0C→0x03), not the live-path `0x55` poison the MEMORY note describes — that half of GAP-1 is structurally fixed in this tree. The residual GAP-1 symptom (SB[0x0C] non-advance / 2nd af38 not emitting `0104 6324`, terminal route-id `077A`=0x3C vs stock 0x3D) is a downstream lane-bond / CL-walk issue, not the width seed.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `sb_router_event_handler` (sb_router.h:558) | `CODE_BANK1::a066` | INT1 C80A.5 service body: per-channel connect poll (PART 1) + connect/disconnect/CL0/Lane-Bonded/Pend-Int edge servicing (PART 2, all W1C). Calls d4cd + a5d8. | GPU-PATH | ✅ |
| `sb_d4cd_transport_edges` (sb_router.h:216) | `CODE_BANK1::d4cd` | SB-transport edge poller: SB[0x28/0x2A]→transport ports 0/1 (plane 0x2a00/0x2b00), SB[0x81/0x83]→link ports 2/3 (0x2c00/0x2d00); sets `sb_active_plane_port`@0x06F0 **before** each cd3f so eaac/af38 read the matching plane; W1C-acks + ping-pong toggle. | GPU-PATH | ✅ |
| `sb_transport_substate_poll` (sb_router.h:247) | `CODE_BANK1::d4cd` (substate-poll tail) | Stock's SB[0x28/0x2A/0x81/0x83] bit.3 edge/plane poll. Handmade collapses it into d4cd and leaves this a no-op stub. | GPU-PATH | 🔶 |
| `sb_cd3f_dispatch` (sb_router.h:182) | `CODE_BANK1::cd3f` | SB descriptor-event ROUTER: edd9 RX-ack first; reads desc4e (cmd) + desc752 (status@0x0752); gates on present/0x10/0xC0/0x04/port; dispatch `(status&0x60)==0x60`→ebb5, `status.0==0`→eaac, else→af38. | GPU-PATH | ✅ |
| `sb_eaac_populate_0777` (sb_router.h:40) | `CODE_BANK1::eaac` | Copy 0x40 bytes of host connect descriptor from RX plane (sb_rxplane_212d[port]) → `u4_host_desc`@0x0777; sets route-query-response@0x0775; W1C XFER2 DMA. Plane selected via 0x06F0. (extra `[eaac]` diag, budgeted.) | GPU-PATH | ✅ |
| `sb_af38_descriptor_response` (sb_router.h:72) | `CODE_BANK1::af38` | SB-transport descriptor RESPONSE engine: builds TX in 0x2900 plane, `[0]=desc_type`, `[1]=desc_dir \| sb_width_lut[desc_type]` (=0x03 for route idx 0x0C); BRANCH-A body copy gated on width/branchA_gate; writes SB[0x0C]/SB[0x15]; tail d5da. | GPU-PATH | ⚠️ |
| `sb_set_connect_present_ebb5` (sb_router.h:159) | `CODE_BANK1::ebb5` | cd3f `(status&0x60)==0x60` arm: if descriptor nibble!=0 set SB[0x57].3 + SB[0x61].3; set `sb_connect_present`@0x0765. | GPU-PATH | ✅ |
| `sb_edd9_receive_ack` (sb_router.h:168) | `CODE_BANK1::edd9` | cd3f's first action each edge: P1[0x0109].0 RX-ack → SB[0xD8]=2, E716 link-status latch, eb62/98ec set FSM to CONN_ROUT. | GPU-PATH | ✅ |
| `sb_channel_connect_service` (sb_router.h:332) | `CODE_BANK1::c3b2` | a066 PART-1 per-active-port (0x06F1) connect-descriptor read SB[0x20..]/[0xA4..], (~hi)==lo validate, dispatch on (lo&0x0F): 1/5 route-up, 3 link-reinit+d5da, 0 CL0 handling. | GPU-PATH | ✅ |
| `sb_con_consequence` (sb_router.h:276) | `CODE_BANK1::dea1` | a066 connect-edge consequence (gated 0x06EC): SB/PHY arm once, lane-train arm, db7a, defer 8a89. | GPU-PATH | ✅ |
| `sb_db7a_route_arm` (sb_router.h:255) | `CODE_BANK1::db7a` | post-connect tunnel-route arm; branch on 0x07B9 (Connect_U4 vs EnterMode-TBT); CA60/PHY-link; sets FSM CONN_ROUT. | GPU-PATH | ✅ |
| `sb_lane_bonded_consequence` (sb_router.h:313) | `CODE_BANK1::eed6` | a066 SB[0x66].0 Lane-Bonded edge: lb_lane_bonded_flag=1, SB[0xC9]=0xFF W1C, re-latch port-rr, u4lb_c593 post-bond tunnel-PHY commit. | GPU-PATH | ✅ |
| `sb_lane_bond_complete_tunnel_up` (sb_router.h:322) | `CODE_BANK1::e52d` | a066 P1[0x0109].0 tunnel-complete: sb_rom_descriptor_load re-seed, defer downstream PCIe bring-up (sb_tunnel_up_pending), CA60 clear. | GPU-PATH | ✅ |
| `sb_a5d8_pend_int` (sb_router.h:415) | `CODE_BANK1::a5d8` | a066 SB[0x26].1 `[Pend Int]` device→host router-op responder: parse routerop hdr0-3, opcode 0/1/2, READ/WRITE body via sb_width_lut/branchA_gate, e1cb/e2b9 TX push. | GPU-PATH | ✅ |
| `sb_a5d8_tx` (sb_router.h:397) | `CODE_BANK1::e1cb`/`e2b9` | router-op TX answer push: LCALL d4cd FIRST (re-sync host_desc), build 0x2900 TX, SB[0x0C]/[0x15], d5da, XFER2 strobe, set e461 token. | GPU-PATH | ✅ |
| `sb_cdf5_routerop_response` (sb_router.h:523) | `CODE_BANK1::cdf5` | DEFERRED router-op config-read response (cb10 tail, `0x072A!=0`): build lane-config hdr (0x0998-0x099B) + body, SB[0x06]=1 TX. The wire event the host blocks on. | GPU-PATH | ✅ |
| `sb_cb10_lane_advance` (sb_router.h:702) | `CODE_BANK1::cb10` (SB tail) | per-super-loop SB[0xA0]/[0xA1] nibble vs 0x072B/072C latch advance (no-op on a stalled host). | GPU-PATH | ✅ |
| `sb_block_init` (sb.h:180) | `CODE:bb37`/`bb80` (SB init) | SB-block RMW keystone + PHY descriptor seed + ROM tables (calls sb_rom_descriptor_load) + PHY-reg config + KB readback diag. | SHARED | ✅ |
| `sb_rom_descriptor_load` (sb.h:138) | `CODE:` DROM loader + `CODE_BANK1::e391` (width-LUT half) | Copies DROM identity (0x213d→0x0800), lane_descriptor, **and seeds sb_width_lut@0x06F2 / sb_branchA_gate@0x0705 from ROM 0x514c/0x515f unconditionally** — the handmade home of the GAP-1 e391 seed. drom_identity[0x1A]=0xD3 (lane1 advertise seed). | GPU-PATH | ✅ |
| `sb_lane_flip_init` (sb.h:52) | `CODE:` lane-flip init | orientation/connect lane map + SB-PHY-RX descriptor + cap-field seed + mailbox strobe; runs before sb_block_init. | SHARED | ✅ |
| `sb_assert` (sb.h:405) | `CODE:` sb_assert entry | SB-assert entry: edbd/e5b0/dd42/bcd7-tail/ccb3/c270/d556 then lane_flip+block init; post-SB E302 + full SB[0x00..0xFF] dump diag. | SHARED | ✅ |
| `u4rx_tab` (usb4_rx_table.h:8) | `CODE:` SB-PHY RX-lane arm ROM table | 324-row {page,reg,AND,OR} SB-PHY RX-lane RMW arm table (consumed by the connect/PHY bring-up engine). | SHARED | ✅ |
| `bank0_8a89` (usb4_connect.h:38) | `CODE:8a89` | USB4 lane-MODE bring-up engine (deferred off ISR, run once post-connect); not strictly SB-EVENT but the connect consequence that feeds the FSM. | SHARED | ✅ |
| `bank0_c9a8` (usb4_connect.h:193) | `CODE:c9a8` | host-link-event connect dispatcher gating bank0_8a89 on the connect gate. | GPU-PATH | ✅ |
| _(none)_ | `CODE_BANK1::e391` (as a standalone fn) | stock standalone width-LUT seeder gated `0x0776==0` at a7de call-site. Handmade has **no separate e391 fn**; its work is folded into sb_rom_descriptor_load (boot-unconditional). The stock loop2 (zero XDATA[0x0B26+i] deep-PHY) is NOT ported. | GPU-PATH | ➕ (folded) / partial-omit loop2 |

### State machines

| Name | State var | States / transitions | Class | Status |
|---|---|---|---|---|
| SB-transport edge/plane | `sb_active_plane_port`@0x06F0 (+0x06EE/06EF) | d4cd sets port 0/1 (transport, plane 0x2a00/0x2b00) or 2/3 (link, 0x2c00/0x2d00) before each cd3f; round-robin connect channel = `sb_active_port_rr`@0x06F1. | GPU-PATH | ✅ ported (0x06F0 tracking present); ⚠️ handmade plane table is `sb_rxplane_212d[]={2a00,2b00,2c00,2d00}` vs stock af38 hard-codes 0x2280D/0x2280E for the port-0/1 status read — handmade reads SB(0x0D/0x0E) directly (status_off), semantically equiv. |
| cd3f dispatch gate | `sb_connect_descriptor`@0x0752 (status) + desc4e (cmd@0x4e) | present & (status&0x60)==0x60 → return; !(cmd&0x10) → return; (status&0xC0)!=0x40 path checks status.2/.4 by port; then 0x60→ebb5 / status.0==0→eaac / else→af38. | GPU-PATH | ✅ |

> **diff note:** The SB-EVENT path is structurally complete and largely byte-true. The one GPU-load-bearing residual is af38 (⚠️): even though `sb_width_lut`@0x06F2 is now correctly boot-seeded (so SBTX[1] for the route descriptor = 0x03, NOT the historical 0x55 poison — GAP-1's seed half is fixed in this tree), the broader symptom from the ground-truth captures (the 2nd af38 not emitting `0104 6324`, SB[0x0C] failing to advance 08→0B→0C, terminal route-id `077A`=0x3C vs stock 0x3D) is a downstream lane-bond/CL-walk divergence (§4 FSM-TICK / §5 territory), not a width-seed defect. Secondary nits: `sb_transport_substate_poll` is a 🔶 no-op (its work absorbed into d4cd — verify the SB[0x28/0x2A/0x81/0x83] bit.3 substate poll isn't lost), and stock e391's loop2 (zeroing the 0x0B26+i deep-PHY shadow) is not reproduced.

---
## 5. CM-Tunnel / Router-ops

The host CM drives the GPU tunnel through the **router-op mailbox** (`EC06.0 → c0a5`). The single GPU-critical op is **`0xE8` PcieTunnel-Deassert/Enable** (`c15f → e4a6`) which HW-resets then deasserts B480 PERST bits0-3, bringing the downstream GPU out of reset. In handmade this entire dispatch + config-R/W + tunnel-reset engine is **MISSING** — `cm_routerop_mailbox` (usb4.h:63) is a STUB that only state-machines and ACKs (`EA90<-0xA5`), never dispatching E2/E3/E8. Separately, the whole 9037/c00d/c1f9/adb0 CM step-machine + config-TLP engine is present in cm_tunnel.h but **DEAD** (no caller); the live downstream-link path is the custom `pcie_power_on()` (main.c, mapped in §7). The wall: host posts `EC06=0` (never `0xE8`) because the adapter is never advertised/enabled and the mailbox never replies a config read/write.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `cm_routerop_mailbox` (usb4.h:63) | `CODE_BANK1::c0a5` cm_routerop_mailbox | Router-op dispatcher. STUB: gates `EA90==0x5A`, latches `EA80→opcode`, runs only the 0/1/2 state arms + ACKs `EA90<-0xA5`. Does NOT run the `0def`/`c0c5` movc jump-table, NO E0-E8 dispatch, NO `cf35` reply / `C8B0<-0xEA` DMA push. | GPU-PATH | 🔶 |
| absent | `CODE:0def` c51_switch_dispatch | Keil C51 movc switch helper for the c0a5 jump-table. | SHARED | ❌ |
| absent | `CODE_BANK1::c0ef` routerop_op_E2_cfgread | E2 = router CONFIG-SPACE READ (cf4c gate, ceef addr, d945 send-read-resp, set `0x0B02=1`). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::c119` routerop_op_E3_cfgwrite | E3 = router CONFIG/PATH WRITE (stage 0x0B08/09, cf5d send-write-resp, set `0x0B02=2`). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::c15f` routerop_op_E8_tunnelreset | **E8 = PCIe-TUNNEL RESET/ENABLE** — `LCALL e4a6`, the host's `[PcieTunnel-Deassert]/[Enable]` that brings the GPU out of reset. (Jump-table arm inside c0a5.) | GPU-PATH | ❌ |
| absent | `CODE_BANK1::e4a6` tunnel_routerop_link_reset | PCIe-tunnel link RESET+re-enable: `C656&=~0x20`,`CA06&=~1`,`CC31.0=1` assert+spin → eec7 PERST-deassert → `B480&=~0x0F`. | GPU-PATH | ❌ |
| absent | `CODE_BANK1::eec7` bank1_eec7 | PERST-DEASSERT helper: e8d9 (`C659.0`) + `[PcieTunnel-Deassert]`. | GPU-PATH | ❌ |
| absent | `CODE_BANK1::e4ea` tunnel_routerop_enable_tail | Post-reset Enable tail: power-on + B480 PERST bits0-3 deassert + `[PcieTunnel-Enable]`. | GPU-PATH | ❌ |
| absent | `CODE_BANK1::d945` cm_routerop_send_read_resp | Build+arm router cfg READ response (bounds, hdr status/code=3, DMA `C8B0=0xEA`). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::cf5d` cm_routerop_send_write_resp | Build+arm router cfg WRITE response (stream write data, advance 0x0B08). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::ceab` cm_routerop_addr_in_bounds | 64-bit addr `0x0B04` vs limit `0x0B0A` continuation/completion compare. | GPU-PATH | ❌ |
| absent | `CODE_BANK1::ceef` cm_routerop_copy_addr_from_mbox | Copy 64-bit addr/payload EAxx→`0x0B04`/`0x0B0A`. | GPU-PATH | ❌ |
| absent | `CODE_BANK1::cf4c` cm_routerop_is_read_opcode | Test EA81 sub-opcode read(0x50)/write(0x51). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::cf35` cm_routerop_reply_trigger | `C805=(C805&0xF9)\|0x02` send-response bit (handmade only writes `EA90=0xA5`, never C805). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::e21b` routerop_op_E5_cfgop | E5 single-reg config read/write (EA82 0x50/0x51). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::d6dc` routerop_op_E4_blockcfg | E4 router cfg BLOCK read/write loop (EA81=len). | GPU-PATH | ❌ |
| absent | `CODE_BANK1::e2b9` sb_issue_transport_cmd | routerop connect-fill helper (stage 0xAA8/9/A, poll d4cd, stream 96f7→d5da). | SHARED | ❌ |
| `usb4_routerop_init` (usb4_irq.h:253) | (router-op engine init — EC00/EA88/EA89/EC05) | Router-op mailbox engine bring-up: EC00 reset+enable, speed EA88=100/EA89=0x24, EC05/INT-DMA. LIVE (main.c:557). | GPU-PATH | ✅ |
| `cm_adb0_tlp` (cm_tunnel.h:174) | `CODE:adb0` cm_adb0_tlp | GENERAL config-TLP issuer (CfgRd/CfgWr FMT 04/05/44/45 by mailbox `0x0A70-75`, result B220-3, 9a60 cplcode). Ported byte-true but **uninvoked** (only DEAD callers). | SHARED | ⚠️ |
| `cm_c1f9` (cm_tunnel.h:135) | `CODE:c1f9` cm_c1f9 | Predecessor cfg/ECAM-MemRd engine (dir 0x05AE, addr 0x05AF-B2, 0x00D000 aperture). Ported; only DEBUG caller. | SHARED | ⚠️ |
| `cm_e89d_read` (cm_tunnel.h:213) | `CODE:e89d` (read+adb0) | Config READ wrapper (m0=0,m5=0xF→adb0). DEAD. | SHARED | ⚠️ |
| `cm_e91d_write` (cm_tunnel.h:215) | `CODE:e91d` route-bind CfgWr primitive | Config WRITE wrapper (m0=1→adb0). Stock e91d = SHARED route-bind primitive. DEAD. | SHARED | ⚠️ |
| `cm_cfg_addr` (cm_tunnel.h:129) | `CODE:e762` | Set cfg addr 0x00D000{token}→0x05AF-B2 BE. DEAD (debug-only path). | SHARED | ⚠️ |
| `cm_mbox_*`/`cm_9a60_cplcode` (cm_tunnel.h:121-170) | `9a53/999d/99eb/9a95/9a74/9a60` | adb0/c1f9 mailbox sub-helpers (stage-clear, kick, busy, ack, status). Ported; only DEAD callers. | SHARED | ⚠️ |
| `cm_arm_c00d` (cm_tunnel.h:95) | `CODE:c00d` cm_arm_c00d | CM downstream-link ARM (B401 pulse, cd6c, B480 PERST assert, d436, 39e4, `0x05B4=0x10`). Present but **DEAD** — removed from the live path 2026-06-18 (PERST/C659/d436 re-drive raced pcie_power_on's LTSSM). | SHARED | 🔶 |
| `cm_pcie_link_init_state` (cm_tunnel.h:75) | `CODE:39e4` cm_pcie_link_init_state | DMA/descriptor-RAM + page-04 reset. Scalar resets only (DMA-engine fills TODO-gated); DEAD (only c00d calls it). | SHARED | ⚠️ |
| `cm_pcie_link_step_machine` (cm_tunnel.h:283) | `CODE:9037` cm_pcie_link_step_machine | CM LTSSM step machine (phases 1-4, `0x05B4==0x10` core, e2a6 link-up). Present but **DEAD** (no caller); link-up path stubbed to a print. | SHARED | 🔶 |
| `cm_e6fc_restart`/`cm_e8e4_settle`/`cm_tunnel_link_bringup_start`/`cm_hddpc_phy_logic`/`cm_e8a9_c659`/`cm_e8d9_c659`/`cm_link_up_check`/`cm_timer4_csr_fired` (cm_tunnel.h:25-69) | `e6fc/e8e4/92bb/e5cb/e8a9/e8d9/e2a6/e30e` | 9037-machine callees (restart, settle, bringup, PHY-logic, C659, link-up, timer4). Ported byte-true; all reachable only via the DEAD c00d/9037. | SHARED | ⚠️ |
| `cm_topo_walk` (cm_tunnel.h:225) | (none — debug) | DEBUG: bus0/bus1 cfg-space enumerator over adb0 (prints vid/did/class/hdr). Not a stock fn. DEAD. | HOUSEKEEPING | ➕ |
| `cm_route_test` (cm_tunnel.h:253) | `CODE:c5ff` route de-alias (partial) | DEBUG: replicates **3 of c5ff's 4** route CfgWr (reg6 bus-num, reg1 cmd, reg8 mem-win) hardcoded bus0/dev0 + ECAM MemRd probe. NOT a live one-shot; the c5ff de-alias is NOT ported into the GPU path. DEAD. | SHARED? | ⚠️ |
| `cm_dbg_cfgrd` (cm_tunnel.h:218) | (none — debug) | DEBUG helper: one CfgRd of {bus,dev0,fn0,reg}. DEAD. | HOUSEKEEPING | ➕ |
| absent | `CODE:c5ff` route de-alias / bridge bus-num | **4× CfgWr-over-tunnel de-alias** (reg6 bus-num d956, reg1 cmd, reg8 mem-win 9a7f, 4th cap-derived 9ee5) + per-port `0x05B4=2` marker. NOT lifted into a live GPU-path one-shot via adb0 (only the DEBUG cm_route_test exists). | SHARED (lift) | ❌ |
| absent | `CODE:a183` NVMe class-01:08:02 enroll | Per-port NVMe device enroll (CfgRd class → per-port table). Intentionally not ported. | NVMe-ONLY | 🚫 |
| absent | `CODE:e91d`/`d956`/`9ee5` route-bind primitives | SHARED route-bind CfgWr primitives (bus-num/cap-walk). e91d has a wrapper (cm_e91d_write, DEAD); d956/9ee5 not separately ported. | SHARED | ❌ |
| `usb4_connect_u4` (usb4.h:23) | `CODE_BANK1::a17c`-region usb4_connect_u4 | Post-Enter_USB connect path: drives sideband bring-up (E716/CA81/CA06 mode, e7c1/e0d9, route_mode/lane_gate) → sb_assert. LIVE (vdm.h:192, usb4_connect.h:179). | GPU-PATH | ✅ |
| `usb4_int_demux` (usb4.h:91) | INT1 USB4 event demux (C80A sources) | LIVE (main.c:471). Routes SB(0x20)/evt(0x10)/routerop(EC06=NVME_EVENT.0)/tunnel(0x0F) INT sources; on routerop calls the STUB cm_routerop_mailbox. | GPU-PATH | ⚠️ |
| `cm_pcie_link_init_state`/`cm_arm_c00d` fwd-decls (cm_tunnel.h:21-22) | — | forward decls for mutual recursion. | HOUSEKEEPING | ➕ |
| `pcie_power_on` (main.c:91) | replaces stock `c00d`/`3578` live path | Custom live downstream-LTSSM bring-up (mapped §7). REPLACES the DEAD c00d/9037. | (see §7) | ➕ |

### State machines

| Name | State var | States | Class | Status |
|---|---|---|---|---|
| router-op mailbox | `u4_routerop_mbox_state` (handmade) / `0x0B02` (stock) | handmade: IDLE→latch opcode; MULTIPKT_1 only ACKs if opcode==E2; MULTIPKT_2 only ACKs if opcode==E3; else→IDLE. **NO E2/E3 actual cfg read/write, NO E8.** stock: 0=IDLE (c0c5 dispatch), 1=E2 read-cont, 2=E3 write-cont. | GPU-PATH | 🔶 |
| router-op working buffer | `0x0B04`(addr)/`0x0B0A`(limit) (stock) | stock: addr from EA82+ (ceef), len=min(limit-addr,7), 0x0B08/09 write cursor. **Absent in handmade** (no ceef/ceab/d945/cf5d). | GPU-PATH | ❌ |
| CM downstream-link step (9037) | `0x05B4`+`0x0B39`+`0x0B3A/B` | Ported in cm_pcie_link_step_machine but **DEAD** (no caller); link-up arm prints `[CMlinkUP]` instead of running the per-port walk. | SHARED | 🔶 |
| CM arm one-shot (c00d) | `0x06E6` | Ported in cm_arm_c00d but **DEAD/uninvoked** (removed from live path 2026-06-18 — raced pcie_power_on). | SHARED | 🔶 |

### diff note

Apply RESOLVED BOUNDARY: the 9037/c00d CM step-machine + PERST arm is NVMe-ONLY and HARMFUL on the GPU path; it is correctly **DEAD/uninvoked** in handmade (cm_arm_c00d / cm_pcie_link_step_machine / cm_pcie_link_init_state / 39e4 and their callees have no caller — `pcie_power_on()` is the live replacement). So those are 🔶/⚠️ "present-but-dead," not the bug.

The **GPU wall is the router-op mailbox**. Stock `c0a5` runs a movc jump-table (`0def`/`c0c5`) dispatching E0-E8; the GPU-critical arm `c15f` (`E8`) calls `e4a6→eec7` to deassert B480 PERST and bring the downstream GPU out of reset; `c0ef`(E2)/`c119`(E3) do the config read/write that lets the host probe the tunneled device. Handmade `cm_routerop_mailbox` (usb4.h:63) is a 🔶 STUB: it only state-machines `u4_routerop_mbox_state` and writes `EA90=0xA5` (ACK) — it never runs `0def`, never dispatches any opcode, never builds a response (`cf35`/`C805`/`C8B0`), and the whole reply/cfg-R-W/tunnel-reset engine (`c0ef/c119/c15f/e4a6/eec7/e4ea/d945/cf5d/ceab/ceef/cf4c/cf35/e21b/d6dc`) is ❌ MISSING. Consequence (memory `EC06 ea90` diag, usb4.h:101): the host never posts a non-zero router-op (`EC06=0`), so even if E8 were wired it would not fire.

Necessary-but-not-sufficient companion gap: the **c5ff route de-alias is not lifted into a live GPU-path one-shot**. Only the DEBUG `cm_route_test` (cm_tunnel.h:253) exists, which replicates 3 of c5ff's 4 CfgWr with hardcoded bus0/dev0 and is uninvoked. Per the boundary, the correct design is to drive c5ff's 4× CfgWr de-alias through the (already byte-true, but DEAD) `cm_adb0_tlp` engine as a PERST-free one-shot — that engine is ready (⚠️ ported, just not driven post-bond). NVMe `a183` enroll is correctly 🚫 omitted.

---
## 6. PD / VDM

Entire subsystem is **PD**. On the GPU path *indirectly*: device must PROMPT the host (Hard Reset), reply to Source_Cap with a Request RDO, ACK Discover_Identity (VID `0x174C`) / SVIDs (`0x8087`) / Modes / Enter_Mode, then bridge into the USB4 SB/lane subsystem. PD is reported WORKING on the wire (contract + VDM + Enter_USB all complete; host reaches Enter_USB Accept). The PD→USB4 bridge in handmade is the **Enter_USB Data Message** path (`pd_handle_enter_usb`→`usb4_connect_u4`), NOT the stock VDM-cmd5 path — see diff note.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `pd_rx_message_dispatch` (pd_dispatch.h:303) | `CODE:83d6` pd_dispatch_data | PD RX entry: parse hdr at `0xE440+0x20*slot`, demux CONTROL (NumObj==0) vs DATA. | PD | ✅ |
| `pd_dispatch_data` (pd_dispatch.h:257) | `CODE:83d6`/`8406` | DATA demux: 1=Source_Cap, 3=ack, 2..7=NAK, 8=Enter_USB, 0xF=VDM, else NAK. | PD | ✅ |
| `pd_dispatch_control` (pd_dispatch.h:242) | `CODE:8406` pd_dispatch_control | CONTROL demux: 1=GoodCRC,3=Accept,4=Reject,6=PS_RDY,0xC=Wait,0xD=Soft_Reset. | PD | ✅ |
| `pd_select_pdo_from_source_cap` (pd_dispatch.h:69) | `CODE:abf5` pd_select_pdo_from_source_cap | Select PDO[0] → op-current `0x7DA/0x7DB`. `[Source_Cap]`. | PD | ✅ |
| `pd_build_send_request_rdo` (pd_dispatch.h:78) | `CODE:acd4` pd_build_send_request_rdo | Build Request(RDO)→E420-E425, substate=3, strobe CC TX, arm SenderResp. | PD | ✅ |
| `pd_handle_enter_usb` (vdm.h:146) | `CODE:a036` pd_handle_enter_usb | Enter_USB (PD3.1): parse EUDO mode@vdo3, mode==2&role0&cable_cur→Accept, accepted=1, `gate_e8=1`, `[Enter_USB 4]`→`[Connect_U4]`→`usb4_connect_u4`. **THE live PD→USB4 bridge.** | PD | ✅ |
| `vdm_tx_dispatch` (vdm.h:211) | `CODE:9ac4` vdm_tx_strobe_commit | VDM dispatcher: cmd 1=Disc_ID,2=SVIDs,3=Modes,4=Enter_Mode; **default→NAK (cmd 5/6 not handled).** | PD | ⚠️ |
| `vdm_tx_strobe_commit` (vdm.h:198) | `CODE:9ac4` (tail, TX strobe portion) | Drive cmd-IF opcode 3 PD-TX via DMA page `0x50` (= stock **CC88** mailbox), commit. | PD | ✅ |
| `vdm_build_discover_id` (vdm.h:33) | `CODE:aa36` vdm_build_discover_id | Disc_ID ACK: **VID 0x174C** (LBA0=0x4C,LBA1=0x17), SOP' TBT gen/mode VDOs. | PD | ✅ |
| `vdm_build_discover_sids` (vdm.h:70) | `CODE:ddad` vdm_build_discover_sids | Disc_SVIDs ACK: **TBT SVID 0x8087**, else NAK. | PD | ✅ |
| `vdm_build_discover_modes` (vdm.h:85) | `CODE:d852` vdm_build_discover_modes | Disc_Modes ACK: TBT3 mode VDO for SVID 0x8087, else NAK. | PD | ✅ |
| `vdm_handle_enter_mode` (vdm.h:117) | `CODE:b966` vdm_handle_enter_mode | Enter_Mode ACK (SVID 0x8087,objpos,role0): latch `route_latch=1`,`pending=1`,`[Enter_TBT]`. | PD | ✅ |
| `usb4_mode_entry_commit` (vdm.h:102) | `CODE:0102` usb4_mode_entry_commit | USB4 mode-entry latch: write `92E1=0x10`, mask `9090.7`, return 4(USB4)/1(USB3). | PD | ✅ |
| `vdm_nak` (vdm.h:24) | `CODE:dd0e` vdm_nak | VDM NAK builder (echo SVID, cmdtype 2). | PD | ✅ |
| `pd_drive_hard_reset` (pd.h:120) | `CODE:be8b` pd_drive_hard_reset | TX Hard-Reset to force Source_Cap re-send; NO-OP when E302 link-mode==3 (`[CCOpen_neednt_HardRst]`). Host-prompt; called from superloop `!pd_seen`. | PD | ✅ |
| `cc_pd_timer_tick` (pd.h:235) | `CODE:b4ba` cc_pd_timer_tick | INT1 policy tick over 6 CC event regs; CC91.1 1s timeout→`route_mode=4`+`usb4_mode_entry_commit`. | PD | ✅ |
| `cc_pd_phy_term_init` (pd.h:42) | `CODE:ae87` cc_pd_phy_term_init | PD PHY/CC term (Rp/Rd), RDO/CRC timing (pd_da51), arm PD engine. | PD | ✅ |
| `pd_internal_state_init` (pd.h:93) | `CODE:b8c3` pd_internal_state_init | Reset PD policy block: substate=1, clear latches, seed timers, enable CC events. `[InternalPD_StateInit]`. | PD | ✅ |
| `pd_rx_isr` (pd.h:166) | (E40F/E410 demux, ISR path) | PD-int handler: W1C-ack E40F/E410, msg-recv→`pd_rx_message_dispatch`. | PD | ✅ |
| `pd_ctrl_accept` (pd_dispatch.h:121) | (within `8406` Accept arm) | Accept: substate3→4, arm PS_RDY timer. `[Accept]`. | PD | ✅ |
| `pd_ctrl_ps_rdy` (pd_dispatch.h:148) | (within `8406` PS_RDY) | PS_RDY: decode contract voltage→`0x07B8`. | PD | ✅ |
| `pd_ctrl_soft_reset` (pd_dispatch.h:223) | (within `8406` Soft_Reset) | Soft_Reset: reset MsgID, reply Accept. | PD | ✅ |
| `pd_tx_commit_engine` (pd_dispatch.h:28) | (TX commit primitive) | Send staged E420-E43F, bump TX MsgID. | PD | ✅ |
| `pd_keystone_init` (pd.h:158) | (init bridge) | Enable INT1, force USB4 mode (`u4_mode_flag=0x87`), PD PHY+state init. | PD | ✅ |
| ❌ Enter_USB4-ACK VDM cmd5 handler | `CODE:ca71` vdm_enter_usb4_ack_sb_init | ENTER_USB4 (VDM cmd5) ACK + SB hand-off (lane_flip_init+sb_block_init); the stock VDM-side PD→USB4 bridge. | PD | ❌ (absent) |
| ❌ VDM cmd6 Exit handler | `CODE:e973` (Exit) | Exit-mode VDM handler (cmd6). | PD | ❌ (absent) |

### State machines

| Name | State var | States / transitions | Class |
|---|---|---|---|
| PD policy-engine substate (`pd_msg_substate`) | XDATA `0x07BD` | 1=INIT (b8c3-ready for Source_Cap); 2=PDO selected; 3=Request sent (await Accept/PS_RDY); 4=Accept→await PS_RDY; 0x0D=Enter_USB pending, 0x0E=Data_Reset. 1→Source_Cap→2→Request→3→Accept→4→PS_RDY→contract. Hard-reset→state-init→1. | PD | ✅ |
| USB4 connect-decision latches | `u4_enter_usb_accepted`/`u4_connect_pending`/`u4_connect_route_latch`/`u4_route_mode`/`u4_mode_flag(0x09F9)` | Enter_USB Accept→accepted=1,route_mode\|=4,gate_e8=1→`usb4_connect_u4`. Enter_Mode ACK→route_latch=1,pending=1. CC91.1 1s→route_mode=4+commit. | PD | ✅ (bridge via Enter_USB, not VDM-cmd5) |

### diff note

The PD/VDM core is ported byte-true and is confirmed working on the wire (contract, Discover_ID/SVIDs/Modes/Enter_Mode, and Enter_USB Accept all transmitted). VDM TX uses DMA page `0x50` (vdm.h:198-205) which is the stock **CC88** mailbox (confirmed against stock 9ac4 `FUN_CODE_9627(...,0xcc88); xfer_write_byte_dma_start(0x50,...)`). VID `0x174C`/SVID `0x8087`/Drive_HardRst E302==3 NO-OP guard all match.

The one real divergence: stock `vdm_tx_strobe_commit@9ac4` dispatches VDM **cmd 5 (Enter_USB4)** → `vdm_enter_usb4_ack_sb_init@ca71` (which ACKs and hands off to the SB/lane subsystem via `sb_lane_flip_init`/`sb_block_init`) and **cmd 6 (Exit)** → `e973`. The handmade `vdm_tx_dispatch` switch (vdm.h:233-253) only covers cmd 1-4 and routes cmd 5/6 to `vdm_nak`. Functionally the handmade design moves the PD→USB4 bridge onto the **Enter_USB Data Message** path (`pd_handle_enter_usb`@vdm.h:146 → `[Connect_U4]` → `usb4_connect_u4`), and the wire trace shows the host completes Enter_USB Accept — so the live bond enters USB4 via that route. Stock additionally calls `sb_lane_flip_init` on VDM cmd4 (Enter_Mode) OR cmd5; the handmade Enter_Mode handler (vdm.h:117) only latches `route_latch`/`pending` and does NOT call `usb4_connect_u4`/`sb_lane_flip_init`, deferring the SB hand-off to the Enter_USB path. If a host ever drives TBT-mode entry via VDM Enter_USB4 (cmd5) rather than PD3.1 Enter_USB, the handmade device would NAK and never bridge — a latent gap, MEDIUM. No other PD handler is missing.

---
## 7. Local PCIe (device→GPU downstream LTSSM)

The local device-side downstream PCIe layer that deasserts PERST and trains the GPU link over the tunnel, plus the host-driven config-TLP/PIO engine that the tinygrad interface uses once the link is up.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `pcie_power_on` (main.c:91) | `CODE:3578` pcie_downstream_link_bringup | Custom downstream bring-up: B431=x2 width, B403=1 stability, B480 PERST assert→deassert cycle, B430 clr, B298 TLP-routing (bank1 6025.7), C656 3.3V / C659 12V rails, then bounded poll of **B450==0x78** + E765.2 for CONNECTED. REPLACES the stock B455-LTSSM/B2D5/B296/B220-cap/d436 sequence wholesale. | GPU-PATH (➕ custom) | ➕ |
| `pcie_power_off` (main.c:82) | (no single stock fn; ≈ inverse of 3578/99e0) | Hold downstream in reset + drop rails: B480 PERST assert, B430=0, E764&=0x10, C659&=~1 (12V off), C656&=~0x20 (3.3V off). | GPU-PATH (➕ custom) | ➕ |
| `cm_link_up_check` (cm_tunnel.h:41) | `CODE:e2a6` pcie_link_up_check_b432_e765 | `(0x07EF==0) && (B432&7)==7 && (E765&2)` link-up criterion. Ported byte-true; lives in §5 CM layer, used by the CM step machine — NOT called by `pcie_power_on` (which instead checks B450==0x78). | GPU-PATH | ✅ |
| `pcie_read_chunk` (pcie_pio.h:32) | (no direct stock fn; ≈ B220/B254/B296 PIO loop, cf. 05a8/0dc5 cap reads) | Streaming PCIe MemRd PIO: trigger via B296=7/B254=0x0F, poll B296&3, read B223..B220, 64-bit auto-inc addr (B21B..B21C carry), pipelined. Backs `do_usb_bulk_in` device→host. | GPU-PATH (➕ custom) | ➕ |
| `pcie_write_chunk` (pcie_pio.h:160) | (no direct stock fn) | Streaming PCIe MemWr PIO: src→B220..B223, B296=7/B254=0x0F trigger, 64-bit auto-inc addr. Host→device bulk path. | GPU-PATH (➕ custom) | ➕ |
| `do_usb_bulk_in` (main.c:129) | (no stock fn; tinygrad-only) | Drains `dma_dwords` from PCIe via `pcie_read_chunk` into 0x8000, sets USB bulk-IN length, arms EP. | GPU-PATH (➕ custom) | ➕ |
| `handle_usb_control` 0xF0 OUT/IN + DATA_OUT TLP engine (main.c:244-335) | (no stock fn; tinygrad-only) | HOST-driven config-TLP engine: host posts fmt/type+BE+64-bit addr (B450-region: B-prefixed FMT/BYTE_EN/ADDR/DATA/STATUS/TRIGGER regs); mode0=single CfgRd/CfgWr/MemRd fired via B296 status + trigger, returns 8-byte completion; mode1/2=streaming bulk. This is the tinygrad MMIO interface to the GPU. | GPU-PATH (➕ custom) | ➕ |
| `pcie_power_on` 0xF3 dispatch (main.c:236-243) | (no stock fn) | Vendor 0xF3 control routes wValue.0 → `pcie_power_on`/`pcie_power_off`. | GPU-PATH (➕ custom) | ➕ |
| `pcie_apply_x2_rxphy_tuning` (pcie_tuning.h:149) + slice helpers (29/37/53/62/93/120) | (boot PHY tune; ≈ stock RX-PHY trace, no single named fn) | Bank1 (DPX=1) RX-PHY tuning for 4 lane slices (78/79/7A/7B) + 4 companions (60/64/68/6C), ported from the 7900XTX trace. Applied at boot only in non-USB4 mode (main.c:531). | GPU-PATH | ⚠️ |
| `bank1_write`/`bank1_or_bits` (pcie_tuning.h:16/22) | (DPX-banked XDATA write) | DPX=1 banked write/OR helper for PHY tuning. | SHARED | ✅ |
| — (absent) | `CODE:8000` pcie_tunnel_link_setup | BANK0 tunnel data-path setup (B220 cfg-TLP, B230/B234/B240/B244/B246 link params, read neg'd width, 0x05B4=2). Handmade has B220 mailbox in cm_tunnel.h but NOT this bank0 tunnel-link-setup as a unit. | GPU-PATH | ❌ |
| — (absent) | `CODE:99e0` pcie_write_xdata_assert_perst | `B480=(B480&0xFE)\|1` standalone PERST-assert. Inlined into `pcie_power_on`/`pcie_power_off`/`cm_arm_c00d` rather than a shared fn. | GPU-PATH | ❌ |
| — (absent) | `CODE:3bcd` / `519e` / `e6fc` etc. (3578 sub-helpers) | The stock 3578 cap-read + retry sub-machine (B220=0x1404600 cap probe, cc10 PHY-cmd waits, 519e width apply). Not ported — `pcie_power_on` does no B220 cap read. | GPU-PATH | ❌ |
| `cm_tunnel_link_bringup_start` (cm_tunnel.h:66) | `CODE:92bb` pcie_tunnel_link_bringup_start | `0x06E6=1; c00d()` — present, but the c00d step-machine it drives is the NVMe-harmful arm (DEAD/uninvoked on GPU path). | SHARED? | 🔶 |

### State machines

| State | Handmade | Stock | Status |
|---|---|---|---|
| Downstream LTSSM bring-up | `pcie_power_on` bounded loop: PERST cycle → poll B450 until 3× `0x78` CONNECTED (or 20-attempt timeout) | 3578: B455=2/4 speed, deassert PERST, poll B455.1 train-done, d436 x4, return 0x0F | ➕ replaced |
| Link-up criterion | TWO coexist: `pcie_power_on` uses **B450==0x78**; `cm_link_up_check` (e2a6) uses **(B432&7)==7 && E765.2** | e2a6 = (B432&7)==7 && E765.2 | ⚠️ divergent gate |

### diff note

Section 7 is almost entirely ➕ handmade-only by design: the stock LTSSM bring-up `CODE:3578` (B455 LTSSM-speed ramp + B220 cap-read retry machine + d436 width → return 0x0F) is REPLACED by `pcie_power_on` (main.c:91), a B431/B403/B480-PERST-cycle + B298/C656/C659 rail sequence that polls a different register/value (B450==0x78) for "CONNECTED". This is the resolved boundary: stock's 3578 is reached only via the 9037/c00d NVMe step-machine whose PERST/d436/C659 re-drive races the downstream LTSSM, so the handmade live path deliberately uses the custom `pcie_power_on` fired at 0xF3 / after lane-bond (main.c:704). The handmade B220 config-TLP work and the e2a6 link-up check (cm_link_up_check) ARE ported byte-true but live in the §5 CM layer; the tinygrad host MMIO interface (0xF0 config-TLP engine + pcie_pio PIO) has no stock counterpart at all. Real GAPS for GPU visibility are NOT in this section's local-PCIe bring-up (that reliably reaches 0x78 CONNECTED per MEMORY) but upstream in §5 (the stubbed c0a5 router-op mailbox + un-lifted c5ff de-alias). Two divergence flags to verify: (1) `pcie_power_on` skips the stock B220 cap-read (0x1404600) and d436 x4 width-apply — link trains x2 only (B431=PCIE_LINK_WIDTH_x2=0x0C), whereas stock 3578 finalizes x4; (2) the two coexisting link-up gates (B450==0x78 vs B432&7==7 && E765.2) test different registers and could disagree.

---
## 8. USB Device

The handmade USB-device subsystem keeps the stock USB2/USB3 enumeration plumbing (descriptors, EP0 control engine, SS PHY tune, int0 demux) but **rips out the entire USB-MSC/NVMe storage data-path and replaces it with a custom vendor BULK-EP interface** (the tinygrad interface). Below, `usb.h` and the `main.c` control/bulk/ISR bodies are mapped against stock §8 (`0e5b`/`9c2b`/`4532`/`4904`/`ca0d`) and the NVMe/MSC storage fns from the NVMe-omit table.

### Functions

| Handmade fn (file:line) | Stock counterpart (addr) | Role | Class | Status |
|---|---|---|---|---|
| `int0_isr` (main.c:374) | `CODE:0e5b` (int0_isr body) | USB-device/peripheral ISR: PERIPH_STATUS demux (bus-reset/91D1 link-event, control, ALT-LINK, bulk, EP-complete, 9302/9300 SS link-event, CBW); MSC-pending ack. NVMe-queue tail of stock dropped. | USB-DEV (SS-link-event arms `bank0_c9a8` = SHARED?) | ⚠️ |
| `handle_usb_control` (main.c:142) | inside `CODE:0e5b` + stock `4532`/SETUP dispatch | EP0 SETUP/DATA/STATUS state machine: SET_ADDRESS, GET_DESCRIPTOR, CLEAR_FEATURE(EP_HALT), SET_CONFIG (bypasses MSC), SET_INTERFACE, vendor 0xC0/0xE4/0xE5/0xF0/0xF2/0xF3. | USB-DEV (vendor reqs ➕ handmade-only) | ⚠️ |
| `handle_usb_bulk_data` (main.c:348) | `CODE:4904`/`4784`/`180d`/`3419` (MSC BOT/UAS data) | Bulk EP1/2 completion: drains BULK-OUT → `pcie_write_chunk(0x7000)`, re-arms; BULK-IN → `do_usb_bulk_in`. Replaces MSC CBW/CSW/SCSI data-path. | GPU-PATH? (vendor bulk = ➕; replaces NVMe-ONLY MSC) | ➕ |
| `do_usb_bulk_in` (main.c:129) | absent (no stock vendor-bulk streamer) | Streaming IN: `pcie_read_chunk(0x8000)` chunk → arm EP IN; chunk = 512/1024 cap by `is_usb2`. | GPU-PATH (tinygrad TLP stream) | ➕ |
| `usb_handle_get_descriptor` (usb.h:256) | inside `CODE:0e5b` GET_DESCRIPTOR arm | DEVICE/CONFIG/BOS/STRING descriptor select (USB2 vs SS variants) → DESC_BUF → `usb_send_data`. | USB-DEV | ✅ |
| `usb_handle_set_address` (usb.h:250) | inside `CODE:0e5b` | SET_ADDRESS: program 0x9090 int-mask, 91D0=0x02, ZLP. | USB-DEV | ✅ |
| `usb_send_data`/`usb_send_zlp` (usb.h:238/244) | EP0 IN DMA arm in `0e5b` | EP0-IN length + DMA trigger + DATA_IN phase. | USB-DEV | ✅ |
| `usb_desc_copy` (usb.h:246) | code-table copy in `0e5b` | Copy `__code` descriptor → DESC_BUF. | USB-DEV | ✅ |
| `usb_build_string_desc` (usb.h:94) | string-desc builder in `0e5b` | UTF-16LE STRING descriptor encoder. | USB-DEV | ✅ |
| `usb_build_serial_desc` (usb.h:109) | OTP-serial string path | OTP serial → 8-char hex STRING (ffffffff fallback). | USB-DEV / HOUSEKEEPING | ✅ |
| `usb_dev_desc` / `usb_dev_desc_ss` / `usb_cfg_desc` / `usb_cfg_desc_ss` / `usb_bos_desc` (usb.h:27-91) | stock descriptor `__code` tables | Device/config/BOS descriptor data. **Handmade VID=0xADD1 PID=0x0001 vendor-class (0xFF) 4-bulk-EP**; SS cfg retains MSC BBB/UAS alt for compat. | USB-DEV (vendor IDs ➕) | ⚠️ |
| `usb_serdes_tune_lane`/`usb_phy_tune` (usb.h:137/161) | stock SS SerDes tune (C280/C300 lane RMW) | SS PHY per-lane register tuning; only run in non-USB4 mode (main.c:526). | USB-DEV / SHARED | ✅ |
| `usb_init_controller` (usb.h:166) | stock SS controller init (part of bring-up; cf. `9c2b` SS-link engine) | USB controller reset/EP0/MSC-init regs; `force_usb2` path. Called only in non-USB4 mode (main.c:569). | USB-DEV | ✅ |
| `usb_pipe_engine_init` (usb.h:181) | stock USB PIPE/PHY engine init | PIPE/PHY bring-up (9300-9305 buf-cfg, 91Cx PHY, USB_MODE=1); run unconditionally at boot. | USB-DEV / SHARED | ✅ |
| `boot_phy_early_settle`/`usb4_phy_arm` (usb.h:205/224) | stock CC10-mailbox PHY arm | Timer0 CC10-mailbox PHY-link arm with bounded waits (USB4 PHY link-up). | SHARED (USB4 PHY) | ✅ |
| `rmw` (usb.h:133) | stock RMW helper | XDATA read-modify-write helper. | HOUSEKEEPING | ✅ |
| (SS link-train) absent | `CODE:9c2b` usb_ss_link_train_engine | SS link-train via 91D1.3 cc10; on SS-FAIL force USB2. Handmade has NO equivalent train engine — int0 only does the SS-FAIL→USB2-fallback policy inline (main.c:416-429), and in USB4 mode suppresses the drop. | USB-DEV | ❌ / ⚠️ (policy-only inline) |
| (SS link-event policy) partial @ main.c:407-435,381-389 | `CODE:4532` usb_ss_link_event_policy_0003 | Stock deferred 0x0003-bitmap policy (b3→pcie-down bringup, b4→USB4 lane train, b6→TypeC recovery, b2→SS re-init). Handmade int0 services 9302.2/9300 inline (→`bank0_c9a8`, SS-FAIL fallback) but does NOT implement the full 0x0003 deferred-policy demux. | USB-DEV / SHARED? | ⚠️ |
| (link-mode finalize) absent | `CODE:ca0d` bank0_ca0d | Latch `entered_usb_mode=0x10`. No direct handmade port (USB4 mode-entry handled in pd/usb4 subsystems). | USB-DEV | ❌ |
| absent | `CODE:4904` usb_msc_init | USB-MSC BOT engine + NVMe doorbell ('USBS' CSW). Replaced by vendor bulk; SET_CONFIG writes `REG_USB_MSC_CFG=0` to bypass. | NVMe-ONLY | 🚫 |
| absent | `CODE:1196` nvme_queue_service | NVMe IOSQ/IOCQ drain + C47A ack. | NVMe-ONLY | 🚫 |
| absent | `CODE:488f` nvme_completion_handler | NVMe CQ handler. | NVMe-ONLY | 🚫 |
| absent | `CODE:3e81` nvme_handler_3e81 | NVMe handler (USB host connected). | NVMe-ONLY | 🚫 |
| absent | `CODE:4784` nvme/MSC service 4784 | BOT/UAS→NVMe bridge. | NVMe-ONLY | 🚫 |
| absent | `CODE:3419` FUN_CODE_3419 | SS enum + SCSI/MSC CBW→DMA; sets 0x06E6 to arm NVMe CM. Replaced by vendor bulk. | NVMe-ONLY | 🚫 |
| absent | `CODE:180d` FUN_CODE_180d | USB2 MSC SCSI + NVMe-param/DMA walk. | NVMe-ONLY | 🚫 |

### State machines

EP0 control state machine (`handle_usb_control`, main.c:142-344), keyed on `REG_USB_CTRL_PHASE`:

| Phase bit | Arm | Handmade action | Stock counterpart |
|---|---|---|---|
| SETUP | `USB_CTRL_PHASE_SETUP` | decode bmReq/bReq → standard (SET_ADDRESS / GET_DESCRIPTOR / CLEAR_FEATURE / SET_CONFIG / SET_INTERFACE) + vendor (0xC0 hw_status, 0xE4 XDATA-rd, 0xE5 XDATA-wr, 0xF0 TLP, 0xF2 SRAM-DMA, 0xF3 PCIe-pwr) | stock SETUP demux in `0e5b` (vendor branch = ➕ handmade) |
| DATA_IN / STAT_IN | `USB_CTRL_PHASE_DATA_IN`/`STAT_IN` | 0xF0 DATA_OUT TLP build (FMT/TYPE/ADDR/DATA, mode0 single / mode1 OUT-stream / mode2 IN-stream) | stock EP0 data phase |
| STAT_OUT / DATA_OUT | resp. | DMA recv / ack | stock EP0 status |

Bulk-streaming SM (`do_usb_bulk_in` ↔ `handle_usb_bulk_data` ↔ `dma_dwords`): vendor 0xF0 mode1/2 arms OUT/IN, completion re-arms until `dma_dwords` drained — **➕ handmade-only**, no stock MSC CBW/CSW/SCSI counterpart.

### diff note

The USB-device **enumeration core** (descriptors, EP0 control engine, SS PHY tune/pipe init, int0 PERIPH_STATUS demux) is ported byte-faithfully and is fine for the GPU path. The **storage data-path** is wholly replaced: every stock USB-MSC/NVMe data fn (`4904`/`1196`/`488f`/`3e81`/`4784`/`3419`/`180d`) is intentionally dropped (🚫 omitted-NVMe) in favor of a vendor BULK-EP streamer (`do_usb_bulk_in`, `handle_usb_bulk_data`, vendor reqs 0xC0/0xE4/0xE5/0xF0/0xF2/0xF3) — ➕ handmade-only, and this is by design (tinygrad interface), NOT a GPU-path defect.

Two real **omissions vs stock §8** that are USB-DEV (not NVMe): (1) the SS link-train engine `CODE:9c2b` has no handmade port — int0 only does the inline SS-FAIL→USB2 fallback policy (main.c:416-429), which is gated off in USB4 mode; and (2) the full deferred 0x0003 link-event policy `CODE:4532` is only partially reproduced inline (9302.2/9300 → `bank0_c9a8`, SS-FAIL). These are LOW severity for the GPU path because in USB4 mode (`0x09F9&0x83`) the SS device bring-up is deliberately skipped (main.c:565) — the USB4 lane-train / pcie-down-bringup that §8's `4532` b3/b4 arms is instead driven by the USB4/SB subsystems. They would matter only for a USB3-fallback storage scenario, which is out of scope here. The vendor BULK path's GPU relevance is the 0xF0 TLP / 0xF2-0xF3 hooks the host uses to drive tunneled config — those are present and functional.

---
## NVMe — intentionally omitted (handmade), documented for reference

The stock NVMe storage path is replaced by the vendor-bulk interface (see
Top-level #2) and the harmful CM step-machine is dead (see #3). Omitted stock
fns (all 🚫 omitted-NVMe / dead): `usb_msc_init(4904)`, `nvme_queue_service(1196)`,
`nvme_completion_handler(488f)`, `nvme_link_event_handler(49e9)`, `nvme_handler(3e81)`,
`4784`, `cm_pcie_link_step_machine(9037)`, `cm_arm_c00d(c00d)`, `e8e4`, `480c`,
`3419`, `180d`, `a183` (class-01:08:02 enroll), `bank1_89db` (IOSQ builder).

> **BOUNDARY (Phase-2 resolved):** `9037`/`c00d` are NVMe + HARMFUL on the GPU
> path and correctly dead. **EXCEPTION:** `c5ff`'s 4× CfgWr route de-alias is
> SHARED and must be **LIFTED** out of 9037 into a GPU-path adb0 one-shot
> (PERST-free); route-bind primitives `e91d`/`d956`/`9ee5` are SHARED. See
> `DRIVER_DISCREPANCIES.md#c5ff-dealias-not-lifted` and `STOCK_DRIVER_MAP.md`
> §NVMe note.

---
## Appendix A — leaf sub-helpers folded into mapped parents

Coverage is complete at the **whole-handler** level (completeness critic: every handmade function resolves to a mapped parent). These leaf helpers are not individually rowed above; they fold into the named parent:

- cc_pd_timer_tick INT1 sub-handlers (pd.h:199-234): cc_cc23_reinit_event, cc_state_full_reset, pd_cc81_hard_reset_4, pd_queue_ctrl_msg, cc_cc99_default_event, cc_ccf9_subdemux, cc_ctrl_enable_events — folded into the §6 cc_pd_timer_tick row but none individually mapped; cc_ccf9_subdemux is the CC91.1 1s-timeout USB4-commit demux worth a row.
- pd_dispatch_control arm handlers NOT rowed in §6: pd_ctrl_goodcrc (pd_dispatch.h:115), pd_ctrl_reject (:176), pd_ctrl_wait (:197), pd_rx_nak_send (:211) — §6 rows only accept/ps_rdy/soft_reset.
- SB connect-service helpers NOT rowed in §4-SB/§3: sb_chan_prelude (sb_router.h:250), sb_clear_cl0_width_latches (:386), sb_set_d4_peer_cl0 (:391), sb_write_c9_ack (:22) — fold into sb_channel_connect_service / a066 servicing.
- usb4_irq_arm sub-helpers NOT rowed: usb4_phy_rx_descriptor_8e31 (usb4_irq.h:44, a large ~160-line PHY-RX descriptor config), usb4_irq_db0d (:211), usb4_irq_ef1e (:227), usb4_irq_ef24 (:236) — §1 says usb4_irq_arm 'covers ef24/ef1e' so acknowledged but 8e31/db0d are unnamed.
- state-3/4 lane-bond engine helpers NOT rowed in §4-FSM: u4lb_e711_tail (usb4_lanebond.h:77), u4lb_engine_go (:83), u4lb_sb_desc_commit (:90), u4lb_e890 (:97), u4lb_e7f8 (:105), u4lb_d195 (:579), u4lb_d1d3 (:584), u4lb_e74e (:619 — the 074E/074F state-4 latch), u4lb_e916 (:775) — fold into the generically-listed 'a840-engine helpers' / b0b4.
- sb-assert bring-up helpers NOT rowed (fold into sb_assert/sb_lane_flip_init/sb_block_init, §4-SB): eng_a30c/a308/a2df/a31c/a348/a327, u4c_sb_desc_commit, u4c_edbd, u4c_e5b0, u4c_ccb3, u4c_c270, u4c_d556, u4c_bcd7_tail, sb_pcie_width_ramp.
- boot/flash/i2c/uart leaf helpers (HOUSEKEEPING, fold into mapped parents): phy_cc11_ack, phy_cc10_cmd(_wait), boot_phy_d118, boot_phy_bceb_set0, boot_phy_e57d_e764_reset_pulse, flash_poll_busy, i2c_fire_wait/point/write_reg16/read, ina231_read_u16, uart_putc/puts/puthex, sleep, hw_status_read (the vendor-0xC0 backing fn), pd_vdm_hdr_build, pcie_apply_rxphy_* slices (§7 lists these as 'slice helpers'), bank0_8a89 connect helpers u4c_bd2a/bcf2/bd41/bd14/bd6c/e7ae_bounded.

---
