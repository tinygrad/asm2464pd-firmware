# Remaining-Driver Plan — finish the handmade USB4→PCIe GPU-tunnel firmware

## Goal
AMD GPU `1002:7590` enumerates in **host** `lspci` over the USB4/TB4 PCIe tunnel, using the handmade
firmware (`handmade/src/main.c` + headers), matching stock `fw_tinygrad.bin`. The literal CLAUDE.md
gate: `DEBUG=2 PYTHONPATH="." DEV=USB+AMD python3 test/test_tiny.py TestTiny.test_plus`.

## Current state (HW ground truth, 2026-06-18) — NOT a regression
Handmade host-visible peak (reached at commit 79b255d, carried forward into HEAD 757c52d): host
enumerates the **host** TB router `1-0:1.1`, both USB4 lanes BONDED (`SB[0xA0]=SB[0xA1]=0x02`,
host `d1[CL0 CL0]`), device↔GPU **local** PCIe LTSSM `0x78 CONNECTED Gen3 x2`. **Wall:** the host
posts **zero** USB4 router-ops (`EC06=0`) in every commit → the **device** TB router `1-1` never
enumerates → its PCIe-Up adapter (stock Route1/Adapter3) is never discovered → GPU never tunnels.
The device-side adb0 config-TLP engine works but downstream reads return UR/route-echo because the
`c5ff`/`a183` ECAM-aperture→GPU route binding is unimplemented. git ancestry proves nothing was lost;
this session's `cm_arm_c00d` removal (link-race bug fix) + `cap20g_gate1=1` + `0x0A52` seed are
additive. Lane bond + LOOP1 finalize + af38 + 8992 + 843C are all byte-true and ruled out.

## Strategy (4 phases — one fresh context per phase where possible)
**PHASE 1 — MAP STOCK (workflow `enum-…`/stock-map):** Produce a high-level architectural map of the
ENTIRE stock driver → `handmade/STOCK_DRIVER_MAP.md`. Cover: reset/init code, ISR/interrupt handlers
(all vectors), the superloop + its dispatched handlers, every state machine (USB4 lane-bond/PHY/
SB-transport, PD/VDM, CM-tunnel/router-ops, local PCIe), classified by subsystem. NOT byte-level —
structure + roles + state lists + transitions + call relationships + Ghidra addresses.

**PHASE 2 — MAP HANDMADE + DIFF (workflow):** Map what `handmade/src/*` currently implements in the
SAME structure → `handmade/HANDMADE_DRIVER_MAP.md`. Diff vs STOCK_DRIVER_MAP.md → a discrepancy list
(`handmade/DRIVER_DISCREPANCIES.md`): missing handlers/sections, wrong dispatch, omitted state-machine
arms. Fix the HIGH-LEVEL structural discrepancies first (a whole missing handler/section).

**PHASE 3 — DEEP-DIVE EACH SECTION (workflow per section):** For each section of the map, dive
byte-level (Ghidra vs handmade source) and fix every implementation bug/omission. Adversarially verify
each fix is byte-true to stock. Prioritize the GPU-tunnel critical path (see below).

**PHASE 4 — HW-VALIDATE (delegate to board subagent):** Iterate flash/capture toward the milestone
chain: `EC06>0` (host posts a router-op) → device router `1-1` enumerates → host posts first `0xE8`
[PcieTunnel-Deassert] → PCIe-Down adapter enabled → GPU `1002:7590` in lspci.

## GPU-tunnel critical path (what PHASE 3 must get byte-perfect first)
Post-bond: device must advertise a discoverable PCIe adapter so the **host CM** starts the config
transport (posts router-ops). Suspect order: (a) what device action between `[PcieTunnel-PwrOn]` and
the host's first `0xE8`/`EC06` provokes the host (the adapter-CS page-1 register; `0x1208/0x1210`
writes tested 2× did NOT provoke it — find the real one); (b) `cm_routerop_mailbox` (usb4.h:63) is a
STUB — port the stock `c0a5→c0ef→d945`/`cf5d` config R/W engine; (c) the `e461` route-push host-ack
(`0x0775`) candidate; (d) `c5ff`/`a183` ECAM-aperture→GPU route binding so a tunneled MemRd returns
the real GPU id (not the route-echo/UR).

## ⚠️ NVMe classification rule (the user's explicit caution)
Stock fw is primarily a **USB4-NVMe-enclosure** firmware — a LOT of it is NVMe. For OUR GPU goal NVMe
is (probably) not needed, BUT do NOT delete it from the maps and BE CAREFUL not to misclassify:
- `[GPU-PATH]` — needed for the GPU tunnel. KEEP + implement.
- `[SHARED]` — infrastructure used by BOTH NVMe and the GPU path (e.g. the adb0 config-TLP engine,
  the PCIe-tunnel adapter setup, the SB/router transport). KEEP + implement. The `9037` config walk
  is NVMe-oriented but the engine it drives may be SHARED — classify the ENGINE shared, the
  NVMe-specific walk/enroll (`a183` class 01:08:02, IOSQ/IOCQ at 0xA000) as NVMe.
- `[NVMe-ONLY]` — pure storage path. DOCUMENT in the map for reference, mark **OMIT (unless needed)**,
  do NOT implement now.
- When uncertain → mark `[SHARED?]` and flag for verification; **never silently omit** an uncertain
  function. Always `log()`/note what was classified NVMe-omittable so we can revisit.

## Key context pointers
- Memory dir: `/home/batman/.claude/projects/-home-batman-asm2464pd-firmware/memory/` (read MEMORY.md
  index; the live task file is `project_cm_walk_host_enable_open.md` — read its TOP sections).
- Ghidra: stock `fw_tinygrad.bin` is loaded in Ghidra MCP WITH handmade symbol names ported; bank1 via
  `CODE_BANK1::<addr>` prefix; bank0/common flat `0x0000-0xFF69`. Static export:
  `handmade/stock_ghidra_export.c`. `handmade/USB4_RE.md`, `handmade/src/*.h`.
- Build: `make -C handmade wrapped` (or `nflash`). HW test on NUC rig: `./test.sh <bin> <secs>`
  (stages binary in HOME + md5-verifies; NUC wipes /tmp). Stock flash: `./ftdi_debug.py -bn &&
  ./flash.py fw_tinygrad.bin && ./ftdi_debug.py -rn`. Reboot NUC `sudo reboot -f`; NEVER unbind FTDI.
- ghidra.c addresses == fw_tinygrad body offsets (NOT fw.bin).

## Artifacts produced by this plan
- `handmade/STOCK_DRIVER_MAP.md`  (Phase 1)
- `handmade/HANDMADE_DRIVER_MAP.md` + `handmade/DRIVER_DISCREPANCIES.md` (Phase 2)
- byte-true fixes in `handmade/src/*` (Phase 3), HW-validated (Phase 4)
