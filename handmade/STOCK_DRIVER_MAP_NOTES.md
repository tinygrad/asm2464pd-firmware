# STOCK_DRIVER_MAP.md — Phase-1 critic findings + priorities (APPLY in Phase 2)

## ✅ PHASE-2 RESOLUTION STATUS (2026-06-19, Ghidra-verified via wf_491f4a4f)
All Phase-1 critic findings below are now APPLIED into `STOCK_DRIVER_MAP.md`:
- **c5ff boundary — RESOLVED.** c5ff's only caller is 9037; reaching its call-site (`CODE:90ce`)
  needs `c00d`→`0x05B4==0x10` = the PERST race. But c5ff itself is 4× CfgWr-over-tunnel (NO
  PERST/PHY) + a `0x05B4=2` marker. **DROP 9037 (NVMe); LIFT c5ff's de-alias into a GPU-path adb0
  one-shot** (PERST-free). Primitives `e91d`/`d956`/`9ee5` = **[SHARED]**; `a183` enroll =
  **NVMe-ONLY**. De-alias is **necessary-but-not-sufficient**; the host-visibility gate is the
  **stubbed `c0a5` router-op mailbox** — implement BOTH. (Full write-up in the map's §NVMe note.)
- **Gap 1 (e391):** added as a §4 GPU-PATH row (width-LUT seeder, ROM `0x514c`/`0x515f`→`0x06F2`/`0x0705`).
- **Gap 2 (a7de callees):** corrected to the full 9 (`9814,e391,99f3,9a31,eda0,edf5,eb62,r3_write,sprint`).
- **Gap 3 (d3a2 gate):** corrected — inner gate `0x06EB.1 && u4_connect_gate.3` (NOT `0x0AF1.3`),
  call-site `af38_t53 && TIMER1_CSR.1`; KEEP.
- **Gap 4 (d2a1):** CORRECTED — `d2a1` is mid-instruction inside `FUN_CODE_d26f`, NOT a `cm_cmd_table`.
- **Misclassifications:** `cd6c` single-tagged **[SHARED]** (callers c00d + boot-stub 0462);
  bank0 `CODE:8000` vs bank1 `CODE_BANK1::8000` annotated DISTINCT.

---

Phase-1 workflow (`stock-driver-map`, wf_b78cb818-912) wrote `handmade/STOCK_DRIVER_MAP.md` (~95 funcs,
9 subsystems). The map SKELETON verified byte-true vs Ghidra (INT1 orchestrator @4486, INT0 @0e5b,
c0a5 router-op mailbox, e672 FSM dispatch 0x06ED→3:a7de/4:b0b4/5:8000|850b, boot @2f80). Critic
verdict: **map-has-gaps** — fix these in Phase 2 while building HANDMADE_DRIVER_MAP + the diff.

## ⚠️⚠️ CRITICAL — the c5ff NVMe/GPU boundary (exactly the user-warned misclassification)
The map self-contradicts: it (a) puts **c5ff** (route de-alias) on the GPU CRITICAL PATH step 8
("Host CfgRd de-aliased route (c5ff) reaches AMD GPU 1002:7590"), yet (b) recommends "drop the entire
a59 NVMe branch (9037)" — and Ghidra `get_function_callers(c5ff)` = **ONLY `cm_pcie_link_step_machine
@9037`**. So c5ff is reachable ONLY through the NVMe-tagged 9037 walk. If the GPU route de-alias
genuinely needs c5ff, then **9037 cannot be cleanly dropped** — we must either keep a 9037 SUBSET or
LIFT c5ff out into a GPU-path caller. Project memory has the same tension ("c5ff route de-aliases" vs
"9037 walk = NVMe not GPU path"). **Phase 2 MUST resolve this before any NVMe omission.** Related:
`a183` (NVMe enroll, correctly NVMe-ONLY) shares the `e91d`/`d956` route-bind primitives with c5ff —
the boundary is subtle; classify the route-bind PRIMITIVES as [SHARED], the NVMe enroll as NVMe-ONLY.

## Map GAPS to fix in Phase 2
1. **e391 (cm_init_routing_tables) MISSING as a function row** — it is the GAP-1 root cause (af38
   SBTX[1]=0x55 uninit vs stock 0x03 → device TX `0C55` vs stock `0C03`). It is a DIRECT callee of
   state-3 `a7de`, gated `if(XDATA[0x0776]==0)`, seeds ROM 0x514c[0x13] width-LUT + 0x515f[0x13]
   branch-A gate into XDATA[0x06F2+i]/[0x0705+i]. Add as GPU-PATH in §4.
2. **a7de callee list incomplete** — map shows only `edf5,eda0,r3_write_dispatch`; Ghidra
   `get_function_callees(a7de)` = `9814, e391, 99f3, r3_write_dispatch, 9a31, sprint, eda0, edf5,
   eb62`. Add e391 (LUT seed) + 9a31 (read_xdata_0819_clr_bit1) + 99f3 (get_xdata81a_bit6) + eb62 —
   these are the 0x0819 lane-advertise deps the map flags as open questions.
3. **d3a2 gate mismatch** — map says gate `0x06EB.1 && 0x0AF1.3`; actual superloop call-site gate is
   `af38_t53!=0 && TIMER1_CSR.1` (then writes TIMER1_CSR=2). Re-decompile d3a2 for its INTERNAL gate
   and reconcile before deciding keep/drop.
4. **d2a1 (cm_cmd_table)** = a movc DATA table (no xrefs), NOT an undecompiled function. Reclassify
   (like the c0c5 router-op table) so Phase-2 doesn't try to decompile it as code.

## Other misclassifications to reconcile
- **cd6c** double-tagged (SHARED §1 line 63 `..._b401` vs GPU-PATH §5 line 297). Callers = `c00d`
  (NVMe arm) AND boot stub `0462` (one-time boot master-init). → single tag **[SHARED]**, canonical
  name; annotate "also on GPU adapter-advertise path".
- **bank0 0x8000 vs bank1 0x8000 namespace collision** — §7 `pcie_tunnel_link_setup` (BANK0, bare
  `CODE:8000`) is a DIFFERENT function from §4 `u4lb_lane_gate` state-5 walker (`CODE_BANK1::8000`).
  Both GPU-PATH but must NOT be cross-wired in the port. Always bank-prefix.

## Top-5 GPU-PATH items to verify against handmade FIRST (Phase 2/3 priority order)
1. **af38** (CODE_BANK1::af38) — SBTX[1] reads uninit 0x55 vs stock 0x03; the **e391 width-LUT seed
   gate (0x0776==0) never fires on the live Connect_U4 path**. Highest-priority known divergence.
2. **cd6c + c8db** (PCIe-Down adapter advertise/enable: B410-B42B / B430|=1 / B401.0) — **THE open
   wall**: host never enables the adapter so never posts 0xE8.
3. **a7de** (state-3 cm_conn_routing_setup) — sets 0x0718=4 ROUTE-ENABLE + 0x0819 lane-advertise from
   0x077A; needs cap20g_gate1(0x09F6)=1 for both-lane (0x03) → route-id 303C → bond.
4. **8000** (CODE_BANK1, state-5 u4lb_lane_gate) — live AMD CL-walk driving SB[0xA0/A1] 0x07→0x02;
   LOOP2 0x30 reads 0x0779 (NOT the dead 850b reading 0x0B26).
5. **c0a5/c15f/e4a6** (router-op mailbox → 0xE8 PcieTunnel-Deassert: CC31.0 reset → B480 PERST
   deassert) — verify handmade's mailbox emits correct E2/E3 config responses so the host proceeds
   to post 0xE8 and bring the GPU out of PERST.
