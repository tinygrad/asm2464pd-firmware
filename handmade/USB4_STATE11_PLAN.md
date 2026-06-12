# state-0x11 port plan — cm_conn_routing_setup deep-PHY/descriptor layer

Status: **RE COMPLETE, NOT YET IMPLEMENTED.** Untestable until the upstream SB-transport TX wall
(see USB4 task #12 / bb37+e1cb channel-arming) is cleared and the route-query completes so the FSM
reaches `0x0758==0x11` with `0x0777==0x0C`. Implement this RIGHT AFTER the wall fix lands.

Source: ghidra RE of stock `cm_conn_routing_setup` (CODE_BANK1::a7de) state-0x11 block + e391/c586/
e175/e282/c17f/d221. All ROM bytes re-read from Ghidra (verified). Adversarial verify: e391 addressing
✓, connect-confirm gates ✓, bank0-stub reachability ✓ (call via shared stub addrs, see risks).

## What's missing in handmade
`u4lb_cm_conn_routing_setup` (usb4_lanebond.h:164) reproduces the FSM + edf5 query but the state-0x11
body is comment-only "OMITTED" markers. This block does: e391 descriptor seed -> connect-confirm ->
0x0718=4 route-enable (EXISTING) -> lane/width latch (EXISTING) -> direction-flag compute (BUGGY:
only clears, must conditionally set) -> SB[0x65] connect-confirm RMW -> c586/e175/e282/c17f tunnel PHY.
Without it, even once the route-query completes the tunnel won't bring up (0x0718 route-enable + PHY
config never happen).

## ROM tables (verified from Ghidra; shared CODE, declare __code)
```c
static const __code uint8_t phy_cfg_514c[19]={0x04,0x04,0,0x04,0x04,0,0,0,0x04,0x04,0x01,0,0x03,0x04,0,0x04,0,0,0x10}; /* CODE:514c -> XDATA 0x06F2..0x0704 */
static const __code uint8_t phy_cfg_515f[19]={0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1};                                   /* CODE:515f -> XDATA 0x0705..0x0717 */
static const __code uint8_t lane_class_21c4[16]={5,5,5,5,5,5,6,6,6,6,6,0x0A,0x0A,0x0A,0x0A,0x0A};                        /* CODE:21c4 -> XDATA 0x071A..0x0729 */
```

## New accessor (r3/bank-2 PHY plane = DPX=1 page 0x12, same window as SB_RD page 0x28)
```c
#define PHY2_RD(reg)    P1_REG8_rd((uint16_t)(0x1200u + (reg)))
#define PHY2_WR(reg,v)  P1_REG8_wr((uint16_t)(0x1200u + (reg)), (uint8_t)(v))
```
No new persistent XDATA state required. Staging bytes 0x0B34..0x0B38 / 0x0A9D..0x0AA1 are native chip
flat XDATA (DPX=0) via PR(); reuse those addresses to stay byte-faithful (do NOT allocate arrays).

## Insertion points (all in usb4_lanebond.h)
1. **File scope ~line 96** (before u4lb_edf5_route_query, so it precedes both call sites): the PHY2_*
   macros, the 3 __code tables, and the helpers e83d/ce23/u4lb_c586/c3ce/u4lb_e175/u4lb_e282/u4lb_c17f.
2. **~line 220-222** (replace the e391 OMITTED marker): the `if(PR(0x0776)==0){...}` seed.
3. **line 242** (replace the bare `PR(0x0763)=0;PR(0x0764)=0;`): the init-then-conditional-set compute.
   This is a CORRECTION — stock conditionally SETS one flag, feeding the 9a06 gate of the SB[0x65] RMW.
4. **~line 245-247** (replace the a95f OMITTED marker): T2..T8 below.

## Ordered body (C-pseudocode on handmade accessors)
```c
/* e391 seed (insert #2): runs only if 0x0776==0 */
if (PR(0x0776)==0) {
  uint8_t i;
  for (i=0;i<19;i++){ PR(0x06F2+i)=phy_cfg_514c[i]; PR(0x0705+i)=phy_cfg_515f[i]; } /* 16-bit base add */
  for (i=0;i<8;i++) PR(0x0B26+i)=0;                                                  /* zero router-op buf */
}

/* [ConnRout]/0x0718 latch + 077A lane-width latch: EXISTING, unchanged */

/* direction-flag compute (insert #3, REPLACES the bare clear at line 242) */
PR(0x0763)=0; PR(0x0764)=0;                                /* a90b init */
if (PR(0x0750)==2) {                                       /* a919 LIVE AMD 20G path */
  if ((PR(0x077A)&0x80) && (PR(0x081A)&0x80)) PR(0x0764)=1;
} else {                                                   /* a930 */
  if (PR(0x07BA)!=0 && (PR(0x077A)&0x40) && (PR(0x081A)&0x40)) PR(0x0763)=1;
  else if (PR(0x07B9)!=0 && ((PR(0x077A)&0x40) || (PR(0x081A)&0x40))) PR(0x0763)=1;
}

/* a95f tail (insert #4) */
/* (T1) bank0_db80_stub(0x0764,0x0763) tunnel descriptor-direction apply — BODY UNMAPPED.
        emit a documented uart-marker placeholder, do NOT fabricate (ROUND B). */
/* (T2) SB[0x65]/0x2865 connect-confirm RMW, gated by (0x0763|0x0764) */
if ((PR(0x0763)|PR(0x0764))==0) { uint8_t v=SB_RD(0x65); SB_WR(0x65,v&0xBF); v=SB_RD(0x65); SB_WR(0x65,v&0x7F); }
else                            { uint8_t v=SB_RD(0x65); v=(v&0xBF)|0x40; SB_WR(0x65,v); v=SB_RD(0x65); v=(v&0x7F)|0x80; SB_WR(0x65,v); }
/* (T3) 0x0750==1 -> 0x21c4 copy (SKIPPED on live AMD path where 0x0750==2) */
if (PR(0x0750)==1) { uint8_t i; for (i=0;i<16;i++) PR(0x071A+i)=lane_class_21c4[i]; }
/* (T4) conditional E716 poke */
if ((PR(0x0AF1)&0x01) && (PR(0x09FA)&0x02)) PR(0xE716)=(PR(0xE716)&0xFC)|0x03;
/* (T5..T8) tunnel-adapter PHY */
u4lb_c586();
u4lb_e175(PR(0x0751));
u4lb_e282(PR(0x07B9));
u4lb_c17f();
PR(0x0758)=0;                                              /* state done */
```

## Helper bodies
(c586 PART A page-1 word seed + PART B C8FF-dispatched C29x/C31x lane bringup with early-RET paths;
e83d bounded poll; ce23 lane-mask commit; e175/e282 two-pass; c17f PASS1 seed + PASS2 connect-enable +
0xB12-gated secondary-adapter + c3ce per-lane broadcast.) See the full transcription in the workflow
output `/tmp/claude-1000/.../tasks/wl9pqfi7q.output` (synth.orderedSequence) — paste verbatim, adjusting
only: bound EVERY busy-poll with a ~0x2000 guard counter (like u4lb_d5da/ebde), and call the bank0 stubs
via their SHARED stub addresses 0x05cf(c17f)/0x05d9(e175)/0x05de(e282)/0x05e3(db80) NOT direct bank1.

## Risks / discipline
- **bank0_db80_stub (T1) body unmapped** — placeholder only, do not fabricate (ROUND B).
- **bank0 stubs**: e175/e282/c17f bodies given are r3-plane (PHY2_*) DPX math, NOT a true code-bank
  switch — faithful to transcribe directly. But the literal stock call is via shared stub 0x05cf/05d9/
  05de; the bodies are reproduced inline here (equivalent), so direct inline is fine for handmade.
- **c586 PART B control flow**: early-RET paths (mode4 & 0x07BA!=0; mode5 & 0x07BA!=0; mode>=6 & 0x07BA!=0)
  must NOT fall through to the single-lane fixup; fall-through paths AND mode<4 MUST reach it. Model PART B
  as a function with early `return`s exactly as written.
- **Bound every poll** (~0x2000) — no unbounded do/while (super-loop must not hang). Per no-speculative-fixes.
- **IRAM**: all C locals + PR()/P1_REG8 on existing chip XDATA — adds NO IRAM globals. Shallow call depth,
  runs in super-loop (not the nested-INT1 stack cliff). DSEG<=0x6C/sp=0x6B unchanged.
- **Untestable** until the TX wall clears (0x0777==0x0C reached). Faithfulness, not behavior, is the bar.
- Optional 1-shot debug byte -> XDATA __at(0x8819), extend main.c boot zero-loop bound + seed.
```
