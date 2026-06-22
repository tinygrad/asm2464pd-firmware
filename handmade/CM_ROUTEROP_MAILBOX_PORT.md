# CM router-op mailbox (c0a5) — byte-true port spec (Phase-3)

Ghidra-RE'd 2026-06-19 (wf_b2754add). This is the implementation spec for porting the stock
`c0a5` EC06.0 router-op mailbox into the handmade fw (replacing the `usb4.h:63` stub).

> **DEPENDENCY (HW-confirmed):** the host posts router-ops (incl. `0xE8` [PcieTunnel-Deassert])
> only AFTER it enumerates the device router `1-1` + discovers/enables the PCIe-Down adapter.
> On the current handmade fw the host posts **EC06=0** (router `1-1` never enumerates), so this
> mailbox is a PREPARED fix — it will not change HW behavior until the upstream SB router-config /
> `1-1`-enumeration divergence is fixed. Implement E2/E3 + E8 here; the E8/e4a6 PERST-deassert is
> the GPU-enable lever once the host starts posting.

## c0a5 top-level dispatcher (EC06.0 mailbox)

BYTE-TRUE c0a5 (CODE_BANK1::c0a5, the EC06.0 router-op mailbox top dispatcher). Disasm in execution order:

GATE: c0a5 MOV DPTR,#0xEA90; c0a8 MOVX A,@DPTR (read REG_SYS_CTRL_EA90); c0a9 XRL A,#0x5A; c0ab JZ c0b0; else c0ad LJMP c1ae (RET, do nothing). => only proceed when EA90==0x5A (host-armed magic).

STATE READ: c0b0 MOV DPTR,#0x0B02; c0b3 MOVX A (read u4_routerop_mbox_state); c0b4 JZ c0b9 (state==0 -> IDLE path); else c0b6 LJMP c167 (continuation handler for state 1/2).

IDLE PATH (state==0), c0b9..c0c2: latch opcode + dispatch.
  c0b9 MOV DPTR,#0xEA80; c0bc MOVX A (read REG_ROUTEROP_OPCODE_EA80); c0bd MOV DPTR,#0x0B03; c0c0 MOVX @DPTR,A (write u4_routerop_mbox_opcode = EA80); c0c1 MOVX A,@DPTR (re-read 0x0B03 into A); c0c2 LCALL 0x0DEF.
  0x0DEF is the 8051 movc switch helper (see send_resp_engine for its disasm). It POPs the return address (= c0c5, the start of the inline table that immediately follows the LCALL), uses A (=opcode 0x0B03) as the key, walks the table, and JMP @A+DPTR to the matched handler. The dispatch does NOT return to c0c5; each handler does its own RET back to c0a5's caller OR falls into the shared epilogue at c1a2 via LJMP/SJMP. (The bytes at c0c5..c0dc are TABLE DATA; Ghidra mis-renders them as code starting at c0de.)

DISPATCH TABLE @c0c5 (25 bytes, raw: c0 de e0 / c0 e8 e1 / c0 ef e2 / c1 19 e3 / c1 51 e4 / c1 58 e5 / c1 5f e8 / 00 00 / c1 ae). Entry format is 3 bytes [DPH][DPL][key]; 0def reads byte0=DPH, byte1=DPL; if BOTH zero it is the terminator and the next 2 bytes are the DEFAULT target. So:
  key 0xE0 -> 0xc0de (E0, dc4e identity-fill)
  key 0xE1 -> 0xc0e8 (E1, dc4e variant)
  key 0xE2 -> 0xc0ef (E2 cfg-READ, main path)
  key 0xE3 -> 0xc119 (E3 cfg-WRITE)
  key 0xE4 -> 0xc151 (E4 block-cfg d6dc)
  key 0xE5 -> 0xc158 (E5 cfgop e21b)
  key 0xE8 -> 0xc15f (E8 tunnel reset e4a6)
  terminator 00 00 then DEFAULT -> 0xc1ae (RET, unknown opcode ignored).
0def's match loop: A=0 MOVC reads DPH; if !=0 compare path; else if DPL !=0 also compare; on terminator (both 0) it INC DPTR x2 and loads [DPH][DPL] of the default (c1ae). Compare path 0e0a: A=2 MOVC reads key byte, XRL R0(opcode); JZ -> 0dff loads target ([DPH]=byte+0? actually byte2->R0=DPH via INC DPTR x2 then A=0/A=1 MOVC) and JMP @A+DPTR. On mismatch INC DPTR x3, loop.

CONTINUATION HANDLER (state 1/2) @c167, reached when 0x0B02 != 0:
  c167 MOV DPTR,#0x0B02; c16a MOVX A; c16b XRL A,#0x01; c16d JNZ c184 (not state 1 -> try state 2).
  STATE==1 (RMBOX_MULTIPKT_1, E2 cfg-read continuation): c16f INC DPTR (->0x0B03); c170 MOVX A (opcode); c171 CJNE A,#0xE2,c182 (if opcode!=0xE2 -> abort to c1a9 reset state); match: c174 LCALL 0xD945 (cm_routerop_send_read_resp); c177 LCALL 0xCEAB (addr-in-bounds; returns Z=more-to-do?). c17a JNZ c180 (NZ -> not done yet, just ack: SJMP c1a2); c17c MOV DPTR,#0x0B02; c17f MOVX @DPTR,A (A=0 here from ceab path -> state=IDLE); c180 SJMP c1a2 (epilogue). [c182 SJMP c1a9 = reset-state-and-RET.]
  STATE==2 (RMBOX_MULTIPKT_2, E3 cfg-write continuation) @c184: c184 MOV DPTR,#0x0B02; c187 MOVX A; c188 XRL A,#0x02; c18a JNZ c1ae (not 2 -> RET); c18c INC DPTR; c18d MOVX A (opcode); c18e CJNE A,#0xE3,c1a9 (!=E3 -> reset state); match: c191 LCALL 0xCF5D (send_write_resp); c194 MOV A,R7; c195 JNZ c199 (R7!=0 -> bounds check); c197 SJMP c1aa (R7==0: CLR-A already? no, falls to c1aa MOV DPTR 0x0B02 MOVX@ with A from R7=0 -> state IDLE, RET); c199 LCALL 0xCEAB; c19c JNZ c1a2 (more -> ack); c19e MOV DPTR,#0x0B02; c1a1 MOVX @DPTR,A (state=0); fall into c1a2.

REPLY/DONE EPILOGUE @c1a2: c1a2 MOV DPTR,#0xEA90; c1a5 MOV A,#0xA5; c1a7 MOVX @DPTR,A (write EA90=0xA5, signals host the op is consumed); c1a8 RET.
RESET-STATE-AND-RET @c1a9: c1a9 CLR A; c1aa MOV DPTR,#0x0B02; c1ad MOVX @DPTR,A (state=IDLE); c1ae RET. NOTE: c1a9 path does NOT write EA90=0xA5 (it abandons without acking — used when a continuation opcode mismatched).

EA81 READ/WRITE SUB-OPCODE: EA81 (REG_ROUTEROP_CFG_EA81) is the read/write selector, NOT decoded in c0a5 itself but inside each op via cf4c. cf4c: MOV DPTR,#0xEA81; MOVX A; XRL A,#0x50; RET -> returns A==0 (Z set) iff EA81==0x50 (READ). The E2/E3 ops do `LCALL cf4c; JZ ok; MOVX A,@DPTR(=EA81 still on DPTR); XRL A,#0x51; JZ ok; LJMP c1ae` -> accept only EA81==0x50 (read) or 0x51 (write); any other value bails to c1ae (RET, no ack). So 0x50=read sub-op, 0x51=write sub-op. (e4a6/d6dc/e21b also range-check EA81 in [0x50,0x51].)


## E2 — router config-space READ (c0ef)

BYTE-TRUE E2 cfg-READ handler @ CODE_BANK1::c0ef (routerop_op_E2_cfgread). Reached from dispatch when opcode 0x0B03==0xE2, state==IDLE.

c0ef LCALL 0xCF4C (EA81 gate: A=EA81^0x50, Z iff EA81==0x50 read).
c0f2 JZ c0fc (EA81==0x50 -> ok).
c0f4 MOVX A,@DPTR (DPTR still 0xEA81 from cf4c; re-read EA81); c0f5 XRL A,#0x51; c0f7 JZ c0fc (EA81==0x51 write also accepted).
c0f9 LJMP c1ae (any other EA81 -> RET, no ack).

c0fc LCALL 0xCEEF (ceef = addr-copy from mailbox; CONTRACT: MOV DPTR,#0xEA81; MOVX A; ANL A,#0x01; MOV DPTR,#0x0004; MOVX @DPTR,A  (store EA81.0 -> XDATA[0x0004], the read/write LSB flag); MOV DPTR,#0xEA82; LCALL 0x0D84 (load 4 bytes EA82..EA85 -> R4..R7 = the 64-bit/32-bit address from the mailbox); MOV DPTR,#0x0B04; LCALL 0x0DC5 (store R4..R7 -> XDATA[0x0B04..0x0B07] = the working-buffer addr); CLR A; RET).  => 0x0B04 := host-posted address from EA82..EA85; XDATA[0x0004] := EA81.0 (op-direction bit).

c0ff MOV DPTR,#0x0B0A; c102 LCALL 0xCF2E (cf2e is just below cf23; it loads 0x0B0A.. limit ptr into regs — pointer setup for the response builder).
c105 LCALL 0xD945 (cm_routerop_send_read_resp: builds + arms the read response packet; see send_resp_engine. Reads 0x0B04 addr + 0x0B0A limit, emits header status/code + data via C8B0<-0xEA / C805|=0x02, advances cursor).
c108 LCALL 0xCEAB (cm_routerop_addr_in_bounds: CONTRACT MOV DPTR,#0x0B04; LCALL 0x0D84 (load 32b addr R4..R7); MOV DPTR,#0x0B0A; LCALL 0x0D9D (load 32b limit R0..R3); CLR CY; LJMP 0x0D22 (32-bit compare addr vs limit). Returns A/Z: Z set (JZ) => addr>=limit => DONE; NZ => more bytes remain).
c10b JNZ c110 (NZ = more to transfer -> set multi-pkt state); c10d LJMP c1a2 (Z = done in one shot -> ack EA90=0xA5, leave state IDLE).
c110 MOV DPTR,#0x0B02; c113 MOV A,#0x01; c115 MOVX @DPTR,A (u4_routerop_mbox_state := 1 = RMBOX_MULTIPKT_1, so next EA90==0x5A re-entry runs the c167 read continuation).
c116 LJMP c1a2 (ack EA90=0xA5).

Net effect: E2 = "read N bytes of router config space starting at posted addr, stream them back, set state=1 if more remain". 0x0B02 set to 1. Register sequence above is exact.


## E3 — router config-space WRITE (c119)

BYTE-TRUE E3 cfg-WRITE handler @ CODE_BANK1::c119 (routerop_op_E3_cfgwrite). Reached when opcode 0x0B03==0xE3, state==IDLE.

c119 LCALL 0xCF4C (EA81 gate, Z iff EA81==0x50); c11c JZ c126; c11e MOVX A,@DPTR (re-read EA81); c11f XRL A,#0x51; c121 JZ c126; c123 LJMP c1ae (bad EA81 -> RET).

c126 LCALL 0xCEEF (same addr-copy as E2: XDATA[0x0004]=EA81.0; load EA82..EA85 -> A/R4..R7; store -> 0x0B04..0x0B07. ceef returns A = low addr byte / R7 etc.).
STAGE WRITE CURSOR 0x0B08/0x0B09:
  c129 MOV DPTR,#0x0B08; c12c MOVX @DPTR,A (0x0B08 := A from ceef = addr low byte / cursor lo);
  c12d INC DPTR; c12e MOVX @DPTR,A (0x0B09 := same A — write cursor hi seeded);
  c12f INC DPTR (->0x0B0A); c130 LCALL 0xCF2E (limit ptr setup, as E2).
ZERO ACCUM 0x0B0E (4-byte): c133 CLR A; c134 MOV R7,A; c135 R6,A; c136 R5,A; c137 R4,A; c138 MOV DPTR,#0x0B0E; c13b LCALL 0x0DC5 (store R4..R7 -> XDATA[0x0B0E..0x0B11] = 0).
c13e LCALL 0xCF5D (cm_routerop_send_write_resp: drains the host-posted write payload into router cfg space, builds the write-response; see send_resp_engine. Returns R7 = status/continue).
c141 MOV A,R7; c142 JZ c1ae (R7==0 -> RET, abort/no progress without ack).
c144 LCALL 0xCEAB (addr-in-bounds vs limit; Z=done).
c147 JZ c180? -> actually c147 JZ c180 region maps into shared SJMP c1a2. Listing: c147 JZ c180 (Z=done -> SJMP c1a2 ack); else fall:
c149 MOV DPTR,#0x0B02; c14c MOV A,#0x02; c14e MOVX @DPTR,A (u4_routerop_mbox_state := 2 = RMBOX_MULTIPKT_2 for the c184 write continuation);
c14f SJMP c1a2 (ack EA90=0xA5).

Net: E3 = "write N bytes from host payload into router config space starting at posted addr, set state=2 if more remain, ack". Full register sequence above is byte-exact (0x0B08/09 cursor stage, 0x0B0E zero-accum, cf5d drain, ceab bounds, 0x0B02=2).


## E8 — PCIe-tunnel reset / PERST-deassert (c15f → e4a6) ★ the [PcieTunnel-Deassert] op

BYTE-TRUE E8 tunnel-reset chain. Dispatch entry @ CODE_BANK1::c15f: c15f LCALL 0xE4A6 (tunnel_routerop_link_reset); c162 MOV A,R7; c163 JZ c1ae (R7==0 -> RET, the EA81-out-of-range early-return path); else c165 SJMP c1a2 (ack EA90=0xA5).

e4a6 (CODE_BANK1::e4a6) EA81 RANGE-GATE first:
  e4a6 MOV DPTR,#0xEA81; e4a9 MOVX A; e4aa MOV 0x4D,A (IRAM 0x4D := EA81); e4ac CLR CY; e4ad SUBB A,#0x50; e4af JNC e4c3 (EA81>=0x50 -> do reset); e4b1 MOV A,0x4D; e4b3 SUBB A,#0x51; e4b5 JC e4c3 (EA81<0x51 ... combined: only EA81 in [0x50,0x50] i.e. ==0x50? the two-sided check passes the reset for valid range; out-of-range path:) e4b7 MOV R3,#0xFF R2,#0x51 R1,#0x72; e4bd LCALL 0x538D (UART/log emit of a [..] string at 0x5172); e4c0 MOV R7,#0x00; e4c2 RET (R7=0 -> c163 takes JZ c1ae, no ack — invalid sub-op).

RESET BODY @e4c3 (EA81 valid):
  e4c3 MOV R5,#0xE7 R4,#0x03 R7,#0x04; e4c9 LCALL 0x051B (log [PcieTunnel-Reset]-class string @03E7); e4cc MOV R7,#0x0F; e4ce LCALL 0x0471 (delay/util).
  C656 &= ~0x20:  e4d1 MOV DPTR,#0xC656 (REG_HDDPC_CTRL); e4d4 MOVX A; e4d5 ANL A,#0xDF; e4d7 MOVX @DPTR,A.
  CA06 &= ~0x01:  e4d8 MOV DPTR,#0xCA06 (REG_CPU_MODE_NEXT); e4db MOVX A; e4dc ANL A,#0xFE; e4de MOVX @DPTR,A.
  CC31.0 = 1 (assert tunnel/CPU reset bit, REG_CPU_RESET): e4df MOV DPTR,#0xCC31; e4e2 MOVX A; e4e3 ANL A,#0xFE; e4e5 ORL A,#0x01; e4e7 MOVX @DPTR,A.
  SPIN-WAIT: e4e8 SJMP e4e8 (`80 fe`, jump-to-self). This is the hardware-reset spin: after CC31.0=1 the tunnel/CPU subsystem resets; execution leaves this spin only via the HW reset/event (in the listing it is an infinite self-loop). For the port this means: assert CC31.0 then BUSY-SPIN until the reset HW releases.

POST-RESET DE-ASSERT continuation @e4ea (the block after the spin — the link bring-back-up sequence):
  e4ea CLR A; e4eb MOV DPTR,#0x0B2F; e4ee MOVX @DPTR,A (u4_reinit_pending := 0).
  C659.0 check (REG_PCIE_LANE_CTRL_C659): e4ef MOV DPTR,#0xC659; e4f2 MOVX A; e4f3 JB 0xE0,e506 (if C659.0 set, skip the PERST re-drive).
  e4f6 MOV R5,#0x81 R7,#0x01; e4fa LCALL 0xE26A; e4fd LCALL 0xD185; e500 MOVX @DPTR,A (PERST/PHY re-drive helpers e26a + d185); e501 MOV R7,#0x01; e503 LCALL 0xEEC7.
  eec7 (PERST-DEASSERT / [PcieTunnel-Deassert]): eec7 LCALL 0x0476 (e8d9-class: drives C659.0 / PERST de-assert); eeca LCALL 0xEF03 (EF03: R3=FF R2=37 R1=C3 LJMP 538D = print the [PcieTunnel-Deassert] string @0x37C3); eecd MOV R3,#0xFF R2,#0x37 R1,#0xEC; eed3 LJMP 0x538D (print string @0x37EC).
  @e506: e506 MOV R7,#0x03; e508 LCALL 0x05B1.
  B480 &= ~0x0F (clear PERST bits0-3, REG_PCIE_PERST_CTRL, done as 4 separate read-ANL-write): e50b MOV DPTR,#0xB480; e50e MOVX A; e50f ANL A,#0xFE; e511 MOVX@; e512 MOVX A; e513 ANL A,#0xFD; e515 MOVX@; e516 MOVX A; e517 ANL A,#0xFB; e519 MOVX@; e51a MOVX A; e51b ANL A,#0xF7; e51d MOVX@ (net B480 &= 0xF0).
  e51e LCALL 0xE9B5; e521 LCALL 0xEF03 (print [PcieTunnel-Deassert] again); e524 MOV R3,#0xFF R2,#0x37 R1,#0xF6; e52a LJMP 0x538D (print [PcieTunnel-Enable] string @0x37F6). Later (e53b..) it polls XDATA[0x09FA].1 and re-runs b7a4/eb0a connect setup, then CA60 &= ~0x08 (e55b..e561), reading 0x0AF1 etc.

For the PORT, the load-bearing register writes in execution order: C656&=~0x20; CA06&=~0x01; CC31|=0x01 (assert)+busy-spin; 0x0B2F=0; if !C659.0 { e26a/d185 PERST re-drive; eec7 PERST-deassert (drive C659.0 + print [PcieTunnel-Deassert]) }; B480&=~0x0F (PERST bits0-3 clear); print [PcieTunnel-Enable]; re-run connect setup. R7=nonzero on success -> c165 SJMP c1a2 acks EA90=0xA5.


## Response engine (d945 read-resp / cf5d write-resp / cf35 reply-arm / ceab bounds)

RESPONSE ENGINE byte-true. Two builders share helpers ced6/cf11/cf35/cf07/cf23/cf3f and the C8B0 DMA-push.

0x0DEF (movc switch helper, used by c0a5 dispatch): POP DPH; POP DPL (DPTR := return-addr = table start); MOV R0,A (key); CLR A; MOVC A,@A+DPTR (entry byte0=DPH); JNZ 0e0a; MOV A,#1; MOVC A,@A+DPTR (byte1=DPL); JNZ 0e0a; [terminator both 0] INC DPTR; INC DPTR; MOVC A,@A+DPTR... -> at 0dff: MOVC A,@A+DPTR (A=0)->R0=DPH; MOV A,#1; MOVC->DPL; MOV DPH,R0; CLR A; JMP @A+DPTR (jump to target). Compare path 0e0a: MOV A,#2; MOVC (byte2=key); XRL A,R0; JZ 0dff (match, load target which is the SAME entry's [DPH][DPL]); INC DPTR x3; SJMP 0df4 (next entry).

cf35 (cm_routerop_arm_reply, "C805|=0x02"): MOV DPTR,#0xC805 (REG_INT_AUX_STATUS); MOVX A; ANL A,#0xF9; ORL A,#0x02; MOVX @DPTR,A; RET. => clears C805 bits1:2 then sets bit1 = the reply-arm strobe.

ced6 (clamp/limit calc): MOV DPTR,#0x0B0A; LCALL 0x0D9D (load 32b limit -> R0..R3); MOV DPTR,#0x0B04; LCALL 0x0D84 (load 32b addr -> R4..R7); LCALL 0x0CAB (subtract addr from limit = remaining); CLR A; MOV R3,#0x80; R2=R1=R0=0; CLR CY; LJMP 0x0D22 (compare remaining vs 0x00000080 = 128). => sets CY if remaining >= 0x80 (i.e. a full 128-byte chunk available). Used to pick chunk length.

cf11 (chunk length = limit-addr, capped): MOV DPTR,#0x0B0A; LCALL 0x0D84 (limit -> R4..R7); MOV R3,0x07 (low byte of limit into R3); MOV DPTR,#0x0B04; LCALL 0x0D84 (addr -> R4..R7); CLR CY; MOV A,R7; SUBB A,R3; RET. => A = (addr_lo - limit_lo) magnitude used as the partial chunk length (when < 128).

d945 (cm_routerop_send_READ_resp), exact:
  d945 MOV 0x4E,#0x01 (IRAM 0x4E := 1, the "is-read" flag).
  d948 LCALL ced6; d94b JNC d952 (remaining < 0x80 -> partial); d94d MOV 0x4D,#0x80 (full 128-byte chunk, 0x4D=len); d950 SJMP d957.
  d952 LCALL cf11; d955 MOV 0x4D,A (partial chunk length).
  d957 LCALL cf35 (ARM reply: C805|=0x02).
  d95a MOV DPTR,#0x0004; d95d MOVX A (read the EA81.0 direction flag stored by ceef); d95e MOV DPTR,#0x0B0A; d961 JZ d96b (flag==0 -> add R5 only); else d963 LCALL cf23 (cf23: load 0x0B0A 32b + adds, returns R6/R7 advanced); d966 MOV A,R5; d967 ADDC A,#0x02; d969 SJMP d970; d96b LCALL cf23; d96e CLR A; d96f ADDC A,R5; d970 MOV R5,A; d971 CLR A; d972 ADDC A,R4; d973 MOV R4,A (advance the limit/cursor 0x0B0A by chunk, +2 header offset on read).
  d974 MOV DPTR,#0x0AAD; d977 LCALL 0x0DC5 (store advanced R4..R7 -> XDATA[0x0AAD..] response descriptor area).
  HEADER BUILD: d97a MOV DPTR,#0x0AB1; d97d CLR A; d97e MOVX @DPTR,A (0x0AB1 := 0 status); d97f INC DPTR; d980 MOV A,0x4D; d982 MOVX @DPTR,A (0x0AB2 := chunk length); d983 MOV R5,#0x03 R7,#0x03; d987 LCALL 0x04C1 (response-code = 3 = "read response" path); d98a MOV DPTR,#0x0B0A; d98d LCALL 0x0D9D (reload limit); d990 MOV R7,0x4D; d992 LCALL cf3f (cf3f: CLR R4/R5/R6; LCALL 0x0C9E (mul/scale by R7=len); MOV DPTR,#0x0B0A; LJMP 0x0DC5 — store the new 0x0B0A limit advanced by len).
  C8B0 DMA-PUSH ARM (the "C8B0<-0xEA" reply): d995 MOV DPTR,#0xC8B0 (REG_DMA_MODE); d998 MOV A,#0x70; MOVX@ (C8B0=0x70); INC DPTR; CLR A; MOVX@ (C8B1=0x00); INC DPTR; MOV A,#0xEA; MOVX@ (C8B2=0xEA = the source/dest page = the EA80 mailbox window); INC DPTR; CLR A; MOVX@ (C8B3=0x00). Then length: d9a5 MOV A,0x4D; ADD A,#0xFF (len-1); MOV R6,A; CLR A; ADDC A,#0xFF; INC DPTR; MOVX@ (C8B4 = len-1 hi); XCH A,R6; LCALL 0xCEC4 (CEC4: INC DPTR + store low byte of count to C8B5). 
  FIRE+WAIT: d9b3 MOV DPTR,#0xC8B8 (REG_DMA_TRIGGER); d9b6 MOVX A; d9b7 JB 0xE0,d9b3 (spin while C8B8.0 busy); d9ba RET.
  => Read response = header[status=0,len][code=3] in 0x0AB1.. then a DMA push of `len` bytes out of the EA-page (0xEA00) window, armed via C805|=2 and triggered by the C8B0/C8B8 engine.

cf5d (cm_routerop_send_WRITE_resp), exact:
  Same len calc head: MOV 0x4E,#1; LCALL ced6; JNC -> MOV 0x4D,#0x80 / else LCALL cf11 MOV 0x4D,A; LCALL cf35 (arm C805|=2).
  cf72 LCALL 0xCF07 (cf07 = MOV A,#0xEA; ... loads the EA page constant).
  WRITE the C8B2/C8B3 destination from 0x0B08/0x0B09 cursor: cf75 MOV DPTR,#0x0B09; MOVX A; ADD A,#0x00; MOV R6,A; MOV DPTR,#0x0B08; MOVX A; ADDC A,#0x70 (add 0x70 page base to cursor); MOV DPTR,#0xC8B2; MOVX@ (C8B2 := cursor_hi+0x70); INC DPTR; XCH A,R6; MOVX@ (C8B3 := cursor_lo). Then len-1 -> C8B4/C8B5 (cf89..cf95 mirror d9a5: A=0x4D-1 -> C8B4 hi, LCALL 0xCEC5 stores lo). FIRE+WAIT: cf98 MOV DPTR,#0xC8B8; MOVX A; JB 0xE0,cf98 (spin busy).
  POST: cf9f advances 0x0B08 cursor by 0x4D (LCALL 0x0C64 add), advances 0x0B0A limit (0x0D9D + cf3f), then checks 0x0B08==0x02 && 0x0B09==0 (cfb4 XRL #2 / INC / JZ) -> if matched LCALL ceab; then drains the 4-byte accumulator at 0x0B0E into XDATA[0x0AA1/0x0AA2] (cfd1..cfdf) and runs the r3-write dispatch (0x04C6) to actually commit the written word, reloads, and on completion zeroes 0x0B08/0x0B09 (cfff..d005). d006 MOV R7,0x4E; d008 RET (R7 = status/continue, consumed by c141/c194).

CONTINUATION LOGIC (0x0B04 addr vs 0x0B0A limit): ceab does the 32-bit compare (addr R4..R7 from 0x0B04 vs limit R0..R3 from 0x0B0A via 0x0D22 unsigned compare, CLR CY). Z (JZ) => addr has reached/passed limit => transfer DONE => leave/clear state. NZ => bytes remain => set 0x0B02=1 (E2) or =2 (E3) so the next EA90==0x5A re-entry resumes via c167/c184 and pushes the next <=128-byte chunk. Each chunk re-arms C805|=2 and re-fires C8B0/C8B8. The header response-code is 3 for both read and write responses (LCALL 0x04C1 with R5=R7=3).


## E0/E1/E4/E5 + default

Remaining table entries (enough to stub/route so the movc table is complete):

E0 @c0de (key 0xE0): c0de LCALL 0xDC4E; c0e1 MOV A,R7; c0e2 JZ c0e7 (R7==0 -> RET); c0e4 LJMP c165 (R7!=0 -> SJMP c1a2 ack); c0e7 RET. dc4e CONTRACT (FUN_CODE_BANK1__dc4e): copies XDATA[0xEA82] -> XDATA[0x0213] (identity/scratch); checks cm_routerop_is_read_opcode + EA81==0x51; if XDATA[0x0213] in range, stores EA81.0 -> XDATA[0x0004], LCALL cf35 (arm C805|=2); then branches on u4_routerop_mbox_opcode: if opcode==0xE0 (ROUTEROP_PATH_E0) it does a DMA fill (cf09(0x70)/cebb(0xEA,..)/spin on REG_DMA_TRIGGER&1) = pushes the identity/descriptor block out the EA window; else it does cf07/cebb(0x70..)/spin and a bank0 r-read stub (04a3). Returns R7=status. => E0 is a "router identity/descriptor fill" op (no 0x0B02 state change; single-shot).

E1 @c0e8 (key 0xE1): c0e8 LCALL 0xDC4E; c0eb MOV A,R7; c0ec JNZ c165 (R7!=0 -> SJMP c1a2 ack); c0ee RET. Same dc4e callee, different success polarity (acks on nonzero). E1 = the read-direction variant of the same identity/descriptor op.

E4 @c151 (key 0xE4): c151 LCALL 0xD6DC; c154 MOV A,R7; c155 JNZ c180 (->SJMP c1a2 ack); c157 RET. d6dc CONTRACT (block-cfg loop): IRAM 0x4F=EA81, 0x4E=EA82(count), 0x50=cf53() (a base byte), 0x52/0x53=params. Validates 0x4F<0x81 (count<=0x80) && (EA82==0x50 || EA82==0x51). Loops 0x4D=0..count: if EA81==0x51 (write) -> add_u32(...) + r3_write_dispatch; else (read) -> reads *(addr+i); stores byte -> XDATA[0xEA00+0x4D] (the EA mailbox window). Returns 0x4D-0x4F (0 when full count done). => E4 = "block config read/write of up to 128 bytes through the EA00 mailbox window" (no 0x0B02 state).

E5 @c158 (key 0xE5): c158 LCALL 0xE21B; c15b MOV A,R7; c15c JNZ c180 (ack); c15e RET. e21b CONTRACT: 0x4D=EA82, 0x4E=cf53(), params 0x50/0x51. Requires EA82==0x50(read) or 0x51(write). If write (0x51): UVar=EA81; r3_write_dispatch(...) (writes a single cfg byte/word). If read (0x50): *(addr)=EA81. => E5 = single-location cfg op (one access, immediate).

DEFAULT @c1ae (table terminator 00 00 -> c1ae): just RET. Unknown/unhandled opcode is silently ignored (NO EA90=0xA5 ack written, so host sees the op un-consumed). For the port, route any opcode not in {E0,E1,E2,E3,E4,E5,E8} here = no-op return.

Note all of E0/E1/E4/E5 are SINGLE-SHOT (they never set 0x0B02 to 1/2); only E2 (->1) and E3 (->2) arm the multi-packet continuation handled at c167/c184.


## XDATA/SFR working-buffer map + SDCC hazards

XDATA WORKING-BUFFER LAYOUT (router-op mailbox), byte-true:
 0x0B02 = u4_routerop_mbox_state (rmbox_state_t): 0=RMBOX_IDLE/single-pkt, 1=RMBOX_MULTIPKT_1 (E2 read continuation), 2=RMBOX_MULTIPKT_2 (E3 write continuation). [handmade usb4_state.h:270]
 0x0B03 = u4_routerop_mbox_opcode: latched copy of EA80 (E0/E1/E2/E3/E4/E5/E8). [usb4_state.h:271]
 0x0B04..0x0B07 = 32/64-bit transfer ADDRESS (current cursor), loaded from EA82..EA85 by ceef; advanced per chunk. (0x0D84 loads 4 bytes -> R4..R7.)
 0x0B08..0x0B09 = E3 WRITE cursor (lo/hi); seeded from ceef addr-low, advanced by cf5d; zeroed on completion.
 0x0B0A..0x0B0D = transfer LIMIT (end pointer); compared against 0x0B04 by ceab (0x0D22 unsigned cmp) for done/continue; advanced by cf3f.
 0x0B0E..0x0B11 = 4-byte write ACCUMULATOR (E3); zeroed at c133-c13b; drained to 0x0AA1/0x0AA2 then committed via r3-write.
 0x0004 = EA81.0 read/write DIRECTION flag (set by ceef/dc4e ANL #1).
 0x0AA1/0x0AA2 = write-commit word staging (cf5d).
 0x0AAD.. = response descriptor staging (d945 stores advanced cursor here via 0x0DC5).
 0x0AB1 = response status byte (=0); 0x0AB2 = response chunk length (=0x4D). [d945 header build]
 0x0B2F = u4_reinit_pending (cleared by e4a6 post-reset). [usb4_state.h:288]
MAILBOX SFR FIELDS (EA8x window, the host-posted op): EA80 = opcode/path (REG_ROUTEROP_OPCODE_EA80). EA81 = cfg sub-op selector: 0x50=READ, 0x51=WRITE; bit0 = direction flag copied to XDATA[0x0004] (REG_ROUTEROP_CFG_EA81). EA82..EA85 = the 32/64-bit target address (read into R4..R7 by ceef/dc4e). EA00..EA7F = the bulk data window (the C8B0/C8B8 DMA pushes/pulls len bytes here; C8B2=0xEA page). EA90 = handshake (REG_SYS_CTRL_EA90): host writes 0x5A to arm, device writes 0xA5 when consumed.
IRAM USAGE (SDCC HAZARD): d945/cf5d/d6dc/e21b use IRAM direct addresses 0x4D (chunk len / EA81 latch), 0x4E (is-read flag / count), 0x4F/0x50/0x51/0x52/0x53 (d6dc loop scratch). MEMORY.md flags IRAM 0x21-0x7F is OWNED BY SDCC — a hand-port MUST NOT hardcode these IRAM cells; use locals/statics and let SDCC allocate, OR move them to the dense 0x0B40-0x0B5F XDATA handmade headroom. 
PORTING-HAZARD on XDATA: c0a5 uses 0x0B02-0x0B1F which is SEPARATELY ZEROED in main.c:573 (`for z 0..(0x0B1F-0x0B02): XDATA[0x0B02+z]=0`) at init — so the mailbox state buffer starts clean; the port must keep all new mailbox vars inside 0x0B02-0x0B1F (already-zeroed) OR inside the 0x0B45-0x0B58 block also zeroed at main.c:590. Do NOT alias pd_seen@0x0B45 or other dense 0x0B40-0x0B5F cells (only 0x0B53/0x0B54/0x0B56/0x0B57 are free per MEMORY.md). 0x0AA1/0x0AA2/0x0AAD/0x0AB1/0x0AB2 are in the 0x0A-page (NOT in the 0x0B02-0x0B1F zero loop) — they are written before read by the engine so no init needed, but reserve them in usb4_state.h to avoid collisions with sb_tx_* (0x0AA8-0x0AAC) and sb_fsm_state (0x0AAD!) — NOTE 0x0AAD is ALREADY declared as sb_fsm_state in handmade (usb4_state.h:260); d945 writes 0x0AAD..0x0AB0 as response-descriptor scratch -> CONFLICT to resolve: either the response builder uses a different staging area in the port, or sb_fsm_state must move. Flag this explicitly when porting d945.


## Concrete port plan (files, call-site, order, hazards)

CONCRETE PORT PLAN.

FILES: implement in handmade/src/usb4.h (the stub at usb4.h:63 cm_routerop_mailbox is what to replace) for the dispatcher/state machine, and reuse handmade/src/cm_tunnel.h for the actual config-TLP data engine. State vars already exist in handmade/src/usb4_state.h (0x0B02 u4_routerop_mbox_state, 0x0B03 u4_routerop_mbox_opcode); add 0x0B04-0x0B11 (addr/cursor/limit/accum) and the 0x0AB1/0x0AB2 header + 0x0AAD descriptor staging there (RESOLVE the 0x0AAD collision with sb_fsm_state first — see xdata_map).

CALL-SITE: already correct — usb4.h:91 usb4_int_demux, the `REG_NVME_EVENT_STATUS & 0x01` (EC06.0) branch at line 99-107, acks REG_NVME_EVENT_ACK=1 then calls cm_routerop_mailbox(). Keep that; just flesh out cm_routerop_mailbox to the full c0a5 logic. The macro names are real: REG_SYS_CTRL_EA90 (EA90), REG_ROUTEROP_OPCODE_EA80 (EA80), REG_ROUTEROP_CFG_EA81 (EA81), REG_NVME_EVENT_STATUS/ACK (EC06/EC04), REG_INT_AUX_STATUS (C805), REG_DMA_MODE (C8B0)/REG_DMA_TRIGGER (C8B8), REG_HDDPC_CTRL (C656), REG_CPU_MODE_NEXT (CA06), REG_CPU_RESET (CC31), REG_PCIE_LANE_CTRL_C659 (C659), REG_PCIE_PERST_CTRL (B480), REG_CPU_CTRL_CA60 (CA60). EA82.. and the EA00 data window need new macros (REG_ROUTEROP_ADDR_EA82, plus a `#define EA_WINDOW(i) XDATA_REG8V(0xEA00+(i))`).

IMPLEMENTATION ORDER:
 (1) c0a5 SKELETON: EA90==0x5A gate; read 0x0B02; if 0 latch EA80->0x0B03 and dispatch by opcode (a C switch on 0x0B03 replacing the movc table — keys E0/E1/E2/E3/E4/E5/E8, default=no-op); else run the state-1/state-2 continuation (c167/c184). Always finish via the epilogue EA90=0xA5 (except the c1a9 abort path which leaves no ack). This makes the host see ops consumed.
 (2) E2/E3 FIRST (config transport — this is the path the host actually drives for cfg read/write over the tunnel): port ceef (addr-copy EA82..->0x0B04, EA81.0->0x0004), the cf4c EA81 50/51 gate, d945 read-resp / cf5d write-resp, ceab bounds, and set 0x0B02=1 (E2) / 2 (E3). For the actual data movement REUSE the existing handmade adb0 engine in cm_tunnel.h (cm_adb0_tlp / cm_e89d_read / cm_e91d_write, mailbox CM_M(i)=0x0A70+i, data at 0xB220-0xB223): where d945/cf5d push a cfg word, route it through cm_adb0_tlp so the response data overlaps the already-validated config-TLP path instead of re-porting the raw C8B0 DMA-to-EA-window plumbing. (Only port the literal C8B0/C8B8 push if the EA00 mailbox semantics turn out to differ from the B220 engine.)
 (3) E8 tunnel reset (e4a6): port C656&=~0x20, CA06&=~1, CC31|=1 + busy-spin, then the post-reset de-assert (0x0B2F=0, C659.0 check, eec7 PERST-deassert via C659 + [PcieTunnel-Deassert], B480&=~0x0F, [PcieTunnel-Enable]). This is the op that finally posts PcieTunnel-Deassert which MEMORY.md identifies as the wall (host never gets PCIeDn enabled). Implement E8 right after E2/E3 since the captured blocker is "host never posts 0xE8" — wiring a correct 0xE8 handler that drives B480/C659 PERST-deassert is the GPU-enable lever.
 (4) E0/E1/E4/E5 as thin single-shot handlers (dc4e/d6dc/e21b) or initially route them to a logged no-op that still acks EA90=0xA5 (so the host doesn't stall on an un-consumed op), then fill in if the host actually posts them.

HAZARDS: keep all new mailbox XDATA inside the main.c:573 zeroed 0x0B02-0x0B1F block; do NOT use raw IRAM 0x4D-0x53 (SDCC-owned) — use C locals/statics. Resolve the 0x0AAD sb_fsm_state vs d945 descriptor-staging overlap before enabling d945.

