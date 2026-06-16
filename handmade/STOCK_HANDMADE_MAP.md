# Stock Ghidra <-> Handmade function map

Generated from `handmade/stock_ghidra_export.c` (full CppExporter decompile, 1764 fns) by workflow wwjy6u6t2.
Each handmade function -> its stock code address + export symbol. Diff: open the export at the symbol, compare to the handmade file:line.

| handmade func | file:line | stock addr | export symbol | covers | role |
|---|---|---|---|---|---|
| `phy_cc11_ack` | boot_phy.h:9 | cc11 | phy_cc11_ack_event | cc11 | W1C the PHY/TIMER0 (CC11) command-complete event: REG_TIMER0_CSR=4 then =2 |
| `phy_cc10_cmd` | boot_phy.h:15 | cc10 | phy_link_train_cmd_cc10 | cc11,cc10,cc12,cc13 | Issue a PHY/SERDES mailbox command: ack, load subcmd into CC10 + args into CC12/CC13, set  |
| `phy_cc10_cmd_wait` | boot_phy.h:24 | cc10 | phy_cmd_cc10_and_wait | cc10,cc11 | Issue a CC10 PHY command then busy-poll CC11.1 for completion and W1C-ack it (also used as |
| `boot_phy_d118` | boot_phy.h:31 | d118 | FUN_CODE_d118 | d118 | Write LTSSM ctrl, run a PHY settle command (cc10 0,0), return the LTSSM ctrl readback |
| `boot_phy_d0d3_typec_sbu` | boot_phy.h:37 | d0d3 | boot_phy_d0d3_typec_sbu | d0d3,d118 | Type-C SBU PHY bring-up: LTSSM ctrl bit dance + cc10 settle commands powering the SBU side |
| `boot_phy_bceb_set0` | boot_phy.h:49 | bceb | FUN_CODE_bceb | bceb | Set bit0 of an XDATA register (RMW: (val & 0xFE) / 0x01) |
| `boot_phy_cf28` | boot_phy.h:53 | cf28 | boot_phy_cf28_phy_config | cf28,bceb | PHY config block: program CC3x/E3xx/E71x link-width/ctrl/status registers via bceb bit-set |
| `boot_phy_bank1_ed02` | boot_phy.h:73 | ed02 | boot_phy_bank1_ed02_sb_enable | ed02 | Early sideband-block enable: CC37 bit, SB[0x05] bit7 (the SB-block power bit), CA70/E780 c |
| `boot_phy_dd42` | boot_phy.h:82 | dd42 | FUN_CODE_dd42 | dd42 | Program the E7E3 PHY config latch (REG_PHY_LINK_CTRL) from a mode selector (0/2->00,4->30, |
| `boot_phy_e57d_e764_reset_pulse` | boot_phy.h:95 | e57d | pcie_tunnel_reset_pulse_e764 | e57d,e764 | PCIe-tunnel PHY timer reset pulse: E764 &=~2,&=~1,&=~8,(&~4)/4 when enable&1 (the e57d res |
| `boot_phy_d630_lane_power` | boot_phy.h:104 | d630 | pcie_tunnel_lane_power_b432_b404 | d630 | Power up PCIe-tunnel lanes: B432[2:0]=7, B404 low nibble=enable, latch E77C PLL when enabl |
| `boot_phy_d436_width` | boot_phy.h:112 | d436 | pcie_tunnel_link_width_config_b434_b436 | d436 | Program PCIe-tunnel lane width: write B434=width, B436 low nibble=width&0xF |
| `boot_phy_d996_pcie_tunnel_boot` | boot_phy.h:117 | d996 | pcie_tunnel_bringup_boot_seq | d996,e57d,d630,d436 | Boot pre-staging of downstream PCIe tunnel: B402/C659 clears, e57d reset pulse, d630 lane  |
| `boot_phy_bringup_early` | boot_phy.h:126 | ce79 | boot_phy_bringup_early | ce79,d0d3,cf28,ed02,dd42,d996 | Full early PHY bring-up (stock's first main() action): optional d0d3, cf28, ed02 SB-enable |
| `bank0_92c5_seed` | boot_phy.h:146 | 92c5 | bank0_92c5 | 92c5 | Seed lane-engine link WIDTH/MODE/state RAM (XDATA 0x0AEx etc.) the USB4 tunnel reads on th |
| `int0_isr` | main.c:373 | 0003 |  | 91D1,9302,9300,9301 | USB peripheral interrupt (8051 vector 0x0003): bus-reset/control/link-event/bulk demux. St |
| `int1_isr` | main.c:461 | 0013 | int1_isr_orchestrator | B4BA,AF5E,A066,C105,C0A5,E911 | INT1 ISR (8051 vector 0x0013) PD/USB4/system demux: C806.0->timer-tick, C80A.6->pd_rx, (0x |
| `pd_da51` | pd.h:29 | da51 | FUN_CODE_da51 | da51 | Programs PD-engine RDO/CRC timing constants E401/E406/E407/E408 (gated on CMD_CONFIG.7). C |
| `cc_pd_phy_term_init` | pd.h:42 | ae87 | cc_pd_phy_term_init | ae87 | PD engine + CC PHY ARM/termination init so the host sees a sink attach; configures the E4x |
| `cc_ctrl_enable_events` | pd.h:81 | e330 | cc_ctrl_enable_cc81_cc98_events | e330 | Clears/enables the CC attach/role event sources (CC81/CC98/CC80, INT_ENABLE.4). Called by  |
| `pd_internal_state_init` | pd.h:93 | b8c3 | pd_internal_state_init | b8c3,e330 | Resets the PD policy-engine state block (0x07Bx-0x07Ex), sets substate=1, seeds timers, en |
| `pd_drive_hard_reset` | pd.h:120 | be8b | pd_drive_hard_reset | be8b,b8c3 | Transmits a USB-PD HARD RESET to force the host to re-send Source_Cap; no-op (only prints  |
| `pd_int1_enable_group` | pd.h:149 | 4be6 | FUN_CODE_4be6 | 4be6 | Routes the C806/C80A PD/system interrupt aggregate to the 8051 EX1 (INT1) line. Implements |
| `pd_rx_isr` | pd.h:166 | af5e | pd_int_handler | af5e,af70 | PD-RX ISR (INT1 C80A.6 path): prints [PD_int:E40F:E410], priority demux over E40F/E410 W1C |
| `cc_cc23_reinit_event` | pd.h:199 | e3d8 | cc_cc23_reinit_event | e3d8 | CC23.1 re-init / SB-reconnect event. Handmade implements the tail (u4_connect_gate_e8=0 /  |
| `cc_state_full_reset` | pd.h:205 | d676 | cc_state_full_reset | d676 | Type-C error-recovery handler. Handmade is diagnostic-print-only ([Error_Recovery]); stock |
| `pd_cc81_hard_reset_4` | pd.h:210 | e90b | pd_cc81_hard_reset_4 | e90b,be8b | CC81/=4 (REG_CPU_INT_CTRL=4) then pd_drive_hard_reset. Called from the cc_pd_timer_tick CC |
| `cc_cc99_default_event` | pd.h:222 | e883 | FUN_CODE_e883 | e883 | CC99.1 default branch. Empty stub in handmade; stock FUN_CODE_e883 clears state buf 0x20-0 |
| `cc_ccf9_subdemux` | pd.h:226 | df79 | FUN_CODE_df79 | df79 | CCF9.1 sub-demux: copies u4_routerop_desc0 (0x0A9D) from cc_subdemux_src (0x0B1B), matchin |
| `cc_pd_timer_tick` | pd.h:235 | b4ba | cc_pd_timer_tick | b4ba | INT1 timer-tick PD/USB4 policy-engine tick (C806.0 via stub 0x0520). Services the 6 CC per |
| `pd_rx_ptr` | pd_dispatch.h:15 | 96d4 | FUN_CODE_96d4 | 96d4 | Compute RX buffer base 0xE440+0x20*slot and stash it into pd_rx_ptr_hi/lo (xdata 0x07BF/0x |
| `pd_rx_ptr_get` | pd_dispatch.h:23 | 96e1 | FUN_CODE_96e1 | 96e1 | Re-read the stashed 16-bit RX buffer pointer from xdata 0x07BF/0x07C0 |
| `pd_tx_commit_engine` | pd_dispatch.h:28 | e1c6 | pd_tx_commit_engine | e1c6,e09a | Wait for the PD TX engine idle, load E403 msg-len, kick E401 busy bit, bump TX MessageID c |
| `pd_cc_event_clear` | pd_dispatch.h:38 | 95c2 | reg_cpu_int_ctrl_4_2 | 95c2 | W1C-clear an arbitrary CC event register by writing 4 then 2 (parameterized form) |
| `pd_e933_clear_cc81` | pd_dispatch.h:43 | e933 | reg_cpu_int_ctrl_4_2 | e933,95c2 | W1C-clear the CC81 attach-event register (hardcoded-CC81 wrapper around 95c2) |
| `pd_arm_cc_timer` | pd_dispatch.h:46 | e81b | FUN_CODE_e81b | e81b | Arm the CC sender-response/PS-transition timer: write CC82/CC83 thresholds, clear CC81, se |
| `pd_tx_set_sop_header` | pd_dispatch.h:54 | dd12 | pd_tx_set_sop_header | dd12 | Build the PD TX header in E420/E421 (SOP type via E41C trigger, NumDataObjects, MessageID, |
| `pd_tx_buf_clear` | pd_dispatch.h:63 | e73a | pd_clear_state_buf_20_3f | e73a | Zero the 0x20-byte PD TX message buffer E420-E43F |
| `pd_select_pdo_from_source_cap` | pd_dispatch.h:69 | abf5 | pd_select_pdo_from_source_cap | abf5 | Select PDO[0] (vSafe5V fixed supply) from the received Source_Cap, latch operating current |
| `pd_build_send_request_rdo` | pd_dispatch.h:78 | acd4 | pd_build_send_request_rdo | acd4 | Build a Request (header + Fixed RDO) into E420+, run the Timer0 tCCDebounce wait, commit T |
| `pd_ctrl_goodcrc` | pd_dispatch.h:115 | 870b | sw8507_case_01 | 870b,96bf,e1c6 | GoodCRC handler (ctrl msgtype 0x01): advance RX slot index (96bf) and commit any staged pe |
| `pd_ctrl_accept` | pd_dispatch.h:121 | 8550 |  | 8550,e933,e81b,870b | Accept handler (ctrl msgtype 0x03, 8507-table case 3): on substate 3 clear CC81, set subst |
| `pd_ctrl_ps_rdy` | pd_dispatch.h:148 | 8591 |  | 8591,e933,870b | PS_RDY handler (ctrl msgtype 0x06, 8507-table case 6): decode contract voltage (0x12C/0x96 |
| `pd_ctrl_reject` | pd_dispatch.h:176 | 8641 |  | 8641,e933 | Reject handler (ctrl msgtype 0x04, 8507-table case 4): on substate 3 clear CC81/arm timer, |
| `pd_ctrl_wait` | pd_dispatch.h:197 | 860e |  | 860e,e933,e81b,870b | Wait handler (ctrl msgtype 0x0C, 8507-table case 0x0c): on substate 3 with active contract |
| `pd_rx_nak_send` | pd_dispatch.h:211 | 871a | sw8507_default | 871a,9664,965d | Default/NAK path (8507-table default): stage a 2-byte control NAK response and advance the |
| `pd_ctrl_soft_reset` | pd_dispatch.h:223 | dc2d | soft_rst | dc2d,8717 | Soft_Reset handler (ctrl msgtype 0x0D): reset both MessageID counters, clear CC81, reply A |
| `pd_dispatch_control` | pd_dispatch.h:242 | 8507 | c51_switch_dispatch | 8507,0def,870b,8550,8591,8641 | CONTROL-message switch by MessageType -> per-case handlers; stock = the 0def keyed jump-ta |
| `pd_dispatch_data` | pd_dispatch.h:257 | 83d6 | pd_rx_message_dispatch | 83d6,8406,dc65,a036,9ac4 | DATA-message branch by MessageType (Source_Cap 0x01, Request 0x03->dc65, Enter_USB 0x08->a |
| `pd_rx_message_dispatch` | pd_dispatch.h:303 | 83d6 | pd_rx_message_dispatch | 83d6,8406,96d4,96e1 | PD RX entry: gate on E40F, parse RX header (NumDataObj/MsgType/SOP) from the slot buffer,  |
| `sb_lane_flip_init` | sb.h:52 | a3f5 | sb_lane_flip_init | a3f5 | connect orientation/lane-flip map + SB-PHY-RX descriptor + mailbox strobe (usb4_connect_u4 |
| `sb_rom_descriptor_load` | sb.h:138 | b7a4 | sb_lane_descriptor_loader | b7a4 | copies router DROM identity (CODE 0x213d[0x64]) + lane descriptor (CODE 0x21d4[0x10]) into |
| `sb_block_init` | sb.h:174 | bb37 | sb_block_init | bb37 | static SB-block register blast (page1 0x2800+off) + PHY descriptor seed + PHY-reg RMW (C23 |
| `sb_pcie_width_ramp` | sb.h:248 | b434 | pcie_tunnel_link_width_config_b434_b436 | b434,b435,b436,b437 | program the PCIe tunnel link width across 4 lanes (B434/B435/B436/B437) |
| `eng_a30c` | sb.h:256 | a30c | FUN_CODE_a30c | a30c | descriptor-engine primitive: write ENGINE[cur]=v then RMW ENGINE[cur+1] (&0x3F)/0x80 |
| `eng_a308` | sb.h:259 | a308 | FUN_CODE_a308 | a308 | descriptor-engine primitive: write ENGINE[cur]=(v&0xF0)/0x0F then RMW ENGINE[cur+1] (&0x3F |
| `eng_a2df` | sb.h:263 | a2df | FUN_CODE_a2df | a2df | descriptor-engine primitive: write ENGINE[cur]=v then RMW ENGINE[cur+1] &0xE0 |
| `eng_a31c` | sb.h:266 | a31c | FUN_CODE_a31c | a31c | descriptor-engine primitive: write ENGINE[cur]=v then two-step RMW ENGINE[cur+1] (&0xC0)/0 |
| `eng_a348` | sb.h:271 | a348 | FUN_CODE_a348 | a348 | descriptor-engine primitive: write ENGINE[cur]=v then dummy read ENGINE[cur+1] |
| `eng_a327` | sb.h:274 | a327 | FUN_CODE_a327 | a327 | descriptor-engine primitive: write ENGINE[cur]=v then RMW ENGINE[cur] (&0x3F)/0x40 |
| `u4c_sb_desc_commit` | sb.h:279 | e7fb | bank0_e7fb | e7fb,e83d,e711 | commit staged descriptor: write ENGINE[0x37] GO bit7, kick 0x38, bounded poll 0x38.0, then |
| `u4c_edbd` | sb.h:292 | edbd | bank1_edbd | edbd | SB[0x1C].0 = !connect (set when no Enter_USB accept, else clear) |
| `u4c_e5b0` | sb.h:298 | e5b0 | bank1_e5b0 | e5b0 | descriptor-engine pre-config before ccb3/c270 program descriptors (clears scratch, primes  |
| `u4c_ccb3` | sb.h:310 | ccb3 | FUN_CODE_ccb3 | ccb3 | lane-config descriptor: main descriptor + two 0x09FB-gated sub-descriptors (uses eng_a30c/ |
| `u4c_c270` | sb.h:336 | c270 | FUN_CODE_c270 | c270 | DROM PID descriptors: emits three descriptors the host CM reads back (PID/bcd payloads via |
| `u4c_d556` | sb.h:364 | d556 | bank1_d556 | d556 | per-route descriptor latch: route-mode-gated PR(0x0250/0x0251)=0x02/0xC3 and mode-gated PR |
| `u4c_bcd7_tail` | sb.h:374 | bcd7 | FUN_CODE_bcd7 | bcd7 | tunnel/lane-rate train tail of usb4_connect_u4 (FUN_CODE_bcd7()!=0 branch): width ramp + b |
| `sb_assert` | sb.h:394 | a3f5 | usb4_connect_u4 | a3f5 | SB-assert entry / USB4 connect handler: orchestrates edbd, e5b0, dd42(boot_phy), bcd7 tail |
| `sb_eaac_populate_0777` | sb_router.h:37 | eaac | FUN_CODE_BANK1__eaac | eaac | Copy host connect descriptor from SB-plane-2 (0x2a00/0x2b00) into the 0x0777 block the sta |
| `sb_af38_descriptor_response` | sb_router.h:67 | af38 | FUN_CODE_BANK1__af38 | af38 | Read host connect descriptor from RX plane, build device->host TX response into 0x2900 pla |
| `sb_set_connect_present_ebb5` | sb_router.h:151 | ebb5 | FUN_CODE_BANK1__ebb5 | ebb5 | The 0x0765 connect-present setter: SB[0x57]/=0x08, SB[0x61]/=0x08 when descriptor nibble!= |
| `sb_edd9_receive_ack` | sb_router.h:160 | edd9 | FUN_CODE_BANK1__edd9 | edd9 | cd3f's first action on every transport edge: device->host receive-ACK (page1 0x0109 W1C, S |
| `sb_cd3f_dispatch` | sb_router.h:172 | cd3f | FUN_CODE_BANK1__cd3f | cd3f,edd9,ebb5,eaac,af38 | Read host descriptor and run the connect dispatch: edd9 ack then branch to ebb5/eaac/af38  |
| `sb_d4cd_transport_edges` | sb_router.h:214 | d4cd | sb_transport_substate_poll | d4cd,cd3f | Per-edge transport/link dispatch: poll SB[0x28]/[0x2A]/[0x81]/[0x83].3, set 0x06F0 port, c |
| `sb_db7a_route_arm` | sb_router.h:253 | db7a | FUN_CODE_BANK1__db7a | db7a | Post-connect tunnel-route arm; branches on 0x07B9 (Connect_U4 vs EnterMode-TBT), drives CA |
| `sb_con_consequence` | sb_router.h:272 | dea1 | FUN_CODE_BANK1__dea1 | dea1,db7a | Post-connect consequence: heavy SB/PHY arm once per session (gated 0x06EC), arm lane-bond  |
| `sb_lane_bonded_consequence` | sb_router.h:308 | eed6 | FUN_CODE_BANK1__eed6 | eed6 | Post-[Lane Bonded] consequence: 0x072D=1, SB[0xC9]=0xFF, page1 0x01C8 confirm |
| `sb_lane_bond_complete_tunnel_up` | sb_router.h:316 | e52d | sb_lane_bond_complete_tunnel_up | e52d | Lane-bond complete -> tunnel up -> downstream PCIe bring-up (deferred via sb_tunnel_up_pen |
| `sb_channel_connect_service` | sb_router.h:326 | c3b2 | sb_channel_connect_service | c3b2,edd9 | Per-active-port connect descriptor read+dispatch: select SB reg pair by 0x06F1, validate ~ |
| `sb_a5d8_tx` | sb_router.h:376 | e1cb | FUN_CODE_BANK1__e1cb | e1cb,e2b9 | SB-transport TX answer push extracted from the a5d8 responder tail: is_e1cb=1 -> READ/rout |
| `sb_a5d8_pend_int` | sb_router.h:400 | a5d8 | FUN_CODE_BANK1__a5d8 | a5d8,e1cb,e2b9 | The [Pend Int] device->host router-op responder: parse router-op header, opcode 1/2 dispat |
| `sb_cdf5_routerop_response` | sb_router.h:508 | cdf5 | FUN_CODE_BANK1__cdf5 | cdf5 | Deferred device->host router-op CONFIG-READ response (cdf5..cea9): build lane-config respo |
| `sb_router_event_handler` | sb_router.h:543 | a066 | sb_router_event_handler_M2 | a066,c3b2,d4cd,e52d,dea1,eed6 | INT1 source C80A.5 service body: PART1 per-channel connect poll (c3b2 service + c9 ack + e |
| `sb_cb10_lane_advance` | sb_router.h:665 | cb10 | bank1_cb10 | cb10,cdf5 | Per-super-loop SB lane-bond advance: read SB[0xA0]/[0xA1] low nibble, compare vs 0x072B/0x |
| `u4c_e0d9` | usb4.h:9 | e0d9 | bank1_e0d9_stub_057a | e0d9 | PHY descriptor seed: mode==4 writes the PHY RXPLL/CDR trim registers (C20E-C217). Port of  |
| `u4c_e7c1` | usb4.h:17 | e7c1 | bank0_e7c1 | e7c1 | Timer-enable gate: mode==1 clears bit1 of REG_TIMER_ENABLE_A/B; else conditionally sets bi |
| `usb4_connect_u4` | usb4.h:23 | a3f5 | usb4_connect_u4 | a3f5,dd42,e7c1,e0d9 | Post-Enter_USB connect path: drives E716/CA81/CPU_MODE link bring-up, calls boot_phy(dd42) |
| `cm_routerop_mailbox` | usb4.h:63 | c0a5 | cm_routerop_mailbox | c0a5 | Config-space USB4 router-op mailbox dispatcher: gated EA90==0x5A, multi-packet state machi |
| `u4c_bd2a` | usb4_connect.h:14 | bd2a | FUN_CODE_bd2a | bd2a | RMW helper: clears bits 5 and 6 of a register (PR(a) &= 0xDF; &= 0xBF). Named for stock mi |
| `u4c_bcf2` | usb4_connect.h:15 | bcf2 | FUN_CODE_bcf2 | bcf2 | RMW helper: sets bit1 of REG_TIMER_ENABLE_B and _A. Implements stock FUN_CODE_bcf2. |
| `u4c_bd41` | usb4_connect.h:16 | bd41 | FUN_CODE_bd41 | bd41 | RMW helper: clears bit1 of REG_TIMER_CTRL_CC3B. Implements stock FUN_CODE_bd41. |
| `u4c_bd14` | usb4_connect.h:17 | bd14 | FUN_CODE_bd14 | bd14 | RMW helper: clears bit1 of REG_TIMER_ENABLE_B and _A. Implements stock FUN_CODE_bd14. |
| `u4c_bd6c` | usb4_connect.h:20 | bd6c | FUN_CODE_bd6c | bd6c,dcd4 | Pumps the link controller (CA00/CA0A inline = stock dcd4 call) then returns the link-mode  |
| `u4c_e7ae_bounded` | usb4_connect.h:27 | e7ae | bank0_e7ae | e7ae | Bounded version of stock bank0_e7ae PHY-lock busy-wait (UART TFBF==0x10 then STATUS&7==0), |
| `bank0_8a89` | usb4_connect.h:38 | 8a89 | bank0_8a89 | 8a89,bd23,bd3a,bd65,bcfe,bceb | USB4 lane-mode bring-up engine (mode 0/1=USB3.2-tunnel/2=USB4): picks config by lane-rate  |
| `bank0_c9a8` | usb4_connect.h:193 | c9a8 | FUN_CODE_c9a8 | c9a8 | Host-link-event connect dispatcher: gates on u4_route_mode&0x04 + connect gate, then invok |
| `C335` | usb4_irq.h:38 | c335 | FUN_CODE_BANK1__c335 | c335 | PHY-config RMW helper: reg &0x0F/0xE0 and reg+1 &0x0F/0x70 (byte-exact match) |
| `C30E` | usb4_irq.h:39 | c30e | FUN_CODE_BANK1__c30e | c30e | PHY-config RMW helper: clears reg bits via &0xFE,&0xFD,&0xFB,&0xF7 (byte-exact match) |
| `C2D9` | usb4_irq.h:40 | c2d9 | FUN_CODE_BANK1__c2d9 | c2d9 | PHY-config RMW helper: reg &0x0F/0x60 and reg+1 &0xF0/0x07 (byte-exact match) |
| `C397` | usb4_irq.h:41 | c397 | FUN_CODE_BANK1__c397 | c397 | PHY-config RMW helper: reg &0xF1/0x0E and reg+1 = 0 (byte-exact match) |
| `usb4_phy_rx_descriptor_8e31` | usb4_irq.h:44 | 8e31 | FUN_CODE_BANK1__8e31 | 8e31 | full PHY-RX descriptor config: dual PHY lanes (LaneA C2xx / LaneB C3xx) + buffer descripto |
| `usb4_irq_db0d` | usb4_irq.h:211 | db0d | FUN_CODE_BANK1__db0d | db0d,c390 | PHY link/SB sideband setup producing SB[0x1C]=0xC2: C21B/0xC0, LINK_CTRL/8, page-0x1262 RM |
| `usb4_irq_ef1e` | usb4_irq.h:227 | ef1e | bank1_ef1e | ef1e,d0ac,9a63 | SB-PHY 4-lane RX arm: applies paged RX equalizer/rate RMW table (u4rx_tab) inline; stock b |
| `usb4_irq_ef24` | usb4_irq.h:236 | ef24 | bank1_ef24 | ef24,db0d,8e31 | PHY link setup + RX descriptor config; calls db0d then 8e31 exactly as stock bank1_ef24 |
| `usb4_irq_arm` | usb4_irq.h:242 | 4be6 | FUN_CODE_4be6 | 4be6,cc3b,ef24,ef1e | init_sys_flags arming tail not covered by pd_int1_enable_group: REG_CPU_EXEC_STATUS_3&=~1, |
| `usb4_routerop_init` | usb4_irq.h:253 | e56f | bank1_e56f | e56f,ec00,ea88,ea89,ec05 | USB4 CM router-op RX-enable: EC00 enable, phy_cc10_cmd_wait(0,9,0), EA88=100/EA89=0x24 spe |
| `u4lb_eb62` | usb4_lanebond.h:12 | eb62 | FUN_CODE_BANK1__eb62 | eb62 | Set the lane-bond FSM state (XDATA 0x06ED), print [SB P0<state>] |
| `u4lb_edf5_route_query` | usb4_lanebond.h:27 | edf5 | FUN_CODE_BANK1__edf5 | edf5,e2b9,d5da | State-3 device->host SB-transport route-query that prompts host to post the connection-rou |
| `u4lb_cm_conn_routing_setup` | usb4_lanebond.h:105 | a869 | FUN_CODE_BANK1__a5d8 | a869,a863,e391,c586,c2c6 | State-3 [ConnRout] connection-routing FSM: 0x0758 sub-FSM, host connect-descriptor confirm |
| `u4lb_96fe` | usb4_lanebond.h:218 | 96fe | FUN_CODE_BANK1__96fe | 96fe | Per-lane OS/CDR command-issue descriptor: SB[0x15]=op, SB[0x0C]=(.&0x80)/3 |
| `u4lb_d5da` | usb4_lanebond.h:225 | d5da | FUN_CODE_BANK1__d5da | d5da | Per-lane PHY-RX/CDR commit + settle handshake (the bounded SB-transport TX trigger) |
| `u4lb_e07d` | usb4_lanebond.h:247 | e07d | FUN_CODE_BANK1__e07d | e07d | Retrain-path per-lane PHY/SB2 lane-block program |
| `u4lb_e9e7` | usb4_lanebond.h:264 | e9e7 | bank1_e9e7_stub_0615 | e9e7 | RstRxpll: reset the RX PLL via C20E + two CC10 settles |
| `u4lb_e764_rxpll_train` | usb4_lanebond.h:281 | cdc6 | bank0_cdc6_stub_046c | cdc6 | E764 RX-PLL train: ramp E764, cc10 settle, poll E762.4 ready latch, finish or clear; re-dr |
| `u4lb_ebde` | usb4_lanebond.h:301 | ebde | FUN_CODE_BANK1__ebde | ebde | Rate-lock settle: pulse C20F then bounded-spin for C2D0.5 / C350.5 lock bits |
| `u4lb_e980` | usb4_lanebond.h:310 | e980 | FUN_CODE_BANK1__e980 | e980 | 20G rate-descriptor apply (C2A8/C328 + C2C9/C349 rate fields + START bit7) |
| `u4lb_d3b0` | usb4_lanebond.h:325 | d3b0 | FUN_CODE_BANK1__d3b0 | d3b0 | Chg2 rate setup (rate=3=20G): SB[0x65] rate bits, commit via CC10 |
| `u4lb_ec51` | usb4_lanebond.h:344 | ec51 | FUN_CODE_BANK1__ec51 | ec51 | Trig-arm: arm the lane-train trigger (CCE0-CCE3) state 5 fires as [Trig] |
| `u4lb_b226` | usb4_lanebond.h:353 | b226 | FUN_CODE_BANK1__b226 | b226 | CC10 settle wrapper (phy_cc10_cmd_wait(2,0,0xC8)) |
| `u4lb_ee57` | usb4_lanebond.h:357 | ee57 | FUN_CODE_BANK1__ee57 | ee57 | Fire ec51 Trig-arm when CCE1.0 clear or CCE1.1 set, before reading the CCE4:CCE5 lane-widt |
| `u4lb_98ec` | usb4_lanebond.h:362 | 98ec | FUN_CODE_BANK1__98ec | 98ec | Lane-width snapshot producer: arm 0x0758=0x10, run ee57, latch CCE4:CCE5 into 0x768:0x769 |
| `u4lb_d195` | usb4_lanebond.h:374 | d195 | FUN_CODE_BANK1__d195 | d195 | P1[0x7104] = (.&0xBF)/0x40 plane-2 PHY strobe |
| `u4lb_d1d3` | usb4_lanebond.h:379 | d1d3 | FUN_CODE_BANK1__d1d3 | d1d3 | Returns (P1[hi:0x8D] & 0xF3)/8 plane-2 lane-block accessor helper |
| `u4lb_df61` | usb4_lanebond.h:385 | df61 | FUN_CODE_BANK1__df61 | df61,d195,d1d3 | Plane-2 PHY lane-block program (1808/1835/7041/6043/6025/508x/520x/408D) |
| `u4lb_ed44` | usb4_lanebond.h:403 | ed44 | bank1_ed44 | ed44,df61 | B401/B402 tunnel-link strobe, then df61 |
| `u4lb_e74e` | usb4_lanebond.h:414 | e74e | bank0_e74e | e74e | 0x0B1B=0; CCF8&=~0x10; CCF9=4; CCF9=2 (cc subdemux reset). Also exists as bank0_e74e_stub_ |
| `u4lb_ee29` | usb4_lanebond.h:422 | ee29 | bank1_ee29 | ee29,ed44,e74e | C659&=~1; B402&=~1; ed44; e74e; 0x0B42=0; 0x0B43=0 |
| `u4lb_d702` | usb4_lanebond.h:432 | d702 | FUN_CODE_d702 | d702 | CC10-mailbox lane-mask bit-distributor (plane-2 0x78AF..0x7BAF slot bit7) |
| `u4lb_c089_lane_ramp` | usb4_lanebond.h:445 | c089 |  | c089,d702 | 4-round B434 lane ramp toward target with d702 + CC10 settle each round. No standalone sto |
| `u4lb_d436` | usb4_lanebond.h:468 | d436 | bank0_d436_stub_047b | d436,c089 | PCIe-tunnel link-width config: bracket B402.1, run lane ramp, strobe B401.0, set B436 |
| `u4lb_a840` | usb4_lanebond.h:485 | a840 | bank0_a840_stub_0444 | a840,d436 | PCIe link-speed/width config (B403/B431 + d436 width), gen/lane table lookups |
| `u4lb_e305` | usb4_lanebond.h:540 | e305 | FUN_CODE_BANK1__e305 | e305,ee29,a840 | State-4 PcieTunnel power-on prologue: CA06 mode-next select, ee29, B402&=~2, a840 |
| `u4lb_e916` | usb4_lanebond.h:554 | e916 | FUN_CODE_e916 | e916 | Returns the plane-2 0x2805 read that seeds the 0x1335 RMWs in c593 |
| `u4lb_c593` | usb4_lanebond.h:555 | c593 | bank0_c593_stub_05c0 | c593,e916 | Bank0 tunnel/PHY commit (CCB0/CCB2/CCB3 + 0x134x/0x1335 lane-bond RMWs) |
| `u4lb_b8db` | usb4_lanebond.h:582 | b8db | bank1_b8db | b8db,e9e7 | CDR/PLL validate loop ([CDRV ok]): per-lane margin window + bounded poll of PLL-lock and C |
| `u4lb_state4_b0b4` | usb4_lanebond.h:615 | b0b4 | FUN_CODE_BANK1__b0b4 | b0b4,e07d,b226,e305,cdc6,e26a | State-4 PCIe-tunnel power-on / lane-bond engine body ([PcieTunnel-PwrOn]/Chg2 20G/RstRxpll |
| `u4lb_ee6e` | usb4_lanebond.h:747 | ee6e | FUN_CODE_BANK1__ee6e | ee6e | Per-lane SB connect-present = SB[lane?0x60:0x56].0 |
| `u4lb_eda0` | usb4_lanebond.h:750 | eda0 | FUN_CODE_BANK1__eda0 | eda0 | Route-special selector (0=eval,1=idle,2=route-special); clears 0x0775/0x0719 |
| `u4lb_e1cb_e2b9` | usb4_lanebond.h:763 | e1cb | FUN_CODE_BANK1__e1cb | e1cb,e2b9,d4cd,997e,9923,99ac | SB-transport descriptor builder: e1cb (0x0776!=0 live AMD path) and e2b9 (0x0776==0 path)  |
| `u4lb_e461` | usb4_lanebond.h:787 | e461 | FUN_CODE_BANK1__e461 | e461,e487,e499,9960,9966,e1cb | SB-transport route push the walker depends on; selects e1cb (0x0718==4 && 0x0776!=0 live A |
| `u4lb_ea7c` | usb4_lanebond.h:817 | ea7c | FUN_CODE_BANK1__ea7c | ea7c | CC-orientation PHY CL bit2 program (C2CB/C34B); sel==0x0F set bit2 else clear |
| `u4lb_8992` | usb4_lanebond.h:825 | 8992 | FUN_CODE_BANK1__8992 | 8992,d5da | Per-lane SB lane-arm: SB[0x15]=v, SB[0x0C]=(.&0x80)/3, d5da(1) |
| `u4lb_lane_gate` | usb4_lanebond.h:841 | 8000 | FUN_CODE_BANK1__8000 | 8006,8010,8015,801e | Lane gate: walk lane L iff work[0x19] & (1<<lane); transcribes the 8000-head DJNZ-after-SJ |
| `u4lb_8501` | usb4_lanebond.h:877 | 8501 | FUN_CODE_BANK1__8501 | 8501 | Banked SB-transport drain/poll; non-load-bearing, implemented as a deliberate no-op |
| `u4lb_lp1_finalize` | usb4_lanebond.h:883 | 81d4 | FUN_CODE_BANK1__8000 | 81d4,81f8,81fb,8208,8211,8215 | 8000 LOOP1 finalize sub-block: width-settle -> advance state cell to 0x60, compose TX[2:3] |
| `u4lb_lp1_width_settle` | usb4_lanebond.h:896 | 8174 | FUN_CODE_BANK1__8000 | 8174,81d4 | 8000 LOOP1 width-settle poll sub-block: advance via finalize when width pair settled vs CC |
| `u4lb_lane_status_b3` | usb4_lanebond.h:915 | 9a11 | FUN_CODE_BANK1__8000 | 9a11,84f3,84fa | Returns 1<<lane (the LANE-STATUS byte bVar3); transcribes 9a11 const-A=1 + the 8000-head s |
| `u4lb_walk_8000` | usb4_lanebond.h:923 | 8000 | FUN_CODE_BANK1__8000 | 8000,802a,804f,8069,807a,80ca | Primary state-5 CL-state lane-bond walker (0x0718==4): LOOP1 @0x0759 + LOOP2 @0x075B per-l |
| `u4lb_walk_850b` | usb4_lanebond.h:1101 | 850b | FUN_CODE_BANK1__850b | 850b | Alternate state-5 walker (0x0718!=4); dead on the live AMD path, kept for completeness |
| `u4lb_state5` | usb4_lanebond.h:1229 | 8000 | FUN_CODE_BANK1__8000 | 8000,850b | State-5 entry dispatch: DPX=0, s5 diag, then 8000 (0x0718==4) or 850b |
| `u4lb_e672` | usb4_lanebond.h:1239 | e672 | FUN_CODE_BANK1__e672 | e672 | Lane-bond FSM dispatcher from cb10 tail (0x06ED): 3->cm_conn_routing_setup, 4->b0b4, 5->fi |
| `pd_vdm_hdr_build` | vdm.h:16 | e120 | pd_vdm_hdr_build | e120 | Build the structured-VDM header VDO into the command registers (cmdtype<<6 / cmd, SOP-depe |
| `vdm_nak` | vdm.h:24 | dd0e | FUN_CODE_dd0e | dd0e,95a0 | Refactored helper: build a VDM NAK echoing the received SVID. In stock this is the inlined |
| `vdm_build_discover_id` | vdm.h:33 | aa36 | vdm_build_discover_id_resp | aa36 | Discover_Identity responder: build the ID ACK VDO chain (VID 0x174C, product PID, USB4 mod |
| `vdm_build_discover_sids` | vdm.h:70 | ddad | vdm_build_discover_sids_resp | ddad | Discover_SVIDs responder: ACK with TBT SVID 0x8087 when mode-flag bit7 set, else NAK. |
| `vdm_build_discover_modes` | vdm.h:85 | d852 | vdm_build_discover_modes_resp | d852 | Discover_Modes responder: ACK TBT3 mode VDO for SVID 0x8087 when mode-flag bit7 set, else  |
| `usb4_mode_entry_commit` | vdm.h:102 | d78a | usb4_mode_entry_commit | d78a | Device-side USB4 mode-entry latch: if 0x09F9 bit6 set write 92E1=0x10 + class=3/param=1 +  |
| `vdm_handle_enter_mode` | vdm.h:117 | b966 | vdm_handle_enter_mode | b966 | EnterMode (VDM cmd 0x04) responder: enter TBT alt-mode ACK + latch connect for SVID 0x8087 |
| `pd_handle_enter_usb` | vdm.h:140 | a036 | pd_handle_enter_usb | a036,a0a7 | Enter_USB Data Message (PD3.1) handler: parse EUDO mode (VDO0[3] bits6:4)/cable-current, t |
| `vdm_tx_strobe_commit` | vdm.h:183 | 9ac4 | vdm_tx_dispatch | 9ac4 | Refactored helper extracted from the inlined strobe sequence inside stock vdm_tx_dispatch  |
| `vdm_tx_dispatch` | vdm.h:196 | 9ac4 | vdm_tx_dispatch | 9ac4 | Structured-VDM command dispatcher: parse RX VDO0 (cmd/objpos/svid), branch to Disc_ID/Disc |

## Handmade-only (no stock counterpart): 35

- `uart_putc` (main.c:21) -- Blocking UART putc with a bounded spin; pure handmade diagnostic scaffolding (no
- `uart_puts` (main.c:22) -- UART string printer over uart_putc; pure handmade diagnostic scaffolding.
- `uart_puthex` (main.c:23) -- Print a byte as two hex nibbles to UART; pure handmade diagnostic scaffolding.
- `sleep` (main.c:31) -- Millisecond busy-sleep on Timer1; handmade scaffolding (avoids the CC10 PD mailb
- `hw_status_read` (main.c:71) -- Read INA231 bus voltage/shunt current into hw_status_t; tinygrad-specific handma
- `pcie_power_off` (main.c:81) -- Assert PERST#, drop rails and clear tunnel link state; handmade PCIe power seque
- `pcie_power_on` (main.c:90) -- PCIe rail/PERST# bring-up plus bounded LTSSM 0x78 stable-link poll; handmade pow
- `do_usb_bulk_in` (main.c:128) -- Read a PCIe dword chunk to 0x8000 and arm USB bulk-IN; tinygrad vendor-class str
- `handle_usb_control` (main.c:141) -- USB EP0 SETUP/DATA/STATUS dispatch incl vendor reqs 0xC0/0xE4/0xE5/0xF0/0xF2/0xF
- `handle_usb_bulk_data` (main.c:347) -- Bulk OUT/IN EP completion: drain to PCIe 0x7000 / re-arm IN; handmade vendor-cla
- `main` (main.c:475) -- Boot init (flash/PHY/PD/USB4 IRQ arm, 0x09F9=0x87 USB4 intent, XDATA scratch see
- `pcie_read_chunk` (pcie_pio.h:32) -- Hand-written 8051 PCIe PIO read loop: triggers TLPs (B296=0x07, B254=0x0F), poll
- `pcie_write_chunk` (pcie_pio.h:160) -- Hand-written 8051 PCIe PIO write loop: reads 4 bytes from src buffer, writes DAT
- `bank1_write` (pcie_tuning.h:16) -- Helper: banked (DPX=1) single-byte XDATA write to a bank-1 PHY/switch-plane regi
- `bank1_or_bits` (pcie_tuning.h:22) -- Helper: banked (DPX=1) read-modify-write OR-mask on a bank-1 XDATA register. Pur
- `pcie_apply_rxphy_lane_stage0` (pcie_tuning.h:29) -- Per-lane PHY setup stage0: writes lane_base+0x64/0xBF/0xBF/0x67. Values transcri
- `pcie_apply_rxphy_preamble` (pcie_tuning.h:37) -- Per lane/companion PHY preamble: lane 0x9B writes + companion 0x00..0x59 init wr
- `pcie_apply_rxphy_lane_stage1` (pcie_tuning.h:53) -- Per-lane PHY follow-up block: lane_base+0x40/0x01/0x11/0x21/0x31 writes. From MM
- `pcie_apply_rxphy_lane_stage2` (pcie_tuning.h:62) -- Main per-lane PHY payload (~27 banked writes to lane_base 0x02..0x6C). From MMIO
- `pcie_apply_rxphy_companion_profile` (pcie_tuning.h:93) -- Companion-slice PHY payload (paired double-writes to companion_base 0x20..0x2B).
- `pcie_apply_rxphy_tail` (pcie_tuning.h:120) -- Final per-slice PHY adjustments (lane 0x87/0x88/0x07.., companion 0x26, plus per
- `pcie_apply_x2_rxphy_tuning` (pcie_tuning.h:149) -- Top-level orchestrator: drives the four lane slices (0x78/0x79/0x7A/0x7B) and co
- `pd_wait` (pd.h:21) -- Pure handmade helper: bounded poll of (*reg & mask) against a wait-for-set/clear
- `pd_keystone_init` (pd.h:158) -- Handmade composite/scaffolding boot sequence: pd_int1_enable_group + u4_mode_fla
- `pd_queue_ctrl_msg` (pd.h:216) -- Enqueue a PD control message: stores code at u4_routerop_op_len (0x0AA3, == stoc
- `P1_REG8_rd` (sb.h:10) -- page-1/sideband 8-bit read accessor: set DPX=1, read XDATA off, restore DPX=0 (h
- `P1_REG8_wr` (sb.h:17) -- page-1/sideband 8-bit write accessor: set DPX=1, write XDATA off, restore DPX=0 
- `sb_write_c9_ack` (sb_router.h:19) -- W1C one connect bit in SB[0xC9] via r3_write_dispatch to 0x28C9; stock helper (9
- `sb_connect_present_poll` (sb_router.h:204) -- Handmade super-loop scaffolding: reproduces ebb5's connect-present effect at the
- `sb_transport_substate_poll` (sb_router.h:245) -- Handmade no-op stub (all edges handled by sb_d4cd_transport_edges); name collide
- `sb_chan_prelude` (sb_router.h:248) -- Handmade helper: consume the SB[0x09] read-ack (edd9 prelude); not a standalone 
- `usb4_int_demux` (usb4.h:88) -- USB4 demux fragment of the INT1 ISR (vector 0x0013): reads REG_INT_PCIE_NVME(C80
- `PG_RD` (usb4_irq.h:10) -- helper: paged (DPX=1) XDATA read of PHY register banks; handmade wrapper, no sta
- `PG_WR` (usb4_irq.h:13) -- helper: paged (DPX=1) XDATA write of PHY register banks; handmade wrapper, no st
- `u4lb_s5_diag` (usb4_lanebond.h:847) -- Pure-handmade [s5 ..] change-gated diagnostic printer (no stock counterpart)
