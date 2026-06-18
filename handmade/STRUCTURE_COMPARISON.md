# ASM2464PD firmware — stock vs handmade structural comparison
Auto-generated reference (Level-0: init + interrupt handling). Stock = fw_tinygrad.bin (Ghidra, handmade-named). Handmade = handmade/src/*.{c,h}.

---

## STOCK INIT TREE (reset → before super-loop)
I have everything needed for a complete, comparable stock-init tree. I have the full L0/L1/L2 with key side-effects and the critical L3 details. Let me compile the final tree.

=== STOCK INIT TREE (reset → before super-loop) ===

```
RESET @ CODE:0000  [reset vector / C runtime startup]
  side-effects: zero IRAM 0xFF..0x00; SP=0x72; func_0x030a(); run cstartup_init_table_0648
                (XDATA/SFR pre-seed via r3_write_dispatch); then tail-call main.
  -> main_boot_and_superloop @ CODE:2f80

main_boot_and_superloop @ CODE:2f80   [== handmade main.c::main]
  ====== ONE-TIME INIT (interrupts disabled), IN ORDER: ======

  1. xdata_00a59 = 0                                  (USB-enum state SM = idle)

  2. set_b0(0xcc32) @ CODE:5418                       XDATA 0xCC32 |= 1 (CC32.0 set; PHY/clk gate seed)

  3. boot_phy_bringup_early @ CODE:ce79  [via stub 0x04d0]   *** PHY/SERDES/Type-C SBU bring-up ***
       reads REG_LTSSM_CTRL(CC3F); gate: if CC3F.1 || CC3F.2:
         - boot_phy_d0d3_typec_sbu @ CODE:d0d3        (Type-C SBU pin mux -> powers SB transport)
       - boot_phy_cf28 @ CODE:cf28                    (PHY analog/SERDES cfg)
       - bank1_ed02 @ CODE_BANK1::ed02 [stub 0x0610]  (SB-enable on SBU; arg 0xcc3f)
       - REG_PHY_CONFIG(CC?) &= 0xFC
       - FUN_CODE_bd5e(0xc233)                         (PHY reg write helper)
       - phy_cmd_cc10_and_wait @ CODE:e80a (0,2)       (PHY cmd CC10 sub=2, wait)
       - REG_PHY_CONFIG &= 0xFB
       - phy_link_train_cmd_cc10 @ CODE:e50d (0,3)     (PHY link-train cmd, sub=3)
       - WAIT: until E712.0|E712.1 (REG_LINK_STATUS_E712) OR CC11.1 (TIMER0_CSR)
       - phy_cc11_ack_event @ CODE:e8ef               (W1C the CC11 event)
       - boot_phy_dd42(0) @ CODE:dd42
       - boot_phy_d996_pcie_tunnel_boot @ CODE:d996   *** PCIe-tunnel PHY pre-stage ***
            callees: FUN_CODE_e25e, bank0_e8a9 @ e8a9, boot_phy_d436_width @ d436,
                     boot_phy_d630_lane_power @ d630, boot_phy_e57d_e764_reset_pulse @ e57d,
                     read_pcie_ctrl_b402_clear_bit1 @ ccac, r3_read/r3_write
            (lane-width + lane-power + e764 RX-PLL reset pulse + B402.1 clear)

  4. hddpc_phy_init @ CODE:5284                        REG_PHY_EXT_5B(.3=1,.5=1), REG_HDDPC_CTRL(.5=0),
                                                       REG_PHY_EXT_2D(&0xE0|7).  (clock/analog HDDPC cfg)

  5. empty_stub @ CODE:04b2 -> e971                    (empty stub, no-op)

  6. boot_hw_init_main @ CODE:4fb6                     *** MAIN HW INIT ***
       a. boot_ramstate_reset_and_analogcfg @ CODE:5305
            - FUN_CODE_4c40 @ CODE:4c40   (clear PD/USB RAM 0x07xx/0x0Axx; seeds
                                            0x09F9=4, 0x09FA=4, 0x09FB=0 = default "USB4-capable")
            - sample E795 (REG_FLASH_READY_STATUS) bits 0x20/0x02/0x01
            - bank0_d6bc @ CODE:d6bc [stub 0x0534]  (analog cfg from straps)
            - xdata_007f6 = 1
       b. bank0_e597 @ CODE:e597 [stub 0x04b7]
       c. bank0_e14b @ CODE:e14b [stub 0x04bc]
       d. pd_int1_enable_group @ CODE:4be6          *** INT1/USB4 INTERRUPT-ENABLE GROUP (first IE arm) ***
            - 0x07F0..0x07F5 = 24 04 17 85 00 00  (version/cfg bytes)
            - REG_CPU_EXEC_STATUS_3 &= 0xFE
            - REG_INT_ENABLE(IE-mirror) = (&0xEF)|0x10
            - REG_INT_STATUS_C800 = (&0xFB)|4
            - REG_CPU_CTRL_CA60 = (&0xF8)|6 then (&0xF7)|8
            - set_b0(0xc800)  (C800.0=1)
            - set_b0(0xcc3b); CC3B = (&0xFD)|2
            - bank0_cb37 @ CODE:cb37 [stub 0x03a4]
            - bank1_ef24 @ CODE_BANK1::ef24 [stub 0x0584]
                 (C21B=(&0x3F)|0xC0; C202|=8; E741/E742 cfg; CC43=(&0x1F)|0x80;
                  C21F=(&0xFB)|4; SB[0x49]=0xA0)
            - bank1_ef1e @ CODE_BANK1::ef1e [stub 0x048f]
                 (page-0x78 PHY RX-lane reads; 0x7834=(&0x8F)|0x60)
       e. bank0_92c5_seed @ CODE:92c5 [stub 0x032c]   *** THE "92c5 SEED" ***
            - seeds XDATA link/lane state: 0xAEA=1, u4_link_busy(0xAE3)=1, 0xAE4=1, 0xAF0=1,
              0xAE5=1,0xAE6=1,0xAE7=1,0xAE8=1, 0xAE9=0xF, 0xAEE=3,0xAEF=3,0xAEB=3,
              u4_link_gen(0xAEC)=3, u4_link_lane(0xAED)=3, 0x213=0, 0xA83=0
            - loop i=0..5: bank0_e957(i); if SPI blob magic 0x707E=='Z' & checksum -> overlay
              cfg from 0x70xx via bb4f/bb8f/bba0/bb75/bb5e/bb96 into 0xAE3..0xAEC, 0x86C..0x871
            - tail: 0xAEB|=1; u4_connect_gate(0xAEx)=0x3F or 0 by 0xAE6; C655/C65A cfg;
              CC35/3-bit clears; FUN_CODE_e5fe (if !link_busy); FUN_CODE_dbbb (gate.0);
              bank1_ece1 @ CODE_BANK1::ece1 (gate.2); REG_USB_EP_CTRL_905F &= 0xEF
       f. bank0_8d77 = usb4_cap_apply_09f9 @ CODE:8d77 [stub 0x0539]   *** USB4 CAPABILITY APPLY (0x09F4-0x09F8 seed) ***
            - seeds: u4_dp_alt_mode(0x09F4)=3, u4_cap20g_gate0(0x09F5)=1,
              u4_cap20g_gate1(0x09F6)=1, 0x09F7=3, 0x09F8=1, 0x0A56=0
            - loop port 0..5: e957(); if SPI blob 0x707E==0xA5 & checksum -> overlay descriptor
              fields (0x0A41..0x0A55 VID/PID/strings) and OVERRIDE gates from 0x07059/0x0705A:
                u4_dp_alt_mode=0x07059[5:4], cap20g_gate0=0x07059.6, cap20g_gate1=0x07059.7,
                0x09F7=0x0705A[1:0], 0x09F8=0x0705A.2
            - MODE SWITCH on u4_dp_alt_mode (R7): ==3 -> u4_mode_flag(0x09F9)=0x87, 0x09FB=3;
              ==2 -> 0x09F9=0x06,0x09FB=1; ==1 -> 0x09F9=0x85; else 0xC1; lane_gate_sel(0x09FB)=2
            - if !E795.5: u4_mode_flag = 4
            - FUN_CODE_bbc7; if 0x0A56==1: bank1_9d90 @ CODE_BANK1::9d90
            (NOTE: this is the seeder of 0x09F4-0x09F9 — runs in INIT, before the b1cb decision)
       g. bank0_de16 @ CODE:de16 [stub 0x04f8]
       h. bank1_eef9 @ CODE_BANK1::eef9 [stub 0x063d]
       i. if u4_link_busy(0xAE3): REG_CPU_EXEC_STATUS &= 0xFE
       j. WAIT: do { } while ((REG_PHY_EXT_B3(C6B3) & 0x30)==0)   *** PHY/clock-ready gate ***
       k. pcie_tunnel_adapter_enable_b401 @ CODE:cd6c [stub 0x0462]
            - CA06 &= ~0x10; pcie_tunnel_adapter_config_b410 (B410-B42B from 0x0A52/53/54);
              mailbox 0x4084<-0x22, 0x5084<-0x50; B401.0=1 (TUNNEL MASTER EN); B482.0=1,
              B482=(&0x0F)|0xF0; B401&=~1 + B480.0=1 (PERST); B430&=~1; B298=(&0xEF)|0x10
       l. xdata_006e6 = 1
       m. bank0_d127 @ CODE:d127 [stub 0x0435]
            (PCIe DMA size/buf regs B?; CEF3=8,CEF2=0x80,CEF0&=0xF7,CEEF&=0x7F;
             INT_DMA_CTRL=(&0xFB)|4; B281=(&0xCF)|0x10)
       n. bank0_bf8e @ CODE:bf8e [stub 0x0340]

  7. boot_usb4_vs_usb3_mode_decision @ CODE:b1cb [stub 0x0327]   *** USB4-vs-USB3 MODE DECISION + PIPE ENGINE ***
       - PIPE/USB-PHY register block:
           POWER_ENABLE(92C0).7=1; 91D1=0x0F; 9300=0x0C;9301=0xC0;9302=0xBF; USB_CTRL_PHASE(9091)=0x1F;
           USB_EP_CFG1(9093)=0x0F; 91C1=0xF0; 9303=0x33;9304=0x3F;9305=0x40; USB_CONFIG(9002)=0xE0;
           USB_EP0_CFG(9005)=0xF0; USB_MODE(90E2)=1; USB_EP_MGMT(905E).0=0; USB_MSC_CTRL(C42C)=1;
           USB_MSC_STATUS(C42D).0=0
       - usb4_phy_set_enable(0) @ CODE:d07f; FUN_CODE_e214 @ CODE:e214
       - 91C3.5=0; pulse 91C0.0 (set then clear); clear_xdata_0af8 @ CODE:545c
       - phy_link_train_cmd_cc10 @ CODE:e50d (1,4)   (CC12=1, sub=4, CC13=0x8F)
       - WAIT: until E318.4 (REG_PHY_COMPLETION_E318) OR CC11.1
       - phy_cc11_ack_event @ CODE:e8ef
       - DECISION: if (91C0 & 0x18)==0x10 && u4_mode_flag(0x09F9)==4:
            FUN_CODE_bbb6; 0x09F9 = 1   (USB4)
         else:
            FUN_CODE_bbb6; 0x09F9 = 2   (USB3)

  8. GATED USB4 INIT (only if u4_mode_flag & 0x83 != 0):     [u4_mode_flag now 1 or 2; default 4 skips]
       bVar2 = u4_mode_flag & 0x81
       a. if bVar2 != 0 (USB4 path):
            bank1_e56f @ CODE_BANK1::e56f [stub 0x0494]   *** USB4 ROUTER-OP ENGINE init ***
              - EC00.0 clear; phy_cmd_cc10_and_wait_stub(0,0); EC00.0=1;
                EA88=0x64,EA89=0x24 (route speed); NVME_EVENT_ACK=1; EC05.0=0;
                INT_DMA_CTRL.6=0 then .7=1; u4_routerop_mbox_state = RMBOX_IDLE
       b. bank1_lane_flip_init = sb_lane_flip_init @ CODE_BANK1::b230 [stub 0x0606]  *** SB lane-flip / SB block init ***   [ALWAYS when &0x83]
              - prints [flp=N] from C6DB.0; clears P1[0x0100] bits4/6/7; sets/clears P1[0x0100].0
                by connect state; orientation SB lane map SB[0x01]/SB[0x02]; E7FC lane-bond gate;
                SB[0xD1].4=1; SB[0x49]=0xA0; XDATA 0x06EC=0; LINK_MODE_CTRL set/clear;
                XFER2 DMA cfg (C800-addr, ADDR_LO=0/HI=0xC8); SB[0x53]=0xFF,SB[0x5D]=0xFF;
                SB[0x2D].0/.1 set; INT_CTRL.3=1; lb_laneA_cl_latch=7, lb_laneB_cl_latch=7;
                if C8FF==4: copy lane_port_map_b[0x10] -> 0x3E.. (lane port map)
              callees: sprint,hex_print, r3_read/write, sb_* helpers, 9685/968e/9945/9900 (bank1)
       c. bank0_d894 @ CODE:d894 [stub 0x0589] (args bVar2,...)     [ALWAYS when &0x83]
              - FUN_CODE_bc8f; SB-reg writes via r3; INT_CTRL=(&0xFD)|2; bank0_b031 @ CODE:b031;
                SB[0x02]/SB[0x03] cfg; 0x121E.0=1
       d. pd_cc_attach_term_setup @ CODE:baa0 [stub 0x0525]         [ALWAYS when &0x83]
              gate: only if E795.5 (CC attach/VBUS present), else return
              - cc_pd_phy_term_init @ CODE:ae87  (PD PHY + CC termination)
              - u4lb_b8db / pd_internal_state_init @ CODE:b8c3
              - boot_phy_dd42(0); FUN_CODE_e6e7 @ CODE:e6e7
              - dispatch on 0x07000 cmd byte:
                  ':' -> pd_role_state=1, pd_contract_state=1, pd_data_reset_tx_enter_state_0e, "[Data_Reset_TX]"
                  ';' -> pd_role_state=2, "[RemoveRd to enable term]"
                  '<' -> pd_role_state=3, "[HardReset to enable term]"
                  else -> pd_arm_cc_timer(0x18,0x9c); cpu_dma_setup_transfer; return
              - (';'/'<' branch): CC98 cfg; set_reg_cc9a_0x5000; sb_lane_bond_complete_tunnel_up;
                u4_route_mode(0x0AE8)=4; usb4_mode_entry_commit; u4_entered_usb_mode set

  9. INTERRUPT-ENABLE FINALIZE (the IE/IP arming for the super-loop):
       PX0 = 0    (INT0 priority low)
       PX1 = 0    (INT1 priority low)
       EX0 = 1    (enable INT0 / int0_isr)
       EX1 = 1    (enable INT1 / int1_isr -- the USB4/PD handler)
       EA  = 1    (global interrupt enable)

  ====== then: do{...}while(1) SUPER-LOOP begins (NOT traced here) ======
```

KEY ORDERING / GATE NOTES for cross-tree diff:
- The `0x09F4-0x09F8` seeding lives in step 6f (`8d77`/usb4_cap_apply_09f9), which runs BEFORE the b1cb mode decision. `4c40` (step 6a) sets the default `0x09F9=4`; `8d77` overwrites `0x09F9` to 0x87/0x06/0x85/0xC1 (or forces back to 4 if `!E795.5`); then `b1cb` (step 7) overwrites `0x09F9=1` (USB4) or `=2` (USB3). So three writers of 0x09F9 in init, in this order: 4c40 → 8d77 → b1cb.
- The interrupt-enable happens in TWO places: (a) the INT1/USB4-handler arming register block `pd_int1_enable_group` @ 4be6 (step 6d, deep inside boot_hw_init_main — C800/CA60/CC3B + bank1 ef24/ef1e), and (b) the actual `EX0/EX1/EA=1` SFR writes at step 9 (top of main, just before the loop). `PX0=PX1=0` set priorities.
- The gated USB4 block (step 8) only runs when `u4_mode_flag & 0x83`; since b1cb sets it to 1 or 2, it runs. `bank1_e56f` (router-op engine) is additionally gated on `& 0x81` (USB4-only). `sb_lane_flip_init`, `d894`, `pd_cc_attach_term_setup` run for both 1 and 2.
- `pd_cc_attach_term_setup` (step 8d) is itself gated on `E795.5` (VBUS/attach present) — returns immediately if not attached.

Relevant handmade file for the comparison: the corresponding handmade init is `/home/batman/asm2464pd-firmware/handmade/src/main.c::main`, with sub-pieces in `boot_phy.h` (boot_phy_bringup_early), `sb.h` (sb_lane_flip_init = b230, sb_block_init = d894), `usb.h` (PIPE engine + phy arm = b1cb), `usb4_irq.h` (pd_int1_enable_group / ef24+ef1e arming), `pd.h` (pd_cc_attach_term_setup), `usb4_lanebond.h`.

---

## HANDMADE INIT TREE
I have all the information needed. Here is the complete handmade init tree.

=== HANDMADE INIT TREE (main.c::main, entry @ main.c:475) ===

```
main()  [main.c:475]  — runs ALL of these IN ORDER before the while(1) super-loop @ main.c:591
│
├─ REG_UART_LCR &= ~LCR_PARITY_MASK            [main.c:477]  wr UART_LCR (clear parity)
├─ uart_puts("[BOOT]")                          [main.c:479]
│
├─ flash_init()                                 [flash.h:7]   wr E2xx CPU_EXEC_STATUS_2=0x04, INT_AUX_STATUS=0x02, FLASH_DIV=0x04
│     (note: REG_CPU_CTRL_CA81|=0x01 deliberately COMMENTED OUT — broke PCIe on 9060)
│
├─ boot_phy_bringup_early()                     [boot_phy.h:131]  full early SERDES/Type-C SBU/PHY bring-up
│   ├─ if (LTSSM_CTRL.1 || LTSSM_CTRL.2):  boot_phy_d0d3_typec_sbu()   [boot_phy.h:37]  (GATE: cc3f bits1/2 set)
│   │     ├─ rmw LTSSM_CTRL &0xDF,&0xBF;  phy_cc10_cmd_wait(0,0,0x09)
│   │     ├─ boot_phy_d118(LTSSM_CTRL&0xFD) → wr LTSSM_CTRL, phy_cc10_cmd_wait(0,0,0xF9), rd back
│   │     ├─ wr LTSSM_CTRL|0x20; phy_cc10_cmd_wait(1,1,0x67)
│   │     ├─ boot_phy_d118(...) ; wr LTSSM_CTRL|0x40 ; phy_cc10_cmd_wait(0,0,0xF9)
│   │     └─ LTSSM_STATE &=0x7F
│   ├─ boot_phy_cf28()                    [boot_phy.h:53]  PHY config block — wr CC30.0, E710(width=4), C6A8.0, CPU_EXEC_STATUS_2=4,
│   │       E324&0xFB, CC3B/CC3E/CC39/CC3B link-timer bits, E716|=3, CPU_MODE_NEXT|=0x60, CA81.0
│   │     └─ boot_phy_bceb_set0(0xCC30/0xC6A8/0xCA81)  rmw bit0=1
│   ├─ boot_phy_bank1_ed02()              [boot_phy.h:73]  SB-block power: wr CC37.2, SB[0x05].7=1 (powers sideband xport),
│   │       CA70&0xFC, E780&0xF9, P1_CLR(0x0000,0x02)
│   ├─ REG_PHY_CONFIG &=0xFC, |=0x04      [main.c:138-139]
│   ├─ phy_cc10_cmd_wait(2,0,0x14)        [boot_phy.h:140]  CC10 mailbox cmd
│   ├─ REG_PHY_CONFIG &=0xFB; phy_cc10_cmd(3,0,0x0A); spin on E712.[1:0]/TIMER0; phy_cc11_ack()
│   ├─ boot_phy_dd42(0)                   [boot_phy.h:82]  mode0 → wr PHY_LINK_CTRL(E7E3)=0x00
│   └─ boot_phy_d996_pcie_tunnel_boot()   [boot_phy.h:122]  pre-stage downstream PCIe tunnel
│         ├─ B402&0xFD ; C659&0xFE
│         ├─ boot_phy_e57d_e764_reset_pulse(1)  [boot_phy.h:95]  E764 reset pulse (clr .1/.0/.3, set .2)
│         ├─ boot_phy_d630_lane_power(1)         [boot_phy.h:104]  wr B432|=7, B404 nibble=1; CLR E76C.4/E774.4/E77C.4 (PLL latch)
│         └─ boot_phy_d436_width(0x0F)           [boot_phy.h:117]  wr B434=0x0F, B436 nibble=0x0F
│
├─ uart_puts("[BOOTPHY ...]")  diag: reads cc3f/cc30/e712/SB[0x05]
│
├─ bank0_92c5_seed()                            [boot_phy.h:151]  seed lane-engine WIDTH/MODE/state RAM
│     ├─ DIAG print OTP 707E/707F/707A/7B/7D/7074-79  (probe only)
│     ├─ ** OTP/strap probe OMITTED ** — stock 92c5 reads fuses if 0x707E==0x5A and overrides
│     │     lane mask 0x0AE9 / lane-gen 0x0AEE / width 0x086C-71 from 0x707A/7B/7D. Handmade hard-codes:
│     ├─ wr XDATA 0x0213=0
│     ├─ wr 0x0AEA=1,0x0AE3=1,0x0AE4=1,0x0AF0=1,0x0AE5=1,0x0AE6=1,0x0AE7=1,0x0AE8=1
│     ├─ wr 0x0AE9=0x0F (lane mask), 0x0AEE=3,0x0AEF=3,0x0AEB=3,0x0AEC=3,0x0AED=3 (lane/gen)
│     ├─ wr 0x0A83=0, 0x0AEB|=0x01, 0x0AF1=0x00
│     └─ wr C65A&0xF7, CPU_EXEC_STATUS_3(E2xx)&0xFB, USB_EP_CTRL_905F&0xEF
│
├─ XDATA[0x09F9] = 0x87                          [main.c:495]  u4_mode_flag — USB4 INTENT (tunnel route + VDM-ACK). Gates everything below.
├─ XDATA[0x09F5]=1 ; XDATA[0x09F6]=1             [main.c:497]  cap20g_gate0=1, cap20g_gate1=1 (stock bank0_8d77 default; keeps 081A.1 → 0x0819=0x03 2-lane bond)
├─ XDATA[0x09F7]=3 ; 0x09F8=1 ; 0x09FB=3         [main.c:498]  (0x09F4 deliberately left at default)
├─ XDATA[0x0A57]=0x63 ; 0x0A58=0x24             [main.c:503-504]  device PID-lo / bcdDevice-hi — manually injected (stock loads from SPI shadow);
│        prevents the SB connect-desc TX going 0104 5555 instead of 0104 6324
│
├─ if (!(0x09F9 & 0x83))  { ... }                [main.c:507]  GATE: non-USB4 only — SKIPPED in USB4 mode (0x87&0x83 != 0)
│     ├─ usb_phy_tune()                          [usb.h:161]  → usb_serdes_tune_lane(0xC280) lane0, (0xC300) lane1  (SERDES RMW table)
│     ├─ wr PCIE_TLP_CTRL=0x01, PCIE_TLP_LENGTH=0x20
│     ├─ pcie_apply_x2_rxphy_tuning()            [pcie_tuning.h:149]
│     ├─ pcie_power_off()                        [main.c:81]  PERST assert, TUNNEL_LINK_STATE=0, E764&0x10, C659.0=0, HDDPC.5=0
│     └─ pcie_power_on()                         [main.c:90]  (back-compat) TUNNEL width, B403=1, PERST cycle, PHY_TLP_ROUTING, link poll
│   ── ALL OF THE ABOVE IS SKIPPED in the live USB4 path ──
│
├─ usb_pipe_engine_init()                        [usb.h:181]  bring up USB PIPE/PHY engine (unconditional)
│     wr POWER_ENABLE.7=1, 91D1=0x0F, 9300=0x0C, 9301=0xC0, 9302=0xBF, USB_CTRL_PHASE=0x1F,
│     EP_CFG1=0x0F, 91C1=0xF0, 9303=0x33, 9304=0x3F, 9305=0x40, USB_CONFIG=0xE0, EP0_CFG=0xF0,
│     USB_MODE=0x01, EP_MGMT.0=0, MSC_CTRL=1, MSC_STATUS.0=0, 91C3.5=0, 91C0.0 toggle
│
├─ usb4_phy_arm()                                [usb.h:224]  arm upstream USB4 PHY link-up via CC10 mailbox
│     wr TIMER0_CSR ack, TIMER0_DIV subcmd=4, THRESHOLD=0x018F, CSR=1, spin on E318.4/TIMER0, ack
│
├─ uart_puts("[PHYarm ...]")  diag: e318/91c0/e712
│
├─ pd_keystone_init()                            [pd.h:158]  PD attach + arm PD engine BEFORE interrupts
│   ├─ pd_int1_enable_group()                    [pd.h:149]  route C806/C80A PD/sys agg → 8051 EX1
│   │       wr INT_ENABLE(C801).4=1, C800.2=1, CA60 nibble→0x06 then 0x08, C800.0=1
│   ├─ u4_mode_flag = 0x87                        [pd.h:160]  (re-affirm 0x09F9=0x87)
│   ├─ cc_pd_phy_term_init()                      [pd.h:42]  CC Rp/Rd term + arm PD cmd/RX engine (host sees sink attach)
│   │       wr CMD_CONFIG.6, E40A=0x0F, E413&0xFE&0xFD, E400.7=0, XFER_DMA addr=0x0A00 cmd,
│   │       pd_wait, CMD_CONFIG.0=1, XFER_DMA addr=0x3C00 cmd, pd_wait, pd_wait E402.3,
│   │       E409 bits, E400.6=1, E411=0xA1, E412=0x79, E400 0x3C, INT_CTRL.5=1, pd_da51(),
│   │       E40E=0x8A, E400.7=1, CMD_CONFIG.0=0, E66A.4=0, E40D=0x28, E413 0x60, CAC4.0=0,
│   │       CMD_CONFIG.5=0, C698.5=0
│   └─ pd_internal_state_init()                   [pd.h:93]  reset PD policy-engine state block
│           uart "[InternalPD_StateInit]"; zero pd_* state vars (tx_staged, contract, msgid, len,
│           pdo_idx/valid, rx_num_obj, slot_idx, 07e3, substate=1, rx_slot_mask from E400.6,
│           sop_field, hardreset_done=0, u4_connect_route_latch/enter_usb/route_confirm_07cc=0,
│           07cb=0, confirm_input_cd/ce/cf=0, connect_pending=0, bist=0, usb3_fallback=0, role=0);
│         └─ cc_ctrl_enable_events()              [pd.h:81]  clr+enable CC attach/role events:
│               wr CC81=0x04 then 0x02, INT_ENABLE.4=1, CC80.4=0/.nibble=3, XFER_DMA_CFG=4/2,
│               INT_ENABLE.4=1, CPU_DMA_READY.4=0/.nibble=4
│         └─ seed pd timers a=1,b=0x2C,c=0,d=0x64,e=5,f=0,g=0
│
├─ uart_puts("[U4irq ...]")  diag: c21b/c202/e741/cc43
│
├─ usb4_irq_arm()                                [usb4_irq.h:242]  arm USB4 SB-transport / router INT path (host connect → C80A.5)
│   ├─ wr CPU_EXEC_STATUS_3(E2xx).0=0, CC3B.1=1
│   ├─ usb4_irq_ef24()                            [usb4_irq.h:236]
│   │   ├─ usb4_irq_db0d()                        [usb4_irq.h:211]  PHY link/SB setup → SB[0x1C]=0xC2
│   │   │       wr C21B 0xC0, C202.3=1, PG[0x1262].4=0, SB[0xED].6=1, SB[0xCE].0=0,
│   │   │       SB_SET 0x1C(0x80,0x40,0x02), C20B.7=0, SB[0x1D].0=0, C390(0xC22F), C22F.6=0
│   │   └─ usb4_phy_rx_descriptor_8e31()          [usb4_irq.h:44]  PHY PLL + lane-A/B RX descriptor config
│   │           wr E741 PLL_CTRL fields, PLL_CFG, CC43 CPU_CLK_CFG.7=1, C390(0xC21F), SB[0x49]=0xA0,
│   │           long lane-A (C2xx) + lane-B (C3xx) SERDES descriptor RMW chain (C34A/C2F8/C351/C335/...)
│   └─ usb4_irq_ef1e()                            [usb4_irq.h:227]  SB-PHY 4-lane RX arm: apply paged u4rx_tab[] RMW table (PG_WR)
│
├─ uart_puts("[U4irq c21b=...]")  diag
│
├─ if (0x09F9 & 0x81)  { ... }                   [main.c:538]  GATE: 0x87&0x81 != 0 → RUNS
│   └─ usb4_routerop_init()                       [usb4_irq.h:253]  CM router-op RX-enable
│         wr EC00.0=0, phy_cc10_cmd_wait(0,9,0), EC00.0=1, EA88=100, EA89=0x24,
│         NVME_EVENT_ACK(EC04)=1, EC05.0=0, INT_DMA_CTRL(C807).6=0/.7=1, u4_routerop_mbox_state=RMBOX_IDLE
│   └─ uart "[U4rop ec00=...]" diag
│
├─ if (0x09F9 & 0x83)  uart "[USB4 mode: skip USB3 device bring-up]"   [main.c:547]  GATE: RUNS → SKIPS usb_init_controller(0)
│     else  usb_init_controller(0)               [usb.h:166]  (NOT run in USB4 mode)
│
│   ─── XDATA scratch clears (crt0 only zeroes IRAM, not XDATA) ───
├─ for z in 0x0B02..0x0B1F: XDATA[z]=0           [main.c:555]  zero router-op mailbox working buf
├─ XDATA[0x06F1]=0                               [main.c:556]  SB-router active-port idx
├─ XDATA[0x0766]=0,0x072D=0,0x074E=0,0x074F=0    [main.c:558-559]  route-up / lane-bonded / per-lane CL0 latches
├─ XDATA[0x06EE]=0,0x06EF=0,0x06F0=0             [main.c:560]  transport-substate / port-plane
├─ XDATA[0x06EC]=0,0x06ED=0                      [main.c:562]  lane-bond FSM enable-gate / FSM state
├─ XDATA[0x0758..0x075C]=0,0x0718=0              [main.c:563-564]  FSM sub-states
├─ XDATA[0x0765]=0,0x0768=0,0x0769=0,0x0752=0,0x0753=0  [main.c:566-567]  b0b4 gate inputs / connect-desc scratch
├─ XDATA[0x07ED]=0                               [main.c:569]  Connect_U4 one-shot suppress clear
├─ for z in 0x0B45..0x0B58: XDATA[z]=0           [main.c:572]  diag-budget block clear
├─ seed diag budgets: sb_con_print_budget=6, af38_s3_budget=30, u4lb_edf5_print_budget=40,
│     u4lb_s5_print_budget=60, isr_dbg_budget/2/3=20, u4lb_s5_* trackers=0   [main.c:573-579]
│
├─ IE = EA|EX0|EX1|ET0  (0x87)                   [main.c:582]  ENABLE INTERRUPTS (EX1=PD/USB4 INT1, EX0=USB INT0, ET0=timer0)
│
├─ i2c_init()                                    [i2c.h:34]   I2C HW block (INA231 power monitor)  — AFTER IE
├─ ina231_init()                                 [i2c.h:101]  INA231 config  — AFTER IE
│
└─ while(1)  super-loop  [main.c:591]  ...
```

ORDER SUMMARY (handmade init, top-level, in execution order):
1. UART parity clear → flash_init
2. boot_phy_bringup_early (d0d3/cf28/ed02/PHY_CONFIG/cc10 cmds/dd42/d996)
3. bank0_92c5_seed (lane WIDTH/MODE RAM; OTP probe stubbed out)
4. 0x09F9=0x87 intent, then cap gates 0x09F5/0x09F6=1, 0x09F7/8/B, 0x0A57/0x0A58 manual seed
5. (non-USB4 PHY/PCIe block — SKIPPED via `if(!(09F9&0x83))`)
6. usb_pipe_engine_init → usb4_phy_arm
7. pd_keystone_init (int1_enable_group → cc_pd_phy_term_init → pd_internal_state_init → cc_ctrl_enable_events)
8. usb4_irq_arm (ef24 → db0d + 8e31, then ef1e)
9. usb4_routerop_init (gated 09F9&0x81)
10. (usb_init_controller — SKIPPED via `if(09F9&0x83)`)
11. bulk XDATA FSM/latch/budget clears
12. IE enable
13. i2c_init / ina231_init

NOTABLE STRUCTURAL POINTS for the cross-tree diff:

- **OTP/strap fuse readout is OMITTED** inside bank0_92c5_seed (boot_phy.h:154): stock 92c5 conditionally reads fuses (0x707E==0x5A) and overrides lane-mask 0x0AE9 / lane-gen 0x0AEE / width 0x086C-71; handmade hard-codes them. Documented gap.

- **0x0A57/0x0A58 manually injected** (main.c:503): no stock-equivalent firmware writer call — stock loads PID/bcdDevice from the SPI/boot-env shadow. This is a hand-patch standing in for a missing structural data-load path.

- **The CM 4CC command dispatch is entirely absent from init AND from the interrupt path** — there is NO call corresponding to stock `cm_command_dispatch@d283 -> cm_RXCM@cc86` anywhere in the handmade init tree. The only USB4 interrupt entry is `usb4_int_demux()` (usb4.h:91) from int1_isr, plus the super-loop FSM driver `sb_cb10_lane_advance()`. The primary-lane selection that stock does via cm_RXCM has no node here.

- **i2c_init / ina231_init run AFTER `IE` is enabled** (main.c:584-585) — they are post-interrupt-enable but still pre-super-loop, so they ARE one-time init, just sequenced after interrupts are live.

- **One-shot inits done INSIDE the super-loop (not before it):**
  - `sb_cb10_lane_advance()` + the e672/eda0/cdf5 lane-bond walker [main.c:597-622] — the entire state-3/4/5 lane-bond FSM advance runs in-loop, gated `if((09F9&0x83) && XDATA[0x06EC])`. This is the live driver of the CL walk; in stock the equivalent is interrupt/CM-driven.
  - `sb_a5d8_pend_int()` SB router-op responder [main.c:630-636], gated `sb_pend_int_pending || SB[0x26].1`.
  - **Deferred `bank0_c9a8(0)` / bank0_8a89 fallback** [main.c:640-654] — a NON-STOCK fallback that fires only after `fsm_stall>=6` and lanes not both CL0. Explicitly flagged in-tree as non-stock (batch-5 HW test was neutral).
  - **Deferred `pcie_power_on()`** [main.c:657-665] gated `sb_tunnel_up_pending` AND both SB[0xA0]/[0xA1] lanes == 0x02 (CL0). The actual downstream PCIe bring-up is deferred into the loop because it uses sleep()/long polls.
  - `pd_drive_hard_reset()` kick loop [main.c:777-780], gated `!pd_seen && kicks<60` — prompts the host into PD; one-shot-ish (stops once pd_seen).

INTERRUPT ENTRY TOPS (for completeness, since they are armed by this init):
- `int0_isr` __interrupt(0) [main.c:373] — USB peripheral demux (RST/CONTROL/ALT_LINK/BULK/EP/LINK_EVENT/CBW/MSC).
- `int1_isr` __interrupt(1) [main.c:461] — PD/USB4/system aggregate: `cc_pd_timer_tick()` (C806.0), `pd_rx_isr()` (C80A NVME.6), then `usb4_int_demux()` gated `XDATA[0x09F9]&0x83`. This is the ONLY USB4 interrupt dispatch; no cm_command_dispatch/cm_RXCM node exists.

Relevant files (all absolute): `/home/batman/asm2464pd-firmware/handmade/src/main.c`, `/boot_phy.h`, `/pd.h`, `/usb.h`, `/usb4_irq.h`, `/flash.h`, `/i2c.h`, `/sb_router.h`, `/usb4.h`.

---

## STOCK INTERRUPT TREE
This confirms the cm_cmd_table structure. Each entry is 6 bytes: [16-bit handler-relative offset][4-byte 4CC ASCII]. The relevant entries:
- `d2f6` + `5258434d` = "RXCM" -> handler at d2f6 LJMP slot -> cc86 (cm_RXCM_handler). This is the primary-lane selection command.
- `d2f9` + `53424552` = "SBER" -> d2f9 LJMP -> ada9
- `d2f3` + `5458434d` = "TXCM" -> d2f3 LJMP -> b3a6
- Plus BBER, EBER, EEIT, ENUM, LSEN, RBER, RDCP, RDSW, RHMG, RSMG.

I now have a complete, verified picture. Let me assemble the final tree.

=== BUILD TREE: stock-isr ===

```
8051 INTERRUPT VECTOR TABLE (CODE:0000..0023)
  0x0000 RESET  -> LJMP CODE:431a  (cold boot, not an ISR — out of scope)
  0x0003 INT0   -> LJMP CODE:0e5b  int0_isr  [USB device-controller link/EP demux]
  0x000B TIMER0 -> inline @0x0006: LCALL 0x50db (UART/serial drain) + 0x000a wakeup-counter dec; no fan-out
  0x0013 INT1   -> LJMP CODE:4486  int1_isr_orchestrator  [PD-RX + USB4 event demux] *** USB4 path ***
  0x001B TIMER1 -> no real handler (vector unused; falls into boot housekeeping)
  0x0023 SERIAL -> no real handler (falls into boot housekeeping)
  (no extended 246x vectors are wired to USB4 work; all USB4/PD events arrive via INT1)

=========================================================================================
INT0  int0_isr @CODE:0e5b   [USB2/3 device-controller event demux; NOT on the USB4-bond path]
  gate REG_INT_USB_STATUS(C0xx).0; saves/restores R0-R7. Demux by REG_USB_PERIPH_STATUS bits:
  - PERIPH.5 & USB_STATUS.0:  EP-complete drain loop (lut_5a6a/stride8_table) -> FUN_CODE_5442 (EP commit); tail USB_STATUS_909E.0 -> 5442 + EP ack 90E3=2
  - PERIPH.5 & EP_READY.0:    FUN_CODE_52a7(0x9096) -> set 0x9096=1
  - PERIPH.3:  buf-cfg 9301: .7->9301=0x80,POWER_DOMAIN rmw,bank0_e969(0x363); else 9302.7->ack; bit6->bank0_e6bd(0x35e),9301=0x40
  - PERIPH.1:  bank0_cf7f(0x33b)  [link-status change]
  - PERIPH.6 & USB_MODE.0:  if !USB_STATUS.0 -> USB_MODE=1, FUN_CODE_3419 (device re-enum);
                             else 0xc47b? -> NVME_QUEUE_BUSY.0 -> FUN_CODE_1196; else FUN_CODE_180d + 90E2=1
  - PERIPH.4:  buf-cfg 9302 bit-demux: .3->e941(0x37c)+9302=8; .4->e947(0x381); .5->e92c(0x386); .0->e96f(0x36d); .1->e970(0x372); .2->e952(0x377); 9300.2->df15(0x368)+9300=4; 9300.3->d2bd(0x38b)
  - else (PERIPH.0 path): EP_CFG1 bit-demux: .1->32a5(0x9093); .3->4d44(0x9093)+9093=8; .0/.2->5455(0x9093)
  - else branch REG_USB_PHY_CTRL_91D1 (the 0x91D1 source):
       .3->91D1=8 + bank0_9c2b(0x345);  .0->91D1=1 + bank0_c66a(0x34a);  .1->91D1=2 + bank0_e94d(0x34f);  .2->91D1=4 + bank0_e925(0x354)
  - TAIL1 REG_INT_SYSTEM.5: CPU_LINK_CEF3.3 -> 0x464=0,CEF3=8,FUN_CODE_2608(0xcef3); else CEF2.7 -> CEF2=0x80, FUN_CODE_3adb(0)
  - TAIL2 REG_INT_USB_STATUS.2: NVME drain loop (NVME_QUEUE_BUSY.0): cond 0x55? + NVME_LINK.1 -> FUN_CODE_488f; then FUN_CODE_1196 x32;
       then if !USB_STATUS.0: NVME_LINK.1->4784(0xc520); NVME_LINK.0->49e9
            else: NVME_LINK.0->3e81; NVME_LINK.1->488f
       then USB_MSC_CTRL.0 -> 4784(0xc42c)+MSC=1

=========================================================================================
INT1  int1_isr_orchestrator @CODE:4486   [the host-event handler — PD + USB4 lane-bond]
  saves R0-R7. Services sources IN THIS ORDER:

  (A) REG_INT_SYSTEM.0 (C806.0)  -> bank0_b4ba_stub(0x520) = cc_pd_timer_tick @CODE:b4ba   [UNCONDITIONAL]
      CC/PD timer-tick. 6 Type-C CC event regs, each bit1=event, W1C by writing 2:
        - CC23(TIMER3_CSR).1 -> cc_cc23_reinit_event @e3d8 (SB-reconnect re-init); CC23=2
        - CC81(CPU_INT_CTRL).1 -> read pd_msg_substate(0x07BD):
              ==0x0E/0x0D: CC81=2; if pd_role_state(0x07BC)!=0 -> sb_lane_bond_complete_tunnel_up(arg 0x3B=Data_Reset queue); cc_state_full_reset @d676
              else:        pd_cc81_hard_reset_4 @e90b (CC81|=4) -> pd_drive_hard_reset @be8b; CC81=2
        - CC91(CPU_DMA_INT).1 -> CC91=2; sprint "[1 sec time out]"; u4_connect_pending(0x07BB)=1; u4_route_mode(0x09FA)=4; usb4_mode_entry_commit; u4_entered_usb_mode set
        - CC99(XFER_DMA_CFG).1 -> role==2: queue 0x3C + pd_drive_hard_reset @be8b; role==3: queue 0xFF; else cc_cc99_default_event @e883; CC99=2
        - CCD9(XFER2_DMA_STATUS).1 -> CCD9=2; e461_inflight_token(0x0719)=2
        - CCF9(CPU_EXT_STATUS).1 -> CCF9=2; cc_ccf9_subdemux @df79 (sub-event demux on XDATA 0x0B1B)

  (B) REG_CPU_EXEC_STATUS_2.2 (CC33.2) -> CC33=4 (W1C); bank0_cd10_stub(0x390) @CODE:cd10   [UNCONDITIONAL]

  (C) REG_INT_PCIE_NVME.6 (C80A.6) -> pd_int_handler_stub(0x52f) = pd_rx_isr @CODE:af5e   [UNCONDITIONAL]  *** PD message RX ***
      prints "[PD_int:" + hex(E40F)+':'+hex(E410)+']'. Then:
        - E40F.7 -> soft_rst_int @dfdc; E40F=0x80
        - E40F.0 (msg received) -> E40F=1; pd_dispatch_data @CODE:83d6  [PD message dispatcher: Source_Cap/Request/Accept/PS_RDY/VDM/Enter_USB -> builds & TX responses]
        - E40F.5 -> E40F=0x20; hard_rst_int @e419
        - E410.0 -> E410=1; E410.3 -> E410=8; E410.4 -> E410=0x10
        - E410.5 -> E410=0x20; pd_reset_dispatch @e876
        - E410.6 (TX-done) -> E410=0x40; FUN_CODE_e439
        - E410.7 -> E410=0x80
        - TAIL: E314.0/.1/.2 W1C (PHY completion); E661.7 -> E661=0x80

  ---- USB4 demux GATE:  if (u4_mode_flag(0x09F9) & 0x83) != 0  ----  [all four below are gated]

  (D) REG_INT_PCIE_NVME.5 (C80A.5) -> sb_router_event_handler_stub(0x61a) -> bank1 sb_router_event_handler @CODE_BANK1::a066   *** SB lane-bond ***
      PART1 (a066-a0d5) per-port connect poll idx 0..3:
        if SB[0xC9].(4+idx) set AND sb_active_port_rr(0x06F1)==idx:
          -> sb_channel_connect_service @c3b2; W1C SB[0xC9]=(1<<idx) then (1<<(idx+4)) via sb_write_c9_ack; 0x06F1=(idx+1)&3;
             if page1 0x0109.0 -> sb_lane_bond_complete_tunnel_up (kick downstream PCIe)
      PART2 (a0d7+) substate/lane-event poll (all SB regs W1C via r3_write_dispatch):
        - sb_transport_substate_poll @ (reads SB[0x2D]/[0x2C] connect/disc latch)
        - SB[0x2D].0 & SB[0x2C].0 -> "[===SB Con===]"; FUN_CODE_BANK1__9797; sb_con_consequence
        - SB[0x2D].1 & SB[0x2C].1 -> "[===SB Dis===]"; sb_write_reg_0x282c(2); sb_phy_link_bringup
        - SB[0x66].0 (Lane Bonded) -> "Lane Bonded"; sb_lane_bonded_consequence; clear_xdata_74e_74f
        - SB[0x26].1 -> sb_a5d8_pend_int  ***[see PART3 below — this is the CM-dispatch entry]***
        - SB[0x9E].0 (L0:CL0) -> "L0:CL0 "; FUN_CODE_BANK1__9716; set_r3_dispatch_reg_bit0; sb_read_0819_bit1 gate; sb_reg_d4_set_bit5_clr_bit6; if sb_notify_flag2 -> bank0_d1cc(0x5a7)
        - SB[0x9E].1 (L1:CL0) -> "L1:CL0 "; sb_write_reg_set_bit1; (0819.0 gate) sb_reg_d4_set_bit5_clr_bit6; if sb_notify_flag3 -> bank0_d1cc(0x5a7)
        - SB[0x66].2 -> "L0:Abr2"; (sb_link_reinit_gate==0 & 989b) sb_phy_link_bringup
        - SB[0x66].5 -> "L1:Abr2"; (0819.1 & gate) sb_phy_link_bringup
        - SB[0x66].3 -> "L0:Bnd Fail"; sb_phy_link_bringup
        - SB[0x66].6 -> "L1:Bnd Fail"; sb_phy_link_bringup
        - SB[0x26].2 -> "L0:Disable"; FUN_CODE_BANK1__9880; sb_write_then_rmw_set_b7_b01(0x80); u4lb_d5da; sb_phy_link_bringup
        - SB[0x26].4 -> "L1:Disable"; FUN_CODE_BANK1__9880; sb_write_then_rmw_set_b7_b01(0xA0); u4lb_d5da; r3_write 0x285a=0x40; read_xdata_0819_clr_bit1
        - SB[0x9E].4 -> "L0:Training";   SB[0x9E].5 -> "L1:Training"
        - TAIL: SB[0x28F6].7 -> bank0_d5a1(0x5bb)

      PART3 *** THE PRIMARY-LANE / CM-COMMAND PATH (the structural omission in handmade) ***
        sb_a5d8_pend_int @CODE_BANK1::a6af  AND  sb_af38_descriptor_response @CODE_BANK1::b01f
          each, when their "command received" gate fires (a6af: XDATA[0x0aa2]==8; b01f: SB[0x50]==8):
            -> cm_command_dispatch @CODE_BANK1::d283
                 read ctx @0x0810; copy 4CC command code -> XDATA[0x0ab7] (R4:R5:R6:R7)
                 if gate XDATA[0x09f8]!=0 -> cm_default_command_code (b76c/b6fa) + write 0x0810; return
                 else -> c51_keyed_dispatch: match 4CC vs cm_cmd_table @d2a1 (6-byte entries [u16 LJMP-slot][4CC ascii]),
                         LJMP to cm_<KEY> handler. Table keys: BBER EBER EEIT ENUM LSEN RBER RDCP RDSW RHMG RSMG RXCM SBER TXCM
                 -> "RXCM" 4CC -> slot d2f6 -> LJMP cc86  cm_RXCM_handler @CODE_BANK1::cc86   *** SELECTS PRIMARY LANE ***
                      reads C6DB.0 (DAT_INTMEM_55), b663, builds 32b cmd word (get_xdata_0814 + shift_left_u32_by_n at 0x10/0x18),
                      extract_bits6_mask3_u32 -> DAT_INTMEM_54 (the lane select);
                      phy_lane_gate(global)=1;
                      DAT_INTMEM_54==0 -> select lane-table 0xc2c3 ; ==1 -> FUN_CODE_BANK1__c343 ; else cm_err_command_code
                      sets selected gate bit |1; SB[0x2840] rmw clr bit0/bit1; b73b;
                      C21C(PHY_LINK_CTRL) &=0xBF; b796(0xc208); PHY_CONFIG rmw |8; SB[..1d] rmw |2; clear_word_xdata_074e;
                      write_u32_xdata_0810
                 (other 4CC slots: TXCM->b3a6, SBER->ada9, ENUM/LSEN/RDCP/RDSW/RHMG/RSMG/RBER/BBER/EBER/EEIT -> their own LJMP slots)

  (E) REG_INT_PCIE_NVME.4 (C80A.4) -> tramp_bank0_c105(0x593) -> usb4_sec_adapter_link_event @CODE:c105   [secondary USB4 adapter/link]
        - rd P1[0x1407].0 -> usb4_linkwidth_event_service_a522 (E710 link-width recovery; 0x09FA|=4; W1C P1[0x1203].7)
        - rd P1[0x1407].3 -> bank1_d855(0x543) (W1C loop on P1[0x1508] bits4/3/2/1)
        - rd P1[0x1603].0 -> W1C P1[0x1603]=1; if 0x09FA.1: (92C2.6 -> 0x0AE2=1 + bank0_ca0d) ; u4lb_e74e (CC re-arm); pd_cm_dispatch_sel(0x07FF)=0x69; return
        - rd P1[0x1603].1 -> W1C P1[0x1603]=2; if 0x09FA.1: deeper reconfig bc88/bc63/e890 + (92xx.6 -> d916, bf8e)

  (F) REG_NVME_EVENT_STATUS.0 (EC06.0) -> EC04=1 (ack); if u4_connect_gate(0x09FA?).5: PHY_LINK_CTRL &=0xBF then &=0x7F;
        -> tramp_bank1_c0a5(0x499) -> cm_routerop_mailbox @CODE_BANK1::c0a5   *** USB4 router-op mailbox ***
        gate EA90==0x5A. state machine on u4_routerop_mbox_state(0x0B02):
          RMBOX_IDLE: latch EA80 -> opcode(0x0B03); c51_switch_dispatch via movc table @c0c5 keyed on opcode:
              0xE0->c0de 0xE1->c0e8 0xE2->c0ef(CONFIG read/write main) 0xE3->c119 0xE4->c151 0xE5->c158 0xE8->c15f(tunnel reset e4a6)
              CONFIG ops: bounds-check 0x0B04 vs 0x0B0A; dispatch d945(read)/ceab(write) router config space
              tunnel-reset e4a6: writes C656/CA06/CC31, sjmp-self wait, B480 PERST clr bits0-3, B402.0 clr
          RMBOX_MULTIPKT_1 (opcode==CONFIG): cm_routerop_send_read_resp + cm_routerop_addr_in_bounds; EA90=0xA5
          RMBOX_MULTIPKT_2 (opcode==PATH_E3): cm_routerop_send_write_resp + cm_routerop_addr_in_bounds; EA90=0xA5
        reply: build resp in 0x0B0A; C805|=0x02; C8B0<-0xEA

  (G) REG_INT_PCIE_NVME.(0..3) (C80A.0-3, mask 0x0F) -> tramp_bank1_e911(0x570) -> tunnel_link_event_e763 @CODE_BANK1::e911   *** PCIe-tunnel link ***
        gate C80A.0. 
          E763.2 (link-UP) -> E763=4; "[PcieTunnel-" + "PcieLinkUp]"; FUN_CODE_BANK1__d17e (read 0x09FA&0x81 latch, discarded)
          E763.3 (link-DOWN) -> E763=8; "[PcieTunnel-" + "PcieLinkDn]"

  (H) REG_INT_SYSTEM.4 (C806.4) -> empty() @CODE:0642  [stub, no-op; documented as bank1 0xEF4E slot — currently empty]
```

KEY STRUCTURAL NOTES for the diff (impact-ranked):

1. **PRIMARY-LANE SELECTION (the cited wall)** lives in branch (D) PART3: `sb_a5d8_pend_int@a6af` / `sb_af38_descriptor_response@b01f` gate on a "command received" state==8 and call `cm_command_dispatch@d283 -> c51_keyed_dispatch -> cm_cmd_table[RXCM]@d2f6 -> cm_RXCM_handler@cc86`. cc86 computes `DAT_INTMEM_54` (lane select) from the 4CC payload, sets `phy_lane_gate`, and writes the PHY primary-lane registers (C21C, C208 via b796, PHY_CONFIG|8). If handmade omits the d283/cc86 dispatch, no primary lane is ever selected -> host never grants CL0. This is reached only as a *sub-branch of the SB-router event handler (a066)*, NOT as a top-level INT1 source — so a "top-of-ISR" comparison alone will miss it; the comparison must descend into a066's command-dispatch sub-tree.

2. INT1 source ORDER is fixed: (A) timer-tick, (B) CC33, (C) PD-RX — all UNCONDITIONAL — then the `0x09F9 & 0x83` gate opens (D) C80A.5 SB-router, (E) C80A.4 sec-adapter, (F) EC06.0 router-op mailbox, (G) C80A.0-3 tunnel-link, then (H) C806.4 (empty). A handmade that services these out of order, or that leaves the `0x09F9&0x83` gate closed, starves D/E/F/G.

3. The router-op mailbox (F, c0a5) and the CM 4CC dispatch (D-PART3, d283) are TWO DISTINCT command channels: c0a5 = EA80/EA90 byte mailbox (config-space read/write + tunnel reset); d283/cc86 = SB 4CC command buffer @0x0ab7 (RXCM/TXCM/ENUM/...). Both must exist.

Relevant stock source-of-truth addresses (all verified in fw_tinygrad.bin Ghidra): int0=CODE:0e5b, int1=CODE:4486, cc_pd_timer_tick=CODE:b4ba, pd_rx_isr=CODE:af5e, pd_dispatch_data=CODE:83d6, sb_router_event_handler=CODE_BANK1::a066, cm_command_dispatch=CODE_BANK1::d283, cm_RXCM_handler=CODE_BANK1::cc86, cm_cmd_table=CODE_BANK1::d2a1, cm_routerop_mailbox=CODE_BANK1::c0a5, usb4_sec_adapter_link_event=CODE:c105, tunnel_link_event_e763=CODE_BANK1::e911. Handmade comparison files: /home/batman/asm2464pd-firmware/handmade/src/sb_router.h, /home/batman/asm2464pd-firmware/handmade/src/usb4_lanebond.h, /home/batman/asm2464pd-firmware/handmade/src/pd.h, /home/batman/asm2464pd-firmware/handmade/src/vdm.h, /home/batman/asm2464pd-firmware/handmade/src/main.c.

---

## HANDMADE INTERRUPT TREE
I now have a complete, comparable picture. The `c105` (C80A.4) handler is significant: it sets `pd_cm_dispatch_sel = 0x69` (the 0x69 token that gates `sb_channel_connect_service` n==0 path) and calls `e74e` (CC-ctrl event re-arm) and `ca0d` (mode-entry latch). Handmade OMITS this entire handler — it only sets `usb4_int_seen |= 0x02`. This is a structurally significant omission tied to the known wall (the 0x69 token / CM command path).

I have everything needed. Let me produce the final nested tree.

=== HANDMADE INTERRUPT TREE (hand-isr) ===

Entry vectors (main.c): `int0_isr __interrupt(0)` @ CODE:0003, `int1_isr __interrupt(1)` @ CODE:0013. Names in parentheses map to the renamed stock function the branch corresponds to.

```
int1_isr  (main.c:461, stock int1_isr@CODE:0013)
  side-effects: saves/restores DPX (forces DPX=0 across the whole ISR); prints I/t/p budget bytes when 0x06ED==5
  │
  ├─[gate REG_INT_SYSTEM(C806) & 0x01]  cc_pd_timer_tick   (pd.h:235, stock bank0_b4ba@CODE:b4ba)
  │     side-effects: tick_seen++(0x0B47); cc_hit bitmask(0x0B48); services 6 CC per-channel event regs (bit1):
  │     ├ CC23.1 (REG_TIMER3_CSR): cc_cc23_reinit_event (pd.h:199, stock e3d8) -> u4_connect_gate_e8(0x07E8)=0; u4_reinit_pending(0x07EF)=1 ; W1C CC23=0x02
  │     ├ CC81.1 (REG_CPU_INT_CTRL): if substate(0x0AB?)==0x0E/0x0D -> W1C; if pd_role_state!=0 pd_queue_ctrl_msg(0x3B); cc_state_full_reset (pd.h:205, stock d676, print-only [Error_Recovery])
  │     │                              else -> pd_cc81_hard_reset_4 (pd.h:210, stock e90b) -> REG_CPU_INT_CTRL(CC81)=0x04; pd_drive_hard_reset (be8b); then W1C CC81=0x02
  │     ├ CC91.1 (REG_CPU_DMA_INT): W1C; [1 sec time out]; u4_connect_pending=1; u4_route_mode(0x09FA)=0x04; u4_entered_usb_mode = usb4_mode_entry_commit (vdm.h, stock d78a)
  │     ├ CC99.1 (REG_XFER_DMA_CFG): role==2 -> pd_queue_ctrl_msg(0x3C)+pd_drive_hard_reset; role==3 -> pd_queue_ctrl_msg(0xFF); else cc_cc99_default_event(no-op,e883)+W1C
  │     ├ CCD9.1 (REG_XFER2_DMA_STATUS): W1C; e461_inflight_token(0x0719)=0x02
  │     └ CCF9.1 (REG_CPU_EXT_STATUS): W1C; cc_ccf9_subdemux (pd.h:226, stock df79) -> u4_routerop_desc0 = cc_subdemux_src(0x0B1B->0x0A9D)
  │     NOTE: stock b4ba ALSO calls sb_lane_bond_complete_tunnel_up(e529) on a path; handmade routes that through CC91/usb4_mode_entry_commit instead — verify equivalence.
  │
  ├─[gate REG_CPU_EXEC_STATUS_2(C82?) >> 2]  W1C only  (main.c:467)
  │     ***OMISSION***: stock acks REG_CPU_EXEC_STATUS_2=4 THEN calls bank0_cd10_stub_0390(@CODE:cd10). Handmade only W1C-acks, drops the cd10 handler entirely.
  │
  ├─[gate REG_INT_PCIE_NVME(C80A) >> 6]  pd_rx_isr   (pd.h:166, stock pd_int_handler_stub)
  │     side-effects: reads E40F; prints [PD_int:E40F:E410]; priority demux, all W1C:
  │     ├ E40F.7 Soft_Rst_Int -> W1C 0x80, print [Soft_Rst_Int]
  │     ├ E40F.0 msg-received -> W1C 0x01; pd_seen(0x0B45)=1; pd_rx_message_dispatch (pd_dispatch.h:303)
  │     │     pd_rx_message_dispatch side-effects: pd_rx_ptr()=0xE440+0x20*slot; parse hdr0/hdr1 -> pd_rx_num_data_obj, pd_msg_type, pd_sop_field; prints [D..]
  │     │       ├ if num_obj==0: pd_dispatch_control(msgtype) (pd_dispatch.h:242)
  │     │       │     0x01 pd_ctrl_goodcrc | 0x03 pd_ctrl_accept(arm CC timer, advance substate) | 0x04 pd_ctrl_reject | 0x06 pd_ctrl_ps_rdy(decode contract->0x07B8) | 0x0C pd_ctrl_wait | 0x0D pd_ctrl_soft_reset(reset msgid, TX Accept) | default pd_rx_nak_send
  │     │       └ if num_obj>0: pd_dispatch_data(msgtype) (pd_dispatch.h:257)
  │     │             0x01 Source_Cap -> pd_select_pdo_from_source_cap + pd_build_send_request_rdo (TX Request RDO via E420-E43F, pd_tx_commit_engine, arm CC timer) | 0x08 Enter_USB -> pd_handle_enter_usb (vdm.h) | 0x0F VDM -> vdm_tx_dispatch (vdm.h) | 2..7 pd_rx_nak_send
  │     ├ E40F.5 Hard_Rst_Int -> W1C 0x20, print [Hard_Rst_Int]
  │     └ else E410 sub-demux: W1C first set of {.0,.3,.4,.5,.6,.7}
  │     tail: if E314.0 W1C E314=0x01
  │
  ├─[gate u4_mode_flag(0x09F9) & 0x83]  usb4_int_demux   (usb4.h:91, = stock int1_isr USB4 block)
  │     side-effects: int_sources=REG_INT_PCIE_NVME(C80A); c80a_acc(0x0B4A) |= int_sources
  │     ├ [C80A.5]  usb4_int_seen|=0x01; sb_router_event_handler   (sb_router.h:540, stock a066 sb_router_event_handler@CODE_BANK1::a066)  ── SEE SUBTREE BELOW
  │     ├ [C80A.4]  usb4_int_seen|=0x02  ***OMISSION***
  │     │     stock calls tramp_bank0_c105 = usb4_sec_adapter_link_event_c80a4(@CODE:c105). That handler: services P1[0x1407].0 (a522 link-width recovery), P1[0x1407].3 (d855), P1[0x1603].0/.1 (W1C 1/2) and on evt0 runs e74e + sets pd_cm_dispatch_sel(0x07FF)=0x69 + ca0d mode-entry. Handmade sets ONLY the seen bit -> the 0x69 CM token is never produced here.
  │     ├ [REG_NVME_EVENT_STATUS(EC06).0]  usb4_int_seen|=0x04; REG_NVME_EVENT_ACK=1; cm_routerop_mailbox (usb4.h:63)
  │     │     handmade cm_routerop_mailbox: gate EA90==0x5A; tiny state machine over RMBOX_IDLE/MULTIPKT_1/2 reading EA80 opcode, ack EA90=0xA5
  │     │     ***DIVERGENCE***: stock has, before the call, a gate (u4_connect_gate.5 -> RMW REG_PHY_LINK_CTRL clear 0x40 then 0x80) that handmade drops; stock body = tramp_bank1_c0a5_usb4_router_op(@CODE_BANK1::c0a5) full router-op dispatcher (movc table func_0def, C805|=2 TX, C8B0<-0xEA), far larger than the handmade mailbox stub.
  │     └ [C80A & 0x0F]  usb4_int_seen|=0x08; inline: if REG_PHY_RXPLL_TRIGGER(E763).2 W1C 0x04; if .3 W1C 0x08  ***OMISSION***
  │           stock calls tramp_bank1_e911 = tunnel_link_event_e763(@CODE_BANK1::e911): gated C80A.0; E763.2 -> [PcieTunnel-PcieLinkUp]+d17e route/ready latch; E763.3 -> [PcieTunnel-PcieLinkDn]. Handmade only W1C-acks the two bits, drops the prints/d17e/gate.
  │
  └─[gate REG_INT_SYSTEM(C806) >> 4]  no-op  (main.c:471, stock empty())  — matches stock.


  SUBTREE: sb_router_event_handler  (sb_router.h:540, stock a066)
    side-effects: prints [a66 ...] diag when u4_fsm_state==5
    │
    ├ PART1 loop idx=0..3:
    │   if SB[0xC9] bit(4+idx) && sb_active_port_rr(0x06F1)==idx:
    │     ├ sb_channel_connect_service (sb_router.h:314, stock c3b2)
    │     │     side-effects: sb_edd9_receive_ack (sb_router.h:156, edd9): if P1[0x0109].0 -> clr, SB[0xD8]=0x02, E716 RMW=0x03, u4lb_eb62(0,CONN_ROUT), u4lb_98ec(0,3)
    │     │     reads desc lo/hi by port (SB[0x20/22],[0x21/23],[0xA4/A6],[0xA5/A7]); validate ~hi==lo;
    │     │     dispatch n=lo&0x0F: 1/5 -> sb_route_up_trigger(0x06EB)=1 ; 3 -> SB[0x15]=0x83, SB[0x0C] RMW, u4lb_d5da(1) (+gate sb_link_reinit_gate: SB[0x50]=0x40,SB[0x5A]=0x40,P1[0x0109]|=1)
    │     │            n==0 -> if u4_fsm_state==3: return (incl. pd_cm_dispatch_sel(0x07FF)==0x69 early-out); else SB[0x5A]=0x40, u4_work_buf[0x19] RMW, clear lb_laneA/B_cl0_latch or read SB[0xA0]
    │     ├ sb_write_c9_ack(idx) ; sb_write_c9_ack(idx+4)  (W1C SB[0xC9])
    │     ├ sb_active_port_rr = (+1)&3
    │     └ if P1[0x0109].0  sb_lane_bond_complete_tunnel_up (sb_router.h:304, stock e52d): sb_rom_descriptor_load; if u4_route_mode.1 sb_tunnel_up_pending(0x0B4F)=1; REG_CPU_CTRL_CA60 &= 0xF7
    │
    ├ sb_d4cd_transport_edges (sb_router.h:204, stock d4cd = stock-name sb_transport_substate_poll@CODE_BANK1::d4cd)
    │     side-effects: for SB[0x28]/[0x2A]/[0x81]/[0x83] bit3 with matching ping-pong toggle:
    │       sets sb_active_plane_port(0x06F0) = 0/1/2/3; sb_cd3f_dispatch(off,off); W1C 0x10/0x20/0x40/0x08
    │     sb_cd3f_dispatch (sb_router.h:170, cd3f): sb_edd9_receive_ack; read SB[desc4e], sb_connect_descriptor=SB[desc752]; gate checks; then ONE of:
    │       ├ (desc752&0x60)==0x60 -> sb_set_connect_present_ebb5 (SB[0x57]|=8, SB[0x61]|=8, sb_connect_present(0x0765)=1)
    │       ├ (desc752&1)==0       -> sb_eaac_populate_0777 (copy 0x40 host-desc bytes from RX plane -> u4_host_desc(0x0777..); W1C XFER2 0x04/0x02)
    │       └ else (route)         -> sb_af38_descriptor_response (build device->host TX into 0x2900 plane using sb_width_lut[0x06F2+type]/branchA_gate; SB[0x0C]/[0x15] status; u4lb_d5da(0))
    │
    ├ sb_transport_substate_poll (sb_router.h:235)  — NO-OP STUB (stock name d4cd; real work folded into sb_d4cd_transport_edges above)
    │
    ├ [SB[0x2D].0 clr && SB[0x2C].0]  ===SB Con=== : W1C SB[0x2C]=01 then 02; SB[0x2D] RMW; sb_con_consequence
    │     sb_con_consequence (sb_router.h:264, stock dea1): gated u4_conn_consequence_done(0x06EC); P1[0x0109] clr+SB[0xD8]=02; SB[0x00]/[0x04]/[0x01] RMW; P1[0x0100] RMW; phy_cc10_cmd(2,0,0x15)+timer wait;
    │       sb_db7a_route_arm (sb_router.h:243, db7a): branch u4_connect_route_latch(0x07B9): CA60/PHY/CA70 RMW; u4lb_eb62(0,CONN_ROUT); u4lb_98ec(0,3)
    │       LANE_TRAIN regs arm (REG_LANE_TRAIN_ARM/CTRL/MASK), u4_lane_train_trigger ^=1; if !sb_8a89_done sb_run_8a89_pending=1
    │
    ├ [SB[0x2D].1 clr && SB[0x2C].1]  ===SB Dis=== : W1C SB[0x2C]=02 then 01; SB[0x2D] RMW
    │     ***OMISSION***: stock Dis branch calls sb_phy_link_bringup(da9f) after the W1C; handmade prints + W1C only.
    │
    ├ [SB[0x66].0]  Lane Bonded : W1C SB[0x66]=01; sb_lane_bonded_consequence (eed6: lb_lane_bonded_flag(0x072D)=1; SB[0xC9]=0xFF; P1[0x01C8] RMW); sb_clear_cl0_width_latches
    ├ [SB[0x26].1]  sb_a5d8_pend_int (sb_router.h:397, a5d8 router-op pend-int); W1C SB[0x26]=02
    ├ [SB[0x9E].0]  L0:CL0 : W1C; print SB[0xA0]&0x0F; SB[0x64] RMW |0x01; gate(u4_work_buf[0x19]) -> sb_clear_cl0_width_latches + sb_set_d4_peer_cl0 (SB[0xD4] RMW set bit5)
    │     ***PARTIAL OMISSION***: stock L0:CL0 also reads 0x0819.1 (read_xdata_0819) and if sb_notify_flag2(0x09F2)!=0 calls bank0_d1cc_stub_05a7 downstream-notify; handmade drops the d1cc notify.
    ├ [SB[0x9E].1]  L1:CL0 : W1C; SB[0x64] RMW |0x02; gate -> clear latches + sb_set_d4_peer_cl0
    │     ***PARTIAL OMISSION***: stock L1:CL0 drops to bank0_d1cc_stub_05a7 via sb_notify_flag3(0x09F3); handmade omits.
    ├ [SB[0x66].2]  L0:Abr2 : W1C 0x04, print  ***OMISSION***: stock calls sb_phy_link_bringup(da9f) (gated sb_link_reinit_gate==0 && 989b)
    ├ [SB[0x66].5]  L1:Abr2 : W1C 0x20, print  ***OMISSION***: stock -> da9f (gated 0x0819.1 && reinit_gate)
    ├ [SB[0x66].3]  L0:Bnd Fail : W1C 0x08, print  ***OMISSION***: stock -> sb_phy_link_bringup(da9f)
    ├ [SB[0x66].6]  L1:Bnd Fail : W1C 0x40, print  ***OMISSION***: stock -> sb_phy_link_bringup(da9f)
    ├ [SB[0x26].2]  L0:Disable : W1C 0x04; SB[0x15]=0x80  ***PARTIAL OMISSION***: stock also FUN_9880 + sb_write_then_rmw_set_b7_b01 + u4lb_d5da + sb_phy_link_bringup(da9f)
    ├ [SB[0x26].4]  L1:Disable : W1C 0x10; SB[0x15]=0xA0; SB[0x5A]=0x40  ***PARTIAL OMISSION***: stock also FUN_9880 + sb_write_then_rmw_set_b7_b01(0xA0) + u4lb_d5da + r3_write 0x40@0x285a + read_xdata_0819_clr_bit1
    ├ [SB[0x9E].4]  L0:Training : W1C 0x10, print
    ├ [SB[0x9E].5]  L1:Training : W1C 0x20, print
    └ tail: (void)SB[0xF6]   ***OMISSION***: stock reads SB[0xF6] and if .7 set calls bank0_d5a1_stub_05bb; handmade reads-and-discards (no d5a1).
```

```
int0_isr  (main.c:373, stock int0_isr@CODE:0003)
  gate REG_INT_USB_STATUS(C800?) & INT_USB_GATE; periph=REG_USB_PERIPH_STATUS:
  ├ .BUS_RESET(0x91xx group):  read REG_USB_PHY_CTRL_91D1; if .0 W1C + bank0_c9a8(0) [91D1.1->c9a8]; else echo-write; print [RST]
  │     ***DIVERGENCE***: stock 91D1 demux has FOUR sub-handlers — .3->bank0_9c2b, .0->bank0_c66a, .1->bank0_e94d, .2->bank0_e925. Handmade collapses to a single 91D1.0->c9a8 (c9a8 is the route/connect-gate reset, NOT stock's c66a). The .1/.2/.3 USB-SS sub-events are unhandled.
  ├ .CONTROL:        handle_usb_control()  (standard USB control EP0)
  ├ .ALT_LINK:       read+echo REG_BUF_CFG_9301, print [ALT LINK]   (stock: 9301.6 -> e6bd / 9301.7 -> e969 power-domain — handmade echoes only)
  ├ .BULK_DATA:      handle_usb_bulk_data (main.c:347): BULK_OUT_COMPLETE -> pcie_write_chunk(0x7000) DMA; BULK_IN_COMPLETE -> do_usb_bulk_in
  ├ .EP_COMPLETE:    read+echo REG_USB_EP_READY, print
  ├ .LINK_EVENT(0x9302/0x9300): if 9302.2 W1C + bank0_c9a8(1) [9302.2->c9a8]; read 9300; if SS_FAIL: USB4-mode(0x09F9&0x83) -> ack only (NO USB2 drop); else is_usb2=1, REG_CPU_MODE=USB2, 91C0=0x10; W1C 9300
  │     ***DIVERGENCE***: stock 9302 demux dispatches .3->e941, .4->e947, .5->e92c, .0->e96f, .1->e970, .2->e952 and 9300 .2->df15/.3->d2bd. Handmade handles only 9302.2 (->c9a8) + 9300 SS_FAIL; the other 9302/9300 router-link sub-events are unhandled.
  ├ .CBW_RECEIVED:   read+echo REG_USB_MODE; REG_USB_BULK_EP_CMD=CBW
  └ else: [UNHANDLED INT0]
  ├ gate INT_USB_CTRL_PENDING: [MSC]; REG_USB_MSC_CTRL=1; REG_USB_MSC_STATUS=0   (stock has a full NVMe/MSC C800.2 service loop here — FUN_488f/1196/4784/49e9/3e81 — handmade stubs to two writes; NVMe path intentionally disabled)
  └ gate other bits: [UNHANDLED INT0 TYPE]
```

=== SUMMARY: stock interrupt sources/handlers NOT present (or stubbed) in the handmade ISRs ===

INT1 (the USB4-relevant path):
- **C80A.4 / c105 (usb4_sec_adapter_link_event)** — FULLY OMITTED (handmade only sets a seen-bit). This is the handler that sets `pd_cm_dispatch_sel=0x69` and runs `e74e`/`ca0d` mode-entry. Directly adjacent to the known CM-command wall.
- **EC06.0 / c0a5 router-op** — handmade replaces the full bank1 c0a5 router-op dispatcher (movc dispatch table, C805/C8B0 TX engine) with a minimal `cm_routerop_mailbox` EA80/EA90 ack stub; also drops the pre-call `u4_connect_gate.5 -> REG_PHY_LINK_CTRL` gate.
- **C80A.0-3 / e911 (tunnel_link_event_e763)** — OMITTED; handmade inlines only the E763.2/.3 W1C acks, no PcieLinkUp/Dn prints, no d17e latch.
- **C82?.2 / cd10 (bank0_cd10_stub_0390)** — OMITTED; handmade only W1C-acks REG_CPU_EXEC_STATUS_2.
- Inside a066: **`sb_phy_link_bringup` (da9f)** is never called from any handmade a066 branch — stock invokes it from SB-Dis, L0/L1:Abr2, L0/L1:Bnd Fail, and L0:Disable. The handmade recovery branches are print-only.
- Inside a066: the **`bank0_d1cc` downstream-notify** (gated `sb_notify_flag2`/`flag3` @0x09F2/0x09F3) on the L0/L1:CL0 branches, and the **`FUN_9880`+`sb_write_then_rmw_set_b7_b01`+`u4lb_d5da`** sequence on L0/L1:Disable, and the **SB[0xF6].7 -> bank0_d5a1** tail, are all omitted.

INT0:
- The stock **91D1** 4-way demux (.0 c66a / .1 e94d / .2 e925 / .3 9c2b) is collapsed to a single 91D1.0 -> `bank0_c9a8` (note: c9a8 is NOT the stock 91D1.0 handler c66a — it is a route/connect-gate reset).
- The stock **9302/9300** router-link multi-handler demux (e941/e947/e92c/e96f/e970/e952 + df15/d2bd) is reduced to 9302.2->c9a8 and 9300.SS_FAIL only.
- The stock **C800.2 NVMe/MSC** service loop (FUN_488f/1196/4784/49e9/3e81/...) is reduced to a 2-write MSC stub (NVMe deliberately disabled per design).
- Stock **9301.6->e6bd / 9301.7->e969** (ALT-link power-domain) reduced to echo-write.

Files: handmade ISRs live in `/home/batman/asm2464pd-firmware/handmade/src/main.c` (int0_isr:373, int1_isr:461, handle_usb_bulk_data:347), `/home/batman/asm2464pd-firmware/handmade/src/pd.h` (cc_pd_timer_tick:235, pd_rx_isr:166), `/home/batman/asm2464pd-firmware/handmade/src/pd_dispatch.h` (pd_rx_message_dispatch:303), `/home/batman/asm2464pd-firmware/handmade/src/usb4.h` (usb4_int_demux:91, cm_routerop_mailbox:63), `/home/batman/asm2464pd-firmware/handmade/src/sb_router.h` (sb_router_event_handler:540 + its callees). Highest-impact omission for the lane-bond wall: **C80A.4 / c105** (the `pd_cm_dispatch_sel=0x69` CM-command producer) and the absent **`sb_phy_link_bringup (da9f)`** re-bringups inside a066's Dis/Abr2/Bnd-Fail/Disable branches.

---

## LEVEL-0 STRUCTURAL DIFF

**Summary:** Two load-bearing high-impact omissions dominate the diff. (1) The PRIMARY-LANE SELECTION path is absent: stock reaches cm_RXCM_handler@cc86 (via cm_command_dispatch@d283 / cm_cmd_table@d2a1 RXCM slot) from inside the SB-router handler's a5d8/af38 cmd-state==8 gate, and cc86 computes the lane-select (DAT_INTMEM_54), sets phy_lane_gate, and writes the PHY primary-lane regs (C21C/C208/PHY_CONFIG|8). Handmade's sb_a5d8_pend_int (sb_router.h:397) and sb_af38_descriptor_response (sb_router.h:70) process the router-op buffer locally and only print [cm8] when op_idx==8 — no 4CC dispatch, no primary lane ever selected, so the host never grants CL0. (2) The C80A.4 handler usb4_sec_adapter_link_event@c105 is fully omitted (handmade only sets a seen-bit at usb4.h:98); it is the producer of pd_cm_dispatch_sel=0x69, which handmade only ever reads (sb_router.h:348) and clears (sb.h:423), never sets — plus its e74e CC re-arm and ca0d mode-entry never run. Also high-impact: sb_phy_link_bringup@da9f is never called from any a066 recovery branch (Dis/Abr2/Bnd-Fail/Disable are print+W1C only), and the EC06.0 router-op mailbox is a tiny EA90 ack stub replacing the full c0a5 config-rw/tunnel-reset/TX dispatcher (also dropping the u4_connect_gate.5 PHY_LINK_CTRL pre-gate). On the INIT side the headline gaps are no b1cb USB4-vs-USB3 mode decision (0x09F9 hard-set to 0x87 with no E318.4/91C0 link evidence), no 8d77 E795.5 attach gate, and no pcie_tunnel_adapter_enable_b401 (B401.0 TUNNEL MASTER EN deferred into the loop). Medium/low items: missing cd10 (CC33), tunnel-link e911 (d17e), d1cc CL0 downstream-notify, d5a1 tail, truncated L0/L1:Disable bodies, OTP-strap overlay in 92c5, hand-injected 0x0A57/0x0A58, missing hddpc_phy_init, and i2c_init/ina231_init running after IE enable. Stock evidence is from the Ghidra-verified trees; handmade evidence verified directly in handmade/src/{main.c,usb4.h,sb_router.h,usb4_lanebond.h,pd.h}. The single most likely bond unblock is wiring c105 (-> 0x69 token) together with the cc86 RXCM primary-lane select, since the 0x69 token gates sb_channel_connect_service's n==0 path and cc86 is the only writer of the PHY primary-lane registers.

### Init discrepancies

| impact | kind | desc | stock | handmade | fix |
|---|---|---|---|---|---|
| high | missing-call | USB4-vs-USB3 mode DECISION (b1cb) is entirely absent. Stock step 7 runs boot_usb4_vs_usb3_mode_decision@b1cb: it drives the USB PIPE/PHY block, issues phy_link_train_cmd_cc10(1,4), WAITs on E318.4 (PHY completion) OR CC11.1, acks, then DECIDES: if (91C0 & 0x18)==0x10 && u4_mode_flag==4 -> 0x09F9=1 (USB4) else 0x09F9=2 (USB3). Handmade never measures the upstream link state to choose USB4 vs USB3; it hard-sets 0x09F9=0x87 at main.c:495 unconditionally. The real host-link evidence (E318.4 / 91C0&0x18) is never consulted, so the device asserts USB4 intent even when the host has not negotiated a USB4-capable upstream link. usb4_phy_arm() (usb.h:224) does poll E318.4 but only to ack, never to gate the mode. | CODE:b1cb boot_usb4_vs_usb3_mode_decision (E318.4/91C0&0x18 -> 0x09F9=1 or 2) | handmade/src/main.c:495 (0x09F9=0x87 hard-set); usb.h:224 usb4_phy_arm polls E318.4 but never decides | Port b1cb's decision: after usb4_phy_arm()'s E318.4 wait, read 91C0 and only keep 0x09F9 in the USB4 family (0x81/0x83) if (91C0 & 0x18)==0x10; otherwise fall to 0x09F9=2 (USB3). This makes USB4 intent conditional on actual upstream-link evidence as stock does, and is the gate every downstream USB4 step keys on. |
| high | missing-call | usb4_cap_apply_09f9 (8d77) seeder is omitted; its E795.5 attach gate and DP-alt-mode mode switch are gone. Stock step 6f seeds 0x09F4=3,0x09F5=1,0x09F6=1,0x09F7=3,0x09F8=1,0x0A56=0, then (per SPI blob) the DP-alt-mode switch sets 0x09F9 to 0x87/0x06/0x85/0xC1 AND crucially: if !E795.5 forces u4_mode_flag=4 (no attach -> not USB4). Handmade hard-writes 0x09F5/6=1, 0x09F7=3, 0x09F8=1, 0x09FB=3 at main.c:497-498 and 0x09F9=0x87 at :495 with NO E795.5 attach check. | CODE:8d77 usb4_cap_apply_09f9 (seeds 09F4-09F8; mode switch; if !E795.5 -> 0x09F9=4) | handmade/src/main.c:495-498 (manual constant writes, no E795.5 gate) | Gate the 0x09F9 USB4 intent on E795.5 (CC attach/VBUS present) exactly as stock 8d77: if !(REG_FLASH_READY_STATUS & 0x20 attach bit) leave 0x09F9=4 so the USB4 path stays closed until a real attach. Pairs with the b1cb decision above. |
| high | missing-call | pcie_tunnel_adapter_enable_b401 (cd6c) is absent from init. Stock step 6k programs the PCIe tunnel adapter: CA06&=~0x10; pcie_tunnel_adapter_config_b410 (B410-B42B from 0x0A52/53/54); mailbox 0x4084<-0x22, 0x5084<-0x50; B401.0=1 (TUNNEL MASTER EN); B482.0=1, B482=(&0x0F)\|0xF0; B401&=~1 + B480.0=1 (PERST); B430&=~1; B298=(&0xEF)\|0x10. Handmade defers all PCIe tunnel bring-up into the super-loop pcie_power_on() (main.c:657) gated on both lanes already at CL0, so the TUNNEL MASTER EN / adapter config that stock establishes during INIT never runs up-front. The B4xx adapter is not configured before the host probes the tunnel. | CODE:cd6c pcie_tunnel_adapter_enable_b401 (B401.0 TUNNEL MASTER EN, B410-B42B adapter cfg, PERST) | handmade/src/main.c:657-665 (pcie_power_on deferred into loop, gated on SB[0xA0]/[0xA1]==0x02) | Port cd6c's adapter-enable into init (or at least the B401.0 master-enable + B410-B42B config + B482/B480 PERST sequence) so the tunnel adapter is configured before the host's tunnel probe, rather than waiting for a bond that depends on it. |
| medium | missing-call | 4c40 (RAM clear + default capability seed) is omitted. Stock step 6a (inside boot_hw_init_main@4fb6 -> boot_ramstate_reset_and_analogcfg@5305) calls FUN_CODE_4c40 which clears PD/USB RAM 0x07xx/0x0Axx AND seeds 0x09F9=4, 0x09FA=4, 0x09FB=0 (default 'USB4-capable'). Handmade relies on crt0 IRAM zeroing plus scattered manual XDATA clears (main.c:555-572) and never establishes the 0x09FA=4 / 0x09FB=0 defaults the way 4c40 does before 8d77/b1cb overwrite them. 0x09FA (u4_route_mode) in particular is left to other paths. | CODE:4c40 (clears 0x07xx/0x0Axx; seeds 0x09F9=4,0x09FA=4,0x09FB=0) | handmade/src/main.c:555-572 (partial manual XDATA clears; no unified 0x09FA/0x09FB default seed) | Add an explicit pre-seed of 0x09FA=4 and 0x09FB=0 (then let 8d77 overlay) and ensure the 0x07xx/0x0Axx ranges 4c40 clears are zeroed, so route-mode and lane-gate defaults match stock before the mode decision overwrites them. |
| medium | omitted-subfn | bank0_92c5_seed omits the OTP/strap fuse readout. Stock 92c5 (step 6e) loops e957(i) and, if SPI blob magic (0x707E=='Z') with valid checksum, overlays lane config from 0x70xx into 0x0AE3..0x0AEC and 0x86C..0x871 via bb4f/bb8f/bba0/bb75/bb5e/bb96. Handmade bank0_92c5_seed (boot_phy.h:151) hard-codes 0x0AE9=0x0F (lane mask), 0x0AEE=3, 0x086C-71 widths etc. and only DIAG-prints 707E/707F (boot_phy.h:154). On a board whose straps differ from the hard-coded defaults, the lane mask / lane-gen / width are wrong. | CODE:92c5 bank0_92c5_seed (OTP 0x707E=='Z' overlay of 0x0AE9/0x0AEE/0x086C-71) | handmade/src/boot_phy.h:151-154 (hard-coded lane mask/gen/width; OTP probe is print-only) | If the board's SPI blob is present (0x707E magic + checksum), overlay 0x0AE9 (lane mask), 0x0AEE (lane gen) and 0x086C-0x0871 (width) from the 0x70xx shadow as stock does, instead of hard-coding. Necessary for correct lane mask/width on non-default straps. |
| medium | divergent-value | 0x0A57/0x0A58 (device PID-lo / bcdDevice-hi) are manually injected with no stock-equivalent data-load. Stock loads PID/bcdDevice from the SPI/boot-env shadow (the same 0x707x blob path 92c5/8d77 use). Handmade hand-patches 0x0A57=0x63, 0x0A58=0x24 at main.c:503-504 to stop the SB connect-desc TX from emitting 0104 5555 instead of 0104 6324. This is a constant standing in for a missing structural SPI-shadow load; if the real device descriptor differs, the wrong constants ship. | SPI/boot-env shadow load (via e957/bb* inside 8d77/92c5) populates 0x0A41-0x0A55 + PID/bcdDevice | handmade/src/main.c:503-504 (XDATA[0x0A57]=0x63, [0x0A58]=0x24 hard-patch) | Port the descriptor-field overlay from 8d77/92c5 (0x0A41..0x0A58 from the 0x07059/0x0705A blob region) so PID/bcdDevice come from the boot env, removing the hand constants. |
| medium | missing-call | hddpc_phy_init (5284) is absent. Stock step 4 sets REG_PHY_EXT_5B(.3=1,.5=1), REG_HDDPC_CTRL(.5=0), REG_PHY_EXT_2D(&0xE0\|7) — clock/analog HDDPC configuration between boot_phy_bringup_early and boot_hw_init_main. Handmade jumps from boot_phy_bringup_early (main.c:485) straight to bank0_92c5_seed (main.c:492) with no HDDPC analog cfg. | CODE:5284 hddpc_phy_init (REG_PHY_EXT_5B/2D, REG_HDDPC_CTRL) | handmade/src/main.c:485-492 (no hddpc_phy_init between bringup and seed) | Port 5284: set PHY_EXT_5B bits3,5; clear HDDPC_CTRL bit5; PHY_EXT_2D = (&0xE0)\|7. Required HDDPC clock/analog setup the downstream PHY/tunnel depends on. |
| medium | missing-call | pd_cc_attach_term_setup (baa0) E795.5-gated mode-entry branch is not reproduced in init. Stock step 8d (gated u4_mode_flag&0x83, then E795.5) dispatches on the 0x07000 cmd byte and on the ';'/'<' branch runs CC98 cfg, set_reg_cc9a_0x5000, sb_lane_bond_complete_tunnel_up, u4_route_mode(0x0AE8)=4, usb4_mode_entry_commit, sets u4_entered_usb_mode. Handmade's pd_keystone_init (pd.h:158) calls cc_pd_phy_term_init + pd_internal_state_init but does NOT do the baa0 0x07000-cmd dispatch nor the route-mode/usb4_mode_entry_commit init-time path (handmade defers usb4_mode_entry_commit to the CC91 timeout in the ISR). | CODE:baa0 pd_cc_attach_term_setup (0x07000 dispatch; route_mode=4; usb4_mode_entry_commit at init) | handmade/src/pd.h:158 pd_keystone_init (no 0x07000 dispatch / init-time mode-entry) | Port the baa0 0x07000-byte dispatch and, on the term-enabling branch, the CC98/CC9A + sb_lane_bond_complete_tunnel_up + u4_route_mode=4 + usb4_mode_entry_commit sequence, so mode-entry is established at attach rather than only on a 1-second timeout. |
| low | wrong-order | i2c_init/ina231_init run AFTER IE is enabled. Stock performs all one-time HW init with interrupts disabled, then arms EX0/EX1/EA last (step 9). Handmade enables interrupts (IE=0x87) at main.c:582 and THEN calls i2c_init() (:584) and ina231_init() (:585). These I2C bring-ups can now be preempted by int1_isr mid-transaction. | CODE:2f80 main (EX0/EX1/EA=1 is the LAST init action, step 9) | handmade/src/main.c:582 (IE set) then :584-585 (i2c_init/ina231_init after IE) | Move i2c_init()/ina231_init() above the IE write (before main.c:582) so they complete with interrupts masked, matching stock's interrupts-last ordering. (Low bond impact but a correctness/ordering divergence.) |
| low | omitted-subfn | boot_hw_init_main (4fb6) sub-calls d6bc (analog cfg from straps), e597, e14b, de16, eef9, d127 (PCIe DMA size/buf regs: CEF3=8,CEF2=0x80,INT_DMA_CTRL\|4,B281 cfg), bf8e are not present as discrete calls. Handmade does flash_init + boot_phy + 92c5 but the boot_hw_init_main body (steps 6a-6n) is largely not ported; notably d127's DMA/buffer-size programming and d6bc strap-driven analog cfg are missing. | CODE:4fb6 boot_hw_init_main (d6bc/e597/e14b/de16/eef9/d127/bf8e) | handmade/src/main.c (no boot_hw_init_main equivalent; sub-calls absent) | Audit 4fb6's sub-calls and port the ones with live side-effects (at minimum d127 PCIe DMA buffer-size regs and d6bc strap analog cfg). Confirm each is needed via Ghidra before adding. |
| low | omitted-subfn | pd_int1_enable_group (4be6) bank1 ef24/ef1e arming is split/relocated vs stock. Stock step 6d arms C800/CA60/CC3B AND immediately calls bank1 ef24 (C21B/C202/E741/CC43/SB[0x49]=0xA0) and ef1e (page-0x78 PHY RX). Handmade splits this: pd_int1_enable_group (pd.h:149) does only C801/C800/CA60, and the ef24/ef1e arming is moved to usb4_irq_arm (usb4_irq.h:242, called later at main.c:531 after pd_keystone_init). The SB[0x49]=0xA0 and C21B seed thus happen at a different point in the sequence than stock. | CODE:4be6 pd_int1_enable_group (C800/CA60/CC3B + bank1 ef24 + ef1e together, step 6d) | handmade/src/pd.h:149 (IE regs) + usb4_irq.h:242 usb4_irq_arm (ef24/ef1e moved later, main.c:531) | Verify the relocated ef24/ef1e ordering doesn't matter (it runs before IE either way); if HW-order-sensitive, fold ef24/ef1e back adjacent to the C800/CA60/CC3B arm as stock keeps them in one group. |

### ISR discrepancies

| impact | kind | desc | stock | handmade | fix |
|---|---|---|---|---|---|
| high | omitted-subfn | PRIMARY-LANE SELECTION (the load-bearing wall): the entire cm_command_dispatch@d283 -> c51_keyed_dispatch -> cm_cmd_table[RXCM]@d2f6 -> cm_RXCM_handler@cc86 sub-tree is absent. Stock reaches it inside the SB-router handler (a066 PART3): sb_a5d8_pend_int@a6af (gate XDATA[0x0aa2]==8) and sb_af38_descriptor_response@b01f (gate SB[0x50]==8) both call cm_command_dispatch, which matches the 4CC code against cm_cmd_table@d2a1 and LJMPs to cm_RXCM_handler@cc86. cc86 computes DAT_INTMEM_54 (the lane select) from the 4CC payload, sets phy_lane_gate, and writes the PHY primary-lane regs (C21C&=0xBF, b796(0xc208), PHY_CONFIG\|8, SB[..1d]\|2, clear_word 0x074e). Handmade sb_a5d8_pend_int (sb_router.h:397) and sb_af38_descriptor_response (sb_router.h:70) process the router-op buffer LOCALLY (build TX, copy width-LUT bytes) but only print [a5d8:cm8]/[af38-A:cm8] when op_idx==8 (sb_router.h:462, :104) — they NEVER invoke a 4CC keyed dispatch and never run cm_RXCM_handler. No primary lane is ever selected, so the host never grants CL0. | CODE_BANK1::cc86 cm_RXCM_handler (via d283 cm_command_dispatch, table d2a1, slot d2f6); reached from a6af/b01f when cmd-state==8 | handmade/src/sb_router.h:461-462 (op_idx==8 -> only uart_puts [a5d8:cm8]); sb_router.h:104 ([af38-A:cm8] print only) | Port cm_command_dispatch@d283 and the RXCM handler@cc86: on the cmd-state==8 gate, decode the 4CC code at 0x0ab7, and for RXCM run cc86's primary-lane select (compute DAT_INTMEM_54 from the payload, set phy_lane_gate, write C21C&=0xBF / b796(0xc208) / PHY_CONFIG\|8 / SB[0x1d]\|2 / clear 0x074e). This is the cited wall — without it CL0 is never granted. |
| high | missing-call | C80A.4 / usb4_sec_adapter_link_event (c105) is FULLY OMITTED. Stock branch (E) handles the secondary USB4 adapter/link: P1[0x1407].0 -> a522 link-width recovery (E710, 0x09FA\|=4, W1C P1[0x1203].7); P1[0x1407].3 -> d855; P1[0x1603].0 -> W1C, and if 0x09FA.1: (92C2.6 -> 0x0AE2=1 + ca0d) + u4lb_e74e CC re-arm + sets pd_cm_dispatch_sel(0x07FF)=0x69 + return; P1[0x1603].1 -> deeper reconfig bc88/bc63/e890. Handmade usb4_int_demux (usb4.h:98) does ONLY usb4_int_seen\|=0x02. Consequently pd_cm_dispatch_sel is read at sb_router.h:348 and cleared at sb.h:423 but is NEVER SET to 0x69 anywhere -> the 0x69 CM token that gates sb_channel_connect_service n==0 (sb_router.h:348) can never fire; the e74e CC re-arm and ca0d mode-entry latch also never run. | CODE:c105 usb4_sec_adapter_link_event (sets pd_cm_dispatch_sel(0x07FF)=0x69, e74e, ca0d, a522 link-width) | handmade/src/usb4.h:98 (int_sources & 0x10 -> usb4_int_seen\|=0x02 only); 0x69 token never produced (only read sb_router.h:348, cleared sb.h:423) | Implement the C80A.4 handler: service P1[0x1407].0/.3 and P1[0x1603].0/.1, and on the 0x1603.0 evt with 0x09FA.1 set pd_cm_dispatch_sel=0x69, call u4lb_e74e() (already in usb4_lanebond.h:546) and ca0d mode-entry. This produces the 0x69 CM token the connect-service n==0 path depends on. |
| high | omitted-subfn | sb_phy_link_bringup (da9f) is never called from ANY handmade a066 branch. Stock a066 PART2 invokes da9f from: SB-Dis (after W1C), L0:Abr2 (gated reinit_gate==0 && 989b), L1:Abr2 (gated 0x0819.1 && reinit), L0:Bnd Fail, L1:Bnd Fail, and L0:Disable. These are the link re-bringup / recovery paths. Handmade's corresponding branches (sb_router.h:585-654) are PRINT + W1C only: SB Dis (sb_router.h:585), L0/L1:Abr2 (:627-634), L0/L1:Bnd Fail (:635-642), L0:Disable (:644-648), L1:Disable (:649-654) all omit the da9f re-bringup. After any abort/bond-fail/disable/disconnect the PHY link is never re-driven, so a transient training failure is terminal. | CODE_BANK1 sb_phy_link_bringup@da9f (called from SB-Dis, L0/L1:Abr2, L0/L1:Bnd Fail, L0:Disable in a066) | handmade/src/sb_router.h:585 (Dis), :627-642 (Abr2/Bnd Fail), :644-654 (Disable) — print+W1C only, no da9f | Port sb_phy_link_bringup@da9f and call it from each recovery branch (SB-Dis, L0/L1:Abr2 with their gates, L0/L1:Bnd Fail, L0:Disable) so a failed/aborted lane retrains instead of dead-ending. Critical for CM event handling robustness. |
| high | omitted-subfn | EC06.0 router-op: handmade replaces the full bank1 cm_routerop_mailbox@c0a5 dispatcher with a stub AND drops the pre-call gate. Stock branch (F): EC04=1 ack; if u4_connect_gate.5 -> RMW REG_PHY_LINK_CTRL clear 0x40 then 0x80; then c0a5 runs a real state machine (movc dispatch table @c0c5 keyed on opcode 0xE0-0xE8: CONFIG read/write via d945/ceab with 0x0B04 vs 0x0B0A bounds-check, tunnel-reset e4a6 writing C656/CA06/CC31 + B480 PERST + B402.0 clr) and replies (build resp in 0x0B0A, C805\|=0x02, C8B0<-0xEA). Handmade cm_routerop_mailbox (usb4.h:63) only: gate EA90==0x5A, tiny RMBOX_IDLE/MULTIPKT_1/2 state read of EA80 opcode, ack EA90=0xA5 — it performs NO config-space read/write, NO tunnel-reset, NO C805/C8B0 TX, and skips the u4_connect_gate.5 PHY_LINK_CTRL gate entirely. | CODE_BANK1::c0a5 cm_routerop_mailbox (movc table c0c5, d945/ceab config rw, e4a6 tunnel-reset, C805/C8B0 TX) + pre-gate u4_connect_gate.5->PHY_LINK_CTRL | handmade/src/usb4.h:63-88 cm_routerop_mailbox (EA90 ack stub, no config rw / no tunnel-reset / no TX); usb4.h:99-103 (no pre-gate) | Port the full c0a5 router-op dispatcher: the c0c5 movc opcode table, CONFIG-space read (d945) / write (ceab) with the 0x0B04/0x0B0A bounds-check, the e4a6 tunnel-reset, and the response path (resp in 0x0B0A, C805\|=0x02, C8B0<-0xEA). Add the pre-call u4_connect_gate.5 -> PHY_LINK_CTRL &=0xBF then &=0x7F gate. The host's router-op config queries currently go unanswered. |
| medium | omitted-subfn | C80A.0-3 / tunnel_link_event_e763 (e911) is reduced to bare W1C acks. Stock branch (G) (gated C80A.0) on E763.2 (link-UP) -> W1C 4, prints [PcieTunnel-][PcieLinkUp], calls FUN_BANK1__d17e (reads 0x09FA&0x81 route/ready latch); on E763.3 (link-DOWN) -> W1C 8, prints [PcieTunnel-][PcieLinkDn]. Handmade usb4_int_demux (usb4.h:104-108) inlines only: if E763.2 W1C 0x04; if E763.3 W1C 0x08 — no C80A.0 gate, no prints, no d17e route/ready latch read. The tunnel link-up/down event consequence (d17e) never runs. | CODE_BANK1::e911 tunnel_link_event_e763 (C80A.0 gate; E763.2->PcieLinkUp+d17e; E763.3->PcieLinkDn) | handmade/src/usb4.h:104-108 (E763.2/.3 W1C only, no gate/prints/d17e) | Port e911: gate on C80A.0, on E763.2 run the [PcieLinkUp] path + d17e route/ready latch, on E763.3 the [PcieLinkDn] path. The d17e consequence on tunnel-up is needed to latch the PCIe-tunnel-ready state. |
| medium | missing-call | CC33.2 / bank0_cd10 (0x0390) handler is dropped. Stock branch (B) acks REG_CPU_EXEC_STATUS_2=4 (W1C) THEN calls bank0_cd10_stub@cd10. Handmade int1_isr (main.c:467) only W1C-acks (REG_CPU_EXEC_STATUS_2 = 0x04) and never calls cd10. | CODE:cd10 bank0_cd10_stub (called after CC33=4 W1C in int1) | handmade/src/main.c:467 (W1C ack only, no cd10 call) | Decompile cd10 and port its body after the CC33 W1C. If verified to be a no-op stub this can stay, but it must be confirmed in Ghidra rather than assumed. |
| medium | omitted-subfn | L0:CL0 / L1:CL0 downstream-notify (bank0_d1cc, gated sb_notify_flag2/flag3) is omitted. Stock L0:CL0 reads 0x0819.1 and if sb_notify_flag2(0x09F2)!=0 calls bank0_d1cc(0x5a7); L1:CL0 likewise via sb_notify_flag3(0x09F3). Handmade L0:CL0 (sb_router.h:605-614) and L1:CL0 (:616-625) do the SB[0x64]/SB[0xD4] latch work but never read 0x09F2/0x09F3 nor call d1cc. The downstream-notify on CL0 (which kicks the PCIe-tunnel consumer) never fires from the ISR. | CODE:d1cc bank0_d1cc (downstream-notify on L0/L1:CL0, gated 0x09F2/0x09F3) | handmade/src/sb_router.h:605-614 (L0:CL0), :616-625 (L1:CL0) — no 0x09F2/0x09F3 read, no d1cc | On L0:CL0 read 0x0819.1 and if 0x09F2!=0 call d1cc; on L1:CL0 if 0x09F3!=0 call d1cc. Wire the downstream PCIe-tunnel notify that stock issues at CL0. |
| medium | omitted-subfn | L0:Disable / L1:Disable bodies are truncated. Stock L0:Disable: FUN_BANK1__9880 + sb_write_then_rmw_set_b7_b01(0x80) + u4lb_d5da + sb_phy_link_bringup(da9f). Stock L1:Disable: FUN_9880 + sb_write_then_rmw_set_b7_b01(0xA0) + u4lb_d5da + r3_write 0x285a=0x40 + read_xdata_0819_clr_bit1. Handmade L0:Disable (sb_router.h:644-648) only W1C + SB[0x15]=0x80; L1:Disable (:649-654) only W1C + SB[0x15]=0xA0 + SB[0x5A]=0x40. The 9880, d5da, da9f re-bringup, and 0x0819 bit-clear are all missing. | a066 L0/L1:Disable -> FUN_9880 + sb_write_then_rmw_set_b7_b01 + u4lb_d5da + da9f (+ 0x0819 clr for L1) | handmade/src/sb_router.h:644-654 (W1C + SB[0x15] write only) | Port the full Disable bodies: FUN_9880, sb_write_then_rmw_set_b7_b01(0x80/0xA0), u4lb_d5da, sb_phy_link_bringup, and for L1 the 0x285a=0x40 write + 0x0819 bit1 clear, so a disabled lane is properly torn down/retrained. |
| low | omitted-subfn | SB[0xF6].7 tail handler (bank0_d5a1) is dropped. Stock a066 TAIL reads SB[0x28F6]/SB[0xF6] and if bit7 set calls bank0_d5a1(0x5bb). Handmade sb_router.h:665 does (void)SB_RD(0xF6) — reads and discards, never checks bit7, never calls d5a1. | CODE:d5a1 bank0_d5a1 (a066 tail, gated SB[0xF6].7) | handmade/src/sb_router.h:665 ((void)SB_RD(0xF6) — discard) | Check SB[0xF6].7 and call d5a1 when set, matching the stock a066 tail. |
| medium | omitted-subfn | SB-Dis branch omits sb_phy_link_bringup (covered above under the da9f finding but distinct W1C-only divergence). Stock SB-Dis: after W1C SB[0x2C]/[0x2D], calls sb_write_reg_0x282c(2) then sb_phy_link_bringup. Handmade (sb_router.h:583-590) does the W1C dance and the SB[0x2D] RMW but no sb_write_reg_0x282c(2) and no link bringup. | a066 SB-Dis -> sb_write_reg_0x282c(2) + sb_phy_link_bringup(da9f) | handmade/src/sb_router.h:583-590 (W1C only) | On SB-Dis, after the W1C, write 0x282c=2 and call sb_phy_link_bringup so a disconnect re-arms the PHY for the next connect. |
| medium | missing-call | INT0 91D1 4-way demux collapsed to a single handler, and the one present is the WRONG stock function. Stock int0 91D1 demux: .3->bank0_9c2b, .0->bank0_c66a, .1->bank0_e94d, .2->bank0_e925. Handmade int0_isr (main.c:373) collapses to 91D1.0 -> bank0_c9a8(0) — and c9a8 is NOT stock's 91D1.0 handler c66a; it is a route/connect-gate reset. The .1/.2/.3 USB-SS sub-events are unhandled. | CODE:0e5b int0_isr 91D1 demux (.0 c66a / .1 e94d / .2 e925 / .3 9c2b) | handmade/src/main.c:373 int0_isr (91D1.0 -> c9a8 only; wrong fn + missing .1/.2/.3) | Restore the 4-way 91D1 demux with the correct handlers (.0 c66a, .1 e94d, .2 e925, .3 9c2b). Verify whether c9a8 belongs here at all or is a handmade-specific reset that should be removed/relocated. |
| low | missing-call | INT0 9302/9300 router-link multi-handler demux is reduced to 9302.2 + 9300 SS_FAIL only. Stock int0 dispatches 9302 .3->e941, .4->e947, .5->e92c, .0->e96f, .1->e970, .2->e952 and 9300 .2->df15, .3->d2bd. Handmade handle_usb_link / LINK_EVENT (main.c:373 region) handles only 9302.2 (->c9a8) and 9300 SS_FAIL; the other 9302/9300 router-link sub-events are unhandled. | CODE:0e5b int0 9302/9300 demux (e941/e947/e92c/e96f/e970/e952 + df15/d2bd) | handmade/src/main.c (int0 LINK_EVENT: only 9302.2->c9a8 + 9300 SS_FAIL) | Port the missing 9302/9300 sub-handlers if the USB3/SS device path is to function; low bond impact since USB4 mode skips the USB3 device engine, but document the deliberate omission. |
| low | divergent-value | cc_pd_timer_tick routes sb_lane_bond_complete_tunnel_up via a different path than stock. Stock b4ba calls sb_lane_bond_complete_tunnel_up(e529) on one of its CC paths directly. Handmade cc_pd_timer_tick (pd.h:235) routes the tunnel-up through CC91 -> usb4_mode_entry_commit instead (per the in-tree NOTE). The exact CC-event that triggers tunnel-up and the queue arg (0x3B Data_Reset) may differ. | CODE:b4ba cc_pd_timer_tick (sb_lane_bond_complete_tunnel_up on a CC path; CC81 0x0E/0x0D -> queue 0x3B) | handmade/src/pd.h:235 cc_pd_timer_tick (routes via CC91/usb4_mode_entry_commit; NOTE: verify equivalence) | Verify the CC81 0x0E/0x0D path queues 0x3B and calls sb_lane_bond_complete_tunnel_up the same as stock; confirm the CC91-routed tunnel-up is byte-equivalent rather than a behavioral substitute. |

