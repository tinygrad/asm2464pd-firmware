/* usb4_state.h -- named USB4 firmware-state globals (refactor #2 phase 2).
 * All state is named and PINNED at its stock XDATA address via __at (behavior matches the
 * byte-true build); object model = one definition per region (no overlapping page arrays). */
#ifndef USB4_STATE_H
#define USB4_STATE_H

/* ===================== USB4 bring-up FSM state enums ===================== */
/* All five enums are 1-byte. Members are EXACT register values.            */

/* (A) MAIN FSM — XDATA 0x06ED (u4_fsm_state). Dispatched by e672.          */
/* Dispatched values = {3,4,5} only; 0 dormant; 1,2 verified-nonexistent.   */
typedef enum {
    U4FSM_IDLE       = 0x00, /* dormant/reset; e672 & cb81 skip dispatch while 0 */
    U4FSM_CONN_ROUT  = 0x03, /* [ConnRout] conn-routing setup (-> a7de) */
    U4FSM_LANE_TRAIN = 0x04, /* PCIe-tunnel power-on / lane-training (-> b0b4) */
    U4FSM_LANE_BOND  = 0x05, /* CL-state lane-bond walker (-> 8000/850b) */
} u4_fsm_state_t;            /* NOTE: values 0x01,0x02 do not exist (no writer/no dispatch) */
_Static_assert(sizeof(u4_fsm_state_t) == 1, "u4_fsm_state_t must be 1 byte");

/* (B) STATE-3 SUB-FSM — XDATA 0x0758 (cm_conn_routing_substate). a7de.     */
/* Complete legal value set = {0x00,0x10,0x11}; any other = no-op RET.       */
typedef enum {
    CONNRT_PRINT_STATUS    = 0x00, /* terminal/print: LJMP eb62(0,4) -> parent->state4; cold-reset default */
    CONNRT_ARM_ROUTE_QUERY = 0x10, /* arm: edf5 route-query push; wait accept */
    CONNRT_AWAIT_RESULT    = 0x11, /* await/confirm body: gate 0x0777==0x0C, [ConnRout], 0x0718=4 */
} connrt_substate_t;
_Static_assert(sizeof(connrt_substate_t) == 1, "connrt_substate_t must be 1 byte");

/* (C) STATE-5 LOOP1 — XDATA 0x0759 (lane0) / 0x075A (lane1) lb_loop1_state. */
/* walk_8000 LOOP1; full set {00,10,20,30,40,50,60,70,80,90,A0,A1}.          */
typedef enum {
    LP1_PARKED            = 0x00, /* parked/idle; default vector 8355 loop-tail, no state work */
    LP1_WIDTH_INIT        = 0x10, /* seed; clear work[0x1C+lane] bits4/5/7 -> 0x20 */
    LP1_ARM_WAIT_PUSH     = 0x20, /* e461 push-wait #1 (84f3); push==1 -> 0x30 */
    LP1_LANE_PRESENT_SEL  = 0x30, /* host-ack sel + host_desc[0x4+lane].7 arm gate -> 0x40 */
    LP1_SETTLE_CLEAR      = 0x40, /* clear width-settle counter -> 0x50 */
    LP1_WIDTH_SETTLE_WALK = 0x50, /* TX[2:3] CL-walk/width-settle step; finalize ->0x60, overflow/re-entry ->0x00 */
    LP1_BOND_WAIT_PUSH    = 0x60, /* e461 push-wait #2 (84f3); push==1 -> 0x70 */
    LP1_WIDTH_LATCH_SEL   = 0x70, /* width-latch sel: snap&0xC0==0xC0 + nibble-match -> 0x80 */
    LP1_FINALIZE_A        = 0x80, /* finalize pass A (ee6e gate work-bit set) -> 0x90 */
    LP1_FINALIZE_B        = 0x90, /* finalize pass B (uncond clear bit7) -> 0xA0 */
    LP1_BOND_WAIT_ACK     = 0xA0, /* e461 push-wait #3 (84fa); push==1 -> 0xA1 */
    LP1_BONDED_MONITOR    = 0xA1, /* bonded monitor; snap&0xC0==0x80 host-retrain -> 0x50, else -> 0xA0 */
} lp1_state_t;
_Static_assert(sizeof(lp1_state_t) == 1, "lp1_state_t must be 1 byte");

/* (D) STATE-5 LOOP2 (LIVE walk_8000 path) — XDATA 0x075B/0x075C lb_loop2_state. */
/* walk_8000 LOOP2 value set {00,10,20,30,50,60}. (850b is a SEPARATE encoding   */
/* on the SAME cell — see CRITICAL CALLOUT C1; do NOT enum-mix.)                  */
typedef enum {
    LP2_CL_IDLE      = 0x00, /* terminal/parked; no ladder case (reached from 0x30 snap.bit4) */
    LP2_CL_INIT      = 0x10, /* seed: work[0x1E+lane]|=0x80,&=0xBF -> 0x20 */
    LP2_CL_PUSH_WAIT = 0x20, /* e461 push-wait (84fa); push==1 -> 0x30 */
    LP2_CL_EVAL      = 0x30, /* PRIME bond-decision: snap=host_desc[0x2+lane]; .4->0x00, !.7->0x20, .7->CL-cfg->0x50 */
    LP2_CL_BOND_WAIT = 0x50, /* e461 push-wait #2 (84a3); push==1 -> 0x60 */
    LP2_CL_BOND_MON  = 0x60, /* bonded monitor; snap.7 retrain -> 0x50, else -> 0x20 */
} lp2_state_t;
_Static_assert(sizeof(lp2_state_t) == 1, "lp2_state_t must be 1 byte");

/* (E) CM ROUTER-OP MAILBOX — XDATA 0x0B02 (u4_routerop_mbox_state). c0a5.   */
typedef enum {
    RMBOX_IDLE       = 0x00, /* idle/single-packet; latch EA80, run 0def dispatch */
    RMBOX_MULTIPKT_1 = 0x01, /* CONFIG-read (E2) continuation */
    RMBOX_MULTIPKT_2 = 0x02, /* E3 write/path-config continuation */
} rmbox_state_t;
_Static_assert(sizeof(rmbox_state_t) == 1, "rmbox_state_t must be 1 byte");

/* ---- computed-access regions: named arrays at fixed base (members accessed as name[off]) ---- */
volatile __xdata __at(0x0600) uint8_t sb_cfg06[0x10];   /* a5d8 router-op config slot bytes */
volatile __xdata __at(0x06F2) uint8_t sb_width_lut[0x13];   /* af38 per-descriptor width LUT (ROM 0x514c) */
volatile __xdata __at(0x0705) uint8_t sb_branchA_gate[0x13];   /* af38 BRANCH-A gate (ROM 0x515f) */
volatile __xdata __at(0x071A) uint8_t sb_lane_desc[0x10];   /* lane-descriptor work buffer (ROM 0x21d4) */
volatile __xdata __at(0x072E) uint8_t lb_cap_field[0x10];   /* per-CL cap source bytes */
volatile __xdata __at(0x073E) uint8_t sb_lane_flip[0x10];   /* lane-flip table (ROM 0x21b4) */
volatile __xdata __at(0x0759) lp1_state_t lb_loop1_state[0x2];   /* state-5 LOOP1 state cell per lane */
volatile __xdata __at(0x075B) lp2_state_t lb_loop2_state[0x2];   /* state-5 LOOP2 CL-walk state per lane */
volatile __xdata __at(0x075D) uint8_t lb_lane_desc_idx[0x2];   /* per-lane 4-bit desc-walk index */
volatile __xdata __at(0x075F) uint8_t lb_settle_counter[0x2];   /* per-lane width-settle counter */
volatile __xdata __at(0x0761) uint8_t lb_cl_value[0x2];   /* per-lane CL value */
volatile __xdata __at(0x076C) uint8_t lb_width_pairA[0x4];   /* per-lane width pair A (lo/hi) */
volatile __xdata __at(0x0770) uint8_t lb_width_pairB[0x4];   /* per-lane width pair B (lo/hi) */
volatile __xdata __at(0x0777) uint8_t u4_host_desc[0x40];   /* host SB descriptor RX mirror (eaac fills 0x40) */
volatile __xdata __at(0x0800) uint8_t u4_work_buf[0x64];   /* DROM/work buffer (ROM 0x213d) + lane shadows */
volatile __xdata __at(0x099C) uint8_t sb_routerop_body[0x40];   /* a5d8 router-op descriptor body */
volatile __xdata __at(0x0AB4) uint8_t phy_lane_cap[0x2];   /* per-lane PHY cap */
volatile __xdata __at(0x0B26) uint8_t lb_cl_status[0x2];   /* per-lane CL status */
volatile __xdata __at(0x0B28) uint8_t lb_eq_status[0x2];   /* per-lane EQ status */
volatile __xdata __at(0x0B2A) uint8_t lb_loop2_scratch[0x2];   /* LOOP2 scratch per lane */
volatile __xdata __at(0x0B2C) uint8_t lb_cl0_width[0x2];   /* per-lane CL0 width */

/* named offsets into the above regions (readability for fixed-offset members) ----------- */
#define SB_XDATA_PAGE06                  0x00   /* sb_cfg06[0x00] = 0x0600 */
#define SB_XDATA_PAGE07                  0x0E   /* sb_width_lut[0x0E] = 0x0700 */
#define SB_LANE_DESC_WORK                0x00   /* sb_lane_desc[0x00] = 0x071A */
#define LB_CAP_FIELD_TABLE               0x00   /* lb_cap_field[0x00] = 0x072E */
#define PHY_RATE_LO_OR_MASK              0x0B   /* lb_cap_field[0x0B] = 0x0739 */
#define SB_FLIP_ROM_21B4                 0x00   /* sb_lane_flip[0x00] = 0x073E */
#define PHY_RATE_SHIFT_SRC               0x0B   /* sb_lane_flip[0x0B] = 0x0749 */
#define LB_LOOP1_STATE                   0x00   /* lb_loop1_state[0x00] = 0x0759 */
#define LB_LOOP1_STATE_LANEB             0x01   /* lb_loop1_state[0x01] = 0x075A */
#define LB_LOOP2_CLWALK_STATE            0x00   /* lb_loop2_state[0x00] = 0x075B */
#define LB_LOOP2_CLWALK_LANEB            0x01   /* lb_loop2_state[0x01] = 0x075C */
#define LB_LANE_DESC_IDX                 0x00   /* lb_lane_desc_idx[0x00] = 0x075D */
#define LB_LANE_DESC_IDX_LANEB           0x01   /* lb_lane_desc_idx[0x01] = 0x075E */
#define LB_LANE_SETTLE_COUNTER           0x00   /* lb_settle_counter[0x00] = 0x075F */
#define LB_LANE_CL_VALUE                 0x00   /* lb_cl_value[0x00] = 0x0761 */
#define LB_LANE_WIDTH_PAIRA_LO           0x00   /* lb_width_pairA[0x00] = 0x076C */
#define LB_LANE_WIDTH_PAIRA_HI           0x01   /* lb_width_pairA[0x01] = 0x076D */
#define LB_LANE_WIDTH_PAIRB_LO           0x00   /* lb_width_pairB[0x00] = 0x0770 */
#define LB_LANE_WIDTH_PAIRB_HI           0x01   /* lb_width_pairB[0x01] = 0x0771 */
#define U4_HOST_CONNECT_DESC             0x00   /* u4_host_desc[0x00] = 0x0777 */
#define U4_HOST_DESC_BYTE1               0x01   /* u4_host_desc[0x01] = 0x0778 */
#define U4_HOST_DESC_BYTE2               0x02   /* u4_host_desc[0x02] = 0x0779 */
#define U4_LANE_WIDTH_BITS               0x03   /* u4_host_desc[0x03] = 0x077A */
#define U4_HOST_DESC_BYTE4               0x04   /* u4_host_desc[0x04] = 0x077B */
#define U4_DROM_BUF                      0x00   /* u4_work_buf[0x00] = 0x0800 */
#define U4_DROM_PID_LO                   0x04   /* u4_work_buf[0x04] = 0x0804 */
#define U4_DROM_PID_HI                   0x05   /* u4_work_buf[0x05] = 0x0805 */
#define U4_LANE_PRESENT                  0x19   /* u4_work_buf[0x19] = 0x0819 */
#define U4_LANE_PRESENT_CAP20G           0x1A   /* u4_work_buf[0x1A] = 0x081A */
#define U4_DROM_CAP_FLAGS                0x1B   /* u4_work_buf[0x1B] = 0x081B */
#define LB_CLWALK_TX_SHADOW              0x1C   /* u4_work_buf[0x1C] = 0x081C */
#define LB_CLWALK_TX_SHADOW_HI           0x1D   /* u4_work_buf[0x1D] = 0x081D */
#define LB_OS_ARMED_SHADOW               0x1E   /* u4_work_buf[0x1E] = 0x081E */
#define LB_OS_ARMED_SHADOW_L1            0x1F   /* u4_work_buf[0x1F] = 0x081F */
#define SB_ROUTEROP_BODY                 0x00   /* sb_routerop_body[0x00] = 0x099C */
#define SB_ROUTEROP_BODY1                0x01   /* sb_routerop_body[0x01] = 0x099D */
#define SB_ROUTEROP_BODY2                0x02   /* sb_routerop_body[0x02] = 0x099E */
#define SB_ROUTEROP_BODY3                0x03   /* sb_routerop_body[0x03] = 0x099F */
#define PHY_LANE_CAP                     0x00   /* phy_lane_cap[0x00] = 0x0AB4 */
#define PHY_LANE_CFG_AB5                 0x01   /* phy_lane_cap[0x01] = 0x0AB5 */
#define LB_CL_STATUS                     0x00   /* lb_cl_status[0x00] = 0x0B26 */
#define LB_EQ_STATUS                     0x00   /* lb_eq_status[0x00] = 0x0B28 */
#define LB_LOOP2_SCRATCH                 0x00   /* lb_loop2_scratch[0x00] = 0x0B2A */
#define LB_CL0_WIDTH                     0x00   /* lb_cl0_width[0x00] = 0x0B2C */

/* ---- standalone firmware state: named __at globals at their stock addresses ----------- */
volatile __xdata __at(0x06E9) uint8_t phy_rxpll_train_busy;   /* PHY CDR/RXPLL train flag (1=start/busy, 0=ready) */
volatile __xdata __at(0x06EC) uint8_t u4_conn_consequence_done;   /* SB-router connect consequence ran; arms cb10 super-loop poll */
volatile __xdata __at(0x06ED) u4_fsm_state_t u4_fsm_state;   /* USB4 connect/lane-bond FSM state (3=ConnRout,4,5) */
volatile __xdata __at(0x06EE) uint8_t sb_transport_edge_toggle;   /* d4cd transport-edge ping-pong toggle, ports 0/1 (SB28/2A) */
volatile __xdata __at(0x06EF) uint8_t sb_link_edge_toggle;   /* d4cd LINK-edge ping-pong toggle, ports 2/3 (SB81/83) */
volatile __xdata __at(0x06F0) uint8_t sb_active_plane_port;   /* cd3f port index selecting SB-RX descriptor plane (ROM 212d) */
volatile __xdata __at(0x06F1) uint8_t sb_active_port_rr;   /* active-port round-robin index for SB[0x20..] reg pairs */
volatile __xdata __at(0x0718) uint8_t u4_route_enable_latch;   /* ROUTE-ENABLE latch (=4) consumed by tunnel bring-up/state-5 */
volatile __xdata __at(0x0719) uint8_t e461_inflight_token;   /* e461 route-query in-flight token (1=sent), cleared by eda0 */
volatile __xdata __at(0x072A) uint8_t sb_cdf5_substate_arm;   /* cdf5 SB-internal substate arm (set=1 in state-3 a74b) */
volatile __xdata __at(0x072B) uint8_t lb_laneA_cl_latch;   /* lane A CL-state latch vs SB[0xA0] nibble (seed 0x07) */
volatile __xdata __at(0x072C) uint8_t lb_laneB_cl_latch;   /* lane B CL-state latch vs SB[0xA1] nibble (seed 0x07) */
volatile __xdata __at(0x072D) uint8_t lb_lane_bonded_flag;   /* Lane-Bonded flag set by eed6 */
volatile __xdata __at(0x074E) uint8_t lb_laneA_cl0_latch;   /* per-lane(A) CL0 confirm latch, zeroed by 9874 */
volatile __xdata __at(0x074F) uint8_t lb_laneB_cl0_latch;   /* per-lane(B) CL0 confirm latch, zeroed by 9874 */
volatile __xdata __at(0x0750) uint8_t lb_lane_width_latch0;   /* lane-width latch (=2 iff 077a.5&081a.5); ConnRout sets */
volatile __xdata __at(0x0751) uint8_t lb_lane_width_latch1;   /* lane-width latch (=1 iff 077a.4&081a.4&0819.0&.1) */
volatile __xdata __at(0x0752) uint8_t sb_connect_descriptor;   /* host SB connect descriptor (from SB[0x18]); af38/cd3f gate */
volatile __xdata __at(0x0753) uint8_t sb_tx_command_desc;   /* SB[0x15] TX command = 0x0752 & 0xDE (bits 0,5 cleared) */
volatile __xdata __at(0x0754) uint8_t sb_af38_copy_len;   /* af38 response copy length / work-buffer byte count */
volatile __xdata __at(0x0758) connrt_substate_t cm_conn_routing_substate;   /* cm_conn_routing_setup sub-FSM state (0x10/0x11/0x00) */
volatile __xdata __at(0x0763) uint8_t u4_phy_gate_a;   /* deep-PHY gate input A (0763|0764 e08d/9a06); ConnRout clears */
volatile __xdata __at(0x0764) uint8_t u4_phy_gate_b;   /* deep-PHY gate input B (0763|0764 gate); ConnRout clears */
volatile __xdata __at(0x0765) uint8_t sb_connect_present;   /* connect-present latch (ebb5/ebd7 set=1); b0b4 gate input */
volatile __xdata __at(0x0766) uint8_t sb_route_up_trigger;   /* route-up trigger (c4a3 set=1), consumed by ConnRout/CM */
volatile __xdata __at(0x0767) uint8_t lb_walk_oneshot_flag;   /* state-5 walker one-shot guard flag (set on first pass) */
volatile __xdata __at(0x0768) uint8_t lb_lane_width_cnt_hi;   /* lane-width counter snapshot hi = CCE4 (98f5) */
volatile __xdata __at(0x0769) uint8_t lb_lane_width_cnt_lo;   /* lane-width counter snapshot lo = CCE5 (98fa) */
volatile __xdata __at(0x076A) uint8_t lb_walk_throttle_snap_hi;   /* cb10 running CCE4 snapshot (e672 width-delta throttle) */
volatile __xdata __at(0x076B) uint8_t lb_walk_throttle_snap_lo;   /* cb10 running CCE5 snapshot */
volatile __xdata __at(0x0774) uint8_t u4_lane_train_trigger;   /* ec51 lane-train trigger toggled ^=1; state-5 [Trig] */
volatile __xdata __at(0x0775) uint8_t u4_route_query_response;   /* host route-query response flag set by eaac (cleared by eda0) */
volatile __xdata __at(0x0776) uint8_t u4_coldboot_seed_gate;   /* e391 cold-boot width-LUT seed gate / connect-confirm (==0) */
volatile __xdata __at(0x07B7) uint8_t pd_tx_staged_pending;   /* PD staged-TX pending flag (commit when !=0) */
volatile __xdata __at(0x07B8) uint8_t pd_contract_state;   /* PD power contract level (1/2/3/4 from voltage decode) */
volatile __xdata __at(0x07B9) uint8_t u4_connect_route_latch;   /* USB4 connect-route latch (Enter_TBT sets; gates ConnRout) */
volatile __xdata __at(0x07BA) uint8_t u4_enter_usb_accepted;   /* Enter_USB Accept latch -> drives usb4_connect_u4 */
volatile __xdata __at(0x07BB) uint8_t u4_connect_pending;   /* connect-pending flag (Connect_U4 trigger from VDM) */
volatile __xdata __at(0x07BC) uint8_t pd_role_state;   /* PD role/contract state */
volatile __xdata __at(0x07BD) uint8_t pd_msg_substate;   /* PD message dispatch substate (1=init,3=wait) */
volatile __xdata __at(0x07BE) uint8_t pd_usb3_fallback_flag;   /* a440 gate flag; when 0 forces 09FA/09FB USB3 fallback */
volatile __xdata __at(0x07BF) uint8_t pd_rx_ptr_hi;   /* PD RX buffer pointer high byte (96d4) */
volatile __xdata __at(0x07C0) uint8_t pd_rx_ptr_lo;   /* PD RX buffer pointer low byte (96d4) */
volatile __xdata __at(0x07C1) uint8_t pd_rx_slot_idx;   /* PD RX slot/MessageID index, rotated mod 0x07D5 */
volatile __xdata __at(0x07C2) uint8_t pd_rx_num_data_obj;   /* RX NumDataObjects = (hdr1>>4)&7 */
volatile __xdata __at(0x07C3) uint8_t pd_tx_msgid_counter;   /* PD TX MessageID counter, bumped (+1)&7 on commit */
volatile __xdata __at(0x07C4) uint8_t pd_tx_msg_len;   /* PD TX byte-length descriptor (2 hdr + 4*NumVDOs) -> E403 */
volatile __xdata __at(0x07C5) uint8_t pd_pdo_selection_valid;   /* PDO selection valid flag (abf5 sets=1) */
volatile __xdata __at(0x07C7) uint8_t pd_selected_pdo_idx;   /* selected PDO index (RDO object position = +1) */
volatile __xdata __at(0x07C8) uint8_t pd_bist_mode;   /* BIST mode flag (dc65 set when RX[1]&0xF0==0x80) */
volatile __xdata __at(0x07CA) uint8_t pd_sop_field;   /* PD SOP/port field = hdr[0]>>6 (1=SOP,2/3=SOP'/cable) */
volatile __xdata __at(0x07CB) uint8_t pd_state_07cb;   /* PD state cell cleared at init; exact use unknown */
volatile __xdata __at(0x07CC) uint8_t u4_route_confirm_07cc;   /* connect/route confirm count gate (07BA&&07CC<3 path) */
volatile __xdata __at(0x07CD) uint8_t u4_confirm_input_cd;   /* 0776 connect-confirm input (07CE!=0 && 07CD==0) */
volatile __xdata __at(0x07CE) uint8_t u4_confirm_input_ce;   /* 0776 connect-confirm input (07CE!=0 && 07CD==0) */
volatile __xdata __at(0x07CF) uint8_t u4_confirm_input_cf;   /* connect-confirm input (07B9 && 07CF in {1,2}) */
volatile __xdata __at(0x07D3) uint8_t pd_op_current_hi;   /* selected PDO operating-current bits 9:8 (RDO pack) */
volatile __xdata __at(0x07D4) uint8_t pd_op_current_lo;   /* selected PDO operating-current bits 7:0 (RDO pack) */
volatile __xdata __at(0x07D5) uint8_t pd_rx_slot_mask;   /* RX slot count/mask from E400.6 (0x10 or 0x01) */
volatile __xdata __at(0x07D6) uint8_t pd_decoded_voltage_hi;   /* decoded source-cap voltage hi (==1 for 5V path) */
volatile __xdata __at(0x07D7) uint8_t pd_decoded_voltage_lo;   /* decoded source-cap voltage lo (==0x2C for 5V) */
volatile __xdata __at(0x07DA) uint8_t pd_timer_a;   /* PD timer cell A (init=1) */
volatile __xdata __at(0x07DB) uint8_t pd_timer_b;   /* PD timer cell B (init=0x2C) */
volatile __xdata __at(0x07DC) uint8_t pd_timer_c;   /* PD timer cell C (init=0) */
volatile __xdata __at(0x07DD) uint8_t pd_timer_d;   /* PD timer cell D (init=0x64) */
volatile __xdata __at(0x07DE) uint8_t pd_softreset_pending;   /* PD soft-reset/ignore-and-reset pending flag */
volatile __xdata __at(0x07DF) uint8_t pd_hardreset_done;   /* PD hard-reset-done flag */
volatile __xdata __at(0x07E0) uint8_t pd_timer_e;   /* PD timer cell E (init=5) */
volatile __xdata __at(0x07E1) uint8_t pd_timer_f;   /* PD timer cell F (init=0) */
volatile __xdata __at(0x07E2) uint8_t pd_timer_g;   /* PD timer cell G (init=0) */
volatile __xdata __at(0x07E3) uint8_t pd_state_07e3;   /* PD state cell cleared at init; exact use unknown */
volatile __xdata __at(0x07E8) uint8_t u4_connect_gate_e8;   /* USB4 route/connect gate term (c9a8); set on route entry */
volatile __xdata __at(0x07EB) uint8_t u4_connect_gate_eb;   /* USB4 connect gate alt term (c9a8: 07E8||07EB) */
volatile __xdata __at(0x07ED) uint8_t u4_connect_oneshot_suppress;   /* [Connect_U4] one-shot suppress (a17c: skip usb4_connect_u4) */
volatile __xdata __at(0x07FF) uint8_t pd_cm_dispatch_sel;   /* CM sub-handler dispatch selector (==0x69 -> e545) */
volatile __xdata __at(0x097A) uint8_t sb_descr_engine_scratch;   /* SB descriptor-engine scratch flag, cleared in e5b0 engine setup */
volatile __xdata __at(0x0998) uint8_t sb_routerop_hdr0;   /* SB router-op descriptor header byte0 (host-posted; snapshot to/from 0x0A9D) */
volatile __xdata __at(0x0999) uint8_t sb_routerop_hdr1;   /* SB router-op descriptor header byte1 (snapshot to/from 0x0A9E) */
volatile __xdata __at(0x099A) uint8_t sb_routerop_hdr2;   /* SB router-op descriptor header byte2 (snapshot to/from 0x0A9F) */
volatile __xdata __at(0x099B) uint8_t sb_routerop_hdr3;   /* SB router-op descriptor header byte3 (valid bit7; snapshot to/from 0x0AA0) */
volatile __xdata __at(0x09F0) uint8_t sb_notify_flag0;   /* SB event/notify flag, cleared in e5b0 engine setup */
volatile __xdata __at(0x09F1) uint8_t sb_notify_flag1;   /* SB event/notify flag, cleared in e5b0 engine setup */
volatile __xdata __at(0x09F2) uint8_t sb_notify_flag2;   /* SB notify flag; if set notifies bank0 stub 05a7(1) (no XDATA effect) */
volatile __xdata __at(0x09F3) uint8_t sb_notify_flag3;   /* SB notify flag; if set notifies bank0 stub 05a7(2) */
volatile __xdata __at(0x09F4) uint8_t u4_dp_alt_mode;   /* DP-alt mode selector; ==0x03 triggers DP-alt route sub-case in connect_u4 */
volatile __xdata __at(0x09F5) uint8_t u4_cap20g_gate0;   /* 20G lane-present capability gate; if set, DROM[0x1A] |= 0x20 */
volatile __xdata __at(0x09F6) uint8_t u4_cap20g_gate1;   /* 20G capability gate; if ==0, DROM[0x1A] &= 0xED */
volatile __xdata __at(0x09F9) uint8_t u4_mode_flag;   /* USB4 runtime mode flag: bit7=VDM-ACK enable, bits1:0=route (0x87=tunnel) */
volatile __xdata __at(0x09FA) uint8_t u4_route_mode;   /* USB4 route-mode latch: bit0/bit1=lane route, bit2=connect-gate bit */
volatile __xdata __at(0x09FB) uint8_t u4_lane_gate_sel;   /* ccb3 sub-descriptor lane-gate selector (bits0/1 gate the two sub-descriptors) */
volatile __xdata __at(0x0A52) uint8_t u4_tunnel_cfg_hi;    /* cd6c/c8db: B411/B41B src (stock SPI 0x7074) */
volatile __xdata __at(0x0A53) uint8_t u4_tunnel_cfg_lo;    /* cd6c/c8db: B410/B41A src (stock SPI 0x7075) */
volatile __xdata __at(0x0A54) uint8_t u4_tunnel_cfg_mode;  /* cd6c/c8db: B413/B419 src (stock SPI 0x7076) */
volatile __xdata __at(0x0A55) uint8_t u4_tunnel_credits;   /* cd6c/c8db: B412/B418 src (stock SPI 0x7077) */
volatile __xdata __at(0x0A57) uint8_t pd_product_pid_lo;   /* PD RAM Product VDO PID/bcdDevice low byte */
volatile __xdata __at(0x0A58) uint8_t pd_product_pid_hi;   /* PD RAM Product VDO PID/bcdDevice high byte */
volatile __xdata __at(0x0A5C) uint8_t u4lb_width_rate_code;   /* lane-bond Chg2 rate/width code (d3b0/d436) */
volatile __xdata __at(0x0A5D) uint8_t u4lb_gen_index;   /* gen index into a840 speed table 38cc */
volatile __xdata __at(0x0A7D) uint8_t u4_lane_mode_arg;   /* lane-MODE arg passed to bank0_8a89 */
volatile __xdata __at(0x0A9D) uint8_t u4_routerop_desc0;   /* a5d8 router-op descriptor byte0 / connect mode */
volatile __xdata __at(0x0A9E) uint8_t u4_routerop_desc1;   /* a5d8 router-op descriptor byte1 / link-rate latch */
volatile __xdata __at(0x0A9F) uint8_t u4_routerop_desc2;   /* a5d8 router-op descriptor byte2 / link-width latch */
volatile __xdata __at(0x0AA0) uint8_t u4_routerop_desc3;   /* a5d8 router-op descriptor byte3, bit7=valid */
volatile __xdata __at(0x0AA1) uint8_t pd_msg_type;   /* PD MessageType; also router-op copy-loop index */
/* Conn-routing width-advert tail (e175/e282/c17f/ce23 P12-engine staging). */
volatile __xdata __at(0x09DD) uint8_t u4lb_lane_active_flags;  /* e175 sets .0, e282 sets .1; ce23/e00c read */
volatile __xdata __at(0x0B34) uint8_t u4lb_b34_lanemask;       /* ce23 lane-mask staging / e00c source[0] */
volatile __xdata __at(0x0B35) uint8_t u4lb_b35;                /* engine sub-block staging / a37b dest */
volatile __xdata __at(0x0B36) uint8_t u4lb_b36;                /* desc byte staging */
volatile __xdata __at(0x0B37) uint8_t u4lb_b37;                /* desc byte staging (=0x0B12=0) */
volatile __xdata __at(0x0B38) uint8_t u4lb_b38_setlanes;       /* c3ce set_lanes selector (=0x02) */
volatile __xdata __at(0x0AA2) uint8_t u4_routerop_op_lo;   /* router-op slot/MessageID; port-map index */
volatile __xdata __at(0x0AA3) uint8_t u4_routerop_op_len;   /* router-op length; PD ctrl-msg code latch */
volatile __xdata __at(0x0AA4) uint8_t u4_routerop_opcode;   /* router-op opcode = desc2 & 7 */
volatile __xdata __at(0x0AA5) uint8_t u4_routerop_flag;   /* router-op valid/route flag = desc3 & 1 */
volatile __xdata __at(0x0AA6) uint8_t u4_routerop_port;   /* router-op mapped port = lane_port_map_a[op_lo] */
volatile __xdata __at(0x0AA7) uint8_t u4_routerop_svid_hi;   /* router-op desc2>>4; VDM SVID high */
volatile __xdata __at(0x0AA8) uint8_t sb_tx_cmd;   /* SB-transport TX command byte (SB[0x15]) */
volatile __xdata __at(0x0AA9) uint8_t sb_tx_byte0;   /* SB-transport TX SBTX[0] payload byte */
volatile __xdata __at(0x0AAA) uint8_t sb_tx_byte1;   /* SB-transport TX SBTX[1] payload byte */
volatile __xdata __at(0x0AAB) uint8_t sb_tx_flag;   /* SB-transport TX flag (SBTX[1].7, SB[0x0C] form) */
volatile __xdata __at(0x0AAC) uint8_t sb_tx_go_param;   /* d5da TX-go param selector (0=plain TX trigger) */
volatile __xdata __at(0x0AAD) u4_fsm_state_t sb_fsm_state;   /* eb62 new SB connect-FSM state (mirrors 0x06ED) */
volatile __xdata __at(0x0AB3) uint8_t phy_lane_gate;   /* e34b PHY-lane gate; 0=emit OS1/retrain tail */
volatile __xdata __at(0x0AB6) uint8_t phy_cdr_arm_mask;   /* e34b CDR-done arm mask (C2D1/C351) */
volatile __xdata __at(0x0ACD) uint8_t u4_mode_entry_class;   /* mode-entry class (3=USB4 path,1=non-USB4) */
volatile __xdata __at(0x0ACE) uint8_t u4_mode_entry_param;   /* mode-entry reply-len/timing param (1/0x0D/5) */
volatile __xdata __at(0x0AE2) uint8_t u4_entered_usb_mode;   /* latched USB-mode from Enter_USB VDM/d78a */
volatile __xdata __at(0x0AE3) uint8_t u4_link_busy;   /* USB4 tunnel link-busy flag; 0=link complete */
volatile __xdata __at(0x0AEC) uint8_t u4_link_gen;   /* USB4 link gen (a840 R7; seeded 3) */
volatile __xdata __at(0x0AED) uint8_t u4_link_lane;   /* USB4 link lane (a840 R6; seeded 3) */
volatile __xdata __at(0x0AF1) uint8_t u4_connect_gate;   /* USB4 connect-time gate/mode flags (.0 gate,.4 PHY-lock) */
volatile __xdata __at(0x0B02) rmbox_state_t u4_routerop_mbox_state;   /* router-op mailbox state (0=idle,1=read,2=write) */
volatile __xdata __at(0x0B03) uint8_t u4_routerop_mbox_opcode;   /* router-op mailbox opcode from EA80 (E2/E3) */
volatile __xdata __at(0x0B13) uint8_t sb_eng_data3c_b;   /* SB descriptor-engine ENGINE[0x3C] data (variant b) */
volatile __xdata __at(0x0B14) uint8_t sb_eng_data3d_b;   /* SB descriptor-engine ENGINE[0x3D] data (variant b) */
volatile __xdata __at(0x0B15) uint8_t sb_eng_data3e_b;   /* SB descriptor-engine ENGINE[0x3E] data (variant b) */
volatile __xdata __at(0x0B16) uint8_t sb_eng_data3f_b;   /* SB descriptor-engine ENGINE[0x3F] data (variant b) */
volatile __xdata __at(0x0B17) uint8_t sb_eng_data3c_a;   /* SB descriptor-engine ENGINE[0x3C] data (variant a) */
volatile __xdata __at(0x0B18) uint8_t sb_eng_data3d_a;   /* SB descriptor-engine ENGINE[0x3D] data (variant a) */
volatile __xdata __at(0x0B19) uint8_t sb_eng_data_lo;   /* SB descriptor-engine data low; copied to 0x0247 */
volatile __xdata __at(0x0B1A) uint8_t sb_eng_data_hi;   /* SB descriptor-engine data high; copied to 0x0248 */
volatile __xdata __at(0x0B1B) uint8_t cc_subdemux_src;   /* CCF9.1 sub-demux source (copied to 0x0A9D); e74e clears */
volatile __xdata __at(0x0B1C) uint8_t sb_link_reinit_gate;   /* SB[0x66] abort/bond-fail link-reinit gate */
volatile __xdata __at(0x0B2F) uint8_t u4_reinit_pending;   /* USB4 connect/SB-reconnect reinit-pending flag */

#endif
