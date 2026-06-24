#ifndef USB4_STATE_H
#define USB4_STATE_H

typedef enum {
    U4FSM_IDLE       = 0x00,
    U4FSM_CONN_ROUT  = 0x03,
    U4FSM_LANE_TRAIN = 0x04,
    U4FSM_LANE_BOND  = 0x05,
} u4_fsm_state_t;
_Static_assert(sizeof(u4_fsm_state_t) == 1, "u4_fsm_state_t must be 1 byte");

typedef enum {
    CONNRT_PRINT_STATUS    = 0x00,
    CONNRT_ARM_ROUTE_QUERY = 0x10,
    CONNRT_AWAIT_RESULT    = 0x11,
} connrt_substate_t;
_Static_assert(sizeof(connrt_substate_t) == 1, "connrt_substate_t must be 1 byte");

typedef enum {
    LP1_PARKED            = 0x00,
    LP1_WIDTH_INIT        = 0x10,
    LP1_ARM_WAIT_PUSH     = 0x20,
    LP1_LANE_PRESENT_SEL  = 0x30,
    LP1_SETTLE_CLEAR      = 0x40,
    LP1_WIDTH_SETTLE_WALK = 0x50,
    LP1_BOND_WAIT_PUSH    = 0x60,
    LP1_WIDTH_LATCH_SEL   = 0x70,
    LP1_FINALIZE_A        = 0x80,
    LP1_FINALIZE_B        = 0x90,
    LP1_BOND_WAIT_ACK     = 0xA0,
    LP1_BONDED_MONITOR    = 0xA1,
} lp1_state_t;
_Static_assert(sizeof(lp1_state_t) == 1, "lp1_state_t must be 1 byte");

typedef enum {
    LP2_CL_IDLE      = 0x00,
    LP2_CL_INIT      = 0x10,
    LP2_CL_PUSH_WAIT = 0x20,
    LP2_CL_EVAL      = 0x30,
    LP2_CL_BOND_WAIT = 0x50,
    LP2_CL_BOND_MON  = 0x60,
} lp2_state_t;
_Static_assert(sizeof(lp2_state_t) == 1, "lp2_state_t must be 1 byte");

typedef enum {
    RMBOX_IDLE       = 0x00,
    RMBOX_MULTIPKT_1 = 0x01,
    RMBOX_MULTIPKT_2 = 0x02,
} rmbox_state_t;
_Static_assert(sizeof(rmbox_state_t) == 1, "rmbox_state_t must be 1 byte");

volatile __xdata __at(0x0600) uint8_t sb_cfg06[0x10];
volatile __xdata __at(0x06F2) uint8_t sb_width_lut[0x13];
volatile __xdata __at(0x0705) uint8_t sb_branchA_gate[0x13];
volatile __xdata __at(0x071A) uint8_t sb_lane_desc[0x10];
volatile __xdata __at(0x072E) uint8_t lb_cap_field[0x10];
volatile __xdata __at(0x073E) uint8_t sb_lane_flip[0x10];
volatile __xdata __at(0x0759) lp1_state_t lb_loop1_state[0x2];
volatile __xdata __at(0x075B) lp2_state_t lb_loop2_state[0x2];
volatile __xdata __at(0x075D) uint8_t lb_lane_desc_idx[0x2];
volatile __xdata __at(0x075F) uint8_t lb_settle_counter[0x2];
volatile __xdata __at(0x0761) uint8_t lb_cl_value[0x2];
volatile __xdata __at(0x076C) uint8_t lb_width_pairA[0x4];
volatile __xdata __at(0x0770) uint8_t lb_width_pairB[0x4];
volatile __xdata __at(0x0777) uint8_t u4_host_desc[0x40];
volatile __xdata __at(0x0800) uint8_t u4_work_buf[0x64];
volatile __xdata __at(0x099C) uint8_t sb_routerop_body[0x40];
volatile __xdata __at(0x0AB4) uint8_t phy_lane_cap[0x2];
volatile __xdata __at(0x0B26) uint8_t lb_cl_status[0x2];
volatile __xdata __at(0x0B28) uint8_t lb_eq_status[0x2];
volatile __xdata __at(0x0B2A) uint8_t lb_loop2_scratch[0x2];
volatile __xdata __at(0x0B2C) uint8_t lb_cl0_width[0x2];

volatile __xdata __at(0x06E9) uint8_t phy_rxpll_train_busy;
volatile __xdata __at(0x06EC) uint8_t u4_conn_consequence_done;
volatile __xdata __at(0x06ED) u4_fsm_state_t u4_fsm_state;
volatile __xdata __at(0x06EE) uint8_t sb_transport_edge_toggle;
volatile __xdata __at(0x06EF) uint8_t sb_link_edge_toggle;
volatile __xdata __at(0x06F0) uint8_t sb_active_plane_port;
volatile __xdata __at(0x06F1) uint8_t sb_active_port_rr;
volatile __xdata __at(0x0718) uint8_t u4_route_enable_latch;
volatile __xdata __at(0x0719) uint8_t e461_inflight_token;
volatile __xdata __at(0x072A) uint8_t sb_cdf5_substate_arm;
volatile __xdata __at(0x072B) uint8_t lb_laneA_cl_latch;
volatile __xdata __at(0x072C) uint8_t lb_laneB_cl_latch;
volatile __xdata __at(0x072D) uint8_t lb_lane_bonded_flag;
volatile __xdata __at(0x074E) uint8_t lb_laneA_cl0_latch;
volatile __xdata __at(0x074F) uint8_t lb_laneB_cl0_latch;
volatile __xdata __at(0x0750) uint8_t lb_lane_width_latch0;
volatile __xdata __at(0x0751) uint8_t lb_lane_width_latch1;
volatile __xdata __at(0x0752) uint8_t sb_connect_descriptor;
volatile __xdata __at(0x0753) uint8_t sb_tx_command_desc;
volatile __xdata __at(0x0754) uint8_t sb_af38_copy_len;
volatile __xdata __at(0x0758) connrt_substate_t cm_conn_routing_substate;
volatile __xdata __at(0x0763) uint8_t u4_phy_gate_a;
volatile __xdata __at(0x0764) uint8_t u4_phy_gate_b;
volatile __xdata __at(0x0765) uint8_t sb_connect_present;
volatile __xdata __at(0x0766) uint8_t sb_route_up_trigger;
volatile __xdata __at(0x0767) uint8_t lb_walk_oneshot_flag;
volatile __xdata __at(0x0768) uint8_t lb_lane_width_cnt_hi;
volatile __xdata __at(0x0769) uint8_t lb_lane_width_cnt_lo;
volatile __xdata __at(0x076A) uint8_t lb_walk_throttle_snap_hi;
volatile __xdata __at(0x076B) uint8_t lb_walk_throttle_snap_lo;
volatile __xdata __at(0x0774) uint8_t u4_lane_train_trigger;
volatile __xdata __at(0x0775) uint8_t u4_route_query_response;
volatile __xdata __at(0x0776) uint8_t u4_coldboot_seed_gate;
volatile __xdata __at(0x07B7) uint8_t pd_tx_staged_pending;
volatile __xdata __at(0x07B8) uint8_t pd_contract_state;
volatile __xdata __at(0x07B9) uint8_t u4_connect_route_latch;
volatile __xdata __at(0x07BA) uint8_t u4_enter_usb_accepted;
volatile __xdata __at(0x07BB) uint8_t u4_connect_pending;
volatile __xdata __at(0x07BC) uint8_t pd_role_state;
volatile __xdata __at(0x07BD) uint8_t pd_msg_substate;
volatile __xdata __at(0x07BE) uint8_t pd_usb3_fallback_flag;
volatile __xdata __at(0x07BF) uint8_t pd_rx_ptr_hi;
volatile __xdata __at(0x07C0) uint8_t pd_rx_ptr_lo;
volatile __xdata __at(0x07C1) uint8_t pd_rx_slot_idx;
volatile __xdata __at(0x07C2) uint8_t pd_rx_num_data_obj;
volatile __xdata __at(0x07C3) uint8_t pd_tx_msgid_counter;
volatile __xdata __at(0x07C4) uint8_t pd_tx_msg_len;
volatile __xdata __at(0x07C5) uint8_t pd_pdo_selection_valid;
volatile __xdata __at(0x07C7) uint8_t pd_selected_pdo_idx;
volatile __xdata __at(0x07C8) uint8_t pd_bist_mode;
volatile __xdata __at(0x07CA) uint8_t pd_sop_field;
volatile __xdata __at(0x07CB) uint8_t pd_state_07cb;
volatile __xdata __at(0x07CC) uint8_t u4_route_confirm_07cc;
volatile __xdata __at(0x07CD) uint8_t u4_confirm_input_cd;
volatile __xdata __at(0x07CE) uint8_t u4_confirm_input_ce;
volatile __xdata __at(0x07CF) uint8_t u4_confirm_input_cf;
volatile __xdata __at(0x07D3) uint8_t pd_op_current_hi;
volatile __xdata __at(0x07D4) uint8_t pd_op_current_lo;
volatile __xdata __at(0x07D5) uint8_t pd_rx_slot_mask;
volatile __xdata __at(0x07D6) uint8_t pd_decoded_voltage_hi;
volatile __xdata __at(0x07D7) uint8_t pd_decoded_voltage_lo;
volatile __xdata __at(0x07DA) uint8_t pd_timer_a;
volatile __xdata __at(0x07DB) uint8_t pd_timer_b;
volatile __xdata __at(0x07DC) uint8_t pd_timer_c;
volatile __xdata __at(0x07DD) uint8_t pd_timer_d;
volatile __xdata __at(0x07DE) uint8_t pd_softreset_pending;
volatile __xdata __at(0x07DF) uint8_t pd_hardreset_done;
volatile __xdata __at(0x07E0) uint8_t pd_timer_e;
volatile __xdata __at(0x07E1) uint8_t pd_timer_f;
volatile __xdata __at(0x07E2) uint8_t pd_timer_g;
volatile __xdata __at(0x07E3) uint8_t pd_state_07e3;
volatile __xdata __at(0x07E8) uint8_t u4_connect_gate_e8;
volatile __xdata __at(0x07EB) uint8_t u4_connect_gate_eb;
volatile __xdata __at(0x07ED) uint8_t u4_connect_oneshot_suppress;
volatile __xdata __at(0x07FF) uint8_t pd_cm_dispatch_sel;
volatile __xdata __at(0x097A) uint8_t sb_descr_engine_scratch;
volatile __xdata __at(0x0998) uint8_t sb_routerop_hdr0;
volatile __xdata __at(0x0999) uint8_t sb_routerop_hdr1;
volatile __xdata __at(0x099A) uint8_t sb_routerop_hdr2;
volatile __xdata __at(0x099B) uint8_t sb_routerop_hdr3;
volatile __xdata __at(0x09F0) uint8_t sb_notify_flag0;
volatile __xdata __at(0x09F1) uint8_t sb_notify_flag1;
volatile __xdata __at(0x09F2) uint8_t sb_notify_flag2;
volatile __xdata __at(0x09F3) uint8_t sb_notify_flag3;
volatile __xdata __at(0x09F4) uint8_t u4_dp_alt_mode;
volatile __xdata __at(0x09F5) uint8_t u4_cap20g_gate0;
volatile __xdata __at(0x09F6) uint8_t u4_cap20g_gate1;
volatile __xdata __at(0x09F9) uint8_t u4_mode_flag;
volatile __xdata __at(0x09FA) uint8_t u4_route_mode;
volatile __xdata __at(0x09FB) uint8_t u4_lane_gate_sel;
volatile __xdata __at(0x0A52) uint8_t u4_tunnel_cfg_hi;
volatile __xdata __at(0x0A53) uint8_t u4_tunnel_cfg_lo;
volatile __xdata __at(0x0A54) uint8_t u4_tunnel_cfg_mode;
volatile __xdata __at(0x0A55) uint8_t u4_tunnel_credits;
volatile __xdata __at(0x0A57) uint8_t pd_product_pid_lo;
volatile __xdata __at(0x0A58) uint8_t pd_product_pid_hi;
volatile __xdata __at(0x0A5C) uint8_t u4lb_width_rate_code;
volatile __xdata __at(0x0A5D) uint8_t u4lb_gen_index;
volatile __xdata __at(0x0A7D) uint8_t u4_lane_mode_arg;
volatile __xdata __at(0x0A9D) uint8_t u4_routerop_desc0;
volatile __xdata __at(0x0A9E) uint8_t u4_routerop_desc1;
volatile __xdata __at(0x0A9F) uint8_t u4_routerop_desc2;
volatile __xdata __at(0x0AA0) uint8_t u4_routerop_desc3;
volatile __xdata __at(0x0AA1) uint8_t pd_msg_type;

volatile __xdata __at(0x09DD) uint8_t u4lb_lane_active_flags;
volatile __xdata __at(0x0B34) uint8_t u4lb_b34_lanemask;
volatile __xdata __at(0x0B35) uint8_t u4lb_b35;
volatile __xdata __at(0x0B36) uint8_t u4lb_b36;
volatile __xdata __at(0x0B37) uint8_t u4lb_b37;
volatile __xdata __at(0x0B38) uint8_t u4lb_b38_setlanes;
volatile __xdata __at(0x0AA2) uint8_t u4_routerop_op_lo;
volatile __xdata __at(0x0AA3) uint8_t u4_routerop_op_len;
volatile __xdata __at(0x0AA4) uint8_t u4_routerop_opcode;
volatile __xdata __at(0x0AA5) uint8_t u4_routerop_flag;
volatile __xdata __at(0x0AA6) uint8_t u4_routerop_port;
volatile __xdata __at(0x0AA7) uint8_t u4_routerop_svid_hi;
volatile __xdata __at(0x0AA8) uint8_t sb_tx_cmd;
volatile __xdata __at(0x0AA9) uint8_t sb_tx_byte0;
volatile __xdata __at(0x0AAA) uint8_t sb_tx_byte1;
volatile __xdata __at(0x0AAB) uint8_t sb_tx_flag;
volatile __xdata __at(0x0AAC) uint8_t sb_tx_go_param;
volatile __xdata __at(0x0AAD) u4_fsm_state_t sb_fsm_state;
volatile __xdata __at(0x0AB3) uint8_t phy_lane_gate;
volatile __xdata __at(0x0AB6) uint8_t phy_cdr_arm_mask;
volatile __xdata __at(0x0ACD) uint8_t u4_mode_entry_class;
volatile __xdata __at(0x0ACE) uint8_t u4_mode_entry_param;
volatile __xdata __at(0x0AE2) uint8_t u4_entered_usb_mode;
volatile __xdata __at(0x0AE3) uint8_t u4_link_busy;
volatile __xdata __at(0x0AEC) uint8_t u4_link_gen;
volatile __xdata __at(0x0AED) uint8_t u4_link_lane;
volatile __xdata __at(0x0AF1) uint8_t u4_connect_gate;
volatile __xdata __at(0x0B02) rmbox_state_t u4_routerop_mbox_state;
volatile __xdata __at(0x0B03) uint8_t u4_routerop_mbox_opcode;

volatile __xdata __at(0x0B04) uint8_t u4_rop_cfg_addr[4];
volatile __xdata __at(0x0B08) uint8_t u4_rop_len_lo;
volatile __xdata __at(0x0B09) uint8_t u4_rop_len_hi;
volatile __xdata __at(0x0B0A) uint8_t u4_rop_limit[4];
volatile __xdata __at(0x0B0E) uint8_t u4_rop_wr_cursor[4];

static volatile __xdata uint8_t u4_rop_shadow_ptr[4];
static volatile __xdata uint8_t u4_rop_resp_hdr[2];
static volatile __xdata uint8_t u4_rop_dir;
static volatile __xdata uint8_t u4_rop_xfer_len;
volatile __xdata __at(0x0B13) uint8_t sb_eng_data3c_b;
volatile __xdata __at(0x0B14) uint8_t sb_eng_data3d_b;
volatile __xdata __at(0x0B15) uint8_t sb_eng_data3e_b;
volatile __xdata __at(0x0B16) uint8_t sb_eng_data3f_b;
volatile __xdata __at(0x0B17) uint8_t sb_eng_data3c_a;
volatile __xdata __at(0x0B18) uint8_t sb_eng_data3d_a;
volatile __xdata __at(0x0B19) uint8_t sb_eng_data_lo;
volatile __xdata __at(0x0B1A) uint8_t sb_eng_data_hi;
volatile __xdata __at(0x0B1B) uint8_t cc_subdemux_src;
volatile __xdata __at(0x0B1C) uint8_t sb_link_reinit_gate;
volatile __xdata __at(0x0B2F) uint8_t u4_reinit_pending;

#endif
