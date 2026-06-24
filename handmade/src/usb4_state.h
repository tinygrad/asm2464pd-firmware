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

#define U4_XDATA_BYTES(sym) ((volatile __xdata uint8_t *)&(sym))
#define U4_ROUTEROP_MBOX_CLEAR_LEN 0x10u

/* Fixed XDATA map shared with stock-style PD, sideband, and router-op flows. */

/* Sideband descriptor tables and host work buffers. */
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
volatile __xdata uint8_t phy_lane_cap[0x2];
volatile __xdata __at(0x0B26) uint8_t lb_cl_status[0x2];
volatile __xdata __at(0x0B28) uint8_t lb_eq_status[0x2];
volatile __xdata __at(0x0B2A) uint8_t lb_loop2_scratch[0x2];
volatile __xdata __at(0x0B2C) uint8_t lb_cl0_width[0x2];

/* Sideband and lane-bond FSM state. */
volatile __xdata uint8_t phy_rxpll_train_busy;
volatile __xdata uint8_t u4_conn_consequence_done;
volatile __xdata u4_fsm_state_t u4_fsm_state;
volatile __xdata uint8_t sb_transport_edge_toggle;
volatile __xdata uint8_t sb_link_edge_toggle;
volatile __xdata uint8_t sb_active_plane_port;
volatile __xdata uint8_t sb_active_port_rr;
volatile __xdata uint8_t u4_route_enable_latch;
volatile __xdata uint8_t e461_inflight_token;
volatile __xdata uint8_t sb_cdf5_substate_arm;
volatile __xdata uint8_t lb_laneA_cl_latch;
volatile __xdata uint8_t lb_laneB_cl_latch;
volatile __xdata uint8_t lb_lane_bonded_flag;
volatile __xdata uint8_t lb_laneA_cl0_latch;
volatile __xdata uint8_t lb_laneB_cl0_latch;
volatile __xdata uint8_t lb_lane_width_latch0;
volatile __xdata uint8_t lb_lane_width_latch1;
volatile __xdata uint8_t sb_connect_descriptor;
volatile __xdata uint8_t sb_tx_command_desc;
volatile __xdata uint8_t sb_af38_copy_len;
volatile __xdata connrt_substate_t cm_conn_routing_substate;
volatile __xdata uint8_t u4_phy_gate_a;
volatile __xdata uint8_t u4_phy_gate_b;
volatile __xdata uint8_t sb_connect_present;
volatile __xdata uint8_t sb_route_up_trigger;
volatile __xdata uint8_t lb_walk_oneshot_flag;
volatile __xdata uint8_t lb_lane_width_cnt_hi;
volatile __xdata uint8_t lb_lane_width_cnt_lo;
volatile __xdata uint8_t lb_walk_throttle_snap_hi;
volatile __xdata uint8_t lb_walk_throttle_snap_lo;
volatile __xdata uint8_t u4_lane_train_trigger;
volatile __xdata uint8_t u4_route_query_response;
volatile __xdata uint8_t u4_coldboot_seed_gate;

/* USB-PD policy state and USB4 mode-entry latches. */
volatile __xdata uint8_t pd_contract_state;
volatile __xdata uint8_t u4_connect_route_latch;
volatile __xdata uint8_t u4_enter_usb_accepted;
volatile __xdata uint8_t u4_connect_pending;
volatile __xdata uint8_t pd_role_state;
volatile __xdata uint8_t pd_msg_substate;
volatile __xdata uint8_t pd_usb3_fallback_flag;
volatile __xdata uint8_t pd_rx_slot_idx;
volatile __xdata uint8_t pd_rx_num_data_obj;
volatile __xdata uint8_t pd_tx_msgid_counter;
volatile __xdata uint8_t pd_tx_msg_len;
volatile __xdata uint8_t pd_bist_mode;
volatile __xdata uint8_t pd_sop_field;
volatile __xdata uint8_t pd_state_07cb;
volatile __xdata uint8_t u4_route_confirm_07cc;
volatile __xdata uint8_t u4_confirm_input_cd;
volatile __xdata uint8_t u4_confirm_input_ce;
volatile __xdata uint8_t u4_confirm_input_cf;
volatile __xdata uint8_t pd_rx_slot_mask;
volatile __xdata uint8_t pd_decoded_voltage_hi;
volatile __xdata uint8_t pd_decoded_voltage_lo;
volatile __xdata uint8_t pd_timer_a;
volatile __xdata uint8_t pd_timer_b;
volatile __xdata uint8_t pd_timer_c;
volatile __xdata uint8_t pd_timer_d;
volatile __xdata uint8_t pd_softreset_pending;
volatile __xdata uint8_t pd_hardreset_done;
volatile __xdata uint8_t pd_timer_e;
volatile __xdata uint8_t pd_timer_f;
volatile __xdata uint8_t pd_timer_g;
volatile __xdata uint8_t pd_state_07e3;
volatile __xdata uint8_t u4_connect_gate_e8;
volatile __xdata uint8_t u4_connect_gate_eb;
volatile __xdata uint8_t u4_connect_oneshot_suppress;
volatile __xdata uint8_t pd_cm_dispatch_sel;

/* Router-op and sideband mailbox state. */
volatile __xdata uint8_t sb_descr_engine_scratch;
volatile __xdata uint8_t sb_routerop_hdr0;
volatile __xdata uint8_t sb_routerop_hdr1;
volatile __xdata uint8_t sb_routerop_hdr2;
volatile __xdata uint8_t sb_routerop_hdr3;

/* Stock capability/config shadows seeded at boot. */
volatile __xdata uint8_t u4_dp_alt_mode;
volatile __xdata uint8_t u4_cap20g_gate0;
volatile __xdata uint8_t u4_cap20g_gate1;
volatile __xdata uint8_t u4_sb_desc_profile;
volatile __xdata uint8_t u4_capability_profile;
volatile __xdata uint8_t u4_mode_flag;
volatile __xdata uint8_t u4_route_mode;
volatile __xdata uint8_t u4_lane_gate_sel;
volatile __xdata uint8_t u4_tunnel_cfg_hi;
volatile __xdata uint8_t u4_tunnel_cfg_lo;
volatile __xdata uint8_t u4_tunnel_cfg_mode;
volatile __xdata uint8_t u4_tunnel_credits;
volatile __xdata uint8_t pd_product_pid_lo;
volatile __xdata uint8_t pd_product_pid_hi;
volatile __xdata uint8_t u4lb_width_rate_code;
volatile __xdata uint8_t u4lb_gen_index;
volatile __xdata uint8_t u4_lane_mode_arg;
volatile __xdata uint8_t u4_routerop_desc0;
volatile __xdata uint8_t u4_routerop_desc1;
volatile __xdata uint8_t u4_routerop_desc2;
volatile __xdata uint8_t u4_routerop_desc3;
volatile __xdata uint8_t pd_msg_type;

/* Lane-bond descriptor-engine scratch. */
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
volatile __xdata __at(0x0AB3) uint8_t phy_lane_gate;
volatile __xdata uint8_t phy_cdr_arm_mask;
volatile __xdata __at(0x0ACD) uint8_t u4_mode_entry_class;
volatile __xdata __at(0x0ACE) uint8_t u4_mode_entry_param;
volatile __xdata __at(0x0AE2) uint8_t u4_entered_usb_mode;
volatile __xdata __at(0x0AE3) uint8_t u4_link_busy;
volatile __xdata __at(0x0AEC) uint8_t u4_link_gen;
volatile __xdata __at(0x0AED) uint8_t u4_link_lane;
volatile __xdata __at(0x0AF1) uint8_t u4_connect_gate;

/* Router-op response staging. */
volatile __xdata __at(0x0B02) rmbox_state_t u4_routerop_mbox_state;
volatile __xdata __at(0x0B03) uint8_t u4_routerop_mbox_opcode;

volatile __xdata __at(0x0B04) uint8_t u4_rop_cfg_addr[4];
volatile __xdata __at(0x0B0A) uint8_t u4_rop_limit[4];

/* P12 descriptor-engine temporaries. */
volatile __xdata uint8_t sb_eng_lane_profile;
volatile __xdata uint8_t sb_eng_data3c_b;
volatile __xdata uint8_t sb_eng_data3d_b;
volatile __xdata uint8_t sb_eng_data3e_b;
volatile __xdata uint8_t sb_eng_data3f_b;
volatile __xdata uint8_t sb_eng_data3c_a;
volatile __xdata uint8_t sb_eng_data3d_a;
volatile __xdata uint8_t sb_eng_data_lo;
volatile __xdata uint8_t sb_eng_data_hi;
volatile __xdata uint8_t cc_subdemux_src;
volatile __xdata uint8_t sb_link_reinit_gate;
volatile __xdata uint8_t u4_reinit_pending;

volatile __xdata uint8_t pcie_ctrl_b402_shadow;
volatile __xdata uint8_t pd_seen;
volatile __xdata uint8_t sb_asserted;
volatile __xdata uint8_t tup_e52d_done;

#endif
