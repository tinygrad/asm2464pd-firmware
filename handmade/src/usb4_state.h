#ifndef USB4_STATE_H
#define USB4_STATE_H

/* u4_cfg.mode_flag bitfield layout: */
#define MODE_FLAG_ROUTE_MASK   0x03u  /* bits 0-1: route mode (0=router, 1=host, 2=device, 3=both) */
#define MODE_FLAG_USB3_DIRECT  0x04u  /* bit 2: USB3 direct mode (no USB4 tunnel) */
#define MODE_FLAG_FULL_COMMIT  0x40u  /* bit 6: full mode-entry commit (power event + USB int mask clear) */
#define MODE_FLAG_VDM_ACK      0x80u  /* bit 7: VDM-ACK policy enabled */
#define USB4_MODE_FLAGS        (MODE_FLAG_ROUTE_MASK | MODE_FLAG_USB3_DIRECT | MODE_FLAG_VDM_ACK)
#define USB4_MODE_USB3_DIRECT  MODE_FLAG_USB3_DIRECT
#define IS_USB4()              (u4_cfg.mode_flag & (MODE_FLAG_ROUTE_MASK | MODE_FLAG_VDM_ACK))

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

typedef struct {
    uint8_t lane_cap[2];
} u4_phy_runtime_t;

typedef struct {
    uint8_t conn_consequence_done;
    u4_fsm_state_t state;
    uint8_t transport_edge_toggle;
    uint8_t link_edge_toggle;
    uint8_t active_plane_port;
    uint8_t active_port_rr;
    uint8_t route_enable_latch;
    uint8_t routerop_push_token;  /* e461 */
    uint8_t routerop_resp_armed;  /* cdf5 */
    uint8_t lane_bonded_flag;
    uint8_t lane_width_latch0;
    uint8_t lane_width_latch1;
    uint8_t connect_descriptor;
    uint8_t tx_command_desc;
    uint8_t sb_desc_resp_len;  /* af38 */
    connrt_substate_t conn_routing_substate;
    uint8_t phy_gate_a;
    uint8_t phy_gate_b;
    uint8_t connect_present;
    uint8_t route_up_trigger;
    uint8_t walk_oneshot_flag;
    uint8_t lane_width_cnt_hi;
    uint8_t lane_width_cnt_lo;
    uint8_t walk_throttle_snap_hi;
    uint8_t walk_throttle_snap_lo;
    uint8_t lane_train_trigger;
    uint8_t route_query_response;
    uint8_t coldboot_seed_gate;
} u4_sb_fsm_t;

typedef struct {
    uint8_t contract_state;  /* negotiated power: 0 none, 1 <5V, 2 5V/1.5A, 3 5V/3A, 4 other */
    uint8_t connect_route_latch;
    uint8_t enter_usb_accepted;
    uint8_t role_state;
    /* PD policy substate: 1 idle/init, 2 Source_Cap rx (staging Request),
     * 3 Request sent (await Accept), 4 Accepted (await PS_RDY),
     * 0x0D Enter_USB pending, 0x0E Data_Reset pending (5/6 transient Source_Cap). */
    uint8_t msg_substate;
    uint8_t usb3_fallback_flag;
    uint8_t rx_slot_idx;
    uint8_t rx_num_data_obj;
    uint8_t tx_msgid_counter;
    uint8_t tx_msg_len;
    uint8_t bist_mode;
    uint8_t sop_field;
    uint8_t eudo_mode_confirm;  /* 07cc */
    uint8_t confirm_input_cd;
    uint8_t confirm_input_ce;
    uint8_t confirm_input_cf;
    uint8_t rx_slot_mask;
    uint8_t decoded_voltage_hi;
    uint8_t decoded_voltage_lo;
    uint8_t softreset_pending;
    uint8_t enter_usb_reinit_gate;  /* e8 */
    uint8_t connect_reinit_gate;  /* eb */
    uint8_t connect_oneshot_suppress;
    uint8_t cm_dispatch_sel;
} u4_pd_policy_t;

typedef struct {
    uint8_t hdr0;
    uint8_t hdr1;
    uint8_t hdr2;
    uint8_t hdr3;
} u4_routerop_header_t;

typedef struct {
    uint8_t dp_alt_mode;
    uint8_t cap20g_gate0;
    uint8_t cap20g_gate1;
    uint8_t sb_desc_profile;
    uint8_t mode_flag;
    uint8_t route_mode;
    uint8_t lane_gate_sel;
    uint8_t tunnel_cfg_hi;
    uint8_t tunnel_cfg_lo;
    uint8_t tunnel_cfg_mode;
    uint8_t tunnel_credits;
    uint8_t product_pid_lo;
    uint8_t product_pid_hi;
    uint8_t lb_width_rate_code;
    uint8_t lb_gen_index;
    uint8_t lane_mode_arg;
    uint8_t routerop_desc0;
    uint8_t routerop_desc1;
    uint8_t routerop_desc2;
    uint8_t routerop_desc3;
    uint8_t msg_type;
} u4_cfg_shadow_t;

typedef struct {
    uint8_t p12_desc_b0;  /* 3c */
    uint8_t p12_desc_b1;  /* 3d */
    uint8_t p12_desc_b2;  /* 3e */
    uint8_t p12_desc_b3;  /* 3f */
    uint8_t p12_desc_a0;  /* 3c */
    uint8_t p12_desc_a1;  /* 3d */
    uint8_t data_lo;
    uint8_t data_hi;
    uint8_t cc_subdemux_src;
    uint8_t link_reinit_gate;
    uint8_t reinit_pending;
} u4_p12_temp_t;

typedef struct {
    uint8_t pcie_ctrl_shadow;  /* b402 */
    uint8_t pd_seen;
    uint8_t sb_asserted;
    uint8_t tunnel_up_done;  /* e52d */
} u4_boot_scratch_t;

#define U4_XDATA_BYTES(sym) ((volatile __xdata uint8_t *)&(sym))
#define U4_ROUTEROP_MBOX_CLEAR_LEN 0x10u

/* Fixed XDATA map shared with stock-style PD, sideband, and router-op flows. */

/* Sideband descriptor tables and host work buffers. */
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
/* u4_host_desc[] — the host's in-band connect descriptor. */
#define HD_TYPE    0x0   /* descriptor type; 0x0C = route descriptor */
#define HD_STATUS  0x1   /* host connect status */
#define HD_WIDTH   0x3   /* host-advertised lane-width mask */
volatile __xdata __at(0x0800) uint8_t u4_work_buf[0x64];
/* u4_work_buf[] — negotiated lane state. */
#define WB_LANE_EN   0x19  /* negotiated lane-enable mask */
#define WB_LANE_CAP  0x1A  /* device lane-capability mask (from the DROM seed) */
volatile __xdata __at(0x099C) uint8_t sb_routerop_body[0x40];
volatile __xdata __at(0x0B26) uint8_t lb_cl_status[0x2];
volatile __xdata __at(0x0B28) uint8_t lb_eq_status[0x2];
volatile __xdata __at(0x0B2A) uint8_t lb_loop2_scratch[0x2];
volatile __xdata __at(0x0B2C) uint8_t lb_cl0_width[0x2];

/* Compiler-placed runtime state. */
volatile __xdata u4_phy_runtime_t u4_phy;
volatile __xdata u4_sb_fsm_t u4_sb;
volatile __xdata u4_pd_policy_t u4_pd;
volatile __xdata __at(0x0A9E) u4_routerop_header_t u4_rop_hdr;
volatile __xdata u4_cfg_shadow_t u4_cfg;
volatile __xdata u4_p12_temp_t u4_p12;
volatile __xdata u4_boot_scratch_t u4_boot;

/* Lane-bond descriptor-engine scratch. */
volatile __xdata __at(0x09DD) uint8_t u4lb_lane_active_flags;
volatile __xdata __at(0x0B34) uint8_t u4lb_desc0_lanemask;
volatile __xdata __at(0x0B35) uint8_t u4lb_desc1;
volatile __xdata __at(0x0B36) uint8_t u4lb_desc2;
volatile __xdata __at(0x0B37) uint8_t u4lb_desc3;
volatile __xdata __at(0x0B38) uint8_t u4lb_desc_set_mode;
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

#endif
