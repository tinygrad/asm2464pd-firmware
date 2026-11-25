#ifndef USB4_H
#define USB4_H

#include "types.h"
#include "registers.h"
#include "usb4_tables.h"

// USB4 device mode after an accepted Enter_USB. "bank 1 xxxx" = XDATA with DPX = 1.

#define USB4_DETACH_MS     25
#define USB4_SPIN          0xFFFF

/*=== State ===*/

// timeouts in a row per wait kind; usb4_to_x: PLL lock
static __xdata uint8_t usb4_to_eng, usb4_to_sb, usb4_to_timer, usb4_to_x;
// usb4_partner_link: partner register 12 byte 1
static __xdata uint8_t usb4_partner_tries, usb4_partner_link, usb4_resyncs, usb4_pcie_seen;
// RX slot ping-pong, next lane-status slot, a lane-status report opened the lane-start gate,
// the host has written the router
static __xdata uint8_t usb4_sb_slot, usb4_slot_i, usb4_slot_gate, usb4_host_owns;
// negotiated link: Gen 3 (else Gen 2), its option bit
static __xdata uint8_t usb4_gen3, usb4_link_opt;

static __xdata uint16_t usb4_ms;

static __xdata uint8_t usb4_saved[sizeof(usb4_saved_regs) / sizeof(usb4_saved_regs[0])];

static __xdata uint8_t usb4_sb_regs[92];
#define USB4_PARTNER_IDLE  0
#define USB4_PARTNER_SENT  1  // register 12 read sent
#define USB4_PARTNER_REPLY 2
static __xdata uint8_t usb4_partner;
static __xdata uint16_t usb4_partner_ms;
static __xdata uint8_t usb4_host_r12;

#define USB4_PHASE_IDLE      0
#define USB4_PHASE_NEGOTIATE 3
#define USB4_PHASE_LANES     4  // start the lanes
#define USB4_PHASE_TRAIN     5
#define USB4_PHASE_TRIGGERED 6
// lane and TxFFE handshake states, one per lane
#define USB4_ST_WAIT     0x01  // with a read state: the partner read is in flight
#define LANE_ST_IDLE     0x00  // finished or lane unused
#define LANE_ST_INIT     0x10
#define LANE_ST_READY    0x20  // read until the partner is ready
#define LANE_ST_PRESETS  0x40
#define LANE_ST_TRAIN    0x50
#define LANE_ST_ECHO     0x60  // read until the partner echoes our preset
#define LANE_ST_ARMED    0x80
#define LANE_ST_TRAINED  0x90
#define LANE_ST_DONE     0xA0  // read until the partner reports the preset done
#define TX_ST_IDLE       0x00
#define TX_ST_INIT       0x10
#define TX_ST_POLL       0x20  // read until the partner requests a preset
#define TX_ST_HOLD       0x50  // read until the partner releases the request
static __xdata uint8_t usb4_phase, usb4_lm_seen, usb4_inflight, usb4_p13_valid;
static __xdata uint8_t usb4_lane_st[2], usb4_p13[4];
static __xdata uint16_t usb4_con_ms, usb4_lane_ms[4];
static __xdata uint8_t usb4_tx_st[2];
// TxFFE preset handshake per lane: retries, preset index, 100 ms window armed / start
static __xdata uint8_t usb4_lane_ctr[2], usb4_lane_idx[2], usb4_lane_armed[2];
static __xdata uint16_t usb4_lane_t50[2];
#define USB4_NO_LANE 0xFF
static __xdata uint16_t usb4_rx_pay;
static __xdata uint8_t usb4_rx_hdr, usb4_rx_avail;

// usb4_ms advances by the armed interval when timer 1 fires; re-arming early drops the elapsed
// part, so deadlines can only come late.
#define USB4_HEARTBEAT_MS 100
static __xdata uint16_t usb4_armed;
static __xdata uint8_t usb4_active;
static __xdata uint32_t usb4_uid_lo;

static __xdata uint8_t usb4_rr_wait;  // ms until the Router Ready write
static __xdata uint16_t usb4_lost;    // ms both lanes have been out of CL0 after PCIe Enable

// Host control packets are held in bank 1 0200-020F until firmware releases them
static __xdata uint8_t usb4_ctl_pdf, usb4_ctl_hdr[8], usb4_ctl_remap;
static __xdata uint16_t usb4_ctl_off;

static __xdata uint8_t usb4_note_v[4];

static void usb4_pcie_lane_reset(void);
static void usb4_sb_kick(void);
static void usb4_sb_rx(void);
static void usb4_link_reset(void);
static void usb4_reset(void);

/*=== Primitives ===*/

static uint8_t bank1_rd(uint16_t a) {
  uint8_t v;
  DPX = 1;
  v = XDATA_REG8V(a);
  DPX = 0;
  return v;
}

static void bank1_mod(uint16_t a, uint8_t clr, uint8_t set) {
  DPX = 1;
  XDATA_REG8V(a) = (XDATA_REG8V(a) & ~clr) | set;
  DPX = 0;
}

static void usb4_timer_arm(uint16_t ms) {
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  usb4_armed = ms;
  if (!ms) {
    REG_TIMER1_DIV |= TIMER_DIV_IRQ_MASK;
    return;
  }
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xE8) | TIMER0_MODE_HALF_MS;
  REG_TIMER1_THRESHOLD_HI = (2 * ms) >> 8;
  REG_TIMER1_THRESHOLD_LO = 2 * ms;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
}

// XDATA and timer 1 survive the CPU reset: called at boot and before the reset
static void usb4_irq_off(void) {
  usb4_timer_arm(0);
  usb4_active = 0;
}

#define USB4_TMR_P1 0x01  // timer 0 prescale
#define USB4_TMR_P2 0x02
static void usb4_timer_once(uint8_t mode, uint16_t count) {
  uint16_t n;
  REG_TIMER0_CSR = TIMER_CSR_CLEAR;
  REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | mode;
  REG_TIMER0_THRESHOLD_HI = count >> 8;
  REG_TIMER0_THRESHOLD_LO = count & 0xFF;
  REG_TIMER0_CSR = TIMER_CSR_ENABLE;
  for (n = 0; !(REG_TIMER0_CSR & TIMER_CSR_EXPIRED) && n < USB4_SPIN; n++);
  if (n == USB4_SPIN) usb4_to_timer++;
  else usb4_to_timer = 0;
  REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
}

static void usb4_delay_ms(uint8_t ms) {
  while (ms--) usb4_timer_once(TIMER0_MODE_HALF_MS, 2);
}

static void usb4_cce_restart(void) {
  REG_TIMER_CCE0_CSR = TIMER_CSR_CLEAR;
  REG_TIMER_CCE0_CSR = TIMER_CSR_EXPIRED;
  rmw(REG_ADDR(REG_TIMER_CCE0_DIV), 0xFC, 0x04);
  REG_TIMER_CCE0_THRESHOLD_HI = 0xFF;
  REG_TIMER_CCE0_THRESHOLD_LO = 0xFF;
  REG_TIMER_CCE0_CSR = TIMER_CSR_ENABLE;
}

static void usb4_req_timer(void) {
  REG_XFER2_DMA_STATUS = TIMER_CSR_CLEAR;
  REG_XFER2_DMA_STATUS = TIMER_CSR_EXPIRED;
  REG_XFER2_DMA_STATUS = TIMER_CSR_ENABLE;
}

/*=== Register sequences ===*/

static void usb4_run_ops(__code const usb4_op_t *t, uint16_t n) {
  for (; n; n--, t++) {
    switch (t->kind) {
    case USB4_OP_W0: XDATA_REG8V(t->addr) = t->set; break;
    case USB4_OP_M0: rmw(t->addr, ~t->clr, t->set); break;
    case USB4_OP_W1: bank1_write(t->addr, t->set); break;
    case USB4_OP_M1: bank1_mod(t->addr, t->clr, t->set); break;
    }
  }
}

static void usb4_lane_wr(__code const usb4_lane_op_t *o, uint8_t lane) __reentrant {
  uint16_t a = LPHY0_BASE + o->off + (lane ? LPHY_STRIDE : 0);
  if (o->clr == 0xFF) XDATA_REG8V(a) = o->set;
  else rmw(a, ~o->clr, o->set);
}

// n rows for lane 0, then the same n rows for lane 1
static void usb4_lane_block(__code const usb4_lane_op_t *t, uint8_t n) __reentrant {
  uint8_t l, i;
  for (l = 0; l < 2; l++)
    for (i = 0; i < n; i++) usb4_lane_wr(&t[i], l);
}

#define USB4_ROUTER      0x80  // B1_ENG_TARGET
#define USB4_ADAPTER(n)  (0x40 | (n))

// mask 0xFF: leave the byte enables as they are. Returns 0 on a timeout.
static uint8_t usb4_cfg_go(uint8_t t, uint8_t idx, uint8_t mask, uint8_t ctl) {
  uint16_t w;
  uint8_t ok;
  if (mask != 0xFF) bank1_mod(B1_ENG_BE, 0x0F, mask);
  bank1_write(B1_ENG_TARGET, t);
  bank1_write(B1_ENG_INDEX, idx);
  bank1_mod(B1_ENG_CTL, ENG_CTL_WRITE | ENG_CTL_INDEX_HI, ctl);
  bank1_write(B1_ENG_GO, ENG_GO);
  for (w = 0; (bank1_rd(B1_ENG_GO) & ENG_GO) && w < USB4_SPIN; w++);
  ok = w != USB4_SPIN;
  if (ok) usb4_to_eng = 0;
  else usb4_to_eng++;
  bank1_mod(B1_ENG_TARGET, 0x3F, 0x00);
  for (w = 0; w < 4; w++) bank1_write(B1_ENG_WDATA + w, 0x00);
  return ok;
}

static void usb4_cfg_wr(uint8_t t, uint8_t idx, uint8_t mask, uint8_t ctl, uint32_t v) __reentrant {
  bank1_write(B1_ENG_WDATA, v);
  bank1_write(B1_ENG_WDATA + 1, v >> 8);
  bank1_write(B1_ENG_WDATA + 2, v >> 16);
  bank1_write(B1_ENG_WDATA + 3, v >> 24);
  usb4_cfg_go(t, idx, mask, ctl | ENG_CTL_WRITE);
}

// the read and the write may use different byte enables
static void usb4_cfg_rmw(uint8_t t, uint8_t idx, uint8_t rmask, uint8_t wmask, uint8_t ctl,
                         uint32_t clr, uint32_t set) __reentrant {
  uint8_t i;
  if (!usb4_cfg_go(t, idx, rmask, ctl)) return;
  for (i = 0; i < 4; i++, clr >>= 8, set >>= 8)
    bank1_write(B1_ENG_WDATA + i, (bank1_rd(B1_ENG_RDATA + i) & ~(uint8_t)clr) | (uint8_t)set);
  usb4_cfg_go(t, idx, wmask, ctl | ENG_CTL_WRITE);
}

// dwords 0x00-0xFF only
static uint8_t usb4_cfg_read(uint8_t target, uint8_t index) {
  uint8_t i;
  if (!usb4_cfg_go(target, index, 0xFF, 0x00)) return 0;
  for (i = 0; i < 4; i++) usb4_note_v[i] = bank1_rd(B1_ENG_RDATA + i);
  return 1;
}

// CS_26 byte 3: router operation status, OV clear
static void usb4_router_op_done(uint8_t status) {
  usb4_cfg_wr(USB4_ROUTER, 0x1A, 0x8, 0x00, (uint32_t)status << 24);
}

// Adapter 1 dword D2 bits 11/12 follow the link type, as stock writes them.
static void usb4_seq_neg(void) {
  usb4_cfg_rmw(USB4_ADAPTER(1), 0xD2, 0xFF, 0x2, 0x00, 0x00000800,
               usb4_gen3 || !usb4_link_opt ? 0x00000000 : 0x00000800);
  usb4_cfg_rmw(USB4_ADAPTER(1), 0xD2, 0x2, 0x2, 0x00, 0x00001000,
               usb4_gen3 && usb4_link_opt ? 0x00001000 : 0x00000000);
  bank1_mod(B1_LINK_GEN_CTRL, LINK_GEN_OPT, usb4_link_opt ? LINK_GEN_OPT : 0x00);
  bank1_mod(B1_LINK_GEN_CTRL, 0x00, LINK_GEN_COMMIT);
  bank1_write(B1_LANE0_TXFFE_HI, 0x00);
  bank1_write(B1_LANE0_TXFFE_LO, 0xA3);
  bank1_write(B1_LANE1_TXFFE_HI, 0x00);
  bank1_write(B1_LANE1_TXFFE_LO, 0xA3);
  bank1_write(0x2874, 0x00);
  bank1_write(0x2875, usb4_gen3 ? 0x1F : 0x0F);
  rmw(LPHY0_BASE + LPHY_CFG_14, 0xF2, 0x02);
  rmw(LPHY1_BASE + LPHY_CFG_14, 0xF2, 0x02);
  rmw(LPHY0_BASE + LPHY_CFG_16, 0xFB, 0x0B);
  rmw(LPHY1_BASE + LPHY_CFG_16, 0xFB, 0x0B);
  usb4_cfg_rmw(USB4_ADAPTER(1), 0xD2, 0x2, 0x2, 0x00, 0x00000000, 0x00000100);
  usb4_cfg_rmw(USB4_ROUTER, 0x8D, 0x2, 0x2, 0x01, 0x00000000, 0x00001000);
  usb4_cfg_rmw(USB4_ADAPTER(1), 0xD2, 0x2, 0x2, 0x00, 0x00000200, 0x00000000);
  usb4_cfg_rmw(USB4_ROUTER, 0x97, 0x2, 0x4, 0x01, 0x00020000, 0x00000000);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x04, 0xC, 0x00, 0x04200000);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x04, 0xC, 0x00, 0x04200000);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x80, 0xC, 0x00, usb4_gen3 ? 0x003C0000 : 0x00380000);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x80, 0xC, 0x00, usb4_gen3 ? 0x003C0000 : 0x00380000);
  bank1_mod(0x1267, 0x04, 0x0A);
  bank1_mod(B1_NOTE_MASK_LO, 0x00, 0x01);
}

static void usb4_seq_router(void) {
  bank1_mod(B1_EVT_MASK, 0x01, 0x00);
  bank1_mod(B1_NOTE_MASK_HI, 0x00, 0x10);
  bank1_mod(B1_NOTE_MASK_LO, 0x10, 0x00);
  bank1_write(B1_NOTE, NOTE_CS3);
  bank1_mod(B1_NOTE_MASK_LO, 0x20, 0x00);
  bank1_write(B1_NOTE, NOTE_CS5);
  bank1_write(0x1258, 0x01);
  bank1_mod(0x1802, 0x1E, 0x00);
  bank1_mod(0x1404, 0x00, 0x02);
  bank1_mod(0x1404, 0x00, 0x10);
  bank1_write(0x1511, 0x01);
  bank1_mod(0x1404, 0x02, 0x00);
  bank1_mod(0x1404, 0x10, 0x00);
  bank1_mod(0x1802, 0x00, 0x1E);
  bank1_mod(0x1403, 0x01, 0x00);
  bank1_mod(0x1403, 0x00, 0x01);
  bank1_write(B1_PCIE_EVT, PCIE_EVT_DISPATH);
  bank1_mod(B1_NOTE_MASK_LO, 0x01, 0x00);
  bank1_write(B1_CTL_REQ, CTL_REQ_HELD);
  bank1_mod(0x1290, 0x04, 0x00);
  bank1_write(B1_CTL_RPL, CTL_RPL_DONE);
  bank1_mod(0x1290, 0x20, 0x00);
  bank1_write(B1_CTL_RPL, 0x20);
  usb4_cfg_wr(USB4_ROUTER, 0x00, 0xF, 0x00, 0x2465174C);  // vendor/device
  usb4_cfg_wr(USB4_ROUTER, 0x07, 0xF, 0x00, usb4_uid_lo);  // UID low
  usb4_cfg_wr(USB4_ROUTER, 0x08, 0xF, 0x00, 0x10FA6970);  // UID high
  usb4_cfg_wr(USB4_ROUTER, 0x01, 0xE, 0x00, 0x00010100);  // CS_1 upper bytes
  usb4_cfg_wr(USB4_ROUTER, 0x01, 0x1, 0x00, 0x000000C0);  // CS_1 low byte
  usb4_cfg_wr(USB4_ROUTER, 0x20, 0x1, 0x00, 0x00000028);
  usb4_cfg_wr(USB4_ADAPTER(4), 0x02, 0x7, 0x00, 0x00000000);
  usb4_cfg_rmw(USB4_ROUTER, 0x20, 0x7, 0x8, 0x00, 0x00000000, 0x40000000);
  usb4_cfg_wr(USB4_ROUTER, 0x41, 0xF, 0x00, 0x00004001);
  usb4_cfg_wr(USB4_ROUTER, 0x44, 0xF, 0x00, 0x0003008A);
  usb4_cfg_wr(USB4_ROUTER, 0x4C, 0xF, 0x00, 0x00000300);
  usb4_cfg_wr(USB4_ROUTER, 0x97, 0x4, 0x01, 0x00040000);
  usb4_cfg_go(USB4_ROUTER, 0xAD, 0x4, 0x01);
}

static void usb4_seq_adapters(void) {
  bank1_write(0x120C, 0x01);
  bank1_mod(0x1208, 0x00, 0x10);
  bank1_write(0x120B, 0x01);
  bank1_mod(0x1208, 0x10, 0x00);
  bank1_mod(0x1206, 0x20, 0x00);
  bank1_write(B1_CS4_NOTE, CS4_NOTE_WRITTEN);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x04, 0xC, 0x00, 0x04200000);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x04, 0xC, 0x00, 0x04200000);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x80, 0xC, 0x00, 0x00380000);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x80, 0xC, 0x00, 0x00380000);
  usb4_cfg_wr(USB4_ADAPTER(3), 0x05, 0x2, 0x00, 0x00004000);
  usb4_cfg_wr(USB4_ADAPTER(4), 0x09, 0x8, 0x00, 0x00000400);
  usb4_cfg_wr(USB4_ADAPTER(4), 0x05, 0x2, 0x00, 0x00004000);
  usb4_cfg_wr(0x00, 0x00, 0x4, 0x00, 0x00040000);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x41, 0xF, 0x00, 0x0008CCCC);
  usb4_cfg_wr(USB4_ADAPTER(1), 0x42, 0xF, 0x00, 0x017B6666);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x41, 0xF, 0x00, 0x0008CCCC);
  usb4_cfg_wr(USB4_ADAPTER(2), 0x42, 0xF, 0x00, 0x017B6666);
  bank1_write(B1_NOTE, NOTE_CS26);
  bank1_write(B1_NOTE_HI, NOTE_HI_DW44);
  bank1_mod(B1_NOTE_MASK_LO, 0x08, 0x00);
  bank1_mod(B1_EVT_MASK, 0x01, 0x00);
  rmw(REG_ADDR(REG_INT_CTRL), 0xFF, 0x02);
  bank1_mod(B1_NOTE_MASK_HI, 0x40, 0x00);
  bank1_mod(0x127A, 0x00, 0x01);
  usb4_cfg_rmw(USB4_ROUTER, 0x06, 0xF, 0x5, 0x00, 0x00380001, 0x00000000);
}

static void usb4_seq_lanephy(void) {
  usb4_run_ops(usb4_tab_lane_prep, USB4_N(usb4_tab_lane_prep));
  rmw(REG_ADDR(REG_CPU_CLK_CFG), 0x9F, 0x80);
  rmw(REG_ADDR(REG_PHY_CTRL_C21F), 0xFF, 0x04);
  bank1_write(0x2849, 0xA0);
  usb4_lane_block(usb4_lane_phy_1, USB4_N(usb4_lane_phy_1));
  rmw(REG_ADDR(REG_PHY_CTRL_C21D), 0xBF, 0x80);
  usb4_lane_block(usb4_lane_phy_2, USB4_N(usb4_lane_phy_2));
}

static void usb4_pll_wait(void) {
  uint16_t i;
  for (i = 0; !((REG_LPHY(0, LPHY_STATUS) & LPHY_PLL_LOCKED) &&
              (REG_LPHY(1, LPHY_STATUS) & LPHY_PLL_LOCKED)) &&
              i < USB4_SPIN; i++);
  if (i == USB4_SPIN) usb4_to_x++;
  else usb4_to_x = 0;
}

static void usb4_lanecfg(uint16_t a) __reentrant {
  XDATA_REG8V(a) = (XDATA_REG8V(a) & 0x80) | 0x40 | ((XDATA_REG8V(a + 0x23) >> 3) & 0x07);
}

static void usb4_sb_ctl(uint8_t hdr) __reentrant {
  bank1_write(B1_SB_TX_HDR, hdr);
  bank1_mod(B1_SB_TX_FRAME_LEN, 0x7F, 0x03);
  bank1_mod(B1_SB_TX_CTRL, 0x00, SB_TX_HDR_ONLY);
  usb4_sb_kick();
}

static void usb4_rxpll_reset(void) {
  rmw(REG_ADDR(REG_CPU_CTRL_CC37), 0xFF, CPU_CTRL_CC37_RXPLL_MODE);
  REG_PHY_RXPLL_RESET = PHY_RXPLL_ASSERT;
  usb4_timer_once(USB4_TMR_P1, 0x14);
  REG_PHY_RXPLL_RESET = 0x00;
  usb4_timer_once(USB4_TMR_P2, 0x28);
  rmw(REG_ADDR(REG_CPU_CTRL_CC37), (uint8_t)~CPU_CTRL_CC37_RXPLL_MODE, 0x00);
}

static void usb4_pcie_start(void) {
  rmw(REG_ADDR(REG_PHY_TIMER_CTRL_E764), 0xFA, 0x0A);
  usb4_timer_once(USB4_TMR_P1, 0x07CF);
  rmw(REG_ADDR(REG_PHY_TIMER_CTRL_E764), 0xF0, 0x00);
  rmw(REG_ADDR(REG_PHY_CFG_C657), 0xFF, PHY_C657_SLOT);
  rmw(REG_ADDR(REG_HDDPC_CTRL), 0xFF, HDDPC_ENABLE);
}

static void usb4_ph4_start(void) {
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  usb4_timer_once(USB4_TMR_P2, 0x28);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), (uint8_t)~LINK_STATUS_E716_MASK, 0x00);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  rmw(REG_ADDR(REG_CPU_CTRL_CA81), 0xFE, 0x00);
  rmw(REG_ADDR(REG_CPU_MODE_NEXT), 0x7F, 0x60);
  rmw(REG_ADDR(REG_CPU_MODE_NEXT), 0x1F, 0x00);
  rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), (uint8_t)~PCIE_LANE_EN, 0x00);
  rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, 0x00);
  usb4_pcie_lane_reset();
  rmw(REG_ADDR(REG_CPU_EXT_CTRL), (uint8_t)~TIMER_DIV_IRQ_MASK, 0x00);
  REG_CPU_EXT_STATUS = TIMER_CSR_CLEAR;
  REG_CPU_EXT_STATUS = TIMER_CSR_EXPIRED;
  rmw(REG_ADDR(REG_TUNNEL_CTRL_B403), 0xFE, 0x00);
  bank1_mod(B1_DSP_CFG + PCI_LNKCTL2, 0x0B, 0x04);
  if (usb4_gen3) rmw(REG_ADDR(REG_TUNNEL_LINK_STATUS), (uint8_t)~PCIE_LINK_STATE_MASK, 0x00);
  else rmw(REG_ADDR(REG_TUNNEL_LINK_STATUS), 0xFC, 0x0C);
  usb4_pcie_lane_reset();
  usb4_pcie_start();
  usb4_sb_ctl(0x82);
  usb4_sb_ctl(0xA2);
  rmw(REG_ADDR(REG_CPU_CTRL_CC37), 0xFF, CPU_CTRL_CC37_RXPLL_MODE);
  if (usb4_gen3) {
    bank1_mod(B1_LINK_GEN_CTRL, 0x10, 0x00);
    bank1_mod(B1_LINK_GEN_CTRL, 0x20, 0x00);
  }
  bank1_mod(B1_LINK_GEN_CTRL, 0x00, 0x10);
  bank1_mod(B1_LINK_GEN_CTRL, 0x00, 0x20);
  if (!usb4_gen3) {
    bank1_mod(B1_LINK_GEN_CTRL, 0x10, 0x00);
    bank1_mod(B1_LINK_GEN_CTRL, 0x20, 0x00);
  }
  usb4_timer_once(USB4_TMR_P2, 0xC8);
  rmw(LPHY0_BASE + LPHY_PLL_CTRL, 0x3F, 0x00);
  rmw(LPHY1_BASE + LPHY_PLL_CTRL, 0x3F, 0x00);
  REG_PHY_CTRL_C20F = PHY_LANE_PLL_RESTART;
  usb4_timer_once(USB4_TMR_P1, 0x14);
  REG_PHY_CTRL_C20F = 0x00;
  usb4_pll_wait();
  rmw(LPHY0_BASE + LPHY_PLL_CTRL, 0x3F, 0x00);
  usb4_lanecfg(LPHY0_BASE + LPHY_LANE_CFG);
  rmw(LPHY1_BASE + LPHY_PLL_CTRL, 0x3F, 0x00);
  usb4_lanecfg(LPHY1_BASE + LPHY_LANE_CFG);
  rmw(LPHY0_BASE + LPHY_PLL_CTRL, 0xBF, 0x80);
  rmw(LPHY1_BASE + LPHY_PLL_CTRL, 0xBF, 0x80);
  usb4_rxpll_reset();
}

static void usb4_bond_cfg(uint8_t bit1) {
  rmw(REG_ADDR(REG_TIMER_CCB0_DIV), 0xFD, 0x05);
  REG_TIMER_CCB0_THRESHOLD_HI = 0x00;
  REG_TIMER_CCB0_THRESHOLD_LO = 0xC8;
  bank1_write(0x134D, 0x04);
  bank1_write(0x1334, 0x02);
  bank1_write(0x1335, 0x02);
  bank1_mod(0x2805, 0x02, 0x05 | bit1);
  bank1_mod(0x2804, 0x00, 0x80);
  bank1_mod(0x1285, 0xC0, 0x30);
}

static void usb4_ph4_end(void) {
  rmw(REG_ADDR(REG_CPU_CTRL_CA60), 0xFF, 0x08);
  usb4_bond_cfg(0x00);
  bank1_write(B1_LANE0_TRIG, LANE_TRIG_OS1);
  bank1_mod(B1_LANE_OS1, 0x00, 0x01);
  bank1_write(B1_LANE1_TRIG, LANE_TRIG_OS1);
  bank1_mod(B1_LANE_OS1, 0x00, 0x02);
  usb4_cce_restart();
}

/*=== Hardware setup ===*/

static void usb4_lane_phy_reset(uint8_t v) {
  REG_PHY_RXPLL_RESET = 0x00;
  REG_PHY_CTRL_C20F = 0x00;
  REG_PHY_CTRL_C210 = 0x00;
  REG_PHY_CTRL_C211 = v;
  REG_PHY_CTRL_C212 = v;
  REG_PHY_CTRL_C214 = 0x00;
  REG_PHY_CTRL_C215 = 0x00;
  REG_PHY_CTRL_C216 = 0x00;
  REG_PHY_CTRL_C217 = v;
}

static void usb4_mux(void) {
  bank1_mod(0x281C, 0x01, 0x00);
  bank1_mod(0x0000, 0x00, 0x02);
  REG_PHY_LINK_CTRL = 0xCC;
  bank1_mod(0x1504, 0x0C, 0x00);
  bank1_mod(0x0000, 0x04, 0x00);
}

static void usb4_pcie_lane_reset(void) {
  REG_PCIE_TUNNEL_CTRL |= PCIE_TUNNEL_ENABLE;
  REG_PCIE_TUNNEL_CTRL |= 0x02;
  REG_PCIE_TUNNEL_CTRL &= ~PCIE_TUNNEL_ENABLE;
  REG_PCIE_TUNNEL_CTRL &= ~0x02;
  rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT1, 0x08);
  bank1_mod(0x7104, 0x00, 0x40);
  bank1_write(0x1808, 0x00);
  bank1_mod(0x1835, 0x00, 0x01);
  bank1_mod(0x7041, 0x00, 0x40);
  bank1_write(0x6043, 0x70);
  bank1_mod(0x6025, 0x00, 0x80);
  bank1_write(B1_USP_CFG + PCI_LNKCAP3, 0x01);
  bank1_mod(B1_USP_CFG + PCI_LNKCAP1, 0x04, 0x08);
  bank1_mod(0x5204, 0x03, 0x00);
  bank1_mod(B1_DSP_CFG + PCI_LNKCAP1, 0x04, 0x08);
}

static void usb4_pcie_reset(void) {
  uint8_t bit;
  REG_TUNNEL_LINK_STATE |= 0x01;
  bank1_mod(B1_EVT_MASK, 0x08, 0x00);
  for (bit = 0x10; bit >= 0x02; bit >>= 1) {
    bank1_mod(B1_PCIE_EVT_MASK, bit, 0x00);
    bank1_write(B1_PCIE_EVT, bit);
  }
  bank1_mod(B1_RTR_EVT_MASK, 0xA0, 0x00);
  bank1_write(B1_RTR_EVT, 0x20);
  REG_PCIE_LANE_CTRL_C659 &= ~PCIE_LANE_EN;
  REG_PCIE_CTRL_B402 &= ~PCIE_CTRL_B402_BIT0;
  usb4_pcie_lane_reset();
}

static void usb4_lanes_off(void) {
  bank1_mod(B1_LANE_MUX_CTL, 0xD1, 0x00);
}

static void usb4_lane_mux(uint8_t flip) {
  uint8_t on = flip ? 0x03 : 0x00;
  usb4_lanes_off();
  bank1_mod(B1_LANE_MUX_FLIP, 0x03, on);
  bank1_mod(B1_LANE_MUX_CFG, 0x63, on | 0x90);
  REG_LINK_MODE_CTRL &= ~LINK_MODE_CTRL_MASK;
  REG_INT_ENABLE |= INT_ENABLE_SYSTEM;
  bank1_mod(B1_SB_LINK_EVT_MASK, 0x00, SB_LINK_CONNECTED);
  bank1_write(B1_SB_LINK_EVT, SB_LINK_CONNECTED);
  bank1_write(B1_SB_LINK_EVT, SB_LINK_DISCONNECTED);
  REG_INT_CTRL |= 0x08;
}

static void usb4_sb_init(void) {
  uint8_t i;
  usb4_lane_phy_reset(0x00);
  usb4_lanes_off();
  bank1_write(B1_SB_LINK_EVT, SB_LINK_CONNECTED);
  bank1_write(B1_SB_LINK_EVT, SB_LINK_DISCONNECTED);
  bank1_write(B1_SB_EVT, 0x02);
  bank1_write(B1_LINK_EVT, LINK_EVT_BONDED);
  bank1_mod(B1_SB_LINK_EVT_MASK, SB_LINK_CONNECTED, SB_LINK_DISCONNECTED);
  bank1_mod(B1_SB_RX0_MASK, 0x08, 0x00);
  bank1_mod(B1_SB_RX1_MASK, 0x08, 0x00);
  bank1_mod(B1_LSW_EVT_MASK, 0xF0, 0x00);
  bank1_mod(B1_SB_EVT_MASK, 0x02, 0x00);
  bank1_mod(B1_LINK_EVT_MASK, 0x81, 0x00);
  bank1_write(B1_SB_RX2_STS, 0x08);
  bank1_write(B1_SB_RX3_STS, 0x08);
  bank1_mod(B1_SB_RX2_MASK, 0x08, 0x00);
  bank1_mod(B1_SB_RX3_MASK, 0x08, 0x00);
  bank1_write(B1_LANE_EVT, LANE_EVT_L0_CL0);
  bank1_write(B1_LANE_EVT, LANE_EVT_L1_CL0);
  bank1_write(B1_LINK_EVT, LINK_EVT_L0_ABORT);
  bank1_write(B1_LINK_EVT, LINK_EVT_L1_ABORT);
  bank1_mod(B1_LANE_EVT_MASK, 0x03, 0x00);
  bank1_mod(B1_LINK_EVT_MASK, 0x24, 0x00);
  bank1_write(B1_LANE_EVT, LANE_EVT_L0_TRAIN);
  bank1_write(B1_LANE_EVT, LANE_EVT_L1_TRAIN);
  bank1_mod(B1_LANE_EVT_MASK, 0x30, 0x00);
  bank1_write(B1_SB_LINK_EVT, SB_LINK_TX_DONE);
  bank1_write(B1_SB_RX0_STS, SB_RX_PRESENT);
  bank1_write(B1_SB_RX1_STS, 0x08);
  for (i = 0x01; i; i <<= 1) bank1_write(B1_LSW_EVT, i);
  bank1_write(B1_LINK_EVT, LINK_EVT_L0_BOND_FAIL);
  bank1_write(B1_LINK_EVT, LINK_EVT_L1_BOND_FAIL);
  REG_XFER2_DMA_STATUS = TIMER_CSR_CLEAR;
  REG_XFER2_DMA_STATUS = TIMER_CSR_EXPIRED;
  bank1_mod(B1_LINK_CTRL_28D4, LINK_28D4_UP, 0x00);
  bank1_mod(0x288F, 0x00, 0x10);
  bank1_mod(0x2801, 0x00, 0xC0);
  bank1_mod(0x28C4, 0x02, 0x00);
  bank1_mod(B1_SB_EVT_MASK, 0x14, 0x00);
  REG_PHY_CONFIG &= ~0x08;
  REG_LPHY(0, LPHY_CFG_44) |= 0x40;
  REG_LPHY(0, LPHY_CFG_5C) &= ~0x3F;
  REG_LPHY(1, LPHY_CFG_44) |= 0x40;
  REG_LPHY(1, LPHY_CFG_5C) &= ~0x3F;
  rmw(LPHY0_BASE + LPHY_LANE_CTRL, 0x80, 0x1C);
  REG_LPHY(0, LPHY_TX_CTRL) &= ~LPHY_TX_PRESET15;
  rmw(LPHY1_BASE + LPHY_LANE_CTRL, 0x80, 0x1C);
  REG_LPHY(1, LPHY_TX_CTRL) &= ~LPHY_TX_PRESET15;
  REG_PHY_CTRL_C21C |= 0x40;
  REG_PHY_LINK_CTRL_C208 &= ~0x40;
  REG_LPHY(0, LPHY_LANE_CTRL) &= ~0x80;
  REG_LPHY(1, LPHY_LANE_CTRL) &= ~0x80;
  bank1_mod(0x281D, 0x02, 0x00);
  bank1_write(0x28BA, 0x3F);
  bank1_write(0x28BD, 0x3F);
  usb4_slot_i = (bank1_rd(B1_LSW_START) & 0x06) >> 1;
}

static void usb4_router_init(void) {
  uint8_t i;
  usb4_seq_router();
  bank1_mod(B1_ENG_SAVE_TGT, 0x3F, bank1_rd(B1_ENG_TARGET) & 0x3F);
  bank1_mod(B1_ENG_SAVE_TGT, 0xC0, bank1_rd(B1_ENG_TARGET) & 0xC0);
  bank1_write(B1_ENG_SAVE_IDX, bank1_rd(B1_ENG_INDEX));
  bank1_mod(B1_ENG_SAVE_HI, 0x1F, bank1_rd(B1_ENG_CTL) & ENG_CTL_INDEX_HI);
  for (i = 0; i < 4; i++) bank1_write(B1_ENG_WDATA + i, bank1_rd(B1_ENG_RDATA + i));
  usb4_seq_adapters();
}

static void usb4_ss_off(void) {
  rmw(REG_ADDR(REG_POWER_STATUS), (uint8_t)~POWER_STATUS_USB_PATH, 0x00);
  rmw(REG_ADDR(REG_USB_PHY_CTRL_91C0), (uint8_t)~USB_PHY_91C0_LINK_UP, 0x00);
  rmw(REG_ADDR(REG_CPU_MODE), (uint8_t)~CPU_MODE_USB3, 0x00);
  REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | LINK_WIDTH_LANES_MASK;
  rmw(REG_ADDR(REG_TIMER_CTRL_CC3B), 0xFF, 0x02);
}

static void usb4_switch_id(void) {
  uint8_t i;
  for (i = 0; i < 12; i++) {
    if (i == 4) continue;
    XDATA_REG8V(REG_ADDR(REG_TUNNEL_CFG_A_LO) + i) = usb4_sw_id[i];
    XDATA_REG8V(REG_ADDR(REG_TUNNEL_DATA_LO) + i) = usb4_sw_id[i];
  }
}

/*=== Sideband ===*/

// USB4 port sideband registers, as in Linux drivers/thunderbolt/sb_regs.h
#define SB_REG_OPCODE    0x08
#define SB_REG_METADATA  0x09
#define SB_REG_DATA      0x12
#define SB_WRITE         0x80
#define SB_PRESENT       0x18  // SB_RX_PRESENT | SB_RX_ARMED
#define SB_RX_NO_DATA    5  // RX length of a transaction without data bytes

static void usb4_sb_regs_init(void) {
  uint8_t i;
  for (i = 0; i < sizeof(usb4_sb_regs); i++) usb4_sb_regs[i] = 0;
  usb4_sb_regs[0] = 0x4C;   // vendor 0x174C (ASMedia, matches the router config space)
  usb4_sb_regs[1] = 0x17;
  usb4_sb_regs[4] = 0x61;   // product 0x2461
  usb4_sb_regs[5] = 0x24;
  usb4_sb_regs[18] = 0xF3;
  usb4_sb_regs[19] = 0x03;
  usb4_sb_slot = 0;
  usb4_partner = USB4_PARTNER_IDLE;
  usb4_phase = usb4_lm_seen = usb4_p13_valid = usb4_slot_gate = usb4_host_owns = 0;
  usb4_lane_st[0] = usb4_lane_st[1] = LANE_ST_IDLE;
  usb4_tx_st[0] = usb4_tx_st[1] = TX_ST_IDLE;
  usb4_lane_ctr[0] = usb4_lane_ctr[1] = usb4_lane_idx[0] = usb4_lane_idx[1] = 0;
  usb4_lane_armed[0] = usb4_lane_armed[1] = 0;
  usb4_inflight = USB4_NO_LANE;
}

static void usb4_sb_kick(void) {
  uint16_t n;
  bank1_mod(B1_LANE_MUX_CTL, 0x01, 0x00);
  bank1_mod(0x2804, 0x02, 0x00);
  bank1_write(B1_SB_TX_GO, 0x01);
  for (n = 0; !(bank1_rd(B1_SB_LINK_EVT) & SB_LINK_TX_DONE) && n < USB4_SPIN; n++);
  if (n == USB4_SPIN) usb4_to_sb++;
  else usb4_to_sb = 0;
  bank1_write(B1_SB_LINK_EVT, SB_LINK_TX_DONE);
  usb4_timer_once(USB4_TMR_P1, 0x0B);
  bank1_mod(B1_SB_TX_CTRL, SB_TX_HDR_ONLY, 0x00);
}

static void usb4_sb_send(uint8_t hdr, uint8_t len) {
  uint8_t i;
  bank1_mod(B1_SB_TX_FRAME_LEN, 0x7F, len + 8);
  bank1_write(B1_SB_TX_HDR, hdr);
  usb4_sb_kick();
  for (i = 0; i < len + 2; i++) bank1_write(B1_SB_TX_BUF + i, 0x00);
}

// length byte = generation << 4 | lanes
static void usb4_ph4_poll(void) {
  uint8_t n;
  for (n = 0; n < 2; n++) {
    bank1_write(B1_SB_TX_BUF, usb4_link_opt ? 0x0D : 0x09);
    bank1_write(B1_SB_TX_WLEN, usb4_gen3 ? 0x23 : 0x13);
    usb4_sb_send(0x61, 0);
    usb4_timer_once(USB4_TMR_P2, 0xC8);
  }
}

// payload byte 0 = register index, byte 1 = SB_WRITE | length
static void usb4_sb_request(void) {
  uint8_t idx, b1, req, i, n, off, len;
  idx = bank1_rd(usb4_rx_pay);
  b1 = bank1_rd(usb4_rx_pay + 1);
  req = b1 & 0x7F;
  len = 0;
  n = idx <= SB_REG_DATA ? sb_len[idx] : 0;
  off = sb_off[idx <= SB_REG_DATA ? idx : 0];
  if (idx == 0x0C) usb4_host_r12 = 1;
  bank1_write(B1_SB_TX_BUF, idx);
  bank1_write(B1_SB_TX_WLEN, (b1 & SB_WRITE) | n);
  if (!(b1 & SB_WRITE)) {
    if (n && !usb4_rx_avail) {
      len = req < n ? req : n;
      for (i = 0; i < len; i++) bank1_write(B1_SB_TX_DATA + i, usb4_sb_regs[off + i]);
    } else {
      bank1_mod(B1_SB_TX_WLEN, 0x7F, 0x00);
    }
  } else {
    len = 1;
    if (n && req == usb4_rx_avail && req <= n &&
        (idx == SB_REG_OPCODE || idx == SB_REG_METADATA || idx == SB_REG_DATA)) {
      for (i = 0; i < req; i++) usb4_sb_regs[off + i] = bank1_rd(usb4_rx_pay + 2 + i);
      if (idx == SB_REG_OPCODE &&
          (usb4_sb_regs[8] | usb4_sb_regs[9] | usb4_sb_regs[10] | usb4_sb_regs[11])) {
        usb4_sb_regs[8] = '!';  // "!CMD": operation not supported
        usb4_sb_regs[9] = 'C';
        usb4_sb_regs[10] = 'M';
        usb4_sb_regs[11] = 'D';
      }
      bank1_write(B1_SB_TX_DATA, 0x00);
    } else {
      bank1_mod(B1_SB_TX_WLEN, 0x7F, 0x00);
      bank1_write(B1_SB_TX_DATA, 0x01);
    }
  }
  usb4_sb_send(usb4_rx_hdr & 0xDE, len);
}

static void usb4_partner_read(void) {
  usb4_sb_rx();
  if (usb4_partner == USB4_PARTNER_REPLY) return;
  bank1_write(B1_SB_TX_BUF, 0x0C);
  bank1_write(B1_SB_TX_WLEN, 0x03);
  usb4_sb_send(0x05, 0);
  usb4_req_timer();
  usb4_partner = USB4_PARTNER_SENT;
  usb4_partner_ms = usb4_ms;
  usb4_partner_tries++;
}

static void usb4_partner_reply(void) {
  uint8_t i, idx;
  idx = bank1_rd(usb4_rx_pay);
  if (idx == 0x0C) {
    usb4_partner_link = bank1_rd(usb4_rx_pay + 3);
    if (usb4_partner == USB4_PARTNER_SENT) usb4_partner = USB4_PARTNER_REPLY;
  } else if (idx == 0x0D) {
    for (i = 0; i < 4; i++) {
      usb4_p13[i] = bank1_rd(usb4_rx_pay + 2 + i);
    }
    usb4_p13_valid = 1;
  }
}

static void usb4_link_mgmt(uint8_t hdr) {
  if (usb4_lm_seen) return;
  if ((hdr >> 1) & 0x0F) {
    bank1_mod(B1_LANE0_CTRL, 0x00, 0x08);
    bank1_mod(B1_LANE1_CTRL, 0x00, 0x08);
  }
  usb4_lm_seen = 1;
}

static void usb4_sb_dispatch(uint8_t hdr) {
  if ((hdr & 0xC0) == 0x40 || ((hdr & 0x04) && !(hdr & 0x10))) {
    if ((hdr & SB_HDR_LINK_MGMT) == SB_HDR_LINK_MGMT) {
      usb4_link_mgmt(hdr);
    } else if (!(hdr & SB_HDR_REQ)) {
      usb4_partner_reply();
    } else if (!(hdr & 0x40) || !((hdr >> 1) & 0x0F)) {
      usb4_sb_request();
    }
  }
}

static void usb4_sb_rx(void) {
  uint16_t st = usb4_sb_slot ? B1_SB_RX1_STS : B1_SB_RX0_STS;
  uint8_t hdr, len;
  if ((bank1_rd(st) & SB_PRESENT) != SB_PRESENT) return;
  usb4_rx_pay = usb4_sb_slot ? B1_SB_RX1_BUF : B1_SB_RX0_BUF;
  usb4_rx_hdr = hdr = bank1_rd(usb4_sb_slot ? B1_SB_RX1_HDR : B1_SB_RX0_HDR);
  len = bank1_rd(usb4_sb_slot ? B1_SB_RX1_LEN : B1_SB_RX0_LEN);
  if (len >= SB_RX_NO_DATA) {
    usb4_rx_avail = len - SB_RX_NO_DATA;
    usb4_sb_dispatch(hdr);
  }
  bank1_write(st, 0x10);
  bank1_write(st, 0x20);
  bank1_write(st, 0x40);
  bank1_write(st, 0x08);
  usb4_sb_slot ^= 1;
}

static void usb4_sb_connected(void) {
  if (bank1_rd(B1_LINK_RESYNC) & LINK_RESYNC_REQ) {
    bank1_mod(B1_LINK_RESYNC, LINK_RESYNC_REQ, 0x00);
    bank1_write(B1_LINK_CTRL_28D8, 0x02);
  }
  bank1_mod(0x2800, 0x00, 0xC0);
  bank1_mod(0x2804, 0x00, 0x01);
  bank1_mod(0x2801, 0x00, 0xC0);
  bank1_mod(B1_LANE_MUX_CTL, 0x01, 0x10);
  usb4_timer_once(USB4_TMR_P2, 0x15);
  bank1_mod(B1_LANE_MUX_CTL, 0x00, 0xC0);
  REG_CPU_CTRL_CA60 &= ~0x08;
  REG_CPU_CTRL_CA70 &= ~0x03;
  REG_SYS_CTRL_E780 &= ~0x06;
  REG_PHY_LINK_TRIGGER &= ~0x10;
  REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & ~0x20) | 0x50;
  bank1_mod(0x2805, 0x00, 0x80);
  usb4_cce_restart();
}

/*=== Link training ===*/

// register 12 byte 1: the link is what both sides set
#define USB4_LINK_LANE0    0x01
#define USB4_LINK_LANE1    0x02
#define USB4_LINK_BONDED   0x10
#define USB4_LINK_GEN3     0x20
#define USB4_LINK_G2_OPT   0x40
#define USB4_LINK_G3_OPT   0x80
#define USB4_LINK_X2       (USB4_LINK_LANE0 | USB4_LINK_LANE1 | USB4_LINK_BONDED)

// Only a bonded two-lane link: the lane and bonding sequences do not cover the other widths.
static void usb4_negotiate(uint8_t partner_link) {
  uint8_t l = partner_link & usb4_sb_regs[18];
  if ((l & USB4_LINK_X2) != USB4_LINK_X2) {
    usb4_phase = USB4_PHASE_IDLE;
    return;
  }
  usb4_gen3 = l & USB4_LINK_GEN3;
  usb4_link_opt = l & (usb4_gen3 ? USB4_LINK_G3_OPT : USB4_LINK_G2_OPT);
  usb4_seq_neg();
  usb4_sb_regs[17] = 0x03;  // both lanes usable
  usb4_phase = USB4_PHASE_LANES;
}

static uint8_t usb4_in(uint16_t v, uint16_t lo, uint16_t hi) {
  return v >= lo && v < hi;
}

// Rx CDR check against the limits of the generation; resets the Rx PLLs and
// retries while it fails
static void usb4_rx_adapt(void) {
  uint8_t i, a0, a1, amax;
  uint16_t b0, b1, blo, bhi;
  amax = 0x21;
  blo = 0x013E;
  bhi = 0x0143;
  if (!(bank1_rd(0x0000) & 0x02)) {
    if (!(REG_POWER_STATUS_92F8 & 0x0C)) {
      return;
    }
    amax = 0x29;
    blo = 0x013D;
    bhi = 0x0144;
  } else if ((REG_LPHY(0, usb4_gen3 ? LPHY_CFG_27 : LPHY_CFG_17) >> 5) & 0x01) {
    return;
  } else if (!usb4_gen3) {
    amax = 0x29;
    blo = 0x013D;
    bhi = 0x0144;
  }
  for (i = 0; i < 10; i++) {
    a0 = REG_LPHY(0, LPHY_RX_ADAPT_A) & 0x3F;
    b0 = (REG_LPHY(0, LPHY_RX_ADAPT_B_HI) << 8) | REG_LPHY(0, LPHY_RX_ADAPT_B_LO);
    a1 = REG_LPHY(1, LPHY_RX_ADAPT_A) & 0x3F;
    b1 = (REG_LPHY(1, LPHY_RX_ADAPT_B_HI) << 8) | REG_LPHY(1, LPHY_RX_ADAPT_B_LO);
    if ((REG_LPHY(0, LPHY_STATUS) & LPHY_RX_ADAPTED) &&
        (REG_LPHY(1, LPHY_STATUS) & LPHY_RX_ADAPTED) &&
        usb4_in(a0, 1, amax) && usb4_in(a1, 1, amax) &&
        usb4_in(b0, blo, bhi) && usb4_in(b1, blo, bhi)) {
      return;
    }
    usb4_rxpll_reset();
  }
}

static void usb4_lane_start(void) {
  usb4_ph4_start();
  usb4_rx_adapt();
  usb4_ph4_end();
  usb4_sb_regs[22] |= 0x80;
  usb4_sb_regs[23] |= 0x80;
  usb4_lane_st[0] = usb4_lane_st[1] = LANE_ST_INIT;
  usb4_tx_st[0] = usb4_tx_st[1] = TX_ST_INIT;
  usb4_lane_ctr[0] = usb4_lane_ctr[1] = usb4_lane_idx[0] = usb4_lane_idx[1] = 0;
  usb4_lane_armed[0] = usb4_lane_armed[1] = 0;
  usb4_inflight = USB4_NO_LANE;
  usb4_phase = USB4_PHASE_TRAIN;
}

static uint8_t usb4_lane_read(uint8_t l) {
  if (usb4_inflight != USB4_NO_LANE) return 0;
  usb4_p13_valid = 0;
  bank1_write(B1_SB_TX_BUF, 0x0D);
  bank1_write(B1_SB_TX_WLEN, 0x04);
  usb4_sb_send(0x41, 0);
  usb4_req_timer();
  usb4_inflight = l;
  usb4_lane_ms[l] = usb4_ms;
  return 1;
}

// Partner's register 13 byte for this lane: 0x21 first handshake (bit 7 ready), 0x61 preset echo
// (0xC0 | our preset nibble), 0xA1 preset done (bits 7:6 = 10).
static void usb4_lane_reply(uint8_t l) {
  uint8_t st = usb4_lane_st[l], v;
  if (usb4_inflight != l || !usb4_p13_valid) {
    if (usb4_ms - usb4_lane_ms[l] >= 100) {
      usb4_inflight = USB4_NO_LANE;
      usb4_lane_st[l] = st & ~USB4_ST_WAIT;
    }
    return;
  }
  usb4_inflight = USB4_NO_LANE;
  v = usb4_p13[2 + l];
  if (st == (LANE_ST_READY | USB4_ST_WAIT)) {
    if (!(v & 0x80)) {
      usb4_lane_st[l] = LANE_ST_READY;
      return;
    }
    bank1_write(B1_LANE_TRAIN_SEL, l + 1);
    usb4_lane_armed[l] = 0;
    usb4_sb_regs[20 + l] |= 0x20;
    usb4_lane_st[l] = LANE_ST_PRESETS;
  } else if (st == (LANE_ST_ECHO | USB4_ST_WAIT)) {
    if ((v & 0xC0) != 0xC0 || (v & 0x0F) != (usb4_sb_regs[20 + l] & 0x0F)) {
      usb4_lane_st[l] = LANE_ST_ECHO;
      usb4_timer_once(USB4_TMR_P2, 0x65);
      return;
    }
    bank1_write(B1_LANE_TRAIN_SEL, l + 1);
    usb4_cce_restart();
    usb4_lane_t50[l] = usb4_ms;
    usb4_lane_armed[l] = 1;
    usb4_lane_st[l] = LANE_ST_ARMED;
  } else {
    usb4_lane_st[l] = (v & 0xC0) == 0x80 ? LANE_ST_TRAIN : LANE_ST_DONE;
  }
  usb4_timer_once(USB4_TMR_P2, 0x65);
}

// one partner read in flight at a time
static void usb4_lane_step(uint8_t l) {
  switch (usb4_lane_st[l]) {
  case LANE_ST_INIT:
    usb4_sb_regs[20 + l] &= ~0xB0;
    usb4_lane_st[l] = LANE_ST_READY;
    break;
  case LANE_ST_READY:
  case LANE_ST_ECHO:
  case LANE_ST_DONE:
    if (usb4_lane_read(l)) usb4_lane_st[l] |= USB4_ST_WAIT;
    break;
  case LANE_ST_READY | USB4_ST_WAIT:
  case LANE_ST_ECHO | USB4_ST_WAIT:
  case LANE_ST_DONE | USB4_ST_WAIT:
    usb4_lane_reply(l);
    break;
  case LANE_ST_PRESETS:
    usb4_lane_ctr[l] = 0;
    usb4_lane_st[l] = LANE_ST_TRAIN;
    break;
  case LANE_ST_TRAIN:
    if ((bank1_rd(l ? B1_LANE1_STS : B1_LANE0_STS) & LANE_STS_TRAINED) && usb4_lane_ctr[l]) {
      bank1_write(l ? B1_LANE1_TRIG : B1_LANE0_TRIG, LANE_TRIG_START);
      usb4_sb_regs[20 + l] |= 0x50;
      usb4_lane_st[l] = LANE_ST_IDLE;
      // close both lanes, whichever lane triggered
      if (((REG_LPHY(0, LPHY_LANE_CTRL) & 0x01) || (REG_LPHY(1, LPHY_LANE_CTRL) & 0x01)) &&
          (usb4_sb_regs[17] & 0x03)) {
        usb4_sb_regs[20] = (usb4_sb_regs[20] & 0x7F) | 0x50;
        usb4_sb_regs[21] = (usb4_sb_regs[21] & 0x7F) | 0x50;
        usb4_lane_st[0] = usb4_lane_st[1] = LANE_ST_IDLE;
      }
    } else if (!usb4_lane_armed[l] || usb4_ms - usb4_lane_t50[l] >= 100) {
      if (usb4_lane_ctr[l] >= 15) {
        usb4_lane_st[l] = LANE_ST_IDLE;
        break;
      }
      usb4_lane_ctr[l]++;
      usb4_lane_idx[l] = (usb4_lane_idx[l] + 1) & 0x0F;
      usb4_sb_regs[20 + l] &= 0xF0;
      usb4_sb_regs[20 + l] |= usb4_presets[usb4_gen3 != 0][usb4_lane_idx[l]] | 0x80;
      usb4_lane_st[l] = LANE_ST_ECHO;
    }
    break;
  case LANE_ST_ARMED:
  case LANE_ST_TRAINED:
    if (bank1_rd(l ? B1_LANE1_STS : B1_LANE0_STS) & LANE_STS_TRAINED) {
      usb4_sb_regs[20 + l] = (usb4_sb_regs[20 + l] & 0x7F) | 0x50;
      usb4_lane_st[l] = usb4_lane_st[l] == LANE_ST_ARMED ? LANE_ST_TRAINED : LANE_ST_DONE;
    }
    break;
  }
}

static void usb4_tx_preset(uint8_t l, uint8_t idx) {
  uint16_t v;
  uint8_t b;
  b = usb4_ffe_b[idx];
  if (REG_CFG_C8FF == 4 && (idx == 4 || idx == 13)) b--;
  v = usb4_ffe_a[idx] | ((uint16_t)b << 5);
  bank1_write(B1_LANE0_TXFFE_HI + 2 * l, v >> 8);
  bank1_write(B1_LANE0_TXFFE_LO + 2 * l, v);
  rmw((l ^ (REG_PHY_VENDOR_CTRL_C6DB & PHY_PLUG_FLIP)) ? LPHY1_BASE + LPHY_TX_CTRL
                                              : LPHY0_BASE + LPHY_TX_CTRL,
      (uint8_t)~LPHY_TX_PRESET15, idx == 15 ? LPHY_TX_PRESET15 : 0x00);
}

// Partner's register 13 byte for this lane: bit 7 = it requests TxFFE preset (bits 3:0) for our
// transmitter, bit 4 = done. We answer in our byte 2 + lane: 0x40 | preset while bit 7 is held.
static void usb4_tx_reply(uint8_t l) {
  uint8_t st = usb4_tx_st[l], p;
  if (usb4_inflight != 2 + l || !usb4_p13_valid) {
    if (usb4_ms - usb4_lane_ms[2 + l] >= 100) {
      usb4_inflight = USB4_NO_LANE;
      usb4_tx_st[l] = st & ~USB4_ST_WAIT;
    }
    return;
  }
  usb4_inflight = USB4_NO_LANE;
  p = usb4_p13[l];
  if (st == (TX_ST_POLL | USB4_ST_WAIT)) {
    if (p & 0x10) {
      usb4_tx_st[l] = TX_ST_IDLE;
    } else if (!(p & 0x80)) {
      usb4_tx_st[l] = TX_ST_POLL;
    } else {
      usb4_sb_regs[22 + l] = (usb4_sb_regs[22 + l] & 0xF0) | (p & 0x0F) | 0x40;
      usb4_tx_preset(l, p & 0x0F);
      usb4_tx_st[l] = TX_ST_HOLD;
    }
  } else if (p & 0x80) {
    usb4_tx_st[l] = TX_ST_HOLD;
  } else {
    usb4_sb_regs[22 + l] &= ~0x40;
    usb4_tx_st[l] = TX_ST_POLL;
  }
  usb4_timer_once(USB4_TMR_P2, 0x65);
}

static void usb4_tx_step(uint8_t l) {
  switch (usb4_tx_st[l]) {
  case TX_ST_INIT:
    usb4_sb_regs[22 + l] = (usb4_sb_regs[22 + l] & ~0x40) | 0x80;
    usb4_tx_st[l] = TX_ST_POLL;
    break;
  case TX_ST_POLL:
  case TX_ST_HOLD:
    if (usb4_lane_read(2 + l)) usb4_tx_st[l] |= USB4_ST_WAIT;
    break;
  case TX_ST_POLL | USB4_ST_WAIT:
  case TX_ST_HOLD | USB4_ST_WAIT:
    usb4_tx_reply(l);
    break;
  }
}

static void usb4_link_tick(void) {
  uint8_t l;
  if ((usb4_phase == USB4_PHASE_NEGOTIATE && usb4_partner == USB4_PARTNER_IDLE &&
       (usb4_host_r12 || usb4_ms - usb4_con_ms >= 20)) ||
      (usb4_partner == USB4_PARTNER_SENT && usb4_ms - usb4_partner_ms >= 100 &&
       usb4_partner_tries < 10))
    usb4_partner_read();
  if (usb4_phase == USB4_PHASE_NEGOTIATE && usb4_partner == USB4_PARTNER_REPLY) {
    usb4_negotiate(usb4_partner_link);
  } else if (usb4_phase == USB4_PHASE_LANES) {
    usb4_ph4_poll();
    if ((usb4_lm_seen || usb4_slot_gate) && usb4_ms - usb4_con_ms >= 30) usb4_lane_start();
  } else if (usb4_phase == USB4_PHASE_TRAIN) {
    for (l = 0; l < 2; l++)
      if (usb4_sb_regs[17] & (1 << l)) usb4_lane_step(l);
    for (l = 0; l < 2; l++)
      if (usb4_sb_regs[17] & (1 << l)) usb4_tx_step(l);
    if (!usb4_lane_st[0] && !usb4_lane_st[1] && !usb4_tx_st[0] && !usb4_tx_st[1]) {
      usb4_phase = USB4_PHASE_TRIGGERED;
      usb4_resyncs = 0;
    }
  }
}

static void usb4_lane_slots(void) {
  uint8_t i = usb4_slot_i, w, c;
  if (!(bank1_rd(B1_LSW_EVT) & (0x10 << i))) return;
  w = bank1_rd(usb4_slot_word[i]);
  c = bank1_rd(usb4_slot_word[i] + 2);
  if ((w ^ c) == 0xFF) {
    if ((w & 0x0F) == 1 || (w & 0x0F) == 5) usb4_slot_gate = 1;
    // code 3: the partner gave up training this lane; code 0: lane lost
    if ((w & 0x0F) == 3) {
      if (usb4_host_owns) {
        bank1_write(B1_LANE0_TRIG, LANE_TRIG_HOLD);
        bank1_write(B1_LANE1_TRIG, LANE_TRIG_HOLD);
        bank1_mod(B1_LINK_RESYNC, 0x00, LINK_RESYNC_REQ);
        usb4_sb_ctl(0x83);
      } else {
        usb4_sb_ctl(0x83);
        usb4_link_reset();
      }
    } else if ((w & 0x0F) == 0) {
      if (usb4_phase > USB4_PHASE_NEGOTIATE && (w & 0x20)) {
        bank1_write(B1_LANE1_TRIG, LANE_TRIG_HOLD);
        usb4_sb_regs[17] &= ~0x02;
      } else if (usb4_phase > USB4_PHASE_NEGOTIATE) {
        usb4_link_reset();
      }
    }
  }
  bank1_write(B1_LSW_EVT, 1 << i);
  bank1_write(B1_LSW_EVT, 0x10 << i);
  usb4_slot_i = (usb4_slot_i + 1) & 0x03;
}

static void usb4_resync(void) {
  if (!(bank1_rd(B1_LINK_RESYNC) & LINK_RESYNC_REQ) || usb4_phase < USB4_PHASE_NEGOTIATE ||
      usb4_resyncs >= 20)
    return;
  usb4_resyncs++;
  bank1_mod(B1_LINK_RESYNC, LINK_RESYNC_REQ, 0x00);
  bank1_write(B1_LINK_CTRL_28D8, 0x02);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), (uint8_t)~LINK_STATUS_E716_MASK, 0x00);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  usb4_sb_regs[17] = 0;
  usb4_sb_regs[20] = usb4_sb_regs[21] = usb4_sb_regs[22] = usb4_sb_regs[23] = 0;
  usb4_host_owns = 0;
  usb4_phase = USB4_PHASE_NEGOTIATE;
  usb4_partner = USB4_PARTNER_IDLE;
  usb4_lane_st[0] = usb4_lane_st[1] = LANE_ST_IDLE;
  usb4_tx_st[0] = usb4_tx_st[1] = TX_ST_IDLE;
  usb4_inflight = USB4_NO_LANE;
  usb4_partner_tries = 0;
}

/*=== Router and PCIe tunnel ===*/

static uint8_t usb4_pcie_pe(void) {
  return usb4_cfg_read(USB4_ADAPTER(3), 0x09) && (usb4_note_v[3] & 0x80);
}

static void usb4_ctl_reply(void) {
  uint8_t i;
  bank1_write(B1_CTL_RPL, CTL_RPL_DONE);
  if (usb4_ctl_pdf == 2) {
    for (i = 0; i < 8; i++) bank1_write(B1_CTL_RPL_HDR + i, usb4_ctl_hdr[i]);
    bank1_mod(B1_CTL_RPL_HDR + 3, 0x00, 0x80);
  }
  if (usb4_ctl_remap) {
    bank1_mod(B1_CTL_RPL_OFF_HI, 0x1F, usb4_ctl_off >> 8);
    bank1_write(B1_CTL_RPL_OFF_LO, usb4_ctl_off);
  }
  bank1_write(B1_CTL_RPL_MODE, 0x00);
  bank1_write(B1_CTL_RPL_GO, 0x01);
  usb4_timer_once(USB4_TMR_P1, 0xC7);
  if (usb4_pcie_pe())
    REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | LINK_WIDTH_LANES_MASK;
}

static void usb4_ctl_request(void) {
  uint8_t pdf, space, i;
  uint16_t off;
  REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | 0x04;
  bank1_write(B1_CTL_REQ, CTL_REQ_HELD);
  if ((bank1_rd(B1_CTL_VALID) & CTL_PKT_VALID) && !(bank1_rd(B1_CTL_PKT_PDF) & 0x04)) {
    pdf = bank1_rd(B1_CTL_PKT_PDF) >> 4;
    off = bank1_rd(B1_CTL_PKT_OFF_LO) | ((uint16_t)(bank1_rd(B1_CTL_PKT_OFF_HI) & 0x1F) << 8);
    space = (bank1_rd(B1_CTL_PKT_SPACE) >> 1) & 0x03;
    usb4_ctl_pdf = pdf;
    usb4_ctl_off = off;
    usb4_ctl_remap = 0;
    for (i = 0; i < 8; i++) usb4_ctl_hdr[i] = bank1_rd(B1_CTL_PKT_HDR + i);
    if (space == 2) {
      if (off >= 0xC0 && off <= 0xD9) off -= 0xA0;
      else if (off >= 0x28 && off <= 0x51) off += 0x18;
      else off = 0xFFFF;
      if (off != 0xFFFF) {
        usb4_ctl_remap = 1;
        bank1_write(B1_CTL_PKT_OFF_HI, (bank1_rd(B1_CTL_PKT_OFF_HI) & 0xE0) | (off >> 8));
        bank1_write(B1_CTL_PKT_OFF_LO, off);
      }
    }
    bank1_write(B1_CTL_RPL_MODE, CTL_RPL_HW);
  }
  bank1_write(B1_CTL_RELEASE0, 0x02);
  bank1_write(B1_CTL_RELEASE1, 0x01);
}

static uint8_t usb4_note(uint16_t ack_reg, uint8_t ack, uint8_t dword) {
  bank1_write(ack_reg, ack);
  return usb4_cfg_read(USB4_ROUTER, dword);
}

static void usb4_rxpll(void) {
  rmw(REG_ADDR(REG_INT_C808), 0xFF, 0x02);
  REG_PHY_RXPLL_CFG_B = 0xFF;
  rmw(REG_ADDR(REG_PHY_RXPLL_CFG_A), 0xFF, 0x04);
  rmw(REG_ADDR(REG_PHY_RXPLL_CFG_B), 0xFB, 0x00);
  rmw(REG_ADDR(REG_PHY_RXPLL_CFG_A), 0xFF, 0x08);
  rmw(REG_ADDR(REG_PHY_RXPLL_CFG_B), 0xF7, 0x00);
  REG_PHY_RXPLL_TRIGGER = 0x04;
  REG_PHY_RXPLL_TRIGGER = 0x08;
}

static void usb4_enabled(void) {
  usb4_lane_phy_reset(0x26);
  usb4_rxpll();
  rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~(PCIE_CTRL_B402_BIT0 | PCIE_CTRL_B402_BIT1), 0x00);
  rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), 0xFF, PCIE_LANE_EN);
  usb4_rr_wait = 2;
}

// Router op 0x24: CS_25 bits 2-14 = dword address, 15-19 = dwords; data returned in CS_9 onwards
static void usb4_drom_read(void) {
  static __xdata uint16_t a;
  static __xdata uint8_t n, d, i;
  if (!usb4_cfg_read(USB4_ROUTER, 0x19)) return;
  a = (((uint16_t)usb4_note_v[1] & 0x7F) << 8 | usb4_note_v[0]) & 0x7FFC;
  n = (usb4_note_v[2] & 0x0F) << 1 | usb4_note_v[1] >> 7;
  if (n > 16) n = 16;
  for (d = 0; d < n; d++) {
    for (i = 0; i < 4; i++, a++)
      bank1_write(B1_ENG_WDATA + i, a < sizeof(usb4_drom) ? usb4_drom[a] : 0xFF);
    usb4_cfg_go(USB4_ROUTER, 0x09 + d, 0xF, ENG_CTL_WRITE);
  }
  usb4_router_op_done(0x00);
}

static void usb4_pcie_events(void) {
  uint8_t s, b;
  if (!(bank1_rd(B1_EVT) & EVT_PCIE)) return;
  s = bank1_rd(B1_PCIE_EVT);
  if (s & (PCIE_EVT_ENABLE | PCIE_EVT_DISPATH)) {
    bank1_write(B1_PCIE_EVT, s & (PCIE_EVT_ENABLE | PCIE_EVT_DISPATH));
    // DisPath with a tunnel up: reset the PCIe block so the next Enable starts like a first plug
    if ((s & PCIE_EVT_DISPATH) && (usb4_pcie_seen & PCIE_EVT_ENABLE)) {
      rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), (uint8_t)~PCIE_LANE_EN, 0x00);
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, 0x00);
      usb4_pcie_lane_reset();
    }
    usb4_pcie_seen |= s & (PCIE_EVT_ENABLE | PCIE_EVT_DISPATH);
  }
  if (s & PCIE_EVT_ENABLE) {
    // after a DisPath block reset (host re-authorize): restart PCIe training and slot power first
    if (!(REG_PCIE_LANE_CTRL_C659 & PCIE_LANE_EN)) {
      usb4_pcie_start();
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, 0x00);
      rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), 0xFF, PCIE_LANE_EN);
    }
    REG_PCIE_PERST_CTRL &= ~0x0F;
    usb4_rxpll();
  }
  if (s & PCIE_EVT_USP_RST_DEASSERT) {
    bank1_write(B1_PCIE_EVT, PCIE_EVT_USP_RST_DEASSERT);
    if (usb4_pcie_pe()) {
      b = REG_PCIE_CTRL_B402 & PCIE_CTRL_B402_BIT1;
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT1, 0x00);
      bank1_mod(0x7041, 0x40, 0x00);
      usb4_timer_once(USB4_TMR_P1, 0x63);
      bank1_mod(0x7104, 0x00, 0x40);
      if (!(REG_PCIE_LANE_CTRL_C659 & PCIE_LANE_EN)) {
        rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, 0x00);
        rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), 0xFF, PCIE_LANE_EN);
        rmw(REG_ADDR(REG_PCIE_CTRL_B402), 0xFF, PCIE_CTRL_B402_BIT0);
      }
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), 0xFF, b);
    }
  }
  if (s & PCIE_EVT_USP_RST_ASSERT) {
    bank1_write(B1_PCIE_EVT, PCIE_EVT_USP_RST_ASSERT);
    if (usb4_pcie_pe()) {
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT1, 0x00);
      bank1_mod(0x7041, 0x00, 0x40);
      rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), (uint8_t)~PCIE_LANE_EN, 0x00);
      usb4_timer_once(USB4_TMR_P1, 0x63);
      rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, PCIE_CTRL_B402_BIT1);
    }
  }
}

static void usb4_ctl_packet(void) {
  usb4_pcie_events();
  if (!(bank1_rd(B1_EVT) & EVT_CTL)) return;
  if (bank1_rd(B1_CTL_REQ) & CTL_REQ_HELD) usb4_ctl_request();
  if (bank1_rd(B1_CTL_RPL) & CTL_RPL_DONE) usb4_ctl_reply();
  if (bank1_rd(B1_NOTE) & NOTE_CS3) {
    if (usb4_note(B1_NOTE, NOTE_CS3, 0x03) && (usb4_note_v[3] & 0x80)) usb4_enabled();
  }
  if ((bank1_rd(B1_CS4_NOTE) & CS4_NOTE_WRITTEN) &&
      usb4_note(B1_CS4_NOTE, CS4_NOTE_WRITTEN, 0x04)) {
    bank1_write(B1_PLUG_DELAY_LO, usb4_note_v[0] << 1);
    bank1_write(B1_PLUG_DELAY_HI, usb4_note_v[0] >> 7);
  }
  if (bank1_rd(B1_NOTE) & NOTE_CS5) {
    usb4_note(B1_NOTE, NOTE_CS5, 0x05);
  }
  if ((bank1_rd(B1_NOTE) & NOTE_CS26) && usb4_note(B1_NOTE, NOTE_CS26, 0x1A)) {
    if (usb4_note_v[0] == 0x33 && !usb4_note_v[1] && (usb4_note_v[3] & 0x80) && usb4_gen3) {
      // buffer allocation: one entry, baMaxPCIE (index 4) = 64; Gen 2 links keep the host's
      // default, 64 there stalls PCIe memory traffic
      usb4_cfg_wr(USB4_ROUTER, 0x09, 0xF, 0x00, 0x00400004);
      usb4_cfg_wr(USB4_ROUTER, 0x19, 0xF, 0x00, 0x00000001);
      usb4_router_op_done(0x00);
    } else if (usb4_note_v[0] == 0x24 && !usb4_note_v[1] && (usb4_note_v[3] & 0x80)) {
      usb4_drom_read();
    } else if (usb4_note_v[3] & 0x80) {
      usb4_router_op_done(0x02);  // not supported
    }
  }
  if (bank1_rd(B1_NOTE_HI) & NOTE_HI_DW44) {
    usb4_note(B1_NOTE_HI, NOTE_HI_DW44, 0x44);
  }
}

/*=== Lifecycle ===*/

static void usb4_teardown_pcie(void) {
  rmw(REG_ADDR(REG_PHY_TIMER_CTRL_E764), 0xF4, 0x04);
  rmw(REG_ADDR(REG_HDDPC_CTRL), (uint8_t)~HDDPC_ENABLE, 0x00);
  usb4_timer_once(USB4_TMR_P2, 0x03E7);
  rmw(REG_ADDR(REG_PCIE_LANE_CTRL_C659), (uint8_t)~PCIE_LANE_EN, 0x00);
  rmw(REG_ADDR(REG_PCIE_CTRL_B402), (uint8_t)~PCIE_CTRL_B402_BIT0, 0x00);
  usb4_pcie_lane_reset();
  rmw(REG_ADDR(REG_CPU_EXT_CTRL), (uint8_t)~TIMER_DIV_IRQ_MASK, 0x00);
  REG_CPU_EXT_STATUS = TIMER_CSR_CLEAR;
  REG_CPU_EXT_STATUS = TIMER_CSR_EXPIRED;
  rmw(REG_ADDR(REG_PCIE_LINK_CTRL_B481), 0xFF, 0xFF);
  rmw(REG_ADDR(REG_PHY_CFG_C655), 0xFF, 0x01);
  rmw(REG_ADDR(REG_GPIO_CTRL(GPIO_LED_R)), 0xE0, 0x00);
  rmw(REG_ADDR(REG_PHY_CFG_C65A), 0xFF, 0x01);
}

static void usb4_teardown(void) {
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  rmw(REG_ADDR(REG_CPU_CTRL_CA81), 0xFE, 0x00);
  rmw(REG_ADDR(REG_CPU_MODE_NEXT), 0x7F, 0x60);
  if (usb4_pcie_seen & PCIE_EVT_ENABLE) usb4_teardown_pcie();
  usb4_lane_phy_reset(0x00);
  bank1_write(B1_LINK_CTRL_28D8, 0x02);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), (uint8_t)~LINK_STATUS_E716_MASK, 0x00);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  usb4_sb_init();
  usb4_router_init();
  rmw(REG_ADDR(REG_CPU_EXT_CTRL), (uint8_t)~TIMER_DIV_IRQ_MASK, 0x00);
  REG_CPU_EXT_STATUS = TIMER_CSR_CLEAR;
  REG_CPU_EXT_STATUS = TIMER_CSR_EXPIRED;
  rmw(REG_ADDR(REG_LINK_STATUS_E716), (uint8_t)~LINK_STATUS_E716_MASK, 0x00);
  rmw(REG_ADDR(REG_CPU_CTRL_CA81), 0xFF, 0x01);
  rmw(REG_ADDR(REG_CPU_MODE_NEXT), 0x9F, 0x80);
  usb4_timer_once(TIMER0_MODE_HALF_MS, 0x65);
}

// A normal connect passes through here once: the sideband disconnects and reconnects early.
static void usb4_link_reset(void) {
  usb4_sb_regs_init();
  usb4_rr_wait = 0;
  usb4_teardown();
  usb4_pcie_seen = usb4_lost = 0;
}

// Returns 1 when an event was acked and the INT1 source is now clear.
static uint8_t usb4_typec_event(uint8_t in_usb4) {
  if (!(REG_CPU_EXEC_STATUS_2 & CPU_EXEC_STATUS_2_INT)) return 0;
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_INT;
  if (!in_usb4 || (REG_PHY_EXT_B3 & PHY_EXT_LINK_READY)) {
    if (XDATA_REG8V(REG_ADDR(REG_CPU_EXEC_STATUS_2)) & CPU_EXEC_STATUS_2_INT) return 0;
    return 1;
  }
  usb4_reset();
  return 1;
}

// Router UID low dword with the SPI flash's 128-bit unique ID folded in; a flash without the
// command reads 0xFF and keeps the base value.
static void usb4_uid_init(void) {
  uint8_t i;
  usb4_uid_lo = 0x174C6819;
  flash_cmd(0x4B, 0, 0x07, 17);  // Read Unique ID: one dummy byte, then 16 bytes
  for (i = 0; i < 16; i++) ((__xdata uint8_t *)&usb4_uid_lo)[i & 3] ^= FLASH_BUF[1 + i];
}

static void usb4_begin(void) {
  uint8_t i, flip;
  pd_usb4_go = 0;
  IE &= ~IE_EX0;
  usb4_uid_init();
  for (i = 0; i < sizeof(usb4_saved); i++) usb4_saved[i] = XDATA_REG8V(usb4_saved_regs[i]);
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_INT;
  usb4_to_eng = usb4_to_sb = usb4_to_timer = usb4_to_x = 0;
  usb4_partner_tries = usb4_partner_link = usb4_resyncs = usb4_pcie_seen = 0;
  usb4_ctl_pdf = usb4_ctl_remap = usb4_rr_wait = usb4_lost = 0;
  flip = REG_PHY_VENDOR_CTRL_C6DB & PHY_PLUG_FLIP;
  // SuperSpeed shares the lane PHY; GPIO 0/8/14 go back to their default functions
  bank1_mod(B1_LANE_CL0_ACK, 0x03, 0x00);
  gpio_set(GPIO_LED_R, GPIO_INPUT);
  gpio_set(GPIO_LED_G, GPIO_BOOT_0B);
  gpio_set(GPIO_LED_B, GPIO_BOOT_0C);
  REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | LINK_WIDTH_LANES_MASK;
  REG_PHY_RXPLL_RESET = PHY_RXPLL_ASSERT;
  rmw(REG_ADDR(REG_LINK_STATUS_E716), (uint8_t)~LINK_STATUS_E716_MASK, 0x00);
  rmw(REG_ADDR(REG_LINK_STATUS_E716), 0xFF, LINK_STATUS_E716_MASK);
  usb4_timer_once(USB4_TMR_P2, 0x27);
  REG_PHY_RXPLL_RESET = 0x00;
  usb4_run_ops(usb4_tab_boot_pre, USB4_N(usb4_tab_boot_pre));
  usb4_run_ops(usb4_tab_boot_post, USB4_N(usb4_tab_boot_post));
  usb4_ss_off();
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_INT;
  usb4_seq_lanephy();
  usb4_sb_init();
  rmw(REG_ADDR(REG_INT_CTRL), 0xFF, 0x02);
  usb4_router_init();
  bank1_mod(B1_USB3_EVT_MASK, 0x01, 0x00);
  bank1_write(B1_USB3_EVT, 0x01);
  bank1_mod(B1_USB3_EVT_MASK, 0x02, 0x00);
  bank1_write(B1_USB3_EVT, 0x02);
  rmw(REG_ADDR(REG_GPIO_CTRL(22)), 0xE0, 0x00);
  rmw(REG_ADDR(REG_PHY_EXT_5C), 0xBF, 0x00);
  bank1_mod(0x121E, 0x00, 0x01);
  REG_CPU_CTRL_CA81 &= ~0x01;
  usb4_mux();
  usb4_pcie_reset();
  usb4_switch_id();
  usb4_lane_mux(flip);
  usb4_sb_regs_init();
  usb4_sb_init();
  usb4_ms = 0;
  usb4_timer_arm(1);
  usb4_active = 1;
  IE |= IE_EA | IE_EX1;
}

// Restarts into a normal boot, as after a power-on.
static void usb4_reset(void) {
  uint8_t i;
  usb4_lanes_off();
  bank1_write(B1_SB_LINK_EVT, SB_LINK_CONNECTED | SB_LINK_DISCONNECTED);
  bank1_write(B1_SB_EVT, 0xFF);
  bank1_write(B1_LINK_EVT, 0xFF);
  bank1_write(B1_LANE_EVT, 0xFF);
  bank1_write(B1_LSW_EVT, 0xFF);
  for (i = 0; i < sizeof(usb4_saved); i++) XDATA_REG8V(usb4_saved_regs[i]) = usb4_saved[i];
  usb4_lane_phy_reset(0x00);
  REG_HDDPC_CTRL &= ~HDDPC_ENABLE;
  REG_PHY_CFG_C657 &= ~PHY_C657_SLOT;
  REG_TIMER_CCE0_CSR = TIMER_CSR_CLEAR;
  REG_TIMER_CCE0_CSR = TIMER_CSR_EXPIRED;
  REG_XFER2_DMA_STATUS = TIMER_CSR_CLEAR;
  REG_XFER2_DMA_STATUS = TIMER_CSR_EXPIRED;
  usb4_irq_off();
  bank1_write(B1_LANE_OS1, 0x00);
  // hold the Type-C front end across the reset so the host sees a detach (released at boot)
  REG_CPU_EXEC_STATUS |= CPU_EXEC_STATUS_ACTIVE;
  REG_PHY_LINK_TRIGGER |= TYPEC_FE_HOLD;
  usb4_delay_ms(USB4_DETACH_MS);
  REG_CPU_RESET |= CPU_RESET_TRIGGER;
  while (1);
}

/*=== Interrupt entry ===*/

static void usb4_events(void) {
  uint8_t ev, st, l1;
  usb4_ctl_packet();
  usb4_resync();
  usb4_lane_slots();
  usb4_sb_rx();
  ev = bank1_rd(B1_SB_LINK_EVT);
  st = bank1_rd(B1_SB_LINK_EVT_MASK);
  if ((ev & SB_LINK_CONNECTED) && !(st & SB_LINK_CONNECTED)) {
    bank1_write(B1_SB_LINK_EVT, SB_LINK_CONNECTED);
    bank1_write(B1_SB_LINK_EVT, SB_LINK_DISCONNECTED);
    bank1_mod(B1_SB_LINK_EVT_MASK, SB_LINK_DISCONNECTED, SB_LINK_CONNECTED);
    usb4_sb_connected();
    usb4_phase = USB4_PHASE_NEGOTIATE;
    usb4_partner = USB4_PARTNER_IDLE;
    usb4_host_r12 = 0;
    usb4_con_ms = usb4_ms;
    usb4_partner_tries = 0;
  }
  if ((ev & SB_LINK_DISCONNECTED) && !(st & SB_LINK_DISCONNECTED)) {
    bank1_write(B1_SB_LINK_EVT, SB_LINK_DISCONNECTED);
    bank1_write(B1_SB_LINK_EVT, SB_LINK_CONNECTED);
    bank1_mod(B1_SB_LINK_EVT_MASK, SB_LINK_CONNECTED, SB_LINK_DISCONNECTED);
    usb4_link_reset();
  }
  ev = bank1_rd(B1_LINK_EVT) & (LINK_EVT_BONDED | LINK_EVT_FAIL);
  if (ev) {
    bank1_write(B1_LINK_EVT, ev);
    if (ev & LINK_EVT_BONDED) {
      bank1_write(B1_LSW_EVT, 0xFF);
      usb4_bond_cfg(0x02);
    }
    if (ev & LINK_EVT_FAIL) {
      if (!(bank1_rd(B1_LINK_RESYNC) & LINK_RESYNC_REQ) && !usb4_host_owns) usb4_link_reset();
    }
  }
  ev = bank1_rd(B1_SB_EVT) & (SB_EVT_L0_DISABLE | SB_EVT_L1_DISABLE);
  if (ev) {
    if (ev & SB_EVT_L0_DISABLE) {
      bank1_write(B1_SB_EVT, SB_EVT_L0_DISABLE);
      usb4_sb_ctl(0x80);
      usb4_link_reset();
    } else {
      bank1_write(B1_SB_EVT, SB_EVT_L1_DISABLE);
      usb4_sb_ctl(0xA0);
      bank1_write(B1_LANE1_TRIG, LANE_TRIG_HOLD);
      usb4_sb_regs[17] &= ~0x02;
    }
  }
  if (bank1_rd(B1_RTR_STS) & 0x80) {
    if (bank1_rd(B1_RTR_EVT) & 0x20) bank1_write(B1_RTR_EVT, 0x20);
    // router dword 0x96 (index bit 8 in CTL), bit 31: the host wrote the router
    if (usb4_cfg_go(USB4_ROUTER, 0x96, 0xFF, 0x01) && (bank1_rd(B1_ENG_RDATA + 3) & 0x80))
      usb4_host_owns = 1;
  }
  ev = bank1_rd(B1_LANE_EVT) & (LANE_EVT_L0_TRAIN | LANE_EVT_L1_TRAIN);
  if (ev) bank1_write(B1_LANE_EVT, ev);
  ev = bank1_rd(B1_LANE_EVT) & (LANE_EVT_L0_CL0 | LANE_EVT_L1_CL0);
  if (ev) {
    bank1_write(B1_LANE_EVT, ev);
    bank1_mod(B1_LANE_CL0_ACK, 0x00, ev);
    st = bank1_rd(B1_LANE0_STATE);
    l1 = bank1_rd(B1_LANE1_STATE);
    // link up once the other lane is in CL0 too (or unused)
    if (((ev & LANE_EVT_L0_CL0) &&
         (!(usb4_sb_regs[17] & 0x02) || (l1 & LANE_STATE_MASK) == LANE_STATE_CL0)) ||
        ((ev & LANE_EVT_L1_CL0) &&
         (!(usb4_sb_regs[17] & 0x01) || (st & LANE_STATE_MASK) == LANE_STATE_CL0)))
      bank1_mod(B1_LINK_CTRL_28D4, 0x00, LINK_28D4_UP);
  }
}

static uint8_t usb4_stepping(void) {
  if (usb4_partner == USB4_PARTNER_SENT || usb4_rr_wait || usb4_lost) return 1;
  if (usb4_phase < USB4_PHASE_NEGOTIATE || usb4_phase > USB4_PHASE_TRAIN) return 0;
  return 1;
}

// A running timer is kept unless a shorter interval is needed now.
static void usb4_rearm(void) {
  uint16_t next;
  next = usb4_stepping() ? 1 : USB4_HEARTBEAT_MS;
  if (usb4_armed && usb4_armed <= next) return;
  usb4_timer_arm(next);
}

static void usb4_on_timer(void) {
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  usb4_ms += usb4_armed;
  usb4_armed = 0;
  if (usb4_stepping()) {
    usb4_link_tick();
    // ROUTER_CS_6 Router Ready + Configuration Ready, ~1 ms after Enabled
    if (usb4_rr_wait && !--usb4_rr_wait) usb4_cfg_wr(USB4_ROUTER, 0x06, 0x8, 0x00, 0x03000000);
  }
  // both lanes out of CL0 after PCIe Enable: tear down after 200 ms, 2 s while a resync retrains
  if ((usb4_pcie_seen & PCIE_EVT_ENABLE) &&
      (bank1_rd(B1_LANE0_STATE) & LANE_STATE_MASK) != LANE_STATE_CL0 &&
      (bank1_rd(B1_LANE1_STATE) & LANE_STATE_MASK) != LANE_STATE_CL0) {
    if (++usb4_lost >= (usb4_phase == USB4_PHASE_TRIGGERED ? 200 : 2000)) usb4_link_reset();
  } else {
    usb4_lost = 0;
  }
}

// All INT1 sources are level-triggered W1C status bits: every pass runs every handler.
static void usb4_isr(void) {
  if (REG_TIMER1_CSR & TIMER_CSR_EXPIRED) usb4_on_timer();
  // the lane sequences unmask CCF8's IRQ; ack a stray expiry so it cannot hold INT1
  if (REG_CPU_EXT_STATUS & TIMER_CSR_EXPIRED) REG_CPU_EXT_STATUS = TIMER_CSR_EXPIRED;
  usb4_events();
  usb4_typec_event(1);
  pd_poll();
  pd_usb4_go = 0;
  // a block that keeps timing out: back to waiting for the next sideband connect
  if (usb4_to_eng > 4 || usb4_to_sb > 4 || usb4_to_timer > 4 || usb4_to_x > 4) {
    usb4_link_reset();
    usb4_to_eng = usb4_to_sb = usb4_to_timer = usb4_to_x = 0;
  }
  usb4_rearm();
}

/*=== Entry points from main.c ===*/

#define TCON_IT1 0x04  // cleared = level-triggered
// INT1 is level-triggered: mask it after this many passes in a row with nothing to ack
#define INT1_IDLE_MAX 32
// sleep(1) passes, ~2 ms each
#define USB4_PARTNER_PASSES    30000  // ~60 s
#define USB4_PD_PASSES         750    // ~1.5 s
#define USB4_PD_PASSES_RX      3000   // ~6 s once the partner has sent a PD message
static __xdata uint8_t int1_idle;

static void usb4_init(void) {
  usb4_irq_off();
  int1_idle = 0;
  TCON &= ~TCON_IT1;
}

// forces a detach the host can see
static void usb4_typec_pulse(void) {
  REG_INT_ENABLE |= INT_ENABLE_BIT6;
  REG_CPU_EXEC_STATUS |= CPU_EXEC_STATUS_ACTIVE;  // Type-C front end
  REG_INT_ENABLE |= INT_ENABLE_BIT5;
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_INT;  // Type-C attach / detach event
  REG_CPU_EXEC_CTRL_2 &= ~CPU_EXEC_CTRL_2_BIT2;
  REG_PHY_LINK_TRIGGER |= TYPEC_FE_HOLD;
  sleep(50);
  REG_PHY_LINK_TRIGGER &= ~TYPEC_FE_HOLD;
}

static void usb4_wait_partner(void) {
  uint16_t n;
  for (n = 0; !(REG_PHY_EXT_B3 & PHY_EXT_LINK_READY) && n < USB4_PARTNER_PASSES; n++)  // attached
    sleep(1);
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_INT;
  REG_CPU_EXEC_STATUS_2 = CPU_EXEC_STATUS_2_BIT1;
}

// Does not return once USB4 is up.
static void usb4_boot_window(void) {
  uint16_t n;
  pd_on = pd_usb4_go = 0;  // XDATA survives usb4_reset's CPU reset
  usb4_typec_pulse();
  usb4_wait_partner();
  pd_rx_count = 0;
  for (n = 0; n < (pd_rx_count ? USB4_PD_PASSES_RX : USB4_PD_PASSES); n++) {
    pd_poll();
    if (pd_usb4_go) {
      REG_PCIE_TLP_CTRL   = 0x01;
      REG_PCIE_TLP_LENGTH = 0x20;
      pcie_apply_x2_rxphy_tuning();
      usb4_begin();
      while (1);
    }
    sleep(1);
  }
}

// PD does not raise INT1 in USB mode: drain the RX ring here. Does not return once USB4 is up.
static void usb4_usb_mode_pd(void) {
  uint8_t i;
  pd_usb4_go = 0;
  for (i = 0; i < PD_RX_SLOTS && !pd_usb4_go; i++)
    if (!pd_poll() && !(REG_PD_INT & PD_INT_RX)) break;
  if (!pd_usb4_go) return;
  usb4_begin();
  while (1);
}

static void usb4_int1(void) {
  uint8_t dpx = DPX;  // not XDATA: an XDATA store would land in the interrupted page
  DPX = 0;
  if (usb4_active) {
    usb4_isr();
  } else {
    if (usb4_typec_event(0)) int1_idle = 0;
    else if (++int1_idle == INT1_IDLE_MAX) IE &= ~IE_EX1;
  }
  DPX = dpx;
}

#endif
