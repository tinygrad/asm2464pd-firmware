#ifndef PD_H
#define PD_H

#include "types.h"
#include "registers.h"

// control messages (no data objects)
#define PD_ACCEPT        0x03
#define PD_REJECT        0x04
#define PD_PS_RDY        0x06
#define PD_GET_SINK_CAP  0x08
#define PD_DR_SWAP       0x09
#define PD_VCONN_SWAP    0x0B
#define PD_WAIT          0x0C
#define PD_SOFT_RESET    0x0D
#define PD_NOT_SUPPORTED 0x10
// data messages
#define PD_SOURCE_CAP    0x01
#define PD_REQUEST       0x02
#define PD_SINK_CAP      0x04
#define PD_ENTER_USB     0x08
#define PD_VDM           0x0F

#define PD_REV20         0x40
#define PD_REV30         0x80
#define PD_SPIN          0xFFFF
#define PD_NO_MSG_ID     0xFF
#define PD_RX_SLOTS      16

#define VDM_DISCOVER_IDENTITY 0x01
#define VDM_ACK               0x40
#define VDM_NAK               0x80

#define EUDO_HOST_PRESENT     0x20  // byte 1
#define EUDO_CABLE_SPEED      0xE0  // byte 2
#define EUDO_USB_MODE         0x70  // byte 3
#define EUDO_USB_MODE_USB4    0x20

static __xdata uint8_t pd_on, pd_rx_slot, pd_msg_id, pd_last_rx_id, pd_rev;
static __xdata uint8_t pd_usb4_go;
static __xdata uint8_t pd_rx_count, pd_eudo[4];

// Discover Identity ACK: ID Header, Cert Stat, Product, UFP VDO (little endian)
static __code const uint8_t pd_identity[16] = {
  U16_LE(USB_VID), 0x40, 0x50,  // Type-C receptacle, USB device, peripheral
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, U16_LE(USB_PID),
  0x03, 0x00, 0x80, 0x6D,  // UFP VDO 1.3: USB4/USB3.2/USB2 device, highest speed USB4 Gen3
};

static void pd_init(void) {
  uint16_t n;
  pd_rx_slot = pd_msg_id = 0;
  pd_last_rx_id = PD_NO_MSG_ID;
  pd_rev = PD_REV30;
  pd_rx_count = 0;
  REG_PD_CFG_E40B |= 0x40;
  REG_PD_CFG_E40A = 0x0F;
  REG_PD_CFG_E413 &= ~0x03;
  REG_PD_CTRL &= ~PD_CTRL_ENABLE;
  sleep(10);
  REG_PD_CFG_E40B |= 0x01;
  sleep(60);
  for (n = 0; (REG_PD_STATUS & 0x08) && n < PD_SPIN; n++);
  REG_PD_CFG_E409 &= ~0x01;
  REG_PD_CFG_E409 |= 0x40;
  REG_PD_TX_HDR_L = PD_REV20;
  REG_PD_CFG_E409 = (REG_PD_CFG_E409 & 0xF1) | 0x06;
  REG_PD_CTRL |= PD_CTRL_RX_RING;
  REG_PD_CFG_E411 = 0xA1;
  REG_PD_CFG_E412 = 0x79;
  REG_PD_CTRL = (REG_PD_CTRL & 0xC3) | 0x3C;
  REG_PD_CFG_E409 &= ~0x80;
  REG_INT_CTRL |= 0x20;
  REG_PD_CFG_E40B |= 0x80;
  if (REG_PD_CFG_E40B & 0x80) {
    REG_PD_CFG_E401 = 0xB4;
    REG_PD_CFG_E406 = 0xA6;
    REG_PD_CFG_E407 = (REG_PD_CFG_E407 & 0xE0) | 0x15;
    REG_PD_CFG_E408 = (REG_PD_CFG_E408 & 0xE0) | 0x1C;
  }
  REG_PD_CFG_E40E = 0x8A;
  REG_PD_CTRL |= PD_CTRL_ENABLE;
  REG_PD_CFG_E40B &= ~0x01;
  REG_PD_CFG_E40D = 0x28;
  REG_PD_CFG_E413 = (REG_PD_CFG_E413 & 0x8F) | 0x60;
  REG_PD_CFG_E40B &= ~0x20;
}

static uint8_t pd_send(uint8_t type, uint8_t nobj) {
  uint16_t n;
  for (n = 0; ((REG_PD_STATUS & PD_STATUS_BUSY) || (REG_PD_TX_START & PD_TX_START)) && n < PD_SPIN;
       n++);
  if (n == PD_SPIN) return 0;
  REG_PD_TX_SOP &= 0xF8;
  REG_PD_TX_HDR_L = pd_rev | type;
  REG_PD_TX_HDR_H = (nobj << 4) | (pd_msg_id << 1);
  REG_PD_TX_LEN = 2 + 4 * nobj;
  REG_PD_TX_START |= PD_TX_START;
  for (n = 0; (REG_PD_TX_START & PD_TX_START) && n < PD_SPIN; n++);
  pd_msg_id = (pd_msg_id + 1) & 0x07;
  return n != PD_SPIN;
}

static void pd_tx_obj(uint8_t i, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  REG_PD_TX_DATA(4 * i) = b0;
  REG_PD_TX_DATA(4 * i + 1) = b1;
  REG_PD_TX_DATA(4 * i + 2) = b2;
  REG_PD_TX_DATA(4 * i + 3) = b3;
}

// RDO for PDO 1 (always vSafe5V) at its full advertised current
static void pd_request_5v(uint8_t slot) {
  uint16_t ma10;
  pd_rev = (REG_PD_RX(slot, 0) & 0xC0) >= PD_REV30 ? PD_REV30 : PD_REV20;
  ma10 = REG_PD_RX(slot, 2) | ((REG_PD_RX(slot, 3) & 0x03) << 8);
  pd_tx_obj(0, ma10 & 0xFF, ((ma10 >> 8) & 0x03) | ((ma10 & 0x3F) << 2), (ma10 >> 6) & 0x0F,
            0x12);  // object position 1, USB communications capable
  pd_send(PD_REQUEST, 1);
}

static void pd_not_supported(void) {
  if (pd_rev == PD_REV30) pd_send(PD_NOT_SUPPORTED, 0);
}

static void pd_vdm(uint8_t slot) {
  uint8_t i, cmd = REG_PD_RX(slot, 2), ver = REG_PD_RX(slot, 3);
  if (!(ver & 0x80)) { pd_not_supported(); return; }
  if (cmd & 0xC0) return;  // only structured VDM requests get an answer
  if ((cmd & 0x1F) == VDM_DISCOVER_IDENTITY && REG_PD_RX(slot, 4) == 0x00 &&
      REG_PD_RX(slot, 5) == 0xFF) {
    pd_tx_obj(0, VDM_DISCOVER_IDENTITY | VDM_ACK, ver & 0xF8, 0x00, 0xFF);
    for (i = 0; i < sizeof(pd_identity); i++) REG_PD_TX_DATA(4 + i) = pd_identity[i];
    pd_send(PD_VDM, 5);
  } else {
    pd_tx_obj(0, (cmd & 0x1F) | VDM_NAK, ver, REG_PD_RX(slot, 4), REG_PD_RX(slot, 5));
    pd_send(PD_VDM, 1);
  }
}

static void pd_enter_usb(uint8_t slot) {
  uint8_t i;
  for (i = 0; i < 4; i++) pd_eudo[i] = REG_PD_RX(slot, 2 + i);
  if ((pd_eudo[3] & EUDO_USB_MODE) == EUDO_USB_MODE_USB4 && (pd_eudo[2] & EUDO_CABLE_SPEED) &&
      (pd_eudo[1] & EUDO_HOST_PRESENT)) {
    if (pd_send(PD_ACCEPT, 0)) pd_usb4_go = 1;
  } else {
    pd_send(PD_REJECT, 0);
  }
}

static void pd_handle(uint8_t slot, uint8_t type, uint8_t nobj, uint8_t extended) {
  if (extended) { pd_not_supported(); return; }
  if (!nobj) {
    if (type == PD_GET_SINK_CAP) {
      pd_tx_obj(0, 0x2C, 0x91, 0x01, 0x04);  // fixed 5V 3A, USB communications capable
      pd_send(PD_SINK_CAP, 1);
    } else if (type >= PD_DR_SWAP && type <= PD_VCONN_SWAP) {
      pd_send(PD_REJECT, 0);
    } else if (type > PD_PS_RDY && type != PD_WAIT && type != PD_SOFT_RESET &&
               type != PD_NOT_SUPPORTED) {
      pd_not_supported();
    }
  } else if (type == PD_SOURCE_CAP) {
    pd_request_5v(slot);
  } else if (type == PD_VDM) {
    pd_vdm(slot);
  } else if (type == PD_ENTER_USB) {
    pd_enter_usb(slot);
  } else {
    pd_not_supported();
  }
}

// Returns 0 when there was nothing to do or an acked status bit did not clear.
static uint8_t pd_poll(void) {
  uint8_t ev, hdr_l, hdr_h, id;
  if (!(REG_PD_ATTACH & PD_ATTACH_SRC)) {
    if (!pd_on) return 0;
    pd_on = 0;
    return 1;
  }
  if (!pd_on) {
    pd_init();
    pd_on = 1;
  }
  ev = REG_PD_INT2 & 0xF9;
  if (ev) {
    REG_PD_INT2 = ev;
    if ((ev & PD_INT2_TX_DISCARDED) && !(REG_PD_INT2 & PD_INT2_TX_DISCARDED))
      pd_msg_id = (pd_msg_id - 1) & 0x07;
    return !(REG_PD_INT2 & ev);
  }
  ev = REG_PD_INT;
  if (ev & (PD_INT_HARD_RESET | PD_INT_SOFT_RESET)) {
    REG_PD_INT = ev & (PD_INT_HARD_RESET | PD_INT_SOFT_RESET);
    if (ev & PD_INT_HARD_RESET) pd_rx_slot = 0;
    pd_msg_id = 0;
    pd_last_rx_id = PD_NO_MSG_ID;
    if (ev & PD_INT_SOFT_RESET) pd_send(PD_ACCEPT, 0);
    return !(REG_PD_INT & ev & (PD_INT_HARD_RESET | PD_INT_SOFT_RESET));
  }
  if (!(ev & PD_INT_RX)) return 0;
  REG_PD_INT = PD_INT_RX;
  hdr_l = REG_PD_RX(pd_rx_slot, 0);
  hdr_h = REG_PD_RX(pd_rx_slot, 1);
  id = (hdr_h >> 1) & 0x07;
  pd_rx_count++;
  if (id != pd_last_rx_id) {
    pd_last_rx_id = id;
    pd_handle(pd_rx_slot, hdr_l & 0x1F, (hdr_h >> 4) & 0x07, hdr_h & 0x80);
  }
  pd_rx_slot = (pd_rx_slot + 1) & 0x0F;
  return !(REG_PD_INT & PD_INT_RX);
}

#endif
