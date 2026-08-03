#pragma once

#include "types.h"
#include "registers.h"
#include "util.h"

#define DESC_BUF               ((__xdata uint8_t *)USB_CTRL_BUF_BASE)
#define USB_EP0_SIZE           64U
#define BOOTSTUB_U16_LE(v)     ((v) & 0xFF), (((v) >> 8) & 0xFF)

static __code const uint8_t usb_dev_desc[] = {
  0x12, USB_DESC_TYPE_DEVICE, BOOTSTUB_U16_LE(0x0200),
  0x00, 0x00, 0x00, USB_EP0_SIZE,
  BOOTSTUB_U16_LE(0x3801), BOOTSTUB_U16_LE(0xB007), BOOTSTUB_U16_LE(0x0001),
  0x00, 0x00, 0x00, 0x01,
};

static __code const uint8_t usb_cfg_desc[] = {
  0x09, USB_DESC_TYPE_CONFIG, BOOTSTUB_U16_LE(18), 0x01, 0x01, 0x00, 0xC0, 0x00,
  0x09, USB_DESC_TYPE_INTERFACE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
};

static bool usb_handle_custom_setup(uint8_t bmReq, uint8_t bReq, uint8_t wValL, uint8_t wValH, uint16_t wLen);

static void usb_send_data(uint16_t len) {
  REG_USB_EP0_LEN_H = len >> 8; REG_USB_EP0_LEN_L = len;
  REG_USB_DMA_TRIGGER = USB_DMA_SEND; REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
}

static void usb_send_zlp(void) { usb_send_data(0); }

static void usb_stall_ep0(void) { REG_USB_DMA_TRIGGER = USB_DMA_STALL; REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL; }

static uint8_t usb_wait_ep0_dma_idle(void) {
  uint16_t timeout = 0xFFFF;
  while (REG_USB_DMA_TRIGGER && --timeout);
  return timeout != 0;
}

static void usb_send_descriptor(__code const uint8_t *src, uint8_t len, uint16_t requested) {
  for (uint8_t i = 0; i < len; i++) DESC_BUF[i] = src[i];
  usb_send_data(requested < len ? requested : len);
}

static void usb_handle_setup(void) {
  uint8_t bmReq = REG_USB_SETUP_BMREQ, bReq = REG_USB_SETUP_BREQ;
  uint8_t wValL = REG_USB_SETUP_WVAL_L, wValH = REG_USB_SETUP_WVAL_H;
  uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

  if (usb_handle_custom_setup(bmReq, bReq, wValL, wValH, wLen)) return;
  if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS && wValH == 0 && wLen == 0) {
    REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
    REG_USB_EP_CTRL_91D0 = 0x02;
    usb_send_zlp();
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR && wValL == 0) {
    if (wValH == USB_DESC_TYPE_DEVICE) usb_send_descriptor(usb_dev_desc, sizeof(usb_dev_desc), wLen);
    else if (wValH == USB_DESC_TYPE_CONFIG) usb_send_descriptor(usb_cfg_desc, sizeof(usb_cfg_desc), wLen);
    else usb_stall_ep0();
  } else if ((bmReq == USB_SETUP_DIR_DEV_TO_HOST || bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) ||
      bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_ENDPOINT)) &&
      bReq == USB_REQ_GET_STATUS && wValL == 0 && wValH == 0 && wLen == 2) {
    *(__xdata uint16_t *)DESC_BUF = bmReq == USB_SETUP_DIR_DEV_TO_HOST;
    usb_send_data(2);
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION && wValH == 0 && wValL <= 1 && wLen == 0) {
    usb_send_zlp();
  } else {
    usb_stall_ep0();
  }
}

static void usb_init(void) {
  REG_USB_EP0_LEN_H = 0; REG_USB_EP0_LEN_L = 0;
  REG_USB_DMA_TRIGGER = 0; REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
  REG_POWER_STATUS &= ~POWER_STATUS_USB_PATH;
  REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;
  REG_USB_EP0_CFG = 0xF0;
  REG_USB_DATA_L = 0;
  REG_USB_EP_MGMT = 0;
  REG_BUF_CFG_9303 = 0x33;
  REG_CPU_MODE = CPU_MODE_USB2;
  REG_USB_PHY_CTRL_91C0 = USB_PHY_91C0_FORCE_HS;
  REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
  REG_USB_POWER_CYCLE = 0;
  timer_delay_ms(25);
  REG_USB_POWER_CYCLE = USB_POWER_CYCLE_TRIGGER;
  timer_delay_ms(25);
}
