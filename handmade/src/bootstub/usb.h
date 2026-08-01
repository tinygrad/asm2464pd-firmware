#ifndef BOOTSTUB_USB_H
#define BOOTSTUB_USB_H

/* Minimal USB2 EP0 transport for bootstub recovery.  Keep this independent of
 * the application's USB3/USB4 and bulk endpoint implementation. */

#include "registers.h"
#include "types.h"
#include "util.h"

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)
#define USB_EP0_SIZE 64U
#define USB_VID 0xADD1
#define USB_PID 0xB007
#define U16_LE(v) ((v) & 0xFF), (((v) >> 8) & 0xFF)

static uint8_t usb_configuration;

static __code const uint8_t usb_dev_desc[] = {
  0x12, 0x01, U16_LE(0x0200),
  0x00, 0x00, 0x00, 0x40,
  U16_LE(USB_VID), U16_LE(USB_PID), U16_LE(0x0001),
  0x00, 0x00, 0x00, 0x01,
};

static __code const uint8_t usb_cfg_desc[] = {
  0x09, 0x02, U16_LE(18), 0x01, 0x01, 0x00, 0xC0, 0x00,
  0x09, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
};

static void usb_send_data(uint16_t len) {
  REG_USB_EP0_LEN_H = len >> 8;
  REG_USB_EP0_LEN_L = len;
  REG_USB_DMA_TRIGGER = USB_DMA_SEND;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
}

static void usb_send_zlp(void) { usb_send_data(0); }

static void usb_stall_ep0(void) {
  REG_USB_DMA_TRIGGER = USB_DMA_STALL;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
}

static uint8_t usb_wait_ep0_dma_idle(void) {
  uint16_t timeout = 0xFFFF;
  do {
    if (REG_USB_DMA_TRIGGER == 0)
      return 1;
  } while (--timeout);
  return 0;
}

static void usb_copy_desc(__code const uint8_t *src, uint8_t len) {
  uint8_t i;
  for (i = 0; i < len; i++)
    DESC_BUF[i] = src[i];
}

static void usb_get_descriptor(uint8_t type, uint8_t index, uint16_t requested) {
  __code const uint8_t *src;
  uint8_t len;

  if (type == USB_DESC_TYPE_DEVICE && index == 0) {
    src = usb_dev_desc;
    len = sizeof(usb_dev_desc);
  } else if (type == USB_DESC_TYPE_CONFIG && index == 0) {
    src = usb_cfg_desc;
    len = sizeof(usb_cfg_desc);
  } else {
    usb_stall_ep0();
    return;
  }

  usb_copy_desc(src, len);
  usb_send_data(requested < len ? requested : len);
}

static bool usb_handle_custom_setup(uint8_t bmReq, uint8_t bReq, uint8_t wValL, uint8_t wValH, uint16_t wLen);

static void usb_handle_setup(void) {
  uint8_t bmReq = REG_USB_SETUP_BMREQ;
  uint8_t bReq = REG_USB_SETUP_BREQ;
  uint8_t wValL = REG_USB_SETUP_WVAL_L;
  uint8_t wValH = REG_USB_SETUP_WVAL_H;
  uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

  if (usb_handle_custom_setup(bmReq, bReq, wValL, wValH, wLen))
    return;

  if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS && wValH == 0 && wLen == 0) {
    REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
    REG_USB_EP_CTRL_91D0 = 0x02;
    usb_send_zlp();
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
    usb_get_descriptor(wValH, wValL, wLen);
  } else if ((bmReq == USB_SETUP_DIR_DEV_TO_HOST ||
      bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) ||
      bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_ENDPOINT)) &&
      bReq == USB_REQ_GET_STATUS && wValL == 0 && wValH == 0 && wLen == 2) {
    DESC_BUF[0] = bmReq == USB_SETUP_DIR_DEV_TO_HOST;
    DESC_BUF[1] = 0;
    usb_send_data(2);
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_CONFIGURATION && wLen == 1) {
    DESC_BUF[0] = usb_configuration;
    usb_send_data(1);
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION &&
      wValH == 0 && wValL <= 1 && wLen == 0) {
    usb_configuration = wValL;
    usb_send_zlp();
  } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) &&
      bReq == USB_REQ_GET_INTERFACE && wLen == 1) {
    DESC_BUF[0] = 0;
    usb_send_data(1);
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) &&
      bReq == USB_REQ_SET_INTERFACE && wValL == 0 && wValH == 0 && wLen == 0) {
    usb_send_zlp();
  } else {
    usb_stall_ep0();
  }
}

/* These lane values are required for the controller to report USB events even
 * when it is forced to its USB2 path. */
static void usb_rmw(uint16_t addr, uint8_t and_mask, uint8_t or_value) {
  XREG(addr) = (XREG(addr) & and_mask) | or_value;
}

typedef struct {
  uint8_t offset;
  uint8_t and_mask;
  uint8_t or_value;
} usb_tune_t;

static __code const usb_tune_t usb_tuning[] = {
  {0x02, 0x00, 0xB0}, {0x03, 0xF3, 0x00}, {0x04, 0x8F, 0x40}, {0x05, 0x0F, 0x60},
  {0x06, 0xF0, 0x07}, {0x07, 0x1F, 0x60}, {0x09, 0x0F, 0x90}, {0x0B, 0xC0, 0x0A},
  {0x0C, 0xFD, 0x00}, {0x10, 0xE0, 0x03}, {0x11, 0xE0, 0x08}, {0x12, 0x1F, 0x20},
  {0x13, 0xF3, 0x04}, {0x14, 0xFF, 0x06}, {0x15, 0xF0, 0x0C}, {0x16, 0xF0, 0x0F},
  {0x17, 0x1F, 0x40}, {0x19, 0x0F, 0x80}, {0x1A, 0xF0, 0x0E}, {0x1B, 0xC0, 0x00},
  {0x1C, 0xFD, 0x02}, {0x20, 0xE0, 0x03}, {0x21, 0xE0, 0x08}, {0x22, 0xE0, 0x0A},
  {0x23, 0xFC, 0x02}, {0x24, 0xF0, 0x07}, {0x25, 0xF0, 0x0F}, {0x26, 0xF0, 0x0B},
  {0x27, 0x1F, 0x40}, {0x29, 0x0F, 0x80}, {0x2A, 0xFF, 0x01}, {0x2B, 0xC0, 0x00},
  {0x2C, 0xFD, 0x02}, {0x3C, 0xFD, 0x00}, {0x43, 0xC3, 0x1C}, {0x45, 0xF0, 0x0B},
  {0x46, 0xF0, 0x0D}, {0x49, 0x80, 0x41}, {0x4A, 0xFE, 0x00}, {0x4C, 0xF1, 0x0E},
  {0x4E, 0xFF, 0x40}, {0x5B, 0xE0, 0x1B},
};

static void usb_tune_lane(uint16_t base) {
  uint8_t i;
  for (i = 0; i < sizeof(usb_tuning) / sizeof(usb_tuning[0]); i++) {
    __code const usb_tune_t *t = &usb_tuning[i];
    usb_rmw(base + t->offset, t->and_mask, t->or_value);
  }
}

static void usb_phy_tune(void) {
  usb_tune_lane(0xC280);
  usb_tune_lane(0xC300);
}

static void usb_reinit_controller(void) {
  usb_configuration = 0;
  REG_DMA_CONFIG = DMA_CONFIG_DISABLE;
  usb_init_endpoint_state();
  REG_POWER_STATUS &= (uint8_t)~POWER_STATUS_USB_PATH;
  REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;
  REG_USB_EP0_CFG = 0xF0;
  REG_BUF_CFG_9303 = 0x33;
  REG_CPU_MODE = CPU_MODE_USB2;
  REG_USB_PHY_CTRL_91C0 = USB_PHY_91C0_FORCE_HS;
}

static void usb_attach_controller(void) {
  REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
  REG_USB_POWER_CYCLE = 0;
  timer_delay_ms(25);
  REG_USB_POWER_CYCLE = USB_POWER_CYCLE_TRIGGER;
  timer_delay_ms(25);
}

#endif
