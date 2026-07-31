#ifndef USB_H
#define USB_H

#include "types.h"
#include "registers.h"
#include "flash.h"
#include "util.h"

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)

static uint8_t is_usb2;
static uint8_t usb_configuration;

#define USB_EP0_SIZE (is_usb2 ? 64U : 512U)

/*=== USB device identification ===*/
#define USB_VID                 0x3801
#define USB_LANG_ID             0x0409   /* US English */

/* String descriptors */
#define USB_STR_MFG             "tiny"

#define USB_STR_IDX_LANG        0
#define USB_STR_IDX_MFG         1
#define USB_STR_IDX_PRODUCT     2
#define USB_STR_IDX_SERIAL      3

/*=== Helpers ===*/
#define U16_LE(v)               ((v) & 0xFF), (((v) >> 8) & 0xFF)

#ifdef BOOTSTUB

#define USB_PID                 0xB007
#define USB_STR_PRODUCT         "bootstub"

static __code const uint8_t usb_cfg_desc[] = {
  0x09, 0x02, U16_LE(18), 0x01, 0x01, 0x00, 0xC0, 0x00,
  0x09, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
};

#define usb_cfg_desc_ss usb_cfg_desc

#else

#define USB_PID                 0x0001
#define USB_STR_PRODUCT         "custom v0.1"

/*=== Configuration descriptors ===*/

/* USB 2.0: 1 interface, 4 bulk EPs @ 64 B (FS) / 512 B (HS). Total=46. */
static __code const uint8_t usb_cfg_desc[] = {
  0x09, 0x02, U16_LE(46), 0x01, 0x01, 0x00, 0xC0, 0x00,
  /* Interface 0: vendor class, 4 bulk EPs */
  0x09, 0x04, 0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, U16_LE(512), 0x00,  /* EP1 IN  bulk */
  0x07, 0x05, 0x02, 0x02, U16_LE(512), 0x00,  /* EP2 OUT bulk */
  0x07, 0x05, 0x83, 0x02, U16_LE(512), 0x00,  /* EP3 IN  bulk */
  0x07, 0x05, 0x04, 0x02, U16_LE(512), 0x00,  /* EP4 OUT bulk */
};

/* USB 3.x: alt 0 = BBB (2 EPs), alt 1 = UAS (4 EPs). Total=121. */
static __code const uint8_t usb_cfg_desc_ss[] = {
  0x09, 0x02, U16_LE(121), 0x01, 0x01, 0x00, 0xC0, 0x00,
  /* Alt 0: BBB */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, U16_LE(1024), 0x00,
  0x06, 0x30, 0x0F, 0x00, U16_LE(0x0000),    /* SS Companion: bMaxBurst=15 */
  0x07, 0x05, 0x02, 0x02, U16_LE(1024), 0x00,
  0x06, 0x30, 0x0F, 0x00, U16_LE(0x0000),
  /* Alt 1: UAS — 4 bulk EPs + SS companions + pipe usage */
  0x09, 0x04, 0x00, 0x01, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, U16_LE(1024), 0x00,           /* EP1 IN  Status */
  0x06, 0x30, 0x0F, 0x05, U16_LE(0x0000),
  0x04, 0x24, 0x03, 0x00,
  0x07, 0x05, 0x02, 0x02, U16_LE(1024), 0x00,           /* EP2 OUT Command */
  0x06, 0x30, 0x0F, 0x05, U16_LE(0x0000),
  0x04, 0x24, 0x04, 0x00,
  0x07, 0x05, 0x83, 0x02, U16_LE(1024), 0x00,           /* EP3 IN  Data-In */
  0x06, 0x30, 0x0F, 0x05, U16_LE(0x0000),
  0x04, 0x24, 0x02, 0x00,
  0x07, 0x05, 0x04, 0x02, U16_LE(1024), 0x00,           /* EP4 OUT Data-Out */
  0x06, 0x30, 0x00, 0x00, U16_LE(0x0000),
  0x04, 0x24, 0x01, 0x00,
};

#endif

/*=== Device descriptors ===*/

static __code const uint8_t usb_dev_desc[] = {
  0x12, 0x01, U16_LE(0x0200),
  0x00, 0x00, 0x00, 0x40,
  U16_LE(USB_VID), U16_LE(USB_PID), U16_LE(0x0001),
  USB_STR_IDX_MFG, USB_STR_IDX_PRODUCT, USB_STR_IDX_SERIAL, 0x01,
};

static __code const uint8_t usb_dev_desc_ss[] = {
  0x12, 0x01, U16_LE(0x0320),
  0x00, 0x00, 0x00, 0x09,
  U16_LE(USB_VID), U16_LE(USB_PID), U16_LE(0x0001),
  USB_STR_IDX_MFG, USB_STR_IDX_PRODUCT, USB_STR_IDX_SERIAL, 0x01,
};

/*=== BOS descriptor ===*/

static __code const uint8_t usb_bos_desc[] = {
  0x05, 0x0F, U16_LE(22), 0x02,                                /* BOS, 2 caps */
  0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,                    /* USB 2.0 Extension */
  0x0A, 0x10, 0x03, 0x00, 0x0E, 0x00, 0x03, 0x00, 0x00, 0x00,  /* SS Capability */
};

/* Encode `s` as a UTF-16LE STRING descriptor in `buf`. Returns total length. */
static uint8_t usb_build_string_desc(__code const char *s, __xdata uint8_t *buf) {
  uint8_t i = 0;
  while (s[i]) {
    buf[2 + 2*i] = s[i];
    buf[2 + 2*i + 1] = 0;
    i++;
  }
  buf[0] = 2 + 2*i;
  buf[1] = 0x03;
  return 2 + 2*i;
}

/* Build a STRING descriptor from the OTP-stored 4-byte serial, lowercase
 * ASCII hex (8 chars). Falls back to "ffffffff" when the OTP is blank,
 * corrupt, or carries an unknown version. */
static uint8_t usb_build_serial_desc(__xdata uint8_t *buf) {
  static __code const char hex[] = "0123456789abcdef";
  __xdata otp_t otp;
  __xdata uint8_t serial[4];
  uint8_t i, b;
  if (flash_read_otp(&otp)) {
    for (i = 0; i < 4; i++) serial[i] = otp.serial[i];
  } else {
    for (i = 0; i < 4; i++) serial[i] = 0xFF;
  }
  buf[0] = 2 + 2 * (4 * 2);
  buf[1] = 0x03;
  for (i = 0; i < 4; i++) {
    b = serial[i];
    buf[2 + 4*i + 0] = hex[b >> 4];
    buf[2 + 4*i + 1] = 0;
    buf[2 + 4*i + 2] = hex[b & 0x0F];
    buf[2 + 4*i + 3] = 0;
  }
  return buf[0];
}

/* SS PHY tuning. Required even when we end up at HS — without it the
 * controller never pushes events to PERIPH_STATUS. */
static void rmw(uint16_t addr, uint8_t and_mask, uint8_t or_val) {
    XDATA_REG8(addr) = (XDATA_REG8(addr) & and_mask) | or_val;
}

static void usb_serdes_tune_lane(uint16_t base) {
    rmw(base + 0x02, 0x00, 0xB0); // b[4:0] correlates with TX drive strength
    rmw(base + 0x03, 0xF3, 0x00);
    rmw(base + 0x04, 0x8F, 0x40); rmw(base + 0x05, 0x0F, 0x60);
    rmw(base + 0x06, 0xF0, 0x07); rmw(base + 0x07, 0x1F, 0x60);
    rmw(base + 0x09, 0x0F, 0x90); rmw(base + 0x0B, 0xC0, 0x0A);
    rmw(base + 0x0C, 0xFD, 0x00); rmw(base + 0x10, 0xE0, 0x03);
    rmw(base + 0x11, 0xE0, 0x08); rmw(base + 0x12, 0x1F, 0x20);
    rmw(base + 0x13, 0xF3, 0x04); rmw(base + 0x14, 0xFF, 0x06);
    rmw(base + 0x15, 0xF0, 0x0C); rmw(base + 0x16, 0xF0, 0x0F);
    rmw(base + 0x17, 0x1F, 0x40); rmw(base + 0x19, 0x0F, 0x80);
    rmw(base + 0x1A, 0xF0, 0x0E); rmw(base + 0x1B, 0xC0, 0x00);
    rmw(base + 0x1C, 0xFD, 0x02); rmw(base + 0x20, 0xE0, 0x03);
    rmw(base + 0x21, 0xE0, 0x08); rmw(base + 0x22, 0xE0, 0x0A);
    rmw(base + 0x23, 0xFC, 0x02); rmw(base + 0x24, 0xF0, 0x07);
    rmw(base + 0x25, 0xF0, 0x0F); rmw(base + 0x26, 0xF0, 0x0B);
    rmw(base + 0x27, 0x1F, 0x40); rmw(base + 0x29, 0x0F, 0x80);
    rmw(base + 0x2A, 0xFF, 0x01); rmw(base + 0x2B, 0xC0, 0x00);
    rmw(base + 0x2C, 0xFD, 0x02); rmw(base + 0x3C, 0xFD, 0x00);
    rmw(base + 0x43, 0xC3, 0x1C); rmw(base + 0x45, 0xF0, 0x0B);
    rmw(base + 0x46, 0xF0, 0x0D); rmw(base + 0x49, 0x80, 0x41);
    rmw(base + 0x4A, 0xFE, 0x00); rmw(base + 0x4C, 0xF1, 0x0E);
    rmw(base + 0x4E, 0xFF, 0x40); rmw(base + 0x5B, 0xE0, 0x1B);
}

static void usb_phy_tune(void) {
    usb_serdes_tune_lane(0xC280);  /* lane 0 */
    usb_serdes_tune_lane(0xC300);  /* lane 1 */
}

/* Bring up the USB PIPE/PHY engine; run unconditionally at boot. */
static void usb_pipe_engine_init(void) {
    REG_POWER_ENABLE      = (REG_POWER_ENABLE & 0x7F) | 0x80;
    REG_USB_PHY_CTRL_91D1 = 0x0F;
    REG_BUF_CFG_9300      = 0x0C;
    REG_BUF_CFG_9301      = 0xC0;
    REG_BUF_CFG_9302      = 0xBF;
    REG_USB_CTRL_PHASE    = 0x1F;
    REG_USB_EP_CFG1       = 0x0F;
    REG_USB_PHY_CTRL_91C1 = 0xF0;
    REG_BUF_CFG_9303      = 0x33;
    REG_BUF_CFG_9304      = 0x3F;
    REG_BUF_CFG_9305      = 0x40;
    REG_USB_CONFIG        = 0xE0;
    REG_USB_EP0_CFG       = 0xF0;
    REG_USB_MODE          = 0x01;
    REG_USB_EP_MGMT      &= (uint8_t)~0x01;
    REG_USB_MSC_CTRL      = 0x01;
    REG_USB_MSC_STATUS   &= (uint8_t)~0x01;
    REG_USB_PHY_CTRL_91C3 &= (uint8_t)~0x20;
    REG_USB_PHY_CTRL_91C0 |= 0x01;
    REG_USB_PHY_CTRL_91C0 &= (uint8_t)~0x01;
}

/* Arm USB4 PHY link-up once at boot: run Timer0 (CC10-CC13) as a bounded-wait
 * timeout while polling E318 for PHY link-up completion. */
static void usb4_phy_arm(void) {
    REG_TIMER0_CSR = TIMER_CSR_CLEAR;
    REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
    REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | 0x04;
    REG_TIMER0_THRESHOLD_HI = 0x01;
    REG_TIMER0_THRESHOLD_LO = 0x8F;
    REG_TIMER0_CSR = TIMER_CSR_ENABLE;
    { uint16_t spin = 0;
      while (!((REG_PHY_COMPLETION_E318 & 0x10) || (REG_TIMER0_CSR & TIMER_CSR_EXPIRED)) && ++spin < 0xFFFF); }
    REG_TIMER0_CSR = TIMER_CSR_CLEAR;
    REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
}

/* CPU reset preserves USB state. */
static void usb_reinit_controller(void) {
    is_usb2 = 0;
    usb_configuration = 0;
    REG_DMA_CONFIG = DMA_CONFIG_DISABLE;
    usb_init_endpoint_state();
    // usb_init_controller
    REG_POWER_STATUS &= ~POWER_STATUS_USB_PATH;
    REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;
    REG_USB_CONFIG = USB_CONFIG_MSC_INIT;
    REG_USB_EP0_CFG = 0xF0;
    REG_USB_DATA_L = 0x00;
    REG_USB_EP_MGMT = 0x00;
    REG_BUF_CFG_9303 = 0x33;
    REG_CPU_MODE = CPU_MODE_USB3;
    REG_USB_PHY_CTRL_91C0 |= USB_PHY_91C0_INIT_TOGGLE;
    REG_USB_PHY_CTRL_91C0 &= (uint8_t)~USB_PHY_91C0_INIT_TOGGLE;
}

static void usb_fallback_to_usb2(void) {
    is_usb2 = 1;
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

static uint8_t usb_wait_ep0_dma_idle(void) {
    uint16_t timeout = 0xFFFF;
    do {
        if (REG_USB_DMA_TRIGGER == 0) return 1;
    } while (--timeout);
    return 0;
}

/* EP0 IN: send `len` bytes of DESC_BUF, or a zero-length ack. */
static void usb_send_data(uint16_t len) {
    REG_USB_EP0_LEN_H = (uint8_t)(len >> 8);
    REG_USB_EP0_LEN_L = (uint8_t)(len & 0xFF);
    REG_USB_DMA_TRIGGER = USB_DMA_SEND;
    REG_USB_CTRL_PHASE  = USB_CTRL_PHASE_DATA_IN;
}
static void usb_send_zlp(void) { usb_send_data(0); }
static void usb_stall_ep0(void) {
    REG_USB_DMA_TRIGGER = USB_DMA_STALL;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
}

static void usb_desc_copy(__code const uint8_t *src, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) DESC_BUF[i] = src[i];
}

static void usb_handle_get_descriptor(uint8_t desc_type, uint8_t desc_idx, uint16_t wlen) {
  __code const uint8_t *src;
  uint8_t desc_len;

  if (desc_type == USB_DESC_TYPE_DEVICE) {
    if (is_usb2) { src = usb_dev_desc;    desc_len = sizeof(usb_dev_desc); }
    else         { src = usb_dev_desc_ss; desc_len = sizeof(usb_dev_desc_ss); }
  } else if (desc_type == USB_DESC_TYPE_CONFIG) {
    if (is_usb2) { src = usb_cfg_desc;    desc_len = sizeof(usb_cfg_desc); }
    else         { src = usb_cfg_desc_ss; desc_len = sizeof(usb_cfg_desc_ss); }
  } else if (desc_type == USB_DESC_TYPE_BOS) {
    src = usb_bos_desc; desc_len = sizeof(usb_bos_desc);
  } else if (desc_type == USB_DESC_TYPE_STRING) {
    /* Built directly into DESC_BUF; bypass desc_copy. */
    if (desc_idx == USB_STR_IDX_LANG) {
      DESC_BUF[0] = 4; DESC_BUF[1] = 0x03;
      DESC_BUF[2] = (uint8_t)USB_LANG_ID;
      DESC_BUF[3] = USB_LANG_ID >> 8;
      desc_len = 4;
    } else if (desc_idx == USB_STR_IDX_SERIAL) {
      desc_len = usb_build_serial_desc(DESC_BUF);
    } else {
      __code const char *s;
      switch (desc_idx) {
        case USB_STR_IDX_MFG:     s = USB_STR_MFG;     break;
        case USB_STR_IDX_PRODUCT: s = USB_STR_PRODUCT; break;
        default:                  s = "";              break;
      }
      desc_len = usb_build_string_desc(s, DESC_BUF);
    }
    usb_send_data(wlen < desc_len ? wlen : desc_len);
    return;
  } else {
    usb_stall_ep0();
    return;
  }

  usb_desc_copy(src, desc_len);
  usb_send_data(wlen < desc_len ? wlen : desc_len);
}

static bool usb_handle_custom_setup(uint8_t bmReq, uint8_t bReq, uint8_t wValL, uint8_t wValH, uint16_t wLen);

static void usb_handle_setup(void) {
  uint8_t bmReq = REG_USB_SETUP_BMREQ;
  uint8_t bReq = REG_USB_SETUP_BREQ;
  uint8_t wValL = REG_USB_SETUP_WVAL_L;
  uint8_t wValH = REG_USB_SETUP_WVAL_H;
  uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

  if (usb_handle_custom_setup(bmReq, bReq, wValL, wValH, wLen)) {
    return;
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS && wValH == 0 && wLen == 0) {
    REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
    REG_USB_EP_CTRL_91D0 = 0x02;
    usb_send_zlp();
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
    usb_handle_get_descriptor(wValH, wValL, wLen);
  } else if ((bmReq == USB_SETUP_DIR_DEV_TO_HOST || bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) ||
              bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_ENDPOINT)) &&
             bReq == USB_REQ_GET_STATUS && wValL == 0 && wValH == 0 && wLen == 2) {
    DESC_BUF[0] = bmReq == USB_SETUP_DIR_DEV_TO_HOST;
    DESC_BUF[1] = 0;
    usb_send_data(2);
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_CONFIGURATION && wLen == 1) {
    DESC_BUF[0] = usb_configuration;
    usb_send_data(1);
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION && wValH == 0 && wValL <= 1 && wLen == 0) {
    usb_configuration = wValL;
    usb_send_zlp();
  } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_GET_INTERFACE && wLen == 1) {
    DESC_BUF[0] = 0;
    usb_send_data(1);
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_SET_INTERFACE &&
             wValH == 0 && wValL == 0 && wLen == 0) {
    usb_send_zlp();
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_SEL && wLen == 6) {
    // Accept and ignore the payload.
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ISOCH_DELAY && wLen == 0) {
    usb_send_zlp();
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && (bReq == USB_REQ_SET_FEATURE || bReq == USB_REQ_CLEAR_FEATURE) &&
             wValH == 0 && (wValL == 48 || wValL == 49) && wLen == 0) {
    usb_send_zlp();
  } else {
    usb_stall_ep0();
  }
}

#endif /* USB_H */
