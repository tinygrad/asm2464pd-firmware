#include "types.h"
#include "registers.h"
#include "flash.h"

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)

/*=== USB device identification ===*/
#define USB_VID                 0xADD1
#define USB_PID                 0x0001
#define USB_BCD_DEVICE          0x0001
#define USB_LANG_ID             0x0409   /* US English */

/* String descriptors */
#define USB_STR_MFG             "tiny"
#define USB_STR_PRODUCT         "custom v0.1"

#define USB_STR_IDX_LANG        0
#define USB_STR_IDX_MFG         1
#define USB_STR_IDX_PRODUCT     2
#define USB_STR_IDX_SERIAL      3

/*=== Helpers ===*/
#define U16_LE(v)               ((v) & 0xFF), (((v) >> 8) & 0xFF)

/*=== Device descriptors ===*/

static __code const uint8_t usb_dev_desc[] = {
  0x12, 0x01,                 /* bLength=18, bDescriptorType=DEVICE */
  U16_LE(0x0200),             /* bcdUSB = 2.00 */
  0x00, 0x00, 0x00,           /* bDeviceClass / SubClass / Protocol */
  0x40,                       /* bMaxPacketSize0 = 64 */
  U16_LE(USB_VID), U16_LE(USB_PID), U16_LE(USB_BCD_DEVICE),
  USB_STR_IDX_MFG, USB_STR_IDX_PRODUCT, USB_STR_IDX_SERIAL,
  0x01,                       /* bNumConfigurations */
};

static __code const uint8_t usb_dev_desc_ss[] = {
  0x12, 0x01,                 /* bLength=18, bDescriptorType=DEVICE */
  U16_LE(0x0320),             /* bcdUSB = 3.20 */
  0x00, 0x00, 0x00,           /* bDeviceClass / SubClass / Protocol */
  0x09,                       /* bMaxPacketSize0 = 2^9 = 512 (SuperSpeed) */
  U16_LE(USB_VID), U16_LE(USB_PID), U16_LE(USB_BCD_DEVICE),
  USB_STR_IDX_MFG, USB_STR_IDX_PRODUCT, USB_STR_IDX_SERIAL,
  0x01,                       /* bNumConfigurations */
};

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
    rmw(base + 0x02, 0x1F, 0xA0); rmw(base + 0x03, 0xF3, 0x00);
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

static void usb_init_controller(uint8_t force_usb2) {
    REG_POWER_STATUS &= ~POWER_STATUS_USB_PATH;
    REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;
    REG_USB_CONFIG = USB_CONFIG_MSC_INIT;
    REG_USB_EP0_CFG = 0xF0;
    REG_USB_DATA_L = 0x00;
    REG_USB_EP_MGMT = 0x00;
    REG_BUF_CFG_9303 = 0x33;
    if (force_usb2) {
        REG_CPU_MODE = CPU_MODE_USB2;
        REG_USB_PHY_CTRL_91C0 = 0x10;
    }
}

/* EP0 IN: send `len` bytes of DESC_BUF, or a zero-length ack. */
static void usb_send_data(uint16_t len) {
    REG_USB_EP0_LEN_H = (uint8_t)(len >> 8);
    REG_USB_EP0_LEN_L = (uint8_t)(len & 0xFF);
    REG_USB_DMA_TRIGGER = USB_DMA_SEND;
    REG_USB_CTRL_PHASE  = USB_CTRL_PHASE_DATA_IN;
}
static void usb_send_zlp(void) { usb_send_data(0); }

static void usb_desc_copy(__code const uint8_t *src, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) DESC_BUF[i] = src[i];
}

static void usb_handle_set_address(uint8_t wValL) {
    REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
    REG_USB_EP_CTRL_91D0  = 0x02;
    usb_send_zlp();
}

static void usb_handle_get_descriptor(uint8_t is_usb2, uint8_t desc_type,
                                      uint8_t desc_idx, uint16_t wlen) {
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
      DESC_BUF[2] = USB_LANG_ID & 0xFF;
      DESC_BUF[3] = (USB_LANG_ID >> 8) & 0xFF;
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
    return;
  }

  usb_desc_copy(src, desc_len);
  usb_send_data(wlen < desc_len ? wlen : desc_len);
}
