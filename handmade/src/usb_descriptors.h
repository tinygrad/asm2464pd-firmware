#ifndef __USB_DESCRIPTORS_H__
#define __USB_DESCRIPTORS_H__

#include "types.h"
#include "flash.h"

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

#endif
