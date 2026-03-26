/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "globals.h"

__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;
#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)
#define EP_BUF(n) XDATA_REG8(0xD800 + (n))

static void desc_copy(__code const uint8_t *src, uint8_t len) {
  uint8_t i;
  for (i = 0; i < len; i++) DESC_BUF[i] = src[i];
}

void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

/*=== USB Control Transfer Helpers ===*/

static void send_control_data(uint8_t len) {
  REG_USB_EP0_STATUS = 0x00;
  REG_USB_EP0_LEN_L = len;
  REG_USB_DMA_TRIGGER = USB_DMA_SEND;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
}
static void send_zlp_ack(void) { send_control_data(0); }

/*=== USB Request Handlers ===*/

/* USB 2.0 Descriptors — no SS companion descriptors, 64-byte bulk EPs for Full Speed */
static __code const uint8_t dev_desc[] = {
  0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
  0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
  0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
  0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
};
static __code const uint8_t bos_desc[] = {
  0x05, 0x0F, 0x16, 0x00, 0x02,
  0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
  0x0A, 0x10, 0x03, 0x00, 0x0E, 0x00, 0x03, 0x00, 0x00, 0x00,
};
static __code const uint8_t str0_desc[] = { 0x04, 0x03, 0x09, 0x04 };
static __code const uint8_t str1_desc[] = { 0x0A, 0x03, 't',0, 'i',0, 'n',0, 'y',0 };
static __code const uint8_t str2_desc[] = { 0x08, 0x03, 'u',0, 's',0, 'b',0 };
static __code const uint8_t str3_desc[] = { 0x08, 0x03, '0',0, '0',0, '1',0 };
static __code const uint8_t str_empty[] = { 0x02, 0x03 };

static void handle_get_descriptor(uint8_t desc_type, uint8_t desc_idx, uint8_t wlen) {
  __code const uint8_t *src;
  uint8_t desc_len;

  if (desc_type == USB_DESC_TYPE_DEVICE) {
    src = dev_desc;
    desc_len = 18;
  } else if (desc_type == USB_DESC_TYPE_CONFIG) {
    src = cfg_desc;
    desc_len = sizeof(cfg_desc);
  } else if (desc_type == USB_DESC_TYPE_BOS) {
    src = bos_desc;
    desc_len = sizeof(bos_desc);
  } else if (desc_type == USB_DESC_TYPE_STRING) {
    if (desc_idx == 0)      { src = str0_desc; desc_len = sizeof(str0_desc); }
    else if (desc_idx == 1) { src = str1_desc; desc_len = sizeof(str1_desc); }
    else if (desc_idx == 2) { src = str2_desc; desc_len = sizeof(str2_desc); }
    else if (desc_idx == 3) { src = str3_desc; desc_len = sizeof(str3_desc); }
    else                    { src = str_empty; desc_len = sizeof(str_empty); }
  } else {
    return;
  }

  desc_copy(src, desc_len);
  send_control_data(wlen < desc_len ? wlen : desc_len);
}

static void handle_usb_control(void) {
  uint8_t phase;
  phase = REG_USB_CTRL_PHASE;
  if (phase & USB_CTRL_PHASE_SETUP) {
    uint8_t bmReq, bReq, wValL, wValH, wLenL;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
    bmReq = REG_USB_SETUP_BMREQ; bReq = REG_USB_SETUP_BREQ;
    wValL = REG_USB_SETUP_WVAL_L; wValH = REG_USB_SETUP_WVAL_H;
    wLenL = REG_USB_SETUP_WLEN_L;

    uart_puts("[C ");
    uart_puthex(bReq);
    uart_puts("]\n");

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
      // the USB_INT_MASK_GLOBAL enabled bulk mode, this makes it not get -1
      REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
      // does set address
      REG_USB_EP_CTRL_91D0 = 0x02;
      send_zlp_ack();
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
      handle_get_descriptor(wValH, wValL, wLenL);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
      // enable USB bulk mode
      REG_USB_MSC_CFG = 0x00;

      // arm bulk endpoint (IN+OUT)
      REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_IN | USB_EP_CFG2_ARM_OUT;

      // set length of IN to 13 (this def has to be in interrupt)
      REG_USB_MSC_LENGTH = 0xd;
      REG_USB_BULK_DMA_TRIGGER = 0x1;

      send_zlp_ack();
      uart_puts("[SET CONFIG]\n");
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_SET_INTERFACE) {
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
      /* Vendor read XDATA via control */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t vi;
      for (vi = 0; vi < wLenL; vi++) DESC_BUF[vi] = XDATA_REG8(addr + vi);
      send_control_data(wLenL);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
      /* Vendor write XDATA via control */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      XDATA_REG8(addr) = REG_USB_SETUP_WIDX_L;
      send_zlp_ack();
    } else {
      send_zlp_ack();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
  } else {
    uart_puts("[UNHANDLED CONTROL ");
    uart_puthex(phase);
    uart_puts("]\n");
  }
}

void int0_isr(void) __interrupt(0) {
  uint8_t periph_status;
  periph_status = REG_USB_PERIPH_STATUS;

  if (periph_status & USB_PERIPH_BUS_RESET) {
    uart_puts("[UNHANDLED RESET]\n");
  } else if (periph_status & USB_PERIPH_CONTROL) {
    handle_usb_control();
  } else {
    uart_puts("[int0] ");
    uart_puthex(periph_status);
    uart_puts("\n");
  }
}

void int1_isr(void) __interrupt(1) {
  uart_puts("[int1]\n");
}

void main(void) {
  // without this, UART has parity
  REG_UART_LCR &= ~LCR_PARITY_MASK;
  uart_puts("\n[BOOT]\n");

  // without this, USB2 is flaky
  REG_CPU_MODE = CPU_MODE_USB2;

  // without this, it doesn't get an interrupt
  REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;

  // this enables a lot of int1
  //REG_INT_ENABLE = 0xff;

  // without this, no USB interrupts
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;

  // enable USB high speed mode
  REG_USB_PHY_CTRL_91C0 = 0x10;

  uart_puts("[GO]\n");

  // enable interrupts and chill
  TCON = 0x04;  /* IT0=0 (level-triggered INT0) */
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;
  while (1) {
    /*uint8_t t;
    t = REG_USB_EP_CFG1;
    if (t) {
      uart_puts("REG_USB_EP_CFG1: ");
      uart_puthex(REG_USB_EP_CFG1);
      uart_puts("\n");
    }*/
    /*uart_puthex(REG_USB_EP_CFG1);
    uart_puts(" ");
    uart_puthex(REG_USB_EP_CFG2);
    uart_puts("\n");
    volatile int i;
    for (int i=0; i < 10000; i++);*/
  }
}
