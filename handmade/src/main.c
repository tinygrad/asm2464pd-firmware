/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "globals.h"

__sfr __at(0x93) DPX;   /* DPTR bank select — DPX=1 accesses internal PHY regs */
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
  0x09, 0x02, 0x2E, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,  /* wTotalLength=46 */
  0x09, 0x04, 0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0x00,  /* bNumEndpoints=4 */
  0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,  /* EP1 IN bulk 64 */
  0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,  /* EP2 OUT bulk 64 */
  0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,  /* EP3 IN bulk 64 */
  0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,  /* EP4 OUT bulk 64 */
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

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
      // the USB_INT_MASK_GLOBAL enabled bulk mode, this makes it not get -1
      REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
      // does set address
      REG_USB_EP_CTRL_91D0 = 0x02;
      send_zlp_ack();
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
      handle_get_descriptor(wValH, wValL, wLenL);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
      // enable USB bulk mode (bypass MSC)
      REG_USB_MSC_CFG = 0x00;
      // enable bulk endpoint (without the clear in, it'll get a spurious IN, without the clear out, it'll miss an out)
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
      send_zlp_ack();
      uart_puts("[*** SET CONFIG ***]\n");
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_SET_INTERFACE) {
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
      /* Vendor read XDATA via control.  wValue=addr, wLength=size.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t vi;
      for (vi = 0; vi < wLenL; vi++) {
        if (bank) DPX = bank;
        uint8_t val = XDATA_REG8(addr + vi);
        if (bank) DPX = 0x00;
        DESC_BUF[vi] = val;
      }
      send_control_data(wLenL);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
      /* Vendor write XDATA via control.  wValue=addr, wIndex low=val.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t val = REG_USB_SETUP_WIDX_L;
      if (bank) DPX = bank;
      XDATA_REG8(addr) = val;
      if (bank) DPX = 0x00;
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* PCIe TLP request — OUT phase: set fmt_type + byte_enable from wValue.
       * DATA_OUT will carry the address and value.
       * wValue = fmt_type | (byte_enable << 8) */
      REG_PCIE_FMT_TYPE = wValL;
      REG_PCIE_BYTE_EN  = wValH;
      /* Don't send ZLP — wait for DATA_OUT phase */
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* PCIe TLP result — IN phase: poll B296 on-chip, return completion data.
       * The OUT phase already cleared, triggered, and armed B296. */
      uint8_t ret_status = 0xFF; // timeout fallthrough
      uint16_t t;
      for (t = 0; t < 50000; t++) {
        uint8_t s = REG_PCIE_STATUS;
        if (s & PCIE_STATUS_COMPLETE) {
          ret_status = 0;
          break;
        }
        if (s & PCIE_STATUS_ERROR) {
          REG_PCIE_STATUS = PCIE_STATUS_ERROR;
          ret_status = 1;
          break;
        }
      }
      DESC_BUF[0] = REG_PCIE_DATA_0;
      DESC_BUF[1] = REG_PCIE_DATA_1;
      DESC_BUF[2] = REG_PCIE_DATA_2;
      DESC_BUF[3] = REG_PCIE_DATA_3;
      DESC_BUF[4] = REG_PCIE_LINK_STATUS_8;
      DESC_BUF[5] = REG_PCIE_CPL_STATUS;
      DESC_BUF[6] = REG_PCIE_COMPL_STATUS;
      DESC_BUF[7] = ret_status;
      send_control_data(8);
    } else {
      uart_puts("[C ");
      uart_puthex(bmReq);
      uart_puts(" ");
      uart_puthex(bReq);
      uart_puts(" ");
      uart_puthex(wLenL);
      uart_puts("]\n");
      if (wLenL == 0) send_zlp_ack();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_IN) {
    if (REG_USB_SETUP_BREQ == 0xF0) {
      /* PCIe TLP DATA_OUT: 12 bytes at DESC_BUF (0x9E00).
       *   [0-3]  address low, little-endian
       *   [4-7]  address high, little-endian
       *   [8-11] value, big-endian (writes only)
       * fmt_type/byte_enable already written to B210/B217 in SETUP phase. */

      /* Write value to B220-B223 if write request (data payload present) */
      if (REG_PCIE_FMT_TYPE & PCIE_FMT_HAS_DATA) {
        REG_PCIE_DATA_0     = DESC_BUF[8];
        REG_PCIE_DATA_1     = DESC_BUF[9];
        REG_PCIE_DATA_2     = DESC_BUF[10];
        REG_PCIE_DATA_3     = DESC_BUF[11];
      }
      /* Address: LE to BE swap into B218-B21F */
      REG_PCIE_ADDR_0      = DESC_BUF[3];
      REG_PCIE_ADDR_1      = DESC_BUF[2];
      REG_PCIE_ADDR_2      = DESC_BUF[1];
      REG_PCIE_ADDR_3      = DESC_BUF[0];
      REG_PCIE_ADDR_HIGH   = DESC_BUF[7];
      REG_PCIE_ADDR_HIGH_1 = DESC_BUF[6];
      REG_PCIE_ADDR_HIGH_2 = DESC_BUF[5];
      REG_PCIE_ADDR_HIGH_3 = DESC_BUF[4];
      /* Stock sequence: clear error, clear completion, arm, then trigger */
      REG_PCIE_STATUS  = PCIE_STATUS_ERROR;
      REG_PCIE_STATUS  = PCIE_STATUS_COMPLETE;
      REG_PCIE_STATUS  = PCIE_STATUS_KICK;
      REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
      // TODO: there's no while loop for wait on the write path. can this happen too fast?
      send_zlp_ack();
    }
    if (REG_USB_SETUP_BREQ == 0xF1) {
      // test packet
      uart_puts("[F1 ");
      uart_puthex(DESC_BUF[0]);
      uart_puthex(DESC_BUF[1]);
      uart_puthex(DESC_BUF[2]);
      uart_puthex(DESC_BUF[3]);
      uart_puts("]");
      uart_puts("\n");
      send_zlp_ack();
    }
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
  } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
  } else {
    uart_puts("[UNHANDLED CONTROL ");
    uart_puthex(phase);
    uart_puts("]\n");
  }
}

void handle_usb_bulk_data(void) {
  uint8_t bulk_cfg1, bulk_cfg2;
  bulk_cfg1 = REG_USB_EP_CFG1;
  bulk_cfg2 = REG_USB_EP_CFG2;
  uart_puts("[BULK ");
  uart_puthex(bulk_cfg1); uart_puts(" "); uart_puthex(bulk_cfg2);
  uart_puts("]\n");
  if (bulk_cfg1 & USB_EP_CFG1_BULK_OUT_COMPLETE) {
    // dump what's at 0x7000
    uart_puts("[7000=");
    uart_puthex(XDATA_REG8(0x7000)); uart_puthex(XDATA_REG8(0x7001));
    uart_puthex(XDATA_REG8(0x7002)); uart_puthex(XDATA_REG8(0x7003));
    uart_puts("]\n");
    // handshake DMA
    //REG_BULK_DMA_HANDSHAKE = 1;
    // re-arm OUT
    REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
  } else if (bulk_cfg1 & USB_EP_CFG1_BULK_IN_COMPLETE) {
    // bulk in needed — send data from D800
    REG_USB_MSC_LENGTH = 0xd;
    REG_USB_BULK_DMA_TRIGGER = 0x1;
  } else if (bulk_cfg1 & USB_EP_CFG1_BULK_OUT_START) {
    // ack
  } else if (bulk_cfg1 & USB_EP_CFG1_BULK_IN_START) {
    // ack
  } else {
    // don't ack
    return;
  }
  // ack
  REG_USB_EP_CFG1 = bulk_cfg1;
}

void int0_isr(void) __interrupt(0) {
  uint8_t periph_status;
  periph_status = REG_USB_PERIPH_STATUS;

  if (periph_status & USB_PERIPH_BUS_RESET) {
    uart_puts("[UNHANDLED RESET]\n");
  } else if (periph_status & USB_PERIPH_CONTROL) {
    handle_usb_control();
  } else if (periph_status & USB_PERIPH_BULK_DATA) {
    handle_usb_bulk_data();
  } else if (periph_status & USB_PERIPH_EP_COMPLETE) {
    uint8_t ep = REG_USB_EP_READY;
    uart_puts("[EP_COMPLETE "); uart_puthex(ep); uart_puts(" "); uart_puthex(REG_USB_EP_STATUS_90E3); uart_puts("]\n");
    REG_USB_EP_READY = ep;
  } else {
    uart_puts("[UNHANDLED INT0 ");
    uart_puthex(periph_status);
    uart_puts("]\n");
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

  // without this, no USB interrupts
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;

  // enable USB high speed mode
  REG_USB_PHY_CTRL_91C0 = 0x10;

  // enable BULK interrupt. mislabeled
  REG_USB_EP0_LEN_H = 0xF0;

  // enables EP_COMPLETE interrupts
  REG_USB_DATA_L = 0x00;

  uart_puts("[GO]\n");

  // enable interrupts and chill
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  while (1) {
    // DO NOT PUT ANYTHING HERE, EVERYTHING SHOULD BE HANDLED IN INTERRUPTS
  }
}
