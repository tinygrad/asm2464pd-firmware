/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Control transfers for enumeration + vendor cmds (0xE4, 0xE5, 0xF0, 0xF1).
 * Bulk IN/OUT via MSC engine.
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

static volatile uint8_t is_usb3;

/*=== USB Control Transfer Helpers ===*/

static void complete_usb3_status(void) {
  while (!(REG_USB_CTRL_PHASE & USB_CTRL_PHASE_STAT_IN)) { }
  REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
}

static void send_zlp_ack(void) {
  if (is_usb3) {
    complete_usb3_status();
  } else {
    REG_USB_EP0_STATUS = 0x00;
    REG_USB_EP0_LEN_L = 0x00;
    REG_USB_DMA_TRIGGER = USB_DMA_SEND;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
  }
}

static void send_control_data(uint8_t len) {
  REG_USB_EP0_STATUS = 0x00;
  REG_USB_EP0_LEN_L = len;
  REG_USB_DMA_TRIGGER = USB_DMA_SEND;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
  if (is_usb3) complete_usb3_status();
}

/*=== USB Descriptors ===*/

static __code const uint8_t dev_desc[] = {
  0x12, 0x01, 0x20, 0x03, 0x00, 0x00, 0x00, 0x09,
  0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
  0x09, 0x02, 0x2C, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,  /* wTotalLength=44 */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,  /* bNumEndpoints=2 */
  0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,  /* EP1 IN bulk 1024 */
  0x06, 0x30, 0x00, 0x00, 0x00, 0x00,        /* SS EP companion */
  0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,  /* EP2 OUT bulk 1024 */
  0x06, 0x30, 0x00, 0x00, 0x00, 0x00,        /* SS EP companion */
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
    desc_copy(dev_desc, 18);
    if (!is_usb3) { DESC_BUF[2] = 0x10; DESC_BUF[3] = 0x02; DESC_BUF[7] = 0x40; }
    desc_len = 18;
  } else if (desc_type == USB_DESC_TYPE_CONFIG) {
    src = cfg_desc; desc_len = sizeof(cfg_desc); desc_copy(src, desc_len);
  } else if (desc_type == USB_DESC_TYPE_BOS) {
    src = bos_desc; desc_len = sizeof(bos_desc); desc_copy(src, desc_len);
  } else if (desc_type == USB_DESC_TYPE_STRING) {
    if (desc_idx == 0)      { src = str0_desc; desc_len = sizeof(str0_desc); }
    else if (desc_idx == 1) { src = str1_desc; desc_len = sizeof(str1_desc); }
    else if (desc_idx == 2) { src = str2_desc; desc_len = sizeof(str2_desc); }
    else if (desc_idx == 3) { src = str3_desc; desc_len = sizeof(str3_desc); }
    else                    { src = str_empty; desc_len = sizeof(str_empty); }
    desc_copy(src, desc_len);
  } else {
    return;
  }

  send_control_data(wlen < desc_len ? wlen : desc_len);
}

/*=== SET_ADDRESS ===*/
static void handle_set_address(uint8_t addr) {
  uint8_t tmp;
  tmp = REG_USB_INT_MASK_9090;
  REG_USB_INT_MASK_9090 = (tmp & USB_INT_MASK_GLOBAL) | (addr & 0x7F);
  REG_USB_EP_CTRL_91D0 = 0x02;

  if (is_usb3) {
    while (!(REG_USB_CTRL_PHASE & USB_CTRL_PHASE_STAT_IN)) { }
    REG_USB_ADDR_CFG_A = 0x03; REG_USB_ADDR_CFG_B = 0x03;
    REG_USB_ADDR_CFG_A = 0x07; REG_USB_ADDR_CFG_B = 0x07;
    tmp = REG_USB_ADDR_CFG_A; REG_USB_ADDR_CFG_A = tmp;
    tmp = REG_USB_ADDR_CFG_B; REG_USB_ADDR_CFG_B = tmp;
    REG_USB_ADDR_PARAM_0 = 0x00; REG_USB_ADDR_PARAM_1 = 0x0A;
    REG_USB_ADDR_PARAM_2 = 0x00; REG_USB_ADDR_PARAM_3 = 0x0A;
    tmp = REG_USB_ADDR_CTRL; REG_USB_ADDR_CTRL = tmp;
    REG_USB_EP_CTRL_9220 = 0x04;
    REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
  } else {
    send_zlp_ack();
  }
  uart_puts("[A]\n");
}

/*=== USB Control Transfer Handler ===*/

static void handle_usb_control(void) {
  uint8_t phase = REG_USB_CTRL_PHASE;

  if (phase == USB_CTRL_PHASE_DATA_OUT || phase == 0x00) {
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
    return;
  }

  if ((phase & USB_CTRL_PHASE_STAT_OUT) && !(phase & USB_CTRL_PHASE_SETUP)) {
    /* USB3 host-to-device status completion */
    REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
    return;
  } else if ((phase & USB_CTRL_PHASE_STAT_IN) && !(phase & USB_CTRL_PHASE_SETUP)) {
    REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
    return;
  } else if (phase & USB_CTRL_PHASE_SETUP) {
    uint8_t bmReq, bReq, wValL, wValH, wLenL;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
    bmReq = REG_USB_SETUP_BMREQ; bReq = REG_USB_SETUP_BREQ;
    wValL = REG_USB_SETUP_WVAL_L; wValH = REG_USB_SETUP_WVAL_H;
    wLenL = REG_USB_SETUP_WLEN_L;

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
      handle_set_address(wValL);
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
      handle_get_descriptor(wValH, wValL, wLenL);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
      REG_USB_MSC_CFG = 0x00;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
      REG_USB_INT_MASK_9090 |= USB_INT_MASK_GLOBAL;
      send_zlp_ack();
      uart_puts("[*** SET CONFIG ***]\n");
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_SET_INTERFACE) {
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_ENDPOINT) && bReq == USB_REQ_CLEAR_FEATURE) {
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
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
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t val = REG_USB_SETUP_WIDX_L;
      if (bank) DPX = bank;
      XDATA_REG8(addr) = val;
      if (bank) DPX = 0x00;
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF1) {
      /* Test packet — on USB3, OUT data arrives with SETUP */
      uart_puts("[F1 ");
      uart_puthex(DESC_BUF[0]); uart_puthex(DESC_BUF[1]);
      uart_puthex(DESC_BUF[2]); uart_puthex(DESC_BUF[3]);
      uart_puts("]\n");
      REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
      REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      REG_PCIE_FMT_TYPE = wValL;
      REG_PCIE_BYTE_EN  = wValH;
      /* Don't send ZLP — wait for DATA_OUT phase */
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      uint8_t ret_status = 0xFF;
      uint16_t t;
      for (t = 0; t < 50000; t++) {
        uint8_t s = REG_PCIE_STATUS;
        if (s & PCIE_STATUS_ERROR) {
          REG_PCIE_STATUS = PCIE_STATUS_ERROR;
          ret_status = 1;
          break;
        }
        if (s & PCIE_STATUS_COMPLETE) {
          ret_status = 0;
          break;
        }
      }
      if (ret_status == 0) {
        DESC_BUF[0] = REG_PCIE_DATA_0;
        DESC_BUF[1] = REG_PCIE_DATA_1;
        DESC_BUF[2] = REG_PCIE_DATA_2;
        DESC_BUF[3] = REG_PCIE_DATA_3;
        DESC_BUF[4] = REG_PCIE_LINK_STATUS_LO;
        DESC_BUF[5] = REG_PCIE_LINK_STATUS_HI;
        DESC_BUF[6] = REG_PCIE_COMPL_STATUS;
      } else {
        int i;
        for (i = 0; i < 7; i++) DESC_BUF[i] = 0;
      }
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
      send_zlp_ack();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_IN) {
    if (REG_USB_SETUP_BREQ == 0xF0) {
      /* PCIe TLP DATA_OUT phase */
      if (REG_PCIE_FMT_TYPE & PCIE_FMT_HAS_DATA) {
        REG_PCIE_DATA_0     = DESC_BUF[8];
        REG_PCIE_DATA_1     = DESC_BUF[9];
        REG_PCIE_DATA_2     = DESC_BUF[10];
        REG_PCIE_DATA_3     = DESC_BUF[11];
      }
      REG_PCIE_ADDR_0      = DESC_BUF[3];
      REG_PCIE_ADDR_1      = DESC_BUF[2];
      REG_PCIE_ADDR_2      = DESC_BUF[1];
      REG_PCIE_ADDR_3      = DESC_BUF[0];
      REG_PCIE_ADDR_HIGH   = DESC_BUF[7];
      REG_PCIE_ADDR_HIGH_1 = DESC_BUF[6];
      REG_PCIE_ADDR_HIGH_2 = DESC_BUF[5];
      REG_PCIE_ADDR_HIGH_3 = DESC_BUF[4];
      REG_PCIE_STATUS  = PCIE_STATUS_ERROR;
      REG_PCIE_STATUS  = PCIE_STATUS_COMPLETE;
      REG_PCIE_STATUS  = PCIE_STATUS_KICK;
      REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
      send_zlp_ack();
    }
    if (REG_USB_SETUP_BREQ == 0xF1) {
      uart_puts("[F1 ");
      uart_puthex(DESC_BUF[0]);
      uart_puthex(DESC_BUF[1]);
      uart_puthex(DESC_BUF[2]);
      uart_puthex(DESC_BUF[3]);
      uart_puts("]\n");
      send_zlp_ack();
    }
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
  } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
  }
}

/*=== Bulk Transfer Handler ===*/

void handle_usb_bulk_data(void) {
  uint8_t bulk_cfg1 = REG_USB_EP_CFG1;
  if (bulk_cfg1 & USB_EP_CFG1_BULK_OUT_COMPLETE) {
    REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
  } else if (bulk_cfg1 & USB_EP_CFG1_BULK_IN_COMPLETE) {
    REG_USB_MSC_LENGTH = 0xd;
    REG_USB_BULK_DMA_TRIGGER = 0x1;
  } else if (!(bulk_cfg1 & (USB_EP_CFG1_BULK_OUT_START | USB_EP_CFG1_BULK_IN_START))) {
    return;
  }
  REG_USB_EP_CFG1 = bulk_cfg1;
}

/*=== Interrupt Handlers ===*/

void int0_isr(void) __interrupt(0) {
  uint8_t periph_status = REG_USB_PERIPH_STATUS;

  if (periph_status & USB_PERIPH_CONTROL) {
    handle_usb_control();
  } else if (periph_status & USB_PERIPH_BULK_DATA) {
    handle_usb_bulk_data();
  }
}

void timer0_isr(void) __interrupt(1) { }

void int1_isr(void) __interrupt(2) {
  uint8_t tmp = REG_POWER_EVENT_92E1;
  if (tmp) {
    REG_POWER_EVENT_92E1 = tmp;
    REG_POWER_STATUS &= ~(POWER_STATUS_USB_PATH | 0x80);
  }
}

void timer1_isr(void) __interrupt(3) { }
void serial_isr(void) __interrupt(4) { }
void timer2_isr(void) __interrupt(5) { }

/*=== Hardware Init ===*/
static void hw_init(void) {
  REG_INT_ENABLE = INT_ENABLE_SYSTEM;
  REG_INT_STATUS_C800 = INT_STATUS_PCIE | POWER_ENABLE_BIT;
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;
  REG_USB_EP0_LEN_H = 0xF0;
  REG_CPU_DMA_CTRL_CC90 = 0x05;
  REG_CPU_DMA_DATA_LO = 0x00;
  REG_CPU_DMA_DATA_HI = 0xC8;
  REG_CPU_DMA_INT = XFER_DMA_CMD_START;
}

void main(void) {
  IE = 0;
  is_usb3 = 0;
  REG_UART_LCR &= ~LCR_PARITY_MASK;
  uart_puts("\n[BOOT]\n");

  hw_init();

  uint8_t link = REG_USB_LINK_STATUS;
  is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
  uart_puts("[link="); uart_puthex(link); uart_puts("]\n");

  uart_puts("[GO]\n");
  TCON = 0x04;
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  while (1) {
    REG_CPU_KEEPALIVE = 0x0C;
  }
}
