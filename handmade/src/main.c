/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "globals.h"

//#define USB3
static uint8_t is_usb3;
static uint8_t pcie_link_up;

/* Streaming PCIe DMA state — configured via 0xF4 control message */
static uint8_t dma_mode;       /* 0=idle, 1=write, 2=read */
static uint8_t dma_count;      /* dwords per bulk packet */
static uint8_t dma_addr_0;     /* shadow of ADDR_3 (addr[7:0]) — written BEFORE trigger */
static uint8_t dma_addr_1;     /* shadow of ADDR_2 (addr[15:8]) */
static uint8_t dma_addr_2;     /* shadow of ADDR_1 (addr[23:16]) */

static void pcie_read_to_d800(void);
static void bulk_in_trigger(void);

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
static __code const uint8_t dev_desc_30[] = {
  0x12, 0x01, 0x20, 0x03, 0x00, 0x00, 0x00, 0x09,
  0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
  0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,  /* wTotalLength=32 */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,  /* bNumEndpoints=2 */
  0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,  /* EP1 IN bulk 64 */
  0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,  /* EP2 OUT bulk 64 */
};
static __code const uint8_t cfg_desc_30[] = {
  0x09, 0x02, 0x2C, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,  /* wTotalLength=44 */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,  /* bNumEndpoints=2 */
  0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,  /* EP1 IN bulk 1024 */
  0x06, 0x30, 0x00, 0x00, 0x00, 0x00,         /* SS EP Companion */
  0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,  /* EP2 OUT bulk 1024 */
  0x06, 0x30, 0x00, 0x00, 0x00, 0x00,         /* SS EP Companion */
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
    if (is_usb3) {
      src = dev_desc_30;
      desc_len = sizeof(dev_desc_30);
    } else {
      src = dev_desc;
      desc_len = sizeof(dev_desc);
    }
  } else if (desc_type == USB_DESC_TYPE_CONFIG) {
    if (is_usb3) {
      src = cfg_desc_30;
      desc_len = sizeof(cfg_desc_30);
    } else {
      src = cfg_desc;
      desc_len = sizeof(cfg_desc);
    }
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

    if (!(bmReq & USB_SETUP_TYPE_VENDOR)) {
      uart_puts("[C ");
      uart_puthex(bmReq);
      uart_puts(" ");
      uart_puthex(bReq);
      uart_puts(" ");
      uart_puthex(wLenL);
      uart_puts("]\n");
    }

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
      // the USB_INT_MASK_GLOBAL enabled bulk mode, this makes it not get -1
      REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL | (wValL & 0x7F);
      // does set address
      REG_USB_EP_CTRL_91D0 = 0x02;
      send_zlp_ack();
      uart_puts("[A]\n");
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
        DESC_BUF[4] = REG_PCIE_CPL_HDR_HI;
        DESC_BUF[5] = REG_PCIE_CPL_HDR_LO;
        DESC_BUF[6] = REG_PCIE_COMPL_STATUS;
      } else {
        int i;
        for (i = 0; i < 7; i++) DESC_BUF[i] = 0;
      }
      DESC_BUF[7] = ret_status;
      send_control_data(8);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF4) {
      /* Configure streaming DMA: wValue=addr[15:0]
       * wIndex low[1:0] = mode (1=write, 2=read, 0=stop)
       * wIndex low[7:2] = dwords per chunk (0 = 128 for writes)
       * wIndex high = addr[39:32] */
      {
        uint8_t widx_l = REG_USB_SETUP_WIDX_L;
        dma_mode = widx_l & 0x03;
        dma_count = widx_l >> 2;
        if (dma_count == 0) dma_count = 128;
      }
      dma_addr_0 = wValL & 0xFC;
      dma_addr_1 = wValH;
      dma_addr_2 = 0;
      /* Pre-configure PCIe address registers */
      REG_PCIE_FMT_TYPE = (dma_mode == 1) ? PCIE_FMT_MEM_WRITE64 : PCIE_FMT_MEM_READ64;
      REG_PCIE_BYTE_EN  = 0x0F;
      REG_PCIE_ADDR_HIGH   = 0;
      REG_PCIE_ADDR_HIGH_1 = 0;
      REG_PCIE_ADDR_HIGH_2 = 0;
      REG_PCIE_ADDR_HIGH_3 = REG_USB_SETUP_WIDX_H;
      REG_PCIE_ADDR_0 = 0;
      REG_PCIE_ADDR_1 = dma_addr_2;
      REG_PCIE_ADDR_2 = dma_addr_1;

      send_zlp_ack();

      if (dma_mode == 2) {
        /* Pre-read first chunk and send it */
        pcie_read_to_d800();
        bulk_in_trigger();
      }
    } else {
      if (wLenL == 0) send_zlp_ack();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_IN || phase & USB_CTRL_PHASE_STAT_IN) {
    // USB_CTRL_PHASE_DATA_IN on USB 2.0, USB_CTRL_PHASE_STAT_IN on USB 3.0
    if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
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
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
  } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
  } else {
    uart_puts("[UNHANDLED CONTROL ");
    uart_puthex(phase);
    uart_puts("]\n");
  }
}

static inline void dma_addr_inc(void) {
  dma_addr_0 += 4;
  if (dma_addr_0 == 0) {
    dma_addr_1++;
    REG_PCIE_ADDR_2 = dma_addr_1;
    if (dma_addr_1 == 0) {
      dma_addr_2++;
      REG_PCIE_ADDR_1 = dma_addr_2;
    }
  }
}

static void bulk_write_packet(void) {
  __xdata uint8_t *src = (__xdata uint8_t *)0x7000;
  uint8_t ci;
  for (ci = 0; ci < 128; ci++) {
    REG_PCIE_DATA_0 = *src++;
    REG_PCIE_DATA_1 = *src++;
    REG_PCIE_DATA_2 = *src++;
    REG_PCIE_DATA_3 = *src++;
    REG_PCIE_ADDR_3 = dma_addr_0;
    REG_PCIE_STATUS  = PCIE_STATUS_ERROR | PCIE_STATUS_COMPLETE | PCIE_STATUS_KICK;
    REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
    dma_addr_inc();
  }
}

/* Read dma_count dwords from PCIe into D800 */
static void pcie_read_to_d800(void) {
  __xdata uint8_t *dst = (__xdata uint8_t *)0xD800;
  uint8_t ci;
  for (ci = 0; ci < dma_count; ci++) {
    REG_PCIE_ADDR_3 = dma_addr_0;
    REG_PCIE_STATUS  = PCIE_STATUS_ERROR | PCIE_STATUS_COMPLETE | PCIE_STATUS_KICK;
    REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
    while (!(REG_PCIE_STATUS & (PCIE_STATUS_ERROR | PCIE_STATUS_COMPLETE)));
    *dst++ = REG_PCIE_DATA_0;
    *dst++ = REG_PCIE_DATA_1;
    *dst++ = REG_PCIE_DATA_2;
    *dst++ = REG_PCIE_DATA_3;
    dma_addr_inc();
  }
}

/* Send D800 contents via bulk IN */
static void bulk_in_trigger(void) {
  REG_USB_MSC_LENGTH = (uint8_t)(dma_count * 4);
  REG_USB_BULK_DMA_TRIGGER = 0x01;
}

void handle_usb_bulk_data(void) {
  uint8_t bulk_cfg1;
  bulk_cfg1 = REG_USB_EP_CFG1;
  if (bulk_cfg1 & USB_EP_CFG1_BULK_OUT_COMPLETE) {
    if (dma_mode == 1) {
      bulk_write_packet();
    }
    REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
  }
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
    REG_USB_EP_STATUS_90E3 = 0x02;
    REG_USB_EP_READY = 0x01;
    /* Bulk IN completed — chain next read+send if streaming */
    if (dma_mode == 2) {
      pcie_read_to_d800();
      bulk_in_trigger();
    }
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

  #ifndef USB3
    // without this, USB2 is flaky
    REG_CPU_MODE = CPU_MODE_USB2;

    // enable USB high speed mode
    REG_USB_PHY_CTRL_91C0 = 0x10;
  #endif

  // clear this to get USB3 interrupts
  REG_POWER_STATUS &= ~POWER_STATUS_USB_PATH;

  // without this, it doesn't get an interrupt
  REG_INT_STATUS_C800 = INT_STATUS_GLOBAL;

  // without this, no USB interrupts
  REG_USB_CONFIG = USB_CONFIG_MSC_INIT;

  // enable BULK interrupt. mislabeled
  REG_USB_EP0_LEN_H = 0xF0;

  // enables EP_COMPLETE interrupts
  REG_USB_DATA_L = 0x00;

  // PCIe TLP engine values that don't change
  REG_PCIE_TLP_CTRL   = 0x01;
  REG_PCIE_TLP_LENGTH = 0x20;

  // PCIe bringup
  REG_TUNNEL_CTRL_B403 = 0x01;           // fix PCIe link stability
  REG_PCIE_PERST_CTRL  = 0x01;           // assert PERST#
  REG_TUNNEL_LINK_STATE = 0x00;          // clear tunnel link state
  DPX = 0x01; XDATA_REG8(0x6025) = 0x80; DPX = 0x00;  // TLP routing enable
  REG_HDDPC_CTRL |= 0x20;                // enable 3.3V
  REG_PCIE_LANE_CTRL_C659 |= 0x01;       // enable 12V
  REG_PHY_TIMER_CTRL_E764 = 0x1C;        // start link training

  uint8_t link = REG_USB_LINK_STATUS;
  is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
  uart_puts("[GO link="); uart_puthex(link); uart_puts("]\n");

  // enable interrupts and chill
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  // wait for PCIe
  while (REG_PCIE_LTSSM_STATE != 0x78);
  REG_PCIE_PERST_CTRL = 0x00; // deassert PERST#
  pcie_link_up = 1;
  uart_puts("[PCIe up]\n");

  while (1) {
    // DO NOT PUT ANYTHING HERE, EVERYTHING SHOULD BE HANDLED IN INTERRUPTS
  }
}
