/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"

void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

static uint8_t is_usb2;
static uint8_t pcie_link_up;

/* Streaming PCIe DMA state — configured via 0xF0 control message */
static uint8_t dma_mode;       /* 0=idle, 1=write, 2=read */
static int32_t dma_dwords;     /* total dwords remaining for streaming read */


#include "pcie_pio.h"

static void do_usb_bulk_in(void) {
  uint16_t max_dwords = is_usb2 ? (512/4) : (1024/4);
  uint16_t chunk = (dma_dwords > max_dwords) ? max_dwords : (uint16_t)dma_dwords;
  pcie_read_chunk((__xdata uint8_t *)0x8000, chunk);
  uint16_t nbytes = chunk * 4;
  REG_USB_BULK_IN_LEN_H = nbytes >> 8;
  REG_USB_BULK_IN_LEN_L = nbytes & 0xFF;
  dma_dwords -= chunk;
  REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_IN;
}

__sfr __at(0x93) DPX;   /* DPTR bank select — DPX=1 accesses internal PHY regs */
__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;
#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)

static void desc_copy(__code const uint8_t *src, uint8_t len) {
  uint8_t i;
  for (i = 0; i < len; i++) DESC_BUF[i] = src[i];
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
  /* Config: wTotalLength=46, 1 interface */
  0x09, 0x02, 0x2E, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
  /* Interface 0: 4 bulk EPs, vendor class */
  0x09, 0x04, 0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, 0x00, 0x02, 0x00,  /* EP1 IN  bulk 512 */
  0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00,  /* EP2 OUT bulk 512 */
  0x07, 0x05, 0x83, 0x02, 0x00, 0x02, 0x00,  /* EP3 IN  bulk 512 */
  0x07, 0x05, 0x04, 0x02, 0x00, 0x02, 0x00,  /* EP4 OUT bulk 512 */
};
static __code const uint8_t cfg_desc_30[] = {
  /* Config: wTotalLength=121, 1 interface */
  0x09, 0x02, 0x79, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
  /* Alt 0: BBB — 2 bulk EPs + SS companions */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,  /* EP1 IN  bulk 1024 */
  0x06, 0x30, 0x0F, 0x00, 0x00, 0x00,         /* SS Companion: bMaxBurst=15 */
  0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,  /* EP2 OUT bulk 1024 */
  0x06, 0x30, 0x0F, 0x00, 0x00, 0x00,         /* SS Companion: bMaxBurst=15 */
  /* Alt 1: UAS — 4 bulk EPs + SS companions + pipe usage, vendor class */
  0x09, 0x04, 0x00, 0x01, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
  0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,  /* EP1 IN  bulk 1024 — Status */
  0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,         /* SS Companion: bMaxBurst=15, MaxStreams=32 */
  0x04, 0x24, 0x03, 0x00,                     /* Pipe Usage: Status */
  0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,  /* EP2 OUT bulk 1024 — Command */
  0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,         /* SS Companion: bMaxBurst=15, MaxStreams=32 */
  0x04, 0x24, 0x04, 0x00,                     /* Pipe Usage: Command */
  0x07, 0x05, 0x83, 0x02, 0x00, 0x04, 0x00,  /* EP3 IN  bulk 1024 — Data-In */
  0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,         /* SS Companion: bMaxBurst=15, MaxStreams=32 */
  0x04, 0x24, 0x02, 0x00,                     /* Pipe Usage: Data-In */
  0x07, 0x05, 0x04, 0x02, 0x00, 0x04, 0x00,  /* EP4 OUT bulk 1024 — Data-Out */
  0x06, 0x30, 0x00, 0x00, 0x00, 0x00,         /* SS Companion: bMaxBurst=0 */
  0x04, 0x24, 0x01, 0x00,                     /* Pipe Usage: Data-Out */
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

static void handle_get_descriptor(uint8_t desc_type, uint8_t desc_idx, uint16_t wlen) {
  __code const uint8_t *src;
  uint8_t desc_len;

  if (desc_type == USB_DESC_TYPE_DEVICE) {
    if (is_usb2) {
      src = dev_desc;
      desc_len = sizeof(dev_desc);
    } else {
      src = dev_desc_30;
      desc_len = sizeof(dev_desc_30);
    }
  } else if (desc_type == USB_DESC_TYPE_CONFIG) {
    if (is_usb2) {
      src = cfg_desc;
      desc_len = sizeof(cfg_desc);
    } else {
      src = cfg_desc_30;
      desc_len = sizeof(cfg_desc_30);
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

/*=== USB Control Handler ===*/

static void handle_usb_control(void) {
  uint8_t phase;
  phase = REG_USB_CTRL_PHASE;
  if (phase & USB_CTRL_PHASE_SETUP) {
    uint8_t bmReq, bReq, wValL, wValH;
    uint16_t wLen;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
    bmReq = REG_USB_SETUP_BMREQ; bReq = REG_USB_SETUP_BREQ;
    wValL = REG_USB_SETUP_WVAL_L; wValH = REG_USB_SETUP_WVAL_H;
    wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

    if (!(bmReq & USB_SETUP_TYPE_VENDOR)) {
      uart_puts("[C ");
      uart_puthex(bmReq);
      uart_puts(" ");
      uart_puthex(bReq);
      uart_puts(" ");
      uart_puthex(wLen >> 8); uart_puthex(wLen & 0xFF);
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
      handle_get_descriptor(wValH, wValL, wLen);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
      // enable USB bulk mode (bypass MSC)
      REG_USB_MSC_CFG = 0x00;
      // enable bulk endpoint (without the clear in, it'll get a spurious IN, without the clear out, it'll miss an out)
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      // receive to 0x911B
      //REG_USB_BULK_EP_CMD = USB_BULK_EP_CMD_CBW;
      // receive to 0x7000
      //REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
      // setup UAS mode
      //REG_USB_STATUS = USB_STATUS_DMA_READY;
      send_zlp_ack();
      uart_puts("[*** SET CONFIG ***]\n");
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
      /* Vendor read XDATA via control.  wValue=addr, wLength=size.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t vi;
      for (vi = 0; vi < wLen; vi++) {
        if (bank) DPX = bank;
        uint8_t val = XDATA_REG8(addr + vi);
        if (bank) DPX = 0x00;
        DESC_BUF[vi] = val;
      }
      send_control_data(wLen);
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
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF2) {
      /* 0xF2: SRAM DMA — init DMA engine and arm for bulk transfer.
      *   wValue bit 15 = direction: 0=BULK OUT (host→SRAM), 1=BULK IN (SRAM→host)
      *   wValue bits 0-14 = total sector count (C426:C427)
      *   wIndex low  = start slot (slot_sel for C429, C414 base)
      *   wIndex high = number of slots (for C415 end range; 0 means 1 slot) */
      uint8_t bulk_in = wValH & 0x80;  /* bit 15 of wValue = direction flag */
      uint16_t sectors = (((uint16_t)(wValH & 0x7F)) << 8) | wValL;
      uint8_t slot_sel = REG_USB_SETUP_WIDX_L;
      uint8_t num_slots = REG_USB_SETUP_WIDX_H;
      if (num_slots == 0) num_slots = 1;
      /* DMA_INIT sequence for SRAM DMA */
      REG_NVME_DOORBELL       = 0x0;
      REG_NVME_SECTOR_SIZE_HI = 0x02;
      REG_NVME_SECTOR_SIZE_LO = 0x00;
      REG_NVME_SLOT_START = NVME_SLOT_ENABLE | slot_sel;
      REG_NVME_SLOT_END   = num_slots + slot_sel;
      REG_NVME_SECTOR_COUNT_HI = (uint8_t)(sectors >> 8);
      REG_NVME_SECTOR_COUNT_LO = (uint8_t)(sectors & 0xFF);
      REG_NVME_CTRL_STATUS = NVME_CTRL_DMA_START | (bulk_in ? 0 : NVME_CTRL_WRITE_DIR);
      REG_NVME_CMD_PARAM   = slot_sel;
      send_zlp_ack();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* 0xF0 OUT: PCIe TLP engine.
      *   wValue = fmt_type | (byte_enable << 8)
      *   wIndex low[1:0] = mode (0=single TLP, 1=stream write, 2=stream read)
      *   wIndex low[7:2] = dwords per read chunk (0 → 128 for writes)
      *   DATA_OUT: 12 bytes = addr_lo[4 LE] + addr_hi[4 LE] + value[4 BE] */
      /* Don't configure yet — wait for DATA_OUT phase.
       * SETUP params (wValue/wIndex) are readable from registers in DATA_OUT. */
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* 0xF0 IN: read TLP completion (mode=0 only). Returns 8 bytes. */
      uint8_t ret_status = 0xFF;
      uint32_t t;
      for (t = 0; t < 500000; t++) {
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
        DESC_BUF[0] = REG_PCIE_DATA_3;
        DESC_BUF[1] = REG_PCIE_DATA_2;
        DESC_BUF[2] = REG_PCIE_DATA_1;
        DESC_BUF[3] = REG_PCIE_DATA_0;
        DESC_BUF[4] = REG_PCIE_CPL_HDR_HI;
        DESC_BUF[5] = REG_PCIE_CPL_HDR_LO;
        DESC_BUF[6] = REG_PCIE_COMPL_STATUS;
      } else {
        if (ret_status == 0xFF) uart_puts("[PCIE TIMEOUT]\n");
        int i;
        for (i = 0; i < 7; i++) DESC_BUF[i] = 0;
      }
      DESC_BUF[7] = ret_status;
      send_control_data(8);
    } else {
      if (wLen == 0) send_zlp_ack();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_IN || phase & USB_CTRL_PHASE_STAT_IN) {
    // USB_CTRL_PHASE_DATA_IN on USB 2.0, USB_CTRL_PHASE_STAT_IN on USB 3.0
    if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    if (REG_USB_SETUP_BREQ == 0xF0) {
      /* 0xF0 DATA_OUT: 12 bytes at DESC_BUF (0x9E00).
       *   [0-3]  address low (LE), [4-7] address high (LE), [8-11] value (BE)
       * Read SETUP params now and configure everything atomically. */
      uint8_t fmt_type = REG_USB_SETUP_WVAL_L;
      uint8_t byte_en  = REG_USB_SETUP_WVAL_H;
      uint8_t widx_l   = REG_USB_SETUP_WIDX_L;
      uint8_t mode  = widx_l & 0x03;

      /* Configure PCIe TLP engine */
      REG_PCIE_FMT_TYPE   = fmt_type;
      REG_PCIE_BYTE_EN    = byte_en;
      REG_PCIE_ADDR_0     = DESC_BUF[3];
      REG_PCIE_ADDR_1     = DESC_BUF[2];
      REG_PCIE_ADDR_2     = DESC_BUF[1];
      REG_PCIE_ADDR_3     = DESC_BUF[0];
      REG_PCIE_ADDR_HIGH   = DESC_BUF[7];
      REG_PCIE_ADDR_HIGH_1 = DESC_BUF[6];
      REG_PCIE_ADDR_HIGH_2 = DESC_BUF[5];
      REG_PCIE_ADDR_HIGH_3 = DESC_BUF[4];

      if (mode == 0) {
        /* Single TLP: fire with data from DESC_BUF[8-11] (LE: [8]=LSB, [11]=MSB) */
        if (fmt_type & PCIE_FMT_HAS_DATA) {
          REG_PCIE_DATA_3 = DESC_BUF[8];
          REG_PCIE_DATA_2 = DESC_BUF[9];
          REG_PCIE_DATA_1 = DESC_BUF[10];
          REG_PCIE_DATA_0 = DESC_BUF[11];
        }
        REG_PCIE_STATUS  = PCIE_STATUS_ERROR;
        REG_PCIE_STATUS  = PCIE_STATUS_COMPLETE;
        REG_PCIE_STATUS  = PCIE_STATUS_KICK;
        REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
      } else {
        /* Streaming: read dword count from value field (LE), ADDR regs already set above */
        dma_dwords = ((uint32_t)DESC_BUF[11] << 24) | ((uint32_t)DESC_BUF[10] << 16) |
                     ((uint32_t)DESC_BUF[9] << 8) | DESC_BUF[8];
        if (dma_dwords > 0) {
          if (mode == 1) {
            // host to device, we arm the OUT endpoint
            REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
          }
          if (mode == 2) {
            // device to host, we do the first IN
            do_usb_bulk_in();
          }
        }
      }
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

/*=== ISR ===*/

void handle_usb_bulk_data(void) {
  uint8_t bulk_cfg1, bulk_cfg2;
  bulk_cfg1 = REG_USB_EP_CFG1;
  bulk_cfg2 = REG_USB_EP_CFG2;
  /*uart_puts("[BULK ");
  uart_puthex(bulk_cfg1); uart_puts(" "); uart_puthex(bulk_cfg2);
  uart_puts("]\n");*/
  if (bulk_cfg1 & USB_EP_CFG1_BULK_OUT_COMPLETE) {
    REG_USB_EP_CFG1 = USB_EP_CFG1_BULK_OUT_COMPLETE;
    uint16_t dword_count = (((uint16_t)REG_USB_BULK_OUT_BC_H << 8) | REG_USB_BULK_OUT_BC_L) >> 2;
    if (dma_dwords >= dword_count) {
      pcie_write_chunk((__xdata uint8_t *)0x7000, dword_count);
      dma_dwords -= dword_count;
      if (dma_dwords > 0) REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT; // re-arm OUT
    }
  } else if (bulk_cfg1 & USB_EP_CFG1_BULK_IN_COMPLETE) {
    REG_USB_EP_CFG1 = USB_EP_CFG1_BULK_IN_COMPLETE;
    if (dma_dwords > 0) do_usb_bulk_in();
    return;
  }
}


void int0_isr(void) __interrupt(0) {
  uint8_t int0_type = REG_INT_USB_STATUS;
  if (int0_type & INT_USB_GATE) {
    uint8_t periph_status;
    periph_status = REG_USB_PERIPH_STATUS;

    if (periph_status & USB_PERIPH_BUS_RESET) {
      uart_puts("[UNHANDLED RESET]\n");
    } else if (periph_status & USB_PERIPH_CONTROL) {
      handle_usb_control();
    } else if (periph_status & USB_PERIPH_ALT_LINK) {
      uint8_t status = REG_BUF_CFG_9301;
      uart_puts("[ALT LINK ");
      uart_puthex(status);
      uart_puts("]\n");
      REG_BUF_CFG_9301 = status;
    } else if (periph_status & USB_PERIPH_BULK_DATA) {
      handle_usb_bulk_data();
    } else if (periph_status & USB_PERIPH_EP_COMPLETE) {
      uint8_t ep = REG_USB_EP_READY;
      uart_puts("[EP_COMPLETE "); uart_puthex(ep); uart_puts("]\n");
      REG_USB_EP_READY = ep;
    } else if (periph_status & USB_PERIPH_LINK_EVENT) {
      uint8_t ep = REG_BUF_CFG_9300;
      if (ep & BUF_CFG_9300_SS_FAIL) {
        uart_puts("[USB2 fallback]\n");
        // fallback to USB2
        is_usb2 = 1;
        // without this, USB2 is flaky
        REG_CPU_MODE = CPU_MODE_USB2;
        // enable USB high speed mode
        REG_USB_PHY_CTRL_91C0 = 0x10;
      }
      REG_BUF_CFG_9300 = ep;
      uart_puts("[LINK EVENT ");
      uart_puthex(ep);
      uart_puts(" link=");
      uart_puthex(REG_USB_LINK_STATUS);
      uart_puts("]\n");
    } else if (periph_status & USB_PERIPH_CBW_RECEIVED) {
      // BULK OUT (but only if pointed to 0x911B)
      uint8_t ep = REG_USB_MODE;
      uart_puts("[CBW_RECEIVED "); uart_puthex(ep); uart_puthex(REG_USB_BULK_EP_CMD); uart_puts("]\n");
      REG_USB_MODE = ep;
      REG_USB_BULK_EP_CMD = USB_BULK_EP_CMD_CBW;
    } else {
      uart_puts("[UNHANDLED INT0 ");
      uart_puthex(periph_status);
      uart_puts("]\n");
    }
  }
  if (int0_type & INT_USB_CTRL_PENDING) {
    // NOTE: MSC interrupts are not enabled, if you want them, you can do the two writes here
    uart_puts("[MSC]\n");
    REG_USB_MSC_CTRL = 1;
    REG_USB_MSC_STATUS = 0;
  }
  if (int0_type & ~(INT_USB_GATE | INT_USB_CTRL_PENDING)) {
    uart_puts("[UNHANDLED INT0 TYPE ");
    uart_puthex(int0_type);
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

  // enables CBW_RECEIVED interrupts
  REG_USB_EP_MGMT = 0x00;

  // PCIe TLP engine values that don't change
  REG_PCIE_TLP_CTRL   = 0x01;
  REG_PCIE_TLP_LENGTH = 0x20;

  // PCIe bringup
  REG_TUNNEL_LINK_STATUS = 0xC;          // this is not needed for AMD GPU, but needed for NVMe
  REG_TUNNEL_CTRL_B403 = 0x01;           // fix PCIe link stability
  REG_PCIE_PERST_CTRL  = 0x01;           // assert PERST#
  REG_TUNNEL_LINK_STATE = 0x00;          // clear tunnel link state
  DPX = 0x01; REG_PHY_TLP_ROUTING = PHY_TLP_ROUTING_ENABLE; DPX = 0x00;
  REG_HDDPC_CTRL |= 0x20;                // enable 3.3V
  REG_PCIE_LANE_CTRL_C659 |= 0x01;       // enable 12V
  REG_PHY_TIMER_CTRL_E764 = 0x1C;        // start link training

  // enable USB_PERIPH_LINK_EVENT to fall back to USB2
  REG_BUF_CFG_9303 = 0x33;

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
