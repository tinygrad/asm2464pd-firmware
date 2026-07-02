/*
 * ASM2464PD USB 3.0/USB4 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "util.h"
#include "registers.h"
#include "usb4_state.h"
#include "usb.h"
#include "gpio.h"

__sfr __at(0x93) DPX;   /* DPTR bank select — DPX=1 accesses internal PHY regs */
__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;

#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

/* Briefly mask all interrupts around a main-loop sideband critical section
 * (the sideband service must not race the USB4 INT1 handler). */
#define U4_CRITICAL_ENTER()  (IE &= (uint8_t)~IE_EA)
#define U4_CRITICAL_EXIT()   (IE |= IE_EA)

// blocking version: void uart_putc(uint8_t ch) { while (!REG_UART_TFBF); REG_UART_THR = ch; }
void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

#define TIMER1_MODE_HALF_MS     0x04U
/* Millisecond busy-sleep on Timer1; must never touch the CC10-CC13 PHY/PD mailbox. */
static void sleep(uint16_t milliseconds) {
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | TIMER1_MODE_HALF_MS;
  uint16_t threshold = 2*milliseconds;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold & 0xFF;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
  { uint32_t g = 0; while (!(REG_TIMER1_CSR & TIMER_CSR_EXPIRED) && ++g < 4000000UL); }
}

static uint8_t is_usb2;
static uint32_t __xdata usb4_skip_magic;

/* Streaming PCIe state — configured via 0xF0 control message */
static uint32_t __xdata dma_dwords;    /* total dwords remaining for streaming transfer */
#define DMA_DWORDS_BYTE(n) (((volatile __xdata uint8_t *)&dma_dwords)[(n)])

#include "pcie_pio.h"
#include "pcie_tuning.h"
#include "i2c.h"
#include "pd.h"
#include "sb.h"
#include "usb4.h"
#include "usb4_lanebond.h"
#include "usb4_boot.h"

/* Hardware status packet */
typedef struct {
  uint16_t voltage_mv;   /* INA231 bus voltage */
  int16_t  current_ma;   /* INA231 shunt current (signed) */
} hw_status_t;

static void hw_status_read(__xdata hw_status_t *s) {
  uint16_t shunt_raw = 0, bus_raw = 0;
  (void)ina231_read_u16(INA231_REG_SHUNT, &shunt_raw);
  (void)ina231_read_u16(INA231_REG_BUS, &bus_raw);
  s->voltage_mv = (uint16_t)(((uint32_t)bus_raw * 125) / 100);               /* 1.25 mV/LSB */
  s->current_ma = (int16_t)(((int32_t)(int16_t)shunt_raw * 2500)             /* shunt uV × 1000 */
                            / INA231_SHUNT_UOHM);                            /* / R (uOhm) = mA */
}

static void pcie_power_off(void) {
  /* Hold the downstream device in reset before removing its rails. */
  REG_PCIE_PERST_CTRL = PCIE_PERST_ASSERT;
  REG_TUNNEL_LINK_STATE = 0x00;
  REG_PHY_TIMER_CTRL_E764 = 0x00;
  REG_PCIE_LANE_CTRL_C659 &= (uint8_t)~0x01;
  REG_HDDPC_CTRL &= (uint8_t)~0x20;
  led_set_rgb(false, false, true);  // blue = PCIe powered down
}

static void pcie_power_on(void) {
  REG_TUNNEL_LINK_STATUS = PCIE_LINK_WIDTH_x2;
  REG_TUNNEL_CTRL_B403 = 0x01;                 // fix PCIe link stability
  REG_PCIE_PERST_CTRL  = PCIE_PERST_ASSERT;    // assert PERST#
  REG_TUNNEL_LINK_STATE = 0x00;                // clear tunnel link state
  DPX = 0x01; REG_PHY_TLP_ROUTING = PHY_TLP_ROUTING_ENABLE; DPX = 0x00;
  bank1_write(0x78AF, 0x4F); bank1_write(0x79AF, 0x4F); // rxphy lane commits
  bank1_write(0x7AAF, 0xCF); bank1_write(0x7BAF, 0xCF);
  REG_HDDPC_CTRL |= 0x20;                      // enable 3.3V
  REG_CPU_CTRL_CA81 = 0x0E;
  REG_CPU_MODE_NEXT = 0x21;
  REG_PCIE_LANE_CTRL_C659 |= 0x01;             // enable 12V
  REG_PHY_TIMER_CTRL_E764 = 0x1C;              // start link training
  REG_PCIE_TUNNEL_CFG = PCIE_TLP_CTRL_TUNNEL;  // fix late issue in RDNA3
  REG_PCIE_PERST_CTRL = 0x00;                  // deassert PERST#

  // wait for stable link, bounded so we don't block forever when nothing is attached
  uint8_t stable_samples = 0;
  uint8_t attempts;
  for (attempts = 0; attempts < 20 && stable_samples < 3; attempts++) {
    uint8_t ltssm_state = REG_PCIE_LTSSM_STATE;
    DPX = 0x01;
    uint8_t link_info = REG_PHY_PCIE_LINK_INFO;
    DPX = 0x00;
    uart_puts("[PCIe ");
    uart_puthex(ltssm_state);
    uart_puts((REG_SYS_CTRL_E765 & SYS_CTRL_E765_PCIE_LINK_UP) ? "  UP " : " down");
    uart_puts(" Gen");
    uart_puthex(link_info & 0x0F);
    uart_puts(" x");
    uart_puthex((link_info >> 4) & 0x0F);
    if (ltssm_state == 0x78) uart_puts(" CONNECTED");
    uart_puts("]\n");
    if (ltssm_state == 0x78) {
      stable_samples++;
    } else {
      stable_samples = 0;
    }
    sleep(100);
  }
  if (stable_samples < 3) uart_puts("[PCIe timeout]\n");

  // green = PCIe link up, red = link down
  bool link_up = (stable_samples >= 3);
  led_set_rgb(!link_up, link_up, false);
}

static void do_usb_bulk_in(void) {
  /* Move at most one bulk max-packet per IN: 512 B (USB2) or 1024 B (USB3),
   * i.e. chunk = min(dma_dwords remaining, packet size in dwords). */
  uint16_t max_dwords = is_usb2 ? (512/4) : (1024/4);
  uint16_t chunk = (dma_dwords > max_dwords) ? max_dwords : (uint16_t)dma_dwords;

  pcie_read_chunk((__xdata uint8_t *)0x8000, chunk);
  uint16_t nbytes = chunk * 4;
  REG_USB_BULK_IN_LEN_H = nbytes >> 8;
  REG_USB_BULK_IN_LEN_L = nbytes & 0xFF;
  dma_dwords -= chunk;
  REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_IN;
}

/* Vendor control requests (bRequest) — the tinygrad USB interface. */
#define USB_VREQ_HW_STATUS  0xC0  /* IN:  INA231 voltage/current */
#define USB_VREQ_XDATA_RD   0xE4  /* IN:  read XDATA  (wValue=addr, wIndexH=bank) */
#define USB_VREQ_XDATA_WR   0xE5  /* OUT: write XDATA (wValue=addr, wIndexL=val) */
#define USB_VREQ_TLP        0xF0  /* PCIe TLP engine (single TLP / streaming) */
#define USB_VREQ_SRAM_DMA   0xF2  /* SRAM DMA init + arm */
#define USB_VREQ_PCIE_PWR   0xF3  /* PCIe power on/off */

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
      usb_handle_set_address(wValL);
      uart_puts("[A]\n");
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
      usb_handle_get_descriptor(is_usb2, wValH, wValL, wLen);
    } else if (bmReq == USB_SETUP_RECIP_ENDPOINT && bReq == USB_REQ_CLEAR_FEATURE && wValL == 0x00) {
      /* CLEAR_FEATURE(ENDPOINT_HALT) — reset bulk endpoint and cancel streaming.
       * bmRequestType=0x02 (host-to-dev, standard, endpoint), wValue=0 (ENDPOINT_HALT),
       * wIndex = endpoint address (0x02=OUT, 0x81=IN). */
      uint8_t ep_addr = REG_USB_SETUP_WIDX_L;
      if (ep_addr == 0x02) {
        REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      } else if (ep_addr == 0x81) {
        REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      }
      dma_dwords = 0;
      usb_send_zlp();
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
      // enable USB bulk mode (bypass MSC)
      REG_USB_MSC_CFG = 0x00;
      // clear bulk endpoints
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      dma_dwords = 0;
      usb_send_zlp();
      uart_puts("[*** SET CONFIG ***]\n");
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) && bReq == USB_REQ_SET_INTERFACE) {
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
      REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
      dma_dwords = 0;
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_HW_STATUS) {
      /* 0xC0 IN: hw_status_t */
      hw_status_read((__xdata hw_status_t *)DESC_BUF);
      usb_send_data(sizeof(hw_status_t));
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_XDATA_RD) {
      /* Vendor read XDATA via control.  wValue=addr, wLength=size.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint16_t maxlen = is_usb2 ? 64 : 512;
      uint16_t rlen = (wLen > maxlen) ? maxlen : wLen;
      uint16_t vi;
      for (vi = 0; vi < rlen; vi++) {
        if (bank) DPX = bank;
        uint8_t val = XDATA_REG8(addr + vi);
        if (bank) DPX = 0x00;
        DESC_BUF[vi] = val;
      }
      usb_send_data(rlen);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_XDATA_WR) {
      /* Vendor write XDATA via control.  wValue=addr, wIndex low=val.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t val = REG_USB_SETUP_WIDX_L;
      if (bank) DPX = bank;
      XDATA_REG8(addr) = val;
      if (bank) DPX = 0x00;
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_SRAM_DMA) {
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
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_PCIE_PWR) {
      /* 0xF3: PCIe power control.
       *   wValue low bit 0 = 0 power off, 1 power on. */
      if (wValL & 0x01) {
        pcie_power_on();
      } else {
        pcie_power_off();
      }
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_TLP) {
      /* 0xF0 OUT: PCIe TLP engine.
      *   wValue = fmt_type | (byte_enable << 8)
      *   wIndex low[1:0] = mode (0=single TLP, 1=stream write, 2=stream read)
      *   wIndex low[7:2] = dwords per read chunk (0 → 128 for writes)
      *   DATA_OUT: 12 bytes = addr_lo[4 LE] + addr_hi[4 LE] + value[4 LE] */
      /* Don't configure yet — wait for DATA_OUT phase.
       * SETUP params (wValue/wIndex) are readable from registers in DATA_OUT. */
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == USB_VREQ_TLP) {
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
      usb_send_data(8);
    } else {
      if (wLen == 0) usb_send_zlp();
    }
  } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
  } else if (phase & USB_CTRL_PHASE_DATA_IN || phase & USB_CTRL_PHASE_STAT_IN) {
    // USB_CTRL_PHASE_DATA_IN on USB 2.0, USB_CTRL_PHASE_STAT_IN on USB 3.0
    if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    if (REG_USB_SETUP_BMREQ == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) &&
        REG_USB_SETUP_BREQ == USB_VREQ_TLP) {
      /* 0xF0 DATA_OUT: 12 bytes at DESC_BUF (0x9E00).
       *   [0-3]  address low (LE), [4-7] address high (LE), [8-11] value (LE)
       * Read SETUP params now and configure everything atomically. */
      uint8_t fmt_type = REG_USB_SETUP_WVAL_L;
      uint8_t byte_en  = REG_USB_SETUP_WVAL_H;
      uint8_t widx_l   = REG_USB_SETUP_WIDX_L;
      uint8_t mode  = widx_l & 0x03;

      /* Reset any in-flight streaming transfer so stale ISRs are no-ops. */
      dma_dwords = 0;

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
        DMA_DWORDS_BYTE(0) = DESC_BUF[8];
        DMA_DWORDS_BYTE(1) = DESC_BUF[9];
        DMA_DWORDS_BYTE(2) = DESC_BUF[10];
        DMA_DWORDS_BYTE(3) = DESC_BUF[11];
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
      usb_send_zlp();
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
      /* 0x91D1 USB-SS / USB4-router link-event demux. */
      uint8_t link_event = REG_USB_PHY_CTRL_91D1;
      if (link_event & USB_91D1_FLAG) {
        REG_USB_PHY_CTRL_91D1 = USB_91D1_FLAG;
        if (IS_USB4()) u4c_lane_reinit_gate(0);
      } else {
        REG_USB_PHY_CTRL_91D1 = link_event;
        uart_puts("[RST ");
        uart_puthex(link_event);
        uart_puts("]\n");
      }
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
      /* 0x9302 USB4-router link-event demux; service .2 then the 9300 SS event. */
      if (REG_BUF_CFG_9302 & 0x04) {
        REG_BUF_CFG_9302 = 0x04;
        if (IS_USB4()) u4c_lane_reinit_gate(1);
      }
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

/* INT1 / EX1: the PD / USB4 / system interrupt aggregate (C806/C80A/EC06). */
void int1_isr(void) __interrupt(1) {
  USB4_INT1_BODY();
}

void main(void) {
  uint8_t usb4_reset_fallback = 0;

  // without this, UART has parity
  REG_UART_LCR &= ~LCR_PARITY_MASK;

  uart_puts("\n[BOOT]\n");
  led_set_rgb(false, false, true);

  // flash controller — needed for the USB serial OTP read on enumeration
  flash_init();

  USB4_SELECT_BOOT_MODE(usb4_reset_fallback);

  if (IS_USB4()) {
    pcie_power_off();
    usb4_state_prepare();
  } else {
    usb_phy_tune();

    if (usb4_reset_fallback) {
      USB4_REINIT_USB3_AFTER_RESET_FALLBACK();
    }

    // PCIe TLP engine values that don't change + tuning
    REG_PCIE_TLP_CTRL   = 0x01;
    REG_PCIE_TLP_LENGTH = 0x20;
    pcie_apply_x2_rxphy_tuning();
    pcie_power_off();

    // PCIe power on for backwards compatibility, can be removed
    pcie_power_on();
  }

  if (IS_USB4()) {
    is_usb2 = 1; // sideband access
    usb4_policy_enable();
  } else {
    usb_init_controller(0);
  }

  // enable interrupts (EX1 = PD/USB4 INT1)
  IE = (uint8_t)(IE_EA | IE_EX0 | (IS_USB4() ? (IE_EX1 | IE_ET0) : 0));

  // INA231 power monitor: init in both modes so the 0xC0 hw_status vendor
  // request works over USB3 and over the USB4-tunneled USB function.
  i2c_init();
  ina231_init();

  uint8_t kicks = 0;
  uint8_t usb4_fallback_ticks = 0;
  while (1) {
    if (IS_USB4()) {
      if (u4_boot.pd_seen && !u4_pd.enter_usb_accepted && !u4_boot.sb_asserted) {
        if (u4_pd.usb3_fallback_flag || usb4_fallback_ticks >= 12) {
          usb4_fallback_to_usb3();
          continue;
        } else {
          usb4_fallback_ticks++;
        }
      }
      if (u4_sb.conn_consequence_done) {
        U4_CRITICAL_ENTER();
        if (u4_sb.state != U4FSM_IDLE) {
          uint16_t cur = u4lb_read_lane_width_cnt();
          uint16_t snap = ((uint16_t)u4_sb.walk_throttle_snap_hi << 8) | u4_sb.walk_throttle_snap_lo;  /* 0x076A:0x076B */
          if ((uint16_t)(snap - cur) >= 3) {
            u4lb_fsm_step();
            cur = u4lb_read_lane_width_cnt();
            u4_sb.walk_throttle_snap_hi = (uint8_t)(cur >> 8);
            u4_sb.walk_throttle_snap_lo = (uint8_t)cur;
          }
        }
        if (u4_sb.routerop_resp_armed != 0) sb_routerop_response(u4lb_routerop_poll());
        U4_CRITICAL_EXIT();
      }

      U4_CRITICAL_ENTER();
      sb_connect_reservice();
      u4c_native_routerop_service();
      U4_CRITICAL_EXIT();

      if (SB_RD(0x26) & 0x02) {
        U4_CRITICAL_ENTER();
        sb_routerop_pending();
        if (SB_RD(0x26) & 0x02) SB_WR(0x26, 0x02);   // W1C SB[0x26].1 after response, like a066
        U4_CRITICAL_EXIT();
      }

      if (u4_sb.conn_consequence_done) {
        continue;
      }
      if (u4_boot.sb_asserted) { uint32_t b; for (b = 0; b < 60000UL; b++) { __asm nop __endasm; } }
      else             { sleep(500); }
      /* Give the host a settle window before each Hard Reset kick. Repeated immediate kicks can keep
       * power-cycling the device before the PD contract completes. */
      { static __xdata uint8_t pd_settle = 0;
        if (!u4_boot.pd_seen) {
          if (pd_settle < 12) { pd_settle++; }
          else if (kicks < 8) { pd_drive_hard_reset(); kicks++; pd_settle = 0; }
        }
      }
    }
  }
}
