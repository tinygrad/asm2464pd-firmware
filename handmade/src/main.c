/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "usb4_state.h"
#include "usb.h"
#include "gpio.h"

__sfr __at(0x93) DPX;   /* DPTR bank select; DPX=1 accesses internal PHY regs */
__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;

#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

/* Blocking UART putc with a bounded spin so a wedged UART can't hang the CPU. */
void uart_putc(uint8_t ch) { uint16_t g = 0; while (!REG_UART_TFBF && ++g < 0x8000) { } REG_UART_THR = ch; }
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

static volatile uint8_t __xdata is_usb2;

/* Streaming PCIe state — dwords remaining for the current transfer. */
static volatile uint32_t __xdata dma_dwords;

/* Super-loop FSM-stall counter that gates the deferred bank0_8a89 fallback. */
static volatile uint8_t __xdata __at(0x0B52) fsm_stall;


#include "pcie_pio.h"
#include "pcie_tuning.h"
#include "i2c.h"
#include "pd.h"
#include "pd_dispatch.h"
#include "vdm.h"
#include "sb.h"
#include "usb4.h"
#include "sb_router.h"
#include "usb4_irq.h"
#include "boot_phy.h"
#include "usb4_connect.h"
#include "usb4_lanebond.h"

/* Hardware status packet */
typedef struct {
  uint16_t voltage_mv;   /* INA231 bus voltage */
  int16_t  current_ma;   /* INA231 shunt current (signed) */
} hw_status_t;

static void hw_status_read(__xdata hw_status_t *s) {
  static __xdata uint16_t shunt_raw, bus_raw;
  shunt_raw = 0; bus_raw = 0;
  (void)ina231_read_u16(INA231_REG_SHUNT, &shunt_raw);
  (void)ina231_read_u16(INA231_REG_BUS, &bus_raw);
  s->voltage_mv = (uint16_t)(((uint32_t)bus_raw * 125) / 100);               /* 1.25 mV/LSB */
  s->current_ma = (int16_t)(((int32_t)(int16_t)shunt_raw * 2500)             /* shunt uV × 1000 */
                            / INA231_SHUNT_UOHM);                            /* / R (uOhm) = mA */
}

static void pcie_power_off(void) {
  /* Hold the downstream device in reset and remove its rails. */
  REG_PCIE_PERST_CTRL = PCIE_PERST_ASSERT;
  REG_TUNNEL_LINK_STATE = 0x00;
  REG_PHY_TIMER_CTRL_E764 &= 0x10;
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
  uint16_t max_dwords = is_usb2 ? (512/4) : (1024/4);
  uint16_t chunk = (dma_dwords > max_dwords) ? max_dwords : (uint16_t)dma_dwords;
  pcie_read_chunk((__xdata uint8_t *)0x8000, chunk);
  uint16_t nbytes = chunk * 4;
  REG_USB_BULK_IN_LEN_H = nbytes >> 8;
  REG_USB_BULK_IN_LEN_L = nbytes & 0xFF;
  dma_dwords -= chunk;
  REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_IN;
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
      usb_handle_set_address(wValL);
      uart_puts("[A]\n");
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
      usb_handle_get_descriptor(is_usb2, wValH, wValL, wLen);
    } else if (bmReq == USB_SETUP_RECIP_ENDPOINT && bReq == USB_REQ_CLEAR_FEATURE && wValL == 0x00) {
      /* CLEAR_FEATURE(ENDPOINT_HALT) — reset bulk endpoint and cancel streaming. */
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
      // clearn bulk endpoints
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
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xC0) {
      /* 0xC0 IN: hw_status_t */
      hw_status_read((__xdata hw_status_t *)DESC_BUF);
      usb_send_data(sizeof(hw_status_t));
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
      /* Vendor read XDATA via control. wValue=addr, wLength=size, wIndex-hi=bank. */
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
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
      /* Vendor write XDATA via control. wValue=addr, wIndex-lo=val, wIndex-hi=bank. */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t val = REG_USB_SETUP_WIDX_L;
      if (bank) DPX = bank;
      XDATA_REG8(addr) = val;
      if (bank) DPX = 0x00;
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF2) {
      /* 0xF2: SRAM DMA — init DMA engine and arm for bulk transfer. */
      uint8_t bulk_in = wValH & 0x80;
      uint16_t sectors = (((uint16_t)(wValH & 0x7F)) << 8) | wValL;
      uint8_t slot_sel = REG_USB_SETUP_WIDX_L;
      uint8_t num_slots = REG_USB_SETUP_WIDX_H;
      if (num_slots == 0) num_slots = 1;
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
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF3) {
      /* 0xF3: PCIe power control. wValue bit0 = 0 off / 1 on. */
      if (wValL & 0x01) {
        pcie_power_on();
      } else {
        pcie_power_off();
      }
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* 0xF0 OUT: PCIe TLP engine. Configured later in the DATA_OUT phase. */
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
        REG_USB_SETUP_BREQ == 0xF0) {
      /* 0xF0 DATA_OUT: 12 bytes at DESC_BUF (addr-lo, addr-hi, value, all LE). */
      uint8_t fmt_type = REG_USB_SETUP_WVAL_L;
      uint8_t byte_en  = REG_USB_SETUP_WVAL_H;
      uint8_t widx_l   = REG_USB_SETUP_WIDX_L;
      uint8_t mode  = widx_l & 0x03;

      dma_dwords = 0;

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
        /* Single TLP: fire with data from DESC_BUF[8-11] (LE). */
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
        /* Streaming: read dword count from the value field (LE). */
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


/* Diagnostic print budgets seeded in main(). */
static volatile uint8_t __xdata isr_dbg_budget, isr_dbg_budget2, isr_dbg_budget3;

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
        bank0_c9a8(0);
        uart_puts("[91D1.1->c9a8]\n");
      } else {
        REG_USB_PHY_CTRL_91D1 = link_event;
      }
      uart_puts("[RST ");
      uart_puthex(link_event);
      uart_puts("]\n");
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
      uint8_t s9302 = REG_BUF_CFG_9302;
      if (s9302 & 0x04) {
        REG_BUF_CFG_9302 = 0x04;
        bank0_c9a8(1);
        uart_puts("[9302.2->c9a8]\n");
      }
      uint8_t ep = REG_BUF_CFG_9300;
      if (ep & BUF_CFG_9300_SS_FAIL) {
        /* In USB4 mode do not drop to USB2 (that abandons lane training); only ack. */
        if (!(XDATA_REG8V(0x09F9) & 0x83)) {
          uart_puts("[USB2 fallback]\n");
          // fallback to USB2
          is_usb2 = 1;
          // without this, USB2 is flaky
          REG_CPU_MODE = CPU_MODE_USB2;
          // enable USB high speed mode
          REG_USB_PHY_CTRL_91C0 = 0x10;
        } else {
          uart_puts("[SS_FAIL ack (USB4, no USB2 drop)]\n");
        }
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
  uint8_t saved_dpx = DPX;
  DPX = 0x00;
  if (isr_dbg_budget && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget--; uart_putc('I'); }
  if (REG_INT_SYSTEM & 0x01) cc_pd_timer_tick();
  if (isr_dbg_budget2 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget2--; uart_putc('t'); }
  if (REG_CPU_EXEC_STATUS_2 & 0x04) { REG_CPU_EXEC_STATUS_2 = 0x04; }
  if (REG_INT_PCIE_NVME & 0x40) pd_rx_isr();
  if (isr_dbg_budget3 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget3--; uart_putc('p'); }
  if (XDATA_REG8V(0x09F9) & 0x83) usb4_int_demux();
  if (REG_INT_SYSTEM & 0x10) { /* C806.4 ack-only (no-op) */ }
  DPX = saved_dpx;
}

void main(void) {
  // without this, UART has parity
  REG_UART_LCR &= ~LCR_PARITY_MASK;

  uart_puts("\n[BOOT]\n");
  led_set_rgb(false, false, true);

  // flash controller — needed for the USB serial OTP read on enumeration
  flash_init();

  // Type-C SBU + PHY config + SB-block enable; powers the sideband transport.
  boot_phy_bringup_early();
  uart_puts("[BOOTPHY cc3f="); uart_puthex(REG_LTSSM_CTRL);
  uart_puts(" cc30=");          uart_puthex(REG_CPU_MODE);
  uart_puts(" e712=");          uart_puthex(REG_LINK_STATUS_E712);
  uart_puts(" sb05=");          uart_puthex(SB_RD(0x05)); uart_puts("]\n");

  // Seed the lane-engine link width/mode and the USB4 tunnel state flags.
  bank0_92c5_seed();

  // Establish USB4 intent (0x87 = tunnel route + VDM-ACK) before the gates below.
  XDATA_REG8V(0x09F9) = 0x87;
  // DROM cap bits read by sb_rom_descriptor_load; 0x09F4 deliberately left at default.
  XDATA_REG8V(0x09F5) = 1; XDATA_REG8V(0x09F6) = 1;
  XDATA_REG8V(0x09F7) = 3; XDATA_REG8V(0x09F8) = 1; XDATA_REG8V(0x09FB) = 3;
  // 0x0A57/0x0A58 = device PID-low / bcdDevice-hi. Stock relies on the boot env (SPI-flash shadow)
  // pre-loading these; there is no firmware writer. Without them they stay uninit 0x55, so the SB
  // connect descriptor TX becomes 0104 5555 instead of 0104 6324 and the host never escalates
  // SB[0x18] 05->63 to assign the route-ID (no lane bond, no GPU). Values from the stock wire trace.
  XDATA_REG8V(0x0A57) = 0x63;
  XDATA_REG8V(0x0A58) = 0x24;

  // PHY tune + PCIe power cycling only in non-USB4 mode (they clobber CM-owned regs).
  if (!(XDATA_REG8V(0x09F9) & 0x83)) {
    usb_phy_tune();

    // PCIe TLP engine values that don't change + tuning
    REG_PCIE_TLP_CTRL   = 0x01;
    REG_PCIE_TLP_LENGTH = 0x20;
    pcie_apply_x2_rxphy_tuning();
    pcie_power_off();

    // PCIe power on for backwards compatibility, can be removed
    pcie_power_on();
  }

  // Bring up the USB PIPE engine and arm the upstream USB4 PHY link.
  usb_pipe_engine_init();
  usb4_phy_arm();
  uart_puts("[PHYarm e318="); uart_puthex(REG_PHY_COMPLETION_E318);
  uart_puts(" 91c0=");        uart_puthex(REG_USB_PHY_CTRL_91C0);
  uart_puts(" e712=");        uart_puthex(REG_LINK_STATUS_E712); uart_puts("]\n");

  // Present a PD-capable Type-C attach and arm the PD engine before interrupts.
  pd_keystone_init();

  // Arm the USB4 SB-transport / router interrupt path so the host's connect raises C80A.5.
  usb4_irq_arm();
  uart_puts("[U4irq c21b="); uart_puthex(REG_PHY_LINK_CTRL_C21B);
  uart_puts(" c202=");        uart_puthex(REG_LINK_CTRL);
  uart_puts(" e741=");        uart_puthex(REG_PHY_PLL_CTRL);
  uart_puts(" cc43=");        uart_puthex(REG_CPU_CLK_CFG); uart_puts("]\n");

  // USB4 CM router-op RX-enable so the host's router-op queries are received.
  if (XDATA_REG8V(0x09F9) & 0x81) {
    usb4_routerop_init();
    uart_puts("[U4rop ec00="); uart_puthex(XDATA_REG8V(0xEC00));
    uart_puts(" ec04=");        uart_puthex(REG_NVME_EVENT_ACK);
    uart_puts(" c807=");        uart_puthex(REG_INT_DMA_CTRL);
    uart_puts(" ea88=");        uart_puthex(XDATA_REG8V(0xEA88)); uart_puts("]\n");
  }

  // USB-mode fork: skip the USB3 device engine in USB4 mode (it would abandon USB4).
  if (XDATA_REG8V(0x09F9) & 0x83) {
    uart_puts("[USB4 mode: skip USB3 device bring-up]\n");
  } else {
    // Bring USB up. force_usb2=0: try SS first, fall back via LINK_EVENT.
    usb_init_controller(0);
  }

  // Zero the USB4 router-op mailbox and SB-router active-port index before any USB4 INT.
  { uint8_t z; for (z = 0; z <= (0x0B1F - 0x0B02); z++) XDATA_REG8V(0x0B02 + z) = 0; }
  XDATA_REG8V(0x06F1) = 0;
  // SB-router route-up / lane-bonded / per-lane CL0 / transport-substate latches.
  XDATA_REG8V(0x0766) = 0; XDATA_REG8V(0x072D) = 0;
  XDATA_REG8V(0x074E) = 0; XDATA_REG8V(0x074F) = 0;
  XDATA_REG8V(0x06EE) = 0; XDATA_REG8V(0x06EF) = 0; XDATA_REG8V(0x06F0) = 0;
  // Lane-bond FSM: enable gate, FSM state, sub-states and route-enable latch.
  XDATA_REG8V(0x06EC) = 0; XDATA_REG8V(0x06ED) = 0;
  XDATA_REG8V(0x0758) = 0; XDATA_REG8V(0x0759) = 0; XDATA_REG8V(0x075A) = 0;
  XDATA_REG8V(0x075B) = 0; XDATA_REG8V(0x075C) = 0; XDATA_REG8V(0x0718) = 0;
  // b0b4 gate inputs: connect-present, lane-width snapshot, connect-descriptor scratch.
  XDATA_REG8V(0x0765) = 0; XDATA_REG8V(0x0768) = 0; XDATA_REG8V(0x0769) = 0;
  XDATA_REG8V(0x0752) = 0; XDATA_REG8V(0x0753) = 0;
  // Connect_U4 one-shot suppress: force clear so the first connect runs the SB assert.
  XDATA_REG8V(0x07ED) = 0;

  // Seed the XDATA scratch flags/budgets (crt0 only zeroes IRAM, not XDATA).
  { uint8_t z; for (z = 0; z <= (0x0B58 - 0x0B45); z++) XDATA_REG8V(0x0B45 + z) = 0; }
  sb_con_print_budget = 6;
  af38_s3_budget = 30;
  u4lb_edf5_print_budget = 40;
  u4lb_s5_print_budget = 60;
  isr_dbg_budget = 20; isr_dbg_budget2 = 20; isr_dbg_budget3 = 20;
  u4lb_s5_seen = 0; u4lb_s5_last759 = 0; u4lb_s5_last75b = 0;

  // enable interrupts (EX1 = PD/USB4 INT1)
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  i2c_init();
  ina231_init();

  // Milestone 1: prompt the host into PD with periodic Hard Resets until it engages
  // (pd_seen is set by the PD-RX ISR on the first received message), then just observe.
  uint8_t kicks = 0;
  uint8_t sb_diag_count = 0;
  while (1) {
    /* FSM-advance, run first and delay-free so it tracks the connect edge tightly. */
    if ((XDATA_REG8V(0x09F9) & 0x83) && XDATA_REG8V(0x06EC)) {
      uint8_t fsm_before = XDATA_REG8V(0x06ED);
      IE &= (uint8_t)~IE_EA;
      if (isr_dbg_budget && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget--; uart_putc('c'); }
      sb_cb10_lane_advance();
      if (isr_dbg_budget2 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget2--; uart_putc('n'); }
      // Set 0x0765 from the host connect descriptor (kept off the ISR stack path).
      sb_connect_present_poll();
      if (isr_dbg_budget3 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget3--; uart_putc('k'); }
      if (XDATA_REG8V(0x06ED) == 0) {             // first-arm the FSM
        uart_puts("[LB arm]");
        u4lb_eb62(0, 3);
        u4lb_98ec();   // snapshot CCE4:CCE5 lane-width into 0x768:0x769
        lb_walk_throttle_snap_hi = REG_LANE_WIDTH_CNT_HI;   // seed cb10's running snapshot (0x076A:0x076B)
        lb_walk_throttle_snap_lo = REG_LANE_WIDTH_CNT_LO;
      }
      /* stock cb10 e672 gate (CODE_BANK1::cb87-cbbd, byte-true): when 0x06ED!=0, re-arm the LANE_TRAIN
       * HW via ee57()->ec51() (CCE0-CCE3 + 0x0774 toggle) so the device keeps emitting fresh training
       * symbols EVERY pass, and step the walker e672 ONCE only when the PHY lane-width counter
       * CCE4:CCE5 has moved >=2 since the last pass, re-snapshotting after. This paces the CL walker to
       * host lane-training progress. The handmade previously free-ran e672 4x with NO re-arm and NO
       * throttle, so the device stopped emitting training symbols at the 2D2D plateau and the host
       * withheld the SB[0xA0] 0x01->0x02 confirm (deep-dive 2026-06-16). */
      if (XDATA_REG8V(0x06ED) != 0) {
        uint16_t cce, snap, delta;
        u4lb_ee57();
        cce   = (uint16_t)(((uint16_t)REG_LANE_WIDTH_CNT_HI << 8) | REG_LANE_WIDTH_CNT_LO);
        snap  = (uint16_t)(((uint16_t)lb_walk_throttle_snap_hi << 8) | lb_walk_throttle_snap_lo);
        delta = (uint16_t)(snap - cce);
        if (delta >= 2) {
          u4lb_e672();
          u4lb_ee57();
          lb_walk_throttle_snap_hi = REG_LANE_WIDTH_CNT_HI;
          lb_walk_throttle_snap_lo = REG_LANE_WIDTH_CNT_LO;
        }
      }
      /* deep-dive rank-1 (2026-06-16): the host stays at tbadapters "Training/Bonding" because the
       * device's E764 RX-PLL train never completes -- pll=F4F4 (PLL locked) but E762=00 so E762.4
       * (the RXPLL "trained/ready" latch) stays clear and phy_rxpll_train_busy=1. Stock runs the cdc6
       * train in state-4; re-drive it per state-5 walker pass while busy so E762.4 can latch under the
       * host's live lane-training pattern, completing the train so the host advances Training->CL0. */
      if (XDATA_REG8V(0x06ED) == 5) {
        u4lb_e764_rxpll_train();   /* re-drive EVERY state-5 pass (not just while busy) -- keep the
                                    * train polling E762.4 under the host's live stimulus to push the
                                    * snap past 28xx toward the 3C3C bond-confirm */
      }
      // cb10 tail: send the deferred router-op CONFIG-READ RESPONSE (stock cdf5) the host blocks on.
      // u4lb_eda0() (called here for its 0x0775/0x0719 token-clear side-effect) yields the selector
      // cdf5 needs (0/2 => build+TX the lane-config response; 1 => nothing this pass, arm stays set).
      if (sb_cdf5_substate_arm != 0) sb_cdf5_routerop_response(u4lb_eda0());
      IE |= IE_EA;
      // Track FSM stall: count when 0x06ED makes no progress, reset when it advances.
      if (XDATA_REG8V(0x06ED) == fsm_before) { if (fsm_stall < 0xFF) fsm_stall++; }
      else                                   { fsm_stall = 0; }
    }

    // SB router-op responder, deferred off the INT1 ISR to keep the stack shallow.
    if (sb_pend_int_pending || (SB_RD(0x26) & 0x02)) {
      sb_pend_int_pending = 0;
      IE &= (uint8_t)~IE_EA;
      if (SB_RD(0x26) & 0x02) SB_WR(0x26, 0x02);   // W1C SB[0x26].1
      sb_a5d8_pend_int();
      IE |= IE_EA;
    }

    // Deferred bank0_8a89 drive: run once, only after the FSM has clearly stalled.
    if (sb_run_8a89_pending && !sb_8a89_done &&
        fsm_stall >= 6 &&
        !((SB_RD(0xA0) & 0x0F) == 0x02 && (SB_RD(0xA1) & 0x0F) == 0x02)) {
      sb_run_8a89_pending = 0;
      sb_8a89_done = 1;
      uart_puts("[SBcon->8a89 (deferred)]\n");
      IE &= (uint8_t)~IE_EX1;
      bank0_c9a8(0);
      IE |= IE_EX1;
      uart_puts("[8a89 ret e764=");  uart_puthex(REG_PHY_TIMER_CTRL_E764);
      uart_puts(" e751=");           uart_puthex(REG_PHY_POLL_E751);
      uart_puts(" e302=");           uart_puthex(REG_PHY_MODE_E302);
      uart_puts(" c80a=");           uart_puthex(REG_INT_PCIE_NVME);
      uart_puts(" ec06=");           uart_puthex(REG_NVME_EVENT_STATUS); uart_puts("]\n");
    }

    // Deferred tunnel-up: run the PCIe bring-up here (uses sleep()/long polls).
    if (sb_tunnel_up_pending) {
      sb_tunnel_up_pending = 0;
      if ((SB_RD(0xA0) & 0x0F) == 0x02 && (SB_RD(0xA1) & 0x0F) == 0x02) {  // both lanes CL0
        uart_puts("[TunnelUp->pcie_on]\n");
        pcie_power_on();
      } else {
        uart_puts("[TunnelUp deferred: lanes not CL0]\n");
      }
    }

    // While a connect is in progress keep the loop tight; only delay when idle.
    if (XDATA_REG8V(0x06EC)) {
      continue;
    }
    if (sb_asserted) { uint32_t b; for (b = 0; b < 60000UL; b++) { __asm nop __endasm; } }
    else             { sleep(500); }
    // E302 diagnostic: poll in a bounded window to read whether the link trained.
    if (sb_asserted && sb_diag_count < 8) {
      sb_diag_count++;
      { static __xdata uint32_t p; static __xdata uint8_t e302; static __xdata uint8_t best;
        e302 = 0; best = 0;
        for (p = 0; p < 2000000UL; p++) {
          e302 = REG_PHY_MODE_E302;
          if (((e302 >> 4) & 3) > ((best >> 4) & 3)) best = e302;
          c80a_acc |= REG_INT_PCIE_NVME;
          if (((e302 >> 4) & 3) >= 2) break;
        }
        uart_puts("[SBDIAG e302=");      uart_puthex(e302);
        uart_puts(" best=");             uart_puthex(best);
        uart_puts(" c80a=");             uart_puthex(REG_INT_PCIE_NVME);
        uart_puts(" ec06=");             uart_puthex(REG_NVME_EVENT_STATUS);
        uart_puts(" 91c0=");             uart_puthex(REG_USB_PHY_CTRL_91C0);
        uart_puts(" e318=");             uart_puthex(REG_PHY_COMPLETION_E318);
        uart_puts(" cc30=");             uart_puthex(REG_CPU_MODE);
        uart_puts(" is_usb2=");          uart_puthex(is_usb2);
        uart_puts(" 09f9=");             uart_puthex(XDATA_REG8V(0x09F9));
        uart_puts(" 09fa=");             uart_puthex(XDATA_REG8V(0x09FA));
        uart_puts(" 07ba=");             uart_puthex(XDATA_REG8V(0x07BA));
        uart_puts(" 0af1=");             uart_puthex(XDATA_REG8V(0x0AF1));
        uart_puts(" c80aACC=");          uart_puthex(c80a_acc);
        uart_puts("]\n");
        uart_puts("[LYR c80a5=");        uart_puthex((c80a_acc >> 5) & 1);
        uart_puts(" e764=");             uart_puthex(REG_PHY_TIMER_CTRL_E764);
        uart_puts(" e751=");             uart_puthex(REG_PHY_POLL_E751);
        uart_puts(" 09fa=");             uart_puthex(XDATA_REG8V(0x09FA));
        uart_puts(" 0af1=");             uart_puthex(XDATA_REG8V(0x0AF1));
        uart_puts(" 07e8=");             uart_puthex(XDATA_REG8V(0x07E8));
        uart_puts(" 8a89?=");            uart_puthex(bank0_8a89_entered);
        uart_puts(" c801=");             uart_puthex(REG_INT_ENABLE);
        uart_puts(" c809=");             uart_puthex(REG_INT_CTRL);
        uart_puts(" 9101=");             uart_puthex(REG_USB_PERIPH_STATUS);
        uart_puts(" 91d1=");             uart_puthex(REG_USB_PHY_CTRL_91D1);
        uart_puts(" 9302=");             uart_puthex(REG_BUF_CFG_9302);
        uart_puts("]\n");
        if (((best >> 4) & 3) >= 2) uart_puts("[*** USB4 TRAINED ***]\n");
        else                        uart_puts("[!!! E302 NOT TRAINED !!!]\n");
      }
    }
    uart_puts("[TICK seen="); uart_puthex(tick_seen);
    uart_puts(" cc_hit=");     uart_puthex(cc_hit);
    uart_puts(" c806=");       uart_puthex(REG_INT_SYSTEM);
    uart_puts(" cc91=");       uart_puthex(REG_CPU_DMA_INT);
    uart_puts(" cc81=");       uart_puthex(REG_CPU_INT_CTRL);
    uart_puts(" c809=");       uart_puthex(REG_INT_CTRL); uart_puts("]\n");
    uart_puts("[U ");
    uart_puthex(REG_PHY_MODE_E302); uart_putc(':');
    uart_puthex(REG_INT_PCIE_NVME); uart_putc(':');
    uart_puthex(REG_NVME_EVENT_STATUS); uart_putc(':');
    uart_puthex(REG_PHY_RXPLL_TRIGGER); uart_putc(':');
    uart_puthex(XDATA_REG8V(0xB432)); uart_putc(':');
    uart_puthex(REG_SYS_CTRL_E765); uart_putc(':');
    uart_puthex(usb4_int_seen); uart_putc(':');
    uart_puthex(pd_cc_timeout); uart_puts("]\n");
    if (((REG_PHY_MODE_E302 >> 4) & 3) >= 2) uart_puts("[*** USB4 TRAINED ***]\n");
    uart_puts("[M2 766=");      uart_puthex(XDATA_REG8V(0x0766));
    uart_puts(" 6f1=");         uart_puthex(XDATA_REG8V(0x06F1));
    uart_puts(" 72d=");         uart_puthex(XDATA_REG8V(0x072D));
    uart_puts(" 72b=");         uart_puthex(XDATA_REG8V(0x072B));
    uart_puts(" 72c=");         uart_puthex(XDATA_REG8V(0x072C));
    uart_puts(" cb10s=");       uart_puthex(cb10_seen);
    uart_puts("]\n");
    uart_puts("[LANE c8ff=");   uart_puthex(REG_LANE_RATE_C8FF);
    uart_puts(" aa0=");          uart_puthex(XDATA_REG8V(0x0AA0));
    uart_puts(" sba0=");         uart_puthex(SB_RD(0xA0));
    uart_puts(" sba1=");         uart_puthex(SB_RD(0xA1));
    uart_puts(" e710=");         uart_puthex(REG_LINK_WIDTH_E710);
    uart_puts(" ca06=");         uart_puthex(REG_CPU_MODE_NEXT);
    uart_puts(" a9e=");          uart_puthex(XDATA_REG8V(0x0A9E));
    uart_puts(" a9f=");          uart_puthex(XDATA_REG8V(0x0A9F));
    uart_puts(" b430=");         uart_puthex(XDATA_REG8V(0xB430));
    uart_puts(" b432=");         uart_puthex(XDATA_REG8V(0xB432));
    uart_puts(" e751=");         uart_puthex(REG_PHY_POLL_E751);
    uart_puts(" e763=");         uart_puthex(REG_PHY_RXPLL_TRIGGER);
    uart_puts(" e765=");         uart_puthex(REG_SYS_CTRL_E765);
    uart_puts(" e764=");         uart_puthex(REG_PHY_TIMER_CTRL_E764);
    uart_puts("]\n");
    uart_puts("[S4 6ed=");       uart_puthex(XDATA_REG8V(0x06ED));
    uart_puts(" 75b=");          uart_puthex(XDATA_REG8V(0x075B));
    uart_puts(" 759=");          uart_puthex(XDATA_REG8V(0x0759));
    uart_puts(" 75c=");          uart_puthex(XDATA_REG8V(0x075C));
    uart_puts(" 75a=");          uart_puthex(XDATA_REG8V(0x075A));
    uart_puts(" 765=");          uart_puthex(XDATA_REG8V(0x0765));
    uart_puts(" 768=");          uart_puthex(XDATA_REG8V(0x0768));
    uart_puts(" 758=");          uart_puthex(XDATA_REG8V(0x0758));
    uart_puts(" 777=");          uart_puthex(XDATA_REG8V(0x0777));
    uart_puts(" sb18=");         uart_puthex(SB_RD(0x18));
    uart_puts(" sb28=");         uart_puthex(SB_RD(0x28));
    uart_puts(" cce4=");         uart_puthex(REG_LANE_WIDTH_CNT_HI);
    uart_puts("]\n");
    { uint8_t k; uart_puts("[P2 775="); uart_puthex(XDATA_REG8V(0x0775));
      uart_puts(" 752="); uart_puthex(XDATA_REG8V(0x0752));
      uart_puts(" sb19="); uart_puthex(SB_RD(0x19));
      uart_puts(" 2a=");
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + k)));
      uart_puts(" 29=");
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2900u + k)));
      uart_puts("]\n"); }
    if (!pd_seen && kicks < 60) {
      pd_drive_hard_reset();
      kicks++;
    }
  }
}
