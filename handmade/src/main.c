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

/* ISR-UART MUTE (2026-06-22 timing experiment): when set, uart_putc returns IMMEDIATELY (no FIFO
 * spin, no THR write). Set at INT1/INT0 ISR entry, cleared at exit, so EVERY UART call in the ISR
 * call tree (af38 [a5]/[cmd], eaac, c105, EC06, sb_router [CLW]/[SFR]/Lane-Bonded) costs ~0 cycles
 * instead of blocking ~250 chars * char-time per CL-walk descriptor exchange. Tests whether the
 * SDCC-ISR's heavy blocking-UART critical path misses the host's lane-bond CL-walk response window.
 * Default 0; set 1 in main() to enable for the experiment. */
volatile __xdata uint8_t isr_uart_mute_arm;   /* master enable (set in main) */
volatile __xdata uint8_t isr_in_ctx;          /* live: inside an ISR right now (mute UART) */
/* Blocking UART putc with a bounded spin so a wedged UART can't hang the CPU. */
void uart_putc(uint8_t ch) { uint16_t g = 0; if (isr_in_ctx) return; while (!REG_UART_TFBF && ++g < 0x8000) { } REG_UART_THR = ch; }
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
#include "ring_log.h"
#include "usb4.h"
#include "sb_router.h"
#include "usb4_irq.h"
#include "boot_phy.h"
#include "usb4_connect.h"
#include "usb4_lanebond.h"
#include "cm_tunnel.h"

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

/* Stock bank0_d127: initialize the PCIe TLP DMA ring and CPU-link event path.
 * Stock runs this immediately after pcie_tunnel_adapter_enable_b401() during boot. */
static void pcie_dma_ring_init_d127(void) {
  REG_PCIE_DMA_SIZE_A = 0x08;
  REG_PCIE_DMA_SIZE_B = 0x00;
  REG_PCIE_DMA_SIZE_C = 0x08;
  REG_PCIE_DMA_SIZE_D = 0x08;
  REG_PCIE_DMA_BUF_A = 0x08;
  REG_PCIE_DMA_BUF_B = 0x20;
  REG_PCIE_DMA_BUF_C = 0x08;
  REG_PCIE_DMA_BUF_D = 0x28;
  REG_PCIE_DMA_CFG_50 = 0x00;
  REG_PCIE_DOORBELL_CMD = 0x00;
  REG_CPU_LINK_CEF3 = CPU_LINK_CEF3_ACTIVE;
  REG_CPU_LINK_CEF2 = CPU_LINK_CEF2_READY;
  REG_CPU_LINK_CEF0 = (uint8_t)(REG_CPU_LINK_CEF0 & 0xF7);
  REG_CPU_LINK_CEEF = (uint8_t)(REG_CPU_LINK_CEEF & 0x7F);
  REG_INT_DMA_CTRL = (uint8_t)((REG_INT_DMA_CTRL & 0xFB) | 0x04);
  REG_PCIE_DMA_CTRL_B281 = (uint8_t)((REG_PCIE_DMA_CTRL_B281 & 0xCF) | 0x10);
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
        DESC_BUF[vi] = XDATA_REG8(addr + vi);
        if (bank) DPX = 0x00;
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

      if ((REG_USB_SETUP_WIDX_L & 0x03) == 0) {
        /* Single TLP: fire with data from DESC_BUF[8-11] (LE). */
        if (fmt_type & PCIE_FMT_HAS_DATA) {
          REG_PCIE_DATA_3 = DESC_BUF[8];
          REG_PCIE_DATA_2 = DESC_BUF[9];
          REG_PCIE_DATA_1 = DESC_BUF[10];
          REG_PCIE_DATA_0 = DESC_BUF[11];
        }
        REG_PCIE_TLP_CTRL = 0x01;
        REG_PCIE_STATUS  = PCIE_STATUS_ERROR;
        REG_PCIE_STATUS  = PCIE_STATUS_COMPLETE;
        REG_PCIE_STATUS  = PCIE_STATUS_KICK;
        REG_PCIE_TRIGGER = PCIE_TRIGGER_EXEC;
      } else {
        /* Streaming: read dword count from the value field (LE). */
        dma_dwords = ((uint32_t)DESC_BUF[11] << 24) | ((uint32_t)DESC_BUF[10] << 16) |
                     ((uint32_t)DESC_BUF[9] << 8) | DESC_BUF[8];
        if (dma_dwords > 0) {
          if ((REG_USB_SETUP_WIDX_L & 0x03) == 1) {
            // host to device, we arm the OUT endpoint
            REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
          }
          if ((REG_USB_SETUP_WIDX_L & 0x03) == 2) {
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

/* Steady-state connect-service re-run toggle (2026-06-22). Stock re-runs c7a5->d7cd->c35b every
 * superloop iteration; handmade lacked it (emulate/diff_trace.py --steady differential). Default 1
 * (enabled, byte-true to stock). Set 0 in main() for a HW A/B if the bond regresses. */
static volatile uint8_t __xdata reservice_enable;

static __xdata uint8_t wseq_lwsig = 0xFF, wseq_lwseen, wseq_lwbud;
static __xdata uint8_t wseq_sig_cur, wseq_e710, wseq_p1201, wseq_p1203, wseq_p1407;
static __xdata uint8_t wseq_sa0, wseq_sa1, wseq_h2, wseq_h3, wseq_w1e, wseq_w1f, wseq_l2a, wseq_l2b;
static void debug_wseq_tick(void) {
#if 0
  if (wseq_lwbud == 0) return;
  wseq_e710 = REG_LINK_WIDTH_E710;
  wseq_p1201 = P1_RD(0x1201);
  wseq_p1203 = P1_RD(0x1203);
  wseq_p1407 = P1_RD(0x1407);
  wseq_sa0 = SB_RD(0xA0);
  wseq_sa1 = SB_RD(0xA1);
  wseq_h2 = u4_host_desc[0x02];
  wseq_h3 = u4_host_desc[0x03];
  wseq_w1e = u4_work_buf[0x1E];
  wseq_w1f = u4_work_buf[0x1F];
  wseq_l2a = lb_loop2_state[0];
  wseq_l2b = lb_loop2_state[1];
  wseq_sig_cur = (uint8_t)(wseq_e710 & 0x0F);
  wseq_sig_cur ^= (uint8_t)(wseq_p1201 << 4);
  wseq_sig_cur ^= wseq_p1407;
  wseq_sig_cur ^= wseq_p1203;
  wseq_sig_cur ^= wseq_h2;
  wseq_sig_cur ^= (uint8_t)(wseq_h3 << 1);
  wseq_sig_cur ^= wseq_w1e;
  wseq_sig_cur ^= (uint8_t)(wseq_w1f << 1);
  wseq_sig_cur ^= wseq_l2a;
  wseq_sig_cur ^= (uint8_t)(wseq_l2b << 1);
  wseq_sig_cur ^= (uint8_t)(wseq_sa0 << 2);
  wseq_sig_cur ^= (uint8_t)(wseq_sa1 << 5);
  if (wseq_lwseen && wseq_sig_cur == wseq_lwsig) return;
  wseq_lwseen = 1;
  wseq_lwsig = wseq_sig_cur;
  wseq_lwbud--;
  uart_puts("\r\n[wseq E710="); uart_puthex(wseq_e710);
  uart_puts(" 1201="); uart_puthex(wseq_p1201);
  uart_puts(" 1203="); uart_puthex(wseq_p1203);
  uart_puts(" 1407="); uart_puthex(wseq_p1407);
  uart_puts(" 819="); uart_puthex(XDATA_REG8V(0x0819)); uart_puthex(XDATA_REG8V(0x081A));
  uart_puts(" 77A="); uart_puthex(XDATA_REG8V(0x077A));
  uart_puts(" 1603="); uart_puthex(P1_RD(0x1603));
  uart_puts(" CA06="); uart_puthex(XDATA_REG8V(0xCA06));
  uart_puts(" E302="); uart_puthex(REG_PHY_MODE_E302);
  uart_puts(" A0="); uart_puthex(wseq_sa0); uart_puts(" A1="); uart_puthex(wseq_sa1);
  uart_puts(" 9E="); uart_puthex(SB_RD(0x9E)); uart_puts(" 66="); uart_puthex(SB_RD(0x66));
  uart_puts(" 2a="); uart_puthex(P1_RD(0x2A02)); uart_puthex(P1_RD(0x2A03));
  uart_puthex(P1_RD(0x2A04)); uart_puthex(P1_RD(0x2A05));
  uart_puts(" 2b="); uart_puthex(P1_RD(0x2B02)); uart_puthex(P1_RD(0x2B03));
  uart_puthex(P1_RD(0x2B04)); uart_puthex(P1_RD(0x2B05));
  uart_puts(" 6A="); uart_puthex(SB_RD(0x6A)); uart_puthex(SB_RD(0x6B));
  uart_puthex(SB_RD(0x6C)); uart_puthex(SB_RD(0x6D));
  uart_puts(" w1C="); uart_puthex(u4_work_buf[0x1C]); uart_puthex(u4_work_buf[0x1D]);
  uart_puthex(wseq_w1e); uart_puthex(wseq_w1f);
  uart_puts(" 779="); uart_puthex(wseq_h2); uart_puthex(wseq_h3);
  uart_puts(" 719="); uart_puthex(e461_inflight_token);
  uart_puts(" 75b="); uart_puthex(wseq_l2a); uart_puthex(wseq_l2b);
  uart_putc(']');
#endif
}

void int0_isr(void) __interrupt(0) {
  uint8_t int0_type = REG_INT_USB_STATUS;
  if (isr_uart_mute_arm) isr_in_ctx = 1;
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
  isr_in_ctx = 0;
}

/* INT1 / EX1: the PD / USB4 / system interrupt aggregate (C806/C80A/EC06). */
void int1_isr(void) __interrupt(1) {
  uint8_t saved_dpx = DPX;
  if (isr_uart_mute_arm) isr_in_ctx = 1;
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
  isr_in_ctx = 0;
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
  // bank0_8d77 (stock cap-apply) UNCONDITIONALLY seeds 0x09F4=3 at its head (CODE:8d77, BEFORE any
  // SPI-blob check). Handmade previously dropped this -> 0x09F4 read uninit 0x55 -> usb4_connect_u4's
  // DP-alt sub-case (0x09F4==3) never ran -> 0x09FA latched 0x07 instead of stock's 0x01 (HW-confirmed
  // stock@commit: 0x09F4=03 0x07BE=01 0x09FA=01). 0x09F4=3 = DP-alt mode default (stock 8d77 head).
  XDATA_REG8V(0x09F4) = 3;
  XDATA_REG8V(0x09F5) = 1; XDATA_REG8V(0x09F6) = 1;   /* cap20g_gate1=1 (stock default): 2-lane. [SINGLE-LANE experiment 2026-06-20: gate1=0 gives a STABLE 1-lane link (La0 CL0) but STILL no router enum — c8c7 config-space is 2-lane-width-event-gated, so 1-lane can't serve the Router-CS read either. The 2-lane bond is required. Reverted.] */
  XDATA_REG8V(0x09F7) = 3; XDATA_REG8V(0x09F8) = 1; XDATA_REG8V(0x09FB) = 3;
  // 0x0A57/0x0A58 = device PID-low / bcdDevice-hi. Stock relies on the boot env (SPI-flash shadow)
  // pre-loading these; there is no firmware writer. Without them they stay uninit 0x55, so the SB
  // connect descriptor TX becomes 0104 5555 instead of 0104 6324 and the host never escalates
  // SB[0x18] 05->63 to assign the route-ID (no lane bond, no GPU). Values from the stock wire trace.
  XDATA_REG8V(0x0A57) = 0x63;
  XDATA_REG8V(0x0A58) = 0x24;

  /* STEP 1: seed PCIe-tunnel adapter-config inputs (stock bank0_8d77 SPI-blob path; handmade has no
   * SPI-shadow load so 0x0A52-0x0A55 read the uninit 0x55 flash-buffer poison -> B410-B42B advertised
   * garbage). Values captured from STOCK on this board via app/patch_stock_adaptercfg.py:
   *   [ACFG A52=21 A53=1B A54=63 A55=24 7E=A5 | B410=1B B411=21 B412=24 B413=63 ...]
   * (7E=A5 == valid SPI blob on stock; A52/A53=21/1B are REAL blob bytes, NOT the PID fallback). The
   * boot_phy.h c8db port maps lo=[0A53]->B410, hi=[0A52]->B411, mode=[0A54]->B413, cred=[0A55]->B412. */
  XDATA_REG8V(0x0A52) = 0x21;  /* B411/B41B hi   (stock SPI[0x7074]) */
  XDATA_REG8V(0x0A53) = 0x1B;  /* B410/B41A lo   (stock SPI[0x7075]) */
  XDATA_REG8V(0x0A54) = 0x63;  /* B413/B419 mode (stock SPI[0x7076]) */
  XDATA_REG8V(0x0A55) = 0x24;  /* B412/B418 cred (stock SPI[0x7077]) */

  // Stock boot_hw_init_main step 6k: enable the USB4 router's PCIe-down tunnel adapter (B401 MASTER
  // EN + B410-B42B cfg + B298 TLP-routing) so the host CM can discover a PCIe-down adapter and tunnel
  // PCIe to the GPU. Stock runs this at boot (before the mode decision); pcie_power_on (after the
  // bond) is the runtime tunnel-up that deasserts PERST and completes the link — keep both.
  pcie_tunnel_adapter_enable_b401();
  pcie_dma_ring_init_d127();

  // Stock boot_hw_init_main "main step 6" runs bank0_de16 (= handmade u4lb_e96c) UNCONDITIONALLY at
  // boot, in sequence after bank0_8d77 and before the PHY-ready gate / mode decision. It zeroes the
  // SB connect-service state (0x0B30-0x0B33), configures the PHY-DMA/timer (CD30/CD31, CC2A/CC2C/CC2D,
  // addr=0x00C7) and calls sb_channel_connect_service() to program the SB PHY connect path (C620/C655/
  // C65A) BEFORE any host/connect event. Stock then re-runs sb_channel_connect_service every main-loop
  // iteration. EMULATION DIFFERENTIAL (2026-06-22, emulate/diff_trace.py): handmade DEFINED u4lb_e96c
  // but NEVER CALLED it -> C620/C655/C65A and 92F7 (lane-advance) accessed 0x in handmade vs 43500x in
  // stock; the entire boot-time SB-connect path was absent. Port stock's unconditional boot call here.
  u4lb_e96c();

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

  // bank1_eef9 (stock boot_hw_init_main step, Phase-2 ❌ omitted in handmade): enable the bank1/USB4
  // INT group (INT_ENABLE bit6). The EC06/router-op (control-channel) INT group may be gated by this
  // bit — without it the host's post-bond Router-CS read never reaches the device mailbox (EC06=0).
  REG_INT_ENABLE = (uint8_t)((REG_INT_ENABLE & 0xBF) | 0x40);

  // d894 boot tail. Disassembly says:
  // P1[0000]&=~2; C809|=2; b031(); P1[1602]&=~1; P1[1603]=1; P1[1602]&=~2; P1[1603]=2; P1[121E]|=1.
  // The old 0x1604 notes were decompiler artefacts; the byte listing addresses this as 0x1602/0x1603.
  P1_WR(0x0000, (uint8_t)(P1_RD(0x0000) & 0xFD));
  REG_INT_CTRL = (uint8_t)((REG_INT_CTRL & 0xFD) | 0x02);
  u4lb_b031_transport_reinit(0);
  P1_WR(0x1602, (uint8_t)(P1_RD(0x1602) & 0xFE));
  P1_WR(0x1603, 0x01);
  P1_WR(0x1602, (uint8_t)(P1_RD(0x1602) & 0xFD));
  P1_WR(0x1603, 0x02);
  P1_WR(0x121E, (uint8_t)(P1_RD(0x121E) | 0x01));

  // USB4 CM router-op RX-enable so the host's router-op queries are received.
  if (XDATA_REG8V(0x09F9) & 0x81) {
    usb4_routerop_init();
    uart_puts("[U4rop ec00="); uart_puthex(XDATA_REG8V(0xEC00));
    uart_puts(" ec04=");        uart_puthex(REG_NVME_EVENT_ACK);
    uart_puts(" c807=");        uart_puthex(REG_INT_DMA_CTRL);
    uart_puts(" ea88=");        uart_puthex(XDATA_REG8V(0xEA88)); uart_puts("]\n");
  }

  // ROOT (traced 2026-06-19): config read needs the 0x1200 config-space, set up by c8c7<-a522<-C105<-
  // C80A.4, which only fires when the in-band transport/control-adapter COMES UP. Handmade reaches
  // lane-CL0 but the transport layer never fully comes up, so the chain never starts. 0x1201 is HW-set
  // by c8c7 only (not firmware-writable; confirmed: direct writes + d894-at-boot both fail to make it
  // stick). See project_inband_routercs_read_wall.md.

  // USB-mode fork: skip the USB3 device engine in USB4 mode. The GPU path is the USB4 PCIe tunnel;
  // tinygrad should use the lspci-backed AMD path, not the ADD1:0001 USB transport.
  if (XDATA_REG8V(0x09F9) & 0x83) {
    uart_puts("[USB4 mode: skip USB3 device bring-up]\n");
  } else {
    // Bring USB up. force_usb2=0: try SS first, fall back via LINK_EVENT.
    usb_init_controller(0);
  }

  // Zero only the USB4 router-op mailbox scratch. 0x0B12+ holds SB transport state that b031 seeds.
  { uint8_t z; for (z = 0; z <= (0x0B11 - 0x0B02); z++) XDATA_REG8V(0x0B02 + z) = 0; }
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
  sb_con_print_budget = 0;
  af38_s3_budget = 0;
  af38_s5_budget = 0;
  c105_fire_bdg = 8;
  u4lb_edf5_print_budget = 0;
  u4lb_s5_print_budget = 0;
  u4lb_clv_budget = 0; u4lb_clv_seen = 0; u4lb_clv_last = 0;
  isr_dbg_budget = 0; isr_dbg_budget2 = 0; isr_dbg_budget3 = 0;
  isr_uart_mute_arm = 0;  /* ISR-UART MUTE TOOL (2026-06-22 timing experiment): when 1, silences ALL
                           * ISR-path UART (no FIFO blocking) so the INT1 connect/lane-bond critical
                           * path runs at near-stock cycle cost. HW-TESTED 1->0: removing ~185-1477ms
                           * of per-bond-window ISR-UART blocking (a single [a5] dump = ~2.7-21.7ms in
                           * one ISR call) is BOND-SAFE (CL0/CL0 deterministic) but route=1 STILL -110
                           * (kprobe tb_cfg_get_upstream_port=0xffffff92, 2 clean runs) -> GPU absent.
                           * TIMING RULED OUT as the route=1 wall. Default 0 (keep connect-window UART
                           * diagnostics); set 1 to re-run the silent-ISR timing A/B. */
  reservice_enable = 1;   /* steady-state c7a5->d7cd->c35b re-run (byte-true to stock; A/B toggle) */
  d855_dbg_budget = 8;
  wseq_lwsig = 0xFF; wseq_lwseen = 0; wseq_lwbud = 0;
  tup_e52d_done = 0; tup_dbg_budget = 4; tup_b031_enable = 0; tup_arm_mode = 0;
  /* ring_log.h init (crt0 does NOT zero XDATA). Disarmed until the connect edge arms it. */
  rl_armed = 0; rl_dumped = 0; rl_idx = 0; rl_wrapped = 0; rl_seq = 0; rl_lean = 0;
  u4lb_s5_seen = 0; u4lb_s5_last759 = 0; u4lb_s5_last75b = 0;
  u4lb_s5_lasta0 = 0; u4lb_s5_last775 = 0; u4lb_s5_lastaf38 = 0; u4lb_s5_lasttx = 0;

  // CM PCIe-tunnel state cells (cm_tunnel.h). crt0 does NOT zero XDATA; seed to idle.
  XDATA_REG8V(0x0002) = 0;
  XDATA_REG8V(0x044B) = 0; XDATA_REG8V(0x044C) = 0; XDATA_REG8V(0x044D) = 0;
  XDATA_REG8V(0x0464) = 0; XDATA_REG8V(0x0579) = 0;
  XDATA_REG8V(0x05A6) = 0; XDATA_REG8V(0x05A7) = 0; XDATA_REG8V(0x05AC) = 0; XDATA_REG8V(0x05AD) = 0;
  XDATA_REG8V(0x05B4) = 0;
  XDATA_REG8V(0x06E5) = 0; XDATA_REG8V(0x06E6) = 0; XDATA_REG8V(0x06E7) = 0;
  XDATA_REG8V(0x06E8) = 0; XDATA_REG8V(0x06EB) = 0;
  XDATA_REG8V(0x07EF) = 0; XDATA_REG8V(0x0A59) = 0;
  XDATA_REG8V(0x0B39) = 0; XDATA_REG8V(0x0B3A) = 0; XDATA_REG8V(0x0B3B) = 0; XDATA_REG8V(0x0B3C) = 0;

  // enable interrupts (EX1 = PD/USB4 INT1)
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  i2c_init();
  ina231_init();

  // Milestone 1: prompt the host into PD with periodic Hard Resets until it engages
  // (pd_seen is set by the PD-RX ISR on the first received message), then just observe.
  uint8_t kicks = 0;
  uint8_t sb_diag_count = 0;
  while (1) {
    /* ===== CM PCIe-tunnel super-loop REMOVED 2026-06-18 =====
     * The cm_arm_c00d()-based CM step machine (the 9037 layer) was PROVEN this session to be the
     * device's NVMe-storage enumeration, NOT the GPU/PCIeDn path. Worse, a HW bond-vs-nobond runtime
     * diff showed it is ACTIVELY HARMFUL: when its bond-complete arm fired on the SAME super-loop pass
     * as the tunnel-up block below (both gate on SB[0xA0]&0xF==2 && SB[0xA1]&0xF==2), cm_arm_c00d's
     * PERST-assert(B480) + C659-clear + boot_phy_d436_width re-drive collided with pcie_power_on()'s
     * PERST-deassert + LTSSM-train, leaving the device<->GPU LTSSM stuck at 0x01 -> [PCIe timeout]
     * (the "cycle-variable" failure was THIS race, not the lane bond — the bond is deterministic).
     * Removing it lets pcie_power_on() train the downstream link to 0x78 CONNECTED every cycle. */

    /* RING-LOG ARM/STOP harness (2026-06-22): high-frequency XDATA ring capture of the real-time
     * connect->bond->1407.0 timeline (ring_log.h). Arm on the connect edge (0x06EC 0->1); stop+dump
     * when the bond commits (A=A1=0x02) OR P1[0x1407].0 fires OR a per-window iteration budget
     * expires (timeout). No UART during the window — only the post-window dump. */
    { static __xdata uint8_t rl_lastconn = 0; static __xdata uint32_t rl_winto = 0;
      static __xdata uint8_t rl_bondhold = 0;
      uint8_t conn = XDATA_REG8V(0x06EC) ? 1 : 0;
      /* Arm on the FIRST connect of this boot (rl_dumped guards a re-arm after we've dumped, so the
       * ring isn't wiped before the post-window UART trace prints). */
      if (conn && !rl_lastconn && !rl_dumped) { rl_arm(); rl_winto = 0; rl_bondhold = 0; }
      rl_lastconn = conn;
      if (rl_armed) {
        uint8_t a0 = SB_RD(0xA0), a1 = SB_RD(0xA1);
        uint8_t p1407 = P1_RD(0x1407);
        IE &= (uint8_t)~IE_EA;  /* serialize ring access vs the INT1 rl_log calls */
        rl_log(2);              /* sample every super-loop pass (change-gated inside) */
        IE |= IE_EA;
        if (p1407 & 0x01) {     /* THE target: width event fired -> dump immediately */
          rl_stop_dump();
        } else if ((a0 & 0x0F) == 0x02 && (a1 & 0x0F) == 0x02) {
          /* Bond reached. Hold a short window AFTER the bond so the ring captures whether 1407.0
           * fires in the post-bond moment (stock raises it AT/just-after the bond), then dump. */
          if (++rl_bondhold >= 200) rl_stop_dump();
        } else if (++rl_winto >= 0x40000UL) {   /* generous window timeout (no bond seen) */
          rl_stop_dump();
        }
      }
    }

    /* FSM-advance, run first and delay-free so it tracks the connect edge tightly. */
    if ((XDATA_REG8V(0x09F9) & 0x83) && XDATA_REG8V(0x06EC)) {
      uint8_t fsm_before = XDATA_REG8V(0x06ED);
      IE &= (uint8_t)~IE_EA;
      if (isr_dbg_budget && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget--; uart_putc('c'); }
      sb_cb10_lane_advance();
      if (isr_dbg_budget2 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget2--; uart_putc('n'); }
      if (isr_dbg_budget3 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget3--; uart_putc('k'); }
      /* Stock cb10 dispatches e672 only while 0x06ED is nonzero. When state 5 finishes it sets
       * 0x06ED=0 and leaves the host to raise the lane-bond/CL0 events; re-arming state 3 here
       * restarts the CL walk before SB[A0]/SB[A1] can become 0x02. */
      /* Stock cb10 throttle (raw cb8d-cbb3): ee57 returns live CCE4:CCE5 in R6:R7. The walker
       * advances when saved 0x076A:0x076B minus the live counter is >= 3; after e672, stock
       * samples ee57 again and stores the fresh live value. */
      if (XDATA_REG8V(0x06ED) != 0) {
        uint16_t cur = u4lb_ee57();
        uint8_t cur_hi = (uint8_t)(cur >> 8), cur_lo = (uint8_t)cur;
        uint8_t snap_hi = lb_walk_throttle_snap_hi, snap_lo = lb_walk_throttle_snap_lo;  /* 0x076A:0x076B */
        uint8_t d_lo = (uint8_t)(snap_lo - cur_lo);
        uint8_t d_hi = (uint8_t)(snap_hi - cur_hi - (uint8_t)((snap_lo < cur_lo) ? 1 : 0));
        if (d_hi >= (uint8_t)((d_lo < 3) ? 1 : 0)) {
          u4lb_e672();
          cur = u4lb_ee57();
          lb_walk_throttle_snap_hi = (uint8_t)(cur >> 8);
          lb_walk_throttle_snap_lo = (uint8_t)cur;
        }
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

    // STEADY-STATE CONNECT-SERVICE RE-RUN (2026-06-22, emulate/diff_trace.py --steady differential).
    // Stock calls bank0_c7a5 -> bank0_d7cd -> sb_channel_connect_service (c35b: C620/C655/C65A) EVERY
    // superloop iteration, UNCONDITIONALLY, recomputing the connect-state 0x0B30 from the live B481
    // lane-count + CD31 timer. handmade ran it only ONCE at boot (u4lb_e96c). The pure-emulation
    // differential exposed this as the next steady-state execution-path gap: stock re-touches
    // C620/C655/C65A x1624 in a 15% tail, handmade 0x. The hypothesis under test: the link/connect
    // state must be CONTINUOUSLY re-driven for the 1->2 width transition (P1[0x1407].0) to be
    // re-evaluated. Gated EA-off (matching stock's c7a5 EA=0/EA=1 bracket) + behind a runtime toggle
    // (reservice_enable, default 1) so it can be disabled for a HW A/B if the bond regresses (Abr2).
    if (reservice_enable) {
      IE &= (uint8_t)~IE_EA;
      sb_connect_service_reservice_d7cd();
      IE |= IE_EA;
    }

    // SB router-op responder, deferred off the INT1 ISR to keep the stack shallow.
    if (sb_pend_int_pending || (SB_RD(0x26) & 0x02)) {
      sb_pend_int_pending = 0;
      IE &= (uint8_t)~IE_EA;
      sb_a5d8_pend_int();
      if (SB_RD(0x26) & 0x02) SB_WR(0x26, 0x02);   // W1C SB[0x26].1 after response, like a066
      IE |= IE_EA;
    }

    // Deferred bank0_8a89 drive: run once, only after the FSM has clearly stalled.
    // (batch-5 HW test: disabling this NON-STOCK fallback was NEUTRAL — not the bond perturbation.)
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

    // LINK-MODE TRANSITION tracker (2026-06-20): does handmade's E302/CA06 TRANSITION at the bond
    // (stock 0x97->0x83 / 0x61->0x01) or sit STUCK at the final value? The HW raises C80A.4/the width
    // event ON the transition. Budgeted + change-gated so it can't flood the bond.
    { static __xdata uint8_t le302 = 0xFF, lca06 = 0xFF, lkbud = 90;
      uint8_t e = REG_PHY_MODE_E302, c = XDATA_REG8V(0xCA06);
      if ((e != le302 || c != lca06) && lkbud) {
        le302 = e; lca06 = c; lkbud--;
        uart_puts("[Lk e302="); uart_puthex(e); uart_puts(" ca06="); uart_puthex(c);
        uart_puts(" c80a="); uart_puthex(REG_INT_PCIE_NVME);
        uart_puts(" e7e3="); uart_puthex(XDATA_REG8V(0xE7E3));   // PHY link-ctrl latch (dd42; 0xCC=mode1/0x30=mode4)
        uart_puts(" af1="); uart_puthex(u4_connect_gate);        // u4_connect_gate (.5 gates E7E3)
        uart_puts(" A0="); uart_puthex(SB_RD(0xA0)); uart_putc(']');
      } }

    debug_wseq_tick();

    // PER-LANE PHY LOCK time-series: RESULT (2026-06-22, agent14, HW-captured both fw + stock RE).
    // The decisive unmeasured mechanism was per-lane PLL/CDR LOCK TIMING during training (does
    // lane0 lock before lane1 = staggered, raising the HW 1->2 width step?). MEASURED, both sides:
    //   handmade in-connect [c2@..]:  pre-bond E2E2 -> rstpll E4E4 -> ec51 6464 -> bond F4F4
    //   stock   patch_stock_cfg [bk]: validate E4E4; a66 pre-bond plA=plB=E2/50; at-bond plA=plB=F4/50
    // => laneA == laneB at EVERY sample, in BOTH stock and handmade — the lanes lock SIMULTANEOUSLY,
    // no stagger, byte-identical timing. RE confirms: stock u4lb_state4_b0b4 (CODE_BANK1::b0b4) arms
    // lane0 then lane1 BACK-TO-BACK (no inter-lane wait/poll); rst_rx_pll + bank1_b8db validate BOTH
    // lanes together. There is NO firmware per-lane enable stagger to port. VERDICT (B): the stagger
    // is analog/host-driven, NOT firmware-reachable. The super-loop sampler only ever caught the
    // pre-bond E2E2 (the lock transition happens inside the connect ISR burst); the in-connect
    // [c2@..] diags in usb4_lanebond.h are the live per-lane lock probe — removed the super-loop
    // duplicate to save IRAM/code.

    // *** ROUTE=1 RESPONDER ARM HARNESS (2026-06-22 host-ftrace lever — RESULT: wall confirmed) ***
    // At the STABLE bond (A=0202) the host posts a route=1 TB_CFG read (4 retries / ~414ms, port held
    // UP — host-ftrace + kretprobe oracle confirmed). The device's in-band config responder is dead
    // (-> tb_cfg_get_upstream_port = -110 -> tb_switch_alloc fails -> no GPU). The responder is brought
    // up by stock e52d's b031 transport reinit, armed by a522->e06b(1)->sb_link_reinit_gate->P1[0x0109]
    // (the WIDTH event handmade never gets). This harness tested firing it device-side:
    //   mode 2 (set P1[0x0109] + run e52d incl. b031): e52d RUNS ([TUP! 109=73]) but b031 POISONS the
    //          SB read engine (e5b0 descriptor reset) -> device super-loop HANGS on the next SB read ->
    //          link drops to Training/Bonding, host sees device-disc. EC06 never fires.
    //   mode 3 (re-run e56f router-op RX engine, bond-safe, no descriptor engine): bond HOLDS but EC06
    //          still 0 (e56f re-arm alone does NOT make the route=1 TLP land; the b031 descriptor
    //          channel is genuinely required — and it's the one that hangs the SB block).
    //   mode 0/1: detect-only baseline — bond HOLDS stable (host CL0/CL0, route=1 still -110).
    // => the route=1 responder CANNOT be brought up device-side at handmade's bond: b031 (the only
    //    thing that arms the in-band control-adapter RX descriptor channel) hangs the SB read engine at
    //    this state. Stock runs b031 only AFTER the host commits (P1[0x0109]) + drives the follow-up
    //    descriptor exchange that re-establishes the SB engine — handmade has no such follow-up.
    //    Default = mode 0 (no-op, bond-safe). Set tup_arm_mode=2/3 in init to re-test.
    if (!tup_e52d_done &&
        (SB_RD(0xA0) & 0x0F) == 0x02 && (SB_RD(0xA1) & 0x0F) == 0x02) {  // both lanes CL0
      tup_e52d_done = 1;
      if (tup_arm_mode != 0) {
        sb_link_reinit_gate = 1;
        uart_puts("\r\n[ROUTE1-ARM mode="); uart_puthex(tup_arm_mode); uart_putc(']');
        if (tup_arm_mode == 2) {
          P1_WR(0x0109, (uint8_t)(P1_RD(0x0109) | 0x01));
          tup_e52d_done = 0;                   // let sb_lane_bond_complete_tunnel_up's own one-shot gate it
          sb_lane_bond_complete_tunnel_up();   // full byte-true e52d (b031 gated by tup_b031_enable)
        } else if (tup_arm_mode == 3) {
          usb4_routerop_init();                // minimal RX re-arm (bond-safe; insufficient for EC06)
        } else if (tup_arm_mode == 4) {
          // *** FORCE the PcieTunnel-Enable leg (u4lb_e4ea) at the stable bond — REFUTED 2026-06-22 ***
          // The host withholds 1508.4 (Enable) because route=1 is unanswered (-110). e4ea is the leg
          // the host's Enable request would trigger: it clears B480 PERST bits 0-3 + re-triggers the
          // PHY RXPLL (e9b5) to bring the DOWNSTREAM PCIe link up. Hypothesis: if the link trains,
          // E763.2 fires -> [PcieLinkUp] -> the PCIe-DOWN adapter width event 1407.0 -> cascade,
          // BYPASSING route=1. HW RESULT (force-e4ea agent): the leg runs CLEANLY and BOND-SAFE
          // ([PwrOn][PcieTunnel-Deassert][PcieTunnel-Enable], A0=02 held, no Abr2) but 1407 STAYS 00,
          // 1201 STAYS 00, E763 STAYS 00, B480 reads back 0x01 (HW re-asserts PERST). The downstream
          // PCIe link does NOT train from a device-local PERST-clear/RXPLL-retrigger: the link traffic
          // flows THROUGH the host's tunnel which is disabled (host withheld Enable b/c route=1=-110).
          // Host oracle unchanged: tb_tx route=1 seq0-3 (4 retries), tb_rx=0, upport=-110, no GPU.
          // => CONFIRMS the wall is the HW transport-RX (route=1), not reachable by forcing the
          //    tunnel-Enable leg device-side. mode 4 kept disabled (harness for future reference).
          uart_puts("\r\n[FORCE-e4ea pre 1407="); uart_puthex(P1_RD(0x1407));
          uart_puts("\r\n[FORCE-e4ea pre 1407="); uart_puthex(P1_RD(0x1407));
          uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
          uart_puts(" C659="); uart_puthex(REG_PCIE_LANE_CTRL_C659);
          uart_puts(" B480="); uart_puthex(REG_PCIE_PERST_CTRL);
          uart_puts(" E763="); uart_puthex(REG_PHY_RXPLL_TRIGGER); uart_putc(']');
          u4lb_e4ea();
          uart_puts("\r\n[FORCE-e4ea post 1407="); uart_puthex(P1_RD(0x1407));
          uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
          uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
          uart_puts(" E763="); uart_puthex(REG_PHY_RXPLL_TRIGGER);
          uart_puts(" A0="); uart_puthex(SB_RD(0xA0)); uart_putc(']');
        } else if (tup_arm_mode == 5) {
          uart_puts("\r\n[FORCE-d90e pre 1407="); uart_puthex(P1_RD(0x1407));
          uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
          uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
          uart_puts(" B402="); uart_puthex(REG_PCIE_CTRL_B402);
          uart_puts(" C659="); uart_puthex(REG_PCIE_LANE_CTRL_C659);
          uart_putc(']');
          u4lb_d90e_link_phy_reconfig();
          uart_puts("\r\n[FORCE-d90e post 1407="); uart_puthex(P1_RD(0x1407));
          uart_puts(" 1201="); uart_puthex(P1_RD(0x1201));
          uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
          uart_puts(" B402="); uart_puthex(REG_PCIE_CTRL_B402);
          uart_puts(" C659="); uart_puthex(REG_PCIE_LANE_CTRL_C659);
          uart_putc(']');
        }
      }
    }

    // Deferred tunnel-up: run the PCIe bring-up here (uses sleep()/long polls).
    // Stay PENDING until BOTH lanes actually reach CL0 (the bond completes), then fire ONCE.
    // (Was a one-shot that cleared the flag before the CL0 check, so an early arm — before the
    //  bond — deferred AND cleared the pending, and pcie_power_on never ran after Lane Bonded.)
    if (sb_tunnel_up_pending &&
        (SB_RD(0xA0) & 0x0F) == 0x02 && (SB_RD(0xA1) & 0x0F) == 0x02) {  // both lanes CL0
      sb_tunnel_up_pending = 0;
      // *** FULL byte-true e52d TESTED + REFUTED on HW (2026-06-20) ***
      // Stock fires e52d from the a066 ISR PART1 gated on P1[0x0109]&1 (the host's lane-bond-DONE
      // bit) — but handmade NEVER gets that bit set (the host withholds the bond commit / SB[0x66].0).
      // sb_tunnel_up_pending is instead set by the connect-path shortcut (bank0_c9a8, usb4_connect.h),
      // which only fires at CL0 (BEFORE the host commits). The full byte-true e52d (b031 -> e06b ->
      // rom_load -> [pcie block] -> CA60 -> 8a89) was assembled + run HERE in stock order. HW RESULT:
      //   - pcie_power_on ALONE (no b031): bond STABLE (A=0202, PCIe 78 CONNECTED) but 1203=00 c80a=20
      //     EC06=00 — transport does NOT come up; config read still -110.
      //   - +b031 (full e52d, byte-true order): L0:Abr2/L1:Abr2 loop — b031's SB-transport REINIT
      //     re-drives the just-bonded lanes -> device HW raises Abr2; host stuck Training/Bonding.
      //   - b031 ALONE (no pcie): SAME Abr2 loop, and [b031 EN ec06=00] -> b031 does NOT raise EC06.
      // => the disturber is b031's SB-transport reinit (NOT pcie_power_on). Stock avoids it because it
      // runs e52d only AFTER the host commits the bond (0x0109 set); handmade can only run it
      // speculatively pre-commit, where the reinit disturbs the unlocked lanes. The byte-true ORDER
      // does NOT save it. The full e52d port lives in sb_lane_bond_complete_tunnel_up (ISR path,
      // gated 0x0109&1) for when/if the host ever commits. RESTORED here to the stable HELD no-op.
      uart_puts("[TunnelUp:HELD]\n");
      // Upstream-adapter / topology state at the stable bond (just before the host's ROUTER_CS read) —
      // read-only baseline to diff against stock (does the device's HW recognize route 0x1?).
      // Full bond-moment SB lane-adapter state, to diff vs stock a066 (66=01 9E=03 A0=02 A1=02 D4=B7
      // 64=03 2C=F1 2D=F5). The divergent byte is the lane-adapter state the host reads to post cm8.
      uart_puts("[SBbond 66="); uart_puthex(SB_RD(0x66)); uart_puts(" 9E="); uart_puthex(SB_RD(0x9E));
      uart_puts(" A0="); uart_puthex(SB_RD(0xA0)); uart_puts(" A1="); uart_puthex(SB_RD(0xA1));
      uart_puts(" D4="); uart_puthex(SB_RD(0xD4)); uart_puts(" 64="); uart_puthex(SB_RD(0x64));
      uart_puts(" 2C="); uart_puthex(SB_RD(0x2C)); uart_puts(" 2D="); uart_puthex(SB_RD(0x2D)); uart_puts("]\n");
      uart_puts("[UPADP 718="); uart_puthex(XDATA_REG8V(0x0718));
      uart_puts(" 819="); uart_puthex(XDATA_REG8V(0x0819)); uart_puthex(XDATA_REG8V(0x081A));
      uart_puts(" 777="); { uint8_t z; for (z=0; z<6; z++) uart_puthex(XDATA_REG8V(0x0777+z)); }
      uart_puts(" 121E="); uart_puthex(P1_RD(0x121E));
      uart_puts(" 124C="); uart_puthex(P1_RD(0x124C)); uart_puthex(P1_RD(0x124D)); uart_puthex(P1_RD(0x124E));
      uart_puts(" 140="); uart_puthex(P1_RD(0x1403)); uart_puthex(P1_RD(0x1404)); uart_puthex(P1_RD(0x1406));
      uart_puts(" 1802="); uart_puthex(P1_RD(0x1802));
      uart_puts(" e302="); uart_puthex(REG_PHY_MODE_E302);
      // H2 check: 0x09F9 should be 0x87, 0x09FA should have 0x81; c80a_acc shows which adapter events
      // fired (.5=SB bond, .4=secondary adapter). usb4_int_seen .04=EC06 ever fired.
      uart_puts(" 9F9="); uart_puthex(XDATA_REG8V(0x09F9)); uart_puts(" 9FA="); uart_puthex(XDATA_REG8V(0x09FA));
      /* 0x09FA now 0x01 (stock-matched) via the 0x09F4=3 seed (main.c) + plain (no &0x04) connect_u4 write. */
      uart_puts(" 9F4="); uart_puthex(XDATA_REG8V(0x09F4));
      uart_puts(" 9F2="); uart_puthex(XDATA_REG8V(0x09F2)); uart_puthex(XDATA_REG8V(0x09F3));
      uart_puts(" c80a="); uart_puthex(c80a_acc); uart_puts(" seen="); uart_puthex(usb4_int_seen);
      uart_puts(" EC00="); uart_puthex(XDATA_REG8V(0xEC00)); uart_puts(" EA90="); uart_puthex(REG_SYS_CTRL_EA90);
      // width-event link-state (a522 gate: CA06&0x1F==0x10 then C00E&7==0): does handmade reach it?
      uart_puts(" CA06="); uart_puthex(XDATA_REG8V(0xCA06)); uart_puts(" C00E="); uart_puthex(XDATA_REG8V(0xC00E));
      uart_puts(" E710="); uart_puthex(REG_LINK_WIDTH_E710); uart_puts(" 1203="); uart_puthex(P1_RD(0x1203));
      uart_puts("]\n");
      // Full connect-time config-space image (DPX=1 page-1 adapter regs) for the stock diff.
      { uint16_t a;
        uart_puts("\r\n[CFG1200="); for (a=0x1200; a<0x1220; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        uart_puts("\r\n[CFG1400="); for (a=0x1400; a<0x1420; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        uart_puts("\r\n[CFG1500="); for (a=0x1500; a<0x1520; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        uart_puts("\r\n[CFG1600="); for (a=0x1600; a<0x1620; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        uart_puts("\r\n[CFG1800="); for (a=0x1800; a<0x1820; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        // c8c7's SNAPSHOT SOURCE: P1[0x0200..0x020F] (r3_read R3=2). If this is populated post-bond (the
        // lanes ARE trained), c8c7 could passively populate the 0x1200 config-space WITHOUT a PHY re-drive.
        // Also c8c7's gate P1[0x1291].6 + the 0x097e descriptor it parses.
        uart_puts("\r\n[P0200="); for (a=0x0200; a<0x0210; a++) uart_puthex(P1_RD(a)); uart_putc(']');
        uart_puts(" 1291="); uart_puthex(P1_RD(0x1291));
        uart_puts(" 097e="); { uint8_t z; for (z=0; z<0x10; z++) uart_puthex(XDATA_REG8V(0x097b+z)); } }
      // EXPERIMENT (2026-06-20): P0200 IS populated post-bond -> test the c8c7 passive config-space bootstrap.
      // Does c8c7's SNAPSHOT (A) make the P1[0x1201]=2 write STICK, where the refuted direct write did not?
      { uint8_t i;
        for (i = 0; i < 0x10; i++) XDATA_REG8V(0x097b + i) = P1_RD((uint16_t)(0x0200 + i)); // c8c7 (A) snapshot
        (void)P1_RD(0x1291);                            // c206 gate read
        P1_WR(0x1201, 2); P1_WR(0x1202, 1);             // c8c7 commit (P1[0x1291].6 clear branch)
        uart_puts("\r\n[c8c7? 1201="); uart_puthex(P1_RD(0x1201));
        uart_puts(" 1202="); uart_puthex(P1_RD(0x1202));
        uart_puts(" CFG="); { uint16_t a; for (a = 0x1200; a < 0x1210; a++) uart_puthex(P1_RD(a)); } uart_putc(']'); }
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
    uart_puts(" OTP7E=");        uart_puthex(XDATA_REG8V(0x707E));   /* ==0x5A -> board fused, stock applies OTP lane cfg */
    uart_puts("/7A");           uart_puthex(XDATA_REG8V(0x707A)); uart_puthex(XDATA_REG8V(0x707B)); uart_puthex(XDATA_REG8V(0x707D));
    uart_puts(" 86C=");          uart_puthex(XDATA_REG8V(0x086C)); uart_puthex(XDATA_REG8V(0x086D)); uart_puthex(XDATA_REG8V(0x086E)); uart_puthex(XDATA_REG8V(0x086F));
    uart_puts("]\n");
    { uint8_t k; uart_puts("[P2 775="); uart_puthex(XDATA_REG8V(0x0775));
      uart_puts(" 752="); uart_puthex(XDATA_REG8V(0x0752));
      uart_puts(" sb19="); uart_puthex(SB_RD(0x19));
      uart_puts(" 2a=");
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + k)));
      uart_puts(" 29=");
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2900u + k)));
      uart_puts("]\n"); }
    /* The boot kick (PD Hard Reset) IS needed to make the host (re)engage PD — but firing it IMMEDIATELY
       & REPEATEDLY each boot causes the reboot-loop (each hard reset cold-boots the device via VBUS, and
       the fresh boot kicks again before the host responds). FIX: give the host a SETTLE window after each
       (re)attach before kicking — so after ONE kick->reboot->re-attach the host engages PD and no further
       kick fires. (RE: agent a49b7d2d — handmade's initial Rd attach alone doesn't trigger the host's PD;
       one VBUS cycle does, then the settle lets the contract complete in one clean pass.) */
    { static __xdata uint8_t pd_settle = 0;
      if (!pd_seen) {
        if (pd_settle < 12) { pd_settle++; }
        else if (kicks < 8) { pd_drive_hard_reset(); kicks++; pd_settle = 0; }
      }
    }
  }
}
