/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "usb.h"
#include "gpio.h"

__sfr __at(0x93) DPX;   /* DPTR bank select — DPX=1 accesses internal PHY regs */
__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;

#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

// Bounded-blocking: wait for TX-FIFO space (REG_UART_TFBF = bytes free; 0 == full) so rapid
// diagnostic bursts are not silently dropped, but cap the spin so a wedged UART can never hang
// the CPU. (Old version wrote THR unconditionally and dropped chars whenever the FIFO filled,
// which made fast diagnostic sequences look like a CPU stall.)
void uart_putc(uint8_t ch) { uint16_t g = 0; while (!REG_UART_TFBF && ++g < 0x8000) { } REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

#define TIMER1_MODE_HALF_MS     0x04U
/* sleep() runs on Timer1 (CC16-CC19), NOT the CC10-CC13 block. CC10-CC13 is the PHY/PD command
 * mailbox the USB4/PD link engine uses (CC10=(&0xF8)|4 == the subcmd-4 link-up arm, CC11.1 the
 * PHY-done bit); driving it from sleep() on the USB4 boot/super-loop path corrupts the policy
 * engine and collapses E302. Timer1 is the stock standalone timer block (d47f). This function
 * must NEVER touch CC10-CC13. */
static void sleep(uint16_t milliseconds) {
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | TIMER1_MODE_HALF_MS;
  uint16_t threshold = 2*milliseconds;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold & 0xFF;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
  /* Bounded spin so a never-expiring timer can't hang the loop. */
  { uint32_t g = 0; while (!(REG_TIMER1_CSR & TIMER_CSR_EXPIRED) && ++g < 4000000UL); }
}

/* IRAM-HEADROOM FIX (2026-06-11): the firmware's persistent state flags/counters are relocated
 * out of internal-RAM (DSEG) into a free XDATA scratch window at 0x8800.. (above the 0x8000-0x83FF
 * PCIe-read DMA buffer, below DESC_BUF@0x9E00). This shrinks DSEG so crt0's stack (sp=0x72) no
 * longer OVERLAPS live globals, giving the deep nested INT1 SB-router connect path (a066 +
 * bank0_8a89 + connect tail) a much larger, collision-free stack budget. As code grew (b0b4 Round A,
 * connect-present poll) the stack overlapped+corrupted these globals on the connect path and the
 * host's C80A.5 SB-connect silently stopped firing (rate fell 4/8 -> 3/8 -> ~0). Moving them to
 * XDATA both frees DSEG bytes AND removes them from the stack-corruption zone. Each is __at a fixed
 * address (no SDCC auto-XINIT) and explicitly seeded in main()'s boot zero-init block. */
/* LAYOUT FIX 2026-06-12 (audit C2-xdata): is_usb2/dma_dwords were __at(0x0B40-0x0B44), which STOCK uses
 * LIVE (the e869/d47f/ee29 USB4 recovery cells). Moved to the SDCC XSEG (chip-CM 0x0BC0, HW+RE-confirmed
 * free) as plain auto-__xdata. is_usb2 is set by the boot fork before any read; dma_dwords is dead on the
 * USB4 build. Handmade must NOT touch 0x0B40-0x0B44 (left for stock's recovery path). */
static volatile uint8_t __xdata is_usb2;

/* Streaming PCIe state — configured via 0xF0 control message */
static volatile uint32_t __xdata dma_dwords;    /* total dwords remaining for streaming transfer */

/* FIX#4 (GOFWD): super-loop FSM-stall counter (XDATA, off the IRAM stack). bank0_8a89 is reached in
 * STOCK ONLY from INT0 link-events (c9a8 via e94d/e952), NEVER from the C80A.5/a066 SB-router path;
 * the host never raises INT0 for handmade, so driving 8a89 from the connect path is a handmade hack
 * that STARVES the lane-bond FSM and masks EX1 during the host's post-connect window. Defer it: only
 * after the FSM has stalled (no 0x06ED progress) for several iterations with SB[0xA0]/[0xA1] not at
 * CL0. Seeded 0 in the boot zero-init loop (extended to 0x8812). */
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
#include "usb4_connect.h"   /* bank0_8a89 (USB4 lane-MODE engine) + c9a8 link-event dispatcher;
                             * MUST be last: needs boot_phy helpers, usb4_connect_u4, sb_tunnel_up_pending */
/* usb4_lanebond.h: the 0x06ED lane-bond FSM (e672 + cm_conn_routing_setup + eb62) that drives the
 * stock [ConnRout]->[SB P04]->[PcieTunnel-PwrOn]->lane-bond->CL0 sequence from the super-loop.
 * After usb4_connect.h (needs the SB/P1/PR/uart accessors). */
#include "usb4_lanebond.h"

/* Hardware status packet */
typedef struct {
  uint16_t voltage_mv;   /* INA231 bus voltage */
  int16_t  current_ma;   /* INA231 shunt current (signed) */
} hw_status_t;

static void hw_status_read(__xdata hw_status_t *s) {
  /* IRAM-HEADROOM FIX: data-plane (I2C/INA, int0 path) scratch -> XDATA, off the int1 connect path. */
  static __xdata uint16_t shunt_raw, bus_raw;
  shunt_raw = 0; bus_raw = 0;
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
  /* RE-AUDIT (E): preserve E764 bit4 (USB4 LINK-MODE-ENABLE, owned by bank0_8a89) -- do not zero
   * the whole reg. The original literal 0 also clobbered the CM-owned link-mode nibble. */
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
  /* RE-AUDIT (E): E764=0x1C was a FABRICATION (stock NEVER writes E764 as a literal). E764 bit4 is
   * the USB4 LINK-MODE-ENABLE nibble owned by bank0_8a89; the 0x1C literal clobbered it and the
   * CM-owned link-mode state. Removed. (boot_phy d996/e57d does the legit E764 reset-pulse RMW.) */
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
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
      /* Vendor write XDATA via control.  wValue=addr, wIndex low=val.
       * wIndex high byte selects bank (0=normal, 1=PHY/switch via DPX). */
      uint16_t addr = ((uint16_t)wValH << 8) | wValL;
      uint8_t bank = REG_USB_SETUP_WIDX_H;
      uint8_t val = REG_USB_SETUP_WIDX_L;
      if (bank) DPX = bank;
      XDATA_REG8(addr) = val;
      if (bank) DPX = 0x00;
      usb_send_zlp();
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
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF3) {
      /* 0xF3: PCIe power control.
       *   wValue low bit 0 = 0 power off, 1 power on. */
      if (wValL & 0x01) {
        pcie_power_on();
      } else {
        pcie_power_off();
      }
      usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xF0) {
      /* 0xF0 OUT: PCIe TLP engine.
      *   wValue = fmt_type | (byte_enable << 8)
      *   wIndex low[1:0] = mode (0=single TLP, 1=stream write, 2=stream read)
      *   wIndex low[7:2] = dwords per read chunk (0 → 128 for writes)
      *   DATA_OUT: 12 bytes = addr_lo[4 LE] + addr_hi[4 LE] + value[4 LE] */
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


/* TRIAGE budgets (auto __xdata -> chip-CM XSEG 0x0BC0, safe at state-5) for the post-walker hang hunt.
 * Uninitialized (seeded in main) -- XISEG copy may not run in this crt0. */
static volatile uint8_t __xdata isr_dbg_budget, isr_dbg_budget2, isr_dbg_budget3;

void int0_isr(void) __interrupt(0) {
  uint8_t int0_type = REG_INT_USB_STATUS;
  if (int0_type & INT_USB_GATE) {
    uint8_t periph_status;
    periph_status = REG_USB_PERIPH_STATUS;

    if (periph_status & USB_PERIPH_BUS_RESET) {
      /* 0x9101.0 -> 0x91D1 USB-SS / USB4-router link-event demux (stock 0e5b). The stock per-bit
       * sub-decode: .1 -> W1C 0x91D1=2 + e94d (= c9a8(0)) = the USB4 link-event entry that
       * (re)runs the lane-MODE engine bank0_8a89; .3 -> 9c2b, .0 -> c66a, .2 -> e925 (USB-SS
       * power/recovery, left as the generic W1C below). This is candidate #2/#4 of the re-audit:
       * the host-driven re-entry that dynamically drives bank0_8a89 per link event. */
      uint8_t link_event = REG_USB_PHY_CTRL_91D1;
      if (link_event & USB_91D1_FLAG) {            /* 0x91D1.1 */
        REG_USB_PHY_CTRL_91D1 = USB_91D1_FLAG;     /* W1C 0x91D1 = 2 */
        bank0_c9a8(0);                             /* e94d -> c9a8(0) -> bank0_8a89(0) */
        uart_puts("[91D1.1->c9a8]\n");
      } else {
        REG_USB_PHY_CTRL_91D1 = link_event;        /* generic W1C-ack of the remaining events */
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
      /* 0x9101.4 -> 0x9302 USB4-router link-event demux (stock 0e5b). Sub-decode: .3 -> e941,
       * .4 -> e947, .5 -> e92c, .0 -> e96f, .1 -> e970, .2 -> W1C 0x9302=4 + e952 (= c9a8(1)) =
       * the second USB4 link-event entry that (re)runs bank0_8a89. We service .2 (the connect
       * re-drive) and fall through to the existing 9300 SS-event handling. */
      uint8_t s9302 = REG_BUF_CFG_9302;
      if (s9302 & 0x04) {                          /* 0x9302.2 */
        REG_BUF_CFG_9302 = 0x04;                    /* W1C 0x9302 = 4 */
        bank0_c9a8(1);                             /* e952 -> c9a8(1) -> bank0_8a89(1) */
        uart_puts("[9302.2->c9a8]\n");
      }
      uint8_t ep = REG_BUF_CFG_9300;
      if (ep & BUF_CFG_9300_SS_FAIL) {
        /* USB2-fallback presentation. In USB4 mode (0x09F9 & 0x83) we must NOT drop to USB2:
         * doing so makes the TB4 host see a USB2-HS device and abandon USB4 lane training
         * (E302 stays 0). Gate the USB2 drop on NOT-USB4 mode; still W1C-ack the SS event
         * (REG_BUF_CFG_9300 = ep below) unconditionally so it does not re-storm. */
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

/* INT1 / EX1 (vector 0x13, wired by crt0.s): the PD / USB4 / system interrupt aggregate
 * (C806/C80A/EC06). For this milestone we service only the PD-RX source (C80A bit6). DPX is
 * forced to 0 so the PD MMIO accesses hit bank 0 even if we preempted a DPX=1 window. */
void int1_isr(void) __interrupt(1) {
  /* Stock orchestrator @0x4486 order: timer-tick FIRST and UNGATED, then CC33.2, then C80A.6
   * PD-RX, then the (0x09F9&0x83)-gated USB4 demux, then C806.4 last. */
  uint8_t saved_dpx = DPX;
  DPX = 0x00;
  if (isr_dbg_budget && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget--; uart_putc('I'); }
  if (REG_INT_SYSTEM & 0x01) cc_pd_timer_tick();        /* O1 timer-tick, ungated, FIRST */
  if (isr_dbg_budget2 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget2--; uart_putc('t'); }
  if (REG_CPU_EXEC_STATUS_2 & 0x04) { REG_CPU_EXEC_STATUS_2 = 0x04; }  /* O2 CC33.2 link-state W1C ack */
  if (REG_INT_PCIE_NVME & 0x40) pd_rx_isr();               /* C80A.6 PD-RX */
  if (isr_dbg_budget3 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget3--; uart_putc('p'); }
  /* USB4 event demux, gated by (0x09F9 & 0x83) like the stock orchestrator @0x4486.
   * M2: the C80A.5 SB-router handler (a066, sb_router.h) now W1C-acks every event, so the demux
   * runs every ISR (the M1 one-shot latch is removed). */
  if (XDATA_REG8V(0x09F9) & 0x83) usb4_int_demux();          /* C80A.5/4 + EC06 + C80A.0-3 */
  if (REG_INT_SYSTEM & 0x10) { /* C806.4 -> bank1 ef4e; ack-only for now (no-op) */ }
  DPX = saved_dpx;
}

void main(void) {
  // without this, UART has parity
  REG_UART_LCR &= ~LCR_PARITY_MASK;

  uart_puts("\n[BOOT]\n");
  led_set_rgb(false, false, true);

  // flash controller — needed for the USB serial OTP read on enumeration
  flash_init();

  // boot_phy_bringup_early @0xCE79 — stock's FIRST main() action: Type-C SBU (d0d3), PHY config
  // (cf28: CC30/CC33/CC39/E324...), bank1 SB-block enable (ed02: SB[0x05].7), CC10 settle, and the
  // PCIe-tunnel boot pre-stage (d996). THIS powers the sideband (SB) transport that rides the
  // Type-C SBU pins — without it the SB block reads back 0 and the host never trains E302.
  boot_phy_bringup_early();
  uart_puts("[BOOTPHY cc3f="); uart_puthex(REG_LTSSM_CTRL);
  uart_puts(" cc30=");          uart_puthex(REG_CPU_MODE);
  uart_puts(" e712=");          uart_puthex(REG_LINK_STATUS_E712);
  uart_puts(" sb05=");          uart_puthex(SB_RD(0x05)); uart_puts("]\n");

  // bank0 92C5 RAM-state seed (audit O6/S4). Seeds the lane-engine link WIDTH (0x0AE9=0x0F), MODE
  // (0x0AEE=3) and the 0x0AE3..0x0AF0 state flags the USB4 tunnel reads, plus the 0x0AF1 gate (->0)
  // and C65A/CC35/0x905F tail. handmade never ran 92C5 before, so all of these were uninitialised
  // garbage (0x0AF1 observed 0x55, width/mode random) -> the lane engine ran against garbage. This
  // is the FULL faithful seed (was: only the single 0x0AF1=0 byte). See bank0_92c5_seed (boot_phy.h).
  bank0_92c5_seed();

  // Establish USB4 intent BEFORE the boot-interference block below. 0x09F9 is the runtime mode
  // flag (0x87 = USB4 tunnel route + VDM-ACK); it is otherwise not set until pd_keystone_init()
  // later, so without setting it here the !(0x09F9&0x83) gates below would read the boot-default
  // 0x04 and wrongly run usb_phy_tune/pcie_power_off/pcie_power_on — which clobber tunnel/PHY
  // state the USB4 CM owns. This build always targets USB4; pd_keystone_init re-asserts 0x87.
  XDATA_REG8V(0x09F9) = 0x87;
  // FIX #8 (cap seeds): bank0_8d77 seeds 0x09F5/0x09F6/0x09F7/0x09F8 (DROM cap bits read by
  // sb_rom_descriptor_load) for the normal USB4 path. NOTE: 0x09F4 is DELIBERATELY NOT seeded to 3
  // here -- in stock, 8d77's initial 0x09F4=3 is OVERWRITTEN from the negotiated DP-alt field
  // (0x07059>>4&3); handmade never runs that negotiation, and forcing 0x09F4=3 wrongly trips the
  // usb4_connect_u4 a434 DP-alt sub-case (-> 0x09FA route 0x07->0x06 = a route REGRESSION, observed
  // on HW). So leave 0x09F4 at its handmade default (the USB4-tunnel route path keeps 0x09FA=0x07).
  XDATA_REG8V(0x09F5) = 1; XDATA_REG8V(0x09F6) = 1;
  XDATA_REG8V(0x09F7) = 3; XDATA_REG8V(0x09F8) = 1; XDATA_REG8V(0x09FB) = 3;

  // usb_phy_tune()/pcie_power_off()/pcie_power_on() are handmade-only additions with ZERO stock
  // boot presence. In USB4 mode they corrupt tunnel/PHY regs the CM owns (B480/B430/E764/C659...),
  // so gate all three behind NON-USB4 mode. The legitimate USB4 tunnel-up path keeps its own
  // pcie_power_on() in the deferred sb_tunnel_up_pending handler in the super-loop below.
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

  // Step B (USB4_BOOT_REDESIGN.md sec 2): bring up the USB PIPE engine + arm the upstream USB4
  // PHY link. usb_pipe_engine_init() is the stock B1CB PIPE config (unconditional, both modes);
  // usb4_phy_arm() issues the CC10 subcmd-4 link-up arm and waits E318.4. Run at boot BEFORE the
  // super-loop so no sleep() (which shares the CC10-CC13 mailbox) races the arm's CC11.1 ack.
  usb_pipe_engine_init();
  // boot_phy_early_settle() removed: the CC10 settle is now done by boot_phy_bringup_early() above
  // (run at stock's early position, with the full Type-C SBU + PHY config + SB-block enable).
  usb4_phy_arm();
  uart_puts("[PHYarm e318="); uart_puthex(REG_PHY_COMPLETION_E318);
  uart_puts(" 91c0=");        uart_puthex(REG_USB_PHY_CTRL_91C0);
  uart_puts(" e712=");        uart_puthex(REG_LINK_STATUS_E712); uart_puts("]\n");

  // USB4/PD keystone: present a PD-capable Type-C attach + arm the PD engine so the host
  // engages USB-PD. Must run before interrupts are enabled (so a Source_Cap can be RX'd).
  // Run BEFORE the USB-mode fork so 0x09F9 (set to 0x87 = USB4 here) is valid for the fork.
  pd_keystone_init();

  // Arm the USB4 SB-transport / router interrupt path (the bank1 ef24/ef1e tail of
  // init_sys_flags @0x4BE6 that pd_int1_enable_group omitted). Enables the SB-PHY RX path that
  // detects the host's sideband connect packets and raises C80A.5 (the SB-router event the M2
  // handler services). Without it the SB block is powered but never signals connect.
  usb4_irq_arm();
  uart_puts("[U4irq c21b="); uart_puthex(REG_PHY_LINK_CTRL_C21B);
  uart_puts(" c202=");        uart_puthex(REG_LINK_CTRL);
  uart_puts(" e741=");        uart_puthex(REG_PHY_PLL_CTRL);
  uart_puts(" cc43=");        uart_puthex(REG_CPU_CLK_CFG); uart_puts("]\n");

  // RE-AUDIT #7a: USB4 CM router-op RX-enable (bank1 e56f). Turns on the EC00 router-op engine and
  // sets C807.7 (SB-transport RX-enable) so the host CM's router-op queries (CE88/CE89 transport +
  // EA80/EA90 mailbox) are actually RECEIVED. STEP 0 prereq: the host can only post to a mailbox
  // the device has enabled; without e56f the [ROP] probe below would always read 0. Gated USB4.
  if (XDATA_REG8V(0x09F9) & 0x81) {
    usb4_routerop_init();
    uart_puts("[U4rop ec00="); uart_puthex(XDATA_REG8V(0xEC00));
    uart_puts(" ec04=");        uart_puthex(REG_NVME_EVENT_ACK);
    uart_puts(" c807=");        uart_puthex(REG_INT_DMA_CTRL);
    uart_puts(" ea88=");        uart_puthex(XDATA_REG8V(0xEA88)); uart_puts("]\n");
  }

  // USB-mode fork (USB4_BOOT_REDESIGN.md Step A.2). In USB4 mode (0x09F9 & 0x83) do NOT bring
  // up the USB3 SuperSpeed *device* engine: usb_init_controller(0) SS_FAILs on a TB4 host and
  // force-drops the link to USB2, which makes the host abandon USB4 (E302 never trains). The
  // PD/CC attach the USB4 branch needs is already done by pd_keystone_init() above.
  // REGRESSION GUARD: a real USB3 host (0x09F9 not USB4) MUST still reach usb_init_controller(0).
  if (XDATA_REG8V(0x09F9) & 0x83) {
    uart_puts("[USB4 mode: skip USB3 device bring-up]\n");
  } else {
    // Bring USB up. force_usb2=0: try SS first, fall back via LINK_EVENT.
    usb_init_controller(0);
  }

  // Zero-init the USB4 CM router-op mailbox working state (0x0B02-0x0B1F) and the SB-router
  // active-port index (0x06F1) before any USB4 INT can fire. (M0 prereq, USB4_TUNNEL_PLAN.md sec5.)
  { uint8_t z; for (z = 0; z <= (0x0B1F - 0x0B02); z++) XDATA_REG8V(0x0B02 + z) = 0; }
  XDATA_REG8V(0x06F1) = 0;
  // M2 SB-router state: route-up trigger (0x0766), lane-bonded flag (0x072D), per-lane CL0 latches
  // (0x074E/0x074F), transport-substate latches (0x06EE/0x06EF/0x06F0) — all uninitialised XDATA.
  XDATA_REG8V(0x0766) = 0; XDATA_REG8V(0x072D) = 0;
  XDATA_REG8V(0x074E) = 0; XDATA_REG8V(0x074F) = 0;
  XDATA_REG8V(0x06EE) = 0; XDATA_REG8V(0x06EF) = 0; XDATA_REG8V(0x06F0) = 0;
  // Lane-bond FSM (usb4_lanebond.h): 0x06EC = cb10 enable gate, 0x06ED = the FSM state, 0x0758/0x075x
  // = the cm_conn_routing_setup sub-states, 0x0718 = the route-enable latch. All uninit XDATA (0x55)
  // -> must zero so the FSM starts idle and the [LB arm] (0x06ED==0) first-arm fires correctly.
  XDATA_REG8V(0x06EC) = 0; XDATA_REG8V(0x06ED) = 0;
  XDATA_REG8V(0x0758) = 0; XDATA_REG8V(0x0759) = 0; XDATA_REG8V(0x075A) = 0;
  XDATA_REG8V(0x075B) = 0; XDATA_REG8V(0x075C) = 0; XDATA_REG8V(0x0718) = 0;
  // b0b4 gate inputs: 0x0765 connect-present (set by sb_connect_present_poll->ebb5), 0x0768/0x0769
  // lane-width snapshot (set by db7a->98ec from CCE4:CCE5), 0x0752/0x0753 connect descriptor scratch.
  // All uninit 0x55 -> zero so the gates read a real value (the producers populate them at connect).
  XDATA_REG8V(0x0765) = 0; XDATA_REG8V(0x0768) = 0; XDATA_REG8V(0x0769) = 0;
  XDATA_REG8V(0x0752) = 0; XDATA_REG8V(0x0753) = 0;
  // 0x07ED is the [Connect_U4] one-shot suppress (a176-a17e: if 0x07ED!=0, skip usb4_connect_u4
  // and clear it). It is uninitialised XDATA in handmade, so the FIRST connect can take the
  // suppress branch and never run the SB assert. Stock has it clear on the happy path -> force 0.
  XDATA_REG8V(0x07ED) = 0;

  // IRAM-HEADROOM FIX (2026-06-11): the firmware's persistent state flags/counters were moved out
  // of internal-RAM (DSEG) to XDATA scratch at 0x8800.. (see the declarations) to shrink DSEG so
  // crt0's stack no longer overlaps live globals on the deep INT1 connect path. XDATA is NOT cleared
  // by crt0 (which only zeroes IRAM), so seed them here explicitly. All start at 0 except the
  // [===SB Con===] print budget (=6).
  /* Zero the __at scratch block 0x0B45-0x0B58 (the budgets/flags). Do NOT touch 0x0B40-0x0B44 -- stock
   * uses those cells LIVE (e869/d47f/ee29 recovery); is_usb2/dma_dwords moved to the XSEG. (audit C2) */
  { uint8_t z; for (z = 0; z <= (0x0B58 - 0x0B45); z++) XDATA_REG8V(0x0B45 + z) = 0; }
  sb_con_print_budget = 6;
  u4lb_edf5_print_budget = 40; /* [EDF5] route-query dump budget */
  u4lb_s5_print_budget = 60;   /* [S5] state-5 CL-walker dump budget (XSEG now in chip-CM RAM) */
  isr_dbg_budget = 20; isr_dbg_budget2 = 20; isr_dbg_budget3 = 20;   /* TRIAGE: post-walker hang */
  u4lb_s5_seen = 0; u4lb_s5_last759 = 0; u4lb_s5_last75b = 0;        /* change-gated [s5] diag state */

  // enable interrupts (EX1 = PD/USB4 INT1)
  IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

  i2c_init();
  ina231_init();

  // Milestone 1: prompt the host into PD with periodic Hard Resets until it engages
  // (pd_seen is set by the PD-RX ISR on the first received message), then just observe.
  uint8_t kicks = 0;
  uint8_t sb_diag_count = 0;
  while (1) {
    /* DEADLOCK-BREAK (SESSION 2026-06-11i): the C80A.5 storm starves this loop SO hard it never ran a
     * single iteration after [SB Init] (HW-confirmed). So the ISR's FIRST connect consequence now
     * masks IE_EX1 itself (sb_ex1_mask_pending=2) and the storm stops on that IRET, letting THIS loop
     * finally run. Print a marker the first time we observe the masked state. The loop pumps the armed
     * FSM (0x06ED=3 -> cb10->e672->[ConnRout]) below, then re-enables IE_EX1 once it advances. */
    if (sb_ex1_mask_pending == 2 && !XDATA_REG8V(0x0B54)) {
      XDATA_REG8V(0x0B54) = 1;
      uart_puts("[EX1masked 6ed="); uart_puthex(XDATA_REG8V(0x06ED)); uart_puts("]\n");
    }
    /* ===== FIX#3 (GOFWD): FSM-advance HOISTED to the TOP of the loop, before any delay/prints =====
     * Stock's super-loop (main_boot_and_superloop@0x2FB4) runs `if((0x09F9&0x83)&&0x06EC){EA=0;cb10();
     * ...EA=1;}` FIRST, delay-free and UART-free, so cb10->e672->[ConnRout]->b0b4 runs within us of the
     * C80A.5 connect edge. handmade had buried this under a 60000-NOP delay + ~70 UART field-prints +
     * the EX1-masked 8a89 drive, so it was mid-diagnostic-dump when the host's CM timer expired and the
     * host HARD_RESET. Hoisting + gating the diagnostics off the connect path (0x06EC!=0) reproduces
     * stock's tight cadence. The FSM bodies (e672/cm_conn_routing_setup/b0b4) print their own markers
     * ([SB P0x]/[ConnRout]/[b4:A-D]) so we still SEE the FSM walk. */
    if ((XDATA_REG8V(0x09F9) & 0x83) && XDATA_REG8V(0x06EC)) {
      uint8_t fsm_before = XDATA_REG8V(0x06ED);
      IE &= (uint8_t)~IE_EA;
      if (isr_dbg_budget && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget--; uart_putc('c'); }
      sb_cb10_lane_advance();                     /* the SB[0xA0]/[0xA1] readout + latch compare */
      if (isr_dbg_budget2 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget2--; uart_putc('n'); }
      /* GAP2: super-loop cd3f reproduction -> ebb5 sets 0x0765=1 from the host SB[0x18]/[0x19]
       * connect descriptor. Done HERE (not the a066/INT1 ISR) to keep the C80A.5 stack path intact. */
      sb_connect_present_poll();
      if (isr_dbg_budget3 && XDATA_REG8V(0x06ED) == 5) { isr_dbg_budget3--; uart_putc('k'); }
      if (XDATA_REG8V(0x06ED) == 0) {             /* db7a effect: first-arm the FSM -> [SB P03] */
        uart_puts("[LB arm]");
        u4lb_eb62(0, 3);
        u4lb_98ec();   /* GAP1: stock db7a TAIL is eb62(0,3);98ec() -> snapshot CCE4:CCE5 lane-width
                        * into 0x768:0x769 so b0b4's width gate (b10f) diffs a real value, not 0x55 */
      }
      /* Pump e672 up to 4x per iteration so the FSM walks 3(ConnRout)->4(b0b4)->5 back-to-back inside
       * the connect window. Each e672 call runs exactly ONE faithful state body; break when 0x06ED
       * stops advancing. */
      { uint8_t pmp, prev; for (pmp = 0; pmp < 4; pmp++) {
          prev = XDATA_REG8V(0x06ED);
          u4lb_e672();
          if (XDATA_REG8V(0x06ED) == 5) uart_putc('P');   /* post-e672 at state-5 (unbudgeted, 1 per pump break) */
          if (XDATA_REG8V(0x06ED) == prev) break;
      } }
      IE |= IE_EA;
      if (XDATA_REG8V(0x06ED) == 5) uart_putc('T');        /* tail reached after pump (state-5) */
      /* FIX#4 stall tracking: if the FSM made no 0x06ED progress this iteration, count toward the
       * 8a89-fallback threshold; reset the counter as soon as it advances. */
      if (XDATA_REG8V(0x06ED) == fsm_before) { if (fsm_stall < 0xFF) fsm_stall++; }
      else                                   { fsm_stall = 0; }
      /* DEADLOCK-BREAK: while IE_EX1 is masked (sb_ex1_mask_pending==2) keep pumping the armed FSM with
       * the storm suppressed. RE-ENABLE IE_EX1 only once the FSM has actually ADVANCED past state-3
       * (0x06ED != 3 -> [ConnRout] confirmed and the engine moved on to b0b4/tunnel), at which point we
       * need the SB-router lane-bond/CL0 events again. If the FSM is stuck at state-3 (the host
       * connect-descriptor gate 0x0777!=0x0C -- host-driven per USB4_GOFWD_PLAN B#4), KEEP EX1 masked
       * so the loop stays alive and keeps pumping rather than re-starving on the storm. */
      if (sb_ex1_mask_pending == 2 && XDATA_REG8V(0x06ED) != 3) {
        sb_ex1_mask_pending = 3;              /* 3 = done (one-shot; do not re-mask) */
        IE |= IE_EX1;
        uart_puts("[EX1unmask 6ed="); uart_puthex(XDATA_REG8V(0x06ED)); uart_puts("]\n");
      }
    }

    /* FIX#4 (GOFWD): bank0_8a89 drive DEFERRED off the connect critical path. In STOCK 8a89 is reached
     * ONLY from INT0 link-events (which the host never raises for handmade), so this drive is a hack
     * that starves the FSM and (worse) masks EX1, blocking C80A.5 servicing during the host's window.
     * Run it at most once, and ONLY after the FSM has stalled for several iterations with the lanes not
     * yet at CL0 -- i.e. after the host window has clearly passed without the FSM making progress. Do
     * NOT mask EX1 on the first post-connect iterations (it only runs once fsm_stall is high). */
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

    /* Deferred tunnel-up: the SB-router ISR (e52d) sets sb_tunnel_up_pending on lane-bond-complete;
     * run the downstream PCIe bring-up here (it uses sleep()/long polls, unsafe inside the ISR).
     * Success axis = per-lane CL state SB[0xA0]/[0xA1] reaching CL0=0x02 (stock reaches GPU at E302
     * mode0, so E302-mode is NOT the gate). */
    if (sb_tunnel_up_pending) {
      sb_tunnel_up_pending = 0;
      if ((SB_RD(0xA0) & 0x0F) == 0x02 && (SB_RD(0xA1) & 0x0F) == 0x02) {  /* both lanes CL0 */
        uart_puts("[TunnelUp->pcie_on]\n");
        pcie_power_on();
      } else {
        uart_puts("[TunnelUp deferred: lanes not CL0]\n");
      }
    }

    /* FIX#3: while a connect is IN PROGRESS (0x06EC!=0) keep the loop TIGHT -- no delay, no diagnostic
     * prints -- so the next e672 dispatch lands within us of the connect edge (like stock). The heavy
     * diagnostics below are gated on 0x06EC==0 for the same reason. Only delay when idle.
     * R-timer hazard: after the SB assert sleep() can hang (CC10-CC13 mailbox shared w/ PHY/PD); use a
     * busy-NOP delay once sb_asserted. */
    if (XDATA_REG8V(0x06EC)) {
      continue;   /* connect in progress: skip delay + diagnostics, pump the FSM again immediately */
    }
    if (sb_asserted) { uint32_t b; for (b = 0; b < 60000UL; b++) { __asm nop __endasm; } }
    else             { sleep(500); }
    /* E302 diagnostic: poll E302 in a bounded window to read whether the upstream USB4 PHY trained
     * the link in response to the sideband block. (M2: INT1 stays enabled now, so the C80A.5
     * SB-router handler can preempt this poll to advance the connect handshake — that is wanted.) */
    if (sb_asserted && sb_diag_count < 8) {
      sb_diag_count++;
      { static __xdata uint32_t p; static __xdata uint8_t e302; static __xdata uint8_t best;
        e302 = 0; best = 0;                   /* XDATA: keep these diag locals out of the full IRAM/DSEG */
        for (p = 0; p < 2000000UL; p++) {     /* bounded ~100ms+ poll window for HW to train */
          e302 = REG_PHY_MODE_E302;
          if (((e302 >> 4) & 3) > ((best >> 4) & 3)) best = e302;
          c80a_acc |= REG_INT_PCIE_NVME;    /* catch a transient C80A.5 even with no INT */
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
        /* RE-AUDIT per-layer instrumentation: which layer engaged? c80aACC bit5 = C80A.5 fired;
         * E764.4 = LINK-MODE armed; E751 = USB4 link arm; 8a89? = engine entered; C801.4/C809.3 =
         * SB-PHY RX unmask landed; 0x07E8/0x9101/0x91D1/0x9302 = did the host DRIVE INT0 link events. */
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
    uart_puthex(REG_PHY_MODE_E302); uart_putc(':');   /* USB4 link-mode ((>>4)&3 >=2 == trained) */
    uart_puthex(REG_INT_PCIE_NVME); uart_putc(':');   /* PD/USB4 int status (bit6=PD,5=SB,4=evt) */
    uart_puthex(REG_NVME_EVENT_STATUS); uart_putc(':');   /* USB4 router-op event (bit0) */
    uart_puthex(REG_PHY_RXPLL_TRIGGER); uart_putc(':');   /* PCIe-tunnel link event */
    uart_puthex(XDATA_REG8V(0xB432)); uart_putc(':');   /* downstream PCIe lane status (&7==7 up) */
    uart_puthex(REG_SYS_CTRL_E765); uart_putc(':');   /* PCIe link-up (bit1) */
    uart_puthex(usb4_int_seen); uart_putc(':');         /* USB4 INT sources seen (1=SB,2=evt,4=rop,8=tun) */
    uart_puthex(pd_cc_timeout); uart_puts("]\n");       /* 1 = a CC cmd wait timed out */
    /* Pass: USB4 upstream link trained (E302 link-mode (E302>>4)&3 >= 2). */
    if (((REG_PHY_MODE_E302 >> 4) & 3) >= 2) uart_puts("[*** USB4 TRAINED ***]\n");
    /* M2/M3 observability: SB-router route-up (0x0766) + tunnel/PCIe state. */
    uart_puts("[M2 766=");      uart_puthex(XDATA_REG8V(0x0766));
    uart_puts(" 6f1=");         uart_puthex(XDATA_REG8V(0x06F1));
    uart_puts(" 72d=");         uart_puthex(XDATA_REG8V(0x072D));
    uart_puts(" 72b=");         uart_puthex(XDATA_REG8V(0x072B));
    uart_puts(" 72c=");         uart_puthex(XDATA_REG8V(0x072C));
    uart_puts(" cb10s=");       uart_puthex(cb10_seen);
    uart_puts("]\n");
    /* LANE-TRAIN trace (this session): watch the rate gate C8FF (>=6 == Gen3, REQUIRED for E751)
     * and the lane state SB[0xA0]/[0xA1] (0x07 -> CL0=2). C8FF is a READ-ONLY HW status (the PHY-
     * negotiated rate) -- nothing in stock writes it (verified). E751 (USB4 link arm) is gated on
     * 0x0AA0.0, which bank0_8a89 sets to 1 only when C8FF>=6 (else 0x0AA0=0x0A). So lane TRAINING
     * to Gen3 is a PHY/host outcome we observe, not a register we can poke. */
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
    /* LANE-BOND state-4 (b0b4) observability: the 0x06ED FSM state + the four 0x075x OS1 latches
     * (must reach 0x10 for e672 to enter state-5) + the SB lane state. */
    uart_puts("[S4 6ed=");       uart_puthex(XDATA_REG8V(0x06ED));
    uart_puts(" 75b=");          uart_puthex(XDATA_REG8V(0x075B));
    uart_puts(" 759=");          uart_puthex(XDATA_REG8V(0x0759));
    uart_puts(" 75c=");          uart_puthex(XDATA_REG8V(0x075C));
    uart_puts(" 75a=");          uart_puthex(XDATA_REG8V(0x075A));
    uart_puts(" 765=");          uart_puthex(XDATA_REG8V(0x0765));   /* GAP2 connect-present latch */
    uart_puts(" 768=");          uart_puthex(XDATA_REG8V(0x0768));   /* GAP1 width snapshot hi */
    uart_puts(" 758=");          uart_puthex(XDATA_REG8V(0x0758));   /* cm_conn_routing_setup sub-FSM */
    uart_puts(" 777=");          uart_puthex(XDATA_REG8V(0x0777));   /* host connect-descriptor confirm gate (==0x0C) */
    uart_puts(" sb18=");         uart_puthex(SB_RD(0x18));           /* host connect descriptor (cd3f) */
    uart_puts(" sb28=");         uart_puthex(SB_RD(0x28));           /* cd3f 0x4E source; bit4 gates eaac */
    uart_puts(" cce4=");         uart_puthex(REG_LANE_WIDTH_CNT_HI);   /* HW lane-width counter */
    uart_puts("]\n");
    /* HOST-DRIVEN 0x0777 question: does the host post the SB-plane-2 (0x2a00) connect descriptor
     * that eaac relays into 0x0777? Dump SB-plane-2[0x2a00..0x2a0F] + 0x0775 from the super-loop
     * (runs regardless of whether eaac's SB[0x28].4 gate opened) to see if the host posts 0x0C. */
    { uint8_t k; uart_puts("[P2 775="); uart_puthex(XDATA_REG8V(0x0775));
      uart_puts(" 752="); uart_puthex(XDATA_REG8V(0x0752));   /* host descriptor (cd3f read SB[0x18]) */
      uart_puts(" sb19="); uart_puthex(SB_RD(0x19));          /* L1 host descriptor */
      uart_puts(" 2a="); /* SB-PLANE-2 RX (HW-latched, eaac source) */
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2a00u + k)));
      uart_puts(" 29="); /* SB-PLANE 0x2900 TX (af38 dest/response) */
      for (k = 0; k < 0x10; k++) uart_puthex(P1_REG8_rd((uint16_t)(0x2900u + k)));
      uart_puts("]\n"); }
    /* NOTE (FIX#3): the FSM-advance block, the deferred bank0_8a89 drive, and the deferred tunnel-up
     * were HOISTED to the TOP of this loop (before the delay + the diagnostic dumps above). The
     * diagnostics above now run ONLY when 0x06EC==0 (no connect in progress) because the loop did
     * `continue` while a connect is in progress -- so they never sit between the connect edge and
     * [ConnRout], matching stock's tight cadence. The lane-bond FSM (e672/cm_conn_routing_setup/b0b4)
     * prints its own [SB P0x]/[ConnRout]/[b4:A-D] markers, so the FSM walk is still visible. */
    if (!pd_seen && kicks < 60) {
      pd_drive_hard_reset();
      kicks++;
    }
  }
}
