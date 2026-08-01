#ifndef BOOTSTUB_UTIL_H
#define BOOTSTUB_UTIL_H

#include "registers.h"
#include "types.h"

#define DFU_COOKIE (*(__xdata volatile uint32_t *)0x5FF8)
#define DFU_COOKIE_MAGIC 0xDF0BC0DEUL

static void cpu_reset(void) {
  REG_CPU_RESET = CPU_RESET_TRIGGER;
  while (1) {
  }
}

static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src, uint16_t length) {
  while (length--)
    *dst++ = *src++;
}

static void timer_delay_ms(uint16_t milliseconds) {
  uint16_t threshold = 2 * milliseconds;
  uint32_t guard = 0;
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
  while (!(REG_TIMER1_CSR & TIMER_CSR_EXPIRED) && ++guard < 4000000UL) {
  }
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
}

static void usb_init_endpoint_state(void) {
  uint8_t pending;
  REG_USB_EP0_LEN_H = 0;
  REG_USB_EP0_LEN_L = 0;
  REG_USB_DMA_TRIGGER = 0;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
  REG_USB_MSC_CFG = 0;
  REG_USB_MSC_LENGTH = 0;
  REG_USB_ALT_SETTING_L = 0;
  REG_USB_ALT_SETTING_H = 0;
  REG_USB_ALT_SETTING2_L = 0;
  REG_USB_ALT_SETTING2_H = 0;
  REG_USB_DATA_L = 0;
  REG_USB_DATA_H = 0;
  REG_USB_EP_CFG_905A = 0;
  REG_USB_EP_BUF_HI = 0;
  REG_USB_EP_BUF_LO = 0;
  REG_USB_EP_MGMT = 0;
  REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
  REG_USB_EP_CFG1 = 0x0F;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
  pending = REG_USB_EP_READY;
  if (pending)
    REG_USB_EP_READY = pending;
  REG_USB_EP_CTRL_9097 = 0xFF;
  REG_USB_EP_MODE_9098 = 0xFF;
  REG_USB_EP_MODE_9099 = 0xFF;
  REG_USB_EP_MODE_909A = 0xFF;
  REG_USB_EP_MODE_909B = 0xFF;
  REG_USB_EP_MODE_909C = 0xFF;
  REG_USB_EP_MODE_909D = 0xFF;
  REG_USB_STATUS_909E = 0x03;
  REG_USB_MODE = 0x01;
}

#endif
