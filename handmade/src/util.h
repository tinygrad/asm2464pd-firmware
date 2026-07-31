#ifndef UTIL_H
#define UTIL_H

#include "types.h"
#include "registers.h"

/* IE register bits (8051 interrupt enable). */
#define IE_EA   0x80

#define CRITICAL_ENTER()  (IE &= (uint8_t)~IE_EA)
#define CRITICAL_EXIT()   (IE |= IE_EA)

#define DFU_COOKIE              (*(__xdata volatile uint32_t *)0x5FF8)
#define DFU_COOKIE_MAGIC        0xDF0BC0DEUL

static void cpu_reset(void) {
  REG_CPU_RESET = CPU_RESET_TRIGGER;
  while (1) { }
}

static void boot_enter_dfu(void) {
  DFU_COOKIE = DFU_COOKIE_MAGIC;
  cpu_reset();
}

/* Memory primitives for generic (banked) pointers.  SDCC inherits the 3-byte
 * generic pointer type from the void * assignment, so these handle __code,
 * __xdata, and __data sources uniformly.  Length is uint8_t (max 255 bytes). */

static void mem_copy(void *dst, const void *src, uint8_t n) {
  uint8_t *d = dst;
  const uint8_t *s = src;
  while (n--) *d++ = *s++;
}

static void mem_set(void *dst, uint8_t val, uint8_t n) {
  uint8_t *d = dst;
  while (n--) *d++ = val;
}

static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src, uint16_t length) {
  while (length--) *dst++ = *src++;
}

static void timer_delay_ms(uint16_t milliseconds) {
  uint16_t threshold = 2 * milliseconds;
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04U;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold & 0xFF;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
  uint32_t guard = 0;
  while (!(REG_TIMER1_CSR & TIMER_CSR_EXPIRED) && ++guard < 4000000UL);
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
}

void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

static void usb_init_endpoint_state(void) {
  REG_USB_EP0_LEN_H = 0; REG_USB_EP0_LEN_L = 0;
  REG_USB_DMA_TRIGGER = 0; REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
  REG_USB_MSC_CFG = 0; REG_USB_MSC_LENGTH = 0;
  REG_USB_ALT_SETTING_L = 0; REG_USB_ALT_SETTING_H = 0;
  REG_USB_ALT_SETTING2_L = 0; REG_USB_ALT_SETTING2_H = 0;
  REG_USB_DATA_L = 0; REG_USB_DATA_H = 0;
  REG_USB_EP_CFG_905A = 0; REG_USB_EP_BUF_HI = 0; REG_USB_EP_BUF_LO = 0;
  REG_USB_EP_MGMT = 0; REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
  REG_USB_EP_CFG1 = 0x0F;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
  uint8_t pending = REG_USB_EP_READY;
  if (pending) REG_USB_EP_READY = pending;
  REG_USB_EP_CTRL_9097 = 0xFF;
  REG_USB_EP_MODE_9098 = 0xFF; REG_USB_EP_MODE_9099 = 0xFF;
  REG_USB_EP_MODE_909A = 0xFF; REG_USB_EP_MODE_909B = 0xFF;
  REG_USB_EP_MODE_909C = 0xFF; REG_USB_EP_MODE_909D = 0xFF;
  REG_USB_STATUS_909E = 0x03;
  REG_USB_MODE = 0x01;
}

#endif /* UTIL_H */
