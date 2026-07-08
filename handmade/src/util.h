#ifndef UTIL_H
#define UTIL_H

#include "types.h"
#include "registers.h"

/* IE register bits (8051 interrupt enable). */
#define IE_EA   0x80

#define CRITICAL_ENTER()  (IE &= (uint8_t)~IE_EA)
#define CRITICAL_EXIT()   (IE |= IE_EA)

/* Userfw layout constants (shared between bootstub and app). */
#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_HEADER_SIZE      0x40UL
#define USERFW_HEADER_CRC_LEN   0x20U
#define USERFW_CODE_BASE        0x2400
#define USERFW_BODY_LIMIT       0xE000UL
#define USERFW_FLASH_END        (USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + USERFW_BODY_LIMIT)
#define SECTOR_SIZE             0x1000UL
#define USERFW_ERASE_END        ((USERFW_FLASH_END + (SECTOR_SIZE - 1)) & ~(SECTOR_SIZE - 1))

/* Userfw header structure. */
typedef struct {
  uint8_t  magic[4];
  uint8_t  gitversion[24];
  uint32_t body_len;
  uint32_t crc;
  uint8_t  _pad[28];
} userfw_hdr_t;

static uint8_t userfw_header_magic_ok(__xdata const userfw_hdr_t *hdr) {
  return hdr->magic[0] == 'A' && hdr->magic[1] == '2' &&
         hdr->magic[2] == '4' && hdr->magic[3] == 'F';
}

static uint8_t userfw_header_has_gitversion(__xdata const userfw_hdr_t *hdr) {
  return userfw_header_magic_ok(hdr) &&
         hdr->gitversion[0] != 0 && hdr->gitversion[0] != 0xFF;
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

static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src, uint16_t len) {
  while (len--) *dst++ = *src++;
}

/* Timer1-based millisecond delay. Used by usb_attach_controller. */
static void timer_delay_ms(uint16_t milliseconds) {
  uint16_t threshold = 2 * milliseconds;
  REG_TIMER1_CSR = 0x04; REG_TIMER1_CSR = 0x02;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold & 0xFF;
  REG_TIMER1_CSR = 0x01;
  { uint32_t g = 0; while (!(REG_TIMER1_CSR & 0x02) && ++g < 4000000UL); }
  REG_TIMER1_CSR = 0x04; REG_TIMER1_CSR = 0x02;
}

/* UART output helpers. */
static void uart_putc(uint8_t ch) { while (!REG_UART_TFBF); REG_UART_THR = ch; }
static void uart_puts(__code const char *s) { while (*s) uart_putc(*s++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

/* USB endpoint state init — shared between app and bootstub. */
static void usb_init_endpoint_state(void) {
  REG_USB_EP0_LEN_H = 0; REG_USB_EP0_LEN_L = 0;
  REG_USB_EP0_CONFIG = 0;
  REG_USB_DMA_TRIGGER = 0; REG_USB_CTRL_PHASE = USB_CTRL_PHASE_ALL;
  REG_USB_MSC_CFG = 0; REG_USB_MSC_LENGTH = 0;
  REG_USB_ALT_SETTING_L = 0; REG_USB_ALT_SETTING_H = 0;
  REG_USB_ALT_SETTING2_L = 0; REG_USB_ALT_SETTING2_H = 0;
  REG_USB_DATA_L = 0; REG_USB_DATA_H = 0;
  REG_USB_EP_CFG_905A = 0; REG_USB_EP_BUF_HI = 0; REG_USB_EP_BUF_LO = 0;
  REG_USB_EP_MGMT = 0; REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
  REG_USB_EP_CFG1 = USB_EP_CFG1_INIT_CLEAR;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
  REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
  { uint8_t p = REG_USB_EP_READY; if (p) REG_USB_EP_READY = p; }
  REG_USB_EP_CTRL_9097 = USB_EP_CTRL_9097_INIT;
  REG_USB_EP_MODE_9098 = USB_EP_MODE_INIT; REG_USB_EP_MODE_9099 = USB_EP_MODE_INIT;
  REG_USB_EP_MODE_909A = USB_EP_MODE_INIT; REG_USB_EP_MODE_909B = USB_EP_MODE_INIT;
  REG_USB_EP_MODE_909C = USB_EP_MODE_INIT; REG_USB_EP_MODE_909D = USB_EP_MODE_INIT;
  REG_USB_STATUS_909E = USB_STATUS_909E_INIT;
  REG_USB_MODE = USB_MODE_INIT;
}

#endif /* UTIL_H */
