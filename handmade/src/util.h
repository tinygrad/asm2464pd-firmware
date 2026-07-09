#ifndef UTIL_H
#define UTIL_H

#include "types.h"
#include "registers.h"

/* IE register bits (8051 interrupt enable). */
#define IE_EA   0x80

#define CRITICAL_ENTER()  (IE &= (uint8_t)~IE_EA)
#define CRITICAL_EXIT()   (IE |= IE_EA)

/* App-to-bootstub DFU handoff cookie. 0x5FF8-0x5FFF is reserved in the
 * bootstub linker flags and survives CPU reset: the app writes the magic and
 * CPU-resets; the bootstub sees it on the next boot and stays in DFU. */
#define DFU_COOKIE              (*(__xdata volatile uint32_t *)0x5FF8)
#define DFU_COOKIE_MAGIC        0xDF0BC0DEUL

/* Boot-attempt watchdog (lives in the reserved 0x5FFC-0x5FFF word). This word
 * survives a soft CPU reset (REG_CPU_RESET / 0xEB) but NOT a hardware reset or
 * power cycle, which clear XRAM — so this specifically catches a firmware
 * release that CRC-validates but soft-reset boot-loops before its USB/tunnel
 * recovery channel comes up (the classic "reboot-loop before enumeration",
 * where 0xEC can never reach the app). The bootstub increments the low-byte
 * count before jumping to a CRC-valid userfw; a healthy app calls
 * boot_mark_healthy() once a host-facing recovery channel is confirmed up (USB
 * enumeration -> 0xEC works, or the USB4 tunnel -> the router-op works). After
 * BOOT_MAX_ATTEMPTS un-cleared attempts the bootstub stays in DFU so the field
 * can reflash without the FTDI strap. A cold boot's uninitialized value fails
 * the marker check and starts at 0. (A pure hang with no reset, or a power
 * cycle, cannot be caught by any counter and still needs an external reset.) */
#define BOOT_TRACK              (*(__xdata volatile uint32_t *)0x5FFC)
#define BOOT_TRACK_MARK         0xB0075B00UL   /* upper 24 bits = validity marker; low byte = count */
#define BOOT_TRACK_MASK         0xFFFFFF00UL
#define BOOT_MAX_ATTEMPTS       3

/* Invalidate the marker so the next boot starts a fresh count. Called by the
 * app; a no-op-shaped write on the classic (no-bootstub) build. */
static void boot_mark_healthy(void) {
    BOOT_TRACK = 0;
}

/* Userfw layout constants (shared between bootstub and app). */
#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_HEADER_SIZE      0x40UL
#define USERFW_HEADER_CRC_LEN   0x20U
#define USERFW_CODE_BASE        0x2400
#define USERFW_BODY_LIMIT       0xDC00UL  /* CODE 0x2400-0xFFFF */
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

#ifdef BOOTSTUB
static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src, uint16_t len) {
  while (len--) *dst++ = *src++;
}
#endif

/* Timer1-based millisecond delay. Used by usb_attach_controller. */
#ifdef BOOTSTUB
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
#endif

/* UART output helpers. */
void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

/* USB endpoint state reset to the documented stock init values. The
 * bootstub runs this before its DFU bring-up; the app runs it when
 * re-initializing USB after a register reset (bootstub 0xEB handoff). */
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
