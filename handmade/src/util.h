#ifndef __HANDMADE_UTIL_H__
#define __HANDMADE_UTIL_H__

#include "types.h"
#include "registers.h"

#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_HEADER_SIZE      0x40UL
#define USERFW_HEADER_CRC_LEN   0x20U
#define USERFW_GITVERSION_SIZE  24U
#define USERFW_CODE_BASE        0x2400
#define USERFW_BODY_LIMIT       0xC000UL
#define USERFW_FLASH_END        (USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + USERFW_BODY_LIMIT)
#define SECTOR_SIZE             0x1000UL
#define USERFW_ERASE_END        ((USERFW_FLASH_END + (SECTOR_SIZE - 1)) & ~(SECTOR_SIZE - 1))

typedef struct {
  uint8_t  magic[4];          /* 'A','2','4','F' */
  uint8_t  gitversion[USERFW_GITVERSION_SIZE];
  uint32_t body_len;
  uint32_t crc;
  uint8_t  _pad2[28];
} userfw_hdr_t;

static uint8_t userfw_header_magic_ok(__xdata const userfw_hdr_t *hdr) {
  return hdr->magic[0] == 'A' && hdr->magic[1] == '2' &&
         hdr->magic[2] == '4' && hdr->magic[3] == 'F';
}

static uint8_t userfw_header_has_gitversion(__xdata const userfw_hdr_t *hdr) {
  return userfw_header_magic_ok(hdr) &&
         hdr->gitversion[0] != 0 &&
         hdr->gitversion[0] != 0xFF;
}

static void uart_putc(uint8_t ch) {
  while (REG_UART_TFBF == 0);
  REG_UART_THR = ch;
}

static void uart_puts(__code const char *s) {
  while (*s) uart_putc(*s++);
}

static void uart_puthex(uint8_t val) {
  static __code const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[val >> 4]);
  uart_putc(hex[val & 0x0F]);
}

#define TIMER1_MODE_HALF_MS     0x04U

/* Millisecond busy-sleep on Timer1; must never touch Timer0 (CC10-CC13),
 * which the USB4 path uses for PHY link-up timeouts and INT1 wake-ups. */
static void timer_delay_ms(uint16_t milliseconds) {
  uint16_t threshold = 2 * milliseconds;
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
  REG_TIMER1_CSR = TIMER_CSR_EXPIRED;
  REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | TIMER1_MODE_HALF_MS;
  REG_TIMER1_THRESHOLD_HI = threshold >> 8;
  REG_TIMER1_THRESHOLD_LO = threshold & 0xFF;
  REG_TIMER1_CSR = TIMER_CSR_ENABLE;
  { uint32_t g = 0; while (!(REG_TIMER1_CSR & TIMER_CSR_EXPIRED) && ++g < 4000000UL); }
  REG_TIMER1_CSR = TIMER_CSR_CLEAR;
}

static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src, uint16_t len) {
  while (len--) *dst++ = *src++;
}

/* Memory primitives for generic (banked) pointers.  SDCC inherits the 3-byte
 * generic pointer type from the void * assignment, so these handle __code,
 * __data, and __xdata sources uniformly.  Length is uint8_t (max 255 bytes). */
static void mem_copy(void *dst, const void *src, uint8_t n) {
  uint8_t *d = dst;
  const uint8_t *s = src;
  while (n--) *d++ = *s++;
}

static void mem_set(void *dst, uint8_t val, uint8_t n) {
  uint8_t *d = dst;
  while (n--) *d++ = val;
}

#endif
