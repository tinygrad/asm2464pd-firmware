#ifndef __FLASH_H__
#define __FLASH_H__

#include "types.h"
#include "registers.h"

static void flash_init(void) {
  REG_CPU_EXEC_STATUS_2 = 0x04;
  REG_CPU_CTRL_CA81 |= 0x01;
  REG_INT_AUX_STATUS = 0x02;
  REG_FLASH_DIV = 0x04;
}

static void flash_poll_busy(void) {
  uint16_t timeout = 0xFFFF;
  do {
    if (!(REG_FLASH_CSR & 0x01)) break;
  } while (--timeout);
}

/* Issue a flash command. addr_len: 0x04 = no address byte; 0x07 = 24-bit
 * address. data_len: bytes the controller will clock in / out via the
 * 0x7000 buffer. */
static void flash_cmd(uint8_t cmd, uint32_t addr, uint8_t addr_len, uint16_t data_len) {
  REG_FLASH_MODE = 0;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = 0;
  REG_FLASH_CMD = cmd;
  REG_FLASH_ADDR_LEN = addr_len;
  REG_FLASH_ADDR_LO = addr & 0xFF;
  REG_FLASH_ADDR_MD = (addr >> 8) & 0xFF;
  REG_FLASH_ADDR_HI = (addr >> 16) & 0xFF;
  REG_FLASH_DATA_PAGE_CNT = (data_len >> 8) & 0xFF;
  REG_FLASH_DATA_BYTE_OFS = data_len & 0xFF;
  REG_FLASH_CSR = 0x01;
  flash_poll_busy();
  REG_FLASH_MODE = 0; REG_FLASH_MODE = 0;
  REG_FLASH_MODE = 0; REG_FLASH_MODE = 0;
}

/* OTP layout (programmed by provisioning scripts): 4-byte serial +
 * XOR checksum. Blank OTP is all 0xFF and fails the checksum check. */
typedef struct {
  uint8_t  serial[4];
  uint8_t  checksum;
} otp_t;

/* Read the OTP header. Returns 1 + populates `out` on a checksum match,
 * 0 if blank or corrupt. The buffer must be copied BEFORE EXSO and we
 * must not pre-touch FLASH_BUF — CPU writes race with the read DMA. */
static uint8_t flash_read_otp(__xdata otp_t *out) {
  __xdata uint8_t *p = (__xdata uint8_t *)out;
  uint8_t i, csum = 0;
  flash_cmd(0xB1, 0, 0x04, 0);              /* ENSO — enter OTP mode */
  flash_cmd(0x03, 0, 0x07, sizeof(otp_t));
  for (i = 0; i < 4; i++) {
    p[i] = FLASH_BUF[i];
    csum ^= p[i];
  }
  out->checksum = FLASH_BUF[4];
  flash_cmd(0xC1, 0, 0x04, 0);              /* EXSO — exit OTP mode */
  return out->checksum == csum;
}

#endif
