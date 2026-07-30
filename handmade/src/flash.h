#ifndef __FLASH_H__
#define __FLASH_H__

#include "types.h"
#include "registers.h"
#include "util.h"

#define FLASH_CMD_WRSR          0x01
#define FLASH_CMD_PAGE_PROGRAM  0x02
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_RDSR          0x05
#define FLASH_CMD_WREN          0x06
#define FLASH_CMD_SECTOR_ERASE  0x20
#define FLASH_CMD_ENSO          0xB1
#define FLASH_CMD_EXSO          0xC1
#define FLASH_3BYTE_LIMIT       0x1000000UL

#ifdef BOOTSTUB

typedef struct {
  uint8_t serial[4];
  uint8_t checksum;
} otp_t;

__xdata static uint8_t flash_unlocked;

static void flash_clear_dma_status(void) {
  REG_DMA_TRIGGER = 0;
  REG_DMA_CHAN_CTRL2 = 0;
  REG_DMA_STATUS = 0;
  REG_DMA_STATUS2 = 0;
}

static uint8_t flash_poll_busy(void) {
  uint32_t timeout = 0xFFFFFUL;
  while (REG_FLASH_CSR & 0x01) {
    if (!--timeout) return 0;
  }
  return 1;
}

static uint8_t flash_poll_dma_idle(void) {
  uint16_t timeout = 0xFFFF;
  while ((REG_DMA_TRIGGER & DMA_TRIGGER_START) ||
         (REG_DMA_CHAN_CTRL2 & DMA_CHAN_CTRL2_ACTIVE)) {
    if (!--timeout) return 0;
  }
  return 1;
}

static void flash_clear_io_modes(void) {
  REG_FLASH_MODE &= (uint8_t)~0x10;
  REG_FLASH_MODE &= (uint8_t)~0x20;
  REG_FLASH_MODE &= (uint8_t)~0x40;
  REG_FLASH_MODE &= (uint8_t)~0x80;
}

static void flash_init(void) {
  REG_CPU_EXEC_STATUS_2 = 0x04;
  REG_INT_AUX_STATUS = 0x02;
  REG_FLASH_DIV = 0x04;
  REG_FLASH_CON = 0;
  REG_FLASH_CSR = 0;
  REG_FLASH_MODE = 0;
  REG_FLASH_ADDR_LEN = 0;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = 0;
  flash_clear_dma_status();
  flash_unlocked = 0;
}

static uint8_t flash_cmd(uint8_t command, uint32_t address,
                         uint8_t address_mode, uint16_t length,
                         uint8_t write) {
  uint8_t ok;
  uint8_t read = length && !write;

  if (address_mode > FLASH_ADDR_LEN_3BYTE || length > FLASH_BUFFER_SIZE ||
      !flash_poll_busy()) return 0;
  if (read) {
    flash_clear_dma_status();
    if (!flash_poll_dma_idle()) return 0;
  }

  REG_FLASH_CON = 0;
  if (write) REG_FLASH_MODE |= FLASH_MODE_ENABLE;
  else       REG_FLASH_MODE &= (uint8_t)~FLASH_MODE_ENABLE;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = 0;
  REG_FLASH_CMD = command;
  REG_FLASH_ADDR_LEN = (REG_FLASH_ADDR_LEN & FLASH_ADDR_LEN_MASK) | address_mode;
  REG_FLASH_ADDR_LO = address;
  REG_FLASH_ADDR_MD = address >> 8;
  REG_FLASH_ADDR_HI = address >> 16;
  REG_FLASH_DATA_LEN_HI = length >> 8;
  REG_FLASH_DATA_LEN_LO = length;
  REG_FLASH_CSR = 0x01;

  ok = flash_poll_busy();
  if (ok && read) ok = flash_poll_dma_idle();
  REG_FLASH_MODE &= (uint8_t)~FLASH_MODE_ENABLE;
  flash_clear_io_modes();
  if (read) flash_clear_dma_status();
  return ok;
}

static uint8_t flash_write_enable(void) {
  return flash_cmd(FLASH_CMD_WREN, 0, FLASH_ADDR_LEN_NOADDR, 0, 0);
}

static uint8_t flash_wait_write(void) {
  uint32_t timeout = 0xFFFFFUL;
  while (timeout--) {
    if (!flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0)) return 0;
    if (!(FLASH_BUF[0] & 0x01)) return 1;
  }
  return 0;
}

static uint8_t flash_clear_protection(void) {
  uint8_t attempt;

  if (flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0) &&
      !(FLASH_BUF[0] & 0x1C)) return 1;

  for (attempt = 0; attempt < 5; attempt++) {
    FLASH_BUF[0] = 0;
    if (flash_write_enable() &&
        flash_cmd(FLASH_CMD_WRSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 1) &&
        flash_wait_write() &&
        flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0) &&
        !(FLASH_BUF[0] & 0x1C)) return 1;
  }
  return 0;
}

static uint8_t flash_unlock(void) {
  if (!flash_unlocked) flash_unlocked = flash_clear_protection();
  return flash_unlocked;
}

static uint8_t flash_erase_sector(uint32_t address) {
  if (!flash_unlock() || !flash_write_enable() ||
      !flash_cmd(FLASH_CMD_SECTOR_ERASE, address, FLASH_ADDR_LEN_3BYTE, 0, 0) ||
      !flash_wait_write()) {
    flash_unlocked = 0;
    return 0;
  }
  return 1;
}

static uint8_t flash_program_page(uint32_t address, uint16_t length) {
  if (!length || length > 256 || ((address & 0xFF) + length) > 0x100)
    return 0;
  if (!flash_unlock() || !flash_write_enable() ||
      !flash_cmd(FLASH_CMD_PAGE_PROGRAM, address, FLASH_ADDR_LEN_3BYTE,
                 length, 1) ||
      !flash_wait_write()) {
    flash_unlocked = 0;
    return 0;
  }
  return 1;
}

static uint8_t flash_read_otp(__xdata otp_t *out) {
  __xdata uint8_t *bytes = (__xdata uint8_t *)out;
  uint8_t i;
  uint8_t checksum = 0;

  if (!flash_cmd(FLASH_CMD_ENSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0)) return 0;
  for (i = 0; i < sizeof(otp_t); i++) {
    if (!flash_cmd(FLASH_CMD_READ, i, FLASH_ADDR_LEN_3BYTE, 1, 0)) {
      (void)flash_cmd(FLASH_CMD_EXSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0);
      return 0;
    }
    bytes[i] = FLASH_BUF[0];
  }
  if (!flash_cmd(FLASH_CMD_EXSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0)) return 0;
  for (i = 0; i < 4; i++) checksum ^= out->serial[i];
  return out->checksum == checksum;
}

/* FLASH_BUF[0] is unreliable after multi-byte DMA reads. */
static uint8_t flash_read(uint32_t address, __xdata uint8_t *dst,
                          uint16_t length) {
  while (length) {
    uint16_t chunk;
    if (address == 0) {
      if (!flash_cmd(FLASH_CMD_READ, 0, FLASH_ADDR_LEN_3BYTE, 1, 0)) return 0;
      *dst++ = FLASH_BUF[0];
      address++;
      length--;
      continue;
    }
    chunk = length < FLASH_BUFFER_SIZE - 1 ? length : FLASH_BUFFER_SIZE - 1;
    if (!flash_cmd(FLASH_CMD_READ, address - 1, FLASH_ADDR_LEN_3BYTE,
                   chunk + 1, 0)) return 0;
    xmemcpy(dst, FLASH_BUF + 1, chunk);
    dst += chunk;
    address += chunk;
    length -= chunk;
  }
  return 1;
}

#else

static void flash_init(void) {
  REG_CPU_EXEC_STATUS_2 = 0x04;
  // NOTE: this broke PCIe enumeration on 9060
  //REG_CPU_CTRL_CA81 |= 0x01;
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
#endif
