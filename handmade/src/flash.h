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
#define FLASH_READ_CHUNK_SIZE   (FLASH_BUFFER_SIZE - 1)

__xdata static uint8_t flash_unlocked;

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
  REG_DMA_TRIGGER = 0;
#ifdef BOOTSTUB
  flash_clear_dma_status();
  flash_unlocked = 0;
#endif
}

#ifdef BOOTSTUB
static void flash_clear_dma_status(void) {
  REG_DMA_TRIGGER = 0;
  REG_DMA_CHAN_CTRL2 = 0;
  REG_DMA_STATUS = 0;
  REG_DMA_STATUS2 = 0;
}

static uint8_t flash_poll_busy(void) {
  uint32_t timeout = 0xFFFFFUL;
  do {
    if (!(REG_FLASH_CSR & 0x01)) return 1;
  } while (--timeout);
  return 0;
}

static uint8_t flash_poll_dma_idle(void) {
  uint16_t timeout = 0xFFFF;
  do {
    if (!(REG_DMA_TRIGGER & DMA_TRIGGER_START) &&
        !(REG_DMA_CHAN_CTRL2 & DMA_CHAN_CTRL2_ACTIVE)) {
      return 1;
    }
  } while (--timeout);
  return 0;
}

static void flash_clear_io_modes(void) {
  REG_FLASH_MODE &= (uint8_t)~0x10;
  REG_FLASH_MODE &= (uint8_t)~0x20;
  REG_FLASH_MODE &= (uint8_t)~0x40;
  REG_FLASH_MODE &= (uint8_t)~0x80;
}
#endif /* BOOTSTUB */

/* Issue one SPI flash controller transaction.
 *
 * Matches the stock helper's register layout:
 *   - C8A3 is transfer length high byte, C8A4 is transfer length low byte.
 *   - C8AC bits 0:1 select the number of address bytes; bits 2:7 are
 *     controller-owned mode bits and must be preserved.
 *
 * A swapped length order turns a 4096-byte read into a 16-byte read,
 * leaving most of FLASH_BUF stale and making CRC validation
 * non-deterministic. */
#ifdef BOOTSTUB
static uint8_t flash_cmd(uint8_t cmd, uint32_t addr, uint8_t addr_bytes, uint16_t data_len, uint8_t write_buf) {
  uint8_t ok;
  uint8_t read_buf = !write_buf && data_len;
  if (addr_bytes > 7 || data_len > FLASH_BUFFER_SIZE) return 0;
  if (!flash_poll_busy()) return 0;
  if (!write_buf && data_len) {
    flash_clear_dma_status();
    if (!flash_poll_dma_idle()) return 0;
  }

  REG_FLASH_CON = 0;
  if (write_buf) REG_FLASH_MODE |= FLASH_MODE_ENABLE;
  else           REG_FLASH_MODE &= (uint8_t)~FLASH_MODE_ENABLE;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = 0;
  REG_FLASH_CMD = cmd;
  REG_FLASH_ADDR_LEN = (REG_FLASH_ADDR_LEN & FLASH_ADDR_LEN_MASK) | addr_bytes;
  REG_FLASH_ADDR_LO = addr & 0xFF;
  REG_FLASH_ADDR_MD = (addr >> 8) & 0xFF;
  REG_FLASH_ADDR_HI = (addr >> 16) & 0xFF;
  REG_FLASH_DATA_LEN_HI = (data_len >> 8) & 0xFF;
  REG_FLASH_DATA_LEN_LO = data_len & 0xFF;
  REG_FLASH_CSR = 0x01;
  ok = flash_poll_busy();
  if (ok && !write_buf && data_len) ok = flash_poll_dma_idle();
  REG_FLASH_MODE &= (uint8_t)~FLASH_MODE_ENABLE;
  flash_clear_io_modes();
  if (!write_buf && data_len) flash_clear_dma_status();
  return ok;
}
#else
static uint8_t flash_cmd(uint8_t cmd, uint32_t addr, uint8_t addr_bytes, uint16_t data_len, uint8_t write_buf) {
  (void)write_buf;
  REG_FLASH_CON = 0;
  REG_FLASH_MODE = 0;
  REG_FLASH_BUF_OFFSET_LO = 0;
  REG_FLASH_BUF_OFFSET_HI = 0;
  REG_FLASH_CMD = cmd;
  REG_FLASH_ADDR_LEN = (REG_FLASH_ADDR_LEN & FLASH_ADDR_LEN_MASK) | addr_bytes;
  REG_FLASH_ADDR_LO = addr & 0xFF;
  REG_FLASH_ADDR_MD = (addr >> 8) & 0xFF;
  REG_FLASH_ADDR_HI = (addr >> 16) & 0xFF;
  REG_FLASH_DATA_LEN_HI = (data_len >> 8) & 0xFF;
  REG_FLASH_DATA_LEN_LO = data_len & 0xFF;
  REG_FLASH_CSR = 0x01;
  { uint16_t t = 0xFFFF; do { if (!(REG_FLASH_CSR & 0x01)) break; } while (--t); }
  REG_FLASH_MODE = 0;
  return !(REG_FLASH_CSR & 0x01);
}
#endif

#ifdef BOOTSTUB
static uint8_t flash_wren(void) {
  return flash_cmd(FLASH_CMD_WREN, 0, FLASH_ADDR_LEN_NOADDR, 0, 0);
}

/* Spin until the SPI flash chip's WIP bit clears (program/erase done). */
static uint8_t flash_wait_wip(void) {
  uint32_t timeout = 0x000FFFFFUL;
  do {
    if (!flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0)) return 0;
    if (!(FLASH_BUF[0] & 0x01)) return 1;
  } while (--timeout);
  return 0;
}

/* Clear the SPI flash chip's block-protect bits (SR bits 2-4). */
static uint8_t flash_clear_bp(void) {
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    FLASH_BUF[0] = 0; FLASH_BUF[1] = 0; FLASH_BUF[2] = 0; FLASH_BUF[3] = 0;
    if (!flash_wren()) continue;
    if (!flash_cmd(FLASH_CMD_WRSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 1)) continue;
    if (!flash_wait_wip()) continue;
    if (!flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0)) continue;
    if (!(FLASH_BUF[0] & 0x1C)) return 1;
  }
  return 0;
}

static uint8_t flash_unlock(void) {
  if (flash_unlocked) return 1;
  flash_unlocked = flash_clear_bp();
  return flash_unlocked;
}

/* Issue a 4 KB sector erase. */
static uint8_t flash_erase_sector(uint32_t addr) {
  if (!flash_unlock()) return 0;
  if (!flash_wren()) { flash_unlocked = 0; return 0; }
  if (!flash_cmd(FLASH_CMD_SECTOR_ERASE, addr, FLASH_ADDR_LEN_3BYTE, 0, 0)) { flash_unlocked = 0; return 0; }
  if (!flash_wait_wip()) { flash_unlocked = 0; return 0; }
  return 1;
}

/* Program `len` bytes from 0x7000 into flash at `addr`. Caller must have
 * loaded 0x7000 (e.g. via bulk OUT). `len` must not cross a 256-byte SPI
 * page boundary. */
static uint8_t flash_program_page(uint32_t addr, uint16_t len) {
  if (len == 0 || len > 256 || ((addr & 0xFF) + len) > 0x100) return 0;
  if (!flash_unlock()) return 0;
  if (!flash_wren()) { flash_unlocked = 0; return 0; }
  if (!flash_cmd(FLASH_CMD_PAGE_PROGRAM, addr, FLASH_ADDR_LEN_3BYTE, len, 1)) { flash_unlocked = 0; return 0; }
  if (!flash_wait_wip()) { flash_unlocked = 0; return 0; }
  return 1;
}
#endif /* BOOTSTUB */

/* OTP layout (programmed by provisioning scripts): 4-byte serial +
 * XOR checksum. Blank OTP is all 0xFF and fails the checksum check. */
typedef struct {
  uint8_t  serial[4];
  uint8_t  checksum;
} otp_t;

/* Read the OTP header. Returns 1 + populates `out` on a checksum match,
 * 0 if blank or corrupt. */
static uint8_t flash_read_otp(__xdata otp_t *out) {
  __xdata uint8_t *p = (__xdata uint8_t *)out;
  uint8_t i, csum = 0;
  if (!flash_cmd(FLASH_CMD_ENSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0)) return 0;
  for (i = 0; i < sizeof(otp_t); i++) {
    if (!flash_cmd(FLASH_CMD_READ, i, FLASH_ADDR_LEN_3BYTE, 1, 0)) {
      flash_cmd(FLASH_CMD_EXSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0);
      return 0;
    }
    p[i] = FLASH_BUF[0];
  }
  for (i = 0; i < 4; i++) {
    csum ^= p[i];
  }
  if (!flash_cmd(FLASH_CMD_EXSO, 0, FLASH_ADDR_LEN_NOADDR, 0, 0)) return 0;
  return out->checksum == csum;
}

#ifdef BOOTSTUB
/* Stream `len` bytes from flash `addr` into `dst`.
 *
 * Multi-byte CPU reads from FLASH_BUF[0] were not stable in hardware testing,
 * so guarded chunks discard slot 0. */
static uint8_t flash_read(uint32_t addr, __xdata uint8_t *dst, uint16_t len) {
  if (len && addr == 0) {
    if (!flash_cmd(FLASH_CMD_READ, 0, FLASH_ADDR_LEN_3BYTE, 1, 0)) return 0;
    flash_clear_dma_status();
    *dst++ = FLASH_BUF[0];
    addr++;
    len--;
  }
  while (len) {
    uint16_t chunk = FLASH_READ_CHUNK_SIZE;
    if (chunk > len) chunk = len;
    if (!flash_cmd(FLASH_CMD_READ, addr - 1, FLASH_ADDR_LEN_3BYTE, chunk + 1, 0)) return 0;
    flash_clear_dma_status();
    xmemcpy(dst, FLASH_BUF + 1, chunk);
    dst  += chunk;
    addr += chunk;
    len  -= chunk;
  }
  return 1;
}
#endif /* BOOTSTUB */

#endif
