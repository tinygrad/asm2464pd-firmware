#ifndef BOOTSTUB_FLASH_H
#define BOOTSTUB_FLASH_H

#include "registers.h"
#include "types.h"
#include "util.h"

#define FLASH_CMD_WRSR 0x01
#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_READ 0x03
#define FLASH_CMD_RDSR 0x05
#define FLASH_CMD_WREN 0x06
#define FLASH_CMD_SECTOR_ERASE 0x20

__xdata static uint8_t flash_unlocked;

static uint8_t flash_poll_busy(void) {
  uint32_t timeout = 0xFFFFFUL;
  while (REG_FLASH_CSR & 0x01) {
    if (!--timeout)
      return 0;
  }
  return 1;
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
  flash_unlocked = 0;
}

static uint8_t flash_cmd(uint8_t command, uint32_t address, uint8_t address_mode, uint16_t length, uint8_t write) {
  uint8_t ok;
  if (address_mode > FLASH_ADDR_LEN_3BYTE || length > FLASH_BUFFER_SIZE || !flash_poll_busy())
    return 0;

  REG_FLASH_CON = 0;
  REG_FLASH_MODE = write ? FLASH_MODE_WRITE : 0;
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
  REG_FLASH_MODE = 0;
  return ok;
}

static uint8_t flash_write_enable(void) { return flash_cmd(FLASH_CMD_WREN, 0, FLASH_ADDR_LEN_NOADDR, 0, 0); }

static uint8_t flash_wait_write(void) {
  uint32_t timeout = 0xFFFFFUL;
  while (timeout--) {
    if (!flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0))
      return 0;
    if (!(FLASH_BUF[0] & 0x01))
      return 1;
  }
  return 0;
}

static uint8_t flash_clear_protection(void) {
  uint8_t attempt;
  if (flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0) && !(FLASH_BUF[0] & 0x1C))
    return 1;

  for (attempt = 0; attempt < 5; attempt++) {
    FLASH_BUF[0] = 0;
    if (flash_write_enable() && flash_cmd(FLASH_CMD_WRSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 1) && flash_wait_write() &&
        flash_cmd(FLASH_CMD_RDSR, 0, FLASH_ADDR_LEN_NOADDR, 1, 0) && !(FLASH_BUF[0] & 0x1C))
      return 1;
  }
  return 0;
}

static uint8_t flash_unlock(void) {
  if (!flash_unlocked)
    flash_unlocked = flash_clear_protection();
  return flash_unlocked;
}

static uint8_t flash_erase_sector(uint32_t address) {
  if (!flash_unlock() || !flash_write_enable() ||
      !flash_cmd(FLASH_CMD_SECTOR_ERASE, address, FLASH_ADDR_LEN_3BYTE, 0, 0) || !flash_wait_write()) {
    flash_unlocked = 0;
    return 0;
  }
  return 1;
}

static uint8_t flash_program_page(uint32_t address, uint16_t length) {
  if (!length || length > 256 || ((address & 0xFF) + length) > 0x100)
    return 0;
  if (!flash_unlock() || !flash_write_enable() ||
      !flash_cmd(FLASH_CMD_PAGE_PROGRAM, address, FLASH_ADDR_LEN_3BYTE, length, 1) || !flash_wait_write()) {
    flash_unlocked = 0;
    return 0;
  }
  return 1;
}

static uint8_t flash_read(uint32_t address, __xdata uint8_t *dst, uint16_t length) {
  while (length) {
    uint16_t chunk = length < FLASH_BUFFER_SIZE ? length : FLASH_BUFFER_SIZE;
    if (!flash_cmd(FLASH_CMD_READ, address, FLASH_ADDR_LEN_3BYTE, chunk, 0))
      return 0;
    xmemcpy(dst, FLASH_BUF, chunk);
    dst += chunk;
    address += chunk;
    length -= chunk;
  }
  return 1;
}

#endif
