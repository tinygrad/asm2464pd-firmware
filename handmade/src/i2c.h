#ifndef __I2C_H__
#define __I2C_H__

#include "types.h"
#include "registers.h"

/*=== HW I2C master ===
 *
 * Generic 7-bit I2C master via the peripheral at C870-C87F + C805 + XRAM at
 * 0xE800. Four transaction shapes:
 *
 *   PING    : ADDR+W, WLEN=0, RLEN=0          -> S, addr|W, STOP
 *   POINT   : ADDR+W, WLEN=0, RLEN>0, DATA0=r -> S, addr|W, r, STOP
 *   WRITE16 : ADDR+W, WLEN>0, DATA0=r,
 *             XRAM[0..1]=msb,lsb               -> S, addr|W, r, msb, lsb, STOP
 *   READ    : ADDR+R, WLEN=0, RLEN=N,
 *             C805[4]=1                        -> S, addr|R, N bytes, STOP
 *
 * WLEN is a 0/non-zero flag, not a count. RLEN is honored strictly. */

static uint8_t i2c_fire_wait(void) {
  uint8_t csr = 0;
  uint16_t timeout = 0xFFFF;
  REG_I2C_CSR = 0xFF;
  REG_I2C_CSR = 0x01;
  do {
    csr = REG_I2C_CSR;
    if (csr & I2C_CSR_DONE) break;
  } while (--timeout);
  REG_I2C_CSR = I2C_CSR_DONE;
  return csr;
}

static void i2c_init(void) {
  REG_GPIO_CTRL(9)  = I2C_GPIO_ALT_SCL;
  REG_GPIO_CTRL(10) = I2C_GPIO_ALT_SDA;
  REG_I2C_MODE = 0xC4;
  REG_I2C_CLK_LO = 0x18;
  REG_I2C_CLK_HI = 0x4A;
}

/* Park the slave's register pointer at `reg`. Non-destructive — slave
 * register is not modified. CSR=I2C_CSR_WR_ADDR_ACK on success. */
static uint8_t i2c_point(uint8_t addr7, uint8_t reg) {
  REG_I2C_ADDR        = (addr7 << 1) | 0;
  REG_I2C_DATA0       = reg;
  REG_I2C_WLEN        = 0;
  REG_I2C_RLEN        = 1;   /* non-zero flips ping into "emit DATA0 then STOP" */
  REG_I2C_DMA_SRC_HI  = 0;
  REG_I2C_DMA_SRC_LO  = 0;
  REG_I2C_DMA_DEST_HI = 0;
  REG_I2C_DMA_DEST_LO = 0;
  return i2c_fire_wait();
}

static uint8_t i2c_write_reg16(uint8_t addr7, uint8_t reg, uint16_t val) {
  (&REG_I2C_XRAM)[0] = (val >> 8) & 0xFF;
  (&REG_I2C_XRAM)[1] = val & 0xFF;
  REG_I2C_ADDR        = (addr7 << 1) | 0;
  REG_I2C_DATA0       = reg;
  REG_I2C_WLEN        = 1;
  REG_I2C_RLEN        = 0;
  REG_I2C_DMA_SRC_HI  = 0;
  REG_I2C_DMA_SRC_LO  = 0;
  REG_I2C_DMA_DEST_HI = 0;
  REG_I2C_DMA_DEST_LO = 0;
  return i2c_fire_wait();
}

static uint8_t i2c_read(uint8_t addr7, uint8_t rlen, uint8_t *out) {
  uint8_t csr, dma_en_save, i;
  REG_I2C_ADDR        = (addr7 << 1) | 1;
  REG_I2C_DATA0       = 0;
  REG_I2C_WLEN        = 0;
  REG_I2C_RLEN        = rlen;
  REG_I2C_DMA_SRC_HI  = 0;
  REG_I2C_DMA_SRC_LO  = 0;
  REG_I2C_DMA_DEST_HI = 0;
  REG_I2C_DMA_DEST_LO = 0;
  dma_en_save = REG_I2C_DMA_ENABLE;
  REG_I2C_DMA_ENABLE = dma_en_save | 0x10;
  csr = i2c_fire_wait();
  REG_I2C_DMA_ENABLE = dma_en_save & (uint8_t)~0x10;
  if (csr == I2C_CSR_RD_DONE) {
    for (i = 0; i < rlen; i++) out[i] = (&REG_I2C_XRAM)[i];
  }
  return csr;
}

/*=== INA231 power monitor (I2C addr 0x45) ===
 * 1 LSB bus = 1.25 mV, 1 LSB shunt = 2.5 uV. Stock CFG 0x4127 = AVG=1,
 * CT=1.1 ms, continuous both channels. Shunt calibrated against a known
 * 2.242 A load on chestnut rev F. */
#define INA231_ADDR        0x45
#define INA231_REG_CFG     0x00
#define INA231_REG_SHUNT   0x01
#define INA231_REG_BUS     0x02
#define INA231_CFG_STOCK   0x4127
#define INA231_SHUNT_UOHM  2408

static void ina231_init(void) {
  i2c_write_reg16(INA231_ADDR, INA231_REG_CFG, INA231_CFG_STOCK);
}

static uint8_t ina231_read_u16(uint8_t reg, uint16_t *out) {
  uint8_t rx[2];
  if (i2c_point(INA231_ADDR, reg) != I2C_CSR_WR_ADDR_ACK) return 0;
  if (i2c_read(INA231_ADDR, 2, rx) != I2C_CSR_RD_DONE) return 0;
  *out = ((uint16_t)rx[0] << 8) | rx[1];
  return 1;
}

#endif
