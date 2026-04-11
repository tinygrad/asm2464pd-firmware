#include "types.h"
#include "registers.h"

static void bank1_write(uint16_t addr, uint8_t val) {
  DPX = 0x01;
  XDATA_REG8(addr) = val;
  DPX = 0x00;
}

static void bank1_or_bits(uint16_t addr, uint8_t mask) {
  DPX = 0x01;
  XDATA_REG8(addr) |= mask;
  DPX = 0x00;
}

static void pcie_apply_rxphy_lane_stage0(uint16_t lane_base) {
  bank1_write(lane_base + 0x64, 0x0F);
  bank1_write(lane_base + 0xBF, 0xB0);
  bank1_write(lane_base + 0xBF, 0x30);
  bank1_write(lane_base + 0x67, 0xD0);
}

static void pcie_apply_rxphy_preamble(uint16_t lane_base, uint16_t companion_base) {
  bank1_write(lane_base + 0x9B, 0x10);
  bank1_write(lane_base + 0x9B, 0x90);

  bank1_write(companion_base + 0x00, 0xC0);
  bank1_write(companion_base + 0x04, 0x2B);
  bank1_write(companion_base + 0x06, 0x41);
  bank1_write(companion_base + 0x07, 0x81);
  bank1_write(companion_base + 0x59, 0x41);
  bank1_write(companion_base + 0x5A, 0x00);
  bank1_write(companion_base + 0x0A, 0x11);
  bank1_write(companion_base + 0x42, 0xBF);
  bank1_write(companion_base + 0x05, 0x8A);
}

static void pcie_apply_rxphy_lane_stage1(uint16_t lane_base) {
  bank1_write(lane_base + 0x40, 0x01);
  bank1_write(lane_base + 0x01, 0x7C);
  bank1_write(lane_base + 0x11, 0x7C);
  bank1_write(lane_base + 0x21, 0xF0);
  bank1_write(lane_base + 0x31, 0xF0);
}

static void pcie_apply_rxphy_lane_stage2(uint16_t lane_base) {
  bank1_write(lane_base + 0x34, 0x07);
  bank1_write(lane_base + 0x35, 0x6D);
  bank1_write(lane_base + 0x26, 0x3A);
  bank1_write(lane_base + 0x36, 0x57);
  bank1_write(lane_base + 0x06, 0x73);
  bank1_write(lane_base + 0x16, 0x73);
  bank1_write(lane_base + 0x26, 0x7A);
  bank1_write(lane_base + 0x36, 0x77);
  bank1_write(lane_base + 0x37, 0x6F);
  bank1_write(lane_base + 0x46, 0x74);
  bank1_write(lane_base + 0x46, 0x64);
  bank1_write(lane_base + 0x46, 0x44);
  bank1_write(lane_base + 0x46, 0x04);
  bank1_write(lane_base + 0x1D, 0x40);
  bank1_write(lane_base + 0x2D, 0x40);
  bank1_write(lane_base + 0x3D, 0x4A);
  bank1_write(lane_base + 0x1A, 0x44);
  bank1_write(lane_base + 0x3A, 0x4D);
  bank1_write(lane_base + 0x1B, 0x4A);
  bank1_write(lane_base + 0x3B, 0x4F);
  bank1_write(lane_base + 0x0C, 0x17);
  bank1_write(lane_base + 0x1C, 0x17);
  bank1_write(lane_base + 0x02, 0xE8);
  bank1_write(lane_base + 0x22, 0x30);
  bank1_write(lane_base + 0x32, 0x30);
  bank1_write(lane_base + 0x6C, 0x0C);
  bank1_write(lane_base + 0x6C, 0x6C);
}

static void pcie_apply_rxphy_companion_profile(uint16_t companion_base) {
  bank1_write(companion_base + 0x20, 0x60);
  bank1_write(companion_base + 0x20, 0x10);
  bank1_write(companion_base + 0x21, 0xF2);
  bank1_write(companion_base + 0x21, 0x42);
  bank1_write(companion_base + 0x22, 0xC6);
  bank1_write(companion_base + 0x22, 0x26);
  bank1_write(companion_base + 0x23, 0xED);
  bank1_write(companion_base + 0x23, 0xA9);
  bank1_write(companion_base + 0x24, 0xEC);
  bank1_write(companion_base + 0x24, 0xCC);
  bank1_write(companion_base + 0x25, 0x7D);
  bank1_write(companion_base + 0x25, 0x41);
  bank1_write(companion_base + 0x26, 0xDF);
  bank1_write(companion_base + 0x27, 0xDA);
  bank1_write(companion_base + 0x27, 0xDE);
  bank1_write(companion_base + 0x28, 0x20);
  bank1_write(companion_base + 0x28, 0x00);
  bank1_write(companion_base + 0x29, 0x61);
  bank1_write(companion_base + 0x29, 0x31);
  bank1_write(companion_base + 0x2A, 0xA4);
  bank1_write(companion_base + 0x2A, 0x74);
  bank1_write(companion_base + 0x2B, 0xE9);
  bank1_write(companion_base + 0x2B, 0xC9);
}

static void pcie_apply_rxphy_tail(uint16_t lane_base, uint16_t companion_base, uint8_t tail_0b) {
  bank1_write(lane_base + 0x87, 0x10);
  bank1_write(lane_base + 0x88, 0x08);
  bank1_write(lane_base + 0x07, 0xEF);
  bank1_write(lane_base + 0x17, 0xEF);
  bank1_write(lane_base + 0x37, 0x4F);
  bank1_write(lane_base + 0x12, 0xA8);
  bank1_write(lane_base + 0x2C, 0x17);
  bank1_write(companion_base + 0x26, 0xD1);

  bank1_write(lane_base + 0x0B, tail_0b);
  bank1_write(lane_base + 0x2A, 0x46);
  bank1_write(lane_base + 0x0D, 0x5A);
  bank1_write(lane_base + 0x1D, 0x50);
  bank1_write(lane_base + 0x2D, 0x50);
  bank1_write(lane_base + 0x3D, 0x5A);
}

static void pcie_apply_x2_rxphy_tuning(void) {
  static __code const uint8_t lane_hi[] = {0x78, 0x79, 0x7A, 0x7B};
  static __code const uint8_t companion_hi[] = {0x60, 0x64, 0x68, 0x6C};
  static __code const uint8_t tail_0b[] = {0x56, 0x56, 0x5C, 0x5C};
  uint8_t i;

  for (i = 0; i < 4; i++) {
    pcie_apply_rxphy_preamble((uint16_t)lane_hi[i] << 8, (uint16_t)companion_hi[i] << 8);
  }

  for (i = 0; i < 4; i++) {
    pcie_apply_rxphy_lane_stage0((uint16_t)lane_hi[i] << 8);
  }

  for (i = 0; i < 4; i++) {
    pcie_apply_rxphy_lane_stage1((uint16_t)lane_hi[i] << 8);
  }

  pcie_apply_rxphy_lane_stage2(0x7800);
  pcie_apply_rxphy_companion_profile(0x6000);
  bank1_write(0x7885, 0xA6);

  pcie_apply_rxphy_lane_stage2(0x7900);
  pcie_apply_rxphy_companion_profile(0x6400);
  bank1_write(0x7985, 0xA6);

  pcie_apply_rxphy_lane_stage2(0x7A00);
  pcie_apply_rxphy_companion_profile(0x6800);
  bank1_write(0x7A85, 0xA6);

  pcie_apply_rxphy_lane_stage2(0x7B00);
  pcie_apply_rxphy_companion_profile(0x6C00);
  bank1_write(0x7B85, 0xA6);

  for (i = 0; i < 4; i++) {
    pcie_apply_rxphy_tail((uint16_t)lane_hi[i] << 8, (uint16_t)companion_hi[i] << 8, tail_0b[i]);
  }
}
