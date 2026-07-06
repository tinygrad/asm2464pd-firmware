/* ASM2464PD bootstub.
 *
 * Lives at CODE 0x0000-0x23FF. On boot, validates a userfw image at flash
 * 0x4000 (header + crc32 + body) and copies it into CODE 0x2400+, or drops
 * into DFU mode where the host can re-flash via USB EP0 control transfers. */

#include "types.h"
#include "registers.h"

__sfr __at(0x87) PCON;
__sfr __at(0xA8) IE;

#define DFU_COOKIE          (*(__xdata volatile uint32_t *)0x5FF8)
#define DFU_COOKIE_MAGIC    0xDF0BC0DEUL

#define USERFW_FLASH_OFFSET   0x4000UL
#define USERFW_HEADER_SIZE    0x40UL
#define USERFW_HEADER_CRC_LEN 0x20U
#define USERFW_CODE_BASE      0x2400
#define USERFW_BODY_LIMIT     0xDC10UL
#define USERFW_FLASH_END      (USERFW_FLASH_OFFSET + USERFW_HEADER_SIZE + USERFW_BODY_LIMIT)
#define SECTOR_SIZE           0x1000UL
#define USERFW_ERASE_END      ((USERFW_FLASH_END + (SECTOR_SIZE - 1)) & ~(SECTOR_SIZE - 1))

typedef struct {
  uint8_t  magic[4];
  uint8_t  gitversion[24];
  uint32_t body_len;
  uint32_t crc;
  uint8_t  _pad[28];
} userfw_hdr_t;

/* ---- UART ---- */

static void uart_putc(uint8_t ch) { while (!REG_UART_TFBF); REG_UART_THR = ch; }
static void uart_puts(__code const char *s) { while (*s) uart_putc(*s++); }

/* ---- flash ---- */

static void flash_init(void) {
  REG_CPU_EXEC_STATUS_2 = 0x04;
  REG_FLASH_DIV = 4;
}

static uint8_t flash_wait_done(void) {
  uint16_t g;
  for (g = 0; (REG_FLASH_CSR & 0x01) && g < 0x4000; g++);
  return !(REG_FLASH_CSR & 0x01);
}

static uint8_t flash_read(uint32_t addr, __xdata uint8_t *buf, uint16_t len) {
  REG_FLASH_MODE = (REG_FLASH_MODE & 0xF0) | 0x02;
  REG_FLASH_ADDR_HI = (uint8_t)(addr >> 16);
  REG_FLASH_ADDR_MD = (uint8_t)(addr >> 8);
  REG_FLASH_ADDR_LO = (uint8_t)addr;
  REG_FLASH_DATA_PAGE_CNT = (uint8_t)(len >> 8);
  REG_FLASH_DATA_BYTE_OFS = (uint8_t)len;
  REG_FLASH_CSR = 0x01;
  if (!flash_wait_done()) return 0;
  __xdata uint8_t *p = buf;
  uint16_t i;
  for (i = 0; i < len; i++) p[i] = FLASH_BUF[i];
  return 1;
}

static uint8_t flash_unlock(void) {
  REG_FLASH_CSR = 0x06;
  flash_wait_done();
  return (REG_FLASH_CSR & 0x04) != 0;
}

static uint8_t flash_erase_sector(uint32_t addr) {
  REG_FLASH_MODE = (REG_FLASH_MODE & 0xF0) | 0x08;
  REG_FLASH_ADDR_HI = (uint8_t)(addr >> 16);
  REG_FLASH_ADDR_MD = (uint8_t)(addr >> 8);
  REG_FLASH_ADDR_LO = (uint8_t)addr;
  REG_FLASH_CSR = 0x01;
  return flash_wait_done();
}

static uint8_t flash_program_page(uint32_t addr, uint16_t len) {
  REG_FLASH_MODE = (REG_FLASH_MODE & 0xF0) | 0x04;
  REG_FLASH_ADDR_HI = (uint8_t)(addr >> 16);
  REG_FLASH_ADDR_MD = (uint8_t)(addr >> 8);
  REG_FLASH_ADDR_LO = (uint8_t)addr;
  REG_FLASH_DATA_PAGE_CNT = (uint8_t)(len >> 8);
  REG_FLASH_DATA_BYTE_OFS = (uint8_t)len;
  REG_FLASH_CSR = 0x01;
  return flash_wait_done();
}

/* ---- USB (EP0 only, polled) ---- */

#define USB_VID 0xADD1
#define USB_PID 0xB007

static uint8_t is_usb2;

static void usb_send_zlp(void) {
  REG_USB_EP0_LEN_H = 0; REG_USB_EP0_LEN_L = 0;
  REG_USB_DMA_TRIGGER = USB_DMA_SEND;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
}

static void usb_send_data(uint16_t len) {
  REG_USB_EP0_LEN_H = (uint8_t)(len >> 8);
  REG_USB_EP0_LEN_L = (uint8_t)(len & 0xFF);
  REG_USB_DMA_TRIGGER = USB_DMA_SEND;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
}

static void usb_handle_set_address(uint8_t addr) {
  REG_USB_ADDR_CTRL = addr;
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
}

static __code const uint8_t usb_dev_desc[] = {
  0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
  USB_VID & 0xFF, USB_VID >> 8, USB_PID & 0xFF, USB_PID >> 8,
  0x01, 0x00, 0x01, 0x02, 0x00, 0x01
};

static __code const uint8_t usb_cfg_desc[] = {
  0x09, 0x02, 0x12, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
  0x09, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00
};

static __code const char usb_str_mfg[] = "tiny";
static __code const char usb_str_product[] = "bootstub";

static void usb_send_string(__code const char *s, uint8_t max_len) {
  uint8_t n = 0;
  while (s[n] && n < max_len) n++;
  __xdata uint8_t *buf = (__xdata uint8_t *)USB_CTRL_BUF_BASE;
  buf[0] = 2 + 2 * n;
  buf[1] = 0x03;
  uint8_t i;
  for (i = 0; i < n; i++) { buf[2 + 2*i] = s[i]; buf[2 + 2*i + 1] = 0; }
  usb_send_data(2 + 2 * n);
}

static void usb_handle_get_descriptor(uint8_t type, uint8_t idx, uint16_t wLen) {
  __xdata uint8_t *buf = (__xdata uint8_t *)USB_CTRL_BUF_BASE;
  if (type == 1) {
    uint8_t i;
    for (i = 0; i < sizeof(usb_dev_desc); i++) buf[i] = usb_dev_desc[i];
    usb_send_data(sizeof(usb_dev_desc));
  } else if (type == 2) {
    uint8_t i;
    for (i = 0; i < sizeof(usb_cfg_desc); i++) buf[i] = usb_cfg_desc[i];
    usb_send_data(sizeof(usb_cfg_desc));
  } else if (type == 3) {
    if (idx == 0) { buf[0] = 4; buf[1] = 0x03; buf[2] = 0x09; buf[3] = 0x04; usb_send_data(4); }
    else if (idx == 1) usb_send_string(usb_str_mfg, 30);
    else if (idx == 2) usb_send_string(usb_str_product, 30);
    else usb_send_zlp();
  } else {
    usb_send_zlp();
  }
}

static void usb_init_controller(void) {
  REG_POWER_ENABLE = (REG_POWER_ENABLE & 0x7F) | 0x80;
  REG_USB_PHY_CTRL_91D1 = 0x0F;
  REG_BUF_CFG_9300 = 0x0C;
  REG_BUF_CFG_9301 = 0xC0;
  REG_BUF_CFG_9302 = 0xBF;
  REG_USB_CTRL_PHASE = 0x1F;
  REG_USB_EP_CFG1 = 0x0F;
  REG_USB_PHY_CTRL_91C1 = 0xF0;
  REG_BUF_CFG_9303 = 0x33;
  REG_USB_CONFIG = 0xE0;
  REG_USB_EP0_CFG = 0xF0;
  REG_USB_MODE = 0x01;
  REG_USB_MSC_CTRL = 0x01;
  REG_USB_PHY_CTRL_91C3 &= ~0x20;
  REG_USB_PHY_CTRL_91C0 |= 0x01;
  REG_USB_PHY_CTRL_91C0 &= ~0x01;
}

static void usb_attach_controller(void) {
  REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
}

/* ---- code write ---- */

static void code_write(uint16_t addr, uint8_t val) {
  uint8_t old = PCON;
  PCON = old | 0x10;
  *(__xdata volatile uint8_t *)addr = val;
  PCON = old;
}

/* ---- userfw loader ---- */

__xdata static uint8_t scratch[256];
__xdata static userfw_hdr_t hdr;

static uint8_t header_magic_ok(void) {
  return hdr.magic[0] == 'A' && hdr.magic[1] == '2' &&
         hdr.magic[2] == '4' && hdr.magic[3] == 'F';
}

static void crc32_step(uint32_t *c, uint8_t b) {
  uint32_t crc = *c ^ b;
  uint8_t i;
  for (i = 0; i < 8; i++)
    crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
  *c = crc;
}

static uint8_t verify_body(uint32_t total_body_len) {
  uint32_t crc = 0xFFFFFFFFUL;
  __xdata uint8_t *p = (__xdata uint8_t *)&hdr;
  uint8_t i;
  for (i = 0; i < USERFW_HEADER_CRC_LEN; i++) crc32_step(&crc, p[i]);
  uint32_t addr = USERFW_FLASH_OFFSET + sizeof(hdr);
  while (total_body_len) {
    uint16_t take = total_body_len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)total_body_len;
    if (!flash_read(addr, scratch, take)) return 0;
    for (i = 0; i < take; i++) crc32_step(&crc, scratch[i]);
    addr += take; total_body_len -= take;
  }
  return ~crc == hdr.crc;
}

static uint8_t load_region(uint32_t flash_addr, uint16_t code_base, uint32_t len) {
  while (len) {
    uint16_t take = len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)len;
    if (!flash_read(flash_addr, scratch, take)) return 0;
    uint16_t i;
    for (i = 0; i < take; i++) code_write(code_base + i, scratch[i]);
    flash_addr += take; code_base += take; len -= take;
  }
  return 1;
}

/* ---- DFU ---- */

__xdata static uint32_t xfer_addr;
enum { XOP_NONE = 0, XOP_ERASE, XOP_WRITE };
__xdata static uint8_t xfer_op;
__xdata static uint32_t xfer_op_addr;
__xdata static uint16_t xfer_op_len;

static void stall_ep0(void) {
  xfer_op = XOP_NONE;
  REG_USB_DMA_TRIGGER = USB_DMA_STALL;
}

static void cpu_reset(void) {
  REG_CPU_RESET = CPU_RESET_TRIGGER;
  while (1);
}

static uint8_t dfu_range_ok(uint32_t addr, uint32_t len, uint32_t end) {
  if (len == 0) return 0;
  if (addr < USERFW_FLASH_OFFSET || addr > end) return 0;
  return len <= (end - addr);
}

static void handle_setup(void) {
  uint8_t bmReq = REG_USB_SETUP_BMREQ;
  uint8_t bReq = REG_USB_SETUP_BREQ;
  uint8_t wValL = REG_USB_SETUP_WVAL_L;
  uint8_t wValH = REG_USB_SETUP_WVAL_H;
  uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

  if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
    usb_handle_set_address(wValL);
  } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
    usb_handle_get_descriptor(wValH, wValL, wLen);
  } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
    usb_send_zlp();
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB0) {
    xfer_op = XOP_ERASE;
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB1) {
    xfer_addr = ((uint32_t)REG_USB_SETUP_WIDX_L << 16) | ((uint16_t)wValH << 8) | wValL;
    usb_send_zlp();
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB2) {
    if (wLen == 0 || wLen > 64) { stall_ep0(); return; }
    if (!dfu_range_ok(xfer_addr, wLen, USERFW_FLASH_END)) { stall_ep0(); return; }
    if ((uint16_t)(xfer_addr & 0xFF) + wLen > 0x100) { stall_ep0(); return; }
    xfer_op_addr = xfer_addr;
    xfer_op_len = wLen;
    xfer_op = XOP_WRITE;
  } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB3) {
    uint16_t n = (wLen > 64) ? 64 : wLen;
    if (n == 0) { usb_send_zlp(); return; }
    if (!flash_read(xfer_addr, scratch, n)) { stall_ep0(); return; }
    __xdata uint8_t *buf = (__xdata uint8_t *)USB_CTRL_BUF_BASE;
    uint16_t i;
    for (i = 0; i < n; i++) buf[i] = scratch[i];
    xfer_addr += n;
    usb_send_data(n);
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xEB) {
    usb_send_zlp();
    cpu_reset();
  } else {
    stall_ep0();
  }
}

static void dfu_loop(void) {
  uart_puts("[DFU]\n");
  is_usb2 = 0;
  usb_init_controller();
  IE = 0x80;
  usb_attach_controller();
  xfer_op = XOP_NONE;
  xfer_addr = 0;

  while (1) {
    uint8_t s = REG_USB_PERIPH_STATUS;
    if (s & USB_PERIPH_CONTROL) {
      uint8_t phase = REG_USB_CTRL_PHASE;
      if (phase & USB_CTRL_PHASE_SETUP) {
        xfer_op = XOP_NONE;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
        handle_setup();
      } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
        REG_USB_DMA_TRIGGER = USB_DMA_RECV;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
      } else if ((phase & USB_CTRL_PHASE_DATA_IN) || (phase & USB_CTRL_PHASE_STAT_IN)) {
        uint8_t r = XOP_NONE;
        if (xfer_op == XOP_WRITE) {
          uint32_t addr = xfer_op_addr;
          uint16_t len = xfer_op_len;
          xfer_op = XOP_NONE;
          __xdata uint8_t *buf = (__xdata uint8_t *)USB_CTRL_BUF_BASE;
          uint16_t i;
          for (i = 0; i < len; i++) FLASH_BUF[i] = buf[i];
          if (flash_program_page(addr, len)) { xfer_addr = addr + len; r = 1; }
        } else if (xfer_op == XOP_ERASE) {
          xfer_op = XOP_NONE;
          __xdata uint8_t *buf = (__xdata uint8_t *)USB_CTRL_BUF_BASE;
          uint32_t addr = *(__xdata uint32_t *)buf;
          uint32_t len = *(__xdata uint32_t *)(buf + 4);
          if (!(addr & (SECTOR_SIZE - 1)) && !(len & (SECTOR_SIZE - 1)) &&
              dfu_range_ok(addr, len, USERFW_ERASE_END)) {
            uint32_t off;
            for (off = 0; off < len; off += SECTOR_SIZE)
              if (!flash_erase_sector(addr + off)) break;
            r = (off >= len) ? 1 : 0;
          }
        }
        if (r == 1) usb_send_zlp();
        else if (xfer_op == XOP_NONE) {
          if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
          REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
        }
      } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
      }
    } else if (s & USB_PERIPH_BUS_RESET) {
      xfer_op = XOP_NONE;
      REG_USB_PHY_CTRL_91D1 = REG_USB_PHY_CTRL_91D1;
    } else if (s & USB_PERIPH_LINK_EVENT) {
      uint8_t e = REG_BUF_CFG_9300;
      if (e & BUF_CFG_9300_SS_FAIL) {
        is_usb2 = 1;
        REG_CPU_MODE = CPU_MODE_USB2;
        REG_USB_PHY_CTRL_91C0 = 0x10;
      }
      REG_BUF_CFG_9300 = e;
    }
  }
}

/* ---- entry ---- */

void main(void) {
  REG_UART_LCR &= ~LCR_PARITY_MASK;
  uart_puts("\n[BS]\n");

  flash_init();
  flash_unlock();

  if (DFU_COOKIE == DFU_COOKIE_MAGIC) {
    DFU_COOKIE = 0;
    dfu_loop();
  }

  if (!flash_read(USERFW_FLASH_OFFSET, (__xdata uint8_t *)&hdr, sizeof(hdr)) || !header_magic_ok()) {
    uart_puts("[BS bad-magic]\n");
    dfu_loop();
  }

  if (hdr.body_len > USERFW_BODY_LIMIT) {
    uart_puts("[BS bad-size]\n");
    dfu_loop();
  }

  if (!verify_body(hdr.body_len)) {
    uart_puts("[BS bad-crc]\n");
    dfu_loop();
  }

  uint32_t base = USERFW_FLASH_OFFSET + sizeof(hdr);
  if (hdr.body_len && !load_region(base, USERFW_CODE_BASE, hdr.body_len)) {
    uart_puts("[BS load-fail]\n");
    dfu_loop();
  }

  DFU_COOKIE = 0;
  uart_puts("[BS->APP]\n");
  __asm ljmp 0x2400 __endasm;
}
