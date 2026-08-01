/* userfw @ SPI 0x4000: A24F | ABI8 | len32le | crc32le | zero[51] | body.
 * CRC covers magic, ABI, length, and body; body loads at CODE 0x3000. */

#include "registers.h"
#include "types.h"
#include "flash.h"
#include "usb.h"
#include "util.h"

#define BOOTSTUB_ABI_VERSION 0x01U

#define USERFW_FLASH_OFFSET 0x4000UL
#define USERFW_HEADER_CRC_LEN 9U
#define USERFW_CODE_BASE 0x3000U
#define USERFW_BODY_LIMIT 0xD000UL
#define USERFW_SECTOR_SIZE 0x1000UL

typedef struct {
  uint8_t magic[4];
  uint8_t bootstub_abi;
  uint32_t body_len;
  uint32_t crc;
  uint8_t reserved[51];
} userfw_hdr_t;

#define USERFW_FLASH_END (USERFW_FLASH_OFFSET + sizeof(userfw_hdr_t) + USERFW_BODY_LIMIT)
#define USERFW_ERASE_END ((USERFW_FLASH_END + USERFW_SECTOR_SIZE - 1) & ~(USERFW_SECTOR_SIZE - 1))

#define DFU_INFO_SIZE 17

static void code_write(uint16_t addr, uint8_t val) {
  uint8_t old = PCON;
  // Keep MEMSEL scoped to this store; SDCC may use MOVX for temporaries.
  PCON = old | PCON_MEMSEL;
  *(__xdata volatile uint8_t *)addr = val;
  PCON = old;
}

__xdata static uint8_t scratch[512];
__xdata static userfw_hdr_t hdr;

static void dfu_loop(void);

__xdata static uint32_t xfer_addr;

enum { XOP_NONE = 0, XOP_ERASE, XOP_WRITE, XOP_READ };
__xdata static uint8_t xfer_op;
__xdata static uint16_t xfer_op_len;

static void stall_ep0(void) {
  xfer_op = XOP_NONE;
  usb_stall_ep0();
}

static uint8_t dfu_range_ok(uint32_t addr, uint32_t len, uint32_t end) {
  return len && addr >= USERFW_FLASH_OFFSET && addr <= end && len <= end - addr;
}

static __code const uint32_t crc32_nibble[16] = {
  0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
  0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
  0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
  0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

static void crc32_step(uint32_t *c, uint8_t b) {
  uint32_t crc = *c ^ b;
  crc = (crc >> 4) ^ crc32_nibble[(uint8_t)crc & 0x0F];
  crc = (crc >> 4) ^ crc32_nibble[(uint8_t)crc & 0x0F];
  *c = crc;
}

/* CRC the same bytes written to CODE RAM, avoiding a validate/load race. */
static uint8_t load_and_verify_body(uint32_t total_body_len) {
  uint32_t crc = 0xFFFFFFFFUL;
  __xdata const uint8_t *p = (__xdata const uint8_t *)&hdr;
  for (uint8_t i = 0; i < USERFW_HEADER_CRC_LEN; i++)
    crc32_step(&crc, p[i]);

  uint32_t addr = USERFW_FLASH_OFFSET + sizeof(hdr);
  uint16_t code_base = USERFW_CODE_BASE;
  while (total_body_len) {
    uint16_t take = total_body_len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)total_body_len;
    if (!flash_read(addr, scratch, take))
      return 0;
    for (uint16_t i = 0; i < take; i++) {
      uint8_t b = scratch[i];
      code_write(code_base + i, b);
      crc32_step(&crc, b);
    }
    addr += take;
    code_base += take;
    total_body_len -= take;
  }
  return ~crc == hdr.crc;
}

static bool usb_handle_custom_setup(uint8_t bmReq, uint8_t bReq, uint8_t wValL, uint8_t wValH, uint16_t wLen) {
  if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB0) {
    if (wLen != 8) {
      stall_ep0();
      return true;
    }
    xfer_op = XOP_ERASE;
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB1) {
    if (wLen != 0) {
      stall_ep0();
      return true;
    }
    xfer_addr = ((uint32_t)REG_USB_SETUP_WIDX_L << 16) | ((uint16_t)wValH << 8) | wValL;
    usb_send_zlp();
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB2) {
    // Multi-packet control OUT overwrites packet 1 in DESC_BUF.
    if (wLen == 0 || wLen > USB_EP0_SIZE) {
      stall_ep0();
      return true;
    }
    if (!dfu_range_ok(xfer_addr, wLen, USERFW_FLASH_END)) {
      stall_ep0();
      return true;
    }
    xfer_op_len = wLen;
    xfer_op = XOP_WRITE;
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
  } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB3) {
    if (wLen == 0 || wLen > USB_EP0_SIZE || !dfu_range_ok(xfer_addr, wLen, USERFW_FLASH_END)) {
      stall_ep0();
      return true;
    }
    xfer_op_len = wLen;
    xfer_op = XOP_READ;
  } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB4) {
    // send_dfu_info
    if (wLen != DFU_INFO_SIZE) {
      stall_ep0();
      return true;
    }
    DESC_BUF[0] = 'A';
    DESC_BUF[1] = '2';
    DESC_BUF[2] = '4';
    DESC_BUF[3] = 'D';
    *(__xdata uint16_t *)(DESC_BUF + 4) = USB_EP0_SIZE;
    *(__xdata uint16_t *)(DESC_BUF + 6) = USERFW_SECTOR_SIZE;
    *(__xdata uint32_t *)(DESC_BUF + 8) = USERFW_FLASH_OFFSET;
    *(__xdata uint32_t *)(DESC_BUF + 12) = USERFW_FLASH_END;
    DESC_BUF[16] = BOOTSTUB_ABI_VERSION;
    usb_send_data(DFU_INFO_SIZE);
  } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xEB) {
    if (wLen != 0) {
      stall_ep0();
      return true;
    }
    usb_send_zlp();
    (void)usb_wait_ep0_dma_idle();
    cpu_reset();
  } else {
    return false;
  }
  return true;
}

static void run_deferred_xfer_op(void) {
  if (xfer_op == XOP_READ) {
    uint16_t len = xfer_op_len;
    xfer_op = XOP_NONE;
    if (!flash_read(xfer_addr, scratch, len)) {
      stall_ep0();
      return;
    }
    xmemcpy(DESC_BUF, scratch, len);
    xfer_addr += len;
    usb_send_data(len);
    return;
  } else if (xfer_op == XOP_WRITE) {
    uint32_t addr = xfer_addr;
    uint16_t len = xfer_op_len;
    xfer_op = XOP_NONE;
    if (!usb_wait_ep0_dma_idle()) {
      stall_ep0();
      return;
    }
    // Unlock DMA touches FLASH_BUF, so it must precede payload staging.
    if (!flash_unlock()) {
      stall_ep0();
      return;
    }
    uint16_t off = 0;
    while (off < len) {
      uint16_t chunk = (uint16_t)(0x100 - (uint8_t)((addr + off) & 0xFF));
      if (chunk > (uint16_t)(len - off))
        chunk = (uint16_t)(len - off);
      xmemcpy(FLASH_BUF, DESC_BUF + off, chunk);
      if (!flash_program_page(addr + off, chunk)) {
        stall_ep0();
        return;
      }
      off += chunk;
    }
    xfer_addr = addr + len;
  } else {
    xfer_op = XOP_NONE;
    if (!usb_wait_ep0_dma_idle()) {
      stall_ep0();
      return;
    }
    uint32_t addr = *(__xdata const uint32_t *)DESC_BUF;
    uint32_t len = *(__xdata const uint32_t *)(DESC_BUF + 4);
    if ((addr & (USERFW_SECTOR_SIZE - 1)) || (len & (USERFW_SECTOR_SIZE - 1)) ||
        !dfu_range_ok(addr, len, USERFW_ERASE_END)) {
      stall_ep0();
      return;
    }
    for (uint32_t off = 0; off < len; off += USERFW_SECTOR_SIZE) {
      if (!flash_erase_sector(addr + off)) {
        stall_ep0();
        return;
      }
    }
  }
  usb_send_zlp();
}

static void dfu_loop(void) {
  usb_phy_tune();
  usb_reinit_controller();
  // PERIPH_STATUS updates only while IE.EA is set.
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
        usb_handle_setup();
      } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
        REG_USB_DMA_TRIGGER = USB_DMA_RECV;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
      } else if ((phase & USB_CTRL_PHASE_DATA_IN) || (phase & USB_CTRL_PHASE_STAT_IN)) {
        if (xfer_op != XOP_NONE) {
          run_deferred_xfer_op();
        } else {
          if (phase & USB_CTRL_PHASE_STAT_IN)
            REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
          REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
        }
      } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
      }
    } else if (s & USB_PERIPH_BUS_RESET) {
      xfer_op = XOP_NONE;
      // INT_MASK_9090[6:0] does not reset with the USB bus.
      REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
      xfer_addr = 0;
      usb_configuration = 0;
      uint8_t e = REG_USB_PHY_CTRL_91D1;
      REG_USB_PHY_CTRL_91D1 = e;
    } else if (s & USB_PERIPH_LINK_EVENT) {
      uint8_t e = REG_BUF_CFG_9300;
      REG_BUF_CFG_9300 = (e & BUF_CFG_9300_SS_EVENT) ? BUF_CFG_9300_SS_FAIL : e;
    }
  }
}

void main(void) {
  flash_init();

  if (DFU_COOKIE == DFU_COOKIE_MAGIC) {
    DFU_COOKIE = 0;
    dfu_loop();
  }

  if (!flash_read(USERFW_FLASH_OFFSET, (__xdata uint8_t *)&hdr, sizeof(hdr)) || hdr.magic[0] != 'A' ||
      hdr.magic[1] != '2' || hdr.magic[2] != '4' || hdr.magic[3] != 'F') {
    dfu_loop();
  }
  if (hdr.bootstub_abi != BOOTSTUB_ABI_VERSION) {
    dfu_loop();
  }

  // An empty body has a valid CRC but no reset vector.
  if (hdr.body_len == 0 || hdr.body_len > USERFW_BODY_LIMIT) {
    dfu_loop();
  }

  if (!load_and_verify_body(hdr.body_len)) {
    dfu_loop();
  }

  __asm ljmp 0x3000 __endasm;
}
