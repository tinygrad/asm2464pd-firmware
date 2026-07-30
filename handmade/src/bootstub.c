/* userfw @ SPI 0x4000: A24F | git[23] | ABI8 | len32le | crc32le | zero[28] | body.
 * CRC covers bytes 0x00..0x1f + body; body loads at CODE 0x3000. */

#include "types.h"
#include "registers.h"
#include "util.h"

#define BOOT_ABI_VERSION        0x01U
#define BOOT_TRACK_MARK         0xB0075B00UL
#define BOOT_TRACK_MASK         0xFFFFFF00UL
#define BOOT_MAX_ATTEMPTS       3

#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_HEADER_CRC_LEN   0x20U
#define USERFW_CODE_BASE        0x3000U
#define USERFW_BODY_LIMIT       0xD000UL
#define USERFW_SECTOR_SIZE      0x1000UL

typedef struct {
    uint8_t  magic[4];
    uint8_t  gitversion[23];
    uint8_t  boot_abi;
    uint32_t body_len;
    uint32_t crc;
    uint8_t  reserved[28];
} userfw_hdr_t;

#define USERFW_FLASH_END        (USERFW_FLASH_OFFSET + sizeof(userfw_hdr_t) + USERFW_BODY_LIMIT)
#define USERFW_ERASE_END        ((USERFW_FLASH_END + USERFW_SECTOR_SIZE - 1) & \
                                 ~(USERFW_SECTOR_SIZE - 1))

static void xmemcpy(__xdata uint8_t *dst, __xdata const uint8_t *src,
                    uint16_t length) {
    while (length--) *dst++ = *src++;
}

#define BOOTSTUB 1
#include "usb.h"

__sfr __at(0x87) PCON;       /* bit 4 = MEMSEL: redirects MOVX writes to CODE */
__sfr __at(0xA8) IE;

static uint8_t is_usb2;

#define DFU_PROTOCOL_VERSION    0x01
#define DFU_INFO_SIZE           18

__xdata static uint8_t usb_configuration;

static void code_write(uint16_t addr, uint8_t val) {
    uint8_t old = PCON;
    /* Keep MEMSEL scoped to this store; SDCC may use MOVX for temporaries. */
    PCON = old | 0x10;
    *(__xdata volatile uint8_t *)addr = val;
    PCON = old;
}

__xdata static uint8_t       scratch[512];
__xdata static userfw_hdr_t  hdr;

static void dfu_loop(void);

__xdata static uint32_t xfer_addr;

enum { XOP_NONE = 0, XOP_ERASE, XOP_WRITE };
__xdata static uint8_t  xfer_op;
__xdata static uint16_t xfer_op_len;

static void clear_pending_xfer(void) {
    xfer_op = XOP_NONE;
}

static void stall_ep0(void) {
    clear_pending_xfer();
    REG_USB_DMA_TRIGGER = USB_DMA_STALL;
}

static void desc_put_u16(uint8_t off, uint16_t value) {
    DESC_BUF[off] = (uint8_t)value;
    DESC_BUF[(uint8_t)(off + 1)] = (uint8_t)(value >> 8);
}

static void desc_put_u32(uint8_t off, uint32_t value) {
    DESC_BUF[off] = (uint8_t)value;
    DESC_BUF[(uint8_t)(off + 1)] = (uint8_t)(value >> 8);
    DESC_BUF[(uint8_t)(off + 2)] = (uint8_t)(value >> 16);
    DESC_BUF[(uint8_t)(off + 3)] = (uint8_t)(value >> 24);
}

static void send_dfu_info(uint16_t wLen) {
    if (wLen != DFU_INFO_SIZE) { stall_ep0(); return; }
    DESC_BUF[0] = 'A'; DESC_BUF[1] = '2'; DESC_BUF[2] = '4'; DESC_BUF[3] = 'D';
    DESC_BUF[4] = DFU_PROTOCOL_VERSION;
    DESC_BUF[5] = BOOT_ABI_VERSION;
    desc_put_u16(6, is_usb2 ? 64 : 512);
    desc_put_u16(8, (uint16_t)USERFW_SECTOR_SIZE);
    desc_put_u32(10, USERFW_FLASH_OFFSET);
    desc_put_u32(14, USERFW_FLASH_END);
    usb_send_data(DFU_INFO_SIZE);
}

static uint8_t dfu_range_ok(uint32_t addr, uint32_t len, uint32_t end) {
    return len && addr >= USERFW_FLASH_OFFSET && addr <= end && len <= end - addr;
}

static void crc32_step(uint32_t *c, uint8_t b) {
    uint32_t crc = *c ^ b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    *c = crc;
}

/* CRC the same bytes written to CODE RAM, avoiding a validate/load race. */
static uint8_t load_and_verify_body(uint32_t total_body_len) {
    uint32_t crc = 0xFFFFFFFFUL;
    __xdata const uint8_t *p = (__xdata const uint8_t *)&hdr;
    for (uint8_t i = 0; i < USERFW_HEADER_CRC_LEN; i++) crc32_step(&crc, p[i]);

    uint32_t addr = USERFW_FLASH_OFFSET + sizeof(hdr);
    uint16_t code_base = USERFW_CODE_BASE;
    while (total_body_len) {
        uint16_t take = total_body_len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)total_body_len;
        if (!flash_read(addr, scratch, take)) return 0;
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

static void boot_userfw(void) {
    DFU_COOKIE = 0;
    uart_puts("[BS->APP]\n");
    __asm
        ljmp 0x3000
    __endasm;
}

static void handle_setup(void) {
    uint8_t bmReq = REG_USB_SETUP_BMREQ;
    uint8_t bReq  = REG_USB_SETUP_BREQ;
    uint8_t wValL = REG_USB_SETUP_WVAL_L;
    uint8_t wValH = REG_USB_SETUP_WVAL_H;
    uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS &&
        wValH == 0 && wLen == 0) {
        usb_handle_set_address(wValL);
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
        usb_handle_get_descriptor(is_usb2, wValH, wValL, wLen);
    } else if ((bmReq == USB_SETUP_DIR_DEV_TO_HOST ||
                bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) ||
                bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_ENDPOINT)) &&
               bReq == USB_REQ_GET_STATUS && wLen == 2) {
        DESC_BUF[0] = (bmReq == USB_SETUP_DIR_DEV_TO_HOST) ? 0x01 : 0x00;
        DESC_BUF[1] = 0;
        usb_send_data(2);
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_CONFIGURATION &&
               wLen == 1) {
        DESC_BUF[0] = usb_configuration;
        usb_send_data(1);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION &&
               wValH == 0 && wValL <= 1 && wLen == 0) {
        usb_configuration = wValL;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_RECIP_INTERFACE) &&
               bReq == USB_REQ_GET_INTERFACE && wLen == 1) {
        DESC_BUF[0] = 0;
        usb_send_data(1);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_RECIP_INTERFACE) &&
               bReq == USB_REQ_SET_INTERFACE && wValH == 0 && wValL == 0 && wLen == 0) {
        usb_send_zlp();
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_SEL && wLen == 6) {
        /* Accept and ignore the payload. */
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ISOCH_DELAY &&
               wLen == 0) {
        usb_send_zlp();
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV &&
               (bReq == USB_REQ_SET_FEATURE || bReq == USB_REQ_CLEAR_FEATURE) &&
               wValH == 0 && (wValL == 48 || wValL == 49) && wLen == 0) {
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB0) {
        if (wLen != 8) { stall_ep0(); return; }
        xfer_op = XOP_ERASE;
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB1) {
        if (wLen != 0) { stall_ep0(); return; }
        xfer_addr = ((uint32_t)REG_USB_SETUP_WIDX_L << 16) | ((uint16_t)wValH << 8) | wValL;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB2) {
        /* Multi-packet control OUT overwrites packet 1 in DESC_BUF. */
        uint16_t maxw = is_usb2 ? 64 : 512;
        if (wLen == 0 || wLen > maxw) { stall_ep0(); return; }
        if (!dfu_range_ok(xfer_addr, wLen, USERFW_FLASH_END)) { stall_ep0(); return; }
        xfer_op_len = wLen;
        xfer_op = XOP_WRITE;
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB3) {
        uint16_t maxr = is_usb2 ? 64 : 512;
        uint16_t n = wLen;
        if (n == 0 || n > maxr || !dfu_range_ok(xfer_addr, n, USERFW_FLASH_END)) {
            stall_ep0(); return;
        }
        if (!usb_wait_ep0_dma_idle()) { stall_ep0(); return; }
        /* Direct FLASH_BUF -> DESC_BUF copies are unreliable on hardware. */
        if (!flash_read(xfer_addr, scratch, n)) { stall_ep0(); return; }
        xmemcpy(DESC_BUF, scratch, n);
        xfer_addr += n;
        usb_send_data(n);
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB4) {
        send_dfu_info(wLen);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xEB) {
        if (wLen != 0) { stall_ep0(); return; }
        usb_send_zlp();
        (void)usb_wait_ep0_dma_idle();
        cpu_reset();
    } else {
        stall_ep0();
    }
}

static uint8_t run_deferred_xfer_op(void) {
    if (xfer_op == XOP_WRITE) {
        uint32_t addr = xfer_addr;
        uint16_t len = xfer_op_len;
        clear_pending_xfer();
        if (!usb_wait_ep0_dma_idle()) {
            stall_ep0();
            return 0;
        }
        /* Unlock DMA touches FLASH_BUF, so it must precede payload staging. */
        if (!flash_unlock()) {
            stall_ep0();
            return 0;
        }
        uint16_t off = 0;
        while (off < len) {
            uint16_t chunk = (uint16_t)(0x100 - (uint8_t)((addr + off) & 0xFF));
            if (chunk > (uint16_t)(len - off)) chunk = (uint16_t)(len - off);
            xmemcpy(FLASH_BUF, DESC_BUF + off, chunk);
            if (!flash_program_page(addr + off, chunk)) {
                stall_ep0();
                return 0;
            }
            off += chunk;
        }
        xfer_addr = addr + len;
        return 1;
    } else {
        clear_pending_xfer();
        if (!usb_wait_ep0_dma_idle()) {
            stall_ep0();
            return 0;
        }
        uint32_t addr = *(__xdata const uint32_t *)DESC_BUF;
        uint32_t len  = *(__xdata const uint32_t *)(DESC_BUF + 4);
        if ((addr & (USERFW_SECTOR_SIZE - 1)) ||
            (len & (USERFW_SECTOR_SIZE - 1)) ||
            !dfu_range_ok(addr, len, USERFW_ERASE_END)) {
            stall_ep0();
            return 0;
        }
        for (uint32_t off = 0; off < len; off += USERFW_SECTOR_SIZE) {
            if (!flash_erase_sector(addr + off)) {
                stall_ep0();
                return 0;
            }
        }
        return 1;
    }
}

static void dfu_loop(void) {
    uart_puts("[DFU]\n");
    usb_phy_tune();
    is_usb2 = 0;
    usb_reinit_controller();
    /* PERIPH_STATUS updates only while IE.EA is set. */
    IE = 0x80;
    usb_attach_controller();

    clear_pending_xfer();
    xfer_addr = 0;
    usb_configuration = 0;

    while (1) {
        uint8_t s = REG_USB_PERIPH_STATUS;
        if (s & USB_PERIPH_CONTROL) {
            uint8_t phase = REG_USB_CTRL_PHASE;
            if (phase & USB_CTRL_PHASE_SETUP) {
                clear_pending_xfer();
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
                handle_setup();
            } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
                REG_USB_DMA_TRIGGER = USB_DMA_RECV;
                REG_USB_CTRL_PHASE  = USB_CTRL_PHASE_STAT_OUT;
            } else if ((phase & USB_CTRL_PHASE_DATA_IN) || (phase & USB_CTRL_PHASE_STAT_IN)) {
                if (xfer_op != XOP_NONE) {
                    if (run_deferred_xfer_op()) {
                        usb_send_zlp();
                    } else {
                        /* Ack the phase without overwriting USB_DMA_STALL. */
                        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
                    }
                } else {
                    if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
                    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
                }
            } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
            }
        } else if (s & USB_PERIPH_BUS_RESET) {
            clear_pending_xfer();
            /* INT_MASK_9090[6:0] does not reset with the USB bus. */
            REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
            xfer_addr = 0;
            usb_configuration = 0;
            uint8_t e = REG_USB_PHY_CTRL_91D1;
            REG_USB_PHY_CTRL_91D1 = e;
        } else if (s & USB_PERIPH_LINK_EVENT) {
            uint8_t e = REG_BUF_CFG_9300;
            if (e & BUF_CFG_9300_SS_FAIL) {
                is_usb2 = 1;
                REG_CPU_MODE = CPU_MODE_USB2;
                REG_USB_PHY_CTRL_91C0 = USB_PHY_91C0_FORCE_HS;
            }
            REG_BUF_CFG_9300 = (e & BUF_CFG_9300_SS_EVENT) ? BUF_CFG_9300_SS_FAIL : e;
        }
    }
}

void main(void) {
    REG_UART_LCR &= ~LCR_PARITY_MASK;
    uart_puts("\n[BS]\n");

    REG_DMA_CONFIG = DMA_CONFIG_DISABLE;
    usb_init_endpoint_state();
    flash_init();

    if (DFU_COOKIE == DFU_COOKIE_MAGIC) {
        DFU_COOKIE = 0;
        dfu_loop();
    }

    if (!flash_read(USERFW_FLASH_OFFSET, (__xdata uint8_t *)&hdr, sizeof(hdr)) ||
        hdr.magic[0] != 'A' || hdr.magic[1] != '2' ||
        hdr.magic[2] != '4' || hdr.magic[3] != 'F') {
        uart_puts("[BS bad-magic]\n");
        dfu_loop();
    }
    if (hdr.boot_abi != BOOT_ABI_VERSION) {
        uart_puts("[BS bad-abi]\n");
        dfu_loop();
    }

    /* An empty body has a valid CRC but no reset vector. */
    if (hdr.body_len == 0 || hdr.body_len > USERFW_BODY_LIMIT) {
        uart_puts("[BS bad-size]\n");
        dfu_loop();
    }

    if (!load_and_verify_body(hdr.body_len)) {
        uart_puts("[BS bad-crc]\n");
        dfu_loop();
    }

    /* Stay in DFU after repeated jumps that never call boot_mark_healthy(). */
    uint8_t attempts = ((BOOT_TRACK & BOOT_TRACK_MASK) == BOOT_TRACK_MARK)
                       ? (uint8_t)(BOOT_TRACK & 0xFF) : 0;
    if (attempts >= BOOT_MAX_ATTEMPTS) {
        uart_puts("[BS wedge-guard]\n");
        dfu_loop();
    }
    BOOT_TRACK = BOOT_TRACK_MARK | (uint32_t)(uint8_t)(attempts + 1);

    boot_userfw();
}
