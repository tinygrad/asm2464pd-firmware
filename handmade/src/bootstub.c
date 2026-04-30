/* ASM2464PD bootstub.
 *
 * Loaded by the chip's mask-ROM bootloader from SPI flash 0x100, lives at
 * CODE 0x0000-0x3FFF. On boot, validates a userfw image at flash 0x4000
 * (header + crc32 + body) and either copies it into CODE 0x4000+ and
 * jumps there, or drops into DFU mode where the host can re-flash via
 * USB. */

#include <stddef.h>

#include "types.h"
#include "registers.h"
#include "usb.h"
#include "flash.h"

__sfr __at(0x87) PCON;       /* bit 4 = MEMSEL: redirects MOVX writes to CODE */
__sfr __at(0x96) PSBANK;     /* CODE bank select (bit 0)                      */
__sfr __at(0xA8) IE;

#define COOKIE_ADDR             0x5FF8
#define COOKIE_MAGIC            0xDF0BC0DEUL

#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_COMMON_BASE      0x4000
#define USERFW_COMMON_LIMIT     0x4000   /* 16 KB at 0x4000-0x7FFF        */
#define USERFW_BANK_BASE        0x8000
#define USERFW_BANK_LIMIT       0x8000   /* 32 KB per bank at 0x8000-0xFFFF*/

/* Header (64 B) the host packs in front of the userfw body. CRC-32/IEEE
 * (zlib) covers everything from `magic` through `_pad1` plus the body. */
typedef struct {
    uint8_t  magic[4];          /* 'A','S','P','2' */
    uint32_t version;
    uint32_t common_len;
    uint32_t bank0_len;
    uint32_t bank1_len;
    uint8_t  _pad1[12];
    uint32_t crc;
    uint8_t  _pad2[28];
} userfw_hdr_t;

/* ---- helpers ----------------------------------------------------------- */

static void uart_putc(uint8_t c) { while (REG_UART_TFBF == 0); REG_UART_THR = c; }
static void uart_puts(__code const char *s) { while (*s) uart_putc(*s++); }

/* MOVX-with-MEMSEL trick: PCON bit 4 redirects MOVX writes to CODE. */
static void code_write(uint16_t addr, uint8_t val) {
    uint8_t old = PCON;
    PCON = old | 0x10;
    *(__xdata volatile uint8_t *)addr = val;
    PCON = old;
}

#define COOKIE  (*(__xdata volatile uint32_t *)COOKIE_ADDR)

/* ---- userfw header & loader ------------------------------------------- */

__xdata static uint8_t       scratch[256];
__xdata static userfw_hdr_t  hdr;

static uint8_t load_header(void) {
    flash_read(USERFW_FLASH_OFFSET, (__xdata uint8_t *)&hdr, sizeof(hdr));
    return hdr.magic[0] == 'A' && hdr.magic[1] == 'S' &&
           hdr.magic[2] == 'P' && hdr.magic[3] == '2';
}

/* Single-byte CRC-32/IEEE step. */
static void crc32_step(uint32_t *c, uint8_t b) {
    uint32_t crc = *c ^ b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    *c = crc;
}

/* CRC-32/IEEE (zlib) over the header bytes up to `crc`, plus the body. */
static uint8_t verify_body(uint32_t total_body_len) {
    uint32_t crc = 0xFFFFFFFFUL;
    __xdata const uint8_t *p = (__xdata const uint8_t *)&hdr;
    for (uint8_t i = 0; i < offsetof(userfw_hdr_t, crc); i++) crc32_step(&crc, p[i]);

    uint32_t addr = USERFW_FLASH_OFFSET + sizeof(hdr);
    while (total_body_len) {
        uint16_t take = total_body_len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)total_body_len;
        flash_read(addr, scratch, take);
        for (uint16_t i = 0; i < take; i++) crc32_step(&crc, scratch[i]);
        addr += take; total_body_len -= take;
    }
    return ~crc == hdr.crc;
}

static void load_region(uint32_t flash_addr, uint16_t code_base, uint32_t len) {
    while (len) {
        uint16_t take = len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)len;
        flash_read(flash_addr, scratch, take);
        for (uint16_t i = 0; i < take; i++) code_write(code_base + i, scratch[i]);
        flash_addr += take; code_base += take; len -= take;
    }
}

static void boot_userfw(void) {
    uint32_t base = USERFW_FLASH_OFFSET + sizeof(hdr);

    PSBANK = 0;
    if (hdr.common_len) load_region(base, USERFW_COMMON_BASE, hdr.common_len);
    base += hdr.common_len;

    if (hdr.bank0_len) load_region(base, USERFW_BANK_BASE, hdr.bank0_len);
    base += hdr.bank0_len;

    if (hdr.bank1_len) {
        PSBANK = 1;
        load_region(base, USERFW_BANK_BASE, hdr.bank1_len);
        PSBANK = 0;
    }

    COOKIE = 0;
    uart_puts("[BS->APP]\n");
    __asm
        ljmp 0x4000
    __endasm;
}

/* ---- DFU-mode USB descriptors (VID:PID 0xADD1:0xB007) ------------------ */

static __code const uint8_t dev_desc[] = {
    0x12, 0x01, U16_LE(0x0200), 0x00, 0x00, 0x00, 0x40,
    U16_LE(0xADD1), U16_LE(0xB007), U16_LE(USB_BCD_DEVICE),
    USB_STR_IDX_MFG, USB_STR_IDX_PRODUCT, USB_STR_IDX_SERIAL, 0x01,
};
/* Vendor class with one bulk OUT EP — the host uses bulk OUT to load the
 * SPI page buffer at XDATA 0x7000, matching handmade/e4_flash.py exactly.
 * No bulk IN: flash readback uses the 0xE4 control transfer. */
static __code const uint8_t cfg_desc[] = {
    /* wTotalLength = 0x09 + 0x09 + 0x07 = 0x19 */
    0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x00,
    /* EP2 OUT, bulk, wMaxPacketSize=512 (USB2 HS) */
    0x07, 0x05, 0x02, 0x02, U16_LE(512), 0x00,
};

static void handle_get_descriptor(uint8_t type, uint8_t idx, uint16_t wlen) {
    __code const uint8_t *src; uint8_t len;
    if      (type == USB_DESC_TYPE_DEVICE) { src = dev_desc;     len = sizeof(dev_desc); }
    else if (type == USB_DESC_TYPE_CONFIG) { src = cfg_desc;     len = sizeof(cfg_desc); }
    else if (type == USB_DESC_TYPE_BOS)    { src = usb_bos_desc; len = sizeof(usb_bos_desc); }
    else if (type == USB_DESC_TYPE_STRING) {
        if (idx == USB_STR_IDX_LANG) {
            DESC_BUF[0] = 4; DESC_BUF[1] = 0x03;
            DESC_BUF[2] = USB_LANG_ID & 0xFF;
            DESC_BUF[3] = (USB_LANG_ID >> 8) & 0xFF;
            len = 4;
        } else {
            __code const char *s = (idx == USB_STR_IDX_MFG)     ? USB_STR_MFG :
                                   (idx == USB_STR_IDX_PRODUCT) ? "bootstub"  :
                                   (idx == USB_STR_IDX_SERIAL)  ? "001"       : "";
            len = usb_build_string_desc(s, DESC_BUF);
        }
        usb_send_data(wlen < len ? wlen : len);
        return;
    } else return;
    usb_desc_copy(src, len);
    usb_send_data(wlen < len ? wlen : len);
}

static void handle_setup(void) {
    uint8_t bmReq = REG_USB_SETUP_BMREQ;
    uint8_t bReq  = REG_USB_SETUP_BREQ;
    uint8_t wValL = REG_USB_SETUP_WVAL_L;
    uint8_t wValH = REG_USB_SETUP_WVAL_H;
    uint16_t wLen = ((uint16_t)REG_USB_SETUP_WLEN_H << 8) | REG_USB_SETUP_WLEN_L;

    if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_ADDRESS) {
        usb_handle_set_address(wValL);
    } else if (bmReq == USB_SETUP_DIR_DEV_TO_HOST && bReq == USB_REQ_GET_DESCRIPTOR) {
        handle_get_descriptor(wValH, wValL, wLen);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
        /* Disable MSC engine, clear bulk EPs — same sequence handmade fw uses
         * to expose raw bulk OUT to the host. After this, libusb bulk_transfer
         * to EP 0x02 deposits payload at XDATA 0x7000. */
        REG_USB_MSC_CFG = 0x00;
        REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_IN;
        REG_USB_EP_CFG2 = USB_EP_CFG2_CLEAR_OUT;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
        /* Read XDATA — host drives SPI flash controller through this. */
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        uint16_t rlen = wLen > 64 ? 64 : wLen;
        for (uint16_t i = 0; i < rlen; i++) DESC_BUF[i] = XDATA_REG8(addr + i);
        usb_send_data(rlen);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        XDATA_REG8(addr) = REG_USB_SETUP_WIDX_L;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE7) {
        /* Vendor LJMP to wValue. Host uses 0xE7 → 0x0000 to re-enter the
         * bootstub after flashing a new userfw. wValue is already in the
         * SETUP shadow at 0x9106/0x9107, just load it into DPTR and jmp. */
        usb_send_zlp();
        __asm
            mov  dptr, #0x9106
            movx a, @dptr
            mov  r0, a
            mov  dptr, #0x9107
            movx a, @dptr
            mov  dph, a
            mov  dpl, r0
            clr  a
            jmp  @a+dptr
        __endasm;
    } else {
        if (wLen == 0) usb_send_zlp();
    }
}

/* ---- DFU loop --------------------------------------------------------- */

static void dfu_loop(void) {
    uart_puts("[DFU]\n");
    usb_phy_tune();
    usb_init_controller(1);
    /* IE.EA must be on for the USB controller to push events to PERIPH_STATUS;
     * we leave individual interrupt sources off and poll. */
    IE = 0x80;

    while (1) {
        uint8_t s = REG_USB_PERIPH_STATUS;
        if (s & USB_PERIPH_BUS_RESET) {
            uint8_t e = REG_USB_PHY_CTRL_91D1;
            REG_USB_PHY_CTRL_91D1 = e;
        } else if (s & USB_PERIPH_LINK_EVENT) {
            uint8_t e = REG_BUF_CFG_9300;
            REG_BUF_CFG_9300 = e;
        } else if (s & USB_PERIPH_CONTROL) {
            uint8_t phase = REG_USB_CTRL_PHASE;
            if (phase & USB_CTRL_PHASE_SETUP) {
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
                handle_setup();
            } else if (phase & USB_CTRL_PHASE_STAT_OUT) {
                REG_USB_DMA_TRIGGER = USB_DMA_RECV;
                REG_USB_CTRL_PHASE  = USB_CTRL_PHASE_STAT_OUT;
            } else if ((phase & USB_CTRL_PHASE_DATA_IN) || (phase & USB_CTRL_PHASE_STAT_IN)) {
                if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
            } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
            }
        }
    }
}

/* ---- entry ------------------------------------------------------------ */

void main(void) {
    REG_UART_LCR &= ~0x08;
    uart_puts("\n[BS]\n");

    if (COOKIE == COOKIE_MAGIC) {
        COOKIE = 0;
        dfu_loop();
    }

    flash_init();

    if (!load_header()) {
        uart_puts("[BS bad-magic]\n");
        dfu_loop();
    }

    if (hdr.common_len > USERFW_COMMON_LIMIT ||
        hdr.bank0_len  > USERFW_BANK_LIMIT   ||
        hdr.bank1_len  > USERFW_BANK_LIMIT) {
        uart_puts("[BS bad-size]\n");
        dfu_loop();
    }

    if (!verify_body(hdr.common_len + hdr.bank0_len + hdr.bank1_len)) {
        uart_puts("[BS bad-crc]\n");
        dfu_loop();
    }

    boot_userfw();
}
