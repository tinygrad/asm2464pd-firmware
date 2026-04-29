/* ASM2464PD bootstub.
 *
 * Loaded by the chip's mask-ROM bootloader from SPI flash 0x100, lives at
 * CODE 0x0000-0x3FFF. On boot, validates a user firmware image at flash
 * 0x4000 (header + sha256 + body) and either copies it into CODE 0x4000+
 * and jumps there, or drops into DFU mode where the host can re-flash via
 * USB. Shares register defs + USB helpers with handmade/main.c via usb.h
 * to keep the two firmwares in lock-step. */

#include "types.h"
#include "registers.h"
#include "usb.h"
#include "flash.h"
#include "sha256.h"

__sfr __at(0x87) PCON;       /* bit 4 = MEMSEL: redirects MOVX writes to CODE */
__sfr __at(0x96) PSBANK;     /* CODE bank select (bit 0)                      */
__sfr __at(0xA8) IE;

#define XR8(addr) (*(__xdata volatile uint8_t *)(addr))

#define COOKIE_ADDR             0x5FF8
#define COOKIE_MAGIC            0xDF0BC0DEUL

#define USERFW_FLASH_OFFSET     0x4000UL
#define USERFW_HDR_SIZE         0x40
#define USERFW_HASH_OFFSET      0x20
#define USERFW_COMMON_BASE      0x4000
#define USERFW_COMMON_LIMIT     0x4000   /* 16 KB at 0x4000-0x7FFF        */
#define USERFW_BANK_BASE        0x8000
#define USERFW_BANK_LIMIT       0x8000   /* 32 KB per bank at 0x8000-0xFFFF*/

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

static uint8_t cookie_says_dfu(void) {
    return XR8(COOKIE_ADDR + 0) == (uint8_t)(COOKIE_MAGIC      ) &&
           XR8(COOKIE_ADDR + 1) == (uint8_t)(COOKIE_MAGIC >>  8) &&
           XR8(COOKIE_ADDR + 2) == (uint8_t)(COOKIE_MAGIC >> 16) &&
           XR8(COOKIE_ADDR + 3) == (uint8_t)(COOKIE_MAGIC >> 24);
}
static void cookie_clear(void) {
    XR8(COOKIE_ADDR + 0) = 0; XR8(COOKIE_ADDR + 1) = 0;
    XR8(COOKIE_ADDR + 2) = 0; XR8(COOKIE_ADDR + 3) = 0;
}

/* ---- userfw header & loader ------------------------------------------- */

__xdata static sha256_t hash_ctx;
__xdata static uint8_t  scratch[256];
__xdata static uint8_t  hdr_buf[USERFW_HDR_SIZE];
__xdata static uint8_t  digest_buf[32];

static uint8_t load_header(__xdata uint8_t hdr[USERFW_HDR_SIZE]) {
    flash_read(USERFW_FLASH_OFFSET, hdr, USERFW_HDR_SIZE);
    return hdr[0] == 'A' && hdr[1] == 'S' && hdr[2] == 'P' && hdr[3] == '2';
}

static uint32_t hdr_u32(__xdata const uint8_t *hdr, uint8_t off) {
    return  (uint32_t)hdr[off]                  |
           ((uint32_t)hdr[off + 1] <<  8)       |
           ((uint32_t)hdr[off + 2] << 16)       |
           ((uint32_t)hdr[off + 3] << 24);
}

static uint8_t verify_body(__xdata const uint8_t hdr[USERFW_HDR_SIZE],
                           uint32_t total_body_len) {
    sha256_init(&hash_ctx);
    sha256_update(&hash_ctx, hdr, USERFW_HASH_OFFSET);

    uint32_t addr = USERFW_FLASH_OFFSET + USERFW_HDR_SIZE;
    uint32_t left = total_body_len;
    while (left) {
        uint16_t take = left > sizeof(scratch) ? sizeof(scratch) : (uint16_t)left;
        flash_read(addr, scratch, take);
        sha256_update(&hash_ctx, scratch, take);
        addr += take; left -= take;
    }

    sha256_final(&hash_ctx, digest_buf);
    for (uint8_t i = 0; i < 32; i++)
        if (digest_buf[i] != hdr[USERFW_HASH_OFFSET + i]) return 0;
    return 1;
}

static void load_region(uint32_t flash_addr, uint16_t code_base, uint32_t len) {
    while (len) {
        uint16_t take = len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)len;
        flash_read(flash_addr, scratch, take);
        for (uint16_t i = 0; i < take; i++) code_write(code_base + i, scratch[i]);
        flash_addr += take; code_base += take; len -= take;
    }
}

static void boot_userfw(__xdata const uint8_t hdr[USERFW_HDR_SIZE]) {
    uint32_t common_len = hdr_u32(hdr, 0x08);
    uint32_t bank0_len  = hdr_u32(hdr, 0x0C);
    uint32_t bank1_len  = hdr_u32(hdr, 0x10);

    uint32_t base = USERFW_FLASH_OFFSET + USERFW_HDR_SIZE;

    PSBANK = 0;
    if (common_len) load_region(base, USERFW_COMMON_BASE, common_len);
    base += common_len;

    if (bank0_len) load_region(base, USERFW_BANK_BASE, bank0_len);
    base += bank0_len;

    if (bank1_len) {
        PSBANK = 1;
        load_region(base, USERFW_BANK_BASE, bank1_len);
        PSBANK = 0;
    }

    cookie_clear();
    uart_puts("[BS->APP]\n");
    __asm
        ljmp 0x4000
    __endasm;
}

/* ---- DFU-mode USB descriptors (VID:PID 0xADD1:0xB007) ------------------ */

static __code const uint8_t dev_desc[] = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0xD1, 0xAD, 0x07, 0xB0, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
/* Vendor class with one bulk OUT EP — the host uses bulk OUT to load the
 * SPI page buffer at XDATA 0x7000, matching handmade/e4_flash.py exactly.
 * No bulk IN: flash readback uses the 0xE4 control transfer. */
static __code const uint8_t cfg_desc[] = {
    /* wTotalLength = 0x09 + 0x09 + 0x07 = 0x19 */
    0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x00,
    /* EP2 OUT, bulk, wMaxPacketSize=512 (USB2 HS) */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00,
};
static __code const uint8_t bos_desc[] = {
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};
static __code const uint8_t str0[]    = { 0x04, 0x03, 0x09, 0x04 };
static __code const uint8_t str_mfr[] = { 0x0A, 0x03, 't',0,'i',0,'n',0,'y',0 };
static __code const uint8_t str_prd[] = { 0x12, 0x03, 'b',0,'o',0,'o',0,'t',0,'s',0,'t',0,'u',0,'b',0 };
static __code const uint8_t str_sn[]  = { 0x08, 0x03, '0',0,'0',0,'1',0 };
static __code const uint8_t str_em[]  = { 0x02, 0x03 };

static void handle_get_descriptor(uint8_t type, uint8_t idx, uint16_t wlen) {
    __code const uint8_t *src; uint8_t len;
    if      (type == USB_DESC_TYPE_DEVICE) { src = dev_desc; len = sizeof(dev_desc); }
    else if (type == USB_DESC_TYPE_CONFIG) { src = cfg_desc; len = sizeof(cfg_desc); }
    else if (type == USB_DESC_TYPE_BOS)    { src = bos_desc; len = sizeof(bos_desc); }
    else if (type == USB_DESC_TYPE_STRING) {
        if      (idx == 0) { src = str0;    len = sizeof(str0); }
        else if (idx == 1) { src = str_mfr; len = sizeof(str_mfr); }
        else if (idx == 2) { src = str_prd; len = sizeof(str_prd); }
        else if (idx == 3) { src = str_sn;  len = sizeof(str_sn); }
        else               { src = str_em;  len = sizeof(str_em); }
    } else { return; }
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
        for (uint16_t i = 0; i < rlen; i++) DESC_BUF[i] = XR8(addr + i);
        usb_send_data(rlen);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        XR8(addr) = REG_USB_SETUP_WIDX_L;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE7) {
        /* Vendor LJMP. Host uses 0xE7 → 0x0000 to re-enter the bootstub
         * after flashing a new userfw. */
        usb_send_zlp();
        XR8(0x9106) = (uint8_t)(wValL);
        XR8(0x9107) = (uint8_t)(wValH);
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

    if (cookie_says_dfu()) {
        cookie_clear();
        dfu_loop();
    }

    flash_init();

    if (!load_header(hdr_buf)) {
        uart_puts("[BS bad-magic]\n");
        dfu_loop();
    }

    uint32_t common_len = hdr_u32(hdr_buf, 0x08);
    uint32_t bank0_len  = hdr_u32(hdr_buf, 0x0C);
    uint32_t bank1_len  = hdr_u32(hdr_buf, 0x10);

    if (common_len > USERFW_COMMON_LIMIT ||
        bank0_len  > USERFW_BANK_LIMIT   ||
        bank1_len  > USERFW_BANK_LIMIT) {
        uart_puts("[BS bad-size]\n");
        dfu_loop();
    }

    if (!verify_body(hdr_buf, common_len + bank0_len + bank1_len)) {
        uart_puts("[BS bad-sha]\n");
        dfu_loop();
    }

    boot_userfw(hdr_buf);
}
