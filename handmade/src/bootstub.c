/* ASM2464PD bootstub.
 *
 * Loaded by the chip's mask-ROM bootloader from SPI flash 0x100, lives at
 * CODE 0x0000-0x23FF. On boot, validates a userfw image at flash 0x4000
 * (header + crc32 + body) and either copies it into CODE 0x2400+ and
 * jumps there, or drops into DFU mode where the host can re-flash via
 * USB. */

#include "types.h"
#include "registers.h"

#define BOOTSTUB 1
#define USB_PID                 0xB007
#define USB_STR_PRODUCT         "bootstub"
#define USB_BCD_DEVICE          0x0002   /* >= 0x0002: 512-byte EP0 DFU payloads at SS */
#define USB_DFU_EP0_ONLY        1
#include "usb.h"
#include "flash.h"

__sfr __at(0x87) PCON;       /* bit 4 = MEMSEL: redirects MOVX writes to CODE */
__sfr __at(0xA8) IE;

static uint8_t is_usb2;

/* ---- helpers ----------------------------------------------------------- */

/* MOVX-with-MEMSEL trick: PCON bit 4 redirects MOVX writes to CODE. */
static void code_write(uint16_t addr, uint8_t val) {
    uint8_t old = PCON;
    /* Keep MEMSEL scoped to this store; SDCC may use MOVX for temporaries. */
    PCON = old | 0x10;
    *(__xdata volatile uint8_t *)addr = val;
    PCON = old;
}

/* ---- userfw header & loader ------------------------------------------- */

__xdata static uint8_t       scratch[512];
__xdata static userfw_hdr_t  hdr;

static void dfu_loop(void);

/* DFU stays on EP0 control transfers.
 *
 * Vendor commands: 0xB0 erase, 0xB1 set addr, 0xB2 write, 0xB3 read,
 * 0xEB reset. Write/erase are confined to the userfw region; reads are
 * allowed over the 24-bit SPI flash address space. Write/read payloads are
 * one EP0 packet: 64 bytes at high speed, 512 at SuperSpeed (bcdDevice
 * >= 0x0002 advertises 512-byte support to the host flasher). */

__xdata static uint32_t xfer_addr;       /* auto-advancing cursor */

/* Operations whose payload arrives via DATA_OUT — processed in the
 * STAT_IN branch of dfu_loop after the host's data has landed in
 * DESC_BUF, BEFORE the IN-status ZLP, so the host's
 * libusb_control_transfer() blocks until the SPI op retires. */
enum { XOP_NONE = 0, XOP_ERASE, XOP_WRITE };
__xdata static uint8_t  xfer_op;
__xdata static uint32_t xfer_op_addr;    /* captured WRITE address */
__xdata static uint16_t xfer_op_len;     /* WRITE: byte count to program (ERASE reads its own params from DESC_BUF) */

static void clear_pending_xfer(void) {
    xfer_op      = XOP_NONE;
    xfer_op_addr = 0;
    xfer_op_len  = 0;
}

static void stall_ep0(void) {
    clear_pending_xfer();
    REG_USB_DMA_TRIGGER = USB_DMA_STALL;
}

static void cpu_reset(void) {
    (void)usb_wait_ep0_dma_idle();
    REG_CPU_RESET = CPU_RESET_TRIGGER;
    while (1) ;
}

static uint8_t dfu_range_ok(uint32_t addr, uint32_t len, uint32_t end) {
    if (len == 0) return 0;
    if (addr < USERFW_FLASH_OFFSET || addr > end) return 0;
    return len <= (end - addr);
}

static uint8_t load_header(void) {
    if (!flash_read(USERFW_FLASH_OFFSET, (__xdata uint8_t *)&hdr, sizeof(hdr))) return 0;
    return userfw_header_magic_ok(&hdr);
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
    for (uint8_t i = 0; i < USERFW_HEADER_CRC_LEN; i++) crc32_step(&crc, p[i]);

    uint32_t addr = USERFW_FLASH_OFFSET + sizeof(hdr);
    while (total_body_len) {
        uint16_t take = total_body_len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)total_body_len;
        if (!flash_read(addr, scratch, take)) return 0;
        for (uint16_t i = 0; i < take; i++) crc32_step(&crc, scratch[i]);
        addr += take; total_body_len -= take;
    }
    return ~crc == hdr.crc;
}

static uint8_t load_region(uint32_t flash_addr, uint16_t code_base, uint32_t len) {
    while (len) {
        uint16_t take = len > sizeof(scratch) ? sizeof(scratch) : (uint16_t)len;
        if (!flash_read(flash_addr, scratch, take)) return 0;
        for (uint16_t i = 0; i < take; i++) code_write(code_base + i, scratch[i]);
        flash_addr += take; code_base += take; len -= take;
    }
    return 1;
}

static void boot_userfw(void) {
    uint32_t base = USERFW_FLASH_OFFSET + sizeof(hdr);

    if (hdr.body_len && !load_region(base, USERFW_CODE_BASE, hdr.body_len)) goto load_fail;

    DFU_COOKIE = 0;
    uart_puts("[BS->APP]\n");
    __asm
        ljmp 0x2400
    __endasm;

load_fail:
    uart_puts("[BS load-fail]\n");
    dfu_loop();
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
        usb_handle_get_descriptor(is_usb2, wValH, wValL, wLen);
    } else if (bmReq == USB_SETUP_DIR_HOST_TO_DEV && bReq == USB_REQ_SET_CONFIGURATION) {
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB0) {
        if (wLen != 8) { stall_ep0(); return; }
        xfer_op = XOP_ERASE;
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB1) {
        xfer_addr     = ((uint32_t)REG_USB_SETUP_WIDX_L << 16) | ((uint16_t)wValH << 8) | wValL;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xB2) {
        /* At most one EP0 packet: 64 bytes at high speed, 512 at SuperSpeed
         * (a multi-packet control-OUT overwrites packet 1 in DESC_BUF).
         * SPI page-program chunking happens in run_deferred_xfer_op. */
        uint16_t maxw = is_usb2 ? 64 : 512;
        if (wLen == 0 || wLen > maxw) { stall_ep0(); return; }
        if (!dfu_range_ok(xfer_addr, wLen, USERFW_FLASH_END)) { stall_ep0(); return; }
        xfer_op_addr = xfer_addr;
        xfer_op_len = wLen;
        xfer_op     = XOP_WRITE;
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xB3) {
        uint16_t maxr = is_usb2 ? 64 : 512;
        uint16_t n = (wLen > maxr) ? maxr : wLen;
        if (n == 0) { usb_send_zlp(); return; }
        /* Do not refill DESC_BUF while the previous EP0 IN DMA can still own it. */
        if (!usb_wait_ep0_dma_idle()) { stall_ep0(); return; }
        /* Hardware testing found the direct 0x7000 -> 0x9e00 copy path unreliable. */
        if (!flash_read(xfer_addr, scratch, n)) { stall_ep0(); return; }
        xmemcpy(DESC_BUF, scratch, n);
        xfer_addr += n;
        usb_send_data(n);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xEB) {
        usb_send_zlp();
        cpu_reset();
    } else {
        stall_ep0();
    }
}

/* ---- DFU loop --------------------------------------------------------- */

enum { XR_NOOP = 0, XR_OK, XR_STALL };

/* Run an OUT-with-data op after its payload lands in DESC_BUF. */
static uint8_t run_deferred_xfer_op(void) {
    if (xfer_op == XOP_WRITE) {
        uint32_t addr = xfer_op_addr;
        uint16_t len = xfer_op_len;
        clear_pending_xfer();
        if (!usb_wait_ep0_dma_idle()) {
            stall_ep0();
            return XR_STALL;
        }
        if (!dfu_range_ok(addr, len, USERFW_FLASH_END)) {
            stall_ep0();
            return XR_STALL;
        }
        /* Unlock BEFORE staging the payload into FLASH_BUF: flash_unlock()
         * may DMA a status byte (and, on a re-lock, WRSR scratch) into
         * FLASH_BUF, which would corrupt bytes we are about to program.
         * With flash already unlocked, flash_program_page's own unlock is a
         * no-op and leaves the staged payload intact. */
        if (!flash_unlock()) {
            stall_ep0();
            return XR_STALL;
        }
        /* SPI page program is capped at 256 bytes and must not cross a
         * 256-byte page; split the packet accordingly. */
        {
            uint16_t off = 0;
            while (off < len) {
                uint16_t chunk = (uint16_t)(0x100 - (uint8_t)((addr + off) & 0xFF));
                if (chunk > (uint16_t)(len - off)) chunk = (uint16_t)(len - off);
                xmemcpy(FLASH_BUF, DESC_BUF + off, chunk);
                if (!flash_program_page(addr + off, chunk)) {
                    stall_ep0();
                    return XR_STALL;
                }
                off += chunk;
            }
        }
        xfer_addr     = addr + len;
        return XR_OK;
    }
    if (xfer_op == XOP_ERASE) {
        clear_pending_xfer();
        if (!usb_wait_ep0_dma_idle()) {
            stall_ep0();
            return XR_STALL;
        }
        uint32_t addr = *(__xdata const uint32_t *)DESC_BUF;
        uint32_t len  = *(__xdata const uint32_t *)(DESC_BUF + 4);
        if ((addr & (SECTOR_SIZE - 1)) || (len & (SECTOR_SIZE - 1)) ||
            !dfu_range_ok(addr, len, USERFW_ERASE_END)) {
            stall_ep0();
            return XR_STALL;
        }
        for (uint32_t off = 0; off < len; off += SECTOR_SIZE) {
            if (!flash_erase_sector(addr + off)) {
                stall_ep0();
                return XR_STALL;
            }
        }
        return XR_OK;
    }
    return XR_NOOP;
}

static void dfu_loop(void) {
    uart_puts("[DFU]\n");
    usb_phy_tune();
    is_usb2 = 0;
    usb_init_controller(0);
    /* IE.EA must be on for the USB controller to push events to PERIPH_STATUS;
     * we leave individual interrupt sources off and poll. */
    IE = 0x80;
    usb_attach_controller();

    xfer_op       = XOP_NONE;
    xfer_op_addr  = 0;
    xfer_op_len   = 0;
    xfer_addr     = 0;

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
                uint8_t r = run_deferred_xfer_op();
                if (r == XR_OK) {
                    usb_send_zlp();
                } else if (r == XR_STALL) {
                    /* stall_ep0() already armed USB_DMA_STALL. Ack the phase
                     * bits so we do NOT re-enter on the next poll and issue
                     * USB_DMA_STATUS_COMPLETE, which would overwrite the
                     * stall and report the failed op to the host as success. */
                    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
                } else {  /* XR_NOOP */
                    if (phase & USB_CTRL_PHASE_STAT_IN) REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
                    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN | USB_CTRL_PHASE_STAT_IN;
                }
            } else if (phase & USB_CTRL_PHASE_DATA_OUT) {
                REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
            }
        } else if (s & USB_PERIPH_BUS_RESET) {
            clear_pending_xfer();
            /* Drop back to the default USB address. The device address lives
             * in firmware-managed INT_MASK_9090[6:0] and persists across a
             * host bus reset (e.g. the host rebooting while the card sits in
             * DFU); without this the device keeps filtering on the stale
             * address and never answers the fresh address-0 enumeration,
             * stranding DFU until a physical power cycle. */
            REG_USB_INT_MASK_9090 = USB_INT_MASK_GLOBAL;
            xfer_addr = 0;
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

/* ---- entry ------------------------------------------------------------ */

void main(void) {
    REG_UART_LCR &= ~LCR_PARITY_MASK;
    uart_puts("\n[BS]\n");

    /* Reset shared DMA and endpoint state before boot-time flash reads. */
    REG_DMA_CONFIG = DMA_CONFIG_DISABLE;
    usb_init_endpoint_state();
    flash_init();
    if (!flash_unlock()) uart_puts("[BS flash-lock]\n");

    if (DFU_COOKIE == DFU_COOKIE_MAGIC) {
        DFU_COOKIE = 0;
        dfu_loop();
    }

    if (!load_header()) {
        uart_puts("[BS bad-magic]\n");
        dfu_loop();
    }

    /* body_len == 0 would pass the CRC (it only covers the header then)
     * and jump into uninitialized CODE RAM on a cold boot. */
    if (hdr.body_len == 0 || hdr.body_len > USERFW_BODY_LIMIT) {
        uart_puts("[BS bad-size]\n");
        dfu_loop();
    }

    if (!verify_body(hdr.body_len)) {
        uart_puts("[BS bad-crc]\n");
        dfu_loop();
    }

    /* Wedge guard: the image is CRC-valid (flashed correctly), so a failure
     * to reach a healthy state is a bad firmware release, not bad flash. If
     * it has failed BOOT_MAX_ATTEMPTS times in a row, stay in DFU so the field
     * can reflash without the FTDI strap. Counted only for images we actually
     * jump to; a healthy app clears the count via boot_mark_healthy(). */
    {
        uint8_t attempts = ((BOOT_TRACK & BOOT_TRACK_MASK) == BOOT_TRACK_MARK)
                           ? (uint8_t)(BOOT_TRACK & 0xFF) : 0;
        if (attempts >= BOOT_MAX_ATTEMPTS) {
            uart_puts("[BS wedge-guard]\n");
            dfu_loop();
        }
        BOOT_TRACK = BOOT_TRACK_MARK | (uint32_t)(uint8_t)(attempts + 1);
    }

    boot_userfw();
}
