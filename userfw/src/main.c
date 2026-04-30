/* ASM2464PD userfw — minimal example.
 *
 * Lives at CODE 0x4000+ (loaded by the bootstub from SPI flash 0x4000+).
 * Enumerates as a vendor-class USB device with VID:PID ADD1:0001 and
 * exposes a small command surface. Add your own commands here.
 *
 * Vendor commands:
 *   0xE4  read XDATA      (host->dev xfer; up to 64 bytes)
 *   0xE5  write XDATA     (single byte; addr in wValue, byte in wIndex low)
 *   0xEC  enter DFU       (set cookie at 0x5FF8 + CC31; bootstub takes over) */

#include "types.h"
#include "registers.h"
#include "usb.h"

__sfr __at(0xA8) IE;

#define COOKIE_ADDR    0x5FF8
#define COOKIE_MAGIC   0xDF0BC0DEUL
#define COOKIE         (*(__xdata volatile uint32_t *)COOKIE_ADDR)

static void uart_putc(uint8_t c) { while (REG_UART_TFBF == 0); REG_UART_THR = c; }
static void uart_puts(__code const char *s) { while (*s) uart_putc(*s++); }

/* Vendor class, no endpoints — control transfers only. */
static __code const uint8_t cfg_desc[] = {
    0x09, 0x02, 0x12, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
};

static void handle_get_descriptor(uint8_t type, uint8_t idx, uint16_t wlen) {
    __code const uint8_t *src; uint8_t len;
    if      (type == USB_DESC_TYPE_DEVICE) { src = usb_dev_desc; len = sizeof(usb_dev_desc); }
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
                                   (idx == USB_STR_IDX_PRODUCT) ? "userfw"    :
                                   (idx == USB_STR_IDX_SERIAL)  ? "001"       : "";
            len = usb_build_string_desc(s, DESC_BUF);
        }
        usb_send_data(wlen < len ? wlen : len);
        return;
    } else return;
    usb_desc_copy(src, len);
    usb_send_data(wlen < len ? wlen : len);
}

static void enter_dfu(void) {
    uart_puts("[DFU!]\n");
    COOKIE = COOKIE_MAGIC;
    REG_CPU_RESET = 0x01;
    /* CC31 self-clears once the CPU reboots; this loop is just a guard. */
    while (1);
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
        REG_USB_MSC_CFG = 0x00;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_DEV_TO_HOST | USB_SETUP_TYPE_VENDOR) && bReq == 0xE4) {
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        uint16_t rlen = wLen > 64 ? 64 : wLen;
        for (uint16_t i = 0; i < rlen; i++) DESC_BUF[i] = XDATA_REG8(addr + i);
        usb_send_data(rlen);
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xE5) {
        XDATA_REG8(((uint16_t)wValH << 8) | wValL) = REG_USB_SETUP_WIDX_L;
        usb_send_zlp();
    } else if (bmReq == (USB_SETUP_DIR_HOST_TO_DEV | USB_SETUP_TYPE_VENDOR) && bReq == 0xEC) {
        usb_send_zlp();
        enter_dfu();        /* never returns */
    } else {
        if (wLen == 0) usb_send_zlp();
    }
}

void int0_isr(void) __interrupt(0) {
    uint8_t s = REG_INT_USB_STATUS;
    if (!(s & 0x01)) return;
    uint8_t periph = REG_USB_PERIPH_STATUS;
    if (periph & USB_PERIPH_BUS_RESET) {
        uint8_t e = REG_USB_PHY_CTRL_91D1;
        REG_USB_PHY_CTRL_91D1 = e;
    } else if (periph & USB_PERIPH_CONTROL) {
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
void int1_isr(void)   __interrupt(2) { }
void timer0_isr(void) __interrupt(1) { }
void timer1_isr(void) __interrupt(3) { }
void serial_isr(void) __interrupt(4) { }
void timer2_isr(void) __interrupt(5) { }

void main(void) {
    REG_UART_LCR &= ~0x08;
    uart_puts("\n[APP]\n");

    usb_phy_tune();
    usb_init_controller(1);
    IE = 0x80 | 0x01;       /* EA + EX0 — interrupt-driven control transfers */

    while (1) { /* everything happens in the ISR */ }
}
