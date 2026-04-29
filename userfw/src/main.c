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

#define XR8(addr)  (*(__xdata volatile uint8_t *)(addr))

__sfr __at(0xA8) IE;

#define COOKIE_ADDR    0x5FF8
#define COOKIE_MAGIC   0xDF0BC0DEUL

static void uart_putc(uint8_t c) { while (REG_UART_TFBF == 0); REG_UART_THR = c; }
static void uart_puts(__code const char *s) { while (*s) uart_putc(*s++); }

/* USB descriptors — vendor class, no endpoints (control transfers only). */
static __code const uint8_t dev_desc[] = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
    0x09, 0x02, 0x12, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
};
static __code const uint8_t bos_desc[] = {
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};
static __code const uint8_t str0[]   = { 0x04, 0x03, 0x09, 0x04 };
static __code const uint8_t str1[]   = { 0x0A, 0x03, 't',0,'i',0,'n',0,'y',0 };
static __code const uint8_t str2[]   = { 0x10, 0x03, 'u',0,'s',0,'e',0,'r',0,' ',0,'f',0,'w',0 };
static __code const uint8_t str3[]   = { 0x08, 0x03, '0',0,'0',0,'1',0 };
static __code const uint8_t str_em[] = { 0x02, 0x03 };

static void handle_get_descriptor(uint8_t type, uint8_t idx, uint16_t wlen) {
    __code const uint8_t *src; uint8_t len;
    if      (type == USB_DESC_TYPE_DEVICE) { src = dev_desc; len = sizeof(dev_desc); }
    else if (type == USB_DESC_TYPE_CONFIG) { src = cfg_desc; len = sizeof(cfg_desc); }
    else if (type == USB_DESC_TYPE_BOS)    { src = bos_desc; len = sizeof(bos_desc); }
    else if (type == USB_DESC_TYPE_STRING) {
        if      (idx == 0) { src = str0; len = sizeof(str0); }
        else if (idx == 1) { src = str1; len = sizeof(str1); }
        else if (idx == 2) { src = str2; len = sizeof(str2); }
        else if (idx == 3) { src = str3; len = sizeof(str3); }
        else               { src = str_em; len = sizeof(str_em); }
    } else { return; }
    usb_desc_copy(src, len);
    usb_send_data(wlen < len ? wlen : len);
}

static void enter_dfu(void) {
    uart_puts("[DFU!]\n");
    XR8(COOKIE_ADDR + 0) = (uint8_t)(COOKIE_MAGIC      );
    XR8(COOKIE_ADDR + 1) = (uint8_t)(COOKIE_MAGIC >>  8);
    XR8(COOKIE_ADDR + 2) = (uint8_t)(COOKIE_MAGIC >> 16);
    XR8(COOKIE_ADDR + 3) = (uint8_t)(COOKIE_MAGIC >> 24);
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

    if (bmReq == 0x00 && bReq == 0x05) {
        REG_USB_INT_MASK_9090 = 0x80 | (wValL & 0x7F);
        REG_USB_EP_CTRL_91D0 = 0x02;
        send_zlp();
    } else if (bmReq == 0x80 && bReq == 0x06) {
        handle_get_descriptor(wValH, wValL, wLen);
    } else if (bmReq == 0x00 && bReq == 0x09) {
        REG_USB_MSC_CFG = 0x00;
        send_zlp();
    } else if (bmReq == 0xC0 && bReq == 0xE4) {
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        uint16_t rlen = wLen > 64 ? 64 : wLen;
        for (uint16_t i = 0; i < rlen; i++) DESC_BUF[i] = XR8(addr + i);
        send_data(rlen);
    } else if (bmReq == 0x40 && bReq == 0xE5) {
        uint16_t addr = ((uint16_t)wValH << 8) | wValL;
        XR8(addr) = REG_USB_SETUP_WIDX_L;
        send_zlp();
    } else if (bmReq == 0x40 && bReq == 0xEC) {
        send_zlp();
        enter_dfu();        /* never returns */
    } else {
        if (wLen == 0) send_zlp();
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
