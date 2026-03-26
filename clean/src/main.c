/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "globals.h"

__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;
#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01

#define DESC_BUF ((__xdata uint8_t *)USB_CTRL_BUF_BASE)
#define EP_BUF(n) XDATA_REG8(0xD800 + (n))

static void desc_copy(__code const uint8_t *src, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; i++) DESC_BUF[i] = src[i];
}

void uart_putc(uint8_t ch) { REG_UART_THR = ch; }
void uart_puts(__code const char *str) { while (*str) uart_putc(*str++); }
static void uart_puthex(uint8_t val) {
    static __code const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[val >> 4]);
    uart_putc(hex[val & 0x0F]);
}

static volatile uint8_t is_usb3;
static volatile uint8_t need_bulk_init;
static volatile uint8_t need_cbw_process;
static uint8_t cbw_tag[4];
static volatile uint8_t bulk_out_state;
static uint16_t bulk_out_addr;
static uint8_t bulk_out_len;

static void poll_bulk_events(void);

/*=== USB Control Transfer Helpers ===*/

static void complete_usb3_status(void) {
    REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
    /* Stock firmware polls 0x9092 bit 2 until clear (0xb974) */
    while (REG_USB_DMA_TRIGGER & USB_DMA_STATUS_COMPLETE) { }
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
}

static void complete_usb20_status(void) {
    REG_USB_CONFIG |= USB_CTRL_PHASE_STAT_OUT;
    REG_USB_DMA_TRIGGER = USB_DMA_RECV;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_OUT;
    REG_USB_CONFIG &= ~USB_CTRL_PHASE_STAT_OUT;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
}

static void send_zlp_ack(void) {
    if (is_usb3) {
        complete_usb3_status();
    } else {
        REG_USB_EP0_STATUS = 0x00;
        REG_USB_EP0_LEN_L = 0x00;
        REG_USB_DMA_TRIGGER = USB_DMA_SEND;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
    }
}

static void send_descriptor_data(uint8_t len) {
    REG_USB_EP0_STATUS = 0x00;
    REG_USB_EP0_LEN_L = len;
    REG_USB_DMA_TRIGGER = USB_DMA_SEND;
    REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_IN;
    if (is_usb3) complete_usb3_status();
}

/* Re-arm MSC engine to receive next CBW */
static void arm_msc(void) {
    EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
    EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
    REG_USB_MSC_LENGTH = 0x0D;
    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS &= ~0x01;
}

/*=== USB Request Handlers ===*/

static void handle_set_address(void) {
    uint8_t tmp;
    REG_USB_INT_MASK_9090 = 0x01;
    REG_USB_EP_CTRL_91D0 = 0x02;

    if (is_usb3) {
        REG_LINK_STATUS_E716 = 0x01;
        REG_USB_ADDR_CFG_A = 0x03; REG_USB_ADDR_CFG_B = 0x03;
        REG_USB_ADDR_CFG_A = 0x07; REG_USB_ADDR_CFG_B = 0x07;
        tmp = REG_USB_ADDR_CFG_A; REG_USB_ADDR_CFG_A = tmp;
        tmp = REG_USB_ADDR_CFG_B; REG_USB_ADDR_CFG_B = tmp;
        REG_USB_ADDR_PARAM_0 = 0x00; REG_USB_ADDR_PARAM_1 = 0x0A;
        REG_USB_ADDR_PARAM_2 = 0x00; REG_USB_ADDR_PARAM_3 = 0x0A;
        tmp = REG_USB_ADDR_CTRL; REG_USB_ADDR_CTRL = tmp;
        REG_USB_EP_CTRL_9220 = 0x04;
        complete_usb3_status();
    } else {
        send_zlp_ack();
    }
    uart_puts("[A]\n");
}

/* Descriptors */
static __code const uint8_t dev_desc[] = {
    0x12, 0x01, 0x20, 0x03, 0x00, 0x00, 0x00, 0x09,
    0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
    0x09, 0x02, 0x2C, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
    0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x00, 0x00, 0x00, 0x00,
};
static __code const uint8_t bos_desc[] = {
    0x05, 0x0F, 0x16, 0x00, 0x02,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
    0x0A, 0x10, 0x03, 0x00, 0x0E, 0x00, 0x03, 0x00, 0x00, 0x00,
};
static __code const uint8_t str0_desc[] = { 0x04, 0x03, 0x09, 0x04 };
static __code const uint8_t str1_desc[] = { 0x0A, 0x03, 't',0, 'i',0, 'n',0, 'y',0 };
static __code const uint8_t str2_desc[] = { 0x08, 0x03, 'u',0, 's',0, 'b',0 };
static __code const uint8_t str3_desc[] = { 0x08, 0x03, '0',0, '0',0, '1',0 };
static __code const uint8_t str_empty[] = { 0x02, 0x03 };

static void handle_get_descriptor(uint8_t desc_type, uint8_t desc_idx, uint8_t wlen) {
    __code const uint8_t *src;
    uint8_t desc_len;

    if (desc_type == USB_DESC_TYPE_DEVICE) {
        desc_copy(dev_desc, 18);
        if (!is_usb3) { DESC_BUF[2] = 0x10; DESC_BUF[3] = 0x02; DESC_BUF[7] = 0x40; }
        desc_len = 18;
    } else if (desc_type == USB_DESC_TYPE_CONFIG) {
        src = cfg_desc; desc_len = sizeof(cfg_desc); desc_copy(src, desc_len);
    } else if (desc_type == USB_DESC_TYPE_BOS) {
        src = bos_desc; desc_len = sizeof(bos_desc); desc_copy(src, desc_len);
    } else if (desc_type == USB_DESC_TYPE_STRING) {
        if (desc_idx == 0)      { src = str0_desc; desc_len = sizeof(str0_desc); }
        else if (desc_idx == 1) { src = str1_desc; desc_len = sizeof(str1_desc); }
        else if (desc_idx == 2) { src = str2_desc; desc_len = sizeof(str2_desc); }
        else if (desc_idx == 3) { src = str3_desc; desc_len = sizeof(str3_desc); }
        else                    { src = str_empty; desc_len = sizeof(str_empty); }
        desc_copy(src, desc_len);
    } else {
        return;
    }

    send_descriptor_data(wlen < desc_len ? wlen : desc_len);
}

/*=== SET_CONFIG ===*/
static void handle_set_config(void) {
    uint8_t t;
    REG_USB_EP_BUF_CTRL = 0x55; REG_USB_EP_BUF_SEL = 0x53;
    REG_USB_EP_BUF_DATA = 0x42; REG_USB_EP_BUF_PTR_LO = 0x53;
    REG_USB_MSC_LENGTH = 0x0D;
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    REG_USB_EP_CFG2 = 0x01; REG_USB_EP_CFG2 = 0x08;
    REG_USB_EP_STATUS_90E3 = 0x02;
    t = REG_USB_EP_CTRL_905F; REG_USB_EP_CTRL_905F = t;
    t = REG_USB_EP_CTRL_905D; REG_USB_EP_CTRL_905D = t;
    REG_USB_EP_STATUS_90E3 = 0x01; REG_USB_CTRL_90A0 = 0x01;
    REG_USB_INT_MASK_9090 |= 0x80;
    t = REG_USB_STATUS; REG_USB_STATUS = t;
    t = REG_USB_CTRL_924C; REG_USB_CTRL_924C = t;
    send_zlp_ack();
    need_bulk_init = 1;
    uart_puts("[C]\n");
}

/*=== Bulk Init -- arms MSC engine for CBW reception ===*/
static void do_bulk_init(void) {
    uint16_t j;
    uint8_t t;

    /* Clear EP/NVMe/FIFO registers */
    REG_USB_EP_READY = 0xFF; REG_USB_EP_CTRL_9097 = 0xFF;
    REG_USB_EP_MODE_9098 = 0xFF; REG_USB_EP_MODE_9099 = 0xFF;
    REG_USB_EP_MODE_909A = 0xFF; REG_USB_EP_MODE_909B = 0xFF;
    REG_USB_EP_MODE_909C = 0xFF; REG_USB_EP_MODE_909D = 0xFF;
    REG_USB_STATUS_909E = 0x03;
    REG_USB_DATA_H = 0x00; REG_USB_FIFO_STATUS = 0x00;
    REG_USB_FIFO_H = 0x00; REG_USB_FIFO_4 = 0x00;
    REG_USB_FIFO_5 = 0x00; REG_USB_FIFO_6 = 0x00;
    REG_USB_FIFO_7 = 0x00;
    REG_USB_XCVR_MODE = 0x02; REG_USB_DATA_L = 0x00;

    /* MSC toggle */
    REG_USB_MSC_CFG |= 0x02;
    REG_USB_MSC_CFG |= 0x04;
    REG_USB_MSC_CFG &= ~0x02;
    REG_USB_MSC_CFG &= ~0x04;
    t = REG_USB_STATUS; REG_USB_STATUS = t;
    t = REG_USB_CTRL_924C; REG_USB_CTRL_924C = t;

    /* EP reconfig + activate */
    t = REG_USB_EP_CTRL_905F; REG_USB_EP_CTRL_905F = t;
    t = REG_USB_EP_CTRL_905D; REG_USB_EP_CTRL_905D = t;
    REG_USB_EP_STATUS_90E3 = 0x01; REG_USB_CTRL_90A0 = 0x01;
    REG_USB_STATUS = 0x01; REG_USB_CTRL_924C = 0x05;

    /* Clear endpoint buffer D800-DE5F */
    for (j = 0; j < 0x0660; j++) XDATA_REG8(0xD800 + j) = 0x00;
    REG_USB_EP_BUF_DE30 = 0x03; REG_USB_EP_BUF_DE36 = 0x00;

    /* 9200 toggle + MSC reset */
    REG_USB_CTRL_9200 |= 0x40;
    REG_USB_MSC_CFG |= 0x01;
    REG_USB_MSC_CFG &= ~0x01;
    REG_USB_CTRL_9200 &= ~0x40;

    /* Final EP config */
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    REG_USB_EP_CFG2 = 0x01; REG_USB_EP_CFG2 = 0x08;
    REG_USB_EP_CTRL_905F |= 0x08;
    REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_CTRL_90A0 = 0x01;

    /* Arm MSC for first CBW */
    REG_USB_STATUS = 0x00;
    t = REG_USB_CTRL_924C; REG_USB_CTRL_924C = t;
    REG_USB_MSC_CFG |= 0x02;
    REG_USB_MSC_CFG |= 0x04;
    REG_NVME_DOORBELL |= NVME_DOORBELL_BIT0;
    REG_USB_MSC_CFG |= 0x01;
    REG_NVME_DOORBELL |= NVME_DOORBELL_BIT1;
    REG_NVME_DOORBELL |= NVME_DOORBELL_BIT2;
    REG_NVME_DOORBELL |= NVME_DOORBELL_BIT3;
    REG_NVME_DOORBELL |= NVME_DOORBELL_BIT4;
    REG_USB_MSC_CFG &= ~0x02;
    REG_USB_MSC_CFG &= ~0x04;
    REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT0;
    REG_USB_MSC_CFG &= ~0x01;
    REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT1;
    REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT2;
    REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT3;
    REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT4;

    arm_msc();

    /* 9200 toggle (second pass) */
    REG_USB_CTRL_9200 |= 0x40;
    REG_USB_MSC_CFG |= 0x01;
    REG_USB_MSC_CFG &= ~0x01;
    REG_USB_CTRL_9200 &= ~0x40;

    /* EP reconfig (second pass) */
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    REG_USB_EP_CFG2 = 0x01; REG_USB_EP_CFG2 = 0x08;
    t = REG_USB_EP_CTRL_905F; REG_USB_EP_CTRL_905F = t;
    REG_USB_EP_STATUS_90E3 = 0x02;

    uart_puts("[rdy]\n");
}

/*=== Bulk Transfer Engine ===*/

static void send_csw(uint8_t status) {
    EP_BUF(0x0C) = status;
    EP_BUF(0x08) = 0x00; EP_BUF(0x09) = 0x00;
    EP_BUF(0x0A) = 0x00; EP_BUF(0x0B) = 0x00;
    REG_USB_BULK_DMA_TRIGGER = 0x01;
    while (!(REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE)) { }
    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS &= ~0x01;
}

static void sw_dma_bulk_in(uint16_t addr, uint8_t len) {
    uint8_t ah = (addr >> 8) & 0xFF;
    uint8_t al = addr & 0xFF;

    /* Clear stale EP_COMPLETE */
    if (REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE) {
        REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    }

    REG_USB_MSC_LENGTH = len;
    REG_DMA_CONFIG = DMA_CONFIG_SW_MODE;
    REG_USB_EP_BUF_HI = ah; REG_USB_EP_BUF_LO = al;
    EP_BUF(0x02) = ah; EP_BUF(0x03) = al;
    EP_BUF(0x04) = 0x00; EP_BUF(0x05) = 0x00;
    EP_BUF(0x06) = 0x00; EP_BUF(0x07) = 0x00;
    EP_BUF(0x0F) = 0x00; EP_BUF(0x00) = 0x03;

    REG_XFER_CTRL_C509 |= 0x01;
    REG_USB_EP_CFG_905A = USB_EP_CFG_BULK_IN;
    REG_USB_SW_DMA_TRIGGER = 0x01;
    REG_XFER_CTRL_C509 &= ~0x01;

    G_XFER_STATE_0AF4 = 0x40;
    REG_USB_BULK_DMA_TRIGGER = 0x01;

    /* Wait for EP_COMPLETE (9101 bit 5) */
    while (!(REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE)) { }
    REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;

    REG_DMA_CONFIG = DMA_CONFIG_DISABLE;
    REG_USB_MSC_LENGTH = 0x0D;
}

/*=== CBW Handler ===*/
static void handle_cbw(void) {
    uint8_t opcode;

    if (!(REG_USB_MODE & 0x01)) return;
    REG_USB_MODE = 0x01;

    /* CE88/CE89 DMA handshake */
    REG_BULK_DMA_HANDSHAKE = 0x00;
    while (!(REG_USB_DMA_STATE & USB_DMA_STATE_READY)) { }

    cbw_tag[0] = REG_CBW_TAG_0; cbw_tag[1] = REG_CBW_TAG_1;
    cbw_tag[2] = REG_CBW_TAG_2; cbw_tag[3] = REG_CBW_TAG_3;
    EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
    EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
    EP_BUF(0x0C) = 0x00;

    opcode = REG_USB_CBWCB_0;
    uart_puts("[CBW:");
    uart_puthex(opcode);
    uart_puts("]\n");

    if (opcode == 0xE5) {
        uint8_t val = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        XDATA_REG8(addr) = val;
        send_csw(0x00);
    } else if (opcode == 0xE4) {
        uint8_t sz = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        if (sz > 4) sz = 4;
        EP_BUF(0x08) = (sz >= 1) ? XDATA_REG8(addr) : 0x00;
        EP_BUF(0x09) = (sz >= 2) ? XDATA_REG8(addr + 1) : 0x00;
        EP_BUF(0x0A) = (sz >= 3) ? XDATA_REG8(addr + 2) : 0x00;
        EP_BUF(0x0B) = (sz >= 4) ? XDATA_REG8(addr + 3) : 0x00;
        REG_USB_BULK_DMA_TRIGGER = 0x01;
        REG_USB_MSC_CTRL = 0x01;
        REG_USB_MSC_STATUS &= ~0x01;
    } else if (opcode == 0xE6) {
        uint8_t len = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        uint8_t i;
        if (len == 0) len = 64;
        for (i = 0; i < len; i++) EP_BUF(i) = XDATA_REG8(addr + i);
        sw_dma_bulk_in(addr, len);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0xE7) {
        bulk_out_addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        bulk_out_len = REG_USB_CBWCB_1;
        if (bulk_out_len == 0) bulk_out_len = 64;
        bulk_out_state = 1;
        return;
    } else if (opcode == 0xE8) {
        send_csw(0x00);
    } else {
        send_csw(0x01);
    }
}

/*=== Link / 91D1 Handlers ===*/

static void handle_link_event(void) {
    uint8_t r9300 = REG_BUF_CFG_9300;
    if (r9300 & BUF_CFG_9300_SS_FAIL) {
        is_usb3 = 0;
        bulk_out_state = 0; need_cbw_process = 0; need_bulk_init = 0;
        uart_puts("[T]\n");
    } else if (r9300 & BUF_CFG_9300_SS_OK) {
        is_usb3 = 1;
        uart_puts("[3]\n");
    }
    REG_BUF_CFG_9300 = BUF_CFG_9300_SS_OK | BUF_CFG_9300_SS_FAIL | BUF_CFG_9300_SS_EVENT;
}

/*
 * 91D1 link training dispatch — keeps SS link alive.
 * Stock firmware ISR at 0x0f4a. Without this, link dies after 30-75s idle.
 */
static void handle_91d1_events(void) {
    uint8_t r91d1;

    if (!(REG_USB_PERIPH_STATUS & USB_PERIPH_BUS_RESET)) return;
    r91d1 = REG_USB_PHY_CTRL_91D1;

    /* bit 3: power management (U1/U2). Stock: 0x9b95 */
    if (r91d1 & USB_91D1_POWER_MGMT) {
        REG_USB_PHY_CTRL_91D1 = USB_91D1_POWER_MGMT;
        G_USB_TRANSFER_FLAG = 0;
        REG_TIMER_CTRL_CC3B &= ~TIMER_CTRL_LINK_POWER;
        G_TLP_BASE_LO = 0x01;
    }

    r91d1 = REG_USB_PHY_CTRL_91D1;

    /* bit 0: link training. Stock: 0xc465 -> bda4 state reset */
    if (r91d1 & USB_91D1_LINK_TRAIN) {
        REG_USB_PHY_CTRL_91D1 = USB_91D1_LINK_TRAIN;
        /* bda4: C6A8 |= 1, 92C8 &= ~3, CD31 reset */
        REG_PHY_CFG_C6A8 |= PHY_CFG_C6A8_ENABLE;
        REG_POWER_CTRL_92C8 &= ~POWER_CTRL_92C8_BIT0;
        REG_POWER_CTRL_92C8 &= ~POWER_CTRL_92C8_BIT1;
        REG_CPU_TIMER_CTRL_CD31 = CPU_TIMER_CD31_CLEAR;
        REG_CPU_TIMER_CTRL_CD31 = CPU_TIMER_CD31_START;
        if (!(REG_USB_PHY_CTRL_91C0 & USB_PHY_91C0_LINK_UP)) {
            REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | LINK_RECOVERY_MODE;
            REG_TIMER_CTRL_CC3B &= ~TIMER_CTRL_LINK_POWER;
        }
        return;
    }

    /* bit 1: simple flag. Stock: 0xe6aa */
    if (r91d1 & USB_91D1_FLAG) {
        REG_USB_PHY_CTRL_91D1 = USB_91D1_FLAG;
        G_EP_DISPATCH_VAL3 = 0;
        G_USB_TRANSFER_FLAG = 1;
        return;
    }

    /* bit 2: link reset ack. Stock: 0xe682 */
    if (r91d1 & USB_91D1_LINK_RESET) {
        REG_PHY_CFG_C6A8 |= PHY_CFG_C6A8_ENABLE;
        G_USB_TRANSFER_FLAG = 0;
        G_SYS_FLAGS_07E8 = 0;
        REG_USB_PHY_CTRL_91D1 = USB_91D1_LINK_RESET;
    }
}

static void handle_usb_reset(void) {
    G_STATE_FLAG_0AF1 = 0x01;
    REG_USB_EP0_CONFIG |= 0x01;
    REG_USB_EP0_CONFIG |= 0x80;
    REG_USB_EP_READY = 0x01;
    bulk_out_state = 0; need_cbw_process = 0; need_bulk_init = 0;
    uart_puts("[R]\n");
}

/*=== Interrupt Handlers ===*/

static void poll_bulk_events(void) {
    uint8_t st = REG_USB_PERIPH_STATUS;
    if (st & USB_PERIPH_EP_COMPLETE) {
        REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    }
    if (st & USB_PERIPH_CBW_RECEIVED) need_cbw_process = 1;
}

void int0_isr(void) __interrupt(0) {
    uint8_t periph_status, phase;
    periph_status = REG_USB_PERIPH_STATUS;

    if (periph_status & USB_PERIPH_LINK_EVENT) handle_link_event();
    handle_91d1_events();

    if ((periph_status & USB_PERIPH_BUS_RESET) && !(periph_status & USB_PERIPH_CONTROL))
        handle_usb_reset();

    if (periph_status & USB_PERIPH_BULK_REQ) {
        uint8_t r9301 = REG_BUF_CFG_9301;
        if (r9301 & BUF_CFG_9301_BIT6)
            REG_BUF_CFG_9301 = BUF_CFG_9301_BIT6;
        else if (r9301 & BUF_CFG_9301_BIT7) {
            REG_BUF_CFG_9301 = BUF_CFG_9301_BIT7;
            REG_POWER_DOMAIN |= POWER_DOMAIN_BIT1;
        } else {
            uint8_t r9302 = REG_BUF_CFG_9302;
            if (r9302 & BUF_CFG_9302_BIT7) REG_BUF_CFG_9302 = BUF_CFG_9302_BIT7;
        }
    }

    if (!(periph_status & USB_PERIPH_CONTROL)) return;
    phase = REG_USB_CTRL_PHASE;

    if (phase == USB_CTRL_PHASE_DATA_OUT || phase == 0x00) {
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_DATA_OUT;
        return;
    }

    if ((phase & USB_CTRL_PHASE_STAT_OUT) && !(phase & USB_CTRL_PHASE_SETUP)) {
        complete_usb20_status();
    } else if ((phase & USB_CTRL_PHASE_STAT_IN) && !(phase & USB_CTRL_PHASE_SETUP)) {
        REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
    } else if (phase & USB_CTRL_PHASE_SETUP) {
        uint8_t bmReq, bReq, wValL, wValH, wLenL;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
        bmReq = REG_USB_SETUP_BMREQ; bReq = REG_USB_SETUP_BREQ;
        wValL = REG_USB_SETUP_WVAL_L; wValH = REG_USB_SETUP_WVAL_H;
        wLenL = REG_USB_SETUP_WLEN_L;

        if (bmReq == 0x00 && bReq == USB_REQ_SET_ADDRESS) {
            handle_set_address();
        } else if (bmReq == 0x80 && bReq == USB_REQ_GET_DESCRIPTOR) {
            handle_get_descriptor(wValH, wValL, wLenL);
        } else if (bmReq == 0x00 && bReq == USB_REQ_SET_CONFIGURATION) {
            handle_set_config();
        } else if (bmReq == 0x01 && bReq == USB_REQ_SET_INTERFACE) {
            need_bulk_init = 1;
            send_zlp_ack();
            uart_puts("[I]\n");
        } else if (bmReq == 0x02 && bReq == 0x01) {
            /* CLEAR_FEATURE(HALT) -- re-arm MSC */
            send_zlp_ack();
            arm_msc();
        } else if (bmReq == 0xC0 && bReq == 0xE4) {
            /* Vendor read XDATA via control */
            uint16_t addr = ((uint16_t)wValH << 8) | wValL;
            uint8_t vi;
            for (vi = 0; vi < wLenL; vi++) DESC_BUF[vi] = XDATA_REG8(addr + vi);
            send_descriptor_data(wLenL);
        } else if (bmReq == 0x40 && bReq == 0xE5) {
            /* Vendor write XDATA via control */
            uint16_t addr = ((uint16_t)wValH << 8) | wValL;
            XDATA_REG8(addr) = REG_USB_SETUP_WIDX_L;
            send_zlp_ack();
        } else if (bmReq == 0x40 && bReq == 0xE6) {
            /* Vendor write XDATA block via control */
            uint16_t addr = ((uint16_t)wValH << 8) | wValL;
            uint8_t vi;
            if (is_usb3) {
                for (vi = 0; vi < wLenL; vi++) XDATA_REG8(addr + vi) = DESC_BUF[vi];
            }
            send_zlp_ack();
        } else {
            send_zlp_ack();
        }
    }
}

void timer0_isr(void) __interrupt(1) { }

void int1_isr(void) __interrupt(2) {
    uint8_t tmp;
    tmp = REG_POWER_EVENT_92E1;
    if (tmp) {
        REG_POWER_EVENT_92E1 = tmp;
        REG_POWER_STATUS &= ~(POWER_STATUS_USB_PATH | 0x80);
    }
}

void timer1_isr(void) __interrupt(3) { }
void serial_isr(void) __interrupt(4) { }
void timer2_isr(void) __interrupt(5) { }

/*=== Hardware Init (from stock firmware trace) ===*/
static void hw_init(void) {
    uint8_t i;

    REG_CPU_EXEC_STATUS = 0x01; REG_CPU_MODE = 0x01;
    REG_LINK_WIDTH_E710 = 0x04; REG_CPU_EXEC_STATUS_2 = 0x04;
    REG_TIMER_CTRL_CC3B = 0x0C; REG_LINK_CTRL_E717 = 0x01;
    REG_CPU_CTRL_CC3E = 0x00; REG_TIMER_CTRL_CC3B = 0x0C;
    REG_TIMER_CTRL_CC3B = 0x0C; REG_LINK_STATUS_E716 = 0x03;
    REG_CPU_CTRL_CC3E = 0x00; REG_TIMER_CTRL_CC39 = 0x06;
    REG_TIMER_ENABLE_B = 0x14; REG_TIMER_ENABLE_A = 0x44;
    REG_CPU_CTRL_CC37 = 0x2C; REG_SYS_CTRL_E780 = 0x00;
    REG_LINK_STATUS_E716 = 0x00; REG_LINK_STATUS_E716 = 0x03;
    REG_CPU_CTRL_CC37 = 0x28;
    REG_PHY_LINK_CTRL = 0x00;
    REG_PHY_TIMER_CTRL_E764 = 0x14; REG_PHY_TIMER_CTRL_E764 = 0x14;
    REG_PHY_TIMER_CTRL_E764 = 0x14; REG_PHY_TIMER_CTRL_E764 = 0x14;
    REG_SYS_CTRL_E76C = 0x04; REG_SYS_CTRL_E774 = 0x04;
    REG_SYS_CTRL_E77C = 0x04;
    REG_INT_AUX_STATUS = 0x02; REG_CPU_EXEC_STATUS_3 = 0x00;
    REG_INT_ENABLE = 0x10;
    REG_INT_STATUS_C800 = 0x04; REG_INT_STATUS_C800 = 0x05;
    REG_TIMER_CTRL_CC3B = 0x0D; REG_TIMER_CTRL_CC3B = 0x0F;
    REG_POWER_CTRL_92C6 = 0x05; REG_POWER_CTRL_92C7 = 0x00;
    REG_USB_CTRL_9201 = 0x0E; REG_USB_CTRL_9201 = 0x0C;
    REG_CLOCK_ENABLE = 0x82; REG_USB_CTRL_920C = 0x61;
    REG_USB_CTRL_920C = 0x60; REG_POWER_ENABLE = 0x87;
    REG_CLOCK_ENABLE = 0x83; REG_PHY_POWER = 0x2F;
    REG_USB_PHY_CONFIG_9241 = 0x10; REG_USB_PHY_CONFIG_9241 = 0xD0;
    REG_PHY_PLL_CTRL = 0x5B; REG_PHY_PLL_CTRL = 0x6B;
    REG_PHY_PLL_CFG = 0x1F; REG_PHY_PLL_CTRL = 0xAB;
    REG_PHY_PLL_CFG = 0x17; REG_CPU_CLK_CFG = 0x88;
    REG_BUF_DESC_STAT0_HI = 0x00; REG_BUF_DESC_STAT0_LO = 0x00;
    REG_BUF_DESC_STAT1_HI = 0x00; REG_BUF_DESC_STAT1_LO = 0x00;
    REG_BUF_DESC_STAT2_HI = 0x00; REG_BUF_DESC_STAT2_LO = 0x00;
    REG_BUF_DESC_BASE0_HI = 0x01; REG_BUF_DESC_BASE0_LO = 0x60;
    REG_BUF_DESC_SIZE0_HI = 0x00; REG_BUF_DESC_SIZE0_LO = 0xE3;
    REG_BUF_DESC_BASE1_HI = 0x01; REG_BUF_DESC_BASE1_LO = 0x60;
    REG_BUF_DESC_BASE2_HI = 0x01; REG_BUF_DESC_BASE2_LO = 0x60;
    REG_BUF_DESC_CFG0_HI = 0x00; REG_BUF_DESC_CFG0_LO = 0x03;
    REG_BUF_DESC_CFG1_HI = 0x00; REG_BUF_DESC_CFG1_LO = 0xE0;
    REG_BUF_DESC_CFG2_HI = 0x00; REG_BUF_DESC_CFG2_LO = 0xE3;
    REG_CPU_EXEC_STATUS_3 = 0x00; REG_USB_EP_CTRL_905F = 0x44;
    REG_CPU_KEEPALIVE = 0x04;
    REG_CPU_KEEPALIVE_CC2C = 0xC7; REG_CPU_KEEPALIVE_CC2D = 0xC7;
    REG_INT_ENABLE = 0x50; REG_CPU_EXEC_STATUS = 0x00;
    REG_INT_DMA_CTRL = 0x04;
    REG_POWER_CTRL_92C8 = 0x24; REG_POWER_CTRL_92C8 = 0x24;
    REG_DMA_STATUS2 = 0x00; REG_DMA_STATUS2 = 0x00;
    REG_DMA_STATUS2 = 0x00; REG_DMA_CTRL = 0x00;
    REG_DMA_STATUS = 0x00; REG_DMA_STATUS = 0x00;
    REG_DMA_STATUS = 0x00; REG_DMA_QUEUE_IDX = 0x00;
    { static __code const uint8_t dma_cfg[][4] = {
        {0x02, 0xA0, 0x0F, 0xFF}, {0x02, 0xB0, 0x01, 0xFF},
        {0x00, 0xA0, 0x0F, 0xFF}, {0x00, 0xB0, 0x01, 0xFF},
        {0x02, 0xB8, 0x03, 0xFF}, {0x02, 0xBC, 0x00, 0x7F},
        {0x00, 0xB8, 0x03, 0xFF}, {0x00, 0xBC, 0x00, 0x7F},
    };
    for (i = 0; i < 8; i++) {
        if (i < 4) REG_DMA_STATUS2 = dma_cfg[i][0];
        else       REG_DMA_STATUS = dma_cfg[i][0];
        REG_DMA_CHAN_STATUS2 = 0x00;
        REG_DMA_CHAN_CTRL2 = 0x14; REG_DMA_CHAN_CTRL2 = 0x14;
        REG_DMA_CHAN_CTRL2 = 0x14; REG_DMA_CHAN_CTRL2 = 0x94;
        REG_DMA_CHAN_AUX = dma_cfg[i][1];
        REG_DMA_CHAN_AUX1 = 0x00;
        REG_DMA_XFER_CNT_HI = dma_cfg[i][2];
        REG_DMA_XFER_CNT_LO = dma_cfg[i][3];
        REG_DMA_TRIGGER = 0x01;
        REG_DMA_CHAN_CTRL2 = 0x14;
    }}
    REG_USB_MSC_CFG = 0x07; REG_USB_MSC_CFG = 0x07;
    REG_USB_MSC_CFG = 0x07; REG_USB_MSC_CFG = 0x05;
    REG_USB_MSC_CFG = 0x01; REG_USB_MSC_CFG = 0x00;
    REG_USB_MSC_LENGTH = 0x0D;
    REG_POWER_ENABLE = 0x87; REG_USB_PHY_CTRL_91D1 = USB_91D1_ALL;
    REG_BUF_CFG_9300 = 0x0C; REG_BUF_CFG_9301 = 0xC0;
    REG_BUF_CFG_9302 = 0xBF; REG_USB_CTRL_PHASE = 0x1F;
    REG_USB_EP_CFG1 = 0x0F; REG_USB_PHY_CTRL_91C1 = 0xF0;
    REG_BUF_CFG_9303 = 0x33; REG_BUF_CFG_9304 = 0x3F;
    REG_BUF_CFG_9305 = 0x40; REG_USB_CONFIG = 0xE0;
    REG_USB_EP0_LEN_H = 0xF0; REG_USB_MODE = 0x01;
    REG_USB_EP_MGMT = 0x00;
    REG_USB_EP_READY = 0xFF; REG_USB_EP_CTRL_9097 = 0xFF;
    REG_USB_EP_MODE_9098 = 0xFF; REG_USB_EP_MODE_9099 = 0xFF;
    REG_USB_EP_MODE_909A = 0xFF; REG_USB_EP_MODE_909B = 0xFF;
    REG_USB_EP_MODE_909C = 0xFF; REG_USB_EP_MODE_909D = 0xFF;
    REG_USB_STATUS_909E = 0x03;
    REG_USB_DATA_H = 0xFF; REG_USB_FIFO_STATUS = 0xFF;
    REG_USB_FIFO_H = 0xFF; REG_USB_FIFO_4 = 0xFF;
    REG_USB_FIFO_5 = 0xFF; REG_USB_FIFO_6 = 0xFF;
    REG_USB_FIFO_7 = 0xFF; REG_USB_XCVR_MODE = 0x03;
    REG_USB_DATA_L = 0xFE;
    REG_USB_PHY_CTRL_91C3 = 0x00;
    REG_USB_PHY_CTRL_91C0 = 0x13; REG_USB_PHY_CTRL_91C0 = 0x12;
    REG_INT_DMA_CTRL = 0x04; REG_INT_DMA_CTRL = 0x84;
    REG_LINK_MODE_CTRL = 0xFF;
    REG_XFER2_DMA_STATUS = 0x04; REG_XFER2_DMA_STATUS = 0x02;
    REG_XFER2_DMA_CTRL = 0x00; REG_INT_ENABLE = 0x50;
    REG_XFER2_DMA_CTRL = 0x04;
    REG_XFER2_DMA_ADDR_LO = 0x00; REG_XFER2_DMA_ADDR_HI = 0xC8;
    REG_INT_CTRL = 0x08; REG_INT_CTRL = 0x0A; REG_INT_CTRL = 0x0A;
    REG_CPU_EXT_CTRL = 0x40;
    REG_CPU_EXT_STATUS = 0x04; REG_CPU_EXT_STATUS = 0x02;
    REG_XFER_DMA_CTRL = 0x10; REG_XFER_DMA_ADDR_LO = 0x00;
    REG_XFER_DMA_ADDR_HI = 0x0A; REG_XFER_DMA_CMD = 0x01;
    REG_XFER_DMA_CMD = 0x02;
    REG_XFER_DMA_CTRL = 0x10; REG_XFER_DMA_ADDR_LO = 0x00;
    REG_XFER_DMA_ADDR_HI = 0x3C; REG_XFER_DMA_CMD = 0x01;
    REG_XFER_DMA_CMD = 0x02;
    REG_INT_CTRL = 0x2A; REG_INT_ENABLE = 0x50;
    REG_CPU_CTRL_CC80 = 0x00; REG_CPU_CTRL_CC80 = 0x03;
    REG_XFER_DMA_CFG = 0x04; REG_XFER_DMA_CFG = 0x02;
    REG_INT_ENABLE = 0x50;
    REG_CPU_DMA_READY = 0x00; REG_CPU_DMA_READY = 0x04;
    REG_CPU_CTRL_CC82 = 0x18; REG_CPU_CTRL_CC83 = 0x9C;
    REG_CPU_DMA_INT = 0x04; REG_CPU_DMA_INT = 0x02;
    REG_INT_ENABLE = 0x50;
    REG_CPU_DMA_CTRL_CC90 = 0x00; REG_CPU_DMA_CTRL_CC90 = 0x05;
    REG_CPU_DMA_DATA_LO = 0x00; REG_CPU_DMA_DATA_HI = 0xC8;
    REG_CPU_DMA_INT = 0x01;
}

void main(void) {
    IE = 0;
    is_usb3 = 0; need_bulk_init = 0; bulk_out_state = 0;
    REG_UART_LCR &= 0xF7;
    uart_puts("\n[BOOT]\n");

    hw_init();

    uint8_t link = REG_USB_LINK_STATUS;
    is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
    uart_puts("[link="); uart_puthex(link); uart_puts("]\n");

    uart_puts("[GO]\n");
    TCON = 0x04;  /* IT0=0 (level-triggered INT0) */
    IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

    while (1) {
        REG_CPU_KEEPALIVE = 0x0C;
        poll_bulk_events();

        if (need_bulk_init) { need_bulk_init = 0; do_bulk_init(); }
        if (need_cbw_process) { need_cbw_process = 0; handle_cbw(); }

        if (bulk_out_state == 1) {
            REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
            REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;
            bulk_out_state = 2;
        } else if (bulk_out_state == 2) {
            uint8_t st = REG_USB_PERIPH_STATUS;
            if (st & USB_PERIPH_BULK_DATA) {
                REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
                REG_INT_AUX_STATUS = (REG_INT_AUX_STATUS & 0xF9) | 0x02;
                REG_BULK_DMA_HANDSHAKE = 0x00;
                while (!(REG_USB_DMA_STATE & USB_DMA_STATE_READY)) { }
                { uint8_t ci;
                  for (ci = 0; ci < bulk_out_len; ci++)
                      XDATA_REG8(bulk_out_addr + ci) = XDATA_REG8(0x7000 + ci); }
                EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
                EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
                EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
                EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
                send_csw(0x00);
                bulk_out_state = 0;
            }
        }
    }
}
