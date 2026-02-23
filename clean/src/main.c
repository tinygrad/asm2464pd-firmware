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
static volatile uint8_t need_dma_setup;  /* phase 1: DMA state setup (like ISR first entry) */
static volatile uint8_t bulk_ready;      /* set after do_bulk_init completes */
static uint8_t cbw_tag[4];
static volatile uint8_t bulk_out_state;
static uint16_t bulk_out_addr;
static uint8_t bulk_out_len;
static volatile uint8_t pcie_initialized;  /* 0=none, 1=phase1_done, 2=phase2_done, 3=link_up */
static volatile uint8_t isr_link_state;  /* 0=none, 1=ss_ok, 2=ss_fail, 3=set_addr */
static volatile uint8_t tur_count;       /* SCSI TUR counter */
static volatile uint8_t pcie_phase2_pending; /* set when tunnel_enable sets state=0x10 */
static volatile uint8_t need_pcie_init;  /* set by SET_CONFIGURATION */
static volatile uint16_t poll_counter;

static void poll_bulk_events(void);
static void sw_dma_bulk_in(uint16_t addr, uint8_t len);
static void pcie_tunnel_enable(void);
static void pcie_phase2(void);

/*=== USB Control Transfer Helpers ===*/

static void complete_usb3_status(void) {
    while (!(REG_USB_CTRL_PHASE & USB_CTRL_PHASE_STAT_IN)) { }
    REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
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

static void handle_set_address(uint8_t addr) {
    uint8_t tmp;
    /* Write device address to 0x9090, preserving bit 7 (addressed flag) */
    tmp = REG_USB_INT_MASK_9090;
    REG_USB_INT_MASK_9090 = (tmp & 0x80) | (addr & 0x7F);
    REG_USB_EP_CTRL_91D0 = 0x02;

    if (is_usb3) {
        /* USB 3.0 SET_ADDRESS (from min_enum / trace/flash_usb3)
         * NOTE: stock firmware does NOT write E716 during SET_ADDRESS.
         * The previous REG_LINK_STATUS_E716=0x01 here was corrupting PCIe
         * (E716 bits 0-1 = 0x03 means PCIe mode, 0x01 = USB2 mode). */

        /* Wait for hardware to indicate status phase ready */
        while (!(REG_USB_CTRL_PHASE & USB_CTRL_PHASE_STAT_IN)) { }

        /* Address programming registers */
        REG_USB_ADDR_CFG_A = 0x03; REG_USB_ADDR_CFG_B = 0x03;
        REG_USB_ADDR_CFG_A = 0x07; REG_USB_ADDR_CFG_B = 0x07;
        tmp = REG_USB_ADDR_CFG_A; REG_USB_ADDR_CFG_A = tmp;
        tmp = REG_USB_ADDR_CFG_B; REG_USB_ADDR_CFG_B = tmp;
        REG_USB_ADDR_PARAM_0 = 0x00; REG_USB_ADDR_PARAM_1 = 0x0A;
        REG_USB_ADDR_PARAM_2 = 0x00; REG_USB_ADDR_PARAM_3 = 0x0A;
        tmp = REG_USB_ADDR_CTRL; REG_USB_ADDR_CTRL = tmp;
        REG_USB_EP_CTRL_9220 = 0x04;

        /* Complete status phase (no polling needed) */
        REG_USB_DMA_TRIGGER = USB_DMA_STATUS_COMPLETE;
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_STAT_IN;
    } else {
        send_zlp_ack();
    }
    isr_link_state = 3;  /* signal SET_ADDRESS happened */
}

/* Descriptors */
static __code const uint8_t dev_desc[] = {
    0x12, 0x01, 0x20, 0x03, 0x00, 0x00, 0x00, 0x09,
    0xD1, 0xAD, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x01,
};
static __code const uint8_t cfg_desc[] = {
    /* Configuration descriptor */
    0x09, 0x02, 0x2C, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    /* Interface descriptor: class=0x08 (Mass Storage), subclass=0x06 (SCSI),
     * protocol=0x50 (Bulk-Only Transport). Stock firmware uses these exact values.
     * The ASM2464PD MSC engine may require class 0x08 to enable C42C bulk IN. */
    0x09, 0x04, 0x00, 0x00, 0x02, 0x08, 0x06, 0x50, 0x00,
    /* EP1 IN (0x81) bulk, maxpacket 1024 */
    0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x00, 0x00, 0x00, 0x00,
    /* EP2 OUT (0x02) bulk, maxpacket 1024 */
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
    /* Only trigger full bulk reinit on first SET_CONFIGURATION.
     * Repeated SET_CONFIGURATION (from libusb_set_configuration) must not
     * reset bulk_ready, or CBWs arriving between reinit trigger and
     * do_bulk_init() completion will be lost (CSW never sent → USB timeout). */
    if (!bulk_ready)
        need_bulk_init = 1;
    /* Trigger PCIe tunnel init after USB enumeration completes.
     * Stock firmware waits for ~21 TURs, but we start immediately
     * after SET_CONFIGURATION so PCIe link is up before bulk commands. */
    if (pcie_initialized == 0)
        need_pcie_init = 1;
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

    /* Stock firmware MSC engine init (0xB1C5-0xB24F)
     * These registers configure the MSC/NVMe-USB bridge engine.
     * Must be written BEFORE the initial C42C=0x01 trigger. */

    /* 92C0 |= 0x80: Power enable bit 7 */
    XDATA_REG8(0x92C0) = (XDATA_REG8(0x92C0) & 0x7F) | 0x80;

    /* 91D1 = 0x0F: Clear all link events */
    XDATA_REG8(0x91D1) = 0x0F;

    /* 9300-9302: Buffer config */
    XDATA_REG8(0x9300) = 0x0C;
    XDATA_REG8(0x9301) = 0xC0;
    XDATA_REG8(0x9302) = 0xBF;

    /* 9091 = 0x1F: Control phase */
    XDATA_REG8(0x9091) = 0x1F;

    /* 9093 = 0x0F: EP status */
    XDATA_REG8(0x9093) = 0x0F;

    /* 91C1 = 0xF0: PHY control */
    XDATA_REG8(0x91C1) = 0xF0;

    /* 9303-9305: Buffer descriptors */
    XDATA_REG8(0x9303) = 0x33;
    XDATA_REG8(0x9304) = 0x3F;
    XDATA_REG8(0x9305) = 0x40;

    /* 9002 = 0xE0: USB config */
    XDATA_REG8(0x9002) = 0xE0;

    /* 9005 = 0xF0: EP0 length high */
    XDATA_REG8(0x9005) = 0xF0;

    /* 90E2 = 0x01: MSC gate register (CRITICAL for C42C) */
    XDATA_REG8(0x90E2) = 0x01;

    /* 905E &= ~0x01: EP control clear bit 0 */
    XDATA_REG8(0x905E) = XDATA_REG8(0x905E) & 0xFE;

    /* Stock 0xB219-0xB223: Initial MSC arm (C42C=0x01, C42D &= ~0x01)
     * NOTE: Stock does NOT write "USBS" signature for initial arm.
     * The "USBS" write only happens in scsi_csw_build (0x494D). */
    REG_USB_MSC_CTRL = 0x01;              /* C42C = 0x01 */
    REG_USB_MSC_STATUS &= ~0x01;          /* C42D &= ~0x01 */

    /* Stock 0xB224-0xB240: Post-C42C initialization
     * nvme_prp_queue_init(R7=0) at 0xCF3D:
     *   C430-C433 = 0xFF, C440-C443 = 0xFF
     *   9096-9097 = 0xFF, 9098 = 0x03
     *   C438-C43B = 0xFF, C448-C44B = 0xFF
     *   9011-9014 = 0xFF, 9018 = 0x03, 9010 = 0xFE */
    XDATA_REG8(0xC430) = 0xFF; XDATA_REG8(0xC431) = 0xFF;
    XDATA_REG8(0xC432) = 0xFF; XDATA_REG8(0xC433) = 0xFF;
    XDATA_REG8(0xC440) = 0xFF; XDATA_REG8(0xC441) = 0xFF;
    XDATA_REG8(0xC442) = 0xFF; XDATA_REG8(0xC443) = 0xFF;
    XDATA_REG8(0x9096) = 0xFF; XDATA_REG8(0x9097) = 0xFF;
    XDATA_REG8(0x9098) = 0x03;
    XDATA_REG8(0xC438) = 0xFF; XDATA_REG8(0xC439) = 0xFF;
    XDATA_REG8(0xC43A) = 0xFF; XDATA_REG8(0xC43B) = 0xFF;
    XDATA_REG8(0xC448) = 0xFF; XDATA_REG8(0xC449) = 0xFF;
    XDATA_REG8(0xC44A) = 0xFF; XDATA_REG8(0xC44B) = 0xFF;
    XDATA_REG8(0x9011) = 0xFF; XDATA_REG8(0x9012) = 0xFF;
    XDATA_REG8(0x9013) = 0xFF; XDATA_REG8(0x9014) = 0xFF;
    XDATA_REG8(0x9018) = 0x03;
    XDATA_REG8(0x9010) = 0xFE;

    /* nvme_link_init at 0xDF5E:
     *   C428 &= ~0x08
     *   C473 = (C473 & ~0x40) | 0x40  (set bit 6)
     *   C473 = (C473 & ~0x20) | 0x20  (set bit 5) */
    XDATA_REG8(0xC428) = XDATA_REG8(0xC428) & 0xF7;
    XDATA_REG8(0xC473) = (XDATA_REG8(0xC473) & 0xBF) | 0x40;
    XDATA_REG8(0xC473) = (XDATA_REG8(0xC473) & 0xDF) | 0x20;

    /* 91C3 &= ~0x20: PHY control */
    XDATA_REG8(0x91C3) = XDATA_REG8(0x91C3) & 0xDF;

    /* 91C0 toggle bit 0: PHY reset */
    XDATA_REG8(0x91C0) = (XDATA_REG8(0x91C0) & 0xFE) | 0x01;
    XDATA_REG8(0x91C0) = XDATA_REG8(0x91C0) & 0xFE;

    /* 0x54BB: clear_pcie_enum_done (0x0AF7 = 0x00) */
    XDATA_REG8(0x0AF7) = 0x00;

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

    bulk_ready = 1;
    uart_puts("[rdy]\n");
}

/*=== Bulk Transfer Engine ===*/

/*
 * send_csw - Send CSW (Command Status Wrapper) to host via bulk IN
 *
 * Uses the stock firmware's scsi_csw_build approach (0x494D-0x49BD):
 *   1. Doorbell dance ramp-up: 900B |= bits 1,2; C42A |= bits 0,1,2,3,4
 *   2. Doorbell dance teardown: clear all those bits
 *   3. Write "USBS" + tag to D800 (EP buffer)
 *   4. 901A = 0x0D (CSW length), C42C = 0x01 (MSC trigger), C42D &= ~0x01
 *   5. Re-arm MSC engine for next CBW
 */
static void send_csw(uint8_t status) {
    /* Stock scsi_csw_build (0x494D): Doorbell dance ramp-up
     * 0x324B (set_bits_1_2): 900B |= 0x02, 900B |= 0x04
     * 0x3172 (set_bit_0): C42A |= 0x01
     * 0x3172 (set_bit_0): 900B |= 0x01
     * 0x324B (set_bits_1_2): C42A |= 0x02, C42A |= 0x04
     * C42A = (C42A & ~0x08) | 0x08
     * 0x32B1 (set_bit_4): C42A |= 0x10 */
    XDATA_REG8(0x900B) = (XDATA_REG8(0x900B) & 0xFD) | 0x02;
    XDATA_REG8(0x900B) = (XDATA_REG8(0x900B) & 0xFB) | 0x04;
    XDATA_REG8(0xC42A) = (XDATA_REG8(0xC42A) & 0xFE) | 0x01;
    XDATA_REG8(0x900B) = (XDATA_REG8(0x900B) & 0xFE) | 0x01;
    XDATA_REG8(0xC42A) = (XDATA_REG8(0xC42A) & 0xFD) | 0x02;
    XDATA_REG8(0xC42A) = (XDATA_REG8(0xC42A) & 0xFB) | 0x04;
    XDATA_REG8(0xC42A) = (XDATA_REG8(0xC42A) & 0xF7) | 0x08;
    XDATA_REG8(0xC42A) = (XDATA_REG8(0xC42A) & 0xEF) | 0x10;

    /* Doorbell dance teardown (stock 0x4971-0x4999) */
    XDATA_REG8(0x900B) = XDATA_REG8(0x900B) & 0xFD;  /* clear bit 1 */
    XDATA_REG8(0x900B) = XDATA_REG8(0x900B) & 0xFB;  /* clear bit 2 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & 0xFE;  /* clear bit 0 */
    XDATA_REG8(0x900B) = XDATA_REG8(0x900B) & 0xFE;  /* clear bit 0 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & 0xFD;  /* clear bit 1 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & 0xFB;  /* clear bit 2 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & 0xF7;  /* clear bit 3 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & 0xEF;  /* clear bit 4 */

    /* Write "USBS" signature + tag + residue + status to D800 EP buffer
     * Stock: 0x0D9D (store_dword) writes D800-D803 = "USBS" */
    EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
    EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
    EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
    EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
    /* Residue = 0 (no data transfer) */
    EP_BUF(0x08) = 0x00; EP_BUF(0x09) = 0x00;
    EP_BUF(0x0A) = 0x00; EP_BUF(0x0B) = 0x00;
    EP_BUF(0x0C) = status;

    /* Stock 0x49AC-0x49BC: Set length, set MSC gate, trigger MSC, clear status */
    REG_USB_MSC_LENGTH = 0x0D;            /* 901A = 0x0D (CSW is 13 bytes) */

    /* 90E2 is volatile — must set immediately before C42C trigger.
     * Stock ISR sets 90E2=0x01 during CBW processing (0x1023, 0x1057). */
    XDATA_REG8(0x90E2) = 0x01;

    REG_USB_MSC_CTRL = 0x01;              /* C42C = 0x01 (trigger MSC send) */
    REG_USB_MSC_STATUS &= ~0x01;          /* C42D &= ~0x01 (clear status) */

    uart_putc('!');

    /* Stock firmware post-CSW cleanup (0xC16C):
     * 1. Clear XDATA 0x000B, 0x000A, 0x0052
     * 2. Loop clearing structure arrays
     * 3. C42A |= 0x20 (NVME_DOORBELL_LINK_GATE)
     * 4. Call NVMe link init (0xDF5E)
     * 5. C42A &= ~0x20
     * We do the C42A LINK_GATE toggle but skip the full NVMe reinit since
     * we don't have NVMe queues configured. */
    XDATA_REG8(0x000B) = 0x00;
    XDATA_REG8(0x000A) = 0x00;
    XDATA_REG8(0x0052) = 0x00;

    /* Stock: C42A |= 0x20, NVMe link init, C42A &= ~0x20 */
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) | 0x20;
    XDATA_REG8(0xC42A) = XDATA_REG8(0xC42A) & ~0x20;

    /* Re-arm MSC engine for next CBW.
     * Stock firmware re-arms via the post-cleanup NVMe reinit,
     * but we need to explicitly re-arm since we skip that. */
    arm_msc();
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

/*
 * cbw_dma_setup - Phase 1 of CBW processing (stock: ISR first entry at 0x0FF8)
 *
 * Stock ISR flow: first INT0 fires with 0x9000 bit 0 set.
 * ISR reads C47B (==0), calls 0x17DB(R7=2), sets 90E2=0x01.
 * 0x17DB checks 0x0A7D: if !=0 reads 911D/911E into 0x0056/0x0057,
 * sets 0x0052 |= 0x06. If ==0, does CE88 handshake.
 *
 * We combine both paths since our firmware handles 0x0A7D differently.
 * This function must complete and return to main loop BEFORE
 * handle_cbw is called — giving hardware time to set up bulk IN path.
 */
static void cbw_dma_setup(void) {
    /* Stock 0x17DB(R7=2) → 0x19C8: DMA state setup */
    XDATA_REG8(0x0A7D) = 0x02;

    /* Stock 0x19C8 with 0x0A7D != 0 (second entry path):
     * Read CBW transfer length from 911D/911E → 0x0056/0x0057
     * Set 0x0052 |= 0x06 */
    {
        uint8_t xfer_hi = XDATA_REG8(0x911D);
        uint8_t xfer_lo = XDATA_REG8(0x911E);
        XDATA_REG8(0x0056) = xfer_hi;
        XDATA_REG8(0x0057) = xfer_lo;
        XDATA_REG8(0x0052) = XDATA_REG8(0x0052) | 0x06;
    }

    /* Stock ISR sets 90E2 = 0x01 as gate register */
    XDATA_REG8(0x90E2) = 0x01;

    uart_puts("[DMA1]\n");
}

/*=== CBW Handler ===*/
static void handle_cbw(void) {
    uint8_t opcode;

    /* DMA handshake (CE88/CE89) — stock firmware does this at 0x3484-0x348C
     * (usb_ep_loop_3419) during the SECOND ISR entry (0x9000 bit 0 clear).
     * CE88 write transitions hardware state machine for bulk transfers. */

    /* Stock ISR second entry also sets 90E2 = 0x01 */
    XDATA_REG8(0x90E2) = 0x01;

    REG_BULK_DMA_HANDSHAKE = 0x00;
    {
        uint16_t dma_to;
        for (dma_to = 30000; dma_to; dma_to--) {
            if (REG_USB_DMA_STATE & USB_DMA_STATE_READY) break;
        }
        if (!dma_to) uart_puts("[dma TO]\n");
    }

    cbw_tag[0] = REG_CBW_TAG_0; cbw_tag[1] = REG_CBW_TAG_1;
    cbw_tag[2] = REG_CBW_TAG_2; cbw_tag[3] = REG_CBW_TAG_3;
    EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
    EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
    EP_BUF(0x0C) = 0x00;

    opcode = REG_USB_CBWCB_0;
    uart_putc('C'); uart_puthex(opcode); uart_putc(' ');

    if (opcode == 0xE5) {
        uint8_t val = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        XDATA_REG8(addr) = val;
        send_csw(0x00);
    } else if (opcode == 0xE4) {
        uint8_t sz = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        uint8_t i;
        if (sz == 0) sz = 1;
        /* Copy read data to EP buffer for data-in phase */
        for (i = 0; i < sz; i++) EP_BUF(i) = XDATA_REG8(addr + i);
        sw_dma_bulk_in(addr, sz);
        /* Restore CSW header */
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
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
    } else if (opcode == 0x00) {
        /* SCSI TEST_UNIT_READY — stock firmware triggers PCIe tunnel
         * enable after ~21 TURs. We defer PCIe init until we've proven
         * bulk IN works by successfully handling the first few TURs.
         * Return status 1 (not ready) until link is up. */
        tur_count++;
        if (tur_count == 3 && pcie_initialized == 0) {
            need_pcie_init = 1;
        }
        send_csw((pcie_initialized >= 2) ? 0x00 : 0x01);
    } else if (opcode == 0x12) {
        /* SCSI INQUIRY — return minimal response matching stock firmware.
         * This is needed so usb-storage stays bound (keeps interface configured). */
        uint8_t alloc_len = REG_USB_CBWCB_4;
        uint8_t resp_len = (alloc_len < 36) ? alloc_len : 36;
        uint8_t vi;
        /* Clear response buffer */
        for (vi = 0; vi < resp_len; vi++) EP_BUF(vi) = 0x00;
        /* Standard INQUIRY response (SPC-4) */
        EP_BUF(0x00) = 0x00;  /* Peripheral: Direct access, connected */
        EP_BUF(0x01) = 0x80;  /* RMB=1 (removable) */
        EP_BUF(0x02) = 0x06;  /* Version: SPC-4 */
        EP_BUF(0x03) = 0x02;  /* Response format: 2 */
        EP_BUF(0x04) = 0x1F;  /* Additional length: 31 */
        /* Vendor: "ASMT    " (8 bytes) */
        EP_BUF(0x08) = 'A'; EP_BUF(0x09) = 'S'; EP_BUF(0x0A) = 'M'; EP_BUF(0x0B) = 'T';
        EP_BUF(0x0C) = ' '; EP_BUF(0x0D) = ' '; EP_BUF(0x0E) = ' '; EP_BUF(0x0F) = ' ';
        /* Product: "2462 NVME       " (16 bytes) */
        EP_BUF(0x10) = '2'; EP_BUF(0x11) = '4'; EP_BUF(0x12) = '6'; EP_BUF(0x13) = '2';
        EP_BUF(0x14) = ' '; EP_BUF(0x15) = 'N'; EP_BUF(0x16) = 'V'; EP_BUF(0x17) = 'M';
        EP_BUF(0x18) = 'E'; EP_BUF(0x19) = ' '; EP_BUF(0x1A) = ' '; EP_BUF(0x1B) = ' ';
        /* Revision: "0   " (4 bytes) */
        EP_BUF(0x20) = '0'; EP_BUF(0x21) = ' '; EP_BUF(0x22) = ' '; EP_BUF(0x23) = ' ';
        sw_dma_bulk_in(0xD800, resp_len);
        /* Restore CSW header after data-in */
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x25) {
        /* SCSI READ_CAPACITY(10) — return capacity info.
         * usb-storage sends this during enumeration. Return small capacity. */
        uint8_t vi;
        for (vi = 0; vi < 8; vi++) EP_BUF(vi) = 0x00;
        /* Last LBA = 0x00000000 (1 block) */
        EP_BUF(0x00) = 0x00; EP_BUF(0x01) = 0x00;
        EP_BUF(0x02) = 0x00; EP_BUF(0x03) = 0x00;
        /* Block size = 512 bytes */
        EP_BUF(0x04) = 0x00; EP_BUF(0x05) = 0x00;
        EP_BUF(0x06) = 0x02; EP_BUF(0x07) = 0x00;
        sw_dma_bulk_in(0xD800, 8);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x1A) {
        /* SCSI MODE_SENSE(6) — return minimal response */
        EP_BUF(0x00) = 0x03; EP_BUF(0x01) = 0x00;
        EP_BUF(0x02) = 0x00; EP_BUF(0x03) = 0x00;
        sw_dma_bulk_in(0xD800, 4);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x03) {
        /* SCSI REQUEST_SENSE — return "not ready, medium not present" */
        uint8_t alloc_len = REG_USB_CBWCB_4;
        uint8_t resp_len = (alloc_len < 18) ? alloc_len : 18;
        uint8_t vi;
        for (vi = 0; vi < resp_len; vi++) EP_BUF(vi) = 0x00;
        EP_BUF(0x00) = 0x70;  /* Response code: current */
        EP_BUF(0x02) = 0x02;  /* Sense key: NOT READY */
        EP_BUF(0x07) = 0x0A;  /* Additional sense length */
        EP_BUF(0x0C) = 0x3A;  /* ASC: MEDIUM NOT PRESENT */
        EP_BUF(0x0D) = 0x00;  /* ASCQ */
        sw_dma_bulk_in(0xD800, resp_len);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x1E) {
        /* PREVENT_ALLOW_MEDIUM_REMOVAL — no-data, just ack */
        send_csw(0x00);
    } else if (opcode == 0x1B) {
        /* START_STOP_UNIT — no-data, just ack */
        send_csw(0x00);
    } else {
        send_csw(0x01);
    }
}

/*=== Link / 91D1 Handlers ===*/

static void handle_link_event(void) {
    uint8_t r9300 = REG_BUF_CFG_9300;
    if (r9300 & BUF_CFG_9300_SS_FAIL) {
        /* USB3 failed — enable USB2 fallback path (from min_enum) */
        REG_POWER_STATUS = REG_POWER_STATUS | POWER_STATUS_USB_PATH;
        REG_POWER_EVENT_92E1 = 0x10;
        REG_USB_STATUS = REG_USB_STATUS | 0x04;
        REG_USB_STATUS = REG_USB_STATUS & ~0x04;
        REG_PHY_LINK_CTRL = 0x00;
        REG_CPU_MODE = 0x00;
        REG_LINK_WIDTH_E710 = 0x1F;
        REG_USB_PHY_CTRL_91C0 = 0x10;
        is_usb3 = 0;
        bulk_out_state = 0; need_cbw_process = 0; need_dma_setup = 0; need_bulk_init = 0; bulk_ready = 0;
        isr_link_state = 2;
    } else if (r9300 & BUF_CFG_9300_SS_OK) {
        REG_BUF_CFG_9300 = BUF_CFG_9300_SS_OK;
        is_usb3 = 1;
        isr_link_state = 1;
    }
    REG_BUF_CFG_9300 = BUF_CFG_9300_SS_FAIL;
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

/*
 * USB Bus Reset Handler (from min_enum / trace/enumerate_usb_20)
 * This is the full reset sequence that reconfigures timers, clocks,
 * PHY, MSC engine, NVMe doorbells, EP buffers, and detects post-reset speed.
 * Without this, USB2 fallback fails because hardware isn't properly reconfigured.
 */
static void handle_usb_reset(void) {
    uint8_t r91d1;
    uint16_t timeout;

    r91d1 = REG_USB_PHY_CTRL_91D1;
    REG_USB_PHY_CTRL_91D1 = r91d1;

    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & ~0x02;
    REG_CLOCK_CTRL_92CF = 0x00;
    REG_CLOCK_CTRL_92CF = 0x04;
    REG_CLOCK_ENABLE = REG_CLOCK_ENABLE | 0x10;
    REG_TIMER0_CSR = 0x04; REG_TIMER0_CSR = 0x02;
    REG_TIMER0_DIV = 0x10;
    REG_TIMER0_THRESHOLD_HI = 0x00; REG_TIMER0_THRESHOLD_LO = 0x0A;
    REG_TIMER0_CSR = 0x01;
    REG_TIMER0_CSR = 0x02;
    REG_CLOCK_ENABLE = REG_CLOCK_ENABLE & ~0x10;
    REG_CLOCK_CTRL_92CF = 0x07;
    REG_CLOCK_CTRL_92CF = 0x03;
    REG_PHY_LINK_CTRL = 0x00;
    REG_POWER_EVENT_92E1 = 0x40;
    REG_POWER_STATUS = REG_POWER_STATUS & ~POWER_STATUS_USB_PATH;

    for (timeout = 10000; timeout; timeout--) {
        r91d1 = REG_USB_PHY_CTRL_91D1;
        if (r91d1 & USB_91D1_LINK_TRAIN) break;
    }
    if (r91d1 & USB_91D1_LINK_TRAIN)
        REG_USB_PHY_CTRL_91D1 = r91d1;

    REG_CPU_TIMER_CTRL_CD31 = 0x04; REG_CPU_TIMER_CTRL_CD31 = 0x02;
    REG_TIMER2_CSR = 0x04; REG_TIMER2_CSR = 0x02;
    REG_TIMER4_CSR = 0x04; REG_TIMER4_CSR = 0x02;
    REG_TIMER2_THRESHOLD_LO = 0x00; REG_TIMER2_THRESHOLD_HI = 0x8B;
    REG_TIMER4_THRESHOLD_LO = 0x00; REG_TIMER4_THRESHOLD_HI = 0xC7;

    /* MSC + NVMe doorbell dance */
    REG_USB_MSC_CFG = REG_USB_MSC_CFG | 0x02;
    REG_USB_MSC_CFG = REG_USB_MSC_CFG | 0x04;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x01;
    REG_USB_MSC_CFG = REG_USB_MSC_CFG | 0x01;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x02;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x04;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x08;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x10;
    REG_USB_MSC_CFG = REG_USB_MSC_CFG & ~0x02;
    REG_USB_MSC_CFG = REG_USB_MSC_CFG & ~0x04;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x01;
    REG_USB_MSC_CFG = REG_USB_MSC_CFG & ~0x01;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x02;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x04;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x08;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x10;

    REG_USB_EP_BUF_CTRL = 0x55;
    REG_USB_EP_BUF_SEL  = 0x53;
    REG_USB_EP_BUF_DATA = 0x42;
    REG_USB_EP_BUF_PTR_LO = 0x53;
    REG_USB_MSC_LENGTH = 0x0D;
    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS = REG_USB_MSC_STATUS;

    /* NVMe init under doorbell bit 5 */
    REG_NVME_DOORBELL = REG_NVME_DOORBELL | 0x20;
    REG_NVME_INIT_CTRL2 = 0xFF;
    REG_NVME_INIT_CTRL2_1 = 0xFF;
    REG_NVME_INIT_CTRL2_2 = 0xFF;
    REG_NVME_INIT_CTRL2_3 = 0xFF;
    REG_NVME_INIT_CTRL = 0xFF;
    REG_NVME_CMD_CDW11 = 0xFF;
    REG_NVME_INT_MASK_A = 0xFF;
    REG_NVME_INT_MASK_B = 0xFF;
    REG_NVME_DOORBELL = REG_NVME_DOORBELL & ~0x20;

    REG_BUF_CFG_9300 = BUF_CFG_9300_SS_FAIL;
    REG_POWER_STATUS = REG_POWER_STATUS | POWER_STATUS_USB_PATH;
    REG_POWER_EVENT_92E1 = 0x10;
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B | 0x02;

    /* Detect USB speed after reset */
    {
        uint8_t link = REG_USB_LINK_STATUS;
        is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
    }

    bulk_out_state = 0; need_cbw_process = 0; need_dma_setup = 0; need_bulk_init = 0; bulk_ready = 0;
}

/*=== Interrupt Handlers ===*/

/*
 * poll_bulk_events - Check for pending bulk transfers
 *
 * Checks 0x9101 for CBW/bulk events.
 */
static void poll_bulk_events(void) {
    uint8_t st = REG_USB_PERIPH_STATUS;

    poll_counter++;
    /* Periodic debug: show 9101 state every ~10000 polls */
    if ((poll_counter & 0x3FFF) == 0) {
        uart_puts("[p ");
        uart_puthex(st);
        uart_puts("]\n");
    }

    /* Ack EP_COMPLETE if pending */
    if (st & USB_PERIPH_EP_COMPLETE) {
        REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    }

    /* Check for CBW received (bit 6 of 9101) OR bulk data (bit 2).
     * Use two-phase processing like stock firmware ISR:
     * Phase 1 (need_dma_setup): DMA state setup + handshake
     * Phase 2 (need_cbw_process): CBW parsing + CSW send */
    if (st & (USB_PERIPH_CBW_RECEIVED | USB_PERIPH_BULK_DATA)) {
        if (!need_dma_setup && !need_cbw_process) {
            need_dma_setup = 1;
        }
    }
}

void int0_isr(void) __interrupt(0) {
    uint8_t periph_status, phase;
    periph_status = REG_USB_PERIPH_STATUS;

    if (periph_status & USB_PERIPH_LINK_EVENT) {
        handle_link_event();
        return;
    }

    if ((periph_status & USB_PERIPH_BUS_RESET) && !(periph_status & USB_PERIPH_CONTROL)) {
        handle_usb_reset();
        isr_link_state = 4;  /* signal bus reset */
        return;
    }

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

    /* CBW handling is done in main loop poll path.
     * ISR just handles bulk request and control transfers. */

    if (!(periph_status & USB_PERIPH_CONTROL)) {
        return;
    }
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
        REG_USB_CONFIG = REG_USB_CONFIG;  /* readback-writeback (from min_enum) */
        REG_USB_CTRL_PHASE = USB_CTRL_PHASE_SETUP;
        bmReq = REG_USB_SETUP_BMREQ; bReq = REG_USB_SETUP_BREQ;
        wValL = REG_USB_SETUP_WVAL_L; wValH = REG_USB_SETUP_WVAL_H;
        wLenL = REG_USB_SETUP_WLEN_L;

        if (bmReq == 0x00 && bReq == USB_REQ_SET_ADDRESS) {
            handle_set_address(wValL);
        } else if (bmReq == 0x80 && bReq == USB_REQ_GET_DESCRIPTOR) {
            handle_get_descriptor(wValH, wValL, wLenL);
        } else if (bmReq == 0x00 && bReq == USB_REQ_SET_CONFIGURATION) {
            handle_set_config();
        } else if (bmReq == 0x01 && bReq == USB_REQ_SET_INTERFACE) {
            need_bulk_init = 1;
            send_zlp_ack();
        } else if (bmReq == 0x00 && (bReq == USB_REQ_SET_SEL || bReq == USB_REQ_SET_ISOCH_DELAY)) {
            /* USB 3.0 required no-data requests */
            send_zlp_ack();
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

/*=== Small delay helper ===*/
static void delay_short(void) {
    volatile uint16_t d;
    for (d = 0; d < 1000; d++) { }
}
static void delay_long(void) {
    volatile uint16_t d;
    for (d = 0; d < 60000U; d++) { }
}

/*=== PHY link training (from stock firmware 0xD702-0xD743) ===*/
/*
 * Configures PHY lane registers via SFR 0x93 bank select.
 * Stock firmware uses r3=0x02 for these accesses, and the bank_read/bank_write
 * helpers set SFR 0x93 = (r3-1) & 0x7F = 0x01 before the XDATA access.
 *
 * Read/write pattern (from stock firmware phy_link_training at 0xD702-0xD743):
 *   Lane 0: read 0x78AF, set bit 7, write 0x78AF
 *   Lane 1: read 0x79AF, set bit 7, write 0x79AF
 *   Lane 2: read 0x7AAF, set bit 7, write 0x7AAF
 *   Lane 3: read 0x7BAF, set bit 7, write 0x7BAF
 */
static void phy_link_training(void) {
    /*
     * Matches stock firmware 0xD702-0xD743: configures PHY lane registers via
     * bank-switched XDATA. Stock firmware passes lane_mask in r7 and checks
     * each lane's bit to decide whether to set bit 7 in the PHY register.
     * Since we always enable all 4 lanes (mask=0x0F), we always set bit 7.
     *
     * SFR 0x93 = 0x01 because stock firmware's bank helpers set 0x93 = (r3-1)
     * where r3=0x02 for these PHY accesses.
     */
    __asm
        mov     0x93, #0x01     ; Bank select (stock: r3=2 → SFR=(2-1)&0x7F=1)

        ; Lane 0: read 0x78AF, set bit 7, write 0x78AF
        mov     dptr, #0x78AF
        movx    a, @dptr
        orl     a, #0x80
        movx    @dptr, a

        ; Lane 1: read 0x79AF, set bit 7, write 0x79AF
        mov     dptr, #0x79AF
        movx    a, @dptr
        orl     a, #0x80
        movx    @dptr, a

        ; Lane 2: read 0x7AAF, set bit 7, write 0x7AAF
        mov     dptr, #0x7AAF
        movx    a, @dptr
        orl     a, #0x80
        movx    @dptr, a

        ; Lane 3: read 0x7BAF, set bit 7, write 0x7BAF
        mov     dptr, #0x7BAF
        movx    a, @dptr
        orl     a, #0x80
        movx    @dptr, a

        mov     0x93, #0x00     ; Clear bank select
    __endasm;
}

/*=== PCIe Init (from stock firmware disassembly) ===*/
/*
 * timer_wait - Wait for hardware timer to expire
 * Address: 0xE80A-0xE81A (17 bytes)
 *
 * Sets up Timer0 with given threshold and mode, then polls until done.
 * Used for ~200ms delays between PCIe lane training steps.
 */
static void timer_wait(uint8_t threshold_hi, uint8_t threshold_lo, uint8_t mode) {
    uint8_t div;
    /* Reset timer */
    REG_TIMER0_CSR = 0x04;  /* TIMER_CSR_CLEAR */
    REG_TIMER0_CSR = 0x02;  /* TIMER_CSR_EXPIRED (clear done) */
    /* Configure prescaler */
    div = REG_TIMER0_DIV;
    div = (div & 0xF8) | (mode & 0x07);
    REG_TIMER0_DIV = div;
    /* Set threshold */
    REG_TIMER0_THRESHOLD_HI = threshold_hi;
    REG_TIMER0_THRESHOLD_LO = threshold_lo;
    /* Start timer */
    REG_TIMER0_CSR = 0x01;  /* TIMER_CSR_ENABLE */
    /* Poll until done (bit 1 set) */
    while (!(REG_TIMER0_CSR & 0x02)) { }
    /* Clear done flag */
    REG_TIMER0_CSR = 0x02;
}

/*
 * pcie_tunnel_setup - Configure PCIe tunnel BEFORE power is applied.
 * This must be called during hw_init, before enabling 12V/3.3V power.
 * Stock firmware has these registers already set at pre-training time.
 */
static void pcie_tunnel_setup(void) {
    uint8_t tmp;

    /* Stock 0xCD6C: CA06 &= 0xEF (clear bit 4) — done first */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;

    /* Stock 0xCD73: lcall 0xC8DB — adapter config B410-B42B from globals
     * Stock uses globals 0x0A52-0x0A55: 0x1B, 0x21, 0x24, 0x63 */
    REG_TUNNEL_CFG_A_LO = 0x1B;  REG_TUNNEL_CFG_A_HI = 0x21;
    REG_TUNNEL_DATA_LO = 0x1B;   REG_TUNNEL_DATA_HI = 0x21;
    REG_TUNNEL_CREDITS = 0x24;   REG_TUNNEL_CFG_MODE = 0x63;
    REG_TUNNEL_STATUS_0 = 0x24;  REG_TUNNEL_STATUS_1 = 0x63;
    REG_TUNNEL_CAP_0 = 0x06;    REG_TUNNEL_CAP_1 = 0x04;    REG_TUNNEL_CAP_2 = 0x00;
    REG_TUNNEL_CAP2_0 = 0x06;   REG_TUNNEL_CAP2_1 = 0x04;   REG_TUNNEL_CAP2_2 = 0x00;
    REG_TUNNEL_LINK_CFG_LO = 0x1B; REG_TUNNEL_LINK_CFG_HI = 0x21;
    REG_TUNNEL_AUX_CFG_LO = 0x1B;  REG_TUNNEL_AUX_CFG_HI = 0x21;
    REG_TUNNEL_PATH_CREDITS = 0x24; REG_TUNNEL_PATH_MODE = 0x63;
    REG_TUNNEL_PATH2_CRED = 0x24;  REG_TUNNEL_PATH2_MODE = 0x63;

    /* Stock 0xCD76-0xCD83: Bank-switched writes: 0x4084=0x22, 0x5084=0x22 */
    __asm
        mov     0x93, #0x01
        mov     dptr, #0x4084
        mov     a, #0x22
        movx    @dptr, a
        mov     dptr, #0x5084
        movx    @dptr, a
        mov     0x93, #0x00
    __endasm;

    /* Stock 0xCD86-0xCD89: B401 |= 0x01 (via 99E4 helper) */
    REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;

    /* Stock 0xCD8C-0xCD97: B482 |= 0x01 (via 99E4), then B482 = (B482 & 0x0F) | 0xF0 */
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0xFE) | 0x01;
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0x0F) | 0xF0;

    /* Stock 0xCD98-0xCD9E: B401 &= 0xFE, then via 99E0: write to B401, then B480 |= 1 */
    tmp = REG_PCIE_TUNNEL_CTRL;
    REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;  /* B480 |= 1 */

    /* Stock 0xCDA1-0xCDA7: B430 &= 0xFE */
    REG_TUNNEL_LINK_STATE = REG_TUNNEL_LINK_STATE & 0xFE;

    /* Stock 0xCDA8-0xCDB0: B298 = (B298 & 0xEF) | 0x10 */
    REG_PCIE_TUNNEL_CFG = (REG_PCIE_TUNNEL_CFG & 0xEF) | 0x10;

    /* Stock 0xCDB1-0xCDC3: Bank-switched writes: 0x6043=0x70, 0x6025 |= 0x80 */
    __asm
        mov     0x93, #0x01
        mov     dptr, #0x6043
        mov     a, #0x70
        movx    @dptr, a
        mov     dptr, #0x6025
        movx    a, @dptr
        orl     a, #0x80
        movx    @dptr, a
        mov     0x93, #0x00
    __endasm;
}

/*
 * pcie_lane_config - Progressive PCIe lane enable with PHY training
 * Based on stock firmware D436 → E84D/C089/E85C (0xD436-0xD47E)
 *
 * For mask=0x0F: enables lanes progressively B434 = 0x01, 0x03, 0x07, 0x0F
 * For mask=0x0E: reconfigures from current B434 to 0x0E (used in phase2/A840)
 * Each step calls phy_link_training() and waits ~200ms.
 * If mask != 0x0F, pulses B401 (PERST) after training (stock D44E-D457).
 * Then configures B436 lane config register.
 */
static void pcie_lane_config_mask(uint8_t mask) {
    uint8_t lane_state, tmp;
    uint8_t b402_saved;

    /* Stock E84D: Save B402 bit 1 and clear it during lane training */
    b402_saved = REG_PCIE_CTRL_B402 & 0x02;
    REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;

    if (mask == 0x0F) {
        /* Stock C089 for mask=0x0F: progressive enable 0x01→0x03→0x07→0x0F */
        static __code const uint8_t lane_steps[] = {0x01, 0x03, 0x07, 0x0F};
        uint8_t i;
        for (i = 0; i < 4; i++) {
            lane_state = REG_PCIE_LINK_STATE;
            REG_PCIE_LINK_STATE = lane_steps[i] | (lane_state & 0xF0);
            phy_link_training();
            timer_wait(0x00, 0xC7, 0x02);
        }
    } else {
        /* Stock C089 for mask<0x0F: single step to target mask */
        lane_state = REG_PCIE_LINK_STATE;
        REG_PCIE_LINK_STATE = mask | (lane_state & 0xF0);
        phy_link_training();
        timer_wait(0x00, 0xC7, 0x02);
    }

    /* Stock D44C-D457: If mask != 0x0F, pulse B401 (PERST) */
    if (mask != 0x0F) {
        REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;
        tmp = REG_PCIE_TUNNEL_CTRL;
        REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;
    }

    /* Stock E85C: Restore B402 bit 1 if it was set */
    if (b402_saved) {
        REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 | 0x02;
    }

    /* Configure B436 lane config register (from stock: 0xD458-0xD47E) */
    tmp = REG_PCIE_LANE_CONFIG;
    tmp = (tmp & 0xF0) | (mask & 0x0E);
    REG_PCIE_LANE_CONFIG = tmp;
    tmp = REG_PCIE_LINK_PARAM_B404;
    tmp = (tmp & 0x0F) ^ 0x0F;
    tmp = (tmp << 4) & 0xF0;
    REG_PCIE_LANE_CONFIG = (REG_PCIE_LANE_CONFIG & 0x0F) | tmp;
}

static void pcie_lane_config(void) {
    pcie_lane_config_mask(0x0F);
}

/*
 * phy_set_bit6 - Set bit 6 on all 4 lane PHY registers (stock: 0xE25E-0xE27F)
 *
 * Called after phy_link_training. Sets bit 6 on 0x78AF/0x79AF/0x7AAF/0x7BAF bank 1.
 * Stock: lcall phy_link_training, then read-modify-write bit 6 on each lane.
 */
static void phy_set_bit6(void) {
    /* First call phy_link_training (sets bit 7 on all lanes) */
    phy_link_training();

    /* Then set bit 6 on each lane register */
    __asm
        mov     0x93, #0x01     ; Bank select

        mov     dptr, #0x78AF
        movx    a, @dptr
        anl     a, #0xBF
        orl     a, #0x40
        movx    @dptr, a

        mov     dptr, #0x79AF
        movx    a, @dptr
        anl     a, #0xBF
        orl     a, #0x40
        movx    @dptr, a

        mov     dptr, #0x7AAF
        movx    a, @dptr
        anl     a, #0xBF
        orl     a, #0x40
        movx    @dptr, a

        mov     dptr, #0x7BAF
        movx    a, @dptr
        anl     a, #0xBF
        orl     a, #0x40
        movx    @dptr, a

        mov     0x93, #0x00     ; Clear bank select
    __endasm;
}

/*
 * pcie_link_controller_init - PCIe link controller config (stock: 0xCF28-0xCF7E)
 *
 * Called before pcie_early_init (D996). Configures CC3x LTSSM controller
 * registers, E710, E717, E324, C6A8, CA06, CA81. Without this, the PCIe
 * LTSSM state machine doesn't properly transition past Detect states.
 *
 * Original disassembly (CF28):
 *   CC30: set bit 0
 *   E710: (E710 & 0xE0) | 0x04
 *   C6A8: set bit 0
 *   CC33: = 0x04
 *   E324: clear bit 2
 *   CC3B: clear bit 0, then clear bit 1, then clear bit 6
 *   E717: set bit 0
 *   CC3E: clear bit 1, then clear bit 0
 *   CC39: set bit 1
 *   CC3A: clear bit 1
 *   CC38: clear bit 1
 *   CA06: (CA06 & 0x1F) | 0x60
 *   CA81: set bit 0
 */
static void pcie_link_controller_init(void) {
    /* Stock firmware writes CC32=0x01 BEFORE CF28 (emulator trace write #0).
     * CC32 must be 0x01 during CC3x init. */
    XDATA_REG8(0xCC32) = 0x01;

    /* CF28: CC30 set bit 0 (via bceb: read, clear bit 0, set bit 0) */
    XDATA_REG8(0xCC30) = (XDATA_REG8(0xCC30) & 0xFE) | 0x01;

    /* CF2E-CF33: E710 = (E710 & 0xE0) | 0x04 (bd49 reads E710 & 0xE0, then | 0x04) */
    XDATA_REG8(0xE710) = (XDATA_REG8(0xE710) & 0xE0) | 0x04;

    /* CF34-CF37: C6A8 set bit 0 */
    REG_PHY_CFG_C6A8 = (REG_PHY_CFG_C6A8 & 0xFE) | 0x01;

    /* CF3A-CF3F: CC33 = 0x04 (write-only register, reads back 0x00) */
    XDATA_REG8(0xCC33) = 0x04;

    /* CF40-CF46: E324 clear bit 2 */
    XDATA_REG8(0xE324) = XDATA_REG8(0xE324) & 0xFB;

    /* CF47-CF4D: CC3B clear bit 0, then (via bce7) write to CC3B, then E717 set bit 0 */
    XDATA_REG8(0xCC3B) = XDATA_REG8(0xCC3B) & 0xFE;
    REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;

    /* CF50-CF53: CC3E clear bit 1 */
    XDATA_REG8(0xCC3E) = XDATA_REG8(0xCC3E) & 0xFD;

    /* CF54-CF5A: CC3B clear bit 1, then clear bit 6 */
    XDATA_REG8(0xCC3B) = XDATA_REG8(0xCC3B) & 0xFD;
    XDATA_REG8(0xCC3B) = XDATA_REG8(0xCC3B) & 0xBF;

    /* Restore CC3B bits 0,1 — stock firmware restores these via ISR handlers
     * and other init paths after CF28. Since we call CF28 during TUR handling
     * (not at boot like stock), the ISR may not fire in time to restore them.
     * Stock "before" dump shows CC3B=0x0F, so bits 0,1 must be set. */
    XDATA_REG8(0xCC3B) = XDATA_REG8(0xCC3B) | 0x03;

    /* CF5B-CF60: E716 = (E716 & 0xFC) | 0x03 */
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;

    /* CF61-CF67: CC3E clear bit 0 */
    XDATA_REG8(0xCC3E) = XDATA_REG8(0xCC3E) & 0xFE;

    /* CF68-CF6E: CC39 set bit 1, then CC3A clear bit 1, CC38 clear bit 1 */
    XDATA_REG8(0xCC39) = (XDATA_REG8(0xCC39) & 0xFD) | 0x02;
    XDATA_REG8(0xCC3A) = XDATA_REG8(0xCC3A) & 0xFD;
    XDATA_REG8(0xCC38) = XDATA_REG8(0xCC38) & 0xFD;

    /* CF72-CF77: CA06 = (CA06 & 0x1F) | 0x60 */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;

    /* CF78-CF7E: CA81 set bit 0 */
    XDATA_REG8(0xCA81) = (XDATA_REG8(0xCA81) & 0xFE) | 0x01;
}

/*
 * phy_soft_reset - PHY soft reset via RXPLL mode toggle
 * Bank 1 Address: 0xED02-0xED22 (33 bytes) [actual addr: 0x16C6D]
 * Called via dispatch at 0x0610 → bank1 0xED02
 *
 * Emulator trace (14 MMIO writes):
 *   1. CC37 |= 0x04   (enable RXPLL reset mode)
 *   2. CA70 = 0x00     (clear CPU control)
 *   3. E780 = 0x00     (clear system control)
 *   4. E716 = 0x00     (clear link status)
 *   5. E716 = 0x03     (restore PCIe mode)
 *   6. Timer wait: 200 ticks mode 2 with E712 polling
 *   7. CC37 &= ~0x04   (disable RXPLL reset mode)
 *
 * The stock firmware implements this via complex bank-switched PHY
 * register access helpers (0x984D, 0x9697, 0x980D, 0x98DE, 0x98C7,
 * 0xEC2C). The net effect is these 14 register writes.
 */
static void phy_soft_reset(void) {
    uint8_t tmp;
    uint16_t timeout;

    /* CC37 |= 0x04 — enable RXPLL reset mode */
    REG_CPU_CTRL_CC37 = REG_CPU_CTRL_CC37 | CPU_CTRL_CC37_RXPLL_MODE;

    /* CA70 = 0x00 — clear CPU control */
    REG_CPU_CTRL_CA70 = 0x00;

    /* E780 = 0x00 — clear system control */
    REG_SYS_CTRL_E780 = 0x00;

    /* E716 = 0x00, then E716 = 0x03 — reset and restore link status */
    XDATA_REG8(0xE716) = 0x00;
    XDATA_REG8(0xE716) = 0x03;

    /* Timer wait + E712 polling (bank1 function at 0xEC2C)
     * Start timer: threshold=0xC8 (200), mode=0x02
     * Poll: E712 bit 0 or bit 1 set, OR timer expired */
    REG_TIMER0_CSR = 0x04;  /* clear */
    REG_TIMER0_CSR = 0x02;  /* clear done */
    tmp = REG_TIMER0_DIV;
    tmp = (tmp & 0xF8) | 0x02;  /* mode 2 */
    REG_TIMER0_DIV = tmp;
    REG_TIMER0_THRESHOLD_HI = 0x00;
    REG_TIMER0_THRESHOLD_LO = 0xC8;  /* 200 ticks */
    REG_TIMER0_CSR = 0x01;  /* start */

    for (timeout = 30000; timeout; timeout--) {
        tmp = XDATA_REG8(0xE712);
        if (tmp & 0x01) break;  /* E712 bit 0 — PHY ready */
        if (tmp & 0x02) break;  /* E712 bit 1 — PHY ready alt */
        if (REG_TIMER0_CSR & 0x02) break;  /* Timer expired */
    }

    /* Clear timer */
    REG_TIMER0_CSR = 0x04;
    REG_TIMER0_CSR = 0x02;

    /* CC37 &= ~0x04 — disable RXPLL reset mode */
    REG_CPU_CTRL_CC37 = REG_CPU_CTRL_CC37 & ~CPU_CTRL_CC37_RXPLL_MODE;
}

/*
 * pcie_pre_init - Full pre-D996 init sequence (stock: 0xCE79-0xCECE)
 *
 * This is the complete initialization that runs before pcie_early_init (D996).
 * Stock firmware runs this at boot. Includes:
 *   1. CC3F check and optional D0D3 LTSSM reset
 *   2. CF28 link controller init (always runs)
 *   3. 0x0610 call — phy_soft_reset (dispatch to ED02)
 *   4. C233 config + timer waits
 *   5. E712 polling (wait for PHY ready)
 *   6. Timer reset (E8EF)
 *   7. E7E3 write (DD42 with r7=0)
 */
static void pcie_pre_init(void) {
    uint8_t tmp;
    uint16_t timeout;

    /* CE79-CE87: Check CC3F bits and call D0D3 if needed, then always CF28 */
    /* D0D3 is called if CC3F bit 1 is set — it's an LTSSM controller reset.
     * We skip D0D3 for now since stock "before" dump shows CC3F=0xF0 (bit 1=0).
     * CF28 is always called. */
    pcie_link_controller_init();

    /* CE8A: lcall 0x0610 — dispatch to bank1 0xED02 (phy_soft_reset)
     * Toggles CC37 bit 2 (RXPLL mode), clears CA70/E780, resets E716,
     * waits for E712 PHY ready with 200-tick timeout */
    phy_soft_reset();

    /* CE8D-CE93: C233 &= 0xFC (clear bits 0,1) */
    XDATA_REG8(0xC233) = XDATA_REG8(0xC233) & 0xFC;

    /* CE94: BD5E with dptr still from prev context.
     * BD5E: @dptr = (@dptr & 0xFB) | 0x04 — sets bit 2, clears bit 2 (net: set bit 2)
     * The dptr here is from the C233 write, so this operates on C233:
     * C233 = (C233 & 0xFB) | 0x04 → set bit 2 on C233 */
    XDATA_REG8(0xC233) = (XDATA_REG8(0xC233) & 0xFB) | 0x04;

    /* CE97-CE9D: timer_wait(0x00, 0x14, 0x02) — ~20 ticks mode 2 */
    timer_wait(0x00, 0x14, 0x02);

    /* CEA0-CEA6: C233 &= 0xFB (clear bit 2) */
    XDATA_REG8(0xC233) = XDATA_REG8(0xC233) & 0xFB;

    /* CEA7-CEAD: E50D timer START (not wait) with r5=0x0A, r4=0x00, r7=0x03
     * E50D: E8EF (clear timer), CC10 = (CC10 & 0xF8) | mode, CC12=hi, CC13=lo, CC11=0x01
     * Then the loop at CEB0-CEC3 polls E712 and CC11 */
    {
        uint8_t div;
        /* Clear timer first (E8EF) */
        REG_TIMER0_CSR = 0x04;
        REG_TIMER0_CSR = 0x02;
        /* Set mode and threshold (E50D) */
        div = REG_TIMER0_DIV;
        div = (div & 0xF8) | (0x03 & 0x07);
        REG_TIMER0_DIV = div;
        REG_TIMER0_THRESHOLD_HI = 0x00;
        REG_TIMER0_THRESHOLD_LO = 0x0A;
        REG_TIMER0_CSR = 0x01;  /* Start timer */
    }

    /* CEB0-CEC3: Poll loop — wait for E712 bit 0 set, OR E712 bit 1 set, OR CC11 bit 1 set */
    for (timeout = 30000; timeout; timeout--) {
        tmp = XDATA_REG8(0xE712);
        if (tmp & 0x01) break;  /* E712 bit 0 set */
        if (tmp & 0x02) break;  /* E712 bit 1 set */
        if (REG_TIMER0_CSR & 0x02) break;  /* Timer expired (CC11 bit 1) */
    }

    /* CEC6: E8EF — clear timer */
    REG_TIMER0_CSR = 0x04;
    REG_TIMER0_CSR = 0x02;

    /* CEC9-CECB: DD42 with r7=0 — E7E3 = 0x00
     * (DD42 checks global 0x0AF1 bit 5, but for r7=0 it always writes E7E3=0) */
    XDATA_REG8(0xE7E3) = 0x00;

    uart_puts("[PI]\n");
}

/*
 * pcie_early_init - First-time PCIe PHY/controller init (stock: 0xD996-0xD9D2)
 *
 * This must run before pcie_tunnel_enable. In stock firmware, it runs during
 * early boot init. Configures B402, E764, B432, B404, runs initial lane config
 * and PHY training, then configures bank-switched PHY registers.
 */
static void pcie_early_init(void) {
    /* Stock 0xD996: B402 &= 0xFD (clear bit 1) */
    REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;

    /* Stock 0xD99C: C659 &= 0xFE (12V off)
     * SKIPPED: Turning 12V off then back on in phase2 kills the link.
     * Keep 12V on throughout since we enabled it at boot. */
    /* REG_PCIE_LANE_CTRL_C659 = REG_PCIE_LANE_CTRL_C659 & 0xFE; */

    /* Stock 0xD99F (E57D): Reset E764 — clear bits 1,0,3, set bit 2 */
    {
        uint8_t e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xFD;   /* clear bit 1 */
        REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xFE;   /* clear bit 0 */
        REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xF7;   /* clear bit 3 */
        REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = (e764 & 0xFB) | 0x04;  /* clear bit 2, set bit 2 */
        REG_PHY_TIMER_CTRL_E764 = e764;
    }

    /* Stock 0xD9A2 (D630): B432/B404 config + E76C/E774/E77C bit 4 clear
     * Since r7=0x01 (bit 1 clear), all three lane regs get bit 4 cleared */
    REG_POWER_CTRL_B432 = (REG_POWER_CTRL_B432 & 0xF8) | 0x07;
    REG_PCIE_LINK_PARAM_B404 = (REG_PCIE_LINK_PARAM_B404 & 0xF0) | 0x01;
    /* D630 when r7==1: clear bit 4 on E76C, E774, E77C (CC69 helper) */
    XDATA_REG8(0xE76C) = XDATA_REG8(0xE76C) & 0xEF;
    XDATA_REG8(0xE774) = XDATA_REG8(0xE774) & 0xEF;
    XDATA_REG8(0xE77C) = XDATA_REG8(0xE77C) & 0xEF;

    /* Stock 0xD9A9 (D436): Full lane config (progressive B434 enable + PHY training) */
    pcie_lane_config();

    /* Stock 0xD9AC (E25E): PHY link training + set bit 6 on all lanes */
    phy_set_bit6();

    /* Stock 0xD9AF-0xD9BA: Bank read 0x7041, clear bit 6, bank write */
    __asm
        mov     0x93, #0x01
        mov     dptr, #0x7041
        movx    a, @dptr
        anl     a, #0xBF
        movx    @dptr, a
        mov     0x93, #0x00
    __endasm;

    /* Stock 0xD9BD-0xD9D2: Bank read/write 0x1507 — set bits 2 and 1 */
    __asm
        mov     0x93, #0x01
        ; First: clear bit 2, set bit 2
        mov     dptr, #0x1507
        movx    a, @dptr
        anl     a, #0xFB
        orl     a, #0x04
        movx    @dptr, a
        ; Second: clear bit 1, set bit 1
        movx    a, @dptr
        anl     a, #0xFD
        orl     a, #0x02
        movx    @dptr, a
        mov     0x93, #0x00
    __endasm;

    uart_puts("[EI]\n");
}

/*
 * pcie_pll_init - PHY PLL and LTSSM config (stock: 0x8E28-0x8E5E)
 *
 * Called AFTER pcie_early_init. Configures PHY PLL registers (E741/E742)
 * and CPU clock config (CC43). Stock firmware reads OTP/flash config from
 * 0x70xx addresses via 0xdace helper, but the net register effect is:
 *   E741 = 0x03 → 0x2B → 0xAB (read-modify-write)
 *   E742 = 0x03
 *   CC43 = 0x80
 * Also clears CC35 bit 0 (LTSSM timer config).
 *
 * Without this, the PHY PLL doesn't lock and SerDes can't detect the
 * PCIe partner, causing LTSSM to stay stuck at Detect.Active (0x01).
 */
static void pcie_pll_init(void) {
    /* CC35 &= 0xFE — clear bit 0 (stock: 0x4C04) */
    REG_CPU_EXEC_STATUS_3 = REG_CPU_EXEC_STATUS_3 & 0xFE;

    /* E741/E742 PHY PLL programming (stock: 0x8E3A-0x8E55)
     * Stock firmware reads E741=0x00/E742=0x00 here (hw_init USB values not retained
     * in stock, but our hw_init leaves E742=0x17). Force correct PCIe PLL values.
     * E742=0x03 is CRITICAL — bits 2,4 from USB mode (0x17) prevent PCIe PLL lock. */
    REG_PHY_PLL_CTRL = 0x03;                                   /* E741: force 0x03 (stock starts from 0x00→0x03) */
    REG_PHY_PLL_CTRL = 0x2B;                                   /* E741: force 0x2B (stock: 0x03→0x2B) */
    REG_PHY_PLL_CFG  = 0x03;                                   /* E742: force 0x03 for PCIe PLL */
    REG_PHY_PLL_CTRL = REG_PHY_PLL_CTRL | 0x80;                /* E741: set bit 7 → 0xAB */
    REG_PHY_PLL_CFG  = 0x03;                                   /* E742: re-strobe 0x03 */

    /* CC43 = 0x80 — CPU clock config (stock: 0x8E5E) */
    REG_CPU_CLK_CFG = 0x80;

    /* Stock power/GPIO init at 0x5284-0x52A6 (after pcie_early_init):
     * Read-modify-write on C65B, C656, C62D */
    REG_PHY_EXT_5B = (REG_PHY_EXT_5B & 0xF7) | 0x08;    /* C65B: clear bit 3, set bit 3 */
    REG_HDDPC_CTRL = REG_HDDPC_CTRL & 0xDF;              /* C656: clear bit 5 */
    REG_PHY_EXT_5B = (REG_PHY_EXT_5B & 0xDF) | 0x20;    /* C65B: set bit 5 */
    XDATA_REG8(0xC62D) = (XDATA_REG8(0xC62D) & 0xE0) | 0x07;  /* C62D: set bits 0,1,2 */

    /* Stock 0xE598-0xE5B0: C004/C007/CA2E controller bus config */
    XDATA_REG8(0xC004) = (XDATA_REG8(0xC004) & 0xFD) | 0x02;  /* C004: set bit 1 */
    XDATA_REG8(0xC007) = XDATA_REG8(0xC007) & 0xF7;            /* C007: clear bit 3 */
    XDATA_REG8(0xCA2E) = (XDATA_REG8(0xCA2E) & 0xFE) | 0x01;  /* CA2E: set bit 0 */

    uart_puts("[PLL]\n");
}

/*
 * pcie_post_early_cleanup - Clear CC32 gate after all init (stock: 0x4FDB)
 *
 * Stock firmware sets CC32=0x01 before pcie_link_controller_init to enable
 * CC3x register writes, then clears it to 0x00 after all init is complete.
 * Leaving CC32=0x01 may prevent LTSSM from operating correctly.
 */
static void pcie_post_early_cleanup(void) {
    XDATA_REG8(0xCC32) = 0x00;
}

/*
 * pcie_power_enable - Enable 3.3V power rails (stock: 0xE5CB)
 * Address: 0xE5CB-0xE5E4 (26 bytes)
 *
 * Checks if C656 bit 5 already set; if not, sets G_STATE_FLAG_06E6=1,
 * then sets bit 5 on both C656 and C65B (3.3V power rails).
 */
static void pcie_power_enable(void) {
    uint8_t tmp = REG_HDDPC_CTRL;
    if (!(tmp & 0x20)) {
        REG_HDDPC_CTRL = (tmp & 0xDF) | 0x20;        /* C656: set bit 5 (3.3V) */
        REG_PHY_EXT_5B = (REG_PHY_EXT_5B & 0xDF) | 0x20; /* C65B: set bit 5 */
    }
}

/*
 * pcie_tunnel_enable - Full PCIe tunnel bringup (stock: 0xC00D via 0xE8E4)
 * Address: 0xC00D-0xC054 (72 bytes)
 *
 * Called when TUR counter reaches 21. Sequence:
 *   1. pcie_power_enable (3.3V)
 *   2. B480 PERST pulse: assert → deassert
 *   3. pcie_tunnel_setup (adapter config, lane setup)
 *   4. CA06 &= 0xEF, then B480 assert again (via 0x99E0)
 *   5. C659 &= 0xFE (12V OFF during tunnel config!)
 *   6. pcie_lane_config (progressive lane enable)
 *   7. Set state = 0x10 (phase 2 pending)
 */
static void pcie_tunnel_enable(void) {
    uint8_t tmp;

    uart_puts("[TEN]\n");

    /* Apply PHY PLL config for PCIe (stock: 0x8E28-0x8E5E).
     * E741/E742 are shared USB/PCIe PHY PLL registers. Cannot call at boot
     * because PCIe PLL values break USB SuperSpeed detection.
     * Called here after USB enumeration is complete. */
    pcie_pll_init();

    /* Stock DE16→DE24: Timer block init (runs before BFE0 wrapper)
     * Clears globals 0x0B30-0x0B33, then initializes CD30/CD32/CD33 timer
     * and CC2A timer mode. Stock before-training dump: CD30=0x15, CC2A=0x04.
     * This is called from the state machine dispatch at E96C → DE16.
     *
     * DE16: 0x0B30-0x0B33 = 0, lcall E726 (CD31 clear), then DE24:
     *   CD30 = (CD30 & 0xF8) | 0x05, CD32 = 0x00, CD33 = 0xC7
     *   CC2A = (CC2A & 0xF8) | 0x04, CC2C = 0xC7, CC2D = 0xC7
     */
    XDATA_REG8(0x0B30) = 0; XDATA_REG8(0x0B31) = 0;
    XDATA_REG8(0x0B32) = 0; XDATA_REG8(0x0B33) = 0;
    XDATA_REG8(0xCD31) = 0x04;  /* Timer clear (E726) */
    XDATA_REG8(0xCD31) = 0x02;
    REG_PHY_DMA_CMD_CD30 = (REG_PHY_DMA_CMD_CD30 & 0xF8) | 0x05;  /* Mode 5 */
    REG_PHY_DMA_ADDR_LO = 0x00;   /* CD32 */
    REG_PHY_DMA_ADDR_HI = 0xC7;   /* CD33 */
    REG_CPU_KEEPALIVE = (REG_CPU_KEEPALIVE & 0xF8) | 0x04;  /* CC2A mode 4 */
    REG_CPU_KEEPALIVE_CC2C = 0xC7;
    REG_CPU_KEEPALIVE_CC2D = 0xC7;

    /* Stock firmware wrapper at BFE0-C00A runs before tunnel_enable:
     * 1. C6A8 |= 0x01 (CB05)
     * 2. 92C8 &= 0xFC (clear bits 0,1)
     * 3. CD31 = 0x04, 0x02 (clear timer 4)
     * 4. D47F: timer 1 init (CC16/CC17/CC18/CC19) + 92C4/9201 config
     * 5. D559: timer 3 init (CC22/CC23)
     * 6. E19E: timer 2 init (CC1C/CC1D/CC1E/CC1F) + timer 5 (CC5C/CC5D/CC5E/CC5F) */

    /* C6A8 set bit 0 (CB05) */
    REG_PHY_CFG_C6A8 = (REG_PHY_CFG_C6A8 & 0xFE) | 0x01;

    /* 92C8 clear bits 0,1 */
    XDATA_REG8(0x92C8) = XDATA_REG8(0x92C8) & 0xFC;

    /* CD31 timer 4 clear */
    XDATA_REG8(0xCD31) = 0x04;
    XDATA_REG8(0xCD31) = 0x02;

    /* D47F: Timer 1 init (for 0x0AE5=0 path) */
    XDATA_REG8(0xCC17) = 0x04;  /* timer 1 clear */
    XDATA_REG8(0xCC17) = 0x02;
    XDATA_REG8(0xCC16) = (XDATA_REG8(0xCC16) & 0xF8) | 0x04;  /* mode 4 */
    XDATA_REG8(0xCC18) = 0x01;  /* threshold hi */
    XDATA_REG8(0xCC19) = 0x90;  /* threshold lo */
    /* 92C4 clear bit 0, 9201 set/clear bit 0 (when 0x0AE5=0) */
    XDATA_REG8(0x92C4) = XDATA_REG8(0x92C4) & 0xFE;
    XDATA_REG8(0x9201) = (XDATA_REG8(0x9201) & 0xFE) | 0x01;
    XDATA_REG8(0x9201) = XDATA_REG8(0x9201) & 0xFE;

    /* D559: Timer 3 init (for 0x0AE9=0 path) */
    XDATA_REG8(0xCC23) = 0x04;  /* timer 3 clear */
    XDATA_REG8(0xCC23) = 0x02;
    XDATA_REG8(0xCC22) = (XDATA_REG8(0xCC22) & 0xE8) | 0x07;  /* clear bit 4, mode 7 */

    /* E19E: Timer 2 + Timer 5 init */
    XDATA_REG8(0xCC1D) = 0x04;  /* timer 2 clear */
    XDATA_REG8(0xCC1D) = 0x02;
    XDATA_REG8(0xCC5D) = 0x04;  /* timer 5 clear */
    XDATA_REG8(0xCC5D) = 0x02;
    XDATA_REG8(0xCC1C) = (XDATA_REG8(0xCC1C) & 0xF8) | 0x06;  /* timer 2 mode 6 */
    XDATA_REG8(0xCC1E) = 0x00;  /* timer 2 threshold hi */
    XDATA_REG8(0xCC1F) = 0x8B;  /* timer 2 threshold lo */
    XDATA_REG8(0xCC5C) = (XDATA_REG8(0xCC5C) & 0xF8) | 0x04;  /* timer 5 mode 4 */
    XDATA_REG8(0xCC5E) = 0x00;  /* timer 5 threshold hi */
    XDATA_REG8(0xCC5F) = 0xC7;  /* timer 5 threshold lo */

    /* Stock 0xC370-0xC393: GPIO/power config BEFORE tunnel setup.
     * Emulator trace cycle ~73946: C655, C620, C65A writes.
     * C655: bit 0 set based on link type (for r7!=1, set bit 0)
     * C620: clear bits 0-4
     * C65A: set bit 0 */
    XDATA_REG8(0xC655) = (XDATA_REG8(0xC655) & 0xFE) | 0x01;
    XDATA_REG8(0xC620) = XDATA_REG8(0xC620) & 0xE0;
    XDATA_REG8(0xC65A) = (XDATA_REG8(0xC65A) & 0xFE) | 0x01;

    /* Step 1: Enable 3.3V power (stock: lcall 0xE5CB) */
    pcie_power_enable();

    /* Stock first tunnel_setup (stock: lcall 0xCD6C at ~C00D)
     * Emulator trace cycle ~74060: CA06, B4xx adapter config, B401/B482/B480 */
    pcie_tunnel_setup();

    /* Stock DMA config (B264-B281): happens AFTER first tunnel_setup, BEFORE
     * the BFE0 wrapper (timer inits). Emulator trace cycle ~74489. */
    REG_PCIE_DMA_SIZE_A = 0x08;  REG_PCIE_DMA_SIZE_B = 0x00;
    REG_PCIE_DMA_SIZE_C = 0x08;  REG_PCIE_DMA_SIZE_D = 0x08;
    REG_PCIE_DMA_BUF_A = 0x08;  REG_PCIE_DMA_BUF_B = 0x20;
    REG_PCIE_DMA_BUF_C = 0x08;  REG_PCIE_DMA_BUF_D = 0x28;
    REG_PCIE_DMA_CFG_50 = 0x00;
    REG_PCIE_DOORBELL_CMD = 0x00;

    /* Stock D14D-D166: CEF3/CEF2 direct writes, CEF0/CEEF read-modify-write
     * CEF3 = 0x08, CEF2 = 0x80
     * CEF0 &= 0xF7 (clear bit 3), CEEF &= 0x7F (clear bit 7) */
    XDATA_REG8(0xCEF3) = 0x08;
    XDATA_REG8(0xCEF2) = 0x80;
    XDATA_REG8(0xCEF0) = XDATA_REG8(0xCEF0) & 0xF7;
    XDATA_REG8(0xCEEF) = XDATA_REG8(0xCEEF) & 0x7F;

    /* Stock D167-D178: C807 and B281 read-modify-write
     * C807 = (C807 & 0xFB) | 0x04 — set bit 2
     * B281 = (B281 & 0xCF) | 0x10 — set bit 4, clear bit 5 */
    XDATA_REG8(0xC807) = (XDATA_REG8(0xC807) & 0xFB) | 0x04;
    XDATA_REG8(0xB281) = (XDATA_REG8(0xB281) & 0xCF) | 0x10;

    /* Step 2: B401 pulse (stock: 0xC02C-0xC035)
     * lcall 0x99E4 with DPTR=B401 → B401 |= 0x01
     * then reads B401, clears bit 0 → B401 &= 0xFE */
    REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;
    tmp = REG_PCIE_TUNNEL_CTRL;
    REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;

    /* Step 3: Second tunnel setup (stock calls CD6C twice, emulator ~74904)
     * This reconfigures the adapter after DMA setup. */
    pcie_tunnel_setup();

    /* Step 4: CA06 &= 0xEF, then B480 |= 0x01 (stock: 0xC039-0xC03F via lcall 0x99E0)
     * tunnel_setup already did CA06 &= 0xEF, this is the second time (no-op)
     * but B480 |= 1 is important */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;

    /* Step 5: C659 &= 0xFE — 12V OFF during tunnel config (stock: 0xC042-0xC044)
     * SKIPPED: Keep 12V on to avoid killing link during phase2 power transition */
    /* REG_PCIE_LANE_CTRL_C659 = REG_PCIE_LANE_CTRL_C659 & 0xFE; */

    /* Step 6: Lane config (stock: lcall 0xD436 with r7=0x0F) */
    pcie_lane_config();

    /* Step 7: Set phase 2 pending (stock sets XDATA[0x05B4] = 0x10) */
    pcie_phase2_pending = 1;
    pcie_initialized = 1;
    XDATA_REG8(0x0F00) = 0xA0;  /* Tunnel enable complete, phase 2 pending */
    uart_puts("[TE1]\n");
}

/*
 * pcie_phase2 - Phase 2 PCIe link training (stock: 0x9078-0x909B)
 *
 * Called from main loop when pcie_phase2_pending is set.
 * Sequence:
 *   1. Timer wait (~300ms) with mode=4
 *   2. CA81 clear bit 0 (stock: 0xA840 partial)
 *   3. E764 link training config (stock: 0xCDC6)
 *   4. Timer wait (~2s) for link to come up
 *   5. C659 |= 0x01 (12V ON — only now!)
 *   6. Timer wait (~1s) settling
 *   7. Clear phase 2 pending
 */
static void pcie_phase2(void) {
    uint8_t tmp;

    uart_puts("[P2]\n");
    /* Debug marker: write phase 2 progress to scratch XDATA */
    XDATA_REG8(0x0F00) = 0x01;  /* Phase 2 started */

    /* Step 1: Timer wait ~100ms (stock: r5=0x2B, r4=0x01 ~300ms, mode=4)
     * Reduced from 300ms to minimize USB blocking time.
     * Stock uses CC11/CC12/CC13 timer, we use Timer0. */
    timer_wait(0x00, 0x64, 0x04);

    XDATA_REG8(0x0F00) = 0x02;  /* After timer 1 */

    /* Step 2: A840 function — full implementation for 0x0AEC=0, 0x0AED=0
     * Stock: CA81 clear, CA06 config, B403 set, bank_write 0x40B0,
     *   B431 = (B431 & 0xF0) | 0x0E, then pcie_lane_config(mask=0x0E) */
    XDATA_REG8(0xCA81) = XDATA_REG8(0xCA81) & 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x20;
    XDATA_REG8(0xB403) = (XDATA_REG8(0xB403) & 0xFE) | 0x01;

    /* Stock 9A46: bank_write 0x40B0 — read upper nibble, set lower to 0x01
     * bank_read(bank=2, addr=0x40B0) → (result & 0xF0) | 0x01 → bank_write */
    __asm
        mov     0x93, #0x01
        mov     dptr, #0x40B0
        movx    a, @dptr        ; Read 0x40B0 bank 2
        anl     a, #0xF0        ; Keep upper nibble
        orl     a, #0x01        ; Set lower nibble to 0x01
        movx    @dptr, a        ; Write back
        mov     0x93, #0x00     ; Clear bank select
    __endasm;

    /* Stock A911-A92D: 0x0A5C = 0x0E, B431 = (B431 & 0xF0) | 0x0E */
    XDATA_REG8(0x0A5C) = 0x0E;
    XDATA_REG8(0xB431) = (XDATA_REG8(0xB431) & 0xF0) | 0x0E;

    /* Stock A93C: pcie_lane_config with mask=0x0E (reconfigure lanes) */
    pcie_lane_config_mask(0x0E);

    /* Step 3: E764 link training config (stock: 0xCDC6 → 0xE7D4)
     * E7D4 with r7 bit 0 set: E764 = (E764 & 0xF7) | 0x08, then E764 &= 0xFB */
    tmp = REG_PHY_TIMER_CTRL_E764;
    tmp = (tmp & 0xF7) | 0x08;     /* Set bit 3 */
    REG_PHY_TIMER_CTRL_E764 = tmp;
    tmp = REG_PHY_TIMER_CTRL_E764;
    tmp = tmp & 0xFB;               /* Clear bit 2 */
    REG_PHY_TIMER_CTRL_E764 = tmp;

    /* Step 3b: E764 bits 0,1 config (stock: 0xCDD5-0xCDE1)
     * Clear bit 0, then set bit 1 */
    tmp = REG_PHY_TIMER_CTRL_E764;
    tmp = tmp & 0xFE;               /* Clear bit 0 */
    REG_PHY_TIMER_CTRL_E764 = tmp;
    tmp = REG_PHY_TIMER_CTRL_E764;
    tmp = (tmp & 0xFD) | 0x02;     /* Set bit 1 */
    REG_PHY_TIMER_CTRL_E764 = tmp;

    XDATA_REG8(0x0F00) = 0x03;  /* Before link wait */

    /* Step 4: Wait for link with diagnostic polling.
     * Stock does a single 2s timer (r5=0xCF, r4=0x07, mode=1).
     * We split into smaller intervals to track LTSSM progression. */
    {
        uint8_t poll;
        for (poll = 0; poll < 3; poll++) {
            /* ~100ms per iteration = 300ms total max */
            timer_wait(0x00, 0x64, 0x04);
            {
                uint8_t e762 = XDATA_REG8(0xE762);
                if (e762 & 0x10) {
                    uart_puts("[LK!]\n");
                    break;
                }
            }
        }
    }

    /* Step 4b: Check E762 bit 4 for link result (stock: 0xCDEB-0xCE1F)
     * If E762 bit 4 set: link trained OK
     * If E762 bit 4 clear: link failed */
    tmp = XDATA_REG8(0xE762);
    /* Save debug snapshot to scratch XDATA for readback via E4 */
    XDATA_REG8(0x0F20) = tmp;                        /* E762 value at decision point */
    XDATA_REG8(0x0F21) = XDATA_REG8(0xB450);         /* LTSSM at decision point */
    XDATA_REG8(0x0F22) = REG_PHY_TIMER_CTRL_E764;    /* E764 at decision point */
    XDATA_REG8(0x0F23) = (tmp & 0x10) ? 0x01 : 0x00; /* which path taken */
    if (tmp & 0x10) {
        /* Link trained (stock: 0xCDFC-0xCE08)
         * Stock: E764 |= 0x01, then E764 &= 0xFD
         * TEMPORARILY SKIP E764 modifications to test if they kill the link */
        uart_puts("[LK+]\n");
    } else {
        /* Link failed (stock: 0xCE09-0xCE1E) — clear E764 bits 3,2,0,1 */
        tmp = REG_PHY_TIMER_CTRL_E764;
        REG_PHY_TIMER_CTRL_E764 = tmp & 0xF7;  /* Clear bit 3 */
        tmp = REG_PHY_TIMER_CTRL_E764;
        REG_PHY_TIMER_CTRL_E764 = tmp & 0xFB;  /* Clear bit 2 */
        tmp = REG_PHY_TIMER_CTRL_E764;
        REG_PHY_TIMER_CTRL_E764 = tmp & 0xFE;  /* Clear bit 0 */
        tmp = REG_PHY_TIMER_CTRL_E764;
        REG_PHY_TIMER_CTRL_E764 = tmp & 0xFD;  /* Clear bit 1 */
        uart_puts("[LK-]\n");
    }

    /* Step 5: C659 |= 0x01 — 12V ON (stock: 0xE8D9 with r7=1 → lcall 0xCC8B)
     * CC8B: read, clear bit 0, set bit 0, write (net: set bit 0)
     *
     * NOTE: In stock firmware, 12V was turned OFF during tunnel_enable and
     * turned back ON here. But we keep 12V on from boot to allow GPU to
     * initialize. Since C659 bit 0 is already set, this is a no-op. */
    REG_PCIE_LANE_CTRL_C659 = (REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01;

    /* Step 6: Timer wait ~100ms settling (stock: r5=0xE7, r4=0x03 ~1s, mode=4)
     * Reduced from 1s to minimize USB blocking time. */
    timer_wait(0x00, 0x64, 0x04);

    /* Step 7: Clear phase 2 pending (stock: XDATA[0x05B4] = 0) */
    pcie_phase2_pending = 0;
    pcie_initialized = 2;
    XDATA_REG8(0x0F00) = 0x07;  /* Phase 2 complete */
    uart_puts("[P2D]\n");
}

/*
 * pcie_phy_channel_config - PHY channel config space access engine
 * Based on stock firmware 0xBF96-0xC00C
 *
 * This function performs a PCIe config space access (read or write)
 * via the B2xx register engine:
 *   1. Clear 12 PHY lane registers (B210-B21B) via loop
 *   2. Set B210 format type based on mode (0x40=write, 0x00=read)
 *   3. B213=0x01, B217=0x0F (byte enable), B216=0x20 (trigger)
 *   4. Copy 32-bit address from XDATA[0x05AC-0x05AF] to B218-B21B
 *   5. Trigger B296 sequence and poll for completion
 *   6. For read mode (mode=0): check B296 bit 1, B22C==0, B22D==0, B22B==4
 *
 * mode=1 (E65F): Set XDATA[0x05AB]=1, B210=0x40, return 0 (success)
 * mode=0 (E656): Set XDATA[0x05AB]=0, B210=0x00, check completion data
 *
 * Returns: 0=success/done, 0xFE=timeout, 0xFF=data error
 */
static uint8_t pcie_phy_channel_config(uint8_t mode) {
    uint8_t i;

    /* Set mode flag (stock: E65F sets 0x05AB=1, E656 sets 0x05AB=0) */
    XDATA_REG8(0x05AB) = mode;

    /* Step 1: Clear 12 PHY channel registers (stock: BF96-BFA4 loop)
     * 0x99B0 helper: XDATA[0xB210 + i] = 0 for i=0..11 */
    for (i = 0; i < 12; i++) {
        XDATA_REG8(0xB210 + i) = 0x00;
    }

    /* Step 2: Set format type based on mode (stock: BFA5-BFB5)
     * mode=1 (write/set): B210=0x40
     * mode=0 (read/clear): B210=0x00 */
    if (mode & 0x01) {
        XDATA_REG8(0xB210) = 0x40;
    } else {
        XDATA_REG8(0xB210) = 0x00;
    }

    /* Step 3: B213=0x01 (stock: BFB6-BFBB) */
    XDATA_REG8(0xB213) = 0x01;

    /* Step 4: B217=0x0F, B216=0x20 (stock: BFBE calling 0x998D)
     * 0x998D: B217=A (0x0F), B216=0x20 */
    XDATA_REG8(0xB217) = 0x0F;
    XDATA_REG8(0xB216) = 0x20;

    /* Step 5: Copy 32-bit address from 0x05AC-0x05AF to B218-B21B
     * (stock: BFC1-BFCA, load_dword(0x05AC) then store_dword(B218)) */
    XDATA_REG8(0xB218) = XDATA_REG8(0x05AC);
    XDATA_REG8(0xB219) = XDATA_REG8(0x05AD);
    XDATA_REG8(0xB21A) = XDATA_REG8(0x05AE);
    XDATA_REG8(0xB21B) = XDATA_REG8(0x05AF);

    /* Step 6: Trigger B296 sequence (stock: BFCD calling 0x98FA)
     * B296=0x01, 0x02, 0x04; B254=0x0F */
    XDATA_REG8(0xB296) = 0x01;
    XDATA_REG8(0xB296) = 0x02;
    XDATA_REG8(0xB296) = 0x04;
    XDATA_REG8(0xB254) = 0x0F;

    /* Step 7: Poll B296 bit 2 for completion (stock: BFD0-BFD3 calling 0x9948) */
    {
        uint16_t timeout;
        for (timeout = 10000U; timeout; timeout--) {
            if (XDATA_REG8(0xB296) & 0x04) break;
        }
    }

    /* Step 8: Acknowledge completion (stock: BFD5 calling 0x99F2)
     * B296 = 0x04 */
    XDATA_REG8(0xB296) = 0x04;

    /* Step 9: Check result based on mode */
    if (mode & 0x01) {
        /* Write mode: just return success (stock: BFDF-BFE1) */
        return 0;
    }

    /* Read mode: check status registers (stock: BFE2-C00C) */
    {
        uint16_t timeout;
        for (timeout = 10000U; timeout; timeout--) {
            uint8_t b296 = XDATA_REG8(0xB296);
            if (b296 & 0x02) {
                /* Bit 1 set: success — check completion data */
                /* Stock: 0x99D1 writes 0x02 to B296, reads B22C */
                XDATA_REG8(0xB296) = 0x02;
                if (XDATA_REG8(0xB22C) != 0x00) return 0xFF;
                if (XDATA_REG8(0xB22D) != 0x00) return 0xFF;
                if (XDATA_REG8(0xB22B) != 0x04) return 0xFF;
                /* Success (stock: C006 calls 0x99BD → reads B22A[7:5]) */
                return 0;
            }
            if (b296 & 0x01) {
                /* Bit 0 set, bit 1 not set: timeout/error */
                XDATA_REG8(0xB296) = 0x01;
                return 0xFE;
            }
        }
    }
    return 0xFE;
}

/*
 * pcie_check_readiness - Check PCIe link readiness via config space read
 * Based on stock firmware 0xE275-0xE291
 *
 * Stock sequence:
 *   1. E4C8(0x1C): Store {0x00, 0xD0, 0x00, 0x1C} to XDATA[0x05AC-0x05AF]
 *   2. E656(): Set 0x05AB=0, run BF96 (config space read at address 0x00D0001C)
 *   3. Check B223 bits [2:1] == 2
 *
 * Returns: 0=ready, 0xFF=not ready
 */
static uint8_t pcie_check_readiness(void) {
    uint8_t result;
    uint8_t b223_field;

    /* Step 1: Set config space address to 0x00D0001C (stock: E4C8 with r7=0x1C)
     * E4C8 computes: r7=0x1C, r6=0, r5=0xD0, r4=0x00
     * Stored big-endian: 0x05AC=r4, 0x05AD=r5, 0x05AE=r6, 0x05AF=r7 */
    XDATA_REG8(0x05AC) = 0x00;
    XDATA_REG8(0x05AD) = 0xD0;
    XDATA_REG8(0x05AE) = 0x00;
    XDATA_REG8(0x05AF) = 0x1C;

    /* Step 2: Run BF96 in read mode (stock: E656 → 0x05AB=0, BF96) */
    result = pcie_phy_channel_config(0);
    if (result != 0) {
        uart_puts("[BFr="); uart_puthex(result); uart_puts("]\n");
        return 0xFF;  /* Config space read failed */
    }

    /* Step 3: Check B223 bits [2:1] (stock: E280-E28E)
     * Stock: rrc a; rrc a; anl a, #0x03 → extracts bits [2:1] shifted to [1:0]
     * Must equal 2 for readiness */
    {
        uint8_t raw = XDATA_REG8(0xB223);
        b223_field = (raw >> 1) & 0x03;
        uart_puts("[B223="); uart_puthex(raw);
        uart_puts(" f="); uart_puthex(b223_field);
        uart_puts("]\n");
    }
    if (b223_field == 0x02) return 0x00;  /* Ready */
    return 0xFF;  /* Not ready */
}

/*
 * pcie_link_train_trigger - PCIe link training via B2xx registers
 * Based on stock firmware 0xAC08 (link_training_init)
 *
 * Stock sequence:
 *   1. Clear B210-B21B (12 registers, via 0x99B0 loop)
 *   2. B210 = 0x44 (Gen4 link speed, IDATA[0x60]=1, IDATA[0x61]=0)
 *   3. B213 = 0x01 (link training command)
 *   4. B217 = 0x03 (lane config from IDATA[0x65]=3)
 *   5. B218-B21B = IDATA[0x61:0x62:0x63:0x64] = {0,0,0,1}
 *   6. B216 = 0x20 (trigger write, via 0x9990)
 *   7. B296 = 0x01, 0x02, 0x04; B254 = 0x0F (trigger, via 0x98FA)
 *   8. Poll B296 bit 2 (completion, via 0x9948)
 *   9. B296 = 0x04 (acknowledge, via 0x99F2)
 *  10. Check B296 bit 1 (success) → B22B==0x04 and B284 bit 0
 */
static uint8_t pcie_link_train_trigger(void) {
    uint8_t i;
    uint16_t timeout;

    /* Step 1: Clear B210-B21B (stock: 0xAC0F loop calling 0x99B0)
     * 0x99B0: adds 0x10 to loop index, writes 0 to B2(10+i) */
    for (i = 0; i < 12; i++) {
        XDATA_REG8(0xB210 + i) = 0x00;
    }

    /* Step 2: B210 = link speed (stock: 0xAC34)
     * IDATA[0x60]=1 (bit 0 set), IDATA[0x61]=0 → r7=0x44 (Gen4)
     * Note: IDATA[0x60]=0 uses 0x04 (Gen1/Gen3 mode).
     * Try Gen4 first; if link drops, consider 0x04. */
    XDATA_REG8(0xB210) = 0x44;

    /* Step 3: B213 = 0x01 (stock: 0xAC3C) */
    XDATA_REG8(0xB213) = 0x01;

    /* Step 4: B217 = IDATA[0x65] & 0x0F = 0x03 (stock: 0xAC44) */
    XDATA_REG8(0xB217) = 0x03;

    /* Step 5: B218-B219 = IDATA[0x61:0x62] = {0,0} (stock: 0xAC48-0xAC50) */
    XDATA_REG8(0xB218) = 0x00;
    XDATA_REG8(0xB219) = 0x00;

    /* Step 6: B21A-B21B computed from IDATA[0x63:0x64] = {0, 1}
     * Stock: AC51-AC7D computes:
     *   r4 = IDATA[0x63] = 0, r5 = IDATA[0x64] = 1
     *   r6 = r4 & 0x03 = 0, a = r5 & 0xC0 = 0
     *   shift right 6: r7 = 0
     *   B21A = (B21A & 0xF0) | r7 = 0 | 0 = 0
     *   r7 = (r5 & 0x3F) * 4 = (1 & 0x3F) * 4 = 4
     *   B21B = (B21B & 0x03) | r7 = 0 | 4 = 4
     * Then calls 0x9990: writes B21B result, then B216=0x20 */
    XDATA_REG8(0xB21A) = (XDATA_REG8(0xB21A) & 0xF0) | 0x00;
    XDATA_REG8(0xB21B) = (XDATA_REG8(0xB21B) & 0x03) | 0x04;

    /* Step 7: B216 = 0x20 (trigger, stock: 0x9990 → 0x9991) */
    XDATA_REG8(0xB216) = 0x20;

    /* Step 8: B296 trigger sequence (stock: 0x98FA)
     * B296=0x01 (reset), B296=0x02 (start), B296=0x04 (trigger), B254=0x0F (mask) */
    XDATA_REG8(0xB296) = 0x01;
    XDATA_REG8(0xB296) = 0x02;
    XDATA_REG8(0xB296) = 0x04;
    XDATA_REG8(0xB254) = 0x0F;

    uart_puts("[LT1]\n");

    /* Step 9: Poll B296 bit 2 for completion (stock: 0xAC83 calling 0x9948)
     * 0x9948: reads B296 & 0x04, returns non-zero when bit 2 set */
    for (timeout = 10000U; timeout; timeout--) {
        if (XDATA_REG8(0xB296) & 0x04) break;
    }

    /* Step 10: Acknowledge completion (stock: 0xAC88 calling 0x99F2)
     * B296 = 0x04 */
    XDATA_REG8(0xB296) = 0x04;

    /* Step 11: Check result (stock: 0xAC8B-0xACDC)
     * Poll B296 for bit 1 (success) or bit 0 (failure) */
    {
        uint8_t b296;
        for (timeout = 10000U; timeout; timeout--) {
            b296 = XDATA_REG8(0xB296);
            if (b296 & 0x02) break;  /* Success bit */
            if (b296 & 0x01) break;  /* Failure bit */
        }

        uart_puts("[B296="); uart_puthex(b296);
        uart_puts(" B22B="); uart_puthex(XDATA_REG8(0xB22B));
        uart_puts(" B284="); uart_puthex(XDATA_REG8(0xB284));
        uart_puts("]\n");

        if (b296 & 0x02) {
            /* Stock: 0x99D1 writes 0x02 to B296, reads B22C */
            XDATA_REG8(0xB296) = 0x02;
            /* Verify completion: B22C==0, B22D==0 (stock: ACAF-ACB6) */
            if (XDATA_REG8(0xB22C) != 0x00) { uart_puts("[LTe1]\n"); return 0; }
            if (XDATA_REG8(0xB22D) != 0x00) { uart_puts("[LTe2]\n"); return 0; }
            /* Check B22B == 0x04 (link width x4, stock: ACB8-ACBE) */
            if (XDATA_REG8(0xB22B) != 0x04) { uart_puts("[LTe3]\n"); return 0; }
            /* Check B284 bit 0 (stock: ACC6-ACCA, for IDATA[0x60]=1 path: skip B284) */
            uart_puts("[LT+]\n");
            return 1;  /* Link trained successfully */
        }
        /* Timeout/failure: ack bit 0 and return 0 */
        if (b296 & 0x01) {
            XDATA_REG8(0xB296) = 0x01;
            uart_puts("[LTtmo]\n");
        }
    }
    return 0;
}

/*
 * pcie_perst_deassert - PERST deassert with link detect (stock: 0x35DF-0x367A)
 *
 * Stock sequence:
 *   1. Pre-PERST: B455=0x02/0x04, B2D5=0x01, B296=0x08 (clear/configure)
 *   2. 20ms delay, then B220={0x01,0x40,0x46,0x00} (stock: 0x3623-0x362E)
 *   3. E65F: Set 0x05AB=1, run BF96 (config space write)
 *   4. Readiness poll loop (stock: 0x3634-0x365E):
 *      - timer_wait(0xC7, 0x00, 4) ~200ms
 *      - E275: E4C8(0x1C) + E656 (BF96 read) + check B223[2:1]==2
 *      - Retry until ready or timeout
 *   5. PERST deassert: B480 &= ~0x01
 *   6. B220-B223 write {0x00,0x00,0x00,0x0B} via 0x51E3
 *   7. Link training via 0xE67A → 0xAC08
 *   8. Spin on B455 bit 1 (link detect)
 *   9. B455 = 0x02 (acknowledge link detect)
 */
static uint8_t pcie_perst_deassert(void) {
    uint16_t timeout;
    uint8_t ready;

    uart_puts("[PD]\n");

    /* Step 1: Pre-PERST configuration (stock: 0x35E2-0x35F6) */
    XDATA_REG8(0xB455) = 0x02;  /* Clear link detect status */
    XDATA_REG8(0xB455) = 0x04;  /* Trigger status clear */
    XDATA_REG8(0xB2D5) = 0x01;  /* PCIe config enable */
    XDATA_REG8(0xB296) = 0x08;  /* PCIe trigger reset */

    /* Step 2: Set config space address to 0x00D00014 (stock: 0x361E, E4C8(0x14))
     * E4C8 with r7=0x14: stores {r4=0x00, r5=0xD0, r6=0x00, r7=0x14} to 0x05AC */
    XDATA_REG8(0x05AC) = 0x00;
    XDATA_REG8(0x05AD) = 0xD0;
    XDATA_REG8(0x05AE) = 0x00;
    XDATA_REG8(0x05AF) = 0x14;

    /* Step 3: Write B220 TLP config (stock: 0x3623-0x362E)
     * Stock: r7=0x01, r6=0x40, r5=0x46, r4=0x00
     * store_dword(B220, r4:r5:r6:r7) → B220=r4, B221=r5, B222=r6, B223=r7 */
    XDATA_REG8(0xB220) = 0x00;  /* r4 */
    XDATA_REG8(0xB221) = 0x46;  /* r5 */
    XDATA_REG8(0xB222) = 0x40;  /* r6 */
    XDATA_REG8(0xB223) = 0x01;  /* r7 */

    /* Step 4: E65F — run BF96 in write mode at address 0x00D00014
     * Stock: 0x3631 calls E65F → sets XDATA[0x05AB]=1, runs BF96
     * BF96 clears B210-B21B, sets B210=0x40, B213=0x01, B217=0x0F, B216=0x20
     * Copies address from 0x05AC (0x00D00014) to B218-B21B
     * Triggers B296 sequence, waits for completion */
    pcie_phy_channel_config(1);  /* Write mode (stock: E65F → 0x05AB=1, BF96) */

    uart_puts("[BF+]\n");

    /* Step 5: Readiness poll loop (stock: 0x3634-0x365E)
     * The stock loop reinitializes timeout each iteration and polls
     * E275 (which does a config space READ and checks B223).
     * We limit to 1 iteration to minimize blocking time — USB host
     * disconnects after ~800ms of no response. */
    ready = 0;
    for (timeout = 1; timeout; timeout--) {
        /* Timer wait ~200ms per iteration (stock: 0x3634, r5=0xC7, r4=0x00, mode=4) */
        timer_wait(0x00, 0xC7, 0x04);

        /* E275 readiness check (stock: 0x363D calling 0x0426 → E275) */
        if (pcie_check_readiness() == 0x00) {
            ready = 1;
            uart_puts("[RDY]\n");
            break;
        }
    }

    if (!ready) {
        uart_puts("[RDY-]\n");
        /* Stock firmware takes failure path at 0x3649 — cleanup and return.
         * We continue anyway to try link training. */
    }

    /* Step 6: PERST deassert (stock: 0x365F-0x3665) */
    REG_PCIE_PERST_CTRL = REG_PCIE_PERST_CTRL & 0xFE;  /* B480 &= ~0x01 */
    uart_puts("[PERST-]\n");

    /* Step 7: PCIe TLP config write (stock: 0x51E3 → 0x5203-0x5206)
     * Writes {0x00, 0x00, 0x00, 0x0B} to B220-B223
     * 0x0B = 0x03 | 0x08 (type field from r7=0x03 | 0x08) */
    XDATA_REG8(0xB220) = 0x00;
    XDATA_REG8(0xB221) = 0x00;
    XDATA_REG8(0xB222) = 0x00;
    XDATA_REG8(0xB223) = 0x0B;

    /* Step 8: PCIe link training (stock: 0x51E3 → 0xE67A → 0xAC08) */
    if (pcie_link_train_trigger()) {
        uart_puts("[LTok]\n");
    } else {
        uart_puts("[LTfail]\n");
    }

    /* Step 9: Poll B455 bit 1 for link detect (stock: 0x366B-0x3673)
     * Stock firmware SPINS (tight loop, no timeout) on B455 bit 1.
     * We add a timeout to avoid hanging. */
    for (timeout = 30000U; timeout; timeout--) {
        if (XDATA_REG8(0xB455) & 0x02) break;
    }
    if (timeout == 0) {
        uart_puts("[LD-]\n");
    } else {
        /* Step 10: Acknowledge link detect (stock: 0x3675-0x367A) */
        XDATA_REG8(0xB455) = 0x02;
        uart_puts("[LD+]\n");
    }

    /* Post-link-detect config (stock: 0x3689-0x36AD)
     * Reduced from ~300ms to ~50ms to minimize USB blocking */
    timer_wait(0x00, 0x32, 0x04);
    /* B2D5 read-back and B296 (stock: 0x3697-0x36AD) */
    if (XDATA_REG8(0xB2D5) & 0x01) {
        XDATA_REG8(0xB2D5) = 0x01;
    }
    XDATA_REG8(0xB296) = 0x08;

    return 1;
}

static void pcie_post_train(void) {
    /* Called once after LTSSM reaches L0 */
    REG_PCIE_DMA_SIZE_A = 0x08;  REG_PCIE_DMA_SIZE_B = 0x00;
    REG_PCIE_DMA_SIZE_C = 0x08;  REG_PCIE_DMA_SIZE_D = 0x08;
    REG_PCIE_DMA_BUF_A = 0x08;  REG_PCIE_DMA_BUF_B = 0x20;
    REG_PCIE_DMA_BUF_C = 0x08;  REG_PCIE_DMA_BUF_D = 0x28;
    REG_PCIE_DMA_CFG_50 = 0x00;
    REG_PCIE_DOORBELL_CMD = 0x00;
    XDATA_REG8(0xC428) |= 0x20;
    XDATA_REG8(0xC450) |= 0x04;
    XDATA_REG8(0xC472) &= 0xFE;
    XDATA_REG8(0xC4EB) |= 0x01;
    XDATA_REG8(0xC4ED) |= 0x01;
    REG_CPU_MODE_NEXT &= 0xBF;
}

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
    /* E764/E76C/E774/E77C are NOT written here — they are configured
     * in pcie_early_init (stock: 0xD996). Stock emulator trace confirms
     * E764 is first written at pcie_early_init as 0x00,0x00,0x00,0x04
     * and E76C/E774/E77C are written as 0x00. */
    REG_INT_AUX_STATUS = 0x02; REG_CPU_EXEC_STATUS_3 = 0x00;
    REG_INT_ENABLE = 0x10;
    REG_INT_STATUS_C800 = 0x04; REG_INT_STATUS_C800 = 0x05;
    REG_TIMER_CTRL_CC3B = 0x0D; REG_TIMER_CTRL_CC3B = 0x0F;
    /* CA60 PD controller config (stock: 0x4BE6 PD init at cycle ~51K)
     * Sets bits 1,2 then bit 3. Stock before-training dump: CA60=0x5E */
    REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF8) | 0x06;
    REG_CPU_CTRL_CA60 = (REG_CPU_CTRL_CA60 & 0xF7) | 0x08;
    REG_POWER_CTRL_92C6 = 0x05; REG_POWER_CTRL_92C7 = 0x00;
    REG_USB_CTRL_9201 = 0x0E; REG_USB_CTRL_9201 = 0x0C;
    REG_CLOCK_ENABLE = 0x82; REG_USB_CTRL_920C = 0x61;
    REG_USB_CTRL_920C = 0x60; REG_POWER_ENABLE = 0x87;
    REG_CLOCK_ENABLE = 0x83; REG_PHY_POWER = 0x2F;
    REG_USB_PHY_CONFIG_9241 = 0x10; REG_USB_PHY_CONFIG_9241 = 0xD0;
    /* PHY PLL config — these values are needed for USB PHY to initialize.
     * Stock firmware writes different E741/E742 values AFTER pcie_early_init
     * via function at 0x8E28 — pcie_pll_init() corrects them for PCIe. */
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
    is_usb3 = 0; need_bulk_init = 0; need_dma_setup = 0; bulk_ready = 0; bulk_out_state = 0;
    pcie_initialized = 0; need_pcie_init = 0;
    REG_UART_LCR &= 0xF7;
    uart_puts("\n[BOOT]\n");

    hw_init();

    /* Stock firmware sets CA06 bits 5,6 and CA81 bit 0 during early init
     * (pcie_config_init_a3f5 at instruction ~26559 in trace).
     * CA06=0x61 is required for PCIe link training to work.
     * E716 must also be 0x03 (bits 0,1) - USB set_address may overwrite it. */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;  /* CA06: set bits 5,6 */
    XDATA_REG8(0xCA81) = XDATA_REG8(0xCA81) | 0x01;         /* CA81: set bit 0 */
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03; /* E716: ensure bits 0,1 */

    /* Stock firmware runs pcie_pre_init (CE79) → pcie_early_init (D996) at boot
     * via dispatch table, BEFORE USB enumeration. This configures the PCIe PHY,
     * LTSSM controller (CC3x), and performs initial lane training. */
    pcie_pre_init();
    pcie_early_init();

    /* Stock firmware clears CC32 gate (0x4FDB) after all early init is done.
     * CC32=0x01 was set to enable CC3x writes during pcie_link_controller_init.
     * Leaving it set may prevent LTSSM from operating correctly.
     * NOTE: CC32 clear is safe at boot — doesn't affect USB. */
    pcie_post_early_cleanup();

    /* Stock firmware runs PHY PLL config (0x8E28) AFTER pcie_early_init.
     * However, calling pcie_pll_init here breaks USB enumeration on real hardware
     * (E741/E742 PCIe values prevent USB SuperSpeed detection).
     * Deferred to pcie_tunnel_enable() instead. */

    /* Save boot-time E762 to scratch for diagnosis */
    XDATA_REG8(0x0F30) = XDATA_REG8(0xE762);
    XDATA_REG8(0x0F31) = XDATA_REG8(0xE765);
    XDATA_REG8(0x0F32) = REG_PHY_TIMER_CTRL_E764;

    tur_count = 0;
    pcie_phase2_pending = 0;

    {
        uint8_t link = REG_USB_LINK_STATUS;
        is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
        uart_puts("[link="); uart_puthex(link); uart_puts("]\n");
    }

    /* Power on at boot so GPU can start initializing early.
     * PCIe tunnel/link training will happen after USB SET_CONFIGURATION. */
    pcie_power_enable();
    REG_PCIE_LANE_CTRL_C659 = (REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01;
    uart_puts("[PWR]\n");

    uart_puts("[GO]\n");
    TCON = 0x04;  /* IT0=0 (level-triggered INT0) */
    IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

    while (1) {
        /* Deferred ISR status print */
        if (isr_link_state) {
            if (isr_link_state == 2) uart_puts("[T]\n");
            else if (isr_link_state == 1) uart_puts("[3]\n");
            else if (isr_link_state == 3) {
                uart_puts("[A ");
                uart_puthex(REG_USB_INT_MASK_9090);
                uart_puts("]\n");
            }
            else if (isr_link_state == 4) {
                uart_puts("[R ");
                uart_puthex(REG_USB_LINK_STATUS);
                uart_puts("]\n");
            }
            isr_link_state = 0;
        }

        /* PCIe init — multi-phase state machine.
         * Stock firmware runs these as separate state machine phases
         * with ISR processing between phases. We split across main loop
         * iterations to allow USB interrupts to fire between phases. */
        if (need_pcie_init && pcie_initialized >= 2) {
            /* Reset state to allow re-init after USB reconnect */
            need_pcie_init = 0;
        }
        if (need_pcie_init && pcie_initialized == 0) {
            need_pcie_init = 0;
            uart_puts("[PCIE]\n");
            pcie_tunnel_enable();
            pcie_perst_deassert();
            pcie_initialized = 1;  /* Phase 1 done, phase 2 on next iteration */
        }

        if (pcie_initialized == 1) {
            /* Phase 2: E764 training, 12V power, link check
             * Run on next main loop iteration to allow ISR processing */
            pcie_phase2();
            /* pcie_phase2() sets pcie_initialized = 2 internally */

            {
                uint8_t e762 = XDATA_REG8(0xE762);
                uint8_t ltssm = XDATA_REG8(0xB450);
                uart_puts("[S="); uart_puthex(ltssm);
                uart_puts(" 62="); uart_puthex(e762);
                uart_puts("]\n");
                /* Use E762 bit 4 (link trained) instead of B450 (LTSSM state).
                 * B450 may transiently read 0x00 even when link trained. */
                if (e762 & 0x10) {
                    pcie_post_train();
                    pcie_initialized = 3;
                    uart_puts("[L0!]\n");
                }
            }
        }

        /* Service USB3 link events (91D1) to keep link alive.
         * Stock firmware handles these in ISR at 0x0F4A.
         * Without this, USB3 link dies after ~30-40 seconds. */
        handle_91d1_events();

        /* Must init bulk engine before polling for CBW events */
        if (need_bulk_init) { need_bulk_init = 0; do_bulk_init(); }

        /* Two-phase CBW processing (matches stock ISR flow):
         * Phase 1: DMA state setup (stock: first ISR entry, 0x9000 bit 0 set)
         * Phase 2: CBW parsing + CSW send (stock: second ISR entry, 0x9000 bit 0 clear)
         * Splitting across main loop iterations gives hardware time to configure bulk IN. */
        if (bulk_ready) {
            poll_bulk_events();
            if (need_dma_setup) {
                need_dma_setup = 0;
                cbw_dma_setup();
                need_cbw_process = 1;  /* Process CBW on NEXT iteration */
            } else if (need_cbw_process) {
                need_cbw_process = 0;
                handle_cbw();
            }
        }

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
