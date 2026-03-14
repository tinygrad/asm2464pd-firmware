/*
 * ASM2464PD USB 3.0 Vendor-Class Firmware
 * Bulk IN/OUT via MSC engine, control transfers for enumeration + vendor cmds.
 */

#include "types.h"
#include "registers.h"
#include "globals.h"

__sfr __at(0xA8) IE;
__sfr __at(0x88) TCON;
__sfr __at(0x89) TMOD;
__sfr __at(0xB8) IP;   /* Interrupt Priority */
__sfr __at(0x82) DPL;  /* Data pointer low */
__sfr __at(0x83) DPH;  /* Data pointer high */
__sfr __at(0x93) DPX;  /* DPTR bank select (extended data pointer) */
__sfr __at(0xE0) ACC;  /* Accumulator */
#define IE_EA   0x80
#define IE_EX1  0x04
#define IE_ET0  0x02
#define IE_EX0  0x01
#define IP_PT0  0x02    /* Timer0 high priority */

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

/* All firmware state variables in XSEG, placed at 0x0800+ via -Wl-bXSEG=0x0800
 * to avoid collision with tinygrad E5 writes to 0x0000-0x07FF.
 * Compiler spill slots also in XSEG, so everything is safe from E5 corruption. */
static volatile __xdata uint8_t is_usb3;
static volatile __xdata uint8_t need_bulk_init;
static volatile __xdata uint8_t need_cbw_process;
static volatile __xdata uint8_t need_dma_setup;
static volatile __xdata uint8_t bulk_ready;
static __xdata uint8_t cbw_tag[4];
static volatile __xdata uint8_t bulk_out_state;
static __xdata uint16_t bulk_out_addr;
static __xdata uint8_t bulk_out_len;
static volatile __xdata uint8_t pcie_initialized;
static volatile __xdata uint8_t isr_link_state;
static volatile __xdata uint8_t tur_count;
static volatile __xdata uint8_t pcie_phase2_pending;
static volatile __xdata uint8_t need_pcie_init;
static volatile __xdata uint16_t poll_counter;
static volatile __xdata uint8_t config_done;
static volatile __xdata uint8_t need_rearm;  /* deferred MSC re-arm (main loop) */
static volatile __xdata uint8_t need_state_init;  /* deferred state_init (main loop) */
static volatile __xdata uint8_t pcie_cfg_pending;  /* B297 write seen before PCIe ready */
static volatile __xdata uint8_t cbw_active;  /* 1 while handle_cbw is running */

static void poll_bulk_events(void);
static void handle_cbw(void);
static void direct_bulk_in(uint8_t len);
static void pcie_tunnel_enable(void);
static void pcie_phase2(void);
static void pcie_early_init(void);
static void phy_soft_reset(void);
static void pcie_lane_config(void);
static void phy_set_bit6(void);
static void state_init(void);
static void do_bulk_init(void);
static void pcie_bridge_config_init(void);

/* Bank-switched PHY write: sets SFR 0x93 (DPX)=1, writes to XDATA[addr], restores DPX=0.
 * Stock firmware uses r3=2 → bank helpers compute SFR 0x93 = (r3-1) & 0x7F = 1.
 * Entire function in asm because --model-large passes params via dptr/XDATA. */
static void bank1_write(uint16_t addr, uint8_t val) {
    (void)addr; (void)val;
    __asm
        ; SDCC --model-large: first param (addr) in DPL:DPH at entry
        ; second param (val) at _bank1_write_PARM_2 in XDATA
        mov  r6, dpl                 ; save addr low
        mov  r7, dph                 ; save addr high
        mov  dptr, #_bank1_write_PARM_2
        movx a, @dptr                ; A = val
        mov  dpl, r6                 ; restore addr
        mov  dph, r7
        mov  0x93, #0x01             ; DPX = 1 (bank 1)
        movx @dptr, a                ; write val to bank1 XDATA[addr]
        mov  0x93, #0x00             ; DPX = 0 (restore)
    __endasm;
}

/* Bank-switched read-modify-write: read bank1[addr], OR with mask, write back.
 * Used for 0x6025 |= 0x80 pattern in stock CC83. */
static void bank1_or_bits(uint16_t addr, uint8_t mask) {
    (void)addr; (void)mask;
    __asm
        ; addr in DPL:DPH, mask at _bank1_or_bits_PARM_2
        mov  r6, dpl
        mov  r7, dph
        mov  dptr, #_bank1_or_bits_PARM_2
        movx a, @dptr                ; A = mask
        mov  r5, a                   ; r5 = mask
        mov  dpl, r6                 ; restore addr
        mov  dph, r7
        mov  0x93, #0x01             ; DPX = 1 (bank 1)
        movx a, @dptr                ; read bank1[addr]
        orl  a, r5                   ; OR with mask
        movx @dptr, a                ; write back
        mov  0x93, #0x00             ; DPX = 0 (restore)
    __endasm;
}

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
    /* Configuration descriptor: wTotalLength=0x0079=121 (both alt settings)
     * Matches stock firmware ROM at 0x58CF */
    0x09, 0x02, 0x79, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,

    /* === Alt Setting 0: BOT (Bulk-Only Transport) === */
    /* Interface: iface=0, alt=0, 2 EPs, class=0xFF vendor, sub=0xFF, proto=0xFF
     * Using vendor class prevents kernel uas/usb-storage from claiming device. */
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
    /* EP 0x81 IN bulk 1024, SS companion burst=15 */
    0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x0F, 0x00, 0x00, 0x00,
    /* EP 0x02 OUT bulk 1024, SS companion burst=15 */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x0F, 0x00, 0x00, 0x00,

    /* === Alt Setting 1: UAS (USB Attached SCSI) === */
    /* Interface: iface=0, alt=1, 4 EPs, class=0xFF vendor, sub=0xFF, proto=0xFF */
    0x09, 0x04, 0x00, 0x01, 0x04, 0xFF, 0xFF, 0xFF, 0x00,
    /* EP 0x81 IN bulk 1024, SS companion burst=15 streams=32 (0x05=2^5) */
    0x07, 0x05, 0x81, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,
    /* UAS pipe usage: Data-In pipe (0x03) */
    0x04, 0x24, 0x03, 0x00,
    /* EP 0x02 OUT bulk 1024, SS companion burst=15 streams=32 */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,
    /* UAS pipe usage: Data-Out pipe (0x04) */
    0x04, 0x24, 0x04, 0x00,
    /* EP 0x83 IN bulk 1024, SS companion burst=15 streams=32 */
    0x07, 0x05, 0x83, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x0F, 0x05, 0x00, 0x00,
    /* UAS pipe usage: Status pipe (0x02) */
    0x04, 0x24, 0x02, 0x00,
    /* EP 0x04 OUT bulk 1024, SS companion burst=0 no streams */
    0x07, 0x05, 0x04, 0x02, 0x00, 0x04, 0x00,
    0x06, 0x30, 0x00, 0x00, 0x00, 0x00,
    /* UAS pipe usage: Command pipe (0x01) */
    0x04, 0x24, 0x01, 0x00,
};
static __code const uint8_t bos_desc[] = {
    /* BOS header: 22 bytes total, 2 capabilities */
    0x05, 0x0F, 0x16, 0x00, 0x02,
    /* USB 2.0 Extension: LPM supported (matches stock: bmAttributes=0x1E) */
    0x07, 0x10, 0x02, 0x1E, 0xF4, 0x00, 0x00,
    /* SuperSpeed USB Device Capability (matches stock BOS at 0x59AB):
     *   bmAttributes=0x00 (no LTM)
     *   wSpeedsSupported=0x000E (FS+HS+SS)
     *   bFunctionalitySupport=0x01 (Full Speed = lowest operational)
     *   bU1DevExitLat=0x0A (10us)
     *   wU2DevExitLat=0x07FF (2047us) */
    0x0A, 0x10, 0x03, 0x00, 0x0E, 0x00, 0x01, 0x0A, 0xFF, 0x07,
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

/*
 * ep_init_8fcf - Replicate stock 0x8FCF EP init state variable setup
 *
 * Stock firmware 0x8FCF: parses flash descriptors at 0x7000. With empty
 * flash (all zeros), the checksum at 0x707E != 0x5A, so 8FCF loops 6
 * times via the 0x91C6 skip path, then exits via 0x91CF.
 *
 * Net effect with empty flash:
 *   Preamble (0x8FCF-0x9001): Sets 0x0AEx state variables
 *   Exit path (0x91CF-0x923E): 0x0AEA|=1, 0x0AF0=0, C65A/CC35/905F config
 *
 * These state variables are CRITICAL because state_init's sub-functions
 * D12A and D387 check them to determine which CC timer registers to configure:
 *   - D12A: if 0x0AE4!=0 AND 0x0AEA>=3 → SKIP CC17/CC16/CC18/CC19/92C4/9201
 *   - D387: if 0x0AE8==0x0F → SKIP CC22/CC23 config entirely
 */
/* NOTE: C8Ax registers (0xC8A1-0xC8AF) are the SPI Flash Controller,
 * NOT an endpoint programming engine. Confirmed via UART proxy trace
 * to real hardware: C8AA=REG_FLASH_CMD, C8A9=REG_FLASH_CSR, etc.
 * The stock firmware's 8FCF D8F2 loop does SPI flash reads, not endpoint config.
 * The old program_bulk_endpoint() function was removed — it was writing
 * to flash controller registers, which has no effect on USB endpoint setup. */

static void ep_init_8fcf(void) {
    /* Preamble (stock 0x8FCF-0x9001): Initialize state variables */
    G_FLASH_READ_TRIGGER = 0x00;
    G_STATE_0AE9 = 0x01;
    G_SYSTEM_STATE_0AE2 = 0x01;
    G_STATE_FLAG_0AE3 = 0x01;
    G_LINK_CFG_0AEF = 0x01;
    G_PHY_LANE_CFG_0AE4 = 0x01;
    G_TLP_INIT_FLAG_0AE5 = 0x01;
    G_LINK_SPEED_MODE_0AE6 = 0x01;
    G_LINK_CFG_BIT_0AE7 = 0x01;
    G_STATE_0AE8 = 0x0F;
    G_PHY_CFG_0AED = 0x03;
    G_STATE_CHECK_0AEE = 0x03;
    G_FLASH_CFG_0AEA = 0x03;
    G_LINK_CFG_0AEB = 0x03;
    G_PHY_CFG_0AEC = 0x03;

    /* Skip path counter: stock loops 6x then exits (0x0A83 = 6) */
    G_ACTION_CODE_0A83 = 0x06;

    /* Exit path (stock 0x91CF-0x923E): */

    /* 0x91CF: 0x0AEA |= 0x01 (already 0x03, stays 0x03) */
    G_FLASH_CFG_0AEA = G_FLASH_CFG_0AEA | 0x01;

    /* 0x91D6: check 0x0AE5 — nonzero path: 0x0AF0 = 0x00 */
    G_FLASH_CFG_0AF0 = 0x00;  /* 0x0AE5 = 1 (nonzero) → 0x0AF0 = 0x00 */

    /* 0x91EA: check 0x0AE3 — nonzero path: C65A &= 0xF7 */
    REG_PHY_CFG_C65A = REG_PHY_CFG_C65A & 0xF7;

    /* 0x9205: check 0x0AE2 (=1, nonzero) and 0x0AE5 (=1, nonzero)
     * → CC35 &= 0xFB (stock 0x9217-0x921D) */
    REG_CPU_EXEC_STATUS_3 = REG_CPU_EXEC_STATUS_3 & 0xFB;

    /* 0x921E: 0x0AE2 != 0 → skip E34D call */

    /* 0x9224: 0x0AF0 bit 0 not set (0x0AF0=0x00) → skip D878 */
    /* 0x922E: 0x0AF0 bit 2 not set → skip 0x0480 */

    /* 0x9238: 905F &= 0xEF (clear bit 4) */
    REG_USB_EP_CTRL_905F = REG_USB_EP_CTRL_905F & 0xEF;
}

/*=== SET_CONFIG ===*/
/*
 * handle_set_config - Handle USB SET_CONFIGURATION request
 *
 * Uses the working approach from master branch: direct register manipulation
 * of endpoint config, EP activation, and 9090 interrupt mask setup.
 * This is simpler and proven to work for bulk transfers.
 */
static void handle_set_config(void) {
    uint8_t t;

    /* Write "USBS" to D800 + set MSC length (from master, matches stock 0x494D preamble) */
    REG_USB_EP_BUF_CTRL = 0x55; REG_USB_EP_BUF_SEL = 0x53;
    REG_USB_EP_BUF_DATA = 0x42; REG_USB_EP_BUF_PTR_LO = 0x53;
    REG_USB_MSC_LENGTH = 0x0D;

    /* EP0 config read-back (triggers hardware latch) */
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    t = REG_USB_EP0_CONFIG; REG_USB_EP0_CONFIG = t;
    REG_USB_EP_CFG2 = 0x01; REG_USB_EP_CFG2 = 0x08;

    /* EP status + activate */
    REG_USB_EP_STATUS_90E3 = 0x02;
    t = REG_USB_EP_CTRL_905F; REG_USB_EP_CTRL_905F = t;
    t = REG_USB_EP_CTRL_905D; REG_USB_EP_CTRL_905D = t;
    REG_USB_EP_STATUS_90E3 = 0x01; REG_USB_CTRL_90A0 = 0x01;

    /* Enable global USB interrupt mask bit 7 */
    REG_USB_INT_MASK_9090 |= 0x80;

    /* Status/config read-back */
    t = REG_USB_STATUS; REG_USB_STATUS = t;
    t = REG_USB_CTRL_924C; REG_USB_CTRL_924C = t;

    /* Run bulk init BEFORE ZLP ACK to avoid race with first CBW.
     * Host sends first CBW immediately after SET_CONFIG completes.
     * MSC engine must be armed before the ZLP ACK so hardware can
     * accept the CBW when it arrives. */
    state_init();
    do_bulk_init();
    need_bulk_init = 0;
    need_state_init = 0;

    send_zlp_ack();
    config_done = 1;
    /* Start PCIe init immediately on SET_CONFIG instead of waiting for TUR.
     * tinygrad sends pcie_cfg_req ~1s after SET_CONFIG. PCIe init takes ~5s.
     * Starting early gives PCIe more time to complete before cfg requests arrive. */
    need_pcie_init = 1;
    uart_puts("[C]\n");
}

/*
 * doorbell_dance - 900B/C42A doorbell sequence (stock 0x494D)
 *
 * Stock firmware calls this BOTH for initial MSC arm (from state_init)
 * and for CSW send (from scsi_csw_build). The sequence configures the
 * 900B (MSC config) and C42A (NVMe doorbell) registers in a specific
 * interleaved order, then writes "USBS" to D800, triggers C42C=0x01,
 * and runs post-CSW cleanup.
 *
 * Stock 0x494D-0x49BD: doorbell dance → "USBS" → C42C → ljmp 0xC16C
 */
static void doorbell_dance(void) {
    /* RAMP UP (stock 0x494D-0x496B):
     * 900B: set bits 1,2 → C42A: set bit 0 → 900B: set bit 0 →
     * C42A: set bits 1,2 → C42A: set bit 3 → C42A: set bit 4 */
    REG_USB_MSC_CFG = (REG_USB_MSC_CFG & ~0x02) | 0x02;  /* 900B set bit 1 */
    REG_USB_MSC_CFG = (REG_USB_MSC_CFG & ~0x04) | 0x04;  /* 900B set bit 2 */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x01) | 0x01;  /* C42A set bit 0 */
    REG_USB_MSC_CFG = (REG_USB_MSC_CFG & ~0x01) | 0x01;  /* 900B set bit 0 */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x02) | 0x02;  /* C42A set bit 1 */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x04) | 0x04;  /* C42A set bit 2 */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x08) | 0x08;  /* C42A set bit 3 */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x10) | 0x10;  /* C42A set bit 4 */

    /* TEARDOWN (stock 0x496E-0x0999): */
    REG_USB_MSC_CFG &= ~0x02;     /* 900B clear bit 1 */
    REG_USB_MSC_CFG &= ~0x04;     /* 900B clear bit 2 */
    REG_NVME_DOORBELL &= ~0x01;   /* C42A clear bit 0 */
    REG_USB_MSC_CFG &= ~0x01;     /* 900B clear bit 0 */
    REG_NVME_DOORBELL &= ~0x02;   /* C42A clear bit 1 */
    REG_NVME_DOORBELL &= ~0x04;   /* C42A clear bit 2 */
    REG_NVME_DOORBELL &= ~0x08;   /* C42A clear bit 3 */
    REG_NVME_DOORBELL &= ~0x10;   /* C42A clear bit 4 */

    /* Stock 0x499A: IDATA[0x6A] = 0 (MSC state clear) */
    __asm
        mov r0, #0x6a
        clr a
        mov @r0, a
    __endasm;

    /* Stock 0x499E-0x49A9: "USBS" signature to D800 (via 0x0D9D helper) */
    EP_BUF(0x00) = 0x55;  /* U */
    EP_BUF(0x01) = 0x53;  /* S */
    EP_BUF(0x02) = 0x42;  /* B */
    EP_BUF(0x03) = 0x53;  /* S */

    /* Stock 0x49AC: 901A = 0x0D (CSW/MSC length) */
    REG_USB_MSC_LENGTH = 0x0D;

    /* Stock 0x49B2-0x49BC: C42C=0x01 (MSC trigger), C42D &= ~0x01 */
    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS &= ~0x01;
}

/*
 * post_csw_cleanup - Post-CSW state cleanup (stock 0xC16C-0xC1DB)
 *
 * Called after every C42C arm (via ljmp from 0x494D).
 * Clears state arrays and toggles C42A bit 5 with nvme_link_init.
 */
static void post_csw_cleanup(void) {
    uint8_t i;

    /* Stock 0xC16C-0xC178: Clear state variables */
    G_USB_CTRL_000B = 0x00;
    G_USB_CTRL_000A = 0x00;
    G_SYS_FLAGS_0052 = 0x00;

    /* Stock 0xC179-0xC1A7: Clear state arrays (32 entries) */
    for (i = 0; i < 32; i++) {
        XDATA_REG8(0x0108 + i) = 0x00;
        XDATA_REG8(0x0171 + i) = 0xFF;
        XDATA_REG8(0x0518 + i) = 0x00;
    }

    /* Stock 0xC1AA-0xC1C6: Additional state init */
    G_USB_WORK_01B4 = 0x00;
    G_STATE_WORK_002D = 0x22;
    G_ENDPOINT_STATE_0051 = 0x21;
    G_IFACE_NUM_002E = 0x22;
    G_EP_ALT_STATE_0050 = 0x21;

    /* Stock 0xC1C4: IDATA[0x0D] = 0x22 */
    __asm
        mov r0, #0x0d
        mov @r0, #0x22
    __endasm;

    /* Stock 0xC1C8-0xC1DA: C42A bit 5 toggle + nvme_link_init */
    REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~0x20) | 0x20;  /* C42A set bit 5 */

    /* nvme_link_init (0xDF5E): C428 &= ~0x08, C473 set bits 6,5 */
    REG_NVME_QUEUE_CFG = REG_NVME_QUEUE_CFG & 0xF7;
    REG_NVME_LINK_PARAM = (REG_NVME_LINK_PARAM & 0xBF) | 0x40;
    REG_NVME_LINK_PARAM = (REG_NVME_LINK_PARAM & 0xDF) | 0x20;

    REG_NVME_DOORBELL &= ~0x20;  /* C42A clear bit 5 */
}

/*
 * state_init - Initialize MSC/DMA state (stock firmware 0xBDA4-0xBE20)
 *
 * Called once from SET_CONFIG (deferred to main loop).
 * Stock flow: clear state vars → CC DMA config → C24C bridge setup →
 *             doorbell dance (0x494D) → post-CSW cleanup (0xC16C)
 *
 * This runs BEFORE do_bulk_init (0xB1C5). The doorbell dance includes
 * the FIRST C42C arm. do_bulk_init does the SECOND C42C arm.
 */
static void state_init(void) {
    /* Stock 0xBDA4-0xBDF9: Clear ~20 state variables */
    G_SYS_FLAGS_07ED = 0x00;
    G_SYS_FLAGS_07EE = 0x00;
    G_EP_DISPATCH_OFFSET = 0x00;
    G_SYS_FLAGS_07EB = 0x00;
    G_STATE_FLAG_0AF1 = 0x00;
    G_LINK_POWER_STATE_0ACA = 0x00;
    G_USB_CTRL_STATE_07E1 = 0x05;  /* stock sets to 0x05, not 0 */
    G_USB_TRANSFER_FLAG = 0x00;
    G_TLP_MASK_0ACB = 0x00;
    G_CMD_WORK_E3 = 0x00;
    G_USB_STATUS_07E6 = 0x00;
    G_USB_STATUS_07E7 = 0x00;
    G_TLP_STATE_07E9 = 0x00;
    G_LINK_EVENT_0B2D = 0x00;
    G_STATE_FLAG_07E2 = 0x00;
    G_EP_STATUS_CTRL = 0x00;
    G_WORK_0006 = 0x00;
    G_SYS_FLAGS_07E8 = 0x00;
    G_TRANSFER_ACTIVE = 0x00;
    G_TRANSFER_BUSY_0B3B = 0x00;
    G_XFER_FLAG_07EA = 0x00;

    /* Stock 0xBDFA: call 0x54BB (clear_pcie_enum_done) */
    G_XFER_CTRL_0AF7 = 0x00;

    /* Stock 0xBDFD: call 0xCC56 — C6A8 |= 0x01 */
    REG_PHY_CFG_C6A8 = (REG_PHY_CFG_C6A8 & 0xFE) | 0x01;

    /* Stock 0xBE00-0xBE0A: 92C8 clear bits 0 then 1 (two separate R-M-W) */
    REG_POWER_CTRL_92C8 = REG_POWER_CTRL_92C8 & 0xFE;
    REG_POWER_CTRL_92C8 = REG_POWER_CTRL_92C8 & 0xFD;

    /* Stock 0xBE0B-0xBE13: CD31 reset sequence */
    REG_CPU_TIMER_CTRL_CD31 = 0x04;
    REG_CPU_TIMER_CTRL_CD31 = 0x02;

    /* Stock 0xBE14: call 0xD12A — CC17/CC16/CC18/CC19 bulk DMA config
     * D12A checks 0x0AE4 and 0x0AEA to determine which path to take:
     *   - 0x0AE4==0: CC17 reset + CC16/CC18/CC19 config + 92C4/9201 toggle
     *   - 0x0AE4!=0 AND 0x0AEA>=3: SKIP all CC17 config, SKIP 92C4/9201
     *   - 0x0AE4!=0 AND 0x0AEA<3: CC17 config but SKIP 92C4/9201
     * After ep_init_8fcf: 0x0AE4=1, 0x0AEA=3 → SKIP all */
    G_STATE_WORK_0B3D = 0x00;
    G_CMD_SLOT_INDEX = G_CMD_INDEX_SRC;  /* stock 0xD12F-0xD136 */
    {
        uint8_t ae4 = G_PHY_LANE_CFG_0AE4;
        if (ae4 == 0) {
            /* D147: CC17 reset + config (first-time path) */
            REG_TIMER1_CSR = 0x04;  /* CC17 reset via C033 */
            REG_TIMER1_CSR = 0x02;
            REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04;
            REG_TIMER1_THRESHOLD_HI = 0x01;
            REG_TIMER1_THRESHOLD_LO = 0x90;
            G_STATE_WORK_0B3D = 0x01;
            /* D166: 0x0AE4==0 → 92C4 &= ~0x01, 9201 toggle bit 0 */
            REG_POWER_MISC_CTRL = REG_POWER_MISC_CTRL & 0xFE;
            REG_USB_CTRL_9201 = (REG_USB_CTRL_9201 & 0xFE) | 0x01;
            REG_USB_CTRL_9201 = REG_USB_CTRL_9201 & 0xFE;
        } else {
            uint8_t aea = G_FLASH_CFG_0AEA;
            if (aea < 3) {
                /* D147: CC17 config only (no 92C4/9201) */
                REG_TIMER1_CSR = 0x04;
                REG_TIMER1_CSR = 0x02;
                REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04;
                REG_TIMER1_THRESHOLD_HI = 0x01;
                REG_TIMER1_THRESHOLD_LO = 0x90;
                G_STATE_WORK_0B3D = 0x01;
            }
            /* D166: 0x0AE4!=0 → D17A (return) — skip 92C4/9201 */
        }
    }

    /* Stock 0xBE17: call 0xD387 — CC22/CC23 config
     * D387 checks 0x0AE8:
     *   - 0x0AE8==0x0F: SKIP all CC22/CC23 config (immediate return)
     *   - 0x0AE8!=0x0F: CC23 reset + CC22 config + update 0x0AE8
     * After ep_init_8fcf: 0x0AE8=0x0F → SKIP */
    {
        uint8_t ae8 = G_STATE_0AE8;
        if (ae8 != 0x0F) {
            /* C02B: 0x0B3C=0, CC23 reset (0x04, 0x02) */
            G_STATE_CTRL_0B3C = 0x00;
            REG_TIMER3_CSR = 0x04;
            REG_TIMER3_CSR = 0x02;
            /* CC22: clear bit 4, set bits 0-2 */
            REG_TIMER3_DIV = REG_TIMER3_DIV & 0xEF;
            REG_TIMER3_DIV = (REG_TIMER3_DIV & 0xF8) | 0x07;
            if (ae8 == 0) {
                G_STATE_0AE8 = 0x0F;
            } else if (ae8 < 0x0B) {
                /* D3B4: lookup CC24/CC25 from code table at 0x4DBF+ae8*2 */
                /* For simplicity, set 0x0AE8 = 0x0F (matches final state) */
                G_STATE_0AE8 = 0x0F;
            } else {
                G_STATE_0AE8 = 0x0F;
            }
        }
        /* 0x0AE8==0x0F → D3CE (return) — skip all CC22/CC23 */
    }

    /* Stock 0xBE1A: call 0xDF86 — CC1C/CC5C bulk transfer descriptors
     * First calls E3C6 (CC1D/CC5D reset), then configures CC1C/CC5C */
    G_LOG_ACTIVE_044C = 0x00;  /* E3C6 preamble */
    REG_TIMER2_CSR = 0x04;
    REG_TIMER2_CSR = 0x02;
    REG_TIMER4_CSR = 0x04;
    REG_TIMER4_CSR = 0x02;
    /* CC1C/CC1E/CC1F */
    REG_TIMER2_DIV = (REG_TIMER2_DIV & 0xF8) | 0x06;
    REG_TIMER2_THRESHOLD_LO = 0x00;
    REG_TIMER2_THRESHOLD_HI = 0x8B;
    /* CC5C/CC5E/CC5F */
    REG_TIMER4_DIV = (REG_TIMER4_DIV & 0xF8) | 0x04;
    REG_TIMER4_THRESHOLD_LO = 0x00;
    REG_TIMER4_THRESHOLD_HI = 0xC7;

    /* Stock 0xBE1D: call 0xC24C — post-link bridge setup (checks 0x06E3)
     * If 0x06E3 != 0: clear it, set 0x06E4/E5=1, clear state, do bridge init.
     * We handle bridge config separately in pcie_post_train, but clear the
     * state variables that stock clears here. */
    if (G_USB_STATE_CLEAR_06E3) {
        uint16_t j;
        G_USB_STATE_CLEAR_06E3 = 0x00;
        G_BRIDGE_INIT_06E4 = 0x01;
        G_MAX_LOG_ENTRIES = 0x01;
        G_EP_CONFIG_05A4 = 0x00;
        G_WORK_06E8 = 0x00;
        G_PCIE_BRIDGE_STATE_05A9 = 0x00;
        G_PCIE_BRIDGE_STATE_05AA = 0x00;
        G_XFER_CTRL_0AF7 = 0x00;  /* 54BB again */
        /* Clear 0x05B0-0x0632 area (stock C24C loop via 0x0BBE helper) */
        for (j = 0x05B0; j <= 0x0631; j++) XDATA_REG8(j) = 0x00;
        G_PCIE_ADDR_2 = 0x10;  /* stock C2B4 */
    }

    /* Stock 0xBE20: ljmp 0x494D — doorbell dance + initial C42C */
    doorbell_dance();

    /* Stock: 0x494D ends with ljmp 0xC16C — post-CSW cleanup */
    post_csw_cleanup();

    uart_puts("[SI]\n");
}

/*
 * do_bulk_init - MSC engine initialization (stock firmware 0xB1C5-0xB285)
 *
 * Called from SET_CONFIG AFTER state_init. Configures MSC/NVMe-USB bridge
 * engine registers, does the SECOND C42C arm, then configures DMA descriptors
 * and USB speed settings.
 *
 * ONLY contains stock B1C5 body — no non-stock EP config, buffer clears,
 * MSC toggles, or EP reconfig that were previously mixed in.
 */
/*
 * do_bulk_init - MSC engine initialization (hybrid: stock registers + working logic)
 *
 * Called from SET_CONFIG AFTER state_init. Adds critical stock firmware
 * register writes (0xB1C5-0xB285) while keeping the working MSC arm logic.
 *
 * Key insight: state_init()->doorbell_dance() already does the FIRST C42C arm.
 * This function should do the SECOND arm with proper buffer setup.
 */
static void do_bulk_init(void) {
    uint16_t j;
    uint8_t t;

    /* === Stock 0xB1C5-0xB211: Critical register config === */
    
    /* Stock 0xB1C5-0xB1CD: 92C0 = (92C0 & 0x7F) | 0x80 */
    REG_POWER_ENABLE = (REG_POWER_ENABLE & 0x7F) | 0x80;

    /* Stock 0xB1CE-0xB1D3: 91D1 = 0x0F (clear all link events) */
    REG_USB_PHY_CTRL_91D1 = 0x0F;

    /* Stock 0xB1D4-0xB1E0: Buffer config 9300-9302 */
    REG_BUF_CFG_9300 = 0x0C;
    REG_BUF_CFG_9301 = 0xC0;
    REG_BUF_CFG_9302 = 0xBF;

    /* Stock 0xB1E1-0xB1EC: Control phase and EP config */
    REG_USB_CTRL_PHASE = 0x1F;
    REG_USB_EP_CFG1 = 0x0F;

    /* Stock 0xB1ED-0xB1F2: PHY control */
    REG_USB_PHY_CTRL_91C1 = 0xF0;

    /* Stock 0xB1F3-0xB1FF: Buffer descriptors 9303-9305 */
    REG_BUF_CFG_9303 = 0x33;
    REG_BUF_CFG_9304 = 0x3F;
    REG_BUF_CFG_9305 = 0x40;

    /* Stock 0xB200-0xB20B: USB config registers */
    REG_USB_CONFIG = 0xE0;
    REG_USB_EP0_LEN_H = 0xF0;

    /* Stock 0xB20C-0xB211: 90E2 = 0x01 (MSC gate - CRITICAL for C42C) */
    REG_USB_MODE = 0x01;

    /* Stock 0xB212-0xB218: 905E &= 0xFE (clear EP control bit 0) */
    REG_USB_EP_MGMT = REG_USB_EP_MGMT & 0xFE;

    /* === Original working logic (proven to work for 18/19 tests) === */

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

    bulk_ready = 1;
    uart_puts("[rdy]\n");
}

/*=== Bulk Transfer Engine ===*/

/*
 * send_csw - Send CSW (Command Status Wrapper) to host via bulk IN
 *
 * Stock firmware scsi_csw_build at 0x494D:
 *   1. Doorbell dance (900B/C42A bit ramp-up then teardown)
 *   2. Write "USBS" + tag + residue + status to D800
 *   3. 901A = 0x0D (CSW length)
 *   4. C42C = 0x01 (MSC trigger — sends CSW + re-arms for next CBW)
 *   5. C42D &= ~0x01 (clear status)
 *   6. Post-CSW cleanup at 0xC16C
 *
 * Stock firmware calls C42C=0x01 INSIDE the INT0 ISR, after CE88 DMA
 * handshake. This works because EP_COMPLETE from the initial arm was
 * processed in a SEPARATE ISR invocation (before CBW_RECEIVED).
 * The EP_COMPLETE handler does CC17 DMA descriptor reset, which
 * prepares the hardware for the next C42C.
 */
/*
 * send_csw - Send CSW via MSC engine trigger (from master, proven working)
 *
 * D800 buffer already has "USBS" + tag set up from handle_cbw.
 * We set status + residue, trigger bulk DMA, wait for EP_COMPLETE,
 * then re-arm MSC for next CBW.
 *
 * IMPORTANT: Do NOT clear EP_COMPLETE here. The C42C re-arm does not
 * generate a new EP_COMPLETE. poll_bulk_events() needs to see the
 * EP_COMPLETE from the CSW DMA to set REG_USB_MODE=0x01, which gates
 * the next handle_cbw() call.
 */
/*
 * send_csw - Send CSW via doorbell dance + C802 bulk DMA trigger
 *
 * Matches stock firmware pattern: doorbell dance transitions MSC
 * state machine, then C802 triggers DMA, wait for EP_COMPLETE.
 */
static void send_csw(uint8_t status) {
    uint8_t saved_ie = IE;
    EP_BUF(0x0C) = status;
    EP_BUF(0x08) = 0x00; EP_BUF(0x09) = 0x00;
    EP_BUF(0x0A) = 0x00; EP_BUF(0x0B) = 0x00;

    /* Doorbell dance to transition MSC state machine */
    doorbell_dance();

    /* Disable interrupts during DMA trigger → EP_COMPLETE critical section */
    IE &= ~0x80;  /* EA = 0 */
    /* Clear stale EP_COMPLETE BEFORE triggering DMA */
    if (REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE) {
        REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    }
    REG_USB_BULK_DMA_TRIGGER = 0x01;
    while (!(REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE)) { }
    /* Clear EP_COMPLETE from CSW DMA + reset DMA engine
     * (same as ISR EP_COMPLETE handler - see direct_bulk_in) */
    G_XFER_STATE_0AF4 = 0x40;
    REG_USB_STATUS_909E = 0x01;
    REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    REG_USB_CTRL_90A0 = 0x01;
    REG_USB_BULK_DMA_TRIGGER = 0x00;

    /* CC17 DMA descriptor reset (see direct_bulk_in comment) */
    if (REG_TIMER1_CSR & 0x02) {
        REG_TIMER1_CSR = 0x02;
    }

    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS &= ~0x01;
    IE = saved_ie;

    /* Stock firmware jumps to 0xC16C after every CSW (ljmp from 0x494D).
     * This cleanup resets DMA descriptors, state arrays, and toggles C42A bit 5.
     * Without it, DMA state accumulates and breaks large bulk OUT after many commands. */
    post_csw_cleanup();
}

/*
 * direct_bulk_in - Send data to host via bulk IN using MSC DMA engine
 *
 * Avoids SW DMA mode (DMA_CONFIG=0xA0, 905A=0x10, 90F0=0x01) which
 * poisons hardware state and breaks subsequent large bulk OUT transfers.
 * Instead, uses the same C802 bulk DMA trigger as send_csw().
 *
 * Data must already be in EP_BUF (D800+) before calling.
 * After return, EP_BUF is consumed — caller must restore CSW header.
 */
static void direct_bulk_in(uint8_t len) {
    uint8_t saved_ie;

    REG_USB_MSC_LENGTH = len;

    saved_ie = IE;
    IE &= ~0x80;  /* EA = 0 */
    /* Clear stale EP_COMPLETE */
    if (REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE) {
        REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    }
    REG_USB_BULK_DMA_TRIGGER = 0x01;
    /* Wait for EP_COMPLETE with timeout */
    {
        uint16_t to;
        for (to = 60000; to; to--) {
            if (REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE) break;
        }
        if (!to) {
            uart_puts("[DBI_TO]");
            G_PCIE_LANE_STATE_BASE = REG_USB_PERIPH_STATUS;
            G_PCIE_LANE_STATE_1 += 1;
        }
    }
    /* Stock ISR EP_COMPLETE handler (0x0ED3-0x0EF1):
     * When 909E bit 0 is set after EP_COMPLETE:
     * 1. Write 0x0AF4 = 0x40 (completion indicator)
     * 2. Call 0x54A1 (state check helper - skipped here)
     * 3. Write 909E = 0x01 (acknowledge buffer status)
     * 4. Write 90E3 = 0x02 (EP status clear)
     */
    G_XFER_STATE_0AF4 = 0x40;
    REG_USB_STATUS_909E = 0x01;  /* Ack buffer status */
    REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
    REG_USB_CTRL_90A0 = 0x01;  /* Reset DMA engine */
    REG_USB_BULK_DMA_TRIGGER = 0x00;

    /* MSC state transition after data-IN DMA. */
    REG_USB_MSC_CTRL = 0x01;
    REG_USB_MSC_STATUS &= ~0x01;

    /* Reset CE88/CE89 DMA state after bulk IN.
     * The C802 bulk DMA trigger may leave CE89 in a state that causes
     * subsequent CE88 handshakes (used by bulk OUT) to fail.
     * Doing a dummy CE88 handshake here ensures the state machine is
     * ready for the next bulk operation. */
    REG_BULK_DMA_HANDSHAKE = 0x00;
    {
        uint16_t wt;
        for (wt = 1000; wt; wt--) {
            if (REG_USB_DMA_STATE & USB_DMA_STATE_READY) break;
        }
    }

    IE = saved_ie;

    REG_USB_MSC_LENGTH = 0x0D;  /* Restore to CSW length */
}

/*=== CBW Handler (from master, proven working) ===*/
static void handle_cbw(void) {
    uint8_t opcode;

    G_PCIE_CFG_STATUS_1++;  /* diag: handle_cbw call count (before 90E2 check) */
    G_PCIE_CFG_STATUS_0 = REG_USB_MODE;  /* diag: 90E2 value at entry */
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
    G_PCIE_CFG_STATUS_3 = opcode;  /* diag: last CBW opcode */
    G_PCIE_CFG_STATUS_2++;         /* diag: CBW count */
    uart_puts("[CBW:"); uart_puthex(opcode); uart_puts("]\n");

    if (opcode == 0xE5) {
        uint8_t val = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        XDATA_REG8(addr) = val;
        /* Track B297 writes (PCIe config trigger) during PCIe init.
         * If link isn't up yet, the hardware can't complete the config access.
         * We'll re-trigger B297 after PCIe init completes. */
        if (addr == 0xB297 && val == 0x01 && pcie_initialized < 3)
            pcie_cfg_pending = 1;
        send_csw(0x00);
    } else if (opcode == 0xE4) {
        /* Read XDATA — two modes based on dCBWDataTransferLength:
         *   xfer_len=0: embed up to 4 bytes in CSW residue field (test_bulk.py)
         *   xfer_len>0: data IN phase via direct_bulk_in (tinygrad ReadOp) */
        uint8_t sz = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        uint8_t xfer_len = REG_USB_CBW_XFER_LEN_0;
        if (sz == 0) sz = 1;
        if (xfer_len == 0) {
            /* No data phase — embed in CSW residue (max 4 bytes) */
            uint8_t saved_ie2;
            EP_BUF(0x08) = XDATA_REG8(addr);
            EP_BUF(0x09) = (sz >= 2) ? XDATA_REG8(addr + 1) : 0x00;
            EP_BUF(0x0A) = (sz >= 3) ? XDATA_REG8(addr + 2) : 0x00;
            EP_BUF(0x0B) = (sz >= 4) ? XDATA_REG8(addr + 3) : 0x00;
            EP_BUF(0x0C) = 0x00;
            saved_ie2 = IE;
            IE &= ~0x80;  /* Disable interrupts during DMA trigger */
            /* Clear stale EP_COMPLETE before triggering DMA */
            if (REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE) {
                REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
            }
            REG_USB_BULK_DMA_TRIGGER = 0x01;
            while (!(REG_USB_PERIPH_STATUS & USB_PERIPH_EP_COMPLETE)) { }
            /* Clear EP_COMPLETE + reset DMA engine after completion */
            REG_USB_EP_STATUS_90E3 = 0x02; REG_USB_EP_READY = 0x01;
            REG_USB_STATUS_909E = 0x01;  /* Ack buffer status (stock 0x0EE9) */
            REG_USB_CTRL_90A0 = 0x01;  /* Reset DMA engine */
            REG_USB_BULK_DMA_TRIGGER = 0x00;
            REG_USB_MSC_CTRL = 0x01;
            REG_USB_MSC_STATUS &= ~0x01;
            IE = saved_ie2;
            post_csw_cleanup();
        } else {
            /* Data IN phase */
            uint8_t i;
            for (i = 0; i < sz; i++) EP_BUF(i) = XDATA_REG8(addr + i);
            direct_bulk_in(sz);
            EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
            EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
            EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
            EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
            send_csw(0x00);
        }
    } else if (opcode == 0xE6) {
        uint8_t len = REG_USB_CBWCB_1;
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        uint8_t i;
        if (len == 0) len = 64;
        for (i = 0; i < len; i++) EP_BUF(i) = XDATA_REG8(addr + i);
        direct_bulk_in(len);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0xE7) {
        /* Bulk OUT: receive data synchronously, copy to XDATA[addr] */
        uint16_t addr = ((uint16_t)REG_USB_CBWCB_3 << 8) | REG_USB_CBWCB_4;
        uint8_t len = REG_USB_CBWCB_1;
        uint8_t ci;
        if (len == 0) len = 64;

        /* Arm bulk OUT endpoint for data phase */
        REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
        REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;

        /* Wait for bulk OUT data */
        while (!(REG_USB_PERIPH_STATUS & USB_PERIPH_BULK_DATA)) { }

        /* Receive data via DMA handshake */
        REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
        REG_INT_AUX_STATUS = (REG_INT_AUX_STATUS & 0xF9) | 0x02;
        REG_BULK_DMA_HANDSHAKE = 0x00;
        while (!(REG_USB_DMA_STATE & USB_DMA_STATE_READY)) { }

        /* Copy from DMA buffer to target address */
        for (ci = 0; ci < len; ci++)
            XDATA_REG8(addr + ci) = XDATA_REG8(0x7000 + ci);

        send_csw(0x00);
    } else if (opcode == 0xE8) {
        /* No-data vendor ping — do NOT trigger PCIe init here.
         * PCIe init kills USB3 (bank-switched PHY writes).
         * Only TUR (0x00) triggers PCIe init (matches tinygrad flow). */
        send_csw(0x00);
    } else if (opcode == 0x00) {
        /* SCSI TEST_UNIT_READY */
        tur_count++;
        G_PCIE_TRAIN_STATE_0 = tur_count;  /* diag: TUR count */
        G_PCIE_TRAIN_STATE_1 = pcie_initialized;  /* diag: pcie state */
        if (tur_count >= 1 && pcie_initialized == 0) {
            need_pcie_init = 1;
            G_PCIE_TRAIN_STATE_2 = 0xAA;  /* diag: pcie init triggered */
        }
        send_csw(0x00);
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
        direct_bulk_in(resp_len);
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
        direct_bulk_in(8);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x1A) {
        /* SCSI MODE_SENSE(6) — return minimal response */
        EP_BUF(0x00) = 0x03; EP_BUF(0x01) = 0x00;
        EP_BUF(0x02) = 0x00; EP_BUF(0x03) = 0x00;
        direct_bulk_in(4);
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
        direct_bulk_in(resp_len);
        EP_BUF(0x00) = 0x55; EP_BUF(0x01) = 0x53;
        EP_BUF(0x02) = 0x42; EP_BUF(0x03) = 0x53;
        EP_BUF(0x04) = cbw_tag[0]; EP_BUF(0x05) = cbw_tag[1];
        EP_BUF(0x06) = cbw_tag[2]; EP_BUF(0x07) = cbw_tag[3];
        send_csw(0x00);
    } else if (opcode == 0x8A) {
        /* SCSI WRITE(16) — receive bulk OUT data from host.
         * Tinygrad sends 64KB chunks: CBW → 64KB data OUT → CSW.
         * Data must land in internal DMA buffer (PCIe visible at 0x200000).
         *
         * Approach: Arm bulk OUT endpoint to receive data in 1024-byte chunks
         * (USB SS max packet size), loop until all data received, using CE88/CE89
         * DMA handshake to route each chunk to the right buffer location.
         * Same mechanism as E7 but in a loop for large transfers. */
        uint32_t xfer_len;
        uint32_t received;
        uint16_t chunk;

        /* Read transfer length from CBW */
        xfer_len = ((uint32_t)REG_USB_CBW_XFER_LEN_3 << 24) |
                   ((uint32_t)REG_USB_CBW_XFER_LEN_2 << 16) |
                   ((uint32_t)REG_USB_CBW_XFER_LEN_1 << 8) |
                   REG_USB_CBW_XFER_LEN_0;

        uart_puts("[W16:"); uart_puthex((uint8_t)(xfer_len >> 16));
        uart_puthex((uint8_t)(xfer_len >> 8)); uart_puthex((uint8_t)xfer_len);
        uart_puts(" 93="); uart_puthex(REG_USB_EP_CFG1);
        uart_puts(" 94="); uart_puthex(REG_USB_EP_CFG2);
        uart_puts(" 5A="); uart_puthex(REG_USB_EP_CFG_905A);
        uart_puts("]\n");

        /* Reset bulk endpoint state before bulk OUT data phase.
         * After E4 data-IN ops, the endpoint may be left in bulk IN mode.
         * Stock firmware resets 905A=0x00 before bulk OUT at various places. */
        REG_USB_EP_CFG_905A = 0x00;
        REG_USB_EP_CFG1 = 0x00;
        REG_USB_EP_CFG2 = 0x00;

        /* Receive bulk OUT data in chunks.
         * Arm endpoint → wait for BULK_DATA → CE88 handshake → repeat.
         * Data arrives at hardware DMA buffer. We don't copy it — the
         * hardware routes it to the internal buffer accessible from PCIe. */
        received = 0;
        while (received < xfer_len) {
            /* Arm bulk OUT endpoint for data phase */
            REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
            REG_USB_EP_CFG2 = USB_EP_CFG2_ARM_OUT;

            /* Wait for bulk OUT data with timeout for diagnostics */
            {
                uint16_t wt;
                for (wt = 60000; wt; wt--) {
                    if (REG_USB_PERIPH_STATUS & USB_PERIPH_BULK_DATA) break;
                    REG_CPU_KEEPALIVE = 0x0C;
                }
                if (!wt) {
                    /* Timed out — dump diagnostic state */
                    G_PCIE_LANE_STATE_2 = (uint8_t)(received >> 8);  /* received KB */
                    G_PCIE_LANE_STATE_3 = REG_USB_PERIPH_STATUS;
                    G_PCIE_LANE_STATE_4 = REG_USB_EP_CFG_905A;
                    G_PCIE_LANE_STATE_5 = REG_DMA_CONFIG;
                    G_PCIE_LANE_STATE_6 = REG_USB_SW_DMA_TRIGGER;
                    G_PCIE_LANE_STATE_7 = REG_USB_BULK_DMA_TRIGGER;
                    uart_puts("[W:STUCK@");
                    uart_puthex((uint8_t)(received >> 8));
                    uart_puts("KB 9101=");
                    uart_puthex(REG_USB_PERIPH_STATUS);
                    uart_puts(" 905A=");
                    uart_puthex(REG_USB_EP_CFG_905A);
                    uart_puts(" C8D4=");
                    uart_puthex(REG_DMA_CONFIG);
                    uart_puts("]\n");
                    /* Try to recover — send error CSW */
                    send_csw(0x01);
                    return;
                }
            }

            /* DMA handshake — receive data */
            REG_USB_EP_CFG1 = USB_EP_CFG1_ARM_OUT;
            REG_INT_AUX_STATUS = (REG_INT_AUX_STATUS & 0xF9) | 0x02;
            REG_BULK_DMA_HANDSHAKE = 0x00;
            while (!(REG_USB_DMA_STATE & USB_DMA_STATE_READY)) { }

            /* Each USB SS packet is up to 1024 bytes */
            chunk = 1024;
            if (received + chunk > xfer_len) chunk = (uint16_t)(xfer_len - received);
            received += chunk;
        }

        uart_puts("[W:OK]\n");
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
        /* Stock firmware SS_FAIL handler at 0xDB30:
         * Does NOT force USB2 fallback on first SS_FAIL.
         * Instead, writes 92E1 bit 6 and increments counter 0x07E4.
         * Only after 5 failures does it do heavy reconfiguration.
         * Our old handler was writing POWER_STATUS, USB_STATUS, PHY_LINK_CTRL,
         * CPU_MODE, 91C0 which CAUSED the disconnect by forcing USB2 mode. */
        G_PCIE_LANE_STATE_7 += 1;  /* SS_FAIL count */
        REG_POWER_EVENT_92E1 = (REG_POWER_EVENT_92E1 & 0xBF) | 0x40;  /* stock: 92E1 set bit 6 */
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

    if (!(REG_USB_PERIPH_STATUS & USB_PERIPH_91D1_EVENT)) return;
    r91d1 = REG_USB_PHY_CTRL_91D1;

    /* bit 3: power management (U1/U2). Stock: 0x9b95
     * Stock does complex clock reconfiguration (92CF/92C1) for U1/U2 transitions.
     * Minimal handler: just ack and set flags. The clock reconfiguration
     * (92CF/92C1/E712 polling) was causing USB3 link instability. */
    if (r91d1 & USB_91D1_POWER_MGMT) {
        REG_USB_PHY_CTRL_91D1 = USB_91D1_POWER_MGMT;
        G_USB_TRANSFER_FLAG = 0;
        REG_TIMER_CTRL_CC3B &= ~TIMER_CTRL_LINK_POWER;
        G_TLP_BASE_LO = 0x01;
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
            /* Stock 0xC4B2-0xC4CA: restore LTSSM + link width after link re-training.
             * CC30 bit 0 = LTSSM enable, E710 lower = 0x04 (link width x4 capable).
             * Always restore CC30 and E710 when link is re-training, regardless of
             * whether this was triggered by SS_FAIL or hardware recovery. */
            REG_CPU_MODE = (REG_CPU_MODE & 0xFE) | 0x01;  /* CC30 set bit 0 */
            REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK) | LINK_RECOVERY_MODE;  /* E710 = 0x04 */
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

    bulk_out_state = 0; need_cbw_process = 0; need_dma_setup = 0; need_bulk_init = 0; bulk_ready = 0; config_done = 0;
}

/*=== Interrupt Handlers ===*/

/*
 * poll_bulk_events - Check for pending bulk transfers
 *
 * Checks 0x9101 for CBW/bulk events.
 */
static void poll_bulk_events(void) {
    uint8_t st = REG_USB_PERIPH_STATUS;
    if (st & USB_PERIPH_EP_COMPLETE) {
        /* Only set REG_USB_MODE here. EP_COMPLETE is cleared by send_csw
         * after each CSW DMA. Redundant clearing here was causing hardware
         * state accumulation that broke bulk OUT after ~100 commands. */
        REG_USB_MODE = 0x01;
    }
    /* Don't trigger CBW processing during E7 bulk-out data phase.
     * CBW_RECEIVED (9101 bit 6) may still be asserted from the E7 CBW;
     * re-entering handle_cbw would corrupt the bulk-out state machine. */
    if ((st & USB_PERIPH_CBW_RECEIVED) && !bulk_out_state) need_cbw_process = 1;
}

void int0_isr(void) __interrupt(0) {
    uint8_t periph_status, phase;

    periph_status = REG_USB_PERIPH_STATUS;

    if (periph_status & USB_PERIPH_LINK_EVENT) handle_link_event();
    handle_91d1_events();

    if ((periph_status & USB_PERIPH_91D1_EVENT) && !(periph_status & USB_PERIPH_CONTROL))
        handle_usb_reset();

    /* BULK_REQ handler: Writing to 9301/9302 interferes with bulk
     * DMA operations and causes EP_COMPLETE timeouts. Disabled for
     * now — hardware seems to handle buffer management autonomously
     * for our use case. */

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

        REG_USB_CONFIG = REG_USB_CONFIG;  /* readback-writeback */
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
            /* SET_INTERFACE: wValL = alt setting (0=BOT, 1=UAS)
             * Stock firmware (0x3D85-0x3DEC):
             *   1. Write wValue to 0x900C-0x900F (hardware alt setting register)
             *   2. Send ZLP (0x391B)
             *   3. Write 0xC1 to XDATA[0x0108+iface] (interface state)
             *   4. Clear XDATA[0x0052]
             *
             * NOTE: On USB3 SuperSpeed, the hardware may auto-complete
             * SET_INTERFACE before firmware sees it (observed 27us completion).
             * When firmware does handle it, the 0x900C writes tell hardware
             * which alt setting is active. */
            {
                uint8_t wIdxL = REG_USB_SETUP_WIDX_L;
                /* Stock 0x3D8D: write alt setting to hardware */
                REG_USB_ALT_SETTING_L = wValL;
                REG_USB_ALT_SETTING_H = wValH;
                REG_USB_ALT_SETTING2_L = wValL;
                REG_USB_ALT_SETTING2_H = wValH;
                send_zlp_ack();
                /* Stock 0x3DA1: mark interface state */
                XDATA_REG8(0x0108 + wIdxL) = 0xC1;
                /* Stock 0x3DEC: clear request state */
                G_SYS_FLAGS_0052 = 0x00;
            }
        } else if (bmReq == 0x00 && (bReq == USB_REQ_SET_SEL || bReq == USB_REQ_SET_ISOCH_DELAY)) {
            send_zlp_ack();
        } else if (bmReq == 0x02 && bReq == 0x01) {
            /* CLEAR_FEATURE(HALT) -- re-arm MSC */
            send_zlp_ack();
            arm_msc();
        } else if (bmReq == 0xC0 && bReq == 0xE4) {
            uint16_t addr = ((uint16_t)wValH << 8) | wValL;
            uint8_t vi;
            for (vi = 0; vi < wLenL; vi++) DESC_BUF[vi] = XDATA_REG8(addr + vi);
            send_descriptor_data(wLenL);
        } else if (bmReq == 0x40 && bReq == 0xE5) {
            uint16_t addr = ((uint16_t)wValH << 8) | wValL;
            XDATA_REG8(addr) = REG_USB_SETUP_WIDX_L;
            send_zlp_ack();
        } else if (bmReq == 0x40 && bReq == 0xE6) {
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

void timer0_isr(void) __interrupt(1) {
    /* Stop Timer0 (one-shot) */
    TCON &= ~0x10;  /* TR0=0 */
    /* Timer0 ISR reserved for future use.
     * C42C is now done inline in send_csw() (matching stock firmware). */
}

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
    /* SKIP: Bank-switched PHY writes kill USB3 SuperSpeed.
     * Stock firmware's bank helpers set SFR 0x93=1 to access internal PHY regs
     * (0x78AF-0x7BAF lanes). These are shared with USB3 PHY and modifying them
     * while USB3 is active crashes the SuperSpeed link.
     * TODO: Either do these before USB3 enumeration, or find a safe way. */
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
    /* Poll until done (bit 1 set), servicing USB bulk during PCIe waits */
    while (!(REG_TIMER0_CSR & 0x02)) {
        poll_bulk_events();
        if (need_cbw_process) { need_cbw_process = 0; handle_cbw(); }
    }
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

    G_PCIE_LINK_WIDTH = 0x41;  /* tunnel: CA06 */
    /* Stock 0xCD6C: CA06 &= 0xEF (clear bit 4) — done first */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;

    G_PCIE_LINK_WIDTH = 0x42;  /* tunnel: adapter config */
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

    G_PCIE_LINK_WIDTH = 0x43;  /* tunnel: bank writes 4084/5084 */
    /* SKIP bank-switched writes 0x4084/0x5084 — may affect USB3 PHY */

    G_PCIE_LINK_WIDTH = 0x44;  /* tunnel: B401/B482/B480 */
    /* Stock 0xCD86-0xCD89: B401 |= 0x01 (via 99E4 helper) */
    REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;

    /* Stock 0xCD8C-0xCD97: B482 |= 0x01 (via 99E4), then B482 = (B482 & 0x0F) | 0xF0 */
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0xFE) | 0x01;
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0x0F) | 0xF0;

    /* Stock 0xCD98-0xCD9E: B401 &= 0xFE, then via 99E0: write to B401, then B480 |= 1 */
    tmp = REG_PCIE_TUNNEL_CTRL;
    REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;  /* B480 |= 1 */

    G_PCIE_LINK_WIDTH = 0x45;  /* tunnel: B430/B298 */
    /* Stock 0xCDA1-0xCDA7: B430 &= 0xFE */
    REG_TUNNEL_LINK_STATE = REG_TUNNEL_LINK_STATE & 0xFE;

    /* Stock 0xCDA8-0xCDB0: B298 = (B298 & 0xEF) | 0x10 */
    REG_PCIE_TUNNEL_CFG = (REG_PCIE_TUNNEL_CFG & 0xEF) | 0x10;

    G_PCIE_LINK_WIDTH = 0x46;  /* tunnel: bank 6043/6025 */
    /* SKIP bank-switched writes 0x6043/0x6025 — may affect USB3 PHY */
}

/*
 * pcie_link_controller_reinit - Link controller re-initialization (stock: 0xCE3D-0xCE92)
 *
 * Called from CD8F (second tunnel_setup path). Similar to pcie_link_controller_init (CF28)
 * but does NOT set CC32=0x01 and sets CA81 bit 0 after each major register group.
 * This is critical for LTSSM re-initialization before lane reconfig.
 *
 * Register sequence:
 *   CC30: set bit 0; CA81: set bit 0
 *   E710: (E710 & 0xE0) | 0x04
 *   C6A8: set bit 0; CA81: set bit 0
 *   CC33: = 0x04
 *   E324: clear bit 2
 *   CC3B: clear bit 0
 *   E717: set bit 0; CA81: set bit 0
 *   CC3E: clear bit 1
 *   CC3B: clear bit 1
 *   CC3B: clear bit 6
 *   E716: (E716 & 0xFC) | 0x03
 *   CC3E: clear bit 0
 *   CC39: set bit 1
 *   CC3A: clear bit 1; CC38: clear bit 1
 *   CA06: (CA06 & 0x1F) | 0x60; CA81: set bit 0
 */
static void pcie_link_controller_reinit(void) {
    /* CE3D-CE42: CC30 set bit 0, then CA81 set bit 0 */
    REG_CPU_MODE = (REG_CPU_MODE & 0xFE) | 0x01;
    REG_CPU_CTRL_CA81 = (REG_CPU_CTRL_CA81 & 0xFE) | 0x01;

    /* CE43-CE48: E710 = (E710 & 0xE0) | 0x04 */
    REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;

    /* CE49-CE4E: C6A8 set bit 0, then CA81 set bit 0 */
    REG_PHY_CFG_C6A8 = (REG_PHY_CFG_C6A8 & 0xFE) | 0x01;
    REG_CPU_CTRL_CA81 = (REG_CPU_CTRL_CA81 & 0xFE) | 0x01;

    /* CE4F-CE54: CC33 = 0x04 */
    REG_CPU_EXEC_STATUS_2 = 0x04;

    /* CE55-CE5B: E324 clear bit 2 */
    REG_LINK_CTRL_E324 = REG_LINK_CTRL_E324 & 0xFB;

    /* CE5C-CE62: CC3B clear bit 0 */
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFE;

    /* CE63-CE68: E717 set bit 0, then CA81 set bit 0 */
    REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;
    REG_CPU_CTRL_CA81 = (REG_CPU_CTRL_CA81 & 0xFE) | 0x01;

    /* CE69: CC3E clear bit 1 */
    REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFD;

    /* CE6C-CE72: CC3B clear bit 1, then clear bit 6 */
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFD;
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xBF;

    /* CE73-CE78: E716 = (E716 & 0xFC) | 0x03 */
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;

    /* CE79-CE82: CC3E clear bit 0 */
    REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFE;

    /* CE83-CE85: CC39 set bit 1 */
    REG_TIMER_CTRL_CC39 = (REG_TIMER_CTRL_CC39 & 0xFD) | 0x02;

    /* CE86-CE89: CC3A clear bit 1; CC38 clear bit 1 */
    REG_TIMER_ENABLE_B = REG_TIMER_ENABLE_B & 0xFD;
    REG_TIMER_ENABLE_A = REG_TIMER_ENABLE_A & 0xFD;

    /* CE8A-CE92: CA06 = (CA06 & 0x1F) | 0x60; then CA81 set bit 0 */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;
    REG_CPU_CTRL_CA81 = (REG_CPU_CTRL_CA81 & 0xFE) | 0x01;
}

/*
 * pcie_tunnel_setup_reinit - Second tunnel setup path (stock: 0xCD8F-0xCDE4)
 *
 * This is the SECOND tunnel_setup call in the stock firmware's tunnel_enable
 * sequence. Unlike the first call (CD6C which just does adapter config),
 * this path does a FULL link controller re-initialization:
 *   1. CE3D: Link controller re-init (CC30, E710, C6A8, CC33, E324, CC3B, etc.)
 *   2. 05ED: PHY soft reset (bank1 ECC3 dispatch)
 *   3. C233 config + timer waits
 *   4. E712 polling (wait for PHY ready)
 *   5. E642: Timer clear
 *   6. DB66: E7E3 write (gated on 0x0AF0)
 *   7. ljmp D9A4: Re-run pcie_early_init (B402, E764, B432, B404, lane config)
 *
 * This is CRITICAL for Gen3 link training — without the full LTSSM re-init
 * and PHY soft reset, the lane reconfig in phase 2 only achieves Gen1 x1.
 */
static void pcie_tunnel_setup_reinit(void) {
    uint8_t tmp;
    uint16_t timeout;

    /* Step 1: CD8F-CD9C: Check CC3F bits, optional CCDD LTSSM reset
     * CC3F bit 1 set → call CCDD (clears CC3F bits 5,6, does timer)
     * CC3F bit 2 not set → skip CCDD
     * We skip CCDD since stock "before" dump shows CC3F=0xF0 (bit 1=0) */

    /* Step 2: CD9D: CE3D link controller re-init */
    pcie_link_controller_reinit();

    /* Step 3: CDA0: 05ED dispatch → bank1 ECC3 (PHY soft reset variant)
     * Stock does a full PHY soft reset (CC37 bit 2 toggle + E712 wait).
     * Previously skipped (USB disconnect), but that was when CC43=0x80 was written.
     * Re-enabling since we no longer write CC43. */
    phy_soft_reset();

    /* Step 4: CDA3-CDA9: C233 &= 0xFC (clear bits 0,1) */
    REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFC;

    /* Step 5: CDAA: C233 = (C233 & 0xFB) | 0x04 (set bit 2) */
    REG_PHY_CONFIG = (REG_PHY_CONFIG & 0xFB) | 0x04;

    /* Step 6: CDAD-CDB5: timer_wait(0x00, 0x14, 0x02) — 20 ticks mode 2 */
    timer_wait(0x00, 0x14, 0x02);

    /* Step 7: CDB6-CDBC: C233 &= 0xFB (clear bit 2) */
    REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFB;

    /* Step 8: CDBD-CDC5: E292 timer start (CC10-CC13 timer with mode 3)
     * E292: CC11=0x04, CC11=0x02 (clear timer), CC10 = (CC10 & 0xF8) | 0x03,
     *       CC12=0x00, CC13=0x0A, CC11=0x01 (start) */
    REG_TIMER0_CSR = 0x04;
    REG_TIMER0_CSR = 0x02;
    REG_TIMER0_DIV = (REG_TIMER0_DIV & 0xF8) | 0x03;
    REG_TIMER0_THRESHOLD_HI = 0x00;
    REG_TIMER0_THRESHOLD_LO = 0x0A;
    REG_TIMER0_CSR = 0x01;

    /* Step 9: CDC6-CDDB: Poll E712 bit 0, bit 1, or CC11 bit 1 (timer expired) */
    for (timeout = 30000; timeout; timeout--) {
        tmp = REG_LINK_STATUS_E712;
        if (tmp & 0x01) break;  /* E712 bit 0 — PHY ready */
        if (tmp & 0x02) break;  /* E712 bit 1 — PHY ready alt */
        if (REG_TIMER0_CSR & 0x02) break;  /* Timer expired */
    }

    /* Step 10: CDDC: E642 — timer clear (CC11=0x04, CC11=0x02) */
    REG_TIMER0_CSR = 0x04;
    REG_TIMER0_CSR = 0x02;

    /* Step 11: CDDF-CDE3: DB66 with r7=0 — E7E3 = 0x00
     * (gated on 0x0AF0 bit 5, but for r7=0 always writes E7E3=0) */
    REG_PHY_LINK_CTRL = 0x00;

    /* Step 12: CDE4: ljmp D9A4 — re-run pcie_early_init equivalent
     * D9A4 sequence:
     *   CB08: B402 &= 0xFD (clear bit 1)
     *   E612(0x0F): C659 &= 0xFE (12V off — we skip, already off from tunnel_enable)
     *   E2E6: E764 reset (clear bits 1,0,3; set bit 2)
     *   D45E(1): B432 = (B432 & 0xF8) | 0x07; B404 = (B404 & 0xF0) | 0x01
     *            E76C/E774/E77C clear bit 4
     *   C7A4(0x0F): pcie_lane_config_mask(0x0F)
     *   E049: phy_set_bit6()
     *   Bank writes: 0x7041 clear bit 6, 0x1507 set bits 2,1
     *
     * D9A4 path re-enabled — needed for downstream LTSSM training.
     * Without this, B450 stays at 0x01 (Detect) instead of reaching 0x78 (L0). */

    /* B402 &= 0xFD (stock CB08) */
    REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;

    /* E764 reset (stock E2E6 with bit 0 set path) */
    {
        uint8_t e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xFD;  REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xFE;  REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = e764 & 0xF7;  REG_PHY_TIMER_CTRL_E764 = e764;
        e764 = REG_PHY_TIMER_CTRL_E764;
        e764 = (e764 & 0xFB) | 0x04;  REG_PHY_TIMER_CTRL_E764 = e764;
    }

    /* B432/B404 config (stock D45E with r7=1) */
    REG_POWER_CTRL_B432 = (REG_POWER_CTRL_B432 & 0xF8) | 0x07;
    REG_PCIE_LINK_PARAM_B404 = (REG_PCIE_LINK_PARAM_B404 & 0xF0) | 0x01;
    REG_SYS_CTRL_E76C = REG_SYS_CTRL_E76C & 0xEF;
    REG_SYS_CTRL_E774 = REG_SYS_CTRL_E774 & 0xEF;
    REG_SYS_CTRL_E77C = REG_SYS_CTRL_E77C & 0xEF;

    /* Progressive lane config (stock C7A4 with r7=0x0F) */
    pcie_lane_config();

    /* PHY set bit6 on all lanes (stock E049) */
    phy_set_bit6();

    /* SKIP bank writes (stock D9BD-D9DC): 0x7041 clear bit 6, 0x1507 set bits 2,1.
     * Bank-switched PHY writes kill USB3 SuperSpeed. */

    uart_puts("[TR]\n");
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
    uint8_t tmp;
    uint8_t b402_saved;

    /* Stock 0xC7A4 (called from 0xACDF via 0xADB0):
     * Full lane reconfiguration with PHY isolation toggle.
     *
     * 1. Save B402 bit 1, clear it (stock E84D)
     * 2. Clear B402 bit 1 via 0xCB08 (B402 &= 0xFD)
     * 3. If mask != 0x0F: set bit 6 on bank 1 register 0x6041 (PHY isolation)
     * 4. Progressive lane enable via 0xBEA0 loop
     * 5. If mask != 0x0F: clear bit 6 on bank 1 register 0x6041
     * 6. Restore B402 bit 1
     * 7. Write B436 lane config
     */

    /* Stock E84D: Save B402 bit 1 and clear it during lane training */
    b402_saved = REG_PCIE_CTRL_B402 & 0x02;
    REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;

    /* Stock 0xCB08: also clear B402 bit 1 (redundant but matches stock) */
    REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 & 0xFD;

    if (mask != 0x0F) {
        /* SKIP: bank-switched write to 0x6041 bit 6 (PHY isolation)
         * Bank-switched PHY writes kill USB3 SuperSpeed. */
    }

    /* Stock 0xBEA0: Progressive lane enable.
     * Reads current B434, progressively transitions to target mask.
     * Each step: write B434, call phy_link_training, wait ~200ms.
     * Loop up to 4 iterations with shifting counter. */
    if (mask == 0x0F) {
        /* Stock C089 for mask=0x0F: progressive enable 0x01→0x03→0x07→0x0F */
        static __code const uint8_t lane_steps[] = {0x01, 0x03, 0x07, 0x0F};
        uint8_t i;
        for (i = 0; i < 4; i++) {
            tmp = REG_PCIE_LINK_STATE;
            REG_PCIE_LINK_STATE = lane_steps[i] | (tmp & 0xF0);
            phy_link_training();
            timer_wait(0x00, 0xC7, 0x02);
        }
    } else {
        /* Stock 0xBEA0 for mask<0x0F: progressive reconfiguration.
         * target=mask (0x0E), current=B434 & 0x0F, counter starts at 0x01.
         * Each iteration: new_state = current & (target | ~counter)
         * Then writes B434, phy_link_training, waits, counter<<=1, up to 4 iters. */
        uint8_t current = REG_PCIE_LINK_STATE & 0x0F;
        uint8_t counter = 0x01;
        uint8_t iter;
        for (iter = 0; iter < 4; iter++) {
            uint8_t new_state;
            if (current == mask) break;  /* Already at target */
            /* Stock: new_state = current & (target | (counter ^ 0x0F)) */
            new_state = current & (mask | (counter ^ 0x0F));
            current = new_state;  /* Update current for next iteration */
            /* Write to B434 (lower nibble, keep upper) */
            tmp = REG_PCIE_LINK_STATE;
            REG_PCIE_LINK_STATE = new_state | (tmp & 0xF0);
            phy_link_training();
            timer_wait(0x00, 0xC7, 0x02);
            counter = counter + counter;  /* counter <<= 1 (stock: add a, 0xe0) */
        }
    }

    if (mask != 0x0F) {
        /* Stock 0xC7D4: Clear bit 6 on bank 1 register 0x6041 (end isolation)
         * SKIPPED: Bank-switched PHY writes (SFR 0x93=1) kill USB3 SuperSpeed */

        /* Stock D44C-D457: Pulse B401 (PERST) */
        REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;
        tmp = REG_PCIE_TUNNEL_CTRL;
        REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;
    }

    /* Stock E85C: Restore B402 bit 1 if it was set */
    if (b402_saved) {
        REG_PCIE_CTRL_B402 = REG_PCIE_CTRL_B402 | 0x02;
    }

    /* Configure B436 lane config register (from stock: 0xD458-0xD47E)
     * Stock 0xC7E8-C808: B436 lower = mask & 0x0E, upper = ~(B404 & 0x0F) << 4 */
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
    /* SKIP: Bank-switched PHY writes kill USB3 SuperSpeed.
     * Would call phy_link_training() and set bit 6 on lanes 0x78AF-0x7BAF.
     * TODO: Find safe way to do these (before USB3, or with PHY quiesce). */
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
    REG_CPU_EXEC_STATUS = 0x01;

    /* CF28: CC30 set bit 0 (via bceb: read, clear bit 0, set bit 0) */
    REG_CPU_MODE = (REG_CPU_MODE & 0xFE) | 0x01;

    /* CF2E-CF33: E710 = (E710 & 0xE0) | 0x04 (bd49 reads E710 & 0xE0, then | 0x04) */
    REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;

    /* CF34-CF37: C6A8 set bit 0 */
    REG_PHY_CFG_C6A8 = (REG_PHY_CFG_C6A8 & 0xFE) | 0x01;

    /* CF3A-CF3F: CC33 = 0x04 (write-only register, reads back 0x00) */
    REG_CPU_EXEC_STATUS_2 = 0x04;

    /* CF40-CF46: E324 clear bit 2 */
    REG_LINK_CTRL_E324 = REG_LINK_CTRL_E324 & 0xFB;

    /* CF47-CF4D: CC3B clear bit 0, then (via bce7) write to CC3B, then E717 set bit 0 */
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFE;
    REG_LINK_CTRL_E717 = (REG_LINK_CTRL_E717 & 0xFE) | 0x01;

    /* CF50-CF53: CC3E clear bit 1 */
    REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFD;

    /* CF54-CF5A: CC3B clear bit 1, then clear bit 6 */
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xFD;
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B & 0xBF;

    /* Restore CC3B bits 0,1 — stock firmware restores these via ISR handlers
     * and other init paths after CF28. Since we call CF28 during TUR handling
     * (not at boot like stock), the ISR may not fire in time to restore them.
     * Stock "before" dump shows CC3B=0x0F, so bits 0,1 must be set. */
    REG_TIMER_CTRL_CC3B = REG_TIMER_CTRL_CC3B | 0x03;

    /* CF5B-CF60: E716 = (E716 & 0xFC) | 0x03 */
    REG_LINK_STATUS_E716 = (REG_LINK_STATUS_E716 & 0xFC) | 0x03;

    /* CF61-CF67: CC3E clear bit 0 */
    REG_CPU_CTRL_CC3E = REG_CPU_CTRL_CC3E & 0xFE;

    /* CF68-CF6E: CC39 set bit 1, then CC3A clear bit 1, CC38 clear bit 1 */
    REG_TIMER_CTRL_CC39 = (REG_TIMER_CTRL_CC39 & 0xFD) | 0x02;
    REG_TIMER_ENABLE_B = REG_TIMER_ENABLE_B & 0xFD;
    REG_TIMER_ENABLE_A = REG_TIMER_ENABLE_A & 0xFD;

    /* CF72-CF77: CA06 = (CA06 & 0x1F) | 0x60 */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;

    /* CF78-CF7E: CA81 set bit 0 */
    REG_CPU_CTRL_CA81 = (REG_CPU_CTRL_CA81 & 0xFE) | 0x01;
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
    REG_LINK_STATUS_E716 = 0x00;
    REG_LINK_STATUS_E716 = 0x03;

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
        tmp = REG_LINK_STATUS_E712;
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
    REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFC;

    /* CE94: BD5E with dptr still from prev context.
     * BD5E: @dptr = (@dptr & 0xFB) | 0x04 — sets bit 2, clears bit 2 (net: set bit 2)
     * The dptr here is from the C233 write, so this operates on C233:
     * C233 = (C233 & 0xFB) | 0x04 → set bit 2 on C233 */
    REG_PHY_CONFIG = (REG_PHY_CONFIG & 0xFB) | 0x04;

    /* CE97-CE9D: timer_wait(0x00, 0x14, 0x02) — ~20 ticks mode 2 */
    timer_wait(0x00, 0x14, 0x02);

    /* CEA0-CEA6: C233 &= 0xFB (clear bit 2) */
    REG_PHY_CONFIG = REG_PHY_CONFIG & 0xFB;

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
        tmp = REG_LINK_STATUS_E712;
        if (tmp & 0x01) break;  /* E712 bit 0 set */
        if (tmp & 0x02) break;  /* E712 bit 1 set */
        if (REG_TIMER0_CSR & 0x02) break;  /* Timer expired (CC11 bit 1) */
    }

    /* CEC6: E8EF — clear timer */
    REG_TIMER0_CSR = 0x04;
    REG_TIMER0_CSR = 0x02;

    /* CEC9-CECB: DD42 with r7=0 — E7E3 = 0x00
     * (DD42 checks global 0x0AF1 bit 5, but for r7=0 it always writes E7E3=0) */
    REG_PHY_LINK_CTRL = 0x00;

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
    REG_SYS_CTRL_E76C = REG_SYS_CTRL_E76C & 0xEF;
    REG_SYS_CTRL_E774 = REG_SYS_CTRL_E774 & 0xEF;
    REG_SYS_CTRL_E77C = REG_SYS_CTRL_E77C & 0xEF;

    /* Stock 0xD9A9 (D436): Full lane config (progressive B434 enable + PHY training) */
    pcie_lane_config();

    /* Stock 0xD9AC (E25E): PHY link training + set bit 6 on all lanes */
    phy_set_bit6();

    /* Stock 0xD9AF-0xD9BA: Bank read 0x7041, clear bit 6, bank write
     * SKIPPED: Bank-switched PHY writes (SFR 0x93=1) kill USB3 SuperSpeed */

    /* Stock 0xD9BD-0xD9D2: Bank read/write 0x1507 — set bits 2 and 1
     * SKIPPED: Bank-switched PHY writes (SFR 0x93=1) kill USB3 SuperSpeed */

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

    /* E741/E742 PHY PLL programming (bank 1 at 0x8E31-0x8E4F)
     * Stock firmware does READ-MODIFY-WRITE on both registers:
     *   E741 = (E741 & 0xF8) | 0x03 → (E741 & 0xC7) | 0x28 → (E741 & 0x3F) | 0x80
     *   E742 = (E742 & 0xFC) | 0x03
     * CRITICAL: E742 must preserve upper bits (0x14 from hw_init's 0x17).
     * Force-writing 0x03 wipes bits 2,4 which are needed for PLL lock. */
    REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0xF8) | 0x03;      /* E741: set lower 3 bits to 3 */
    REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0xC7) | 0x28;      /* E741: set bits 5:3 to 5 → 0x2B */
    REG_PHY_PLL_CFG  = (REG_PHY_PLL_CFG  & 0xFC) | 0x03;      /* E742: set lower 2 bits to 3, PRESERVE upper bits */
    REG_PHY_PLL_CTRL = (REG_PHY_PLL_CTRL & 0x3F) | 0x80;      /* E741: set bit 7 → 0xAB */

    /* CC43 = 0x80 — CPU clock config (stock: 0x8E5E) */
    REG_CPU_CLK_CFG = 0x80;

    /* Stock power/GPIO init at 0x5284-0x52A6 (after pcie_early_init):
     * Read-modify-write on C65B, C656, C62D */
    REG_PHY_EXT_5B = (REG_PHY_EXT_5B & 0xF7) | 0x08;    /* C65B: clear bit 3, set bit 3 */
    REG_HDDPC_CTRL = REG_HDDPC_CTRL & 0xDF;              /* C656: clear bit 5 */
    REG_PHY_EXT_5B = (REG_PHY_EXT_5B & 0xDF) | 0x20;    /* C65B: set bit 5 */
    REG_PHY_EXT_2D = (REG_PHY_EXT_2D & 0xE0) | 0x07;  /* C62D: set bits 0,1,2 */

    /* Stock 0xE598-0xE5B0: C004/C007/CA2E controller bus config */
    REG_UART_IIR = (REG_UART_IIR & 0xFD) | 0x02;  /* C004: set bit 1 */
    REG_UART_LCR = REG_UART_LCR & 0xF7;            /* C007: clear bit 3 */
    REG_CPU_CTRL_CA2E = (REG_CPU_CTRL_CA2E & 0xFE) | 0x01;  /* CA2E: set bit 0 */

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
    REG_CPU_EXEC_STATUS = 0x00;
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

    G_PCIE_PHY_STATE = 0x01;  /* step 1: post-EQ regs */

    /* Post-SerDes-EQ register setup (stock: 0x91CF-0x923F)
     * SKIP all PLL/PHY registers here — they are part of the stock PLL init
     * sequence that requires CC43=0x80 (full PLL reset) to be safe.
     * Writing them without CC43 reset kills USB3 SuperSpeed.
     * The stock firmware does these during initial boot before USB is up,
     * or with a full CC43 PLL reset around them. We can't do either.
     *
     * Registers skipped from stock pcie_pll_init (0x8E28):
     *   CC35 &= 0xFE, CC35 &= 0xFB, C65A &= 0xF7, 905F &= 0xEF
     *   C65B bits, C62D, C004 (KILLS USB3!), C007, CA2E, CC43 (KILLS USB3!)
     *
     * E710 re-init is safe (it's in the PCIe controller space, not USB PHY) */
    REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;

    uart_puts("[PLL-]\n");
    G_PCIE_PHY_STATE = 0x02;  /* step 2: PLL done, timers */
    /* Stock DE16→DE24: Timer block init (runs before BFE0 wrapper)
     * Clears globals 0x0B30-0x0B33, then initializes CD30/CD32/CD33 timer
     * and CC2A timer mode. Stock before-training dump: CD30=0x15, CC2A=0x04.
     * This is called from the state machine dispatch at E96C → DE16.
     *
     * DE16: 0x0B30-0x0B33 = 0, lcall E726 (CD31 clear), then DE24:
     *   CD30 = (CD30 & 0xF8) | 0x05, CD32 = 0x00, CD33 = 0xC7
     *   CC2A = (CC2A & 0xF8) | 0x04, CC2C = 0xC7, CC2D = 0xC7
     */
    G_PCIE_BUS_NUM_0B30 = 0; G_PCIE_DEV_NUM_0B31 = 0;
    G_PCIE_FN_NUM_0B32 = 0; G_PCIE_CFG_OFFSET_0B33 = 0;
    REG_CPU_TIMER_CTRL_CD31 = 0x04;  /* Timer clear (E726) */
    REG_CPU_TIMER_CTRL_CD31 = 0x02;
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
    REG_POWER_CTRL_92C8 = REG_POWER_CTRL_92C8 & 0xFC;

    /* CD31 timer 4 clear */
    REG_CPU_TIMER_CTRL_CD31 = 0x04;
    REG_CPU_TIMER_CTRL_CD31 = 0x02;

    /* D47F: Timer 1 init (for 0x0AE5=0 path) */
    REG_TIMER1_CSR = 0x04;  /* timer 1 clear */
    REG_TIMER1_CSR = 0x02;
    REG_TIMER1_DIV = (REG_TIMER1_DIV & 0xF8) | 0x04;  /* mode 4 */
    REG_TIMER1_THRESHOLD_HI = 0x01;  /* threshold hi */
    REG_TIMER1_THRESHOLD_LO = 0x90;  /* threshold lo */
    /* 92C4 clear bit 0, 9201 set/clear bit 0 (when 0x0AE5=0) */
    REG_POWER_MISC_CTRL = REG_POWER_MISC_CTRL & 0xFE;
    REG_USB_CTRL_9201 = (REG_USB_CTRL_9201 & 0xFE) | 0x01;
    REG_USB_CTRL_9201 = REG_USB_CTRL_9201 & 0xFE;

    /* D559: Timer 3 init (for 0x0AE9=0 path) */
    REG_TIMER3_CSR = 0x04;  /* timer 3 clear */
    REG_TIMER3_CSR = 0x02;
    REG_TIMER3_DIV = (REG_TIMER3_DIV & 0xE8) | 0x07;  /* clear bit 4, mode 7 */

    /* E19E: Timer 2 + Timer 5 init */
    REG_TIMER2_CSR = 0x04;  /* timer 2 clear */
    REG_TIMER2_CSR = 0x02;
    REG_TIMER4_CSR = 0x04;  /* timer 5 clear */
    REG_TIMER4_CSR = 0x02;
    REG_TIMER2_DIV = (REG_TIMER2_DIV & 0xF8) | 0x06;  /* timer 2 mode 6 */
    REG_TIMER2_THRESHOLD_LO = 0x00;  /* timer 2 threshold hi */
    REG_TIMER2_THRESHOLD_HI = 0x8B;  /* timer 2 threshold lo */
    REG_TIMER4_DIV = (REG_TIMER4_DIV & 0xF8) | 0x04;  /* timer 5 mode 4 */
    REG_TIMER4_THRESHOLD_LO = 0x00;  /* timer 5 threshold hi */
    REG_TIMER4_THRESHOLD_HI = 0xC7;  /* timer 5 threshold lo */

    G_PCIE_PHY_STATE = 0x03;  /* step 3: GPIO/power config */
    /* Stock 0xC370-0xC393: GPIO/power config BEFORE tunnel setup.
     * Emulator trace cycle ~73946: C655, C620, C65A writes.
     * C655: bit 0 set based on link type (for r7!=1, set bit 0)
     * C620: clear bits 0-4
     * C65A: set bit 0 */
    REG_PHY_CFG_C655 = (REG_PHY_CFG_C655 & 0xFE) | 0x01;
    REG_GPIO_CTRL_0 = REG_GPIO_CTRL_0 & 0xE0;
    REG_PHY_CFG_C65A = (REG_PHY_CFG_C65A & 0xFE) | 0x01;

    /* Step 1: Enable 3.3V power (stock: lcall 0xE5CB) */
    pcie_power_enable();

    G_PCIE_PHY_STATE = 0x04;  /* step 4: tunnel_setup */
    /* Stock first tunnel_setup (stock: lcall 0xCD6C at ~C00D)
     * Emulator trace cycle ~74060: CA06, B4xx adapter config, B401/B482/B480 */
    pcie_tunnel_setup();

    G_PCIE_PHY_STATE = 0x05;  /* step 5: DMA config */
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
    REG_CPU_LINK_CEF3 = 0x08;
    REG_CPU_LINK_CEF2 = 0x80;
    REG_CPU_LINK_CEF0 = REG_CPU_LINK_CEF0 & 0xF7;
    REG_CPU_LINK_CEEF = REG_CPU_LINK_CEEF & 0x7F;

    /* Stock D167-D178: C807 and B281 read-modify-write
     * C807 = (C807 & 0xFB) | 0x04 — set bit 2
     * B281 = (B281 & 0xCF) | 0x10 — set bit 4, clear bit 5 */
    REG_INT_DMA_CTRL = (REG_INT_DMA_CTRL & 0xFB) | 0x04;
    REG_PCIE_DMA_CTRL_B281 = (REG_PCIE_DMA_CTRL_B281 & 0xCF) | 0x10;

    G_PCIE_PHY_STATE = 0x06;  /* step 6: B401 pulse */
    /* Step 2: B401 pulse (stock: 0xC02C-0xC035)
     * lcall 0x99E4 with DPTR=B401 → B401 |= 0x01
     * then reads B401, clears bit 0 → B401 &= 0xFE */
    REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;
    tmp = REG_PCIE_TUNNEL_CTRL;
    REG_PCIE_TUNNEL_CTRL = tmp & 0xFE;

    G_PCIE_PHY_STATE = 0x07;  /* step 7: tunnel_setup_reinit */
    /* Step 3: Second tunnel setup — full CD8F path.
     * Stock CD8F does CE3D (link controller re-init) + C233 config
     * + E712 polling + D9A4 (PHY lane config).
     * This is critical for proper LTSSM state and downstream link training. */
    pcie_tunnel_setup_reinit();

    G_PCIE_PHY_STATE = 0x08;  /* step 8: CA06/B480 */
    /* Step 4: CA06 &= 0xEF, then B480 |= 0x01 (stock: 0xC039-0xC03F via lcall 0x99E0)
     * tunnel_setup already did CA06 &= 0xEF, this is the second time (no-op)
     * but B480 |= 1 is important */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;

    G_PCIE_PHY_STATE = 0x09;  /* step 9: 12V OFF */
    /* Step 5: C659 &= 0xFE — 12V OFF during tunnel config (stock: 0xC042-0xC044)
     * Stock firmware turns 12V OFF here, then back ON in phase 2.
     * The power cycle forces the GPU to re-initialize its PCIe endpoint
     * for Gen3 x2 link training. Without this, the link trains but collapses. */
    REG_PCIE_LANE_CTRL_C659 = REG_PCIE_LANE_CTRL_C659 & 0xFE;

    G_PCIE_PHY_STATE = 0x0A;  /* step A: lane config */
    /* Step 6: Lane config (stock: lcall 0xD436 with r7=0x0F) */
    pcie_lane_config();

    /* Step 7: Set phase 2 pending (stock sets XDATA[0x05B4] = 0x10) */
    pcie_phase2_pending = 1;
    pcie_initialized = 1;
    G_PCIE_LTSSM_STATE = 0xA0;  /* Tunnel enable complete, phase 2 pending */
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
    G_PCIE_LTSSM_STATE = 0x01;  /* Phase 2 started */

    /* Re-ensure E710 and CC30 are correct before phase 2.
     * SS_FAIL events during USB enumeration may have corrupted E710 to 0x1F
     * and cleared CC30. Both are critical for Gen3 link negotiation. */
    REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x04;
    REG_CPU_MODE = (REG_CPU_MODE & 0xFE) | 0x01;
    /* Diag: save CC30 right after write to see if it sticks */
    G_PCIE_TLP_HEADER_0 = REG_CPU_MODE;

    /* Step 1: Timer wait ~300ms (stock: r5=0x2B, r4=0x01, mode=4).
     * Stock uses CC11/CC12/CC13 timer, we use Timer0. */
    timer_wait(0x01, 0x2B, 0x04);

    G_PCIE_TLP_HEADER_1 = REG_CPU_MODE;  /* CC30 after 300ms timer */
    G_PCIE_LTSSM_STATE = 0x02;  /* After timer 1 */

    /* Step 2: A840 function — full implementation for 0x0AEC=0, 0x0AED=0
     * Stock: CA81 clear, CA06 config, B403 set, bank_write 0x40B0,
     *   B431 = (B431 & 0xF0) | 0x0E, then pcie_lane_config(mask=0x0E) */
    /* Stock ACDF lane reconfig — match stock firmware's register state exactly.
     * Stock firmware with 0x707B=0xFF has 0x0AEB=3, 0x0AEC=3, 0x09FA=0x04.
     * D1C9 returns R7=3 during phase 2 (USB3 SS active), lookup tables give:
     *   5D83[3]=0x02 → R7=2, 5D88[3]=0x01 → R6=1
     * With R7=2, R6=1: CA06 = (CA06 & 0x1F) | 0x20, lane mask = 0x0C
     * After ACDF: CA06=0x21, B403 bit 0 set, B434=0x0C, B431=0x0C. */
    REG_CPU_CTRL_CA81 = REG_CPU_CTRL_CA81 & 0xFE;
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x20;  /* CA06: set bit 5 (Gen3 mode) */
    REG_TUNNEL_CTRL_B403 = (REG_TUNNEL_CTRL_B403 & 0xFE) | 0x01;

    /* Stock bank_write 0x40B0 — set lower nibble to 0x03
     * SKIPPED: Bank-switched PHY writes (SFR 0x93=1) kill USB3 SuperSpeed */

    /* Lane mask from stock firmware: 0x0C (lanes 2,3 = Gen3 x2) */
    G_EP_CFG_0A5C = 0x0C;
    REG_TUNNEL_LINK_STATUS = (REG_TUNNEL_LINK_STATUS & 0xF0) | 0x0C;

    G_PCIE_TLP_HEADER_2 = REG_CPU_MODE;  /* CC30 before lane config */
    uart_puts("[LC "); uart_puthex(REG_PCIE_CPL_STATUS); uart_puts("]\n");
    pcie_lane_config_mask(0x0C);
    G_PCIE_TLP_HEADER_3 = REG_CPU_MODE;  /* CC30 after lane config */
    uart_puts("[LC2 "); uart_puthex(REG_PCIE_CPL_STATUS);
    uart_puts(" E="); uart_puthex(REG_PHY_RXPLL_STATUS);
    uart_puts(" B4="); uart_puthex(REG_PCIE_LINK_STATE);
    uart_puts("]\n");

    /* LTSSM equalization setup */
    {
        uint8_t saved_e710, saved_ca06_upper;
        uint8_t tmp;

        saved_e710 = REG_LINK_WIDTH_E710 & 0x1F;
        REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | 0x1F;
        saved_ca06_upper = REG_CPU_MODE_NEXT & 0xE0;
        REG_PHY_POLL_E751 = 0x01;
        uart_puts("[EQ1]\n");

        /* E764 training trigger */
        tmp = REG_PHY_TIMER_CTRL_E764;
        tmp = (tmp & 0xF7) | 0x08;
        REG_PHY_TIMER_CTRL_E764 = tmp;
        tmp = REG_PHY_TIMER_CTRL_E764;
        tmp = tmp & 0xFB;
        REG_PHY_TIMER_CTRL_E764 = tmp;
        tmp = REG_PHY_TIMER_CTRL_E764;
        tmp = tmp & 0xFE;
        REG_PHY_TIMER_CTRL_E764 = tmp;
        tmp = REG_PHY_TIMER_CTRL_E764;
        tmp = (tmp & 0xFD) | 0x02;
        REG_PHY_TIMER_CTRL_E764 = tmp;

        G_PCIE_LTSSM_STATE = 0x03;

        /* 2s link training wait */
        timer_wait(0x07, 0xCF, 0x01);

        tmp = REG_PHY_RXPLL_STATUS;
        G_PCIE_BRIDGE_BUS_PRI = tmp;
        G_PCIE_BRIDGE_BUS_SEC = REG_PCIE_LTSSM_STATE;
        G_PCIE_BRIDGE_BUS_SUB = REG_PHY_TIMER_CTRL_E764;
        G_PCIE_LINK_OK = (tmp & 0x10) ? 0x01 : 0x00;
        G_PCIE_GPU_VID_LO = REG_PCIE_CPL_STATUS;
        G_PCIE_GPU_VID_HI = REG_SYS_CTRL_E765;
        G_PCIE_GPU_DID_LO = REG_LINK_WIDTH_E710;

        if (tmp & 0x10) {
            G_PCIE_PHY_E762 = REG_PCIE_CPL_STATUS;
            G_PCIE_PHY_E765 = REG_PCIE_LTSSM_STATE;
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = (tmp & 0xFE) | 0x01;
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = tmp & 0xFD;
            G_PCIE_PHY_E764 = REG_PCIE_CPL_STATUS;
            G_PCIE_PHY_FLAGS = REG_PCIE_LTSSM_STATE;
            G_STATE_FLAG_06E6 = 0x00;
            uart_puts("[LK+]\n");
        } else {
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = tmp & 0xF7;
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = tmp & 0xFB;
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = tmp & 0xFE;
            tmp = REG_PHY_TIMER_CTRL_E764;
            REG_PHY_TIMER_CTRL_E764 = tmp & 0xFD;
            uart_puts("[LK-]\n");
        }

        /* Restore */
        REG_LINK_WIDTH_E710 = (REG_LINK_WIDTH_E710 & 0xE0) | saved_e710;
        REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | saved_ca06_upper;
        REG_CPU_CTRL_CA81 = REG_CPU_CTRL_CA81 & 0xFE;
    }

    /* Step 5: C659 |= 0x01 — 12V ON (stock: 0xE8D9 with r7=1 → lcall 0xCC8B)
     * CC8B: read, clear bit 0, set bit 0, write (net: set bit 0)
     *
     * NOTE: In stock firmware, 12V was turned OFF during tunnel_enable and
     * turned back ON here. But we keep 12V on from boot to allow GPU to
     * initialize. Since C659 bit 0 is already set, this is a no-op. */
    REG_PCIE_LANE_CTRL_C659 = (REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01;

    /* Step 6: Timer wait ~1s settling (stock: r5=0xE7, r4=0x03, mode=4)
     * Stock waits ~1s for link to fully stabilize in L0. */
    timer_wait(0x03, 0xE7, 0x04);

    /* Step 7: Clear phase 2 pending (stock: XDATA[0x05B4] = 0) */
    pcie_phase2_pending = 0;
    pcie_initialized = 2;

    /* Post-settling diagnostics */
    G_PCIE_GPU_DID_HI = REG_PCIE_CPL_STATUS;         /* B22B after settling */
    G_PCIE_BRIDGE_REG_28 = REG_PCIE_LTSSM_STATE;         /* B450 after settling */
    G_PCIE_BRIDGE_REG_29 = REG_SYS_CTRL_E765;         /* E765 after settling */
    G_PCIE_BRIDGE_REG_2A = REG_PHY_RXPLL_STATUS;         /* E762 after settling */
    G_PCIE_LTSSM_STATE = 0x07;  /* Phase 2 complete */
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
    G_PCIE_BRIDGE_STATE_05AB = mode;

    /* Step 1: Clear 12 PHY channel registers (stock: BF96-BFA4 loop)
     * 0x99B0 helper: XDATA[0xB210 + i] = 0 for i=0..11 */
    for (i = 0; i < 12; i++) {
        XDATA_REG8(0xB210 + i) = 0x00;
    }

    /* Step 2: Set format type based on mode (stock: BFA5-BFB5)
     * mode=1 (write/set): B210=0x40
     * mode=0 (read/clear): B210=0x00 */
    if (mode & 0x01) {
        REG_PCIE_FMT_TYPE = 0x40;
    } else {
        REG_PCIE_FMT_TYPE = 0x00;
    }

    /* Step 3: B213=0x01 (stock: BFB6-BFBB) */
    REG_PCIE_TLP_CTRL = 0x01;

    /* Step 4: B217=0x0F, B216=0x20 (stock: BFBE calling 0x998D)
     * 0x998D: B217=A (0x0F), B216=0x20 */
    REG_PCIE_BYTE_EN = 0x0F;
    REG_PCIE_TLP_LENGTH = 0x20;

    /* Step 5: Copy 32-bit address from 0x05AC-0x05AF to B218-B21B
     * (stock: BFC1-BFCA, load_dword(0x05AC) then store_dword(B218)) */
    REG_PCIE_ADDR_0 = G_PCIE_ADDR_OFFSET_LO;
    REG_PCIE_ADDR_1 = G_PCIE_ADDR_OFFSET_HI;
    REG_PCIE_ADDR_2 = G_PCIE_DIRECTION;
    REG_PCIE_ADDR_3 = G_PCIE_ADDR_0;

    /* Step 6: Trigger B296 sequence (stock: BFCD calling 0x98FA)
     * B296=0x01, 0x02, 0x04; B254=0x0F */
    REG_PCIE_STATUS = 0x01;
    REG_PCIE_STATUS = 0x02;
    REG_PCIE_STATUS = 0x04;
    REG_PCIE_TRIGGER = 0x0F;

    /* Step 7: Poll B296 bit 2 for completion (stock: BFD0-BFD3 calling 0x9948) */
    {
        uint16_t timeout;
        for (timeout = 10000U; timeout; timeout--) {
            if (REG_PCIE_STATUS & 0x04) break;
        }
    }

    /* Step 8: Acknowledge completion (stock: BFD5 calling 0x99F2)
     * B296 = 0x04 */
    REG_PCIE_STATUS = 0x04;

    /* Step 9: Check result based on mode */
    if (mode & 0x01) {
        /* Write mode: just return success (stock: BFDF-BFE1) */
        return 0;
    }

    /* Read mode: check status registers (stock: BFE2-C00C) */
    {
        uint16_t timeout;
        for (timeout = 10000U; timeout; timeout--) {
            uint8_t b296 = REG_PCIE_STATUS;
            if (b296 & 0x02) {
                /* Bit 1 set: success — check completion data */
                /* Stock: 0x99D1 writes 0x02 to B296, reads B22C */
                REG_PCIE_STATUS = 0x02;
                if (REG_PCIE_CPL_DATA != 0x00) return 0xFF;
                if (REG_PCIE_CPL_DATA_ALT != 0x00) return 0xFF;
                if (REG_PCIE_CPL_STATUS != 0x04) return 0xFF;
                /* Success (stock: C006 calls 0x99BD → reads B22A[7:5]) */
                return 0;
            }
            if (b296 & 0x01) {
                /* Bit 0 set, bit 1 not set: timeout/error */
                REG_PCIE_STATUS = 0x01;
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
    G_PCIE_ADDR_OFFSET_LO = 0x00;
    G_PCIE_ADDR_OFFSET_HI = 0xD0;
    G_PCIE_DIRECTION = 0x00;
    G_PCIE_ADDR_0 = 0x1C;

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
        uint8_t raw = REG_PCIE_EXT_STATUS;
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
    REG_PCIE_FMT_TYPE = 0x44;

    /* Step 3: B213 = 0x01 (stock: 0xAC3C) */
    REG_PCIE_TLP_CTRL = 0x01;

    /* Step 4: B217 = IDATA[0x65] & 0x0F = 0x03 (stock: 0xAC44) */
    REG_PCIE_BYTE_EN = 0x03;

    /* Step 5: B218-B219 = IDATA[0x61:0x62] = {0,0} (stock: 0xAC48-0xAC50) */
    REG_PCIE_ADDR_0 = 0x00;
    REG_PCIE_ADDR_1 = 0x00;

    /* Step 6: B21A-B21B computed from IDATA[0x63:0x64] = {0, 1}
     * Stock: AC51-AC7D computes:
     *   r4 = IDATA[0x63] = 0, r5 = IDATA[0x64] = 1
     *   r6 = r4 & 0x03 = 0, a = r5 & 0xC0 = 0
     *   shift right 6: r7 = 0
     *   B21A = (B21A & 0xF0) | r7 = 0 | 0 = 0
     *   r7 = (r5 & 0x3F) * 4 = (1 & 0x3F) * 4 = 4
     *   B21B = (B21B & 0x03) | r7 = 0 | 4 = 4
     * Then calls 0x9990: writes B21B result, then B216=0x20 */
    REG_PCIE_ADDR_2 = (REG_PCIE_ADDR_2 & 0xF0) | 0x00;
    REG_PCIE_ADDR_3 = (REG_PCIE_ADDR_3 & 0x03) | 0x04;

    /* Step 7: B216 = 0x20 (trigger, stock: 0x9990 → 0x9991) */
    REG_PCIE_TLP_LENGTH = 0x20;

    /* Step 8: B296 trigger sequence (stock: 0x98FA)
     * B296=0x01 (reset), B296=0x02 (start), B296=0x04 (trigger), B254=0x0F (mask) */
    REG_PCIE_STATUS = 0x01;
    REG_PCIE_STATUS = 0x02;
    REG_PCIE_STATUS = 0x04;
    REG_PCIE_TRIGGER = 0x0F;

    uart_puts("[LT1]\n");

    /* Step 9: Poll B296 bit 2 for completion (stock: 0xAC83 calling 0x9948)
     * 0x9948: reads B296 & 0x04, returns non-zero when bit 2 set */
    for (timeout = 10000U; timeout; timeout--) {
        if (REG_PCIE_STATUS & 0x04) break;
    }

    /* Step 10: Acknowledge completion (stock: 0xAC88 calling 0x99F2)
     * B296 = 0x04 */
    REG_PCIE_STATUS = 0x04;

    /* Step 11: Check result (stock: 0xAC8B-0xACDC)
     * Poll B296 for bit 1 (success) or bit 0 (failure) */
    {
        uint8_t b296;
        for (timeout = 10000U; timeout; timeout--) {
            b296 = REG_PCIE_STATUS;
            if (b296 & 0x02) break;  /* Success bit */
            if (b296 & 0x01) break;  /* Failure bit */
        }

        uart_puts("[B296="); uart_puthex(b296);
        uart_puts(" B22B="); uart_puthex(REG_PCIE_CPL_STATUS);
        uart_puts(" B284="); uart_puthex(REG_PCIE_COMPL_STATUS);
        uart_puts("]\n");

        if (b296 & 0x02) {
            /* Stock: 0x99D1 writes 0x02 to B296, reads B22C */
            REG_PCIE_STATUS = 0x02;
            /* Verify completion: B22C==0, B22D==0 (stock: ACAF-ACB6) */
            if (REG_PCIE_CPL_DATA != 0x00) { uart_puts("[LTe1]\n"); return 0; }
            if (REG_PCIE_CPL_DATA_ALT != 0x00) { uart_puts("[LTe2]\n"); return 0; }
            /* Check B22B == 0x04 (link width x4, stock: ACB8-ACBE) */
            if (REG_PCIE_CPL_STATUS != 0x04) { uart_puts("[LTe3]\n"); return 0; }
            /* Check B284 bit 0 (stock: ACC6-ACCA, for IDATA[0x60]=1 path: skip B284) */
            uart_puts("[LT+]\n");
            return 1;  /* Link trained successfully */
        }
        /* Timeout/failure: ack bit 0 and return 0 */
        if (b296 & 0x01) {
            REG_PCIE_STATUS = 0x01;
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
    REG_PCIE_LTSSM_B455 = 0x02;  /* Clear link detect status */
    REG_PCIE_LTSSM_B455 = 0x04;  /* Trigger status clear */
    REG_PCIE_CTRL_B2D5 = 0x01;  /* PCIe config enable */
    REG_PCIE_STATUS = 0x08;  /* PCIe trigger reset */

    /* Step 2: Set config space address to 0x00D00014 (stock: 0x361E, E4C8(0x14))
     * E4C8 with r7=0x14: stores {r4=0x00, r5=0xD0, r6=0x00, r7=0x14} to 0x05AC */
    G_PCIE_ADDR_OFFSET_LO = 0x00;
    G_PCIE_ADDR_OFFSET_HI = 0xD0;
    G_PCIE_DIRECTION = 0x00;
    G_PCIE_ADDR_0 = 0x14;

    /* Step 3: Write B220 TLP config (stock: 0x3623-0x362E)
     * Stock: r7=0x01, r6=0x40, r5=0x46, r4=0x00
     * store_dword(B220, r4:r5:r6:r7) → B220=r4, B221=r5, B222=r6, B223=r7 */
    REG_PCIE_DATA = 0x00;  /* r4 */
    REG_PCIE_DATA_1 = 0x46;  /* r5 */
    REG_PCIE_DATA_2 = 0x40;  /* r6 */
    REG_PCIE_EXT_STATUS = 0x01;  /* r7 */

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
    for (timeout = 5; timeout; timeout--) {
        /* Timer wait ~200ms per iteration (stock: 0x3634, r5=0xC7, r4=0x00, mode=4)
         * Stock firmware polls indefinitely with a timer-based timeout.
         * We use 5 iterations (~1 second) to balance speed vs reliability. */
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
    REG_PCIE_DATA = 0x00;
    REG_PCIE_DATA_1 = 0x00;
    REG_PCIE_DATA_2 = 0x00;
    REG_PCIE_EXT_STATUS = 0x0B;

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
        if (REG_PCIE_LTSSM_B455 & 0x02) break;
    }
    if (timeout == 0) {
        uart_puts("[LD-]\n");
    } else {
        /* Step 10: Acknowledge link detect (stock: 0x3675-0x367A) */
        REG_PCIE_LTSSM_B455 = 0x02;
        uart_puts("[LD+]\n");
    }

    /* Post-link-detect config (stock: 0x3689-0x36AD)
     * Reduced from ~300ms to ~50ms to minimize USB blocking */
    timer_wait(0x00, 0x32, 0x04);
    /* B2D5 read-back and B296 (stock: 0x3697-0x36AD) */
    if (REG_PCIE_CTRL_B2D5 & 0x01) {
        REG_PCIE_CTRL_B2D5 = 0x01;
    }
    REG_PCIE_STATUS = 0x08;

    return 1;
}

/*
 * pcie_bridge_config_init - Set up PCIe bridge config space shadow registers
 * Based on stock firmware 0xCC83-0xCCDA (called from 0xC275 post-link)
 *
 * The ASM2464PD has an internal PCIe switch with two downstream ports.
 * The B4xx registers are shadow config space for these ports. Without
 * the class code set to PCI-to-PCI bridge (0x060400), the hardware
 * won't forward Type 1 config TLPs downstream, causing all config
 * reads to return the bridge's own VID/DID regardless of bus number.
 *
 * Stock firmware flow (0xCC83):
 *   1. CA06 &= ~0x10
 *   2. Call 0xC6D7: write VID/DID/class/subsystem to B410-B42B
 *   3. B401 |= 0x01, B482 |= 0x01
 *   4. B482 = (B482 & 0x0F) | 0xF0
 *   5. B401 &= ~0x01, B480 |= 0x01  (bridge enable)
 *   6. B430 &= ~0x01
 *   7. B298 = (B298 & ~0x10) | 0x10
 *
 * Caller (0xC275) also does:
 *   8. CA06 &= ~0x10, B480 |= 0x01  (again)
 *   9. Call pcie_lane_config_mask(0x0F)
 */
static void pcie_bridge_config_init(void) {
    /* Step 1: CA06 clear bit 4 (stock: 0xCC83-CC89) */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;

    /* Step 2: Bridge config shadow registers (stock: 0xC6D7-C73D)
     * Port 0 shadow: B410-B41B
     * Port 1 shadow: B420-B42B
     * Values from stock dump — VID=0x1B21, DID=0x2463, class=0x060400 */

    /* Port 0: VID/DID */
    REG_TUNNEL_CFG_A_LO = 0x1B;  /* VID low */
    REG_TUNNEL_CFG_A_HI = 0x21;  /* VID high */
    REG_TUNNEL_CREDITS = 0x24;  /* DID low */
    REG_TUNNEL_CFG_MODE = 0x63;  /* DID high */
    /* Port 0: Class code = PCI-to-PCI bridge (0x060400) */
    REG_TUNNEL_CAP_0 = 0x06;  /* base class: bridge */
    REG_TUNNEL_CAP_1 = 0x04;  /* sub class: PCI-to-PCI */
    REG_TUNNEL_CAP_2 = 0x00;  /* prog interface */
    /* Port 0: Subsystem IDs */
    REG_TUNNEL_PATH_CREDITS = 0x24;  /* subsystem DID low */
    REG_TUNNEL_PATH_MODE = 0x63;  /* subsystem DID high */
    REG_TUNNEL_LINK_CFG_LO = 0x1B;  /* subsystem VID low */
    REG_TUNNEL_LINK_CFG_HI = 0x21;  /* subsystem VID high */

    /* Port 1: same values (stock: 0xC6E9-C73C) */
    REG_TUNNEL_DATA_LO = 0x1B;
    REG_TUNNEL_DATA_HI = 0x21;
    REG_TUNNEL_STATUS_0 = 0x24;
    REG_TUNNEL_STATUS_1 = 0x63;
    REG_TUNNEL_CAP2_0 = 0x06;
    REG_TUNNEL_CAP2_1 = 0x04;
    REG_TUNNEL_CAP2_2 = 0x00;
    REG_TUNNEL_PATH2_CRED = 0x24;
    REG_TUNNEL_PATH2_MODE = 0x63;
    REG_TUNNEL_AUX_CFG_LO = 0x1B;
    REG_TUNNEL_AUX_CFG_HI = 0x21;

    /* Step 3: B401 set bit 0, B482 set bit 0 (stock: 0xCC9D-CCA6) */
    REG_PCIE_TUNNEL_CTRL = (REG_PCIE_TUNNEL_CTRL & 0xFE) | 0x01;
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0xFE) | 0x01;

    /* Step 4: B482 upper nibble = 0xF0 (stock: 0xCCA9-CCAE) */
    REG_TUNNEL_ADAPTER_MODE = (REG_TUNNEL_ADAPTER_MODE & 0x0F) | 0xF0;

    /* Step 5: B401 clear bit 0, then B480 set bit 0 = bridge enable
     * (stock: 0xCCAF-CCB7 calling 0x993D) */
    REG_PCIE_TUNNEL_CTRL = REG_PCIE_TUNNEL_CTRL & 0xFE;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;

    /* Step 6: B430 clear bit 0 (stock: 0xCCB8-CCBE) */
    REG_TUNNEL_LINK_STATE = REG_TUNNEL_LINK_STATE & 0xFE;

    /* Step 7: B298 set bit 4 (stock: 0xCCBF-CCC7) */
    REG_PCIE_TUNNEL_CFG = (REG_PCIE_TUNNEL_CFG & 0xEF) | 0x10;

    /* Step 8: Caller (0xC278-C27E): CA06 &= ~0x10 again, B480 |= 0x01 again */
    REG_CPU_MODE_NEXT = REG_CPU_MODE_NEXT & 0xEF;
    REG_PCIE_PERST_CTRL = (REG_PCIE_PERST_CTRL & 0xFE) | 0x01;

    /* Step 9: Bank-switched PHY writes (stock: 0xCC8D-CC9C via bank helpers)
     * These configure PCIe switch port PHY and are required for type 1 TLP forwarding.
     * R3=2 → bank 1 (SFR 0x93 = 1). Done AFTER link reaches L0 (stock: CC83 called from C24C). */
    bank1_write(0x4084, 0x22);
    bank1_write(0x5084, 0x22);

    /* Step 10: B401 set bit 0, B482 set bit 0 — already done in step 3 */
    /* Step 11: B482 upper nibble, B401 clear, B480 set — already done in steps 4-5 */
    /* Step 12: B430, B298 — already done in steps 6-7 */

    /* Step 13: More bank-switched PHY writes (stock: 0xCCC8-CCDA)
     * 0x6043 = 0x70, 0x6025 |= 0x80 */
    bank1_write(0x6043, 0x70);
    bank1_or_bits(0x6025, 0x80);

    /* Step 14: B481 bits (stock dump shows B481=0x03) */
    REG_PCIE_LINK_CTRL_B481 = (REG_PCIE_LINK_CTRL_B481 & 0xFC) | 0x03;
}

static void pcie_post_train(void) {
    uint8_t i;
    /* Called once after LTSSM reaches L0 */
    /* Ensure 12V is ON (in case phase 2 was skipped) */
    REG_PCIE_LANE_CTRL_C659 = (REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01;
    REG_PCIE_DMA_SIZE_A = 0x08;  REG_PCIE_DMA_SIZE_B = 0x00;
    REG_PCIE_DMA_SIZE_C = 0x08;  REG_PCIE_DMA_SIZE_D = 0x08;
    REG_PCIE_DMA_BUF_A = 0x08;  REG_PCIE_DMA_BUF_B = 0x20;
    REG_PCIE_DMA_BUF_C = 0x08;  REG_PCIE_DMA_BUF_D = 0x28;
    REG_PCIE_DMA_CFG_50 = 0x00;
    REG_PCIE_DOORBELL_CMD = 0x00;
    REG_NVME_QUEUE_CFG |= 0x20;
    REG_NVME_CMD_STATUS_50 |= 0x04;
    REG_NVME_LINK_CTRL &= 0xFE;
    REG_NVME_PARAM_C4EB |= 0x01;
    REG_NVME_DMA_CTRL_ED |= 0x01;
    REG_CPU_MODE_NEXT &= 0xBF;

    /* Bridge config init (stock: CC83 called from C24C after link L0) */
    pcie_bridge_config_init();

    /* B455 trigger sequence (stock: 0x35E2-0x367A)
     * This enables PCIe config TLP forwarding through the internal switch.
     * B455 is a hardware control register:
     *   Write 0x02: clear/ack pending status
     *   Write 0x04: trigger downstream port enable
     *   Bit 1 set by hardware: operation complete
     * B2D5 = 0x01: enable PCIe config routing */
    REG_PCIE_LTSSM_B455 = 0x02;   /* Clear pending */
    REG_PCIE_LTSSM_B455 = 0x04;   /* Trigger */
    REG_PCIE_CTRL_B2D5 = 0x01;   /* Enable config routing */
    REG_PCIE_STATUS = 0x08;   /* Ack */

    /* Poll B455 bit 1 for completion (stock: 0x366B-0x3673) */
    for (i = 0; i < 200; i++) {
        if (REG_PCIE_LTSSM_B455 & 0x02) {
            REG_PCIE_LTSSM_B455 = 0x02;  /* Ack completion */
            break;
        }
        delay_short();
    }
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
    pcie_initialized = 0; need_pcie_init = 0; config_done = 0; need_rearm = 0; need_state_init = 0; pcie_cfg_pending = 0; cbw_active = 0;
    /* Clear diagnostic area */
    { uint8_t di; for (di = 0; di < 32; di++) XDATA_REG8(0x0F00 + di) = 0x00; }
    REG_UART_LCR &= 0xF7;
    uart_puts("\n[BOOT]\n");

    hw_init();

    /* Stock firmware sets CA06 bits 5,6 and CA81 bit 0 during early init
     * (pcie_config_init_a3f5 at instruction ~26559 in trace).
     * CA06=0x61 is required for PCIe link training to work.
     * E716 must also be 0x03 (bits 0,1) - USB set_address may overwrite it. */
    REG_CPU_MODE_NEXT = (REG_CPU_MODE_NEXT & 0x1F) | 0x60;  /* CA06: set bits 5,6 */
    REG_CPU_CTRL_CA81 = REG_CPU_CTRL_CA81 | 0x01;         /* CA81: set bit 0 */
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
    G_PCIE_PHY_E762 = REG_PHY_RXPLL_STATUS;
    G_PCIE_PHY_E765 = REG_SYS_CTRL_E765;
    G_PCIE_PHY_E764 = REG_PHY_TIMER_CTRL_E764;

    tur_count = 0;
    pcie_phase2_pending = 0;

    {
        uint8_t link = REG_USB_LINK_STATUS;
        is_usb3 = (link >= USB_SPEED_SUPER) ? 1 : 0;
        uart_puts("[link="); uart_puthex(link); uart_puts("]\n");
    }

    /* Power on at boot so GPU can start initializing early.
     * Run FULL PCIe init at boot BEFORE USB enumeration.
     * This ensures PCIe link is trained and bridge is configured before
     * tinygrad sends pcie_cfg_req. Stock firmware also does PCIe init
     * before USB activity. PLL changes in tunnel_enable are safe here
     * because USB PHY hasn't started SuperSpeed negotiation yet. */
    pcie_power_enable();
    REG_PCIE_LANE_CTRL_C659 = (REG_PCIE_LANE_CTRL_C659 & 0xFE) | 0x01;
    uart_puts("[PWR]\n");

    /* Full PCIe init sequence at boot */
    uart_puts("[PCIE_BOOT]\n");
    pcie_tunnel_enable();
    pcie_perst_deassert();
    pcie_phase2();
    {
        uint8_t link_ok = G_PCIE_LINK_OK;
        uint8_t b22b = REG_PCIE_CPL_STATUS;
        if (link_ok || b22b == 0x04) {
            pcie_post_train();
            pcie_initialized = 3;
            G_XFER_CTRL_0AF7 = 0x01;
            uart_puts("[L0!]\n");
        } else {
            pcie_initialized = 2;
            uart_puts("[LF]\n");
        }
    }

    uart_puts("[GO]\n");
    TMOD = 0x01;  /* Timer0 Mode 1 (16-bit), Timer1 Mode 0 */
    TCON = 0x04;  /* IT0=0 (level-triggered INT0) */
    IP = IP_PT0;  /* Timer0 = high priority (can interrupt INT0) */
    IE = IE_EA | IE_EX0 | IE_EX1 | IE_ET0;

    uart_puts("[ML]\n");
    while (1) {
        REG_CPU_KEEPALIVE = 0x0C;
        poll_bulk_events();

        if (need_bulk_init) { need_bulk_init = 0; do_bulk_init(); }
        if (need_cbw_process) {
            need_cbw_process = 0;
            cbw_active = 1;
            handle_cbw();
            cbw_active = 0;
        }

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

        /* Post-CSW cleanup runs here when need_rearm is set. */
        if (need_rearm) {
            need_rearm = 0;
            post_csw_cleanup();
            G_PCIE_DIAG_STATUS = 0xC5;  /* diagnostic: post-CSW cleanup done */
        }

        /* 91D1 events are now handled in ISR (line ~1090) matching stock firmware.
         * No need to poll from main loop. */

        /* NOTE: Do NOT continuously write CC30 in main loop — it interferes
         * with USB operation and prevents enumeration. CC30 is set at end of
         * phase 2 and maintained by the LTSSM hardware. */

        /* PCIe init — run ALL phases as one block before bulk init.
         * This ensures PCIe link is trained before host sends pcie_cfg_req.
         * tinygrad flow: SET_CONFIG → TUR (caught) → sleep(0.5) → E5 writes → pcie_cfg_req
         * PCIe init must complete within the 500ms sleep window.
         * ISR handles CBWs during timer_wait polling, but bulk_ready=0 so
         * CBWs are dropped. tinygrad catches the TUR failure. */
        if (need_pcie_init && pcie_initialized >= 2) {
            need_pcie_init = 0;
        }
        if (need_pcie_init && pcie_initialized == 0) {
            need_pcie_init = 0;
            uart_puts("[PCIE]\n");
            pcie_tunnel_enable();

            /* Init bulk engine AFTER PLL change (in tunnel_enable) but BEFORE
             * the long perst_deassert sequence. This sets bulk_ready=1 so the
             * ISR can process CBWs during PCIe timer_waits.
             * Without this, tinygrad's E5 writes timeout because bulk_ready=0
             * during the ~3s PCIe init window. */
            if (need_state_init) { need_state_init = 0; state_init(); }
            if (need_bulk_init) { need_bulk_init = 0; do_bulk_init(); }

            pcie_perst_deassert();

            /* Phase 2: Gen3 x2 retraining. */
            {
                uint8_t b22b = REG_PCIE_CPL_STATUS;
                uart_puts("[W="); uart_puthex(b22b); uart_puts("]\n");

                pcie_phase2();
                {
                    uint8_t link_ok = G_PCIE_LINK_OK;
                    b22b = REG_PCIE_CPL_STATUS;
                    uart_puts("[W2="); uart_puthex(b22b);
                    uart_puts(" P="); uart_puthex(link_ok);
                    uart_puts(" L="); uart_puthex(REG_PCIE_LTSSM_STATE);
                    uart_puts("]\n");
                    if (link_ok || b22b == 0x04) {
                        pcie_post_train();  /* includes bridge_config_init + B455 trigger */
                        pcie_initialized = 3;
                        /* Stock firmware 0x3914: set PCIE_ENUM_DONE after successful init.
                         * CBW handler at 0x34A3 checks this — without it, CBW processing
                         * takes the error path and doesn't send CSW properly. */
                        G_XFER_CTRL_0AF7 = 0x01;
                        /* Re-trigger any PCIe config request that arrived during init.
                         * tinygrad writes B297=0x01 before PCIe link is up — hardware
                         * can't complete the config access. Now that link is trained,
                         * re-write B297 to restart the config transaction. */
                        if (pcie_cfg_pending) {
                            pcie_cfg_pending = 0;
                            REG_PCIE_BRIDGE_CTRL = 0x01;
                            uart_puts("[B297!]\n");
                        }
                        uart_puts("[L0!]\n");
                    } else {
                        pcie_initialized = 2;
                        uart_puts("[LF]\n");
                    }
                }
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
