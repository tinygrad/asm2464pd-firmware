/*
 * dma.h - DMA Engine Driver
 *
 * DMA engine control for the ASM2464PD USB4/Thunderbolt to NVMe bridge.
 * Handles high-speed data transfers between USB, NVMe, and internal buffers
 * without CPU intervention.
 *
 * ===========================================================================
 * MEMORY ARCHITECTURE
 * ===========================================================================
 *
 * The ASM2464PD has a large internal SRAM (~6-8 MB) that is accessed via
 * two different address spaces:
 *
 *   1. PCI Address Space (32-bit) - What DMA engines and PCIe devices see:
 *      0x00200000  Data buffer start (6+ MB for bulk transfers)
 *      0x00820000  Queue region (NVMe queues, can be repurposed)
 *
 *   2. 8051 XDATA Space (16-bit) - What the CPU sees:
 *      0x7000-0x7FFF  USB bulk OUT landing buffer (read-only to CPU, written by HW)
 *      0x8000-0x8FFF  Writable XDATA (NOT aliased with 0xF000)
 *      0xA000-0xAFFF  Writable XDATA
 *      0xF000-0xFFFF  Writable XDATA (NOT aliased with 0x8000)
 *
 * NOTE: Probing with the handmade firmware shows 0x8000 and 0xF000 are
 * independent memory regions, not aliases of the same SRAM.  The SRAM
 * is only accessible through the CE00 DMA engine, which uses 32-bit PCI
 * addresses (CE76-CE79) and is tightly coupled to the USB MSC state machine.
 * The 8051 CPU cannot directly read/write SRAM beyond what the DMA engine
 * deposits into the XDATA windows.
 *
 *   USB Host                           Internal SRAM (6-8 MB)
 *       │                                    │
 *       │  USB3 bulk packets                 │  PCI Address Space
 *       │  (1024 bytes each)                 │  0x00200000+
 *       ▼                                    ▼
 *   ┌───────────┐                     ┌──────────────┐
 *   │ USB       │   CE00 DMA engine   │  Data Buffer │
 *   │ Controller├────────────────────>│  (6+ MB)     │
 *   │           │   (CE76-79 = addr)  │              │
 *   └─────┬─────┘                     └──────────────┘
 *         │                                 ▲
 *         │ bulk OUT lands at               │ GPU DMA via PCIe
 *         ▼ XDATA 0x7000                    │ bus mastering
 *   ┌──────────────┐                  ┌──────────────┐
 *   │ 0x7000 buf   │                  │  GPU SDMA    │
 *   │ (staging)    │                  │  engine      │
 *   └──────────────┘                  └──────────────┘
 *
 * ===========================================================================
 * DMA ENGINE TYPES - SOURCE/DESTINATION SUMMARY
 * ===========================================================================
 *
 *   | Engine          | Source       | Destination  | Addressing  | Max Size |
 *   |-----------------|--------------|--------------|-------------|----------|
 *   | USB Descriptor  | ROM          | USB HW       | 16-bit ROM  | ~256 B   |
 *   | SCSI/Bulk       | USB/SRAM     | SRAM/USB     | 32-bit PCI  | ~8 MB    |
 *   | PCIe/Vendor     | XDATA        | USB buf      | 24-bit      | 255 B    |
 *   | Flash           | SPI chip     | XDATA        | 24-bit      | 4 KB     |
 *   | Internal        | XDATA        | XDATA        | 16-bit      | 64 KB    |
 *
 * ===========================================================================
 * DMA ENGINE 1: USB Descriptor DMA (0x9092)
 * ===========================================================================
 *   Source:      Code ROM (descriptor tables)
 *   Destination: USB hardware buffer (feeds directly to USB PHY)
 *   Trigger:     Write 0x01 to 0x9092
 *   Completion:  Check 0xE712 bits 0-1
 *   Registers:   0x905B-905C (source addr), 0x9004 (length)
 *   Use:         USB enumeration only, NOT for bulk data
 *
 * ===========================================================================
 * DMA ENGINE 2: SCSI/Bulk DMA (0xCE00-0xCE9F) - THE HIGH-SPEED PATH
 * ===========================================================================
 *   Source:      USB bulk endpoint (data at 0x7000) -> SRAM
 *   Destination: Internal SRAM at 32-bit PCI address
 *   Addressing:  32-bit PCI address (0x00200000+) via CE76-CE79
 *   Trigger:     Write 0x03 to 0xCE00
 *   Completion:  Poll 0xCE00 until 0x00 (returns 0x00 when done)
 *
 *   IMPORTANT: This engine is tightly coupled to the USB MSC state machine.
 *   CE00=0x03 is a no-op unless data was received through the CE88/CE89
 *   bulk handshake protocol (ARM endpoint -> wait for BULK_DATA -> CE88=0x00
 *   -> poll CE89 ready -> poll CE55 != 0 -> THEN CE00=0x03).  In raw bulk
 *   mode (MSC_CFG=0x00), CE00 completes instantly without moving data.
 *
 *   The 8051 CPU never touches bulk data bytes — it only sets the 32-bit
 *   PCI target address in CE76-CE79 and triggers the sector transfer.
 *
 *   Address calculation (from SCSI LBA):
 *     pci_addr = 0x00200000 + (lba * 512)
 *     CE76 = (pci_addr >> 0) & 0xFF   // LSB
 *     CE77 = (pci_addr >> 8) & 0xFF
 *     CE78 = (pci_addr >> 16) & 0xFF
 *     CE79 = (pci_addr >> 24) & 0xFF  // MSB
 *
 * ===========================================================================
 * DMA ENGINE 3: PCIe/Vendor DMA (0xB296)
 * ===========================================================================
 *   Source:      XDATA (any 16-bit address)
 *   Destination: USB buffer at 0x8000
 *   Trigger:     Write 0x08 to 0xB296
 *   Completion:  Check 0xB296 bits 1-2
 *   Registers:   0x910F-9111 (24-bit source addr), 0x910E (size)
 *   Use:         E4/E5 vendor commands (SLOW - max 255 bytes)
 *
 * ===========================================================================
 * DMA ENGINE 4: Flash DMA (0xC8A9)
 * ===========================================================================
 *   Source:      SPI flash (external chip)
 *   Destination: Flash buffer at XDATA 0x7000-0x7FFF
 *   Trigger:     Write 0x01 to 0xC8A9
 *   Completion:  Poll 0xC8A9 bit 0 until clear
 *   Use:         Config/firmware loading at boot
 *
 * ===========================================================================
 * DMA ENGINE 5: Internal Transfer DMA (0xC8B8)
 * ===========================================================================
 *   Source:      XDATA address
 *   Destination: XDATA address
 *   Trigger:     Write 0x01 to 0xC8B8
 *   Completion:  Poll 0xC8D6 bit 2
 *   Use:         Block memory copies within XDATA
 *
 * ===========================================================================
 * SCSI DMA STATE MACHINE (0xCE89)
 * ===========================================================================
 * The SCSI DMA uses a state machine controlled by 0xCE89 bits:
 *
 *   Bit 0 (USB_DMA_STATE_READY):
 *     - SET when DMA engine ready for next transfer
 *     - Firmware polls at 0x348C waiting for this
 *     - CLEAR while DMA in progress
 *
 *   Bit 1 (USB_DMA_STATE_CBW):
 *     - 1=CBW received, 0=bulk data
 *     - Checked at 0x3493 for enumeration path
 *
 *   Bit 2 (USB_DMA_STATE_ERROR):
 *     - DMA error in copy loop (stock: 0x3546)
 *     - Controls state 3→4→5 transitions
 *
 *   State Progression (emulator behavior):
 *     - Reads 1-2: Return 0x00 (busy)
 *     - Reads 3-4: Return 0x01 (ready)
 *     - Reads 5-6: Return 0x03 (ready + success)
 *     - Reads 7+:  Return 0x07 (complete)
 *
 * ===========================================================================
 * SCSI DMA CONTROL (0xCE00)
 * ===========================================================================
 *   Write 0x03 to start DMA transfer (at 0x3531-0x3533)
 *   Poll until value returns 0x00 (transfer complete)
 *   Firmware loops at 0x3534-0x3538 waiting for completion
 *
 * ===========================================================================
 * DMA ENGINE CORE REGISTERS (0xC8B0-0xC8DF)
 * ===========================================================================
 *   0xC8B0  DMA_MODE          DMA mode configuration
 *   0xC8B2  DMA_CHAN_AUX      Channel auxiliary config (2 bytes)
 *   0xC8B4-B5 Transfer count (16-bit)
 *   0xC8B6  DMA_CHAN_CTRL2    Channel control 2
 *                             Bit 0: Start/busy
 *                             Bit 1: Direction (0=read, 1=write)
 *                             Bit 2: Enable
 *                             Bit 7: Active/in-progress
 *   0xC8B7  DMA_CHAN_STATUS2  Channel status 2
 *   0xC8B8  DMA_TRIGGER       Trigger register:
 *                             Write 0x01 to start
 *                             Poll bit 0 until clear (complete)
 *                             Auto-clears after 5 reads (emulator)
 *   0xC8D4  DMA_CONFIG        Global DMA configuration
 *   0xC8D6  DMA_STATUS        DMA status
 *                             Bit 2: Done flag
 *                             Bit 3: Error flag
 *                             Default: 0x04 (done)
 *   0xC8D8  DMA_STATUS2       DMA status 2
 *
 * ===========================================================================
 * SCSI/MASS STORAGE DMA REGISTERS (0xCE40-0xCE9F)
 * ===========================================================================
 *   0xCE00  SCSI_DMA_CTRL     DMA control (write 0x03 to start, poll for 0)
 *   0xCE40  SCSI_DMA_PARAM0   SCSI parameter 0
 *   0xCE41  SCSI_DMA_PARAM1   SCSI parameter 1
 *   0xCE42  SCSI_DMA_PARAM2   SCSI parameter 2
 *   0xCE43  SCSI_DMA_PARAM3   SCSI parameter 3
 *   0xCE55  SCSI_TAG_VALUE    Transfer slot count for loop iterations
 *   0xCE5C  SCSI_DMA_COMPL    Completion status
 *                             Bit 0: Mode 0 complete
 *                             Bit 1: Mode 0x10 complete
 *   0xCE5D  SCSI_DEBUG_MASK   Debug enable mask (0xFF = all enabled)
 *   0xCE66  SCSI_DMA_TAG_CNT  Tag count (5-bit, 0-31)
 *   0xCE67  SCSI_DMA_QUEUE    Queue status (4-bit, 0-15)
 *   0xCE6C  XFER_STATUS_6C    USB controller ready (bit 7 must be SET)
 *   0xCE6E  SCSI_DMA_CTRL2    SCSI DMA control register 2
 *   0xCE75  SCSI_BUF_LEN      Transfer length
 *   0xCE76  SCSI_BUF_ADDR0    32-bit PCI address byte 0 (LSB)
 *   0xCE77  SCSI_BUF_ADDR1    32-bit PCI address byte 1
 *   0xCE78  SCSI_BUF_ADDR2    32-bit PCI address byte 2
 *   0xCE79  SCSI_BUF_ADDR3    32-bit PCI address byte 3 (MSB)
 *                             Example: For LBA=256, pci_addr = 0x00220000
 *                             CE76=0x00, CE77=0x00, CE78=0x22, CE79=0x00
 *   0xCE86  XFER_STATUS       USB status (bit 4 checked at 0x349D)
 *   0xCE88  XFER_CTRL         DMA trigger (write resets CE89 state machine)
 *   0xCE89  USB_DMA_STATE     DMA state machine (see above)
 *                             Bit 0: Ready, Bit 1: Success, Bit 2: Complete
 *   0xCE96  SCSI_DMA_STATUS   DMA status/completion flags
 *
 * ===========================================================================
 * TRANSFER MODES
 * ===========================================================================
 *   DMA_MODE_USB_RX (0x00): USB bulk OUT - host to device
 *   DMA_MODE_USB_TX (0x01): USB bulk IN - device to host
 *   DMA_MODE_SCSI_STATUS (0x03): SCSI status transfer
 *
 * ===========================================================================
 * KEY XDATA GLOBALS
 * ===========================================================================
 *   0x0203  G_DMA_MODE_SELECT    Current DMA mode (0x00/0x01/0x03)
 *   0x020D  G_DMA_PARAM1         Transfer parameter 1
 *   0x020E  G_DMA_PARAM2         Transfer parameter 2
 *   0x021A-1B G_BUF_BASE         Buffer base address (16-bit)
 *   0x0472-73 G_DMA_LOAD_PARAM   Load parameters
 *   0x0564  G_EP_QUEUE_CTRL      Endpoint queue control
 *   0x0565  G_EP_QUEUE_STATUS    Endpoint queue status
 *   0x07E5  G_TRANSFER_ACTIVE    Transfer active flag
 *   0x0AA0  G_DMA_XFER_STATUS    DMA transfer status/size
 *   0x0AA3-A4 G_STATE_COUNTER    16-bit state counter
 *
 * ===========================================================================
 * BUFFER ADDRESS SPACES (XDATA)
 * ===========================================================================
 *   0x0000-0x00FF: Endpoint queue descriptors
 *   0x0100-0x01FF: Transfer work areas
 *   0x0400-0x04FF: DMA configuration tables
 *   0x0A00-0x0AFF: SCSI buffer management
 *   0x7000-0x7FFF: USB bulk OUT landing buffer (4KB, read-only to CPU)
 *   0x8000-0x8FFF: Writable XDATA (4KB, NOT aliased with 0xF000)
 *   0x9000-0x9FFF: MMIO - USB controller registers
 *   0xA000-0xAFFF: Writable XDATA (4KB)
 *   0xB000-0xBFFF: NVMe ASQ/ACQ + PCIe TLP engine registers
 *   0xC000-0xEFFF: MMIO - Various controllers
 *   0xD800-0xDFFF: USB endpoint buffer (2KB)
 *   0xF000-0xFFFF: Writable XDATA (4KB, NOT aliased with 0x8000)
 *
 * ===========================================================================
 * INTERNAL SRAM (PCI Address Space)
 * ===========================================================================
 *   0x00200000: Data buffer start (6+ MB)
 *               - SCSI bulk transfers land here
 *               - GPU can DMA to/from here
 *               - Accessed via CE76-CE79 (32-bit address)
 *   0x00820000: Queue region (128 KB)
 *               - NVMe submission/completion queues
 *               - Can be repurposed for other uses
 *
 * ===========================================================================
 * TRANSFER SEQUENCE
 * ===========================================================================
 *   1. Set transfer parameters in work area (G_DMA_MODE_SELECT, etc)
 *   2. Configure channel via dma_config_channel()
 *   3. Set buffer pointers and length
 *   4. Trigger transfer:
 *      - USB descriptor: Write 0x01 to 0x9092
 *      - XDATA transfer: Write 0x08 to 0xB296
 *      - SCSI bulk: Write 0x03 to 0xCE00
 *      - Flash: Write 0x01 to 0xC8A9
 *   5. Poll for completion:
 *      - USB: Check 0xE712 bits 0,1
 *      - XDATA: Check 0xB296 bits 1,2
 *      - SCSI: Poll 0xCE89 or 0xCE00
 *      - Flash: Poll 0xC8A9 bit 0
 *   6. Check status register for errors
 *   7. Clear status via appropriate clear function
 */
#ifndef _DMA_H_
#define _DMA_H_

#include "../types.h"

/*
 * DMA Transfer Mode constants for dma_setup_transfer()
 * Mode is stored in G_DMA_MODE_SELECT (0x0203)
 */
#define DMA_MODE_USB_RX         0x00    /* USB bulk OUT: host to device */
#define DMA_MODE_USB_TX         0x01    /* USB bulk IN: device to host */
#define DMA_MODE_SCSI_STATUS    0x03    /* SCSI status transfer */

/* DMA control */
void dma_clear_status(void);                    /* 0x1bcb-0x1bd4 */
void dma_set_scsi_param3(void);                 /* 0x16f3-0x16fe */
void dma_set_scsi_param1(void);                 /* 0x1709-0x1712 */
uint8_t dma_reg_wait_bit(__xdata uint8_t *ptr); /* 0x1713-0x171c */
void dma_load_transfer_params(void);            /* 0x16ff-0x1708 */

/* DMA channel configuration */
void dma_config_channel(uint8_t channel, uint8_t r4_param);     /* 0x171d-0x172b */
void dma_setup_transfer(uint8_t r7_mode, uint8_t r5_param, uint8_t r3_param);  /* 0x4a57-0x4a93 */
void dma_init_channel_b8(void);                 /* 0x523c-0x525f */
void dma_init_channel_with_config(uint8_t config);              /* 0x5260-0x5283 */
void dma_config_channel_0x10(void);             /* 0x1795-0x179c */

/* DMA status */
uint8_t dma_check_scsi_status(uint8_t mode);    /* 0x17a9-0x17b4 */
void dma_clear_state_counters(void);            /* 0x17b5-0x17c0 */
void dma_init_ep_queue(void);                   /* 0x172c-0x173a */
uint8_t scsi_get_tag_count_status(void);        /* 0x173b-0x1742 */
uint8_t dma_check_state_counter(void);          /* 0x17c1-0x17cc */
uint8_t scsi_get_queue_status(void);            /* 0x17cd-0x17d7 */
uint8_t dma_shift_and_check(uint8_t val);       /* 0x4a94-0x4abe */

/* DMA transfer */
void dma_start_transfer(uint8_t aux0, uint8_t aux1, uint8_t count_hi, uint8_t count_lo);  /* 0x1787-0x178d */
void dma_set_error_flag(void);                  /* 0x1743-0x1751 */

/* DMA address calculation */
uint8_t dma_get_config_offset_05a8(void);       /* 0x1779-0x1786 */
__xdata uint8_t *dma_calc_offset_0059(uint8_t offset);          /* 0x17f3-0x17fc */
__xdata uint8_t *dma_calc_addr_0478(uint8_t index);             /* 0x178e-0x1794 */
__xdata uint8_t *dma_calc_addr_0479(uint8_t index);             /* 0x179d-0x17a8 */
__xdata uint8_t *dma_calc_addr_00c2(void);      /* 0x180d-0x1819 */
__xdata uint8_t *dma_calc_ep_config_ptr(void);  /* 0x1602-0x1619 */
__xdata uint8_t *dma_calc_addr_046x(uint8_t offset);            /* 0x161a-0x1639 */
__xdata uint8_t *dma_calc_addr_0466(uint8_t offset);            /* 0x163a-0x1645 */
__xdata uint8_t *dma_calc_addr_0456(uint8_t offset);            /* 0x1646-0x1657 */
uint16_t dma_calc_addr_002c(uint8_t offset, uint8_t high);      /* 0x16ae-0x16b6 */

/* DMA SCSI operations */
uint8_t dma_shift_rrc2_mask(uint8_t val);       /* 0x16b7-0x16c2 */
void dma_store_to_0a7d(uint8_t val);            /* 0x16de-0x16e8 */
void dma_calc_scsi_index(void);                 /* 0x16e9-0x16f2 */
uint8_t dma_write_to_scsi_ce96(void);           /* 0x17d8-0x17e2 */
void dma_write_to_scsi_ce6e(void);              /* 0x17e3-0x17ec */
void dma_write_idata_to_dptr(__xdata uint8_t *ptr);             /* 0x17ed-0x17f2 */
void dma_read_0461(void);                       /* 0x17fd-0x1803 */
void dma_store_and_dispatch(uint8_t val);       /* 0x180d-0x181d */
void dma_clear_dword(__xdata uint8_t *ptr);     /* 0x173b-0x1742 */

/* Transfer functions */
uint16_t transfer_set_dptr_0464_offset(void);   /* 0x1659-0x1667 */
uint16_t transfer_calc_work43_offset(__xdata uint8_t *dptr);    /* 0x1668-0x1676 */
uint16_t transfer_calc_work53_offset(void);     /* 0x1677-0x1686 */
uint16_t transfer_get_ep_queue_addr(void);      /* 0x1687-0x1695 */
uint16_t transfer_calc_work55_offset(void);     /* 0x1696-0x16a1 */
void dma_write_scsi_status_pair(uint8_t param); /* 0x16b0-0x16b6 */
void dma_set_register_bit0(uint16_t addr);      /* 0x1633-0x1639 */

/* DMA handlers */
void dma_interrupt_handler(void);               /* 0x2608-0x2809 */
void dma_transfer_handler(uint8_t param);       /* 0xce23-0xce76 */
void transfer_continuation_d996(void);          /* 0xd996-0xda8e */
void dma_poll_complete(void);                   /* 0xceab-0xcece */
void dma_buffer_store_result_e68f(void);        /* 0xe68f-0xe6fb (Bank 1) */
void dma_poll_link_ready(void);                 /* 0xe6fc-0xe725 (Bank 1) */

#endif /* _DMA_H_ */
