#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "types.h"

/*
 * ASM2464PD Firmware - Global Variables
 *
 * Global variables in XRAM work area (addresses < 0x6000).
 * These are NOT hardware registers - they are RAM locations used
 * by the firmware for state tracking and data transfer.
 *
 * Memory map (from reverse engineering):
 * 0x0000-0x5FFF: 24 kB XRAM (work area)
 * 0x6000-0x6FFF: 4 kB unused
 * 0x7000-0x7FFF: 4 kB XRAM (SPI flash buffer)
 */

//=============================================================================
// Helper Macro
//=============================================================================
#define XDATA_VAR8(addr)   (*(__xdata uint8_t *)(addr))

//=============================================================================
// IDATA Work Variables (0x00-0x7F)
//=============================================================================
/* These are IDATA (internal 8051 RAM) locations used as fast work variables.
 * With __at() absolute addressing, these can be defined in the header. */
/* Boot signature bytes (0x09-0x0C) - used in startup_0016 */
__idata __at(0x09) uint8_t I_BOOT_SIG_0;      /* Boot signature byte 0 */
__idata __at(0x0A) uint8_t I_BOOT_SIG_1;      /* Boot signature byte 1 */
__idata __at(0x0B) uint8_t I_BOOT_SIG_2;      /* Boot signature byte 2 */
__idata __at(0x0C) uint8_t I_BOOT_SIG_3;      /* Boot signature byte 3 */
__idata __at(0x0D) uint8_t I_QUEUE_IDX;       /* Queue index / endpoint offset */
__idata __at(0x11) uint8_t I_SCSI_TAG;        /* SCSI command tag */
__idata __at(0x12) uint8_t I_WORK_12;         /* Work variable 0x12 */
__idata __at(0x13) uint8_t I_WORK_13;         /* Work variable 0x13 */
__idata __at(0x14) uint8_t I_WORK_14;         /* Work variable 0x14 */
__idata __at(0x15) uint8_t I_WORK_15;         /* Work variable 0x15 */

/* IDATA pointers for SCSI command buffer (0x12-0x15) */
#define IDATA_SCSI_CMD_BUF    ((__idata uint8_t *)0x12)   /* SCSI cmd buffer pointer */
__idata __at(0x16) uint8_t I_CORE_STATE_L;    /* Core state low byte */
#define I_WORK_16 I_CORE_STATE_L              /* Alias for work variable 0x16 */
__idata __at(0x17) uint8_t I_CORE_STATE_H;    /* Core state high byte */
#define I_WORK_17 I_CORE_STATE_H              /* Alias for work variable 0x17 */
__idata __at(0x18) uint8_t I_WORK_18;         /* Work variable 0x18 */
__idata __at(0x19) uint8_t I_WORK_19;         /* Work variable 0x19 */
__idata __at(0x21) uint8_t I_LOG_INDEX;       /* Log index */
#define I_WORK_21 I_LOG_INDEX  /* Alias for work variable 0x21 */
__idata __at(0x22) uint8_t I_SCSI_SLOT_INDEX;  /* SCSI command slot index (0-9) */
__idata __at(0x23) uint8_t I_WORK_23;         /* Work variable 0x23 */
__idata __at(0x38) uint8_t I_WORK_38;         /* Work variable 0x38 */
__idata __at(0x39) uint8_t I_WORK_39;         /* Work variable 0x39 */
__idata __at(0x3A) uint8_t I_WORK_3A;         /* Work variable 0x3A */
__idata __at(0x3B) uint8_t I_WORK_3B;         /* Work variable 0x3B */
__idata __at(0x3C) uint8_t I_WORK_3C;         /* Work variable 0x3C */
__idata __at(0x3D) uint8_t I_WORK_3D;         /* Work variable 0x3D */
__idata __at(0x3E) uint8_t I_WORK_3E;         /* Work variable 0x3E */
__idata __at(0x3F) uint8_t I_TRANSFER_COUNT;   /* Transfer count */
__idata __at(0x40) uint8_t I_XFER_STATUS;      /* Transfer status from REG_XFER_STATUS_CE60 */
__idata __at(0x41) uint8_t I_SLOT_START_INDEX; /* Slot start index for calculations */
__idata __at(0x42) uint8_t I_TAG_STATUS;       /* Command tag status */
__idata __at(0x43) uint8_t I_CMD_SLOT_INDEX;   /* Command slot index */
__idata __at(0x44) uint8_t I_MULTIPLIER;       /* Multiplier value */
__idata __at(0x45) uint8_t I_CHAIN_INDEX;      /* DMA chain index */
__idata __at(0x46) uint8_t I_CHAIN_FLAG;       /* DMA chain flag */
__idata __at(0x47) uint8_t I_PRODUCT_CAP;      /* Product capabilities */
__idata __at(0x4D) uint8_t I_FLASH_STATE_4D;  /* Flash state dispatch value */
__idata __at(0x51) uint8_t I_LOOP_COUNTER;     /* Loop counter / status index */
__idata __at(0x52) uint8_t I_POLL_STATUS;      /* Poll status / DMA offset */
__idata __at(0x53) uint8_t I_QUEUE_STATUS;     /* Queue status / system status copy */
__idata __at(0x54) uint8_t I_WORK_54;          /* Work variable 0x54 */
__idata __at(0x55) uint8_t I_VENDOR_STATE;     /* Vendor state/mode */
__idata __at(0x56) uint8_t I_DMA_QUEUE_INDEX;  /* DMA queue index */
__idata __at(0x61) uint8_t I_PCIE_TXN_DATA_0;  /* PCIe transaction data byte 0 */
__idata __at(0x62) uint8_t I_PCIE_TXN_DATA_1;  /* PCIe transaction data byte 1 */
__idata __at(0x57) uint8_t I_VENDOR_CDB_ADDR_LO;   /* Vendor CDB address low byte */
__idata __at(0x58) uint8_t I_VENDOR_CDB_VALUE;     /* Vendor CDB value / address mid */
__idata __at(0x59) uint8_t I_VENDOR_CDB_ADDR_HI1;  /* Vendor CDB address high byte 1 */
__idata __at(0x5A) uint8_t I_VENDOR_CDB_ADDR_HI0;  /* Vendor CDB address high byte 0 */
__idata __at(0x63) uint8_t I_EP_CONFIG_HI;     /* Endpoint config high byte */
__idata __at(0x64) uint8_t I_EP_CONFIG_LO;     /* Endpoint config low byte */
__idata __at(0x65) uint8_t I_EP_MODE;          /* Endpoint mode */
/*
 * USB State Machine (IDATA 0x6A)
 * Tracks current USB device state per USB 2.0 specification.
 * State transitions occur in ISR at 0x0E68 and main loop at 0x202A.
 */
__idata __at(0x6A) uint8_t I_USB_STATE;       /* USB device state machine */

/* USB Device State Values */
#define USB_STATE_DISCONNECTED  0   /* No USB connection */
#define USB_STATE_ATTACHED      1   /* Cable connected, not powered */
#define USB_STATE_POWERED       2   /* Bus powered, no address */
#define USB_STATE_DEFAULT       3   /* Default address assigned */
#define USB_STATE_ADDRESS       4   /* Device address assigned */
#define USB_STATE_CONFIGURED    5   /* Configuration set, ready for commands */
__idata __at(0x6B) uint8_t I_TRANSFER_6B;     /* Transfer pending byte 0 */
__idata __at(0x6C) uint8_t I_TRANSFER_6C;     /* Transfer pending byte 1 */
__idata __at(0x6D) uint8_t I_TRANSFER_6D;     /* Transfer pending byte 2 */
__idata __at(0x6E) uint8_t I_TRANSFER_6E;     /* Transfer pending byte 3 */
__idata __at(0x6F) uint8_t I_BUF_FLOW_CTRL;   /* Buffer flow control */
__idata __at(0x70) uint8_t I_BUF_THRESH_LO;   /* Buffer threshold low */
__idata __at(0x71) uint8_t I_BUF_THRESH_HI;   /* Buffer threshold high */
__idata __at(0x72) uint8_t I_BUF_CTRL_GLOBAL; /* Buffer control global */

/* IDATA pointers for SCSI register-based operations */
#define IDATA_CMD_BUF     ((__idata uint8_t *)0x09)   /* Command buffer pointer */
#define IDATA_TRANSFER    ((__idata uint8_t *)0x6B)   /* Transfer data pointer */
#define IDATA_BUF_CTRL    ((__idata uint8_t *)0x6F)   /* Buffer control pointer */

//=============================================================================
// System Work Area (0x0000-0x01FF)
//=============================================================================
#define G_SYSTEM_CTRL           XDATA_VAR8(0x0000)  /* System control byte */
#define G_IO_CMD_TYPE           XDATA_VAR8(0x0001)  /* I/O command type byte */
#define G_IO_CMD_STATE          XDATA_VAR8(0x0002)  /* I/O command state byte */
#define G_EP_STATUS_CTRL        XDATA_VAR8(0x0003)  /* Endpoint status control (checked by usb_ep_process) */
#define G_WORK_0004             XDATA_VAR8(0x0004)  /* Work variable 0x0004 */
#define G_DMA_DIRECTION_0005    XDATA_VAR8(0x0005)  /* DMA direction / transfer flag */
#define G_WORK_0006             XDATA_VAR8(0x0006)  /* Work variable 0x0006 */
#define G_WORK_0007             XDATA_VAR8(0x0007)  /* Work variable 0x0007 */
#define G_BOOT_STATUS_0009      XDATA_VAR8(0x0009)  /* Boot status byte */
#define G_USB_CTRL_000A         XDATA_VAR8(0x000A)  /* USB control byte (increment counter) */
#define G_EP_CHECK_FLAG         G_USB_CTRL_000A     /* Alias: Endpoint check flag */
#define G_USB_CTRL_000B         XDATA_VAR8(0x000B)  /* USB control byte B (cleared in post_csw_cleanup) */
#define G_IFACE_NUM_002E        XDATA_VAR8(0x002E)  /* Interface number (set by post_csw_cleanup) */
#define G_EP_ALT_STATE_0050     XDATA_VAR8(0x0050)  /* EP alt setting state (0x21 = configured) */
#define G_ENDPOINT_STATE_0051   XDATA_VAR8(0x0051)  /* Endpoint state storage */
#define G_SYS_FLAGS_0052        XDATA_VAR8(0x0052)  /* System flags 0x0052 */
#define G_USB_SETUP_RESULT      XDATA_VAR8(0x0053)  /* USB setup result storage */
#define G_BUFFER_LENGTH_HIGH    XDATA_VAR8(0x0054)  /* Buffer length high byte (for mode 4) */
#define G_NVME_QUEUE_READY      XDATA_VAR8(0x0055)  /* NVMe queue ready flag */
#define G_DMA_SRC_HI             XDATA_VAR8(0x0056)  /* Software DMA source/dest address high byte */
#define G_DMA_SRC_LO             XDATA_VAR8(0x0057)  /* Software DMA source/dest address low byte */
#define G_USB_ADDR_HI_0056      G_DMA_SRC_HI        /* Alias: USB address high 0x0056 */
#define G_USB_ADDR_LO_0057      G_DMA_SRC_LO        /* Alias: USB address low 0x0057 */
#define G_USB_WORK_009F         XDATA_VAR8(0x009F)  /* USB work array base */
#define G_USB_MODE_FLAG_00BF    XDATA_VAR8(0x00BF)  /* USB mode flag */
#define G_INIT_STATE_00C2       XDATA_VAR8(0x00C2)  /* Initialization state flag */
#define G_LOOP_STATE_00E2       XDATA_VAR8(0x00E2)  /* Loop state variable */
#define G_USB_BUF_BASE          ((__xdata uint8_t *)0x0100)  /* USB buffer base */
#define G_USB_STATE_0105        XDATA_VAR8(0x0105)  /* USB state variable */
#define G_USB_INIT_STATE_0108   XDATA_VAR8(0x0108)  /* USB initialization state */
#define G_WORK_012B             XDATA_VAR8(0x012B)  /* Work variable 0x012B */
#define G_WORK_0128             XDATA_VAR8(0x0128)  /* Work variable 0x0128 */
#define G_INIT_STATE_00E5       XDATA_VAR8(0x00E5)  /* Initialization state flag 2 */
#define G_USB_INDEX_COUNTER     XDATA_VAR8(0x014E)  /* USB index counter (5-bit) */
/* SCSI slot arrays (indexed by I_SCSI_SLOT_INDEX) */
#define G_SCSI_SLOT_4E_BASE     ((__xdata uint8_t *)0x004E)  /* SCSI slot config base */
#define G_SCSI_SLOT_71_BASE     ((__xdata uint8_t *)0x0171)  /* SCSI slot ctrl base */
#define G_SCSI_SLOT_7C_BASE     ((__xdata uint8_t *)0x017C)  /* SCSI slot status base */
#define G_SCSI_CTRL             XDATA_VAR8(0x0171)  /* SCSI control (alias for slot 0) */
#define G_USB_WORK_01B4         XDATA_VAR8(0x01B4)  /* USB work variable 0x01B4 */
#define G_USB_WORK_01B6         XDATA_VAR8(0x01B6)  /* USB work variable 0x01B6 */

//=============================================================================
// DMA Work Area (0x0200-0x02FF)
//=============================================================================
#define G_DMA_MODE_SELECT       XDATA_VAR8(0x0203)  /* DMA mode select */
#define G_FLASH_READ_TRIGGER    XDATA_VAR8(0x0213)  /* Flash read trigger */
#define G_DMA_STATE_0214        XDATA_VAR8(0x0214)  /* DMA state/status */
#define G_DMA_PARAM1            XDATA_VAR8(0x020D)  /* DMA parameter 1 */
#define G_DMA_PARAM2            XDATA_VAR8(0x020E)  /* DMA parameter 2 */
#define G_DMA_WORK_0216         XDATA_VAR8(0x0216)  /* DMA work variable 0x0216 */
#define G_DMA_OFFSET            XDATA_VAR8(0x0217)  /* DMA offset storage */
#define G_BUF_ADDR_HI           XDATA_VAR8(0x0218)  /* Buffer address high */
#define G_BUF_ADDR_LO           XDATA_VAR8(0x0219)  /* Buffer address low */
#define G_BUF_BASE_HI           XDATA_VAR8(0x021A)  /* Buffer base address high */
#define G_BUF_BASE_LO           XDATA_VAR8(0x021B)  /* Buffer base address low */
#define G_BANK1_STATE_023F      XDATA_VAR8(0x023F)  /* Bank 1 state flag */
#define G_STATE_CHECKSUM        XDATA_VAR8(0x0240)  /* State checksum storage */
#define G_STATE_CHECKSUM_DATA   ((__xdata uint8_t *)0x0241)  /* State checksum data base (8 bytes) */

//=============================================================================
// System Status Work Area (0x0400-0x04FF)
//=============================================================================
#define G_PCIE_STATE_0442       XDATA_VAR8(0x0442)  /* PCIe state buffer byte 0 */
#define G_PCIE_STATE_0443       XDATA_VAR8(0x0443)  /* PCIe state buffer byte 1 */
#define G_LOG_COUNTER_044B      XDATA_VAR8(0x044B)  /* Log counter */
#define G_LOG_ACTIVE_044C       XDATA_VAR8(0x044C)  /* Log active flag */
#define G_LOG_INIT_044D         XDATA_VAR8(0x044D)  /* Log init flag */
#define G_SYS_STATUS_BASE       ((__xdata uint8_t *)0x0456)  /* System status array base */
#define G_REG_WAIT_BIT          XDATA_VAR8(0x045E)  /* Register wait bit */
#define G_DMA_ADDR_LO           XDATA_VAR8(0x045F)  /* DMA address low byte */
#define G_DMA_ADDR_HI           XDATA_VAR8(0x0460)  /* DMA address high byte */
#define G_SYS_STATUS_PRIMARY    XDATA_VAR8(0x0464)  /* Primary system status */
#define G_EP_INDEX_ALT          G_SYS_STATUS_PRIMARY  /* Alias for endpoint index */
#define G_SYS_STATUS_SECONDARY  XDATA_VAR8(0x0465)  /* Secondary system status */
#define G_EP_INDEX              G_SYS_STATUS_SECONDARY  /* Alias for endpoint index */
#define G_SYSTEM_CONFIG         XDATA_VAR8(0x0466)  /* System configuration */
#define G_SYSTEM_STATE          XDATA_VAR8(0x0467)  /* System state */
#define G_DATA_PORT             XDATA_VAR8(0x0468)  /* Data port */
#define G_INT_STATUS            XDATA_VAR8(0x0469)  /* Interrupt status */
#define G_SCSI_CMD_PARAM_0470   XDATA_VAR8(0x0470)  /* SCSI command parameter */
#define G_DMA_LOAD_PARAM1       XDATA_VAR8(0x0472)  /* DMA load parameter 1 */
#define G_DMA_LOAD_PARAM2       XDATA_VAR8(0x0473)  /* DMA load parameter 2 */
#define G_STATE_HELPER_41       XDATA_VAR8(0x0474)  /* State helper byte from R41 */
#define G_STATE_HELPER_42       XDATA_VAR8(0x0475)  /* State helper byte from R42 (masked) */
#define G_XFER_DIV_0476         XDATA_VAR8(0x0476)  /* Transfer division result */
#define G_STATE_FLAG_04D7       XDATA_VAR8(0x04D7)  /* State flag variable */
#define G_STATE_FLAG_04F7       XDATA_VAR8(0x04F7)  /* State flag variable 2 */

//=============================================================================
// Endpoint Configuration Work Area (0x0500-0x05FF)
//=============================================================================
#define G_EP_WORK_BASE          ((__xdata uint8_t *)0x0500)  /* EP work area base */
#define G_EP_SLOT_COUNTER_OFF   0x17  /* Offset to slot counter within EP work slot */
#define G_EP_INIT_0517          XDATA_VAR8(0x0517)  /* Endpoint init state */
#define G_NVME_PARAM_053A       XDATA_VAR8(0x053A)  /* NVMe parameter storage */
#define G_NVME_STATE_053B       XDATA_VAR8(0x053B)  /* NVMe state flag */
#define G_SCSI_CMD_TYPE         XDATA_VAR8(0x053D)  /* SCSI command type */
#define G_SCSI_TRANSFER_FLAG    XDATA_VAR8(0x053E)  /* SCSI transfer flag */
#define G_SCSI_BUF_LEN_0        XDATA_VAR8(0x053F)  /* SCSI buffer length byte 0 */
#define G_SCSI_BUF_LEN_1        XDATA_VAR8(0x0540)  /* SCSI buffer length byte 1 */
#define G_SCSI_BUF_LEN_2        XDATA_VAR8(0x0541)  /* SCSI buffer length byte 2 */
#define G_SCSI_BUF_LEN_3        XDATA_VAR8(0x0542)  /* SCSI buffer length byte 3 */
#define G_SCSI_LBA_0            XDATA_VAR8(0x0543)  /* SCSI LBA byte 0 */
#define G_SCSI_LBA_1            XDATA_VAR8(0x0544)  /* SCSI LBA byte 1 */
#define G_SCSI_LBA_2            XDATA_VAR8(0x0545)  /* SCSI LBA byte 2 */
#define G_SCSI_LBA_3            XDATA_VAR8(0x0546)  /* SCSI LBA byte 3 */
#define G_SCSI_DEVICE_IDX       XDATA_VAR8(0x0547)  /* SCSI device index */
#define G_EP_CONFIG_BASE        XDATA_VAR8(0x054B)  /* EP config base */
#define G_EP_CONFIG_ARRAY       XDATA_VAR8(0x054E)  /* EP config array */
#define G_SCSI_MODE_FLAG        XDATA_VAR8(0x054F)  /* SCSI mode flag */
#define G_SCSI_STATUS_FLAG      XDATA_VAR8(0x0552)  /* SCSI status flag */
#define G_EP_QUEUE_CTRL         XDATA_VAR8(0x0564)  /* Endpoint queue control */
#define G_EP_QUEUE_STATUS       XDATA_VAR8(0x0565)  /* Endpoint queue status */
#define G_EP_QUEUE_PARAM        XDATA_VAR8(0x0566)  /* Endpoint queue parameter */
#define G_EP_QUEUE_IDATA        XDATA_VAR8(0x0567)  /* Endpoint queue IDATA copy */
#define G_BUF_OFFSET_HI         XDATA_VAR8(0x0568)  /* Buffer offset result high */
#define G_BUF_OFFSET_LO         XDATA_VAR8(0x0569)  /* Buffer offset result low */
#define G_EP_QUEUE_IDATA2       XDATA_VAR8(0x056A)  /* Endpoint queue IDATA byte 2 */
#define G_EP_QUEUE_IDATA3       XDATA_VAR8(0x056B)  /* Endpoint queue IDATA byte 3 */
#define G_LOG_PROCESS_STATE     XDATA_VAR8(0x0574)  /* Log processing state */
#define G_LOG_ENTRY_VALUE       XDATA_VAR8(0x0575)  /* Log entry value */
/*
 * Vendor Command Index Tracking (0x05A3-0x05A5)
 * Used by E4/E5 command processing to track current command slot.
 */
#define G_CMD_SLOT_INDEX        XDATA_VAR8(0x05A3)  /* Current command slot index (0-9) */
#define G_CMD_INDEX_SRC         XDATA_VAR8(0x05A5)  /* Command index source/copy */
#define G_PCIE_TXN_COUNT_LO     XDATA_VAR8(0x05A6)  /* PCIe transaction count low */
#define G_PCIE_TXN_COUNT_HI     XDATA_VAR8(0x05A7)  /* PCIe transaction count high */
#define G_EP_CONFIG_05A4        XDATA_VAR8(0x05A4)  /* EP config state (cleared in C24C bridge init) */
#define G_EP_CONFIG_05A8        XDATA_VAR8(0x05A8)  /* EP config 0x05A8 */
#define G_PCIE_BRIDGE_STATE_05A9 XDATA_VAR8(0x05A9) /* PCIe bridge state (cleared in C24C) */
#define G_PCIE_BRIDGE_STATE_05AA XDATA_VAR8(0x05AA) /* PCIe bridge state (cleared in C24C) */
#define G_PCIE_BRIDGE_STATE_05AB XDATA_VAR8(0x05AB) /* PCIe bridge state */
#define G_PCIE_ADDR_OFFSET_LO   XDATA_VAR8(0x05AC)  /* PCIe address offset low */
#define G_PCIE_ADDR_OFFSET_HI   XDATA_VAR8(0x05AD)  /* PCIe address offset high */
#define G_PCIE_DIRECTION        XDATA_VAR8(0x05AE)  /* PCIe direction (bit 0: 0=read, 1=write) */
#define G_PCIE_ADDR_0           XDATA_VAR8(0x05AF)  /* PCIe target address byte 0 */
#define G_PCIE_ADDR_1           XDATA_VAR8(0x05B0)  /* PCIe target address byte 1 */

/*
 * Command Table (0x05B1-0x06CF)
 * Array of 10 command slots, each 34 (0x22) bytes.
 * Used for tracking pending E4/E5 vendor commands.
 *
 * Table entry structure (34 bytes per entry):
 *   Offset 0x00: Command type/status
 *   Offset 0x01: Command parameter 1
 *   Offset 0x02: Command parameter 2
 *   ... (additional fields vary by command type)
 *
 * Entry addresses: 0x05B1, 0x05D3, 0x05F5, 0x0617, 0x0639, ...
 */
#define G_CMD_TABLE_BASE        ((__xdata uint8_t *)0x05B1)  /* Command table base */
#define G_CMD_TABLE_ENTRY_SIZE  0x22                /* 34 bytes per entry */
#define G_CMD_TABLE_MAX_ENTRIES 10                  /* Maximum 10 command slots */

/* Legacy PCIe address aliases (overlap with command table) */
#define G_PCIE_ADDR_2           XDATA_VAR8(0x05B1)  /* PCIe target address byte 2 / CMD_TABLE[0] */
#define G_PCIE_ADDR_3           XDATA_VAR8(0x05B2)  /* PCIe target address byte 3 / CMD_TABLE[1] */
#define G_PCIE_TXN_TABLE        ((__xdata uint8_t *)0x05B7) /* PCIe transaction table (34-byte entries) */
#define G_PCIE_TXN_ENTRY_SIZE   34                  /* Size of each table entry */
#define G_EP_LOOKUP_TABLE       XDATA_VAR8(0x057A)  /* EP lookup table index */
#define G_EP_CONFIG_05F8        XDATA_VAR8(0x05F8)  /* EP config 0x05F8 */

//=============================================================================
// Transfer Work Area (0x0600-0x07FF)
//=============================================================================
#define G_STATE_FLAG_0719       XDATA_VAR8(0x0719)  /* State flag for NVMe queue handling */
#define G_DMA_WORK_05AC         XDATA_VAR8(0x05AC)  /* DMA work byte */
#define G_DMA_WORK_05AD         XDATA_VAR8(0x05AD)  /* DMA work byte */
#define G_USB_STATE_CLEAR_06E3  XDATA_VAR8(0x06E3)  /* USB state clear flag */
#define G_BRIDGE_INIT_06E4      XDATA_VAR8(0x06E4)  /* Bridge init flag (set to 1 in C24C) */
#define G_MAX_LOG_ENTRIES       XDATA_VAR8(0x06E5)  /* Max error log entries */
#define G_QUEUE_COUNT_06E5      G_MAX_LOG_ENTRIES   /* Alias - queue count */
#define G_STATE_FLAG_06E6       XDATA_VAR8(0x06E6)  /* Processing complete flag / error flag */
#define G_SCSI_STATUS_06CB      XDATA_VAR8(0x06CB)  /* SCSI status byte */
#define G_WORK_06E7             XDATA_VAR8(0x06E7)  /* Work variable 0x06E7 */
#define G_WORK_06E8             XDATA_VAR8(0x06E8)  /* Work variable 0x06E8 */
#define G_ERROR_CODE_06EA       XDATA_VAR8(0x06EA)  /* Error code */
#define G_WORK_06EB             XDATA_VAR8(0x06EB)  /* Work variable 0x06EB */
#define G_MISC_FLAG_06EC        XDATA_VAR8(0x06EC)  /* Miscellaneous flag */

//=============================================================================
// Vendor Command Work Area (0x0750-0x081F)
//=============================================================================
#define G_VENDOR_ACTIVE_0750    XDATA_VAR8(0x0750)  /* Vendor command active flag */
#define G_VENDOR_STATE_0765     XDATA_VAR8(0x0765)  /* Vendor command state */
#define G_VENDOR_STATE_0775     XDATA_VAR8(0x0775)  /* Vendor command state 2 */
#define G_VENDOR_CMD_BUF_0804   XDATA_VAR8(0x0804)  /* Vendor command buffer byte 0 */
#define G_VENDOR_CMD_BUF_0805   XDATA_VAR8(0x0805)  /* Vendor command buffer byte 1 */
#define G_VENDOR_CDB_BASE       XDATA_VAR8(0x0810)  /* Vendor CDB buffer base (16 bytes) */
#define G_VENDOR_RESP_BUF       XDATA_VAR8(0x0816)  /* Vendor response buffer */
#define G_VENDOR_STATUS_081A    XDATA_VAR8(0x081A)  /* Vendor status register */
#define G_VENDOR_STATUS_081B    XDATA_VAR8(0x081B)  /* Vendor status register 2 */
#define G_FLASH_CMD_FLAG        XDATA_VAR8(0x07B8)  /* Flash command flag */
#define G_VENDOR_CTRL_07B9      XDATA_VAR8(0x07B9)  /* Vendor control 0x07B9 */
#define G_VENDOR_MODE_07CC      XDATA_VAR8(0x07CC)  /* Vendor mode state */
#define G_VENDOR_MODE_07CF      XDATA_VAR8(0x07CF)  /* Vendor mode state 2 */
#define G_FLASH_CMD_TYPE        XDATA_VAR8(0x07BC)  /* Flash command type (1,2,3) */
#define G_FLASH_OP_COUNTER      XDATA_VAR8(0x07BD)  /* Flash operation counter */
#define G_SYS_FLAGS_BASE        XDATA_VAR8(0x07E4)  /* Flags base */
#define G_TRANSFER_ACTIVE       XDATA_VAR8(0x07E5)  /* Transfer active flag */
#define G_USB_CTRL_STATE_07E1   XDATA_VAR8(0x07E1)  /* USB control transfer state (5=ready to send) */
#define G_STATE_FLAG_07E2       XDATA_VAR8(0x07E2)  /* State flag (cleared in bda4 reset) */
#define G_USB_STATUS_07E6       XDATA_VAR8(0x07E6)  /* USB status (cleared in bda4 reset) */
#define G_USB_STATUS_07E7       XDATA_VAR8(0x07E7)  /* USB status (cleared in bda4 reset) */
#define G_XFER_FLAG_07EA        XDATA_VAR8(0x07EA)  /* Transfer flag 0x07EA (set in SCSI DMA) */
#define G_SYS_FLAGS_07EB        XDATA_VAR8(0x07EB)  /* System flags 0x07EB */
#define G_USB_CMD_CONFIG        XDATA_VAR8(0x07EC)  /* USB command configuration (vendor cmd state) */
#define G_SYS_FLAGS_07ED        XDATA_VAR8(0x07ED)  /* System flags 0x07ED */
#define G_SYS_FLAGS_07EE        XDATA_VAR8(0x07EE)  /* System flags 0x07EE */
#define G_SYS_FLAGS_07EF        XDATA_VAR8(0x07EF)  /* System flags 0x07EF */
#define G_SYS_FLAGS_07F0        XDATA_VAR8(0x07F0)  /* System flags 0x07F0 */
#define G_SYS_FLAGS_07F1        XDATA_VAR8(0x07F1)  /* System flags 0x07F1 */
#define G_SYS_FLAGS_07F2        XDATA_VAR8(0x07F2)  /* System flags 0x07F2 */
#define G_SYS_FLAGS_07F3        XDATA_VAR8(0x07F3)  /* System flags 0x07F3 */
#define G_SYS_FLAGS_07F4        XDATA_VAR8(0x07F4)  /* System flags 0x07F4 */
#define G_SYS_FLAGS_07F5        XDATA_VAR8(0x07F5)  /* System flags 0x07F5 */
#define G_SYS_FLAGS_07F6        XDATA_VAR8(0x07F6)  /* System flags 0x07F6 */
#define G_SYS_FLAGS_07E8        XDATA_VAR8(0x07E8)  /* System flags 0x07E8 */
#define G_TLP_STATE_07E9        XDATA_VAR8(0x07E9)  /* TLP state / queue status */
#define G_SYS_FLAGS_07F7        XDATA_VAR8(0x07F7)  /* System flags 0x07F7 */

//=============================================================================
// Flash Config Storage (0x0860-0x08FF)
// Loaded from flash buffer at 0x7074+ during config initialization
//=============================================================================
#define G_SERIAL_NUM_0864       XDATA_VAR8(0x0864)  /* Serial number string start */
#define G_FLASH_CFG_086C        XDATA_VAR8(0x086C)  /* Flash config from 0x7074 */
#define G_FLASH_CFG_086D        XDATA_VAR8(0x086D)  /* Flash config from 0x7075 */
#define G_FLASH_CFG_086E        XDATA_VAR8(0x086E)  /* Flash config from 0x7076 */
#define G_FLASH_CFG_086F        XDATA_VAR8(0x086F)  /* Flash config from 0x7077 */
#define G_FLASH_CFG_0870        XDATA_VAR8(0x0870)  /* Flash config from 0x7078 */
#define G_FLASH_CFG_0871        XDATA_VAR8(0x0871)  /* Flash config from 0x7079 */

//=============================================================================
// Event/Loop State Work Area (0x0900-0x09FF)
//=============================================================================
#define G_EVENT_INIT_097A       XDATA_VAR8(0x097A)  /* Event init value */
#define G_LOOP_CHECK_098E       XDATA_VAR8(0x098E)  /* Loop check byte */
#define G_LOOP_STATE_0991       XDATA_VAR8(0x0991)  /* Loop state byte */
#define G_EVENT_CHECK_09EF      XDATA_VAR8(0x09EF)  /* Event check byte */
#define G_FLASH_MODE_1          XDATA_VAR8(0x09F4)  /* Flash mode config 1 */
#define G_FLASH_MODE_2          XDATA_VAR8(0x09F5)  /* Flash mode config 2 */
#define G_FLASH_MODE_3          XDATA_VAR8(0x09F6)  /* Flash mode config 3 */
#define G_FLASH_MODE_4          XDATA_VAR8(0x09F7)  /* Flash mode config 4 */
#define G_FLASH_MODE_5          XDATA_VAR8(0x09F8)  /* Flash mode config 5 */
#define G_EVENT_FLAGS           XDATA_VAR8(0x09F9)  /* Event flags */
#define   EVENT_FLAG_PENDING      0x01  // Bit 0: Event pending
#define   EVENT_FLAG_PROCESS      0x02  // Bit 1: Process event
#define   EVENT_FLAG_POWER        0x04  // Bit 2: Power event
#define   EVENT_FLAG_ACTIVE       0x80  // Bit 7: Events active
#define   EVENT_FLAGS_ANY         0x83  // Bits 0,1,7: Any event flag
#define G_EVENT_CTRL_09FA       XDATA_VAR8(0x09FA)  /* Event control */
#define G_FLASH_STATUS_09FB     XDATA_VAR8(0x09FB)  /* Flash status mode */

//=============================================================================
// Flash Config Work Area (0x0A40-0x0A4F)
//=============================================================================
#define G_FLASH_CFG_0A41        XDATA_VAR8(0x0A41)  /* Flash config byte */
#define G_FLASH_CFG_0A42        XDATA_VAR8(0x0A42)  /* Flash config byte */
#define G_FLASH_CFG_0A43        XDATA_VAR8(0x0A43)  /* Flash config byte */
#define G_FLASH_CFG_0A44        XDATA_VAR8(0x0A44)  /* Flash config byte */
#define G_FLASH_CFG_0A45        XDATA_VAR8(0x0A45)  /* Flash config byte */

//=============================================================================
// PCIe Tunnel Adapter Config (0x0A52-0x0A55)
//=============================================================================
#define G_PCIE_ADAPTER_CFG_LO   XDATA_VAR8(0x0A52)  /* Adapter config low (link config high) */
#define G_PCIE_ADAPTER_CFG_HI   XDATA_VAR8(0x0A53)  /* Adapter config high (link config low) */
#define G_PCIE_ADAPTER_MODE     XDATA_VAR8(0x0A54)  /* Adapter mode config */
#define G_PCIE_ADAPTER_AUX      XDATA_VAR8(0x0A55)  /* Adapter auxiliary config */
#define G_FLASH_CONFIG_VALID    XDATA_VAR8(0x0A56)  /* Flash config valid flag */
#define G_CMD_CTRL_PARAM        XDATA_VAR8(0x0A57)  /* Command control parameter for E430 */
#define G_CMD_TIMEOUT_PARAM     XDATA_VAR8(0x0A58)  /* Command timeout parameter for E431 */

//=============================================================================
// Endpoint Dispatch Work Area (0x0A00-0x0BFF)
//=============================================================================
#define G_LOOP_STATE            XDATA_VAR8(0x0A59)  /* Main loop state flag */
#define G_LOOP_STATE_0A5A       XDATA_VAR8(0x0A5A)  /* Main loop state secondary */

/*
 * Vendor Command CDB Storage (0x0A81-0x0A82)
 * Stores the parsed CDB bytes from USB vendor commands (E4/E5).
 * Copied from USB registers 0x910D-0x910E by protocol handler at 0x33c1.
 */
#define G_VENDOR_CMD_TYPE       XDATA_VAR8(0x0A81)  /* CDB byte 0: Command type (0xE4=read, 0xE5=write) */
#define G_VENDOR_CMD_SIZE       XDATA_VAR8(0x0A82)  /* CDB byte 1: Transfer size (0-255 bytes) */

#define G_ACTION_CODE_0A83      XDATA_VAR8(0x0A83)  /* Action code storage for state_action_dispatch */
#define   ACTION_CODE_EXTENDED    0x02  // Bit 1: Extended mode flag
#define G_ACTION_PARAM_0A84     XDATA_VAR8(0x0A84)  /* Action parameter (byte after action code) */
#define G_DMA_PARAM_0A8D        XDATA_VAR8(0x0A8D)  /* DMA parameter storage */
#define G_DMA_MODE_0A8E         XDATA_VAR8(0x0A8E)  /* DMA mode storage */
#define G_EP_DISPATCH_VAL1      XDATA_VAR8(0x0A7B)  /* Endpoint dispatch value 1 */
#define G_EP_DISPATCH_VAL2      XDATA_VAR8(0x0A7C)  /* Endpoint dispatch value 2 */
#define G_EP_DISPATCH_VAL3      XDATA_VAR8(0x0A7D)  /* Endpoint dispatch value 3 */
#define G_USB_EP_MODE           G_EP_DISPATCH_VAL3  /* Alias: USB endpoint mode flag */
#define G_EP_DISPATCH_VAL4      XDATA_VAR8(0x0A7E)  /* Endpoint dispatch value 4 */
/*
 * Vendor Command Handler State (0x0AA0-0x0AA1)
 * Used by pcie_vendor_handler_35b7 to track E4/E5 command state.
 *
 * 0x0AA0 values:
 *   0x00: Idle / no command
 *   0x01: E5 write command in progress
 *   0x03: E4 read command complete (sets 0x80 in cmd table entry)
 *   0x81: Passed as initial param from vendor_dispatch_4583
 *
 * 0x0AA1 values:
 *   0x00: Normal completion
 *   0x01: Error/retry needed (sets 0x81 in cmd table entry)
 */
#define G_VENDOR_HANDLER_STATE  XDATA_VAR8(0x0AA0)  /* Vendor handler state (E4/E5) */
#define G_VENDOR_HANDLER_RESULT XDATA_VAR8(0x0AA1)  /* Vendor handler result flag */
/* Legacy aliases for compatibility */
#define G_DMA_XFER_STATUS       G_VENDOR_HANDLER_STATE

#define G_STATE_COUNTER_HI      XDATA_VAR8(0x0AA3)  /* State counter high */
#define G_STATE_COUNTER_LO      XDATA_VAR8(0x0AA4)  /* State counter low */
#define G_STATE_COUNTER_0AA5    XDATA_VAR8(0x0AA5)  /* State counter byte 2 */
#define G_LOG_PROCESSED_INDEX   XDATA_VAR8(0x0AA2)  /* Current processed log index (was 0x0AA1) */
#define G_STATE_PARAM_0AA2      XDATA_VAR8(0x0AA2)  /* State machine parameter */
/* NOTE: G_STATE_RESULT_0AA3 removed - use G_STATE_COUNTER_HI (same address) */
/* NOTE: G_STATE_WORK_0A84 removed - use G_ACTION_PARAM_0A84 (same address) */
/* TLP handler state variables (also used for flash operations) */
#define G_TLP_COUNT_HI          XDATA_VAR8(0x0AA8)  /* TLP transfer count high byte */
#define G_TLP_COUNT_LO          XDATA_VAR8(0x0AA9)  /* TLP transfer count low byte */
#define G_TLP_STATUS            XDATA_VAR8(0x0AAA)  /* TLP status / pending count */
/* Legacy names for flash compatibility */
#define G_FLASH_ERROR_0         G_TLP_COUNT_HI      /* Alias: Flash error flag 0 */
#define G_FLASH_ERROR_1         G_TLP_COUNT_LO      /* Alias: Flash error flag 1 */
#define G_FLASH_RESET_0AAA      G_TLP_STATUS        /* Alias: Flash reset flag */
#define G_STATE_HELPER_0AAB     XDATA_VAR8(0x0AAB)  /* State helper variable */
#define G_STATE_COUNTER_0AAC    XDATA_VAR8(0x0AAC)  /* State counter/index */
#define G_FLASH_ADDR_0          XDATA_VAR8(0x0AAD)  /* Flash address byte 0 (low) */
#define G_FLASH_ADDR_1          XDATA_VAR8(0x0AAE)  /* Flash address byte 1 */
#define G_FLASH_ADDR_2          XDATA_VAR8(0x0AAF)  /* Flash address byte 2 */
#define G_FLASH_ADDR_3          XDATA_VAR8(0x0AB0)  /* Flash address byte 3 (high) */
#define G_FLASH_LEN_LO          XDATA_VAR8(0x0AB1)  /* Flash data length low */
#define G_FLASH_LEN_HI          XDATA_VAR8(0x0AB2)  /* Flash data length high */
#define G_VENDOR_DATA_0AB5      XDATA_VAR8(0x0AB5)  /* Vendor data storage */
#define G_STATE_0AB6            XDATA_VAR8(0x0AB6)  /* State control 0x0AB6 */
#define G_VENDOR_DATA_0AB7      XDATA_VAR8(0x0AB7)  /* Vendor data storage 2 */
#define G_SYSTEM_STATE_0AE2     XDATA_VAR8(0x0AE2)  /* System state */
#define G_STATE_FLAG_0AE3       XDATA_VAR8(0x0AE3)  /* System state flag */
#define G_PHY_LANE_CFG_0AE4     XDATA_VAR8(0x0AE4)  /* PHY lane configuration */
#define G_TLP_INIT_FLAG_0AE5    XDATA_VAR8(0x0AE5)  /* TLP init complete flag */
#define G_LINK_SPEED_MODE_0AE6  XDATA_VAR8(0x0AE6)  /* Link speed mode */
#define G_LINK_CFG_BIT_0AE7     XDATA_VAR8(0x0AE7)  /* Link config bit (from 0x707D bit 3) */
#define G_STATE_0AE8            XDATA_VAR8(0x0AE8)  /* State control 0x0AE8 */
#define G_STATE_0AE9            XDATA_VAR8(0x0AE9)  /* State control 0x0AE9 */
#define G_FLASH_CFG_0AEA        XDATA_VAR8(0x0AEA)  /* Flash config 0x0AEA (from 0x707D bit 0) */
#define G_LINK_CFG_0AEB         XDATA_VAR8(0x0AEB)  /* Link config 0x0AEB */
#define G_PHY_CFG_0AEC          XDATA_VAR8(0x0AEC)  /* PHY config 0x0AEC */
#define G_PHY_CFG_0AED          XDATA_VAR8(0x0AED)  /* PHY config 0x0AED */
#define G_STATE_CHECK_0AEE      XDATA_VAR8(0x0AEE)  /* State check byte */
#define G_LINK_CFG_0AEF         XDATA_VAR8(0x0AEF)  /* Link config 0x0AEF */
#define G_FLASH_CFG_0AF0        XDATA_VAR8(0x0AF0)  /* Flash config 0x0AF0 */
#define G_STATE_FLAG_0AF1       XDATA_VAR8(0x0AF1)  /* State flag */
#define   STATE_FLAG_INIT         0x02  // Bit 1: Init state flag
#define   STATE_FLAG_PHY_READY    0x20  // Bit 5: PHY link ready
#define G_TRANSFER_FLAG_0AF2    XDATA_VAR8(0x0AF2)  /* Transfer flag 0x0AF2 */
#define G_EP_DISPATCH_OFFSET    XDATA_VAR8(0x0AF5)  /* Endpoint dispatch offset */
#define G_XFER_STATE_0AF6       XDATA_VAR8(0x0AF6)  /* Transfer state 0x0AF6 */
#define G_XFER_CTRL_0AF7        XDATA_VAR8(0x0AF7)  /* Transfer control 0x0AF7 */
#define G_POWER_INIT_FLAG       XDATA_VAR8(0x0AF8)  /* Power init flag (set to 0 in usb_power_init) */
#define G_TRANSFER_FLAG_0AF8    G_POWER_INIT_FLAG   /* Alias: Transfer flag for USB loop */
#define G_XFER_MODE_0AF9        XDATA_VAR8(0x0AF9)  /* Transfer mode/state: 1=mode1, 2=mode2 */
#define G_TRANSFER_PARAMS_HI    XDATA_VAR8(0x0AFA)  /* Transfer params high byte */
#define G_TRANSFER_PARAMS_LO    XDATA_VAR8(0x0AFB)  /* Transfer params low byte */
#define G_XFER_COUNT_LO         XDATA_VAR8(0x0AFC)  /* Transfer counter low byte */
#define G_XFER_COUNT_HI         XDATA_VAR8(0x0AFD)  /* Transfer counter high byte */
#define G_XFER_RETRY_CNT        XDATA_VAR8(0x0AFE)  /* Transfer retry counter */
#define G_USB_PARAM_0B00        XDATA_VAR8(0x0B00)  /* USB parameter storage */
#define G_USB_INIT_0B01         XDATA_VAR8(0x0B01)  /* USB init state flag */
#define G_PCIE_WORK_0B12        XDATA_VAR8(0x0B12)  /* PCIe work variable */
/* NOTE: G_TLP_PENDING_0B21 removed - use G_DMA_WORK_0B21 (same address) */
#define G_LINK_EVENT_0B2D       XDATA_VAR8(0x0B2D)  /* Link event flag (cleared in bda4 reset) */
#define G_USB_TRANSFER_FLAG     XDATA_VAR8(0x0B2E)  /* USB transfer flag */
#define G_INTERFACE_READY_0B2F  XDATA_VAR8(0x0B2F)  /* Interface ready flag */
#define G_PCIE_BUS_NUM_0B30     XDATA_VAR8(0x0B30)  /* PCIe bus number (for bridge config) */
#define G_PCIE_DEV_NUM_0B31     XDATA_VAR8(0x0B31)  /* PCIe device number */
#define G_PCIE_FN_NUM_0B32      XDATA_VAR8(0x0B32)  /* PCIe function number */
#define G_PCIE_CFG_OFFSET_0B33  XDATA_VAR8(0x0B33)  /* PCIe config space offset */
#define G_STATE_0B39            XDATA_VAR8(0x0B39)  /* State control 0x0B39 */
#define G_STATE_0B3A            XDATA_VAR8(0x0B3A)  /* State control 0x0B3A */
#define G_TRANSFER_BUSY_0B3B    XDATA_VAR8(0x0B3B)  /* Transfer busy flag */
#define G_STATE_CTRL_0B3C       XDATA_VAR8(0x0B3C)  /* State control 0x0B3C */
#define G_USB_STATE_0B41        XDATA_VAR8(0x0B41)  /* USB state check */
#define G_BUFFER_STATE_0AA6     XDATA_VAR8(0x0AA6)  /* Buffer state flags */
#define G_BUFFER_STATE_0AA7     XDATA_VAR8(0x0AA7)  /* Buffer state control */
#define G_STATE_CTRL_0B3E       XDATA_VAR8(0x0B3E)  /* State control 0x0B3E */
/* NOTE: G_STATE_WORK_0B3E removed - use G_STATE_CTRL_0B3E (same address) */
#define G_STATE_CTRL_0B3F       XDATA_VAR8(0x0B3F)  /* State control 0x0B3F */
#define G_DMA_ENDPOINT_0578     XDATA_VAR8(0x0578)  /* DMA endpoint control */
#define G_XFER_STATE_0AF3       XDATA_VAR8(0x0AF3)  /* Transfer state / direction (bit 7) */
#define G_XFER_STATE_0AF4        XDATA_VAR8(0x0AF4)  /* Transfer state: 0x40=SW DMA active, checked by wait fn (>= 0x20 = done) */
#define G_XFER_LUN_0AF4         G_XFER_STATE_0AF4   /* Alias: Transfer LUN (bits 0-3) */

//=============================================================================
// State Machine Work Area (0x0A80-0x0ABF)
//=============================================================================
#define G_STATE_WORK_0A85       XDATA_VAR8(0x0A85)  /* State machine temp storage */
#define G_STATE_WORK_0A86       XDATA_VAR8(0x0A86)  /* State machine counter */
#define G_STATE_WORK_0B3D       XDATA_VAR8(0x0B3D)  /* State machine flag 0B3D */
/* G_STATE_WORK_0B3E removed - use G_STATE_CTRL_0B3E */
#define G_STATE_WORK_002D       XDATA_VAR8(0x002D)  /* System work byte 0x2D */

//=============================================================================
// Flash Buffer Area (0x7000-0x7FFF) - 4 KB SPI Flash Configuration Cache
//=============================================================================
/*
 * FLASH BUFFER STRUCTURE
 *
 * This 4 KB region is loaded from SPI flash during boot and contains
 * device configuration, USB descriptors, and hardware settings.
 *
 * MEMORY MAP:
 *   0x7000-0x7003: Header (4 bytes)
 *   0x7004-0x702B: Device Configuration Block (40 bytes)
 *   0x702C-0x7058: Serial Number String (45 bytes, null-terminated)
 *   0x7059-0x705B: PD Configuration Flags (3 bytes)
 *   0x705C-0x7063: USB Mode Configuration (8 bytes)
 *   0x7064-0x7073: PCIe/NVMe Settings (16 bytes)
 *   0x7074-0x707D: Power Configuration (10 bytes)
 *   0x707E:        Header Marker (0xA5 = valid config)
 *   0x707F:        Checksum (XOR of 0x7000-0x707E)
 *   0x7080-0x7FFF: Reserved / USB Descriptor Templates
 *
 * DEVICE CONFIGURATION BLOCK (0x7004-0x702B):
 *   0x7004-0x7005: USB Vendor ID (little-endian)
 *   0x7006-0x7007: USB Product ID (little-endian)
 *   0x7008:        Device Revision
 *   0x7009-0x700A: Max Power (mA, little-endian)
 *   0x700B:        USB Attributes (self-powered, remote wakeup)
 *   0x700C-0x702B: Reserved for expansion
 *
 * PD CONFIGURATION FLAGS (0x7059-0x705B):
 *   0x7059:        PD Mode Enable (bit 0 = PD 3.0, bit 1 = EPR)
 *   0x705A:        PD Source Capabilities Index
 *   0x705B:        PD Sink Capabilities Index
 *
 * USB MODE CONFIGURATION (0x705C-0x7063):
 *   0x705C:        USB Mode (0=USB2, 1=USB3, 2=USB4)
 *   0x705D:        Lane Configuration
 *   0x705E:        Max Link Speed (0=5G, 1=10G, 2=20G, 3=40G)
 *   0x705F:        Tunnel Mode Flags
 *   0x7060-0x7063: Reserved
 *
 * PCIE/NVME SETTINGS (0x7064-0x7073):
 *   0x7064:        PCIe Link Width (1, 2, or 4 lanes)
 *   0x7065:        PCIe Gen (1=2.5G, 2=5G, 3=8G, 4=16G)
 *   0x7066:        NVMe Queue Depth
 *   0x7067:        NVMe Admin Queue Timeout (ms/10)
 *   0x7068-0x7073: Reserved
 *
 * POWER CONFIGURATION (0x7074-0x707D):
 *   0x7074:        Power Profile Index
 *   0x7075:        Voltage Reporting Mode
 *   0x7076-0x7079: Power Source PDOs (4 bytes)
 *   0x707A:        Thermal Threshold (°C)
 *   0x707B:        Fan Control Mode
 *   0x707C:        LED Mode
 *   0x707D:        Bit 0: Flash config enable, Bit 3: Link config override
 *
 * VALIDATION:
 *   - 0x707E must be 0xA5 for configuration to be valid
 *   - Checksum at 0x707F must match XOR of bytes 0x7000-0x707E
 *   - If invalid, firmware uses hardcoded defaults
 *
 * INITIALIZATION FLOW:
 *   1. flash_load_config() reads 128 bytes from SPI flash sector 0
 *   2. Validates header marker (0x707E == 0xA5) and checksum (0x707F)
 *   3. If valid, copies settings to working area (0x0860-0x08FF)
 *   4. If invalid, uses ROM defaults
 *   5. config_apply() applies settings to hardware registers
 */
#define G_FLASH_BUF_PTR         ((__xdata uint8_t *)0x7000)  /* Flash buffer base pointer */
#define G_FLASH_BUF_SIZE        0x1000              /* 4 KB flash buffer */
#define G_FLASH_BUF_BASE        XDATA_VAR8(0x7000)  /* Flash buffer byte 0 (for byte access) */

/* Header Region (0x7000-0x7003) */
#define G_FLASH_HEADER_0        XDATA_VAR8(0x7000)  /* Header byte 0 (magic 'A') */
#define G_FLASH_HEADER_1        XDATA_VAR8(0x7001)  /* Header byte 1 (magic 'S') */
#define G_FLASH_HEADER_2        XDATA_VAR8(0x7002)  /* Header byte 2 (magic 'M') */
#define G_FLASH_HEADER_3        XDATA_VAR8(0x7003)  /* Header byte 3 (version) */

/* Device Configuration Block (0x7004-0x702B) */
#define G_FLASH_CFG_START       XDATA_VAR8(0x7004)  /* Config block start */
#define G_FLASH_VID_LO          XDATA_VAR8(0x7004)  /* USB Vendor ID low */
#define G_FLASH_VID_HI          XDATA_VAR8(0x7005)  /* USB Vendor ID high */
#define G_FLASH_PID_LO          XDATA_VAR8(0x7006)  /* USB Product ID low */
#define G_FLASH_PID_HI          XDATA_VAR8(0x7007)  /* USB Product ID high */

/* Serial Number String (0x702C-0x7058) */
#define G_FLASH_SERIAL_BASE     ((__xdata uint8_t *)0x702C)  /* Serial string base */
#define G_FLASH_SERIAL_MAX_LEN  45                  /* Max serial length */

/* PD Configuration Flags (0x7059-0x705B) */
#define G_FLASH_PD_MODE         XDATA_VAR8(0x7059)  /* PD mode enable flags */
#define   FLASH_PD_MODE_30        0x01              /* Bit 0: PD 3.0 enabled */
#define   FLASH_PD_MODE_EPR       0x02              /* Bit 1: Extended Power Range */
#define G_FLASH_PD_SRC_CAP      XDATA_VAR8(0x705A)  /* PD source capabilities index */
#define G_FLASH_PD_SNK_CAP      XDATA_VAR8(0x705B)  /* PD sink capabilities index */

/* USB Mode Configuration (0x705C-0x7063) */
#define G_FLASH_USB_MODE        XDATA_VAR8(0x705C)  /* USB mode (0=USB2, 1=USB3, 2=USB4) */
#define G_FLASH_LANE_CFG        XDATA_VAR8(0x705D)  /* Lane configuration */
#define G_FLASH_LINK_SPEED      XDATA_VAR8(0x705E)  /* Max link speed */
#define G_FLASH_TUNNEL_FLAGS    XDATA_VAR8(0x705F)  /* USB4 tunnel mode flags */

/* PCIe/NVMe Settings (0x7064-0x7073) */
#define G_FLASH_PCIE_ARRAY      ((__xdata uint8_t *)0x7064)  /* PCIe settings base */
#define G_FLASH_PCIE_WIDTH      XDATA_VAR8(0x7064)  /* PCIe link width (1/2/4) */
#define G_FLASH_PCIE_GEN        XDATA_VAR8(0x7065)  /* PCIe generation (1-4) */
#define G_FLASH_NVME_QDEPTH     XDATA_VAR8(0x7066)  /* NVMe queue depth */
#define G_FLASH_NVME_TIMEOUT    XDATA_VAR8(0x7067)  /* NVMe admin timeout (ms/10) */

/* Power Configuration (0x7074-0x707D) */
#define G_FLASH_PWR_PROFILE     XDATA_VAR8(0x7074)  /* Power profile index */
#define G_FLASH_VOLT_MODE       XDATA_VAR8(0x7075)  /* Voltage reporting mode */
#define G_FLASH_PWR_PDO_BASE    ((__xdata uint8_t *)0x7076)  /* Power PDOs (4 bytes) */
#define G_FLASH_PWR_PDO_0       XDATA_VAR8(0x7076)  /* Power PDO byte 0 */
#define G_FLASH_PWR_PDO_1       XDATA_VAR8(0x7077)  /* Power PDO byte 1 */
#define G_FLASH_PWR_PDO_2       XDATA_VAR8(0x7078)  /* Power PDO byte 2 */
#define G_FLASH_PWR_PDO_3       XDATA_VAR8(0x7079)  /* Power PDO byte 3 */
#define G_FLASH_THERMAL_THRESH  XDATA_VAR8(0x707A)  /* Thermal threshold (°C) */
#define G_FLASH_FAN_MODE        XDATA_VAR8(0x707B)  /* Fan control mode */
#define G_FLASH_LED_MODE        XDATA_VAR8(0x707C)  /* LED mode */
#define G_FLASH_CFG_FLAGS       XDATA_VAR8(0x707D)  /* Configuration flags */
#define   FLASH_CFG_ENABLE        0x01              /* Bit 0: Flash config enabled */
#define   FLASH_CFG_LINK_OVRD     0x08              /* Bit 3: Link config override */

/* Validation (0x707E-0x707F) */
#define G_FLASH_MARKER          XDATA_VAR8(0x707E)  /* Header marker (0xA5 = valid) */
#define   FLASH_MARKER_VALID      0xA5              /* Valid configuration marker */
#define G_FLASH_CHECKSUM        XDATA_VAR8(0x707F)  /* XOR checksum of 0x7000-0x707E */


//=============================================================================
// Work Variables 0x0A5x-0x0A9x
//=============================================================================
#define G_EP_CFG_FLAG_0A5B      XDATA_VAR8(0x0A5B)  /* EP config flag (set to 1 by 0x99c7) */
#define G_NIBBLE_SWAP_0A5B      G_EP_CFG_FLAG_0A5B  /* Alias for nibble_swap_helper */
#define G_EP_CFG_0A5C           XDATA_VAR8(0x0A5C)  /* EP config value */
#define G_NIBBLE_SWAP_0A5C      G_EP_CFG_0A5C       /* Alias for nibble_swap_helper */
#define G_EP_CFG_0A5D           XDATA_VAR8(0x0A5D)  /* EP config value */
#define G_EP_CFG_0A5E           XDATA_VAR8(0x0A5E)  /* EP config value (cleared by 0x9741) */
#define G_EP_CFG_0A5F           XDATA_VAR8(0x0A5F)  /* EP config value (cleared by 0x9741) */
#define G_EP_CFG_0A60           XDATA_VAR8(0x0A60)  /* EP config value (cleared by 0x9741) */
#define G_LANE_STATE_0A9D       XDATA_VAR8(0x0A9D)  /* Lane state value */
#define G_LINK_POWER_STATE_0ACA XDATA_VAR8(0x0ACA)  /* Link power state (cleared in bda4 reset) */
#define G_TLP_MASK_0ACB         XDATA_VAR8(0x0ACB)  /* TLP mask value */
#define G_TLP_BLOCK_SIZE_0ACC   XDATA_VAR8(0x0ACC)  /* TLP block size (bit 1 checked in 91D1 bit 3) */
#define G_TLP_STATE_0ACF        XDATA_VAR8(0x0ACF)  /* TLP state (low 5 bits) */
#define G_TLP_CMD_STATE_0AD0    XDATA_VAR8(0x0AD0)  /* TLP command state */
#define G_LINK_STATE_0AD1       XDATA_VAR8(0x0AD1)  /* Link state counter */
#define G_USB_DESC_FLAG_0ACD    XDATA_VAR8(0x0ACD)  /* USB descriptor flag */
#define G_USB_DESC_MODE_0ACE    XDATA_VAR8(0x0ACE)  /* USB descriptor mode */
/* NOTE: G_USB_DESC_STATE_0AD7 removed - use G_TLP_COUNT_0AD7 (same address) */
/* NOTE: G_USB_DESC_INDEX_0ADE removed - use G_TLP_LIMIT_HI (same address) */
#define G_LINK_STATE_0AD2       XDATA_VAR8(0x0AD2)  /* Link state flag */
#define G_TLP_MODE_0AD3         XDATA_VAR8(0x0AD3)  /* TLP mode flag */
#define G_TLP_ADDR_OFFSET_HI    XDATA_VAR8(0x0AD5)  /* TLP address offset high */
#define G_TLP_ADDR_OFFSET_LO    XDATA_VAR8(0x0AD6)  /* TLP address offset low */
#define G_TLP_COUNT_0AD7        XDATA_VAR8(0x0AD7)  /* TLP iteration count */
#define G_TLP_TIMEOUT_HI        XDATA_VAR8(0x0AD8)  /* TLP timeout counter high */
#define G_TLP_TIMEOUT_LO        XDATA_VAR8(0x0AD9)  /* TLP timeout counter low */
#define G_TLP_TRANSFER_HI       XDATA_VAR8(0x0ADA)  /* TLP transfer address high */
#define G_TLP_TRANSFER_LO       XDATA_VAR8(0x0ADB)  /* TLP transfer address low */
#define G_TLP_COMPUTED_HI       XDATA_VAR8(0x0ADC)  /* TLP computed value high */
#define G_TLP_COMPUTED_LO       XDATA_VAR8(0x0ADD)  /* TLP computed value low */
#define G_TLP_LIMIT_HI          XDATA_VAR8(0x0ADE)  /* TLP limit/max high */
#define G_TLP_LIMIT_LO          XDATA_VAR8(0x0ADF)  /* TLP limit/max low */
#define G_TLP_BASE_HI           XDATA_VAR8(0x0AE0)  /* TLP buffer base high */
#define G_TLP_BASE_LO           XDATA_VAR8(0x0AE1)  /* TLP buffer base low */

//=============================================================================
// Timer/Init Control 0x0B40
//=============================================================================
#define G_TIMER_INIT_0B40       XDATA_VAR8(0x0B40)  /* Timer init control */
#define G_PCIE_CTRL_SAVE_0B44   XDATA_VAR8(0x0B44)  /* PCIe control saved state */

//=============================================================================
// USB/SCSI Buffer Area Control (0xD800-0xDFFF)
// Note: This is XRAM buffer area, not true MMIO registers
//=============================================================================
#define G_BUF_XFER_START        XDATA_VAR8(0xD80C)  /* Buffer transfer start */

//=============================================================================
// PD State Machine Work Area (0x07B0-0x07E0)
//=============================================================================
#define G_PD_STATE_07B4         XDATA_VAR8(0x07B4)  /* PD state byte 0 */
#define G_PD_STATE_07B5         XDATA_VAR8(0x07B5)  /* PD state byte 1 */
#define G_PD_FLAG_07B6          XDATA_VAR8(0x07B6)  /* PD flag - checked in flp print */
#define G_PD_INIT_07BA          XDATA_VAR8(0x07BA)  /* PD init flag - set to 1 in state init */
#define G_PD_STATE_07BE         XDATA_VAR8(0x07BE)  /* PD state - cleared in init */
#define G_PD_MODE_07D2          XDATA_VAR8(0x07D2)  /* PD mode - set based on E400 bit 6 */
#define G_PD_COUNTER_07DB       XDATA_VAR8(0x07DB)  /* PD counter - cleared in init */
#define G_PD_COUNTER_07DC       XDATA_VAR8(0x07DC)  /* PD counter - cleared in init */
#define G_PD_STATE_07E0         XDATA_VAR8(0x07E0)  /* PD state - cleared in init */

//=============================================================================
// Command Engine Work Area (0x07B0-0x07FF)
//=============================================================================
#define G_CMD_ENGINE_SLOT       XDATA_VAR8(0x07B7)  /* Command engine slot index (3-bit) */
#define G_CMD_PENDING_07BB      XDATA_VAR8(0x07BB)  /* Command pending flag */
/* NOTE: G_CMD_STATE_07BC removed - use G_FLASH_CMD_TYPE (same address) */
/* NOTE: G_CMD_OP_COUNTER removed - use G_FLASH_OP_COUNTER (same address) */
#define G_CMD_ADDR_HI           XDATA_VAR8(0x07BF)  /* Computed slot address high */
#define G_CMD_ADDR_LO           XDATA_VAR8(0x07C0)  /* Computed slot address low */
#define G_CMD_SLOT_C1           XDATA_VAR8(0x07C1)  /* Slot index for address calc */
#define G_CMD_WORK_C2           XDATA_VAR8(0x07C2)  /* Command work byte */
#define G_CMD_STATE             XDATA_VAR8(0x07C3)  /* Command state (3-bit) */
#define G_CMD_STATUS            XDATA_VAR8(0x07C4)  /* Command status byte */
#define G_CMD_WORK_C5           XDATA_VAR8(0x07C5)  /* Command work byte */
#define G_CMD_WORK_C7           XDATA_VAR8(0x07C7)  /* Command work byte */
#define G_CMD_MODE              XDATA_VAR8(0x07CA)  /* Command mode (1=mode1, 2=mode2, 3=mode3) */
#define G_CMD_PARAM_0           XDATA_VAR8(0x07D3)  /* Command parameter 0 */
#define G_CMD_PARAM_1           XDATA_VAR8(0x07D4)  /* Command parameter 1 */
#define G_CMD_PARAM_2           XDATA_VAR8(0x07D5)  /* Command parameter 2 (slot count) */
#define G_CMD_LBA_0             XDATA_VAR8(0x07DA)  /* Command LBA byte 0 (low) */
#define G_CMD_LBA_1             XDATA_VAR8(0x07DB)  /* Command LBA byte 1 */
#define G_CMD_LBA_2             XDATA_VAR8(0x07DC)  /* Command LBA byte 2 */
#define G_CMD_LBA_3             XDATA_VAR8(0x07DD)  /* Command LBA byte 3 (high) */
#define G_CMD_FLAG_07DE         XDATA_VAR8(0x07DE)  /* Command flag 0x07DE */
#define G_PCIE_COMPLETE_07DF    XDATA_VAR8(0x07DF)  /* PCIe link complete flag */
#define G_CMD_WORK_E3           XDATA_VAR8(0x07E3)  /* Command work byte */
#define G_CMD_DEBUG_FF          XDATA_VAR8(0x07FF)  /* Debug marker byte */

//=============================================================================
// PCIe Interrupt Handler Work Area
//=============================================================================
#define G_PCIE_LANE_STATE_0A9E  XDATA_VAR8(0x0A9E)  /* PCIe lane state */
#define G_PCIE_WORK_0B34        XDATA_VAR8(0x0B34)  /* PCIe work byte */
#define G_PCIE_STATUS_0B35      XDATA_VAR8(0x0B35)  /* PCIe status work byte */
#define G_PCIE_STATUS_0B36      XDATA_VAR8(0x0B36)  /* PCIe status work byte */
#define G_PCIE_STATUS_0B37      XDATA_VAR8(0x0B37)  /* PCIe status work byte */
#define G_PCIE_STATUS_0B13      XDATA_VAR8(0x0B13)  /* PCIe status work */
#define G_PCIE_STATUS_0B14      XDATA_VAR8(0x0B14)  /* PCIe status work */
#define G_PCIE_STATUS_0B15      XDATA_VAR8(0x0B15)  /* PCIe status work */
#define G_PCIE_STATUS_0B16      XDATA_VAR8(0x0B16)  /* PCIe status work */
#define G_PCIE_STATUS_0B17      XDATA_VAR8(0x0B17)  /* PCIe status work */
#define G_PCIE_STATUS_0B18      XDATA_VAR8(0x0B18)  /* PCIe status work */
#define G_PCIE_STATUS_0B19      XDATA_VAR8(0x0B19)  /* PCIe status flag */
#define G_PCIE_STATUS_0B1A      XDATA_VAR8(0x0B1A)  /* PCIe status work */
#define G_STATE_0B1B            XDATA_VAR8(0x0B1B)  /* State variable for protocol dispatch */
#define G_PCIE_STATUS_0B1C      XDATA_VAR8(0x0B1C)  /* PCIe status work */
#define G_DMA_WORK_0B1D         XDATA_VAR8(0x0B1D)  /* DMA work byte (r4) */
#define G_DMA_WORK_0B1E         XDATA_VAR8(0x0B1E)  /* DMA work byte (r5) */
#define G_DMA_WORK_0B1F         XDATA_VAR8(0x0B1F)  /* DMA work byte (r6) */
#define G_DMA_WORK_0B20         XDATA_VAR8(0x0B20)  /* DMA work byte (r7) */
#define G_DMA_WORK_0B21         XDATA_VAR8(0x0B21)  /* DMA work byte */
#define G_DMA_WORK_0B24         XDATA_VAR8(0x0B24)  /* DMA work byte */
#define G_DMA_WORK_0B25         XDATA_VAR8(0x0B25)  /* DMA work byte */

//=============================================================================
// Power State Machine Work Area (0x0A60-0x0A6F)
//=============================================================================
#define G_POWER_STATE_MAX_0A61  XDATA_VAR8(0x0A61)  /* Power state iteration max */
#define G_POWER_STATE_IDX_0A62  XDATA_VAR8(0x0A62)  /* Power state iteration index */

//=============================================================================
// Clean Firmware Debug/PCIe State (0x0F00-0x0F4F)
// Used by clean firmware for PCIe link training and diagnostic state.
//=============================================================================
#define G_PCIE_DEBUG_BASE       0x0F00               /* Base address for debug state */
#define G_PCIE_LTSSM_STATE      XDATA_VAR8(0x0F00)  /* LTSSM state snapshot */
#define G_PCIE_PHY_STATE        XDATA_VAR8(0x0F01)  /* PHY state snapshot */
#define G_PCIE_LINK_WIDTH       XDATA_VAR8(0x0F02)  /* Link width (from B22B) */
#define G_PCIE_CFG_STATUS_0     XDATA_VAR8(0x0F0C)  /* PCIe config status bytes */
#define G_PCIE_CFG_STATUS_1     XDATA_VAR8(0x0F0D)
#define G_PCIE_CFG_STATUS_2     XDATA_VAR8(0x0F0E)
#define G_PCIE_CFG_STATUS_3     XDATA_VAR8(0x0F0F)
#define G_PCIE_TRAIN_STATE_0    XDATA_VAR8(0x0F10)  /* PCIe training state */
#define G_PCIE_TRAIN_STATE_1    XDATA_VAR8(0x0F11)
#define G_PCIE_TRAIN_STATE_2    XDATA_VAR8(0x0F12)
#define G_PCIE_DIAG_STATUS      XDATA_VAR8(0x0F14)  /* Diagnostic status byte */
#define G_PCIE_LANE_STATE_BASE  XDATA_VAR8(0x0F16)  /* Per-lane state base */
#define G_PCIE_LANE_STATE_1     XDATA_VAR8(0x0F17)
#define G_PCIE_LANE_STATE_2     XDATA_VAR8(0x0F18)
#define G_PCIE_LANE_STATE_3     XDATA_VAR8(0x0F19)
#define G_PCIE_LANE_STATE_4     XDATA_VAR8(0x0F1A)
#define G_PCIE_LANE_STATE_5     XDATA_VAR8(0x0F1B)
#define G_PCIE_LANE_STATE_6     XDATA_VAR8(0x0F1C)
#define G_PCIE_LANE_STATE_7     XDATA_VAR8(0x0F1D)
#define G_PCIE_BRIDGE_BUS_PRI   XDATA_VAR8(0x0F20)  /* Bridge primary bus */
#define G_PCIE_BRIDGE_BUS_SEC   XDATA_VAR8(0x0F21)  /* Bridge secondary bus */
#define G_PCIE_BRIDGE_BUS_SUB   XDATA_VAR8(0x0F22)  /* Bridge subordinate bus */
#define G_PCIE_LINK_OK          XDATA_VAR8(0x0F23)  /* Link training OK flag */
#define G_PCIE_GPU_VID_LO       XDATA_VAR8(0x0F24)  /* GPU vendor ID low */
#define G_PCIE_GPU_VID_HI       XDATA_VAR8(0x0F25)  /* GPU vendor ID high */
#define G_PCIE_GPU_DID_LO       XDATA_VAR8(0x0F26)  /* GPU device ID low */
#define G_PCIE_GPU_DID_HI       XDATA_VAR8(0x0F27)  /* GPU device ID high */
#define G_PCIE_BRIDGE_REG_28    XDATA_VAR8(0x0F28)  /* Bridge config reg 0x28 */
#define G_PCIE_BRIDGE_REG_29    XDATA_VAR8(0x0F29)  /* Bridge config reg 0x29 */
#define G_PCIE_BRIDGE_REG_2A    XDATA_VAR8(0x0F2A)  /* Bridge config reg 0x2A */
#define G_PCIE_PHY_E762         XDATA_VAR8(0x0F30)  /* PHY RXPLL status snapshot */
#define G_PCIE_PHY_E765         XDATA_VAR8(0x0F31)  /* PHY E765 snapshot */
#define G_PCIE_PHY_E764         XDATA_VAR8(0x0F32)  /* PHY timer ctrl snapshot */
#define G_PCIE_PHY_FLAGS        XDATA_VAR8(0x0F33)  /* PHY flags */
#define G_PCIE_TLP_HEADER_0     XDATA_VAR8(0x0F40)  /* TLP header byte 0 */
#define G_PCIE_TLP_HEADER_1     XDATA_VAR8(0x0F41)  /* TLP header byte 1 */
#define G_PCIE_TLP_HEADER_2     XDATA_VAR8(0x0F42)  /* TLP header byte 2 */
#define G_PCIE_TLP_HEADER_3     XDATA_VAR8(0x0F43)  /* TLP header byte 3 */

#endif /* __GLOBALS_H__ */
