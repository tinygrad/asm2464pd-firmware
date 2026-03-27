#ifndef __REGISTERS_H__
#define __REGISTERS_H__

#include "types.h"

/*
 * ASM2464PD USB4/Thunderbolt NVMe Controller - Hardware Register Map
 *
 * All registers are memory-mapped in XDATA space.
 * Organized by functional block in address order.
 *
 * Address Space Layout:
 *   0x7000-0x7FFF  USB bulk OUT landing buffer (read-only to CPU, written by HW)
 *   0x8000-0x8FFF  Writable XDATA (NOT aliased with 0xF000)
 *   0x9000-0x93FF  USB Interface
 *   0xA000-0xAFFF  Writable XDATA
 *   0xB000-0xB1FF  NVMe Admin Queues
 *   0xB200-0xB4FF  PCIe Passthrough / Tunnel
 *   0xC000-0xC0FF  UART Controller
 *   0xC200-0xC2FF  Link/PHY Control
 *   0xC400-0xC5FF  NVMe / MSC Interface
 *   0xC600-0xC6FF  PHY Extended
 *   0xC800-0xC8FF  Interrupt / I2C / Flash / DMA
 *   0xCA00-0xCAFF  CPU Mode
 *   0xCC00-0xCCFF  Timer / CPU Control
 *   0xCE00-0xCEFF  SCSI DMA / Transfer Control
 *   0xD800-0xDFFF  USB Endpoint Buffer (MSC data/CSW)
 *   0xE300-0xE3FF  PHY Completion / Debug
 *   0xE400-0xE4FF  Command Engine
 *   0xE600-0xE6FF  Debug/Interrupt
 *   0xE700-0xE7FF  System Status / Link Control
 *   0xEC00-0xECFF  NVMe Event
 *   0xEF00-0xEFFF  System Control
 *   0xF000-0xFFFF  Writable XDATA (NOT aliased with 0x8000)
 *
 * DMA Paths for USB Data:
 *   EP0 Control (0x9092): descriptor/control transfers via 0x9E00 buffer
 *   Bulk IN (0xC42C): MSC engine, data at 0xD800+, works on USB 3.0
 *   Bulk IN (0x90A1): NVMe DMA bridge, needs C471=0x01 (NVMe device)
 *   Both bulk paths need PCIe link (B480=0x01) for USB 2.0
 */

//=============================================================================
// Helper Macros
//=============================================================================
#define XDATA_REG8(addr)   (*(__xdata uint8_t *)(addr))
#define XDATA_REG16(addr)  (*(__xdata uint16_t *)(addr))
#define XDATA_REG32(addr)  (*(__xdata uint32_t *)(addr))
#define XDATA_REG8V(addr)  (*(__xdata volatile uint8_t *)(addr))
#define XDATA_REG16V(addr) (*(__xdata volatile uint16_t *)(addr))
#define XDATA_REG32V(addr) (*(__xdata volatile uint32_t *)(addr))

//=============================================================================
// Memory Buffers
//=============================================================================
#define FLASH_BUFFER_BASE       0x7000
#define FLASH_BUFFER_SIZE       0x1000
/*
 * USB bulk OUT landing buffer (0x7000-0x7FFF)
 * Read-only to the 8051 CPU (E5 writes are ignored).
 * Written by USB hardware when bulk OUT data arrives at EP 0x02.
 * The CE00 DMA engine reads from here to move data to SRAM.
 */
#define REG_FLASH_BUF_BYTE(off) XDATA_REG8(FLASH_BUFFER_BASE + (off))

// Flash buffer control registers (0x7041, 0x78AF-0x78B2)
#define REG_FLASH_BUF_CTRL_7041 XDATA_REG8(0x7041)  /* Flash buffer control */
#define   FLASH_BUF_CTRL_BIT6    0x40  // Bit 6: Buffer control enable
#define REG_FLASH_BUF_CFG_78AF  XDATA_REG8(0x78AF)  /* Flash buffer config 0 */
#define REG_FLASH_BUF_CFG_78B0  XDATA_REG8(0x78B0)  /* Flash buffer config 1 */
#define REG_FLASH_BUF_CFG_78B1  XDATA_REG8(0x78B1)  /* Flash buffer config 2 */
#define REG_FLASH_BUF_CFG_78B2  XDATA_REG8(0x78B2)  /* Flash buffer config 3 */
#define   FLASH_BUF_CFG_BIT6     0x40  // Bit 6: Buffer config enable

#define USB_SCSI_BUF_BASE       0x8000
#define USB_SCSI_BUF_SIZE       0x1000

// USB/SCSI buffer control registers (0x8005-0x800D)
#define REG_USB_BUF_COUNT_8005  XDATA_REG8(0x8005)  /* USB buffer count */
#define REG_USB_BUF_MAX_8006    XDATA_REG8(0x8006)  /* USB buffer max count */
#define REG_USB_BUF_CTRL_8008   XDATA_REG8(0x8008)  /* USB buffer control (power check: ==0x01) */
#define REG_USB_BUF_CTRL_8009   XDATA_REG8(0x8009)  /* USB buffer control (power check: ==0x08) */
#define REG_USB_BUF_CTRL_800A   XDATA_REG8(0x800A)  /* USB buffer control (power check: ==0x02) */
#define REG_USB_BUF_STATUS_800D XDATA_REG8(0x800D)  /* USB buffer status (mask 0x7F != 0 check) */

/*
 * USB Setup Packet Buffer (0x9E00-0x9E07)
 * Hardware writes the 8-byte USB setup packet here when received.
 * Firmware reads these registers in ISR at 0xA5EA-0xA604 to process request.
 *
 * Standard USB Setup Packet Format:
 *   Byte 0 (bmRequestType): Request characteristics
 *     Bit 7: Direction (0=Host-to-device, 1=Device-to-host)
 *     Bits 6-5: Type (0=Standard, 1=Class, 2=Vendor)
 *     Bits 4-0: Recipient (0=Device, 1=Interface, 2=Endpoint)
 *   Byte 1 (bRequest): Specific request code
 *     0x00=GET_STATUS, 0x01=CLEAR_FEATURE, 0x05=SET_ADDRESS
 *     0x06=GET_DESCRIPTOR, 0x09=SET_CONFIGURATION
 *   Bytes 2-3 (wValue): Request-specific value
 *   Bytes 4-5 (wIndex): Request-specific index
 *   Bytes 6-7 (wLength): Number of bytes to transfer
 */
#define USB_CTRL_BUF_BASE       0x9E00
#define USB_CTRL_BUF_SIZE       0x0200

// USB Setup Packet Registers
#define REG_USB_SETUP_TYPE      XDATA_REG8(0x9E00)  /* bmRequestType (direction/type/recipient) */
#define REG_USB_SETUP_REQUEST   XDATA_REG8(0x9E01)  /* bRequest (request code) */
#define REG_USB_SETUP_VALUE_L   XDATA_REG8(0x9E02)  /* wValue low byte (descriptor index) */
#define REG_USB_SETUP_VALUE_H   XDATA_REG8(0x9E03)  /* wValue high byte (descriptor type) */
#define REG_USB_SETUP_INDEX_L   XDATA_REG8(0x9E04)  /* wIndex low byte */
#define REG_USB_SETUP_INDEX_H   XDATA_REG8(0x9E05)  /* wIndex high byte */
#define REG_USB_SETUP_LENGTH_L  XDATA_REG8(0x9E06)  /* wLength low byte */
#define REG_USB_SETUP_LENGTH_H  XDATA_REG8(0x9E07)  /* wLength high byte */

// bmRequestType bit definitions
#define   USB_SETUP_DIR_HOST_TO_DEV  0x00  // Direction: Host to Device
#define   USB_SETUP_DIR_DEV_TO_HOST  0x80  // Direction: Device to Host
#define   USB_SETUP_TYPE_STANDARD    0x00  // Type: Standard request
#define   USB_SETUP_TYPE_CLASS       0x20  // Type: Class request
#define   USB_SETUP_TYPE_VENDOR      0x40  // Type: Vendor request
#define   USB_SETUP_RECIP_DEVICE     0x00  // Recipient: Device
#define   USB_SETUP_RECIP_INTERFACE  0x01  // Recipient: Interface
#define   USB_SETUP_RECIP_ENDPOINT   0x02  // Recipient: Endpoint

// Standard bRequest codes
#define   USB_REQ_GET_STATUS         0x00
#define   USB_REQ_CLEAR_FEATURE      0x01
#define   USB_REQ_SET_FEATURE        0x03
#define   USB_REQ_SET_ADDRESS        0x05
#define   USB_REQ_GET_DESCRIPTOR     0x06
#define   USB_REQ_SET_DESCRIPTOR     0x07
#define   USB_REQ_GET_CONFIGURATION  0x08
#define   USB_REQ_SET_CONFIGURATION  0x09
#define   USB_REQ_GET_INTERFACE      0x0A
#define   USB_REQ_SET_INTERFACE      0x0B
#define   USB_REQ_SET_SEL            0x30  // USB 3.0: Set System Exit Latency
#define   USB_REQ_SET_ISOCH_DELAY    0x31  // USB 3.0: Set Isochronous Delay

// Descriptor types (for wValue high byte in GET_DESCRIPTOR)
#define   USB_DESC_TYPE_DEVICE       0x01
#define   USB_DESC_TYPE_CONFIG       0x02
#define   USB_DESC_TYPE_STRING       0x03
#define   USB_DESC_TYPE_INTERFACE    0x04
#define   USB_DESC_TYPE_ENDPOINT     0x05
#define   USB_DESC_TYPE_BOS          0x0F  // Binary Object Store (USB 3.0)

// Additional USB control buffer registers
#define REG_USB_CTRL_BUF_9E16   XDATA_REG8(0x9E16)  /* USB control buffer descriptor 1 hi */
#define REG_USB_CTRL_BUF_9E17   XDATA_REG8(0x9E17)  /* USB control buffer descriptor 1 lo */
#define REG_USB_CTRL_BUF_9E1D   XDATA_REG8(0x9E1D)  /* USB control buffer descriptor 2 hi */
#define REG_USB_CTRL_BUF_9E1E   XDATA_REG8(0x9E1E)  /* USB control buffer descriptor 2 lo */

#define NVME_IOSQ_BASE          0xA000
#define NVME_IOSQ_SIZE          0x1000
#define NVME_IOSQ_DMA_ADDR      0x00820000

#define NVME_ASQ_BASE           0xB000
#define NVME_ASQ_SIZE           0x0100
#define NVME_ACQ_BASE           0xB100
#define NVME_ACQ_SIZE           0x0100

#define NVME_DATA_BUF_BASE      0xF000
#define NVME_DATA_BUF_SIZE      0x1000
#define NVME_DATA_BUF_DMA_ADDR  0x00200000
/*
 * XDATA 0xF000-0xFFFF: Writable XDATA region (4KB).
 * NOT aliased with 0x8000 — probing confirms they are independent.
 * In the stock NVMe flow, the CE00 DMA engine deposits data here from
 * the SRAM at PCI 0x200000, but the mapping requires MSC mode active.
 * tinygrad uses address 0xF000 as the base for its SRAM copy buffer.
 */
#define REG_NVME_DATA_BYTE(off) XDATA_REG8(NVME_DATA_BUF_BASE + (off))

//=============================================================================
// USB Interface Registers (0x9000-0x93FF)
//=============================================================================
/*
 * USB Controller Overview:
 * The USB controller handles USB 2.0 and USB 3.0 (SuperSpeed) connections.
 *
 * USB State Machine (IDATA[0x6A]):
 *   0 = DISCONNECTED  - No USB connection
 *   1 = ATTACHED      - Cable connected
 *   2 = POWERED       - Bus powered
 *   3 = DEFAULT       - Default address assigned
 *   4 = ADDRESS       - Device address assigned
 *   5 = CONFIGURED    - Ready for vendor commands
 *
 * Key MMIO registers for USB:
 *   0x9000: Connection status (bit 7=connected, bit 0=active)
 *   0x9091: Control transfer phase (bit 0=setup, bit 1=data)
 *   0x9092: DMA trigger for descriptor transfers
 *   0x9101: Interrupt flags (bit 5 triggers command handler)
 *   0x9E00-0x9E07: USB setup packet buffer
 *   0xCE89: USB/DMA status (state machine control)
 */

/*
 * Core USB Status (0x9000) — READ-ONLY hardware status register
 *
 * WARNING: Writing to this register BREAKS DMA transfers!
 * All 0x9000 writes have been removed from CBW handlers.
 *
 * Bit 0: Data path status — set by hardware when bulk OUT data is
 *        in USB FIFO awaiting SW/NVMe DMA extraction.
 *        Stock firmware checks this at 0x3427: if set → call dispatch_0206.
 *        If clear → data went elsewhere (e.g., flash controller DMA).
 *
 * Read-modify-writeback is OK during init (SET_CONFIG, do_bulk_init)
 * but NEVER write specific values during runtime transfer handling.
 */
#define REG_USB_STATUS          XDATA_REG8(0x9000)
#define   USB_STATUS_DMA_READY    0x01  // Bit 0: Data in FIFO for DMA (read-only)
#define   USB_STATUS_BIT2         0x04  // Bit 2: USB status flag
#define   USB_STATUS_INDICATOR    0x10  // Bit 4: USB status indicator
#define   USB_STATUS_CONNECTED    0x80  // Bit 7: USB cable connected
#define REG_USB_CONTROL         XDATA_REG8(0x9001)
#define REG_USB_CONFIG          XDATA_REG8(0x9002)
#define   USB_CONFIG_MASK        0x0F  // Bits 0-3: USB configuration value
#define   USB_CONFIG_BIT1        0x02  // Bit 1: Must be CLEAR to reach 0x9091 check at 0xCDF5
#define   USB_CONFIG_MSC_INIT    0xE0  // MSC engine init value (stock 0xB203; partially volatile)
#define REG_USB_EP0_STATUS      XDATA_REG8(0x9003)
#define REG_USB_EP0_LEN_L       XDATA_REG8(0x9004)  /* EP0 transfer length low byte */
#define REG_USB_EP0_LEN_H       XDATA_REG8(0x9005)  /* EP0 transfer length high byte */
/*
 * USB EP0 Config / Bulk Ready (0x9006)
 * Dual purpose: EP0 config during enumeration, bulk ready during transfers.
 * In CBW handler: set bit 0 (0x01) and bit 7 (0x80) after receiving CBW
 * to indicate endpoint is ready for data transfer.
 */
#define REG_USB_EP0_CONFIG      XDATA_REG8(0x9006)
#define   USB_EP0_CONFIG_ENABLE   0x01  // Bit 0: EP0 config / bulk endpoint ready
#define   USB_EP0_CONFIG_READY    0x80  // Bit 7: EP0 ready / data transfer ready
#define REG_USB_SCSI_BUF_LEN    XDATA_REG16(0x9007)
#define REG_USB_SCSI_BUF_LEN_L  XDATA_REG8(0x9007)
#define REG_USB_SCSI_BUF_LEN_H  XDATA_REG8(0x9008)
/*
 * USB MSC Config (0x900B)
 * Controls MSC (Mass Storage Class) engine state.
 * Used in 900B/C42A doorbell dance for bulk IN transfers.
 *
 * Doorbell dance sequence (before C42C trigger):
 *   Ramp up:  900B |= 0x02, |= 0x04, then C42A |= 0x01, 900B |= 0x01, C42A ramp
 *   Teardown: 900B &= ~0x02, &= ~0x04, C42A &= ~0x01, 900B &= ~0x01, C42A teardown
 *
 * Also toggled during SET_INTERFACE and bus reset:
 *   Phase 5: |= 0x02, |= 0x04, &= ~0x02, &= ~0x04 (MSC toggle)
 *   Phase 9: |= 0x01, &= ~0x01 (MSC bit 0 toggle)
 *   Phase 11: full ramp up + teardown (re-arm MSC engine)
 */
#define REG_USB_MSC_CFG         XDATA_REG8(0x900B)
#define REG_USB_ALT_SETTING_L   XDATA_REG8(0x900C)  /* Alt setting wValue low (written by SET_INTERFACE) */
#define REG_USB_ALT_SETTING_H   XDATA_REG8(0x900D)  /* Alt setting wValue high */
#define REG_USB_ALT_SETTING2_L  XDATA_REG8(0x900E)  /* Alt setting wValue low (duplicate) */
#define REG_USB_ALT_SETTING2_H  XDATA_REG8(0x900F)  /* Alt setting wValue high (duplicate) */
#define REG_USB_DATA_L          XDATA_REG8(0x9010)
#define REG_USB_DATA_H          XDATA_REG8(0x9011)
#define REG_USB_FIFO_STATUS     XDATA_REG8(0x9012)  /* USB FIFO/status register */
#define   USB_FIFO_STATUS_READY   0x01  // Bit 0: USB ready/active
#define REG_USB_FIFO_H          XDATA_REG8(0x9013)
/* 0x9014-0x9017: FIFO/status registers, cleared to 0x00 in Phase 2 */
#define REG_USB_FIFO_4          XDATA_REG8(0x9014)  /* USB FIFO reg 4 */
#define REG_USB_FIFO_5          XDATA_REG8(0x9015)  /* USB FIFO reg 5 */
#define REG_USB_FIFO_6          XDATA_REG8(0x9016)  /* USB FIFO reg 6 */
#define REG_USB_FIFO_7          XDATA_REG8(0x9017)  /* USB FIFO reg 7 */
#define REG_USB_XCVR_MODE       XDATA_REG8(0x9018)
#define REG_USB_MODE_VAL_9019   XDATA_REG8(0x9019)
/*
 * USB MSC Transfer Length (0x901A)
 * Controls transfer size for C42C, 90A1, and 90E1 paths.
 *
 * Default: 0x0D (13 bytes for CSW). Restore after SW DMA transfers.
 *
 * For C42C path: always 0x0D (13-byte CSW, hardware uses internal buffer).
 * For SW DMA IN (C8D4=0xA0 + 90A1): set to actual data length (max 255).
 *   90A1 sends this many bytes from 905B:905C address to host.
 * For SW DMA OUT: NOT explicitly set by stock firmware's dispatch_0206.
 *   May not control receive length (hardware uses CBW transfer length).
 * For NVMe DMA: set by dispatch_0206 NVMe path at 0x02E4.
 */
#define REG_USB_MSC_LENGTH      XDATA_REG8(0x901A)

// USB endpoint registers (0x905A-0x90FF)
/*
 * Software DMA EP config (0x905A)
 * Written before 90E1 trigger in dispatch_0206 software path.
 * Sets the direction for the SW DMA engine.
 * 0x10 = bulk IN (DMA from XDATA to USB host)
 * 0x08 = bulk OUT (DMA from USB FIFO to XDATA)
 */
#define REG_USB_EP_CFG_905A     XDATA_REG8(0x905A)
#define   USB_EP_CFG_BULK_IN      0x10  // Bulk IN (send data to host)
#define   USB_EP_CFG_BULK_OUT     0x08  // Bulk OUT (receive data from host)
/*
 * Software DMA buffer address (0x905B:905C)
 * Source address for bulk IN, dest address for bulk OUT.
 * Set from XDATA[0x0056:0x0057] in dispatch_0206 software path.
 */
#define REG_USB_EP_BUF_HI       XDATA_REG8(0x905B)
#define REG_USB_EP_BUF_LO       XDATA_REG8(0x905C)
#define REG_USB_EP_CTRL_905D    XDATA_REG8(0x905D)  /* USB endpoint control 1 */
#define REG_USB_EP_MGMT         XDATA_REG8(0x905E)
#define REG_USB_EP_CTRL_905F    XDATA_REG8(0x905F)  /* USB endpoint control 2 */
#define   USB_EP_CTRL_905F_BIT3   0x08  // Bit 3: Endpoint enable flag
#define   USB_EP_CTRL_905F_BIT4   0x10  // Bit 4: Endpoint control flag
#define REG_USB_INT_MASK_9090   XDATA_REG8(0x9090)  /* USB interrupt mask */
#define   USB_INT_MASK_GLOBAL     0x80  // Bit 7: Global interrupt mask

/*
 * USB Control Transfer Phase Register (0x9091)
 *
 * Controls USB control transfer state machine. Read to check current phase,
 * write to acknowledge/advance phase.
 *
 * Control Transfer Sequence (verified working):
 *
 * 1. SETUP Phase:
 *    - Hardware sets bit 0 when SETUP packet received
 *    - Read 0x9002, write back; read 0x9220
 *    - Write 0x01 to acknowledge setup phase
 *    - Read setup packet from 0x9104-0x910B
 *
 * 2. DATA Phase (GET_DESCRIPTOR):
 *    - Poll until bit 3 (0x08) is set = data phase ready
 *    - Write descriptor to 0x9E00+ buffer
 *    - Set length: 0x9003=0, 0x9004=len
 *    - Write 0x9092=0x04 to trigger DMA send
 *    - Poll 0x9092 until 0 (DMA complete)
 *    - Write 0x08 to acknowledge data phase
 *
 * 3. STATUS Phase:
 *    - Poll until bit 4 (0x10) is set = status phase ready
 *    - Write 0x9092=0x08 to complete status
 *    - Write 0x10 to acknowledge status phase
 *
 * For no-data requests (SET_ADDRESS, SET_CONFIG):
 *    - Skip data phase, go directly to status phase
 */
/*
 * Original firmware dispatcher (fw.bin:0xCDE7):
 *   bit0 set → LCALL 0xA5A6 (setup init),     ACK: write 0x01
 *   bit1 set → LCALL 0xD088 (status OUT),      ACK: write 0x02
 *   bit2 set → LCALL 0xDCD5 (OUT data recv),   ACK: write 0x04
 *   bit3 set → LCALL 0xB286 (IN data/GET_DESC),ACK: write 0x08
 *   bit4 set → LCALL 0xB612 (STATUS/SET_ADDR), ACK: write 0x10
 *
 * Init: hw_init writes 0x1F (all bits set = clear all pending).
 */
#define REG_USB_CTRL_PHASE      XDATA_REG8(0x9091)
#define   USB_CTRL_PHASE_SETUP    0x01  // Bit 0: Setup packet received
#define   USB_CTRL_PHASE_STAT_OUT 0x02  // Bit 1: Status phase (OUT/host-to-device)
#define   USB_CTRL_PHASE_DATA_OUT 0x04  // Bit 2: OUT data received from host
#define   USB_CTRL_PHASE_DATA_IN  0x08  // Bit 3: IN data phase ready (poll for GET_DESC)
#define   USB_CTRL_PHASE_STAT_IN  0x10  // Bit 4: Status phase (IN/device-to-host, SET_ADDR)

/*
 * USB EP0 DMA Control Register (0x9092)
 *
 * Controls DMA transfers for EP0 control transfers.
 * Write to trigger operations, reads back 0 when complete.
 *
 * Values:
 *   0x01 = Stall/reset endpoint
 *   0x02 = Receive from host (OUT) / USB 2.0 status phase ACK
 *          Used in complete_usb20_status() for IN transfer status phase.
 *   0x04 = Start DMA send (IN transfer - device to host)
 *          Used for GET_DESCRIPTOR data phase.
 *          Set 0x9003=0, 0x9004=length first, then write 0x04.
 *          Poll until reads 0 for completion.
 *   0x08 = Complete status phase
 *          Write after status phase ready (0x9091 & 0x10).
 *          For IN transfers: host sends ZLP, we ACK.
 *          For OUT transfers: we send ZLP.
 */
#define REG_USB_DMA_TRIGGER     XDATA_REG8(0x9092)
#define   USB_DMA_STALL           0x01  // Stall/reset endpoint
#define   USB_DMA_RECV            0x02  // Receive from host (OUT) / status ACK
#define   USB_DMA_SEND            0x04  // Trigger DMA send (IN data phase)
#define   USB_DMA_STATUS_COMPLETE 0x08  // Complete status phase

/*
 * USB Endpoint Config 1 (0x9093)
 * Controls endpoint transfer mode and triggers bulk operations.
 *
 * Read: current endpoint state (ISR checks bits to dispatch handlers)
 *   Bit 0: Calls cleanup (0x54b4 -> 0x3169, 0x320d)
 *   Bit 1: Bulk data complete - calls 0x32e4 (BULK_DATA handler)
 *   Bit 2: Calls cleanup (same as bit 0)
 *   Bit 3: State machine handler - calls 0x4d3e
 *
 * Write: arm/ack endpoint
 *   Arm bulk IN:  9093=0x08, 9094=0x02 (at 0x1CCA from main loop)
 *   Arm bulk OUT: 9093=0x02, 9094=0x10 (at 0x1CD5/0x32A6 from main loop)
 *     CRITICAL: Must be set from main loop, NOT from within CBW handler!
 *     The MSC engine needs the CBW handler to return before it transitions
 *     to the data-ready state. Arming inside handle_cbw → BULK_DATA never fires.
 *   Ack bulk done: 9093=0x02 (at 0x0FAC, after BULK_DATA fires in ISR)
 */
#define REG_USB_EP_CFG1         XDATA_REG8(0x9093)
#define   USB_EP_CFG1_CLEANUP     0x01  // Bit 0: Cleanup pending
#define   USB_EP_CFG1_BULK_DONE   0x02  // Bit 1: Bulk data complete / ack
#define   USB_EP_CFG1_CLEANUP2    0x04  // Bit 2: Cleanup pending (alt)
#define   USB_EP_CFG1_STATE_MACH  0x08  // Bit 3: State machine event
#define   USB_EP_CFG1_ARM_IN      0x08  // Write: Arm bulk IN endpoint
#define   USB_EP_CFG1_ARM_OUT     0x02  // Write: Arm bulk OUT endpoint / ack

#define   USB_EP_CFG1_BULK_OUT_START     0x01
#define   USB_EP_CFG1_BULK_OUT_COMPLETE  0x02
#define   USB_EP_CFG1_BULK_IN_COMPLETE   0x04
#define   USB_EP_CFG1_BULK_IN_START      0x08

#define REG_USB_EP_CFG2         XDATA_REG8(0x9094)
#define   USB_EP_CFG2_CLEAR_IN    0x01
#define   USB_EP_CFG2_ARM_IN      0x02  // Bit 1: Arm bulk IN endpoint (write)
#define   USB_EP_CFG2_CLEAR_OUT   0x08
#define   USB_EP_CFG2_ARM_OUT     0x10  // Bit 4: Arm bulk OUT endpoint (write)
/*
 * USB Endpoint Ready/Status Masks (0x9096-0x909E)
 * These 9 registers (0x9096-0x909E) control endpoint ready state.
 * All set to 0xFF during Phase 2 clears and hw_init.
 * 0x909E set to 0x03 after clears.
 *
 * In EP completion handler (9101 & 0x20):
 *   Read 9118 (EP status), loop while non-zero:
 *     Read 9096 (EP ready), write back to acknowledge.
 */
#define REG_USB_EP_READY        XDATA_REG8(0x9096)  /* EP ready (read/writeback to ack) */
#define REG_USB_EP_CTRL_9097    XDATA_REG8(0x9097)  /* EP control (init: 0xFF) */
#define REG_USB_EP_MODE_9098    XDATA_REG8(0x9098)  /* EP mode (init: 0xFF) */
#define REG_USB_EP_MODE_9099    XDATA_REG8(0x9099)  /* EP mode 2 (init: 0xFF) */
#define REG_USB_EP_MODE_909A    XDATA_REG8(0x909A)  /* EP mode 3 (init: 0xFF) */
#define REG_USB_EP_MODE_909B    XDATA_REG8(0x909B)  /* EP mode 4 (init: 0xFF) */
#define REG_USB_EP_MODE_909C    XDATA_REG8(0x909C)  /* EP mode 5 (init: 0xFF) */
#define REG_USB_EP_MODE_909D    XDATA_REG8(0x909D)  /* EP mode 6 (init: 0xFF) */
#define REG_USB_STATUS_909E     XDATA_REG8(0x909E)  /* EP status (init: 0x03) */
#define REG_USB_CTRL_90A0       XDATA_REG8(0x90A0)  /* Bulk strobe (auto-clears after write) */
/*
 * Bulk DMA Trigger (0x90A1)
 * Write 0x01 to trigger bulk data transfer.
 *
 * Behavior depends on C8D4 (DMA config):
 *   C8D4=0x00 (NVMe mode): sends from NVMe DMA registers. 901A=0x0D for CSW.
 *     REQUIRES: C471=0x01 (NVMe queue busy), B480=0x01 (PCIe link up).
 *   C8D4=0xA0 (SW DMA mode): sends from 905B:905C address. 901A=data_len.
 *     Proven working for bulk IN: 90E1 sets up source, 90A1 triggers send.
 *     For bulk OUT: stock firmware does NOT write 90A1 in dispatch_0206
 *       SW DMA path — only 90E1 is used. 90A1 is written at 0x3261 inside
 *       the CSW send function (0x3258), AFTER data has been received.
 *
 * EP completion fires 9101 bit 5 (0x20) when host completes the read.
 * On success: reads back 0x00 (consumed). On failure: stays 0x01.
 */
#define REG_USB_BULK_DMA_TRIGGER XDATA_REG8(0x90A1)
#define REG_USB_SPEED           XDATA_REG8(0x90E0)
#define   USB_SPEED_MASK         0x03  // Bits 0-1: USB speed mode
/*
 * Software DMA Trigger (0x90E1)
 * Used by dispatch_0206 software path to set up SW DMA transfer.
 * Write 0x01 to trigger (via 0x3288: writes 0x01, clears C509 bit 0).
 *
 * For bulk IN: set 905A=0x10 first. 90E1 sets up DMA source at 905B:905C.
 *   Does NOT send data by itself — 90A1=0x01 is needed to actually trigger.
 * For bulk OUT: set 905A=0x08 first. 90E1 should set up DMA destination.
 *   Stock firmware does NOT write 90A1 after 90E1 for OUT direction.
 *   Data should move from USB FIFO to XDATA[905B:905C] address.
 *
 * Prerequisites before writing 90E1:
 *   C8D4 = 0xA0 (SW DMA mode enabled)
 *   905B:905C = DMA buffer address (source for IN, dest for OUT)
 *   D800 = 0x03 (IN) or 0x04 (OUT) — direction
 *   D802:D803 = same address as 905B:905C
 *   905A = 0x10 (IN) or 0x08 (OUT) — EP config
 *   C509 bit 0 set (pre-trigger, cleared by 0x3288 after 90E1 write)
 */
#define REG_USB_SW_DMA_TRIGGER  XDATA_REG8(0x90E1)
/*
 * MSC Engine Gate Register (0x90E2)
 * CRITICAL for C42C bulk IN to work. Acts as a gate in the stock ISR:
 *   - Stock ISR checks 90E2 bit 0 before processing CBW_RECEIVED (9101 bit 6)
 *   - If 90E2=0, ISR exits without processing CBW (first pass)
 *   - 90E2 gets set to 0x01 via the EP_COMPLETE (bit 5) → 9096 path
 *   - On second ISR entry, 90E2=1, CBW processing proceeds
 *
 * Written to 0x01 during:
 *   - MSC engine init at 0xB20C (stock 0xB1C5 function)
 *   - ISR CBW second-pass entry at 0x1023/0x100D
 *
 * Volatile: does NOT retain value after hardware processing.
 * Must be written at the right time relative to C42C and hardware state.
 * Reading it back after C42C or DMA operations may return 0x00.
 */
#define REG_USB_MODE            XDATA_REG8(0x90E2)
#define REG_USB_EP_STATUS_90E3  XDATA_REG8(0x90E3)

/*
 * USB Link Status and Speed Registers (0x9100-0x912F)
 * These indicate current USB connection state and speed mode.
 */
#define REG_USB_LINK_STATUS     XDATA_REG8(0x9100)
#define   USB_LINK_STATUS_MASK    0x03  // Bits 0-1: USB speed mode
#define   USB_SPEED_FULL          0x00  // Full Speed (USB 1.x, 12 Mbps)
#define   USB_SPEED_HIGH          0x01  // High Speed (USB 2.0, 480 Mbps)
#define   USB_SPEED_SUPER         0x02  // SuperSpeed (USB 3.0, 5 Gbps)
#define   USB_SPEED_SUPER_PLUS    0x03  // SuperSpeed+ (USB 3.1+, 10+ Gbps)

/*
 * USB Peripheral Status (0x9101) — READ-ONLY STATUS REGISTER
 *
 * IMPORTANT: These bits REFLECT CURRENT STATE, they are NOT write-1-to-clear
 * event flags. The bits stay asserted as long as the condition exists.
 * This means if the ISR returns with bits still set (e.g., incomplete control
 * transfer during libusb_close), the bits remain asserted and will re-trigger
 * on the next interrupt (if using level-triggered INT0).
 *
 * Stock firmware ISR at 0x0e33 reads this register and dispatches:
 *   Bit 5 (EP_COMPLETE) → checks 9118 for EP status (table dispatch)
 *   Bit 3 (BULK_REQ) → reads 9301/9302 for bulk request handling
 *   Bit 0 (without bit 1) → bus reset handler at 0x0ef4
 *   Bit 0 (BUS_RESET) → 91D1 dispatch at 0x0f4a
 *   Bit 4 (LINK_EVENT) → 9300 link status check
 *
 * Verified bit meanings from firmware and trace analysis:
 *   0x01 = Bus reset / 91D1 events pending (when NOT combined with 0x02)
 *   0x02 = Setup/control packet received (EP0)
 *   0x04 = Bulk OUT data available in FIFO
 *   0x08 = Bulk transfer request (9301/9302 handling)
 *   0x10 = Link event (USB 3.0→2.0 transition, cable state)
 *   0x20 = Bulk EP completion (IN transfer done)
 *   0x40 = CBW received (bulk OUT, SCSI command ready at 0x912A+)
 */
#define REG_USB_PERIPH_STATUS   XDATA_REG8(0x9101)
#define   USB_PERIPH_91D1_EVENT   0x01  // Bit 0: 91D1 event pending (link train, power mgmt, flag, reset)
#define   USB_PERIPH_BUS_RESET    USB_PERIPH_91D1_EVENT  // Legacy alias
#define   USB_PERIPH_CONTROL      0x02  // Bit 1: Setup/control packet (EP0)
#define   USB_PERIPH_BULK_DATA    0x04  // Bit 2: Bulk OUT data available
#define   USB_PERIPH_BULK_REQ     0x08  // Bit 3: Bulk transfer request (USB 3.0)
#define   USB_PERIPH_LINK_EVENT   0x10  // Bit 4: Link event (speed change, cable)
#define   USB_PERIPH_EP_COMPLETE  0x20  // Bit 5: Bulk EP completion (IN done)
#define   USB_PERIPH_CBW_RECEIVED 0x40  // Bit 6: CBW received (bulk OUT ready)

/*
 * USB Setup Packet Registers (0x9104-0x910B)
 * Hardware writes the 8-byte USB setup packet here when received.
 * Read at PC=0xA5F5 in ISR during control transfer setup phase.
 * Different from 0x9E00-0x9E07 which is the response buffer.
 *
 * Standard USB Setup Packet Format:
 *   Byte 0 (bmRequestType): Direction(1) | Type(2) | Recipient(5)
 *   Byte 1 (bRequest): Request code (GET_DESCRIPTOR=0x06, SET_ADDRESS=0x05, etc.)
 *   Bytes 2-3 (wValue): Request-specific (descriptor type/index for GET_DESCRIPTOR)
 *   Bytes 4-5 (wIndex): Request-specific (language ID for string descriptors)
 *   Bytes 6-7 (wLength): Number of bytes to transfer
 */
#define REG_USB_SETUP_BMREQ     XDATA_REG8(0x9104)  /* bmRequestType */
#define REG_USB_SETUP_BREQ      XDATA_REG8(0x9105)  /* bRequest */
#define REG_USB_SETUP_WVAL_L    XDATA_REG8(0x9106)  /* wValue low (descriptor index) */
#define REG_USB_SETUP_WVAL_H    XDATA_REG8(0x9107)  /* wValue high (descriptor type) */
#define REG_USB_SETUP_WIDX_L    XDATA_REG8(0x9108)  /* wIndex low */
#define REG_USB_SETUP_WIDX_H    XDATA_REG8(0x9109)  /* wIndex high */
#define REG_USB_SETUP_WLEN_L    XDATA_REG8(0x910A)  /* wLength low */
#define REG_USB_SETUP_WLEN_H    XDATA_REG8(0x910B)  /* wLength high */


#define REG_USB_STAT_EXT_L      XDATA_REG8(0x910D)
#define REG_USB_STAT_EXT_H      XDATA_REG8(0x910E)
/*
 * USB Endpoint Status / CBW Registers (0x9118-0x912E)
 *
 * When a CBW is received (9101 & 0x40), hardware populates these registers
 * with the parsed CBW fields. The firmware reads them to dispatch SCSI commands.
 *
 * CBW Register Layout (verified by firmware testing):
 *   0x9118     EP status (non-zero = endpoints active, read in EP completion)
 *   0x9119     CBW data transfer length high byte (read in CBW handler)
 *   0x911A     CBW data transfer length low byte
 *   0x911B-911E  dCBWSignature "USBC" (0x55,0x53,0x42,0x43)
 *   0x911F-9122  dCBWTag (4 bytes, echoed back in CSW)
 *   0x9123-9126  dCBWDataTransferLength (4 bytes, LE)
 *   0x9127       bmCBWFlags (bit 7 = data direction: 1=IN, 0=OUT)
 *   0x9128       bCBWLUN (bits 0-3)
 *   0x9129       bCBWCBLength (not typically read)
 *   0x912A       CBWCB[0] = SCSI opcode
 *   0x912B       CBWCB[1] = param (size for E4, value for E5)
 *   0x912C       CBWCB[2] = addr>>16 (E4/E5 vendor cmds)
 *   0x912D       CBWCB[3] = addr high byte
 *   0x912E       CBWCB[4] = addr low byte
 *
 * Vendor SCSI commands (tinygrad):
 *   0xE4 (read reg):  CDB=[0xE4, size, addr>>16, addr_hi, addr_lo, 0] → bulk IN
 *   0xE5 (write reg): CDB=[0xE5, value, addr>>16, addr_hi, addr_lo, 0] → no data
 *
 * Standard SCSI commands handled:
 *   0x00 TEST UNIT READY, 0x03 REQUEST SENSE, 0x12 INQUIRY,
 *   0x1A MODE SENSE(6), 0x1E PREVENT ALLOW, 0x25 READ CAPACITY(10),
 *   0x28 READ(10)
 */
#define REG_USB_EP_STATUS       XDATA_REG8(0x9118)
#define REG_USB_CBW_LEN_HI      XDATA_REG8(0x9119)  /* CBW data xfer length high (read in handler) */
#define REG_USB_CBW_LEN_LO      XDATA_REG8(0x911A)  /* CBW data xfer length low */
#define REG_USB_CBW_SIG0        XDATA_REG8(0x911B)  /* dCBWSignature[0] = 'U' (0x55) */
#define REG_USB_CBW_SIG1        XDATA_REG8(0x911C)  /* dCBWSignature[1] = 'S' (0x53) */
#define REG_USB_CBW_SIG2        XDATA_REG8(0x911D)  /* dCBWSignature[2] = 'B' (0x42) */
#define REG_USB_CBW_SIG3        XDATA_REG8(0x911E)  /* dCBWSignature[3] = 'C' (0x43) */
#define REG_CBW_TAG_0           XDATA_REG8(0x911F)  /* dCBWTag byte 0 (echo in CSW) */
#define REG_CBW_TAG_1           XDATA_REG8(0x9120)  /* dCBWTag byte 1 */
#define REG_CBW_TAG_2           XDATA_REG8(0x9121)  /* dCBWTag byte 2 */
#define REG_CBW_TAG_3           XDATA_REG8(0x9122)  /* dCBWTag byte 3 */
#define REG_USB_CBW_XFER_LEN_0  XDATA_REG8(0x9123)  /* dCBWDataTransferLength byte 0 (LSB) */
#define REG_USB_CBW_XFER_LEN_1  XDATA_REG8(0x9124)  /* dCBWDataTransferLength byte 1 */
#define REG_USB_CBW_XFER_LEN_2  XDATA_REG8(0x9125)  /* dCBWDataTransferLength byte 2 */
#define REG_USB_CBW_XFER_LEN_3  XDATA_REG8(0x9126)  /* dCBWDataTransferLength byte 3 (MSB) */
#define REG_USB_CBW_FLAGS       XDATA_REG8(0x9127)  /* bmCBWFlags (bit 7 = direction) */
#define   CBW_FLAGS_DIRECTION     0x80  // Bit 7: Data direction (1=IN, 0=OUT)
#define REG_USB_CBW_LUN         XDATA_REG8(0x9128)  /* bCBWLUN (bits 0-3) */
#define   CBW_LUN_MASK            0x0F  // Bits 0-3: Logical Unit Number
#define REG_USB_CBW_CB_LEN      XDATA_REG8(0x9129)  /* bCBWCBLength */
#define REG_USB_CBWCB_0         XDATA_REG8(0x912A)  /* CBWCB[0] = SCSI opcode */
#define REG_USB_CBWCB_1         XDATA_REG8(0x912B)  /* CBWCB[1] = param/size/value */
#define REG_USB_CBWCB_2         XDATA_REG8(0x912C)  /* CBWCB[2] = addr>>16 */
#define REG_USB_CBWCB_3         XDATA_REG8(0x912D)  /* CBWCB[3] = addr high */
#define REG_USB_CBWCB_4         XDATA_REG8(0x912E)  /* CBWCB[4] = addr low */
#define REG_USB_CBWCB_5         XDATA_REG8(0x912F)  /* CBWCB[5] = reserved */
#define REG_USB_CBWCB_6         XDATA_REG8(0x9130)  /* CBWCB[6] */
#define REG_USB_CBWCB_7         XDATA_REG8(0x9131)  /* CBWCB[7] */
#define REG_USB_CBWCB_8         XDATA_REG8(0x9132)  /* CBWCB[8] */
#define REG_USB_CBWCB_9         XDATA_REG8(0x9133)  /* CBWCB[9] */
#define REG_USB_CBWCB_10        XDATA_REG8(0x9134)  /* CBWCB[10] = WRITE16 sector count MSB */
#define REG_USB_CBWCB_11        XDATA_REG8(0x9135)  /* CBWCB[11] */
#define REG_USB_CBWCB_12        XDATA_REG8(0x9136)  /* CBWCB[12] */
#define REG_USB_CBWCB_13        XDATA_REG8(0x9137)  /* CBWCB[13] = WRITE16 sector count LSB */

// USB Endpoint Control (0x9220)
#define REG_USB_EP_CTRL_9220    XDATA_REG8(0x9220)  /* EP control (read in setup phase) */

// USB PHY registers (0x91C0-0x91FF)
/*
 * USB PHY Status (0x91C0) — Link state indicators
 *   Bit 1: Link up indicator. Checked during 91D1 bit 0 recovery handler
 *          at 0xc465. If clear → link is down → writes E710 and clears CC3B bit 1.
 */
#define REG_USB_PHY_CTRL_91C0   XDATA_REG8(0x91C0)
#define   USB_PHY_91C0_LINK_UP    0x02  // Bit 1: SS link up (checked in 91D1 bit 0 handler)
#define REG_USB_PHY_CTRL_91C1   XDATA_REG8(0x91C1)
#define REG_USB_PHY_CTRL_91C3   XDATA_REG8(0x91C3)
#define REG_USB_EP_CTRL_91D0    XDATA_REG8(0x91D0)
/*
 * USB SS Link Event Register (0x91D1) — WRITE-1-TO-CLEAR
 *
 * Each bit signals a link-layer event. Stock firmware ISR at 0x0f4a
 * dispatches on individual bits in priority order: bit 3 > 0 > 1 > 2.
 * Ack each bit by writing its mask back (write-1-to-clear).
 * Bit 2 handler is called BEFORE ack; all others AFTER ack.
 *
 * Without handling these events, the SS link dies after 30-75s idle.
 * Init: hw_init writes 0x0F to clear all pending events.
 *
 * Stock firmware dispatch addresses:
 *   Bit 3 → 0x9b95 (power management / U1/U2 transitions)
 *   Bit 0 → 0xc465 (link training/recovery, calls bda4 state reset)
 *   Bit 1 → 0xe6aa (simple: sets 0x0A7D=0, 0x0B2E=1)
 *   Bit 2 → 0xe682 (link reset ack: C6A8|=1, clears 0x0B2E, 0x07E8)
 */
#define REG_USB_PHY_CTRL_91D1   XDATA_REG8(0x91D1)
#define   USB_91D1_LINK_TRAIN     0x01  // Bit 0: Link training/recovery (→ bda4 state reset)
#define   USB_91D1_FLAG           0x02  // Bit 1: Link flag (sets G_EP_DISPATCH_VAL3=0, G_USB_TRANSFER_FLAG=1)
#define   USB_91D1_LINK_RESET     0x04  // Bit 2: Link reset ack (C6A8|=1, clears flags)
#define   USB_91D1_POWER_MGMT     0x08  // Bit 3: Power management U1/U2 entry (CC3B&=~2, TLP_BASE_LO=1)
#define   USB_91D1_ALL            0x0F  // All event bits (for init clear)


// USB control registers (0x9200-0x92BF)
#define REG_USB_CTRL_9200       XDATA_REG8(0x9200)  /* USB control base */
#define   USB_CTRL_9200_BIT6     0x40  // Bit 6: USB control enable flag
#define REG_USB_CTRL_9201       XDATA_REG8(0x9201)
#define   USB_CTRL_9201_BIT4      0x10  // Bit 4: USB control flag
/*
 * USB Address Control (0x9202)
 * Read-modify-writeback during SET_ADDRESS on USB 3.0.
 * Part of the address programming register dance.
 */
#define REG_USB_ADDR_CTRL       XDATA_REG8(0x9202)
/*
 * USB Address Config Registers (0x9206-0x920B)
 * Written during USB 3.0 SET_ADDRESS to program device address.
 * Sequence: 9206/9207 = 0x03 → 0x07 → readback, then 9208-920B params.
 */
#define REG_USB_ADDR_CFG_A      XDATA_REG8(0x9206)  /* Address config A (0x03→0x07→readback) */
#define REG_USB_ADDR_CFG_B      XDATA_REG8(0x9207)  /* Address config B (0x03→0x07→readback) */
#define REG_USB_ADDR_PARAM_0    XDATA_REG8(0x9208)  /* Address param 0 (written 0x00) */
#define REG_USB_ADDR_PARAM_1    XDATA_REG8(0x9209)  /* Address param 1 (written 0x0A) */
#define REG_USB_ADDR_PARAM_2    XDATA_REG8(0x920A)  /* Address param 2 (written 0x00) */
#define REG_USB_ADDR_PARAM_3    XDATA_REG8(0x920B)  /* Address param 3 (written 0x0A) */
#define REG_USB_CTRL_920C       XDATA_REG8(0x920C)
#define REG_USB_PHY_CONFIG_9241 XDATA_REG8(0x9241)
#define REG_USB_CTRL_924C       XDATA_REG8(0x924C)  // USB control (bit 0: endpoint ready)

/*
 * Power Management Registers (0x92C0-0x92E0)
 * Control power domains, clocks, and device power state.
 *
 * REG_POWER_STATUS (0x92C2):
 *   Bit 6: PCIe PHY power domain enable.
 *     Stock firmware D92E sets this BEFORE USB enumeration (92C2 |= 0x40).
 *     ISR clears it when power event fires (92C2 &= 0x3F).
 *     Both stock and custom firmware read 0x07 when stable (bit 6 cleared).
 *     The brief pulse activates the PHY power domain which stays latched.
 *   Bit 7: Additional power status, cleared by ISR along with bit 6.
 *   Controls ISR vs main loop execution paths for USB:
 *     When CLEAR: ISR calls 0xBDA4 for descriptor init
 *     When SET: Main loop calls 0x0322 for transfer
 *
 * REG_POWER_EVENT_92E1 (0x92E1):
 *   Power event status, write-to-clear. ISR reads and acks this.
 *   Stock trained value: 0x20 (bit 5 set = pending event).
 */
#define REG_POWER_ENABLE        XDATA_REG8(0x92C0)
#define   POWER_ENABLE_BIT        0x01  // Bit 0: Main power enable
#define   POWER_ENABLE_MAIN       0x80  // Bit 7: Main power on (set during MSC init at 0xB1C5)
/*
 * Clock Enable / PHY Clock Control (0x92C1)
 * Bit 4 toggled (set then clear) in full 91D1 bit 3 handler during
 * PHY clock recovery after 92CF sequence.
 */
#define REG_CLOCK_ENABLE        XDATA_REG8(0x92C1)
#define   CLOCK_ENABLE_BIT        0x01  // Bit 0: Clock enable
#define   CLOCK_ENABLE_BIT1       0x02  // Bit 1: Secondary clock
#define   CLOCK_ENABLE_PHY_TOGGLE 0x10  // Bit 4: PHY clock toggle (in 91D1 bit 3 handler)
#define REG_POWER_STATUS        XDATA_REG8(0x92C2)
#define   POWER_STATUS_READY      0x02  // Bit 1: Power ready
#define   POWER_STATUS_USB_PATH   0x40  // Bit 6: Controls ISR/main loop USB path
#define REG_POWER_MISC_CTRL     XDATA_REG8(0x92C4)
#define REG_PHY_POWER           XDATA_REG8(0x92C5)
#define   PHY_POWER_ENABLE        0x04  // Bit 2: PHY power enable
#define REG_POWER_CTRL_92C6     XDATA_REG8(0x92C6)
#define REG_POWER_CTRL_92C7     XDATA_REG8(0x92C7)
/*
 * Power/PHY Control (0x92C8) — Link training PHY config
 *   Bits 0-1 cleared in bda4 state reset (91D1 bit 0 handler).
 *   Written 0x24 during hw_init.
 *   Stock sequence at 0xc465: 92C8 &= ~0x01; 92C8 &= ~0x02.
 */
#define REG_POWER_CTRL_92C8     XDATA_REG8(0x92C8)
#define   POWER_CTRL_92C8_BIT0    0x01  // Bit 0: Cleared in link training recovery
#define   POWER_CTRL_92C8_BIT1    0x02  // Bit 1: Cleared in link training recovery
#define REG_POWER_DOMAIN        XDATA_REG8(0x92E0)
#define   POWER_DOMAIN_BIT1       0x02  // Bit 1: Power domain control
#define REG_POWER_EVENT_92E1    XDATA_REG8(0x92E1)  // Power event register
/*
 * Clock Recovery Control (0x92CF)
 * Used in full 91D1 bit 3 handler (0x9b95) for PHY clock recovery
 * during U1/U2 power state transitions. Sequence: 0x00→0x04→0x07→0x03.
 * Also involves 92C1 bit 4 toggle and E712 polling for completion.
 */
#define REG_CLOCK_CTRL_92CF     XDATA_REG8(0x92CF)
#define REG_POWER_STATUS_92F7   XDATA_REG8(0x92F7)  // Power status (high nibble = state)
#define REG_POWER_STATUS_92F8   XDATA_REG8(0x92F8)  /* Power status (read in reset/SET_ADDRESS) */
/*
 * Link Status Poll (0x92FB)
 * Read in full 91D1 bit 3 handler. Value 0x01 triggers link recovery
 * sequence with timeout loops. Part of U1/U2 power management.
 */
#define REG_POWER_POLL_92FB     XDATA_REG8(0x92FB)

/*
 * Buffer Config Registers (0x9300-0x9305)
 * Configure MSC/bulk DMA buffer parameters.
 *
 * Stock MSC engine init (0xB1C5) writes:
 *   9300 = 0x0C  (volatile — doesn't retain after HW processing)
 *   9301 = 0xC0  (volatile)
 *   9302 = 0xBF  (volatile)
 *   9303 = 0x33  (retains value)
 *   9304 = 0x3F  (retains value)
 *   9305 = 0x40  (retains value)
 *
 * 9300-9302 are also used as link event status registers by the ISR.
 */
#define REG_BUF_CFG_9300        XDATA_REG8(0x9300)
#define   BUF_CFG_9300_SS_OK      0x02  // Bit 1: USB 3.0 link established (empirically verified)
#define   BUF_CFG_9300_SS_FAIL    0x04  // Bit 2: USB 3.0 link failed, fall back to 2.0
#define   BUF_CFG_9300_SS_EVENT   0x08  // Bit 3: SS link event (checked at 0x10b2 after 91D1 dispatch)
#define   BUF_CFG_9300_MSC_INIT   0x0C  // MSC engine init value (stock 0xB1D7)
#define REG_BUF_CFG_9301        XDATA_REG8(0x9301)
#define   BUF_CFG_9301_BIT6      0x40  // Bit 6: Buffer config flag
#define   BUF_CFG_9301_BIT7      0x80  // Bit 7: Buffer config flag
#define   BUF_CFG_9301_MSC_INIT  0xC0  // MSC engine init value (stock 0xB1DB)
#define REG_BUF_CFG_9302        XDATA_REG8(0x9302)
#define   BUF_CFG_9302_BIT7      0x80  // Bit 7: Buffer status flag
#define   BUF_CFG_9302_MSC_INIT  0xBF  // MSC engine init value (stock 0xB1DF)
#define REG_BUF_CFG_9303        XDATA_REG8(0x9303)  /* MSC init: 0x33 (retains) */
#define REG_BUF_CFG_9304        XDATA_REG8(0x9304)  /* MSC init: 0x3F (retains) */
#define REG_BUF_CFG_9305        XDATA_REG8(0x9305)  /* MSC init: 0x40 (retains) */

/*
 * Buffer Descriptor Table (0x9310-0x9323)
 * Configures DMA buffer regions for USB/NVMe data transfers.
 * Written during hw_init to set up buffer base addresses and sizes.
 */
#define REG_BUF_DESC_BASE0_HI   XDATA_REG8(0x9310)  /* Buffer 0 base address high */
#define REG_BUF_DESC_BASE0_LO   XDATA_REG8(0x9311)  /* Buffer 0 base address low */
#define REG_BUF_DESC_SIZE0_HI   XDATA_REG8(0x9312)  /* Buffer 0 size high */
#define REG_BUF_DESC_SIZE0_LO   XDATA_REG8(0x9313)  /* Buffer 0 size low */
#define REG_BUF_DESC_BASE1_HI   XDATA_REG8(0x9314)  /* Buffer 1 base address high */
#define REG_BUF_DESC_BASE1_LO   XDATA_REG8(0x9315)  /* Buffer 1 base address low */
#define REG_BUF_DESC_STAT0_HI   XDATA_REG8(0x9316)  /* Buffer status 0 high */
#define REG_BUF_DESC_STAT0_LO   XDATA_REG8(0x9317)  /* Buffer status 0 low */
#define REG_BUF_DESC_BASE2_HI   XDATA_REG8(0x9318)  /* Buffer 2 base address high */
#define REG_BUF_DESC_BASE2_LO   XDATA_REG8(0x9319)  /* Buffer 2 base address low */
#define REG_BUF_DESC_STAT1_HI   XDATA_REG8(0x931A)  /* Buffer status 1 high */
#define REG_BUF_DESC_STAT1_LO   XDATA_REG8(0x931B)  /* Buffer status 1 low */
#define REG_BUF_DESC_CFG0_HI    XDATA_REG8(0x931C)  /* Buffer config 0 high */
#define REG_BUF_DESC_CFG0_LO    XDATA_REG8(0x931D)  /* Buffer config 0 low */
#define REG_BUF_DESC_CFG1_HI    XDATA_REG8(0x931E)  /* Buffer config 1 high */
#define REG_BUF_DESC_CFG1_LO    XDATA_REG8(0x931F)  /* Buffer config 1 low */
#define REG_BUF_DESC_CFG2_HI    XDATA_REG8(0x9320)  /* Buffer config 2 high */
#define REG_BUF_DESC_CFG2_LO    XDATA_REG8(0x9321)  /* Buffer config 2 low */
#define REG_BUF_DESC_STAT2_HI   XDATA_REG8(0x9322)  /* Buffer status 2 high */
#define REG_BUF_DESC_STAT2_LO   XDATA_REG8(0x9323)  /* Buffer status 2 low */

//=============================================================================
// PCIe Passthrough Registers (0xB210-0xB8FF)
//=============================================================================

// PCIe extended register access (0x12xx banked -> 0xB2xx XDATA)
#define PCIE_EXT_REG(offset)  XDATA_REG8(0xB200 + (offset))

// PCIe TLP registers (0xB210-0xB284)
/* Raw byte access to B210-B21B request header window (12 bytes). */
#define REG_PCIE_TLP_BYTE(off)  XDATA_REG8V(0xB210 + (off))
#define REG_PCIE_FMT_TYPE       XDATA_REG8V(0xB210)
#define REG_PCIE_TLP_CTRL       XDATA_REG8V(0xB213)
#define REG_PCIE_TLP_LENGTH     XDATA_REG8V(0xB216)
#define REG_PCIE_BYTE_EN        XDATA_REG8V(0xB217)
#define   PCIE_BYTE_EN_DWORD      0x0F  /* Enable all 4 byte lanes */
#define REG_PCIE_ADDR_0         XDATA_REG8V(0xB218)
#define REG_PCIE_ADDR_1         XDATA_REG8V(0xB219)
#define REG_PCIE_ADDR_2         XDATA_REG8V(0xB21A)
#define REG_PCIE_ADDR_3         XDATA_REG8V(0xB21B)
#define REG_PCIE_ADDR_HIGH      XDATA_REG8V(0xB21C)
#define REG_PCIE_ADDR_HIGH_1    XDATA_REG8V(0xB21D)  // Upper address byte 1 (64-bit addressing)
#define REG_PCIE_ADDR_HIGH_2    XDATA_REG8V(0xB21E)  // Upper address byte 2 (64-bit addressing)
#define REG_PCIE_ADDR_HIGH_3    XDATA_REG8V(0xB21F)  // Upper address byte 3 (64-bit addressing)
#define REG_PCIE_DATA           XDATA_REG8V(0xB220)
#define REG_PCIE_DATA_0         XDATA_REG8V(0xB220)
#define REG_PCIE_DATA_1         XDATA_REG8V(0xB221)  // Data register byte 1
#define REG_PCIE_DATA_2         XDATA_REG8V(0xB222)  // Data register byte 2
#define REG_PCIE_DATA_3         XDATA_REG8V(0xB223)  // Data register byte 3
/*
 * PCIe Extended Status (0xB223)
 * Bit 0: PLL lock / CDR lock indicator.
 * After phy_rst_rxpll_core returns 0 (success), stock firmware checks
 * bit 0 of B223 — if set, CDR lock confirmed (returns 0x13 = "[CDRV ok]").
 */
#define REG_PCIE_EXT_STATUS     XDATA_REG8V(0xB223)
#define   PCIE_EXT_STATUS_PLL_LOCK 0x01  // Bit 0: PLL/CDR lock confirmed
#define REG_PCIE_TLP_CPL_HEADER XDATA_REG32(0xB224)
#define REG_PCIE_LINK_STATUS    XDATA_REG16V(0xB22A)
#define REG_PCIE_LINK_STATUS_8  XDATA_REG8V(0xB22A)
#define REG_PCIE_CPL_STATUS     XDATA_REG8V(0xB22B)
#define REG_PCIE_CPL_DATA       XDATA_REG8V(0xB22C)
#define REG_PCIE_CPL_DATA_ALT   XDATA_REG8V(0xB22D)

// PCIe Extended Link Registers (0xB234-0xB24E)
#define REG_PCIE_LINK_STATE_EXT XDATA_REG8(0xB234)   // Extended link state machine state
#define REG_PCIE_LINK_CFG       XDATA_REG8(0xB235)   // Link configuration (bits 6-7 kept on reset)
#define REG_PCIE_LINK_PARAM     XDATA_REG8(0xB236)   // Link parameter
#define REG_PCIE_LINK_STATUS_EXT XDATA_REG8(0xB237)  // Extended link status (bit 7 = active)
#define REG_PCIE_LINK_TRIGGER   XDATA_REG8(0xB238)   // Link trigger (bit 0 = busy)
#define   PCIE_LINK_TRIGGER_BUSY  0x01  // Bit 0: Link trigger busy
#define REG_PCIE_EXT_CFG_0      XDATA_REG8(0xB23C)   // Extended config 0
#define REG_PCIE_EXT_CFG_1      XDATA_REG8(0xB23D)   // Extended config 1
#define REG_PCIE_EXT_CFG_2      XDATA_REG8(0xB23E)   // Extended config 2
#define REG_PCIE_EXT_CFG_3      XDATA_REG8(0xB23F)   // Extended config 3
#define REG_PCIE_EXT_STATUS_RD  XDATA_REG8(0xB240)   // Extended status read
#define REG_PCIE_EXT_STATUS_RD1 XDATA_REG8(0xB241)   // Extended status read 1
#define REG_PCIE_EXT_STATUS_RD2 XDATA_REG8(0xB242)   // Extended status read 2
#define REG_PCIE_EXT_STATUS_RD3 XDATA_REG8(0xB243)   // Extended status read 3
#define REG_PCIE_EXT_STATUS_ALT XDATA_REG8(0xB24E)   // Extended status alternate

// PCIe DMA Config (0xB250-0xB281)
#define REG_PCIE_NVME_DOORBELL  XDATA_REG32(0xB250)
#define REG_PCIE_DMA_CFG_50     XDATA_REG8(0xB250)   // DMA config byte 0
#define REG_PCIE_DMA_CFG_51     XDATA_REG8(0xB251)   // DMA config byte 1
#define REG_PCIE_DOORBELL_CMD   XDATA_REG8(0xB251)   // Byte 1 of doorbell - command byte
#define REG_PCIE_TRIGGER        XDATA_REG8V(0xB254)
#define   PCIE_TRIGGER_EXEC       0x0F  /* Trigger PCIe request execution */
#define REG_PCIE_DMA_SIZE_A     XDATA_REG8(0xB264)   // DMA size config A
#define REG_PCIE_DMA_SIZE_B     XDATA_REG8(0xB265)   // DMA size config B
#define REG_PCIE_DMA_SIZE_C     XDATA_REG8(0xB266)   // DMA size config C
#define REG_PCIE_DMA_SIZE_D     XDATA_REG8(0xB267)   // DMA size config D
#define REG_PCIE_DMA_BUF_A      XDATA_REG8(0xB26C)   // DMA buffer config A
#define REG_PCIE_DMA_BUF_B      XDATA_REG8(0xB26D)   // DMA buffer config B
#define REG_PCIE_DMA_BUF_C      XDATA_REG8(0xB26E)   // DMA buffer config C
#define REG_PCIE_DMA_BUF_D      XDATA_REG8(0xB26F)   // DMA buffer config D
#define REG_PCIE_DMA_CTRL_B281  XDATA_REG8(0xB281)   // DMA control
#define REG_PCIE_PM_ENTER       XDATA_REG8(0xB255)
#define REG_PCIE_COMPL_STATUS   XDATA_REG8(0xB284)
#define REG_PCIE_POWER_B294     XDATA_REG8(0xB294)  /* PCIe power control */
// PCIe status registers (0xB296-0xB298)
#define REG_PCIE_STATUS         XDATA_REG8V(0xB296)
#define REG_PCIE_BRIDGE_CTRL    XDATA_REG8(0xB297)  /* PCIe bridge control (bit 0 = enable) */
#define   PCIE_STATUS_ERROR       0x01  // Bit 0: Error flag
#define   PCIE_STATUS_COMPLETE    0x02  // Bit 1: Completion status
#define   PCIE_STATUS_BUSY        0x04  // Bit 2: Busy flag
#define   PCIE_STATUS_KICK        0x04  /* Command kick/write strobe value */
#define   PCIE_STATUS_RESET       0x08  /* Request engine reset/re-arm value */
#define REG_PCIE_TUNNEL_CFG     XDATA_REG8(0xB298)  // TLP control (bit 4 = tunnel enable)
#define   PCIE_TLP_CTRL_TUNNEL    0x10  // Bit 4: Tunnel enable
#define REG_PCIE_CTRL_B2D5      XDATA_REG8(0xB2D5)  /* PCIe control */

// PCIe Tunnel Control (0xB401-0xB404)
#define REG_PCIE_TUNNEL_CTRL    XDATA_REG8(0xB401)  // PCIe tunnel control
#define   PCIE_TUNNEL_ENABLE      0x01  // Bit 0: Tunnel enable
#define REG_PCIE_CTRL_B402      XDATA_REG8(0xB402)
#define   PCIE_CTRL_B402_BIT0     0x01  // Bit 0: Control flag 0
#define   PCIE_CTRL_B402_BIT1     0x02  // Bit 1: Control flag 1
#define REG_PCIE_LINK_PARAM_B404 XDATA_REG8(0xB404) // PCIe link parameters
#define   PCIE_LINK_PARAM_MASK    0x0F  // Bits 0-3: Link parameters

// PCIe Tunnel Adapter Configuration (0xB410-0xB42B)
// These registers configure the USB4 PCIe tunnel adapter path
#define REG_TUNNEL_CFG_A_LO     XDATA_REG8(0xB410)  // Tunnel config A low (from 0x0A53)
#define REG_TUNNEL_CFG_A_HI     XDATA_REG8(0xB411)  // Tunnel config A high (from 0x0A52)
#define REG_TUNNEL_CREDITS      XDATA_REG8(0xB412)  // Tunnel credits (from 0x0A55)
#define REG_TUNNEL_CFG_MODE     XDATA_REG8(0xB413)  // Tunnel mode config (from 0x0A54)
#define REG_TUNNEL_CAP_0        XDATA_REG8(0xB415)  // Tunnel capability 0 (fixed 0x06)
#define REG_TUNNEL_CAP_1        XDATA_REG8(0xB416)  // Tunnel capability 1 (fixed 0x04)
#define REG_TUNNEL_CAP_2        XDATA_REG8(0xB417)  // Tunnel capability 2 (fixed 0x00)
#define REG_TUNNEL_PATH_CREDITS XDATA_REG8(0xB418)  // Tunnel path credits (from 0x0A55)
#define REG_TUNNEL_PATH_MODE    XDATA_REG8(0xB419)  // Tunnel path mode (from 0x0A54)
#define REG_TUNNEL_LINK_CFG_LO  XDATA_REG8(0xB41A)  // Tunnel link config low (from 0x0A53)
#define REG_TUNNEL_LINK_CFG_HI  XDATA_REG8(0xB41B)  // Tunnel link config high (from 0x0A52)
#define REG_TUNNEL_DATA_LO      XDATA_REG8(0xB420)  // Tunnel data register low
#define REG_TUNNEL_DATA_HI      XDATA_REG8(0xB421)  // Tunnel data register high
#define REG_TUNNEL_STATUS_0     XDATA_REG8(0xB422)  // Tunnel status byte 0
#define REG_TUNNEL_STATUS_1     XDATA_REG8(0xB423)  // Tunnel status byte 1

#define REG_PCIE_LANE_COUNT     XDATA_REG8(0xB424)
#define REG_TUNNEL_CAP2_0       XDATA_REG8(0xB425)  // Tunnel capability set 2 (fixed 0x06)
#define REG_TUNNEL_CAP2_1       XDATA_REG8(0xB426)  // Tunnel capability set 2 (fixed 0x04)
#define REG_TUNNEL_CAP2_2       XDATA_REG8(0xB427)  // Tunnel capability set 2 (fixed 0x00)
#define REG_TUNNEL_PATH2_CRED   XDATA_REG8(0xB428)  // Tunnel path 2 credits
#define REG_TUNNEL_PATH2_MODE   XDATA_REG8(0xB429)  // Tunnel path 2 mode
#define REG_TUNNEL_AUX_CFG_LO   XDATA_REG8(0xB42A)  // Tunnel auxiliary config low
#define REG_TUNNEL_AUX_CFG_HI   XDATA_REG8(0xB42B)  // Tunnel auxiliary config high

// Adapter Link State (0xB430-0xB4C8)
#define REG_TUNNEL_LINK_STATE   XDATA_REG8(0xB430)  // Tunnel link state (bit 0 = up)
#define REG_TUNNEL_LINK_STATUS  XDATA_REG8(0xB431)  // Tunnel link training status (stock=0x0C when trained)
#define REG_POWER_CTRL_B432     XDATA_REG8(0xB432)  // Power control for lanes (low 3 bits = link width)
#define REG_TUNNEL_CTRL_B403    XDATA_REG8(0xB403)  // Tunnel control (stock=0x01 when trained, set by PHY events)
#define REG_PCIE_LINK_STATE     XDATA_REG8(0xB434)  // PCIe link state (low nibble = lane enable mask)
#define   PCIE_LINK_STATE_MASK    0x0F  // Bits 0-3: PCIe link state/lane mask
#define REG_PCIE_LANE_CONFIG    XDATA_REG8(0xB436)  // PCIe lane configuration
#define   PCIE_LANE_CFG_LO_MASK   0x0F  // Bits 0-3: Low config
#define   PCIE_LANE_CFG_HI_MASK   0xF0  // Bits 4-7: High config
#define REG_PCIE_LINK_TRAIN     XDATA_REG8(0xB438)  // PCIe link training pattern

/*
 * LTSSM State Register (0xB450)
 * Reports current PCIe LTSSM (Link Training and Status State Machine) state.
 * Read-only hardware status register.
 *
 * Key values observed:
 *   0x00 = Detect.Quiet (idle, no activity)
 *   0x01 = Detect.Active (checking for receiver impedance)
 *   0x10+ = Polling (TS1/TS2 exchange started)
 *   0x48 = L0 (link trained, normal operation) — observed on ASMedia 174C:2463 stock
 *   0x78 = L0 (link trained) — observed on tinygrad ADD1:0001 stock
 *
 * B450 oscillating between 0x00 and 0x01 means the PHY detects receiver
 * impedance (GPU present) but cannot advance to Polling. This indicates
 * a PHY configuration or power issue, not a missing device.
 */
#define REG_PCIE_LTSSM_STATE    XDATA_REG8(0xB450)
#define   LTSSM_DETECT_QUIET      0x00
#define   LTSSM_DETECT_ACTIVE     0x01
#define   LTSSM_POLLING_MIN       0x10  // >= 0x10 means Polling started
#define   LTSSM_L0                0x48  // Link trained (varies by FW version)

#define REG_PCIE_LTSSM_B451     XDATA_REG8(0xB451)  // LTSSM sub-state (stock=0x01)
#define REG_PCIE_LTSSM_B452     XDATA_REG8(0xB452)  // LTSSM sub-state (stock=0x01)
#define REG_PCIE_LTSSM_B453     XDATA_REG8(0xB453)  // LTSSM sub-state (stock=0x01)
#define REG_PCIE_LTSSM_B454     XDATA_REG8(0xB454)  // LTSSM config (stock=0x1F)
#define REG_PCIE_LTSSM_B455     XDATA_REG8(0xB455)  // LTSSM link speed (stock=0x19 trained, 0x10 untrained)
#define REG_POWER_CTRL_B455     REG_PCIE_LTSSM_B455  // Legacy alias

/*
 * PCIe PERST Control / Tunnel Link Status (0xB480-0xB482)
 *
 * B480 bit 0: PERST# control for downstream PCIe device.
 *   Set bit 0 = assert PERST# (hold device in reset)
 *   Clear bit 0 = deassert PERST# (release reset, allow link training)
 *
 * Stock firmware power-on sequence:
 *   1. Assert PERST: B480 |= 0x01  (functions at 0x160B, 0x993E, 0xC024)
 *   2. Enable power (GPIO5=HIGH for 12V, C656 bit 5 for 3.3V)
 *   3. Wait for power stabilization
 *   4. Release PERST: B480 &= ~0x01  (at 0x365F, 0x20F2)
 *   5. Poll B455 bit 1 for link detect
 *
 * Stock firmware pre-trigger check: reads B480, expects 0x01 (PERST asserted
 * during active operation means device is managed).
 */
#define REG_PCIE_PERST_CTRL     XDATA_REG8(0xB480)  /* PCIe PERST# control */
#define   PCIE_PERST_ASSERT       0x01  // Bit 0: Assert PERST# (hold device in reset)
#define REG_TUNNEL_LINK_CTRL    REG_PCIE_PERST_CTRL  /* Legacy alias */
#define   TUNNEL_LINK_UP          PCIE_PERST_ASSERT   // Legacy alias
#define REG_PCIE_LINK_CTRL_B481 XDATA_REG8(0xB481)  /* PCIe link control (bits 0-1 = speed) */
#define REG_TUNNEL_ADAPTER_MODE XDATA_REG8(0xB482)  /* Tunnel adapter mode */
#define   TUNNEL_MODE_MASK        0xF0  // Bits 4-7: Tunnel mode
#define   TUNNEL_MODE_ENABLED     0xF0  // High nibble 0xF0 = tunnel mode enabled

#define REG_PCIE_LINK_STATUS_ALT XDATA_REG16(0xB4AE)
#define REG_PCIE_LANE_MASK      XDATA_REG8(0xB4C8)

// PCIe Queue Registers (0xB80C-0xB80F)
#define REG_PCIE_QUEUE_INDEX_LO XDATA_REG8(0xB80C)  // Queue index low
#define REG_PCIE_QUEUE_INDEX_HI XDATA_REG8(0xB80D)  // Queue index high
#define REG_PCIE_QUEUE_FLAGS_LO XDATA_REG8(0xB80E)  // Queue flags low
#define   PCIE_QUEUE_FLAG_VALID    0x01  // Bit 0: Queue entry valid
#define REG_PCIE_QUEUE_FLAGS_HI XDATA_REG8(0xB80F)  // Queue flags high
#define   PCIE_QUEUE_ID_MASK       0x0E  // Bits 1-3: Queue ID (shifted)

//=============================================================================
// UART Controller (0xC000-0xC00F)
// Based on ASM1142 UART at 0xF100-0xF10A, adapted for ASM2464PD
// Default config: 921600 baud, 8O1 (set LCR=0x03 for 8N1)
//=============================================================================
#define REG_UART_RBR            XDATA_REG8(0xC000)  // Receive Buffer Register (RO)
#define REG_UART_THR            XDATA_REG8(0xC001)  // Transmit Holding Register (WO)
#define REG_UART_IER            XDATA_REG8(0xC002)  // Interrupt Enable Register
#define REG_UART_IIR            XDATA_REG8(0xC004)  // Interrupt Identification Register (RO)
#define REG_UART_FCR            XDATA_REG8(0xC004)  // FIFO Control Register (WO)
#define REG_UART_RFBR           XDATA_REG8(0xC005)  // RX FIFO Bytes Received (RO) - count of bytes in RX FIFO
#define REG_UART_TFBF           XDATA_REG8(0xC006)  // TX FIFO Bytes Free (RO)
#define REG_UART_LCR            XDATA_REG8(0xC007)  // Line Control Register
#define   LCR_DATA_BITS_MASK      0x03  // Bits 0-1: 0=5, 1=6, 2=7, 3=8 data bits
#define   LCR_STOP_BITS           0x04  // Bit 2: 0=1 stop, 1=2 stop bits
#define   LCR_PARITY_MASK         0x38  // Bits 3-5: Parity (XX0=None, 001=Odd, 011=Even)
#define   LCR_LOOPBACK            0x80  // Bit 7: Enable loopback mode
#define REG_UART_MCR            XDATA_REG8(0xC008)  // Modem Control Register
#define REG_UART_LSR            XDATA_REG8(0xC009)  // Line Status Register
#define   LSR_RX_FIFO_OVERFLOW    0x01  // Bit 0: RX FIFO overflow (RW1C)
#define   LSR_TX_EMPTY            0x20  // Bit 5: TX empty
#define REG_UART_MSR            XDATA_REG8(0xC00A)  // Modem Status Register
#define REG_UART_STATUS         XDATA_REG8(0xC00E)  // UART status (bits 0-2 = busy flags)

//=============================================================================
// Link/PHY Control Registers (0xC200-0xC2FF)
//=============================================================================
#define REG_LINK_CTRL           XDATA_REG8(0xC202)
#define   LINK_CTRL_BIT3          0x08  /* Bit 3: link controller enable (set by DAC8 init) */
#define REG_LINK_CONFIG         XDATA_REG8(0xC203)
#define REG_LINK_STATUS         XDATA_REG8(0xC204)
#define REG_PHY_CTRL            XDATA_REG8(0xC205)
#define REG_PHY_LINK_CTRL_C208  XDATA_REG8(0xC208)
#define REG_PHY_LINK_CTRL_C20B  XDATA_REG8(0xC20B)  /* PHY link control (bit 7 cleared by DAC8 init) */
#define REG_PHY_LINK_CONFIG_C20C XDATA_REG8(0xC20C)
/*
 * PHY RXPLL Reset Register (0xC20E)
 * Controls the downstream PCIe receiver PLL reset.
 * Used in phy_rst_rxpll (bank1 0xE989):
 *   Write 0xFF = assert RXPLL reset
 *   Write 0x00 = de-assert RXPLL reset (PLL begins re-lock)
 * Must bracket writes with CC37 bit 2 set/clear (RXPLL reset mode).
 */
#define REG_PHY_RXPLL_RESET     XDATA_REG8(0xC20E)
#define REG_PHY_CTRL_C20F       XDATA_REG8(0xC20F)  /* PHY control (cleared during U1/U2 entry, restored to 0xC8) */
#define REG_PHY_LINK_CTRL_C21B  XDATA_REG8(0xC21B)  /* PHY link control (bits 7:6 set by DAC8 init) */
#define REG_PHY_SERDES_C22F     XDATA_REG8(0xC22F)  /* SerDes config (bit 2 set, bit 6 cleared by DAC8 init) */
#define REG_PHY_CONFIG          XDATA_REG8(0xC233)
#define   PHY_CONFIG_MODE_MASK    0x03  // Bits 0-1: PHY config mode
#define REG_PHY_STATUS          XDATA_REG8(0xC284)
#define REG_PHY_VENDOR_CTRL_C2E0 XDATA_REG8(0xC2E0)  /* PHY vendor control (bit 6/7 = read control) */
#define REG_PHY_VENDOR_CTRL_C2E2 XDATA_REG8(0xC2E2)  /* PHY vendor control 2 (bit 6/7 = read control) */

//=============================================================================
// Vendor/Debug Registers (0xC300-0xC3FF)
//=============================================================================
#define REG_VENDOR_CTRL_C343    XDATA_REG8(0xC343)  /* Vendor control (bit 6 = enable, bit 5 = mode) */
#define   VENDOR_CTRL_C343_BIT5   0x20              /* Bit 5: Vendor mode */
#define   VENDOR_CTRL_C343_BIT6   0x40              /* Bit 6: Vendor enable */
#define REG_VENDOR_CTRL_C360    XDATA_REG8(0xC360)  /* Vendor control (bit 6/7 = read control) */
#define REG_VENDOR_CTRL_C362    XDATA_REG8(0xC362)  /* Vendor control 2 (bit 6/7 = read control) */

//=============================================================================
// NVMe Interface Registers (0xC400-0xC5FF)
//=============================================================================
// NVMe DMA control (0xC4ED-0xC4EF)
#define REG_NVME_DMA_CTRL_ED    XDATA_REG8(0xC4ED)  // NVMe DMA control
#define REG_NVME_DMA_ADDR_LO    XDATA_REG8(0xC4EE)  // NVMe DMA address low
#define REG_NVME_DMA_ADDR_HI    XDATA_REG8(0xC4EF)  // NVMe DMA address high
#define REG_NVME_CTRL           XDATA_REG8(0xC400)
#define REG_NVME_STATUS         XDATA_REG8(0xC401)
#define REG_NVME_CTRL_STATUS    XDATA_REG8(0xC412)
#define   NVME_CTRL_WRITE_DIR    0x01  // Bit 0: 1=WRITE (host→device), 0=READ
#define   NVME_CTRL_DMA_START    0x02  // Bit 1: Start DMA transfer
#define   NVME_CTRL_STATUS_READY  0x02  // Bit 1: NVMe controller ready (alias)
#define REG_NVME_CONFIG         XDATA_REG8(0xC413)
#define   NVME_CONFIG_EP_MASK    0x3F  // Bits 0-5: Endpoint/channel index
#define   NVME_CONFIG_MASK_HI    0xC0  // Bits 6-7: Config mode
#define REG_NVME_DATA_CTRL      XDATA_REG8(0xC414)
#define   NVME_DATA_CTRL_MASK     0xC0  // Bits 6-7: Data control mode
#define   NVME_DATA_CTRL_BIT7     0x80  // Bit 7: Data control high bit
#define REG_NVME_DEV_STATUS     XDATA_REG8(0xC415)
#define   NVME_DEV_STATUS_MASK    0xC0  // Bits 6-7: Device status
// NVMe SCSI Command Buffer (0xC4C0-0xC4CA) - used for SCSI to NVMe translation
#define REG_NVME_SCSI_CMD_BUF_0 XDATA_REG8(0xC4C0)  // SCSI cmd buffer byte 0
#define REG_NVME_SCSI_CMD_BUF_1 XDATA_REG8(0xC4C1)  // SCSI cmd buffer byte 1
#define REG_NVME_SCSI_CMD_BUF_2 XDATA_REG8(0xC4C2)  // SCSI cmd buffer byte 2
#define REG_NVME_SCSI_CMD_BUF_3 XDATA_REG8(0xC4C3)  // SCSI cmd buffer byte 3
#define REG_NVME_SCSI_CMD_LEN_0 XDATA_REG8(0xC4C4)  // SCSI cmd length byte 0
#define REG_NVME_SCSI_CMD_LEN_1 XDATA_REG8(0xC4C5)  // SCSI cmd length byte 1
#define REG_NVME_SCSI_CMD_LEN_2 XDATA_REG8(0xC4C6)  // SCSI cmd length byte 2
#define REG_NVME_SCSI_CMD_LEN_3 XDATA_REG8(0xC4C7)  // SCSI cmd length byte 3
#define REG_NVME_SCSI_TAG       XDATA_REG8(0xC4C8)  // SCSI command tag
#define REG_NVME_SCSI_CTRL      XDATA_REG8(0xC4C9)  // SCSI control byte
#define REG_NVME_SCSI_DATA      XDATA_REG8(0xC4CA)  // SCSI data byte

#define REG_NVME_CMD            XDATA_REG8(0xC420)  /* Also DMA xfer byte count high */
#define REG_NVME_CMD_OPCODE     XDATA_REG8(0xC421)  /* Also DMA xfer byte count low */
#define REG_NVME_DMA_XFER_HI   XDATA_REG8(0xC420)  /* DMA transfer byte count high (alias) */
#define REG_NVME_DMA_XFER_LO   XDATA_REG8(0xC421)  /* DMA transfer byte count low (alias) */
#define REG_NVME_LBA_LOW        XDATA_REG8(0xC422)
#define REG_NVME_LBA_MID        XDATA_REG8(0xC423)
#define REG_NVME_LBA_HIGH       XDATA_REG8(0xC424)
#define REG_NVME_COUNT_LOW      XDATA_REG8(0xC425)
#define REG_NVME_DMA_ADDR_C426  XDATA_REG8(0xC426)  /* DMA sector count / buffer addr high */
#define REG_NVME_DMA_ADDR_C427  XDATA_REG8(0xC427)  /* DMA sector count / buffer addr low */
#define REG_NVME_COUNT_HIGH     XDATA_REG8(0xC426)  /* Alias for compatibility */
#define REG_NVME_ERROR          XDATA_REG8(0xC427)  /* Alias for compatibility */
#define REG_NVME_QUEUE_CFG      XDATA_REG8(0xC428)
#define   NVME_QUEUE_CFG_MASK_LO  0x03  // Bits 0-1: Queue config low
#define   NVME_QUEUE_CFG_BIT3     0x08  // Bit 3: Queue config flag
#define REG_NVME_CMD_PARAM      XDATA_REG8(0xC429)
#define   NVME_CMD_PARAM_TYPE    0xE0  // Bits 5-7: Command parameter type
/*
 * NVMe/MSC Doorbell (0xC42A)
 * Part of 900B/C42A "doorbell dance" for bulk IN transfers.
 *
 * Pre-trigger ramp up (before C42C write):
 *   C42A |= 0x01, |= 0x02, |= 0x04, |= 0x08, |= 0x10
 * Post-trigger teardown:
 *   C42A &= ~0x01, &= ~0x02, &= ~0x04, &= ~0x08, &= ~0x10
 *
 * Bit 5 (0x20): NVMe link init gate.
 *   Set before NVMe link param writes (C473/C472), clear after.
 *   Used in hw_init Phase 1, SET_INTERFACE, and post-trigger cleanup.
 *
 * In hw_init, initial state has various bits set from prior ROM config;
 * the MSC init dance reads and modifies bits in specific order.
 */
#define REG_NVME_DOORBELL       XDATA_REG8(0xC42A)
#define   NVME_DOORBELL_BIT0      0x01  // Bit 0: MSC/queue doorbell
#define   NVME_DOORBELL_BIT1      0x02  // Bit 1: Queue config
#define   NVME_DOORBELL_BIT2      0x04  // Bit 2: Queue config
#define   NVME_DOORBELL_BIT3      0x08  // Bit 3: Queue config
#define   NVME_DOORBELL_BIT4      0x10  // Bit 4: Queue config
#define   NVME_DOORBELL_LINK_GATE 0x20  // Bit 5: NVMe link init gate
#define REG_NVME_CMD_FLAGS      XDATA_REG8(0xC42B)
/*
 * USB MSC Engine (0xC42C-0xC42D)
 * Primary bulk IN trigger for MSC (Mass Storage Class) transfers.
 * Used by stock firmware for ALL CSW sends and data-in phases.
 *
 * Usage: data at D800+, set 901A=data_len, write C42C=0x01, clear C42D bit 0.
 * Preceded by 900B/C42A doorbell dance (ramp up bits, then tear down).
 *
 * REQUIRES: Full MSC engine init (stock 0xB1C5) must have been run first.
 * This sets 92C0, 91D1, 9300-9305, 9091, 9093, 91C1, 9002, 9005, 90E2, 905E.
 * Without these registers, the C42C trigger is consumed (reads back 0x00)
 * but NO USB packet is generated.
 *
 * REQUIRES: Interface descriptor class must be 0x08 (Mass Storage) / 0x06 / 0x50.
 * The hardware MSC engine may check the interface class to enable C42C routing.
 *
 * USB 3.0: Confirmed working on stock firmware even without NVMe device (GPU only).
 *          C471=0x00 does NOT prevent C42C from working on USB3.
 * USB 2.0: Only works when C471=0x01 (NVMe queue busy).
 *
 * Stock firmware writes C42C=0x01 at three locations:
 *   0x49B5 — scsi_csw_build: CSW send (doorbell dance + USBS + trigger)
 *   0xB21C — msc_engine_init: Initial arm during SET_INTERFACE
 *   0x1150 — ISR exit: C42C ack (read bit 0, if set → call 0x47D5 → write 0x01)
 *
 * Post-trigger cleanup: C42A |= 0x20, then NVMe link init (C428/C473/C472),
 * then C42A &= ~0x20. Stock does NOT call arm_msc again immediately after CSW.
 */
#define REG_USB_MSC_CTRL        XDATA_REG8(0xC42C)  /* Write 0x01 to trigger bulk IN */
#define REG_USB_MSC_STATUS      XDATA_REG8(0xC42D)  /* Read status; clear bit 0 after trigger */
#define REG_NVME_CMD_PRP1       XDATA_REG8(0xC430)  // NVMe command PRP1 (init: 0xFF)
#define REG_NVME_CMD_PRP2       XDATA_REG8(0xC431)  // NVMe command PRP2 (init: 0xFF)
#define REG_NVME_CMD_PRP3       XDATA_REG8(0xC432)  // NVMe PRP byte 2 (init: 0xFF)
#define REG_NVME_CMD_PRP4       XDATA_REG8(0xC433)  // NVMe PRP byte 3 (init: 0xFF)
#define REG_NVME_CMD_CDW10      XDATA_REG8(0xC435)
/*
 * NVMe Init Control / Interrupt Masks (0xC438-0xC44B)
 * Two groups of 4 bytes each, written to 0xFF during NVMe link init,
 * cleared to 0x00 during Phase 2 PRP/Queue clears.
 *
 * Group 1 (0xC438-C43B): Under C42A bit 5, after C473 bit 5+2, C472 clear bit 2
 * Group 2 (0xC448-C44B): Under C42A bit 5, after C473 bit 6+1, C472 clear bit 1
 */
#define REG_NVME_INIT_CTRL      XDATA_REG8(0xC438)  // NVMe init control (set to 0xFF)
#define REG_NVME_CMD_CDW11      XDATA_REG8(0xC439)
#define REG_NVME_INT_MASK_A     XDATA_REG8(0xC43A)  /* NVMe/Interrupt mask A (init: 0xFF) */
#define REG_NVME_INT_MASK_B     XDATA_REG8(0xC43B)  /* NVMe/Interrupt mask B (init: 0xFF) */
#define REG_NVME_QUEUE_PTR      XDATA_REG8(0xC43D)
#define REG_NVME_QUEUE_DEPTH    XDATA_REG8(0xC43E)
#define REG_NVME_PHASE          XDATA_REG8(0xC43F)
#define REG_NVME_QUEUE_CTRL     XDATA_REG8(0xC440)
#define REG_NVME_SQ_HEAD        XDATA_REG8(0xC441)
#define REG_NVME_SQ_TAIL        XDATA_REG8(0xC442)
#define REG_NVME_CQ_HEAD        XDATA_REG8(0xC443)
#define REG_NVME_CQ_TAIL        XDATA_REG8(0xC444)
#define REG_NVME_CQ_STATUS      XDATA_REG8(0xC445)
#define REG_NVME_LBA_3          XDATA_REG8(0xC446)
#define REG_NVME_INIT_CTRL2     XDATA_REG8(0xC448)  // NVMe init ctrl2 byte 0 (0xFF)
#define REG_NVME_INIT_CTRL2_1   XDATA_REG8(0xC449)  // NVMe init ctrl2 byte 1 (0xFF)
#define REG_NVME_INIT_CTRL2_2   XDATA_REG8(0xC44A)  // NVMe init ctrl2 byte 2 (0xFF)
#define REG_NVME_INIT_CTRL2_3   XDATA_REG8(0xC44B)  // NVMe init ctrl2 byte 3 (0xFF)
#define REG_NVME_CMD_STATUS_50  XDATA_REG8(0xC450)  // NVMe command status
#define REG_NVME_QUEUE_STATUS_51 XDATA_REG8(0xC451) // NVMe queue status
#define   NVME_QUEUE_STATUS_51_MASK 0x1F  // Bits 0-4: Queue status index
#define REG_DMA_ENTRY           XDATA_REG16(0xC462)
#define REG_CMDQ_DIR_END        XDATA_REG16(0xC470)
/*
 * NVMe Queue Busy (0xC471)
 * Indicates NVMe command queue is active.
 *
 * With NVMe device: write 0x01 sticks, enables 90A1/C42C bulk IN paths.
 * With GPU only:    write 0x01 does NOT stick (reads back 0x00).
 *
 * USB 3.0: C471=0x00 does NOT block C42C CSW sends. Stock firmware confirmed
 *          working on USB3 with GPU (C471=0x00). The earlier assumption that
 *          C471 was required for C42C on USB3 was WRONG.
 * USB 2.0: C471=0x01 IS required for both 90A1 and C42C bulk IN paths.
 *
 * Stock ISR exit at 0x10E5 reads C471 in a loop (up to 32 times) when
 * C802 bit 2 is set. Also checked at 0x1013 in CBW processing path.
 *
 * Stock firmware writes 0x01 during SET_INTERFACE (Phase 4, doorbell dance).
 * Read-only without NVMe queues configured by hardware.
 */
#define REG_NVME_QUEUE_BUSY     XDATA_REG8(0xC471)  /* Queue busy (R/O without NVMe) */
#define   NVME_QUEUE_BUSY_BIT     0x01              /* Bit 0: Queue busy */
#define REG_NVME_LINK_CTRL      XDATA_REG8(0xC472)  // NVMe link control
#define REG_NVME_LINK_PARAM     XDATA_REG8(0xC473)  // NVMe link parameter (bit 4)
#define REG_NVME_CMD_STATUS_C47A XDATA_REG8(0xC47A) // NVMe command status (used by usb_ep_loop)
#define REG_NVME_DMA_CTRL_C4E9  XDATA_REG8(0xC4E9)  // NVMe DMA control extended
#define REG_NVME_PARAM_C4EA     XDATA_REG8(0xC4EA)  // NVMe parameter storage
#define REG_NVME_PARAM_C4EB     XDATA_REG8(0xC4EB)  // NVMe parameter storage high
/*
 * Transfer Control (0xC509)
 * Used by both NVMe and software DMA paths in dispatch_0206.
 * Pre-trigger: C509 = (C509 & 0xFE) | 0x01 (set bit 0, via 0x3172)
 * Post-trigger: C509 &= 0xFE (clear bit 0, via 0x3288)
 */
#define REG_XFER_CTRL_C509      XDATA_REG8(0xC509)
#define REG_NVME_BUF_CFG        XDATA_REG8(0xC508)  // NVMe buffer configuration
#define   NVME_BUF_CFG_MASK_LO   0x3F  // Bits 0-5: Buffer index
#define   NVME_BUF_CFG_MASK_HI   0xC0  // Bits 6-7: Buffer mode
#define REG_NVME_QUEUE_INDEX    XDATA_REG8(0xC512)
#define REG_NVME_QUEUE_PENDING  XDATA_REG8(0xC516)  /* Pending queue status */
#define   NVME_QUEUE_PENDING_IDX  0x3F              /* Bits 0-5: Queue index */
#define REG_NVME_QUEUE_TRIGGER  XDATA_REG8(0xC51A)
#define REG_NVME_QUEUE_STATUS   XDATA_REG8(0xC51E)
#define   NVME_QUEUE_STATUS_IDX   0x3F  // Bits 0-5: Queue index
#define REG_NVME_LINK_STATUS    XDATA_REG8(0xC520)
#define   NVME_LINK_STATUS_BIT1   0x02  // Bit 1: NVMe link status flag
#define   NVME_LINK_STATUS_BIT7   0x80  // Bit 7: NVMe link ready

//=============================================================================
// PHY Extended Registers (0xC600-0xC6FF)
//=============================================================================
#define REG_PHY_EXT_2D          XDATA_REG8(0xC62D)
#define   PHY_EXT_LANE_MASK       0x07  // Bits 0-2: Lane configuration
#define REG_PHY_CFG_C655        XDATA_REG8(0xC655)  /* PHY config (bit 3 set by flash_set_bit3) */
/*
 * HDDPC Power Control / PHY Extended Signal (0xC656)
 * Controls PCIE_3V3_EN via the dedicated HDDPC pin (C21).
 *
 * Bit 5: HDDPC enable (PCIE_3V3 power to downstream PCIe slot)
 *   Set bit 5 = enable 3.3V power (C656 |= 0x20)
 *   Clear bit 5 = disable 3.3V power (C656 &= ~0x20)
 *
 * Stock firmware enables at 0xE31A (via helper at 0xC049).
 * Stock firmware disables at 0xE462 (C656 &= ~0x20).
 * Function at 0x46DE checks bit 5: if not set, returns error (power not ready).
 *
 * When enabled with GPIO5=HIGH (12V), GPU draws ~87W from +12V rail.
 * When disabled, GPU draws ~0.5W (12V converter in hiccup mode).
 */
#define REG_HDDPC_CTRL         XDATA_REG8(0xC656)
#define REG_PHY_EXT_SIGNAL      REG_HDDPC_CTRL       // Legacy alias
#define REG_PHY_EXT_56          REG_HDDPC_CTRL       // Legacy alias
#define   HDDPC_ENABLE            0x20  // Bit 5: Enable PCIE_3V3 power
#define   PHY_EXT_SIGNAL_READY    HDDPC_ENABLE        // Legacy alias
#define   PHY_EXT_SIGNAL_CFG      HDDPC_ENABLE        // Legacy alias
/*
 * PCIe Lane Control (0xC659)
 * Bit 0: Lane enable control.
 *   Stock firmware clears it during init (E612 with r7 bit 0 set).
 *   Gets SET back during link training by PHY event handlers.
 *   Stock trained value: 0x01. Custom untrained value: 0x00.
 */
#define REG_PCIE_LANE_CTRL_C659 XDATA_REG8(0xC659)
#define REG_PHY_CFG_C65A        XDATA_REG8(0xC65A)  /* PHY config (bit 3 set by flash_set_bit3) */
#define   PHY_CFG_C65A_BIT3       0x08  // Bit 3: PHY config flag
#define REG_PHY_EXT_5B          XDATA_REG8(0xC65B)
#define   PHY_EXT_ENABLE          0x08  // Bit 3: PHY extended enable
#define   PHY_EXT_MODE            0x20  // Bit 5: PHY mode
#define REG_PHY_EXT_B3          XDATA_REG8(0xC6B3)
#define   PHY_EXT_LINK_READY      0x30  // Bits 4,5: Link ready status
#define REG_PHY_LINK_CTRL_BD    XDATA_REG8(0xC6BD)  /* PHY link control (bit 0 = enable) */
/*
 * PHY Config (0xC6A8) — Link state control
 * Bit 0 set (|= 0x01) in bda4 state reset, called by both
 * 91D1 bit 0 (link training) and bit 2 (link reset ack) handlers.
 */
#define REG_PHY_CFG_C6A8        XDATA_REG8(0xC6A8)
#define   PHY_CFG_C6A8_ENABLE     0x01  // Bit 0: PHY link state enable
#define REG_PHY_VENDOR_CTRL_C6DB XDATA_REG8(0xC6DB) /* PHY vendor control (bit 2 = status) */
#define   PHY_VENDOR_CTRL_C6DB_BIT2 0x04            /* Bit 2: Vendor status flag */

//=============================================================================
// Interrupt Controller (0xC800-0xC80F)
//=============================================================================
#define REG_INT_STATUS_C800     XDATA_REG8(0xC800)  /* Interrupt status register */
#define   INT_STATUS_GLOBAL       0x01
#define   INT_STATUS_PCIE         0x04  // Bit 2: PCIe interrupt status
#define REG_INT_ENABLE          XDATA_REG8(0xC801)  /* Interrupt enable register */
#define   INT_ENABLE_GLOBAL       0x01  // Bit 0: Global interrupt enable
#define   INT_ENABLE_USB          0x02  // Bit 1: USB interrupt enable
#define   INT_ENABLE_PCIE         0x04  // Bit 2: PCIe interrupt enable
#define   INT_ENABLE_SYSTEM       0x10  // Bit 4: System interrupt enable
/*
 * USB Interrupt Gate (0xC802) — ISR dispatch control
 *
 * Stock firmware ISR at 0x0e33 checks bit 0 as a gate condition before
 * processing any USB events. Our firmware skips this check.
 *
 * Bit 2: Control transfer processing pending. Stock ISR exit at 0x10e5
 *        checks this bit and if set, enters a loop reading C471 up to
 *        32 times for control transfer completion polling.
 */
#define REG_INT_USB_STATUS      XDATA_REG8(0xC802)  /* USB interrupt gate/status */
#define   INT_USB_GATE            0x01  // Bit 0: ISR gate (stock checks before processing)
#define   INT_USB_CTRL_PENDING    0x04  // Bit 2: Control transfer processing (C471 poll loop)
/*
 * Auxiliary/DMA Mode Status (0xC805)
 * Also used for DMA mode configuration before bulk OUT data handling.
 * Stock firmware at 0x32BF: C805 = (C805 & 0xF9) | 0x02 before ALL
 * bulk OUT data handlers (E1 flash write, SCSI writes, etc.)
 * This configures the DMA path for receiving bulk data.
 */
#define REG_INT_AUX_STATUS      XDATA_REG8(0xC805)
#define   INT_AUX_ENABLE          0x02  // Bit 1: DMA mode / auxiliary enable
#define   INT_AUX_STATUS          0x04  // Bit 2: Auxiliary status
#define REG_INT_SYSTEM          XDATA_REG8(0xC806)  /* System interrupt status */
#define   INT_SYSTEM_EVENT        0x01  // Bit 0: System event interrupt
#define   INT_SYSTEM_TIMER        0x10  // Bit 4: System timer event
#define   INT_SYSTEM_LINK         0x20  // Bit 5: Link state change
#define REG_INT_DMA_CTRL        XDATA_REG8(0xC807)  /* Interrupt/DMA control */
#define REG_INT_CTRL            XDATA_REG8(0xC809)  /* Interrupt control register */
#define REG_INT_PCIE_NVME       XDATA_REG8(0xC80A)  /* PCIe/NVMe interrupt status */
#define   INT_PCIE_NVME_EVENTS    0x0F  // Bits 0-3: PCIe event flags
#define   INT_PCIE_NVME_TIMER     0x10  // Bit 4: NVMe command completion
#define   INT_PCIE_NVME_EVENT     0x20  // Bit 5: PCIe link event
#define   INT_PCIE_NVME_STATUS    0x40  // Bit 6: NVMe queue interrupt

//=============================================================================
// I2C Controller (0xC870-0xC87F)
//=============================================================================
#define REG_I2C_ADDR            XDATA_REG8(0xC870)
#define REG_I2C_MODE            XDATA_REG8(0xC871)
#define REG_I2C_LEN             XDATA_REG8(0xC873)
#define REG_I2C_CSR             XDATA_REG8(0xC875)
#define REG_I2C_SRC             XDATA_REG32(0xC878)
#define REG_I2C_DST             XDATA_REG32(0xC87C)
#define REG_I2C_CSR_ALT         XDATA_REG8(0xC87F)

//=============================================================================
// Alternate Flash Controller (0xC880-0xC886)
//=============================================================================
#define REG_FLASH_CMD_ALT       XDATA_REG8(0xC880)  /* Alternate flash command */
#define REG_FLASH_CSR_ALT       XDATA_REG8(0xC881)  /* Alternate flash CSR */
#define REG_FLASH_ADDR_LO_ALT   XDATA_REG8(0xC882)  /* Alternate flash addr low */
#define REG_FLASH_ADDR_MD_ALT   XDATA_REG8(0xC883)  /* Alternate flash addr mid */
#define REG_FLASH_ADDR_HI_ALT   XDATA_REG8(0xC884)  /* Alternate flash addr high */
#define REG_FLASH_DATA_LEN_ALT  XDATA_REG8(0xC885)  /* Alternate flash data len */
#define REG_FLASH_DATA_HI_ALT   XDATA_REG8(0xC886)  /* Alternate flash data len hi */

//=============================================================================
// SPI Flash Controller (0xC89F-0xC8AE)
//=============================================================================
#define REG_FLASH_CON           XDATA_REG8(0xC89F)
#define REG_FLASH_ADDR_LO       XDATA_REG8(0xC8A1)
#define REG_FLASH_ADDR_MD       XDATA_REG8(0xC8A2)
#define REG_FLASH_DATA_LEN      XDATA_REG8(0xC8A3)
#define REG_FLASH_DATA_LEN_HI   XDATA_REG8(0xC8A4)
#define REG_FLASH_DIV           XDATA_REG8(0xC8A6)
#define REG_FLASH_CSR           XDATA_REG8(0xC8A9)
#define   FLASH_CSR_BUSY          0x01  // Bit 0: Flash controller busy
#define REG_FLASH_CMD           XDATA_REG8(0xC8AA)
#define REG_FLASH_ADDR_HI       XDATA_REG8(0xC8AB)
#define REG_FLASH_ADDR_LEN      XDATA_REG8(0xC8AC)
#define   FLASH_ADDR_LEN_MASK     0xFC  // Bits 2-7: Address length (upper bits)
#define REG_FLASH_MODE          XDATA_REG8(0xC8AD)
#define   FLASH_MODE_ENABLE       0x01  // Bit 0: Flash mode enable
#define REG_FLASH_BUF_OFFSET    XDATA_REG16(0xC8AE)
#define REG_FLASH_BUF_OFFSET_LO XDATA_REG8(0xC8AE)  /* Flash buffer offset low byte */
#define REG_FLASH_BUF_OFFSET_HI XDATA_REG8(0xC8AF)  /* Flash buffer offset high byte */

//=============================================================================
// DMA Engine Registers (0xC8B0-0xC8D9)
//=============================================================================
#define REG_DMA_MODE            XDATA_REG8(0xC8B0)
#define REG_DMA_CHAN_AUX        XDATA_REG8(0xC8B2)
#define REG_DMA_CHAN_AUX1       XDATA_REG8(0xC8B3)
#define REG_DMA_XFER_CNT_HI     XDATA_REG8(0xC8B4)
#define REG_DMA_XFER_CNT_LO     XDATA_REG8(0xC8B5)
#define REG_DMA_CHAN_CTRL2      XDATA_REG8(0xC8B6)
#define   DMA_CHAN_CTRL2_START    0x01  // Bit 0: Start/busy
#define   DMA_CHAN_CTRL2_DIR      0x02  // Bit 1: Direction
#define   DMA_CHAN_CTRL2_ENABLE   0x04  // Bit 2: Enable
#define   DMA_CHAN_CTRL2_ACTIVE   0x80  // Bit 7: Active
#define REG_DMA_CHAN_STATUS2    XDATA_REG8(0xC8B7)
#define REG_DMA_TRIGGER         XDATA_REG8(0xC8B8)
#define   DMA_TRIGGER_START       0x01  // Bit 0: Trigger transfer
/*
 * DMA Config (0xC8D4)
 * 0xA0 = software DMA mode (used by dispatch_0206 software path)
 * 0x80 | param = NVMe DMA mode (param_2 | 0x80)
 * 0x00 = disable DMA (cleanup after transfer)
 */
#define REG_DMA_CONFIG          XDATA_REG8(0xC8D4)
#define   DMA_CONFIG_SW_MODE      0xA0  // Software DMA mode (no NVMe)
#define   DMA_CONFIG_ENABLE       0x80  // Enable DMA engine (NVMe path)
#define   DMA_CONFIG_DISABLE      0x00  // Disable DMA engine
#define REG_DMA_QUEUE_IDX       XDATA_REG8(0xC8D5)
#define REG_DMA_STATUS          XDATA_REG8(0xC8D6)
#define   DMA_STATUS_TRIGGER      0x01  // Bit 0: Status trigger
#define   DMA_STATUS_DONE         0x04  // Bit 2: Done flag
#define   DMA_STATUS_ERROR        0x08  // Bit 3: Error flag
#define REG_DMA_CTRL            XDATA_REG8(0xC8D7)
#define REG_DMA_STATUS2         XDATA_REG8(0xC8D8)
#define   DMA_STATUS2_TRIGGER     0x01  // Bit 0: Status 2 trigger
#define REG_DMA_STATUS3         XDATA_REG8(0xC8D9)
#define   DMA_STATUS3_UPPER      0xF8  // Bits 3-7: Status upper bits

//=============================================================================
// CPU Mode/Control (0xCA00-0xCAFF)
//=============================================================================
#define REG_CPU_MODE_NEXT       XDATA_REG8(0xCA06)
#define REG_CPU_CTRL_CA2E       XDATA_REG8(0xCA2E)  /* CPU control */
#define REG_CPU_CTRL_CA60       XDATA_REG8(0xCA60)  /* CPU control CA60 */
#define REG_CPU_CTRL_CA70       XDATA_REG8(0xCA70)  /* CPU control */
#define REG_CPU_CTRL_CA81       XDATA_REG8(0xCA81)  /* CPU control CA81 - PCIe init */

//=============================================================================
// Timer Registers (0xCC10-0xCC24)
//=============================================================================
#define REG_TIMER0_DIV          XDATA_REG8(0xCC10)
#define REG_TIMER0_CSR          XDATA_REG8(0xCC11)
#define   TIMER_CSR_ENABLE        0x01  // Bit 0: Timer enable
#define   TIMER_CSR_EXPIRED       0x02  // Bit 1: Timer expired flag
#define   TIMER_CSR_CLEAR         0x04  // Bit 2: Clear interrupt
#define REG_TIMER0_THRESHOLD    XDATA_REG16(0xCC12)
#define REG_TIMER0_THRESHOLD_HI XDATA_REG8(0xCC12)  /* Timer 0 threshold high byte */
#define REG_TIMER0_THRESHOLD_LO XDATA_REG8(0xCC13)  /* Timer 0 threshold low byte */
#define REG_TIMER1_DIV          XDATA_REG8(0xCC16)
#define REG_TIMER1_CSR          XDATA_REG8(0xCC17)
#define REG_TIMER1_THRESHOLD    XDATA_REG16(0xCC18)
#define REG_TIMER1_THRESHOLD_HI XDATA_REG8(0xCC18)  /* Timer 1 threshold high byte */
#define REG_TIMER1_THRESHOLD_LO XDATA_REG8(0xCC19)  /* Timer 1 threshold low byte */
#define REG_TIMER2_DIV          XDATA_REG8(0xCC1C)
#define REG_TIMER2_CSR          XDATA_REG8(0xCC1D)
#define REG_TIMER2_THRESHOLD    XDATA_REG16(0xCC1E)
#define REG_TIMER2_THRESHOLD_LO XDATA_REG8(0xCC1E)  /* Timer 2 threshold low */
#define REG_TIMER2_THRESHOLD_HI XDATA_REG8(0xCC1F)  /* Timer 2 threshold high */
#define REG_TIMER3_DIV          XDATA_REG8(0xCC22)
#define REG_TIMER3_CSR          XDATA_REG8(0xCC23)
#define REG_TIMER3_IDLE_TIMEOUT XDATA_REG8(0xCC24)

//=============================================================================
// CPU Control Extended (0xCC30-0xCCFF)
//=============================================================================
/*
 * CPU Mode / USB Speed Control (0xCC30)
 * Controls USB speed capability. Written 0x01 at boot (hw_init).
 * Write 0x00 to force USB 2.0 High Speed fallback (handle_link_event).
 */
#define REG_CPU_MODE            XDATA_REG8(0xCC30)
#define   CPU_MODE_USB2           0x00  // Force USB 2.0 High Speed (fallback)
#define   CPU_MODE_USB3           0x01  // USB 3.0 SuperSpeed capable (boot default)
#define REG_CPU_EXEC_CTRL       XDATA_REG8(0xCC31)  /* CPU execution control */
#define   CPU_EXEC_ENABLE         0x01  // Bit 0: Execution enable
#define REG_CPU_EXEC_STATUS     XDATA_REG8(0xCC32)  /* CPU execution status */
#define   CPU_EXEC_STATUS_ACTIVE  0x01  // Bit 0: CPU execution active
#define REG_CPU_EXEC_STATUS_2   XDATA_REG8(0xCC33)  /* CPU execution status 2 */
#define   CPU_EXEC_STATUS_2_INT   0x04  // Bit 2: Interrupt pending
#define REG_CPU_EXEC_CTRL_2     XDATA_REG8(0xCC34)  /* CPU execution control 2 */
#define REG_CPU_EXEC_STATUS_3   XDATA_REG8(0xCC35)  /* CPU execution status 3 */
#define   CPU_EXEC_STATUS_3_BIT0  0x01  // Bit 0: Exec active flag
#define   CPU_EXEC_STATUS_3_BIT2  0x04  // Bit 2: Exec status flag
#define REG_CPU_CTRL_CC36       XDATA_REG8(0xCC36)  /* CPU control */
/*
 * CPU Control CC37 — RXPLL reset mode control
 * Bit 2 must be set before asserting RXPLL reset (C20E=0xFF),
 * and cleared after de-asserting (C20E=0x00) and PLL re-lock delay.
 * Stock firmware helper at bank1 0x9877 reads CC37 & 0xFB (bit 2 cleared).
 */
#define REG_CPU_CTRL_CC37       XDATA_REG8(0xCC37)
#define   CPU_CTRL_CC37_RXPLL_MODE 0x04  // Bit 2: RXPLL reset mode enable
// Timer enable/disable control registers
#define REG_TIMER_ENABLE_A      XDATA_REG8(0xCC38)  /* Timer enable control A */
#define   TIMER_ENABLE_A_BIT      0x02              /* Bit 1: Timer enable */
#define REG_TIMER_CTRL_CC39     XDATA_REG8(0xCC39)  /* Timer control */
#define REG_TIMER_ENABLE_B      XDATA_REG8(0xCC3A)  /* Timer enable control B */
#define   TIMER_ENABLE_B_BIT      0x02              /* Bit 1: Timer enable */
#define   TIMER_ENABLE_B_BITS56   0x60              /* Bits 5-6: Timer extended mode */
/*
 * Timer / Link Power Control (0xCC3B)
 * Bit 1 (0x02) is the SS link power management control bit.
 * 91D1 bit 3 handler: clears bit 1 (CC3B &= ~0x02) during U1/U2 entry.
 * 91D1 bit 0 handler: clears bit 1 if link is down (91C0 bit 1 == 0).
 * Init: written 0x0C, then 0x0D, then 0x0F during hw_init.
 */
#define REG_TIMER_CTRL_CC3B     XDATA_REG8(0xCC3B)
#define   TIMER_CTRL_ENABLE       0x01              /* Bit 0: Timer active */
#define   TIMER_CTRL_LINK_POWER   0x02              /* Bit 1: SS link power control (cleared in 91D1 handlers) */
/*
 * CPU Keepalive (0xCC2A)
 * Written in main loop to prevent watchdog reset.
 * Main loop writes 0x0C every iteration.
 */
#define REG_CPU_KEEPALIVE       XDATA_REG8(0xCC2A)  /* Write 0x0C in main loop */
#define REG_CPU_KEEPALIVE_CC2C  XDATA_REG8(0xCC2C)  /* Keepalive param (init: 0xC7) */
#define REG_CPU_KEEPALIVE_CC2D  XDATA_REG8(0xCC2D)  /* Keepalive param (init: 0xC7) */
/*
 * LTSSM State Register (0xCC3D)
 * Link Training and Status State Machine state control.
 * Bit 7 cleared at end of LTSSM manipulation sequence (bank1 0xCCDD-0xCD26).
 */
#define REG_LTSSM_STATE         XDATA_REG8(0xCC3D)
#define REG_CPU_CTRL_CC3D       REG_LTSSM_STATE      // Legacy alias
#define   LTSSM_STATE_FORCE       0x80  // Bit 7: Force/lock LTSSM state
#define REG_CPU_CTRL_CC3E       XDATA_REG8(0xCC3E)
/*
 * LTSSM Control Register (0xCC3F)
 * Controls Link Training and Status State Machine transitions.
 * Stock firmware LTSSM manipulation at bank1 0xCCDD-0xCD26:
 *   Phase 1: Clear bits 5,6 (disable override + force)
 *   Phase 2: Clear bit 1, write, delay, set bit 5 (enable override)
 *   Phase 3: Delay, clear bit 2, write, delay, set bit 6 (force state)
 *   Phase 4: Delay, clear CC3D bit 7
 */
#define REG_LTSSM_CTRL          XDATA_REG8(0xCC3F)
#define REG_CPU_CTRL_CC3F       REG_LTSSM_CTRL       // Legacy alias
#define   LTSSM_CTRL_WRITE_TRIG   0x02  // Bit 1: Write trigger
#define   LTSSM_CTRL_STATE_TRIG   0x04  // Bit 2: State change trigger
#define   LTSSM_CTRL_OVERRIDE_EN  0x20  // Bit 5: LTSSM override enable
#define   LTSSM_CTRL_FORCE_STATE  0x40  // Bit 6: Force LTSSM state
#define REG_CPU_CLK_CFG         XDATA_REG8(0xCC43)  /* CPU clock config */

// Timer 4 Registers (0xCC5C-0xCC5F)
#define REG_TIMER4_DIV          XDATA_REG8(0xCC5C)  /* Timer 4 divisor */
#define REG_TIMER4_CSR          XDATA_REG8(0xCC5D)  /* Timer 4 control/status */
#define REG_TIMER4_THRESHOLD_LO XDATA_REG8(0xCC5E)  /* Timer 4 threshold low */
#define REG_TIMER4_THRESHOLD_HI XDATA_REG8(0xCC5F)  /* Timer 4 threshold high */

// CPU control registers (0xCC80-0xCC83)
#define REG_CPU_CTRL_CC80       XDATA_REG8(0xCC80)  /* CPU control 0xCC80 */
#define   CPU_CTRL_CC80_ENABLE   0x03  // Bits 0-1: CPU control enable mask
#define REG_CPU_INT_CTRL        XDATA_REG8(0xCC81)
#define   CPU_INT_CTRL_ENABLE    0x01  // Bit 0: Enable/start interrupt
#define   CPU_INT_CTRL_ACK       0x02  // Bit 1: Acknowledge interrupt
#define   CPU_INT_CTRL_TRIGGER   0x04  // Bit 2: Trigger interrupt
#define REG_CPU_CTRL_CC82       XDATA_REG8(0xCC82)  /* CPU control 0xCC82 */
#define REG_CPU_CTRL_CC83       XDATA_REG8(0xCC83)  /* CPU control 0xCC83 */

// Transfer DMA controller - for internal memory block transfers
#define REG_XFER_DMA_CTRL       XDATA_REG8(0xCC88)  /* Transfer DMA control */
#define REG_XFER_DMA_CMD        XDATA_REG8(0xCC89)  /* Transfer DMA command/status */
#define   XFER_DMA_CMD_START     0x01  // Bit 0: Start transfer
#define   XFER_DMA_CMD_DONE      0x02  // Bit 1: Transfer complete
#define   XFER_DMA_CMD_MODE      0x30  // Bits 4-5: Transfer mode (0x31 = mode 1)
#define REG_XFER_DMA_ADDR_LO    XDATA_REG8(0xCC8A)  /* Transfer DMA address low */
#define REG_XFER_DMA_ADDR_HI    XDATA_REG8(0xCC8B)  /* Transfer DMA address high */

#define REG_CPU_DMA_CTRL_CC90   XDATA_REG8(0xCC90)  /* CPU DMA control */
#define REG_CPU_DMA_INT         XDATA_REG8(0xCC91)  /* CPU DMA interrupt status */
#define   CPU_DMA_INT_ACK        0x02  // Bit 1: Acknowledge DMA interrupt
#define   CPU_DMA_INT_TRIGGER    0x04  // Bit 2: Trigger DMA
#define REG_CPU_DMA_DATA_LO     XDATA_REG8(0xCC92)  /* CPU DMA data low */
#define REG_CPU_DMA_DATA_HI     XDATA_REG8(0xCC93)  /* CPU DMA data high */
#define REG_CPU_DMA_READY       XDATA_REG8(0xCC98)  /* CPU DMA ready status */
#define   CPU_DMA_READY_BIT2     0x04              /* Bit 2: DMA ready flag */
#define REG_XFER_DMA_CFG        XDATA_REG8(0xCC99)  /* Transfer DMA config */
#define   XFER_DMA_CFG_ACK       0x02  // Bit 1: Acknowledge config
#define   XFER_DMA_CFG_ENABLE    0x04  // Bit 2: Config enable
#define REG_XFER_DMA_DATA_LO    XDATA_REG8(0xCC9A)  /* Transfer DMA data low */
#define REG_XFER_DMA_DATA_HI    XDATA_REG8(0xCC9B)  /* Transfer DMA data high */
// Secondary transfer DMA controller
#define REG_XFER2_DMA_CTRL      XDATA_REG8(0xCCD8)  /* Transfer 2 DMA control */
#define REG_XFER2_DMA_STATUS    XDATA_REG8(0xCCD9)  /* Transfer 2 DMA status */
#define   XFER2_DMA_STATUS_ACK   0x02  // Bit 1: Acknowledge status
#define REG_TIMER5_CSR          XDATA_REG8(0xCCB9)  /* Timer 5 control/status (alternate) */
#define REG_XFER2_DMA_ADDR_LO   XDATA_REG8(0xCCDA)  /* Transfer 2 DMA address low */
#define REG_XFER2_DMA_ADDR_HI   XDATA_REG8(0xCCDB)  /* Transfer 2 DMA address high */
#define REG_CPU_EXT_CTRL        XDATA_REG8(0xCCF8)  /* CPU extended control */
#define REG_CPU_EXT_STATUS      XDATA_REG8(0xCCF9)  /* CPU extended status */
#define   CPU_EXT_STATUS_ACK     0x02  // Bit 1: Acknowledge extended status

//=============================================================================
// CPU Extended Control (0xCD00-0xCD3F)
//=============================================================================
/*
 * CPU Timer / Link Reset Control (0xCD31)
 * Written in bda4 state reset (91D1 bit 0 handler): 0x04 then 0x02.
 * This sequence resets the link timer state machine.
 */
#define REG_PHY_DMA_CMD_CD30    XDATA_REG8(0xCD30)   // PHY DMA command
#define REG_CPU_TIMER_CTRL_CD31 XDATA_REG8(0xCD31)
#define   CPU_TIMER_CD31_CLEAR    0x04  // Write first: clear/reset timer
#define   CPU_TIMER_CD31_START    0x02  // Write second: restart timer
#define REG_PHY_DMA_ADDR_LO    XDATA_REG8(0xCD32)   // PHY DMA address low
#define REG_PHY_DMA_ADDR_HI    XDATA_REG8(0xCD33)   // PHY DMA address high

//=============================================================================
// SCSI DMA Control (0xCE00-0xCE3F)
//=============================================================================
/*
 * SCSI DMA Engine (0xCE00-0xCE01)
 * CE00: Write 0x03 to start sector DMA, poll until 0x00 for completion.
 *       Stock firmware uses at 0x352E-0x3538 in per-sector copy loop.
 * CE01: DMA parameter -- combined with XDATA[0x0AFF] and CE01 upper bits.
 */
#define REG_SCSI_DMA_CTRL       XDATA_REG8(0xCE00)  /* Write 0x03 to start, poll 0x00 for done */
#define REG_SCSI_DMA_PARAM      XDATA_REG8(0xCE01)  /* DMA parameter (upper 2 bits | tag value) */
#define REG_SCSI_DMA_CFG_CE36   XDATA_REG8(0xCE36)  // SCSI DMA config 0xCE36
#define REG_SCSI_DMA_TAG_CE3A   XDATA_REG8(0xCE3A)  // SCSI DMA tag storage

//=============================================================================
// SCSI/Mass Storage DMA (0xCE40-0xCE97)
//=============================================================================
#define REG_SCSI_DMA_PARAM0     XDATA_REG8(0xCE40)
#define REG_SCSI_DMA_PARAM1     XDATA_REG8(0xCE41)
#define REG_SCSI_DMA_PARAM2     XDATA_REG8(0xCE42)
#define REG_SCSI_DMA_PARAM3     XDATA_REG8(0xCE43)
#define REG_SCSI_DMA_PARAM4     XDATA_REG8(0xCE44)
#define REG_SCSI_DMA_PARAM5     XDATA_REG8(0xCE45)
#define REG_SCSI_TAG_IDX        XDATA_REG8(0xCE51)   /* SCSI tag index */
#define REG_SCSI_DMA_XFER_CNT  XDATA_REG8(0xCE55)   /* DMA transfer byte count (after CE88/CE89 handshake) */
#define REG_SCSI_DMA_COMPL      XDATA_REG8(0xCE5C)
#define REG_SCSI_DMA_MASK       XDATA_REG8(0xCE5D)  /* SCSI DMA mask register */
#define REG_SCSI_DMA_QUEUE      XDATA_REG8(0xCE5F)  /* SCSI DMA queue control */
#define REG_SCSI_TRANSFER_CTRL  XDATA_REG8(0xCE70)
#define REG_SCSI_TRANSFER_MODE  XDATA_REG8(0xCE72)
#define REG_SCSI_BUF_CTRL0      XDATA_REG8(0xCE73)
#define REG_SCSI_BUF_CTRL1      XDATA_REG8(0xCE74)
#define REG_SCSI_BUF_LEN_LO     XDATA_REG8(0xCE75)
#define REG_SCSI_BUF_ADDR0      XDATA_REG8(0xCE76)
#define REG_SCSI_BUF_ADDR1      XDATA_REG8(0xCE77)
#define REG_SCSI_BUF_ADDR2      XDATA_REG8(0xCE78)
#define REG_SCSI_BUF_ADDR3      XDATA_REG8(0xCE79)
#define REG_SCSI_BUF_CTRL       XDATA_REG8(0xCE80)  /* SCSI buffer control global */
#define REG_SCSI_BUF_THRESH_HI  XDATA_REG8(0xCE81)  /* SCSI buffer threshold high */
#define REG_SCSI_BUF_THRESH_LO  XDATA_REG8(0xCE82)  /* SCSI buffer threshold low */
#define REG_SCSI_BUF_FLOW       XDATA_REG8(0xCE83)  /* SCSI buffer flow control */
#define   SCSI_DMA_COMPL_MODE0    0x01  // Bit 0: Mode 0 complete
#define   SCSI_DMA_COMPL_MODE10   0x02  // Bit 1: Mode 0x10 complete
#define REG_XFER_STATUS_CE60    XDATA_REG8(0xCE60)  // Transfer status CE60
#define   XFER_STATUS_BIT6        0x40  // Bit 6: Status flag
#define REG_XFER_CTRL_CE65      XDATA_REG8(0xCE65)
#define REG_SCSI_DMA_TAG_COUNT  XDATA_REG8(0xCE66)
#define   SCSI_DMA_TAG_MASK       0x1F  // Bits 0-4: Tag count (0-31)
#define REG_SCSI_DMA_QUEUE_STAT XDATA_REG8(0xCE67)
#define   SCSI_DMA_QUEUE_MASK     0x0F  // Bits 0-3: Queue status (0-15)
#define REG_XFER_STATUS_CE6C    XDATA_REG8(0xCE6C)  // Transfer status CE6C (bit 7: ready)
#define REG_SCSI_DMA_STATUS     XDATA_REG16(0xCE6E)
#define REG_SCSI_DMA_STATUS_L   XDATA_REG8(0xCE6E)   /* SCSI DMA status low byte */
#define REG_SCSI_DMA_STATUS_H   XDATA_REG8(0xCE6F)   /* SCSI DMA status high byte */
/*
 * USB/DMA State Machine Control (0xCE86-0xCE89)
 *
 * REG_BULK_DMA_HANDSHAKE (0xCE88): Bulk transfer DMA handshake trigger.
 *   Write 0x00 to init/reset the DMA state machine.
 *   After writing 0x00, CE89 transitions to 0x03 (ready).
 *   Written at end of handle_set_interface_inner() to arm bulk transfers.
 *
 * REG_USB_DMA_STATE (0xCE89): DMA state machine status.
 *   Must read 0x03 for bulk DMA to work.
 *   Checked by stock firmware pre-trigger: if CE89 != 0x03, skip trigger.
 *   Value 0x03 = bits 0+1 set = ready + success = DMA path available.
 *
 * Sequence: write CE88=0x00 → CE89 becomes 0x03 → bulk IN enabled.
 */
/*
 * USB Bulk DMA Handshake Registers (0xCE86-0xCE8A)
 *
 * CE88/CE89 handshake triggers USB bulk OUT data DMA to flash buffer (0x7000).
 * Stock firmware sequence at 0x3484-0x349D:
 *   1. Write CE88 = 0x00 (init DMA state machine)
 *   2. Poll CE89 bit 0 until set (DMA triggered)
 *   3. Check CE89 bit 1: set = CBW (ljmp 0x35A1), clear = bulk data
 *   4. Check CE86 bit 4: set = error (ljmp 0x35A1), clear = OK
 *   5. Read CE55 for DMA transfer byte count
 *
 * After handshake completes, poll CE55 != 0 to confirm data at 0x7000.
 */
#define REG_USB_DMA_ERROR       XDATA_REG8(0xCE86)  /* Bit 4: DMA error flag (stock: 0x349D) */
#define   USB_DMA_ERROR_BIT       0x10  // Bit 4: DMA error
#define REG_BULK_DMA_HANDSHAKE  XDATA_REG8(0xCE88)  /* Write 0x00 to start bulk DMA handshake */
#define REG_USB_DMA_STATE       XDATA_REG8(0xCE89)  /* DMA handshake status */
#define   USB_DMA_STATE_READY     0x01  // Bit 0: DMA ready (poll this after CE88 write)
#define   USB_DMA_STATE_CBW       0x02  // Bit 1: 1=CBW received, 0=bulk data
#define   USB_DMA_STATE_ERROR     0x04  // Bit 2: DMA error in copy loop (stock: 0x3546)
#define REG_USB_DMA_SECTOR_CTRL XDATA_REG8(0xCE8A)  /* Per-sector DMA control (stock: bit 2 set at 0x3251) */
#define REG_XFER_MODE_CE95      XDATA_REG8(0xCE95)
#define REG_SCSI_DMA_CMD_REG    XDATA_REG8(0xCE96)
#define REG_SCSI_DMA_RESP_REG   XDATA_REG8(0xCE97)

//=============================================================================
// USB Descriptor Validation (0xCEB0-0xCEB3)
//=============================================================================
#define REG_USB_DESC_VAL_CEB2   XDATA_REG8(0xCEB2)
#define REG_USB_DESC_VAL_CEB3   XDATA_REG8(0xCEB3)

//=============================================================================
// CPU Link Control (0xCEEF-0xCEFF)
//=============================================================================
#define REG_CPU_LINK_CEEF       XDATA_REG8(0xCEEF)   // CPU link control low
#define REG_CPU_LINK_CEF0       XDATA_REG8(0xCEF0)   // CPU link status
#define REG_CPU_LINK_CEF2       XDATA_REG8(0xCEF2)
#define   CPU_LINK_CEF2_READY     0x80  // Bit 7: Link ready
#define REG_CPU_LINK_CEF3       XDATA_REG8(0xCEF3)
#define   CPU_LINK_CEF3_ACTIVE    0x08  // Bit 3: Link active

/*
 * USB Endpoint Buffer (0xD800-0xDE5F)
 * Dual-purpose: used for both C42C MSC data and 90A1 DMA transfers.
 *
 * C42C path (current firmware):
 *   Data written directly at D800+. No header needed.
 *   CSW: D800-D80C = "USBS" + tag + residue + status (13 bytes)
 *   SCSI data: D800+ = response data (inquiry, sense, etc.)
 *   Length in 0x901A = payload bytes.
 *
 * 90A1 DMA path (alternative, needs NVMe/C471):
 *   Header at D800-D80F:
 *     D800 = 0x03 (buffer control, written LAST)
 *     D801 = 0x00 (must be 0x00 at trigger)
 *     D802-D803 = DMA base addr (0x00, 0x01)
 *     D804-D807 = DMA params (zeroed)
 *     D806 = 0x02 (data ready, written after data fill)
 *     D808-D80B = DMA source descriptor
 *     D80F = data length
 *   Payload at D810+.
 *   Length in 0x901A = 0x10 + data_len.
 *
 * Init: SET_CONFIG writes "USBS" to D800-D803 for CSW template.
 *       Phase 7 clears D800-DE5F. Phase 8 writes DE30=0x03, DE36=0x00.
 */
#define REG_USB_EP_BUF_CTRL     XDATA_REG8(0xD800)  // Buffer control / CSW sig 'U'
#define REG_USB_EP_BUF_SEL      XDATA_REG8(0xD801)  // Buffer select / CSW sig 'S'
#define REG_USB_EP_BUF_DATA     XDATA_REG8(0xD802)  // Buffer data / CSW sig 'B'
#define REG_USB_EP_BUF_PTR_LO   XDATA_REG8(0xD803)  // Pointer low / CSW sig 'S'
#define REG_USB_EP_BUF_PTR_HI   XDATA_REG8(0xD804)  // CSW tag byte 0
#define REG_USB_EP_BUF_LEN_LO   XDATA_REG8(0xD805)  // CSW tag byte 1
#define REG_USB_EP_BUF_STATUS   XDATA_REG8(0xD806)  // DMA status (0x02=data ready) / CSW tag 2
#define REG_USB_EP_BUF_LEN_HI   XDATA_REG8(0xD807)  // CSW tag byte 3
#define REG_USB_EP_RESIDUE0     XDATA_REG8(0xD808)  // CSW residue byte 0 / DMA desc 0
#define REG_USB_EP_RESIDUE1     XDATA_REG8(0xD809)  // CSW residue byte 1 / DMA desc 1
#define REG_USB_EP_RESIDUE2     XDATA_REG8(0xD80A)  // CSW residue byte 2 / DMA desc 2
#define REG_USB_EP_RESIDUE3     XDATA_REG8(0xD80B)  // CSW residue byte 3 / DMA desc 3
#define REG_USB_EP_CSW_STATUS   XDATA_REG8(0xD80C)  // CSW status byte
#define REG_USB_EP_CTRL_0D      XDATA_REG8(0xD80D)  // Control 0D
#define REG_USB_EP_CTRL_0E      XDATA_REG8(0xD80E)  // Control 0E
#define REG_USB_EP_DMA_LEN      XDATA_REG8(0xD80F)  // DMA data length (90A1 path)
#define REG_USB_EP_DATA_BASE    XDATA_REG8(0xD810)  // Start of payload (90A1 path)
/* Linear byte access for CSW/data payload region rooted at 0xD800. */
#define REG_USB_EP_BUF_BYTE(off) XDATA_REG8(0xD800 + (off))
// USB Endpoint buffers at 0xDE30, 0xDE36 (extended region)
#define REG_USB_EP_BUF_DE30     XDATA_REG8(0xDE30)  // EP buf extended ctrl (init: 0x03)
#define REG_USB_EP_BUF_DE36     XDATA_REG8(0xDE36)  // EP buf extended cfg (init: 0x00)

// Note: Full struct access at 0xD800 - see structs.h

//=============================================================================
// PHY Completion / Debug (0xE300-0xE3FF)
//=============================================================================
#define REG_PHY_MODE_E302       XDATA_REG8(0xE302)  /* PHY mode (bits 4-5 = lane config) */
#define REG_DEBUG_STATUS_E314   XDATA_REG8(0xE314)
#define REG_PHY_COMPLETION_E318 XDATA_REG8(0xE318)
#define REG_LINK_CTRL_E324      XDATA_REG8(0xE324)
#define   LINK_CTRL_E324_BIT2     0x04  // Bit 2: Link control flag

//=============================================================================
// Command Engine (0xE400-0xE4FF)
//=============================================================================
#define REG_CMD_CTRL_E400       XDATA_REG8(0xE400)  /* Command control (bit 7 = enable, bit 6 = busy) */
#define   CMD_CTRL_E400_BIT6      0x40  // Bit 6: Command busy flag
#define   CMD_CTRL_E400_BIT7      0x80  // Bit 7: Command enable
#define REG_CMD_STATUS_E402     XDATA_REG8(0xE402)  /* Command status (bit 3 = poll status) */
#define REG_CMD_CTRL_E403       XDATA_REG8(0xE403)
#define REG_CMD_CFG_E404        XDATA_REG8(0xE404)
#define REG_CMD_CFG_E405        XDATA_REG8(0xE405)
#define REG_CMD_CTRL_E409       XDATA_REG8(0xE409)  /* Command control (bit 0,7 = flags) */
#define REG_CMD_CFG_E40A        XDATA_REG8(0xE40A)  /* Command config - write 0x0F */
#define REG_CMD_CONFIG          XDATA_REG8(0xE40B)  /* Command config (bit 0 = flag) */
#define REG_CMD_CFG_E40D        XDATA_REG8(0xE40D)  /* Command config - write 0x28 */
#define REG_CMD_CFG_E40E        XDATA_REG8(0xE40E)  /* Command config - write 0x8A */
/*
 * PHY Event Register (0xE40F) — Write-1-to-clear
 * Stock firmware PHY maintenance dispatcher at bank1 0xAE9B reads this
 * and dispatches on individual bits in priority order: bit 7 > 0 > 5.
 */
#define REG_PHY_EVENT_E40F      XDATA_REG8(0xE40F)
#define REG_CMD_CTRL_E40F       REG_PHY_EVENT_E40F   // Legacy alias
#define   PHY_EVENT_LINK_CHANGE   0x01  // Bit 0: Link state change (→ 0x83D6)
#define   PHY_EVENT_SPEED_CHANGE  0x20  // Bit 5: Speed change (→ 0xE19E)
#define   PHY_EVENT_MAJOR         0x80  // Bit 7: Major PHY event / reset (→ 0xDD9C)
/*
 * PHY Interrupt Status (0xE410) — Write-1-to-clear
 * Checked after E40F events. Dispatches CDR and link training events.
 * Stock firmware at bank1 0xAEE4 checks individual bits.
 */
#define REG_PHY_INT_STATUS_E410 XDATA_REG8(0xE410)
#define REG_CMD_CTRL_E410       REG_PHY_INT_STATUS_E410  // Legacy alias
#define   PHY_INT_MINOR_EVENT     0x01  // Bit 0: Minor event (ack only)
#define   PHY_INT_CDR_TIMEOUT     0x08  // Bit 3: CDR timeout
#define   PHY_INT_PLL_EVENT       0x10  // Bit 4: PLL event
#define   PHY_INT_CDR_RECOVERY    0x20  // Bit 5: CDR recovery needed (→ 0xE5DF)
#define   PHY_INT_LINK_TRAINING   0x40  // Bit 6: Link training event (→ 0xE1BE)
#define   PHY_INT_MAJOR_ERROR     0x80  // Bit 7: Major PHY error
#define REG_CMD_CFG_E411        XDATA_REG8(0xE411)  /* Command config - write 0xA1 */
#define REG_CMD_CFG_E412        XDATA_REG8(0xE412)  /* Command config - write 0x79 */
#define REG_CMD_CFG_E413        XDATA_REG8(0xE413)  /* Command config (bits 0,1,4,5,6 = flags) */
#define REG_CMD_BUSY_STATUS     XDATA_REG8(0xE41C)
#define   CMD_BUSY_STATUS_BUSY    0x01  // Bit 0: Command engine busy
#define REG_CMD_TRIGGER         XDATA_REG8(0xE420)
#define REG_CMD_MODE_E421       XDATA_REG8(0xE421)
#define REG_CMD_PARAM           XDATA_REG8(0xE422)
#define REG_CMD_STATUS          XDATA_REG8(0xE423)
#define REG_CMD_ISSUE           XDATA_REG8(0xE424)
#define REG_CMD_TAG             XDATA_REG8(0xE425)
#define REG_CMD_LBA_0           XDATA_REG8(0xE426)
#define REG_CMD_LBA_1           XDATA_REG8(0xE427)
#define REG_CMD_LBA_2           XDATA_REG8(0xE428)
#define REG_CMD_LBA_3           XDATA_REG8(0xE429)
#define REG_CMD_COUNT_LOW       XDATA_REG8(0xE42A)
#define REG_CMD_COUNT_HIGH      XDATA_REG8(0xE42B)
#define REG_CMD_LENGTH_LOW      XDATA_REG8(0xE42C)
#define REG_CMD_LENGTH_HIGH     XDATA_REG8(0xE42D)
#define REG_CMD_RESP_TAG        XDATA_REG8(0xE42E)
#define REG_CMD_RESP_STATUS     XDATA_REG8(0xE42F)
#define REG_CMD_CTRL            XDATA_REG8(0xE430)
#define REG_CMD_TIMEOUT         XDATA_REG8(0xE431)
#define REG_CMD_PARAM_L         XDATA_REG8(0xE432)
#define REG_CMD_PARAM_H         XDATA_REG8(0xE433)
#define REG_CMD_EXT_PARAM_0     XDATA_REG8(0xE434)
#define REG_CMD_EXT_PARAM_1     XDATA_REG8(0xE435)

//=============================================================================
// Timer/CPU Control (0xCC00-0xCCFF)
//=============================================================================


//=============================================================================
// Debug/Interrupt (0xE600-0xE6FF)
//=============================================================================
#define REG_DEBUG_INT_E62F      XDATA_REG8(0xE62F)  // Debug interrupt 0x62F
#define REG_DEBUG_INT_E65F      XDATA_REG8(0xE65F)  // Debug interrupt 0x65F
#define REG_DEBUG_INT_E661      XDATA_REG8(0xE661)
#define   DEBUG_INT_E661_FLAG     0x80  // Bit 7: Debug interrupt flag
#define REG_PD_CTRL_E66A        XDATA_REG8(0xE66A)  /* PD control - clear bit 4 */
#define   PD_CTRL_E66A_BIT4       0x10  // Bit 4: PD control flag

//=============================================================================
// System Status / Link Control (0xE700-0xE7FF)
//=============================================================================
/*
 * Link Width / Recovery Control (0xE710)
 * Bits 5-7: Link width status (preserved in read-modify-write).
 * Bits 0-4: Lane/recovery configuration.
 *
 * 91D1 bit 0 handler: if link is down (91C0 bit 1 == 0),
 * writes (E710 & 0xE0) | 0x04 to set recovery mode while
 * preserving the upper link width bits.
 *
 * Written 0x04 during hw_init (before any link is up).
 */
#define REG_LINK_WIDTH_E710     XDATA_REG8(0xE710)
#define   LINK_WIDTH_MASK         0xE0  // Bits 5-7: Link width (preserve in RMW)
#define   LINK_RECOVERY_MODE      0x04  // Bit 2: Link recovery mode
#define   LINK_WIDTH_LANES_MASK   0x1F  // Bits 0-4: Lane configuration

/*
 * USB EP0 / Link Status (0xE712)
 * Two uses:
 * 1. Main loop at 0xCDC6: polls for bits 0|1 to process USB events.
 * 2. 91D1 bit 3 handler (0x9b95): bit 0 = busy, bit 1 = done.
 *    Polled during U1/U2 power transitions after 92CF clock recovery.
 *    Also written 0x01 during SET_ADDRESS to signal link status change.
 */
#define REG_LINK_STATUS_E712    XDATA_REG8(0xE712)
#define   LINK_E712_BUSY          0x01  // Bit 0: Operation busy / EP0 transfer pending
#define   LINK_E712_DONE          0x02  // Bit 1: Operation done / EP0 status complete

#define REG_LINK_STATUS_E716    XDATA_REG8(0xE716)
#define   LINK_STATUS_E716_MASK  0x03  // Bits 0-1: Link status
#define REG_LINK_CTRL_E717      XDATA_REG8(0xE717)  /* Link control (bit 0 = enable) */
#define REG_PHY_PLL_CTRL        XDATA_REG8(0xE741)  /* PHY PLL control */
#define REG_PHY_PLL_CFG         XDATA_REG8(0xE742)  /* PHY PLL config */
#define REG_PHY_POLL_E750       XDATA_REG8(0xE750)  /* PHY poll (read during reset 91D1 wait) */
#define REG_PHY_POLL_E751       XDATA_REG8(0xE751)  /* PHY poll alt */
/*
 * PHY RXPLL Configuration (0xE760-0xE763)
 * Used in phy_rxpll_config (bank1 0xE957) to configure RXPLL before reset.
 * E760: PHY PLL config A — bits 2,3 toggled (clear then set = no-op on those bits)
 * E761: PHY PLL config B — bits 2,3 cleared after E760 sets them
 * E763: PHY PLL event trigger — write 0x04 then 0x08 to trigger PLL reconfiguration
 *
 * Stock firmware trained values: E760=0x00, E761=0x00, E763=0x00
 * These should be CLEAR after training.
 */
#define REG_PHY_RXPLL_CFG_A     XDATA_REG8(0xE760)
#define REG_PHY_RXPLL_STATUS    XDATA_REG8(0xE762)  /* PHY RXPLL status */
#define REG_SYS_CTRL_E760       REG_PHY_RXPLL_CFG_A  // Legacy alias
#define REG_PHY_RXPLL_CFG_B     XDATA_REG8(0xE761)
#define REG_SYS_CTRL_E761       REG_PHY_RXPLL_CFG_B  // Legacy alias
#define REG_PHY_RXPLL_TRIGGER   XDATA_REG8(0xE763)
#define REG_SYS_CTRL_E763       REG_PHY_RXPLL_TRIGGER // Legacy alias
/*
 * PHY Timer Control (0xE764)
 * Configured during hw_init (written 0x14 four times).
 * Stock trained value: 0x19 (bits 0,3,4 set) — bits 0,3 get set during link training.
 * Custom untrained value: 0x14 (bit 2,4 set).
 */
#define REG_PHY_TIMER_CTRL_E764 XDATA_REG8(0xE764)
#define REG_SYS_CTRL_E765       XDATA_REG8(0xE765)  /* System control E765 */
#define REG_SYS_CTRL_E76C       XDATA_REG8(0xE76C)  /* System control */
#define REG_SYS_CTRL_E774       XDATA_REG8(0xE774)  /* System control */
#define REG_SYS_CTRL_E77C       XDATA_REG8(0xE77C)  /* System control */
#define REG_SYS_CTRL_E780       XDATA_REG8(0xE780)  /* System control */
#define REG_FLASH_READY_STATUS  XDATA_REG8(0xE795)
#define REG_PHY_LINK_CTRL       XDATA_REG8(0xE7E3)
#define   PHY_LINK_CTRL_BIT6      0x40  // Bit 6: PHY link control flag
#define   PHY_LINK_CTRL_BIT7      0x80  // Bit 7: PHY link ready
#define REG_PHY_LINK_TRIGGER    XDATA_REG8(0xE7FA)  /* PHY link trigger/config */
#define REG_LINK_MODE_CTRL      XDATA_REG8(0xE7FC)
#define   LINK_MODE_CTRL_MASK     0x03  // Bits 0-1: Link mode control

//=============================================================================
// System Control Extended (0xEA00-0xEAFF)
//=============================================================================
#define REG_SYS_CTRL_EA90       XDATA_REG8(0xEA90)  /* System control EA90 */

//=============================================================================
// NVMe Event (0xEC00-0xEC0F)
//=============================================================================
#define REG_NVME_EVENT_ACK      XDATA_REG8(0xEC04)
#define REG_NVME_EVENT_STATUS   XDATA_REG8(0xEC06)
#define   NVME_EVENT_PENDING      0x01  // Bit 0: NVMe event pending

//=============================================================================
// System Control (0xEF00-0xEFFF)
//=============================================================================
#define REG_CRITICAL_CTRL       XDATA_REG8(0xEF4E)

//=============================================================================
// PCIe TLP Format/Type Codes (for REG_PCIE_FMT_TYPE at 0xB210)
//=============================================================================
/*
 * PCIe TLP format/type byte encoding (PCIe Base Spec 3.0, Table 2-3):
 *   Bits [7:5] = Fmt (Format):
 *     000 = 3DW header, no data payload
 *     001 = 4DW header, no data payload
 *     010 = 3DW header, with data payload
 *     011 = 4DW header, with data payload
 *   Bits [4:0] = Type:
 *     00000 = Memory Read/Write (MRd/MWr)
 *     00100 = Config Read/Write Type 0 (CfgRd0/CfgWr0)
 *     00101 = Config Read/Write Type 1 (CfgRd1/CfgWr1)
 *
 * Bit 6 (0x40) = data payload present (i.e., write operation)
 * Bit 5 (0x20) = 4DW header (64-bit addressing)
 *
 * Use PCIE_FMT_HAS_DATA to test if a TLP type is a write.
 */
#define PCIE_FMT_HAS_DATA       0x40  /* Bit 6: TLP carries a data payload (write) */
#define PCIE_FMT_4DW_HDR        0x20  /* Bit 5: 4DW header (64-bit address) */

/* Memory Read/Write (Type 0x00) */
#define PCIE_FMT_MEM_READ       0x00  /* MRd:   3DW header, no data, 32-bit addr */
#define PCIE_FMT_MEM_WRITE      0x40  /* MWr:   3DW header, with data, 32-bit addr */
#define PCIE_FMT_MEM_READ64     0x20  /* MRd64: 4DW header, no data, 64-bit addr */
#define PCIE_FMT_MEM_WRITE64    0x60  /* MWr64: 4DW header, with data, 64-bit addr */

/* Config Read/Write Type 0 (targets device on local bus) */
#define PCIE_FMT_CFG_READ_0     0x04  /* CfgRd0: 3DW header, no data */
#define PCIE_FMT_CFG_WRITE_0    0x44  /* CfgWr0: 3DW header, with data */

/* Config Read/Write Type 1 (forwarded by bridges to downstream bus) */
#define PCIE_FMT_CFG_READ_1     0x05  /* CfgRd1: 3DW header, no data */
#define PCIE_FMT_CFG_WRITE_1    0x45  /* CfgWr1: 3DW header, with data */

//=============================================================================
// Bank-Selected Registers (0x0xxx-0x2xxx)
// These are accessed via bank switching or as part of extended memory access
//=============================================================================
#define REG_BANK_0200           XDATA_REG8(0x0200)  /* Bank register at 0x0200 */
#define REG_BANK_1200           XDATA_REG8(0x1200)  /* Bank register at 0x1200 */
#define REG_BANK_1235           XDATA_REG8(0x1235)  /* Bank register at 0x1235 */
#define REG_BANK_1407           XDATA_REG8(0x1407)  /* Bank register at 0x1407 */
#define REG_BANK_1504           XDATA_REG8(0x1504)  /* Bank register at 0x1504 */
#define REG_BANK_1507           XDATA_REG8(0x1507)  /* Bank register at 0x1507 */
#define REG_BANK_1603           XDATA_REG8(0x1603)  /* Bank register at 0x1603 */
#define REG_BANK_2269           XDATA_REG8(0x2269)  /* Bank register at 0x2269 */

//=============================================================================
// GPIO (0xC620-0xC638 control, 0xC650-0xC653 input)
//=============================================================================
// Control: write 0x00=input(pull-up), 0x02=output LOW, 0x03=output HIGH
#define REG_GPIO_CTRL(n)        XDATA_REG8(0xC620 + (n))  /* n = 0-27 */
#define REG_GPIO_CTRL_0         XDATA_REG8(0xC620)  /* GPIO 0 control */
// Input: read actual pin level regardless of mode
#define REG_GPIO_INPUT(n)       XDATA_REG8(0xC650 + ((n) >> 3))  /* bit (n & 7) */

//=============================================================================
// Timeouts (milliseconds)
//=============================================================================
#define TIMEOUT_NVME            5000
#define TIMEOUT_DMA             10000

#endif /* __REGISTERS_H__ */
