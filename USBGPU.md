# ASM2464PD as USBGPU - How Tinygrad Uses It

## Overview

Tinygrad uses the ASM2464PD USB4-to-NVMe bridge as a "USB GPU" by exploiting its PCIe
initiator capability and large internal SRAM buffer. This document explains the actual
implementation and identifies the slow path that needs improvement.

## Hardware Architecture

```
USB Host (tinygrad)           ASM2464PD Bridge                   GPU
       │                            │                              │
       │  USB3 Bulk Streams         │  PCIe TLPs                   │
       │  (31 concurrent)           │                              │
       │                            │                              │
       ├──EP 0x02 OUT (data)───────>│                              │
       ├──EP 0x04 OUT (commands)───>│  8051 CPU                    │
       │                            │     │                        │
       │<──EP 0x81 IN (data)────────│     ├──Config Space──────────>│
       │<──EP 0x83 IN (status)──────│     ├──Memory Write──────────>│
       │                            │     └──Memory Read───────────>│
       │                            │            │                  │
       │                            │            v                  │
       │                      Internal SRAM (6-8 MB)               │
       │                      ┌────────────────────┐               │
       │                      │ Data @ 0x00200000  │<──GPU DMA────>│
       │                      │ Queues @ 0x00820000│               │
       │                      └────────────────────┘               │
```

## Tinygrad's USB Communication

### Endpoints Used

| Endpoint | Direction | Purpose |
|----------|-----------|---------|
| 0x02     | OUT       | Bulk data to device (SCSI writes) |
| 0x04     | OUT       | SCSI command blocks |
| 0x81     | IN        | Bulk data from device |
| 0x83     | IN        | SCSI status responses |

### Buffer Allocations (from usb.py)

```python
# Per-stream buffers (31 streams for USB3 bulk streaming)
buf_data_out = 512 KB per stream    # Host → Device data
buf_data_in  = 4 KB per stream      # Device → Host data (NOTE: small!)
buf_cmd      = 32 bytes             # SCSI CDB wrapper
buf_stat     = 64 bytes             # Status response
```

### Transfer Methods

**1. SCSI WRITE (Fast - 64KB chunks)**
```python
# ScsiWriteOp in usb.py line 147-149
struct.pack('>BBQIBB', 0x8A, 0, op.lba, sectors, 0, 0)
# 0x8A = SCSI WRITE(16) opcode
# lba = Logical Block Address (maps to PCI address 0x00200000 + lba*512)
# sectors = size / 512

# Chunking (usb.py lines 156-160)
for i in range(0, len(buf), 0x10000):  # 64 KB chunks
    self.exec_ops([ScsiWriteOp(buf[i:i+0x10000], lba), ...])
```

**2. E4 Read (SLOW - 255 byte chunks)**
```python
# ReadOp in usb.py line 143-146
struct.pack('>BBBHB', 0xE4, op.size, addr >> 16, addr & 0xFFFF, 0)
# op.size limited to 0xFF (255 bytes)

# Read method (usb.py line 165-167)
def read(self, base_addr, length, stride=0xff):  # stride = 255!
    parts = self.exec_ops([ReadOp(base_addr + off, min(stride, length - off))
                           for off in range(0, length, stride)])
```

**3. E5 Write (Register access - 1 byte at a time)**
```python
# WriteOp in usb.py line 136-141
struct.pack('>BBBHB', 0xE5, value, addr >> 16, addr & 0xFFFF, 0)
# Single byte per command - used for MMIO register writes, not bulk data
```

### Speed Comparison

| Operation | Chunk Size | Throughput | Use Case |
|-----------|------------|------------|----------|
| SCSI WRITE (0x8A) | 64 KB | ~100+ MB/s | Fast bulk OUT |
| E4 Read (0xE4) | 255 bytes | ~1-5 MB/s | **SLOW** bulk IN |
| E5 Write (0xE5) | 1 byte | ~10-50 KB/s | Register config |
| PCIe MemWr | 4 bytes | ~4 MB/s | GPU BAR writes |
| PCIe MemRd | 4 bytes | ~2 MB/s | GPU BAR reads |

**The asymmetry is severe**: Writes are ~20-100x faster than reads.

## The Firmware Patch

Tinygrad's `patch.py` downloads ASMedia firmware and makes one modification:

```python
patches = [(0x2a0d + 1 + 4, b'\x0a', b'\x05')]
# File offset: 0x2a12
# Changes: mov r7, #0x0a → mov r7, #0x05
```

### What The Patch Does

At code address 0x2a0d, there's a SCSI write completion status check:

```c
// Decompiled from firmware code address 0x2a0d
if (*ptr != 0) {
    goto return_0x0a;  // Return error code 10
}
// ...
return_0x0a:
    // NOTE: patch.py changes this return value from 0x0a to 0x05
    return 0x0a;  // Original returns 10
```

The patch changes this return value from 0x0a (10) to 0x05 (5).

**Why?** The return value is a status code that affects the SCSI state machine.
Code 10 (0x0a) indicates one type of completion status, while code 5 (0x05)
indicates another. The patch likely changes how partial/retry scenarios are
handled to make the write path more reliable for tinygrad's usage pattern.

### VID/PID Change

The patch also changes the device identity via config blocks:

```
Original:  VID=0x174C PID=0x2464 (ASMedia USB4 NVMe)
Patched:   VID=0xADD1 PID=0x0001 ("tiny")
```

This allows tinygrad to identify its patched devices.

## Internal SRAM Buffer

### Size: 6-8 MB

```
PCI Address       Purpose
0x00200000        Data buffer start (6+ MB available)
0x00820000        NVMe queues (can be repurposed)

Gap = 0x00620000 = 6.125 MB for data buffer
```

### LBA Addressing

SCSI commands use LBA (Logical Block Address) which firmware converts:

```
buffer_addr = 0x00200000 + (LBA × 512)
```

So `ScsiWriteOp(data, lba=256)` writes to PCI address:
```
0x00200000 + (256 × 512) = 0x00220000
```

### 32-bit DMA Addressing

The 8051 has 64KB address space but accesses the full buffer via DMA registers:

```
Register   Purpose
CE76-CE79  32-bit DMA address (little-endian)
```

USB bulk transfers program these registers and let hardware DMA handle the data.
The 8051 CPU never touches the bulk data bytes.

## The Slow Path Problem

### Why E4 Reads Are Slow

E4 is a vendor-specific SCSI command for register/memory reads:

1. **Limited to 255 bytes per request** (line 143: `assert op.size <= 0xff`)
2. **Control transfer overhead** - each E4 goes through SCSI CDB processing
3. **No bulk streaming** - cannot pipeline like SCSI WRITE does

### Why SCSI READ Isn't Used

Standard SCSI READ(16) command (opcode 0x88) isn't implemented in tinygrad.
The firmware supports it, but tinygrad only implemented SCSI WRITE.

### Comparison: SCSI WRITE vs E4 READ

```
SCSI WRITE (current - fast):
  1 command → 64KB data → 1 status
  ~100 MB/s

E4 READ (current - slow):
  1 command → 255 bytes data → 1 status
  × 4000 times for 1 MB
  ~1-5 MB/s
```

## Plan to Fix the Slow Path

### Option 1: Implement ScsiReadOp (Recommended)

Add SCSI READ(16) support to tinygrad, mirroring the write path:

```python
@dataclasses.dataclass(frozen=True)
class ScsiReadOp: size:int; lba:int=0

# In exec_ops:
elif isinstance(op, ScsiReadOp):
    sectors = round_up(op.size, 512) // 512
    _add_req(struct.pack('>BBQIBB', 0x88, 0, op.lba, sectors, 0, 0),
             sectors * 512, None)  # Request data IN

def scsi_read(self, size:int, lba:int=0) -> bytes:
    # Chunk reads like writes
    result = b''
    for i in range(0, size, 0x10000):
        chunk_size = min(0x10000, size - i)
        resp = self.exec_ops([ScsiReadOp(chunk_size, lba + i // 512)])
        result += resp[0][:chunk_size]
    return result
```

**Expected improvement:** 255 bytes → 64 KB per request = **~250x faster**

### Option 2: Batch E4 Requests

If firmware changes are easier than tinygrad changes:

```python
# Current: Sequential 255-byte reads
for off in range(0, length, 255):
    result += exec_ops([ReadOp(addr + off, 255)])

# Better: Parallel 255-byte reads (use all 31 streams)
reads = [ReadOp(addr + off, 255) for off in range(0, length, 255)]
results = exec_ops(reads[:31])  # 31 concurrent = 7.9KB per batch
```

**Expected improvement:** ~31x faster (still limited by E4 overhead)

### Option 3: Larger E4 Transfers (Firmware Change)

The 255-byte limit is arbitrary. Firmware could support larger E4 reads:

```c
// Current: uint8_t limits to 255
// Change: Use 16-bit length field in E4 command

// E4 format: 0xE4 [size_hi] [addr>>16] [addr_lo] [addr_hi] [size_lo]
// Allows up to 64KB per E4 request
```

**Expected improvement:** ~250x faster with firmware mod

### Option 4: Use GPU Bus Mastering

For large reads from GPU memory, use the GPU as bus master:

1. GPU DMAs data to PCI address 0x00200000 (bridge's SRAM)
2. Tinygrad reads from SRAM via SCSI READ or E4
3. Eliminates slow PCIe MemRd TLPs from 8051

**Expected improvement:** GPU-limited rather than bridge-limited

## Summary

| Path | Current | After Fix | Improvement |
|------|---------|-----------|-------------|
| USB → Device (SCSI WRITE) | 64 KB/cmd | 64 KB/cmd | Already fast |
| Device → USB (E4 READ) | 255 B/cmd | 64 KB/cmd | **~250x** |
| 8051 → GPU (PCIe MemWr) | 4 B/TLP | 4 B/TLP | Can't improve* |
| GPU → 8051 (PCIe MemRd) | 4 B/TLP | 4 B/TLP | Can't improve* |
| GPU ↔ SRAM (bus master) | N/A | ~4 GB/s | Use this! |

*The 8051-initiated PCIe path is fundamentally slow (~4 MB/s max). Use GPU
bus mastering for high-throughput data movement.

## Quick Reference

### Tinygrad Device Opening

```python
# From usb.py line 117
self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04)
#              VID     PID     data_in stat_in data_out cmd_out
```

### Controller Initialization

```python
# usb.py lines 123-124
self.exec_ops([
    WriteOp(0x54b, b' '),        # Unknown config
    WriteOp(0x54e, b'\x04'),     # Unknown config
    WriteOp(0x5a8, b'\x02'),     # Unknown config
    WriteOp(0x5f8, b'\x04'),     # Unknown config
    WriteOp(0x7ec, b'\x01\x00\x00\x00'),  # 4-byte config
    WriteOp(0xc422, b'\x02'),    # Enable something
    WriteOp(0x0, b'\x33'),       # Start signal?
])
```

### PCIe Memory Access

```python
# usb.py lines 185-214
def pcie_request(self, fmt_type, address, value=None, size=4):
    # fmt_type: 0x60=MemWr, 0x20=MemRd
    # Writes to B210-B296 registers to generate TLPs

def pcie_mem_write(self, address, values, size):
    # Batch PCIe memory writes (4 on OSX, 16 on Linux per batch)
```

---

# Hardware Blocks for USBGPU Firmware

This section details all hardware blocks in the ASM2464PD and their relevance for
a USBGPU-specific firmware rewrite.

## Block Diagram

```
                              ASM2464PD SoC
    ┌──────────────────────────────────────────────────────────────────┐
    │                                                                  │
    │  ┌──────────┐    ┌──────────────────────────────────────────┐   │
    │  │  8051    │    │           Internal SRAM (6-8 MB)          │   │
    │  │   CPU    │    │  ┌────────────────────────────────────┐  │   │
    │  │ ~100MHz  │    │  │ 0x00200000: Data Buffer (6 MB)     │  │   │
    │  │          │    │  │ 0x00820000: Queue Region (128 KB)  │  │   │
    │  │ 64KB     │    │  └────────────────────────────────────┘  │   │
    │  │ XDATA    │    │                    ▲                     │   │
    │  │ views    │    │                    │ 32-bit DMA          │   │
    │  └────┬─────┘    └────────────────────┼─────────────────────┘   │
    │       │                               │                         │
    │       ▼                               ▼                         │
    │  ┌─────────────────────────────────────────────────────────┐   │
    │  │                    DMA Engines                          │   │
    │  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐    │   │
    │  │  │USB DMA  │  │SCSI DMA │  │XFER DMA │  │Flash DMA│    │   │
    │  │  │0x90xx   │  │0xCExx   │  │0xCCxx   │  │0xC8Ax   │    │   │
    │  │  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘    │   │
    │  └───────┼────────────┼────────────┼────────────┼──────────┘   │
    │          │            │            │            │               │
    │          ▼            ▼            ▼            ▼               │
    │  ┌─────────────┐ ┌─────────────┐ ┌──────┐ ┌──────────────┐     │
    │  │USB3 Ctrl   │ │PCIe Engine  │ │Timers│ │SPI Flash     │     │
    │  │0x9000-93FF │ │0xB200-B4FF  │ │0xCC10│ │0xC89F-C8AE   │     │
    │  │            │ │             │ │      │ │              │     │
    │  │EP0 Control │ │TLP Generate │ │Timer0│ │Config Store  │     │
    │  │EP1 Bulk IN │ │Config Space │ │Timer1│ │              │     │
    │  │EP2 Bulk OUT│ │Memory R/W   │ │Timer2│ │              │     │
    │  └──────┬─────┘ └──────┬──────┘ │Timer3│ └──────────────┘     │
    │         │              │        └──────┘                       │
    └─────────┼──────────────┼───────────────────────────────────────┘
              │              │
              ▼              ▼
         USB3 PHY       PCIe PHY
              │              │
              ▼              ▼
         To Host         To GPU
```

## Hardware Blocks - Detailed Analysis

### 1. USB3 Controller (0x9000-0x93FF) - **ESSENTIAL**

The USB3 controller handles all host communication. **Required for USBGPU.**

| Register Range | Purpose | USBGPU Need |
|----------------|---------|-------------|
| 0x9000-0x901F | Core USB status/control | Required |
| 0x905A-0x90FF | Endpoint configuration | Required |
| 0x9091-0x9092 | Control transfer phase, DMA trigger | Required |
| 0x9100-0x912F | Link status, CDB registers | Required |
| 0x91C0-0x91FF | USB PHY control | Required |
| 0x9200-0x92FF | Power management | Required |
| 0x9E00-0x9E07 | Setup packet buffer | Required |

**Key Registers for USBGPU:**
```
0x9000  USB_STATUS      - Connection state (bit 7=connected)
0x9091  USB_CTRL_PHASE  - Control transfer state machine
0x9092  USB_DMA_TRIGGER - Triggers descriptor DMA
0x9100  USB_LINK_STATUS - USB speed (bits 0-1: 2=SuperSpeed)
0x910D-0x9112 USB_CDB   - SCSI Command Descriptor Block
0x9E00-0x9E07 Setup Pkt - 8-byte USB setup packet
```

**Simplification:** Remove Mass Storage class, use Vendor class for simpler protocol.

---

### 2. SCSI/Bulk DMA Engine (0xCE00-0xCE9F) - **ESSENTIAL**

This is the high-speed bulk transfer engine. **Critical for USBGPU performance.**

| Register Range | Purpose | USBGPU Need |
|----------------|---------|-------------|
| 0xCE00-0xCE01 | DMA control/parameter | Required |
| 0xCE40-0xCE45 | DMA parameters | Required |
| 0xCE55 | Tag value | Required |
| 0xCE66-0xCE67 | Tag count, queue status | Required |
| 0xCE70-0xCE79 | Transfer control, **32-bit PCI address** | **Critical** |
| 0xCE86-0xCE89 | USB/DMA state machine | Required |

**Key Registers:**
```
0xCE00  SCSI_DMA_CTRL   - Write 0x03 to start transfer
0xCE76-CE79 BUF_ADDR    - 32-bit PCI address into SRAM (NOT XDATA!)
0xCE89  USB_DMA_STATE   - State machine (bit 0=ready, 1=success, 2=complete)
```

**This is how USB bulk transfers access the full 6-8MB SRAM without 8051 involvement.**

---

## DMA Engines - Deep Dive

This section explains exactly what each DMA engine transfers between.

### Memory Architecture

```
                    ASM2464PD Memory Architecture
                    ==============================

┌─────────────────────────────────────────────────────────────────────┐
│                        PCI Address Space                            │
│  (What downstream devices see / what DMA engines use)               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  0x00000000 ┌─────────────────────────────────────────────────┐    │
│             │              Reserved                            │    │
│  0x00200000 ├─────────────────────────────────────────────────┤    │
│             │                                                  │    │
│             │         DATA BUFFER (6+ MB)                      │    │
│             │                                                  │    │
│             │   - USB bulk data lands here                     │    │
│             │   - GPU can DMA to/from here                     │    │
│             │   - NVMe data transfers use this                 │    │
│             │                                                  │    │
│  0x00820000 ├─────────────────────────────────────────────────┤    │
│             │         COMMAND QUEUES (128 KB)                  │    │
│             │   - NVMe submission/completion queues            │    │
│             │   - Can be repurposed for GPU commands           │    │
│  0x00840000 └─────────────────────────────────────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                              ▲
                              │ Hardware maps to
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     Internal SRAM (Physical)                        │
│                     ~8 MB on-chip memory                            │
└─────────────────────────────────────────────────────────────────────┘
                              ▲
                              │ 8051 sees only windows
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    8051 XDATA Address Space                         │
│                    (64 KB, 0x0000-0xFFFF)                           │
├─────────────────────────────────────────────────────────────────────┤
│  0x0000-0x5FFF  Working memory, globals, tables                     │
│  0x6000-0x6FFF  Reserved                                            │
│  0x7000-0x7FFF  Flash buffer (4KB window)                           │
│  0x8000-0x8FFF  USB buffer (4KB window into PCI 0x00200000+)        │
│  0x9000-0x9FFF  MMIO: USB controller                                │
│  0xA000-0xAFFF  NVMe IOSQ (4KB window into PCI 0x00820000+)         │
│  0xB000-0xBFFF  NVMe ASQ/ACQ + PCIe TLP engine                      │
│  0xC000-0xEFFF  MMIO: Various controllers                           │
│  0xF000-0xFFFF  NVMe data (4KB window into PCI 0x00200000+)         │
└─────────────────────────────────────────────────────────────────────┘
```

### DMA Engine 1: USB Descriptor DMA (0x9092)

**Purpose:** Transfer USB descriptors from firmware ROM to USB hardware buffer

```
Source:      Code ROM (descriptor tables at 0x0627, 0x58CF, etc.)
Destination: USB hardware buffer (feeds directly to USB PHY)
Trigger:     Write 0x01 to 0x9092
Completion:  Check 0xE712 bits 0-1
```

**Data Flow:**
```
Firmware ROM ──DMA──> USB TX Buffer ──USB PHY──> Host
  (0x0627)           (hardware)
```

**Key Registers:**
```
0x905B-905C  Source address (16-bit ROM address)
0x9004-9005  Transfer length (16-bit)
0x9092       Trigger (write 0x01)
0xE712       Completion status
```

**Not for bulk data** - only for enumeration descriptors (<256 bytes).

---

### DMA Engine 2: SCSI/Bulk DMA (0xCE00-0xCE9F) - **THE FAST PATH**

**Purpose:** Transfer bulk data between USB and internal SRAM using PCI addresses

```
Source:      USB bulk endpoint OR internal SRAM
Destination: Internal SRAM OR USB bulk endpoint
Addressing:  32-bit PCI address space (0x00200000+)
Trigger:     Write 0x03 to 0xCE00
Completion:  Poll 0xCE89 or 0xCE00
```

**This is the high-speed path that enables USB3 throughput.**

**Data Flow (USB OUT - Host to Device):**
```
Host ──USB3 Bulk──> USB HW ──SCSI DMA──> SRAM[PCI addr]
                                          (0x00200000+)
```

**Data Flow (USB IN - Device to Host):**
```
SRAM[PCI addr] ──SCSI DMA──> USB HW ──USB3 Bulk──> Host
 (0x00200000+)
```

**Key Registers:**
```
0xCE76  BUF_ADDR0  - PCI address byte 0 (LSB)
0xCE77  BUF_ADDR1  - PCI address byte 1
0xCE78  BUF_ADDR2  - PCI address byte 2
0xCE79  BUF_ADDR3  - PCI address byte 3 (MSB)
0xCE75  BUF_LEN    - Transfer length
0xCE00  DMA_CTRL   - Write 0x03 to start, poll for 0x00
0xCE89  DMA_STATE  - State machine (bits: ready, success, complete)
```

**Address Calculation:**
```c
// Firmware calculates PCI address from SCSI LBA:
pci_addr = 0x00200000 + (lba * 512);

// Then writes to CE76-CE79:
REG_SCSI_BUF_ADDR0 = (pci_addr >> 0) & 0xFF;
REG_SCSI_BUF_ADDR1 = (pci_addr >> 8) & 0xFF;
REG_SCSI_BUF_ADDR2 = (pci_addr >> 16) & 0xFF;
REG_SCSI_BUF_ADDR3 = (pci_addr >> 24) & 0xFF;
```

**The 8051 CPU never touches bulk data bytes** - it only configures addresses.

---

### DMA Engine 3: PCIe/Vendor DMA (0xB296)

**Purpose:** Transfer data between XDATA and USB buffer for E4/E5 vendor commands

```
Source:      XDATA (any address 0x0000-0xFFFF)
Destination: USB buffer at 0x8000
Trigger:     Write 0x08 to 0xB296
Completion:  Check 0xB296 bits 1-2
```

**Data Flow (E4 Read):**
```
XDATA[addr] ──DMA──> USB buffer (0x8000) ──USB──> Host
                         (4KB)
```

**Key Registers:**
```
0x910F  ADDR_HI   - Source address high byte
0x9110  ADDR_MID  - Source address mid byte
0x9111  ADDR_LO   - Source address low byte
0x910E  SIZE      - Transfer size (max 255 bytes)
0xB296  TRIGGER   - Write 0x08 to start
0xB296  STATUS    - Bit 1=complete, Bit 0=error
```

**Limited to XDATA window** - cannot access full PCI address space.
**Slow** - only 255 bytes per transfer.

---

### DMA Engine 4: Flash DMA (0xC8A9)

**Purpose:** Transfer between SPI flash and XDATA buffer

```
Source:      SPI flash (external chip)
Destination: Flash buffer at 0x7000-0x7FFF
Trigger:     Write 0x01 to 0xC8A9
Completion:  Poll 0xC8A9 bit 0 until clear
```

**Data Flow:**
```
SPI Flash ──DMA──> XDATA[0x7000-0x7FFF]
                       (4KB buffer)
```

**Not relevant for USBGPU runtime** - only used for config/firmware loading.

---

### DMA Engine 5: Internal Transfer DMA (0xC8B8, 0xCC89)

**Purpose:** Block transfers within XDATA memory

```
Source:      XDATA address
Destination: XDATA address
Trigger:     Write 0x01 to 0xC8B8
Completion:  Poll 0xC8D6 bit 2
```

**Key Registers:**
```
0xC8B0  DMA_MODE      - Mode configuration
0xC8B4-B5 XFER_CNT    - Transfer count (16-bit)
0xC8B6  CHAN_CTRL     - Direction, enable
0xC8B8  TRIGGER       - Write 0x01 to start
0xC8D6  STATUS        - Bit 2=done, Bit 3=error
```

**Used for:** Internal memory copies, register block operations.

---

### Summary: DMA Source/Destination Matrix

| DMA Engine | Source | Destination | Addressing | Max Size |
|------------|--------|-------------|------------|----------|
| USB Descriptor | ROM | USB HW | 16-bit ROM | ~256 B |
| **SCSI/Bulk** | **USB/SRAM** | **SRAM/USB** | **32-bit PCI** | **~8 MB** |
| PCIe/Vendor | XDATA | USB buf | 24-bit XDATA | 255 B |
| Flash | SPI chip | XDATA | 24-bit flash | 4 KB |
| Internal | XDATA | XDATA | 16-bit XDATA | 64 KB |

### Critical Insight for USBGPU

**The SCSI/Bulk DMA (CE76-CE79) is the ONLY path for high-speed transfers.**

- It uses **32-bit PCI addresses** (0x00200000+), NOT 8051 XDATA addresses
- The 8051's 0x8000 and 0xF000 windows are just **4KB views** into this larger space
- The hardware DMA moves data directly between USB and SRAM at **GB/s speeds**
- The 8051 CPU only writes addresses and triggers - **never touches data bytes**

```
USB3 Host                           Internal SRAM
    │                                    │
    │  USB bulk packets                  │
    │  (1024 bytes each)                 │
    ▼                                    ▼
┌───────────┐                     ┌──────────────┐
│ USB       │   Hardware DMA      │  0x00200000  │
│ Controller│ ════════════════════│  Data Buffer │
│           │   (CE76-79 addr)    │  (6-8 MB)    │
└───────────┘                     └──────────────┘
                                        ▲
                                        │ 4KB window
                                        ▼
                                  ┌──────────────┐
                                  │ 8051 XDATA   │
                                  │ 0x8000-0x8FFF│
                                  │ (CPU access) │
                                  └──────────────┘
```

---

### 3. PCIe TLP Engine (0xB200-0xB4FF) - **ESSENTIAL**

Generates PCIe transactions to the GPU. **Required for GPU communication.**

| Register Range | Purpose | USBGPU Need |
|----------------|---------|-------------|
| 0xB210-0xB220 | TLP format, address, data | Required |
| 0xB22A-0xB22D | Completion status/data | Required |
| 0xB234-0xB24E | Extended link state | Optional |
| 0xB250-0xB254 | Doorbell, trigger | Required |
| 0xB296 | PCIe status | Required |
| 0xB401-0xB482 | Tunnel control | USB4 only |

**Key Registers:**
```
0xB210  PCIE_FMT_TYPE   - TLP type (0x00=MemRd, 0x40=MemWr)
0xB213  PCIE_TLP_CTRL   - Enable TLP (write 0x01)
0xB217  PCIE_BYTE_EN    - Byte enables (0x0F = all 4)
0xB218-B21B PCIE_ADDR   - 32-bit GPU address
0xB220  PCIE_DATA       - 4-byte write data
0xB22C  PCIE_CPL_DATA   - 4-byte read completion data
0xB254  PCIE_TRIGGER    - Send TLP (write 0x0F)
0xB296  PCIE_STATUS     - Bit 1=complete, 0=error
```

**Limitation:** Only 4 bytes per TLP. No hardware DMA to GPU - all CPU-initiated.

---

### 4. Internal DMA/Transfer Engine (0xC8B0-0xCCFF) - **ESSENTIAL**

Internal memory block transfers and CPU DMA control.

| Register Range | Purpose | USBGPU Need |
|----------------|---------|-------------|
| 0xC8B0-0xC8D9 | DMA engine core | Required |
| 0xCC80-0xCC9B | CPU DMA control | Required |
| 0xCCD8-0xCCDB | Secondary transfer DMA | Optional |

**Key Registers:**
```
0xC8B8  DMA_TRIGGER     - Start internal DMA
0xC8D6  DMA_STATUS      - Bit 2=done, bit 3=error
0xCC89  XFER_DMA_CMD    - Transfer DMA command/status
```

---

### 5. Timer System (0xCC10-0xCC24) - **REQUIRED**

Hardware timers for delays and timeouts.

| Timer | Address | Purpose | USBGPU Need |
|-------|---------|---------|-------------|
| Timer0 | 0xCC10-0xCC13 | System tick, ISR | Required |
| Timer1 | 0xCC16-0xCC19 | Protocol timeout | Required |
| Timer2 | 0xCC1C-0xCC1F | USB timing | Required |
| Timer3 | 0xCC22-0xCC24 | Idle timeout | Optional |

**Key Registers:**
```
0xCC11  TIMER0_CSR      - Bit 0=enable, 1=expired, 2=clear
0xCC12-13 THRESHOLD     - 16-bit count value
```

---

### 6. Interrupt Controller (0xC800-0xC80F) - **REQUIRED**

Manages hardware interrupts to 8051.

| Register | Purpose | USBGPU Need |
|----------|---------|-------------|
| 0xC800 | Interrupt status | Required |
| 0xC801 | Interrupt enable | Required |
| 0xC802 | USB interrupt status | Required |
| 0xC806 | System interrupt status | Required |
| 0xC80A | PCIe/NVMe interrupt status | Required |

---

### 7. USB Endpoint Buffer (0xD800-0xDFFF) - **REQUIRED**

Hardware buffer for USB endpoint data.

| Register | Purpose | USBGPU Need |
|----------|---------|-------------|
| 0xD800-0xD80F | EP control/CSW | Required |
| 0xD803-0xD804 | Buffer pointer (16-bit) | Required |
| 0xDE30-0xDE36 | Extended EP config | Optional |

---

### 8. SPI Flash Controller (0xC89F-0xC8AE) - **MINIMAL**

For firmware/config storage. Minimal use in USBGPU.

| Register | Purpose | USBGPU Need |
|----------|---------|-------------|
| 0xC8A9 | Flash CSR (bit 0=busy) | Config read only |
| 0xC8AA | Flash command | Config read only |

**Simplification:** Read-only config at boot, no runtime flash access needed.

---

### 9. NVMe Controller (0xC400-0xC5FF) - **NOT NEEDED**

NVMe command processing and queue management.

| Register Range | Purpose | USBGPU Need |
|----------------|---------|-------------|
| 0xC400-0xC44F | NVMe command engine | Not needed |
| 0xC450-0xC52F | Queue management | Not needed |
| 0xC4C0-0xC4CA | SCSI-NVMe translation | Not needed |
| 0xC4ED-0xC4EF | NVMe DMA control | Not needed |

**Remove entirely** - USBGPU doesn't need NVMe protocol.

---

### 10. NVMe Queue Buffers (0xA000-0xB1FF) - **REPURPOSE**

XDATA regions for NVMe queues. **Can be repurposed for USBGPU.**

| Region | Original Use | USBGPU Use |
|--------|--------------|------------|
| 0xA000-0xAFFF (4KB) | NVMe I/O Submission Queue | GPU command queue |
| 0xB000-0xB0FF (256B) | NVMe Admin Submission Queue | General use |
| 0xB100-0xB1FF (256B) | NVMe Admin Completion Queue | GPU completion |

---

### 11. UART (0xC000-0xC00F) - **OPTIONAL**

Debug output. Keep for development, remove for production.

---

### 12. I2C Controller (0xC870-0xC87F) - **NOT NEEDED**

For external EEPROM/sensors. Not used in USBGPU.

---

### 13. PHY Controllers (0xC200-0xC6FF) - **MINIMAL**

USB and PCIe PHY configuration. Mostly init-time only.

| Block | Address | USBGPU Need |
|-------|---------|-------------|
| Link/PHY Control | 0xC200-0xC2FF | Init only |
| PHY Extended | 0xC600-0xC6FF | Init only |
| PHY Completion | 0xE300-0xE3FF | Debug only |

---

## USBGPU Firmware Architecture

### Minimal Required Blocks

```
┌────────────────────────────────────────────────────────────────┐
│                     USBGPU Firmware                            │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐          │
│  │USB Handler  │   │DMA Engine   │   │PCIe Engine  │          │
│  │             │   │             │   │             │          │
│  │- Vendor Cmd │   │- Bulk IN    │   │- MemWr TLP  │          │
│  │- Bulk EP    │   │- Bulk OUT   │   │- MemRd TLP  │          │
│  │- Descriptors│   │- 32-bit Addr│   │- Config Spc │          │
│  └─────────────┘   └─────────────┘   └─────────────┘          │
│        │                 │                 │                   │
│        ▼                 ▼                 ▼                   │
│  ┌──────────────────────────────────────────────────┐         │
│  │              Register Interface                   │         │
│  │  USB: 0x9000    DMA: 0xCE00    PCIe: 0xB200      │         │
│  └──────────────────────────────────────────────────┘         │
│                          │                                     │
│                          ▼                                     │
│  ┌──────────────────────────────────────────────────┐         │
│  │                 Minimal Support                   │         │
│  │  Timers: 0xCC10   Interrupts: 0xC800   Init only │         │
│  └──────────────────────────────────────────────────┘         │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### Code Size Estimate

| Component | Current FW | USBGPU FW | Notes |
|-----------|------------|-----------|-------|
| USB Core | ~8 KB | ~4 KB | Remove MSC class |
| SCSI Layer | ~12 KB | 0 KB | Remove entirely |
| NVMe Layer | ~16 KB | 0 KB | Remove entirely |
| DMA Engine | ~4 KB | ~2 KB | Keep bulk path |
| PCIe TLP | ~6 KB | ~3 KB | Keep MemR/W |
| Init/Config | ~4 KB | ~2 KB | Simplify |
| **Total** | ~50 KB | **~11 KB** | **~78% reduction** |

### Register Summary for USBGPU

**Must Have (Essential):**
```
0x9000-0x93FF  USB Controller
0xB210-0xB296  PCIe TLP Engine
0xCE00-0xCE9F  SCSI/Bulk DMA
0xC800-0xC80A  Interrupt Controller
0xCC10-0xCC24  Timers
0xD800-0xD80F  EP Buffer
```

**Init Only:**
```
0x92C0-0x92FF  Power Management
0xC200-0xC2FF  PHY Control
0xC89F-0xC8AE  Flash (config read)
```

**Remove/Ignore:**
```
0xC400-0xC5FF  NVMe Controller
0xA000-0xB1FF  NVMe Queues (repurpose as data)
0xC870-0xC87F  I2C
0xEC00-0xECFF  NVMe Events
```

### Simplified Data Flow

```
                    USBGPU Data Flow
                    ================

USB Bulk OUT (Host → GPU):
  Host ──USB3──> EP2 ──DMA──> SRAM[0x00200000]
                              8051 reads SRAM window
                              8051 ──TLP──> GPU BAR

USB Bulk IN (GPU → Host):
  GPU ──DMA──> SRAM[0x00200000]  (GPU as bus master)
             or
  8051 ──TLP──> GPU BAR → 8051 → SRAM  (slow path)
  SRAM ──DMA──> EP1 ──USB3──> Host

Register Access (E5 Write):
  Host ──USB3──> Setup Pkt (0x9E00)
                8051 parses → writes register

Memory Read (E4 Read):
  Host ──USB3──> CDB (0x910D)
                8051 reads XDATA → DMA → USB IN
```

### Implementation Priority

1. **Phase 1: USB Vendor Class**
   - Replace Mass Storage descriptors with Vendor class
   - Simplify enumeration (no SCSI inquiry, mode sense, etc.)
   - Keep bulk endpoints for data transfer

2. **Phase 2: Direct Bulk Protocol**
   - Define simple command format (opcode, address, length)
   - Direct DMA from bulk EP to SRAM
   - Bypass all SCSI/NVMe processing

3. **Phase 3: Fast Read Path**
   - Implement SCSI READ(16) equivalent for bulk reads
   - Or extend E4 to support 64KB reads
   - Use hardware DMA for data movement

4. **Phase 4: GPU Bus Master Integration**
   - Configure GPU as PCIe bus master
   - GPU DMAs directly to SRAM (0x00200000)
   - 8051 only handles command/status, not data
