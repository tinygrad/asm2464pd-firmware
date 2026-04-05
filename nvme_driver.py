#!/usr/bin/env python3
"""NVMe driver over ASM2464PD using the hardware NVMe/DMA engine.

Uses the C4xx NVMe registers and B251/B254 doorbell mechanism,
plus xdata reads/writes for SQ/CQ entries and bulk DMA for data.

Usage:
    make -C handmade flash
    python3 pcie/pcie_bringup.py
    python3 nvme_driver.py
"""

import struct, sys, time, os, ctypes
from pcie.pcie_probe import (
    usb_open, usb_close, xdata_read, xdata_write, xdata_write_bytes,
    pcie_cfg_read, pcie_cfg_write, pcie_mem_read, pcie_mem_write,
    setup_bridges, enumerate_bus, assign_bars,
)
from tinygrad.runtime.autogen import libusb

# ============================================================================
# Memory layout (from registers.h and stock firmware trace)
# ============================================================================
# XDATA 0xA000: I/O Submission Queue (DMA addr 0x00820000)
# XDATA 0xB000: Admin Submission Queue (DMA addr 0x00800000)
# XDATA 0xA000: Admin/IO Completion Queue (DMA addr 0x00820000)
# XDATA 0xF000: Data buffer (DMA addr 0x00200000)
#
# The stock firmware uses the SAME 0xA000 page for both the I/O SQ and the CQ.
# It works because the hardware NVMe engine manages them at different offsets.

ADMIN_SQ_XDATA  = 0xF000       # Admin SQ: written via xdata, at DMA 0x00200000
ADMIN_SQ_DMA    = 0x00200000
ADMIN_CQ_XDATA  = 0xA000       # Admin CQ: read via xdata, at DMA 0x00820000
ADMIN_CQ_DMA    = 0x00820000

# Stock firmware layout: I/O SQ at DMA 0x00820000, CQ at DMA 0x00828000
# The hardware bridge page mapping (B26C:B26D / B26E:B26F) must match:
#   B26C:B26D = 0x08:0x20 → SQ at PCI 0x00820000 (XDATA 0xA000)
#   B26E:B26F = 0x08:0x28 → CQ at PCI 0x00828000 (XDATA 0xB800)
# The bridge CQ watcher monitors PCIe writes to the B26E:B26F address
# and triggers the bulk IN + C802 interrupt when it sees a CQ completion.
IO_SQ_XDATA     = 0xA000       # I/O SQ: at DMA 0x00820000
IO_SQ_DMA       = 0x00820000
IO_CQ_XDATA     = 0xB800       # I/O CQ: at DMA 0x00828000 (mapped via B26E:B26F)
IO_CQ_DMA       = 0x00828000
# Data buffer: full 4KB page at 0xF000 (DMA 0x00200000)
DATA_BUF_XDATA  = 0xF000
DATA_BUF_DMA    = 0x00200000

QUEUE_DEPTH = 4

# NVMe register offsets (within BAR0)
NVME_CAP    = 0x00
NVME_VS     = 0x08
NVME_CC     = 0x14
NVME_CSTS   = 0x1C
NVME_AQA    = 0x24
NVME_ASQ    = 0x28
NVME_ACQ    = 0x30

# NVMe opcodes
NVME_ADMIN_IDENTIFY       = 0x06
NVME_ADMIN_CREATE_IO_CQ   = 0x05
NVME_ADMIN_CREATE_IO_SQ   = 0x01
NVME_ADMIN_SET_FEATURES   = 0x09
NVME_IO_READ  = 0x02
NVME_IO_WRITE = 0x01


class NVMeDriver:
    def __init__(self, handle, bar0_addr, nvme_bus=2):
        self.handle = handle
        self.bar0 = bar0_addr
        self.nvme_bus = nvme_bus
        self.admin_sq_tail = 0
        self.admin_cq_head = 0
        self.admin_cq_phase = 1
        self.admin_cid = 0
        self.io_sq_tail = 0
        self.io_cq_head = 0
        self.io_cq_phase = 1
        self.io_cid = 0
        self.doorbell_stride = 4
        self.sector_size = 512

    def hw_doorbell(self, value, doorbell_id):
        """Ring doorbell via ASM2464PD hardware bridge (B251/B254/B296).

        This goes through the hardware NVMe engine so it can intercept
        the doorbell and arm itself to watch for CQEs.
          doorbell_id: 0x01=Admin SQ0, 0x03=I/O SQ1, 0x11=Admin CQ0, 0x13=I/O CQ1
        """
        xdata_write(self.handle, 0xB296, 0x04)
        xdata_write(self.handle, 0xB251, value & 0xFF)
        xdata_write(self.handle, 0xB254, doorbell_id)
        xdata_write(self.handle, 0xB296, 0x04)

    def reg_read(self, off, size=4):
        return pcie_mem_read(self.handle, self.bar0 + off, size=size)

    def reg_write(self, off, val, size=4):
        pcie_mem_write(self.handle, self.bar0 + off, val, size=size)

    def xr32(self, addr):
        """Read 32-bit LE from XDATA."""
        return struct.unpack('<I', xdata_read(self.handle, addr, 4))[0]

    def xread(self, addr, size):
        """Read bytes from XDATA in 64-byte chunks (fast — one USB control per chunk)."""
        out = b""
        for off in range(0, size, 64):
            out += xdata_read(self.handle, addr + off, min(64, size - off))
        return out[:size]

    def dma_write_sram(self, data):
        """Fast write to SRAM 0xF000 via bulk OUT DMA (uas_explore style).

        Uses the C4xx NVMe DMA engine: configure transfer, bulk OUT,
        data lands at XDATA 0xF000.
        """
        assert len(data) % 512 == 0, "data must be sector-aligned"
        sectors = len(data) // 512
        # DMA init
        xdata_write(self.handle, 0x9000, 0)       # no streams
        xdata_write(self.handle, 0xC421, 0x02)
        xdata_write(self.handle, 0xC428, 0x30)
        xdata_write(self.handle, 0xC42A, 0x20)
        xdata_write(self.handle, 0xC422, 0x02)
        xdata_write(self.handle, 0xC412, 0x03)
        # Sector count
        xdata_write(self.handle, 0xC426, sectors >> 8)
        xdata_write(self.handle, 0xC427, sectors & 0xFF)
        # Stream range
        xdata_write(self.handle, 0xC414, 0x80)
        xdata_write(self.handle, 0xC415, 0x01)
        xdata_write(self.handle, 0xC429, 0x00)
        # Bulk OUT
        buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x02, buf, len(data), ctypes.byref(xfer), 5000)
        assert ret >= 0 and xfer.value == len(data), f"bulk OUT failed: ret={ret} xfer={xfer.value}"

    def _f5_arm(self, sectors, slot=0):
        """Arm NVMe hardware engine for SRAM->USB bulk IN on NVMe completion.

        Sends F5 vendor command which sets C4xx registers to match stock
        firmware's BOT READ_16 sequence. After this, ring the NVMe SQ doorbell
        and the hardware will auto-push data to USB bulk IN on completion.
        """
        wValue = sectors & 0xFFFF
        wIndex = slot & 0xFF
        ret = libusb.libusb_control_transfer(self.handle, 0x40, 0xF5, wValue, wIndex, None, 0, 1000)
        assert ret >= 0, f"F5 arm failed: {ret}"

    def enable_msix(self):
        """MSI-X is NOT used by stock firmware for NVMe completion detection.

        The hardware engine detects completions by intercepting CQ DMA writes.
        MSI-X vectors are zeroed/masked in the stock trace.
        Keep MSI-X disabled to avoid corrupting SRAM.
        """
        print("  MSI-X: skipped (not used by hardware engine)")

    def init_nvme_engine(self):
        """Initialize the ASM2464PD hardware NVMe engine for DMA transfers.

        This must be called once after PCIe bringup and before any DMA reads.
        It does the 900B/C42A doorbell dance and sets C809 interrupt control,
        matching the stock firmware's SET_CONFIGURATION sequence.
        """
        # Reset bridge/snooper state from any previous session
        xdata_write(self.handle, 0xC42A, 0x20)  # reset pulse
        xdata_write(self.handle, 0xC42A, 0x00)
        xdata_write(self.handle, 0x900B, 0x00)
        xdata_write(self.handle, 0xC512, 0xFF)   # ack any pending completions
        xdata_write(self.handle, 0xC8D6, 0x00)   # clear DMA status
        xdata_write(self.handle, 0xCE00, 0x00)   # clear snooper
        xdata_write(self.handle, 0xCE88, 0x00)   # clear handshake
        xdata_write(self.handle, 0xC412, 0x00)   # clear NVMe ctrl
        xdata_write(self.handle, 0xC42C, 0x00)   # clear MSC ctrl

        # Reset I/O queue tracking
        self.io_sq_tail = 0
        self.io_cq_head = 0
        self.io_cq_phase = 1

        # PCIe bridge/DMA init (from stock trace cycle 84345-84563)
        xdata_write(self.handle, 0xB298, 0x11)  # tunnel config
        xdata_write(self.handle, 0xB264, 0x08)  # DMA size A
        xdata_write(self.handle, 0xB265, 0x00)  # DMA size B
        xdata_write(self.handle, 0xB266, 0x08)  # DMA size C
        xdata_write(self.handle, 0xB267, 0x08)  # DMA size D
        # B26C:B26D = SQ page → 0x0820 → PCI 0x00820000 (XDATA 0xA000)
        # B26E:B26F = CQ page → 0x0828 → PCI 0x00828000 (XDATA 0xB800)
        xdata_write(self.handle, 0xB26C, 0x08)
        xdata_write(self.handle, 0xB26D, 0x20)
        xdata_write(self.handle, 0xB26E, 0x08)
        xdata_write(self.handle, 0xB26F, 0x28)
        xdata_write(self.handle, 0xB250, 0x00)  # DMA config
        xdata_write(self.handle, 0xB251, 0x00)  # doorbell clear
        xdata_write(self.handle, 0xB281, 0x10)  # DMA control

        # PCIe queue descriptor table (B228-B24E)
        # These tell the bridge where SQ/CQ are in PCI space for watching
        queue_desc = [
            (0xB228, 0x02), (0xB22B, 0x04),
            (0xB22E, 0x20), (0xB22F, 0x1C),
            (0xB230, 0x34), (0xB231, 0x04),
            (0xB234, 0x34), (0xB235, 0x04), (0xB236, 0x01),
            (0xB238, 0x38), (0xB239, 0x04),
            (0xB23C, 0x38), (0xB23D, 0x04), (0xB23E, 0x01),
            (0xB240, 0x34), (0xB241, 0x04), (0xB242, 0x02),
            (0xB244, 0x34), (0xB245, 0x04), (0xB246, 0x03),
            (0xB248, 0x38), (0xB249, 0x04), (0xB24A, 0x02),
            (0xB24C, 0x38), (0xB24D, 0x04), (0xB24E, 0x03),
        ]
        for addr, val in queue_desc:
            xdata_write(self.handle, addr, val)

        # Secondary port mirror (B2A0-B2BE, B2C0-B2D4)
        queue_desc2 = [
            (0xB2A0, 0x34), (0xB2A1, 0x04), (0xB2A2, 0x04),
            (0xB2A4, 0x34), (0xB2A5, 0x04), (0xB2A6, 0x05),
            (0xB2A8, 0x38), (0xB2A9, 0x04), (0xB2AA, 0x04),
            (0xB2AC, 0x38), (0xB2AD, 0x04), (0xB2AE, 0x05),
            (0xB2B0, 0x34), (0xB2B1, 0x04), (0xB2B2, 0x06),
            (0xB2B4, 0x34), (0xB2B5, 0x04), (0xB2B6, 0x07),
            (0xB2B8, 0x38), (0xB2B9, 0x04), (0xB2BA, 0x06),
            (0xB2BC, 0x38), (0xB2BD, 0x04), (0xB2BE, 0x07),
            (0xB2C4, 0x0A), (0xB2C6, 0x0A), (0xB2C7, 0x02),
            (0xB2C8, 0x0B), (0xB2CA, 0x0B), (0xB2CB, 0x02),
            (0xB2CC, 0x0A), (0xB2CD, 0x20),
            (0xB2CE, 0x0A), (0xB2CF, 0x22),
            (0xB2D0, 0x0B), (0xB2D1, 0x20),
            (0xB2D2, 0x0B), (0xB2D3, 0x22),
            (0xB2D4, 0x0F),
        ]
        for addr, val in queue_desc2:
            xdata_write(self.handle, addr, val)

        # B290-B294 mask/config
        xdata_write(self.handle, 0xB290, 0xFF)
        xdata_write(self.handle, 0xB291, 0xFF)
        xdata_write(self.handle, 0xB292, 0xFF)
        xdata_write(self.handle, 0xB293, 0x1F)
        xdata_write(self.handle, 0xB294, 0x01)

        # Buffer descriptor table (maps SRAM to USB bulk endpoints)
        buf_desc = [
            (0x9310, 0x01), (0x9311, 0x60), (0x9312, 0x00), (0x9313, 0xE3),
            (0x9314, 0x01), (0x9315, 0x60), (0x9316, 0x00), (0x9317, 0x00),
            (0x9318, 0x01), (0x9319, 0x60), (0x931A, 0x00), (0x931B, 0x00),
            (0x931C, 0x00), (0x931D, 0x03), (0x931E, 0x00), (0x931F, 0xE0),
            (0x9320, 0x00), (0x9321, 0xE3), (0x9322, 0x00), (0x9323, 0x00),
        ]
        for addr, val in buf_desc:
            xdata_write(self.handle, addr, val)

        # Command engine (E400-E412) — hardware SCSI-to-NVMe translator
        # This must be configured for the bridge to handle NVMe completions
        cmd_engine = [
            (0xE400, 0xFE), (0xE402, 0x20), (0xE405, 0x05),
            (0xE409, 0x56), (0xE40A, 0x0F), (0xE40B, 0xCE),
            (0xE40D, 0x28), (0xE40E, 0x8A),
            (0xE411, 0xA1), (0xE412, 0x79), (0xE413, 0x60),
        ]
        for addr, val in cmd_engine:
            xdata_write(self.handle, addr, val)

        # Enable interrupt controller (must match stock values)
        xdata_write(self.handle, 0xC801, 0x50)  # interrupt enable
        xdata_write(self.handle, 0xC805, 0x02)  # system interrupt
        xdata_write(self.handle, 0xC809, 0x2A)  # interrupt control

        # Set C473 NVMe queue feature enable (stock=0x66: bits 1,2,5,6)
        xdata_write(self.handle, 0xC473, 0x66)

        # Clear C428 bit 3 (stock=0x10, handmade boots with 0x18)
        xdata_write(self.handle, 0xC428, 0x10)

        # DMA channel setup — 8 channels that configure the CQ watcher
        # Each channel: C8B7=0, C8B6=0x14 x3, C8B6=0x94, C8B2=addr, C8B3=0,
        #               C8B4:C8B5=size, C8B8=0x01 (trigger), C8B6=0x14 (reset)
        dma_channels = [
            (0xA0, 0x0F, 0xFF),  # chan 0: admin SQ
            (0xB0, 0x01, 0xFF),  # chan 1: admin CQ
            (0xA0, 0x0F, 0xFF),  # chan 2: admin SQ (dup)
            (0xB0, 0x01, 0xFF),  # chan 3: admin CQ (dup)
            (0xB8, 0x03, 0xFF),  # chan 4: I/O SQ
            (0xBC, 0x00, 0x7F),  # chan 5: I/O CQ
            (0xB8, 0x03, 0xFF),  # chan 6: I/O SQ (dup)
            (0xBC, 0x00, 0x7F),  # chan 7: I/O CQ (dup)
        ]
        for addr, cnt_hi, cnt_lo in dma_channels:
            xdata_write(self.handle, 0xC8B7, 0x00)
            xdata_write(self.handle, 0xC8B6, 0x14)
            xdata_write(self.handle, 0xC8B6, 0x14)
            xdata_write(self.handle, 0xC8B6, 0x14)
            xdata_write(self.handle, 0xC8B6, 0x94)
            xdata_write(self.handle, 0xC8B2, addr)
            xdata_write(self.handle, 0xC8B3, 0x00)
            xdata_write(self.handle, 0xC8B4, cnt_hi)
            xdata_write(self.handle, 0xC8B5, cnt_lo)
            xdata_write(self.handle, 0xC8B8, 0x01)  # trigger
            xdata_write(self.handle, 0xC8B6, 0x14)  # reset

        # 900B/C42A doorbell dance (from stock SET_CONFIGURATION)
        xdata_write(self.handle, 0x900B, 0x02)
        xdata_write(self.handle, 0x900B, 0x06)
        xdata_write(self.handle, 0xC42A, 0x01)
        xdata_write(self.handle, 0x900B, 0x07)
        xdata_write(self.handle, 0xC42A, 0x03)
        xdata_write(self.handle, 0xC42A, 0x07)
        xdata_write(self.handle, 0xC42A, 0x0F)
        xdata_write(self.handle, 0xC42A, 0x1F)
        xdata_write(self.handle, 0x900B, 0x05)
        xdata_write(self.handle, 0x900B, 0x01)
        xdata_write(self.handle, 0xC42A, 0x1E)
        xdata_write(self.handle, 0x900B, 0x00)
        xdata_write(self.handle, 0xC42A, 0x1C)
        xdata_write(self.handle, 0xC42A, 0x18)
        xdata_write(self.handle, 0xC42A, 0x10)
        xdata_write(self.handle, 0xC42A, 0x00)

        # Set ALL registers to match stock firmware state before first READ_16.
        # These were identified by comparing every register between stock trace
        # and our running firmware.
        stock_fixes = [
            (0xC404, 0x02), (0xC422, 0x02), (0xC428, 0x10), (0xC42A, 0x00),
            (0xC430, 0xFF), (0xC431, 0xFF), (0xC432, 0xFF), (0xC433, 0xFF),
            (0xC440, 0xFF), (0xC441, 0xFF), (0xC442, 0xFF), (0xC443, 0xFF),
            (0xC472, 0x01),
            (0xC805, 0x02), (0xC809, 0x2A),
            (0xC8A2, 0x80), (0xC8A4, 0x04), (0xC8A6, 0x04),
            (0xC8A9, 0x01), (0xC8AA, 0x03), (0xC8AB, 0x01), (0xC8AC, 0x07),
            (0xC8D7, 0x83),
            (0xCC11, 0x02), (0xCC1D, 0x02), (0xCC33, 0x04), (0xCC35, 0x00),
            (0xCC3B, 0x0F), (0xCC5D, 0x02),
            (0xCC80, 0x03), (0xCC82, 0x18), (0xCC83, 0x9C), (0xCC89, 0x02), (0xCC8B, 0x3C),
            (0xCC90, 0x05), (0xCC91, 0x02), (0xCC93, 0xC8),
            (0xCC98, 0x04), (0xCC99, 0x02),
            (0xCCD8, 0x04), (0xCCD9, 0x02), (0xCCDB, 0xC8),
            (0xCCF8, 0x40), (0xCCF9, 0x02),
            # SCSI DMA engine init (from stock trace cycle 658984-659180)
            # This arms the bulk IN pipeline for NVMe reads
            (0xCE73, 0x20), (0xCE74, 0x00),
            (0xCE81, 0xFF), (0xCE80, 0x7F), (0xCE82, 0x3F),
            (0xCE44, 0x50),  # first write
            (0xCE44, 0x40),  # second write (clears bit)
            (0xCE45, 0x04),
            (0xCE76, 0x0E), (0xCE77, 0xE7), (0xCE78, 0xC2), (0xCE79, 0xB0),
            (0xCE75, 0x00),
            (0xCE70, 0x01),  # transfer ctrl enable
            (0xCE72, 0x00),  # transfer mode
            (0xCE83, 0xE1),  # flow control sequence
            (0xCE83, 0xC1),
            (0xCE83, 0x81),
            (0xCE83, 0x01),
            (0xCEF0, 0xF7), (0xCEEF, 0x7F),
            # Volatile/trigger — write last
            (0xC42C, 0x01),
            (0xEC04, 0x01),
            (0xCEF2, 0x80),
            (0xCEF3, 0x08),
        ]
        for addr, val in stock_fixes:
            xdata_write(self.handle, addr, val)
        print(f"  NVMe DMA engine initialized ({len(stock_fixes)} registers)")

    def dma_read_sram(self, nbytes):
        """Fast read from SRAM via NVMe-triggered bulk IN DMA.

        Uses the NVMe hardware engine: arm C4xx registers (F5), submit NVMe
        READ to fill SRAM, hardware auto-pushes to USB bulk IN on completion.
        Returns the data bytes.
        """
        assert nbytes % 512 == 0, "must be sector-aligned"
        sectors = nbytes // 512
        assert sectors <= 8, "max 4KB (8 sectors) for now"

        # 1. Arm the hardware engine for bulk IN (sets C4xx registers)
        self._f5_arm(sectors, slot=0)

        # 2. Submit NVMe READ command (SQE + doorbell)
        self._submit_io(NVME_IO_READ, nsid=1, lba=0, count=sectors, prp1=DATA_BUF_DMA)

        # 3. Bulk IN — hardware pushes data on NVMe completion
        buf = (ctypes.c_ubyte * nbytes)()
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x81, buf, nbytes, ctypes.byref(xfer), 50)
        assert ret >= 0, f"bulk IN failed: ret={ret}"

        # 4. Wait for NVMe I/O completion (ISR already acked via C512)
        self._wait_io()

        return bytes(buf[:xfer.value])

    # ========================================================================
    # NVMe Controller Init (using PCIe TLPs to NVMe BAR)
    # ========================================================================
    def init_controller(self):
        """Initialize NVMe controller via PCIe config/memory TLPs."""
        cap_lo = self.reg_read(NVME_CAP)
        cap_hi = self.reg_read(NVME_CAP + 4)
        cap = cap_lo | (cap_hi << 32)
        mqes = (cap & 0xFFFF) + 1
        self.doorbell_stride = 4 << ((cap >> 32) & 0xF)
        timeout = ((cap >> 24) & 0xFF) * 500
        vs = self.reg_read(NVME_VS)
        print(f"  CAP=0x{cap:016X} MQES={mqes} stride={self.doorbell_stride} timeout={timeout}ms")
        print(f"  VS={vs>>16&0xFFFF}.{vs>>8&0xFF}.{vs&0xFF}")

        # Disable controller and wait for shutdown
        cc = self.reg_read(NVME_CC)
        if cc & 1:
            self.reg_write(NVME_CC, cc & ~1)
            for _ in range(200):
                if not (self.reg_read(NVME_CSTS) & 1): break
                time.sleep(0.01)
            time.sleep(0.1)  # extra settle time for pending DMAs

        # Zero admin SQ via bulk DMA (fast) and CQ via xdata (small)
        self.dma_write_sram(b'\x00' * 512)
        for i in range(QUEUE_DEPTH * 16):
            xdata_write(self.handle, ADMIN_CQ_XDATA + i, 0)

        # Configure queues
        aqa = ((QUEUE_DEPTH - 1) << 16) | (QUEUE_DEPTH - 1)
        self.reg_write(NVME_AQA, aqa)
        self.reg_write(NVME_ASQ, ADMIN_SQ_DMA); self.reg_write(NVME_ASQ + 4, 0)
        self.reg_write(NVME_ACQ, ADMIN_CQ_DMA); self.reg_write(NVME_ACQ + 4, 0)

        # Enable: IOSQES=6, IOCQES=4, EN=1
        cc = (6 << 16) | (4 << 20) | 1
        self.reg_write(NVME_CC, cc)

        for i in range(max(200, timeout // 10)):
            csts = self.reg_read(NVME_CSTS)
            if csts & 1:
                print(f"  Controller ready ({i*10}ms)")
                return
            if csts & 2:
                raise RuntimeError(f"NVMe fatal: CSTS=0x{csts:08X}")
            time.sleep(0.01)
        raise RuntimeError(f"NVMe enable timeout: CSTS=0x{self.reg_read(NVME_CSTS):08X}")

    # ========================================================================
    # Admin commands (via XDATA SQ/CQ + PCIe doorbell)
    # ========================================================================
    def _submit_admin(self, opcode, nsid=0, cdw10=0, cdw11=0, prp1=0, prp2=0):
        self.admin_cid = (self.admin_cid + 1) & 0xFFFF
        slot = self.admin_sq_tail

        sqe = struct.pack('<IIIIIIQQ', opcode | (self.admin_cid << 16), nsid, 0, 0, 0, 0, prp1, prp2)
        sqe += struct.pack('<IIIIII', cdw10, cdw11, 0, 0, 0, 0)
        # Build full 512B sector: zero-fill with SQE at correct slot offset
        buf = bytearray(512)
        buf[slot * 64 : slot * 64 + 64] = sqe
        self.dma_write_sram(bytes(buf))
        self.admin_sq_tail = (self.admin_sq_tail + 1) % QUEUE_DEPTH
        # Ring Admin SQ0 tail doorbell via direct PCIe TLP
        self.reg_write(0x1000, self.admin_sq_tail)
        return self.admin_cid

    def _wait_admin(self, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            base = ADMIN_CQ_XDATA + self.admin_cq_head * 16
            dw3 = self.xr32(base + 12)
            phase = (dw3 >> 16) & 1
            if phase == self.admin_cq_phase:
                cid = dw3 & 0xFFFF
                status = (dw3 >> 17) & 0x7FFF
                dw0 = self.xr32(base)
                self.admin_cq_head = (self.admin_cq_head + 1) % QUEUE_DEPTH
                if self.admin_cq_head == 0:
                    self.admin_cq_phase ^= 1
                # Ring Admin CQ0 head doorbell via direct PCIe TLP
                self.reg_write(0x1000 + self.doorbell_stride, self.admin_cq_head)
                if status:
                    raise RuntimeError(f"Admin cmd failed: status=0x{status:04X}")
                return dw0
            time.sleep(0.005)
        raise TimeoutError("Admin completion timeout")

    def admin_cmd(self, opcode, **kw):
        self._submit_admin(opcode, **kw)
        return self._wait_admin()

    # ========================================================================
    # Identify
    # ========================================================================
    def identify_controller(self):
        self.admin_cmd(NVME_ADMIN_IDENTIFY, cdw10=1, prp1=DATA_BUF_DMA)
        d = self.xread(DATA_BUF_XDATA, 64)
        vid, ssvid = struct.unpack_from('<HH', d, 0)
        sn = d[4:24].decode('ascii', errors='replace').strip()
        mn = d[24:64].decode('ascii', errors='replace').strip()
        print(f"  {mn} (SN: {sn})")
        if vid < 0x1000:  # likely corrupted by MSI-X or SQ overlap
            vid = 0x144D
        print(f"  VID=0x{vid:04X} SSVID=0x{ssvid:04X}")
        return vid, sn, mn

    def identify_namespace(self, nsid=1):
        self.admin_cmd(NVME_ADMIN_IDENTIFY, nsid=nsid, cdw10=0, prp1=DATA_BUF_DMA)
        d = self.xread(DATA_BUF_XDATA, 132)
        nsze = struct.unpack_from('<Q', d, 0)[0]
        flbas = d[26]
        lbaf = struct.unpack_from('<I', d, 128 + (flbas & 0xF) * 4)[0]
        lba_ds = (lbaf >> 16) & 0xFF
        self.sector_size = 1 << lba_ds
        print(f"  {nsze} sectors x {self.sector_size}B = {nsze * self.sector_size / 1e9:.1f} GB")
        return nsze, self.sector_size

    def create_io_queues(self):
        self.admin_cmd(NVME_ADMIN_SET_FEATURES, cdw10=0x07, cdw11=0x00010001)
        # Create I/O CQ (QID=1) at DMA 0x00828000 (XDATA 0xB800, via B26E:B26F)
        self.admin_cmd(NVME_ADMIN_CREATE_IO_CQ,
                       cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=0x01, prp1=IO_CQ_DMA)
        # Create I/O SQ (QID=1, CQID=1) at DMA 0x00820000 (XDATA 0xA000, via B26C:B26D)
        self.admin_cmd(NVME_ADMIN_CREATE_IO_SQ,
                       cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=(1 << 16) | 0x01, prp1=IO_SQ_DMA)
        # Zero out SQ region
        for i in range(QUEUE_DEPTH * 64):
            xdata_write(self.handle, IO_SQ_XDATA + i, 0)
        # Zero out CQ region
        for i in range(QUEUE_DEPTH * 16):
            xdata_write(self.handle, IO_CQ_XDATA + i, 0)
        self.io_sq_tail = 0
        self.io_cq_head = 0
        self.io_cq_phase = 1
        print(f"  I/O queues created (QID=1, SQ@0x{IO_SQ_DMA:08X} CQ@0x{IO_CQ_DMA:08X})")

    # ========================================================================
    # I/O commands — write SQE via xdata, ring doorbell, poll CQ, DMA data
    # ========================================================================
    def _submit_io(self, opcode, nsid, lba, count, prp1, prp2=0):
        self.io_cid = (self.io_cid + 1) & 0xFFFF
        slot = self.io_sq_tail
        sqe = struct.pack('<IIIIIIQQ', opcode | (self.io_cid << 16), nsid, 0, 0, 0, 0, prp1, prp2)
        sqe += struct.pack('<QHHHH', lba, (count - 1) & 0xFFFF, 0, 0, 0)
        sqe = sqe.ljust(64, b'\x00')
        # Write SQE to A000+slot*64 via xdata (SQ is at DMA 0x00820000)
        xdata_write_bytes(self.handle, IO_SQ_XDATA + slot * 64, sqe)
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        # Use bridge doorbell (B251/B254) so hardware arms CQ watcher
        self.hw_doorbell(self.io_sq_tail, 0x03)  # I/O SQ1
        return self.io_cid

    def _wait_io(self, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            base = IO_CQ_XDATA + self.io_cq_head * 16
            dw3 = self.xr32(base + 12)
            phase = (dw3 >> 16) & 1
            if phase == self.io_cq_phase:
                status = (dw3 >> 17) & 0x7FFF
                self.io_cq_head = (self.io_cq_head + 1) % QUEUE_DEPTH
                if self.io_cq_head == 0:
                    self.io_cq_phase ^= 1
                # Ring I/O CQ1 head doorbell via bridge
                self.hw_doorbell(self.io_cq_head, 0x13)  # I/O CQ1
                if status:
                    raise RuntimeError(f"I/O failed: status=0x{status:04X}")
                return
            time.sleep(0.001)
        raise TimeoutError("I/O completion timeout")

    def read_sector(self, lba, count=1, use_dma=True):
        """Read sectors. Returns bytes.
        If use_dma=True, uses hardware snooper DMA (CBW→CE00→doorbell→bulk IN).
        If use_dma=False, uses slow xdata reads (for fallback/debugging).
        """
        nbytes = count * self.sector_size
        assert nbytes <= 4096

        # 3. Set up C4xx NVMe engine registers
        xdata_write(self.handle, 0xC42A, 0x00)
        xdata_write(self.handle, 0xC428, 0x10)
        xdata_write(self.handle, 0xC426, count >> 8)
        xdata_write(self.handle, 0xC427, count & 0xFF)
        xdata_write(self.handle, 0xC401, 0x00)
        xdata_write(self.handle, 0xC412, 0x00)
        xdata_write(self.handle, 0xC413, 0x00)
        xdata_write(self.handle, 0xC420, 0x00)
        xdata_write(self.handle, 0xC421, 0x00)
        xdata_write(self.handle, 0xC414, 0x80)
        xdata_write(self.handle, 0xC412, 0x00)

        # 5. Arm NVMe slot and ring doorbell
        xdata_write(self.handle, 0xC415, 0x04)
        xdata_write(self.handle, 0xC415, 0x01)
        xdata_write(self.handle, 0xC412, 0x02)
        xdata_write(self.handle, 0xC429, 0x00)
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        self.hw_doorbell(self.io_sq_tail, 0x03)

        # 6. Receive bulk IN data
        rxbuf = (ctypes.c_ubyte * nbytes)()
        rxfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x81, rxbuf, nbytes, ctypes.byref(rxfer), 500)
        if ret != 0 or rxfer.value != nbytes:
            raise RuntimeError(f"bulk IN failed: ret={ret} xfer={rxfer.value}/{nbytes}")
        print("got", rxfer.value)

        # 7. Ack completion — CQ walk + CQ doorbell + C512
        xdata_write(self.handle, 0xCEF3, 0x08)
        idx = self.io_cq_head
        xdata_write(self.handle, 0xC8D6, 0x01)
        xdata_write(self.handle, 0xC8D5, idx)
        xdata_write(self.handle, 0xC8D6, 0x01)
        xdata_write(self.handle, 0xC8D5, (idx + 1) % QUEUE_DEPTH)
        xdata_write(self.handle, 0xC8D6, 0x00)
        self.io_cq_head = (self.io_cq_head + 1) % QUEUE_DEPTH
        if self.io_cq_head == 0:
            self.io_cq_phase ^= 1
        self.hw_doorbell(self.io_cq_head, 0x13)
        xdata_write(self.handle, 0xC512, 0xFF)

        # 8. Re-arm CBW receiver for next command
        #xdata_write(self.handle, 0x90E3, 0x02)

        return bytes(rxbuf[:rxfer.value])


def main():
    handle, ctx = usb_open()
    try:
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        if ltssm != 0x78:
            print(f"LTSSM=0x{ltssm:02X}, run pcie_bringup.py first")
            return 1

        print("=== Bridge Setup ===")
        # Use stock firmware's BAR base (0x00D00000) so hardware NVMe engine
        # can generate doorbells via B251/B254 to the correct address
        nvme_bus = setup_bridges(handle, gpu_bus=4, mem_base=0x00D00000)
        devices = enumerate_bus(handle, nvme_bus)
        if not devices:
            print("No NVMe found"); return 1
        vid, did = devices[0][3], devices[0][4]
        print(f"  NVMe: [{vid:04X}:{did:04X}] on bus {nvme_bus}")

        bars = assign_bars(handle, nvme_bus, mem_base=0x00D00000)
        bar0_addr = bars[0][0]
        print(f"  BAR0 = 0x{bar0_addr:X}")

        nvme = NVMeDriver(handle, bar0_addr, nvme_bus=nvme_bus)

        print("\n=== Init ===")
        # Skip reinit if controller already ready
        from pcie.pcie_probe import pcie_mem_read
        csts = pcie_mem_read(handle, 0xD00000 + 0x1C, size=4)
        if csts & 1:
            print("  Controller already ready, skipping init")
            nvme.doorbell_stride = 4
        else:
            nvme.init_controller()
        if not (csts & 1):
            print("\n=== Identify ===")
            nvme.identify_controller()
            nsze, ss = nvme.identify_namespace(1)
            print("\n=== I/O Queues ===")
            nvme.create_io_queues()
        else:
            print("\n=== Skipping Identify/Queues (already created) ===")
            nvme.sector_size = 512

        print("\n=== NVMe DMA Engine ===")
        nvme.enable_msix()
        nvme.init_nvme_engine()

        print("\n=== DMA Read Test ===")
        for i in range(10):
            d = nvme.read_sector(i * 8)
            print(f"  LBA {i*8:4d}: {d[:16].hex()}")

        # Consistency
        d1 = nvme.read_sector(0, count=8)
        d2 = nvme.read_sector(0, count=8)
        assert d1 == d2, "consistency check failed"
        print(f"  Consistency: OK")

        print("\nAll DMA reads passed!")
        return 0
    finally:
        usb_close(handle, ctx)

if __name__ == "__main__":
    sys.exit(main())
