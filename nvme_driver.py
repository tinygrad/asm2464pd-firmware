#!/usr/bin/env python3
"""NVMe driver over ASM2464PD PCIe TLP engine.

Directly programs the NVMe controller via PCIe memory read/write TLPs,
without using BOT/UAS/SCSI. Requires handmade firmware + pcie_bringup.py.

Usage:
    python3 pcie/pcie_bringup.py && python3 nvme_driver.py
"""

import struct, sys, time, os
from pcie.pcie_probe import (
    usb_open, usb_close, xdata_read, xdata_write,
    pcie_cfg_read, pcie_cfg_write, pcie_mem_read, pcie_mem_write,
    setup_bridges, enumerate_bus, assign_bars,
    PCI_COMMAND, PCI_COMMAND_MEMORY, PCI_COMMAND_MASTER,
    CFGRD1, CFGWR1, MRD32, MWR32,
)

# ============================================================================
# NVMe register offsets (within BAR0)
# ============================================================================
NVME_CAP    = 0x00   # Controller Capabilities (64-bit)
NVME_VS     = 0x08   # Version
NVME_INTMS  = 0x0C   # Interrupt Mask Set
NVME_INTMC  = 0x10   # Interrupt Mask Clear
NVME_CC     = 0x14   # Controller Configuration
NVME_CSTS   = 0x1C   # Controller Status
NVME_AQA    = 0x24   # Admin Queue Attributes
NVME_ASQ    = 0x28   # Admin SQ Base Address (64-bit)
NVME_ACQ    = 0x30   # Admin Completion Queue Base Address (64-bit)
NVME_SQ0TDBL = 0x1000  # SQ0 Tail Doorbell

# NVMe Admin opcodes
NVME_ADMIN_IDENTIFY       = 0x06
NVME_ADMIN_CREATE_IO_CQ   = 0x05
NVME_ADMIN_CREATE_IO_SQ   = 0x01
NVME_ADMIN_SET_FEATURES    = 0x09

# NVMe I/O opcodes
NVME_IO_READ  = 0x02
NVME_IO_WRITE = 0x01

# ============================================================================
# Memory layout
#
# The ASM2464 has internal SRAM accessible via XDATA from the 8051 side, and
# via PCIe DMA addresses from the NVMe device side. The mappings are:
#   XDATA 0xB000 (ASQ)  -> PCIe DMA 0x00800000  (from stock trace)
#   XDATA 0xB100 (ACQ)  -> PCIe DMA 0x00808000  (from stock trace)
#   XDATA 0xA000 (IOSQ) -> PCIe DMA 0x00820000  (from registers.h)
#   XDATA 0xF000 (data) -> PCIe DMA 0x00200000  (from registers.h)
#
# We write queue entries via MWr32 to the DMA addresses (NVMe sees them),
# and read completions via xdata_read from XDATA addresses (host side).
# ============================================================================
# Known XDATA <-> PCIe DMA address mappings:
#   XDATA 0xF000 (4KB) <-> PCIe DMA 0x00200000
#   XDATA 0xA000 (4KB) <-> PCIe DMA 0x00820000
#
# We pack everything into these two known regions:
#   0xF000 region: Admin SQ (256B) + Admin CQ (64B) + I/O CQ (64B) = ~384B
#   0xA000 region: I/O SQ (256B, at start) + data buffer (remaining ~3.7KB)
#
# Layout in 0xF000 (DMA 0x00200000):
#   0xF000 +0x000: Admin SQ  (4 entries * 64B = 256B)
#   0xF000 +0x100: Admin CQ  (4 entries * 16B = 64B)
#   0xF000 +0x140: I/O CQ    (4 entries * 16B = 64B)
#   0xF000 +0x180: (free)
#
# Layout in 0xA000 (DMA 0x00820000):
#   0xA000 +0x000: I/O SQ    (4 entries * 64B = 256B)
#   0xA000 +0x100: Data buf  (3.75KB for sector read/write + identify)

# Known XDATA <-> PCIe DMA address mappings:
#   XDATA 0xF000 (4KB) <-> PCIe DMA 0x00200000
#   XDATA 0xA000 (4KB) <-> PCIe DMA 0x00820000
#
# NVMe CQ/SQ bases must be page-aligned (4KB). PRP1 only needs dword alignment.
#
# Layout:
#   DMA 0x00200000 (XDATA 0xF000): Admin SQ + I/O SQ (first 512B) + data buf (offset 0x200)
#   DMA 0x00820000 (XDATA 0xA000): Admin CQ + I/O CQ (shared, sequential use)
#
# Admin SQ: 4 entries * 64B = 256B at DMA 0x00200000 (XDATA 0xF000)
# I/O SQ:   reuses same region after admin init
# Admin CQ: 4 entries * 16B = 64B at DMA 0x00820000 (XDATA 0xA000)
# I/O CQ:   reuses same region after admin init
# Data buf: at DMA 0x00200200 (XDATA 0xF200), 3.5KB usable

ADMIN_SQ_XDATA  = 0xF000
ADMIN_SQ_DMA    = 0x00200000
ADMIN_CQ_XDATA  = 0xA000
ADMIN_CQ_DMA    = 0x00820000

IO_SQ_XDATA     = ADMIN_SQ_XDATA
IO_SQ_DMA       = ADMIN_SQ_DMA
IO_CQ_XDATA     = ADMIN_CQ_XDATA
IO_CQ_DMA       = ADMIN_CQ_DMA

DATA_BUF_XDATA  = 0xF200
DATA_BUF_DMA    = 0x00200200

QUEUE_DEPTH = 4  # keep small, SRAM is limited

class NVMeDriver:
    def __init__(self, handle, bar0_addr, nvme_bus):
        self.handle = handle
        self.bar0 = bar0_addr
        self.bus = nvme_bus
        self.admin_sq_tail = 0
        self.admin_cq_head = 0
        self.admin_cq_phase = 1
        self.admin_cid = 0
        self.io_sq_tail = 0
        self.io_cq_head = 0
        self.io_cq_phase = 1
        self.io_cid = 0
        self.doorbell_stride = 4  # default, read from CAP

    def reg_read(self, offset, size=4):
        return pcie_mem_read(self.handle, self.bar0 + offset, size=size)

    def reg_write(self, offset, value, size=4):
        pcie_mem_write(self.handle, self.bar0 + offset, value, size=size)

    def mem_write32(self, addr, value):
        """Write 32-bit value to PCIe DMA address (goes to ASM2464 internal SRAM)."""
        pcie_mem_write(self.handle, addr, value, size=4)

    def mem_read32(self, addr):
        """Read 32-bit from PCIe DMA address — NOT usable for internal SRAM!"""
        return pcie_mem_read(self.handle, addr, size=4)

    def xdata_read32(self, xdata_addr):
        """Read 32-bit value from XDATA (little-endian)."""
        data = xdata_read(self.handle, xdata_addr, 4)
        return struct.unpack('<I', data)[0]

    def xdata_write32(self, xdata_addr, value):
        """Write 32-bit value to XDATA (little-endian, byte-at-a-time)."""
        for i in range(4):
            xdata_write(self.handle, xdata_addr + i, (value >> (8 * i)) & 0xFF)

    def xdata_read_buf(self, xdata_addr, size):
        """Read bytes from XDATA."""
        data = b""
        for off in range(0, size, 64):
            chunk = min(64, size - off)
            data += xdata_read(self.handle, xdata_addr + off, chunk)
        return data[:size]

    def init_controller(self):
        """Full NVMe controller initialization."""
        # Read CAP
        cap_lo = self.reg_read(NVME_CAP)
        cap_hi = self.reg_read(NVME_CAP + 4)
        cap = cap_lo | (cap_hi << 32)
        mqes = (cap & 0xFFFF) + 1  # Maximum Queue Entries Supported
        self.doorbell_stride = 4 << ((cap >> 32) & 0xF)  # DSTRD
        timeout = ((cap >> 24) & 0xFF) * 500  # CAP.TO in ms
        print(f"  CAP: 0x{cap:016X}")
        print(f"  MQES={mqes}, doorbell_stride={self.doorbell_stride}, timeout={timeout}ms")

        vs = self.reg_read(NVME_VS)
        print(f"  VS: {(vs>>16)&0xFFFF}.{(vs>>8)&0xFF}.{vs&0xFF}")

        # 1. Disable controller (CC.EN = 0)
        cc = self.reg_read(NVME_CC)
        if cc & 0x01:
            print("  Disabling controller...")
            self.reg_write(NVME_CC, cc & ~0x01)
            # Wait for CSTS.RDY = 0
            for _ in range(100):
                csts = self.reg_read(NVME_CSTS)
                if not (csts & 0x01):
                    break
                time.sleep(0.01)
            else:
                raise RuntimeError(f"Controller disable timeout, CSTS=0x{self.reg_read(NVME_CSTS):08X}")

        # 2. Configure Admin Queues
        # AQA: ACQS | ASQS (0-based)
        aqa = ((QUEUE_DEPTH - 1) << 16) | (QUEUE_DEPTH - 1)
        self.reg_write(NVME_AQA, aqa)
        print(f"  AQA = 0x{aqa:08X}")

        # ASQ base address (PCIe DMA address)
        self.reg_write(NVME_ASQ, ADMIN_SQ_DMA & 0xFFFFFFFF)
        self.reg_write(NVME_ASQ + 4, (ADMIN_SQ_DMA >> 32) & 0xFFFFFFFF)
        print(f"  ASQ = 0x{ADMIN_SQ_DMA:08X}")

        # ACQ base address (PCIe DMA address)
        self.reg_write(NVME_ACQ, ADMIN_CQ_DMA & 0xFFFFFFFF)
        self.reg_write(NVME_ACQ + 4, (ADMIN_CQ_DMA >> 32) & 0xFFFFFFFF)
        print(f"  ACQ = 0x{ADMIN_CQ_DMA:08X}")

        # Zero out queue memory via XDATA
        for i in range(0, QUEUE_DEPTH * 64):
            xdata_write(self.handle, ADMIN_SQ_XDATA + i, 0)
        for i in range(0, QUEUE_DEPTH * 16):
            xdata_write(self.handle, ADMIN_CQ_XDATA + i, 0)

        # 3. Enable controller
        # CC: CSS=0 (NVM), MPS=0 (4KB pages), AMS=0, SHN=0, IOSQES=6 (64B), IOCQES=4 (16B), EN=1
        cc = (6 << 16) | (4 << 20) | 0x01  # IOSQES=6, IOCQES=4, EN=1
        self.reg_write(NVME_CC, cc)
        print(f"  CC = 0x{cc:08X} (enabling...)")

        # 4. Wait for CSTS.RDY = 1
        for i in range(max(200, timeout // 10)):
            csts = self.reg_read(NVME_CSTS)
            if csts & 0x01:
                print(f"  CSTS = 0x{csts:08X} — controller ready ({i*10}ms)")
                return True
            if csts & 0x02:  # CFS - fatal
                raise RuntimeError(f"Controller fatal status: CSTS=0x{csts:08X}")
            time.sleep(0.01)
        raise RuntimeError(f"Controller enable timeout, CSTS=0x{self.reg_read(NVME_CSTS):08X}")

    def _submit_admin_cmd(self, opcode, nsid=0, cdw10=0, cdw11=0, cdw12=0, cdw13=0, prp1=0, prp2=0):
        """Submit a command to the admin submission queue via XDATA."""
        self.admin_cid = (self.admin_cid + 1) & 0xFFFF
        slot = self.admin_sq_tail
        base = ADMIN_SQ_XDATA + slot * 64

        # Build 64-byte submission queue entry (SQE) via XDATA writes
        self.xdata_write32(base + 0, opcode | (self.admin_cid << 16))
        self.xdata_write32(base + 4, nsid)
        self.xdata_write32(base + 8, 0)
        self.xdata_write32(base + 12, 0)
        self.xdata_write32(base + 16, 0)
        self.xdata_write32(base + 20, 0)
        self.xdata_write32(base + 24, prp1 & 0xFFFFFFFF)
        self.xdata_write32(base + 28, (prp1 >> 32) & 0xFFFFFFFF)
        self.xdata_write32(base + 32, prp2 & 0xFFFFFFFF)
        self.xdata_write32(base + 36, (prp2 >> 32) & 0xFFFFFFFF)
        self.xdata_write32(base + 40, cdw10)
        self.xdata_write32(base + 44, cdw11)
        self.xdata_write32(base + 48, cdw12)
        self.xdata_write32(base + 52, cdw13)
        self.xdata_write32(base + 56, 0)
        self.xdata_write32(base + 60, 0)

        # Ring doorbell
        self.admin_sq_tail = (self.admin_sq_tail + 1) % QUEUE_DEPTH
        self.reg_write(NVME_SQ0TDBL, self.admin_sq_tail)
        return self.admin_cid

    def _wait_admin_completion(self, cid, timeout_s=5.0):
        """Poll admin CQ via XDATA for completion of given command ID."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            base = ADMIN_CQ_XDATA + self.admin_cq_head * 16
            dw3 = self.xdata_read32(base + 12)
            phase = (dw3 >> 16) & 0x01
            if phase == self.admin_cq_phase:
                cq_cid = dw3 & 0xFFFF
                status = (dw3 >> 17) & 0x7FFF
                dw0 = self.xdata_read32(base + 0)
                # Advance CQ head
                self.admin_cq_head = (self.admin_cq_head + 1) % QUEUE_DEPTH
                if self.admin_cq_head == 0:
                    self.admin_cq_phase ^= 1
                # Ring CQ doorbell (SQ0TDBL + 1 * stride)
                self.reg_write(NVME_SQ0TDBL + self.doorbell_stride, self.admin_cq_head)
                if status != 0:
                    raise RuntimeError(f"Admin cmd 0x{cid:04X} failed: status=0x{status:04X}")
                return dw0
            time.sleep(0.005)
        raise TimeoutError(f"Admin completion timeout for CID=0x{cid:04X}")

    def admin_cmd(self, opcode, nsid=0, cdw10=0, cdw11=0, cdw12=0, cdw13=0, prp1=0, prp2=0):
        """Submit admin command and wait for completion."""
        cid = self._submit_admin_cmd(opcode, nsid, cdw10, cdw11, cdw12, cdw13, prp1, prp2)
        return self._wait_admin_completion(cid)

    def identify_controller(self):
        """Send Identify Controller (CNS=1) and return basic info."""
        # Identify data will be DMA'd to DATA_BUF_DMA, readable at DATA_BUF_XDATA
        self.admin_cmd(NVME_ADMIN_IDENTIFY, cdw10=1, prp1=DATA_BUF_DMA)

        # Read key fields from XDATA
        data = self.xdata_read_buf(DATA_BUF_XDATA, 64)
        vid = struct.unpack_from('<H', data, 0)[0]
        ssvid = struct.unpack_from('<H', data, 2)[0]
        sn = data[4:24].decode('ascii', errors='replace').strip()
        mn = data[24:64].decode('ascii', errors='replace').strip()

        print(f"  VID=0x{vid:04X} SSVID=0x{ssvid:04X}")
        print(f"  SN: {sn}")
        print(f"  MN: {mn}")
        return vid, sn, mn

    def identify_namespace(self, nsid=1):
        """Send Identify Namespace and return size info."""
        self.admin_cmd(NVME_ADMIN_IDENTIFY, nsid=nsid, cdw10=0, prp1=DATA_BUF_DMA)

        # Read from XDATA
        data = self.xdata_read_buf(DATA_BUF_XDATA, 132)
        nsze = struct.unpack_from('<Q', data, 0)[0]
        flbas = data[26]
        lba_fmt_idx = flbas & 0x0F
        lbaf = struct.unpack_from('<I', data, 128 + lba_fmt_idx * 4)[0]
        lba_ds = (lbaf >> 16) & 0xFF
        sector_size = 1 << lba_ds

        size_gb = (nsze * sector_size) / (1024**3)
        print(f"  NSZE={nsze} sectors, sector_size={sector_size}, capacity={size_gb:.1f} GB")
        return nsze, sector_size

    def create_io_queues(self):
        """Create one I/O completion queue and one I/O submission queue."""
        # Zero I/O queue memory via XDATA
        for i in range(0, QUEUE_DEPTH * 64):
            xdata_write(self.handle, IO_SQ_XDATA + i, 0)
        for i in range(0, QUEUE_DEPTH * 16):
            xdata_write(self.handle, IO_CQ_XDATA + i, 0)

        # Set Number of Queues feature (1 I/O SQ + 1 I/O CQ)
        self.admin_cmd(NVME_ADMIN_SET_FEATURES, cdw10=0x07, cdw11=0x00010001)

        # Create I/O Completion Queue (QID=1)
        cdw10 = ((QUEUE_DEPTH - 1) << 16) | 1
        cdw11 = 0x01  # PC=1 (physically contiguous)
        self.admin_cmd(NVME_ADMIN_CREATE_IO_CQ, cdw10=cdw10, cdw11=cdw11, prp1=IO_CQ_DMA)

        # Create I/O Submission Queue (QID=1, CQID=1)
        cdw10 = ((QUEUE_DEPTH - 1) << 16) | 1
        cdw11 = (1 << 16) | 0x01  # CQID=1, PC=1
        self.admin_cmd(NVME_ADMIN_CREATE_IO_SQ, cdw10=cdw10, cdw11=cdw11, prp1=IO_SQ_DMA)
        print("  I/O queues created (QID=1)")

    def _submit_io_cmd(self, opcode, nsid, lba, count, prp1, prp2=0):
        """Submit a command to I/O submission queue 1 via XDATA."""
        self.io_cid = (self.io_cid + 1) & 0xFFFF
        slot = self.io_sq_tail
        base = IO_SQ_XDATA + slot * 64

        self.xdata_write32(base + 0, opcode | (self.io_cid << 16))
        self.xdata_write32(base + 4, nsid)
        self.xdata_write32(base + 8, 0)
        self.xdata_write32(base + 12, 0)
        self.xdata_write32(base + 16, 0)
        self.xdata_write32(base + 20, 0)
        self.xdata_write32(base + 24, prp1 & 0xFFFFFFFF)
        self.xdata_write32(base + 28, (prp1 >> 32) & 0xFFFFFFFF)
        self.xdata_write32(base + 32, prp2 & 0xFFFFFFFF)
        self.xdata_write32(base + 36, (prp2 >> 32) & 0xFFFFFFFF)
        self.xdata_write32(base + 40, lba & 0xFFFFFFFF)
        self.xdata_write32(base + 44, (lba >> 32) & 0xFFFFFFFF)
        self.xdata_write32(base + 48, (count - 1) & 0xFFFF)
        self.xdata_write32(base + 52, 0)
        self.xdata_write32(base + 56, 0)
        self.xdata_write32(base + 60, 0)

        # Ring I/O SQ doorbell (SQ1 = SQ0TDBL + 2 * stride)
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        self.reg_write(NVME_SQ0TDBL + 2 * self.doorbell_stride, self.io_sq_tail)
        return self.io_cid

    def _wait_io_completion(self, cid, timeout_s=5.0):
        """Poll I/O CQ via XDATA for completion."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            base = IO_CQ_XDATA + self.io_cq_head * 16
            dw3 = self.xdata_read32(base + 12)
            phase = (dw3 >> 16) & 0x01
            if phase == self.io_cq_phase:
                status = (dw3 >> 17) & 0x7FFF
                self.io_cq_head = (self.io_cq_head + 1) % QUEUE_DEPTH
                if self.io_cq_head == 0:
                    self.io_cq_phase ^= 1
                # Ring I/O CQ doorbell (CQ1 = SQ0TDBL + 3 * stride)
                self.reg_write(NVME_SQ0TDBL + 3 * self.doorbell_stride, self.io_cq_head)
                if status != 0:
                    raise RuntimeError(f"I/O cmd CID=0x{cid:04X} failed: status=0x{status:04X}")
                return
            time.sleep(0.001)
        raise TimeoutError(f"I/O completion timeout for CID=0x{cid:04X}")

    def read_sector(self, lba, count=1):
        """Read sectors from the NVMe drive. Returns bytes."""
        assert count * 512 <= 4096, "max 4KB per read (single PRP page)"
        cid = self._submit_io_cmd(NVME_IO_READ, nsid=1, lba=lba, count=count, prp1=DATA_BUF_DMA)
        self._wait_io_completion(cid)

        # Read data from XDATA buffer
        return self.xdata_read_buf(DATA_BUF_XDATA, count * 512)

    def write_sector(self, lba, data):
        """Write sectors to the NVMe drive."""
        count = len(data) // 512
        assert count * 512 == len(data), "data must be multiple of 512"
        assert len(data) <= 4096, "max 4KB per write (single PRP page)"

        # Write data to XDATA buffer
        for i, b in enumerate(data):
            xdata_write(self.handle, DATA_BUF_XDATA + i, b)

        cid = self._submit_io_cmd(NVME_IO_WRITE, nsid=1, lba=lba, count=count, prp1=DATA_BUF_DMA)
        self._wait_io_completion(cid)


def main():
    handle, ctx = usb_open()
    print(f"Opened device")

    try:
        # Check LTSSM
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        if ltssm != 0x78:
            print(f"PCIe link not at L0 (LTSSM=0x{ltssm:02X}). Run pcie_bringup.py first.")
            return 1

        # Setup bridges and find NVMe
        print("\n=== Bridge Setup ===")
        nvme_bus = setup_bridges(handle, gpu_bus=4)

        print(f"\n=== Bus {nvme_bus} ===")
        devices = enumerate_bus(handle, nvme_bus)
        if not devices:
            print("No devices found!")
            return 1

        for bus, dev, fn, vid, did, cc, sc, ht in devices:
            print(f"  {bus:02X}:{dev:02X}.{fn} [{vid:04X}:{did:04X}] class={cc:02X}.{sc:02X}")

        # Assign BARs
        print("\n=== BAR Assignment ===")
        bars = assign_bars(handle, nvme_bus, verbose=True)
        bar0_info = bars.get(0)
        if not bar0_info:
            print("No BAR0 found!")
            return 1

        bar0_addr, bar0_size = bar0_info
        print(f"  NVMe BAR0: 0x{bar0_addr:X} size=0x{bar0_size:X}")

        # Initialize NVMe
        nvme = NVMeDriver(handle, bar0_addr, nvme_bus)

        print("\n=== NVMe Controller Init ===")
        nvme.init_controller()

        print("\n=== Identify Controller ===")
        vid, sn, mn = nvme.identify_controller()

        print("\n=== Identify Namespace 1 ===")
        nsze, sector_size = nvme.identify_namespace(1)

        print("\n=== Create I/O Queues ===")
        nvme.create_io_queues()

        # Test: read sector 0
        print("\n=== Read LBA 0 ===")
        data = nvme.read_sector(0, 1)
        print(f"  First 64 bytes: {data[:64].hex()}")
        # Check for MBR/GPT signature
        if data[510:512] == b'\x55\xAA':
            print("  MBR signature found!")
        if data[:8] == b'EFI PART':
            print("  GPT header found!")

        # Test: write/read/verify at high LBA
        test_lba = nsze - 16
        print(f"\n=== Write/Read Test at LBA {test_lba} ===")

        # Save original
        orig = nvme.read_sector(test_lba, 1)
        print(f"  Original: {orig[:32].hex()}")

        # Write test pattern
        pattern = os.urandom(512)
        print(f"  Writing pattern: {pattern[:16].hex()}...")
        nvme.write_sector(test_lba, pattern)

        # Read back
        readback = nvme.read_sector(test_lba, 1)
        print(f"  Readback: {readback[:32].hex()}")

        if readback == pattern:
            print("  PASS: write/read verified!")
        else:
            print("  FAIL: data mismatch!")
            for i in range(512):
                if readback[i] != pattern[i]:
                    print(f"  First diff at byte {i}: got 0x{readback[i]:02X} expected 0x{pattern[i]:02X}")
                    break

        # Restore original
        print("  Restoring original data...")
        nvme.write_sector(test_lba, orig)
        verify = nvme.read_sector(test_lba, 1)
        if verify == orig:
            print("  Original data restored.")
        else:
            print("  WARNING: restore verification failed!")

        return 0

    finally:
        usb_close(handle, ctx)


if __name__ == "__main__":
    sys.exit(main())
