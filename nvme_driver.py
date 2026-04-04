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

IO_SQ_XDATA     = 0xF000       # I/O SQ: same as admin SQ (reused after admin init)
IO_CQ_XDATA     = 0xA000       # I/O CQ: same page as admin CQ
# Data buffer at offset 0x200 (512B) into 0xF000 page, after the SQ sector
DATA_BUF_XDATA  = 0xF200
DATA_BUF_DMA    = 0x00200200

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
    def __init__(self, handle, bar0_addr):
        self.handle = handle
        self.bar0 = bar0_addr
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

        # Disable
        cc = self.reg_read(NVME_CC)
        if cc & 1:
            self.reg_write(NVME_CC, cc & ~1)
            for _ in range(200):
                if not (self.reg_read(NVME_CSTS) & 1): break
                time.sleep(0.01)

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
        # Ring SQ0 tail doorbell
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
                # Ring CQ0 head doorbell
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
        # Create I/O CQ (QID=1) — use same DMA page as admin CQ
        self.admin_cmd(NVME_ADMIN_CREATE_IO_CQ,
                       cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=0x01, prp1=ADMIN_CQ_DMA)
        # Create I/O SQ (QID=1, CQID=1)
        self.admin_cmd(NVME_ADMIN_CREATE_IO_SQ,
                       cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=(1 << 16) | 0x01, prp1=ADMIN_SQ_DMA)
        # Reset I/O CQ: zero out the shared CQ region and reset phase tracking
        for i in range(QUEUE_DEPTH * 16):
            xdata_write(self.handle, IO_CQ_XDATA + i, 0)
        self.io_sq_tail = 0
        self.io_cq_head = 0
        self.io_cq_phase = 1
        print("  I/O queues created (QID=1)")

    # ========================================================================
    # I/O commands — write SQE via xdata, ring doorbell, poll CQ, DMA data
    # ========================================================================
    def _submit_io(self, opcode, nsid, lba, count, prp1, prp2=0):
        self.io_cid = (self.io_cid + 1) & 0xFFFF
        slot = self.io_sq_tail
        sqe = struct.pack('<IIIIIIQQ', opcode | (self.io_cid << 16), nsid, 0, 0, 0, 0, prp1, prp2)
        sqe += struct.pack('<QHHHH', lba, (count - 1) & 0xFFFF, 0, 0, 0)
        sqe = sqe.ljust(64, b'\x00')
        buf = bytearray(512)
        buf[slot * 64 : slot * 64 + 64] = sqe
        self.dma_write_sram(bytes(buf))
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        self.reg_write(0x1000 + 2 * self.doorbell_stride, self.io_sq_tail)
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
                # Ring I/O CQ1 head doorbell (offset = 3 * stride)
                self.reg_write(0x1000 + 3 * self.doorbell_stride, self.io_cq_head)
                if status:
                    raise RuntimeError(f"I/O failed: status=0x{status:04X}")
                return
            time.sleep(0.001)
        raise TimeoutError("I/O completion timeout")

    def read_sector(self, lba, count=1):
        """Read sectors. Returns bytes."""
        nbytes = count * self.sector_size
        assert nbytes <= 4096
        self._submit_io(NVME_IO_READ, nsid=1, lba=lba, count=count, prp1=DATA_BUF_DMA)
        self._wait_io()
        return self.xread(DATA_BUF_XDATA, nbytes)

    def write_sector(self, lba, data):
        """Write sectors. DMA SQE + data in one bulk transfer."""
        count = len(data) // self.sector_size
        assert count * self.sector_size == len(data) and len(data) <= 3584
        self.io_cid = (self.io_cid + 1) & 0xFFFF
        slot = self.io_sq_tail
        # SQE with PRP1 = DATA_BUF_DMA (0x00200200 = XDATA 0xF200, after SQ sector)
        sqe = struct.pack('<IIIIIIQQ', NVME_IO_WRITE | (self.io_cid << 16), 1, 0, 0, 0, 0, DATA_BUF_DMA, 0)
        sqe += struct.pack('<QHHHH', lba, (count - 1) & 0xFFFF, 0, 0, 0)
        sqe = sqe.ljust(64, b'\x00')
        # First 512B: SQ sector with SQE at slot offset
        sq_sector = bytearray(512)
        sq_sector[slot * 64 : slot * 64 + 64] = sqe
        # Remaining: data padded to 512B
        padded = data + b'\x00' * ((-len(data)) % 512)
        # DMA both: SQ at 0xF000, data at 0xF200
        self.dma_write_sram(bytes(sq_sector) + padded)
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        self.reg_write(0x1000 + 2 * self.doorbell_stride, self.io_sq_tail)
        self._wait_io()


def main():
    handle, ctx = usb_open()
    try:
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        if ltssm != 0x78:
            print(f"LTSSM=0x{ltssm:02X}, run pcie_bringup.py first")
            return 1

        print("=== Bridge Setup ===")
        nvme_bus = setup_bridges(handle, gpu_bus=4)
        devices = enumerate_bus(handle, nvme_bus)
        if not devices:
            print("No NVMe found"); return 1
        vid, did = devices[0][3], devices[0][4]
        print(f"  NVMe: [{vid:04X}:{did:04X}] on bus {nvme_bus}")

        bars = assign_bars(handle, nvme_bus)
        bar0_addr = bars[0][0]
        print(f"  BAR0 = 0x{bar0_addr:X}")

        nvme = NVMeDriver(handle, bar0_addr)

        print("\n=== Init ===")
        nvme.init_controller()
        print("\n=== Identify ===")
        nvme.identify_controller()
        nsze, ss = nvme.identify_namespace(1)
        print("\n=== I/O Queues ===")
        nvme.create_io_queues()

        print("\n=== Read LBA 0 ===")
        d = nvme.read_sector(0)
        print(f"  {d[:32].hex()}")

        test_lba = nsze - 16
        print(f"\n=== Write/Read Test LBA {test_lba} ===")
        orig = nvme.read_sector(test_lba)
        pattern = os.urandom(512)
        nvme.write_sector(test_lba, pattern)
        rb = nvme.read_sector(test_lba)
        if rb == pattern:
            print("  PASS")
        else:
            print("  FAIL")
            for i in range(512):
                if rb[i] != pattern[i]:
                    print(f"  byte {i}: got 0x{rb[i]:02X} exp 0x{pattern[i]:02X}")
                    break
        nvme.write_sector(test_lba, orig)
        assert nvme.read_sector(test_lba) == orig, "restore failed"
        print("  Restored")
        return 0
    finally:
        usb_close(handle, ctx)

if __name__ == "__main__":
    sys.exit(main())
