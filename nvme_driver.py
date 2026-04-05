#!/usr/bin/env python3
"""NVMe driver over ASM2464PD using the hardware NVMe/DMA engine.

Uses the C4xx NVMe registers and B251/B254 doorbell mechanism,
plus xdata reads/writes for SQ/CQ entries and bulk DMA for data.

Usage:
    make -C handmade flash
    python3 pcie/pcie_bringup.py
    python3 nvme_driver.py
"""

import struct, sys, time, ctypes
from pcie.pcie_probe import (
    usb_open, usb_close, xdata_read, xdata_write, xdata_write_bytes,
    pcie_mem_read, pcie_mem_write, setup_bridges, enumerate_bus, assign_bars,
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
IO_CQ_DMA       = 0x00822000

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
        self.io_cq_head = (self.io_cq_head + 1) % QUEUE_DEPTH
        self.hw_doorbell(self.io_cq_head, 0x13)
        return bytes(rxbuf[:rxfer.value])


def main():
    handle, ctx = usb_open()
    try:
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        if ltssm != 0x78:
            print(f"LTSSM=0x{ltssm:02X}, run pcie_bringup.py first")
            return 1

        # pcie_post_train: DMA config + NVMe regs (important!)
        xdata_write(handle, 0xB26C, 0x08); xdata_write(handle, 0xB26D, 0x20)
        xdata_write(handle, 0xB26E, 0x08); xdata_write(handle, 0xB26F, 0x22)

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
            nvme.sector_size = 512
        else:
            nvme.init_controller()
            nvme.create_io_queues()

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
