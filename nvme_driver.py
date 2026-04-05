#!/usr/bin/env python3
"""NVMe driver for ASM2464PD — hardware DMA bulk IN reads.

Usage:
    make -C handmade flash
    python3 nvme_driver.py
"""

import struct, sys, time, ctypes
from pcie.pcie_probe import (
    usb_open, usb_close, xdata_read, xdata_write, xdata_write_bytes,
    pcie_mem_read, pcie_mem_write, setup_bridges, enumerate_bus, assign_bars,
)
from tinygrad.runtime.autogen import libusb

# Memory layout
ADMIN_SQ_XDATA  = 0xF000
ADMIN_SQ_DMA    = 0x00200000
ADMIN_CQ_XDATA  = 0xA000
ADMIN_CQ_DMA    = 0x00820000
IO_SQ_XDATA     = 0xA000
IO_SQ_DMA       = 0x00820000
IO_CQ_DMA       = 0x00822000
QUEUE_DEPTH     = 4

# NVMe BAR0 offsets
NVME_CAP, NVME_VS, NVME_CC, NVME_CSTS = 0x00, 0x08, 0x14, 0x1C
NVME_AQA, NVME_ASQ, NVME_ACQ = 0x24, 0x28, 0x30

# NVMe opcodes
NVME_ADMIN_SET_FEATURES   = 0x09
NVME_ADMIN_CREATE_IO_CQ   = 0x05
NVME_ADMIN_CREATE_IO_SQ   = 0x01
NVME_IO_READ  = 0x02


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
        self.doorbell_stride = 4
        self.sector_size = 512

    def reg_read(self, off, size=4):
        return pcie_mem_read(self.handle, self.bar0 + off, size=size)

    def reg_write(self, off, val, size=4):
        pcie_mem_write(self.handle, self.bar0 + off, val, size=size)

    def xr32(self, addr):
        return struct.unpack('<I', xdata_read(self.handle, addr, 4))[0]

    def hw_doorbell(self, value, doorbell_id):
        """Ring doorbell via bridge (B251/B254)."""
        xdata_write(self.handle, 0xB296, 0x04)
        xdata_write(self.handle, 0xB251, value & 0xFF)
        xdata_write(self.handle, 0xB254, doorbell_id)
        xdata_write(self.handle, 0xB296, 0x04)

    def dma_write_sram(self, data):
        """Write to SRAM 0xF000 via bulk OUT DMA."""
        assert len(data) % 512 == 0
        sectors = len(data) // 512
        xdata_write(self.handle, 0x9000, 0)
        xdata_write(self.handle, 0xC421, 0x02)
        xdata_write(self.handle, 0xC428, 0x30)
        xdata_write(self.handle, 0xC42A, 0x20)
        xdata_write(self.handle, 0xC422, 0x02)
        xdata_write(self.handle, 0xC412, 0x03)
        xdata_write(self.handle, 0xC426, sectors >> 8)
        xdata_write(self.handle, 0xC427, sectors & 0xFF)
        xdata_write(self.handle, 0xC414, 0x80)
        xdata_write(self.handle, 0xC415, 0x01)
        xdata_write(self.handle, 0xC429, 0x00)
        buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x02, buf, len(data), ctypes.byref(xfer), 5000)
        assert ret >= 0 and xfer.value == len(data), f"bulk OUT failed: ret={ret} xfer={xfer.value}"

    # === Admin commands (direct PCIe TLPs) ===

    def _admin_cmd(self, opcode, nsid=0, cdw10=0, cdw11=0, prp1=0, prp2=0):
        self.admin_cid = (self.admin_cid + 1) & 0xFFFF
        slot = self.admin_sq_tail
        sqe = struct.pack('<IIIIIIQQ', opcode | (self.admin_cid << 16), nsid, 0, 0, 0, 0, prp1, prp2)
        sqe += struct.pack('<IIIIII', cdw10, cdw11, 0, 0, 0, 0)
        buf = bytearray(512)
        buf[slot * 64 : slot * 64 + 64] = sqe
        self.dma_write_sram(bytes(buf))
        self.admin_sq_tail = (self.admin_sq_tail + 1) % QUEUE_DEPTH
        self.reg_write(0x1000, self.admin_sq_tail)
        # Poll CQ
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            dw3 = self.xr32(ADMIN_CQ_XDATA + self.admin_cq_head * 16 + 12)
            if ((dw3 >> 16) & 1) == self.admin_cq_phase:
                self.admin_cq_head = (self.admin_cq_head + 1) % QUEUE_DEPTH
                if self.admin_cq_head == 0:
                    self.admin_cq_phase ^= 1
                self.reg_write(0x1000 + self.doorbell_stride, self.admin_cq_head)
                status = (dw3 >> 17) & 0x7FFF
                if status:
                    raise RuntimeError(f"Admin cmd failed: status=0x{status:04X}")
                return dw3 & 0xFFFF
            time.sleep(0.005)
        raise TimeoutError("Admin completion timeout")

    def init_controller(self):
        cap = self.reg_read(NVME_CAP) | (self.reg_read(NVME_CAP + 4) << 32)
        self.doorbell_stride = 4 << ((cap >> 32) & 0xF)
        timeout = ((cap >> 24) & 0xFF) * 500
        print(f"  CAP=0x{cap:016X} MQES={(cap&0xFFFF)+1} stride={self.doorbell_stride} timeout={timeout}ms")
        print(f"  VS={self.reg_read(NVME_VS)>>16&0xFFFF}.{self.reg_read(NVME_VS)>>8&0xFF}.{self.reg_read(NVME_VS)&0xFF}")
        # Disable
        cc = self.reg_read(NVME_CC)
        if cc & 1:
            self.reg_write(NVME_CC, cc & ~1)
            for _ in range(200):
                if not (self.reg_read(NVME_CSTS) & 1): break
                time.sleep(0.01)
            time.sleep(0.1)
        # Zero queues
        self.dma_write_sram(b'\x00' * 512)
        for i in range(QUEUE_DEPTH * 16):
            xdata_write(self.handle, ADMIN_CQ_XDATA + i, 0)
        # Configure + enable
        self.reg_write(NVME_AQA, ((QUEUE_DEPTH-1) << 16) | (QUEUE_DEPTH-1))
        self.reg_write(NVME_ASQ, ADMIN_SQ_DMA); self.reg_write(NVME_ASQ + 4, 0)
        self.reg_write(NVME_ACQ, ADMIN_CQ_DMA); self.reg_write(NVME_ACQ + 4, 0)
        self.reg_write(NVME_CC, (6 << 16) | (4 << 20) | 1)
        for i in range(max(200, timeout // 10)):
            csts = self.reg_read(NVME_CSTS)
            if csts & 1: print(f"  Controller ready ({i*10}ms)"); return
            if csts & 2: raise RuntimeError(f"NVMe fatal: CSTS=0x{csts:08X}")
            time.sleep(0.01)
        raise RuntimeError("NVMe enable timeout")

    def create_io_queues(self):
        self._admin_cmd(NVME_ADMIN_SET_FEATURES, cdw10=0x07, cdw11=0x00010001)
        self._admin_cmd(NVME_ADMIN_CREATE_IO_CQ,
                        cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=0x01, prp1=IO_CQ_DMA)
        self._admin_cmd(NVME_ADMIN_CREATE_IO_SQ,
                        cdw10=((QUEUE_DEPTH-1) << 16) | 1, cdw11=(1 << 16) | 0x01, prp1=IO_SQ_DMA)
        for i in range(QUEUE_DEPTH * 64):
            xdata_write(self.handle, IO_SQ_XDATA + i, 0)
        self.io_sq_tail = 0
        self.io_cq_head = 0
        print(f"  I/O queues created (QID=1, SQ@0x{IO_SQ_DMA:08X} CQ@0x{IO_CQ_DMA:08X})")

    # === DMA Read ===

    def read_sector(self, lba, count=1):
        """Read sectors via hardware NVMe DMA → USB bulk IN."""
        nbytes = count * self.sector_size
        # NVMe engine setup
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
        # Arm + doorbell
        xdata_write(self.handle, 0xC415, 0x04)
        xdata_write(self.handle, 0xC415, 0x01)
        xdata_write(self.handle, 0xC412, 0x02)
        xdata_write(self.handle, 0xC429, 0x00)
        self.io_sq_tail = (self.io_sq_tail + 1) % QUEUE_DEPTH
        self.hw_doorbell(self.io_sq_tail, 0x03)
        # Bulk IN
        rxbuf = (ctypes.c_ubyte * nbytes)()
        rxfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x81, rxbuf, nbytes, ctypes.byref(rxfer), 500)
        if ret != 0 or rxfer.value != nbytes:
            raise RuntimeError(f"bulk IN failed: ret={ret} xfer={rxfer.value}/{nbytes}")
        print("got", rxfer.value)
        # Ack CQ
        self.io_cq_head = (self.io_cq_head + 1) % QUEUE_DEPTH
        self.hw_doorbell(self.io_cq_head, 0x13)
        return bytes(rxbuf[:rxfer.value])

def main():
    handle, ctx = usb_open()
    try:
        ltssm = xdata_read(handle, 0xB450, 1)[0]
        if ltssm != 0x78:
            print(f"LTSSM=0x{ltssm:02X}, PCIe link not up"); return 1

        print("=== Bridge Setup ===")
        nvme_bus = setup_bridges(handle, gpu_bus=4, mem_base=0x00D00000)
        devices = enumerate_bus(handle, nvme_bus)
        if not devices:
            print("No NVMe found"); return 1
        print(f"  NVMe: [{devices[0][3]:04X}:{devices[0][4]:04X}] on bus {nvme_bus}")
        bars = assign_bars(handle, nvme_bus, mem_base=0x00D00000)
        bar0 = bars[0][0]
        print(f"  BAR0 = 0x{bar0:X}")

        nvme = NVMeDriver(handle, bar0)

        print("\n=== Init ===")
        csts = pcie_mem_read(handle, bar0 + 0x1C, size=4)
        if csts & 1:
            print("  Already ready, skipping init")
        else:
            nvme.init_controller()
            nvme.create_io_queues()

        print("\n=== DMA Read Test ===")
        for i in range(10):
            d = nvme.read_sector(i * 8)
            print(f"  LBA {i*8:4d}: {d[:16].hex()}")
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
