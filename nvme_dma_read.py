#!/usr/bin/env python3
"""NVMe DMA read via the hardware snooper/bulk IN pipeline.

Usage:
    make -C handmade nflash
    python3 pcie/pcie_bringup.py
    python3 nvme_driver.py        # init NVMe, create I/O queues
    python3 nvme_dma_read.py      # do DMA reads via bulk IN
"""

import struct, ctypes, time, sys
from pcie.pcie_probe import usb_open, usb_close, xdata_read, xdata_write
from tinygrad.runtime.autogen import libusb

class NVMeDMAReader:
    def __init__(self, handle):
        self.handle = handle
        self.sq_tail = 0
        self.cq_head = 0
        self.tag = 0

    def arm_cbw_receiver(self):
        """Arm the USB MSC hardware to receive a CBW into 0x911B+ registers."""
        xdata_write(self.handle, 0x90E3, 0x02)
        time.sleep(0.02)

    def send_read16_cbw(self, lba, count=1):
        """Send a READ_16 CBW via USB bulk OUT. Hardware populates 0x91xx."""
        self.tag += 1
        xfer_len = count * 512
        cbw = struct.pack('<III', 0x43425355, self.tag, xfer_len)
        cbw += struct.pack('BBB', 0x80, 0, 16)  # flags=IN, LUN=0, CB_LEN=16
        cbw += struct.pack('>BBQIBB', 0x88, 0x01, lba, count, 0, 0)
        buf = (ctypes.c_ubyte * len(cbw)).from_buffer_copy(cbw)
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x02, buf, len(cbw), ctypes.byref(xfer), 1000)
        assert ret == 0 and xfer.value == len(cbw), f"CBW send failed: ret={ret} xfer={xfer.value}"
        time.sleep(0.02)

    def setup_nvme_engine(self, count=1):
        """Configure C4xx NVMe registers for a read of count sectors."""
        xdata_write(self.handle, 0xC428, 0x10)    # queue config (BOT mode)
        xdata_write(self.handle, 0xC426, count >> 8)
        xdata_write(self.handle, 0xC427, count & 0xFF)
        xdata_write(self.handle, 0xC401, 0x00)    # status clear
        xdata_write(self.handle, 0xC412, 0x00)    # ctrl clear
        xdata_write(self.handle, 0xC413, 0x00)    # config
        xdata_write(self.handle, 0xC420, 0x00)    # DMA xfer hi
        xdata_write(self.handle, 0xC421, 0x00)    # stream ID
        xdata_write(self.handle, 0xC414, 0x80)    # slot start
        xdata_write(self.handle, 0xC412, 0x00)    # ctrl clear again

    def arm_snooper(self):
        """Trigger CE00=0x03 which arms the PCIe CQ snooper and builds the SQE from CBW."""
        xdata_write(self.handle, 0x90E2, 0x01)    # USB mode
        xdata_write(self.handle, 0xCE88, 0x00)    # handshake stream 0
        xdata_write(self.handle, 0xCE36, 0x00)    # DMA config
        xdata_write(self.handle, 0xCE01, 0x00)    # DMA param
        xdata_write(self.handle, 0xCE3A, 0x00)    # DMA tag
        xdata_write(self.handle, 0xCE00, 0x03)    # TRIGGER — arms snooper + builds SQE
        time.sleep(0.01)

    def arm_and_doorbell(self):
        """Arm NVMe slot and ring the doorbell."""
        xdata_write(self.handle, 0xC415, 0x04)    # slot end (arm)
        xdata_write(self.handle, 0xC415, 0x01)    # slot end (commit)
        xdata_write(self.handle, 0xC412, 0x02)    # ctrl status = READ
        xdata_write(self.handle, 0xC429, 0x00)    # cmd param
        # Ring SQ doorbell via bridge
        xdata_write(self.handle, 0xB296, 0x04)
        xdata_write(self.handle, 0xB251, (self.sq_tail + 1) & 0x1F)
        xdata_write(self.handle, 0xB254, 0x03)    # I/O SQ1
        xdata_write(self.handle, 0xB296, 0x04)
        self.sq_tail = (self.sq_tail + 1) & 0x1F

    def receive_data(self, nbytes, timeout_ms=500):
        """Read bulk IN data."""
        buf = (ctypes.c_ubyte * nbytes)()
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.handle, 0x81, buf, nbytes, ctypes.byref(xfer), timeout_ms)
        if ret == 0 and xfer.value > 0:
            return bytes(buf[:xfer.value])
        return None

    def ack_completion(self):
        """Ack the NVMe completion: walk CQ, CQ doorbell, C512=0xFF."""
        # CEF3 link active
        xdata_write(self.handle, 0xCEF3, 0x08)
        # Walk CQ
        idx = self.cq_head
        xdata_write(self.handle, 0xC8D6, 0x01)
        xdata_write(self.handle, 0xC8D5, idx)
        time.sleep(0.005)
        xdata_write(self.handle, 0xC8D6, 0x01)
        xdata_write(self.handle, 0xC8D5, (idx + 1) % 32)
        xdata_write(self.handle, 0xC8D6, 0x00)
        # CQ doorbell
        self.cq_head = (self.cq_head + 1) & 0x1F
        xdata_write(self.handle, 0xB296, 0x04)
        xdata_write(self.handle, 0xB251, self.cq_head)
        xdata_write(self.handle, 0xB254, 0x13)  # I/O CQ1
        xdata_write(self.handle, 0xB296, 0x04)
        # Ack queue index
        xdata_write(self.handle, 0xC512, 0xFF)

    def read_sector(self, lba, count=1):
        """Full DMA read: send CBW, arm snooper, doorbell, receive data."""
        nbytes = count * 512
        self.arm_cbw_receiver()
        self.send_read16_cbw(lba, count)
        self.setup_nvme_engine(count)
        self.arm_snooper()
        self.arm_and_doorbell()
        data = self.receive_data(nbytes)
        if data:
            self.ack_completion()
            # Re-arm CBW for next command
            self.arm_cbw_receiver()
        return data


def main():
    handle, ctx = usb_open()
    try:
        reader = NVMeDMAReader(handle)

        # Read LBA 0
        print("=== DMA Read LBA 0 ===")
        data = reader.read_sector(0)
        if data:
            print(f"  OK: {len(data)} bytes, first 32: {data[:32].hex()}")
        else:
            print("  FAILED")
            return 1

        # Read a few more to verify repeatability
        print("\n=== Repeatability Test ===")
        for i in range(5):
            data = reader.read_sector(i * 8)
            if data:
                print(f"  LBA {i*8:4d}: {len(data)} bytes, {data[:8].hex()}...")
            else:
                print(f"  LBA {i*8:4d}: FAILED")
                return 1

        # Verify data consistency: read same LBA twice
        print("\n=== Consistency Test ===")
        d1 = reader.read_sector(0)
        d2 = reader.read_sector(0)
        if d1 == d2:
            print(f"  Two reads of LBA 0 match: {d1[:16].hex()}")
        else:
            print(f"  MISMATCH! d1={d1[:16].hex()} d2={d2[:16].hex()}")
            return 1

        print("\nAll tests passed!")
        return 0
    finally:
        usb_close(handle, ctx)


if __name__ == "__main__":
    sys.exit(main())
