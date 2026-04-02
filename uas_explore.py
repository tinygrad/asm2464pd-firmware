#!/usr/bin/env python3
"""UAS explore: open device, init NVMe DMA engine, send data via SCSI WRITE to 0xF000."""

import ctypes, os, struct, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb
from hexdump import hexdump

# NVMe DMA init sequence from trace/usb2_dma.
# This is what the stock firmware does after SET_INTERFACE alt=1 to bring up
# the NVMe/UAS DMA engine so bulk writes land at 0xF000.
C4XX_INIT = [
    (0xC42A, 0x20),  # REG_NVME_DOORBELL = 0x20
    (0xC422, 0x02),  # REG_NVME_LBA_LOW = 0x02
    (0xC427, 0x01),  # REG_NVME_ERROR (sector count) = 0x01
    (0xC413, 0x02),  # REG_NVME_CONFIG = 0x02 (write mode)
    (0xC414, 0x80),  # REG_NVME_DATA_CTRL = 0x80
    (0xC415, 0x01),  # REG_NVME_DEV_STATUS = 0x01
    (0xC412, 0x03),  # REG_NVME_CTRL_STATUS = 0x03
    (0xC429, 0x00),  # REG_NVME_CMD_PARAM = 0x00
]

class Dev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04)

    def read8(self, addr):
        buf = (ctypes.c_ubyte * 1)()
        ret = libusb.libusb_control_transfer(self.usb.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
        assert ret >= 0, f"read(0x{addr:04X}) failed: {ret}"
        return buf[0]

    def write(self, addr, val):
        ret = libusb.libusb_control_transfer(self.usb.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

    def init_dma(self):
        for addr, val in C4XX_INIT:
            self.write(addr, val)

    def scsi_write(self, data):
        usb = self.usb
        usb._bulk_in(usb.ep_stat_in, 64)
        usb._bulk_out(usb.ep_data_out, data)

def main():
    dev = Dev()

    print(f"Init DMA ({len(C4XX_INIT)} writes)...", end="")
    dev.init_dma()
    print(" done.")

    tag = os.urandom(4)
    test_data = tag + bytes(range(252)) + bytes(range(256))
    print(f"SCSI WRITE {len(test_data)} bytes (tag={tag.hex()})...", end="")
    dev.scsi_write(test_data)

    got = bytes(dev.read8(0xF000 + j) for j in range(0x10))
    if got[:4] == tag:
        print(f"  PASS")
    else:
        print(f"  FAIL: expected {tag.hex()}, got {got.hex()}")
    hexdump(got)


if __name__ == "__main__":
    main()
