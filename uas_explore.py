#!/usr/bin/env python3
"""UAS explore: open device, init NVMe DMA engine, send data via SCSI WRITE to 0xF000."""

import ctypes, os, struct, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb
from tinygrad.helpers import getenv

# NVMe DMA init sequence from trace/usb2_dma.
# This is what the stock firmware does after SET_INTERFACE alt=1 to bring up
# the NVMe/UAS DMA engine so bulk writes land at 0xF000.
DMA_INIT = [
    (0xC42A, 0x20),  # REG_NVME_DOORBELL = 0x20 (breaks bulk IN)
    (0xC422, 0x02),  # REG_NVME_LBA_LOW = 0x02 (data is wrong)
    (0xC427, 0x01),  # REG_NVME_ERROR (sector count) = 0x01 (data is wrong)
    (0xC414, 0x80),  # REG_NVME_DATA_CTRL = 0x80 (breaks bulk IN)
    (0xC415, 0x01),  # REG_NVME_DEV_STATUS = 0x01 (without this, read 0xF000 fails)
    (0xC412, 0x03),  # REG_NVME_CTRL_STATUS = 0x03 (breaks bulk IN)
]

class Dev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

    def read8(self, addr):
        buf = (ctypes.c_ubyte * 1)()
        ret = libusb.libusb_control_transfer(self.usb.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
        assert ret >= 0, f"read(0x{addr:04X}) failed: {ret}"
        return buf[0]

    def write(self, addr, val):
        ret = libusb.libusb_control_transfer(self.usb.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

    def init_dma(self):
        for addr, val in DMA_INIT:
            self.write(addr, val)

    def scsi_write(self, data):
        usb = self.usb
        usb._bulk_out(usb.ep_data_out, data)

def main():
    dev = Dev()
    dev.init_dma()

    # NOTE: if this is less than 10, it won't work a second time and will fail with BULK IN STALL (-7)
    for i in range(getenv("CNT", 3)):
        dev.write(0xC429, 0x00) # NVMe slot 0 select + DMA re-arm (slot*4, see registers.h)
        tag = os.urandom(4)
        test_data = tag + bytes(range(252)) + bytes(range(256))
        print(f"[{i}] SCSI WRITE (tag={tag.hex()})...", end="")
        dev.scsi_write(test_data)
        got = bytes(dev.read8(0xF000 + j) for j in range(4))
        if got == tag:
            print(f"  PASS")
        else:
            print(f"  FAIL: expected {tag.hex()}, got {got.hex()}")
            break


if __name__ == "__main__":
    main()
