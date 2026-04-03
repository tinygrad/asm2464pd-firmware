#!/usr/bin/env python3
"""UAS explore: open device, init NVMe DMA engine, send data via bulk OUT to 0xF000 and verify."""

import ctypes, os, struct, sys
from hexdump import hexdump
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb
from tinygrad.helpers import getenv

STREAMS = getenv("STREAMS", 0)

DMA_INIT = [
    # enable streams globally
    (0x9000, int(STREAMS > 0)),

    # === DMA engine init ===
    (0xC428, 0x30),  # not strictly needed
    (0xC42A, 0x20),
    (0xC422, 0x02),  # REG_NVME_LBA_LOW
    (0xC414, 0x80),  # REG_NVME_DATA_CTRL
    (0xC412, 0x03),  # REG_NVME_CTRL_STATUS
    (0xC421, 0x01),  # use streams for dma

    # MSC interrupts
    #(0xC42C, 1), (0xC42D, 0),
]

class Dev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, max_streams=32, use_bot=STREAMS==0)
        if STREAMS: assert self.usb.use_streams, "streams are required"

    def readn(self, addr, size):
        buf = (ctypes.c_ubyte * size)()
        ret = libusb.libusb_control_transfer(self.usb.handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
        assert ret >= 0, f"read(0x{addr:04X}, {size}) failed: {ret}"
        return bytes(buf[:ret])

    def write(self, addr, val):
        ret = libusb.libusb_control_transfer(self.usb.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

    def f2_arm(self, sectors, slot=0):
        ret = libusb.libusb_control_transfer(self.usb.handle, 0x40, 0xF2, sectors, slot * 4, None, 0, 1000)
        assert ret >= 0, f"0xF2 arm failed (sectors={sectors}, slot={slot}): {ret}"

def main():
    dev = Dev()
    use_f2 = getenv("F2", 0)

    if not use_f2:
        for addr, val in DMA_INIT:
            dev.write(addr, val)

    size = getenv("SIZE", 4096)
    assert size % 512 == 0

    for i in range(getenv("CNT", 3)):
        tag = os.urandom(4)
        test_data = bytearray(size)
        test_data[0:4] = tag
        for j in range(4, size):
            test_data[j] = (j * 7 + 0x42) & 0xFF
        test_data = bytes(test_data)

        print(f"[{i}] bulk OUT {size} bytes (tag={tag.hex()})...", end="")
        if use_f2:
            dev.f2_arm(len(test_data) // 512)
        else:
            dev.write(0xC427, len(test_data) // 512)

            # stream range
            first_stream = 0
            dev.write(0xC414, 0x80+first_stream)
            dev.write(0xC415, 0x01+first_stream)
            dev.write(0xC429, first_stream)

        if STREAMS:
            # send on stream 1 using async transfer API
            submit = []
            for slot in range(STREAMS):
                stream_id = 1+slot
                usb = dev.usb
                buf = usb.buf_data_out[slot]
                mv = usb.buf_data_out_mvs[slot]
                mv[:len(test_data)] = test_data
                tr = usb._prep_transfer(usb.tr[usb.ep_data_out][slot], usb.ep_data_out, stream_id, buf, len(test_data))
                submit.append(tr)
            usb._submit_and_wait(submit)
        else:
            dev.usb._bulk_out(dev.usb.ep_data_out, test_data)

        if getenv("DUMP"): hexdump(dev.readn(0xc400, 0x80))

        # read back full 0xF000-0x10000 and compare
        got = b""
        for off in range(0, 0x1000, 64):
            got += dev.readn(0xF000 + off, 64)

        errors = 0
        for off in range(0, 0x1000, 4):
            if got[off:off+4] != test_data[off:off+4]:
                errors += 1
                if errors <= 4:
                    print(f"\n  FAIL @ 0x{off:04X}: got {got[off:off+4].hex()} exp {test_data[off:off+4].hex()}", end="")

        if errors == 0:
            print(f"  PASS")
        else:
            print(f"\n  {errors} errors total")
            break

if __name__ == "__main__":
    main()
