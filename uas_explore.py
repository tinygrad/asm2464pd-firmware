#!/usr/bin/env python3
"""UAS explore: open device, init NVMe DMA engine, send data via bulk OUT to 0xF000 and verify."""

import ctypes, os, struct, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb
from tinygrad.helpers import getenv

DMA_INIT = [
    # === SET_INTERFACE alt=1 sequence (from stock trace cycle 509958) ===

    # Round 1: NVMe link init
    (0xC42A, 0x20),
    (0xC428, 0x30),
    (0xC473, 0x66),
    (0xC472, 0x00),

    # Round 1: Init ctrl set
    (0xC448, 0xFF), (0xC449, 0xFF), (0xC44A, 0xFF), (0xC44B, 0xFF),
    (0xC473, 0x66),
    (0xC472, 0x00),
    (0xC438, 0xFF), (0xC439, 0xFF), (0xC43A, 0xFF), (0xC43B, 0xFF),

    # Round 1: Doorbell clear
    (0xC42A, 0x00),

    # Round 1: PRP + Queue init
    (0xC430, 0xFF), (0xC431, 0xFF), (0xC432, 0xFF), (0xC433, 0xFF),
    (0xC440, 0xFF), (0xC441, 0xFF), (0xC442, 0xFF), (0xC443, 0xFF),

    # Round 1: EP mode
    (0x9096, 0xFF), (0x9097, 0xFF),
    (0x9098, 0xFF), (0x9099, 0xFF), (0x909A, 0xFF), (0x909B, 0xFF),
    (0x909C, 0xFF), (0x909D, 0xFF),
    (0x909E, 0x03),

    # Round 1: Init ctrl clear
    (0xC438, 0x00), (0xC439, 0x00), (0xC43A, 0x00), (0xC43B, 0x00),
    (0xC448, 0x00), (0xC449, 0x00), (0xC44A, 0x00), (0xC44B, 0x00),

    # Round 1: FIFO/XCVR clear
    (0x9011, 0x00), (0x9012, 0x00), (0x9013, 0x00), (0x9014, 0x00),
    (0x9015, 0x00), (0x9016, 0x00), (0x9017, 0x00),
    (0x9018, 0x02),
    (0x9010, 0x00),

    # Round 2: NVMe link init (again)
    (0xC428, 0x30),
    (0xC473, 0x66),
    (0xC428, 0x30),
    (0xC473, 0x66),
    (0xC472, 0x00),

    # Round 2: Init ctrl set (again, no clear this time)
    (0xC448, 0xFF), (0xC449, 0xFF), (0xC44A, 0xFF), (0xC44B, 0xFF),
    (0xC473, 0x66),
    (0xC472, 0x00),
    (0xC438, 0xFF), (0xC439, 0xFF), (0xC43A, 0xFF), (0xC43B, 0xFF),

    # Round 2: Doorbell dance
    (0xC42A, 0x02),
    (0xC42A, 0x06),
    (0xC42A, 0x0E),
    (0xC42A, 0x0C),
    (0xC42A, 0x08),
    (0xC42A, 0x00),
    (0xC471, 0x01),
    (0xC472, 0x00),
    (0xC42A, 0x10),
    (0xC42A, 0x00),

    # Round 2: MSC sequence
    (0x900B, 0x02),
    (0x900B, 0x06),
    (0x900B, 0x04),
    (0x900B, 0x00),

    # Status + EP init
    (0x9000, 0x00),
    (0x924C, 0x04),
    (0x905F, 0x4C),
    (0x905D, 0x00),
    (0x90E3, 0x01),
    (0x90A0, 0x01),

    # Enable
    (0x9000, 0x01),
    (0x924C, 0x05),

    # Stream finalize
    (0x9200, 0xF1),
    (0x900B, 0x01),
    (0x900B, 0x00),
    (0x9200, 0xB1),

    # === DMA engine init ===
    (0xC42A, 0x20),
    (0xC422, 0x02),  # REG_NVME_LBA_LOW
    (0xC414, 0x80),  # REG_NVME_DATA_CTRL
    (0xC412, 0x03),  # REG_NVME_CTRL_STATUS
    (0xC415, 0x01),  # stream count

    (0xC428, 0x30),
    (0xC420, 0x00),
    (0xC421, 0x01),

    # MSC interrupts
    (0xC42C, 1), (0xC42D, 0),
]

STREAMS = bool(getenv("STREAMS"))

class Dev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, max_streams=32, use_bot=not STREAMS)
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
            dev.write(0xC429, 0x00)


        if STREAMS:
            # send on stream 1 using async transfer API
            slot = 0
            stream_id = 1
            usb = dev.usb
            buf = usb.buf_data_out[slot]
            mv = usb.buf_data_out_mvs[slot]
            mv[:len(test_data)] = test_data
            tr = usb._prep_transfer(usb.tr[usb.ep_data_out][slot], usb.ep_data_out, stream_id, buf, len(test_data))
            usb._submit_and_wait([tr])
        else:
            dev.usb._bulk_out(dev.usb.ep_data_out, test_data)

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
