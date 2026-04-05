#!/usr/bin/env python3
"""Test DMA read matching stock firmware trace EXACTLY."""
import struct, time, sys
from pcie.pcie_probe import usb_open, usb_close, xdata_read, xdata_write, xdata_write_bytes, pcie_mem_read, pcie_mem_write
from tinygrad.runtime.autogen import libusb

BAR0 = 0x00D00000
# Known working XDATA mappings
ADMIN_SQ_XDATA = 0xF000   # DMA 0x00200000
ADMIN_CQ_XDATA = 0xA000   # DMA 0x00820000

# Stock I/O queue addresses (decoded from trace CREATE_IO_CQ/SQ):
IO_SQ_DMA = 0x00820000    # XDATA 0xA000
IO_CQ_DMA = 0x00828000    # NOT 0x00820000! Separate CQ address.
IO_SQ_XDATA = 0xA000
DATA_DMA  = 0x00200000    # XDATA 0xF000
QDEPTH = 32               # Stock uses 32 (QSIZE=31)

def stock_admin_cmd(handle, opcode, cid, prp1=0, cdw10_bytes=b'\x00'*4, cdw11_bytes=b'\x00'*4,
                    dma_chan=[0], sq_tail=[0], cq_head=[0], cq_phase=[1]):
    """Submit admin command matching stock firmware sequence exactly."""
    chan = dma_chan[0]

    # 1. DMA channel zero (clear admin SQ slot at DMA 0x00800000)
    xdata_write(handle, 0xC8D8, 0x00)
    xdata_write(handle, 0xC8B7, 0x00)
    xdata_write(handle, 0xC8B6, 0x14)
    xdata_write(handle, 0xC8B6, 0x14)
    xdata_write(handle, 0xC8B6, 0x14)
    xdata_write(handle, 0xC8B6, 0x94)
    xdata_write(handle, 0xC8B2, 0xB0)
    xdata_write(handle, 0xC8B3, (chan * 0x40) & 0xFF)  # B0+00, B0+40, B0+80, B0+C0
    xdata_write(handle, 0xC8B4, 0x00)
    xdata_write(handle, 0xC8B5, 0x3F)  # 64 bytes
    xdata_write(handle, 0xC8B8, 0x01)  # trigger
    xdata_write(handle, 0xC8B6, 0x14)

    # 2. Select DMA bank/channel
    xdata_write(handle, 0xC8D8, 0x01)
    xdata_write(handle, 0xC8D7, 0x80 | chan)

    # 3. Write SQE to A000 (XDATA window, engine copies to 0x00800000)
    xdata_write(handle, 0xA000, opcode)
    xdata_write(handle, 0xA002, cid & 0xFF)
    # PRP1 at offset 0x18
    xdata_write(handle, 0xA018, prp1 & 0xFF)
    xdata_write(handle, 0xA019, (prp1 >> 8) & 0xFF)
    xdata_write(handle, 0xA01A, (prp1 >> 16) & 0xFF)
    xdata_write(handle, 0xA01B, (prp1 >> 24) & 0xFF)
    # cdw10 at offset 0x28
    for i, b in enumerate(cdw10_bytes):
        if b: xdata_write(handle, 0xA028 + i, b)
    # cdw11 at offset 0x2C
    for i, b in enumerate(cdw11_bytes):
        if b: xdata_write(handle, 0xA02C + i, b)

    # 4. Restore bank
    xdata_write(handle, 0xC8D8, 0x00)

    # 5. Ring admin SQ doorbell via B254=0x01
    sq_tail[0] = (sq_tail[0] + 1) % QDEPTH
    xdata_write(handle, 0xB296, 0x04)
    xdata_write(handle, 0xB251, sq_tail[0] & 0xFF)
    xdata_write(handle, 0xB254, 0x01)
    xdata_write(handle, 0xB296, 0x04)

    # 6. CEF2 + C8D6=0x08 (stock sequence)
    xdata_write(handle, 0xCEF2, 0x80)
    xdata_write(handle, 0xC8D6, 0x08)
    xdata_write(handle, 0xC8D6, 0x08)

    # 7. Poll for completion (B80E+chan*0x10)
    poll_base = 0xB80E + chan * 0x10
    for _ in range(500):
        v = xdata_read(handle, poll_base, 1)[0]
        if v & 0x01: break
        time.sleep(0.01)
    else:
        print(f"  admin cmd 0x{opcode:02X} timeout (poll 0x{poll_base:04X})")
        dma_chan[0] = (chan + 1) % 3  # skip channel 3
        return 0xFFFF

    # 8. Ring admin CQ doorbell via B254=0x11
    cq_head[0] = (cq_head[0] + 1) % QDEPTH
    xdata_write(handle, 0xB296, 0x04)
    xdata_write(handle, 0xB251, cq_head[0] & 0xFF)
    xdata_write(handle, 0xB254, 0x11)
    xdata_write(handle, 0xB296, 0x04)

    # 9. Clear C8D6
    xdata_write(handle, 0xC8D6, 0x00)
    xdata_write(handle, 0xC8D6, 0x00)

    dma_chan[0] = (chan + 1) % 3  # skip channel 3 (B83E broken)
    return 0

def main():
    handle, ctx = usb_open()
    try:
        csts = pcie_mem_read(handle, BAR0 + 0x1C, size=4)
        print(f"CSTS=0x{csts:08X}")
        if not (csts & 1):
            print("NVMe not ready!"); return 1
        # Use replay's NVMe state as-is (ASQ=0x00800000, ACQ=0x00808000)
        # The hardware engine knows these addresses from the replay.

        # Admin commands matching stock trace EXACTLY (8 commands)
        # Cmd 1: IDENTIFY controller
        r = stock_admin_cmd(handle, 0x06, 0, prp1=0x00200000, cdw10_bytes=b'\x01\x00\x00\x00')
        print(f"1 IDENTIFY ctrl: {r}")
        # Cmd 2: IDENTIFY namespace
        r = stock_admin_cmd(handle, 0x06, 1, prp1=0x00200000, cdw10_bytes=b'\x00\x00\x00\x00')
        print(f"2 IDENTIFY ns: {r}")
        # Cmd 3: SET_FEATURES number of queues (cdw10=7)
        r = stock_admin_cmd(handle, 0x09, 2, cdw10_bytes=b'\x07\x00\x00\x00', cdw11_bytes=b'\x01\x00\x01\x00')
        print(f"3 SET_FEATURES queues: {r}")
        # Cmd 4: SET_FEATURES interrupt coalescing (cdw10=8)
        r = stock_admin_cmd(handle, 0x09, 3, cdw10_bytes=b'\x08\x00\x00\x00')
        print(f"4 SET_FEATURES int_coal: {r}")
        # Cmd 5: SET_FEATURES arbitration (cdw10=1)
        r = stock_admin_cmd(handle, 0x09, 4, cdw10_bytes=b'\x01\x00\x00\x00')
        print(f"5 SET_FEATURES arb: {r}")
        # Cmd 6: SET_FEATURES async event (cdw10=6, cdw11=1)
        r = stock_admin_cmd(handle, 0x09, 5, cdw10_bytes=b'\x06\x00\x00\x00', cdw11_bytes=b'\x01\x00\x00\x00')
        print(f"6 SET_FEATURES async: {r}")
        # Cmd 7: CREATE_IO_CQ
        r = stock_admin_cmd(handle, 0x05, 6,
                           prp1=IO_CQ_DMA,
                           cdw10_bytes=struct.pack('<I', (31<<16)|1),
                           cdw11_bytes=struct.pack('<I', 0x00010001))
        print(f"7 CREATE_IO_CQ: {r}")
        # Cmd 8: CREATE_IO_SQ
        r = stock_admin_cmd(handle, 0x01, 7,
                           prp1=IO_SQ_DMA,
                           cdw10_bytes=struct.pack('<I', (31<<16)|1),
                           cdw11_bytes=struct.pack('<I', 0x00010005))
        print(f"8 CREATE_IO_SQ: {r}")

        # Zero I/O SQ area
        for i in range(256): xdata_write(handle, IO_SQ_XDATA + i, 0)

        # Restore ALL register differences to stock values
        xdata_write(handle, 0xC8D7, 0x83)
        xdata_write(handle, 0xC8D8, 0x00)
        xdata_write(handle, 0xC430, 0xFF)
        xdata_write(handle, 0xC431, 0xFF)
        xdata_write(handle, 0xC432, 0xFF)
        xdata_write(handle, 0xC433, 0xFF)
        xdata_write(handle, 0xC440, 0xFF)
        xdata_write(handle, 0xC441, 0xFF)
        xdata_write(handle, 0xC442, 0xFF)
        xdata_write(handle, 0xC443, 0xFF)
        xdata_write(handle, 0xC805, 0x02)
        xdata_write(handle, 0xC42C, 0x01)  # auto-clears but stock has it

        # === I/O READ exact stock sequence ===
        xdata_write(handle, 0xC428, 0x10)
        xdata_write(handle, 0xC426, 0x00)
        xdata_write(handle, 0xC427, 0x01)
        xdata_write(handle, 0xC401, 0x00)
        xdata_write(handle, 0xC412, 0x00)
        xdata_write(handle, 0xC413, 0x00)
        xdata_write(handle, 0xC420, 0x00)
        xdata_write(handle, 0xC421, 0x00)
        xdata_write(handle, 0xC414, 0x00)
        xdata_write(handle, 0xC414, 0x80)
        xdata_write(handle, 0xC412, 0x00)

        # SQE at A000
        sqe = struct.pack('<IIIIIIQQ', 0x02|(0x1111<<16), 1, 0,0,0,0, DATA_DMA, 0)
        sqe += struct.pack('<QHHHH', 0, 0, 0, 0, 0)
        xdata_write_bytes(handle, IO_SQ_XDATA, sqe[:64])

        xdata_write(handle, 0xC415, 0x04)
        xdata_write(handle, 0xC415, 0x01)
        xdata_write(handle, 0xC412, 0x02)  # ARM
        xdata_write(handle, 0xC429, 0x00)

        xdata_write(handle, 0xB296, 0x04)
        xdata_write(handle, 0xB251, 0x01)
        xdata_write(handle, 0xB254, 0x03)
        xdata_write(handle, 0xB296, 0x04)

        xdata_write(handle, 0xCEF3, 0x08)

        # Poll
        for poll in range(500):
            c802 = xdata_read(handle, 0xC802, 1)[0]
            b80e = xdata_read(handle, 0xB80E, 1)[0]
            c520 = xdata_read(handle, 0xC520, 1)[0]
            if c802 or b80e or c520:
                print(f"*** POLL {poll}: C802=0x{c802:02X} B80E=0x{b80e:02X} C520=0x{c520:02X} ***")
                break
            time.sleep(0.002)
        else:
            print("No engine response after 500 polls")

        d = xdata_read(handle, 0xF000, 16)
        print(f"F000: {d.hex()}")
        for reg in [0xC802, 0xC806, 0xB80E, 0xC520, 0xC512]:
            print(f"  0x{reg:04X} = 0x{xdata_read(handle, reg, 1)[0]:02X}")

        return 0
    finally:
        usb_close(handle, ctx)

if __name__ == "__main__":
    sys.exit(main())
