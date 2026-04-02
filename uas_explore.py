#!/usr/bin/env python3
"""UAS explore: open device, replay UAS init from trace, send data via SCSI WRITE to 0xF000."""

import ctypes, os, re, struct, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

TRACE = "trace/usb2_dma"
# Replay writes from after SET_INTERFACE alt=1 through the UAS init
AFTER = 813676
BEFORE = 889507
BLACKLIST = {0x9091, 0x9092}

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

    def scsi_write(self, data):
        usb = self.usb
        slot = 0
        cdb = struct.pack('>BBQIBB', 0x8A, 0, 0, len(data) // 512, 0, 0)
        usb.buf_cmd[slot][16:16+len(cdb)] = list(cdb)
        usb._uas_tag = (usb._uas_tag % 255) + 1
        usb.buf_cmd[slot][3] = usb._uas_tag
        # 1. Send command IU
        usb._bulk_out(usb.ep_cmd_out, bytes(usb.buf_cmd[slot]))
        # 2. Wait for RTT
        for _retry in range(10):
            _rtt = usb._bulk_in(usb.ep_stat_in, 64)
            if _rtt[0] == 0x07: break
            usb._uas_tag = (usb._uas_tag % 255) + 1
            usb.buf_cmd[slot][3] = usb._uas_tag
            usb._bulk_out(usb.ep_cmd_out, bytes(usb.buf_cmd[slot]))
        else: raise RuntimeError("UAS: failed to get RTT after 10 retries")
        # 3. Send data
        usb._bulk_out(usb.ep_data_out, data)
        # 4. Re-arm DMA for next command
        self.write(0xD000, 0x00)
        self.write(0xD020, 0x00)
        self.write(0x9093, 0x08)
        self.write(0xCE88, 0x02)
        # Re-arm DMA slots
        self.write(0xC8D4, 0x80)
        self.read8(0xC4ED)
        self.write(0xC4ED, self.read8(0xC4ED) & 0xFE)
        self.write(0xD802, 0x00)
        self.write(0xD803, 0x0B)
        self.write(0xD804, 0x00)
        self.write(0xD805, 0x00)
        self.write(0xD806, 0x00)
        self.write(0xD807, 0x00)
        self.write(0xD80F, 0x00)
        self.write(0xD800, 0x03)
        self.write(0xC509, 0x01)
        self.write(0x901A, 0x10)
        self.write(0x90A1, 0x01)
        self.write(0xC509, 0x00)
        self.write(0xC8D4, 0x00)
        self.write(0x9093, 0x08)
        self.write(0xCE88, 0x02)


def replay_init(dev):
    """Replay register ops from the trace to set up UAS DMA state."""
    line_re = re.compile(
        r'\[\s*(\d+)\]\s+PC=0x[0-9A-Fa-f]+\s+(Read|Write)\s+0x([0-9A-Fa-f]+)\s+=\s+0x([0-9A-Fa-f]+)')
    count = 0
    with open(TRACE) as f:
        for line in f:
            m = line_re.search(line)
            if not m: continue
            cycle = int(m.group(1))
            if cycle <= AFTER: continue
            if cycle >= BEFORE: break
            op = m.group(2)
            addr = int(m.group(3), 16)
            val = int(m.group(4), 16)
            if addr in BLACKLIST: continue
            if 0xD800 <= addr < 0xE000: continue  # buffer region
            #if 0x8000 <= addr < 0x8100: continue  # NVMe write data
            if op == "Write":
                dev.write(addr, val)
            else:
                dev.read8(addr)
            count += 1
    return count


def main():
    dev = Dev()

    print("Replaying UAS init from trace...", end="")
    n = replay_init(dev)
    print(f"  {n} ops.")

    for i in range(1):
        tag = os.urandom(4)
        test_data = tag + bytes(range(252)) + bytes(range(256))
        print(f"[{i}] SCSI WRITE {len(test_data)} bytes (tag={tag.hex()})...", end="")
        try:
            dev.scsi_write(test_data)
        except Exception as e:
            print(f"  {e}")
        # Search for data in F000-FFFF
        found = None
        for base in range(0xF000, 0x10000, 0x200):
            got = bytes(dev.read8(base + j) for j in range(4))
            if got == tag:
                found = base
                break
        if found is not None:
            print(f"  PASS at 0x{found:04X}")
        else:
            print(f"  FAIL: tag {tag.hex()} not found")


if __name__ == "__main__":
    main()
