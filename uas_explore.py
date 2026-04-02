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
            if 0x8000 <= addr < 0x8100: continue  # NVMe write data
            if 0xF000 <= addr < 0xF100: continue  # NVMe read data
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
    print(f"  {n} writes.")

    # Send test data
    tag = os.urandom(4)
    test_data = tag + bytes(range(252)) + bytes(range(256))
    print(f"SCSI WRITE {len(test_data)} bytes (tag={tag.hex()})...", end="")
    dev.scsi_write(test_data)
    print("  done.")

    # Verify at 0xF000
    print("0xF000:")
    ok = True
    for off in range(0, 64, 16):
        vals = [dev.read8(0xF000 + off + i) for i in range(16)]
        hex_str = ' '.join(f'{v:02X}' for v in vals)
        print(f"  {hex_str}")
    got = bytes(dev.read8(0xF000 + i) for i in range(4))
    if got == tag:
        print(f"PASS: tag {tag.hex()} found at 0xF000")
    else:
        print(f"FAIL: expected {tag.hex()}, got {got.hex()}")


if __name__ == "__main__":
    main()
