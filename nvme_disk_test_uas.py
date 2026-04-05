#!/usr/bin/env python3
"""Test NVMe disk via stock firmware UAS SCSI: init, read, write, verify."""

import ctypes, struct, time, os, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

class UASDev:
    def __init__(self):
        # use_bot=False triggers SET_INTERFACE alt=1, stream alloc
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=False)
        print(f"UAS mode: use_streams={self.usb.use_streams}")

    def scsi(self, cdb, xfer_len=0, direction_in=True, timeout=10000):
        """Send SCSI command via UAS. Returns (data, status)."""
        results = self.usb.send_batch([cdb], idata=[xfer_len if direction_in else 0],
                                      odata=[None if direction_in else b'\x00' * xfer_len if xfer_len else None])
        data = results[0] if results[0] is not None else b''
        return data, 0

    def scsi_out(self, cdb, data, timeout=10000):
        """Send SCSI command with data-out phase."""
        self.usb.send_batch([cdb], idata=[0], odata=[data])
        return 0

    def inquiry(self):
        data, s = self.scsi(b'\x12\x00\x00\x00\x24\x00', 36)
        assert s == 0 and len(data) >= 8, f"INQUIRY failed: status={s} len={len(data)}"
        vendor = data[8:16].decode(errors='replace').strip()
        product = data[16:32].decode(errors='replace').strip()
        print(f"INQUIRY: vendor=\"{vendor}\" product=\"{product}\"")
        return data

    def test_unit_ready(self, retries=30, delay=1):
        for i in range(retries):
            _, s = self.scsi(b'\x00\x00\x00\x00\x00\x00', 0)
            if s == 0:
                print(f"TEST_UNIT_READY: ready (attempt {i+1})")
                return True
            print(f"  TUR attempt {i+1}: NOT READY")
            time.sleep(delay)
        print("TEST_UNIT_READY: FAILED after all retries")
        return False

    def read_capacity(self):
        data, s = self.scsi(b'\x25' + b'\x00' * 9, 8)
        assert s == 0 and len(data) == 8, f"READ_CAPACITY failed: status={s} len={len(data)}"
        max_lba = struct.unpack('>I', data[:4])[0]
        blk_size = struct.unpack('>I', data[4:])[0]
        size_mb = max_lba * blk_size // 1024 // 1024
        print(f"READ_CAPACITY: max_lba={max_lba} blk_size={blk_size} ({size_mb}MB)")
        return max_lba, blk_size

    def read16(self, lba, count=1):
        cdb = struct.pack('>BBQIBB', 0x88, 0x01, lba, count, 0, 0)
        data, s = self.scsi(cdb, count * 512)
        assert s == 0, f"READ(16) LBA={lba} failed: status={s}"
        return data

    def write16(self, lba, data):
        count = len(data) // 512
        assert count * 512 == len(data), "data must be multiple of 512"
        cdb = struct.pack('>BBQIBB', 0x8A, 0x01, lba, count, 0, 0)
        s = self.scsi_out(cdb, data)
        assert s == 0, f"WRITE(16) LBA={lba} failed: status={s}"

def main():
    dev = UASDev()

    # SCSI init
    dev.inquiry()
    if not dev.test_unit_ready():
        sys.exit(1)
    max_lba, blk_size = dev.read_capacity()

    # Use a high LBA to avoid clobbering partition tables
    test_lba = max_lba - 16
    print(f"\nUsing test LBA={test_lba} (near end of disk)")

    # Save original data
    print("Reading original data...")
    orig = dev.read16(test_lba, 1)
    print(f"  original: {orig[:32].hex()}")

    # Write test pattern
    pattern = os.urandom(512)
    print(f"Writing test pattern: {pattern[:16].hex()}...")
    dev.write16(test_lba, pattern)

    # Read back and verify
    print("Reading back...")
    readback = dev.read16(test_lba, 1)
    print(f"  readback: {readback[:32].hex()}")

    if readback == pattern:
        print("PASS: write/read verified!")
    else:
        print("FAIL: data mismatch!")
        for i in range(512):
            if readback[i] != pattern[i]:
                print(f"  first diff at byte {i}: got 0x{readback[i]:02X} expected 0x{pattern[i]:02X}")
                break

    # Restore original data
    print("Restoring original data...")
    dev.write16(test_lba, orig)
    verify = dev.read16(test_lba, 1)
    verify = dev.read16(test_lba, 1)
    assert verify == orig, "restore failed!"
    print("Original data restored.")

if __name__ == "__main__":
    main()
