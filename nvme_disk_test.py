#!/usr/bin/env python3
"""Test NVMe disk via stock firmware BOT SCSI: init, read, write, verify."""

import ctypes, struct, time, os, sys
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

class BOTDev:
    def __init__(self):
        self.usb = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)
        self._tag = 0

    def scsi(self, cdb, xfer_len=0, direction_in=True, timeout=10000):
        """Send SCSI command via BOT. Returns (data, csw_status)."""
        self._tag += 1
        flags = 0x80 if direction_in else 0x00
        cbw = struct.pack('<III', 0x43425355, self._tag, xfer_len)
        cbw += struct.pack('BBB', flags, 0, len(cdb))
        cbw += cdb + b'\x00' * (16 - len(cdb))
        self.usb._bulk_out(2, cbw)

        data = b''
        if xfer_len > 0:
            buf = (ctypes.c_ubyte * xfer_len)()
            xfer = ctypes.c_int(0)
            if direction_in:
                ret = libusb.libusb_bulk_transfer(self.usb.handle, 0x81, buf, xfer_len, ctypes.byref(xfer), timeout)
            else:
                ctypes.memmove(buf, b'\x00' * xfer_len, xfer_len)  # placeholder
                ret = libusb.libusb_bulk_transfer(self.usb.handle, 0x02, buf, xfer_len, ctypes.byref(xfer), timeout)
            if ret < 0:
                # Try to recover CSW
                csw = (ctypes.c_ubyte * 13)()
                xfer2 = ctypes.c_int(0)
                libusb.libusb_bulk_transfer(self.usb.handle, 0x81, csw, 13, ctypes.byref(xfer2), 2000)
                return b'', csw[12] if xfer2.value == 13 else -1
            data = bytes(buf[:xfer.value])

        csw = (ctypes.c_ubyte * 13)()
        xfer2 = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.usb.handle, 0x81, csw, 13, ctypes.byref(xfer2), timeout)
        return data, csw[12] if xfer2.value == 13 else -1

    def scsi_out(self, cdb, data, timeout=10000):
        """Send SCSI command with data-out phase."""
        self._tag += 1
        cbw = struct.pack('<III', 0x43425355, self._tag, len(data))
        cbw += struct.pack('BBB', 0x00, 0, len(cdb))
        cbw += cdb + b'\x00' * (16 - len(cdb))
        self.usb._bulk_out(2, cbw)

        buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.usb.handle, 0x02, buf, len(data), ctypes.byref(xfer), timeout)
        if ret < 0:
            csw = (ctypes.c_ubyte * 13)()
            xfer2 = ctypes.c_int(0)
            libusb.libusb_bulk_transfer(self.usb.handle, 0x81, csw, 13, ctypes.byref(xfer2), 2000)
            return csw[12] if xfer2.value == 13 else -1

        csw = (ctypes.c_ubyte * 13)()
        xfer2 = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(self.usb.handle, 0x81, csw, 13, ctypes.byref(xfer2), timeout)
        return csw[12] if xfer2.value == 13 else -1

    def read_reg(self, addr, size=1):
        """Read XDATA register via SCSI vendor command 0xE4 (stock firmware encoding)."""
        addr_enc = (addr & 0x1FFFF) | 0x500000
        cdb = struct.pack('>BBBHB', 0xE4, size, addr_enc >> 16, addr_enc & 0xFFFF, 0)
        data, s = self.scsi(cdb, size)
        assert s == 0, f"E4 read 0x{addr:04X} failed: status={s}"
        return data

    def write_reg(self, addr, val):
        """Write XDATA register via SCSI vendor command 0xE5 (stock firmware encoding)."""
        addr_enc = (addr & 0x1FFFF) | 0x500000
        cdb = struct.pack('>BBBHB', 0xE5, val, addr_enc >> 16, addr_enc & 0xFFFF, 0)
        _, s = self.scsi(cdb, 0)
        assert s == 0, f"E5 write 0x{addr:04X}=0x{val:02X} failed: status={s}"

    def dump_regs(self):
        """Dump key PCIe/NVMe link state registers."""
        regs = [
            (0xB450, "LTSSM_STATE"),
            (0xB451, "LTSSM_B451"),
            (0xB455, "LTSSM_B455"),
            (0xB434, "PCIE_LINK_STATE"),
            (0xB436, "PCIE_LANE_CONFIG"),
            (0xB432, "POWER_CTRL_B432"),
            (0xB431, "TUNNEL_LINK_STATUS"),
            (0xB480, "PCIE_PERST_CTRL"),
            (0xC659, "PCIE_LANE_CTRL"),
            (0xB403, "TUNNEL_CTRL_B403"),
        ]
        for addr, name in regs:
            val = self.read_reg(addr)
            print(f"  0x{addr:04X} {name:24s} = 0x{val[0]:02X}")

    def inquiry(self):
        data, s = self.scsi(b'\x12\x00\x00\x00\x24\x00', 36)
        assert s == 0, f"INQUIRY failed: status={s}"
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
            # Request sense
            sense, _ = self.scsi(b'\x03\x00\x00\x00\x12\x00', 18)
            sk = sense[2] & 0xF if len(sense) > 2 else -1
            asc = sense[12] if len(sense) > 12 else -1
            ascq = sense[13] if len(sense) > 13 else -1
            print(f"  TUR attempt {i+1}: NOT READY sense={sk:X}/{asc:02X}/{ascq:02X}")
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
        # DLD2 bit (byte 1 bit 0) is READ_16-only; MSC engine can't map to READ_10
        cdb = struct.pack('>BBQIBB', 0x88, 0x01, lba, count, 0, 0)
        data, s = self.scsi(cdb, count * 512)
        assert s == 0, f"READ(16) LBA={lba} failed: status={s}"
        return data

    def write16(self, lba, data):
        count = len(data) // 512
        assert count * 512 == len(data), "data must be multiple of 512"
        # DLD2 bit (byte 1 bit 0) is WRITE_16-only; MSC engine can't map to WRITE_10
        cdb = struct.pack('>BBQIBB', 0x8A, 0x01, lba, count, 0, 0)
        s = self.scsi_out(cdb, data)
        assert s == 0, f"WRITE(16) LBA={lba} failed: status={s}"

def main():
    dev = BOTDev()

    if "--dump-regs" in sys.argv:
        print("=== Register Dump ===")
        dev.dump_regs()
        return

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
    assert verify == orig, "restore failed!"
    verify = dev.read16(test_lba, 1)
    print("Original data restored.")

if __name__ == "__main__":
    main()
