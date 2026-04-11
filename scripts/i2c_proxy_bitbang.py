#!/usr/bin/env python3
"""
Bit-bang I2C master via UART proxy on ASM2464PD.

Uses GPIO9 (SCL) and GPIO10 (SDA) to bit-bang I2C transactions
through the proxy firmware running on real hardware.

Usage:
    python3 scripts/i2c_proxy_bitbang.py scan
    python3 scripts/i2c_proxy_bitbang.py read <slave_addr> <reg> [count]
    python3 scripts/i2c_proxy_bitbang.py write <slave_addr> <reg> <byte0> [byte1 ...]
    python3 scripts/i2c_proxy_bitbang.py dump <slave_addr>

Examples:
    python3 scripts/i2c_proxy_bitbang.py scan
    python3 scripts/i2c_proxy_bitbang.py read 0x45 0x00        # read 1 register (16-bit)
    python3 scripts/i2c_proxy_bitbang.py read 0x45 0xFE        # manufacturer ID
    python3 scripts/i2c_proxy_bitbang.py dump 0x45             # dump all INA231 regs
"""
import sys
import time
import argparse

sys.path.insert(0, '.')
from emulate.uart_proxy import UARTProxy

# GPIO registers
GPIO_SCL = 0xC629   # GPIO9
GPIO_SDA = 0xC62A   # GPIO10
GPIO_INPUT = 0xC651  # GPIO8-15 input register
SCL_BIT = 1          # GPIO9 = bit 1 of input reg
SDA_BIT = 2          # GPIO10 = bit 2 of input reg

# GPIO control values
PIN_RELEASE = 0x00   # High-Z input, pullup pulls high (open-drain high)
PIN_LOW = 0x02       # Drive low


class I2CBitBang:
    """Bit-bang I2C master over UART proxy."""

    def __init__(self, proxy, delay=0.001):
        self.proxy = proxy
        self.delay = delay

    def _read_sda(self):
        return (self.proxy.read(GPIO_INPUT) >> SDA_BIT) & 1

    def _scl_high(self):
        self.proxy.write(GPIO_SCL, PIN_RELEASE)

    def _scl_low(self):
        self.proxy.write(GPIO_SCL, PIN_LOW)

    def _sda_high(self):
        self.proxy.write(GPIO_SDA, PIN_RELEASE)

    def _sda_low(self):
        self.proxy.write(GPIO_SDA, PIN_LOW)

    def init(self):
        """Release both lines to idle high."""
        self._sda_high()
        self._scl_high()
        time.sleep(0.01)

    def start(self):
        """I2C START condition: SDA falls while SCL is high."""
        self._sda_high()
        self._scl_high()
        time.sleep(self.delay)
        self._sda_low()
        time.sleep(self.delay)
        self._scl_low()
        time.sleep(self.delay)

    def stop(self):
        """I2C STOP condition: SDA rises while SCL is high."""
        self._sda_low()
        time.sleep(self.delay)
        self._scl_high()
        time.sleep(self.delay)
        self._sda_high()
        time.sleep(self.delay)

    def write_byte(self, byte):
        """Clock out 8 bits MSB-first, return True if slave ACKs."""
        for i in range(8):
            if (byte >> (7 - i)) & 1:
                self._sda_high()
            else:
                self._sda_low()
            time.sleep(self.delay)
            self._scl_high()
            time.sleep(self.delay)
            self._scl_low()
        # ACK bit: release SDA, clock, read
        self._sda_high()
        time.sleep(self.delay)
        self._scl_high()
        time.sleep(self.delay)
        ack = self._read_sda()
        self._scl_low()
        return ack == 0

    def read_byte(self, ack=True):
        """Clock in 8 bits MSB-first, send ACK (ack=True) or NACK (ack=False)."""
        val = 0
        for i in range(8):
            self._sda_high()
            time.sleep(self.delay)
            self._scl_high()
            time.sleep(self.delay)
            val = (val << 1) | self._read_sda()
            self._scl_low()
        # Send ACK/NACK
        if ack:
            self._sda_low()
        else:
            self._sda_high()
        time.sleep(self.delay)
        self._scl_high()
        time.sleep(self.delay)
        self._scl_low()
        self._sda_high()
        return val

    def ping(self, addr):
        """Check if a device ACKs at 7-bit address."""
        self.start()
        ack = self.write_byte((addr << 1) | 0)
        self.stop()
        return ack

    def scan(self, start=0x03, end=0x77):
        """Scan bus, return list of responding 7-bit addresses."""
        found = []
        for addr in range(start, end + 1):
            if self.ping(addr):
                found.append(addr)
        return found

    def write(self, addr, data):
        """Write bytes to device. data[0] is typically the register pointer."""
        self.start()
        if not self.write_byte((addr << 1) | 0):
            self.stop()
            return False
        for b in data:
            if not self.write_byte(b):
                self.stop()
                return False
        self.stop()
        return True

    def read(self, addr, count):
        """Read count bytes from device (after register pointer is set)."""
        self.start()
        if not self.write_byte((addr << 1) | 1):
            self.stop()
            return None
        result = []
        for i in range(count):
            result.append(self.read_byte(ack=(i < count - 1)))
        self.stop()
        return bytes(result)

    def write_read(self, addr, write_data, read_count):
        """Write then repeated-start read. Standard register read pattern."""
        self.start()
        if not self.write_byte((addr << 1) | 0):
            self.stop()
            return None
        for b in write_data:
            self.write_byte(b)
        # Repeated start
        self.start()
        if not self.write_byte((addr << 1) | 1):
            self.stop()
            return None
        result = []
        for i in range(read_count):
            result.append(self.read_byte(ack=(i < read_count - 1)))
        self.stop()
        return bytes(result)

    def read_reg16(self, addr, reg):
        """Read a 16-bit big-endian register (common for TI power monitors)."""
        data = self.write_read(addr, bytes([reg]), 2)
        if data is None:
            return None
        return (data[0] << 8) | data[1]

    def write_reg16(self, addr, reg, value):
        """Write a 16-bit big-endian register."""
        return self.write(addr, bytes([reg, (value >> 8) & 0xFF, value & 0xFF]))


def cmd_scan(i2c, args):
    print("Scanning I2C bus (0x03-0x77)...")
    found = i2c.scan()
    if found:
        print(f"Found {len(found)} device(s):")
        for addr in found:
            print(f"  0x{addr:02X}")
    else:
        print("No devices found.")


def cmd_read(i2c, args):
    addr = int(args.slave_addr, 0)
    reg = int(args.reg, 0)
    count = int(args.count) if args.count else 1

    for r in range(reg, reg + count):
        val = i2c.read_reg16(addr, r)
        if val is not None:
            print(f"  [0x{addr:02X}] reg 0x{r:02X} = 0x{val:04X} ({val})")
        else:
            print(f"  [0x{addr:02X}] reg 0x{r:02X} = NACK")


def cmd_write(i2c, args):
    addr = int(args.slave_addr, 0)
    reg = int(args.reg, 0)
    data_bytes = [int(b, 0) for b in args.bytes]
    payload = bytes([reg] + data_bytes)
    ok = i2c.write(addr, payload)
    if ok:
        print(f"  Wrote {len(data_bytes)} byte(s) to [0x{addr:02X}] reg 0x{reg:02X}")
    else:
        print(f"  Write NACK")


INA231_REGS = {
    0x00: "Configuration",
    0x01: "Shunt Voltage",
    0x02: "Bus Voltage",
    0x03: "Power",
    0x04: "Current",
    0x05: "Calibration",
    0x06: "Mask/Enable",
    0x07: "Alert Limit",
    0xFE: "Manufacturer ID",
    0xFF: "Die ID",
}


def cmd_dump(i2c, args):
    addr = int(args.slave_addr, 0)
    print(f"Dumping registers for device at 0x{addr:02X}:")
    regs = INA231_REGS if addr == 0x45 else {i: f"Reg 0x{i:02X}" for i in range(8)}
    for reg, name in sorted(regs.items()):
        val = i2c.read_reg16(addr, reg)
        if val is not None:
            print(f"  0x{reg:02X} ({name:>20s}) = 0x{val:04X} ({val:5d})")
        else:
            print(f"  0x{reg:02X} ({name:>20s}) = NACK")


def main():
    parser = argparse.ArgumentParser(description="Bit-bang I2C via UART proxy")
    parser.add_argument("-d", "--device", default="ftdi://ftdi:230x/1", help="FTDI device URL")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("scan", help="Scan bus for devices")

    p_read = sub.add_parser("read", help="Read 16-bit register(s)")
    p_read.add_argument("slave_addr", help="7-bit slave address (hex)")
    p_read.add_argument("reg", help="Register address (hex)")
    p_read.add_argument("count", nargs="?", help="Number of consecutive registers")

    p_write = sub.add_parser("write", help="Write bytes to register")
    p_write.add_argument("slave_addr", help="7-bit slave address (hex)")
    p_write.add_argument("reg", help="Register address (hex)")
    p_write.add_argument("bytes", nargs="+", help="Data bytes (hex)")

    p_dump = sub.add_parser("dump", help="Dump all known registers")
    p_dump.add_argument("slave_addr", help="7-bit slave address (hex)")

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return

    proxy = UARTProxy(args.device)
    if not proxy.test_connection():
        print("Proxy connection failed!")
        return 1

    i2c = I2CBitBang(proxy)
    i2c.init()

    try:
        {"scan": cmd_scan, "read": cmd_read, "write": cmd_write, "dump": cmd_dump}[args.command](i2c, args)
    finally:
        # Restore pins to bootloader defaults
        proxy.write(GPIO_SCL, 0x13)
        proxy.write(GPIO_SDA, 0x14)
        proxy.close()


if __name__ == "__main__":
    sys.exit(main() or 0)
