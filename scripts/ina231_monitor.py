#!/usr/bin/env python3
"""
Monitor the INA231 on ASM2464PD via E4/E5 USB control transfers.

This reuses the GPIO9/GPIO10 bit-banged I2C path from
scripts/i2c_proxy_bitbang.py, but swaps the transport to the custom
firmware's vendor control messages:

  E4 IN : wValue = XDATA address, wIndex[15:8] = bank, wLength = size
  E5 OUT: wValue = XDATA address, wIndex[15:8] = bank, wIndex[7:0] = value

The board's INA231 shunt network is nominally two 1 mOhm resistors in series,
but measured power matches an effective sense resistance closer to 2.78 mOhm,
so that is the default used here.
"""

import argparse
import sys
import time

import usb.core
import usb.util


SUPPORTED_CONTROLLERS = [
    (0xADD1, 0x0001),
]

# GPIO registers used by scripts/i2c_proxy_bitbang.py
GPIO_SCL = 0xC629
GPIO_SDA = 0xC62A
GPIO_INPUT = 0xC651
SCL_BIT = 1
SDA_BIT = 2

PIN_RELEASE = 0x00
PIN_LOW = 0x02

INA231_REG_SHUNT = 0x01
INA231_REG_BUS = 0x02
INA231_REG_CONFIG = 0x00
INA231_REG_CAL = 0x05
INA231_REG_MFG = 0xFE
INA231_REG_DIE = 0xFF

INA231_SHUNT_LSB_UV = 2.5
INA231_BUS_LSB_V = 0.00125
DEFAULT_SHUNT_OHMS = 0.00278
DEFAULT_INA231_CONFIG = 0x4127


def sign_extend_16(value):
    return value - 0x10000 if value & 0x8000 else value


def fmt_timestamp(now=None):
    if now is None:
        now = time.time()
    whole = time.localtime(now)
    millis = int((now - int(now)) * 1000)
    return time.strftime("%H:%M:%S", whole) + f".{millis:03d}"


def find_device(vid=None, pid=None):
    if vid is not None and pid is not None:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            raise RuntimeError(f"Device {vid:04X}:{pid:04X} not found")
        return dev, vid, pid

    for cand_vid, cand_pid in SUPPORTED_CONTROLLERS:
        dev = usb.core.find(idVendor=cand_vid, idProduct=cand_pid)
        if dev is not None:
            return dev, cand_vid, cand_pid

    supported = ", ".join(f"{vid:04X}:{pid:04X}" for vid, pid in SUPPORTED_CONTROLLERS)
    raise RuntimeError(f"No ASM2464 device found (looked for {supported})")


def setup_device(dev):
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (NotImplementedError, usb.core.USBError):
        pass

    try:
        dev.set_configuration()
    except usb.core.USBError as exc:
        msg = str(exc).lower()
        if "busy" not in msg and "already" not in msg:
            raise


class USBXData:
    def __init__(self, dev, timeout_ms=1000):
        self.dev = dev
        self.timeout_ms = timeout_ms

    def read(self, addr, size=1, bank=0):
        data = self.dev.ctrl_transfer(
            0xC0,
            0xE4,
            addr & 0xFFFF,
            (bank & 0xFF) << 8,
            size,
            timeout=self.timeout_ms,
        )
        data = bytes(data)
        if len(data) != size:
            raise RuntimeError(f"short E4 read from 0x{addr:04X}: expected {size}, got {len(data)}")
        return data

    def read8(self, addr, bank=0):
        return self.read(addr, 1, bank)[0]

    def write8(self, addr, value, bank=0):
        self.dev.ctrl_transfer(
            0x40,
            0xE5,
            addr & 0xFFFF,
            ((bank & 0xFF) << 8) | (value & 0xFF),
            b"",
            timeout=self.timeout_ms,
        )


class I2CBitBang:
    def __init__(self, xdata, delay=0.0):
        self.xdata = xdata
        self.delay = delay

    def _pause(self):
        if self.delay:
            time.sleep(self.delay)

    def _read_sda(self):
        return (self.xdata.read8(GPIO_INPUT) >> SDA_BIT) & 1

    def _scl_high(self):
        self.xdata.write8(GPIO_SCL, PIN_RELEASE)

    def _scl_low(self):
        self.xdata.write8(GPIO_SCL, PIN_LOW)

    def _sda_high(self):
        self.xdata.write8(GPIO_SDA, PIN_RELEASE)

    def _sda_low(self):
        self.xdata.write8(GPIO_SDA, PIN_LOW)

    def init(self):
        self._sda_high()
        self._scl_high()
        time.sleep(0.01)

    def start(self):
        self._sda_high()
        self._scl_high()
        self._pause()
        self._sda_low()
        self._pause()
        self._scl_low()
        self._pause()

    def stop(self):
        self._sda_low()
        self._pause()
        self._scl_high()
        self._pause()
        self._sda_high()
        self._pause()

    def write_byte(self, byte):
        for i in range(8):
            if (byte >> (7 - i)) & 1:
                self._sda_high()
            else:
                self._sda_low()
            self._pause()
            self._scl_high()
            self._pause()
            self._scl_low()
        self._sda_high()
        self._pause()
        self._scl_high()
        self._pause()
        ack = self._read_sda()
        self._scl_low()
        return ack == 0

    def read_byte(self, ack=True):
        value = 0
        for _ in range(8):
            self._sda_high()
            self._pause()
            self._scl_high()
            self._pause()
            value = (value << 1) | self._read_sda()
            self._scl_low()
        if ack:
            self._sda_low()
        else:
            self._sda_high()
        self._pause()
        self._scl_high()
        self._pause()
        self._scl_low()
        self._sda_high()
        return value

    def write_read(self, addr, write_data, read_count):
        self.start()
        if not self.write_byte((addr << 1) | 0):
            self.stop()
            return None
        for byte in write_data:
            if not self.write_byte(byte):
                self.stop()
                return None
        self.start()
        if not self.write_byte((addr << 1) | 1):
            self.stop()
            return None
        result = []
        for index in range(read_count):
            result.append(self.read_byte(ack=(index < read_count - 1)))
        self.stop()
        return bytes(result)

    def write(self, addr, data):
        self.start()
        if not self.write_byte((addr << 1) | 0):
            self.stop()
            return False
        for byte in data:
            if not self.write_byte(byte):
                self.stop()
                return False
        self.stop()
        return True

    def read_reg16(self, addr, reg):
        data = self.write_read(addr, bytes([reg]), 2)
        if data is None:
            return None
        return (data[0] << 8) | data[1]

    def write_reg16(self, addr, reg, value):
        return self.write(addr, bytes([reg, (value >> 8) & 0xFF, value & 0xFF]))


def read_sensor_info(i2c, addr):
    return {
        "config": i2c.read_reg16(addr, INA231_REG_CONFIG),
        "cal": i2c.read_reg16(addr, INA231_REG_CAL),
        "mfg": i2c.read_reg16(addr, INA231_REG_MFG),
        "die": i2c.read_reg16(addr, INA231_REG_DIE),
    }


def ensure_sensor_config(i2c, addr, config_value):
    current = i2c.read_reg16(addr, INA231_REG_CONFIG)
    if current is None:
        raise RuntimeError(f"INA231 at 0x{addr:02X} did not ACK config read")
    if current == config_value:
        return current, False
    if not i2c.write_reg16(addr, INA231_REG_CONFIG, config_value):
        raise RuntimeError(f"INA231 at 0x{addr:02X} did not ACK config write")
    time.sleep(0.05)
    updated = i2c.read_reg16(addr, INA231_REG_CONFIG)
    if updated != config_value:
        raise RuntimeError(
            f"INA231 config write failed: expected 0x{config_value:04X}, got "
            f"{('0x%04X' % updated) if updated is not None else 'NACK'}"
        )
    return updated, True


def read_sample(i2c, addr, shunt_ohms):
    shunt_raw = i2c.read_reg16(addr, INA231_REG_SHUNT)
    bus_raw = i2c.read_reg16(addr, INA231_REG_BUS)
    if shunt_raw is None or bus_raw is None:
        return None

    shunt_volts = sign_extend_16(shunt_raw) * INA231_SHUNT_LSB_UV * 1e-6
    bus_volts = bus_raw * INA231_BUS_LSB_V
    current_amps = shunt_volts / shunt_ohms
    power_watts = bus_volts * current_amps

    return {
        "shunt_raw": shunt_raw,
        "bus_raw": bus_raw,
        "shunt_volts": shunt_volts,
        "bus_volts": bus_volts,
        "current_amps": current_amps,
        "power_watts": power_watts,
    }


def cleanup_gpio(xdata):
    xdata.write8(GPIO_SCL, 0x13)
    xdata.write8(GPIO_SDA, 0x14)


def print_sample_header():
    print("time          bus[V]    current[A]  power[W]", flush=True)
    print("------------  -------  -----------  --------", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Monitor INA231 via ASM2464 E4/E5 control transfers")
    parser.add_argument("--vid", type=lambda x: int(x, 0), help="USB vendor ID")
    parser.add_argument("--pid", type=lambda x: int(x, 0), help="USB product ID")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x45, help="INA231 7-bit I2C address")
    parser.add_argument("--interval-ms", type=float, default=100.0, help="sample period in milliseconds")
    parser.add_argument("--bit-delay", type=float, default=0.0, help="extra delay between I2C edges in seconds")
    parser.add_argument("--shunt-ohms", type=float, default=DEFAULT_SHUNT_OHMS, help="shunt resistance in ohms")
    parser.add_argument("--no-configure", action="store_true", help="do not program INA231 config register")
    parser.add_argument("--count", type=int, help="number of samples to capture")
    args = parser.parse_args()

    if (args.vid is None) != (args.pid is None):
        parser.error("--vid and --pid must be supplied together")
    if args.shunt_ohms <= 0:
        parser.error("--shunt-ohms must be positive")
    if args.interval_ms <= 0:
        parser.error("--interval-ms must be positive")

    dev, vid, pid = find_device(args.vid, args.pid)
    setup_device(dev)
    xdata = USBXData(dev)
    i2c = I2CBitBang(xdata, delay=args.bit_delay)
    i2c.init()

    interval_s = args.interval_ms / 1000.0

    try:
        configured = False
        prev_config = None
        if not args.no_configure:
            prev_config = i2c.read_reg16(args.addr, INA231_REG_CONFIG)
            _, configured = ensure_sensor_config(i2c, args.addr, DEFAULT_INA231_CONFIG)

        info = read_sensor_info(i2c, args.addr)
        if any(value is None for value in info.values()):
            raise RuntimeError(f"INA231 at 0x{args.addr:02X} did not ACK")

        if configured:
            print(
                f"Configured INA231: 0x{prev_config:04X} -> 0x{info['config']:04X}",
                flush=True,
            )

        print(
            f"Device {vid:04X}:{pid:04X}  INA231@0x{args.addr:02X}  "
            f"shunt={args.shunt_ohms * 1e3:.3f} mOhm  "
            f"config=0x{info['config']:04X}  cal=0x{info['cal']:04X}  "
            f"mfg=0x{info['mfg']:04X}  die=0x{info['die']:04X}",
            flush=True,
        )
        print_sample_header()

        next_sample = time.monotonic()
        sample_count = 0
        while args.count is None or sample_count < args.count:
            now = time.monotonic()
            if now < next_sample:
                time.sleep(next_sample - now)

            wall_now = time.time()
            sample = read_sample(i2c, args.addr, args.shunt_ohms)
            if sample is None:
                print(f"{fmt_timestamp(wall_now)}  {'NACK':>7}  {'-':>11}  {'-':>8}", flush=True)
            else:
                print(
                    f"{fmt_timestamp(wall_now)}  "
                    f"{sample['bus_volts']:7.4f}  "
                    f"{sample['current_amps']:11.3f}  "
                    f"{sample['power_watts']:+8.3f}",
                    flush=True,
                )

            sample_count += 1
            next_sample += interval_s
            if next_sample < time.monotonic():
                next_sample = time.monotonic()

    except KeyboardInterrupt:
        return 0
    finally:
        try:
            cleanup_gpio(xdata)
        finally:
            usb.util.dispose_resources(dev)

    return 0


if __name__ == "__main__":
    sys.exit(main())
