#!/usr/bin/env python3
"""
PCIe Device Scanner for ASM2464PD USB-to-PCIe Bridge

Scans for PCIe devices connected to the ASM2464PD chip over USB.
The ASM2464PD is a USB 3.2 to PCIe bridge chip commonly found in
external NVMe/GPU enclosures.

Device: Bus 004 Device 003: ID 3801:0001 tiny USB 3.2 PCIe TinyEnclosure

Usage:
    python app/scan_pcie.py              # Scan all buses
    python app/scan_pcie.py --bus 1      # Scan specific bus
    python app/scan_pcie.py --verbose    # Show raw config space
    python app/scan_pcie.py --reset      # Reset buses before scan
    python app/scan_pcie.py --deep       # Deep scan with full bridge config (for GPUs)

Requirements:
    - tinygrad (for USB communication library)
    - libusb
    - sudo/root access for USB device access
"""

import sys
import time
import struct
import argparse
from pathlib import Path

# Add tinygrad to path if needed
sys.path.insert(0, str(Path.home() / "tinygrad"))

try:
    from tinygrad.runtime.support.usb import ASM24Controller
except ImportError:
    print("Error: Could not import ASM24Controller from tinygrad")
    print("Make sure tinygrad is installed or available at ~/tinygrad")
    sys.exit(1)

# PCI Configuration Space offsets
PCI_VENDOR_ID = 0x00
PCI_DEVICE_ID = 0x02
PCI_COMMAND = 0x04
PCI_STATUS = 0x06
PCI_REVISION_ID = 0x08
PCI_CLASS_PROG = 0x09
PCI_CLASS_DEVICE = 0x0A
PCI_HEADER_TYPE = 0x0E
PCI_BASE_ADDRESS_0 = 0x10
PCI_PRIMARY_BUS = 0x18
PCI_SECONDARY_BUS = 0x19
PCI_SUBORDINATE_BUS = 0x1A
PCI_MEMORY_BASE = 0x20
PCI_MEMORY_LIMIT = 0x22
PCI_PREF_MEMORY_BASE = 0x24
PCI_PREF_MEMORY_LIMIT = 0x26
PCI_BRIDGE_CONTROL = 0x3E

# PCI Bridge Control bits
PCI_BRIDGE_CTL_BUS_RESET = 0x40
PCI_BRIDGE_CTL_PARITY = 0x01
PCI_BRIDGE_CTL_SERR = 0x02

# PCI Command bits
PCI_COMMAND_IO = 0x01
PCI_COMMAND_MEMORY = 0x02
PCI_COMMAND_MASTER = 0x04

# PCI Class codes
PCI_CLASS_NAMES = {
    0x00: "Unclassified",
    0x01: "Mass Storage Controller",
    0x02: "Network Controller",
    0x03: "Display Controller",
    0x04: "Multimedia Controller",
    0x05: "Memory Controller",
    0x06: "Bridge",
    0x07: "Communication Controller",
    0x08: "System Peripheral",
    0x09: "Input Device Controller",
    0x0A: "Docking Station",
    0x0B: "Processor",
    0x0C: "Serial Bus Controller",
    0x0D: "Wireless Controller",
    0x0E: "Intelligent Controller",
    0x0F: "Satellite Controller",
    0x10: "Encryption Controller",
    0x11: "Signal Processing Controller",
    0x12: "Processing Accelerator",
    0x13: "Non-Essential Instrumentation",
    0xFF: "Unassigned",
}

# Known vendor IDs
VENDOR_NAMES = {
    0x1002: "AMD/ATI",
    0x10DE: "NVIDIA",
    0x8086: "Intel",
    0x1022: "AMD",
    0x144D: "Samsung",
    0x1987: "Phison",
    0x1C5C: "SK Hynix",
    0x15B7: "SanDisk/WD",
    0x1179: "Toshiba/Kioxia",
    0x126F: "Silicon Motion",
    0x1E0F: "KIOXIA",
    0x2646: "Kingston",
    0x1D97: "Shenzhen Longsys",
    0x1CC1: "ADATA",
    0x174C: "ASMedia",
    0x1B21: "ASMedia",  # ASM chips use this VID
}

# Known device IDs
DEVICE_NAMES = {
    # ASMedia bridges
    (0x1B21, 0x2461): "ASM2461 PCIe Switch Port",
    (0x1B21, 0x2464): "ASM2464 USB-PCIe Bridge",
    # AMD GPUs
    (0x1002, 0x1478): "AMD Navi 31 Upstream Port",
    (0x1002, 0x1479): "AMD Navi 31 Downstream Port",
    (0x1002, 0x744C): "AMD Navi 31 (RX 7900 XTX)",
    (0x1002, 0x7480): "AMD Navi 32 (RX 7800 series)",
    (0x1002, 0x7550): "AMD Navi 33 (RX 7600 series)",
    # AMD Audio
    (0x1002, 0xAB30): "AMD Navi 31 HDMI Audio",
}


class PCIeDevice:
    """Represents a PCIe device found during scan."""

    def __init__(self, bus: int, dev: int, fn: int, config_space: bytes):
        self.bus = bus
        self.dev = dev
        self.fn = fn
        self.config_space = config_space

        # Parse basic fields
        self.vendor_id = struct.unpack("<H", config_space[0:2])[0]
        self.device_id = struct.unpack("<H", config_space[2:4])[0]
        self.command = struct.unpack("<H", config_space[4:6])[0]
        self.status = struct.unpack("<H", config_space[6:8])[0]
        self.revision = config_space[8]
        self.prog_if = config_space[9]
        self.subclass = config_space[10]
        self.class_code = config_space[11]
        self.header_type = config_space[14] & 0x7F
        self.multi_function = bool(config_space[14] & 0x80)

        # For bridges, parse bridge-specific fields
        if self.header_type == 1:
            self.primary_bus = config_space[PCI_PRIMARY_BUS]
            self.secondary_bus = config_space[PCI_SECONDARY_BUS]
            self.subordinate_bus = config_space[PCI_SUBORDINATE_BUS]
        else:
            self.primary_bus = None
            self.secondary_bus = None
            self.subordinate_bus = None

    @property
    def is_valid(self) -> bool:
        """Check if this is a valid device (not 0xFFFF vendor ID)."""
        return self.vendor_id != 0xFFFF and self.vendor_id != 0x0000

    @property
    def is_bridge(self) -> bool:
        """Check if this is a PCI-to-PCI bridge."""
        return self.header_type == 1

    @property
    def vendor_name(self) -> str:
        """Get vendor name if known."""
        return VENDOR_NAMES.get(self.vendor_id, f"Unknown (0x{self.vendor_id:04X})")

    @property
    def device_name(self) -> str | None:
        """Get device name if known."""
        return DEVICE_NAMES.get((self.vendor_id, self.device_id), None)

    @property
    def class_name(self) -> str:
        """Get class name."""
        return PCI_CLASS_NAMES.get(
            self.class_code, f"Unknown (0x{self.class_code:02X})"
        )

    @property
    def bdf(self) -> str:
        """Get Bus:Device.Function string."""
        return f"{self.bus:02X}:{self.dev:02X}.{self.fn}"

    def __str__(self) -> str:
        s = f"{self.bdf} "
        if self.device_name:
            s += f"{self.device_name} "
        else:
            s += f"{self.vendor_name} "
        s += f"[{self.vendor_id:04X}:{self.device_id:04X}] {self.class_name}"
        if self.is_bridge:
            s += f" (Bridge -> Bus {self.secondary_bus:02X})"
        return s


class PCIeScanner:
    """Scanner for PCIe devices connected via ASM2464PD."""

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.controller = None
        self.devices: list[PCIeDevice] = []

    def connect(self) -> bool:
        """Connect to the ASM2464PD controller."""
        try:
            print("Connecting to ASM2464PD controller (VID:PID = 3801:0001)...")
            self.controller = ASM24Controller()
            print("Connected successfully!")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            print("\nTroubleshooting:")
            print("  - Make sure the device is connected (lsusb | grep 3801)")
            print("  - Run with sudo for USB access")
            print("  - Check that no other program is using the device")
            return False

    def read_config_space(self, bus: int, dev: int, fn: int, size: int = 256) -> bytes:
        """Read PCIe configuration space for a device."""
        config = bytearray(size)

        for offset in range(0, size, 4):
            try:
                value = self.controller.pcie_cfg_req(
                    offset, bus=bus, dev=dev, fn=fn, size=4
                )
                if value is not None:
                    struct.pack_into("<I", config, offset, value)
                else:
                    struct.pack_into("<I", config, offset, 0xFFFFFFFF)
            except Exception as e:
                if self.verbose:
                    print(
                        f"  Config read error at {bus:02X}:{dev:02X}.{fn} offset 0x{offset:02X}: {e}"
                    )
                struct.pack_into("<I", config, offset, 0xFFFFFFFF)

        return bytes(config)

    def scan_device(self, bus: int, dev: int, fn: int) -> PCIeDevice | None:
        """Scan a single device/function."""
        try:
            # First just read vendor ID to check if device exists
            vendor_id = self.controller.pcie_cfg_req(
                PCI_VENDOR_ID, bus=bus, dev=dev, fn=fn, size=2
            )

            if vendor_id is None or vendor_id == 0xFFFF or vendor_id == 0x0000:
                return None

            # Device exists, read full config space
            config = self.read_config_space(bus, dev, fn)
            device = PCIeDevice(bus, dev, fn, config)

            if device.is_valid:
                return device
        except RuntimeError as e:
            # "Unsupported Request" is expected for non-existent devices
            if "Unsupported Request" not in str(e):
                if self.verbose:
                    print(f"  Error scanning {bus:02X}:{dev:02X}.{fn}: {e}")
        except Exception as e:
            if self.verbose:
                print(f"  Error scanning {bus:02X}:{dev:02X}.{fn}: {e}")

        return None

    def scan_bus(self, bus: int) -> list[PCIeDevice]:
        """Scan all devices on a bus."""
        devices = []

        for dev in range(32):  # 32 devices per bus
            # Check function 0
            device = self.scan_device(bus, dev, 0)
            if device is None:
                continue

            devices.append(device)

            # If multi-function device, scan other functions
            if device.multi_function:
                for fn in range(1, 8):
                    sub_device = self.scan_device(bus, dev, fn)
                    if sub_device:
                        devices.append(sub_device)

        return devices

    def setup_bridge(self, bus: int, secondary_bus: int, subordinate_bus: int):
        """Configure a PCIe bridge for scanning downstream devices."""
        print(f"  Configuring bridge on bus {bus} -> secondary bus {secondary_bus}")

        try:
            # Set bus numbers
            self.controller.pcie_cfg_req(
                PCI_SUBORDINATE_BUS, bus=bus, dev=0, fn=0, value=subordinate_bus, size=1
            )
            self.controller.pcie_cfg_req(
                PCI_SECONDARY_BUS, bus=bus, dev=0, fn=0, value=secondary_bus, size=1
            )
            self.controller.pcie_cfg_req(
                PCI_PRIMARY_BUS, bus=bus, dev=0, fn=0, value=max(0, bus - 1), size=1
            )

            # Set memory windows
            self.controller.pcie_cfg_req(
                PCI_MEMORY_BASE, bus=bus, dev=0, fn=0, value=0x1000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_MEMORY_LIMIT, bus=bus, dev=0, fn=0, value=0x2000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_PREF_MEMORY_BASE, bus=bus, dev=0, fn=0, value=0x2000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_PREF_MEMORY_LIMIT, bus=bus, dev=0, fn=0, value=0xFFFF, size=2
            )

            # Enable bus mastering
            self.controller.pcie_cfg_req(
                PCI_COMMAND,
                bus=bus,
                dev=0,
                fn=0,
                value=PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
                size=1,
            )
        except Exception as e:
            print(f"  Warning: Bridge setup failed: {e}")

    def reset_bus(self, bus: int):
        """Reset a PCIe bus via bridge control."""
        print(f"  Resetting bus {bus}...")
        try:
            self.controller.pcie_cfg_req(
                PCI_BRIDGE_CONTROL,
                bus=bus,
                dev=0,
                fn=0,
                value=PCI_BRIDGE_CTL_BUS_RESET,
                size=1,
            )
            time.sleep(0.1)
            self.controller.pcie_cfg_req(
                PCI_BRIDGE_CONTROL,
                bus=bus,
                dev=0,
                fn=0,
                value=PCI_BRIDGE_CTL_PARITY | PCI_BRIDGE_CTL_SERR,
                size=1,
            )
        except Exception as e:
            print(f"  Warning: Bus reset failed: {e}")

    def rescan_bus(self, bus: int, subordinate_bus: int):
        """
        Rescan a bus by resetting it and configuring bridge.

        This is used during initial enumeration to set up the bridge hierarchy.
        Based on tinygrad's rescan_bus() function.
        """
        print(f"  Rescanning bus {bus} (subordinate={subordinate_bus})...")
        try:
            # Set bus numbers
            self.controller.pcie_cfg_req(
                PCI_SUBORDINATE_BUS, bus=bus, dev=0, fn=0, value=subordinate_bus, size=1
            )
            self.controller.pcie_cfg_req(
                PCI_SECONDARY_BUS, bus=bus, dev=0, fn=0, value=bus + 1, size=1
            )
            self.controller.pcie_cfg_req(
                PCI_PRIMARY_BUS, bus=bus, dev=0, fn=0, value=max(0, bus - 1), size=1
            )

            # Reset the bus
            self.controller.pcie_cfg_req(
                PCI_BRIDGE_CONTROL,
                bus=bus,
                dev=0,
                fn=0,
                value=PCI_BRIDGE_CTL_BUS_RESET,
                size=1,
            )
            time.sleep(0.1)
            self.controller.pcie_cfg_req(
                PCI_BRIDGE_CONTROL,
                bus=bus,
                dev=0,
                fn=0,
                value=PCI_BRIDGE_CTL_PARITY | PCI_BRIDGE_CTL_SERR,
                size=1,
            )

            # Set memory windows
            self.controller.pcie_cfg_req(
                PCI_MEMORY_BASE, bus=bus, dev=0, fn=0, value=0x1000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_MEMORY_LIMIT, bus=bus, dev=0, fn=0, value=0x2000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_PREF_MEMORY_BASE, bus=bus, dev=0, fn=0, value=0x2000, size=2
            )
            self.controller.pcie_cfg_req(
                PCI_PREF_MEMORY_LIMIT, bus=bus, dev=0, fn=0, value=0xFFFF, size=2
            )
        except Exception as e:
            print(f"  Warning: Rescan failed: {e}")

    def scan_all(
        self, max_bus: int = 8, reset: bool = False, deep: bool = False
    ) -> list[PCIeDevice]:
        """
        Scan all PCIe buses for devices.

        The ASM2464PD typically presents as bus 0, with downstream bridges
        leading to the actual devices (NVMe drives, GPUs, etc).

        Args:
            max_bus: Maximum bus number to scan
            reset: Reset buses before scanning
            deep: Deep scan mode - fully configure all bridges including GPU internal bridges
        """
        self.devices = []
        buses_to_scan = [0]
        scanned_buses = set()

        print(f"\nScanning PCIe buses (max bus: {max_bus})...")
        if deep:
            print("Deep scan mode: configuring all bridges for GPU access")
        print("-" * 60)

        # In deep mode, pre-configure the bridge hierarchy like tinygrad does
        if deep:
            print("\nConfiguring bridge hierarchy...")
            # Configure ASM bridges (bus 0 and 1) to reach bus 4
            self.rescan_bus(0, subordinate_bus=max_bus)
            time.sleep(0.1)
            self.rescan_bus(1, subordinate_bus=max_bus)
            time.sleep(0.1)
            # Configure GPU internal bridges (bus 2 and 3)
            self.setup_bridge(2, secondary_bus=3, subordinate_bus=max_bus)
            time.sleep(0.1)
            self.setup_bridge(3, secondary_bus=4, subordinate_bus=max_bus)
            time.sleep(0.1)

        while buses_to_scan:
            bus = buses_to_scan.pop(0)

            if bus in scanned_buses or bus > max_bus:
                continue
            scanned_buses.add(bus)

            print(f"\nBus {bus:02X}:")

            # Setup bridges if needed (for buses > 0)
            if bus > 0 and reset and not deep:
                self.reset_bus(bus - 1)

            devices = self.scan_bus(bus)

            if not devices:
                print("  No devices found")
                continue

            for device in devices:
                print(f"  {device}")
                self.devices.append(device)

                # If this is a bridge, add secondary bus to scan list
                if device.is_bridge and device.secondary_bus is not None:
                    if device.secondary_bus not in scanned_buses:
                        if not deep:
                            # In non-deep mode, setup bridge before scanning downstream
                            self.setup_bridge(device.bus, device.secondary_bus, max_bus)
                        buses_to_scan.append(device.secondary_bus)

                if self.verbose:
                    self.print_config_space(device)

        return self.devices

    def print_config_space(self, device: PCIeDevice):
        """Print raw configuration space in hex dump format."""
        print(f"\n    Config space for {device.bdf}:")
        data = device.config_space
        for i in range(0, len(data), 16):
            hex_str = " ".join(f"{b:02X}" for b in data[i : i + 16])
            ascii_str = "".join(
                chr(b) if 32 <= b < 127 else "." for b in data[i : i + 16]
            )
            print(f"    {i:04X}: {hex_str}  {ascii_str}")

    def pcie_mem_read(self, address: int, size: int = 4) -> int:
        """
        Read from PCIe memory space.

        Args:
            address: 64-bit PCIe memory address
            size: Number of bytes to read (1, 2, or 4)

        Returns:
            Value read from PCIe memory
        """
        return self.controller.pcie_mem_req(address, value=None, size=size)

    def pcie_mem_write(self, address: int, value: int, size: int = 4):
        """
        Write to PCIe memory space.

        Args:
            address: 64-bit PCIe memory address
            value: Value to write
            size: Number of bytes to write (1, 2, or 4)
        """
        self.controller.pcie_mem_req(address, value=value, size=size)

    def pcie_mem_read_block(self, address: int, length: int) -> bytes:
        """
        Read a block of data from PCIe memory space.

        Args:
            address: Starting 64-bit PCIe memory address
            length: Number of bytes to read

        Returns:
            Bytes read from PCIe memory
        """
        data = bytearray()
        for offset in range(0, length, 4):
            remaining = min(4, length - offset)
            try:
                value = self.pcie_mem_read(address + offset, size=4)
                if value is not None:
                    data.extend(struct.pack("<I", value)[:remaining])
                else:
                    data.extend(b"\xff" * remaining)
            except Exception as e:
                if self.verbose:
                    print(f"  Read error at 0x{address + offset:X}: {e}")
                data.extend(b"\xff" * remaining)
        return bytes(data)

    def pcie_mem_write_block(self, address: int, data: bytes):
        """
        Write a block of data to PCIe memory space.

        Args:
            address: Starting 64-bit PCIe memory address
            data: Bytes to write
        """
        # Pad to 4-byte alignment
        padded = data + b"\x00" * (4 - len(data) % 4) if len(data) % 4 else data

        for offset in range(0, len(padded), 4):
            value = struct.unpack("<I", padded[offset : offset + 4])[0]
            self.pcie_mem_write(address + offset, value, size=4)

    def get_gpu_bar_address(
        self, bus: int = 4, dev: int = 0, fn: int = 0
    ) -> tuple[int, int] | None:
        """
        Get GPU BAR0 (MMIO) and BAR2 (VRAM) addresses from config space.

        AMD GPUs typically have:
        - BAR0: MMIO registers (256MB)
        - BAR2: VRAM (full size, 64-bit)

        Returns:
            Tuple of (bar0_addr, bar2_addr) or None if not found
        """
        try:
            # Read BAR0 (32-bit)
            bar0 = self.controller.pcie_cfg_req(0x10, bus=bus, dev=dev, fn=fn, size=4)
            if bar0 is None:
                return None
            bar0_addr = bar0 & 0xFFFFFFF0  # Mask out lower bits

            # Read BAR2 (64-bit, spans BAR2 and BAR3)
            bar2_lo = self.controller.pcie_cfg_req(
                0x18, bus=bus, dev=dev, fn=fn, size=4
            )
            bar2_hi = self.controller.pcie_cfg_req(
                0x1C, bus=bus, dev=dev, fn=fn, size=4
            )
            if bar2_lo is None or bar2_hi is None:
                return None
            bar2_addr = (bar2_hi << 32) | (bar2_lo & 0xFFFFFFF0)

            return (bar0_addr, bar2_addr)
        except Exception as e:
            if self.verbose:
                print(f"  Error reading BARs: {e}")
            return None

    def print_summary(self):
        """Print summary of found devices."""
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)

        if not self.devices:
            print("No PCIe devices found!")
            return

        print(f"Found {len(self.devices)} device(s):\n")

        # Group by class
        by_class: dict[str, list[PCIeDevice]] = {}
        for dev in self.devices:
            by_class.setdefault(dev.class_name, []).append(dev)

        for class_name, devices in sorted(by_class.items()):
            print(f"{class_name}:")
            for dev in devices:
                print(
                    f"  {dev.bdf} - {dev.vendor_name} [{dev.vendor_id:04X}:{dev.device_id:04X}]"
                )
            print()

        # Check for known devices (GPUs, NVMe)
        gpus = [d for d in self.devices if d.class_code == 0x03]
        nvme = [d for d in self.devices if d.class_code == 0x01 and d.subclass == 0x08]

        if gpus:
            print("GPU(s) detected:")
            for gpu in gpus:
                print(f"  {gpu}")

        if nvme:
            print("NVMe drive(s) detected:")
            for drive in nvme:
                print(f"  {drive}")


def main():
    parser = argparse.ArgumentParser(
        description="Scan for PCIe devices connected via ASM2464PD USB-to-PCIe bridge"
    )
    parser.add_argument(
        "--bus", "-b", type=int, default=None, help="Scan only specific bus number"
    )
    parser.add_argument(
        "--max-bus",
        "-m",
        type=int,
        default=8,
        help="Maximum bus number to scan (default: 8)",
    )
    parser.add_argument(
        "--reset", "-r", action="store_true", help="Reset buses before scanning"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Show verbose output including raw config space",
    )
    parser.add_argument(
        "--deep",
        "-d",
        action="store_true",
        help="Deep scan: fully configure bridge hierarchy for GPU access",
    )
    parser.add_argument(
        "--dma-test",
        action="store_true",
        help="Test DMA read/write to GPU memory (requires --deep)",
    )
    parser.add_argument(
        "--read-mem",
        type=str,
        default=None,
        metavar="ADDR",
        help="Read 256 bytes from PCIe memory address (hex, e.g., 0x10000000)",
    )
    parser.add_argument(
        "--write-mem",
        type=str,
        nargs=2,
        default=None,
        metavar=("ADDR", "VALUE"),
        help="Write 32-bit value to PCIe memory address (hex)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("ASM2464PD PCIe Device Scanner")
    print("=" * 60)

    scanner = PCIeScanner(verbose=args.verbose)

    if not scanner.connect():
        sys.exit(1)

    try:
        if args.bus is not None:
            # Scan specific bus only
            print(f"\nScanning bus {args.bus}...")
            devices = scanner.scan_bus(args.bus)
            for dev in devices:
                print(f"  {dev}")
                if args.verbose:
                    scanner.print_config_space(dev)
        else:
            # Scan all buses
            scanner.scan_all(max_bus=args.max_bus, reset=args.reset, deep=args.deep)
            scanner.print_summary()

    except KeyboardInterrupt:
        print("\nScan interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError during scan: {e}")
        if args.verbose:
            import traceback

            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
