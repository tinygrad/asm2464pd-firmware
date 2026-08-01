#!/usr/bin/env python3
"""
Test USB messages against the real emulated device.

This tests against the USB device created by emulate/usb_device.py using
the raw-gadget/dummy_hcd infrastructure.

Prerequisites:
    sudo modprobe dummy_hcd raw_gadget
    sudo python emulate/usb_device.py --speed high

Then run:
    sudo python test/test_usb_device.py

Protocol reference from ~/tinygrad:
- tinygrad/runtime/support/usb.py: USB3 class, ASM24Controller
- extra/usbgpu/patch.py: firmware reflash protocol
"""

import sys
import struct
import time
import ctypes
from pathlib import Path

# Try to import libusb
try:
    import usb.core
    import usb.util
    HAVE_PYUSB = True
except ImportError:
    HAVE_PYUSB = False
    print("pyusb not available, trying direct ctypes")

# Constants
ASM2464_VID_ASMEDIA = 0x174C
ASM2464_VID_TINYGRAD = 0x3801
ASM2464_PIDS = [0x2464, 0x2463, 0x2461, 0x0001]

# USB request types
USB_TYPE_STANDARD = 0x00
USB_TYPE_CLASS = 0x20
USB_TYPE_VENDOR = 0x40

# Standard USB requests
USB_REQ_GET_DESCRIPTOR = 0x06
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_SET_CONFIGURATION = 0x09

# Descriptor types
USB_DT_DEVICE = 0x01
USB_DT_CONFIG = 0x02
USB_DT_STRING = 0x03


class ASM2464USBTest:
    """Test class for ASM2464 USB device."""

    def __init__(self):
        self.dev = None
        self.handle = None

    def find_device(self):
        """Find the ASM2464 device."""
        if not HAVE_PYUSB:
            print("ERROR: pyusb not available")
            return False

        # Try ASMedia VID first
        for pid in ASM2464_PIDS:
            dev = usb.core.find(idVendor=ASM2464_VID_ASMEDIA, idProduct=pid)
            if dev:
                self.dev = dev
                print(f"Found device: {ASM2464_VID_ASMEDIA:04X}:{pid:04X}")
                return True

        # Try tinygrad VID
        for pid in ASM2464_PIDS:
            dev = usb.core.find(idVendor=ASM2464_VID_TINYGRAD, idProduct=pid)
            if dev:
                self.dev = dev
                print(f"Found device: {ASM2464_VID_TINYGRAD:04X}:{pid:04X}")
                return True

        print("Device not found")
        return False

    def setup(self):
        """Set up USB connection."""
        if not self.dev:
            return False

        try:
            # Detach kernel driver if attached
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
                print("Detached kernel driver")

            # Set configuration
            self.dev.set_configuration()
            print("Set configuration")

            return True
        except usb.core.USBError as e:
            print(f"Setup failed: {e}")
            return False

    def test_get_device_descriptor(self):
        """Test GET_DESCRIPTOR for device descriptor."""
        print("\n=== Test: GET_DESCRIPTOR (Device) ===")

        try:
            # Request device descriptor
            desc = self.dev.ctrl_transfer(
                0x80,  # bmRequestType: device-to-host, standard, device
                USB_REQ_GET_DESCRIPTOR,  # bRequest
                (USB_DT_DEVICE << 8) | 0,  # wValue: descriptor type and index
                0,  # wIndex
                18  # wLength
            )

            print(f"  Received {len(desc)} bytes")
            print(f"  Raw: {bytes(desc).hex()}")

            if len(desc) >= 18:
                bLength = desc[0]
                bDescriptorType = desc[1]
                bcdUSB = struct.unpack('<H', bytes(desc[2:4]))[0]
                bDeviceClass = desc[4]
                bDeviceSubClass = desc[5]
                bDeviceProtocol = desc[6]
                bMaxPacketSize0 = desc[7]
                idVendor = struct.unpack('<H', bytes(desc[8:10]))[0]
                idProduct = struct.unpack('<H', bytes(desc[10:12]))[0]
                bcdDevice = struct.unpack('<H', bytes(desc[12:14]))[0]
                iManufacturer = desc[14]
                iProduct = desc[15]
                iSerialNumber = desc[16]
                bNumConfigurations = desc[17]

                print(f"  bLength: {bLength}")
                print(f"  bDescriptorType: {bDescriptorType}")
                print(f"  bcdUSB: {bcdUSB:04X} (USB {bcdUSB>>8}.{(bcdUSB>>4)&0xF}{bcdUSB&0xF})")
                print(f"  VID:PID: {idVendor:04X}:{idProduct:04X}")
                print(f"  bMaxPacketSize0: {bMaxPacketSize0}")
                print(f"  bNumConfigurations: {bNumConfigurations}")

                assert bLength == 18, f"Device descriptor length should be 18, got {bLength}"
                assert bDescriptorType == 1, f"Descriptor type should be 1, got {bDescriptorType}"
                print("  PASSED")
                return True
            else:
                print("  FAILED: Descriptor too short")
                return False

        except usb.core.USBError as e:
            print(f"  FAILED: {e}")
            return False

    def test_get_config_descriptor(self):
        """Test GET_DESCRIPTOR for configuration descriptor."""
        print("\n=== Test: GET_DESCRIPTOR (Config) ===")

        try:
            # First get header only (9 bytes)
            desc = self.dev.ctrl_transfer(
                0x80, USB_REQ_GET_DESCRIPTOR,
                (USB_DT_CONFIG << 8) | 0, 0, 9
            )

            print(f"  Header: {bytes(desc).hex()}")

            if len(desc) >= 9:
                bLength = desc[0]
                bDescriptorType = desc[1]
                wTotalLength = struct.unpack('<H', bytes(desc[2:4]))[0]
                bNumInterfaces = desc[4]
                bConfigurationValue = desc[5]

                print(f"  wTotalLength: {wTotalLength}")
                print(f"  bNumInterfaces: {bNumInterfaces}")

                # Now get full config
                full_desc = self.dev.ctrl_transfer(
                    0x80, USB_REQ_GET_DESCRIPTOR,
                    (USB_DT_CONFIG << 8) | 0, 0, wTotalLength
                )
                print(f"  Full descriptor: {len(full_desc)} bytes")
                print(f"  Raw: {bytes(full_desc[:64]).hex()}...")

                assert bDescriptorType == 2, f"Descriptor type should be 2, got {bDescriptorType}"
                assert bNumInterfaces >= 1, f"Should have at least 1 interface"
                print("  PASSED")
                return True

        except usb.core.USBError as e:
            print(f"  FAILED: {e}")
            return False

    def test_get_string_descriptor(self):
        """Test GET_DESCRIPTOR for string descriptors."""
        print("\n=== Test: GET_DESCRIPTOR (Strings) ===")

        try:
            # String 0: Language IDs
            desc = self.dev.ctrl_transfer(
                0x80, USB_REQ_GET_DESCRIPTOR,
                (USB_DT_STRING << 8) | 0, 0, 255
            )

            print(f"  String 0 (languages): {bytes(desc).hex()}")

            if len(desc) >= 4:
                lang_id = struct.unpack('<H', bytes(desc[2:4]))[0]
                print(f"  Language ID: 0x{lang_id:04X}")

                # Try to get string 1 (manufacturer) and 2 (product)
                for idx in [1, 2]:
                    try:
                        str_desc = self.dev.ctrl_transfer(
                            0x80, USB_REQ_GET_DESCRIPTOR,
                            (USB_DT_STRING << 8) | idx, lang_id, 255
                        )
                        if len(str_desc) > 2:
                            # Decode UTF-16LE string
                            string = bytes(str_desc[2:]).decode('utf-16-le', errors='replace')
                            print(f"  String {idx}: {string}")
                    except usb.core.USBError:
                        print(f"  String {idx}: (not available)")

                print("  PASSED")
                return True

        except usb.core.USBError as e:
            print(f"  FAILED: {e}")
            return False

    def test_e4_read(self):
        """Test E4 vendor command (read XDATA)."""
        print("\n=== Test: E4 Read Command ===")

        try:
            # E4 read format: bmRequestType=0xC0, bRequest=0xE4
            # wValue = size, wIndex = address
            test_addr = 0x0100
            size = 4

            # Address format: (addr & 0x1FFFF) | 0x500000
            # But for control transfer, just use raw address

            result = self.dev.ctrl_transfer(
                0xC0,  # bmRequestType: device-to-host, vendor, device
                0xE4,  # bRequest: E4 read
                size,  # wValue: size
                test_addr,  # wIndex: address
                size  # wLength
            )

            print(f"  Read from 0x{test_addr:04X}: {bytes(result).hex()}")
            print("  PASSED")
            return True

        except usb.core.USBError as e:
            print(f"  FAILED: {e}")
            return False

    def test_e5_write(self):
        """Test E5 vendor command (write XDATA)."""
        print("\n=== Test: E5 Write Command ===")

        try:
            # E5 write format: bmRequestType=0x40, bRequest=0xE5
            # wValue = value, wIndex = address
            test_addr = 0x0100
            test_value = 0x42

            result = self.dev.ctrl_transfer(
                0x40,  # bmRequestType: host-to-device, vendor, device
                0xE5,  # bRequest: E5 write
                test_value,  # wValue: value
                test_addr,  # wIndex: address
                b''  # No data phase
            )

            print(f"  Wrote 0x{test_value:02X} to 0x{test_addr:04X}")

            # Read back to verify
            readback = self.dev.ctrl_transfer(
                0xC0, 0xE4, 1, test_addr, 1
            )
            print(f"  Readback: 0x{readback[0]:02X}")

            if readback[0] == test_value:
                print("  PASSED")
                return True
            else:
                print(f"  FAILED: Expected 0x{test_value:02X}, got 0x{readback[0]:02X}")
                return False

        except usb.core.USBError as e:
            print(f"  FAILED: {e}")
            return False

    def test_bulk_endpoints(self):
        """Test bulk endpoint enumeration."""
        print("\n=== Test: Bulk Endpoints ===")

        try:
            cfg = self.dev.get_active_configuration()
            intf = cfg[(0, 0)]

            print(f"  Interface: {intf.bInterfaceNumber}")
            print(f"  Num endpoints: {intf.bNumEndpoints}")

            for ep in intf:
                ep_addr = ep.bEndpointAddress
                ep_dir = "IN" if ep_addr & 0x80 else "OUT"
                ep_type = ["Control", "Isochronous", "Bulk", "Interrupt"][ep.bmAttributes & 0x03]
                print(f"    EP 0x{ep_addr:02X}: {ep_dir} {ep_type} maxPacket={ep.wMaxPacketSize}")

            print("  PASSED")
            return True

        except Exception as e:
            print(f"  FAILED: {e}")
            return False

    def cleanup(self):
        """Clean up USB connection."""
        if self.dev:
            usb.util.dispose_resources(self.dev)
            print("Cleaned up USB resources")


def main():
    """Run USB device tests."""
    print("=" * 60)
    print("ASM2464 USB Device Tests (Against Real Emulated Device)")
    print("=" * 60)

    if not HAVE_PYUSB:
        print("\nERROR: pyusb required. Install with: pip install pyusb")
        return 1

    tester = ASM2464USBTest()

    # Find device
    if not tester.find_device():
        print("\nDevice not found. Make sure emulate/usb_device.py is running.")
        return 1

    # Setup
    if not tester.setup():
        print("\nFailed to set up USB connection. Try running with sudo.")
        return 1

    # Run tests
    results = []
    results.append(("GET_DESCRIPTOR (Device)", tester.test_get_device_descriptor()))
    results.append(("GET_DESCRIPTOR (Config)", tester.test_get_config_descriptor()))
    results.append(("GET_DESCRIPTOR (Strings)", tester.test_get_string_descriptor()))
    results.append(("E4 Read", tester.test_e4_read()))
    results.append(("E5 Write", tester.test_e5_write()))
    results.append(("Bulk Endpoints", tester.test_bulk_endpoints()))

    # Summary
    print("\n" + "=" * 60)
    print("Test Results Summary")
    print("=" * 60)
    passed = 0
    failed = 0
    for name, result in results:
        status = "PASSED" if result else "FAILED"
        print(f"  {name}: {status}")
        if result:
            passed += 1
        else:
            failed += 1

    print(f"\nTotal: {passed} passed, {failed} failed")

    tester.cleanup()

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
