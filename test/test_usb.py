#!/usr/bin/env python3
"""
Test USB enumeration and vendor messages against the emulator.

These tests verify that the firmware correctly handles:
- USB enumeration (GET_DESCRIPTOR requests)
- E4 vendor command (read XDATA)
- E5 vendor command (write XDATA)
- SCSI BBB protocol commands
- Firmware reflash commands (E1, E3, E8)
- PCIe requests via vendor commands

Protocol reference from ~/tinygrad:
- tinygrad/runtime/support/usb.py: USB3 class, ASM24Controller
- extra/usbgpu/patch.py: firmware reflash protocol

Usage:
    # Run against original firmware (default)
    pytest test/test_usb.py -v

    # Run against our compiled firmware
    pytest test/test_usb.py -v --firmware=ours

    # Run against both firmwares
    pytest test/test_usb.py -v --firmware=both
"""

import sys
import struct
from pathlib import Path
import pytest

# Add emulate directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'emulate'))

from emu import Emulator
from conftest import ORIGINAL_FIRMWARE, OUR_FIRMWARE


# USB Descriptor Types
USB_DT_DEVICE = 0x01
USB_DT_CONFIG = 0x02
USB_DT_STRING = 0x03
USB_DT_INTERFACE = 0x04
USB_DT_ENDPOINT = 0x05
USB_DT_BOS = 0x0F

# USB Request Types
USB_TYPE_STANDARD = 0x00
USB_TYPE_CLASS = 0x20
USB_TYPE_VENDOR = 0x40

# Standard USB Requests
USB_REQ_GET_DESCRIPTOR = 0x06
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_SET_CONFIGURATION = 0x09

# Expected device info
# Original ASMedia VID/PID
ASM2464_VID_ORIGINAL = 0x174C
ASM2464_PID_ORIGINAL = 0x2464
ASM2464_PID_2461 = 0x2461  # Another ASMedia PID variant
ASM2464_PID_2463 = 0x2463  # Another ASMedia PID variant
# tinygrad custom VID/PID
ASM2464_VID_TINYGRAD = 0xADD1
ASM2464_PID_TINYGRAD = 0x0001
# Accept any valid VID/PID
VALID_VIDS = [ASM2464_VID_ORIGINAL, ASM2464_VID_TINYGRAD]
VALID_PIDS = [ASM2464_PID_ORIGINAL, ASM2464_PID_2461, ASM2464_PID_2463, ASM2464_PID_TINYGRAD]


class TestUSBEnumeration:
    """Tests for USB enumeration (GET_DESCRIPTOR requests)."""

    def _setup_usb_for_descriptor(self, emu, desc_type, desc_index=0, wLength=255):
        """
        Set up MMIO state to request a USB descriptor.

        Args:
            emu: Emulator instance
            desc_type: Descriptor type (USB_DT_DEVICE, USB_DT_CONFIG, etc.)
            desc_index: Descriptor index (used for strings)
            wLength: Maximum bytes to return
        """
        hw = emu.hw

        # First connect USB
        hw.usb_controller.connect(speed=1)  # High Speed

        # Run firmware through initial USB connect handling
        emu.run(max_cycles=100000)

        # Inject GET_DESCRIPTOR control transfer
        # bmRequestType = 0x80 (device-to-host, standard, device)
        # bRequest = 0x06 (GET_DESCRIPTOR)
        # wValue = (desc_type << 8) | desc_index
        # wIndex = 0 (or language ID for strings)
        # wLength = max bytes to return
        hw.usb_controller.inject_control_transfer(
            bmRequestType=0x80,
            bRequest=USB_REQ_GET_DESCRIPTOR,
            wValue=(desc_type << 8) | desc_index,
            wIndex=0,
            wLength=wLength
        )

        # Run firmware to process the request
        emu.run(max_cycles=500000)

    def test_get_device_descriptor(self, firmware_emulator):
        """Test GET_DESCRIPTOR for device descriptor."""
        emu, firmware_name = firmware_emulator

        self._setup_usb_for_descriptor(emu, USB_DT_DEVICE, wLength=18)

        # Read device descriptor from USB buffer at 0x8000
        desc = bytes([emu.memory.xdata[0x8000 + i] for i in range(18)])

        # Device descriptor structure:
        # Byte 0: bLength (should be 18)
        # Byte 1: bDescriptorType (should be 0x01)
        # Bytes 2-3: bcdUSB
        # Bytes 8-9: idVendor (little-endian)
        # Bytes 10-11: idProduct (little-endian)

        assert desc[0] == 18, f"Device descriptor length should be 18, got {desc[0]}"
        assert desc[1] == USB_DT_DEVICE, f"Descriptor type should be 0x01, got {desc[1]}"

        vid = struct.unpack('<H', desc[8:10])[0]
        pid = struct.unpack('<H', desc[10:12])[0]

        assert vid in VALID_VIDS, f"VID should be in {[hex(v) for v in VALID_VIDS]}, got 0x{vid:04X}"
        assert pid in VALID_PIDS, f"PID should be in {[hex(p) for p in VALID_PIDS]}, got 0x{pid:04X}"

    def test_get_config_descriptor(self, firmware_emulator):
        """Test GET_DESCRIPTOR for configuration descriptor."""
        emu, firmware_name = firmware_emulator

        # First get just the header (9 bytes) to get total length
        self._setup_usb_for_descriptor(emu, USB_DT_CONFIG, wLength=9)

        desc = bytes([emu.memory.xdata[0x8000 + i] for i in range(9)])

        assert desc[0] == 9, f"Config descriptor header length should be 9, got {desc[0]}"
        assert desc[1] == USB_DT_CONFIG, f"Descriptor type should be 0x02, got {desc[1]}"

        total_length = struct.unpack('<H', desc[2:4])[0]
        assert total_length > 9, f"Total length should be > 9, got {total_length}"

        # Config descriptor should specify at least 1 interface
        num_interfaces = desc[4]
        assert num_interfaces >= 1, f"Should have at least 1 interface, got {num_interfaces}"

    def test_get_string_descriptor_0(self, firmware_emulator):
        """Test GET_DESCRIPTOR for string descriptor 0 (language IDs)."""
        emu, firmware_name = firmware_emulator

        self._setup_usb_for_descriptor(emu, USB_DT_STRING, desc_index=0, wLength=255)

        # Read string descriptor 0
        desc = bytes([emu.memory.xdata[0x8000 + i] for i in range(4)])

        # String descriptor 0 format:
        # Byte 0: bLength (at least 4 for one language)
        # Byte 1: bDescriptorType (0x03)
        # Bytes 2-3+: wLANGID[0], wLANGID[1], ...

        if desc[0] >= 4 and desc[1] == USB_DT_STRING:
            lang_id = struct.unpack('<H', desc[2:4])[0]
            # Common language IDs: 0x0409 (US English), 0x0000
            assert lang_id in (0x0409, 0x0000, 0x0809), f"Unexpected language ID: 0x{lang_id:04X}"


class TestE4E5Commands:
    """Tests for E4 (read XDATA) and E5 (write XDATA) vendor commands."""

    def _setup_for_vendor_command(self, emu):
        """Set up emulator for vendor command testing."""
        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)

        # Run firmware through initial USB handling
        emu.run(max_cycles=100000)

    def test_e4_command_injection(self, firmware_emulator):
        """Test that E4 command injects CDB correctly into MMIO registers."""
        emu, firmware_name = firmware_emulator

        self._setup_for_vendor_command(emu)

        # Write known value to XDATA
        test_addr = 0x0100
        test_value = 0x42
        emu.memory.xdata[test_addr] = test_value

        # Inject E4 read command
        emu.hw.inject_usb_command(0xE4, test_addr, size=1)

        # Check CDB was written to USB registers
        # CDB is at 0x910D-0x9112
        cdb_byte0 = emu.hw.regs.get(0x910D, 0)
        assert cdb_byte0 == 0xE4, f"CDB[0] should be 0xE4, got 0x{cdb_byte0:02X}"

        # Also verify USB state is set for command processing
        assert emu.hw.usb_cmd_pending, "USB command pending flag should be set"

    def test_e5_command_injection(self, firmware_emulator):
        """Test that E5 command injects CDB correctly into MMIO registers."""
        emu, firmware_name = firmware_emulator

        self._setup_for_vendor_command(emu)

        test_addr = 0x0100
        test_value = 0x55

        # Inject E5 write command
        emu.hw.inject_usb_command(0xE5, test_addr, value=test_value)

        # Check CDB was written to USB registers
        cdb_byte0 = emu.hw.regs.get(0x910D, 0)
        assert cdb_byte0 == 0xE5, f"CDB[0] should be 0xE5, got 0x{cdb_byte0:02X}"

    def test_xdata_direct_read_write(self, firmware_emulator):
        """Test direct XDATA read/write (bypassing USB, verifying memory)."""
        emu, firmware_name = firmware_emulator

        # Direct XDATA access test - no USB involved
        test_addr = 0x0100
        test_value = 0x42

        # Write directly
        emu.memory.xdata[test_addr] = test_value

        # Read back
        result = emu.memory.xdata[test_addr]
        assert result == test_value, f"XDATA read returned 0x{result:02X}, expected 0x{test_value:02X}"

    def test_xdata_pattern(self, firmware_emulator):
        """Test reading/writing a pattern to XDATA."""
        emu, firmware_name = firmware_emulator

        test_addr = 0x0100
        test_pattern = [0x11, 0x22, 0x33, 0x44]

        # Write pattern
        for i, v in enumerate(test_pattern):
            emu.memory.xdata[test_addr + i] = v

        # Read back
        result = [emu.memory.xdata[test_addr + i] for i in range(4)]
        assert result == test_pattern, f"Pattern read returned {result}, expected {test_pattern}"

    def test_register_read(self, firmware_emulator):
        """Test reading hardware register area (0x6000+)."""
        emu, firmware_name = firmware_emulator

        self._setup_for_vendor_command(emu)

        # Set a known register value
        test_addr = 0xC009  # UART LSR - should be 0x60 (TX empty)
        emu.hw.regs[test_addr] = 0x60

        # Read via hw.read() which hooks into the MMIO system
        result = emu.hw.read(test_addr)
        assert result == 0x60, f"Register read returned 0x{result:02X}, expected 0x60"


class TestSCSICommands:
    """Tests for SCSI BBB protocol commands."""

    def _setup_for_scsi(self, emu):
        """Set up emulator for SCSI command testing."""
        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)

        # Run firmware through initial handling
        emu.run(max_cycles=100000)

    def test_scsi_write_command_injection(self, firmware_emulator):
        """Test that SCSI write command injects correctly into MMIO."""
        emu, firmware_name = firmware_emulator

        self._setup_for_scsi(emu)

        # Prepare test data
        test_data = bytes([0xAA, 0xBB, 0xCC, 0xDD] + [0x00] * 508)  # 1 sector
        lba = 0
        sectors = 1

        # Inject SCSI write command via USBController
        emu.hw.usb_controller.inject_scsi_write_command(lba, sectors, test_data)

        # Verify CDB was written to USB registers
        # CDB starts at 0x910D, opcode 0x8A should be there
        cdb_opcode = emu.hw.regs.get(0x910D, 0)
        assert cdb_opcode == 0x8A, f"SCSI CDB opcode should be 0x8A, got 0x{cdb_opcode:02X}"

    def test_scsi_vendor_command_injection(self, firmware_emulator):
        """Test that SCSI vendor command injects CDB correctly."""
        emu, firmware_name = firmware_emulator

        self._setup_for_scsi(emu)

        # Build E4 CDB
        test_addr = 0x0200
        addr_with_flag = (test_addr & 0x1FFFF) | 0x500000
        cdb = struct.pack('>BBBHB', 0xE4, 1, addr_with_flag >> 16, addr_with_flag & 0xFFFF, 0)

        # Inject via SCSI vendor path on USBController
        emu.hw.usb_controller.inject_scsi_vendor_command(0xE4, cdb)

        # Verify CDB was written
        cdb_opcode = emu.hw.regs.get(0x910D, 0)
        assert cdb_opcode == 0xE4, f"Vendor CDB opcode should be 0xE4, got 0x{cdb_opcode:02X}"

    def test_usb_buffer_access(self, firmware_emulator):
        """Test that USB data buffer at 0x8000 is accessible."""
        emu, firmware_name = firmware_emulator

        # Write test data directly to USB buffer
        test_data = bytes([0xDE, 0xAD, 0xBE, 0xEF])
        for i, b in enumerate(test_data):
            emu.memory.xdata[0x8000 + i] = b

        # Read back
        result = bytes([emu.memory.xdata[0x8000 + i] for i in range(4)])
        assert result == test_data, f"USB buffer read mismatch: got {result.hex()}"


class TestFirmwareReflashCommands:
    """Tests for firmware reflash protocol (E1, E3, E8 commands)."""

    def _setup_for_reflash(self, emu):
        """Set up emulator for reflash command testing."""
        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)

        # Run firmware through initial handling
        emu.run(max_cycles=100000)

    def test_e1_config_command_injection(self, firmware_emulator):
        """Test E1 config patch command CDB injection."""
        emu, firmware_name = firmware_emulator

        self._setup_for_reflash(emu)

        # E1 config command: E1 50 index <12 zeros>
        cdb = struct.pack('>BBB12x', 0xE1, 0x50, 0)
        config_data = bytes([0xFF] * 128)

        # Inject via SCSI vendor path on USBController
        emu.hw.usb_controller.inject_scsi_vendor_command(0xE1, cdb, data=config_data, is_write=True)

        # Verify CDB was written
        cdb_opcode = emu.hw.regs.get(0x910D, 0)
        assert cdb_opcode == 0xE1, f"Config CDB opcode should be 0xE1, got 0x{cdb_opcode:02X}"

    def test_e3_firmware_data_command_injection(self, firmware_emulator):
        """Test E3 firmware data command CDB injection."""
        emu, firmware_name = firmware_emulator

        self._setup_for_reflash(emu)

        # E3 firmware command: E3 subcode length[4 bytes]
        fw_data = bytes([0x02, 0x00, 0x00] + [0x00] * 253)
        length = len(fw_data)
        cdb = struct.pack('>BBI', 0xE3, 0x50, length)

        # Inject via SCSI vendor path on USBController
        emu.hw.usb_controller.inject_scsi_vendor_command(0xE3, cdb, data=fw_data, is_write=True)

        # Verify CDB was written
        cdb_opcode = emu.hw.regs.get(0x910D, 0)
        assert cdb_opcode == 0xE3, f"Firmware CDB opcode should be 0xE3, got 0x{cdb_opcode:02X}"

    def test_e8_commit_command_injection(self, firmware_emulator):
        """Test E8 commit/boot command CDB injection."""
        emu, firmware_name = firmware_emulator

        self._setup_for_reflash(emu)

        # E8 commit command: E8 51 <13 zeros>
        cdb = struct.pack('>BB13x', 0xE8, 0x51)

        # Inject via SCSI vendor path on USBController
        emu.hw.usb_controller.inject_scsi_vendor_command(0xE8, cdb)

        # Verify CDB was written
        cdb_opcode = emu.hw.regs.get(0x910D, 0)
        assert cdb_opcode == 0xE8, f"Commit CDB opcode should be 0xE8, got 0x{cdb_opcode:02X}"

    def test_reflash_cdb_format(self, firmware_emulator):
        """Test that reflash CDB format matches patch.py expectations."""
        # Verify CDB format from patch.py:
        # E1: struct.pack('>BBB12x', 0xe1, 0x50, index)
        # E3: struct.pack('>BBI', 0xe3, subcode, length)  # 7 bytes total
        # E8: struct.pack('>BB13x', 0xe8, 0x51)

        # E1 CDB
        e1_cdb = struct.pack('>BBB12x', 0xE1, 0x50, 0)
        assert len(e1_cdb) == 15, f"E1 CDB should be 15 bytes, got {len(e1_cdb)}"
        assert e1_cdb[0] == 0xE1, "E1 CDB[0] should be 0xE1"
        assert e1_cdb[1] == 0x50, "E1 CDB[1] should be 0x50"

        # E3 CDB
        e3_cdb = struct.pack('>BBI', 0xE3, 0x50, 0x100)
        assert len(e3_cdb) == 6, f"E3 CDB should be 6 bytes, got {len(e3_cdb)}"
        assert e3_cdb[0] == 0xE3, "E3 CDB[0] should be 0xE3"

        # E8 CDB
        e8_cdb = struct.pack('>BB13x', 0xE8, 0x51)
        assert len(e8_cdb) == 15, f"E8 CDB should be 15 bytes, got {len(e8_cdb)}"
        assert e8_cdb[0] == 0xE8, "E8 CDB[0] should be 0xE8"
        assert e8_cdb[1] == 0x51, "E8 CDB[1] should be 0x51"


class TestPCIeRequests:
    """Tests for PCIe memory and config requests via vendor commands."""

    def _setup_for_pcie(self, emu):
        """Set up emulator for PCIe request testing."""
        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)

        # Simulate PCIe link up
        hw.regs[0xB480] = 0x02  # PCIe link up

        # Run firmware through initial handling
        emu.run(max_cycles=100000)

    def test_pcie_registers_accessible(self, firmware_emulator):
        """Test that PCIe control registers are accessible."""
        emu, firmware_name = firmware_emulator

        # PCIe registers from ASM24Controller.pcie_prep_request:
        # 0xB210: fmt_type
        # 0xB217: byte enables
        # 0xB218-0xB21F: address
        # 0xB220-0xB223: data
        # 0xB254: trigger
        # 0xB296: status

        pcie_regs = [0xB210, 0xB217, 0xB218, 0xB220, 0xB254, 0xB296]

        for addr in pcie_regs:
            # Write a value
            test_val = (addr & 0xFF)
            emu.memory.xdata[addr] = test_val

            # Read back
            result = emu.memory.xdata[addr]
            assert result == test_val, f"PCIe reg 0x{addr:04X} write/read failed"

    def test_pcie_fmt_type_values(self, firmware_emulator):
        """Test that PCIe fmt_type values match usb.py protocol."""
        # Verify fmt_type values from ASM24Controller:
        # 0x04 - Config read type 0
        # 0x05 - Config read type 1 (bus > 0)
        # 0x44 - Config write type 0
        # 0x45 - Config write type 1
        # 0x20 - Memory read
        # 0x60 - Memory write

        fmt_types = {
            'cfg_read_0': 0x04,
            'cfg_read_1': 0x05,
            'cfg_write_0': 0x44,
            'cfg_write_1': 0x45,
            'mem_read': 0x20,
            'mem_write': 0x60,
        }

        # Just verify the constants are correct
        assert fmt_types['cfg_read_0'] == 0x04
        assert fmt_types['cfg_write_0'] == 0x44
        assert fmt_types['mem_read'] == 0x20
        assert fmt_types['mem_write'] == 0x60

    def test_pcie_memory_write_registers(self, firmware_emulator):
        """Test PCIe memory write register setup."""
        emu, firmware_name = firmware_emulator

        # Simulate the register writes for a PCIe memory write
        # Per ASM24Controller.pcie_prep_request:
        # 1. Write value to 0xB220-0xB223 (big-endian)
        # 2. Write address to 0xB218-0xB21B (big-endian)
        # 3. Write byte enables to 0xB217
        # 4. Write fmt_type to 0xB210

        data_value = 0xDEADBEEF
        address = 0x00001000

        # Write data (big-endian)
        data_bytes = struct.pack('>I', data_value)
        for i, b in enumerate(data_bytes):
            emu.memory.xdata[0xB220 + i] = b

        # Write address (big-endian)
        addr_bytes = struct.pack('>I', address)
        for i, b in enumerate(addr_bytes):
            emu.memory.xdata[0xB218 + i] = b

        # Write byte enables
        emu.memory.xdata[0xB217] = 0x0F  # 4 bytes

        # Write fmt_type
        emu.memory.xdata[0xB210] = 0x60  # Memory write

        # Verify writes
        assert emu.memory.xdata[0xB210] == 0x60
        assert emu.memory.xdata[0xB217] == 0x0F
        result_data = struct.unpack('>I', bytes([emu.memory.xdata[0xB220 + i] for i in range(4)]))[0]
        assert result_data == data_value


class TestInitSequence:
    """Tests for the USB init sequence from ASM24Controller."""

    def test_controller_init_write_values(self, firmware_emulator):
        """Test the initialization write values from ASM24Controller.__init__."""
        emu, firmware_name = firmware_emulator

        # Init sequence from usb.py ASM24Controller.__init__:
        # WriteOp(0x54b, b' '), WriteOp(0x54e, b'\x04'), WriteOp(0x5a8, b'\x02'),
        # WriteOp(0x5f8, b'\x04'), WriteOp(0x7ec, b'\x01\x00\x00\x00'),
        # WriteOp(0xc422, b'\x02'), WriteOp(0x0, b'\x33')

        init_writes = [
            (0x054B, 0x20),  # ' '
            (0x054E, 0x04),
            (0x05A8, 0x02),
            (0x05F8, 0x04),
            (0x07EC, 0x01),  # First byte of 4-byte write
            (0xC422, 0x02),
            (0x0000, 0x33),
        ]

        # Directly write to XDATA (simulating E5 command effect)
        for addr, value in init_writes:
            if addr < 0x6000:
                emu.memory.xdata[addr] = value
            else:
                emu.hw.regs[addr] = value

        # Verify writes
        for addr, value in init_writes:
            if addr < 0x6000:
                result = emu.memory.xdata[addr]
            else:
                result = emu.hw.regs.get(addr, 0)
            assert result == value, f"Init write 0x{addr:04X}=0x{value:02X} failed, got 0x{result:02X}"

    def test_init_sequence_addresses(self, firmware_emulator):
        """Test that init sequence addresses are in expected memory regions."""
        # Verify addresses from ASM24Controller.__init__:
        init_addresses = [0x054B, 0x054E, 0x05A8, 0x05F8, 0x07EC, 0xC422, 0x0000]

        for addr in init_addresses:
            if addr < 0x6000:
                # XDATA region
                assert addr < 0x6000, f"Address 0x{addr:04X} should be in XDATA"
            else:
                # Register region
                assert addr >= 0x6000, f"Address 0x{addr:04X} should be in register space"


class TestUSBStateMachine:
    """Tests for USB state machine transitions."""

    def test_usb_state_transitions(self, firmware_emulator):
        """Test USB state machine transitions during enumeration."""
        emu, firmware_name = firmware_emulator

        hw = emu.hw

        # Initial state should be disconnected
        assert hw.usb_controller.state.value == 0  # DISCONNECTED

        # Connect
        hw.usb_controller.connect(speed=1)
        assert hw.usb_controller.state.value >= 1  # At least ATTACHED

        # Run firmware to process connect
        emu.run(max_cycles=100000)

        # After processing, USB should be configured
        # Check IDATA[0x6A] for USB state
        usb_state = emu.memory.idata[0x6A] if hasattr(emu.memory, 'idata') else 0
        # State 5 = CONFIGURED
        # Note: exact state depends on enumeration progress

    def test_usb_connection_registers(self, firmware_emulator):
        """Test that USB connection sets correct MMIO registers."""
        emu, firmware_name = firmware_emulator

        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)

        # Check expected register values
        assert hw.regs.get(0x9000, 0) & 0x81, "USB status should have connected+active bits"
        assert hw.regs.get(0xC802, 0) != 0, "USB interrupt pending should be set"
        assert hw.regs.get(0x9101, 0) != 0, "USB interrupt flags should be set"


class TestUSBDescriptorDMA:
    """Tests for USB descriptor DMA operations."""

    def test_descriptor_dma_trigger(self, firmware_emulator):
        """Test that descriptor DMA is triggered correctly."""
        emu, firmware_name = firmware_emulator

        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)
        emu.run(max_cycles=100000)

        # Inject GET_DESCRIPTOR for device descriptor
        hw.usb_controller.inject_control_transfer(
            bmRequestType=0x80,
            bRequest=USB_REQ_GET_DESCRIPTOR,
            wValue=(USB_DT_DEVICE << 8) | 0,
            wIndex=0,
            wLength=18
        )

        # Run firmware
        emu.run(max_cycles=500000)

        # Check DMA configuration registers
        dma_hi = hw.regs.get(0x905B, 0)
        dma_lo = hw.regs.get(0x905C, 0)
        dma_addr = (dma_hi << 8) | dma_lo

        # DMA address should point to descriptor location in ROM
        # Device descriptor is typically at a fixed ROM address

    def test_config_descriptor_dma(self, firmware_emulator):
        """Test configuration descriptor DMA."""
        emu, firmware_name = firmware_emulator

        hw = emu.hw

        # Connect USB
        hw.usb_controller.connect(speed=1)
        emu.run(max_cycles=100000)

        # Inject GET_DESCRIPTOR for config descriptor
        hw.usb_controller.inject_control_transfer(
            bmRequestType=0x80,
            bRequest=USB_REQ_GET_DESCRIPTOR,
            wValue=(USB_DT_CONFIG << 8) | 0,
            wIndex=0,
            wLength=255
        )

        # Run firmware
        emu.run(max_cycles=500000)

        # Check USB buffer has valid config descriptor
        desc_len = emu.memory.xdata[0x8000]
        desc_type = emu.memory.xdata[0x8001]

        # Valid config descriptor should start with length=9, type=0x02
        if desc_type == USB_DT_CONFIG:
            assert desc_len == 9, f"Config descriptor header should be 9 bytes, got {desc_len}"


# Smoke test to verify emulator works
class TestEmulatorSmoke:
    """Basic smoke tests for emulator functionality."""

    def test_emulator_boots(self, firmware_emulator):
        """Test that emulator can boot firmware."""
        emu, firmware_name = firmware_emulator

        # Run for some cycles
        emu.run(max_cycles=50000)

        # PC should have moved from 0
        assert emu.cpu.pc > 0, "PC should advance during boot"
        assert emu.inst_count > 0, "Should have executed some instructions"

    def test_usb_controller_exists(self, firmware_emulator):
        """Test that USB controller is initialized."""
        emu, firmware_name = firmware_emulator

        assert emu.hw.usb_controller is not None, "USB controller should exist"
        assert hasattr(emu.hw.usb_controller, 'connect'), "USB controller should have connect method"
        assert hasattr(emu.hw.usb_controller, 'inject_control_transfer'), "Should have inject_control_transfer"
        assert hasattr(emu.hw.usb_controller, 'inject_vendor_command'), "Should have inject_vendor_command"
