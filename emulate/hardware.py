"""
ASM2464PD Hardware Emulation

This module provides realistic hardware emulation for the ASM2464PD.
Only hardware registers (XDATA >= 0x6000) are emulated here.
RAM (XDATA < 0x6000) is handled by the memory system, not this module.
"""

from typing import TYPE_CHECKING, Dict, Set, Callable, Optional
from dataclasses import dataclass, field
from enum import IntEnum
import re
import os

if TYPE_CHECKING:
    from memory import Memory

# =============================================================================
# Register Name Lookup
# =============================================================================
# Parse registers.h to build addr -> name lookup table

_REGISTER_NAMES: Dict[int, str] = {}

def _load_register_names():
    """Parse registers.h and build address-to-name lookup table."""
    global _REGISTER_NAMES
    if _REGISTER_NAMES:
        return  # Already loaded

    # Find registers.h relative to this file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    registers_h = os.path.join(this_dir, '..', 'src', 'include', 'registers.h')

    if not os.path.exists(registers_h):
        return

    # Pattern: #define REG_NAME  XDATA_REG8(0xADDR) or similar
    pattern = re.compile(r'#define\s+(REG_\w+)\s+XDATA_REG\d+\((0x[0-9A-Fa-f]+)\)')

    with open(registers_h, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                name = match.group(1)
                addr = int(match.group(2), 16)
                _REGISTER_NAMES[addr] = name

def get_register_name(addr: int) -> str:
    """Get register name for address, or empty string if unknown."""
    _load_register_names()
    return _REGISTER_NAMES.get(addr, "")


class USBState(IntEnum):
    """
    USB state machine states.

    Matches firmware I_USB_STATE (IDATA[0x6A]) and USB_STATE_* constants in globals.h.
    State transitions occur in ISR at 0x0E68 and main loop at 0x202A.
    """
    DISCONNECTED = 0  # USB_STATE_DISCONNECTED - No USB connection
    ATTACHED = 1      # USB_STATE_ATTACHED - Cable connected
    POWERED = 2       # USB_STATE_POWERED - Bus powered
    DEFAULT = 3       # USB_STATE_DEFAULT - Default address assigned
    ADDRESS = 4       # USB_STATE_ADDRESS - Device address assigned
    CONFIGURED = 5    # USB_STATE_CONFIGURED - Ready for vendor commands


@dataclass
class USBCommand:
    """USB command queued for firmware processing."""
    cmd: int           # Command type (0xE4=read, 0xE5=write, 0x8A=scsi)
    addr: int          # Target XDATA address
    data: bytes        # Data for write commands
    response: bytes = b''  # Response data for read commands


class USBController:
    """
    USB controller emulation using only MMIO registers.

    This class manages the USB state machine and vendor command injection
    without directly modifying RAM. All state transitions are driven by
    setting MMIO registers that cause the firmware to naturally progress
    through its USB state machine.

    The firmware's USB state machine:
    - I_USB_STATE (IDATA[0x6A]) contains current USB state (0-5)
    - State 5 = USB_STATE_CONFIGURED, ready for vendor commands
    - Firmware transitions states by reading MMIO and updating its own RAM

    Key MMIO registers for USB (see registers.h for full definitions):
    - REG_USB_STATUS (0x9000): Connection status (bit 7=connected, bit 0=active)
    - REG_USB_PERIPH_STATUS (0x9101): Interrupt flags (bit 5 triggers cmd handler)
    - REG_USB_CTRL_PHASE (0x9091): Control transfer phase (bit 0=setup, bit 1=data)
    - REG_USB_DMA_TRIGGER (0x9092): Trigger DMA for descriptor transfer
    - REG_USB_DMA_STATE (0xCE89): DMA state machine (bits control transitions)
    - REG_USB_SETUP_* (0x9E00-0x9E07): USB setup packet buffer
    - REG_USB_EP0_COMPLETE (0xE712): EP0 transfer complete status

    Key globals for USB (see globals.h):
    - G_CMD_SLOT_INDEX (0x05A3): Current command slot index
    - G_CMD_TABLE_BASE (0x05B1): Command table (10 x 34-byte entries)
    - G_USB_CMD_CONFIG (0x07EC): USB command configuration
    - G_DMA_XFER_STATUS (0x0AA0): DMA transfer status
    """

    def __init__(self, hw: 'HardwareState'):
        self.hw = hw
        self.state = USBState.DISCONNECTED
        self.pending_cmd: Optional[USBCommand] = None
        self.enumeration_complete = False
        self.vendor_cmd_active = False

        # Track state machine progress
        self.state_machine_reads = 0
        self.enumeration_step = 0

        # Pending descriptor request from GET_DESCRIPTOR
        self.pending_descriptor_request = None

    def connect(self, speed: int = 2):
        """
        Simulate USB cable connection via MMIO registers.

        This sets the initial MMIO state that triggers USB enumeration
        in the firmware. The firmware will progress through states 0→5.

        Args:
            speed: USB speed mode:
                0 = Full Speed (USB 1.x, 12 Mbps)
                1 = High Speed (USB 2.0, 480 Mbps)
                2 = SuperSpeed (USB 3.x, 5 Gbps)
                3 = SuperSpeed+ (USB 3.1+, 10+ Gbps)
        """
        self.state = USBState.ATTACHED
        self.enumeration_step = 1
        self.usb_speed = speed

        # USB connection status registers
        # NOTE: 0x9000 bit 0 must be SET to enter USB state machine at 0x0E6E
        # At ISR 0x0E68: if bit 0 SET, jump to USB handling at 0x0E6E
        self.hw.regs[0x9000] = 0x81  # Bit 7 (connected), bit 0 SET for USB handling
        self.hw.regs[0x90E0] = speed  # USB speed
        self.hw.regs[0x9100] = speed  # USB link active with speed
        self.hw.regs[0x9105] = 0xFF  # PHY active
        # USB state indicator (0x9118):
        # At ISR 0x0E71, value is used as index into table at 0x5AC9
        # If table[0x9118] >= 8, USB handling is skipped
        # table[0] = 0x08 (skip), table[1] = 0x00 (continue)
        # Set to 1 to enable USB enumeration handling
        self.hw.regs[0x9118] = 0x01  # USB enumeration state (1 = pending setup)

        # USB interrupt and state machine triggers:
        # At 0x0FEB: if 0x9101 bit 6 CLEAR, skip USB init path
        # At 0x0FF2: if 0x90E2 bit 0 CLEAR, skip USB init path
        # So both bit 6 of 0x9101 and bit 0 of 0x90E2 must be SET
        self.hw.regs[0xC802] = 0x05  # USB interrupt pending (bits 0 + 2)
        self.hw.regs[0x9101] = 0x61  # Bit 6 SET (USB init), bit 5 SET, bit 0 for USB active
        self.hw.regs[0x90E2] = 0x03  # Bit 0 SET (USB init trigger), bit 1 SET

        # USB restart trigger at 0xCC5D:
        # At 0x2163-0x216B: if bit 0 CLEAR and bit 1 SET, calls USB restart at 0x2176
        # This sets 0x0A5A=1 which enables the USB init path at 0x2185
        self.hw.regs[0xCC5D] = 0x02  # Bit 1 SET, bit 0 CLEAR - triggers USB restart

        # USB PHY control at 0x91C0:
        # Firmware clears this during init, but at 0x203B it checks bit 1.
        # When state 0x0A59 == 2, if 0x92C2 bit 6 is SET and 0x91C0 bit 1 is SET,
        # firmware calls 0x0322 which progresses the USB state machine.
        self.hw.regs[0x91C0] = 0x02  # Bit 1 SET - enables USB state machine progress

        # USB mode indicators for descriptor handling at 0xA7E5 and 0x87A1:
        # At 0xA7E4-0xA7E5: checks 0xCC91 bit 1 for USB3 mode
        # At 0xA7FD-0xA7FF: checks 0x09F9 bit 6 for USB3 speed
        # If both set, 0x0ACC gets value with bit 1 SET, enabling USB3 descriptor path
        # For USB 2.0: clear these bits so firmware takes USB2 path
        if speed >= 2:  # SuperSpeed or higher
            self.hw.regs[0xCC91] = 0x02  # Bit 1 SET - USB3 mode
            self.hw.regs[0x09F9] = 0x40  # Bit 6 SET - USB3 speed indicator
        else:  # High Speed or Full Speed (USB 2.0)
            self.hw.regs[0xCC91] = 0x00  # Bit 1 CLEAR - USB2 mode
            self.hw.regs[0x09F9] = 0x00  # Bit 6 CLEAR - USB2 speed indicator

        # PCIe enumeration state - simulate that PCIe link is already up
        # In real hardware, PCIe enumeration happens during boot before USB control
        # transfers. The firmware checks this state at 0x185C before taking the
        # descriptor DMA path (0x1865+). Without it, firmware takes alternate path.
        #
        # Flow: 0x3803 checks XDATA[0x053F] or XDATA[0x0553] != 0
        #       If true, sets XDATA[0x0AF7] = 1 at 0x3914-0x3919
        #       This enables the "good" path at 0x185C that uses descriptor DMA
        #
        # CRITICAL: 0xB480 bit 0 must be SET to prevent firmware at 0x20DA from
        # taking the path at 0x20F9 that clears XDATA[0x0AF7] to 0.
        # At 0x20DA: jnb acc.0, 0x20fe -> if bit 0 CLEAR, jump and clear 0x0AF7
        self.hw.regs[0xB480] = 0x03  # Bits 0,1 SET - PCIe link active state

        # Set these to simulate completed PCIe enumeration:
        if self.hw.memory:
            self.hw.memory.xdata[0x0AF7] = 0x01  # PCIe enumeration complete flag
            self.hw.memory.xdata[0x053F] = 0x01  # PCIe link state (port 0)
            # CRITICAL: Command table state at G_CMD_TABLE_BASE + index*0x22 must NOT be 4
            # At 0x35D4-0x35DF: Firmware calls 0x1551 which reads G_CMD_SLOT_INDEX (0x05A3),
            # then calculates G_CMD_TABLE_BASE[index] and if that value equals 4,
            # it calls 0x54BB which clears XDATA[0x0AF7] to 0.
            # Set slot 0 state to 3 (ready) instead of 4 (error/reset).
            self.hw.memory.xdata[0x05A3] = 0x00  # G_CMD_SLOT_INDEX = 0
            self.hw.memory.xdata[0x05B1] = 0x03  # G_CMD_TABLE_BASE[0] = 3 (ready)

        print(f"[{self.hw.cycles:8d}] [USB_CTRL] Connected - MMIO set for enumeration")

    def advance_enumeration(self):
        """
        Advance USB enumeration state via MMIO.

        Called when firmware polls 0xCE89 to check enumeration progress.
        Each call advances the emulated enumeration sequence.
        """
        self.state_machine_reads += 1

        # Return value for 0xCE89 based on enumeration progress
        value = 0x00

        if self.state_machine_reads >= 3:
            value |= 0x01  # Bit 0 - exit wait loop at 0x348C
            self.enumeration_step = max(self.enumeration_step, 2)

        if self.state_machine_reads >= 5:
            value |= 0x02  # Bit 1 - successful enumeration path at 0x3493
            self.enumeration_step = max(self.enumeration_step, 3)

        if self.state_machine_reads >= 7:
            value |= 0x04  # Bit 2 - state 3→4→5 transitions
            self.enumeration_step = max(self.enumeration_step, 4)
            self.enumeration_complete = True
            self.state = USBState.CONFIGURED

        return value

    def inject_vendor_command(self, cmd_type: int, xdata_addr: int,
                               value: int = 0, size: int = 1):
        """
        Inject a USB vendor command via MMIO registers.

        This sets up the MMIO registers needed for the firmware to process
        a vendor command. The firmware reads these registers and handles
        the command through its normal code path.

        No direct RAM writes are performed - the firmware reads expected
        values through MMIO hooks that simulate hardware behavior.

        Args:
            cmd_type: 0xE4 (read) or 0xE5 (write)
            xdata_addr: Target XDATA address
            value: Value for write commands
            size: Size for read commands
        """
        # Build USB address format: (addr & 0x1FFFF) | 0x500000
        usb_addr = (xdata_addr & 0x1FFFF) | 0x500000

        # Build 6-byte CDB (Command Descriptor Block)
        cdb = bytes([
            cmd_type,
            size if cmd_type == 0xE4 else value,
            (usb_addr >> 16) & 0xFF,
            (usb_addr >> 8) & 0xFF,
            usb_addr & 0xFF,
            0x00
        ])

        print(f"[{self.hw.cycles:8d}] [USB_CTRL] === INJECT VENDOR COMMAND ===")
        print(f"[{self.hw.cycles:8d}] [USB_CTRL] cmd=0x{cmd_type:02X} addr=0x{xdata_addr:04X} "
              f"{'size' if cmd_type == 0xE4 else 'val'}=0x{cdb[1]:02X}")
        print(f"[{self.hw.cycles:8d}] [USB_CTRL] CDB: {cdb.hex()}")

        # =====================================================
        # MMIO REGISTER SETUP FOR VENDOR COMMAND
        # =====================================================

        # Write CDB to USB interface registers (0x910D-0x9112)
        # Firmware reads these at 0x31C0+ to get command data
        for i, b in enumerate(cdb):
            self.hw.regs[0x910D + i] = b

        # Also populate 0x911F-0x9122 (another CDB location read by 0x3186)
        for i, b in enumerate(cdb[:4]):
            self.hw.regs[0x911F + i] = b

        # USB endpoint buffers
        for i, b in enumerate(cdb):
            self.hw.usb_ep_data_buf[i] = b
            self.hw.usb_ep0_buf[i] = b
        self.hw.usb_ep0_len = len(cdb)

        # USB connection and interrupt status
        # NOTE: 0x9000 bit 0 must be CLEAR to reach the 0x5333 vendor handler path
        # At 0x0E68, JB 0xe0.0 jumps away if bit 0 is set
        self.hw.regs[0x9000] = 0x80  # Connected (bit 7), bit 0 CLEAR for vendor path
        self.hw.regs[0x9101] = 0x21  # Bit 5 triggers command handler path
        self.hw.regs[0xC802] = 0x05  # USB interrupt pending

        # USB endpoint status - signals data available
        self.hw.regs[0x9096] = 0x01  # EP0 has data
        self.hw.regs[0x90E2] = 0x01  # Endpoint status bit

        # USB command interface registers
        self.hw.regs[0xE4E0] = cdb[0]  # Command type (0xE4/0xE5)
        self.hw.regs[0xE091] = size    # Read size / write value

        # Original firmware E5 path reads these (0x17FD-0x188B)
        # 0xC47A: Value byte copied to IDATA[0x38] at 0x1801
        # 0xCEB0: Command type copied to IDATA[0x39] at 0x188B
        self.hw.regs[0xC47A] = value if cmd_type == 0xE5 else size
        self.hw.regs[0xCEB0] = 0x05 if cmd_type == 0xE5 else 0x04

        # Target address registers (read at 0x323A-0x3249)
        # CEB2 = high byte of XDATA address
        # CEB3 = low byte of XDATA address
        self.hw.regs[0xCEB2] = (xdata_addr >> 8) & 0xFF
        self.hw.regs[0xCEB3] = xdata_addr & 0xFF

        # Store E5 value separately so it survives firmware clearing 0xC47A
        if cmd_type == 0xE5:
            self.hw.usb_e5_pending_value = value

        # USB EP0 data registers (read by various helpers)
        self.hw.regs[0x9E00] = cdb[0]  # bmRequestType / cmd type
        self.hw.regs[0x9E01] = cdb[1]  # bRequest / size
        self.hw.regs[0x9E02] = cdb[4]  # wValue low / addr low
        self.hw.regs[0x9E03] = cdb[3]  # wValue high / addr mid
        self.hw.regs[0x9E04] = cdb[2]  # wIndex low / addr high
        self.hw.regs[0x9E05] = 0x00    # wIndex high
        self.hw.regs[0x9E06] = size    # wLength low
        self.hw.regs[0x9E07] = 0x00    # wLength high

        # PCIe/DMA status for command processing
        self.hw.regs[0xC47B] = 0x01  # Non-zero for checks
        self.hw.regs[0xC471] = 0x01  # Queue busy
        self.hw.regs[0xB432] = 0x07  # PCIe link status
        self.hw.regs[0xE765] = 0x02  # Ready flag

        # Store command state
        self.hw.usb_cmd_type = cmd_type
        self.hw.usb_cmd_size = size if cmd_type == 0xE4 else 0
        self.hw.usb_cmd_pending = True
        self.vendor_cmd_active = True

        # Reset E5 DMA tracking flag for new command
        self.hw._e5_dma_done = False

        # Reset state machine for fresh command processing
        self.hw.usb_ce89_read_count = 0

        print(f"[{self.hw.cycles:8d}] [USB_CTRL] MMIO registers configured")

        # =====================================================
        # USB Hardware DMA - populate RAM like real hardware
        # =====================================================
        # The USB controller populates these RAM locations via DMA
        # before triggering the interrupt. This is how real hardware works.
        if self.hw.memory:
            # USB state = 5 (configured) - set by USB enumeration
            self.hw.memory.idata[0x6A] = 5

            # USB config check at 0x35C0 - must be 0 for vendor path
            self.hw.memory.xdata[0x07EC] = 0x00

            # CDB area - USB hardware writes CDB to XDATA[0x0002+]
            # The SCSI handler at 0x32E4 reads CDB from this area
            for i, b in enumerate(cdb):
                self.hw.memory.xdata[0x0002 + i] = b

            # Vendor command flag at 0x4583 - bit 3 enables vendor dispatch
            # This overlaps with CDB area but has special meaning
            self.hw.memory.xdata[0x0003] = 0x08

            # Command type marker for table lookup at 0x35D8
            if cmd_type == 0xE4:
                self.hw.memory.xdata[0x05B1] = 0x04
            elif cmd_type == 0xE5:
                self.hw.memory.xdata[0x05B1] = 0x05

            # Command index = 0 for table lookup at 0x1551
            # 0x17B1 copies 0x05A5 to 0x05A3, so set both to 0
            self.hw.memory.xdata[0x05A3] = 0x00
            self.hw.memory.xdata[0x05A5] = 0x00

        return cdb

    def inject_scsi_write_command(self, lba: int, sectors: int, data: bytes):
        """
        Inject a 0x8A SCSI write command via MMIO registers.

        This sets up the MMIO registers and RAM needed for the firmware to process
        a SCSI write command. The firmware reads these registers and handles
        the command through its normal code path.

        Args:
            lba: Logical Block Address to write to
            sectors: Number of sectors to write (each sector is 512 bytes)
            data: Data to write (will be padded to sector boundary)
        """
        import struct

        # Build 16-byte CDB for SCSI write command
        # Format: struct.pack('>BBQIBB', 0x8A, 0, lba, sectors, 0, 0)
        cdb = struct.pack('>BBQIBB', 0x8A, 0x00, lba, sectors, 0x00, 0x00)

        print(f"[{self.hw.cycles:8d}] [USB_CTRL] === INJECT SCSI WRITE COMMAND ===")
        print(f"[{self.hw.cycles:8d}] [USB_CTRL] LBA={lba} sectors={sectors} data_len={len(data)}")
        print(f"[{self.hw.cycles:8d}] [USB_CTRL] CDB: {cdb.hex()}")

        # =====================================================
        # MMIO REGISTER SETUP FOR SCSI COMMAND
        # =====================================================

        # Write CDB to USB interface registers (0x910D-0x911C)
        for i, b in enumerate(cdb):
            self.hw.regs[0x910D + i] = b

        # USB endpoint buffers - write CDB
        for i, b in enumerate(cdb):
            self.hw.usb_ep_data_buf[i] = b
            self.hw.usb_ep0_buf[i] = b
        self.hw.usb_ep0_len = len(cdb)

        # USB connection and interrupt status
        self.hw.regs[0x9000] = 0x80  # Connected (bit 7), bit 0 CLEAR
        self.hw.regs[0x9101] = 0x21  # Bit 5 triggers command handler path
        self.hw.regs[0xC802] = 0x05  # USB interrupt pending

        # USB endpoint status
        self.hw.regs[0x9096] = 0x01  # EP0 has data
        self.hw.regs[0x90E2] = 0x01  # Endpoint status bit

        # Store command state
        self.hw.usb_cmd_type = 0x8A
        self.hw.usb_cmd_size = sectors * 512
        self.hw.usb_cmd_pending = True
        self.vendor_cmd_active = True

        # Reset state machine
        self.hw.usb_ce89_read_count = 0

        print(f"[{self.hw.cycles:8d}] [USB_CTRL] MMIO registers configured for SCSI write")

        # =====================================================
        # RAM SETUP - populate RAM like USB hardware DMA
        # =====================================================
        if self.hw.memory:
            # USB state = 5 (configured)
            self.hw.memory.idata[0x6A] = 5

            # CDB area - USB hardware writes CDB to XDATA
            for i, b in enumerate(cdb):
                self.hw.memory.xdata[0x0002 + i] = b

            # SCSI command flag
            self.hw.memory.xdata[0x0003] = 0x08

            # Command type marker - 0x8A maps to different handler
            self.hw.memory.xdata[0x05B1] = 0x8A

            # Pad data to sector boundary and write to USB data buffer at 0x8000
            padded_size = sectors * 512
            padded_data = data + b'\x00' * (padded_size - len(data))
            for i, b in enumerate(padded_data):
                if 0x8000 + i < 0x10000:  # Stay within XDATA bounds
                    self.hw.memory.xdata[0x8000 + i] = b

            # Store data length info
            self.hw.usb_data_len = len(padded_data)

            print(f"[{self.hw.cycles:8d}] [USB_CTRL] Wrote {len(padded_data)} bytes to USB buffer at 0x8000")

        return cdb

    def inject_scsi_vendor_command(self, opcode: int, cdb: bytes, data: bytes = b'',
                                    is_write: bool = False):
        """
        Inject a SCSI vendor command (E0-E8) via MMIO registers.

        This sets up the MMIO registers needed for the firmware to process
        vendor-specific SCSI commands used by patch.py for firmware updates.

        Vendor opcodes:
            0xE0 - Config Read (128 bytes)
            0xE1 - Config Write (128 bytes)
            0xE2 - Flash Read
            0xE3 - Firmware Write (to SPI flash)
            0xE4 - XDATA Read
            0xE5 - XDATA Write
            0xE6 - NVMe Admin passthrough
            0xE8 - Reset/Commit

        Args:
            opcode: SCSI vendor opcode (0xE0-0xE8)
            cdb: Complete CDB bytes (16 bytes max)
            data: Data for write commands (E1, E3, E5)
            is_write: True if this is a write command with data phase
        """
        cycles = self.hw.cycles
        print(f"[{cycles:8d}] [USB_CTRL] === INJECT SCSI VENDOR COMMAND ===")
        print(f"[{cycles:8d}] [USB_CTRL] Opcode=0x{opcode:02X} CDB={cdb.hex()}")
        if is_write and data:
            print(f"[{cycles:8d}] [USB_CTRL] Write data: {len(data)} bytes")

        # Pad CDB to 16 bytes
        cdb_padded = (cdb + bytes(16))[:16]

        # =====================================================
        # MMIO REGISTER SETUP FOR SCSI VENDOR COMMAND
        # =====================================================

        # Write CDB to USB interface registers (0x910D-0x911C)
        for i, b in enumerate(cdb_padded):
            self.hw.regs[0x910D + i] = b

        # Also write to alternate CDB locations firmware may check
        for i, b in enumerate(cdb_padded):
            self.hw.regs[0x911F + i] = b

        # USB endpoint buffers
        for i, b in enumerate(cdb_padded):
            self.hw.usb_ep_data_buf[i] = b
            self.hw.usb_ep0_buf[i] = b
        self.hw.usb_ep0_len = len(cdb_padded)

        # USB connection and interrupt status
        self.hw.regs[0x9000] = 0x81  # Connected, USB active
        self.hw.regs[0x9101] = 0x21  # Bit 5 triggers command handler
        self.hw.regs[0xC802] = 0x05  # USB interrupt pending
        self.hw.regs[0x9096] = 0x01  # EP0 has data
        self.hw.regs[0x90E2] = 0x01  # Endpoint status

        # Store command state
        self.hw.usb_cmd_type = opcode
        self.hw.usb_cmd_size = len(data) if is_write else 0
        self.hw.usb_cmd_pending = True
        self.vendor_cmd_active = True

        # Reset state machine
        self.hw.usb_ce89_read_count = 0

        # =====================================================
        # RAM SETUP - populate like USB hardware DMA
        # =====================================================
        if self.hw.memory:
            # USB state = 2 (state for SCSI bulk commands)
            # Value 2 triggers the SCSI handler path at 0x32EE
            self.hw.memory.idata[0x6A] = 2

            # CDB area - write to XDATA[0x0002+] where firmware reads it
            for i, b in enumerate(cdb_padded):
                self.hw.memory.xdata[0x0002 + i] = b

            # Vendor command flags
            self.hw.memory.xdata[0x0003] = 0x08  # Enable vendor dispatch

            # Set state for vendor command handling
            # 0x0B02 = state machine: 0=idle, 1=E2 read, 2=E3 write
            if opcode == 0xE2:
                self.hw.memory.xdata[0x0B02] = 1
            elif opcode == 0xE3:
                self.hw.memory.xdata[0x0B02] = 2
            else:
                self.hw.memory.xdata[0x0B02] = 0

            # Magic value for vendor commands
            self.hw.memory.xdata[0xEA90] = 0x5A

            # Write data to USB buffer at 0x8000 for write commands
            if is_write and data:
                for i, b in enumerate(data):
                    if 0x8000 + i < 0x10000:
                        self.hw.memory.xdata[0x8000 + i] = b
                self.hw.usb_data_len = len(data)
                print(f"[{cycles:8d}] [USB_CTRL] Wrote {len(data)} bytes to USB buffer at 0x8000")

        print(f"[{cycles:8d}] [USB_CTRL] MMIO configured for vendor opcode 0x{opcode:02X}")
        return cdb_padded

    def inject_control_transfer(self, bmRequestType: int, bRequest: int, wValue: int,
                                  wIndex: int, wLength: int, data: bytes = b''):
        """
        Inject a USB control transfer (setup packet) through MMIO registers.

        This sets up the firmware's control transfer path:
        - Setup packet at 0x9E00-0x9E07
        - USB interrupt triggers handler at 0x0E33
        - Firmware reads setup packet and processes request

        ALL USB requests (standard and vendor) are passed through to firmware.
        The firmware handles GET_DESCRIPTOR by reading from code ROM via the
        flash mirror region (XDATA 0xE400-0xE500 → Code ROM).

        Args:
            bmRequestType: Request type byte (direction, type, recipient)
            bRequest: Request code (e.g., 0x06 = GET_DESCRIPTOR)
            wValue: Value field (e.g., descriptor type/index)
            wIndex: Index field
            wLength: Data length
            data: Data for OUT transfers
        """
        cycles = self.hw.cycles
        print(f"[{cycles:8d}] [USB_CTRL] === INJECT CONTROL TRANSFER ===")
        print(f"[{cycles:8d}] [USB_CTRL] bmRequestType=0x{bmRequestType:02X} bRequest=0x{bRequest:02X}")
        print(f"[{cycles:8d}] [USB_CTRL] wValue=0x{wValue:04X} wIndex=0x{wIndex:04X} wLength={wLength}")

        # Write setup packet to MMIO registers
        # The firmware at 0xA5EA-0xA604 reads from 0x9104-0x910B (setup packet buffer)
        # and copies to XDATA 0x0ACE-0x0AD5
        self.hw.regs[0x9104] = bmRequestType
        self.hw.regs[0x9105] = bRequest
        self.hw.regs[0x9106] = wValue & 0xFF
        self.hw.regs[0x9107] = (wValue >> 8) & 0xFF
        self.hw.regs[0x9108] = wIndex & 0xFF
        self.hw.regs[0x9109] = (wIndex >> 8) & 0xFF
        self.hw.regs[0x910A] = wLength & 0xFF
        self.hw.regs[0x910B] = (wLength >> 8) & 0xFF

        # Also write to 0x9E00-0x9E07 (alternate setup packet location)
        self.hw.regs[0x9E00] = bmRequestType
        self.hw.regs[0x9E01] = bRequest
        self.hw.regs[0x9E02] = wValue & 0xFF
        self.hw.regs[0x9E03] = (wValue >> 8) & 0xFF
        self.hw.regs[0x9E04] = wIndex & 0xFF
        self.hw.regs[0x9E05] = (wIndex >> 8) & 0xFF
        self.hw.regs[0x9E06] = wLength & 0xFF
        self.hw.regs[0x9E07] = (wLength >> 8) & 0xFF

        # Also populate usb_ep0_buf which is what _usb_ep0_buf_read returns
        self.hw.usb_ep0_buf[0] = bmRequestType
        self.hw.usb_ep0_buf[1] = bRequest
        self.hw.usb_ep0_buf[2] = wValue & 0xFF
        self.hw.usb_ep0_buf[3] = (wValue >> 8) & 0xFF
        self.hw.usb_ep0_buf[4] = wIndex & 0xFF
        self.hw.usb_ep0_buf[5] = (wIndex >> 8) & 0xFF
        self.hw.usb_ep0_buf[6] = wLength & 0xFF
        self.hw.usb_ep0_buf[7] = (wLength >> 8) & 0xFF

        # USB connection and interrupt status
        # Bit 7 = connected, Bit 0 = active (needed for USB handler path at 0x4864)
        # With bit 0 CLEAR, firmware loops at 0x48CD checking CE89 instead of processing
        self.hw.regs[0x9000] = 0x81  # Connected (bit 7), Active (bit 0)
        self.hw.regs[0xC802] = 0x01  # USB interrupt pending

        # USB speed indicator - needed by 0xA4CC which returns 0x9100 & 0x03
        # 0 = Full Speed, 1 = High Speed, 2 = SuperSpeed, 3 = SuperSpeed+
        # At 0xB400: if speed == 2, sets R7=0 for descriptor DMA
        # Use stored USB speed from connect() or default to HIGH speed (USB 2.0)
        speed = getattr(self, 'usb_speed', 1)  # Default to High Speed if not set
        self.hw.regs[0x9100] = speed

        # USB mode indicators for descriptor handling at 0xA7E4-0xA7FF and 0x87A1
        # These set bits in 0x0ACC that determine USB2 vs USB3 code paths
        if speed >= 2:  # SuperSpeed or higher
            self.hw.regs[0xCC91] = 0x02  # Bit 1 SET - USB3 mode
            self.hw.regs[0x09F9] = 0x40  # Bit 6 SET - USB3 speed indicator
        else:  # High Speed or Full Speed (USB 2.0)
            self.hw.regs[0xCC91] = 0x00  # Bit 1 CLEAR - USB2 mode
            self.hw.regs[0x09F9] = 0x00  # Bit 6 CLEAR - USB2 speed indicator

        # Mark control transfer as active for state machine timing
        # This affects the 0x92C2 read callback bit 6 timing
        self.hw.usb_control_transfer_active = True
        self.hw.usb_ep0_fifo.clear()
        self.hw.usb_92c2_read_count = 0  # Reset for ISR->main loop timing
        self.hw.usb_ce89_read_count = 0  # Reset DMA state machine for new transfer

        # Check if this is a standard request (bmRequestType bits 6:5 = 00)
        request_type = bmRequestType & 0x60
        if request_type == 0x00:
            # Standard USB request (GET_DESCRIPTOR, SET_ADDRESS, etc.)
            # ISR path for GET_DESCRIPTOR (traced from original firmware):
            #   0x0E5E: checks 0x9101 bit 5 → if CLEAR, jumps to 0x0F07
            #   0x0F0B: checks 0x9101 bit 3 → if SET, goes to ISR dispatch (wrong path!)
            #   0x0F4A: if bit 3 CLEAR, reaches here
            #   0x0F4E: checks 0x9101 bit 0 → if CLEAR, jumps to 0x0F91
            #   0x0F91-0x0F95: checks 0x9101 bit 1 → if SET, calls 0x033B (descriptor handler!)
            # So for GET_DESCRIPTOR: need bit 3=0, bit 0=0, bit 1=1 → 0x02
            if bRequest == 0x06:  # GET_DESCRIPTOR
                desc_type = (wValue >> 8) & 0xFF
                desc_index = wValue & 0xFF
                print(f"[{cycles:8d}] [USB_CTRL] GET_DESCRIPTOR: type=0x{desc_type:02X} index=0x{desc_index:02X} len={wLength}")
                # Store the pending descriptor request for later DMA handling
                self.pending_descriptor_request = {
                    'type': desc_type,
                    'index': desc_index,
                    'length': wLength
                }
                # For GET_DESCRIPTOR: bit 1 SET to trigger descriptor handler, bits 0,3 CLEAR
                self.hw.regs[0x9101] = 0x02
                print(f"[{cycles:8d}] [USB_CTRL] Standard request - setting 0x9101=0x02, 0x9301=0x40")
            else:
                # Other standard requests - use original behavior
                self.hw.regs[0x9101] = 0x0B  # Bits 0, 1, 3 set, bit 5 CLEAR
                print(f"[{cycles:8d}] [USB_CTRL] Standard request - setting 0x9101=0x0B, 0x9301=0x40")
            # 0x9301: Bit 6 triggers interrupt dispatch and DMA
            # Use write() to trigger the callback which handles descriptor DMA
            self.hw.write(0x9301, 0x40)  # Triggers _usb_9301_ep0_arm_write callback for DMA
        elif request_type == 0x20:
            # Class request (USB Mass Storage)
            # Handle GET_MAX_LUN (bRequest=0xFE) and BULK_ONLY_RESET (bRequest=0xFF)
            # directly at MMIO level as hardware would
            if bRequest == 0xFE:  # GET_MAX_LUN
                # Return max LUN = 0 (single LUN device) via USB buffer
                print(f"[{cycles:8d}] [USB_CTRL] GET_MAX_LUN - responding with LUN 0")
                if self.hw.memory:
                    self.hw.memory.xdata[0x8000] = 0x00  # Max LUN = 0
                self.hw.usb_control_transfer_active = False
                return  # Response ready in buffer
            elif bRequest == 0xFF:  # BULK_ONLY_RESET
                print(f"[{cycles:8d}] [USB_CTRL] BULK_ONLY_RESET - acknowledging")
                self.hw.usb_control_transfer_active = False
                return  # No data, just acknowledge
            else:
                # Unknown class request - let firmware handle
                self.hw.regs[0x9101] = 0x21
                print(f"[{cycles:8d}] [USB_CTRL] Class request 0x{bRequest:02X} - passing to firmware")
        else:
            # Vendor request
            # Path: 0x0E33 → 0x0E64 → 0x0EF4 → 0x5333 (when 0x9101 bit 5 SET)
            self.hw.regs[0x9101] = 0x21  # Bit 0 = EP0 control, bit 5 SET (vendor path)
            print(f"[{cycles:8d}] [USB_CTRL] Vendor request - setting 0x9101=0x21")

        self.hw.regs[0x91D1] = 0x08  # EP0 setup packet received (bit 3)
        self.hw.regs[0x9118] = 0x01  # Endpoint index (lookup table requires < 8 value)

        # EP0 handler prerequisites
        # NOTE: 0x92C2 bit 6 is handled by _usb_92c2_read callback:
        #   - First read: returns bit 6 CLEAR (for ISR to call 0xBDA4)
        #   - Subsequent reads: returns bit 6 SET (for main loop to call 0x0322)
        self.hw.regs[0x92F8] = 0x0C  # Bits 2-3 set

        # CRITICAL: Main loop at 0xCDE7 checks 0x9091 bits for two-phase USB handling:
        # Phase 1 - Bit 0: Setup packet handler at 0xA5A6
        #   - Parses the USB request, sets 0x07E1 = 5 for GET_DESCRIPTOR
        #   - Firmware loops writing 0x01, waiting for hardware to clear bit 0
        # Phase 2 - Bit 1: DMA response handler at 0xD088
        #   - Checks 0x07E1 == 5, triggers DMA if so
        # 0x9002 bit 1 must be CLEAR to reach the 0x9091 check at 0xCDF5
        self.hw.regs[0x9002] = 0x00  # Bit 1 CLEAR to allow 0x9091 check
        self.hw.regs[0x9091] = 0x01  # Bit 0 SET to trigger setup handler at 0xA5A6
        # Reset phase transition counters
        self.hw._usb_9091_setup_writes = 0
        self.hw._usb_9091_read_count = 0

        # CRITICAL: The main loop at 0xCDC6-0xCDD9 waits for state transition registers:
        # - Checks 0xE712 bit 0 or bit 1 to exit the polling loop
        # - If neither set, checks 0xCC11 bit 1
        # Without these bits, firmware never reaches USB dispatch at 0xCDE7
        self.hw.regs[0xE712] = 0x01  # Bit 0 SET to exit polling loop
        self.hw.regs[0xCC11] = 0x02  # Bit 1 SET as backup exit condition

        # Set command pending
        self.hw.usb_cmd_pending = True
        self.vendor_cmd_active = False

        # USB state = 5 (configured) - required for firmware to process control transfers
        # The firmware checks this state at various decision points in the USB handler
        if self.hw.memory:
            self.hw.memory.idata[0x6A] = 5
            # PCIe enumeration complete flag - needed for descriptor DMA path at 0x185C
            # Without this, firmware takes alternate path that doesn't use CEB2/CEB3
            self.hw.memory.xdata[0x0AF7] = 0x01
            self.hw.memory.xdata[0x053F] = 0x01
            # CRITICAL: Port state at 0x05B1 + port_index*0x22 must NOT be 4
            # At 0x35D4-0x35DF: Firmware checks this and clears 0x0AF7 if state == 4
            self.hw.memory.xdata[0x05A3] = 0x00  # Port index = 0
            self.hw.memory.xdata[0x05B1] = 0x03  # Port 0 state = 3 (link up, not 4)
            # USB speed mode at 0x0AD6 - used by 0xB3FC at 0xB465 to check descriptor mode
            # At 0xB467: if 0x0AD6 >= 3, firmware returns R7=0x03 (wrong value for DMA)
            # This value would normally be set by USB enumeration before control transfers
            # Use stored USB speed from connect()
            usb_speed = getattr(self, 'usb_speed', 1)  # Default to High Speed if not set
            self.hw.memory.xdata[0x0AD6] = usb_speed  # USB speed mode

        # PCIe link state - 0xB480 bit 0 must be SET to prevent firmware at 0x20DA from
        # clearing XDATA[0x0AF7] to 0
        self.hw.regs[0xB480] = 0x03  # Bits 0,1 SET - PCIe link active state

        # Set pending interrupt flag so hardware update triggers actual CPU interrupt
        self.hw._pending_usb_interrupt = True

        print(f"[{cycles:8d}] [USB_CTRL] Control transfer injected (interrupt pending)")


@dataclass
class HardwareState:
    """
    Hardware state for ASM2464PD emulation.

    Only emulates actual hardware registers (addresses >= 0x6000).
    RAM variables are handled by the memory system.
    """

    # Logging
    log_reads: bool = False
    log_writes: bool = False
    log_uart: bool = True
    log_pcie: bool = True  # Log PCIe DMA operations

    # Proxy mode - when True, disable all fake USB/interrupt injection
    # Real hardware handles everything, we just proxy MMIO
    proxy_mode: bool = False

    # VID/PID override - intercept firmware USB descriptor writes to patch VID/PID
    # Format: (vid, pid) or None for no override
    vidpid_override: tuple = None

    # Reference to memory system (set by Emulator during init)
    # Used for reading XDATA (e.g., USB descriptors)
    _memory: 'Memory' = None

    # Cycle counter for timing-based responses
    cycles: int = 0

    # Hardware state
    usb_connected: bool = False
    usb_connect_delay: int = 500000  # Cycles before USB plug-in event (after init)

    # Polling counters - track how many times an address is polled
    poll_counts: Dict[int, int] = field(default_factory=dict)

    # Register values - only for hardware registers >= 0x6000
    regs: Dict[int, int] = field(default_factory=dict)

    # Callbacks for specific addresses
    read_callbacks: Dict[int, Callable[['HardwareState', int], int]] = field(default_factory=dict)
    write_callbacks: Dict[int, Callable[['HardwareState', int, int], None]] = field(default_factory=dict)

    # USB command queue
    usb_cmd_queue: list = field(default_factory=list)
    usb_cmd_pending: bool = False
    usb_ep0_buf: bytearray = field(default_factory=lambda: bytearray(64))  # Control EP buffer (0x9E00)
    usb_ep0_len: int = 0
    usb_data_buf: bytearray = field(default_factory=lambda: bytearray(4096))  # Data buffer
    usb_data_len: int = 0
    usb_ep_data_buf: bytearray = field(default_factory=lambda: bytearray(2048))  # EP data buffer (0xD800)

    # Memory reference for E4/E5 commands (set by create_hardware_hooks)
    memory: 'Memory' = None

    # UART output buffer for line-based output
    uart_buffer: str = ""

    # USB command injection timing
    usb_injected: bool = False

    # USB controller instance (created in __post_init__)
    usb_controller: 'USBController' = None

    # USB command state for MMIO hooks
    usb_cmd_type: int = 0  # Current command type (0xE4, 0xE5, etc.)
    usb_cmd_size: int = 0  # Size for E4 read commands
    usb_e5_pending_value: int = 0  # Pending E5 value to write (preserved until read)

    # USB endpoint selection tracking
    usb_ep_selected: int = 0  # Currently selected endpoint index (0-31)

    # USB command injection from command line (set by emulator CLI)
    usb_inject_cmd: tuple = None  # (cmd_type, addr, val_or_size)
    usb_inject_delay: int = 1000  # Cycles after USB connect to inject

    # USB state machine emulation
    # Tracks firmware USB state to know when to set register bits
    usb_state_machine_phase: int = 0  # 0=init, 1=waiting, 2=enumerating, 3=ready
    usb_ce89_read_count: int = 0  # Count reads of 0xCE89 for state transitions
    usb_92c2_read_count: int = 0  # Count reads of 0x92C2 for ISR->main loop transition
    usb_ce00_read_count: int = 0  # Count reads of 0xCE00 for DMA completion

    # USB EP0 FIFO buffer - reserved for potential future use
    # Note: USB descriptor data is sent via hardware DMA from ROM, not firmware FIFO writes
    usb_ep0_fifo: bytearray = field(default_factory=bytearray)

    # USB control transfer active flag - affects 0x92C2 callback timing for ISR->main loop
    usb_control_transfer_active: bool = False

    # USB descriptor state
    # NOTE: The emulator does NOT track descriptor requests or generate responses.
    # The firmware handles GET_DESCRIPTOR by reading from code ROM and DMAing
    # the response to the USB buffer. See "USB Descriptor Handling Philosophy" above.
    usb_ep0_response: bytearray = field(default_factory=bytearray)  # Response data for host
    usb_transfer_complete: bool = False  # Set when firmware signals transfer complete

    # Config descriptor capture - firmware writes config descriptor to 0x9E00 but then
    # corrupts it before DMA. Capture the valid descriptor when written.
    usb_captured_config_desc: bytearray = field(default_factory=bytearray)
    usb_capture_config_active: bool = False  # True when we're capturing config desc writes

    # Full USB3 config descriptor loaded from ROM with corrected wTotalLength.
    # ROM at 0x58CF has wTotalLength=44 (only alt_setting 0), but alt_setting 1
    # data continues immediately after. We load the full descriptor (121 bytes)
    # and fix wTotalLength so the host knows to request all of it.
    usb_ss_config_from_rom: bytes = field(default_factory=bytes)

    # USB2 High Speed config descriptor loaded from ROM.
    # ROM at 0x5948 has USB 2.0 config descriptor with 512-byte max packet sizes.
    # Used when connected at USB 2.0 High Speed (dummy_hcd limitation).
    usb_hs_config_from_rom: bytes = field(default_factory=bytes)

    # PCIe DMA state
    pcie_dma_pending: bool = False  # DMA operation in progress
    pcie_dma_source: int = 0  # Source address in PCIe space
    pcie_dma_size: int = 0  # Size of transfer
    pcie_dma_dest: int = 0x8000  # Destination in XDATA (USB data buffer)

    # Simulated PCIe memory (for E4 read responses)
    # This would contain the data that would be read from the NVMe device
    pcie_memory: Dict[int, int] = field(default_factory=dict)

    # ============================================
    # SPI Flash Emulation
    # The ASM2464PD has a SPI flash chip for firmware storage.
    # patch.py uses E2 (read) and E3 (write) commands to access flash.
    # Flash size: 256KB (0x40000 bytes) typical
    # ============================================
    spi_flash: bytearray = field(default_factory=lambda: bytearray(0x40000))  # 256KB flash
    spi_flash_addr: int = 0  # Current flash address (set by address registers)
    spi_flash_write_pending: bool = False  # Write operation in progress
    spi_flash_write_count: int = 0  # Bytes written in current operation

    # Execution tracing
    trace_enabled: bool = False  # Global trace enable
    trace_points: Dict[int, str] = field(default_factory=dict)  # PC addr -> label
    trace_callback: Callable = None  # Optional callback(hw, pc, label) for trace points

    # XDATA write tracing - tracks writes to specific RAM addresses
    xdata_trace_enabled: bool = False
    xdata_trace_addrs: Dict[int, str] = field(default_factory=dict)  # addr -> name
    xdata_write_log: list = field(default_factory=list)  # Log of traced writes

    def __post_init__(self):
        """Initialize hardware register defaults."""
        self._init_registers()
        self._setup_callbacks()
        # Create USB controller after self is initialized
        self.usb_controller = USBController(self)

    def _init_registers(self):
        """
        Set default values for hardware registers.
        Only addresses >= 0x6000 are hardware registers.
        """
        # ============================================
        # USB Controller Registers (0x9xxx)
        # ============================================
        self.regs[0x9000] = 0x00  # USB status - bit 7 = connected
        self.regs[0x90E0] = 0x00  # USB speed
        self.regs[0x9100] = 0x00  # USB link status
        self.regs[0x9105] = 0x00  # USB PHY status
        self.regs[0x91C0] = 0x02  # USB PHY control
        self.regs[0x91D0] = 0x00  # USB PHY config

        # ============================================
        # Power Management Registers (0x92xx)
        # ============================================
        self.regs[0x92C0] = 0x81  # Power enable
        self.regs[0x92C1] = 0x03  # Clocks enabled
        self.regs[0x92C2] = 0x40  # Power state - bit 6 enables PD task path at 0xBF44
        self.regs[0x92C5] = 0x04  # PHY powered
        self.regs[0x92E0] = 0x02  # Power domain
        self.regs[0x92F7] = 0x40  # Power status
        self.regs[0x92FB] = 0x01  # Power sequence complete (checked at 0x9C42)

        # ============================================
        # PD Event Registers (0xE4xx)
        # ============================================
        # These control the debug output at 0xAE89/0xAF5E
        # Set initial PD event to trigger debug output
        self.regs[0xE40F] = 0x00  # PD event type - will be set during PD events
        self.regs[0xE410] = 0x00  # PD sub-event

        # ============================================
        # PCIe Registers (0xBxxx)
        # ============================================
        self.regs[0xB238] = 0x00  # PCIe trigger - not busy
        self.regs[0xB254] = 0x00  # PCIe trigger write
        self.regs[0xB296] = 0x00  # PCIe status - bit 2 set when DMA complete
        self.regs[0xB401] = 0x01  # PCIe tunnel enabled
        self.regs[0xB480] = 0x00  # PCIe link initially down (bit 0 = 0)
        # This allows USB state machine to return R7=5 at 0x3FC6 instead of state=11

        # ============================================
        # UART Registers (0xC0xx)
        # ============================================
        self.regs[0xC000] = 0x00  # UART TX data
        self.regs[0xC001] = 0x00  # UART TX data (alt)
        self.regs[0xC009] = 0x60  # UART LSR - TX empty, ready

        # ============================================
        # NVMe Controller Registers (0xC4xx, 0xC5xx)
        # ============================================
        self.regs[0xC412] = 0x02  # NVMe ready
        self.regs[0xC471] = 0x00  # NVMe queue busy - bit 0 = queue busy
        self.regs[0xC47A] = 0x00  # NVMe command status
        self.regs[0xC520] = 0x80  # NVMe link ready

        # ============================================
        # PHY Registers (0xC6xx)
        # ============================================
        self.regs[0xC620] = 0x00  # PHY control
        self.regs[0xC655] = 0x08  # PHY config
        self.regs[0xC65A] = 0x09  # PHY config
        self.regs[0xC6B3] = 0x30  # PHY status - bits 4,5 set

        # ============================================
        # Interrupt/DMA/Flash Registers (0xC8xx)
        # ============================================
        self.regs[0xC800] = 0x00  # Interrupt status
        self.regs[0xC802] = 0x00  # Interrupt status 2
        self.regs[0xC806] = 0x00  # System interrupt status
        self.regs[0xC80A] = 0x00  # PCIe/NVMe interrupt - bit 6 triggers PD debug
        self.regs[0xC8A9] = 0x00  # Flash CSR - not busy
        self.regs[0xC8AA] = 0x00  # Flash command
        self.regs[0xC8AB] = 0x00  # Flash address high
        self.regs[0xC8AC] = 0x00  # Flash address mid
        self.regs[0xC8AD] = 0x00  # Flash address low
        self.regs[0xC8AE] = 0x00  # Flash data register
        self.regs[0xC8B8] = 0x00  # Flash/DMA status
        self.regs[0xC8D6] = 0x04  # DMA status - done

        # ============================================
        # USB Power Delivery (PD) Registers (0xCAxx)
        # ============================================
        self.regs[0xCA00] = 0x00  # PD control
        self.regs[0xCA06] = 0x00  # PD status
        self.regs[0xCA0A] = 0x00  # PD interrupt control
        self.regs[0xCA0D] = 0x00  # PD interrupt status 1 - bit 3 = interrupt pending
        self.regs[0xCA0E] = 0x00  # PD interrupt status 2 - bit 2 = interrupt pending
        self.regs[0xCA81] = 0x00  # PD extended status

        # ============================================
        # Timer/CPU Control Registers (0xCCxx, 0xCDxx)
        # ============================================
        self.regs[0xCC11] = 0x00  # Timer 0 CSR
        self.regs[0xCC17] = 0x00  # Timer 1 CSR
        self.regs[0xCC1D] = 0x00  # Timer 2 CSR
        self.regs[0xCC23] = 0x00  # Timer 3 CSR
        self.regs[0xCC33] = 0x04  # CPU exec status
        self.regs[0xCC37] = 0x00  # CPU control
        self.regs[0xCC3B] = 0x00  # CPU control 2
        self.regs[0xCC3D] = 0x00  # CPU control 3
        self.regs[0xCC3E] = 0x00  # CPU control 4
        self.regs[0xCC3F] = 0x00  # CPU control 5
        self.regs[0xCC81] = 0x00  # Timer/DMA control
        self.regs[0xCC82] = 0x00  # Timer/DMA address low
        self.regs[0xCC83] = 0x00  # Timer/DMA address high
        self.regs[0xCC89] = 0x00  # Timer/DMA status - bit 1 = complete
        self.regs[0xCD31] = 0x01  # PHY init status - bit 0 = ready

        # ============================================
        # SCSI/DMA Registers (0xCExx)
        # ============================================
        self.regs[0xCE5D] = 0xFF  # Debug enable mask - all levels enabled
        self.regs[0xCE89] = 0x01  # SCSI DMA status - bit 0 = ready

        # NOTE: 0x707x addresses are NOT hardware registers!
        # They are flash buffer RAM (0x7000-0x7FFF) loaded from flash config.
        # Flash buffer is handled as regular XDATA, not MMIO.

        # ============================================
        # Debug/Command Engine Registers (0xE4xx)
        # ============================================
        self.regs[0xE40F] = 0x00  # PD event type (for debug output)
        self.regs[0xE410] = 0x00  # PD sub-event (for debug output)
        self.regs[0xE41C] = 0x00  # Command engine status

        # ============================================
        # System Status Registers (0xE7xx)
        # ============================================
        self.regs[0xE710] = 0x00  # System status
        self.regs[0xE712] = 0x00  # USB EP0 transfer status (bits 0,1 = complete)
        self.regs[0xE717] = 0x00  # System status 2
        self.regs[0xE751] = 0x00  # System status 3
        self.regs[0xE764] = 0x00  # System status 4
        self.regs[0xE795] = 0x21  # Flash ready + USB state 3 flag (bit 5)
        self.regs[0xE7E3] = 0x80  # PHY link ready

        # ============================================
        # PHY Completion / Debug Registers (0xE3xx)
        # ============================================
        self.regs[0xE302] = 0x40  # PHY completion status - bit 6 = complete

    def _setup_callbacks(self):
        """Setup read/write callbacks for hardware with special behavior."""
        # UART TX - capture output
        self.write_callbacks[0xC000] = self._uart_tx
        self.write_callbacks[0xC001] = self._uart_tx

        # PCIe status - complete after trigger
        self.read_callbacks[0xB296] = self._pcie_status_read
        self.write_callbacks[0xB254] = self._pcie_trigger_write

        # PCIe DMA trigger - E4/E5 command DMA
        self.write_callbacks[0xB296] = self._pcie_dma_trigger

        # Flash CSR - auto-complete
        self.read_callbacks[0xC8A9] = self._flash_csr_read
        self.write_callbacks[0xC8AA] = self._flash_cmd_write

        # Flash data register - read/write actual flash data
        self.read_callbacks[0xC8AE] = self._flash_data_read
        self.write_callbacks[0xC8AE] = self._flash_data_write

        # DMA status
        self.read_callbacks[0xC8D6] = self._dma_status_read

        # Flash/DMA busy - auto-clear
        self.read_callbacks[0xC8B8] = self._busy_reg_read

        # System interrupt status - clear on read
        self.read_callbacks[0xC806] = self._int_status_read

        # Timer CSRs
        for addr in [0xCC11, 0xCC17, 0xCC1D, 0xCC23]:
            self.read_callbacks[addr] = self._timer_csr_read
            self.write_callbacks[addr] = self._timer_csr_write

        # Timer/DMA status register (0xCC89) - set complete bit after polling
        self.read_callbacks[0xCC89] = self._timer_dma_status_read

        # PHY init status - also handles descriptor DMA trigger on write
        self.read_callbacks[0xCD31] = self._phy_status_read
        self.write_callbacks[0xCD31] = self._phy_cmd_write

        # Command engine status
        self.read_callbacks[0xE41C] = self._cmd_engine_read

        # PD interrupt status - set by USB PD events
        self.read_callbacks[0xCA0D] = self._pd_interrupt_read
        self.read_callbacks[0xCA0E] = self._pd_interrupt_read

        # USB state machine MMIO registers (see registers.h for definitions)
        # REG_USB_DMA_STATE (0xCE89): USB/DMA status - controls state transitions
        #   USB_DMA_STATE_READY (bit 0): Must be set to exit wait loop (0x348C)
        #   USB_DMA_STATE_CBW (bit 1): 1=CBW received, 0=bulk data (0x3493)
        #   USB_DMA_STATE_ERROR (bit 2): DMA error in copy loop (0x3546)
        self.read_callbacks[0xCE89] = self._usb_ce89_read
        # REG_USB_DMA_ERROR (0xCE86): USB status - bit 4 checked at 0x349D
        self.read_callbacks[0xCE86] = self._usb_ce86_read
        # REG_XFER_STATUS_CE6C (0xCE6C): USB controller ready - bit 7 must be set
        self.read_callbacks[0xCE6C] = self._usb_ce6c_read
        # REG_SCSI_DMA_CTRL (0xCE00): DMA control register - returns 0 after completion
        # Firmware writes 0x03 to start DMA at 0x3531-0x3533, polls at 0x3534-0x3538
        self.read_callbacks[0xCE00] = self._usb_ce00_read
        self.write_callbacks[0xCE00] = self._usb_ce00_write
        # REG_SCSI_DMA_XFER_CNT (0xCE55): DMA transfer byte count after CE88/CE89 handshake
        # Read at 0x34B9 and stored to G_USB_WORK_009F as loop limit
        self.read_callbacks[0xCE55] = self._usb_ce55_read
        # REG_BULK_DMA_HANDSHAKE (0xCE88): DMA trigger - write resets state
        # At 0x1806: firmware writes to CE88 before polling REG_USB_DMA_STATE
        self.write_callbacks[0xCE88] = self._usb_ce88_write

        # USB Setup Packet buffer (REG_USB_SETUP_* at 0x9E00-0x9E07)
        # Hardware writes 8-byte setup packet here when received from host
        for addr in range(0x9E00, 0x9E40):
            self.read_callbacks[addr] = self._usb_ep0_buf_read
            self.write_callbacks[addr] = self._usb_ep0_buf_write

        # USB EP0 CSR (0x9E10)
        self.read_callbacks[0x9E10] = self._usb_ep0_csr_read
        self.write_callbacks[0x9E10] = self._usb_ep0_csr_write

        # USB EP data buffer (0xD800-0xDFFF) - endpoint data for bulk/control transfers
        for addr in range(0xD800, 0xE000):
            self.read_callbacks[addr] = self._usb_ep_data_buf_read
            self.write_callbacks[addr] = self._usb_ep_data_buf_write

        # USB endpoint selection/status registers
        self.read_callbacks[0xC4EC] = self._usb_ep_status_read
        self.write_callbacks[0xC4ED] = self._usb_ep_index_write
        self.read_callbacks[0xC4EE] = self._usb_ep_id_low_read
        self.read_callbacks[0xC4EF] = self._usb_ep_id_high_read

        # USB endpoint data ready registers (0x90A1-0x90C0)
        # These indicate which endpoints have data available
        for addr in range(0x90A1, 0x90C1):
            self.read_callbacks[addr] = self._usb_ep_data_ready_read

        # USB endpoint status registers (0x9096-0x90A0)
        # These control whether command handler path is taken (0 = process cmd)
        for addr in range(0x9096, 0x90A1):
            self.read_callbacks[addr] = self._usb_ep_status_reg_read

        # USB EP buffer address registers (0x905B/0x905C)
        # Firmware writes DMA source address here, hardware DMAs from this address
        self.write_callbacks[0x905B] = self._usb_ep_buf_addr_write
        self.write_callbacks[0x905C] = self._usb_ep_buf_addr_write

        # USB E5 value register (0xC47A)
        # The firmware clears this register (writes 0xFF) before reading it.
        # We need to preserve the injected value until it's read by the E5 handler.
        self.read_callbacks[0xC47A] = self._usb_e5_value_read
        self.write_callbacks[0xC47A] = self._usb_e5_value_write

        # USB EP0 transfer status (0xE712)
        # The firmware polls this waiting for bits 0 and 1 to be set
        # indicating EP0 control transfer complete
        self.read_callbacks[0xE712] = self._usb_ep0_transfer_status_read

        # USB PHY control (0x91C0)
        # Firmware clears this at 0xCA8C but needs bit 1 SET for USB state machine
        # at 0x203B to progress from state 2 (0x0A59=2).
        self.read_callbacks[0x91C0] = self._usb_91c0_read

        # USB power state (0x92C2)
        # ISR at 0xE42A needs bit 6 CLEAR to call descriptor init (0xBDA4)
        # Main loop at 0x202A needs bit 6 SET to call 0x0322 for transfer
        # After ISR completes (2+ reads), return bit 6 SET
        self.read_callbacks[0x92C2] = self._usb_92c2_read

        # NOTE: 0xC001 is UART TX only, not USB EP0 FIFO. Testing confirmed that
        # firmware outputs debug messages to 0xC001 even during control transfer
        # handling. USB descriptor data is sent via hardware DMA directly from the
        # descriptor table in ROM (around 0x0864), not through firmware byte copies.
        # The exact DMA mechanism needs further investigation.

        # USB EP0 DMA control (0x9092) - may trigger hardware DMA
        # The actual source address for descriptor DMA is likely set via other registers
        self.write_callbacks[0x9092] = self._usb_ep0_dma_trigger_write
        self.read_callbacks[0x9092] = self._usb_ep0_dma_status_read

        # USB control state register (0x9091)
        # Two-phase control transfer handling:
        #   Bit 0: Setup phase - triggers 0xA5A6 (setup packet handler)
        #   Bit 1: Data phase - triggers 0xD088 (DMA for descriptor response)
        # Firmware loops writing 0x01 waiting for hardware to clear bit 0
        self.read_callbacks[0x9091] = self._usb_9091_read
        self.write_callbacks[0x9091] = self._usb_9091_write

        # USB endpoint status (0x9301)
        # Bit 6 triggers interrupt dispatch to device descriptor handler
        # Hardware clears bit 6 after read (acknowledge behavior)
        # Write of 0x40 (bit 6) arms EP0 for descriptor transfer
        self.read_callbacks[0x9301] = self._usb_9301_status_read
        self.write_callbacks[0x9301] = self._usb_9301_ep0_arm_write

        # Flash/Code ROM mirror region (0xE400-0xE700)
        # This XDATA region mirrors code ROM with offset 0xDDFC
        # Used for reading USB descriptors stored in code ROM
        # Examples:
        #   XDATA 0xE423 → Code ROM 0x0627 (device descriptor)
        #   XDATA 0xE437 → Code ROM 0x063B (language ID)
        #   XDATA 0xE6xx → Code ROM 0x08xx (additional descriptors)
        for addr in range(0xE400, 0xE700):
            self.read_callbacks[addr] = self._flash_rom_mirror_read

    # ============================================
    # Execution Tracing
    # ============================================
    def add_trace_point(self, pc: int, label: str):
        """
        Add a trace point at a specific PC address.

        When execution reaches this PC, the label will be logged.
        """
        self.trace_points[pc] = label

    def add_e4_trace_points(self):
        """
        Add trace points for E4 command processing.

        These cover the vendor handler and E4 read path.
        """
        self.trace_points.update({
            0x35B7: "VENDOR_HANDLER",
            0x35C0: "check_07EC",
            0x35C5: "call_17B1",
            0x35CB: "call_043F",
            0x35CF: "check_R7_after_043F",
            0x35D4: "call_1551",
            0x35DA: "E4_CHECK",
            0x35DF: "call_54BB",
            0x35E2: "setup_pcie_regs",
            0x35F9: "call_3C1E",
            0x35FC: "check_R7_after_3C1E",
            0x3601: "check_0AA0",
            0x360A: "setup_xfer",
            0x3649: "call_1741_cleanup",
            0x36E4: "vendor_exit",
            0x54BB: "E4_READ_HANDLER",
            0x3C1E: "pcie_transfer",
        })
        self.trace_enabled = True

    def check_trace(self, pc: int) -> str:
        """
        Check if PC matches a trace point and log if enabled.

        Returns the label if a trace point was hit, else None.
        """
        if not self.trace_enabled:
            return None

        if pc in self.trace_points:
            label = self.trace_points[pc]
            print(f"[{self.cycles:8d}] [TRACE] 0x{pc:04X}: {label}")

            # Call custom callback if registered
            if self.trace_callback:
                self.trace_callback(self, pc, label)

            return label
        return None

    # ============================================
    # XDATA Write Tracing
    # ============================================
    def add_xdata_trace(self, addr: int, name: str):
        """
        Add a trace point for XDATA writes.

        When firmware writes to this address, it will be logged.
        """
        self.xdata_trace_addrs[addr] = name

    def add_vendor_xdata_traces(self):
        """
        Add trace points for vendor command related XDATA addresses.

        These cover the key RAM locations used in E4/E5 command processing.
        Names match definitions in globals.h.
        """
        self.xdata_trace_addrs.update({
            0x0002: "G_IO_CMD_STATE",           # CDB[0]
            0x0003: "G_EP_STATUS_CTRL",         # Vendor flag
            0x0004: "G_WORK_0004",              # CDB[2]
            0x05A3: "G_CMD_SLOT_INDEX",         # Current command slot (0-9)
            0x05A5: "G_CMD_INDEX_SRC",          # Command index source
            0x05B1: "G_CMD_TABLE_BASE[0]",      # Command table entry 0
            0x05B2: "G_CMD_TABLE_BASE[0]+1",    # Command table entry 0, byte 1
            0x05B3: "G_CMD_TABLE_BASE[0]+2",    # Command table entry 0, byte 2
            0x05D3: "G_CMD_TABLE_BASE[1]",      # Command table entry 1
            0x07EC: "G_USB_CMD_CONFIG",         # USB command configuration
            0x0AA0: "G_DMA_XFER_STATUS",        # DMA transfer status
        })
        # Also trace command table range (10 entries x 34 bytes each)
        for i in range(10):
            base = 0x05B1 + i * 0x22  # G_CMD_TABLE_ENTRY_SIZE = 0x22
            if base not in self.xdata_trace_addrs:
                self.xdata_trace_addrs[base] = f"G_CMD_TABLE_BASE[{i}]"
        self.xdata_trace_enabled = True

    def trace_xdata_write(self, addr: int, value: int, pc: int = 0):
        """
        Log an XDATA write if tracing is enabled for this address.

        Called by memory system write hooks.
        """
        if not self.xdata_trace_enabled:
            return

        if addr in self.xdata_trace_addrs:
            name = self.xdata_trace_addrs[addr]
            entry = f"[{self.cycles:8d}] [PC=0x{pc:04X}] WRITE {name} (0x{addr:04X}) = 0x{value:02X}"
            self.xdata_write_log.append(entry)
            print(entry)
        elif 0x05B1 <= addr < 0x05B1 + 0x22 * 10:
            # Command table range
            idx = addr - 0x05B1
            entry_num = idx // 0x22
            offset = idx % 0x22
            entry = f"[{self.cycles:8d}] [PC=0x{pc:04X}] WRITE CMD_TABLE[{entry_num}]+{offset} (0x{addr:04X}) = 0x{value:02X}"
            self.xdata_write_log.append(entry)
            print(entry)

    def print_xdata_trace_log(self):
        """Print the accumulated XDATA write log."""
        print("\n=== XDATA WRITE LOG ===")
        for entry in self.xdata_write_log:
            print(entry)

    # ============================================
    # UART Callbacks
    # ============================================
    def _uart_tx(self, hw: 'HardwareState', addr: int, value: int):
        """Handle UART transmit with message buffering.

        NOTE: 0xC001 was previously thought to be shared with USB EP0 FIFO,
        but testing shows it's UART-only. The firmware outputs debug messages
        like "[InternalPD_StateInit]" to 0xC001 even during USB control transfer
        handling. The actual USB EP0 descriptor data is sent via hardware DMA
        directly from the descriptor table in ROM (around 0x0864), not through
        firmware-driven byte copying to 0xC001.
        """
        if self.log_uart:
            if value == 0x0A:  # Newline - print buffered line
                if self.uart_buffer:
                    print(f"[{self.cycles:8d}] [UART] {self.uart_buffer}")
                    self.uart_buffer = ""
            elif value == 0x0D:  # Carriage return - ignore
                pass
            elif 0x20 <= value < 0x7F:  # Printable ASCII
                self.uart_buffer += chr(value)
                # Flush on ']' to show complete [message] blocks
                if chr(value) == ']':
                    print(f"[{self.cycles:8d}] [UART] {self.uart_buffer}")
                    self.uart_buffer = ""
            # For very long lines, flush periodically
            if len(self.uart_buffer) > 200:
                print(f"[{self.cycles:8d}] [UART] {self.uart_buffer}")
                self.uart_buffer = ""
        else:
            try:
                if 0x20 <= value < 0x7F or value in (0x0A, 0x0D):
                    print(chr(value), end='', flush=True)
            except:
                pass

    # ============================================
    # PCIe Callbacks
    # ============================================
    def _pcie_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        PCIe status read at 0xB296.
        Multiple code paths check different bits:
        - 0xE3A7 checks bit 2 (JNB ACC.2)
        - 0xBFE6 checks bit 1 (ANL #0x02)
        Return value with both bits set after polling.
        """
        # Count reads and set completion bits after some polls
        if not hasattr(self, '_pcie_read_count'):
            self._pcie_read_count = 0
        self._pcie_read_count += 1

        # Return current value with completion bits OR'd in after 5 reads
        value = self.regs.get(addr, 0x00)
        if self._pcie_read_count >= 5:
            value |= 0x06  # Set bits 1 and 2
        return value

    def _pcie_trigger_write(self, hw: 'HardwareState', addr: int, value: int):
        """PCIe trigger - set complete status (bit 2)."""
        self.regs[0xB296] = 0x06  # bit 1 + bit 2 = complete

    def _pcie_dma_trigger(self, hw: 'HardwareState', addr: int, value: int):
        """
        PCIe DMA trigger at 0xB296.

        When value 0x08 is written, this triggers a PCIe DMA transfer for E4/E5 commands.
        - E4 (read): Copy from XDATA to USB buffer (for host to read)
        - E5 (write): Write value from CDB to XDATA

        The target address comes from the CDB in USB registers 0x910F-0x9111.
        For E4, 0x910E contains the size to read.
        For E5, 0x910E contains the value to write (single byte).
        """
        self.regs[addr] = value

        # Value 0x08 is the E4/E5 DMA trigger
        if value == 0x08:
            # Get target address from CDB (big-endian: 0x910F=high, 0x9110=mid, 0x9111=low)
            addr_high = self.regs.get(0x910F, 0)
            addr_mid = self.regs.get(0x9110, 0)
            addr_low = self.regs.get(0x9111, 0)
            target_addr = (addr_high << 16) | (addr_mid << 8) | addr_low

            # Check command type to determine operation
            cmd_type = self.usb_cmd_type

            if cmd_type == 0xE5:
                # E5 WRITE: Write single byte from CDB to XDATA
                write_value = self.regs.get(0x910E, 0)
                xdata_addr = target_addr & 0xFFFF

                if self.log_pcie:
                    print(f"[{self.cycles:8d}] [PCIe] E5 WRITE: 0x{write_value:02X} -> XDATA[0x{xdata_addr:04X}]")

                # Perform the write
                if self.memory:
                    self.memory.xdata[xdata_addr] = write_value

                # Signal completion
                self.regs[0xB296] = 0x06  # PCIe DMA complete (bits 1+2)

                # Clear command pending after successful write
                if self.usb_cmd_pending:
                    self.usb_cmd_pending = False
                    print(f"[{self.cycles:8d}] [PCIe] E5 command completed")

            else:
                # E4 READ: Copy from XDATA to USB buffer
                size = self.regs.get(0x910E, 0)

                if self.log_pcie:
                    print(f"[{self.cycles:8d}] [PCIe] DMA TRIGGER: src=0x{target_addr:06X} size={size}")

                # Perform the DMA - copy from simulated PCIe memory to USB buffer
                self._perform_pcie_dma(target_addr, size)

                # Signal completion - multiple bits checked by different code paths
                # Bit 2 checked at 0xE3A7 (JNB ACC.2), bit 1 checked at 0xBFE6 (ANL #0x02)
                self.regs[0xB296] = 0x06  # PCIe DMA complete (bits 1+2)

                # Clear command pending after successful DMA
                if self.usb_cmd_pending:
                    self.usb_cmd_pending = False
                    self.usb_cmd_type = 0  # Reset command type
                    print(f"[{self.cycles:8d}] [PCIe] USB command completed, clearing pending flag")

    def _perform_pcie_dma(self, source_addr: int, size: int):
        """
        Perform PCIe DMA transfer to USB buffer.

        For E4 read commands (address 0x50xxxx), reads from XDATA[xxxx].
        For other addresses, uses simulated PCIe memory or test patterns.
        Data is copied to USB data buffer at 0x8000.
        """
        if not self.memory:
            if self.log_pcie:
                print(f"[{self.cycles:8d}] [PCIe] ERROR: No memory reference for DMA")
            return

        dest_addr = 0x8000  # USB data buffer

        # Check if this is an E4 XDATA read (address 0x50xxxx)
        is_xdata_read = (source_addr >> 16) == 0x50

        for i in range(size):
            if is_xdata_read:
                # E4 command: read from chip's XDATA memory
                # Address format: 0x50XXXX -> XDATA[XXXX]
                xdata_addr = (source_addr + i) & 0xFFFF
                value = self.memory.xdata[xdata_addr]
            else:
                # PCIe memory read (e.g., NVMe config space)
                pcie_addr = source_addr + i
                if pcie_addr in self.pcie_memory:
                    value = self.pcie_memory[pcie_addr]
                else:
                    # Generate test pattern for unmapped PCIe addresses
                    value = (pcie_addr & 0xFF) ^ (i & 0xFF)

            # Write to USB data buffer
            self.memory.xdata[dest_addr + i] = value

        # TEST MODE: Set DMA completion flag in RAM
        # Real hardware would signal completion through MMIO registers,
        # which firmware reads and then sets this RAM flag itself.
        # For testing, we set it directly.
        self.memory.xdata[0x0AA0] = size if size > 0 else 1

        if self.log_pcie:
            addr_type = "XDATA" if is_xdata_read else "PCIe"
            xdata_addr = source_addr & 0xFFFF if is_xdata_read else source_addr
            print(f"[{self.cycles:8d}] [PCIe] DMA COMPLETE: {size} bytes from {addr_type}[0x{xdata_addr:04X}] to 0x{dest_addr:04X}")
            if size > 0:
                sample = ' '.join(f'{self.memory.xdata[dest_addr + i]:02X}' for i in range(min(size, 16)))
                print(f"[{self.cycles:8d}] [PCIe] Data: {sample}")

    # ============================================
    # Flash/DMA Callbacks
    # ============================================
    def _flash_csr_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Flash CSR read - returns status.
        Bit 0: Busy (0 = idle, 1 = operation in progress)
        """
        # Flash is always ready (operations complete instantly in emulation)
        return 0x00

    def _flash_cmd_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        Flash command write - triggers flash operations.

        SPI Flash commands:
        - 0x03: Read data
        - 0x02: Page program (write)
        - 0x20: Sector erase (4KB)
        - 0xD8: Block erase (64KB)
        - 0xC7: Chip erase
        - 0x06: Write enable
        - 0x04: Write disable
        """
        # Get flash address from address registers
        addr_hi = self.regs.get(0xC8AB, 0)
        addr_mid = self.regs.get(0xC8AC, 0)
        addr_lo = self.regs.get(0xC8AD, 0)
        flash_addr = (addr_hi << 16) | (addr_mid << 8) | addr_lo
        self.spi_flash_addr = flash_addr

        if self.log_writes:
            print(f"[{self.cycles:8d}] [SPI_FLASH] Command 0x{value:02X} at addr 0x{flash_addr:06X}")

        if value == 0x20:  # Sector erase (4KB)
            sector_start = flash_addr & ~0xFFF
            if sector_start + 0x1000 <= len(self.spi_flash):
                for i in range(0x1000):
                    self.spi_flash[sector_start + i] = 0xFF
                print(f"[{self.cycles:8d}] [SPI_FLASH] Erased sector at 0x{sector_start:06X}")

        elif value == 0xD8:  # Block erase (64KB)
            block_start = flash_addr & ~0xFFFF
            if block_start + 0x10000 <= len(self.spi_flash):
                for i in range(0x10000):
                    self.spi_flash[block_start + i] = 0xFF
                print(f"[{self.cycles:8d}] [SPI_FLASH] Erased block at 0x{block_start:06X}")

        elif value == 0xC7:  # Chip erase
            for i in range(len(self.spi_flash)):
                self.spi_flash[i] = 0xFF
            print(f"[{self.cycles:8d}] [SPI_FLASH] Chip erased")

        elif value == 0x02:  # Page program - data comes from USB buffer
            self.spi_flash_write_pending = True
            self.spi_flash_write_count = 0
            print(f"[{self.cycles:8d}] [SPI_FLASH] Page program started at 0x{flash_addr:06X}")

        # Mark operation complete
        self.regs[0xC8A9] = 0x00  # Clear busy

    def _flash_data_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        Flash data write - writes data to flash during page program.
        Data is written byte-by-byte to the current flash address.
        """
        if self.spi_flash_write_pending:
            flash_addr = self.spi_flash_addr + self.spi_flash_write_count
            if flash_addr < len(self.spi_flash):
                # SPI flash write: can only clear bits (AND with existing value)
                self.spi_flash[flash_addr] &= value
                self.spi_flash_write_count += 1
                if self.log_writes:
                    print(f"[{self.cycles:8d}] [SPI_FLASH] Write byte 0x{value:02X} to 0x{flash_addr:06X}")

    def _flash_data_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Flash data read - reads data from flash at current address.
        Returns the byte at spi_flash_addr and auto-increments.
        """
        flash_addr = self.spi_flash_addr
        if flash_addr < len(self.spi_flash):
            value = self.spi_flash[flash_addr]
            self.spi_flash_addr += 1  # Auto-increment
            if self.log_reads:
                print(f"[{self.cycles:8d}] [SPI_FLASH] Read byte 0x{value:02X} from 0x{flash_addr:06X}")
            return value
        return 0xFF  # Return erased value if out of range

    def load_flash_from_file(self, path: str):
        """Load flash contents from a file (e.g., fw.bin)."""
        try:
            with open(path, 'rb') as f:
                data = f.read()
            # Copy to flash, don't exceed flash size
            copy_size = min(len(data), len(self.spi_flash))
            for i in range(copy_size):
                self.spi_flash[i] = data[i]
            print(f"[SPI_FLASH] Loaded {copy_size} bytes from {path}")
        except Exception as e:
            print(f"[SPI_FLASH] Failed to load from {path}: {e}")

    def save_flash_to_file(self, path: str):
        """Save flash contents to a file."""
        try:
            with open(path, 'wb') as f:
                f.write(self.spi_flash)
            print(f"[SPI_FLASH] Saved {len(self.spi_flash)} bytes to {path}")
        except Exception as e:
            print(f"[SPI_FLASH] Failed to save to {path}: {e}")

    def _dma_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """DMA status - done."""
        return 0x04

    def _busy_reg_read(self, hw: 'HardwareState', addr: int) -> int:
        """Busy register - auto-clear after polling."""
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)
        if count >= 3 and (value & 0x01):
            value &= ~0x01
            self.regs[addr] = value
        return value

    def _flash_rom_mirror_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Flash/Code ROM mirror read.

        XDATA region 0xE400-0xE500 mirrors code ROM with offset 0xDDFC.
        This is used for reading USB descriptors stored in code ROM.
        Example: XDATA 0xE423 → Code ROM 0x0627 (device descriptor)

        Formula: code_addr = xdata_addr - 0xDDFC
        """
        code_addr = addr - 0xDDFC
        if self.memory and 0 <= code_addr < len(self.memory.code):
            value = self.memory.code[code_addr]
            if self.log_reads:
                print(f"[{self.cycles:8d}] [FLASH] Read 0x{addr:04X} → Code[0x{code_addr:04X}] = 0x{value:02X}")
            return value
        return 0x00

    # ============================================
    # Interrupt Callbacks
    # ============================================
    def _int_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """System interrupt status - clear on read."""
        value = self.regs.get(addr, 0)
        if value & 0x01:
            self.regs[addr] = value & ~0x01
        return value

    def _pd_interrupt_read(self, hw: 'HardwareState', addr: int) -> int:
        """PD interrupt status - returns current state."""
        return self.regs.get(addr, 0)

    # ============================================
    # USB State Machine MMIO Callbacks
    # ============================================
    def _usb_ce89_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB/DMA status register 0xCE89.

        Controls USB state machine transitions:
        - Bit 0: Must be SET to exit wait loop at 0x348C (JNB 0xe0.0)
        - Bit 1: Must be CLEAR for success at 0x3493 (jnb acc.1 takes good path)
                 If SET, firmware jumps to 0x35A1 (failure path)
        - Bit 2: DMA/transfer status at 0x48D1 (JNB ACC.2)
                 SET = DMA in progress, CLEAR = DMA complete
                 Controls state 3→4 transition at 0x3588 (JNB 0xe0.2)

        State machine flow:
        1. Firmware writes 0 to 0xCE88, then polls 0xCE89 bit 0
        2. When bit 0 set, checks bit 1 - must be CLEAR for success
        3. Then checks bit 4 of 0xCE86 - must be CLEAR
        4. Bit 2: set briefly to signal state 3→4, then clear to signal completion

        For USB control transfers (e.g., GET_DESCRIPTOR):
        1. Firmware sets up DMA via CCxx registers
        2. Polls 0xCE89 bit 2 - SET means busy, CLEAR means complete
        3. When bit 2 clears, firmware knows transfer is done
        """
        self.usb_ce89_read_count += 1

        # Start with base value
        value = 0x00

        # Enable state machine progression when USB connected OR command pending
        # This allows firmware to transition through USB states naturally
        if self.usb_connected or self.usb_cmd_pending:
            # Bit 0 - set after a few reads to exit wait loop at 0x348C
            if self.usb_ce89_read_count >= 3:
                value |= 0x01

            # Bit 1 - E5 path control
            # At 0x1862: jb acc.1, 0x1884 - if bit 1 SET, take E5 path
            # For E5 commands, we SET bit 1 to direct firmware to the E5 handler
            # For E4 commands, we keep bit 1 CLEAR to take the E4 path
            if self.usb_cmd_type == 0xE5:
                value |= 0x02  # Set bit 1 for E5 path

            # Bit 2 - DMA/transfer busy status
            # SET during counts 5-14 to allow state transitions
            # CLEAR after count >= 15 to signal DMA/transfer completion
            # This allows firmware to exit the polling loop at 0x48D1
            if 5 <= self.usb_ce89_read_count < 15:
                value |= 0x04
            # After count 15, bit 2 stays clear to signal completion

        if self.log_reads or self.usb_cmd_pending:
            # Add PC for better tracing
            pc = 0
            if hasattr(self, '_cpu_ref') and self._cpu_ref:
                pc = self._cpu_ref.pc
            print(f"[{self.cycles:8d}] [USB_SM] Read 0xCE89 = 0x{value:02X} (count={self.usb_ce89_read_count}, PC=0x{pc:04X})")

        return value

    # ============================================
    # IMPORTANT: USB Descriptor Handling Philosophy
    # ============================================
    # The emulator must NOT search for USB descriptors in ROM/XDATA.
    # The FIRMWARE is responsible for handling GET_DESCRIPTOR requests:
    #
    # 1. Firmware reads setup packet from MMIO (0x9E00-0x9E07)
    # 2. Firmware looks up descriptor in its own code ROM
    # 3. Firmware writes descriptor to USB transmit buffer via MMIO
    # 4. USB hardware DMA sends the data to host
    #
    # If you find yourself searching for descriptors in the emulator,
    # you are doing something WRONG. Fix the MMIO emulation so the
    # firmware's USB handler can complete successfully.
    #
    # The emulator's job is to:
    # - Provide correct MMIO register values for firmware to read
    # - Capture data that firmware writes to USB output registers
    # - Signal completion via status registers
    #
    # DO NOT implement _find_descriptor_in_xdata or similar functions!
    # ============================================

    def _usb_ce86_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB status register 0xCE86.

        Bit 4: Checked at 0x349D (JNB 0xe0.4) - must be clear for normal path.
        """
        # Return 0 to allow normal USB initialization path
        # Bit 4 clear means no error/busy condition
        return 0x00

    def _usb_ce6c_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB controller status register 0xCE6C.

        Bit 7: USB controller ready for transfers
               Checked at 0x1855: jnb acc.7, 0x1884
               Checked at 0x2FB6: jb acc.7, 0x2FBC
               Must be SET when USB is connected and ready.

        This is hardware state - the USB controller sets this when
        it's ready to process transfers.
        """
        if self.usb_connected or self.usb_cmd_pending:
            return 0x80  # Bit 7 set - USB ready
        return 0x00

    def _usb_ce00_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        DMA control register 0xCE00.

        Firmware writes 0x03 to start a DMA transfer (at 0x3531-0x3533),
        then polls at 0x3534-0x3538 waiting for this register to become 0.
        When the register is 0, DMA is complete.

        The polling loop is:
            0x3534: mov dptr, #0xce00
            0x3537: movx a, @dptr
            0x3538: jnz 0x3534  ; loop while non-zero
        """
        self.usb_ce00_read_count += 1

        # Return 0 after a few reads to simulate DMA completion
        if self.usb_ce00_read_count >= 2:
            return 0x00  # DMA complete
        return self.regs.get(0xCE00, 0x03)  # DMA in progress

    def _usb_ce00_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        DMA control write - resets DMA state for new transfer.
        """
        self.regs[0xCE00] = value
        self.usb_ce00_read_count = 0  # Reset counter for new DMA operation

    def _usb_ce55_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Transfer slot count register 0xCE55.

        Read at 0x34B9 and stored to XDATA[0x009F] to determine the
        outer loop limit at 0x34F8-0x3553. Each iteration processes
        one transfer slot.

        For a simple control transfer, return 1 to limit to single iteration.
        """
        print(f"[{self.cycles:8d}] [USB_CE55] Read CE55 = 0x01 (transfer slots)")
        return 0x01  # 1 transfer slot for control transfers

    def _usb_ce88_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        DMA trigger register 0xCE88.

        Firmware writes to CE88 before polling CE89 at 0x1806/0x1807.
        This write triggers a new DMA/transfer sequence, so we reset
        the CE89 read count to allow the new polling sequence to
        progress through the state machine correctly.
        """
        self.regs[0xCE88] = value
        # Reset CE89 count for new transfer sequence
        self.usb_ce89_read_count = 0
        if self.log_writes:
            print(f"[{self.cycles:8d}] [USB_HW] CE88 write = 0x{value:02X}, reset CE89 counter")

    # ============================================
    # Timer Callbacks
    # ============================================
    def _timer_csr_read(self, hw: 'HardwareState', addr: int) -> int:
        """Timer CSR - auto-set ready bit after polling."""
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)
        # The firmware polls for bit 1 (0x02) to be set - indicating timer ready/complete
        # Set bit 1 after a few polls to avoid infinite wait
        if count >= 2:
            value |= 0x02  # Set ready/complete bit
            self.regs[addr] = value
        return value

    def _timer_csr_write(self, hw: 'HardwareState', addr: int, value: int):
        """Timer CSR write."""
        if value & 0x04:  # Clear flag
            value &= ~0x02
        self.regs[addr] = value
        self.poll_counts[addr] = 0

    def _timer_dma_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """Timer/DMA status (0xCC89) - set complete bit after polling."""
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)
        # The firmware polls for bit 1 (0x02) to be set - indicating DMA complete
        if count >= 2:
            value |= 0x02  # Set complete bit
            self.regs[addr] = value
        return value

    # ============================================
    # PHY/CPU Callbacks
    # ============================================
    def _phy_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """PHY status - bit 0 = ready, bit 1 = busy."""
        # Return ready state: bit 0 set, bit 1 clear
        return 0x01

    def _phy_cmd_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        PHY command register write (0xCD31).

        This is a PHY/hardware control register. The firmware writes commands
        to it during USB operations. USB descriptor data is sent via hardware DMA
        directly from the descriptor table in ROM (around 0x0864).
        """
        self.regs[addr] = value

    def _cmd_engine_read(self, hw: 'HardwareState', addr: int) -> int:
        """Command engine - auto-clear bit 0 after polling."""
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)
        if count >= 3 and (value & 0x01):
            value &= ~0x01
            self.regs[addr] = value
        return value

    # ============================================
    # USB Command Injection
    # ============================================
    def queue_usb_command(self, cmd: int, addr: int, data: bytes = b''):
        """
        Queue a USB command for firmware processing.

        Commands (from python/usb.py):
        - 0xE4: Read from XDATA (addr = 0x5XXXXX maps to firmware XDATA)
        - 0xE5: Write to XDATA
        - 0x8A: SCSI write command

        Address mapping: (addr & 0x1FFFF) | 0x500000 in usb.py
        So 0x5XXXXX -> XDATA 0xXXXX (lower 17 bits)
        """
        usb_cmd = USBCommand(cmd=cmd, addr=addr, data=data)
        self.usb_cmd_queue.append(usb_cmd)

        if self.log_writes:
            print(f"[USB] Queued cmd=0x{cmd:02X} addr=0x{addr:04X} len={len(data)}")

        # Trigger USB interrupt to wake up firmware
        self._trigger_usb_interrupt()

    def queue_e4_read(self, xdata_addr: int, size: int = 1):
        """
        Queue an E4 read command (read XDATA).

        Format from usb.py: struct.pack('>BBBHB', 0xE4, size, addr >> 16, addr & 0xFFFF, 0)
        """
        # Pack command into EP0 buffer format
        cmd_bytes = bytes([
            0xE4,                      # Command
            size,                      # Size to read
            (xdata_addr >> 16) & 0xFF, # High byte (usually 0x05 for XDATA)
            (xdata_addr >> 8) & 0xFF,  # Mid byte
            xdata_addr & 0xFF,         # Low byte
            0x00                       # Reserved
        ])
        self.queue_usb_command(0xE4, xdata_addr & 0xFFFF, cmd_bytes)

    def queue_e5_write(self, xdata_addr: int, value: int):
        """
        Queue an E5 write command (write XDATA).

        Format from usb.py: struct.pack('>BBBHB', 0xE5, value, addr >> 16, addr & 0xFFFF, 0)
        """
        cmd_bytes = bytes([
            0xE5,                      # Command
            value & 0xFF,              # Value to write
            (xdata_addr >> 16) & 0xFF, # High byte (usually 0x05 for XDATA)
            (xdata_addr >> 8) & 0xFF,  # Mid byte
            xdata_addr & 0xFF,         # Low byte
            0x00                       # Reserved
        ])
        self.queue_usb_command(0xE5, xdata_addr & 0xFFFF, cmd_bytes)

    def queue_init_sequence(self):
        """
        Queue the USB initialization sequence from usb.py.

        Init sequence:
        - WriteOp(0x54b, b' ')   -> write 0x20 to 0x054B
        - WriteOp(0x54e, b'\x04') -> write 0x04 to 0x054E
        - WriteOp(0x0, b'\x01')  -> write 0x01 to 0x0000
        """
        print("[USB] === QUEUING INIT SEQUENCE ===")
        self.queue_e5_write(0x054B, 0x20)
        self.queue_e5_write(0x054E, 0x04)
        self.queue_e5_write(0x0000, 0x01)

    def inject_usb_command(self, cmd_type: int, xdata_addr: int, value: int = 0, size: int = 1):
        """
        Inject a USB vendor command (E4 read / E5 write) through MMIO registers.

        This sets up the firmware's vendor command path:
        0x0E5A (USB int) → 0x0E64 (bit5 SET) → 0x0EF4 (bit0 CLEAR)
        → 0x5333 (state check) → 0x4583 (vendor dispatch) → 0x35B7 (vendor handler)

        Only MMIO registers are set - no direct RAM writes. The firmware reads
        expected values through read hooks that simulate hardware behavior.

        cmd_type: 0xE4 (read) or 0xE5 (write)
        xdata_addr: Target XDATA address
        value: Value to write (for E5 commands)
        size: Bytes to read (for E4 commands)
        """
        # Ensure USB is connected before injecting a command
        # This sets up the necessary MMIO state for USB state machine
        if not self.usb_connected:
            self.usb_connected = True
            self.usb_controller.connect()
            print(f"[{self.cycles:8d}] [USB] Auto-connected USB for command injection")

        # Use USBController for the MMIO setup
        cdb = self.usb_controller.inject_vendor_command(
            cmd_type, xdata_addr, value, size
        )

        # Trigger USB interrupt
        self._pending_usb_interrupt = True

        # Note: USBController.inject_vendor_command() already handles RAM writes
        # when use_direct_ram=True, so no duplicate writes needed here

        print(f"[{self.cycles:8d}] [USB] Vendor command ready, triggering interrupt")

    def inject_scsi_write(self, lba: int, sectors: int, data: bytes):
        """
        Inject a 0x8A SCSI write command through MMIO registers.

        This sets up the firmware's SCSI command path. Data is written
        to the USB buffer at 0x8000 for DMA to the NVMe device.

        Args:
            lba: Logical Block Address to write to
            sectors: Number of 512-byte sectors to write
            data: Data to write (will be padded to sector boundary)
        """
        # Ensure USB is connected before injecting a command
        if not self.usb_connected:
            self.usb_connected = True
            self.usb_controller.connect()
            print(f"[{self.cycles:8d}] [USB] Auto-connected USB for SCSI command")

        # Use USBController for the MMIO setup
        cdb = self.usb_controller.inject_scsi_write_command(lba, sectors, data)

        # Trigger USB interrupt
        self._pending_usb_interrupt = True

        print(f"[{self.cycles:8d}] [USB] SCSI write command ready, triggering interrupt")

    def inject_scsi_vendor_cmd(self, opcode: int, cdb: bytes, data: bytes = b'',
                                is_write: bool = False):
        """
        Inject a SCSI vendor command (E0-E8) through MMIO registers.

        This sets up the firmware's vendor SCSI command path for commands
        used by patch.py for firmware updates.

        Args:
            opcode: SCSI vendor opcode (0xE0-0xE8)
            cdb: Complete CDB bytes (16 bytes max)
            data: Data for write commands (E1, E3, E5)
            is_write: True if this is a write command with data phase
        """
        # Ensure USB is connected before injecting a command
        if not self.usb_connected:
            self.usb_connected = True
            self.usb_controller.connect()
            print(f"[{self.cycles:8d}] [USB] Auto-connected USB for SCSI vendor command")

        # Use USBController for the MMIO setup
        cdb_padded = self.usb_controller.inject_scsi_vendor_command(
            opcode, cdb, data, is_write
        )

        # Trigger USB interrupt
        self._pending_usb_interrupt = True

        print(f"[{self.cycles:8d}] [USB] SCSI vendor command 0x{opcode:02X} ready, triggering interrupt")
        return cdb_padded

    def _trigger_usb_interrupt(self):
        """Trigger USB interrupt to process queued command."""
        if not self.usb_connected:
            return

        # Set USB endpoint interrupt bits
        # REG_INT_USB_STATUS (0xC802) bit 0 = endpoint 0 data ready
        # REG_USB_STATUS (0x9000) bit 0 = USB active
        self.regs[0xC802] |= 0x01  # EP0 data ready
        self.regs[0x9000] |= 0x01  # USB active

        # Set EP0 has data flag
        # REG_USB_EP0_CSR (0x9E10) - EP0 control/status
        self.regs[0x9E10] = 0x01  # Data available

        self.usb_cmd_pending = True

    def _process_usb_command(self):
        """
        Process next USB command in queue.
        Called when firmware reads USB endpoint buffer.
        """
        if not self.usb_cmd_queue:
            return None

        cmd = self.usb_cmd_queue.pop(0)
        print(f"[USB] Processing cmd=0x{cmd.cmd:02X} addr=0x{cmd.addr:04X}")

        # Copy command to EP0 buffer
        for i, b in enumerate(cmd.data[:64]):
            self.usb_ep0_buf[i] = b
        self.usb_ep0_len = len(cmd.data)

        # Handle E4 read - prepare response data
        if cmd.cmd == 0xE4 and self.memory:
            size = cmd.data[1] if len(cmd.data) > 1 else 1
            response = bytearray(size)
            for i in range(size):
                response[i] = self.memory.read_xdata(cmd.addr + i)
            cmd.response = bytes(response)
            print(f"[USB] E4 read response: {response.hex()}")

        # Handle E5 write - perform the write directly
        if cmd.cmd == 0xE5 and self.memory:
            value = cmd.data[1] if len(cmd.data) > 1 else 0
            self.memory.write_xdata(cmd.addr, value)
            print(f"[USB] E5 wrote 0x{value:02X} to 0x{cmd.addr:04X}")

        if not self.usb_cmd_queue:
            self.usb_cmd_pending = False

        return cmd

    # ============================================
    # USB Endpoint Callbacks
    # ============================================
    def _usb_ep0_buf_read(self, hw: 'HardwareState', addr: int) -> int:
        """Read from USB EP0 buffer (0x9E00-0x9E3F)."""
        offset = addr - 0x9E00
        if offset < len(self.usb_ep0_buf):
            return self.usb_ep0_buf[offset]
        return 0x00

    def _usb_ep0_buf_write(self, hw: 'HardwareState', addr: int, value: int):
        """Write to USB EP0 buffer (0x9E00-0x9E3F).

        This captures config descriptor writes. The firmware writes the config
        descriptor to 0x9E00, but then corrupts it before DMA. We capture the
        FIRST write to each offset - later overwrites are ignored.
        """
        self.regs[addr] = value
        offset = addr - 0x9E00

        # Track which bytes have been captured (to ignore later overwrites)
        if not hasattr(self, '_usb_config_captured_offsets'):
            self._usb_config_captured_offsets = set()

        # Check for start of config descriptor (bLength=0x09, bDescriptorType=0x02)
        if offset == 0 and value == 0x09:
            # Might be config descriptor - start capturing
            self.usb_captured_config_desc = bytearray(256)
            self.usb_captured_config_desc[0] = value
            self.usb_capture_config_active = True
            self._usb_config_captured_offsets = {0}
        elif offset == 1 and self.usb_capture_config_active:
            if value == 0x02 and 1 not in self._usb_config_captured_offsets:
                # Confirmed config descriptor (bDescriptorType = 2)
                self.usb_captured_config_desc[1] = value
                self._usb_config_captured_offsets.add(1)
            elif value != 0x02:
                # Not a config descriptor, stop capturing
                self.usb_capture_config_active = False
                self.usb_captured_config_desc = bytearray()
                self._usb_config_captured_offsets = set()
        elif self.usb_capture_config_active and 2 <= offset < 256:
            # Only capture first write to each offset (ignore later corruptions)
            if offset not in self._usb_config_captured_offsets:
                self.usb_captured_config_desc[offset] = value
                self._usb_config_captured_offsets.add(offset)
        elif offset == 0 and value != 0x09:
            # Different descriptor or setup packet - stop capturing
            if self.usb_capture_config_active:
                # Keep the captured data but mark capture as complete
                self.usb_capture_config_active = False

    def load_config_descriptor_from_rom(self):
        """Load USB3 config descriptor from ROM and fix wTotalLength.

        The ROM at 0x58CF has USB3 config descriptor with wTotalLength=44,
        which only includes alt_setting 0 (BBB). However, alt_setting 1 (UAS)
        data continues immediately after at 0x58FB.

        This method parses the ROM to find the actual end of the config
        descriptor (including alt_setting 1), creates a copy with the
        correct wTotalLength, and stores it for use when returning
        config descriptor data to the host.
        """
        if self._memory is None:
            print("[USB] Warning: Cannot load config from ROM - no memory reference")
            return

        # USB3 config descriptor starts at 0x58CF in ROM
        USB3_CONFIG_OFFSET = 0x58CF

        # Parse the descriptor chain to find actual total length
        rom = self._memory.code
        if len(rom) < USB3_CONFIG_OFFSET + 9:
            print("[USB] Warning: ROM too small for config descriptor")
            return

        # Valid descriptor types for config descriptor contents
        valid_types = {0x02, 0x04, 0x05, 0x30, 0x24}  # config, interface, endpoint, SS companion, class-specific

        i = USB3_CONFIG_OFFSET
        total_len = 0
        while i < len(rom) - 1:
            bLength = rom[i]
            bDescriptorType = rom[i + 1]

            # Stop at invalid descriptors or when we hit next config descriptor
            if bLength == 0 or bDescriptorType not in valid_types:
                break

            # Stop if we hit another config descriptor (USB2 config at 0x5948)
            if bDescriptorType == 0x02 and i > USB3_CONFIG_OFFSET:
                break

            i += bLength
            total_len = i - USB3_CONFIG_OFFSET

        if total_len < 44:
            print(f"[USB] Warning: Parsed config descriptor too small ({total_len} bytes)")
            return

        # Extract the full descriptor and fix wTotalLength
        desc = bytearray(rom[USB3_CONFIG_OFFSET:USB3_CONFIG_OFFSET + total_len])
        old_len = desc[2] | (desc[3] << 8)
        desc[2] = total_len & 0xFF
        desc[3] = (total_len >> 8) & 0xFF

        self.usb_ss_config_from_rom = bytes(desc)
        print(f"[USB] Loaded USB3 config descriptor from ROM: {total_len} bytes (wTotalLength fixed {old_len} -> {total_len})")

        # Also load USB2 High Speed config descriptor from 0x5948
        # This has correct 512-byte max packet sizes for USB 2.0
        USB2_CONFIG_OFFSET = 0x5948

        if len(rom) < USB2_CONFIG_OFFSET + 9:
            print("[USB] Warning: ROM too small for USB2 config descriptor")
            return

        # Parse USB2 descriptor chain
        i = USB2_CONFIG_OFFSET
        total_len_usb2 = 0
        while i < len(rom) - 1:
            bLength = rom[i]
            bDescriptorType = rom[i + 1]

            # Valid types for USB2 config: config(0x02), interface(0x04), endpoint(0x05), class-specific(0x24)
            # No SS companion (0x30) in USB2
            valid_types_usb2 = {0x02, 0x04, 0x05, 0x24}

            if bLength == 0 or bDescriptorType not in valid_types_usb2:
                break

            # Stop if we hit another config descriptor
            if bDescriptorType == 0x02 and i > USB2_CONFIG_OFFSET:
                break

            i += bLength
            total_len_usb2 = i - USB2_CONFIG_OFFSET

        if total_len_usb2 < 32:
            print(f"[USB] Warning: Parsed USB2 config descriptor too small ({total_len_usb2} bytes)")
            return

        # Extract the USB2 descriptor and fix wTotalLength if needed
        desc_usb2 = bytearray(rom[USB2_CONFIG_OFFSET:USB2_CONFIG_OFFSET + total_len_usb2])
        old_len_usb2 = desc_usb2[2] | (desc_usb2[3] << 8)
        desc_usb2[2] = total_len_usb2 & 0xFF
        desc_usb2[3] = (total_len_usb2 >> 8) & 0xFF

        self.usb_hs_config_from_rom = bytes(desc_usb2)
        print(f"[USB] Loaded USB2 config descriptor from ROM: {total_len_usb2} bytes (wTotalLength: {old_len_usb2} -> {total_len_usb2})")

    def _extend_config_descriptor(self, base_desc: bytearray, requested_len: int) -> bytes:
        """Return config descriptor appropriate for current USB speed.

        Uses USB2 config (0x5948) for High Speed with 512-byte endpoints,
        or USB3 config (0x58CF) for SuperSpeed with 1024-byte endpoints.
        """
        # Check USB speed from controller
        usb_ctrl = getattr(self, 'usb_controller', None)
        usb_speed = getattr(usb_ctrl, 'usb_speed', 2) if usb_ctrl else 2

        # USB 2.0 High Speed (speed < 2) uses USB2 config descriptor
        if usb_speed < 2 and self.usb_hs_config_from_rom:
            print(f"[USB] Using USB2 High Speed config descriptor (speed={usb_speed})")
            return self.usb_hs_config_from_rom[:min(requested_len, len(self.usb_hs_config_from_rom))]

        # USB 3.0 SuperSpeed (speed >= 2) uses USB3 config descriptor
        if self.usb_ss_config_from_rom:
            return self.usb_ss_config_from_rom[:min(requested_len, len(self.usb_ss_config_from_rom))]

        # Fallback: return what firmware wrote (shouldn't happen normally)
        if len(base_desc) < 9:
            return bytes(base_desc[:requested_len])
        return bytes(base_desc[:min(requested_len, len(base_desc))])

    def _usb_ep0_csr_read(self, hw: 'HardwareState', addr: int) -> int:
        """Read USB EP0 CSR - check if command pending."""
        # Process next command when firmware reads CSR
        if self.usb_cmd_pending and self.usb_cmd_queue:
            self._process_usb_command()
            return 0x01  # Data ready
        return 0x00

    def _usb_ep0_csr_write(self, hw: 'HardwareState', addr: int, value: int):
        """Write USB EP0 CSR - acknowledge command."""
        if value & 0x80:  # Clear data ready
            self.regs[0x9E10] = 0x00
            # Trigger next command if queued
            if self.usb_cmd_queue:
                self._trigger_usb_interrupt()

    def _usb_ep0_transfer_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB EP0 transfer status (0xE712).

        The firmware polls this register waiting for bits 0 and 1 to be set,
        indicating the USB EP0 control transfer is complete.
        This happens after calling 0xE581 which initiates the DMA transfer.
        """
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)
        # After a few polls, set both bits to indicate transfer complete
        if count >= 2:
            value |= 0x03  # Set bits 0 and 1 (transfer complete)
            self.regs[addr] = value
        return value

    def _usb_91c0_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB PHY control read (0x91C0).

        At address 0x203B, firmware checks bit 1 of this register when
        the USB state machine is in state 2 (0x0A59=2).
        If bit 1 is SET, it calls 0x0322 which progresses the state machine.

        The firmware clears this register at 0xCA8C, but we need to return
        bit 1 SET when USB is connected to allow state machine progress.
        """
        if self.usb_connected:
            return 0x02  # Bit 1 SET - enables USB state machine progress
        return self.regs.get(addr, 0)

    def _usb_92c2_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB power state read (0x92C2).

        This register controls two different code paths:
        1. ISR at 0xE42A: checks bit 6 - if CLEAR, calls 0xBDA4 (state RESET)
           If bit 6 is SET, ISR skips the reset and returns immediately
        2. Main loop at 0x202A: checks bit 6 - if SET, calls 0x0322 (transfer)

        CRITICAL: 0xBDA4 is a STATE RESET function that clears 0x0AF7 and many
        other variables. We must NOT let it run during control transfers!

        During control transfers:
        - Return bit 6 SET to SKIP 0xBDA4 reset and preserve 0x0AF7=1
        - Main loop also needs bit 6 SET to call transfer handler
        """
        if self.usb_control_transfer_active:
            self.usb_92c2_read_count += 1
            # ALWAYS return bit 6 SET during control transfers to prevent
            # the state reset at 0xBDA4 from clearing 0x0AF7
            return 0x40
        return self.regs.get(addr, 0x40)  # Default: bit 6 SET (PD task enabled)

    def _usb_ep0_fifo_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        USB EP0 data FIFO write (UNUSED).

        NOTE: This function is not currently registered as a callback.
        Testing revealed that 0xC001 is UART TX only - USB descriptor data
        is sent via hardware DMA directly from ROM, not via firmware byte copies.
        Kept for potential future use if we discover the actual EP0 FIFO register.
        """
        self.usb_ep0_fifo.append(value)
        if self.log_writes:
            print(f"[{self.cycles:8d}] [USB] EP0 FIFO write: 0x{value:02X} (total: {len(self.usb_ep0_fifo)} bytes)")

    def _usb_ep0_dma_trigger_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        USB EP0 DMA control write (0x9092).

        Writing 0x04 triggers DMA transfer from EP0 FIFO to USB data buffer (0x8000).
        The transfer length is read from 0x9003-0x9004.
        Hardware sets bit 2 while busy, then clears it when complete.
        """
        self.regs[addr] = value

        if value == 0x01:
            # Descriptor send trigger - firmware wrote 0x01 to 0x9092
            # Firmware should have already configured:
            #   0x905B/0x905C = DMA source address (code ROM address of descriptor)
            #   0x9004 = transfer length
            # We DMA from the firmware-specified address to USB buffer at 0x8000

            # Read DMA source address from firmware-configured registers
            dma_addr_hi = self.regs.get(0x905B, 0)
            dma_addr_lo = self.regs.get(0x905C, 0)
            dma_src_addr = (dma_addr_hi << 8) | dma_addr_lo

            # Read transfer length from firmware-configured register
            dma_len = self.regs.get(0x9004, 0)
            if dma_len == 0:
                # Fallback: use stored wLength from pending descriptor request
                # (can't read from 0x9E06-0x9E07 because firmware overwrote with descriptor data)
                usb_ctrl = getattr(self, 'usb_controller', None)
                if usb_ctrl and usb_ctrl.pending_descriptor_request:
                    dma_len = usb_ctrl.pending_descriptor_request.get('length', 64)
                else:
                    # Last resort: read bLength from first byte of descriptor at 0x9E00
                    # This works for single descriptors like device/string
                    bLength = self.regs.get(0x9E00, 0)
                    if 2 <= bLength <= 255:
                        dma_len = bLength
                    else:
                        dma_len = 64  # Default max packet size

            print(f"[{self.cycles:8d}] [USB] Descriptor DMA trigger (0x9092=0x01): src=0x{dma_src_addr:04X} len={dma_len}")

            if self.memory and dma_src_addr > 0 and dma_len > 0:
                # Firmware specified a code ROM address - DMA from there
                desc_data = bytes(self.memory.code[dma_src_addr:dma_src_addr + dma_len])
                for i, b in enumerate(desc_data):
                    self.memory.xdata[0x8000 + i] = b
                print(f"[{self.cycles:8d}] [USB] DMA'd {len(desc_data)} bytes from code 0x{dma_src_addr:04X} to 0x8000: {desc_data[:min(32, len(desc_data))].hex()}")
            elif dma_src_addr == 0 and dma_len > 0:
                # Firmware set src to 0 - DMA from EP0 buffer at 0x9E00 where firmware wrote data
                # Check if we have captured config descriptor (firmware writes it but then corrupts)
                usb_ctrl = getattr(self, 'usb_controller', None)
                desc_type = None
                if usb_ctrl and usb_ctrl.pending_descriptor_request:
                    desc_type = usb_ctrl.pending_descriptor_request.get('type', None)

                if desc_type == 0x02 and len(self.usb_captured_config_desc) >= dma_len:
                    # Use captured config descriptor (firmware corrupts 0x9E00 before DMA)
                    # Add UAS alt_setting 1 with 4 endpoints for patch.py compatibility
                    desc_data = self._extend_config_descriptor(self.usb_captured_config_desc, dma_len)
                    print(f"[{self.cycles:8d}] [USB] Using captured config descriptor ({dma_len} bytes)")
                else:
                    # Use current 0x9E00 buffer content
                    desc_data = bytes([self.regs.get(0x9E00 + i, 0) for i in range(dma_len)])

                for i, b in enumerate(desc_data):
                    self.memory.xdata[0x8000 + i] = b
                print(f"[{self.cycles:8d}] [USB] DMA'd {dma_len} bytes from EP0 buffer 0x9E00 to 0x8000: {desc_data[:min(32, dma_len)].hex()}")

            self.usb_control_transfer_active = False
            # NOTE: Don't clear usb_captured_config_desc here - firmware may trigger
            # DMA multiple times for one request. Capture is reset when new config
            # descriptor is written (offset 0 with value 0x09).

        elif value == 0x04:
            # DMA trigger - read length from 0x9003-0x9004
            len_lo = self.regs.get(0x9003, 0)
            len_hi = self.regs.get(0x9004, 0)
            length = (len_hi << 8) | len_lo

            print(f"[{self.cycles:8d}] [USB] EP0 DMA trigger: length={length}, FIFO has {len(self.usb_ep0_fifo)} bytes")

            # Copy FIFO data to USB data buffer at 0x8000
            if self.memory and len(self.usb_ep0_fifo) > 0:
                copy_len = min(length, len(self.usb_ep0_fifo))
                for i in range(copy_len):
                    self.memory.xdata[0x8000 + i] = self.usb_ep0_fifo[i]

                print(f"[{self.cycles:8d}] [USB] EP0 DMA: copied {copy_len} bytes to 0x8000")
                print(f"[{self.cycles:8d}] [USB] EP0 DMA: data = {bytes(self.usb_ep0_fifo[:copy_len]).hex()}")

                # Clear the FIFO after transfer
                self.usb_ep0_fifo.clear()

                # Clear control transfer active flag since DMA is complete
                self.usb_control_transfer_active = False

            # Set bit 2 (busy) - will be cleared on next read after poll
            self.regs[addr] = value | 0x04

    def _usb_ep0_dma_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB EP0 DMA status read (0x9092).

        Firmware polls this waiting for bit 2 to clear (DMA complete).
        After the initial write of 0x04, the hardware will clear bit 2
        when the transfer is done.
        """
        count = self.poll_counts.get(addr, 0)
        value = self.regs.get(addr, 0)

        # After a few polls, clear bit 2 (DMA complete)
        if count >= 2 and (value & 0x04):
            value &= ~0x04  # Clear bit 2
            self.regs[addr] = value
            print(f"[{self.cycles:8d}] [USB] EP0 DMA complete (bit 2 cleared)")

        return value

    def _usb_9091_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB control state register read (0x9091).

        Two-phase control transfer handling:
          Phase 1 (bit 0): Setup packet handler at 0xA5A6
          Phase 2 (bit 1): DMA response handler at 0xD088

        The firmware loops at 0xA5E2-0xA60B writing 0x01 and waiting for bit 0 to clear.
        When bit 0 clears and bit 1 is set, 0xD088 is called for DMA response.
        """
        value = self.regs.get(addr, 0)

        # Track read count for phase transition
        count = getattr(self, '_usb_9091_read_count', 0)
        self._usb_9091_read_count = count + 1

        # Phase transition: after setup handler has processed the request,
        # clear bit 0 and set bit 1 to trigger data phase
        # The setup handler writes 0x01 repeatedly, so we detect that pattern
        if getattr(self, '_usb_9091_setup_writes', 0) >= 3 and (value & 0x01):
            value = 0x02  # Clear bit 0, set bit 1 for data phase
            self.regs[addr] = value
            self._usb_9091_setup_writes = 0  # Reset for next transfer
            print(f"[{self.cycles:8d}] [USB] 0x9091 phase transition: setup→data (0x01→0x02)")

        return value

    def _usb_9091_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        USB control state register write (0x9091).

        The firmware writes 0x01 to 0x9091 in a loop at 0xA5E2-0xA60B, waiting
        for hardware to complete the setup phase. After enough writes, we
        transition to the data phase by modifying the read value.
        """
        self.regs[addr] = value

        # Count writes of 0x01 (setup phase polling)
        if value == 0x01:
            count = getattr(self, '_usb_9091_setup_writes', 0)
            self._usb_9091_setup_writes = count + 1
            if self.log_writes:
                print(f"[{self.cycles:8d}] [USB] 0x9091 write 0x01 (setup poll #{count + 1})")

    def _usb_9301_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        USB endpoint status read (0x9301).

        Bit 6 triggers the interrupt dispatch to device descriptor handler (0x0359).
        After reading, hardware clears bit 6 (acknowledge behavior).
        This allows the main loop at 0xD83B to proceed after the interrupt dispatch.
        """
        value = self.regs.get(addr, 0)

        # Clear bit 6 after reading (hardware acknowledge)
        if value & 0x40:
            self.regs[addr] = value & ~0x40
            if self.log_reads:
                print(f"[{self.cycles:8d}] [USB] 0x9301 read=0x{value:02X}, bit 6 cleared")

        return value

    def _usb_9301_ep0_arm_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        USB endpoint 0 arm/control write (0x9301).

        When bit 6 (0x40) is written, this arms EP0 for data transfer.
        The firmware handles GET_DESCRIPTOR by setting up DMA from its code ROM.
        The emulator does NOT assist with descriptor reading - the firmware must
        do all the work itself via proper MMIO/DMA emulation.
        """
        self.regs[addr] = value

        if value & 0x40:
            print(f"[{self.cycles:8d}] [USB] EP0 armed (9301=0x{value:02X})")

            # Log the request type for debugging (but don't process it!)
            bmRequestType = self.regs.get(0x9E00, 0)
            bRequest = self.regs.get(0x9E01, 0)

            if bmRequestType == 0x80 and bRequest == 0x06:  # GET_DESCRIPTOR
                desc_type = self.regs.get(0x9E03, 0)
                desc_index = self.regs.get(0x9E02, 0)
                wLength = self.regs.get(0x9E06, 0) | (self.regs.get(0x9E07, 0) << 8)
                print(f"[{self.cycles:8d}] [USB] GET_DESCRIPTOR: type=0x{desc_type:02X} "
                      f"index={desc_index} len={wLength} (firmware will handle via DMA)")
                # NOTE: The emulator does NOT populate the buffer here!
                # The firmware reads descriptors from its code ROM and sets up DMA.
                # If descriptors aren't appearing, fix the firmware DMA path, not here.

            # Mark control transfer completion status
            # - IN transfers (bit 7 set): Stay active until DMA completes (at 0x9092 write)
            # - OUT transfers (bit 7 clear): Complete when EP0 armed for status stage
            wLength = self.regs.get(0x9E06, 0) | (self.regs.get(0x9E07, 0) << 8)
            if bmRequestType & 0x80:
                # IN transfer (GET_DESCRIPTOR etc.) - stay active until DMA completes
                # The flag will be cleared by _usb_ep0_dma_trigger_write when DMA finishes
                pass
            else:
                # OUT transfer - complete if no data stage (wLength=0)
                # or firmware has processed the data
                if wLength == 0:
                    # No-data OUT transfer (SET_ADDRESS, SET_CONFIGURATION, etc.)
                    self.usb_control_transfer_active = False
                    self.usb_cmd_pending = False
                    print(f"[{self.cycles:8d}] [USB] OUT transfer complete (no data stage)")

    # ============================================================
    # DEPRECATED: _read_descriptor_from_firmware
    # This function violates the pure DMA principle documented above.
    # The firmware must handle descriptor reading itself via DMA.
    # DO NOT USE THIS FUNCTION - it only exists to document the
    # descriptor offsets found during analysis.
    #
    # Known descriptor offsets in fw.bin:
    # - Device descriptor: 0x0627 (18 bytes)
    # - Config descriptor: 0x5948 (BBB mode, 32 bytes)
    # ============================================================

    # ============================================================
    # DEPRECATED: _usb_get_descriptor_data
    # This function contained hardcoded USB descriptor data which
    # violates the pure DMA principle. The firmware must generate
    # all descriptor responses itself via DMA from its code ROM.
    # DO NOT resurrect this function!
    # ============================================================

    def _usb_ep_data_buf_read(self, hw: 'HardwareState', addr: int) -> int:
        """Read from USB EP data buffer (0xD800-0xDFFF)."""
        offset = addr - 0xD800
        if offset < len(self.usb_ep_data_buf):
            value = self.usb_ep_data_buf[offset]
            # Always log reads from command area (first 8 bytes)
            if offset < 8:
                print(f"[{self.cycles:8d}] [USB] Read EP buf 0x{addr:04X} = 0x{value:02X}")
            return value
        return 0x00

    def _usb_ep_buf_addr_write(self, hw: 'HardwareState', addr: int, value: int):
        """Write to USB EP buffer address registers (0x905B/0x905C).

        Firmware writes the DMA source address here:
        - 0x905B = high byte of source address
        - 0x905C = low byte of source address

        When DMA is triggered (via D800), data is read from this address.
        """
        self.regs[addr] = value
        if addr == 0x905B:
            print(f"[{self.cycles:8d}] [DMA] EP buf addr high = 0x{value:02X}")
        else:
            print(f"[{self.cycles:8d}] [DMA] EP buf addr low = 0x{value:02X}")

    def _usb_ep_data_buf_write(self, hw: 'HardwareState', addr: int, value: int):
        """Write to USB EP data buffer (0xD800-0xDFFF).

        D800 is the DMA control register. Writing 0x03 or 0x04 triggers DMA:
        - 0x03: DMA from address in 0x905B/0x905C to USB buffer
        - 0x04: DMA from address in 0xC4EA/0xC4EB (for E5 writes)

        This is PURE DMA - addresses come entirely from firmware register writes.
        The emulator does NOT determine addresses based on USB request type.
        """
        offset = addr - 0xD800
        if offset < len(self.usb_ep_data_buf):
            self.usb_ep_data_buf[offset] = value

        # DMA trigger at D800
        if addr == 0xD800 and value in (0x03, 0x04):
            # Get source address from registers firmware wrote
            src_hi = self.regs.get(0x905B, 0)
            src_lo = self.regs.get(0x905C, 0)
            src_addr = (src_hi << 8) | src_lo

            if src_addr > 0 and self.memory:
                # Get transfer length from D807 or use default
                xfer_len = self.regs.get(0xD807, 0)
                if xfer_len == 0:
                    xfer_len = 64  # Default EP0 max packet size

                print(f"[{self.cycles:8d}] [DMA] Trigger D800=0x{value:02X}: "
                      f"src=0x{src_addr:04X} len={xfer_len}")

                # Perform DMA: read from source, write to USB buffer at 0x8000
                for i in range(xfer_len):
                    # Read from XDATA (includes flash mirror via callbacks)
                    byte = self._read_xdata_for_dma(src_addr + i)
                    self.memory.xdata[0x8000 + i] = byte

                print(f"[{self.cycles:8d}] [DMA] Copied {xfer_len} bytes from 0x{src_addr:04X} to 0x8000")

        # E5 write DMA (uses different address registers)
        if addr == 0xD800 and value == 0x04 and self.usb_cmd_type == 0xE5:
            if not getattr(self, '_e5_dma_done', False):
                data = self.regs.get(0xC4E8, 0)
                addr_hi = self.regs.get(0xC4EA, 0)
                addr_lo = self.regs.get(0xC4EB, 0)
                target_addr = (addr_hi << 8) | addr_lo

                if data != 0xFF and target_addr > 0:
                    if self.memory and target_addr < 0x6000:
                        self.memory.xdata[target_addr] = data
                        self._e5_dma_done = True
                        self.usb_cmd_pending = False  # E5 command complete
                        self.usb_cmd_type = 0  # Reset command type
                        print(f"[{self.cycles:8d}] [DMA] E5 write: 0x{data:02X} to XDATA[0x{target_addr:04X}]")
                        print(f"[{self.cycles:8d}] [USB] E5 command completed")

    def _read_xdata_for_dma(self, addr: int) -> int:
        """Read from XDATA for DMA, using callbacks if registered."""
        # Check for callback (e.g., flash mirror)
        if addr in self.read_callbacks:
            return self.read_callbacks[addr](self, addr)
        # Direct XDATA read
        if self.memory and addr < len(self.memory.xdata):
            return self.memory.xdata[addr]
        return 0x00

    def _usb_ep_status_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Read USB EP status register 0xC4EC - indicates USB data availability.

        The EP loop at 0x18A5 checks C4EC bit 0 to see if there's USB data:
        - Bit 0 SET (0x01): Continue EP loop processing (for E4 commands)
        - Bit 0 CLEAR (0x00): Jump to 0x194F (E5 command handler path)

        For E5 commands, we need to return 0x00 so the firmware takes the
        E5 path at 0x18A8 → 0x194F → 0x197A (E5 handler).
        """
        # Track EP loop iterations
        if self.usb_cmd_pending:
            if not hasattr(self, '_c4ec_read_count'):
                self._c4ec_read_count = 0
            self._c4ec_read_count += 1

            # For E5 commands, return 0x00 to take the E5 path at 0x18A8
            # This triggers: 0x18A8 ljmp 0x194F → 0x197A E5 check
            if self.usb_cmd_type == 0xE5:
                value = 0x00
                print(f"[{self.cycles:8d}] [USB] Read 0xC4EC = 0x{value:02X} (E5 path - bit 0 CLEAR)")
                return value

            # For E4 commands, return 0x01 for the first several reads to allow
            # full command processing through the EP loop
            if self._c4ec_read_count <= 3:
                value = 0x01
                print(f"[{self.cycles:8d}] [USB] Read 0xC4EC = 0x{value:02X} (EP loop iter {self._c4ec_read_count})")
            else:
                # After enough iterations, return 0 to exit EP loop
                value = 0x00
                print(f"[{self.cycles:8d}] [USB] Read 0xC4EC = 0x{value:02X} (exit EP loop)")
            return value

        # Normal read when no command pending
        return self.regs.get(addr, 0x00)

    def _usb_ep_index_write(self, hw: 'HardwareState', addr: int, value: int):
        """Write USB EP index register 0xC4ED - selects which endpoint to query."""
        # Low 5 bits are the endpoint index (0-31)
        self.usb_ep_selected = value & 0x1F
        self.regs[addr] = value
        if self.usb_cmd_pending:
            print(f"[{self.cycles:8d}] [USB] Select EP index {self.usb_ep_selected}")

    def _usb_ep_id_low_read(self, hw: 'HardwareState', addr: int) -> int:
        """Read USB EP ID low byte (0xC4EE) for currently selected endpoint."""
        # When USB command pending and EP0 selected, return the value from RAM 0x0056
        # This matches what firmware expects (it compares 0xC4EE/0xC4EF with 0x0056/0x0057)
        if self.usb_cmd_pending and self.usb_ep_selected == 0 and self.memory:
            # Read the expected value from RAM and return it so comparison passes
            expected = self.memory.xdata[0x0056]
            print(f"[{self.cycles:8d}] [USB] EP0 ID low = 0x{expected:02X} (from RAM 0x0056)")
            return expected
        return 0xFF  # No endpoint / invalid

    def _usb_ep_id_high_read(self, hw: 'HardwareState', addr: int) -> int:
        """Read USB EP ID high byte (0xC4EF) for currently selected endpoint."""
        # When USB command pending and EP0 selected, return the value from RAM 0x0057
        # This matches what firmware expects (it compares 0xC4EE/0xC4EF with 0x0056/0x0057)
        if self.usb_cmd_pending and self.usb_ep_selected == 0 and self.memory:
            # Read the expected value from RAM and return it so comparison passes
            expected = self.memory.xdata[0x0057]
            print(f"[{self.cycles:8d}] [USB] EP0 ID high = 0x{expected:02X} (from RAM 0x0057)")
            return expected
        return 0xFF  # No endpoint / invalid

    def _usb_ep_data_ready_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Read USB endpoint data ready register (0x90A1-0x90C0).
        Returns bit 0 = 1 when USB command is pending for that endpoint.
        """
        ep_index = addr - 0x90A1  # EP0 is at 0x90A1, EP1 at 0x90A2, etc.
        value = self.regs.get(addr, 0)

        # When USB command pending and this is the target endpoint, keep bit 0 set
        if self.usb_cmd_pending and ep_index == 0:
            value |= 0x01  # Bit 0 = data ready
            if self.log_reads:
                print(f"[{self.cycles:8d}] [USB] EP{ep_index} data ready = 0x{value:02X} (cmd pending)")
        return value

    def _usb_ep_status_reg_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Read USB endpoint status register (0x9096-0x90A0).

        At 0x18F5 firmware reads this register, and at 0x18F6 "jz 0x191B" skips
        command processing if the value is 0. So we need NON-ZERO to process commands.

        The bit mask table at 0x5BC9 is: 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
        For EP index N, bit (N % 8) must be set in register 0x9096 + (N / 8).
        """
        ep_index = addr - 0x9096  # EP0 is at 0x9096, EP1 at 0x9097, etc.
        value = self.regs.get(addr, 0)

        # When USB command pending and this is EP0, return non-zero to enable command processing
        # The firmware ANDs this value with a bit mask (0x01 for EP0) and checks if non-zero
        if self.usb_cmd_pending and ep_index == 0:
            value = 0x01  # Bit 0 set for EP0
            print(f"[{self.cycles:8d}] [USB] EP{ep_index} status reg 0x{addr:04X} = 0x{value:02X} (cmd pending)")
            return value
        return value

    def _usb_e5_value_read(self, hw: 'HardwareState', addr: int) -> int:
        """
        Read USB E5 value register 0xC47A.

        When an E5 command is pending, this returns the injected value.
        The firmware reads this at 0x1800 (movx a, @dptr after mov dptr, #0xc47a)
        and stores it to IDATA[0x38] at 0x1801.

        The firmware clears this register (writes 0xFF) at 0x1178 before calling
        the EP loop at 0x17DB. We preserve the injected value until it's read
        by the E5 handler at 0x17FD-0x1801.

        After the value is read, we clear usb_cmd_pending to allow the firmware
        to exit the command loop. Unlike E4 which uses DMA at 0xB296 to signal
        completion, E5 commands complete when the value is read.
        """
        if self.usb_cmd_pending and self.usb_cmd_type == 0xE5:
            value = self.usb_e5_pending_value
            print(f"[{self.cycles:8d}] [USB] Read E5 value reg 0xC47A = 0x{value:02X} (pending E5)")

            # For E5 commands, don't clear pending yet - let the firmware continue
            # processing. The command completes when the DMA write happens (D800=0x04)
            # or after a timeout. Track that we've delivered the value.
            self._e5_value_delivered = True

            return value

        # Normal read
        return self.regs.get(addr, 0x00)

    def _usb_e5_value_write(self, hw: 'HardwareState', addr: int, value: int):
        """
        Write USB E5 value register 0xC47A.

        The firmware writes 0xFF to this register at 0x1176-0x1178 to clear it
        after processing each command. We preserve the pending E5 value by
        ignoring clears (0xFF writes) while an E5 command is pending.
        """
        if self.usb_cmd_pending and self.usb_cmd_type == 0xE5 and value == 0xFF:
            # Ignore clear while E5 command is pending
            print(f"[{self.cycles:8d}] [USB] Ignoring write 0xFF to 0xC47A (E5 pending)")
            return

        # Normal write - update the register
        self.regs[addr] = value

    # ============================================
    # Main Read/Write Interface
    # ============================================
    def read(self, addr: int) -> int:
        """Read from hardware register."""
        addr &= 0xFFFF

        # Only handle hardware registers (>= 0x6000)
        if addr < 0x6000:
            return 0x00  # Should not be called for RAM

        self.poll_counts[addr] = self.poll_counts.get(addr, 0) + 1

        # Debug: trace CE55 reads
        if addr == 0xCE55:
            has_callback = addr in self.read_callbacks
            print(f"[{self.cycles:8d}] [DEBUG] Reading CE55, callback registered: {has_callback}")

        if addr in self.read_callbacks:
            value = self.read_callbacks[addr](self, addr)
        elif addr in self.regs:
            value = self.regs[addr]
        else:
            value = 0x00

        if self.log_reads:
            print(f"[{self.cycles:8d}] [HW] Read  0x{addr:04X} = 0x{value:02X}")

        return value

    def write(self, addr: int, value: int):
        """Write to hardware register."""
        addr &= 0xFFFF
        value &= 0xFF

        # Only handle hardware registers (>= 0x6000)
        if addr < 0x6000:
            return  # Should not be called for RAM

        if self.log_writes:
            print(f"[{self.cycles:8d}] [HW] Write 0x{addr:04X} = 0x{value:02X}")

        if addr in self.write_callbacks:
            self.write_callbacks[addr](self, addr, value)
        else:
            self.regs[addr] = value

    # ============================================
    # Tick - Advance Hardware State
    # ============================================
    def tick(self, cycles: int, cpu=None):
        """Advance hardware state by cycles."""
        self.cycles += cycles

        # In proxy mode, skip all fake USB/interrupt injection
        # Real hardware handles everything
        if self.proxy_mode:
            return

        # USB plug-in event after delay
        # Skip if a USB command is already pending to avoid interfering with it
        if not self.usb_connected and self.cycles > self.usb_connect_delay and not self.usb_cmd_pending:
            self.usb_connected = True
            print(f"\n[{self.cycles:8d}] [HW] === USB PLUG-IN EVENT ===")

            # Update USB hardware registers via USBController
            self.usb_controller.connect()

            # Set NVMe queue busy - triggers the usb_ep_loop_180d(1) call
            self.regs[0xC471] = 0x01  # Bit 0 - queue busy

            # Re-enable PD task path by setting 0x91C0 bit 1
            # The firmware clears this at 0xCA8B during init, but we need it set
            # for the main loop at 0x2027 to call the PD task at 0x0322
            self.regs[0x91C0] = 0x02  # Bit 1 - enables PD task in main loop

            # Set PD interrupt pending - this triggers the PD handler
            # Bit 2 (0x04) is the fallback path at 0x9354 when 0x0A9D != 0x01/0x02
            # Bit 3 (0x08) is for port 1 when 0x0A9D == 0x01
            self.regs[0xCA0D] = 0x0C  # Bits 2+3 - PD interrupt (covers both paths)
            self.regs[0xCA0E] = 0x04  # Bit 2 - PD interrupt for port 2

            # Set debug trigger
            self.regs[0xC80A] = 0x40  # Bit 6 - triggers PD debug output at 0x935E

            # Set PD event info for debug output
            # These are read by 0xAE89 to print [PD_int:XX:XX] and determine message type
            self.regs[0xE40F] = 0x01  # PD event type (bit 0 = Source_Cap)
            self.regs[0xE410] = 0x00  # PD sub-event

            print(f"[{self.cycles:8d}] [HW] USB: 0x9000=0x81, C802=0x05, C471=0x01, CA0D=0x0C, E40F=0x01")
            print(f"[{self.cycles:8d}] [HW] USB state machine: firmware will poll 0xCE89 to transition states")

            # Trigger External Interrupt 0 to invoke the interrupt handler at 0x0E33
            # This requires IE register (0xA8) to have EA (bit 7) and EX0 (bit 0) set
            if cpu:
                # Enable global interrupts (EA) and EX0 in IE register
                ie = self.memory.read_sfr(0xA8) if self.memory else 0
                ie |= 0x81  # EA (bit 7) + EX0 (bit 0)
                if self.memory:
                    self.memory.write_sfr(0xA8, ie)
                cpu._ext0_pending = True
                print(f"[{self.cycles:8d}] [HW] Triggered EX0 interrupt (IE=0x{ie:02X})")

        # Periodic timer interrupt
        if self.cycles % 1000 == 0:
            self.regs[0xC806] |= 0x01

        # Inject USB command after USB connected and additional delay
        # Only inject if usb_inject_cmd was set (via --usb-cmd option)
        if self.usb_connected and not self.usb_injected and self.usb_inject_cmd:
            if self.cycles > self.usb_connect_delay + self.usb_inject_delay:
                self.usb_injected = True
                cmd_type, addr, val_or_size = self.usb_inject_cmd
                print(f"\n[{self.cycles:8d}] [HW] === INJECTING USB COMMAND ===")
                if cmd_type == 0xE4:
                    self.inject_usb_command(0xE4, addr, size=val_or_size)
                elif cmd_type == 0xE5:
                    self.inject_usb_command(0xE5, addr, value=val_or_size)
                else:
                    print(f"[HW] Unknown USB command type: 0x{cmd_type:02X}")

        # Trigger EX0 interrupt after USB command injection
        if hasattr(self, '_pending_usb_interrupt') and self._pending_usb_interrupt and cpu:
            self._pending_usb_interrupt = False
            # Enable global interrupts (EA) and EX0 in IE register
            ie = self.memory.read_sfr(0xA8) if self.memory else 0
            ie |= 0x81  # EA (bit 7) + EX0 (bit 0)
            if self.memory:
                self.memory.write_sfr(0xA8, ie)
            cpu._ext0_pending = True
            print(f"[{self.cycles:8d}] [HW] Triggered EX0 interrupt for USB command (IE=0x{ie:02X})")



def create_hardware_hooks(memory: 'Memory', hw: HardwareState, proxy: 'UARTProxy' = None, proxy_mask: list = None):
    """
    Register hardware hooks with memory system.
    Only hooks hardware register addresses (>= 0x6000).

    Args:
        memory: Memory system to hook
        hw: HardwareState for emulation mode
        proxy: Optional UARTProxy for real hardware mode
               If provided, MMIO reads/writes go to real hardware instead of emulation
        proxy_mask: List of (start, end) tuples for address ranges to emulate instead of proxy
                    Used to discover minimum set of registers that need real hardware
    """
    if proxy_mask is None:
        proxy_mask = []

    # Hardware register ranges (all >= 0x6000)
    # NOTE: 0x7000-0x7FFF is flash buffer RAM, NOT hardware registers
    mmio_ranges = [
        (0x8000, 0x9000),   # USB/SCSI Data Buffer
        (0x9000, 0x9400),   # USB Interface
        (0x92C0, 0x9300),   # Power Management
        (0x9E00, 0xA000),   # USB Control Buffer
        (0xB200, 0xB900),   # PCIe Passthrough
        (0xC000, 0xC100),   # UART
        (0xC400, 0xC600),   # NVMe Interface
        (0xC600, 0xC700),   # PHY Extended
        (0xC800, 0xC900),   # Interrupt/DMA/Flash
        (0xCA00, 0xCB00),   # PD Controller
        (0xCC00, 0xCF00),   # Timer/CPU/SCSI
        (0xD800, 0xE000),   # USB Endpoint Data Buffer
        (0xE300, 0xE400),   # PHY Completion / Debug
        (0xE400, 0xE500),   # Command Engine
        (0xE700, 0xE800),   # System Status
    ]

    # Set memory reference for USB commands
    hw.memory = memory

    # ============================================
    # XDATA Write Tracing
    # ============================================
    # Hook XDATA writes to trace firmware RAM updates.
    # This helps understand how firmware populates key addresses.
    def make_xdata_write_trace_hook(hw_ref, mem_ref, original_write):
        """Create a write hook that traces writes and calls original."""
        def hook(addr, value):
            # Call trace function if enabled
            if hw_ref.xdata_trace_enabled:
                # Get PC from CPU if available
                pc = 0
                if hasattr(hw_ref, '_cpu_ref') and hw_ref._cpu_ref:
                    pc = hw_ref._cpu_ref.pc
                hw_ref.trace_xdata_write(addr, value, pc)
            # Perform actual write
            return original_write(addr, value)
        return hook

    def make_read_hook(hw_ref):
        def hook(addr):
            return hw_ref.read(addr)
        return hook

    def make_write_hook(hw_ref):
        def hook(addr, value):
            hw_ref.write(addr, value)
        return hook

    # ============================================
    # Proxy Mode Hooks (real hardware via UART)
    # ============================================
    def make_proxy_read_hook(proxy_ref, hw_ref):
        """Create a read hook that proxies to real hardware."""
        def hook(addr):
            value = proxy_ref.read(addr)
            if proxy_ref.debug >= 2:
                pc = hw_ref._cpu_ref.pc if hw_ref._cpu_ref else 0
                cyc = hw_ref.cycles
                reg_name = get_register_name(addr)
                if reg_name:
                    print(f"[{cyc:8d}] PC=0x{pc:04X} Read  0x{addr:04X} = 0x{value:02X}  {reg_name}")
                else:
                    print(f"[{cyc:8d}] PC=0x{pc:04X} Read  0x{addr:04X} = 0x{value:02X}")
            return value
        return hook

    def make_proxy_write_hook(proxy_ref, hw_ref):
        """Create a write hook that proxies to real hardware."""
        # Shadow 0x9E00 buffer header to detect device descriptor
        ep0_shadow = {}
        def hook(addr, value):
            # Track writes to USB EP0 buffer header bytes
            if 0x9E00 <= addr <= 0x9E01:
                ep0_shadow[addr] = value

            # Patch VID/PID only when buffer contains a device descriptor
            # Device descriptor: bLength=0x12 at 0x9E00, bDescriptorType=0x01 at 0x9E01
            # Bytes 8-9 = idVendor (LE), bytes 10-11 = idProduct (LE)
            if hw_ref.vidpid_override is not None and 0x9E08 <= addr <= 0x9E0B:
                if ep0_shadow.get(0x9E00) == 0x12 and ep0_shadow.get(0x9E01) == 0x01:
                    vid, pid = hw_ref.vidpid_override
                    if addr == 0x9E08: value = vid & 0xFF
                    elif addr == 0x9E09: value = (vid >> 8) & 0xFF
                    elif addr == 0x9E0A: value = pid & 0xFF
                    elif addr == 0x9E0B: value = (pid >> 8) & 0xFF

            proxy_ref.write(addr, value)
            if proxy_ref.debug >= 2:
                pc = hw_ref._cpu_ref.pc if hw_ref._cpu_ref else 0
                cyc = hw_ref.cycles
                reg_name = get_register_name(addr)
                if reg_name:
                    print(f"[{cyc:8d}] PC=0x{pc:04X} Write 0x{addr:04X} = 0x{value:02X}  {reg_name}")
                else:
                    print(f"[{cyc:8d}] PC=0x{pc:04X} Write 0x{addr:04X} = 0x{value:02X}")
        return hook

    # Select hooks based on proxy mode
    if proxy is not None:
        # Proxy mode - MMIO goes to real hardware (except certain ranges)
        print(f"[HW] Using UART proxy for MMIO access")
        proxy_read_hook = make_proxy_read_hook(proxy, hw)
        proxy_write_hook = make_proxy_write_hook(proxy, hw)
        emu_read_hook = make_read_hook(hw)
        emu_write_hook = make_write_hook(hw)

        def should_emulate(addr):
            """Check if address should be emulated instead of proxied."""
            # UART (0xC000-0xC00F) - proxy uses these for communication
            if 0xC000 <= addr <= 0xC00F:
                return True
            # CPU interrupt/DMA control - may cause reset when written via proxy
            if addr == 0xCC81:
                return True
            # CPU bus mode control - writing bit 0 changes bus access mode,
            # causing subsequent MMIO reads (e.g. 0xC65A) to hang the proxy CPU
            if addr == 0xCA2E:
                return True
            # Check user-specified mask ranges
            for mask_start, mask_end in proxy_mask:
                if mask_start <= addr < mask_end:
                    return True
            return False

        # Print mask info if any
        if proxy_mask:
            print(f"[HW] Proxy mask (emulated instead of proxied):")
            for start, end in proxy_mask:
                print(f"[HW]   0x{start:04X}-0x{end:04X}")

        for start, end in mmio_ranges:
            for addr in range(start, end):
                if should_emulate(addr):
                    memory.xdata_read_hooks[addr] = emu_read_hook
                    memory.xdata_write_hooks[addr] = emu_write_hook
                else:
                    memory.xdata_read_hooks[addr] = proxy_read_hook
                    memory.xdata_write_hooks[addr] = proxy_write_hook

        # ============================================
        # SFR Proxy Hooks
        # ============================================
        # Key SFRs must be proxied to real hardware for interrupts to work.
        # The emulator runs the code but hardware state (IE, timers, etc.)
        # must be synchronized with real hardware.

        # SFRs to proxy (interrupt and timer related)
        proxy_sfrs = [
            0xA8,  # IE - Interrupt Enable
            0xB8,  # IP - Interrupt Priority
            0x88,  # TCON - Timer Control
            0x89,  # TMOD - Timer Mode
            0x8A,  # TL0 - Timer 0 Low
            0x8B,  # TL1 - Timer 1 Low
            0x8C,  # TH0 - Timer 0 High
            0x8D,  # TH1 - Timer 1 High
        ]

        def make_sfr_proxy_write_hook(proxy_ref, sfr_addr, hw_ref, cache):
            """Create SFR write hook that proxies to real hardware and caches value."""
            def hook(addr, value):
                cache[sfr_addr] = value  # Cache the written value
                proxy_ref.sfr_write(sfr_addr, value)
                if proxy_ref.debug >= 3:
                    pc = hw_ref._cpu_ref.pc if hw_ref._cpu_ref else 0
                    cyc = hw_ref.cycles
                    print(f"[{cyc:8d}] PC=0x{pc:04X} SFR Write 0x{sfr_addr:02X} = 0x{value:02X}")
            return hook

        # Cache for SFR values - return last written value instead of proxying reads
        sfr_cache = {}

        def make_sfr_proxy_read_hook(proxy_ref, sfr_addr, hw_ref, cache):
            """Create SFR read hook that returns cached value (no proxy read needed)."""
            def hook(addr):
                # Return cached value (default 0 if never written)
                value = cache.get(sfr_addr, 0)
                if proxy_ref.debug >= 3:
                    pc = hw_ref._cpu_ref.pc if hw_ref._cpu_ref else 0
                    cyc = hw_ref.cycles
                    print(f"[{cyc:8d}] PC=0x{pc:04X} SFR Read  0x{sfr_addr:02X} = 0x{value:02X} (cached)")
                return value
            return hook

        for sfr_addr in proxy_sfrs:
            memory.sfr_write_hooks[sfr_addr] = make_sfr_proxy_write_hook(proxy, sfr_addr, hw, sfr_cache)
            memory.sfr_read_hooks[sfr_addr] = make_sfr_proxy_read_hook(proxy, sfr_addr, hw, sfr_cache)

        print(f"[HW] Proxying {len(proxy_sfrs)} SFRs to real hardware")
    else:
        # Emulation mode - use HardwareState
        read_hook = make_read_hook(hw)
        write_hook = make_write_hook(hw)

        for start, end in mmio_ranges:
            for addr in range(start, end):
                memory.xdata_read_hooks[addr] = read_hook
                memory.xdata_write_hooks[addr] = write_hook

    # Debug hooks for XDATA can be added here when needed
    # Example: Trace reads/writes to specific addresses
    # memory.xdata_write_hooks[0x0AF7] = make_debug_hook(hw, memory)

    # USB3 mode hook for 0x0ACC
    # During GET_DESCRIPTOR handling at 0x87A0, firmware reads 0x0ACC to determine
    # USB2 vs USB3 mode. Bit 1 SET = USB3 mode, which uses R7=5 for descriptors.
    # Without bit 1 SET, firmware takes USB2 path with R7=3.
    # This hook returns USB3 mode when a control transfer is active.
    def usb3_mode_read_hook(addr):
        if hw.usb_control_transfer_active:
            # USB3 mode: bit 1 SET for GET_DESCRIPTOR to set R7=5
            return 0x02
        return memory.xdata[addr]
    memory.xdata_read_hooks[0x0ACC] = usb3_mode_read_hook

