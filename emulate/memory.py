"""
ASM2464PD Memory System

This module implements the memory subsystem for the 8051 emulator including:
- CODE memory (with banking support for ~98KB firmware)
- IDATA (internal 256 bytes)
- XDATA (external data memory including MMIO)
- SFR (Special Function Registers)
- Bit-addressable memory
"""

from typing import Callable, Optional, Dict
from dataclasses import dataclass, field


@dataclass
class Memory:
    """Memory subsystem for ASM2464PD emulation."""

    # Memory arrays
    code: bytearray = field(default_factory=lambda: bytearray(0x18000))  # 96KB (2 banks)
    idata: bytearray = field(default_factory=lambda: bytearray(256))
    xdata: bytearray = field(default_factory=lambda: bytearray(0x10000))  # 64KB
    sfr: bytearray = field(default_factory=lambda: bytearray(128))  # 0x80-0xFF

    # XDATA read/write hooks for MMIO
    xdata_read_hooks: Dict[int, Callable[[int], int]] = field(default_factory=dict)
    xdata_write_hooks: Dict[int, Callable[[int, int], None]] = field(default_factory=dict)

    # IDATA read/write hooks (for USB state and other internal RAM emulation)
    idata_read_hooks: Dict[int, Callable[[int], int]] = field(default_factory=dict)
    idata_write_hooks: Dict[int, Callable[[int, int], None]] = field(default_factory=dict)

    # DMA/Timer sync flag polling counters (for RAM sync flags that need auto-clear)
    # These flags are set by firmware and should be cleared by DMA/timer completion
    sync_flag_polls: Dict[int, int] = field(default_factory=dict)

    # SFR read/write hooks
    sfr_read_hooks: Dict[int, Callable[[int], int]] = field(default_factory=dict)
    sfr_write_hooks: Dict[int, Callable[[int, int], None]] = field(default_factory=dict)

    # Code bank register (DPX at 0x96)
    # Accessed via SFR but affects code reads
    SFR_DPX = 0x96

    # Proxy reference for DPX-banked XDATA access (set by create_hardware_hooks)
    proxy: object = field(default=None, repr=False)

    def load_firmware(self, data: bytes, offset: int = 0):
        """Load firmware binary into code memory."""
        end = min(offset + len(data), len(self.code))
        self.code[offset:end] = data[:end - offset]

    # Bank 1 base offset in firmware file
    # Bank 1 code starts at file offset 0xFF6B, mapped to address space 0x8000-0xFFFF
    BANK1_FILE_BASE = 0xFF6B

    def read_code(self, addr: int) -> int:
        """
        Read from CODE memory with banking.

        The ASM2464PD has ~98KB of firmware with banking:
        - 0x0000-0x7FFF: Always bank 0 (32KB shared)
        - 0x8000-0xFFFF: Bank 0 or Bank 1 based on DPX register

        Bank 0 upper: file offset 0x8000-0xFFFF
        Bank 1: file offset 0xFF6B + (addr - 0x8000)
        """
        addr &= 0xFFFF

        # If accessing upper 32KB, check bank
        if addr >= 0x8000:
            dpx = self.sfr[self.SFR_DPX - 0x80]
            if dpx & 1:  # Bank 1
                # Map 0x8000-0xFFFF to file offset 0xFF6B + offset
                file_addr = self.BANK1_FILE_BASE + (addr - 0x8000)
                if file_addr < len(self.code):
                    return self.code[file_addr]
                return 0xFF

        # Bank 0 or lower 32KB
        if addr < len(self.code):
            return self.code[addr]
        return 0xFF

    def read_idata(self, addr: int) -> int:
        """Read from IDATA (internal 256 bytes) with hooks."""
        addr &= 0xFF

        # Check for hooks
        if addr in self.idata_read_hooks:
            return self.idata_read_hooks[addr](addr)

        return self.idata[addr]

    def write_idata(self, addr: int, value: int):
        """Write to IDATA with hooks."""
        addr &= 0xFF
        value &= 0xFF

        # Check for hooks
        if addr in self.idata_write_hooks:
            self.idata_write_hooks[addr](addr, value)
            # Still update backing store
            self.idata[addr] = value
            return

        self.idata[addr] = value

    # Known DMA/timer sync flag addresses that need auto-clear when polled
    # These are RAM flags that firmware sets and waits for hardware to clear
    SYNC_FLAG_ADDRS = {0x1238}  # Timer/DMA sync flag at 0x1238
    SYNC_FLAG_CLEAR_AFTER = 5   # Clear after this many polls

    def read_xdata(self, addr: int) -> int:
        """Read from XDATA with MMIO hooks."""
        addr &= 0xFFFF

        # DPX-banked access (SFR 0x93 != 0): entire XDATA address space maps
        # DPX-banked access: when SFR 0x93 != 0, the entire XDATA address space
        # maps to PHY registers.  Use dedicated DPX read/write proxy commands.
        dpx = self.sfr[0x93 - 0x80]
        if dpx and self.proxy is not None:
            value = self.proxy.read_dpx(addr)
            if self.proxy.debug >= 2:
                print(f"Read  0x{addr:04X} = 0x{value:02X}  DPX={dpx}")
            return value

        # Check for MMIO hooks
        if addr in self.xdata_read_hooks:
            return self.xdata_read_hooks[addr](addr)

        # Handle DMA/timer sync flags - auto-clear after polling
        # This simulates hardware completing the DMA/timer operation
        if addr in self.SYNC_FLAG_ADDRS:
            value = self.xdata[addr]
            if value & 0x01:  # Flag is set, count polls
                self.sync_flag_polls[addr] = self.sync_flag_polls.get(addr, 0) + 1
                if self.sync_flag_polls[addr] >= self.SYNC_FLAG_CLEAR_AFTER:
                    # Simulate DMA/timer completion by clearing the flag
                    self.xdata[addr] = 0x00
                    self.sync_flag_polls[addr] = 0
                    return 0x00
            return value

        # Check for range hooks (for efficiency, could use interval tree)
        # For now, direct array access
        return self.xdata[addr]

    def write_xdata(self, addr: int, value: int):
        """Write to XDATA with MMIO hooks."""
        addr &= 0xFFFF
        value &= 0xFF

        # DPX-banked access: use dedicated DPX write command to proxy.
        dpx = self.sfr[0x93 - 0x80]
        if dpx and self.proxy is not None:
            if self.proxy.debug >= 2:
                print(f"Write 0x{addr:04X} = 0x{value:02X}  DPX={dpx}")
            self.proxy.write_dpx(addr, value)
            return

        # Check for MMIO hooks
        if addr in self.xdata_write_hooks:
            self.xdata_write_hooks[addr](addr, value)
            return

        self.xdata[addr] = value

    def read_sfr(self, addr: int) -> int:
        """Read from SFR space (0x80-0xFF)."""
        if addr < 0x80:
            raise ValueError(f"SFR address must be >= 0x80, got 0x{addr:02X}")

        # Check for hooks
        if addr in self.sfr_read_hooks:
            return self.sfr_read_hooks[addr](addr)

        return self.sfr[addr - 0x80]

    def write_sfr(self, addr: int, value: int):
        """Write to SFR space."""
        if addr < 0x80:
            raise ValueError(f"SFR address must be >= 0x80, got 0x{addr:02X}")

        value &= 0xFF

        # Check for hooks
        if addr in self.sfr_write_hooks:
            self.sfr_write_hooks[addr](addr, value)
            # Still update the backing store
            self.sfr[addr - 0x80] = value
            return

        self.sfr[addr - 0x80] = value

    def read_bit(self, bit_addr: int) -> bool:
        """
        Read bit-addressable memory.

        Bit addresses 0x00-0x7F: IDATA 0x20-0x2F (bytes 0x20-0x2F, 8 bits each)
        Bit addresses 0x80-0xFF: SFR bit-addressable registers
        """
        if bit_addr < 0x80:
            # IDATA bit-addressable area (0x20-0x2F)
            byte_addr = 0x20 + (bit_addr >> 3)
            bit_pos = bit_addr & 0x07
            return bool(self.idata[byte_addr] & (1 << bit_pos))
        else:
            # SFR bit-addressable (addresses ending in 0 or 8)
            # Bit address = SFR_addr + bit_position
            # SFR addresses: 0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8
            sfr_addr = bit_addr & 0xF8  # Base SFR address
            bit_pos = bit_addr & 0x07
            return bool(self.read_sfr(sfr_addr) & (1 << bit_pos))

    def write_bit(self, bit_addr: int, value: bool):
        """Write to bit-addressable memory."""
        if bit_addr < 0x80:
            # IDATA bit-addressable area
            byte_addr = 0x20 + (bit_addr >> 3)
            bit_pos = bit_addr & 0x07
            if value:
                self.idata[byte_addr] |= (1 << bit_pos)
            else:
                self.idata[byte_addr] &= ~(1 << bit_pos)
        else:
            # SFR bit-addressable
            sfr_addr = bit_addr & 0xF8
            bit_pos = bit_addr & 0x07
            val = self.read_sfr(sfr_addr)
            if value:
                val |= (1 << bit_pos)
            else:
                val &= ~(1 << bit_pos)
            self.write_sfr(sfr_addr, val)

    def add_xdata_hook(self, addr: int, read_fn: Optional[Callable] = None,
                       write_fn: Optional[Callable] = None):
        """Add read/write hooks for XDATA address."""
        if read_fn:
            self.xdata_read_hooks[addr] = read_fn
        if write_fn:
            self.xdata_write_hooks[addr] = write_fn

    def add_xdata_range_hook(self, start: int, end: int,
                             read_fn: Optional[Callable] = None,
                             write_fn: Optional[Callable] = None):
        """Add read/write hooks for XDATA address range."""
        for addr in range(start, end):
            if read_fn:
                self.xdata_read_hooks[addr] = read_fn
            if write_fn:
                self.xdata_write_hooks[addr] = write_fn

    def add_sfr_hook(self, addr: int, read_fn: Optional[Callable] = None,
                     write_fn: Optional[Callable] = None):
        """Add read/write hooks for SFR address."""
        if read_fn:
            self.sfr_read_hooks[addr] = read_fn
        if write_fn:
            self.sfr_write_hooks[addr] = write_fn

    def reset(self):
        """Reset memory to initial state."""
        # Clear IDATA
        for i in range(len(self.idata)):
            self.idata[i] = 0

        # Reset SFRs to defaults
        for i in range(len(self.sfr)):
            self.sfr[i] = 0

        # SP defaults to 0x07
        self.sfr[0x81 - 0x80] = 0x07

        # Clear XDATA (optional, for clean state)
        # Note: In real hardware, XDATA may retain values

        # Initialize flash buffer area (0x7000-0x7FFF) to 0xFF
        # This simulates unprogrammed/erased flash.
        # The firmware checks these values during init to determine
        # PD configuration mode. If 0x7076-0x7079 == 0xFF, it enables
        # the debug output path by setting 0x0AE5 = 0.
        for i in range(0x7000, 0x8000):
            self.xdata[i] = 0xFF

        # Pre-initialize key variables for PD debug path
        # The firmware init at 0x91D6 sets:
        #   If 0x0AE5 == 0: 0x0AF0 = 0x3F (enables debug)
        #   If 0x0AE5 != 0: 0x0AF0 = 0x00 (disables debug)
        # Since the 0x90xx init that sets 0x0AE5 = 0 is not being called
        # in our execution path, we pre-set the values needed for debug.
        self.xdata[0x0AE5] = 0x00  # G_TLP_INIT_FLAG - set to 0 for debug
        self.xdata[0x0AF0] = 0x3F  # G_FLASH_CFG - enable debug (bit 5 set)


def create_memory_system():
    """Create and configure memory system for ASM2464PD."""
    mem = Memory()
    return mem


# Memory map constants for ASM2464PD
class MemoryMap:
    """ASM2464PD Memory Map Constants."""

    # XRAM Work Area
    XRAM_START = 0x0000
    XRAM_END = 0x5FFF  # 24KB

    # Flash Buffer
    FLASH_BUF_START = 0x7000
    FLASH_BUF_END = 0x7FFF  # 4KB

    # USB/SCSI Buffer
    USB_SCSI_BUF_START = 0x8000
    USB_SCSI_BUF_END = 0x8FFF  # 4KB

    # USB Interface Registers
    USB_REG_START = 0x9000
    USB_REG_END = 0x93FF

    # USB Control Buffer
    USB_CTRL_BUF_START = 0x9E00
    USB_CTRL_BUF_END = 0x9FFF

    # NVMe I/O Queue
    NVME_IOSQ_START = 0xA000
    NVME_IOSQ_END = 0xAFFF

    # NVMe Admin Queues
    NVME_ASQ_START = 0xB000
    NVME_ASQ_END = 0xB0FF
    NVME_ACQ_START = 0xB100
    NVME_ACQ_END = 0xB1FF

    # PCIe Passthrough
    PCIE_REG_START = 0xB200
    PCIE_REG_END = 0xB8FF

    # UART Controller
    UART_START = 0xC000
    UART_END = 0xC00F

    # Link/PHY Control
    LINK_PHY_START = 0xC200
    LINK_PHY_END = 0xC2FF

    # NVMe Interface
    NVME_IF_START = 0xC400
    NVME_IF_END = 0xC5FF

    # PHY Extended
    PHY_EXT_START = 0xC600
    PHY_EXT_END = 0xC6FF

    # Interrupt / I2C / Flash / DMA
    INT_START = 0xC800
    INT_END = 0xC8FF

    # CPU Mode
    CPU_MODE_START = 0xCA00
    CPU_MODE_END = 0xCAFF

    # Timer / CPU Control
    TIMER_START = 0xCC00
    TIMER_END = 0xCCFF

    # SCSI DMA / Transfer Control
    SCSI_DMA_START = 0xCE00
    SCSI_DMA_END = 0xCEFF

    # USB Endpoint Buffer
    USB_EP_BUF_START = 0xD800
    USB_EP_BUF_END = 0xDFFF

    # PHY Completion / Debug
    PHY_DEBUG_START = 0xE300
    PHY_DEBUG_END = 0xE3FF

    # Command Engine
    CMD_ENGINE_START = 0xE400
    CMD_ENGINE_END = 0xE4FF

    # Debug/Interrupt Extended
    DEBUG_INT_START = 0xE600
    DEBUG_INT_END = 0xE6FF

    # System Status / Link Control
    SYS_STATUS_START = 0xE700
    SYS_STATUS_END = 0xE7FF

    # NVMe Event
    NVME_EVENT_START = 0xEC00
    NVME_EVENT_END = 0xECFF

    # System Control
    SYS_CTRL_START = 0xEF00
    SYS_CTRL_END = 0xEFFF

    # NVMe Data Buffer
    NVME_DATA_BUF_START = 0xF000
    NVME_DATA_BUF_END = 0xFFFF  # 4KB
