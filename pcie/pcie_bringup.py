#!/usr/bin/env python3
"""
PCIe Bringup for ASM2464PD via USB control transfers (0xE4/0xE5).

Replicates the full PCIe initialization sequence from pcie4_main.c,
driving the ASM2464PD's XDATA registers over USB from the host side.

The handmade/src/main.c firmware running on the device exposes:
  0xE4 (vendor read):  bmRequestType=0xC0, bRequest=0xE4, wValue=addr, wLength=size
  0xE5 (vendor write): bmRequestType=0x40, bRequest=0xE5, wValue=addr, wIndex=val

Usage:
    python3 pcie_bringup.py              # Full PCIe bringup
    python3 pcie_bringup.py --skip-hw    # Skip hw_init (if firmware already ran it)
    python3 pcie_bringup.py --verbose    # Verbose register trace
    python3 pcie_bringup.py --dry-run    # Print sequence without writing
"""

import ctypes
import sys
import time
import argparse

from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

# =============================================================================
# Device IDs
# =============================================================================
SUPPORTED_DEVICES = [
    (0xADD1, 0x0001),  # Custom firmware
    (0x174C, 0x2463),  # Stock 2463
    (0x174C, 0x2464),  # Stock 2464
]

# =============================================================================
# ASM2464PD XDATA Register Map (from src/include/registers.h)
# =============================================================================

# --- CPU Control / LTSSM Controller (CC3x) ---
CC30 = 0xCC30  # CPU_MODE: bit 0 = LTSSM enable
CC31 = 0xCC31  # CPU_TIMER_CTRL_CD31 (NOT CC31 — see CD31 below)
CC32 = 0xCC32  # CPU_EXEC_STATUS: CC3x write gate (0x01 to enable)
CC33 = 0xCC33  # CPU_EXEC_STATUS_2: interrupt pending (0x04)
CC35 = 0xCC35  # CPU_EXEC_STATUS_3: LTSSM timer (bit 0)
CC37 = 0xCC37  # CPU_CTRL_CC37: RXPLL reset (bit 2)
CC38 = 0xCC38  # TIMER_ENABLE_A: bit 1
CC39 = 0xCC39  # TIMER_CTRL: bit 1
CC3A = 0xCC3A  # TIMER_ENABLE_B: bit 1
CC3B = 0xCC3B  # TIMER_CTRL: bit 0=active, bit 1=SS link power, bit 6
CC3E = 0xCC3E  # CPU_CTRL: bits 0,1
CC3F = 0xCC3F  # LTSSM_CTRL
CC43 = 0xCC43  # CPU_CLK_CFG: 0x80 for PCIe PLL

# --- Timer Registers ---
CC10 = 0xCC10  # Timer 0 divisor
CC11 = 0xCC11  # Timer 0 CSR (0x01=enable, 0x02=expired, 0x04=clear)
CC12 = 0xCC12  # Timer 0 threshold high
CC13 = 0xCC13  # Timer 0 threshold low
CC16 = 0xCC16  # Timer 1 divisor
CC17 = 0xCC17  # Timer 1 CSR
CC18 = 0xCC18  # Timer 1 threshold high
CC19 = 0xCC19  # Timer 1 threshold low
CC1C = 0xCC1C  # Timer 2 divisor
CC1D = 0xCC1D  # Timer 2 CSR
CC1E = 0xCC1E  # Timer 2 threshold low
CC1F = 0xCC1F  # Timer 2 threshold high
CC22 = 0xCC22  # Timer 3 divisor
CC23 = 0xCC23  # Timer 3 CSR
CC2A = 0xCC2A  # CPU_KEEPALIVE: watchdog (write 0x0C)
CC2C = 0xCC2C  # Keepalive param
CC2D = 0xCC2D  # Keepalive param
CC5C = 0xCC5C  # Timer 4 divisor
CC5D = 0xCC5D  # Timer 4 CSR
CC5E = 0xCC5E  # Timer 4 threshold low
CC5F = 0xCC5F  # Timer 4 threshold high
CC80 = 0xCC80  # CPU control
CC82 = 0xCC82  # CPU control
CC83 = 0xCC83  # CPU control
CC90 = 0xCC90  # CPU DMA control

# --- CPU Mode / Control (CA0x) ---
CA06 = 0xCA06  # CPU_MODE_NEXT: bits 4,5,6 for PCIe generation
CA2E = 0xCA2E  # Controller bus config (bit 0)
CA60 = 0xCA60  # PD controller config
CA70 = 0xCA70  # CPU control (cleared in phy_soft_reset)
CA81 = 0xCA81  # PCIe init trigger (bit 0)

# --- System Status / Link Control (E7xx) ---
E710 = 0xE710  # Link width/recovery (bits 5-7=width, bits 0-4=config)
E712 = 0xE712  # PHY ready polling (bit 0, bit 1)
E716 = 0xE716  # Link status (bits 0-1: 0x03=PCIe mode)
E717 = 0xE717  # Link control (bit 0=enable)
E741 = 0xE741  # PHY PLL control
E742 = 0xE742  # PHY PLL config
E751 = 0xE751  # PHY poll alt (0x01 for EQ)
E764 = 0xE764  # PHY timer / link training trigger
E76C = 0xE76C  # System control (bit 4)
E774 = 0xE774  # System control (bit 4)
E77C = 0xE77C  # System control (bit 4)
E780 = 0xE780  # System control
E7E3 = 0xE7E3  # PHY link control
E762 = 0xE762  # PHY RXPLL status (bit 4=link ok)

# --- PHY Completion / Debug (E3xx) ---
E324 = 0xE324  # Link control (bit 2)

# --- PCIe TLP / Passthrough Engine (B2xx) ---
B210 = 0xB210  # TLP format/type
B213 = 0xB213  # TLP control
B216 = 0xB216  # TLP length / trigger
B217 = 0xB217  # Byte enable
B218 = 0xB218  # PCIe address byte 0
B219 = 0xB219  # PCIe address byte 1
B21A = 0xB21A  # PCIe address byte 2
B21B = 0xB21B  # PCIe address byte 3
B220 = 0xB220  # PCIe data byte 0
B221 = 0xB221  # PCIe data byte 1
B222 = 0xB222  # PCIe data byte 2
B223 = 0xB223  # PCIe data byte 3 / ext status
B22B = 0xB22B  # Completion status (0x04=link width x4)
B22C = 0xB22C  # Completion data
B22D = 0xB22D  # Completion data alt
B250 = 0xB250  # DMA config
B251 = 0xB251  # Doorbell command
B254 = 0xB254  # PCIe request trigger (0x0F)
B264 = 0xB264  # DMA size A
B265 = 0xB265  # DMA size B
B266 = 0xB266  # DMA size C
B267 = 0xB267  # DMA size D
B26C = 0xB26C  # DMA buffer A
B26D = 0xB26D  # DMA buffer B
B26E = 0xB26E  # DMA buffer C
B26F = 0xB26F  # DMA buffer D
B281 = 0xB281  # DMA control
B284 = 0xB284  # Completion status
B296 = 0xB296  # PCIe status (bit 0=err, 1=complete, 2=busy)
B298 = 0xB298  # TLP tunnel control (bit 4)
B2D5 = 0xB2D5  # PCIe config routing enable

# --- PCIe Tunnel Adapter / Bridge Config (B4xx) ---
B401 = 0xB401  # Tunnel control (bit 0)
B402 = 0xB402  # PCIe control (bit 1)
B403 = 0xB403  # Tunnel control (bit 0 in phase2)
B404 = 0xB404  # Link parameters (low nibble)
B410 = 0xB410  # Port 0 VID low
B411 = 0xB411  # Port 0 VID high
B412 = 0xB412  # Port 0 DID low
B413 = 0xB413  # Port 0 DID high
B415 = 0xB415  # Port 0 base class
B416 = 0xB416  # Port 0 sub class
B417 = 0xB417  # Port 0 prog interface
B418 = 0xB418  # Port 0 subsystem DID low
B419 = 0xB419  # Port 0 subsystem DID high
B41A = 0xB41A  # Port 0 subsystem VID low
B41B = 0xB41B  # Port 0 subsystem VID high
B420 = 0xB420  # Port 1 VID low
B421 = 0xB421  # Port 1 VID high
B422 = 0xB422  # Port 1 DID low
B423 = 0xB423  # Port 1 DID high
B425 = 0xB425  # Port 1 base class
B426 = 0xB426  # Port 1 sub class
B427 = 0xB427  # Port 1 prog interface
B428 = 0xB428  # Port 1 subsystem DID low
B429 = 0xB429  # Port 1 subsystem DID high
B42A = 0xB42A  # Port 1 subsystem VID low
B42B = 0xB42B  # Port 1 subsystem VID high
B430 = 0xB430  # Tunnel link state (bit 0)
B431 = 0xB431  # Tunnel link training status
B432 = 0xB432  # Power control for lanes
B434 = 0xB434  # PCIe link state (low nibble = lane enable mask)
B436 = 0xB436  # PCIe lane config
B450 = 0xB450  # LTSSM state
B455 = 0xB455  # LTSSM link detect trigger
B480 = 0xB480  # PERST# control (bit 0=assert)
B481 = 0xB481  # PCIe link control
B482 = 0xB482  # Tunnel adapter mode

# --- PHY Config (C2xx) ---
C233 = 0xC233  # PHY config (bits 0-1=mode, bit 2=toggle)

# --- NVMe (C4xx, used in post-train) ---
C428 = 0xC428  # NVMe queue config
C450 = 0xC450  # NVMe command status
C472 = 0xC472  # NVMe link control
C4EB = 0xC4EB  # NVMe parameter
C4ED = 0xC4ED  # NVMe DMA control

# --- PHY Extended / GPIO (C6xx) ---
C620 = 0xC620  # GPIO control 0
C62D = 0xC62D  # PHY ext lane
C655 = 0xC655  # PHY config
C656 = 0xC656  # HDDPC / 3.3V enable (bit 5)
C659 = 0xC659  # PCIe lane ctrl / 12V enable (bit 0)
C65A = 0xC65A  # PHY config
C65B = 0xC65B  # PHY extended
C6A8 = 0xC6A8  # PHY link state control (bit 0)

# --- Interrupt / DMA (C8xx) ---
C800 = 0xC800  # Interrupt status
C807 = 0xC807  # Interrupt/DMA control

# --- SCSI DMA / Link (CExx) ---
CEEF = 0xCEEF  # CPU link control
CEF0 = 0xCEF0  # CPU link status
CEF2 = 0xCEF2  # CPU link ready
CEF3 = 0xCEF3  # CPU link active

# --- CPU Extended / PHY DMA (CDxx) ---
CD30 = 0xCD30  # PHY DMA command
CD31 = 0xCD31  # CPU timer ctrl (0x04=clear, 0x02=start)
CD32 = 0xCD32  # PHY DMA address low
CD33 = 0xCD33  # PHY DMA address high

# --- UART (used for bus config, not serial) ---
C004 = 0xC004  # UART IIR
C007 = 0xC007  # UART LCR

# --- Power Management ---
P92C4 = 0x92C4  # Power misc control
P92C8 = 0x92C8  # Power/PHY control
P9201 = 0x9201  # USB control

# =============================================================================
# USB Access Layer
# =============================================================================

class ASM2464PD:
    """Low-level USB control transfer interface to ASM2464PD XDATA registers."""

    def __init__(self, verbose=False, dry_run=False):
        self.verbose = verbose
        self.dry_run = dry_run
        self.dev = None
        self._write_count = 0
        self._read_count = 0

    def open(self):
        """Open USB device."""
        for vid, pid in SUPPORTED_DEVICES:
            try:
                self.dev = USB3(vid, pid, 0x81, 0x83, 0x02, 0x04, use_bot=True)
                print(f"Opened device {vid:04X}:{pid:04X}")
                return
            except RuntimeError:
                pass
        raise RuntimeError("No ASM2464PD device found")

    def close(self):
        """Release USB interface."""
        if self.dev:
            libusb.libusb_release_interface(self.dev.handle, 0)
            libusb.libusb_close(self.dev.handle)
            self.dev = None

    def read(self, addr, size=1):
        """Read bytes from XDATA via 0xE4 vendor control transfer."""
        self._read_count += 1
        if self.dry_run:
            if self.verbose:
                print(f"  RD  0x{addr:04X} [{size}]  (dry-run)")
            return bytes(size)  # return zeros
        buf = (ctypes.c_ubyte * size)()
        ret = libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
        if ret < 0:
            raise IOError(f"E4 read(0x{addr:04X}, {size}) failed: {ret}")
        data = bytes(buf[:ret])
        if self.verbose:
            print(f"  RD  0x{addr:04X} = {data.hex()}")
        return data

    def read8(self, addr):
        """Read single byte from XDATA."""
        return self.read(addr, 1)[0]

    def write(self, addr, val):
        """Write single byte to XDATA via 0xE5 vendor control transfer."""
        self._write_count += 1
        if self.verbose:
            print(f"  WR  0x{addr:04X} = 0x{val:02X}")
        if self.dry_run:
            return
        ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        if ret < 0:
            raise IOError(f"E5 write(0x{addr:04X}, 0x{val:02X}) failed: {ret} (USB3 link may have died — register write killed SuperSpeed PHY)")

    def bank1_write(self, addr, val):
        """Write byte to XDATA bank 1 via E5 with wIndex high=1 (DPX bank select)."""
        self._write_count += 1
        if self.verbose:
            print(f"  B1W 0x{addr:04X} = 0x{val:02X}")
        if self.dry_run:
            return
        ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val | (1 << 8), None, 0, 1000)
        if ret < 0:
            raise IOError(f"E5 bank1_write(0x{addr:04X}, 0x{val:02X}) failed: {ret}")

    def bank1_read(self, addr, size=1):
        """Read bytes from XDATA bank 1 via E4 with wIndex high=1 (DPX bank select)."""
        self._read_count += 1
        if self.dry_run:
            return bytes(size)
        buf = (ctypes.c_ubyte * size)()
        ret = libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 1 << 8, buf, size, 1000)
        if ret < 0:
            raise IOError(f"E4 bank1_read(0x{addr:04X}, {size}) failed: {ret}")
        data = bytes(buf[:ret])
        if self.verbose:
            print(f"  B1R 0x{addr:04X} = {data.hex()}")
        return data

    def bank1_or_bits(self, addr, mask):
        """Read-modify-write in bank 1: val |= mask."""
        old = self.bank1_read(addr, 1)[0]
        self.bank1_write(addr, old | mask)

    def rmw(self, addr, clear_mask, set_mask):
        """Read-modify-write: val = (val & ~clear_mask) | set_mask."""
        val = self.read8(addr)
        new = (val & ~clear_mask) | set_mask
        self.write(addr, new & 0xFF)
        return new

    def set_bits(self, addr, mask):
        """Set bits in register."""
        return self.rmw(addr, 0, mask)

    def clear_bits(self, addr, mask):
        """Clear bits in register."""
        return self.rmw(addr, mask, 0)

    def poll_bit(self, addr, mask, value=None, timeout_ms=5000, interval_ms=1):
        """Poll register until (reg & mask) == value, or timeout."""
        if value is None:
            value = mask  # default: wait for bits to be set
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            v = self.read8(addr)
            if (v & mask) == value:
                return v
            time.sleep(interval_ms / 1000.0)
        raise TimeoutError(f"Timeout polling 0x{addr:04X} & 0x{mask:02X} == 0x{value:02X} (last=0x{v:02X})")


# =============================================================================
# PCIe Bringup Phases (matching pcie4_main.c)
# =============================================================================

def timer_wait(dev, threshold_hi, threshold_lo, mode, timeout_s=2.0):
    """Wait for hardware timer to expire (pcie4_main.c:1983).

    Programs Timer0 with given threshold/mode and polls until done.
    Since we're on the host side, we just sleep for an estimated duration
    AND poll the timer expired bit (CC11 bit 1).
    """
    # Reset timer
    dev.write(CC11, 0x04)  # clear
    dev.write(CC11, 0x02)  # clear done flag
    # Configure prescaler
    div = dev.read8(CC10)
    div = (div & 0xF8) | (mode & 0x07)
    dev.write(CC10, div)
    # Set threshold
    dev.write(CC12, threshold_hi)
    dev.write(CC13, threshold_lo)
    # Start timer
    dev.write(CC11, 0x01)
    # Poll until expired
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        csr = dev.read8(CC11)
        if csr & 0x02:
            break
        time.sleep(0.01)
    # Clear done
    dev.write(CC11, 0x02)


def phy_soft_reset(dev):
    """PHY soft reset via RXPLL mode toggle (pcie4_main.c:2460)."""
    print("  [phy_soft_reset]")
    # CC37 |= 0x04 — enable RXPLL reset mode
    dev.set_bits(CC37, 0x04)
    # CA70 = 0x00
    dev.write(CA70, 0x00)
    # E780 = 0x00
    dev.write(E780, 0x00)
    # E716 = 0x00, then 0x03 — skip on USB3, kills SuperSpeed link
    link = dev.read8(0x9100)
    if link < 0x02:  # USB2 or unconnected
        dev.write(E716, 0x00)
        dev.write(E716, 0x03)
    else:
        print("    (skipping E716 toggle — USB3 link active)")
    # Timer wait + E712 polling
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    div = dev.read8(CC10)
    dev.write(CC10, (div & 0xF8) | 0x02)
    dev.write(CC12, 0x00)
    dev.write(CC13, 0xC8)
    dev.write(CC11, 0x01)
    # Poll E712
    for _ in range(300):
        e712 = dev.read8(E712)
        if e712 & 0x03:
            break
        csr = dev.read8(CC11)
        if csr & 0x02:
            break
        time.sleep(0.01)
    # Clear timer
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    # CC37 &= ~0x04
    dev.clear_bits(CC37, 0x04)


def pcie_link_controller_init(dev):
    """PCIe link controller config (pcie4_main.c:2387 / stock CF28)."""
    print("  [pcie_link_controller_init]")
    # CC32 = 0x01 (gate enable)
    dev.write(CC32, 0x01)
    # CC30: DO NOT write — setting bit 0 starts LTSSM which causes ~10% USB read
    # failures on all subsequent control transfers. PCIe works without it.
    # E710 = (E710 & 0xE0) | 0x04
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | 0x04)
    # C6A8: set bit 0
    dev.set_bits(C6A8, 0x01)
    # CC33 = 0x04
    dev.write(CC33, 0x04)
    # E324: clear bit 2
    dev.clear_bits(E324, 0x04)
    # CC3B: clear bit 0
    dev.clear_bits(CC3B, 0x01)
    # E717: set bit 0
    dev.set_bits(E717, 0x01)
    # CC3E: clear bit 1
    dev.clear_bits(CC3E, 0x02)
    # CC3B: clear bit 1, then clear bit 6
    dev.clear_bits(CC3B, 0x02)
    dev.clear_bits(CC3B, 0x40)
    # Restore CC3B bits 0,1
    dev.set_bits(CC3B, 0x03)
    # E716 = (E716 & 0xFC) | 0x03
    e716 = dev.read8(E716)
    dev.write(E716, (e716 & 0xFC) | 0x03)
    # CC3E: clear bit 0
    dev.clear_bits(CC3E, 0x01)
    # CC39: set bit 1
    dev.set_bits(CC39, 0x02)
    # CC3A: clear bit 1; CC38: clear bit 1
    dev.clear_bits(CC3A, 0x02)
    dev.clear_bits(CC38, 0x02)
    # CA06 = (CA06 & 0x1F) | 0x60
    ca06 = dev.read8(CA06)
    dev.write(CA06, (ca06 & 0x1F) | 0x60)
    # CA81: set bit 0
    dev.set_bits(CA81, 0x01)


def pcie_link_controller_reinit(dev):
    """Link controller re-init for tunnel setup (pcie4_main.c:2077 / stock CE3D)."""
    print("  [pcie_link_controller_reinit]")
    # CC30: DO NOT write — causes USB flakiness (see pcie_link_controller_init)
    dev.set_bits(CA81, 0x01)
    # E710 = (E710 & 0xE0) | 0x04
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | 0x04)
    # C6A8 set bit 0, CA81 set bit 0
    dev.set_bits(C6A8, 0x01)
    dev.set_bits(CA81, 0x01)
    # CC33 = 0x04
    dev.write(CC33, 0x04)
    # E324: clear bit 2
    dev.clear_bits(E324, 0x04)
    # CC3B: clear bit 0
    dev.clear_bits(CC3B, 0x01)
    # E717: set bit 0, CA81 set bit 0
    dev.set_bits(E717, 0x01)
    dev.set_bits(CA81, 0x01)
    # CC3E: clear bit 1
    dev.clear_bits(CC3E, 0x02)
    # CC3B: clear bit 1, then clear bit 6
    dev.clear_bits(CC3B, 0x02)
    dev.clear_bits(CC3B, 0x40)
    # E716 = (E716 & 0xFC) | 0x03
    e716 = dev.read8(E716)
    dev.write(E716, (e716 & 0xFC) | 0x03)
    # CC3E: clear bit 0
    dev.clear_bits(CC3E, 0x01)
    # CC39: set bit 1
    dev.set_bits(CC39, 0x02)
    # CC3A: clear bit 1; CC38: clear bit 1
    dev.clear_bits(CC3A, 0x02)
    dev.clear_bits(CC38, 0x02)
    # CA06 = (CA06 & 0x1F) | 0x60; CA81 set bit 0
    ca06 = dev.read8(CA06)
    dev.write(CA06, (ca06 & 0x1F) | 0x60)
    dev.set_bits(CA81, 0x01)


def pcie_lane_config(dev, mask=0x0F):
    """Progressive PCIe lane enable (pcie4_main.c:2258)."""
    print(f"  [pcie_lane_config mask=0x{mask:02X}]")
    # Save B402 bit 1 and clear it
    b402_saved = dev.read8(B402) & 0x02
    dev.clear_bits(B402, 0x02)
    dev.clear_bits(B402, 0x02)  # redundant, matches stock

    if mask == 0x0F:
        # Progressive enable: 0x01 → 0x03 → 0x07 → 0x0F
        for step in [0x01, 0x03, 0x07, 0x0F]:
            b434 = dev.read8(B434)
            dev.write(B434, step | (b434 & 0xF0))
            # phy_link_training skipped (bank-switched, kills USB3)
            timer_wait(dev, 0x00, 0xC7, 0x02)
    else:
        # Progressive reconfiguration to target mask
        current = dev.read8(B434) & 0x0F
        counter = 0x01
        for _ in range(4):
            if current == mask:
                break
            new_state = current & (mask | (counter ^ 0x0F))
            current = new_state
            b434 = dev.read8(B434)
            dev.write(B434, new_state | (b434 & 0xF0))
            timer_wait(dev, 0x00, 0xC7, 0x02)
            counter = (counter << 1) & 0xFF

    if mask != 0x0F:
        # Pulse B401
        dev.set_bits(B401, 0x01)
        b401 = dev.read8(B401)
        dev.write(B401, b401 & 0xFE)

    # Restore B402 bit 1
    if b402_saved:
        dev.set_bits(B402, 0x02)

    # Configure B436
    b436 = dev.read8(B436)
    dev.write(B436, (b436 & 0xF0) | (mask & 0x0E))
    b404 = dev.read8(B404)
    upper = ((b404 & 0x0F) ^ 0x0F) << 4
    b436 = dev.read8(B436)
    dev.write(B436, (b436 & 0x0F) | (upper & 0xF0))


def pcie_pre_init(dev):
    """Full pre-init sequence (pcie4_main.c:2517 / stock CE79)."""
    print("[pcie_pre_init]")
    pcie_link_controller_init(dev)
    phy_soft_reset(dev)
    # C233 &= 0xFC
    dev.clear_bits(C233, 0x03)
    # C233 = (C233 & 0xFB) | 0x04
    # NOTE: setting bit 2 of C233 kills USB3 SuperSpeed link
    c233 = dev.read8(C233)
    link = dev.read8(0x9100)
    if link >= 0x02:  # USB3 active — skip PHY reset that kills link
        print("    (skipping C233 bit2 toggle — USB3 link active)")
    else:
        dev.write(C233, (c233 & 0xFB) | 0x04)
    # Timer wait ~20 ticks mode 2
    timer_wait(dev, 0x00, 0x14, 0x02)
    # C233 &= 0xFB — only needed if we set bit 2 above
    if link < 0x02:
        dev.clear_bits(C233, 0x04)
    # Timer start + E712 poll
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    div = dev.read8(CC10)
    dev.write(CC10, (div & 0xF8) | 0x03)
    dev.write(CC12, 0x00)
    dev.write(CC13, 0x0A)
    dev.write(CC11, 0x01)
    for _ in range(300):
        e712 = dev.read8(E712)
        if e712 & 0x03:
            break
        csr = dev.read8(CC11)
        if csr & 0x02:
            break
        time.sleep(0.01)
    # Clear timer
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    # E7E3 = 0x00
    dev.write(E7E3, 0x00)
    print("[pcie_pre_init done]")


def pcie_early_init(dev):
    """First-time PCIe PHY/controller init (pcie4_main.c:2590 / stock D996)."""
    print("[pcie_early_init]")
    # B402 &= 0xFD
    dev.clear_bits(B402, 0x02)
    # E764 reset: clear bits 1,0,3; set bit 2
    e764 = dev.read8(E764)
    e764 = e764 & 0xFD; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = e764 & 0xFE; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = e764 & 0xF7; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = (e764 & 0xFB) | 0x04; dev.write(E764, e764)
    # B432/B404 config
    b432 = dev.read8(B432)
    dev.write(B432, (b432 & 0xF8) | 0x07)
    b404 = dev.read8(B404)
    dev.write(B404, (b404 & 0xF0) | 0x01)
    # E76C/E774/E77C: clear bit 4
    dev.clear_bits(E76C, 0x10)
    dev.clear_bits(E774, 0x10)
    dev.clear_bits(E77C, 0x10)
    # Progressive lane config
    pcie_lane_config(dev, 0x0F)
    # phy_set_bit6 skipped (bank-switched)
    print("[pcie_early_init done]")


def pcie_post_early_cleanup(dev):
    """Clear CC32 gate (pcie4_main.c:2693)."""
    print("[pcie_post_early_cleanup]")
    dev.write(CC32, 0x00)


def pcie_pll_init(dev):
    """PHY PLL and LTSSM config (pcie4_main.c:2653 / stock 8E28)."""
    print("[pcie_pll_init]")
    # CC35 &= 0xFE
    dev.clear_bits(CC35, 0x01)
    # E741 read-modify-write sequence
    e741 = dev.read8(E741)
    dev.write(E741, (e741 & 0xF8) | 0x03)
    e741 = dev.read8(E741)
    dev.write(E741, (e741 & 0xC7) | 0x28)
    # E742: preserve upper bits
    e742 = dev.read8(E742)
    dev.write(E742, (e742 & 0xFC) | 0x03)
    e741 = dev.read8(E741)
    dev.write(E741, (e741 & 0x3F) | 0x80)
    # CC43 = 0x80
    dev.write(CC43, 0x80)
    # Power/GPIO init
    dev.set_bits(C65B, 0x08)
    dev.clear_bits(C656, 0x20)
    dev.set_bits(C65B, 0x20)
    c62d = dev.read8(C62D)
    dev.write(C62D, (c62d & 0xE0) | 0x07)
    # Bus config
    dev.set_bits(C004, 0x02)
    dev.clear_bits(C007, 0x08)
    dev.set_bits(CA2E, 0x01)
    print("[pcie_pll_init done]")


def pcie_power_enable(dev):
    """Enable 3.3V power (pcie4_main.c:2704 / stock E5CB)."""
    print("[pcie_power_enable]")
    c656 = dev.read8(C656)
    if not (c656 & 0x20):
        dev.write(C656, (c656 & 0xDF) | 0x20)
        dev.set_bits(C65B, 0x20)


def pcie_tunnel_setup(dev):
    """Configure PCIe tunnel (pcie4_main.c:2011 / stock CD6C)."""
    print("  [pcie_tunnel_setup]")
    # CA06 &= 0xEF
    dev.clear_bits(CA06, 0x10)
    # Adapter config B410-B42B
    dev.write(B410, 0x1B); dev.write(B411, 0x21)
    dev.write(B420, 0x1B); dev.write(B421, 0x21)
    dev.write(B412, 0x24); dev.write(B413, 0x63)
    dev.write(B422, 0x24); dev.write(B423, 0x63)
    dev.write(B415, 0x06); dev.write(B416, 0x04); dev.write(B417, 0x00)
    dev.write(B425, 0x06); dev.write(B426, 0x04); dev.write(B427, 0x00)
    dev.write(B41A, 0x1B); dev.write(B41B, 0x21)
    dev.write(B42A, 0x1B); dev.write(B42B, 0x21)
    dev.write(B418, 0x24); dev.write(B419, 0x63)
    dev.write(B428, 0x24); dev.write(B429, 0x63)
    # B401 |= 0x01
    dev.set_bits(B401, 0x01)
    # B482 |= 0x01, then (B482 & 0x0F) | 0xF0
    dev.set_bits(B482, 0x01)
    b482 = dev.read8(B482)
    dev.write(B482, (b482 & 0x0F) | 0xF0)
    # B401 &= 0xFE, B480 |= 0x01
    b401 = dev.read8(B401)
    dev.write(B401, b401 & 0xFE)
    dev.set_bits(B480, 0x01)
    # B430 &= 0xFE
    dev.clear_bits(B430, 0x01)
    # B298 = (B298 & 0xEF) | 0x10
    b298 = dev.read8(B298)
    dev.write(B298, (b298 & 0xEF) | 0x10)


def pcie_tunnel_setup_reinit(dev):
    """Full tunnel reinit: link controller + PHY reset + lane config (pcie4_main.c:2144)."""
    print("  [pcie_tunnel_setup_reinit]")
    pcie_link_controller_reinit(dev)
    phy_soft_reset(dev)
    # C233 &= 0xFC
    dev.clear_bits(C233, 0x03)
    # C233 = (C233 & 0xFB) | 0x04 — kills USB3
    c233 = dev.read8(C233)
    link = dev.read8(0x9100)
    if link >= 0x02:
        print("    (skipping C233 bit2 toggle — USB3 link active)")
    else:
        dev.write(C233, (c233 & 0xFB) | 0x04)
    timer_wait(dev, 0x00, 0x14, 0x02)
    # C233 &= 0xFB
    if link < 0x02:
        dev.clear_bits(C233, 0x04)
    # Timer start + E712 poll
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    div = dev.read8(CC10)
    dev.write(CC10, (div & 0xF8) | 0x03)
    dev.write(CC12, 0x00)
    dev.write(CC13, 0x0A)
    dev.write(CC11, 0x01)
    for _ in range(300):
        e712 = dev.read8(E712)
        if e712 & 0x03:
            break
        csr = dev.read8(CC11)
        if csr & 0x02:
            break
        time.sleep(0.01)
    dev.write(CC11, 0x04)
    dev.write(CC11, 0x02)
    # E7E3 = 0x00
    dev.write(E7E3, 0x00)
    # Re-run early_init equivalent (D9A4 path)
    dev.clear_bits(B402, 0x02)
    # E764 reset
    e764 = dev.read8(E764)
    e764 = e764 & 0xFD; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = e764 & 0xFE; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = e764 & 0xF7; dev.write(E764, e764)
    e764 = dev.read8(E764)
    e764 = (e764 & 0xFB) | 0x04; dev.write(E764, e764)
    # B432/B404
    b432 = dev.read8(B432)
    dev.write(B432, (b432 & 0xF8) | 0x07)
    b404 = dev.read8(B404)
    dev.write(B404, (b404 & 0xF0) | 0x01)
    dev.clear_bits(E76C, 0x10)
    dev.clear_bits(E774, 0x10)
    dev.clear_bits(E77C, 0x10)
    pcie_lane_config(dev, 0x0F)


def pcie_tunnel_enable(dev):
    """Full PCIe tunnel bringup (pcie4_main.c:2725 / stock C00D)."""
    print("[pcie_tunnel_enable]")
    # E710 re-init
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | 0x04)
    # Timer block init
    dev.write(CD31, 0x04); dev.write(CD31, 0x02)
    cd30 = dev.read8(CD30)
    dev.write(CD30, (cd30 & 0xF8) | 0x05)
    dev.write(CD32, 0x00); dev.write(CD33, 0xC7)
    cc2a = dev.read8(CC2A)
    dev.write(CC2A, (cc2a & 0xF8) | 0x04)
    dev.write(CC2C, 0xC7); dev.write(CC2D, 0xC7)
    # BFE0 wrapper: C6A8, 92C8, timers
    dev.set_bits(C6A8, 0x01)
    p92c8 = dev.read8(P92C8)
    dev.write(P92C8, p92c8 & 0xFC)
    dev.write(CD31, 0x04); dev.write(CD31, 0x02)
    # Timer 1
    dev.write(CC17, 0x04); dev.write(CC17, 0x02)
    div = dev.read8(CC16)
    dev.write(CC16, (div & 0xF8) | 0x04)
    dev.write(CC18, 0x01); dev.write(CC19, 0x90)
    p92c4 = dev.read8(P92C4)
    dev.write(P92C4, p92c4 & 0xFE)
    dev.set_bits(P9201, 0x01)
    dev.clear_bits(P9201, 0x01)
    # Timer 3
    dev.write(CC23, 0x04); dev.write(CC23, 0x02)
    div = dev.read8(CC22)
    dev.write(CC22, (div & 0xE8) | 0x07)
    # Timer 2 + Timer 4
    dev.write(CC1D, 0x04); dev.write(CC1D, 0x02)
    dev.write(CC5D, 0x04); dev.write(CC5D, 0x02)
    div = dev.read8(CC1C)
    dev.write(CC1C, (div & 0xF8) | 0x06)
    dev.write(CC1E, 0x00); dev.write(CC1F, 0x8B)
    div = dev.read8(CC5C)
    dev.write(CC5C, (div & 0xF8) | 0x04)
    dev.write(CC5E, 0x00); dev.write(CC5F, 0xC7)
    # GPIO/power config
    dev.set_bits(C655, 0x01)
    c620 = dev.read8(C620)
    dev.write(C620, c620 & 0xE0)
    dev.set_bits(C65A, 0x01)
    # 3.3V power
    pcie_power_enable(dev)
    # Tunnel setup
    pcie_tunnel_setup(dev)
    # DMA config
    dev.write(B264, 0x08); dev.write(B265, 0x00)
    dev.write(B266, 0x08); dev.write(B267, 0x08)
    dev.write(B26C, 0x08); dev.write(B26D, 0x20)
    dev.write(B26E, 0x08); dev.write(B26F, 0x28)
    dev.write(B250, 0x00); dev.write(B251, 0x00)
    # CEF3/CEF2/CEF0/CEEF
    dev.write(CEF3, 0x08); dev.write(CEF2, 0x80)
    dev.clear_bits(CEF0, 0x08)
    dev.clear_bits(CEEF, 0x80)
    # C807/B281
    dev.set_bits(C807, 0x04)
    b281 = dev.read8(B281)
    dev.write(B281, (b281 & 0xCF) | 0x10)
    # B401 pulse
    dev.set_bits(B401, 0x01)
    b401 = dev.read8(B401)
    dev.write(B401, b401 & 0xFE)
    # Full tunnel reinit
    pcie_tunnel_setup_reinit(dev)
    # CA06 &= 0xEF, B480 |= 0x01
    dev.clear_bits(CA06, 0x10)
    dev.set_bits(B480, 0x01)
    # C659 &= 0xFE — 12V OFF during tunnel config
    dev.clear_bits(C659, 0x01)
    # Lane config
    pcie_lane_config(dev, 0x0F)
    print("[pcie_tunnel_enable done]")


def pcie_phy_channel_config(dev, mode):
    """PCIe config space access engine (pcie4_main.c:3027 / stock BF96).
    mode=0: read, mode=1: write. Address set via XDATA[0x05AC-0x05AF]."""
    # Set mode flag
    dev.write(0x05AB, mode)
    # Clear B210-B21B
    for i in range(12):
        dev.write(B210 + i, 0x00)
    # Set format type
    if mode & 0x01:
        dev.write(B210, 0x40)
    else:
        dev.write(B210, 0x00)
    # B213=0x01, B217=0x0F, B216=0x20
    dev.write(B213, 0x01)
    dev.write(B217, 0x0F)
    dev.write(B216, 0x20)
    # Copy address from 0x05AC-0x05AF
    dev.write(B218, dev.read8(0x05AC))
    dev.write(B219, dev.read8(0x05AD))
    dev.write(B21A, dev.read8(0x05AE))
    dev.write(B21B, dev.read8(0x05AF))
    # Trigger
    dev.write(B296, 0x01)
    dev.write(B296, 0x02)
    dev.write(B296, 0x04)
    dev.write(B254, 0x0F)
    # Poll B296 bit 2
    for _ in range(100):
        if dev.read8(B296) & 0x04:
            break
        time.sleep(0.001)
    dev.write(B296, 0x04)
    if mode & 0x01:
        return 0
    # Read mode: check result
    for _ in range(100):
        b296 = dev.read8(B296)
        if b296 & 0x02:
            dev.write(B296, 0x02)
            if dev.read8(B22C) != 0x00:
                return 0xFF
            if dev.read8(B22D) != 0x00:
                return 0xFF
            if dev.read8(B22B) != 0x04:
                return 0xFF
            return 0
        if b296 & 0x01:
            dev.write(B296, 0x01)
            return 0xFE
        time.sleep(0.001)
    return 0xFE


def pcie_check_readiness(dev):
    """Check PCIe link readiness (pcie4_main.c:3124 / stock E275)."""
    # Address = 0x00D0001C
    dev.write(0x05AC, 0x00)  # addr_offset_lo
    dev.write(0x05AD, 0xD0)  # addr_offset_hi
    dev.write(0x05AE, 0x00)  # direction
    dev.write(0x05AF, 0x1C)  # addr_0
    result = pcie_phy_channel_config(dev, 0)
    if result != 0:
        return False
    raw = dev.read8(B223)
    field = (raw >> 1) & 0x03
    print(f"  B223=0x{raw:02X} field={field}")
    return field == 0x02


def pcie_link_train_trigger(dev):
    """PCIe link training (pcie4_main.c:3173 / stock AC08)."""
    print("  [pcie_link_train_trigger]")
    # Clear B210-B21B
    for i in range(12):
        dev.write(B210 + i, 0x00)
    # B210=0x44 (Gen4 speed)
    dev.write(B210, 0x44)
    dev.write(B213, 0x01)
    dev.write(B217, 0x03)
    dev.write(B218, 0x00); dev.write(B219, 0x00)
    b21a = dev.read8(B21A)
    dev.write(B21A, (b21a & 0xF0) | 0x00)
    b21b = dev.read8(B21B)
    dev.write(B21B, (b21b & 0x03) | 0x04)
    dev.write(B216, 0x20)
    # Trigger
    dev.write(B296, 0x01); dev.write(B296, 0x02); dev.write(B296, 0x04)
    dev.write(B254, 0x0F)
    # Poll completion
    for _ in range(100):
        if dev.read8(B296) & 0x04:
            break
        time.sleep(0.001)
    dev.write(B296, 0x04)
    # Check result
    for _ in range(100):
        b296 = dev.read8(B296)
        if b296 & 0x02:
            dev.write(B296, 0x02)
            if dev.read8(B22C) != 0x00:
                print("  [LT: B22C error]")
                return False
            if dev.read8(B22D) != 0x00:
                print("  [LT: B22D error]")
                return False
            cpl = dev.read8(B22B)
            if cpl != 0x04:
                print(f"  [LT: B22B=0x{cpl:02X} != 0x04]")
                return False
            print("  [LT: success]")
            return True
        if b296 & 0x01:
            dev.write(B296, 0x01)
            print("  [LT: timeout]")
            return False
        time.sleep(0.001)
    print("  [LT: poll timeout]")
    return False


def pcie_perst_deassert(dev):
    """PERST deassert with link training (pcie4_main.c:3286 / stock 35DF)."""
    print("[pcie_perst_deassert]")
    # Pre-PERST
    dev.write(B455, 0x02); dev.write(B455, 0x04)
    dev.write(B2D5, 0x01)
    dev.write(B296, 0x08)
    # Config space address 0x00D00014
    dev.write(0x05AC, 0x00); dev.write(0x05AD, 0xD0)
    dev.write(0x05AE, 0x00); dev.write(0x05AF, 0x14)
    # B220 TLP config
    dev.write(B220, 0x00); dev.write(B221, 0x46)
    dev.write(B222, 0x40); dev.write(B223, 0x01)
    # Config space write
    pcie_phy_channel_config(dev, 1)
    # Readiness poll
    ready = False
    for _ in range(5):
        timer_wait(dev, 0x00, 0xC7, 0x04)
        if pcie_check_readiness(dev):
            ready = True
            print("  [READY]")
            break
    if not ready:
        print("  [NOT READY — continuing anyway]")
    # PERST deassert
    dev.clear_bits(B480, 0x01)
    print("  [PERST deasserted]")
    # TLP config
    dev.write(B220, 0x00); dev.write(B221, 0x00)
    dev.write(B222, 0x00); dev.write(B223, 0x0B)
    # Link training
    lt_ok = pcie_link_train_trigger(dev)
    # Poll B455 bit 1 (link detect)
    detected = False
    for _ in range(300):
        if dev.read8(B455) & 0x02:
            dev.write(B455, 0x02)
            detected = True
            break
        time.sleep(0.01)
    if detected:
        print("  [Link detected]")
    else:
        print("  [Link detect timeout]")
    # Post-link-detect
    timer_wait(dev, 0x00, 0x32, 0x04)
    if dev.read8(B2D5) & 0x01:
        dev.write(B2D5, 0x01)
    dev.write(B296, 0x08)
    print("[pcie_perst_deassert done]")
    return detected


def pcie_phase2(dev):
    """Phase 2 PCIe link retraining for Gen3 (pcie4_main.c:2895 / stock 9078)."""
    print("[pcie_phase2]")
    # Re-ensure E710 (CC30 NOT written — causes USB flakiness)
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | 0x04)
    # Timer wait ~300ms
    timer_wait(dev, 0x01, 0x2B, 0x04)
    # A840: CA81 clear, CA06 Gen3, B403 set
    dev.clear_bits(CA81, 0x01)
    ca06 = dev.read8(CA06)
    dev.write(CA06, (ca06 & 0x1F) | 0x20)
    dev.set_bits(B403, 0x01)
    # Lane mask 0x0C (lanes 2,3 = Gen3 x2)
    b431 = dev.read8(B431)
    dev.write(B431, (b431 & 0xF0) | 0x0C)
    pcie_lane_config(dev, 0x0C)
    # LTSSM equalization
    saved_e710 = dev.read8(E710) & 0x1F
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | 0x1F)
    saved_ca06 = dev.read8(CA06) & 0xE0
    dev.write(E751, 0x01)
    # E764 training trigger
    e764 = dev.read8(E764)
    dev.write(E764, (e764 & 0xF7) | 0x08)
    e764 = dev.read8(E764)
    dev.write(E764, e764 & 0xFB)
    e764 = dev.read8(E764)
    dev.write(E764, e764 & 0xFE)
    e764 = dev.read8(E764)
    dev.write(E764, (e764 & 0xFD) | 0x02)
    # 2s link training wait
    timer_wait(dev, 0x07, 0xCF, 0x01, timeout_s=5.0)
    # Check link
    rxpll = dev.read8(E762)
    link_ok = bool(rxpll & 0x10)
    print(f"  RXPLL status: 0x{rxpll:02X}  link_ok={link_ok}")
    if link_ok:
        e764 = dev.read8(E764)
        dev.write(E764, (e764 & 0xFE) | 0x01)
        e764 = dev.read8(E764)
        dev.write(E764, e764 & 0xFD)
    else:
        e764 = dev.read8(E764)
        dev.write(E764, e764 & 0xF7)
        e764 = dev.read8(E764)
        dev.write(E764, e764 & 0xFB)
        e764 = dev.read8(E764)
        dev.write(E764, e764 & 0xFE)
        e764 = dev.read8(E764)
        dev.write(E764, e764 & 0xFD)
    # Restore
    e710 = dev.read8(E710)
    dev.write(E710, (e710 & 0xE0) | saved_e710)
    ca06 = dev.read8(CA06)
    dev.write(CA06, (ca06 & 0x1F) | saved_ca06)
    dev.clear_bits(CA81, 0x01)
    # 12V ON
    dev.set_bits(C659, 0x01)
    # ~1s settling
    timer_wait(dev, 0x03, 0xE7, 0x04, timeout_s=5.0)
    print(f"[pcie_phase2 done — link_ok={link_ok}]")
    return link_ok


def pcie_bridge_config_init(dev):
    """Bridge config space shadow registers (pcie4_main.c:3416 / stock CC83)."""
    print("  [pcie_bridge_config_init]")
    # CA06 clear bit 4
    dev.clear_bits(CA06, 0x10)
    # Port 0: VID=0x1B21, DID=0x2463, class=0x060400
    dev.write(B410, 0x1B); dev.write(B411, 0x21)
    dev.write(B412, 0x24); dev.write(B413, 0x63)
    dev.write(B415, 0x06); dev.write(B416, 0x04); dev.write(B417, 0x00)
    dev.write(B418, 0x24); dev.write(B419, 0x63)
    dev.write(B41A, 0x1B); dev.write(B41B, 0x21)
    # Port 1: same
    dev.write(B420, 0x1B); dev.write(B421, 0x21)
    dev.write(B422, 0x24); dev.write(B423, 0x63)
    dev.write(B425, 0x06); dev.write(B426, 0x04); dev.write(B427, 0x00)
    dev.write(B428, 0x24); dev.write(B429, 0x63)
    dev.write(B42A, 0x1B); dev.write(B42B, 0x21)
    # B401 set bit 0, B482 set bit 0
    dev.set_bits(B401, 0x01)
    dev.set_bits(B482, 0x01)
    # B482 upper nibble
    b482 = dev.read8(B482)
    dev.write(B482, (b482 & 0x0F) | 0xF0)
    # B401 clear bit 0, B480 set bit 0
    dev.clear_bits(B401, 0x01)
    dev.set_bits(B480, 0x01)
    # B430 clear bit 0
    dev.clear_bits(B430, 0x01)
    # B298 set bit 4
    b298 = dev.read8(B298)
    dev.write(B298, (b298 & 0xEF) | 0x10)
    # CA06 clear bit 4 again, B480 set bit 0 again
    dev.clear_bits(CA06, 0x10)
    dev.set_bits(B480, 0x01)
    # Bank-switched PHY writes for PCIe switch TLP forwarding.
    # These use 0xEF (bank 1 write) which sets DPX=1 on the 8051 CPU.
    dev.bank1_write(0x4084, 0x22)  # switch port 0 PHY
    dev.bank1_write(0x5084, 0x22)  # switch port 1 PHY
    dev.bank1_write(0x6043, 0x70)  # TLP routing config
    dev.bank1_or_bits(0x6025, 0x80)  # TLP routing enable (set bit 7)
    # B481
    b481 = dev.read8(B481)
    dev.write(B481, (b481 & 0xFC) | 0x03)


def pcie_post_train(dev):
    """Post-link-training config (pcie4_main.c:3494)."""
    print("[pcie_post_train]")
    # 12V ON
    dev.set_bits(C659, 0x01)
    # DMA config
    dev.write(B264, 0x08); dev.write(B265, 0x00)
    dev.write(B266, 0x08); dev.write(B267, 0x08)
    dev.write(B26C, 0x08); dev.write(B26D, 0x20)
    dev.write(B26E, 0x08); dev.write(B26F, 0x28)
    dev.write(B250, 0x00); dev.write(B251, 0x00)
    # NVMe registers
    dev.set_bits(C428, 0x20)
    dev.set_bits(C450, 0x04)
    dev.clear_bits(C472, 0x01)
    dev.set_bits(C4EB, 0x01)
    dev.set_bits(C4ED, 0x01)
    dev.clear_bits(CA06, 0x40)
    # Bridge config
    pcie_bridge_config_init(dev)
    # Deassert PERST — bridge_config_init re-asserts it for internal config
    dev.clear_bits(B480, 0x01)
    time.sleep(0.1)  # Wait for GPU to exit reset
    # B455 trigger for TLP forwarding
    dev.write(B455, 0x02); dev.write(B455, 0x04)
    dev.write(B2D5, 0x01); dev.write(B296, 0x08)
    # Poll B455 bit 1
    for _ in range(200):
        if dev.read8(B455) & 0x02:
            dev.write(B455, 0x02)
            break
        time.sleep(0.005)
    print("[pcie_post_train done]")


# =============================================================================
# Status / Diagnostics
# =============================================================================

def print_pcie_status(dev):
    """Read and display key PCIe status registers."""
    print("\n=== PCIe Status ===")
    ltssm = dev.read8(B450)
    b22b = dev.read8(B22B)
    b434 = dev.read8(B434)
    b455 = dev.read8(B455)
    e710 = dev.read8(E710)
    cc30 = dev.read8(CC30)
    ca06 = dev.read8(CA06)
    b480 = dev.read8(B480)
    c659 = dev.read8(C659)
    link_up = ltssm in (0x48, 0x78)
    print(f"  LTSSM state (B450): 0x{ltssm:02X}  {'(L0!)' if link_up else '(Detect)' if ltssm == 0x00 else ''}")
    print(f"  Link width  (B22B): 0x{b22b:02X}  {'(x4)' if b22b == 0x04 else ''}")
    print(f"  Lane enable (B434): 0x{b434:02X}")
    print(f"  Link detect (B455): 0x{b455:02X}")
    print(f"  Link width  (E710): 0x{e710:02X}")
    print(f"  CPU mode    (CC30): 0x{cc30:02X}")
    print(f"  CPU next    (CA06): 0x{ca06:02X}")
    print(f"  PERST ctrl  (B480): 0x{b480:02X}  {'(asserted)' if b480 & 0x01 else '(deasserted)'}")
    print(f"  12V enable  (C659): 0x{c659:02X}  {'(on)' if c659 & 0x01 else '(off)'}")
    print(f"  Link status: {'UP' if link_up else 'DOWN'}")
    return ltssm, b22b


# =============================================================================
# Main Bringup Orchestration
# =============================================================================

def full_bringup(dev, skip_hw=False):
    """Run the full PCIe bringup sequence matching pcie4_main.c:main()."""
    print("=" * 60)
    print("ASM2464PD PCIe Bringup via USB E4/E5")
    print("=" * 60)

    if not skip_hw:
        # NOTE: hw_init() is NOT replicated here because the firmware
        # (handmade/src/main.c) already runs minimal hardware init at boot.
        # The clean/src/main.c firmware also runs its own hw_init().
        # If needed, hw_init() register writes can be done here, but most
        # of them configure USB/interrupt state which is already active.
        print("\n[Skipping hw_init — firmware handles this at boot]")

    print("\n--- Phase 1: Pre-Init ---")
    # Set CA06 bits 5,6 and CA81 bit 0 (stock pcie_config_init)
    ca06 = dev.read8(CA06)
    dev.write(CA06, (ca06 & 0x1F) | 0x60)
    dev.set_bits(CA81, 0x01)
    e716 = dev.read8(E716)
    dev.write(E716, (e716 & 0xFC) | 0x03)

    pcie_pre_init(dev)
    pcie_early_init(dev)
    pcie_post_early_cleanup(dev)

    print("\n--- Phase 2: Power Enable ---")
    pcie_power_enable(dev)
    dev.set_bits(C659, 0x01)  # 12V ON

    print("\n--- Phase 3: Tunnel Enable ---")
    pcie_tunnel_enable(dev)

    print("\n--- Phase 4: PERST Deassert ---")
    link_detected = pcie_perst_deassert(dev)

    print("\n--- Phase 5: Gen3 Retraining ---")
    link_ok = pcie_phase2(dev)

    print("\n--- Phase 6: Post-Train ---")
    b22b = dev.read8(B22B)
    if link_ok or b22b == 0x04:
        pcie_post_train(dev)
        print("\n*** PCIe link is UP! ***")
    else:
        print(f"\n*** PCIe link FAILED (link_ok={link_ok}, B22B=0x{b22b:02X}) ***")

    # Final status
    ltssm, width = print_pcie_status(dev)

    print(f"\nTotal: {dev._write_count} writes, {dev._read_count} reads")
    return link_ok or b22b == 0x04


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="PCIe bringup for ASM2464PD via USB E4/E5")
    parser.add_argument("--skip-hw", action="store_true", help="Skip hw_init (firmware already ran it)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose register trace")
    parser.add_argument("--dry-run", action="store_true", help="Print sequence without writing")
    parser.add_argument("--status-only", action="store_true", help="Only read and print PCIe status")
    args = parser.parse_args()

    dev = ASM2464PD(verbose=args.verbose, dry_run=args.dry_run)
    dev.open()

    try:
        if args.status_only:
            print_pcie_status(dev)
        else:
            ok = full_bringup(dev, skip_hw=args.skip_hw)
            sys.exit(0 if ok else 1)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
