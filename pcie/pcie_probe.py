#!/usr/bin/env python3
"""
PCIe Probe for ASM2464PD — enumerate devices, set up BARs, read GPU registers.

Assumes pcie_bringup.py has already been run (LTSSM in L0).
Uses 0xE4/0xE5 USB control transfers (not BOT) to drive the B2xx TLP engine.

Usage:
    python3 pcie/pcie_bringup.py && python3 pcie/pcie_probe.py
    python3 pcie/pcie_probe.py --gpu-bus 4       # default
    python3 pcie/pcie_probe.py --verbose
"""

import ctypes
import struct
import sys
import time
import argparse

from tinygrad.runtime.autogen import libusb

# =============================================================================
# USB transport — raw 0xE4/0xE5 control transfers
# =============================================================================

VID, PID = 0xADD1, 0x0001

def usb_open():
    """Open the USB device, return (handle, ctx)."""
    ctx = ctypes.POINTER(libusb.libusb_context)()
    libusb.libusb_init(ctypes.byref(ctx))
    handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
    if not handle:
        raise RuntimeError(f"Device {VID:04X}:{PID:04X} not found")
    libusb.libusb_claim_interface(handle, 0)
    return handle, ctx

def usb_close(handle, ctx):
    libusb.libusb_release_interface(handle, 0)
    libusb.libusb_close(handle)
    libusb.libusb_exit(ctx)

def xdata_read(handle, addr, size=1):
    """Read bytes from XDATA via 0xE4."""
    buf = (ctypes.c_ubyte * size)()
    ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
    assert ret >= 0, f"E4 read 0x{addr:04X} failed: {ret}"
    return bytes(buf[:ret])

def xdata_write(handle, addr, val):
    """Write single byte to XDATA via 0xE5."""
    ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, addr, val, None, 0, 1000)
    assert ret >= 0, f"E5 write 0x{addr:04X}=0x{val:02X} failed: {ret}"

def xdata_write_bytes(handle, base, data):
    """Write multiple bytes to consecutive XDATA addresses."""
    for i, b in enumerate(data):
        xdata_write(handle, base + i, b)

# =============================================================================
# PCIe TLP engine via 0xF0 control message
# =============================================================================

# TLP format/type codes
CFGRD0 = 0x04  # Config Read Type 0 (local, bus 0)
CFGRD1 = 0x05  # Config Read Type 1 (routed, bus > 0)
CFGWR0 = 0x44  # Config Write Type 0
CFGWR1 = 0x45  # Config Write Type 1
MRD32  = 0x20  # Memory Read 32-bit
MWR32  = 0x60  # Memory Write 32-bit


def pcie_request(handle, fmt_type, address, value=None, size=4, verbose=False, retries=10):
    """Send a PCIe TLP via 0xF0 control message.

    OUT phase (0x40): SETUP wValue = fmt_type | (byte_enable << 8).
      DATA payload (12 bytes): addr_lo[4] LE + addr_hi[4] LE + value[4] BE.
      Firmware writes B210/B217 from wValue, then programs B218-B21F and
      B220-B223 from payload, clears stale status, arms, and triggers.

    IN phase (0xC0): Firmware polls B296 on-chip in tight loop, returns 8 bytes:
      [0-3] B220-B223 completion data, [4-5] B22A-B22B completion header,
      [6] B284 completion type, [7] status (0=ok, 1=UR, 0xFF=timeout).
    """
    masked = address & 0xFFFFFFFC
    offset = address & 0x3
    assert size + offset <= 4
    be = ((1 << size) - 1) << offset
    shifted = ((value << (8 * offset)) & 0xFFFFFFFF) if value is not None else 0

    # OUT: addr_lo[4] LE + addr_hi[4] LE + value[4] BE
    payload = struct.pack('<II', masked, address >> 32) + struct.pack('>I', shifted)
    buf_out = (ctypes.c_ubyte * 12)(*payload)
    ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, fmt_type | (be << 8), 0, buf_out, 12, 5000)
    if ret < 0:
        raise IOError(f"F0 OUT failed: {ret}")

    # Posted writes: no completion expected
    is_write = ((fmt_type & 0xDF) == 0x40) or ((fmt_type & 0xB8) == 0x30)
    if is_write:
        return None

    # IN: poll + read 8-byte result
    buf_in = (ctypes.c_ubyte * 8)()
    ret = libusb.libusb_control_transfer(handle, 0xC0, 0xF0, 0, 0, buf_in, 8, 5000)
    if ret < 0:
        raise IOError(f"F0 IN failed: {ret}")

    fw_status = buf_in[7]
    if fw_status == 0x01:
        # Firmware saw B296 error bit — retry (link glitch)
        if retries > 0:
            return pcie_request(handle, fmt_type, address, value, size, verbose, retries - 1)
        raise RuntimeError(f"Unsupported Request at 0x{address:08X} (fmt=0x{fmt_type:02X})")
    if fw_status == 0xFF:
        raise TimeoutError(f"PCIe completion timeout at 0x{address:08X} (fmt=0x{fmt_type:02X})")

    # Validate PCIe completion header (B22A:B22B) and completion type (B284)
    is_cfg = (fmt_type & 0xBE) == 0x04
    cpl_header = struct.unpack('>H', bytes(buf_in[4:6]))[0]
    b284 = buf_in[6]
    cpl_status = (cpl_header >> 13) & 0x7
    cpl_byte_count = cpl_header & 0xFFF
    expected_bc = 4 if is_cfg else size

    CPL_STATUS_MAP = {
        0b001: f"Unsupported Request at 0x{address:08X}",
        0b010: f"Configuration Request Retry Status at 0x{address:08X}",
        0b100: f"Completer Abort at 0x{address:08X}",
    }

    if cpl_status:
        msg = CPL_STATUS_MAP.get(cpl_status, f"Reserved completion status 0b{cpl_status:03b} at 0x{address:08X}")
        if cpl_status == 0b001 and retries > 0:
            return pcie_request(handle, fmt_type, address, value, size, verbose, retries - 1)
        raise RuntimeError(msg)

    # For config requests, validate B284 consistency:
    #   reads should have B284 bit 0 set (CplD), writes should not (Cpl)
    if is_cfg:
        if value is None and not (b284 & 0x01):
            raise RuntimeError(f"Config read at 0x{address:08X}: expected CplD (B284 bit 0), got 0x{b284:02X}")
        if value is not None and (b284 & 0x01):
            raise RuntimeError(f"Config write at 0x{address:08X}: unexpected CplD (B284=0x{b284:02X})")

    if cpl_byte_count != expected_bc:
        raise RuntimeError(f"Completion byte count mismatch at 0x{address:08X}: "
                           f"expected {expected_bc}, got {cpl_byte_count}")

    # Extract completion data
    raw = struct.unpack('>I', bytes(buf_in[0:4]))[0]
    result = (raw >> (8 * offset)) & ((1 << (8 * size)) - 1)
    if verbose:
        print(f"  PCIe {'cfg' if is_cfg else 'mem'} read 0x{address:08X} = 0x{result:0{size*2}X}")
    return result


def pcie_cfg_read(handle, byte_addr, bus=0, dev=0, fn=0, size=4, verbose=False):
    """PCIe config space read."""
    assert byte_addr < 4096 and bus < 256 and dev < 32 and fn < 8
    fmt = CFGRD1 if bus > 0 else CFGRD0
    address = (bus << 24) | (dev << 19) | (fn << 16) | (byte_addr & 0xFFF)
    return pcie_request(handle, fmt, address, size=size, verbose=verbose)


def pcie_cfg_write(handle, byte_addr, value, bus=0, dev=0, fn=0, size=4, verbose=False):
    """PCIe config space write."""
    assert byte_addr < 4096 and bus < 256 and dev < 32 and fn < 8
    fmt = CFGWR1 if bus > 0 else CFGWR0
    address = (bus << 24) | (dev << 19) | (fn << 16) | (byte_addr & 0xFFF)
    return pcie_request(handle, fmt, address, value=value, size=size, verbose=verbose)


def pcie_mem_read(handle, address, size=4, verbose=False):
    """PCIe memory read (32-bit address)."""
    return pcie_request(handle, MRD32, address, size=size, verbose=verbose)


def pcie_mem_write(handle, address, value, size=4, verbose=False):
    """PCIe memory write (32-bit address)."""
    return pcie_request(handle, MWR32, address, value=value, size=size, verbose=verbose)


# =============================================================================
# PCI constants
# =============================================================================

PCI_VENDOR_ID        = 0x00
PCI_DEVICE_ID        = 0x02
PCI_COMMAND          = 0x04
PCI_STATUS           = 0x06
PCI_REVISION_ID      = 0x08
PCI_CLASS_PROG       = 0x09
PCI_CLASS_DEVICE     = 0x0A
PCI_HEADER_TYPE      = 0x0E
PCI_BASE_ADDRESS_0   = 0x10
PCI_PRIMARY_BUS      = 0x18
PCI_SECONDARY_BUS    = 0x19
PCI_SUBORDINATE_BUS  = 0x1A
PCI_MEMORY_BASE      = 0x20
PCI_MEMORY_LIMIT     = 0x22
PCI_PREF_MEMORY_BASE = 0x24
PCI_PREF_MEMORY_LIMIT = 0x26
PCI_PREF_BASE_UPPER32 = 0x28
PCI_PREF_LIMIT_UPPER32 = 0x2C
PCI_COMMAND_IO       = 0x01
PCI_COMMAND_MEMORY   = 0x02
PCI_COMMAND_MASTER   = 0x04
PCI_EXT_CAP_ID_REBAR = 0x15

VENDOR_NAMES = {
    0x1002: "AMD/ATI", 0x10DE: "NVIDIA", 0x8086: "Intel", 0x1022: "AMD",
    0x174C: "ASMedia", 0x1B21: "ASMedia",
}

CLASS_NAMES = {
    0x00: "Unclassified", 0x01: "Storage", 0x02: "Network", 0x03: "Display",
    0x06: "Bridge", 0x0C: "Serial Bus", 0x12: "Accelerator",
}


# =============================================================================
# Bus enumeration
# =============================================================================

def setup_bridges(handle, gpu_bus, mem_base=0x10000000, pref_mem_base=0x800000000, verbose=False):
    """Discover bridge topology incrementally and configure bus numbers.

    Instead of blindly writing bus numbers to all buses (which corrupts non-bridge
    devices), we walk one hop at a time: configure bus N's bridge, scan bus N+1,
    check if it's another bridge, repeat.
    """
    print(f"Setting up bridges (target gpu_bus={gpu_bus})...")
    next_bus = 1

    for bus in range(gpu_bus):
        # Configure this bridge: primary=bus, secondary=next_bus, subordinate=gpu_bus
        buses = (bus << 0) | (next_bus << 8) | (gpu_bus << 16)
        pcie_cfg_write(handle, PCI_PRIMARY_BUS, buses, bus=bus, size=4, verbose=verbose)

        pcie_cfg_write(handle, PCI_MEMORY_BASE, (mem_base >> 16) & 0xFFFF, bus=bus, size=2, verbose=verbose)
        pcie_cfg_write(handle, PCI_MEMORY_LIMIT, 0xFFFF, bus=bus, size=2, verbose=verbose)
        pcie_cfg_write(handle, PCI_PREF_MEMORY_BASE, (pref_mem_base >> 16) & 0xFFFF, bus=bus, size=2, verbose=verbose)
        pcie_cfg_write(handle, PCI_PREF_MEMORY_LIMIT, 0xFFFF, bus=bus, size=2, verbose=verbose)
        pcie_cfg_write(handle, PCI_PREF_BASE_UPPER32, pref_mem_base >> 32, bus=bus, size=4, verbose=verbose)
        pcie_cfg_write(handle, PCI_PREF_LIMIT_UPPER32, 0xFFFFFFFF, bus=bus, size=4, verbose=verbose)

        pcie_cfg_write(handle, PCI_COMMAND, PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
                        bus=bus, size=1, verbose=verbose)

        # Scan what appeared on next_bus
        try:
            vid_did = pcie_cfg_read(handle, PCI_VENDOR_ID, bus=next_bus, dev=0, size=4)
        except RuntimeError:
            vid_did = None

        vid = (vid_did & 0xFFFF) if vid_did else 0xFFFF
        did = ((vid_did >> 16) & 0xFFFF) if vid_did else 0xFFFF

        if vid in (0xFFFF, 0x0000):
            print(f"  Bus {bus} -> {next_bus}: no device found, stopping")
            break

        vendor = VENDOR_NAMES.get(vid, f"0x{vid:04X}")
        ht = pcie_cfg_read(handle, PCI_HEADER_TYPE, bus=next_bus, dev=0, size=1)
        header_type = (ht & 0x7F) if ht is not None else 0
        cr = pcie_cfg_read(handle, PCI_REVISION_ID, bus=next_bus, dev=0, size=4) or 0
        cc = (cr >> 24) & 0xFF

        print(f"  Bus {bus} -> {next_bus}: [{vid:04X}:{did:04X}] {vendor} "
              f"class=0x{cc:02X} htype={header_type}")

        if header_type != 1:
            # Not a bridge — this is the endpoint (GPU)
            print(f"  Found endpoint on bus {next_bus}")
            return next_bus

        next_bus += 1

    return next_bus


def enumerate_bus(handle, bus, verbose=False):
    """Scan a bus for devices, return list of (bus, dev, fn, vid, did, class_code, header_type)."""
    devices = []
    for dev in range(32):
        try:
            vid_did = pcie_cfg_read(handle, PCI_VENDOR_ID, bus=bus, dev=dev, size=4)
        except RuntimeError:
            continue
        vid = vid_did & 0xFFFF
        did = (vid_did >> 16) & 0xFFFF
        if vid in (0xFFFF, 0x0000):
            continue
        class_rev = pcie_cfg_read(handle, PCI_REVISION_ID, bus=bus, dev=dev, size=4)
        class_code = (class_rev >> 24) & 0xFF
        subclass = (class_rev >> 16) & 0xFF
        header_raw = pcie_cfg_read(handle, PCI_HEADER_TYPE, bus=bus, dev=dev, size=1)
        header_type = header_raw & 0x7F
        multi_fn = bool(header_raw & 0x80)

        devices.append((bus, dev, 0, vid, did, class_code, subclass, header_type))

        if multi_fn:
            for fn in range(1, 8):
                try:
                    vd = pcie_cfg_read(handle, PCI_VENDOR_ID, bus=bus, dev=dev, fn=fn, size=4)
                except RuntimeError:
                    continue
                v2 = vd & 0xFFFF
                if v2 in (0xFFFF, 0x0000):
                    continue
                cr = pcie_cfg_read(handle, PCI_REVISION_ID, bus=bus, dev=dev, fn=fn, size=4)
                devices.append((bus, dev, fn, v2, (vd >> 16) & 0xFFFF,
                                (cr >> 24) & 0xFF, (cr >> 16) & 0xFF,
                                pcie_cfg_read(handle, PCI_HEADER_TYPE, bus=bus, dev=dev, fn=fn, size=1) & 0x7F))
    return devices


def resize_bars(handle, bus, dev=0, fn=0, verbose=False):
    """Walk extended capability list and resize BAR0 to maximum (matches tinygrad)."""
    cap_ptr = 0x100
    while cap_ptr:
        try:
            hdr = pcie_cfg_read(handle, cap_ptr, bus=bus, dev=dev, fn=fn, size=4)
        except RuntimeError:
            break
        if hdr is None or hdr == 0xFFFFFFFF:
            break
        cap_id = hdr & 0xFFFF
        if cap_id == PCI_EXT_CAP_ID_REBAR:
            cap = pcie_cfg_read(handle, cap_ptr + 0x04, bus=bus, dev=dev, fn=fn, size=4)
            ctrl = pcie_cfg_read(handle, cap_ptr + 0x08, bus=bus, dev=dev, fn=fn, size=4)
            # Supported sizes are bits 4+ of cap register
            supported = cap >> 4
            if supported:
                max_bit = supported.bit_length() - 1
                new_ctrl = (ctrl & ~0x1F00) | (max_bit << 8)
                pcie_cfg_write(handle, cap_ptr + 0x08, new_ctrl, bus=bus, dev=dev, fn=fn, size=4)
                if verbose:
                    print(f"  Resizable BAR: cap=0x{cap:08X} ctrl 0x{ctrl:08X} -> 0x{new_ctrl:08X} (max_bit={max_bit})")
        cap_ptr = (hdr >> 20) & 0xFFC  # next capability pointer
    return


def assign_bars(handle, bus, dev=0, fn=0, mem_base=0x10000000, pref_mem_base=0x800000000, verbose=False):
    """Probe BAR sizes and assign addresses in a single pass.

    Matches tinygrad's pci_setup_usb_bars: for each BAR, write 0xFFFFFFFF to
    probe the size mask, compute size, then immediately assign an address.
    For 64-bit BARs, probe both low and high dwords.
    """
    result = {}
    mem_addr = [mem_base, pref_mem_base]  # [non-pref, pref]
    bar_off = 0

    while bar_off < 24:  # 6 BARs * 4 bytes
        reg = PCI_BASE_ADDRESS_0 + bar_off
        cfg = pcie_cfg_read(handle, reg, bus=bus, dev=dev, fn=fn, size=4)

        is_io = cfg & 0x01
        if is_io:
            bar_off += 4
            continue

        is_pref = bool(cfg & 0x08)
        is_64 = bool(cfg & 0x04)

        # Probe low dword
        pcie_cfg_write(handle, reg, 0xFFFFFFFF, bus=bus, dev=dev, fn=fn, size=4)
        lo = pcie_cfg_read(handle, reg, bus=bus, dev=dev, fn=fn, size=4) & 0xFFFFFFF0

        # Probe high dword for 64-bit BARs
        hi = 0
        if is_64:
            pcie_cfg_write(handle, reg + 4, 0xFFFFFFFF, bus=bus, dev=dev, fn=fn, size=4)
            hi = pcie_cfg_read(handle, reg + 4, bus=bus, dev=dev, fn=fn, size=4)

        combined = ((hi << 32) | lo) if is_64 else lo
        if combined == 0:
            bar_off += 8 if is_64 else 4
            continue

        mask = 0xFFFFFFFFFFFFFFFF if is_64 else 0xFFFFFFFF
        bar_size = ((~(combined & ~0xF)) + 1) & mask

        # Assign address from appropriate pool
        pool = int(is_pref)
        addr = mem_addr[pool]

        # Write assigned address
        pcie_cfg_write(handle, reg, addr & 0xFFFFFFFF, bus=bus, dev=dev, fn=fn, size=4)
        if is_64:
            pcie_cfg_write(handle, reg + 4, (addr >> 32) & 0xFFFFFFFF, bus=bus, dev=dev, fn=fn, size=4)

        bar_idx = bar_off // 4
        result[bar_idx] = (addr, bar_size)
        mem_addr[pool] += _round_up(bar_size, 2 << 20)

        if verbose:
            print(f"  BAR{bar_idx}: addr=0x{addr:X} size=0x{bar_size:X} "
                  f"{'64-bit' if is_64 else '32-bit'} {'pref' if is_pref else 'non-pref'}")

        bar_off += 8 if is_64 else 4

    # Enable memory + bus master
    pcie_cfg_write(handle, PCI_COMMAND, PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
                    bus=bus, dev=dev, fn=fn, size=1)

    return result


def _round_up(x, align):
    return (x + align - 1) & ~(align - 1)


# =============================================================================
# GPU register test
# =============================================================================

# AMD GPU MMIO scratch registers (GC 10.x/11.x) — safe to read/write
SCRATCH_REG0 = 0x2040  # mmSCRATCH_REG0 (offset within BAR5 MMIO)
SCRATCH_REG1 = 0x2044
SCRATCH_REG2 = 0x2048
SCRATCH_REG3 = 0x204C


def gpu_register_test(handle, bar5_addr, verbose=False):
    """Test GPU MMIO communication via BAR5.

    The GPU engine blocks (GC, RLC, etc.) are clock-gated at power-on,
    so scratch registers return 0xFFFFFFFF until the PSP boots.
    We verify that memory TLPs complete successfully (no UR/timeout)
    and that we get valid completion data from different MMIO regions.
    """
    print("\n=== GPU Register Test (BAR5 MMIO) ===")
    print(f"  BAR5 base: 0x{bar5_addr:X}")

    # Test reads across different MMIO regions
    test_offsets = [
        (0x0000, "BAR5[0x0000]"),
        (0x2040, "SCRATCH_REG0"),
        (0x5010, "GRBM_CNTL"),
        (0x8010, "GRBM_STATUS"),
        (0xD048, "RLC_CHIP_ID"),
    ]

    completions_ok = 0
    for off, name in test_offsets:
        try:
            val = pcie_mem_read(handle, bar5_addr + off, size=4, verbose=verbose)
            print(f"  {name:20s} [0x{off:04X}] = 0x{val:08X}")
            completions_ok += 1
        except (RuntimeError, TimeoutError) as e:
            print(f"  {name:20s} [0x{off:04X}] = ERROR: {e}")

    # Test write/read to SCRATCH_REG0 (will only work if GC block is unclocked)
    test_val = 0xDEAD_BEEF
    scratch_ok = False
    try:
        pcie_mem_write(handle, bar5_addr + SCRATCH_REG0, test_val, size=4, verbose=verbose)
        readback = pcie_mem_read(handle, bar5_addr + SCRATCH_REG0, size=4, verbose=verbose)
        if readback == test_val:
            print(f"\n  SCRATCH_REG0 write/read: PASS (0x{readback:08X})")
            scratch_ok = True
            pcie_mem_write(handle, bar5_addr + SCRATCH_REG0, 0, size=4)
        else:
            print(f"\n  SCRATCH_REG0 write/read: 0x{readback:08X} (GC block is clock-gated, expected before PSP boot)")
    except (RuntimeError, TimeoutError) as e:
        print(f"\n  SCRATCH_REG0 write/read: ERROR: {e}")

    # MMIO is working if all completions succeeded (even if values are 0/FF)
    mmio_ok = completions_ok == len(test_offsets)
    print(f"\n  MMIO completions: {completions_ok}/{len(test_offsets)}")
    print(f"  MMIO access: {'PASS' if mmio_ok else 'FAIL'}")
    if mmio_ok and not scratch_ok:
        print("  (GPU engine blocks are clock-gated — this is normal before PSP boot)")

    return mmio_ok


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="PCIe probe for ASM2464PD")
    parser.add_argument("--gpu-bus", type=int, default=4, help="GPU bus number (default: 4)")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--skip-bringup-check", action="store_true", help="Skip LTSSM L0 check")
    args = parser.parse_args()

    handle, ctx = usb_open()
    print(f"Opened device {VID:04X}:{PID:04X}")

    try:
        # Check PCIe link is up
        if not args.skip_bringup_check:
            ltssm = xdata_read(handle, 0xB450, 1)[0]
            if ltssm not in (0x48, 0x78):
                print(f"PCIe link not up (LTSSM=0x{ltssm:02X}). Run pcie_bringup.py first.")
                return 1

        # Step 1: Read bus 0 (ASM2464PD bridge)
        print("\n=== Bus 0 (ASM2464PD Bridge) ===")
        vid_did = pcie_cfg_read(handle, PCI_VENDOR_ID, bus=0, dev=0, size=4, verbose=args.verbose)
        vid = vid_did & 0xFFFF
        did = (vid_did >> 16) & 0xFFFF
        vendor = VENDOR_NAMES.get(vid, f"Unknown")
        print(f"  VID:DID = {vid:04X}:{did:04X} ({vendor})")

        # Step 2: Set up bridges, discover GPU bus
        gpu_bus = setup_bridges(handle, args.gpu_bus, verbose=args.verbose)

        # Step 3: Enumerate GPU bus
        print(f"\n=== Bus {gpu_bus} (GPU) ===")
        devices = enumerate_bus(handle, gpu_bus, verbose=args.verbose)
        if not devices:
            print("  No devices found!")
            return 1

        gpu_vid, gpu_did = None, None
        for bus, dev, fn, v, d, cc, sc, ht in devices:
            vendor = VENDOR_NAMES.get(v, f"0x{v:04X}")
            cname = CLASS_NAMES.get(cc, f"0x{cc:02X}")
            print(f"  {bus:02X}:{dev:02X}.{fn} [{v:04X}:{d:04X}] {vendor} — {cname} (subclass 0x{sc:02X})")
            if cc == 0x03:  # Display controller
                gpu_vid, gpu_did = v, d

        if gpu_vid is None:
            print("\n  No GPU (display controller) found on the bus.")
            # Still try the first device
            if devices:
                gpu_vid, gpu_did = devices[0][3], devices[0][4]

        # Step 4: Resize BARs, then probe and assign
        print(f"\n=== BAR Enumeration (bus {gpu_bus}) ===")
        resize_bars(handle, gpu_bus, verbose=args.verbose)
        bars = assign_bars(handle, gpu_bus, verbose=args.verbose)
        for idx, (addr, size) in sorted(bars.items()):
            if size >= 1024*1024*1024:
                human = f"{size / (1024*1024*1024):.1f} GB"
            elif size >= 1024*1024:
                human = f"{size / (1024*1024):.0f} MB"
            else:
                human = f"{size / 1024:.0f} KB"
            print(f"  BAR{idx}: addr=0x{addr:X}  size=0x{size:X} ({human})")

        if not bars:
            print("  No BARs found!")
            return 1

        # Step 5: GPU register test
        # BAR5 (index 5) is typically the MMIO register space for AMD GPUs
        # BAR0 (index 0) is VRAM
        bar5 = bars.get(5)
        bar0 = bars.get(0)

        if bar5:
            bar5_addr, bar5_size = bar5
            ok = gpu_register_test(handle, bar5_addr, verbose=args.verbose)
        elif bar0:
            # Try BAR0 as MMIO (some configs)
            bar0_addr, bar0_size = bar0
            print(f"\n  No BAR5 found, trying BAR0 at 0x{bar0_addr:X}")
            ok = gpu_register_test(handle, bar0_addr, verbose=args.verbose)
        else:
            print("\n  No suitable BAR for GPU register test")
            ok = False

        # Summary
        print("\n" + "=" * 50)
        print("SUMMARY")
        print(f"  GPU: {VENDOR_NAMES.get(gpu_vid, '?')} [{gpu_vid:04X}:{gpu_did:04X}]")
        print(f"  BARs: {len(bars)} found")
        if bar5:
            print(f"  MMIO (BAR5): 0x{bar5[0]:X} ({bar5[1] / 1024:.0f} KB)")
        if bar0:
            print(f"  VRAM (BAR0): 0x{bar0[0]:X} ({bar0[1] / (1024*1024*1024):.1f} GB)")
        print(f"  Register test: {'PASS' if ok else 'FAIL'}")

        return 0 if ok else 1

    finally:
        usb_close(handle, ctx)


if __name__ == "__main__":
    sys.exit(main())
