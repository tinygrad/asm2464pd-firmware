#!/usr/bin/env python3
"""
DMA Probe for ASM2464PD — probe the DMA engines via E4/E5 control transfers.

Goal: DMA to/from internal SRAM (PCI 0x200000+) without firmware changes.
The handmade firmware runs with MSC_CFG=0x00 (raw bulk mode).
We poke all registers from the host side via E4 (read) / E5 (write).

FINDINGS:
  - CE00=0x03 generates completion descriptors at A000 but does NOT move data
    unless CE55 (transfer count) is nonzero. CE55 is read-only, set only by
    the USB MSC engine after a CE88/CE89 handshake with MSC_CFG enabled.
  - CE10-CE13: SRAM write pointer (32-bit PCI addr, big-endian). Writable.
    Auto-increments by 0x4000 per CE00=0x03 trigger.
  - 0x8000, 0xF000: Independent XDATA RAM, NOT live SRAM windows.
  - 0xA000: Completion queue (CE00 writes descriptors here).
  - 0x7000: USB bulk OUT landing buffer (hardware-filled, read-only to CPU).
  - SW DMA bulk IN (905A/905B/905C): always reads from D800 EP buffer,
    ignores 905B:905C source address.
  - MSC re-enable from host side causes bulk OUT timeout (-7) because the
    handmade firmware ISR conflicts with MSC state machine.
  - CE50-CE5F: all read-only (hardware DMA state).
  - CE01 must be 0x00 for CE00=0x03 to do anything.

  CONCLUSION: To DMA to SRAM, firmware must implement the CE88/CE89/CE00
  pipeline (needs MSC mode or equivalent bulk handshake that sets CE55).
  No way to bypass this from host-side register pokes alone.

Usage:
    python3 pcie/dma_probe.py              # run all probes
    python3 pcie/dma_probe.py -t baseline  # run one probe
    python3 pcie/dma_probe.py -v           # verbose register traces
"""

import ctypes, struct, sys, time, argparse

# =============================================================================
# USB transport — raw E4/E5 control transfers
# =============================================================================

VID, PID = 0xADD1, 0x0001

def usb_open():
    """Open the handmade firmware device, return (handle, ctx)."""
    from tinygrad.runtime.autogen import libusb as _libusb
    global libusb
    libusb = _libusb
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

def peek(handle, addr, size=1):
    """Read bytes from XDATA via E4 control transfer."""
    buf = (ctypes.c_ubyte * size)()
    ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, addr & 0xFFFF, 0, buf, size, 1000)
    if ret < 0:
        raise IOError(f"E4 read 0x{addr:04X} failed: {ret}")
    return bytes(buf[:ret])

def peek8(handle, addr):
    return peek(handle, addr, 1)[0]

def poke(handle, addr, val):
    """Write single byte to XDATA via E5 control transfer."""
    ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, addr & 0xFFFF, val & 0xFF, None, 0, 1000)
    if ret < 0:
        raise IOError(f"E5 write 0x{addr:04X}=0x{val:02X} failed: {ret}")

def poke32_le(handle, base, val):
    """Write 32-bit value little-endian to 4 consecutive addresses."""
    poke(handle, base + 0, (val >> 0) & 0xFF)
    poke(handle, base + 1, (val >> 8) & 0xFF)
    poke(handle, base + 2, (val >> 16) & 0xFF)
    poke(handle, base + 3, (val >> 24) & 0xFF)

def bulk_out(handle, ep, data):
    """Send raw bulk OUT data."""
    buf = (ctypes.c_ubyte * len(data))(*data)
    transferred = ctypes.c_int(0)
    ret = libusb.libusb_bulk_transfer(handle, ep, buf, len(data), ctypes.byref(transferred), 2000)
    if ret < 0:
        raise IOError(f"bulk OUT ep 0x{ep:02X} failed: {ret}")
    return transferred.value

def bulk_in(handle, ep, size, timeout_ms=2000):
    """Receive raw bulk IN data."""
    buf = (ctypes.c_ubyte * size)()
    transferred = ctypes.c_int(0)
    ret = libusb.libusb_bulk_transfer(handle, ep, buf, size, ctypes.byref(transferred), timeout_ms)
    if ret < 0:
        return None  # timeout or error — not fatal for probing
    return bytes(buf[:transferred.value])

# =============================================================================
# Register definitions
# =============================================================================

# SCSI/Bulk DMA engine
CE00 = 0xCE00   # SCSI DMA ctrl (write 0x03 to trigger)
CE01 = 0xCE01   # DMA parameter
CE55 = 0xCE55   # DMA transfer byte count (read-only, set by USB HW)
CE72 = 0xCE72   # SCSI transfer mode (direction?)
CE73 = 0xCE73   # Buffer ctrl 0
CE74 = 0xCE74   # Buffer ctrl 1
CE75 = 0xCE75   # Buffer length low
CE76 = 0xCE76   # PCI address byte 0 (LSB)
CE77 = 0xCE77   # PCI address byte 1
CE78 = 0xCE78   # PCI address byte 2
CE79 = 0xCE79   # PCI address byte 3 (MSB)
CE83 = 0xCE83   # Buffer flow control
CE88 = 0xCE88   # Bulk DMA handshake (write 0x00 to start)
CE89 = 0xCE89   # USB DMA state

# USB endpoint / SW DMA
MSC_CFG  = 0x900B  # MSC config
EP_CFG_905A = 0x905A  # SW DMA direction (0x10=IN, 0x08=OUT)
EP_BUF_HI  = 0x905B  # DMA source/dest addr high
EP_BUF_LO  = 0x905C  # DMA source/dest addr low
EP_CFG1    = 0x9093   # EP arm/ack
EP_CFG2    = 0x9094   # EP arm secondary
MSC_LENGTH = 0x901A   # Transfer length for MSC/DMA
EP_READY   = 0x9096   # EP ready ack
BULK_DMA_TRIGGER = 0x90A1  # Bulk DMA trigger
SW_DMA_TRIGGER   = 0x90E1  # SW DMA trigger
USB_MODE   = 0x90E2   # MSC engine gate
EP_STATUS_90E3 = 0x90E3
STATUS_909E = 0x909E
CTRL_90A0  = 0x90A0
PERIPH_STATUS = 0x9101  # USB peripheral status

# DMA config
DMA_CONFIG = 0xC8D4  # DMA config (0xA0=SW mode)
DMA_STATUS = 0xC8D6  # DMA status
INT_AUX    = 0xC805  # DMA mode config
XFER_CTRL_C509 = 0xC509

# NVMe/MSC
MSC_CTRL   = 0xC42C  # MSC engine trigger
MSC_STATUS = 0xC42D  # MSC status

# SRAM windows
FLASH_BUF  = 0x7000
SRAM_WIN_8 = 0x8000
SRAM_WIN_A = 0xA000
SRAM_WIN_F = 0xF000

# Bulk endpoints (handmade firmware: EP1 IN, EP2 OUT, EP3 IN, EP4 OUT)
EP_OUT = 0x02
EP_IN  = 0x81

VERBOSE = False

def vprint(*args, **kwargs):
    if VERBOSE: print(*args, **kwargs)

# =============================================================================
# Probe helpers
# =============================================================================

def dump_regs(handle, label=""):
    """Dump all DMA-relevant registers."""
    if label: print(f"  --- {label} ---")
    regs = [
        (CE00, "CE00 DMA_CTRL"), (CE01, "CE01 DMA_PARAM"), (CE55, "CE55 XFER_CNT"),
        (CE72, "CE72 XFER_MODE"), (CE76, "CE76 ADDR0"), (CE77, "CE77 ADDR1"),
        (CE78, "CE78 ADDR2"), (CE79, "CE79 ADDR3"), (CE83, "CE83 BUF_FLOW"),
        (CE88, "CE88 HANDSHAKE"), (CE89, "CE89 DMA_STATE"),
        (MSC_CFG, "900B MSC_CFG"), (EP_CFG_905A, "905A EP_DIR"),
        (EP_BUF_HI, "905B BUF_HI"), (EP_BUF_LO, "905C BUF_LO"),
        (EP_CFG1, "9093 EP_CFG1"), (EP_CFG2, "9094 EP_CFG2"),
        (MSC_LENGTH, "901A MSC_LEN"), (BULK_DMA_TRIGGER, "90A1 BULK_TRIG"),
        (SW_DMA_TRIGGER, "90E1 SW_TRIG"), (USB_MODE, "90E2 USB_MODE"),
        (DMA_CONFIG, "C8D4 DMA_CFG"), (DMA_STATUS, "C8D6 DMA_STAT"),
        (INT_AUX, "C805 INT_AUX"), (PERIPH_STATUS, "9101 PERIPH"),
    ]
    for addr, name in regs:
        val = peek8(handle, addr)
        print(f"    {name:20s} = 0x{val:02X}")

def dump_mem(handle, addr, size=16, label=None):
    """Dump memory region."""
    data = peek(handle, addr, size)
    label = label or f"0x{addr:04X}"
    print(f"    {label}: {data.hex()}")
    return data

def make_pattern(size, seed=0xDE):
    """Generate a recognizable byte pattern."""
    return bytes([(seed + i) & 0xFF for i in range(size)])

# =============================================================================
# Probe 1: Baseline
# =============================================================================

def probe_baseline(handle):
    print("\n=== Probe 1: Baseline Register State ===")
    dump_regs(handle)
    print("  Memory windows:")
    dump_mem(handle, FLASH_BUF, 16, "0x7000 (flash buf)")
    dump_mem(handle, SRAM_WIN_8, 16, "0x8000 (SRAM win)")
    dump_mem(handle, SRAM_WIN_A, 16, "0xA000 (compl Q)")
    dump_mem(handle, SRAM_WIN_F, 16, "0xF000 (NVMe win)")
    print("  [baseline done]")

# =============================================================================
# Probe 2: Raw bulk OUT → 0x7000
# =============================================================================

def probe_raw_bulk_out(handle):
    print("\n=== Probe 2: Raw Bulk OUT → 0x7000 ===")
    pattern = make_pattern(32)
    print(f"  Sending {len(pattern)} bytes to EP2...")
    try:
        n = bulk_out(handle, EP_OUT, pattern)
        print(f"  Sent {n} bytes")
    except IOError as e:
        print(f"  Bulk OUT failed: {e}")
        return False
    time.sleep(0.05)
    result = peek(handle, FLASH_BUF, 32)
    print(f"  Sent:   {pattern[:16].hex()}")
    print(f"  0x7000: {result[:16].hex()}")
    match = result == pattern
    print(f"  Match: {match}")
    return match

# =============================================================================
# Probe 3: CE00=0x03 raw (no MSC, expected no-op)
# =============================================================================

def probe_ce00_raw(handle):
    print("\n=== Probe 3: CE00=0x03 Without MSC ===")

    # Write known data to 0x7000 via bulk OUT
    pattern = make_pattern(512, seed=0xAA)
    try:
        bulk_out(handle, EP_OUT, pattern)
    except IOError:
        print("  Bulk OUT failed, skipping")
        return False
    time.sleep(0.05)

    # Clear A000 marker
    for i in range(16):
        poke(handle, SRAM_WIN_A + i, 0xCC)

    # Write F000 marker
    poke(handle, SRAM_WIN_F, 0x11)
    poke(handle, SRAM_WIN_F + 1, 0x22)

    # Set CE76-79 to PCI 0x00200000
    poke(handle, CE76, 0x00)
    poke(handle, CE77, 0x00)
    poke(handle, CE78, 0x20)
    poke(handle, CE79, 0x00)

    # Set direction = write
    poke(handle, CE72, 0x00)
    # Clear flow control upper nibble
    ce83 = peek8(handle, CE83)
    poke(handle, CE83, ce83 & 0x8F)

    print(f"  CE55 before trigger: 0x{peek8(handle, CE55):02X}")
    print(f"  CE00 before trigger: 0x{peek8(handle, CE00):02X}")

    # Trigger
    poke(handle, CE00, 0x03)
    time.sleep(0.05)

    ce00_after = peek8(handle, CE00)
    print(f"  CE00 after trigger: 0x{ce00_after:02X}")

    a000 = peek(handle, SRAM_WIN_A, 16)
    f000_0 = peek8(handle, SRAM_WIN_F)
    f000_1 = peek8(handle, SRAM_WIN_F + 1)

    print(f"  A000 (completion): {a000.hex()}")
    a000_changed = a000 != b'\xCC' * 16
    print(f"  A000 changed: {a000_changed}")
    print(f"  F000[0:1] = 0x{f000_0:02X} 0x{f000_1:02X} (was 0x11 0x22)")
    print(f"  CE00 raw: {'WORKED' if a000_changed else 'NO-OP (expected)'}")
    return a000_changed

# =============================================================================
# Probe 4: MSC re-enable + CE88/CE89 handshake + CE00 DMA
# =============================================================================

def probe_msc_dma(handle):
    print("\n=== Probe 4: MSC Re-enable + CE88/CE89 + CE00 DMA ===")

    # Clear A000 marker
    for i in range(16):
        poke(handle, SRAM_WIN_A + i, 0xCC)

    # Mark F000
    poke(handle, SRAM_WIN_F, 0x11)
    poke(handle, SRAM_WIN_8, 0x22)

    # Set PCI target address = 0x00200000
    poke(handle, CE76, 0x00)
    poke(handle, CE77, 0x00)
    poke(handle, CE78, 0x20)
    poke(handle, CE79, 0x00)
    poke(handle, CE72, 0x00)  # write direction
    ce83 = peek8(handle, CE83)
    poke(handle, CE83, ce83 & 0x8F)

    # --- Re-enable MSC engine ---
    # The handmade firmware set MSC_CFG=0x00. We try to re-enable it.
    # Step 1: Set USB_MODE gate
    poke(handle, USB_MODE, 0x01)
    # Step 2: Set MSC length for data (512 bytes = 0x200)
    # Actually MSC_LENGTH is 8-bit, used for CSW. For bulk data, this is different.
    # Step 3: Configure DMA mode
    poke(handle, INT_AUX, (peek8(handle, INT_AUX) & 0xF9) | 0x02)

    # Step 4: Arm bulk OUT endpoint
    poke(handle, EP_CFG1, 0x02)  # ARM_OUT
    poke(handle, EP_CFG2, 0x10)  # ARM_OUT

    print("  Armed bulk OUT, sending 512 bytes...")
    pattern = make_pattern(512, seed=0x50)
    try:
        bulk_out(handle, EP_OUT, pattern)
    except IOError as e:
        print(f"  Bulk OUT failed: {e}")
        return False
    time.sleep(0.1)

    # Check if PERIPH_STATUS shows bulk data
    periph = peek8(handle, PERIPH_STATUS)
    print(f"  PERIPH_STATUS = 0x{periph:02X} (bit 3 = BULK_DATA)")

    # Re-arm for CE88 handshake
    poke(handle, EP_CFG1, 0x02)  # ARM_OUT
    poke(handle, INT_AUX, (peek8(handle, INT_AUX) & 0xF9) | 0x02)

    # CE88/CE89 handshake
    poke(handle, CE88, 0x00)
    time.sleep(0.05)

    ce89 = peek8(handle, CE89)
    ce55 = peek8(handle, CE55)
    print(f"  After CE88=0x00: CE89=0x{ce89:02X} CE55=0x{ce55:02X}")

    if not (ce89 & 0x01):
        print("  CE89 not ready — handshake failed")
        # Try polling
        for i in range(100):
            ce89 = peek8(handle, CE89)
            if ce89 & 0x01:
                print(f"  CE89 ready after {i+1} polls")
                break
            time.sleep(0.01)
        else:
            print("  CE89 never became ready")

    ce55 = peek8(handle, CE55)
    print(f"  CE55 (xfer count) = 0x{ce55:02X}")

    # Trigger CE00=0x03
    print("  Triggering CE00=0x03...")
    poke(handle, CE00, 0x03)
    for i in range(100):
        ce00 = peek8(handle, CE00)
        if ce00 == 0x00:
            print(f"  CE00 completed after {i+1} polls")
            break
        time.sleep(0.01)
    else:
        print(f"  CE00 still 0x{ce00:02X} after 100 polls")

    # Check results
    a000 = peek(handle, SRAM_WIN_A, 16)
    f000 = peek8(handle, SRAM_WIN_F)
    x8000 = peek8(handle, SRAM_WIN_8)
    print(f"  A000 (completion): {a000.hex()}")
    a000_changed = a000 != b'\xCC' * 16
    print(f"  A000 changed: {a000_changed}")
    print(f"  F000[0] = 0x{f000:02X} (was 0x11)")
    print(f"  8000[0] = 0x{x8000:02X} (was 0x22)")

    # Verify 0x7000 got our pattern
    x7000 = peek(handle, FLASH_BUF, 8)
    print(f"  7000: {x7000.hex()} (expected {pattern[:8].hex()})")

    worked = a000_changed
    print(f"  MSC DMA: {'WORKED' if worked else 'FAILED'}")

    # Restore MSC_CFG=0x00 (raw bulk mode, as handmade firmware expects)
    poke(handle, USB_MODE, 0x00)
    return worked

# =============================================================================
# Probe 5: Full pcie4_main.c style CE00 DMA (with all register setup)
# =============================================================================

def probe_full_dma_sequence(handle):
    print("\n=== Probe 5: Full DMA Sequence (pcie4_main.c style) ===")

    # Clear markers
    for i in range(16):
        poke(handle, SRAM_WIN_A + i, 0xCC)
    poke(handle, SRAM_WIN_F, 0x33)
    poke(handle, SRAM_WIN_8, 0x44)

    # Replicate scsi_write16_dma_preset from pcie4_main.c
    # These NVMe/SCSI registers may be needed to arm the DMA engine
    # CE40-CE45 are DMA params, clear them
    for i in range(6):
        poke(handle, 0xCE40 + i, 0x00)

    # CE6E/CE6F clear DMA status
    poke(handle, 0xCE6E, 0x00)
    poke(handle, 0xCE6F, 0x00)

    # Set PCI target = 0x00200000
    poke(handle, CE76, 0x00)
    poke(handle, CE77, 0x00)
    poke(handle, CE78, 0x20)
    poke(handle, CE79, 0x00)

    poke(handle, CE72, 0x00)  # write direction
    ce83 = peek8(handle, CE83)
    poke(handle, CE83, ce83 & 0x8F)  # clear upper nibble

    # Save and configure EP state
    ep_905a_saved = peek8(handle, EP_CFG_905A)
    poke(handle, EP_CFG_905A, 0x00)
    poke(handle, EP_CFG1, 0x00)
    poke(handle, EP_CFG2, 0x00)

    # Set USB_MODE and DMA mode
    poke(handle, USB_MODE, 0x01)
    poke(handle, INT_AUX, (peek8(handle, INT_AUX) & 0xF9) | 0x02)

    # Arm bulk OUT
    poke(handle, EP_CFG1, 0x02)  # ARM_OUT
    poke(handle, EP_CFG2, 0x10)  # ARM_OUT

    # Send 512 bytes
    pattern = make_pattern(512, seed=0xBE)
    print(f"  Sending {len(pattern)} bytes (first 8: {pattern[:8].hex()})...")
    try:
        bulk_out(handle, EP_OUT, pattern)
    except IOError as e:
        print(f"  Bulk OUT failed: {e}")
        poke(handle, EP_CFG_905A, ep_905a_saved)
        return False
    time.sleep(0.1)

    # Wait for BULK_DATA on PERIPH_STATUS
    periph = peek8(handle, PERIPH_STATUS)
    vprint(f"  PERIPH_STATUS = 0x{periph:02X}")

    # Re-arm and do CE88/CE89 handshake
    poke(handle, EP_CFG1, 0x02)  # ARM_OUT
    poke(handle, INT_AUX, (peek8(handle, INT_AUX) & 0xF9) | 0x02)
    poke(handle, CE88, 0x00)

    # Poll CE89
    ce89_ready = False
    for i in range(200):
        ce89 = peek8(handle, CE89)
        if ce89 & 0x01:
            ce89_ready = True
            vprint(f"  CE89 ready after {i+1} polls (0x{ce89:02X})")
            break
        time.sleep(0.005)
    if not ce89_ready:
        print(f"  CE89 not ready (0x{ce89:02X})")

    # Check transfer count
    ce55 = peek8(handle, CE55)
    print(f"  CE55 (xfer count) = 0x{ce55:02X}")

    # Settle (like the stock firmware's 0x80 loop)
    for _ in range(16):
        peek8(handle, CE89)

    # Trigger CE00=0x03
    poke(handle, CE00, 0x03)
    ce00_done = False
    for i in range(200):
        ce00 = peek8(handle, CE00)
        if ce00 == 0x00:
            ce00_done = True
            vprint(f"  CE00 done after {i+1} polls")
            break
        time.sleep(0.005)
    if not ce00_done:
        print(f"  CE00 stuck at 0x{ce00:02X}")

    # Check results
    a000 = peek(handle, SRAM_WIN_A, 16)
    f000 = peek8(handle, SRAM_WIN_F)
    x8000 = peek8(handle, SRAM_WIN_8)
    x7000 = peek(handle, FLASH_BUF, 8)

    print(f"  A000 (completion): {a000.hex()}")
    a000_changed = a000 != b'\xCC' * 16
    print(f"  A000 changed: {a000_changed}")
    print(f"  F000[0] = 0x{f000:02X} (was 0x33)")
    print(f"  8000[0] = 0x{x8000:02X} (was 0x44)")
    print(f"  7000: {x7000.hex()} (expected {pattern[:8].hex()})")

    # Cleanup: final CE88 handshake
    poke(handle, CE88, 0x00)
    for i in range(100):
        if peek8(handle, CE89) & 0x01: break
        time.sleep(0.005)

    # Restore
    poke(handle, STATUS_909E, 0x01)
    poke(handle, EP_STATUS_90E3, 0x02)
    poke(handle, EP_READY, 0x01)
    poke(handle, EP_CFG1, 0x00)
    poke(handle, EP_CFG2, 0x00)
    poke(handle, EP_CFG_905A, ep_905a_saved)
    poke(handle, USB_MODE, 0x00)

    # Re-arm OUT for handmade firmware
    poke(handle, EP_CFG2, 0x10)

    worked = a000_changed
    print(f"  Full DMA: {'WORKED' if worked else 'FAILED'}")
    return worked

# =============================================================================
# Probe 6: CE72 direction probe (try to read SRAM → XDATA)
# =============================================================================

def probe_ce72_direction(handle):
    print("\n=== Probe 6: CE72 Direction Probe ===")

    # First, write a known pattern to F000 via E5 (just local XDATA, not SRAM)
    for i in range(8):
        poke(handle, SRAM_WIN_F + i, 0x00)

    # Set PCI source = 0x00200000 (where SRAM data might live)
    poke(handle, CE76, 0x00)
    poke(handle, CE77, 0x00)
    poke(handle, CE78, 0x20)
    poke(handle, CE79, 0x00)

    print("  Trying CE72 values to reverse DMA direction (SRAM → 0x7000)...")

    for ce72_val in [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]:
        # Clear 0x7000 marker
        # (0x7000 is read-only to CPU, so we can't clear it via E5.
        #  We'll just read it and compare before/after.)
        x7000_before = peek(handle, FLASH_BUF, 8)

        poke(handle, CE72, ce72_val)
        ce83 = peek8(handle, CE83)
        poke(handle, CE83, ce83 & 0x8F)

        poke(handle, CE00, 0x03)
        time.sleep(0.05)

        x7000_after = peek(handle, FLASH_BUF, 8)
        f000_after = peek(handle, SRAM_WIN_F, 8)
        x8000_after = peek(handle, SRAM_WIN_8, 8)

        changed_7000 = x7000_before != x7000_after
        changed_f000 = f000_after != b'\x00' * 8

        status = ""
        if changed_7000: status += " 7000-CHANGED"
        if changed_f000: status += " F000-CHANGED"
        if not status: status = " no-change"

        print(f"    CE72=0x{ce72_val:02X}:{status}")
        if changed_7000:
            print(f"      7000 before: {x7000_before.hex()}")
            print(f"      7000 after:  {x7000_after.hex()}")
        if changed_f000:
            print(f"      F000: {f000_after.hex()}")

    # Restore
    poke(handle, CE72, 0x00)
    print("  [CE72 probe done]")

# =============================================================================
# Probe 7: SW DMA bulk IN (905A/905B/905C → bulk IN to host)
# =============================================================================

def probe_sw_dma_bulk_in(handle):
    print("\n=== Probe 7: SW DMA Bulk IN ===")

    # Write a known pattern to F000 via E5
    pattern = make_pattern(64, seed=0x42)
    for i in range(64):
        poke(handle, SRAM_WIN_F + i, pattern[i])
    print(f"  Wrote pattern to F000: {pattern[:8].hex()}...")

    # Also write pattern at 0x7000 contents (already have bulk data there)
    x7000 = peek(handle, FLASH_BUF, 8)
    print(f"  0x7000 current: {x7000.hex()}")

    # Try SW DMA from various source addresses
    for src_addr, name in [(0xF000, "F000"), (0x8000, "8000"), (0x7000, "7000")]:
        print(f"\n  --- SW DMA from {name} ---")
        src_hi = (src_addr >> 8) & 0xFF
        src_lo = src_addr & 0xFF
        xfer_len = 32

        # Clear stale EP_COMPLETE
        periph = peek8(handle, PERIPH_STATUS)
        if periph & 0x20:  # EP_COMPLETE
            poke(handle, EP_STATUS_90E3, 0x02)
            poke(handle, EP_READY, 0x01)

        # Set MSC length
        poke(handle, MSC_LENGTH, xfer_len)

        # Enable SW DMA mode
        poke(handle, DMA_CONFIG, 0xA0)

        # Set source address
        poke(handle, EP_BUF_HI, src_hi)
        poke(handle, EP_BUF_LO, src_lo)

        # EP buffer descriptor at D800
        poke(handle, 0xD802, src_hi)
        poke(handle, 0xD803, src_lo)
        poke(handle, 0xD804, 0x00)
        poke(handle, 0xD805, 0x00)
        poke(handle, 0xD806, 0x00)
        poke(handle, 0xD807, 0x00)
        poke(handle, 0xD80F, 0x00)
        poke(handle, 0xD800, 0x03)  # DMA control: bulk IN

        # Pre-trigger
        c509 = peek8(handle, XFER_CTRL_C509)
        poke(handle, XFER_CTRL_C509, c509 | 0x01)

        # Set direction = IN
        poke(handle, EP_CFG_905A, 0x10)

        # Trigger SW DMA
        poke(handle, SW_DMA_TRIGGER, 0x01)

        # Post-trigger
        poke(handle, XFER_CTRL_C509, c509 & ~0x01)

        # Set USB_MODE gate
        poke(handle, USB_MODE, 0x01)

        # Arm bulk IN
        poke(handle, EP_CFG1, 0x08)  # ARM_IN
        poke(handle, EP_CFG2, 0x02)  # ARM_IN

        # Trigger bulk DMA
        poke(handle, BULK_DMA_TRIGGER, 0x01)

        # Try to receive via bulk IN
        time.sleep(0.05)
        result = bulk_in(handle, EP_IN, xfer_len, timeout_ms=500)

        if result is not None:
            print(f"    Received {len(result)} bytes: {result[:16].hex()}")
            if src_addr == 0xF000:
                match = result == pattern[:xfer_len]
                print(f"    Pattern match: {match}")
        else:
            print(f"    Bulk IN: timeout/error")

        # Check EP_COMPLETE
        periph = peek8(handle, PERIPH_STATUS)
        vprint(f"    PERIPH after: 0x{periph:02X}")

        # Cleanup
        poke(handle, EP_STATUS_90E3, 0x02)
        poke(handle, EP_READY, 0x01)
        poke(handle, STATUS_909E, 0x01)
        poke(handle, CTRL_90A0, 0x01)
        poke(handle, BULK_DMA_TRIGGER, 0x00)
        poke(handle, DMA_CONFIG, 0x00)
        poke(handle, MSC_LENGTH, 0x0D)
        poke(handle, USB_MODE, 0x00)

    # Re-arm OUT for handmade firmware
    poke(handle, EP_CFG2, 0x10)
    print("  [SW DMA bulk IN done]")

# =============================================================================
# Probe 8: SRAM Roundtrip
# =============================================================================

def probe_sram_roundtrip(handle):
    print("\n=== Probe 8: SRAM Roundtrip ===")
    print("  Step 1: Write pattern to SRAM via CE00 DMA")

    # Attempt the full DMA write (reuse probe 5 logic)
    worked = probe_full_dma_sequence(handle)
    if not worked:
        print("  DMA write failed — cannot test roundtrip")
        return False

    print("\n  Step 2: Read back from SRAM")
    print("  Checking if F000/8000 reflect SRAM content...")

    f000 = peek(handle, SRAM_WIN_F, 16)
    x8000 = peek(handle, SRAM_WIN_8, 16)
    print(f"  F000: {f000.hex()}")
    print(f"  8000: {x8000.hex()}")

    # Check if either window shows our pattern
    expected = make_pattern(16, seed=0xBE)
    if f000 == expected:
        print("  F000 matches written pattern — SRAM visible at F000!")
        return True
    if x8000 == expected:
        print("  8000 matches written pattern — SRAM visible at 8000!")
        return True

    print("  Neither window shows the pattern.")
    print("  Trying CE72 reverse DMA...")
    probe_ce72_direction(handle)

    return False

# =============================================================================
# Main
# =============================================================================

PROBES = {
    "baseline": probe_baseline,
    "bulk_out": probe_raw_bulk_out,
    "ce00_raw": probe_ce00_raw,
    "msc_dma": probe_msc_dma,
    "full_dma": probe_full_dma_sequence,
    "ce72_dir": probe_ce72_direction,
    "sw_dma_in": probe_sw_dma_bulk_in,
    "roundtrip": probe_sram_roundtrip,
}

def main():
    parser = argparse.ArgumentParser(description="DMA probe for ASM2464PD")
    parser.add_argument("-t", "--test", choices=list(PROBES.keys()), help="Run specific probe")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()

    global VERBOSE
    VERBOSE = args.verbose

    handle, ctx = usb_open()
    print(f"Opened device {VID:04X}:{PID:04X}")

    try:
        if args.test:
            PROBES[args.test](handle)
        else:
            probe_baseline(handle)
            probe_raw_bulk_out(handle)
            probe_ce00_raw(handle)
            probe_msc_dma(handle)
            probe_full_dma_sequence(handle)
            probe_ce72_direction(handle)
            probe_sw_dma_bulk_in(handle)
            # roundtrip depends on full_dma working, run last
            # probe_sram_roundtrip(handle)

            print("\n" + "=" * 50)
            print("All probes complete. Check output above for results.")
    finally:
        usb_close(handle, ctx)

if __name__ == "__main__":
    main()
