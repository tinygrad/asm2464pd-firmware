#!/usr/bin/env python3
"""
Probe the ASM2464PD's internal SRAM via E4/E5 + bulk OUT.

The chip has ~6 MB of internal SRAM at PCI address 0x200000.

XDATA regions (probing results with handmade firmware):
  - 0x7000-0x7FFF: USB bulk OUT landing buffer.  Read-only via E5, writable via bulk OUT.
  - 0x8000-0x8FFF: Writable XDATA. NOT aliased with 0xF000 (independent memory).
  - 0xA000-0xAFFF: Writable XDATA. Independent from 0xF000 and 0x8000.
  - 0xF000-0xFFFF: Writable XDATA. Independent from 0x8000.

The CE00 DMA engine (SCSI/Bulk DMA) uses 32-bit PCI addresses (CE76-CE79)
and is designed to move USB bulk data into SRAM.  However, it is tightly
coupled to the USB MSC state machine — CE00=0x03 is a no-op unless the
CE88/CE89 handshake has received data through the MSC protocol.  The
handmade firmware's raw bulk mode bypasses MSC, so CE00 DMA doesn't work
without adding SCSI WRITE(16) support.

Usage:
    python3 pcie/sram_probe.py
"""

import ctypes, sys, struct, time
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

# ─── USB helpers ─────────────────────────────────────────────

def usb_open():
    for vid, pid in [(0xADD1, 0x0001), (0x174C, 0x2463), (0x174C, 0x2464)]:
        try:
            dev = USB3(vid, pid, 0x81, 0x83, 0x02, 0x04, use_bot=True)
            print(f"opened {vid:04X}:{pid:04X}")
            return dev
        except RuntimeError:
            pass
    raise RuntimeError("no ASM2464PD found")

def peek(dev, addr, size=1):
    buf = (ctypes.c_ubyte * size)()
    ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr & 0xFFFF, 0, buf, size, 1000)
    if ret < 0: raise IOError(f"E4 read 0x{addr:04X} failed: {ret}")
    return bytes(buf[:ret])

def poke(dev, addr, val):
    ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr & 0xFFFF, val & 0xFF, None, 0, 1000)
    if ret < 0: raise IOError(f"E5 write 0x{addr:04X}=0x{val:02X} failed: {ret}")

def peek8(dev, addr): return peek(dev, addr, 1)[0]

def bulk_out(dev, data):
    """Send data via USB bulk OUT to EP 0x02. Lands at 0x7000."""
    dev._bulk_out(0x02, data)

def hexdump(data, addr, width=16):
    for off in range(0, len(data), width):
        chunk = data[off:off+width]
        hexpart = ' '.join(f'{b:02X}' for b in chunk)
        ascpart = ''.join(chr(b) if 0x20 <= b < 0x7F else '.' for b in chunk)
        print(f"  {addr+off:04X}: {hexpart:<{width*3}}  {ascpart}")

def write_xdata(dev, addr, data):
    """Write bytes to consecutive XDATA addresses via E5."""
    for i, b in enumerate(data):
        poke(dev, addr + i, b)

# ─── probe 1: register survey ───────────────────────────────

def probe_registers(dev):
    print("\n=== DMA & Buffer Registers ===")

    print("\n  SCSI DMA (CE00-CE9F):")
    for addr, name in [
        (0xCE00, "DMA ctrl"), (0xCE01, "DMA param"), (0xCE55, "xfer count"),
        (0xCE70, "xfer ctrl"), (0xCE72, "xfer mode"),
        (0xCE73, "buf ctrl0"), (0xCE74, "buf ctrl1"), (0xCE75, "buf len"),
        (0xCE80, "buf ctrl"), (0xCE83, "flow ctrl"),
        (0xCE88, "bulk hs"), (0xCE89, "DMA state"), (0xCE8A, "DMA aux"),
    ]:
        print(f"    {addr:04X} ({name:10s}): 0x{peek8(dev, addr):02X}")

    ce76 = peek(dev, 0xCE76, 4)
    print(f"    CE76-79 (PCI addr):    0x{struct.unpack('<I', ce76)[0]:08X}")
    print(f"    CE6E-6F (DMA status):  {peek(dev, 0xCE6E, 2).hex()}")

    print("\n  Buffer Descriptors (0x9310-0x9323):")
    for addr in range(0x9310, 0x9324, 2):
        val = struct.unpack('<H', peek(dev, addr, 2))[0]
        print(f"    {addr:04X}: 0x{val:04X}")

# ─── probe 2: window alias test ─────────────────────────────

def probe_alias(dev):
    print("\n=== Window Alias Test ===")

    orig_f = peek8(dev, 0xF000)
    orig_8 = peek8(dev, 0x8000)

    v1 = (orig_f ^ 0x55) & 0xFF
    poke(dev, 0xF000, v1)
    r8 = peek8(dev, 0x8000)
    rf = peek8(dev, 0xF000)
    print(f"  wrote 0xF000=0x{v1:02X}, read 0x8000=0x{r8:02X}, 0xF000=0x{rf:02X} -> {'ALIASED' if r8==v1 else 'INDEPENDENT'}")

    v2 = (v1 ^ 0xAA) & 0xFF
    poke(dev, 0x8000, v2)
    rf2 = peek8(dev, 0xF000)
    print(f"  wrote 0x8000=0x{v2:02X}, read 0xF000=0x{rf2:02X} -> {'ALIASED' if rf2==v2 else 'INDEPENDENT'}")

    poke(dev, 0xF000, orig_f)
    poke(dev, 0x8000, orig_8)

# ─── probe 3: bulk OUT to 0x7000 ────────────────────────────

def probe_bulk_out(dev):
    print("\n=== Bulk OUT -> 0x7000 ===")

    # Send known pattern via bulk OUT
    pattern = b'\xDE\xAD\xBE\xEF' + b'\xCA\xFE\xBA\xBE' + b'\x01\x02\x03\x04'
    # Pad to at least 31 bytes (CBW size) since the firmware expects CBW-shaped data
    # Actually the firmware just dumps whatever lands at 0x7000
    padded = pattern + b'\x00' * (32 - len(pattern))
    bulk_out(dev, padded)

    # Small delay for hardware
    time.sleep(0.01)

    # Read back from 0x7000
    result = peek(dev, 0x7000, 16)
    print(f"  sent:    {pattern[:16].hex()}")
    print(f"  0x7000:  {result.hex()}")
    match = result[:len(pattern)] == pattern
    print(f"  match: {match}")

    if not match:
        # The firmware's bulk handler might consume/transform the data
        # Try reading more context
        print(f"  0x7000 full 32B:")
        hexdump(peek(dev, 0x7000, 32), 0x7000)

    return match

# ─── probe 4: bulk OUT + CE00 DMA to SRAM ───────────────────

def probe_dma_write(dev):
    print("\n=== Bulk OUT -> 0x7000 -> CE00 DMA -> SRAM at 0x200000 ===")

    # Clear 0xF000 first
    orig_f = peek(dev, 0xF000, 8)
    write_xdata(dev, 0xF000, b'\x00' * 8)
    print(f"  0xF000 cleared: {peek(dev, 0xF000, 8).hex()}")

    # Send pattern via bulk OUT to 0x7000
    pattern = b'\xDE\xAD\xBE\xEF\xCA\xFE\xBA\xBE'
    padded = pattern + b'\x00' * (512 - len(pattern))  # pad to sector
    bulk_out(dev, padded)
    time.sleep(0.01)

    # Verify it landed at 0x7000
    at_7000 = peek(dev, 0x7000, 8)
    print(f"  0x7000 after bulk: {at_7000.hex()}")

    # Set CE76-79 to PCI 0x200000
    poke(dev, 0xCE76, 0x00)
    poke(dev, 0xCE77, 0x00)
    poke(dev, 0xCE78, 0x20)
    poke(dev, 0xCE79, 0x00)

    # Trigger CE00 DMA
    print("  triggering CE00 = 0x03...")
    poke(dev, 0xCE00, 0x03)
    for i in range(200):
        ce00 = peek8(dev, 0xCE00)
        if ce00 == 0x00:
            print(f"  CE00 done after {i+1} polls")
            break
    else:
        print(f"  CE00 stuck at 0x{ce00:02X}")

    # Check result at 0xF000
    result = peek(dev, 0xF000, 8)
    print(f"  0xF000 after DMA: {result.hex()}")
    print(f"  0x8000 after DMA: {peek(dev, 0x8000, 8).hex()}")

    if result[:8] == pattern:
        print("  SUCCESS: bulk -> 0x7000 -> DMA -> SRAM -> visible at 0xF000!")
    elif at_7000[:8] == pattern:
        print("  bulk landed OK but DMA didn't move it to SRAM")
    else:
        print("  bulk data didn't land at 0x7000 either")

    # Restore
    write_xdata(dev, 0xF000, orig_f)

# ─── probe 5: DMA to deep SRAM then verify via second DMA ───

def probe_dma_deep(dev):
    print("\n=== DMA to deep SRAM (PCI 0x201000) ===")

    # Write marker to 0xF000 via XDATA
    orig_f = peek(dev, 0xF000, 8)
    write_xdata(dev, 0xF000, b'\xAA\xBB\xCC\xDD\x00\x00\x00\x00')

    # Send different pattern via bulk OUT
    pattern = b'\x11\x22\x33\x44\x55\x66\x77\x88'
    padded = pattern + b'\x00' * (512 - len(pattern))
    bulk_out(dev, padded)
    time.sleep(0.01)

    at_7000 = peek(dev, 0x7000, 8)
    print(f"  0x7000 after bulk: {at_7000.hex()}")

    # DMA to PCI 0x201000 (past the 4KB window)
    poke(dev, 0xCE76, 0x00)
    poke(dev, 0xCE77, 0x10)
    poke(dev, 0xCE78, 0x20)
    poke(dev, 0xCE79, 0x00)

    poke(dev, 0xCE00, 0x03)
    for _ in range(200):
        if peek8(dev, 0xCE00) == 0: break

    # 0xF000 should still show our marker
    result = peek(dev, 0xF000, 8)
    print(f"  0xF000 (should be marker): {result.hex()}")

    if result[:4] == b'\xAA\xBB\xCC\xDD':
        print("  GOOD: 0xF000 unchanged, DMA went to different SRAM region")

        # Now DMA FROM 0x201000 back to see if data is there
        # We can verify by DMA-ing it back to 0x200000 (the window)
        # First clear 0xF000
        write_xdata(dev, 0xF000, b'\x00' * 8)

        # Set source to PCI 0x201000... but CE00 always reads from 0x7000
        # We can't read back from deep SRAM this way.
        # Instead: write the deep pattern to 0x200000 via DMA and check window
        poke(dev, 0xCE76, 0x00)
        poke(dev, 0xCE77, 0x00)
        poke(dev, 0xCE78, 0x20)
        poke(dev, 0xCE79, 0x00)

        # Send the same pattern again to 0x7000, DMA to 0x200000
        bulk_out(dev, padded)
        time.sleep(0.01)
        poke(dev, 0xCE00, 0x03)
        for _ in range(200):
            if peek8(dev, 0xCE00) == 0: break

        result2 = peek(dev, 0xF000, 8)
        print(f"  0xF000 after DMA to 0x200000: {result2.hex()}")
        if result2[:8] == pattern:
            print("  CONFIRMED: DMA to 0x200000 works, visible at 0xF000!")
    else:
        print(f"  0xF000 changed unexpectedly")

    write_xdata(dev, 0xF000, orig_f)

# ─── probe 6: DMA granularity ───────────────────────────────

def probe_dma_granularity(dev):
    print("\n=== DMA Granularity ===")
    print("  (How many bytes does one CE00=0x03 transfer?)")

    # Clear first 2KB of SRAM window
    orig_f = peek(dev, 0xF000, 32)
    for i in range(0, 2048, 4):
        write_xdata(dev, 0xF000 + i, b'\x00\x00\x00\x00')

    # Fill 0x7000 via bulk OUT with a pattern that's identifiable at each offset
    pattern = bytes([(i * 37 + 0xAB) & 0xFF for i in range(1024)])
    bulk_out(dev, pattern)
    time.sleep(0.01)

    # Verify bulk landing
    at_7000 = peek(dev, 0x7000, 16)
    print(f"  0x7000 first 16B: {at_7000.hex()}")

    # DMA one sector to PCI 0x200000
    poke(dev, 0xCE76, 0x00); poke(dev, 0xCE77, 0x00)
    poke(dev, 0xCE78, 0x20); poke(dev, 0xCE79, 0x00)
    poke(dev, 0xCE00, 0x03)
    for _ in range(200):
        if peek8(dev, 0xCE00) == 0: break

    # Check what arrived
    for off in [0x000, 0x010, 0x0F0, 0x100, 0x1F0, 0x200, 0x3F0]:
        data = peek(dev, 0xF000 + off, 16)
        expected = pattern[off:off+16] if off < len(pattern) else b'\x00' * 16
        match = data == expected
        tag = "MATCH" if match else ("ZERO" if data == b'\x00'*16 else "DIFF")
        print(f"  F000+{off:03X}: {data[:8].hex()} ... [{tag}]")

    write_xdata(dev, 0xF000, orig_f)

# ─── probe 7: multiple DMA sectors ──────────────────────────

def probe_dma_multi_sector(dev):
    print("\n=== Multiple CE00 DMA triggers ===")
    print("  (Does CE76-79 auto-increment per sector?)")

    orig_f = peek(dev, 0xF000, 32)
    # Clear first 2KB
    for i in range(0, 2048, 4):
        write_xdata(dev, 0xF000 + i, b'\x00\x00\x00\x00')

    # Send 1KB via bulk OUT
    pattern = bytes([(i * 13 + 0x42) & 0xFF for i in range(1024)])
    bulk_out(dev, pattern)
    time.sleep(0.01)

    # Set target to PCI 0x200000
    poke(dev, 0xCE76, 0x00); poke(dev, 0xCE77, 0x00)
    poke(dev, 0xCE78, 0x20); poke(dev, 0xCE79, 0x00)

    # Trigger TWO DMA transfers (should do 2x512B = 1KB if auto-increment)
    poke(dev, 0xCE00, 0x03)
    for _ in range(200):
        if peek8(dev, 0xCE00) == 0: break
    poke(dev, 0xCE00, 0x03)
    for _ in range(200):
        if peek8(dev, 0xCE00) == 0: break

    # Check CE76-79 after - did it auto-increment?
    ce76_after = peek(dev, 0xCE76, 4)
    pci_after = struct.unpack('<I', ce76_after)[0]
    print(f"  CE76-79 after 2 DMAs: 0x{pci_after:08X}")
    if pci_after == 0x200000:
        print("  NO auto-increment")
    elif pci_after == 0x200200:
        print("  auto-incremented by 512B per DMA!")
    elif pci_after == 0x200400:
        print("  auto-incremented by 1024B per DMA!")
    else:
        print(f"  unexpected value")

    # Check what's at 0xF000
    for off in [0x000, 0x100, 0x1F0, 0x200, 0x300, 0x3F0]:
        data = peek(dev, 0xF000 + off, 16)
        expected = pattern[off:off+16] if off < len(pattern) else b'\x00' * 16
        match = data == expected
        tag = "MATCH" if match else ("ZERO" if data == b'\x00'*16 else "DIFF")
        print(f"  F000+{off:03X}: {data[:8].hex()} [{tag}]")

    write_xdata(dev, 0xF000, orig_f)

# ─── probe 8: XDATA survey ──────────────────────────────────

def probe_xdata_survey(dev):
    print("\n=== XDATA Region Survey ===")
    regions = [
        (0x0000, "work RAM"), (0x0500, "globals"), (0x7000, "flash buf"),
        (0x8000, "SRAM win1"), (0x9000, "USB regs"), (0x9310, "buf desc"),
        (0xA000, "queue SRAM"), (0xB200, "PCIe TLP"), (0xB450, "LTSSM"),
        (0xC800, "int/DMA"), (0xCE00, "SCSI DMA"), (0xCE76, "DMA addr"),
        (0xD800, "EP buffer"), (0xF000, "SRAM win2"),
    ]
    for base, name in regions:
        try:
            data = peek(dev, base, 16)
            hexstr = ' '.join(f'{b:02X}' for b in data)
            print(f"  {base:04X} ({name:10s}): {hexstr}")
        except IOError as e:
            print(f"  {base:04X} ({name:10s}): ERROR {e}")

# ─── main ────────────────────────────────────────────────────

def main():
    dev = usb_open()
    try:
        probe_xdata_survey(dev)
        probe_registers(dev)
        probe_alias(dev)
        probe_bulk_out(dev)
        probe_dma_write(dev)
        probe_dma_deep(dev)
        probe_dma_granularity(dev)
        probe_dma_multi_sector(dev)
    finally:
        libusb.libusb_release_interface(dev.handle, 0)
        libusb.libusb_close(dev.handle)
    print("\ndone")

if __name__ == "__main__":
    main()
