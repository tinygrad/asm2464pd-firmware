#!/usr/bin/env python3
"""
Comprehensive tests for the clean firmware covering everything tinygrad needs.

Tests are ordered to match the tinygrad initialization sequence:
  1. USB BOT basics (TUR, E8, INQUIRY, READ_CAPACITY)
  2. E5 writes / E4 reads (XDATA access)
  3. Controller init E5 writes (ASM24Controller.__init__)
  4. PCIe config space (bridge VID/DID, bus enumeration)
  5. PCIe BAR setup (pci_setup_usb_bars)
  6. PCIe memory reads (GPU MMIO)
  7. SCSI WRITE(16) — 512B, 4KB, 64KB
  8. SCSI WRITE + doorbell sequence (E5 0x171-0x173 + CE6E/CE6F)
  9. SCSI WRITE data at PCI 0x200000 (PCIe readback)
  10. B296 clean after SCSI WRITE (pcie_mem_req still works)
  11. Full PSP-like flow (scsi_write + pcie_mem_req interleaved)

Run:
  sudo PYTHONPATH=/home/geohot/tinygrad python3 test_clean_firmware.py
  sudo PYTHONPATH=/home/geohot/tinygrad python3 test_clean_firmware.py 1-5     # range
  sudo PYTHONPATH=/home/geohot/tinygrad python3 test_clean_firmware.py 9 10    # specific
  sudo PYTHONPATH=/home/geohot/tinygrad python3 test_clean_firmware.py --no-stop  # don't stop on fail
"""

import ctypes, struct, sys, time, traceback
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import USB3, ASM24Controller, WriteOp, ReadOp, ScsiWriteOp
from tinygrad.runtime.autogen import libusb
from tinygrad.helpers import round_up

# ============================================================
# Low-level helpers
# ============================================================

def bot_send(dev, cdb, rlen=0, send_data=None, timeout=2000):
    """Send a single BOT CBW, optional data phase, receive CSW."""
    dev._tag += 1
    dir_in = rlen > 0
    data_len = rlen if dir_in else (len(send_data) if send_data else 0)
    flags = 0x80 if dir_in else 0x00
    cbw = struct.pack('<IIIBBB', 0x43425355, dev._tag, data_len, flags, 0, len(cdb)) + cdb + b'\x00' * (16 - len(cdb))
    dev._bulk_out(dev.ep_data_out, cbw)
    data_in = None
    if dir_in:
        data_in = dev._bulk_in(dev.ep_data_in, rlen, timeout=timeout)
    elif send_data is not None:
        dev._bulk_out(dev.ep_data_out, send_data, timeout=timeout)
    csw = dev._bulk_in(dev.ep_data_in, 13, timeout=timeout)
    sig, rtag, residue, status = struct.unpack('<IIIB', csw)
    assert sig == 0x53425355, f"Bad CSW sig 0x{sig:08X}"
    assert rtag == dev._tag, f"CSW tag mismatch: got {rtag}, expected {dev._tag}"
    return residue, status, data_in

def e5_write(dev, addr, val):
    """E5 write byte to XDATA[addr]."""
    full_addr = (addr & 0x1FFFF) | 0x500000
    cdb = struct.pack('>BBBHB', 0xE5, val, full_addr >> 16, full_addr & 0xFFFF, 0)
    residue, status, _ = bot_send(dev, cdb)
    assert status == 0, f"E5 write 0x{addr:04X}=0x{val:02X} failed: status={status}"

def e4_read(dev, addr, size=1):
    """E4 read via CSW residue (no data phase). Returns bytes."""
    full_addr = (addr & 0x1FFFF) | 0x500000
    cdb = struct.pack('>BBBHB', 0xE4, size, full_addr >> 16, full_addr & 0xFFFF, 0)
    residue, status, _ = bot_send(dev, cdb)
    assert status == 0, f"E4 read 0x{addr:04X} failed: status={status}"
    return bytes([(residue >> (i * 8)) & 0xFF for i in range(size)])

def e4_read_datain(dev, addr, size=1):
    """E4 read via data-IN phase. Returns bytes."""
    full_addr = (addr & 0x1FFFF) | 0x500000
    cdb = struct.pack('>BBBHB', 0xE4, size, full_addr >> 16, full_addr & 0xFFFF, 0)
    residue, status, data = bot_send(dev, cdb, rlen=size)
    assert status == 0, f"E4 data-IN 0x{addr:04X} failed: status={status}"
    return data

def scsi_write_raw(dev, data, lba=0, timeout=10000):
    """Send SCSI WRITE(16) with data OUT phase."""
    sectors = round_up(len(data), 512) // 512
    padded = data + b'\x00' * (sectors * 512 - len(data))
    cdb = struct.pack('>BBQIBB', 0x8A, 0, lba, sectors, 0, 0)
    residue, status, _ = bot_send(dev, cdb, send_data=padded, timeout=timeout)
    return status

def make_asm24(dev):
    """Create an ASM24Controller wrapper around raw USB3 device."""
    usb = ASM24Controller.__new__(ASM24Controller)
    usb.usb = dev
    usb._cache = {}
    usb._pci_cacheable = []
    usb._pci_cache = {}
    return usb

# ============================================================
# Test 1: USB BOT basics
# ============================================================

def test_01_tur(dev):
    """TEST_UNIT_READY (0x00) — basic CBW/CSW handshake."""
    cdb = b'\x00' * 6
    _, status, _ = bot_send(dev, cdb)
    assert status == 0, f"TUR failed: status={status}"
    return True

def test_02_e8_noop(dev):
    """E8 no-data vendor command x10."""
    for i in range(10):
        cdb = struct.pack('>BB13x', 0xE8, 0x00)
        _, status, _ = bot_send(dev, cdb)
        assert status == 0, f"E8 #{i} failed: status={status}"
    return True

def test_03_inquiry(dev):
    """SCSI INQUIRY (0x12) — returns vendor/product strings."""
    cdb = struct.pack('>BBBBBx', 0x12, 0, 0, 0, 36)
    _, status, data = bot_send(dev, cdb, rlen=36)
    assert status == 0, f"INQUIRY failed: status={status}"
    assert len(data) == 36, f"Expected 36 bytes, got {len(data)}"
    vendor = data[8:16].decode('ascii', errors='replace').strip()
    product = data[16:32].decode('ascii', errors='replace').strip()
    print(f"  Vendor='{vendor}' Product='{product}'")
    assert 'ASMT' in vendor or 'ASM' in vendor, f"Unexpected vendor: {vendor}"
    return True

def test_04_read_capacity(dev):
    """SCSI READ_CAPACITY(10) (0x25) — returns capacity info."""
    cdb = struct.pack('>BB8x', 0x25, 0)
    _, status, data = bot_send(dev, cdb, rlen=8)
    assert status == 0, f"READ_CAPACITY failed: status={status}"
    assert len(data) == 8, f"Expected 8 bytes, got {len(data)}"
    lba = struct.unpack('>I', data[0:4])[0]
    blksz = struct.unpack('>I', data[4:8])[0]
    print(f"  LastLBA={lba} BlockSize={blksz}")
    assert blksz == 512, f"Expected block size 512, got {blksz}"
    return True

# ============================================================
# Test 2: E5 writes / E4 reads
# ============================================================

def test_05_e5_e4_roundtrip(dev):
    """E5 write + E4 read roundtrip at 5 XDATA addresses."""
    for i, val in enumerate([0xA5, 0x5A, 0xFF, 0x00, 0x42]):
        addr = 0x5000 + i
        e5_write(dev, addr, val)
        got = e4_read(dev, addr, 1)[0]
        assert got == val, f"addr 0x{addr:04X}: wrote 0x{val:02X}, read 0x{got:02X}"
    return True

def test_06_e4_datain(dev):
    """E4 data-IN phase: 4-byte read after E5 seeding."""
    for i in range(4):
        e5_write(dev, 0x5030 + i, 0xA0 + i)
    data = e4_read_datain(dev, 0x5030, 4)
    for i in range(4):
        assert data[i] == 0xA0 + i, f"byte[{i}]: expected 0x{0xA0+i:02X}, got 0x{data[i]:02X}"
    return True

def test_07_e4_datain_stress(dev):
    """20 consecutive E4 data-IN reads (tests EP_COMPLETE state machine)."""
    for i in range(4):
        e5_write(dev, 0x5040 + i, 0x50 + i)
    for n in range(20):
        data = e4_read_datain(dev, 0x5040, 4)
        for i in range(4):
            assert data[i] == 0x50 + i, f"read #{n} byte[{i}]: expected 0x{0x50+i:02X}, got 0x{data[i]:02X}"
    return True

# ============================================================
# Test 3: Controller init E5 writes
# ============================================================

def test_08_controller_init_writes(dev):
    """ASM24Controller.__init__ E5 writes — all 10 init writes."""
    writes = [
        (0x054B, 0x20), (0x054E, 0x04), (0x05A8, 0x02), (0x05F8, 0x04),
        (0x07EC, 0x01), (0x07ED, 0x00), (0x07EE, 0x00), (0x07EF, 0x00),
        (0xC422, 0x02), (0x0000, 0x33),
    ]
    for addr, val in writes:
        e5_write(dev, addr, val)
    # Verify key values stuck
    assert e4_read(dev, 0x054B, 1)[0] == 0x20, "054B != 0x20"
    assert e4_read(dev, 0x0000, 1)[0] == 0x33, "0000 != 0x33"
    assert e4_read(dev, 0x07EC, 1)[0] == 0x01, "07EC != 0x01"
    return True

# ============================================================
# Test 4: PCIe config space
# ============================================================

def test_09_pcie_cfg_bus0(dev):
    """PCIe config read of bus 0 (ASM2464PD bridge VID/DID)."""
    usb = make_asm24(dev)
    vid = usb.pcie_cfg_req(0, bus=0, dev=0, fn=0, size=4)
    print(f"  Bus 0 VID/DID: 0x{vid:08X}")
    assert vid == 0x24631B21, f"Expected 0x24631B21, got 0x{vid:08X}"
    return True

def test_10_pcie_cfg_multi_reads(dev):
    """10 consecutive PCIe config reads (B296 stability)."""
    usb = make_asm24(dev)
    for i in range(10):
        vid = usb.pcie_cfg_req(0, bus=0, dev=0, fn=0, size=4)
        assert vid == 0x24631B21, f"Read #{i}: expected 0x24631B21, got 0x{vid:08X}"
    return True

def test_11_pcie_cfg_gpu(dev):
    """PCIe config read of GPU (bus 4) — sets up bus hierarchy first."""
    usb = make_asm24(dev)
    gpu_bus = 4
    for bus in range(gpu_bus):
        buses = (0 << 0) | ((bus + 1) << 8) | (gpu_bus << 16)
        usb.pcie_cfg_req(0x18, bus=bus, dev=0, fn=0, value=buses, size=4)
        usb.pcie_cfg_req(0x04, bus=bus, dev=0, fn=0, value=0x07, size=1)
    vid = usb.pcie_cfg_req(0, bus=gpu_bus, dev=0, fn=0, size=4)
    print(f"  Bus {gpu_bus} GPU VID/DID: 0x{vid:08X}")
    assert (vid & 0xFFFF) == 0x1002, f"Expected AMD VID 0x1002, got 0x{vid & 0xFFFF:04X}"
    return True

# ============================================================
# Test 5: PCIe BAR setup
# ============================================================

def test_12_pcie_setup_bars(dev):
    """Full pci_setup_usb_bars flow (bridge + BAR enumeration)."""
    usb = make_asm24(dev)
    from tinygrad.runtime.support.system import System
    bars = System.pci_setup_usb_bars(usb, gpu_bus=4, mem_base=0x10000000, pref_mem_base=(32 << 30))
    print(f"  BARs: { {k: f'addr=0x{v.addr:X} size=0x{v.size:X}' for k,v in bars.items()} }")
    assert 0 in bars, "BAR 0 not found"
    assert 5 in bars, "BAR 5 not found"
    return True

# ============================================================
# Test 6: PCIe memory reads
# ============================================================

def test_13_pcie_mem_read(dev):
    """PCIe memory read of GPU BAR5 (MMIO)."""
    usb = make_asm24(dev)
    from tinygrad.runtime.support.system import System
    bars = System.pci_setup_usb_bars(usb, gpu_bus=4, mem_base=0x10000000, pref_mem_base=(32 << 30))
    bar5_addr = bars[5].addr
    val = usb.pcie_mem_req(bar5_addr, size=4)
    print(f"  BAR5[0] = 0x{val:08X}")
    return True

def test_14_pcie_mem_multi_reads(dev):
    """10 consecutive PCIe memory reads (B296 polling stability)."""
    usb = make_asm24(dev)
    from tinygrad.runtime.support.system import System
    bars = System.pci_setup_usb_bars(usb, gpu_bus=4, mem_base=0x10000000, pref_mem_base=(32 << 30))
    bar5_addr = bars[5].addr
    for i in range(10):
        val = usb.pcie_mem_req(bar5_addr + i * 4, size=4)
    print(f"  10 reads OK")
    return True

# ============================================================
# Test 7: SCSI WRITE(16) — various sizes
# ============================================================

def test_15_scsi_write_512(dev):
    """SCSI WRITE(16) with 512 bytes (1 sector)."""
    data = bytes(range(256)) + bytes(range(256))
    status = scsi_write_raw(dev, data)
    assert status == 0, f"SCSI WRITE 512B failed: status={status}"
    return True

def test_16_scsi_write_4k(dev):
    """SCSI WRITE(16) with 4KB (8 sectors)."""
    data = bytes([(i * 7) & 0xFF for i in range(4096)])
    status = scsi_write_raw(dev, data)
    assert status == 0, f"SCSI WRITE 4KB failed: status={status}"
    return True

def test_17_scsi_write_64k(dev):
    """SCSI WRITE(16) with 64KB (128 sectors) — PSP firmware load size."""
    data = bytes([(i * 13) & 0xFF for i in range(0x10000)])
    status = scsi_write_raw(dev, data, timeout=30000)
    assert status == 0, f"SCSI WRITE 64KB failed: status={status}"
    return True

# ============================================================
# Test 8: SCSI WRITE + doorbell sequence
# ============================================================

def test_18_scsi_write_doorbell(dev):
    """SCSI WRITE(16) + E5 doorbell writes (0x171-0x173 + CE6E/CE6F).
    This is the exact sequence tinygrad's scsi_write() does per chunk."""
    data = bytes([(i * 3) & 0xFF for i in range(512)])
    status = scsi_write_raw(dev, data)
    assert status == 0, f"SCSI WRITE failed: status={status}"
    # Doorbell writes (tinygrad: WriteOp(0x171, b'\xff\xff\xff'))
    e5_write(dev, 0x0171, 0xFF)
    e5_write(dev, 0x0172, 0xFF)
    e5_write(dev, 0x0173, 0xFF)
    # DMA handshake clear (tinygrad: WriteOp(0xce6e, b'\x00\x00'))
    e5_write(dev, 0xCE6E, 0x00)
    e5_write(dev, 0xCE6F, 0x00)
    return True

def test_19_scsi_write_64k_doorbell(dev):
    """SCSI WRITE 64KB + full doorbell sequence + CE40-CE43 cleanup.
    Matches tinygrad scsi_write() for large transfers."""
    data = bytes([(i * 17) & 0xFF for i in range(0x10000)])
    status = scsi_write_raw(dev, data, timeout=30000)
    assert status == 0, f"SCSI WRITE 64KB failed: status={status}"
    # Doorbell
    e5_write(dev, 0x0171, 0xFF)
    e5_write(dev, 0x0172, 0xFF)
    e5_write(dev, 0x0173, 0xFF)
    # CE6E/CE6F
    e5_write(dev, 0xCE6E, 0x00)
    e5_write(dev, 0xCE6F, 0x00)
    # CE40-CE43 cleanup (only for transfers > 0x4000)
    e5_write(dev, 0xCE40, 0x00)
    e5_write(dev, 0xCE41, 0x00)
    e5_write(dev, 0xCE42, 0x00)
    e5_write(dev, 0xCE43, 0x00)
    return True

# ============================================================
# Test 9: SCSI WRITE data reaches PCI 0x200000 (PCIe readback)
# ============================================================

def test_20_scsi_write_f000_readback_512(dev):
    """Write 512B via SCSI WRITE, verify via F000 window (B26D=0x20)."""
    pattern = bytes([(i * 17 + 0xAB) & 0xFF for i in range(512)])
    status = scsi_write_raw(dev, pattern)
    assert status == 0, f"SCSI WRITE failed: status={status}"
    # Read back via F000 window (SRAM mapped at B26D=0x20)
    errors = []
    for offset in [0, 1, 2, 3, 0x100, 0x1FF]:
        got = e4_read(dev, 0xF000 + offset, 1)[0]
        expected = pattern[offset]
        if got != expected:
            errors.append(f"offset {offset:#x}: got 0x{got:02X}, expected 0x{expected:02X}")
    if errors:
        for e in errors:
            print(f"  FAIL: {e}")
        assert False, f"{len(errors)} readback mismatches"
    print(f"  6 offsets verified OK via F000 window")
    return True

def test_21_scsi_write_f000_readback_multi(dev):
    """Write 4KB, verify at multiple offsets within first 4KB page."""
    pattern = bytes([(i * 23 + 0x42) & 0xFF for i in range(4096)])
    status = scsi_write_raw(dev, pattern)
    assert status == 0, f"SCSI WRITE failed: status={status}"
    errors = []
    for offset in [0, 0x100, 0x400, 0x800, 0xC00, 0xFFF]:
        got = e4_read(dev, 0xF000 + offset, 1)[0]
        expected = pattern[offset]
        if got != expected:
            errors.append(f"offset {offset:#x}: got 0x{got:02X}, expected 0x{expected:02X}")
    if errors:
        for e in errors:
            print(f"  FAIL: {e}")
        assert False, f"{len(errors)} readback mismatches"
    print(f"  6 offsets verified OK")
    return True

def test_22_scsi_write_f000_readback_64k(dev):
    """Write 64KB via SCSI WRITE, verify all 16 pages via B26D sliding."""
    pattern = bytes([(i & 0xFF) for i in range(0x10000)])
    status = scsi_write_raw(dev, pattern, timeout=30000)
    assert status == 0, f"SCSI WRITE 64KB failed: status={status}"
    errors = []
    # Verify all 16 pages (B26D 0x20 through 0x2F)
    for page in range(16):
        b26d_val = 0x20 + page
        e5_write(dev, 0xB26D, b26d_val)
        base_offset = page * 0x1000
        for off in [0, 0x100, 0xFFF]:
            got = e4_read(dev, 0xF000 + off, 1)[0]
            data_off = base_offset + off
            expected = pattern[data_off]
            if got != expected:
                errors.append(f"page {page} off {off:#x} (data[{data_off:#x}]): got 0x{got:02X}, expected 0x{expected:02X}")
    # Restore B26D
    e5_write(dev, 0xB26D, 0x20)
    if errors:
        for e in errors[:5]:
            print(f"  FAIL: {e}")
        assert False, f"{len(errors)} readback mismatches"
    print(f"  All 16 pages (64KB) verified OK via B26D sliding")
    return True

# ============================================================
# Test 10: B296 clean after SCSI WRITE
# ============================================================

def test_23_b296_after_scsi_write(dev):
    """pcie_mem_req read works AFTER SCSI WRITE (B296 not corrupted)."""
    usb = make_asm24(dev)
    # Do SCSI WRITE first
    data = bytes(range(256)) + bytes(range(256))
    status = scsi_write_raw(dev, data)
    assert status == 0, f"SCSI WRITE failed: status={status}"
    # Now do a PCIe config read — this MUST work (B296 must be clean)
    vid = usb.pcie_cfg_req(0, bus=0, dev=0, fn=0, size=4)
    assert vid == 0x24631B21, f"Post-WRITE cfg_req failed: got 0x{vid:08X}"
    return True

def test_24_pcie_cfg_after_scsi_write(dev):
    """PCIe cfg_req works AFTER SCSI WRITE + doorbell sequence."""
    usb = make_asm24(dev)
    data = bytes([(i * 7) & 0xFF for i in range(512)])
    status = scsi_write_raw(dev, data)
    assert status == 0, f"SCSI WRITE failed"
    # Doorbell sequence (as tinygrad does)
    e5_write(dev, 0x0171, 0xFF)
    e5_write(dev, 0x0172, 0xFF)
    e5_write(dev, 0x0173, 0xFF)
    e5_write(dev, 0xCE6E, 0x00)
    e5_write(dev, 0xCE6F, 0x00)
    # PCIe config read should still work
    vid = usb.pcie_cfg_req(0, bus=0, dev=0, fn=0, size=4)
    assert vid == 0x24631B21, f"Post-WRITE+doorbell cfg_req failed: got 0x{vid:08X}"
    print(f"  PCIe cfg_req OK after SCSI WRITE + doorbell")
    return True

def test_25_interleaved_write_cfg(dev):
    """Multiple rounds: SCSI WRITE → F000 verify → PCIe cfg_req → ..."""
    usb = make_asm24(dev)
    for round_num in range(5):
        pattern = bytes([(i + round_num * 37) & 0xFF for i in range(512)])
        status = scsi_write_raw(dev, pattern)
        assert status == 0, f"Round {round_num}: SCSI WRITE failed"
        # Verify via F000
        got = e4_read(dev, 0xF000, 1)[0]
        expected = pattern[0]
        assert got == expected, f"Round {round_num}: F000 mismatch: got 0x{got:02X}, expected 0x{expected:02X}"
        # PCIe still works
        vid = usb.pcie_cfg_req(0, bus=0, dev=0, fn=0, size=4)
        assert vid == 0x24631B21, f"Round {round_num}: cfg_req failed"
    print(f"  5 rounds of WRITE + F000 verify + cfg_req OK")
    return True

# ============================================================
# Test 11: Full PSP-like flow
# ============================================================

def test_26_psp_like_flow(dev):
    """Simulate PSP firmware load: setup BARs → scsi_write → verify → pcie_cfg.
    This is the sequence tinygrad does in _bootloader_load_component."""
    usb = make_asm24(dev)
    from tinygrad.runtime.support.system import System
    
    # Phase 1: PCIe BAR setup (same as tinygrad)
    bars = System.pci_setup_usb_bars(usb, gpu_bus=4, mem_base=0x10000000, pref_mem_base=(32 << 30))
    bar5_addr = bars[5].addr
    print(f"  BAR5 at 0x{bar5_addr:08X}")
    
    # Phase 2: Read a GPU MMIO register (simulates _wait_for_bootloader polling)
    val = usb.pcie_mem_req(bar5_addr, size=4)
    print(f"  BAR5[0] = 0x{val:08X}")
    
    # Phase 3: SCSI WRITE 64KB (simulates _prep_msg1 → scsi_write)
    fw_blob = bytes([(i * 13 + 0x42) & 0xFF for i in range(0x10000)])
    status = scsi_write_raw(dev, fw_blob, timeout=30000)
    assert status == 0, f"SCSI WRITE 64KB failed: status={status}"
    
    # Phase 3b: Doorbell sequence (tinygrad scsi_write post-chunk)
    e5_write(dev, 0x0171, 0xFF)
    e5_write(dev, 0x0172, 0xFF)
    e5_write(dev, 0x0173, 0xFF)
    e5_write(dev, 0xCE6E, 0x00)
    e5_write(dev, 0xCE6F, 0x00)
    e5_write(dev, 0xCE40, 0x00)
    e5_write(dev, 0xCE41, 0x00)
    e5_write(dev, 0xCE42, 0x00)
    e5_write(dev, 0xCE43, 0x00)
    
    # Phase 4: Verify data in SRAM via F000 window (first + last page)
    got0 = e4_read(dev, 0xF000, 1)[0]
    assert got0 == fw_blob[0], f"F000[0] mismatch: got 0x{got0:02X}, expected 0x{fw_blob[0]:02X}"
    e5_write(dev, 0xB26D, 0x2F)  # Last page
    got_last = e4_read(dev, 0xFFFF, 1)[0]
    e5_write(dev, 0xB26D, 0x20)  # Restore
    assert got_last == fw_blob[0xFFFF], f"Last byte mismatch"
    print(f"  64KB data verified in SRAM (first + last byte)")
    
    # Phase 5: GPU MMIO still accessible (simulates post-load status polling)
    val2 = usb.pcie_mem_req(bar5_addr, size=4)
    print(f"  BAR5[0] = 0x{val2:08X} (post-write)")
    
    return True

def test_27_multi_chunk_psp_flow(dev):
    """Simulate multi-chunk PSP load: 3 × 64KB SCSI WRITEs with doorbell.
    Each chunk overwrites the SRAM buffer — verify each chunk's data."""
    for chunk_num in range(3):
        fw_chunk = bytes([((i + chunk_num * 100) & 0xFF) for i in range(0x10000)])
        status = scsi_write_raw(dev, fw_chunk, timeout=30000)
        assert status == 0, f"Chunk {chunk_num}: SCSI WRITE failed"
        # Doorbell
        e5_write(dev, 0x0171, 0xFF)
        e5_write(dev, 0x0172, 0xFF)
        e5_write(dev, 0x0173, 0xFF)
        e5_write(dev, 0xCE6E, 0x00)
        e5_write(dev, 0xCE6F, 0x00)
        # Verify this chunk's first byte via F000
        got = e4_read(dev, 0xF000, 1)[0]
        expected = fw_chunk[0]
        assert got == expected, f"Chunk {chunk_num}: F000 mismatch: got 0x{got:02X}, expected 0x{expected:02X}"
        print(f"  Chunk {chunk_num}: OK (verified via F000)")
    
    # Cleanup
    e5_write(dev, 0xCE40, 0x00)
    e5_write(dev, 0xCE41, 0x00)
    e5_write(dev, 0xCE42, 0x00)
    e5_write(dev, 0xCE43, 0x00)
    return True

# ============================================================
# Runner
# ============================================================

TESTS = [
    ("01 TUR",                          test_01_tur),
    ("02 E8 noop x10",                  test_02_e8_noop),
    ("03 INQUIRY",                       test_03_inquiry),
    ("04 READ_CAPACITY",                 test_04_read_capacity),
    ("05 E5+E4 roundtrip",              test_05_e5_e4_roundtrip),
    ("06 E4 data-IN",                    test_06_e4_datain),
    ("07 E4 data-IN stress x20",         test_07_e4_datain_stress),
    ("08 Controller init writes",        test_08_controller_init_writes),
    ("09 PCIe cfg bus 0",               test_09_pcie_cfg_bus0),
    ("10 PCIe cfg multi reads",          test_10_pcie_cfg_multi_reads),
    ("11 PCIe cfg GPU bus",              test_11_pcie_cfg_gpu),
    ("12 PCIe setup BARs",              test_12_pcie_setup_bars),
    ("13 PCIe mem read",                 test_13_pcie_mem_read),
    ("14 PCIe mem multi reads",          test_14_pcie_mem_multi_reads),
    ("15 SCSI WRITE 512B",              test_15_scsi_write_512),
    ("16 SCSI WRITE 4KB",               test_16_scsi_write_4k),
    ("17 SCSI WRITE 64KB",              test_17_scsi_write_64k),
    ("18 SCSI WRITE doorbell",           test_18_scsi_write_doorbell),
    ("19 SCSI WRITE 64KB doorbell",      test_19_scsi_write_64k_doorbell),
     ("20 SCSI WRITE F000 readback 512B", test_20_scsi_write_f000_readback_512),
     ("21 SCSI WRITE F000 readback multi", test_21_scsi_write_f000_readback_multi),
     ("22 SCSI WRITE F000 readback 64KB", test_22_scsi_write_f000_readback_64k),
     ("23 B296 clean after SCSI WRITE",   test_23_b296_after_scsi_write),
     ("24 pcie_cfg after SCSI WRITE",     test_24_pcie_cfg_after_scsi_write),
     ("25 Interleaved WRITE+cfg x5",      test_25_interleaved_write_cfg),
     ("26 PSP-like full flow",            test_26_psp_like_flow),
     ("27 Multi-chunk PSP flow",          test_27_multi_chunk_psp_flow),
]

def main():
    selected = set()
    stop_on_fail = True
    for arg in sys.argv[1:]:
        if arg == '--no-stop':
            stop_on_fail = False
        elif arg.isdigit():
            selected.add(int(arg))
        elif '-' in arg and not arg.startswith('-'):
            lo, hi = arg.split('-')
            for i in range(int(lo), int(hi) + 1):
                selected.add(i)

    dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)
    time.sleep(0.2)

    results = []
    for name, fn in TESTS:
        test_num = int(name.split()[0])
        if selected and test_num not in selected:
            continue
        print(f"\n--- {name} ---")
        try:
            ok = fn(dev)
            print(f"  PASS")
        except Exception as e:
            print(f"  FAIL: {e}")
            traceback.print_exc()
            ok = False
        results.append((name, ok))
        if not ok and stop_on_fail:
            print("\n  Stopping on first failure.")
            break

    print("\n" + "=" * 55)
    print("RESULTS:")
    passed = sum(1 for _, ok in results if ok)
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}: {name}")
    print(f"\n{passed}/{len(results)} tests passed")

    libusb.libusb_release_interface(dev.handle, 0)
    libusb.libusb_close(dev.handle)
    return 0 if passed == len(results) else 1

if __name__ == "__main__":
    sys.exit(main())
