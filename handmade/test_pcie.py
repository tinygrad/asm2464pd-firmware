#!/usr/bin/env python3
"""
PCIe streaming DMA tests for ASM2464PD handmade firmware.
Tests 0xF0 vendor command in various patterns: write, read, transitions, offsets.
Requires PCIe link up + GPU present.

Streaming data (mode 1/2) is little-endian dwords on the wire.
Single-TLP data (mode 0) returns raw PCIe completion bytes (DATA_0..DATA_3).
"""
import ctypes, struct, unittest, time, sys, os, array, pytest
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001
EP_OUT, EP_IN = 0x02, 0x81
MWR64, MRD64 = 0x60, 0x20
GPU_BUS = 4

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pcie'))
from pcie_probe import setup_bridges, assign_bars, resize_bars, xdata_read

def dma_setup(handle, addr, mode, ndwords=0):
  """0xF0: wValue = fmt_type|(be<<8), wIndex = mode. DATA_OUT = addr[4 LE] + addr_hi[4 LE] + value[4 LE]."""
  fmt = {0: 0, 1: MWR64, 2: MRD64}[mode]
  wval = fmt | (0x0F << 8)
  widx = mode & 0x03
  payload = struct.pack('<III', addr & 0xFFFFFFFF, addr >> 32, ndwords)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, widx, buf, 12, 5000)
  assert ret >= 0, f"F0 setup failed: {ret}"

def make_le_data(dwords):
  """Build little-endian dword array for streaming PCIe writes."""
  return array.array('I', dwords).tobytes()

def parse_le_data(data):
  """Parse little-endian dword data from streaming PCIe reads."""
  arr = array.array('I')
  arr.frombytes(data)
  return list(arr)

def stream_write(handle, addr, data):
  """Set up streaming write mode, then bulk OUT the data."""
  dma_setup(handle, addr, 1, len(data) // 4)
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), 30000)
  assert ret == 0, f"bulk write failed: {ret} (transferred {transferred.value})"
  time.sleep(0.001)

def stream_read(handle, addr, nbytes):
  """Set up streaming read mode, then bulk IN the data."""
  ndwords = nbytes // 4
  dma_setup(handle, addr, 2, ndwords)
  resp = (ctypes.c_ubyte * nbytes)()
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), 5000)
  assert ret == 0, f"bulk read failed: {ret}"
  return bytes(resp[:transferred.value])

def single_tlp_write(handle, addr, value, size=4):
  """Write a single dword/byte via mode 0 single TLP."""
  offset = addr & 0x3
  aligned = addr & ~0x3
  byte_en = ((1 << size) - 1) << offset
  # Pack value into the correct byte lane (shifted left by offset bytes)
  packed = (value & ((1 << (8 * size)) - 1)) << (8 * offset)
  wval = MWR64 | (byte_en << 8)
  payload = struct.pack('<III', aligned & 0xFFFFFFFF, aligned >> 32, packed)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
  assert ret >= 0, f"F0 single write failed: {ret}"

def single_tlp_read(handle, addr, size=4):
  """Read a single dword/byte via mode 0 single TLP. Returns integer value."""
  offset = addr & 0x3
  aligned = addr & ~0x3
  byte_en = ((1 << size) - 1) << offset
  wval = MRD64 | (byte_en << 8)
  payload = struct.pack('<III', aligned & 0xFFFFFFFF, aligned >> 32, 0)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
  assert ret >= 0, f"F0 single read OUT failed: {ret}"
  resp = (ctypes.c_ubyte * 8)()
  ret = libusb.libusb_control_transfer(handle, 0xC0, 0xF0, 0, 0, resp, 8, 5000)
  assert ret == 8, f"F0 single read IN failed: {ret}"
  assert resp[7] == 0, f"TLP error (addr 0x{addr:X}): status={resp[7]}"
  dword = struct.unpack('<I', bytes(resp[0:4]))[0]
  return (dword >> (8 * offset)) & ((1 << (8 * size)) - 1)

class TestF0(unittest.TestCase):
  """Tests for 0xF0 PCIe streaming DMA."""

  @classmethod
  def setUpClass(cls):
    ctx = ctypes.POINTER(libusb.libusb_context)()
    libusb.libusb_init(ctypes.byref(ctx))
    cls._ctx = ctx
    cls.handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
    if not cls.handle:
      raise unittest.SkipTest("Device not found")
    libusb.libusb_claim_interface(cls.handle, 0)
    # check PCIe link
    ltssm = xdata_read(cls.handle, 0xB450, 1)[0]
    if ltssm != 0x78:
      raise unittest.SkipTest(f"PCIe link not up (LTSSM=0x{ltssm:02X})")
    gpu_bus = setup_bridges(cls.handle, GPU_BUS)
    resize_bars(cls.handle, gpu_bus)
    bars = assign_bars(cls.handle, gpu_bus)
    cls.vram = bars[0][0]
    print(f"VRAM BAR0: 0x{cls.vram:X}")

  @classmethod
  def tearDownClass(cls):
    libusb.libusb_release_interface(cls.handle, 0)
    libusb.libusb_close(cls.handle)
    libusb.libusb_exit(cls._ctx)

  def test_f0_write_read_small(self):
    """Write 512 bytes (1 bulk packet), read back, verify."""
    n = 128  # dwords
    data = make_le_data(range(n))
    stream_write(self.handle, self.vram, data)
    got = stream_read(self.handle, self.vram, len(data))
    self.assertEqual(parse_le_data(got), list(range(n)))

  def test_f0_write_read_4k(self):
    """Write 4KB, read back, verify."""
    n = 1024
    data = make_le_data(range(n))
    stream_write(self.handle, self.vram, data)
    got = stream_read(self.handle, self.vram, len(data))
    self.assertEqual(parse_le_data(got), list(range(n)))

  def test_f0_write_read_offset(self):
    """Write at offset 0x1000, read back from same offset."""
    n = 256
    offset = 0x1000
    vals = [0xDEAD0000 | i for i in range(n)]
    data = make_le_data(vals)
    stream_write(self.handle, self.vram + offset, data)
    got = stream_read(self.handle, self.vram + offset, len(data))
    self.assertEqual(parse_le_data(got), vals)

  def test_f0_back_to_back_writes(self):
    """Two writes to different offsets, then read both back."""
    n = 128
    vals_a = [0xAAAA0000 | i for i in range(n)]
    vals_b = [0xBBBB0000 | i for i in range(n)]
    stream_write(self.handle, self.vram, make_le_data(vals_a))
    stream_write(self.handle, self.vram + n * 4, make_le_data(vals_b))
    got_a = parse_le_data(stream_read(self.handle, self.vram, n * 4))
    got_b = parse_le_data(stream_read(self.handle, self.vram + n * 4, n * 4))
    self.assertEqual(got_a, vals_a)
    self.assertEqual(got_b, vals_b)

  def test_f0_back_to_back_reads(self):
    """Write once, read twice from same address."""
    n = 128
    vals = [0xCCCC0000 | i for i in range(n)]
    stream_write(self.handle, self.vram, make_le_data(vals))
    got1 = parse_le_data(stream_read(self.handle, self.vram, n * 4))
    got2 = parse_le_data(stream_read(self.handle, self.vram, n * 4))
    self.assertEqual(got1, vals)
    self.assertEqual(got2, vals)

  def test_f0_write_read_write_read(self):
    """Alternating write/read pattern."""
    n = 128
    for i in range(3):
      vals = [(i << 24) | j for j in range(n)]
      stream_write(self.handle, self.vram, make_le_data(vals))
      got = parse_le_data(stream_read(self.handle, self.vram, n * 4))
      self.assertEqual(got, vals, f"mismatch on iteration {i}")

  def test_f0_read_without_write(self):
    """Read from VRAM without prior write (should not hang)."""
    got = stream_read(self.handle, self.vram, 64)
    self.assertEqual(len(got), 64)

  def test_f0_write_read_16mb_boundary(self):
    """Write/read spanning a 16MB boundary in VRAM, verified with single-TLP reads."""
    boundary = (self.vram + 0x1000000) & ~0xFFFFFF
    offset = boundary - 256  # start 256 bytes before the boundary
    n = 128  # 128 dwords = 512 bytes, straddles the boundary
    vals = [0xF0F00000 | i for i in range(n)]
    data = make_le_data(vals)
    stream_write(self.handle, offset, data)
    # Verify with single-TLP reads (mode=0) — these set the full address each time
    for i in range(n):
      addr = offset + i * 4
      got_dword = single_tlp_read(self.handle, addr)
      self.assertEqual(got_dword, vals[i],
        f"mismatch at dword {i} (addr 0x{addr:X}): got 0x{got_dword:08X}, expected 0x{vals[i]:08X}")

  def test_f0_single_byte_write_read(self):
    """Write and read individual bytes via single-TLP mode 0."""
    base = self.vram + 0x2000
    # Write a known dword first to clear the area
    single_tlp_write(self.handle, base, 0x00000000)
    # Write individual bytes at each offset within a dword
    for off in range(4):
      val = 0x10 + off * 0x11  # 0x10, 0x21, 0x32, 0x43
      single_tlp_write(self.handle, base + off, val, size=1)
    # Read back the full dword
    got = single_tlp_read(self.handle, base)
    self.assertEqual(got, 0x43322110,
      f"dword after byte writes: got 0x{got:08X}, expected 0x43322110")
    # Read back each byte individually
    for off in range(4):
      expected = 0x10 + off * 0x11
      got_byte = single_tlp_read(self.handle, base + off, size=1)
      self.assertEqual(got_byte, expected,
        f"byte at offset {off}: got 0x{got_byte:02X}, expected 0x{expected:02X}")

if __name__ == '__main__':
  pytest.main([__file__, "-v", "-s", *sys.argv[1:]])
