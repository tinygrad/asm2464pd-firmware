#!/usr/bin/env python3
"""
PCIe streaming DMA tests for ASM2464PD handmade firmware.
Tests 0xF0 vendor command in various patterns: write, read, transitions, offsets.
Requires PCIe link up + GPU present.
"""
import ctypes, struct, unittest, time, sys, os, array, pytest
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001
EP_OUT, EP_IN = 0x02, 0x81
MWR64, MRD64 = 0x60, 0x20
GPU_BUS = 4
READ_CHUNK = 16  # dwords per bulk IN chunk

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pcie'))
from pcie_probe import setup_bridges, assign_bars, resize_bars, xdata_read

def dma_setup(handle, addr, mode, count=0):
  """0xF0: wValue = fmt_type|(be<<8), wIndex = mode|(count<<2). DATA_OUT = addr[8] + value[4]."""
  fmt = {0: 0, 1: MWR64, 2: MRD64}[mode]
  wval = fmt | (0x0F << 8)
  widx = (mode & 0x03) | ((count & 0x3F) << 2)
  payload = struct.pack('<II', addr & 0xFFFFFFFF, addr >> 32) + struct.pack('>I', 0)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, widx, buf, 12, 5000)
  assert ret >= 0, f"F0 setup failed: {ret}"

def stream_write(handle, addr, data):
  """Set up streaming write mode, then bulk OUT the data."""
  dma_setup(handle, addr, 1)
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), 30000)
  assert ret == 0, f"bulk write failed: {ret} (transferred {transferred.value})"
  # Wait for last bulk packet to be processed by firmware ISR.
  # USB hardware blocks bulk while control is active, so we need a gap.
  time.sleep(0.001)

def stream_read(handle, addr, nbytes, chunk=READ_CHUNK):
  """Set up streaming read mode, then bulk IN the data."""
  dma_setup(handle, addr, 2, chunk)
  chunk_bytes = chunk * 4
  resp = (ctypes.c_ubyte * chunk_bytes)()
  result = bytearray()
  transferred = ctypes.c_int()
  while len(result) < nbytes:
    ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, chunk_bytes, ctypes.byref(transferred), 5000)
    assert ret == 0, f"bulk read failed: {ret}"
    result.extend(bytes(resp[:transferred.value]))
  return bytes(result[:nbytes])

def make_be_data(dwords):
  """Build big-endian dword array for PCIe writes."""
  arr = array.array('I', dwords)
  if sys.byteorder == 'little':
    arr.byteswap()
  return arr.tobytes()

def parse_be_data(data):
  """Parse big-endian dword data from PCIe reads."""
  arr = array.array('I')
  arr.frombytes(data)
  if sys.byteorder == 'little':
    arr.byteswap()
  return list(arr)

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
    data = make_be_data(range(n))
    stream_write(self.handle, self.vram, data)
    got = stream_read(self.handle, self.vram, len(data))
    self.assertEqual(parse_be_data(got), list(range(n)))

  def test_f0_write_read_4k(self):
    """Write 4KB, read back, verify."""
    n = 1024
    data = make_be_data(range(n))
    stream_write(self.handle, self.vram, data)
    got = stream_read(self.handle, self.vram, len(data))
    self.assertEqual(parse_be_data(got), list(range(n)))

  def test_f0_write_read_offset(self):
    """Write at offset 0x1000, read back from same offset."""
    n = 256
    offset = 0x1000
    vals = [0xDEAD0000 | i for i in range(n)]
    data = make_be_data(vals)
    stream_write(self.handle, self.vram + offset, data)
    got = stream_read(self.handle, self.vram + offset, len(data))
    self.assertEqual(parse_be_data(got), vals)

  def test_f0_back_to_back_writes(self):
    """Two writes to different offsets, then read both back."""
    n = 128
    vals_a = [0xAAAA0000 | i for i in range(n)]
    vals_b = [0xBBBB0000 | i for i in range(n)]
    stream_write(self.handle, self.vram, make_be_data(vals_a))
    stream_write(self.handle, self.vram + n * 4, make_be_data(vals_b))
    got_a = parse_be_data(stream_read(self.handle, self.vram, n * 4))
    got_b = parse_be_data(stream_read(self.handle, self.vram + n * 4, n * 4))
    self.assertEqual(got_a, vals_a)
    self.assertEqual(got_b, vals_b)

  def test_f0_back_to_back_reads(self):
    """Write once, read twice from same address."""
    n = 128
    vals = [0xCCCC0000 | i for i in range(n)]
    stream_write(self.handle, self.vram, make_be_data(vals))
    got1 = parse_be_data(stream_read(self.handle, self.vram, n * 4))
    got2 = parse_be_data(stream_read(self.handle, self.vram, n * 4))
    self.assertEqual(got1, vals)
    self.assertEqual(got2, vals)

  def test_f0_write_read_write_read(self):
    """Alternating write/read pattern."""
    n = 128
    for i in range(3):
      vals = [(i << 24) | j for j in range(n)]
      stream_write(self.handle, self.vram, make_be_data(vals))
      got = parse_be_data(stream_read(self.handle, self.vram, n * 4))
      self.assertEqual(got, vals, f"mismatch on iteration {i}")

  def test_f0_read_without_write(self):
    """Read from VRAM without prior write (should not hang)."""
    got = stream_read(self.handle, self.vram, 64)
    self.assertEqual(len(got), 64)

  def test_f0_write_read_16mb_boundary(self):
    """Write/read spanning a 16MB boundary in VRAM, verified with single-TLP reads."""
    # Pick the first 16MB boundary above vram base.
    boundary = (self.vram + 0x1000000) & ~0xFFFFFF
    offset = boundary - 256  # start 256 bytes before the boundary
    n = 128  # 128 dwords = 512 bytes, straddles the boundary
    vals = [0xF0F00000 | i for i in range(n)]
    data = make_be_data(vals)
    stream_write(self.handle, offset, data)
    # Verify with single-TLP reads (mode=0) — these set the full address each time
    for i in range(n):
      addr = offset + i * 4
      # mode=0 single TLP: send F0 OUT with MRD64 fmt, then F0 IN for completion
      wval = MRD64 | (0x0F << 8)
      payload = struct.pack('<II', addr & 0xFFFFFFFF, addr >> 32) + struct.pack('>I', 0)
      buf = (ctypes.c_ubyte * 12)(*payload)
      ret = libusb.libusb_control_transfer(self.handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
      assert ret >= 0, f"F0 OUT failed at dword {i}: ret={ret}"
      resp = (ctypes.c_ubyte * 8)()
      ret = libusb.libusb_control_transfer(self.handle, 0xC0, 0xF0, 0, 0, resp, 8, 5000)
      assert ret == 8, f"F0 IN read failed at dword {i}: ret={ret}"
      assert resp[7] == 0, f"TLP error at dword {i} (addr 0x{addr:X}): status={resp[7]}"
      got_dword = struct.unpack('>I', bytes(resp[0:4]))[0]
      self.assertEqual(got_dword, vals[i],
        f"mismatch at dword {i} (addr 0x{addr:X}): got 0x{got_dword:08X}, expected 0x{vals[i]:08X}")

if __name__ == '__main__':
  pytest.main([__file__, "-v", "-s", *sys.argv[1:]])
