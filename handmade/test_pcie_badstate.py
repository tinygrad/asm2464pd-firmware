#!/usr/bin/env python3
"""
Bad-state tests for ASM2464PD firmware.

Try to get the firmware into broken states through:
- Abandoned transfers (send fewer dwords than promised)
- Mode switches mid-transfer
- Operations after incomplete transfers
- Double setups without data
- Unexpected bulk traffic
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
  fmt = {0: 0, 1: MWR64, 2: MRD64}[mode]
  wval = fmt | (0x0F << 8)
  widx = mode & 0x03
  payload = struct.pack('<III', addr & 0xFFFFFFFF, addr >> 32, ndwords)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, widx, buf, 12, 5000)
  assert ret >= 0, f"F0 setup failed: {ret}"

def stream_write(handle, addr, data):
  ndwords = len(data) // 4
  dma_setup(handle, addr, 1, ndwords)
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), 30000)
  assert ret == 0, f"bulk write failed: {ret} (transferred {transferred.value})"
  assert transferred.value == len(data), f"short write: {transferred.value}/{len(data)}"
  time.sleep(0.001)

def stream_read(handle, addr, nbytes, timeout=5000):
  ndwords = nbytes // 4
  dma_setup(handle, addr, 2, ndwords)
  resp = (ctypes.c_ubyte * nbytes)()
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), timeout)
  assert ret == 0, f"bulk read failed: {ret}"
  return bytes(resp[:transferred.value])

def single_tlp_read(handle, addr):
  wval = MRD64 | (0x0F << 8)
  payload = struct.pack('<III', addr & 0xFFFFFFFF, addr >> 32, 0)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
  assert ret >= 0
  resp = (ctypes.c_ubyte * 8)()
  ret = libusb.libusb_control_transfer(handle, 0xC0, 0xF0, 0, 0, resp, 8, 5000)
  assert ret == 8
  assert resp[7] == 0, f"TLP error: status={resp[7]}"
  return struct.unpack('<I', bytes(resp[0:4]))[0]

def single_tlp_write(handle, addr, value):
  wval = MWR64 | (0x0F << 8)
  payload = struct.pack('<III', addr & 0xFFFFFFFF, addr >> 32, value)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
  assert ret >= 0

def make_pattern(n_dwords, seed=0):
  return array.array('I', [(seed + i) ^ (0xA5A5A5A5 * (i & 1)) for i in range(n_dwords)]).tobytes()

def clear_halt(handle, ep):
  """Clear a halted endpoint."""
  libusb.libusb_clear_halt(handle, ep)

def try_bulk_in(handle, nbytes, timeout=1000):
  """Non-asserting bulk IN — returns (ret, transferred, data)."""
  resp = (ctypes.c_ubyte * nbytes)()
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), timeout)
  return ret, transferred.value, bytes(resp[:transferred.value])

def try_bulk_out(handle, data, timeout=1000):
  """Non-asserting bulk OUT — returns (ret, transferred)."""
  buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(data), ctypes.byref(transferred), timeout)
  return ret, transferred.value


class TestBadStates(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    ctx = ctypes.POINTER(libusb.libusb_context)()
    libusb.libusb_init(ctypes.byref(ctx))
    cls._ctx = ctx
    cls.handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
    if not cls.handle: raise unittest.SkipTest("Device not found")
    libusb.libusb_claim_interface(cls.handle, 0)
    ltssm = xdata_read(cls.handle, 0xB450, 1)[0]
    if ltssm != 0x78: raise unittest.SkipTest(f"PCIe link not up (LTSSM=0x{ltssm:02X})")
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

  def setUp(self):
    """Reset firmware and host state before each test."""
    self._reset_state()

  def _reset_state(self):
    """Get firmware + host back to a clean state."""
    # Clear halts — firmware handles CLEAR_FEATURE(ENDPOINT_HALT)
    # which resets the endpoint and sets dma_dwords=0
    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)
    # Drain any data that was already in the USB TX FIFO before CLEAR_IN took effect
    try_bulk_in(self.handle, 65536, timeout=100)
    clear_halt(self.handle, EP_IN)

  def _verify_normal_op(self, label=""):
    """Verify firmware is still functional with a basic write/read cycle."""
    self._reset_state()
    addr = self.vram + 0xF0000
    vals = [0xDEAD0000 | i for i in range(16)]
    data = array.array('I', vals).tobytes()
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    got_vals = list(array.array('I', got))
    self.assertEqual(got_vals, vals, f"normal op check failed after: {label}")

  # === Abandoned write tests ===

  def test_abandoned_write_then_read(self):
    """Set up write for 100 dwords, send only 16, then do a normal read."""
    addr = self.vram + 0x100000
    # Write 16 dwords of known data first so we can verify the read
    data16 = make_pattern(16, seed=0xAB)
    stream_write(self.handle, addr, data16)

    # Now set up a write for 100 dwords but only send 16
    dma_setup(self.handle, addr + 0x1000, 1, 100)
    partial = make_pattern(16, seed=0xCD)
    ret, xferred = try_bulk_out(self.handle, partial, timeout=1000)
    # Partial send might succeed or fail depending on timing
    time.sleep(0.01)
    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)

    # Now do a normal read — this sends a new 0xF0 control which should reset state
    got = stream_read(self.handle, addr, 16 * 4)
    self.assertEqual(got, data16, "read after abandoned write returned wrong data")

  def test_abandoned_write_then_write(self):
    """Set up write for 100 dwords, send only 16, then do another write+read."""
    addr_a = self.vram + 0x110000
    addr_b = self.vram + 0x111000

    # Partial write
    dma_setup(self.handle, addr_a, 1, 100)
    partial = make_pattern(16, seed=0x111)
    try_bulk_out(self.handle, partial, timeout=1000)
    time.sleep(0.01)
    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)

    # New complete write to different address
    data32 = make_pattern(32, seed=0x222)
    stream_write(self.handle, addr_b, data32)
    got = stream_read(self.handle, addr_b, len(data32))
    self.assertEqual(got, data32, "write after abandoned write failed")

  @unittest.skip("flaky — stale IN drain timing")
  def test_abandoned_read_then_write(self):
    """Set up read for 256 dwords but only consume 64 bytes, then do a write."""
    addr = self.vram + 0x120000
    # First write known data
    data = make_pattern(256, seed=0x333)
    stream_write(self.handle, addr, data)

    # Set up read for 256 dwords — firmware preps first chunk and arms IN
    dma_setup(self.handle, addr, 2, 256)
    # Read only the first chunk
    ret, xferred, _ = try_bulk_in(self.handle, 64, timeout=1000)
    time.sleep(0.01)

    # Clear any stale state
    clear_halt(self.handle, EP_IN)
    clear_halt(self.handle, EP_OUT)

    # Now do a normal write + read
    addr2 = self.vram + 0x121000
    data2 = make_pattern(32, seed=0x444)
    stream_write(self.handle, addr2, data2)
    got = stream_read(self.handle, addr2, len(data2))
    self.assertEqual(got, data2, "write/read after abandoned read failed")

  # === Double setup tests ===

  def test_double_write_setup_no_data(self):
    """Send two 0xF0 write setups without any bulk data, then do normal op."""
    dma_setup(self.handle, self.vram + 0x130000, 1, 100)
    time.sleep(0.005)
    dma_setup(self.handle, self.vram + 0x131000, 1, 200)
    time.sleep(0.005)
    self._verify_normal_op("double write setup")

  def test_double_read_setup_no_data(self):
    """Send two 0xF0 read setups without reading bulk data."""
    dma_setup(self.handle, self.vram + 0x140000, 2, 100)
    time.sleep(0.005)
    # The firmware armed IN for the first setup. Second setup overwrites dma_dwords
    # and arms IN again. There might be stale IN data.
    clear_halt(self.handle, EP_IN)
    dma_setup(self.handle, self.vram + 0x141000, 2, 200)
    time.sleep(0.005)
    clear_halt(self.handle, EP_IN)
    self._verify_normal_op("double read setup")

  def test_write_setup_then_read_setup(self):
    """Set up write, then immediately set up read without sending write data."""
    addr = self.vram + 0x150000
    # First write known data
    data = make_pattern(64, seed=0x555)
    stream_write(self.handle, addr, data)

    # Set up write (arms OUT) but don't send data
    dma_setup(self.handle, addr + 0x1000, 1, 100)
    time.sleep(0.005)

    # Now set up read — this should override the write state
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data, "read after abandoned write setup failed")

  def test_read_setup_then_write_setup(self):
    """Set up read, then immediately set up write without reading."""
    addr = self.vram + 0x160000

    # Set up read (arms IN, firmware preps first chunk)
    dma_setup(self.handle, addr, 2, 100)
    time.sleep(0.005)
    # Drain any IN data that was already sent to host before we can cancel
    try_bulk_in(self.handle, 65536, timeout=200)
    clear_halt(self.handle, EP_IN)
    clear_halt(self.handle, EP_OUT)

    # Set up write — should override read state
    data = make_pattern(32, seed=0x666)
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data, "write/read after abandoned read setup failed")

  # === Zero-length tests ===

  def test_zero_dwords_write_setup(self):
    """Set up write with 0 dwords, then normal op."""
    dma_setup(self.handle, self.vram + 0x170000, 1, 0)
    time.sleep(0.005)
    self._verify_normal_op("zero dwords write")

  def test_zero_dwords_read_setup(self):
    """Set up read with 0 dwords, then normal op."""
    dma_setup(self.handle, self.vram + 0x180000, 2, 0)
    time.sleep(0.005)
    self._verify_normal_op("zero dwords read")

  # === Single TLP interleaving ===

  def test_single_tlp_during_write_stream(self):
    """Start a write stream, do a single TLP read mid-stream, then continue."""
    addr = self.vram + 0x190000
    # Write a known value via single TLP first
    single_tlp_write(self.handle, addr, 0x12345678)

    # Start a streaming write to a different address
    addr2 = self.vram + 0x191000
    dma_setup(self.handle, addr2, 1, 100)
    # Send partial data
    partial = make_pattern(16, seed=0x777)
    try_bulk_out(self.handle, partial, timeout=1000)
    time.sleep(0.005)
    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)

    # Now do a single-TLP read from the first address
    # This sends a new 0xF0 mode=0 which will change PCIe addr registers
    got = single_tlp_read(self.handle, addr)
    self.assertEqual(got, 0x12345678, "single TLP read during write stream got wrong value")

    # Try to continue normal operation
    self._verify_normal_op("single TLP during write stream")

  def test_single_tlp_during_read_stream(self):
    """Start a read stream, do a single TLP write mid-stream, then verify."""
    addr = self.vram + 0x1A0000
    data = make_pattern(256, seed=0x888)
    stream_write(self.handle, addr, data)

    # Start read stream
    dma_setup(self.handle, addr, 2, 256)
    # Read first chunk only
    ret, xferred, got_partial = try_bulk_in(self.handle, 1024, timeout=1000)
    time.sleep(0.005)
    clear_halt(self.handle, EP_IN)
    clear_halt(self.handle, EP_OUT)

    # Do a single TLP write somewhere else — this 0xF0 resets firmware state
    single_tlp_write(self.handle, self.vram + 0x1A1000, 0xAAAABBBB)

    # Verify the single TLP write landed
    got = single_tlp_read(self.handle, self.vram + 0x1A1000)
    self.assertEqual(got, 0xAAAABBBB, "single TLP write during read stream failed")

    # Verify normal op still works
    self._verify_normal_op("single TLP during read stream")

  # === Recovery tests ===

  def test_recovery_after_multiple_abandoned(self):
    """Abandon 5 writes and 5 reads, then verify normal op."""
    for i in range(5):
      dma_setup(self.handle, self.vram + i * 0x1000, 1, 100)
      try_bulk_out(self.handle, make_pattern(4, seed=i), timeout=500)
      time.sleep(0.002)
      clear_halt(self.handle, EP_OUT)

    for i in range(5):
      dma_setup(self.handle, self.vram + i * 0x1000, 2, 100)
      try_bulk_in(self.handle, 256, timeout=500)
      clear_halt(self.handle, EP_IN)
      time.sleep(0.002)

    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)
    self._verify_normal_op("5 abandoned writes + 5 abandoned reads")

  def test_rapid_mode_switching(self):
    """Rapidly switch between write and read setups 50 times, then verify."""
    for i in range(50):
      mode = 1 + (i % 2)  # alternate 1 and 2
      dma_setup(self.handle, self.vram + 0x1B0000, mode, 16)
    time.sleep(0.01)
    clear_halt(self.handle, EP_IN)
    clear_halt(self.handle, EP_OUT)
    self._verify_normal_op("50 rapid mode switches")

  # === Large dword count ===

  def test_very_large_dword_count(self):
    """Set up with a huge dword count (0x100000 = 4MB), send 16 dwords, then normal op."""
    dma_setup(self.handle, self.vram + 0x1C0000, 1, 0x100000)
    partial = make_pattern(16, seed=0x999)
    try_bulk_out(self.handle, partial, timeout=1000)
    time.sleep(0.01)
    clear_halt(self.handle, EP_OUT)
    clear_halt(self.handle, EP_IN)
    self._verify_normal_op("huge dword count abandoned write")

  def test_max_uint32_dword_count(self):
    """Set up with dword count = 0xFFFFFFFF, then normal op."""
    dma_setup(self.handle, self.vram + 0x1D0000, 1, 0xFFFFFFFF)
    time.sleep(0.005)
    self._verify_normal_op("max uint32 dword count")


if __name__ == '__main__':
  pytest.main([__file__, "-v", "-s", "--tb=short", *sys.argv[1:]])
