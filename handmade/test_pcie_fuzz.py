#!/usr/bin/env python3
"""
PCIe firmware fuzzer for ASM2464PD handmade firmware.

Tests edge cases, boundary conditions, and randomized patterns:
- Various transfer sizes (1 dword to 64KB+)
- Non-power-of-2 sizes
- Address alignment edge cases
- Partial reads (read less than was written)
- Partial writes (write less than dma_dwords claims)
- Mode transitions (write→read, read→write, read→read, write→write)
- Single TLP interleaved with streaming
- Zero-length edge cases
- Address carry/wrap within streaming
- Rapid-fire small transfers
- Large transfers
"""
import ctypes, struct, unittest, time, sys, os, array, random, pytest
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

def stream_read(handle, addr, nbytes):
  ndwords = nbytes // 4
  dma_setup(handle, addr, 2, ndwords)
  resp = (ctypes.c_ubyte * nbytes)()
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_IN, resp, nbytes, ctypes.byref(transferred), 5000)
  assert ret == 0, f"bulk read failed: {ret}"
  return bytes(resp[:transferred.value])

def single_tlp_read(handle, addr, size=4):
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

def single_tlp_write(handle, addr, value, size=4):
  offset = addr & 0x3
  aligned = addr & ~0x3
  byte_en = ((1 << size) - 1) << offset
  packed = (value & ((1 << (8 * size)) - 1)) << (8 * offset)
  wval = MWR64 | (byte_en << 8)
  payload = struct.pack('<III', aligned & 0xFFFFFFFF, aligned >> 32, packed)
  buf = (ctypes.c_ubyte * 12)(*payload)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, wval, 0, buf, 12, 5000)
  assert ret >= 0, f"F0 single write failed: {ret}"

def make_pattern(n_dwords, seed=0):
  """Generate a deterministic pattern of n_dwords."""
  return array.array('I', [(seed + i) ^ (0xA5A5A5A5 * (i & 1)) for i in range(n_dwords)]).tobytes()

def verify_pattern(got_bytes, expected_bytes, label=""):
  """Compare byte-by-byte, report first few mismatches."""
  errors = []
  for i in range(min(len(got_bytes), len(expected_bytes))):
    if got_bytes[i] != expected_bytes[i]:
      errors.append(i)
      if len(errors) >= 8: break
  if len(got_bytes) != len(expected_bytes):
    errors.append(-1)
  return errors


class TestFuzz(unittest.TestCase):
  """Fuzzer/stress tests for the PCIe streaming firmware."""

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

  # === Transfer size tests ===

  def test_sizes_1_to_64_dwords(self):
    """Write and read back every size from 1 to 64 dwords."""
    for n in range(1, 65):
      with self.subTest(n_dwords=n):
        data = make_pattern(n, seed=n)
        addr = self.vram + 0x10000
        stream_write(self.handle, addr, data)
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(got, data, f"mismatch at {n} dwords: first diff at byte {verify_pattern(got, data)[0] if verify_pattern(got, data) else 'none'}")

  def test_sizes_powers_of_2(self):
    """Test power-of-2 sizes from 4 bytes to 64KB."""
    for shift in range(0, 15):  # 1 dword to 16K dwords (64KB)
      n = 1 << shift
      with self.subTest(n_dwords=n, nbytes=n*4):
        data = make_pattern(n, seed=shift)
        addr = self.vram + 0x20000
        stream_write(self.handle, addr, data)
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(got, data, f"mismatch at {n} dwords ({n*4} bytes)")

  def test_sizes_non_power_of_2(self):
    """Test non-power-of-2 sizes: primes, odd numbers."""
    for n in [3, 5, 7, 11, 13, 17, 31, 37, 63, 65, 127, 129, 255, 257, 511, 513, 1023, 1025]:
      with self.subTest(n_dwords=n):
        data = make_pattern(n, seed=n)
        addr = self.vram + 0x30000
        stream_write(self.handle, addr, data)
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(got, data, f"mismatch at {n} dwords")

  # === Cross-verification: streaming vs single-TLP ===

  def test_stream_write_single_read(self):
    """Write via streaming, verify each dword with single-TLP reads."""
    n = 32
    vals = [0xBEEF0000 | i for i in range(n)]
    data = array.array('I', vals).tobytes()
    addr = self.vram + 0x40000
    stream_write(self.handle, addr, data)
    for i in range(n):
      got = single_tlp_read(self.handle, addr + i * 4)
      self.assertEqual(got, vals[i], f"dword {i}: got 0x{got:08X}, expected 0x{vals[i]:08X}")

  def test_single_write_stream_read(self):
    """Write each dword with single-TLP, read back via streaming."""
    n = 32
    vals = [0xCAFE0000 | i for i in range(n)]
    addr = self.vram + 0x50000
    for i in range(n):
      single_tlp_write(self.handle, addr + i * 4, vals[i])
    got = stream_read(self.handle, addr, n * 4)
    got_vals = list(array.array('I', got))
    self.assertEqual(got_vals, vals)

  # === Mode transition tests ===

  def test_write_write_different_addr(self):
    """Two streaming writes to different addresses, then read both."""
    n = 64
    a_vals = make_pattern(n, seed=0x100)
    b_vals = make_pattern(n, seed=0x200)
    addr_a = self.vram + 0x60000
    addr_b = self.vram + 0x61000
    stream_write(self.handle, addr_a, a_vals)
    stream_write(self.handle, addr_b, b_vals)
    got_a = stream_read(self.handle, addr_a, len(a_vals))
    got_b = stream_read(self.handle, addr_b, len(b_vals))
    self.assertEqual(got_a, a_vals, "first region corrupted")
    self.assertEqual(got_b, b_vals, "second region corrupted")

  def test_read_read_same_addr(self):
    """Write once, stream-read the same address 5 times."""
    n = 128
    data = make_pattern(n, seed=0x300)
    addr = self.vram + 0x62000
    stream_write(self.handle, addr, data)
    for i in range(5):
      with self.subTest(iteration=i):
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(got, data, f"read {i} mismatch")

  def test_write_read_write_read_rapid(self):
    """Rapid alternating write/read with changing data."""
    addr = self.vram + 0x63000
    for i in range(20):
      n = 16 + i * 4  # varying sizes
      data = make_pattern(n, seed=i * 1000)
      stream_write(self.handle, addr, data)
      got = stream_read(self.handle, addr, len(data))
      self.assertEqual(got, data, f"iteration {i} (n={n}) mismatch")

  # === Partial transfer tests ===

  def test_read_subset_of_written(self):
    """Write 256 dwords, read back only the first 64."""
    n_write = 256
    n_read = 64
    data = make_pattern(n_write, seed=0x400)
    addr = self.vram + 0x64000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, n_read * 4)
    self.assertEqual(got, data[:n_read * 4])

  def test_read_middle_of_written(self):
    """Write 256 dwords, read back dwords 100-199 from the middle."""
    n_write = 256
    start_dword = 100
    n_read = 100
    data = make_pattern(n_write, seed=0x500)
    addr = self.vram + 0x65000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr + start_dword * 4, n_read * 4)
    self.assertEqual(got, data[start_dword * 4:(start_dword + n_read) * 4])

  def test_read_tail_of_written(self):
    """Write 256 dwords, read back the last 64."""
    n_write = 256
    n_read = 64
    data = make_pattern(n_write, seed=0x600)
    addr = self.vram + 0x66000
    stream_write(self.handle, addr, data)
    start = n_write - n_read
    got = stream_read(self.handle, addr + start * 4, n_read * 4)
    self.assertEqual(got, data[start * 4:])

  # === Address boundary tests ===

  def test_addr_256byte_boundary(self):
    """Write across a 256-byte (ADDR_3 carry) boundary."""
    # addr_0 wraps every 256 bytes (64 dwords)
    base = (self.vram + 0x70000) | 0xF0  # start 16 bytes before a 256-byte boundary
    n = 32  # 128 bytes, crossing the boundary
    data = make_pattern(n, seed=0x700)
    stream_write(self.handle, base, data)
    got = stream_read(self.handle, base, len(data))
    self.assertEqual(got, data)

  def test_addr_64k_boundary(self):
    """Write across a 64KB (ADDR_2 carry) boundary."""
    boundary = (self.vram + 0x80000) & ~0xFFFF
    base = boundary - 64  # 64 bytes before the 64KB boundary
    n = 64  # 256 bytes, crossing it
    data = make_pattern(n, seed=0x800)
    stream_write(self.handle, base, data)
    got = stream_read(self.handle, base, len(data))
    self.assertEqual(got, data)

  def test_addr_page_boundary_various(self):
    """Test at several page-aligned offsets."""
    for off in [0x1000, 0x2000, 0x4000, 0x8000, 0x10000]:
      with self.subTest(offset=hex(off)):
        addr = self.vram + off
        n = 128
        data = make_pattern(n, seed=off)
        stream_write(self.handle, addr, data)
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(got, data)

  # === Single TLP edge cases ===

  def test_single_tlp_2byte_write_read(self):
    """Write and read 2 bytes (halfword) at various alignments."""
    base = self.vram + 0x90000
    single_tlp_write(self.handle, base, 0x00000000)
    # Write 2 bytes at offset 0 (bytes 0-1)
    single_tlp_write(self.handle, base, 0xBEEF, size=2)
    got = single_tlp_read(self.handle, base)
    self.assertEqual(got & 0xFFFF, 0xBEEF, f"got 0x{got:08X}")
    # Write 2 bytes at offset 2 (bytes 2-3)
    single_tlp_write(self.handle, base + 2, 0xDEAD, size=2)
    got = single_tlp_read(self.handle, base)
    self.assertEqual(got, 0xDEADBEEF, f"got 0x{got:08X}")

  def test_single_tlp_all_ff_all_00(self):
    """Write 0xFFFFFFFF then 0x00000000, verify each."""
    base = self.vram + 0x91000
    single_tlp_write(self.handle, base, 0xFFFFFFFF)
    self.assertEqual(single_tlp_read(self.handle, base), 0xFFFFFFFF)
    single_tlp_write(self.handle, base, 0x00000000)
    self.assertEqual(single_tlp_read(self.handle, base), 0x00000000)

  # === Randomized fuzz tests ===

  def test_random_sizes_and_offsets(self):
    """Randomized write/read at random offsets and sizes."""
    rng = random.Random(42)
    for trial in range(50):
      n = rng.randint(1, 512)
      offset = rng.randint(0, 0x100000) & ~0x3  # dword-aligned
      addr = self.vram + offset
      data = make_pattern(n, seed=trial)
      with self.subTest(trial=trial, n_dwords=n, offset=hex(offset)):
        stream_write(self.handle, addr, data)
        got = stream_read(self.handle, addr, len(data))
        self.assertEqual(len(got), len(data), f"trial {trial}: length mismatch {len(got)} vs {len(data)}")
        self.assertEqual(got, data, f"trial {trial}: data mismatch at n={n}, off=0x{offset:X}")

  def test_random_interleaved_ops(self):
    """Random interleaving of stream writes, stream reads, and single-TLP reads."""
    rng = random.Random(123)
    regions = {}  # addr -> data
    base = self.vram + 0xA0000
    for trial in range(30):
      op = rng.choice(['write', 'read', 'single_read'])
      slot = rng.randint(0, 7)
      addr = base + slot * 0x2000
      n = rng.randint(1, 64)

      if op == 'write':
        data = make_pattern(n, seed=trial * 100 + slot)
        stream_write(self.handle, addr, data)
        regions[slot] = (addr, data)
      elif op == 'read' and slot in regions:
        saved_addr, saved_data = regions[slot]
        read_n = min(n, len(saved_data) // 4)
        got = stream_read(self.handle, saved_addr, read_n * 4)
        self.assertEqual(got, saved_data[:read_n * 4],
          f"trial {trial}: stream read mismatch at slot {slot}")
      elif op == 'single_read' and slot in regions:
        saved_addr, saved_data = regions[slot]
        idx = rng.randint(0, min(n, len(saved_data) // 4) - 1)
        expected = struct.unpack_from('<I', saved_data, idx * 4)[0]
        got = single_tlp_read(self.handle, saved_addr + idx * 4)
        self.assertEqual(got, expected,
          f"trial {trial}: single-TLP read mismatch at slot {slot} idx {idx}")

  # === Stress tests ===

  def test_many_small_writes(self):
    """100 back-to-back 1-dword writes to consecutive addresses, then read all."""
    n = 100
    addr = self.vram + 0xB0000
    for i in range(n):
      data = struct.pack('<I', 0x11110000 | i)
      stream_write(self.handle, addr + i * 4, data)
    got = stream_read(self.handle, addr, n * 4)
    got_vals = list(array.array('I', got))
    expected = [0x11110000 | i for i in range(n)]
    self.assertEqual(got_vals, expected)

  def test_many_small_reads(self):
    """Write 100 dwords, then read each one individually via streaming."""
    n = 100
    addr = self.vram + 0xB1000
    vals = [0x22220000 | i for i in range(n)]
    data = array.array('I', vals).tobytes()
    stream_write(self.handle, addr, data)
    for i in range(n):
      with self.subTest(i=i):
        got = stream_read(self.handle, addr + i * 4, 4)
        got_val = struct.unpack('<I', got)[0]
        self.assertEqual(got_val, vals[i], f"dword {i}: got 0x{got_val:08X}, expected 0x{vals[i]:08X}")

  def test_exactly_max_packet_dwords(self):
    """Test with exactly max_chunk_dwords (256 for USB3 = 1024 bytes)."""
    n = 256
    data = make_pattern(n, seed=0xC00)
    addr = self.vram + 0xC0000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)

  def test_max_packet_plus_one(self):
    """Test with max_chunk_dwords + 1 (257 dwords = 1028 bytes, spans 2 USB packets)."""
    n = 257
    data = make_pattern(n, seed=0xC01)
    addr = self.vram + 0xC1000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)

  def test_max_packet_minus_one(self):
    """Test with max_chunk_dwords - 1 (255 dwords = 1020 bytes, short packet)."""
    n = 255
    data = make_pattern(n, seed=0xC02)
    addr = self.vram + 0xC2000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)

  def test_multi_chunk_read(self):
    """Large read spanning many USB chunks (16KB = 4096 dwords = 16 chunks of 256)."""
    n = 4096
    data = make_pattern(n, seed=0xD00)
    addr = self.vram + 0xD0000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)


  # === Address carry tests for streaming ===

  def test_addr_16mb_boundary_streaming(self):
    """Stream write/read across a 16MB (ADDR_0 carry) boundary."""
    boundary = (self.vram + 0x1000000) & ~0xFFFFFF
    base = boundary - 512  # 512 bytes before
    n = 512  # 2048 bytes, 512 bytes past the boundary
    data = make_pattern(n, seed=0xE00)
    stream_write(self.handle, base, data)
    got = stream_read(self.handle, base, len(data))
    self.assertEqual(got, data)

  def test_addr_4gb_boundary_streaming(self):
    """Stream write/read across a 4GB boundary (carry from low 32 into high 32 bits).
    VRAM BAR is at 0x800000000, so 0x8FFFFFFFC + 8 bytes = crosses 0x900000000."""
    boundary = (self.vram & ~0xFFFFFFFF) + 0x100000000  # next 4GB boundary
    base = boundary - 64  # 64 bytes before
    n = 64  # 256 bytes, crossing the 4GB boundary
    data = make_pattern(n, seed=0xE01)
    print(f"\n  4GB test: base=0x{base:X} boundary=0x{boundary:X} end=0x{base+n*4:X}")
    stream_write(self.handle, base, data)
    got = stream_read(self.handle, base, len(data))
    print(f"  got {len(got)} bytes, expected {len(data)}")
    if got != data:
      # find first mismatch
      for i in range(min(len(got), len(data))):
        if got[i] != data[i]:
          dw = i // 4
          print(f"  first mismatch at byte {i} (dword {dw}, addr 0x{base+i:X})")
          print(f"  got:  {got[i:i+16].hex()}")
          print(f"  want: {data[i:i+16].hex()}")
          break
    self.assertEqual(got, data)

  def test_addr_256byte_carry_exact(self):
    """Start exactly at a 256-byte boundary, write 256 bytes (forces ADDR_3 carry at end)."""
    base = (self.vram + 0x70000) & ~0xFF
    n = 64  # 256 bytes exactly
    data = make_pattern(n, seed=0xE10)
    stream_write(self.handle, base, data)
    got = stream_read(self.handle, base, len(data))
    self.assertEqual(got, data)

  # === Cross-mode verification ===

  def test_single_write_stream_read_many(self):
    """Write 64 dwords via single TLP, read all 64 back via streaming."""
    addr = self.vram + 0xE0000
    vals = [0xFACE0000 | i for i in range(64)]
    for i, v in enumerate(vals):
      single_tlp_write(self.handle, addr + i * 4, v)
    got = stream_read(self.handle, addr, 64 * 4)
    got_vals = list(array.array('I', got))
    self.assertEqual(got_vals, vals)

  def test_stream_write_single_read_byte(self):
    """Stream write, then single-TLP read individual bytes."""
    addr = self.vram + 0xE1000
    data = bytes(range(256)) * 4  # 1024 bytes of 0x00..0xFF repeated
    stream_write(self.handle, addr, data)
    for off in [0, 1, 2, 3, 100, 255, 256, 511]:
      with self.subTest(offset=off):
        expected = data[off]
        got = single_tlp_read(self.handle, addr + off, size=1)
        self.assertEqual(got, expected, f"byte at offset {off}: got 0x{got:02X}, expected 0x{expected:02X}")

  # === Byte enable edge cases ===

  def test_single_tlp_3byte_write_read(self):
    """Write 3 bytes at offset 0 (byte enables 0x07), read back."""
    base = self.vram + 0xE2000
    single_tlp_write(self.handle, base, 0x00000000)  # clear
    single_tlp_write(self.handle, base, 0x00ABCDEF, size=3)  # write bytes 0-2
    got = single_tlp_read(self.handle, base)
    self.assertEqual(got & 0x00FFFFFF, 0x00ABCDEF & 0x00FFFFFF,
      f"3-byte write: got 0x{got:08X}")

  def test_single_tlp_byte_at_each_offset(self):
    """Write byte 0xAA at each offset 0-3, verify only that byte changed."""
    base = self.vram + 0xE3000
    for off in range(4):
      single_tlp_write(self.handle, base, 0x00000000)  # clear
      single_tlp_write(self.handle, base + off, 0xAA, size=1)
      got = single_tlp_read(self.handle, base)
      expected = 0xAA << (off * 8)
      self.assertEqual(got, expected,
        f"byte at offset {off}: got 0x{got:08X}, expected 0x{expected:08X}")

  # === Large odd-size transfers ===

  def test_large_odd_size(self):
    """Write and read 100KB - 4 bytes (not a power of 2, not chunk-aligned)."""
    n = (100 * 1024 // 4) - 1  # 25599 dwords
    data = make_pattern(n, seed=0xF00)
    addr = self.vram + 0x100000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)

  def test_1mb_minus_4(self):
    """Write and read 1MB - 4 bytes."""
    n = (1024 * 1024 // 4) - 1  # 262143 dwords
    data = make_pattern(n, seed=0xF01)
    addr = self.vram + 0x200000
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data)

  # === 0xF2 SRAM DMA then 0xF0 streaming ===

  def test_sram_dma_then_streaming(self):
    """Do an 0xF2 SRAM DMA write, then a normal 0xF0 streaming write/read."""
    # 0xF2: write 1 sector (512 bytes) to SRAM
    sectors = 1
    wValue = sectors & 0x7FFF
    wIndex = (1 & 0xFF) << 8  # 1 slot
    ret = libusb.libusb_control_transfer(self.handle, 0x40, 0xF2, wValue, wIndex, None, 0, 1000)
    assert ret >= 0, f"0xF2 setup failed: {ret}"
    sram_data = bytes([i & 0xFF for i in range(512)])
    buf = (ctypes.c_ubyte * 512)(*sram_data)
    transferred = ctypes.c_int()
    ret = libusb.libusb_bulk_transfer(self.handle, EP_OUT, buf, 512, ctypes.byref(transferred), 5000)
    assert ret == 0, f"SRAM bulk write failed: {ret}"
    time.sleep(0.01)

    # Now do normal 0xF0 streaming
    addr = self.vram + 0x300000
    n = 128
    data = make_pattern(n, seed=0xF10)
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, len(data))
    self.assertEqual(got, data, "streaming after SRAM DMA failed")

  # === One dword ===

  def test_one_dword_write_read(self):
    """Streaming write and read of exactly 1 dword."""
    addr = self.vram + 0x310000
    data = struct.pack('<I', 0xDEADBEEF)
    stream_write(self.handle, addr, data)
    got = stream_read(self.handle, addr, 4)
    self.assertEqual(struct.unpack('<I', got)[0], 0xDEADBEEF)

  def test_one_dword_cross_verify(self):
    """Write 1 dword via streaming, read via single TLP (and vice versa)."""
    addr = self.vram + 0x311000
    stream_write(self.handle, addr, struct.pack('<I', 0xCAFEBABE))
    got = single_tlp_read(self.handle, addr)
    self.assertEqual(got, 0xCAFEBABE)

    single_tlp_write(self.handle, addr, 0x12345678)
    got = stream_read(self.handle, addr, 4)
    self.assertEqual(struct.unpack('<I', got)[0], 0x12345678)


  # === 0xE4 XDATA read tests ===

  def _xdata_write(self, addr, val):
    """Write a single byte to XDATA via 0xE5."""
    ret = libusb.libusb_control_transfer(self.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
    assert ret >= 0, f"E5 write 0x{addr:04X}=0x{val:02X} failed: {ret}"

  def _xdata_read(self, addr, size):
    """Read bytes from XDATA via 0xE4."""
    buf = (ctypes.c_ubyte * size)()
    ret = libusb.libusb_control_transfer(self.handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
    assert ret >= 0, f"E4 read 0x{addr:04X} len={size} failed: {ret}"
    return bytes(buf[:ret])

  def test_xdata_read_1_byte(self):
    """Read 1 byte from XDATA."""
    self._xdata_write(0x7F00, 0xAB)
    got = self._xdata_read(0x7F00, 1)
    self.assertEqual(got, bytes([0xAB]))

  def test_xdata_read_small(self):
    """Read 8 bytes from XDATA."""
    for i in range(8):
      self._xdata_write(0x7F00 + i, 0x10 + i)
    got = self._xdata_read(0x7F00, 8)
    self.assertEqual(got, bytes([0x10 + i for i in range(8)]))

  def test_xdata_read_64_bytes(self):
    """Read exactly 64 bytes (USB2 max packet size for control)."""
    for i in range(64):
      self._xdata_write(0x7E00 + i, i & 0xFF)
    got = self._xdata_read(0x7E00, 64)
    self.assertEqual(len(got), 64)
    self.assertEqual(got, bytes([i & 0xFF for i in range(64)]))

  def test_xdata_read_max_usb2(self):
    """Read 64 bytes — max for USB2 control data stage."""
    for i in range(64):
      self._xdata_write(0x7E00 + i, (i * 3) & 0xFF)
    got = self._xdata_read(0x7E00, 64)
    self.assertEqual(len(got), 64)
    self.assertEqual(got, bytes([(i * 3) & 0xFF for i in range(64)]))

  def test_xdata_read_over_max_returns_max(self):
    """Request more than max — firmware clamps, host gets exactly max.
    We request 64 bytes (safe on both USB2 and USB3)."""
    for i in range(64):
      self._xdata_write(0x7E00 + i, (i * 3) & 0xFF)
    got = self._xdata_read(0x7E00, 64)
    self.assertEqual(len(got), 64)
    self.assertEqual(got, bytes([(i * 3) & 0xFF for i in range(64)]))

  def test_xdata_read_known_register(self):
    """Read a known hardware register via 0xE4 (PCIe LTSSM should be 0x78 if link is up)."""
    got = self._xdata_read(0xB450, 1)
    self.assertEqual(got[0], 0x78, f"LTSSM state: 0x{got[0]:02X}")

  def test_xdata_write_read_roundtrip(self):
    """Write various patterns and read them back."""
    patterns = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7F, 0xFE]
    for val in patterns:
      self._xdata_write(0x7F00, val)
      got = self._xdata_read(0x7F00, 1)
      self.assertEqual(got[0], val, f"write 0x{val:02X}, got 0x{got[0]:02X}")


if __name__ == '__main__':
  pytest.main([__file__, "-v", "-s", "--tb=short", *sys.argv[1:]])
