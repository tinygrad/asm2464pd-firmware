import importlib.util
import random
import sys
import types
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
E4_FLASH = PROJECT_ROOT / "handmade" / "e4_flash.py"


def load_e4_flash(monkeypatch):
  """Load e4_flash without tinygrad or physical USB hardware."""
  tinygrad = types.ModuleType("tinygrad")
  runtime = types.ModuleType("tinygrad.runtime")
  autogen = types.ModuleType("tinygrad.runtime.autogen")
  autogen.libusb = types.SimpleNamespace()
  pcie_probe = types.ModuleType("pcie_probe")
  pcie_probe.xdata_read = lambda *_args, **_kwargs: b""
  pcie_probe.xdata_write = lambda *_args, **_kwargs: None

  monkeypatch.setitem(sys.modules, "tinygrad", tinygrad)
  monkeypatch.setitem(sys.modules, "tinygrad.runtime", runtime)
  monkeypatch.setitem(sys.modules, "tinygrad.runtime.autogen", autogen)
  monkeypatch.setitem(sys.modules, "pcie_probe", pcie_probe)

  spec = importlib.util.spec_from_file_location("e4_flash_under_test", E4_FLASH)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_flash_read_splits_256_byte_read_into_usb2_packets(monkeypatch):
  """A 256-byte verification read must use four 64-byte E4 transfers."""
  e4_flash = load_e4_flash(monkeypatch)
  transactions = []
  reads = []

  monkeypatch.setattr(
    e4_flash,
    "flash_transaction",
    lambda _handle, **kwargs: transactions.append(kwargs),
  )

  def fake_xdata_read(_handle, addr, size):
    reads.append((addr, size))
    return bytes((addr + offset) & 0xFF for offset in range(size))

  monkeypatch.setattr(e4_flash, "xdata_read", fake_xdata_read)

  result = e4_flash.flash_read(object(), 0x1234, 256)

  assert transactions == [{
    "cmd": 0x03,
    "addr": 0x1234,
    "data_len": 4096,
    "addr_len": 0x07,
    "mode": 0x00,
  }]
  assert reads == [
    (0x7000, 64),
    (0x7040, 64),
    (0x7080, 64),
    (0x70C0, 64),
  ]
  assert result == bytes(range(256))


def test_flash_verify_keeps_256_byte_comparison_windows(monkeypatch):
  """Transport packet sizing must not reduce the verification window."""
  e4_flash = load_e4_flash(monkeypatch)
  firmware = bytes(index & 0xFF for index in range(600))
  reads = []

  def fake_flash_read(_handle, addr, size):
    reads.append((addr, size))
    offset = addr - 0x100
    return firmware[offset:offset + size]

  monkeypatch.setattr(e4_flash, "flash_read", fake_flash_read)

  assert e4_flash.flash_verify(object(), 0x100, firmware) == 0
  assert reads == [
    (0x100, 256),
    (0x200, 256),
    (0x300, 88),
  ]


def test_flash_read_rejects_short_usb2_control_read(monkeypatch):
  e4_flash = load_e4_flash(monkeypatch)
  monkeypatch.setattr(e4_flash, "flash_transaction", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(e4_flash, "xdata_read", lambda _handle, _addr, size: bytes(size - 1))

  with pytest.raises(IOError, match=r"Short E4 read at 0x7000: 63/64 bytes"):
    e4_flash.flash_read(object(), 0, 64)


def test_flash_clear_dma_status_resets_all_activity_registers(monkeypatch):
  e4_flash = load_e4_flash(monkeypatch)
  writes = []
  monkeypatch.setattr(e4_flash, "xdata_write", lambda _handle, addr, value: writes.append((addr, value)))

  e4_flash.flash_clear_dma_status(object())

  assert writes == [
    (0xC8B8, 0),
    (0xC8B6, 0),
    (0xC8D6, 0),
    (0xC8D8, 0),
  ]


def test_flash_poll_dma_idle_waits_for_both_activity_bits(monkeypatch):
  e4_flash = load_e4_flash(monkeypatch)
  trigger = iter((0x01, 0x00, 0x00))
  channel = iter((0x80, 0x00))

  def fake_xdata_read(_handle, addr, size):
    assert size == 1
    if addr == 0xC8B8:
      return bytes([next(trigger)])
    if addr == 0xC8B6:
      return bytes([next(channel)])
    raise AssertionError(f"Unexpected DMA register: 0x{addr:04X}")

  monkeypatch.setattr(e4_flash, "xdata_read", fake_xdata_read)

  e4_flash.flash_poll_dma_idle(object())


def test_flash_transaction_waits_for_read_dma_and_clears_mode_bits(monkeypatch):
  e4_flash = load_e4_flash(monkeypatch)
  events = []
  registers = {}
  mode_writes = []

  monkeypatch.setattr(e4_flash, "flash_poll_busy", lambda _handle: events.append("flash_busy"))
  monkeypatch.setattr(e4_flash, "flash_clear_dma_status", lambda _handle: events.append("dma_clear"), raising=False)
  monkeypatch.setattr(e4_flash, "flash_poll_dma_idle", lambda _handle: events.append("dma_idle"), raising=False)

  def fake_xdata_write(_handle, addr, value):
    registers[addr] = value
    if addr == 0xC8AD:
      mode_writes.append(value)

  monkeypatch.setattr(e4_flash, "xdata_write", fake_xdata_write)
  monkeypatch.setattr(e4_flash, "xdata_read", lambda _handle, addr, _size: bytes([registers.get(addr, 0)]))

  e4_flash.flash_transaction(
    object(),
    cmd=0x03,
    addr=0x100,
    data_len=4096,
    addr_len=0x07,
    mode=0xF0,
  )

  assert events == [
    "flash_busy",
    "dma_clear",
    "dma_idle",
    "flash_busy",
    "dma_idle",
    "dma_clear",
  ]
  assert mode_writes == [0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00]


def test_flash_write_uses_usb2_safe_chunks_for_config_and_firmware(monkeypatch):
  """Config restoration and firmware writes must stay within 64 bytes."""
  e4_flash = load_e4_flash(monkeypatch)
  config = bytes(range(256))
  firmware = bytes(range(130))
  erased = []
  bulk_writes = []
  page_programs = []

  monkeypatch.setattr(e4_flash, "flash_read", lambda _handle, addr, size: config if (addr, size) == (0, 256) else None)
  monkeypatch.setattr(e4_flash, "flash_block_erase", lambda _handle, addr: erased.append(addr))
  monkeypatch.setattr(e4_flash, "bulk_out", lambda _handle, data: bulk_writes.append(bytes(data)))
  monkeypatch.setattr(
    e4_flash,
    "flash_page_program",
    lambda _handle, addr, size: page_programs.append((addr, size)),
  )

  e4_flash.flash_write(object(), 0x100, firmware)

  assert erased == [0]
  assert [len(chunk) for chunk in bulk_writes] == [64, 64, 64, 64, 64, 64, 4]
  assert bulk_writes[-1] == firmware[-2:] + b"\x00\x00"
  assert page_programs == [
    (0x000, 64),
    (0x040, 64),
    (0x080, 64),
    (0x0C0, 64),
    (0x100, 64),
    (0x140, 64),
    (0x180, 2),
  ]


def test_flash_test_uses_usb2_safe_chunks(monkeypatch):
  """The destructive flash self-test must use the same safe write size."""
  e4_flash = load_e4_flash(monkeypatch)
  rng = random.Random(42)
  pattern = bytes(rng.randint(0, 255) for _ in range(4096))
  erased = []
  bulk_sizes = []
  page_programs = []

  def fake_flash_read(_handle, _addr, size):
    if size == 16:
      return b"\xFF" * size
    if size == len(pattern):
      return pattern
    raise AssertionError(f"Unexpected flash read size: {size}")

  monkeypatch.setattr(e4_flash, "flash_read", fake_flash_read)
  monkeypatch.setattr(e4_flash, "flash_block_erase", lambda _handle, addr: erased.append(addr))
  monkeypatch.setattr(e4_flash, "bulk_out", lambda _handle, data: bulk_sizes.append(len(data)))
  monkeypatch.setattr(
    e4_flash,
    "flash_page_program",
    lambda _handle, addr, size: page_programs.append((addr, size)),
  )

  assert e4_flash.flash_test(object()) == 0
  assert erased == [0x10000, 0x10000]
  assert bulk_sizes == [64] * 64
  assert page_programs == [(0x10000 + offset, 64) for offset in range(0, 4096, 64)]


def test_flash_test_erases_scratch_block_after_write_failure(monkeypatch):
  """A failed self-test must still erase its scratch flash block."""
  e4_flash = load_e4_flash(monkeypatch)
  erased = []

  monkeypatch.setattr(e4_flash, "flash_read", lambda _handle, _addr, size: b"\xFF" * size)
  monkeypatch.setattr(e4_flash, "flash_block_erase", lambda _handle, addr: erased.append(addr))
  monkeypatch.setattr(e4_flash, "bulk_out", lambda *_args, **_kwargs: None)

  def fail_page_program(*_args, **_kwargs):
    raise IOError("simulated page program failure")

  monkeypatch.setattr(e4_flash, "flash_page_program", fail_page_program)

  with pytest.raises(IOError, match="simulated page program failure"):
    e4_flash.flash_test(object())

  assert erased == [0x10000, 0x10000]
