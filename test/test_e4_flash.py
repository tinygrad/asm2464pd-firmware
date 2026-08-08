import importlib.util
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
