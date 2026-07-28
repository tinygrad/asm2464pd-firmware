import importlib.util
from pathlib import Path
import struct
import zlib
import pytest


spec = importlib.util.spec_from_file_location(
  "bootflash", Path(__file__).parents[1] / "handmade" / "flash.py")
bootflash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bootflash)


def image(body=b"\x02\x30\x30" + bytes(range(64))):
  version = b"test-CLEAN"
  pre = b"A24F" + version + bytes(23-len(version))
  pre += bytes([bootflash.BOOT_ABI_VERSION]) + struct.pack("<I", len(body))
  return pre + struct.pack("<I", zlib.crc32(pre + body)) + bytes(28) + body


def field(blob, offset, value):
  return blob[:offset] + value + blob[offset+len(value):]


BAD_IMAGES = [
  (lambda x: x[:3], "truncated"),
  (lambda x: field(x, 0, b"BAD!"), "magic"),
  (lambda x: field(x, 0x1B, b"\x02"), "ABI"),
  (lambda x: field(x, 0x1C, struct.pack("<I", 0)), "body length"),
  (lambda x: field(x, 0x1C, struct.pack("<I", bootflash.USERFW_BODY_LIMIT+1)), "body length"),
  (lambda x: x[:-1], "length mismatch"),
  (lambda x: field(x, 0x24, b"\x01"), "reserved"),
  (lambda x: field(x, 0x20, bytes([x[0x20] ^ 1])), "CRC"),
]


def test_image_validation():
  good = image()
  assert bootflash.validate_image(good) == len(good) - 0x40
  for mutate, error in BAD_IMAGES:
    with pytest.raises(ValueError, match=error):
      bootflash.validate_image(mutate(good))


def info(**changes):
  values = dict(magic=b"A24D", protocol=1, abi=1, flags=7, reserved=0,
                page=512, sector=0x1000, start=0x4000, end=0x11040)
  values.update(changes)
  return bootflash.DFU_INFO.pack(*values.values())


@pytest.mark.parametrize("changes,error", [
  ({}, None),
  ({"protocol": 2}, "protocol"),
  ({"abi": 2}, "ABI"),
  ({"page": 256}, "geometry"),
  ({"sector": 8192}, "geometry"),
  ({"start": 0x5000}, "geometry"),
])
def test_dfu_contract(monkeypatch, changes, error):
  monkeypatch.setattr(bootflash, "_ctl", lambda *args, **kwargs: info(**changes))
  if error:
    with pytest.raises(RuntimeError, match=error):
      bootflash.cmd_get_info(object())
  else:
    assert bootflash.cmd_get_info(object()) == 512


def test_write_retry_readdresses_identical_data(monkeypatch):
  calls, writes = [], 0
  def ctl(handle, bm, request, value, index, data, length, what, timeout=0):
    nonlocal writes
    calls.append((request, bytes(data or b"")))
    if request == 0xB2:
      writes += 1
      if writes == 1: raise IOError("lost status")
  monkeypatch.setattr(bootflash, "_ctl", ctl)
  bootflash.cmd_write(object(), 0x4567, b"abc", retries=2)
  assert [request for request, _ in calls] == [0xB1, 0xB2, 0xB6, 0xB1, 0xB2]
  assert [data for request, data in calls if request == 0xB2] == [b"abc", b"abc"]
