#!/usr/bin/env python3
"""Physically qualify firmware-managed chained SRAM writes without booting a GPU."""

import ctypes
import re
import time
from pathlib import Path

from tinygrad.runtime.autogen import libusb

from pcie_probe import usb_close, usb_open, xdata_read


EP_OUT = 0x02
RING_SLOTS = (18, 25, 11, 11)


def control_write(handle, request, value=0, index=0):
  ret = libusb.libusb_control_transfer(handle, 0x40, request, value, index, None, 0, 1000)
  assert ret == 0, f"control write 0x{request:02x} returned {ret}"


def bulk_write(handle, payload):
  buf = (ctypes.c_ubyte * len(payload)).from_buffer_copy(payload)
  transferred = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, EP_OUT, buf, len(payload), ctypes.byref(transferred), 5000)
  assert ret == 0, f"bulk OUT failed: {ret}"
  assert transferred.value == len(payload), f"short bulk OUT: {transferred.value}/{len(payload)}"


def stream_symbols():
  listing = Path(__file__).parent.parent / "handmade" / "build" / "obj" / "main.rst"
  text = listing.read_text()
  names = ("remaining", "first_slot", "slot", "slots", "dma_active")
  symbols = {}
  for name in names:
    match = re.search(rf"^\s*([0-9A-F]{{6}})\s+\d+\s+_sram_stream_{name}:\s*$", text, re.MULTILINE)
    assert match is not None, f"missing sram_stream_{name} in {listing}"
    symbols[name] = int(match.group(1), 16)
  return symbols


def stream_state(handle, symbols):
  return {
    "remaining": int.from_bytes(xdata_read(handle, symbols["remaining"], 2), "little"),
    "first_slot": xdata_read(handle, symbols["first_slot"], 1)[0],
    "slot": xdata_read(handle, symbols["slot"], 1)[0],
    "slots": xdata_read(handle, symbols["slots"], 1)[0],
    "dma_active": xdata_read(handle, symbols["dma_active"], 1)[0],
  }


def wait_for_state(handle, symbols, **expected):
  deadline = time.monotonic() + 1.0
  while True:
    state = stream_state(handle, symbols)
    if all(state[key] == value for key, value in expected.items()):
      return state
    if time.monotonic() >= deadline:
      raise AssertionError(f"stream state {state}, expected {expected}")
    time.sleep(0.001)


def pattern(seed, size):
  block = bytes((seed + i * 17) & 0xFF for i in range(256))
  return (block * ((size + len(block) - 1) // len(block)))[:size]


def arm_stream(handle, count, start_slot, slots):
  control_write(handle, 0xF5, value=count, index=start_slot | (slots << 8))


def main():
  symbols = stream_symbols()
  handle, context = usb_open()
  try:
    # A full-window stream proves that a completed transfer is rearmed and that
    # the following transfer replaces SRAM rather than falling into EP scratch.
    full_size = 32 * 0x4000
    arm_stream(handle, count=2, start_slot=0, slots=32)
    for remaining, seed in ((1, 0x31), (0, 0xA7)):
      payload = pattern(seed, full_size)
      bulk_write(handle, payload)
      state = wait_for_state(handle, symbols, remaining=remaining, slot=0, slots=32, dma_active=int(remaining != 0))
      print(f"full-window transfer complete: {state}")
    assert xdata_read(handle, 0xF000, 64) == payload[:64]

    # Match the exact slot geometry used while SEC2 consumes the GSP image.
    ring_size = 7 * 0x4000
    arm_stream(handle, count=4, start_slot=11, slots=7)
    for index, expected_slot in enumerate(RING_SLOTS):
      bulk_write(handle, pattern(0x40 + index, ring_size))
      state = wait_for_state(handle, symbols, remaining=3-index, slot=expected_slot, slots=7, dma_active=int(index != 3))
      print(f"ring transfer {index + 1}/4 complete: {state}")

    started = time.monotonic()
    arm_stream(handle, count=4, start_slot=11, slots=7)
    period = 0.0014 / 3
    for index in range(4):
      deadline = started + index * period
      while time.monotonic() < deadline:
        pass
      bulk_write(handle, pattern(0x60 + index, ring_size))
    elapsed = time.monotonic() - started
    state = wait_for_state(handle, symbols, remaining=0, slot=11, slots=7, dma_active=0)
    print(f"production-paced ring complete in {elapsed * 1000:.3f} ms: {state}")

    print("PASS: chained SRAM data replacement and GSP ring rotation physically verified")
  finally:
    usb_close(handle, context)


if __name__ == "__main__":
  main()
