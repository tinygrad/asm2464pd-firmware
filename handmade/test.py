#!/usr/bin/env python3
"""
Tests for ASM2464PD handmade firmware.
Uses USB3 from tinygrad to open the device, then vendor control transfers for register access.
"""
import ctypes
import pytest
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001

@pytest.fixture(scope="session")
def dev(): return USB3(VID, PID, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def ctrl_read(dev, addr, size=1):
  """Read bytes from XDATA via vendor control IN (0xE4)."""
  buf = (ctypes.c_ubyte * size)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
  assert ret >= 0, f"ctrl_read(0x{addr:04X}) failed: {ret}"
  return bytes(buf[:ret])

def ctrl_write(dev, addr, val):
  """Write byte to XDATA via vendor control OUT (0xE5)."""
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0, f"ctrl_write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

class TestDevice:
  def test_device_opens(self, dev):
    assert dev.handle is not None

  def test_read_regs(self, dev):
    link = ctrl_read(dev, 0x9100)[0]          # REG_USB_LINK_STATUS
    assert link in (0x00, 0x01, 0x03), f"unexpected link 0x{link:02X}"
    mode = ctrl_read(dev, 0xCC30)[0]          # REG_CPU_MODE
    assert mode in (0x00, 0x01), f"unexpected cpu mode 0x{mode:02X}"
    power = ctrl_read(dev, 0x92C0)[0]         # REG_POWER_ENABLE
    assert power & 0x80, f"POWER_ENABLE bit 7 not set: 0x{power:02X}"

  def test_write_read_control(self, dev):
    ctrl_write(dev, 0xF000, 0xAB)
    assert ctrl_read(dev, 0xF000)[0] == 0xAB
    ctrl_write(dev, 0xF000, 0x00)
    assert ctrl_read(dev, 0xF000)[0] == 0x00

if __name__ == "__main__":
  pytest.main([__file__, "-v"])
