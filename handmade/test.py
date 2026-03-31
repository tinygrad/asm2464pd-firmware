#!/usr/bin/env python3
"""
Tests for ASM2464PD handmade firmware.
Uses USB3 from tinygrad to open the device, then vendor control transfers for register access.
"""
import ctypes, struct, unittest, random, sys, pytest
from hexdump import hexdump
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001

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

class TestDevice(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.dev = USB3(VID, PID, 0x81, 0x83, 0x02, 0x04, use_bot=True)
    #BLK = 0x40
    #regs = b''.join(ctrl_read(cls.dev, i, BLK) for i in range(0x9000, 0x9400, BLK))
    #print("\n"+hexdump(regs, 'return'), file=sys.stderr)

  def test_device_opens(self):
    assert self.dev.handle is not None

  def test_read_regs(self):
    link = ctrl_read(self.dev, 0x9100)[0]          # REG_USB_LINK_STATUS
    assert link in (0x00, 0x01, 0x02, 0x03), f"unexpected link 0x{link:02X}"
    mode = ctrl_read(self.dev, 0xCC30)[0]          # REG_CPU_MODE
    assert mode in (0x00, 0x01), f"unexpected cpu mode 0x{mode:02X}"
    power = ctrl_read(self.dev, 0x92C0)[0]         # REG_POWER_ENABLE
    assert power & 0x80, f"POWER_ENABLE bit 7 not set: 0x{power:02X}"

  def test_control_host_to_device(self):
    buf = b"\xaa\xbb\xcc\xdd"
    cbuf = (ctypes.c_ubyte * len(buf))(*buf)
    ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xF1, 0, 0, cbuf, len(buf), 1000)
    assert ret >= 0, f"control host to device failed: {ret}"
    self.assertEqual(ctrl_read(self.dev, 0x9E00, size=4), buf)

  def test_write_read_control(self):
    ctrl_write(self.dev, 0xF000, 0xAB)
    assert ctrl_read(self.dev, 0xF000)[0] == 0xAB
    ctrl_write(self.dev, 0xF000, 0x00)
    assert ctrl_read(self.dev, 0xF000)[0] == 0x00

  def test_bulk_out(self, check=True):
    self.dev._tag += 1
    test = random.randint(0, 0x7FFF) | 0xC0DE0000
    cbw = struct.pack('<IIIBBB', test, self.dev._tag, 0, 0x80, 0, 16) + b'\xE8' + b'\x00' * 15
    self.dev._bulk_out(0x02, cbw)
    # CBW lands at 0x7000 — check test matches
    if check: self.assertEqual(struct.unpack('<I', ctrl_read(self.dev, 0x7000, size=4))[0], test)

  def test_bulk_in(self, check=True):
    # Write test data to D800 — firmware's BULK_IN_START handler will send it
    test = random.randint(0, 0x7fffffff)
    if check:
      ctrl_write(self.dev, 0xD800, (test>>0)&0xFF)
      ctrl_write(self.dev, 0xD801, (test>>8)&0xFF)
      ctrl_write(self.dev, 0xD802, (test>>16)&0xFF)
      ctrl_write(self.dev, 0xD803, (test>>24)&0xFF)
    # Host requests bulk IN — firmware handles via 0x04 interrupt
    csw = self.dev._bulk_in(0x81, 13)
    sig, tag, residue, status = struct.unpack('<IIIB', csw)
    if check: self.assertEqual(sig, test)

  @unittest.skip("no fuzzing")
  def test_fuzz_bulk_in_out(self):
    for _ in range(5):
      out = bool(random.randint(0,1))
      check = bool(random.randint(0,1))
      if out: self.test_bulk_out(check=check)
      else: self.test_bulk_in(check=check)

if __name__ == '__main__':
  pytest.main([__file__, "-v", "-s", *sys.argv[1:]])
