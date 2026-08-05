#!/usr/bin/env python3
"""Toggle ATX PSU power via the custom USB adapter's GPIO 15 (ATX_EN).
Also controls PCIe power (3.3V/12V rails) via vendor request 0xF3.

Usage:
  python3 atx_power.py on          # power on ATX PSU + PCIe
  python3 atx_power.py off         # power off PCIe + ATX PSU
  python3 atx_power.py toggle      # toggle current state
  python3 atx_power.py status      # read current GPIO 15 state
"""

import ctypes, sys, time
from tinygrad.runtime.autogen import libusb

VID, PID = 0x3801, 0x0001

# GPIO_ATX_EN = 15, control register at 0xC620 + 15 = 0xC62F
GPIO_ATX_EN_REG = 0xC62F
GPIO_INPUT  = 0x00
GPIO_LOW    = 0x02  # drive low  = ATX off
GPIO_HIGH   = 0x03  # drive high = ATX on

# GPIO input register: 0xC650 + (15 >> 3) = 0xC651, bit (15 & 7) = bit 7
GPIO_ATX_INPUT_REG = 0xC650 + (15 >> 3)
GPIO_ATX_INPUT_BIT = 1 << (15 & 7)

# PCIe power is controlled via vendor request 0xF3 (wValue=1 on, 0 off)
PCIE_POWER_ON  = 1
PCIE_POWER_OFF = 0


def open_device():
  ctx = ctypes.POINTER(libusb.libusb_context)()
  ret = libusb.libusb_init(ctypes.byref(ctx))
  if ret != 0:
    raise RuntimeError(f"libusb_init failed: {ret}")
  handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
  if not handle:
    libusb.libusb_exit(ctx)
    raise RuntimeError(f"device {VID:04X}:{PID:04X} not found")
  libusb.libusb_claim_interface(handle, 0)
  return handle, ctx


def close_device(handle, ctx):
  libusb.libusb_release_interface(handle, 0)
  libusb.libusb_close(handle)
  libusb.libusb_exit(ctx)


def xdata_read(handle, addr, size=1):
  buf = (ctypes.c_ubyte * size)()
  ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, addr, 0, buf, size, 1000)
  if ret < 0:
    raise RuntimeError(f"E4 read 0x{addr:04X} failed: {ret}")
  return bytes(buf[:ret])


def xdata_write(handle, addr, val):
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  if ret < 0:
    raise RuntimeError(f"E5 write 0x{addr:04X}=0x{val:02X} failed: {ret}")


def pcie_power(handle, enabled):
  """Toggle PCIe power rails via vendor request 0xF3."""
  val = PCIE_POWER_ON if enabled else PCIE_POWER_OFF
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF3, val, 0, None, 0, 5000)
  if ret < 0:
    state = 'on' if enabled else 'off'
    raise RuntimeError(f"F3 PCIe power {state} failed: {ret}")


def atx_on(handle):
  xdata_write(handle, GPIO_ATX_EN_REG, GPIO_HIGH)
  time.sleep(0.5)  # let ATX rails stabilize before enabling PCIe
  pcie_power(handle, enabled=True)
  print("ATX PSU + PCIe powered on (GPIO 15 -> HIGH, PCIe power ON)")


def atx_off(handle):
  pcie_power(handle, enabled=False)
  xdata_write(handle, GPIO_ATX_EN_REG, GPIO_LOW)
  print("PCIe + ATX PSU powered off (PCIe power OFF, GPIO 15 -> LOW)")


def atx_status(handle):
  ctrl = xdata_read(handle, GPIO_ATX_EN_REG, 1)[0]
  if ctrl == GPIO_HIGH:
    print("ATX PSU: ON (GPIO 15 = HIGH)")
    return True
  elif ctrl == GPIO_LOW:
    print("ATX PSU: OFF (GPIO 15 = LOW)")
    return False
  else:
    print(f"ATX PSU: UNKNOWN (GPIO 15 ctrl = 0x{ctrl:02X})")
    return None


def main():
  if len(sys.argv) < 2 or sys.argv[1] not in ("on", "off", "toggle", "status"):
    print(f"Usage: {sys.argv[0]} <on|off|toggle|status>")
    sys.exit(1)

  action = sys.argv[1]
  handle, ctx = open_device()
  try:
    if action == "on":
      atx_on(handle)
    elif action == "off":
      atx_off(handle)
    elif action == "toggle":
      if atx_status(handle):
        atx_off(handle)
      else:
        atx_on(handle)
    elif action == "status":
      atx_status(handle)
  finally:
    close_device(handle, ctx)


if __name__ == "__main__":
  main()
