#!/usr/bin/env python3
import ctypes

from tinygrad.runtime.autogen import libusb

VID, PID = 0x3801, 0x0001


def main():
  ctx = ctypes.POINTER(libusb.libusb_context)()
  ret = libusb.libusb_init(ctypes.byref(ctx))
  if ret != 0:
    raise RuntimeError(f"libusb_init failed: {ret}")

  handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
  if not handle:
    libusb.libusb_exit(ctx)
    raise RuntimeError(f"device {VID:04X}:{PID:04X} not found")

  try:
    libusb.libusb_claim_interface(handle, 0)
    ret = libusb.libusb_control_transfer(handle, 0x40, 0xF3, 0, 0, None, 0, 1000)
    if ret < 0:
      raise RuntimeError(f"0xF3 power off failed: {ret}")
  finally:
    libusb.libusb_release_interface(handle, 0)
    libusb.libusb_close(handle)
    libusb.libusb_exit(ctx)


if __name__ == "__main__":
  main()
