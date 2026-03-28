#!/usr/bin/env python3
"""
Minimal DMA: bulk OUT -> 0x7000 -> C400 DMA -> 0xF000 (PCIe BAR).

Trigger: C420 descriptor + C400/C401/C402/C404/C406 = 1
Unlock:  C42A = 1 (REG_NVME_DOORBELL)
"""

import ctypes
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

class Dev:
  def __init__(self):
    self.dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)
  def read8(self, addr):
    buf = (ctypes.c_ubyte * 1)()
    ret = libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
    assert ret >= 0, f"read(0x{addr:04X}) failed: {ret}"
    return buf[0]
  def write(self, addr, val):
    ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
    assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"
  def read_range(self, addr, n):
    return bytes([self.read8(addr + i) for i in range(n)])

def dma_trigger(d, sectors=1):
  """Copy 0x7000 -> PCIe BAR (0xF000). sectors * 512 bytes."""
  # Transfer descriptor
  d.write(0xC420, 0x00)  # REG_NVME_DMA_XFER_HI
  d.write(0xC421, 0x01)  # REG_NVME_DMA_XFER_LO
  d.write(0xC422, 0x02)  # REG_NVME_LBA_LOW
  d.write(0xC423, 0x00)  # REG_NVME_LBA_MID
  d.write(0xC424, 0x00)  # REG_NVME_LBA_HIGH
  d.write(0xC425, 0x00)  # REG_NVME_COUNT_LOW
  d.write(0xC426, 0x00)  # REG_NVME_COUNT_HIGH
  d.write(0xC427, sectors)  # REG_NVME_DMA_ADDR_C427 (sector count)
  d.write(0xC428, 0x30)  # REG_NVME_QUEUE_CFG
  d.write(0xC429, 0x00)  # REG_NVME_CMD_PARAM
  d.write(0xC42A, 0x00)  # REG_NVME_DOORBELL
  d.write(0xC42B, 0x00)  # REG_NVME_CMD_FLAGS
  d.write(0xC42C, 0x00)  # REG_USB_MSC_CTRL
  d.write(0xC42D, 0x00)  # REG_USB_MSC_STATUS
  # Arm and trigger
  d.write(0xC400, 1)  # REG_NVME_CTRL
  d.write(0xC401, 1)  # REG_NVME_STATUS
  d.write(0xC402, 1)
  d.write(0xC404, 1)
  d.write(0xC406, 1)  # trigger

def dma_unlock(d):
  """Unlock 0x7000 buffer for next transfer."""
  d.write(0xC42A, 0x01)  # REG_NVME_DOORBELL

def main():
  d = Dev()
  print("5 DMA transfers:\n")
  for i in range(5):
    pat = bytes([0x10*i + j for j in range(16)]) + b'\x00' * 16
    d.dev._bulk_out(0x02, pat)
    x7 = d.read_range(0x7000, 4)
    dma_trigger(d)
    f = d.read_range(0xF000, 4)
    ok = f == pat[:4]
    print(f"  [{i}] 7000={' '.join(f'{b:02X}' for b in x7)} "
          f"F000={' '.join(f'{b:02X}' for b in f)} {'OK' if ok else 'FAIL'}")
    dma_unlock(d)

if __name__ == "__main__":
  main()
