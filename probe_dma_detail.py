#!/usr/bin/env python3
"""
Detailed DMA register probing: identify volatile regs, test write-ability,
and document register behavior around the SCSI write path.

Usage:
  PYTHONPATH=~/tinygrad python3 probe_dma_detail.py
"""
import sys, os, random, time
sys.path.insert(0, "/home/geohot/tinygrad")

from tinygrad.runtime.support.usb import ASM24Controller

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def r16(addr): return ctrl.read(addr, 2)
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# ============================================================
# Test 1: Volatile register detection
# Read each register twice to find volatile ones
# ============================================================
print("="*60)
print("TEST 1: Volatile register detection (C400-C4FF)")
print("="*60)
for base in [0xC400, 0xC450, 0xC4B0]:
  d1 = rn(base, 0x30)
  d2 = rn(base, 0x30)
  for i in range(len(d1)):
    if d1[i] != d2[i]:
      print(f"  VOLATILE 0x{base+i:04X}: {d1[i]:02X} -> {d2[i]:02X}")

print("\nVolatile in CE00-CE9F:")
for base in [0xCE00, 0xCE40, 0xCE80]:
  d1 = rn(base, 0x20)
  d2 = rn(base, 0x20)
  for i in range(len(d1)):
    if d1[i] != d2[i]:
      print(f"  VOLATILE 0x{base+i:04X}: {d1[i]:02X} -> {d2[i]:02X}")

# ============================================================
# Test 2: Registers that changed between idle and post-write
# (from the big dump above, these changed)
# ============================================================
print("\n" + "="*60)
print("TEST 2: Key register values (current state)")
print("="*60)

interesting = [
  # C400 block - DMA arm/trigger
  (0xC400, "REG_NVME_CTRL (DMA arm 0)"),
  (0xC401, "REG_NVME_STATUS (DMA arm 1)"),
  (0xC402, "REG_NVME_DMA_ARM2"),
  (0xC404, "REG_NVME_DMA_ARM3"),
  (0xC406, "REG_NVME_DMA_TRIGGER"),
  # C410 block - changed
  (0xC412, "REG_NVME_CTRL_STATUS"),
  (0xC413, "REG_NVME_CONFIG"),
  (0xC414, "REG_NVME_DATA_CTRL"),
  (0xC415, "REG_NVME_DEV_STATUS"),
  # C420 block - descriptor
  (0xC420, "REG_NVME_DMA_XFER_HI"),
  (0xC421, "REG_NVME_DMA_XFER_LO"),
  (0xC422, "REG_NVME_LBA_LOW"),
  (0xC427, "REG_NVME_DMA_SECTOR_CNT"),
  (0xC428, "REG_NVME_QUEUE_CFG"),
  (0xC42A, "REG_NVME_DOORBELL"),
  (0xC42C, "REG_USB_MSC_CTRL"),
  (0xC42D, "REG_USB_MSC_STATUS"),
  # C430 - PRP (changed: C430 went 00->01 after write)
  (0xC430, "REG_NVME_CMD_PRP1"),
  # C450 block - changed significantly
  (0xC450, "REG_NVME_CMD_STATUS_50"),
  (0xC456, "C456 (unknown)"),
  (0xC45A, "C45A (unknown)"),
  (0xC45D, "C45D (unknown)"),
  (0xC45F, "C45F (unknown)"),
  (0xC46A, "C46A (unknown)"),
  (0xC46D, "C46D (unknown)"),
  (0xC46F, "C46F (unknown)"),
  # C470 block
  (0xC470, "C470 (changed C0 after write)"),
  (0xC471, "REG_NVME_QUEUE_BUSY"),
  (0xC473, "C473 (=0x66 always)"),
  (0xC478, "C478 (counter?)"),
  (0xC479, "C479 (counter?)"),
  (0xC47A, "C47A"),
  (0xC47B, "C47B"),
  (0xC47D, "C47D (=0x01 after write)"),
  (0xC47E, "C47E"),
  # C480 block
  (0xC487, "C487 (counter: 01->02->01)"),
  (0xC489, "C489 (01->00->01)"),
  (0xC48A, "C48A"),
  # C4B0 block - changed significantly
  (0xC4B0, "C4B0 (47->EA->9A)"),
  (0xC4B1, "C4B1 (00->03->58)"),
  (0xC4B2, "C4B2 (84->94)"),
  (0xC4B3, "C4B3"),
  (0xC4B4, "C4B4"),
  (0xC4B5, "C4B5"),
  # C4E block
  (0xC4EB, "C4EB"),
  (0xC4EC, "C4EC"),
  (0xC4ED, "REG_NVME_DMA_CTRL_ED"),
  (0xC4EF, "REG_NVME_DMA_ADDR_HI"),
  (0xC4F7, "C4F7"),
  (0xC4FB, "C4FB (appeared after write)"),
  # CE00 block
  (0xCE00, "REG_SCSI_DMA_CTRL"),
  (0xCE01, "REG_SCSI_DMA_PARAM"),
  (0xCE02, "CE02 (01->00 changed)"),
  (0xCE05, "CE05 (=0xFF always)"),
  (0xCE10, "REG_SCSI_DMA_SRAM_PTR_0"),
  (0xCE11, "REG_SCSI_DMA_SRAM_PTR_1"),
  (0xCE12, "REG_SCSI_DMA_SRAM_PTR_2"),
  (0xCE13, "REG_SCSI_DMA_SRAM_PTR_3"),
  # CE40 block
  (0xCE44, "CE44 (=0x50)"),
  (0xCE45, "CE45 (=0x05)"),
  (0xCE46, "CE46 (=0x55)"),
  (0xCE47, "CE47 (=0x50)"),
  (0xCE48, "CE48 (=0x05)"),
  (0xCE49, "CE49 (=0x55)"),
  (0xCE55, "REG_SCSI_DMA_XFER_CNT"),
  (0xCE5B, "CE5B (=0x10)"),
  (0xCE5C, "REG_SCSI_DMA_COMPL"),
  (0xCE5D, "REG_SCSI_DMA_MASK"),
  (0xCE5F, "REG_SCSI_DMA_QUEUE"),
  # CE60 block
  (0xCE63, "CE63 (=0xFF)"),
  (0xCE64, "CE64 (=0xFF)"),
  (0xCE6C, "CE6C"),
  (0xCE6D, "CE6D"),
  (0xCE6E, "REG_SCSI_DMA_STATUS_L"),
  (0xCE6F, "REG_SCSI_DMA_STATUS_H"),
  (0xCE70, "REG_SCSI_TRANSFER_CTRL"),
  (0xCE71, "CE71"),
  (0xCE72, "REG_SCSI_TRANSFER_MODE"),
  (0xCE73, "CE73 (=0x08)"),
  (0xCE76, "REG_SCSI_BUF_ADDR0 (LE LSB)"),
  (0xCE77, "REG_SCSI_BUF_ADDR1"),
  (0xCE78, "REG_SCSI_BUF_ADDR2"),
  (0xCE79, "REG_SCSI_BUF_ADDR3 (LE MSB)"),
  # CE80 block
  (0xCE80, "REG_SCSI_BUF_CTRL"),
  (0xCE81, "REG_SCSI_BUF_THRESH_HI"),
  (0xCE82, "REG_SCSI_BUF_THRESH_LO"),
  (0xCE83, "REG_SCSI_BUF_FLOW"),
  (0xCE86, "REG_USB_DMA_ERROR"),
  (0xCE87, "CE87"),
  (0xCE88, "REG_BULK_DMA_HANDSHAKE"),
  (0xCE89, "REG_USB_DMA_STATE"),
  (0xCE8A, "REG_USB_DMA_SECTOR_CTRL"),
  (0xCE8C, "CE8C"),
  (0xCE8D, "CE8D"),
  (0xCE8F, "CE8F"),
  (0xCE97, "REG_SCSI_DMA_RESP_REG"),
]

for addr, name in interesting:
  val = r(addr)
  if val != 0:
    print(f"  0x{addr:04X} = 0x{val:02X}  ({val:3d})  {name}")
  else:
    print(f"  0x{addr:04X} = 0x00         {name}")

# ============================================================
# Test 3: Write-read test on unknown registers
# Try to write and read back to see which are R/W vs R/O
# ============================================================
print("\n" + "="*60)
print("TEST 3: Write-read test (safe registers only)")
print("="*60)

# Test a few undocumented but stable registers
test_regs = [
  0xCE02, 0xCE03, 0xCE04, 0xCE06, 0xCE07,
  0xCE14, 0xCE15, 0xCE16, 0xCE17,
  0xCE20, 0xCE21, 0xCE22,
  0xCE3C, 0xCE3D, 0xCE3E, 0xCE3F,
  0xCE46, 0xCE47, 0xCE48, 0xCE49,
  0xCE50, 0xCE51, 0xCE52, 0xCE53, 0xCE54,
  0xCE5B,
  0xCE70, 0xCE71, 0xCE73, 0xCE74, 0xCE75,
  0xCE84, 0xCE85,
  0xCE8B, 0xCE8C, 0xCE8D, 0xCE8E, 0xCE8F,
  0xCE90, 0xCE91, 0xCE92, 0xCE93, 0xCE94, 0xCE95, 0xCE96, 0xCE97,
]

for addr in test_regs:
  orig = r(addr)
  test_val = 0xAA if orig != 0xAA else 0x55
  w(addr, test_val)
  readback = r(addr)
  w(addr, orig)  # restore
  rw = "R/W" if readback == test_val else "R/O" if readback == orig else f"PARTIAL({readback:02X})"
  if rw != "R/O" or orig != 0:
    print(f"  0x{addr:04X}: orig=0x{orig:02X}, wrote=0x{test_val:02X}, read=0x{readback:02X} -> {rw}")

# ============================================================
# Test 4: Read the 0x900B MSC config and related USB regs
# ============================================================
print("\n" + "="*60)
print("TEST 4: MSC/USB Engine registers")
print("="*60)

msc_regs = [
  (0x900B, "REG_USB_MSC_CFG"),
  (0x9002, "REG_USB_CONFIG"),
  (0x9005, "REG_USB_CTRL"),
  (0x90E2, "REG_USB_MODE"),
  (0x9091, "REG_USB_EP1_CFG1"),
  (0x9093, "REG_USB_EP2_CFG1"),
  (0x901A, "REG_USB_XFER_LEN"),
  (0x9100, "REG_USB_LINK_STATUS"),
  (0x91C1, "REG_USB_EP_ARM"),
  (0x91D1, "REG_USB_INT_FLAGS"),
  (0x92C0, "REG_POWER_ENABLE"),
  (0x9300, "REG_USB_EP_DESC0"),
  (0x9301, "REG_USB_EP_DESC1"),
  (0x9302, "REG_USB_EP_DESC2"),
  (0x9303, "REG_USB_EP_DESC3"),
  (0x9304, "REG_USB_EP_DESC4"),
  (0x9305, "REG_USB_EP_DESC5"),
]

for addr, name in msc_regs:
  val = r(addr)
  print(f"  0x{addr:04X} = 0x{val:02X}  {name}")

# ============================================================
# Test 5: Rapid reads of CE88/CE89 to capture state machine
# ============================================================
print("\n" + "="*60)
print("TEST 5: CE88/CE89 state reads (10 samples)")
print("="*60)
for i in range(10):
  s88 = r(0xCE88)
  s89 = r(0xCE89)
  print(f"  [{i}] CE88=0x{s88:02X}  CE89=0x{s89:02X}  (bits: {s89:08b})")

# ============================================================
# Test 6: Read entire C450-C470 to identify the queue/status
# counters that appeared after scsi_write
# ============================================================
print("\n" + "="*60)
print("TEST 6: C450-C470 detail (queue status area)")
print("="*60)
d = rn(0xC450, 0x30)
for i in range(0, len(d), 1):
  if d[i] != 0:
    print(f"  0x{0xC450+i:04X} = 0x{d[i]:02X}")

print("\nDone!")
