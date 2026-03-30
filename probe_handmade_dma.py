#!/usr/bin/env python3
"""
Test C400 DMA on HANDMADE firmware.
Handmade firmware uses control-transfer E4/E5 and raw bulk OUT.
MSC is disabled (MSC_CFG=0x00), so bulk OUT data goes directly to 0x7000.
"""
import sys, ctypes, struct, random
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

dev = USB3(0xADD1, 0x0001, 0x81, 0x83, 0x02, 0x04, use_bot=True)

def read8(addr):
  buf = (ctypes.c_ubyte * 1)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
  assert ret >= 0, f"read(0x{addr:04X}) failed: {ret}"
  return buf[0]

def write(addr, val):
  ret = libusb.libusb_control_transfer(dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
  assert ret >= 0, f"write(0x{addr:04X}, 0x{val:02X}) failed: {ret}"

def read_range(addr, n):
  buf = (ctypes.c_ubyte * n)()
  ret = libusb.libusb_control_transfer(dev.handle, 0xC0, 0xE4, addr, 0, buf, n, 1000)
  assert ret >= 0, f"read(0x{addr:04X}, {n}) failed: {ret}"
  return bytes(buf[:ret])

def bulk_out(data):
  dev._bulk_out(0x02, data)

# ============================================================
print("="*60)
print("Handmade firmware: C400 DMA test")
print("="*60)

print(f"  900B (MSC_CFG): 0x{read8(0x900B):02X}")
print(f"  F000 (SRAM): {read_range(0xF000, 4).hex()}")
print(f"  7000 (buf): {read_range(0x7000, 4).hex()}")

# ============================================================
print("\n--- Step 1: bulk OUT 32 bytes ---")
pat = bytes([0xDE, 0xAD, 0xBE, 0xEF] + [i for i in range(28)])
bulk_out(pat)

buf = read_range(0x7000, 8)
print(f"  7000: {buf.hex()} (expect deadbeef...)")

# ============================================================
print("\n--- Step 2: Set C420 descriptor + arm ---")
write(0xC420, 0x00)
write(0xC421, 0x01)
write(0xC422, 0x02)
write(0xC423, 0x00)
write(0xC424, 0x00)
write(0xC425, 0x00)
write(0xC426, 0x00)
write(0xC427, 0x01)  # 1 sector
write(0xC428, 0x30)
write(0xC429, 0x00)
write(0xC42B, 0x00)
write(0xC42C, 0x00)
write(0xC42D, 0x00)

print("  Arming...")
write(0xC400, 1)
write(0xC401, 1)
write(0xC402, 1)
write(0xC404, 1)

print("  Triggering C406...")
write(0xC406, 1)

# ============================================================
print("\n--- Step 3: Check result ---")
f = read_range(0xF000, 8)
print(f"  F000: {f.hex()}")
print(f"  Expected: {pat[:8].hex()}")
print(f"  MATCH: {f == pat[:8]}")

# Check 0x7000 lock state
buf = read_range(0x7000, 4)
print(f"  7000: {buf.hex()} (0x55555555 = locked)")

# ============================================================
print("\n--- Step 4: Unlock and repeat ---")
write(0xC42A, 0x01)
buf = read_range(0x7000, 4)
print(f"  7000 after unlock: {buf.hex()}")

# Second transfer
pat2 = bytes([0xCA, 0xFE, 0xBA, 0xBE] + [0]*28)
bulk_out(pat2)
buf = read_range(0x7000, 4)
print(f"  7000: {buf.hex()}")

write(0xC427, 0x01)
write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
write(0xC406, 1)

f = read_range(0xF000, 4)
print(f"  F000: {f.hex()} (expect cafebabe)")
print(f"  MATCH: {f == pat2[:4]}")
write(0xC42A, 0x01)

# ============================================================
print("\n--- Step 5: 5 rapid transfers ---")
for i in range(5):
  p = bytes([0x10*i + j for j in range(4)] + [0]*28)
  bulk_out(p)
  write(0xC427, 0x01)
  write(0xC400, 1); write(0xC401, 1); write(0xC402, 1); write(0xC404, 1)
  write(0xC406, 1)
  f = read_range(0xF000, 4)
  ok = f == p[:4]
  print(f"  [{i}] F000={f.hex()} {'OK' if ok else 'FAIL'}")
  write(0xC42A, 0x01)

# ============================================================
print("\n--- Step 6: Minimum arm bits ---")
def test_arm(label, c400, c401, c402, c404):
  p = bytes(random.getrandbits(8) for _ in range(32))
  bulk_out(p)
  write(0xC427, 0x01)
  if c400: write(0xC400, 1)
  if c401: write(0xC401, 1)
  if c402: write(0xC402, 1)
  if c404: write(0xC404, 1)
  write(0xC406, 1)
  f = read_range(0xF000, 4)
  ok = f == p[:4]
  print(f"  {label}: {'OK' if ok else 'FAIL'} (F000={f.hex()} expected={p[:4].hex()})")
  write(0xC42A, 0x01)
  return ok

test_arm("all arms",    True,  True,  True,  True)
test_arm("no C400",     False, True,  True,  True)
test_arm("no C401",     True,  False, True,  True)
test_arm("no C402",     True,  True,  False, True)
test_arm("no C404",     True,  True,  True,  False)
test_arm("only C406",   False, False, False, False)

print("\nDone!")
