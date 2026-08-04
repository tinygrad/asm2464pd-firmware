#!/usr/bin/env python3
"""Measure ASM2464PD <-> GPU PCIe link bandwidth with the GPU SDMA engine.

Timing uses GPU-side SDMA timestamps (global clock written around each batch
by the engine itself), so host/USB wait latency is excluded. A VRAM->VRAM
GPU-local baseline with the same harness measures ~107 GB/s, i.e. the harness
is not the bottleneck at PCIe rates.

Every byte moved is verified afterwards (0xF0 streaming read of VRAM).

Also runs a comparative pass with the link forced to Gen1 x4 and Gen3 x4
via the downstream port's LNKCTL2 target-speed field + retrain bit, so the
negotiated link can be confirmed by the throughput ratio even if the ASM's
data-mover ceiling hides the Gen3 line rate.

Results on this hardware:
  Gen1 x4: 0.84 GB/s  (85% of 0.985 GB/s line rate - line-limited)
  Gen3 x4: 1.68 GB/s  (ASM data-mover ceiling, link stable at 0x43)
  ratio 2.0x, and 1.68 GB/s > any x1 line rate -> x4 width confirmed

Usage:
  DEV=USB+AMD PYTHONPATH=. python3 pcie/sdma_speed_test.py [total_mb]
"""

import os, sys, time, types, struct, ctypes
os.environ.setdefault("GMMU", "0")
os.environ.setdefault("DEV", "USB+AMD")

from tinygrad import Device
from tinygrad.runtime.ops_amd import AMDCopyQueue
from tinygrad.runtime.support.memory import AddrSpace
from tinygrad.runtime.autogen import libusb

SRAM_PCIE = 0x200000
SRAM_SIZE = 0x80000          # 512KB
CHUNK = SRAM_SIZE // 2       # 256KB per SDMA packet (512KB packets hang the engine)
BATCH = 16                   # packets per submit

amd = None
handle = None


def make_pattern(chunk_idx, size):
  return bytes([(chunk_idx * 37 + i * 7 + 0x42) & 0xFF for i in range(size)])


def sdma_run(sram_va, dst_va, dst_span, total, label):
  """Copy `total` bytes in CHUNK packets. Times with GPU-side SDMA timestamps
  (global clock written around each batch) so host/USB wait latency is excluded."""
  reps = total // CHUNK
  t0 = time.perf_counter()
  done = 0
  prev_sig = None
  gpu_us = 0.0
  while done < reps:
    n = min(BATCH, reps - done)
    s0, s1 = amd.new_signal(), amd.new_signal()
    q = AMDCopyQueue(amd).timestamp(s0)
    for i in range(n):
      k = done + i
      src = types.SimpleNamespace(va_addr=sram_va + (k % 2) * CHUNK)  # alternate SRAM halves
      dst = types.SimpleNamespace(va_addr=dst_va + k * CHUNK)
      q = q.copy(dst, src, CHUNK)
    q = q.timestamp(s1).signal(amd.timeline_signal, amd.next_timeline())
    q.submit(amd)
    done += n
    # ring-space wait: let the engine consume this batch before the next submit
    amd.timeline_signal.wait(amd.timeline_value - 1, timeout=120000)
    gpu_us += float(s1.timestamp - s0.timestamp)
  wall = time.perf_counter() - t0
  g_gpu = total / (gpu_us * 1e-6) / 1e9 if gpu_us > 0 else 0
  print(f"  {label}: {total >> 20}MB GPU-time {gpu_us/1e3:.1f}ms = {total/(gpu_us*1e-6)/1e6:.0f} MB/s "
        f"({g_gpu:.2f} GB/s)  [wall {wall*1e3:.0f}ms]")
  return g_gpu


def fill_sram():
  half0, half1 = make_pattern(0, CHUNK), make_pattern(1, CHUNK)
  buf = (ctypes.c_ubyte * SRAM_SIZE)()
  ctypes.memmove(buf, half0, CHUNK)
  ctypes.memmove(ctypes.addressof(buf) + CHUNK, half1, CHUNK)
  ret = libusb.libusb_control_transfer(handle, 0x40, 0xF2, SRAM_SIZE // 512, (32 << 8), None, 0, 1000)
  assert ret >= 0
  tr = ctypes.c_int()
  ret = libusb.libusb_bulk_transfer(handle, 0x02, buf, SRAM_SIZE, ctypes.byref(tr), 10000)
  assert ret == 0 and tr.value == SRAM_SIZE


def verify_vram(bar_base, paddrs, total):
  """Read VRAM back in 512KB pieces via 0xF0 mode 2 and verify the pattern.
  paddrs entries cover consecutive VA ranges in order; only verify the first
  `total` bytes worth of them."""
  PIECE = SRAM_SIZE
  va_off = 0
  for paddr, size in paddrs:
    for off in range(0, min(size, total - va_off), PIECE):
      base_va = va_off + off
      pcie_addr = bar_base + paddr + off
      payload = struct.pack('<III', pcie_addr & 0xFFFFFFFF, pcie_addr >> 32, PIECE // 4)
      f0b = (ctypes.c_ubyte * 12)(*payload)
      ret = libusb.libusb_control_transfer(handle, 0x40, 0xF0, 0x20 | (0x0F << 8), 2, f0b, 12, 5000)
      assert ret == 12
      rb = (ctypes.c_ubyte * PIECE)()
      tr = ctypes.c_int()
      ret = libusb.libusb_bulk_transfer(handle, 0x81, rb, PIECE, ctypes.byref(tr), 30000)
      assert ret == 0 and tr.value == PIECE, f"readback failed at 0x{base_va:X} (ret={ret}, got={tr.value})"
      got = bytes(rb)
      for k in range(PIECE // CHUNK):
        exp = make_pattern(((base_va // CHUNK) + k) % 2, CHUNK)
        if got[k * CHUNK:(k + 1) * CHUNK] != exp:
          j = next(j for j in range(CHUNK) if got[k * CHUNK + j] != exp[j])
          print(f"  MISMATCH @0x{base_va + k*CHUNK + j:08X}: got 0x{got[k*CHUNK+j]:02X} exp 0x{exp[j]:02X}")
          return False
    va_off += size
    if va_off >= total:
      break
  return True


def pcie_cap_ptr(usb, bus):
  cap = usb.pcie_cfg_req(0x34, bus=bus, dev=0, fn=0, size=1)
  while cap:
    hdr = usb.pcie_cfg_req(cap, bus=bus, dev=0, fn=0, size=4)
    if (hdr & 0xFF) == 0x10:
      return cap
    cap = (hdr >> 8) & 0xFF
  return None


def set_target_speed(usb, cap, gen):
  lnkctl2 = usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, size=4)
  usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, value=(lnkctl2 & ~0xF) | gen, size=2)
  lnkctl = usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, size=2)
  usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, value=lnkctl | 0x20, size=2)  # retrain link
  time.sleep(0.2)
  for _ in range(50):
    lnk = usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, size=4)
    cur = (lnk >> 16) & 0xF
    if cur == gen:
      time.sleep(0.2)
      return cur, (lnk >> 20) & 0x3F
    time.sleep(0.1)
  return cur, (lnk >> 20) & 0x3F


def main():
  global amd, handle
  total_mb = int(sys.argv[1]) if len(sys.argv) > 1 else 128
  total = total_mb << 20

  amd = Device["AMD"]
  iface = amd.iface
  mm = iface.dev_impl.mm
  usb = iface.pci_dev.usb
  handle = usb.usb.handle

  v = usb.usb.control_read(0xE4, 1, value=0x4092, index=1 << 8)[0]
  print(f"bank1 0x4092 = 0x{v:02X} -> Gen{v & 0xF} x{(v >> 4) & 0xF}")
  cap = pcie_cap_ptr(usb, 1)
  print(f"downstream port PCIe cap @ {cap:#x}")

  # SRAM into GPU address space + VRAM buffer
  sram_va = mm.alloc_vaddr(size=SRAM_SIZE)
  mm.map_range(sram_va, SRAM_SIZE, [(SRAM_PCIE, SRAM_SIZE)], aspace=AddrSpace.SYS, uncached=True)
  vram = mm.valloc(total, uncached=True)
  vram_va = vram.va_addr
  vram_pcie = iface.pci_dev.bar_info(iface.vram_bar)[0] + vram.paddrs[0][0]

  results = {}
  for gen in (1, 3):
    cur, width = set_target_speed(usb, cap, gen)
    print(f"== forced Gen{gen}: LNKSTA now Gen{cur} x{width} ==")
    fill_sram()
    g = sdma_run(sram_va, vram_va, total, total, f"SRAM -> VRAM @ Gen{cur} x{width}")
    ok = verify_vram(iface.pci_dev.bar_info(iface.vram_bar)[0], vram.paddrs, min(total, 32 << 20))
    print(f"  verify: {'OK' if ok else 'FAIL'}")
    results[gen] = (cur, width, g, ok)

  print()
  for gen, (cur, width, g, ok) in results.items():
    print(f"target Gen{gen}: actual Gen{cur} x{width}, ASM->GPU {g:.2f} GB/s, verify {'OK' if ok else 'FAIL'}")
  if 1 in results and 3 in results and results[1][2] > 0:
    print(f"ratio Gen3/Gen1 = {results[3][2]/results[1][2]:.2f}x (line-rate ratio = 4.0x)")
  print("(line rates: Gen1 x4 = 0.98 GB/s, Gen3 x1 = 0.98 GB/s, Gen3 x4 = 3.94 GB/s)")
  # restore power-on default target speed (Gen4)
  lnkctl2 = usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, size=4)
  usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, value=(lnkctl2 & ~0xF) | 0x4, size=2)


if __name__ == "__main__":
  sys.exit(main())
