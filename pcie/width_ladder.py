#!/usr/bin/env python3
"""Width ladder measurement. Two phases:

  phase set <b431>:   raw USB (no GPU driver): write B431, re-kick training
                      (E764 stop/start), wait for link, print result.
  phase measure:      boot GPU driver, force Gen2 target, measure SDMA
                      SRAM->VRAM throughput with GPU timestamps.

Drive per width from bash with a chip reset in between.
"""
import sys
import time

sys.path.insert(0, "pcie")


def phase_set(b431):
    from pcie_bringup import ASM2464PD
    dev = ASM2464PD()
    dev.open()
    dev.write(0xB431, b431)
    dev.write(0xE764, 0x00)
    time.sleep(0.05)
    dev.write(0xE764, 0x1C)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8:
        v = dev.bank1_read(0x4092)[0]
        e765 = dev.read8(0xE765)
        if e765 & 0x02:
            print(f"link up after %.1fs: 0x4092=0x{v:02X} -> Gen{v & 0xF} x{(v >> 4) & 0xF}" % (time.monotonic() - t0))
            dev.close()
            return True
        time.sleep(0.1)
    print("link did NOT come up, LTSSM=0x%02X" % dev.read8(0xB450))
    dev.close()
    return False


def phase_measure(gen=2):
    import types
    from tinygrad import Device
    from tinygrad.runtime.ops_amd import AMDCopyQueue
    from tinygrad.runtime.support.memory import AddrSpace

    amd = Device["AMD"]
    mm = amd.iface.dev_impl.mm
    usb = amd.iface.pci_dev.usb

    def pcie_cap(bus):
        cap = usb.pcie_cfg_req(0x34, bus=bus, dev=0, fn=0, size=1)
        while cap:
            hdr = usb.pcie_cfg_req(cap, bus=bus, dev=0, fn=0, size=4)
            if (hdr & 0xFF) == 0x10:
                return cap
            cap = (hdr >> 8) & 0xFF

    cap = pcie_cap(1)
    # force target speed + retrain for a constant-speed ladder
    lnkctl2 = usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, size=4)
    usb.pcie_cfg_req(cap + 0x30, bus=1, dev=0, fn=0, value=(lnkctl2 & ~0xF) | gen, size=2)
    lnkctl = usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, size=2)
    usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, value=lnkctl | 0x20, size=2)
    time.sleep(0.5)
    lnk = usb.pcie_cfg_req(cap + 0x10, bus=1, dev=0, fn=0, size=4)
    gen_a, width = (lnk >> 16) & 0xF, (lnk >> 20) & 0x3F

    total = 64 << 20
    CHUNK = 0x40000
    vram = mm.valloc(total, uncached=True)
    sram_va = mm.alloc_vaddr(size=2 * CHUNK)
    mm.map_range(sram_va, 2 * CHUNK, [(0x200000, 2 * CHUNK)], aspace=AddrSpace.SYS, uncached=True)
    reps = total // CHUNK
    done = 0
    gpu_us = 0.0
    BATCH = 16
    while done < reps:
        n = min(BATCH, reps - done)
        s0, s1 = amd.new_signal(), amd.new_signal()
        q = AMDCopyQueue(amd).timestamp(s0)
        for i in range(n):
            k = done + i
            q = q.copy(types.SimpleNamespace(va_addr=vram.va_addr + k * CHUNK),
                       types.SimpleNamespace(va_addr=sram_va + (k % 2) * CHUNK), CHUNK)
        q = q.timestamp(s1).signal(amd.timeline_signal, amd.next_timeline())
        q.submit(amd)
        amd.timeline_signal.wait(amd.timeline_value - 1, timeout=120000)
        gpu_us += float(s1.timestamp - s0.timestamp)
        done += n
    bw = total / (gpu_us * 1e-6) / 1e9
    print(f"MEASURE: Gen{gen_a} x{width} = {bw:.2f} GB/s")


if __name__ == "__main__":
    if sys.argv[1] == "set":
        ok = phase_set(int(sys.argv[2], 0))
        sys.exit(0 if ok else 1)
    elif sys.argv[1] == "measure":
        phase_measure(int(sys.argv[2]) if len(sys.argv) > 2 else 2)
