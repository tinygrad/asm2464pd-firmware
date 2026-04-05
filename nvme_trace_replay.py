#!/usr/bin/env python3
"""Replay clean_nvme trace to trigger hardware BULK IN DMA.

Prerequisites:
  make -C handmade nflash   # flash handmade firmware
  ./ftdi_debug.py -rn       # reset device
  sleep 8
  python3 pcie/pcie_bringup.py --skip-hw   # bring PCIe link up
  python3 nvme_trace_replay.py              # this script

Replays the entire stock firmware trace (skipping only USB-killing
registers: E716, C800, C2xx SerDes, 92xx USB PHY, CC11 timer polls,
idle loop status reads) then captures the hardware-initiated BULK IN
DMA data from the NVMe read.
"""

import re, ctypes, time, threading, sys
from tinygrad.runtime.autogen import libusb

VID, PID = 0xADD1, 0x0001

def main():
    ctx = ctypes.POINTER(libusb.libusb_context)()
    libusb.libusb_init(ctypes.byref(ctx))
    handle = libusb.libusb_open_device_with_vid_pid(ctx, VID, PID)
    assert handle, 'Device not found'
    libusb.libusb_claim_interface(handle, 0)

    def w8(a,v):
        ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, a&0xFFFF, v&0xFF, None, 0, 1000)
        if ret < 0: raise IOError(ret)
    def r8(a):
        buf = (ctypes.c_ubyte*1)()
        ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, a&0xFFFF, 0, buf, 1, 1000)
        if ret < 0: raise IOError(ret)
        return buf[0]
    def dpx_w8(a,v,b):
        ret = libusb.libusb_control_transfer(handle, 0x40, 0xE5, a&0xFFFF, ((b&0xFF)<<8)|(v&0xFF), None, 0, 1000)
        if ret < 0: raise IOError(ret)
    def dpx_r8(a,b):
        buf = (ctypes.c_ubyte*1)()
        ret = libusb.libusb_control_transfer(handle, 0xC0, 0xE4, a&0xFFFF, b<<8, buf, 1, 1000)
        if ret < 0: raise IOError(ret)
        return buf[0]

    # Parse trace
    ops = []
    lc = 0
    with open('trace/clean_nvme') as f:
        for line in f:
            line = line.strip()
            m = re.match(r'\[\s*(\d+)\]\s+PC=0x\w+\s+Write\s+0x(\w+)\s*=\s*0x(\w+)', line)
            if m: lc=int(m.group(1)); ops.append(('W',lc,int(m.group(2),16),int(m.group(3),16),0)); continue
            m = re.match(r'\[\s*(\d+)\]\s+PC=0x\w+\s+Read\s+0x(\w+)\s*=\s*0x(\w+)', line)
            if m: lc=int(m.group(1)); ops.append(('R',lc,int(m.group(2),16),int(m.group(3),16),0)); continue
            m = re.match(r'Write\s+0x(\w+)\s*=\s*0x(\w+)\s+DPX=(\d+)', line)
            if m: ops.append(('D',lc,int(m.group(1),16),int(m.group(2),16),int(m.group(3)))); continue
            m = re.match(r'Read\s+0x(\w+)\s*=\s*0x(\w+)\s+DPX=(\d+)', line)
            if m: ops.append(('r',lc,int(m.group(1),16),int(m.group(2),16),int(m.group(3)))); continue

    print(f'Parsed {len(ops)} trace operations')

    skip_data = lambda a: (0x7000<=a<0x7100)or(0x8000<=a<0x9000)or(0x9E00<=a<0x9F00)
    IDLE_READS = {0xE716, 0x92F7, 0x9091, 0x9101, 0x9000, 0xCD31, 0xC655, 0xC620,
                  0xC65A, 0xCE89, 0x9002, 0x9003, 0x9004, 0xC806, 0x9100, 0x9104,
                  0x9105, 0x9106, 0x9220, 0xCC11, 0xC802}
    def unsafe_write(addr):
        return (addr == 0xE716 or addr == 0xC800 or
                0xC200 <= addr < 0xC400 or 0x9200 <= addr < 0x9300 or
                addr == 0x90A1)

    def replay(start_cycle, end_cycle):
        errs = 0
        for typ,cyc,addr,val,bnk in ops:
            if cyc < start_cycle: continue
            if cyc >= end_cycle: break
            if typ=='W' and unsafe_write(addr): continue
            if typ=='R' and addr in IDLE_READS: continue
            if typ in('W','R') and skip_data(addr): continue
            try:
                if typ=='W': w8(addr,val)
                elif typ=='R':
                    if addr == 0xB80E:
                        dl=time.monotonic()+5.0
                        while time.monotonic()<dl:
                            if r8(addr)==val: break
                            time.sleep(0.001)
                    else: r8(addr)
                elif typ=='D': dpx_w8(addr,val,bnk)
                elif typ=='r': dpx_r8(addr,bnk)
            except IOError:
                errs += 1
        return errs

    # Check PCIe
    ltssm = r8(0xB450)
    print(f'LTSSM=0x{ltssm:02X}')
    if ltssm != 0x78:
        print('PCIe not up! Run: python3 pcie/pcie_bringup.py --skip-hw')
        return 1

    # Phase 1: replay everything up to the final doorbell
    print('Phase 1: replay trace (85k to 717500)...')
    t0 = time.monotonic()
    errs = replay(85000, 717500)
    print(f'  {time.monotonic()-t0:.1f}s, {errs} errors')

    # Phase 2: submit bulk IN reads in background
    results = []
    def bulk_read(size, timeout):
        buf = (ctypes.c_ubyte*size)()
        xfer = ctypes.c_int(0)
        ret = libusb.libusb_bulk_transfer(handle, 0x81, buf, size, ctypes.byref(xfer), timeout)
        results.append((ret, bytes(buf[:xfer.value])))

    t1 = threading.Thread(target=bulk_read, args=(512, 15000))
    t2 = threading.Thread(target=bulk_read, args=(64, 15000))
    t1.start()
    t2.start()
    time.sleep(0.05)
    print('Phase 2: bulk IN submitted, replaying doorbell + completion...')

    # Phase 3: replay the doorbell ring + ISR completion sequence
    replay(717500, 718200)

    # Wait for results
    t1.join(timeout=20)
    t2.join(timeout=20)

    print(f'\n=== Results ({len(results)} bulk transfers) ===')
    for idx, (ret, data) in enumerate(results):
        print(f'  Transfer #{idx+1}: ret={ret} len={len(data)}')
        if len(data) > 0:
            if len(data) >= 512:
                print(f'\n  ===== HARDWARE DMA: {len(data)} BYTES OF NVME DATA =====')
            for off in range(0, min(len(data), 128), 16):
                h = ' '.join(f'{b:02x}' for b in data[off:off+16])
                a = ''.join(chr(c) if 32<=c<127 else '.' for c in data[off:off+16])
                print(f'    {off:04x}: {h}  {a}')

    libusb.libusb_release_interface(handle, 0)
    libusb.libusb_close(handle)
    libusb.libusb_exit(ctx)

    # Success if we got any bulk data
    total_bytes = sum(len(d) for _, d in results)
    if total_bytes > 0:
        print(f'\nSUCCESS: {total_bytes} bytes received via hardware BULK IN DMA')
        return 0
    return 1

if __name__ == '__main__':
    sys.exit(main())
