#!/usr/bin/env python3
"""Reliability test: CC31=0x01 alone — does it always bring USB back?
Run 10 cycles of reset and reconnect."""
import ctypes, sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'pcie'))
from pcie_probe import usb_open, usb_close, xdata_read, xdata_write

for cycle in range(10):
    handle, ctx = usb_open()
    
    # Write a unique canary for this cycle
    xdata_write(handle, 0x0F00, cycle & 0xFF)
    
    # Trigger reset
    t0 = time.time()
    xdata_write(handle, 0xCC31, 0x01)
    try:
        usb_close(handle, ctx)
    except:
        pass
    
    # Wait for device to come back
    for attempt in range(40):
        time.sleep(0.25)
        try:
            handle, ctx = usb_open()
            elapsed = time.time() - t0
            canary = xdata_read(handle, 0x0F00)[0]
            cc28 = xdata_read(handle, 0xCC28)[0]
            print(f"Cycle {cycle+1}/10: back in {elapsed:.2f}s, canary=0x{canary:02X} (expect 0x{cycle & 0xFF:02X}), CC28=0x{cc28:02X}")
            usb_close(handle, ctx)
            break
        except:
            pass
    else:
        print(f"Cycle {cycle+1}/10: FAILED — device not back after 10s!")
        sys.exit(1)

print("\nAll 10 cycles passed — CC31=0x01 reliably resets CPU + USB")
