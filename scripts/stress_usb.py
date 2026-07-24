#!/usr/bin/env python3
"""USB3 bulk stress test for ASM2464PD custom firmware.

Modes: out (sink), in (source), loopback, all.

Usage:
    python3 test_stress_usb.py --mode all
    python3 test_stress_usb.py --mode out --size 65536 --duration 5
"""
import argparse, signal, sys, time
import usb.core, usb.util

VID, PID, EP_OUT, EP_IN, IFACE = 0xADD1, 0x0001, 0x02, 0x81, 0
STRESS_OFF, STRESS_OUT, STRESS_IN, STRESS_LOOP = 0, 1, 2, 3
stop_requested = False

def request_stop(signum, frame):
    """Let the transfer loop turn stress mode off before exiting."""
    global stop_requested
    stop_requested = True

def open_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None: raise RuntimeError(f"Device {VID:04X}:{PID:04X} not found")
    if dev.is_kernel_driver_active(IFACE): dev.detach_kernel_driver(IFACE)
    dev.set_configuration()
    usb.util.claim_interface(dev, IFACE)
    # A deliberately bad PHY candidate can leave the xHCI endpoint context
    # halted even after the ASM has returned to U0.  Start every measurement
    # from explicit standard endpoint-halt clears; the firmware's handlers
    # also use these requests to discard stale stress/endpoint state.
    dev.clear_halt(EP_OUT)
    dev.clear_halt(EP_IN)
    return dev

def set_stress(dev, mode):
    dev.ctrl_transfer(0x40, 0xF4, mode, 0, None, 5000)

def stress(dev, mode, size, duration):
    global stop_requested
    stop_requested = False
    set_stress(dev, mode)
    buf = bytes(range(256)) * (size // 256 + 1)
    buf = buf[:size]
    total, errors = 0, 0
    start = time.perf_counter()
    while not stop_requested and time.perf_counter() - start < duration:
        try:
            if mode == STRESS_OUT:
                total += dev.write(EP_OUT, buf, timeout=5000)
            elif mode == STRESS_IN:
                total += len(dev.read(EP_IN, size, timeout=5000))
            elif mode == STRESS_LOOP:
                dev.write(EP_OUT, buf, timeout=5000)
                total += len(dev.read(EP_IN, size, timeout=5000))
        except usb.core.USBError as e:
            errors += 1
            print(
                f"{'out' if mode == STRESS_OUT else 'in' if mode == STRESS_IN else 'loopback'} "
                f"transfer error #{errors}: errno={e.errno} backend={e.backend_error_code} {e}",
                file=sys.stderr,
            )
            if e.errno in (5, 32):
                dev.clear_halt(EP_OUT)
                dev.clear_halt(EP_IN)
                set_stress(dev, mode)
            if errors > 50: break
    elapsed = time.perf_counter() - start
    set_stress(dev, STRESS_OFF)
    return total, elapsed, errors

def main():
    p = argparse.ArgumentParser(description="USB3 bulk stress test for ASM2464PD")
    p.add_argument("--mode", choices=["out", "in", "loopback", "all"], default="all")
    p.add_argument("--size", type=int, default=65536, help="transfer size in bytes")
    p.add_argument("--duration", type=float, default=5.0, help="seconds per mode")
    args = p.parse_args()
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    dev = open_device()
    modes = {"out": STRESS_OUT, "in": STRESS_IN, "loopback": STRESS_LOOP}
    run = modes.items() if args.mode == "all" else [(args.mode, modes[args.mode])]
    try:
        for name, mode in run:
            total, elapsed, errors = stress(dev, mode, args.size, args.duration)
            mbps = (total * 8) / elapsed / 1e6 if elapsed else 0
            print(f"{name:10s} {total/1e6:8.2f} MB  {mbps:8.1f} Mbps  errors={errors}  {'PASS' if errors == 0 else 'FAIL'}")
    finally:
        usb.util.release_interface(dev, IFACE)
        usb.util.dispose_resources(dev)

if __name__ == "__main__":
    sys.exit(main())
