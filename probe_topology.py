#!/usr/bin/env python3
"""Probe PCIe bus topology behind the ASM2464PD clean firmware."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

def program_bridge(ctrl, bus, secondary, subordinate):
    """Program a bridge's bus numbers and open memory windows."""
    buses = (0 << 0) | (secondary << 8) | (subordinate << 16)
    ctrl.pcie_cfg_req(0x18, bus=bus, dev=0, fn=0, value=buses, size=4)
    ctrl.pcie_cfg_req(0x20, bus=bus, dev=0, fn=0, value=0x1000, size=2)  # mem base
    ctrl.pcie_cfg_req(0x22, bus=bus, dev=0, fn=0, value=0xFFFF, size=2)  # mem limit
    ctrl.pcie_cfg_req(0x24, bus=bus, dev=0, fn=0, value=0x0000, size=2)  # pref mem base
    ctrl.pcie_cfg_req(0x26, bus=bus, dev=0, fn=0, value=0xFFFF, size=2)  # pref mem limit
    ctrl.pcie_cfg_req(0x28, bus=bus, dev=0, fn=0, value=0x08, size=4)    # pref base upper
    ctrl.pcie_cfg_req(0x2C, bus=bus, dev=0, fn=0, value=0xFFFFFFFF, size=4)  # pref limit upper
    ctrl.pcie_cfg_req(0x04, bus=bus, dev=0, fn=0, value=0x07, size=1)  # IO|MEM|MASTER
    print(f"  Programmed bus {bus}: secondary={secondary}, subordinate={subordinate}")

def probe_bus(ctrl, bus, dev=0):
    """Try to read VID/DID from a bus/dev. Returns (vid, did, class, hdr_type) or None."""
    try:
        vid_did = ctrl.pcie_cfg_req(0x00, bus=bus, dev=dev, fn=0, size=4)
        vendor = vid_did & 0xFFFF
        device = (vid_did >> 16) & 0xFFFF
        class_rev = ctrl.pcie_cfg_req(0x08, bus=bus, dev=dev, fn=0, size=4)
        class_code = (class_rev >> 24) & 0xFF
        subclass = (class_rev >> 16) & 0xFF
        hdr = ctrl.pcie_cfg_req(0x0E, bus=bus, dev=dev, fn=0, size=1)
        is_bridge = bool(hdr & 0x01)
        print(f"  Bus {bus} Dev {dev}: {vendor:04X}:{device:04X}  Class={class_code:02X}:{subclass:02X}  Hdr=0x{hdr:02X} ({'Bridge' if is_bridge else 'Endpoint'})")
        if is_bridge:
            bus_nums = ctrl.pcie_cfg_req(0x18, bus=bus, dev=dev, fn=0, size=4)
            print(f"         Bus nums: primary={bus_nums&0xFF}, secondary={(bus_nums>>8)&0xFF}, subordinate={(bus_nums>>16)&0xFF}")
        return vendor, device, class_code, hdr
    except RuntimeError as e:
        if "Unsupported Request" in str(e):
            print(f"  Bus {bus} Dev {dev}: No device (UR)")
        else:
            print(f"  Bus {bus} Dev {dev}: Error: {e}")
        return None

def main():
    print("Creating ASM24Controller...")
    ctrl = ASM24Controller()
    print("Waiting 3s for PCIe link training...")
    time.sleep(3)
    
    b22b = ctrl.read(0xB22B, 1)[0]
    print(f"B22B (link width) = 0x{b22b:02X}\n")

    # Step 1: Probe bus 0
    print("=== Initial bus 0 probe ===")
    result = probe_bus(ctrl, 0)
    
    if result is None:
        print("Bus 0 not responding, aborting")
        return

    # Step 2: Program bus 0 bridge and walk down
    print("\n=== Programming bridges progressively ===")
    max_depth = 6
    for depth in range(max_depth):
        current_bus = depth
        next_bus = depth + 1
        print(f"\nProgramming bus {current_bus} → secondary={next_bus}, subordinate={max_depth}")
        program_bridge(ctrl, current_bus, next_bus, max_depth)
        
        # Probe what's at next_bus
        result = probe_bus(ctrl, next_bus)
        if result is None:
            print(f"\nNothing at bus {next_bus}. GPU search complete.")
            break
        
        vendor, device, class_code, hdr = result
        if not (hdr & 0x01):  # Not a bridge — this is an endpoint (GPU?)
            print(f"\n*** Found endpoint at bus {next_bus}: {vendor:04X}:{device:04X} ***")
            # Read more details
            try:
                bar0 = ctrl.pcie_cfg_req(0x10, bus=next_bus, dev=0, fn=0, size=4)
                print(f"  BAR0 = 0x{bar0:08X}")
                sub = ctrl.pcie_cfg_req(0x2C, bus=next_bus, dev=0, fn=0, size=4)
                print(f"  Subsystem: {(sub>>16)&0xFFFF:04X}:{sub&0xFFFF:04X}")
            except Exception as e:
                print(f"  (Error reading BARs: {e})")
            break
    else:
        print(f"\nReached max depth {max_depth} without finding endpoint")

if __name__ == "__main__":
    main()
