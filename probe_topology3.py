#!/usr/bin/env python3
"""Program both bridges and probe bus 2 for GPU."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

def program_bridge(ctrl, bus, secondary, subordinate):
    buses = (0 << 0) | (secondary << 8) | (subordinate << 16)
    ctrl.pcie_cfg_req(0x18, bus=bus, dev=0, fn=0, value=buses, size=4)
    ctrl.pcie_cfg_req(0x20, bus=bus, dev=0, fn=0, value=0x1000, size=2)
    ctrl.pcie_cfg_req(0x22, bus=bus, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x24, bus=bus, dev=0, fn=0, value=0x0000, size=2)
    ctrl.pcie_cfg_req(0x26, bus=bus, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x28, bus=bus, dev=0, fn=0, value=0x08, size=4)
    ctrl.pcie_cfg_req(0x2C, bus=bus, dev=0, fn=0, value=0xFFFFFFFF, size=4)
    ctrl.pcie_cfg_req(0x04, bus=bus, dev=0, fn=0, value=0x07, size=1)
    print(f"  Programmed bus {bus}: secondary={secondary}, subordinate={subordinate}")

def main():
    print("Creating ASM24Controller...")
    ctrl = ASM24Controller()
    print("Waiting 3s for PCIe link training...")
    time.sleep(3)
    
    b22b = ctrl.read(0xB22B, 1)[0]
    print(f"B22B (link width) = 0x{b22b:02X}")

    # Program bus 0 → secondary=1, subordinate=6
    print("\nProgramming bus 0...")
    program_bridge(ctrl, 0, 1, 6)

    # Program bus 1 → secondary=2, subordinate=6  
    print("Programming bus 1...")
    program_bridge(ctrl, 1, 2, 6)

    # Probe bus 2
    print("\nProbing bus 2...")
    for dev in range(32):
        try:
            vid_did = ctrl.pcie_cfg_req(0x00, bus=2, dev=dev, fn=0, size=4)
            vendor = vid_did & 0xFFFF
            device = (vid_did >> 16) & 0xFFFF
            if vid_did != 0xFFFFFFFF:
                print(f"  Bus 2 Dev {dev}: {vendor:04X}:{device:04X}")
                class_rev = ctrl.pcie_cfg_req(0x08, bus=2, dev=dev, fn=0, size=4)
                print(f"    Class: {(class_rev>>24)&0xFF:02X}:{(class_rev>>16)&0xFF:02X}")
                hdr = ctrl.pcie_cfg_req(0x0E, bus=2, dev=dev, fn=0, size=1)
                print(f"    Header: 0x{hdr:02X}")
        except RuntimeError as e:
            if "Unsupported Request" in str(e):
                if dev == 0:
                    print(f"  Bus 2 Dev {dev}: UR (no device)")
                break  # No more devices on this bus
            else:
                print(f"  Bus 2 Dev {dev}: {e}")
                break

    # Also check bus 1 link status after programming
    print("\nRe-reading bus 1 link status...")
    try:
        # PCIe cap at 0x80, link status at 0x80+0x12=0x92
        link_status = ctrl.pcie_cfg_req(0x92, bus=1, dev=0, fn=0, size=2)
        cur_speed = link_status & 0xF
        cur_width = (link_status >> 4) & 0x3F
        print(f"  Bus 1 Link Status: Speed={cur_speed} Width=x{cur_width}")
        
        link_ctrl2 = ctrl.pcie_cfg_req(0xB0, bus=1, dev=0, fn=0, size=2)
        print(f"  Bus 1 Link Control 2: 0x{link_ctrl2:04X}")
    except Exception as e:
        print(f"  Error: {e}")

    # Check some relevant MMIO registers
    print("\nRelevant MMIO registers:")
    for addr, name in [(0xB22B, "B22B link width"), (0xB296, "B296 status"), 
                        (0xB450, "B450"), (0xB455, "B455 link detect"),
                        (0xE762, "E762"), (0xE764, "E764"), (0xE765, "E765")]:
        val = ctrl.read(addr, 1)[0]
        print(f"  {name} = 0x{val:02X}")

if __name__ == "__main__":
    main()
