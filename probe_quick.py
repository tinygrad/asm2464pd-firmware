#!/usr/bin/env python3
"""Quick PCIe topology probe - matches stock firmware's pci_setup_usb_bars flow."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

def main():
    print("Creating ASM24Controller...")
    ctrl = ASM24Controller()
    print("Waiting 6s for PCIe link training (phase2 now enabled)...")
    time.sleep(6)

    b22b = ctrl.read(0xB22B, 1)[0]
    print(f"B22B = 0x{b22b:02X}")

    # Probe bus 0 first (no bridge programming needed)
    print("\n=== Bus 0 (before programming) ===")
    try:
        vid = ctrl.pcie_cfg_req(0x00, bus=0, dev=0, fn=0, size=4)
        bus_nums = ctrl.pcie_cfg_req(0x18, bus=0, dev=0, fn=0, size=4)
        print(f"  VID:DID = {vid&0xFFFF:04X}:{(vid>>16)&0xFFFF:04X}")
        print(f"  Bus nums: pri={bus_nums&0xFF} sec={(bus_nums>>8)&0xFF} sub={(bus_nums>>16)&0xFF}")
        
        # PCIe cap link status (cap at 0x80, link status at 0x80+0x12=0x92)
        link_stat = ctrl.pcie_cfg_req(0x92, bus=0, dev=0, fn=0, size=2)
        print(f"  Bus 0 Link Status: Gen{link_stat&0xF} x{(link_stat>>4)&0x3F}")
    except Exception as e:
        print(f"  Error: {e}")

    # Program bus 0 to forward to bus 1-6
    print("\n=== Programming bus 0 (sec=1, sub=6) ===")
    buses = (0 << 0) | (1 << 8) | (6 << 16)
    ctrl.pcie_cfg_req(0x18, bus=0, dev=0, fn=0, value=buses, size=4)
    ctrl.pcie_cfg_req(0x20, bus=0, dev=0, fn=0, value=0x1000, size=2)
    ctrl.pcie_cfg_req(0x22, bus=0, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x24, bus=0, dev=0, fn=0, value=0x0000, size=2)
    ctrl.pcie_cfg_req(0x26, bus=0, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x28, bus=0, dev=0, fn=0, value=0x08, size=4)
    ctrl.pcie_cfg_req(0x2C, bus=0, dev=0, fn=0, value=0xFFFFFFFF, size=4)
    ctrl.pcie_cfg_req(0x04, bus=0, dev=0, fn=0, value=0x07, size=1)

    # Probe bus 1
    print("\n=== Bus 1 ===")
    try:
        vid = ctrl.pcie_cfg_req(0x00, bus=1, dev=0, fn=0, size=4)
        print(f"  VID:DID = {vid&0xFFFF:04X}:{(vid>>16)&0xFFFF:04X}")
        link_stat = ctrl.pcie_cfg_req(0x92, bus=1, dev=0, fn=0, size=2)
        print(f"  Bus 1 Link Status: Gen{link_stat&0xF} x{(link_stat>>4)&0x3F}")
        bus_nums = ctrl.pcie_cfg_req(0x18, bus=1, dev=0, fn=0, size=4)
        print(f"  Bus nums: pri={bus_nums&0xFF} sec={(bus_nums>>8)&0xFF} sub={(bus_nums>>16)&0xFF}")
    except Exception as e:
        print(f"  Error: {e}")

    # Program bus 1 to forward to bus 2-6
    print("\n=== Programming bus 1 (sec=2, sub=6) ===")
    buses = (0 << 0) | (2 << 8) | (6 << 16)
    ctrl.pcie_cfg_req(0x18, bus=1, dev=0, fn=0, value=buses, size=4)
    ctrl.pcie_cfg_req(0x20, bus=1, dev=0, fn=0, value=0x1000, size=2)
    ctrl.pcie_cfg_req(0x22, bus=1, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x24, bus=1, dev=0, fn=0, value=0x0000, size=2)
    ctrl.pcie_cfg_req(0x26, bus=1, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x28, bus=1, dev=0, fn=0, value=0x08, size=4)
    ctrl.pcie_cfg_req(0x2C, bus=1, dev=0, fn=0, value=0xFFFFFFFF, size=4)
    ctrl.pcie_cfg_req(0x04, bus=1, dev=0, fn=0, value=0x07, size=1)

    # Probe bus 2
    print("\n=== Bus 2 ===")
    try:
        vid = ctrl.pcie_cfg_req(0x00, bus=2, dev=0, fn=0, size=4)
        print(f"  VID:DID = {vid&0xFFFF:04X}:{(vid>>16)&0xFFFF:04X}")
        class_rev = ctrl.pcie_cfg_req(0x08, bus=2, dev=0, fn=0, size=4)
        print(f"  Class: {(class_rev>>24)&0xFF:02X}:{(class_rev>>16)&0xFF:02X}")
        hdr = ctrl.pcie_cfg_req(0x0E, bus=2, dev=0, fn=0, size=1)
        print(f"  Header type: 0x{hdr:02X}")
    except RuntimeError as e:
        print(f"  Error: {e}")
    except Exception as e:
        print(f"  Exception: {e}")

    # Also read some diagnostic registers via E4
    print("\n=== Diagnostics ===")
    for addr, name in [(0x0F00, "Phase"), (0x0F20, "E762_dec"), (0x0F23, "link_path"), 
                        (0x0F24, "B22B_dec"), (0x0F27, "B22B_post"), (0x0F2A, "E762_post")]:
        val = ctrl.read(addr, 1)[0]
        print(f"  {name} = 0x{val:02X}")

if __name__ == "__main__":
    main()
