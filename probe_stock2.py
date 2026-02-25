#!/usr/bin/env python3
"""Probe stock firmware topology after tinygrad programs bridges."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller
from tinygrad.runtime.support.system import System

def main():
    print("Creating ASM24Controller (stock firmware)...")
    ctrl = ASM24Controller()
    print("Waiting 5s for full PCIe init...")
    time.sleep(5)

    # Run pci_setup_usb_bars like tinygrad does
    print("\nRunning pci_setup_usb_bars(gpu_bus=4)...")
    try:
        bars = System.pci_setup_usb_bars(ctrl, gpu_bus=4, mem_base=0x10000000, pref_mem_base=(32 << 30))
        print(f"Success! BARs: {bars}")
    except Exception as e:
        print(f"Error: {e}")

    # Now probe the full topology
    print("\n=== Post-setup bus topology ===")
    for bus in range(6):
        try:
            vid_did = ctrl.pcie_cfg_req(0x00, bus=bus, dev=0, fn=0, size=4)
            if vid_did == 0xFFFFFFFF: 
                print(f"  Bus {bus}: FFFFFFFF")
                continue
            vendor = vid_did & 0xFFFF
            device = (vid_did >> 16) & 0xFFFF
            
            class_rev = ctrl.pcie_cfg_req(0x08, bus=bus, dev=0, fn=0, size=4)
            cc = (class_rev >> 24) & 0xFF
            sc = (class_rev >> 16) & 0xFF
            
            hdr = ctrl.pcie_cfg_req(0x0E, bus=bus, dev=0, fn=0, size=1)
            is_bridge = bool(hdr & 0x7F == 0x01)
            
            info = f"  Bus {bus}: {vendor:04X}:{device:04X}  Class={cc:02X}:{sc:02X}  Hdr=0x{hdr:02X}"
            
            if is_bridge:
                bus_nums = ctrl.pcie_cfg_req(0x18, bus=bus, dev=0, fn=0, size=4)
                info += f"  [pri={bus_nums&0xFF} sec={(bus_nums>>8)&0xFF} sub={(bus_nums>>16)&0xFF}]"
            
            # Find PCIe cap
            cap_ptr = ctrl.pcie_cfg_req(0x34, bus=bus, dev=0, fn=0, size=1)
            ptr = cap_ptr
            while ptr and ptr != 0xFF:
                cap_hdr = ctrl.pcie_cfg_req(ptr, bus=bus, dev=0, fn=0, size=4)
                cap_id = cap_hdr & 0xFF
                if cap_id == 0x10:
                    pcie_cap = ctrl.pcie_cfg_req(ptr + 0x02, bus=bus, dev=0, fn=0, size=2)
                    dev_type = (pcie_cap >> 4) & 0xF
                    names = {0:"EP", 5:"UpSw", 6:"DnSw", 4:"Root"}
                    link_stat = ctrl.pcie_cfg_req(ptr + 0x12, bus=bus, dev=0, fn=0, size=2)
                    info += f"  PCIe={names.get(dev_type,str(dev_type))} Link=Gen{link_stat&0xF}x{(link_stat>>4)&0x3F}"
                    break
                ptr = (cap_hdr >> 8) & 0xFF
            
            print(info)
        except RuntimeError as e:
            if "Unsupported Request" in str(e):
                print(f"  Bus {bus}: UR")
            else:
                print(f"  Bus {bus}: {e}")

if __name__ == "__main__":
    main()
