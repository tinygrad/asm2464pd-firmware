#!/usr/bin/env python3
"""Deep probe of PCIe config space at bus 0 and bus 1."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

def dump_config(ctrl, bus, dev=0, start=0, end=0x100):
    """Dump config space range."""
    data = []
    for off in range(start, end, 4):
        try:
            val = ctrl.pcie_cfg_req(off, bus=bus, dev=dev, fn=0, size=4)
            data.append((off, val))
        except RuntimeError:
            data.append((off, None))
    return data

def main():
    print("Creating ASM24Controller...")
    ctrl = ASM24Controller()
    print("Waiting 3s for PCIe link training...")
    time.sleep(3)
    
    b22b = ctrl.read(0xB22B, 1)[0]
    print(f"B22B (link width) = 0x{b22b:02X}\n")

    # Dump bus 0 config space (first 256 bytes)
    print("=== Bus 0 Dev 0 Config Space ===")
    for off, val in dump_config(ctrl, 0, 0, 0, 0x80):
        if val is not None:
            print(f"  [{off:03X}] {val:08X}", end="")
            if off == 0x00: print(f"  VID={val&0xFFFF:04X} DID={(val>>16)&0xFFFF:04X}", end="")
            if off == 0x08: print(f"  Class={(val>>24)&0xFF:02X}:{(val>>16)&0xFF:02X}:{(val>>8)&0xFF:02X} Rev={val&0xFF:02X}", end="")
            if off == 0x18: print(f"  Pri={val&0xFF} Sec={(val>>8)&0xFF} Sub={(val>>16)&0xFF}", end="")
            if off == 0x34: print(f"  Cap ptr=0x{val&0xFF:02X}", end="")
            print()
        else:
            print(f"  [{off:03X}] UR")

    # Walk capability list
    print("\n=== Bus 0 Capability List ===")
    try:
        cap_ptr = ctrl.pcie_cfg_req(0x34, bus=0, dev=0, fn=0, size=1)
        while cap_ptr and cap_ptr != 0xFF:
            cap_hdr = ctrl.pcie_cfg_req(cap_ptr, bus=0, dev=0, fn=0, size=4)
            cap_id = cap_hdr & 0xFF
            next_ptr = (cap_hdr >> 8) & 0xFF
            cap_names = {0x01: "Power Management", 0x05: "MSI", 0x10: "PCIe", 0x11: "MSI-X"}
            name = cap_names.get(cap_id, f"Unknown(0x{cap_id:02X})")
            print(f"  Cap @ 0x{cap_ptr:02X}: {name} (ID=0x{cap_id:02X}), next=0x{next_ptr:02X}")
            
            if cap_id == 0x10:  # PCIe cap
                pcie_cap = ctrl.pcie_cfg_req(cap_ptr + 0x02, bus=0, dev=0, fn=0, size=2)
                dev_type = (pcie_cap >> 4) & 0xF
                type_names = {0: "Endpoint", 1: "Legacy EP", 4: "Root Port", 5: "Upstream Switch", 6: "Downstream Switch", 7: "PCIe-PCI Bridge"}
                print(f"    PCIe Cap: DevType={dev_type} ({type_names.get(dev_type, 'Unknown')})")
                
                link_cap = ctrl.pcie_cfg_req(cap_ptr + 0x0C, bus=0, dev=0, fn=0, size=4)
                max_speed = link_cap & 0xF
                max_width = (link_cap >> 4) & 0x3F
                print(f"    Link Cap: MaxSpeed={max_speed} MaxWidth=x{max_width}")
                
                link_status = ctrl.pcie_cfg_req(cap_ptr + 0x12, bus=0, dev=0, fn=0, size=2)
                cur_speed = link_status & 0xF
                cur_width = (link_status >> 4) & 0x3F
                print(f"    Link Status: Speed={cur_speed} Width=x{cur_width}")
            
            cap_ptr = next_ptr
    except Exception as e:
        print(f"  Error walking caps: {e}")

    # Now program bus 0 and try to enumerate bus 1  
    print("\n=== Programming bus 0, probing bus 1 ===")
    # Program bus 0 bridge
    buses = (0 << 0) | (1 << 8) | (6 << 16)
    ctrl.pcie_cfg_req(0x18, bus=0, dev=0, fn=0, value=buses, size=4)
    ctrl.pcie_cfg_req(0x20, bus=0, dev=0, fn=0, value=0x1000, size=2)
    ctrl.pcie_cfg_req(0x22, bus=0, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x24, bus=0, dev=0, fn=0, value=0x0000, size=2)
    ctrl.pcie_cfg_req(0x26, bus=0, dev=0, fn=0, value=0xFFFF, size=2)
    ctrl.pcie_cfg_req(0x28, bus=0, dev=0, fn=0, value=0x08, size=4)
    ctrl.pcie_cfg_req(0x2C, bus=0, dev=0, fn=0, value=0xFFFFFFFF, size=4)
    ctrl.pcie_cfg_req(0x04, bus=0, dev=0, fn=0, value=0x07, size=1)
    
    # Dump bus 1 config (first 128 bytes)
    print("\n=== Bus 1 Dev 0 Config Space ===")
    for off, val in dump_config(ctrl, 1, 0, 0, 0x80):
        if val is not None:
            print(f"  [{off:03X}] {val:08X}", end="")
            if off == 0x00: print(f"  VID={val&0xFFFF:04X} DID={(val>>16)&0xFFFF:04X}", end="")
            if off == 0x08: print(f"  Class={(val>>24)&0xFF:02X}:{(val>>16)&0xFF:02X}:{(val>>8)&0xFF:02X} Rev={val&0xFF:02X}", end="")
            if off == 0x0C: print(f"  CacheLine={(val>>0)&0xFF} LatTimer={(val>>8)&0xFF} HdrType={(val>>16)&0xFF:02X}", end="")
            if off == 0x18: print(f"  Pri={val&0xFF} Sec={(val>>8)&0xFF} Sub={(val>>16)&0xFF}", end="")
            if off == 0x34: print(f"  Cap ptr=0x{val&0xFF:02X}", end="")
            print()
        else:
            print(f"  [{off:03X}] UR")

    # Walk bus 1 capability list
    print("\n=== Bus 1 Capability List ===")
    try:
        cap_ptr = ctrl.pcie_cfg_req(0x34, bus=1, dev=0, fn=0, size=1)
        while cap_ptr and cap_ptr != 0xFF:
            cap_hdr = ctrl.pcie_cfg_req(cap_ptr, bus=1, dev=0, fn=0, size=4)
            cap_id = cap_hdr & 0xFF
            next_ptr = (cap_hdr >> 8) & 0xFF
            cap_names = {0x01: "Power Management", 0x05: "MSI", 0x10: "PCIe", 0x11: "MSI-X"}
            name = cap_names.get(cap_id, f"Unknown(0x{cap_id:02X})")
            print(f"  Cap @ 0x{cap_ptr:02X}: {name} (ID=0x{cap_id:02X}), next=0x{next_ptr:02X}")
            
            if cap_id == 0x10:  # PCIe cap
                pcie_cap = ctrl.pcie_cfg_req(cap_ptr + 0x02, bus=1, dev=0, fn=0, size=2)
                dev_type = (pcie_cap >> 4) & 0xF
                type_names = {0: "Endpoint", 1: "Legacy EP", 4: "Root Port", 5: "Upstream Switch", 6: "Downstream Switch", 7: "PCIe-PCI Bridge"}
                print(f"    PCIe Cap: DevType={dev_type} ({type_names.get(dev_type, 'Unknown')})")
                
                link_cap = ctrl.pcie_cfg_req(cap_ptr + 0x0C, bus=1, dev=0, fn=0, size=4)
                max_speed = link_cap & 0xF
                max_width = (link_cap >> 4) & 0x3F
                print(f"    Link Cap: MaxSpeed={max_speed} MaxWidth=x{max_width}")
                
                link_status = ctrl.pcie_cfg_req(cap_ptr + 0x12, bus=1, dev=0, fn=0, size=2)
                cur_speed = link_status & 0xF
                cur_width = (link_status >> 4) & 0x3F
                print(f"    Link Status: Speed={cur_speed} Width=x{cur_width}")
            
            cap_ptr = next_ptr
    except Exception as e:
        print(f"  Error walking caps: {e}")

if __name__ == "__main__":
    main()
