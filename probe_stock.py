#!/usr/bin/env python3
"""Probe PCIe topology with stock firmware to understand expected bus layout."""
import sys, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

def main():
    print("Creating ASM24Controller (stock firmware)...")
    ctrl = ASM24Controller()
    print("Waiting 5s for full PCIe init...")
    time.sleep(5)

    # Walk the bus topology that tinygrad expects (gpu_bus=4)
    print("\n=== Probing bus 0-5 config space ===")
    for bus in range(6):
        for dev in range(2):
            try:
                vid_did = ctrl.pcie_cfg_req(0x00, bus=bus, dev=dev, fn=0, size=4)
                if vid_did == 0xFFFFFFFF:
                    if dev == 0: print(f"  Bus {bus} Dev {dev}: FFFFFFFF (no device)")
                    continue
                vendor = vid_did & 0xFFFF
                device = (vid_did >> 16) & 0xFFFF
                print(f"  Bus {bus} Dev {dev}: {vendor:04X}:{device:04X}", end="")

                class_rev = ctrl.pcie_cfg_req(0x08, bus=bus, dev=dev, fn=0, size=4)
                cc = (class_rev >> 24) & 0xFF
                sc = (class_rev >> 16) & 0xFF
                print(f"  Class={cc:02X}:{sc:02X}", end="")

                hdr = ctrl.pcie_cfg_req(0x0E, bus=bus, dev=dev, fn=0, size=1)
                is_bridge = bool(hdr & 0x7F == 0x01)
                print(f"  Hdr=0x{hdr:02X}", end="")

                if is_bridge:
                    bus_nums = ctrl.pcie_cfg_req(0x18, bus=bus, dev=dev, fn=0, size=4)
                    pri = bus_nums & 0xFF
                    sec = (bus_nums >> 8) & 0xFF
                    sub = (bus_nums >> 16) & 0xFF
                    print(f"  [Bridge: pri={pri} sec={sec} sub={sub}]", end="")

                # PCIe cap at 0x50 or 0x80 - try to find it
                cap_ptr = ctrl.pcie_cfg_req(0x34, bus=bus, dev=dev, fn=0, size=1)
                ptr = cap_ptr
                while ptr and ptr != 0xFF:
                    cap_hdr = ctrl.pcie_cfg_req(ptr, bus=bus, dev=dev, fn=0, size=4)
                    cap_id = cap_hdr & 0xFF
                    if cap_id == 0x10:  # PCIe cap
                        pcie_cap = ctrl.pcie_cfg_req(ptr + 0x02, bus=bus, dev=dev, fn=0, size=2)
                        dev_type = (pcie_cap >> 4) & 0xF
                        type_names = {0: "EP", 1: "LegEP", 4: "RootPort", 5: "UpSwitch", 6: "DnSwitch", 7: "PCI-Bridge", 8: "PCIe-Bridge", 9: "RCIEP"}
                        print(f"  PCIeType={dev_type}({type_names.get(dev_type, '?')})", end="")
                        
                        link_stat = ctrl.pcie_cfg_req(ptr + 0x12, bus=bus, dev=dev, fn=0, size=2)
                        spd = link_stat & 0xF
                        wid = (link_stat >> 4) & 0x3F
                        print(f"  Link=Gen{spd}x{wid}", end="")
                        break
                    ptr = (cap_hdr >> 8) & 0xFF

                print()
            except RuntimeError as e:
                if "Unsupported Request" in str(e):
                    if dev == 0: print(f"  Bus {bus} Dev {dev}: UR")
                else:
                    if dev == 0: print(f"  Bus {bus} Dev {dev}: {e}")
                if dev == 0: break  # If dev 0 fails, don't try dev 1

if __name__ == "__main__":
    main()
