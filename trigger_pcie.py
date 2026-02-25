#!/usr/bin/env python3
"""Trigger PCIe init (send TUR + wait) and read diagnostics."""
import sys, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller

print("Creating ASM24Controller (triggers TUR + E5 writes)...")
ctrl = ASM24Controller()
print("Waiting 10s for full PCIe init with phase2...")
time.sleep(10)

# Read diagnostics
print("\n=== Diagnostics ===")
for addr, name in [(0x0F00, "Phase"), (0x0F20, "E762_dec"), (0x0F21, "B450_dec"),
                    (0x0F22, "E764_dec"), (0x0F23, "link_path"), (0x0F24, "B22B_dec"),
                    (0x0F25, "E765_dec"), (0x0F26, "E710_dec"),
                    (0x0F27, "B22B_post"), (0x0F28, "B450_post"),
                    (0x0F29, "E765_post"), (0x0F2A, "E762_post")]:
    val = ctrl.read(addr, 1)[0]
    print(f"  {name} = 0x{val:02X}")

# Read current state
print("\n=== Current state ===")
for addr, name in [(0xB22B, "B22B"), (0xB296, "B296"), (0xB434, "B434"),
                    (0xB432, "B432"), (0xE762, "E762"), (0xE764, "E764"),
                    (0xE765, "E765"), (0xB431, "B431"), (0xCA06, "CA06"),
                    (0xCA81, "CA81"), (0xB403, "B403")]:
    val = ctrl.read(addr, 1)[0]
    print(f"  {name} = 0x{val:02X}")

print("\nDone. Read UART for detailed phase2 trace.")
