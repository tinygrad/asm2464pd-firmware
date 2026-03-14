#!/usr/bin/env python3
"""Test extended XDATA addressing on stock firmware."""
import sys
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.support.usb import ASM24Controller, ReadOp, WriteOp

def main():
    ctrl = ASM24Controller()

    def read_reg(addr):
        try:
            result = ctrl.exec_ops([ReadOp(addr, 1)])
            return result[0][0] if result and result[0] else -1
        except:
            return -1

    # The dma_view starts at ctrl_addr=0xF000
    # Write patterns at different offsets
    print("=== Testing extended XDATA addressing ===")
    
    # Write pattern at offset 0 (addr = 0xF000)
    ctrl.exec_ops([WriteOp(0xF000, b'\xDE')])
    ctrl.exec_ops([WriteOp(0xF001, b'\xAD')])
    
    # Write pattern at offset 0x1000 (addr = 0x10000 — beyond 16-bit!)
    # Tinygrad encoding: addr = (0x10000 & 0x1FFFF) | 0x500000 = 0x510000
    # This should map to XDATA page 1, addr 0x0000
    ctrl.exec_ops([WriteOp(0x10000, b'\xBE')])
    ctrl.exec_ops([WriteOp(0x10001, b'\xEF')])

    # Read back
    print(f"  F000: 0x{read_reg(0xF000):02X} (expected 0xDE)")
    print(f"  F001: 0x{read_reg(0xF001):02X} (expected 0xAD)")
    print(f"  10000: 0x{read_reg(0x10000):02X} (expected 0xBE)")
    print(f"  10001: 0x{read_reg(0x10001):02X} (expected 0xEF)")
    
    # Does 0x10000 alias to 0x0000?
    print(f"  0000: 0x{read_reg(0x0000):02X} (same as 10000?)")
    print(f"  0001: 0x{read_reg(0x0001):02X} (same as 10001?)")

    # Also test: what's the actual 17-bit address layout?
    print("\n=== 17-bit address test ===")
    for test_addr in [0xF000, 0xFFFF, 0x10000, 0x10001, 0x1F000, 0x1FFFF]:
        v = read_reg(test_addr)
        xdata_addr = test_addr & 0xFFFF
        bank = (test_addr >> 16) & 1
        print(f"  addr=0x{test_addr:05X} (bank{bank} xdata=0x{xdata_addr:04X}): 0x{v:02X}")

if __name__ == "__main__":
    main()
