#!/usr/bin/env python3
"""
The size-scaling regs (C427, C456, etc) are OUTPUTS.
The actual DMA size is determined by the SCSI WRITE(16) CDB sector count,
which the USB/UAS hardware parses directly.

So the question is: how does the hardware know the transfer size?
Answer: it's in the SCSI CDB itself. The SCSI WRITE(16) opcode 0x8A 
has the transfer length in bytes 10-13 of the CDB.

tinygrad sends: struct.pack('>BBQIBB', 0x8A, 0, lba, sectors, 0, 0)

The hardware parses this CDB and configures the DMA internally.
We can't replicate this with raw bulk OUT — the hardware needs the 
SCSI framing to know the size.

BUT: on the handmade firmware, C403 controls the per-trigger DMA size
(up to 1KB). Can we do MULTIPLE triggers to write to sequential
SRAM addresses? The question is: does the SRAM pointer advance?

Let me test this carefully on the handmade firmware by doing:
  1. bulk_out 4KB of unique data
  2. C403=4, trigger -> copies first 1KB to F000-F3FF
  3. Check if another trigger copies next 1KB to F400-F7FF
  
Or does each trigger always start from 0x7000 offset 0?
"""
import sys
sys.path.insert(0, "/home/geohot/tinygrad")
from tinygrad.runtime.support.usb import ASM24Controller, ScsiWriteOp, WriteOp

ctrl = ASM24Controller()

def r(addr): return ctrl.read(addr, 1)[0]
def rn(addr, n): return ctrl.read(addr, n)
def w(addr, val): ctrl.write(addr, bytes([val]), ignore_cache=True)

# On stock firmware: what happens if we do ScsiWriteOp of different
# sizes and check the 0x7000 buffer size vs what lands in SRAM?

# The key question: for a 4KB scsi_write, does the hardware do
# ONE big DMA or MULTIPLE small DMAs internally?

# Let's check by reading C403 value after different size writes:
print("="*60)
print("C403 after different size scsi_writes (stock firmware)")
print("="*60)

for size in [512, 1024, 2048, 4096, 8192, 16384]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  c403 = r(0xC403)
  c427 = r(0xC427)
  print(f"  {size:5d}B: C403=0x{c403:02X} C427=0x{c427:02X}")
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

# Also check C420/C421
print("\nC420/C421 after different sizes:")
for size in [512, 1024, 2048, 4096, 16384, 65536]:
  ctrl.exec_ops([ScsiWriteOp(bytes(size), lba=0)])
  c420 = rn(0xC420, 8)
  c403 = r(0xC403)
  print(f"  {size:5d}B: C403=0x{c403:02X} C420-C427={c420.hex()}")
  ctrl.exec_ops([WriteOp(0x171, b'\xff\xff\xff', ignore_cache=True)])
  ctrl.exec_ops([WriteOp(0xCE6E, b'\x00\x00', ignore_cache=True)])

print("\nDone!")
