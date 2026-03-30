#!/usr/bin/env python3
"""Diff stock_after4k vs handmade register dumps. Focus on finding the address register."""
import json

with open("dump_stock_after4k.json") as f: stock = json.load(f)
with open("dump_handmade.json") as f: handmade = json.load(f)

# Skip known-noisy areas
SKIP_RANGES = [
  (0x0100, 0x0600),  # RAM/stack
  (0x07E0, 0x0800),  # buffer area  
  (0x0900, 0x1000),  # RAM
]

def in_skip(addr):
  for s, e in SKIP_RANGES:
    if s <= addr < e: return True
  return False

all_addrs = sorted(set(stock.keys()) | set(handmade.keys()))

print("Registers different between stock(4KB write) and handmade(dma_explore):")
print(f"{'ADDR':>8s}  {'STOCK':>6s}  {'HANDM':>6s}  {'DELTA':>6s}")
print("-" * 35)

for addr_s in all_addrs:
  addr = int(addr_s, 16)
  if in_skip(addr): continue
  
  s = stock.get(addr_s, 0)
  h = handmade.get(addr_s, 0)
  if s == -1 or h == -1: continue
  if s != h:
    print(f"  {addr_s}    0x{s:02X}    0x{h:02X}    {'+' if s > h else '-'}{abs(s-h):02X}")
