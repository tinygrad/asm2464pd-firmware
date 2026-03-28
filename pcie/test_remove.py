#!/usr/bin/env python3
"""Test removing each numbered write from pcie_bringup_min.py one at a time."""
import subprocess, sys, time

# Each removable write with its line content. We test by commenting it out.
# Format: (line_number, description)
# We skip polls/loops and focus on individual writes.

WRITES = [
    (57, "C656 3.3V enable"),
    (58, "C65B PHY mode"),
    (59, "C659 12V enable"),
    (62, "B455 clear link detect"),
    (63, "B455 arm link detect"),
    (64, "B2D5 config routing"),
    (65, "B296 reset TLP engine"),
    (66, "B210-B21B clear header"),
    (67, "B210 FMT_TYPE CfgWr0"),
    (68, "B213 TLP_CTRL"),
    (69, "B217 BYTE_EN"),
    (70, "B216 TLP_LENGTH"),
    (71, "B218 addr0"),
    (72, "B219 addr1"),
    (73, "B21A addr2"),
    (74, "B21B addr3"),
    (75, "B220 data0"),
    (76, "B221 data1"),
    (77, "B222 data2"),
    (78, "B223 data3"),
    (79, "B296 clear error"),
    (80, "B296 clear completion"),
    (81, "B296 arm busy"),
    (82, "B254 trigger TLP"),
    (86, "B296 clear busy"),
    (89, "B480 deassert PERST"),
    (91, "CA06 Gen3 mode"),
    (92, "B403 enable tunnel"),
    (94, "B402 clear bit1"),
    (95, "B402 clear bit1 dup"),
    (103, "B401 tunnel pulse set"),
    (104, "B401 tunnel pulse clr"),
    (106, "B436 low nibble"),
    (109, "B436 high nibble"),
    (112, "E710 all lanes EQ"),
    (113, "E751 equalization"),
    (115, "E764 set bit3"),
    (116, "E764 clear bit2"),
    (117, "E764 clear bit0"),
    (118, "E764 set bit1"),
    (123, "B430 clear tunnel"),
    (124, "bank1 6025 TLP route"),
    (125, "B455 clear detect2"),
    (126, "B455 arm detect2"),
    (127, "B2D5 config route2"),
    (128, "B296 reset TLP2"),
]

def test_skip(line_num, desc):
    """Read the file, comment out one line, run bringup+probe, return pass/fail."""
    with open("pcie/pcie_bringup_min.py") as f:
        lines = f.readlines()

    original = lines[line_num - 1]
    lines[line_num - 1] = f"        pass  # SKIPPED: {original.strip()}\n"

    with open("/tmp/bringup_test.py", "w") as f:
        f.writelines(lines)

    # flash
    r = subprocess.run(["make", "-C", "handmade", "flash"], capture_output=True, timeout=60)
    if r.returncode != 0:
        return None  # flash failed

    # bringup
    r = subprocess.run(["python3", "/tmp/bringup_test.py"], capture_output=True, timeout=30)
    if r.returncode != 0:
        return False

    # probe
    r = subprocess.run(["python3", "pcie/pcie_probe.py"], capture_output=True, timeout=30)
    return r.returncode == 0


results = []
for line_num, desc in WRITES:
    print(f"Testing skip line {line_num}: {desc}...", end=" ", flush=True)
    try:
        ok = test_skip(line_num, desc)
        status = "PASS" if ok else ("FAIL" if ok is not None else "ERR")
    except Exception as e:
        status = f"ERR({e})"
    print(status)
    results.append((line_num, desc, status))

print("\n=== Summary ===")
for line_num, desc, status in results:
    flag = "  CAN REMOVE" if status == "PASS" else ""
    print(f"  line {line_num:3d}: {desc:30s} {status}{flag}")
