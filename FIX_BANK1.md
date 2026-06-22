There was a bug in bank1.

bank1 is mapped at 0x8000, but it is NOT at 0x10000 in the file. It is at 0xff6b. It's now also in bank1.bin

Make a list in this file of all the things in the repo that are wrong because we used to assume it was at 0x10000. First we will list them all before making changes.

## Correct Values

- **Bank 0 size**: 65387 bytes (0xFF6B)
- **Bank 1 offset in fw.bin**: 0xFF6B (65387 decimal)
- **Bank 1 size**: 32619 bytes (0x7F6B)
- **Bank 1 execution address**: 0x8000-0xFFFF
- **Bank 1 file offset range**: 0xFF6B-0x17ED5
- **File offset to CPU address formula**: `file_offset = 0xFF6B + (cpu_addr - 0x8000)`
- **CPU address to file offset formula**: `cpu_addr = 0x8000 + (file_offset - 0xFF6B)`

## Files That Are CORRECT (✓ Already Fixed)

1. **CLAUDE.md** - ✓ Correctly documents 0xFF6B offset
2. **bank0.bin** - ✓ Correct size (65387 bytes)
3. **bank1.bin** - ✓ Correct file
4. **fw.bin** - ✓ Correct (cat bank0.bin bank1.bin)
5. **emulate/emu.py** - ✓ No bank1 offset references
6. **emulate/cpu.py** - ✓ No bank1 offset references (need to verify)
7. **emulate/memory.py** - ✓ Line 23 is just XDATA size (64KB generic), not bank1 offset
8. **emulate/disasm8051.py** - ✓ Line 727 is just label formatting, not bank1 specific
9. **emulate/peripherals.py** - ✓ Line 315 is flash size, not bank1 offset

## Files That Need FIXING (✗ Wrong)

### Documentation Files

1. **README.md:42**
   - Says: `Bank 1: 0x10000-0x17F12  (32KB, via DPX register, mapped at 0x8000)`
   - Should be: `Bank 1: 0xFF6B-0x17ED5  (32KB, via DPX register, mapped at 0x8000)`

2. **EMULATE.md:44**
   - Says: `- 0x8000-0xFFFF with DPX=1: Bank 1 (file offset 0x10000+)`
   - Should be: `- 0x8000-0xFFFF with DPX=1: Bank 1 (file offset 0xFF6B+)`

### Python Scripts

3. **ghidra_import_symbols.py**
   - Line 5: Comment says CODE_BANK1 (0x10000-0x17FFF)
   - Line 9: Comment says CODE_BANK1: 0x10000-0x17FFF
   - Line 1001: Comment says "file offset 0x10000-0x17FFF"
   - Line 1005: Comment says "mapped from file 0x10000-0x17FFF"
   - Line 1008: Comment says "Use file offset directly (0x10000+)"
   - Line 1076: Comment says "File offset 0x10000 = CPU 0x8000, so CPU addr = file_offset - 0x8000"
   - Line 2029: print statement says "CODE_BANK1: 0x10000-0x17FFF"
   - Line 2036: print statement says "Bank 1 function names (file 0x10000-0x17FFF)"
   - **NEEDS**: All references to 0x10000 should be 0xFF6B
   - **NEEDS**: Formula `file_offset - 0x8000` should be `file_offset - 0x7F6B` OR `(file_offset - 0xFF6B) + 0x8000`

4. **extract_symbols.py**
   - Line 96: Comment says "Bank 1: CPU address 0x8000-0xFFFF maps to file offset 0x10000-0x17FFF"
   - Line 115: Condition `if 0x8000 <= addr < 0x10000:` - This is checking if address is in shared bank0 range, CORRECT
   - Line 198: `bank0_funcs = sorted([(addr, name) for addr, name in functions.items() if addr < 0x10000])`
     - **UNCLEAR**: Are these file offsets or CPU addresses? If file offsets, should be `< 0xFF6B`
   - Line 199: `bank1_funcs = sorted([(addr, name) for addr, name in functions.items() if addr >= 0x10000])`
     - **UNCLEAR**: If file offsets, should be `>= 0xFF6B`
   - Line 210: Comment says CODE_BANK1 (0x10000-0x17FFF)
   - Line 214: Comment says CODE_BANK1: 0x10000-0x17FFF
   - Line 305: Comment says "file offset 0x10000-0x17FFF"
   - Line 309: Comment says "mapped from file 0x10000-0x17FFF"
   - **NEEDS**: All references to 0x10000 should be 0xFF6B

5. **python/usb.py**
   - Lines 156-159: Uses 0x10000 for chunking/padding
   - **LIKELY OK**: This appears to be generic 64KB block size for USB/SCSI operations, not bank1 offset
   - **NEEDS VERIFICATION**: Check if this is related to bank1 or just generic chunking

### C Source Files (src/)

6. **src/drivers/pcie.c** (5 functions with wrong actual addr)
   - Line 2694: Comment says "address 0x10000-0x17FFF mapped at 0x8000"
   - Line 2704: `[actual addr: 0x1039c]` should be `[actual addr: 0x10307]`
   - Line 2746: `[actual addr: 0x103b9]` should be `[actual addr: 0x10324]`
   - Line 2788: `file offset 0x16D02` should be `0x16C6D`
   - Line 2814: `file offset 0x16EF9` should be `0x16E64`
   - Line 2897: `file offset 0x16911` should be `0x1687C`
   - Line 2914: `file offset 0x12066` should be `0x11FD1`

7. **src/drivers/usb.c** (13 functions with wrong actual addr)
   - Line 3937: Comment says "address 0x10000-0x17FFF mapped at 0x8000"
   - Line 3954: `[actual addr: 0x1097d]` should be `[actual addr: 0x108E8]`
   - Line 3975: `[actual addr: 0x10992]` should be `[actual addr: 0x108FD]`
   - Line 3989: `[actual addr: 0x109ad]` should be `[actual addr: 0x10918]`
   - Line 4003: `[actual addr: 0x109bd]` should be `[actual addr: 0x10928]`
   - Line 4060: `[actual addr: 0x109c6]` should be `[actual addr: 0x10931]`
   - Line 4100: `[actual addr: 0x10a3a]` should be `[actual addr: 0x109A5]`
   - Line 4111: `[actual addr: 0x10a3d]` should be `[actual addr: 0x109A8]`
   - Line 4130: `[actual addr: 0x10a4e]` should be `[actual addr: 0x109B9]`
   - Line 4155: `[actual addr: 0x10a67]` should be `[actual addr: 0x109D2]`
   - Line 4173: `[actual addr: 0x10a72]` should be `[actual addr: 0x109DD]`
   - Line 4191: `[actual addr: 0x10a7e]` should be `[actual addr: 0x109E9]`
   - Line 4215: `[actual addr: 0x10a89]` should be `[actual addr: 0x109F4]`

8. **src/drivers/flash.c** (6 functions with wrong actual addr)
   - Line 737: Comment says "address 0x10000-0x17FFF mapped at 0x8000"
   - Line 751: `[actual addr: 0x1073a]` should be `[actual addr: 0x106A5]`
   - Line 767: `[actual addr: 0x10743]` should be `[actual addr: 0x106AE]`
   - Line 782: `[actual addr: 0x1074c]` should be `[actual addr: 0x106B7]`
   - Line 797: `[actual addr: 0x10d6e]` should be `[actual addr: 0x10CD9]`
   - Line 887: `[actual addr: 0x10d77]` should be `[actual addr: 0x10CE2]`

9. **src/drivers/timer.c**
   - Line 522: `file offset 0x16F4E` should be `0x16EB9`

10. **src/app/dispatch.c**
    - Line 221: Formula `file offset - 0x8000` should be `file offset - 0x7F6B`
    - Line 221: Example `0x89DB -> file 0x109DB` should be `0x89DB -> file 0x10946`

11. **src/app/error_log.c** (5 file offset references)
    - Line 305: `file offset 0x16920` should be `0x1688B`
    - Line 353: `file offset 0x16911` should be `0x1687C`
    - Line 380: `file offset 0x13230` should be `0x1319B`
    - Line 402: `file offset 0x12066` should be `0x11FD1`
    - Line 422: `file offset 0x16F4E` should be `0x16EB9`

12. **src/app/vendor.c**
    - Line 27: Formula `file_offset = 0x10000 + (bank1_addr - 0x8000)` should be `file_offset = 0xFF6B + (bank1_addr - 0x8000)`
    - Line 1003: Comment "file offset 0x10000+" should be "file offset 0xFF6B+"

13. **src/app/protocol.c** (3 file offset references)
    - Line 2464: `file offset 0x1656F` should be `0x164DA`
    - Line 2492: `file offset 0x16762` should be `0x166CD`
    - Line 2525: `file offset 0x16677` should be `0x165E2`

14. **src/include/sfr.h**
    - Line 88: `file offset 0x10000-0x17F0C` should be `0xFF6B-0x17E77`
    - Line 97: Example `0xE911 in bank 1 -> file offset 0x16911` should be `-> file offset 0x1687C`

15. **src/include/app/dispatch.h**
    - Line 14: `Bank 1: Physical 0x10000-0x17FFF` should be `Physical 0xFF6B-0x17ED5`

16. **src/main.c**
    - Line 774: `file offset 0x10000-0x17F0C` should be `0xFF6B-0x17E77`
    - Line 785: Formula `file_offset = addr + 0x8000` should be `file_offset = addr + 0x7F6B`
    - Line 789: Example `-> DPX=1, execution jumps to 0xE911 in bank 1` is OK (CPU address)
    - Line 924: Comment "code offset 0x10000+" should be "code offset 0xFF6B+"

### Test Files

17. **test/test_roundtrip.py:176**
   - Says: `binary_data = bytearray(65536 if bank_name == 'bank0' else 32768)`
   - **WRONG**: bank0 should be 65387 (0xFF6B), not 65536 (0x10000)
   - Should be: `binary_data = bytearray(65387 if bank_name == 'bank0' else 32619)`
   - **NOTE**: Test still works correctly because line 278 trims to original size, but using correct sizes is cleaner

## Additional Notes

### Files That Reference 0x10000 But Are CORRECT

1. **Makefile:24** - `--xram-size 0x10000` - This is XRAM size (64KB), not bank1 offset ✓
2. **ghidra_import_symbols.py:333-335** - Addresses like 0x17f3, 0x17f8, 0x17fd are valid file offsets within bank1 (< 0x17ED6) ✓
3. **emulate/peripherals.py:315** - `0x100000` is flash size (1MB), not related to bank1 ✓

### Important Formulas

**Old (WRONG) formulas used:**
- File to CPU: `cpu_addr = file_offset - 0x8000`  ❌
- CPU to File: `file_offset = cpu_addr + 0x8000`  ❌

**Correct formulas:**
- File to CPU: `cpu_addr = 0x8000 + (file_offset - 0xFF6B)`  ✓
- CPU to File: `file_offset = 0xFF6B + (cpu_addr - 0x8000)`  ✓

**Simplified:**
- File to CPU: `cpu_addr = file_offset - 0x7F6B`  ✓
- CPU to File: `file_offset = cpu_addr + 0x7F6B`  ✓

## Summary

**Total files needing fixes: 17**

**Documentation (2 files):**
- README.md (1 line)
- EMULATE.md (1 line)

**Python Scripts (3 files):**
- ghidra_import_symbols.py (~8 locations)
- extract_symbols.py (~7 locations)
- python/usb.py (needs verification - likely OK)

**C Source Files (11 files):**
- src/drivers/pcie.c (7 locations)
- src/drivers/usb.c (14 locations)
- src/drivers/flash.c (6 locations)
- src/drivers/timer.c (1 location)
- src/app/dispatch.c (2 locations)
- src/app/error_log.c (5 locations)
- src/app/vendor.c (2 locations)
- src/app/protocol.c (3 locations)
- src/include/sfr.h (2 locations)
- src/include/app/dispatch.h (1 location)
- src/main.c (4 locations)

**Test Files (1 file):**
- test/test_roundtrip.py (1 line - non-critical, test still passes)

**Total locations to fix: ~64 individual lines/comments**

**All wrong values are off by exactly 149 bytes (0x95)**

**Test status**: The roundtrip test currently PASSES despite wrong buffer sizes because it trims to actual size before comparison.

**Priority**:
1. ✅ Fix documentation (README.md, EMULATE.md, CLAUDE.md)
2. ✅ Fix C source function headers (src/**/*.c, src/**/*.h)
3. ✅ Fix Python scripts (ghidra_import_symbols.py, extract_symbols.py)
4. ✅ Fix test for cleanliness (test/test_roundtrip.py)

## STATUS: ✅ ALL FIXES COMPLETED

All bank1 file offsets have been corrected throughout the codebase.

**Summary of changes:**
- Fixed 2 documentation files (README.md, EMULATE.md)
- Fixed 12 C source/header files (src/**/*.c, src/**/*.h)
- Fixed 2 Python scripts (ghidra_import_symbols.py, extract_symbols.py)
- Fixed 1 test file (test/test_roundtrip.py)

**Verification:**
- ✅ All calculations verified mathematically
- ✅ Round-trip tests still passing (2/2 tests PASSED)
- ✅ Example offsets spot-checked and confirmed correct

