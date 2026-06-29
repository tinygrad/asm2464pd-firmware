# MMIO proxy firmware Makefile
#
# Builds the standalone proxy firmware used by emulate/emu.py --proxy to run
# the original firmware in an emulator while proxying MMIO over UART to real
# hardware. proxy.c and crt0.s are self-contained (no external headers).
#
# proxy.c does side-effecting MMIO reads, so it must NOT be built with
# --allow-unsafe-read (speculative reads).

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
CFLAGS = -mmcs51 --model-large --opt-code-size
CFLAGS += --std-c99
CFLAGS += --no-xinit-opt

# Memory model settings
LDFLAGS = --code-loc 0x0000 --code-size 0x10000
LDFLAGS += --xram-loc 0x0000 --xram-size 0x10000
LDFLAGS += --iram-size 0x100

# Directories
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
CRT0_SRC = $(SRC_DIR)/crt0.s
PROXY_SRC = $(SRC_DIR)/proxy.c

# Object files
CRT0_OBJ = $(OBJ_DIR)/crt0.rel
PROXY_OBJ = $(OBJ_DIR)/proxy.rel

# Output files
TARGET = $(BUILD_DIR)/proxy
HEX = $(TARGET).ihx
BIN = $(TARGET).bin

.PHONY: all proxy flash-proxy clean

all: proxy

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Assemble startup code (full ISR vector table)
$(CRT0_OBJ): $(CRT0_SRC) | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0.rel $(CRT0_OBJ)
	-mv $(SRC_DIR)/crt0.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0.sym $(OBJ_DIR)/ 2>/dev/null || true

# Compile proxy source
$(PROXY_OBJ): $(PROXY_SRC) | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link proxy firmware to Intel HEX
$(HEX): $(CRT0_OBJ) $(PROXY_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $(CRT0_OBJ) $(PROXY_OBJ) -o $@

# Convert Intel HEX to binary
$(BIN): $(HEX)
	@echo "Converting HEX to BIN..."
	@if command -v objcopy > /dev/null 2>&1; then \
		objcopy -I ihex -O binary $< $@; \
	elif command -v sdobjcopy > /dev/null 2>&1; then \
		sdobjcopy -I ihex -O binary $< $@; \
	else \
		python3 -c "import sys; \
data = bytearray(0x10000); \
for line in open('$<'): \
    if line[0] != ':': continue; \
    n = int(line[1:3], 16); \
    addr = int(line[3:7], 16); \
    t = int(line[7:9], 16); \
    if t == 0: \
        for i in range(n): \
            data[addr+i] = int(line[9+i*2:11+i*2], 16); \
open('$@', 'wb').write(bytes(data).rstrip(b'\x00'))"; \
	fi

# Wrap proxy firmware with ASM2464 header (body_len + body + magic + checksum + crc)
$(BUILD_DIR)/proxy_wrapped.bin: $(BIN)
	@echo "Wrapping proxy firmware with ASM2464 header..."
	@python3 -c "\
import zlib; \
body = open('$<', 'rb').read(); \
body_len = len(body).to_bytes(4, 'little'); \
checksum = bytes([sum(body) & 0xFF]); \
crc = zlib.crc32(body).to_bytes(4, 'little'); \
magic = bytes([0xA5]); \
open('$@', 'wb').write(body_len + body + magic + checksum + crc)"
	@echo "Built: $@"
	@ls -la $@

# Build proxy firmware
proxy: $(BUILD_DIR)/proxy_wrapped.bin
	@echo "Proxy firmware built successfully"

# Flash proxy firmware to device
flash-proxy: $(BUILD_DIR)/proxy_wrapped.bin
	@echo "Resetting to bootloader..."
	@python3 ../ftdi_debug.py -bn
	@echo "Flashing proxy firmware to device..."
	@python3 ../flash.py $<
	@echo "Resetting device..."
	@python3 ../ftdi_debug.py -rt 1

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
