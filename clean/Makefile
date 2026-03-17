# Minimal "hello" firmware Makefile

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
CFLAGS = -mmcs51 --model-large --opt-code-size
CFLAGS += --std-c99
CFLAGS += -I$(SRC_DIR)
CFLAGS += -I../src/include
CFLAGS += --no-xinit-opt

# Memory model settings
LDFLAGS = --code-loc 0x0000 --code-size 0x10000
LDFLAGS += --xram-loc 0x0800 --xram-size 0xF800
LDFLAGS += --iram-size 0x100
LDFLAGS += --no-std-crt0
LDFLAGS += -Wl-bXSEG=0x0800

# Directories
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
CRT0_SRC = $(SRC_DIR)/crt0.s
MAIN_SRC = $(SRC_DIR)/main.c
PROXY_SRC = $(SRC_DIR)/proxy.c

# Object files
CRT0_OBJ = $(OBJ_DIR)/crt0.rel
MAIN_OBJ = $(OBJ_DIR)/main.rel
PROXY_OBJ = $(OBJ_DIR)/proxy.rel
OBJS = $(CRT0_OBJ) $(MAIN_OBJ)

# Output files
TARGET = $(BUILD_DIR)/firmware
HEX = $(TARGET).ihx
BIN = $(TARGET).bin

# Proxy firmware output files
PROXY_TARGET = $(BUILD_DIR)/proxy
PROXY_HEX = $(PROXY_TARGET).ihx
PROXY_BIN = $(PROXY_TARGET).bin

.PHONY: all clean flash wrapped proxy flash-proxy

all: $(BIN)
	@echo "Built: $(BIN)"
	@ls -la $(BIN)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Assemble startup code
$(CRT0_OBJ): $(CRT0_SRC) | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0.rel $(CRT0_OBJ)
	-mv $(SRC_DIR)/crt0.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0.sym $(OBJ_DIR)/ 2>/dev/null || true

# Compile main source
$(MAIN_OBJ): $(MAIN_SRC) | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files to Intel HEX
$(HEX): $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

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

# Add ASM2464 firmware wrapper (body_len + body + magic + checksum + crc)
$(BUILD_DIR)/firmware_wrapped.bin: $(BIN)
	@echo "Wrapping firmware with ASM2464 header..."
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

wrapped: $(BUILD_DIR)/firmware_wrapped.bin

# Flash to device
flash: $(BUILD_DIR)/firmware_wrapped.bin
	@echo "Resetting to bootloader..."
	@python3 ../ftdi_debug.py -bn
	@echo "Flashing firmware to device..."
	@python3 ../flash.py $<
	@echo "Resetting device..."
	@python3 ../ftdi_debug.py -rt 5

# Show hex dump
hex: $(BIN)
	xxd $(BIN) | head -64

# Disassemble
disasm: $(BIN)
	r2 -a 8051 -q -c 'pd 100' $(BIN) 2>/dev/null || echo "radare2 not available"

# ============================================
# Proxy firmware targets
# ============================================

# Compile proxy source
$(PROXY_OBJ): $(PROXY_SRC) | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link proxy firmware
$(PROXY_HEX): $(CRT0_OBJ) $(PROXY_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $(CRT0_OBJ) $(PROXY_OBJ) -o $@

# Convert proxy HEX to binary
$(PROXY_BIN): $(PROXY_HEX)
	@echo "Converting proxy HEX to BIN..."
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

# Wrap proxy firmware
$(BUILD_DIR)/proxy_wrapped.bin: $(PROXY_BIN)
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

# ============================================
# Clean
# ============================================

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
