# Handmade firmware Makefile

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
CFLAGS = -mmcs51 --model-small --opt-code-speed --peep-asm --allow-unsafe-read
CFLAGS += --std-c99
CFLAGS += -I$(SRC_DIR)
CFLAGS += --no-xinit-opt
CFLAGS += $(CFLAGS_EXTRA)

# Memory model settings
LDFLAGS = --code-loc 0x0000 --code-size 0x10000
# Place SDCC's auto-XDATA segment in the chip's low CM RAM. The USB4 lane-bond
# path can stall on the 0x88xx scratch area once the tunnel is active, while
# 0x0B60-0x0BFF stays valid and clear of the mailbox/FSM state.
LDFLAGS += --xram-loc 0x0B60 --xram-size 0xA0
LDFLAGS += --iram-size 0x100

# Directories
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
CRT0_SRC = $(SRC_DIR)/crt0.s
MAIN_SRC = $(SRC_DIR)/main.c

# Object files
CRT0_OBJ = $(OBJ_DIR)/crt0.rel
MAIN_OBJ = $(OBJ_DIR)/main.rel
OBJS = $(CRT0_OBJ) $(MAIN_OBJ)

# Output files
TARGET = $(BUILD_DIR)/firmware
HEX = $(TARGET).ihx
BIN = $(TARGET).bin

.PHONY: all clean flash wrapped uflash nflash bootstub bootstub-flash image

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

# Compile main source. Header deps generated as a side effect via `sdcc -MM`
# (one extra preprocessor pass; SDCC has no -MMD/-MD).
$(MAIN_OBJ): $(MAIN_SRC) | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
	@$(CC) $(CFLAGS) -MM $< 2>/dev/null | sed 's|^[^:]*:|$@:|' > $(OBJ_DIR)/main.d

-include $(OBJ_DIR)/main.d

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

# ===== Bootstub =====
# Separate binary at CODE 0x0000-0x23FF. Loads userfw from flash to CODE 0x2400+.

BOOTSTUB_CFLAGS = -mmcs51 --model-large --opt-code-size --std-c99 --no-xinit-opt -I$(SRC_DIR)
BOOTSTUB_LDFLAGS = --code-loc 0x0000 --code-size 0x2400 --xram-loc 0x0000 --xram-size 0x5FF8 --iram-size 0x100

BOOTSTUB_CRT0_OBJ = $(OBJ_DIR)/crt0_bootstub.rel
BOOTSTUB_OBJ = $(OBJ_DIR)/bootstub.rel
BOOTSTUB_HEX = $(BUILD_DIR)/bootstub.ihx
BOOTSTUB_BIN = $(BUILD_DIR)/bootstub.bin
BOOTSTUB_WRAPPED = $(BUILD_DIR)/bootstub_wrapped.bin

$(BOOTSTUB_CRT0_OBJ): $(SRC_DIR)/crt0_bootstub.s | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0_bootstub.rel $(BOOTSTUB_CRT0_OBJ)
	-mv $(SRC_DIR)/crt0_bootstub.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0_bootstub.sym $(OBJ_DIR)/ 2>/dev/null || true

$(BOOTSTUB_OBJ): $(SRC_DIR)/bootstub.c | $(OBJ_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) -c $< -o $@

$(BOOTSTUB_HEX): $(BOOTSTUB_CRT0_OBJ) $(BOOTSTUB_OBJ) | $(BUILD_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) $(BOOTSTUB_LDFLAGS) $(BOOTSTUB_CRT0_OBJ) $(BOOTSTUB_OBJ) -o $@

$(BOOTSTUB_BIN): $(BOOTSTUB_HEX)
	@objcopy -I ihex -O binary $< $@ 2>/dev/null || sdobjcopy -I ihex -O binary $< $@
	@sz=$$(stat -c %s $@); echo "bootstub body: $$sz bytes (budget 9216)"; \
	 if [ $$sz -gt 9216 ]; then echo "FAIL: bootstub > 0x2400"; exit 1; fi

# ASM2464 firmware wrapper so the mask-ROM bootloader loads the bootstub.
$(BOOTSTUB_WRAPPED): $(BOOTSTUB_BIN)
	@python3 -c "\
import zlib; \
body = open('$<', 'rb').read(); \
body_len = len(body).to_bytes(4, 'little'); \
checksum = bytes([sum(body) & 0xFF]); \
crc = zlib.crc32(body).to_bytes(4, 'little'); \
magic = bytes([0xA5]); \
open('$@', 'wb').write(body_len + body + magic + checksum + crc)"

bootstub: $(BOOTSTUB_WRAPPED)
	@echo "Built: $(BOOTSTUB_WRAPPED)"
	@ls -la $(BOOTSTUB_WRAPPED)

# ===== Userfw image (bootstub-loaded) =====
APP_LDFLAGS = --code-loc 0x2430 --code-size 0xDBD0 --xram-loc 0x0B60 --xram-size 0xA0 --iram-size 0x100
APP_HEX = $(BUILD_DIR)/firmware_userfw.ihx
APP_BIN = $(BUILD_DIR)/firmware_userfw.bin
APP_IMAGE = $(BUILD_DIR)/firmware_image.bin

APP_CRT0_SRC = $(SRC_DIR)/crt0_userfw.s
APP_CRT0_OBJ = $(OBJ_DIR)/crt0_userfw.rel

$(APP_CRT0_OBJ): $(APP_CRT0_SRC) | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0_userfw.rel $(APP_CRT0_OBJ)
	-mv $(SRC_DIR)/crt0_userfw.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0_userfw.sym $(OBJ_DIR)/ 2>/dev/null || true

$(APP_HEX): $(APP_CRT0_OBJ) $(MAIN_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(APP_LDFLAGS) $(APP_CRT0_OBJ) $(MAIN_OBJ) -o $@

$(APP_BIN): $(APP_HEX)
	@objcopy -I ihex -O binary $< $@ 2>/dev/null || sdobjcopy -I ihex -O binary $< $@

$(APP_IMAGE): $(APP_BIN)
	@python3 ../tools/pack_userfw.py --common $< -o $@
	@echo "Built: $@"

image: $(APP_IMAGE)

bootstub-flash: $(BOOTSTUB_WRAPPED)
	@python3 ../ftdi_debug.py -bn
	@python3 ../flash.py $<
	@python3 ../ftdi_debug.py -rn

uflash: $(BUILD_DIR)/firmware_wrapped.bin
	@python3 e4_flash.py $<

nflash: $(BUILD_DIR)/firmware_wrapped.bin
	@echo "Resetting to bootloader..."
	@python3 ../ftdi_debug.py -bn
	@echo "Flashing firmware to device..."
	@python3 ../flash.py $<
	@python3 ../ftdi_debug.py -rn

# Flash to device
flash: $(BUILD_DIR)/firmware_wrapped.bin
	@echo "Resetting to bootloader..."
	@python3 ../ftdi_debug.py -bn
	@echo "Flashing firmware to device..."
	@python3 ../flash.py $<
	@echo "Resetting device..."
	@python3 ../ftdi_debug.py -rt 2

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
