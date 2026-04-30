# Minimal "hello" firmware Makefile

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
CFLAGS = -mmcs51 --model-small --opt-code-speed --peep-asm --allow-unsafe-read
CFLAGS += --std-c99
CFLAGS += -I$(SRC_DIR)
CFLAGS += -I../src/include
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

.PHONY: all clean flash wrapped proxy flash-proxy uflash bootstub bootstub-flash

all: $(BIN) bootstub
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

# Add ASM2464 mask-ROM wrapper: body_len(4) + body + magic(0xA5) + sum(1) + crc32(4)
$(BUILD_DIR)/%_wrapped.bin: $(BUILD_DIR)/%.bin
	@python3 -c "import zlib; \
body = open('$<', 'rb').read(); \
header = len(body).to_bytes(4, 'little'); \
trailer = bytes([0xA5]) + bytes([sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, 'little'); \
open('$@', 'wb').write(header + body + trailer)"
	@ls -la $@

wrapped: $(BUILD_DIR)/firmware_wrapped.bin

uflash: $(BUILD_DIR)/firmware_wrapped.bin
	@python3 ../scripts/e4_flash.py $<

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

# ===== Bootstub build =====
# Same compiler as the full firmware so usb.h, registers.h, etc. are
# guaranteed identical. Code limited to 16 KB at the bottom of CODE
# space; userfw lives at 0x4000+.

BOOTSTUB_CFLAGS = -mmcs51 --model-large --opt-code-size --std-c99 \
                  --no-xinit-opt -I$(SRC_DIR) -I../src/include
BOOTSTUB_LDFLAGS = --code-loc 0x0000 --code-size 0x4000 \
                   --xram-loc 0x0000 --xram-size 0x6000 --iram-size 0x100

BOOTSTUB_CRT0_OBJ = $(OBJ_DIR)/crt0_bootstub.rel
BOOTSTUB_OBJS = $(BOOTSTUB_CRT0_OBJ) \
                $(OBJ_DIR)/bootstub.rel
BOOTSTUB_TARGET  = $(BUILD_DIR)/bootstub
BOOTSTUB_HEX     = $(BOOTSTUB_TARGET).ihx
BOOTSTUB_BIN     = $(BOOTSTUB_TARGET).bin
BOOTSTUB_WRAPPED = $(BUILD_DIR)/bootstub_wrapped.bin

$(BOOTSTUB_CRT0_OBJ): $(SRC_DIR)/crt0_bootstub.s | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0_bootstub.rel $@
	-mv $(SRC_DIR)/crt0_bootstub.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0_bootstub.sym $(OBJ_DIR)/ 2>/dev/null || true

$(OBJ_DIR)/bootstub.rel: $(SRC_DIR)/bootstub.c $(SRC_DIR)/usb.h $(SRC_DIR)/flash.h | $(OBJ_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) -c $< -o $@

$(BOOTSTUB_HEX): $(BOOTSTUB_OBJS) | $(BUILD_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) $(BOOTSTUB_LDFLAGS) $(BOOTSTUB_OBJS) -o $@

$(BOOTSTUB_BIN): $(BOOTSTUB_HEX)
	@objcopy -I ihex -O binary $< $@
	@sz=$$(stat -c %s $@); echo "bootstub body: $$sz bytes (budget 16384)"; \
	 if [ $$sz -gt 16384 ]; then echo "FAIL: bootstub > 16 KB"; exit 1; fi

bootstub: $(BOOTSTUB_WRAPPED)

# One-time install via the FTDI bootloader strap. After this, future userfw
# updates can go through handmade/flash.py without the FTDI.
bootstub-flash: $(BOOTSTUB_WRAPPED)
	@python3 ../ftdi_debug.py -bn
	@python3 ../flash.py $<
	@python3 ../ftdi_debug.py -rn