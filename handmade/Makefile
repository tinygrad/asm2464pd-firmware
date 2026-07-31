# Handmade firmware Makefile

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
COMMON_CFLAGS = -mmcs51 --std-c99 --no-xinit-opt -I$(SRC_DIR)
CFLAGS = $(COMMON_CFLAGS) --model-small --opt-code-speed --peep-asm --allow-unsafe-read
CFLAGS += $(CFLAGS_EXTRA)
BOOTSTUB_CFLAGS = $(COMMON_CFLAGS) --model-large --opt-code-size

# Memory model settings
# Place SDCC's auto-XDATA segment in the chip's low CM RAM. The USB4 lane-bond
# path can stall on the 0x88xx scratch area once the tunnel is active, while
# 0x0B60-0x0BFF stays valid and clear of the mailbox/FSM state.
APP_RAM_LDFLAGS = --xram-loc 0x0B60 --xram-size 0xA0 --iram-size 0x100
LDFLAGS = --code-loc 0x0000 --code-size 0x10000 $(APP_RAM_LDFLAGS)
BOOTSTUB_LDFLAGS = --code-loc 0x0000 --code-size 0x3000 --xram-loc 0x0C00 --xram-size 0x400 --iram-size 0x100
USERFW_LDFLAGS = --code-loc 0x3030 --code-size 0xCFD0 $(APP_RAM_LDFLAGS)

# Directories
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Object files
CRT0_OBJ = $(OBJ_DIR)/crt0.rel
MAIN_OBJ = $(OBJ_DIR)/main.rel
BOOTSTUB_CRT0_OBJ = $(OBJ_DIR)/crt0_bootstub.rel
BOOTSTUB_OBJ = $(OBJ_DIR)/bootstub.rel
USERFW_CRT0_OBJ = $(OBJ_DIR)/crt0_userfw.rel
OBJS = $(CRT0_OBJ) $(MAIN_OBJ)

# Output files
TARGET = $(BUILD_DIR)/firmware
HEX = $(TARGET).ihx
BIN = $(TARGET).bin
BOOTSTUB = $(BUILD_DIR)/bootstub
USERFW = $(BUILD_DIR)/firmware_userfw
USERFW_IMAGE = $(BUILD_DIR)/firmware_image.bin

.PHONY: all clean flash nrecover wrapped uflash bootstub image FORCE

all: $(BIN)
	@echo "Built: $(BIN)"
	@ls -la $(BIN)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Assemble startup code
$(OBJ_DIR)/%.rel: $(SRC_DIR)/%.s | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/$*.rel $@
	-mv $(SRC_DIR)/$*.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/$*.sym $(OBJ_DIR)/ 2>/dev/null || true

# Compile C sources. Header deps need a second pass because SDCC has no -MMD.
$(MAIN_OBJ): FW_CFLAGS = $(CFLAGS)
$(BOOTSTUB_OBJ): FW_CFLAGS = $(BOOTSTUB_CFLAGS)

$(OBJ_DIR)/%.rel: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(FW_CFLAGS) -c $< -o $@
	@$(CC) $(FW_CFLAGS) -MM $< 2>/dev/null | sed 's|^[^:]*:|$@:|' > $(OBJ_DIR)/$*.d

-include $(OBJ_DIR)/main.d $(OBJ_DIR)/bootstub.d

# Link object files to Intel HEX
$(HEX): $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

# Convert Intel HEX to binary
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.ihx
	objcopy -I ihex -O binary $< $@

# Add ASM2464 firmware wrapper (body_len + body + magic + checksum + crc)
$(BUILD_DIR)/%_wrapped.bin: $(BUILD_DIR)/%.bin
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

$(BOOTSTUB).ihx: $(BOOTSTUB_CRT0_OBJ) $(BOOTSTUB_OBJ) | $(BUILD_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) $(BOOTSTUB_LDFLAGS) $^ -o $@

$(USERFW).ihx: $(USERFW_CRT0_OBJ) $(MAIN_OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(USERFW_LDFLAGS) $^ -o $@

$(USERFW_IMAGE): $(USERFW).bin ../tools/pack_userfw.py FORCE
	@python3 ../tools/pack_userfw.py --common $< -o $@

bootstub: $(BOOTSTUB)_wrapped.bin
image: $(USERFW_IMAGE)

FORCE:

uflash: $(BUILD_DIR)/firmware_wrapped.bin
	@python3 e4_flash.py $<

nrecover: $(BOOTSTUB)_wrapped.bin $(USERFW_IMAGE)
	@echo "Resetting to mask-ROM bootloader..."
	@python3 ../ftdi_debug.py -bn
	@echo "Installing bootstub..."
	@python3 ../flash.py $(BOOTSTUB)_wrapped.bin
	@python3 ../ftdi_debug.py -rn
	@echo "Installing user firmware through bootstub DFU..."
	@python3 flash.py flash $(USERFW_IMAGE) --no-reboot --verify
	@echo "Resetting into user firmware..."
	@python3 ../ftdi_debug.py -rn
	@echo "Verifying application-to-bootstub recovery handoff..."
	@python3 flash.py check

flash: $(USERFW_IMAGE)
	@python3 flash.py flash $<

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
