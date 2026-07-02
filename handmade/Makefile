# Bootstub and bootstub-loaded user firmware Makefile

# Compiler and tools
CC = sdcc
AS = sdas8051

# Compiler flags
CFLAGS = -mmcs51 --model-small --opt-code-speed --peep-asm --allow-unsafe-read
CFLAGS += --std-c99
CFLAGS += -I$(SRC_DIR)
CFLAGS += --no-xinit-opt
CFLAGS += $(CFLAGS_EXTRA)

# Directories
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

MAIN_SRC = $(SRC_DIR)/main.c

# ===== Bootstub build =====
# Bootstub code is limited to 9 KB at the bottom of CODE space. User firmware
# lives at 0x2400+ and is updated through the bootstub DFU path.

BOOTSTUB_CFLAGS = -mmcs51 --model-large --opt-code-size --std-c99 \
                  --no-xinit-opt -I$(SRC_DIR) -I../src/include
# Reserve 0x5FF8-0x5FFF for the app-to-bootstub DFU cookie.
BOOTSTUB_LDFLAGS = --code-loc 0x0000 --code-size 0x2400 \
                   --xram-loc 0x0000 --xram-size 0x5FF8 --iram-size 0x100

BOOTSTUB_CRT0_OBJ = $(OBJ_DIR)/crt0_bootstub.rel
BOOTSTUB_OBJS = $(BOOTSTUB_CRT0_OBJ) \
                $(OBJ_DIR)/bootstub.rel
BOOTSTUB_TARGET  = $(BUILD_DIR)/bootstub
BOOTSTUB_HEX     = $(BOOTSTUB_TARGET).ihx
BOOTSTUB_BIN     = $(BOOTSTUB_TARGET).bin
BOOTSTUB_WRAPPED = $(BUILD_DIR)/bootstub_wrapped.bin

# ===== Bootstub-loaded handmade app image =====

APP_CRT0_OBJ = $(OBJ_DIR)/crt0.rel
APP_MAIN_OBJ = $(OBJ_DIR)/main_userfw.rel
APP_OBJS = $(APP_CRT0_OBJ) $(APP_MAIN_OBJ)
APP_TARGET = $(BUILD_DIR)/firmware_userfw
APP_HEX = $(APP_TARGET).ihx
APP_BIN = $(APP_TARGET).bin
APP_IMAGE = $(BUILD_DIR)/firmware_image.bin
# Place SDCC's auto-XDATA segment in the chip's low CM RAM. The USB4 lane-bond
# path can stall on the 0x88xx scratch area once the tunnel is active, while
# 0x0B60-0x0BFF stays valid and clear of the mailbox/FSM state.
# 0x5FF8-0x5FFF is reserved for the app-to-bootstub DFU cookie (accessed via
# an explicit pointer, not through SDCC auto-allocation).
APP_LDFLAGS = --code-loc 0x2430 --code-size 0xDBD0 \
              --xram-loc 0x0B60 --xram-size 0xA0 --iram-size 0x100

.PHONY: all clean flash nflash image bootstub bootstub-flash wrapped FORCE

all: $(APP_IMAGE) bootstub
	@echo "Built bootstub app image: $(APP_IMAGE)"
	@ls -la $(APP_IMAGE)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(APP_CRT0_OBJ): $(SRC_DIR)/crt0.s | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0.rel $@
	-mv $(SRC_DIR)/crt0.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0.sym $(OBJ_DIR)/ 2>/dev/null || true

$(APP_MAIN_OBJ): $(MAIN_SRC) $(SRC_DIR)/usb.h $(SRC_DIR)/flash.h \
                 $(SRC_DIR)/util.h $(SRC_DIR)/pcie_pio.h \
                 $(SRC_DIR)/pcie_tuning.h | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(APP_HEX): $(APP_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(APP_LDFLAGS) $(APP_OBJS) -o $@

$(APP_BIN): $(APP_HEX)
	@objcopy -I ihex -O binary $< $@
	@sz=$$(stat -c %s $@); echo "userfw body: $$sz bytes (budget 56320)"; \
	 if [ $$sz -gt 56320 ]; then echo "FAIL: userfw > 55 KB"; exit 1; fi

$(APP_IMAGE): $(APP_BIN) ../tools/pack_userfw.py FORCE
	@python3 ../tools/pack_userfw.py --common $< -o $@

image: $(APP_IMAGE)

$(BOOTSTUB_CRT0_OBJ): $(SRC_DIR)/crt0_bootstub.s | $(OBJ_DIR)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0_bootstub.rel $@
	-mv $(SRC_DIR)/crt0_bootstub.lst $(OBJ_DIR)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0_bootstub.sym $(OBJ_DIR)/ 2>/dev/null || true

$(OBJ_DIR)/bootstub.rel: $(SRC_DIR)/bootstub.c $(SRC_DIR)/usb.h $(SRC_DIR)/flash.h \
                         $(SRC_DIR)/util.h | $(OBJ_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) -c $< -o $@

$(BOOTSTUB_HEX): $(BOOTSTUB_OBJS) | $(BUILD_DIR)
	$(CC) $(BOOTSTUB_CFLAGS) $(BOOTSTUB_LDFLAGS) $(BOOTSTUB_OBJS) -o $@

$(BOOTSTUB_BIN): $(BOOTSTUB_HEX)
	@objcopy -I ihex -O binary $< $@
	@sz=$$(stat -c %s $@); echo "bootstub body: $$sz bytes (budget 9216)"; \
	 if [ $$sz -gt 9216 ]; then echo "FAIL: bootstub > 9 KB"; exit 1; fi

# Add ASM2464 firmware wrapper (body_len + body + magic + checksum + crc)
$(BOOTSTUB_WRAPPED): $(BOOTSTUB_BIN)
	@python3 -c "import zlib; \
body = open('$<', 'rb').read(); \
header = len(body).to_bytes(4, 'little'); \
trailer = bytes([0xA5]) + bytes([sum(body) & 0xFF]) + zlib.crc32(body).to_bytes(4, 'little'); \
open('$@', 'wb').write(header + body + trailer)"
	@ls -la $@

bootstub: $(BOOTSTUB_WRAPPED)

wrapped: $(BOOTSTUB_WRAPPED)

nflash: $(APP_IMAGE)
	@echo "Flashing userfw through bootstub DFU..."
	@python3 flash.py $<

flash: nflash

# One-time install via the FTDI bootloader strap. After this, app updates
# use the bootstub DFU path and do not need the FTDI strap.
bootstub-flash: $(BOOTSTUB_WRAPPED)
	@python3 ../ftdi_debug.py -bn
	@python3 ../flash.py $<
	@python3 ../ftdi_debug.py -rn

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
