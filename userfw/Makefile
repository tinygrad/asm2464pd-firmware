# ASM2464PD userfw Makefile
#
# Userfw lives at CODE 0x4000+ — vectors at 0x4000-0x402F (forwarded to
# from the bootstub at 0x0003-0x002B), code from 0x4030 onwards.
#
# The output `userfw.bin` is the raw image bytes; `tools/pack_userfw.py`
# wraps it with our header (magic, sizes, CRC-32) for flashing to SPI
# 0x4000+.

CC = sdcc
AS = sdas8051

CFLAGS  = -mmcs51 --model-small --opt-code-size --std-c99 --no-xinit-opt
CFLAGS += -Isrc -I../handmade/src -I../src/include

# Common-bank slot at 0x4000-0x7FFF (16 KB). If you outgrow it, build
# with USERFW_USE_BANKS=1 and split body into common/bank0/bank1 .bin
# files — pack_userfw.py glues them.
LDFLAGS  = --code-loc 0x4030 --code-size 0x3FD0
LDFLAGS += --xram-loc 0x0000 --xram-size 0x6000
LDFLAGS += --iram-size 0x100

SRC_DIR  = src
BUILD    = build
OBJ      = $(BUILD)/obj

OBJS = $(OBJ)/crt0.rel $(OBJ)/main.rel

TARGET = $(BUILD)/userfw
HEX    = $(TARGET).ihx
BIN    = $(TARGET).bin

.PHONY: all clean image

all: $(BIN)

$(OBJ):
	mkdir -p $(OBJ)
$(BUILD):
	mkdir -p $(BUILD)

$(OBJ)/crt0.rel: $(SRC_DIR)/crt0.s | $(OBJ)
	$(AS) -l -o -s $<
	mv $(SRC_DIR)/crt0.rel $@
	-mv $(SRC_DIR)/crt0.lst $(OBJ)/ 2>/dev/null || true
	-mv $(SRC_DIR)/crt0.sym $(OBJ)/ 2>/dev/null || true

$(OBJ)/%.rel: $(SRC_DIR)/%.c | $(OBJ)
	$(CC) $(CFLAGS) -c $< -o $@

$(HEX): $(OBJS) | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

# objcopy outputs bytes whose offset 0 = lowest IHX address (= 0x4000),
# so the .bin file IS the common-bank slice — pass-through.
$(BIN): $(HEX)
	@objcopy -I ihex -O binary $< $@
	@echo "userfw common bytes: $$(stat -c %s $@) (max 16384 in this slot)"

# Pack into the wire-format image consumed by the bootstub.
image: $(BIN)
	@python3 ../tools/pack_userfw.py --common $(BIN) -o $(BUILD)/userfw_image.bin

clean:
	rm -rf $(BUILD)
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.sym
	rm -f $(SRC_DIR)/*.rel $(SRC_DIR)/*.rst $(SRC_DIR)/*.map
	rm -f *.lk *.mem *.map
