# ASM2464PD Firmware Reimplementation

Open-source C firmware for the ASM2464PD USB4/Thunderbolt to NVMe bridge controller.

## Target Hardware

**ASM2464PD** - Multi-interface bridge IC:
- USB 3.2 Gen2x2 / USB4 / Thunderbolt host interface
- PCIe 4.0 x4 NVMe storage interface
- 8051 CPU core (~114 MHz, 1T architecture)
- 98KB firmware in two 64KB code banks

## Building

### Prerequisites

* [SDCC](https://sdcc.sourceforge.net/) (Small Device C Compiler)
* GNU Make
* Python 3 (for the wrapper step)

On Debian/Ubuntu the toolchain can be installed with:

```bash
sudo apt-get update
sudo apt-get install -y sdcc make python3
```

### Build steps

```bash
make              # Build build/firmware.bin
make wrapped      # Build with ASM2464 header (checksum + CRC)
make compare      # Compare against original fw.bin
make clean        # Remove build artifacts
```

## Memory Map

### Code Banks
```
Bank 0: 0x00000-0x0FFFF  (64KB, direct access)
Bank 1: 0xFF6B-0x17ED5  (32KB, via DPX register, mapped at 0x8000)
```

### XDATA Memory Map
```
0x0000-0x5FFF  RAM (globals, work areas, stack)
0x6000-0xFFFF  Memory-mapped I/O registers
```

### Hardware Register Blocks
```
0x7000-0x7FFF  Flash Buffer (4KB)
0x8000-0x8FFF  USB/SCSI Buffer (4KB)
0x9000-0x93FF  USB Interface Controller
0x9E00-0x9FFF  USB Control Buffer
0xA000-0xAFFF  NVMe I/O Submission Queue (4KB)
0xB000-0xB0FF  NVMe Admin Submission Queue
0xB100-0xB1FF  NVMe Admin Completion Queue
0xB200-0xB4FF  PCIe Passthrough / TLP Engine
0xB800-0xB80F  PCIe Queue Control
0xC000-0xC00F  UART Controller (921600 baud)
0xC200-0xC2FF  Link / PHY Control
0xC400-0xC5FF  NVMe Interface Controller
0xC600-0xC6FF  PHY Extended Registers
0xC800-0xC80F  Interrupt Controller
0xC870-0xC87F  I2C Controller
0xC89F-0xC8AE  SPI Flash Controller
0xC8B0-0xC8D9  DMA Engine
0xCA00-0xCAFF  CPU Mode Control
0xCC10-0xCC24  Hardware Timers (4 channels)
0xCC30-0xCCFF  CPU Control Extended
0xCE00-0xCE9F  SCSI DMA / Transfer Control
0xCEB0-0xCEB3  USB Descriptor Validation
0xCEF0-0xCEFF  CPU Link Control
0xD800-0xD810  USB Endpoint Buffer / CSW
0xE300-0xE3FF  PHY Completion / Debug
0xE400-0xE4FF  Command Engine
0xE600-0xE6FF  Debug / Interrupt Status
0xE700-0xE7FF  System Status / Link Control
0xEC00-0xEC0F  NVMe Event Controller
0xEF00-0xEFFF  System Control
0xF000-0xFFFF  NVMe Data Buffer (4KB)
```

## Project Structure

```
src/
├── main.c           # Entry, init, main loop, ISRs
├── registers.h      # Hardware register definitions
├── globals.h        # Global variables
├── drivers/
│   ├── usb.c        # USB protocol and endpoints
│   ├── nvme.c       # NVMe command interface
│   ├── pcie.c       # PCIe/Thunderbolt interface
│   ├── dma.c        # DMA engine
│   ├── flash.c      # SPI flash
│   ├── timer.c      # Hardware timers
│   ├── uart.c       # Debug UART
│   ├── phy.c        # PHY/link layer
│   └── power.c      # Power management
└── app/
    ├── scsi.c       # SCSI/Mass Storage handler
    ├── protocol.c   # Protocol state machine
    ├── buffer.c     # Buffer management
    └── dispatch.c   # Bank switching stubs
```

## Emulator

The `emulate/` directory contains a full 8051 CPU emulator that can run `fw.bin`
with hardware emulation or proxy MMIO to real hardware over UART.

### Architecture

```
emulate/
├── emu.py          # Main Emulator class, CLI entry point
├── cpu.py          # 8051 CPU: instructions, interrupts, bank switching
├── memory.py       # Memory: code banks, XDATA, IDATA, SFR, bit addressing
├── hardware.py     # MMIO register emulation, USB/PCIe/DMA state machines
├── uart_proxy.py   # Binary protocol to proxy MMIO reads/writes over FTDI UART
├── usb_host.py     # USB host interface for injecting control/vendor commands
├── usb_device.py   # raw-gadget USB device (makes emulator appear in lsusb)
├── raw_gadget.py   # Linux raw_gadget kernel interface
└── disasm8051.py   # 8051 instruction table for trace/disassembly
```

### Modes of Operation

**Pure emulation** (no hardware):
```bash
python3 emulate/emu.py fw.bin
```
Runs firmware in software with emulated MMIO. Good for tracing code paths,
testing ISR handling, and analyzing firmware behavior. USB state machine is
driven by callbacks in `hardware.py`.

**UART proxy** (real hardware):
```bash
cd clean && make flash-proxy   # Flash proxy firmware to device
python3 emulate/emu.py --proxy fw.bin
```
CPU executes in the emulator, but every XDATA/SFR read/write is forwarded
to real hardware over UART (921600 baud via FTDI). The real USB PHY handles
link training, so the device appears in `lsusb` with real enumeration.
Interrupts from hardware (INT0, timers) are forwarded back to the emulator.

**Proxy with MMIO trace** (for reverse engineering):
```bash
python3 emulate/emu.py --proxy --proxy-debug 2 fw.bin
```
Same as proxy mode but logs every MMIO read/write with PC address and
register name. Debug levels: 1=interrupts only, 2=+XDATA, 3=+SFR.

### Proxy Firmware Protocol

The proxy firmware (`clean/src/proxy.c`) runs on the real 8051 and speaks a
binary protocol over UART:

| Command | Bytes Sent | Response |
|---------|-----------|----------|
| `0x00 <val>` Echo | 1 | `<val> <~val>` |
| `0x01 <hi> <lo>` XDATA Read | 2 | `<val> <~val>` |
| `0x02 <hi> <lo> <val>` XDATA Write | 3 | `0x00 0xFF` |
| `0x03 <addr>` SFR Read | 1 | `<val> <~val>` |
| `0x04 <addr> <val>` SFR Write | 2 | `0x00 0xFF` |
| `0x05 <mask>` INT ACK | 1 | `0x00 0xFF` |

Interrupt signals are sent asynchronously as `0x7E <mask>` where mask bits
correspond to 8051 interrupts (bit 0=INT0, 1=Timer0, etc).

### Key Emulator Options

```bash
# Trace specific PC addresses (e.g. ISR entry, function calls)
python3 emulate/emu.py --trace-pc 0x0E33 --trace-pc 0x3458 fw.bin

# Watch XDATA addresses for reads/writes
python3 emulate/emu.py --watch 0x0AF1 --watch 0x0B00 fw.bin

# Set breakpoints
python3 emulate/emu.py --break 0x494D fw.bin

# Full instruction trace (very verbose)
python3 emulate/emu.py --trace fw.bin

# Limit execution
python3 emulate/emu.py --max-cycles 5000000 fw.bin

# Inject USB vendor command after boot
python3 emulate/emu.py --usb-cmd E4:0x9000:1 fw.bin

# Proxy with emulated MMIO range (e.g. emulate UART locally)
python3 emulate/emu.py --proxy --proxy-mask 0xC000-0xC010 fw.bin

# USB device mode (appears in lsusb via raw-gadget/dummy_hcd)
sudo python3 emulate/emu.py --usb-device fw.bin
```

### Observed Bulk Transfer Sequence (Stock Firmware)

From proxy MMIO traces, the stock firmware's CBW-to-CSW flow for a TUR
(Test Unit Ready) command:

```
ISR triggered by INT0, reads 0x9101:
  bit 6 set (0x40) → CBW received path

CBW reception:
  Read  0x90E2 bit 0 → must be set (bulk data ready)
  Write 0x90E2 = 0x01      (ack/re-arm)
  Write 0xCE88 = 0x00      (init DMA state machine)
  Read  0xCE89 → 0x03      (DMA ready + error bits)
  Read  0x9119/911A         (CBW length = 0x001F = 31 bytes)
  Read  0x911B-911E         (CBW signature "USBC")
  Read  0x911F-9122         (CBW tag → written to D804-D807)
  Read  0x9123-9126         (transfer length)
  Read  0x9127              (flags/direction)
  Read  0x9128              (LUN)
  Write 0xD80C = 0x00       (clear CSW status)
  Read  0x912A              (SCSI opcode, 0x00 = TUR)

SCSI dispatch + state machine processing...

CSW send:
  Doorbell ramp up (C42A bit toggling: 0x01→0x03→0x07→0x0F→0x1F)
  Doorbell ramp down (0x1F→0x1E→0x1C→0x18→0x10→0x00)
  Write 0xD800 = 0x55 ("U")   CSW signature byte 0
  Write 0xD801 = 0x53 ("S")   CSW signature byte 1
  Write 0xD802 = 0x42 ("B")   CSW signature byte 2
  Write 0xD803 = 0x53 ("S")   CSW signature byte 3
  Write 0x901A = 0x0D          MSC transfer length (13 = CSW size)
  Write 0xC42C = 0x01          MSC trigger
  Read  0xC42D → clear bit 0   MSC status ack

Epilogue re-arm:
  Write 0xC42C = 0x01          Re-arm for next CBW
  Read/Write 0xC42D            Clear status
```

## Reference Materials

- `fw.bin` - Original firmware (98,012 bytes)
- `ghidra.c` - Ghidra decompilation reference
- `usb-to-pcie-re/` - Reverse engineering docs

## Development

1. Each function matches one in original firmware
2. Include address comments: `/* 0xABCD-0xABEF */`
3. Use `REG_` for registers, `G_` for globals
4. Analyze with radare2 or Ghidra

## Flashing

Custom firmware is built and flashed from the `clean/` directory:
```bash
cd clean
make flash          # Build, wrap with header, flash via USB, reset, show UART
make flash-proxy    # Flash the UART proxy firmware instead
```

`flash.py` uses tinygrad's USB3 library to send SCSI vendor commands (E1, E3, E8)
over bulk transfers to the device's bootloader.
## Contributing

Contributions are welcome! Feel free to open issues or pull requests.

## License

Reverse engineering project for educational and interoperability purposes.
