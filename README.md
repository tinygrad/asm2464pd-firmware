# ASM2464PD Firmware

Open-source C firmware for the ASM2464PD USB4/Thunderbolt to NVMe bridge controller.

Install the bootstub and initial tinygrad firmware from the factory/debug
connection with

```bash
make -C handmade nrecover
```

After the bootstub is installed, update only the application over USB with

```bash
make -C handmade flash
```

Application source is at `handmade/src/main.c`. The self-contained USB2
recovery bootstub is in `handmade/src/bootstub/`.

The bootstub enumerates as `ADD1:B007` and uses a small vendor-specific EP0
protocol. It is not USB class DFU and is intentionally unable to modify the
bootstub region below flash offset `0x4000`.

There's a 0xF0 control message that reads/writes PCIe TLPs that gets 3.6 MB/s write, 1.8 MB/s read.
This is slow, but it can read and write anywhere on the PCIe BAR.

There's a 0xF2 DMA message that reads/writes to the 512 kB SRAM on the ASM chip.
It gets 700 MB/s read+write on 10 Gbps USB3. It is accessible on the PCIe BAR at 0x200000.
The SRAM read needs a write from the PCIe device side at 0x822000 to trigger the read.
You can see an example of this working / speed test with `pcie/test_sram_verify.py`

See CLAUDE.md for more stuff like how to run the emulator.
