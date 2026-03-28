Your goal is to make:

GMMU=0 AM_DEBUG=2 DEBUG=5 USE_BOT=1 PYTHONPATH="." AMD=1 AMD_IFACE=USB python3 test/test_tiny.py TestTiny.test_plus

work with the custom firmware in handmade/src/main.c


You can rebuild and flash the firmware with:

make -C handmade nflash


Goals:
* Fix the tests.
* Make the main tinygrad USB interface work.
* Document device registers in registers.h.



Using the proxy:

We have a proxy to run the firmware in an emulator while we proxy MMIO reads and write.

Flash it with:
make -C clean flash-proxy

Run it with:
python3 emulate/emu.py --proxy --proxy-debug 2 fw_tinygrad.bin


You can reset the device with:

./ftdi_debug.py -rn

If you don't need to change the code, this is faster than a reflash.