Your goal is to make:

GMMU=0 AM_DEBUG=2 DEBUG=5 USE_BOT=1 PYTHONPATH="." AMD=1 AMD_IFACE=USB python3 test/test_tiny.py TestTiny.test_plus

work with the custom firmware in clean/src/main.c

This is the final goal to keep in mind.
However, if you find issues, it's better to add local failing tests and fix them locally first.
Then when you are confident, try the full tinygrad flow.


You can run the local tests with:

make -C clean flash && python3 test_tinygrad_flow.py --timeout=15

Currently some of these tests are failing.
IMPORTANT: you must be using DMA for the write path.
The processor is very slow and cannot take action on each byte.


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