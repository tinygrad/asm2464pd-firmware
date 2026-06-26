Your goal is to make:

DEBUG=2 PYTHONPATH="." DEV=USB+AMD python3 test/test_tiny.py TestTiny.test_plus

work with the custom firmware in handmade/src/main.c


You can rebuild and flash the firmware with:

make -C handmade nflash


Goals:
* Fix the tests.
* Make the main tinygrad USB interface work.
* Document device registers in registers.h.



You can reset the device with:

./ftdi_debug.py -rn

If you don't need to change the code, this is faster than a reflash.


If you want to do experiments with the stock firmware, you can flash it with:

./ftdi_debug.py -bn && ./flash.py fw_tinygrad.bin && ./ftdi_debug.py -rn


You can access the serial port with this in a different screen, then tail /tmp/serial:

PYTHONUNBUFFERED=1 ./ftdi_debug.py | tee /tmp/serial
