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


========

Current task:

Your goal is to understand the DMA transfer mechanism. It works on the stock firmware, it doesn't work in the emulator. You need to fix it in the emulator.

You can flash the stock firmware with:
./ftdi_debug.py -bn && ./flash.py fw_tinygrad.bin && ./ftdi_debug.py -rn

Then you can trigger the DMA with:

asm2464pd-firmware fix_dma_emulator  ✗ python3 test_scsi_write.py
Connecting via ASM24Controller (add1:0001, UAS mode)...
Connected + initialized!

[1] Read 0xf000 baseline...
  0xf000: 5555555555555555555555555555555555555555555555555555555555555555

[2] scsi_write(512 bytes, lba=0)...
  first 16: 2ba400aff1a823deee15f52a13ec4194
  Done!

[3] Read 0xf000 after scsi_write...
  0xf000: 2ba400aff1a823deee15f52a13ec41945e553a45deca831a5f87369d838cc3489fa971486826e15554bdc684ed4a67bcae7054d7b01bdfdb29eb2b2d042baddc
  MATCH at 0xf000!

Done!

The emulator functions by proxying all the MMIO read/writes to the device while emulating the stock firmware:

You can flash the MMIO proxy with:
make -C clean flash-proxy

Then run the emulator with:
PYTHONUNBUFFERED=1 python3 emulate/emu.py --proxy --proxy-debug 2 fw_tinygrad.bin | tee /tmp/emulator_trace

The DMA fails with:

asm2464pd-firmware fix_dma_emulator  ❯ python3 test_scsi_write.py
Connecting via ASM24Controller (add1:0001, UAS mode)...
Connected + initialized!

[1] Read 0xf000 baseline...
Traceback (most recent call last):
  File "/home/geohot/asm2464pd-firmware/test_scsi_write.py", line 20, in <module>
    before = ctrl.read(0xf000, 32)
  File "/home/geohot/tinygrad/tinygrad/runtime/support/usb.py", line 225, in read
    parts = self.exec_ops([ReadOp(base_addr + off, min(stride, length - off)) for off in range(0, length, stride)])
  File "/home/geohot/tinygrad/tinygrad/runtime/support/usb.py", line 210, in exec_ops
    return self.usb.send_batch(cdbs, idata, odata)
           ~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^
  File "/home/geohot/tinygrad/tinygrad/runtime/support/usb.py", line 156, in send_batch
    self._submit_and_wait(tr_window)
    ~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^
  File "/home/geohot/tinygrad/tinygrad/runtime/support/usb.py", line 73, in _submit_and_wait
    elif tr.contents.status != 0xFF: raise RuntimeError(f"EP 0x{tr.contents.endpoint:02X} error: {tr.contents.status}")
                                     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RuntimeError: EP 0x81 error: 2

You need to fix this
