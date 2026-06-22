We are switching from sdcc to the Keil compiler

The Keil compiler used to build the real firmware is installed on this computer

Use it like this:

```bash
wine ~/.wine/drive_c/Keil_v5/C51/BIN/C51.exe test.c 'OPTIMIZE(9,SIZE)'
wine ~/.wine/drive_c/Keil_v5/C51/BIN/BL51.exe test.obj 'CODE(0x0800)' NODEFAULTLIBRARY
wine ~/.wine/drive_c/Keil_v5/C51/BIN/OH51.exe test
```

We want to produce a file as close to byte for byte identical to fw.bin that we can.

The addresses the functions belong at are in the function comments. I believe you can use linker scripts to correctly place them.

Go slow, adding things file by file. If things aren't perfectly byte for byte identical that's okay, but it should be very close.

Create a directory compare/ for any Python compare scripts that help you can check progress.

Ideally the sdcc compiler and Makefile should keep working, but the exact reconstruction is more important.

Do the reconstruction in address order.