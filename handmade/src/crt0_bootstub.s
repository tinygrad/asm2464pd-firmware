; ASM2464PD bootstub crt0.
;
; The interrupt vectors at 0x0003-0x002B forward into userfw space at
; 0x4003-0x402B. While the bootstub itself runs we only ever set IE.EA
; (no individual interrupt source enabled), so these forwards are
; unreachable. Once we LJMP into userfw at 0x4000, userfw enables INT0
; and the chip's hardwired vector dispatch lands here, which trampolines
; into userfw's _int0_isr at 0x4003.

    .module bootstub_crt0
    .globl  _main

    .area   VECTOR  (ABS,CODE)

    .org    0x0000
__reset:
    ljmp    __sdcc_program_startup

    .org    0x0003
    ljmp    0x4003          ; INT0 → userfw's INT0 ISR

    .org    0x000B
    ljmp    0x400B          ; Timer 0

    .org    0x0013
    ljmp    0x4013          ; INT1

    .org    0x001B
    ljmp    0x401B          ; Timer 1

    .org    0x0023
    ljmp    0x4023          ; Serial

    .org    0x002B
    ljmp    0x402B          ; Timer 2

    .area   HOME    (CODE)
__sdcc_program_startup:
    mov     r0, #0xff
    clr     a
clr_loop:
    mov     @r0, a
    djnz    r0, clr_loop

    mov     sp, #0xC0
    mov     0x96, #0x00     ; PSBANK = 0
    mov     0xA8, #0x00     ; IE = 0 — main re-enables EA when ready

    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
