; Bootstub startup skips GSINIT; writable initializers are forbidden.

    .module bootstub_crt0
    .globl  _main
    .globl  __start__stack

    .area   VECTOR  (ABS,CODE)

    .org    0x0000
__reset:
    ljmp    __sdcc_program_startup
; Interrupts re-routed to userfw
    .org    0x0003
    ljmp    0x3003

    .org    0x000B
    ljmp    0x300B

    .org    0x0013
    ljmp    0x3013

    .org    0x001B
    ljmp    0x301B

    .org    0x0023
    ljmp    0x3023

    .org    0x002B
    ljmp    0x302B

    .area   HOME    (CODE)
__sdcc_program_startup:
    mov     r0, #0xff
    clr     a
clr_loop:
    mov     @r0, a
    djnz    r0, clr_loop

    mov     sp, #(__start__stack - 1)
    mov     0x96, #0x00     ; PSBANK = 0
    mov     0xA8, #0x00     ; IE = 0

    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
