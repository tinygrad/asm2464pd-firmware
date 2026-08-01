; Startup skips GSINIT; writable initializers are forbidden.

    .module crt0_userfw
    .globl  _main

    .globl  _int0_isr
    .globl  _int1_isr

    .globl  __start__stack

    .area   VECTOR  (ABS,CODE)

    .org    0x3000
__reset:
    ljmp    __sdcc_program_startup

    .org    0x3003
__ext0_vector:
    ljmp    _int0_isr

    .org    0x300B
__timer0_vector:
    reti

    .org    0x3013
__ext1_vector:
    ljmp    _int1_isr

    .org    0x301B
__timer1_vector:
    reti

    .org    0x3023
__serial_vector:
    reti

    .org    0x302B
__timer2_vector:
    reti

    .area   HOME    (CODE)
__sdcc_program_startup:
    mov     r0, #0xff
    clr     a
clear_ram_loop:
    mov     @r0, a
    djnz    r0, clear_ram_loop

    mov     sp, #(__start__stack - 1)

    mov     0x96, #0x00
    mov     0xA8, #0x00     ; IE = 0, main re-enables interrupts

    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
