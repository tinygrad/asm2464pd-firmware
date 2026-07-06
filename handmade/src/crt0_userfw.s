; crt0_userfw.s - Startup for bootstub-loaded userfw
;
; The userfw lives in flash at 0x4000 and is executed directly from flash
; (mapped to CODE space). Interrupt vectors are at 0x4000+.

    .module crt0_userfw
    .globl  _main

    ; ISR function symbols
    .globl  _int0_isr
    .globl  _int1_isr

    ; Linker-computed stack base (start of SSEG, after all IDATA/overlay)
    .globl  __start__stack

; Interrupt vectors — absolute at 0x4000 (where bootstub trampolines to)
    .area   VECTOR  (ABS,CODE)

    .org    0x4000
__reset:
    ljmp    __sdcc_program_startup

    .org    0x4003
__ext0_vector:
    ljmp    _int0_isr

    .org    0x400B
__timer0_vector:
    reti

    .org    0x4013
__ext1_vector:
    ljmp    _int1_isr

    .org    0x401B
__timer1_vector:
    reti

    .org    0x4023
__serial_vector:
    reti

    .org    0x402B
__timer2_vector:
    reti

; Startup code in relocatable area
    .area   HOME    (CODE)
__sdcc_program_startup:
    ; Clear all internal RAM (IDATA 0x00-0xFF)
    mov     r0, #0xff
    clr     a
clear_ram_loop:
    mov     @r0, a
    djnz    r0, clear_ram_loop

    ; Set SP so the first push lands at SDCC's computed stack base
    mov     sp, #(__start__stack - 1)

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
