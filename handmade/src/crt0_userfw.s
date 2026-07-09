; crt0_userfw.s - Startup for bootstub-loaded userfw
;
; The bootstub copies this image from flash to CODE 0x2400 using the
; PCON bit 4 (MEMSEL) code_write mechanism. Interrupt vectors are at
; 0x2400+ and the bootstub's crt0 trampolines to them.

    .module crt0_userfw
    .globl  _main

    ; ISR function symbols
    .globl  _int0_isr
    .globl  _int1_isr

    ; Linker-computed stack base (start of SSEG, after all IDATA/overlay)
    .globl  __start__stack

; Interrupt vectors — absolute at 0x2400 (where bootstub trampolines to)
    .area   VECTOR  (ABS,CODE)

    .org    0x2400
__reset:
    ljmp    __sdcc_program_startup

    .org    0x2403
__ext0_vector:
    ljmp    _int0_isr

    .org    0x240B
__timer0_vector:
    reti

    .org    0x2413
__ext1_vector:
    ljmp    _int1_isr

    .org    0x241B
__timer1_vector:
    reti

    .org    0x2423
__serial_vector:
    reti

    .org    0x242B
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
    mov     0xA8, #0x00     ; IE = 0, main re-enables interrupts

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
