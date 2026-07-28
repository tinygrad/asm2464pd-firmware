; crt0_userfw.s - Startup for bootstub-loaded userfw
;
; The bootstub copies this image from flash to CODE 0x3000 using the
; PCON bit 4 (MEMSEL) code_write mechanism. Reset enters here after validation;
; every interrupt vector is forwarded here by the bootstub.

    .module crt0_userfw
    .globl  _main

    ; ISR function symbols
    .globl  _int0_isr
    .globl  _int1_isr

    ; Linker-computed stack base (start of SSEG, after all IDATA/overlay)
    .globl  __start__stack

; Application vector table — absolute at 0x3000.
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
