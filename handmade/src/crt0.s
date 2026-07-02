; crt0.s - Minimal startup code for clean firmware
;
; Simple startup: clear RAM, set stack, jump to main
;
; ISR vectors jump to SDCC-generated ISR functions (e.g. _int0_isr)
; If no ISR is defined, the weak symbol will just reti

    .module crt0
    .globl  _main

    ; ISR function symbols
    .globl  _int0_isr
    .globl  _int1_isr

    ; Linker-computed stack base (start of SSEG, after all IDATA/overlay)
    .globl  __start__stack

; Interrupt vectors in absolute area
    .area   VECTOR  (ABS,CODE)

; Reset vector (address 0x2400)
    .org    0x2400
__reset:
    ljmp    __sdcc_program_startup

; External interrupt 0 vector (address 0x2403)
    .org    0x2403
__ext0_vector:
    ljmp    _int0_isr

; Timer 0 vector (address 0x240B)
    .org    0x240B
    ljmp    _int1_isr

; External interrupt 1 vector (address 0x2413)
    .org    0x2413
__ext1_vector:
    ljmp    _int1_isr

; Timer 1 vector (address 0x241B)
    .org    0x241B
    reti

; Serial vector (address 0x2423)
    .org    0x2423
    reti

; Timer 2 vector (address 0x242B)
    .org    0x242B
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
    ; (__start__stack, placed right after all IDATA/overlay). Deriving it from
    ; the linker symbol keeps the stack correct as internal-RAM usage changes,
    ; instead of hardcoding an address that silently collides with data.
    mov     sp, #(__start__stack - 1)

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00
    mov     0xA8, #0x00     ; IE = 0, main re-enables interrupts

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
