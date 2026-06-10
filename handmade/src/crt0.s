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

; Interrupt vectors in absolute area
    .area   VECTOR  (ABS,CODE)

; Reset vector (address 0x0000)
    .org    0x0000
__reset:
    ljmp    __sdcc_program_startup

; External interrupt 0 vector (address 0x0003)
    .org    0x0003
__ext0_vector:
    ljmp    _int0_isr

; External interrupt 1 vector (address 0x0013)
    .org    0x0013
__ext1_vector:
    ljmp    _int1_isr

; Startup code in relocatable area
    .area   HOME    (CODE)
__sdcc_program_startup:
    ; Clear all internal RAM (IDATA 0x00-0xFF)
    mov     r0, #0xff
    clr     a
clear_ram_loop:
    mov     @r0, a
    djnz    r0, clear_ram_loop

    ; Stack at 0xB0-0xFF (80 bytes). Raised from 0x72 because the lane-bond-FSM DSEG (IRAM globals)
    ; grew to ~0x74, and a fixed sp=0x72 then sat INSIDE DSEG -> the stack corrupted the PD/FSM
    ; globals on the first push -> boot hang. 0xB0 leaves DSEG the whole 0x08-0xAF window. (The
    ; linker's __start__stack symbol does NOT relocate into a #imm here -- it assembles to 0xFF and
    ; wraps the stack -> use a fixed literal with margin instead.)
    mov     sp, #0xB0

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
