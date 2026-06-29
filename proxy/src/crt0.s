; crt0.s - Minimal startup code for clean firmware
;
; Simple startup: clear RAM, set stack, jump to main
; 
; ISR vectors jump to SDCC-generated ISR functions (e.g. _int0_isr)
; If no ISR is defined, the weak symbol will just reti

    .module crt0
    .globl  _main
    
    ; ISR function symbols (defined by SDCC when using __interrupt())
    .globl  _int0_isr
    .globl  _timer0_isr
    .globl  _int1_isr
    .globl  _timer1_isr
    .globl  _serial_isr
    .globl  _timer2_isr

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

; Timer 0 overflow vector (address 0x000B)
    .org    0x000B
__timer0_vector:
    ljmp    _timer0_isr

; External interrupt 1 vector (address 0x0013)
    .org    0x0013
__ext1_vector:
    ljmp    _int1_isr

; Timer 1 overflow vector (address 0x001B)
    .org    0x001B
__timer1_vector:
    ljmp    _timer1_isr

; Serial interrupt vector (address 0x0023)
    .org    0x0023
__serial_vector:
    ljmp    _serial_isr

; Timer 2 overflow vector (address 0x002B)
    .org    0x002B
__timer2_vector:
    ljmp    _timer2_isr

; Startup code in relocatable area
    .area   HOME    (CODE)
__sdcc_program_startup:
    ; Clear all internal RAM (IDATA 0x00-0xFF)
    mov     r0, #0xff
    clr     a
clear_ram_loop:
    mov     @r0, a
    djnz    r0, clear_ram_loop

    ; Initialize stack pointer
    mov     sp, #0x72

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
