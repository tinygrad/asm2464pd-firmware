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

    ; Stack at 0x72. Do NOT raise this. The USB4 SB-router CONNECT path (nested INT1 a066 +
    ; bank0_8a89 + the deep connect-tail call chain) needs ~133 bytes of stack headroom; sp=0x72
    ; gives 0x72..0xFF = 141 bytes. HW bisection (afb938e..038a6e0) proved that 4d1cc11's change
    ; raising this to 0xB0 (only 79 bytes) is THE regression that killed C80A.5: the connect path
    ; overflowed the short stack so the host's [===SB Con===] never fired. Empirically firing needs
    ; sp <= ~0x7A (0x7A fires, 0x7D/0x80/0xB0 do not) -- a stack-DEPTH cliff, not a DSEG-overlap
    ; issue. 0x72 overlaps the top few DSEG global bytes harmlessly (the firmware clears IRAM at
    ; boot and re-seeds those globals); restoring it brings C80A.5 back (HEAD 0/7 -> 6/8). If DSEG
    ; grows enough to make this marginal, shrink IRAM globals -- do not move the stack up.
    mov     sp, #0x72

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
