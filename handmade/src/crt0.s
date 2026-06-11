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

    ; Stack at 0x6B (IRAM-HEADROOM FIX, 2026-06-11). The USB4 SB-router CONNECT path (nested INT1
    ; a066 + bank0_8a89 + connect tail) needs a deep, collision-free stack. The 8051 stack lives at
    ; [sp+1 .. 0xFF]; DSEG globals/locals occupy [0x21 .. dseg_end]. sp MUST sit at dseg_end so the
    ; stack does not corrupt live data. MORE stack = LOWER sp (after shrinking DSEG). RAISING sp =
    ; LESS stack = the 4d1cc11 regression (sp 0x72->0xB0 killed C80A.5) -- NEVER do that.
    ;
    ; Regression history: as code grew, DSEG grew (data top 0x80 at 2f596c9), so the FIXED sp=0x72
    ; overlapped + corrupted LIVE globals/super-loop locals on the connect path; C80A.5 fire-rate
    ; fell c385da1 4/8 -> 23b9914 3/8 -> 2f596c9 0/8 (no reboot-loop -> silent state corruption, not
    ; a crash). FIX: shrink DSEG (persistent state flags/counters -> XDATA @0x8800; data-plane
    ; I2C/INA scratch -> XDATA) so the SDCC-reported data top dropped 0x80 -> 0x6C, then LOWER sp to
    ; 0x6B (just below data top) for a 148-byte CLEAN (non-overlapping) stack -- bigger AND
    ; collision-free vs HEAD's overlapping 141. Verify the build's __start__stack (= dseg_end) stays
    ; >= this sp; if DSEG grows again, move MORE IRAM state to XDATA -- do NOT raise sp.
    mov     sp, #0x6B

    ; Initialize DPX = 0 (bank 0)
    mov     0x96, #0x00

    ; Jump to main
    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
