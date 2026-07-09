; Bootstub startup. Interrupt vectors trampoline into userfw at 0x2400+.
;
; Structure matches crt0_userfw.s / crt0.s: the whole project deliberately
; avoids initialized statics, so GSINIT is empty and startup jumps straight
; to _main. Keep it that way — the bootstub must not gain an initialized
; static without also running GSINIT here.

    .module bootstub_crt0
    .globl  _main
    ; Linker-computed stack base (start of SSEG, after all IDATA/overlay).
    .globl  __start__stack

    .area   VECTOR  (ABS,CODE)

    .org    0x0000
__reset:
    ljmp    __sdcc_program_startup

    .org    0x0003
    ljmp    0x2403

    .org    0x000B
    ljmp    0x240B

    .org    0x0013
    ljmp    0x2413

    .org    0x001B
    ljmp    0x241B

    .org    0x0023
    ljmp    0x2423

    .org    0x002B
    ljmp    0x242B

    .area   HOME    (CODE)
__sdcc_program_startup:
    mov     r0, #0xff
    clr     a
clr_loop:
    mov     @r0, a
    djnz    r0, clr_loop

    ; Derive SP from the linker's stack base (after all IDATA/overlay) rather
    ; than a hardcoded address, so a future rebuild that grows internal RAM
    ; cannot silently collide with the stack.
    mov     sp, #(__start__stack - 1)
    mov     0x96, #0x00     ; PSBANK = 0
    mov     0xA8, #0x00     ; IE = 0

    ljmp    _main

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
