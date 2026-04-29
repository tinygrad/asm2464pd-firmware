; ASM2464PD userfw - startup code, relocated to CODE 0x4000.
;
; The bootstub at 0x0000-0x3FFF jumps here after copying our body from
; flash and validating SHA-256. CPU interrupt vectors at 0x0003+ in the
; bootstub LJMP forward to 0x4003+, which is this table.

    .module userfw_crt0
    .globl  _main

    .area   VECTOR  (ABS,CODE)

    .org    0x4000
__reset:
    ljmp    __sdcc_program_startup

    .org    0x4003
    ljmp    _int0_isr

    .org    0x400B
    ljmp    _timer0_isr

    .org    0x4013
    ljmp    _int1_isr

    .org    0x401B
    ljmp    _timer1_isr

    .org    0x4023
    ljmp    _serial_isr

    .org    0x402B
    ljmp    _timer2_isr

    .area   HOME    (CODE)
__sdcc_program_startup:
    ; IRAM clear
    mov     r0, #0xff
    clr     a
clr_loop:
    mov     @r0, a
    djnz    r0, clr_loop

    ; Stack and bank-select init.
    mov     sp, #0x72
    mov     0x96, #0x00     ; PSBANK = 0
    mov     0xA8, #0x00     ; IE = 0 — main() re-enables when ready

    ljmp    _main

    ; Weak ISR stubs in case sdcc didn't generate them.
    .globl  _int0_isr
    .globl  _timer0_isr
    .globl  _int1_isr
    .globl  _timer1_isr
    .globl  _serial_isr
    .globl  _timer2_isr

    .area   GSINIT  (CODE)
    .area   GSFINAL (CODE)
    .area   HOME    (CODE)
