/*
 * pcie_pio.h — Hand-optimized 8051 assembly routines for PCIe programmed I/O.
 *
 * These stream dwords between an XDATA buffer and the PCIe TLP engine,
 * auto-incrementing the PCIe address by 4 after each dword.
 *
 * PCIe ADDR registers must be primed with the starting address before calling.
 * Carry propagates through the full 64-bit address (low 32 + high 32).
 *
 * Data byte order (LE): buf[0]=LSB -> DATA_3, buf[3]=MSB -> DATA_0.
 *
 * ADDR register layout (consecutive):
 *   0xB218 ADDR_0  (addr[31:24])   — MSB of low 32
 *   0xB219 ADDR_1  (addr[23:16])
 *   0xB21A ADDR_2  (addr[15:8])
 *   0xB21B ADDR_3  (addr[7:0])     — LSB of low 32
 *   0xB21C ADDR_HIGH   (addr[39:32]) — LSB of high 32
 *   0xB21D ADDR_HIGH_1 (addr[47:40])
 *   0xB21E ADDR_HIGH_2 (addr[55:48])
 *   0xB21F ADDR_HIGH_3 (addr[63:56]) — MSB of high 32
 */
#ifndef PCIE_PIO_H
#define PCIE_PIO_H

#include "types.h"

/* Optimized PCIe read with pipelining.
 * Uses count-down loop to avoid expensive 16-bit compare.
 * Triggers next TLP before storing data to overlap PCIe latency.
 *
 * dst in DPTR (dpl/dph), cnt in _pcie_read_chunk_PARM_2 (2 bytes). */
static void pcie_read_chunk(__xdata uint8_t *dst, uint16_t cnt) {
  (void)dst; (void)cnt;
  __asm
    mov   r6, dpl
    mov   r7, dph
    mov   r4, (_pcie_read_chunk_PARM_2)
    mov   r5, (_pcie_read_chunk_PARM_2 + 1)

    ; trigger first TLP
    mov   dptr, #0xB296
    mov   a, #0x07
    movx  @dptr, a
    mov   dptr, #0xB254
    mov   a, #0x0F
    movx  @dptr, a

  _pcie_rd_loop:
  _pcie_rd_poll:
    mov   dptr, #0xB296
    movx  a, @dptr
    anl   a, #0x03
    jz    _pcie_rd_poll

    ; read all 4 DATA regs (B223 down to B220)
    mov   dptr, #0xB223
    movx  a, @dptr
    mov   r2, a           ; DATA_3 (LSB)
    dec   dpl
    movx  a, @dptr
    mov   r3, a           ; DATA_2
    dec   dpl
    movx  a, @dptr
    mov   r1, a           ; DATA_1
    dec   dpl
    movx  a, @dptr
    mov   r0, a           ; DATA_0 (MSB)

    ; increment 64-bit PCIe address by 4, carry through all 8 bytes
    mov   dptr, #0xB21B
    movx  a, @dptr
    add   a, #0x04
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B21A = ADDR_2
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B219 = ADDR_1
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B218 = ADDR_0
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    ; carry into high 32: ADDR_HIGH_3 (B21F) = LSB
    mov   dptr, #0xB21F
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B21E = ADDR_HIGH_2
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B21D = ADDR_HIGH_1
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_rd_noc
    dec   dpl             ; B21C = ADDR_HIGH
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
  _pcie_rd_noc:

    ; decrement count, check if more remain
    dec   r4
    cjne  r4, #0xFF, _pcie_rd_nodec5
    dec   r5
  _pcie_rd_nodec5:
    mov   a, r4
    orl   a, r5
    jz    _pcie_rd_last

    ; trigger NEXT TLP (overlap PCIe latency with buffer store below)
    mov   dptr, #0xB296
    mov   a, #0x07
    movx  @dptr, a
    mov   dptr, #0xB254
    mov   a, #0x0F
    movx  @dptr, a

  _pcie_rd_last:
    ; store 4 bytes to dst buffer
    mov   dpl, r6
    mov   dph, r7
    mov   a, r2
    movx  @dptr, a        ; DATA_3
    inc   dptr
    mov   a, r3
    movx  @dptr, a        ; DATA_2
    inc   dptr
    mov   a, r1
    movx  @dptr, a        ; DATA_1
    inc   dptr
    mov   a, r0
    movx  @dptr, a        ; DATA_0
    inc   dptr
    mov   r6, dpl
    mov   r7, dph

    mov   a, r4
    orl   a, r5
    jz    _pcie_rd_done
    ljmp  _pcie_rd_loop

  _pcie_rd_done:
  __endasm;
}

/* Optimized PCIe write: buffer -> DATA regs -> trigger (fire-and-forget).
 * Uses count-down loop.
 * src in DPTR, cnt in _pcie_write_chunk_PARM_2 (2 bytes). */
static void pcie_write_chunk(__xdata uint8_t *src, uint16_t cnt) {
  (void)src; (void)cnt;
  __asm
    mov   r6, dpl
    mov   r7, dph
    mov   r4, (_pcie_write_chunk_PARM_2)
    mov   r5, (_pcie_write_chunk_PARM_2 + 1)

  _pcie_wr_loop:
    ; read 4 bytes from src
    mov   dpl, r6
    mov   dph, r7
    movx  a, @dptr
    mov   r2, a           ; src[0] -> DATA_3 (LSB)
    inc   dptr
    movx  a, @dptr
    mov   r3, a           ; src[1] -> DATA_2
    inc   dptr
    movx  a, @dptr
    mov   r1, a           ; src[2] -> DATA_1
    inc   dptr
    movx  a, @dptr
    ; a = src[3] -> DATA_0 (MSB)
    inc   dptr
    mov   r6, dpl
    mov   r7, dph

    ; write DATA regs B220-B223
    mov   dptr, #0xB220
    movx  @dptr, a        ; DATA_0
    inc   dpl
    mov   a, r1
    movx  @dptr, a        ; DATA_1
    inc   dpl
    mov   a, r3
    movx  @dptr, a        ; DATA_2
    inc   dpl
    mov   a, r2
    movx  @dptr, a        ; DATA_3

    ; clear status + trigger
    mov   dptr, #0xB296
    mov   a, #0x07
    movx  @dptr, a
    mov   dptr, #0xB254
    mov   a, #0x0F
    movx  @dptr, a

    ; increment 64-bit PCIe address by 4, carry through all 8 bytes
    mov   dptr, #0xB21B
    movx  a, @dptr
    add   a, #0x04
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B21A = ADDR_2
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B219 = ADDR_1
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B218 = ADDR_0
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    ; carry into high 32: ADDR_HIGH_3 (B21F) = LSB
    mov   dptr, #0xB21F
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B21E = ADDR_HIGH_2
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B21D = ADDR_HIGH_1
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
    jnc   _pcie_wr_noc
    dec   dpl             ; B21C = ADDR_HIGH
    movx  a, @dptr
    add   a, #0x01
    movx  @dptr, a
  _pcie_wr_noc:

    ; count down, loop if nonzero
    dec   r4
    cjne  r4, #0xFF, _pcie_wr_nodec5
    dec   r5
  _pcie_wr_nodec5:
    mov   a, r4
    orl   a, r5
    jnz   _pcie_wr_loop

  _pcie_wr_done:
  __endasm;
}

#endif /* PCIE_PIO_H */
