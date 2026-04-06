/*
 * nvme.c - NVMe Command and Queue Management
 *
 * See drivers/nvme.h for hardware documentation.
 */

#include "drivers/nvme.h"
#include "types.h"
#include "sfr.h"
#include "registers.h"
#include "globals.h"
#include "drivers/dma.h"
#include "drivers/usb.h"
#include "drivers/cmd.h"
#include "utils.h"
#include "drivers/power.h"

/* External helper functions not in headers */
extern void usb_get_descriptor_ptr(void);
extern uint16_t core_state_read16(void);
extern uint8_t stub_return_zero(void);
extern void queue_param_buf_calc(uint8_t param);

/* Forward declarations */
void nvme_util_get_queue_depth(uint8_t p1, uint8_t p2);

/*
 * nvme_status_update - Update NVMe device and control status
 * Address: 0x1b47-0x1b5f (25 bytes)
 *
 * Reads G_STATE_HELPER_42, ORs with high bits of REG_NVME_DEV_STATUS,
 * writes result. Then sets bit 1 of REG_NVME_CTRL_STATUS.
 */
void nvme_status_update(void)
{
    uint8_t state_val = G_STATE_HELPER_42;
    uint8_t dev_status_hi = REG_NVME_DEV_STATUS & 0xC0;
    REG_NVME_DEV_STATUS = state_val | dev_status_hi;
    REG_NVME_CTRL_STATUS = (REG_NVME_CTRL_STATUS & 0xFD) | 0x02;
}

/*
 * nvme_set_usb_mode_bit - Set USB mode bit 0 of register 0x9006
 * Address: 0x1bde-0x1be7 (10 bytes)
 *
 * Reads 0x9006, clears bit 0, sets bit 0, writes back.
 *
 * Original disassembly:
 *   1bde: mov dptr, #0x9006
 *   1be1: movx a, @dptr
 *   1be2: anl a, #0xfe        ; clear bit 0
 *   1be4: orl a, #0x01        ; set bit 0
 *   1be6: movx @dptr, a
 *   1be7: ret
 */
void nvme_set_usb_mode_bit(void)
{
    uint8_t val;

    val = REG_USB_EP0_CONFIG;
    val = (val & 0xFE) | 0x01;
    REG_USB_EP0_CONFIG = val;
}

/*
 * nvme_get_config_offset - Get configuration offset address
 * Address: 0x1be9-0x1bf5 (13 bytes)
 *
 * Reads from 0x0464, adds 0x56 to form address in 0x04XX region,
 * and returns that address.
 *
 * Original disassembly:
 *   1be8: mov dptr, #0x0464
 *   1beb: movx a, @dptr       ; A = XDATA[0x0464]
 *   1bec: add a, #0x56        ; A = A + 0x56
 *   1bee: mov 0x82, a         ; DPL = A
 *   1bf0: clr a
 *   1bf1: addc a, #0x04       ; DPH = 0x04 + carry
 *   1bf3: mov 0x83, a
 *   1bf5: ret                 ; returns with DPTR = 0x04XX
 */
__xdata uint8_t *nvme_get_config_offset(void)
{
    uint8_t val = G_SYS_STATUS_PRIMARY;
    uint16_t addr = 0x0400 + val + 0x56;
    return (__xdata uint8_t *)addr;
}

/*
 * nvme_calc_buffer_offset - Calculate buffer offset with multiplier
 * Address: 0x1bf6-0x1c0e (25 bytes)
 *
 * Multiplies input by 0x40, adds to values from 0x021A-0x021B,
 * and stores result to 0x0568-0x0569.
 *
 * Original disassembly:
 *   1bf6: mov 0xf0, #0x40     ; B = 0x40
 *   1bf9: mul ab              ; A*B, result in B:A
 *   1bfa: mov r7, a           ; R7 = low byte
 *   1bfb: mov dptr, #0x021b
 *   1bfe: movx a, @dptr       ; A = XDATA[0x021B]
 *   1bff: add a, r7           ; A = A + R7
 *   1c00: mov r6, a           ; R6 = low result
 *   1c01: mov dptr, #0x021a
 *   1c04: movx a, @dptr       ; A = XDATA[0x021A]
 *   1c05: addc a, 0xf0        ; A = A + B + carry
 *   1c07: mov dptr, #0x0568
 *   1c0a: movx @dptr, a       ; XDATA[0x0568] = high byte
 *   1c0b: inc dptr
 *   1c0c: xch a, r6
 *   1c0d: movx @dptr, a       ; XDATA[0x0569] = low byte
 *   1c0e: ret
 */
void nvme_calc_buffer_offset(uint8_t index)
{
    uint16_t offset;
    uint16_t base;
    uint16_t result;

    /* Calculate offset = index * 0x40 */
    offset = (uint16_t)index * 0x40;

    /* Read base address (big-endian) */
    base = G_BUF_BASE_HI;
    base = (base << 8) | G_BUF_BASE_LO;

    /* Calculate result = base + offset */
    result = base + offset;

    /* Store result (big-endian) */
    G_BUF_OFFSET_HI = (uint8_t)(result >> 8);
    G_BUF_OFFSET_LO = (uint8_t)(result & 0xFF);
}

/*
 * nvme_load_transfer_data - Load transfer data from IDATA
 * Address: 0x1bcb-0x1bd4 (10 bytes)
 *
 * Loads 32-bit value from IDATA[0x6B] and stores to IDATA[0x6F].
 *
 * Original disassembly:
 *   1bcb: mov r0, #0x6b
 *   1bcd: lcall 0x0d78        ; idata_load_dword
 *   1bd0: mov r0, #0x6f
 *   1bd2: ljmp 0x0db9         ; idata_store_dword
 */
void nvme_load_transfer_data(void)
{
    uint32_t val;

    /* Load from IDATA[0x6B] */
    val = ((__idata uint8_t *)0x6B)[0];
    val |= ((uint32_t)((__idata uint8_t *)0x6B)[1]) << 8;
    val |= ((uint32_t)((__idata uint8_t *)0x6B)[2]) << 16;
    val |= ((uint32_t)((__idata uint8_t *)0x6B)[3]) << 24;

    /* Store to IDATA[0x6F] */
    ((__idata uint8_t *)0x6F)[0] = (uint8_t)(val & 0xFF);
    ((__idata uint8_t *)0x6F)[1] = (uint8_t)((val >> 8) & 0xFF);
    ((__idata uint8_t *)0x6F)[2] = (uint8_t)((val >> 16) & 0xFF);
    ((__idata uint8_t *)0x6F)[3] = (uint8_t)((val >> 24) & 0xFF);
}

/*
 * nvme_calc_idata_offset - Calculate address from IDATA offset
 * Address: 0x1c0f-0x1c1a (12 bytes)
 *
 * Returns pointer to 0x000C + IDATA[0x3C].
 *
 * Original disassembly:
 *   1c0f: mov a, #0x0c
 *   1c11: add a, 0x3c         ; A = 0x0C + IDATA[0x3C]
 *   1c13: mov 0x82, a         ; DPL
 *   1c15: clr a
 *   1c16: addc a, #0x00       ; DPH = carry
 *   1c18: mov 0x83, a
 *   1c1a: ret
 */
__xdata uint8_t *nvme_calc_idata_offset(void)
{
    uint8_t offset = *(__idata uint8_t *)0x3C;
    return (__xdata uint8_t *)(0x000C + offset);
}

/*
 * nvme_check_scsi_ctrl - Check SCSI control status
 * Address: 0x1c23-0x1c29 (8 bytes, after store)
 *
 * Reads SCSI control from 0x0171 and checks if non-zero.
 * Returns with carry set if value is zero.
 *
 * Original disassembly:
 *   1c22: mov dptr, #0x0171
 *   1c25: movx a, @dptr       ; A = G_SCSI_CTRL
 *   1c26: setb c
 *   1c27: subb a, #0x00       ; compare with 0
 *   1c29: ret
 */
uint8_t nvme_check_scsi_ctrl(void)
{
    return (G_SCSI_CTRL != 0) ? 1 : 0;
}

/*
 * nvme_get_cmd_param_upper - Get upper 3 bits of NVMe command param
 * Address: 0x1c77-0x1c7d (7 bytes)
 *
 * Reads NVMe command parameter and masks to upper 3 bits.
 *
 * Original disassembly:
 *   1c77: mov dptr, #0xc429
 *   1c7a: movx a, @dptr       ; A = REG_NVME_CMD_PARAM
 *   1c7b: anl a, #0xe0        ; mask bits 7-5
 *   1c7d: ret
 */
uint8_t nvme_get_cmd_param_upper(void)
{
    return REG_NVME_CMD_PARAM & NVME_CMD_PARAM_TYPE;
}

/*
 * nvme_subtract_idata_16 - Subtract 16-bit value from IDATA[0x16:0x17]
 * Address: 0x1c6d-0x1c76 (10 bytes)
 *
 * Subtracts R6:R7 from the 16-bit value at IDATA[0x16:0x17].
 *
 * Original disassembly:
 *   1c6d: mov r0, #0x17
 *   1c6f: mov a, @r0          ; A = IDATA[0x17]
 *   1c70: subb a, r7          ; A = A - R7 - C
 *   1c71: mov @r0, a          ; IDATA[0x17] = A
 *   1c72: dec r0
 *   1c73: mov a, @r0          ; A = IDATA[0x16]
 *   1c74: subb a, r6          ; A = A - R6 - C
 *   1c75: mov @r0, a          ; IDATA[0x16] = A
 *   1c76: ret
 */
void nvme_subtract_idata_16(uint8_t hi, uint8_t lo)
{
    uint16_t val = ((*(__idata uint8_t *)0x16) << 8) | (*(__idata uint8_t *)0x17);
    uint16_t sub = ((uint16_t)hi << 8) | lo;
    val -= sub;
    *(__idata uint8_t *)0x16 = (uint8_t)(val >> 8);
    *(__idata uint8_t *)0x17 = (uint8_t)(val & 0xFF);
}

/*
 * nvme_calc_addr_01xx - Calculate address in 0x01XX region
 * Address: 0x1c88-0x1c8f (8 bytes)
 *
 * Returns pointer to 0x0100 + input value.
 *
 * Original disassembly:
 *   1c88: mov 0x82, a         ; DPL = A
 *   1c8a: clr a
 *   1c8b: addc a, #0x01       ; DPH = 0x01 + carry
 *   1c8d: mov 0x83, a
 *   1c8f: ret
 */
__xdata uint8_t *nvme_calc_addr_01xx(uint8_t offset)
{
    return (__xdata uint8_t *)(0x0100 + offset);
}

/*
 * nvme_inc_circular_counter - Increment circular counter at 0x0B00
 * Address: 0x1cae-0x1cb6 (9 bytes)
 *
 * Reads counter, increments, masks to 5 bits (0-31 range), writes back.
 *
 * Original disassembly:
 *   1cae: mov dptr, #0x0b00
 *   1cb1: movx a, @dptr       ; read counter
 *   1cb2: inc a               ; increment
 *   1cb3: anl a, #0x1f        ; mask to 5 bits
 *   1cb5: movx @dptr, a       ; write back
 *   1cb6: ret
 */
void nvme_inc_circular_counter(void)
{
    uint8_t val = G_USB_PARAM_0B00;
    val = (val + 1) & 0x1F;
    G_USB_PARAM_0B00 = val;
}

/*
 * nvme_calc_addr_012b - Calculate address in 0x012B+ region
 * Address: 0x1cb7-0x1cc0 (10 bytes)
 *
 * Returns pointer to 0x012B + input value.
 *
 * Original disassembly:
 *   1cb7: add a, #0x2b
 *   1cb9: mov 0x82, a         ; DPL
 *   1cbb: clr a
 *   1cbc: addc a, #0x01       ; DPH = 0x01 + carry
 *   1cbe: mov 0x83, a
 *   1cc0: ret
 */
__xdata uint8_t *nvme_calc_addr_012b(uint8_t offset)
{
    return (__xdata uint8_t *)(0x012B + offset);
}

/*
 * nvme_set_ep_queue_ctrl_84 - Set endpoint queue control to 0x84
 * Address: 0x1cc1-0x1cc7 (7 bytes)
 *
 * Sets G_EP_QUEUE_CTRL to 0x84 (busy flag set).
 *
 * Original disassembly:
 *   1cc1: mov dptr, #0x0564
 *   1cc4: mov a, #0x84
 *   1cc6: movx @dptr, a       ; G_EP_QUEUE_CTRL = 0x84
 *   1cc7: ret
 */
void nvme_set_ep_queue_ctrl_84(void)
{
    G_EP_QUEUE_CTRL = 0x84;
}

/*
 * nvme_get_dev_status_upper - Get upper 2 bits of device status
 * Address: 0x1c56-0x1c5c (7 bytes)
 *
 * Reads NVMe device status register and masks to upper 2 bits.
 * These bits indicate device presence and ready state.
 *
 * Original disassembly:
 *   1c56: mov dptr, #0xc415   ; REG_NVME_DEV_STATUS
 *   1c59: movx a, @dptr       ; read status
 *   1c5a: anl a, #0xc0        ; mask bits 7-6
 *   1c5c: ret
 */
uint8_t nvme_get_dev_status_upper(void)
{
    return REG_NVME_DEV_STATUS & NVME_DEV_STATUS_MASK;
}

/*
 * nvme_get_data_ctrl_upper - Get upper 2 bits of data control
 * Address: 0x1d24-0x1d2a (7 bytes)
 *
 * Reads NVMe data control register and masks to upper 2 bits.
 *
 * Original disassembly:
 *   1d24: mov dptr, #0xc414   ; REG_NVME_DATA_CTRL
 *   1d27: movx a, @dptr       ; read control
 *   1d28: anl a, #0xc0        ; mask bits 7-6
 *   1d2a: ret
 */
uint8_t nvme_get_data_ctrl_upper(void)
{
    return REG_NVME_DATA_CTRL & NVME_DATA_CTRL_MASK;
}

/*
 * nvme_clear_status_bit1 - Clear bit 1 of NVMe status register
 * Address: 0x1cd5-0x1cdb (7 bytes)
 *
 * Reads status, clears bit 1, writes back.
 * Bit 1 is typically an error/interrupt flag.
 *
 * Original disassembly:
 *   1cd4: mov dptr, #0xc401   ; REG_NVME_STATUS
 *   1cd7: movx a, @dptr       ; read status
 *   1cd8: anl a, #0xfd        ; clear bit 1
 *   1cda: movx @dptr, a       ; write back
 *   1cdb: ret
 */
void nvme_clear_status_bit1(void)
{
    uint8_t val = REG_NVME_STATUS;
    val &= 0xFD;
    REG_NVME_STATUS = val;
}

/*
 * nvme_set_data_ctrl_bit7 - Set bit 7 of data control register
 * Address: 0x1d2b-0x1d31 (7 bytes)
 *
 * Reads from DPTR (caller sets it), clears bit 7, sets bit 7, writes back.
 * This is called after nvme_get_data_ctrl_upper with DPTR still pointing
 * to 0xC414.
 *
 * Original disassembly:
 *   1d2b: movx a, @dptr       ; read current value
 *   1d2c: anl a, #0x7f        ; clear bit 7
 *   1d2e: orl a, #0x80        ; set bit 7
 *   1d30: movx @dptr, a       ; write back
 *   1d31: ret
 */
void nvme_set_data_ctrl_bit7(void)
{
    uint8_t val = REG_NVME_DATA_CTRL;
    val = (val & 0x7F) | 0x80;
    REG_NVME_DATA_CTRL = val;
}

/*
 * nvme_store_idata_16 - Store 16-bit value to IDATA[0x16:0x17]
 * Address: 0x1d32-0x1d38 (7 bytes)
 *
 * Stores R6:R7 (hi:lo) to IDATA[0x16:0x17].
 *
 * Original disassembly:
 *   1d32: mov r1, #0x17
 *   1d34: mov @r1, a          ; store low byte (A = R7)
 *   1d35: mov a, r6           ; get high byte
 *   1d36: dec r1              ; point to 0x16
 *   1d37: mov @r1, a          ; store high byte
 *   1d38: ret
 */
void nvme_store_idata_16(uint8_t hi, uint8_t lo)
{
    *(__idata uint8_t *)0x17 = lo;
    *(__idata uint8_t *)0x16 = hi;
}

/*
 * nvme_calc_addr_04b7 - Calculate address in 0x04B7+ region
 * Address: 0x1ce4-0x1cef (12 bytes)
 *
 * Returns pointer to 0x04B7 + IDATA[0x23].
 *
 * Original disassembly:
 *   1ce4: mov a, #0xb7
 *   1ce6: add a, 0x23         ; A = 0xB7 + IDATA[0x23]
 *   1ce8: mov 0x82, a         ; DPL
 *   1cea: clr a
 *   1ceb: addc a, #0x04       ; DPH = 0x04 + carry
 *   1ced: mov 0x83, a
 *   1cef: ret
 */
__xdata uint8_t *nvme_calc_addr_04b7(void)
{
    uint8_t offset = *(__idata uint8_t *)0x23;
    uint16_t addr = 0x04B7 + offset;
    return (__xdata uint8_t *)addr;
}

/*
 * nvme_add_to_global_053a - Add 0x20 to value at 0x053A
 * Address: 0x1cdc-0x1ce3 (8 bytes)
 *
 * Reads value from 0x053A, adds 0x20, writes back.
 *
 * Original disassembly:
 *   1cdc: mov dptr, #0x053a
 *   1cdf: movx a, @dptr       ; read value
 *   1ce0: add a, #0x20        ; add 0x20
 *   1ce2: movx @dptr, a       ; write back
 *   1ce3: ret
 */
void nvme_add_to_global_053a(void)
{
    uint8_t val = G_NVME_PARAM_053A;
    val += 0x20;
    G_NVME_PARAM_053A = val;
}

/*
 * nvme_check_completion - Set completion bit on register
 * Address: 0x3244-0x3248 (5 bytes)
 *
 * Sets bit 0 of the register pointed to by the parameter.
 * Used to signal completion status.
 *
 * Original disassembly:
 *   3244: movx a, @dptr       ; read current value
 *   3245: anl a, #0xfe        ; clear bit 0
 *   3247: orl a, #0x01        ; set bit 0
 *   3247: movx @dptr, a       ; write back
 *   3248: ret
 */
void nvme_check_completion(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xFE) | 0x01;
}

/*
 * nvme_initialize - Initialize NVMe state
 * Address: 0x3249-0x3256 (14 bytes)
 *
 * Sets the target register to 1 and clears bit 0 of 0xC509.
 * Called to initialize NVMe state after command completion.
 *
 * Original disassembly:
 *   3249: mov a, #0x01
 *   324b: movx @dptr, a       ; *param = 1
 *   324c: mov dptr, #0xc509
 *   324f: movx a, @dptr       ; read 0xC509
 *   3250: anl a, #0xfe        ; clear bit 0
 *   3252: movx @dptr, a       ; write back
 *   3253: mov dptr, #0x0af5   ; setup for return
 *   3256: ret
 */
void nvme_initialize(__xdata uint8_t *ptr)
{
    uint8_t val;

    /* Set target to 1 */
    *ptr = 1;

    /* Clear bit 0 of 0xC509 */
    val = REG_NVME_LINK_STATUS;
    val &= 0xFE;
    REG_NVME_LINK_STATUS = val;
}

/*
 * nvme_ring_doorbell - Ring NVMe doorbell register
 * Address: 0x3247 (1 byte - just a write)
 *
 * Writes value to doorbell register at specified offset.
 * Used to notify NVMe device of new commands in queue.
 *
 * Original disassembly:
 *   3247: movx @dptr, a       ; write to doorbell
 *   3248: ret
 */
void nvme_ring_doorbell(__xdata uint8_t *doorbell)
{
    *doorbell = 0x00;  /* Ring by writing any value */
}

/*
 * nvme_read_and_sum_index - Read value and calculate indexed address
 * Address: 0x1c3b-0x1c49 (15 bytes)
 *
 * Reads from DPTR, adds to value from 0x0216, masks to 5 bits,
 * then writes to 0x01B4.
 *
 * Original disassembly:
 *   1c3a: movx a, @dptr        ; Read from caller's DPTR
 *   1c3b: mov r7, a
 *   1c3c: mov dptr, #0x0216
 *   1c3f: movx a, @dptr        ; Read [0x0216]
 *   1c40: mov r6, a
 *   1c41: mov a, r7
 *   1c42: add a, r6            ; A = R7 + R6
 *   1c43: anl a, #0x1f         ; Mask to 5 bits
 *   1c45: mov dptr, #0x01b4
 *   1c48: movx @dptr, a        ; Write to [0x01B4]
 *   1c49: ret
 */
void nvme_read_and_sum_index(__xdata uint8_t *ptr)
{
    uint8_t val1, val2, result;

    val1 = *ptr;
    val2 = G_DMA_WORK_0216;
    result = (val1 + val2) & 0x1F;
    G_USB_WORK_01B4 = result;
}

/*
 * nvme_write_params_to_dma - Write value to DMA mode and param registers
 * Address: 0x1c4a-0x1c54 (11 bytes)
 *
 * Writes A to 0x0203, 0x020D, and 0x020E.
 *
 * Original disassembly:
 *   1c4a: mov dptr, #0x0203
 *   1c4d: movx @dptr, a        ; [0x0203] = A
 *   1c4e: mov dptr, #0x020d
 *   1c51: movx @dptr, a        ; [0x020D] = A
 *   1c52: inc dptr
 *   1c53: movx @dptr, a        ; [0x020E] = A
 *   1c54: ret
 */
void nvme_write_params_to_dma(uint8_t val)
{
    G_DMA_MODE_SELECT = val;
    G_DMA_PARAM1 = val;
    G_DMA_PARAM2 = val;
}

/*
 * nvme_calc_addr_from_dptr - Calculate address from DPTR value + 0xA8
 * Address: 0x1c5e-0x1c6c (15 bytes)
 *
 * Reads from DPTR, adds 0xA8, forms address in 0x05XX region,
 * reads that address and stores to 0x05A6.
 *
 * Original disassembly:
 *   1c5d: movx a, @dptr        ; Read from caller's DPTR
 *   1c5e: add a, #0xa8         ; A = A + 0xA8
 *   1c60: mov 0x82, a          ; DPL
 *   1c62: clr a
 *   1c63: addc a, #0x05        ; DPH = 0x05 + carry
 *   1c65: mov 0x83, a
 *   1c67: movx a, @dptr        ; Read from 0x05XX
 *   1c68: mov dptr, #0x05a6
 *   1c6b: movx @dptr, a        ; Store to [0x05A6]
 *   1c6c: ret
 */
void nvme_calc_addr_from_dptr(__xdata uint8_t *ptr)
{
    uint8_t val = *ptr;
    uint16_t addr = 0x0500 + val + 0xA8;
    uint8_t result = *(__xdata uint8_t *)addr;
    G_PCIE_TXN_COUNT_LO = result;
}

/*
 * nvme_copy_idata_to_dptr - Copy 2 bytes from IDATA[0x16-0x17] to DPTR
 * Address: 0x1cc8-0x1cd3 (12 bytes)
 *
 * Reads IDATA[0x16:0x17] and writes to consecutive DPTR addresses.
 *
 * Original disassembly:
 *   1cc8: mov r0, #0x16
 *   1cca: mov a, @r0           ; A = IDATA[0x16]
 *   1ccb: mov r7, a
 *   1ccc: inc r0
 *   1ccd: mov a, @r0           ; A = IDATA[0x17]
 *   1cce: xch a, r7            ; Swap
 *   1ccf: movx @dptr, a        ; Write IDATA[0x16] to DPTR
 *   1cd0: inc dptr
 *   1cd1: mov a, r7
 *   1cd2: movx @dptr, a        ; Write IDATA[0x17] to DPTR+1
 *   1cd3: ret
 */
void nvme_copy_idata_to_dptr(__xdata uint8_t *ptr)
{
    uint8_t hi = *(__idata uint8_t *)0x16;
    uint8_t lo = *(__idata uint8_t *)0x17;
    ptr[0] = hi;
    ptr[1] = lo;
}

/*
 * nvme_get_pcie_count_config - Read and calculate PCIe transaction config
 * Address: 0x1c90-0x1c9e (15 bytes)
 *
 * Reads 0x05A6, multiplies by 0x22, adds to 0x05B4 index, reads result.
 *
 * Original disassembly:
 *   1c90: mov dptr, #0x05a6
 *   1c93: movx a, @dptr        ; A = [0x05A6]
 *   1c94: mov 0xf0, #0x22      ; B = 0x22
 *   1c97: mov dptr, #0x05b4
 *   1c9a: lcall 0x0dd1         ; table_index_read
 *   1c9d: movx a, @dptr
 *   1c9e: ret
 */
uint8_t nvme_get_pcie_count_config(void)
{
    uint8_t index = G_PCIE_TXN_COUNT_LO;
    uint16_t addr = 0x05B4 + (index * 0x22);
    return *(__xdata uint8_t *)addr;
}

/*
 * nvme_calc_dptr_0500_base - Calculate DPTR = 0x0500 + A (with carry)
 * Address: 0x3258-0x325e (7 bytes)
 *
 * Sets DPTR to 0x0500 + A. If carry is set from previous operation,
 * it will be added to the high byte.
 *
 * Original disassembly:
 *   3257: mov 0x82, a           ; DPL = A
 *   3259: clr a                 ; A = 0
 *   325a: addc a, #0x05         ; A = 5 + carry
 *   325c: mov 0x83, a           ; DPH = A
 *   325e: ret
 */
__xdata uint8_t *nvme_calc_dptr_0500_base(uint8_t val)
{
    return (__xdata uint8_t *)(0x0500 + val);
}

/*
 * nvme_calc_dptr_direct_with_carry - Calculate DPTR = A (with carry to high)
 * Address: 0x325f-0x3266 (8 bytes)
 *
 * Sets DPTR to A in low byte, carry in high byte.
 * Similar to usb_calc_dptr_direct but carries to high.
 *
 * Original disassembly:
 *   325f: mov 0x82, a           ; DPL = A
 *   3261: clr a                 ; A = 0
 *   3262: addc a, #0x00         ; A = 0 + carry
 *   3264: mov 0x83, a           ; DPH = A
 *   3266: ret
 */
__xdata uint8_t *nvme_calc_dptr_direct_with_carry(uint8_t val)
{
    /* Without actual carry flag, this just returns the value as an address */
    return (__xdata uint8_t *)val;
}

/*
 * nvme_setup_regs_from_a - Setup R0-R3 with A and memory at 0x06
 * Address: 0x3279-0x327f (7 bytes)
 *
 * Sets R3=A, R2=(0x06), R1=0, R0=0.
 * This is a register setup helper function.
 *
 * Original disassembly:
 *   3279: mov r3, a             ; R3 = A
 *   327a: mov r2, 0x06          ; R2 = value at address 0x06
 *   327c: clr a                 ; A = 0
 *   327d: mov r1, a             ; R1 = 0
 *   327e: mov r0, a             ; R0 = 0
 *   327f: ret
 *
 * Note: This function modifies registers directly, so C implementation
 * returns a struct to capture the 32-bit result.
 */
/* This function is typically called inline and modifies R0-R3 directly.
 * In C, we can't directly modify registers, so callers should use
 * inline assembly or handle this differently. For now, marking as stub. */

/*
 * nvme_init_step - Set EP configuration for NVMe endpoint setup
 * Address: 0x3267-0x3271 (11 bytes)
 *
 * Writes endpoint configuration values for NVMe mode.
 * Sets REG_USB_EP_CFG1 to 2 and REG_USB_EP_CFG2 to 0x10.
 *
 * Original disassembly:
 *   3267: mov dptr, #0x9093
 *   326a: mov a, #0x02
 *   326c: movx @dptr, a        ; REG_USB_EP_CFG1 = 2
 *   326d: inc dptr
 *   326e: mov a, #0x10
 *   3270: movx @dptr, a        ; REG_USB_EP_CFG2 = 0x10
 *   3271: ret
 */
void nvme_init_step(void)
{
    REG_USB_EP_CFG1 = 0x02;
    REG_USB_EP_CFG2 = 0x10;
}

/*
 * nvme_read_status - Set bit 4 on the given register pointer
 * Address: 0x3272-0x3278 (7 bytes)
 *
 * Reads the byte at ptr, clears bit 4, sets bit 4, writes back.
 * Used for status indication on NVMe registers.
 *
 * Original disassembly:
 *   3272: movx a, @dptr        ; read
 *   3273: anl a, #0xef         ; clear bit 4
 *   3275: orl a, #0x10         ; set bit 4
 *   3277: movx @dptr, a        ; write
 *   3278: ret
 */
void nvme_read_status(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xEF) | 0x10;
}

/*
 * nvme_set_int_aux_bit1 - Set bit 1 on interrupt auxiliary register
 * Address: 0x3280-0x3289 (10 bytes)
 *
 * Manipulates REG_INT_AUX_STATUS: clears bits 1 and 2, then sets bit 1.
 *
 * Original disassembly:
 *   3280: mov dptr, #0xc805
 *   3283: movx a, @dptr        ; read REG_INT_AUX_STATUS
 *   3284: anl a, #0xf9         ; clear bits 1 and 2
 *   3286: orl a, #0x02         ; set bit 1
 *   3288: movx @dptr, a        ; write back
 *   3289: ret
 */
void nvme_set_int_aux_bit1(void)
{
    uint8_t val = REG_INT_AUX_STATUS;
    val = (val & 0xF9) | 0x02;
    REG_INT_AUX_STATUS = val;
}

/*
 * nvme_get_link_status_masked - Get NVMe link status masked to bits 0-1
 * Address: 0x328a-0x3290 (7 bytes)
 *
 * Reads 0x9100 (USB peripheral status) and masks to lower 2 bits.
 *
 * Original disassembly:
 *   328a: mov dptr, #0x9100
 *   328d: movx a, @dptr        ; read 0x9100
 *   328e: anl a, #0x03         ; mask bits 0-1
 *   3290: ret
 */
uint8_t nvme_get_link_status_masked(void)
{
    return REG_USB_LINK_STATUS & USB_LINK_STATUS_MASK;
}

/*
 * nvme_set_ep_ctrl_bits - Set control bits on EP register (bits 1 and 2)
 * Address: 0x320d-0x3218 (12 bytes)
 *
 * Sets bit 1 on register, then sets bit 2 on same register.
 * Used for endpoint control configuration.
 *
 * Original disassembly:
 *   320c: movx a, @dptr        ; read
 *   320d: anl a, #0xfd         ; clear bit 1
 *   320f: orl a, #0x02         ; set bit 1
 *   3211: movx @dptr, a        ; write
 *   3212: movx a, @dptr        ; read again
 *   3213: anl a, #0xfb         ; clear bit 2
 *   3215: orl a, #0x04         ; set bit 2
 *   3217: movx @dptr, a        ; write
 *   3218: ret
 */
void nvme_set_ep_ctrl_bits(__xdata uint8_t *ptr)
{
    uint8_t val;

    /* Set bit 1 */
    val = *ptr;
    val = (val & 0xFD) | 0x02;
    *ptr = val;

    /* Set bit 2 */
    val = *ptr;
    val = (val & 0xFB) | 0x04;
    *ptr = val;
}

/*
 * nvme_set_usb_ep_ctrl_bit2 - Set just bit 2 on EP register
 * Address: 0x3212-0x3218 (7 bytes, alt entry point)
 *
 * Sets only bit 2 on the register - subset of nvme_set_ep_ctrl_bits.
 *
 * Original disassembly:
 *   3212: movx a, @dptr        ; read
 *   3213: anl a, #0xfb         ; clear bit 2
 *   3215: orl a, #0x04         ; set bit 2
 *   3217: movx @dptr, a        ; write
 *   3218: ret
 */
void nvme_set_usb_ep_ctrl_bit2(__xdata uint8_t *ptr)
{
    uint8_t val = *ptr;
    val = (val & 0xFB) | 0x04;
    *ptr = val;
}

/* External helper from protocol.c */
extern void dma_buffer_write(void);

/*
 * nvme_call_and_signal - Call function and signal via 0x90A1
 * Address: 0x3219-0x3222 (10 bytes)
 *
 * Calls function at 0x53c0, then writes 1 to 0x90A1.
 * The function at 0x53c0 copies IDATA[0x6F-0x72] to XDATA 0xD808-0xD80B.
 *
 * Original disassembly:
 *   3219: lcall 0x53c0         ; call function
 *   321c: mov dptr, #0x90a1
 *   321f: mov a, #0x01
 *   3221: movx @dptr, a        ; write 1 to 0x90A1
 *   3222: ret
 */
void nvme_call_and_signal(void)
{
    /* Call DMA buffer write helper */
    dma_buffer_write();

    /* Set the signal register */
    REG_USB_BULK_DMA_TRIGGER = 0x01;
}

/*
 * nvme_add_8_to_addr - Add 8 to 16-bit address in R1:R2
 * Address: 0x3223-0x322d (11 bytes)
 *
 * Takes a 16-bit address in R1:R2 (R1=low, R2=high) and adds 8.
 * Returns the result in DPTR.
 *
 * Original disassembly:
 *   3223: mov a, r1           ; A = R1 (low byte)
 *   3224: add a, #0x08        ; A += 8
 *   3226: mov r1, a           ; R1 = result low
 *   3227: clr a               ; A = 0
 *   3228: addc a, r2          ; A = R2 + carry
 *   3229: mov 0x82, r1        ; DPL = R1
 *   322b: mov 0x83, a         ; DPH = A
 *   322d: ret
 */
__xdata uint8_t *nvme_add_8_to_addr(uint8_t addr_lo, uint8_t addr_hi)
{
    uint16_t addr = ((uint16_t)addr_hi << 8) | addr_lo;
    addr += 8;
    return (__xdata uint8_t *)addr;
}

/*
 * nvme_set_buffer_flags - Set buffer length and system flags
 * Address: 0x323b-0x3248 (14 bytes)
 *
 * Writes 0x0A to G_BUFFER_LENGTH_HIGH (0x0054) and sets bits 1,4,5
 * on G_SYS_FLAGS_0052 (0x0052).
 *
 * Original disassembly:
 *   323b: mov dptr, #0x0054   ; G_BUFFER_LENGTH_HIGH
 *   323e: mov a, #0x0a        ; A = 10
 *   3240: movx @dptr, a       ; write 10
 *   3241: mov dptr, #0x0052   ; G_SYS_FLAGS_0052
 *   3244: movx a, @dptr       ; read flags
 *   3245: orl a, #0x32        ; set bits 1, 4, 5
 *   3247: movx @dptr, a       ; write back
 *   3248: ret
 */
void nvme_set_buffer_flags(void)
{
    G_BUFFER_LENGTH_HIGH = 0x0A;
    G_SYS_FLAGS_0052 |= 0x32;
}

/*
 * usb_validate_descriptor - Copy descriptor validation data
 * Address: 0x31fb-0x320b (17 bytes)
 *
 * Copies values from 0xCEB2-0xCEB3 to 0x0056-0x0057.
 * Used during USB descriptor validation.
 *
 * Original disassembly:
 *   31fb: mov dptr, #0xceb2
 *   31fe: movx a, @dptr        ; read 0xCEB2
 *   31ff: mov dptr, #0x0056
 *   3202: movx @dptr, a        ; write to 0x0056
 *   3203: mov dptr, #0xceb3
 *   3206: movx a, @dptr        ; read 0xCEB3
 *   3207: mov dptr, #0x0057
 *   320a: movx @dptr, a        ; write to 0x0057
 *   320b: ret
 */
void usb_validate_descriptor(void)
{
    G_USB_ADDR_HI_0056 = REG_USB_DESC_VAL_CEB2;
    G_USB_ADDR_LO_0057 = REG_USB_DESC_VAL_CEB3;
}

/*
 * nvme_get_idata_0d_r7 - Read IDATA[0x0D] into return value
 * Address: 0x3291-0x3297 (7 bytes)
 *
 * Reads the value at IDATA[0x0D] and returns it.
 * Also clears an internal counter (R5 in assembly).
 *
 * Original disassembly:
 *   3291: mov r0, #0x0d         ; R0 = 0x0D
 *   3293: mov a, @r0            ; A = IDATA[0x0D]
 *   3294: mov r7, a             ; R7 = A
 *   3295: clr a                 ; A = 0
 *   3296: mov r5, a             ; R5 = 0
 *   3297: ret
 */
uint8_t nvme_get_idata_0d_r7(void)
{
    return *(__idata uint8_t *)0x0D;
}

/*
 * nvme_get_dma_status_masked - Get DMA status masked to upper bits
 * Address: 0x3298-0x329e (7 bytes)
 *
 * Reads DMA status register 0xC8D9 and masks to upper 5 bits.
 *
 * Original disassembly:
 *   3298: mov dptr, #0xc8d9
 *   329b: movx a, @dptr        ; read 0xC8D9
 *   329c: anl a, #0xf8         ; mask to upper bits
 *   329e: ret
 */
uint8_t nvme_get_dma_status_masked(void)
{
    return REG_DMA_STATUS3 & DMA_STATUS3_UPPER;
}

/* Forward declaration for reg_wait_bit_clear from event_handler.c */
extern void reg_wait_bit_clear(uint16_t addr, uint8_t mask, uint8_t flags, uint8_t timeout);

/*
 * nvme_process_cmd - Process NVMe command with register wait
 * Address: 0x31a0 region (referenced from ghidra line 6571)
 *
 * Calculates a lookup table address based on param, reads configuration
 * from there, and calls reg_wait_bit_clear with the values.
 *
 * The address calculation:
 * - Low byte: param * 2 + 0xAD
 * - High byte: 0x5C, or 0x5B if there's overflow from low byte
 *
 * This points to a table at 0x5CAD containing 2-byte entries with
 * register wait configuration (mask and timeout).
 */
void nvme_process_cmd(uint8_t param)
{
    uint16_t addr;
    uint8_t *ptr;
    uint8_t low_byte;
    uint8_t high_byte;

    /* Calculate address with overflow handling */
    low_byte = param * 2 + 0xAD;
    high_byte = (param * 2 > 0x52) ? 0x5B : 0x5C;
    addr = ((uint16_t)high_byte << 8) | low_byte;

    ptr = (__xdata uint8_t *)addr;

    /* Call reg_wait_bit_clear with config from table
     * ptr[0] = mask, ptr[1] = flags/timeout */
    reg_wait_bit_clear(0x0A7E, ptr[1], 0x01, ptr[0]);
}

/*
 * nvme_io_request - Copy byte from one computed address to another
 * Address: 0x31a5 region (referenced from ghidra line 6597)
 *
 * This is a memory copy operation between two computed addresses.
 * The source and destination addresses are computed from the parameters.
 *
 * Source: (param2 + offset, param1 + param4) where offset depends on carry
 * Dest: (param3 - 0x80, param4)
 */
void nvme_io_request(uint8_t param1, __xdata uint8_t *param2, uint8_t param3, uint8_t param4)
{
    uint8_t src_lo, src_hi;
    uint8_t dst_lo, dst_hi;
    uint16_t src_addr, dst_addr;

    /* Calculate source address */
    src_lo = param1 + param4;
    /* Handle carry from addition */
    src_hi = *param2 + param3;
    if ((uint16_t)param1 + param4 > 0xFF) {
        src_hi--;
    }
    src_addr = ((uint16_t)src_hi << 8) | src_lo;

    /* Calculate destination address */
    dst_lo = param4;
    dst_hi = param3 - 0x80;
    dst_addr = ((uint16_t)dst_hi << 8) | dst_lo;

    /* Copy byte */
    *(__xdata uint8_t *)dst_addr = *(__xdata uint8_t *)src_addr;
}

/*
 * nvme_build_cmd - Calculate build command result
 * Address: 0x31da-0x31e0 (7 bytes)
 *
 * Returns a computed value based on input parameter.
 * If param > 0xF3, returns 0xFF; otherwise returns 0.
 *
 * Original disassembly:
 *   31da: clr a
 *   31db: mov r7, a
 *   31dc: mov a, param
 *   31dd: clr c
 *   31de: subb a, #0xf4       ; compare with 0xF4
 *   31e0: ret                 ; carry indicates result
 */
uint8_t nvme_build_cmd(uint8_t param)
{
    return (param > 0xF3) ? 0xFF : 0x00;
}

/*
 * nvme_get_ep_table_entry - Get endpoint table entry value
 * Address: 0x31ea-0x31fa (17 bytes)
 *
 * Reads index from DPTR, then reads from table at 0x057F + (index * 10).
 * This is an endpoint configuration lookup table.
 *
 * Original disassembly:
 *   31ea: movx a, @dptr        ; read index from DPTR
 *   31eb: mov 0xf0, #0x0a      ; B = 10
 *   31ee: mul ab               ; A = index * 10
 *   31ef: add a, #0x7f         ; A += 0x7F
 *   31f1: mov 0x82, a          ; DPL = A
 *   31f3: clr a                ; A = 0
 *   31f4: addc a, #0x05        ; A = 5 + carry
 *   31f6: mov 0x83, a          ; DPH = A  (DPTR = 0x057F + index*10)
 *   31f8: movx a, @dptr        ; read from table
 *   31f9: mov r7, a            ; return in R7
 *   31fa: ret
 */
uint8_t nvme_get_ep_table_entry(__xdata uint8_t *index_ptr)
{
    uint8_t index = *index_ptr;
    uint16_t addr = 0x057F + ((uint16_t)index * 10);
    return *(__xdata uint8_t *)addr;
}

/*
 * nvme_submit_cmd - Submit command to NVMe controller
 * Address: 0x31fb region (part of larger function)
 *
 * Note: This is related to descriptor validation.
 * The actual command submission goes through nvme_ring_doorbell.
 * This function is called as part of the command flow.
 */
void nvme_submit_cmd(void)
{
    /* This function is essentially usb_validate_descriptor in the binary,
     * but in the NVMe context it's called to finalize command state */
    usb_validate_descriptor();
}

/* These functions are defined in usb.c - use extern declarations */
extern uint16_t usb_read_status_pair(void);
extern void usb_copy_status_to_buffer(void);
extern void usb_set_transfer_active_flag(void);

/*
 * nvme_io_handler - Main NVMe I/O handler state machine
 * Address: 0x32a4-0x3418 (large function)
 *
 * Handles NVMe I/O operations based on current state in IDATA[0x6A].
 * This is a complex state machine that processes NVMe commands.
 *
 * States:
 * - State 2 (0x02): Process command based on XDATA[0x0002]
 * - Other states: Set transfer active and read status
 *
 * Note: This is a simplified implementation. The full function has
 * many more branches and state transitions.
 */
void nvme_io_handler(uint8_t param)
{
    uint8_t state;
    uint8_t cmd_type;
    uint8_t usb_status;

    state = *(__idata uint8_t *)0x6A;

    if (state != 0x02) {
        goto handle_default;
    }

    /* State 2: Check command type from XDATA[0x0002] */
    cmd_type = G_IO_CMD_STATE;

    if (cmd_type == 0xE3 || cmd_type == 0xFB || cmd_type == 0xE1) {
        /* These command types (0xE3=-0x1D, 0xFB=-0x05, 0xE1=-0x1F)
         * trigger the I/O path */

        if (cmd_type == 0xE3 || cmd_type == 0xFB) {
            /* Load IDATA dword, process status, store back */
            /* This is simplified - actual implementation needs
             * idata_load_dword, usb_read_status_pair, etc. */
            goto handle_io_path;
        }

        /* Check XDATA[0x0001] for additional processing */
        if (G_IO_CMD_TYPE != 0x07) {
            nvme_set_int_aux_bit1();
            /* Additional processing would go here */
            if (param == 0) {
                /* dma_setup_transfer(0, 3, 3); */
            }
            usb_status = REG_USB_STATUS;
            if ((usb_status & 0x01) == 0) {
                nvme_call_and_signal();
            }
            *(__idata uint8_t *)0x6A = 0x05;
            return;
        }

handle_io_path:
        /* Simplified I/O path handling */
        usb_read_status_pair();
        return;
    }

    /* cmd_type == 0xF9 (-7) also goes through special handling */
    if (cmd_type == 0xF9) {
        if (G_IO_CMD_TYPE != 0x07) {
            nvme_set_int_aux_bit1();
            if (param == 0) {
                /* dma_setup_transfer(0, 3, 3); */
            }
            *(__idata uint8_t *)0x6A = 0x05;
            return;
        }
        /* Fall through to I/O path */
        goto handle_io_path;
    }

handle_default:
    /* Default state: set transfer active and update status */
    usb_set_transfer_active_flag();
    nvme_read_status((__xdata uint8_t *)&REG_USB_STATUS);

    usb_status = REG_USB_STATUS;
    if (usb_status & 0x01) {
        nvme_check_completion((__xdata uint8_t *)0x905F);
        nvme_check_completion((__xdata uint8_t *)0x905D);
    }
}

/* Forward declaration for reg_wait_bit_set from event_handler.c */
extern void reg_wait_bit_set(uint16_t addr);

/*
 * nvme_wait_for_ready - Wait for NVMe ready state via bit poll
 * Address: 0x329f-0x32a3 (5 bytes)
 *
 * Calls reg_wait_bit_set with address 0x0A7E to wait for ready state.
 *
 * Original disassembly:
 *   329f: mov dptr, #0x0a7e
 *   32a2: ljmp 0x0ddd         ; reg_wait_bit_set
 */
void nvme_wait_for_ready(void)
{
    reg_wait_bit_set(0x0A7E);
}

/* Forward declaration for function called by init_registers */
extern void nvme_func_04da(uint8_t param);

/*
 * nvme_init_registers - Initialize NVMe and USB registers for new transfer
 * Address: 0x3419-0x3576 (large function)
 *
 * This function initializes the NVMe subsystem for a new transfer by:
 * 1. Checking USB transfer state at 0x0B41
 * 2. If IDATA[0x6A] is non-zero, jumps to continuation at 0x3577
 * 3. Otherwise clears many registers to initialize state:
 *    - 0x0B01, 0x053B, 0x00C2, 0x0517, 0x014E, 0x00E5 = 0
 *    - REG at 0xCE88 = 0
 * 4. Waits for bit 0 of 0xCE89 to be set
 * 5. Checks additional status bits and continues initialization
 *
 * Original disassembly (start):
 *   3419: mov dptr, #0x0b41
 *   341c: movx a, @dptr
 *   341d: jz 0x3424
 *   341f: mov r7, #0x01
 *   3421: lcall 0x04da
 *   3424: mov r0, #0x6a
 *   3426: mov a, @r0
 *   3427: jz 0x342c
 *   3429: ljmp 0x3577
 *   342c: clr a
 *   342d: mov dptr, #0x0b01
 *   3430: movx @dptr, a        ; [0x0B01] = 0
 *   ...
 */
void nvme_init_registers(void)
{
    uint8_t val;

    /* Check USB transfer state */
    val = G_USB_STATE_0B41;
    if (val != 0) {
        /* Call helper function with param 1 */
        nvme_func_04da(0x01);
    }

    /* Check IDATA[0x6A] state */
    val = *(__idata uint8_t *)0x6A;
    if (val != 0) {
        /* State is non-zero, skip initialization */
        /* In full implementation, this would ljmp to 0x3577 */
        return;
    }

    /* Initialize state registers to zero */
    G_USB_INIT_0B01 = 0;
    G_NVME_STATE_053B = 0;
    G_INIT_STATE_00C2 = 0;
    G_EP_INIT_0517 = 0;
    G_USB_INDEX_COUNTER = 0;  /* 0x014E */
    G_INIT_STATE_00E5 = 0;

    /* Clear SCSI DMA control register */
    REG_BULK_DMA_HANDSHAKE = 0;

    /* Wait for bit 0 of 0xCE89 to be set */
    do {
        val = REG_USB_DMA_STATE;
    } while ((val & USB_DMA_STATE_READY) == 0);

    /* Check bit 1 for error/abort condition */
    val = REG_USB_DMA_STATE;
    if (val & USB_DMA_STATE_CBW) {
        /* Error condition, abort initialization */
        return;
    }

    /* Check bit 4 of 0xCE86 */
    val = REG_USB_DMA_ERROR;
    if (val & 0x10) {
        /* Abort initialization */
        return;
    }

    /* Check transfer flag at 0x0AF8 */
    val = G_POWER_INIT_FLAG;
    if (val != 0x01) {
        /* Not ready for transfer */
        return;
    }

    /* Continue with full initialization sequence */
    /* Call function 0x4C98 */
    /* ... additional initialization would go here ... */
}

/*
 * nvme_get_queue_slot_value - Get IDATA calculated address value
 * Address: 0x1b07-0x1b0a (4 bytes)
 *
 * Returns value from address calculated from IDATA[0x3E].
 * Address = 0x0100 + IDATA[0x3E] + 0x71, with carry to high byte.
 *
 * Original disassembly:
 *   1b07: mov a, #0x71
 *   1b09: add a, 0x3e         ; A = 0x71 + IDATA[0x3E]
 *   1b0b: mov dpl, a
 *   ...
 */
uint8_t nvme_get_queue_slot_value(void)
{
    uint8_t offset = *(__idata uint8_t *)0x3E;
    uint16_t addr;

    /* Calculate address: if 0x3E + 0x71 > 0xFF, high byte is 0x01, else 0x00 */
    if (offset > 0x8E) {
        addr = 0x0100 + offset + 0x71 - 0x100;  /* Wrap around */
    } else {
        addr = 0x0100 + offset + 0x71;
    }

    return *(__xdata uint8_t *)addr;
}

/*
 * nvme_read_xdata_0100 - Get IDATA value from parameter address
 * Address: 0x1b0b-0x1b13 (9 bytes)
 *
 * Returns value from 0x01XX address where XX is the parameter.
 *
 * Original disassembly:
 *   1b0b: mov dpl, a          ; DPL = param
 *   1b0d: clr a
 *   1b0e: addc a, #0x01       ; DPH = 0x01 + carry from PSW
 *   1b10: mov dph, a
 *   1b12: movx a, @dptr
 *   1b13: ret
 */
uint8_t nvme_read_xdata_0100(uint8_t param)
{
    /* Simple read from 0x01XX address */
    return *(__xdata uint8_t *)(0x0100 + param);
}

/*
 * nvme_check_threshold_r5 - Check if BANK1_R5 > 0xF7
 * Address: 0x1b2b-0x1b2c (2 bytes)
 *
 * Returns 1 if value is <= 0xF7, 0 if > 0xF7.
 * Used for boundary checking in queue operations.
 *
 * Original disassembly:
 *   1b2b: mov a, r5           ; Get BANK1_R5 (need context)
 *   1b2c: setb c
 *   1b2d: subb a, #0xf8       ; Compare with 0xF8
 *   ...
 */
uint8_t nvme_check_threshold_r5(uint8_t val)
{
    return (val > 0xF7) ? 0 : 1;
}

/*
 * nvme_check_threshold_0x3e - Check if IDATA[0x3E] > 0xB1
 * Address: 0x1b38-0x1b3d (6 bytes)
 *
 * Returns 1 if IDATA[0x3E] <= 0xB1, 0 otherwise.
 *
 * Original disassembly:
 *   1b38: mov a, 0x3e         ; A = IDATA[0x3E]
 *   1b3a: setb c
 *   1b3b: subb a, #0xb2       ; Compare with 0xB2
 *   1b3d: ret                 ; Return with carry set if A < 0xB2
 */
uint8_t nvme_check_threshold_0x3e(void)
{
    uint8_t val = *(__idata uint8_t *)0x3E;
    return (val > 0xB1) ? 0 : 1;
}

/* usb_func_1b47 is defined in usb.c */
extern void usb_func_1b47(void);

/*
 * usb_check_status - Store parameter and set control status bit 1
 * Address: 0x1b4d-0x1b5c (16 bytes)
 *
 * Stores param_1 to *param_2, then sets bit 1 on REG_NVME_CTRL_STATUS.
 *
 * Original disassembly:
 *   1b4d: movx @dptr, a       ; *param_2 = param_1
 *   1b4e: mov dptr, #0xc412
 *   1b51: movx a, @dptr
 *   1b52: anl a, #0xfd        ; Clear bit 1
 *   1b54: orl a, #0x02        ; Set bit 1
 *   1b56: movx @dptr, a
 *   1b57: ret
 */
void usb_check_status(uint8_t param_1, __xdata uint8_t *param_2)
{
    *param_2 = param_1;
    REG_NVME_CTRL_STATUS = (REG_NVME_CTRL_STATUS & ~NVME_CTRL_STATUS_READY) | NVME_CTRL_STATUS_READY;
}

/*
 * usb_configure - Set bit 1 on register pointed to
 * Address: 0x1b58-0x1b5f (8 bytes)
 *
 * Reads value, clears bit 1, sets bit 1, writes back.
 *
 * Original disassembly:
 *   1b58: movx a, @dptr
 *   1b59: anl a, #0xfd        ; Clear bit 1
 *   1b5b: orl a, #0x02        ; Set bit 1
 *   1b5d: movx @dptr, a
 *   1b5e: ret
 */
void usb_configure(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xFD) | 0x02;
}

/*
 * nvme_get_idata_009f - Get value from 0x00XX calculated from IDATA[0x3E]
 * Address: 0x1b84-0x1b8c (9 bytes)
 *
 * Calculates address 0x0000 + IDATA[0x3E] + 0x9F, reads and returns.
 *
 * Original disassembly:
 *   1b84: mov a, 0x3e
 *   1b86: add a, #0x9f
 *   1b88: mov dpl, a
 *   1b8a: movx a, @dptr
 *   1b8b: ret
 */
uint8_t nvme_get_idata_009f(void)
{
    uint8_t offset = *(__idata uint8_t *)0x3E;
    uint16_t addr;

    /* Calculate address with potential overflow */
    if (offset > 0x60) {
        addr = 0x0100 + offset + 0x9F - 0x100;  /* Carry to high byte */
    } else {
        addr = offset + 0x9F;
    }

    return *(__xdata uint8_t *)addr;
}

/* usb_get_ep_config_indexed is defined in usb.c */
extern uint8_t usb_get_ep_config_indexed(void);

/* usb_read_buf_addr_pair is defined in usb.c */
extern uint16_t usb_read_buf_addr_pair(void);

/*
 * usb_data_handler - Set bit 0 on register
 * Address: 0x1bde-0x1be7 (same as nvme_set_usb_mode_bit)
 * Alt entry point for different use.
 *
 * Sets bit 0 on the register pointed to.
 */
void usb_data_handler(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xFE) | 0x01;
}

/*
 * nvme_get_table_5cad_entry - Get table entry from 0x5CAD table
 * Address: 0x1c2b-0x1c2f (5 bytes)
 *
 * Reads from table at 0x5CAD indexed by IDATA[0x3C] * 2 + param.
 *
 * Original disassembly:
 *   1c2a: mov dptr, #0x5cad
 *   1c2d: mov a, 0x3c
 *   1c2f: mov 0xf0, #0x02     ; B = 2
 *   1c32: mul ab              ; A = IDATA[0x3C] * 2
 *   1c33: add a, param        ; A = A + param
 *   1c35: add a, dpl
 *   ...
 */
uint8_t nvme_get_table_5cad_entry(uint8_t param)
{
    uint8_t idx = *(__idata uint8_t *)0x3C;
    uint16_t addr = 0x5CAD + (idx * 2) + param;
    return *(__xdata uint8_t *)(addr + 1);  /* Returns second byte of entry */
}

/*
 * nvme_store_masked_01b4 - Store masked value to 0x01B4
 * Address: 0x1c43-0x1c49 (7 bytes)
 *
 * Masks parameter to 5 bits and stores to 0x01B4.
 *
 * Original disassembly:
 *   1c43: anl a, #0x1f        ; Mask to 5 bits
 *   1c45: mov dptr, #0x01b4
 *   1c48: movx @dptr, a
 *   1c49: ret
 */
void nvme_store_masked_01b4(uint8_t param)
{
    G_USB_WORK_01B4 = param & 0x1F;
}

/*
 * nvme_get_dev_status_bits - Get device status upper bits (alias)
 * Address: 0x1c55-0x1c5c (8 bytes)
 *
 * Same as nvme_get_dev_status_upper but as separate entry point.
 * Returns upper 2 bits of REG_NVME_DEV_STATUS.
 */
uint8_t nvme_get_dev_status_bits(void)
{
    return REG_NVME_DEV_STATUS & NVME_DEV_STATUS_MASK;
}

/*
 * nvme_load_idata_dword_0e - Load IDATA dword and read register dword
 * Address: 0x1c7e-0x1c87 (10 bytes)
 *
 * Calls idata_load_dword(0x0E), then reg_read_dword(3).
 *
 * Original disassembly:
 *   1c7e: mov r0, #0x0e
 *   1c80: lcall 0x0d78        ; idata_load_dword
 *   1c83: mov r7, #0x03
 *   1c85: ljmp 0x0de5         ; reg_read_dword
 */
void nvme_load_idata_dword_0e(void)
{
    /* Load IDATA dword from offset 0x0E */
    uint32_t val;
    val = ((__idata uint8_t *)0x0E)[0];
    val |= ((uint32_t)((__idata uint8_t *)0x0E)[1]) << 8;
    val |= ((uint32_t)((__idata uint8_t *)0x0E)[2]) << 16;
    val |= ((uint32_t)((__idata uint8_t *)0x0E)[3]) << 24;

    /* Would call reg_read_dword(3) - reads register and stores result */
    /* For now this is a placeholder */
    (void)val;
}

/*
 * nvme_dispatch_and_check - Process command and check result
 * Address: 0x1c9f-0x1cad (15 bytes)
 *
 * Calls scsi_core_dispatch and FUN_CODE_4e6d, returns OR of R6 and R7.
 *
 * Original disassembly:
 *   1c9f: lcall 0x4ff2        ; scsi_core_dispatch
 *   1ca2: lcall 0x4e6d        ; FUN_CODE_4e6d
 *   1ca5: mov a, r7
 *   1ca6: orl a, r6           ; A = R7 | R6
 *   1ca8: mov r7, a
 *   1ca9: ret
 */
uint8_t nvme_dispatch_and_check(void)
{
    /* This would call scsi_core_dispatch() and FUN_CODE_4e6d() */
    /* Returns combined status from R6 | R7 */
    /* Placeholder implementation */
    return 0;
}

/*
 * nvme_get_addr_012b - Calculate address in 0x012B region (alternate entry)
 * Address: 0x1cb9-0x1cc0 (8 bytes)
 *
 * Returns pointer to 0x012B (no offset added in this version).
 *
 * Original disassembly:
 *   1cb9: mov a, #0x2b
 *   1cbb: mov dpl, a
 *   1cbd: clr a
 *   1cbe: addc a, #0x01
 *   1cc0: mov dph, a
 *   1cc2: ret
 */
__xdata uint8_t *nvme_get_addr_012b(void)
{
    return (__xdata uint8_t *)0x012B;
}

/*
 * nvme_clear_pcie_state - Clear buffer and state registers
 * Address: 0x1cf0-0x1d23 (52 bytes)
 *
 * Clears multiple XDATA locations for state reset.
 *
 * Original disassembly (partial):
 *   1cf0: mov dptr, #0x05b0
 *   1cf3: e4             clr a
 *   1cf4: movx @dptr, a
 *   ...
 */
void nvme_clear_pcie_state(void)
{
    /* Clear state registers */
    G_PCIE_ADDR_1 = 0;           /* 0x05B0 */
    G_PCIE_ADDR_2 = 0;           /* 0x05B1 */
    G_PCIE_ADDR_3 = 0;           /* 0x05B2 */
    G_PCIE_TXN_COUNT_LO = 0;     /* 0x05A6 */
    G_PCIE_TXN_COUNT_HI = 0;     /* 0x05A7 */
}

/*
 * nvme_calc_dptr_0100_base - Calculate DPTR = 0x0100 + A (with carry)
 * Address: 0x31d8-0x31df (8 bytes)
 *
 * Sets DPTR to 0x0100 + A. If carry is set from previous operation,
 * it will be added to the high byte.
 *
 * Original disassembly:
 *   31d8: mov 0x82, a           ; DPL = A
 *   31da: clr a                 ; A = 0
 *   31db: addc a, #0x01         ; A = 1 + carry
 *   31dd: mov 0x83, a           ; DPH = A
 *   31df: ret
 */
__xdata uint8_t *nvme_calc_dptr_0100_base(uint8_t val)
{
    return (__xdata uint8_t *)(0x0100 + val);
}

/*
 * nvme_process_queue_entries - Process NVMe queue entries
 * Address: 0x488f-0x4903 (117 bytes)
 *
 * Main NVMe queue processing loop. Sets error flag, then iterates through
 * up to 32 queue entries, processing each one based on link status.
 *
 * Key registers used:
 * - 0x06E6: G_STATE_FLAG_06E6 - Error/processing flag (set to 1 at start)
 * - 0xC520: REG_NVME_LINK_STATUS - Bit 1 indicates link ready
 * - 0x0464: G_SYS_STATUS_PRIMARY - Cleared when link not ready
 * - 0x0465: G_SYS_STATUS_SECONDARY - Cleared when link not ready
 * - 0x0AF8: G_POWER_INIT_FLAG - Power state check
 * - 0xC51A: REG_NVME_QUEUE_TRIGGER - Queue trigger
 * - 0xC51E: REG_NVME_QUEUE_STATUS - Queue status (bits 0-5 = queue index)
 * - IDATA 0x38: Local queue index
 * - IDATA 0x39: Loop counter
 * - IDATA 0x3A: Flags register
 *
 * Algorithm:
 * 1. Set G_STATE_FLAG_06E6 = 1
 * 2. Loop counter = 0
 * 3. While counter < 32:
 *    a. Check REG_NVME_LINK_STATUS bit 1
 *    b. If not set, clear statuses and exit
 *    c. If G_POWER_INIT_FLAG != 0:
 *       - Read from 0xC51A, call 0x4b25
 *       - Get queue index from REG_NVME_QUEUE_STATUS (& 0x3F)
 *    d. Else:
 *       - Get queue index from REG_NVME_QUEUE_STATUS (& 0x3F)
 *       - Call 0x3da1 with index
 *    e. Build flags at IDATA 0x3A = 0x75
 *    f. Calculate address 0x0171 + queue_index
 *    g. If value at that address != 0, set bit 7 of flags
 *    h. Calculate address 0x0108 + queue_index
 *    i. Write flags there
 *    j. Calculate address 0x0517 + queue_index
 *    k. Increment value there
 *    l. Write 0xFF to REG_NVME_QUEUE_TRIGGER
 *    m. Increment counter
 * 4. Return
 *
 * Original disassembly summary:
 *   488f: mov dptr, #0x06e6    ; G_STATE_FLAG_06E6
 *   4892: mov a, #0x01
 *   4894: movx @dptr, a        ; Set flag = 1
 *   4895: clr a
 *   4896: mov 0x39, a          ; Counter = 0
 *   4898: mov dptr, #0xc520    ; Loop start - REG_NVME_LINK_STATUS
 *   489b: movx a, @dptr
 *   489c: jnb 0xe0.1, 0x4903   ; If bit 1 not set, exit
 *   489f: clr a
 *   48a0-48a7: Clear 0x0464 and 0x0465
 *   48a8-48ac: Check G_POWER_INIT_FLAG
 *   ... (queue processing)
 *   48fa: inc 0x39             ; counter++
 *   48fc-4901: if counter < 32, loop back to 4898
 *   4903: ret
 */
void nvme_process_queue_entries(void)
{
    __idata uint8_t *counter = (__idata uint8_t *)0x39;
    __idata uint8_t *queue_idx = (__idata uint8_t *)0x38;
    __idata uint8_t *flags = (__idata uint8_t *)0x3A;
    uint8_t status;
    __xdata uint8_t *ptr;

    /* Set error/processing flag */
    G_STATE_FLAG_06E6 = 0x01;

    /* Initialize counter */
    *counter = 0;

    /* Process up to 32 queue entries */
    while (*counter < 0x20) {
        /* Check NVMe link status - bit 1 must be set */
        status = REG_NVME_LINK_STATUS;
        if (!(status & 0x02)) {
            /* Link not ready - exit loop */
            return;
        }

        /* Clear system status when processing */
        G_SYS_STATUS_PRIMARY = 0;
        G_SYS_STATUS_SECONDARY = 0;

        /* Check power init flag */
        if (G_POWER_INIT_FLAG != 0) {
            /* Power initialized - read trigger and call 0x4b25 */
            uint8_t trigger_val = REG_NVME_QUEUE_TRIGGER;
            (void)trigger_val;  /* TODO: call nvme_helper_4b25(trigger_val) */

            /* Get queue index from status register */
            *queue_idx = REG_NVME_QUEUE_STATUS & NVME_QUEUE_STATUS_IDX;
        } else {
            /* Not initialized - get index and call 0x3da1 */
            *queue_idx = REG_NVME_QUEUE_STATUS & NVME_QUEUE_STATUS_IDX;
            /* TODO: call nvme_helper_3da1(*queue_idx) */
        }

        /* Build flags: start with 0x75 */
        *flags = 0x75;

        /* Check address 0x0171 + queue_index (IDATA region via XDATA) */
        ptr = nvme_calc_dptr_0100_base(0x71 + *queue_idx);
        if (*ptr != 0) {
            /* Set bit 7 of flags if value is non-zero */
            *flags |= 0x80;
        }

        /* Write flags to address 0x0108 + queue_index */
        ptr = nvme_calc_dptr_0100_base(0x08 + *queue_idx);
        *ptr = *flags;

        /* Increment value at 0x0517 + queue_index */
        ptr = nvme_calc_dptr_0500_base(0x17 + *queue_idx);
        (*ptr)++;

        /* Write 0xFF to queue trigger to acknowledge */
        REG_NVME_QUEUE_TRIGGER = 0xFF;

        /* Increment counter */
        (*counter)++;
    }
}

/*
 * usb_addr_copy_to_regs - Copy USB address to endpoint and buffer registers
 * Address: 0x0206-0x022a (37 bytes)
 *
 * If R5 has bits 1 or 2 set, copies address from G_USB_ADDR (0x0056-0x0057)
 * to USB endpoint buffer (0x905B-0x905C) and USB EP buffer data (0xD802-0xD803).
 * Also writes 0xA0 to DMA config register 0xC8D4.
 *
 * Original disassembly:
 *   0206: mov a, r5
 *   0207: anl a, #0x06         ; check bits 1-2
 *   0209: jz 0x022b            ; if zero, skip
 *   020b: mov dptr, #0xc8d4
 *   020e: mov a, #0xa0
 *   0210: movx @dptr, a        ; write 0xA0 to 0xC8D4
 *   0211: mov dptr, #0x0056
 *   0214: movx a, @dptr
 *   0215: mov r2, a            ; R2 = XDATA[0x0056]
 *   0216: inc dptr
 *   0217: movx a, @dptr
 *   0218: mov r3, a            ; R3 = XDATA[0x0057]
 *   0219: mov dptr, #0x905b
 *   021c: mov a, r2
 *   021d: movx @dptr, a        ; write R2 to 0x905B
 *   021e: inc dptr
 *   021f: mov a, r3
 *   0220: movx @dptr, a        ; write R3 to 0x905C
 *   0221: mov dptr, #0xd802
 *   0224: mov a, r2
 *   0225: movx @dptr, a        ; write R2 to 0xD802
 *   0226: inc dptr
 *   0227: mov a, r3
 *   0228: movx @dptr, a        ; write R3 to 0xD803
 *   0229: sjmp 0x0247          ; continue
 */
static void usb_addr_copy_to_regs(uint8_t mode)
{
    uint8_t addr_hi, addr_lo;

    /* Check if bits 1 or 2 are set */
    if ((mode & 0x06) == 0) {
        return;
    }

    /* Write 0xA0 to DMA config register */
    REG_DMA_CONFIG = 0xA0;

    /* Read USB address from globals */
    addr_hi = G_USB_ADDR_HI_0056;
    addr_lo = G_USB_ADDR_LO_0057;

    /* Copy to USB endpoint buffer registers */
    REG_USB_EP_BUF_HI = addr_hi;
    REG_USB_EP_BUF_LO = addr_lo;  /* 0x905C */

    /* Copy to USB endpoint data registers */
    REG_USB_EP_BUF_DATA = addr_hi;
    REG_USB_EP_BUF_PTR_LO = addr_lo;  /* 0xD803 */
}

/*
 * nvme_set_transfer_flag - Set transfer flag and USB mode bit
 * Address: 0x312a-0x3139 (16 bytes)
 *
 * Sets G_TRANSFER_FLAG_0AF2 to 1, then sets bit 0 of REG_USB_EP0_CONFIG.
 *
 * Original disassembly:
 *   312a: mov dptr, #0x0af2
 *   312d: mov a, #0x01
 *   312f: movx @dptr, a        ; G_TRANSFER_FLAG_0AF2 = 1
 *   3130: mov dptr, #0x9006
 *   3133: movx a, @dptr
 *   3134: anl a, #0xfe         ; clear bit 0
 *   3136: orl a, #0x01         ; set bit 0
 *   3138: movx @dptr, a
 *   3139: ret
 */
static void nvme_set_transfer_flag(void)
{
    G_TRANSFER_FLAG_0AF2 = 0x01;
    REG_USB_EP0_CONFIG = (REG_USB_EP0_CONFIG & 0xFE) | 0x01;
}

/*
 * nvme_reg_set_bit7 - Set bit 7 of register pointed to by DPTR
 * Address: 0x31ce-0x31d4 (7 bytes)
 *
 * Generic read-modify-write: reads register via DPTR, clears bit 7,
 * sets bit 7, writes back. Used with registers that need bit 7 set.
 *
 * Original disassembly:
 *   31ce: movx a, @dptr
 *   31cf: anl a, #0x7f         ; clear bit 7
 *   31d1: orl a, #0x80         ; set bit 7
 *   31d3: movx @dptr, a
 *   31d4: ret
 */
static void nvme_reg_set_bit7(__xdata uint8_t *reg)
{
    uint8_t val = *reg;
    val = (val & 0x7F) | 0x80;
    *reg = val;
}

/*
 * nvme_reg_set_bit0 - Set bit 0 of register pointed to by DPTR
 * Address: 0x3133-0x3139 (7 bytes)
 *
 * Generic read-modify-write: reads register via DPTR, clears bit 0,
 * sets bit 0, writes back. Used with registers that need bit 0 set.
 *
 * Original disassembly:
 *   3133: movx a, @dptr
 *   3134: anl a, #0xfe         ; clear bit 0
 *   3136: orl a, #0x01         ; set bit 0
 *   3138: movx @dptr, a
 *   3139: ret
 */
static void nvme_reg_set_bit0(__xdata uint8_t *reg)
{
    uint8_t val = *reg;
    val = (val & 0xFE) | 0x01;
    *reg = val;
}

/*
 * nvme_link_restart - Restart NVMe link and increment counter
 * Address: 0x53d4-0x53e5 (18 bytes)
 *
 * Writes 0xFF to REG_NVME_LINK_CTRL (0xC51A), then reads G_LINK_COUNTER
 * at 0x012B, increments it (masked to 5 bits), and calls the power
 * check status function at 0x041c (which checks 0xE647).
 *
 * Original disassembly:
 *   53d4: mov dptr, #0xc51a
 *   53d7: mov a, #0xff
 *   53d9: movx @dptr, a        ; REG_NVME_LINK_CTRL = 0xFF
 *   53da: mov dptr, #0x012b
 *   53dd: movx a, @dptr        ; Read G_LINK_COUNTER
 *   53de: inc a                ; Increment
 *   53df: anl a, #0x1f         ; Mask to 5 bits
 *   53e1: mov r7, a            ; Save to R7
 *   53e2: movx @dptr, a        ; Write back
 *   53e3: ljmp 0x041c          ; Jump to power_check_status_e647
 */
static void nvme_link_restart(void)
{
    uint8_t counter;

    /* Write 0xFF to link control register (0xC51A) */
    REG_NVME_QUEUE_TRIGGER = 0xFF;

    /* Read, increment, and mask counter */
    counter = G_WORK_012B;
    counter = (counter + 1) & 0x1F;
    G_WORK_012B = counter;

    /* Call power check status */
    power_check_status_e647();
}

/*
 * nvme_state_handler - Handle NVMe state based on device mode
 * Address: 0x4784-0x47f1 (110 bytes)
 *
 * Handles different NVMe states based on device mode. Reads IDATA 0x6A
 * for state, then handles based on G_IO_CMD_TYPE value:
 * - State 3: Initialize if G_IO_CMD_TYPE == 5
 * - State 4: Link check/restart if G_IO_CMD_TYPE == 7
 * - State 3 with G_IO_CMD_TYPE == 3: Check USB status
 *
 * Original disassembly:
 *   4784: mov r0, #0x6a         ; Get state from IDATA 0x6A
 *   4786: mov a, @r0
 *   4787: add a, #0xfd          ; a = state - 3
 *   4789: jz 0x47be             ; if state == 3, goto handle_state_3
 *   478b: dec a                 ; a = state - 4
 *   478c: jz 0x47af             ; if state == 4, goto handle_state_4
 *   478e: dec a                 ; a = state - 5
 *   478f: jnz 0x47f2            ; if state != 5, goto error
 *   ... (state 5 handling = initialization)
 *   47f1: ret
 */
void nvme_state_handler(void)
{
    __idata uint8_t *state_ptr = (__idata uint8_t *)0x6A;
    uint8_t state = *state_ptr;
    uint8_t cmd_type;

    if (state == 3) {
        /* State 3 - Check command type for mode */
        cmd_type = G_IO_CMD_TYPE;
        if (cmd_type == 3) {
            /* Mode 3: Check USB status bit 0 */
            if (REG_USB_STATUS & USB_STATUS_DMA_READY) {
                /* USB active - call status handlers */
                nvme_call_and_signal();
                usb_addr_copy_to_regs(cmd_type);
            } else {
                /* USB not active - call DMA buffer write and signal */
                dma_buffer_write();
                REG_USB_BULK_DMA_TRIGGER = 0x01;
            }
        } else {
            /* Other modes - set transfer flag and bit 7 of EP0 config */
            nvme_set_transfer_flag();
            nvme_reg_set_bit7(&REG_USB_EP0_CONFIG);
        }
        /* Set state to 5 (done) */
        *state_ptr = 0x05;
        return;
    }

    if (state == 4) {
        /* State 4 - Check for mode 7 (link restart) */
        cmd_type = G_IO_CMD_TYPE;
        if (cmd_type == 7) {
            /* Mode 7 - jump to 0x53d4 (link restart) */
            nvme_link_restart();
            return;
        }
        /* Other modes - fall through to error */
    }

    if (state == 5) {
        /* State 5 - Initialization */
        cmd_type = G_IO_CMD_TYPE;
        if (cmd_type != 5) {
            /* Wrong mode - error */
            goto handle_error;
        }

        /* Mode 5 initialization */
        /* Set up NVMe doorbell register */
        REG_NVME_DOORBELL = (REG_NVME_DOORBELL & ~NVME_DOORBELL_BIT3) | NVME_DOORBELL_BIT3;  /* Set bit 3 */
        REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT0;  /* Clear bit 0 */
        REG_NVME_DOORBELL &= ~NVME_DOORBELL_BIT3;  /* Clear bit 3 */

        /* Jump to common exit */
        goto common_exit;
    }

handle_error:
    /* Error path - set transfer flag and bit 7 of EP0 config */
    nvme_set_transfer_flag();
    nvme_reg_set_bit7(&REG_USB_EP0_CONFIG);

common_exit:
    /* Check USB status and handle */
    if (REG_USB_STATUS & USB_STATUS_DMA_READY) {
        /* USB active */
        nvme_call_and_signal();
        usb_addr_copy_to_regs(cmd_type);
    } else {
        /* USB not active - call DMA buffer write and signal */
        dma_buffer_write();
        REG_USB_BULK_DMA_TRIGGER = 0x01;
    }

    /* Set state to 5 */
    *state_ptr = 0x05;
}

/*
 * nvme_queue_sync - Synchronize NVMe queue state
 * Address: 0x49e9-0x4a56 (110 bytes)
 *
 * Synchronizes NVMe queue state between controller and firmware.
 * Reads queue index from 0xC512, writes 0xFF to it (acknowledge),
 * then checks sync counters and handles pending operations.
 *
 * Key registers:
 * - 0xC512: REG_NVME_QUEUE_INDEX - Current queue index
 * - 0x009F: Queue sync reference
 * - 0x0517: Queue counter
 * - 0x00C2: Init state (G_INIT_STATE_00C2)
 * - 0x00E5: Init state 2 (G_INIT_STATE_00E5)
 * - 0xCEF3: REG_CPU_LINK_CEF3 - Link control register
 * - 0x0171: G_SCSI_CTRL - SCSI control
 * - 0x0B01: G_USB_INIT_0B01 - USB init state
 *
 * Original disassembly:
 *   49e9: mov dptr, #0xc512     ; REG_NVME_QUEUE_INDEX
 *   49ec: movx a, @dptr
 *   49ed: mov 0x38, a           ; Save index to IDATA 0x38
 *   49ef: mov a, #0xff
 *   49f1: movx @dptr, a         ; Acknowledge by writing 0xFF
 *   49f2: mov dptr, #0x009f     ; Read sync reference
 *   49f5: movx a, @dptr
 *   49f6: mov r7, a
 *   49f7: mov dptr, #0x0517     ; Read queue counter
 *   49fa: movx a, @dptr
 *   49fb: inc a                 ; Increment
 *   49fc: movx @dptr, a         ; Store back
 *   49fd: xrl a, r7             ; Compare with reference
 *   49fe: jnz 0x4a36            ; If not equal, skip main processing
 *   4a00: ... (main processing loop)
 *   4a56: ret
 */
void nvme_queue_sync(void)
{
    __idata uint8_t *queue_idx = (__idata uint8_t *)0x38;
    uint8_t sync_ref, counter_val;
    __xdata uint8_t *ptr;

    /* Read queue index and acknowledge */
    *queue_idx = REG_NVME_QUEUE_INDEX;
    REG_NVME_QUEUE_INDEX = 0xFF;

    /* Read sync reference from G_USB_WORK_009F */
    sync_ref = G_USB_WORK_009F;

    /* Increment queue counter at G_EP_INIT_0517 */
    G_EP_INIT_0517++;
    counter_val = G_EP_INIT_0517;

    /* If counter doesn't match reference, skip main processing */
    if (counter_val != sync_ref) {
        goto check_scsi;
    }

    /* Main processing loop - check init state and process */
    while (1) {
        sync_ref = G_INIT_STATE_00C2;
        counter_val = G_EP_INIT_0517;

        if (counter_val != sync_ref) {
            /* Check G_INIT_STATE_00E5 */
            if (G_INIT_STATE_00E5 != 0) {
                /* Call helper with params: R3=0x03, R5=0x47, R7=0x0B */
                /* TODO: call protocol_setup_params(0x03, 0x47, 0x0B) */
            }
            goto call_state_handler;
        }

        /* Check REG_CPU_LINK_CEF3 bit 3 */
        if (REG_CPU_LINK_CEF3 & CPU_LINK_CEF3_ACTIVE) {
            /* Set CEF3 to 0x08 and call dma_interrupt_handler */
            REG_CPU_LINK_CEF3 = CPU_LINK_CEF3_ACTIVE;
            /* TODO: call dma_interrupt_handler() */
            continue;  /* Loop back */
        }

        /* Call helper 0x0395 and check result */
        /* TODO: result = call_0395() */
        /* if (result == 0) continue; else break; */
        break;
    }
    return;

call_state_handler:
    /* Call nvme_state_handler */
    nvme_state_handler();
    goto check_scsi;

check_scsi:
    /* Check G_SCSI_CTRL (0x0171) for pending operations */
    if (G_SCSI_CTRL > 0) {
        /* Decrement SCSI control */
        G_SCSI_CTRL--;

        /* Check G_USB_INIT_0B01 */
        if (G_USB_INIT_0B01 != 0) {
            /* Call helper with queue index */
            /* TODO: call helper_4eb3(0, *queue_idx) */
        } else {
            /* Call alternative helper */
            /* TODO: call helper_46f8(0, *queue_idx) */
        }
    }
}

/*
 * nvme_queue_process_pending - Process pending NVMe queue operations
 * Address: 0x3e81-0x3f2b (171 bytes)
 *
 * Processes pending queue operations. Reads queue status from 0xC516,
 * increments queue counters, and handles completion based on various
 * state flags.
 *
 * Key registers:
 * - 0xC516: REG_NVME_QUEUE_PENDING - Queue status (bits 0-5: index)
 * - 0x009F: Queue reference base
 * - 0x00C2: Init state comparison
 * - 0x00E5: Secondary state flag
 * - 0xCEF3: REG_CPU_LINK_CEF3 - Link control
 * - 0xCEF2: REG_CPU_LINK_CEF2 - Link status
 * - 0x05AC: Buffer state flag
 *
 * Original disassembly:
 *   3e81: mov dptr, #0xc516     ; REG_NVME_QUEUE_PENDING
 *   3e84: movx a, @dptr
 *   3e85: anl a, #0x3f          ; Mask to 6 bits
 *   3e87: mov 0x38, a           ; Store to IDATA 0x38
 *   3e89: add a, #0x17          ; A = index + 0x17
 *   3e8b: lcall 0x3257          ; calc_dptr_0500_base
 *   3e8e: movx a, @dptr         ; Read queue counter
 *   3e8f: inc a                 ; Increment
 *   3e90: movx @dptr, a         ; Store back
 *   3e91: mov 0x39, a           ; Save new counter to IDATA 0x39
 *   3e93: mov a, #0x9f          ; Calculate 0x009F + index
 *   3e95: add a, 0x38
 *   3e97-3e9e: Read value and compare with counter
 *   ... (processing logic)
 *   3f2b: ret
 */
void nvme_queue_process_pending(void)
{
    __idata uint8_t *queue_idx = (__idata uint8_t *)0x38;
    __idata uint8_t *counter = (__idata uint8_t *)0x39;
    uint8_t status, ref_val;
    __xdata uint8_t *ptr;

    /* Read queue status and get index */
    status = REG_NVME_QUEUE_PENDING;
    *queue_idx = status & 0x3F;

    /* Increment counter at 0x0517 + index */
    ptr = nvme_calc_dptr_0500_base(0x17 + *queue_idx);
    (*ptr)++;
    *counter = *ptr;

    /* Calculate reference address: 0x009F + index */
    ptr = (__xdata uint8_t *)(0x009F + *queue_idx);
    ref_val = *ptr;

    /* Compare counter with reference */
    if (*counter != ref_val) {
        /* Not matched - jump to error handling */
        goto handle_mismatch;
    }

    /* Counter matches - check init state at 0x00C2 + index */
    ptr = (__xdata uint8_t *)(0x00C2 + *queue_idx);
    if (*ptr != *counter) {
        /* Check secondary state at 0x00E5 + index */
        ptr = (__xdata uint8_t *)(0x00E5 + *queue_idx);
        if (*ptr != 0) {
            /* Non-zero secondary state - call helper with params */
            /* TODO: call protocol_setup_params(0x03, 0x47, 0x0B) */
        }
        goto done;
    }

    /* Check link control register bit 3 */
    if (!(REG_CPU_LINK_CEF3 & CPU_LINK_CEF3_ACTIVE)) {
        /* Bit 3 not set - check buffer state at 0x05AC */
        ptr = (__xdata uint8_t *)0x05AC;
        if (*ptr == 0) {
            goto done;
        }

        /* Check REG_CPU_LINK_CEF2 bit 7 */
        if (REG_CPU_LINK_CEF2 & CPU_LINK_CEF2_READY) {
            goto done;
        }
    }

    /* Set link control bit 3 and call handler */
    REG_CPU_LINK_CEF3 = CPU_LINK_CEF3_ACTIVE;
    /* TODO: call dma_interrupt_handler() */
    return;

handle_mismatch:
    /* Handle counter mismatch - typically retry or error */
    /* TODO: Additional error handling */

done:
    return;
}

/* NOTE: nvme_queue_helper_180d removed - duplicate of usb_ep_loop_180d in usb.c (0x180d) */
/* NOTE: nvme_queue_helper removed - duplicate of nvme_cmd_status_init in event_handler.c (0x1196) */

/*===========================================================================
 * NVMe Command Engine Functions (0x9500-0x9900)
 *===========================================================================*/

/* Additional registers for NVMe command engine - now defined in registers.h */
/* The addresses below are kept as comments for reference:
 * REG_CMD_ISSUE           0xE400 (now defined in registers.h)
 * REG_CMD_TAG             0xE401 (now defined in registers.h)
 * REG_CMD_LBA_0           0xE422 (now defined in registers.h as REG_CMD_PARAM)
 * REG_CMD_LBA_1           0xE423
 * REG_CMD_LBA_2           0xE424
 * REG_CMD_LBA_3           0xE446
 * REG_CMD_COUNT_LOW       0xE425
 * REG_CMD_COUNT_HIGH      0xE426
 * REG_CMD_CTRL            0xE427
 * REG_CMD_TIMEOUT         0xE428
 * REG_CMD_PARAM_L         0xE42B
 * REG_CMD_PARAM_H         0xE42C
 * REG_CMD_PARAM           0xE430
 * REG_CMD_STATUS          0xE431
 * REG_INT_ENABLE       0xC801 (now defined in registers.h)
 */

/* Additional globals */
#define G_CMD_STATE_07C4        XDATA_VAR8(0x07C4)
#define G_WORK_07BF             XDATA_VAR8(0x07BF)
#define G_WORK_07C0             XDATA_VAR8(0x07C0)
#define G_WORK_07C1             XDATA_VAR8(0x07C1)
#define G_WORK_07C3             XDATA_VAR8(0x07C3)
#define G_WORK_07D5             XDATA_VAR8(0x07D5)
#define G_WORK_07DC             XDATA_VAR8(0x07DC)
#define G_WORK_07DD             XDATA_VAR8(0x07DD)
#define G_WORK_CC89             XDATA_VAR8(0xCC89)
#define G_WORK_CC88             XDATA_VAR8(0xCC88)
#define G_WORK_CC8A             XDATA_VAR8(0xCC8A)
#define G_WORK_CC99             XDATA_VAR8(0xCC99)
#define G_WORK_CC9A             XDATA_VAR8(0xCC9A)
#define G_WORK_CC9B             XDATA_VAR8(0xCC9B)
#define G_WORK_E41C             XDATA_VAR8(0xE41C)
#define G_WORK_E405             XDATA_VAR8(0xE405)
#define G_WORK_E420             XDATA_VAR8(0xE420)
#define G_WORK_E421             XDATA_VAR8(0xE421)
#define G_WORK_E409             XDATA_VAR8(0xE409)

/* External function declarations */
extern void wait_status_loop(void);

/*
 * nvme_cmd_store_and_trigger - Store parameter and trigger command
 * Address: 0x955d-0x9593 (55 bytes)
 */
void nvme_cmd_store_and_trigger(uint8_t param, __xdata uint8_t *ptr)
{
    ptr[1] = param;
    G_WORK_CC89 = 1;
}

/*
 * nvme_cmd_store_direct - Store parameter directly
 * Address: 0x955e-0x9577 (26 bytes)
 */
void nvme_cmd_store_direct(uint8_t param, __xdata uint8_t *ptr)
{
    *ptr = param;
    G_WORK_CC89 = 1;
}

/*
 * nvme_cmd_store_and_read - Store parameter and read work vars
 * Address: 0x957b-0x959f (37 bytes)
 */
uint8_t nvme_cmd_store_and_read(uint8_t param, __xdata uint8_t *ptr)
{
    uint8_t val;
    *ptr = param;
    val = G_WORK_07BF;
    val = G_WORK_07C0;
    return val;
}

/*
 * nvme_cmd_read_offset - Read value at offset+1 from pointer
 * Address: 0x9581-0x959f
 */
uint8_t nvme_cmd_read_offset(__xdata uint8_t *ptr)
{
    return ptr[1];
}

/*
 * nvme_cmd_issue_with_setup - Issue command with setup call
 * Address: 0x95a0-0x95a1
 */
void nvme_cmd_issue_with_setup(uint8_t param)
{
    cmd_param_setup(2, 0);
    REG_CMD_ISSUE = param;
    REG_CMD_TAG = *(__idata uint8_t *)0x03;  /* BANK0_R3 */
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_cmd_issue_alternate - Issue command alternate entry
 * Address: 0x95a2-0x95a4
 */
void nvme_cmd_issue_alternate(uint8_t param)
{
    cmd_param_setup(0, 0);
    REG_CMD_ISSUE = param;
    REG_CMD_TAG = *(__idata uint8_t *)0x03;
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_cmd_issue_simple - Simple command issue
 * Address: 0x95a5-0x95a7
 */
void nvme_cmd_issue_simple(uint8_t param)
{
    REG_CMD_ISSUE = param;
    REG_CMD_TAG = *(__idata uint8_t *)0x03;
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_cmd_issue_with_tag - Issue command with explicit tag
 * Address: 0x95a9-0x95aa
 */
void nvme_cmd_issue_with_tag(uint8_t param1, uint8_t param2)
{
    REG_CMD_ISSUE = param1;
    REG_CMD_TAG = param2;
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_cmd_store_pair_trigger - Store pair and trigger
 * Address: 0x95ab-0x95ae
 */
void nvme_cmd_store_pair_trigger(uint8_t param1, __xdata uint8_t *ptr, uint8_t param2)
{
    *ptr = param1;
    ptr[1] = param2;
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_cmd_set_state_6 - Set command state to 6
 * Address: 0x95b0-0x95b5
 */
void nvme_cmd_set_state_6(void)
{
    G_CMD_STATE_07C4 = 6;
}

/*
 * nvme_timer_init_95b6 - Initialize timer/CSR
 * Address: 0x95b7-0x95be
 */
void nvme_timer_init_95b6(void)
{
    G_WORK_CC9A = 0;
    G_WORK_CC9B = 0x50;
    G_WORK_CC99 = 4;
    G_WORK_CC99 = 2;
}

/*
 * nvme_timer_ack_95bf - Timer CSR acknowledge
 * Address: 0x95bf-0x95c4
 */
void nvme_timer_ack_95bf(void)
{
    G_WORK_CC99 = 4;
    G_WORK_CC99 = 2;
}

/*
 * nvme_timer_ack_ptr - Timer CSR acknowledge via pointer
 * Address: 0x95c5-0x95c8
 */
void nvme_timer_ack_ptr(__xdata uint8_t *ptr)
{
    *ptr = 2;
}

/*
 * nvme_cmd_clear_5_bytes - Clear 5 bytes at pointer
 * Address: 0x95f9-0x9604
 */
void nvme_cmd_clear_5_bytes(__xdata uint8_t *ptr)
{
    ptr[0] = 0;
    ptr[1] = 0;
    ptr[2] = 0;
    ptr[3] = 0;
    ptr[4] = 0;
}

/*
 * nvme_cmd_set_bit1_e41c - Set bit 1 on E41C register
 * Address: 0x9605-0x9607
 */
void nvme_cmd_set_bit1_e41c(void)
{
    uint8_t val = G_WORK_E41C;
    G_WORK_E41C = (val & 0xFE) | 1;
}

/*
 * nvme_cmd_set_bit1_ptr - Set bit 1 on register via pointer
 * Address: 0x9608-0x960e
 */
void nvme_cmd_set_bit1_ptr(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xFE) | 1;
}

/*
 * nvme_cmd_shift_6 - Shift value right 6 bits and store
 * Address: 0x960f-0x9616
 */
void nvme_cmd_shift_6(__xdata uint8_t *ptr, uint8_t val)
{
    *ptr = val >> 6;
}

/*
 * nvme_int_ctrl_set_bit4 - Set bit 4 on interrupt control
 * Address: 0x9617-0x9620
 */
void nvme_int_ctrl_set_bit4(void)
{
    uint8_t val = REG_INT_ENABLE;
    REG_INT_ENABLE = (val & ~INT_ENABLE_SYSTEM) | INT_ENABLE_SYSTEM;
}

/*
 * nvme_cmd_clear_cc88 - Clear CC88 and CC8A
 * Address: 0x9621-0x9626
 */
void nvme_cmd_clear_cc88(void)
{
    uint8_t val = G_WORK_CC88;
    G_WORK_CC88 = val & 0xF8;
    G_WORK_CC8A = 0;
}

/*
 * nvme_cmd_store_clear_cc8a - Store param and clear CC8A
 * Address: 0x9627-0x962d
 */
void nvme_cmd_store_clear_cc8a(uint8_t param, __xdata uint8_t *ptr)
{
    *ptr = param;
    G_WORK_CC8A = 0;
}

/*
 * nvme_flash_check_xor5 - Check flash counter XOR 5
 * Address: 0x962e-0x9634
 */
uint8_t nvme_flash_check_xor5(void)
{
    uint8_t val = G_FLASH_OP_COUNTER;
    return val ^ 5;
}

/*
 * nvme_cmd_clear_e405_setup - Clear E405 and setup E421
 * Address: 0x9635-0x9646
 */
void nvme_cmd_clear_e405_setup(void)
{
    uint8_t val = G_WORK_E405;
    G_WORK_E405 = val & 0xF8;
    G_WORK_E421 = ((*(__idata uint8_t *)0x05) & 7) << 4;  /* BANK0_R5 */
}

/*
 * nvme_cmd_clear_bit4_mask - Clear bit 4 and mask to low 3 bits
 * Address: 0x9647-0x964e
 */
uint8_t nvme_cmd_clear_bit4_mask(__xdata uint8_t *ptr)
{
    *ptr = *ptr & 0xEF;
    return *ptr & 0xF8;
}

/*
 * nvme_cmd_set_cc89_2 - Set CC89 to 2
 * Address: 0x964f-0x9655
 */
void nvme_cmd_set_cc89_2(void)
{
    G_WORK_CC89 = 2;
}

/*
 * nvme_cmd_shift_6_store - Shift 6 and store
 * Address: 0x9656
 */
void nvme_cmd_shift_6_store(uint8_t val, __xdata uint8_t *ptr)
{
    *ptr = val >> 6;
}

/*
 * nvme_cmd_shift_2_mask3 - Shift 2 and mask to 2 bits
 * Address: 0x9657-0x965c
 */
void nvme_cmd_shift_2_mask3(uint8_t val, __xdata uint8_t *ptr)
{
    *ptr = (val >> 2) & 3;
}

/*
 * nvme_set_flash_counter_5 - Set flash counter to 5
 * Address: 0x965e-0x9663
 */
void nvme_set_flash_counter_5(void)
{
    G_FLASH_OP_COUNTER = 5;
}

/*
 * nvme_cmd_dd12_0x10 - Call dd12 with params 0, 0x10
 * Address: 0x9664-0x9674
 */
void nvme_cmd_dd12_0x10(void)
{
    cmd_trigger_params(0, 0x10);
}

/*
 * nvme_lba_combine - Combine LBA with 07DD
 * Address: 0x9677-0x9683
 */
uint8_t nvme_lba_combine(uint8_t val)
{
    uint8_t tmp = G_WORK_07DD;
    return val | (tmp * 4);
}

/*
 * nvme_set_flash_counter_1 - Set flash counter to 1
 * Address: 0x9684-0x9686
 */
void nvme_set_flash_counter_1(void)
{
    G_FLASH_OP_COUNTER = 1;
}

/*
 * nvme_set_ptr_1 - Set pointer value to 1
 * Address: 0x9687-0x968b
 */
void nvme_set_ptr_1(__xdata uint8_t *ptr)
{
    *ptr = 1;
}

/*
 * nvme_nop_968c - NOP function
 * Address: 0x968c
 */
void nvme_nop_968c(void)
{
    /* Empty - NOP */
}

/*
 * nvme_lba_combine_07dc - Combine with 07DC
 * Address: 0x968f-0x969c
 */
uint8_t nvme_lba_combine_07dc(__xdata uint8_t *ptr)
{
    uint8_t tmp = G_WORK_07DC;
    return *ptr | (tmp * 4);
}

/*
 * nvme_set_flash_counter_call_e1c6 - Set counter and call e1c6
 * Address: 0x969e-0x96a6
 */
uint8_t nvme_set_flash_counter_call_e1c6(uint8_t param1, uint8_t param2)
{
    G_FLASH_OP_COUNTER = param1;
    wait_status_loop();
    return param2;
}

/*
 * nvme_nop_96a7 - NOP function
 * Address: 0x96a7-0x96a8
 */
void nvme_nop_96a7(void)
{
    /* Empty */
}

/*
 * nvme_nop_96a9 - NOP function
 * Address: 0x96a9-0x96ad
 */
void nvme_nop_96a9(void)
{
    /* Empty */
}

/*
 * nvme_call_e73a - Call e73a and return R7
 * Address: 0x96ae-0x96b7
 */
uint8_t nvme_call_e73a(void)
{
    cmd_engine_clear();
    return *(__idata uint8_t *)0x07;  /* BANK0_R7 */
}

/*
 * nvme_read_offset_1 - Read at offset+1
 * Address: 0x96b8-0x96be
 */
uint8_t nvme_read_offset_1(uint8_t hi, uint8_t lo)
{
    uint16_t addr = ((uint16_t)hi << 8) | lo;
    return *(__xdata uint8_t *)(addr + 1);
}

/*
 * nvme_circular_inc_07c1 - Circular increment 07C1
 * Address: 0x96bf-0x96d3
 */
void nvme_circular_inc_07c1(void)
{
    uint8_t c1 = G_WORK_07C1;
    uint8_t d5 = G_WORK_07D5;
    G_WORK_07C1 = (c1 + 1) & (d5 - 1);
}

/*
 * nvme_cmd_calc_store - Calculate and store to work vars
 * Address: 0x96d4-0x96d5
 */
void nvme_cmd_calc_store(uint8_t param1, uint8_t param2)
{
    /* Complex calculation involving PSW carry - simplified */
    G_WORK_07BF = param2 - 0x1C;
    G_WORK_07C0 = param1;
}

/*
 * nvme_cmd_calc_store_3 - Calculate and store with ptr
 * Address: 0x96d6
 */
void nvme_cmd_calc_store_3(uint8_t param1, __xdata uint8_t *ptr, uint8_t param2)
{
    *ptr = param1;
    G_WORK_07BF = param1 - 0x1C;
    G_WORK_07C0 = param2;
}

/*
 * nvme_cmd_calc_store_2 - Calculate and store 2 params
 * Address: 0x96d7-0x96ed
 */
void nvme_cmd_calc_store_2(uint8_t param1, uint8_t param2)
{
    G_WORK_07BF = param1 - 0x1C;
    G_WORK_07C0 = param2;
}

/*
 * nvme_get_07c3_mul2 - Get 07C3 * 2
 * Address: 0x96ee-0x96ef
 */
uint8_t nvme_get_07c3_mul2(void)
{
    uint8_t val = G_WORK_07C3;
    return val * 2;
}

/*
 * nvme_cmd_store_e420 - Store to E420 and mask E409
 * Address: 0x96f7-0x9702
 */
void nvme_cmd_store_e420(uint8_t param1, __xdata uint8_t *ptr, uint8_t param2)
{
    uint8_t val;
    *ptr = param1;
    val = G_WORK_E420;
    G_WORK_E420 = val & 0xC0;
    val = G_WORK_E420;
    G_WORK_E420 = val | param2;
}

/*
 * nvme_get_work_07c0 - Get work vars and return 07C0
 * Address: 0x9703-0x9712
 */
uint8_t nvme_get_work_07c0(void)
{
    uint8_t val;
    val = G_WORK_07BF;
    val = G_WORK_07C0;
    return val;
}

/*
 * nvme_store_e420_mask_e409 - Store E420, mask E409
 * Address: 0x9713-0x971d
 */
uint8_t nvme_store_e420_mask_e409(uint8_t param)
{
    uint8_t val;
    G_WORK_E420 = param;
    val = G_WORK_E409;
    return val & 0xF1;
}

/*
 * nvme_calc_07c1_addr - Calculate address from 07C1
 * Address: 0x971e-0x9728
 */
uint8_t nvme_calc_07c1_addr(void)
{
    uint8_t val = G_WORK_07C1;
    return val * 0x20 + 0x42;
}

/*
 * nvme_set_bit6_ptr - Set bit 6 on pointer
 * Address: 0x9729-0x9740
 */
void nvme_set_bit6_ptr(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xBF) | 0x40;
}

/*===========================================================================
 * NVMe Queue Management Functions (0xa600-0xa800)
 *===========================================================================*/

/* Additional globals for queue management */
#define G_QUEUE_STATE_0AD7      XDATA_VAR8(0x0AD7)
#define G_QUEUE_STATE_0ADE      XDATA_VAR8(0x0ADE)
#define G_QUEUE_STATE_0ACB      XDATA_VAR8(0x0ACB)
#define G_QUEUE_STATE_0ACF      XDATA_VAR8(0x0ACF)
#define G_QUEUE_STATE_0AE0      XDATA_VAR8(0x0AE0)
#define G_QUEUE_STATE_0AE1      XDATA_VAR8(0x0AE1)
#define G_TRANSFER_FLAG_0AF2    XDATA_VAR8(0x0AF2)
#define G_USB_MODE_90E2         XDATA_VAR8(0x90E2)
#define G_USB_MODE_9092         XDATA_VAR8(0x9092)

/*
 * nvme_queue_get_e710_masked - Get E710 register masked
 * Address: 0xa62d-0xa636
 */
uint8_t nvme_queue_get_e710_masked(void)
{
    extern void uart_wait_tx_ready(void);
    uint8_t val;
    uart_wait_tx_ready();
    val = REG_LINK_WIDTH_E710;
    return val & LINK_WIDTH_MASK;
}

/*
 * nvme_queue_init_0ad7 - Initialize queue state 0AD7
 * Address: 0xa637-0xa638
 */
void nvme_queue_init_0ad7(void)
{
    G_QUEUE_STATE_0AD7 = 1;
    G_QUEUE_STATE_0ADE = 0;
}

/*
 * nvme_queue_set_0ad7 - Set queue state 0AD7
 * Address: 0xa639-0xa63b
 */
void nvme_queue_set_0ad7(uint8_t param)
{
    G_QUEUE_STATE_0AD7 = param;
    G_QUEUE_STATE_0ADE = 0;
}

/*
 * nvme_queue_store_clear_0ade - Store and clear 0ADE
 * Address: 0xa63c
 */
void nvme_queue_store_clear_0ade(uint8_t param, __xdata uint8_t *ptr)
{
    *ptr = param;
    G_QUEUE_STATE_0ADE = 0;
}

/*
 * nvme_queue_clear_0ade - Clear 0ADE
 * Address: 0xa63d-0xa643
 */
void nvme_queue_clear_0ade(void)
{
    G_QUEUE_STATE_0ADE = 0;
}

/*
 * nvme_queue_init_9100 - Initialize with 9100 read
 * Address: 0xa660-0xa665
 */
uint8_t nvme_queue_init_9100(void)
{
    uint8_t val;
    G_QUEUE_STATE_0AD7 = 1;
    val = REG_USB_LINK_STATUS;
    return val & 3;
}

/*
 * nvme_queue_get_9100 - Get 9100 masked
 * Address: 0xa666-0xa66c
 */
uint8_t nvme_queue_get_9100(void)
{
    uint8_t val = REG_USB_LINK_STATUS;
    return val & 3;
}

/*
 * nvme_queue_and_acb_af2 - AND ACB and AF2
 * Address: 0xa66d-0xa678
 */
uint8_t nvme_queue_and_acb_af2(void)
{
    uint8_t v1 = G_QUEUE_STATE_0ACB;
    uint8_t v2 = G_TRANSFER_FLAG_0AF2;
    return v2 & v1;
}

/*
 * nvme_queue_clear_usb_bit0 - Clear USB status bit 0
 * Address: 0xa679-0xa67e
 */
uint8_t nvme_queue_clear_usb_bit0(void)
{
    uint8_t val = REG_USB_STATUS;
    REG_USB_STATUS = val & 0xFE;
    val = REG_USB_CTRL_924C;
    return val & 0xFE;
}

/*
 * nvme_queue_set_usb_config - Set USB config bits
 * Address: 0xa687-0xa691
 */
void nvme_queue_set_usb_config(void)
{
    uint8_t val = REG_USB_CONFIG;
    REG_USB_CONFIG = (val & 0xFD) | 2;
    G_USB_MODE_9092 = 2;
}

/*
 * nvme_queue_set_9092 - Set 9092 register
 * Address: 0xa692-0xa699
 */
void nvme_queue_set_9092(uint8_t param)
{
    G_USB_MODE_9092 = param;
}

/*
 * nvme_queue_init_9e16 - Initialize 9E16 and 9E1D
 * Address: 0xa69a-0xa6ac
 */
void nvme_queue_init_9e16(void)
{
    REG_USB_CTRL_BUF_9E16 = 0x40;
    REG_USB_CTRL_BUF_9E17 = 0;
    REG_USB_CTRL_BUF_9E1D = 0x40;
    REG_USB_CTRL_BUF_9E1E = 0;
}

/*
 * nvme_queue_init_905x - Initialize 905F/905D/90E3/90A0
 * Address: 0xa6ad-0xa6c5
 */
void nvme_queue_init_905x(void)
{
    uint8_t val;
    val = REG_USB_EP_CTRL_905F;
    REG_USB_EP_CTRL_905F = val & 0xFE;
    val = REG_USB_EP_CTRL_905D;
    REG_USB_EP_CTRL_905D = val & 0xFE;
    REG_USB_EP_STATUS_90E3 = 1;
    REG_USB_CTRL_90A0 = 1;
}

/*
 * nvme_queue_config_9006 - Configure 9006 and 9094
 * Address: 0xa6c6-0xa6db
 */
void nvme_queue_config_9006(uint8_t param, __xdata uint8_t *ptr)
{
    uint8_t val;
    *ptr = param;
    val = REG_USB_EP0_CONFIG;
    REG_USB_EP0_CONFIG = val & 0xFE;
    val = REG_USB_EP0_CONFIG;
    REG_USB_EP0_CONFIG = val & 0x7F;
    REG_USB_EP_CFG2 = 1;
    REG_USB_EP_CFG2 = 8;
}

/*
 * nvme_queue_calc_0ae0_0ae1 - Calculate from 0AE0/0AE1
 * Address: 0xa6dc-0xa6ee
 */
uint8_t nvme_queue_calc_0ae0_0ae1(uint8_t param)
{
    uint8_t v1 = G_QUEUE_STATE_0AE1;
    uint8_t v0 = G_QUEUE_STATE_0AE0;
    /* Calculate with carry - simplified */
    return v0 - ((v1 < param) ? 1 : 0);
}

/*
 * nvme_queue_get_e302_shift - Get E302 shifted
 * Address: 0xa6ef-0xa6f5
 */
uint8_t nvme_queue_get_e302_shift(void)
{
    uint8_t val = REG_PHY_MODE_E302;
    return ((val & 0x30) >> 4) - 2;
}

/*
 * nvme_queue_shift_param - Shift parameter
 * Address: 0xa6f6-0xa6fc
 */
uint8_t nvme_queue_shift_param(uint8_t param)
{
    return (param >> 4) - 2;
}

/*
 * nvme_queue_mask_0acf - Mask 0ACF to 5 bits
 * Address: 0xa6fd-0xa703
 */
uint8_t nvme_queue_mask_0acf(void)
{
    uint8_t val = G_QUEUE_STATE_0ACF;
    return val & 0x1F;
}

/*
 * nvme_queue_clear_9003 - Clear 9003 register
 * Address: 0xa71b-0xa721
 */
void nvme_queue_clear_9003(void)
{
    REG_USB_EP0_LEN_H = 0;
}

/*
 * nvme_queue_set_bit0_ptr - Set bit 0 on pointer
 * Address: 0xa72b-0xa731
 */
void nvme_queue_set_bit0_ptr(__xdata uint8_t *ptr)
{
    *ptr = (*ptr & 0xFE) | 1;
}

/*
 * nvme_queue_get_9090_mask - Get 9090 masked to 7 bits
 * Address: 0xa732-0xa738
 */
uint8_t nvme_queue_get_9090_mask(void)
{
    uint8_t val = REG_USB_INT_MASK_9090;
    return val & 0x7F;
}

/*
 * nvme_queue_set_90e3_2 - Set 90E3 to 2
 * Address: 0xa739-0xa73f
 */
void nvme_queue_set_90e3_2(void)
{
    REG_USB_EP_STATUS_90E3 = 2;
}

/*===========================================================================
 * NVMe Admin Command Functions (0xaa00-0xac00)
 *===========================================================================*/

/* Additional globals for admin commands */
#define G_WORK_07CA             XDATA_VAR8(0x07CA)
#define G_WORK_0A57             XDATA_VAR8(0x0A57)
#define G_WORK_0A58             XDATA_VAR8(0x0A58)
#define G_WORK_E434             XDATA_VAR8(0xE434)
#define G_WORK_E435             XDATA_VAR8(0xE435)

/*
 * nvme_admin_check_param - Check admin parameter
 * Address: 0xaa2a-0xaa2f
 */
void nvme_admin_check_param(uint8_t param)
{
    if (param < 0x80) {
        return;
    }
    /* param >= 0x80, continue processing */
    return;
}

/*
 * nvme_admin_nop_aa30 - NOP function
 * Address: 0xaa30-0xaa32
 */
void nvme_admin_nop_aa30(void)
{
    /* Empty - NOP */
}

/*
 * nvme_admin_nop_aa33 - NOP function
 * Address: 0xaa33-0xaa35
 */
void nvme_admin_nop_aa33(void)
{
    /* Empty - NOP */
}

/*
 * nvme_admin_set_state_07c4 - Set command state from 07CA
 * Address: 0xaafb-0xab0c
 */
void nvme_admin_set_state_07c4(void)
{
    uint8_t val = G_WORK_07CA;
    if (val == 2) {
        G_CMD_STATE_07C4 = 0x16;
    } else {
        G_CMD_STATE_07C4 = 0x12;
    }
}

/*
 * nvme_admin_call_dd0e_95a0 - Call dd0e and 95a0
 * Address: 0xab0d-0xab15
 */
void nvme_admin_call_dd0e_95a0(void)
{
    extern void cmd_trigger_default(void);
    cmd_trigger_default();
    nvme_cmd_issue_with_setup(1);
}

/*
 * nvme_admin_abd4 - Admin function ABD4
 * Address: 0xabd4-0xabd6
 */
void nvme_admin_abd4(void)
{
    /* Stub - complex function */
}

/*
 * nvme_admin_abd7 - Admin function ABD7
 * Address: 0xabd7-0xabe8
 */
void nvme_admin_abd7(uint8_t param)
{
    /* Stub - complex function with xdata_store_dword calls */
}

/*
 * nvme_admin_abe9 - Admin function ABE9
 * Address: 0xabe9-0xabff
 */
void nvme_admin_abe9(uint8_t param1, uint8_t param2, uint8_t param3)
{
    /* Stub - complex function with xdata_store_dword calls */
}

/*===========================================================================
 * PCIe TLP/NVMe Handler Functions (0xb100-0xba00)
 *===========================================================================*/

/* Additional registers for PCIe/flash - now defined in registers.h */
/* REG_FLASH_CMD_ALT (0xC880), REG_FLASH_CSR_ALT (0xC881), etc. are in registers.h */
/* REG_CPU_INT_CTRL (CC81) and REG_CPU_DMA_INT (CC91) are now in registers.h */
#define G_WORK_CCCF9            XDATA_VAR8(0xCCF9)
#define G_WORK_CCD9             XDATA_VAR8(0xCCD9)

/* Additional globals for flash - now using alternate names to avoid conflicts */
/* G_FLASH_RESET_0AAA, G_FLASH_ERROR_0, G_FLASH_ERROR_1 are now in globals.h */
/* G_FLASH_LEN_LO (0x0AB1) and G_FLASH_LEN_HI (0x0AB2) are in globals.h */
/* These are DIFFERENT addresses (0x0AAB/0x0AAC vs 0x0AB1/0x0AB2): */
#define G_FLASH_LEN_LO_ALT          XDATA_VAR8(0x0AAB)  /* Alternate flash len low */
#define G_FLASH_LEN_HI_ALT          XDATA_VAR8(0x0AAC)  /* Alternate flash len high */

/*
 * nvme_pcie_init_b820 - PCIe/flash init
 * Address: 0xb820-0xb824
 */
void nvme_pcie_init_b820(void)
{
    extern void xdata_store_dword(__xdata uint8_t *ptr, uint32_t val);
    xdata_store_dword((__xdata uint8_t *)0x0AAD, 0);
    G_FLASH_LEN_LO = 0;
    G_FLASH_LEN_HI = 0;
}

/*
 * nvme_pcie_init_b825 - PCIe/flash init alternate
 * Address: 0xb825-0xb832
 */
void nvme_pcie_init_b825(void)
{
    extern void xdata_store_dword(__xdata uint8_t *ptr, uint32_t val);
    xdata_store_dword((__xdata uint8_t *)0x0AAD, 0);
    G_FLASH_LEN_LO = 0;
    G_FLASH_LEN_HI = 0;
}

/*
 * nvme_pcie_init_b833 - PCIe/flash init without LEN_HI
 * Address: 0xb833-0xb837
 */
void nvme_pcie_init_b833(void)
{
    extern void xdata_store_dword(__xdata uint8_t *ptr, uint32_t val);
    xdata_store_dword((__xdata uint8_t *)0x0AAD, 0);
    G_FLASH_LEN_LO = 0;
}

/*
 * nvme_pcie_init_b838 - PCIe/flash init minimal
 * Address: 0xb838-0xb847
 */
void nvme_pcie_init_b838(void)
{
    extern void xdata_store_dword(__xdata uint8_t *ptr, uint32_t val);
    xdata_store_dword((__xdata uint8_t *)0x0AAD, 0);
}

/*
 * nvme_pcie_get_b848 - Get flash command setup
 * Address: 0xb848-0xb84f
 */
uint8_t nvme_pcie_get_b848(uint8_t param)
{
    /* Returns modified flash command value */
    return param & 0xFC;
}

/*
 * nvme_pcie_store_b850 - Store and trigger
 * Address: 0xb850
 */
void nvme_pcie_store_b850(uint8_t param, __xdata uint8_t *ptr)
{
    ptr[1] = param;
}

/*
 * nvme_pcie_store_b851 - Store to pointer
 * Address: 0xb851-0xb8b8
 */
void nvme_pcie_store_b851(uint8_t param, __xdata uint8_t *ptr)
{
    *ptr = param;
}

/*
 * nvme_pcie_handler_b8b9 - PCIe timer/event handler
 * Address: 0xb8b9-0xba05
 */
void nvme_pcie_handler_b8b9(void)
{
    extern void usb_state_event_handler(void);
    extern void pcie_store_param_transfer(uint8_t param);
    extern void pcie_init_error_halt(void);
    extern void pcie_int_ctrl_link_init(void);
    extern void pcie_link_state_init(void);
    extern void timer_event_init(void);
    extern void protocol_state_dispatch(void);

    uint8_t val;

    /* Check Timer 3 */
    val = REG_TIMER3_CSR;
    if ((val >> 1) & 1) {
        usb_state_event_handler();
        REG_TIMER3_CSR = 2;
    }

    /* Check CPU interrupt ACK bit */
    val = REG_CPU_INT_CTRL;
    if ((val >> 1) & 1) {
        uint8_t cmd = G_FLASH_OP_COUNTER;
        if (cmd == 0x0E || cmd == 0x0D) {
            REG_CPU_INT_CTRL = CPU_INT_CTRL_ACK;
            if (G_FLASH_CMD_TYPE != 0) {
                pcie_store_param_transfer(0x3B);
            }
            pcie_init_error_halt();
        } else {
            pcie_int_ctrl_link_init();
            REG_CPU_INT_CTRL = CPU_INT_CTRL_ACK;
        }
    }

    /* Check CPU DMA interrupt */
    val = REG_CPU_DMA_INT;
    if ((val >> 1) & 1) {
        REG_CPU_DMA_INT = CPU_INT_CTRL_ACK;
        /* flash_func_0bc8(0xF8, 0x53, 0xFF) */
    }

    /* Check CC99 */
    val = G_WORK_CC99;
    if ((val >> 1) & 1) {
        uint8_t cmd = G_FLASH_CMD_TYPE;
        if (cmd == 2) {
            pcie_store_param_transfer(0x3C);
            pcie_link_state_init();
        } else if (cmd == 3) {
            pcie_store_param_transfer(0xFF);
        } else {
            timer_event_init();
            G_WORK_CC99 = 2;
        }
    }

    /* Check CCD9 */
    val = G_WORK_CCD9;
    if ((val >> 1) & 1) {
        G_WORK_CCD9 = 2;
        G_STATE_FLAG_0719 = 2;
    }

    /* Check CCF9 */
    val = G_WORK_CCCF9;
    if ((val >> 1) & 1) {
        G_WORK_CCCF9 = 2;
        protocol_state_dispatch();
    }
}

/*===========================================================================
 * NVMe Command Engine Functions (0x9000-0x93FF)
 *===========================================================================*/

/* Forward declarations */
extern void pcie_tunnel_init_c00d(void);
extern uint8_t check_link_with_delay_e6a7(void);
extern uint8_t queue_check_status_c4a9(void);
extern __xdata uint8_t * queue_calc_dptr_c44f(void);

/*
 * nvme_state_error_set_92bb - Set error state and call PCIe tunnel init
 * Address: 0x92bb-0x92c4 (10 bytes)
 *
 * Sets G_STATE_FLAG_06E6 = 1 to indicate error state,
 * then calls PCIe tunnel initialization to recover.
 *
 * Original disassembly:
 *   92bb: mov dptr, #0x06e6
 *   92be: mov a, #0x01
 *   92c0: movx @dptr, a      ; G_STATE_FLAG_06E6 = 1
 *   92c1: lcall 0xc00d       ; pcie_tunnel_init_c00d
 *   92c4: ret
 */
void nvme_state_error_set_92bb(void)
{
    G_STATE_FLAG_06E6 = 1;
    pcie_tunnel_init_c00d();
}

/*
 * nvme_queue_status_read - Call helper and store result to IDATA[0x24]
 * Address: 0x92b3-0x92ba (8 bytes)
 *
 * Calls helper_e6a7 which returns a value in R7,
 * stores that value to IDATA[0x24], and returns it.
 *
 * Original disassembly:
 *   92b3: lcall 0xe6a7
 *   92b6: mov 0x24, r7       ; IDATA[0x24] = R7
 *   92b8: mov a, 0x24        ; return value
 *   92ba: ret
 */
uint8_t nvme_queue_status_read(void)
{
    uint8_t result = check_link_with_delay_e6a7();
    *((__idata uint8_t *)0x24) = result;
    return result;
}

/*
 * nvme_queue_count_matches_9264 - Count queue entries matching criteria
 * Address: 0x9264-0x92af (76 bytes)
 *
 * Entry 0x9264: Clears counters, then falls through to 0x9267
 * Entry 0x9267: Takes initial count in A (IDATA[0x21])
 *
 * Loops calling queue_check_status_c4a9:
 *   - If carry clear, exit loop
 *   - Calls queue_calc_dptr_c44f, reads value
 *   - If value == 0x0F, increment IDATA[0x22]
 *   - If value == 0x01, increment IDATA[0x22]
 *   - Always increment IDATA[0x21]
 *   - Loop back
 *
 * After loop:
 *   - If IDATA[0x22] == R7 (param), set G_STATE_RESULT_06E8 = 0, return 1
 *   - Otherwise check XDATA[0x0002]:
 *     - If 0x12: check G_COUNTER_0B39 >= 0x14, if so set 0x044C = 1, return 1
 *     - If 0x00: set 0x044C = 1, return 1
 *     - Otherwise return 0
 *
 * Original disassembly:
 *   9264: clr a
 *   9265: mov 0x22, a        ; counter2 = 0
 *   9267: mov 0x21, a        ; counter1 = 0
 *   9269: lcall 0xc4a9       ; queue_check_status_c4a9
 *   926c: jnc 0x9281         ; if no carry, exit loop
 *   926e: lcall 0xc44f       ; queue_calc_dptr_c44f
 *   9271: movx a, @dptr
 *   9272: mov r6, a
 *   9273: xrl a, #0x0f       ; check if == 0x0F
 *   9275: jz 0x927b          ; if 0x0F, inc counter2
 *   9277: mov a, r6
 *   9278: cjne a, #0x01, 0x927d  ; if != 0x01, skip inc counter2
 *   927b: inc 0x22           ; counter2++
 *   927d: inc 0x21           ; counter1++
 *   927f: sjmp 0x9269        ; loop
 *   9281: mov a, 0x22        ; get counter2
 *   9283: cjne a, 0x07, 0x928e  ; if != R7 (param), branch
 *   9286: clr a
 *   9287: mov dptr, #0x06e8
 *   928a: movx @dptr, a      ; G_STATE_RESULT_06E8 = 0
 *   928b: mov r7, #0x01
 *   928d: ret                ; return 1
 *   928e: mov dptr, #0x0002
 *   9291: movx a, @dptr      ; read XDATA[0x0002]
 *   9292: add a, #0xee       ; check if == 0x12 (0x12 + 0xee = 0x100, sets Z)
 *   9294: jz 0x92a8          ; if 0x12, branch to set flag
 *   9296: add a, #0x12       ; restore, check if == 0x00
 *   9298: jnz 0x92b0         ; if not 0, return 0
 *   929a: mov dptr, #0x0b39  ; G_COUNTER_0B39
 *   929d: movx a, @dptr
 *   929e: setb c
 *   929f: subb a, #0x14      ; check if >= 0x14
 *   92a1: jc 0x92b0          ; if < 0x14, return 0
 *   92a3: mov dptr, #0x044c
 *   92a6: sjmp 0x92ab
 *   92a8: mov dptr, #0x044c
 *   92ab: mov a, #0x01
 *   92ad: movx @dptr, a      ; 0x044C = 1
 *   92ae: mov r7, a
 *   92af: ret                ; return 1
 *   92b0: mov r7, #0x00
 *   92b2: ret                ; return 0
 */
uint8_t nvme_queue_count_matches_9264(uint8_t expected_count)
{
    __idata uint8_t *counter1 = (__idata uint8_t *)0x21;
    __idata uint8_t *counter2 = (__idata uint8_t *)0x22;
    __xdata uint8_t *ptr;
    uint8_t val;
    uint8_t cmd;

    /* Clear counters */
    *counter2 = 0;
    *counter1 = 0;

    /* Loop checking queue entries */
    while (queue_check_status_c4a9()) {
        ptr = queue_calc_dptr_c44f();
        val = *ptr;

        if (val == 0x0F || val == 0x01) {
            (*counter2)++;
        }
        (*counter1)++;
    }

    /* Check if counter2 matches expected */
    if (*counter2 == expected_count) {
        G_WORK_06E8 = 0;
        return 1;
    }

    /* Check command type at XDATA[0x0002] */
    cmd = G_IO_CMD_STATE;
    if (cmd == 0x12) {
        /* Command 0x12: set flag and return 1 */
        G_LOG_ACTIVE_044C = 1;
        return 1;
    } else if (cmd == 0x00) {
        /* Command 0x00: check counter threshold */
        if (G_STATE_0B39 >= 0x14) {
            G_LOG_ACTIVE_044C = 1;
            return 1;
        }
    }

    return 0;
}


/* ============================================================
 * NVMe Queue Utility Functions
 * ============================================================ */

void nvme_util_advance_queue(void)
{
    uint8_t i, limit;

    nvme_check_scsi_ctrl();

    limit = G_NVME_STATE_053B;  /* Command count */
    for (i = 0; i <= limit; i++) {
        nvme_calc_addr_04b7();  /* Uses G_SCSI_CTRL internally */
        if (G_USB_INDEX_COUNTER == G_NVME_STATE_053B) {
            nvme_calc_addr_04b7();
            G_USB_INDEX_COUNTER = 0xff;
            G_SCSI_CTRL--;
            if (G_USB_INIT_0B01 != 0) {
                /* FUN_CODE_4eb3() - error recovery */
                return;
            }
            nvme_util_get_queue_depth(1, i);
            return;
        }
    }
}

void nvme_util_check_command_ready(void)
{
    uint8_t pending, status;

    pending = REG_NVME_QUEUE_PENDING & 0x3F;
    I_WORK_38 = pending;

    /* Increment and store back */
    I_WORK_39 = REG_NVME_QUEUE_PENDING + 1;
    REG_NVME_QUEUE_PENDING = I_WORK_39;

    /* Wait for queue ready with interrupt check */
    while (1) {
        status = REG_CPU_LINK_CEF3;
        if ((status >> 3) & 1) {
            REG_CPU_LINK_CEF3 = 8;  /* Clear interrupt */
            dma_interrupt_handler();
        }

        /* Check completion */
        if ((REG_NVME_LINK_STATUS >> 1) & 1) {
            break;  /* Ready */
        }
    }
}

uint8_t nvme_util_clear_completion(void)
{
    uint8_t i, status, pending;

    G_STATE_FLAG_06E6 = 1;

    for (i = 0; i < 0x20; i++) {
        status = REG_NVME_LINK_STATUS;
        if (((status >> 1) & 1) == 0) {
            return status;
        }

        G_SYS_STATUS_PRIMARY = 0;
        G_SYS_STATUS_SECONDARY = 0;

        /* Process based on transfer flag */
        pending = REG_NVME_QUEUE_STATUS & 0x3F;
        I_WORK_38 = pending;

        usb_get_descriptor_ptr();

        REG_NVME_QUEUE_TRIGGER = 0xFF;
    }

    return 0;
}

void nvme_util_get_queue_depth(uint8_t p1, uint8_t p2)
{
    uint8_t param_for_aad;
    uint8_t scsi_ctrl;
    uint8_t temp;

    /* Store parameters */
    G_EP_DISPATCH_VAL3 = p2;   /* param_3 in ghidra */
    G_EP_DISPATCH_VAL4 = p1;   /* param_2 in ghidra */

    /* Call helper functions */
    core_state_read16();

    /* Check stub_return_zero result */
    if (stub_return_zero() == 0) {
        /* If helper returns 0, do buffer address pair processing */
        usb_read_buf_addr_pair();
        nvme_subtract_idata_16(0, 0);  /* Simplified - actual uses R6/R7 */
    }

    /* Set EP queue control */
    nvme_set_ep_queue_ctrl_84();

    /* Update queue status */
    G_EP_QUEUE_STATUS = G_EP_DISPATCH_VAL3;

    /* Select param for queue_param_buf_calc based on mode */
    param_for_aad = (G_EP_DISPATCH_VAL4 == 1) ? 1 : 2;
    queue_param_buf_calc(param_for_aad);

    /* Initialize PCIe transaction state */
    usb_init_pcie_txn_state();

    /* Clear high bit of NVME data control */
    REG_NVME_DATA_CTRL &= ~NVME_DATA_CTRL_BIT7;

    /* Handle SCSI control state */
    scsi_ctrl = G_SCSI_CTRL;
    if (scsi_ctrl == 0) {
        /* Call nvme_status_update */
        nvme_status_update();
    } else {
        /* Update G_EP_DISPATCH_VAL3 with device status */
        temp = G_EP_DISPATCH_VAL3;
        temp = temp + G_DMA_WORK_0216;
        temp |= nvme_get_dev_status_upper();
        G_EP_DISPATCH_VAL3 = temp;
    }

    /* Update with command param upper bits */
    G_EP_DISPATCH_VAL3 |= nvme_get_cmd_param_upper();

    /* Handle queue increment based on SCSI control */
    if (scsi_ctrl != 0) {
        usb_add_masked_counter(G_DMA_WORK_0216);
        temp = scsi_ctrl - 1;  /* Simplified from FUN_CODE_4f37 */
    } else {
        temp = scsi_ctrl - 1;
    }
    (void)temp;  /* temp used for logic but function doesn't take param */
    nvme_inc_circular_counter();

    /* Power check if mode flag is 0 */
    if (G_EP_DISPATCH_VAL4 == 0) {
        power_check_status(G_USB_PARAM_0B00);
    }
}

void nvme_queue_state_update(uint8_t param)
{
    uint8_t status;
    uint8_t new_val;

    status = G_SYS_STATUS_PRIMARY;
    get_sys_status_ptr_0456(status);

    I_LOOP_COUNTER = G_SYS_STATUS_PRIMARY;
    new_val = (I_LOOP_COUNTER + param) & 0x1F;

    get_sys_status_ptr_0400(status + 0x56);  /* 'V' = 0x56 */
    G_SYS_STATUS_PRIMARY = new_val;
}

/*
 * check_nvme_ready_e03c - Check if NVMe subsystem is ready
 * Address: 0xe03c-0xe06a (47 bytes)
 *
 * Checks multiple status conditions:
 *   - 0x0ACF must equal 0xA1 (XOR check)
 *   - 0x0AD3 must be 0
 *   - 0x0AD5 must equal 1
 *   - 0x0AD1 must be 0
 * If all pass: writes init sequence to 0x9003-0x9004, 0x9E00, returns 3
 * If any fail: returns 5
 */
uint8_t check_nvme_ready_e03c(void)
{
    /* Check 0x0ACF == 0xA1 */
    if (G_TLP_STATE_0ACF != 0xA1) {
        return 5;  /* Not ready */
    }

    /* Check 0x0AD3 == 0 */
    if (G_TLP_MODE_0AD3 != 0) {
        return 5;
    }

    /* Check 0x0AD5 == 1 */
    if (G_TLP_ADDR_OFFSET_HI != 1) {
        return 5;
    }

    /* Check 0x0AD1 == 0 */
    if (G_LINK_STATE_0AD1 != 0) {
        return 5;
    }

    /* All checks passed - initialize NVMe registers */
    REG_USB_EP0_LEN_H = 0x00;
    REG_USB_EP0_LEN_L = 0x01;
    REG_USB_SETUP_TYPE = 0x00;

    return 3;  /* Ready */
}

/*
 * nvme_clear_bits_and_init - Clear bit 4 in DPTR reg, clear bit 2 in 0xC472, write 0xFF to init regs
 * Address: 0xbb37-0xbb4e (24 bytes)
 *
 * Clears bit 4 in register pointed by DPTR, clears bit 2 in REG_NVME_LINK_CTRL,
 * then writes 0xFF to 4 consecutive registers at 0xC438-0xC43B.
 */
void nvme_clear_bits_and_init(__xdata uint8_t *reg)
{
    uint8_t val;

    /* Clear bit 4 in the input register */
    val = *reg;
    *reg = val & 0xEF;

    /* Clear bit 2 in REG_NVME_LINK_CTRL (0xC472) */
    val = REG_NVME_LINK_CTRL;
    REG_NVME_LINK_CTRL = val & 0xFB;

    /* Write 0xFF to 4 consecutive registers at 0xC438-0xC43B */
    REG_NVME_INIT_CTRL = 0xFF;
    REG_NVME_CMD_CDW11 = 0xFF;
    REG_NVME_INT_MASK_A = 0xFF;
    REG_NVME_INT_MASK_B = 0xFF;
}

/*
 * nvme_clear_bit3_link_ctrl - Clear bit 3 in DPTR reg, clear bit 1 in 0xC472, return 0xFF
 * Address: 0xbb7e-0xbb8e (17 bytes)
 *
 * Clears bit 3 in register pointed by DPTR, clears bit 1 in REG_NVME_LINK_CTRL,
 * returns 0xFF and sets DPTR to 0xC448.
 */
uint8_t nvme_clear_bit3_link_ctrl(__xdata uint8_t *reg)
{
    uint8_t val;

    /* Clear bit 3 in the input register */
    val = *reg;
    *reg = val & 0xF7;

    /* Clear bit 1 in REG_NVME_LINK_CTRL (0xC472) */
    val = REG_NVME_LINK_CTRL;
    REG_NVME_LINK_CTRL = val & 0xFD;

    return 0xFF;
}
