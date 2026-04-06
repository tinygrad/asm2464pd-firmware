/*
 * pcie.c - PCIe Interface Driver
 *
 * See drivers/pcie.h for hardware documentation.
 */

#include "drivers/pcie.h"
#include "types.h"
#include "sfr.h"
#include "registers.h"
#include "globals.h"
#include "utils.h"
#include "drivers/phy.h"
#include "drivers/timer.h"
#include "app/dispatch.h"
#include "drivers/cmd.h"
#include "drivers/uart.h"
#include "drivers/dma.h"

/* External helper functions not yet in headers - TODO: add to proper headers */
extern void timer_config_update(uint8_t param);
extern void protocol_param_handler(uint8_t param);
extern void protocol_init_setup(void);
extern void protocol_finalize(void);
extern void phy_link_ctrl_update(uint8_t param);
extern void pcie_transfer_handler(void);
extern void pcie_link_state_init(void);
extern void process_log_entries(uint8_t param);
extern void pcie_stage_address(uint8_t param);
extern uint8_t pcie_read_transaction_start(void);
extern uint8_t pcie_read_ext_reg(uint8_t reg_offset);
extern uint8_t pcie_setup_tlp_transaction(void);
extern void dispatch_helper(void);
extern void scsi_data_copy(uint8_t r3, uint8_t r2, uint8_t r1);
extern uint8_t nvme_clear_ep0_status(void);
extern void pcie_config_helper(void);
extern void pcie_status_helper(void);
extern void ext_mem_read_stub(uint8_t r3, uint8_t r2, uint8_t r1);
extern void power_state_handler_ca0d(void);  /* drivers/power.h */

/* Forward declarations */
uint8_t pcie_poll_and_read_completion(void);
void tlp_init_addr_buffer(void);
void flash_set_mode_enable(void);
uint8_t tlp_write_flash_cmd(uint8_t cmd);

/*
 * pcie_clear_and_trigger - Clear status flags and trigger transaction
 * Address: 0x999d-0x99ae (18 bytes)
 *
 * Sequence:
 *   1. Write 1 to status (clear error flag)
 *   2. Write 2 to status (clear complete flag)
 *   3. Write 4 to status (clear busy flag)
 *   4. Write 0x0F to trigger register to start transaction
 *
 * Original disassembly:
 *   999d: mov dptr, #0xb296     ; REG_PCIE_STATUS
 *   99a0: mov a, #0x01
 *   99a2: movx @dptr, a         ; write 1
 *   99a3: inc a                 ; a = 2
 *   99a4: movx @dptr, a         ; write 2
 *   99a5: mov a, #0x04
 *   99a7: movx @dptr, a         ; write 4
 *   99a8: mov dptr, #0xb254     ; REG_PCIE_TRIGGER
 *   99ab: mov a, #0x0f
 *   99ad: movx @dptr, a         ; write 0x0F
 *   99ae: ret
 */
void pcie_clear_and_trigger(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_ERROR;     /* Clear error flag */
    REG_PCIE_STATUS = PCIE_STATUS_COMPLETE;  /* Clear complete flag */
    REG_PCIE_STATUS = PCIE_STATUS_BUSY;      /* Clear busy flag */
    REG_PCIE_TRIGGER = 0x0F; /* Trigger all lanes */
}

/*
 * pcie_get_completion_status - Check if transaction completed
 * Address: 0x99eb-0x99f5 (11 bytes)
 *
 * Returns bit 2 of status register shifted to position 0.
 * Returns 1 if busy/complete, 0 otherwise.
 *
 * Original disassembly:
 *   99eb: mov dptr, #0xb296     ; REG_PCIE_STATUS
 *   99ee: movx a, @dptr         ; read status
 *   99ef: anl a, #0x04          ; mask bit 2
 *   99f1: rrc a                 ; rotate right
 *   99f2: rrc a                 ; rotate right (now bit 0)
 *   99f3: anl a, #0x3f          ; mask upper bits
 *   99f5: ret
 */
uint8_t pcie_get_completion_status(void)
{
    return (REG_PCIE_STATUS & PCIE_STATUS_BUSY) >> 2;
}

/*
 * pcie_get_link_speed - Get PCIe link speed from status
 * Address: 0x9a60-0x9a6b (12 bytes)
 *
 * Extracts bits 7:5 from link status register.
 * Returns link speed encoding (0-7).
 *
 * Original disassembly:
 *   9a60: mov dptr, #0xb22a     ; REG_PCIE_LINK_STATUS
 *   9a63: movx a, @dptr         ; read link status
 *   9a64: anl a, #0xe0          ; mask bits 7:5
 *   9a66: swap a                ; swap nibbles
 *   9a67: rrc a                 ; rotate right
 *   9a68: anl a, #0x07          ; mask to 3 bits
 *   9a6a: mov r7, a             ; return value
 *   9a6b: ret
 */
uint8_t pcie_get_link_speed(void)
{
    return (REG_PCIE_LINK_STATUS >> 5) & 0x07;
}

/*
 * pcie_set_byte_enables - Set TLP byte enables and length mode
 * Address: 0x9a30-0x9a3a (11 bytes)
 *
 * Sets byte enable mask for TLP and configures length to 0x20.
 *
 * Original disassembly:
 *   9a30: mov dptr, #0xb217     ; REG_PCIE_BYTE_EN
 *   9a33: movx @dptr, a         ; write byte enables from A
 *   9a34: mov dptr, #0xb216     ; REG_PCIE_TLP_LENGTH
 *   9a37: mov a, #0x20
 *   9a39: movx @dptr, a         ; write 0x20 (32 dwords)
 *   9a3a: ret
 */
void pcie_set_byte_enables(uint8_t byte_en)
{
    REG_PCIE_BYTE_EN = byte_en;
    REG_PCIE_TLP_LENGTH = 0x20;
}

/*
 * pcie_read_completion_data - Write status and read completion data
 * Address: 0x9a74-0x9a7e (11 bytes)
 *
 * Sets status to 0x02 (complete) then reads completion data register.
 *
 * Original disassembly:
 *   9a74: mov dptr, #0xb296     ; REG_PCIE_STATUS
 *   9a77: mov a, #0x02
 *   9a79: movx @dptr, a         ; write 2 to status
 *   9a7a: mov dptr, #0xb22c     ; REG_PCIE_CPL_DATA
 *   9a7d: movx a, @dptr         ; read completion data
 *   9a7e: ret
 */
uint8_t pcie_read_completion_data(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_COMPLETE;
    return REG_PCIE_CPL_DATA;
}

/*
 * pcie_write_status_complete - Write completion status flag
 * Address: 0x99f2-0x99f8 (7 bytes)
 *
 * Writes 0x04 to status register to indicate completion/busy clear.
 *
 * Original disassembly:
 *   99f2: mov dptr, #0xb296     ; REG_PCIE_STATUS
 *   99f5: mov a, #0x04
 *   99f7: movx @dptr, a         ; write 4
 *   99f8: ret
 */
void pcie_write_status_complete(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_BUSY;
}

/*
 * pcie_init - Initialize PCIe interface
 * Address: 0x9902-0x990b (10 bytes)
 *
 * Initializes PCIe controller by clearing bit configuration
 * and calling initialization routine.
 *
 * Original disassembly:
 *   9902: mov r0, #0x66
 *   9904: lcall 0x0db9       ; reg_clear_bit type function
 *   9907: lcall 0xde7e       ; initialization
 *   990a: mov a, r7
 *   990b: ret
 */
uint8_t pcie_init(void)
{
    /* Store R4-R7 (cleared) to IDATA[0x66..0x69] */
    /* Original uses lcall 0x0db9 (idata_store_dword) with R0=0x66 */
    idata_store_dword((__idata uint8_t *)0x66, 0);

    /* Call main PCIe initialization at 0xde7e */
    /* This is in Bank 1 - returns status in R7 */
    /* For now, just return success */
    return 0;
}

/*
 * pcie_init_alt - Alternative PCIe initialization
 * Address: 0x990c-0x9915 (10 bytes)
 *
 * Same pattern as pcie_init, possibly for different link mode.
 *
 * Original disassembly:
 *   990c: mov r0, #0x66
 *   990e: lcall 0x0db9
 *   9911: lcall 0xde7e
 *   9914: mov a, r7
 *   9915: ret
 */
uint8_t pcie_init_alt(void)
{
    /* Same as pcie_init - may be called in different contexts */
    return 0;
}

/*
 * pcie_set_idata_params - Set IDATA parameters for transaction
 * Address: 0x99f6-0x99ff (10 bytes)
 *
 * Sets IDATA location 0x65 to 0x0F and 0x63 to 0x00.
 * Used to configure byte enables and address offset.
 *
 * Original disassembly:
 *   99f6: mov r0, #0x65
 *   99f8: mov @r0, #0x0f        ; IDATA[0x65] = 0x0F
 *   99fa: mov r0, #0x63
 *   99fc: mov @r0, #0x00        ; IDATA[0x63] = 0x00
 *   99fe: inc r0                ; r0 = 0x64
 *   99ff: ret
 */
void pcie_set_idata_params(void)
{
    I_EP_MODE = 0x0F;
    I_EP_CONFIG_HI = 0x00;
    /* Note: original leaves R0 pointing to 0x64, but that's an artifact */
}

/*
 * pcie_clear_address_regs - Clear address offset registers
 * Address: 0x9a9c-0x9aa2 (7 bytes)
 *
 * Clears IDATA locations 0x63 and 0x64 (address offset).
 *
 * Original disassembly:
 *   9a9c: clr a
 *   9a9d: mov r0, #0x63
 *   9a9f: mov @r0, a            ; IDATA[0x63] = 0
 *   9aa0: inc r0
 *   9aa1: mov @r0, a            ; IDATA[0x64] = 0
 *   9aa2: ret
 */
void pcie_clear_address_regs(void)
{
    I_EP_CONFIG_HI = 0;
    I_EP_CONFIG_LO = 0;
}

/*
 * pcie_inc_txn_counters - Increment PCIe transaction counters
 * Address: 0x9a8a-0x9a94 (11 bytes)
 *
 * Increments both transaction count bytes at 0x05a6 and 0x05a7.
 * Used for tracking PCIe transactions for debugging/statistics.
 *
 * Original disassembly:
 *   9a8a: mov dptr, #0x05a6
 *   9a8d: movx a, @dptr         ; read low byte
 *   9a8e: inc a
 *   9a8f: movx @dptr, a         ; write low byte
 *   9a90: inc dptr              ; dptr = 0x05a7
 *   9a91: movx a, @dptr         ; read high byte
 *   9a92: inc a
 *   9a93: movx @dptr, a         ; write high byte
 *   9a94: ret
 */
void pcie_inc_txn_counters(void)
{
    G_PCIE_TXN_COUNT_LO++;
    G_PCIE_TXN_COUNT_HI++;
}

/*
 * pcie_get_txn_count_hi - Get high byte of transaction count
 * Address: 0x9aa9-0x9ab2 (10 bytes)
 *
 * Reads transaction count high byte and compares with IDATA[0x25].
 * Returns difference (used for transaction tracking).
 *
 * Original disassembly:
 *   9aa9: mov dptr, #0x05a7
 *   9aac: movx a, @dptr         ; read high count
 *   9aad: mov r7, a             ; save to r7
 *   9aae: mov a, 0x25           ; get IDATA[0x25]
 *   9ab0: clr c
 *   9ab1: subb a, r7            ; a = IDATA[0x25] - count_hi
 *   9ab2: ret
 */
uint8_t pcie_get_txn_count_hi(void)
{
    return G_PCIE_TXN_COUNT_HI;
}

/*
 * pcie_write_status_error - Clear error status flag
 * Address: inline pattern
 *
 * Writes 0x01 to status register to clear error flag.
 */
void pcie_write_status_error(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_ERROR;
}

/*
 * pcie_write_status_done - Clear completion status flag
 * Address: inline pattern
 *
 * Writes 0x02 to status register to clear completion flag.
 */
void pcie_write_status_done(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_COMPLETE;
}

/*
 * pcie_check_status_complete - Check if transaction complete bit set
 * Address: inline from pattern in pcie_set_address
 *
 * Returns non-zero if status bit 1 (complete) is set.
 */
uint8_t pcie_check_status_complete(void)
{
    return REG_PCIE_STATUS & PCIE_STATUS_COMPLETE;
}

/*
 * pcie_check_status_error - Check if error bit set
 * Address: inline from pattern in pcie_set_address
 *
 * Returns non-zero if status bit 0 (error) is set.
 */
uint8_t pcie_check_status_error(void)
{
    return REG_PCIE_STATUS & PCIE_STATUS_ERROR;
}

/*
 * pcie_setup_buffer_params - Setup buffer parameters for TLP
 * Address: 0x9a18-0x9a1f (8 bytes)
 *
 * Writes TLP buffer size parameters (0x34, 0x04) to consecutive
 * locations pointed to by DPTR. Called with DPTR pointing to
 * PCIe buffer descriptor area (0xB234, 0xB240, 0xB244).
 *
 * Original disassembly:
 *   9a18: mov a, #0x34
 *   9a1a: movx @dptr, a         ; [DPTR] = 0x34
 *   9a1b: inc dptr
 *   9a1c: mov a, #0x04
 *   9a1e: movx @dptr, a         ; [DPTR+1] = 0x04
 *   9a1f: ret
 */
void pcie_setup_buffer_params(uint16_t addr)
{
    XDATA8(addr) = 0x34;
    XDATA8(addr + 1) = 0x04;
}

/*
 * pcie_clear_reg_at_offset - Clear PCIe register at given offset
 * Address: 0x9a53-0x9a5f (13 bytes)
 *
 * Computes PCIe register address from offset (0xB2xx) and clears it.
 * offset + 0x10 forms the low byte, 0xB2 is the high byte.
 *
 * Original disassembly:
 *   9a53: add a, #0x10          ; a = offset + 0x10
 *   9a55: mov r7, a
 *   9a56: clr a
 *   9a57: addc a, #0xb2         ; a = 0xB2 (with carry)
 *   9a59: mov DPL, r7           ; DPL = offset + 0x10
 *   9a5b: mov DPH, a            ; DPH = 0xB2
 *   9a5d: clr a
 *   9a5e: movx @dptr, a         ; [0xB2xx] = 0
 *   9a5f: ret
 */
void pcie_clear_reg_at_offset(uint8_t offset)
{
    uint16_t addr = 0xB200 | (uint16_t)(offset + 0x10);
    XDATA8(addr) = 0;
}

/*
 * pcie_wait_for_completion - Wait for PCIe transaction to complete
 * Address: polling loop pattern from pcie_set_address
 *
 * Polls status register until completion or error bit is set.
 * Returns 0 on success, non-zero on error.
 */
uint8_t pcie_wait_for_completion(void)
{
    uint8_t status;

    /* First wait for busy bit to clear */
    do {
        status = pcie_get_completion_status();
    } while (status == 0);

    /* Write completion status */
    pcie_write_status_complete();

    /* Now wait for completion or error */
    while (1) {
        status = REG_PCIE_STATUS;
        if (status & PCIE_STATUS_COMPLETE) {
            /* Transaction complete */
            return 0;
        }
        if (status & PCIE_STATUS_ERROR) {
            /* Error occurred */
            REG_PCIE_STATUS = PCIE_STATUS_ERROR;  /* Clear error */
            return 0xFE;  /* Error code */
        }
    }
}

/*
 * pcie_setup_memory_tlp - Setup PCIe memory read/write TLP
 * Address: 0xc20c-0xc244 (57 bytes)
 *
 * Sets up a PCIe memory TLP based on direction from global 0x05AE bit 0:
 *   bit 0 = 1: Memory write (fmt_type = 0x40)
 *   bit 0 = 0: Memory read  (fmt_type = 0x00)
 *
 * Reads address from 0x05AF (4 bytes), calls helper functions to set up
 * TLP registers, triggers transaction, and polls for completion.
 * Returns 0 on success (if 0x05AE bit 0 clear), otherwise calls
 * pcie_poll_and_read_completion.
 *
 * Original disassembly at 0xc20c:
 *   c20c: mov dptr, #0xb210     ; FMT_TYPE register
 *   c20f: jnb acc.0, 0xc217     ; if read, jump
 *   c212: mov a, #0x40          ; write fmt_type
 *   c214: movx @dptr, a
 *   c215: sjmp 0xc219
 *   c217: clr a                 ; read fmt_type = 0
 *   c218: movx @dptr, a
 *   c219: mov dptr, #0xb213
 *   c21c: mov a, #0x01
 *   c21e: movx @dptr, a         ; enable bit
 *   c21f: mov a, #0x0f
 *   c221: lcall 0x9a30          ; pcie_set_byte_enables
 *   c224: mov dptr, #0x05af     ; address source
 *   c227: lcall 0x0d84          ; load 32-bit value
 *   c22a: mov dptr, #0xb218     ; address target
 *   c22d: lcall 0x0dc5          ; store 32-bit value
 *   c230: lcall 0x999d          ; pcie_clear_and_trigger
 *   c233: lcall 0x99eb          ; poll loop
 *   c236: jz 0xc233
 *   c238: lcall 0x9a95          ; pcie_write_status_complete
 *   c23b: mov dptr, #0x05ae
 *   c23e: movx a, @dptr
 *   c23f: jnb acc.0, 0xc245     ; if read, go to poll
 *   c242: mov r7, #0x00
 *   c244: ret
 */
uint8_t pcie_setup_memory_tlp(void)
{
    uint8_t direction;
    uint8_t status;

    /* Read direction from global */
    direction = G_PCIE_DIRECTION;

    /* Set format/type based on direction */
    if (direction & 0x01) {
        REG_PCIE_FMT_TYPE = PCIE_FMT_MEM_WRITE;
    } else {
        REG_PCIE_FMT_TYPE = PCIE_FMT_MEM_READ;
    }

    /* Enable TLP control */
    REG_PCIE_TLP_CTRL = 0x01;

    /* Set byte enables to 0x0F */
    pcie_set_byte_enables(0x0F);

    /* Copy 32-bit address from globals to PCIe address registers */
    REG_PCIE_ADDR_0 = G_PCIE_ADDR_0;
    REG_PCIE_ADDR_1 = G_PCIE_ADDR_1;
    REG_PCIE_ADDR_2 = G_PCIE_ADDR_2;
    REG_PCIE_ADDR_3 = G_PCIE_ADDR_3;

    /* Trigger transaction */
    pcie_clear_and_trigger();

    /* Poll for completion */
    do {
        status = pcie_get_completion_status();
    } while (status == 0);

    /* Write completion status */
    pcie_write_status_complete();

    /* Check direction again */
    direction = G_PCIE_DIRECTION;
    if (direction & 0x01) {
        /* Write - return success immediately */
        return 0;
    }

    /* Read - call poll and read completion */
    return pcie_poll_and_read_completion();
}

/*
 * pcie_poll_and_read_completion - Poll for completion and read result
 * Address: 0xc245-0xc26f (43 bytes)
 *
 * Polls PCIe status until complete or error. On completion, reads
 * completion data and verifies status. Returns:
 *   Link speed (0-7) on success (if completion status == 0x04)
 *   0xFE on error (bit 0 set)
 *   0xFF on completion error (non-zero completion data or wrong status)
 *
 * Original disassembly:
 *   c245: mov dptr, #0xb296     ; poll loop start
 *   c248: movx a, @dptr         ; read status
 *   c249: anl a, #0x02          ; check complete bit
 *   c24b: clr c
 *   c24c: rrc a                 ; shift to bit 0
 *   c24d: jnz 0xc259            ; if complete, read data
 *   c24f: movx a, @dptr
 *   c250: jnb acc.0, 0xc245     ; if no error, keep polling
 *   c253: mov a, #0x01          ; clear error
 *   c255: movx @dptr, a
 *   c256: mov r7, #0xfe         ; return 0xFE (error)
 *   c258: ret
 *   c259: lcall 0x9a74          ; pcie_read_completion_data
 *   c25c: jnz 0xc26d            ; if non-zero, error
 *   c25e: inc dptr              ; dptr = 0xB22D
 *   c25f: movx a, @dptr
 *   c260: jnz 0xc26d            ; if non-zero, error
 *   c262: mov dptr, #0xb22b
 *   c265: movx a, @dptr
 *   c266: cjne a, #0x04, 0xc26d ; check status == 4
 *   c269: lcall 0x9a60          ; pcie_get_link_speed
 *   c26c: ret
 *   c26d: mov r7, #0xff         ; return 0xFF (completion error)
 *   c26f: ret
 */
uint8_t pcie_poll_and_read_completion(void)
{
    uint8_t status;
    uint8_t cpl_data;

    /* Poll for complete or error */
    while (1) {
        status = REG_PCIE_STATUS;

        /* Check completion bit (bit 1) */
        if (status & TIMER_CSR_EXPIRED) {
            /* Complete - read completion data */
            cpl_data = pcie_read_completion_data();
            if (cpl_data != 0) {
                return 0xFF;  /* Completion error */
            }

            /* Check 0xB22D (next byte after CPL_DATA) */
            if (REG_PCIE_CPL_DATA_ALT != 0) {
                return 0xFF;  /* Completion error */
            }

            /* Check completion status code */
            if (REG_PCIE_CPL_STATUS != 0x04) {
                return 0xFF;  /* Completion error */
            }

            /* Success - return link speed */
            return pcie_get_link_speed();
        }

        /* Check error bit (bit 0) */
        if (status & 0x01) {
            REG_PCIE_STATUS = PCIE_STATUS_ERROR;  /* Clear error */
            return 0xFE;  /* Error code */
        }
    }
}

/*
 * pcie_write_tlp_addr_low - Write A to 0xB21B and set TLP length
 * Address: 0x9a33-0x9a3a (8 bytes)
 *
 * Called with address low byte in A, writes to 0xB21B then sets
 * TLP length register to 0x20.
 *
 * Original disassembly:
 *   9a33: movx @dptr, a         ; [B21B] = A
 *   9a34: mov dptr, #0xb216
 *   9a37: mov a, #0x20
 *   9a39: movx @dptr, a         ; [B216] = 0x20
 *   9a3a: ret
 */
void pcie_write_tlp_addr_low(uint8_t val)
{
    REG_PCIE_ADDR_3 = val;
    REG_PCIE_TLP_LENGTH = 0x20;
}

/*
 * pcie_setup_config_tlp - Setup PCIe configuration space TLP
 * Address: 0xadc3-0xae54 (approximately 145 bytes)
 *
 * Sets up a PCIe configuration space read or write TLP based on
 * parameters in IDATA:
 *   IDATA[0x60] bit 0: 0=read, 1=write
 *   IDATA[0x61]: Type (0=type0, non-0=type1)
 *   IDATA[0x62-0x64]: Address bytes
 *   IDATA[0x65]: Byte enables
 *
 * PCIe Config TLP format types:
 *   0x04: Type 0 Config Read
 *   0x05: Type 1 Config Read
 *   0x44: Type 0 Config Write
 *   0x45: Type 1 Config Write
 *
 * PCIe Config Address format (in 0xB218-0xB21B):
 *   The address bytes encode Bus/Device/Function/Register
 *   Bit manipulation is required to place these in the correct positions
 *
 * Original disassembly starts at 0xadc3:
 *   adc3: mov r0, #0x60
 *   adc5: mov a, @r0           ; check direction
 *   adc6: jnb acc.0, 0xadd3    ; if read, jump
 *   adc9: inc r0
 *   adca: mov a, @r0           ; check type
 *   adcb: mov r7, #0x44        ; type 0 write
 *   adcd: jz 0xadd1
 *   adcf: mov r7, #0x45        ; type 1 write
 *   add1: sjmp 0xaddc
 *   (read path)
 *   add3: mov r0, #0x61
 *   add5: mov a, @r0           ; check type
 *   add6: mov r7, #0x04        ; type 0 read
 *   add8: jz 0xaddc
 *   adda: mov r7, #0x05        ; type 1 read
 *   addc: mov dptr, #0xb210
 *   addf: mov a, r7
 *   ade0: movx @dptr, a        ; write fmt_type
 *   ... (continues with address setup and polling)
 */
void pcie_setup_config_tlp(void)
{
    uint8_t fmt_type;
    uint8_t direction;
    uint8_t type;
    uint8_t byte_en;
    uint8_t addr1, addr2, addr3;
    uint8_t shifted_hi, shifted_lo;
    uint8_t tmp;
    uint8_t status;
    uint8_t i;

    /* Read parameters from IDATA */
    direction = ((__idata uint8_t *)0x60)[0];
    type = ((__idata uint8_t *)0x61)[0];

    /* Determine format/type based on direction and type */
    if (direction & 0x01) {
        /* Config write */
        fmt_type = (type != 0) ? 0x45 : 0x44;  /* Type 1 or Type 0 */
    } else {
        /* Config read */
        fmt_type = (type != 0) ? 0x05 : 0x04;  /* Type 1 or Type 0 */
    }

    /* Setup PCIe TLP registers */
    REG_PCIE_FMT_TYPE = fmt_type;
    REG_PCIE_TLP_CTRL = 0x01;  /* Enable TLP control */

    /* Read byte enables and mask to lower 4 bits */
    byte_en = ((__idata uint8_t *)0x65)[0];
    REG_PCIE_BYTE_EN = byte_en & 0x0F;

    /* Read address bytes from IDATA[0x61-0x64] */
    addr1 = ((__idata uint8_t *)0x61)[0];
    REG_PCIE_ADDR_0 = addr1;

    addr1 = ((__idata uint8_t *)0x62)[0];
    REG_PCIE_ADDR_1 = addr1;

    /* Complex bit manipulation for config address format */
    /* addr2 = IDATA[0x63], addr3 = IDATA[0x64] */
    addr2 = ((__idata uint8_t *)0x63)[0];
    addr3 = ((__idata uint8_t *)0x64)[0];

    /* Extract bits: shifted_hi = addr2 & 0x03, tmp = addr3 & 0xC0 */
    shifted_hi = addr2 & 0x03;
    tmp = addr3 & 0xC0;

    /* Rotate 16-bit value (shifted_hi:tmp) right by 6 bits */
    /* This is: (shifted_hi << 8 | tmp) >> 6 = (shifted_hi << 2) | (tmp >> 6) */
    for (i = 0; i < 6; i++) {
        /* 16-bit right rotate through carry */
        uint8_t carry = shifted_hi & 0x01;
        shifted_hi >>= 1;
        tmp = (tmp >> 1) | (carry ? 0x80 : 0);
    }
    shifted_lo = tmp;

    /* Update address byte 2: preserve upper nibble, set lower nibble from shifted result */
    tmp = REG_PCIE_ADDR_2;
    tmp = (tmp & 0xF0) | (shifted_lo & 0x0F);
    REG_PCIE_ADDR_2 = tmp;

    /* Compute value for address byte 3 */
    /* Take lower 6 bits of addr3, multiply by 4 (shift left 2) */
    tmp = (addr3 & 0x3F) << 2;

    /* Read address byte 3, preserve low 2 bits, OR with shifted value */
    shifted_lo = REG_PCIE_ADDR_3;
    shifted_lo = (shifted_lo & 0x03) | tmp;

    /* Write to 0xB21B and set TLP length */
    pcie_write_tlp_addr_low(shifted_lo);

    /* Trigger the transaction */
    pcie_clear_and_trigger();

    /* Poll for completion */
    do {
        status = pcie_get_completion_status();
    } while (status == 0);

    /* Write completion status */
    pcie_write_status_complete();

    /* Check for errors - poll until complete or error */
    while (1) {
        status = REG_PCIE_STATUS;
        if (status & TIMER_CSR_EXPIRED) {
            /* Transaction complete - success */
            return;
        }
        if (status & 0x01) {
            /* Error occurred */
            REG_PCIE_STATUS = PCIE_STATUS_ERROR;  /* Clear error flag */
            /* Set error indicators in globals */
            G_ERROR_CODE_06EA = 0xFE;   /* Error code */
            G_STATE_FLAG_06E6 = 0x01;   /* Error flag */
            /* Error handler at 0xc00d clears state and recovery:
             * - Clears 0x06E6..0x06E8 (state flags)
             * - Clears 0x05A7, 0x06EB, 0x05AC, 0x05AD (transaction params)
             * - Calls 0x545c (USB reset helper)
             * - Calls 0x99e4 with DPTR=0xB401 (PCIe config write)
             * - Additional state machine reset logic
             */
            return;
        }
    }
}

/*
 * pcie_event_handler - PCIe event handler
 * Address: 0xC105-0xC17E (122 bytes)
 *
 * Processes PCIe-related events. Checks various status bits and dispatches
 * to appropriate sub-handlers.
 *
 * Original disassembly:
 *   c105: lcall 0xbcde        ; reg_read_bank_1407 - read from bank 1
 *   c108: jnb acc.0, 0xc10e   ; skip if bit 0 clear
 *   c10b: lcall 0xa522        ; pcie_interrupt_handler
 *   c10e: lcall 0xbcde        ; reg_read_bank_1407 again
 *   c111: jnb acc.3, 0xc117   ; skip if bit 3 clear
 *   c114: lcall 0x0543        ; handler_0543
 *   c117: lcall 0xbcaf        ; reg_read_bank_1603
 *   c11a: jnb acc.0, 0xc143   ; if bit 0 clear, goto alternate path
 *   c11d: mov a, #0x01
 *   c11f: lcall 0x0be6        ; banked_store_byte with A=0x01
 *   c122: mov dptr, #0x09fa   ; G_EVENT_CTRL_09FA
 *   c125: movx a, @dptr
 *   c126: jnb acc.1, 0xc17e   ; if bit 1 clear, return
 *   c129: mov dptr, #0x92c2   ; REG_POWER_STATUS
 *   c12c: movx a, @dptr
 *   c12d: jnb acc.6, 0xc139   ; if bit 6 clear, skip
 *   c130: mov dptr, #0x0ae2   ; G_SYSTEM_STATE_0AE2
 *   c133: mov a, #0x01
 *   c135: movx @dptr, a       ; write 0x01
 *   c136: lcall 0xca0d        ; handler_ca0d
 *   c139: lcall 0xe74e        ; handler_e74e
 *   c13c: mov dptr, #0x07ff   ; G_CMD_DEBUG_FF
 *   c13f: mov a, #0x69
 *   c141: movx @dptr, a       ; write 0x69
 *   c142: ret
 *   c143: lcall 0xbcaf        ; reg_read_bank_1603 (alternate path)
 *   c146: jnb acc.1, 0xc17e   ; if bit 1 clear, return
 *   c149: mov a, #0x02
 *   c14b: lcall 0x0be6        ; banked_store_byte with A=0x02
 *   c14e: mov dptr, #0x09fa
 *   c151: movx a, @dptr
 *   c152: jnb acc.1, 0xc17e   ; if bit 1 clear, return
 *   c155: lcall 0xbc88        ; reg_read_bank_1235
 *   c158: anl a, #0xc0        ; mask with 0xC0
 *   c15a: orl a, #0x04        ; OR with 0x04
 *   c15c: lcall 0xbc9f        ; helper_bc9f - store back
 *   c15f: anl a, #0x3f        ; mask with 0x3F
 *   c161: orl a, #0x40        ; OR with 0x40
 *   c163: lcall 0x0be6        ; banked_store_byte
 *   c166: mov a, #0x09
 *   c168: lcall 0xbc63        ; helper_bc63
 *   c16b: lcall 0xe890        ; handler_e890
 *   c16e: mov r1, #0x43
 *   c170: lcall 0xbc98        ; reg_read_bank_1200
 *   c173: jnb acc.6, 0xc17e   ; if bit 6 clear, return
 *   c176: clr a
 *   c177: mov r7, a           ; R7 = 0
 *   c178: lcall 0xd916        ; pcie_dispatch_d916
 *   c17b: lcall 0xbf8e        ; reg_clear_state_flags
 *   c17e: ret
 */
void pcie_event_handler(void)
{
    uint8_t status;
    uint8_t event_ctrl;
    uint8_t val;

    /* c105: lcall 0xbcde - read from bank 1407 */
    status = reg_read_bank_1407();

    /* c108: jnb acc.0, 0xc10e - if bit 0 set, call pcie interrupt handler */
    if (status & 0x01) {
        /* c10b: lcall 0xa522 */
        pcie_interrupt_handler();
    }

    /* c10e: lcall 0xbcde - read again */
    status = reg_read_bank_1407();

    /* c111: jnb acc.3, 0xc117 - if bit 3 set, call dispatch_0543 */
    if (status & 0x08) {
        /* c114: lcall 0x0543 */
        dispatch_0543();
    }

    /* c117: lcall 0xbcaf - read from bank 1603 */
    status = reg_read_bank_1603();

    /* c11a: jnb acc.0, 0xc143 - if bit 0 clear, goto alternate path */
    if (status & 0x01) {
        /* c11d-c11f: mov a, #0x01; lcall 0x0be6 */
        banked_store_byte(0, 0, 0x01, 0x01);

        /* c122-c125: read G_EVENT_CTRL_09FA */
        event_ctrl = G_EVENT_CTRL_09FA;

        /* c126: jnb acc.1, 0xc17e - if bit 1 clear, return */
        if (!(event_ctrl & TIMER_CSR_EXPIRED)) {
            return;
        }

        /* c129-c12c: read REG_POWER_STATUS (0x92C2) */
        val = REG_POWER_STATUS;

        /* c12d: jnb acc.6, 0xc139 - if bit 6 set */
        if (val & 0x40) {
            /* c130-c135: write 0x01 to G_SYSTEM_STATE_0AE2 */
            G_SYSTEM_STATE_0AE2 = 0x01;
            /* c136: lcall 0xca0d */
            power_state_handler_ca0d();
        }

        /* c139: lcall 0xe74e */
        pcie_disable_and_trigger_e74e();

        /* c13c-c141: write 0x69 to G_CMD_DEBUG_FF (0x07FF) */
        G_CMD_DEBUG_FF = 0x69;

        /* c142: ret */
        return;
    }

    /* Alternate path starting at c143 */
    /* c143: lcall 0xbcaf */
    status = reg_read_bank_1603();

    /* c146: jnb acc.1, 0xc17e - if bit 1 clear, return */
    if (!(status & TIMER_CSR_EXPIRED)) {
        return;
    }

    /* c149-c14b: mov a, #0x02; lcall 0x0be6 */
    banked_store_byte(0, 0, 0x01, 0x02);

    /* c14e-c151: read G_EVENT_CTRL_09FA */
    event_ctrl = G_EVENT_CTRL_09FA;

    /* c152: jnb acc.1, 0xc17e - if bit 1 clear, return */
    if (!(event_ctrl & TIMER_CSR_EXPIRED)) {
        return;
    }

    /* c155: lcall 0xbc88 - reg_read_bank_1235 */
    val = reg_read_bank_1235();

    /* c158-c15a: anl a, #0xc0; orl a, #0x04 */
    val = (val & 0xC0) | 0x04;

    /* c15c: lcall 0xbc9f - store back */
    banked_store_and_load_bc9f(val);

    /* c15f-c161: anl a, #0x3f; orl a, #0x40 */
    val = (val & 0x3F) | 0x40;

    /* c163: lcall 0x0be6 */
    banked_store_byte(0, 0, 0x01, val);

    /* c166-c168: mov a, #0x09; lcall 0xbc63 */
    banked_multi_store_bc63(0x09);

    /* c16b: lcall 0xe890 */
    pcie_handler_e890();

    /* c16e-c170: mov r1, #0x43; lcall 0xbc98 */
    val = reg_read_bank_1200();

    /* c173: jnb acc.6, 0xc17e - if bit 6 clear, return */
    if (!(val & 0x40)) {
        return;
    }

    /* c176-c178: clr a; mov r7, a; lcall 0xd916 */
    pcie_dispatch_d916(0);

    /* c17b: lcall 0xbf8e */
    reg_clear_state_flags();
}

/*
 * pcie_tunnel_enable - Enable PCIe tunneling for USB4/eGPU mode
 * Address: 0xC00D-0xC087
 *
 * Enables USB4 PCIe tunneling. This function:
 * 1. Checks if tunnel flag (0x06E6) is set - returns if not
 * 2. Clears state flags at 0x06E6-0x06E8
 * 3. Clears transaction parameters at 0x05A7, 0x06EB, 0x05AC-0x05AD
 * 4. Calls USB sync helper at 0x545C
 * 5. Clears tunnel control bit 0 (0xB401)
 * 6. Calls pcie_tunnel_setup (0xCD6C)
 * 7. Clears CPU mode bit 4 (0xCA06) - exit NVMe mode
 * 8. Configures all 4 PCIe lanes (0xD436 with 0x0F)
 * 9. Clears adapter config array at 0x05B3
 *
 * Original disassembly:
 *   c00d: mov dptr, #0x06e6     ; Tunnel flag
 *   c010: movx a, @dptr         ; Read flag
 *   c011: jz 0xc088             ; Return if not set
 *   c013: clr a                 ; Clear A
 *   c014: movx @dptr, a         ; Clear 0x06E6
 *   c015: inc dptr              ; 0x06E7
 *   c016: inc a                 ; A = 1
 *   c017: movx @dptr, a         ; Write 1 to 0x06E7
 *   c018: inc dptr              ; 0x06E8
 *   c019: movx @dptr, a         ; Write 1 to 0x06E8
 *   c01a: clr a
 *   c01b: mov dptr, #0x05a7     ; Clear PCIe TXN count hi
 *   c01e: movx @dptr, a
 *   c01f: mov dptr, #0x06eb
 *   c022: movx @dptr, a
 *   c023: mov dptr, #0x05ac
 *   c026: movx @dptr, a         ; Clear 0x05AC
 *   c027: inc dptr
 *   c028: movx @dptr, a         ; Clear 0x05AD
 *   c029: lcall 0x545c          ; USB sync helper
 *   c02c: mov dptr, #0xb401     ; Tunnel control
 *   c02f: lcall 0x99e4          ; Helper
 *   c032: movx a, @dptr
 *   c033: anl a, #0xfe          ; Clear bit 0
 *   c035: movx @dptr, a
 *   c036: lcall 0xcd6c          ; pcie_tunnel_setup
 *   c039: mov dptr, #0xca06     ; CPU mode
 *   c03c: movx a, @dptr
 *   c03d: anl a, #0xef          ; Clear bit 4 (NVMe mode)
 *   ... (continues with lane config and array clear)
 */
void pcie_tunnel_enable(void)
{
    uint8_t tmp;

    /* Check if tunnel enable flag is set */
    if (G_STATE_FLAG_06E6 == 0) {
        return;
    }

    /* Clear state flags */
    G_STATE_FLAG_06E6 = 0x00;
    G_WORK_06E7 = 0x01;
    G_WORK_06E8 = 0x01;

    /* Clear PCIe transaction parameters */
    G_PCIE_TXN_COUNT_HI = 0x00;
    G_WORK_06EB = 0x00;
    G_DMA_WORK_05AC = 0x00;
    G_DMA_WORK_05AD = 0x00;

    /* Call USB sync helper (0x545C) */

    /* Clear tunnel control bit 0 (0xB401) */
    tmp = REG_PCIE_TUNNEL_CTRL;
    tmp &= 0xFE;  /* Clear bit 0 */
    REG_PCIE_TUNNEL_CTRL = tmp;

    /* Call pcie_tunnel_setup (0xCD6C) */
    pcie_tunnel_setup();

    /* Clear CPU mode bit 4 (0xCA06) - exit NVMe mode */
    tmp = REG_CPU_MODE_NEXT;
    tmp &= 0xEF;  /* Clear bit 4 */
    REG_CPU_MODE_NEXT = tmp;

    /* Configure all 4 PCIe lanes (0xD436 with R7=0x0F) */
    pcie_lane_config(0x0F);

    /* Clear IDATA[0x62] */
    I_PCIE_TXN_DATA_1 = 0;

    /* Clear max log entries */
    G_MAX_LOG_ENTRIES = 0x00;
}

/*
 * pcie_adapter_config - Configure PCIe tunnel adapter registers
 * Address: 0xC8DB-0xC941
 *
 * Writes adapter configuration from globals (0x0A52-0x0A55) to
 * PCIe tunnel adapter hardware registers (0xB410-0xB42B).
 *
 * Config globals:
 *   0x0A52 (G_PCIE_ADAPTER_CFG_LO) - Link config high byte
 *   0x0A53 (G_PCIE_ADAPTER_CFG_HI) - Link config low byte
 *   0x0A54 (G_PCIE_ADAPTER_MODE)   - Mode config
 *   0x0A55 (G_PCIE_ADAPTER_AUX)    - Auxiliary config
 *
 * Original disassembly:
 *   c8db: mov dptr, #0x0a53    ; Load cfg_hi
 *   c8de: movx a, @dptr
 *   c8df: mov r7, a
 *   c8e0: mov dptr, #0xb410    ; Write to adapter reg 10
 *   c8e3: movx @dptr, a
 *   ...
 *   c941: ret
 */
void pcie_adapter_config(void)
{
    uint8_t cfg_hi, cfg_lo, cfg_mode, cfg_aux;

    /* Read config from globals */
    cfg_hi = G_PCIE_ADAPTER_CFG_HI;    /* 0x0A53 */
    cfg_lo = G_PCIE_ADAPTER_CFG_LO;    /* 0x0A52 */

    /* Write to tunnel config registers 0xB410-0xB411 */
    REG_TUNNEL_CFG_A_LO = cfg_hi;
    REG_TUNNEL_CFG_A_HI = cfg_lo;

    /* Write to tunnel data registers 0xB420-0xB421, load aux config */
    REG_TUNNEL_DATA_LO = cfg_hi;
    REG_TUNNEL_DATA_HI = cfg_lo;
    cfg_aux = G_PCIE_ADAPTER_AUX;      /* 0x0A55 */

    /* Write credits to 0xB412 */
    REG_TUNNEL_CREDITS = cfg_aux;

    /* Read mode config and write to 0xB413 */
    cfg_mode = G_PCIE_ADAPTER_MODE;    /* 0x0A54 */
    REG_TUNNEL_CFG_MODE = cfg_mode;

    /* Write status registers 0xB422-0xB423 */
    REG_TUNNEL_STATUS_0 = cfg_aux;
    REG_TUNNEL_STATUS_1 = cfg_mode;

    /* Write fixed capability pattern 0x06, 0x04, 0x00 to 0xB415-0xB417 */
    REG_TUNNEL_CAP_0 = 0x06;
    REG_TUNNEL_CAP_1 = 0x04;
    REG_TUNNEL_CAP_2 = 0x00;

    /* Write same pattern to 0xB425-0xB427 */
    REG_TUNNEL_CAP2_0 = 0x06;
    REG_TUNNEL_CAP2_1 = 0x04;
    REG_TUNNEL_CAP2_2 = 0x00;

    /* Reload config values */
    cfg_hi = G_PCIE_ADAPTER_CFG_HI;
    cfg_lo = G_PCIE_ADAPTER_CFG_LO;

    /* Write to tunnel link config registers 0xB41A-0xB41B */
    REG_TUNNEL_LINK_CFG_LO = cfg_hi;
    REG_TUNNEL_LINK_CFG_HI = cfg_lo;

    /* Write to auxiliary config 0xB42A-0xB42B, reload aux */
    REG_TUNNEL_AUX_CFG_LO = cfg_hi;
    REG_TUNNEL_AUX_CFG_HI = cfg_lo;
    cfg_aux = G_PCIE_ADAPTER_AUX;

    /* Write path credits to 0xB418 */
    REG_TUNNEL_PATH_CREDITS = cfg_aux;

    /* Reload mode and write to path mode 0xB419 */
    cfg_mode = G_PCIE_ADAPTER_MODE;
    REG_TUNNEL_PATH_MODE = cfg_mode;

    /* Write to path 2 registers 0xB428-0xB429 */
    REG_TUNNEL_PATH2_CRED = cfg_aux;
    REG_TUNNEL_PATH2_MODE = cfg_mode;
}

/*===========================================================================
 * PCIe Config/Helper Functions (0x9916-0x9aba)
 *
 * These are small helper functions for PCIe configuration table access
 * and transaction management. Most use the table at 0x05B4-0x05C0 which
 * stores per-endpoint PCIe configuration (34 bytes per entry).
 *
 * The helper at 0x0dd1 is a table multiply/add function:
 *   DPTR = base + (A * B)
 * where B is stored in the B register.
 *===========================================================================*/

/*
 * pcie_store_txn_idx - Store transaction index to 0x05A6
 * Address: 0x9916-0x9922 (13 bytes)
 *
 * Stores R6 to G_PCIE_TXN_COUNT_LO (0x05A6), copies to R7,
 * calls bank1 helper 0xE77A, stores 1 to IDATA[0x65].
 *
 * Original disassembly:
 *   9916: mov dptr, #0x05a6
 *   9919: mov a, r6
 *   991a: movx @dptr, a        ; store txn idx
 *   991b: mov r7, a
 *   991c: lcall 0xe77a         ; bank1 helper
 *   991f: mov r0, #0x65
 *   9921: mov @r0, #0x01       ; IDATA[0x65] = 1
 *   9923: ret (actually continues to 9923)
 */
void pcie_store_txn_idx(uint8_t idx)
{
    G_PCIE_TXN_COUNT_LO = idx;
    /* Call bank1 helper - simplified */
    *(__idata uint8_t *)0x65 = 0x01;
}

/*
 * pcie_config_table_lookup - Look up in config table at 0x05C0
 * Address: 0x9923-0x992f (13 bytes)
 *
 * Reads index from 0x05A6, multiplies by 0x22 (34),
 * adds to 0x05C0, returns via ljmp to 0x0dd1.
 *
 * Original disassembly:
 *   9923: mov dptr, #0x05a6
 *   9926: movx a, @dptr        ; A = txn index
 *   9927: mov dptr, #0x05c0    ; base table
 *   992a: mov b, #0x22         ; entry size 34 bytes
 *   992d: ljmp 0x0dd1          ; table lookup helper
 */
__xdata uint8_t *pcie_config_table_lookup(void)
{
    uint8_t idx = G_PCIE_TXN_COUNT_LO;
    uint16_t addr = 0x05C0 + ((uint16_t)idx * 0x22);
    return (__xdata uint8_t *)addr;
}

/*
 * pcie_lookup_config_05bd - Look up in config table at 0x05BD
 * Address: 0x9930-0x994a (27 bytes)
 *
 * Reads 0x05A6, multiplies by 0x22, adds to 0x05BD, reads result,
 * adds 4, stores to IDATA[0x64], carries to IDATA[0x63],
 * sets R7=0x40, R6=0x01, R5=0, R4=0, then calls 0x0dc5 with DPTR=0xB220.
 *
 * Original disassembly:
 *   9930: mov dptr, #0x05a6
 *   9933: movx a, @dptr
 *   9934: mov b, #0x22
 *   9937: mov dptr, #0x05bd    ; different table offset
 *   993a: lcall 0x0dd1         ; table lookup
 *   993d: movx a, @dptr
 *   993e: add a, #0x04         ; add 4 to result
 *   9940: mov r0, #0x64
 *   9942: mov @r0, a           ; IDATA[0x64] = result + 4
 *   9943: clr a
 *   9944: rlc a                ; carry to A
 *   9945: dec r0
 *   9946: mov @r0, a           ; IDATA[0x63] = carry
 *   9947: mov r7, #0x40
 *   9949: mov r6, #0x01
 *   994b: clr a
 *   994c: mov r5, a
 *   994d: mov r4, a
 *   994e: mov dptr, #0xb220
 *   9951: ljmp 0x0dc5          ; dword store helper
 */
void pcie_setup_buffer_from_config(void)
{
    uint8_t idx = G_PCIE_TXN_COUNT_LO;
    uint16_t addr = 0x05BD + ((uint16_t)idx * 0x22);
    uint8_t val = XDATA8(addr);
    uint8_t result = val + 4;
    uint8_t carry = (result < val) ? 1 : 0;

    *(__idata uint8_t *)0x64 = result;
    *(__idata uint8_t *)0x63 = carry;

    /* Store 0x00010040 to PCIe data register */
    (&REG_PCIE_DATA)[0] = 0x40;
    (&REG_PCIE_DATA)[1] = 0x01;
    (&REG_PCIE_DATA)[2] = 0x00;
    (&REG_PCIE_DATA)[3] = 0x00;
}

/*
 * pcie_write_data_reg_with_val - Entry point 0x994c
 * Address: 0x994c-0x9953 (8 bytes)
 *
 * This is an alternate entry point into pcie_setup_buffer_from_config.
 * When called at 0x994c, it takes A as input and sets R4=R5=A,
 * then stores R4/R5/R6/R7 to REG_PCIE_DATA.
 *
 * R6 and R7 are expected to be set by the caller.
 *
 * Original disassembly (entry at 0x994c):
 *   994c: mov r5, a          ; R5 = A (input)
 *   994d: mov r4, a          ; R4 = A (same as input)
 *   994e: mov dptr, #0xb220
 *   9951: ljmp 0x0dc5        ; store R4-R7 to DPTR
 */
void pcie_write_data_reg_with_val(uint8_t val, uint8_t r6, uint8_t r7)
{
    (&REG_PCIE_DATA)[0] = val;  /* R4 = val */
    (&REG_PCIE_DATA)[1] = val;  /* R5 = val */
    (&REG_PCIE_DATA)[2] = r6;
    (&REG_PCIE_DATA)[3] = r7;
}

/*
 * pcie_write_data_reg - Write R4-R7 to PCIe data register
 * Address: 0x994e-0x9953 (6 bytes)
 *
 * Sets DPTR to 0xB220 (REG_PCIE_DATA) and jumps to the dword
 * store helper at 0x0dc5 which writes R4/R5/R6/R7 to consecutive
 * addresses.
 *
 * Original disassembly:
 *   994e: mov dptr, #0xb220
 *   9951: ljmp 0x0dc5
 */
void pcie_write_data_reg(uint8_t r4, uint8_t r5, uint8_t r6, uint8_t r7)
{
    (&REG_PCIE_DATA)[0] = r4;
    (&REG_PCIE_DATA)[1] = r5;
    (&REG_PCIE_DATA)[2] = r6;
    (&REG_PCIE_DATA)[3] = r7;
}

/*
 * pcie_shift_r7_to_r6 - Shift R7 right twice and mask
 * Address: 0x9954-0x9961 (14 bytes)
 *
 * Takes R7, shifts right twice (divide by 4), masks to 6 bits,
 * stores to R6, then does table lookup at 0x05A6 with B=0x22.
 *
 * Original disassembly:
 *   9954: mov a, r7
 *   9955: rrc a               ; shift right through carry
 *   9956: rrc a               ; shift right again
 *   9957: anl a, #0x3f        ; mask to 6 bits
 *   9959: mov r6, a
 *   995a: mov dptr, #0x05a6
 *   995d: movx a, @dptr
 *   995e: mov b, #0x22
 *   9961: ret
 */
uint8_t pcie_calc_queue_idx(uint8_t val)
{
    uint8_t result = (val >> 2) & 0x3F;
    return result;
}

/*
 * pcie_get_txn_count_with_mult - Read transaction count with multiplier
 * Address: 0x995a-0x9961 (8 bytes)
 *
 * Reads G_PCIE_TXN_COUNT_LO, sets B register to 0x22 (34) for
 * subsequent multiply operation. Used to calculate offsets into
 * 34-byte entry tables.
 *
 * Returns: A = transaction count from 0x05A6
 *          B = 0x22 (set for MUL AB)
 *
 * Original disassembly:
 *   995a: mov dptr, #0x05a6
 *   995d: movx a, @dptr
 *   995e: mov b, #0x22
 *   9961: ret
 */
uint8_t pcie_get_txn_count_with_mult(void)
{
    B = 0x22;
    return G_PCIE_TXN_COUNT_LO;
}

/*
 * pcie_idata_table_lookup - Look up using IDATA[0x26] as index
 * Address: 0x9962-0x9969 (8 bytes)
 *
 * Reads IDATA[0x26], multiplies by 0x22, jumps to 0x0dd1.
 *
 * Original disassembly:
 *   9962: mov a, 0x26         ; A = IDATA[0x26]
 *   9964: mov b, #0x22
 *   9967: ljmp 0x0dd1
 */
__xdata uint8_t *pcie_idata_table_lookup(void)
{
    uint8_t idx = *(__idata uint8_t *)0x26;
    /* Base address implied by caller's DPTR setup */
    return (__xdata uint8_t *)(0x05B4 + ((uint16_t)idx * 0x22));
}

/*
 * pcie_check_txn_count - Compare transaction counts
 * Address: 0x996a-0x9976 (13 bytes)
 *
 * Reads 0x05A7 into R7, reads 0x0A5B into R6,
 * subtracts R7 from R6 to check difference.
 *
 * Original disassembly:
 *   996a: mov dptr, #0x05a7
 *   996d: movx a, @dptr       ; A = [0x05A7]
 *   996e: mov r7, a
 *   996f: mov dptr, #0x0a5b
 *   9972: movx a, @dptr       ; A = [0x0A5B]
 *   9973: mov r6, a
 *   9974: clr c
 *   9975: subb a, r7          ; A = R6 - R7
 *   9976: ret
 */
uint8_t pcie_check_txn_count(void)
{
    uint8_t count_hi = G_PCIE_TXN_COUNT_HI;
    uint8_t count_ref = G_NIBBLE_SWAP_0A5B;
    return count_ref - count_hi;
}

/*
 * pcie_param_table_lookup - Look up in table at 0x05B6
 * Address: 0x9977-0x997f (9 bytes)
 *
 * Sets DPTR=0x05B6, B=0x22, jumps to table lookup helper.
 *
 * Original disassembly:
 *   9977: mov dptr, #0x05b6
 *   997a: mov b, #0x22
 *   997d: ljmp 0x0dd1
 */
__xdata uint8_t *pcie_param_table_lookup(uint8_t idx)
{
    uint16_t addr = 0x05B6 + ((uint16_t)idx * 0x22);
    return (__xdata uint8_t *)addr;
}

/*
 * pcie_store_to_05b8 - Store R7 to table at 0x05B8
 * Address: 0x9980-0x9989 (10 bytes)
 *
 * Sets DPTR=0x05B8, stores A (from R7), multiplies by 0x22 for lookup.
 *
 * Original disassembly:
 *   9980: mov dptr, #0x05b8
 *   9983: mov a, r7
 *   9984: mov b, #0x22
 *   9987: ljmp 0x0dd1
 */
void pcie_store_to_05b8(uint8_t idx, uint8_t val)
{
    uint16_t addr = 0x05B8 + ((uint16_t)idx * 0x22);
    XDATA8(addr) = val;
}

/*
 * pcie_store_r6_to_05a6 - Store R6 to 0x05A6 and call helper
 * Address: 0x998a-0x9995 (12 bytes)
 *
 * Stores R6 to 0x05A6, copies to R7, calls 0xE77A, sets IDATA[0x65].
 *
 * Original disassembly:
 *   998a: mov a, r6
 *   998b: mov dptr, #0x05a6
 *   998e: movx @dptr, a
 *   998f: mov r7, a
 *   9990: lcall 0xe77a
 *   9993: mov r0, #0x65
 *   9995: ret
 */
void pcie_store_r6_to_05a6(uint8_t val)
{
    G_PCIE_TXN_COUNT_LO = val;
    /* bank1 helper 0xE77A would be called here */
}

/*
 * pcie_lookup_r3_multiply - Multiply R3 by 0x22 for table lookup
 * Address: 0x9996-0x999c (7 bytes)
 *
 * Takes R3, multiplies by 0x22, jumps to helper.
 *
 * Original disassembly:
 *   9996: mov a, r3
 *   9997: mov b, #0x22
 *   999a: ljmp 0x0dd1
 */
__xdata uint8_t *pcie_lookup_r3_multiply(uint8_t idx)
{
    /* DPTR base set by caller */
    return (__xdata uint8_t *)(0x05B4 + ((uint16_t)idx * 0x22));
}

/*
 * pcie_init_b296_regs - Initialize PCIe registers at 0xB296
 * Address: 0x999d-0x99ae (18 bytes)
 *
 * Writes 0x01, 0x02, 0x04 to 0xB296 (status clear sequence),
 * then writes 0x0F to 0xB254 (trigger).
 *
 * Original disassembly:
 *   999d: mov dptr, #0xb296
 *   99a0: mov a, #0x01
 *   99a2: movx @dptr, a        ; clear bit 0 flag
 *   99a3: inc a
 *   99a4: movx @dptr, a        ; write 0x02
 *   99a5: mov a, #0x04
 *   99a7: movx @dptr, a        ; write 0x04
 *   99a8: mov dptr, #0xb254
 *   99ab: mov a, #0x0f
 *   99ad: movx @dptr, a        ; trigger = 0x0F
 *   99ae: ret
 */
void pcie_init_b296_regs(void)
{
    REG_PCIE_STATUS = PCIE_STATUS_ERROR;  /* Clear flag 0 */
    REG_PCIE_STATUS = PCIE_STATUS_COMPLETE;  /* Clear flag 1 */
    REG_PCIE_STATUS = PCIE_STATUS_BUSY;  /* Clear flag 2 */
    REG_PCIE_TRIGGER = 0x0F; /* Trigger transaction */
}

/*
 * pcie_read_and_store_idata - Read DPTR and store to IDATA
 * Address: 0x99b0-0x99bb (12 bytes)
 *
 * Reads 2 bytes from DPTR+1, adds 2 to second byte,
 * stores to IDATA[0x64:0x63] with carry propagation.
 *
 * Original disassembly:
 *   99af: movx a, @dptr
 *   99b0: mov r6, a
 *   99b1: inc dptr
 *   99b2: movx a, @dptr
 *   99b3: add a, #0x02        ; add 2
 *   99b5: dec r0
 *   99b6: mov @r0, a          ; store to IDATA
 *   99b7: clr a
 *   99b8: addc a, r6          ; add carry to high byte
 *   99b9: dec r0
 *   99ba: mov @r0, a
 *   99bb: ret
 */
void pcie_read_and_store_idata(__xdata uint8_t *ptr)
{
    uint8_t hi = ptr[0];
    uint8_t lo = ptr[1];
    uint8_t new_lo = lo + 2;
    uint8_t new_hi = hi + ((new_lo < lo) ? 1 : 0);

    *(__idata uint8_t *)0x64 = new_lo;
    *(__idata uint8_t *)0x63 = new_hi;
}

/*
 * pcie_store_r7_to_05b7 - Store R7 to table at 0x05B7
 * Address: 0x99bd-0x99c5 (9 bytes)
 *
 * Sets DPTR=0x05B7, stores R7, multiplies by 0x22.
 *
 * Original disassembly:
 *   99bc: mov dptr, #0x05b7
 *   99bf: mov a, r7
 *   99c0: mov b, #0x22
 *   99c3: ljmp 0x0dd1
 */
void pcie_store_r7_to_05b7(uint8_t idx, uint8_t val)
{
    uint16_t addr = 0x05B7 + ((uint16_t)idx * 0x22);
    XDATA8(addr) = val;
}

/*
 * pcie_set_0a5b_flag - Set flag at 0x0A5B
 * Address: 0x99c6-0x99cd (8 bytes)
 *
 * Stores A to DPTR, then writes 1 to 0x0A5B.
 *
 * Original disassembly:
 *   99c6: movx @dptr, a
 *   99c7: mov dptr, #0x0a5b
 *   99ca: mov a, #0x01
 *   99cc: movx @dptr, a
 *   99cd: ret
 */
void pcie_set_0a5b_flag(__xdata uint8_t *ptr, uint8_t val)
{
    *ptr = val;
    G_NIBBLE_SWAP_0A5B = 0x01;
}

/*
 * pcie_inc_0a5b - Increment value at 0x0A5B
 * Address: 0x99ce-0x99d4 (7 bytes)
 *
 * Reads 0x0A5B, increments, writes back.
 *
 * Original disassembly:
 *   99ce: mov dptr, #0x0a5b
 *   99d1: movx a, @dptr
 *   99d2: inc a
 *   99d3: movx @dptr, a
 *   99d4: ret
 */
void pcie_inc_0a5b(void)
{
    uint8_t val = G_NIBBLE_SWAP_0A5B;
    G_NIBBLE_SWAP_0A5B = val + 1;
}

/*
 * pcie_lookup_and_store_idata - Table lookup and store result
 * Address: 0x99d5-0x99df (11 bytes)
 *
 * Calls table lookup helper, reads result, stores to IDATA[0x63:0x64].
 *
 * Original disassembly:
 *   99d5: lcall 0x0dd1        ; table lookup
 *   99d8: movx a, @dptr       ; read result
 *   99d9: mov r0, #0x63
 *   99db: mov @r0, #0x00      ; IDATA[0x63] = 0
 *   99dd: inc r0
 *   99de: mov @r0, a          ; IDATA[0x64] = result
 *   99df: ret
 */
void pcie_lookup_and_store_idata(uint8_t idx, uint16_t base)
{
    uint16_t addr = base + ((uint16_t)idx * 0x22);
    uint8_t val = XDATA8(addr);

    *(__idata uint8_t *)0x63 = 0x00;
    *(__idata uint8_t *)0x64 = val;
}

/*
 * pcie_write_config_and_trigger - Write to DPTR and trigger PCIe
 * Address: 0x99e0-0x99ea (11 bytes)
 *
 * Stores A to DPTR, reads 0xB480, sets bit 0, writes back.
 * This triggers a PCIe configuration write.
 *
 * Original disassembly:
 *   99e0: movx @dptr, a       ; write value
 *   99e1: mov dptr, #0xb480
 *   99e4: movx a, @dptr       ; read control reg
 *   99e5: anl a, #0xfe        ; clear bit 0
 *   99e7: orl a, #0x01        ; set bit 0
 *   99e9: movx @dptr, a       ; write back
 *   99ea: ret
 */
void pcie_write_config_and_trigger(__xdata uint8_t *ptr, uint8_t val)
{
    uint8_t ctrl;

    *ptr = val;

    ctrl = REG_TUNNEL_LINK_CTRL;
    ctrl = (ctrl & 0xFE) | 0x01;
    REG_TUNNEL_LINK_CTRL = ctrl;
}

/*
 * pcie_get_status_bit2 - Extract bit 2 from PCIe status
 * Address: 0x99eb-0x99f5 (11 bytes)
 *
 * Reads 0xB296, masks bit 2, shifts right twice, masks to 6 bits.
 *
 * Original disassembly:
 *   99eb: mov dptr, #0xb296
 *   99ee: movx a, @dptr
 *   99ef: anl a, #0x04        ; isolate bit 2
 *   99f1: rrc a               ; shift right
 *   99f2: rrc a               ; shift again
 *   99f3: anl a, #0x3f        ; mask
 *   99f5: ret
 */
uint8_t pcie_get_status_bit2(void)
{
    uint8_t val = REG_PCIE_STATUS;
    val = (val & 0x04) >> 2;
    return val & 0x3F;
}

/*
 * pcie_init_idata_65_63 - Initialize IDATA transfer params
 * Address: 0x99f6-0x99ff (10 bytes)
 *
 * Sets IDATA[0x65]=0x0F, IDATA[0x63]=0x00, increments R0.
 *
 * Original disassembly:
 *   99f6: mov r0, #0x65
 *   99f8: mov @r0, #0x0f      ; IDATA[0x65] = 0x0F
 *   99fa: mov r0, #0x63
 *   99fc: mov @r0, #0x00      ; IDATA[0x63] = 0
 *   99fe: inc r0
 *   99ff: ret
 */
void pcie_init_idata_65_63(void)
{
    *(__idata uint8_t *)0x65 = 0x0F;
    *(__idata uint8_t *)0x63 = 0x00;
}

/*
 * pcie_add_2_to_idata - Add 2 to value and store to IDATA
 * Address: 0x9a00-0x9a08 (9 bytes)
 *
 * Adds 2 to A, stores to IDATA[0x64], propagates carry to IDATA[0x63].
 *
 * Original disassembly:
 *   9a00: add a, #0x02
 *   9a02: dec r0
 *   9a03: mov @r0, a          ; store result
 *   9a04: clr a
 *   9a05: rlc a               ; get carry
 *   9a06: dec r0
 *   9a07: mov @r0, a          ; store carry
 *   9a08: ret
 */
void pcie_add_2_to_idata(uint8_t val)
{
    uint8_t result = val + 2;
    uint8_t carry = (result < val) ? 1 : 0;

    *(__idata uint8_t *)0x64 = result;
    *(__idata uint8_t *)0x63 = carry;
}

/*
 * pcie_lookup_r6_multiply - Multiply R6 by 0x22 for table lookup
 * Address: 0x9a09-0x9a0f (7 bytes)
 *
 * Takes R6, multiplies by 0x22, jumps to helper.
 *
 * Original disassembly:
 *   9a09: mov a, r6
 *   9a0a: mov b, #0x22
 *   9a0d: ljmp 0x0dd1
 */
__xdata uint8_t *pcie_lookup_r6_multiply(uint8_t idx)
{
    /* DPTR base set by caller */
    return (__xdata uint8_t *)(0x05BD + ((uint16_t)idx * 0x22));
}

/*
 * pcie_offset_table_lookup - Look up in table at 0x05BD
 * Address: 0x9a10-0x9a1f (16 bytes)
 *
 * Sets DPTR=0x05BD, reads index from 0x05A6, multiplies by 0x22.
 *
 * Original disassembly:
 *   9a10: mov dptr, #0x05bd
 *   ... (continues with table lookup)
 */
__xdata uint8_t *pcie_offset_table_lookup(void)
{
    uint8_t idx = G_PCIE_TXN_COUNT_LO;
    uint16_t addr = 0x05BD + ((uint16_t)idx * 0x22);
    return (__xdata uint8_t *)addr;
}

/*
 * pcie_set_byte_enables_0f - Set byte enables to 0x0F
 * Address: 0x9a3b-0x9a45 (11 bytes)
 *
 * Writes 0x0F to REG_PCIE_BYTE_EN (0xB254).
 * This enables all 4 byte lanes for PCIe transactions.
 *
 * Original disassembly:
 *   9a3b: mov dptr, #0xb254
 *   9a3e: mov a, #0x0f
 *   9a40: movx @dptr, a
 *   ... (may continue)
 */
void pcie_set_byte_enables_0f(void)
{
    REG_PCIE_TRIGGER = 0x0F;
}

/*
 * pcie_setup_buffer_params_ext - Extended buffer parameter setup
 * Address: 0x9a46-0x9a6b (38 bytes)
 *
 * Sets up PCIe buffer parameters for DMA transfers.
 * Reads configuration from table, calculates addresses.
 */
void pcie_setup_buffer_params_ext(uint8_t idx)
{
    uint16_t table_addr = 0x05B4 + ((uint16_t)idx * 0x22);
    uint8_t val;

    val = XDATA8(table_addr);
    /* Setup buffer pointers based on config */
    (void)val;
}

/*
 * pcie_get_link_speed_masked - Get link speed with mask
 * Address: 0x9a6c-0x9aa2 (55 bytes)
 *
 * Reads link speed from PCIe status registers.
 * Returns speed code in bits 7:5.
 */
uint8_t pcie_get_link_speed_masked(void)
{
    uint8_t val = REG_PCIE_LINK_STATUS;
    return val & 0xE0;  /* Bits 7:5 are link speed */
}

/*
 * pcie_clear_address_regs_full - Clear all PCIe address registers
 * Address: 0x9aa3-0x9ab2 (16 bytes)
 *
 * Clears PCIe address registers 0xB218-0xB21B.
 *
 * Original disassembly:
 *   9aa3: mov dptr, #0xb218
 *   9aa6: clr a
 *   9aa7: movx @dptr, a
 *   9aa8: inc dptr
 *   9aa9: movx @dptr, a
 *   9aaa: inc dptr
 *   9aab: movx @dptr, a
 *   9aac: inc dptr
 *   9aad: movx @dptr, a
 *   9aae: ret
 */
void pcie_clear_address_regs_full(void)
{
    REG_PCIE_ADDR_0 = 0;
    REG_PCIE_ADDR_1 = 0;
    REG_PCIE_ADDR_2 = 0;
    REG_PCIE_ADDR_3 = 0;
}

/*
 * pcie_inc_txn_count - Increment transaction count at 0x05A6/0x05A7
 * Address: 0x9ab3-0x9ab9 (7 bytes)
 *
 * Increments the 16-bit transaction counter.
 *
 * Original disassembly:
 *   9ab3: mov dptr, #0x05a6
 *   9ab6: movx a, @dptr
 *   9ab7: inc a
 *   9ab8: movx @dptr, a
 *   9ab9: ret
 */
void pcie_inc_txn_count(void)
{
    uint8_t val = G_PCIE_TXN_COUNT_LO;
    G_PCIE_TXN_COUNT_LO = val + 1;
}

/*
 * pcie_tlp_handler_b104 - Main PCIe TLP handler
 * Address: 0xb104-0xb1ca (~199 bytes)
 *
 * Handles PCIe TLP (Transaction Layer Packet) processing.
 * This is the main entry point for TLP-related operations.
 *
 * Uses globals:
 *   0x0aa4: TLP config base (state counter)
 *   0x0aa8-0x0aa9: TLP count high/low
 *   0x0aaa: TLP status / pending count
 *   IDATA 0x51-0x52: Local transfer counters
 *
 * Algorithm:
 *   1. Initialize TLP state (clear counters at 0x0AA4, IDATA 0x51/0x52)
 *   2. Call tlp_init_addr_buffer() to clear address/length buffers
 *   3. Call flash_set_mode_enable() to enable flash operations
 *   4. Main loop: process pending TLPs until complete
 *      - Compare pending count vs processed count
 *      - For each TLP: set up DMA transfer, wait for completion
 *      - Update counters
 *   5. Return 1 on success, 0 on error/timeout
 */
uint8_t pcie_tlp_handler_b104(void)
{
    uint8_t pending_count, processed_lo, processed_hi;
    uint8_t flash_cmd_result;

    /* Initialize: Store 0 to TLP config at 0x0AA4 (dword store via 0x0dc5) */
    G_STATE_COUNTER_LO = 0;
    G_STATE_COUNTER_HI = 0;

    /* Clear IDATA counters */
    I_LOOP_COUNTER = 0;
    I_POLL_STATUS = 0;

    /* Main processing loop */
    do {
        /* Initialize address buffer */
        tlp_init_addr_buffer();

        /* Enable flash mode */
        flash_set_mode_enable();

        /* Store local counters to TLP config area */
        G_STATE_COUNTER_LO = I_LOOP_COUNTER;
        G_STATE_COUNTER_0AA5 = I_POLL_STATUS;

        /* Write flash command 0x02, get addr length result */
        flash_cmd_result = tlp_write_flash_cmd(0x02);

        /* Set command byte with mode bits */
        REG_FLASH_CMD = flash_cmd_result | 0x03;

        /* Load flash address from 0x0AA4 area (via 0x0d84 helper) */
        /* Store R7 to flash address low register */
        REG_FLASH_ADDR_LO = flash_cmd_result;

        /* Read pending count */
        pending_count = G_TLP_STATUS;

        /* Compare counts: [0xAA9] >= pending ? */
        processed_lo = G_TLP_COUNT_LO;
        processed_hi = G_TLP_COUNT_HI;

        if (processed_hi > 0 || processed_lo >= pending_count) {
            /* More to process - continue loop */
            uint8_t current_pending = G_TLP_STATUS;

            /* Store current pending to flash data length registers */
            REG_FLASH_DATA_LEN = processed_hi;
            REG_FLASH_DATA_LEN_HI = current_pending;

            /* Update processed counts */
            G_TLP_COUNT_LO = G_TLP_COUNT_LO - current_pending;
            G_TLP_COUNT_HI = G_TLP_COUNT_HI - ((G_TLP_COUNT_LO < current_pending) ? 1 : 0);

            /* Update local counters */
            I_POLL_STATUS = I_POLL_STATUS + current_pending;
            I_LOOP_COUNTER = I_LOOP_COUNTER + ((I_POLL_STATUS < current_pending) ? 1 : 0);
        } else {
            /* Clear processed counts */
            G_TLP_COUNT_HI = 0;
            G_TLP_COUNT_LO = 0;
        }

        /* Trigger flash operation */
        REG_FLASH_CSR = 0x01;

        /* Wait for completion (poll bit 0 of CSR) */
        while ((REG_FLASH_CSR & 0x01) == 0x01) {
            /* Spin wait */
        }

        /* Check for timeout/error via 0xdf47 helper */
        /* For now, simplified check */
        if (G_TLP_COUNT_HI == 0 && G_TLP_COUNT_LO == 0) {
            /* All done - success */
            return 1;
        }

    } while (G_TLP_COUNT_HI != 0 || G_TLP_COUNT_LO != 0);

    return 1;  /* Success */
}

/* External declarations for helper functions from nvme.c */
extern uint8_t nvme_queue_get_9100(void);
extern uint8_t nvme_queue_mask_0acf(void);
extern void nvme_queue_clear_9003(void);
extern void nvme_queue_set_9092(uint8_t param);
extern void nvme_queue_set_bit0_ptr(__xdata uint8_t *ptr);
extern uint8_t nvme_queue_shift_param(uint8_t param);
extern void nvme_completion_poll(void);
extern void usb_buffer_handler(void);

/*
 * pcie_tlp_handler_b28c - Secondary PCIe TLP handler
 * Address: 0xb28c-0xb401 (~374 bytes)
 *
 * State machine for TLP processing based on G_TLP_CMD_STATE_0AD0.
 * Handles queue status and transfer operations.
 *
 * Original disassembly:
 *   b28c: mov dptr, #0x07e4
 *   b28f: movx a, @dptr          ; read state
 *   b291: cjne a, #0x05, b298    ; if state == 5, quick return
 *   b294: lcall 0xa714           ; write 0x9092 = 1
 *   b297: ret
 *   b298: mov dptr, #0x0ad0
 *   b29b: movx a, @dptr          ; get cmd state
 *   b29c: lcall 0x0def           ; jump table dispatch
 *   [switch based on state...]
 */
void pcie_tlp_handler_b28c(void)
{
    uint8_t state;
    uint8_t r3;

    /* Check for state 5 - quick completion */
    state = G_SYS_FLAGS_BASE;
    if (state == 0x05) {
        /* Write 0x9092 = 1 and return */
        REG_USB_DMA_TRIGGER = 1;
        return;
    }

    /* Get command state and dispatch */
    state = G_TLP_CMD_STATE_0AD0;

    /* Switch based on command state (jump table at 0x0def) */
    switch (state) {
        case 0x00:
            /* State 0: b2de entry point */
            /* Read 0x07e4 again */
            state = G_SYS_FLAGS_BASE;
            if (state == 0x03) {
                /* State is 3: write 4 to 0x9092 and 4 to 0x07e4 */
                REG_USB_DMA_TRIGGER = 4;
                G_SYS_FLAGS_BASE = 4;
            } else {
                /* Default: write 1 to 0x9092 */
                REG_USB_DMA_TRIGGER = 1;
            }
            /* Fall through to call d810 */
            usb_buffer_handler();
            break;

        case 0x03:
            /* State 3: b2f3 entry point */
            r3 = G_SYS_FLAGS_BASE;
            if (r3 == 0x03) {
                /* Call 0xba06 */
                nvme_completion_poll();
                /* Check 0x0ae5 init flag */
                if (G_TLP_INIT_FLAG_0AE5 != 0) {
                    usb_buffer_handler();
                    break;
                }
                /* Check 0x07e9 queue status */
                if (G_TLP_STATE_07E9 == 0) {
                    usb_buffer_handler();
                    break;
                }
                /* Clear 0x07e9 */
                G_TLP_STATE_07E9 = 0;
                /* Check USB status bit 0 */
                if (REG_USB_STATUS & USB_STATUS_DMA_READY) {
                    /* Check 0xc471 bit 0 */
                    if (REG_NVME_QUEUE_BUSY & NVME_QUEUE_BUSY_BIT) {
                        usb_buffer_handler();
                        break;
                    }
                    /* Check 0x000a */
                    if (G_EP_CHECK_FLAG != 0) {
                        usb_buffer_handler();
                        break;
                    }
                } else {
                    /* Check 0x9101 bit 6 */
                    if (REG_USB_PERIPH_STATUS & USB_PERIPH_CBW_RECEIVED) {
                        usb_buffer_handler();
                        break;
                    }
                    /* Check idata 0x6a */
                    if (I_USB_STATE != 0) {
                        usb_buffer_handler();
                        break;
                    }
                }
                /* Call 0xa72b - set bit 0 on 0x92c4 */
                nvme_queue_set_bit0_ptr((__xdata uint8_t *)0x92C4);
            } else if (r3 == 0) {
                /* State is 0 - short exit */
                return;
            }
            /* Default: write 1 to 0x9092 */
            REG_USB_DMA_TRIGGER = 1;
            usb_buffer_handler();
            break;

        case 0x04:
            /* State 4: b2cb entry - b2ce: lcall 0xa71b then write */
            nvme_queue_clear_9003();
            nvme_queue_set_9092(4);
            /* Write 1 to (0x9003+1) = 0x9004 */
            REG_USB_EP0_LEN_L = 1;
            usb_buffer_handler();
            break;

        case 0x05:
            /* State 5: Already handled above, but for completeness */
            REG_USB_DMA_TRIGGER = 1;
            break;

        default:
            /* Other states: write 1 to 0x9092 and return via d810 */
            REG_USB_DMA_TRIGGER = 1;
            usb_buffer_handler();
            break;
    }
}

/* External helper declarations for b402 */
extern uint8_t nvme_queue_get_9100(void);       /* 0xa666 */
extern uint8_t nvme_queue_clear_usb_bit0(void); /* 0xa679 */
extern void nvme_queue_init_905x(void);         /* 0xa6ad */
extern void nvme_queue_config_9006(uint8_t param, __xdata uint8_t *ptr); /* 0xa6c6 */
extern void nvme_queue_set_90e3_2(void);        /* 0xa739 */
extern uint8_t table_lookup(void);               /* 0xa704 - table lookup */

/*
 * pcie_tlp_handler_b402 - Tertiary PCIe TLP handler
 * Address: 0xb402-0xb623 (~546 bytes)
 *
 * Complex TLP handler with multiple return points.
 * Handles transfer timeout calculation, DMA buffer clearing,
 * and coordination with USB/NVMe subsystems.
 *
 * Return values in R7:
 *   3 - Setup complete, base set to 0x9E
 *   4 - Full init/transfer complete
 *   5 - Early exit conditions
 */
uint8_t pcie_tlp_handler_b402(void)
{
    uint8_t state;
    uint8_t r6, r7;
    uint16_t computed;
    uint16_t limit;
    uint8_t iter_count;
    uint16_t i;

    /* b402: Call a666 to get USB link state */
    state = nvme_queue_get_9100();

    /* b406: Check if state == 2 */
    if (state == 0x02) {
        /* State 2: Use timeout 0x0200 (512) */
        r6 = 0x02;
        r7 = 0x00;
    } else {
        /* Other states: Use timeout 0x0040 (64) */
        r6 = 0x00;
        r7 = 0x40;
    }

    /* b413: Store timeout to 0x0AD8:0x0AD9 */
    G_TLP_TIMEOUT_HI = r6;
    G_TLP_TIMEOUT_LO = r7;

    /* b41b-b41f: Clear 0x0ADA:0x0ADB (transfer addresses) */
    G_TLP_TRANSFER_HI = 0;
    G_TLP_TRANSFER_LO = 0;

    /* b420-b437: Compute transfer limit from offset */
    r6 = G_TLP_ADDR_OFFSET_LO;
    r7 = G_TLP_ADDR_OFFSET_HI;

    /* Store to 0x0ADC:0x0ADD (computed value) */
    G_TLP_COMPUTED_HI = r7;
    G_TLP_COMPUTED_LO = r6;

    /* b438-b43d: If computed == 0, return 4 */
    if (r6 == 0 && r7 == 0) {
        return 4;
    }

    /* b43e-b45b: Compare computed with limit from 0x0ADE:0x0ADF */
    r6 = G_TLP_LIMIT_HI;
    r7 = G_TLP_LIMIT_LO;
    computed = ((uint16_t)G_TLP_COMPUTED_HI << 8) | G_TLP_COMPUTED_LO;
    limit = ((uint16_t)r6 << 8) | r7;

    /* b452: If computed <= limit, use computed; else use limit */
    if (computed >= limit) {
        G_TLP_COMPUTED_HI = r6;
        G_TLP_COMPUTED_LO = r7;
    } else {
        G_TLP_LIMIT_HI = G_TLP_COMPUTED_HI;
        G_TLP_LIMIT_LO = G_TLP_COMPUTED_LO;
    }

    /* b466-b4a8: Process iteration counter 0x0AD7 */
    iter_count = G_TLP_COUNT_0AD7;
    if (iter_count >= 3) {
        /* b4a9-b4b9: Counter overflow - set base to 0x9E, return 3 */
        G_TLP_COUNT_0AD7 = 0;
        G_TLP_BASE_HI = 0x9E;
        G_TLP_BASE_LO = 0;
        return 3;
    }

    /* b46f-b4a7: Loop through computed bytes, process DMA buffer */
    r6 = 0;
    r7 = 0;
    while (1) {
        uint8_t val;
        uint8_t idx;

        limit = ((uint16_t)G_TLP_COMPUTED_HI << 8) | G_TLP_COMPUTED_LO;
        computed = ((uint16_t)r6 << 8) | r7;
        if (computed >= limit) {
            break;
        }

        idx = G_TLP_COUNT_0AD7;
        if (idx == 0) {
            val = table_lookup();
        } else if (idx == 1) {
            table_lookup();
            val = 0;
        } else {
            val = 0;
        }

        /* Write to DMA buffer at 0xD800 + offset */
        *(__xdata uint8_t *)(0xD800 + r7) = val;

        r7++;
        if (r7 == 0) r6++;
        if (r7 == 0x60 && r6 == 0x06) break;
    }

    /* b4ba-b570: Process CC status registers - simplified */
    /* Check CC23 (Timer3 CSR) bit 1 */
    if (REG_TIMER3_CSR & TIMER_CSR_EXPIRED) {
        REG_TIMER3_CSR = TIMER_CSR_EXPIRED;
    }

    /* Check CPU interrupt ACK bit */
    if (REG_CPU_INT_CTRL & CPU_INT_CTRL_ACK) {
        REG_CPU_INT_CTRL = CPU_INT_CTRL_ACK;
    }

    /* Check CPU DMA interrupt bit */
    if (REG_CPU_DMA_INT & CPU_DMA_INT_ACK) {
        REG_CPU_DMA_INT = 0x02;
        G_CMD_PENDING_07BB = 1;
    }

    /* Check transfer DMA config bit */
    if (REG_XFER_DMA_CFG & XFER_DMA_CFG_ACK) {
        REG_XFER_DMA_CFG = 0x02;
    }

    /* Check transfer2 DMA status bit */
    if (REG_XFER2_DMA_STATUS & XFER2_DMA_STATUS_ACK) {
        REG_XFER2_DMA_STATUS = 0x02;
    }

    /* Check CPU extended status bit */
    if (REG_CPU_EXT_STATUS & CPU_EXT_STATUS_ACK) {
        REG_CPU_EXT_STATUS = 0x02;
    }

    /* b571-b594: Check 9090 bit 7 and 0x0AD3/0x0AD1 */
    state = REG_USB_INT_MASK_9090;
    if ((state & 0x80) == 0) {
        return 5;
    }

    if (G_TLP_MODE_0AD3 <= 0) {
        return 5;
    }

    if (G_LINK_STATE_0AD1 >= 1) {
        return 5;
    }

    /* b595-b5f8: Full processing path */
    G_XFER_STATE_0AF6 = 1;

    if (G_EP_CHECK_FLAG != 0) {
        /* Would call helper_4e25 here */
    }

    nvme_queue_init_905x();

    state = REG_USB_STATUS;
    REG_USB_STATUS = (state & 0xFE) | 1;

    state = nvme_queue_clear_usb_bit0();
    REG_USB_CTRL_9200 = state | 1;

    I_WORK_3C = 0;
    I_WORK_3D = 0;

    /* Clear DMA buffer 0xD800-0xDE60 */
    for (i = 0; i < 0x0660; i++) {
        *(__xdata uint8_t *)(0xD800 + i) = 0;
    }

    REG_USB_EP_BUF_DE30 = 3;
    REG_USB_EP_BUF_DE36 = 0;

    /* b5f9-b623: Final register setup */
    REG_USB_CTRL_9200 = (REG_USB_CTRL_9200 & 0xBF) | 0x40;

    state = REG_USB_MSC_CFG;
    REG_USB_MSC_CFG = state & 0xFE;

    REG_USB_CTRL_9200 = REG_USB_CTRL_9200 & 0xBF;
    nvme_queue_config_9006(REG_USB_CTRL_9200, &REG_USB_CTRL_9200);

    state = REG_USB_EP_CTRL_905F;
    REG_USB_EP_CTRL_905F = (state & 0xF7) | 0x08;

    nvme_queue_set_90e3_2();

    return 4;
}

/*
 * nvme_cmd_setup_b624 - NVMe command setup
 * Address: 0xb624-0xb6ce (~171 bytes)
 *
 * Sets up NVMe command structures for TLP processing.
 */
void nvme_cmd_setup_b624(void)
{
    /* TODO: Implement NVMe command setup */
}

/*
 * nvme_cmd_setup_b6cf - NVMe command setup variant
 * Address: 0xb6cf-0xb778 (~170 bytes)
 *
 * Variant of NVMe command setup.
 */
void nvme_cmd_setup_b6cf(void)
{
    /* TODO: Implement NVMe command setup variant */
}

/*
 * nvme_cmd_setup_b779 - NVMe command setup variant 2
 * Address: 0xb779-0xb81f (~167 bytes)
 *
 * Second variant of NVMe command setup.
 */
void nvme_cmd_setup_b779(void)
{
    /* TODO: Implement NVMe command setup variant 2 */
}

/*
 * tlp_init_addr_buffer - Initialize TLP address buffer
 * Address: 0xb820-0xb832 (19 bytes)
 *
 * Clears the flash/TLP address buffer at 0x0aae-0x0AB0 to zero (32-bit),
 * and clears 0x0AB1-0x0AB2 (length).
 *
 * Original disassembly:
 *   b820: clr a
 *   b821: mov r7, a
 *   b822: mov r6, a
 *   b823: mov r5, a
 *   b824: mov r4, a
 *   b825: mov dptr, #0x0aad
 *   b828: lcall 0x0dc5      ; store dword (R4:R5:R6:R7) to DPTR
 *   b82b: clr a
 *   b82c: mov dptr, #0x0ab1
 *   b82f: movx @dptr, a     ; clear 0x0AB1
 *   b830: inc dptr
 *   b831: movx @dptr, a     ; clear 0x0AB2
 *   b832: ret
 */
void tlp_init_addr_buffer(void)
{
    /* Clear 32-bit address at 0x0aae-0x0AB0 */
    G_FLASH_ADDR_0 = 0;
    G_FLASH_ADDR_1 = 0;
    G_FLASH_ADDR_2 = 0;
    G_FLASH_ADDR_3 = 0;

    /* Clear length at 0x0AB1-0x0AB2 */
    G_FLASH_LEN_LO = 0;
    G_FLASH_LEN_HI = 0;
}

/*
 * nvme_queue_b825 - NVMe queue helper 2
 * Address: 0xb825-0xb832 (14 bytes)
 *
 * Queue helper for DMA setup.
 */
void nvme_queue_b825(void)
{
    /* TODO: Implement queue helper 2 */
}

/*
 * nvme_queue_b833 - NVMe queue helper 3
 * Address: 0xb833-0xb837 (5 bytes)
 */
void nvme_queue_b833(void)
{
    /* TODO: Implement queue helper 3 */
}

/*
 * nvme_queue_b838 - NVMe queue helper 4
 * Address: 0xb838-0xb847 (16 bytes)
 */
void nvme_queue_b838(void)
{
    /* TODO: Implement queue helper 4 */
}

/*
 * tlp_write_flash_cmd - Write to flash command register
 * Address: 0xb845-0xb84f (11 bytes)
 *
 * Writes command to 0xC8AA (flash CMD), reads 0xC8AC (addr length),
 * masks with 0xFC.
 *
 * Original disassembly:
 *   b845: mov dptr, #0xc8aa
 *   b848: movx @dptr, a      ; write A to REG_FLASH_CMD
 *   b849: mov dptr, #0xc8ac
 *   b84c: movx a, @dptr      ; read REG_FLASH_ADDR_LEN
 *   b84d: anl a, #0xfc       ; mask bits 0-1
 *   b84f: ret
 */
uint8_t tlp_write_flash_cmd(uint8_t cmd)
{
    REG_FLASH_CMD = cmd;
    return REG_FLASH_ADDR_LEN & FLASH_ADDR_LEN_MASK;
}

/*
 * nvme_queue_b850 - NVMe queue store
 * Address: 0xb850-0xb850 (1 byte - just movx)
 */
void nvme_queue_b850(void)
{
    /* Single instruction - store */
}

/*
 * nvme_queue_b851 - NVMe queue increment
 * Address: 0xb851-0xb880 (48 bytes)
 *
 * Queue increment and management.
 */
void nvme_queue_b851(void)
{
    /* TODO: Implement queue increment */
}

/*
 * nvme_queue_submit - NVMe handler
 * Address: 0xb881-0xb8a1 (33 bytes)
 */
void nvme_queue_submit(void)
{
    /* TODO: Implement NVMe handler */
}

/*
 * nvme_queue_poll - NVMe handler 3
 * Address: 0xb8b9-0xba05 (~333 bytes)
 *
 * Large NVMe event handler.
 */
void nvme_queue_poll(void)
{
    /* TODO: Implement NVMe handler 3 */
}

/*
 * nvme_completion_poll - NVMe handler 4
 * Address: 0xba06+
 *
 * Final NVMe handler in this range.
 */
void nvme_completion_poll(void)
{
    /* TODO: Implement NVMe handler 4 */
}

/*
 * PCIe interrupt sub-handler forward declarations
 * These functions access registers through extended addressing (Bank 1)
 */
extern uint8_t pcie_check_int_source_a374(uint8_t source);  /* 0xa374 - Check int source via R1 */
extern uint8_t pcie_check_int_source_a3c4(uint8_t source);  /* 0xa3c4 - Check int source (variant) */
extern uint8_t pcie_get_status_a34f(void);                   /* 0xa34f - Read status register 0x4E */
extern void pcie_setup_lane_a310(uint8_t lane);              /* 0xa310 - Setup lane configuration */
extern void pcie_set_state_a2df(uint8_t state);              /* 0xa2df - Set PCIe state */
extern void pcie_handler_e890(void);                          /* 0xe890 - Bank 1 PCIe handler */
extern void pcie_handler_d8d5(void);                          /* 0xd8d5 - PCIe completion handler */
extern uint8_t dispatch_handler_0557(void);                   /* 0x0557 - Dispatch handler */
extern void pcie_write_reg_0633(void);                        /* 0x0633 - Register write helper */
extern void pcie_write_reg_0638(void);                        /* 0x0638 - Register write helper */
extern void pcie_cleanup_05f7(void);                          /* 0x05f7 - Cleanup handler */
extern uint8_t pcie_cleanup_05fc(void);                       /* 0x05fc - Returns 0xF0 */
extern void pcie_handler_e974(void);                          /* 0xe974 - Bank 1 handler */
extern void pcie_handler_e06b(uint8_t param);                 /* 0xe06b - Bank 1 handler with param */
extern void pcie_setup_a38b(uint8_t source);                  /* 0xa38b - Setup helper */
extern uint8_t pcie_get_status_a372(void);                    /* 0xa372 - Get status at 0x40 */

/*
 * pcie_interrupt_handler - Main PCIe interrupt handler
 * Address: 0xa522-0xa62c (~267 bytes)
 *
 * This is the main interrupt handler for PCIe events. It:
 *   1. Checks various interrupt sources and dispatches to sub-handlers
 *   2. Manages interrupt acknowledgment
 *   3. Coordinates NVMe completion events
 *   4. Handles error conditions and cleanup
 *
 * Original disassembly structure:
 *   Phase 1 (0xa522-0xa54c): Check int source 0x03, handle queue events
 *   Phase 2 (0xa54d-0xa577): Check int source 0x8f, handle dispatch
 *   Phase 3 (0xa578-0xa59a): Check bit 1, do lane setup and state change
 *   Phase 4 (0xa59b-0xa5dd): Check bit 0, extended processing with state
 *   Phase 5 (0xa5de-0xa603): Check bit 2, error path
 *   Phase 6 (0xa604-0xa62c): Check bits 0,3, final cleanup
 */
void pcie_interrupt_handler(void)
{
    uint8_t result;
    uint8_t status_af1;
    uint8_t event_flags;
    uint8_t state_val;

    /*
     * Phase 1: Check interrupt source 0x03
     * a522: mov r1, #0x03
     * a524: lcall 0xa374
     * a527: jnb 0xe0.7, 0xa54d  ; if bit 7 not set, skip
     */
    result = pcie_check_int_source_a374(0x03);
    if (result & 0x80) {
        /* Check state flag 0x0af1 bit 4 */
        status_af1 = G_STATE_FLAG_0AF1;
        if (status_af1 & 0x10) {
            /* Check event control 0x09fa bits 0 and 7 */
            event_flags = G_EVENT_CTRL_09FA;
            if (event_flags & 0x81) {
                /* Call queue handler and set bit 2 */
                result = pcie_queue_handler_a62d();
                G_EVENT_CTRL_09FA = result | 0x04;
            }
        }
        /* Write 0x80 to register, then call helpers */
        /* a53f-a54c: setup r3=2,r2=0x12,r1=3, write 0x80, call 0x0be6, 0x0633 */
        pcie_write_reg_0633();
    }

    /*
     * Phase 2: Check interrupt source 0x8f
     * a54d: mov r1, #0x8f
     * a54f: lcall 0xa3c4
     * a552: jnb 0xe0.7, 0xa578  ; if bit 7 not set, skip
     */
    result = pcie_check_int_source_a3c4(0x8f);
    if (result & 0x80) {
        /* Write 0x80 to register, call helper */
        pcie_write_reg_0638();

        /* Check state flag 0x0af1 bit 4 */
        status_af1 = G_STATE_FLAG_0AF1;
        if (status_af1 & 0x10) {
            /* Check event control 0x09fa bits 0 and 7 */
            event_flags = G_EVENT_CTRL_09FA;
            if (event_flags & 0x81) {
                /* Call dispatch handler 0x0557 */
                result = dispatch_handler_0557();
                if (result) {
                    /* Call queue handler and set bits 0-4 */
                    result = pcie_queue_handler_a62d();
                    G_EVENT_CTRL_09FA = result | 0x1f;
                }
            }
        }
    }

    /*
     * Phase 3: Check status bit 1 (link training)
     * a578: lcall 0xa34f
     * a57b: jnb 0xe0.1, 0xa59b  ; if bit 1 not set, skip
     */
    result = pcie_get_status_a34f();
    if (result & TIMER_CSR_EXPIRED) {
        /* Write 0x02 to register */
        /* a57e: mov a, #0x02; lcall 0x0be6 */

        /* Setup lane with parameter 0x35 */
        pcie_setup_lane_a310(0x35);

        /* Set state to 0x03 */
        pcie_set_state_a2df(0x03);

        /* Call bank 1 handler */
        pcie_handler_e890();

        /* Check source 0x43 */
        result = pcie_check_int_source_a3c4(0x43);
        if (result & 0x80) {
            /* Call completion handler */
            pcie_handler_d8d5();
        }
    }

    /*
     * Phase 4: Check status bit 0 (completion)
     * a59b: mov r3, #0x02; mov r2, #0x12; mov r1, #0x07
     * a5a1: lcall 0x0bc8
     * a5a4: jnb 0xe0.0, 0xa5de  ; if bit 0 not set, skip
     */
    /* Read from extended address 0x1207 */
    /* For now, use the status check pattern */
    result = pcie_get_status_a34f();  /* This reads 0x4E, but pattern is similar */
    if (result & 0x01) {
        /* Write 0x01 to register */
        /* a5a7: mov a, #0x01; lcall 0x0be6 */

        /* Setup lane with parameter 0x35 */
        pcie_setup_lane_a310(0x35);

        /* Set state to 0x04 */
        pcie_set_state_a2df(0x04);

        /* Call bank 1 handler */
        pcie_handler_e890();

        /* Clear transaction state */
        G_LANE_STATE_0A9D = 0;

        /* Get status and update lane state */
        result = pcie_get_status_a372();
        G_PCIE_LANE_STATE_0A9E = result;

        /* Double the lane state value */
        state_val = G_PCIE_LANE_STATE_0A9E;
        G_PCIE_LANE_STATE_0A9E = state_val + state_val;

        /* Rotate lane state left through carry */
        state_val = G_LANE_STATE_0A9D;
        /* rlc a is rotate left through carry */
        G_LANE_STATE_0A9D = (state_val << 1);  /* Simplified - actual has carry */

        /* Read lane state for next operation */
        state_val = G_PCIE_LANE_STATE_0A9E;
        /* Write to register with value 0x04 */
        /* a5d3: mov r1, #0x04; lcall 0x0be6 */

        /* Write lane state to register 0x05 */
        state_val = G_LANE_STATE_0A9D;
        /* a5db: mov r1, #0x05; lcall 0x0be6 */
    }

    /*
     * Phase 5: Check status bit 2 (error)
     * a5de: lcall 0xa34f
     * a5e1: jnb 0xe0.2, 0xa604  ; if bit 2 not set, skip
     */
    result = pcie_get_status_a34f();
    if (result & 0x04) {
        /* Write 0x04 to register */
        /* a5e4: mov a, #0x04; lcall 0x0be6 */

        /* Setup lane with parameter 0x35 */
        pcie_setup_lane_a310(0x35);

        /* Set state to 0x05 */
        pcie_set_state_a2df(0x05);

        /* Call bank 1 handler */
        pcie_handler_e890();

        /* Get status at 0x40 */
        result = pcie_get_status_a372();
        if (result & 0x01) {
            /* Call handler 0xe974 */
            pcie_handler_e974();
            /* Call handler with param 0x01 */
            pcie_handler_e06b(0x01);
        }
    }

    /*
     * Phase 6: Final status checks (bits 0 and 3)
     * a604: lcall 0xa34f
     * a607: jnb 0xe0.0, 0xa612  ; if bit 0 not set, skip
     */
    result = pcie_get_status_a34f();
    if (result & 0x01) {
        /* Call cleanup handler */
        pcie_cleanup_05f7();
        /* Setup with source 0x4E */
        pcie_setup_a38b(0x4E);
    }

    /*
     * Check bit 3
     * a612-a62c: setup and final write
     */
    /* Read from extended address 0x124F */
    /* a612: mov r3, #0x02; mov r2, #0x12; mov r1, #0x4f */
    result = pcie_get_status_a34f();  /* Simplified - would read 0x4F */
    if (result & 0x08) {
        /* Call cleanup handler */
        pcie_cleanup_05fc();
        /* Write 0x08 to register 0x4F */
        /* a627: mov a, #0x08; lcall 0x0be6 */
    }
}

/*
 * pcie_queue_handler_a62d - Queue event handler
 * Address: 0xa62d-0xa636 (10 bytes)
 *
 * Handles PCIe queue events.
 *
 * Original disassembly:
 *   a62d: lcall 0xe7ae
 *   a630: mov dptr, #0xe710
 *   a633: movx a, @dptr
 *   a634: anl a, #0xe0
 *   a636: ret
 */
uint8_t pcie_queue_handler_a62d(void)
{
    /* Call bank 1 handler at 0xe7ae */
    /* Read status from link width register, mask with 0xe0 */
    return REG_LINK_WIDTH_E710 & LINK_WIDTH_MASK;
}

/*
 * pcie_set_interrupt_flag - Set interrupt flag
 * Address: 0xa637-0xa643 (13 bytes)
 *
 * Sets interrupt control flag and clears state.
 *
 * Original disassembly:
 *   a637: mov a, #0x01
 *   a639: mov dptr, #0x0ad7
 *   a63c: movx @dptr, a
 *   a63d: mov dptr, #0x0ade
 *   a640: clr a
 *   a641: movx @dptr, a
 *   a642: inc dptr
 *   a643: ret
 */
void pcie_set_interrupt_flag(void)
{
    G_TLP_COUNT_0AD7 = 0x01;
    G_TLP_LIMIT_HI = 0;
}

/*===========================================================================
 * Bank 1 PCIe Address Helper Functions (0x839c-0x83b9)
 *
 * These functions are in Bank 1 (address 0xFF6B-0x17ED5 mapped at 0x8000)
 * and handle PCIe address setup for transactions. They store 32-bit addresses
 * to the global G_PCIE_ADDR at 0x05AF.
 *===========================================================================*/

/* External helper from Bank 1 */
extern void pcie_bank1_helper_e902(void);  /* 0xe902 - Bank 1 setup */

/*
 * pcie_addr_store_839c - Store PCIe address with offset adjustment
 * Bank 1 Address: 0x839c-0x83b8 (29 bytes) [actual addr: 0x10307]
 *
 * Calls e902 helper, loads current address from 0x05AF,
 * then stores back adjusted address (param4 + 4 with borrow handling).
 *
 * The borrow handling: if param4 > 0xFB (251), there's overflow when adding 4,
 * so param3 is decremented by 1.
 *
 * Original disassembly (from ghidra.c):
 *   FUN_CODE_e902();
 *   xdata_load_dword(0x5af);
 *   xdata_store_dword(0x5af, param_1, param_2,
 *                     param_3 - (((0xfb < param_4) << 7) >> 7),
 *                     param_4 + 4);
 *
 * Note: The expression (((0xfb < param_4) << 7) >> 7) evaluates to:
 *   1 if param_4 > 0xFB (i.e., param_4 >= 0xFC, so param_4 + 4 overflows)
 *   0 otherwise
 */
void pcie_addr_store_839c(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    uint8_t borrow;
    uint8_t new_p3;
    uint8_t new_p4;

    /* Call bank 1 helper for setup */
    pcie_bank1_helper_e902();

    /* Calculate adjusted values */
    new_p4 = p4 + 4;
    borrow = (p4 > 0xFB) ? 1 : 0;  /* Borrow if overflow */
    new_p3 = p3 - borrow;

    /* Store 32-bit address to globals */
    G_PCIE_ADDR_0 = p1;
    G_PCIE_ADDR_1 = p2;
    G_PCIE_ADDR_2 = new_p3;
    G_PCIE_ADDR_3 = new_p4;
}

/*
 * pcie_addr_store_83b9 - Store PCIe address with offset (variant)
 * Bank 1 Address: 0x83b9-0x83d5 (29 bytes) [actual addr: 0x10324]
 *
 * Identical to 839c - likely called in different context or a duplicate
 * for code alignment/bank purposes.
 *
 * Original disassembly:
 *   FUN_CODE_e902();
 *   xdata_load_dword(0x5af);
 *   xdata_store_dword(0x5af, param_1, param_2,
 *                     param_3 - (((0xfb < param_4) << 7) >> 7),
 *                     param_4 + 4);
 */
void pcie_addr_store_83b9(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    uint8_t borrow;
    uint8_t new_p3;
    uint8_t new_p4;

    /* Call bank 1 helper for setup */
    pcie_bank1_helper_e902();

    /* Calculate adjusted values */
    new_p4 = p4 + 4;
    borrow = (p4 > 0xFB) ? 1 : 0;  /* Borrow if overflow */
    new_p3 = p3 - borrow;

    /* Store 32-bit address to globals */
    G_PCIE_ADDR_0 = p1;
    G_PCIE_ADDR_1 = p2;
    G_PCIE_ADDR_2 = new_p3;
    G_PCIE_ADDR_3 = new_p4;
}

/*===========================================================================
 * BANK 1 PCIE HANDLERS
 *
 * These functions handle PCIe-related operations in Bank 1. Called via
 * the bank switching mechanism (jump_bank_1 at 0x0311).
 *===========================================================================*/

/*
 * pcie_state_clear_ed02 - Clear PCIe state and check transfer status
 * Bank 1 Address: 0xED02 (file offset 0x16C6D)
 * Size: ~38 bytes (0x16D02-0x16D27)
 *
 * Called by dispatch stub handler_063d. This handler:
 *   1. Calls 0x05C5 helper for PCIe setup
 *   2. Clears XDATA[0x023F] (transfer state)
 *   3. Checks XDATA[0x0775], clears if non-zero
 *   4. Checks XDATA[0x0719] for value 2
 *   5. Returns different values in R7 based on result
 *
 * Return values (via R7):
 *   0 - State was non-zero and cleared
 *   1 - State at 0x0719 != 2
 *   2 - State at 0x0719 == 2 (cleared)
 */
void pcie_state_clear_ed02(void)
{
    /* Call setup helper at 0x05C5 */
    /* TODO: Implement call to 0x05C5 when available */

    /* Clear transfer state at 0x023F */
    G_BANK1_STATE_023F = 0;
}

/*
 * pcie_handler_unused_eef9 - PCIe handler (UNUSED)
 * Bank 1 Address: 0xEEF9 (file offset 0x16E64)
 *
 * Called by handler_063d.
 * NOTE: This address contains all NOPs in the original firmware.
 * This is likely unused/padding space.
 */
void pcie_handler_unused_eef9(void)
{
    /* Empty - original firmware has NOPs at this address */
}

/*
 * pcie_nvme_event_handler - PCIe/NVMe Event Handler
 * Address: 0x052f-0x0533 (5 bytes) -> dispatches to bank 0 0xAF5E
 *
 * Function at 0xAF5E:
 * PCIe/NVMe event handler called when PCIe status bit 6 is set.
 * Handles NVMe command completion and error events.
 *
 * Algorithm:
 *   1. Write 0x0A, 0x0D to 0xC001 (NVMe command register)
 *   2. Call helper 0x538D with R3=0xFF, R2=0x23, R1=0xEE
 *   3. Read 0xE40F, pass to helper 0x51C7
 *   4. Write 0x3A to 0xC001
 *   5. Read 0xE410, pass to helper 0x51C7
 *   6. Write 0x5D to 0xC001
 *   7. Check 0xE40F bits 7, 0, 5 for various dispatch paths
 *
 * Original disassembly:
 *   af5e: mov dptr, #0xc001
 *   af61: mov a, #0x0a
 *   af63: movx @dptr, a
 *   af64: mov a, #0x0d
 *   af66: movx @dptr, a
 *   af67: mov r3, #0xff
 *   af69: mov r2, #0x23
 *   af6b: mov r1, #0xee
 *   af6d: lcall 0x538d
 *   ... (continues with NVMe event processing)
 */
void pcie_nvme_event_handler(void)
{
    uint8_t val;

    /* Write NVMe command sequence to UART THR (debug output) */
    REG_UART_THR = 0x0A;
    REG_UART_THR = 0x0D;

    /* Call helper 0x538D with R3=0xFF, R2=0x23, R1=0xEE */
    /* This reads/processes NVMe response data */

    /* Read NVMe status from command control register */
    val = REG_CMD_CTRL_E40F;

    /* Call helper 0x51C7 with status in R7 */

    /* Write next command 0x3A */
    REG_UART_THR = 0x3A;

    /* Read command control 10 and process */
    val = REG_CMD_CTRL_E410;

    /* Write command 0x5D */
    REG_UART_THR = 0x5D;

    /* Check status bits in command control register */
    val = REG_CMD_CTRL_E40F;

    if (val & 0x80) {
        /* Bit 7 set: call 0xDFDC helper */
    } else if (val & 0x01) {
        /* Bit 0 set: acknowledge, call 0x83D6 */
        REG_CMD_CTRL_E40F = 0x01;
    } else if (val & 0x20) {
        /* Bit 5 set: acknowledge, call 0xDFDC */
        REG_CMD_CTRL_E40F = 0x20;
    }
}

/*
 * pcie_error_dispatch - PCIe Error Dispatch
 * Address: 0x0570-0x0574 (5 bytes)
 *
 * Dispatches to bank 1 code at 0xE911 (file offset 0x1687C)
 * Called from ext1_isr when PCIe/NVMe status & 0x0F != 0.
 *
 * Original disassembly:
 *   0570: mov dptr, #0xe911
 *   0573: ajmp 0x0311
 */
extern void error_handler_pcie_nvme(void);  /* Bank 1: file 0x16911 */
void pcie_error_dispatch(void)
{
    error_handler_pcie_nvme();
}

/*
 * pcie_event_bit5_handler - PCIe Event Handler (bit 5)
 * Address: 0x061a-0x061e (5 bytes)
 *
 * Dispatches to bank 1 code at 0xA066 (file offset 0x11FD1)
 * Called from ext1_isr when event flags & 0x83 and PCIe/NVMe status bit 5 set.
 *
 * Original disassembly:
 *   061a: mov dptr, #0xa066
 *   061d: ajmp 0x0311
 */
extern void error_handler_pcie_bit5(void);  /* Bank 1: file 0x12066 */
void pcie_event_bit5_handler(void)
{
    error_handler_pcie_bit5();
}

/*
 * pcie_timer_bit4_handler - PCIe Timer Handler (bit 4)
 * Address: 0x0593-0x0597 (5 bytes)
 *
 * Called from ext1_isr when event flags & 0x83 and PCIe/NVMe status bit 4 set.
 * Dispatches to 0xC105 in bank 0.
 *
 * Original disassembly:
 *   0593: mov dptr, #0xc105
 *   0596: ajmp 0x0300
 */
extern void jump_bank_0(uint16_t addr);
void pcie_timer_bit4_handler(void)
{
    jump_bank_0(0xC105);
}

/*
 * pcie_tlp_init_and_transfer - Initialize PCIe TLP registers and perform transfer
 * Address: 0xC1F9-0xC26F (119 bytes)
 *
 * This function:
 *   1. Clears 12 PCIe TLP registers (offsets 0x00-0x0B -> addresses 0xB210-0xB21B)
 *   2. Sets format/type based on G_PCIE_DIRECTION bit 0:
 *      - bit 0 = 1: Memory write (fmt_type = 0x40)
 *      - bit 0 = 0: Memory read  (fmt_type = 0x00)
 *   3. Enables TLP control register
 *   4. Sets byte enables to 0x0F
 *   5. Copies 32-bit address from G_PCIE_ADDR (0x05AF) to REG_PCIE_ADDR (0xB218)
 *   6. Triggers transaction and waits for completion
 *   7. For reads: polls for completion data
 *
 * Returns: 0 on write success, link speed on read success, 0xFE/0xFF on error
 *
 * Original disassembly at 0xc1f9:
 *   c1f9: clr a                   ; loop counter = 0
 *   c1fa: mov 0x51, a             ; [0x51] = 0
 *   c1fc: mov a, 0x51             ; loop start
 *   c1fe: lcall 0x9a53            ; pcie_clear_reg_at_offset(a)
 *   c201: inc 0x51                ; counter++
 *   c203: mov a, 0x51
 *   c205: cjne a, #0x0c, 0xc1fc   ; loop until counter == 12
 *   c208: mov dptr, #0x05ae       ; read direction
 *   c20b: movx a, @dptr
 *   c20c: mov dptr, #0xb210       ; FMT_TYPE register
 *   c20f: jnb acc.0, 0xc217       ; if read, jump
 *   c212: mov a, #0x40            ; write fmt_type
 *   c214: movx @dptr, a
 *   c215: sjmp 0xc219
 *   c217: clr a                   ; read fmt_type = 0
 *   c218: movx @dptr, a
 *   c219: mov dptr, #0xb213
 *   c21c: mov a, #0x01
 *   c21e: movx @dptr, a           ; enable TLP control
 *   c21f: mov a, #0x0f
 *   c221: lcall 0x9a30            ; pcie_set_byte_enables
 *   c224: mov dptr, #0x05af       ; address source
 *   c227: lcall 0x0d84            ; load 32-bit value
 *   c22a: mov dptr, #0xb218       ; address target
 *   c22d: lcall 0x0dc5            ; store 32-bit value
 *   c230: lcall 0x999d            ; pcie_clear_and_trigger
 *   c233: lcall 0x99eb            ; poll loop
 *   c236: jz 0xc233
 *   c238: lcall 0x9a95            ; pcie_write_status_complete
 *   c23b: mov dptr, #0x05ae
 *   c23e: movx a, @dptr
 *   c23f: jnb acc.0, 0xc245       ; if read, go to poll
 *   c242: mov r7, #0x00
 *   c244: ret                     ; write complete
 *   c245-c26f: poll and read completion handling
 */
uint8_t pcie_tlp_init_and_transfer(void)
{
    uint8_t i;
    uint8_t direction;
    uint8_t status;

    /* Step 1: Clear 12 PCIe TLP registers (offsets 0-11) */
    for (i = 0; i < 12; i++) {
        pcie_clear_reg_at_offset(i);
    }

    /* Step 2: Read direction and set format/type */
    direction = G_PCIE_DIRECTION;

    if (direction & 0x01) {
        /* Memory write */
        REG_PCIE_FMT_TYPE = PCIE_FMT_MEM_WRITE;  /* 0x40 */
    } else {
        /* Memory read */
        REG_PCIE_FMT_TYPE = PCIE_FMT_MEM_READ;   /* 0x00 */
    }

    /* Step 3: Enable TLP control */
    REG_PCIE_TLP_CTRL = 0x01;

    /* Step 4: Set byte enables to 0x0F */
    pcie_set_byte_enables(0x0F);

    /* Step 5: Copy 32-bit address from globals to PCIe registers */
    REG_PCIE_ADDR_0 = G_PCIE_ADDR_0;
    REG_PCIE_ADDR_1 = G_PCIE_ADDR_1;
    REG_PCIE_ADDR_2 = G_PCIE_ADDR_2;
    REG_PCIE_ADDR_3 = G_PCIE_ADDR_3;

    /* Step 6: Trigger transaction */
    pcie_clear_and_trigger();

    /* Step 7: Poll for initial completion */
    do {
        status = pcie_get_completion_status();
    } while (status == 0);

    /* Write completion status */
    pcie_write_status_complete();

    /* Step 8: Check direction again */
    direction = G_PCIE_DIRECTION;
    if (direction & 0x01) {
        /* Write operation - done */
        return 0;
    }

    /* Step 9: Read operation - poll for completion data */
    return pcie_poll_and_read_completion();
}

/*
 * pcie_init_read_e8f9 - Initialize PCIe direction for read and execute
 * Address: 0xe8f9-0xe901 (Bank 1)
 *
 * Sets direction to read mode and executes a PCIe TLP transaction.
 *
 * Disassembly:
 *   e8f9: clr a                ; a = 0
 *   e8fa: mov dptr, #0x05ae    ; G_PCIE_DIRECTION
 *   e8fd: movx @dptr, a        ; Write 0 (read mode)
 *   e8fe: lcall 0xc1f9         ; pcie_tlp_init_and_transfer
 *   e901: ret
 *
 * Returns: Result from pcie_tlp_init_and_transfer
 */
uint8_t pcie_init_read_e8f9(void)
{
    G_PCIE_DIRECTION = 0;           /* Set direction to read */
    return pcie_tlp_init_and_transfer();
}

/*
 * pcie_init_write_e902 - Initialize PCIe direction for write and execute
 * Address: 0xe902-0xe90a (Bank 1)
 *
 * Sets direction to write mode and executes a PCIe TLP transaction.
 *
 * Disassembly:
 *   e902: mov dptr, #0x05ae    ; G_PCIE_DIRECTION
 *   e905: mov a, #0x01         ; Write mode
 *   e907: movx @dptr, a
 *   e908: ljmp 0xc1f9          ; tail call to pcie_tlp_init_and_transfer
 *
 * Returns: Result from pcie_tlp_init_and_transfer
 */
uint8_t pcie_init_write_e902(void)
{
    G_PCIE_DIRECTION = 1;           /* Set direction to write */
    return pcie_tlp_init_and_transfer();
}

/*===========================================================================
 * Bank 1 PCIe Handler Functions
 *===========================================================================*/

/* Extern declarations for helper functions */
extern void pcie_timer_channels_init(void);
extern void interrupt_enable_bit4(void);
extern void dma_config_write(void);
extern void reg_modify_helper(__xdata uint8_t *reg);
extern void timer0_reset(void);
extern uint8_t pcie_read_link_state(void);
extern void xdata_cond_write(void);

/*
 * pcie_handler_e890 - Bank 1 PCIe link state reset handler
 * Address: 0xe890-0xe89a, 0xe83d-0xe84a, 0xe712-0xe725 (Bank 1)
 *
 * Resets PCIe extended registers and waits for completion.
 *
 * Disassembly (0xe890-0xe89a):
 *   e890: mov r1, #0x37
 *   e892: lcall 0xa351      ; ext_mem_read(0x02, 0x12, 0x37)
 *   e895: anl a, #0x7f      ; Clear bit 7
 *   e897: lcall 0x0be6      ; ext_mem_write
 *   e89a: ljmp 0xe83d       ; Continue
 *
 * Continuation (0xe83d-0xe84a):
 *   e83d: mov r1, #0x38
 *   e83f: lcall 0xa38b      ; Write 0x01 to reg 0x38
 *   e842: mov r1, #0x38     ; Poll loop
 *   e844: lcall 0xa336      ; Read reg 0x38
 *   e847: jb 0xe0.0, 0xe842 ; Loop while bit 0 set
 *   e84a: ljmp 0xe711       ; Continue
 *
 * Continuation (0xe711-0xe725):
 *   e711: mov r1, #0x35
 *   e713: lcall 0xa301      ; ext_mem_read reg 0x35
 *   e716: anl a, #0xc0      ; Keep bits 6-7 only
 *   e718: lcall 0x0be6      ; Write back
 *   e71b: clr a
 *   e71c: lcall 0xa367      ; Write 0 to 0x3C and 0x3D
 *   e71f: lcall 0x0be6      ; Write 0 to 0x3E
 *   e722: inc r1            ; r1 = 0x3F
 *   e723: ljmp 0x0be6       ; Write 0 to 0x3F
 *
 * PCIe extended registers (bank 0x02:0x12xx -> XDATA 0xB2xx):
 *   0xB235: Link config
 *   0xB237: Link status
 *   0xB238: Command trigger
 *   0xB23C-0xB23F: Lane config registers
 */
void pcie_handler_e890(void)
{
    uint8_t val;

    /* Read link status, clear bit 7, write back */
    val = REG_PCIE_LINK_STATUS_EXT;
    val &= 0x7F;
    REG_PCIE_LINK_STATUS_EXT = val;

    /* Write 0x01 to command trigger register */
    REG_PCIE_LINK_TRIGGER = 0x01;

    /* Poll until bit 0 clears (command complete) */
    while (REG_PCIE_LINK_TRIGGER & 0x01) {
        /* Wait for hardware */
    }

    /* Read link config, keep only bits 6-7, write back */
    val = REG_PCIE_LINK_CFG;
    val &= 0xC0;
    REG_PCIE_LINK_CFG = val;

    /* Clear lane config registers 0x3C-0x3F */
    REG_PCIE_EXT_CFG_0 = 0x00;
    REG_PCIE_EXT_CFG_1 = 0x00;
    REG_PCIE_EXT_CFG_2 = 0x00;
    REG_PCIE_EXT_CFG_3 = 0x00;
}

/*
 * pcie_txn_setup_e775 - Setup PCIe transaction from global count
 * Address: 0xe775-0xe787 (19 bytes)
 *
 * Reads transaction count from G_PCIE_TXN_COUNT_LO, looks up entry in
 * the 34-byte transaction table at 0x05B7, and stores bytes 0 and 1
 * of the entry to I_PCIE_TXN_DATA_0 and I_PCIE_TXN_DATA_1.
 *
 * Original disassembly:
 *   e775: mov dptr, #0x05a6  ; G_PCIE_TXN_COUNT_LO
 *   e778: movx a, @dptr
 *   e779: mov r7, a
 *   e77a: lcall 0x99bc       ; DPTR = 0x05b7 + R7*0x22
 *   e77d: movx a, @dptr      ; Read byte from table
 *   e77e: mov r0, #0x61      ; I_PCIE_TXN_DATA_0
 *   e780: mov @r0, a         ; Store to idata 0x61
 *   e781: lcall 0x9980       ; DPTR = 0x05b8 + R7*0x22
 *   e784: movx a, @dptr      ; Read byte from table
 *   e785: inc r0             ; I_PCIE_TXN_DATA_1
 *   e786: mov @r0, a         ; Store to idata 0x62
 *   e787: ret
 */
void pcie_txn_setup_e775(void)
{
    uint8_t count = G_PCIE_TXN_COUNT_LO;
    __xdata uint8_t *entry = G_PCIE_TXN_TABLE + (count * G_PCIE_TXN_ENTRY_SIZE);

    I_PCIE_TXN_DATA_0 = entry[0];
    I_PCIE_TXN_DATA_1 = entry[1];
}

/*
 * pcie_channel_setup_e19e - Set up PCIe channel configuration
 * Address: 0xe19e-0xe1c5 (40 bytes)
 *
 * Disassembly:
 *   e19e: lcall 0xe677        ; Initial setup
 *   e1a1: mov dptr, #0xcc1c   ; PCIe channel control
 *   e1a4: movx a, @dptr
 *   e1a5: anl a, #0xf8        ; Clear bits 0-2
 *   e1a7: orl a, #0x06        ; Set bits 1-2
 *   e1a9: movx @dptr, a
 *   e1aa: mov dptr, #0xcc1e   ; Channel config
 *   e1ad: clr a
 *   e1ae: movx @dptr, a       ; Write 0
 *   e1af: inc dptr            ; 0xcc1f
 *   e1b0: mov a, #0x8b
 *   e1b2: movx @dptr, a       ; Write 0x8b
 *   e1b3: mov dptr, #0xcc5c   ; Secondary channel
 *   e1b6: movx a, @dptr
 *   e1b7: anl a, #0xf8        ; Clear bits 0-2
 *   e1b9: orl a, #0x04        ; Set bit 2
 *   e1bb: movx @dptr, a
 *   e1bc: mov dptr, #0xcc5e   ; Secondary config
 *   e1bf: clr a
 *   e1c0: movx @dptr, a       ; Write 0
 *   e1c1: inc dptr            ; 0xcc5f
 *   e1c2: mov a, #0xc7
 *   e1c4: movx @dptr, a       ; Write 0xc7
 *   e1c5: ret
 */
void pcie_channel_setup_e19e(void)
{
    uint8_t val;

    /* Initial setup */
    pcie_timer_channels_init();

    /* Configure primary PCIe channel 0xCC1C-0xCC1F */
    val = REG_TIMER2_DIV;
    val = (val & 0xF8) | 0x06;  /* Clear bits 0-2, set bits 1-2 */
    REG_TIMER2_DIV = val;

    REG_TIMER2_THRESHOLD_LO = 0x00;
    REG_TIMER2_THRESHOLD_HI = 0x8B;

    /* Configure secondary PCIe channel 0xCC5C-0xCC5F */
    val = REG_TIMER4_DIV;
    val = (val & 0xF8) | 0x04;  /* Clear bits 0-2, set bit 2 */
    REG_TIMER4_DIV = val;

    REG_TIMER4_THRESHOLD_LO = 0x00;
    REG_TIMER4_THRESHOLD_HI = 0xC7;
}

/*
 * pcie_dma_config_e330 - Configure PCIe DMA channels
 * Address: 0xe330-0xe351 (34 bytes)
 *
 * Triggers DMA on 0xCC81, waits, sets bits on 0xCC80 and 0xCC98.
 */
void pcie_dma_config_e330(void)
{
    uint8_t val;

    /* Trigger DMA on 0xCC81 */
    REG_CPU_INT_CTRL = 0x04;
    REG_CPU_INT_CTRL = 0x02;

    /* Wait for DMA */
    interrupt_enable_bit4();

    /* Configure 0xCC80: set bits 0-1 */
    val = REG_CPU_CTRL_CC80;
    REG_CPU_CTRL_CC80 = val | 0x03;

    /* Clear DMA */
    dma_config_write();
    interrupt_enable_bit4();

    /* Configure 0xCC98: set bit 2 */
    val = REG_CPU_DMA_READY;
    REG_CPU_DMA_READY = val | 0x04;
}

/*
 * pcie_channel_disable_e5fe - Disable PCIe channel and clear flags
 * Address: 0xe5fe-0xe616 (25 bytes)
 *
 * Clears bit 0 of 0xC6BD, calls helper, writes to 0xCC33/0xCC34.
 */
void pcie_channel_disable_e5fe(void)
{
    uint8_t val;

    /* Clear bit 0 of 0xC6BD */
    val = REG_PHY_LINK_CTRL_BD;
    REG_PHY_LINK_CTRL_BD = val & 0xFE;

    /* Call helper with dptr=0xC801 - sets bit 5 */
    reg_modify_helper((__xdata uint8_t *)0xC801);

    /* Write 4 to 0xCC33 */
    REG_CPU_EXEC_STATUS_2 = 0x04;

    /* Clear bit 2 of 0xCC34 */
    val = REG_CPU_EXEC_CTRL_2;
    REG_CPU_EXEC_CTRL_2 = val & 0xFB;
}

/*
 * pcie_disable_and_trigger_e74e - Disable PCIe and trigger channel
 * Address: 0xe74e-0xe761 (20 bytes)
 *
 * Clears 0x0B1B, clears bit 4 of 0xCCF8, triggers on 0xCCF9.
 */
void pcie_disable_and_trigger_e74e(void)
{
    uint8_t val;

    /* Clear status at 0x0B1B */
    G_STATE_0B1B = 0;

    /* Clear bit 4 of 0xCCF8 */
    val = REG_CPU_EXT_CTRL;
    REG_CPU_EXT_CTRL = val & 0xEF;

    /* Trigger sequence on 0xCCF9 */
    REG_CPU_EXT_STATUS = 0x04;
    REG_CPU_EXT_STATUS = 0x02;
}

/*
 * pcie_wait_and_ack_e80a - Wait for PCIe status and acknowledge
 * Address: 0xe80a-0xe81a (17 bytes)
 *
 * Calls e50d helper, then polls bit 1 of 0xCC11 until set, then writes 2.
 */
void pcie_wait_and_ack_e80a(void)
{
    /* Call setup helper */
    timer0_reset();

    /* Wait for bit 1 of 0xCC11 to be set */
    while (!(REG_TIMER0_CSR & TIMER_CSR_EXPIRED)) {
        /* spin */
    }

    /* Acknowledge by writing 2 */
    REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
}

/*
 * pcie_trigger_cc11_e8ef - Trigger PCIe on 0xCC11
 * Address: 0xe8ef-0xe8f8 (10 bytes)
 */
void pcie_trigger_cc11_e8ef(void)
{
    REG_TIMER0_CSR = TIMER_CSR_CLEAR;
    REG_TIMER0_CSR = TIMER_CSR_EXPIRED;
}

/*
 * clear_pcie_status_bytes_e8cd - Clear PCIe status bytes
 * Address: 0xe8cd-0xe8d7 (11 bytes)
 *
 * Clears 0x0B34, 0x0B35, 0x0B36, 0x0B37 to 0.
 */
void clear_pcie_status_bytes_e8cd(void)
{
    G_PCIE_WORK_0B34 = 0;
    G_PCIE_STATUS_0B35 = 0;
    G_PCIE_STATUS_0B36 = 0;
    G_PCIE_STATUS_0B37 = 0;
}

/*
 * get_pcie_status_flags_e00c - Build status flags from PCIe buffers
 * Address: 0xe00c-0xe03b (48 bytes)
 *
 * Reads status buffers 0x0B34-0x0B37 and builds a status byte:
 *   bit 0: set if 0x0B34 != 0
 *   bit 1: set if 0x0B35 != 0
 *   bit 2: set if 0x0B36 != 0
 *   bit 3: set if 0x0B37 != 0
 * Then calls pcie_read_link_state, combines results, and writes via xdata_cond_write
 */
uint8_t get_pcie_status_flags_e00c(void)
{
    uint8_t flags = 0;

    if (G_PCIE_WORK_0B34 != 0) flags |= 0x01;
    if (G_PCIE_STATUS_0B35 != 0) flags |= 0x02;
    if (G_PCIE_STATUS_0B36 != 0) flags |= 0x04;
    if (G_PCIE_STATUS_0B37 != 0) flags |= 0x08;

    /* Combine with upper nibble from helper */
    flags |= (pcie_read_link_state() & 0xF0);

    /* Write result via xdata_cond_write */
    xdata_cond_write();

    return flags;
}

/* Extern declarations for phy/timer functions */
extern void phy_link_training(void);
extern void timer_wait(uint8_t timeout_lo, uint8_t timeout_hi, uint8_t mode);

/*
 * pcie_lane_config_helper - PCIe Lane Configuration State Machine
 * Address: 0xdb30-0xdb92 (100 bytes, Bank 0)
 *
 * Purpose: Configures PCIe lane state for link training with target lane count.
 * This is CRITICAL for eGPU - it trains the PCIe link.
 *
 * Algorithm:
 *   1. Store param in G_FLASH_ERROR_1, init G_STATE_COUNTER_0AAC = 1
 *   2. Read current lane state from REG_PCIE_LINK_STATE low nibble
 *   3. Store lane state in G_STATE_HELPER_0AAB
 *   4. Loop up to 4 times (link training retries):
 *      - If param < 0x0F, check if G_STATE_HELPER_0AAB == param
 *      - Otherwise check if G_STATE_HELPER_0AAB == 0x0F
 *      - Merge state values, write to B434, call phy_link_training, delay 200ms
 *   5. Return after success or 4 iterations
 */
void pcie_lane_config_helper(uint8_t param)
{
    uint8_t lane_state, counter, temp;

    G_FLASH_ERROR_1 = param;
    G_STATE_COUNTER_0AAC = 1;

    /* Read current lane state from B434 low nibble */
    lane_state = REG_PCIE_LINK_STATE & PCIE_LINK_STATE_MASK;
    G_STATE_HELPER_0AAB = lane_state;
    G_FLASH_RESET_0AAA = 0;

    /* Loop up to 4 times for link training */
    for (counter = 0; counter < 4; counter++) {
        temp = G_FLASH_ERROR_1;

        if (temp < 0x0F) {
            /* Check if we've reached target lane config */
            if (G_STATE_HELPER_0AAB == temp) {
                return;  /* Success */
            }
            /* Merge lane state with counter */
            temp = (temp | (G_STATE_COUNTER_0AAC ^ 0x0F)) & G_STATE_HELPER_0AAB;
        } else {
            /* Full lane mode - check for 0x0F */
            if (G_STATE_HELPER_0AAB == 0x0F) {
                return;  /* Success */
            }
            /* Set all lanes active */
            temp = G_STATE_COUNTER_0AAC | G_STATE_HELPER_0AAB;
        }

        G_STATE_HELPER_0AAB = temp;

        /* Update B434 with new lane state */
        lane_state = REG_PCIE_LINK_STATE;
        REG_PCIE_LINK_STATE = temp | (lane_state & 0xF0);

        /* Call PHY link training (0xD702) */
        phy_link_training();

        /* Wait ~200ms for link to train (0xE80A with r4=0, r5=199, r7=2) */
        timer_wait(0x00, 0xC7, 0x02);

        /* Shift counter for next iteration */
        G_STATE_COUNTER_0AAC = G_STATE_COUNTER_0AAC * 2;
        G_FLASH_RESET_0AAA++;
    }
}

/*
 * pcie_doorbell_trigger - PCIe doorbell command trigger
 * Address: 0xe617-0xe62e (24 bytes, Bank 0)
 *
 * Sends a command via PCIe doorbell:
 *   1. Writes 4 to STATUS, param to CMD, reads G_SYS_STATUS
 *   2. Writes 1 or 2 to TRIGGER based on status
 *   3. Waits for busy flag to be set
 *   4. Clears the status
 */
void pcie_doorbell_trigger(uint8_t param)
{
    uint8_t status;

    /* Step 1: Setup (equivalent to lcall 0xc45f) */
    REG_PCIE_STATUS = PCIE_STATUS_BUSY;           /* Write 4 to 0xB296 */
    REG_PCIE_DOORBELL_CMD = param;    /* Write param to 0xB251 */
    status = G_SYS_STATUS_PRIMARY;    /* Read from 0x0464 */

    /* Step 2: Write to trigger based on status */
    if (status != 0) {
        REG_PCIE_TRIGGER = 0x02;
    } else {
        REG_PCIE_TRIGGER = 0x01;
    }

    /* Step 3: Wait for busy flag (bit 2) */
    while (!(REG_PCIE_STATUS & PCIE_STATUS_BUSY)) {
        /* Spin wait */
    }

    /* Step 4: Clear status (equivalent to lcall 0xc48f) */
    REG_PCIE_STATUS = PCIE_STATUS_BUSY;
}

/*
 * pcie_tunnel_setup - USB4/PCIe Tunnel Path Setup
 * Address: 0xcd66-0xcdc6 (97 bytes, Bank 0)
 *
 * Configures USB4 tunnel mode for PCIe passthrough:
 *   1. Clears CPU mode bit 4 (exit NVMe mode)
 *   2. Configures adapter registers
 *   3. Bank-switched writes to USB4 tunnel path registers
 *   4. Sets tunnel control and link state registers
 */
void pcie_tunnel_setup(void)
{
    uint8_t tmp;

    /* Clear CPU mode bit 4 (exit NVMe mode) */
    tmp = REG_CPU_MODE_NEXT;
    tmp &= 0xEF;
    REG_CPU_MODE_NEXT = tmp;

    /* Configure adapter registers */
    pcie_adapter_config();

    /* Bank-switched writes to USB4 tunnel path registers
     * Uses SFR 0x93 as bank select (bank=1 for USB4 path)
     * Write 0x22 to 0x4084 and 0x5084 */
    __asm
        mov     0x93, #0x01     ; Set bank = 1
        mov     dptr, #0x4084
        mov     a, #0x22
        movx    @dptr, a        ; bank1[0x4084] = 0x22
        mov     dptr, #0x5084
        movx    @dptr, a        ; bank1[0x5084] = 0x22
        mov     0x93, #0x00     ; Clear bank select
    __endasm;

    /* Set bit 0 of REG_PCIE_TUNNEL_CTRL (0xB401) */
    tmp = REG_PCIE_TUNNEL_CTRL;
    tmp &= 0xFE;  /* Clear bit 0 first */
    tmp |= 0x01;  /* Then set bit 0 */
    REG_PCIE_TUNNEL_CTRL = tmp;

    /* Set bit 0 of REG_TUNNEL_ADAPTER_MODE (0xB482) */
    tmp = REG_TUNNEL_ADAPTER_MODE;
    tmp &= 0xFE;
    tmp |= 0x01;
    REG_TUNNEL_ADAPTER_MODE = tmp;

    /* Set high nibble of 0xB482 to 0xF0 (tunnel mode) */
    tmp = REG_TUNNEL_ADAPTER_MODE;
    tmp &= 0x0F;  /* Keep low nibble */
    tmp |= TUNNEL_MODE_ENABLED;  /* Set high nibble to 0xF0 */
    REG_TUNNEL_ADAPTER_MODE = tmp;

    /* Clear bit 0 of REG_PCIE_TUNNEL_CTRL (0xB401) */
    tmp = REG_PCIE_TUNNEL_CTRL;
    tmp &= 0xFE;
    REG_PCIE_TUNNEL_CTRL = tmp;

    /* Set bit 0 of REG_TUNNEL_LINK_CTRL (0xB480) */
    tmp = REG_TUNNEL_LINK_CTRL;
    tmp &= 0xFE;
    tmp |= 0x01;
    REG_TUNNEL_LINK_CTRL = tmp;

    /* Clear bit 0 of REG_TUNNEL_LINK_STATE (0xB430) */
    tmp = REG_TUNNEL_LINK_STATE;
    tmp &= 0xFE;
    REG_TUNNEL_LINK_STATE = tmp;

    /* Set bit 4 of REG_PCIE_TUNNEL_CFG (0xB298) */
    tmp = REG_PCIE_TUNNEL_CFG;
    tmp &= 0xEF;  /* Clear bit 4 first */
    tmp |= 0x10;  /* Then set bit 4 */
    REG_PCIE_TUNNEL_CFG = tmp;

    /* Final bank-switched writes:
     * Write 0x70 to 0x6043
     * Set bit 7 of 0x6025 */
    __asm
        mov     0x93, #0x01     ; Set bank = 1
        mov     dptr, #0x6043
        mov     a, #0x70
        movx    @dptr, a        ; bank1[0x6043] = 0x70
        mov     dptr, #0x6025
        movx    a, @dptr        ; Read bank1[0x6025]
        anl     a, #0x7f        ; Clear bit 7
        orl     a, #0x80        ; Set bit 7
        movx    @dptr, a        ; Write back
        mov     0x93, #0x00     ; Clear bank select
    __endasm;
}

/* ============================================================
 * PCIe Initialization and Error Handling
 * ============================================================ */

/*
 * pcie_init_error_halt - Initialize PCIe/DMA with error halt
 * Address: 0xd676-0xd701 (140 bytes)
 *
 * This function initializes DMA registers with polling and error handling.
 * IMPORTANT: This function ends with an infinite loop (hang on error).
 *
 * Disassembly:
 *   d676: mov r3, #0xff           ; R3:R2:R1 = 0xFF234B (debug string addr)
 *   d678: mov r2, #0x23
 *   d67a: mov r1, #0x4b
 *   d67c: lcall 0x538d            ; uart_puts
 *   d67f: mov dptr, #0xcc32
 *   d682: lcall 0x9608            ; Set bit 0 of CC32
 *   d685: mov dptr, #0xe7fa
 *   d688: mov a, #0x0f
 *   d68a: movx @dptr, a           ; Write 0x0F to E7FA
 *   d68b: mov dptr, #0xcc88
 *   d68e-d693: Read CC88, clear bits 0-2, set bit 2, write back
 *   d696-d699: Inc to CC89, write 0x31
 *   d69c-d6a0: Poll CC89 bit 1
 *   d6a3: lcall 0x964f            ; cmd_write_cc89_02
 *   d6a6-d6b0: Setup CC31/CC32
 *   d6b1-d6b7: uart_puts with error message
 *   d6ba: sjmp 0xd6ba             ; **INFINITE LOOP - HANG**
 *   d6bc-d701: Error code determination based on R7, R5, R3
 */
void pcie_init_error_halt(void)
{
    uint8_t val;

    /* Print debug message (string at 0xFF234B) */
    /* uart_puts(0xFF234B); */

    /* Set bit 0 of CC32 */
    val = REG_CPU_EXEC_STATUS;
    val = (val & 0xFE) | 0x01;
    REG_CPU_EXEC_STATUS = val;

    /* Write 0x0F to E7FA */
    REG_PHY_LINK_TRIGGER = 0x0F;

    /* CC88: clear bits 0-2, set bit 2 */
    val = REG_XFER_DMA_CTRL;
    val = (val & 0xF8) | 0x04;
    REG_XFER_DMA_CTRL = val;

    /* Write 0x31 to CC89 - start DMA mode 1 */
    REG_XFER_DMA_CMD = XFER_DMA_CMD_START | XFER_DMA_CMD_MODE;

    /* Poll CC89 until transfer complete */
    while (!(REG_XFER_DMA_CMD & XFER_DMA_CMD_DONE)) {
        /* Spin */
    }

    /* Write 0x02 to CC89 */
    cmd_write_cc89_02();

    /* Set bit 0 of CC31 */
    val = REG_CPU_EXEC_CTRL;
    val = (val & 0xFE) | 0x01;
    REG_CPU_EXEC_CTRL = val;

    /* Inc to CC32, clear bit 0 */
    val = REG_CPU_EXEC_STATUS;
    val &= 0xFE;
    REG_CPU_EXEC_STATUS = val;

    /* Print error message (string at 0xFF235C) */
    /* uart_puts(0xFF235C); */

    /* ERROR: Infinite loop - hang the system */
    /* This is intentional - the function never returns */
    while (1) {
        /* Hang forever */
    }
}

/*
 * usb_state_event_handler - Event handler with conditional processing
 * Address: 0xe3d8-0xe3f8 (33 bytes)
 *
 * Disassembly:
 *   e3d8: mov dptr, #0x0b41  ; G_USB_STATE_0B41
 *   e3db: movx a, @dptr      ; Read flags
 *   e3dc: jz 0xe3e3          ; Skip if zero
 *   e3de: mov r7, #0x03      ; Param = 3
 *   e3e0: lcall 0xe3b7       ; Call timer_config_update
 *   e3e3: mov dptr, #0x0aee  ; G_STATE_CHECK_0AEE
 *   e3e6: movx a, @dptr      ; Read state
 *   e3e7: mov r7, a          ; R7 = state
 *   e3e8: lcall 0x3578       ; protocol_param_handler
 *   e3eb: lcall 0xd810       ; dispatch_039a (usb_buffer_handler)
 *   e3ee: clr a
 *   e3ef: mov dptr, #0x07e8  ; G_SYS_FLAGS_07E8
 *   e3f2: movx @dptr, a      ; Clear flags
 *   e3f3: mov dptr, #0x0b2f  ; G_INTERFACE_READY_0B2F
 *   e3f6: inc a              ; A = 1
 *   e3f7: movx @dptr, a      ; Set ready flag
 *   e3f8: ret
 */
void usb_state_event_handler(void)
{
    uint8_t flags;

    /* Check USB state flags */
    flags = G_USB_STATE_0B41;
    if (flags != 0) {
        /* Call timer_config_update with param = 3 */
        timer_config_update(3);
    }

    /* Read state and call protocol_param_handler */
    flags = G_STATE_CHECK_0AEE;
    protocol_param_handler(flags);

    /* Call USB buffer handler (dispatch_039a) */
    dispatch_039a();

    /* Clear system flags and set interface ready */
    G_SYS_FLAGS_07E8 = 0;
    G_INTERFACE_READY_0B2F = 1;
}

/*
 * pcie_param_handler - State check and conditional call helper
 * Address: 0xe7c1-0xe7d3 (19 bytes)
 *
 * Disassembly:
 *   e7c1: mov a, r7            ; Get param
 *   e7c2: cjne a, #0x01, 0xe7c9 ; If param != 1, skip
 *   e7c5: lcall 0xbd14         ; Call state_update_bd14
 *   e7c8: ret
 *   e7c9: mov dptr, #0x0af1    ; G_STATE_FLAG_0AF1
 *   e7cc: movx a, @dptr        ; Read flag
 *   e7cd: jnb 0xe0.4, 0xe7d3   ; If bit 4 clear, skip
 *   e7d0: lcall 0xbcf2         ; Call state_sync_bcf2
 *   e7d3: ret
 *
 * If param == 1: calls state_update_bd14
 * Else: if G_STATE_FLAG_0AF1 bit 4 set, calls state_sync_bcf2
 */
void pcie_param_handler(uint8_t param)
{
    if (param == 0x01) {
        /* Call state_update_bd14 - state update function */
        /* TODO: implement state_update_bd14 call */
        return;
    }

    /* Check G_STATE_FLAG_0AF1 bit 4 */
    if (G_STATE_FLAG_0AF1 & 0x10) {
        /* Call state_sync_bcf2 - state sync function */
        /* TODO: implement state_sync_bcf2 call */
    }
}

/*
 * protocol_setup_32bit - Protocol setup with 32-bit parameter
 * Address: 0xe6d2-0xe6e6 (21 bytes)
 *
 * Disassembly:
 *   e6d2: lcall 0xe396       ; Call helper
 *   e6d5: mov r7, #0x00      ; 32-bit value = 0x00010080
 *   e6d7: mov r6, #0x80
 *   e6d9: mov r5, #0x01
 *   e6db: mov r4, #0x00
 *   e6dd: mov dptr, #0x0b1d
 *   e6e0: lcall 0x0dc5       ; Store 32-bit value
 *   e6e3: lcall 0xd17a       ; Finalize
 *   e6e6: ret
 *
 * Calls protocol_init_setup, then stores 0x00010080 to 0x0b1d, then calls d17a.
 */
uint8_t protocol_setup_32bit(void)
{
    protocol_init_setup();

    /* Store 32-bit value 0x00010080 to 0x0b1d using helper_0dc5 */
    /* The DPTR is set to 0x0b1d before calling 0x0dc5 */
    /* r7:r6:r5:r4 = 0x00:0x80:0x01:0x00 = 0x00010080 (little endian read) */
    G_DMA_WORK_0B1D = 0x00;  /* r4 */
    G_DMA_WORK_0B1E = 0x01;  /* r5 */
    G_DMA_WORK_0B1F = 0x80;  /* r6 */
    G_DMA_WORK_0B20 = 0x00;  /* r7 */

    protocol_finalize();

    return 0;  /* Result in r7 */
}

/*
 * pcie_store_param_transfer - Store param and process transfer
 * Address: 0xe529-0xe544 (28 bytes)
 *
 * Disassembly:
 *   e529: mov dptr, #0x0aa3  ; G_STATE_COUNTER_HI
 *   e52c: mov a, r7          ; Get param
 *   e52d: movx @dptr, a      ; Store it
 *   e52e: clr a
 *   e52f: mov r7, a          ; R7 = 0
 *   e530: lcall 0xdd42       ; phy_link_ctrl_update
 *   e533: lcall 0xe6d2       ; protocol_setup_32bit
 *   e536: mov a, r7          ; Check result
 *   e537: jz 0xe544          ; Return if zero
 *   e539: mov dptr, #0x0aa3  ; G_STATE_COUNTER_HI
 *   e53c: movx a, @dptr      ; Read param
 *   e53d: mov dptr, #0x7000  ; Log buffer base
 *   e540: movx @dptr, a      ; Write to log
 *   e541: lcall 0xe478       ; dispatch_0638 -> pcie_transfer_handler
 *   e544: ret
 *
 * Stores param, calls helper functions, and if result is non-zero,
 * writes saved param to 0x7000 and dispatches to Bank1 handler.
 */
void pcie_store_param_transfer(uint8_t param)
{
    uint8_t result;

    /* Store param to G_STATE_COUNTER_HI */
    G_STATE_COUNTER_HI = param;

    /* Call phy_link_ctrl_update with param = 0 */
    phy_link_ctrl_update(0);

    /* Call protocol_setup_32bit and get result */
    result = protocol_setup_32bit();

    /* If result non-zero, process further */
    if (result != 0) {
        /* Read back saved param and write to flash buffer */
        G_FLASH_BUF_BASE = G_STATE_COUNTER_HI;
        /* Dispatch to Bank1 pcie_transfer_handler */
        pcie_transfer_handler();  /* was: dispatch_0638 */
    }
}

void pcie_int_ctrl_link_init(void)
{
    REG_CPU_INT_CTRL = CPU_INT_CTRL_TRIGGER;
    pcie_link_state_init();
}

void timer_config_update(uint8_t param)
{
    /* Write 0x04 then 0x02 to REG_TIMER1_CSR (start timer) */
    REG_TIMER1_CSR = TIMER_CSR_CLEAR;
    REG_TIMER1_CSR = TIMER_CSR_EXPIRED;

    /* If bit 0 set: clear bit 0 of REG_POWER_MISC_CTRL */
    if (param & 0x01) {
        REG_POWER_MISC_CTRL = REG_POWER_MISC_CTRL & 0xFE;
    }

    /* If bit 1 set: set bit 0 of REG_TUNNEL_LINK_CTRL and call log processor */
    if (param & TIMER_CSR_EXPIRED) {
        REG_TUNNEL_LINK_CTRL = (REG_TUNNEL_LINK_CTRL & 0xFE) | 0x01;
        process_log_entries(0);
    }
}

/*
 * protocol_init_setup - Protocol initialization setup
 * Address: 0xe396-0xe3b6 (33 bytes)
 *
 * Disassembly:
 *   e396: lcall 0xb8b9       ; Call helper
 *   e399: lcall 0xb833       ; Set up base
 *   e39c: mov a, #0x03
 *   e39e: movx @dptr, a      ; Write 0x03
 *   e39f: clr a
 *   e3a0: mov r5, a          ; R5 = 0
 *   e3a1: mov r7, #0x9f      ; R7 = 0x9F
 *   e3a3: lcall 0xbe02       ; Call delay/wait
 *   e3a6: mov dptr, #0x0b21
 *   e3a9: mov a, #0x80
 *   e3ab: movx @dptr, a      ; [0x0b21] = 0x80
 *   e3ac: mov dptr, #0x0b24
 *   e3af: mov a, #0xd8
 *   e3b1: movx @dptr, a      ; [0x0b24] = 0xd8
 *   e3b2: inc dptr           ; dptr = 0x0b25
 *   e3b3: mov a, #0x20
 *   e3b5: movx @dptr, a      ; [0x0b25] = 0x20
 *   e3b6: ret
 */
void protocol_init_setup(void)
{
    /* Complex initialization - calls multiple sub-helpers */
    /* For now, just set up the values at the known addresses */
    G_DMA_WORK_0B21 = 0x80;
    G_DMA_WORK_0B24 = 0xd8;
    G_DMA_WORK_0B25 = 0x20;
}

void pcie_bank1_helper_e902(void)
{
    G_PCIE_DIRECTION = 1;  /* Set write direction */
    pcie_tlp_init_and_transfer();
}

void flash_dma_trigger_handler(void)
{
    uint8_t state;

    /* Read flash read trigger state */
    state = G_FLASH_READ_TRIGGER;

    /* Set I_FLASH_STATE_4D based on state */
    if (state == 0) {
        I_FLASH_STATE_4D = 0;
    } else if (state == 1) {
        I_FLASH_STATE_4D = 0x80;
    } else {
        /* State > 1: early return */
        return;
    }

    /* Store state to XDATA work area (0x0aae-0x0AB0) */
    /* Original stores R4:R5:R6:R7 = 0:0:0:I_FLASH_STATE_4D */
    G_FLASH_ADDR_0 = I_FLASH_STATE_4D;
    G_FLASH_ADDR_1 = 0;
    G_FLASH_ADDR_2 = 0;
    G_FLASH_ADDR_3 = 0;

    /* Clear 0x0AB1 */
    G_FLASH_LEN_LO = 0;

    /* Write 0x80 to 0x0AB2 */
    G_FLASH_LEN_HI = 0x80;

    /* Call DMA handler with R5=3, R7=3 */
    dispatch_04c1();
}

/*
 * pcie_trigger_trampoline - Trampoline to pcie_trigger_cc11_e8ef (0xe8ef)
 * Address: 0x050c
 * Original: mov dptr, #0xe8ef; ajmp 0x0300
 */
void pcie_trigger_trampoline(uint8_t param)
{
    (void)param;
    pcie_trigger_cc11_e8ef();
}

/*
 * pcie_link_state_init - PCIe link status check with state machine
 * Address: 0xbe8b-0xbefa (112 bytes)
 *
 * Reads REG_PHY_MODE_E302, checks bits 4-5 for link state.
 * If link state == 3: short path (delay and return)
 * Otherwise: full initialization with polling loops
 *
 * Original disassembly:
 *   be8b: mov dptr, #0xe302   ; REG_PHY_MODE_E302
 *   be8e: movx a, @dptr
 *   be8f: anl a, #0x30        ; Mask bits 4-5
 *   be91: mov r7, a
 *   be92: swap a              ; Swap nibbles
 *   be93: anl a, #0x0f        ; Keep low nibble
 *   be95: xrl a, #0x03        ; Compare with 3
 *   be97: jz 0xbeeb           ; Jump if link state == 3
 *   [main path: call helpers, poll registers, setup command engine]
 *   beea: ret
 *   [alternate path at beeb: short delay and return]
 */
void pcie_link_state_init(void)
{
    uint8_t val;
    uint8_t link_state;

    /* Read PHY mode register and extract link state (bits 4-5) */
    val = REG_PHY_MODE_E302;
    val &= 0x30;
    link_state = (val >> 4) & 0x0F;

    /* If link state == 3, take short path */
    if (link_state == 0x03) {
        /* Short path: delay and return */
        pcie_short_delay();
        uart_puthex(0);  /* Placeholder for 0x51c7 call */
        /* Delay with 0xFF2285 params - just return */
        return;
    }

    /* Main initialization path */
    pcie_short_delay();
    uart_puthex(0);

    /* Additional delay */
    /* uart_puts with delay params 0xFF2274 */

    /* Call cmd_engine_clear */
    cmd_engine_clear();

    /* Clear command state */
    cmd_engine_wait_idle();

    /* Setup E40F/E40B/DMA registers */
    link_state_init_stub();

    /* Wait for transfer complete */
    while (!(REG_XFER_DMA_CMD & XFER_DMA_CMD_DONE)) {
        /* Spin */
    }

    /* Configure command register E40B */
    cmd_config_e40b();

    /* Write 0 to E403, 0x40 to E404 */
    REG_CMD_CTRL_E403 = 0;
    REG_CMD_CFG_E404 = 0x40;

    /* Read-modify-write E405: clear bits 0-2, set bits 0 and 2 */
    val = REG_CMD_CFG_E405;
    val = (val & 0xF8) | 0x05;
    REG_CMD_CFG_E405 = val;

    /* Read-modify-write E402: clear bits 5-7, set bit 5 */
    val = REG_CMD_STATUS_E402;
    val = (val & 0x1F) | 0x20;
    REG_CMD_STATUS_E402 = val;

    /* Wait for command engine to be ready */
    while (cmd_check_busy()) {
        /* Spin */
    }

    /* Trigger command start */
    cmd_start_trigger();

    /* Wait for busy bit to clear */
    while (REG_CMD_BUSY_STATUS & CMD_BUSY_STATUS_BUSY) {
        /* Spin */
    }

    /* Set PCIe complete flag */
    G_PCIE_COMPLETE_07DF = 1;
}

/*
 * check_link_status_e2b9 - Check link status bit 1 at 0xE765
 * Address: 0xe2b9-0xe2c8 (16 bytes)
 *
 * Disassembly:
 *   e2b9: mov dptr, #0xe765
 *   e2bc: movx a, @dptr
 *   e2bd: anl a, #0x02       ; Check bit 1
 *   e2bf: clr c
 *   e2c0: rrc a              ; Shift bit 1 to bit 0
 *   e2c1: jz 0xe2c6          ; If zero, return 0
 *   e2c3: mov r7, #0x01      ; Return 1
 *   e2c5: ret
 *   e2c6: mov r7, #0x00      ; Return 0
 *   e2c8: ret
 *
 * Returns: 1 if bit 1 of 0xE765 is set, 0 otherwise
 */
uint8_t check_link_status_e2b9(void)
{
    uint8_t val = REG_SYS_CTRL_E765;
    return (val & TIMER_CSR_EXPIRED) ? 1 : 0;
}

/*
 * pcie_check_int_source_a374 - Check interrupt source via extended address
 * Address: 0xa374-0xa37a (7 bytes)
 *
 * Sets up r3=0x02, r2=0x12 and reads from extended address 0x01:0x12:source.
 * Returns the status byte with bit 7 indicating interrupt pending.
 *
 * Original disassembly:
 *   a374: mov r3, #0x02
 *   a376: mov r2, #0x12
 *   a378: ljmp 0x0bc8      ; Generic register read
 */
uint8_t pcie_check_int_source_a374(uint8_t source)
{
    /* Access extended memory at Bank 1 address 0x1200 + source */
    /* This reads from code space in Bank 1 */
    /* Simplified: return a value that won't trigger unnecessary processing */
    (void)source;
    return 0;  /* No interrupt pending */
}

/*
 * pcie_handler_d8d5 - PCIe completion handler
 * Address: 0xd8d5+
 *
 * Handles PCIe transaction completion events.
 */
void pcie_handler_d8d5(void)
{
    /* Completion handler - stub */
}

/*
 * pcie_write_reg_0633 - Power state check and conditional restart
 * Address: 0x0633 -> targets 0xc8c7
 *
 * Checks power state via power_check_state_dde2().
 * If state == 2, triggers a restart by jumping to main entry (0x8000).
 * Otherwise returns (with 0xFF, but return value not used).
 *
 * Original disassembly at 0xc8c7:
 *   c8c7: lcall 0xdde2    ; power_check_state_dde2()
 *   c8ca: mov a, r7       ; result in R7
 *   c8cb: xrl a, #0x02    ; test if result == 2
 *   c8cd: jz 0xc8d2       ; if result == 2, jump
 *   c8cf: mov r7, #0xff   ; return 0xff
 *   c8d1: ret
 *   c8d2: ljmp 0x8000     ; jump to main entry (restart)
 *
 * Note: The ljmp 0x8000 is a soft restart. In our C implementation,
 * we cannot easily replicate this behavior. The function just returns
 * and the caller continues. This may need revisiting if restart
 * behavior is critical.
 */
void pcie_write_reg_0633(void)
{
    extern uint8_t power_check_state_dde2(void);

    uint8_t state = power_check_state_dde2();
    if (state == 2) {
        /* Original firmware jumps to 0x8000 (restart).
         * In C, we cannot easily restart. For now, just return.
         * The caller in pcie_interrupt_handler doesn't check return value.
         */
        return;
    }
    /* state != 2: original returns 0xFF but value not checked */
}

/*
 * pcie_write_reg_0638 - PCIe register write and state clear
 * Address: 0x0638 -> targets 0xeadb (bank 1) -> ajmp 0x6cff (0x16cff)
 *
 * Original disassembly at 0x16cff:
 *   ecff: lcall 0x0be6    ; banked_store_byte (R1/A from caller)
 *   ed02: lcall 0x05c5    ; dispatch_05c5 -> handler at 0xe7fb
 *   ed05: clr a
 *   ed06: mov dptr, #0x023f
 *   ed09: movx @dptr, a   ; clear G_BANK1_STATE_023F
 *   ed0a: ret
 *
 * Called from pcie.c after writing 0x80 to source 0x8F via banked_store_byte.
 * This function continues processing by calling dispatch_05c5 and clearing
 * the bank 1 state flag.
 *
 * Note: The first lcall to banked_store_byte uses register context from caller.
 * In C, we skip that call as the caller already did the register write.
 */
void pcie_write_reg_0638(void)
{
    extern void dispatch_05c5(void);

    /* Call dispatch_05c5 -> 0xe7fb handler */
    dispatch_05c5();

    /* Clear bank 1 state flag */
    G_BANK1_STATE_023F = 0;
}

/*
 * pcie_cleanup_05f7 - PCIe status cleanup with computed index
 * Address: 0x05f7 -> targets 0xcbcc (bank 1)
 *
 * Original disassembly at 0x14bcc:
 *   cbcc: xch a, r0      ; exchange A with R0
 *   cbcd: mov r6, a      ; R6 = original R0
 *   cbce: add a, ACC     ; A = A * 2 (double)
 *   cbd0: anl a, #0x1e   ; mask to get valid index
 *   cbd2: add a, r7      ; add status value
 *   cbd3: mov dptr, #0x0442
 *   cbd6: movx @dptr, a  ; write computed index to 0x0442
 *   cbd7: ljmp 0xd7d9    ; continue to cleanup handler
 *
 * The continuation at 0xd7d9:
 *   d7d9: mov a, #0xff
 *   d7db: movx @dptr, a  ; write 0xFF to 0x0443
 *   d7dc: mov dptr, #0x0b2f
 *   d7df: movx a, @dptr  ; read status
 *   ... (additional status checks)
 *
 * This function is called when PCIe status bit 0 is set.
 * It computes an index from input parameters and writes to state buffer,
 * then continues with cleanup/status update logic.
 *
 * Since the original uses R0/R7 from caller context (which we don't have
 * in C), we implement a simplified version that does the essential cleanup.
 */
void pcie_cleanup_05f7(void)
{
    /* The original function computes an index and writes to 0x0442,
     * then continues to the cleanup handler at 0xd7d9.
     *
     * For C implementation, we simulate the essential behavior:
     * - Write to state buffer at 0x0442/0x0443
     * - The ljmp 0xd7d9 continues cleanup which we inline here
     */

    /* Write 0xFF to the second byte (as done at 0xd7d9) */
    /* XDATA_VAR8(0x0443) = 0xFF; - need to add this global if critical */

    /* The continuation checks XDATA[0x0B2F] and does conditional writes.
     * This is complex state machine logic that depends on runtime state.
     * Keeping as simplified stub for now.
     */
}

/*
 * pcie_cleanup_05fc - Return constant 0xF0
 * Address: 0x05fc -> targets 0xbe88
 *
 * Original disassembly at 0xbe88:
 *   be88: mov r7, #0xf0
 *   be8a: ret
 *
 * Simple function that just returns 0xF0.
 */
uint8_t pcie_cleanup_05fc(void)
{
    return 0xF0;
}

/*
 * pcie_handler_e974 - Empty handler (NOP)
 * Address: 0xe974 (1 byte - just ret)
 *
 * This is an empty handler - firmware only has `ret` at 0xe974.
 */
void pcie_handler_e974(void)
{
    /* Empty - firmware has just `ret` at 0xe974 */
}

/*
 * pcie_check_and_trigger_d5da - Check memory bit 7 and trigger PCIe handlers
 * Address: 0xd5da-0xd5e8 (15 bytes)
 *
 * Reads from banked memory using caller-set R1/R2/R3, checks if bit 7 is set,
 * and if so, triggers pcie_handler_e974 and pcie_handler_e06b(1).
 *
 * This function is called from bank 1 code after setting up:
 *   R1 = address low byte
 *   R2 = address high byte
 *   R3 = memory type (0x01=XDATA, 0x02=CODE, etc.)
 *
 * Original disassembly:
 *   d5da: lcall 0x0bc8       ; banked_load_byte(R1, R2, R3)
 *   d5dd: jnb acc.7, 0xd5e8  ; if bit 7 not set, skip to ret
 *   d5e0: lcall 0xe974       ; pcie_handler_e974()
 *   d5e3: mov r7, #0x01
 *   d5e5: lcall 0xe06b       ; pcie_handler_e06b(1)
 *   d5e8: ret
 *
 * Note: In C, caller must pass parameters explicitly since we can't
 * access caller's R1/R2/R3 register state.
 */
void pcie_check_and_trigger_d5da(uint8_t addrlo, uint8_t addrhi, uint8_t memtype)
{
    uint8_t val;

    /* Read from memory using banked_load_byte */
    val = banked_load_byte(addrlo, addrhi, memtype);

    /* Check if bit 7 is set */
    if (val & 0x80) {
        /* Trigger PCIe handlers */
        pcie_handler_e974();
        pcie_handler_e06b(0x01);
    }
}

void pcie_handler_e06b(uint8_t param)
{
    G_USB_WORK_009F = param;
    ext_mem_read_stub(0x02, 0x12, 0x35);
    G_PCIE_WORK_0B34 = 1;
    param = G_USB_WORK_009F;
    dma_transfer_handler(param);
    G_PCIE_STATUS_0B1C = (G_USB_WORK_009F != 0) ? 1 : 0;
}

/*
 * table_lookup - Table lookup helper
 * Address: 0xa704-0xa713 (16 bytes)
 *
 * Computes DPTR = (0x0AE0:0x0AE1) + R6:R7
 * Used for table-based address calculation.
 *
 * Original disassembly:
 *   a704: mov dptr, #0x0ae1    ; Base low byte address
 *   a707: movx a, @dptr        ; Read low byte
 *   a708: add a, r7            ; Add R7
 *   a709: mov r5, a            ; Save to R5
 *   a70a: mov dptr, #0x0ae0    ; Base high byte address
 *   a70d: movx a, @dptr        ; Read high byte
 *   a70e: addc a, r6           ; Add R6 with carry
 *   a70f: mov 0x82, r5         ; DPL = R5
 *   a711: mov 0x83, a          ; DPH = A
 *   a713: ret
 *
 * Returns: Computed address in DPTR
 */
uint8_t table_lookup(void)
{
    __xdata uint8_t *base_lo = (__xdata uint8_t *)0x0AE1;
    __xdata uint8_t *base_hi = (__xdata uint8_t *)0x0AE0;

    /* This function returns a computed address based on table base
     * For now return 0 as stub - actual return is via DPTR
     */
    (void)base_lo;
    (void)base_hi;
    return 0;
}

void pcie_timer_ctrl_e7c1(uint8_t param)
{
    if (param == 1) {
        /* Disable timer */
        reg_timer_clear_bits();
        return;
    }

    /* If bit 4 of G_STATE_FLAG_0AF1 is set, enable timer */
    if (G_STATE_FLAG_0AF1 & 0x10) {
        reg_timer_setup_and_set_bits();
    }
}

/*
 * phy_set_config_bit0 - Set PHY config enable bit
 * Address: 0xcb05-0xcb0e (10 bytes)
 *
 * Sets bit 0 in REG_PHY_CFG_C6A8.
 *
 * Original disassembly:
 *   cb05: mov dptr, #0xc6a8  ; REG_PHY_CFG_C6A8
 *   cb08: movx a, @dptr      ; Read
 *   cb09: anl a, #0xfe       ; Clear bit 0
 *   cb0b: orl a, #0x01       ; Set bit 0
 *   cb0d: movx @dptr, a      ; Write back
 *   cb0e: ret
 */
void phy_set_config_bit0(void)
{
    uint8_t val;
    val = REG_PHY_CFG_C6A8;
    val &= 0xFE;  /* Clear bit 0 */
    val |= 0x01;  /* Set bit 0 */
    REG_PHY_CFG_C6A8 = val;
}

void helper_e5fe(void)
{
    /* Stub */
}

uint8_t check_link_with_delay_e6a7(void)
{
    uint8_t result;

    /* Wait with timeout parameter 0x1c */
    pcie_stage_address(0x1c);

    /* Check status */
    result = pcie_read_transaction_start();

    if (result != 0) {
        return 0xFF;  /* Error/timeout */
    }

    /* Return bit 0 of PCIe extended status register */
    return REG_PCIE_EXT_STATUS & 0x01;
}

void pcie_lane_init_e7f8(void)
{
    uint8_t val;

    /* Call initial helper */
    xdata_cond_write();

    /* Read extended register 0x37, set bit 7 */
    val = pcie_read_ext_reg(0x37);
    val &= 0x7F;  /* Clear bit 7 */
    val |= 0x80;  /* Set bit 7 */

    /* Write back via helper and continue to e83d */
    /* The ljmp to e83d continues the pcie_handler sequence */
}

/*
 * pcie_stage_address - Store address calculation to G_WORK_05AF
 * Address: 0xe762-0xe774 (19 bytes)
 *
 * Computes a 32-bit address (0x00D0_00_xx) where xx = param,
 * then stores it at G_WORK_05AF (0x05AF).
 */
void pcie_stage_address(uint8_t param)
{
    /* Store 32-bit address value at 0x05AF */
    volatile uint8_t __xdata *ptr = (__xdata uint8_t *)0x05AF;

    /* r4:r5:r6:r7 = (0, 0xD0, 0, param) */
    ptr[0] = 0;     /* r4 - high byte */
    ptr[1] = 0xD0;  /* r5 */
    ptr[2] = 0;     /* r6 */
    ptr[3] = param; /* r7 - low byte */
}

uint8_t pcie_read_transaction_start(void)
{
    G_PCIE_DIRECTION = 0;
    return pcie_setup_tlp_transaction();
}

/*
 * pcie_read_ext_reg - Read PCIe extended register by offset
 * Address: 0xa33d-0xa343 (7 bytes)
 *
 * Disassembly:
 *   a33d: mov r3, #0x02      ; R3 = 0x02 (bank)
 *   a33f: mov r2, #0x12      ; R2 = 0x12 (high byte)
 *   a341: ljmp 0x0bc8        ; banked_load_byte
 *
 * Takes R1 from caller as the register offset within the PCIe extended
 * register space. Reads from bank 0x02:0x12xx which maps to XDATA 0xB2xx.
 *
 * Parameters:
 *   reg_offset - The low byte offset (0x00-0xFF) of the register to read
 *
 * Returns: Value read from PCIe extended register 0xB200 + reg_offset
 */
uint8_t pcie_read_ext_reg(uint8_t reg_offset)
{
    /* Read from PCIe extended register at 0xB200 + offset */
    return XDATA_REG8(0xB200 + reg_offset);
}

void pcie_timer_channels_init(void)
{
    /* Clear buffer flag at 0x044C */
    G_LOG_ACTIVE_044C = 0;

    /* Initialize primary channel 0xCC1D with trigger sequence */
    REG_TIMER2_CSR = TIMER_CSR_CLEAR;
    REG_TIMER2_CSR = TIMER_CSR_EXPIRED;

    /* Initialize secondary channel 0xCC5D with trigger sequence */
    REG_TIMER4_CSR = TIMER_CSR_CLEAR;
    REG_TIMER4_CSR = TIMER_CSR_EXPIRED;
}

void pcie_dma_init_e0e4(void)
{
    /* Initial setup */
    dispatch_helper();

    /* Set up extended memory parameters and call */
    scsi_data_copy(0xFF, 0x52, 0xE6);

    /* Final PCIe/DMA handler */
    uart_wait_tx_ready();
}

/*
 * pcie_read_link_state - Read PCIe extended register 0x34
 * Address: 0xa2ff-0xa307 (9 bytes)
 *
 * Disassembly:
 *   a2ff: mov r1, #0x34      ; R1 = 0x34 (register offset)
 *   a301: mov r3, #0x02      ; R3 = 0x02 (bank)
 *   a303: mov r2, #0x12      ; R2 = 0x12 (high byte)
 *   a305: ljmp 0x0bc8        ; banked_load_byte
 *
 * Reads PCIe extended register 0x34 (link state) from banked memory.
 * Bank 0x02:0x12xx maps to XDATA 0xB2xx.
 */
uint8_t pcie_read_link_state(void)
{
    /* Read from PCIe extended link state register */
    return REG_PCIE_LINK_STATE_EXT;
}

uint8_t check_pcie_status_e239(void)
{
    uint8_t val;

    /* Check USB interrupt mask bit 7 */
    val = REG_USB_INT_MASK_9090;
    if (!(val & 0x80)) {
        return 5;  /* Not ready */
    }

    /* Check 0x0AD3 == 0 */
    if (G_TLP_MODE_0AD3 != 0) {
        return 5;  /* Busy */
    }

    /* Call helper and increment result */
    val = nvme_clear_ep0_status();
    G_TLP_MODE_0AD3 = val + 1;

    /* Copy bit 0 of USB status to USB control buffer */
    val = REG_USB_STATUS & 0x01;
    REG_USB_SETUP_TYPE = val;

    return 3;  /* Success */
}

/*
 * nvme_clear_ep0_status - Clear NVME status register
 * Address: 0xa71b-0xa721 (7 bytes)
 *
 * Original disassembly:
 *   a71b: mov dptr, #0x9003  ; REG_NVME_STATUS_9003
 *   a71e: clr a
 *   a71f: movx @dptr, a      ; Write 0 to 0x9003
 *   a720: inc dptr
 *   a721: ret
 *
 * Returns 0.
 */
uint8_t nvme_clear_ep0_status(void)
{
    REG_USB_EP0_LEN_H = 0;
    return 0;
}

void timer0_configure(uint8_t div_bits, uint8_t threshold_hi, uint8_t threshold_lo)
{
    uint8_t val;

    /* Reset timer */
    pcie_trigger_cc11_e8ef();

    /* Configure timer divisor - clear bits 0-2, set new value */
    val = REG_TIMER0_DIV;
    val = (val & 0xF8) | (div_bits & 0x07);
    REG_TIMER0_DIV = val;

    /* Set threshold value (16-bit across CC12:CC13) */
    REG_TIMER0_THRESHOLD_HI = threshold_hi;
    REG_TIMER0_THRESHOLD_LO = threshold_lo;

    /* Start timer */
    REG_TIMER0_CSR = TIMER_CSR_ENABLE;
}

void timer0_reset(void)
{
    timer0_configure(0, 0, 0);
}

/*
 * pcie_write_transaction_start - Set flag and call helper
 * Address: 0xe902-0xe908 (7 bytes)
 *
 * Sets 0x05AE to 1 and calls pcie_setup_tlp_transaction (opposite of pcie_read_transaction_start).
 */
void pcie_write_transaction_start(void)
{
    G_PCIE_DIRECTION = 1;
    (void)pcie_setup_tlp_transaction();  /* pcie_setup_tlp_transaction is declared earlier */
}

void pcie_trigger_link_init(void)
{
    REG_CPU_INT_CTRL = 0x04;
    pcie_config_helper();
}

void pcie_bd05_wrapper(void)
{
    pcie_status_helper();
}

uint8_t pcie_setup_tlp_transaction(void)
{
    uint8_t i;
    uint8_t val;

    /* Loop 12 times - clear registers 0xB210-0xB21B */
    for (i = 0; i < 12; i++) {
        pcie_clear_reg_at_offset(i);
    }

    /* Check 0x05AE bit 0 and configure PCIe format/type */
    val = G_PCIE_DIRECTION;
    if (val & 0x01) {
        REG_PCIE_FMT_TYPE = 0x40;
    } else {
        REG_PCIE_FMT_TYPE = 0;
    }

    /* Enable TLP control */
    REG_PCIE_TLP_CTRL = 0x01;

    /* Set byte enables to 0x0F */
    pcie_set_byte_enables(0x0F);

    /* Copy 32-bit value from 0x05AF to 0xB218 */
    REG_PCIE_ADDR_0 = G_PCIE_ADDR_0;
    REG_PCIE_ADDR_1 = G_PCIE_ADDR_1;
    REG_PCIE_ADDR_2 = G_PCIE_ADDR_2;
    REG_PCIE_ADDR_3 = G_PCIE_ADDR_3;

    /* Clear and trigger PCIe transaction */
    pcie_clear_and_trigger();

    /* Wait for completion (poll until non-zero) */
    while (pcie_get_completion_status() == 0) {
        /* Busy-wait */
    }

    /* Write completion status */
    pcie_write_status_complete();

    /* Check 0x05AE bit 0 - if set, return success */
    val = G_PCIE_DIRECTION;
    if (val & 0x01) {
        return 0;
    }

    /* Poll 0xB296 for status bits */
    while (1) {
        val = REG_PCIE_STATUS;
        /* Check bit 1 */
        if (val & TIMER_CSR_EXPIRED) {
            break;  /* Bit 1 set - exit poll loop */
        }
        /* Check bit 0 */
        if (!(val & 0x01)) {
            /* Bit 0 not set - timeout, write 1 and return 0xFE */
            REG_PCIE_STATUS = PCIE_STATUS_ERROR;
            return 0xFE;
        }
    }

    /* Read completion data and check for errors */
    val = pcie_read_completion_data();
    if (val != 0) {
        return 0xFF;
    }

    /* Check PCIe completion data alternate */
    val = REG_PCIE_CPL_DATA_ALT;
    if (val != 0) {
        return 0xFF;
    }

    /* Check if completion status equals 4 */
    val = REG_PCIE_CPL_STATUS;
    if (val != 0x04) {
        return 0xFF;
    }

    /* Get link speed on success */
    (void)pcie_get_link_speed();

    return 0;  /* Success */
}

/*
 * pcie_tunnel_init_c00d - PCIe tunnel initialization
 * Address: 0xc00d-0xc088 (~124 bytes)
 *
 * Initializes PCIe tunnel state and clears various status variables.
 * This is a complex function that's called during NVMe initialization.
 */
void pcie_tunnel_init_c00d(void)
{
    /* Check if tunnel already active - skip if G_TUNNEL_ACTIVE is 0 */
    if (G_STATE_FLAG_06E6 == 0) {
        return;
    }

    /* Clear tunnel active flag */
    G_STATE_FLAG_06E6 = 0;
    /* Set sequence numbers */
    G_WORK_06E7 = 1;  /* inc dptr, inc a */
    G_WORK_06E8 = 1;  /* inc dptr */
    /* Clear state variables */
    G_PCIE_TXN_COUNT_HI = 0;
    G_WORK_06EB = 0;
    G_DMA_WORK_05AC = 0;
    G_DMA_WORK_05AD = 0;
    /* Additional tunnel setup would go here */
}

uint8_t pcie_read_ctrl_b402(void)
{
    return REG_PCIE_CTRL_B402 & 0xFD;
}

void pcie_lane_disable_e8a9(uint8_t param)
{
    if (param & 0x01) {
        REG_PCIE_LANE_CTRL_C659 &= 0xFE;
    }
}

/*
 * pcie_txn_index_load - Read PCIe transaction count and set up array access
 * Address: 0x1579-0x157c (4 bytes)
 *
 * Reads G_PCIE_TXN_COUNT_LO then falls through to pcie_txn_array_calc.
 */
void pcie_txn_index_load(void)
{
    uint8_t idx = G_PCIE_TXN_COUNT_LO;
    (void)idx;  /* Sets up for subsequent array access */
}

/*
 * pcie_txn_array_calc - Set up array access with index calculation
 * Address: 0x157d-0x1585 (9 bytes)
 *
 * Disassembly:
 *   157d: mov dptr, #0x05b4  ; Base address (G_PCIE_DIRECTION area)
 *   1580: mov B, #0x22       ; Element size = 34 bytes
 *   1583: ljmp 0x0dd1        ; Array index calculation helper
 *
 * The 0x0dd1 function calculates: DPTR = DPTR + (A * B)
 * This sets DPTR to point to a 34-byte structure in an array at 0x05B4.
 */
void pcie_txn_array_calc(void)
{
    /* This sets up DPTR for array access - DPTR = 0x05B4 + (A * 0x22)
     * In context, A contains the index from prior call */
}

/*
 * pcie_txn_table_lookup - USB/Transfer table lookup helper
 * Address: 0x1c5e-0x1c6b (14 bytes)
 *
 * Reads a value from a table based on G_SYS_STATUS_PRIMARY (0x0464)
 * and stores result in G_PCIE_TXN_COUNT_LO (0x05a6).
 *
 * From ghidra.c FUN_CODE_1c5d:
 *   G_PCIE_TXN_COUNT_LO = table[0x05A8 + *param_1];
 *   where param_1 points to G_SYS_STATUS_PRIMARY (0x0464)
 */
void pcie_txn_table_lookup(void)
{
    uint8_t idx = G_SYS_STATUS_PRIMARY;
    /* Table lookup at 0x05A8 + idx */
    G_PCIE_TXN_COUNT_LO = (&G_EP_CONFIG_05A8)[idx];
}
