/*
 * MMIO Proxy Firmware for ASM2464PD
 *
 * This firmware runs on real hardware and proxies MMIO reads/writes from
 * an emulator running the original firmware. This allows USB enumeration
 * to happen on real hardware while firmware executes in the emulator.
 *
 * Protocol (binary):
 *   CMD_ECHO (0x00):      Send 1 byte -> receive 2 bytes: <value> <~value>
 *   CMD_READ (0x01):      Send addr_hi, addr_lo -> receive 2 bytes: <value> <~value>
 *   CMD_WRITE (0x02):     Send addr_hi, addr_lo, value -> receive 2 bytes: 0x00 0xFF
 *   CMD_SFR_READ (0x03):  Send sfr_addr -> receive 2 bytes: <value> <~value>
 *   CMD_SFR_WRITE (0x04): Send sfr_addr, value -> receive 2 bytes: 0x00 0xFF
 *   CMD_INT_ACK (0x05):   Send int_mask -> receive 2 bytes: 0x00 0xFF (ACK ISR complete)
 *
 * Interrupt signaling:
 *   After each command response, if any NEW interrupts fired, proxy sends:
 *     0xFE 0xFE <bitmask>
 *   where bitmask has bits set for each pending interrupt:
 *     bit 0 = INT0, bit 1 = Timer0, bit 2 = INT1, bit 3 = Timer1,
 *     bit 4 = Serial, bit 5 = Timer2
 *
 * All responses are 2 bytes: byte[0]=value, byte[1]=~value (complement for verification).
 * Interrupt signal is 3 bytes (0xFE 0xFE <bitmask>).
 */

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

/* UART registers */
#define REG_UART_RBR    (*(__xdata volatile uint8_t *)0xC000)
#define REG_UART_THR    (*(__xdata volatile uint8_t *)0xC001)
#define REG_UART_RFBR   (*(__xdata volatile uint8_t *)0xC005)
#define REG_UART_TFBF   (*(__xdata volatile uint8_t *)0xC006)
#define REG_UART_LCR    (*(__xdata volatile uint8_t *)0xC007)

/* Protocol commands */
#define CMD_ECHO        0x00
#define CMD_READ        0x01
#define CMD_WRITE       0x02
#define CMD_SFR_READ    0x03
#define CMD_SFR_WRITE   0x04
#define CMD_INT_ACK     0x05

/* Interrupt signal - 0x7E followed by int_mask (0x00-0x3F)
 * This can never be confused with a valid response since ~0x7E = 0x81,
 * and int_mask high bits are never set */
#define INT_SIGNAL      0x7E

/* Place proxy variables at high XDATA addresses so firmware writes to low
 * XDATA (0x0000+) don't corrupt them. XDATA 0x5FF0-0x5FFF is safe since
 * MMIO starts at 0x6000 and firmware RAM typically uses lower addresses. */
volatile __xdata __at(0x5FF0) uint8_t pending_int_mask;
volatile __xdata __at(0x5FF1) uint8_t sent_int_mask;
__xdata __at(0x5FF2) uint8_t shadow_ie;

/*
 * SFR access - SFRs are at 0x80-0xFF and require direct addressing.
 */
__sfr __at(0xA8) SFR_IE;    /* Interrupt Enable */
__sfr __at(0xB8) SFR_IP;    /* Interrupt Priority */
__sfr __at(0x88) SFR_TCON;  /* Timer Control */
__sfr __at(0x89) SFR_TMOD;  /* Timer Mode */
__sfr __at(0x8A) SFR_TL0;   /* Timer 0 Low */
__sfr __at(0x8B) SFR_TL1;   /* Timer 1 Low */
__sfr __at(0x8C) SFR_TH0;   /* Timer 0 High */
__sfr __at(0x8D) SFR_TH1;   /* Timer 1 High */
__sfr __at(0xD0) SFR_PSW;   /* Program Status Word */
__sfr __at(0xE0) SFR_ACC;   /* Accumulator */
__sfr __at(0xF0) SFR_B;     /* B Register */
__sfr __at(0x81) SFR_SP;    /* Stack Pointer */
__sfr __at(0x82) SFR_DPL;   /* Data Pointer Low */
__sfr __at(0x83) SFR_DPH;   /* Data Pointer High */
__sfr __at(0x87) SFR_PCON;  /* Power Control */

static uint8_t uart_getc(void)
{
    while (REG_UART_RFBR == 0);
    return REG_UART_RBR;
}

static void uart_putc(uint8_t val)
{
    while (REG_UART_TFBF == 0);
    REG_UART_THR = val;
}

static void send_response(uint8_t val)
{
    uart_putc(val);
    uart_putc(~val);
}

static void send_ack(void)
{
    uart_putc(0x00);
    uart_putc(0xFF);
}

static uint8_t is_uart_addr(uint16_t addr)
{
    return (addr >= 0xC000 && addr <= 0xC00F);
}

static uint8_t xdata_read(uint16_t addr)
{
    if (is_uart_addr(addr)) {
        if (addr == 0xC009) return 0x60;
        return 0x00;
    }
    return *(__xdata volatile uint8_t *)addr;
}

static void xdata_write(uint16_t addr, uint8_t val)
{
    if (is_uart_addr(addr)) return;
    *(__xdata volatile uint8_t *)addr = val;
}

static uint8_t sfr_read(uint8_t addr)
{
    switch (addr) {
        case 0xA8: return shadow_ie;  /* Return what emulator expects */
        case 0xB8: return SFR_IP;
        case 0x88: return SFR_TCON;
        case 0x89: return SFR_TMOD;
        case 0x8A: return SFR_TL0;
        case 0x8B: return SFR_TL1;
        case 0x8C: return SFR_TH0;
        case 0x8D: return SFR_TH1;
        case 0xD0: return SFR_PSW;
        case 0xE0: return SFR_ACC;
        case 0xF0: return SFR_B;
        case 0x81: return SFR_SP;
        case 0x82: return SFR_DPL;
        case 0x83: return SFR_DPH;
        case 0x87: return SFR_PCON;
        default: return 0x00;
    }
}

static void sfr_write(uint8_t addr, uint8_t val)
{
    switch (addr) {
        case 0xA8: shadow_ie = val; break;
        case 0xB8: SFR_IP = val; break;
        case 0x88: SFR_TCON = val; break;
        case 0x89: SFR_TMOD = val; break;
        case 0x8A: SFR_TL0 = val; break;
        case 0x8B: SFR_TL1 = val; break;
        case 0x8C: SFR_TH0 = val; break;
        case 0x8D: SFR_TH1 = val; break;
        case 0xD0: SFR_PSW = val; break;
        case 0xE0: SFR_ACC = val; break;
        case 0xF0: SFR_B = val; break;
        case 0x81: SFR_SP = val; break;
        case 0x82: SFR_DPL = val; break;
        case 0x83: SFR_DPH = val; break;
        case 0x87: SFR_PCON = val; break;
        default: break;
    }
}

/*
 * Interrupt handlers - just set flag bit and return.
 * We track which interrupts we've sent to emulator in sent_int_mask.
 */
void int0_isr(void) __interrupt(0)   { pending_int_mask |= (1 << 0); }
void timer0_isr(void) __interrupt(1) { pending_int_mask |= (1 << 1); }
void int1_isr(void) __interrupt(2)   { pending_int_mask |= (1 << 2); }
void timer1_isr(void) __interrupt(3) { pending_int_mask |= (1 << 3); }
void serial_isr(void) __interrupt(4) { pending_int_mask |= (1 << 4); }
void timer2_isr(void) __interrupt(5) { pending_int_mask |= (1 << 5); }

void main(void)
{
    uint8_t cmd, val, addr_hi, addr_lo, mask;
    uint16_t addr;

    // global init doesn't work, need to do it here
    pending_int_mask = 0;
    sent_int_mask = 0;
    shadow_ie = 0;

    /* Disable parity to get 8N1 */
    REG_UART_LCR &= 0xF7;

    /* Hello */
    uart_putc('P');
    uart_putc('K');
    uart_putc('\n');

    /* Proxy loop */
    while (1) {
        /* Enable interrupts, but mask out any currently being handled by emulator.
         * This prevents the same interrupt from firing again before emulator ACKs it.
         * sent_int_mask bits: 0=INT0, 1=T0, 2=INT1, 3=T1, 4=Serial, 5=T2
         * IE bits: 0=EX0, 1=ET0, 2=EX1, 3=ET1, 4=ES, 5=ET2, 7=EA
         * They match! So we can directly mask them. */
        SFR_IE = shadow_ie & ~sent_int_mask;

        /* Wait for UART data - interrupts can fire here */
        while (REG_UART_RFBR == 0);

        SFR_IE = 0;

        /* Read command byte */
        cmd = REG_UART_RBR;

        switch (cmd) {
        case CMD_ECHO:
            val = uart_getc();
            send_response(val);
            break;

        case CMD_READ:
            addr_hi = uart_getc();
            addr_lo = uart_getc();
            addr = ((uint16_t)addr_hi << 8) | addr_lo;
            val = xdata_read(addr);
            send_response(val);
            break;

        case CMD_WRITE:
            addr_hi = uart_getc();
            addr_lo = uart_getc();
            val = uart_getc();
            addr = ((uint16_t)addr_hi << 8) | addr_lo;
            xdata_write(addr, val);
            send_ack();
            break;

        case CMD_SFR_READ:
            addr_lo = uart_getc();
            val = sfr_read(addr_lo);
            send_response(val);
            break;

        case CMD_SFR_WRITE:
            addr_lo = uart_getc();
            val = uart_getc();
            sfr_write(addr_lo, val);
            send_ack();
            break;

        case CMD_INT_ACK:
            /* Emulator completed ISR (RETI) - clear from sent mask */
            val = uart_getc();
            sent_int_mask &= ~val;
            /* Also clear from pending so it can fire again */
            pending_int_mask &= ~val;
            send_ack();
            break;

        default:
            uart_putc('?');
            uart_putc('?');
            break;
        }

        /* Check for NEW interrupts (fired but not yet sent to emulator) */
        mask = pending_int_mask & ~sent_int_mask;
        if (mask) {
            sent_int_mask |= mask;
            uart_putc(INT_SIGNAL);  /* 0x7E */
            uart_putc(mask);        /* 0x00-0x3F, never 0x81 (~0x7E) */
        }

    }
}
