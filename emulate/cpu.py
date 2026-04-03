"""
ASM2464PD 8051 CPU Emulator Core

This module implements the Intel 8051 instruction set with extensions
for the ASM2464PD including code banking via DPX register.
"""

from typing import Callable, Optional
from dataclasses import dataclass, field

# Instruction timing (cycles) - simplified, most are 1-2 cycles
INST_CYCLES = {
    'NOP': 1, 'AJMP': 2, 'LJMP': 2, 'RR': 1, 'INC': 1, 'JBC': 2,
    'ACALL': 2, 'LCALL': 2, 'RRC': 1, 'DEC': 1, 'JB': 2, 'RET': 2,
    'RL': 1, 'ADD': 1, 'JNB': 2, 'RETI': 2, 'RLC': 1, 'ADDC': 1,
    'JC': 2, 'ORL': 1, 'JNC': 2, 'ANL': 1, 'JZ': 2, 'XRL': 1,
    'JNZ': 2, 'JMP': 2, 'MOV': 1, 'SJMP': 2, 'MOVC': 2, 'DIV': 4,
    'SUBB': 1, 'MUL': 4, 'CPL': 1, 'CJNE': 2, 'PUSH': 2, 'CLR': 1,
    'SWAP': 1, 'XCH': 1, 'POP': 2, 'SETB': 1, 'DA': 1, 'DJNZ': 2,
    'XCHD': 1, 'MOVX': 2,
}


@dataclass
class CPU8051:
    """8051 CPU emulator with ASM2464PD extensions."""

    # Memory interfaces (set by Memory class)
    read_code: Callable[[int], int] = None
    read_xdata: Callable[[int], int] = None
    write_xdata: Callable[[int, int], None] = None
    read_idata: Callable[[int], int] = None
    write_idata: Callable[[int, int], None] = None
    read_sfr: Callable[[int], int] = None
    write_sfr: Callable[[int, int], None] = None
    read_bit: Callable[[int], bool] = None
    write_bit: Callable[[int, bool], None] = None

    # Registers - accessible via SFR space
    # PC is not in SFR space
    pc: int = 0

    # Cycle counter for timing
    cycles: int = 0

    # Interrupt state
    interrupt_pending: list = field(default_factory=list)
    in_interrupt: bool = False

    # Halt flag
    halted: bool = False

    # Debug/trace
    trace: bool = False
    breakpoints: set = field(default_factory=set)
    _timer0_pending: bool = False  # Timer 0 interrupt pending flag
    _ext0_pending: bool = False     # External Interrupt 0 pending flag
    _ext1_pending: bool = False     # External Interrupt 1 pending flag
    
    # Proxy mode - when True, skip interrupt checking (hardware handles it)
    proxy_mode: bool = False
    _saved_dpx: int = 0  # DPX saved/restored around ISR dispatch

    # SFR addresses
    SFR_ACC = 0xE0
    SFR_B = 0xF0
    SFR_PSW = 0xD0
    SFR_SP = 0x81
    SFR_DPL = 0x82
    SFR_DPH = 0x83
    SFR_DPX = 0x96  # ASM2464PD code bank select
    SFR_IE = 0xA8
    SFR_IP = 0xB8
    SFR_TCON = 0x88
    SFR_TMOD = 0x89
    SFR_P0 = 0x80
    SFR_P1 = 0x90
    SFR_P2 = 0xA0
    SFR_P3 = 0xB0
    SFR_PCON = 0x87
    SFR_SCON = 0x98
    SFR_SBUF = 0x99
    SFR_TL0 = 0x8A
    SFR_TL1 = 0x8B
    SFR_TH0 = 0x8C
    SFR_TH1 = 0x8D

    # PSW bit positions
    PSW_P = 0
    PSW_F1 = 1
    PSW_OV = 2
    PSW_RS0 = 3
    PSW_RS1 = 4
    PSW_F0 = 5
    PSW_AC = 6
    PSW_CY = 7

    # Property accessors for common registers
    @property
    def A(self) -> int:
        return self.read_sfr(self.SFR_ACC)

    @A.setter
    def A(self, value: int):
        self.write_sfr(self.SFR_ACC, value & 0xFF)

    @property
    def B(self) -> int:
        return self.read_sfr(self.SFR_B)

    @B.setter
    def B(self, value: int):
        self.write_sfr(self.SFR_B, value & 0xFF)

    @property
    def PSW(self) -> int:
        return self.read_sfr(self.SFR_PSW)

    @PSW.setter
    def PSW(self, value: int):
        self.write_sfr(self.SFR_PSW, value & 0xFF)

    @property
    def SP(self) -> int:
        return self.read_sfr(self.SFR_SP)

    @SP.setter
    def SP(self, value: int):
        self.write_sfr(self.SFR_SP, value & 0xFF)

    @property
    def DPTR(self) -> int:
        return (self.read_sfr(self.SFR_DPH) << 8) | self.read_sfr(self.SFR_DPL)

    @DPTR.setter
    def DPTR(self, value: int):
        self.write_sfr(self.SFR_DPL, value & 0xFF)
        self.write_sfr(self.SFR_DPH, (value >> 8) & 0xFF)

    @property
    def DPX(self) -> int:
        return self.read_sfr(self.SFR_DPX)

    @DPX.setter
    def DPX(self, value: int):
        self.write_sfr(self.SFR_DPX, value & 0xFF)

    # PSW flag accessors
    @property
    def CY(self) -> bool:
        return bool(self.PSW & (1 << self.PSW_CY))

    @CY.setter
    def CY(self, value: bool):
        if value:
            self.PSW |= (1 << self.PSW_CY)
        else:
            self.PSW &= ~(1 << self.PSW_CY)

    @property
    def AC(self) -> bool:
        return bool(self.PSW & (1 << self.PSW_AC))

    @AC.setter
    def AC(self, value: bool):
        if value:
            self.PSW |= (1 << self.PSW_AC)
        else:
            self.PSW &= ~(1 << self.PSW_AC)

    @property
    def OV(self) -> bool:
        return bool(self.PSW & (1 << self.PSW_OV))

    @OV.setter
    def OV(self, value: bool):
        if value:
            self.PSW |= (1 << self.PSW_OV)
        else:
            self.PSW &= ~(1 << self.PSW_OV)

    def get_regbank(self) -> int:
        """Get current register bank (0-3)."""
        return (self.PSW >> 3) & 0x03

    def get_reg(self, n: int) -> int:
        """Get R0-R7 from current register bank."""
        bank = self.get_regbank()
        addr = (bank * 8) + n
        return self.read_idata(addr)

    def set_reg(self, n: int, value: int):
        """Set R0-R7 in current register bank."""
        bank = self.get_regbank()
        addr = (bank * 8) + n
        self.write_idata(addr, value & 0xFF)

    def push(self, value: int):
        """Push byte onto stack."""
        sp = self.SP + 1
        self.SP = sp
        self.write_idata(sp, value & 0xFF)

    def pop(self) -> int:
        """Pop byte from stack."""
        sp = self.SP
        value = self.read_idata(sp)
        self.SP = sp - 1
        return value

    def fetch(self) -> int:
        """Fetch next instruction byte and increment PC."""
        byte = self.read_code(self.pc)
        self.pc = (self.pc + 1) & 0xFFFF
        return byte

    def fetch16(self) -> int:
        """Fetch 16-bit address (big-endian)."""
        hi = self.fetch()
        lo = self.fetch()
        return (hi << 8) | lo

    def rel_jump(self, offset: int):
        """Perform relative jump with signed offset."""
        if offset > 127:
            offset -= 256
        self.pc = (self.pc + offset) & 0xFFFF

    def get_direct(self, addr: int) -> int:
        """Read from direct address (IDATA or SFR)."""
        if addr >= 0x80:
            return self.read_sfr(addr)
        else:
            return self.read_idata(addr)

    def set_direct(self, addr: int, value: int):
        """Write to direct address (IDATA or SFR)."""
        if addr >= 0x80:
            self.write_sfr(addr, value & 0xFF)
        else:
            self.write_idata(addr, value & 0xFF)

    def _check_interrupts(self):
        """Check for pending interrupts and trigger if enabled."""
        if self.in_interrupt:
            return  # Don't nest interrupts

        # In proxy mode, assume interrupts are enabled - the hardware fired the interrupt
        # so it must be enabled. Don't read IE via proxy (expensive and causes recursion).
        if not self.proxy_mode:
            # Read interrupt enable register
            ie = self.read_sfr(0xA8)  # IE register at 0xA8

            # Global interrupt enable (EA bit 7)
            if not (ie & 0x80):
                return
        else:
            ie = 0xFF  # In proxy mode, assume all interrupts enabled

        # Check External Interrupt 0 (EX0 bit 0)
        # ASM2464PD uses EX0 (at 0x0003) for main ISR at 0x0E33
        if ie & 0x01:  # EX0 enabled
            if hasattr(self, '_ext0_pending') and self._ext0_pending:
                self._ext0_pending = False
                # Save DPX (SFR 0x93) - ISR doesn't save/restore it but
                # DPX affects XDATA banking, so we must preserve it
                self._saved_dpx = self.read_sfr(0x93)
                if self._saved_dpx:
                    self.write_sfr(0x93, 0)  # ISR runs with DPX=0
                # Push return address onto stack before jumping to ISR
                # Order matches LCALL: low byte first, high byte second
                # So high is on top of stack, RET pops high first then low
                self.push(self.pc & 0xFF)         # PC low byte (pushed first)
                self.push((self.pc >> 8) & 0xFF)  # PC high byte (on top)
                self.pc = 0x0003  # INT0 vector
                self.in_interrupt = True
                return

        # Check Timer 0 interrupt (ET0 bit 1)
        if ie & 0x02:  # ET0 enabled
            if hasattr(self, '_timer0_pending') and self._timer0_pending:
                self._timer0_pending = False
                self._trigger_interrupt(1)  # Timer 0 interrupt vector at 0x0B

        # Check External Interrupt 1 (EX1 bit 2)
        # ASM2464PD uses EX1 (at 0x0013) as main ISR, not Timer 0
        if ie & 0x04:  # EX1 enabled
            if hasattr(self, '_ext1_pending') and self._ext1_pending:
                self._ext1_pending = False
                self._trigger_interrupt(2)  # EX1 interrupt vector at 0x13

    def _trigger_interrupt(self, vector: int):
        """Trigger an interrupt with the given vector number.
        
        8051 interrupt vectors:
          0: INT0   -> 0x0003
          1: Timer0 -> 0x000B
          2: INT1   -> 0x0013
          3: Timer1 -> 0x001B
          4: Serial -> 0x0023
          5: Timer2 -> 0x002B
        """
        if self.in_interrupt:
            return

        # Save DPX (SFR 0x93) - ISR doesn't save/restore it but
        # DPX affects XDATA banking, so we must preserve it
        self._saved_dpx = self.read_sfr(0x93)
        if self._saved_dpx:
            self.write_sfr(0x93, 0)  # ISR runs with DPX=0

        # Push current PC onto stack (matches LCALL order)
        # Low byte first, high byte second (high ends up on top)
        self.push(self.pc & 0xFF)         # PC low (pushed first)
        self.push((self.pc >> 8) & 0xFF)  # PC high (on top for RET)

        # Set PC to interrupt vector address
        # Formula: vector * 8 + 3
        self.pc = vector * 8 + 3
        self.in_interrupt = True

    def step(self) -> int:
        """Execute one instruction. Returns cycles consumed."""
        if self.halted:
            return 1

        if self.pc in self.breakpoints:
            self.halted = True
            return 0

        opcode = self.fetch()
        cycles = self.execute(opcode)
        self.cycles += cycles

        # Check for interrupts after executing instruction (so hardware can set flags)
        # In proxy mode, pending flags are set by the proxy when hardware interrupts fire
        self._check_interrupts()

        return cycles

    def execute(self, opcode: int) -> int:
        """Execute instruction by opcode. Returns cycles consumed."""

        # Decode and execute based on opcode
        # This is a large switch-like implementation

        if opcode == 0x00:  # NOP
            return 1

        # AJMP addr11 - 5 variants (0x01, 0x21, 0x41, 0x61, 0x81, 0xA1, 0xC1, 0xE1)
        elif opcode & 0x1F == 0x01:
            addr11 = ((opcode & 0xE0) << 3) | self.fetch()
            self.pc = (self.pc & 0xF800) | addr11
            return 2

        # LJMP addr16
        elif opcode == 0x02:
            self.pc = self.fetch16()
            return 2

        # RR A
        elif opcode == 0x03:
            a = self.A
            self.A = ((a >> 1) | (a << 7)) & 0xFF
            return 1

        # INC A
        elif opcode == 0x04:
            self.A = (self.A + 1) & 0xFF
            return 1

        # INC direct
        elif opcode == 0x05:
            addr = self.fetch()
            self.set_direct(addr, (self.get_direct(addr) + 1) & 0xFF)
            return 1

        # INC @R0
        elif opcode == 0x06:
            addr = self.get_reg(0)
            self.write_idata(addr, (self.read_idata(addr) + 1) & 0xFF)
            return 1

        # INC @R1
        elif opcode == 0x07:
            addr = self.get_reg(1)
            self.write_idata(addr, (self.read_idata(addr) + 1) & 0xFF)
            return 1

        # INC R0-R7
        elif 0x08 <= opcode <= 0x0F:
            n = opcode & 0x07
            self.set_reg(n, (self.get_reg(n) + 1) & 0xFF)
            return 1

        # JBC bit, rel
        elif opcode == 0x10:
            bit = self.fetch()
            rel = self.fetch()
            if self.read_bit(bit):
                self.write_bit(bit, False)
                self.rel_jump(rel)
            return 2

        # ACALL addr11 - 5 variants
        elif opcode & 0x1F == 0x11:
            addr11 = ((opcode & 0xE0) << 3) | self.fetch()
            self.push(self.pc & 0xFF)
            self.push((self.pc >> 8) & 0xFF)
            self.pc = (self.pc & 0xF800) | addr11
            return 2

        # LCALL addr16
        elif opcode == 0x12:
            addr = self.fetch16()
            self.push(self.pc & 0xFF)
            self.push((self.pc >> 8) & 0xFF)
            self.pc = addr
            return 2

        # RRC A
        elif opcode == 0x13:
            a = self.A
            c = 1 if self.CY else 0
            self.CY = bool(a & 1)
            self.A = (c << 7) | (a >> 1)
            return 1

        # DEC A
        elif opcode == 0x14:
            self.A = (self.A - 1) & 0xFF
            return 1

        # DEC direct
        elif opcode == 0x15:
            addr = self.fetch()
            self.set_direct(addr, (self.get_direct(addr) - 1) & 0xFF)
            return 1

        # DEC @R0
        elif opcode == 0x16:
            addr = self.get_reg(0)
            self.write_idata(addr, (self.read_idata(addr) - 1) & 0xFF)
            return 1

        # DEC @R1
        elif opcode == 0x17:
            addr = self.get_reg(1)
            self.write_idata(addr, (self.read_idata(addr) - 1) & 0xFF)
            return 1

        # DEC R0-R7
        elif 0x18 <= opcode <= 0x1F:
            n = opcode & 0x07
            self.set_reg(n, (self.get_reg(n) - 1) & 0xFF)
            return 1

        # JB bit, rel
        elif opcode == 0x20:
            bit = self.fetch()
            rel = self.fetch()
            if self.read_bit(bit):
                self.rel_jump(rel)
            return 2

        # RET
        elif opcode == 0x22:
            hi = self.pop()
            lo = self.pop()
            self.pc = (hi << 8) | lo
            return 2

        # RL A
        elif opcode == 0x23:
            a = self.A
            self.A = ((a << 1) | (a >> 7)) & 0xFF
            return 1

        # ADD A, #imm
        elif opcode == 0x24:
            imm = self.fetch()
            self._add(imm, False)
            return 1

        # ADD A, direct
        elif opcode == 0x25:
            addr = self.fetch()
            self._add(self.get_direct(addr), False)
            return 1

        # ADD A, @R0
        elif opcode == 0x26:
            self._add(self.read_idata(self.get_reg(0)), False)
            return 1

        # ADD A, @R1
        elif opcode == 0x27:
            self._add(self.read_idata(self.get_reg(1)), False)
            return 1

        # ADD A, R0-R7
        elif 0x28 <= opcode <= 0x2F:
            self._add(self.get_reg(opcode & 0x07), False)
            return 1

        # JNB bit, rel
        elif opcode == 0x30:
            bit = self.fetch()
            rel = self.fetch()
            if not self.read_bit(bit):
                self.rel_jump(rel)
            return 2

        # RETI
        elif opcode == 0x32:
            hi = self.pop()
            lo = self.pop()
            self.pc = (hi << 8) | lo
            self.in_interrupt = False
            # Restore DPX (SFR 0x93) saved at interrupt entry
            if self._saved_dpx:
                self.write_sfr(0x93, self._saved_dpx)
                self._saved_dpx = 0
            return 2

        # RLC A
        elif opcode == 0x33:
            a = self.A
            c = 1 if self.CY else 0
            self.CY = bool(a & 0x80)
            self.A = ((a << 1) | c) & 0xFF
            return 1

        # ADDC A, #imm
        elif opcode == 0x34:
            imm = self.fetch()
            self._add(imm, True)
            return 1

        # ADDC A, direct
        elif opcode == 0x35:
            addr = self.fetch()
            self._add(self.get_direct(addr), True)
            return 1

        # ADDC A, @R0
        elif opcode == 0x36:
            self._add(self.read_idata(self.get_reg(0)), True)
            return 1

        # ADDC A, @R1
        elif opcode == 0x37:
            self._add(self.read_idata(self.get_reg(1)), True)
            return 1

        # ADDC A, R0-R7
        elif 0x38 <= opcode <= 0x3F:
            self._add(self.get_reg(opcode & 0x07), True)
            return 1

        # JC rel
        elif opcode == 0x40:
            rel = self.fetch()
            if self.CY:
                self.rel_jump(rel)
            return 2

        # ORL direct, A
        elif opcode == 0x42:
            addr = self.fetch()
            self.set_direct(addr, self.get_direct(addr) | self.A)
            return 1

        # ORL direct, #imm
        elif opcode == 0x43:
            addr = self.fetch()
            imm = self.fetch()
            self.set_direct(addr, self.get_direct(addr) | imm)
            return 2

        # ORL A, #imm
        elif opcode == 0x44:
            self.A = self.A | self.fetch()
            return 1

        # ORL A, direct
        elif opcode == 0x45:
            addr = self.fetch()
            self.A = self.A | self.get_direct(addr)
            return 1

        # ORL A, @R0
        elif opcode == 0x46:
            self.A = self.A | self.read_idata(self.get_reg(0))
            return 1

        # ORL A, @R1
        elif opcode == 0x47:
            self.A = self.A | self.read_idata(self.get_reg(1))
            return 1

        # ORL A, R0-R7
        elif 0x48 <= opcode <= 0x4F:
            self.A = self.A | self.get_reg(opcode & 0x07)
            return 1

        # JNC rel
        elif opcode == 0x50:
            rel = self.fetch()
            if not self.CY:
                self.rel_jump(rel)
            return 2

        # ANL direct, A
        elif opcode == 0x52:
            addr = self.fetch()
            self.set_direct(addr, self.get_direct(addr) & self.A)
            return 1

        # ANL direct, #imm
        elif opcode == 0x53:
            addr = self.fetch()
            imm = self.fetch()
            self.set_direct(addr, self.get_direct(addr) & imm)
            return 2

        # ANL A, #imm
        elif opcode == 0x54:
            self.A = self.A & self.fetch()
            return 1

        # ANL A, direct
        elif opcode == 0x55:
            addr = self.fetch()
            self.A = self.A & self.get_direct(addr)
            return 1

        # ANL A, @R0
        elif opcode == 0x56:
            self.A = self.A & self.read_idata(self.get_reg(0))
            return 1

        # ANL A, @R1
        elif opcode == 0x57:
            self.A = self.A & self.read_idata(self.get_reg(1))
            return 1

        # ANL A, R0-R7
        elif 0x58 <= opcode <= 0x5F:
            self.A = self.A & self.get_reg(opcode & 0x07)
            return 1

        # JZ rel
        elif opcode == 0x60:
            rel = self.fetch()
            if self.A == 0:
                self.rel_jump(rel)
            return 2

        # XRL direct, A
        elif opcode == 0x62:
            addr = self.fetch()
            self.set_direct(addr, self.get_direct(addr) ^ self.A)
            return 1

        # XRL direct, #imm
        elif opcode == 0x63:
            addr = self.fetch()
            imm = self.fetch()
            self.set_direct(addr, self.get_direct(addr) ^ imm)
            return 2

        # XRL A, #imm
        elif opcode == 0x64:
            self.A = self.A ^ self.fetch()
            return 1

        # XRL A, direct
        elif opcode == 0x65:
            addr = self.fetch()
            self.A = self.A ^ self.get_direct(addr)
            return 1

        # XRL A, @R0
        elif opcode == 0x66:
            self.A = self.A ^ self.read_idata(self.get_reg(0))
            return 1

        # XRL A, @R1
        elif opcode == 0x67:
            self.A = self.A ^ self.read_idata(self.get_reg(1))
            return 1

        # XRL A, R0-R7
        elif 0x68 <= opcode <= 0x6F:
            self.A = self.A ^ self.get_reg(opcode & 0x07)
            return 1

        # JNZ rel
        elif opcode == 0x70:
            rel = self.fetch()
            if self.A != 0:
                self.rel_jump(rel)
            return 2

        # ORL C, bit
        elif opcode == 0x72:
            bit = self.fetch()
            self.CY = self.CY or self.read_bit(bit)
            return 2

        # JMP @A+DPTR
        elif opcode == 0x73:
            self.pc = (self.A + self.DPTR) & 0xFFFF
            return 2

        # MOV A, #imm
        elif opcode == 0x74:
            self.A = self.fetch()
            return 1

        # MOV direct, #imm
        elif opcode == 0x75:
            addr = self.fetch()
            imm = self.fetch()
            self.set_direct(addr, imm)
            return 2

        # MOV @R0, #imm
        elif opcode == 0x76:
            imm = self.fetch()
            self.write_idata(self.get_reg(0), imm)
            return 1

        # MOV @R1, #imm
        elif opcode == 0x77:
            imm = self.fetch()
            self.write_idata(self.get_reg(1), imm)
            return 1

        # MOV R0-R7, #imm
        elif 0x78 <= opcode <= 0x7F:
            imm = self.fetch()
            self.set_reg(opcode & 0x07, imm)
            return 1

        # SJMP rel
        elif opcode == 0x80:
            rel = self.fetch()
            self.rel_jump(rel)
            return 2

        # ANL C, bit
        elif opcode == 0x82:
            bit = self.fetch()
            self.CY = self.CY and self.read_bit(bit)
            return 2

        # MOVC A, @A+PC
        elif opcode == 0x83:
            addr = (self.A + self.pc) & 0xFFFF
            self.A = self.read_code(addr)
            return 2

        # DIV AB
        elif opcode == 0x84:
            if self.B == 0:
                self.OV = True
            else:
                q = self.A // self.B
                r = self.A % self.B
                self.A = q
                self.B = r
                self.OV = False
            self.CY = False
            return 4

        # MOV direct, direct
        elif opcode == 0x85:
            src = self.fetch()
            dst = self.fetch()
            self.set_direct(dst, self.get_direct(src))
            return 2

        # MOV direct, @R0
        elif opcode == 0x86:
            addr = self.fetch()
            self.set_direct(addr, self.read_idata(self.get_reg(0)))
            return 2

        # MOV direct, @R1
        elif opcode == 0x87:
            addr = self.fetch()
            self.set_direct(addr, self.read_idata(self.get_reg(1)))
            return 2

        # MOV direct, R0-R7
        elif 0x88 <= opcode <= 0x8F:
            addr = self.fetch()
            self.set_direct(addr, self.get_reg(opcode & 0x07))
            return 2

        # MOV DPTR, #imm16
        elif opcode == 0x90:
            self.DPTR = self.fetch16()
            return 2

        # MOV bit, C
        elif opcode == 0x92:
            bit = self.fetch()
            self.write_bit(bit, self.CY)
            return 2

        # MOVC A, @A+DPTR
        elif opcode == 0x93:
            addr = (self.A + self.DPTR) & 0xFFFF
            self.A = self.read_code(addr)
            return 2

        # SUBB A, #imm
        elif opcode == 0x94:
            imm = self.fetch()
            self._subb(imm)
            return 1

        # SUBB A, direct
        elif opcode == 0x95:
            addr = self.fetch()
            self._subb(self.get_direct(addr))
            return 1

        # SUBB A, @R0
        elif opcode == 0x96:
            self._subb(self.read_idata(self.get_reg(0)))
            return 1

        # SUBB A, @R1
        elif opcode == 0x97:
            self._subb(self.read_idata(self.get_reg(1)))
            return 1

        # SUBB A, R0-R7
        elif 0x98 <= opcode <= 0x9F:
            self._subb(self.get_reg(opcode & 0x07))
            return 1

        # ORL C, /bit
        elif opcode == 0xA0:
            bit = self.fetch()
            self.CY = self.CY or (not self.read_bit(bit))
            return 2

        # MOV C, bit
        elif opcode == 0xA2:
            bit = self.fetch()
            self.CY = self.read_bit(bit)
            return 1

        # INC DPTR
        elif opcode == 0xA3:
            self.DPTR = (self.DPTR + 1) & 0xFFFF
            return 2

        # MUL AB
        elif opcode == 0xA4:
            result = self.A * self.B
            self.A = result & 0xFF
            self.B = (result >> 8) & 0xFF
            self.CY = False
            self.OV = (result > 0xFF)
            return 4

        # Reserved
        elif opcode == 0xA5:
            return 1

        # MOV @R0, direct
        elif opcode == 0xA6:
            addr = self.fetch()
            self.write_idata(self.get_reg(0), self.get_direct(addr))
            return 2

        # MOV @R1, direct
        elif opcode == 0xA7:
            addr = self.fetch()
            self.write_idata(self.get_reg(1), self.get_direct(addr))
            return 2

        # MOV R0-R7, direct
        elif 0xA8 <= opcode <= 0xAF:
            addr = self.fetch()
            self.set_reg(opcode & 0x07, self.get_direct(addr))
            return 2

        # ANL C, /bit
        elif opcode == 0xB0:
            bit = self.fetch()
            self.CY = self.CY and (not self.read_bit(bit))
            return 2

        # CPL bit
        elif opcode == 0xB2:
            bit = self.fetch()
            self.write_bit(bit, not self.read_bit(bit))
            return 1

        # CPL C
        elif opcode == 0xB3:
            self.CY = not self.CY
            return 1

        # CJNE A, #imm, rel
        elif opcode == 0xB4:
            imm = self.fetch()
            rel = self.fetch()
            self.CY = self.A < imm
            if self.A != imm:
                self.rel_jump(rel)
            return 2

        # CJNE A, direct, rel
        elif opcode == 0xB5:
            addr = self.fetch()
            rel = self.fetch()
            val = self.get_direct(addr)
            self.CY = self.A < val
            if self.A != val:
                self.rel_jump(rel)
            return 2

        # CJNE @R0, #imm, rel
        elif opcode == 0xB6:
            imm = self.fetch()
            rel = self.fetch()
            val = self.read_idata(self.get_reg(0))
            self.CY = val < imm
            if val != imm:
                self.rel_jump(rel)
            return 2

        # CJNE @R1, #imm, rel
        elif opcode == 0xB7:
            imm = self.fetch()
            rel = self.fetch()
            val = self.read_idata(self.get_reg(1))
            self.CY = val < imm
            if val != imm:
                self.rel_jump(rel)
            return 2

        # CJNE R0-R7, #imm, rel
        elif 0xB8 <= opcode <= 0xBF:
            imm = self.fetch()
            rel = self.fetch()
            val = self.get_reg(opcode & 0x07)
            self.CY = val < imm
            if val != imm:
                self.rel_jump(rel)
            return 2

        # PUSH direct
        elif opcode == 0xC0:
            addr = self.fetch()
            self.push(self.get_direct(addr))
            return 2

        # CLR bit
        elif opcode == 0xC2:
            bit = self.fetch()
            self.write_bit(bit, False)
            return 1

        # CLR C
        elif opcode == 0xC3:
            self.CY = False
            return 1

        # SWAP A
        elif opcode == 0xC4:
            a = self.A
            self.A = ((a << 4) | (a >> 4)) & 0xFF
            return 1

        # XCH A, direct
        elif opcode == 0xC5:
            addr = self.fetch()
            tmp = self.A
            self.A = self.get_direct(addr)
            self.set_direct(addr, tmp)
            return 1

        # XCH A, @R0
        elif opcode == 0xC6:
            ptr = self.get_reg(0)
            tmp = self.A
            self.A = self.read_idata(ptr)
            self.write_idata(ptr, tmp)
            return 1

        # XCH A, @R1
        elif opcode == 0xC7:
            ptr = self.get_reg(1)
            tmp = self.A
            self.A = self.read_idata(ptr)
            self.write_idata(ptr, tmp)
            return 1

        # XCH A, R0-R7
        elif 0xC8 <= opcode <= 0xCF:
            n = opcode & 0x07
            tmp = self.A
            self.A = self.get_reg(n)
            self.set_reg(n, tmp)
            return 1

        # POP direct
        elif opcode == 0xD0:
            addr = self.fetch()
            self.set_direct(addr, self.pop())
            return 2

        # SETB bit
        elif opcode == 0xD2:
            bit = self.fetch()
            self.write_bit(bit, True)
            return 1

        # SETB C
        elif opcode == 0xD3:
            self.CY = True
            return 1

        # DA A (Decimal Adjust)
        elif opcode == 0xD4:
            a = self.A
            cy = self.CY

            if (a & 0x0F) > 9 or self.AC:
                a += 6
                if a > 0xFF:
                    cy = True
                    a &= 0xFF

            if (a >> 4) > 9 or cy:
                a += 0x60
                if a > 0xFF:
                    cy = True
                    a &= 0xFF

            self.A = a
            self.CY = cy
            return 1

        # DJNZ direct, rel
        elif opcode == 0xD5:
            addr = self.fetch()
            rel = self.fetch()
            val = (self.get_direct(addr) - 1) & 0xFF
            self.set_direct(addr, val)
            if val != 0:
                self.rel_jump(rel)
            return 2

        # XCHD A, @R0
        elif opcode == 0xD6:
            ptr = self.get_reg(0)
            val = self.read_idata(ptr)
            self.write_idata(ptr, (val & 0xF0) | (self.A & 0x0F))
            self.A = (self.A & 0xF0) | (val & 0x0F)
            return 1

        # XCHD A, @R1
        elif opcode == 0xD7:
            ptr = self.get_reg(1)
            val = self.read_idata(ptr)
            self.write_idata(ptr, (val & 0xF0) | (self.A & 0x0F))
            self.A = (self.A & 0xF0) | (val & 0x0F)
            return 1

        # DJNZ R0-R7, rel
        elif 0xD8 <= opcode <= 0xDF:
            rel = self.fetch()
            n = opcode & 0x07
            val = (self.get_reg(n) - 1) & 0xFF
            self.set_reg(n, val)
            if val != 0:
                self.rel_jump(rel)
            return 2

        # MOVX A, @DPTR
        elif opcode == 0xE0:
            self.A = self.read_xdata(self.DPTR)
            return 2

        # CLR A
        elif opcode == 0xE4:
            self.A = 0
            return 1

        # MOV A, direct
        elif opcode == 0xE5:
            addr = self.fetch()
            self.A = self.get_direct(addr)
            return 1

        # MOV A, @R0
        elif opcode == 0xE6:
            self.A = self.read_idata(self.get_reg(0))
            return 1

        # MOV A, @R1
        elif opcode == 0xE7:
            self.A = self.read_idata(self.get_reg(1))
            return 1

        # MOV A, R0-R7
        elif 0xE8 <= opcode <= 0xEF:
            self.A = self.get_reg(opcode & 0x07)
            return 1

        # MOVX @DPTR, A
        elif opcode == 0xF0:
            self.write_xdata(self.DPTR, self.A)
            return 2

        # MOVX A, @R0 (external with P2)
        elif opcode == 0xE2:
            p2 = self.read_sfr(self.SFR_P2)
            addr = (p2 << 8) | self.get_reg(0)
            self.A = self.read_xdata(addr)
            return 2

        # MOVX A, @R1 (external with P2)
        elif opcode == 0xE3:
            p2 = self.read_sfr(self.SFR_P2)
            addr = (p2 << 8) | self.get_reg(1)
            self.A = self.read_xdata(addr)
            return 2

        # MOVX @R0, A (external with P2)
        elif opcode == 0xF2:
            p2 = self.read_sfr(self.SFR_P2)
            addr = (p2 << 8) | self.get_reg(0)
            self.write_xdata(addr, self.A)
            return 2

        # MOVX @R1, A (external with P2)
        elif opcode == 0xF3:
            p2 = self.read_sfr(self.SFR_P2)
            addr = (p2 << 8) | self.get_reg(1)
            self.write_xdata(addr, self.A)
            return 2

        # CPL A - complement accumulator
        elif opcode == 0xF4:
            self.A = (~self.A) & 0xFF
            return 1

        # MOV direct, A
        elif opcode == 0xF5:
            addr = self.fetch()
            self.set_direct(addr, self.A)
            return 1

        # MOV @R0, A
        elif opcode == 0xF6:
            self.write_idata(self.get_reg(0), self.A)
            return 1

        # MOV @R1, A
        elif opcode == 0xF7:
            self.write_idata(self.get_reg(1), self.A)
            return 1

        # MOV R0-R7, A
        elif 0xF8 <= opcode <= 0xFF:
            self.set_reg(opcode & 0x07, self.A)
            return 1

        else:
            raise ValueError(f"Unknown opcode: 0x{opcode:02X} at PC=0x{self.pc-1:04X}")

    def _add(self, value: int, with_carry: bool):
        """ADD/ADDC helper - adds value to A with flags."""
        a = self.A
        c = (1 if self.CY else 0) if with_carry else 0

        result = a + value + c

        # Carry from bit 7
        self.CY = result > 0xFF

        # Auxiliary carry from bit 3
        self.AC = ((a & 0x0F) + (value & 0x0F) + c) > 0x0F

        # Overflow: both operands same sign, result different sign
        self.OV = ((a ^ result) & (value ^ result) & 0x80) != 0

        self.A = result & 0xFF

    def _subb(self, value: int):
        """SUBB helper - subtracts value and borrow from A with flags."""
        a = self.A
        c = 1 if self.CY else 0

        result = a - value - c

        # Borrow from bit 8
        self.CY = result < 0

        # Auxiliary borrow from bit 4
        self.AC = ((a & 0x0F) - (value & 0x0F) - c) < 0

        # Overflow
        self.OV = (((a ^ value) & (a ^ result)) & 0x80) != 0

        self.A = result & 0xFF

    def reset(self):
        """Reset CPU to initial state."""
        self.pc = 0
        self.cycles = 0
        self.halted = False
        self.in_interrupt = False
        self.interrupt_pending.clear()
