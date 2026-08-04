#!/usr/bin/env python3
"""
Register documentation probe for ASM2464PD via the UART MMIO proxy.

Requires the proxy firmware flashed (make -C proxy flash-proxy).

Subcommands:
  dump            Read all curated registers and print current values
  bits ADDR...    Bit-behavior test per register:
                    init -> write 0x00 -> write 0xFF -> restore
                    classifies each bit: R/W, RO0 (stuck 0), RO1 (stuck 1),
                    VOL (changed on its own), W1C-ish
  ltssm           Watch B450/E765/bank1-0x4092 once per 100ms for N seconds

Safe-list approach: only registers in REGS are probed by 'dump'.
'bits' is explicit-address only (you take responsibility for the write).
"""
import sys
import time

sys.path.insert(0, "emulate")
from uart_proxy import UARTProxy

# (addr, name, dpx) — registers in the PCIe bring-up / link-training path
REGS = [
    # PCIe TLP engine / bridge config
    (0xB210, "PCIE_FMT_TYPE", 0),
    (0xB213, "PCIE_TLP_CTRL", 0),
    (0xB216, "PCIE_TLP_LENGTH", 0),
    (0xB223, "PCIE_EXT_STATUS", 0),
    (0xB296, "PCIE_STATUS", 0),
    (0xB297, "PCIE_BRIDGE_CTRL", 0),
    (0xB298, "PCIE_TUNNEL_CFG", 0),
    (0xB2D5, "PCIE_CTRL_B2D5", 0),
    # PCIe link / bridge config (0xB4xx)
    (0xB401, "PCIE_TUNNEL_CTRL", 0),
    (0xB402, "PCIE_CTRL_B402", 0),
    (0xB403, "TUNNEL_CTRL_B403", 0),
    (0xB404, "PCIE_LINK_PARAM_B404", 0),
    (0xB424, "PCIE_LANE_COUNT", 0),
    (0xB430, "TUNNEL_LINK_STATE", 0),
    (0xB431, "TUNNEL_LINK_STATUS", 0),
    (0xB432, "POWER_CTRL_B432", 0),
    (0xB434, "PCIE_LINK_STATE", 0),
    (0xB436, "PCIE_LANE_CONFIG", 0),
    (0xB438, "PCIE_LINK_TRAIN", 0),
    (0xB450, "PCIE_LTSSM_STATE", 0),
    (0xB451, "PCIE_LTSSM_B451", 0),
    (0xB452, "PCIE_LTSSM_B452", 0),
    (0xB453, "PCIE_LTSSM_B453", 0),
    (0xB454, "PCIE_LTSSM_B454", 0),
    (0xB455, "PCIE_LTSSM_B455", 0),
    (0xB480, "PCIE_PERST_CTRL", 0),
    (0xB481, "PCIE_LINK_CTRL_B481", 0),
    (0xB482, "TUNNEL_ADAPTER_MODE", 0),
    (0xB4AE, "PCIE_LINK_STATUS_ALT(lo)", 0),
    (0xB4AF, "PCIE_LINK_STATUS_ALT(hi)", 0),
    (0xB4C8, "PCIE_LANE_MASK", 0),
    # Tunnel adapter config
    (0xB410, "TUNNEL_CFG_A_LO", 0),
    (0xB411, "TUNNEL_CFG_A_HI", 0),
    (0xB412, "TUNNEL_CREDITS", 0),
    (0xB413, "TUNNEL_CFG_MODE", 0),
    (0xB415, "TUNNEL_CAP_0", 0),
    (0xB41A, "TUNNEL_LINK_CFG_LO", 0),
    (0xB41B, "TUNNEL_LINK_CFG_HI", 0),
    # PHY / power
    (0xC20E, "PHY_RXPLL_RESET", 0),
    (0xC233, "PHY_CONFIG", 0),
    (0xC62D, "PHY_EXT_2D", 0),
    (0xC655, "PHY_CFG_C655", 0),
    (0xC656, "HDDPC_CTRL (3.3V)", 0),
    (0xC659, "PCIE_LANE_CTRL_C659", 0),
    (0xC65A, "PHY_CFG_C65A", 0),
    (0xC65B, "PHY_EXT_5B", 0),
    (0xC6A8, "PHY_CFG_C6A8", 0),
    # CPU mode / control
    (0xCA06, "CPU_MODE_NEXT", 0),
    (0xCA81, "CPU_CTRL_CA81", 0),
    (0xCC37, "CPU_CTRL_CC37", 0),
    # System status / link control
    (0xE324, "LINK_CTRL_E324", 0),
    (0xE710, "LINK_WIDTH_E710", 0),
    (0xE716, "LINK_STATUS_E716", 0),
    (0xE717, "LINK_CTRL_E717", 0),
    (0xE760, "PHY_RXPLL_CFG_A", 0),
    (0xE761, "PHY_RXPLL_CFG_B", 0),
    (0xE762, "PHY_RXPLL_STATUS", 0),
    (0xE763, "PHY_RXPLL_TRIGGER", 0),
    (0xE764, "PHY_TIMER_CTRL_E764", 0),
    (0xE765, "SYS_CTRL_E765", 0),
    (0xE7E3, "PHY_LINK_CTRL", 0),
    # bank1 (DPX=1) switch/PHY regs
    (0x4084, "PHY_PORT0_CFG", 1),
    (0x4092, "PHY_PCIE_LINK_INFO", 1),
    (0x5084, "PHY_PORT1_CFG", 1),
    (0x6025, "PHY_TLP_ROUTING", 1),
    (0x6041, "PHY_ISOLATION", 1),
    (0x6043, "PHY_TLP_CONFIG", 1),
    (0x78AF, "lane0 commit", 1),
    (0x79AF, "lane1 commit", 1),
    (0x7AAF, "lane2 commit", 1),
    (0x7BAF, "lane3 commit", 1),
]


def read_reg(p, addr, dpx):
    return p.read_dpx(addr) if dpx else p.read(addr)


def write_reg(p, addr, val, dpx):
    if dpx:
        p.write_dpx(addr, val)
    else:
        p.write(addr, val)


def cmd_dump(p):
    for addr, name, dpx in REGS:
        v = read_reg(p, addr, dpx)
        print(f"{'DPX:' if dpx else ''}0x{addr:04X} = 0x{v:02X}  {name}")


def classify(init, zero, ones, final):
    """Return per-bit classification string, 8 chars for bits 7..0."""
    out = []
    for b in range(7, -1, -1):
        m = 1 << b
        i = bool(init & m)
        z = bool(zero & m)
        o = bool(ones & m)
        f = bool(final & m)
        if z == o == i:
            ch = '1' if i else '0'          # stuck
        elif not z and o:
            ch = 'W' if f == i else 'w'     # writable (w = self-clearing)
        elif z and not o:
            ch = 'C'                        # write-1-clear-ish (reads 1, W1C to 0)
        else:
            ch = '?'                        # weird (0->1 but 1->0?)
        if f != i:
            ch = ch.lower() if ch.isupper() else ch + 'v'
        out.append(ch)
    return ''.join(out)


def cmd_bits(p, addrs, dpx):
    for addr in addrs:
        init = read_reg(p, addr, dpx)
        write_reg(p, addr, 0x00, dpx)
        zero = read_reg(p, addr, dpx)
        write_reg(p, addr, 0xFF, dpx)
        ones = read_reg(p, addr, dpx)
        write_reg(p, addr, init, dpx)       # restore
        final = read_reg(p, addr, dpx)
        cls = classify(init, zero, ones, final)
        print(f"{'DPX:' if dpx else ''}0x{addr:04X}: init={init:02X} w00->{zero:02X} wFF->{ones:02X} restore->{final:02X}  [{cls}]")
    print("legend per bit 7..0: 0/1=stuck, W=writable, w=self-clearing, C=W1C-ish, ?=weird, lowercase/v=changed after restore")


def cmd_bits1(p, addrs, dpx):
    """Safer per-bit probe: for each bit, write init^bit, read back, restore.
    Reports: W=bit toggles & retains, w=toggles but self-reverts after restore-write,
    .=no effect (RO), X=read changed after restore (volatile), H=hang.
    addrs entries may be 'ADDR:mask' to limit tested bits (mask hex)."""
    for spec in addrs:
        if isinstance(spec, str) and ':' in spec:
            astr, mstr = spec.split(':', 1)
            addr, bitmask = int(astr, 0), int(mstr, 0)
        else:
            addr, bitmask = spec, 0xFF
        init = read_reg(p, addr, dpx)
        res = []
        for b in range(7, -1, -1):
            m = 1 << b
            if not (bitmask & m):
                res.append(' ')
                continue
            try:
                write_reg(p, addr, init ^ m, dpx)
                v = read_reg(p, addr, dpx)
                write_reg(p, addr, init, dpx)
                back = read_reg(p, addr, dpx)
            except TimeoutError:
                res.append('H')
                print(f"{'DPX:' if dpx else ''}0x{addr:04X}: init={init:02X}  bits7..0 [{''.join(res)}]  *** HANG on bit {b} (write 0x{(init^m):02X}), proxy dead — reset device ***")
                return
            if v == (init ^ m):
                ch = 'W' if back == init else 'w'
            elif v == init:
                ch = '.' if back == init else 'X'
            else:
                ch = '?'
            res.append(ch)
        print(f"{'DPX:' if dpx else ''}0x{addr:04X}: init={init:02X}  bits7..0 [{''.join(res)}]")
    print("legend per bit 7..0: W=RW retains, w=toggles but reverts, .=RO/no-effect, X=volatile, H=hangs CPU, ?=weird")


def cmd_ltssm(p, seconds):
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < seconds:
        b450 = p.read(0xB450)
        e765 = p.read(0xE765)
        info = p.read_dpx(0x4092)
        b480 = p.read(0xB480)
        e764 = p.read(0xE764)
        state = (b450, e765, info, b480, e764)
        if state != last:
            dt = time.monotonic() - t0
            print(f"{dt:7.2f}s LTSSM=0x{b450:02X} E765=0x{e765:02X} linkinfo=0x{info:02X} B480=0x{b480:02X} E764=0x{e764:02X}")
            last = state
        time.sleep(0.02)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd = sys.argv[1]
    with UARTProxy() as p:
        if cmd == "dump":
            cmd_dump(p)
        elif cmd in ("bits", "bits1"):
            dpx = 0
            addrs = []
            for a in sys.argv[2:]:
                if a == "dpx":
                    dpx = 1
                elif cmd == "bits1":
                    addrs.append(a if ':' in a else int(a, 0))
                else:
                    addrs.append(int(a, 0))
            (cmd_bits if cmd == "bits" else cmd_bits1)(p, addrs, dpx)
        elif cmd == "ltssm":
            cmd_ltssm(p, float(sys.argv[2]) if len(sys.argv) > 2 else 10.0)
        else:
            print(__doc__)


if __name__ == "__main__":
    main()
