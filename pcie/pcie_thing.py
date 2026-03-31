#!/usr/bin/env python3
"""Minimal PCIe bringup for ASM2464PD.

Includes critical PHY/link controller initialization that the original
minimal script was missing. Uses register polling instead of blind sleeps.
"""

import ctypes, sys, time
from tinygrad.runtime.support.usb import USB3
from tinygrad.runtime.autogen import libusb

SUPPORTED_DEVICES = [(0xADD1, 0x0001), (0x174C, 0x2463), (0x174C, 0x2464)]

class ASM2464PD:
    def __init__(self):
        self.dev = None
        self._wc = 0
        self._rc = 0
    def open(self):
        for vid, pid in SUPPORTED_DEVICES:
            try:
                self.dev = USB3(vid, pid, 0x81, 0x83, 0x02, 0x04, use_bot=True)
                print(f"Opened {vid:04X}:{pid:04X}")
                return
            except RuntimeError: pass
        raise RuntimeError("No device found")
    def close(self):
        if self.dev:
            libusb.libusb_release_interface(self.dev.handle, 0)
            libusb.libusb_close(self.dev.handle)
    def read8(self, addr):
        self._rc += 1
        buf = (ctypes.c_ubyte * 1)()
        ret = libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 0, buf, 1, 1000)
        assert ret >= 0, f"read 0x{addr:04X} failed: {ret}"
        return buf[0]
    def write(self, addr, val):
        self._wc += 1
        ret = libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val, None, 0, 1000)
        assert ret >= 0, f"write 0x{addr:04X}=0x{val:02X} failed: {ret}"
    def set_bits(self, addr, mask):
        self.write(addr, self.read8(addr) | mask)
    def clear_bits(self, addr, mask):
        self.write(addr, self.read8(addr) & ~mask & 0xFF)
    def rmw(self, addr, clear_mask, set_mask):
        val = self.read8(addr)
        self.write(addr, (val & ~clear_mask | set_mask) & 0xFF)
    def bank1_read8(self, addr):
        self._rc += 1
        buf = (ctypes.c_ubyte * 1)()
        libusb.libusb_control_transfer(self.dev.handle, 0xC0, 0xE4, addr, 1 << 8, buf, 1, 1000)
        return buf[0]
    def bank1_write(self, addr, val):
        self._wc += 1
        libusb.libusb_control_transfer(self.dev.handle, 0x40, 0xE5, addr, val | (1 << 8), None, 0, 1000)
    def bank1_or_bits(self, addr, mask):
        old = self.bank1_read8(addr)
        self.bank1_write(addr, old | mask)


def poll(dev, addr, expected, timeout_s=5.0, mask=0xFF, name=None):
    """Poll register until (value & mask) == expected. Returns value. Raises on timeout."""
    label = name or f"0x{addr:04X}"
    t0 = time.monotonic()
    i = 0
    while True:
        v = dev.read8(addr)
        if (v & mask) == expected:
            print(f"  {label}: {(time.monotonic()-t0)*1000:.1f}ms ({i+1} polls, 0x{v:02X})")
            return v
        i += 1
        if time.monotonic() - t0 > timeout_s:
            break
    raise RuntimeError(f"{label} timeout after {time.monotonic()-t0:.3f}s (0x{addr:04X}=0x{dev.read8(addr):02X}, wanted 0x{expected:02X} mask 0x{mask:02X})")


def poll_any(dev, addr, mask, timeout_s=2.0, name=None):
    """Poll register until (value & mask) != 0. Returns value."""
    label = name or f"0x{addr:04X}"
    t0 = time.monotonic()
    i = 0
    while True:
        v = dev.read8(addr)
        if v & mask:
            print(f"  {label}: {(time.monotonic()-t0)*1000:.1f}ms ({i+1} polls, 0x{v:02X})")
            return v
        i += 1
        if time.monotonic() - t0 > timeout_s:
            break
    print(f"  {label}: timeout ({time.monotonic()-t0:.3f}s, last=0x{v:02X})")
    return v


def hw_timer_wait(dev, threshold_hi, threshold_lo, mode, timeout_s=2.0):
    """Program hardware Timer0 and poll CC11 bit 1 for expiry."""
    dev.write(0xCC11, 0x04)  # clear
    dev.write(0xCC11, 0x02)  # clear done
    div = dev.read8(0xCC10)
    dev.write(0xCC10, (div & 0xF8) | (mode & 0x07))
    dev.write(0xCC12, threshold_hi)
    dev.write(0xCC13, threshold_lo)
    dev.write(0xCC11, 0x01)  # start
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if dev.read8(0xCC11) & 0x02:
            break
    dev.write(0xCC11, 0x02)  # clear done


def phy_poll_ready(dev, timeout_s=2.0, name="E712 PHY ready"):
    """Poll E712 bits 0|1 for PHY PLL ready."""
    return poll_any(dev, 0xE712, 0x03, timeout_s=timeout_s, name=name)


def print_pcie_status(dev):
    """Read and display key PCIe status registers."""
    print("\n=== PCIe Status ===")
    ltssm = dev.read8(0xB450)
    b22b = dev.read8(0xB22B)
    b434 = dev.read8(0xB434)
    b455 = dev.read8(0xB455)
    e710 = dev.read8(0xE710)
    cc30 = dev.read8(0xCC30)
    ca06 = dev.read8(0xCA06)
    b480 = dev.read8(0xB480)
    c659 = dev.read8(0xC659)
    link_up = ltssm in (0x48, 0x78)
    print(f"  LTSSM state (B450): 0x{ltssm:02X}  {'(L0!)' if link_up else '(Detect)' if ltssm == 0x00 else ''}")
    print(f"  Link width  (B22B): 0x{b22b:02X}  {'(x4)' if b22b == 0x04 else ''}")
    print(f"  Lane enable (B434): 0x{b434:02X}")
    print(f"  Link detect (B455): 0x{b455:02X}")
    print(f"  Link width  (E710): 0x{e710:02X}")
    print(f"  CPU mode    (CC30): 0x{cc30:02X}")
    print(f"  CPU next    (CA06): 0x{ca06:02X}")
    print(f"  PERST ctrl  (B480): 0x{b480:02X}  {'(asserted)' if b480 & 0x01 else '(deasserted)'}")
    print(f"  12V enable  (C659): 0x{c659:02X}  {'(on)' if c659 & 0x01 else '(off)'}")
    print(f"  Link status: {'UP' if link_up else 'DOWN'}")


def pcie_link_controller_init(dev):
    """PCIe link controller config (from pcie4_main.c:2387)."""
    print("  [link_controller_init]")
    # CC32 = 0x01 (gate enable)
    dev.write(0xCC32, 0x01)
    # E710 = (E710 & 0xE0) | 0x04
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | 0x04)
    # C6A8: set bit 0
    dev.set_bits(0xC6A8, 0x01)
    # CC33 = 0x04
    dev.write(0xCC33, 0x04)
    # E324: clear bit 2
    dev.clear_bits(0xE324, 0x04)
    # CC3B: clear bit 0
    dev.clear_bits(0xCC3B, 0x01)
    # E717: set bit 0
    dev.set_bits(0xE717, 0x01)
    # CC3E: clear bit 1
    dev.clear_bits(0xCC3E, 0x02)
    # CC3B: clear bit 1, then clear bit 6
    dev.clear_bits(0xCC3B, 0x02)
    dev.clear_bits(0xCC3B, 0x40)
    # Restore CC3B bits 0,1
    dev.set_bits(0xCC3B, 0x03)
    # E716 = (E716 & 0xFC) | 0x03
    e716 = dev.read8(0xE716)
    dev.write(0xE716, (e716 & 0xFC) | 0x03)
    # CC3E: clear bit 0
    dev.clear_bits(0xCC3E, 0x01)
    # CC39: set bit 1
    dev.set_bits(0xCC39, 0x02)
    # CC3A: clear bit 1; CC38: clear bit 1
    dev.clear_bits(0xCC3A, 0x02)
    dev.clear_bits(0xCC38, 0x02)
    # CA06 = (CA06 & 0x1F) | 0x60  (Gen3/Gen4 mode)
    ca06 = dev.read8(0xCA06)
    dev.write(0xCA06, (ca06 & 0x1F) | 0x60)
    # CA81: set bit 0
    dev.set_bits(0xCA81, 0x01)


def phy_soft_reset(dev):
    """PHY soft reset via RXPLL mode toggle, polls E712 for completion."""
    print("  [phy_soft_reset]")
    # CC37 |= 0x04 — enable RXPLL reset mode
    dev.set_bits(0xCC37, 0x04)
    # CA70 = 0x00, E780 = 0x00
    dev.write(0xCA70, 0x00)
    dev.write(0xE780, 0x00)
    # E716 = 0x00, then 0x03 (reset and restore PCIe mode)
    dev.write(0xE716, 0x00)
    dev.write(0xE716, 0x03)
    # Poll E712 for PHY PLL relock
    phy_poll_ready(dev, timeout_s=2.0, name="E712 PHY relock")
    # CC37 &= ~0x04
    dev.clear_bits(0xCC37, 0x04)


def phy_config_toggle(dev):
    """C233 PHY config toggle + E712 ready poll (from pcie_pre_init)."""
    # C233 &= 0xFC (clear mode bits)
    dev.clear_bits(0xC233, 0x03)
    # C233 |= 0x04 (set toggle bit)
    c233 = dev.read8(0xC233)
    dev.write(0xC233, (c233 & 0xFB) | 0x04)
    # Short hardware timer for settling (threshold=0x14=20 ticks, mode=2)
    hw_timer_wait(dev, 0x00, 0x14, 0x02)
    # C233 &= ~0x04 (clear toggle bit)
    dev.clear_bits(0xC233, 0x04)
    # Poll E712 for PHY ready after config change
    phy_poll_ready(dev, timeout_s=2.0, name="E712 PHY config ready")
    # E7E3 = 0x00
    dev.write(0xE7E3, 0x00)


def pcie_lane_config(dev, mask=0x0F):
    """Progressive PCIe lane enable with E712 polling between steps."""
    print(f"  [lane_config mask=0x{mask:02X}]")
    # Save B402 bit 1 and clear it
    b402_saved = dev.read8(0xB402) & 0x02
    dev.clear_bits(0xB402, 0x02)

    if mask == 0x0F:
        for step in [0x01, 0x03, 0x07, 0x0F]:
            b434 = dev.read8(0xB434)
            dev.write(0xB434, step | (b434 & 0xF0))
            # Poll hardware timer for lane settling (threshold=0xC7=199 ticks, mode=2)
            hw_timer_wait(dev, 0x00, 0xC7, 0x02)
    else:
        current = dev.read8(0xB434) & 0x0F
        counter = 0x01
        for _ in range(4):
            if current == mask:
                break
            new_state = current & (mask | (counter ^ 0x0F))
            current = new_state
            b434 = dev.read8(0xB434)
            dev.write(0xB434, new_state | (b434 & 0xF0))
            hw_timer_wait(dev, 0x00, 0xC7, 0x02)
            counter = (counter << 1) & 0xFF

    # Restore B402 bit 1
    if b402_saved:
        dev.set_bits(0xB402, 0x02)

    # Configure B436
    b436 = dev.read8(0xB436)
    dev.write(0xB436, (b436 & 0xF0) | (mask & 0x0E))
    b404 = dev.read8(0xB404)
    upper = ((b404 & 0x0F) ^ 0x0F) << 4
    b436 = dev.read8(0xB436)
    dev.write(0xB436, (b436 & 0x0F) | (upper & 0xF0))


def pcie_early_init(dev):
    """PCIe PHY/controller init (from pcie4_main.c:2590)."""
    print("  [early_init]")
    dev.clear_bits(0xB402, 0x02)
    # E764 reset: clear bits individually, then set bit 2
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFD)  # clear bit 1
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFE)  # clear bit 0
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xF7)  # clear bit 3
    e764 = dev.read8(0xE764)
    dev.write(0xE764, (e764 & 0xFB) | 0x04)  # set bit 2
    # B432/B404 config
    b432 = dev.read8(0xB432)
    dev.write(0xB432, (b432 & 0xF8) | 0x07)
    b404 = dev.read8(0xB404)
    dev.write(0xB404, (b404 & 0xF0) | 0x01)
    # E76C/E774/E77C: clear bit 4
    dev.clear_bits(0xE76C, 0x10)
    dev.clear_bits(0xE774, 0x10)
    dev.clear_bits(0xE77C, 0x10)
    # Progressive lane config
    pcie_lane_config(dev, 0x0F)


def pcie_pll_init(dev):
    """PHY PLL and LTSSM config (from pcie4_main.c:2653)."""
    print("  [pll_init]")
    dev.clear_bits(0xCC35, 0x01)
    # E741 PLL config
    e741 = dev.read8(0xE741)
    dev.write(0xE741, (e741 & 0xF8) | 0x03)
    e741 = dev.read8(0xE741)
    dev.write(0xE741, (e741 & 0xC7) | 0x28)
    e742 = dev.read8(0xE742)
    dev.write(0xE742, (e742 & 0xFC) | 0x03)
    e741 = dev.read8(0xE741)
    dev.write(0xE741, (e741 & 0x3F) | 0x80)
    # CC43 = 0x80 (PCIe PLL clock)
    dev.write(0xCC43, 0x80)
    # Power/GPIO init
    dev.set_bits(0xC65B, 0x08)
    dev.clear_bits(0xC656, 0x20)
    dev.set_bits(0xC65B, 0x20)
    c62d = dev.read8(0xC62D)
    dev.write(0xC62D, (c62d & 0xE0) | 0x07)
    # Bus config
    dev.set_bits(0xC004, 0x02)
    dev.clear_bits(0xC007, 0x08)
    dev.set_bits(0xCA2E, 0x01)


def pcie_tunnel_setup(dev):
    """Configure PCIe tunnel adapter and bridge (from pcie4_main.c:2011)."""
    print("  [tunnel_setup]")
    dev.clear_bits(0xCA06, 0x10)
    # Bridge config: VID=0x1B21, DID=0x2463, class=0x060400
    dev.write(0xB410, 0x1B); dev.write(0xB411, 0x21)
    dev.write(0xB420, 0x1B); dev.write(0xB421, 0x21)
    dev.write(0xB412, 0x24); dev.write(0xB413, 0x63)
    dev.write(0xB422, 0x24); dev.write(0xB423, 0x63)
    dev.write(0xB415, 0x06); dev.write(0xB416, 0x04); dev.write(0xB417, 0x00)
    dev.write(0xB425, 0x06); dev.write(0xB426, 0x04); dev.write(0xB427, 0x00)
    dev.write(0xB41A, 0x1B); dev.write(0xB41B, 0x21)
    dev.write(0xB42A, 0x1B); dev.write(0xB42B, 0x21)
    dev.write(0xB418, 0x24); dev.write(0xB419, 0x63)
    dev.write(0xB428, 0x24); dev.write(0xB429, 0x63)
    # B401 |= 0x01
    dev.set_bits(0xB401, 0x01)
    # B482: set bit 0, then upper nibble 0xF0
    dev.set_bits(0xB482, 0x01)
    b482 = dev.read8(0xB482)
    dev.write(0xB482, (b482 & 0x0F) | 0xF0)
    # B401 clear bit 0, B480 assert PERST
    b401 = dev.read8(0xB401)
    dev.write(0xB401, b401 & 0xFE)
    dev.set_bits(0xB480, 0x01)
    # B430 clear bit 0
    dev.clear_bits(0xB430, 0x01)
    # B298 set bit 4 (tunnel enable)
    b298 = dev.read8(0xB298)
    dev.write(0xB298, (b298 & 0xEF) | 0x10)


def pcie_tunnel_enable(dev):
    """Full PCIe tunnel bringup (from pcie4_main.c:2725)."""
    print("[tunnel_enable]")
    # E710 re-init
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | 0x04)
    # Timer block init
    dev.write(0xCD31, 0x04); dev.write(0xCD31, 0x02)
    cd30 = dev.read8(0xCD30)
    dev.write(0xCD30, (cd30 & 0xF8) | 0x05)
    dev.write(0xCD32, 0x00); dev.write(0xCD33, 0xC7)
    cc2a = dev.read8(0xCC2A)
    dev.write(0xCC2A, (cc2a & 0xF8) | 0x04)
    dev.write(0xCC2C, 0xC7); dev.write(0xCC2D, 0xC7)
    # BFE0 wrapper
    dev.set_bits(0xC6A8, 0x01)
    p92c8 = dev.read8(0x92C8)
    dev.write(0x92C8, p92c8 & 0xFC)
    dev.write(0xCD31, 0x04); dev.write(0xCD31, 0x02)
    # Timer 1
    dev.write(0xCC17, 0x04); dev.write(0xCC17, 0x02)
    div = dev.read8(0xCC16)
    dev.write(0xCC16, (div & 0xF8) | 0x04)
    dev.write(0xCC18, 0x01); dev.write(0xCC19, 0x90)
    p92c4 = dev.read8(0x92C4)
    dev.write(0x92C4, p92c4 & 0xFE)
    dev.set_bits(0x9201, 0x01)
    dev.clear_bits(0x9201, 0x01)
    # Timer 3
    dev.write(0xCC23, 0x04); dev.write(0xCC23, 0x02)
    div = dev.read8(0xCC22)
    dev.write(0xCC22, (div & 0xE8) | 0x07)
    # Timer 2 + Timer 4
    dev.write(0xCC1D, 0x04); dev.write(0xCC1D, 0x02)
    dev.write(0xCC5D, 0x04); dev.write(0xCC5D, 0x02)
    div = dev.read8(0xCC1C)
    dev.write(0xCC1C, (div & 0xF8) | 0x06)
    dev.write(0xCC1E, 0x00); dev.write(0xCC1F, 0x8B)
    div = dev.read8(0xCC5C)
    dev.write(0xCC5C, (div & 0xF8) | 0x04)
    dev.write(0xCC5E, 0x00); dev.write(0xCC5F, 0xC7)
    # GPIO/power config
    dev.set_bits(0xC655, 0x01)
    c620 = dev.read8(0xC620)
    dev.write(0xC620, c620 & 0xE0)
    dev.set_bits(0xC65A, 0x01)
    # 3.3V power
    c656 = dev.read8(0xC656)
    if not (c656 & 0x20):
        dev.write(0xC656, (c656 & 0xDF) | 0x20)
        dev.set_bits(0xC65B, 0x20)
    # Tunnel setup
    pcie_tunnel_setup(dev)
    # DMA config
    dev.write(0xB264, 0x08); dev.write(0xB265, 0x00)
    dev.write(0xB266, 0x08); dev.write(0xB267, 0x08)
    dev.write(0xB26C, 0x08); dev.write(0xB26D, 0x20)
    dev.write(0xB26E, 0x08); dev.write(0xB26F, 0x28)
    dev.write(0xB250, 0x00); dev.write(0xB251, 0x00)
    # CEF3/CEF2/CEF0/CEEF
    dev.write(0xCEF3, 0x08); dev.write(0xCEF2, 0x80)
    dev.clear_bits(0xCEF0, 0x08)
    dev.clear_bits(0xCEEF, 0x80)
    # C807/B281
    dev.set_bits(0xC807, 0x04)
    b281 = dev.read8(0xB281)
    dev.write(0xB281, (b281 & 0xCF) | 0x10)
    # B401 pulse
    dev.set_bits(0xB401, 0x01)
    b401 = dev.read8(0xB401)
    dev.write(0xB401, b401 & 0xFE)
    # Full tunnel reinit (link controller + PHY reset + lane config)
    print("  [tunnel_reinit]")
    pcie_link_controller_init(dev)
    phy_soft_reset(dev)
    phy_config_toggle(dev)
    # Re-run early_init equivalent
    dev.clear_bits(0xB402, 0x02)
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFD)
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFE)
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xF7)
    e764 = dev.read8(0xE764)
    dev.write(0xE764, (e764 & 0xFB) | 0x04)
    b432 = dev.read8(0xB432)
    dev.write(0xB432, (b432 & 0xF8) | 0x07)
    b404 = dev.read8(0xB404)
    dev.write(0xB404, (b404 & 0xF0) | 0x01)
    dev.clear_bits(0xE76C, 0x10)
    dev.clear_bits(0xE774, 0x10)
    dev.clear_bits(0xE77C, 0x10)
    pcie_lane_config(dev, 0x0F)
    # CA06 clear bit 4, B480 assert PERST
    dev.clear_bits(0xCA06, 0x10)
    dev.set_bits(0xB480, 0x01)
    # 12V OFF during tunnel config
    dev.clear_bits(0xC659, 0x01)
    # Lane config
    pcie_lane_config(dev, 0x0F)
    print("[tunnel_enable done]")


def pcie_perst_deassert(dev):
    """PERST deassert with link training (from pcie4_main.c:3286)."""
    print("[perst_deassert]")
    # Pre-PERST
    dev.write(0xB455, 0x02); dev.write(0xB455, 0x04)
    dev.write(0xB2D5, 0x01)
    dev.write(0xB296, 0x08)
    # Deassert PERST#
    dev.clear_bits(0xB480, 0x01)
    print("  [PERST deasserted]")
    # Poll B455 bit 1 (link detect)
    detected = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        if dev.read8(0xB455) & 0x02:
            dev.write(0xB455, 0x02)
            detected = True
            break
    if detected:
        print(f"  [Link detected: {(time.monotonic()-t0)*1000:.1f}ms]")
    else:
        print("  [Link detect timeout]")
    # Post cleanup
    if dev.read8(0xB2D5) & 0x01:
        dev.write(0xB2D5, 0x01)
    dev.write(0xB296, 0x08)
    print("[perst_deassert done]")
    return detected


def pcie_phase2(dev):
    """Phase 2: Gen3 link retraining with E762 polling (from pcie4_main.c:2895)."""
    print("[phase2 — Gen3 retrain]")
    # E710 re-init
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | 0x04)
    # Wait for LTSSM to be stable before reconfiguring
    hw_timer_wait(dev, 0x01, 0x2B, 0x04)
    # CA81 clear, CA06 Gen3, B403 set
    dev.clear_bits(0xCA81, 0x01)
    ca06 = dev.read8(0xCA06)
    dev.write(0xCA06, (ca06 & 0x1F) | 0x20)
    dev.set_bits(0xB403, 0x01)
    # Lane mask 0x0C (lanes 2,3 for Gen3 x2)
    b431 = dev.read8(0xB431)
    dev.write(0xB431, (b431 & 0xF0) | 0x0C)
    pcie_lane_config(dev, 0x0C)
    # Save E710/CA06 and set up equalization
    saved_e710 = dev.read8(0xE710) & 0x1F
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | 0x1F)
    saved_ca06 = dev.read8(0xCA06) & 0xE0
    dev.write(0xE751, 0x01)
    # E764 training trigger sequence
    e764 = dev.read8(0xE764)
    dev.write(0xE764, (e764 & 0xF7) | 0x08)  # set bit 3
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFB)            # clear bit 2
    e764 = dev.read8(0xE764)
    dev.write(0xE764, e764 & 0xFE)            # clear bit 0
    e764 = dev.read8(0xE764)
    dev.write(0xE764, (e764 & 0xFD) | 0x02)   # set bit 1
    # Poll E762 bit 4 for link ok (instead of 2s timer_wait)
    print("  [polling E762 for PHY link...]")
    rxpll = poll_any(dev, 0xE762, 0x10, timeout_s=5.0, name="E762 RXPLL link ok")
    link_ok = bool(rxpll & 0x10)
    print(f"  RXPLL status: 0x{rxpll:02X}  link_ok={link_ok}")
    if link_ok:
        e764 = dev.read8(0xE764)
        dev.write(0xE764, (e764 & 0xFE) | 0x01)  # set bit 0
        e764 = dev.read8(0xE764)
        dev.write(0xE764, e764 & 0xFD)            # clear bit 1
    else:
        e764 = dev.read8(0xE764)
        dev.write(0xE764, e764 & 0xF7)
        e764 = dev.read8(0xE764)
        dev.write(0xE764, e764 & 0xFB)
        e764 = dev.read8(0xE764)
        dev.write(0xE764, e764 & 0xFE)
        e764 = dev.read8(0xE764)
        dev.write(0xE764, e764 & 0xFD)
    # Restore E710/CA06
    e710 = dev.read8(0xE710)
    dev.write(0xE710, (e710 & 0xE0) | saved_e710)
    ca06 = dev.read8(0xCA06)
    dev.write(0xCA06, (ca06 & 0x1F) | saved_ca06)
    dev.clear_bits(0xCA81, 0x01)
    # 12V ON
    dev.set_bits(0xC659, 0x01)
    # Poll LTSSM for L0 instead of blind timer wait
    print("  [polling B450 for L0...]")
    try:
        poll(dev, 0xB450, 0x78, timeout_s=5.0, name="B450 LTSSM L0 (phase2)")
    except RuntimeError:
        # Also accept 0x48
        ltssm = dev.read8(0xB450)
        if ltssm == 0x48:
            print(f"  B450 LTSSM L0 (phase2): 0x{ltssm:02X} (alt L0)")
        else:
            print(f"  B450 LTSSM: 0x{ltssm:02X} (not L0, continuing)")
    print(f"[phase2 done — link_ok={link_ok}]")
    return link_ok


def pcie_post_train(dev):
    """Post-link-training config (from pcie4_main.c:3494)."""
    print("[post_train]")
    # 12V ON
    dev.set_bits(0xC659, 0x01)
    # DMA config
    dev.write(0xB264, 0x08); dev.write(0xB265, 0x00)
    dev.write(0xB266, 0x08); dev.write(0xB267, 0x08)
    dev.write(0xB26C, 0x08); dev.write(0xB26D, 0x20)
    dev.write(0xB26E, 0x08); dev.write(0xB26F, 0x28)
    dev.write(0xB250, 0x00); dev.write(0xB251, 0x00)
    # NVMe registers
    dev.set_bits(0xC428, 0x20)
    dev.set_bits(0xC450, 0x04)
    dev.clear_bits(0xC472, 0x01)
    dev.set_bits(0xC4EB, 0x01)
    dev.set_bits(0xC4ED, 0x01)
    dev.clear_bits(0xCA06, 0x40)
    # Bridge config
    print("  [bridge_config]")
    dev.clear_bits(0xCA06, 0x10)
    dev.write(0xB410, 0x1B); dev.write(0xB411, 0x21)
    dev.write(0xB412, 0x24); dev.write(0xB413, 0x63)
    dev.write(0xB415, 0x06); dev.write(0xB416, 0x04); dev.write(0xB417, 0x00)
    dev.write(0xB418, 0x24); dev.write(0xB419, 0x63)
    dev.write(0xB41A, 0x1B); dev.write(0xB41B, 0x21)
    dev.write(0xB420, 0x1B); dev.write(0xB421, 0x21)
    dev.write(0xB422, 0x24); dev.write(0xB423, 0x63)
    dev.write(0xB425, 0x06); dev.write(0xB426, 0x04); dev.write(0xB427, 0x00)
    dev.write(0xB428, 0x24); dev.write(0xB429, 0x63)
    dev.write(0xB42A, 0x1B); dev.write(0xB42B, 0x21)
    dev.set_bits(0xB401, 0x01)
    dev.set_bits(0xB482, 0x01)
    b482 = dev.read8(0xB482)
    dev.write(0xB482, (b482 & 0x0F) | 0xF0)
    dev.clear_bits(0xB401, 0x01)
    dev.set_bits(0xB480, 0x01)
    dev.clear_bits(0xB430, 0x01)
    b298 = dev.read8(0xB298)
    dev.write(0xB298, (b298 & 0xEF) | 0x10)
    dev.clear_bits(0xCA06, 0x10)
    dev.set_bits(0xB480, 0x01)
    # Bank-switched PHY writes for TLP forwarding
    dev.bank1_write(0x4084, 0x22)
    dev.bank1_write(0x5084, 0x22)
    dev.bank1_write(0x6043, 0x70)
    dev.bank1_or_bits(0x6025, 0x80)
    # B481
    b481 = dev.read8(0xB481)
    dev.write(0xB481, (b481 & 0xFC) | 0x03)
    # B455 trigger for TLP forwarding
    dev.write(0xB455, 0x02); dev.write(0xB455, 0x04)
    dev.write(0xB2D5, 0x01); dev.write(0xB296, 0x08)
    # Poll B455 bit 1
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        if dev.read8(0xB455) & 0x02:
            dev.write(0xB455, 0x02)
            break
    print("[post_train done]")


def main():
    dev = ASM2464PD()
    dev.open()
    try:
        # verify firmware set B213
        assert dev.read8(0xB213) == 0x01, f"B213=0x{dev.read8(0xB213):02X}, firmware didn't set TLP_CTRL"

        print("--- Phase 1: Pre-Init ---")
        # Set CA06 Gen3/Gen4 and CA81 init trigger
        ca06 = dev.read8(0xCA06)
        dev.write(0xCA06, (ca06 & 0x1F) | 0x60)
        dev.set_bits(0xCA81, 0x01)
        e716 = dev.read8(0xE716)
        dev.write(0xE716, (e716 & 0xFC) | 0x03)

        # Link controller init + PHY soft reset + config toggle
        pcie_link_controller_init(dev)
        phy_soft_reset(dev)
        phy_config_toggle(dev)

        # Early init: E764 sequencing + progressive lane enable
        pcie_early_init(dev)

        # Clear CC32 gate
        dev.write(0xCC32, 0x00)

        print("\n--- Phase 2: PLL + Power ---")
        pcie_pll_init(dev)
        # 3.3V power
        c656 = dev.read8(0xC656)
        if not (c656 & 0x20):
            dev.write(0xC656, (c656 & 0xDF) | 0x20)
            dev.set_bits(0xC65B, 0x20)
        # 12V ON
        dev.set_bits(0xC659, 0x01)

        print("\n--- Phase 3: Tunnel Enable ---")
        pcie_tunnel_enable(dev)

        print("\n--- Phase 4: PERST Deassert ---")
        link_detected = pcie_perst_deassert(dev)

        print("\n--- Phase 5: Gen3 Retrain ---")
        link_ok = pcie_phase2(dev)

        print("\n--- Phase 6: Post-Train ---")
        b22b = dev.read8(0xB22B)
        if link_ok or b22b == 0x04:
            pcie_post_train(dev)
            print("\n*** PCIe link is UP! ***")
        else:
            print(f"\n*** PCIe link FAILED (link_ok={link_ok}, B22B=0x{b22b:02X}) ***")

        # Final status
        print(f"\nTotal: {dev._wc} writes, {dev._rc} reads")
        print_pcie_status(dev)
        sys.exit(0 if (link_ok or b22b == 0x04) else 1)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
