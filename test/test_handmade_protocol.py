import re
from pathlib import Path

import pytest

from emu import Emulator


FIRMWARE = Path(__file__).parent.parent / "handmade" / "build" / "firmware.bin"
LISTING = FIRMWARE.parent / "obj" / "main.rst"
CONTROL_REGS = (0xB480, 0xC656, 0xE764, 0xC659, 0xB455, *range(0xB264, 0xB268), *range(0xB26C, 0xB270), 0xB250, 0xB251)
DMA_APERTURES = {
    0xB264: 0x08, 0xB265: 0x00, 0xB266: 0x08, 0xB267: 0x08,
    0xB26C: 0x08, 0xB26D: 0x20, 0xB26E: 0x08, 0xB26F: 0x28,
    0xB250: 0x00, 0xB251: 0x00,
}
BRIDGE_CONFIG = {
    0xB410: 0x1B, 0xB411: 0x21, 0xB412: 0x24, 0xB413: 0x63,
    0xB415: 0x06, 0xB416: 0x04, 0xB417: 0x00, 0xB418: 0x24,
    0xB419: 0x63, 0xB41A: 0x1B, 0xB41B: 0x21,
    0xB420: 0x1B, 0xB421: 0x21, 0xB422: 0x24, 0xB423: 0x63,
    0xB425: 0x06, 0xB426: 0x04, 0xB427: 0x00, 0xB428: 0x24,
    0xB429: 0x63, 0xB42A: 0x1B, 0xB42B: 0x21,
}
BANK1_ROUTING = {0x4084: 0x22, 0x5084: 0x22, 0x6043: 0x70, 0x6025: 0x80}


@pytest.fixture
def handmade_emulator():
    if not FIRMWARE.exists(): pytest.skip("build handmade firmware first with `make -C handmade wrapped`")
    emu = Emulator(log_uart=False, usb_delay=1000)
    emu.load_firmware(str(FIRMWARE))
    emu.reset()
    emulate_forwarding_commit(emu)
    emu.hw.usb_controller.connect(speed=2)
    emu.run(max_cycles=100000)
    return emu


@pytest.fixture
def handmade_usb3_emulator():
    if not FIRMWARE.exists(): pytest.skip("build handmade firmware first with `make -C handmade wrapped`")
    emu = Emulator(log_uart=False, usb_delay=1000)
    emu.load_firmware(str(FIRMWARE))
    emu.reset()
    emulate_forwarding_commit(emu)
    # The emulator's C startup clears low XDATA, while a hardware CPU reset retains it.
    for addr, value in zip(range(0xBC3, 0xBC7), bytes.fromhex("a55aa55a")):
        emu.memory.xdata_read_hooks[addr] = lambda _, value=value: value
    emu.hw.regs[0xB450] = 0x78
    emu.hw.usb_controller.connect(speed=2)
    emu.run(max_cycles=500000)
    return emu


def emulate_forwarding_commit(emu):
    def write_b455(hw, addr, value):
        hw.regs[addr] = 0x02 if value == 0x04 else value
    emu.hw.write_callbacks[0xB455] = write_b455


def queue_handmade_control(emu, request_type, request, value=0, index=0, length=0):
    emu.hw.usb_controller.inject_control_transfer(request_type, request, value, index, length)
    # The generic injector models the stock handler. Handmade firmware dispatches EP0 on 9101.1.
    emu.hw.regs[0x9101] = 0x02
    emu.hw.regs[0xC802] = 0x01
    emu.hw.regs[0x9091] = 0x01


def run_control(emu, cycles=100000): emu.run(max_cycles=emu.cpu.cycles + cycles)


def listing_symbol(name):
    match = re.search(rf"^\s*([0-9A-F]{{6}})\s+\d+\s+_{name}:\s*$", LISTING.read_text(), re.MULTILINE)
    assert match is not None, name
    return int(match.group(1), 16)


def finish_usb3_init(emu):
    # The generic emulator has no INA231 model; complete its one startup I2C transaction.
    emu.hw.read_callbacks[0xC875] = lambda _hw, _addr: 0x40
    run_control(emu)


def test_cold_boot_commits_gpu_post_train_state(handmade_usb3_emulator):
    emu = handmade_usb3_emulator
    assert {addr: emu.hw.regs.get(addr, 0) for addr in DMA_APERTURES} == DMA_APERTURES
    assert {addr: emu.hw.regs.get(addr, 0) for addr in BRIDGE_CONFIG} == BRIDGE_CONFIG
    assert {addr: emu.memory.xdata[addr] for addr in BANK1_ROUTING} == BANK1_ROUTING
    assert emu.hw.regs[0xB481] & 0x03 == 0x03
    assert emu.hw.regs[0xB2D5] == 0x01
    assert emu.hw.regs[0xC428] & 0x20
    assert emu.hw.regs[0xC450] & 0x04
    assert emu.hw.regs[0xC4EB] & 0x01
    assert emu.hw.regs[0xC4ED] & 0x01


def record_control_writes(emu):
    events = []
    for addr in CONTROL_REGS:
        original = emu.hw.write_callbacks.get(addr)
        def record(hw, written_addr, value, original=original):
            events.append((written_addr, value))
            if original is not None: original(hw, written_addr, value)
            else: hw.regs[written_addr] = value
        emu.hw.write_callbacks[addr] = record
    return events


def test_f3_power_on_commits_lane_control_after_training(handmade_emulator):
    emu = handmade_emulator
    queue_handmade_control(emu, 0x40, 0xF3, value=1)
    emu.hw.regs[0xB450] = 0x78
    emu.hw.regs[0xC659] = 0
    events = record_control_writes(emu)
    run_control(emu)

    assert events[:5] == [(0xB480, 0x01), (0xC656, 0x20), (0xE764, 0x1C), (0xB480, 0x00), (0xC659, 0x01)]
    assert events[5:15] == list(DMA_APERTURES.items())
    assert [value for addr, value in events if addr == 0xB455] == [0x02, 0x04, 0x02]
    assert {addr: emu.hw.regs.get(addr, 0) for addr in DMA_APERTURES} == DMA_APERTURES


def test_f3_power_on_repairs_reduced_dma_apertures(handmade_emulator):
    emu = handmade_emulator
    emu.hw.regs.update({0xB267: 0x02, 0xB26F: 0x22, 0xB450: 0x78, 0xC659: 0})
    queue_handmade_control(emu, 0x40, 0xF3, value=1)
    run_control(emu)

    assert emu.hw.regs[0xB267] == 0x08
    assert emu.hw.regs[0xB26F] == 0x28


def test_f3_power_off_asserts_perst_before_removing_rails(handmade_emulator):
    emu = handmade_emulator
    queue_handmade_control(emu, 0x40, 0xF3, value=0)
    emu.hw.regs.update({0xB480: 0x00, 0xC656: 0x20, 0xE764: 0x1C, 0xC659: 0x01})
    events = record_control_writes(emu)
    run_control(emu)

    assert events[:4] == [(0xB480, 0x01), (0xE764, 0x00), (0xC659, 0x00), (0xC656, 0x00)]


def test_f5_rotates_sram_slots_on_each_dma_active_to_idle_transition(handmade_usb3_emulator):
    emu = handmade_usb3_emulator
    finish_usb3_init(emu)
    armed_slots = []
    dma_status = {"value": 0}

    def record_slot(hw, addr, value):
        armed_slots.append(value)
        hw.regs[addr] = value

    def arm_dma(hw, addr, value):
        hw.regs[addr] = value
        dma_status["value"] = 0x02

    emu.hw.write_callbacks[0xC429] = record_slot
    emu.hw.write_callbacks[0xC412] = arm_dma
    emu.hw.read_callbacks[0xC450] = lambda _hw, _addr: dma_status["value"]

    queue_handmade_control(emu, 0x40, 0xF5, value=4, index=0x070B)
    run_control(emu)

    assert armed_slots == [11]
    assert (emu.hw.regs[0xC426], emu.hw.regs[0xC427]) == (0, 224)
    assert (emu.hw.regs[0xC414], emu.hw.regs[0xC415]) == (0x80 | 11, 18)

    for expected in (18, 25, 11):
        dma_status["value"] = 0x00
        run_control(emu, cycles=10000)
        assert armed_slots[-1] == expected

    dma_status["value"] = 0x00
    run_control(emu, cycles=10000)
    assert armed_slots == [11, 18, 25, 11]


def test_usb4_fallback_idle_sleep_is_interrupted_by_f5_state():
    emu = Emulator(log_uart=False)
    emu.load_firmware(str(FIRMWARE))
    emu.reset()
    emu.memory.xdata[listing_symbol("sram_stream_remaining")] = 1
    emu.cpu.pc = listing_symbol("sleep")
    emu.cpu.DPTR = 500
    emu.cpu.push(0x34)
    emu.cpu.push(0x12)
    emu.cpu.breakpoints.add(0x1234)

    assert emu.run(max_instructions=100) == "breakpoint"


def test_f2_cancels_an_unfinished_f5_stream(handmade_usb3_emulator):
    emu = handmade_usb3_emulator
    finish_usb3_init(emu)
    armed_slots = []
    bulk_status = {"value": 0}
    dma_status = {"value": 0x02}

    def record_slot(hw, addr, value):
        armed_slots.append(value)
        hw.regs[addr] = value

    emu.hw.write_callbacks[0xC429] = record_slot
    emu.hw.read_callbacks[0x9093] = lambda _hw, _addr: bulk_status["value"]
    emu.hw.write_callbacks[0x9093] = lambda _hw, _addr, value: bulk_status.__setitem__("value", bulk_status["value"] & ~value)
    emu.hw.read_callbacks[0xC450] = lambda _hw, _addr: dma_status["value"]

    queue_handmade_control(emu, 0x40, 0xF5, value=4, index=0x070B)
    run_control(emu)
    queue_handmade_control(emu, 0x40, 0xF2, value=224, index=0x0703)
    run_control(emu)
    assert armed_slots == [11, 3]

    bulk_status["value"] = 0x02
    emu.hw.regs.update({0xC802: 0x01, 0x9101: 0x04, 0x910D: 0xC0, 0x910E: 0x00})
    emu.cpu._ext0_pending = True
    run_control(emu, cycles=10000)
    assert armed_slots == [11, 3]
