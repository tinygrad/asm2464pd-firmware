#!/usr/bin/env python3
"""
Pytest configuration for ASM2464PD firmware tests.

Provides fixtures and command-line options for testing against different firmwares:
- Original firmware (fw.bin) - the reference implementation
- Our compiled firmware (build/firmware.bin) - what we're building

Usage:
    # Run tests against original firmware only (default)
    pytest test/

    # Run tests against our compiled firmware only
    pytest test/ --firmware=ours

    # Run tests against both firmwares (compare behavior)
    pytest test/ --firmware=both

    # Run tests against a specific firmware file
    pytest test/ --firmware-path=/path/to/firmware.bin
"""

import sys
from pathlib import Path
import pytest

# Add emulate directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'emulate'))

from emu import Emulator


# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
ORIGINAL_FIRMWARE = PROJECT_ROOT / 'fw.bin'
OUR_FIRMWARE = PROJECT_ROOT / 'build' / 'firmware.bin'


def pytest_addoption(parser):
    """Add command-line options for firmware selection."""
    parser.addoption(
        "--firmware",
        action="store",
        default="original",
        choices=["original", "ours", "both"],
        help="Which firmware to test: original (fw.bin), ours (build/firmware.bin), or both"
    )
    parser.addoption(
        "--firmware-path",
        action="store",
        default=None,
        help="Path to a specific firmware file to test"
    )


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "firmware_test: mark test as requiring firmware"
    )
    config.addinivalue_line(
        "markers", "original_only: mark test as only valid for original firmware"
    )
    config.addinivalue_line(
        "markers", "ours_only: mark test as only valid for our firmware"
    )


def get_firmware_paths(config):
    """Get list of firmware paths to test based on config."""
    custom_path = config.getoption("--firmware-path")
    if custom_path:
        path = Path(custom_path)
        if path.exists():
            return [(path, "custom")]
        pytest.skip(f"Custom firmware not found: {custom_path}")

    firmware_option = config.getoption("--firmware")

    paths = []
    if firmware_option in ("original", "both"):
        if ORIGINAL_FIRMWARE.exists():
            paths.append((ORIGINAL_FIRMWARE, "original"))
    if firmware_option in ("ours", "both"):
        if OUR_FIRMWARE.exists():
            paths.append((OUR_FIRMWARE, "ours"))

    return paths


def pytest_generate_tests(metafunc):
    """Parametrize tests that use firmware fixtures."""
    if "firmware_path" in metafunc.fixturenames:
        paths = get_firmware_paths(metafunc.config)
        if not paths:
            # No firmware available, tests will be skipped
            metafunc.parametrize("firmware_path,firmware_name", [(None, "none")])
        else:
            metafunc.parametrize(
                "firmware_path,firmware_name",
                paths,
                ids=[name for _, name in paths]
            )


@pytest.fixture
def firmware_emulator(firmware_path, firmware_name):
    """
    Create an emulator loaded with the specified firmware.

    This fixture is parametrized by pytest_generate_tests based on --firmware option.

    Returns:
        tuple: (Emulator instance, firmware_name string)
    """
    if firmware_path is None:
        pytest.skip("No firmware available")

    emu = Emulator(log_uart=False, usb_delay=1000)
    emu.load_firmware(str(firmware_path))
    emu.reset()
    return emu, firmware_name


@pytest.fixture
def original_firmware_emulator():
    """
    Create an emulator loaded with the original firmware only.

    Use this for tests that should only run against the original firmware.
    """
    if not ORIGINAL_FIRMWARE.exists():
        pytest.skip("Original firmware (fw.bin) not found")

    emu = Emulator(log_uart=False, usb_delay=1000)
    emu.load_firmware(str(ORIGINAL_FIRMWARE))
    emu.reset()
    return emu


@pytest.fixture
def our_firmware_emulator():
    """
    Create an emulator loaded with our compiled firmware only.

    Use this for tests that should only run against our firmware.
    """
    if not OUR_FIRMWARE.exists():
        pytest.skip("Our firmware (build/firmware.bin) not found")

    emu = Emulator(log_uart=False, usb_delay=1000)
    emu.load_firmware(str(OUR_FIRMWARE))
    emu.reset()
    return emu


@pytest.fixture
def emulator():
    """
    Create a bare emulator without firmware loaded.

    Use this for tests that don't need firmware or will load it themselves.
    """
    emu = Emulator(log_uart=False)
    emu.reset()
    return emu


# Helper functions for tests
def firmware_exists(which="original"):
    """Check if a firmware file exists."""
    if which == "original":
        return ORIGINAL_FIRMWARE.exists()
    elif which == "ours":
        return OUR_FIRMWARE.exists()
    elif which == "both":
        return ORIGINAL_FIRMWARE.exists() and OUR_FIRMWARE.exists()
    return False


def skip_if_no_firmware(which="original"):
    """Return a pytest skip decorator if firmware doesn't exist."""
    if which == "original":
        return pytest.mark.skipif(
            not ORIGINAL_FIRMWARE.exists(),
            reason="Original firmware (fw.bin) not found"
        )
    elif which == "ours":
        return pytest.mark.skipif(
            not OUR_FIRMWARE.exists(),
            reason="Our firmware (build/firmware.bin) not found"
        )
    elif which == "both":
        return pytest.mark.skipif(
            not (ORIGINAL_FIRMWARE.exists() and OUR_FIRMWARE.exists()),
            reason="Both firmwares required"
        )
