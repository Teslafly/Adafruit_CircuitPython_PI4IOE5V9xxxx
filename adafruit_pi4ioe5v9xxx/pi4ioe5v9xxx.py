# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2019 Carter Nelson
# SPDX-FileCopyrightText: 2021 Red_M
# SPDX-FileCopyrightText: 2024 Marshall Scholz
#
# SPDX-License-Identifier: MIT

"""
`pi4ioe5v9xxx`
====================================================

CircuitPython module for PI4IOE5V9xxx series of I2C I/O extenders.

* Author(s): Tony DiCola, Red_M (2021)
             Teslafly (2024)
"""

from adafruit_bus_device import i2c_device

try:
    from typing import Optional
    from busio import I2C
    import digitalio
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx.git"

# Global buffer for reading and writing registers with the devices.  This is
# shared between both the MCP23008 and MCP23017 class to reduce memory allocations.
# However this is explicitly not thread safe or re-entrant by design!
_BUFFER = bytearray(3)


# pylint: disable=too-few-public-methods
class PI4IOE5V9xxx:
    """Base class for PI4IOE5V9xxx devices."""

    def __init__(
        self,
        bus_device: I2C,
        address: int,
        chip_select: Optional[digitalio.DigitalInOut] = None,
        baudrate: int = 100000,
    ) -> None:
        self._device = i2c_device.I2CDevice(bus_device, address)

    def _read_u16le(self, register: int) -> int:
        # Read an unsigned 16 bit little endian value from the specified 8-bit
        # register.
        with self._device as bus_device:
            _BUFFER[0] = register & 0xFF

            bus_device.write_then_readinto(
                _BUFFER, _BUFFER, out_end=1, in_start=1, in_end=3
            )
            return (_BUFFER[2] << 8) | _BUFFER[1]

    def _write_u16le(self, register: int, val: int) -> None:
        # Write an unsigned 16 bit little endian value to the specified 8-bit
        # register.
        with self._device as bus_device:
            _BUFFER[0] = register & 0xFF
            _BUFFER[1] = val & 0xFF
            _BUFFER[2] = (val >> 8) & 0xFF
            bus_device.write(_BUFFER, end=3)

    def _read_u8(self, register: int) -> int:
        # Read an unsigned 8 bit value from the specified 8-bit register.
        with self._device as bus_device:
            _BUFFER[0] = register & 0xFF

            bus_device.write_then_readinto(
                _BUFFER, _BUFFER, out_end=1, in_start=1, in_end=2
            )
            return _BUFFER[1]

    def _write_u8(self, register: int, val: int) -> None:
        # Write an 8 bit value to the specified 8-bit register.
        with self._device as bus_device:
            _BUFFER[0] = register & 0xFF
            _BUFFER[1] = val & 0xFF
            bus_device.write(_BUFFER, end=2)
