# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2019 Carter Nelson
#
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2024 Marshall Scholz

"""
`digital_inout`
====================================================

Digital input/output of the PI4IOE5V9xxx.

* Author(s): Tony DiCola
"""

import digitalio

try:
    from typing import Optional
    from .pi4ioe5v9xxx import PI4IOE5V9xxx
    from digitalio import Pull, Direction
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx.git"


# Internal helpers to simplify setting and getting a bit inside an integer.
def _get_bit(val, bit: int) -> int:
    return val & (1 << bit) > 0


def _enable_bit(val, bit: int) -> int:
    return val | (1 << bit)


def _clear_bit(val, bit: int) -> int:
    return val & ~(1 << bit)


class DigitalInOut:
    """Digital input/output of the PI4IOE5V9xxx.  The interface is exactly the
    same as the digitalio.DigitalInOut class, however:

      * PI4IOE5V9xxx family also supports pin drive strengths.

    Exceptions will be thrown when attempting to do something the expander can not do.
    """

    def __init__(self, pin_number: int, pi4ioe5v9xxx: PI4IOE5V9xxx) -> None:
        """Specify the pin number of the PI4IOE5V9xxx (0...7 for PI4IOE5V9x08, or 0...15
        for PI4IOE5V6416) and PI4IOE5V9xxx instance.
        """
        self._pin = int(pin_number)
        self._exp = pi4ioe5v9xxx

    # kwargs in switch functions below are _necessary_ for compatibility
    # with DigitalInout class (which allows specifying pull, etc. which
    # is unused by this class).  Do not remove them, instead turn off pylint
    # in this case.
    # pylint: disable=unused-argument
    def switch_to_output(
        self, value: bool = False, drivestrength: int = 3, **kwargs
    ) -> None:
        """Switch the pin state to a digital output with the provided starting
        value (True/False for high or low, default is False/low).
        """
        self.direction = digitalio.Direction.OUTPUT
        self.value = value
        self.drivestrength = drivestrength

    def switch_to_input(
        self, pull: Pull = None, invert_polarity: bool = False, **kwargs
    ) -> None:
        """Switch the pin state to a digital input with the provided starting
        pull-up resistor state (optional, no pull-xx by default) and input polarity.
        """
        self.direction = digitalio.Direction.INPUT
        self.pull = pull
        self.invert_polarity = invert_polarity

    # pylint: enable=unused-argument

    @property
    def value(self) -> bool:
        """The value of the pin, either True for high or False for
        low.  Note you must configure as an output or input appropriately
        before reading and writing this value.
        """

        if self.direction == digitalio.Direction.INPUT:
            reg = self._exp.gpio_in
        else:
            reg = self._exp.gpio_out

        return _get_bit(reg, self._pin)

    @value.setter
    def value(self, val: bool) -> None:
        # only set the output as gpio_in is read only.

        if val:
            self._exp.gpio_out = _enable_bit(self._exp.gpio_out, self._pin)
        else:
            self._exp.gpio_out = _clear_bit(self._exp.gpio_out, self._pin)

    @property
    def direction(self) -> bool:
        """The direction of the pin, either True for an input or
        False for an output.
        """
        if _get_bit(self._exp.gpio_dir, self._pin):
            return digitalio.Direction.INPUT
        return digitalio.Direction.OUTPUT

    @direction.setter
    def direction(self, val: Direction) -> None:
        if val == digitalio.Direction.INPUT:
            self._exp.gpio_dir = _enable_bit(self._exp.gpio_dir, self._pin)
        elif val == digitalio.Direction.OUTPUT:
            self._exp.gpio_dir = _clear_bit(self._exp.gpio_dir, self._pin)
        else:
            raise ValueError("Expected INPUT or OUTPUT direction!")

    @property
    def pull(self) -> Optional[digitalio.Pull]:
        """Enable or disable internal pull-up resistors for this pin.  A
        value of digitalio.Pull.UP will enable a pull-up resistor, and None will
        disable it.  Pull-down resistors are NOT supported!
        """
        try:
            if _get_bit(self._exp.pull_en, self._pin):
                if _get_bit(self._exp.pull_sel, self._pin):
                    return digitalio.Pull.UP
                else:
                    return digitalio.Pull.DOWN
            else:
                return None

        except AttributeError as error:
            # Some expanders do not support pullups and/or pulldowns.
            raise ValueError("Pull-up/pull-down resistors not supported.") from error
        return None

    @pull.setter
    def pull(self, val: Pull) -> None:
        try:
            if val is None:
                self._exp.pull_en = _clear_bit(self._exp.pull_en, self._pin)
            elif val == digitalio.Pull.UP:
                self._exp.pull_sel = _enable_bit(self._exp.pull_sel, self._pin)
                self._exp.pull_en = _enable_bit(self._exp.pull_en, self._pin)
            elif val == digitalio.Pull.DOWN:
                self._exp.pull_sel = _clear_bit(self._exp.pull_sel, self._pin)
                self._exp.pull_en = _enable_bit(self._exp.pull_en, self._pin)
            else:
                raise ValueError("Expected UP, DOWN, or None for pull state!")
        except AttributeError as error:
            # Some expanders do not support pullups and/or pulldowns.
            raise ValueError("Pull-up/pull-down resistors not supported.") from error

    @property
    def drive_strength(self) -> int:
        """The drive strength of a pin when configured as an output"""

        pin_bit_offset = self._pin < 1  # divide pin by two for every 2 bits.
        mask = 0b11

        if self._pin >= 0 and self._pin <= 7:
            return (self._exp.iodrv_str_p0 << pin_bit_offset) & mask

        if self._pin >= 8 and self._pin <= 15:
            return (self._exp.iodrv_str_p1 << pin_bit_offset) & mask

    @drive_strength.setter
    def drive_strength(self, val: int):
        pass

    @property
    def invert_polarity(self) -> bool:
        """The polarity of the pin, either True for an Inverted or
        False for an normal.
        """
        if _get_bit(self._exp.ipol, self._pin):
            return True
        return False

    @invert_polarity.setter
    def invert_polarity(self, val: bool) -> None:
        if val:
            self._exp.ipol = _enable_bit(self._exp.ipol, self._pin)
        else:
            self._exp.ipol = _clear_bit(self._exp.ipol, self._pin)
