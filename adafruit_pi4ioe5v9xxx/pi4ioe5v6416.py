# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2019 Carter Nelson
# SPDX-FileCopyrightText: 2024 Marshall Scholz
#
# SPDX-License-Identifier: MIT

# pylint: disable=too-many-public-methods

"""
`PI4IOE5V6416`
====================================================

CircuitPython module for the PI4IOE5V6416 I2C I/O extenders.

* Author(s): Tony DiCola (initial base library), Marshall Scholz (PI4IOE5V6416 adaptation)

Implementation Notes
--------------------

**Hardware:**

* `PI4IOE5V6416 - i2c 16 input/output port expander
  <https://www.diodes.com/assets/Datasheets/PI4IOE5V6416.pdf>`_


"""

from micropython import const
from .pi4ioe5v9xxx import PI4IOE5V9xxx
from .digital_inout import DigitalInOut

try:
    from typing import List
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx.git"

_PI4IOE5V6416_ADDRESS = const(0x20)
_PI4IOE5V6416_IN0 = const(0x00)  # R Input port 0
_PI4IOE5V6416_IN1 = const(0x01)  # R Input port 1
_PI4IOE5V6416_OUT0 = const(0x02)  # R/W Output port 0
_PI4IOE5V6416_OUT1 = const(0x03)  # R/W Output port 1
_PI4IOE5V6416_POL0 = const(0x04)  # R/W Polarity Inversion port 0 (inputs only)
_PI4IOE5V6416_POL1 = const(0x05)  # R/W Polarity Inversion port 1
_PI4IOE5V6416_IODIR0 = const(0x06)  # R/W Configuration port 0
_PI4IOE5V6416_IODIR1 = const(0x07)  # R/W Configuration port 1
_PI4IOE5V6416_ODRV00 = const(0x40)  # R/W Output drive strength register 00
_PI4IOE5V6416_ODRV01 = const(0x41)  # R/W Output drive strength register 0
_PI4IOE5V6416_ODRV10 = const(0x42)  # R/W Output drive strength register 1
_PI4IOE5V6416_ODRV11 = const(0x43)  # R/W Output drive strength register 1
_PI4IOE5V6416_ILATCH0 = const(0x44)  # R/W Input latch register 0
_PI4IOE5V6416_ILATCH1 = const(0x45)  # R/W Input latch register 1
_PI4IOE5V6416_PULLEN0 = const(0x46)  # R/W Pull-up/pull-down enable register 0
_PI4IOE5V6416_PULLEN1 = const(0x47)  # R/W Pull-up/pull-down enable register 1
_PI4IOE5V6416_PULLSEL0 = const(0x48)  # R/W Pull-up/pull-down selection register 0
_PI4IOE5V6416_PULLSEL1 = const(0x49)  # R/W Pull-up/pull-down selection register 1
_PI4IOE5V6416_INTMSK0 = const(0x4A)  # R/W Interrupt mask register 0
_PI4IOE5V6416_INTMSK1 = const(0x4B)  # R/W Interrupt mask register 1
_PI4IOE5V6416_INTSTAT0 = const(0x4C)  # R Interrupt status register 0
_PI4IOE5V6416_INTSTAT1 = const(0x4D)  # R Interrupt status register 1
_PI4IOE5V6416_OPNDRNCFG = const(0x4E)  # R/W Output port configuration register


class PI4IOE5V6416(PI4IOE5V9xxx):
    """Supports PI4IOE5V6416 instance on specified I2C bus and optionally
    at the specified I2C address.
    also cpmpatable as superset for KTS1622
    """

    def __init__(
        self, i2c: I2C, address: int = _PI4IOE5V6416_ADDRESS, reset: bool = True
    ) -> None:
        super().__init__(i2c, address)
        # if reset:
        # Reset to all inputs with no pull-ups and no inverted polarity.
        # self.iodir = 0xFFFF
        # self.gppu = 0x0000
        # self.iocon = 0x4  # turn on IRQ Pins as open drain
        # self._write_u16le(_PI4IOE5V6416_IPOLA, 0x0000)

    def get_pin(self, pin: int) -> DigitalInOut:
        """Convenience function to create an instance of the DigitalInOut class
        pointing at the specified pin of this PI4IOE5V6416 device.
        """
        if not (0 <= int(pin) <= 15):
            raise ValueError("Pin number must be 0-15.")
        return DigitalInOut(pin, self)

    # IO input read registers
    @property
    def gpio_in(self) -> int:
        """The raw GPIO Input registers. Read only.  Each bit represents the
        input value of the associated pin (0 = low, 1 = high).
        Input valiue can be inverted using POLx Registers.
        """
        return self._read_u16le(_PI4IOE5V6416_IN0)

    @property
    def gpio_in_p0(self) -> int:
        """The raw GPIO port 0 Input register. Read only.  Each bit represents the
        input value of the associated pin (0 = low, 1 = high).
        Input valiue can be inverted using POLx Registers.
        """
        return self._read_u8(_PI4IOE5V6416_IN0)

    @property
    def gpio_in_p1(self) -> int:
        """The raw GPIO port 0 Input register. Read only.  Each bit represents the
        input value of the associated pin (0 = low, 1 = high).
        Input valiue can be inverted using POLx Registers.
        """
        return self._read_u8(_PI4IOE5V6416_IN1)

    # IO input polarity
    @property
    def ipol(self) -> int:
        """The raw POL input register.  Each bit represents the
        polarity value of the associated pin (0 = normal, 1 = inverted).
        Applies to inputs ony.
        """
        return self._read_u16le(_PI4IOE5V6416_POL0)

    @ipol.setter
    def ipol(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_POL0, val)

    @property
    def ipol_p0(self) -> int:
        """The raw POL port 0 input register.  Each bit represents the
        polarity value of the associated pin (0 = normal, 1 = inverted).
        Applies to inputs ony.
        """
        return self._read_u8(_PI4IOE5V6416_POL0)

    @ipol_p0.setter
    def ipol_p0(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_POL0, val)

    @property
    def ipol_p1(self) -> int:
        """The raw POL port 1 input register.  Each bit represents the
        polarity value of the associated pin (0 = normal, 1 = inverted).
        Applies to inputs ony.
        """
        return self._read_u8(_PI4IOE5V6416_POL1)

    @ipol_p1.setter
    def ipol_p1(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_POL1, val)

    # IO output registers
    @property
    def gpio_out(self) -> int:
        """The raw GPIO output register.  Each bit represents the
        output value of the associated pin (0 = low, 1 = high), assuming that
        pin has been configured as an output previously.
        """
        return self._read_u16le(_PI4IOE5V6416_OUT0)

    @gpio_out.setter
    def gpio_out(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_OUT0, val)

    @property
    def gpio_out_p0(self) -> int:
        """The raw GPIO A output register.  Each bit represents the
        output value of the associated pin (0 = low, 1 = high), assuming that
        pin has been configured as an output previously.
        """
        return self._read_u8(_PI4IOE5V6416_OUT0)

    @gpio_out_p0.setter
    def gpio_out_p0(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_OUT0, val)

    @property
    def gpio_out_p1(self) -> int:
        """The raw GPIO B output register.  Each bit represents the
        output value of the associated pin (0 = low, 1 = high), assuming that
        pin has been configured as an output previously.
        """
        return self._read_u8(_PI4IOE5V6416_OUT1)

    @gpio_out_p1.setter
    def gpio_out_p1(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_OUT1, val)

    @property
    def iodir(self) -> int:
        """The raw IODIR direction register.  Each bit represents
        direction of a pin, either 1 for an input or 0 for an output mode.
        """
        return self._read_u16le(_PI4IOE5V6416_IODIR0)

    # IO direction
    @iodir.setter
    def iodir(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_IODIR0, val)

    @property
    def iodir_p0(self) -> int:
        """The raw IODIR A direction register.  Each bit represents
        direction of a pin, either 1 for an input or 0 for an output mode.
        """
        return self._read_u8(_PI4IOE5V6416_IODIR0)

    @iodir_p0.setter
    def iodir_p0(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_IODIR0, val)

    @property
    def iodir_p1(self) -> int:
        """The raw IODIR B direction register.  Each bit represents
        direction of a pin, either 1 for an input or 0 for an output mode.
        """
        return self._read_u8(_PI4IOE5V6416_IODIR1)

    @iodir_p1.setter
    def iodir_p1(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_IODIR1, val)

    # IO drive strength
    @property
    def iodrv_str_p0(self) -> int:
        """The raw ODRV0 drive strength register.  Every 2 bits represents
        drive strength of a pin, ranging from 0 to 3.
        """

        # todo, this is 32 bits of data across 4 bytes. not 16.

        # _PI4IOE5V6416_ODRV00
        # _PI4IOE5V6416_ODRV01

        return self._read_u16le(_PI4IOE5V6416_ODRV00)

    @iodrv_str_p0.setter
    def iodrv_str_p0(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_ODRV00, val)

        # IO drive strength
    @property
    def iodrv_str_p1(self) -> int:
        """The raw ODRV1 drive strength register.  Every 2 bits represents
        drive strength of a pin, ranging from 0 to 3.
        """

        # _PI4IOE5V6416_ODRV10
        # _PI4IOE5V6416_ODRV11
        return self._read_u16le(_PI4IOE5V6416_ODRV10)

    @iodrv_str_p1.setter
    def iodrv_str_p1(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_ODRV00, val)

    # Pull-up/pull-down enable
    @property
    def pull_en(self) -> int:
        """The raw pullup/pulldown enable registers.  Each bit represents
        if a pull-xx is enabled on the specified pin (1 = pull-xx enabled,
        0 = pull-xx disabled).
        Pullup/pulldown selection occurs in the PULLSEL register.
        """
        return self._read_u16le(_PI4IOE5V6416_PULLEN0)

    @pull_en.setter
    def pull_en(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_PULLEN0, val)

    @property
    def pull_en_p0(self) -> int:
        """The raw pullup/pulldown port 0 enable registers.  Each bit represents
        if a pull-xx is enabled on the specified pin (1 = pull-xx enabled,
        0 = pull-xx disabled).
        Pullup/pulldown selection occurs in the PULLSEL register.
        """
        return self._read_u8(_PI4IOE5V6416_PULLEN0)

    @pull_en_p0.setter
    def pull_en_p0(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_PULLEN0, val)

    @property
    def pull_en_p1(self) -> int:
        """The raw pullup/pulldown port 1 enable registers.  Each bit represents
        if a pull-xx is enabled on the specified pin (1 = pull-xx enabled,
        0 = pull-xx disabled).
        Pullup/pulldown selection occurs in the PULLSEL register.
        """
        return self._read_u8(_PI4IOE5V6416_PULLEN1)

    @pull_en_p1.setter
    def pull_en_p1(self, val: int) -> None:
        self._write_u8(_PI4IOE5V6416_PULLEN1, val)

    # Pull-up/Pull-down select
    @property
    def pull_sel(self) -> int:
        """The raw pullup/pulldown selection registers.  Each bit represents
        if a pull-up or pull-down is applied on the specified pin when the 
        PULLEN register is enabled for that pin. 
        (1 = pull-up 100kohm, 0 = pull-down 100kohm).
        """
        return self._read_u16le(_PI4IOE5V6416_PULLSEL0)

    @pull_sel.setter
    def pull_sel(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_PULLSEL0, val)

    @property
    def pull_sel_p0(self) -> int:
        """The raw pullup/pulldown selection registers.  Each bit represents
        if a pull-up or pull-down is applied on the specified pin when the 
        PULLEN register is enabled for that pin. 
        (1 = pull-up 100kohm, 0 = pull-down 100kohm).
        """
        return self._read_u16le(_PI4IOE5V6416_PULLSEL0)

    @pull_sel_p0.setter
    def pull_sel_p0(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_PULLSEL0, val)

    @property
    def pull_sel_p1(self) -> int:
        """The raw pullup/pulldown selection registers.  Each bit represents
        if a pull-up or pull-down is applied on the specified pin when the 
        PULLEN register is enabled for that pin. 
        (1 = pull-up 100kohm, 0 = pull-down 100kohm).
        """
        return self._read_u16le(_PI4IOE5V6416_PULLSEL0)

    @pull_sel_p1.setter
    def pull_sel_p1(self, val: int) -> None:
        self._write_u16le(_PI4IOE5V6416_PULLSEL0, val)

    # @property
    # def interrupt_configuration(self) -> int:
    #     """The raw INTCON interrupt control register. The INTCON register
    #     controls how the associated pin value is compared for the
    #     interrupt-on-change feature. If  a  bit  is  set,  the  corresponding
    #     I/O  pin  is  compared against the associated bit in the DEFVAL
    #     register. If a bit value is clear, the corresponding I/O pin is
    #     compared against the previous value.
    #     """
    #     return self._read_u16le(_PI4IOE5V6416_INTCONA)

    # @interrupt_configuration.setter
    # def interrupt_configuration(self, val: int) -> None:
    #     self._write_u16le(_PI4IOE5V6416_INTCONA, val)

    # @property
    # def interrupt_enable(self) -> int:
    #     """The raw GPINTEN interrupt control register. The GPINTEN register
    #     controls the interrupt-on-change feature for each pin. If a bit is
    #     set, the corresponding pin is enabled for interrupt-on-change.
    #     The DEFVAL and INTCON registers must also be configured if any pins
    #     are enabled for interrupt-on-change.
    #     """
    #     return self._read_u16le(_PI4IOE5V6416_GPINTENA)

    # @interrupt_enable.setter
    # def interrupt_enable(self, val: int) -> None:
    #     self._write_u16le(_PI4IOE5V6416_GPINTENA, val)

    # @property
    # def default_value(self) -> int:
    #     """The raw DEFVAL interrupt control register. The default comparison
    #     value is configured in the DEFVAL register. If enabled (via GPINTEN
    #     and INTCON) to compare against the DEFVAL register, an opposite value
    #     on the associated pin will cause an interrupt to occur.
    #     """
    #     return self._read_u16le(_PI4IOE5V6416_DEFVALA)

    # @default_value.setter
    # def default_value(self, val: int) -> None:
    #     self._write_u16le(_PI4IOE5V6416_DEFVALA, val)

    # @property
    # def io_control(self) -> int:
    #     """The raw IOCON configuration register. Bit 1 controls interrupt
    #     polarity (1 = active-high, 0 = active-low). Bit 2 is whether irq pin
    #     is open drain (1 = open drain, 0 = push-pull). Bit 3 is unused.
    #     Bit 4 is whether SDA slew rate is enabled (1 = yes). Bit 5 is if I2C
    #     address pointer auto-increments (1 = no). Bit 6 is whether interrupt
    #     pins are internally connected (1 = yes). Bit 7 is whether registers
    #     are all in one bank (1 = no), this is silently ignored if set to ``1``.
    #     """
    #     return self._read_u8(_PI4IOE5V6416_IOCON)

    # @io_control.setter
    # def io_control(self, val: int) -> None:
    #     val &= ~0x80
    #     self._write_u8(_PI4IOE5V6416_IOCON, val)

    # @property
    # def int_flag(self) -> List[int]:
    #     """Returns a list with the pin numbers that caused an interrupt
    #     port A ----> pins 0-7
    #     port B ----> pins 8-15
    #     """
    #     intf = self._read_u16le(_PI4IOE5V6416_INTFA)
    #     flags = [pin for pin in range(16) if intf & (1 << pin)]
    #     return flags

    # @property
    # def int_flaga(self) -> List[int]:
    #     """Returns a list of pin numbers that caused an interrupt in port A
    #     pins: 0-7
    #     """
    #     intfa = self._read_u8(_PI4IOE5V6416_INTFA)
    #     flags = [pin for pin in range(8) if intfa & (1 << pin)]
    #     return flags

    # @property
    # def int_flagb(self) -> List[int]:
    #     """Returns a list of pin numbers that caused an interrupt in port B
    #     pins: 8-15
    #     """
    #     intfb = self._read_u8(_PI4IOE5V6416_INTFB)
    #     flags = [pin + 8 for pin in range(8) if intfb & (1 << pin)]
    #     return flags

    # @property
    # def int_cap(self) -> List[int]:
    #     """Returns a list with the pin values at time of interrupt
    #     port A ----> pins 0-7
    #     port B ----> pins 8-15
    #     """
    #     intcap = self._read_u16le(_PI4IOE5V6416_INTCAPA)
    #     return [(intcap >> pin) & 1 for pin in range(16)]

    # @property
    # def int_capa(self) -> List[int]:
    #     """Returns a list of pin values at time of interrupt
    #     pins: 0-7
    #     """
    #     intcapa = self._read_u8(_PI4IOE5V6416_INTCAPA)
    #     return [(intcapa >> pin) & 1 for pin in range(8)]

    # @property
    # def int_capb(self) -> List[int]:
    #     """Returns a list of pin values at time of interrupt
    #     pins: 8-15
    #     """
    #     intcapb = self._read_u8(_PI4IOE5V6416_INTCAPB)
    #     return [(intcapb >> pin) & 1 for pin in range(8)]

    # def clear_ints(self) -> None:
    #     """Clears interrupts by reading INTCAP."""
    #     self._read_u16le(_PI4IOE5V6416_INTCAPA)

    # def clear_inta(self) -> None:
    #     """Clears port A interrupts."""
    #     self._read_u8(_PI4IOE5V6416_INTCAPA)

    # def clear_intb(self) -> None:
    #     """Clears port B interrupts."""
    #     self._read_u8(_PI4IOE5V6416_INTCAPB)
