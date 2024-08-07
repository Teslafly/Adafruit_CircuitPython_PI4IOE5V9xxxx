Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-PI4IOE5V9xxx/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/PI4IOE5V9xxx/en/latest/
    :alt: Documentation Status

.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx/actions/
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython module for PI4IOE5V9xxx series of I2C I/O extenders.



REFACTOR CURRENTLY IN PROGRESS. FORKED FROM https://github.com/adafruit/Adafruit_CircuitPython_MCP230xx







Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-PI4IOE5V9xxx/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-pi4ioe5v9xxx

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-pi4ioe5v9xxx

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .venv/bin/activate
    pip3 install adafruit-circuitpython-pi4ioe5v9xxx

Usage Example
=============

See examples/ for more examples of usage.

Single Ended
------------

.. code-block:: python

    import time
    import board
    import busio
    import adafruit_ads1x15.ads1015 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn

    # Create the I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create the ADC object using the I2C bus
    ads = ADS.ADS1015(i2c)

    # Create single-ended input on channel 0
    chan = AnalogIn(ads, ADS.P0)

    # Create differential input between channel 0 and 1
    #chan = AnalogIn(ads, ADS.P0, ADS.P1)


Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/pi4ioe5v9xxx/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Supported Devices
=============

* PI4IOE5V6416 (16 IO, full io)
* KTS1622 (16 IO)

Not Supported:
* PI4IOE5V9535 (16, no pulldown)


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_PI4IOE5V9xxx/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.


pi4ioe5v9xxx
PI4IOE5V9xxx