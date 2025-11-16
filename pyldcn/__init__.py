"""
pyldcn - Python LDCN Communication Library

A Python library for communicating with Logosol LDCN (Logosol Distributed
Control Network) devices including servo drives and I/O controllers.

Example:
    from pyldcn import LDCNNetwork

    with LDCNNetwork('/dev/ttyUSB0') as network:
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.devices[0]  # LS231SE
        servo.initialize()
        servo.move_to(10.0, 100.0, 50.0, scale=2000.0)

Author: NickyDoes
License: GPL v2 or later
Version: 0.1.0
"""

# Import from modular architecture
from .network import LDCNNetwork
from .device import LDCNDevice, UnknownDevice
from .discovery import InitMode
from .exceptions import (
    LDCNError,
    LDCNTimeoutError,
    LDCNChecksumError,
    LDCNDetectionError,
    LDCNInitializationError,
)

from .devices import (
    # Base device classes
    Servo,
    IOController,

    # Specific device types
    LS231SE,
    SK2310g2,
    LS773,
)

__version__ = '0.2.0'
__author__ = 'NickyDoes'
__license__ = 'GPL-2.0-or-later'

__all__ = [
    # Main classes
    'LDCNNetwork',
    'LDCNDevice',
    'UnknownDevice',
    'InitMode',

    # Base device classes
    'Servo',
    'IOController',

    # Specific devices
    'LS231SE',
    'SK2310g2',
    'LS773',

    # Exceptions
    'LDCNError',
    'LDCNTimeoutError',
    'LDCNChecksumError',
    'LDCNDetectionError',
    'LDCNInitializationError',

    # Version
    '__version__',
]
