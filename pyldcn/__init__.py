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

Author: LinuxCNC Community
License: GPL v2 or later
Version: 0.1.0
"""

from .network import (
    # Main classes
    LDCNNetwork,
    LDCNDevice,
    UnknownDevice,
    LS231SE,
    SK2310g2,

    # Exceptions
    LDCNError,
    LDCNTimeoutError,
    LDCNChecksumError,
    LDCNDetectionError,
    LDCNInitializationError,

    # Constants
    HEADER,
    ADDRESS_UNADDRESSED,
    ADDRESS_GROUP,
    DEFAULT_BAUD,
    BAUD_RATES,

    # Commands
    CMD_RESET_POS,
    CMD_SET_ADDRESS,
    CMD_DEFINE_STATUS,
    CMD_READ_STATUS,
    CMD_NOP,
    CMD_HARD_RESET,
    CMD_SET_BAUD,
    CMD_LOAD_TRAJECTORY,
    CMD_START_MOTION,
    CMD_LOAD_GAINS,
    CMD_STOP_MOTOR,
    CMD_CLEAR_BITS,

    # Status bits
    STATUS_MOVE_DONE,
    STATUS_CKSUM_ERROR,
    STATUS_CURRENT_LIMIT,
    STATUS_POWER_ON,
    STATUS_POS_ERROR,
    STATUS_HOME_SOURCE,
    STATUS_LIMIT2,
    STATUS_HOME_IN_PROG,
)

__version__ = '0.1.0'
__author__ = 'LinuxCNC Community'
__license__ = 'GPL-2.0-or-later'

__all__ = [
    # Main classes
    'LDCNNetwork',
    'LDCNDevice',
    'UnknownDevice',
    'LS231SE',
    'SK2310g2',

    # Exceptions
    'LDCNError',
    'LDCNTimeoutError',
    'LDCNChecksumError',
    'LDCNDetectionError',
    'LDCNInitializationError',

    # Version
    '__version__',
]
