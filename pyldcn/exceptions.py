"""
LDCN Exception Classes

Custom exceptions for LDCN network operations.

Author: NickyDoes
License: GPL v2 or later
"""


class LDCNError(Exception):
    """Base exception for LDCN errors."""
    pass


class LDCNTimeoutError(LDCNError):
    """No response from device (timeout)."""
    pass


class LDCNChecksumError(LDCNError):
    """Response checksum mismatch."""
    pass


class LDCNDetectionError(LDCNError):
    """Auto-detection failed."""
    pass


class LDCNInitializationError(LDCNError):
    """Device initialization failed."""
    pass
