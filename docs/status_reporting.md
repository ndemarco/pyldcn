# Logosol Status Reporting

**Author:** NickyDoes
**Source:** LS-773 Network I/O Node Datasheet (Doc# 712773001, Rev. B) and CNC-SK-2310g2 Manual (Doc # 710231005 / Rev. D, 03/05/2020)
**Date:** 2025-11-13

## General Introduction

This document amplifies the Logosol device status reporting subsystem. Refer to device type and individual device model documents for specfic implementation details.

---

### Status Reporting

Many Logosol LDCN-compliant devices report their internal state through status reporting. Because LDCN is a poll-based protocol—devices never push updates on their own—state information is returned only when the controller issues a Read Status or Nop command. Each response contains a configurable set of status items.

After power on or 'HARD RESET', both methods to read status will return no status items (`0x00`). 

To request a one-time custom set of status items, call Read Status with the byte-encoded list of the desired status items. This temporarily overrides the set defined by Define Status. After that, the next Nop command will revert the device back to the previously defined status-item set.


### Status Response

The structure of every status response is the same, but the contents of the status items section vary by device. Each device defines its own bit assignments, the meaning of each bit, and the size of the associated data.For exact mappings, see that device's documentation.

A status response packet always contains three parts. It's total length changes depending on which status items the device is configured to return.


#### 1. Status Byte

The first byte of every response packet contains error flags:

| Bit | Description | Note |
|-----|-------------|-----|
| **0** | Undefined | ignore |
| **1** | Checksum error flag | Set if a checksum error was detected in the most recent command packet |
| **2-7** | Undefined | ignore |

#### 2. Status Items

The next section contains the device's status items. One or more selector bytes determine which items are included: set a bit to return an item, clear it to omit it. Each device defines its own bit mapping, item definition, and the size of each return value.

| Bit | Data Item | Size | Description |
|-----|-----------|------|-------------|
| **0** | Input Bytes | 2 bytes | Digital input Byte 0, Byte 1 |
| **1** | Analog Input 0 | 1 byte | Analog input channel 0 (0-255) |
| **2** | Analog Input 1 | 1 byte | Analog input channel 1 (0-255) |
| **3** | Analog Input 2 | 1 byte | Analog input channel 2 (0-255) |
| **4** | Counter/Timer | 4 bytes | Counter/timer value (LSB first) |
| **5** | Device ID/Version | 2 bytes | Device ID byte, Version byte|
| **6** | Sync Input Bits | 2 bytes | Input bits captured with Sync Input command |
| **7** | Sync Counter/Timer | 4 bytes | Counter/timer captured with Sync Input (LSB first) |

#### 3. Checksum Byte

The final byte is the 8-bit sum of the status bytes and all retuned data bytes.
