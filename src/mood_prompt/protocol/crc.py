"""CRC-16 for Dynamixel Protocol 2.0.

The protocol appends a 16-bit CRC (polynomial 0x8005, no reflection, init 0)
over every byte of a packet up to but not including the CRC field itself.
The lookup table below is the standard ROBOTIS table; see
``compute_crc`` for the reduction step.
"""

# CRC-16 table for polynomial 0x8005 (CRC-16/BUYPASS style: MSB-first,
# no input/output reflection, initial value 0). Generated once at import
# time rather than hard-coding 256 magic numbers.
_CRC_POLY = 0x8005


def _build_table() -> tuple[int, ...]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ _CRC_POLY
            else:
                crc <<= 1
            crc &= 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_table()


def compute_crc(data: bytes) -> int:
    """Return the 16-bit Protocol 2.0 CRC of ``data``.

    ``data`` is every byte of the packet that precedes the CRC field
    (header, reserved, id, length, instruction/error, and parameters).
    The result is a 16-bit int; the wire format stores it little-endian
    (CRC_L then CRC_H).
    """
    crc = 0
    for byte in data:
        index = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _CRC_TABLE[index]) & 0xFFFF
    return crc
