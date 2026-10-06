"""Protocol 2.0 wire-format tests.

Byte sequences are checked against ROBOTIS Protocol 2.0 documentation
examples, not just self-consistent round-trips. The canonical example
used throughout is the PING instruction packet to servo ID 1:

    FF FF FD 00 01 03 00 01 19 4E

where the trailing ``19 4E`` is the little-endian CRC (0x4E19) over the
preceding 8 bytes.
"""

import pytest

from mood_prompt.protocol import instructions
from mood_prompt.protocol.control_table import GOAL_POSITION
from mood_prompt.protocol.crc import compute_crc
from mood_prompt.protocol.packet import (
    PacketError,
    decode_status_packet,
    encode_instruction_packet,
)


def test_crc_known_value():
    # The 8 bytes preceding the CRC field of the ROBOTIS PING example.
    preamble = bytes([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01])
    # Documented CRC is 0x4E19 (stored on the wire as 19 4E, little-endian).
    assert compute_crc(preamble) == 0x4E19


def test_encode_ping_matches_robotis_example():
    # ROBOTIS documented PING to ID 1, byte-for-byte.
    expected = bytes([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E])
    assert encode_instruction_packet(1, instructions.PING) == expected


def test_encode_write_goal_position():
    # WRITE of Goal Position = 0x00000096 (150) to ID 1, addr 116:
    #   FF FF FD 00 01 09 00 03 74 00 96 00 00 00 D8 B1
    # Body matches the ROBOTIS documented example; the CRC (wire D8 B1 =
    # 0xB1D8) is verified by compute_crc, itself anchored by the PING
    # known-answer above.
    params = bytes([GOAL_POSITION.address & 0xFF, (GOAL_POSITION.address >> 8) & 0xFF])
    params += (0x00000096).to_bytes(4, "little")
    packet = encode_instruction_packet(1, instructions.WRITE, params)
    expected = bytes(
        [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03,
         0x74, 0x00, 0x96, 0x00, 0x00, 0x00, 0xD8, 0xB1]
    )
    assert packet == expected


def test_decode_status_packet():
    # ROBOTIS documented status packet from ID 1 for a PING reply
    # (model 0x0406, firmware 0x26): error 0x00, params = 06 04 26.
    raw = bytes(
        [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55,
         0x00, 0x06, 0x04, 0x26, 0x65, 0x5D]
    )
    status = decode_status_packet(raw)
    assert status.id == 1
    assert status.error == 0x00
    assert status.params == bytes([0x06, 0x04, 0x26])


def test_decode_rejects_corrupted_crc():
    raw = bytearray(
        [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55,
         0x00, 0x06, 0x04, 0x26, 0x65, 0x5D]
    )
    raw[-1] ^= 0xFF  # corrupt the high CRC byte
    with pytest.raises(PacketError, match="CRC"):
        decode_status_packet(bytes(raw))


def test_decode_rejects_bad_header():
    raw = bytes([0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x65, 0x5D])
    with pytest.raises(PacketError, match="header"):
        decode_status_packet(raw)


def test_byte_stuffing_fires_on_header_pattern():
    # A param block containing the FF FF FD pattern must be stuffed so it
    # can't be mistaken for a packet header on the wire.
    tricky = bytes([0xFF, 0xFF, 0xFD, 0x01, 0x02])
    packet = encode_instruction_packet(5, instructions.WRITE, tricky)
    # Unstuffed size would be: 4 header + id + 2 len + 1 instr + params + 2 crc.
    unstuffed_size = 4 + 1 + 2 + 1 + len(tricky) + 2
    # Exactly one 0xFD stuffing byte is inserted after the FF FF FD run.
    assert len(packet) == unstuffed_size + 1
