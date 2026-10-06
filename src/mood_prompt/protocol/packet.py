"""Dynamixel Protocol 2.0 packet encode/decode.

Packet layout (Protocol 2.0)::

    FF FF FD 00  ID  LEN_L LEN_H  INSTR/ERR  PARAM...  CRC_L CRC_H

``LEN`` counts the bytes from ``INSTR/ERR`` through ``CRC_H`` inclusive,
i.e. ``len(params) + 3`` (1 instruction/error byte + 2 CRC bytes).

Byte stuffing: anywhere the sequence ``FF FF FD`` appears inside the
payload (instruction + params), an extra ``FD`` is inserted so it can't be
mistaken for the header. The inserted bytes are counted in ``LEN`` and
included in the CRC. Status packets arrive stuffed and must be unstuffed
before interpreting.
"""

from __future__ import annotations

from dataclasses import dataclass

from .crc import compute_crc

HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])


class PacketError(Exception):
    """Raised on a malformed packet or a CRC mismatch."""


def _stuff(payload: bytes) -> bytes:
    """Insert a 0xFD after every ``FF FF FD`` run in ``payload``."""
    out = bytearray()
    run = 0  # count of consecutive leading-pattern bytes matched so far
    for byte in payload:
        out.append(byte)
        if run == 0 and byte == 0xFF:
            run = 1
        elif run == 1 and byte == 0xFF:
            run = 2
        elif run == 2 and byte == 0xFD:
            out.append(0xFD)  # stuffing byte
            run = 0
        else:
            run = 1 if byte == 0xFF else 0
    return bytes(out)


def _unstuff(payload: bytes) -> bytes:
    """Reverse ``_stuff``: drop the extra 0xFD after each ``FF FF FD``."""
    out = bytearray()
    i = 0
    n = len(payload)
    while i < n:
        out.append(payload[i])
        if (
            i + 3 < n
            and payload[i] == 0xFF
            and payload[i + 1] == 0xFF
            and payload[i + 2] == 0xFD
            and payload[i + 3] == 0xFD
        ):
            out.append(payload[i + 1])
            out.append(payload[i + 2])
            i += 4  # skip the stuffing 0xFD
            continue
        i += 1
    return bytes(out)


def encode_instruction_packet(id: int, instruction: int, params: bytes = b"") -> bytes:
    """Build a complete instruction packet, byte-stuffed and CRC-appended."""
    payload = _stuff(bytes([instruction]) + params)
    length = len(payload) + 2  # payload bytes + 2 CRC bytes
    body = HEADER + bytes([id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload
    crc = compute_crc(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


@dataclass
class InstructionPacket:
    id: int
    instruction: int
    params: bytes


def decode_instruction_packet(data: bytes) -> InstructionPacket:
    """Parse an instruction packet, validating header, length, and CRC.

    The mirror of :func:`encode_instruction_packet`. Raises
    :class:`PacketError` on a malformed field or CRC mismatch.
    """
    if len(data) < 10:
        raise PacketError(f"packet too short: {len(data)} bytes")
    if data[:4] != HEADER:
        raise PacketError(f"bad header: {data[:4].hex()}")

    length = data[5] | (data[6] << 8)
    expected_total = 7 + length
    if len(data) != expected_total:
        raise PacketError(
            f"length mismatch: field says {length} "
            f"(total {expected_total}), got {len(data)} bytes"
        )

    received_crc = data[-2] | (data[-1] << 8)
    if compute_crc(data[:-2]) != received_crc:
        raise PacketError("CRC mismatch")

    payload = _unstuff(data[7:-2])  # instruction byte + params
    return InstructionPacket(id=data[4], instruction=payload[0], params=payload[1:])


@dataclass
class StatusPacket:
    id: int
    error: int
    params: bytes


def encode_status_packet(id: int, error: int, params: bytes = b"") -> bytes:
    """Build a status packet (instruction byte 0x55), stuffed and CRC'd.

    The mirror of :func:`decode_status_packet`; used by the fake servo to
    reply to the driver.
    """
    payload = _stuff(bytes([0x55, error & 0xFF]) + params)
    length = len(payload) + 2  # payload bytes + 2 CRC bytes
    body = HEADER + bytes([id & 0xFF, length & 0xFF, (length >> 8) & 0xFF]) + payload
    crc = compute_crc(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def decode_status_packet(data: bytes) -> StatusPacket:
    """Parse a status packet, validating header, length, and CRC.

    Raises :class:`PacketError` on any malformed field or CRC mismatch.
    """
    if len(data) < 11:
        raise PacketError(f"packet too short: {len(data)} bytes")
    if data[:4] != HEADER:
        raise PacketError(f"bad header: {data[:4].hex()}")

    servo_id = data[4]
    length = data[5] | (data[6] << 8)
    expected_total = 7 + length  # 7 header/id/len bytes + length field
    if len(data) != expected_total:
        raise PacketError(
            f"length mismatch: field says {length} "
            f"(total {expected_total}), got {len(data)} bytes"
        )

    received_crc = data[-2] | (data[-1] << 8)
    if compute_crc(data[:-2]) != received_crc:
        raise PacketError("CRC mismatch")

    # Status packets have an instruction byte of 0x55 followed by the error
    # byte; the stuffed region is error + params.
    if data[7] != 0x55:
        raise PacketError(f"not a status packet (instr=0x{data[7]:02x})")

    payload = _unstuff(data[8:-2])  # error byte + params
    if not payload:
        raise PacketError("status packet missing error byte")
    return StatusPacket(id=servo_id, error=payload[0], params=payload[1:])
