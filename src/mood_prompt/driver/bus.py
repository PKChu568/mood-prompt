"""DynamixelBus: packet I/O over a serial port, with timeout and retries."""

from __future__ import annotations

import serial

from mood_prompt.protocol import instructions
from mood_prompt.protocol.packet import (
    StatusPacket,
    decode_status_packet,
    encode_instruction_packet,
)


class BusError(Exception):
    """Raised when a servo does not reply within the allowed retries."""


class DynamixelBus:
    """Talks Protocol 2.0 to servos on a serial port.

    ``port`` may be a device path (``/dev/ttyUSB0``) opened as a real
    ``pyserial`` port, or any object exposing ``write()``/``read()`` — the
    fake bus in tests uses the latter so this code path runs unchanged.
    """

    def __init__(self, port, baudrate: int = 1_000_000, timeout: float = 0.05):
        if isinstance(port, str):
            self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        else:
            self._serial = port  # already a port-like object (e.g. FakeBus)
        self._timeout = timeout

    # --- raw packet I/O (T2.3) ---------------------------------------

    def write_packet(self, data: bytes) -> None:
        self._serial.write(data)

    def read_packet(self) -> bytes:
        """Read one status packet, framed by its own length field.

        Returns an empty ``bytes`` if nothing arrives (timeout).
        """
        header = self._read_exact(7)  # FF FF FD 00 ID LEN_L LEN_H
        if len(header) < 7:
            return b""
        length = header[5] | (header[6] << 8)
        rest = self._read_exact(length)
        return header + rest

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._serial.read(n - len(buf))
            if not chunk:
                break  # timeout / nothing more available
            buf += chunk
        return bytes(buf)

    # --- operations (T2.4) -------------------------------------------

    def _txn(self, id: int, instruction: int, params: bytes, retries: int) -> StatusPacket:
        packet = encode_instruction_packet(id, instruction, params)
        last_exc: Exception | None = None
        for _ in range(retries + 1):
            if hasattr(self._serial, "reset_input_buffer"):
                self._serial.reset_input_buffer()
            self.write_packet(packet)
            raw = self.read_packet()
            if not raw:
                last_exc = BusError(f"no reply from servo {id}")
                continue
            try:
                return decode_status_packet(raw)
            except Exception as exc:  # malformed / CRC -> retry
                last_exc = exc
        raise last_exc if last_exc else BusError(f"no reply from servo {id}")

    def ping(self, id: int, retries: int = 2) -> StatusPacket:
        return self._txn(id, instructions.PING, b"", retries)

    def read(self, id: int, address: int, size: int, retries: int = 2) -> bytes:
        params = bytes(
            [address & 0xFF, (address >> 8) & 0xFF, size & 0xFF, (size >> 8) & 0xFF]
        )
        return self._txn(id, instructions.READ, params, retries).params

    def write(self, id: int, address: int, data: bytes, retries: int = 2) -> None:
        params = bytes([address & 0xFF, (address >> 8) & 0xFF]) + data
        self._txn(id, instructions.WRITE, params, retries)

    def close(self) -> None:
        self._serial.close()
