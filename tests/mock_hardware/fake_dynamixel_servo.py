"""In-memory fake Dynamixel servo and bus for driver tests.

Lets the real driver code path run end-to-end with no physical hardware.
The FakeBus is exposed with ``read()``/``write()`` so ``DynamixelBus`` can
talk to it exactly as it would a ``pyserial`` port.

Approach: FakeBus buffers bytes in memory (no socketpair / loop:// url).
The driver writes a full instruction packet, FakeBus parses it, routes it
to the addressed FakeServo, and stashes the servo's status reply in a read
buffer that the driver then drains via ``read()``.
"""

from __future__ import annotations

from mood_prompt.protocol import instructions
from mood_prompt.protocol.packet import (
    HEADER,
    decode_instruction_packet,
    encode_status_packet,
)

BROADCAST_ID = 0xFE

# XL330 factory defaults for the registers we model, as (address: value).
# Positions default to 2048 (center of the 0..4095 range).
_XL330_DEFAULTS = {
    64: 0,  # Torque Enable off
    116: 2048,  # Goal Position
    132: 2048,  # Present Position
}


class FakeServo:
    """A single servo holding a byte-addressable register space."""

    # Register sizes (bytes) for addresses this fake understands.
    _SIZES = {11: 1, 64: 1, 70: 1, 102: 2, 116: 4, 126: 2, 132: 4}

    def __init__(self, servo_id: int):
        self.id = servo_id
        # Flat byte store; registers are little-endian slices of this.
        self._mem = bytearray(1024)
        for addr, value in _XL330_DEFAULTS.items():
            self._write_reg(addr, value)

    def _size(self, address: int) -> int:
        return self._SIZES.get(address, 1)

    def _read_reg(self, address: int, size: int) -> int:
        return int.from_bytes(self._mem[address : address + size], "little")

    def _write_reg(self, address: int, value: int) -> None:
        size = self._size(address)
        self._mem[address : address + size] = value.to_bytes(size, "little")

    def handle_packet(self, raw: bytes) -> bytes | None:
        """Decode an instruction packet and return a status reply.

        Returns ``None`` if the packet is not addressed to this servo (and
        isn't a broadcast it should answer).
        """
        pkt = decode_instruction_packet(raw)
        if pkt.id != self.id and pkt.id != BROADCAST_ID:
            return None

        if pkt.instruction == instructions.PING:
            return encode_status_packet(self.id, 0, b"")

        if pkt.instruction == instructions.READ:
            address = pkt.params[0] | (pkt.params[1] << 8)
            size = pkt.params[2] | (pkt.params[3] << 8)
            data = bytes(self._mem[address : address + size])
            return encode_status_packet(self.id, 0, data)

        if pkt.instruction == instructions.WRITE:
            address = pkt.params[0] | (pkt.params[1] << 8)
            data = pkt.params[2:]
            self._mem[address : address + len(data)] = data
            # Mirror Goal Position straight into Present Position so a
            # write-then-read round-trips without a physics model.
            if address == 116:
                self._mem[132:136] = data[:4]
            return encode_status_packet(self.id, 0, b"")

        return encode_status_packet(self.id, 0, b"")


class FakeBus:
    """pyserial-compatible fake holding several FakeServos by id."""

    def __init__(self, servos: list[FakeServo]):
        self._servos = {s.id: s for s in servos}
        self._rx = bytearray()  # bytes waiting to be read() by the driver

    def write(self, data: bytes) -> int:
        """Accept a full instruction packet and queue any status reply."""
        pkt_id = data[4]
        if pkt_id == BROADCAST_ID:
            for servo in self._servos.values():
                reply = servo.handle_packet(data)
                if reply:
                    self._rx += reply
        else:
            servo = self._servos.get(pkt_id)
            if servo is not None:
                reply = servo.handle_packet(data)
                if reply:
                    self._rx += reply
        # Absent servo id -> no reply queued, so the driver read times out.
        return len(data)

    def read(self, size: int = 1) -> bytes:
        """Drain up to ``size`` bytes from the reply buffer."""
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        self._rx.clear()
