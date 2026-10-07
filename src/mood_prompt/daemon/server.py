"""Daemon: owns one Backend, serves it over a WebSocket API.

Clients send SetTargetMsg (head pose + antennas + body yaw) and GetStateMsg;
the daemon runs IK on the pose to get the 7 head joints and calls the
backend's set_target_joints. A background tick steps the sim (no-op for
hardware) and broadcasts StateMsg to connected clients.
"""

from __future__ import annotations

import argparse
import asyncio
import logging

import numpy as np
import websockets

from mood_prompt.backend.abstract import Backend
from mood_prompt.kinematics.stewart_ik import StewartIK

from .protocol_messages import GetStateMsg, SetTargetMsg, StateMsg, parse

logger = logging.getLogger(__name__)


class Daemon:
    def __init__(
        self,
        backend: Backend,
        host: str = "127.0.0.1",
        port: int = 8765,
        tick_hz: float = 100.0,
        broadcast_hz: float = 20.0,
        step_sim: bool = False,
        render: bool = False,
    ):
        self.backend = backend
        self.host = host
        self.port = port
        self._tick_dt = 1.0 / tick_hz
        self._broadcast_every = max(1, int(tick_hz / broadcast_hz))
        self._step_sim = step_sim
        self._render = render
        self._ik = StewartIK()
        self._clients: set = set()

    async def _handler(self, ws) -> None:
        self._clients.add(ws)
        try:
            async for raw in ws:
                await self._on_message(ws, raw)
        finally:
            self._clients.discard(ws)

    async def _on_message(self, ws, raw: str) -> None:
        try:
            msg = parse(raw)
        except ValueError as exc:
            logger.warning("dropping bad message: %s", exc)
            return
        if isinstance(msg, SetTargetMsg):
            horns = self._ik.inverse_kinematics(msg.head_matrix, clamp=True)
            head = [msg.body_yaw, *horns]
            self.backend.set_target_joints(head, msg.antennas)
        elif isinstance(msg, GetStateMsg):
            await ws.send(self._state_msg().to_json())

    def _state_msg(self) -> StateMsg:
        head, antennas = self.backend.get_present_joints()
        return StateMsg(head=list(head), antennas=list(antennas))

    async def _tick_loop(self) -> None:
        n = 0
        while True:
            if self._step_sim:
                if self._render:
                    self.backend.step_and_render()
                else:
                    self.backend.step()
            if self._clients and n % self._broadcast_every == 0:
                payload = self._state_msg().to_json()
                websockets.broadcast(self._clients, payload)
            n += 1
            await asyncio.sleep(self._tick_dt)

    async def serve(self) -> None:
        self.backend.enable()
        logger.info("daemon listening on ws://%s:%d", self.host, self.port)
        async with websockets.serve(self._handler, self.host, self.port):
            await self._tick_loop()

    def run(self) -> None:
        try:
            asyncio.run(self.serve())
        finally:
            self.backend.close()


def _build_backend(args) -> tuple[Backend, bool]:
    """Return (backend, step_sim)."""
    if args.backend == "sim":
        from mood_prompt.backend.mujoco_sim import MujocoBackend

        return MujocoBackend(args.scene), True
    if args.backend == "hardware":
        from mood_prompt.backend.hardware import HardwareBackend

        return HardwareBackend(args.port), False
    raise ValueError(f"unknown backend: {args.backend}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Mood-Prompt daemon")
    parser.add_argument("--backend", choices=["sim", "hardware"], default="sim")
    parser.add_argument("--scene", default="scene.xml", help="MJCF scene (sim)")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port (hardware)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument(
        "--render",
        action="store_true",
        help="open the MuJoCo viewer (sim only; macOS needs mjpython)",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO)
    backend, step_sim = _build_backend(args)
    Daemon(
        backend,
        host=args.host,
        port=args.ws_port,
        step_sim=step_sim,
        render=args.render,
    ).run()


if __name__ == "__main__":
    main()
