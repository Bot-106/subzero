"""In-process NT4 server + client helpers for tests (separate NetworkTableInstance per role)."""
from __future__ import annotations

import asyncio
import socket
import time
from pathlib import Path

import ntcore


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class NtServer:
    def __init__(self, tmp: Path):
        self.port = free_port()
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startServer(
            persist_filename=str(tmp / "nt_persist.json"), listen_address="127.0.0.1", port3=0, port4=self.port
        )

    def stop(self) -> None:
        self.inst.stopServer()
        ntcore.NetworkTableInstance.destroy(self.inst)


def make_client(port: int, identity: str) -> ntcore.NetworkTableInstance:
    inst = ntcore.NetworkTableInstance.create()
    inst.startClient4(identity)
    inst.setServer("127.0.0.1", port)
    return inst


def destroy(inst: ntcore.NetworkTableInstance) -> None:
    try:
        inst.stopClient()
    finally:
        ntcore.NetworkTableInstance.destroy(inst)


async def wait_for(pred, timeout: float, what: str = "condition", step: float = 0.005) -> float:
    """Poll `pred` (sync) while letting the event loop run; returns elapsed seconds; raises on timeout."""
    t0 = time.perf_counter()
    while True:
        if pred():
            return time.perf_counter() - t0
        if time.perf_counter() - t0 > timeout:
            raise TimeoutError(f"timed out after {timeout}s waiting for {what}")
        await asyncio.sleep(step)
