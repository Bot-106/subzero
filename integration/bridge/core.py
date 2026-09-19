"""Bridge: wires NtClient <-> one ToolHttp per configured tool (H-09: only ids in config are polled)."""
from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path

import httpx
import yaml

from .nt_client import NtClient
from .tool_http import Timings, ToolHttp

log = logging.getLogger("bridge")

ALIVE_PERIOD_S = 0.2
CMD_POLL_S = 0.01
WAIT_LOG_PERIOD_S = 2.0


def load_config(path: str | Path, profile: str) -> tuple[dict[int, str], Timings, dict]:
    cfg = yaml.safe_load(Path(path).read_text()) or {}
    profiles = cfg.get("profiles") or {}
    if profile not in profiles:
        raise SystemExit(f"profile {profile!r} not in {path} (have: {', '.join(profiles)})")
    prof = profiles[profile] or {}
    tools = {int(k): str(v) for k, v in (prof.get("tools") or {}).items()}
    return tools, Timings.from_dict(cfg.get("timings")), prof


class Bridge:
    def __init__(
        self,
        nt: NtClient,
        tools: dict[int, str],
        timings: Timings | None = None,
        *,
        clients: dict[int, httpx.AsyncClient] | None = None,
    ) -> None:
        self.nt = nt
        self.timings = timings or Timings()
        self.tools: dict[int, ToolHttp] = {
            tid: ToolHttp(
                tid, host, self.timings, client=(clients or {}).get(tid),
                on_online=self.nt.publish_online, on_ack=self.nt.publish_last_seq, on_status=self.nt.publish_status,
            )
            for tid, host in tools.items()
        }
        self._tasks: set[asyncio.Task] = set()
        self.stats = {"cmds": 0, "acks": 0, "estops": 0}

    def _spawn(self, coro, name: str) -> None:
        t = asyncio.create_task(coro, name=name)
        self._tasks.add(t)
        t.add_done_callback(self._tasks.discard)

    async def run(self) -> None:
        self.nt.connect()
        for tool in self.tools.values():
            await tool.start()
        log.info("bridge up: tools=%s", {k: v.base_url for k, v in self.tools.items()} or "none")
        loops = [
            asyncio.create_task(self._alive_loop(), name="alive"),
            asyncio.create_task(self._cmd_loop(), name="cmd"),
        ]
        try:
            await asyncio.gather(*loops)
        finally:
            for t in loops:
                t.cancel()
            for t in list(self._tasks):
                t.cancel()
            for tool in self.tools.values():
                await tool.stop()
            self.nt.close()
            log.info("bridge stopped")

    async def _alive_loop(self) -> None:
        connected = False
        last_wait_log = 0.0
        while True:
            self.nt.toggle_alive()
            now = time.monotonic()
            c = self.nt.is_connected()
            if c and not connected:
                log.info("connected to NT server %s:%s", self.nt.server, self.nt.port or 5810)
            elif not c and (connected or now - last_wait_log >= WAIT_LOG_PERIOD_S):
                log.info("waiting for NT server %s:%s", self.nt.server, self.nt.port or 5810)
                last_wait_log = now
            connected = c
            await asyncio.sleep(ALIVE_PERIOD_S)

    async def _cmd_loop(self) -> None:
        while True:
            for cmd in self.nt.read_cmds():
                self.stats["cmds"] += 1
                self.dispatch(cmd)
            if any(self.nt.read_estop()):
                self.estop_all()
            await asyncio.sleep(CMD_POLL_S)

    def dispatch(self, cmd: dict) -> None:
        try:
            tid = int(cmd.get("tool"))
        except (TypeError, ValueError):
            log.warning("cmd without valid tool id: %r", cmd)
            return
        tool = self.tools.get(tid)
        if tool is None:
            log.warning("cmd for unconfigured tool %d ignored (H-09): %r", tid, cmd)
            return
        self._spawn(self._run_op(tool, cmd), name=f"op-{tid}-{cmd.get('seq')}")

    async def _run_op(self, tool: ToolHttp, cmd: dict) -> None:
        if await tool.op(cmd):
            self.stats["acks"] += 1

    def estop_all(self) -> None:
        self.stats["estops"] += 1
        log.warning("/subzero/estop == true -> POST /estop to every tool")
        for tool in self.tools.values():
            self._spawn(tool.estop(), name=f"estop-{tool.tool_id}")
