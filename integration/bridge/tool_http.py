"""ToolHttp: one HTTP client per ESP32 tool (contract C.2), with heartbeat + online/offline state."""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, fields
from typing import Any, Callable

import httpx

log = logging.getLogger("bridge.tool_http")

OnlineCb = Callable[[int, bool], None]
AckCb = Callable[[int, int], None]
StatusCb = Callable[[int, dict], None]


class ToolError(Exception):
    """A request to the tool failed after all retries (transport error, timeout or non-2xx)."""


@dataclass(frozen=True)
class Timings:
    heartbeat_ms: int = 200
    request_timeout_ms: int = 500
    retries: int = 3
    backoff_ms: int = 100
    offline_after_ms: int = 1000

    @classmethod
    def from_dict(cls, d: dict[str, Any] | None) -> "Timings":
        names = {f.name for f in fields(cls)}
        return cls(**{k: int(v) for k, v in (d or {}).items() if k in names})


class ToolHttp:
    """HTTP leg to one tool. Ops are serialized per tool (seq order); heartbeat runs as a task."""

    def __init__(
        self,
        tool_id: int,
        base_url: str,
        timings: Timings | None = None,
        *,
        client: httpx.AsyncClient | None = None,
        on_online: OnlineCb | None = None,
        on_ack: AckCb | None = None,
        on_status: StatusCb | None = None,
    ) -> None:
        self.tool_id = int(tool_id)
        if not base_url.startswith("http"):
            base_url = "http://" + base_url
        self.base_url = base_url.rstrip("/")
        self.t = timings or Timings()
        self._owns_client = client is None
        self._client = client or httpx.AsyncClient(
            base_url=self.base_url, timeout=self.t.request_timeout_ms / 1000
        )
        self._on_online, self._on_ack, self._on_status = on_online, on_ack, on_status
        self.online = False
        self.last_seq = 0
        self.last_status: dict | None = None
        self._last_success: float | None = None
        self._hb_task: asyncio.Task | None = None
        self._hb_count = 0
        self._op_lock = asyncio.Lock()

    # ---- lifecycle -------------------------------------------------------------------------
    async def start(self) -> None:
        if self._on_online:
            self._on_online(self.tool_id, False)
        self._hb_task = asyncio.create_task(self._hb_loop(), name=f"tool{self.tool_id}-heartbeat")

    async def stop(self) -> None:
        if self._hb_task:
            self._hb_task.cancel()
            try:
                await self._hb_task
            except (asyncio.CancelledError, Exception):
                pass
            self._hb_task = None
        if self._owns_client:
            await self._client.aclose()

    # ---- online state machine --------------------------------------------------------------
    def _set_online(self, value: bool) -> None:
        if value != self.online:
            self.online = value
            log.info("tool %d %s (%s)", self.tool_id, "ONLINE" if value else "OFFLINE", self.base_url)
            if self._on_online:
                self._on_online(self.tool_id, value)

    def _note_success(self) -> None:
        self._last_success = time.monotonic()
        self._set_online(True)

    def _note_failure(self) -> None:
        now = time.monotonic()
        if self._last_success is None:
            return  # never reached: stays offline
        if (now - self._last_success) * 1000 >= self.t.offline_after_ms:
            self._set_online(False)

    # ---- HTTP ------------------------------------------------------------------------------
    async def _once(self, method: str, path: str, json: dict | None = None) -> dict:
        r = await self._client.request(
            method, path, json=json, timeout=self.t.request_timeout_ms / 1000
        )
        if r.status_code // 100 != 2:
            raise ToolError(f"{method} {path} -> HTTP {r.status_code}: {r.text[:120]}")
        data = r.json()
        return data if isinstance(data, dict) else {"value": data}

    async def request(self, method: str, path: str, json: dict | None = None) -> dict:
        """One attempt + `retries` retries with `backoff_ms` between; raises ToolError at the end."""
        last: Exception | None = None
        for attempt in range(self.t.retries + 1):
            try:
                data = await self._once(method, path, json)
                self._note_success()
                return data
            except Exception as e:  # httpx errors, ASGI app exceptions, ToolError (non-2xx)
                last = e
                self._note_failure()
                log.debug("tool %d %s %s attempt %d failed: %s", self.tool_id, method, path, attempt + 1, e)
                if attempt < self.t.retries:
                    await asyncio.sleep(self.t.backoff_ms / 1000)
        raise ToolError(f"tool {self.tool_id}: {method} {path} failed after {self.t.retries + 1} attempts: {last}") from last

    async def _hb_loop(self) -> None:
        period = self.t.heartbeat_ms / 1000
        next_t = time.monotonic()
        while True:
            try:
                await self._once("POST", "/heartbeat", {"t": int(time.time() * 1000)})
                self._note_success()
                self._hb_count += 1
                if self._hb_count % 5 == 1:  # ~1 Hz status mirror on top of the after-ack refresh
                    await self.refresh_status()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._note_failure()
                log.debug("tool %d heartbeat failed: %s", self.tool_id, e)
            next_t = max(next_t + period, time.monotonic())
            await asyncio.sleep(max(0.0, next_t - time.monotonic()))

    async def refresh_status(self) -> dict | None:
        try:
            st = await self._once("GET", "/status")
        except Exception as e:
            self._note_failure()
            log.debug("tool %d status failed: %s", self.tool_id, e)
            return None
        self._note_success()
        self._publish_status(st)
        return st

    def _publish_status(self, st: dict) -> None:
        # C.1 mirror: tool /status fields verbatim + {"seq": lastSeq, "ok": true}
        mirrored = {**st, "seq": int(st.get("lastSeq", self.last_seq)), "ok": True}
        self.last_status = mirrored
        if self._on_status:
            self._on_status(self.tool_id, mirrored)

    # ---- op dispatch (C.1 /subzero/tool/cmd -> C.2 routes) ---------------------------------
    async def op(self, cmd: dict) -> bool:
        """Execute one /subzero/tool/cmd dict. Returns True when the tool acked (2xx)."""
        try:
            seq = int(cmd.get("seq", 0))
        except (TypeError, ValueError):
            log.warning("tool %d: bad seq in %r", self.tool_id, cmd)
            return False
        op = str(cmd.get("op", ""))
        arg = cmd.get("arg", 0)
        async with self._op_lock:
            if op in ("latch", "release", "lateral") and seq <= self.last_seq:
                log.info("tool %d: ignoring stale seq %d <= lastSeq %d (%s)", self.tool_id, seq, self.last_seq, op)
                return False
            try:
                if op in ("latch", "release"):
                    data = await self.request("POST", f"/{op}", {"seq": seq})
                elif op == "lateral":
                    data = await self.request("POST", "/lateral", {"seq": seq, "mm": float(arg or 0)})
                elif op == "estop":
                    data = await self.request("POST", "/estop")
                elif op == "ping":
                    data = await self.request("GET", "/status")
                else:
                    log.warning("tool %d: unknown op %r (seq %d)", self.tool_id, op, seq)
                    return False
            except ToolError as e:
                log.error("tool %d: %s seq %d FAILED: %s", self.tool_id, op, seq, e)
                return False
            if data.get("ok") is False:
                log.warning("tool %d: %s seq %d rejected: %s", self.tool_id, op, seq, data)
                return False
            self.last_seq = max(self.last_seq, seq)
            log.info("tool %d: %s seq %d acked%s", self.tool_id, op, seq, " (dup)" if data.get("dup") else "")
            if self._on_ack:
                self._on_ack(self.tool_id, self.last_seq)
            if op == "ping":
                self._publish_status(data)
            else:
                await self.refresh_status()
            return True

    async def estop(self) -> bool:
        """Forward /subzero/estop: POST /estop (idempotent). Never deduped."""
        try:
            await self.request("POST", "/estop")
            log.warning("tool %d: ESTOP sent", self.tool_id)
            await self.refresh_status()
            return True
        except ToolError as e:
            log.error("tool %d: ESTOP FAILED: %s", self.tool_id, e)
            return False
