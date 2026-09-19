"""Mock ESP32 tool: implements contract C.2 (docs/contracts.md) exactly, for bridge tests with zero hardware.

    uv run python mock_tool.py --port 8031 --tool 1

Tests: `create_app(tool_id)` + httpx ASGITransport; set `app.state.drop_requests = True` to simulate link loss
(every request then raises ConnectionError / returns 500 under uvicorn).
"""
from __future__ import annotations

import argparse
import time

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

LATERAL_MAX_MM = 40.0  # C.2: clamp mm to [0, LATERAL_MAX_MM]
LINK_LOSS_MS = 1000    # C.2: LOST_LINK after 1000 ms without /heartbeat (targets held, never detached)
STATES = ("OK", "LOST_LINK", "ESTOP")


def _bad_json() -> JSONResponse:
    return JSONResponse(status_code=400, content={"ok": False, "seq": 0, "err": "bad_json"})


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


class _DropLink:
    """Pure-ASGI wrapper: when app.state.drop_requests is set, every HTTP request fails."""

    def __init__(self, app, state):
        self.app, self.state = app, state

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http" and getattr(self.state, "drop_requests", False):
            raise ConnectionError("mock tool: link dropped")
        await self.app(scope, receive, send)


def create_app(tool_id: int = 1) -> FastAPI:
    app = FastAPI(title=f"subzero mock tool {tool_id}", docs_url=None, redoc_url=None)
    s = app.state
    s.tool_id = int(tool_id)
    s.latched = False
    s.lateral_mm = 0.0
    s.lastSeq = 0
    s.state = "OK"
    s.servos_attached = True
    s.rssi = -61
    s.boot = time.monotonic()
    s.last_heartbeat = s.boot
    s.drop_requests = False
    s.actuations = 0  # test hook: counts real servo target changes (dups must not bump it)

    def tick() -> None:
        if s.state == "OK" and (time.monotonic() - s.last_heartbeat) * 1000 > LINK_LOSS_MS:
            s.state = "LOST_LINK"  # hold targets; do NOT detach

    async def body_of(request: Request) -> dict | None:
        try:
            body = await request.json()
        except Exception:
            return None
        return body if isinstance(body, dict) else None

    def actuate(seq: int):
        """Common seq/dup/re-attach handling. Returns a dup response or None (caller actuates)."""
        if seq <= s.lastSeq:
            return {"ok": True, "seq": seq, "dup": True}
        s.servos_attached = True  # ESTOP/LOST_LINK cleared by the next actuate (gate Q2 / H-11)
        s.state = "OK"
        s.last_heartbeat = time.monotonic()
        s.lastSeq = seq
        s.actuations += 1
        return None

    @app.get("/status")
    async def status():
        tick()
        return {
            "tool": s.tool_id,
            "latched": s.latched,
            "lateral_mm": float(s.lateral_mm),
            "rssi": s.rssi,
            "uptime_s": int(time.monotonic() - s.boot),
            "lastSeq": s.lastSeq,
            "state": s.state,
        }

    @app.post("/latch")
    async def latch(request: Request):
        body = await body_of(request)
        if body is None or not _is_int(body.get("seq")):
            return _bad_json()
        seq = body["seq"]
        if dup := actuate(seq):
            return dup
        s.latched = True
        return {"ok": True, "seq": seq}

    @app.post("/release")
    async def release(request: Request):
        body = await body_of(request)
        if body is None or not _is_int(body.get("seq")):
            return _bad_json()
        seq = body["seq"]
        if dup := actuate(seq):
            return dup
        s.latched = False
        return {"ok": True, "seq": seq}

    @app.post("/lateral")
    async def lateral(request: Request):
        body = await body_of(request)
        if body is None or not _is_int(body.get("seq")) or not _is_num(body.get("mm")):
            return _bad_json()
        seq = body["seq"]
        if dup := actuate(seq):
            return dup
        s.lateral_mm = max(0.0, min(LATERAL_MAX_MM, float(body["mm"])))  # clamp, never reject
        return {"ok": True, "seq": seq, "lateral_mm": s.lateral_mm}

    @app.post("/heartbeat")
    async def heartbeat(request: Request):
        body = await body_of(request)
        if body is None or not _is_int(body.get("t")):
            return _bad_json()
        tick()
        s.last_heartbeat = time.monotonic()
        if s.state == "LOST_LINK":
            s.state = "OK"  # link back; ESTOP stays until the next actuate
        return {"ok": True}

    @app.post("/estop")
    async def estop():
        s.state = "ESTOP"
        s.servos_attached = False  # H-11 (gate Q2): both servos detach on /estop
        return {"ok": True}

    app.add_middleware(_DropLink, state=s)
    return app


def main(argv=None) -> None:
    import uvicorn

    p = argparse.ArgumentParser(description="Subzero mock ESP32 tool (contract C.2)")
    p.add_argument("--port", type=int, default=8031)
    p.add_argument("--tool", type=int, default=1)
    p.add_argument("--host", default="127.0.0.1")
    a = p.parse_args(argv)
    uvicorn.run(create_app(a.tool), host=a.host, port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
