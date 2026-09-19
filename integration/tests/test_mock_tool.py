"""Contract C.2 conformance for mock_tool.py: every route, every Δ rule, LOST_LINK, ESTOP/re-attach."""
from __future__ import annotations

import time

import httpx
import pytest
from httpx import ASGITransport

from mock_tool import LATERAL_MAX_MM, create_app

STATUS_KEYS = {"tool", "latched", "lateral_mm", "rssi", "uptime_s", "lastSeq", "state"}


@pytest.fixture
async def tool():
    app = create_app(tool_id=2)
    async with httpx.AsyncClient(transport=ASGITransport(app=app), base_url="http://tool2") as c:
        c.app = app  # type: ignore[attr-defined]
        yield c


def _json(r: httpx.Response) -> dict:
    assert r.headers["content-type"].startswith("application/json"), r.headers
    return r.json()


async def test_status_shape(tool):
    r = await tool.get("/status")
    assert r.status_code == 200
    st = _json(r)
    assert set(st) == STATUS_KEYS
    assert st["tool"] == 2 and st["latched"] is False and st["lateral_mm"] == 0.0
    assert st["lastSeq"] == 0 and st["state"] == "OK"
    assert isinstance(st["rssi"], int) and isinstance(st["uptime_s"], int)


async def test_latch_release_roundtrip(tool):
    assert _json(await tool.post("/latch", json={"seq": 1})) == {"ok": True, "seq": 1}
    assert _json(await tool.get("/status"))["latched"] is True
    assert _json(await tool.post("/release", json={"seq": 2})) == {"ok": True, "seq": 2}
    st = _json(await tool.get("/status"))
    assert st["latched"] is False and st["lastSeq"] == 2


@pytest.mark.parametrize("mm,expect", [(12.5, 12.5), (999, LATERAL_MAX_MM), (-3, 0.0), (0, 0.0), (40, 40.0)])
async def test_lateral_clamps_and_echoes(tool, mm, expect):
    r = await tool.post("/lateral", json={"seq": 5, "mm": mm})
    assert r.status_code == 200
    assert _json(r) == {"ok": True, "seq": 5, "lateral_mm": expect}
    assert _json(await tool.get("/status"))["lateral_mm"] == expect


async def test_heartbeat(tool):
    r = await tool.post("/heartbeat", json={"t": int(time.time() * 1000)})
    assert r.status_code == 200 and _json(r) == {"ok": True}


@pytest.mark.parametrize("path", ["/latch", "/release", "/lateral", "/heartbeat"])
@pytest.mark.parametrize(
    "content", [b"not json", b"", b"[1,2]", b'{"nope":1}', b'{"seq":"7"}', b'{"seq":true}', b'{"t":"x"}']
)
async def test_bad_json_is_400(tool, path, content):
    r = await tool.post(path, content=content, headers={"content-type": "application/json"})
    assert r.status_code == 400
    assert _json(r) == {"ok": False, "seq": 0, "err": "bad_json"}


async def test_lateral_missing_mm_is_bad_json(tool):
    r = await tool.post("/lateral", json={"seq": 3})
    assert r.status_code == 400 and _json(r)["err"] == "bad_json"


async def test_dup_seq_acks_without_reactuating(tool):
    assert _json(await tool.post("/latch", json={"seq": 10})) == {"ok": True, "seq": 10}
    n = tool.app.state.actuations
    assert _json(await tool.post("/release", json={"seq": 10})) == {"ok": True, "seq": 10, "dup": True}
    assert _json(await tool.post("/release", json={"seq": 9})) == {"ok": True, "seq": 9, "dup": True}
    assert _json(await tool.post("/lateral", json={"seq": 10, "mm": 30})) == {"ok": True, "seq": 10, "dup": True}
    st = _json(await tool.get("/status"))
    assert st["latched"] is True and st["lateral_mm"] == 0.0 and st["lastSeq"] == 10
    assert tool.app.state.actuations == n


async def test_lost_link_after_1000ms_holds_targets(tool):
    assert _json(await tool.post("/lateral", json={"seq": 1, "mm": 20})) ["lateral_mm"] == 20.0
    await tool.post("/heartbeat", json={"t": 1})
    assert _json(await tool.get("/status"))["state"] == "OK"
    tool.app.state.last_heartbeat = time.monotonic() - 0.9
    assert _json(await tool.get("/status"))["state"] == "OK", "999 ms silence is still OK"
    tool.app.state.last_heartbeat = time.monotonic() - 1.001
    st = _json(await tool.get("/status"))
    assert st["state"] == "LOST_LINK"
    assert st["lateral_mm"] == 20.0 and tool.app.state.servos_attached is True, "hold, never detach"
    await tool.post("/heartbeat", json={"t": 2})
    assert _json(await tool.get("/status"))["state"] == "OK", "heartbeat resumes -> OK"


async def test_lost_link_real_time(tool):
    """Real wall-clock check of the 1000 ms rule (single test, ~1.1 s)."""
    await tool.post("/heartbeat", json={"t": 1})
    t0 = time.perf_counter()
    while _json(await tool.get("/status"))["state"] == "OK":
        assert time.perf_counter() - t0 < 1.5
        time.sleep(0.02)
    dt = time.perf_counter() - t0
    print(f"\nMOCK LOST_LINK after {dt*1000:.0f} ms of heartbeat silence")
    assert 1.0 <= dt < 1.3


async def test_estop_detaches_then_actuate_reattaches(tool):
    await tool.post("/latch", json={"seq": 1})
    r = await tool.post("/estop")
    assert r.status_code == 200 and _json(r) == {"ok": True}
    st = _json(await tool.get("/status"))
    assert st["state"] == "ESTOP" and tool.app.state.servos_attached is False
    await tool.post("/heartbeat", json={"t": 5})
    assert _json(await tool.get("/status"))["state"] == "ESTOP", "heartbeat does not clear ESTOP"
    assert _json(await tool.post("/estop")) == {"ok": True}, "estop is idempotent"
    assert _json(await tool.post("/latch", json={"seq": 1})) == {"ok": True, "seq": 1, "dup": True}
    assert _json(await tool.get("/status"))["state"] == "ESTOP", "a dup does not re-attach"
    assert _json(await tool.post("/release", json={"seq": 2})) == {"ok": True, "seq": 2}
    st = _json(await tool.get("/status"))
    assert st["state"] == "OK" and st["latched"] is False and tool.app.state.servos_attached is True


async def test_every_response_is_json_even_404_405(tool):
    for r in (await tool.get("/nope"), await tool.get("/latch")):
        assert r.headers["content-type"].startswith("application/json")


async def test_drop_requests_simulates_link_loss(tool):
    tool.app.state.drop_requests = True
    with pytest.raises(Exception):
        await tool.get("/status")
    tool.app.state.drop_requests = False
    assert (await tool.get("/status")).status_code == 200
