"""NT4 server + bridge + mock tool round trip: /subzero/tool/cmd -> HTTP -> /subzero/tool/1/lastSeq|status."""
from __future__ import annotations

import json
import time

from tests.nt_util import wait_for


async def test_latch_roundtrip_under_300ms(harness):
    h = harness
    assert h.last_seq.get() in (-1, 0)
    t0 = time.perf_counter()
    h.send(1, "latch")
    dt = await wait_for(lambda: h.last_seq.get() >= 1, 1.0, "lastSeq >= 1")
    print(f"\nROUNDTRIP cmd(seq=1 latch) -> /subzero/tool/1/lastSeq==1: {dt*1000:.1f} ms")
    assert dt < 0.3, f"round trip took {dt*1000:.0f} ms"
    assert h.last_seq.get() == 1
    dt2 = await wait_for(lambda: h.status_dict().get("latched") is True, 1.0, "status latched")
    print(f"ROUNDTRIP status mirror latched=true: +{dt2*1000:.1f} ms")
    st = h.status_dict()
    assert st["latched"] is True and st["seq"] == 1 and st["ok"] is True and st["lastSeq"] == 1
    assert {"tool", "latched", "lateral_mm", "rssi", "uptime_s", "lastSeq", "state"} <= set(st)
    assert h.app.state.latched is True and h.app.state.lastSeq == 1


async def test_lateral_clamp_dup_and_burst(harness):
    h = harness
    h.send(1, "latch")
    await wait_for(lambda: h.last_seq.get() >= 1, 1.0)
    h.send(2, "lateral", 55.0)  # clamped to 40 by the tool, echoed back into the status mirror
    await wait_for(lambda: h.last_seq.get() >= 2, 1.0)
    await wait_for(lambda: h.status_dict().get("lateral_mm") == 40.0, 1.0, "lateral_mm==40")
    n = h.app.state.actuations
    h.send(1, "release")  # stale seq: bridge ignores (consumers ignore seq <= lastSeen)
    # burst: three commands inside one 20 ms periodic window must all arrive (sendAll + keepDuplicates)
    h.send(3, "lateral", 10.0)
    h.send(4, "lateral", 10.0)
    h.send(5, "release")
    await wait_for(lambda: h.last_seq.get() >= 5, 1.5, "lastSeq >= 5")
    await wait_for(lambda: h.status_dict().get("latched") is False, 1.0)
    assert h.app.state.actuations == n + 3 and h.app.state.latched is False and h.app.state.lastSeq == 5
    assert h.bridge.stats["cmds"] == 6 and h.bridge.stats["acks"] == 5


async def test_ping_and_unknown_tool(harness):
    h = harness
    h.send(1, "ping")
    await wait_for(lambda: h.last_seq.get() >= 1, 1.0)
    assert h.status_dict()["state"] == "OK"
    h.send(2, "latch", tool=9)  # unconfigured (H-09): ignored, never crashes the bridge
    await wait_for(lambda: h.bridge.stats["cmds"] >= 2, 1.0)
    assert h.last_seq.get() == 1 and h.app.state.latched is False


async def test_estop_topic_forwards_to_every_tool_and_alive_toggles(harness):
    h = harness
    h.send(1, "latch")
    await wait_for(lambda: h.last_seq.get() >= 1, 1.0)
    first = h.alive.get()
    await wait_for(lambda: h.alive.get() != first, 1.0, "alive toggles")
    h.estop_pub.set(True)
    dt = await wait_for(lambda: h.app.state.state == "ESTOP", 1.0, "mock in ESTOP")
    print(f"\nESTOP /subzero/estop -> POST /estop: {dt*1000:.1f} ms")
    assert h.app.state.servos_attached is False
    await wait_for(lambda: h.status_dict().get("state") == "ESTOP", 1.0, "status mirror ESTOP")
    h.send(2, "estop")  # estop op via /subzero/tool/cmd is acked too (idempotent)
    await wait_for(lambda: h.last_seq.get() >= 2, 1.0)
    h.send(3, "release")  # next actuate re-attaches
    await wait_for(lambda: h.app.state.state == "OK", 1.0)
    assert h.app.state.servos_attached is True
