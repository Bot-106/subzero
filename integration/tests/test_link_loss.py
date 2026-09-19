"""Tool online -> cut the mock -> /subzero/tool/1/online False within 1.5 s -> restore -> True again."""
from __future__ import annotations

import time

from tests.nt_util import wait_for


async def test_online_false_within_1500ms_then_recovers(harness):
    h = harness
    assert h.online.get() is True
    t0 = time.perf_counter()
    h.app.state.drop_requests = True
    dt = await wait_for(lambda: h.online.get() is False, 2.0, "online False")
    print(f"\nLINK LOSS: online=false after {dt*1000:.0f} ms (limit 1500)")
    assert dt < 1.5
    assert h.bridge.tools[1].online is False
    h.app.state.drop_requests = False
    dt2 = await wait_for(lambda: h.online.get() is True, 1.0, "online True")
    print(f"LINK RESTORE: online=true after {dt2*1000:.0f} ms")
    assert dt2 < 0.5


async def test_command_during_link_loss_fails_without_ack_then_works(harness):
    h = harness
    h.app.state.drop_requests = True
    await wait_for(lambda: h.online.get() is False, 2.0)
    h.send(1, "latch")
    await wait_for(lambda: h.bridge.stats["cmds"] >= 1, 1.0)
    # 4 attempts x 100 ms backoff -> the op gives up quickly with no ack (RoboRIO times out at 1.5 s)
    await wait_for(lambda: h.bridge.stats["acks"] == 0 and not any(t.get_name().startswith("op-") for t in h.bridge._tasks), 2.0, "op gave up")
    assert h.last_seq.get() in (-1, 0)
    h.app.state.drop_requests = False
    await wait_for(lambda: h.online.get() is True, 1.0)
    h.send(2, "latch")
    await wait_for(lambda: h.last_seq.get() >= 2, 1.0)
    assert h.app.state.latched is True
