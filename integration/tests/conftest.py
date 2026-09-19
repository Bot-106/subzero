from __future__ import annotations

import asyncio
import contextlib
import json
import sys
from pathlib import Path

import httpx
import ntcore
import pytest
from httpx import ASGITransport

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # integration/ on sys.path

from bridge import Bridge, NtClient, Timings  # noqa: E402
from bridge.nt_client import cmd_pub_options, cmd_sub_options, fast_pub_options  # noqa: E402
from mock_tool import create_app  # noqa: E402
from tests.nt_util import NtServer, destroy, make_client, wait_for  # noqa: E402


@pytest.fixture
def nt_server(tmp_path):
    srv = NtServer(tmp_path)
    yield srv
    srv.stop()


class Harness:
    """NT server + bridge (against the mock app via ASGITransport) + a test-side NT peer."""

    def __init__(self, nt_server: NtServer, tool_id: int = 1):
        self.tool_id = tool_id
        self.app = create_app(tool_id)
        self.http = httpx.AsyncClient(transport=ASGITransport(app=self.app), base_url=f"http://tool{tool_id}")
        self.bridge_inst = ntcore.NetworkTableInstance.create()
        self.nt = NtClient("127.0.0.1", nt_server.port, identity="subzero-bridge-test", inst=self.bridge_inst)
        self.bridge = Bridge(self.nt, {tool_id: f"tool{tool_id}"}, Timings(), clients={tool_id: self.http})
        self.peer = make_client(nt_server.port, "test-peer")
        self.cmd_pub = self.peer.getStringTopic("/subzero/tool/cmd").publish(cmd_pub_options())
        self.estop_pub = self.peer.getBooleanTopic("/subzero/estop").publish(cmd_pub_options())
        self.last_seq = self.peer.getIntegerTopic(f"/subzero/tool/{tool_id}/lastSeq").subscribe(-1, cmd_sub_options())
        self.online = self.peer.getBooleanTopic(f"/subzero/tool/{tool_id}/online").subscribe(False, cmd_sub_options())
        self.status = self.peer.getStringTopic(f"/subzero/tool/{tool_id}/status").subscribe("", cmd_sub_options())
        self.alive = self.peer.getBooleanTopic("/subzero/bridge/alive").subscribe(False, cmd_sub_options())
        self.task: asyncio.Task | None = None

    async def start(self) -> None:
        self.task = asyncio.create_task(self.bridge.run(), name="bridge")
        await wait_for(lambda: self.peer.isConnected() and self.bridge_inst.isConnected(), 3.0, "NT clients connected")
        await wait_for(lambda: self.online.get() is True, 3.0, "tool online")

    def send(self, seq: int, op: str, arg: float = 0, tool: int | None = None) -> None:
        self.cmd_pub.set(json.dumps({"seq": seq, "tool": self.tool_id if tool is None else tool, "op": op, "arg": arg}))

    def status_dict(self) -> dict:
        raw = self.status.get()
        return json.loads(raw) if raw else {}

    async def stop(self) -> None:
        if self.task:
            self.task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self.task
        await self.http.aclose()
        destroy(self.peer)
        ntcore.NetworkTableInstance.destroy(self.bridge_inst)


@pytest.fixture
async def harness(nt_server):
    h = Harness(nt_server)
    await h.start()
    yield h
    await h.stop()
