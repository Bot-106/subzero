"""NtClient: the bridge's NT4 client side (contract C.1, string-JSON only per A-03)."""
from __future__ import annotations

import json
import logging

import ntcore

log = logging.getLogger("bridge.nt")

TOPIC_CMD = "/subzero/tool/cmd"
TOPIC_ESTOP = "/subzero/estop"
TOPIC_ALIVE = "/subzero/bridge/alive"


def topic_status(n: int) -> str:
    return f"/subzero/tool/{n}/status"


def topic_online(n: int) -> str:
    return f"/subzero/tool/{n}/online"


def topic_last_seq(n: int) -> str:
    return f"/subzero/tool/{n}/lastSeq"


def cmd_pub_options() -> ntcore.PubSubOptions:
    """C.1: command publishers use keepDuplicates=true, periodic=0.02; sendAll stops in-window coalescing."""
    return ntcore.PubSubOptions(keepDuplicates=True, periodic=0.02, sendAll=True)


def cmd_sub_options() -> ntcore.PubSubOptions:
    """Subscriber side for readQueue(): keep every value (pollStorage) so no command is collapsed."""
    return ntcore.PubSubOptions(keepDuplicates=True, periodic=0.02, sendAll=True, pollStorage=64)


def fast_pub_options() -> ntcore.PubSubOptions:
    return ntcore.PubSubOptions(periodic=0.02)


class NtClient:
    def __init__(
        self,
        server: str = "127.0.0.1",
        port: int = 0,
        identity: str = "subzero-bridge",
        inst: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        self.server, self.port, self.identity = server, int(port), identity
        self.inst = inst or ntcore.NetworkTableInstance.getDefault()
        self._cmd_sub = None
        self._estop_sub = None
        self._alive_pub = None
        self._status_pubs: dict[int, ntcore.StringPublisher] = {}
        self._online_pubs: dict[int, ntcore.BooleanPublisher] = {}
        self._seq_pubs: dict[int, ntcore.IntegerPublisher] = {}
        self._alive = False

    def connect(self) -> None:
        self.inst.startClient4(self.identity)
        self.inst.setServer(self.server, self.port)  # port 0 = NT4 default 5810
        self._cmd_sub = self.inst.getStringTopic(TOPIC_CMD).subscribe("", cmd_sub_options())
        self._estop_sub = self.inst.getBooleanTopic(TOPIC_ESTOP).subscribe(False, cmd_sub_options())
        self._alive_pub = self.inst.getBooleanTopic(TOPIC_ALIVE).publish(fast_pub_options())
        log.info("NT4 client '%s' -> %s:%s", self.identity, self.server, self.port or 5810)

    def close(self) -> None:
        try:
            self.inst.stopClient()
        except Exception:  # pragma: no cover
            pass

    def is_connected(self) -> bool:
        return self.inst.isConnected()

    # ---- inbound -----------------------------------------------------------------------------
    def read_cmds(self) -> list[dict]:
        """Drain /subzero/tool/cmd via readQueue() (never .get()) -> parsed dicts, bad JSON skipped."""
        out: list[dict] = []
        if self._cmd_sub is None:
            return out
        for ts in self._cmd_sub.readQueue():
            try:
                d = json.loads(ts.value)
                if not isinstance(d, dict):
                    raise ValueError("not an object")
                out.append(d)
            except Exception as e:
                log.warning("bad /subzero/tool/cmd JSON %r: %s", ts.value[:200], e)
        return out

    def read_estop(self) -> list[bool]:
        if self._estop_sub is None:
            return []
        return [bool(ts.value) for ts in self._estop_sub.readQueue()]

    # ---- outbound ----------------------------------------------------------------------------
    def publish_status(self, tool: int, status: dict) -> None:
        pub = self._status_pubs.get(tool)
        if pub is None:
            pub = self._status_pubs[tool] = self.inst.getStringTopic(topic_status(tool)).publish(fast_pub_options())
        pub.set(json.dumps(status, separators=(",", ":")))

    def publish_online(self, tool: int, online: bool) -> None:
        pub = self._online_pubs.get(tool)
        if pub is None:
            pub = self._online_pubs[tool] = self.inst.getBooleanTopic(topic_online(tool)).publish(fast_pub_options())
        pub.set(bool(online))

    def publish_last_seq(self, tool: int, seq: int) -> None:
        pub = self._seq_pubs.get(tool)
        if pub is None:
            pub = self._seq_pubs[tool] = self.inst.getIntegerTopic(topic_last_seq(tool)).publish(fast_pub_options())
        pub.set(int(seq))

    def toggle_alive(self) -> None:
        self._alive = not self._alive
        if self._alive_pub is not None:
            self._alive_pub.set(self._alive)
