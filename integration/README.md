# integration/ — Subzero bridge, mock tool, HRI CLI

Laptop-side glue for contracts v0 (`docs/contracts.md` §C.1 NT topics, §C.2 ESP32 HTTP, §C.3 primitives).
The bridge is an **NT4 client** (pyntcore, string-JSON only — A-03) and an **HTTP client** to each ESP32 tool.

```
bridge/          NtClient (NT4 leg) · ToolHttp (HTTP leg + heartbeat + online/offline) · Bridge (wiring) · config.yaml
mock_tool.py     FastAPI mock of one ESP32 tool, implements §C.2 byte-exactly (create_app(tool_id) for tests)
hri_cli.py       keyboard -> /subzero/task/request, prints /subzero/task/state
tests/           pytest (in-process NT4 server + bridge + mock; no hardware, no robot sim needed)
```

## Setup (Python >= 3.11 via uv; never the system python3)
```sh
cd integration
uv sync                      # pins 3.12; installs robotpy (-> pyntcore), httpx, fastapi, uvicorn, pyyaml, pytest
```

## Run
```sh
uv run python mock_tool.py --port 8031 --tool 1                 # terminal 1: a fake tool on 127.0.0.1:8031
uv run python -m bridge --server 127.0.0.1 --profile mock       # terminal 2: bridge -> WPILib sim NT (localhost:5810)
uv run python hri_cli.py --server 127.0.0.1                     # terminal 3: a/e/r/l/u/s/x keys, q quits
```
The bridge logs `waiting for NT server <host>:5810` every 2 s until the robot (sim or RoboRIO) is up and keeps running.
Ctrl-C shuts it down cleanly. `-v` for per-request debug.

- **Robot sim**: `./gradlew simulateJava` in `frc/` exposes NT4 on `localhost:5810` -> `--server 127.0.0.1`.
- **Real RoboRIO**: laptop on the robot radio (H-13), then `--server 10.13.60.2 --profile real`
  (tools `10.13.60.31/.32/.33` in `bridge/config.yaml`, `# TODO(hardware) H-13`, only ids listed are polled — H-09).
- `--config` points at another YAML; `--port` overrides the NT4 port (0 = 5810).

## Test (the proof)
```sh
uv run pytest -q          # add -s to see the measured round-trip / link-loss timings
```
- `tests/test_mock_tool.py` — every §C.2 route and Δ rule (400 bad_json, clamp+echo, dup seq, LOST_LINK after 1000 ms, ESTOP detach / re-attach, JSON on every response).
- `tests/test_bridge_roundtrip.py` — in-process NT4 server + bridge + mock via ASGITransport: `/subzero/tool/cmd` -> `/subzero/tool/1/lastSeq` in < 300 ms, status mirror, clamp, stale-seq ignore, 3-command burst, `/subzero/estop` fan-out, `bridge/alive` toggling.
- `tests/test_link_loss.py` — `online` false within 1.5 s of cutting the mock, true again after restore.

## Behaviour notes
- `seq` is the ordering key; the bridge ignores `latch/release/lateral` with `seq <= lastSeq` (estop/ping never deduped) and publishes `lastSeq` only after a 2xx ack, then refreshes `/subzero/tool/<n>/status` (tool `/status` fields + `seq`/`ok`).
- Heartbeat `POST /heartbeat {"t": ms}` every 200 ms per tool; request timeout 500 ms; ops retry 3x with 100 ms backoff; `online` drops once failures span >= 1000 ms.
- `/subzero/estop == true` -> `POST /estop` to every configured tool (tools detach; link loss holds — gate Q2 / H-11).
- Command topics are published with `keepDuplicates=True, periodic=0.02` **plus `sendAll=True`** and read with `readQueue()` + `pollStorage=64`: without `sendAll`, ntcore coalesces two values inside one 20 ms window, which would drop a command.
