## C. Contracts frozen before fan-out
Copied from `00-brief/comms-architecture.md §2–4` (v0). REPO POP copies this section verbatim into `docs/contracts.md` at root step 1. Fields marked **Δ** are additions; nothing was removed or renamed.

### C.1 NetworkTables topic contract (v0)
All command payloads are **JSON strings**. Numeric telemetry uses typed topics.

| Topic | Type | Publisher → Subscriber | Payload / semantics |
|---|---|---|---|
| `/subzero/tool/cmd` | string (JSON) | RoboRIO → bridge | `{"seq":123,"tool":1,"op":"latch"\|"release"\|"lateral"\|"estop"\|"ping","arg":0.0}` — `arg` = mm for `lateral`. Publish with `keepDuplicates=true`. |
| `/subzero/tool/<n>/status` | string (JSON) | bridge → RoboRIO | `{"seq":123,"ok":true,"latched":true,"lateral_mm":12.5,"rssi":-61,"uptime_s":88}` mirrors the tool's `/status` |
| `/subzero/tool/<n>/online` | boolean | bridge → RoboRIO | true while heartbeats succeed |
| `/subzero/tool/<n>/lastSeq` | integer | bridge → RoboRIO | last `seq` acknowledged by tool n (RoboRIO waits for `lastSeq >= seq` or timeout) |
| `/subzero/bridge/alive` | boolean | bridge → all | bridge heartbeat (toggle or timestamp) |
| `/subzero/task/request` | string (JSON) | integration layer → RoboRIO | `{"seq":7,"primitive":"alignToTag","args":{"tagId":3,"offsetX_m":0.45,"offsetY_m":0.0}}` — see C.3 |
| `/subzero/task/state` | string (JSON) | RoboRIO → all | `{"seq":7,"primitive":"alignToTag","state":"RUNNING"\|"DONE"\|"FAILED"\|"ABORTED","detail":"..."}` |
| `/subzero/robot/pose` | struct `Pose2d` | RoboRIO → all | fused estimate (also in AK log) — **the Python bridge does not subscribe to this in v0** (string-JSON only) |
| `/subzero/elevator/height_m`, `/subzero/arm/extension_m` | double | RoboRIO → all | telemetry |
| `/subzero/estop` | boolean | anyone → RoboRIO + bridge | soft e-stop: RoboRIO cancels all commands; bridge sends `/estop` to every tool |
| **Δ** `/subzero/robot/homed` | boolean | RoboRIO → all | true once elevator **and** arm have completed homing since power-on; `elevatorTo`/`armTo`/composites refuse to run while false |

Rules: `seq` is monotonically increasing per publisher; consumers ignore `seq <= lastSeen`; the RoboRIO treats a tool command as failed after **1.5 s** without ack; `estop` is idempotent. **Δ** Publishers of `/subzero/tool/cmd`, `/subzero/task/request` and `/subzero/task/state` use `keepDuplicates=true` and `periodic=0.02`; subscribers read with `readQueue()` so no command is collapsed.

### C.2 ESP32 HTTP API (v0) — each tool serves this on port 80
| Method | Path | Body (JSON) | Response (JSON) | Semantics |
|---|---|---|---|---|
| GET | `/status` | — | `{"tool":1,"latched":bool,"lateral_mm":f,"rssi":i,"uptime_s":i,"lastSeq":i,"state":"OK"\|"LOST_LINK"\|"ESTOP"}` | health + state |
| POST | `/latch` | `{"seq":i}` | `{"ok":true,"seq":i}` | close latch servo to `LATCH_CLOSED_DEG` |
| POST | `/release` | `{"seq":i}` | `{"ok":true,"seq":i}` | open latch servo |
| POST | `/lateral` | `{"seq":i,"mm":f}` | `{"ok":true,"seq":i,"lateral_mm":f}` | move lateral servo; clamp to `[0, LATERAL_MAX_MM]` |
| POST | `/heartbeat` | `{"t":i}` | `{"ok":true}` | bridge sends every **200 ms**; tool enters `LOST_LINK` after **1000 ms** silence (holds position) |
| POST | `/estop` | — | `{"ok":true}` | **Decided at the gate (H-11):** both servos **detach** (PWM off, go limp), `state="ESTOP"`; the next `/latch`, `/release` or `/lateral` re-attaches and clears it. **Link loss (1000 ms no heartbeat) holds** the last targets (`state="LOST_LINK"`), never detaches. `ESTOP_BEHAVIOUR` is one constant in `config.h` in case hardware says a limp latch drops the tool |
Timeouts: client 500 ms per request, 3 retries with 100 ms backoff. Tool marks a `seq` as done only after the servo target is set (no position feedback on hobby servos).
**Δ** Decided (were open in `10-research/esp32-platformio-arduino.md` L295): malformed/missing JSON → HTTP `400` `{"ok":false,"seq":0,"err":"bad_json"}`; out-of-range `mm` is **clamped** (never rejected) and the clamped value is returned; no authentication (same closed robot WiFi); every response carries `Content-Type: application/json`; a `seq` ≤ `lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` without re-actuating.

### C.3 Task primitives (v0)
Implemented on the RoboRIO as command factories; triggered from NT (`/subzero/task/request`) *or* controller buttons for MVP.

| Primitive | Args | Done when |
|---|---|---|
| `alignToTag` | `tagId`, `offsetX_m`, `offsetY_m`, `yaw_deg` (robot pose relative to tag) | pose error < 3 cm / 2° for 0.3 s (port of `AlignToTagPoseCommand`); refuses if tag unseen > 0.5 s or pose jump > 1 m; timeout 8 s (`safety.md §7`) |
| `elevatorTo` | `height_m` or `setpoint` name | within tolerance for 0.2 s |
| `armTo` | `extension_m` or `setpoint` name | within tolerance |
| `toolOp` | `tool`, `op`, `arg` | `lastSeq` ack or timeout (1.5 s → `FAILED`) |
| `swapTool` | `tool` | composite: retract → slot(current) → release → slot(target) → latch → retract |
| `pickAt` | `locationTagId`, `objectTagId` (**ignored this weekend — gate Q4**) | composite: alignToTag(location, fixed pick offset) → elevator/arm to grasp pose (H-03/H-07) → latch → retract. Object-tag fine align is a stretch step inserted after `alignToTag` |
| `placeAt` | `locationTagId` | composite: alignToTag → elevator/arm to place pose → release → retract |
| `abort` | — | all subsystems to safe hold; `state=ABORTED` |
MVP scope (Sat night): `alignToTag`, `elevatorTo`, `armTo`, `toolOp`. Composites after. **Gate Q4:** no object-tag alignment this weekend — every pose is a location-tag offset + elevator/arm setpoint. **Δ** Every primitive is interruptible by drive-stick input and by `/subzero/estop`; every composite starts and ends at **stow** (elevator down, arm retracted) per `safety.md §6`.

