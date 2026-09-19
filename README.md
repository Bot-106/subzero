# Subzero (Hack the North 2026)

An FRC-style swerve robot with a 1.5 m elevator, an extending arm and WiFi servo tools, aligning to AprilTags in a hackathon room and picking/placing with swappable tools. The monorepo holds the RoboRIO code (`frc/`, WPILib 2026 + Phoenix6 + PhotonLib + AdvantageKit), the ESP32-S3 tool firmware (`esp32/`), the laptop NT4↔HTTP bridge and human-control CLI (`integration/`), the Jetson/PhotonVision runbook (`jetson/`) and the room-layout / hardware docs (`docs/`). See `CLAUDE.md` for how to build, simulate and test each part; `docs/contracts.md` for the frozen NT / HTTP / primitive contracts.

## Status (REPO POP session 2026-09-19 01:57–02:40; agents prove sim only — humans prove hardware)

| Item | Status | Evidence |
|---|---|---|
| `frc/` builds (`cd frc && ./gradlew build`) | **passes** | exit 0 at commit `74667b0` (tag `m1-sim`) |
| Headless sim runs auto-enabled (`SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless`) | **passes** | "Robot program startup complete", no GUI, AK log written |
| Acceptance 1 — sim `alignToTag(3)` < 3 cm / 2° from 1.5 m within 5 s | **passes in sim** | `frc/logs/akit_26-09-19_02-33-08.wpilog`: `Align/error_m` 1.500 → 0.0144, `error_deg` 20.0 → 0.012, `SimSequence/align/seconds` 3.64, `refused` false; start (4.9, 4.35, 110°) → target (4.0, 5.55, 90°) |
| Acceptance 2 — sim `elevatorTo(0.8)`, `armTo(0.3)` within tolerance, no soft-limit crossing; homing sets zero | **passes in sim** | same log: `Elevator/homed` true @1.1 s, `Arm/homed` true @2.0 s (switch path); `SimSequence/elevator/error` 0.0002 m, `maxHeight_m` 0.802 < 1.15; `SimSequence/arm/error` 0.0002 m, `maxExtension_m` 0.300 < 0.43; `Stow` ends at 0.050 / 0.000 m |
| Acceptance 3 — bridge ↔ `mock_tool.py`: ack < 300 ms, link-loss → `online=false` ≤ 1.5 s | **passes** | `cd integration && uv run pytest -q` → `49 passed`; measured round-trip 65.6 ms, link loss 981 ms, ESTOP fan-out 43.9 ms. **Live against the robot sim** (127.0.0.1:5810): `/subzero/task/request` `toolOp latch` → `task/state` RUNNING +58 ms, DONE +86 ms; bridge log `tool 1: latch seq 1 acked`, `/subzero/tool/1/lastSeq` 0 → 1; `elevatorTo 0.5` from rest → DONE +975 ms at 0.5002 m |
| ESP32 firmware (`cd esp32 && pio run -e esp32-s3-devkitc-1`) | **builds** | exit 0; RAM 14.2 % (46516 B), Flash 29.0 % (967806 B); `[env:tool2]` also builds |
| Jetson runbook scripts (`bash -n jetson/gst/*.sh`) | **passes** | exit 0; no pack references |
| Room layout validator (`uv run --project integration python docs/validate_layout.py docs/room-layout.template.json`) | **passes** | exit 0, 4 tags valid; exit 1 on a broken copy |
| M0 on metal — swerve under joystick, elevator homes + holds | **humans** (Fri night) | `docs/hardware-checklist.md` M0 checks |
| Acceptance 4 — ESP32 bench (`curl … /latch` moves the servo; LOST_LINK) | **humans** | `esp32/README.md` curl lines |
| Acceptance 5 — hardware M1 3× from the taped start pose, AK log in AdvantageScope | **humans** (Sat night) | `docs/hardware-checklist.md` |
| M2 — `integration/hri_cli.py` issues primitives | code shipped, **humans** (Sun) | `uv run python hri_cli.py --server 10.13.60.2`; its automated test is a stretch |

Bug fixed at the gate: `Elevator.goTo`/`Arm.goTo` could finish instantly when issued from rest (settle latch computed from the previous setpoint); now the latch resets on initialize (`74667b0`).

## Cuts (A-18 — reopen by a human; item · owner · exact command that would prove it)

| Cut | Owner | Command that would prove it |
|---|---|---|
| JUnit sim tests (`frc/src/test/java`) — proof is the headless `SimSequence` run instead | frc/W2–W4 | `cd frc && ./gradlew test` |
| `AimAtTagCommand` (port of `AimAtTagPoseCommand.java`) | frc/W3 | `./gradlew compileJava` after adding `commands/AimAtTagCommand.java` |
| Composites `swapTool` / `pickAt` / `placeAt` — factories exist and report `FAILED "not implemented"` without moving | frc/W4 | `uv run python hri_cli.py` → `/subzero/task/state` DONE for `pickAt` |
| `util/{PIDLogger,ClosedLoopConstants,TriggerLogger}.java` lifts | frc/W4 | `./gradlew compileJava` |
| Object-tag fine alignment (gate Q4 — out of scope this weekend) | frc/W3 | n/a until H-15 object tags exist |
| `docs/demo-script.md`, `docs/decisions.md`, `jetson/net/` | docs / jetson | n/a (docs) |
| `hri_cli.py` automated test | integration | `cd integration && uv run pytest tests/test_hri_cli.py` |
| `frc-mcp` connection in this session (`.mcp.json` written after start; needs a session restart) — logs were read with a stdlib WPILOG parser instead | root | `/mcp` → `mcp__frc-mcp__read_simulation_log latest` |

Open for humans (values, not code): H-01…H-26 in `docs/hardware-checklist.md` item 10; radio 2.4 GHz SSID/PSK (blocks every real-WiFi tool test); H-11 hardware half (limp latch); H-26 board revision; Jetson on-device facts (H-18).
