# Subzero — HTN 2026 swerve robot (monorepo)

FRC-style swerve robot (~65 kg, 1.5 m elevator, belt-driven extending arm, ESP32-S3 servo tools on an on-robot rack, PhotonVision on a Jetson) built at Hack the North 2026. Goal: **M1** — align to a location AprilTag → elevator + arm to pose → tool actuates (via the laptop bridge) → retract. The robot moving is the deliverable; the agent pipeline is the means.

| Milestone | What | Who proves it |
|---|---|---|
| M0 (Fri) | 2026 project builds + sims; swerve drives under joystick; elevator homes + holds | agents in sim, humans on metal |
| M1 (Sat) | trigger → alignToTag → elevatorTo/armTo → toolOp → Stow, 3× repeatable on hardware | agents in sim (`SimSequence`), humans on metal |
| M2 (Sun) | `integration/hri_cli.py` issues `/subzero/task/request` primitives | humans |

## Environment (prefix every Gradle / PlatformIO command)
`export JAVA_HOME="$HOME/wpilib/2026/jdk"; export PATH="$JAVA_HOME/bin:$HOME/.platformio/penv/bin:$PATH"` — `uv` must already be on PATH. Never use the system python (3.8).

## Commands per directory
- `frc/` — `./gradlew build` · `./gradlew simulateJava` (GUI, keyboard joystick via simgui.json) · `SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless` (headless, auto-enabled; runs `SimSequence`) · `./gradlew deploy` (**humans only**, robot at 10.13.60.2)
- `esp32/` — `pio run -e esp32-s3-devkitc-1` · `pio run -t upload` · `pio device monitor`
- `integration/` — `uv sync` · `uv run pytest` · `uv run python -m bridge --server 127.0.0.1` · `uv run python mock_tool.py` · `uv run python hri_cli.py`
- `docs/` — `uv run --project integration python docs/validate_layout.py docs/room-layout.template.json`
- `jetson/` — docs only; `bash -n jetson/gst/*.sh`

## Rules
- **Sim is the proof.** Agents claim "passes in sim (`frc/logs/akit_*.wpilog`)", never "tested on hardware" — humans say that.
- **Contracts are frozen** in `docs/contracts.md` (NT topics C.1, ESP32 HTTP C.2, task primitives C.3). Implement, never rename.
- Every `TODO(hardware)` number lives in exactly one of `frc/src/main/java/frc/robot/Constants.java`, `esp32/include/config.h`, `integration/bridge/config.yaml`, tagged with its H-id (`00-brief/hardware-todos.md`). Never invent a hardware number; placeholders fail safe.
- Safety (safety.md): DS enable is the deadman; automated motion ≤ 1.0 m/s / 90 °/s, slew-limited, interruptible by stick + `/subzero/estop`; elevator/arm soft limits, Brake, 80 A / 60 A, MotionMagic only, homing ≤ 10 % duty with timeout; every composite starts and ends in `Stow`; alignToTag leash 0.5 s / 1 m / 8 s.
- Pinned versions — do not drift: WPILib/GradleRIO 2026.2.1 · Java 17 · Phoenix6 26.3.0 · PhotonLib v2026.3.4 (docs: `https://docs.photonvision.org/en/v2026.3.4/`) · AdvantageKit 26.0.2 · Python 3.12 via `uv` (`pyntcore` comes from `uv add robotpy`) · PlatformIO 6.2.0 / espressif32 (Arduino core 3.2.0) · Node 22.17.1. Phoenix6 Javadoc: `https://api.ctr-electronics.com/phoenix6/stable/java/`.
- Never re-run `./gradlew vendordep` against the rolling Phoenix6 URL; the committed JSON is the pin.
