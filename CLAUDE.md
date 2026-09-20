# Subzero — HTN 2026 swerve robot (monorepo)

FRC-style swerve robot (~65 kg, 1.5 m elevator, linear belt-driven arm, pincher jaws, swappable end effectors on a rack) built at Hack the North 2026. The robot moving is the deliverable.

| Directory | What | Owner |
|---|---|---|
| `frc/` | RoboRIO code — WPILib 2026.2.1 + Phoenix6 26.3.0 + PhotonLib v2026.3.4 + AdvantageKit 26.0.2 | software |
| `esp32/` | ESP32-S3 firmware for the end-effector boards (DRV8833 DC motor, timed runs, HTTP over the robot WiFi) | software |
| `CAD/` | mechanical design (source, exports, drawings, BOM) | mech team |

## Environment (prefix every Gradle / PlatformIO command)
`export JAVA_HOME="$HOME/wpilib/2026/jdk"; export PATH="$JAVA_HOME/bin:$HOME/.platformio/penv/bin:$PATH"`

## Commands
- `frc/` — `./gradlew build` · `./gradlew simulateJava` (GUI, keyboard joystick via simgui.json) · `SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless` (headless, auto-enabled; add `SUBZERO_SIM_ARM_TEST=1` for the arm + pincher self-test) · `./gradlew deploy` (**humans only**, robot at 10.13.60.2)
- `esp32/endeffector/` — `pio run -e endeffector-1` (tool 1 @ 10.13.60.31), `-e endeffector-2` (tool 2 @ .32) · `pio run -e <env> -t upload` · `pio device monitor`

## Current robot configuration (tag `m0-hw`)
Drivetrain (2025 chassis TunerConstants, rio bus) + Elevator (Tuner X CTREELEVATOR config, ids 50/61) + Arm (single Kraken X60 CAN 40, 14T HTD-5 pulley direct on the shaft = 0.070 m/rev, no switches, zero at power-on) + Pincher (two servos on RoboRIO PWM 0/1). Controls in `RobotContainer.java`. The full M1 vision/align/task stack is commented out in place; the last fully-wired version is tag `m1-sim`.

## Rules
- **Sim is the proof.** Agents claim "passes in sim (`frc/logs/akit_*.wpilog`)", never "tested on hardware" — humans say that.
- Every `TODO(hardware)` number lives in exactly one of `frc/src/main/java/frc/robot/Constants.java` or `esp32/endeffector/include/config.h`, tagged with its H-id. Never invent a hardware number; placeholders fail safe.
- The ESP32 HTTP API (routes, JSON, heartbeat 200 ms / LOST_LINK 1000 ms, `/estop`) is documented in `esp32/endeffector/README.md`; the RoboRIO is its client over the robot WiFi (no laptop bridge).
- Safety: DS enable is the deadman; automated motion ≤ 1.0 m/s / 90 °/s and interruptible; elevator/arm soft limits + current limits + MotionMagic only; a spotter whenever the arm is extended; PWM servos go limp on disable. Elevator neutral is **Coast / 120 A** (matches the reference project) — hands clear.
- Pinned versions — do not drift: WPILib/GradleRIO 2026.2.1 · Java 17 · Phoenix6 26.3.0 (`https://api.ctr-electronics.com/phoenix6/stable/java/`) · PhotonLib v2026.3.4 · AdvantageKit 26.0.2 · PlatformIO 6.2.0 / espressif32 (Arduino core 3.2.0). Never re-run `./gradlew vendordep` against the rolling Phoenix6 URL; the committed JSON is the pin.
- MotionMagic tuning: `frc/MOTIONMAGIC-TUNING.md`.
