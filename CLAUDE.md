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
- `frc/` — `./gradlew build` · `./gradlew simulateJava` (GUI, keyboard joystick via simgui.json) · `SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless` (headless, auto-enabled; `SUBZERO_SIM_VISION_TEST=1` = injected-error localization test; `SUBZERO_SIM_JOG_TEST=1` = elevator/arm 1-inch jog self-test; `SUBZERO_SIM_CAROUSEL_TEST=1` = dock L1 → grab L2 choreography) · `./gradlew deploy` (**humans only**, robot at 10.13.60.2)
- `esp32/endeffector/` — `pio run -e endeffector-1` (tool 1 @ 10.13.60.31), `-e endeffector-2` (tool 2 @ .32) · `pio run -e <env> -t upload` · `pio device monitor`

## Current robot configuration (tag `carousel-demo`)
**Drivetrain + vision + elevator + arm. `commands/Superstructure` coordinates both (`setEndpointPosition`, `grabFromCarousel`, `dockToCarousel`; geometry in `CarouselConstants`, all mock). A = dock L1, B = dock L2, X = dock L1 → grab L2; D-pad = 1-inch jogs; Start = zero yaw; both mechanisms capped at 0.75 m/s; pincher disabled (pinch steps are dwells).** Vision: Two PhotonVision cameras `photoncamera_left` / `photoncamera_right` (side-facing, level, transforms measured in `Constants.VisionConstants`) fuse into the swerve pose estimator Rebuilt2026-style (`RoomCamera` = OrbitCamera lift, `updatePose()` in `CommandSwerveDrivetrain`, `util/RobotState` publishes the pose). Room AprilTag layout: `frc/tools/gen_room_layout.py` → `frc/src/main/deploy/room-layout.json` (upload the same file to PhotonVision; pipeline tag size 0.1651 m). Elevator / Arm / Pincher are disabled in `RobotContainer` only (classes intact; wiring at tag `m0-hw`). Sim proof: `SUBZERO_SIM_VISION_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless` injects a 1 m / 30° odometry error and vision must pull `Drive/Pose` back onto `Drive/SimTruthPose`.

## Rules
- **Sim is the proof.** Agents claim "passes in sim (`frc/logs/akit_*.wpilog`)", never "tested on hardware" — humans say that.
- Every `TODO(hardware)` number lives in exactly one of `frc/src/main/java/frc/robot/Constants.java` or `esp32/endeffector/include/config.h`, tagged with its H-id. Never invent a hardware number; placeholders fail safe.
- The ESP32 HTTP API (routes, JSON, heartbeat 200 ms / LOST_LINK 1000 ms, `/estop`) is documented in `esp32/endeffector/README.md`; the RoboRIO is its client over the robot WiFi (no laptop bridge).
- Safety: DS enable is the deadman; automated motion ≤ 1.0 m/s / 90 °/s and interruptible; elevator/arm soft limits + current limits + MotionMagic only; a spotter whenever the arm is extended; PWM servos go limp on disable. Elevator neutral is **Coast / 120 A** (matches the reference project) — hands clear.
- Pinned versions — do not drift: WPILib/GradleRIO 2026.2.1 · Java 17 · Phoenix6 26.3.0 (`https://api.ctr-electronics.com/phoenix6/stable/java/`) · PhotonLib v2026.3.4 · AdvantageKit 26.0.2 · PlatformIO 6.2.0 / espressif32 (Arduino core 3.2.0). Never re-run `./gradlew vendordep` against the rolling Phoenix6 URL; the committed JSON is the pin.
- MotionMagic tuning: `frc/MOTIONMAGIC-TUNING.md`.
