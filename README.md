# Subzero (Hack the North 2026)

An FRC-style swerve robot with a 1.5 m elevator, a linear belt-driven arm, pincher jaws and swappable end effectors on a rack. `frc/` is the RoboRIO code, `esp32/` the end-effector firmware, `CAD/` the mechanical design. See `CLAUDE.md` for how to build, simulate and deploy.

## Layout

```
frc/     WPILib 2026 project (Phoenix6 swerve + elevator + arm, pincher servos, AdvantageKit logging)
         MOTIONMAGIC-TUNING.md — how to tune the arm/elevator gains
esp32/   endeffector/       one ESP32-S3 per end effector: DRV8833 DC motor, timed open-loop runs, HTTP over robot WiFi
         legacy-servo-tool/ original servo-based tool firmware (reference only)
CAD/     mechanical design — source, exports, drawings, BOM
```

## Vision prototyping (tag `vision-proto`, HEAD) — drivetrain + localization only

- **Room layout** (`frc/tools/gen_room_layout.py` → `frc/src/main/deploy/room-layout.json`): 3D-printed 36h11 sheets (266.7 mm plate, 165.1 mm black square — confirm with a tape measure), laid edge-to-edge with every other sheet omitted (centre pitch 2 sheets), bottom edge 4.6 sheets up (centre 1.360 m). LEFT wall from the front-left corner: IDs 5, 6, 8, 9 (first sheet in the corner). FRONT wall from the front-left corner: IDs 2, 3, 4 (first sheet 3.27 sheets from the corner). Frame: +X toward the FRONT wall (joystick forward), +Y toward the LEFT wall, Z up; the room box (6 × 4 m) is a placeholder that only translates the layout. **Upload the same JSON to PhotonVision** (AprilTag pipeline, tag size 0.1651 m, camera names `photoncamera_left` / `photoncamera_right`).
- **Cameras** (`Constants.VisionConstants`): lens 36.722 in (0.9327 m) up, 26 cm behind the front edge and 19.5 cm in from each side of the 29.5 in frame → (+0.115, ±0.180, 0.933) m; level (roll/pitch 0); left faces +Y (yaw +90°), right faces −Y (yaw −90°).
- **Localization** (Rebuilt2026 lift): `RoomCamera` = OrbitCamera (PhotonPoseEstimator, coprocessor multi-tag PnP → lowest-ambiguity fallback, std-devs (2,2,2) single / (0.5,0.5,1) multi × (1 + d²/30), single-tag > 4 m rejected); `CommandSwerveDrivetrain.updatePose()` is the Rebuilt2026 loop; `util/RobotState` publishes `RobotState/Robot Odometry Pose`, velocity vectors and `Distances/Tag N To Robot Center`; each camera publishes `Photon Cameras/<name>/{Photon_Pose, Robot To Cam Offset, Robot Pose Transformed By Robot To Cam}` (NT, Rebuilt2026 names) plus AdvantageKit mirrors `Vision/<name>/…`. One deliberate change from Rebuilt2026: the 0.2 ambiguity gate is applied only to single-tag fallbacks, not to multi-tag PnP estimates (in this room head-on multi-tag frames were ~90 % "ambiguous" yet exactly as accurate) — flag `kApplyAmbiguityGateToMultiTag` in `RoomCamera`.
- **Sim proof** (`frc/logs/akit_26-09-20_01-41-03.wpilog`, `SUBZERO_SIM_VISION_TEST=1`): odometry deliberately reset 1.118 m / 30° away from the sim ground truth at t = 1.8 s → error 0.254 m / 13.6° at +0.2 s, 0.028 m / 4.4° at +0.5 s, 0.005 m / 1.8° at +0.7 s, then ≤ 0.002 m / ≤ 0.1° for the rest of a 20 s sweep; 573 vision measurements fused, 0 rejected; both cameras produced estimates.
- **AdvantageScope**: 3D Field → add `RobotState/RobotPose3d` (robot), `Vision/photoncamera_left|right/RobotPoseTransformedByRobotToCam` (camera Pose3d — check placement), `RobotState/AprilTags` (Pose3d[] → AprilTag objects), `Vision/…/EstimatedPose3d` (per-camera estimate); Odometry tab → `RobotState/RobotPose` or the NT `RobotState/Robot Odometry Pose`.

## Mechanism configuration (tag `m0-hw`; disabled in RobotContainer while prototyping vision)

| Part | Config | Proof |
|---|---|---|
| Drivetrain | 2025 SwerveProgrammingChassis `TunerConstants` on the `rio` bus (Pigeon2 5, MK4i L2, FL 25/26/27, FR 20/21/22, BL 15/16/17, BR 10/11/12) | builds; headless sim runs |
| Elevator | Tuner X CTREELEVATOR config matched exactly — leader 50 / follower 61, ratio 4, drum 0.0191008 m, **Coast, 120 A**, kP 16 / kS 0.2 / kV 0.48, MotionMagic 12 rps / 80 rps², hardware limit switches on the leader, Top = 7 rot / Ground = 6 rot, `calibrateZero` (−10 % duty to the hard stop) on the first enable | sim: calibrates on enable (hard-stop trigger fired), no exceptions |
| Arm | one Kraken X60 CAN 40, 14T HTD-5 pulley **directly on the spline** → 0.070 m/rev, ratio 1:1, no switches (zero = power-on position, Back re-zeroes), soft limits [0, 0.43 m] (travel still H-06), Brake, 60 A, MotionMagic with kS + kV feedforward and kG = 0 (horizontal linear axis) | sim: 0 → 0.151 → 0 m, `position_rot` max 2.16 = 0.15/0.070 |
| Pincher | two micro-servos on RoboRIO **PWM 0/1** (not DIO), jaw gap 0–40 mm onto mirrored open/closed angles, slew-limited 120 °/s; goes limp on DS disable | sim: gap 40 → 0 → 40 mm, jaw B mirrored |
| End effectors | `esp32/endeffector`: DRV8833 DC motor, `/latch` = fwd for `LATCH_RUN_MS`, `/release` = rev, `/lateral` = dead-reckoned timed move, `POST /run {dir,ms,speed}` for the bench; heartbeat loss stops the motor, `/estop` sleeps the driver, every run capped at 5 s | `pio run` exit 0 for `endeffector-1/-2` (RAM 14.1 %, Flash 28.7 %) |

**Controls:** left stick translate · right stick X rotate (field-centric, 50 % scalar) · **B** zero yaw · **X / Y** elevator Ground / Top · **LB / RB** arm Retracted / Rack (0.15 m) · **RT / LT** jog arm out / in (≤ 15 % duty) · **Back** re-zero arm · **D-pad down / up** pinch / release. Elevator calibrates zero on the first teleop/test enable.

## Before first power-on
- Physical e-stop + DS space-bar tested; battery ≥ 11.0 V; spotter whenever the arm is extended.
- Elevator: confirm the follower is physically opposed (`Follower(…, Opposed)`) before enabling — a wrong flag stalls both motors at 120 A. It is **Coast** with a 0-output default: it drops when disabled or idle.
- Arm: power on (or press Back) fully retracted; `−0.08` homing/jog sign must mean *retract*; set the real travel in `ArmConstants.kMaxExtension` / `kSoftLimitOut`.
- Pincher: find the real open/closed angles with the jaws empty (`PincherConstants`).
- End effectors: set `WIFI_SSID`/`WIFI_PSK` and the static IP in `esp32/endeffector/include/config.h`; check motor direction with `POST /run` at low speed (`MOTOR_INVERT`), then time the strokes to set `LATCH_RUN_MS` / `RELEASE_RUN_MS`. Bench `curl`s need a background heartbeat loop (README).
- Every `TODO(hardware)` placeholder (H-01…H-26) is listed by `grep -rn "TODO(hardware)" frc/src esp32/endeffector`.

## Not wired yet
- RoboRIO → end-effector HTTP client (the laptop bridge was removed; `frc/.../tasks/ToolClient.java` is commented out and spoke NetworkTables to that bridge — it needs a rewrite as a direct HTTP client with the 200 ms heartbeat).
- Vision / `alignToTag` / task primitives (`RoomCamera`, `AlignToTagCommand`, `tasks/*`, `SimSequence`) — commented out in place; last wired version at tag `m1-sim`.
