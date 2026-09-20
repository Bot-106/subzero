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

## Current HEAD (tag `tag-tour-auto`): "tag tour" autonomous (PathPlanner + camera-only align + poke)

`autos/TagTourAuto.java` = `getAutonomousCommand()`. Speeds: translation ≤ 0.47 m/s (10 %), rotation ≤ 120 °/s (`AutoConstants`). No pincher use.
1. Calibrate the elevator (if not yet), **spin 360° in ~3 s** (`SpinToLocalize`) so both cameras sweep every tag and the global fused pose settles.
2. For tags 2, 3, 4, 6, 8, 9 (ID 5 skipped): **PathPlanner** on the global pose to a staging pose 2.1 m in front of the tag with the RIGHT side toward it (`DriveToPose`, on-the-fly `PathPlannerPath`, `AutoBuilder` configured on the CTRE drivetrain like Rebuilt2026) → **`AlignRightCameraToTag`**: holonomic control on the right camera's own view of that tag (`bestCameraToTarget`), to the 0.80 m stand-off → **`PokeTag`**: arm at 0 → elevator up to 0.84 m → arm out to 0.43 m (to the RIGHT, into the tag) → arm back → elevator down → PathPlanner back to the room centre (global pose). A poke is skipped if the alignment did not succeed.
3. **Geometry caveat (real):** the right lens is 0.43 m below the tag centres and level, so with the camera's ~40° vertical FOV the tag leaves the frame closer than ~1.6 m (all four corners are required). The align therefore uses the camera down to that point and carries the last sighting on the drivetrain pose delta for the final ~0.9 m (`AutoConstants.kAlignOdometryBridge = true`). Pitching the right camera up ≈ 25° (`VisionConstants.kRightCameraPitchDeg = -25`, then re-measure H-14) would make it camera-only all the way and allow `kAlignOdometryBridge = false`.
4. **Where to see the paths in AdvantageScope** (log or live NT): `PathPlanner/ActivePath` (Pose2d[] — drag onto the Odometry/3D field as a *Trajectory*; it appears when a leg starts and clears when it ends), `PathPlanner/TargetPose` and `CurrentPose` (follower), `Auto/PlanPoses` (every staging/aligned/centre pose of the whole tour, logged once at start), `Auto/Align/GoalPose` (camera-align goal), `Auto/Align/tagInRobot`, `Auto/Step`, `Auto/Poke/step`, `Auto/Spin/progressDeg`. Live NT: `/PathPlanner/ActivePath`, `/PathPlanner/TargetPose`.
5. Sim proof (`SUBZERO_SIM_TOUR_TEST=1 SUBZERO_SIM_AUTOENABLE=auto ./gradlew simulateJava -Pheadless`, `frc/logs/akit_26-09-20_07-47-26.wpilog`): odometry started 0.5 m / 20° wrong → 7 mm / 0.26° after the spin; all six tags aligned within ≤ 11 mm / 0.36° of the goal and poked (elevator 0.841 m, arm 0.431 m, correct order); peak 0.467 m/s; yaw peaks 130 °/s briefly inside PathPlanner rotation corrections (`kPathConstraintDerate` 0.85); 127 s total; final global error 4 mm / 0.25°.

## Previous step (tag `pinch-wired`): measured servo angles locked in, pinch/un-pinch in the choreography

- Measured 2026-09-20 with the NT angle test: **servo A (PWM 8) 54° open / 89° pinched; servo B (PWM 9) 72° open / 37° pinched** (`PincherConstants.kJaw{A,B}{Open,Closed}Deg`). `Pincher.pinch()` / `release()` write those pairs; the jaws boot OPEN. `Superstructure` now really pinches at grab/3 and un-pinches at dock/4, each followed by `kServoTravelSeconds` (0.5 s). **RB / LB = pinch / release by hand.** The dashboard entries still override live for retesting. Sim (`akit_26-09-20_06-16-08.wpilog`): jaws 54/72 through the dock, 89/37 from grab/3 onward.

## Previous step (tag `servo-test`): pinch-servo angle test on PWM 8/9

- `subsystems/Pincher.java` drives the two micro-servos from **RoboRIO PWM header channels 8 / 9** with WPILib `Servo` (dedicated FPGA servo PWM, ~0.1° resolution; 0.5–2.5 ms = 0–180°, `PincherConstants.kServoMin/MaxPulseUs`). (The earlier DIO 8/9 `DigitalOutput.enablePWM` version had ~7° steps — in git history at `5fda487`.) **Test mode:** every loop it follows NetworkTables `/SmartDashboard/Pincher/servoA_deg` and `servoB_deg` (0–180, default 90) — set them from AdvantageScope (NetworkTables tab → tuning mode), Elastic or Shuffleboard and watch the servos. Logged: `Pincher/servo{A,B}_{deg,pulse_us,duty}`. Sim-verified: NT 90/0/45/180/90 → 1500/500/1000/2500/1500 µs.
- Carousel choreography below is unchanged (pinch steps are still dwells; wire `pincher.setAngles(...)` in once the open/closed angles are known).

## Carousel grab / dock choreography (tag `carousel-demo`)

- `commands/Superstructure.java`: `setEndpointPosition(armExtension, elevatorHeight)` moves both mechanisms together (done when both settle; 0.75 m/s caps stay in the subsystems); `grabFromCarousel(slot)` = align (arm `kPreGrabArmPosition`, elevator at the slot height) → arm to `kAttachmentPinchPosition` → [pinch dwell] → elevator up `kPostPinchElevatorRaiseHeight` → arm back → elevator to `kCarouselClearHeight`; `dockToCarousel(slot)` = align at slot + offset → arm extends → elevator drops → [un-pinch dwell] → arm pulls back. Every step requires its subsystem, so a new sequence interrupts the running one and the hold defaults take over.
- `CarouselConstants.java` (all mock, `TODO(hardware) H-08`): `CarouselSlot.LEVEL_1` 0.30 m / `LEVEL_2` 0.60 m, pre-grab arm 0.02 m, pinch position 0.25 m, post-pinch raise 0.06 m, clear height 0.80 m (≤ the 0.84 m elevator cap), pinch dwell 0.5 s.
- **Bindings:** **A** = dock LEVEL_1 · **B** = dock LEVEL_2 · **X** = dock LEVEL_1 then grab LEVEL_2 · D-pad **up/down** = elevator ±1 in · D-pad **right/left** = arm ±1 in · **Start** = zero yaw. Sequences are gated on the first-enable calibration having finished.
- Sim proof (`SUBZERO_SIM_CAROUSEL_TEST=1`, `frc/logs/akit_26-09-20_04-54-52.wpilog`): dock L1 → (0.36, 0.02) → arm 0.25 → elevator 0.30 → arm 0.02; grab L2 → (0.60, 0.02) → arm 0.25 → elevator 0.66 → arm 0.02 → elevator 0.80; 8 s total; peaks 0.80 m/s elevator / 0.67 m/s arm.

## Previous step (tag `jog-control`): incremental (1-inch) control

- **Elevator** (CTREELEVATOR config; MotionMagic cruise capped to **0.75 m/s** = 6.25 rps, accel 3 m/s²): the default command holds a persistent target; **X / Y = target +1 in / −1 in**, clamped to [0, 7 rot = 0.84 m]; `calibrateZero` on the first enable resets the target to 0. (Deviation from the reference file: hold-target default instead of `manualDrive(0)`; `kMaxHeight` 1.0 m for the sim only.)
- **Arm** (single Kraken, 14T HTD-5 direct drive, no switches; cruise **0.75 m/s**, accel 1.5 m/s²): **A / B = target +1 in / −1 in**, clamped to the soft limits [0, 0.43 m]; zero = power-on position.
- **Start** = zero yaw (moved from B). Pincher remains disabled in `RobotContainer`.
- Sim proof (`SUBZERO_SIM_JOG_TEST=1`, `frc/logs/akit_26-09-20_04-34-04.wpilog`): elevator target 0.025 → 0.051 → 0.076 → 0.102 → 0.076 → 0.051 m with the carriage following (peak 0.28 m/s), arm 0.025 → 0.051 → 0.076 → 0.051 m (peak 0.23 m/s); jogs are requirement-free so they never interrupt the hold or the first-enable calibration.

## Vision prototyping (tag `vision-proto`) — localization

- **Room layout** (`frc/tools/gen_room_layout.py` → `frc/src/main/deploy/room-layout.json`): 3D-printed 36h11 sheets (266.7 mm plate, 165.1 mm black square — confirm with a tape measure), laid edge-to-edge with every other sheet omitted (centre pitch 2 sheets), bottom edge 4.6 sheets up (centre 1.360 m). LEFT wall from the front-left corner: IDs 5, 6, 8, 9 (first sheet in the corner). FRONT wall from the front-left corner: IDs 2, 3, 4 (first sheet 3.27 sheets from the corner). Frame: +X toward the FRONT wall (joystick forward), +Y toward the LEFT wall, Z up; the room box (6 × 4 m) is a placeholder that only translates the layout. **Upload the same JSON to PhotonVision** (AprilTag pipeline, tag size 0.1651 m, camera names `photoncamera_left` / `photoncamera_right`).
- **Heading sanity (2026-09-20):** the drivetrain's operator perspective is now FORCED to 0° (`DriveConstants.kOperatorPerspective`) — the generated code took it from the DS alliance, which defaults to Red 1 without an FMS and flipped "forward" (and `seedFieldCentric`) by 180°. If the fused pose still reads 180° off while facing the front wall, the cameras were measured from the drivetrain's −X side: set `VisionConstants.kMeasuredFrontIsRobotPlusX = false` (rotates both transforms 180° about Z). Test: with the DS enabled push the stick forward briefly — the side that leads is the drivetrain's +X. The sim cannot detect this class of error (its virtual cameras use the same transforms as the estimator).
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
