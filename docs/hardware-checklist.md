# Hardware checklist — M1 hardware session

Printed by the orchestrator at the end of the session. This checklist
encodes `00-brief/safety.md` items 2, 6, 8, 9 explicitly (marked below) and
`00-brief/hardware-todos.md` H-01…H-26 for humans to fill in.

**M0 checks** (do these before anything below — they gate M1):
- [ ] Swerve drives under joystick on the real chassis
- [ ] Elevator homes and holds a setpoint

---

## Session checklist

1. [ ] **(safety §8)** Physical e-stop on the robot **and** the DS space-bar
   disable are tested, right now, at the start of this session.
2. [ ] **(safety §9)** Battery voltage ≥ 11.0 V under load. If it drops below
   11.0 V under load at any point, **stop testing**.
3. [ ] **(safety §6)** A spotter stands by whenever the arm is extended. No
   motion without a human at the DS and a second person spotting.
4. [ ] Speed cap confirmed active: automated motion (alignToTag, composites)
   ≤ 1.0 m/s translation, ≤ 90 °/s rotation, acceleration slew-limited;
   joystick teleop defaults to a 50 % scalar.
5. [ ] Start pose taped on the floor.
6. [ ] **(acceptance test 5)** Run the M1 sequence 3× from the same taped
   start pose:
   - [ ] Run 1
   - [ ] Run 2
   - [ ] Run 3
7. [ ] AK log saved (RoboRIO has no USB stick — H-24 — logs land in
   `/home/lvuser/logs`) and opened in AdvantageScope
   (`/Users/adityachoudhuri/wpilib/2026/advantagescope/`).
8. [ ] Tool bench (**acceptance test 4**) — run each of the following against
   a tool at `10.13.60.31` (adjust octet per tool, H-13):
   ```
   curl http://10.13.60.31/status
   curl -X POST http://10.13.60.31/latch    -H 'Content-Type: application/json' -d '{"seq":1}'
   curl -X POST http://10.13.60.31/release  -H 'Content-Type: application/json' -d '{"seq":2}'
   curl -X POST http://10.13.60.31/lateral  -H 'Content-Type: application/json' -d '{"seq":3,"mm":12.5}'
   curl -X POST http://10.13.60.31/heartbeat -H 'Content-Type: application/json' -d '{"t":1}'
   curl -X POST http://10.13.60.31/estop
   ```
   Then stop sending heartbeats for 1 s and confirm `curl
   http://10.13.60.31/status` shows `"state":"LOST_LINK"`.
8b. [ ] **Pincher** (RoboRIO **PWM 0/1** — not DIO; servos on a 5–6 V rail with common ground): D-pad down = pinch, D-pad up = release. Find the real `PincherConstants.kJaw*OpenDeg/ClosedDeg` with the jaws empty before loading a tool; note a PWM servo goes limp when the DS is disabled — never rely on the pincher to hold a tool while disabled.
8c. [ ] **End-effector calibration** (`10.13.60.31/.32`, `esp32/endeffector/README.md`): open-loop DC motor via DRV8833, no feedback. Use `POST /run {"seq","dir":"fwd"|"rev","ms","speed"}` at low speed to check direction (`MOTOR_INVERT`), then time a full stroke to set `LATCH_RUN_MS` / `RELEASE_RUN_MS` / `LATERAL_MS_PER_MM`. Heartbeat loss STOPS the motor; `/estop` puts the DRV8833 to sleep.
9. [ ] **H-11 hardware half** — with the end-effector axis detached (limp, DRV8833 asleep) on
   `/estop`, does the tool fall off the hook?
   - If **yes**: flip `ESTOP_BEHAVIOUR` in `esp32/include/config.h` from
     `ESTOP_BEHAVIOUR_DETACH` to `HOLD`.
   - If **no**: leave as-is (current default: detach on `/estop`).
9b. [ ] **H-26** — read the ESP32-S3-DevKitC-1 silkscreen for its board
   revision.
   - If it reads **v1.1**: set `STATUS_LED_PIN` to `38` in
     `esp32/include/config.h`.
   - Default assumption (v1.0) uses `RGB_BUILTIN` (GPIO 48) — no change
     needed if the board is v1.0.
10. [ ] Every `TODO(hardware)` constant below is still at its documented
    placeholder value (confirm nothing was silently changed by a worker) —
    fill in real values here first, then in code:

| ID | Item | Placeholder | File |
|---|---|---|---|
| H-01 | Elevator: real travel (m) home→top; whether "~60 in" is carriage-above-floor or travel | 1.2 m travel, soft limit 1.15 m | `frc/src/main/java/frc/robot/Constants.java` |
| H-02 | Elevator: SensorToMechanismRatio (4) and drum radius (0.0191008 m) | keep generated | `frc/src/main/java/frc/robot/Constants.java` |
| H-03 | Elevator setpoints stow/rack/pick/place | Top 1.0 m, Ground 0.05 m | `frc/src/main/java/frc/robot/Constants.java` |
| H-04 | Elevator Brake vs Coast | Brake | `frc/src/main/java/frc/robot/Constants.java` |
| H-05 | Arm CAN id | 40 rio | `frc/src/main/java/frc/robot/Constants.java` |
| H-06 | Arm ratio, pulley pitch, travel, DIO channel NO/NC | 9:1, 0.05 m/rot, 0.45 m, DIO 0 NO | `frc/src/main/java/frc/robot/Constants.java` |
| H-07 | Arm setpoints retracted/rack/pick/place | 0 / 0.15 / 0.40 / 0.40 m | `frc/src/main/java/frc/robot/Constants.java` |
| H-08 | Tool rack slots + poses | 2 slots at 0.30 / 0.60 m, 0.15 m extension | `frc/src/main/java/frc/robot/Constants.java` |
| H-09 | Tools: count, servos, LiPo, buck, latch switch? | 2 tools, MG996R-class, 2S + 6 V buck, no switch | `esp32/include/config.h` |
| H-10 | LATCH_OPEN/CLOSED_DEG, lateral deg↔mm | 20 / 110°; 0–40 mm | `esp32/include/config.h` |
| H-11 | Tool e-stop semantics | DECIDED: detach on /estop, hold on link loss; hardware half: does a limp latch drop the tool? | `esp32/include/config.h` (`ESTOP_BEHAVIOUR`) |
| H-12 | ESP32-S3 GPIO servo1/servo2/switch/LED | 4 / 5 / 6 / 48 | `esp32/include/config.h` |
| H-13 | Radio model, SSID/passphrase, 2.4 GHz, static IPs | radio 10.13.60.1, RoboRIO .2, Jetson .11, laptop .5/DHCP, tools .31/.32/.33 | `esp32/include/config.h` + `integration/bridge/config.yaml` |
| H-14 | Cameras: count, model, mounts → robotToCamera; PhotonVision names | 2 cams; placeholder transforms = 2025 chassis mounts, NOT Subzero's; names photoncamera_fl / photoncamera_fr | `frc/src/main/java/frc/robot/Constants.java` |
| H-15 | AprilTag sizes/IDs | 165.1 mm location IDs 1–12; 63.5 mm object IDs 20–29 unused this weekend | `frc/src/main/java/frc/robot/Constants.java` |
| H-16 | Room dimensions, origin, measured tag poses | 8 × 6 m, 4 tags on 4 walls | `frc/src/main/java/frc/robot/Constants.java` + `docs/room-layout.template.json` + `frc/src/main/deploy/room-layout.json` |
| H-17 | Objects: dimensions, mass, grasp offset | 0.10 m cube, 0.3 kg, tag on front | `frc/src/main/java/frc/robot/Constants.java` |
| H-18 | Jetson JetPack/L4T, disk, camera, static IP | detect on device | `jetson/runbook.md` (table) |
| H-19 | Swerve encoder offsets, kSpeedAt12Volts | keep 2025 values | `frc/src/main/java/frc/robot/Constants.java` |
| H-20 | Robot mass & MOI | 65 kg, 6 kg·m² | `frc/src/main/java/frc/robot/Constants.java` |
| H-21 | Driver mapping | A=alignToTag(nearest), B=stow, X=toolLatch, Y=toolRelease | `frc/src/main/java/frc/robot/Constants.java` |
| H-22 | Elevator rigging factor | 1:1 | `frc/src/main/java/frc/robot/Constants.java` |
| H-23 | Phoenix Pro licence (FusedCANcoder fallback) | keep FusedCANcoder | `frc/src/main/java/frc/robot/Constants.java` |
| H-24 | USB stick in the RoboRIO? | none → `/home/lvuser/logs`, SignalLogger off | — (RoboRIO filesystem convention, not a code constant) |
| H-25 | Servo stall current → buck/LiPo rating | 2.5 A/servo @ 6 V, ≥ 5 A buck | `esp32/include/config.h` |
| H-26 | DevKitC-1 revision: v1.0 RGB GPIO48 / v1.1 GPIO38 | default v1.0 | `esp32/include/config.h` (`STATUS_LED_PIN`) |

## Before first power-on — findings from the sim session (2026-09-19)

- [ ] **Elevator follower direction (H-02-adjacent).** `Elevator.java` uses the Tuner X generated `Follower(leader, MotorAlignmentValue.Opposed)`. Confirm the second motor is physically opposed before first power-on — a wrong flag stalls both motors against each other at the 80 A stator cap.
- [ ] **Homing direction signs.** `ElevatorConstants.kHomingDutyCycle` and `ArmConstants.kHomingDutyCycle` are −0.08. On the robot, negative MUST mean elevator *down* / arm *retract*. Verify with a spotter and the DS ready to disable on the first enable (safety §4, §6).
- [ ] **Homing runs on the first teleop/test enable (A-01):** expect the arm to retract, then the elevator to drive down at ≤ 10 % duty, each with a timeout. If either times out, `isHomed()` stays false and every `elevatorTo`/`armTo`/composite refuses (prints "refused … not homed"). `/subzero/robot/homed` shows the state.
- [ ] **Field-centric heading:** a DS with no FMS defaults to the Red perspective, so field-forward = 180° until the driver presses **Start** (`seedFieldCentric`). Do this once at the start pose.
- [ ] **Vision first fix:** the drivetrain accepts the first vision estimate unconditionally (later ones are gated at 0.5 m). Expect one odometry jump when a tag is first seen — intended.
- [ ] **Camera aim (H-14):** the placeholder mounts are the 2025 chassis' (±45° yaw). The sim used a 120° FOV; a real ~60° camera on those mounts may not see the tag from a 20°-yawed start. `alignToTag` then refuses ("leash") rather than moving — re-aim the mounts and update `VisionConstants.kRobotTo*Camera`.
- [ ] **Known-good commit:** deploy from tag `m1-sim` (`git checkout m1-sim`), not from an untested HEAD.

---

## Deploy notes

- `git checkout` a tagged known-good commit **first**, before deploying to
  hardware.
- Deploy: `cd frc && ./gradlew deploy` — robot is at `10.13.60.2`.
