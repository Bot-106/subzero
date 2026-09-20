package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Single home of every TODO(hardware) number in frc/ (H-01…H-26, see 00-brief/hardware-todos.md).
 * Every value below is a PLACEHOLDER that fails safe (short travel, low speed, low current) until a
 * human measures the robot. Workers reference these by name and cite the H-id — they never invent a
 * number. Humans: fill in hardware-todos.md first, then this file.
 *
 * <p>Owned by the orchestrator; workers return constants for this file instead of editing it.
 */
public final class Constants {
  private Constants() {}

  /** Robot-wide CAN bus (the 2025 chassis wiring; every device is on the RoboRIO bus). */
  public static final String kCanBusName = "rio";

  // ───────────────────────────── Elevator ─────────────────────────────
  /**
   * NOTE (2026-09-19, M0 hardware config): subsystems/Elevator.java is the Tuner X generated file from
   * FRC1360/SwerveProgrammingChassis/CTREELEVATOR, matched EXACTLY (leader 50 / follower 61 on rio,
   * ratio 4, drum 0.0191008 m, Coast, 120 A stator, kP 16 / kS 0.2 / kV 0.48, MotionMagic 12 rps /
   * 80 rps², hardware limit switches on the leader, setpoints Top = 7 rot, Ground = 6 rot,
   * calibrateZero at −10 % duty). It does NOT read the constants below; they are kept for the full
   * (seam-based) Elevator at git tag m1-sim.
   */
  public static final class ElevatorConstants {
    private ElevatorConstants() {}

    /** Leader/follower TalonFX CAN ids (Tuner X Elevator generator, rio bus). */
    public static final int kLeaderCanId = 30;
    public static final int kFollowerCanId = 31;

    // TODO(hardware) H-01 — real travel home→top; whether "~60 in" is carriage-above-floor or travel.
    // NEVER use elevator_upper_bound (37.48…) from elevator.TunerXProject.json — unit unmeasured (§9 trap).
    public static final Distance kMaxHeight = Meters.of(1.2);
    public static final Distance kSoftLimitTop = Meters.of(1.15);
    public static final Distance kSoftLimitBottom = Meters.of(0.0);

    // TODO(hardware) H-02 — verified SensorToMechanismRatio and drum radius (keep generated values).
    public static final double kSensorToMechanismRatio = 4.0;
    public static final Distance kDrumRadius = Meters.of(0.0191008);

    // TODO(hardware) H-22 — continuous rigging factor (carriage travel ÷ drum-surface travel), 1:1 or 2:1.
    public static final double kRiggingFactor = 1.0;

    // TODO(hardware) H-03 — real setpoints (m): stow / rack slot 1..3 / pick / place.
    public static final Distance kStowHeight = Meters.of(0.05);
    public static final Distance kGroundHeight = Meters.of(0.05);
    public static final Distance kTopHeight = Meters.of(1.0);
    public static final Distance kRackSlot1Height = Meters.of(0.30); // H-08
    public static final Distance kRackSlot2Height = Meters.of(0.60); // H-08
    public static final Distance kPickHeight = Meters.of(0.40);
    public static final Distance kPlaceHeight = Meters.of(0.60);

    // TODO(hardware) H-04 — Brake decided (safety §4); confirm nothing mechanical requires Coast.
    public static final boolean kBrakeNeutral = true;

    // A-11 / safety §4 — stator limit lowered from the generated 120 A. TODO(hardware) raise toward 120 if it stalls.
    public static final Current kStatorCurrentLimit = Amps.of(80);

    /** Homing (A-01): current-based zero at ≤ 10 % duty with a timeout. */
    public static final double kHomingDutyCycle = -0.08;
    public static final Current kHomingCurrentThreshold = Amps.of(20);
    public static final double kHomingTimeoutSeconds = 4.0;

    public static final Distance kTolerance = Meters.of(0.02);
    public static final double kToleranceHoldSeconds = 0.2;

    /** Slot0 (MotionMagicVoltage): V per mechanism rotation / per mechanism rps. Generated except kG. */
    public static final double kP = 16.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.2;
    // TODO(tuning) generated kG = 0 (no gravity hold); 0.35 V ≈ 8 kg carriage on 2× Kraken X60 @ 4:1 (sim-derived).
    public static final double kG = 0.35;
    public static final double kV = 0.48; // 12 V / 100 rps × ratio 4
    public static final double kA = 0.0;
    /** MotionMagic profile in carriage units; converted to mechanism rot/s and rot/s² in Elevator. */
    public static final LinearVelocity kCruiseVelocity = MetersPerSecond.of(1.0);
    public static final LinearAcceleration kAcceleration = MetersPerSecondPerSecond.of(2.5);
    public static final double kGoToTimeoutSeconds = 8.0;
    public static final double kHomingStallVelocityRps = 0.05;
    public static final double kHomingMinRunSeconds = 0.3;
    public static final double kHomingStallHoldSeconds = 0.2;
    public static final double kHomingCurrentDebounceSeconds = 0.1;
    // TODO(hardware) H-01-adjacent, sim only — real carriage mass (never reaches the robot).
    public static final double kSimCarriageMassKg = 8.0;
    public static final Distance kSimStartHeight = Meters.of(0.10);
    public static final Distance kSimFloorEpsilon = Meters.of(0.001);
    public static final double kSimLoopPeriodSeconds = 0.005;
  }

  // ─────────────────────────────── Arm ────────────────────────────────
  public static final class ArmConstants {
    private ArmConstants() {}

    // TODO(hardware) H-05 — arm Kraken X60 CAN id (rio bus). ONE motor, no follower.
    public static final int kCanId = 40;

    // The arm is a LINEAR axis (3D-printer X carriage), not a pivot: no limit switches; zero = wherever the
    // carriage is at power-on (Back button re-zeroes at the current position). Soft limits are relative to that zero.
    // Belt drive (measured 2026-09-19): 14-tooth HTD 5 mm pulley, belt anchored at both ends and wrapped over the
    // pulley → carriage travel = belt surface travel = 14 × 5 mm = 70 mm per PULLEY revolution (no 2:1 reeving).
    // (Pitch diameter 70/π = 22.28 mm; the sim's "drum radius" kMetersPerRotation/2π = 11.14 mm matches.)
    public static final Distance kMetersPerRotation = Meters.of(0.070);
    // H-06 (resolved 2026-09-19): the 14T pulley sits DIRECTLY on the Kraken's spline shaft — no gearbox.
    // 1 rotor rev = 1 pulley rev = 0.070 m. Free speed ≈ 100 rps ≈ 7 m/s (!) — the MotionMagic profile is the
    // only thing limiting speed, so keep kCruiseVelocity modest. At the 60 A stator cap the belt force is
    // ≈ 1.16 N·m / 11.14 mm ≈ 105 N (~10 kgf): enough for the carriage, still a sane pinch limit.
    public static final double kSensorToMechanismRatio = 1.0;
    // TODO(hardware) H-06 — total travel (m) from the retracted zero.
    public static final Distance kMaxExtension = Meters.of(0.45);
    public static final Distance kSoftLimitOut = Meters.of(0.43);
    public static final Distance kSoftLimitIn = Meters.of(0.0);

    // TODO(hardware) H-07 — extension setpoints (m): retracted / rack / pick / place.
    public static final Distance kRetracted = Meters.of(0.0);
    public static final Distance kRackExtension = Meters.of(0.15);
    public static final Distance kPickExtension = Meters.of(0.40);
    public static final Distance kPlaceExtension = Meters.of(0.40);

    // A-11 / safety §4. TODO(hardware) confirm.
    public static final Current kStatorCurrentLimit = Amps.of(60);
    public static final boolean kBrakeNeutral = true;

    /** Manual jog (triggers) duty cycle — low on purpose for the first hardware test. */
    public static final double kJogDutyCycle = 0.15;

    public static final Distance kTolerance = Meters.of(0.02);

    // ── MotionMagic + feedforward (see frc/MOTIONMAGIC-TUNING.md). Units: pulley = rotor rotations (direct drive).
    // Linear horizontal axis → no gravity term: kG = 0 (GravityType Elevator_Static = constant, so a small
    // kG only if the axis is inclined). TODO(tuning) — all of these are untuned placeholders.
    public static final double kP = 12.0; // V per mechanism (pulley) rotation (= per 0.070 m) of error
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.20; // V to overcome static friction (direct drive → carriage friction is felt 1:1; find with the jog test)
    public static final double kG = 0.0;  // horizontal axis
    public static final double kV = 0.12; // V per pulley rps: 12 V / 100 rps (Kraken X60 free speed, direct drive)
    public static final double kA = 0.0;  // V per mechanism rps²
    public static final LinearVelocity kCruiseVelocity = MetersPerSecond.of(0.75);      // 10.7 pulley rps (speed cap requested 2026-09-20)
    public static final LinearAcceleration kAcceleration = MetersPerSecondPerSecond.of(1.5); // 21.4 pulley rps²
    public static final double kGoToTimeoutSeconds = 6.0;
    public static final double kToleranceHoldSeconds = 0.2;
    // TODO(hardware) H-06-adjacent, sim only — real carriage mass.
    public static final double kSimCarriageMassKg = 2.0;
    public static final double kSimLoopPeriodSeconds = 0.005;
  }

  // ───────────────────────────── Pincher ──────────────────────────────
  /** Two micro-servos on the arm carriage that pinch / release a swappable end effector. Driven by the RoboRIO. */
  public static final class PincherConstants {
    private PincherConstants() {}

    // TODO(hardware) H-12 — RoboRIO PWM channels (servos MUST be on the PWM header 0–9, not DIO: WPILib's Servo
    // only drives PWM channels and DIO pins cannot generate servo pulses). Power the servos from a 5–6 V rail.
    public static final int kJawAPwmChannel = 0;
    public static final int kJawBPwmChannel = 1;

    // TODO(hardware) H-10 — jaw angles (deg, WPILib Servo.setAngle 0–180). Jaw B is mirrored.
    public static final double kJawAOpenDeg = 20.0;
    public static final double kJawAClosedDeg = 110.0;
    public static final double kJawBOpenDeg = 160.0;
    public static final double kJawBClosedDeg = 70.0;

    // TODO(hardware) H-10 — jaw gap when fully open (mm); 0 = closed on the tool.
    public static final double kJawMaxMm = 40.0;

    /** Never slam the jaws: max angle rate (deg/s). */
    public static final double kSlewDegPerSec = 120.0;
    /** Gap below which the jaws count as closed / above which they count as open. */
    public static final double kClosedThresholdMm = 2.0;
  }

  // ─────────────────────────────── Drive ──────────────────────────────
  public static final class DriveConstants {
    private DriveConstants() {}

    /** Safety §2 / A-14 — caps for any automated motion (alignToTag, composites). */
    public static final LinearVelocity kAutoMaxSpeed = MetersPerSecond.of(1.0);
    public static final AngularVelocity kAutoMaxAngularRate = DegreesPerSecond.of(90);
    public static final double kAutoMaxAccelMps2 = 1.5;
    public static final double kTeleopScalar = 0.1;

    /** alignToTag tolerance + leash (safety §7). */
    public static final Distance kAlignTolerance = Meters.of(0.03);
    public static final double kAlignToleranceDeg = 2.0;
    public static final double kAlignSettleSeconds = 0.3;
    public static final double kAlignTagStaleSeconds = 0.5;
    public static final Distance kAlignMaxPoseJump = Meters.of(1.0);
    public static final double kAlignTimeoutSeconds = 8.0;

    /** alignToTag gains (not hardware). Sim-proven (W3): 1.5 m / 20° → < 3 cm / 2° in 3.1 s under the caps. */
    public static final double kAlignTranslationP = 4.5;
    public static final double kAlignTranslationI = 0.0;
    public static final double kAlignTranslationD = 0.05;
    public static final double kAlignRotationP = 5.0;
    public static final double kAlignRotationI = 0.0;
    public static final double kAlignRotationD = 0.1;

    // TODO(hardware) H-19 — confirm 2025 encoder offsets + kSpeedAt12Volts after any module service (keep 2025 values).
    public static final boolean kUse2025ModuleOffsets = true;
    // TODO(hardware) H-20 — robot mass & MOI.
    public static final double kRobotMassKg = 65.0;
    public static final double kRobotMoiKgM2 = 6.0;
    // TODO(hardware) H-23 — Phoenix Pro licence on TalonFX/CANcoders (FusedCANcoder silently falls back).
    public static final boolean kAssumePhoenixPro = true;
  }

  // ────────────────────────────── Vision ──────────────────────────────
  public static final class VisionConstants {
    private VisionConstants() {}

    /**
     * Room AprilTag layout (deploy dir). Generated by frc/tools/gen_room_layout.py from the 2026-09-20 room
     * measurements; the SAME file must be uploaded to PhotonVision. Frame: +X toward the FRONT wall, +Y toward
     * the LEFT wall, origin = virtual rear-right corner (room box is a placeholder, H-16).
     */
    public static final String kRoomLayoutFile = "room-layout.json";

    // TODO(hardware) H-14 — camera names MUST equal the PhotonVision UI names.
    public static final String kLeftCameraName = "photoncamera_left";
    public static final String kRightCameraName = "photoncamera_right";

    /**
     * Robot → camera transforms (measured 2026-09-20). Robot origin = frame centre at FLOOR level; +X forward,
     * +Y left, +Z up. Frame outer edge 29.5 in square → half-width 14.75 in = 0.3747 m.
     * <ul>
     *   <li>lens height: 33 in + 3.722 in = 36.722 in = 0.9327 m
     *   <li>26 cm behind the FRONT edge → x = 0.3747 − 0.260 = +0.1147 m
     *   <li>19.5 cm in from each SIDE edge → y = ±(0.3747 − 0.195) = ±0.1797 m (left +, right −)
     *   <li>both cameras parallel to the ground: roll 0, pitch 0
     *   <li>each camera points OUTWARD to its own side: left lens along +Y (yaw +90°), right lens along −Y (yaw −90°)
     * </ul>
     * Logged as "Photon Cameras/<name>/Robot To Cam Offset" and
     * "…/Robot Pose Transformed By Robot To Cam" so the placement can be checked visually in AdvantageScope 3D.
     */
    public static final double kFrameHalfWidthMeters = Units.inchesToMeters(29.5 / 2.0);
    public static final double kCameraHeightMeters = Units.inchesToMeters(33.0 + 3.722);
    public static final double kCameraFromFrontMeters = 0.26;
    public static final double kCameraFromSideMeters = 0.195;
    public static final double kLeftCameraYawDeg = 90.0;   // faces left (+Y)
    public static final double kRightCameraYawDeg = -90.0; // faces right (−Y)
    public static final Transform3d kRobotToLeftCamera =
        new Transform3d(
            new Translation3d(
                kFrameHalfWidthMeters - kCameraFromFrontMeters,
                kFrameHalfWidthMeters - kCameraFromSideMeters,
                kCameraHeightMeters),
            new Rotation3d(0.0, 0.0, Math.toRadians(kLeftCameraYawDeg)));
    public static final Transform3d kRobotToRightCamera =
        new Transform3d(
            new Translation3d(
                kFrameHalfWidthMeters - kCameraFromFrontMeters,
                -(kFrameHalfWidthMeters - kCameraFromSideMeters),
                kCameraHeightMeters),
            new Rotation3d(0.0, 0.0, Math.toRadians(kRightCameraYawDeg)));

    // TODO(hardware) H-15 — 3D-printed sheets: the common "Full Size" 36h11 plate is 10.5 in = 266.7 mm (= AndyMark
    // plate) with the true-scale 165.1 mm black square centred ("approx 27 cm" measured). kLocationTagSize is what
    // PhotonVision's pipeline "tag size" must be set to. Confirm both with a tape measure / calipers; the layout
    // generator (frc/tools/gen_room_layout.py) scales every distance with the sheet size.
    // Location tag IDs in the room: 2, 3, 4 (front wall), 5, 6, 8, 9 (left wall).
    public static final Distance kSheetSize = Meters.of(0.2667);
    public static final Distance kLocationTagSize = Meters.of(0.1651);
    public static final int kLocationTagIdMin = 1;
    public static final int kLocationTagIdMax = 12;

    // TODO(hardware) H-16 — placeholder room box (see gen_room_layout.py; translation only).
    public static final double kRoomLengthMeters = 6.0;
    public static final double kRoomWidthMeters = 4.0;

    /** Std-dev heuristic (Rebuilt2026 OrbitCamera): single-tag (2,2,2), multi-tag (0.5,0.5,1); ambiguity gate; single-tag > 4 m rejected. */
    public static final double kMaxAmbiguity = 0.2;
    public static final Distance kMaxSingleTagDistance = Meters.of(4.0);
    public static final Distance kVisionOutlierGate = Meters.of(0.5);

    /** Sim camera model (not hardware). */
    public static final int kSimCameraWidthPx = 640;
    public static final int kSimCameraHeightPx = 400;
    public static final double kSimCameraFovDiagDeg = 68.0;
    public static final double kSimCameraFps = 50.0;
    public static final double kSimCameraAvgLatencyMs = 21.0;
    public static final double kSimCameraLatencyStdDevMs = 3.0;
    public static final double kSimCameraAvgErrorPx = 0.05;
    public static final double kSimCameraErrorStdDevPx = 0.02;
  }

  // ─────────────────────────────── Tasks / tools ──────────────────────
  public static final class TaskConstants {
    private TaskConstants() {}

    /** Contract C.1: tool command treated as failed after 1.5 s without ack. */
    public static final double kToolAckTimeoutSeconds = 1.5;
    public static final double kBridgeAliveStaleSeconds = 1.0;
    /** Time-box for elevatorTo / armTo (stow uses 2x) and each SimSequence step; MotionMagic moves take < 3 s. */
    public static final double kMechanismTimeoutSeconds = 6.0;

    // TODO(hardware) H-13 — end-effector boards join the robot WiFi with static IPs; the RoboRIO talks HTTP to them
    // directly (no laptop bridge any more). Index = tool id − 1. Radio 10.13.60.1, RoboRIO 10.13.60.2.
    public static final String[] kToolIps = {"10.13.60.31", "10.13.60.32"};
    public static final int kToolIdCount = 2; // H-09
    public static final int kDefaultToolId = 1;

    // TODO(hardware) H-08 — tool rack: 2 slots; approach/latch poses (elevator height, arm extension, lateral mm).
    public static final int kRackSlotCount = 2;
    public static final double kRackLateralMm = 20.0;

    // TODO(hardware) H-17 — object 0.10 m cube, 0.3 kg, tag on the front face; grasp offset relative to the tag.
    public static final Distance kObjectSize = Meters.of(0.10);
    public static final double kObjectMassKg = 0.3;

    /** Fixed location-tag offsets for pick/place (gate Q4): robot 0.45 m in front of the tag, facing it. */
    public static final double kPickOffsetX_m = 0.45;
    public static final double kPickOffsetY_m = 0.0;
    public static final double kPickYaw_deg = 180.0;
  }

  // ─────────────────────────────── Operator ───────────────────────────
  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int kDriverControllerPort = 0;
    // TODO(hardware) H-21 — driver mapping: A=alignToTag(nearest), B=stow, X=toolLatch, Y=toolRelease.
    public static final double kStickDeadband = 0.1;
    /** Any stick input above this interrupts an automated command (safety §3). */
    public static final double kDriverInterruptThreshold = 0.15;
  }

  /** Default sim acceptance targets (interface-and-demo §4 tests 1–2). */
  public static final class SimConstants {
    private SimConstants() {}

    public static final Distance kSimElevatorTarget = Meters.of(0.8);
    public static final Distance kSimArmTarget = Meters.of(0.3);
    public static final int kSimAlignTagId = 3;
    public static final Distance kSimAlignStartOffset = Meters.of(1.5);
  }

  static {
    // Compile-time sanity on placeholders (fail-safe ordering).
    assert ElevatorConstants.kSoftLimitTop.lt(ElevatorConstants.kMaxHeight);
    assert ArmConstants.kSoftLimitOut.lt(ArmConstants.kMaxExtension);
    assert DriveConstants.kAlignToleranceDeg > 0 && Degrees.of(DriveConstants.kAlignToleranceDeg).gt(Degrees.of(0));
  }
}
