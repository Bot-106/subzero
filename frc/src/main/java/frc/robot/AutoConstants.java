package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * "Tag tour" autonomous: spin to localise, then visit every location tag (ascending, skipping ID 5 in the corner),
 * camera-align the RIGHT side to it, poke it with the arm, and return to the room centre on the global pose.
 *
 * <p>Speeds (user 2026-09-20): translation 10 % of the chassis (≈ 0.47 m/s), rotation 120 °/s everywhere.
 * Geometry note: the ARM EXTENDS TO THE RIGHT (robot −Y, the side the right camera faces), not forward.
 */
public final class AutoConstants {
  private AutoConstants() {}

  // ── speed caps ─────────────────────────────────────────────────────────────────────────────────────
  /** 10 % of kSpeedAt12Volts (4.73 m/s). */
  public static final double kMaxSpeedMps = 0.47;
  public static final double kMaxAccelMps2 = 0.5;
  public static final double kMaxAngularDegPerSec = 120.0;
  public static final double kMaxAngularAccelDegPerSec2 = 240.0;
  /** PathPlanner trajectories are generated at this fraction of the caps so feed-forward + PID feedback stays under them. */
  public static final double kPathConstraintDerate = 0.85;
  /** Localisation spin: one full turn at kMaxAngularDegPerSec ≈ 3 s. */
  public static final double kSpinDegrees = 360.0;

  // ── tour ───────────────────────────────────────────────────────────────────────────────────────────
  /** Tags to visit, in order (ID 5 is in the corner and skipped). */
  public static final int[] kTourTagIds = {2, 3, 4, 6, 8, 9};

  /**
   * Room centre used as the waypoint between tags — 1.5 m from both tagged walls, inside the region the
   * measurements actually cover (the 6 × 4 m room box is a placeholder, H-16).
   */
  public static final Translation2d kFieldCenter =
      new Translation2d(
          Constants.VisionConstants.kRoomLengthMeters - 1.5, Constants.VisionConstants.kRoomWidthMeters - 1.5);

  /**
   * Distance from the tag (along its normal, into the room) of the PathPlanner staging pose; the camera align starts
   * here. GEOMETRY: the right lens sits 0.43 m below the tag centres with pitch 0; the camera's ±20° vertical FOV
   * needs all four tag corners in frame, which is only true with the robot centre ≥ ~1.6 m from the wall. So the
   * staging pose is 2.1 m out (tag well inside the frame).
   */
  public static final Distance kStagingDistance = Meters.of(2.10);

  /**
   * The poke stand-off (0.80 m) is INSIDE the camera's blind zone (see kStagingDistance). With this true, the align
   * command uses the camera while it can see the tag, then carries the last sighting forward on the drivetrain's
   * pose delta for the final ~0.9 m (sim: 0.7 cm / 0.02° at the stand-off). Set false only if the right camera is
   * pitched up enough to keep the tag in frame at 0.80 m (VisionConstants.kRightCameraPitchDeg ≈ −25).
   */
  public static final boolean kAlignOdometryBridge = true;

  /**
   * Final robot-centre-to-tag distance for the poke (arm at 0.43 m must reach the tag: frame half-width 0.375 m +
   * arm reach beyond the frame — unknown). TODO(hardware) H-06/H-08: measure and set so the arm tip just touches.
   */
  public static final Distance kPokeStandoff = Meters.of(0.80);

  /** Alignment tolerance / settle / timeouts. */
  public static final double kAlignToleranceMeters = 0.03;
  public static final double kAlignToleranceDeg = 2.0;
  public static final double kAlignSettleSeconds = 0.3;
  public static final double kAlignTagLostHoldSeconds = 0.5; // no fresh sighting → hold still
  public static final double kAlignTagLostFailSeconds = 3.0; // no sighting this long → give up
  public static final double kAlignTimeoutSeconds = 15.0;
  /** Camera-align P gains (robot-frame m/s per m, deg/s per deg). */
  public static final double kAlignTranslationP = 1.0;
  public static final double kAlignRotationP = 2.0;

  // ── poke (elevator first up, arm out; arm back first, elevator down — protects the carousel) ───────
  /** Elevator top = the current cap (7 rot = 0.84 m). */
  public static final Distance kPokeElevatorHeight = Meters.of(0.84);
  /** Arm out = the current soft limit (0.43 m placeholder). */
  public static final Distance kPokeArmExtension = Meters.of(0.43);
  public static final double kPokeDwellSeconds = 0.5;

  /**
   * Robot heading that points the RIGHT side (and the arm) at a tag whose normal is {@code tagFacing}: the right
   * side direction is heading − 90°, so heading = angle(−normal) + 90°. Front wall (normal −X) → 90°; left wall
   * (normal −Y) → 180°.
   */
  public static Rotation2d headingWithRightSideToward(Rotation2d tagFacing) {
    return tagFacing.plus(Rotation2d.k180deg).plus(Rotation2d.kCCW_90deg);
  }

  /** Staging pose in front of a tag: {@code distance} out along the tag normal, right side toward the tag. */
  public static Pose2d stagingPose(Pose2d tagPose, Distance distance) {
    Translation2d p = tagPose.getTranslation().plus(new Translation2d(distance.in(Meters), tagPose.getRotation()));
    return new Pose2d(p, headingWithRightSideToward(tagPose.getRotation()));
  }
}
