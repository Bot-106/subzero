package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RoomCamera;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Camera-direct holonomic alignment of the robot's RIGHT side (the arm side) to one AprilTag, using ONLY the right
 * camera's view of that tag — the fused pose is never used for control (it is only compared in the logs).
 * Port of the camera-direct branch of FRC 1360's {@code AlignToTagPoseCommand} ({@code methodBias > threshold}) to
 * the robot frame.
 *
 * <p>Each loop the newest sighting of {@code tagId} from {@link RoomCamera#getLatestTarget(int)} gives the tag pose
 * in the ROBOT frame: {@code tagInRobot = Pose3d() ∘ robotToCamera ∘ bestCameraToTarget → toPose2d() = (x_m, y_m,
 * ψ_m)}. Desired: the tag squarely off the right side at {@code standoff}, i.e. at {@code (0, −standoff)} with its
 * outward normal (the tag's +X axis) pointing at the robot along robot +Y, so {@code ψ_des = +90°}. Robot-relative
 * P control ({@link AutoConstants#kAlignTranslationP}, {@link AutoConstants#kAlignRotationP}):
 *
 * <ul>
 *   <li>{@code vx = kP · x_m} — tag ahead of the right-side line → drive forward;
 *   <li>{@code vy = kP · (y_m + standoff)} — tag farther right than desired ({@code y_m < −standoff}) → drive right
 *       (negative vy);
 *   <li>{@code ω = +kR · wrap(ψ_m − 90°)} — SIGN: rotating the robot CCW by δ rotates every robot-frame
 *       representation of a world-fixed object by −δ, so ψ_m' = ψ_m − δ. To drive ψ_m to 90° we need δ = ψ_m − 90°,
 *       i.e. rotate CCW (positive ω) by exactly the error. Check: tag 3 (world normal 180°) with the robot at
 *       heading 80° (10° short of the 90° goal) appears at ψ_m = 180° − 80° = 100° → error +10° → ω > 0 → CCW →
 *       heading rises toward 90°.
 * </ul>
 *
 * Caps: {@code |v| ≤ kMaxSpeedMps} (vector), {@code |ω| ≤ kMaxAngularDegPerSec}, isotropic slew at
 * {@code kMaxAccelMps2} / {@code kMaxAngularAccelDegPerSec2} while tracking (a hold / fail / done drops to zero
 * immediately). Applied with {@link SwerveRequest.RobotCentric} (closed-loop module velocity: open-loop voltage stalls
 * short of the goal at these speeds).
 *
 * <p>Sighting freshness: the sighting's own capture timestamp (FPGA base) is compared with now. Older than
 * {@link AutoConstants#kAlignTagLostHoldSeconds} → zero speeds (hold); older than
 * {@link AutoConstants#kAlignTagLostFailSeconds} (or never seen that long after start) → finish with
 * {@code succeeded() == false}. Done when {@code |error_xy| < kAlignToleranceMeters} and
 * {@code |error_deg| < kAlignToleranceDeg} continuously for {@link AutoConstants#kAlignSettleSeconds} on FRESH
 * sightings; overall timeout {@link AutoConstants#kAlignTimeoutSeconds}. {@code end()} applies
 * {@link SwerveRequest.Idle}. Requires the drivetrain.
 *
 * <p>OPTIONAL odometry bridge ({@link #AlignRightCameraToTag(CommandSwerveDrivetrain, int, Distance, boolean)} with
 * {@code odometryBridge = true}; OFF in the 3-arg constructor): the tag leaves the right camera's view before the
 * poke stand-off — with the sim camera model (640×400, 68° diag → ±19.7° vertical) and the lens 0.43 m below the tag
 * centre, all four tag corners are in frame only while the LENS is ≥ 1.43 m from the wall, i.e. robot centre ≥
 * ~1.61 m (measured in sim: lost at tagInRobot.y = −1.607 m). With the bridge ON, once the tag has been seen at
 * least once the last sighting is carried forward on the drivetrain's pose delta (odometry) while the camera is
 * blind, instead of holding / failing; {@code Auto/Align/bridged} is true and {@code Auto/Align/blindDistance_m} is
 * the distance driven since the last real sighting. The tag-lost hold / fail timers apply only until the first
 * sighting; the overall timeout always applies. Sim-only truth: the sim odometry is exact, so this proves the
 * geometry, not odometry drift.
 *
 * <p>Logs {@code Auto/Align/{active, tagId, standoff_m, tagInRobot, error_m, error_deg, vx, vy, omegaDegPerSec,
 * sightingAge_s, ambiguity, holding, bridged, blindDistance_m, done, failed, failReason, succeeded, elapsed_s}} plus, for AdvantageScope,
 * {@code Auto/Align/GoalPose} (the world pose the robot has when aligned, from the room layout =
 * {@link AutoConstants#stagingPose}(tag, standoff)) and {@code Auto/Align/CameraRobotPose} (the world robot pose the
 * camera sighting alone implies — compare with Drive/Pose or Drive/SimTruthPose).
 */
public class AlignRightCameraToTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final RoomCamera camera;
  private final int tagId;
  private final double standoffM;
  private final boolean odometryBridge;

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private final Timer timer = new Timer();
  private Debouncer settled = new Debouncer(AutoConstants.kAlignSettleSeconds, DebounceType.kRising);

  /** Loop dt clamp for the slew step so a scheduler stall can never produce a huge jump. */
  private static final double kMaxDtSeconds = 0.1;
  /**
   * Stiction floor: the swerve drive velocity loop (TunerConstants driveGains kS = 0, kP = 0.1 V/rps, 0.2 V friction
   * in sim) delivers only ≈ (command − 0.04 m/s): nothing below ≈ 0.05 m/s, so a pure P approach creeps for seconds inside
   * the last ~8 cm (sim: 8 cm → 3 cm took 6 s). While the translation error is above half the tolerance the commanded
   * speed vector is at least this long; below it the P term (< floor) is left as is, so the robot stops within
   * ~kAlignToleranceMeters/2 and never limit-cycles. Not a hardware number.
   */
  private static final double kStictionFloorMps = 0.10;

  private Optional<Pose2d> tagWorldPose = Optional.empty();
  private Pose2d goalPose = Pose2d.kZero;

  private PhotonTrackedTarget lastTarget = null;
  private double lastSightingTime = 0.0;
  /** Bridge: world pose of the tag as implied by the last sighting + the fused pose at that moment. */
  private Pose2d tagWorldSeen = Pose2d.kZero;
  private Pose2d poseAtSighting = Pose2d.kZero;
  private boolean bridged = false;
  private double blindDistanceM = 0.0;
  private double lastLoopTime = 0.0;
  private Pose2d tagInRobot = Pose2d.kZero;
  private Translation2d lastVelocity = Translation2d.kZero;
  private double lastOmega = 0.0;

  private boolean holding = true;
  private boolean done = false;
  private boolean failed = false;
  private boolean interrupted = false;
  private String failReason = "";

  /**
   * @param drivetrain the swerve drivetrain (required); its {@link CommandSwerveDrivetrain#getRightCamera()} is used
   * @param tagId fiducial id to align to
   * @param standoff final robot-centre-to-tag distance measured along robot −Y (e.g. {@link AutoConstants#kPokeStandoff})
   */
  public AlignRightCameraToTag(CommandSwerveDrivetrain drivetrain, int tagId, Distance standoff) {
    this(drivetrain, tagId, standoff, false);
  }

  /**
   * @param odometryBridge true → carry the last sighting forward on odometry while the camera is blind (see class
   *     doc); false → strict camera-only (hold on a stale sighting, fail when lost too long)
   */
  public AlignRightCameraToTag(
      CommandSwerveDrivetrain drivetrain, int tagId, Distance standoff, boolean odometryBridge) {
    this.drivetrain = drivetrain;
    this.camera = drivetrain.getRightCamera();
    this.tagId = tagId;
    this.standoffM = standoff.in(Meters);
    this.odometryBridge = odometryBridge;
    addRequirements(drivetrain);
    setName("AlignRightCameraToTag(" + tagId + (odometryBridge ? ", bridge" : "") + ")");
  }

  @Override
  public void initialize() {
    timer.restart();
    settled = new Debouncer(AutoConstants.kAlignSettleSeconds, DebounceType.kRising);
    lastTarget = null;
    // No sighting yet counts as "stale since the command started": kAlignTagLostFailSeconds to acquire.
    lastSightingTime = Timer.getFPGATimestamp();
    lastLoopTime = lastSightingTime;
    tagInRobot = Pose2d.kZero;
    tagWorldSeen = Pose2d.kZero;
    poseAtSighting = Pose2d.kZero;
    bridged = false;
    blindDistanceM = 0.0;
    lastVelocity = Translation2d.kZero;
    lastOmega = 0.0;
    holding = true;
    done = false;
    failed = false;
    interrupted = false;
    failReason = "";

    tagWorldPose = RoomCamera.getRoomLayout().getTagPose(tagId).map(Pose3d::toPose2d);
    goalPose =
        tagWorldPose
            .map(p -> AutoConstants.stagingPose(p, Meters.of(standoffM)))
            .orElse(Pose2d.kZero);
    if (tagWorldPose.isEmpty()) {
      System.out.println("AlignRightCameraToTag: tag " + tagId + " is not in the room layout (GoalPose = 0)");
    }
    log(true, 0.0, 0.0, 0.0, Double.NaN, Double.NaN, 0.0, Double.NaN);
  }

  @Override
  public void execute() {
    final double now = Timer.getFPGATimestamp();
    final double dt = MathUtil.clamp(now - lastLoopTime, 0.0, kMaxDtSeconds);
    lastLoopTime = now;

    final Pose2d fusedPose = drivetrain.getPose(); // logging + (bridge only) odometry propagation
    final var sighting = camera.getLatestTarget(tagId);
    boolean fresh = false;
    if (sighting.isPresent()) {
      lastTarget = sighting.get().target();
      lastSightingTime = sighting.get().timestampSeconds();
      fresh = true;
      // Tag pose in the ROBOT frame: robot ∘ (robot→camera) ∘ (camera→tag).
      final Pose3d tagInRobot3d =
          new Pose3d().plus(camera.getRobotToCamera()).plus(lastTarget.getBestCameraToTarget());
      tagInRobot = tagInRobot3d.toPose2d();
      poseAtSighting = fusedPose;
      tagWorldSeen = fusedPose.plus(new Transform2d(tagInRobot.getTranslation(), tagInRobot.getRotation()));
    }
    final double age = now - lastSightingTime;
    final boolean stale = age > AutoConstants.kAlignTagLostHoldSeconds;

    // Bridge: with a sighting on record and no sighting THIS loop, carry the tag forward on the odometry delta
    // (also covers the frames between two sightings, so the estimate never lags the robot's own motion).
    bridged = odometryBridge && lastTarget != null && !fresh;
    if (bridged) {
      tagInRobot = tagWorldSeen.relativeTo(fusedPose);
      blindDistanceM = fusedPose.getTranslation().getDistance(poseAtSighting.getTranslation());
    } else if (fresh) {
      blindDistanceM = 0.0;
    }

    if (!failed && age > AutoConstants.kAlignTagLostFailSeconds && !(odometryBridge && lastTarget != null)) {
      fail(
          lastTarget == null
              ? String.format("tag %d never seen in %.1f s", tagId, age)
              : String.format("tag %d lost for %.1f s", tagId, age));
    }
    if (!failed && timer.get() > AutoConstants.kAlignTimeoutSeconds) {
      fail(String.format("timeout after %.1f s", timer.get()));
    }
    holding = lastTarget == null || (stale && !bridged);

    double vx = 0.0, vy = 0.0, omega = 0.0;
    double errorM = Double.NaN, errorDeg = Double.NaN, ambiguity = Double.NaN;

    if (lastTarget != null) {
      ambiguity = lastTarget.getPoseAmbiguity();

      final double ex = tagInRobot.getX(); // desired x = 0
      final double ey = tagInRobot.getY() + standoffM; // desired y = −standoff
      // wrap(ψ_m − 90°): Rotation2d.minus wraps to (−180°, 180°].
      final double ePsiRad = tagInRobot.getRotation().minus(Rotation2d.kCCW_90deg).getRadians();
      errorM = Math.hypot(ex, ey);
      errorDeg = Math.abs(Math.toDegrees(ePsiRad));

      final boolean inTolerance =
          errorM < AutoConstants.kAlignToleranceMeters && errorDeg < AutoConstants.kAlignToleranceDeg;
      // Only non-stale sightings (or, with the bridge, odometry-carried ones) may count toward "settled".
      done = settled.calculate(inTolerance && !holding && !failed);

      if (!holding && !failed && !done) {
        // Robot-relative P control (see class doc for the sign derivation).
        Translation2d velocity =
            new Translation2d(AutoConstants.kAlignTranslationP * ex, AutoConstants.kAlignTranslationP * ey);
        omega = AutoConstants.kAlignRotationP * ePsiRad; // deg/s per deg == rad/s per rad

        // Stiction floor (see kStictionFloorMps), then caps: vector speed, angular rate, isotropic slew on both.
        if (errorM > AutoConstants.kAlignToleranceMeters / 2.0
            && velocity.getNorm() > 1e-6
            && velocity.getNorm() < kStictionFloorMps) {
          velocity = velocity.times(kStictionFloorMps / velocity.getNorm());
        }
        final double maxSpeed = AutoConstants.kMaxSpeedMps;
        if (velocity.getNorm() > maxSpeed) velocity = velocity.times(maxSpeed / velocity.getNorm());
        Translation2d delta = velocity.minus(lastVelocity);
        final double maxDelta = AutoConstants.kMaxAccelMps2 * dt;
        if (delta.getNorm() > maxDelta) delta = delta.times(maxDelta / delta.getNorm());
        velocity = lastVelocity.plus(delta);

        final double maxOmega = Math.toRadians(AutoConstants.kMaxAngularDegPerSec);
        omega = MathUtil.clamp(omega, -maxOmega, maxOmega);
        final double maxOmegaDelta = Math.toRadians(AutoConstants.kMaxAngularAccelDegPerSec2) * dt;
        omega = lastOmega + MathUtil.clamp(omega - lastOmega, -maxOmegaDelta, maxOmegaDelta);

        vx = velocity.getX();
        vy = velocity.getY();
      }
    } else {
      settled.calculate(false);
    }
    lastVelocity = new Translation2d(vx, vy);
    lastOmega = omega;

    if (failed || done) {
      drivetrain.setControl(idleRequest);
    } else {
      // Hold (stale sighting) commands explicit zero speeds rather than Idle so the modules brake in place.
      drivetrain.setControl(
          driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }
    log(true, vx, vy, omega, errorM, errorDeg, age, ambiguity);
  }

  @Override
  public boolean isFinished() {
    return done || failed;
  }

  @Override
  public void end(boolean interrupted) {
    this.interrupted = interrupted;
    drivetrain.setControl(idleRequest);
    lastVelocity = Translation2d.kZero;
    lastOmega = 0.0;
    log(false, 0.0, 0.0, 0.0, Double.NaN, Double.NaN, Timer.getFPGATimestamp() - lastSightingTime, Double.NaN);
    System.out.printf(
        "AlignRightCameraToTag(%d): %s after %.2f s (tagInRobot x=%.3f y=%.3f psi=%.1f deg%s)%n",
        tagId,
        succeeded() ? "ALIGNED" : (interrupted ? "interrupted" : "FAILED: " + failReason),
        timer.get(),
        tagInRobot.getX(),
        tagInRobot.getY(),
        tagInRobot.getRotation().getDegrees(),
        bridged ? String.format(", bridged %.2f m blind", blindDistanceM) : "");
  }

  /** True once the command finished by settling inside tolerance (not failed, not interrupted). */
  public boolean succeeded() {
    return done && !failed && !interrupted;
  }

  /** Why the command gave up, or "" if it has not failed. */
  public String failReason() {
    return failReason;
  }

  /** Latest tag pose in the robot frame (zero until the first sighting). */
  public Pose2d getTagInRobot() {
    return tagInRobot;
  }

  /** World pose the robot has when aligned (from the room layout; zero if the tag is not in the layout). */
  public Pose2d getGoalPose() {
    return goalPose;
  }

  private void fail(String reason) {
    if (!failed) {
      failed = true;
      failReason = reason;
      done = false;
    }
  }

  private void log(
      boolean active,
      double vx,
      double vy,
      double omega,
      double errorM,
      double errorDeg,
      double age,
      double ambiguity) {
    Logger.recordOutput("Auto/Align/active", active);
    Logger.recordOutput("Auto/Align/tagId", tagId);
    Logger.recordOutput("Auto/Align/standoff_m", standoffM);
    Logger.recordOutput("Auto/Align/tagInRobot", tagInRobot);
    Logger.recordOutput("Auto/Align/error_m", errorM);
    Logger.recordOutput("Auto/Align/error_deg", errorDeg);
    Logger.recordOutput("Auto/Align/vx", vx);
    Logger.recordOutput("Auto/Align/vy", vy);
    Logger.recordOutput("Auto/Align/omegaDegPerSec", Math.toDegrees(omega));
    Logger.recordOutput("Auto/Align/sightingAge_s", age);
    Logger.recordOutput("Auto/Align/ambiguity", ambiguity);
    Logger.recordOutput("Auto/Align/holding", holding);
    Logger.recordOutput("Auto/Align/bridged", bridged);
    Logger.recordOutput("Auto/Align/blindDistance_m", blindDistanceM);
    Logger.recordOutput("Auto/Align/done", done);
    Logger.recordOutput("Auto/Align/failed", failed);
    Logger.recordOutput("Auto/Align/failReason", failReason);
    Logger.recordOutput("Auto/Align/succeeded", succeeded());
    Logger.recordOutput("Auto/Align/elapsed_s", timer.get());
    Logger.recordOutput("Auto/Align/GoalPose", goalPose);
    // World robot pose implied by the camera sighting alone: tagWorld ∘ (tag→robot, expressed in the tag frame).
    if (lastTarget != null && tagWorldPose.isPresent()) {
      Logger.recordOutput(
          "Auto/Align/CameraRobotPose", tagWorldPose.get().plus(new Transform2d(tagInRobot, Pose2d.kZero)));
    }
  }
}
