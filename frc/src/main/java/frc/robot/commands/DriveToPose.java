package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * On-the-fly PathPlanner drive to a field pose (Subzero "tag tour" legs: room centre ↔ staging pose in front of a
 * tag), using the drivetrain's fused global pose. Every leg is a fresh straight-line path from wherever the robot
 * IS when the command starts to {@code target}, followed by {@link AutoBuilder#followPath} under the very slow
 * {@link AutoConstants} caps (0.47 m/s, 0.5 m/s², 120 °/s, 240 °/s²). The holonomic heading is interpolated by
 * PathPlanner along the path and ends exactly at {@code target.getRotation()} ({@link GoalEndState}).
 *
 * <p>PathPlanner's FollowPathCommand finishes on trajectory TIME, not on pose error, so every leg ends with a short
 * settle-on-goal tail (P on the fused pose, same caps) until the robot is inside {@link #kSettleToleranceMeters} /
 * {@link #kSettleToleranceDeg} for {@link #kSettleSeconds}, or {@link #kSettleTimeoutSeconds} elapses. A target
 * within {@link #kDegenerateDistanceMeters} skips the path (PathPlanner cannot build a sane spline) and only settles.
 *
 * <p>Logging (AdvantageScope): {@code PathPlanner/GoalPose} (this leg's target), {@code PathPlanner/ActivePath}
 * (Pose2d[] — every calculated path, cleared when the path ends), {@code PathPlanner/TargetPose} /
 * {@code CurrentPose} (per loop while following), {@code PathPlanner/DriveToPose/*} (start pose, path length,
 * degenerate flag, final error).
 *
 * <p>Requires AutoBuilder to be configured (done in the {@link CommandSwerveDrivetrain} constructor); if it is not,
 * the command reports the error to the DS and does nothing rather than throwing inside the scheduler.
 */
public final class DriveToPose {
  private DriveToPose() {}

  /** Below this start→target distance PathPlanner cannot build a sane spline: settle (rotate in place) instead. */
  public static final double kDegenerateDistanceMeters = 0.05;

  /** Settle tail: done when inside these for kSettleSeconds; gives up (leaves the pose as-is) after the timeout. */
  public static final double kSettleToleranceMeters = 0.02;
  public static final double kSettleToleranceDeg = 1.5;
  public static final double kSettleSeconds = 0.2;
  public static final double kSettleTimeoutSeconds = 2.0;
  /** Settle translation gain, m/s per m of error (capped at AutoConstants.kMaxSpeedMps); heading uses the drivetrain's
   * FieldCentricFacingAngle heading PID (kP 7 rad/s per rad). */
  public static final double kSettleTranslationP = 2.0;

  /**
   * The one set of constraints every Subzero path uses: the AutoConstants speed caps derated by
   * {@link AutoConstants#kPathConstraintDerate}, because PathPlanner's holonomic PID feedback stacks on top of the
   * feed-forward while it corrects (sim showed 0.53 m/s / 147 °/s peaks at 100 %).
   */
  public static final PathConstraints kConstraints =
      new PathConstraints(
          AutoConstants.kMaxSpeedMps * AutoConstants.kPathConstraintDerate,
          AutoConstants.kMaxAccelMps2 * AutoConstants.kPathConstraintDerate,
          Math.toRadians(AutoConstants.kMaxAngularDegPerSec * AutoConstants.kPathConstraintDerate),
          Math.toRadians(AutoConstants.kMaxAngularAccelDegPerSec2 * AutoConstants.kPathConstraintDerate));

  /**
   * Drive from the CURRENT fused pose (read when the command initialises, not when it is built) to {@code target}.
   *
   * @param drivetrain the swerve drivetrain (AutoBuilder must be configured on it)
   * @param target field pose to finish at, heading included
   * @return a deferred command; safe to build at RobotContainer construction and reuse across runs
   */
  public static Command driveToPose(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    return driveToPose(drivetrain, () -> target);
  }

  /**
   * Same as {@link #driveToPose(CommandSwerveDrivetrain, Pose2d)} but the target is evaluated when the command
   * initialises (e.g. a pose computed from the latest tag sighting).
   */
  public static Command driveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
    return Commands.defer(() -> buildLeg(drivetrain, targetSupplier.get()), Set.of(drivetrain))
        .withName("DriveToPose");
  }

  /**
   * Builds the concrete leg for the pose the robot is at RIGHT NOW. Called by the deferred wrapper at initialise;
   * public so the orchestrator can pre-build a leg when the start pose is known (e.g. for a plan preview).
   */
  public static Command buildLeg(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    final Pose2d current = drivetrain.getPose();
    final double distance = current.getTranslation().getDistance(target.getTranslation());

    Logger.recordOutput("PathPlanner/GoalPose", target);
    Logger.recordOutput("PathPlanner/DriveToPose/StartPose", current);
    Logger.recordOutput("PathPlanner/DriveToPose/StraightLineDistance_m", distance);

    if (!AutoBuilder.isConfigured()) {
      DriverStation.reportError("DriveToPose: AutoBuilder is not configured — leg skipped", false);
      Logger.recordOutput("PathPlanner/DriveToPose/Degenerate", true);
      return Commands.none();
    }

    if (distance < kDegenerateDistanceMeters) {
      // Already there in translation: only the heading may differ. Settle in place (heading PID on the odometry
      // thread + translation P), capped by AutoConstants.
      Logger.recordOutput("PathPlanner/DriveToPose/Degenerate", true);
      return settleAtGoal(drivetrain, target)
          .andThen(logFinalError(drivetrain, target))
          .withName("DriveToPose(settle-only)");
    }
    Logger.recordOutput("PathPlanner/DriveToPose/Degenerate", false);

    // The Pose2d rotations given to waypointsFromPoses are the PATH TANGENT directions (spline heading), NOT the
    // robot heading: a straight segment from current to target has the same tangent at both ends.
    final Rotation2d tangent = target.getTranslation().minus(current.getTranslation()).getAngle();
    final List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(current.getTranslation(), tangent), new Pose2d(target.getTranslation(), tangent));

    final PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            kConstraints,
            null, // ideal starting state unknown → PathPlanner generates the trajectory from the live speeds/heading
            new GoalEndState(0.0, target.getRotation()));
    path.preventFlipping = true; // room coordinates: never mirror for an alliance
    path.name = "DriveToPose";

    return AutoBuilder.followPath(path)
        .andThen(settleAtGoal(drivetrain, target))
        .andThen(logFinalError(drivetrain, target))
        .withName("DriveToPose(path)");
  }

  /**
   * Close the residual error left by the time-based path follower: field-centric translation P on the fused pose
   * (capped at kMaxSpeedMps) while the drivetrain's FieldCentricFacingAngle heading PID turns to the goal heading
   * (capped at kMaxAngularDegPerSec). Ends once inside tolerance for kSettleSeconds, or after kSettleTimeoutSeconds.
   */
  private static Command settleAtGoal(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    final double maxOmega = Math.toRadians(AutoConstants.kMaxAngularDegPerSec);
    final Timer inTolerance = new Timer();
    return drivetrain
        .runOnce(inTolerance::restart)
        .andThen(
            drivetrain.applyRequest(
                () -> {
                  final Pose2d cur = drivetrain.getPose();
                  final Translation2d err = target.getTranslation().minus(cur.getTranslation());
                  final double headingErrDeg = Math.abs(target.getRotation().minus(cur.getRotation()).getDegrees());
                  if (err.getNorm() > kSettleToleranceMeters || headingErrDeg > kSettleToleranceDeg) {
                    inTolerance.restart();
                  }
                  Translation2d v = err.times(kSettleTranslationP);
                  if (v.getNorm() > AutoConstants.kMaxSpeedMps) {
                    v = v.times(AutoConstants.kMaxSpeedMps / v.getNorm());
                  }
                  Logger.recordOutput("PathPlanner/DriveToPose/SettleError_m", err.getNorm());
                  Logger.recordOutput("PathPlanner/DriveToPose/SettleError_deg", headingErrDeg);
                  return drivetrain
                      .facingAngleRequest
                      // Absolute field frame (the operator perspective is forced to 0° anyway, but be explicit).
                      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                      .withTargetDirectionPerspective(TargetDirectionPerspectiveValue.BlueAlliance)
                      .withTargetDirection(target.getRotation())
                      .withVelocityX(v.getX())
                      .withVelocityY(v.getY())
                      .withMaxAbsRotationalRate(maxOmega);
                }))
        .until(() -> inTolerance.hasElapsed(kSettleSeconds))
        .withTimeout(kSettleTimeoutSeconds)
        .withName("DriveToPose(settle)");
  }

  private static Command logFinalError(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    return Commands.runOnce(
        () -> {
          final Pose2d end = drivetrain.getPose();
          final Translation2d err = target.getTranslation().minus(end.getTranslation());
          Logger.recordOutput("PathPlanner/DriveToPose/EndPose", end);
          Logger.recordOutput("PathPlanner/DriveToPose/FinalError_m", err.getNorm());
          Logger.recordOutput(
              "PathPlanner/DriveToPose/FinalError_deg",
              Math.abs(target.getRotation().minus(end.getRotation()).getDegrees()));
        });
  }
}
