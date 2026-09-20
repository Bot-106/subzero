package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * Rotate in place through {@link AutoConstants#kSpinDegrees} (CCW) at {@link AutoConstants#kMaxAngularDegPerSec}
 * (≈ 3 s per turn) so both side cameras sweep every tag and the fused pose settles before the tag tour.
 *
 * <p>Progress is the accumulated, unwrapped heading change of {@code drivetrain.getPose().getRotation()}; the
 * per-loop delta is wrapped to (−180°, 180°] by {@code Rotation2d.minus}, and a delta larger than
 * {@link #kMaxPlausibleLoopDeltaDeg} (a vision correction / pose reset, not rotation) is ignored. Hard timeout
 * {@code kSpinDegrees / kMaxAngularDegPerSec + 2 s}. {@code end()} applies {@link SwerveRequest.Idle}. Requires
 * the drivetrain. Logs {@code Auto/Spin/{progressDeg, done, timedOut, elapsed_s, active}}.
 */
public class SpinToLocalize extends Command {
  /** A per-loop heading jump above this is a pose reset / vision snap, not real rotation — skipped. */
  private static final double kMaxPlausibleLoopDeltaDeg = 45.0;

  private final CommandSwerveDrivetrain drivetrain;
  private final double targetDeg = AutoConstants.kSpinDegrees;
  private final double rateRadPerSec = Math.toRadians(AutoConstants.kMaxAngularDegPerSec);
  private final double timeoutSeconds =
      AutoConstants.kSpinDegrees / AutoConstants.kMaxAngularDegPerSec + 2.0;

  // Closed-loop module velocity so the low, constant rate is tracked rather than stalled (open-loop voltage
  // under-delivers at small commands).
  private final SwerveRequest.RobotCentric spinRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private final Timer timer = new Timer();

  private Rotation2d lastHeading = Rotation2d.kZero;
  private double progressDeg = 0.0;
  private boolean done = false;
  private boolean timedOut = false;

  public SpinToLocalize(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    setName("SpinToLocalize");
  }

  @Override
  public void initialize() {
    lastHeading = drivetrain.getPose().getRotation();
    progressDeg = 0.0;
    done = false;
    timedOut = false;
    timer.restart();
    log(true);
  }

  @Override
  public void execute() {
    final Rotation2d heading = drivetrain.getPose().getRotation();
    final double deltaDeg = heading.minus(lastHeading).getDegrees(); // wrapped to (-180, 180]
    lastHeading = heading;
    if (Math.abs(deltaDeg) <= kMaxPlausibleLoopDeltaDeg) {
      progressDeg += deltaDeg;
    }

    done = Math.abs(progressDeg) >= targetDeg;
    timedOut = timer.get() >= timeoutSeconds;

    if (done || timedOut) {
      drivetrain.setControl(idleRequest);
    } else {
      // Zero translation, +rate = CCW. Rotation direction is the sign of kSpinDegrees.
      drivetrain.setControl(
          spinRequest
              .withVelocityX(0.0)
              .withVelocityY(0.0)
              .withRotationalRate(Math.copySign(rateRadPerSec, targetDeg)));
    }
    log(true);
  }

  @Override
  public boolean isFinished() {
    return done || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(idleRequest);
    log(false);
    System.out.printf(
        "SpinToLocalize: %s after %.2f s, %.1f deg%n",
        done ? "done" : (interrupted ? "interrupted" : "TIMED OUT"), timer.get(), progressDeg);
  }

  /** Accumulated heading change so far (degrees, signed). */
  public double getProgressDeg() {
    return progressDeg;
  }

  private void log(boolean active) {
    Logger.recordOutput("Auto/Spin/active", active);
    Logger.recordOutput("Auto/Spin/progressDeg", progressDeg);
    Logger.recordOutput("Auto/Spin/targetDeg", targetDeg);
    Logger.recordOutput("Auto/Spin/done", done);
    Logger.recordOutput("Auto/Spin/timedOut", timedOut);
    Logger.recordOutput("Auto/Spin/elapsed_s", timer.get());
  }
}
