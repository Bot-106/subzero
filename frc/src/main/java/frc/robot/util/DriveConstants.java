// Lifted from 1360 Orbit Robotics' SwerveProgrammingChassis `frc.robot.util.DriveConstants` (itself
// derived from FRC 6328 Mechanical Advantage, GPL-3.0) and adapted to WPILib 2026.2.1 / Phoenix6 26.3.0.
// Anything that is a Subzero safety cap or tolerance lives in frc.robot.Constants (orchestrator-owned)
// and is REFERENCED here, never duplicated.
package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Derived drive numbers (m/s, rad/s) for RobotContainer, DriveCommands and the align commands. */
public final class DriveConstants {
  private DriveConstants() {}

  // ───────────── joystick teleop ─────────────
  /** kSpeedAt12Volts desired top speed (m/s). TODO(hardware) H-19 lives in TunerConstants. */
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  /** 3/4 of a rotation per second max angular velocity (rad/s). */
  public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  /** Deadband applied as a decimal (0.1 = 10 %) — Constants.OperatorConstants.kStickDeadband. */
  public static final double joystickDeadbandDecimal = Constants.OperatorConstants.kStickDeadband;
  /** Safety §2: joystick teleop defaults to a 50 % scalar (Constants.DriveConstants.kTeleopScalar). */
  public static final double TeleopMaxSpeed = MaxSpeed * Constants.DriveConstants.kTeleopScalar;
  public static final double TeleopMaxAngularRate =
      MaxAngularRate * Constants.DriveConstants.kTeleopScalar;
  /** Any stick magnitude above this interrupts an automated command (safety §3). */
  public static final double driverInterruptThreshold =
      Constants.OperatorConstants.kDriverInterruptThreshold;

  // ───────────── automated motion caps (safety §2 / A-14) ─────────────
  /** Translation cap for alignToTag / composites, m/s. */
  public static final double AutoMaxSpeed = Constants.DriveConstants.kAutoMaxSpeed.in(MetersPerSecond);
  /** Rotation cap for alignToTag / composites, rad/s. */
  public static final double AutoMaxAngularRate =
      Constants.DriveConstants.kAutoMaxAngularRate.in(RadiansPerSecond);
  /** Translation slew limit for automated motion, m/s². */
  public static final double AutoMaxAccel = Constants.DriveConstants.kAutoMaxAccelMps2;

  // ───────────── PID constants for tag alignment (lifted 1360 values) ─────────────
  public static final double tagXKp = 2.0;
  public static final double tagXKi = 0.0;
  public static final double tagXKd = 0.0;

  public static final double tagYKp = 2.0;
  public static final double tagYKi = 0.0;
  public static final double tagYKd = 0.0;

  public static final double tagRKp = 0.2 * 0.6;
  public static final double tagRKi = (1.2 * 0.2) / 0.37;
  public static final double tagRKd = (3 * 0.2 * 0.37) / 40;
  /** Degrees — Constants.DriveConstants.kAlignToleranceDeg. */
  public static final double tagRTolerance = Constants.DriveConstants.kAlignToleranceDeg;

  /** HolonomicDriveController xy PID (AlignToTagPoseCommand.java L79–80). */
  public static final double holonomicXYKp = 3.0;
  public static final double holonomicXYKi = 0.0;
  public static final double holonomicXYKd = 0.0;
  /** HolonomicDriveController heading ProfiledPID (AlignToTagPoseCommand.java L81). */
  public static final double holonomicThetaKp = 3.0;
  public static final double holonomicThetaKi = 0.0;
  public static final double holonomicThetaKd = 0.0;

  /**
   * Heading profile constraints used for PID. The 1360 file computed these as MaxAngularRate × 2π
   * (a units slip — MaxAngularRate is already rad/s, giving ~29 rad/s). Subzero clamps them to the
   * automated rotation cap (90 °/s) with a 0.5 s ramp to the cap, per safety §2.
   */
  public static final double MaxAngularSpeedRadians = AutoMaxAngularRate; // Used for PID
  public static final double MaxAngularAccelerationRadians = AutoMaxAngularRate * 2.0; // Used for PID

  /** Position tolerance for HolonomicDriveController (3 cm / 2°, Constants.DriveConstants). */
  public static final Pose2d holonomicTolerance =
      new Pose2d(
          Constants.DriveConstants.kAlignTolerance.in(edu.wpi.first.units.Units.Meters),
          Constants.DriveConstants.kAlignTolerance.in(edu.wpi.first.units.Units.Meters),
          Rotation2d.fromDegrees(Constants.DriveConstants.kAlignToleranceDeg));
}
