package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * VISION PROTOTYPING configuration (2026-09-20): DRIVETRAIN + VISION ONLY.
 *
 * <p>The two RoomCameras ("photoncamera_left" / "photoncamera_right") live inside the drivetrain,
 * Rebuilt2026-style, and fuse into its pose estimator in {@code updatePose()}. {@link RobotState}
 * publishes the fused pose / velocity / tag distances (Rebuilt2026 topic names) for AdvantageScope.
 *
 * <p>Elevator, Arm and Pincher are DISABLED HERE ONLY — their classes are intact; the wiring is in the
 * commented block below (previous version: git tag {@code m0-hw}).
 *
 * <p>Controls: left stick = translate (field-centric), right stick X = rotate; B = zero yaw
 * (seedFieldCentric — current heading becomes "forward" = toward the FRONT wall).
 */
public class RobotContainer {
  private final double kMaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.kTeleopScalar; // safety §2
  private final double kMaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.DriveConstants.kTeleopScalar;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(kMaxSpeed * OperatorConstants.kStickDeadband)
          .withRotationalDeadband(kMaxAngularRate * OperatorConstants.kStickDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  /** Drivetrain — constructs and fuses the two RoomCameras (Rebuilt2026 pattern). */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final RobotState robotState = RobotState.getInstance();

  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // ───────────── DISABLED for vision prototyping (classes intact; wiring at tag m0-hw) ─────────────
  // public final Elevator elevator = new Elevator();
  // private boolean elevatorCalibrated = false;
  // public final Arm arm = new Arm();
  // public final Pincher pincher = new Pincher();

  public RobotContainer() {
    robotState.setAllSuppliers(drivetrain::getPose, drivetrain::getChassisSpeeds);
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * kMaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * kMaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * kMaxAngularRate)));

    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // B: zero yaw — the current heading becomes field-forward (toward the FRONT wall).
    joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("ZeroYaw"));

    // ── DISABLED for vision prototyping (restore from tag m0-hw) ──
    // RobotModeTriggers.teleop().or(RobotModeTriggers.test()).onTrue(elevator.calibrateZero()...);
    // joystick.x().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Ground));
    // joystick.y().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Top));
    // joystick.leftBumper().onTrue(arm.goToSetpoint(() -> Arm.Setpoint.Retracted));
    // joystick.rightBumper().onTrue(arm.goToSetpoint(() -> Arm.Setpoint.Rack));
    // joystick.rightTrigger(0.1).whileTrue(arm.manualDrive(() -> joystick.getRightTriggerAxis() * ArmConstants.kJogDutyCycle));
    // joystick.leftTrigger(0.1).whileTrue(arm.manualDrive(() -> -joystick.getLeftTriggerAxis() * ArmConstants.kJogDutyCycle));
    // joystick.back().onTrue(arm.zeroHere());
    // joystick.povDown().onTrue(pincher.pinch());
    // joystick.povUp().onTrue(pincher.release());

    // Sim self-test (agents): SUBZERO_SIM_VISION_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
    // → the sim truth + odometry start at (4.5, 2.3, 0°); 1 s later the odometry is deliberately reset 1.0 m / −0.5 m /
    //   30° AWAY from the truth, then the robot sweeps in place (0.05 m/s, 0.3 rad/s) for 20 s. Vision must pull Drive/Pose back onto Drive/SimTruthPose:
    //   watch Drive/SimPoseError_m and Drive/SimPoseError_deg in frc/logs/akit_*.wpilog.
    if (RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_VISION_TEST") != null) {
      new Trigger(DriverStation::isEnabled).onTrue(simVisionTest());
    }
  }

  private Command simVisionTest() {
    // Start facing the FRONT wall (+X) at (4.5, 2.3): the LEFT camera (yaw +90°) sees left-wall tags 8/9/6 at y = 4
    // from ~1.7 m; as the robot turns, the RIGHT camera (yaw −90°) sweeps across the front-wall tags 2/3/4.
    final Pose2d kSimStart = new Pose2d(4.5, 2.3, Rotation2d.kZero);
    final Transform2d kInjectedError = new Transform2d(1.0, -0.5, Rotation2d.fromDegrees(30.0));
    final Timer t = new Timer();
    return Commands.sequence(
            Commands.waitSeconds(0.2),
            drivetrain.runOnce(
                () -> {
                  drivetrain.resetSimTruth(kSimStart);
                  drivetrain.resetPose(kSimStart);
                }),
            Commands.waitSeconds(1.0),
            drivetrain.runOnce(
                () -> {
                  Pose2d truth = drivetrain.getSimTruthPose();
                  Pose2d wrong = truth.plus(kInjectedError);
                  drivetrain.resetPose(wrong);
                  Logger.recordOutput("SimVisionTest/injectedFrom", truth);
                  Logger.recordOutput("SimVisionTest/injectedTo", wrong);
                  t.restart();
                }),
            // Slow sweep in place: 0.05 m/s forward + 0.3 rad/s yaw for 20 s (≈ 345°) — both side cameras pan across
            // both tagged walls while the robot stays inside the room.
            drivetrain
                .applyRequest(
                    () -> {
                      Logger.recordOutput("SimVisionTest/elapsed_s", t.get());
                      return robotSpeeds.withSpeeds(new ChassisSpeeds(0.05, 0.0, 0.3));
                    })
                .withTimeout(20.0),
            drivetrain.applyRequest(() -> idle).withTimeout(0.5))
        .withName("SimVisionTest");
  }

  /** Called from Robot.robotPeriodic(); logging only. */
  public void logPeriodic() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
