package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

/**
 * M0 hardware configuration (2026-09-19): DRIVETRAIN + ELEVATOR + ARM.
 *
 * <p>Everything else (RoomCamera, AlignToTagCommand, Stow, SimSequence, tasks/*) is commented out —
 * the full M1 wiring is at git tag {@code m1-sim}.
 *
 * <p>Controls: left stick = translate (field-centric), right stick X = rotate; B = zero yaw
 * (seedFieldCentric — current heading becomes "forward"); X = elevator to Ground (6 rot);
 * Y = elevator to Top (7 rot); LB = arm Retracted (0 m); RB = arm Rack (0.15 m); right/left trigger =
 * jog the arm out/in at ≤ 15 % duty; Back = re-zero the arm at its current position. The elevator
 * calibrates its zero (hard stop, −10 % duty) on the first teleop/test enable (A-01); the arm's zero
 * is wherever it sits at power-on (no switch).
 */
public class RobotContainer {
  // ───────────── drivetrain (W1) ─────────────
  private final double kMaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.kTeleopScalar; // safety §2: 50 % teleop default
  private final double kMaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.DriveConstants.kTeleopScalar;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(kMaxSpeed * OperatorConstants.kStickDeadband)
          .withRotationalDeadband(kMaxAngularRate * OperatorConstants.kStickDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // ───────────── elevator (CTREELEVATOR config, exact) ─────────────
  public final Elevator elevator = new Elevator();
  private boolean elevatorCalibrated = false;

  // ───────────── arm (single Kraken X60, CAN 40, linear axis, no switches) ─────────────
  public final Arm arm = new Arm();

  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // ───────────── commented out for M0 (restore from tag m1-sim) ─────────────
  // private final RoomCamera camFL = new RoomCamera(VisionConstants.kFrontLeftCameraName, VisionConstants.kRobotToFrontLeftCamera, drivetrain::getPose);
  // private final RoomCamera camFR = new RoomCamera(VisionConstants.kFrontRightCameraName, VisionConstants.kRobotToFrontRightCamera, drivetrain::getPose);
  // private final ToolClient toolClient = new ToolClient();
  // private final TaskPrimitives primitives = ...;
  // private final TaskNtBridge ntBridge = ...;
  // private final Command simSequence = new SimSequence(...);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Two sticks: left = translate, right X = rotate. forward = -Y, left = -X, CCW = -X.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * kMaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * kMaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * kMaxAngularRate)));

    // Idle while disabled so the configured neutral mode is applied (safety §1).
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // B: zero yaw — the current heading becomes field-forward.
    joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("ZeroYaw"));

    // Elevator: calibrate zero on the FIRST teleop/test enable (A-01), never while disabled.
    RobotModeTriggers.teleop()
        .or(RobotModeTriggers.test())
        .onTrue(
            elevator
                .calibrateZero()
                .andThen(Commands.runOnce(() -> elevatorCalibrated = true))
                .unless(() -> elevatorCalibrated)
                .withName("ElevatorCalibrateZero"));

    // X / Y: two setpoints other than the homed zero (Ground = 6 rot, Top = 7 rot — CTREELEVATOR
    // Setpoint enum). goToSetpoint runs until the other button replaces it, so it holds there.
    joystick.x().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Ground).withName("ElevatorGround"));
    joystick.y().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Top).withName("ElevatorTop"));

    // Arm: LB = retracted, RB = rack (0.15 m); each holds until replaced (default = hold in place).
    joystick.leftBumper().onTrue(arm.goToSetpoint(() -> Arm.Setpoint.Retracted).withName("ArmRetracted"));
    joystick.rightBumper().onTrue(arm.goToSetpoint(() -> Arm.Setpoint.Rack).withName("ArmRack"));
    // Triggers: manual jog (right = extend, left = retract) at ≤ kJogDutyCycle; soft limits still apply.
    joystick
        .rightTrigger(0.1)
        .whileTrue(arm.manualDrive(() -> joystick.getRightTriggerAxis() * ArmConstants.kJogDutyCycle));
    joystick
        .leftTrigger(0.1)
        .whileTrue(arm.manualDrive(() -> -joystick.getLeftTriggerAxis() * ArmConstants.kJogDutyCycle));
    // Back: re-zero the arm at its current position (do this fully retracted before extending).
    joystick.back().onTrue(arm.zeroHere());

    // Sim self-test (agents): SUBZERO_SIM_ARM_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
    // → arm to Rack for 2 s, then Retracted; read Arm/* in frc/logs/akit_*.wpilog.
    if (RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_ARM_TEST") != null) {
      new Trigger(DriverStation::isEnabled)
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(0.5),
                      arm.goToSetpoint(() -> Arm.Setpoint.Rack).withTimeout(2.0),
                      arm.goToSetpoint(() -> Arm.Setpoint.Retracted).withTimeout(2.0))
                  .withName("SimArmTest"));
    }

    // ── commented out for M0 (H-21 M1 bindings; restore from tag m1-sim) ──
    // joystick.a().onTrue(alignToNearestTag());
    // joystick.b().onTrue(new Stow(elevator, arm));
    // joystick.x().onTrue(primitives.toolOp(TaskConstants.kDefaultToolId, "latch", 0));
    // joystick.y().onTrue(primitives.toolOp(TaskConstants.kDefaultToolId, "release", 0));
    // new Trigger(DriverStation::isEnabled).onTrue(simSequence);   // SUBZERO_SIM_AUTOENABLE mode
  }

  /** Called from Robot.robotPeriodic(); logging only. */
  public void logPeriodic() {
    Logger.recordOutput("Elevator/calibrated", elevatorCalibrated);
    Logger.recordOutput("Arm/zeroed", true); // zero is defined at power-on / Back
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
