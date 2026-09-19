package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TaskConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SimSequence;
import frc.robot.commands.Stow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RoomCamera;
import frc.robot.tasks.TaskNtBridge;
import frc.robot.tasks.TaskPrimitives;
import frc.robot.tasks.ToolClient;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wires subsystems, the NT task bridge and the H-21 driver bindings.
 *
 * <p>Safety §3: every automated command is `.until(driverInputActive)`; teleop defaults to
 * {@link DriveConstants#kTeleopScalar} (50 %). Safety §1: homing only runs on an enable edge (A-01).
 */
public class RobotContainer {
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double kMaxAngularRateRadPerSec =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Arm arm;
  private final RoomCamera frontLeftCamera;
  private final RoomCamera frontRightCamera;
  private final ToolClient toolClient;
  private final TaskPrimitives primitives;
  private final TaskNtBridge ntBridge;

  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** Any stick input above the threshold interrupts an automated command (safety §3). */
  private final BooleanSupplier driverInputActive =
      () -> {
        double t = OperatorConstants.kDriverInterruptThreshold;
        return Math.abs(driver.getLeftX()) > t
            || Math.abs(driver.getLeftY()) > t
            || Math.abs(driver.getRightX()) > t;
      };

  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(kMaxSpeedMps * OperatorConstants.kStickDeadband)
          .withRotationalDeadband(kMaxAngularRateRadPerSec * OperatorConstants.kStickDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** True when the orchestrator's headless acceptance driver owns the first enable (A-05). */
  private final boolean simSequenceMode =
      RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_AUTOENABLE") != null;

  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    elevator = new Elevator();
    arm = new Arm();
    frontLeftCamera =
        new RoomCamera(
            VisionConstants.kFrontLeftCameraName,
            VisionConstants.kRobotToFrontLeftCamera,
            drivetrain::getPose);
    frontRightCamera =
        new RoomCamera(
            VisionConstants.kFrontRightCameraName,
            VisionConstants.kRobotToFrontRightCamera,
            drivetrain::getPose);
    drivetrain.setCameras(frontLeftCamera, frontRightCamera);

    toolClient = new ToolClient();
    // The sink reads the ntBridge field at call time, so the construction order below is safe.
    primitives =
        new TaskPrimitives(
            drivetrain, elevator, arm, toolClient, driverInputActive, this::onTaskState);
    ntBridge = new TaskNtBridge(drivetrain, elevator, arm, toolClient, primitives);

    configureDefaultCommands();
    configureBindings();
    configureHoming();
    Logger.recordOutput("Robot/SimSequenceMode", simSequenceMode);
  }

  private void onTaskState(TaskPrimitives.TaskReport report) {
    if (ntBridge != null) {
      ntBridge.publishState(report);
    }
  }

  private void configureDefaultCommands() {
    double s = DriveConstants.kTeleopScalar;
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(
                () ->
                    fieldCentric
                        .withVelocityX(-driver.getLeftY() * kMaxSpeedMps * s)
                        .withVelocityY(-driver.getLeftX() * kMaxSpeedMps * s)
                        .withRotationalRate(-driver.getRightX() * kMaxAngularRateRadPerSec * s))
            .withName("TeleopDrive"));
  }

  /** A-01: home on the first teleop/test enable — unless SimSequence homes itself. */
  private void configureHoming() {
    if (simSequenceMode) {
      new Trigger(DriverStation::isEnabled).onTrue(new SimSequence(drivetrain, elevator, arm));
      return;
    }
    RobotModeTriggers.teleop()
        .or(RobotModeTriggers.test())
        .onTrue(
            Commands.sequence(arm.home(), elevator.home())
                .unless(() -> elevator.isHomed() && arm.isHomed())
                .withName("HomeOnEnable"));
  }

  /** H-21: A = alignToTag(nearest), B = stow, X = tool latch, Y = tool release. */
  private void configureBindings() {
    driver.a().onTrue(Commands.defer(this::alignToNearestTag, Set.of(drivetrain)));
    driver.b().onTrue(new Stow(elevator, arm).until(driverInputActive).withName("StowButton"));
    driver.x().onTrue(primitives.toolOp(TaskConstants.kDefaultToolId, "latch", 0));
    driver.y().onTrue(primitives.toolOp(TaskConstants.kDefaultToolId, "release", 0));
  }

  /** First non-empty getBestVisibleTagId() across the cameras, else a "no tag" print. */
  private Command alignToNearestTag() {
    Optional<Integer> tag = Optional.empty();
    for (RoomCamera cam : drivetrain.getCameras()) {
      tag = cam.getBestVisibleTagId();
      if (tag.isPresent()) {
        break;
      }
    }
    if (tag.isEmpty()) {
      return Commands.print("alignToTag: no tag");
    }
    return primitives.alignToTag(
        tag.get(),
        TaskConstants.kPickOffsetX_m,
        TaskConstants.kPickOffsetY_m,
        TaskConstants.kPickYaw_deg);
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
