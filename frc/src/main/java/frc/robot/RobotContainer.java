package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.AutoConstants;
import frc.robot.CarouselConstants.CarouselSlot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.TagTourAuto;
import frc.robot.commands.AutoSimHooks;
import frc.robot.commands.Superstructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincher;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Configuration (2026-09-20): DRIVETRAIN + VISION + ELEVATOR + ARM (incremental control).
 *
 * <p>The two RoomCameras ("photoncamera_left" / "photoncamera_right") live inside the drivetrain,
 * Rebuilt2026-style, and fuse into its pose estimator in {@code updatePose()}. {@link RobotState}
 * publishes the fused pose / velocity / tag distances (Rebuilt2026 topic names) for AdvantageScope.
 *
 * <p>Carousel demo (mock geometry in {@link frc.robot.CarouselConstants}): A = dock at LEVEL_1, B = dock at
 * LEVEL_2, X = dock at LEVEL_1 then grab from LEVEL_2 — with real pinch / un-pinch (measured servo angles).
 * RB / LB = pinch / release by hand. {@link Superstructure#setEndpointPosition} moves both mechanisms together.
 * D-pad = continuous jog while held (up / down = elevator, right / left = arm), stops on release.
 * Both mechanisms hold their targets with MotionMagic, cruise capped at 0.75 m/s. The elevator calibrates its
 * zero on the first teleop/test enable; the arm's zero is its power-on position. The pincher boots OPEN.
 *
 * <p>Controls: left stick = translate (field-centric), right stick X = rotate; Start = zero yaw
 * (seedFieldCentric — current heading becomes "forward" = toward the FRONT wall; moved from B).
 */
public class RobotContainer {
  private final double kMaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.kTeleopScalar; // safety §2
  private final double kMaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.DriveConstants.kTeleopRotationScalar;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // Stick deadband is applied per axis (with rescaling) in the default command; this is only a tiny backstop.
          .withDeadband(kMaxSpeed * 0.02)
          .withRotationalDeadband(kMaxAngularRate * 0.02)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  /** Drivetrain — constructs and fuses the two RoomCameras (Rebuilt2026 pattern). */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final RobotState robotState = RobotState.getInstance();

  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // ───────────── elevator + arm (incremental jog control) ─────────────
  public final Elevator elevator = new Elevator();
  private boolean elevatorCalibrated = false;
  public final Arm arm = new Arm();
  public final Pincher pincher = new Pincher();
  public final Superstructure superstructure = new Superstructure(elevator, arm, pincher);
  private static final edu.wpi.first.units.measure.Distance kJogStep = Inches.of(1.0);

  // Pincher servos on PWM 8/9: angles follow NetworkTables /SmartDashboard/Pincher/servo{A,B}_deg every loop
  // (dashboard override for testing); pinch()/release() write the measured closed/open pairs into those entries.

  public RobotContainer() {
    robotState.setAllSuppliers(drivetrain::getPose, drivetrain::getChassisSpeeds);
    configureBindings();
    AutoSimHooks.install(drivetrain, elevator, arm); // sim-only env hooks (SUBZERO_SIM_ALIGN_TEST / SPIN_TEST / POKE_TEST)
    // Sim tour test: SUBZERO_SIM_TOUR_TEST=1 SUBZERO_SIM_AUTOENABLE=auto → truth at the room centre, odometry deliberately
    // 0.5 m / 20° off, then the autonomous routine runs (Robot.autonomousInit schedules getAutonomousCommand()).
    if (RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_TOUR_TEST") != null) {
      final Pose2d truth = new Pose2d(AutoConstants.kFieldCenter, Rotation2d.kZero);
      drivetrain.resetSimTruth(truth);
      drivetrain.resetPose(truth.plus(new Transform2d(0.5, 0.0, Rotation2d.fromDegrees(20.0))));
    }
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-stick(joystick.getLeftY()) * kMaxSpeed)
                    .withVelocityY(-stick(joystick.getLeftX()) * kMaxSpeed)
                    .withRotationalRate(-stick(joystick.getRightX()) * kMaxAngularRate)));

    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // MENU (☰, WPILib "start", button 8): reset gyro / zero yaw — the current heading becomes field-forward (front wall).
    joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("ZeroYaw"));

    // Elevator: calibrate zero on the FIRST teleop/test enable (A-01), never while disabled.
    RobotModeTriggers.teleop()
        .or(RobotModeTriggers.test())
        .onTrue(
            elevator
                .calibrateZero()
                .andThen(Commands.runOnce(() -> elevatorCalibrated = true))
                .unless(() -> elevatorCalibrated)
                .withName("ElevatorCalibrateZero"));

    // Carousel demo sequences (mock geometry, CarouselConstants). A new sequence interrupts a running one.
    // Gated on elevatorCalibrated so a press during the first-enable calibration cannot cancel it (bad zero).
    joystick.a().onTrue(superstructure.dockToCarousel(CarouselSlot.LEVEL_1).onlyIf(() -> elevatorCalibrated));
    joystick.b().onTrue(superstructure.dockToCarousel(CarouselSlot.LEVEL_2).onlyIf(() -> elevatorCalibrated));
    joystick
        .x()
        .onTrue(
            superstructure
                .dockToCarousel(CarouselSlot.LEVEL_1)
                .andThen(superstructure.grabFromCarousel(CarouselSlot.LEVEL_2))
                .withName("DockL1ThenGrabL2")
                .onlyIf(() -> elevatorCalibrated));

    // Manual pinch / release for bench checks (the choreography calls the same commands).
    joystick.rightBumper().onTrue(pincher.pinch());
    joystick.leftBumper().onTrue(pincher.release());

    // D-pad: continuous jog while held, stop on release (targets move at kElevatorJogRateMps / kArmJogRateMps).
    joystick.povUp().whileTrue(elevator.jogContinuous(OperatorConstants.kElevatorJogRateMps));    // elevator up
    joystick.povDown().whileTrue(elevator.jogContinuous(-OperatorConstants.kElevatorJogRateMps)); // elevator down
    joystick.povRight().whileTrue(arm.jogContinuous(OperatorConstants.kArmJogRateMps));           // arm out (right)
    joystick.povLeft().whileTrue(arm.jogContinuous(-OperatorConstants.kArmJogRateMps));           // arm in

    // ── DISABLED (restore from tag m0-hw) ──
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

  /** Per-axis stick deadband (OperatorConstants.kStickDeadband), rescaled so motion starts smoothly from zero. */
  private static double stick(double raw) {
    return MathUtil.applyDeadband(raw, OperatorConstants.kStickDeadband);
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

  /**
   * Sim self-test (agents): SUBZERO_SIM_JOG_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
   * → calibrate, then continuous jogs: elevator +2 s / −1 s, arm +2 s / −1 s at the D-pad rates;
   * expect Elevator/height_m ≈ 0.15 m and Arm/extension_m ≈ 0.15 m at the end.
   */
  {
    if (RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_JOG_TEST") != null) {
      new Trigger(DriverStation::isEnabled)
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(3.0), // let calibrateZero finish
                      elevator.jogContinuous(OperatorConstants.kElevatorJogRateMps).withTimeout(2.0), // +0.30 m
                      Commands.waitSeconds(1.0),
                      elevator.jogContinuous(-OperatorConstants.kElevatorJogRateMps).withTimeout(1.0), // −0.15 m
                      Commands.waitSeconds(1.0),
                      arm.jogContinuous(OperatorConstants.kArmJogRateMps).withTimeout(2.0), // +0.30 m
                      Commands.waitSeconds(1.0),
                      arm.jogContinuous(-OperatorConstants.kArmJogRateMps).withTimeout(1.0), // −0.15 m
                      Commands.waitSeconds(1.0))
                  .withName("SimJogTest"));
    }
  }

  /**
   * Sim self-test (agents): SUBZERO_SIM_CAROUSEL_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
   * → the X-button sequence (dock LEVEL_1, then grab LEVEL_2); watch Superstructure/step, Elevator/height_m,
   * Arm/extension_m in frc/logs/akit_*.wpilog.
   */
  {
    if (RobotBase.isSimulation() && System.getenv("SUBZERO_SIM_CAROUSEL_TEST") != null) {
      // Starts once calibrateZero has finished (the sequence requires the elevator, so it must not be scheduled
      // on the same enable edge as the calibration command).
      new Trigger(() -> elevatorCalibrated)
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(1.0),
                      superstructure.dockToCarousel(CarouselSlot.LEVEL_1),
                      superstructure.grabFromCarousel(CarouselSlot.LEVEL_2))
                  .withName("SimCarouselTest"));
    }
  }

  /** Called from Robot.robotPeriodic(); logging only. */
  public void logPeriodic() {
    Logger.recordOutput("Elevator/calibrated", elevatorCalibrated);
  }

  /** Autonomous = the tag tour (spin → visit every tag: path, camera-align, poke, path back). */
  public Command getAutonomousCommand() {
    return TagTourAuto.create(drivetrain, elevator, arm, () -> elevatorCalibrated, () -> elevatorCalibrated = true);
  }
}
