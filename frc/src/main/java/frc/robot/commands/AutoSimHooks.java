package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RoomCamera;
import org.littletonrobotics.junction.Logger;

/**
 * Sim-only self-tests for the tag-tour building blocks (agents / frc-mcp). Call {@link #install} once at the end of
 * the RobotContainer constructor; it is a no-op on the robot and when no test env var is set. Run ONE test at a time:
 *
 * <pre>
 * SUBZERO_SIM_ALIGN_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
 *   1 s after enable: sim truth + odometry → AutoConstants.stagingPose(tag 3, kStagingDistance), then
 *   AlignRightCameraToTag(tag 3, kPokeStandoff). Watch Auto/Align/* and Drive/SimTruthPose vs Auto/Align/GoalPose.
 *   Optional overrides: SUBZERO_SIM_ALIGN_TAG=&lt;id&gt;, SUBZERO_SIM_ALIGN_STAGING_M=&lt;m&gt;,
 *   SUBZERO_SIM_ALIGN_STANDOFF_M=&lt;m&gt;, SUBZERO_SIM_ALIGN_PERTURB=dx,dy,ddeg (robot-frame offset added to the
 *   start pose so the x / rotation loops are exercised too), SUBZERO_SIM_ALIGN_BRIDGE=1 (odometry bridge ON).
 * SUBZERO_SIM_SPIN_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
 *   1 s after enable: sim truth + odometry → room centre facing +X, then SpinToLocalize. Watch Auto/Spin/*.
 * SUBZERO_SIM_POKE_TEST=1 SUBZERO_SIM_AUTOENABLE=teleop ./gradlew simulateJava -Pheadless
 *   3 s after enable (elevator calibration done): PokeTag. Watch Auto/Poke/step, Elevator/height_m, Arm/extension_m.
 * </pre>
 */
public final class AutoSimHooks {
  private AutoSimHooks() {}

  public static void install(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm) {
    if (!RobotBase.isSimulation()) return;

    if (System.getenv("SUBZERO_SIM_ALIGN_TEST") != null) {
      new Trigger(DriverStation::isEnabled).onTrue(alignTest(drivetrain));
    }
    if (System.getenv("SUBZERO_SIM_SPIN_TEST") != null) {
      new Trigger(DriverStation::isEnabled).onTrue(spinTest(drivetrain));
    }
    if (System.getenv("SUBZERO_SIM_POKE_TEST") != null) {
      // PokeTag requires the elevator + arm; proxy it so this sequence holds no requirements on the enable edge
      // (the first-enable calibrateZero must not be interrupted) — the proxy schedules PokeTag 3 s later.
      new Trigger(DriverStation::isEnabled)
          .onTrue(
              Commands.sequence(Commands.waitSeconds(3.0), new PokeTag(elevator, arm).asProxy())
                  .withName("SimPokeTest"));
    }
  }

  private static Command alignTest(CommandSwerveDrivetrain drivetrain) {
    final int tagId = envInt("SUBZERO_SIM_ALIGN_TAG", 3);
    final Distance staging =
        Meters.of(envDouble("SUBZERO_SIM_ALIGN_STAGING_M", AutoConstants.kStagingDistance.in(Meters)));
    final Distance standoff =
        Meters.of(envDouble("SUBZERO_SIM_ALIGN_STANDOFF_M", AutoConstants.kPokeStandoff.in(Meters)));
    final Transform2d perturb = envTransform("SUBZERO_SIM_ALIGN_PERTURB");
    final boolean bridge = System.getenv("SUBZERO_SIM_ALIGN_BRIDGE") != null;

    final var tagPose3d = RoomCamera.getRoomLayout().getTagPose(tagId);
    if (tagPose3d.isEmpty()) {
      return Commands.print("AutoSimHooks: tag " + tagId + " is not in the room layout — align test skipped");
    }
    final Pose2d tagPose = tagPose3d.get().toPose2d();
    final Pose2d start = AutoConstants.stagingPose(tagPose, staging).plus(perturb);
    final Pose2d goal = AutoConstants.stagingPose(tagPose, standoff);
    final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    return Commands.sequence(
            Commands.waitSeconds(1.0),
            drivetrain.runOnce(
                () -> {
                  drivetrain.resetSimTruth(start);
                  drivetrain.resetPose(start);
                  Logger.recordOutput("Auto/SimTest/StartPose", start);
                  Logger.recordOutput("Auto/SimTest/GoalPose", goal);
                  System.out.printf(
                      "AutoSimHooks: align test tag %d, start %s, goal %s, bridge=%b%n", tagId, start, goal, bridge);
                }),
            new AlignRightCameraToTag(drivetrain, tagId, standoff, bridge),
            drivetrain.applyRequest(() -> idle).withTimeout(0.5))
        .withName("SimAlignTest");
  }

  private static Command spinTest(CommandSwerveDrivetrain drivetrain) {
    final Pose2d start = new Pose2d(AutoConstants.kFieldCenter, Rotation2d.kZero);
    final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            drivetrain.runOnce(
                () -> {
                  drivetrain.resetSimTruth(start);
                  drivetrain.resetPose(start);
                  Logger.recordOutput("Auto/SimTest/StartPose", start);
                }),
            new SpinToLocalize(drivetrain),
            drivetrain.applyRequest(() -> idle).withTimeout(0.5))
        .withName("SimSpinTest");
  }

  private static int envInt(String name, int fallback) {
    final String v = System.getenv(name);
    if (v == null || v.isBlank()) return fallback;
    try {
      return Integer.parseInt(v.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  private static double envDouble(String name, double fallback) {
    final String v = System.getenv(name);
    if (v == null || v.isBlank()) return fallback;
    try {
      return Double.parseDouble(v.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  /** "dx,dy,ddeg" → robot-frame Transform2d; identity when unset / malformed. */
  private static Transform2d envTransform(String name) {
    final String v = System.getenv(name);
    if (v == null || v.isBlank()) return Transform2d.kZero;
    try {
      final String[] parts = v.split(",");
      return new Transform2d(
          Double.parseDouble(parts[0].trim()),
          Double.parseDouble(parts[1].trim()),
          Rotation2d.fromDegrees(Double.parseDouble(parts[2].trim())));
    } catch (RuntimeException e) {
      return Transform2d.kZero;
    }
  }
}
