package frc.robot.autos;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoConstants;
import frc.robot.commands.AlignRightCameraToTag;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.PokeTag;
import frc.robot.commands.SpinToLocalize;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RoomCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * "Tag tour" autonomous — demonstrates that the robot understands the AprilTags around it. VERY slow: translation
 * ≤ 0.47 m/s (10 %), rotation ≤ 120 °/s ({@link AutoConstants}). No pincher use.
 *
 * <ol>
 *   <li>Calibrate the elevator zero (if the first-enable calibration has not run yet).
 *   <li>Spin 360° in place (~3 s) so both cameras sweep every tag → the GLOBAL fused pose settles.
 *   <li>For each tag in {@link AutoConstants#kTourTagIds} (ascending, ID 5 skipped):
 *       <ol>
 *         <li>PathPlanner (global pose) to the staging pose 1.2 m in front of the tag, RIGHT side toward it.
 *         <li>{@link AlignRightCameraToTag}: holonomic control on the RIGHT camera's view of that tag ONLY, to the
 *             poke stand-off.
 *         <li>{@link PokeTag}: arm at 0 → elevator up to the cap → arm out (to the right, into the tag) → arm back →
 *             elevator down. Order protects the carousel.
 *         <li>PathPlanner (global pose) back to the room centre.
 *       </ol>
 * </ol>
 *
 * <p>AdvantageScope: every planned path is on {@code PathPlanner/ActivePath} (Pose2d[] → add as a Trajectory),
 * the whole tour plan is on {@code Auto/PlanPoses} once at the start, the current step on {@code Auto/Step}, the
 * camera-align goal on {@code Auto/Align/GoalPose}.
 */
public final class TagTourAuto {
  private TagTourAuto() {}

  public static Command create(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Arm arm,
      BooleanSupplier elevatorCalibrated,
      Runnable markElevatorCalibrated) {
    final AprilTagFieldLayout layout = RoomCamera.getRoomLayout();
    final Pose2d center = new Pose2d(AutoConstants.kFieldCenter, Rotation2d.kZero);

    List<Command> steps = new ArrayList<>();
    List<Pose2d> plan = new ArrayList<>();
    plan.add(center);

    steps.add(
        step("0 calibrate elevator",
            elevator.calibrateZero().andThen(Commands.runOnce(markElevatorCalibrated)).unless(elevatorCalibrated)));
    steps.add(step("1 spin 360 to localise", new SpinToLocalize(drivetrain)));

    for (int id : AutoConstants.kTourTagIds) {
      var tagPose3d = layout.getTagPose(id);
      if (tagPose3d.isEmpty()) {
        steps.add(step("tag " + id + " not in layout — skipped", Commands.none()));
        continue;
      }
      final Pose2d tagPose = tagPose3d.get().toPose2d();
      final Pose2d staging = AutoConstants.stagingPose(tagPose, AutoConstants.kStagingDistance);
      final Pose2d aligned = AutoConstants.stagingPose(tagPose, AutoConstants.kPokeStandoff);
      plan.add(staging);
      plan.add(aligned);
      plan.add(center);

      final AlignRightCameraToTag align =
          new AlignRightCameraToTag(
              drivetrain, id, AutoConstants.kPokeStandoff, AutoConstants.kAlignOdometryBridge);
      steps.add(step("tag " + id + " /1 path to staging (global pose)", DriveToPose.driveToPose(drivetrain, staging)));
      steps.add(step("tag " + id + " /2 camera-align right side to tag", align));
      // Never poke blind: skip the poke if the camera alignment did not succeed (logged as Auto/Align/failReason).
      steps.add(step("tag " + id + " /3 poke (only if aligned)", new PokeTag(elevator, arm).onlyIf(align::succeeded)));
      steps.add(step("tag " + id + " /4 path back to centre (global pose)", DriveToPose.driveToPose(drivetrain, center)));
    }
    steps.add(step("tour done", Commands.none()));

    final Pose2d[] planArray = plan.toArray(new Pose2d[0]);
    return Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("Auto/PlanPoses", planArray)),
            Commands.sequence(steps.toArray(new Command[0])))
        .withName("TagTourAuto");
  }

  private static Command step(String name, Command inner) {
    return Commands.runOnce(
            () -> {
              System.out.println("TagTourAuto: " + name);
              Logger.recordOutput("Auto/Step", name);
            })
        .andThen(inner);
  }
}
