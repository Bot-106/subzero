// COMMENTED OUT 2026-09-19 (M0 hardware config: drivetrain + elevator only). Restore with: sed -i '' '1d;s|^// ||' commands/SimSequence.java  — full version at git tag m1-sim.
// package frc.robot.commands;
// 
// import static edu.wpi.first.units.Units.Meters;
// 
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.SimConstants;
// import frc.robot.Constants.TaskConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator;
// import java.io.IOException;
// import java.util.LinkedHashMap;
// import java.util.Map;
// import java.util.Optional;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import org.littletonrobotics.junction.Logger;
// 
// /**
//  * Headless sim acceptance driver (interface-and-demo §4 tests 1–2). Built only when
//  * `RobotBase.isSimulation() && SUBZERO_SIM_AUTOENABLE` is set, scheduled once on the first enable.
//  *
//  * <p>Steps run in order, each time-boxed, each CONTINUING to the next even on failure: home →
//  * elevator(0.8 m) → arm(0.3 m) → align(tag 3, pick offset, from 1.5 m away) → stow. Every step logs
//  * `SimSequence/<step>/{ok,seconds,error}` and prints one greppable line
//  * `SimSequence: <step> ok=<bool> seconds=<x> error=<y>`; a parallel monitor logs the running maxima
//  * `SimSequence/elevator/maxHeight_m` and `SimSequence/arm/maxExtension_m` as soft-limit evidence.
//  */
// public class SimSequence extends SequentialCommandGroup {
//   private static final double kStepTimeoutSeconds = Constants.TaskConstants.kMechanismTimeoutSeconds;
// 
//   private final CommandSwerveDrivetrain drivetrain;
//   private final Elevator elevator;
//   private final Arm arm;
// 
//   private final Map<String, String> results = new LinkedHashMap<>();
//   private double maxHeightM = Double.NEGATIVE_INFINITY;
//   private double maxExtensionM = Double.NEGATIVE_INFINITY;
// 
//   /** Target robot pose for the align step (tag 3 + pick offset), if the layout loaded. */
//   private final Optional<Pose2d> alignTarget;
// 
//   public SimSequence(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm) {
//     this.drivetrain = drivetrain;
//     this.elevator = elevator;
//     this.arm = arm;
//     this.alignTarget = computeAlignTarget();
//     setName("SimSequence");
// 
//     Command steps =
//         Commands.sequence(
//             Commands.runOnce(
//                 () -> {
//                   results.clear();
//                   maxHeightM = Double.NEGATIVE_INFINITY;
//                   maxExtensionM = Double.NEGATIVE_INFINITY;
//                   Logger.recordOutput("SimSequence/done", false);
//                   Logger.recordOutput("SimSequence/summary", "");
//                   System.out.println("SimSequence: start");
//                 }),
//             homeStep(),
//             elevatorStep(),
//             armStep(),
//             alignStep(),
//             stowStep(),
//             Commands.runOnce(this::finish));
// 
//     // deadline: the monitor is killed when the steps finish; the group never holds a subsystem.
//     addCommands(Commands.deadline(steps, monitor()));
//   }
// 
//   // ───────────────────────────── steps ─────────────────────────────
// 
//   private Command homeStep() {
//     return step(
//         "home",
//         Commands.parallel(arm.home(), elevator.home()),
//         kStepTimeoutSeconds,
//         () -> elevator.isHomed() && arm.isHomed(),
//         () -> (elevator.isHomed() ? 0.0 : 1.0) + (arm.isHomed() ? 0.0 : 1.0));
//   }
// 
//   private Command elevatorStep() {
//     double target = SimConstants.kSimElevatorTarget.in(Meters);
//     DoubleSupplier err = () -> Math.abs(elevator.getHeight().in(Meters) - target);
//     return step(
//         "elevator",
//         elevator.goTo(SimConstants.kSimElevatorTarget),
//         kStepTimeoutSeconds,
//         () -> err.getAsDouble() < ElevatorConstants.kTolerance.in(Meters),
//         err);
//   }
// 
//   private Command armStep() {
//     double target = SimConstants.kSimArmTarget.in(Meters);
//     DoubleSupplier err = () -> Math.abs(arm.getExtension().in(Meters) - target);
//     return step(
//         "arm",
//         arm.goTo(SimConstants.kSimArmTarget),
//         kStepTimeoutSeconds,
//         () -> err.getAsDouble() < ArmConstants.kTolerance.in(Meters),
//         err);
//   }
// 
//   private Command alignStep() {
//     if (alignTarget.isEmpty()) {
//       return step(
//           "align",
//           Commands.print("SimSequence: align skipped — room layout did not load"),
//           1.0,
//           () -> false,
//           () -> Double.NaN);
//     }
//     Pose2d target = alignTarget.get();
//     Pose2d start = target.transformBy(new Transform2d(-1.2, -0.9, Rotation2d.fromDegrees(20)));
//     Pose2d offset =
//         new Pose2d(
//             TaskConstants.kPickOffsetX_m,
//             TaskConstants.kPickOffsetY_m,
//             Rotation2d.fromDegrees(TaskConstants.kPickYaw_deg));
// 
//     DoubleSupplier errM = () -> drivetrain.getPose().getTranslation().getDistance(target.getTranslation());
//     DoubleSupplier errDeg =
//         () -> Math.abs(drivetrain.getPose().getRotation().minus(target.getRotation()).getDegrees());
//     Command logErr =
//         Commands.run(
//             () -> {
//               Logger.recordOutput("SimSequence/align/error_m", errM.getAsDouble());
//               Logger.recordOutput("SimSequence/align/error_deg", errDeg.getAsDouble());
//             });
// 
//     Command inner =
//         Commands.sequence(
//             Commands.runOnce(
//                 () -> {
//                   Logger.recordOutput("SimSequence/align/target", target);
//                   Logger.recordOutput("SimSequence/align/start", start);
//                   drivetrain.resetPose(start);
//                 }),
//             Commands.waitSeconds(0.5),
//             Commands.deadline(
//                 drivetrain.alignToTag(SimConstants.kSimAlignTagId, offset)
//                     .withTimeout(DriveConstants.kAlignTimeoutSeconds),
//                 logErr));
// 
//     return step(
//         "align",
//         inner,
//         DriveConstants.kAlignTimeoutSeconds + 1.0,
//         () ->
//             errM.getAsDouble() < DriveConstants.kAlignTolerance.in(Meters)
//                 && errDeg.getAsDouble() < DriveConstants.kAlignToleranceDeg,
//         errM,
//         errDeg);
//   }
// 
//   private Command stowStep() {
//     DoubleSupplier errH =
//         () -> Math.abs(elevator.getHeight().in(Meters) - ElevatorConstants.kStowHeight.in(Meters));
//     DoubleSupplier errE =
//         () -> Math.abs(arm.getExtension().in(Meters) - ArmConstants.kRetracted.in(Meters));
//     return step(
//         "stow",
//         new Stow(elevator, arm),
//         kStepTimeoutSeconds,
//         () ->
//             errH.getAsDouble() < ElevatorConstants.kTolerance.in(Meters)
//                 && errE.getAsDouble() < ArmConstants.kTolerance.in(Meters),
//         () -> Math.max(errH.getAsDouble(), errE.getAsDouble()));
//   }
// 
//   // ───────────────────────────── plumbing ─────────────────────────────
// 
//   private Command step(
//       String name, Command inner, double timeout, BooleanSupplier ok, DoubleSupplier error) {
//     return step(name, inner, timeout, ok, error, null);
//   }
// 
//   /** One time-boxed step; `errorDeg` (optional) is only used for the summary line of `align`. */
//   private Command step(
//       String name,
//       Command inner,
//       double timeout,
//       BooleanSupplier ok,
//       DoubleSupplier error,
//       DoubleSupplier errorDeg) {
//     Timer timer = new Timer();
//     return Commands.sequence(
//         Commands.runOnce(
//             () -> {
//               timer.restart();
//               Logger.recordOutput("SimSequence/step", name);
//             }),
//         inner.withTimeout(timeout),
//         Commands.runOnce(
//             () -> {
//               double seconds = timer.get();
//               boolean pass = ok.getAsBoolean();
//               double err = error.getAsDouble();
//               Logger.recordOutput("SimSequence/" + name + "/ok", pass);
//               Logger.recordOutput("SimSequence/" + name + "/seconds", seconds);
//               Logger.recordOutput("SimSequence/" + name + "/error", err);
//               String summary;
//               if (pass) {
//                 summary = "OK";
//               } else if (errorDeg != null) {
//                 summary =
//                     String.format("FAIL(%.2fm/%.1fdeg)", err, errorDeg.getAsDouble());
//               } else {
//                 summary = String.format("FAIL(%.3f)", err);
//               }
//               results.put(name, summary);
//               System.out.println(
//                   String.format(
//                       "SimSequence: %s ok=%b seconds=%.2f error=%.4f", name, pass, seconds, err));
//             }));
//   }
// 
//   /** Samples soft-limit evidence every loop while the steps run. */
//   private Command monitor() {
//     return Commands.run(
//         () -> {
//           maxHeightM = Math.max(maxHeightM, elevator.getHeight().in(Meters));
//           maxExtensionM = Math.max(maxExtensionM, arm.getExtension().in(Meters));
//           Logger.recordOutput("SimSequence/elevator/maxHeight_m", maxHeightM);
//           Logger.recordOutput("SimSequence/arm/maxExtension_m", maxExtensionM);
//           Logger.recordOutput(
//               "SimSequence/elevator/underSoftLimit",
//               maxHeightM <= ElevatorConstants.kSoftLimitTop.in(Meters));
//           Logger.recordOutput(
//               "SimSequence/arm/underSoftLimit",
//               maxExtensionM <= ArmConstants.kSoftLimitOut.in(Meters));
//         });
//   }
// 
//   private void finish() {
//     StringBuilder b = new StringBuilder();
//     results.forEach(
//         (k, v) -> {
//           if (b.length() > 0) {
//             b.append(' ');
//           }
//           b.append(k).append('=').append(v);
//         });
//     String summary = b.toString();
//     Logger.recordOutput("SimSequence/summary", summary);
//     Logger.recordOutput("SimSequence/done", true);
//     Logger.recordOutput("SimSequence/step", "done");
//     System.out.println("SimSequence: summary " + summary);
//     System.out.println(
//         String.format(
//             "SimSequence: maxHeight_m=%.3f (softLimitTop=%.3f) maxExtension_m=%.3f (softLimitOut=%.3f)",
//             maxHeightM,
//             ElevatorConstants.kSoftLimitTop.in(Meters),
//             maxExtensionM,
//             ArmConstants.kSoftLimitOut.in(Meters)));
//   }
// 
//   /** Tag 3 pose from the deploy layout + the pick offset; placeholder layout → (4.0, 5.55, 90°). */
//   private static Optional<Pose2d> computeAlignTarget() {
//     try {
//       AprilTagFieldLayout layout =
//           new AprilTagFieldLayout(
//               Filesystem.getDeployDirectory().toPath().resolve(VisionConstants.kRoomLayoutFile));
//       return layout
//           .getTagPose(SimConstants.kSimAlignTagId)
//           .map(
//               p3 ->
//                   p3.toPose2d()
//                       .transformBy(
//                           new Transform2d(
//                               TaskConstants.kPickOffsetX_m,
//                               TaskConstants.kPickOffsetY_m,
//                               Rotation2d.fromDegrees(TaskConstants.kPickYaw_deg))));
//     } catch (IOException | RuntimeException e) {
//       System.err.println("SimSequence: could not load room layout: " + e);
//       return Optional.empty();
//     }
//   }
// }
