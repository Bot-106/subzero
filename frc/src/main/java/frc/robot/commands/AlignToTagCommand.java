// COMMENTED OUT 2026-09-19 (M0 hardware config: drivetrain + elevator only). Restore with: sed -i '' '1d;s|^// ||' commands/AlignToTagCommand.java  — full version at git tag m1-sim.
// package frc.robot.commands;
// 
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// 
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.RoomCamera;
// import java.util.List;
// import org.littletonrobotics.junction.Logger;
// 
// /**
//  * Drive the fused robot pose to a location tag + offset (port of AlignToTagPoseCommand.java to the
//  * room layout, field-relative PID, Phoenix6 26 {@code ApplyFieldSpeeds}).
//  *
//  * <p>Offset semantics (FROZEN, §3.1): target robot pose =
//  * {@code tagPose2d.transformBy(new Transform2d(offset.getTranslation(), offset.getRotation()))}
//  * where the tag's 2D +X axis is the tag normal pointing into the room; so
//  * {@code new Pose2d(0.45, 0.0, Rotation2d.k180deg)} = 0.45 m in front of the tag, facing it.
//  *
//  * <p>Feedback = {@code drivetrain.getPose()} (already vision-fused by updatePose()). The cameras are
//  * the safety §7 LEASH: refuse at initialize() unless some camera saw the tag within
//  * {@link DriveConstants#kAlignTagStaleSeconds}; end refused if the fused pose jumps more than
//  * {@link DriveConstants#kAlignMaxPoseJump} between loops; internal deadline
//  * {@link DriveConstants#kAlignTimeoutSeconds}. Safety §2 caps: |v| ≤ kAutoMaxSpeed, |ω| ≤
//  * kAutoMaxAngularRate, isotropic slew at kAutoMaxAccelMps2 on the field-relative velocity vector.
//  * Done when error ≤ kAlignTolerance / kAlignToleranceDeg for kAlignSettleSeconds.
//  */
// public class AlignToTagCommand extends Command {
//   // Gains (not hardware; → Constants.DriveConstants). Chosen so W4's SimSequence (1.5 m / 20° start)
//   // converges < 3 cm / 2° in ≈ 3 s under the 1.0 m/s, 90 °/s, 1.5 m/s² caps: P-only saturates at
//   // 0.22 m / 18° of error, then settles with ~3 mm slew-limited overshoot.
//   private static final double kTranslationP = Constants.DriveConstants.kAlignTranslationP;
//   private static final double kTranslationI = Constants.DriveConstants.kAlignTranslationI;
//   private static final double kTranslationD = Constants.DriveConstants.kAlignTranslationD;
//   private static final double kRotationP = Constants.DriveConstants.kAlignRotationP;
//   private static final double kRotationI = Constants.DriveConstants.kAlignRotationI;
//   private static final double kRotationD = Constants.DriveConstants.kAlignRotationD;
//   /** Loop dt is clamped to this when the scheduler stalls so the slew step can never explode. */
//   private static final double kMaxDtSeconds = 0.1;
// 
//   private final CommandSwerveDrivetrain drivetrain;
//   private final List<RoomCamera> cameras;
//   private final int tagId;
//   private final Pose2d robotToTagOffset;
// 
//   private final PIDController xController =
//       new PIDController(kTranslationP, kTranslationI, kTranslationD);
//   private final PIDController yController =
//       new PIDController(kTranslationP, kTranslationI, kTranslationD);
//   private final PIDController thetaController =
//       new PIDController(kRotationP, kRotationI, kRotationD);
//   private final Debouncer settled =
//       new Debouncer(DriveConstants.kAlignSettleSeconds, Debouncer.DebounceType.kRising);
//   // Closed-loop module velocity: open-loop voltage stalls ~3 cm short at the low speeds near the goal.
//   private final SwerveRequest.ApplyFieldSpeeds driveRequest =
//       new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
//   private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
//   private final Timer timer = new Timer();
// 
//   private Pose2d targetPose = Pose2d.kZero;
//   private Pose2d lastPose = Pose2d.kZero;
//   private Translation2d lastVelocity = Translation2d.kZero; // field-relative, for the slew
//   private double lastTime = 0.0;
//   private boolean refused = false;
//   private String refuseReason = "";
//   private boolean done = false;
// 
//   public AlignToTagCommand(
//       CommandSwerveDrivetrain drivetrain, List<RoomCamera> cameras, int tagId, Pose2d robotToTagOffset) {
//     this.drivetrain = drivetrain;
//     this.cameras = cameras == null ? List.of() : cameras;
//     this.tagId = tagId;
//     this.robotToTagOffset = robotToTagOffset;
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     setName("AlignToTag(" + tagId + ")");
//     addRequirements(drivetrain);
//   }
// 
//   /** True if the command ended because the leash refused it (tag unseen > 0.5 s / pose jump > 1 m / timeout). */
//   public boolean wasRefused() {
//     return refused;
//   }
// 
//   /** The field pose the robot is driven to (valid after initialize()). */
//   public Pose2d getTargetPose() {
//     return targetPose;
//   }
// 
//   @Override
//   public void initialize() {
//     refused = false;
//     refuseReason = "";
//     done = false;
//     xController.reset();
//     yController.reset();
//     thetaController.reset();
//     settled.calculate(false);
//     lastVelocity = Translation2d.kZero;
//     timer.restart();
//     lastTime = Timer.getFPGATimestamp();
//     lastPose = drivetrain.getPose();
// 
//     var tagPose = RoomCamera.getRoomLayout().getTagPose(tagId);
//     if (tagPose.isEmpty()) {
//       refuse("tag " + tagId + " is not in the room layout");
//       targetPose = Pose2d.kZero;
//     } else {
//       targetPose =
//           tagPose
//               .get()
//               .toPose2d()
//               .transformBy(
//                   new Transform2d(robotToTagOffset.getTranslation(), robotToTagOffset.getRotation()));
//     }
// 
//     // Safety §7 leash: refuse unless some camera has seen the tag in the last kAlignTagStaleSeconds.
//     double sinceSeen = secondsSinceSeen();
//     if (!refused && cameras.isEmpty()) {
//       refuse("no cameras");
//     } else if (!refused && sinceSeen > DriveConstants.kAlignTagStaleSeconds) {
//       refuse(
//           String.format(
//               "tag %d not seen for %.2f s (> %.2f s)",
//               tagId, Math.min(sinceSeen, 9999.0), DriveConstants.kAlignTagStaleSeconds));
//     }
//     log(lastPose, 0.0, 0.0, 0.0, sinceSeen);
//     if (refused) drivetrain.setControl(idleRequest);
//   }
// 
//   @Override
//   public void execute() {
//     if (refused) {
//       drivetrain.setControl(idleRequest);
//       return;
//     }
//     double now = Timer.getFPGATimestamp();
//     double dt = MathUtil.clamp(now - lastTime, 0.0, kMaxDtSeconds);
//     lastTime = now;
// 
//     Pose2d pose = drivetrain.getPose();
// 
//     // Leash: fused pose jump > kAlignMaxPoseJump between consecutive loops, or the 8 s deadline.
//     double jump = pose.getTranslation().getDistance(lastPose.getTranslation());
//     lastPose = pose;
//     if (jump > DriveConstants.kAlignMaxPoseJump.in(Meters)) {
//       refuse(String.format("pose jumped %.2f m", jump));
//     } else if (timer.get() > DriveConstants.kAlignTimeoutSeconds) {
//       refuse(String.format("timeout after %.1f s", timer.get()));
//     }
//     if (refused) {
//       drivetrain.setControl(idleRequest);
//       log(pose, 0.0, 0.0, 0.0, secondsSinceSeen());
//       return;
//     }
// 
//     // Field-relative PID on the fused pose.
//     double vx = xController.calculate(pose.getX(), targetPose.getX());
//     double vy = yController.calculate(pose.getY(), targetPose.getY());
//     double omega =
//         thetaController.calculate(
//             pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
// 
//     // Safety §2 caps: speed, angular rate, then isotropic slew on the velocity vector.
//     double maxSpeed = DriveConstants.kAutoMaxSpeed.in(MetersPerSecond);
//     Translation2d velocity = new Translation2d(vx, vy);
//     if (velocity.getNorm() > maxSpeed) velocity = velocity.times(maxSpeed / velocity.getNorm());
//     Translation2d delta = velocity.minus(lastVelocity);
//     double maxDelta = DriveConstants.kAutoMaxAccelMps2 * dt;
//     if (delta.getNorm() > maxDelta) delta = delta.times(maxDelta / delta.getNorm());
//     velocity = lastVelocity.plus(delta);
//     lastVelocity = velocity;
//     double maxOmega = DriveConstants.kAutoMaxAngularRate.in(RadiansPerSecond);
//     omega = MathUtil.clamp(omega, -maxOmega, maxOmega);
// 
//     // Tolerance + settle.
//     double errorM = pose.getTranslation().getDistance(targetPose.getTranslation());
//     double errorDeg = Math.abs(targetPose.getRotation().minus(pose.getRotation()).getDegrees());
//     boolean inTolerance =
//         errorM <= DriveConstants.kAlignTolerance.in(Meters)
//             && errorDeg <= DriveConstants.kAlignToleranceDeg;
//     done = settled.calculate(inTolerance);
// 
//     drivetrain.setControl(
//         driveRequest.withSpeeds(new ChassisSpeeds(velocity.getX(), velocity.getY(), omega)));
//     log(pose, velocity.getX(), velocity.getY(), omega, secondsSinceSeen());
//   }
// 
//   @Override
//   public boolean isFinished() {
//     return refused || done;
//   }
// 
//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.setControl(idleRequest);
//     lastVelocity = Translation2d.kZero;
//     Logger.recordOutput("Align/done", done && !interrupted);
//     Logger.recordOutput("Align/interrupted", interrupted);
//     Logger.recordOutput("Align/refused", refused);
//     Logger.recordOutput("Align/refuseReason", refuseReason);
//     Logger.recordOutput("Align/elapsed_s", timer.get());
//     Logger.recordOutput("Align/active", false);
//   }
// 
//   private void refuse(String reason) {
//     if (!refused) {
//       refused = true;
//       refuseReason = reason;
//     }
//   }
// 
//   /** Smallest secondsSinceSeen(tagId) over all cameras; Double.MAX_VALUE if none has seen it. */
//   private double secondsSinceSeen() {
//     double best = Double.MAX_VALUE;
//     for (var cam : cameras) best = Math.min(best, cam.secondsSinceSeen(tagId));
//     return best;
//   }
// 
//   private void log(Pose2d pose, double vx, double vy, double omega, double sinceSeen) {
//     Logger.recordOutput("Align/active", true);
//     Logger.recordOutput("Align/tagId", tagId);
//     Logger.recordOutput("Align/targetPose", targetPose);
//     Logger.recordOutput("Align/pose", pose);
//     Logger.recordOutput(
//         "Align/error_m", pose.getTranslation().getDistance(targetPose.getTranslation()));
//     Logger.recordOutput(
//         "Align/error_deg", Math.abs(targetPose.getRotation().minus(pose.getRotation()).getDegrees()));
//     Logger.recordOutput("Align/vx", vx);
//     Logger.recordOutput("Align/vy", vy);
//     Logger.recordOutput("Align/omega", omega);
//     Logger.recordOutput("Align/tagSecondsSinceSeen", Math.min(sinceSeen, 9999.0));
//     Logger.recordOutput("Align/refused", refused);
//     Logger.recordOutput("Align/refuseReason", refuseReason);
//     Logger.recordOutput("Align/elapsed_s", timer.get());
//     Logger.recordOutput("Align/done", done);
//   }
// }
