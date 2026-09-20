package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.RoomCamera;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Robot-wide state singleton, HEAVILY based on FRC 1360's {@code RobotState} (Rebuilt2026) minus
 * the turret / hood / flywheel / intake / shoot-on-the-move machinery (Subzero has none of those).
 *
 * <p>Suppliers are set once by the drivetrain (and may be set again by RobotContainer, Rebuilt2026
 * style) via {@link #setAllSuppliers}; until then they return a zero pose / zero speeds so nothing
 * NPEs. {@link #logAllInputs()} and {@link #logAllDistances()} are meant to be called every loop
 * from {@code Robot.robotPeriodic()}. Everything is published both to the NetworkTable
 * {@code RobotState} (StructPublishers, exactly like the original) and mirrored through AdvantageKit
 * {@code Logger.recordOutput("RobotState/…")} so the .wpilog carries it too.
 */
public class RobotState {

  private final NetworkTable loggingTable;
  private final StructPublisher<Pose2d> robotPosePublisher;
  private final StructPublisher<Translation2d> robotVelocityOnFieldPublisher;
  private final StructPublisher<Translation3d> robotVelocityAsVisionTargetPublisher;
  private final IntegerPublisher nearestTagIdPublisher;
  private final DoublePublisher nearestTagDistancePublisher;
  private final List<DoublePublisher> tagToRobotCenterPublishers;

  private final AprilTagFieldLayout fieldLayout;
  private final List<AprilTag> layoutTags;
  private final Pose3d[] layoutTagPoses;
  private final long[] layoutTagIds;

  private Supplier<Pose2d> robotOdomPoseSupplier = () -> Pose2d.kZero;
  private Supplier<ChassisSpeeds> robotChassisSpeedsSupplier = ChassisSpeeds::new;
  private boolean suppliersSet = false;

  private static RobotState instance = null;

  private RobotState() {
    // Same layout the cameras localise against (deploy/room-layout.json, loaded once).
    fieldLayout = RoomCamera.getRoomLayout();
    layoutTags = new ArrayList<>(fieldLayout.getTags());
    layoutTags.sort(Comparator.comparingInt(t -> t.ID));
    layoutTagPoses = new Pose3d[layoutTags.size()];
    layoutTagIds = new long[layoutTags.size()];
    for (int i = 0; i < layoutTags.size(); i++) {
      layoutTagPoses[i] = layoutTags.get(i).pose;
      layoutTagIds[i] = layoutTags.get(i).ID;
    }

    loggingTable = NetworkTableInstance.getDefault().getTable("RobotState");
    robotPosePublisher = loggingTable.getStructTopic("Robot Odometry Pose", Pose2d.struct).publish();
    robotVelocityOnFieldPublisher =
        loggingTable.getStructTopic("Robot Velocity Vector", Translation2d.struct).publish();
    robotVelocityAsVisionTargetPublisher =
        loggingTable
            .getStructTopic("Robot Velocity Vector Offset By Robot", Translation3d.struct)
            .publish();

    nearestTagIdPublisher = loggingTable.getIntegerTopic("Distances/Nearest Tag Id").publish();
    nearestTagDistancePublisher =
        loggingTable.getDoubleTopic("Distances/Nearest Tag To Robot Center").publish();
    tagToRobotCenterPublishers = new ArrayList<>();
    for (AprilTag tag : layoutTags) {
      tagToRobotCenterPublishers.add(
          loggingTable.getDoubleTopic("Distances/Tag " + tag.ID + " To Robot Center").publish());
    }
  }

  public static synchronized RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  public Trigger isBlueAlliance =
      new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

  public void setAllSuppliers(
      Supplier<Pose2d> robotOdomPoseSupplier, Supplier<ChassisSpeeds> robotChassisSpeedsSupplier) {
    this.robotOdomPoseSupplier = robotOdomPoseSupplier;
    this.robotChassisSpeedsSupplier = robotChassisSpeedsSupplier;
    this.suppliersSet = true;
  }

  /** True once {@link #setAllSuppliers} has been called (before that the pose is Pose2d.kZero). */
  public boolean hasSuppliers() {
    return suppliersSet;
  }

  public void logAllInputs() {
    Pose2d robotPose = this.getRobotOdomPose();
    robotPosePublisher.accept(robotPose);

    Translation2d currentRobotVelocity = getFieldRelativeRobotVelocityVector();
    Translation3d velocityAsVisionTarget =
        new Translation3d(currentRobotVelocity.plus(robotPose.getTranslation()))
            .plus(new Translation3d(0.0, 0.0, 0.5));
    robotVelocityOnFieldPublisher.accept(currentRobotVelocity);
    robotVelocityAsVisionTargetPublisher.accept(velocityAsVisionTarget);

    // AdvantageKit mirrors + the layout tags so AdvantageScope 3D renders them (Pose3d[] + ids).
    Logger.recordOutput("RobotState/RobotPose", robotPose);
    Logger.recordOutput("RobotState/RobotPose3d", new Pose3d(robotPose));
    Logger.recordOutput("RobotState/VelocityOnField", currentRobotVelocity);
    Logger.recordOutput("RobotState/VelocityAsVisionTarget", velocityAsVisionTarget);
    Logger.recordOutput("RobotState/SuppliersSet", suppliersSet);
    Logger.recordOutput("RobotState/AprilTags", layoutTagPoses);
    Logger.recordOutput("RobotState/AprilTagIds", layoutTagIds);
    Logger.recordOutput("RobotState/AprilTagLayoutLoadFailed", RoomCamera.roomLayoutLoadFailed());
  }

  public void logAllDistances() {
    Translation2d robotCenter = this.getRobotOdomPose().getTranslation();

    long nearestId = -1;
    double nearestDistance = Double.MAX_VALUE;
    for (int i = 0; i < layoutTags.size(); i++) {
      AprilTag tag = layoutTags.get(i);
      double distance = robotCenter.getDistance(tag.pose.toPose2d().getTranslation());
      tagToRobotCenterPublishers.get(i).accept(distance);
      Logger.recordOutput("RobotState/Distances/Tag" + tag.ID + "ToRobotCenter", distance);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestId = tag.ID;
      }
    }
    if (nearestId < 0) nearestDistance = -1.0;
    nearestTagIdPublisher.accept(nearestId);
    nearestTagDistancePublisher.accept(nearestDistance);
    Logger.recordOutput("RobotState/Distances/NearestTagId", nearestId);
    Logger.recordOutput("RobotState/Distances/NearestTagToRobotCenter", nearestDistance);
  }

  public Pose2d getRobotOdomPose() {
    return robotOdomPoseSupplier.get();
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return robotChassisSpeedsSupplier.get();
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  /** Id of the layout tag nearest to the robot centre, or -1 if the layout is empty. */
  public int getNearestTagId() {
    Translation2d robotCenter = this.getRobotOdomPose().getTranslation();
    int nearestId = -1;
    double nearestDistance = Double.MAX_VALUE;
    for (AprilTag tag : layoutTags) {
      double distance = robotCenter.getDistance(tag.pose.toPose2d().getTranslation());
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestId = tag.ID;
      }
    }
    return nearestId;
  }

  public Translation2d getFieldRelativeRobotVelocityVector() {
    ChassisSpeeds currentSpeeds = robotChassisSpeedsSupplier.get();
    return new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
        .rotateBy(getRobotOdomPose().getRotation());
  }
}
