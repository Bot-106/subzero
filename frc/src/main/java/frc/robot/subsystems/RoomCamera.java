package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** SEAM STUB (§3.1) — W3 replaces this file. Signatures are frozen. */
public class RoomCamera {
  private final String name;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> odomPose;

  public RoomCamera(String name, Transform3d robotToCamera, Supplier<Pose2d> odomPose) {
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.odomPose = odomPose;
  }

  public String getName() { return name; }
  public Transform3d getRobotToCamera() { return robotToCamera; }

  /** Call once per loop (drains getAllUnreadResults). Latest accepted estimate, if any. */
  public Optional<EstimatedRobotPose> update() { return Optional.empty(); }
  /** Std devs for the estimate returned by the last update(). */
  public Matrix<N3, N1> getEstimationStdDevs() { return VecBuilder.fill(2, 2, 2); }
  /** Results drained by the last update(). */
  public List<PhotonPipelineResult> getPipelineResults() { return List.of(); }
  /** Latest observation of a tag, if seen recently. */
  public Optional<PhotonTrackedTarget> getTarget(int tagId) { return Optional.empty(); }
  /** Seconds since tagId was last seen by this camera (Double.MAX_VALUE if never). */
  public double secondsSinceSeen(int tagId) { return Double.MAX_VALUE; }
  /** Closest visible location tag, for the A-button binding (H-21). */
  public Optional<Integer> getBestVisibleTagId() { return Optional.empty(); }
  /** Sim only: feed the ground-truth pose to VisionSystemSim (safe to call once per camera per loop). */
  public void updateSimPose(Pose2d truth) {}
}
