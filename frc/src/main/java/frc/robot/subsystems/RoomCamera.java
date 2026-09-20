package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.util.RobotState;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * One PhotonVision camera localising against the room AprilTag layout.
 *
 * <p>This is FRC 1360's {@code OrbitCamera} (Rebuilt2026, PhotonLib v2026.3.4) lifted nearly
 * verbatim. Differences from the original:
 *
 * <ul>
 *   <li>the tag layout is the deploy file {@link Constants.VisionConstants#kRoomLayoutFile}
 *       (frc/src/main/deploy/room-layout.json), loaded once into a static — an IOException reports
 *       to the DriverStation and leaves an EMPTY layout (no estimates, never invented tag poses);
 *   <li>the ambiguity cutoff / single-tag distance gate come from {@link Constants.VisionConstants};
 *   <li>every NetworkTables StructPublisher under {@code Photon Cameras/<name>/…} is mirrored with an
 *       AdvantageKit {@code Logger.recordOutput("Vision/<name>/…")} so the .wpilog carries it too;
 *   <li>sim: the {@link PhotonCameraSim} is built with the ROOM layout (the 2-arg ctor solves sim
 *       MultiTag PnP against the official FRC field, which places tag ids 2..9 elsewhere), calib /
 *       pixel error are small (0.05 / 0.02 px) so convergence is clean, the wireframe and the video
 *       streams are off (CPU), and sim objects only exist when {@code Utils.isSimulation()};
 *   <li>PhotonVision NetworkTables reads are wrapped so a missing coprocessor cannot throw.
 * </ul>
 *
 * <p>Drivetrain contract (Rebuilt2026 {@code updatePose()}): once per loop call {@link
 * #updatePipelineResults()}, then for each result of {@link #getPipelineResults()} run {@code
 * getPhotonPoseEstimator().estimateCoprocMultiTagPose(result)} with an {@code
 * estimateLowestAmbiguityPose(result)} fallback, {@link #updateEstimationStdDevs(Optional, List)},
 * fuse with {@link #getEstimationStdDevs()}, and {@link #updateStructPublisher(Pose2d)}.
 */
public class RoomCamera {
  // ── Shared (static) room layout ─────────────────────────────────────────────────────────────
  private static AprilTagFieldLayout sRoomLayout;
  private static boolean sRoomLayoutLoadFailed = false;

  /**
   * The room layout from the deploy dir ({@link Constants.VisionConstants#kRoomLayoutFile}), loaded
   * once. Every AprilTagFieldLayout ctor sets origin = identity, so poses come back exactly as
   * authored. On an IOException the layout is EMPTY and the error is reported — fail safe.
   */
  public static synchronized AprilTagFieldLayout getRoomLayout() {
    if (sRoomLayout == null) {
      try {
        sRoomLayout =
            new AprilTagFieldLayout(
                Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve(Constants.VisionConstants.kRoomLayoutFile));
      } catch (IOException | RuntimeException e) {
        sRoomLayoutLoadFailed = true;
        DriverStation.reportError(
            "RoomCamera: failed to load deploy/"
                + Constants.VisionConstants.kRoomLayoutFile
                + " — vision disabled (empty layout): "
                + e,
            false);
        // Dimensions only matter for origin flipping, which is never used; the layout is empty.
        sRoomLayout = new AprilTagFieldLayout(List.of(), 6.0, 4.0);
      }
    }
    return sRoomLayout;
  }

  /** True if the deploy layout could not be read (the layout in use is empty). */
  public static synchronized boolean roomLayoutLoadFailed() {
    getRoomLayout();
    return sRoomLayoutLoadFailed;
  }

  // ── Instance (OrbitCamera) ──────────────────────────────────────────────────────────────────
  private final AprilTagFieldLayout aprilTagFieldLayout;

  private final String cameraName;
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Transform3d robotToCamera;

  private List<PhotonPipelineResult> currentUnreadResults;

  StructPublisher<Pose2d> photonPose;
  StructPublisher<Transform3d> robotToCameraOffset;
  StructPublisher<Pose3d> robotPoseTransformedByCameraOffset;
  private Matrix<N3, N1> curStdDevs;
  private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 2);
  private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private final double AMBIGUITY_CUTOFF_THRESHOLD = Constants.VisionConstants.kMaxAmbiguity;
  private final double SINGLE_TAG_MAX_DISTANCE_METERS =
      Constants.VisionConstants.kMaxSingleTagDistance.in(Meters);
  /**
   * Rebuilt2026 rejects the whole estimate if ANY target's single-tag pose ambiguity exceeds the
   * cutoff — even a MultiTag PnP estimate, which is unambiguous by construction. In Subzero's room
   * the walls are viewed head-on from ~5 m, where per-tag ambiguity is inherently high (fronto-
   * parallel tags): sim measured only 9–17 % of multi-tag frames passing while the rejected ones
   * were exactly as accurate (~1 cm / 0.1°). So the gate applies to single-tag (LOWEST_AMBIGUITY
   * fallback) estimates only. Set true to restore the verbatim Rebuilt2026 behaviour.
   */
  private static final boolean kApplyAmbiguityGateToMultiTag = false;

  /** AdvantageKit mirror key prefix: {@code Vision/<cameraName>/}. */
  private final String akPrefix;
  private boolean readWarned = false;

  /* Simulation Stuff */
  private PhotonCameraSim photonCameraSim;
  private SimCameraProperties photonCameraSimProperties;
  private VisionSystemSim photonVisionSystemSim;

  /**
   * @param robotToCamera robot-centre → camera transform (TODO(hardware) H-14: translations are
   *     placeholders; both cameras lie parallel to the ground, pitch 0)
   * @param cameraName the camera nickname exactly as in the PhotonVision UI
   */
  public RoomCamera(Transform3d robotToCamera, String cameraName) {
    aprilTagFieldLayout = getRoomLayout();

    this.cameraName = cameraName;
    this.akPrefix = "Vision/" + cameraName + "/";
    this.robotToCamera = robotToCamera;
    this.photonCamera = new PhotonCamera(cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);

    photonPose =
        NetworkTableInstance.getDefault()
            .getStructTopic("Photon Cameras/" + cameraName + "/Photon_Pose", Pose2d.struct)
            .publish();
    robotToCameraOffset =
        NetworkTableInstance.getDefault()
            .getStructTopic(
                "Photon Cameras/" + cameraName + "/Robot To Cam Offset", Transform3d.struct)
            .publish();
    robotPoseTransformedByCameraOffset =
        NetworkTableInstance.getDefault()
            .getStructTopic(
                "Photon Cameras/" + cameraName + "/Robot Pose Transformed By Robot To Cam",
                Pose3d.struct)
            .publish();
    robotToCameraOffset.accept(this.robotToCamera);
    Logger.recordOutput(akPrefix + "RobotToCamOffset", this.robotToCamera);

    this.currentUnreadResults = new ArrayList<PhotonPipelineResult>();
    this.curStdDevs = kSingleTagStdDevs;

    if (Utils.isSimulation()) {
      /* Simulation Settings */
      photonVisionSystemSim = new VisionSystemSim(cameraName + "-main");
      photonVisionSystemSim.addAprilTags(aprilTagFieldLayout);

      photonCameraSimProperties = new SimCameraProperties();
      photonCameraSimProperties.setCalibration(640, 400, Rotation2d.fromDegrees(68));
      // Rebuilt2026 used (0.25, 0.08); small error here so sim convergence is clean (sim is proof).
      photonCameraSimProperties.setCalibError(0.05, 0.02);
      photonCameraSimProperties.setFPS(50);
      photonCameraSimProperties.setAvgLatencyMs(21);
      photonCameraSimProperties.setLatencyStdDevMs(3);

      // 3-arg ctor: sim MultiTag PnP must solve against the ROOM layout, not the official field.
      photonCameraSim = new PhotonCameraSim(photonCamera, photonCameraSimProperties, aprilTagFieldLayout);

      photonVisionSystemSim.addCamera(photonCameraSim, robotToCamera);

      // Raw / processed streams default ON; off here to save CPU in headless runs (flip for a human
      // who wants to watch the sim camera in the sim GUI / PhotonVision-style stream).
      photonCameraSim.enableRawStream(false);
      photonCameraSim.enableProcessedStream(false);

      // Enable drawing a wireframe visualization of the field to the camera streams.
      // This is extremely resource-intensive and is disabled by default.
      photonCameraSim.enableDrawWireframe(false);
    }
  }

  public void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDist = 0;
    double maxAmbiguity = -1.0;
    boolean rejected = false;

    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      boolean isPoseAmbiguous = false;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        maxAmbiguity = Math.max(maxAmbiguity, tgt.getPoseAmbiguity());
        if (tgt.getPoseAmbiguity() > AMBIGUITY_CUTOFF_THRESHOLD) {
          isPoseAmbiguous = true;
        }
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // MultiTag PnP is unambiguous by construction; the per-tag ambiguity gate is for single-tag.
        boolean isMultiTagPnp =
            estimatedPose.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                || estimatedPose.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_RIO;
        boolean ambiguityGateApplies = kApplyAmbiguityGateToMultiTag || !isMultiTagPnp;
        // Increase std devs based on (average) distance
        if ((numTags == 1 && avgDist > SINGLE_TAG_MAX_DISTANCE_METERS)
            || (isPoseAmbiguous && ambiguityGateApplies)) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          rejected = true;
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }

    // AdvantageKit mirrors (D-21 lite: outputs only). Last write in a loop wins.
    Logger.recordOutput(akPrefix + "HasEstimate", estimatedPose.isPresent());
    Logger.recordOutput(akPrefix + "NumTags", numTags);
    Logger.recordOutput(akPrefix + "AvgTagDistance", avgDist);
    Logger.recordOutput(akPrefix + "Ambiguity", maxAmbiguity);
    Logger.recordOutput(akPrefix + "Rejected", rejected);
    Logger.recordOutput(akPrefix + "StdDevXY", curStdDevs.get(0, 0));
    Logger.recordOutput(akPrefix + "StdDevTheta", curStdDevs.get(2, 0));
    if (estimatedPose.isPresent()) {
      Logger.recordOutput(akPrefix + "EstimatedPose3d", estimatedPose.get().estimatedPose);
      Logger.recordOutput(akPrefix + "Strategy", String.valueOf(estimatedPose.get().strategy));
      long[] ids = new long[estimatedPose.get().targetsUsed.size()];
      for (int i = 0; i < ids.length; i++) {
        ids[i] = estimatedPose.get().targetsUsed.get(i).getFiducialId();
      }
      Logger.recordOutput(akPrefix + "TagIdsUsed", ids);
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * True when the last {@link #updateEstimationStdDevs} rejected the estimate (std devs = MAX_VALUE,
   * i.e. the Kalman gain is zero and the measurement is a no-op in the pose estimator).
   */
  public boolean isEstimateRejected() {
    return curStdDevs.get(0, 0) >= Double.MAX_VALUE;
  }

  public PhotonCamera getPhotonCamera() {
    return this.photonCamera;
  }

  public PhotonPoseEstimator getPhotonPoseEstimator() {
    return this.photonPoseEstimator;
  }

  public String getCameraName() {
    return this.cameraName;
  }

  public void updateStructPublisher(Pose2d cameraEstimatedPose) {
    Pose3d robotPoseTransformed =
        new Pose3d(RobotState.getInstance().getRobotOdomPose()).transformBy(this.robotToCamera);
    photonPose.accept(cameraEstimatedPose);
    robotPoseTransformedByCameraOffset.accept(robotPoseTransformed);

    Logger.recordOutput(akPrefix + "PhotonPose", cameraEstimatedPose);
    Logger.recordOutput(akPrefix + "RobotPoseTransformedByRobotToCam", robotPoseTransformed);
  }

  // Functions for updating and retreiving pipeling results
  public void updatePipelineResults() {
    boolean connected = false;
    try {
      connected = this.photonCamera.isConnected();
      currentUnreadResults = this.photonCamera.getAllUnreadResults();
      if (currentUnreadResults == null) currentUnreadResults = new ArrayList<>();
    } catch (RuntimeException e) {
      // No coprocessor / malformed packet must never take the robot loop down.
      currentUnreadResults = new ArrayList<>();
      if (!readWarned) {
        readWarned = true;
        DriverStation.reportWarning(
            "RoomCamera " + cameraName + ": PhotonVision read failed: " + e, false);
      }
    }

    // Per-loop AdvantageKit mirrors (overwritten by updateEstimationStdDevs when a result exists).
    Logger.recordOutput(akPrefix + "Connected", connected);
    Logger.recordOutput(akPrefix + "NumResults", currentUnreadResults.size());
    Logger.recordOutput(akPrefix + "RobotToCamOffset", this.robotToCamera);
    Logger.recordOutput(
        akPrefix + "CameraPose3d",
        new Pose3d(RobotState.getInstance().getRobotOdomPose()).transformBy(this.robotToCamera));
    if (currentUnreadResults.isEmpty()) {
      Logger.recordOutput(akPrefix + "HasEstimate", false);
      Logger.recordOutput(akPrefix + "NumTags", 0);
    } else {
      var last = currentUnreadResults.get(currentUnreadResults.size() - 1);
      Logger.recordOutput(akPrefix + "LatencyMs", last.metadata.getLatencyMillis());
      Logger.recordOutput(akPrefix + "NumTargetsSeen", last.getTargets().size());
    }
  }

  public List<PhotonPipelineResult> getPipelineResults() {
    return currentUnreadResults;
  }

  // Update Vision Pose (sim only — no-op on the robot)
  public void updateSimPose(Pose2d robotPose) {
    if (photonVisionSystemSim == null) return;
    photonVisionSystemSim.update(robotPose);
  }

  /**
   * Sim only: teleport the sim world's robot (clears the pose history so latency-compensated frames
   * are not interpolated across the jump). No-op on the robot.
   */
  public void resetSimPose(Pose2d robotPose) {
    if (photonVisionSystemSim == null) return;
    photonVisionSystemSim.resetRobotPose(robotPose);
  }

  public Transform3d getRobotToCamera() {
    return this.robotToCamera;
  }
}
