// COMMENTED OUT 2026-09-19 (M0 hardware config: drivetrain + elevator only). Restore with: sed -i '' '1d;s|^// ||' subsystems/RoomCamera.java  — full version at git tag m1-sim.
// package frc.robot.subsystems;
// 
// import static edu.wpi.first.units.Units.Meters;
// 
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.VisionConstants;
// import java.io.IOException;
// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import java.util.Optional;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// 
// /**
//  * One PhotonVision camera localising against the room layout (deploy/room-layout.json, H-16).
//  *
//  * <p>Lifted from OrbitCamera.2026.java (PhotonLib v2026.3.4 API): {@code estimateCoprocMultiTagPose}
//  * with {@code estimateLowestAmbiguityPose} fallback, the single/multi-tag std-dev heuristic, the
//  * ambiguity gate ({@link VisionConstants#kMaxAmbiguity}) and the single-tag distance gate
//  * ({@link VisionConstants#kMaxSingleTagDistance}). ONE pipeline (36h11 @ 165.1 mm location tags,
//  * H-15); no {@code setPipelineIndex} (gate Q4 — object tags unused).
//  *
//  * <p>Seam (§3.1, frozen): the drivetrain calls {@link #update()} once per loop and fuses the returned
//  * estimate with {@link #getEstimationStdDevs()}; {@code AlignToTagCommand} uses
//  * {@link #secondsSinceSeen(int)} for the safety §7 leash. In simulation every RoomCamera shares one
//  * static {@link VisionSystemSim} fed by {@link #updateSimPose(Pose2d)}.
//  */
// public class RoomCamera {
//   // OrbitCamera L45–46: single-tag (2,2,2) / multi-tag (0.5,0.5,1); rejected = +inf everywhere.
//   private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 2);
//   private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
//   private static final Matrix<N3, N1> kRejectedStdDevs =
//       VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
// 
//   // Sim camera model (sim only — not hardware; → Constants.VisionConstants.kSim*). A 120° diagonal on
//   // the ±45°-yawed H-14 placeholder mounts is needed so a camera still sees the tag from W4's
//   // SimSequence start pose (tag ≈ 45° off the FL optical axis). Zero calib/pixel error and zero
//   // latency jitter keep the sim estimate exact so vision cannot degrade the fused pose (D: sim is proof).
//   private static final int kSimCameraWidthPx = Constants.VisionConstants.kSimCameraWidthPx;
//   private static final int kSimCameraHeightPx = Constants.VisionConstants.kSimCameraHeightPx;
//   private static final double kSimCameraFovDiagDeg = Constants.VisionConstants.kSimCameraFovDiagDeg;
//   private static final double kSimCameraFps = Constants.VisionConstants.kSimCameraFps;
//   private static final double kSimCameraAvgLatencyMs = Constants.VisionConstants.kSimCameraAvgLatencyMs;
//   private static final double kSimCameraLatencyStdDevMs = Constants.VisionConstants.kSimCameraLatencyStdDevMs;
//   private static final double kSimCameraAvgErrorPx = Constants.VisionConstants.kSimCameraAvgErrorPx;
//   private static final double kSimCameraErrorStdDevPx = Constants.VisionConstants.kSimCameraErrorStdDevPx;
//   /** Two cameras call updateSimPose in the same 20 ms loop; treat calls closer than this as one. */
//   private static final double kSimUpdateDedupSeconds = 0.005;
// 
//   // ── Shared (static) room layout + sim world ─────────────────────────────────────────────────
//   private static AprilTagFieldLayout sRoomLayout;
//   private static VisionSystemSim sVisionSim;
//   private static double sLastSimUpdateSeconds = -1.0;
//   private static Pose2d sLastSimTruth = null;
// 
//   /**
//    * The room layout from the deploy dir ({@link VisionConstants#kRoomLayoutFile}); loaded once. Every
//    * AprilTagFieldLayout ctor sets origin = identity, so poses come back exactly as authored. On an
//    * IOException the layout is EMPTY (no estimates, alignToTag refuses) and the error is reported —
//    * fail safe, never invent tag poses.
//    */
//   public static synchronized AprilTagFieldLayout getRoomLayout() {
//     if (sRoomLayout == null) {
//       try {
//         sRoomLayout =
//             new AprilTagFieldLayout(
//                 Filesystem.getDeployDirectory().toPath().resolve(VisionConstants.kRoomLayoutFile));
//       } catch (IOException | RuntimeException e) {
//         DriverStation.reportError(
//             "RoomCamera: failed to load " + VisionConstants.kRoomLayoutFile + " — vision disabled: " + e,
//             false);
//         sRoomLayout =
//             new AprilTagFieldLayout(
//                 List.of(), VisionConstants.kRoomLengthMeters, VisionConstants.kRoomWidthMeters);
//       }
//     }
//     return sRoomLayout;
//   }
// 
//   private static synchronized VisionSystemSim getVisionSim() {
//     if (sVisionSim == null) {
//       sVisionSim = new VisionSystemSim("subzero");
//       sVisionSim.addAprilTags(getRoomLayout());
//     }
//     return sVisionSim;
//   }
// 
//   // ── Instance ───────────────────────────────────────────────────────────────────────────────
//   private final String name;
//   private final Transform3d robotToCamera;
//   private final Supplier<Pose2d> odomPose;
//   private final PhotonCamera camera;
//   private final PhotonPoseEstimator estimator;
// 
//   private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;
//   private List<PhotonPipelineResult> lastResults = List.of();
//   private Optional<EstimatedRobotPose> lastEstimate = Optional.empty();
//   /** Latest observation of each fiducial id and the FPGA time it was captured. */
//   private final Map<Integer, PhotonTrackedTarget> lastTargetById = new HashMap<>();
//   private final Map<Integer, Double> lastSeenSecondsById = new HashMap<>();
// 
//   private PhotonCameraSim cameraSim; // sim only
// 
//   /**
//    * @param name camera nickname exactly as in the PhotonVision UI (H-14)
//    * @param robotToCamera robot-center → camera transform (H-14; placeholders are the 2025 mounts)
//    * @param odomPose the drivetrain's fused pose (replaces RobotState.getRobotOdomPose() — SWAP 1)
//    */
//   public RoomCamera(String name, Transform3d robotToCamera, Supplier<Pose2d> odomPose) {
//     this.name = name;
//     this.robotToCamera = robotToCamera;
//     this.odomPose = odomPose;
//     this.camera = new PhotonCamera(name);
//     this.estimator = new PhotonPoseEstimator(getRoomLayout(), robotToCamera); // 2-arg 2026 ctor
// 
//     if (RobotBase.isSimulation()) {
//       var props = new SimCameraProperties();
//       props.setCalibration(
//           kSimCameraWidthPx, kSimCameraHeightPx, Rotation2d.fromDegrees(kSimCameraFovDiagDeg));
//       props.setCalibError(kSimCameraAvgErrorPx, kSimCameraErrorStdDevPx);
//       props.setFPS(kSimCameraFps);
//       props.setAvgLatencyMs(kSimCameraAvgLatencyMs);
//       props.setLatencyStdDevMs(kSimCameraLatencyStdDevMs);
//       // 3-arg ctor: the 2-arg one solves sim MultiTag PnP against AprilTagFields.kDefaultField (the
//       // official FRC field), which returns poses outside the room for tag ids 1..4.
//       cameraSim = new PhotonCameraSim(camera, props, getRoomLayout());
//       // No video rendering in headless sim runs (streams default on and cost CPU); results unaffected.
//       cameraSim.enableRawStream(false);
//       cameraSim.enableProcessedStream(false);
//       cameraSim.enableDrawWireframe(false);
//       getVisionSim().addCamera(cameraSim, robotToCamera);
//     }
//   }
// 
//   public String getName() {
//     return name;
//   }
// 
//   public Transform3d getRobotToCamera() {
//     return robotToCamera;
//   }
// 
//   /**
//    * Call ONCE per loop. Drains {@code getAllUnreadResults()} (FIFO, 20 deep), updates the per-tag
//    * last-seen map, and returns the latest estimate that passed the gates (ambiguity ≤ kMaxAmbiguity,
//    * single tag ≤ kMaxSingleTagDistance). {@link #getEstimationStdDevs()} belongs to that estimate.
//    */
//   public Optional<EstimatedRobotPose> update() {
//     List<PhotonPipelineResult> results;
//     try {
//       results = camera.getAllUnreadResults(); // defensive: no coprocessor on the bench must not throw
//     } catch (RuntimeException e) {
//       results = List.of();
//     }
//     lastResults = results;
//     double now = Timer.getFPGATimestamp();
// 
//     Optional<EstimatedRobotPose> accepted = Optional.empty();
//     Matrix<N3, N1> acceptedStdDevs = kSingleTagStdDevs;
//     int numTagsUsed = 0;
//     double ambiguity = -1.0;
//     double latencySeconds = -1.0;
//     int rejected = 0;
// 
//     for (var result : results) {
//       double seen = result.getTimestampSeconds(); // capture time (receive − pipeline latency)
//       if (!(seen > 0.0) || seen > now) seen = now;
//       if (result.hasTargets()) {
//         for (var target : result.getTargets()) {
//           int id = target.getFiducialId();
//           if (id < 0) continue;
//           lastTargetById.put(id, target);
//           lastSeenSecondsById.put(id, seen);
//         }
//       }
// 
//       Optional<EstimatedRobotPose> est;
//       try {
//         est = estimator.estimateCoprocMultiTagPose(result); // primary (coprocessor MultiTag PnP)
//         if (est.isEmpty()) est = estimator.estimateLowestAmbiguityPose(result); // fallback
//       } catch (RuntimeException e) {
//         est = Optional.empty();
//       }
//       if (est.isEmpty()) continue;
// 
//       Matrix<N3, N1> stdDevs = computeStdDevs(est.get(), result.getTargets());
//       if (stdDevs == kRejectedStdDevs) {
//         rejected++;
//         continue;
//       }
//       accepted = est;
//       acceptedStdDevs = stdDevs;
//       numTagsUsed = est.get().targetsUsed.size();
//       ambiguity = maxAmbiguity(est.get().targetsUsed);
//       latencySeconds = now - seen;
//     }
// 
//     lastEstimate = accepted;
//     curStdDevs = acceptedStdDevs;
// 
//     // AK outputs (D-21 lite: outputs only).
//     String prefix = "Vision/" + name + "/";
//     Logger.recordOutput(prefix + "hasEstimate", accepted.isPresent());
//     Logger.recordOutput(
//         prefix + "estimatedPose", accepted.map(e -> e.estimatedPose.toPose2d()).orElse(Pose2d.kZero));
//     Logger.recordOutput(prefix + "numTags", numTagsUsed);
//     Logger.recordOutput(prefix + "ambiguity", ambiguity);
//     Logger.recordOutput(prefix + "latency", latencySeconds);
//     Logger.recordOutput(prefix + "numResults", results.size());
//     Logger.recordOutput(prefix + "rejected", rejected);
//     Logger.recordOutput(prefix + "stdDevXY", acceptedStdDevs.get(0, 0));
//     Logger.recordOutput(prefix + "cameraPose", new Pose3d(odomPose.get()).transformBy(robotToCamera));
//     return accepted;
//   }
// 
//   /** OrbitCamera L103–149 with the Constants gates: rejected = kRejectedStdDevs (identity-compared). */
//   private Matrix<N3, N1> computeStdDevs(EstimatedRobotPose est, List<PhotonTrackedTarget> targets) {
//     var layout = estimator.getFieldTags();
//     var estXY = est.estimatedPose.toPose2d().getTranslation();
//     int numTags = 0;
//     double avgDist = 0.0;
//     boolean ambiguous = false;
//     for (var tgt : targets) {
//       if (tgt.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) ambiguous = true;
//       var tagPose = layout.getTagPose(tgt.getFiducialId());
//       if (tagPose.isEmpty()) continue;
//       numTags++;
//       avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estXY);
//     }
//     if (numTags == 0) return kRejectedStdDevs; // estimate with no layout tags behind it — never fuse
//     avgDist /= numTags;
//     boolean tooFar = numTags == 1 && avgDist > VisionConstants.kMaxSingleTagDistance.in(Meters);
//     if (ambiguous || tooFar) return kRejectedStdDevs;
//     var stdDevs = numTags > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
//     return stdDevs.times(1 + (avgDist * avgDist / 30));
//   }
// 
//   private static double maxAmbiguity(List<PhotonTrackedTarget> targets) {
//     double max = -1.0;
//     for (var t : targets) max = Math.max(max, t.getPoseAmbiguity());
//     return max;
//   }
// 
//   /** Std devs for the estimate returned by the last {@link #update()}. */
//   public Matrix<N3, N1> getEstimationStdDevs() {
//     return curStdDevs;
//   }
// 
//   /** Results drained by the last {@link #update()}. */
//   public List<PhotonPipelineResult> getPipelineResults() {
//     return lastResults;
//   }
// 
//   /** Latest observation of {@code tagId} by this camera (any age — check {@link #secondsSinceSeen}). */
//   public Optional<PhotonTrackedTarget> getTarget(int tagId) {
//     return Optional.ofNullable(lastTargetById.get(tagId));
//   }
// 
//   /** Seconds since {@code tagId} was last captured by this camera; Double.MAX_VALUE if never. */
//   public double secondsSinceSeen(int tagId) {
//     Double seen = lastSeenSecondsById.get(tagId);
//     if (seen == null) return Double.MAX_VALUE;
//     return Math.max(0.0, Timer.getFPGATimestamp() - seen);
//   }
// 
//   /**
//    * Closest location tag (ids kLocationTagIdMin..Max, H-15) seen within the last
//    * {@link DriveConstants#kAlignTagStaleSeconds}, by {@code bestCameraToTarget} norm — for the A-button
//    * alignToTag(nearest) binding (H-21).
//    */
//   public Optional<Integer> getBestVisibleTagId() {
//     Integer best = null;
//     double bestDist = Double.MAX_VALUE;
//     for (var entry : lastTargetById.entrySet()) {
//       int id = entry.getKey();
//       if (id < VisionConstants.kLocationTagIdMin || id > VisionConstants.kLocationTagIdMax) continue;
//       if (secondsSinceSeen(id) > DriveConstants.kAlignTagStaleSeconds) continue;
//       double dist = entry.getValue().getBestCameraToTarget().getTranslation().getNorm();
//       if (dist < bestDist) {
//         bestDist = dist;
//         best = id;
//       }
//     }
//     return Optional.ofNullable(best);
//   }
// 
//   /** The estimate returned by the last {@link #update()} (convenience for logging/tests). */
//   public Optional<EstimatedRobotPose> getLastEstimate() {
//     return lastEstimate;
//   }
// 
//   /**
//    * Sim only: feed the ground-truth robot pose to the shared VisionSystemSim. Safe to call once per
//    * camera per loop — the shared world is stepped at most once per loop (static FPGA-time guard).
//    */
//   public void updateSimPose(Pose2d truth) {
//     if (cameraSim == null) return;
//     synchronized (RoomCamera.class) {
//       double now = Timer.getFPGATimestamp();
//       if (now - sLastSimUpdateSeconds < kSimUpdateDedupSeconds) return;
//       sLastSimUpdateSeconds = now;
//       var sim = getVisionSim();
//       // A teleport (resetPose in a sim sequence) must clear the sim's pose history, otherwise frames
//       // whose capture time falls between the two samples get an interpolated mid-teleport pose.
//       if (sLastSimTruth != null
//           && truth.getTranslation().getDistance(sLastSimTruth.getTranslation())
//               > VisionConstants.kVisionOutlierGate.in(Meters)) {
//         sim.resetRobotPose(truth);
//       }
//       sLastSimTruth = truth;
//       sim.update(truth);
//     }
//   }
// }
