package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;

/** SEAM STUB (§3.1) — W1 replaces this file (real one extends TunerConstants.TunerSwerveDrivetrain). */
public class CommandSwerveDrivetrain extends SubsystemBase {
  private List<RoomCamera> cameras = List.of();

  public void setCameras(RoomCamera... cams) { this.cameras = List.of(cams); }
  public List<RoomCamera> getCameras() { return cameras; }
  public Pose2d getPose() { return new Pose2d(); }
  public void resetPose(Pose2d pose) {}
  /** Fuse every camera's update() via addVisionMeasurement (called from periodic()). */
  public void updatePose() {}
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) { return run(() -> {}); }
  /** Returns the AlignToTagCommand (W3) with leash, caps and 8 s timeout. */
  public Command alignToTag(int tagId, Pose2d robotToTagOffset) { return Commands.none(); }
}
