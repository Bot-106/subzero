package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RoomCamera;
import java.util.List;

/** SEAM STUB (§3.1) — W3 replaces this file. */
public class AlignToTagCommand extends Command {
  public AlignToTagCommand(CommandSwerveDrivetrain drivetrain, List<RoomCamera> cameras, int tagId, Pose2d robotToTagOffset) {
    addRequirements(drivetrain);
  }
  /** True if the command ended because the leash refused it (tag unseen > 0.5 s / pose jump > 1 m / timeout). */
  public boolean wasRefused() { return false; }
  @Override public boolean isFinished() { return true; }
}
