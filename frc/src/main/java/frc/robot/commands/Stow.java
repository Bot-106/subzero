package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** SEAM STUB (§3.1) — W2 replaces this file: arm retracted, then elevator to stow height. */
public class Stow extends SequentialCommandGroup {
  public Stow(Elevator elevator, Arm arm) {
    addCommands(arm.goTo(frc.robot.Constants.ArmConstants.kRetracted), elevator.goTo(frc.robot.Constants.ElevatorConstants.kStowHeight));
  }
}
