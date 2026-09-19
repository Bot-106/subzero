package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** SEAM STUB (§3.1) — W2 replaces this file. Signatures are frozen. */
public class Arm extends SubsystemBase {
  public Command goTo(Distance extension) { return Commands.none(); }
  /** Retract at low duty until the DIO limit switch, with timeout. */
  public Command home() { return Commands.none(); }
  public Command hold() { return Commands.none(); }
  public boolean isHomed() { return false; }
  public Distance getExtension() { return Meters.of(0); }
  public boolean atSetpoint() { return false; }
}
