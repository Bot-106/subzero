package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** SEAM STUB (§3.1) — W2 replaces this file. Signatures are frozen. */
public class Elevator extends SubsystemBase {
  /** Refuses (returns a no-op that logs) while !isHomed(). */
  public Command goTo(Distance height) { return Commands.none(); }
  /** Current-based calibrateZero at ≤ 10 % duty with timeout (A-01). */
  public Command home() { return Commands.none(); }
  /** Hold the current position (safe hold for abort). */
  public Command hold() { return Commands.none(); }
  public boolean isHomed() { return false; }
  public Distance getHeight() { return Meters.of(0); }
  public boolean atSetpoint() { return false; }
}
