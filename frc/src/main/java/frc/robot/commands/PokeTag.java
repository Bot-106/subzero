package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

/**
 * "Poke" the tag the robot is aligned to: the arm extends sideways (to the robot's RIGHT, −Y — the side the right
 * camera faces) into the tag. No drivetrain involvement; {@link AlignRightCameraToTag} must have finished first.
 *
 * <p>ORDER IS A SAFETY REQUIREMENT (the real carousel is small and the arm must never sweep across it low):
 *
 * <ol>
 *   <li>arm to 0 — precondition, asserted by commanding it (a no-op if already retracted);
 *   <li>elevator UP to {@link AutoConstants#kPokeElevatorHeight} (the arm is still retracted);
 *   <li>arm OUT to {@link AutoConstants#kPokeArmExtension};
 *   <li>dwell {@link AutoConstants#kPokeDwellSeconds};
 *   <li>arm BACK to 0 — before the elevator moves;
 *   <li>elevator DOWN to 0.
 * </ol>
 *
 * Every step is an {@link Elevator#goTo} / {@link Arm#goTo} that requires its subsystem and finishes when settled, so
 * the sequence cannot overlap them. Logs {@code Auto/Poke/step} (like {@code Superstructure/step}).
 */
public class PokeTag extends SequentialCommandGroup {
  public PokeTag(Elevator elevator, Arm arm) {
    addCommands(
        step("poke/0 precondition: arm to 0 (elevator still down)", arm.goTo(Meters.of(0))),
        step(
            String.format("poke/1 elevator UP to %.2f m (arm retracted)", AutoConstants.kPokeElevatorHeight.in(Meters)),
            elevator.goTo(AutoConstants.kPokeElevatorHeight)),
        step(
            String.format("poke/2 arm OUT to %.2f m (into the tag)", AutoConstants.kPokeArmExtension.in(Meters)),
            arm.goTo(AutoConstants.kPokeArmExtension)),
        step(
            String.format("poke/3 dwell %.1f s", AutoConstants.kPokeDwellSeconds),
            Commands.waitSeconds(AutoConstants.kPokeDwellSeconds)),
        step("poke/4 arm BACK to 0 (before the elevator moves)", arm.goTo(Meters.of(0))),
        step("poke/5 elevator DOWN to 0", elevator.goTo(Meters.of(0))),
        step("poke done", Commands.none()));
    setName("PokeTag");
  }

  /** Logs the step name (console + AdvantageKit "Auto/Poke/step") before running it. */
  private static Command step(String name, Command inner) {
    return Commands.runOnce(
            () -> {
              System.out.println("PokeTag: " + name);
              Logger.recordOutput("Auto/Poke/step", name);
            })
        .andThen(inner);
  }
}
