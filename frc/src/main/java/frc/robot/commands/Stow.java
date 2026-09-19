// COMMENTED OUT 2026-09-19 (M0 hardware config: drivetrain + elevator only). Restore with: sed -i '' '1d;s|^// ||' commands/Stow.java  — full version at git tag m1-sim.
// package frc.robot.commands;
// 
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
// 
// /**
//  * Stow pose (safety §6): arm fully retracted FIRST, then elevator down to {@code kStowHeight}. Every
//  * composite starts and ends here. Ordering matters — the arm must be inside the robot footprint before
//  * the carriage moves (1360 Session 10 "armFirst" branch). The {@code goTo} refusals mean an unhomed axis
//  * never moves: this group then degrades to two printed refusals and finishes.
//  *
//  * <p>Each step is bounded by its own {@code atSetpoint()} finish and internal timeout; callers may still
//  * wrap the whole group in {@code .withTimeout(...)} / {@code .until(driverInput)} per safety §3.
//  */
// public class Stow extends SequentialCommandGroup {
//   public Stow(Elevator elevator, Arm arm) {
//     addCommands(
//         arm.goTo(ArmConstants.kRetracted), // H-07 placeholder
//         elevator.goTo(ElevatorConstants.kStowHeight)); // H-03 placeholder
//     setName("Stow");
//   }
// }
