package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CarouselConstants;
import frc.robot.CarouselConstants.CarouselSlot;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PincherConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pincher;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinated elevator + arm motion: end-effector positioning and the carousel grab / dock choreography.
 * Every step is a {@link Elevator#goTo} / {@link Arm#goTo} that requires its subsystem, so a new sequence
 * (or a jog-free button press) interrupts the running one and the hold defaults take over at the last
 * commanded targets. Speeds stay under the 0.75 m/s MotionMagic caps configured in the subsystems.
 */
public class Superstructure {
  private final Elevator elevator;
  private final Arm arm;
  private final Pincher pincher;

  public Superstructure(Elevator elevator, Arm arm, Pincher pincher) {
    this.elevator = elevator;
    this.arm = arm;
    this.pincher = pincher;
  }

  /** Move the arm to {@code armExtension} and the elevator to {@code elevatorHeight} together; done when both settle. */
  public Command setEndpointPosition(Distance armExtension, Distance elevatorHeight) {
    return step(
            String.format("setEndpoint(arm %.3f m, elev %.3f m)", armExtension.in(Meters), elevatorHeight.in(Meters)),
            Commands.parallel(arm.goTo(armExtension), elevator.goTo(elevatorHeight)))
        .withName("SetEndpointPosition");
  }

  /** Convenience overload in metres: {@code setEndpointPosition(0.1, 1.0)} = arm 0.1 m, elevator 1.0 m. */
  public Command setEndpointPosition(double armExtensionMeters, double elevatorHeightMeters) {
    return setEndpointPosition(Meters.of(armExtensionMeters), Meters.of(elevatorHeightMeters));
  }

  /** Pick an end effector off the carousel at {@code slot}; ends retracted at the clear height, tool in hand. */
  public Command grabFromCarousel(CarouselSlot slot) {
    final Distance h = slot.elevatorHeight;
    return Commands.sequence(
            step("grab/1 align: arm pre-grab + elevator to slot " + slot,
                setEndpointPosition(CarouselConstants.kPreGrabArmPosition, h)),
            step("grab/2 arm to pinch position", arm.goTo(CarouselConstants.kAttachmentPinchPosition)),
            step("grab/3 PINCH", pinch()),
            step("grab/4 elevator lift off hook", elevator.goTo(h.plus(CarouselConstants.kPostPinchElevatorRaiseHeight))),
            step("grab/5 arm back to pre-grab", arm.goTo(CarouselConstants.kPreGrabArmPosition)),
            step("grab/6 elevator to carousel clear height", elevator.goTo(CarouselConstants.kCarouselClearHeight)),
            step("grab done", Commands.none()))
        .withName("GrabFromCarousel(" + slot + ")");
  }

  /** Put the held end effector back onto the carousel at {@code slot}; ends with the arm retracted at slot height. */
  public Command dockToCarousel(CarouselSlot slot) {
    final Distance h = slot.elevatorHeight;
    final Distance approach = h.plus(CarouselConstants.kPostPinchElevatorRaiseHeight);
    return Commands.sequence(
            step("dock/1 align: arm pre-grab + elevator to slot " + slot + " + offset",
                setEndpointPosition(CarouselConstants.kPreGrabArmPosition, approach)),
            step("dock/2 arm extends to pinch position", arm.goTo(CarouselConstants.kAttachmentPinchPosition)),
            step("dock/3 elevator drops onto hook", elevator.goTo(h)),
            step("dock/4 UN-PINCH", release()),
            step("dock/5 arm pulls back", arm.goTo(CarouselConstants.kPreGrabArmPosition)),
            step("dock done", Commands.none()))
        .withName("DockToCarousel(" + slot + ")");
  }

  // ───────────── helpers ─────────────

  /** Close the jaws (measured angles), then wait for the servos to get there. */
  private Command pinch() {
    return pincher.pinch().andThen(Commands.waitSeconds(PincherConstants.kServoTravelSeconds));
  }

  /** Open the jaws, then wait for the servos to get there. */
  private Command release() {
    return pincher.release().andThen(Commands.waitSeconds(PincherConstants.kServoTravelSeconds));
  }

  /** Logs the step name (console + AdvantageKit "Superstructure/step") before running it. */
  private static Command step(String name, Command inner) {
    return Commands.runOnce(
            () -> {
              System.out.println("Superstructure: " + name);
              Logger.recordOutput("Superstructure/step", name);
            })
        .andThen(inner);
  }
}
