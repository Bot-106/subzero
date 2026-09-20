package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/**
 * Tool carousel geometry for grabbing / docking end effectors (all MOCK values until measured —
 * TODO(hardware) H-08). Heights are elevator heights above the calibrated zero; extensions are arm
 * extensions from the power-on zero. The carousel is at most 1 m tall.
 *
 * <p>Grab (pick up an end effector): arm at {@link #kPreGrabArmPosition} + elevator at the slot height →
 * arm to {@link #kAttachmentPinchPosition} (butts against the tool) → [pinch] → elevator UP by
 * {@link #kPostPinchElevatorRaiseHeight} (lifts the tool off its hook) → arm back to
 * {@link #kPreGrabArmPosition} → elevator to {@link #kCarouselClearHeight}.
 *
 * <p>Dock is the reverse: elevator to slot height + raise offset (arm pre-grab) → arm EXTENDS to the pinch
 * position → elevator DROPS to the slot height (tool onto its hook) → [un-pinch] → arm PULLS BACK.
 */
public final class CarouselConstants {
  private CarouselConstants() {}

  /** Which carousel level a tool sits at; the value is the elevator height at which the arm is aligned with it. */
  public enum CarouselSlot {
    // TODO(hardware) H-08 — measured slot heights (mock: 0.30 m / 0.60 m; carousel ≤ 1 m).
    LEVEL_1(Meters.of(0.30)),
    LEVEL_2(Meters.of(0.60));

    public final Distance elevatorHeight;

    CarouselSlot(Distance elevatorHeight) {
      this.elevatorHeight = elevatorHeight;
    }
  }

  // TODO(hardware) H-08 — arm extension that keeps the pincher clear of the carousel while the elevator moves (near zero).
  public static final Distance kPreGrabArmPosition = Meters.of(0.02);

  // TODO(hardware) H-08 — arm extension at which the pincher butts up against a tool in its slot.
  public static final Distance kAttachmentPinchPosition = Meters.of(0.25);

  // TODO(hardware) H-08 — how far the elevator lifts after pinching to unhook the tool (and the docking approach offset).
  public static final Distance kPostPinchElevatorRaiseHeight = Meters.of(0.06);

  // TODO(hardware) H-08 — elevator height that clears the top of the carousel with a tool held and the arm retracted.
  // Must stay ≤ the elevator's target cap (7 rot = 0.84 m until H-01 is measured).
  public static final Distance kCarouselClearHeight = Meters.of(0.80);

}
