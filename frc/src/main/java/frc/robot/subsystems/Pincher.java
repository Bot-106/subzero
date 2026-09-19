package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PincherConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Two micro-servos on the arm carriage that pinch / release a swappable end effector — driven
 * directly by the RoboRIO (PWM header), no ESP32 in between.
 *
 * <p>Model: a jaw GAP in mm, 0 = closed on the tool, {@link PincherConstants#kJawMaxMm} = fully
 * open, mapped linearly onto each servo's open/closed angle (jaw B is mirrored). Commanded angles
 * are slew-limited in {@link #periodic()} so the jaws never slam a tool. Hobby servos give no
 * position feedback, so {@link #isClosed()} / {@link #isOpen()} report the *commanded* gap once the
 * slew has finished.
 *
 * <p>Fail-safe: a PWM servo simply holds its last pulse — on DS disable the RoboRIO stops the PWM
 * output and the servo goes limp (this is the RoboRIO's behaviour, not ours). Do not rely on the
 * pincher to hold a tool while disabled.
 */
public class Pincher extends SubsystemBase {
  private final Servo jawA = new Servo(PincherConstants.kJawAPwmChannel);
  private final Servo jawB = new Servo(PincherConstants.kJawBPwmChannel);

  /** Commanded gap target and the slewed gap actually sent to the servos, both in mm. */
  private double targetGapMm = PincherConstants.kJawMaxMm;
  private double currentGapMm = PincherConstants.kJawMaxMm;
  private static final double kLoopSeconds = 0.02;

  public Pincher() {
    applyGap(currentGapMm);
  }

  // ───────────────────────── state ─────────────────────────

  public double getGapMm() {
    return currentGapMm;
  }

  public boolean atTarget() {
    return Math.abs(currentGapMm - targetGapMm) < 1e-3;
  }

  /** Jaws closed on the tool (commanded, after the slew). */
  public boolean isClosed() {
    return atTarget() && currentGapMm <= PincherConstants.kClosedThresholdMm;
  }

  public boolean isOpen() {
    return atTarget() && currentGapMm >= PincherConstants.kJawMaxMm - PincherConstants.kClosedThresholdMm;
  }

  // ───────────────────────── commands ─────────────────────────

  /** Close the jaws onto the end effector ("latch"). Finishes when the slew reaches the target. */
  public Command pinch() {
    return setGap(0.0).withName("PincherPinch");
  }

  /** Open the jaws fully ("release"). */
  public Command release() {
    return setGap(PincherConstants.kJawMaxMm).withName("PincherRelease");
  }

  /** Move to a jaw gap in mm (clamped to [0, kJawMaxMm]); finishes when the slew reaches it. */
  public Command setGap(double gapMm) {
    final double target = MathUtil.clamp(gapMm, 0.0, PincherConstants.kJawMaxMm);
    return runOnce(() -> targetGapMm = target).andThen(run(() -> {}).until(this::atTarget)).withName("PincherGap(" + target + ")");
  }

  // ───────────────────────── periodic ─────────────────────────

  @Override
  public void periodic() {
    // Slew in gap space so both jaws move together; convert the rate from deg/s to mm/s via jaw A's span.
    final double spanDeg = Math.abs(PincherConstants.kJawAClosedDeg - PincherConstants.kJawAOpenDeg);
    final double mmPerDeg = PincherConstants.kJawMaxMm / Math.max(spanDeg, 1e-6);
    final double maxStepMm = PincherConstants.kSlewDegPerSec * mmPerDeg * kLoopSeconds;
    final double delta = MathUtil.clamp(targetGapMm - currentGapMm, -maxStepMm, maxStepMm);
    if (delta != 0.0) {
      currentGapMm += delta;
      applyGap(currentGapMm);
    }

    Logger.recordOutput("Pincher/gap_mm", currentGapMm);
    Logger.recordOutput("Pincher/target_mm", targetGapMm);
    Logger.recordOutput("Pincher/jawA_deg", jawA.getAngle());
    Logger.recordOutput("Pincher/jawB_deg", jawB.getAngle());
    Logger.recordOutput("Pincher/closed", isClosed());
    Logger.recordOutput("Pincher/moving", !atTarget());
  }

  /** gap 0 → closed angles, gap max → open angles; jaw B mirrored by its own pair of constants. */
  private void applyGap(double gapMm) {
    final double t = gapMm / PincherConstants.kJawMaxMm; // 0 = closed, 1 = open
    jawA.setAngle(lerp(PincherConstants.kJawAClosedDeg, PincherConstants.kJawAOpenDeg, t));
    jawB.setAngle(lerp(PincherConstants.kJawBClosedDeg, PincherConstants.kJawBOpenDeg, t));
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
