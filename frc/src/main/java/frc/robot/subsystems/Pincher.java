package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PincherConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Two pinch micro-servos on RoboRIO PWM header channels 8 / 9 (WPILib {@link Servo}: dedicated FPGA servo PWM,
 * ~1 µs pulse resolution ≈ 0.1°). Pulse range {@link PincherConstants#kServoMinPulseUs}–{@link
 * PincherConstants#kServoMaxPulseUs} = 0–180°.
 *
 * <p>The target angles live in NetworkTables — {@code /SmartDashboard/Pincher/servoA_deg} and
 * {@code servoB_deg} (0–180) — and are followed every loop, so a dashboard (AdvantageScope tuning mode, Elastic,
 * Shuffleboard) can always override them for testing; {@link #pinch()} / {@link #release()} write the measured
 * closed / open angles ({@link PincherConstants}) into the same entries. What is actually generated is logged as
 * {@code Pincher/servoA_pulse_us} etc.
 *
 * <p>Power: the PWM header's +5 V pins come from the RoboRIO's own regulator — fine for two idle micro-servos,
 * but a stalled servo can brown the rail out. A dedicated 5–6 V rail with common ground is the robust option
 * (signal stays on PWM 8/9).
 */
public class Pincher extends SubsystemBase {
  private final Servo jawA = new Servo(PincherConstants.kJawAPwmChannel);
  private final Servo jawB = new Servo(PincherConstants.kJawBPwmChannel);

  private final DoubleEntry servoAEntry;
  private final DoubleEntry servoBEntry;

  // Boot in the OPEN (unpinched) pose so the jaws never grab or strike anything on power-up.
  private double angleADeg = PincherConstants.kJawAOpenDeg;
  private double angleBDeg = PincherConstants.kJawBOpenDeg;

  public Pincher() {
    // Map 0–180° onto the configured pulse range (WPILib's default is the same 0.5–2.5 ms; kept explicit + tunable).
    final int min = (int) PincherConstants.kServoMinPulseUs;
    final int max = (int) PincherConstants.kServoMaxPulseUs;
    final int center = (min + max) / 2;
    jawA.setBoundsMicroseconds(max, center, center, center, min);
    jawB.setBoundsMicroseconds(max, center, center, center, min);
    setAngleA(angleADeg);
    setAngleB(angleBDeg);

    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Pincher");
    servoAEntry = table.getDoubleTopic("servoA_deg").getEntry(angleADeg);
    servoBEntry = table.getDoubleTopic("servoB_deg").getEntry(angleBDeg);
    servoAEntry.set(angleADeg); // publish the defaults so they show up (and are editable) on the dashboard
    servoBEntry.set(angleBDeg);
  }

  /** Servo pulse width for an angle: 0° → kServoMinPulseUs, 180° → kServoMaxPulseUs (linear). */
  private static double pulseUsFor(double deg) {
    final double t = MathUtil.clamp(deg, 0.0, 180.0) / 180.0;
    return PincherConstants.kServoMinPulseUs + t * (PincherConstants.kServoMaxPulseUs - PincherConstants.kServoMinPulseUs);
  }

  /** Directly command servo A (0–180°). */
  public void setAngleA(double deg) {
    angleADeg = MathUtil.clamp(deg, 0.0, 180.0);
    jawA.setAngle(angleADeg);
  }

  /** Directly command servo B (0–180°). */
  public void setAngleB(double deg) {
    angleBDeg = MathUtil.clamp(deg, 0.0, 180.0);
    jawB.setAngle(angleBDeg);
  }

  public double getAngleA() {
    return angleADeg;
  }

  public double getAngleB() {
    return angleBDeg;
  }

  /** Close the jaws on the end effector: A → kJawAClosedDeg, B → kJawBClosedDeg. */
  public Command pinch() {
    return setAngles(PincherConstants.kJawAClosedDeg, PincherConstants.kJawBClosedDeg).withName("PincherPinch");
  }

  /** Open the jaws: A → kJawAOpenDeg, B → kJawBOpenDeg. */
  public Command release() {
    return setAngles(PincherConstants.kJawAOpenDeg, PincherConstants.kJawBOpenDeg).withName("PincherRelease");
  }

  /** Commanded pose is the closed pair (no feedback on hobby servos). */
  public boolean isPinched() {
    return Math.abs(angleADeg - PincherConstants.kJawAClosedDeg) < 1.0
        && Math.abs(angleBDeg - PincherConstants.kJawBClosedDeg) < 1.0;
  }

  /** Both servos to explicit angles; also updates the dashboard entries (which periodic() follows). */
  public Command setAngles(double aDeg, double bDeg) {
    return runOnce(
            () -> {
              servoAEntry.set(aDeg);
              servoBEntry.set(bDeg);
            })
        .withName(String.format("PincherAngles(%.0f, %.0f)", aDeg, bDeg));
  }

  @Override
  public void periodic() {
    // TEST MODE: follow the dashboard entries every loop.
    setAngleA(servoAEntry.get(angleADeg));
    setAngleB(servoBEntry.get(angleBDeg));

    Logger.recordOutput("Pincher/servoA_deg", angleADeg);
    Logger.recordOutput("Pincher/servoB_deg", angleBDeg);
    Logger.recordOutput("Pincher/servoA_pulse_us", pulseUsFor(angleADeg));
    Logger.recordOutput("Pincher/servoB_pulse_us", pulseUsFor(angleBDeg));
    Logger.recordOutput("Pincher/pinched", isPinched());
    Logger.recordOutput("Pincher/servoA_hw_pulse_us", jawA.getPulseTimeMicroseconds());
    Logger.recordOutput("Pincher/servoB_hw_pulse_us", jawB.getPulseTimeMicroseconds());
  }
}
