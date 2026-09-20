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
 * <p>SERVO-ANGLE TEST MODE (2026-09-20): the target angles are read live from NetworkTables every loop —
 * {@code /SmartDashboard/Pincher/servoA_deg} and {@code /SmartDashboard/Pincher/servoB_deg} (0–180, default
 * 90) — so any dashboard (AdvantageScope tuning mode, Elastic, Shuffleboard) can set them and the servos follow.
 * What is actually generated is logged as {@code Pincher/servoA_pulse_us} etc.
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

  private double angleADeg = 90.0;
  private double angleBDeg = 90.0;

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

  /** Both servos to explicit angles (for later choreography); also updates the dashboard entries. */
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
    Logger.recordOutput("Pincher/servoA_hw_pulse_us", jawA.getPulseTimeMicroseconds());
    Logger.recordOutput("Pincher/servoB_hw_pulse_us", jawB.getPulseTimeMicroseconds());
  }
}
