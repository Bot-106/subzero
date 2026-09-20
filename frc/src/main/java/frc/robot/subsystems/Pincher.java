package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PincherConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Two pinch micro-servos driven from RoboRIO DIO 8 / DIO 9 (NOT the PWM header) using the FPGA's DIO PWM
 * generator ({@link DigitalOutput#enablePWM}). 50 Hz, 0.5–2.5 ms pulses = 0–180°.
 *
 * <p>SERVO-ANGLE TEST MODE (2026-09-20): the target angles are read live from NetworkTables every loop —
 * {@code /SmartDashboard/Pincher/servoA_deg} and {@code /SmartDashboard/Pincher/servoB_deg} (0–180, default
 * 90) — so any dashboard (AdvantageScope tuning mode, Elastic, Shuffleboard) can set them and the servos follow.
 * What is actually generated is logged as {@code Pincher/servoA_pulse_us} / {@code servoA_duty} etc.
 *
 * <p>Resolution caveat: the DIO PWM duty cycle is 8-bit at ≤ 1 kHz, i.e. 256 steps per 20 ms period = 78 µs per
 * step → the 2 ms servo span is ~26 steps ≈ 7° per step. Good enough to find open/closed angles; if finer control is
 * needed later, move the servos to the PWM header and use {@code edu.wpi.first.wpilibj.Servo}.
 *
 * <p>Power: the DIO header's 5 V pins come from the RoboRIO's own regulator — fine for two idle micro-servos,
 * but a stalled servo can brown the rail out. A dedicated 5–6 V rail with common ground is the robust option.
 */
public class Pincher extends SubsystemBase {
  private static final double kPwmHz = 50.0;
  private static final double kPeriodUs = 1e6 / kPwmHz;

  private final DigitalOutput jawA = new DigitalOutput(PincherConstants.kJawADioChannel);
  private final DigitalOutput jawB = new DigitalOutput(PincherConstants.kJawBDioChannel);

  private final DoubleEntry servoAEntry;
  private final DoubleEntry servoBEntry;

  private double angleADeg = 90.0;
  private double angleBDeg = 90.0;

  public Pincher() {
    // One PWM rate for every DIO PWM output on the RoboRIO (valid 0.6 Hz … 19 kHz).
    jawA.setPWMRate(kPwmHz);
    jawA.enablePWM(dutyFor(angleADeg));
    jawB.enablePWM(dutyFor(angleBDeg));

    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Pincher");
    servoAEntry = table.getDoubleTopic("servoA_deg").getEntry(angleADeg);
    servoBEntry = table.getDoubleTopic("servoB_deg").getEntry(angleBDeg);
    servoAEntry.set(angleADeg); // publish the defaults so they show up (and are editable) on the dashboard
    servoBEntry.set(angleBDeg);
  }

  /** Servo pulse width for an angle: 0° → kServoMinUs, 180° → kServoMaxUs (linear). */
  private static double pulseUsFor(double deg) {
    final double t = MathUtil.clamp(deg, 0.0, 180.0) / 180.0;
    return PincherConstants.kServoMinPulseUs + t * (PincherConstants.kServoMaxPulseUs - PincherConstants.kServoMinPulseUs);
  }

  private static double dutyFor(double deg) {
    return pulseUsFor(deg) / kPeriodUs;
  }

  /** Directly command servo A (0–180°). */
  public void setAngleA(double deg) {
    angleADeg = MathUtil.clamp(deg, 0.0, 180.0);
    jawA.updateDutyCycle(dutyFor(angleADeg));
  }

  /** Directly command servo B (0–180°). */
  public void setAngleB(double deg) {
    angleBDeg = MathUtil.clamp(deg, 0.0, 180.0);
    jawB.updateDutyCycle(dutyFor(angleBDeg));
  }

  public double getAngleA() {
    return angleADeg;
  }

  public double getAngleB() {
    return angleBDeg;
  }

  /** Both servos to explicit angles (for later choreography). */
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
    Logger.recordOutput("Pincher/servoA_duty", dutyFor(angleADeg));
    Logger.recordOutput("Pincher/servoB_duty", dutyFor(angleBDeg));
  }
}
