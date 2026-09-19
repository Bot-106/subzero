package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Linear extending arm — one Kraken X60 (CAN {@link ArmConstants#kCanId}) driving a belt carriage,
 * like the X axis of a 3D printer. NOT a pivot: horizontal travel, so the feedforward has no gravity
 * term (kG = 0); kS + kV (+kA) only.
 *
 * <p>No limit switches. Zero = wherever the carriage sits at power-on (the constructor calls
 * {@code setPosition(0)}); {@link #zeroHere()} re-zeroes at the current position. Travel is protected
 * only by the software soft limits [{@link ArmConstants#kSoftLimitIn}, {@link ArmConstants#kSoftLimitOut}]
 * measured from that zero, and by the stator current limit — so power on with the carriage fully
 * RETRACTED (or re-zero there with Back) before extending.
 *
 * <p>Written in the same shape as the Tuner X Elevator generator output so the two subsystems read alike.
 * Units: the TalonFX reports MECHANISM rotations (after SensorToMechanismRatio); extension_m = rot ×
 * {@link ArmConstants#kMetersPerRotation}.
 */
public class Arm extends SubsystemBase {
  /** Extension setpoints (metres from the power-on zero, H-07). */
  public enum Setpoint {
    Retracted(ArmConstants.kRetracted),
    Rack(ArmConstants.kRackExtension),
    Pick(ArmConstants.kPickExtension),
    Place(ArmConstants.kPlaceExtension);

    /** Target in mechanism rotations. */
    public final Angle target;
    /** Target in linear units. */
    public final Distance targetDist;

    private Setpoint(Distance target) {
      this.targetDist = target;
      this.target = Rotations.of(rotOf(target.in(Meters)));
    }
  }

  private static final int kNumConfigAttempts = 2;
  private static final double kMetersPerRot = ArmConstants.kMetersPerRotation.in(Meters);

  static double rotOf(double meters) { return meters / kMetersPerRot; }
  static double metersOf(double rot) { return rot * kMetersPerRot; }

  /* single motor, no follower */
  private final CANBus kCANBus = new CANBus(Constants.kCanBusName);
  private final TalonFX motor = new TalonFX(ArmConstants.kCanId, kCANBus);

  /* device status signals */
  private final StatusSignal<Angle> motorPosition = motor.getPosition(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
  private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage(false);

  /* controls */
  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut manualRequest = new DutyCycleOut(0);

  private double setpointMeters = 0.0;
  private boolean atSetpointLatched = false;
  private Debouncer atSetpointDebouncer =
      new Debouncer(ArmConstants.kToleranceHoldSeconds, DebounceType.kRising);

  /* simulation: a linear carriage = ElevatorSim with gravity OFF */
  private final ElevatorSim carriageSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ArmConstants.kSensorToMechanismRatio,
          ArmConstants.kSimCarriageMassKg,
          kMetersPerRot / (2 * Math.PI), // "drum radius" that gives kMetersPerRot per mechanism rotation
          0.0,
          ArmConstants.kMaxExtension.in(Meters),
          false, // no gravity — horizontal axis
          0.0);
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  /** Motor configuration. */
  private final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withMotorOutput(
              new TalonFXConfiguration()
                  .MotorOutput
                  .withNeutralMode(
                      ArmConstants.kBrakeNeutral ? NeutralModeValue.Brake : NeutralModeValue.Coast))
          .withCurrentLimits(
              new TalonFXConfiguration()
                  .CurrentLimits
                  .withStatorCurrentLimit(ArmConstants.kStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true))
          .withSlot0(
              new TalonFXConfiguration()
                  .Slot0
                  .withKP(ArmConstants.kP)
                  .withKI(ArmConstants.kI)
                  .withKD(ArmConstants.kD)
                  .withKS(ArmConstants.kS)
                  .withKV(ArmConstants.kV)
                  .withKA(ArmConstants.kA)
                  .withKG(ArmConstants.kG) // 0 for a horizontal linear axis
                  .withGravityType(GravityTypeValue.Elevator_Static)) // constant offset, not cosine (not a pivot)
          .withFeedback(
              new TalonFXConfiguration()
                  .Feedback
                  .withSensorToMechanismRatio(ArmConstants.kSensorToMechanismRatio))
          .withSoftwareLimitSwitch(
              new TalonFXConfiguration()
                  .SoftwareLimitSwitch
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(Rotations.of(rotOf(ArmConstants.kSoftLimitOut.in(Meters))))
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(Rotations.of(rotOf(ArmConstants.kSoftLimitIn.in(Meters)))))
          .withMotionMagic(
              new TalonFXConfiguration()
                  .MotionMagic
                  .withMotionMagicCruiseVelocity(
                      RotationsPerSecond.of(rotOf(ArmConstants.kCruiseVelocity.in(MetersPerSecond))))
                  .withMotionMagicAcceleration(
                      RotationsPerSecondPerSecond.of(
                          rotOf(ArmConstants.kAcceleration.in(MetersPerSecondPerSecond)))));

  public Arm() {
    for (int i = 0; i < kNumConfigAttempts; ++i) {
      var status = motor.getConfigurator().apply(motorConfigs);
      if (status.isOK()) break;
    }

    /* no limit switch: the power-on position is zero */
    motor.setPosition(Rotations.of(0));

    /* default: hold wherever we are (Brake + MotionMagic at the current position) */
    setDefaultCommand(holdPosition());

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // ───────────────────────── state ─────────────────────────

  /** Extension from the power-on zero. */
  public Distance getExtension() {
    return Meters.of(metersOf(motorPosition.getValueAsDouble()));
  }

  /** Within {@link ArmConstants#kTolerance} of the last setpoint for kToleranceHoldSeconds. */
  public boolean atSetpoint() {
    return atSetpointLatched;
  }

  /** Always true — zero is defined at power-on (no homing routine, no switch). Kept for the M1 seam. */
  public boolean isHomed() {
    return true;
  }

  // ───────────────────────── commands ─────────────────────────

  /** Holds the arm at its current position with MotionMagic (default command). */
  public Command holdPosition() {
    return runOnce(
            () -> {
              setpointMeters = metersOf(motorPosition.getValueAsDouble());
              setpointRequest.withPosition(motorPosition.getValue());
            })
        .andThen(run(() -> motor.setControl(setpointRequest)))
        .withName("ArmHold");
  }

  /** Drives to a named setpoint and keeps holding it until interrupted. */
  public Command goToSetpoint(Supplier<Setpoint> setpoint) {
    return run(
            () -> {
              setpointMeters = setpoint.get().targetDist.in(Meters);
              motor.setControl(setpointRequest.withPosition(setpoint.get().target));
            })
        .withName("ArmGoToSetpoint");
  }

  /**
   * M1 seam: MotionMagic to {@code extension} (clamped to the soft limits); finishes once within
   * tolerance for kToleranceHoldSeconds, or after kGoToTimeoutSeconds.
   */
  public Command goTo(Distance extension) {
    final double targetMeters =
        MathUtil.clamp(
            extension.in(Meters),
            ArmConstants.kSoftLimitIn.in(Meters),
            ArmConstants.kSoftLimitOut.in(Meters));
    return runOnce(
            () -> {
              setpointMeters = targetMeters;
              atSetpointLatched = false; // reset the settle latch so a goTo from rest cannot finish instantly
              atSetpointDebouncer =
                  new Debouncer(ArmConstants.kToleranceHoldSeconds, DebounceType.kRising);
            })
        .andThen(run(() -> motor.setControl(setpointRequest.withPosition(Rotations.of(rotOf(targetMeters))))))
        .until(this::atSetpoint)
        .withTimeout(ArmConstants.kGoToTimeoutSeconds)
        .withName("ArmGoTo(" + targetMeters + ")");
  }

  /** Manual jog with a duty cycle (soft limits still apply). Use for the first hardware test and to find kS. */
  public Command manualDrive(DoubleSupplier manualOutput) {
    return run(() -> motor.setControl(manualRequest.withOutput(manualOutput.getAsDouble())))
        .withName("ArmManual");
  }

  /** Re-zero at the current position (no switch: this IS the homing routine). */
  public Command zeroHere() {
    return Commands.runOnce(() -> motor.setPosition(Rotations.of(0))).ignoringDisable(true).withName("ArmZeroHere");
  }

  /** M1 seam name for {@link #zeroHere()}. */
  public Command home() {
    return zeroHere();
  }

  /** M1 seam name for {@link #holdPosition()}. */
  public Command hold() {
    return holdPosition();
  }

  // ───────────────────────── periodic / sim ─────────────────────────

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorStatorCurrent, motorVoltage);

    final double extension = metersOf(motorPosition.getValueAsDouble());
    atSetpointLatched =
        atSetpointDebouncer.calculate(
            Math.abs(extension - setpointMeters) < ArmConstants.kTolerance.in(Meters));

    Logger.recordOutput("Arm/extension_m", extension);
    Logger.recordOutput("Arm/setpoint_m", setpointMeters);
    Logger.recordOutput("Arm/position_rot", motorPosition.getValueAsDouble());
    Logger.recordOutput("Arm/velocity_mps", metersOf(motorVelocity.getValueAsDouble()));
    Logger.recordOutput("Arm/statorCurrent_a", motorStatorCurrent.getValueAsDouble());
    Logger.recordOutput("Arm/appliedVolts", motorVoltage.getValueAsDouble());
    Logger.recordOutput("Arm/atSetpoint", atSetpointLatched);
  }

  private void startSimThread() {
    motor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    lastSimTime = Utils.getCurrentTimeSeconds();

    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              final var sim = motor.getSimState();
              sim.setSupplyVoltage(RobotController.getBatteryVoltage());
              carriageSim.setInputVoltage(sim.getMotorVoltage());
              carriageSim.update(deltaTime);

              /* rotor = mechanism × ratio; mechanism rotations = metres / kMetersPerRot */
              final double mechRot = rotOf(carriageSim.getPositionMeters());
              final double mechRps = rotOf(carriageSim.getVelocityMetersPerSecond());
              sim.setRawRotorPosition(Radians.of(mechRot * 2 * Math.PI * ArmConstants.kSensorToMechanismRatio));
              sim.setRotorVelocity(RadiansPerSecond.of(mechRps * 2 * Math.PI * ArmConstants.kSensorToMechanismRatio));
            });
    simNotifier.startPeriodic(ArmConstants.kSimLoopPeriodSeconds);
  }
}
