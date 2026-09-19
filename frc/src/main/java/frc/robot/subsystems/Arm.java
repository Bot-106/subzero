package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
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
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/**
 * Subzero arm — linear, belt-driven, non-pivoting, rides on the elevator carriage and extends
 * robot-forward. Same control stack as {@link Elevator} (Phoenix6 digest §5.7): one Kraken X60,
 * MotionMagicVoltage, {@code SensorToMechanismRatio = kSensorToMechanismRatio}, soft limits at both
 * ends, Brake, 60 A (A-11). Home = retracted limit switch on a RoboRIO DIO (a Kraken has no limit
 * pin), applied to every request through {@code withLimitReverseMotion(pressed)}.
 *
 * <p>Units: {@code extension_m = mechanismRotations × kMetersPerRotation} (H-06). The TalonFX applies the
 * ratio once; only the sim rotor push multiplies by it again.
 */
public class Arm extends SubsystemBase {
  // ── Tuning constants not (yet) in Constants.ArmConstants — orchestrator: move verbatim ──
  static final double kP = Constants.ArmConstants.kP;
  static final double kI = Constants.ArmConstants.kI;
  static final double kD = Constants.ArmConstants.kD;
  static final double kS = Constants.ArmConstants.kS;
  static final double kG = Constants.ArmConstants.kG;
  static final double kV = Constants.ArmConstants.kV;
  static final double kA = Constants.ArmConstants.kA;
  static final LinearVelocity kCruiseVelocity = Constants.ArmConstants.kCruiseVelocity;
  static final LinearAcceleration kAcceleration = Constants.ArmConstants.kAcceleration;
  static final double kGoToTimeoutSeconds = Constants.ArmConstants.kGoToTimeoutSeconds;
  /** Contract C.3 armTo: within tolerance held this long (Elevator has the same in Constants). */
  static final double kToleranceHoldSeconds = Constants.ArmConstants.kToleranceHoldSeconds;
  /** DIO switch debounce before it counts as "pressed" for homing. */
  static final double kSwitchDebounceSeconds = Constants.ArmConstants.kSwitchDebounceSeconds;
  /** Sim-only placeholders. */
  static final double kSimCarriageMassKg = Constants.ArmConstants.kSimCarriageMassKg;
  static final Distance kSimStartExtension = Constants.ArmConstants.kSimStartExtension;
  static final Distance kSimSwitchPressedBelow = Constants.ArmConstants.kSimSwitchPressedBelow;
  static final double kSimLoopPeriodSeconds = Constants.ArmConstants.kSimLoopPeriodSeconds;

  private static final int kNumConfigAttempts = 2;

  // ── unit helpers ──
  private static final double kMetersPerMechanismRotation = ArmConstants.kMetersPerRotation.in(Meters);

  static double rotOf(double meters) {
    return meters / kMetersPerMechanismRotation;
  }

  static double metersOf(double mechanismRotations) {
    return mechanismRotations * kMetersPerMechanismRotation;
  }

  // ── hardware ──
  private final TalonFX motor = new TalonFX(ArmConstants.kCanId, new CANBus(Constants.kCanBusName)); // H-05
  private final DigitalInput homeSwitch = new DigitalInput(ArmConstants.kHomeSwitchDioChannel); // H-06

  private final StatusSignal<Angle> position = motor.getPosition(false);
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity(false);
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage(false);

  // ── control requests ──
  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage holdRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final DutyCycleOut homingRequest =
      new DutyCycleOut(ArmConstants.kHomingDutyCycle) // ≤ 10 %, negative = retract (verify sign, H-06)
          .withIgnoreHardwareLimits(true)
          .withIgnoreSoftwareLimits(true);

  // ── state ──
  private volatile boolean homed = false;
  private boolean homing = false;
  private double setpointMeters = 0.0;
  private boolean atSetpointLatched = false;
  private Debouncer atSetpointDebouncer =
      new Debouncer(kToleranceHoldSeconds, DebounceType.kRising);
  private final Debouncer switchDebouncer = new Debouncer(kSwitchDebounceSeconds, DebounceType.kRising);
  private boolean switchPressedRaw = false;
  private boolean switchPressed = false;

  private double extensionMeters = 0.0;
  private double velocityRps = 0.0;
  private double statorCurrentAmps = 0.0;
  private double motorVolts = 0.0;

  // ── simulation ──
  private ElevatorSim armSim = null;
  private DIOSim homeSwitchSim = null;
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;
  private volatile double simExtensionMeters = kSimStartExtension.in(Meters);
  private final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(ArmConstants.kMaxExtension.in(Meters) + 0.2, 0.4);
  private final LoggedMechanismLigament2d armLigament =
      mech2d
          .getRoot("ArmRoot", 0.1, 0.2)
          .append(new LoggedMechanismLigament2d("Arm", kSimStartExtension.in(Meters), 0.0));

  public Arm() {
    final TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.withNeutralMode(
        ArmConstants.kBrakeNeutral ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits.withStatorCurrentLimit(ArmConstants.kStatorCurrentLimit) // A-11
        .withStatorCurrentLimitEnable(true);
    cfg.Slot0.withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG)
        .withGravityType(GravityTypeValue.Elevator_Static);
    cfg.Feedback.withSensorToMechanismRatio(ArmConstants.kSensorToMechanismRatio); // H-06
    cfg.SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(rotOf(ArmConstants.kSoftLimitIn.in(Meters)))
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(rotOf(ArmConstants.kSoftLimitOut.in(Meters)));
    // HardwareLimitSwitch left disabled: Kraken X60 has no limit pin; the DIO is applied per request.
    cfg.MotionMagic.withMotionMagicCruiseVelocity(rotOf(kCruiseVelocity.in(MetersPerSecond)))
        .withMotionMagicAcceleration(rotOf(kAcceleration.in(MetersPerSecondPerSecond)))
        .withMotionMagicJerk(0);

    for (int i = 0; i < kNumConfigAttempts; ++i) {
      if (motor.getConfigurator().apply(cfg).isOK()) break;
    }

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, statorCurrent, motorVoltage);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    setDefaultCommand(hold());
  }

  // ─────────────────────────── frozen seams (§3.1) ───────────────────────────

  public boolean isHomed() {
    return homed;
  }

  public Distance getExtension() {
    return Meters.of(extensionMeters);
  }

  public boolean atSetpoint() {
    return atSetpointLatched;
  }

  /** True when the retracted switch reads pressed (debounced), honouring {@code kHomeSwitchNormallyOpen}. */
  public boolean isSwitchPressed() {
    return switchPressed;
  }

  /**
   * MotionMagic to {@code extension}, clamped to [kSoftLimitIn, kSoftLimitOut]; finishes when
   * {@link #atSetpoint()} or after {@code kGoToTimeoutSeconds}. Refuses (prints, no motion) while not homed,
   * decided when scheduled.
   */
  public Command goTo(Distance extension) {
    final double targetMeters =
        MathUtil.clamp(
            extension.in(Meters),
            ArmConstants.kSoftLimitIn.in(Meters),
            ArmConstants.kSoftLimitOut.in(Meters));
    final Command move =
        runOnce(
                () -> {
                  setpointMeters = targetMeters;
                  // Reset the settle latch so a goTo issued from rest cannot finish on the stale
                  // "at previous setpoint" value computed by periodic() earlier in this loop.
                  atSetpointLatched = false;
                  atSetpointDebouncer =
                      new Debouncer(ArmConstants.kToleranceHoldSeconds, DebounceType.kRising);
                })
            .andThen(
                run(
                    () ->
                        motor.setControl(
                            setpointRequest
                                .withPosition(rotOf(targetMeters))
                                .withLimitReverseMotion(switchPressed))))
            .until(this::atSetpoint)
            .withTimeout(kGoToTimeoutSeconds);
    return Commands.either(
            move,
            Commands.print("Arm: refused goTo(" + targetMeters + " m) — not homed"),
            this::isHomed)
        .withName("ArmGoTo(" + targetMeters + ")");
  }

  /**
   * Retract at {@code kHomingDutyCycle} (≤ 10 %) until the DIO switch is pressed, then zero and set homed.
   * Timeout {@code kHomingTimeoutSeconds}; a timeout leaves the motor neutral and {@code homed} false.
   */
  public Command home() {
    return new HomeCommand().withTimeout(ArmConstants.kHomingTimeoutSeconds).withName("ArmHome");
  }

  /**
   * Default command. Homed: MotionMagic hold at the last commanded setpoint if within tolerance, else at
   * the extension read when the command starts (clamped). NOT homed: {@code NeutralOut} (Brake) — an
   * unhomed axis never moves under closed loop.
   */
  public Command hold() {
    return runOnce(
            () -> {
              if (homed) {
                double e = extensionMeters;
                if (Math.abs(e - setpointMeters) >= ArmConstants.kTolerance.in(Meters)) {
                  setpointMeters =
                      MathUtil.clamp(
                          e, ArmConstants.kSoftLimitIn.in(Meters), ArmConstants.kSoftLimitOut.in(Meters));
                }
                holdRequest.withPosition(rotOf(setpointMeters));
              } else {
                setpointMeters = extensionMeters; // logging only
              }
            })
        .andThen(
            run(
                () -> {
                  if (homed) {
                    motor.setControl(holdRequest.withLimitReverseMotion(switchPressed));
                  } else {
                    motor.setControl(neutralRequest);
                  }
                }))
        .withName("ArmHold");
  }

  // ─────────────────────────── homing command ───────────────────────────

  private final class HomeCommand extends Command {
    private final Timer timer = new Timer();

    HomeCommand() {
      addRequirements(Arm.this);
    }

    @Override
    public void initialize() {
      homed = false;
      homing = true;
      timer.restart();
    }

    @Override
    public void execute() {
      // Defense in depth: the switch also clamps reverse output in firmware once pressed.
      motor.setControl(homingRequest.withLimitReverseMotion(switchPressed));
    }

    @Override
    public boolean isFinished() {
      return switchPressed;
    }

    @Override
    public void end(boolean interrupted) {
      motor.setControl(neutralRequest);
      homing = false;
      if (!interrupted) {
        motor.setPosition(Rotations.of(0));
        setpointMeters = 0.0;
        homed = true;
        System.out.println("Arm: homed on switch (t=" + timer.get() + " s)");
      } else {
        System.out.println("Arm: home() interrupted/timed out after " + timer.get() + " s — NOT homed");
      }
    }
  }

  // ─────────────────────────── periodic ───────────────────────────

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity, statorCurrent, motorVoltage);
    extensionMeters = metersOf(position.getValueAsDouble());
    velocityRps = velocity.getValueAsDouble();
    statorCurrentAmps = statorCurrent.getValueAsDouble();
    motorVolts = motorVoltage.getValueAsDouble();

    // NO to ground with the RoboRIO pull-up: open → HIGH (true), pressed → LOW (false).
    final boolean raw = homeSwitch.get();
    switchPressedRaw = ArmConstants.kHomeSwitchNormallyOpen ? !raw : raw;
    switchPressed = switchDebouncer.calculate(switchPressedRaw);

    final boolean withinTol =
        homed
            && !homing
            && Math.abs(extensionMeters - setpointMeters) < ArmConstants.kTolerance.in(Meters);
    atSetpointLatched = atSetpointDebouncer.calculate(withinTol);

    armLigament.setLength(Math.max(0.0, extensionMeters));

    Logger.recordOutput("Arm/extension_m", extensionMeters);
    Logger.recordOutput("Arm/setpoint_m", setpointMeters);
    Logger.recordOutput("Arm/homed", homed);
    Logger.recordOutput("Arm/homing", homing);
    Logger.recordOutput("Arm/switchPressed", switchPressed);
    Logger.recordOutput("Arm/switchRaw", raw);
    Logger.recordOutput("Arm/current_a", statorCurrentAmps);
    Logger.recordOutput("Arm/atSetpoint", atSetpointLatched);
    Logger.recordOutput("Arm/velocity_mps", metersOf(velocityRps));
    Logger.recordOutput("Arm/appliedVolts", motorVolts);
    if (Utils.isSimulation()) {
      Logger.recordOutput("Arm/sim/extension_m", simExtensionMeters);
    }
    Logger.recordOutput("Arm/Mechanism2d", mech2d);
  }

  // ─────────────────────────── simulation ───────────────────────────

  private void startSimThread() {
    // Linear stage as an ElevatorSim without gravity; "drum radius" = pulley pitch circumference / 2π.
    armSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.kSensorToMechanismRatio,
            kSimCarriageMassKg,
            kMetersPerMechanismRotation / (2.0 * Math.PI),
            0.0,
            ArmConstants.kMaxExtension.in(Meters),
            false,
            kSimStartExtension.in(Meters));
    homeSwitchSim = new DIOSim(homeSwitch);
    homeSwitchSim.setIsInput(true);

    final TalonFXSimState sim = motor.getSimState();
    sim.Orientation = ChassisReference.CounterClockwise_Positive;
    sim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    pushSimState(sim, kSimStartExtension.in(Meters), 0.0);

    lastSimTime = Utils.getCurrentTimeSeconds();
    simNotifier =
        new Notifier(
            () -> {
              final double now = Utils.getCurrentTimeSeconds();
              final double dt = now - lastSimTime;
              lastSimTime = now;

              sim.setSupplyVoltage(RobotController.getBatteryVoltage());
              armSim.setInputVoltage(sim.getMotorVoltage());
              armSim.update(dt);

              simExtensionMeters = armSim.getPositionMeters();
              pushSimState(sim, simExtensionMeters, armSim.getVelocityMetersPerSecond());
            });
    simNotifier.setName("ArmSim");
    simNotifier.startPeriodic(kSimLoopPeriodSeconds);
  }

  /** Rotor = mechanism × ratio (the one place we apply it); DIO raw value follows the NO/NC convention. */
  private void pushSimState(TalonFXSimState sim, double extensionM, double velocityMps) {
    sim.setRawRotorPosition(Rotations.of(rotOf(extensionM) * ArmConstants.kSensorToMechanismRatio));
    sim.setRotorVelocity(RotationsPerSecond.of(rotOf(velocityMps) * ArmConstants.kSensorToMechanismRatio));
    final boolean pressed = extensionM <= kSimSwitchPressedBelow.in(Meters);
    homeSwitchSim.setValue(ArmConstants.kHomeSwitchNormallyOpen ? !pressed : pressed);
  }
}
