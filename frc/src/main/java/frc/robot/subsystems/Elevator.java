package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/**
 * Subzero elevator — lifted from the Tuner X Elevator Generator output
 * (PACK/20-reference-repos/key-files/Elevator.TunerXGenerated.java) and made safe per safety.md §4:
 *
 * <ul>
 *   <li>leader/follower Kraken X60 on {@link Constants#kCanBusName}, follower {@code Opposed} as generated;
 *   <li>{@code Brake} neutral (H-04), stator limit {@link ElevatorConstants#kStatorCurrentLimit} (A-11);
 *   <li>{@code SoftwareLimitSwitch} at both ends in <b>mechanism</b> rotations (H-01);
 *   <li>MotionMagic only — no step commands; {@code hold()} is the default command;
 *   <li>{@code home()} = current-based calibrateZero at ≤ 10 % duty with a timeout (A-01); a timeout
 *       never marks the axis homed;
 *   <li>{@code goTo()} refuses while not homed (checked at <i>schedule</i> time, not construction time).
 * </ul>
 *
 * <p>Units: {@code height_m = mechanismRotations × 2π × kDrumRadius × kRiggingFactor}. The TalonFX
 * applies {@code SensorToMechanismRatio} once, so every position/velocity we read or command is in
 * mechanism rotations; only the sim rotor push multiplies by the ratio again.
 *
 * <p>{@code elevator_upper_bound} from elevator.TunerXProject.json is NEVER used (H-01 governs).
 */
public class Elevator extends SubsystemBase {
  // ── Tuning constants not (yet) in Constants.ElevatorConstants — orchestrator: move verbatim ──
  /** Slot0 gains, MotionMagicVoltage → volts. kP/kD per mechanism rotation, kV per mechanism rps. */
  static final double kP = Constants.ElevatorConstants.kP;
  static final double kI = Constants.ElevatorConstants.kI;
  static final double kD = Constants.ElevatorConstants.kD;
  static final double kS = Constants.ElevatorConstants.kS;
  /** Generated 0 (no gravity hold). 0.35 V ≈ 8 kg carriage on 2× Kraken X60 @ 4:1 — TODO(tuning) on the robot. */
  static final double kG = Constants.ElevatorConstants.kG;
  static final double kV = Constants.ElevatorConstants.kV;
  static final double kA = Constants.ElevatorConstants.kA;
  /** MotionMagic profile in carriage units (converted to mechanism rot/s, rot/s² below). */
  static final LinearVelocity kCruiseVelocity = Constants.ElevatorConstants.kCruiseVelocity;
  static final LinearAcceleration kAcceleration = Constants.ElevatorConstants.kAcceleration;
  /** goTo gives up (and hold() takes over, safely) after this long even if never within tolerance. */
  static final double kGoToTimeoutSeconds = Constants.ElevatorConstants.kGoToTimeoutSeconds;
  /** Homing detection: stall = |v| below this after the minimum run, or stator current over threshold. */
  static final double kHomingStallVelocityRps = Constants.ElevatorConstants.kHomingStallVelocityRps;
  static final double kHomingMinRunSeconds = Constants.ElevatorConstants.kHomingMinRunSeconds;
  static final double kHomingStallHoldSeconds = Constants.ElevatorConstants.kHomingStallHoldSeconds;
  static final double kHomingCurrentDebounceSeconds = Constants.ElevatorConstants.kHomingCurrentDebounceSeconds;
  /** Sim-only placeholders (not H-numbered: they never reach the robot). */
  static final double kSimCarriageMassKg = Constants.ElevatorConstants.kSimCarriageMassKg;
  static final Distance kSimStartHeight = Constants.ElevatorConstants.kSimStartHeight;
  static final Distance kSimFloorEpsilon = Constants.ElevatorConstants.kSimFloorEpsilon;
  static final double kSimLoopPeriodSeconds = Constants.ElevatorConstants.kSimLoopPeriodSeconds;

  private static final int kNumConfigAttempts = 2;

  // ── unit helpers (mechanism rotations ↔ carriage metres) ──
  private static final double kMetersPerMechanismRotation =
      2.0 * Math.PI * ElevatorConstants.kDrumRadius.in(Meters) * ElevatorConstants.kRiggingFactor;

  /** Carriage height → mechanism rotations (what the TalonFX reads/commands after SensorToMechanismRatio). */
  static double rotOf(double meters) {
    return meters / kMetersPerMechanismRotation;
  }

  /** Mechanism rotations → carriage height. */
  static double metersOf(double mechanismRotations) {
    return mechanismRotations * kMetersPerMechanismRotation;
  }

  // ── hardware ──
  private final CANBus canBus = new CANBus(Constants.kCanBusName);
  private final TalonFX leader = new TalonFX(ElevatorConstants.kLeaderCanId, canBus);
  private final TalonFX follower = new TalonFX(ElevatorConstants.kFollowerCanId, canBus);

  private final StatusSignal<Angle> leaderPosition = leader.getPosition(false);
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity(false);
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent(false);
  private final StatusSignal<Voltage> leaderMotorVoltage = leader.getMotorVoltage(false);

  // ── control requests (reused, never re-allocated per loop) ──
  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage holdRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  /** Homing must be allowed to cross the (pre-home, meaningless) soft limits. ≤ 10 % duty by Constants. */
  private final DutyCycleOut homingRequest =
      new DutyCycleOut(ElevatorConstants.kHomingDutyCycle)
          .withIgnoreHardwareLimits(true)
          .withIgnoreSoftwareLimits(true);

  // ── state ──
  private volatile boolean homed = false;
  private boolean homing = false;
  private double setpointMeters = 0.0;
  private boolean atSetpointLatched = false;
  private Debouncer atSetpointDebouncer =
      new Debouncer(ElevatorConstants.kToleranceHoldSeconds, DebounceType.kRising);

  // cached from the last periodic() refresh
  private double heightMeters = 0.0;
  private double velocityRps = 0.0;
  private double statorCurrentAmps = 0.0;
  private double motorVolts = 0.0;

  // ── simulation ──
  private ElevatorSim elevatorSim = null;
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;
  private volatile double simHeightMeters = kSimStartHeight.in(Meters);
  private volatile double simVelocityMps = 0.0;
  private final LoggedMechanism2d mech2d =
      new LoggedMechanism2d(1.0, ElevatorConstants.kMaxHeight.in(Meters) + 0.2);
  private final LoggedMechanismLigament2d carriageLigament =
      mech2d
          .getRoot("ElevatorRoot", 0.5, 0.0)
          .append(new LoggedMechanismLigament2d("Carriage", kSimStartHeight.in(Meters), 90.0));

  public Elevator() {
    final TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.withNeutralMode(
        ElevatorConstants.kBrakeNeutral ? NeutralModeValue.Brake : NeutralModeValue.Coast); // H-04
    cfg.CurrentLimits.withStatorCurrentLimit(ElevatorConstants.kStatorCurrentLimit) // A-11
        .withStatorCurrentLimitEnable(true);
    cfg.Slot0.withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKA(kA)
        .withKG(kG)
        .withGravityType(GravityTypeValue.Elevator_Static);
    cfg.Feedback.withSensorToMechanismRatio(ElevatorConstants.kSensorToMechanismRatio); // H-02
    // Both ends, in mechanism rotations (H-01). Homing ignores these via the request flag.
    cfg.SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(rotOf(ElevatorConstants.kSoftLimitBottom.in(Meters)))
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(rotOf(ElevatorConstants.kSoftLimitTop.in(Meters)));
    // HardwareLimitSwitch on LimitSwitchPin is inert on a Kraken X60 — deliberately left disabled (default).
    cfg.MotionMagic.withMotionMagicCruiseVelocity(rotOf(kCruiseVelocity.in(MetersPerSecond)))
        .withMotionMagicAcceleration(rotOf(kAcceleration.in(MetersPerSecondPerSecond)))
        .withMotionMagicJerk(0);

    // Follower gets the same MotorOutput + CurrentLimits (+ everything else, harmlessly).
    final TalonFXConfiguration followerCfg = cfg.clone();
    followerCfg.SoftwareLimitSwitch.withReverseSoftLimitEnable(false).withForwardSoftLimitEnable(false);

    for (int i = 0; i < kNumConfigAttempts; ++i) {
      if (leader.getConfigurator().apply(cfg).isOK()) break;
    }
    for (int i = 0; i < kNumConfigAttempts; ++i) {
      if (follower.getConfigurator().apply(followerCfg).isOK()) break;
    }

    // Generated opposition flag — TODO(hardware) confirm the second motor really faces the other way.
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leaderPosition, leaderVelocity, leaderStatorCurrent, leaderMotorVoltage);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    setDefaultCommand(hold());
  }

  // ─────────────────────────── frozen seams (§3.1) ───────────────────────────

  /** True once {@link #home()} has completed since power-on (never set by a timed-out home). */
  public boolean isHomed() {
    return homed;
  }

  /** Carriage height above the homed zero. Only meaningful once {@link #isHomed()}. */
  public Distance getHeight() {
    return Meters.of(heightMeters);
  }

  /** Within {@code kTolerance} of the last setpoint for {@code kToleranceHoldSeconds} (debounced, latching). */
  public boolean atSetpoint() {
    return atSetpointLatched;
  }

  /**
   * MotionMagic to {@code height}, clamped to [kSoftLimitBottom, kSoftLimitTop]. Finishes when
   * {@link #atSetpoint()} (or after {@code kGoToTimeoutSeconds}, in which case {@code hold()} resumes
   * and {@code atSetpoint()} stays false so the caller can tell). Refuses — prints, no motion — while
   * {@code !isHomed()}; the check happens when the command is scheduled, so bindings built before homing
   * work once homing has run.
   */
  public Command goTo(Distance height) {
    final double targetMeters =
        MathUtil.clamp(
            height.in(Meters),
            ElevatorConstants.kSoftLimitBottom.in(Meters),
            ElevatorConstants.kSoftLimitTop.in(Meters));
    final Command move =
        runOnce(
                () -> {
                  setpointMeters = targetMeters;
                  // Reset the settle latch so a goTo issued from rest cannot finish on the stale
                  // "at previous setpoint" value computed by periodic() earlier in this loop.
                  atSetpointLatched = false;
                  atSetpointDebouncer =
                      new Debouncer(ElevatorConstants.kToleranceHoldSeconds, DebounceType.kRising);
                })
            .andThen(run(() -> leader.setControl(setpointRequest.withPosition(rotOf(targetMeters)))))
            .until(this::atSetpoint)
            .withTimeout(kGoToTimeoutSeconds);
    return Commands.either(
            move,
            Commands.print("Elevator: refused goTo(" + targetMeters + " m) — not homed"),
            this::isHomed)
        .withName("ElevatorGoTo(" + targetMeters + ")");
  }

  /**
   * Current-based calibrateZero (A-01, safety §4): drive down at {@code kHomingDutyCycle} (≤ 10 %) until a
   * hard stop is detected, then zero both encoders and set homed. Hard stop = stator current above
   * {@code kHomingCurrentThreshold} while stalled, OR (after 0.3 s) |velocity| ≈ 0 held 0.2 s, OR in sim the
   * ElevatorSim floor (no current spike exists there). Timeout {@code kHomingTimeoutSeconds}: on timeout the
   * motor goes neutral and {@code homed} stays false.
   */
  public Command home() {
    return new HomeCommand().withTimeout(ElevatorConstants.kHomingTimeoutSeconds).withName("ElevatorHome");
  }

  /**
   * Default command and the safe hold for abort. Homed: MotionMagic hold at the last commanded setpoint if
   * we are within tolerance of it, else at the height read when the command starts (clamped to the soft
   * limits). NOT homed: {@code NeutralOut} — Brake holds the carriage (H-04) and an unhomed axis never
   * moves under closed loop (its zero is arbitrary and a stale first read would otherwise become a move).
   */
  public Command hold() {
    return runOnce(
            () -> {
              if (homed) {
                double h = heightMeters;
                if (Math.abs(h - setpointMeters) >= ElevatorConstants.kTolerance.in(Meters)) {
                  setpointMeters =
                      MathUtil.clamp(
                          h,
                          ElevatorConstants.kSoftLimitBottom.in(Meters),
                          ElevatorConstants.kSoftLimitTop.in(Meters));
                }
                holdRequest.withPosition(rotOf(setpointMeters));
              } else {
                setpointMeters = heightMeters; // logging only
              }
            })
        .andThen(
            run(
                () -> {
                  if (homed) {
                    leader.setControl(holdRequest);
                  } else {
                    leader.setControl(neutralRequest);
                  }
                }))
        .withName("ElevatorHold");
  }

  // ─────────────────────────── homing command ───────────────────────────

  private final class HomeCommand extends Command {
    private final Timer timer = new Timer();
    private Debouncer currentStall;
    private Debouncer velocityStall;
    private Debouncer simFloor;

    HomeCommand() {
      addRequirements(Elevator.this);
    }

    @Override
    public void initialize() {
      homed = false;
      homing = true;
      timer.restart();
      currentStall = new Debouncer(kHomingCurrentDebounceSeconds, DebounceType.kRising);
      velocityStall = new Debouncer(kHomingStallHoldSeconds, DebounceType.kRising);
      simFloor = new Debouncer(kHomingStallHoldSeconds, DebounceType.kRising);
    }

    @Override
    public void execute() {
      leader.setControl(homingRequest);
    }

    @Override
    public boolean isFinished() {
      final boolean slow = Math.abs(velocityRps) < kHomingStallVelocityRps;
      final boolean pastMinRun = timer.hasElapsed(kHomingMinRunSeconds);
      final boolean byCurrent =
          currentStall.calculate(
              slow && statorCurrentAmps > ElevatorConstants.kHomingCurrentThreshold.in(Amps));
      final boolean byStall = velocityStall.calculate(pastMinRun && slow);
      final boolean bySimFloor =
          simFloor.calculate(
              Utils.isSimulation()
                  && simHeightMeters <= kSimFloorEpsilon.in(Meters)
                  && Math.abs(simVelocityMps) < 0.01);
      return byCurrent || byStall || bySimFloor;
    }

    @Override
    public void end(boolean interrupted) {
      leader.setControl(neutralRequest);
      homing = false;
      if (!interrupted) {
        leader.setPosition(Rotations.of(0));
        follower.setPosition(Rotations.of(0));
        setpointMeters = 0.0;
        homed = true;
        System.out.println("Elevator: homed (t=" + timer.get() + " s)");
      } else {
        System.out.println("Elevator: home() interrupted/timed out after " + timer.get() + " s — NOT homed");
      }
    }
  }

  // ─────────────────────────── periodic ───────────────────────────

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderStatorCurrent, leaderMotorVoltage);
    heightMeters = metersOf(leaderPosition.getValueAsDouble());
    velocityRps = leaderVelocity.getValueAsDouble();
    statorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    motorVolts = leaderMotorVoltage.getValueAsDouble();

    final boolean withinTol =
        homed
            && !homing
            && Math.abs(heightMeters - setpointMeters) < ElevatorConstants.kTolerance.in(Meters);
    atSetpointLatched = atSetpointDebouncer.calculate(withinTol);

    carriageLigament.setLength(Math.max(0.0, heightMeters));

    Logger.recordOutput("Elevator/height_m", heightMeters);
    Logger.recordOutput("Elevator/setpoint_m", setpointMeters);
    Logger.recordOutput("Elevator/homed", homed);
    Logger.recordOutput("Elevator/homing", homing);
    Logger.recordOutput("Elevator/current_a", statorCurrentAmps);
    Logger.recordOutput("Elevator/atSetpoint", atSetpointLatched);
    Logger.recordOutput("Elevator/velocity_mps", metersOf(velocityRps));
    Logger.recordOutput("Elevator/appliedVolts", motorVolts);
    if (Utils.isSimulation()) {
      Logger.recordOutput("Elevator/sim/height_m", simHeightMeters);
    }
    Logger.recordOutput("Elevator/Mechanism2d", mech2d);
  }

  // ─────────────────────────── simulation ───────────────────────────

  private void startSimThread() {
    // Effective drum radius folds the rigging factor in so the sim state is carriage height directly.
    final double effectiveDrumRadius =
        ElevatorConstants.kDrumRadius.in(Meters) * ElevatorConstants.kRiggingFactor;
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.kSensorToMechanismRatio,
            kSimCarriageMassKg,
            effectiveDrumRadius,
            0.0,
            ElevatorConstants.kMaxHeight.in(Meters), // H-01 placeholder, never elevator_upper_bound
            true,
            kSimStartHeight.in(Meters));

    final TalonFXSimState leaderSim = leader.getSimState();
    final TalonFXSimState followerSim = follower.getSimState();
    leaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    leaderSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    followerSim.Orientation = ChassisReference.CounterClockwise_Positive;
    followerSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    // Make the very first signal read agree with the sim start height (not 0).
    pushRotorState(leaderSim, followerSim, kSimStartHeight.in(Meters), 0.0);

    lastSimTime = Utils.getCurrentTimeSeconds();
    simNotifier =
        new Notifier(
            () -> {
              final double now = Utils.getCurrentTimeSeconds();
              final double dt = now - lastSimTime;
              lastSimTime = now;

              leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
              followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              elevatorSim.setInputVoltage(leaderSim.getMotorVoltage());
              elevatorSim.update(dt);

              simHeightMeters = elevatorSim.getPositionMeters();
              simVelocityMps = elevatorSim.getVelocityMetersPerSecond();
              pushRotorState(leaderSim, followerSim, simHeightMeters, simVelocityMps);
            });
    simNotifier.setName("ElevatorSim");
    simNotifier.startPeriodic(kSimLoopPeriodSeconds);
  }

  /** rotor rotations = mechanism rotations × SensorToMechanismRatio (the ONE place the ratio is applied by us). */
  private static void pushRotorState(
      TalonFXSimState leaderSim, TalonFXSimState followerSim, double heightM, double velocityMps) {
    final double rotorRot = rotOf(heightM) * ElevatorConstants.kSensorToMechanismRatio;
    final double rotorRps = rotOf(velocityMps) * ElevatorConstants.kSensorToMechanismRatio;
    leaderSim.setRawRotorPosition(Rotations.of(rotorRot));
    leaderSim.setRotorVelocity(RotationsPerSecond.of(rotorRps));
    followerSim.setRawRotorPosition(Rotations.of(rotorRot));
    followerSim.setRotorVelocity(RotationsPerSecond.of(rotorRps));
  }
}
