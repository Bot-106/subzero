package frc.robot.tasks;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * C.3 task primitives (docs/contracts.md) as command factories. Names/args are FROZEN.
 *
 * <p>Every primitive goes through one mechanism, {@link #tracked}: RUNNING on initialize, then
 * DONE / FAILED (with detail) / ABORTED (stick input, estop, cancelAll) on end. Every primitive is
 * `.until(driverInputActive)` and time-boxed (safety §3). `elevatorTo` / `armTo` / composites refuse
 * with FAILED "not homed" until both mechanisms are homed (A-12). Composites are v0 stubs that report
 * FAILED "not implemented" and never move.
 */
public class TaskPrimitives {
  /** C.1 `/subzero/task/state` values. */
  public enum TaskState {
    RUNNING,
    DONE,
    FAILED,
    ABORTED
  }

  /** One `/subzero/task/state` message. */
  public record TaskReport(long seq, String primitive, TaskState state, String detail) {}

  /** Generous mechanism time-box (elevator 1.2 m / arm 0.45 m under MotionMagic take < 3 s). */
  static final double kMechanismTimeoutSeconds = Constants.TaskConstants.kMechanismTimeoutSeconds;

  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Arm arm;
  private final ToolClient toolClient;
  private final BooleanSupplier driverInputActive;
  private final Consumer<TaskReport> stateSink;

  /** Request seq of the next scheduled primitive (set by TaskNtBridge; consumed once on initialize). */
  private long pendingSeq = 0;

  public TaskPrimitives(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Arm arm,
      ToolClient toolClient,
      BooleanSupplier driverInputActive,
      Consumer<TaskReport> stateSink) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.arm = arm;
    this.toolClient = toolClient;
    this.driverInputActive = driverInputActive;
    this.stateSink = stateSink;
  }

  // ───────────────────────────── seq / state plumbing ─────────────────────────────

  /** TaskNtBridge calls this right before scheduling an NT-requested primitive. */
  public void setRequestSeq(long seq) {
    pendingSeq = seq;
  }

  private long consumeSeq() {
    long s = pendingSeq;
    pendingSeq = 0; // controller-triggered primitives report seq 0
    return s;
  }

  public boolean isHomed() {
    return elevator.isHomed() && arm.isHomed();
  }

  private void report(long seq, String primitive, TaskState state, String detail) {
    TaskReport r = new TaskReport(seq, primitive, state, detail == null ? "" : detail);
    Logger.recordOutput("Task/State", state);
    Logger.recordOutput("Task/Primitive", primitive);
    Logger.recordOutput("Task/Seq", seq);
    Logger.recordOutput("Task/Detail", r.detail());
    if (stateSink != null) {
      stateSink.accept(r);
    }
  }

  /** Result of an inner command once it has stopped on its own (not interrupted / stick / timeout). */
  public record Result(boolean ok, String detail) {
    static final Result OK = new Result(true, "");

    static Result fail(String detail) {
      return new Result(false, detail);
    }
  }

  /** A command that only reports FAILED with the given detail and never moves anything. */
  private Command failNow(String primitive, String detail) {
    return Commands.runOnce(() -> report(consumeSeq(), primitive, TaskState.FAILED, detail))
        .withName(primitive + ":" + detail);
  }

  /** Composites / mechanism moves refuse while not homed (contract C.1 Δ, A-12). */
  private Command requireHomed(String primitive, Command whenHomed) {
    return Commands.either(whenHomed, failNow(primitive, "not homed"), this::isHomed)
        .withName(primitive);
  }

  /**
   * The one wrapping mechanism. `inner` is decorated with `.until(driverInputActive)` and
   * `.withTimeout(timeout)`; on end, the outcome is classified in this order: interrupted → ABORTED;
   * stick active → ABORTED "driver input"; resolver ok → DONE; timed out → FAILED "timeout";
   * otherwise FAILED with the resolver's detail.
   */
  private Command tracked(
      String primitive, Command inner, double timeoutSeconds, Supplier<Result> resolver) {
    Command guarded = inner.until(driverInputActive).withTimeout(timeoutSeconds);
    Command wrapped = new WrapperCommand(guarded) {
      private long seq;
      private final Timer timer = new Timer();

      @Override
      public void initialize() {
        seq = consumeSeq();
        timer.restart();
        report(seq, primitive, TaskState.RUNNING, "");
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
        double elapsed = timer.get();
        TaskState state;
        String detail;
        if (interrupted) {
          state = TaskState.ABORTED;
          detail = "interrupted";
        } else if (driverInputActive.getAsBoolean()) {
          state = TaskState.ABORTED;
          detail = "driver input";
        } else {
          Result r = resolver.get();
          if (r.ok()) {
            state = TaskState.DONE;
            detail = r.detail();
          } else if (elapsed >= timeoutSeconds - 0.05) {
            state = TaskState.FAILED;
            detail = "timeout";
          } else {
            state = TaskState.FAILED;
            detail = r.detail();
          }
        }
        report(seq, primitive, state, detail);
      }

      @Override
      public boolean runsWhenDisabled() {
        return false; // safety §1
      }
    };
    wrapped.setName(primitive);
    return wrapped;
  }

  // ───────────────────────────── MVP primitives ─────────────────────────────

  /**
   * C.3 `alignToTag`: target robot pose = tagPose2d.transformBy(Transform2d(offsetX, offsetY, yaw)).
   * Leash (tag unseen > 0.5 s / pose jump > 1 m) and caps live in AlignToTagCommand (W3); the 8 s
   * time-box is {@link DriveConstants#kAlignTimeoutSeconds} (safety §7).
   */
  public Command alignToTag(int tagId, double offsetX_m, double offsetY_m, double yaw_deg) {
    Pose2d offset = new Pose2d(offsetX_m, offsetY_m, Rotation2d.fromDegrees(yaw_deg));
    AlignToTagCommand align =
        new AlignToTagCommand(drivetrain, drivetrain.getCameras(), tagId, offset);
    return tracked(
        "alignToTag",
        align,
        DriveConstants.kAlignTimeoutSeconds,
        () -> align.wasRefused() ? Result.fail("leash") : Result.OK);
  }

  /** C.3 `elevatorTo`: refuses while not homed; DONE when within tolerance. */
  public Command elevatorTo(Distance height) {
    return requireHomed(
        "elevatorTo",
        tracked(
            "elevatorTo",
            elevator.goTo(height),
            kMechanismTimeoutSeconds,
            () -> withinTol(elevator.getHeight(), height, ElevatorConstants.kTolerance)
                    || elevator.atSetpoint()
                ? Result.OK
                : Result.fail("not at setpoint")));
  }

  /** C.3 `armTo`: refuses while not homed; DONE when within tolerance. */
  public Command armTo(Distance extension) {
    return requireHomed(
        "armTo",
        tracked(
            "armTo",
            arm.goTo(extension),
            kMechanismTimeoutSeconds,
            () -> withinTol(arm.getExtension(), extension, ArmConstants.kTolerance)
                    || arm.atSetpoint()
                ? Result.OK
                : Result.fail("not at setpoint")));
  }

  /** C.3 `toolOp`: outcome comes straight from the ToolClient (ack / ack timeout / bridge offline). */
  public Command toolOp(int tool, String op, double arg) {
    return tracked(
        "toolOp",
        toolClient.op(tool, op, arg),
        frc.robot.Constants.TaskConstants.kToolAckTimeoutSeconds + 0.5,
        () ->
            toolClient.getLastOutcome() == ToolClient.Outcome.DONE
                ? Result.OK
                : Result.fail(toolClient.getLastDetail()));
  }

  /** Not a C.3 primitive, but the HRI CLI sends it: stow = arm retracted, then elevator down. */
  public Command stow() {
    return requireHomed(
        "stow",
        tracked(
            "stow",
            new Stow(elevator, arm),
            2 * kMechanismTimeoutSeconds,
            () ->
                withinTol(elevator.getHeight(), ElevatorConstants.kStowHeight, ElevatorConstants.kTolerance)
                        && withinTol(arm.getExtension(), ArmConstants.kRetracted, ArmConstants.kTolerance)
                    ? Result.OK
                    : Result.fail("not stowed")));
  }

  // ───────────────────────────── composites (v0 stubs) ─────────────────────────────

  /** C.3 `swapTool` — v0 stub: FAILED "not implemented", never moves. */
  public Command swapTool(int tool) {
    return requireHomed("swapTool", failNow("swapTool", "not implemented"));
  }

  /** C.3 `pickAt` — v0 stub (gate Q4: location tag + fixed offset only). */
  public Command pickAt(int locationTagId) {
    return requireHomed("pickAt", failNow("pickAt", "not implemented"));
  }

  /** C.3 `placeAt` — v0 stub. */
  public Command placeAt(int locationTagId) {
    return requireHomed("placeAt", failNow("placeAt", "not implemented"));
  }

  // ───────────────────────────── abort ─────────────────────────────

  /** C.3 `abort`: cancel everything, put every subsystem in a safe hold, report ABORTED. */
  public Command abort() {
    return abortWith("abort");
  }

  /** Same as {@link #abort()} with a specific detail (TaskNtBridge uses "estop"). */
  public Command abortWith(String detail) {
    return Commands.runOnce(
            () -> {
              CommandScheduler s = CommandScheduler.getInstance();
              s.cancelAll();
              // Scheduled separately so no group keeps a requirement on the drivetrain: the driver
              // gets the stick back after 0.1 s of Idle; elevator/arm hold until the next goTo.
              s.schedule(elevator.hold());
              s.schedule(arm.hold());
              s.schedule(
                  drivetrain.applyRequest(() -> new SwerveRequest.Idle()).withTimeout(0.1));
              report(consumeSeq(), "abort", TaskState.ABORTED, detail);
            })
        .withName("abort");
  }

  // ───────────────────────────── helpers ─────────────────────────────

  private static boolean withinTol(Distance actual, Distance target, Distance tol) {
    return Math.abs(actual.in(Meters) - target.in(Meters)) <= tol.in(Meters);
  }
}
