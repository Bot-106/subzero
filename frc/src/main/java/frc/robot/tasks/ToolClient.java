package frc.robot.tasks;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TaskConstants;
import org.littletonrobotics.junction.Logger;

/**
 * RoboRIO side of the tool-changer contract (docs/contracts.md C.1, `/subzero/tool/*`).
 *
 * <p>Publishes `/subzero/tool/cmd` (JSON, keepDuplicates + periodic 0.02 so the bridge never
 * collapses two commands), waits for `/subzero/tool/<n>/lastSeq >= seq` from the Python bridge, and
 * gates everything on the bridge heartbeat `/subzero/bridge/alive` (comms-architecture §5: bridge down
 * → toolOp fails fast, the robot still drives).
 *
 * <p>Tool ids are 1..{@link TaskConstants#kToolIdCount} (H-13: the RoboRIO only knows ids; the IPs
 * live in integration/bridge/config.yaml).
 */
public class ToolClient {
  /** Outcome of the most recent tool op, mirrored into the C.1 task state by TaskPrimitives. */
  public enum Outcome {
    NONE,
    DONE,
    FAILED,
    ABORTED
  }

  private final StringPublisher cmdPub;
  private final IntegerSubscriber[] lastSeqSubs; // index 0 unused; ids are 1..N
  private final BooleanSubscriber aliveSub;

  private long seq = 0;
  private Outcome lastOutcome = Outcome.NONE;
  private String lastDetail = "";

  // Heartbeat change detector: the bridge toggles /subzero/bridge/alive every ~200 ms, so "fresh"
  // means the value OR its timestamp changed within kBridgeAliveStaleSeconds.
  private boolean seenAlive = false;
  private boolean lastAliveValue = false;
  private long lastAliveTimestampUs = 0;
  private double lastAliveChangeSec = Double.NEGATIVE_INFINITY;

  public ToolClient() {
    this(NetworkTableInstance.getDefault());
  }

  public ToolClient(NetworkTableInstance inst) {
    cmdPub =
        inst.getStringTopic("/subzero/tool/cmd")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.02));
    lastSeqSubs = new IntegerSubscriber[TaskConstants.kToolIdCount + 1];
    for (int id = 1; id <= TaskConstants.kToolIdCount; id++) {
      lastSeqSubs[id] = inst.getIntegerTopic("/subzero/tool/" + id + "/lastSeq").subscribe(0);
    }
    aliveSub = inst.getBooleanTopic("/subzero/bridge/alive").subscribe(false);
  }

  /** Call once per loop (TaskNtBridge.periodic) so the heartbeat change detector stays current. */
  public void poll() {
    TimestampedBoolean a = aliveSub.getAtomic(false);
    if (a.timestamp == 0) {
      return; // never received
    }
    if (!seenAlive || a.value != lastAliveValue || a.timestamp != lastAliveTimestampUs) {
      seenAlive = true;
      lastAliveValue = a.value;
      lastAliveTimestampUs = a.timestamp;
      lastAliveChangeSec = Timer.getFPGATimestamp();
    }
    Logger.recordOutput("Tool/BridgeAlive", aliveFresh());
  }

  /** True while the bridge heartbeat changed within {@link TaskConstants#kBridgeAliveStaleSeconds}. */
  public boolean isBridgeAlive() {
    poll();
    return aliveFresh();
  }

  private boolean aliveFresh() {
    return seenAlive
        && (Timer.getFPGATimestamp() - lastAliveChangeSec) < TaskConstants.kBridgeAliveStaleSeconds;
  }

  public boolean isValidToolId(int tool) {
    return tool >= 1 && tool <= TaskConstants.kToolIdCount;
  }

  /** Last `seq` acknowledged by tool n (0 if unknown / invalid id). */
  public long lastSeq(int tool) {
    return isValidToolId(tool) ? lastSeqSubs[tool].get(0) : 0;
  }

  public long getSeq() {
    return seq;
  }

  public Outcome getLastOutcome() {
    return lastOutcome;
  }

  public String getLastDetail() {
    return lastDetail;
  }

  /** Publishes one C.1 tool command (JSON built by hand — the shape is fixed and tiny). */
  private void publish(long s, int tool, String op, double arg) {
    String json =
        "{\"seq\":" + s + ",\"tool\":" + tool + ",\"op\":\"" + op + "\",\"arg\":" + arg + "}";
    cmdPub.set(json);
    Logger.recordOutput("Tool/LastCmd", json);
  }

  /**
   * C.3 `toolOp`: seq++ → publish → DONE when `lastSeq(tool) >= seq`, FAILED "ack timeout" after
   * {@link TaskConstants#kToolAckTimeoutSeconds}, FAILED "bridge offline" immediately (without
   * publishing) when the heartbeat is stale — except `estop`, which is always published (idempotent).
   */
  public Command op(int tool, String op, double arg) {
    return new ToolOpCommand(tool, op, arg);
  }

  /** Concrete command so the timeout is classified precisely (not confused with an interrupt). */
  public class ToolOpCommand extends Command {
    private final int tool;
    private final String op;
    private final double arg;
    private final Timer timer = new Timer();
    private long mySeq = -1;
    private boolean refused = false;
    private Outcome outcome = Outcome.NONE;
    private String detail = "";

    ToolOpCommand(int tool, String op, double arg) {
      this.tool = tool;
      this.op = op;
      this.arg = arg;
      setName("toolOp(" + tool + "," + op + "," + arg + ")");
    }

    @Override
    public void initialize() {
      refused = false;
      outcome = Outcome.NONE;
      detail = "";
      boolean isEstop = "estop".equals(op);
      if (!isValidToolId(tool)) {
        refused = true;
        outcome = Outcome.FAILED;
        detail = "bad tool id " + tool;
        return;
      }
      if (!isEstop && !isBridgeAlive()) {
        refused = true;
        outcome = Outcome.FAILED;
        detail = "bridge offline";
        return;
      }
      seq++;
      mySeq = seq;
      publish(mySeq, tool, op, arg);
      timer.restart();
    }

    private boolean acked() {
      return mySeq >= 0 && lastSeq(tool) >= mySeq;
    }

    @Override
    public boolean isFinished() {
      return refused || acked() || timer.hasElapsed(TaskConstants.kToolAckTimeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
      if (!refused) {
        if (acked()) {
          outcome = Outcome.DONE;
          detail = "";
        } else if (interrupted) {
          outcome = Outcome.ABORTED;
          detail = "interrupted";
        } else {
          outcome = Outcome.FAILED;
          detail = "ack timeout";
        }
      }
      lastOutcome = outcome;
      lastDetail = detail;
      Logger.recordOutput("Tool/LastOutcome", outcome);
      Logger.recordOutput("Tool/LastDetail", detail);
    }

    public Outcome getOutcome() {
      return outcome;
    }

    public String getDetail() {
      return detail;
    }

    @Override
    public boolean runsWhenDisabled() {
      return false; // safety §1: deadman = DS enable
    }
  }
}
