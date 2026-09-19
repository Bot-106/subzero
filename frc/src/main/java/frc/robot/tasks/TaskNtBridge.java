package frc.robot.tasks;

import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TaskConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.tasks.TaskPrimitives.TaskReport;
import frc.robot.tasks.TaskPrimitives.TaskState;
import org.littletonrobotics.junction.Logger;

/**
 * NT4 side of the task contract (docs/contracts.md C.1): `/subzero/task/request` → primitive
 * factory → `/subzero/task/state`, plus robot telemetry and the soft e-stop.
 *
 * <p>Requests are drained with `readQueue()` (never a bare get) so no queued command is collapsed;
 * `seq <= lastSeen` is ignored. A SubsystemBase only so that `periodic()` runs every loop — nothing
 * ever requires it.
 */
public class TaskNtBridge extends SubsystemBase {
  private final Elevator elevator;
  private final Arm arm;
  private final CommandSwerveDrivetrain drivetrain;
  private final ToolClient toolClient;
  private final TaskPrimitives primitives;

  private final StringSubscriber requestSub;
  private final StringPublisher statePub;
  private final BooleanPublisher homedPub;
  private final DoublePublisher heightPub;
  private final DoublePublisher extensionPub;
  private final StructPublisher<Pose2d> posePub;
  private final BooleanSubscriber estopSub;

  private final ObjectMapper mapper = new ObjectMapper();
  private long lastSeenSeq = 0;
  private boolean estopLatched = false;
  private String lastStateJson = "";

  public TaskNtBridge(
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Arm arm,
      ToolClient toolClient,
      TaskPrimitives primitives) {
    this(NetworkTableInstance.getDefault(), drivetrain, elevator, arm, toolClient, primitives);
  }

  public TaskNtBridge(
      NetworkTableInstance inst,
      CommandSwerveDrivetrain drivetrain,
      Elevator elevator,
      Arm arm,
      ToolClient toolClient,
      TaskPrimitives primitives) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.arm = arm;
    this.toolClient = toolClient;
    this.primitives = primitives;

    requestSub =
        inst.getStringTopic("/subzero/task/request")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.02));
    statePub =
        inst.getStringTopic("/subzero/task/state")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.02));
    homedPub = inst.getBooleanTopic("/subzero/robot/homed").publish();
    heightPub = inst.getDoubleTopic("/subzero/elevator/height_m").publish();
    extensionPub = inst.getDoubleTopic("/subzero/arm/extension_m").publish();
    posePub = inst.getStructTopic("/subzero/robot/pose", Pose2d.struct).publish();
    estopSub = inst.getBooleanTopic("/subzero/estop").subscribe(false);

    homedPub.set(false);
    setName("TaskNtBridge");
  }

  public TaskPrimitives getPrimitives() {
    return primitives;
  }

  public boolean isEstopActive() {
    return estopLatched;
  }

  // ───────────────────────────── state out ─────────────────────────────

  /** Publish one C.1 `/subzero/task/state` message (also the TaskPrimitives state sink). */
  public void publishState(TaskReport r) {
    publishState(r.seq(), r.primitive(), r.state(), r.detail());
  }

  public void publishState(long seq, String primitive, TaskState state, String detail) {
    String json =
        "{\"seq\":"
            + seq
            + ",\"primitive\":\""
            + escape(primitive)
            + "\",\"state\":\""
            + state.name()
            + "\",\"detail\":\""
            + escape(detail == null ? "" : detail)
            + "\"}";
    lastStateJson = json;
    statePub.set(json);
    Logger.recordOutput("Task/StateJson", json);
    Logger.recordOutput("Task/State", state);
  }

  private static String escape(String s) {
    StringBuilder b = new StringBuilder(s.length() + 8);
    for (char c : s.toCharArray()) {
      switch (c) {
        case '"' -> b.append("\\\"");
        case '\\' -> b.append("\\\\");
        case '\n' -> b.append("\\n");
        case '\r' -> b.append("\\r");
        case '\t' -> b.append("\\t");
        default -> {
          if (c < 0x20) {
            b.append(String.format("\\u%04x", (int) c));
          } else {
            b.append(c);
          }
        }
      }
    }
    return b.toString();
  }

  // ───────────────────────────── periodic ─────────────────────────────

  @Override
  public void periodic() {
    toolClient.poll();
    pollEstop();
    drainRequests();
    publishTelemetry();
  }

  /** `/subzero/estop` rising edge → cancelAll + abort() (ABORTED "estop"); idempotent while true. */
  private void pollEstop() {
    boolean estop = estopSub.get(false);
    if (estop && !estopLatched) {
      estopLatched = true;
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(primitives.abortWith("estop"));
      DriverStation.reportWarning("[subzero] /subzero/estop asserted — all commands cancelled", false);
    } else if (!estop && estopLatched) {
      estopLatched = false;
      DriverStation.reportWarning("[subzero] /subzero/estop cleared", false);
    }
    Logger.recordOutput("Task/EstopActive", estopLatched);
  }

  private void drainRequests() {
    for (String json : requestSub.readQueueValues()) {
      handleRequest(json);
    }
  }

  private void publishTelemetry() {
    boolean homed = primitives.isHomed();
    homedPub.set(homed);
    heightPub.set(elevator.getHeight().in(Meters));
    extensionPub.set(arm.getExtension().in(Meters));
    posePub.set(drivetrain.getPose());
    Logger.recordOutput("Robot/Homed", homed);
    Logger.recordOutput("Task/LastSeenSeq", lastSeenSeq);
  }

  // ───────────────────────────── requests in ─────────────────────────────

  /** Parse one C.1 request and schedule the matching primitive. Package-private for tests. */
  void handleRequest(String json) {
    Logger.recordOutput("Task/LastRequestJson", json);
    long seq;
    String primitive;
    JsonNode args;
    try {
      JsonNode root = mapper.readTree(json);
      if (root == null || !root.isObject()) {
        publishState(0, "?", TaskState.FAILED, "bad json");
        return;
      }
      seq = root.path("seq").asLong(0);
      primitive = root.path("primitive").asText("");
      args = root.path("args");
    } catch (Exception e) {
      publishState(0, "?", TaskState.FAILED, "bad json");
      return;
    }
    if (seq <= lastSeenSeq) {
      return; // contract: consumers ignore seq <= lastSeen
    }
    lastSeenSeq = seq;

    if (!DriverStation.isEnabled()) {
      publishState(seq, primitive, TaskState.FAILED, "disabled");
      return;
    }
    if (estopLatched && !"abort".equals(primitive)) {
      publishState(seq, primitive, TaskState.FAILED, "estop");
      return;
    }

    Command cmd;
    try {
      cmd = build(primitive, args);
    } catch (IllegalArgumentException e) {
      publishState(seq, primitive, TaskState.FAILED, "bad args: " + e.getMessage());
      return;
    }
    if (cmd == null) {
      publishState(seq, primitive, TaskState.FAILED, "unknown");
      return;
    }
    primitives.setRequestSeq(seq);
    CommandScheduler.getInstance().schedule(cmd);
  }

  /** Maps a C.3 primitive name + args onto its factory; null = unknown primitive. */
  private Command build(String primitive, JsonNode args) {
    switch (primitive) {
      case "alignToTag":
        return primitives.alignToTag(
            requireInt(args, "tagId"),
            optDouble(args, "offsetX_m", TaskConstants.kPickOffsetX_m),
            optDouble(args, "offsetY_m", TaskConstants.kPickOffsetY_m),
            optDouble(args, "yaw_deg", TaskConstants.kPickYaw_deg));
      case "elevatorTo":
        return primitives.elevatorTo(elevatorTarget(args));
      case "armTo":
        return primitives.armTo(armTarget(args));
      case "toolOp":
        return primitives.toolOp(
            optInt(args, "tool", TaskConstants.kDefaultToolId),
            requireText(args, "op"),
            optDouble(args, "arg", 0.0));
      case "swapTool":
        return primitives.swapTool(requireInt(args, "tool"));
      case "pickAt":
        return primitives.pickAt(requireInt(args, "locationTagId"));
      case "placeAt":
        return primitives.placeAt(requireInt(args, "locationTagId"));
      case "abort":
        return primitives.abort();
      case "stow": // not a C.3 primitive, but the HRI CLI sends it
        return primitives.stow();
      default:
        return null;
    }
  }

  /** `height_m` or a `setpoint` name from ElevatorConstants (H-03). */
  private static Distance elevatorTarget(JsonNode args) {
    if (args.hasNonNull("height_m")) {
      return Meters.of(args.get("height_m").asDouble());
    }
    String name = args.path("setpoint").asText("");
    switch (name) {
      case "stow":
        return ElevatorConstants.kStowHeight;
      case "ground":
        return ElevatorConstants.kGroundHeight;
      case "top":
        return ElevatorConstants.kTopHeight;
      case "rackSlot1":
        return ElevatorConstants.kRackSlot1Height;
      case "rackSlot2":
        return ElevatorConstants.kRackSlot2Height;
      case "pick":
        return ElevatorConstants.kPickHeight;
      case "place":
        return ElevatorConstants.kPlaceHeight;
      default:
        throw new IllegalArgumentException("height_m or setpoint required");
    }
  }

  /** `extension_m` or a `setpoint` name from ArmConstants (H-07). */
  private static Distance armTarget(JsonNode args) {
    if (args.hasNonNull("extension_m")) {
      return Meters.of(args.get("extension_m").asDouble());
    }
    String name = args.path("setpoint").asText("");
    switch (name) {
      case "retracted":
        return ArmConstants.kRetracted;
      case "rack":
        return ArmConstants.kRackExtension;
      case "pick":
        return ArmConstants.kPickExtension;
      case "place":
        return ArmConstants.kPlaceExtension;
      default:
        throw new IllegalArgumentException("extension_m or setpoint required");
    }
  }

  private static int requireInt(JsonNode args, String key) {
    if (!args.hasNonNull(key) || !args.get(key).isNumber()) {
      throw new IllegalArgumentException(key + " required");
    }
    return args.get(key).asInt();
  }

  private static int optInt(JsonNode args, String key, int dflt) {
    return args.hasNonNull(key) && args.get(key).isNumber() ? args.get(key).asInt() : dflt;
  }

  private static double optDouble(JsonNode args, String key, double dflt) {
    return args.hasNonNull(key) && args.get(key).isNumber() ? args.get(key).asDouble() : dflt;
  }

  private static String requireText(JsonNode args, String key) {
    if (!args.hasNonNull(key) || !args.get(key).isTextual()) {
      throw new IllegalArgumentException(key + " required");
    }
    return args.get(key).asText();
  }

  public String getLastStateJson() {
    return lastStateJson;
  }
}
