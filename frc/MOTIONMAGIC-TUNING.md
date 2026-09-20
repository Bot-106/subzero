# MotionMagic tuning — quick guide (Phoenix 6, Kraken X60)

Applies to `frc/src/main/java/frc/robot/subsystems/Arm.java` (constants in `Constants.ArmConstants`)
and the elevator (`Elevator.java`, Tuner X generated — same ideas, edit the numbers in that file).

## 1. What MotionMagic actually does
Plain position PID gets a step setpoint and slams toward it. **MotionMagic runs a trapezoidal motion
profile *on the TalonFX*:** every 1 ms it generates an intermediate position/velocity target that ramps
up at `MotionMagicAcceleration`, cruises at `MotionMagicCruiseVelocity`, and ramps down to land on the
setpoint. The controller then does two things at once:

```
V = kS·sign(v_target) + kV·v_target + kA·a_target + kG          ← feedforward, "what voltage should I need"
  + kP·(pos_target − pos) + kI·∫err + kD·(vel_target − vel)     ← feedback,    "fix whatever is left"
```

So **feedforward does most of the work and PID only cleans up.** A well-tuned MotionMagic axis has a
small kP and follows the profile almost exactly. If you find yourself needing a huge kP, the FF is wrong.

Units (this matters more than anything): the pulley is directly on the Kraken's spline shaft
(`SensorToMechanismRatio = 1`), so every position/velocity the TalonFX reports and every gain is in
**pulley rotations = rotor rotations**. The arm's
14-tooth HTD-5 pulley moves the belt (and the carriage, since the belt is anchored at both ends)
14 × 5 mm = **0.070 m per pulley revolution** (`kMetersPerRotation`). So:
- kP = **volts per rotation of error** (12 → 1 rot (7 cm) of error asks for 12 V)
- kV = **volts per rotation/second**
- kS = volts, kG = volts, kA = volts per rot/s²
- `MotionMagicCruiseVelocity` = rot/s, `MotionMagicAcceleration` = rot/s²
  (`Constants.ArmConstants` holds these in m/s and m/s² and converts: 0.30 m/s = 4.3 rot/s, 1.0 m/s² = 14.3 rot/s²).

## 2. Why the arm's FF is different from an elevator or a pivot
| Mechanism | Gravity term |
|---|---|
| Pivot arm (`Arm_Cosine`) | kG·cos(angle) — changes with angle |
| Elevator (`Elevator_Static`) | constant kG holding the carriage up |
| **This arm — horizontal linear axis** | **none: kG = 0**. Only kS (friction) + kV (speed) (+ kA) |

The arm is configured `GravityType.Elevator_Static` with `kG = 0`; if the rail turns out to be tilted,
a small constant kG is the only thing to add. Never use `Arm_Cosine` for it.

## 3. Before you tune (5 min, with a spotter — safety.md §4/§6)
1. **Direction.** Power on, jog with the **right trigger** (extend). Position in AdvantageScope /
   Tuner X must **increase** and the carriage must move **out**. If it moves in, flip `Inverted` in
   the MotorOutput config; if the sign of the position is wrong, same fix.
2. **Zero.** Retract fully by hand or with the left trigger, press **Back** (`zeroHere`). The soft limits
   (`kSoftLimitIn` 0, `kSoftLimitOut` 0.43 m) are measured from this zero — set `kMaxExtension` /
   `kSoftLimitOut` to the real travel (H-06) before any setpoint test.
3. **Ratio check.** Jog out a known distance (tape measure), read `Arm/position_rot`.
   `metres ÷ rotations` must equal `kMetersPerRotation` (0.070 — fixed by the 14T HTD-5 pulley on the
   motor shaft). If it doesn't, the belt is slipping or the pulley isn't 14T — fix that first.
4. Keep the stator limit low (60 A) and cruise/accel low for the first runs.

## 4. Tune in this order (each step ~2 min)
Log `Arm/position_rot`, `Arm/setpoint_m`, `Arm/velocity_mps`, `Arm/appliedVolts`, `Arm/statorCurrent_a`
(already in the AK log), or plot `Position`, `ClosedLoopReference`, `ClosedLoopError`, `MotorVoltage`
in Tuner X → Plot.

1. **kS (static friction).** Set kP = kV = 0. Jog with the trigger and note the smallest duty that
   just starts moving, ×12 V → kS (typically 0.1–0.4 V; placeholder 0.20 — direct drive has no gearbox
   to multiply torque, so carriage friction shows up 1:1 and kS tends to be on the higher side). Same both ways?
   If very different, the rail is tilted → that difference/2 is your kG.
2. **kV (velocity).** Theory: Kraken X60 free speed ≈ 100 rot/s at 12 V, and the pulley is on the
   rotor → kV ≈ 12/100 = **0.12 V per rot/s** (placeholder). Direct drive means the unloaded axis could
   do 100 rot/s × 0.07 m = 7 m/s — never let the profile ask for that; cruise stays ≤ ~1 m/s. Verify: command a
   move with kP still 0; during the cruise part `velocity` should sit near the profile's cruise. Lower
   than target → raise kV; overshooting/running fast → lower it. Loaded mechanisms usually land
   5–15 % above theory.
3. **kP.** Start at ~5 V/rot, command Retracted ↔ Rack (LB/RB), watch `ClosedLoopError`. Double it
   until the error during the move is a few hundredths of a rotation and it doesn't oscillate at
   the end; then back off 20–30 %. Typical: 5–20. (Placeholder 12.)
4. **kD** only if it overshoots at the end of a move: 0.05–0.5 (units V per rot/s of velocity error).
5. **kI**: leave at 0. If it stops just short every time, that is kS being slightly low — raise kS.
6. **kA**: usually 0 for a light carriage. If the error spikes during the accel/decel ramps only, add
   0.01–0.05.
7. **Cruise / accel.** Now raise `kCruiseVelocity` / `kAcceleration` toward what you want
   (`0.30 m/s`, `1.0 m/s²` placeholders → 6 rot/s, 20 rot/s²). Rule of thumb: cruise ≤ 80 % of the
   free speed you measured in step 2 — but for this direct-drive axis cap cruise at ~1 m/s (14 rot/s)
   regardless; accel such that it reaches cruise in ~0.2–0.4 s. If it can't
   keep up, the profile is asking for more than kV·v + kA·a can give and the error grows → lower them.

## 5. What "good" looks like
- `Position` hugs `ClosedLoopReference` the whole way (error < 0.05 rot ≈ 3.5 mm).
- `MotorVoltage` is a smooth trapezoid, not a saw-tooth (saw-tooth = kP too high / kD missing).
- Arrives without overshoot; `Arm/atSetpoint` goes true within ~0.2 s of the profile ending.
- Holding still draws only kS-level voltage (a horizontal axis needs almost nothing to hold).

## 6. Gotchas
- **Direct drive, no gearbox:** the motor has no torque multiplication, so kP's authority is lower than
  on a geared axis and the 60 A stator cap (≈ 105 N of belt force) is what keeps a jam safe. If you ever
  add a reduction, set `kSensorToMechanismRatio` and re-tune kV/kP — they are per mechanism rotation.
- kP is applied to *profile* error, not to the distance to the final target, so a big kP does not make
  the move faster — cruise/accel do. Raise those, not kP, for speed.
- `MotionMagicExpoVoltage` is the alternative profile that derives its shape from kV/kA
  (`MotionMagicExpo_kV/_kA`) — nicer for long moves, same tuning of the gains. Not needed for M1.
- Soft limits clamp the *setpoint* too: asking for 0.60 m when the limit is 0.43 m just goes to 0.43.
- After `setPosition(0)` (Back) the held setpoint is the new position — the arm does not jump.
