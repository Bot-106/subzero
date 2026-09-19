// EmuServo — a brushed DC motor (DRV8833 H-bridge) + potentiometer position feedback,
// wrapped in the shape of a hobby-servo API so main.cpp can treat it like `Servo`.
//
//   begin()            configure pins (PWM, nSLEEP, nFAULT, ADC); driver left ASLEEP
//   attach()           nSLEEP HIGH, closed loop enabled (target = current position until write())
//   detach()           both IN LOW + nSLEEP LOW: motor coasts, driver asleep (ESTOP "limp")
//   write(deg)         set target (clamped to [EMU_SOFT_MIN_DEG, EMU_SOFT_MAX_DEG]); clears FAULT
//   read()             current angle from the pot (deg)
//   update()           run the control loop; call from loop() — it self-paces at EMU_LOOP_HZ
//
// Sign convention (see README "Direction / sign conventions"): "forward" = PWM on
// MOTOR_IN1_PIN with IN2 LOW and must move the axis toward INCREASING degrees, i.e.
// toward POT_MV_AT_180DEG. POT_INVERT=1 swaps forward/reverse.
//
// Degrees <-> millivolts: linear through (POT_MV_AT_0DEG, 0) and (POT_MV_AT_180DEG, 180).
//
// Safety: while driving, if |error| stops improving for EMU_STALL_TIMEOUT_MS the loop
// enters FAULT (motor coasts) — the motor is never left driven forever.

#pragma once

#include <Arduino.h>
#include "config.h"

class EmuServo {
 public:
  enum State { IDLE, MOVING, AT_TARGET, FAULT };
  enum Fault { FAULT_NONE, FAULT_STALL, FAULT_POT_RANGE };

  void begin();
  void attach();
  void detach();
  bool attached() const { return attached_; }

  // Hobby-servo-shaped API
  void write(float deg);     // set target; clamps; clears FAULT; re-arms the stall guard
  float read() const;        // current position, deg (from the last update() pot read)
  float target() const { return targetDeg_; }
  bool atTarget() const { return attached_ && state_ == AT_TARGET; }
  State state() const { return state_; }
  Fault fault() const { return fault_; }
  const char *stateName() const;
  const char *faultName() const;
  void clearFault();

  // Sensor access (fresh reads, safe to call from an HTTP handler on the loop task)
  uint32_t readMilliVolts();  // averaged POT_ADC_SAMPLES x analogReadMilliVolts()
  uint16_t readRaw();         // single analogRead(), 12-bit
  bool driverFault();         // DRV8833 nFAULT asserted (LOW); false if pin = -1

  // Control loop tick — call every loop(); does nothing until 1/EMU_LOOP_HZ has elapsed.
  void update();

  // Diagnostics
  int lastDuty() const { return lastDuty_; }  // signed, +forward / -reverse, 0 = brake/coast

  static float mvToDeg(float mv);
  static float degToMv(float deg);
  static float clampDeg(float deg);

 private:
  void outForward(uint32_t duty);
  void outReverse(uint32_t duty);
  void outBrake();
  void outCoast();
  void applyDuty(int duty);   // signed; applies POT_INVERT; 0 -> brake or coast per hold mode
  void enterFault(Fault f);
  void rearmStall(float absErr);

  bool attached_ = false;
  State state_ = IDLE;
  Fault fault_ = FAULT_NONE;

  float targetDeg_ = 0.0f;
  float posDeg_ = 0.0f;
  float posMv_ = 0.0f;
  bool havePos_ = false;

  float integ_ = 0.0f;
  int lastDuty_ = 0;

  bool ticked_ = false;
  unsigned long lastUpdateUs_ = 0;
  unsigned long inTolSinceMs_ = 0;
  bool inTol_ = false;

  float stallBestAbsErr_ = 0.0f;
  unsigned long stallSinceMs_ = 0;
};
