// TimedMotor — one brushed DC motor on a DRV8833, driven OPEN LOOP as timed runs:
// "run fwd/rev for N ms at speed 0..1". No feedback. Non-blocking: run() returns at once,
// update() (from loop()) ends the run at its deadline with stop().
//
//   begin()                 pins + LEDC; driver ASLEEP (nSLEEP LOW), outputs coast. Nothing moves.
//   run(dir, ms, speed01)   wake + start immediately; REPLACES any run in progress.
//                           ms clamped to [0, RUN_MAX_MS], speed to [0, RUN_MAX_SPEED].
//   stop()                  cancel the run; brake (RUN_END_BRAKE=1) or coast. Driver stays awake.
//   sleep()                 stop (coast) + nSLEEP LOW = outputs Hi-Z = limp (ESTOP detach).
//   wake()                  nSLEEP HIGH (run() calls this itself).
//   update()                call every loop(); ends the run at its deadline.
//
// Safety: every run has a deadline <= RUN_MAX_MS, checked in update(); if loop() ever stalls
// the task watchdog resets the board and begin() puts the driver back to sleep.
// Direction: "fwd" = PWM on MOTOR_IN1_PIN with IN2 LOW; MOTOR_INVERT=1 swaps the physical
// pins for both directions. Direction changes always pass through coast (never both-PWM).

#pragma once

#include <Arduino.h>
#include "config.h"

class TimedMotor {
 public:
  enum Direction { FWD, REV };

  void begin();
  void run(Direction dir, uint32_t ms, float speed01);
  void stop();
  void sleep();
  void wake();
  void update();

  bool running() const { return running_; }
  uint32_t remainingMs() const;
  Direction lastDir() const { return lastDir_; }
  const char *lastDirName() const { return dirName(lastDir_); }
  float lastSpeed() const { return lastSpeed_; }   // 0..1 as clamped and applied
  uint32_t lastMs() const { return lastMs_; }      // ms as clamped and applied
  bool awake() const { return awake_; }
  bool driverFault();                              // DRV8833 nFAULT asserted (false if pin = -1)

  static const char *dirName(Direction d) { return d == FWD ? "fwd" : "rev"; }
  static uint32_t clampMs(uint32_t ms) { return ms > RUN_MAX_MS ? (uint32_t)RUN_MAX_MS : ms; }
  static float clampSpeed(float s);

 private:
  void outCoast();
  void outBrake();
  void outDrive(Direction dir, uint32_t duty);

  bool awake_ = false;
  bool running_ = false;
  unsigned long startMs_ = 0;
  uint32_t lastMs_ = 0;
  float lastSpeed_ = 0.0f;
  Direction lastDir_ = FWD;
};
