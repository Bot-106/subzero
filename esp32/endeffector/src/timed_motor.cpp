// TimedMotor — open-loop timed runs on a DRV8833 channel. See include/timed_motor.h.

#include "timed_motor.h"

#include <math.h>

static_assert(RUN_MAX_MS > 0, "RUN_MAX_MS is mandatory (never leave the motor running)");
static_assert(RUN_MAX_SPEED > 0.0f && RUN_MAX_SPEED <= 1.0f, "RUN_MAX_SPEED must be in (0, 1]");
static_assert(MOTOR_PWM_BITS >= 1 && MOTOR_PWM_BITS <= 14, "MOTOR_PWM_BITS out of LEDC range");

static const uint32_t kDutyMax = (1u << MOTOR_PWM_BITS) - 1;  // 1023 at 10 bit; core maps this to 100 % HIGH

float TimedMotor::clampSpeed(float s) {
  if (isnan(s) || s < 0.0f) return 0.0f;
  if (s > RUN_MAX_SPEED) return RUN_MAX_SPEED;
  return s;
}

// --- raw outputs (write the pin that goes LOW first: direction changes pass through coast)
void TimedMotor::outCoast() {  // both LOW = outputs Hi-Z
  ledcWrite(MOTOR_IN1_PIN, 0);
  ledcWrite(MOTOR_IN2_PIN, 0);
}

void TimedMotor::outBrake() {  // both HIGH = motor terminals shorted (dynamic braking, no supply current)
  ledcWrite(MOTOR_IN1_PIN, kDutyMax);
  ledcWrite(MOTOR_IN2_PIN, kDutyMax);
}

void TimedMotor::outDrive(Direction dir, uint32_t duty) {
  bool forward = (dir == FWD);
#if MOTOR_INVERT
  forward = !forward;
#endif
  if (duty > kDutyMax) duty = kDutyMax;
  if (forward) { ledcWrite(MOTOR_IN2_PIN, 0); ledcWrite(MOTOR_IN1_PIN, duty); }
  else         { ledcWrite(MOTOR_IN1_PIN, 0); ledcWrite(MOTOR_IN2_PIN, duty); }
}

// --- lifecycle
void TimedMotor::begin() {
  pinMode(MOTOR_NSLEEP_PIN, OUTPUT);
  digitalWrite(MOTOR_NSLEEP_PIN, LOW);  // driver asleep until the first run()
  ledcAttach(MOTOR_IN1_PIN, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(MOTOR_IN2_PIN, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  outCoast();
#if MOTOR_NFAULT_PIN >= 0
  pinMode(MOTOR_NFAULT_PIN, INPUT_PULLUP);
#endif
  awake_ = false;
  running_ = false;
  Serial.printf("[motor] begin: driver asleep, pwm %d Hz / %d bit, invert=%d, max %lu ms @ %.2f\n",
                MOTOR_PWM_HZ, MOTOR_PWM_BITS, MOTOR_INVERT, (unsigned long)RUN_MAX_MS, (double)RUN_MAX_SPEED);
}

void TimedMotor::wake() {
  if (awake_) return;
  outCoast();
  digitalWrite(MOTOR_NSLEEP_PIN, HIGH);  // DRV8833 wakes in ~1 ms; the first ~1 ms of a run is lost, negligible
  awake_ = true;
}

void TimedMotor::sleep() {
  running_ = false;
  outCoast();
  digitalWrite(MOTOR_NSLEEP_PIN, LOW);  // Hi-Z outputs = limp
  awake_ = false;
  Serial.println("[motor] sleep: coast + driver asleep");
}

void TimedMotor::stop() {
  bool wasRunning = running_;
  running_ = false;
#if RUN_END_BRAKE
  outBrake();
#else
  outCoast();
#endif
  if (wasRunning) Serial.printf("[motor] stop (%s)\n", RUN_END_BRAKE ? "brake" : "coast");
}

void TimedMotor::run(Direction dir, uint32_t ms, float speed01) {
  ms = clampMs(ms);
  float speed = clampSpeed(speed01);
  uint32_t duty = (uint32_t)lroundf(speed * (float)kDutyMax);
  lastDir_ = dir;
  lastMs_ = ms;
  lastSpeed_ = speed;
  if (ms == 0 || duty == 0) {  // nothing to do: end in the stopped state, never "running"
    wake();
    stop();
    Serial.printf("[motor] run %s 0 ms / duty 0 -> stop\n", dirName(dir));
    return;
  }
  wake();
  startMs_ = millis();   // set before running_ so remainingMs() is never stale
  running_ = true;       // replaces any run in progress (new deadline, new direction)
  outDrive(dir, duty);
  Serial.printf("[motor] run %s %lu ms @ %.2f (duty %lu/%lu)\n", dirName(dir), (unsigned long)ms, (double)speed,
                (unsigned long)duty, (unsigned long)kDutyMax);
}

void TimedMotor::update() {
  if (!running_) return;
  if ((unsigned long)(millis() - startMs_) >= (unsigned long)lastMs_) stop();  // deadline (wrap-safe)
}

uint32_t TimedMotor::remainingMs() const {
  if (!running_) return 0;
  unsigned long elapsed = millis() - startMs_;
  return elapsed >= lastMs_ ? 0 : (uint32_t)(lastMs_ - elapsed);
}

bool TimedMotor::driverFault() {
#if MOTOR_NFAULT_PIN >= 0
  return digitalRead(MOTOR_NFAULT_PIN) == LOW;
#else
  return false;
#endif
}
