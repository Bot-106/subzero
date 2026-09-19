// EmuServo — DC motor (DRV8833) + pot closed-loop position control, hobby-servo-shaped.
// See include/emu_servo.h for the API and README.md "Direction / sign conventions".

#include "emu_servo.h"

#include <math.h>

static_assert(POT_MV_AT_0DEG != POT_MV_AT_180DEG, "POT_MV_AT_0DEG and POT_MV_AT_180DEG must differ (H-10)");
static_assert(EMU_MIN_DUTY >= 0 && EMU_MIN_DUTY <= EMU_MAX_DUTY, "EMU_MIN_DUTY must be <= EMU_MAX_DUTY");
static_assert(EMU_MAX_DUTY <= ((1 << MOTOR_PWM_BITS) - 1), "EMU_MAX_DUTY exceeds the PWM resolution");
static_assert(EMU_STALL_TIMEOUT_MS > 0, "EMU_STALL_TIMEOUT_MS is mandatory (never drive the motor forever)");
static_assert(EMU_SOFT_MIN_DEG >= 0.0f && EMU_SOFT_MAX_DEG <= 180.0f && EMU_SOFT_MIN_DEG < EMU_SOFT_MAX_DEG,
              "EMU_SOFT_*_DEG must lie inside [0, 180]");

static const uint32_t kDutyMax = (1u << MOTOR_PWM_BITS) - 1;  // 1023 at 10 bit; core maps this to 100 % HIGH

// ---------------------------------------------------------------------------
// Static maps
// ---------------------------------------------------------------------------
float EmuServo::mvToDeg(float mv) {
  return (mv - (float)POT_MV_AT_0DEG) * 180.0f / ((float)POT_MV_AT_180DEG - (float)POT_MV_AT_0DEG);
}

float EmuServo::degToMv(float deg) {
  return (float)POT_MV_AT_0DEG + deg * ((float)POT_MV_AT_180DEG - (float)POT_MV_AT_0DEG) / 180.0f;
}

float EmuServo::clampDeg(float deg) {
  if (isnan(deg)) return EMU_SOFT_MIN_DEG;
  if (deg < EMU_SOFT_MIN_DEG) return EMU_SOFT_MIN_DEG;
  if (deg > EMU_SOFT_MAX_DEG) return EMU_SOFT_MAX_DEG;
  return deg;
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------
const char *EmuServo::stateName() const {
  switch (state_) {
    case IDLE: return "IDLE";
    case MOVING: return "MOVING";
    case AT_TARGET: return "AT_TARGET";
    case FAULT: return "FAULT";
  }
  return "IDLE";
}

const char *EmuServo::faultName() const {
  switch (fault_) {
    case FAULT_NONE: return "none";
    case FAULT_STALL: return "stall";
    case FAULT_POT_RANGE: return "pot_range";
  }
  return "none";
}

// ---------------------------------------------------------------------------
// Raw outputs (DRV8833 channel A). Always write the pin that goes LOW first so a
// direction change passes through coast, never through a both-PWM state.
// ---------------------------------------------------------------------------
void EmuServo::outForward(uint32_t duty) {
  ledcWrite(MOTOR_IN2_PIN, 0);
  ledcWrite(MOTOR_IN1_PIN, duty);
}

void EmuServo::outReverse(uint32_t duty) {
  ledcWrite(MOTOR_IN1_PIN, 0);
  ledcWrite(MOTOR_IN2_PIN, duty);
}

void EmuServo::outBrake() {  // both HIGH = motor terminals shorted (dynamic braking, no supply current)
  ledcWrite(MOTOR_IN1_PIN, kDutyMax);
  ledcWrite(MOTOR_IN2_PIN, kDutyMax);
}

void EmuServo::outCoast() {  // both LOW = outputs Hi-Z
  ledcWrite(MOTOR_IN1_PIN, 0);
  ledcWrite(MOTOR_IN2_PIN, 0);
}

// duty is LOGICAL: + = toward increasing degrees (toward POT_MV_AT_180DEG).
// POT_INVERT flips which physical pin gets the PWM.
void EmuServo::applyDuty(int duty) {
  if (duty == 0) {
#if EMU_HOLD_BRAKE
    outBrake();
#else
    outCoast();
#endif
    lastDuty_ = 0;
    return;
  }
  int mag = duty < 0 ? -duty : duty;
  if ((uint32_t)mag > kDutyMax) mag = (int)kDutyMax;
  bool forward = (duty > 0);
#if POT_INVERT
  forward = !forward;
#endif
  if (forward) outForward((uint32_t)mag);
  else outReverse((uint32_t)mag);
  lastDuty_ = duty;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void EmuServo::begin() {
  pinMode(MOTOR_NSLEEP_PIN, OUTPUT);
  digitalWrite(MOTOR_NSLEEP_PIN, LOW);  // driver asleep until attach()

  ledcAttach(MOTOR_IN1_PIN, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(MOTOR_IN2_PIN, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  outCoast();

#if MOTOR_NFAULT_PIN >= 0
  pinMode(MOTOR_NFAULT_PIN, INPUT_PULLUP);
#endif

  analogReadResolution(12);
  analogSetPinAttenuation(POT_ADC_PIN, ADC_11db);  // ~0-3.1 V usable; pot is 3V3-wiper-GND

  posMv_ = (float)readMilliVolts();
  posDeg_ = mvToDeg(posMv_);
  havePos_ = true;
  targetDeg_ = clampDeg(posDeg_);

  attached_ = false;
  state_ = IDLE;
  fault_ = FAULT_NONE;
  lastDuty_ = 0;
  Serial.printf("[emu] begin: pot=%.0f mV -> %.1f deg (cal %d/%d mV, invert=%d)\n",
                posMv_, posDeg_, POT_MV_AT_0DEG, POT_MV_AT_180DEG, POT_INVERT);
}

void EmuServo::attach() {
  if (attached_) return;
  outCoast();
  digitalWrite(MOTOR_NSLEEP_PIN, HIGH);  // DRV8833 wakes in ~1 ms; first PWM edge is at most one loop tick later
  attached_ = true;
  // Hold wherever the axis is until write() says otherwise.
  posMv_ = (float)readMilliVolts();
  posDeg_ = mvToDeg(posMv_);
  targetDeg_ = clampDeg(posDeg_);
  integ_ = 0.0f;
  inTol_ = false;
  state_ = MOVING;
  rearmStall(fabsf(targetDeg_ - posDeg_));
  Serial.printf("[emu] attach: hold at %.1f deg\n", targetDeg_);
}

void EmuServo::detach() {
  outCoast();
  digitalWrite(MOTOR_NSLEEP_PIN, LOW);  // asleep = Hi-Z outputs = limp
  attached_ = false;
  state_ = IDLE;
  lastDuty_ = 0;
  integ_ = 0.0f;
  inTol_ = false;
  Serial.println("[emu] detach: coast + driver asleep");
}

void EmuServo::clearFault() {
  if (fault_ == FAULT_NONE && state_ != FAULT) return;
  fault_ = FAULT_NONE;
  if (state_ == FAULT) state_ = attached_ ? MOVING : IDLE;
  Serial.println("[emu] fault cleared");
}

void EmuServo::write(float deg) {
  float clamped = clampDeg(deg);
  clearFault();
  targetDeg_ = clamped;
  integ_ = 0.0f;
  inTol_ = false;
  posMv_ = (float)readMilliVolts();
  posDeg_ = mvToDeg(posMv_);
  havePos_ = true;
  rearmStall(fabsf(targetDeg_ - posDeg_));
  if (attached_) state_ = MOVING;
  Serial.printf("[emu] write %.1f deg (req %.1f) from %.1f deg%s\n", targetDeg_, deg, posDeg_,
                attached_ ? "" : " [detached: stored only]");
}

float EmuServo::read() const { return posDeg_; }

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
uint32_t EmuServo::readMilliVolts() {
  uint32_t sum = 0;
  for (int i = 0; i < POT_ADC_SAMPLES; ++i) sum += analogReadMilliVolts(POT_ADC_PIN);
  return sum / POT_ADC_SAMPLES;
}

uint16_t EmuServo::readRaw() { return (uint16_t)analogRead(POT_ADC_PIN); }

bool EmuServo::driverFault() {
#if MOTOR_NFAULT_PIN >= 0
  return digitalRead(MOTOR_NFAULT_PIN) == LOW;
#else
  return false;
#endif
}

// ---------------------------------------------------------------------------
// Faults / stall guard
// ---------------------------------------------------------------------------
void EmuServo::enterFault(Fault f) {
#if EMU_FAULT_BRAKE
  outBrake();
#else
  outCoast();
#endif
  lastDuty_ = 0;
  integ_ = 0.0f;
  inTol_ = false;
  fault_ = f;
  state_ = FAULT;
  Serial.printf("[emu] FAULT %s: target %.1f pos %.1f deg (%.0f mV) — motor %s until next command\n",
                faultName(), targetDeg_, posDeg_, posMv_, EMU_FAULT_BRAKE ? "braked" : "coasting");
}

void EmuServo::rearmStall(float absErr) {
  stallBestAbsErr_ = absErr;
  stallSinceMs_ = millis();
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
void EmuServo::update() {
  const unsigned long periodUs = 1000000UL / EMU_LOOP_HZ;
  unsigned long nowUs = micros();
  if (ticked_ && (nowUs - lastUpdateUs_) < periodUs) return;
  float dt = ticked_ ? (float)(nowUs - lastUpdateUs_) * 1e-6f : (1.0f / EMU_LOOP_HZ);
  if (dt < 1e-4f) dt = 1e-4f;
  lastUpdateUs_ = nowUs;
  ticked_ = true;
  unsigned long nowMs = millis();

  // 1. Sense (always, even when detached, so /status shows the live position).
  posMv_ = (float)readMilliVolts();
  float prev = havePos_ ? posDeg_ : mvToDeg(posMv_);
  posDeg_ = mvToDeg(posMv_);
  havePos_ = true;
  float vel = (posDeg_ - prev) / dt;  // deg/s (measurement derivative -> no setpoint kick)

  if (!attached_) { lastDuty_ = 0; return; }  // outputs are coast + nSLEEP low already
  if (state_ == FAULT) return;                 // outputs were set in enterFault(); wait for a command

  // 2. Pot wiring sanity (optional window, H-10).
  if ((POT_MV_FAULT_LOW > 0 && posMv_ < (float)POT_MV_FAULT_LOW) || posMv_ > (float)POT_MV_FAULT_HIGH) {
    enterFault(FAULT_POT_RANGE);
    return;
  }

  float err = targetDeg_ - posDeg_;
  float absErr = fabsf(err);

  // 3. Tolerance band with hysteresis -> brake + settle.
  if (inTol_) {
    if (absErr > EMU_TOLERANCE_DEG + EMU_REENGAGE_DEG) {
      inTol_ = false;
      if (state_ == AT_TARGET) Serial.printf("[emu] AT_TARGET -> MOVING (pushed to %.1f deg)\n", posDeg_);
      state_ = MOVING;
      integ_ = 0.0f;
      rearmStall(absErr);
    }
  } else if (absErr < EMU_TOLERANCE_DEG) {
    inTol_ = true;
    inTolSinceMs_ = nowMs;
  }

  if (inTol_) {
    applyDuty(0);  // brake (or coast) inside the band
    integ_ = 0.0f;
    if (state_ != AT_TARGET && (nowMs - inTolSinceMs_) >= EMU_SETTLE_MS) {
      state_ = AT_TARGET;
      Serial.printf("[emu] AT_TARGET %.1f deg (target %.1f)\n", posDeg_, targetDeg_);
    }
    return;
  }

  // 4. PID outside the band -> signed duty with stiction kick and cap.
  state_ = MOVING;
  float iTerm = 0.0f;
  if (EMU_KI > 0.0f) {
    integ_ += err * dt;
    float lim = (float)EMU_MAX_DUTY / EMU_KI;  // anti-windup
    if (integ_ > lim) integ_ = lim;
    if (integ_ < -lim) integ_ = -lim;
    iTerm = EMU_KI * integ_;
  }
  float u = EMU_KP * err + iTerm - EMU_KD * vel;
  int mag = (int)lroundf(fabsf(u));
  if (mag < EMU_MIN_DUTY) mag = EMU_MIN_DUTY;
  if (mag > EMU_MAX_DUTY) mag = EMU_MAX_DUTY;
  bool positive = (u != 0.0f) ? (u > 0.0f) : (err > 0.0f);
  int duty = positive ? mag : -mag;

  // 5. Stall / no-progress guard (mandatory).
  if (absErr < stallBestAbsErr_ - EMU_STALL_PROGRESS_DEG) {
    stallBestAbsErr_ = absErr;
    stallSinceMs_ = nowMs;
  } else if ((nowMs - stallSinceMs_) >= EMU_STALL_TIMEOUT_MS) {
    enterFault(FAULT_STALL);
    return;
  }

  applyDuty(duty);
}
