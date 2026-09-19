// Subzero END-EFFECTOR firmware — hardware/config constants.
// Contract: docs/contracts.md §C.2 (ESP32 HTTP API v0) — implement byte-identically, never rename.
//
// One brushed DC motor (DRV8833 H-bridge) + one potentiometer coupled to the axis =
// ONE emulated hobby servo (EmuServo).  /latch, /release and /lateral all drive this
// single axis.  Every TODO(hardware) constant below carries an H-id; humans fill in the
// real value (00-brief/hardware-todos.md first, then here).  Placeholders are chosen to
// fail safe: low duty cap, short stall timeout, motor asleep at boot until commanded.

#pragma once

// ---------------------------------------------------------------------------
// H-09 — tool identity (injected from platformio.ini build_flags per board:
// [env:endeffector-1] -> TOOL_ID=1 / .31, [env:endeffector-2] -> TOOL_ID=2 / .32)
// ---------------------------------------------------------------------------
#ifndef TOOL_ID
#define TOOL_ID 1  // TODO(hardware) H-09 — overridden by platformio.ini build_flags per board
#endif
#ifndef TOOL_IP_LAST_OCTET
#define TOOL_IP_LAST_OCTET 31  // TODO(hardware) H-09 — overridden by platformio.ini build_flags per board
#endif

// ---------------------------------------------------------------------------
// H-13 — WiFi + static IP. SSID/PSK unknown until the venue radio is confirmed
// 2.4 GHz; placeholders below are deliberately unusable until a human fills them in.
// ---------------------------------------------------------------------------
#define WIFI_SSID "subzero"     // TODO(hardware) H-13 — real SSID, confirm 2.4GHz
#define WIFI_PSK  "changeme"    // TODO(hardware) H-13 — real passphrase

#define TOOL_IP_OCTET1 10
#define TOOL_IP_OCTET2 13
#define TOOL_IP_OCTET3 60
// TOOL_IP = 10.13.60.<TOOL_IP_LAST_OCTET> — end effector 1 = .31, 2 = .32, 3 = .33 (TODO(hardware) H-13)
#define GATEWAY_IP_OCTET4 1     // 10.13.60.1
#define SUBNET_MASK "255.255.255.0"

// ---------------------------------------------------------------------------
// H-12 — GPIO assignment (ESP32-S3-DevKitC-1-N8 safe pins: 1-10 (ADC1), 4/5/6/7,
// 15-18, 21, 38-42. Avoid 0/3/45/46 strapping, 19/20 USB, 43/44 UART0, 26-32
// flash/PSRAM (do not exist on WROOM-1), 48/38 on-board WS2812).
//
// DRV8833 single channel A:  AIN1 / AIN2 / nSLEEP / nFAULT.
//   forward = PWM on IN1, IN2 LOW      reverse = IN1 LOW, PWM on IN2
//   brake   = both HIGH (motor shorted) coast   = both LOW
//   nSLEEP HIGH = driver enabled; LOW = asleep = outputs Hi-Z = coast (used for ESTOP detach)
//   nFAULT  = open-drain active-low, needs a pull-up (INPUT_PULLUP is fine)
// ---------------------------------------------------------------------------
#define MOTOR_IN1_PIN     4   // TODO(hardware) H-12 — DRV8833 AIN1 (LEDC PWM)
#define MOTOR_IN2_PIN     5   // TODO(hardware) H-12 — DRV8833 AIN2 (LEDC PWM)
#define MOTOR_NSLEEP_PIN  6   // TODO(hardware) H-12 — DRV8833 nSLEEP (HIGH = enabled)
#define MOTOR_NFAULT_PIN  7   // TODO(hardware) H-12 — DRV8833 nFAULT (optional; -1 = not wired)

// The pot MUST be on an ADC1 pin (GPIO1..GPIO10 on the S3). ADC2 (GPIO11..20) is
// shared with the WiFi radio and returns garbage / fails while WiFi is up — and WiFi
// is always up on this board. GPIO1 = ADC1_CH0. Pot: 3V3 -> one end, GND -> other
// end, wiper -> GPIO1 (NEVER 5 V into the ADC pin; optional 1k series + 100 nF to GND).
#define POT_ADC_PIN       1   // TODO(hardware) H-12 — pot wiper, ADC1_CH0 (must stay in GPIO1..10)
#define POT_ADC_SAMPLES   8   // samples averaged per position read (analogReadMilliVolts, factory-calibrated)

// ---------------------------------------------------------------------------
// H-10 — emulated-servo calibration + travel limits
//
// Calibration: the two pot readings (mV, analogReadMilliVolts) at the mechanical
// end stops define the linear deg <-> mV map ("0 deg" and "180 deg" are just names
// for the two ends of travel). Read them from GET /status ("pot_mv") while moving
// the axis by hand to each end stop with the motor asleep (boot default), then
// write them here and rebuild. They may be in either order (the map is linear).
// ---------------------------------------------------------------------------
#define POT_MV_AT_0DEG    300   // TODO(hardware) H-10 — pot mV at the "0 deg" end stop (placeholder)
#define POT_MV_AT_180DEG  2800  // TODO(hardware) H-10 — pot mV at the "180 deg" end stop (placeholder)

// Motor direction sign. CONVENTION: "forward" (PWM on IN1, IN2 LOW) must move the
// axis toward INCREASING degrees, i.e. toward POT_MV_AT_180DEG. If on the first
// power-on the axis runs AWAY from the target, set POT_INVERT 1 (equivalent to
// swapping the two motor leads) — do not touch the calibration points for this.
#define POT_INVERT        0     // TODO(hardware) H-10 — 1 if forward drive DEcreases degrees

// Optional pot-wiring sanity window: readings outside [LOW, HIGH] mV -> FAULT
// "pot_range" (broken wiper / rail short). Disabled by default (0 / 4000) because a
// pot with no gearing may legitimately sit near a rail at an end stop; tighten once
// the calibration points are known (e.g. min(cal)-150 / max(cal)+150).
#define POT_MV_FAULT_LOW  0     // TODO(hardware) H-10 — 0 = disabled
#define POT_MV_FAULT_HIGH 4000  // TODO(hardware) H-10 — 4000 = disabled

// Route -> angle mapping on the ONE axis (same numbers as the legacy servo firmware).
#define EMU_LATCH_OPEN_DEG    20    // TODO(hardware) H-10 — axis angle, object released
#define EMU_LATCH_CLOSED_DEG  110   // TODO(hardware) H-10 — axis angle, object latched
#define LATCHED_TOLERANCE_DEG 5.0f  // TODO(hardware) H-10 — /status "latched" = |pos - closed| < this

#define LATERAL_MIN_DEG   0     // TODO(hardware) H-10 — axis angle at 0 mm
#define LATERAL_MAX_DEG   110   // TODO(hardware) H-10 — axis angle at LATERAL_MAX_MM
#define LATERAL_MIN_MM    0.0f
#define LATERAL_MAX_MM    40.0f  // TODO(hardware) H-10 — physical lateral travel, mm

// mm -> deg linear map: deg = LATERAL_MIN_DEG + (mm - LATERAL_MIN_MM) *
//       (LATERAL_MAX_DEG - LATERAL_MIN_DEG) / (LATERAL_MAX_MM - LATERAL_MIN_MM)
// mm is clamped to [LATERAL_MIN_MM, LATERAL_MAX_MM] first per §C.2 Δ (src/main.cpp).

// Software travel limits: no target is ever commanded outside this window (targets are
// clamped). Must stay inside [0, 180] (the calibrated range).
#define EMU_SOFT_MIN_DEG  0.0f    // TODO(hardware) H-10 — keep a margin off the physical end stop
#define EMU_SOFT_MAX_DEG  180.0f  // TODO(hardware) H-10 — keep a margin off the physical end stop

// ---------------------------------------------------------------------------
// Emulated-servo control loop (P + I + D on angle error -> signed PWM duty).
// Duty units are LEDC counts out of (2^MOTOR_PWM_BITS - 1) = 1023 at 10 bit.
// First-test values are deliberately weak: EMU_MAX_DUTY ~68 %, stall timeout 2 s.
// ---------------------------------------------------------------------------
#define MOTOR_PWM_HZ       20000  // inaudible; DRV8833 max 250 kHz
#define MOTOR_PWM_BITS     10     // 0..1023

#define EMU_KP             12.0f  // duty counts per degree of error  (12 -> saturates at ~58 deg error)
#define EMU_KI             0.0f   // duty counts per (degree*second); 0 = off. Anti-windup clamps to EMU_MAX_DUTY
#define EMU_KD             0.0f   // duty counts per (degree/second) on measured velocity; 0 = off
#define EMU_MIN_DUTY       180    // TODO(hardware) H-10 — stiction kick: never drive below this when outside tolerance
#define EMU_MAX_DUTY       700    // TODO(hardware) H-10 — duty cap (700/1023 = 68 %), LOW ON PURPOSE for the first test
#define EMU_TOLERANCE_DEG  3.0f   // inside this = "there": brake, start the settle timer
#define EMU_REENGAGE_DEG   1.0f   // hysteresis: re-drive only once |error| > TOLERANCE + this
#define EMU_SETTLE_MS      150    // inside tolerance this long -> AT_TARGET
#define EMU_LOOP_HZ        200    // update() rate (from loop(), micros()-paced, no delay())
#define EMU_HOLD_BRAKE     1      // 1 = brake (both IN HIGH) while holding at target, 0 = coast

// Stall / no-progress guard (MANDATORY — safety: never leave the motor driven forever).
// While driving at >= EMU_MIN_DUTY, if |error| has not improved by EMU_STALL_PROGRESS_DEG
// for EMU_STALL_TIMEOUT_MS -> FAULT "stall": motor coasts, emu_state="FAULT" in /status,
// cleared by the next command (/latch, /release, /lateral, /pos). Also catches a
// runaway oscillation (|error| stops improving) and a broken pot wire (no movement seen).
#define EMU_STALL_TIMEOUT_MS   2000  // TODO(hardware) H-10 — must exceed the slowest full-travel move
#define EMU_STALL_PROGRESS_DEG 1.0f  // "progress" = |error| shrinks by at least this
#define EMU_FAULT_BRAKE        0     // 0 = coast on FAULT (spec), 1 = brake on FAULT (holds a bit better)

// ---------------------------------------------------------------------------
// H-11 — e-stop vs link-loss behaviour (DECIDED at the gate; hardware half open:
// does a limp axis drop the object? if yes, flip ESTOP_BEHAVIOUR to HOLD).
// ---------------------------------------------------------------------------
#define DETACH 1
#define HOLD   0

#define ESTOP_BEHAVIOUR_DETACH DETACH          // TODO(hardware) H-11 — alternative value, if needed
// Decided at the gate: POST /estop detaches (nSLEEP LOW + both IN LOW = driver asleep,
// motor coasts / limp), state=ESTOP. Next /latch, /release, /lateral (or /pos) re-attaches.
#define ESTOP_BEHAVIOUR ESTOP_BEHAVIOUR_DETACH  // TODO(hardware) H-11 — flip to HOLD if a limp axis drops the object

// Link loss (1000 ms silence, see HEARTBEAT_TIMEOUT_MS) always HOLDs the last target
// (keeps closed-loop controlling toward it) and never detaches — fixed by contract §C.2.
#define LINK_LOSS_HOLD HOLD
#define LINK_LOSS_BEHAVIOUR LINK_LOSS_HOLD

// Boot behaviour (not in the contract). Fail-safe default: BOOT_DETACHED — the driver
// stays asleep and NOTHING moves until the first command; /status still reports the
// pot position so calibration can be done with the motor off. BOOT_HOLD attaches and
// holds wherever the axis is; BOOT_OPEN attaches and moves to EMU_LATCH_OPEN_DEG (legacy).
#define BOOT_DETACHED 0
#define BOOT_HOLD     1
#define BOOT_OPEN     2
#define EMU_BOOT_BEHAVIOUR BOOT_DETACHED  // TODO(hardware) H-11 — BOOT_OPEN once direction + calibration are verified

// ---------------------------------------------------------------------------
// H-26 — status LED. DevKitC-1 v1.0 -> on-board WS2812 RGB on GPIO48; v1.1
// silkscreen -> GPIO38. The Arduino core hardcodes RGB_BUILTIN=48 with no
// revision detection, so this stays a hardware TODO even though it compiles.
// Colours: green OK, yellow LOST_LINK, red ESTOP, blue WiFi connecting, magenta FAULT.
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN RGB_BUILTIN  // TODO(hardware) H-26 — 48 on v1.0 board, 38 on v1.1 silkscreen; GPIO15 if plain LED
#define STATUS_LED_IS_RGB 1         // TODO(hardware) H-26 — set to 0 + STATUS_LED_PIN=15 for a plain on/off LED

// ---------------------------------------------------------------------------
// Timing (frozen by docs/contracts.md §C.2 — not hardware TODOs)
// ---------------------------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 1000  // silence beyond this -> state = LOST_LINK
#define WDT_TIMEOUT_MS       5000  // task watchdog
#define HTTP_PORT            80
