// Subzero END-EFFECTOR firmware — hardware/config constants.
// Contract: docs/contracts.md §C.2 (ESP32 HTTP API v0) — implement byte-identically, never rename.
//
// One brushed DC motor on a DRV8833 H-bridge, driven OPEN LOOP: "run forward for x ms at
// speed y" / "run reverse for x ms at speed y". There is NO position feedback of any kind.
// /latch, /release and /lateral are each a timed run on this single motor (TimedMotor).
// Every TODO(hardware) constant below carries an H-id; humans fill in the real value
// (00-brief/hardware-todos.md first, then here). Placeholders are chosen to fail safe:
// low speed, short runs, hard run-length cap, motor asleep at boot until commanded.

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
// H-12 — GPIO assignment (ESP32-S3-DevKitC-1-N8 safe pins: 1-10, 4/5/6/7, 15-18, 21,
// 38-42. Avoid 0/3/45/46 strapping, 19/20 USB, 43/44 UART0, 26-32 flash/PSRAM (do not
// exist on WROOM-1), 48/38 on-board WS2812).
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

// Direction sign. CONVENTION: "fwd" = PWM on IN1, IN2 LOW, and /latch runs "fwd" while
// /release runs "rev"; /lateral runs "fwd" toward increasing mm. If on the first power-on
// `POST /run {"dir":"fwd",...}` moves the mechanism the wrong way (opens instead of
// closes / retracts instead of extends), set MOTOR_INVERT 1 — equivalent to swapping the
// two motor leads. Nothing else needs to change.
#define MOTOR_INVERT      0   // TODO(hardware) H-12 — 1 if "fwd" drives the mechanism the wrong way

// ---------------------------------------------------------------------------
// PWM (LEDC, core 3.x ledcAttach/ledcWrite). Duty counts are 0..(2^MOTOR_PWM_BITS - 1).
// ---------------------------------------------------------------------------
#define MOTOR_PWM_HZ       20000  // inaudible; DRV8833 max 250 kHz
#define MOTOR_PWM_BITS     10     // 0..1023

// ---------------------------------------------------------------------------
// Timed-run limits (SAFETY — the motor can never be left running).
// RUN_MAX_MS is the hard cap on ANY run (/latch, /release, /lateral and /run alike),
// enforced inside TimedMotor::run() independently of the watchdog. RUN_MAX_SPEED caps
// the duty of any run (speed 0..1 -> duty; 0.8 = 80 %).
// ---------------------------------------------------------------------------
#define RUN_MAX_MS         5000   // TODO(hardware) H-10 — hard cap, must exceed the longest real stroke
#define RUN_MAX_SPEED      0.8f   // TODO(hardware) H-10 — duty cap 0..1, LOW ON PURPOSE for the first tests
#define RUN_END_BRAKE      1      // 1 = brake (both IN HIGH) when a run ends / is stopped, 0 = coast

// ---------------------------------------------------------------------------
// H-10 — route -> timed run mapping (open loop, no feedback; calibrate with POST /run,
// see README "Calibration"). All speeds are 0..1 and are further clamped to RUN_MAX_SPEED.
// ---------------------------------------------------------------------------
#define LATCH_RUN_MS       800    // TODO(hardware) H-10 — /latch: run "fwd" this long (time a full closing stroke)
#define LATCH_SPEED        0.5f   // TODO(hardware) H-10 — /latch speed 0..1
#define RELEASE_RUN_MS     800    // TODO(hardware) H-10 — /release: run "rev" this long (time a full opening stroke)
#define RELEASE_SPEED      0.5f   // TODO(hardware) H-10 — /release speed 0..1

// /lateral is DEAD RECKONED: the firmware keeps est_mm (0 at boot), runs "fwd" for
// (mm - est_mm) * LATERAL_MS_PER_MM ms if the request is beyond the estimate, "rev" for
// (est_mm - mm) * LATERAL_MS_PER_MM ms otherwise, and sets est_mm = mm when the run starts.
#define LATERAL_MS_PER_MM  25.0f  // TODO(hardware) H-10 — ms of run per mm of travel at LATERAL_SPEED
#define LATERAL_SPEED      0.5f   // TODO(hardware) H-10 — /lateral speed 0..1
#define LATERAL_MIN_MM     0.0f
#define LATERAL_MAX_MM     40.0f  // TODO(hardware) H-10 — physical lateral travel, mm (contract clamp bound)

// ---------------------------------------------------------------------------
// H-11 — e-stop vs link-loss behaviour (DECIDED at the gate; hardware half open:
// does a limp (coasting) mechanism drop the object? if yes, flip ESTOP_BEHAVIOUR to HOLD).
// ---------------------------------------------------------------------------
#define DETACH 1   // stop + nSLEEP LOW: driver asleep, outputs Hi-Z, motor coasts (limp)
#define HOLD   0   // stop with brake (both IN HIGH, motor shorted), driver stays awake
#define STOP   2   // link loss only: brake, cancel the run, refuse new runs until a heartbeat

// Decided at the gate: POST /estop DETACHes (nSLEEP LOW, motor coasts), state=ESTOP.
// The next /latch, /release, /lateral or /run wakes the driver and clears it.
#define ESTOP_BEHAVIOUR_DETACH DETACH
#define ESTOP_BEHAVIOUR_HOLD   HOLD
#define ESTOP_BEHAVIOUR ESTOP_BEHAVIOUR_DETACH   // TODO(hardware) H-11 — flip to ESTOP_BEHAVIOUR_HOLD if a limp mechanism drops the object

// Link loss (1000 ms silence, see HEARTBEAT_TIMEOUT_MS): the contract says "holds". For an
// open-loop DC motor with no feedback the hold-equivalent is STOP: any run in progress is
// cancelled with a brake, the driver stays awake (braked = holds against back-driving,
// draws no supply current), and NEW runs are refused (HTTP 503 err "lost_link", lastSeq
// not advanced) until a /heartbeat returns. Never DETACHes on link loss alone.
#define LINK_LOSS_BEHAVIOUR STOP                 // TODO(hardware) H-11 — STOP is the only sane open-loop choice; DETACH also compiles

// Boot behaviour (not in the contract): the driver stays ASLEEP and nothing moves until
// the first command. There is no feedback to know where the mechanism is at boot, so the
// firmware assumes est_mm = 0 and latched = false (the mechanism should be stowed/open
// before power-on).

// ---------------------------------------------------------------------------
// H-26 — status LED. DevKitC-1 v1.0 -> on-board WS2812 RGB on GPIO48; v1.1
// silkscreen -> GPIO38. The Arduino core hardcodes RGB_BUILTIN=48 with no
// revision detection, so this stays a hardware TODO even though it compiles.
// Colours: green OK, cyan OK+motor running, yellow LOST_LINK, red ESTOP,
// blue WiFi connecting, magenta DRV8833 nFAULT asserted.
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN RGB_BUILTIN  // TODO(hardware) H-26 — 48 on v1.0 board, 38 on v1.1 silkscreen; GPIO15 if plain LED
#define STATUS_LED_IS_RGB 1         // TODO(hardware) H-26 — set to 0 + STATUS_LED_PIN=15 for a plain on/off LED

// ---------------------------------------------------------------------------
// Timing (frozen by docs/contracts.md §C.2 — not hardware TODOs)
// ---------------------------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 1000  // silence beyond this -> state = LOST_LINK
#define WDT_TIMEOUT_MS       5000  // task watchdog
#define HTTP_PORT            80
