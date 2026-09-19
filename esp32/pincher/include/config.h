// Subzero pincher firmware — hardware/config constants.
// Contract: docs/contracts.md §C.2 (ESP32 HTTP API v0) — implement byte-identically, never rename.
//
// The pincher is tool id 0, on the arm carriage, at 10.13.60.30. It carries TWO real
// micro-servos mounted as mirrored jaws that pinch an end-effector tool to pick it off
// the rack, carry it, and put it back. The six contract routes are re-mapped onto the
// jaws: /latch = close (pinch), /release = open, /lateral{mm} = jaw gap in mm.
//
// Every TODO(hardware) constant here corresponds to exactly one H-id. Humans fill in the
// real value on the hardware side first, then here. Placeholders below are chosen to fail
// safe (conservative angles, slow slew, HOLD on both e-stop and link loss).

#pragma once

// ---------------------------------------------------------------------------
// H-09 — tool identity. TOOL_ID / TOOL_IP_LAST_OCTET are injected from
// platformio.ini build_flags (-D TOOL_ID=0 -D TOOL_IP_LAST_OCTET=30). The
// #ifndef defaults below only matter if someone compiles this file standalone
// without those build flags.
// ---------------------------------------------------------------------------
#ifndef TOOL_ID
#define TOOL_ID 0  // TODO(hardware) H-09 — overridden by platformio.ini build_flags; pincher = 0
#endif
#ifndef TOOL_IP_LAST_OCTET
#define TOOL_IP_LAST_OCTET 30  // TODO(hardware) H-09 — overridden by platformio.ini build_flags; pincher = 30
#endif

// ---------------------------------------------------------------------------
// H-10 — jaw servo travel limits.
// Two servos (A, B) are mounted mirrored: as the jaw gap opens, servo A's angle
// moves one direction and servo B's angle moves the opposite direction, so the
// same mm -> angle formula is applied to each servo's own OPEN/CLOSED pair
// (the "mirroring" lives in these constants, not in extra code branches).
//   gap_mm = 0            -> both jaws fully CLOSED (pinched onto the tool)
//   gap_mm = JAW_MAX_MM   -> both jaws fully OPEN
// CALIBRATION WARNING: these angle placeholders are guesses. Find the real
// values on the bench with /lateral at small mm steps (see README.md) BEFORE
// pinching an actual tool, to avoid crushing it or stalling a servo against a
// mechanical hard stop.
// ---------------------------------------------------------------------------
#define JAW_A_OPEN_DEG    20   // TODO(hardware) H-10 — servo A angle, jaws fully open
#define JAW_A_CLOSED_DEG  110  // TODO(hardware) H-10 — servo A angle, jaws fully closed (pinched)
#define JAW_B_OPEN_DEG    160  // TODO(hardware) H-10 — servo B angle, jaws fully open (mirrored)
#define JAW_B_CLOSED_DEG  70   // TODO(hardware) H-10 — servo B angle, jaws fully closed (mirrored)

#define JAW_MIN_MM  0.0f
#define JAW_MAX_MM  40.0f  // TODO(hardware) H-10 — physical jaw gap travel, mm, at fully open

// mm -> deg linear map per servo (implemented in src/main.cpp; mm is clamped to
// [JAW_MIN_MM, JAW_MAX_MM] first per contract §C.2 Δ):
//   deg = CLOSED_DEG + (mm - JAW_MIN_MM) * (OPEN_DEG - CLOSED_DEG) / (JAW_MAX_MM - JAW_MIN_MM)

// ---------------------------------------------------------------------------
// Motion quality — non-blocking slew limiter (never slam the jaws onto a tool).
// A servo `write()` target is approached at this rate from loop()'s update(),
// driven by millis(); `moving` in /status is true while a jaw is still slewing
// toward its target. A conservative default; raise only after bench testing
// shows the mechanism and the servos tolerate it.
// ---------------------------------------------------------------------------
#define JAW_SLEW_DEG_PER_S 120.0f

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// ---------------------------------------------------------------------------
// H-11 — e-stop vs link-loss behaviour.
// DELIBERATE CHOICE for THIS board (differs from the end-effector boards,
// which DETACH per gate decision H-11 in docs/contracts.md — see README.md):
// the pincher may be HOLDING A TOOL over people's feet, so /estop here HOLDs
// — stop any motion, keep both servos attached at their current (already-
// slewed) angle, state="ESTOP" — instead of going limp and dropping whatever
// is pinched. DETACH is the alternative, selectable below, in case hardware
// says otherwise (e.g. a stalled/overheating servo needs to de-energize).
// Either way, the next /latch, /release or /lateral re-attaches (if detached)
// and clears the state back to OK.
// ---------------------------------------------------------------------------
#define ESTOP_HOLD   0
#define ESTOP_DETACH 1

// Pincher default: HOLD (keep pinching, do not drop the tool). Flip to
// ESTOP_DETACH here to go limp on /estop instead — TODO(hardware) H-11: confirm
// with the mechanical/electrical side which failure mode is safer for this rig.
#define ESTOP_BEHAVIOUR ESTOP_HOLD  // TODO(hardware) H-11 — HOLD chosen deliberately; DETACH is the alternative

// Link loss (HEARTBEAT_TIMEOUT_MS silence) ALWAYS holds last commanded targets
// and never detaches, regardless of ESTOP_BEHAVIOUR above — this is fixed by
// contract §C.2 / safety.md §5 (tool servos fail safe), not a hardware TODO.
#define LINK_LOSS_HOLD 1
#define LINK_LOSS_BEHAVIOUR LINK_LOSS_HOLD

// ---------------------------------------------------------------------------
// H-12 — GPIO assignment (verified safe on ESP32-S3-DevKitC-1-N8: not
// strapping pins 0/3/45/46, not USB D+/D- 19/20, not UART0 43/44, not the
// nonexistent 26-32). No latch-confirm switch on this board (dropped from the
// legacy-servo-tool reference — the pincher has no such input).
// ---------------------------------------------------------------------------
#define SERVO_A_PIN 4  // TODO(hardware) H-12 — jaw servo A PWM
#define SERVO_B_PIN 5  // TODO(hardware) H-12 — jaw servo B PWM

// ---------------------------------------------------------------------------
// H-13 — WiFi + static IP. SSID/PSK unknown until the venue radio is confirmed
// 2.4GHz; placeholders below are deliberately unusable until a human fills
// them in.
// ---------------------------------------------------------------------------
#define WIFI_SSID "subzero"     // TODO(hardware) H-13 — real SSID, confirm 2.4GHz
#define WIFI_PSK  "changeme"    // TODO(hardware) H-13 — real passphrase

#define TOOL_IP_OCTET1 10
#define TOOL_IP_OCTET2 13
#define TOOL_IP_OCTET3 60
// TOOL_IP = 10.13.60.<TOOL_IP_LAST_OCTET> — pincher (tool 0) = .30 (TODO(hardware) H-13)
#define GATEWAY_IP_OCTET4 1     // 10.13.60.1
#define SUBNET_MASK "255.255.255.0"

// ---------------------------------------------------------------------------
// H-26 — status LED. DevKitC-1 v1.0 -> on-board WS2812 RGB on GPIO48; v1.1
// silkscreen -> GPIO38. The Arduino core hardcodes RGB_BUILTIN=48 with no
// revision detection, so this stays a hardware TODO even though it compiles.
// Plain-LED alternative: wire a normal LED to GPIO7 and set
// STATUS_LED_IS_RGB=0.
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN RGB_BUILTIN  // TODO(hardware) H-26 — 48 on v1.0 board, 38 on v1.1 silkscreen; GPIO7 if plain LED
#define STATUS_LED_IS_RGB 1         // TODO(hardware) H-26 — set to 0 + STATUS_LED_PIN=7 for a plain on/off LED

// ---------------------------------------------------------------------------
// Timing (frozen by docs/contracts.md §C.2 — not hardware TODOs)
// ---------------------------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 1000  // silence beyond this -> state = LOST_LINK (jaws HOLD)
#define WDT_TIMEOUT_MS       5000  // task watchdog
#define HTTP_PORT            80
