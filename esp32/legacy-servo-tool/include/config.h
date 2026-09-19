// Subzero tool-changer firmware — hardware/config constants.
// Contract: docs/contracts.md §C.2 (ESP32 HTTP API v0) — implement byte-identically, never rename.
//
// Every TODO(hardware) constant here corresponds to exactly one H-id in
// 00-brief/hardware-todos.md. Humans fill in the real value there first, then here.
// Placeholders below are chosen to fail safe (short travel, low speed, conservative timeouts).

#pragma once

// ---------------------------------------------------------------------------
// H-09 — tool identity
// 2 tools total, MG996R-class servos, 2S LiPo + 6V buck per tool, no latch-confirm
// switch fitted yet (LATCH_SWITCH_PIN below is wired but not guaranteed populated).
// TOOL_ID / TOOL_IP_LAST_OCTET are injected from platformio.ini build_flags per board
// (-D TOOL_ID=1 -D TOOL_IP_LAST_OCTET=31 for tool 1, =2/=32 for tool 2, etc).
// ---------------------------------------------------------------------------
#ifndef TOOL_ID
#define TOOL_ID 1  // TODO(hardware) H-09 — overridden by platformio.ini build_flags per board
#endif
#ifndef TOOL_IP_LAST_OCTET
#define TOOL_IP_LAST_OCTET 31  // TODO(hardware) H-09 — overridden by platformio.ini build_flags per board
#endif

// ---------------------------------------------------------------------------
// H-10 — servo travel limits
// LATCH_*_DEG: latch servo angle for closed/open. LATERAL_*_DEG: lateral servo angle
// range mapped linearly to LATERAL_MIN_MM..LATERAL_MAX_MM (0-40mm slide travel).
// ---------------------------------------------------------------------------
#define LATCH_OPEN_DEG    20   // TODO(hardware) H-10 — latch servo angle, tool released
#define LATCH_CLOSED_DEG  110  // TODO(hardware) H-10 — latch servo angle, tool latched

#define LATERAL_MIN_DEG   0    // TODO(hardware) H-10 — lateral servo angle at 0mm
#define LATERAL_MAX_DEG   110  // TODO(hardware) H-10 — lateral servo angle at LATERAL_MAX_MM
#define LATERAL_MIN_MM    0.0f
#define LATERAL_MAX_MM    40.0f  // TODO(hardware) H-10 — physical lateral slide travel, mm

// mm -> deg linear map: deg = LATERAL_MIN_DEG + (mm - LATERAL_MIN_MM) *
//       (LATERAL_MAX_DEG - LATERAL_MIN_DEG) / (LATERAL_MAX_MM - LATERAL_MIN_MM)
// (implemented in src/main.cpp; mm is clamped to [LATERAL_MIN_MM, LATERAL_MAX_MM] first per §C.2 Δ)

// ---------------------------------------------------------------------------
// H-11 — e-stop vs link-loss behaviour (DECIDED at the gate; hardware half open:
// does a limp latch drop the tool? if yes, flip ESTOP_BEHAVIOUR to HOLD).
// ---------------------------------------------------------------------------
#define DETACH 1
#define HOLD   0

#define ESTOP_BEHAVIOUR_DETACH DETACH          // TODO(hardware) H-11 — alternative value, if needed
// Decided at the gate: POST /estop detaches both servos (PWM off, limp), state=ESTOP.
#define ESTOP_BEHAVIOUR ESTOP_BEHAVIOUR_DETACH  // TODO(hardware) H-11 — flip to HOLD if a limp latch drops the tool

// Link loss (1000ms silence, see HEARTBEAT_TIMEOUT_MS) always HOLDs last commanded
// targets and never detaches — this is fixed by contract §C.2, not a hardware TODO.
#define LINK_LOSS_HOLD HOLD
#define LINK_LOSS_BEHAVIOUR LINK_LOSS_HOLD

// ---------------------------------------------------------------------------
// H-12 — GPIO assignment (verified safe on ESP32-S3-DevKitC-1-N8: not strapping
// pins 0/3/45/46, not USB D+/D- 19/20, not UART0 43/44, not the nonexistent 26-32).
// ---------------------------------------------------------------------------
#define SERVO_LATCH_PIN    4  // TODO(hardware) H-12 — latch servo PWM
#define SERVO_LATERAL_PIN  5  // TODO(hardware) H-12 — lateral servo PWM
#define LATCH_SWITCH_PIN   6  // TODO(hardware) H-12 — optional latch-confirm switch, INPUT_PULLUP (not fitted per H-09)

// ---------------------------------------------------------------------------
// H-13 — WiFi + static IP. SSID/PSK unknown until the venue radio is confirmed
// 2.4GHz; placeholders below are deliberately unusable until a human fills them in.
// ---------------------------------------------------------------------------
#define WIFI_SSID "subzero"     // TODO(hardware) H-13 — real SSID, confirm 2.4GHz
#define WIFI_PSK  "changeme"    // TODO(hardware) H-13 — real passphrase

#define TOOL_IP_OCTET1 10
#define TOOL_IP_OCTET2 13
#define TOOL_IP_OCTET3 60
// TOOL_IP = 10.13.60.<TOOL_IP_LAST_OCTET> — tool1=.31, tool2=.32, tool3=.33 (TODO(hardware) H-13)
#define GATEWAY_IP_OCTET4 1     // 10.13.60.1
#define SUBNET_MASK "255.255.255.0"

// ---------------------------------------------------------------------------
// H-25 — servo current draw. Informational only (sizes the buck/LiPo, not used
// directly by firmware logic), kept here so the number lives in exactly one place.
// ---------------------------------------------------------------------------
#define SERVO_STALL_A 2.5f  // TODO(hardware) H-25 — amps/servo @ 6V stalled; buck must supply >= 5A for both

// ---------------------------------------------------------------------------
// H-26 — status LED. DevKitC-1 v1.0 -> on-board WS2812 RGB on GPIO48; v1.1
// silkscreen -> GPIO38. The Arduino core hardcodes RGB_BUILTIN=48 with no
// revision detection, so this stays a hardware TODO even though it compiles.
// Plain-LED alternative: wire a normal LED to GPIO7 and set STATUS_LED_IS_RGB=0.
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN RGB_BUILTIN  // TODO(hardware) H-26 — 48 on v1.0 board, 38 on v1.1 silkscreen; GPIO7 if plain LED
#define STATUS_LED_IS_RGB 1         // TODO(hardware) H-26 — set to 0 + STATUS_LED_PIN=7 for a plain on/off LED

// ---------------------------------------------------------------------------
// Timing (frozen by docs/contracts.md §C.2 — not hardware TODOs)
// ---------------------------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 1000  // silence beyond this -> state = LOST_LINK
#define WDT_TIMEOUT_MS       5000  // task watchdog
#define HTTP_PORT            80
