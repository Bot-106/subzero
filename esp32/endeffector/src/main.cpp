// Subzero END-EFFECTOR firmware — ESP32-S3-DevKitC-1 (one board per swappable end effector)
//
// Implements docs/contracts.md §C.2 (ESP32 HTTP API v0) byte-identically:
// GET /status, POST /latch /release /lateral /heartbeat /estop, 404 JSON otherwise.
// Additive (allowed by §C.2 "additive fields ok"): extra /status fields and POST /pos.
//
// Actuator: ONE brushed DC motor on a DRV8833 + a pot on the axis = ONE emulated servo
// (EmuServo). /latch -> EMU_LATCH_CLOSED_DEG, /release -> EMU_LATCH_OPEN_DEG,
// /lateral {"mm"} -> [0, LATERAL_MAX_MM] mapped onto [LATERAL_MIN_DEG, LATERAL_MAX_DEG].
// It is the SAME axis: "latched" in /status = |pos - closed| < LATCHED_TOLERANCE_DEG and
// "lateral_mm" = measured position mapped back to mm.
//
// State machine (contract enum, byte-exact):
//   OK          - normal operation
//   LOST_LINK   - no /heartbeat for > HEARTBEAT_TIMEOUT_MS; keeps controlling toward the
//                 last target (HOLD), never detaches (safety.md §5)
//   ESTOP       - POST /estop detached the axis (H-11 ESTOP_BEHAVIOUR=DETACH: nSLEEP low,
//                 both IN low -> motor coasts); cleared by the next /latch, /release,
//                 /lateral or /pos (re-attach + write target)
// EmuServo FAULT (stall / pot range) is NOT in the contract enum; it is reported in the
// additive "emu_state"/"fault" fields only. "state" stays OK|LOST_LINK|ESTOP.
//
// Sync WebServer per DECISION A-02 (not ESPAsyncWebServer) — one bridge client at a time.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "emu_servo.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WebServer server(HTTP_PORT);
EmuServo axis;

enum ToolState { STATE_OK, STATE_LOST_LINK, STATE_ESTOP };
ToolState state = STATE_OK;

float lateralTargetMM = 0.0f;      // last commanded (clamped) lateral target, echoed on dup
long lastSeq = 0;                  // highest seq acknowledged (dedup per §C.2 Δ)
unsigned long lastHeartbeatMs = 0;

const IPAddress kToolIp(TOOL_IP_OCTET1, TOOL_IP_OCTET2, TOOL_IP_OCTET3, TOOL_IP_LAST_OCTET);
const IPAddress kGatewayIp(TOOL_IP_OCTET1, TOOL_IP_OCTET2, TOOL_IP_OCTET3, GATEWAY_IP_OCTET4);
IPAddress kSubnetMask;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
const char *stateToString(ToolState s) {
  switch (s) {
    case STATE_OK: return "OK";
    case STATE_LOST_LINK: return "LOST_LINK";
    case STATE_ESTOP: return "ESTOP";
  }
  return "OK";
}

float clampMM(float mm) {
  if (isnan(mm)) return LATERAL_MIN_MM;
  if (mm < LATERAL_MIN_MM) return LATERAL_MIN_MM;
  if (mm > LATERAL_MAX_MM) return LATERAL_MAX_MM;
  return mm;
}

float mmToDeg(float mm) {
  float t = (mm - LATERAL_MIN_MM) / (LATERAL_MAX_MM - LATERAL_MIN_MM);
  return (float)LATERAL_MIN_DEG + t * (float)(LATERAL_MAX_DEG - LATERAL_MIN_DEG);
}

float degToMm(float deg) {
  float t = (deg - (float)LATERAL_MIN_DEG) / (float)(LATERAL_MAX_DEG - LATERAL_MIN_DEG);
  return LATERAL_MIN_MM + t * (LATERAL_MAX_MM - LATERAL_MIN_MM);
}

// Measured "latched": the ONE axis is within LATCHED_TOLERANCE_DEG of the closed angle.
bool measuredLatched() { return fabsf(axis.read() - (float)EMU_LATCH_CLOSED_DEG) < LATCHED_TOLERANCE_DEG; }

// Measured lateral position in mm, clamped to the contract range.
float measuredLateralMM() { return clampMM(degToMm(axis.read())); }

void sendJson(int code, const JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  server.sendHeader("Content-Type", "application/json", true);
  server.send(code, "application/json", out);
}

void sendBadJson() {
  Serial.println("[http] 400 bad_json");
  server.send(400, "application/json", "{\"ok\":false,\"seq\":0,\"err\":\"bad_json\"}");
}

void sendNotFound() {
  Serial.printf("[http] 404 %s %s\n", server.method() == HTTP_GET ? "GET" : "POST", server.uri().c_str());
  server.send(404, "application/json", "{\"ok\":false,\"seq\":0,\"err\":\"not_found\"}");
}

// Parses server.arg("plain") into doc. Returns false (and has NOT sent a response)
// if the body is missing or not valid JSON.
bool parseBody(JsonDocument &doc) {
  if (!server.hasArg("plain") || server.arg("plain").length() == 0) return false;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  return !err;
}

// Advances OK -> LOST_LINK when heartbeat has been silent too long. Never touches
// ESTOP (only an explicit actuate clears that) and never detaches on link loss alone
// (safety.md §5 — tool fails safe / holds: EmuServo keeps controlling toward its target).
void updateLinkState() {
  if (state == STATE_OK && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    state = STATE_LOST_LINK;
    Serial.println("[state] OK -> LOST_LINK (heartbeat timeout; holding target)");
  }
}

// Clears ESTOP back to OK when an actuate arrives. Does not touch LOST_LINK
// (still gated by heartbeat cadence); mirrors §C.2 semantics where re-actuation
// implicitly resumes control.
void clearEstopIfNeeded() {
  if (state == STATE_ESTOP) {
    state = STATE_OK;
    Serial.println("[state] ESTOP -> OK (re-actuated)");
  } else if (state == STATE_LOST_LINK) {
    state = STATE_OK;
    Serial.println("[state] LOST_LINK -> OK (re-actuated)");
  }
}

// The one place every actuating route goes through: clear ESTOP, re-attach the
// emulated servo if it was detached, write the (already clamped) target.
void actuateTo(float deg) {
  clearEstopIfNeeded();
  if (!axis.attached()) axis.attach();
  axis.write(deg);
}

// ---------------------------------------------------------------------------
// Status LED (H-26): green=OK, yellow=LOST_LINK, red=ESTOP, blue=WiFi connecting,
// magenta=FAULT (EmuServo stall/pot fault or DRV8833 nFAULT)
// ---------------------------------------------------------------------------
void updateStatusLed() {
  static unsigned long lastUpdate = 0;
  static bool blinkOn = false;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();

  uint8_t r = 0, g = 0, b = 0;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  bool fault = (axis.state() == EmuServo::FAULT) || axis.driverFault();

  if (!wifiUp) {
    b = 255;  // connecting
  } else if (state == STATE_ESTOP) {
    r = 255;  // red
  } else if (fault) {
    r = 255; b = 255;  // magenta
  } else if (state == STATE_LOST_LINK) {
    r = 255; g = 200;  // yellow
  } else {
    g = 255;  // OK
  }

#if STATUS_LED_IS_RGB
  rgbLedWrite(STATUS_LED_PIN, r, g, b);
#else
  bool attention = !wifiUp || state != STATE_OK || fault;
  if (attention) {
    blinkOn = !blinkOn;
    digitalWrite(STATUS_LED_PIN, blinkOn ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
#endif
}

// ---------------------------------------------------------------------------
// WiFi (H-13): static IP, non-blocking reconnect in loop()
// ---------------------------------------------------------------------------
void wifiBeginOnce() {
  WiFi.mode(WIFI_STA);
  kSubnetMask.fromString(SUBNET_MASK);
  WiFi.config(kToolIp, kGatewayIp, kSubnetMask);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);  // modem sleep adds latency that fights the 200ms heartbeat cadence
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  Serial.printf("[wifi] connecting to %s, static ip %s\n", WIFI_SSID, kToolIp.toString().c_str());
}

void maintainWifi() {
  static unsigned long lastAttempt = 0;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastAttempt < 2000) return;
  lastAttempt = millis();
  Serial.println("[wifi] not connected, retrying begin()");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PSK);
}

// ---------------------------------------------------------------------------
// Route handlers
// ---------------------------------------------------------------------------
void handleStatus() {
  updateLinkState();
  Serial.println("[http] GET /status");

  JsonDocument doc;
  // Contract fields (§C.2) — names and enum byte-exact, never renamed/removed.
  doc["tool"] = TOOL_ID;
  doc["latched"] = measuredLatched();
  doc["lateral_mm"] = measuredLateralMM();
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_s"] = (unsigned long)(millis() / 1000);
  doc["lastSeq"] = lastSeq;
  doc["state"] = stateToString(state);
  // Additive fields (emulated servo diagnostics / calibration)
  doc["pos_deg"] = axis.read();
  doc["target_deg"] = axis.target();
  doc["pot_mv"] = axis.readMilliVolts();
  doc["pot_raw"] = axis.readRaw();
  doc["emu_state"] = axis.stateName();    // IDLE | MOVING | AT_TARGET | FAULT
  doc["fault"] = axis.faultName();        // none | stall | pot_range
  doc["driver_fault"] = axis.driverFault();  // DRV8833 nFAULT asserted (false if not wired)
  doc["attached"] = axis.attached();
  doc["duty"] = axis.lastDuty();          // signed, + = toward increasing degrees
  sendJson(200, doc);
}

void handleLatch() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /latch seq=%ld\n", seq);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  actuateTo((float)EMU_LATCH_CLOSED_DEG);

  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleRelease() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /release seq=%ld\n", seq);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  actuateTo((float)EMU_LATCH_OPEN_DEG);

  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleLateral() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>() || !in["mm"].is<float>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  float requestedMM = in["mm"].as<float>();
  Serial.printf("[http] POST /lateral seq=%ld mm=%.2f\n", seq, requestedMM);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["lateral_mm"] = lateralTargetMM;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  lateralTargetMM = clampMM(requestedMM);  // §C.2 Δ: out-of-range mm clamped, never rejected
  actuateTo(mmToDeg(lateralTargetMM));

  out["ok"] = true;
  out["seq"] = seq;
  out["lateral_mm"] = lateralTargetMM;
  sendJson(200, out);
}

// Additive: POST /pos {"seq":i,"deg":f} -> {"ok":true,"seq":i,"deg":f} (clamped to the
// soft window). Bench/calibration route; same seq dedup and ESTOP re-attach rules.
void handlePos() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>() || !in["deg"].is<float>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  float requestedDeg = in["deg"].as<float>();
  Serial.printf("[http] POST /pos seq=%ld deg=%.2f\n", seq, requestedDeg);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["deg"] = axis.target();
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  float deg = EmuServo::clampDeg(requestedDeg);
  actuateTo(deg);

  out["ok"] = true;
  out["seq"] = seq;
  out["deg"] = deg;
  sendJson(200, out);
}

void handleHeartbeat() {
  JsonDocument in;
  if (!parseBody(in) || !in["t"].is<long>()) { sendBadJson(); return; }
  Serial.println("[http] POST /heartbeat");

  lastHeartbeatMs = millis();
  if (state == STATE_LOST_LINK) {
    state = STATE_OK;
    Serial.println("[state] LOST_LINK -> OK (heartbeat resumed)");
  }

  JsonDocument out;
  out["ok"] = true;
  sendJson(200, out);
}

void handleEstop() {
  Serial.println("[http] POST /estop");

#if ESTOP_BEHAVIOUR == DETACH
  axis.detach();  // nSLEEP low + both IN low: driver asleep, motor coasts (limp)
  Serial.println("[state] -> ESTOP (axis detached)");
#else
  Serial.println("[state] -> ESTOP (axis held, HOLD behaviour)");
#endif
  state = STATE_ESTOP;

  JsonDocument out;
  out["ok"] = true;
  sendJson(200, out);
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) { delay(10); }
  Serial.println();
  Serial.printf("[boot] Subzero end effector %d firmware starting (DRV8833 + pot emulated servo)\n", TOOL_ID);

#if STATUS_LED_IS_RGB
  // rgbLedWrite handles pin setup internally
#else
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  // Emulated servo: pins configured, driver ASLEEP. Boot behaviour is a hardware TODO
  // (H-11): default BOOT_DETACHED = nothing moves until the first command.
  axis.begin();
#if EMU_BOOT_BEHAVIOUR == BOOT_OPEN
  axis.attach();
  axis.write((float)EMU_LATCH_OPEN_DEG);
  Serial.println("[boot] BOOT_OPEN: moving to EMU_LATCH_OPEN_DEG");
#elif EMU_BOOT_BEHAVIOUR == BOOT_HOLD
  axis.attach();  // hold current position
  Serial.println("[boot] BOOT_HOLD: holding current position");
#else
  Serial.println("[boot] BOOT_DETACHED: driver asleep until first command (calibrate now via /status)");
#endif
  lateralTargetMM = measuredLateralMM();

  wifiBeginOnce();
  unsigned long wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiWaitStart < 5000) {
    axis.update();  // keep the loop alive if BOOT_HOLD/BOOT_OPEN attached it
    updateStatusLed();
    delay(5);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected, ip=%s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[wifi] not yet connected, will keep retrying in loop()");
  }

  lastHeartbeatMs = millis();  // avoid an immediate LOST_LINK before the first heartbeat arrives

  server.on("/status", HTTP_GET, handleStatus);
  server.on("/latch", HTTP_POST, handleLatch);
  server.on("/release", HTTP_POST, handleRelease);
  server.on("/lateral", HTTP_POST, handleLateral);
  server.on("/pos", HTTP_POST, handlePos);  // additive
  server.on("/heartbeat", HTTP_POST, handleHeartbeat);
  server.on("/estop", HTTP_POST, handleEstop);
  server.onNotFound(sendNotFound);
  server.begin();
  Serial.printf("[http] server listening on port %d\n", HTTP_PORT);

  esp_task_wdt_config_t twdtConfig = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_err_t wdtErr = esp_task_wdt_init(&twdtConfig);
  if (wdtErr == ESP_ERR_INVALID_STATE) {
    // Arduino core may already have initialised the WDT; reconfigure instead.
    esp_task_wdt_reconfigure(&twdtConfig);
  }
  esp_task_wdt_add(NULL);
}

void loop() {
  server.handleClient();
  axis.update();  // self-paced at EMU_LOOP_HZ; keeps holding through LOST_LINK
  maintainWifi();
  updateLinkState();
  updateStatusLed();
  esp_task_wdt_reset();
}
