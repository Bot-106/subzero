// Subzero tool-changer firmware — ESP32-S3-DevKitC-1
//
// Implements docs/contracts.md §C.2 (ESP32 HTTP API v0) byte-identically:
// GET /status, POST /latch /release /lateral /heartbeat /estop, 404 JSON otherwise.
//
// State machine:
//   OK          - normal operation
//   LOST_LINK   - no /heartbeat for > HEARTBEAT_TIMEOUT_MS; holds last targets, never detaches
//   ESTOP       - POST /estop detached both servos (H-11 ESTOP_BEHAVIOUR=DETACH);
//                 cleared by the next /latch, /release or /lateral (re-attach + write target)
//
// Sync WebServer per DECISION A-02 (not ESPAsyncWebServer) — one bridge client at a time.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WebServer server(HTTP_PORT);
Servo servoLatch;
Servo servoLateral;

enum ToolState { STATE_OK, STATE_LOST_LINK, STATE_ESTOP };
ToolState state = STATE_OK;

bool latched = false;              // current commanded latch position
float lateralMM = 0.0f;            // current commanded lateral position, mm
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
  if (mm < LATERAL_MIN_MM) return LATERAL_MIN_MM;
  if (mm > LATERAL_MAX_MM) return LATERAL_MAX_MM;
  return mm;
}

int mmToDeg(float mm) {
  float t = (mm - LATERAL_MIN_MM) / (LATERAL_MAX_MM - LATERAL_MIN_MM);
  float deg = LATERAL_MIN_DEG + t * (float)(LATERAL_MAX_DEG - LATERAL_MIN_DEG);
  return (int)lroundf(deg);
}

void attachLatchServo() {
  if (!servoLatch.attached()) {
    servoLatch.setPeriodHertz(50);
    servoLatch.attach(SERVO_LATCH_PIN, 500, 2500);
  }
}

void attachLateralServo() {
  if (!servoLateral.attached()) {
    servoLateral.setPeriodHertz(50);
    servoLateral.attach(SERVO_LATERAL_PIN, 500, 2500);
  }
}

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
// ESTOP (only an explicit /latch, /release or /lateral clears that) and never
// detaches on link loss alone (safety.md §5 — tool servos fail safe / hold).
void updateLinkState() {
  if (state == STATE_OK && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    state = STATE_LOST_LINK;
    Serial.println("[state] OK -> LOST_LINK (heartbeat timeout)");
  }
}

// Clears ESTOP back to OK when a /latch, /release or /lateral arrives. Does not
// touch LOST_LINK (still gated by heartbeat cadence); mirrors §C.2 semantics
// where re-actuation implicitly resumes control.
void clearEstopIfNeeded() {
  if (state == STATE_ESTOP) {
    state = STATE_OK;
    Serial.println("[state] ESTOP -> OK (re-actuated)");
  } else if (state == STATE_LOST_LINK) {
    state = STATE_OK;
    Serial.println("[state] LOST_LINK -> OK (re-actuated)");
  }
}

// ---------------------------------------------------------------------------
// Status LED (H-26): green=OK, yellow=LOST_LINK, red=ESTOP, blue=WiFi connecting
// ---------------------------------------------------------------------------
void updateStatusLed() {
  static unsigned long lastUpdate = 0;
  static bool blinkOn = false;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();

  uint8_t r = 0, g = 0, b = 0;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);

  if (!wifiUp) {
    b = 255;  // connecting
  } else {
    switch (state) {
      case STATE_OK: g = 255; break;
      case STATE_LOST_LINK: r = 255; g = 200; break;  // yellow
      case STATE_ESTOP: r = 255; break;                // red
    }
  }

#if STATUS_LED_IS_RGB
  rgbLedWrite(STATUS_LED_PIN, r, g, b);
#else
  bool attention = !wifiUp || state != STATE_OK;
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
  doc["tool"] = TOOL_ID;
  doc["latched"] = latched;
  doc["lateral_mm"] = lateralMM;
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_s"] = (unsigned long)(millis() / 1000);
  doc["lastSeq"] = lastSeq;
  doc["state"] = stateToString(state);
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
  clearEstopIfNeeded();
  attachLatchServo();
  latched = true;
  servoLatch.write(LATCH_CLOSED_DEG);

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
  clearEstopIfNeeded();
  attachLatchServo();
  latched = false;
  servoLatch.write(LATCH_OPEN_DEG);

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
    out["lateral_mm"] = lateralMM;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  clearEstopIfNeeded();
  lateralMM = clampMM(requestedMM);  // §C.2 Δ: out-of-range mm clamped, never rejected
  attachLateralServo();
  servoLateral.write(mmToDeg(lateralMM));

  out["ok"] = true;
  out["seq"] = seq;
  out["lateral_mm"] = lateralMM;
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
  servoLatch.detach();
  servoLateral.detach();
  Serial.println("[state] -> ESTOP (servos detached)");
#else
  Serial.println("[state] -> ESTOP (servos held, HOLD behaviour)");
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
  Serial.printf("[boot] Subzero tool %d firmware starting\n", TOOL_ID);

#if STATUS_LED_IS_RGB
  // rgbLedWrite handles pin setup internally
#else
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif
  pinMode(LATCH_SWITCH_PIN, INPUT_PULLUP);

  // Known-safe boot pose: latch open, lateral retracted. Servos are attached and
  // written once so the physical tool starts in a defined position.
  attachLatchServo();
  servoLatch.write(LATCH_OPEN_DEG);
  latched = false;

  attachLateralServo();
  servoLateral.write(mmToDeg(0.0f));
  lateralMM = 0.0f;

  wifiBeginOnce();
  unsigned long wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiWaitStart < 5000) {
    updateStatusLed();
    delay(50);
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
  maintainWifi();
  updateLinkState();
  updateStatusLed();
  esp_task_wdt_reset();
}
