// Subzero pincher firmware — ESP32-S3-DevKitC-1
//
// Implements docs/contracts.md §C.2 (ESP32 HTTP API v0) byte-identically, re-mapped onto
// two mirrored jaw servos: GET /status, POST /latch /release /lateral /heartbeat /estop,
// 404 JSON otherwise.
//
//   /latch    -> close both jaws onto the tool (pinch), gap -> 0mm
//   /release  -> open both jaws, gap -> JAW_MAX_MM
//   /lateral  -> set the jaw gap directly, {"seq":i,"mm":f}, clamped to [0, JAW_MAX_MM]
//   /status   -> contract fields (latched/lateral_mm/...) + additive jaw_a_deg/jaw_b_deg/moving
//
// State machine:
//   OK          - normal operation
//   LOST_LINK   - no /heartbeat for > HEARTBEAT_TIMEOUT_MS; HOLDs last targets, never detaches
//   ESTOP       - POST /estop; ESTOP_BEHAVIOUR=HOLD (default, see config.h/README.md): both
//                 servos stay attached and freeze at their current (already-slewed) angle —
//                 this board may be holding a tool over people's feet, so it must not drop it.
//                 ESTOP_BEHAVIOUR=DETACH is the selectable alternative (servos go limp).
//                 Either way, cleared by the next /latch, /release or /lateral.
//
// Motion quality: jaw angles are approached via a non-blocking slew limiter
// (JAW_SLEW_DEG_PER_S, config.h) driven by millis() in loop()'s updateJaws() — never slam
// the jaws shut on a tool. A seq is marked done (lastSeq) as soon as the target is SET, per
// contract ("no position feedback on hobby servos"), but `moving` keeps reporting true in
// /status until the slew finishes so the bridge/humans can see the jaws are still closing.
//
// Sync WebServer per decision A-02 (not ESPAsyncWebServer) — one bridge client at a time.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <math.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WebServer server(HTTP_PORT);
Servo servoA;
Servo servoB;

enum ToolState { STATE_OK, STATE_LOST_LINK, STATE_ESTOP };
ToolState state = STATE_OK;

float targetGapMM = 0.0f;      // current commanded jaw gap, mm (0 = closed) — contract "lateral_mm"
float targetAngleA = 0.0f;     // where servo A is being slewed toward, deg
float targetAngleB = 0.0f;     // where servo B is being slewed toward, deg
float currentAngleA = 0.0f;    // servo A's slewed (actually written) angle, deg
float currentAngleB = 0.0f;    // servo B's slewed (actually written) angle, deg
bool moving = false;           // true while either jaw is still slewing toward its target

long lastSeq = 0;                  // highest seq acknowledged (dedup per §C.2 Δ)
unsigned long lastHeartbeatMs = 0;

const IPAddress kToolIp(TOOL_IP_OCTET1, TOOL_IP_OCTET2, TOOL_IP_OCTET3, TOOL_IP_LAST_OCTET);
const IPAddress kGatewayIp(TOOL_IP_OCTET1, TOOL_IP_OCTET2, TOOL_IP_OCTET3, GATEWAY_IP_OCTET4);
IPAddress kSubnetMask;

static const float kAngleEpsilonDeg = 0.5f;  // below this, consider a jaw "arrived"
static const float kGapEpsilonMM = 0.01f;    // below this, consider the gap "closed"

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
  if (mm < JAW_MIN_MM) return JAW_MIN_MM;
  if (mm > JAW_MAX_MM) return JAW_MAX_MM;
  return mm;
}

// mm -> deg linear map, per servo's own OPEN/CLOSED pair. The "mirroring" between
// servo A and servo B lives entirely in the OPEN/CLOSED constants (config.h H-10),
// not in extra branches here: A's angle decreases as the gap opens (in this
// board's default placeholders) while B's increases, because that's what mounting
// them opposite each other to pinch symmetrically requires.
float mmToAngleA(float mm) {
  float t = (mm - JAW_MIN_MM) / (JAW_MAX_MM - JAW_MIN_MM);
  return JAW_A_CLOSED_DEG + t * (float)(JAW_A_OPEN_DEG - JAW_A_CLOSED_DEG);
}

float mmToAngleB(float mm) {
  float t = (mm - JAW_MIN_MM) / (JAW_MAX_MM - JAW_MIN_MM);
  return JAW_B_CLOSED_DEG + t * (float)(JAW_B_OPEN_DEG - JAW_B_CLOSED_DEG);
}

void attachJaws() {
  if (!servoA.attached()) {
    servoA.setPeriodHertz(50);
    servoA.attach(SERVO_A_PIN, SERVO_MIN_US, SERVO_MAX_US);
  }
  if (!servoB.attached()) {
    servoB.setPeriodHertz(50);
    servoB.attach(SERVO_B_PIN, SERVO_MIN_US, SERVO_MAX_US);
  }
}

// Sets the jaw gap target (mm, already clamped) and recomputes both servos'
// target angles from it. Does NOT touch currentAngle* — the slew limiter in
// updateJaws() carries the jaws there smoothly. Per contract, the seq is
// considered "done" the instant this returns (no position feedback exists).
void setGapTargetMM(float clampedMM) {
  targetGapMM = clampedMM;
  targetAngleA = mmToAngleA(clampedMM);
  targetAngleB = mmToAngleB(clampedMM);
}

bool jawsLatched() {
  return targetGapMM <= (JAW_MIN_MM + kGapEpsilonMM);
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
// detaches on link loss alone (safety.md §5 / LINK_LOSS_BEHAVIOUR — tool servos
// fail safe / hold, and this board holds even harder: it may be over someone's feet).
void updateLinkState() {
  if (state == STATE_OK && (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    state = STATE_LOST_LINK;
    Serial.println("[state] OK -> LOST_LINK (heartbeat timeout, jaws HOLD)");
  }
}

// Clears ESTOP/LOST_LINK back to OK when a /latch, /release or /lateral arrives,
// re-attaching the servos first if ESTOP_BEHAVIOUR had detached them.
void clearEstopIfNeeded() {
  if (state == STATE_ESTOP) {
    attachJaws();
    state = STATE_OK;
    Serial.println("[state] ESTOP -> OK (re-actuated)");
  } else if (state == STATE_LOST_LINK) {
    state = STATE_OK;
    Serial.println("[state] LOST_LINK -> OK (re-actuated)");
  }
}

// ---------------------------------------------------------------------------
// Slew limiter (motion quality — never slam the jaws). Non-blocking, driven by
// millis(); throttled to ~50Hz (matches the 50Hz servo signal period).
// ---------------------------------------------------------------------------
bool stepToward(float &current, float target, float maxStepDeg) {
  float diff = target - current;
  if (fabsf(diff) <= 0.01f) {
    current = target;
    return false;
  }
  float step = diff;
  if (step > maxStepDeg) step = maxStepDeg;
  if (step < -maxStepDeg) step = -maxStepDeg;
  current += step;
  return true;
}

void updateJaws() {
  static unsigned long lastUpdateMs = 0;
  unsigned long now = millis();
  if (lastUpdateMs == 0) lastUpdateMs = now;
  unsigned long elapsedMs = now - lastUpdateMs;
  if (elapsedMs < 20) return;  // ~50Hz, matches the servo PWM period
  lastUpdateMs = now;

  float maxStepDeg = JAW_SLEW_DEG_PER_S * (elapsedMs / 1000.0f);

  stepToward(currentAngleA, targetAngleA, maxStepDeg);
  stepToward(currentAngleB, targetAngleB, maxStepDeg);

  if (servoA.attached()) servoA.write((int)lroundf(currentAngleA));
  if (servoB.attached()) servoB.write((int)lroundf(currentAngleB));

  moving = (fabsf(currentAngleA - targetAngleA) > kAngleEpsilonDeg) ||
           (fabsf(currentAngleB - targetAngleB) > kAngleEpsilonDeg);
}

// ---------------------------------------------------------------------------
// Status LED (H-26): green=OK, yellow=LOST_LINK, red=ESTOP, blue=WiFi
// connecting, white=moving (OK state, jaws still slewing).
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
      case STATE_ESTOP: r = 255; break;                // red
      case STATE_LOST_LINK: r = 255; g = 200; break;    // yellow
      case STATE_OK:
        if (moving) {
          r = 255; g = 255; b = 255;  // white — jaws slewing
        } else {
          g = 255;  // green — idle, holding
        }
        break;
    }
  }

#if STATUS_LED_IS_RGB
  rgbLedWrite(STATUS_LED_PIN, r, g, b);
#else
  bool attention = !wifiUp || state != STATE_OK || moving;
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
  doc["latched"] = jawsLatched();
  doc["lateral_mm"] = targetGapMM;
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_s"] = (unsigned long)(millis() / 1000);
  doc["lastSeq"] = lastSeq;
  doc["state"] = stateToString(state);
  // Δ additive fields for this board (nothing renamed/removed from the contract):
  doc["jaw_a_deg"] = currentAngleA;
  doc["jaw_b_deg"] = currentAngleB;
  doc["moving"] = moving;
  sendJson(200, doc);
}

void handleLatch() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /latch seq=%ld (pinch: jaws -> closed)\n", seq);

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
  attachJaws();
  setGapTargetMM(JAW_MIN_MM);  // pinch closed

  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleRelease() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /release seq=%ld (jaws -> open)\n", seq);

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
  attachJaws();
  setGapTargetMM(JAW_MAX_MM);  // fully open

  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleLateral() {
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>() || !in["mm"].is<float>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  float requestedMM = in["mm"].as<float>();
  Serial.printf("[http] POST /lateral seq=%ld mm=%.2f (jaw gap)\n", seq, requestedMM);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["lateral_mm"] = targetGapMM;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }

  lastSeq = seq;
  clearEstopIfNeeded();
  attachJaws();
  float clamped = clampMM(requestedMM);  // §C.2 Δ: out-of-range mm clamped, never rejected
  setGapTargetMM(clamped);

  out["ok"] = true;
  out["seq"] = seq;
  out["lateral_mm"] = targetGapMM;
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

#if ESTOP_BEHAVIOUR == ESTOP_HOLD
  // Deliberate choice for this board (README.md): the pincher may be holding a
  // tool over people's feet. Freeze exactly where the jaws already are — do
  // NOT change the target (that would resume slewing) and stay attached so
  // the servos keep supplying holding torque.
  targetAngleA = currentAngleA;
  targetAngleB = currentAngleB;
  moving = false;
  Serial.println("[state] -> ESTOP (jaws HELD at current angle, servos stay attached)");
#else
  servoA.detach();
  servoB.detach();
  moving = false;
  Serial.println("[state] -> ESTOP (servos detached, DETACH behaviour)");
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
  Serial.printf("[boot] Subzero pincher (tool %d) firmware starting\n", TOOL_ID);
  Serial.printf("[boot] ESTOP_BEHAVIOUR=%s (deliberate choice, see README.md)\n",
                (ESTOP_BEHAVIOUR == ESTOP_HOLD) ? "HOLD" : "DETACH");

#if STATUS_LED_IS_RGB
  // rgbLedWrite handles pin setup internally
#else
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  // Known-safe boot pose: jaws open (not pinching anything by default). Servos
  // are attached and written once so the physical jaws start in a defined
  // position; currentAngle* start there too, so there's no phantom slew at boot.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  attachJaws();
  setGapTargetMM(JAW_MAX_MM);
  currentAngleA = targetAngleA;
  currentAngleB = targetAngleB;
  servoA.write((int)lroundf(currentAngleA));
  servoB.write((int)lroundf(currentAngleB));

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
  updateJaws();
  updateStatusLed();
  esp_task_wdt_reset();
}
