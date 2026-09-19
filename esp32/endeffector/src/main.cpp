// Subzero END-EFFECTOR firmware — ESP32-S3-DevKitC-1 (one board per swappable end effector)
//
// Implements docs/contracts.md §C.2 (ESP32 HTTP API v0) byte-identically:
// GET /status, POST /latch /release /lateral /heartbeat /estop, 404 JSON otherwise.
// Additive (allowed by §C.2 "additive fields ok"): extra /status fields and POST /run.
//
// Actuator: ONE brushed DC motor on a DRV8833, OPEN LOOP (no feedback of any kind).
// Every actuate is a timed run on that motor (TimedMotor):
//   /latch            -> run "fwd" for LATCH_RUN_MS at LATCH_SPEED
//   /release          -> run "rev" for RELEASE_RUN_MS at RELEASE_SPEED
//   /lateral {"mm"}   -> mm clamped to [0, LATERAL_MAX_MM]; dead-reckoned: run toward the
//                        request for |mm - est_mm| * LATERAL_MS_PER_MM ms at LATERAL_SPEED,
//                        est_mm := mm when the run STARTS (caveat: an interrupted run leaves
//                        est_mm ahead of the mechanism until the next full-stroke /latch or
//                        /release resets the human's mental model — there is no feedback).
//   /run  (additive)  -> {"dir":"fwd"|"rev","ms":i,"speed":f} raw timed run: THE bench tool.
// The contract's seq is acknowledged when the run STARTS (no feedback, same as hobby servos);
// /status "running" / "remaining_ms" tell you when it ends. A new actuate while a run is in
// progress REPLACES it (new direction, new deadline). /status "latched" = the last COMPLETED
// /latch or /release run was /latch (persists; /lateral and /run leave it alone).
//
// State machine (contract enum, byte-exact):
//   OK          - normal operation
//   LOST_LINK   - no /heartbeat for > HEARTBEAT_TIMEOUT_MS. Open-loop "hold" = STOP: any run in
//                 progress is cancelled with a brake, new runs are refused (503 "lost_link",
//                 lastSeq not advanced) until a /heartbeat returns (safety.md §5).
//   ESTOP       - POST /estop: stop + nSLEEP low, motor coasts (H-11 DETACH); cleared by the
//                 next /latch, /release, /lateral or /run (wake + run).
// Every run has a deadline <= RUN_MAX_MS regardless of state: the motor can never be left on.
//
// Sync WebServer per DECISION A-02 (not ESPAsyncWebServer) — one bridge client at a time.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "timed_motor.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WebServer server(HTTP_PORT);
TimedMotor motor;

enum ToolState { STATE_OK, STATE_LOST_LINK, STATE_ESTOP };
ToolState state = STATE_OK;

enum RunKind { RUN_LATCH, RUN_RELEASE, RUN_LATERAL, RUN_RAW };
enum PendingLatch { PENDING_NONE, PENDING_TRUE, PENDING_FALSE };

bool latched = false;                     // contract: last COMPLETED latch/release run was /latch
PendingLatch pendingLatch = PENDING_NONE; // committed to `latched` when the run reaches its deadline
float estMM = 0.0f;                       // dead-reckoned lateral position (0 at boot), [0, LATERAL_MAX_MM]
long lastSeq = 0;                         // highest seq acknowledged (dedup per §C.2 Δ)
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

// Additive: a run refused because the link is down. lastSeq is NOT advanced, so the bridge's
// retry (3 x 100 ms) succeeds as soon as its heartbeat has landed.
void sendLostLink(long seq) {
  Serial.printf("[http] 503 lost_link (seq=%ld refused: no heartbeat for %lu ms)\n", seq,
                (unsigned long)(millis() - lastHeartbeatMs));
  JsonDocument out;
  out["ok"] = false;
  out["seq"] = seq;
  out["err"] = "lost_link";
  sendJson(503, out);
}

void sendDup(long seq) {
  JsonDocument out;
  out["ok"] = true;
  out["seq"] = seq;
  out["dup"] = true;
  sendJson(200, out);
}

// Parses server.arg("plain") into doc. Returns false (and has NOT sent a response)
// if the body is missing or not valid JSON.
bool parseBody(JsonDocument &doc) {
  if (!server.hasArg("plain") || server.arg("plain").length() == 0) return false;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  return !err;
}

bool linkLost() { return (millis() - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS; }

// Advances OK -> LOST_LINK when the heartbeat has been silent too long, and applies the
// open-loop hold-equivalent: STOP the motor (brake) and cancel any run. Never touches ESTOP
// (the motor is already asleep there; only an explicit actuate clears ESTOP).
void updateLinkState() {
  if (state == STATE_OK && linkLost()) {
    state = STATE_LOST_LINK;
    pendingLatch = PENDING_NONE;  // an interrupted latch/release commits nothing
#if LINK_LOSS_BEHAVIOUR == DETACH
    motor.sleep();
    Serial.println("[state] OK -> LOST_LINK (heartbeat timeout; motor detached)");
#else
    motor.stop();  // brake, driver stays awake; new runs refused until a heartbeat returns
    Serial.println("[state] OK -> LOST_LINK (heartbeat timeout; motor STOPPED, runs refused)");
#endif
  }
}

// The one place every actuating route goes through. Caller has already checked dedup and
// linkLost(). Clears ESTOP (wake happens inside motor.run()), records the latch intent,
// starts the (clamped) timed run — replacing any run in progress.
void startRun(RunKind kind, TimedMotor::Direction dir, uint32_t ms, float speed) {
  if (state == STATE_ESTOP) {
    state = STATE_OK;
    Serial.println("[state] ESTOP -> OK (re-actuated, driver woken)");
  }
  switch (kind) {
    case RUN_LATCH:   pendingLatch = PENDING_TRUE;  break;
    case RUN_RELEASE: pendingLatch = PENDING_FALSE; break;
    default:          pendingLatch = PENDING_NONE;  break;  // lateral / raw runs leave `latched` alone
  }
  motor.run(dir, ms, speed);
}

// Called once when a run reaches its deadline (natural completion only, see loop()).
void onRunEnded() {
  if (pendingLatch == PENDING_TRUE) { latched = true; Serial.println("[state] latched = true"); }
  else if (pendingLatch == PENDING_FALSE) { latched = false; Serial.println("[state] latched = false"); }
  pendingLatch = PENDING_NONE;
}

// ---------------------------------------------------------------------------
// Status LED (H-26): green=OK, cyan=OK+running, yellow=LOST_LINK, red=ESTOP,
// blue=WiFi connecting, magenta=DRV8833 nFAULT asserted
// ---------------------------------------------------------------------------
void updateStatusLed() {
  static unsigned long lastUpdate = 0;
  static bool blinkOn = false;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();

  uint8_t r = 0, g = 0, b = 0;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  bool fault = motor.driverFault();

  if (!wifiUp) {
    b = 255;  // connecting
  } else if (state == STATE_ESTOP) {
    r = 255;  // red
  } else if (fault) {
    r = 255; b = 255;  // magenta
  } else if (state == STATE_LOST_LINK) {
    r = 255; g = 200;  // yellow
  } else if (motor.running()) {
    g = 255; b = 255;  // cyan: motor being driven
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
  doc["latched"] = latched;
  doc["lateral_mm"] = estMM;
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_s"] = (unsigned long)(millis() / 1000);
  doc["lastSeq"] = lastSeq;
  doc["state"] = stateToString(state);
  // Additive fields (open-loop timed-run diagnostics)
  doc["running"] = motor.running();
  doc["remaining_ms"] = motor.remainingMs();
  doc["dir"] = motor.lastDirName();       // "fwd" | "rev" — direction of the last/current run
  doc["speed"] = motor.lastSpeed();       // 0..1 as clamped and applied to the last/current run
  doc["driver_fault"] = motor.driverFault();  // DRV8833 nFAULT asserted (false if not wired)
  doc["est_mm"] = estMM;                  // dead-reckoned lateral estimate (same as lateral_mm)
  doc["awake"] = motor.awake();           // nSLEEP high (false after /estop and at boot)
  sendJson(200, doc);
}

void handleLatch() {
  updateLinkState();
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /latch seq=%ld\n", seq);

  if (seq <= lastSeq) { sendDup(seq); return; }
  if (linkLost()) { sendLostLink(seq); return; }

  lastSeq = seq;
  startRun(RUN_LATCH, TimedMotor::FWD, LATCH_RUN_MS, LATCH_SPEED);

  JsonDocument out;
  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleRelease() {
  updateLinkState();
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  Serial.printf("[http] POST /release seq=%ld\n", seq);

  if (seq <= lastSeq) { sendDup(seq); return; }
  if (linkLost()) { sendLostLink(seq); return; }

  lastSeq = seq;
  startRun(RUN_RELEASE, TimedMotor::REV, RELEASE_RUN_MS, RELEASE_SPEED);

  JsonDocument out;
  out["ok"] = true;
  out["seq"] = seq;
  sendJson(200, out);
}

void handleLateral() {
  updateLinkState();
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>() || !in["mm"].is<float>()) { sendBadJson(); return; }
  long seq = in["seq"].as<long>();
  float requestedMM = in["mm"].as<float>();
  Serial.printf("[http] POST /lateral seq=%ld mm=%.2f\n", seq, requestedMM);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["lateral_mm"] = estMM;
    out["dup"] = true;
    sendJson(200, out);
    return;
  }
  if (linkLost()) { sendLostLink(seq); return; }

  lastSeq = seq;
  float targetMM = clampMM(requestedMM);  // §C.2 Δ: out-of-range mm clamped, never rejected
  float deltaMM = targetMM - estMM;
  uint32_t ms = (uint32_t)lroundf(fabsf(deltaMM) * LATERAL_MS_PER_MM);
  Serial.printf("[lateral] est %.2f -> %.2f mm: %s %lu ms\n", estMM, targetMM, deltaMM >= 0 ? "fwd" : "rev",
                (unsigned long)ms);
  estMM = targetMM;  // dead reckoning: assume the run completes (see header caveat)
  if (ms > 0) startRun(RUN_LATERAL, deltaMM > 0 ? TimedMotor::FWD : TimedMotor::REV, ms, LATERAL_SPEED);
  else if (state == STATE_ESTOP) startRun(RUN_LATERAL, TimedMotor::FWD, 0, 0.0f);  // no motion, still clears ESTOP

  out["ok"] = true;
  out["seq"] = seq;
  out["lateral_mm"] = targetMM;
  sendJson(200, out);
}

// Additive: POST /run {"seq":i,"dir":"fwd"|"rev","ms":i,"speed":f}
//   -> {"ok":true,"seq":i,"dir":"fwd","ms":i,"speed":f}  (ms clamped to [0, RUN_MAX_MS],
//      speed clamped to [0, RUN_MAX_SPEED]; the clamped values are echoed).
// Bench/calibration route; same seq dedup, link-loss refusal and ESTOP re-wake rules.
void handleRun() {
  updateLinkState();
  JsonDocument in;
  if (!parseBody(in) || !in["seq"].is<long>() || !in["dir"].is<const char *>() || !in["ms"].is<long>() ||
      !in["speed"].is<float>()) {
    sendBadJson();
    return;
  }
  long seq = in["seq"].as<long>();
  const char *dirStr = in["dir"].as<const char *>();
  long msReq = in["ms"].as<long>();
  float speedReq = in["speed"].as<float>();
  TimedMotor::Direction dir;
  if (strcmp(dirStr, "fwd") == 0) dir = TimedMotor::FWD;
  else if (strcmp(dirStr, "rev") == 0) dir = TimedMotor::REV;
  else { sendBadJson(); return; }
  Serial.printf("[http] POST /run seq=%ld dir=%s ms=%ld speed=%.2f\n", seq, dirStr, msReq, speedReq);

  JsonDocument out;
  if (seq <= lastSeq) {
    out["ok"] = true;
    out["seq"] = seq;
    out["dir"] = motor.lastDirName();
    out["ms"] = motor.lastMs();
    out["speed"] = motor.lastSpeed();
    out["dup"] = true;
    sendJson(200, out);
    return;
  }
  if (linkLost()) { sendLostLink(seq); return; }

  lastSeq = seq;
  uint32_t ms = TimedMotor::clampMs(msReq < 0 ? 0u : (uint32_t)msReq);
  float speed = TimedMotor::clampSpeed(speedReq);
  startRun(RUN_RAW, dir, ms, speed);

  out["ok"] = true;
  out["seq"] = seq;
  out["dir"] = TimedMotor::dirName(dir);
  out["ms"] = ms;
  out["speed"] = speed;
  sendJson(200, out);
}

void handleHeartbeat() {
  JsonDocument in;
  if (!parseBody(in) || !in["t"].is<long>()) { sendBadJson(); return; }
  Serial.println("[http] POST /heartbeat");

  lastHeartbeatMs = millis();
  if (state == STATE_LOST_LINK) {
    state = STATE_OK;
    Serial.println("[state] LOST_LINK -> OK (heartbeat resumed; motor stays stopped until commanded)");
  }

  JsonDocument out;
  out["ok"] = true;
  sendJson(200, out);
}

void handleEstop() {
  Serial.println("[http] POST /estop");
  pendingLatch = PENDING_NONE;  // an interrupted latch/release commits nothing
#if ESTOP_BEHAVIOUR == DETACH
  motor.sleep();  // stop + nSLEEP low: driver asleep, outputs Hi-Z, motor coasts (limp)
  Serial.println("[state] -> ESTOP (motor detached)");
#else
  motor.stop();   // brake, driver stays awake (HOLD behaviour)
  Serial.println("[state] -> ESTOP (motor braked, HOLD behaviour)");
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
  Serial.printf("[boot] Subzero end effector %d firmware starting (DRV8833 open-loop timed motor)\n", TOOL_ID);

#if STATUS_LED_IS_RGB
  // rgbLedWrite handles pin setup internally
#else
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  // Motor: pins configured, driver ASLEEP. Nothing moves until the first command.
  motor.begin();
  Serial.println("[boot] driver asleep until first command; est_mm=0, latched=false assumed");

  wifiBeginOnce();
  unsigned long wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiWaitStart < 5000) {
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
  server.on("/run", HTTP_POST, handleRun);  // additive
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
  // Run deadline: the only stop that can happen between these two lines is update()'s
  // natural deadline stop (estop / link-loss stops happen elsewhere), so this detects
  // completed runs only.
  bool wasRunning = motor.running();
  motor.update();
  if (wasRunning && !motor.running()) onRunEnded();
  maintainWifi();
  updateLinkState();
  updateStatusLed();
  esp_task_wdt_reset();
}
