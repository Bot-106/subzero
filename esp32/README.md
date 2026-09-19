# Subzero tool-changer firmware (ESP32-S3-DevKitC-1)

Implements `docs/contracts.md` §C.2 (ESP32 HTTP API v0) byte-identically. One PlatformIO
project, one `platformio.ini` environment per physical tool board (`TOOL_ID` /
`TOOL_IP_LAST_OCTET` build flags pick the identity).

## Build / flash / monitor

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"

# tool 1 (default env, TOOL_ID=1, 10.13.60.31)
cd esp32
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1 -t upload
pio device monitor -b 115200   # ctrl-c to exit; look for /dev/cu.usbmodem* on macOS

# tool 2 (TOOL_ID=2, 10.13.60.32)
pio run -e tool2
pio run -e tool2 -t upload
```

Tool 3 (`10.13.60.33`): copy the `[env:tool2]` block in `platformio.ini`, rename it
`[env:tool3]`, set `-D TOOL_ID=3 -D TOOL_IP_LAST_OCTET=33`.

If upload hangs on "Connecting...": hold BOOT, tap RESET, release BOOT, retry.

## Bench test (acceptance test 4 — all six routes)

Point `curl` at the tool's static IP (`10.13.60.31` for tool 1). No auth, no TLS
(closed robot WiFi, `docs/contracts.md` §C.2 Δ).

```bash
curl http://10.13.60.31/status

curl -X POST http://10.13.60.31/latch \
  -H 'Content-Type: application/json' -d '{"seq":1}'

curl -X POST http://10.13.60.31/release \
  -H 'Content-Type: application/json' -d '{"seq":2}'

curl -X POST http://10.13.60.31/lateral \
  -H 'Content-Type: application/json' -d '{"seq":3,"mm":12.5}'

curl -X POST http://10.13.60.31/heartbeat \
  -H 'Content-Type: application/json' -d '{"t":1}'

curl -X POST http://10.13.60.31/estop
```

Expected: `/status` returns `{"tool":1,"latched":...,"lateral_mm":...,"rssi":...,"uptime_s":...,"lastSeq":...,"state":"OK"|"LOST_LINK"|"ESTOP"}`;
each POST returns `{"ok":true,"seq":i,...}`; `/estop` returns `{"ok":true}` and flips
`state` to `ESTOP` (both servos detach — H-11); the next `/latch`, `/release` or
`/lateral` re-attaches and clears `state` back to `OK`.

**Link-loss check:** stop sending `/heartbeat` for 1 s (the bridge's normal cadence is
every 200 ms) → `GET /status` shows `"state":"LOST_LINK"` and the tool holds its last
commanded positions (never detaches on link loss alone — only an explicit `/estop`
detaches, per H-11). Sending one more `/heartbeat` returns it to `"state":"OK"`.

**Malformed / duplicate seq:**
```bash
curl -X POST http://10.13.60.31/latch -d 'not json'          # -> 400 {"ok":false,"seq":0,"err":"bad_json"}
curl -X POST http://10.13.60.31/latch -d '{"seq":1}'          # repeat an already-seen seq
                                                               # -> 200 {"ok":true,"seq":1,"dup":true}, no re-actuation
curl -X POST http://10.13.60.31/lateral -d '{"seq":9,"mm":999}'
                                                               # -> 200 {"ok":true,"seq":9,"lateral_mm":40.0} (clamped, never rejected)
```

## Status LED (H-26)

Green = `OK`, yellow = `LOST_LINK`, red = `ESTOP`, blue = WiFi still connecting.
Default `STATUS_LED_PIN` is `RGB_BUILTIN`, which resolves to the on-board WS2812
addressable RGB LED — **this is correct for board revision v1.0 (silkscreen), which
wires it to GPIO48**. The Arduino core does not detect board revision at compile time
(confirmed: `RGB_BUILTIN` / `rgbLedWrite` compile fine either way, but the core has no
knowledge of which physical GPIO your board's LED is actually wired to).

**If your board's silkscreen says v1.1** (LED on GPIO38 instead of GPIO48), edit
`include/config.h`:
```cpp
#define STATUS_LED_PIN 38
```
(`rgbLedWrite(38, r, g, b)` still works — it's a plain GPIO number, not board-specific.)
If no RGB LED is fitted at all, wire a plain LED to GPIO7 and set:
```cpp
#define STATUS_LED_IS_RGB 0
#define STATUS_LED_PIN 7
```

## H-11 hardware question (open)

Software is decided: `POST /estop` **detaches** both servos (PWM off, goes limp),
`ESTOP_BEHAVIOUR` = `DETACH` in `include/config.h`. **Ask on hardware:** does a limp
latch servo drop a held tool under its own weight / vibration? If yes, flip
`ESTOP_BEHAVIOUR` to `HOLD` (servos stay attached and hold their last written angle on
e-stop instead of going limp) before running acceptance test 4 with a tool actually
loaded.

## Contract notes (docs/contracts.md §C.2, implemented byte-identically)

- Every response is `Content-Type: application/json`.
- Malformed/missing JSON body → HTTP 400 `{"ok":false,"seq":0,"err":"bad_json"}`.
- `mm` outside `[0, LATERAL_MAX_MM]` is clamped, never rejected; the clamped value is
  echoed back in the response.
- `seq <= lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` without
  re-actuating the servo.
- No authentication (same closed robot WiFi as the bridge).
- Any undefined route/method → HTTP 404 `{"ok":false,"seq":0,"err":"not_found"}`.
