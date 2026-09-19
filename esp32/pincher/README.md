# Subzero pincher firmware (ESP32-S3-DevKitC-1)

Tool id **0**, static IP **10.13.60.30**. Implements `docs/contracts.md` §C.2 (ESP32 HTTP
API v0) byte-identically, with the six contract routes re-mapped onto **two mirrored jaw
servos** instead of a latch + lateral slide:

| Route | Pincher semantics |
|---|---|
| `POST /latch` | close both jaws onto the tool — **pinch** (`lateral_mm` -> 0) |
| `POST /release` | open both jaws (`lateral_mm` -> `JAW_MAX_MM`) |
| `POST /lateral {"seq","mm"}` | set the **jaw gap** directly, in mm, clamped to `[0, JAW_MAX_MM]` |
| `GET /status` | contract fields (`latched` = jaws at the closed target, `lateral_mm` = current commanded gap) **+** additive `jaw_a_deg`, `jaw_b_deg`, `moving` |
| `POST /heartbeat` | unchanged |
| `POST /estop` | unchanged shape, **different behaviour** — see "ESTOP = HOLD" below |

Nothing from the contract was renamed or removed; `jaw_a_deg`/`jaw_b_deg`/`moving` are
additive.

## Wiring

- **Signal:** `SERVO_A_PIN` = GPIO4, `SERVO_B_PIN` = GPIO5 (`include/config.h`, H-12) —
  both safe RTC_GPIO/ADC1 pins on the DevKitC-1, unaffected by strapping/USB/UART. Signal
  is 3.3 V logic, which SG90/MG90S-class micro-servos accept fine.
- **Power:** run both servos from a **5–6 V rail with a common ground**, sized for two
  stalled micro-servos simultaneously (figure ~2–3 A/servo stalled for an SG90/MG90S-class
  servo — confirm against your actual servo's datasheet). **Do NOT power the servos from
  the DevKitC-1's 3V3 pin** — it cannot source enough current and will brown out the board.
  Tie the servo rail's ground to the ESP32's ground (common ground is required for the PWM
  signal to be read correctly).
- No latch-confirm switch on this board (unlike the legacy reference firmware) — the
  pincher has no such input; jaw position is command-only (no feedback), per contract.

## Build / flash / monitor

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"

cd esp32/pincher
pio run -e pincher
pio run -e pincher -t upload
pio device monitor -b 115200   # ctrl-c to exit; look for /dev/cu.usbmodem* on macOS
```

If upload hangs on "Connecting...": hold BOOT, tap RESET, release BOOT, retry.

## Jaw angle calibration (do this before pinching a real tool)

The angles in `include/config.h` (`JAW_A_OPEN_DEG`, `JAW_A_CLOSED_DEG`, `JAW_B_OPEN_DEG`,
`JAW_B_CLOSED_DEG`, `JAW_MAX_MM`) are **placeholders** (`TODO(hardware)` H-10) — they are
guesses, not measured values. Find the real ones on the bench, without a tool loaded
first, then with one:

1. Flash and connect (see above). Confirm `/status` responds and jaws are at rest (boot
   pose = fully open).
2. Sweep the gap in small steps with `/lateral`, watching the physical jaws (start with no
   tool between them):
   ```bash
   curl -X POST http://10.13.60.30/lateral -H 'Content-Type: application/json' -d '{"seq":1,"mm":40}'   # should read as fully open
   curl -X POST http://10.13.60.30/lateral -H 'Content-Type: application/json' -d '{"seq":2,"mm":30}'
   curl -X POST http://10.13.60.30/lateral -H 'Content-Type: application/json' -d '{"seq":3,"mm":20}'
   curl -X POST http://10.13.60.30/lateral -H 'Content-Type: application/json' -d '{"seq":4,"mm":10}'
   curl -X POST http://10.13.60.30/lateral -H 'Content-Type: application/json' -d '{"seq":5,"mm":0}'    # should read as fully closed
   ```
   After each step, `GET /status` and read `jaw_a_deg`/`jaw_b_deg` to see the actual
   commanded angle for that gap.
3. **Find CLOSED without a tool first**: increase `mm` from 0 in small steps until the jaws
   just meet (or reach the desired minimum gap) without the servo stalling against a hard
   mechanical stop — a stalled hobby servo draws high current and can overheat or strip
   gears. Record that `mm`/angle pair as the true closed position.
4. **Find OPEN**: the largest gap the mechanism can physically reach — record that
   `mm`/angle pair.
5. **Then, with an actual tool in the jaws**: bring `mm` down slowly from open toward your
   candidate closed value and stop as soon as the pinch feels secure (won't slip when you
   tug the tool) — do NOT drive all the way to the no-tool closed angle, or you risk
   crushing the tool or stalling the servo against it. Update `JAW_A_CLOSED_DEG` /
   `JAW_B_CLOSED_DEG` / `JAW_MAX_MM` in `config.h` to match, re-flash, and re-verify.
6. Re-run the sweep once more end-to-end to confirm `/latch` and `/release` land where you
   expect.

## Bench test (all six routes)

No auth, no TLS (closed robot WiFi, `docs/contracts.md` §C.2 Δ).

```bash
curl http://10.13.60.30/status

curl -X POST http://10.13.60.30/latch \
  -H 'Content-Type: application/json' -d '{"seq":1}'

curl -X POST http://10.13.60.30/release \
  -H 'Content-Type: application/json' -d '{"seq":2}'

curl -X POST http://10.13.60.30/lateral \
  -H 'Content-Type: application/json' -d '{"seq":3,"mm":12.5}'

curl -X POST http://10.13.60.30/heartbeat \
  -H 'Content-Type: application/json' -d '{"t":1}'

curl -X POST http://10.13.60.30/estop
```

Expected: `/status` returns
`{"tool":0,"latched":...,"lateral_mm":...,"rssi":...,"uptime_s":...,"lastSeq":...,"state":"OK"|"LOST_LINK"|"ESTOP","jaw_a_deg":...,"jaw_b_deg":...,"moving":...}`;
each POST returns `{"ok":true,"seq":i,...}`; `/estop` returns `{"ok":true}` and flips
`state` to `ESTOP` — **jaws HOLD in place** (default `ESTOP_BEHAVIOUR`, see below), not
detach; the next `/latch`, `/release` or `/lateral` clears `state` back to `OK`.

**Link-loss check:** stop sending `/heartbeat` for 1 s (the bridge's normal cadence is
every 200 ms) → `GET /status` shows `"state":"LOST_LINK"` and the jaws hold their last
commanded gap (never detach on link loss alone). Sending one more `/heartbeat` returns it
to `"state":"OK"`.

**Malformed / duplicate seq / clamp:**
```bash
curl -X POST http://10.13.60.30/latch -d 'not json'          # -> 400 {"ok":false,"seq":0,"err":"bad_json"}
curl -X POST http://10.13.60.30/latch -d '{"seq":1}'          # repeat an already-seen seq
                                                               # -> 200 {"ok":true,"seq":1,"dup":true}, no re-actuation
curl -X POST http://10.13.60.30/lateral -d '{"seq":9,"mm":999}'
                                                               # -> 200 {"ok":true,"seq":9,"lateral_mm":40.0} (clamped, never rejected)
```

## ESTOP = HOLD (deliberate choice for this board)

`include/config.h` sets `ESTOP_BEHAVIOUR = ESTOP_HOLD` by default: on `POST /estop`, both
jaw servos **stay attached and freeze at their current (already-slewed) angle**; nothing
moves; `state` becomes `"ESTOP"` until the next `/latch`, `/release` or `/lateral`.

This is **different from the end-effector boards**, which `DETACH` (go limp) on `/estop`
per gate decision H-11 in `docs/contracts.md` §C.2. The reason for the difference: the
pincher may be **holding a tool clamped between its jaws, in the air, over people's feet**.
Going limp on e-stop would let gravity/momentum open a spring-loaded or gravity-biased
jaw and drop whatever is pinched — exactly the failure mode `safety.md §5` ("tool servos
fail safe: on heartbeat loss the tool holds, does not release a held object") warns
against. `HOLD` keeps the servos energized and holding torque so a pinched tool stays
pinched through an e-stop.

Link loss (1000 ms without `/heartbeat`) is **always** `HOLD` regardless of
`ESTOP_BEHAVIOUR` — that part is fixed by contract, not a per-board choice.

**To switch to DETACH** (e.g. if hardware testing shows HOLD causes a stalled/overheating
servo, or the mechanical design makes a limp jaw safe): edit `include/config.h`:
```cpp
#define ESTOP_BEHAVIOUR ESTOP_DETACH
```
Re-flash. This is a `TODO(hardware)` (H-11) decision for this board — confirm with the
mechanical/electrical side which failure mode is actually safer for the built rig before
running acceptance tests with a real tool loaded.

## Status LED (H-26)

Green = `OK` (idle, holding), **white = `OK` and jaws still slewing (`moving:true`)**,
yellow = `LOST_LINK`, red = `ESTOP`, blue = WiFi still connecting. Default
`STATUS_LED_PIN` is `RGB_BUILTIN`, which resolves to the on-board WS2812 addressable RGB
LED — **this is correct for board revision v1.0 (silkscreen), which wires it to GPIO48**.
The Arduino core does not detect board revision at compile time.

**If your board's silkscreen says v1.1** (LED on GPIO38 instead of GPIO48), edit
`include/config.h`:
```cpp
#define STATUS_LED_PIN 38
```
If no RGB LED is fitted at all, wire a plain LED to GPIO7 and set:
```cpp
#define STATUS_LED_IS_RGB 0
#define STATUS_LED_PIN 7
```

## Motion quality — slew limiter

Jaw angles are never written directly to their final target; a non-blocking slew limiter
(`updateJaws()` in `src/main.cpp`, driven by `millis()` from `loop()`) ramps each servo's
commanded angle toward its target at `JAW_SLEW_DEG_PER_S` (default 120 °/s). `moving` in
`/status` is `true` until both jaws arrive. A `seq` is still marked done (`lastSeq`
advances) the instant the target is *set* — per contract, there's no position feedback on
hobby servos, so "done" means "commanded," not "arrived."

## Contract notes (docs/contracts.md §C.2, implemented byte-identically)

- Every response is `Content-Type: application/json`.
- Malformed/missing JSON body → HTTP 400 `{"ok":false,"seq":0,"err":"bad_json"}`.
- `mm` outside `[0, JAW_MAX_MM]` is clamped, never rejected; the clamped value is echoed
  back in the response.
- `seq <= lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` without re-actuating.
- No authentication (same closed robot WiFi as the bridge).
- Any undefined route/method → HTTP 404 `{"ok":false,"seq":0,"err":"not_found"}`.

## TODO(hardware) constants introduced here

All in `include/config.h`:

| Constant | H-id | Placeholder | Notes |
|---|---|---|---|
| `TOOL_ID` / `TOOL_IP_LAST_OCTET` | H-09 | `0` / `30` | set via `platformio.ini` build_flags |
| `JAW_A_OPEN_DEG` / `JAW_A_CLOSED_DEG` | H-10 | `20` / `110` | guesses — calibrate per procedure above |
| `JAW_B_OPEN_DEG` / `JAW_B_CLOSED_DEG` | H-10 | `160` / `70` | mirrored guesses — calibrate |
| `JAW_MAX_MM` | H-10 | `40.0` | physical jaw travel, mm, at fully open — measure on the rig |
| `ESTOP_BEHAVIOUR` | H-11 | `ESTOP_HOLD` | deliberate default for this board; `ESTOP_DETACH` is the alternative |
| `SERVO_A_PIN` / `SERVO_B_PIN` | H-12 | `4` / `5` | verified safe GPIOs, but confirm against the physical harness |
| `WIFI_SSID` / `WIFI_PSK` | H-13 | `"subzero"` / `"changeme"` | unusable placeholders until the venue radio is confirmed |
| `STATUS_LED_PIN` | H-26 | `RGB_BUILTIN` | 48 on v1.0 silkscreen, 38 on v1.1 |

Non-hardware software tuning constant (not a hardware unknown, but bench-tunable):
`JAW_SLEW_DEG_PER_S` (default `120.0f`) — raise once the mechanism and servos are shown to
tolerate faster jaw motion; lower if servos strain or jaws overshoot.
