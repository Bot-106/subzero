# Subzero end-effector firmware (ESP32-S3-DevKitC-1, DRV8833 + pot = emulated servo)

One board per swappable end effector. Implements `docs/contracts.md` §C.2 (ESP32 HTTP API v0)
byte-identically, lifted from `esp32/legacy-servo-tool/`. The two hobby servos of the legacy
firmware are replaced by **one brushed DC motor on a DRV8833 H-bridge plus a potentiometer
mechanically coupled to the axis**, wrapped as an emulated servo (`EmuServo`: `write(deg)` →
closed-loop position control on the pot). `/latch`, `/release` and `/lateral` all drive this
single axis:

| Route | Emulated-servo target |
|---|---|
| `POST /latch` | `EMU_LATCH_CLOSED_DEG` (110) |
| `POST /release` | `EMU_LATCH_OPEN_DEG` (20) |
| `POST /lateral {"mm"}` | `mm` clamped to `[0, LATERAL_MAX_MM]`, mapped linearly onto `[LATERAL_MIN_DEG, LATERAL_MAX_DEG]` |
| `POST /pos {"deg"}` (additive) | `deg` clamped to `[EMU_SOFT_MIN_DEG, EMU_SOFT_MAX_DEG]` — bench/calibration |

`/status` reports the **measured** axis: `latched` = `|pos_deg − EMU_LATCH_CLOSED_DEG| < LATCHED_TOLERANCE_DEG`,
`lateral_mm` = `pos_deg` mapped back to mm (clamped to `[0, LATERAL_MAX_MM]`).

Nothing here has been run on hardware by an agent. **Humans test on hardware** (safety rule 10).

## Wiring

| DRV8833 pin | Connect to | Notes |
|---|---|---|
| `VM` | motor supply, **2.7–10.8 V** (2S LiPo / 6 V buck is fine) | ≥ 10 µF bulk cap at VM; DRV8833 abs max 11.8 V |
| `VCC` (logic, if the breakout exposes it) | ESP32 **3V3** | never 5 V — the ESP32 GPIOs are 3.3 V |
| `GND` | ESP32 GND **and** motor-supply GND | one common ground, star at the driver |
| `AIN1` | GPIO **4** (`MOTOR_IN1_PIN`) | LEDC PWM 20 kHz / 10 bit |
| `AIN2` | GPIO **5** (`MOTOR_IN2_PIN`) | LEDC PWM 20 kHz / 10 bit |
| `nSLEEP` | GPIO **6** (`MOTOR_NSLEEP_PIN`) | HIGH = enabled, LOW = asleep (ESTOP "detach") — if the breakout has `EEP`/`SLP` this is it |
| `nFAULT` | GPIO **7** (`MOTOR_NFAULT_PIN`) | optional; open-drain, firmware enables the internal pull-up; set `-1` in `config.h` if not wired |
| `AOUT1` / `AOUT2` | motor leads | swap them (or set `POT_INVERT`) if the axis runs the wrong way |
| `BIN1` / `BIN2` | leave unconnected (or tie LOW) | channel B unused |

| Potentiometer | Connect to | Notes |
|---|---|---|
| end 1 | ESP32 **3V3** | **never 5 V** into the ADC |
| end 2 | ESP32 GND | |
| wiper | GPIO **1** (`POT_ADC_PIN`, ADC1_CH0) | optional 1 kΩ series + 100 nF wiper→GND for noise |

**Why the pot must be on GPIO1–GPIO10 (ADC1):** on the ESP32-S3, ADC2 (GPIO11–20) is shared
with the WiFi radio and returns garbage or fails while WiFi is up — and WiFi is always up on
this board. `analogReadMilliVolts()` uses the factory calibration; the firmware sets 11 dB
attenuation (≈ 0–3.1 V usable) and averages `POT_ADC_SAMPLES` (8) per read.

DRV8833 truth table as used by the firmware: forward = PWM on IN1, IN2 LOW · reverse = IN1 LOW,
PWM on IN2 · brake = both HIGH (motor shorted, no supply current) · coast = both LOW ·
nSLEEP LOW = outputs Hi-Z (limp).

Status LED: on-board WS2812 (`STATUS_LED_PIN RGB_BUILTIN` → GPIO48 on v1.0 boards, **GPIO38 on
v1.1** — H-26, edit `config.h`). Avoid GPIO 0/3/45/46 (strapping), 19/20 (USB), 43/44 (UART0).

## Direction / sign conventions (read before the first power-on)

* Degrees are a *name* for the linear map through the two calibration points:
  `deg = (mV − POT_MV_AT_0DEG) · 180 / (POT_MV_AT_180DEG − POT_MV_AT_0DEG)`.
  "0 deg" and "180 deg" are simply the two mechanical end stops. With the default calibration
  (`300` / `2800` mV) **increasing degrees = increasing pot voltage**; if you calibrate them the
  other way round the map inverts by itself — that is fine.
* The controller computes `error = target − pos` and a signed duty; **positive duty = "forward"
  = PWM on `MOTOR_IN1_PIN` (AIN1) with AIN2 LOW**, and forward **must move the axis toward
  increasing degrees, i.e. toward `POT_MV_AT_180DEG`** (with the default calibration: toward
  *increasing* pot voltage).
* If the motor is wired so that forward drives the axis toward *decreasing* degrees, the loop
  will run away from the target: the axis hits an end stop, no progress is made, and after
  `EMU_STALL_TIMEOUT_MS` (2 s) it enters **FAULT** with the motor coasting. Fix: set
  `POT_INVERT 1` in `config.h` (equivalent to swapping the motor leads) and rebuild. Do **not**
  swap the calibration points to fix direction — they describe the pot, not the motor.
* `/status` `"duty"` is the *logical* signed duty (+ = toward increasing degrees) before
  `POT_INVERT` is applied, so a positive duty with a falling `pos_deg` is the tell-tale of a
  direction mismatch.

Control loop (`EmuServo::update()`, `EMU_LOOP_HZ` = 200, `micros()`-paced from `loop()`, no
`delay()`): P (+ optional I, D) on angle error → signed duty; below `EMU_MIN_DUTY` the duty is
raised to the stiction kick, above `EMU_MAX_DUTY` it is capped; inside `EMU_TOLERANCE_DEG` the
motor **brakes**, and after `EMU_SETTLE_MS` inside the band the state is `AT_TARGET`
(re-drives when pushed more than `EMU_TOLERANCE_DEG + EMU_REENGAGE_DEG` away). The stall guard
is mandatory: while driving, if `|error|` has not improved by `EMU_STALL_PROGRESS_DEG` for
`EMU_STALL_TIMEOUT_MS` → `FAULT` (`fault: "stall"`), motor coasts, cleared by the next
`/latch`, `/release`, `/lateral` or `/pos`. The motor is never left driven forever.

## Build / flash / monitor

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
cd esp32/endeffector

# end effector 1 (TOOL_ID=1, 10.13.60.31)
pio run -e endeffector-1
pio run -e endeffector-1 -t upload
pio device monitor -b 115200      # ctrl-c to exit; /dev/cu.usbmodem* on macOS

# end effector 2 (TOOL_ID=2, 10.13.60.32)
pio run -e endeffector-2
pio run -e endeffector-2 -t upload
pio device monitor -b 115200
```

End effector 3 (`10.13.60.33`): copy the `[env:endeffector-2]` block in `platformio.ini`,
rename it `endeffector-3`, set `-D TOOL_ID=3 -D TOOL_IP_LAST_OCTET=33`.
If upload hangs on "Connecting...": hold BOOT, tap RESET, release BOOT, retry.
Fill in `WIFI_SSID` / `WIFI_PSK` in `include/config.h` first (H-13) — the placeholders are
deliberately unusable.

## First power-on (safety steps)

1. Leave `EMU_MAX_DUTY` at the shipped **700/1023 (≈ 68 %)** and `EMU_STALL_TIMEOUT_MS` at 2000
   for the first test. Keep one hand on the motor-supply switch.
2. Boot default is `EMU_BOOT_BEHAVIOUR BOOT_DETACHED`: the driver stays **asleep and nothing
   moves** until the first command. `/status` shows `"emu_state":"IDLE","attached":false`.
3. Calibrate the pot first (next section) so that degrees mean something.
4. Command a small move away from wherever the axis sits, e.g. `/pos {"deg": <pos_deg + 15>}`,
   and watch `pos_deg` in `/status` (or the serial monitor `[emu]` lines).
   * `pos_deg` moves *toward* the target and `emu_state` becomes `AT_TARGET` → direction is right.
   * the axis runs **away** from the target / into the end stop and `emu_state` becomes `FAULT`
     after 2 s → set `POT_INVERT 1` in `config.h`, rebuild, retry. (Motor is coasting in FAULT;
     the next command clears it.)
5. Only then try `/release`, `/latch`, `/lateral`, and only then raise `EMU_MAX_DUTY` / tune
   `EMU_KP` if the move is too slow or sluggish (raise `EMU_MIN_DUTY` if it stalls short of the
   target with `fault: "stall"`; lower `EMU_KP` or raise `EMU_TOLERANCE_DEG` if it hunts).
6. Once direction + calibration are verified you may set `EMU_BOOT_BEHAVIOUR BOOT_OPEN` to get
   the legacy "go to open at boot" behaviour.

## Calibration procedure (H-10: `POT_MV_AT_0DEG` / `POT_MV_AT_180DEG`)

1. Flash, power up, leave the boot default (`BOOT_DETACHED` — motor asleep, axis free to move
   by hand).
2. Move the axis by hand to one mechanical end stop; read the pot:
   ```bash
   curl http://10.13.60.31/status      # note "pot_mv"
   ```
3. Move it by hand to the other end stop; read `pot_mv` again.
4. In `include/config.h` set `POT_MV_AT_0DEG` to the first reading and `POT_MV_AT_180DEG` to
   the second (either order works; pick "0 deg" = the *released/retracted* end so the shipped
   `EMU_LATCH_OPEN_DEG 20` / `EMU_LATCH_CLOSED_DEG 110` / `LATERAL_*_DEG` still make sense),
   then set `EMU_SOFT_MIN_DEG` / `EMU_SOFT_MAX_DEG` a few degrees inside the stops.
5. Rebuild + flash. Verify `pos_deg` in `/status` now reads ≈ 0 and ≈ 180 at the two stops.
6. Then set the real `EMU_LATCH_OPEN_DEG`, `EMU_LATCH_CLOSED_DEG`, `LATERAL_MIN_DEG`,
   `LATERAL_MAX_DEG`, `LATERAL_MAX_MM` for the mechanism (use `/pos` to find them).
7. Optionally tighten `POT_MV_FAULT_LOW` / `POT_MV_FAULT_HIGH` to ≈ ±150 mV outside the two
   readings so a broken wiper wire faults immediately (`fault: "pot_range"`) instead of after
   the 2 s stall timeout.

## Bench test (acceptance test 4 — all routes)

Point `curl` at the board's static IP (`10.13.60.31` for end effector 1). No auth, no TLS
(closed robot WiFi, `docs/contracts.md` §C.2 Δ).

```bash
curl http://10.13.60.31/status

curl -X POST http://10.13.60.31/latch \
  -H 'Content-Type: application/json' -d '{"seq":1}'

curl -X POST http://10.13.60.31/release \
  -H 'Content-Type: application/json' -d '{"seq":2}'

curl -X POST http://10.13.60.31/lateral \
  -H 'Content-Type: application/json' -d '{"seq":3,"mm":12.5}'

curl -X POST http://10.13.60.31/pos \
  -H 'Content-Type: application/json' -d '{"seq":4,"deg":90}'        # additive bench route

curl -X POST http://10.13.60.31/heartbeat \
  -H 'Content-Type: application/json' -d '{"t":1}'

curl -X POST http://10.13.60.31/estop
```

Expected:

* `/status` → `{"tool":1,"latched":bool,"lateral_mm":f,"rssi":i,"uptime_s":i,"lastSeq":i,"state":"OK"|"LOST_LINK"|"ESTOP",`
  plus additive `"pos_deg":f,"target_deg":f,"pot_mv":i,"pot_raw":i,"emu_state":"IDLE"|"MOVING"|"AT_TARGET"|"FAULT","fault":"none"|"stall"|"pot_range","driver_fault":bool,"attached":bool,"duty":i}`.
  `state` never carries FAULT — that lives only in `emu_state` / `fault`.
* `/latch`, `/release` → `{"ok":true,"seq":i}`; `/lateral` → `{"ok":true,"seq":i,"lateral_mm":f}`
  (clamped value echoed); `/pos` → `{"ok":true,"seq":i,"deg":f}` (clamped); `/heartbeat`,
  `/estop` → `{"ok":true}`. A `seq` is acknowledged as soon as the target is set (contract);
  poll `/status` `emu_state == "AT_TARGET"` if you want to know the move finished.
* `/estop` flips `state` to `ESTOP` and **detaches** (nSLEEP low, both IN low: motor coasts,
  `"attached":false`, `"emu_state":"IDLE"` — H-11). The next `/latch`, `/release`, `/lateral`
  or `/pos` re-attaches and clears `state` back to `OK`.

**Link-loss check:** stop sending `/heartbeat` for 1 s (the bridge's normal cadence is every
200 ms) → `GET /status` shows `"state":"LOST_LINK"` and the axis **keeps closed-loop holding
its last target** (never detaches on link loss alone — only an explicit `/estop` detaches, per
H-11). One more `/heartbeat` returns it to `"state":"OK"`.

**Stall check (mandatory guard):** block the axis by hand, command a move → after 2 s the
serial log prints `[emu] FAULT stall …`, `/status` shows `"emu_state":"FAULT","fault":"stall"`,
the LED goes magenta and the motor coasts. The next command clears it.

**Malformed / duplicate seq / clamp:**
```bash
curl -X POST http://10.13.60.31/latch -d 'not json'          # -> 400 {"ok":false,"seq":0,"err":"bad_json"}
curl -X POST http://10.13.60.31/latch -d '{"seq":1}'          # repeat an already-seen seq
                                                               # -> 200 {"ok":true,"seq":1,"dup":true}, no re-actuation
curl -X POST http://10.13.60.31/lateral -d '{"seq":9,"mm":999}'
                                                               # -> 200 {"ok":true,"seq":9,"lateral_mm":40.0} (clamped, never rejected)
curl -X POST http://10.13.60.31/pos -d '{"seq":10,"deg":999}'
                                                               # -> 200 {"ok":true,"seq":10,"deg":180.0} (clamped to EMU_SOFT_MAX_DEG)
```

## Status LED (H-26)

Green = `OK`, yellow = `LOST_LINK`, red = `ESTOP`, blue = WiFi still connecting,
**magenta = FAULT** (EmuServo stall / pot-range fault, or DRV8833 `nFAULT` asserted).
`STATUS_LED_PIN RGB_BUILTIN` is right for DevKitC-1 **v1.0** (GPIO48); on **v1.1** silkscreen set
`#define STATUS_LED_PIN 38`. Plain LED: `STATUS_LED_IS_RGB 0` + a free GPIO (e.g. 15).

## H-11 hardware question (open)

Software is decided: `POST /estop` **detaches** (driver asleep, axis limp),
`ESTOP_BEHAVIOUR = DETACH` in `include/config.h`. **Ask on hardware:** does a limp axis drop a
held object under its own weight / vibration? If yes, flip `ESTOP_BEHAVIOUR` to `HOLD`
(the loop keeps holding the last target on e-stop instead of going limp) before running
acceptance test 4 with an object actually loaded. Related knobs: `EMU_FAULT_BRAKE 1` makes a
stall FAULT brake (motor shorted) instead of coast, which holds a little better without drawing
supply current; `EMU_HOLD_BRAKE` (default 1) is what holds position at target.

## Contract notes (docs/contracts.md §C.2, implemented byte-identically)

- Every response is `Content-Type: application/json`.
- Malformed/missing JSON body → HTTP 400 `{"ok":false,"seq":0,"err":"bad_json"}`.
- `mm` outside `[0, LATERAL_MAX_MM]` is clamped, never rejected; the clamped value is echoed.
- `seq <= lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` without re-actuating.
- `/heartbeat` every 200 ms from the bridge; `LOST_LINK` after 1000 ms silence, target held.
- No authentication (same closed robot WiFi as the bridge).
- Any undefined route/method → HTTP 404 `{"ok":false,"seq":0,"err":"not_found"}`.
- Additive only: extra `/status` fields listed above and `POST /pos`. Nothing renamed/removed.

## TODO(hardware) constants (`include/config.h`)

| Constant | H-id | Placeholder (fails safe) |
|---|---|---|
| `TOOL_ID`, `TOOL_IP_LAST_OCTET` | H-09 | 1 / 31 (overridden per env) |
| `WIFI_SSID`, `WIFI_PSK` | H-13 | `"subzero"` / `"changeme"` (unusable on purpose) |
| `MOTOR_IN1_PIN`, `MOTOR_IN2_PIN`, `MOTOR_NSLEEP_PIN`, `MOTOR_NFAULT_PIN`, `POT_ADC_PIN` | H-12 | 4, 5, 6, 7 (−1 = unwired), 1 (ADC1_CH0) |
| `POT_MV_AT_0DEG`, `POT_MV_AT_180DEG`, `POT_INVERT` | H-10 | 300 mV, 2800 mV, 0 |
| `POT_MV_FAULT_LOW`, `POT_MV_FAULT_HIGH` | H-10 | 0 / 4000 (window disabled) |
| `EMU_LATCH_OPEN_DEG`, `EMU_LATCH_CLOSED_DEG`, `LATCHED_TOLERANCE_DEG` | H-10 | 20, 110, 5 |
| `LATERAL_MIN_DEG`, `LATERAL_MAX_DEG`, `LATERAL_MAX_MM` | H-10 | 0, 110, 40.0 |
| `EMU_SOFT_MIN_DEG`, `EMU_SOFT_MAX_DEG` | H-10 | 0, 180 |
| `EMU_MIN_DUTY`, `EMU_MAX_DUTY`, `EMU_STALL_TIMEOUT_MS` | H-10 | 180, 700 (68 %, low on purpose), 2000 |
| `ESTOP_BEHAVIOUR` | H-11 | `DETACH` (gate decision; flip to `HOLD` if a limp axis drops the object) |
| `EMU_BOOT_BEHAVIOUR` | H-11 | `BOOT_DETACHED` (nothing moves at boot) |
| `STATUS_LED_PIN`, `STATUS_LED_IS_RGB` | H-26 | `RGB_BUILTIN` (GPIO48 v1.0 / 38 v1.1), 1 |
