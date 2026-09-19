# Subzero end-effector firmware (ESP32-S3-DevKitC-1, DRV8833, open-loop timed DC motor)

One board per swappable end effector. Implements `docs/contracts.md` §C.2 (ESP32 HTTP API v0)
byte-identically, lifted from `esp32/legacy-servo-tool/`. The two hobby servos of the legacy
firmware are replaced by **one brushed DC motor on a DRV8833 H-bridge, driven open loop**:
there is no position feedback of any kind, every command is simply *"run forwards for x ms"*
or *"run backwards for x ms"* at some speed *y* (`TimedMotor`). `/latch`, `/release` and
`/lateral` are each a timed run on this single motor:

| Route | Timed run |
|---|---|
| `POST /latch` | `fwd` for `LATCH_RUN_MS` (800) at `LATCH_SPEED` (0.5) |
| `POST /release` | `rev` for `RELEASE_RUN_MS` (800) at `RELEASE_SPEED` (0.5) |
| `POST /lateral {"mm"}` | `mm` clamped to `[0, LATERAL_MAX_MM]`; dead-reckoned: `fwd` if `mm > est_mm` else `rev`, for `|mm − est_mm| × LATERAL_MS_PER_MM` ms (25 ms/mm) at `LATERAL_SPEED` (0.5) |
| `POST /run {"dir","ms","speed"}` (additive) | raw run: `dir` = `"fwd"`/`"rev"`, `ms` clamped to `[0, RUN_MAX_MS]` (5000), `speed` clamped to `[0, RUN_MAX_SPEED]` (0.8) — **the bench / calibration tool** |

`/status` contract fields: `latched` = the last *completed* `/latch`-or-`/release` run was
`/latch` (persists across `/lateral`, `/run`, reboots reset it to `false`); `lateral_mm` = the
dead-reckoned estimate `est_mm` (0 at boot).

Nothing here has been run on hardware by an agent. **Humans test on hardware** (safety rule 10).

## Wiring

| DRV8833 pin | Connect to | Notes |
|---|---|---|
| `VM` | motor supply, **2.7–10.8 V** (2S LiPo / 6 V buck is fine) | ≥ 10 µF bulk cap at VM; DRV8833 abs max 11.8 V |
| `VCC` (logic, if the breakout exposes it) | ESP32 **3V3** | never 5 V — the ESP32 GPIOs are 3.3 V |
| `GND` | ESP32 GND **and** motor-supply GND | one common ground, star at the driver |
| `AIN1` | GPIO **4** (`MOTOR_IN1_PIN`) | LEDC PWM 20 kHz / 10 bit |
| `AIN2` | GPIO **5** (`MOTOR_IN2_PIN`) | LEDC PWM 20 kHz / 10 bit |
| `nSLEEP` | GPIO **6** (`MOTOR_NSLEEP_PIN`) | HIGH = enabled, LOW = asleep (ESTOP "detach", also the boot state) — if the breakout has `EEP`/`SLP` this is it |
| `nFAULT` | GPIO **7** (`MOTOR_NFAULT_PIN`) | optional; open-drain, firmware enables the internal pull-up; set `-1` in `config.h` if not wired |
| `AOUT1` / `AOUT2` | motor leads | swap them (or set `MOTOR_INVERT 1`) if `fwd` runs the mechanism the wrong way |
| `BIN1` / `BIN2` | leave unconnected (or tie LOW) | channel B unused |

There is **no sensor**: no encoder, no end-stop switch, nothing on the ADC. The firmware only
knows what it commanded.

DRV8833 truth table as used by the firmware: `fwd` = PWM on IN1, IN2 LOW · `rev` = IN1 LOW,
PWM on IN2 · brake = both HIGH (motor shorted, no supply current) · coast = both LOW ·
nSLEEP LOW = outputs Hi-Z (limp). Direction changes always pass through coast.

Status LED: on-board WS2812 (`STATUS_LED_PIN RGB_BUILTIN` → GPIO48 on v1.0 boards, **GPIO38 on
v1.1** — H-26, edit `config.h`). Avoid GPIO 0/3/45/46 (strapping), 19/20 (USB), 43/44 (UART0).

## How it moves (open loop — read before the first power-on)

* A run is `TimedMotor::run(dir, ms, speed)`: `speed` 0..1 → PWM duty, clamped to
  `RUN_MAX_SPEED`; `ms` clamped to `RUN_MAX_MS`. It starts immediately and returns; `loop()`
  calls `update()` which stops the motor at the deadline (`RUN_END_BRAKE 1` → brake, both
  IN HIGH; `0` → coast). **A new actuate while a run is in progress replaces it** (new
  direction, new deadline) — no queueing.
* **No feedback → the contract's `seq` is acknowledged when the run STARTS**, exactly like the
  hobby-servo firmware acked when the target was written. `/status` `"running"` and
  `"remaining_ms"` tell you when it ends; `latched` flips only when a `/latch` or `/release`
  run reaches its deadline (an interrupted one commits nothing).
* `/lateral` is **dead reckoned**: `est_mm` starts at 0 at boot; each request runs toward the
  requested mm for `|Δ| × LATERAL_MS_PER_MM` ms and sets `est_mm := mm` when the run *starts*.
  Caveat: if that run is interrupted (`/estop`, link loss, another actuate) `est_mm` is ahead
  of the mechanism until a human re-homes it (drive it to the 0 mm end with `/run rev` and
  power-cycle, or command `/lateral 0` and accept the drift). A request equal to `est_mm`
  starts no run.
* Direction convention: `fwd` = PWM on `MOTOR_IN1_PIN` (AIN1); `/latch` runs `fwd`,
  `/release` runs `rev`, `/lateral` runs `fwd` toward increasing mm. If `fwd` moves the
  mechanism the wrong way set `MOTOR_INVERT 1` in `config.h` (equivalent to swapping the
  motor leads) — nothing else changes.
* **A run can never be left running:** every run has a deadline ≤ `RUN_MAX_MS` (5000 ms)
  enforced in `TimedMotor::run()`/`update()` independently of the watchdog; link loss stops it
  (below); if `loop()` ever stalls the 5 s task WDT resets the board and `begin()` puts the
  driver back to sleep.
* Boot: the driver is **asleep** (`nSLEEP` LOW) until the first actuate — nothing moves at
  boot. The firmware assumes the mechanism is at 0 mm / not latched; stow it before power-on.

## Safety semantics (`state` ∈ `OK` | `LOST_LINK` | `ESTOP`, byte-exact)

* `POST /estop` → `stop()` + `nSLEEP` LOW: driver asleep, outputs Hi-Z, motor coasts (limp),
  `"state":"ESTOP"`, `"awake":false`. Stays there until the next `/latch`, `/release`,
  `/lateral` or `/run`, which wakes the driver and runs (`ESTOP_BEHAVIOUR`, H-11 — see below).
* **`LOST_LINK`** (no `/heartbeat` for `HEARTBEAT_TIMEOUT_MS` = 1000 ms): the contract says
  "holds position". For an open-loop DC motor the hold-equivalent is **STOP**: any run in
  progress is cancelled with a brake (motor shorted, resists back-driving, draws no supply
  current), the driver stays awake, and **new runs are refused** — HTTP `503`
  `{"ok":false,"seq":i,"err":"lost_link"}` with `lastSeq` *not* advanced, so the bridge's
  normal retry (3 × 100 ms) succeeds as soon as its heartbeat lands. One `/heartbeat` returns
  `state` to `OK`; the motor stays stopped until commanded again. It never detaches on link
  loss alone (`LINK_LOSS_BEHAVIOUR STOP`). Consequence for the bench: **`curl` actuates only
  work while something is heartbeating** (one-liner in the bench test below).
* A `seq ≤ lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` with no motion, in every
  state.

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

1. Leave `RUN_MAX_SPEED` at the shipped **0.8** and use low speeds (0.3) and short runs
   (≤ 300 ms) for the first tests. Keep one hand on the motor-supply switch. Nothing moves at
   boot (driver asleep, `"awake":false` in `/status`).
2. Start a heartbeat in the background (otherwise every actuate is refused with `503 lost_link`
   after the first second):
   ```bash
   T=http://10.13.60.31
   while :; do curl -s -X POST $T/heartbeat -H 'Content-Type: application/json' -d '{"t":1}' >/dev/null; sleep 0.2; done &
   ```
3. One short, slow run forward:
   ```bash
   curl -X POST $T/run -H 'Content-Type: application/json' -d '{"seq":1,"dir":"fwd","ms":300,"speed":0.3}'
   ```
   Watch the mechanism and the serial monitor (`[motor] run fwd 300 ms @ 0.30 …` then
   `[motor] stop (brake)`).
   * It moved in the **closing / extending** direction → convention is right.
   * It moved the **wrong way** → set `MOTOR_INVERT 1` in `config.h`, rebuild, flash, retry.
   * It did not move → raise `speed` in 0.1 steps (stiction); check `VM`, `nSLEEP` wiring and
     `"driver_fault"` in `/status` (`nFAULT` low = over-current / thermal / under-voltage).
4. Same with `"dir":"rev"` (`"seq":2`) — it must go back.
5. Only then calibrate (next section) and try `/release`, `/latch`, `/lateral`.

## Calibration (H-10: `LATCH_RUN_MS`, `RELEASE_RUN_MS`, `LATERAL_MS_PER_MM`, speeds)

Everything is timed, so calibration = *time a full stroke with `/run`*:

1. With the heartbeat loop running, drive the mechanism to the fully-open / retracted end
   with short `rev` runs (`{"dir":"rev","ms":200,"speed":0.5}`, increment `seq` each time).
2. Run `fwd` at the speed you intend to use (`LATCH_SPEED`, 0.5) with a generous `ms`
   (e.g. 1500) and watch/listen for the mechanism to reach the closed end stop; note how long
   it actually took (serial timestamps or a phone video). Set **`LATCH_RUN_MS`** to that time
   plus ~10 % (a little over-run into a stop at 50 % duty is harmless for a short time; a lot
   is not — the DRV8833 `nFAULT` will tell you if it over-currents).
3. Repeat in `rev` for **`RELEASE_RUN_MS`** / `RELEASE_SPEED` (stroke times may differ under
   load).
4. Lateral: from the 0 mm end, run `fwd` at `LATERAL_SPEED` for a known `ms` (e.g. 500),
   measure the travel with a ruler → **`LATERAL_MS_PER_MM` = ms ÷ mm**. Check that
   `LATERAL_MAX_MM × LATERAL_MS_PER_MM` (40 × 25 = 1000 ms) is comfortably below
   `RUN_MAX_MS` (5000) — raise `RUN_MAX_MS` only if a real stroke needs it.
5. Rebuild + flash; verify `/latch`, `/release`, then `/lateral 20` / `/lateral 0` and check
   the mechanism returns to where it started (dead reckoning drifts with load and battery
   voltage — this is a v0 hackathon tool, not a positioner).

## Bench test (acceptance test 4 — all routes)

Point `curl` at the board's static IP (`10.13.60.31` for end effector 1). No auth, no TLS
(closed robot WiFi, `docs/contracts.md` §C.2 Δ). Keep the heartbeat loop from "First
power-on" running (`kill %1` stops it).

```bash
T=http://10.13.60.31
curl $T/status

curl -X POST $T/latch    -H 'Content-Type: application/json' -d '{"seq":1}'
curl -X POST $T/release  -H 'Content-Type: application/json' -d '{"seq":2}'
curl -X POST $T/lateral  -H 'Content-Type: application/json' -d '{"seq":3,"mm":12.5}'
curl -X POST $T/run      -H 'Content-Type: application/json' -d '{"seq":4,"dir":"fwd","ms":300,"speed":0.3}'   # additive bench route
curl -X POST $T/heartbeat -H 'Content-Type: application/json' -d '{"t":1}'
curl -X POST $T/estop
```

Expected:

* `/status` → `{"tool":1,"latched":bool,"lateral_mm":f,"rssi":i,"uptime_s":i,"lastSeq":i,"state":"OK"|"LOST_LINK"|"ESTOP",`
  plus additive `"running":bool,"remaining_ms":i,"dir":"fwd"|"rev","speed":f,"driver_fault":bool,"est_mm":f,"awake":bool}`.
* `/latch`, `/release` → `{"ok":true,"seq":i}`; `/lateral` → `{"ok":true,"seq":i,"lateral_mm":f}`
  (clamped value echoed); `/run` → `{"ok":true,"seq":i,"dir":"fwd","ms":i,"speed":f}`
  (clamped `ms`/`speed` echoed); `/heartbeat`, `/estop` → `{"ok":true}`. A `seq` is
  acknowledged as soon as the run **starts**; poll `/status` `"running":false` to know it
  finished, and `latched` flips only then.
* `/estop` flips `state` to `ESTOP`, stops the motor and puts the driver to sleep
  (`"awake":false`, `"running":false` — H-11). The next `/latch`, `/release`, `/lateral` or
  `/run` wakes it and clears `state` back to `OK`.

**Link-loss check (motor must stop):** start a long run, then kill the heartbeat loop:
```bash
curl -X POST $T/run -H 'Content-Type: application/json' -d '{"seq":5,"dir":"fwd","ms":4000,"speed":0.3}'
kill %1                     # stop the heartbeats
sleep 1.2; curl $T/status   # -> "state":"LOST_LINK","running":false  (motor braked well before the 4 s deadline)
curl -X POST $T/latch -H 'Content-Type: application/json' -d '{"seq":6}'
                            # -> 503 {"ok":false,"seq":6,"err":"lost_link"}  (refused, lastSeq unchanged)
curl -X POST $T/heartbeat -H 'Content-Type: application/json' -d '{"t":2}'
curl $T/status              # -> "state":"OK", motor still stopped
curl -X POST $T/latch -H 'Content-Type: application/json' -d '{"seq":6}'   # -> now runs
```
Also confirm the plain deadline: a `/run` with `"ms":300` stops on its own after 300 ms, and a
`/run` with `"ms":99999` is clamped to `RUN_MAX_MS` (echoed `"ms":5000`).

**Malformed / duplicate seq / clamp:**
```bash
curl -X POST $T/latch -d 'not json'                       # -> 400 {"ok":false,"seq":0,"err":"bad_json"}
curl -X POST $T/run   -d '{"seq":7,"dir":"up","ms":100,"speed":0.3}'   # -> 400 bad_json (dir must be "fwd"|"rev")
curl -X POST $T/run   -d '{"seq":7,"dir":"fwd"}'                       # -> 400 bad_json (ms, speed missing)
curl -X POST $T/latch -d '{"seq":1}'                      # repeat an already-seen seq
                                                          # -> 200 {"ok":true,"seq":1,"dup":true}, no motion
curl -X POST $T/lateral -d '{"seq":8,"mm":999}'           # -> 200 {"ok":true,"seq":8,"lateral_mm":40.0} (clamped, never rejected)
curl -X POST $T/run -d '{"seq":9,"dir":"rev","ms":99999,"speed":5}'
                                                          # -> 200 {"ok":true,"seq":9,"dir":"rev","ms":5000,"speed":0.8} (clamped)
```

## Status LED (H-26)

Green = `OK` idle, **cyan = `OK` + motor running**, yellow = `LOST_LINK`, red = `ESTOP`,
blue = WiFi still connecting, **magenta = DRV8833 `nFAULT` asserted**.
`STATUS_LED_PIN RGB_BUILTIN` is right for DevKitC-1 **v1.0** (GPIO48); on **v1.1** silkscreen set
`#define STATUS_LED_PIN 38`. Plain LED: `STATUS_LED_IS_RGB 0` + a free GPIO (e.g. 15).

## H-11 hardware question (open)

Software is decided: `POST /estop` **detaches** (driver asleep, mechanism limp),
`ESTOP_BEHAVIOUR ESTOP_BEHAVIOUR_DETACH` in `include/config.h`. **Ask on hardware:** does a
limp mechanism drop a held object under its own weight / vibration? If yes, flip
`ESTOP_BEHAVIOUR` to `ESTOP_BEHAVIOUR_HOLD` (e-stop then stops with a **brake** — motor shorted,
driver awake — instead of going limp) before running acceptance test 4 with an object actually
loaded. Note an open-loop DC motor cannot actively hold against a load in either mode; if the
mechanism needs active holding force it needs a self-locking drive (worm / lead screw) — a
hardware answer, not a firmware one. `LINK_LOSS_BEHAVIOUR STOP` is the only sane open-loop
choice for link loss and should stay.

## Contract notes (docs/contracts.md §C.2, implemented byte-identically)

- Every response is `Content-Type: application/json`.
- Malformed/missing JSON body → HTTP 400 `{"ok":false,"seq":0,"err":"bad_json"}`.
- `mm` outside `[0, LATERAL_MAX_MM]` is clamped, never rejected; the clamped value is echoed.
- `seq <= lastSeq` is acknowledged `{"ok":true,"seq":i,"dup":true}` without re-actuating.
- `/heartbeat` every 200 ms from the bridge; `LOST_LINK` after 1000 ms silence → motor stopped
  (open-loop hold), new runs refused with `503 lost_link` until a heartbeat returns.
- No authentication (same closed robot WiFi as the bridge).
- Any undefined route/method → HTTP 404 `{"ok":false,"seq":0,"err":"not_found"}`.
- Additive only: extra `/status` fields listed above, `POST /run`, and the `503 lost_link`
  refusal. Nothing renamed/removed.

## TODO(hardware) constants (`include/config.h`)

| Constant | H-id | Placeholder (fails safe) |
|---|---|---|
| `TOOL_ID`, `TOOL_IP_LAST_OCTET` | H-09 | 1 / 31 (overridden per env) |
| `WIFI_SSID`, `WIFI_PSK` | H-13 | `"subzero"` / `"changeme"` (unusable on purpose) |
| `MOTOR_IN1_PIN`, `MOTOR_IN2_PIN`, `MOTOR_NSLEEP_PIN`, `MOTOR_NFAULT_PIN` | H-12 | 4, 5, 6, 7 (−1 = unwired) |
| `MOTOR_INVERT` | H-12 | 0 (set 1 if `fwd` runs the wrong way) |
| `RUN_MAX_MS`, `RUN_MAX_SPEED` | H-10 | 5000 ms, 0.8 (hard caps on every run) |
| `LATCH_RUN_MS`, `LATCH_SPEED` | H-10 | 800 ms, 0.5 |
| `RELEASE_RUN_MS`, `RELEASE_SPEED` | H-10 | 800 ms, 0.5 |
| `LATERAL_MS_PER_MM`, `LATERAL_SPEED`, `LATERAL_MAX_MM` | H-10 | 25 ms/mm, 0.5, 40.0 mm |
| `ESTOP_BEHAVIOUR` | H-11 | `DETACH` (gate decision; flip to `HOLD` if a limp mechanism drops the object) |
| `LINK_LOSS_BEHAVIOUR` | H-11 | `STOP` (brake + refuse runs; the open-loop "hold") |
| `STATUS_LED_PIN`, `STATUS_LED_IS_RGB` | H-26 | `RGB_BUILTIN` (GPIO48 v1.0 / 38 v1.1), 1 |
