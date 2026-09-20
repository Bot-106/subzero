# esp32/ — ESP32-S3 boards

| Directory | Board | What |
|---|---|---|
| `endeffector/` | one per swappable end effector (tool ids 1..n, IPs 10.13.60.31+) | DRV8833-driven DC motor, open-loop timed runs ("forward/backward for x ms at speed y"); serves the HTTP API documented in `endeffector/README.md` (the RoboRIO calls it directly over the robot WiFi) |
| *(pincher)* | — | the two jaw servos are driven by the **RoboRIO** (PWM 0/1, `frc/.../subsystems/Pincher.java`), no ESP32 |
| `legacy-servo-tool/` | — | the original servo-based tool firmware (reference only; no servos on tools any more) |

Build any of them: `cd esp32/<dir> && ~/.platformio/penv/bin/pio run -e <env>`.
