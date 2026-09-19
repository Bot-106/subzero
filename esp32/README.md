# esp32/ — ESP32-S3 boards

| Directory | Board | What |
|---|---|---|
| `endeffector/` | one per swappable end effector (tool ids 1..n, IPs 10.13.60.31+) | DRV8833-driven DC motor + potentiometer = emulated servo; serves HTTP v0 (`docs/contracts.md §C.2`) |
| `pincher/` | on the arm carriage (tool id 0, IP 10.13.60.30) | two micro-servos that pinch / release an end effector; serves HTTP v0 (`/lateral` = jaw gap in mm) |
| `legacy-servo-tool/` | — | the original servo-based tool firmware (reference only; no servos on tools any more) |

Build any of them: `cd esp32/<dir> && ~/.platformio/penv/bin/pio run -e <env>`.
