# Subzero (Hack the North 2026)

An FRC-style swerve robot with a 1.5 m elevator, an extending arm and WiFi servo tools, aligning to AprilTags in a hackathon room and picking/placing with swappable tools. The monorepo holds the RoboRIO code (`frc/`, WPILib 2026 + Phoenix6 + PhotonLib + AdvantageKit), the ESP32-S3 tool firmware (`esp32/`), the laptop NT4↔HTTP bridge and human-control CLI (`integration/`), the Jetson/PhotonVision runbook (`jetson/`) and the room-layout / hardware docs (`docs/`). See `CLAUDE.md` for how to build, simulate and test each part; `docs/contracts.md` for the frozen NT / HTTP / primitive contracts.

## Status

| Item | Status | Evidence |
|---|---|---|
| `frc/` builds (`./gradlew build`) | pending | — |
| Sim: `alignToTag(3)` < 3 cm / 2° from 1.5 m in 5 s (acceptance 1) | pending | — |
| Sim: homing + `elevatorTo(0.8)` + `armTo(0.3)` within soft limits (acceptance 2) | pending | — |
| Bridge ↔ `mock_tool.py` round-trip < 300 ms, link-loss ≤ 1.5 s (acceptance 3) | pending | — |
| ESP32 firmware `pio run` | pending | — |
| Jetson runbook `bash -n` | pending | — |
| `docs/validate_layout.py` on the template | pending | — |
| ESP32 bench (acceptance 4) | **humans** | — |
| Hardware M1 3× (acceptance 5) | **humans** | — |

## Cuts
(none yet)
