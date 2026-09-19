# Jetson / PhotonVision sign-off checklist — Subzero (Team 1360)

Run through this after `runbook.md`. Every box should be checked against the
live device/UI, not assumed from a previous session.

- [ ] PhotonVision UI reachable at **http://10.13.60.11:5800**
- [ ] Both cameras listed in the PhotonVision UI (or one, if this robot only
      has one camera per H-14 — confirm against `Constants.java` at the time)
- [ ] NetworkTables connected: PhotonVision Settings shows team **1360** /
      server `10.13.60.2`, and a NetworkTables client on the RoboRIO side
      sees `/photonvision/...` topics populated
- [ ] AprilTag field layout uploaded (`docs/room-layout.template.json`),
      **validated first** with
      `uv run --project integration python docs/validate_layout.py docs/room-layout.template.json`
      on the laptop before uploading
- [ ] Each camera's pipeline set: AprilTag, 36h11, tag size **165.1 mm**
      (H-15), 3D mode enabled
- [ ] Each camera calibrated **at the resolution actually used** in its
      pipeline (record the resolution — see H-18 table in `runbook.md`)
- [ ] Camera names in the PhotonVision UI exactly match
      `frc/src/main/java/frc/robot/Constants.java`
      (`VisionConstants.kFrontLeftCameraName` = `photoncamera_fl`,
      `kFrontRightCameraName` = `photoncamera_fr`)
- [ ] fps ≥ 20 and latency < 60 ms shown in the PhotonVision UI for each
      camera with a tag in view
- [ ] `photonvision.service` (and `picam-bridge@0`/`@1` if CSI cameras are
      used) are `enabled` and come back `active (running)` after
      `sudo reboot` with no manual steps
- [ ] Static IP `10.13.60.11/24`, gateway `10.13.60.1` survives a reboot
- [ ] H-18 fill-in table in `runbook.md` is fully filled in from live device
      output (no blank/guessed rows)

## "Jetson up" sign-off

Once every box above is checked on the physical device:

Name: ______________________  Time: ______________________

Notes / deviations from the runbook (if any): ______________________
