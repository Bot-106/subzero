# Jetson PhotonVision bring-up runbook — Subzero (Team 1360)

Follow this cold, over SSH, on the Jetson. Every value marked "detect on
device" must be read from the box in front of you — never assume it matches
a previous robot, a demo, or anything in this doc. Where two paths are given
(e.g. NetworkManager vs netplan), pick the one this device actually uses and
skip the other.

**Pinned versions:** PhotonVision **v2026.3.4** (docs:
https://docs.photonvision.org/en/v2026.3.4/ — do NOT use `/en/latest/`, that
is the 2027 alpha). Java **17**. Robot code camera names this must match:
`photoncamera_fl`, `photoncamera_fr` (see step 10, H-14).

**Network facts (fixed, do not detect):** RoboRIO `10.13.60.2`, Jetson static
IP `10.13.60.11/24`, gateway/radio `10.13.60.1`, FRC team number `1360`.

## Safety (read first — non-negotiable)

- The Jetson lives on the robot. Power it only from the robot PDH/VRM, and
  only while the robot is **disabled** at the Driver Station — never from a
  bench supply substituted in ad hoc, and never while anyone is testing
  robot motion.
- Never work on the Jetson (cabling, reseating, rebooting it on the robot)
  while the arm is extended. Stow pose (elevator down, arm retracted)
  first, and have a second person spotting if the arm is anywhere but
  stowed.
- If battery voltage reads below **11.0 V** under load at any point during a
  session that touches this robot, stop testing — this applies to the whole
  robot, not just vision work.

This runbook only brings up software/network on the coprocessor. It does not
move any actuator and does not require the robot enabled. No claim in this
document or its outputs should ever say "tested on hardware" unless a human
did it — agents may only say "passes in sim" with a log path.

---

## 0. Backup + free disk (5 min)

Detect free disk before doing anything else:

```bash
df -h /
```

If less than ~4 GB free, `sudo apt clean` and `sudo journalctl --vacuum-size=200M`
before continuing.

Back up package state and running services so this box can be restored if a
later step (especially step 4's kernel module work) goes wrong:

```bash
dpkg --get-selections > ~/pkgs-backup-$(date +%F).txt
systemctl list-units --type=service --state=running > ~/services-backup-$(date +%F).txt
crontab -l > ~/crontab-backup-$(date +%F).txt 2>&1 || echo "no crontab"
```

Copy those three files (and, if there's somewhere to put it, `/etc`) off the
Jetson to a USB drive or a laptop over `scp`/`rsync`. There is no fixed
backup target for this device — **decide on the day** (USB drive plugged
into the Jetson, or `user@laptop:/path` over the network) and write down
which one you used at the top of your session notes. Example once you've
picked a target:

```bash
rsync -aHAX --info=progress2 /home /opt /etc <your-chosen-backup-target>/jetson-backup-$(date +%F)/
```

## 1. Detect JetPack / L4T / existing hardware (5 min)

Do not guess any of this — read it off the device:

```bash
cat /etc/nv_tegra_release        # R35.x = JetPack 5 (Ubuntu 20.04 focal); R36.x = JetPack 6 (Ubuntu 22.04 jammy)
dpkg -l | grep nvidia-jetpack    # JetPack meta-package version, if installed
jetson_release 2>/dev/null || echo "jetson_release not installed (jetson-stats not present, skip)"
uname -r                         # kernel: 5.10.x-tegra (JP5) or 5.15.x-tegra (JP6) — must match step 4's headers
lsb_release -a 2>/dev/null || cat /etc/os-release   # Ubuntu 20.04 vs 22.04, confirms JetPack branch
df -h /                          # free disk (record for H-18)
ls /dev/video*; v4l2-ctl --list-devices 2>/dev/null   # any cameras already visible (record model/driver for H-18)
gst-inspect-1.0 nvarguscamerasrc  # confirms the Argus CSI plugin is present; needed for step 5
ip link                          # NIC name(s) — record for step 6, do NOT assume eth0
```

**Branch here:** JetPack 5 boxes are Ubuntu 20.04 (`focal`) and typically use
NetworkManager for networking. JetPack 6 boxes are Ubuntu 22.04 (`jammy`) and
typically use netplan/systemd-networkd. Confirm which this box actually runs
in step 6 rather than assuming from the JetPack version alone — some images
are customized.

## 2. apt sources sanity + Java 17 (10 min)

```bash
cat /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null   # confirm sources aren't dead/pointing at a decommissioned mirror
sudo apt update
sudo apt install -y openjdk-17-jre-headless
java -version   # expect "openjdk version \"17..." — PhotonVision v2026.3.4 requires Java 17, not 11 or 21
```

If `apt update` fails on a mirror, that mirror is stale on this image —
switch `sources.list` to a working Ubuntu/NVIDIA mirror before continuing;
which mirror works is something to detect from the error, not guess.

## 3. Install PhotonVision v2026.3.4 as a systemd service (10 min)

Confirm the exact release asset filename first — do not guess it:

```bash
curl -sL https://api.github.com/repos/PhotonVision/photonvision/releases/tags/v2026.3.4 | grep browser_download_url
```

Look for the `linuxarm64` JAR in that output (the URL pattern is
`https://github.com/PhotonVision/photonvision/releases/download/v2026.3.4/photonvision-v2026.3.4-linuxarm64.jar`,
but confirm the filename the command above actually prints before using it —
release asset names can change build-to-build).

```bash
sudo mkdir -p /opt/photonvision
sudo curl -L -o /opt/photonvision/photonvision.jar "<confirmed browser_download_url from above>"
ls -la /opt/photonvision/photonvision.jar   # sanity-check it's a real JAR (tens of MB), not an HTML error page
```

Install the service unit from this repo (`jetson/systemd/photonvision.service`
— copy it to the Jetson however you move files onto this box, e.g. `scp`):

```bash
sudo cp photonvision.service /etc/systemd/system/photonvision.service
sudo systemctl daemon-reload
sudo systemctl enable --now photonvision.service
sudo systemctl status photonvision.service --no-pager   # expect "Active: active (running)"
curl -sI http://localhost:5800 | head -1                 # expect HTTP/1.1 200 OK
```

UI should now be reachable at **http://10.13.60.11:5800** once step 6's
static IP is applied (before that, it's reachable at whatever DHCP address
the box currently has).

## 4. CSI cameras: v4l2loopback (10–60 min, riskiest step — see decision point)

Detect kernel headers before attempting a module build:

```bash
uname -r
ls /usr/src/ | grep linux-headers    # look for linux-headers-$(uname -r)... — empty means no matching headers for this L4T rev
```

Try the packaged module first:

```bash
sudo apt install -y v4l2loopback-dkms v4l2loopback-utils
modinfo v4l2loopback | head -5   # success -> skip the source-build block below
```

If that's unavailable, build from source against the header tree you found
above (only proceed if `ls /usr/src/` above showed a matching headers
directory — if it didn't, stop this path and go to the decision point below
instead of chasing a header rebuild):

```bash
sudo apt install -y build-essential dkms git
git clone https://github.com/v4l2loopback/v4l2loopback.git /tmp/v4l2loopback
cd /tmp/v4l2loopback
make KERNELRELEASE=$(uname -r) KERNEL_DIR=/usr/src/linux-headers-$(uname -r)*
sudo make install KERNELRELEASE=$(uname -r) KERNEL_DIR=/usr/src/linux-headers-$(uname -r)*
sudo depmod -a
```

Load it and create two loopback devices (video10, video11 — one per CSI
camera; drop the second if there's only one camera):

```bash
sudo modprobe v4l2loopback devices=2 video_nr=10,11 exclusive_caps=1 card_label="picam0,picam1" max_buffers=2
v4l2-ctl --list-devices   # expect picam0 -> /dev/video10, picam1 -> /dev/video11
```

Persist across reboot:

```bash
echo "v4l2loopback" | sudo tee /etc/modules-load.d/v4l2loopback.conf
sudo tee /etc/modprobe.d/v4l2loopback.conf > /dev/null <<'EOF'
options v4l2loopback devices=2 video_nr=10,11 card_label="picam0,picam1" exclusive_caps=1 max_buffers=2
EOF
```

### DECISION POINT — v4l2loopback/CSI not working after 1 hour total

**If you have spent about an hour on this step (headers missing, module
won't build, module builds but no CSI frames, etc.), stop and switch to USB
webcams instead of continuing to fight the CSI path:**

1. Plug in a USB webcam (repeat per camera).
2. `v4l2-ctl --list-devices` — it should show up immediately as `uvcvideo`,
   no loopback and no GStreamer bridge needed.
3. PhotonVision picks it up directly in its Cameras tab — no step 5 required
   for that camera.
4. Calibrate it exactly as in step 9, and name it exactly as in step 10
   (`photoncamera_fl` / `photoncamera_fr`) — everything downstream of "a
   camera exists in PhotonVision" is identical regardless of which physical
   path got it there.

Skip step 5 entirely for any camera handled this way.

## 5. GStreamer YUY2 bridge per CSI camera (10 min/camera — skip if using USB webcams)

Copy `gst/picam-to-v4l2.sh` from this repo onto the Jetson at
`/opt/subzero/picam-to-v4l2.sh` (the path the service unit below expects):

```bash
sudo mkdir -p /opt/subzero
sudo cp picam-to-v4l2.sh /opt/subzero/picam-to-v4l2.sh
sudo chmod +x /opt/subzero/picam-to-v4l2.sh
```

Test manually first (sensor-id 0 -> /dev/video10):

```bash
/opt/subzero/picam-to-v4l2.sh 0 /dev/video10 1280 720 30
```

In a second SSH session while that's running:

```bash
v4l2-ctl -d /dev/video10 --all   # expect Pixel Format 'YUYV'
```

Ctrl-C the manual run once confirmed, then install as a service per camera
using `systemd/picam-bridge@.service` from this repo (instance `%i` = sensor
id; it derives the output device `/dev/video1%i` itself):

```bash
sudo cp picam-bridge@.service /etc/systemd/system/picam-bridge@.service
sudo systemctl daemon-reload
sudo systemctl enable --now picam-bridge@0.service
sudo systemctl enable --now picam-bridge@1.service   # only if a second CSI camera is attached
```

Verify:

```bash
v4l2-ctl --list-devices
```

and confirm PhotonVision (UI → Cameras tab, or `curl -s http://localhost:5800`)
lists `/dev/video10` and `/dev/video11` as available camera sources.

If `nvargus-daemon` crashes ("No cameras available" after previously
working): `sudo systemctl restart nvargus-daemon`, wait ~15 s, retry.

## 6. Static IP 10.13.60.11/24, gateway 10.13.60.1 (10–15 min)

Detect the NIC name first — do not assume `eth0`:

```bash
ip link
```

Detect which network manager is actually active on this box, then use the
matching path below (not both):

```bash
systemctl is-active NetworkManager
ls /etc/netplan 2>/dev/null
```

**Path A — NetworkManager is active (`nmcli`):**

```bash
nmcli con show                    # find the connection name for your NIC — resolve on device, don't assume "Wired connection 1"
sudo nmcli con mod "<connection name from above>" \
  ipv4.addresses 10.13.60.11/24 \
  ipv4.gateway 10.13.60.1 \
  ipv4.dns "10.13.60.2 8.8.8.8" \
  ipv4.method manual
sudo nmcli con up "<connection name from above>"
```

**Path B — netplan is present (`/etc/netplan/*.yaml`):**

```bash
sudo tee /etc/netplan/99-jetson-static.yaml > /dev/null <<'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    <NIC name from `ip link`>:
      addresses: [10.13.60.11/24]
      routes: [{to: default, via: 10.13.60.1}]
      nameservers: {addresses: [10.13.60.2, 8.8.8.8]}
EOF
sudo netplan apply
```

Verify either path:

```bash
ip addr show <NIC name>     # expect 10.13.60.11/24
ping -c 4 10.13.60.1        # radio/gateway
ping -c 4 10.13.60.2        # RoboRIO (only responds once the RIO is powered and on the same network)
```

The PhotonVision UI should now be reachable at **http://10.13.60.11:5800**.

## 7. PhotonVision NetworkTables → RoboRIO (5 min)

In the PhotonVision web UI (http://10.13.60.11:5800) → **Settings** →
**Networking / Team Number**: set team number `1360` (this makes PhotonVision
connect out to the RoboRIO at `10.13.60.2` as an NT4 client — leave any
"NetworkTables Server" toggle OFF; PhotonVision is the client here, the
RoboRIO is the server per `comms-architecture` §1).

Confirm the RoboRIO side sees it: from a NetworkTables viewer (e.g. OutlineViewer,
or the robot's own NT client/log) pointed at `10.13.60.2:5810`, confirm
`/photonvision/...` topics appear once PhotonVision has a camera configured.

## 8. Upload the AprilTag field layout (validate first)

Validate the layout JSON **on the laptop** (not the Jetson) before uploading,
from the repo root:

```bash
uv run --project integration python docs/validate_layout.py docs/room-layout.template.json
```

Once it validates, upload it in the PhotonVision UI: **Settings** → **Import**
→ **AprilTag Field Layout**, selecting that same `docs/room-layout.template.json`
file. (The same file is also deployed with the robot code — this is just the
Jetson/PhotonVision-side copy.)

For each camera, create one pipeline: **AprilTag**, tag family **36h11**, tag
size **165.1 mm** (H-15), **3D mode** enabled.

## 9. Calibrate each camera (per camera, ~15–20 min)

Calibrate at the **exact resolution used in the bridge/webcam pipeline**
(1280x720 if you kept step 5's defaults) — this is mandatory for 3D/PnP
accuracy, a mismatched calibration resolution silently produces bad poses.

In the PhotonVision UI: **Cameras** tab → select the camera → **Calibration**
→ print the ChArUco board PhotonVision provides, capture **at least 12**
snapshots covering different angles/distances/corners of the frame, run the
calibration. Note the resolution you calibrated at in your session notes /
the H-18 table below.

## 10. Camera names must match Constants.java (H-14)

In the PhotonVision UI, name each camera exactly:

- `photoncamera_fl`
- `photoncamera_fr`

These strings are hard-coded in
`frc/src/main/java/frc/robot/Constants.java` (`VisionConstants.kFrontLeftCameraName`,
`kFrontRightCameraName`) — `new PhotonCamera("photoncamera_fl")` on the robot
will silently fail to find the camera if the UI name doesn't match exactly
(case-sensitive, no extra whitespace). If this robot ends up with a
different camera count/position than "front-left/front-right", that's a
`TODO(hardware)` for whoever owns H-14, not something to freelance here —
name the cameras to match whatever Constants.java says at the time you do
this step.

## 11. Performance tuning (5–10 min)

Detect the available power modes before picking one — don't assume an index:

```bash
sudo nvpmodel -q --verbose 2>/dev/null || cat /etc/nvpmodel.conf   # lists available modes and their indices for THIS device
nvpmodel -q                                                         # current mode
```

Set the maximum-performance mode (use the index the previous command
actually listed as max for this device, not a number from a different
Jetson model):

```bash
sudo nvpmodel -m <max-performance index from `nvpmodel -q --verbose` above>
sudo jetson_clocks
```

## 12. Final verification

- PhotonVision UI (http://10.13.60.11:5800) shows both cameras with live
  frames, fps and latency displayed.
- Each camera's pipeline is detecting tags when one is in view.
- The robot (sim or real, per whoever is testing that side) shows a fused
  pose update in its log / AdvantageKit output consistent with vision
  corrections.
- Reboot the Jetson (`sudo reboot`) and confirm, after it comes back:
  - `systemctl status photonvision.service` is active
  - `systemctl status picam-bridge@0.service` (and `@1`) are active, if used
  - the static IP is still `10.13.60.11/24`
  - the PhotonVision UI is reachable and cameras reappear without manual
    intervention

---

## H-18 fill-in table — fill in from the device, never guess

| Field | Value | Detect with |
|---|---|---|
| JetPack version | | `cat /etc/nv_tegra_release`; `dpkg -l \| grep nvidia-jetpack` |
| L4T version | | `cat /etc/nv_tegra_release` |
| Ubuntu version | | `lsb_release -a` or `cat /etc/os-release` |
| Kernel release | | `uname -r` |
| Free disk (at start) | | `df -h /` |
| Camera model(s) detected | | `v4l2-ctl --list-devices`; `ls /dev/video*` |
| NIC name | | `ip link` |
| Network manager in use | | `systemctl is-active NetworkManager`; `ls /etc/netplan` |
| nvpmodel mode set | | `nvpmodel -q --verbose`; `nvpmodel -q` |
| Static IP applied | | `ip addr show <NIC>` (should read 10.13.60.11/24 once step 6 is done) |
| PhotonVision version running | | PhotonVision UI footer, or `curl -s http://localhost:5800/api/...` version endpoint |
| Calibration resolution used | | PhotonVision UI → Cameras → Calibration (record what you actually captured at) |

Fill this table in as you go — it's the source of truth for anyone who
touches this Jetson after you.

## Open items for whoever picks this up next

- H-14 (camera count/mounts) and H-15/H-16 (tag IDs/room layout) are owned
  elsewhere (`vision-localization` decisions / `docs/room-layout.template.json`)
  — this runbook consumes those names/values but does not define them.
- `docs/validate_layout.py` referenced in step 8 is expected to exist at the
  repo root's `docs/` by the time step 8 is run; if it isn't there yet,
  that's a blocker for step 8 specifically, not for steps 0–7 or 9–12.
