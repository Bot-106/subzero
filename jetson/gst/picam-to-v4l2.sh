#!/usr/bin/env bash
# picam-to-v4l2.sh — bridge one Jetson CSI (Argus) camera into a v4l2loopback
# device so PhotonVision can open it like any other /dev/videoN.
#
# Usage:
#   picam-to-v4l2.sh [SENSOR_ID] [OUTPUT_DEVICE] [WIDTH] [HEIGHT] [FPS]
#
#   SENSOR_ID      Argus sensor-id as reported by `v4l2-ctl --list-devices`
#                  or `ls /dev/video*` for the raw CSI nodes. Default: 0
#   OUTPUT_DEVICE  v4l2loopback device node to write YUY2 frames into.
#                  Default: /dev/video10
#   WIDTH          Frame width.  Default: 1280
#   HEIGHT         Frame height. Default: 720
#   FPS            Frame rate.   Default: 30
#
# Example (second CSI camera -> loopback node 11):
#   picam-to-v4l2.sh 1 /dev/video11 1280 720 30
#
# This script is invoked by systemd/picam-bridge@.service, which passes the
# systemd instance name (%i) as SENSOR_ID and derives OUTPUT_DEVICE from it
# (see that unit file). It can also be run by hand for debugging — Ctrl-C to
# stop, and check the second terminal with:
#   v4l2-ctl -d /dev/video1X --all
# which should report Pixel Format 'YUYV' once frames are flowing.
#
# NOTE: the pipeline outputs format=YUY2 (not UYVY). cscore/PhotonVision does
# not recognize UYVY as a valid capture mode — do not change this.
#
# Requires: gst-launch-1.0 with the nvarguscamerasrc / nvvidconv plugins
# (part of the Jetson L4T multimedia API) and a v4l2loopback device already
# created (see runbook.md step 4) at OUTPUT_DEVICE.

set -euo pipefail

SENSOR="${1:-0}"
DEV="${2:-/dev/video10}"
WIDTH="${3:-1280}"
HEIGHT="${4:-720}"
FPS="${5:-30}"

if [[ ! -e "${DEV}" ]]; then
  echo "picam-to-v4l2.sh: ${DEV} does not exist yet." >&2
  echo "Create the v4l2loopback device first (runbook.md step 4), e.g.:" >&2
  echo "  sudo modprobe v4l2loopback devices=2 video_nr=10,11 exclusive_caps=1 card_label=\"picam0,picam1\"" >&2
  exit 1
fi

echo "picam-to-v4l2.sh: sensor-id=${SENSOR} -> ${DEV} (${WIDTH}x${HEIGHT}@${FPS})" >&2

exec gst-launch-1.0 -e \
  nvarguscamerasrc sensor-id="${SENSOR}" ! \
  "video/x-raw(memory:NVMM),width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
  nvvidconv ! \
  "video/x-raw,format=YUY2" ! \
  v4l2sink device="${DEV}" sync=false
