#!/usr/bin/env python3
"""Generate frc/src/main/deploy/room-layout.json (WPILib AprilTagFieldLayout) from the room measurements.

Run from frc/:  python3 tools/gen_room_layout.py            (writes the JSON + prints the table)
                python3 tools/gen_room_layout.py --check    (prints only)

FRAME (WPILib, right-handed, Z up):
  +X points toward the FRONT wall (the direction "forward" on the joystick drives with no DS alliance),
  +Y points toward the LEFT wall, origin = the (virtual) REAR-RIGHT corner at floor level.
  Only distances from the FRONT and LEFT walls were measured, so ROOM_LENGTH_M / ROOM_WIDTH_M below are
  PLACEHOLDERS that merely translate the whole layout (front wall plane x = L, left wall plane y = W).
  Relative tag geometry — the only thing localization needs — is exact regardless of L and W.
  A tag's +X axis is its normal pointing INTO the room (WPILib convention): front-wall tags yaw 180°,
  left-wall tags yaw −90°.

MEASUREMENTS (2026-09-20, in units of one 3D-printed "apriltag sheet" = tag + white border):
  * sheets are laid edge-to-edge along a wall with every OTHER sheet omitted → adjacent sheets' near
    edges are exactly one sheet apart → centre pitch = 2 sheets
  * LEFT wall, reading from the front-left corner toward the rear: IDs 5, 6, 8, 9; the first sheet's
    edge is IN the corner (no offset)
  * FRONT wall, reading from the front-left corner toward the right: IDs 2, 3, 4; the first sheet's
    left edge is 3.27 sheets from the corner
  * every sheet's bottom edge is 4.6 sheets above the floor → centre at 5.1 sheets
"""
import argparse, json, math, os

# ── TODO(hardware) H-15 — sheet + tag dimensions ───────────────────────────────────────────────
SHEET_M = 0.2667        # measured "approx 27 × 27 cm"; the common 3D-printed 36h11 plate ("Full Size" Printables 1137835 =
                        # AndyMark AprilTag plate) is 10.5 in = 266.7 mm with the true-scale tag centred. CONFIRM with a tape measure;
                        # every distance below scales with it.
TAG_BLACK_SQUARE_M = 0.1651  # 36h11 black square (FRC standard 6.5 in; the 266.7 mm plate is true scale) — PhotonVision "tag size",
                             # NOT in this JSON (the layout format has no size field). If the print was scaled to fill 270 mm the
                             # black square would be ≈ 216 mm — measure one with calipers.
SHEET_THICKNESS_M = 0.0 # print thickness: tag face stands this far off the wall (into the room)

# ── TODO(hardware) H-16 — placeholder room box (translation only, see FRAME above) ─────────────
ROOM_LENGTH_M = 6.0     # rear wall x = 0 … front wall x = L
ROOM_WIDTH_M = 4.0      # right wall y = 0 … left wall y = W

# ── measurements in sheets ─────────────────────────────────────────────────────────────────────
LEFT_WALL_IDS = [5, 6, 8, 9]       # from the front-left corner toward the rear
LEFT_WALL_FIRST_EDGE_SHEETS = 0.0  # first sheet edge in the corner
FRONT_WALL_IDS = [2, 3, 4]         # from the front-left corner toward the right
FRONT_WALL_FIRST_EDGE_SHEETS = 3.27
CENTER_PITCH_SHEETS = 2.0          # edge-to-edge chain with every other sheet omitted
BOTTOM_EDGE_HEIGHT_SHEETS = 4.6
CENTER_HEIGHT_SHEETS = BOTTOM_EDGE_HEIGHT_SHEETS + 0.5


def quat_yaw(yaw_rad):
    return {"W": math.cos(yaw_rad / 2), "X": 0.0, "Y": 0.0, "Z": math.sin(yaw_rad / 2)}


def tag(tag_id, x, y, z, yaw_deg):
    return {"ID": tag_id, "pose": {"translation": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4)},
                                   "rotation": {"quaternion": {k: round(v, 10) for k, v in quat_yaw(math.radians(yaw_deg)).items()}}}}


def build():
    S = SHEET_M
    L, W = ROOM_LENGTH_M, ROOM_WIDTH_M
    z = CENTER_HEIGHT_SHEETS * S
    tags = []
    # LEFT wall: plane y = W, faces −Y (yaw −90°); distance from the FRONT wall along −X
    for k, tid in enumerate(LEFT_WALL_IDS):
        d = (LEFT_WALL_FIRST_EDGE_SHEETS + 0.5 + CENTER_PITCH_SHEETS * k) * S
        tags.append(tag(tid, L - d, W - SHEET_THICKNESS_M, z, -90.0))
    # FRONT wall: plane x = L, faces −X (yaw 180°); distance from the LEFT wall along −Y
    for k, tid in enumerate(FRONT_WALL_IDS):
        d = (FRONT_WALL_FIRST_EDGE_SHEETS + 0.5 + CENTER_PITCH_SHEETS * k) * S
        tags.append(tag(tid, L - SHEET_THICKNESS_M, W - d, z, 180.0))
    tags.sort(key=lambda t: t["ID"])
    return {"tags": tags, "field": {"length": L, "width": W}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="print only, do not write")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "src", "main", "deploy", "room-layout.json"))
    a = ap.parse_args()
    layout = build()
    print(f"sheet {SHEET_M} m  tag {TAG_BLACK_SQUARE_M} m  room {ROOM_LENGTH_M} x {ROOM_WIDTH_M} m (placeholder box)  centre z {CENTER_HEIGHT_SHEETS*SHEET_M:.3f} m")
    print(f"{'ID':>3} {'wall':>5} {'x':>7} {'y':>7} {'z':>6} {'yaw':>6}   from-front  from-left")
    for t in layout["tags"]:
        p = t["pose"]["translation"]; q = t["pose"]["rotation"]["quaternion"]
        yaw = math.degrees(2 * math.atan2(q["Z"], q["W"]))
        wall = "left" if t["ID"] in LEFT_WALL_IDS else "front"
        print(f"{t['ID']:>3} {wall:>5} {p['x']:7.3f} {p['y']:7.3f} {p['z']:6.3f} {yaw:6.1f}   {ROOM_LENGTH_M-p['x']:9.3f}  {ROOM_WIDTH_M-p['y']:9.3f}")
    if not a.check:
        out = os.path.abspath(a.out)
        with open(out, "w") as f:
            json.dump(layout, f, indent=2)
            f.write("\n")
        print("wrote", out)


if __name__ == "__main__":
    main()
