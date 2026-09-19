# Room layout — coordinate convention, tape-measure procedure, upload

This document explains the coordinate/quaternion convention behind
`docs/room-layout.template.json` (and its live copy,
`frc/src/main/deploy/room-layout.json`), how to physically measure tags into
that convention, and how to publish a new layout safely. It does not restate
the contract in `docs/contracts.md` (read-only, not duplicated here).

## 1. Coordinate frame

- **Origin**: one room corner, at **floor level** (z = 0).
- **+X**: along the long wall of the room, away from the origin corner.
- **+Y**: 90° left of +X (right-handed: +Z = +X × +Y), i.e. along the short
  wall.
- **+Z**: up, floor to ceiling.
- The frame is right-handed and matches WPILib's field-coordinate convention
  (`AprilTagFieldLayout` origin is always the identity pose in this convention
  — no separate "origin" field exists in the JSON).
- `field.length` = room extent along X (metres). `field.width` = room extent
  along Y (metres). For Subzero's room this weekend: **8.0 m × 6.0 m**. These
  numbers are only used as a bounding box for validation and PhotonVision's
  UI — they have no effect on MultiTag PnP accuracy, and there is no required
  minimum size.

## 2. Tag pose convention

Each tag pose is `{translation: {x, y, z}, rotation: {quaternion: {W, X, Y, Z}}}`.

- **translation** is the tag's **printed-face centre**, in metres, in the
  room frame above.
- **rotation** encodes which way the tag *faces*. By WPILib convention, a tag
  pose's **+X axis is the tag's normal, pointing out of the tag face** — i.e.
  the direction a camera must approach from to read it. For a wall-mounted
  tag this is simply "the direction pointing away from the wall, into the
  room."
- Tags on Subzero's four walls only ever need a **pure yaw** rotation about
  +Z (no pitch/roll — tags are mounted vertically, flush to the wall). For a
  yaw angle θ (measured the usual way, counter-clockwise from +X):

  ```
  W = cos(θ / 2)
  X = 0
  Y = 0
  Z = sin(θ / 2)
  ```

  This is always a **unit quaternion** (`W² + X² + Y² + Z² = 1`) since
  `cos²(θ/2) + sin²(θ/2) = 1`.

### The four wall cases

| Wall the tag is mounted on | Direction tag must face (into room) | yaw θ | W | Z |
|---|---|---|---|---|
| y = 0 (wall running along +X, at the origin end of Y) | +Y | 90° | cos45° = 0.70710678 | sin45° = 0.70710678 |
| x = field.length (wall running along +Y, at the far end of X) | −X | 180° | cos90° = 0 | sin90° = 1 |
| y = field.width (wall running along +X, at the far end of Y) | −Y | −90° | cos(−45°) = 0.70710678 | sin(−45°) = −0.70710678 |
| x = 0 (wall running along +Y, at the origin end of X) | +X | 0° | cos0° = 1 | sin0° = 0 |

These four cases are exactly what `room-layout.template.json` encodes today:

| ID | wall | (x, y, z) | yaw | W | Z |
|---|---|---|---|---|---|
| 1 | y = 0 | (4.0, 0.0, 1.0) | +90° | 0.7071067811865476 | 0.7071067811865476 |
| 2 | x = 8 (length) | (8.0, 3.0, 1.0) | 180° | 0.0 | 1.0 |
| 3 | y = 6 (width) | (4.0, 6.0, 1.0) | −90° | 0.7071067811865476 | −0.7071067811865476 |
| 4 | x = 0 | (0.0, 3.0, 1.0) | 0° | 1.0 | 0.0 |

If you ever need a tag at some other yaw (e.g. a tag angled into a corner,
not flush on a wall), compute θ as the compass bearing of the direction the
tag faces (counter-clockwise from +X, degrees), then plug into the formula
above. `--strict` validation (§5) only *warns*, it does not fail, if a tag
isn't flush on a wall — angled/corner tags are legal.

## 3. Tag IDs and placement (H-15 / H-16)

Per `00-brief/hardware-todos.md`:

- **H-16** — room is **8 m × 6 m**, one location tag centred on each of the
  four walls, tag centre height **z = 1.0 m** off the floor.
- **H-15** — AprilTag family **36h11**. Location tags are **165.1 mm**
  (6.5 in) and use **IDs 1–12** (this weekend only IDs 1–4 are populated, one
  per wall). Smaller **63.5 mm** object tags would use **IDs 20–29**, but
  per the approval-gate decision (Q4 in `outputs/DECISIONS.md §5`), object
  tags are **out of scope this weekend** — do not add IDs 20–29 to this file.

To add a fifth (or later) location tag: pick an unused ID in 1–12, tape-measure
it per §4 below, add an entry to the `tags` array in **both** copies of the
file (§5), and re-run the validator. There's no limit on tag count beyond
unique IDs and the field bounding box.

## 4. Tape-measuring a tag into this convention

For each tag on the wall:

1. **Which wall → which yaw.** Identify which of the four walls (or an
   angled placement) the tag is mounted on and use the table in §2 to get
   the yaw / (W, Z) pair. Double-check the tag is glued/mounted **facing
   into the room**, not into the wall.
2. **Distance along the wall.** Measure from the origin corner, along the
   wall the tag sits on, to the point on the floor directly below the tag's
   centre. That gives you the x (for tags on the x=0/x=length walls, this is
   the y coordinate; for tags on the y=0/y=width walls, this is the x
   coordinate) — i.e. whichever coordinate varies along that wall.
3. **The other (fixed) coordinate** is 0 or the field length/width, since the
   tag sits flush on that wall (x=0 or x=length for the two "end" walls; y=0
   or y=width for the two "side" walls).
4. **Centre height z.** Measure from the floor straight up to the centre of
   the printed tag square (not the top or the white border edge). Record it
   in **metres**, not millimetres (see §6 — this is the single most common
   mistake).
5. Enter the four numbers `(x, y, z)` and the `(W, Z)` pair (X = Y = 0
   always, for wall-mounted tags) into the JSON.

## 5. Upload procedure

1. **Edit** `docs/room-layout.template.json` only (this is the source of
   truth / working copy for docs-side edits).
2. **Validate** it:
   ```
   uv run --project integration python docs/validate_layout.py docs/room-layout.template.json
   ```
   Fix every reported problem before proceeding — do not upload a file the
   validator rejects.
3. **Upload to PhotonVision**: PhotonVision UI → Settings → AprilTag Layout
   → Import → select the validated file. PhotonVision stores whatever you
   upload **raw, with no validation of its own** — this is why the validator
   in this repo is the only safety net; always run it immediately before
   every upload, even for a one-line change.
4. **Copy the same file** to `frc/src/main/deploy/room-layout.json` (byte
   for byte — e.g. `cp docs/room-layout.template.json
   frc/src/main/deploy/room-layout.json`). **Both files must stay
   identical.** The robot code loads the deploy copy via
   `AprilTagFieldLayout` at boot; PhotonVision uses its own uploaded copy for
   MultiTag PnP on the coprocessor. If they drift, the robot's fused pose
   estimate and PhotonVision's tag-relative geometry disagree.
5. **Redeploy robot code** after changing the deploy copy:
   `cd frc && ./gradlew deploy`. A changed `room-layout.json` is not picked
   up until the next deploy (it's a deploy-directory resource, not read from
   disk live).

## 6. Common mistakes

- **Tag rotated 180° from intended.** Yaw sign errors are the most common
  bug — e.g. entering the yaw for "the wall the tag is on" instead of "the
  direction the tag faces." A tag on the x=0 wall facing *into* the wall
  (yaw 180°, W=0,Z=1) instead of into the room (yaw 0°, W=1,Z=0) will make
  PhotonVision/PhotonLib solve a pose that's mirrored front-to-back —
  `alignToTag` will drive the robot away from the tag or spin in place.
  Always sanity check: standing at the tag facing the room interior, the
  +X arrow described in §2 should point at your chest.
- **Wrong wall sign** (using `x = 0` when the tag is actually on `x =
  field.length`, or the y equivalent). This silently passes the "unit
  quaternion" check but fails the `--strict` "on a wall" warning and, more
  importantly, positions the tag on the wrong side of the room entirely.
- **mm vs m.** Every number in this file is in **metres**. A tag centre
  height of "1000" (mm, meant to be 1.0 m) will fail the field-bounding-box
  check for any z clearly outside a sane room height, but a smaller mm/m
  slip (e.g. entering `1.5` when you meant `150` mm = 0.15 m) will pass
  validation silently and just be wrong — always re-derive your tape
  measurement in metres before typing it in, and sanity-check against the
  known room dimensions (8 m × 6 m) before uploading.
- **Editing only one of the two copies.** See §5 step 4 — always copy
  `docs/room-layout.template.json` → `frc/src/main/deploy/room-layout.json`
  after every edit and before every deploy.
