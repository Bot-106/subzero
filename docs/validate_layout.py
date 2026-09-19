#!/usr/bin/env python3
"""Validate a Subzero room-layout AprilTagFieldLayout JSON file.

Usage:
    uv run --project integration python docs/validate_layout.py docs/room-layout.template.json

(run from the repo root). Stdlib only (json, math, sys, argparse) — no
project dependencies are actually required, but the `uv run --project
integration` wrapper guarantees a modern Python (never the system 3.8).

Checks performed (see docs/room-layout.md for the coordinate/quaternion
convention this enforces):
  - top level has a non-empty "tags" list and a "field" object with
    numeric length/width > 0
  - each tag has an integer ID >= 1, a numeric translation {x, y, z}, and a
    numeric rotation.quaternion {W, X, Y, Z}
  - each quaternion is a unit quaternion (|norm - 1| < 1e-3)
  - no duplicate tag IDs
  - every tag's translation lies inside the field box
    (0 <= x <= length, 0 <= y <= width, z >= 0)

With --strict, additionally *warns* (does not fail) when:
  - a tag is not flush on a wall (none of x in {0, length} or y in {0,
    width}, within a 5 cm tolerance)
  - the tag's yaw (derived from the quaternion) does not point into the
    room for the wall it appears to be on

Exit code 0 and a one-line summary per tag (ID, x, y, z, yaw deg) printed
on success; exit code 1 and every problem listed on failure.
"""

import argparse
import json
import math
import sys

WALL_TOL_M = 0.05  # 5 cm, for --strict "on a wall" check
QUAT_UNIT_TOL = 1e-3


def fail(problems):
    for p in problems:
        print(f"PROBLEM: {p}")
    print(f"FAILED — {len(problems)} problem(s)")
    return 1


def is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def quat_to_yaw_deg(w, x, y, z):
    # Yaw about +Z from a unit quaternion (WPILib/ROS convention).
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main():
    parser = argparse.ArgumentParser(
        description="Validate a Subzero room-layout AprilTagFieldLayout JSON file."
    )
    parser.add_argument("path", help="path to the room-layout JSON file")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="also warn (not fail) on off-wall tags / yaw not pointing into the room",
    )
    args = parser.parse_args()

    problems = []
    warnings = []

    try:
        with open(args.path, "r") as f:
            raw = f.read()
    except OSError as e:
        return fail([f"cannot open {args.path}: {e}"])

    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        return fail([f"invalid JSON: {e}"])

    if not isinstance(data, dict):
        return fail(["top level must be a JSON object"])

    # --- field ---
    field = data.get("field")
    length = width = None
    if not isinstance(field, dict):
        problems.append('top-level "field" object is missing or not an object')
    else:
        length = field.get("length")
        width = field.get("width")
        if not is_number(length) or length <= 0:
            problems.append(f'"field.length" must be a positive number, got {length!r}')
            length = None
        if not is_number(width) or width <= 0:
            problems.append(f'"field.width" must be a positive number, got {width!r}')
            width = None

    # --- tags ---
    tags = data.get("tags")
    if not isinstance(tags, list) or len(tags) == 0:
        problems.append('top-level "tags" must be a non-empty list')
        tags = []

    seen_ids = set()
    summaries = []

    for i, tag in enumerate(tags):
        loc = f"tags[{i}]"
        if not isinstance(tag, dict):
            problems.append(f"{loc}: tag entry must be an object")
            continue

        tag_id = tag.get("ID")
        if not isinstance(tag_id, int) or isinstance(tag_id, bool) or tag_id < 1:
            problems.append(f"{loc}: ID must be an integer >= 1, got {tag_id!r}")
            tag_id = None
        elif tag_id in seen_ids:
            problems.append(f"{loc}: duplicate ID {tag_id}")
        else:
            seen_ids.add(tag_id)

        pose = tag.get("pose")
        if not isinstance(pose, dict):
            problems.append(f"{loc} (ID {tag_id}): missing or invalid \"pose\" object")
            continue

        translation = pose.get("translation")
        x = y = z = None
        if not isinstance(translation, dict):
            problems.append(f"{loc} (ID {tag_id}): missing or invalid \"pose.translation\"")
        else:
            x, y, z = translation.get("x"), translation.get("y"), translation.get("z")
            for name, v in (("x", x), ("y", y), ("z", z)):
                if not is_number(v):
                    problems.append(
                        f"{loc} (ID {tag_id}): translation.{name} must be numeric, got {v!r}"
                    )

        rotation = pose.get("rotation")
        quat = None
        w = qx = qy = qz = None
        if not isinstance(rotation, dict) or not isinstance(rotation.get("quaternion"), dict):
            problems.append(f"{loc} (ID {tag_id}): missing or invalid \"pose.rotation.quaternion\"")
        else:
            quat = rotation["quaternion"]
            w, qx, qy, qz = quat.get("W"), quat.get("X"), quat.get("Y"), quat.get("Z")
            for name, v in (("W", w), ("X", qx), ("Y", qy), ("Z", qz)):
                if not is_number(v):
                    problems.append(
                        f"{loc} (ID {tag_id}): rotation.quaternion.{name} must be numeric, got {v!r}"
                    )

        all_numeric = all(is_number(v) for v in (x, y, z, w, qx, qy, qz))
        if not all_numeric:
            continue

        # unit quaternion check
        norm = math.sqrt(w * w + qx * qx + qy * qy + qz * qz)
        if abs(norm - 1.0) >= QUAT_UNIT_TOL:
            problems.append(
                f"{loc} (ID {tag_id}): quaternion is not unit-length "
                f"(|norm - 1| = {abs(norm - 1.0):.6f} >= {QUAT_UNIT_TOL})"
            )

        # field-box check
        if length is not None and width is not None:
            if not (0 <= x <= length):
                problems.append(
                    f"{loc} (ID {tag_id}): x={x} is outside field bounds [0, {length}]"
                )
            if not (0 <= y <= width):
                problems.append(
                    f"{loc} (ID {tag_id}): y={y} is outside field bounds [0, {width}]"
                )
            if z < 0:
                problems.append(f"{loc} (ID {tag_id}): z={z} must be >= 0")

        yaw = quat_to_yaw_deg(w, qx, qy, qz)
        summaries.append((tag_id, x, y, z, yaw))

        if args.strict and length is not None and width is not None:
            on_wall = (
                abs(x - 0.0) <= WALL_TOL_M
                or abs(x - length) <= WALL_TOL_M
                or abs(y - 0.0) <= WALL_TOL_M
                or abs(y - width) <= WALL_TOL_M
            )
            if not on_wall:
                warnings.append(
                    f"{loc} (ID {tag_id}): not within {WALL_TOL_M} m of any wall "
                    f"(x={x}, y={y}, field {length}x{width})"
                )
            else:
                # Determine which wall and whether yaw points inward (+X of the
                # tag pose should point into the room).
                dirx, diry = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                expected = None
                if abs(x - 0.0) <= WALL_TOL_M:
                    expected = (1.0, 0.0)  # +X wall -> face +X
                elif abs(x - length) <= WALL_TOL_M:
                    expected = (-1.0, 0.0)  # far X wall -> face -X
                elif abs(y - 0.0) <= WALL_TOL_M:
                    expected = (0.0, 1.0)  # y=0 wall -> face +Y
                elif abs(y - width) <= WALL_TOL_M:
                    expected = (0.0, -1.0)  # far Y wall -> face -Y
                if expected is not None:
                    dot = dirx * expected[0] + diry * expected[1]
                    if dot < 0.7071:  # more than 45 deg off expected inward normal
                        warnings.append(
                            f"{loc} (ID {tag_id}): yaw={yaw:.1f} deg does not point "
                            f"into the room for its wall (expected facing "
                            f"{expected})"
                        )

    if problems:
        return fail(problems)

    for tag_id, x, y, z, yaw in summaries:
        print(f"ID {tag_id}: x={x:.4f} y={y:.4f} z={z:.4f} yaw={yaw:.1f} deg")

    for w in warnings:
        print(f"WARNING: {w}")

    print(f"OK — {len(summaries)} tag(s) valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
