#!/usr/bin/env python3
"""Make a spiral-in mowing mission from a lawn boundary.

Reads a QGroundControl .plan and takes the boundary from, in order of preference, the
first inclusion GeoFence polygon or the first Survey polygon. Offsets it inward one
lane at a time and joins the rings into one continuous path, so every corner is a
single pivot instead of the survey pattern's two pivots and a short leg per lane end.
Writes a new .plan to load and upload in QGroundControl's Plan view.

    pip install shapely
    python3 tools/mow_spiral.py "mow test.plan" --spacing 0.9 --out "mow spiral.plan"

Counter-clockwise by default, so every corner is a left turn. Use --cw for right turns.
The middle strip narrower than one lane is left uncut; mow it by hand or add a pass.
"""

import argparse
import json
import math
import sys

from shapely.geometry import LineString, Polygon
from shapely.geometry.polygon import orient

R_EARTH = 6378137.0
MIN_STEP_M = 0.05      # drop points closer together than this
MIN_TURN_DEG = 5.0     # drop points where the path barely changes direction
MIN_RING_AREA_M2 = 0.5


def load_boundary(plan):
    for fence in plan.get("geoFence", {}).get("polygons", []):
        if fence.get("inclusion", True) and len(fence.get("polygon", [])) >= 3:
            return fence["polygon"], "geofence", None
    for item in plan.get("mission", {}).get("items", []):
        if item.get("complexItemType") == "survey" and len(item.get("polygon", [])) >= 3:
            side = (item.get("TransectStyleComplexItem", {}).get("CameraCalc", {})
                    .get("AdjustedFootprintSide"))
            return item["polygon"], "survey", side
    sys.exit("No inclusion GeoFence polygon or Survey polygon found in the plan.")


def to_local(points, lat0, lon0):
    k = math.cos(math.radians(lat0))
    return [(math.radians(lon - lon0) * R_EARTH * k, math.radians(lat - lat0) * R_EARTH)
            for lat, lon in points]


def to_global(xy, lat0, lon0):
    k = math.cos(math.radians(lat0))
    return [(lat0 + math.degrees(y / R_EARTH), lon0 + math.degrees(x / (R_EARTH * k)))
            for x, y in xy]


def rings(boundary, spacing, clockwise, edge):
    """Inward offset rings, outermost first, each a list of (x, y) without the closing point."""
    out, k = [], 0
    while True:
        inset = boundary.buffer(-(edge + k * spacing), join_style="mitre", mitre_limit=5)
        if inset.is_empty or inset.area < MIN_RING_AREA_M2:
            break
        parts = list(inset.geoms) if inset.geom_type == "MultiPolygon" else [inset]
        if len(parts) > 1:
            print(f"Ring {k + 1} splits into {len(parts)} pieces; keeping the largest.")
        piece = max(parts, key=lambda p: p.area)
        piece = orient(piece, sign=-1.0 if clockwise else 1.0)
        out.append(list(piece.exterior.coords)[:-1])
        k += 1
    return out


def rotate_to_nearest(ring, point):
    i = min(range(len(ring)), key=lambda j: math.dist(ring[j], point))
    return ring[i:] + ring[:i]


def simplify(path):
    pts = [path[0]]
    for p in path[1:]:
        if math.dist(p, pts[-1]) >= MIN_STEP_M:
            pts.append(p)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = out[-1], pts[i], pts[i + 1]
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        turn = abs((math.degrees(h2 - h1) + 180) % 360 - 180)
        if turn >= MIN_TURN_DEG:
            out.append(b)
    out.append(pts[-1])
    return out


def build_path(ring_list, spacing):
    """True spiral: each ring's last side stops one lane short of its first corner. That
    point lies on the line of the next ring's first side, so a single turn joins them."""
    path, start = [], None
    for k, ring in enumerate(ring_list):
        if start is not None:
            ring = rotate_to_nearest(ring, start)
        start = ring[0]
        path.extend(ring if k == 0 else ring[1:])
        last, first = ring[-1], ring[0]
        edge = math.dist(last, first)
        if k + 1 < len(ring_list) and edge > spacing:
            t = (edge - spacing) / edge
            path.append((last[0] + (first[0] - last[0]) * t, last[1] + (first[1] - last[1]) * t))
        else:
            path.append(first)
    return simplify(path)


def corner_count(path, min_deg=20):
    n = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        if abs((math.degrees(h2 - h1) + 180) % 360 - 180) >= min_deg:
            n += 1
    return n


def mission_item(seq, lat, lon):
    return {"autoContinue": True, "command": 16, "doJumpId": seq, "frame": 3,
            "params": [0, 0, 0, None, lat, lon, 0], "type": "SimpleItem"}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--spacing", type=float,
                    help="lane spacing in metres (cut width minus overlap). "
                         "Default: the survey's lane spacing")
    ap.add_argument("--edge", type=float,
                    help="boundary to first pass centre line, metres. Draw the boundary on the "
                         "fence and use half the mower width plus clearance. Default: spacing/2")
    ap.add_argument("--cw", action="store_true", help="clockwise, so right-hand corners")
    ap.add_argument("--out", help="output .plan (default: <input> spiral.plan)")
    a = ap.parse_args()

    plan = json.load(open(a.plan))
    pts, source, survey_spacing = load_boundary(plan)
    spacing = a.spacing or survey_spacing
    if not spacing or spacing <= 0:
        sys.exit("Pass --spacing (no survey lane spacing to fall back on).")

    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    boundary = Polygon(to_local(pts, lat0, lon0))
    if not boundary.is_valid:
        boundary = boundary.buffer(0)

    edge = a.edge if a.edge is not None else spacing / 2
    ring_list = rings(boundary, spacing, a.cw, edge)
    if not ring_list:
        sys.exit("Boundary is too small for that spacing.")
    path = build_path(ring_list, spacing)

    length = LineString(path).length
    print(f"Boundary from {source}, {boundary.area:.0f} m2, spacing {spacing:.2f} m")
    print(f"{len(ring_list)} rings, {len(path)} waypoints, {length:.0f} m, "
          f"{corner_count(path)} corners, all {'right' if a.cw else 'left'} turns")

    geo = to_global(path, lat0, lon0)
    out = dict(plan)
    out["mission"] = dict(plan["mission"])
    out["mission"]["items"] = [mission_item(i + 1, lat, lon) for i, (lat, lon) in enumerate(geo)]
    out["mission"]["plannedHomePosition"] = [geo[0][0], geo[0][1], 0]
    dest = a.out or a.plan.rsplit(".plan", 1)[0] + " spiral.plan"
    with open(dest, "w") as f:
        json.dump(out, f, indent=4)
    print(f"Wrote {dest}")


if __name__ == "__main__":
    main()
