#!/usr/bin/env python3
"""Analyze an OBJ road mesh and suggest spawn points.

Reads vertex data from a Wavefront .obj file (expected to be a road
surface mesh), finds straight road sections, and prints candidate spawn
locations with headings.

Usage:
    python3 tools/pick_spawn.py level/milford_asphault.obj

The output includes:
  - Mesh bounding box and vertex/face counts
  - Road density analysis by X and Y buckets
  - Candidate spawn points with computed heading angles
"""

import math
import sys
from collections import defaultdict


def load_vertices(path: str) -> list[tuple[float, float, float]]:
    """Read vertex positions from an OBJ file."""
    verts = []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return verts


def count_faces(path: str) -> int:
    """Count faces in an OBJ file."""
    n = 0
    with open(path) as f:
        for line in f:
            if line.startswith("f "):
                n += 1
    return n


def bounding_box(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def find_straight_sections(verts, bucket_size=5.0, min_verts=50, min_span=200.0):
    """Group vertices into Y-buckets and find sections with wide X span."""
    buckets = defaultdict(list)
    for x, y, z in verts:
        by = round(y / bucket_size) * bucket_size
        buckets[by].append((x, y, z))

    sections = []
    for by in sorted(buckets.keys()):
        pts = buckets[by]
        if len(pts) < min_verts:
            continue
        xs = [p[0] for p in pts]
        x_span = max(xs) - min(xs)
        if x_span < min_span:
            continue
        avg_z = sum(p[2] for p in pts) / len(pts)
        sections.append((by, len(pts), x_span, avg_z))
    return sections


def find_spawn_candidates(verts, target_ys=None, lookahead=20.0):
    """Find spawn point candidates at specified Y values.

    For each target Y, finds road center and computes heading by
    comparing to a point `lookahead` meters ahead.
    """
    if target_ys is None:
        # Pick some Y values from the straight sections
        sections = find_straight_sections(verts)
        if not sections:
            return []
        # Pick sections at ~25%, 50%, 75% of the Y range
        ys = [s[0] for s in sections]
        target_ys = [ys[len(ys) // 4], ys[len(ys) // 2], ys[3 * len(ys) // 4]]

    candidates = []
    for ty in target_ys:
        near = [(x, y, z) for x, y, z in verts if abs(y - ty) < 5]
        if len(near) < 4:
            continue

        cx = sum(p[0] for p in near) / len(near)
        cy = sum(p[1] for p in near) / len(near)
        cz = sum(p[2] for p in near) / len(near)

        ahead = [(x, y, z) for x, y, z in verts if abs(y - (ty + lookahead)) < 5]
        if not ahead:
            continue

        ax = sum(p[0] for p in ahead) / len(ahead)
        ay = sum(p[1] for p in ahead) / len(ahead)

        heading = math.atan2(ay - cy, ax - cx) * 180.0 / math.pi
        spawn_z = cz + 1.0  # 1m above road surface
        candidates.append((cx, cy, spawn_z, cz, heading))

    return candidates


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <road.obj> [bucket_size]")
        sys.exit(1)

    path = sys.argv[1]
    bucket_size = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    print(f"Analyzing: {path}")
    print()

    verts = load_vertices(path)
    faces = count_faces(path)
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounding_box(verts)

    print(f"Vertices: {len(verts):,}")
    print(f"Faces:    {faces:,}")
    print(f"Bounds:   X[{xmin:.1f}, {xmax:.1f}]  ({xmax-xmin:.0f} m)")
    print(f"          Y[{ymin:.1f}, {ymax:.1f}]  ({ymax-ymin:.0f} m)")
    print(f"          Z[{zmin:.1f}, {zmax:.1f}]  ({zmax-zmin:.0f} m elevation)")
    print()

    # Centroid
    cx = sum(v[0] for v in verts) / len(verts)
    cy = sum(v[1] for v in verts) / len(verts)
    cz = sum(v[2] for v in verts) / len(verts)
    print(f"Centroid: ({cx:.1f}, {cy:.1f}, {cz:.1f})")
    print()

    # Straight sections
    sections = find_straight_sections(verts, bucket_size)
    print(f"Straight road sections (Y-bucket, vert count, X span, avg Z):")
    for by, count, span, avg_z in sections[:20]:
        print(f"  Y~{by:7.0f}: {count:4d} verts, X span={span:.0f} m, avg Z={avg_z:.1f}")
    if len(sections) > 20:
        print(f"  ... ({len(sections) - 20} more)")
    print()

    # Spawn candidates
    candidates = find_spawn_candidates(verts)
    print("Suggested spawn points:")
    print(f"  {'X':>10s} {'Y':>10s} {'Z (spawn)':>10s} {'Z (road)':>10s} {'Heading':>8s}")
    for cx, cy, sz, rz, heading in candidates:
        print(f"  {cx:10.1f} {cy:10.1f} {sz:10.1f} {rz:10.1f} {heading:8.1f}°")

    if candidates:
        best = candidates[len(candidates) // 2]
        print()
        print("Suggested level JSON spawn block:")
        print(f'    "spawn": {{')
        print(f'        "x": {best[0]:.1f},')
        print(f'        "y": {best[1]:.1f},')
        print(f'        "z": {best[2]:.1f},')
        print(f'        "yaw_deg": {best[4]:.1f}')
        print(f"    }}")


if __name__ == "__main__":
    main()
