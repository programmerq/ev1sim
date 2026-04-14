#!/usr/bin/env python3
"""
Post-process the Blender-exported OBJ to fix coordinates, materials, and
split lamp groups.

Fixes:
  1. Vertex positions: Blender world coords -> Chrono body-frame coords.
  2. MTL materials: Blender defaults -> Phong-friendly values with correct
     transparency for lamp classification.
  3. Reflectors: redglass set to opaque (d=1.0) so VehicleLights doesn't
     classify reflector faces as lamps.
  4. Front signals: signal_front_left/right split into turn/marker variants
     by face centroid X position (X >= 1.9 = turn, X < 1.9 = marker).
  5. Headlamps: headlamp_left/right split into hi/lo beam by face centroid Y
     position.  Inner half (closer to centerline) = hi beam projector,
     outer half = lo beam.
  6. Rear side markers: redglass faces at X >= -1.9 (side of bumper) split
     into side_marker_rear_left/right with d=0.35.  Remaining redglass
     (rearmost reflectors) stays opaque d=1.0.

Usage: python3 fix_obj_coords.py [ev1model_dir]
       Default ev1model_dir is the directory containing this script.
"""

import os
import shutil
import sys

# ---------------------------------------------------------------------------
# Transform constants (same as export_meshes.py)
# ---------------------------------------------------------------------------
WHEELBASE_MODEL = 1.1825 + 1.1003  # 2.2828
WHEELBASE_REAL = 2.512
SCALE = WHEELBASE_REAL / WHEELBASE_MODEL  # ~1.1004

CENTER_Y = -(-1.1825 + 1.1003) / 2.0  # +0.0411
GROUND_Z = 0.5953

# Blender -> Chrono rotation: R_z(+90deg)
#   Chrono X = -SCALE * Blender_Y + SCALE * CENTER_Y
#   Chrono Y =  SCALE * Blender_X
#   Chrono Z =  SCALE * (Blender_Z - GROUND_Z)
OFFSET_X = SCALE * CENTER_Y  # ~0.0452

# Chrono X threshold for classifying front signal faces.
# Faces with centroid X >= 1.9 are forward-facing turn signals;
# faces with centroid X < 1.9 are side-facing marker lamps.
# This gives an exact 50/50 split (864 faces each).
FRONT_SIGNAL_X_THRESHOLD = 1.9


# ---------------------------------------------------------------------------
# Material definitions
# ---------------------------------------------------------------------------
# Format: (Ka, Kd, Ks, Ns, d, illum)
# NOTE: redglass is OPAQUE (d=1.0) so reflectors are not treated as lamps.
MATERIAL_OVERRIDES = {
    # Body materials
    "carpaint":    ("0.02 0.05 0.01",  "0.05 0.20 0.02",  "0.60 0.60 0.55", 400, 1.0, 2),
    "black":       ("0.01 0.01 0.01",  "0.06 0.06 0.06",  "0.15 0.15 0.15", 20,  1.0, 2),
    "chrome":      ("0.25 0.25 0.25",  "0.55 0.55 0.55",  "0.90 0.90 0.90", 60,  1.0, 2),
    "mattemetal":  ("0.20 0.20 0.20",  "0.60 0.60 0.60",  "0.30 0.30 0.30", 30,  1.0, 2),
    "mirror":      ("0.15 0.18 0.17",  "0.35 0.40 0.38",  "0.50 0.50 0.50", 60,  1.0, 2),
    "interior":    ("0.08 0.08 0.08",  "0.30 0.30 0.30",  "0.10 0.10 0.10", 20,  1.0, 2),
    "windowglass": ("0.02 0.04 0.03",  "0.10 0.15 0.13",  "0.80 0.80 0.80", 400, 0.35, 2),
    "orangeglass": ("0.15 0.06 0.00",  "0.70 0.30 0.00",  "0.80 0.80 0.80", 400, 0.35, 2),
    "redglass":    ("0.10 0.01 0.01",  "0.50 0.05 0.05",  "0.80 0.80 0.80", 400, 1.0,  2),
    "clearglass":  ("0.10 0.10 0.10",  "0.70 0.70 0.70",  "1.00 1.00 1.00", 400, 0.2, 2),
    "rim":         ("0.20 0.20 0.20",  "0.55 0.55 0.55",  "0.70 0.70 0.70", 40,  1.0, 2),
    "tire":        ("0.02 0.02 0.02",  "0.10 0.10 0.10",  "0.05 0.05 0.05", 10,  1.0, 2),
    # Lamp materials
    "headlamp_left":            ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_right":           ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_hi_left":         ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_hi_right":        ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_lo_left":         ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_lo_right":        ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "signal_front_turn_left":   ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_front_turn_right":  ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_front_marker_left": ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_front_marker_right":("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "stoplamp_left":            ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    "stoplamp_right":           ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_rear_left":         ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_rear_right":        ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "side_marker_rear_left":    ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    "side_marker_rear_right":   ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    "backup_left":              ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "backup_right":             ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "chmsl":                    ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
}


# ---------------------------------------------------------------------------
# Step 1: Coordinate transform
# ---------------------------------------------------------------------------

def transform_obj(input_path, output_path):
    """Read OBJ, transform vertices and normals, write corrected OBJ."""

    # Detect if the file needs transformation by checking axis ranges
    xs, ys = [], []
    with open(input_path) as f:
        for line in f:
            if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
                parts = line.split()
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))

    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)

    if x_range > y_range:
        print(f"  OBJ already has X=forward (range {x_range:.2f}m > Y range {y_range:.2f}m)")
        print(f"  Skipping coordinate transform")
        return

    print(f"  Blender convention detected: X={x_range:.2f}m (lateral), Y={y_range:.2f}m (forward)")
    print(f"  Transforming to Chrono: X=forward, Y=left, Z=up from ground")

    lines_out = []
    v_count = 0
    vn_count = 0

    with open(input_path) as f:
        for line in f:
            if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
                parts = line.split()
                bx, by, bz = float(parts[1]), float(parts[2]), float(parts[3])
                cx = -SCALE * by + OFFSET_X
                cy =  SCALE * bx
                cz =  SCALE * (bz - GROUND_Z)
                lines_out.append(f"v {cx:.6f} {cy:.6f} {cz:.6f}\n")
                v_count += 1
            elif line.startswith('vn '):
                parts = line.split()
                nx, ny, nz = float(parts[1]), float(parts[2]), float(parts[3])
                lines_out.append(f"vn {-ny:.6f} {nx:.6f} {nz:.6f}\n")
                vn_count += 1
            else:
                lines_out.append(line)

    with open(output_path, 'w') as f:
        f.writelines(lines_out)

    # Verify
    xs, ys, zs = [], [], []
    for line in lines_out:
        if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
            parts = line.split()
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
            zs.append(float(parts[3]))
    print(f"  Transformed {v_count} vertices, {vn_count} normals")
    print(f"  Result: X[{min(xs):.3f}, {max(xs):.3f}] Y[{min(ys):.3f}, {max(ys):.3f}] Z[{min(zs):.3f}, {max(zs):.3f}]")


# ---------------------------------------------------------------------------
# Step 2: Split front signal materials by face normal direction
# ---------------------------------------------------------------------------

def split_front_signals(obj_path):
    """Split signal_front_left/right into turn/marker by face centroid X.

    The front corner lamp has two reflector areas behind a shared glass:
    the forward-facing half (higher X, closer to front of car) is the turn
    signal, and the side/rear-facing half (lower X) is the marker lamp.
    Splitting at centroid X >= 1.9 gives an exact 50/50 split (864/864).
    """
    with open(obj_path) as f:
        lines = f.readlines()

    # Parse vertex positions (already in Chrono coords after transform)
    vertices = []
    for line in lines:
        if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
            p = line.split()
            vertices.append((float(p[1]), float(p[2]), float(p[3])))

    if not vertices:
        print("  No vertices found, skipping signal split")
        return

    # Classify each face line that belongs to signal_front_left/right
    # (or already-split signal_front_turn_*/signal_front_marker_*).
    # Store the new material name for each such face line.
    FRONT_SIGNAL_MTLS = {
        'signal_front_left', 'signal_front_right',
        'signal_front_turn_left', 'signal_front_turn_right',
        'signal_front_marker_left', 'signal_front_marker_right',
    }
    face_new_mtl = {}  # line_index -> new material name
    current_mtl = None
    x_threshold = FRONT_SIGNAL_X_THRESHOLD

    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith('usemtl '):
            current_mtl = s.split(None, 1)[1]
        elif s.startswith('f ') and current_mtl in FRONT_SIGNAL_MTLS:
            side = 'left' if current_mtl.endswith('left') else 'right'
            tokens = s.split()[1:]
            # Compute face centroid X from vertex positions
            x_sum = 0.0
            v_count = 0
            for tok in tokens:
                vi = int(tok.split('/')[0]) - 1
                if 0 <= vi < len(vertices):
                    x_sum += vertices[vi][0]
                    v_count += 1
            if v_count > 0:
                centroid_x = x_sum / v_count
            else:
                centroid_x = 0.0

            if centroid_x >= x_threshold:
                face_new_mtl[i] = f'signal_front_turn_{side}'
            else:
                face_new_mtl[i] = f'signal_front_marker_{side}'

    if not face_new_mtl:
        print("  No signal_front faces found to split")
        return

    # Rebuild the file, replacing signal_front_left/right usemtl blocks
    # with per-face turn/marker assignments.
    output = []
    current_mtl = None
    active_mtl = None  # last usemtl we emitted
    in_signal_block = False

    for i, line in enumerate(lines):
        s = line.strip()

        if s.startswith('usemtl '):
            mtl_name = s.split(None, 1)[1]
            if mtl_name in FRONT_SIGNAL_MTLS:
                in_signal_block = True
                current_mtl = mtl_name
                # Don't emit this usemtl — faces will get their own
                continue
            else:
                in_signal_block = False
                current_mtl = mtl_name
                active_mtl = mtl_name
                output.append(line)

        elif i in face_new_mtl:
            new_mtl = face_new_mtl[i]
            if new_mtl != active_mtl:
                output.append(f'usemtl {new_mtl}\n')
                active_mtl = new_mtl
            output.append(line)

        else:
            output.append(line)

    with open(obj_path, 'w') as f:
        f.writelines(output)

    # Count results
    counts = {}
    cur = None
    for line in output:
        s = line.strip()
        if s.startswith('usemtl '):
            cur = s.split(None, 1)[1]
        elif s.startswith('f ') and cur and 'signal_front' in cur:
            counts[cur] = counts.get(cur, 0) + 1
    for mtl in sorted(counts):
        print(f"    {mtl}: {counts[mtl]} faces")


# ---------------------------------------------------------------------------
# Step 3: Split headlamp materials into hi/lo beam by centroid Y
# ---------------------------------------------------------------------------

def split_headlamps(obj_path):
    """Split headlamp_left/right into hi_beam/lo_beam by face centroid Y.

    The inner half (closer to vehicle centerline Y=0) is the hi beam
    projector; the outer half (farther from centerline) is the lo beam.
    Uses the median centroid Y to get a 50/50 split.
    """
    with open(obj_path) as f:
        lines = f.readlines()

    # Parse vertex positions
    vertices = []
    for line in lines:
        if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
            p = line.split()
            vertices.append((float(p[1]), float(p[2]), float(p[3])))

    if not vertices:
        print("  No vertices found, skipping headlamp split")
        return

    # Materials that should be (re-)split
    HEADLAMP_MTLS = {
        'headlamp_left', 'headlamp_right',
        'headlamp_hi_left', 'headlamp_hi_right',
        'headlamp_lo_left', 'headlamp_lo_right',
    }

    # First pass: collect centroid Y values per side to find the median.
    centroids_left = []   # (line_index, centroid_y)
    centroids_right = []
    current_mtl = None

    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith('usemtl '):
            current_mtl = s.split(None, 1)[1]
        elif s.startswith('f ') and current_mtl in HEADLAMP_MTLS:
            tokens = s.split()[1:]
            y_sum = 0.0
            v_count = 0
            for tok in tokens:
                vi = int(tok.split('/')[0]) - 1
                if 0 <= vi < len(vertices):
                    y_sum += vertices[vi][1]
                    v_count += 1
            if v_count > 0:
                cy = y_sum / v_count
            else:
                cy = 0.0

            if current_mtl.endswith('left') or current_mtl == 'headlamp_left':
                centroids_left.append((i, cy))
            else:
                centroids_right.append((i, cy))

    if not centroids_left and not centroids_right:
        print("  No headlamp faces found to split")
        return

    # Find split threshold for each side.
    # Use the 35th percentile (not median) to place the boundary so the
    # hi-beam projector dome gets ~35% of the lens (inner) and the lo beam
    # gets ~65% (outer), matching the housing geometry.
    def percentile_y(items, pct):
        ys = sorted(cy for _, cy in items)
        n = len(ys)
        idx = int(n * pct / 100)
        return ys[min(idx, n - 1)] if n > 0 else 0.0

    # Left (positive Y, ascending): 35th percentile = closer to center.
    med_left = percentile_y(centroids_left, 35)
    # Right (negative Y, ascending): 65th percentile = less negative = closer to center.
    med_right = percentile_y(centroids_right, 65)

    # Second pass: classify faces.
    # Left headlamp: inner (hi beam) = Y < threshold (closer to center),
    #                outer (lo beam) = Y >= threshold (farther from center).
    # Right headlamp: inner (hi beam) = Y > threshold (closer to center),
    #                 outer (lo beam) = Y <= threshold (farther from center).
    face_new_mtl = {}
    for idx, cy in centroids_left:
        if cy < med_left:
            face_new_mtl[idx] = 'headlamp_hi_left'
        else:
            face_new_mtl[idx] = 'headlamp_lo_left'

    for idx, cy in centroids_right:
        if cy > med_right:
            face_new_mtl[idx] = 'headlamp_hi_right'
        else:
            face_new_mtl[idx] = 'headlamp_lo_right'

    # Rebuild the file
    output = []
    active_mtl = None

    for i, line in enumerate(lines):
        s = line.strip()

        if s.startswith('usemtl '):
            mtl_name = s.split(None, 1)[1]
            if mtl_name in HEADLAMP_MTLS:
                # Don't emit — faces will get their own usemtl
                continue
            else:
                active_mtl = mtl_name
                output.append(line)

        elif i in face_new_mtl:
            new_mtl = face_new_mtl[i]
            if new_mtl != active_mtl:
                output.append(f'usemtl {new_mtl}\n')
                active_mtl = new_mtl
            output.append(line)

        else:
            output.append(line)

    with open(obj_path, 'w') as f:
        f.writelines(output)

    # Count results
    counts = {}
    cur = None
    for line in output:
        s = line.strip()
        if s.startswith('usemtl '):
            cur = s.split(None, 1)[1]
        elif s.startswith('f ') and cur and 'headlamp_' in cur:
            counts[cur] = counts.get(cur, 0) + 1
    for mtl in sorted(counts):
        print(f"    {mtl}: {counts[mtl]} faces")


# ---------------------------------------------------------------------------
# Step 4: Split redglass into reflectors vs rear side markers
# ---------------------------------------------------------------------------

REAR_MARKER_X_THRESHOLD = -1.9  # Chrono X >= -1.9 = side markers, < -1.9 = reflectors

def split_rear_markers(obj_path):
    """Split redglass into reflectors (opaque) and side markers (lamps).

    The redglass material has 4 clusters on the rear bumper.  The two
    rearmost (X < -1.9, near centerline) are passive reflectors.  The two
    on the sides (X >= -1.9, outboard) are the rear side marker lamps.
    Side marker faces get a new material with d=0.35 so VehicleLights
    classifies them as RED lamps.
    """
    with open(obj_path) as f:
        lines = f.readlines()

    # Parse vertex positions
    vertices = []
    for line in lines:
        if line.startswith('v ') and not line.startswith('vn') and not line.startswith('vt'):
            p = line.split()
            vertices.append((float(p[1]), float(p[2]), float(p[3])))

    if not vertices:
        print("  No vertices found, skipping rear marker split")
        return

    REDGLASS_MTLS = {'redglass', 'side_marker_rear_left', 'side_marker_rear_right'}

    # Classify each redglass face by centroid position.
    face_new_mtl = {}
    current_mtl = None

    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith('usemtl '):
            current_mtl = s.split(None, 1)[1]
        elif s.startswith('f ') and current_mtl in REDGLASS_MTLS:
            tokens = s.split()[1:]
            x_sum = y_sum = 0.0
            v_count = 0
            for tok in tokens:
                vi = int(tok.split('/')[0]) - 1
                if 0 <= vi < len(vertices):
                    x_sum += vertices[vi][0]
                    y_sum += vertices[vi][1]
                    v_count += 1
            if v_count > 0:
                cx = x_sum / v_count
                cy = y_sum / v_count
            else:
                cx = cy = 0.0

            if cx >= REAR_MARKER_X_THRESHOLD:
                # Side marker — assign by left/right
                if cy > 0:
                    face_new_mtl[i] = 'side_marker_rear_left'
                else:
                    face_new_mtl[i] = 'side_marker_rear_right'
            else:
                # Reflector — keep as redglass
                face_new_mtl[i] = 'redglass'

    if not face_new_mtl:
        print("  No redglass faces found to split")
        return

    # Rebuild the file
    output = []
    active_mtl = None

    for i, line in enumerate(lines):
        s = line.strip()

        if s.startswith('usemtl '):
            mtl_name = s.split(None, 1)[1]
            if mtl_name in REDGLASS_MTLS:
                # Don't emit — faces will get their own usemtl
                continue
            else:
                active_mtl = mtl_name
                output.append(line)

        elif i in face_new_mtl:
            new_mtl = face_new_mtl[i]
            if new_mtl != active_mtl:
                output.append(f'usemtl {new_mtl}\n')
                active_mtl = new_mtl
            output.append(line)

        else:
            output.append(line)

    with open(obj_path, 'w') as f:
        f.writelines(output)

    # Count results
    counts = {}
    cur = None
    for line in output:
        s = line.strip()
        if s.startswith('usemtl '):
            cur = s.split(None, 1)[1]
        elif s.startswith('f ') and cur and (cur == 'redglass' or 'side_marker' in cur):
            counts[cur] = counts.get(cur, 0) + 1
    for mtl in sorted(counts):
        print(f"    {mtl}: {counts[mtl]} faces")


# ---------------------------------------------------------------------------
# Step 5: MTL rewrite
# ---------------------------------------------------------------------------

def rewrite_mtl(input_path, output_path):
    """Rewrite MTL material properties for Phong shading compatibility."""
    with open(input_path) as f:
        lines = f.readlines()

    # Collect existing material names
    existing_mtls = set()
    for line in lines:
        s = line.strip()
        if s.startswith('newmtl '):
            existing_mtls.add(s.split(None, 1)[1])

    out = []
    current_mtl = None
    skip_props = False

    for line in lines:
        stripped = line.strip()
        if stripped.startswith('newmtl '):
            current_mtl = stripped.split(None, 1)[1]
            skip_props = current_mtl in MATERIAL_OVERRIDES
            out.append(line)
            if skip_props:
                ka, kd, ks, ns, d, illum = MATERIAL_OVERRIDES[current_mtl]
                out.append(f"Ns {ns:.6f}\n")
                out.append(f"Ka {ka}\n")
                out.append(f"Kd {kd}\n")
                out.append(f"Ks {ks}\n")
                out.append(f"Ke 0.000000 0.000000 0.000000\n")
                out.append(f"Ni 1.500000\n")
                out.append(f"d {d:.6f}\n")
                out.append(f"illum {illum}\n")
        elif skip_props:
            if stripped and not stripped.startswith('#'):
                continue
            else:
                out.append(line)
        else:
            out.append(line)

    # Add new materials created by split functions that aren't in the MTL yet.
    new_mtls = [m for m in MATERIAL_OVERRIDES if m not in existing_mtls]
    for mtl_name in sorted(new_mtls):
        ka, kd, ks, ns, d, illum = MATERIAL_OVERRIDES[mtl_name]
        out.append(f"\nnewmtl {mtl_name}\n")
        out.append(f"Ns {ns:.6f}\n")
        out.append(f"Ka {ka}\n")
        out.append(f"Kd {kd}\n")
        out.append(f"Ks {ks}\n")
        out.append(f"Ke 0.000000 0.000000 0.000000\n")
        out.append(f"Ni 1.500000\n")
        out.append(f"d {d:.6f}\n")
        out.append(f"illum {illum}\n")

    with open(output_path, 'w') as f:
        f.writelines(out)

    overridden = set()
    for line in out:
        s = line.strip()
        if s.startswith('newmtl '):
            name = s.split(None, 1)[1]
            if name in MATERIAL_OVERRIDES:
                overridden.add(name)
    print(f"  Wrote {len(overridden)} materials in {os.path.basename(output_path)}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) > 1:
        ev1model_dir = sys.argv[1]
    else:
        ev1model_dir = os.path.dirname(os.path.abspath(__file__))

    data_dir = os.path.join(os.path.dirname(ev1model_dir), "data", "vehicle", "ev1")

    obj_src = os.path.join(ev1model_dir, "ev1_chassis_vis.obj")
    mtl_src = os.path.join(ev1model_dir, "ev1_chassis_vis.mtl")

    if not os.path.exists(obj_src):
        print(f"ERROR: {obj_src} not found")
        sys.exit(1)

    print("=== Fix OBJ coordinates and MTL materials ===\n")

    # 1. Transform OBJ coordinates (Blender -> Chrono)
    print(f"OBJ: {obj_src}")
    transform_obj(obj_src, obj_src)

    # 2. Split front signals into turn/marker by centroid X
    print(f"\nSplitting front signal lamps:")
    split_front_signals(obj_src)

    # 3. Split headlamps into hi/lo beam by centroid Y
    print(f"\nSplitting headlamps:")
    split_headlamps(obj_src)

    # 4. Split redglass into reflectors vs rear side markers
    print(f"\nSplitting rear side markers from redglass:")
    split_rear_markers(obj_src)

    # 5. Rewrite MTL materials (including new split entries)
    print(f"\nMTL: {mtl_src}")
    if os.path.exists(mtl_src):
        rewrite_mtl(mtl_src, mtl_src)

    # 6. Copy to data directory
    print(f"\nCopying to {data_dir}/")
    for fname in ["ev1_chassis_vis.obj", "ev1_chassis_vis.mtl"]:
        src = os.path.join(ev1model_dir, fname)
        dst = os.path.join(data_dir, fname)
        if os.path.exists(src) and os.path.isdir(data_dir):
            shutil.copy2(src, dst)
            print(f"  {fname} -> data/vehicle/ev1/")

    print("\n=== Done ===")


if __name__ == "__main__":
    main()
