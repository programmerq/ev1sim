"""
Export EV1 Blender model to OBJ files for Project Chrono.

Produces:
  ev1_chassis_vis.obj  — body + details + glass + lights (with per-lamp materials)
  ev1_chassis_col.obj  — low-poly collision hull (~500 faces, no materials)
  ev1_rim.obj          — single wheel rim (decimated)
  ev1_tire.obj         — single tire

Coordinate transform: Blender → Chrono (SAE body-fixed)
  Chrono X = -Blender Y  (forward)
  Chrono Y =  Blender X  (left)
  Chrono Z =  Blender Z  (up)
  This is R_z(+90°), a proper rotation (det=+1, preserves normals).

Origin: center of wheelbase at ground contact level.
Scale:  uniform factor to match real EV1 wheelbase (2.512 m).

Run:  blender --background GMEV1-pluslight.blend --python export_meshes.py
"""

import bpy
import mathutils
import math
import os
import bmesh

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Geometric reference points (world-space meters after parent 0.01 scale) ---
# Front axle center Y ≈ -1.1825, rear axle center Y ≈ 1.1003
# Model wheelbase ≈ 2.2828
WHEELBASE_MODEL = 1.1825 + 1.1003  # 2.2828
WHEELBASE_REAL = 2.512
SCALE = WHEELBASE_REAL / WHEELBASE_MODEL  # ~1.1004

CENTER_Y = -(-1.1825 + 1.1003) / 2.0  # longitudinal center (+0.0411)
GROUND_Z = 0.5953                      # tire contact patch Z

# 4×4 transform: SCALE * R_z(90°) with origin offset
# v_out = SCALE * R_z(90) @ (v_in - [0, CENTER_Y, GROUND_Z])
BODY_MATRIX = mathutils.Matrix([
    [0,     -SCALE, 0,     SCALE * (-CENTER_Y)],
    [SCALE,  0,     0,     0                   ],
    [0,      0,     SCALE, -SCALE * GROUND_Z   ],
    [0,      0,     0,     1                   ],
])

# Objects to EXCLUDE from the chassis mesh
EXCLUDE_PREFIXES = ('rim', 'tire')

# Rim target face count after decimation
RIM_TARGET_FACES = 2000
# Collision mesh target face count (very low-poly hull)
COLLISION_TARGET_FACES = 500


# =============================================================================
#  Light material assignment table
# =============================================================================
#
# Each entry maps a Blender object name to a light-material assignment rule.
# Rules:
#   "split_lr:<base>"        — Split faces by Y into <base>_left / <base>_right
#   "material:<name>"        — Assign all faces to material <name>
#   "reflector"              — Leave as original material (not a light)
#   "bulbs"                  — Assign bulb faces to the nearest lamp group
#   "chmsl"                  — Extract redglass faces as chmsl material
#   "split_lr_fb:<base>"     — Split by Y (left/right) AND by X relative to
#                              centroid (front=turn, back=marker)
#
LIGHT_ASSIGNMENTS = {
    # --- Front ---
    "detail03":  "split_lr:headlamp",          # Main headlight lens (clearglass)
    "detail20":  "split_lr:headlamp",          # Headlight inner ring (clearglass)
    "detail10":  "split_lr:signal_front",       # Front corner turn/marker lens (orangeglass)
    # --- Rear ---
    "light_glass": "split_lr:stoplamp",        # Rear stop/tail lamp lens (redglass)
    "glass":       "split_lr:signal_rear",     # Rear turn/marker lens (orangeglass)
    "detail16":    "split_lr:backup",          # Backup lamp lens (clearglass)
    # --- Reflectors (NOT lights) ---
    "detail11":    "reflector",                # Lower bumper reflector (redglass)
    "detail12":    "reflector",                # Far rear reflector (redglass)
    # --- CHMSL ---
    "detail24":    "chmsl",                    # Center high mount stop lamp (black + redglass)
    # --- Bulbs ---
    # "lights" object stays as clearglass — the visual effect comes from
    # the coloured glass lens in front, not the bulb behind it.
    # (User preference: "skip it entirely in favor of only using the glass face itself")
}


# =============================================================================
#  Lamp material definitions for the MTL file
# =============================================================================
# (Ka, Kd, Ks, Ns, d, illum) — Phong-friendly values for Irrlicht.
# The C++ VehicleLights class sets EmissiveColor at runtime.
LAMP_MATERIALS = {
    # Headlamp lenses (clear glass in front of bulb)
    "headlamp_left":      ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "headlamp_right":     ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    # Front corner signal lenses (orange glass)
    "signal_front_left":  ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_front_right": ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    # Rear stop/tail lamp lenses (red glass)
    "stoplamp_left":      ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    "stoplamp_right":     ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
    # Rear signal lenses (orange glass)
    "signal_rear_left":   ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    "signal_rear_right":  ("0.15 0.06 0.00", "0.70 0.30 0.00", "0.80 0.80 0.80", 400, 0.35, 2),
    # Backup lamp lenses (clear glass)
    "backup_left":        ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    "backup_right":       ("0.10 0.10 0.10", "0.70 0.70 0.70", "1.00 1.00 1.00", 400, 0.20, 2),
    # CHMSL (red glass)
    "chmsl":              ("0.10 0.01 0.01", "0.50 0.05 0.05", "0.80 0.80 0.80", 400, 0.35, 2),
}

# Non-light material overrides (same as before — Phong-friendly colors)
BODY_MATERIAL_OVERRIDES = {
    "carpaint":    ("0.02 0.05 0.01",  "0.05 0.20 0.02",  "0.60 0.60 0.55", 400, 1.0, 2),
    "black":       ("0.01 0.01 0.01",  "0.06 0.06 0.06",  "0.15 0.15 0.15", 20,  1.0, 2),
    "chrome":      ("0.25 0.25 0.25",  "0.55 0.55 0.55",  "0.90 0.90 0.90", 60,  1.0, 2),
    "mattemetal":   ("0.20 0.20 0.20",  "0.60 0.60 0.60",  "0.30 0.30 0.30", 30,  1.0, 2),
    "mirror":      ("0.15 0.18 0.17",  "0.35 0.40 0.38",  "0.50 0.50 0.50", 60,  1.0, 2),
    "interior":    ("0.08 0.08 0.08",  "0.30 0.30 0.30",  "0.10 0.10 0.10", 20,  1.0, 2),
    "windowglass": ("0.02 0.04 0.03",  "0.10 0.15 0.13",  "0.80 0.80 0.80", 400, 0.35, 2),
    "orangeglass": ("0.15 0.06 0.00",  "0.70 0.30 0.00",  "0.80 0.80 0.80", 400, 0.35, 2),
    "redglass":    ("0.10 0.01 0.01",  "0.50 0.05 0.05",  "0.80 0.80 0.80", 400, 0.35, 2),
    "clearglass":  ("0.10 0.10 0.10",  "0.70 0.70 0.70",  "1.00 1.00 1.00", 400, 0.2, 2),
    "rim":         ("0.20 0.20 0.20",  "0.55 0.55 0.55",  "0.70 0.70 0.70", 40,  1.0, 2),
    "tire":        ("0.02 0.02 0.02",  "0.10 0.10 0.10",  "0.05 0.05 0.05", 10,  1.0, 2),
}


# =============================================================================
#  Helpers
# =============================================================================

def remove_shape_keys(obj):
    """Remove all shape keys from a mesh object so modifiers can be applied."""
    if obj.data.shape_keys:
        bpy.context.view_layer.objects.active = obj
        while obj.data.shape_keys:
            obj.shape_key_remove(obj.data.shape_keys.key_blocks[0])


def apply_parent_transforms():
    """Unparent all mesh objects while preserving world transforms."""
    for obj in list(bpy.data.objects):
        if obj.type == 'MESH' and obj.parent:
            wm = obj.matrix_world.copy()
            obj.parent = None
            obj.matrix_world = wm


def apply_all_transforms():
    """Bake each mesh object's transform into its vertex data.

    Processes each object individually so shape-keyed objects don't block
    the entire batch.  Shape keys are removed first since they prevent
    bpy.ops.object.transform_apply from working.
    """
    for obj in list(bpy.data.objects):
        if obj.type != 'MESH':
            continue
        # Shape keys prevent transform_apply — remove them first.
        remove_shape_keys(obj)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        try:
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        except RuntimeError as e:
            print(f"  WARNING: Could not apply transform on {obj.name}: {e}")
    bpy.ops.object.select_all(action='DESELECT')


def transform_mesh_vertices(obj, matrix):
    """Apply a 4×4 matrix to every vertex of an object's mesh.

    Also resets matrix_world to identity so that the OBJ exporter
    (which writes world-space = matrix_world @ local) outputs the
    transformed local coordinates directly.
    """
    mesh = obj.data
    for v in mesh.vertices:
        v.co = (matrix @ v.co.to_4d()).to_3d()
    mesh.update()
    obj.matrix_world = mathutils.Matrix.Identity(4)


def categorize_objects():
    """Sort scene mesh objects into body parts, rims, and tires."""
    body_parts = []
    rims = []
    tires = []
    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        name = obj.name.lower()
        if name.startswith('rim'):
            rims.append(obj)
        elif name.startswith('tire'):
            tires.append(obj)
        else:
            body_parts.append(obj)
    return body_parts, rims, tires


def join_objects(objects, name):
    """Join a list of mesh objects into one, returning the result."""
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    result = bpy.context.active_object
    result.name = name
    bpy.ops.object.select_all(action='DESELECT')
    return result


def decimate_object(obj, target_faces):
    """Add and apply a Decimate modifier to reduce face count."""
    current = len(obj.data.polygons)
    if current <= target_faces:
        return
    # Shape keys prevent modifier_apply — remove them first.
    remove_shape_keys(obj)
    ratio = target_faces / current
    print(f"  Decimating {obj.name}: {current} → ~{target_faces} faces (ratio {ratio:.4f})")
    mod = obj.modifiers.new(name='Decimate', type='DECIMATE')
    mod.ratio = ratio
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)
    print(f"  Result: {len(obj.data.polygons)} faces")


def export_obj(obj, filename, materials=True):
    """Export a single object as OBJ (triangulated, with normals)."""
    filepath = os.path.join(OUT_DIR, filename)
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    bpy.ops.wm.obj_export(
        filepath=filepath,
        export_selected_objects=True,
        forward_axis='Y',
        up_axis='Z',
        export_triangulated_mesh=True,
        export_normals=True,
        export_materials=materials,
        global_scale=1.0,
    )
    print(f"  Exported: {filepath}")


def duplicate_object(obj, name):
    """Duplicate a mesh object (geometry only, deep copy of mesh data)."""
    new_mesh = obj.data.copy()
    new_obj = bpy.data.objects.new(name, new_mesh)
    bpy.context.collection.objects.link(new_obj)
    new_obj.matrix_world = obj.matrix_world.copy()
    return new_obj


def compute_center(obj):
    """Return the world-space bounding box center of an object."""
    bbox = [mathutils.Vector(c) for c in obj.bound_box]
    center = sum(bbox, mathutils.Vector()) / 8
    return center


def get_or_create_material(name):
    """Get an existing material or create a new one with the given name."""
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)
    return mat


def chrono_pos(blender_pos):
    """Convert a Blender world-space position to Chrono coordinates."""
    return (BODY_MATRIX @ blender_pos.to_4d()).to_3d()


# =============================================================================
#  Light material assignment
# =============================================================================

def assign_light_materials(body_parts):
    """Assign per-lamp materials to light objects BEFORE joining.

    This modifies the material slots of each light-related object so that
    after joining, the OBJ will contain one material group per lamp.
    """
    print("\n--- Assigning per-lamp materials ---")

    # Pre-compute the Chrono-frame centroid for each lamp group.
    # Used by "bulbs" assignment to match bulbs to the nearest lamp.
    lamp_centroids = {}

    for obj in body_parts:
        rule = LIGHT_ASSIGNMENTS.get(obj.name)
        if rule is None:
            continue

        # Compute Chrono-frame vertex positions for this object
        wm = obj.matrix_world
        chrono_verts = [chrono_pos(wm @ v.co) for v in obj.data.vertices]

        if rule.startswith("split_lr:"):
            base = rule.split(":", 1)[1]
            _assign_split_lr(obj, base, chrono_verts)
            # Record centroids for both sides
            left_verts = [v for v in chrono_verts if v.y > 0]
            right_verts = [v for v in chrono_verts if v.y <= 0]
            if left_verts:
                lamp_centroids[f"{base}_left"] = sum(left_verts, mathutils.Vector()) / len(left_verts)
            if right_verts:
                lamp_centroids[f"{base}_right"] = sum(right_verts, mathutils.Vector()) / len(right_verts)

        elif rule == "reflector":
            print(f"  {obj.name}: keeping as reflector (not a light)")

        elif rule == "chmsl":
            _assign_chmsl(obj, chrono_verts)
            chmsl_verts = [v for i, v in enumerate(chrono_verts)]
            if chmsl_verts:
                lamp_centroids["chmsl"] = sum(chmsl_verts, mathutils.Vector()) / len(chmsl_verts)

        elif rule == "bulbs":
            # Deferred — need centroids from other lamp groups first
            pass

    # Now handle bulbs with the known lamp centroids
    for obj in body_parts:
        if LIGHT_ASSIGNMENTS.get(obj.name) == "bulbs":
            _assign_bulbs(obj, lamp_centroids)


def _assign_split_lr(obj, base_name, chrono_verts):
    """Split an object's faces into _left and _right materials by Y position."""
    mesh = obj.data

    mat_left = get_or_create_material(f"{base_name}_left")
    mat_right = get_or_create_material(f"{base_name}_right")

    # Add materials to object if not present
    if mat_left.name not in [s.material.name for s in obj.material_slots if s.material]:
        obj.data.materials.append(mat_left)
    if mat_right.name not in [s.material.name for s in obj.material_slots if s.material]:
        obj.data.materials.append(mat_right)

    # Find slot indices
    left_idx = None
    right_idx = None
    for i, slot in enumerate(obj.material_slots):
        if slot.material and slot.material.name == mat_left.name:
            left_idx = i
        if slot.material and slot.material.name == mat_right.name:
            right_idx = i

    # Assign each face to left or right based on centroid Y in Chrono frame
    left_count = 0
    right_count = 0
    for poly in mesh.polygons:
        # Compute face centroid in Chrono frame
        cy = sum(chrono_verts[vi].y for vi in poly.vertices) / len(poly.vertices)
        if cy > 0:
            poly.material_index = left_idx
            left_count += 1
        else:
            poly.material_index = right_idx
            right_count += 1

    print(f"  {obj.name}: split into {base_name}_left ({left_count}) / {base_name}_right ({right_count})")


def _assign_chmsl(obj, chrono_verts):
    """Assign redglass faces of detail24 to 'chmsl' material, keep black as-is."""
    mesh = obj.data

    mat_chmsl = get_or_create_material("chmsl")
    if mat_chmsl.name not in [s.material.name for s in obj.material_slots if s.material]:
        obj.data.materials.append(mat_chmsl)

    chmsl_idx = None
    redglass_idx = None
    for i, slot in enumerate(obj.material_slots):
        if slot.material:
            if slot.material.name == "chmsl":
                chmsl_idx = i
            elif slot.material.name == "redglass":
                redglass_idx = i

    if redglass_idx is None or chmsl_idx is None:
        print(f"  {obj.name}: could not find redglass/chmsl slots")
        return

    count = 0
    for poly in mesh.polygons:
        if poly.material_index == redglass_idx:
            poly.material_index = chmsl_idx
            count += 1

    print(f"  {obj.name}: assigned {count} faces to chmsl")


def _assign_bulbs(obj, lamp_centroids):
    """Assign bulb faces to the nearest lamp group material."""
    if not lamp_centroids:
        print(f"  {obj.name}: no lamp centroids available, skipping bulb assignment")
        return

    mesh = obj.data
    wm = obj.matrix_world
    chrono_verts = [chrono_pos(wm @ v.co) for v in mesh.vertices]

    # Add all lamp materials to this object
    for lamp_name in lamp_centroids:
        mat = get_or_create_material(lamp_name)
        if mat.name not in [s.material.name for s in obj.material_slots if s.material]:
            mesh.materials.append(mat)

    # Build slot index map
    slot_map = {}
    for i, slot in enumerate(obj.material_slots):
        if slot.material:
            slot_map[slot.material.name] = i

    # Assign each face to nearest lamp group
    assigned = {}
    for poly in mesh.polygons:
        cx = sum(chrono_verts[vi].x for vi in poly.vertices) / len(poly.vertices)
        cy = sum(chrono_verts[vi].y for vi in poly.vertices) / len(poly.vertices)
        cz = sum(chrono_verts[vi].z for vi in poly.vertices) / len(poly.vertices)
        face_pos = mathutils.Vector((cx, cy, cz))

        # Find nearest lamp centroid
        best_dist = float('inf')
        best_name = None
        for name, centroid in lamp_centroids.items():
            d = (face_pos - centroid).length
            if d < best_dist:
                best_dist = d
                best_name = name

        if best_name and best_name in slot_map:
            poly.material_index = slot_map[best_name]
            assigned[best_name] = assigned.get(best_name, 0) + 1

    print(f"  {obj.name}: assigned bulbs to nearest lamp groups:")
    for name, count in sorted(assigned.items()):
        print(f"    {name}: {count} faces")


# =============================================================================
#  MTL post-processing
# =============================================================================

def rewrite_mtl(filepath):
    """Rewrite MTL material properties for Phong shading compatibility."""
    if not os.path.exists(filepath):
        return

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Merge all override tables
    all_overrides = {}
    all_overrides.update(BODY_MATERIAL_OVERRIDES)
    for name, props in LAMP_MATERIALS.items():
        all_overrides[name] = props

    out = []
    current_mtl = None
    skip_props = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('newmtl '):
            current_mtl = stripped.split(None, 1)[1]
            skip_props = current_mtl in all_overrides
            out.append(line)
            if skip_props:
                ka, kd, ks, ns, d, illum = all_overrides[current_mtl]
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

    with open(filepath, 'w') as f:
        f.writelines(out)
    print(f"  Rewrote: {filepath}")


# =============================================================================
#  Main
# =============================================================================
print("\n=== EV1 Mesh Export ===\n")

# Step 1: Flatten hierarchy
print("Applying parent transforms...")
apply_parent_transforms()
apply_all_transforms()

# Step 2: Categorize
body_parts, rims, tires = categorize_objects()
print(f"Body parts: {len(body_parts)}  Rims: {len(rims)}  Tires: {len(tires)}")

# Step 3: Assign per-lamp materials to light objects
assign_light_materials(body_parts)

# Step 4: Export chassis (body) — vis + collision
print("\n--- Chassis (vis) ---")
chassis = join_objects(body_parts, "ev1_chassis")
total_faces = len(chassis.data.polygons)
print(f"  Joined chassis: {len(chassis.data.vertices)} verts, {total_faces} faces")

# Verify pre-transform (should be Blender world coords)
mesh = chassis.data
_xs = [v.co.x for v in mesh.vertices]
_ys = [v.co.y for v in mesh.vertices]
_zs = [v.co.z for v in mesh.vertices]
print(f"  Pre-transform local: X[{min(_xs):.3f},{max(_xs):.3f}] Y[{min(_ys):.3f},{max(_ys):.3f}] Z[{min(_zs):.3f},{max(_zs):.3f}]")
mw = chassis.matrix_world
print(f"  matrix_world diag: [{mw[0][0]:.4f}, {mw[1][1]:.4f}, {mw[2][2]:.4f}]  "
      f"trans: [{mw[0][3]:.4f}, {mw[1][3]:.4f}, {mw[2][3]:.4f}]")

transform_mesh_vertices(chassis, BODY_MATRIX)

# Verify post-transform (should be Chrono coords: X≈±2.1, Y≈±1.0, Z≈0.17-1.31)
_xs = [v.co.x for v in mesh.vertices]
_ys = [v.co.y for v in mesh.vertices]
_zs = [v.co.z for v in mesh.vertices]
print(f"  Post-transform:     X[{min(_xs):.3f},{max(_xs):.3f}] Y[{min(_ys):.3f},{max(_ys):.3f}] Z[{min(_zs):.3f},{max(_zs):.3f}]")
mw = chassis.matrix_world
print(f"  matrix_world diag: [{mw[0][0]:.4f}, {mw[1][1]:.4f}, {mw[2][2]:.4f}]  "
      f"trans: [{mw[0][3]:.4f}, {mw[1][3]:.4f}, {mw[2][3]:.4f}]")

export_obj(chassis, "ev1_chassis_vis.obj", materials=True)

# Step 4b: Collision mesh — duplicate, decimate aggressively, no materials
print("\n--- Chassis (collision) ---")
col_mesh = duplicate_object(chassis, "ev1_chassis_col")
col_mesh.matrix_world = mathutils.Matrix.Identity(4)
decimate_object(col_mesh, COLLISION_TARGET_FACES)
export_obj(col_mesh, "ev1_chassis_col.obj", materials=False)

# Step 5: Export rim (pick first, decimate, center, transform, export)
print("\n--- Rim ---")
rim = rims[0]
rim_center = compute_center(rim)
print(f"  Rim '{rim.name}' center: {rim_center}")
cx, cy, cz = rim_center.x, rim_center.y, rim_center.z
rim_matrix = mathutils.Matrix([
    [0,     -SCALE, 0,     SCALE * cy ],
    [SCALE,  0,     0,     -SCALE * cx],
    [0,      0,     SCALE, -SCALE * cz],
    [0,      0,     0,     1          ],
])

decimate_object(rim, RIM_TARGET_FACES)
transform_mesh_vertices(rim, rim_matrix)
export_obj(rim, "ev1_rim.obj")

# Verify rim bounds
mesh = rim.data
xs = [v.co.x for v in mesh.vertices]
ys = [v.co.y for v in mesh.vertices]
zs = [v.co.z for v in mesh.vertices]
print(f"  Rim OBJ bounds: X[{min(xs):.4f},{max(xs):.4f}] Y[{min(ys):.4f},{max(ys):.4f}] Z[{min(zs):.4f},{max(zs):.4f}]")

# Step 6: Export tire (pick first, center, transform, export)
print("\n--- Tire ---")
tire = tires[0]
tire_center = compute_center(tire)
print(f"  Tire '{tire.name}' center: {tire_center}")
tx, ty, tz = tire_center.x, tire_center.y, tire_center.z
tire_matrix = mathutils.Matrix([
    [0,     -SCALE, 0,     SCALE * ty ],
    [SCALE,  0,     0,     -SCALE * tx],
    [0,      0,     SCALE, -SCALE * tz],
    [0,      0,     0,     1          ],
])
transform_mesh_vertices(tire, tire_matrix)
export_obj(tire, "ev1_tire.obj")

mesh = tire.data
xs = [v.co.x for v in mesh.vertices]
ys = [v.co.y for v in mesh.vertices]
zs = [v.co.z for v in mesh.vertices]
print(f"  Tire OBJ bounds: X[{min(xs):.4f},{max(xs):.4f}] Y[{min(ys):.4f},{max(ys):.4f}] Z[{min(zs):.4f},{max(zs):.4f}]")

# Step 7: Post-process MTL files
print("\n--- Post-processing MTL files ---")
for mtl in ["ev1_chassis_vis.mtl", "ev1_rim.mtl", "ev1_tire.mtl"]:
    rewrite_mtl(os.path.join(OUT_DIR, mtl))

print("\n=== Done ===")
