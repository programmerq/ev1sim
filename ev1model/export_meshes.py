"""
Export EV1 Blender model to OBJ files for Project Chrono.

Produces:
  ev1_chassis_vis.obj  — body + details + glass + etc. (with materials)
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

Run:  blender --background GMEV1.blend --python export_meshes.py
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

CENTER_Y = (-1.1825 + 1.1003) / 2.0  # longitudinal center (-0.0411)
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
# Interior objects — include them (visible through windows)

# Rim target face count after decimation
RIM_TARGET_FACES = 2000
# Collision mesh target face count (very low-poly hull)
COLLISION_TARGET_FACES = 500


def apply_parent_transforms():
    """Unparent all mesh objects while preserving world transforms."""
    for obj in list(bpy.data.objects):
        if obj.type == 'MESH' and obj.parent:
            wm = obj.matrix_world.copy()
            obj.parent = None
            obj.matrix_world = wm


def apply_all_transforms():
    """Bake each mesh object's transform into its vertex data."""
    bpy.ops.object.select_all(action='DESELECT')
    for obj in bpy.data.objects:
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.select_all(action='DESELECT')


def transform_mesh_vertices(obj, matrix):
    """Apply a 4×4 matrix to every vertex of an object's mesh."""
    mesh = obj.data
    for v in mesh.vertices:
        v.co = (matrix @ v.co.to_4d()).to_3d()
    mesh.update()


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
    ratio = target_faces / current
    print(f"  Decimating {obj.name}: {current} → ~{target_faces} faces (ratio {ratio:.4f})")
    mod = obj.modifiers.new(name='Decimate', type='DECIMATE')
    mod.ratio = ratio
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)
    print(f"  Result: {len(obj.data.polygons)} faces")


def export_obj(obj, filename, materials=True):
    """Export a single object as OBJ (triangulated, with normals).

    We already baked the Blender→Chrono rotation into vertex data via
    BODY_MATRIX, so the OBJ exporter must use identity axes (forward=Y,
    up=Z) to avoid an extra 180° flip.
    """
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

# Step 3: Export chassis (body) — vis + collision
print("\n--- Chassis (vis) ---")
chassis = join_objects(body_parts, "ev1_chassis")
total_faces = len(chassis.data.polygons)
print(f"  Joined chassis: {len(chassis.data.vertices)} verts, {total_faces} faces")
transform_mesh_vertices(chassis, BODY_MATRIX)
export_obj(chassis, "ev1_chassis_vis.obj", materials=True)

# Step 3b: Collision mesh — duplicate, decimate aggressively, no materials
print("\n--- Chassis (collision) ---")
col_mesh = duplicate_object(chassis, "ev1_chassis_col")
decimate_object(col_mesh, COLLISION_TARGET_FACES)
export_obj(col_mesh, "ev1_chassis_col.obj", materials=False)

# Step 4: Export rim (pick first, decimate, center, transform, export)
print("\n--- Rim ---")
rim = rims[0]
# Center the rim at its own bounding-box center
rim_center = compute_center(rim)
print(f"  Rim '{rim.name}' center: {rim_center}")
# Build the rim transform: same rotation, but centered on rim's own center
rim_matrix = mathutils.Matrix([
    [0,     -SCALE, 0,     SCALE * (-rim_center.y)],
    [SCALE,  0,     0,     -SCALE * rim_center.x  ],
    [0,      0,     SCALE, -SCALE * rim_center.z   ],
    [0,      0,     0,     1                       ],
])
# Wait — the rim should be centered at origin in the output.
# v_out = SCALE * R_z(90) @ (v - rim_center)
# Using the same formula: translation = SCALE * R_z(90) @ (-rim_center)
# R_z(90) @ [-cx, -cy, -cz] = [cy, -cx, -cz]
# translation = SCALE * [cy, -cx, -cz]
# Hmm, let me just recompute properly.
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

# Step 5: Export tire (pick first, center, transform, export)
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

# Verify tire bounds
mesh = tire.data
xs = [v.co.x for v in mesh.vertices]
ys = [v.co.y for v in mesh.vertices]
zs = [v.co.z for v in mesh.vertices]
print(f"  Tire OBJ bounds: X[{min(xs):.4f},{max(xs):.4f}] Y[{min(ys):.4f},{max(ys):.4f}] Z[{min(zs):.4f},{max(zs):.4f}]")

# Step 6: Post-process MTL files for non-PBR renderers (Irrlicht, Quick Look).
# Blender 5.1's Principled BSDF → OBJ MTL conversion produces very dark Kd
# values because the PBR appearance comes from metallic/roughness/clearcoat
# which can't be represented in Phong shading.  We rewrite the MTL with
# hand-tuned Phong-friendly values that look correct in Irrlicht and Finder.
print("\n--- Post-processing MTL files ---")

# Mapping: material name → (Ka, Kd, Ks, Ns, d, illum)
MATERIAL_OVERRIDES = {
    # Chassis materials
    "carpaint":    ("0.02 0.05 0.01",  "0.05 0.20 0.02",  "0.60 0.60 0.55", 400, 1.0, 2),
    "black":       ("0.01 0.01 0.01",  "0.06 0.06 0.06",  "0.15 0.15 0.15", 20,  1.0, 2),
    "chrome":      ("0.25 0.25 0.25",  "0.55 0.55 0.55",  "0.90 0.90 0.90", 60,  1.0, 2),
    "mattemetal":   ("0.20 0.20 0.20",  "0.60 0.60 0.60",  "0.30 0.30 0.30", 30,  1.0, 2),
    "mirror":      ("0.15 0.18 0.17",  "0.35 0.40 0.38",  "0.50 0.50 0.50", 60,  1.0, 2),
    "interior":    ("0.08 0.08 0.08",  "0.30 0.30 0.30",  "0.10 0.10 0.10", 20,  1.0, 2),
    "clearglass":  ("0.10 0.10 0.10",  "0.70 0.70 0.70",  "1.00 1.00 1.00", 400, 0.2, 2),
    "windowglass": ("0.02 0.04 0.03",  "0.10 0.15 0.13",  "0.80 0.80 0.80", 400, 0.35, 2),
    "redglass":    ("0.10 0.01 0.01",  "0.50 0.05 0.05",  "0.80 0.80 0.80", 400, 0.35, 2),
    "orangeglass": ("0.15 0.06 0.00",  "0.70 0.30 0.00",  "0.80 0.80 0.80", 400, 0.35, 2),
    # Wheel / tire materials
    "rim":         ("0.20 0.20 0.20",  "0.55 0.55 0.55",  "0.70 0.70 0.70", 40,  1.0, 2),
    "tire":        ("0.02 0.02 0.02",  "0.10 0.10 0.10",  "0.05 0.05 0.05", 10,  1.0, 2),
}


def rewrite_mtl(filepath):
    """Rewrite MTL material properties for Phong shading compatibility."""
    if not os.path.exists(filepath):
        return
    with open(filepath, 'r') as f:
        lines = f.readlines()

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
            # Skip original property lines for overridden materials
            if stripped and not stripped.startswith('#'):
                continue
            else:
                out.append(line)
        else:
            out.append(line)

    with open(filepath, 'w') as f:
        f.writelines(out)
    print(f"  Rewrote: {filepath}")


for mtl in ["ev1_chassis_vis.mtl", "ev1_rim.mtl", "ev1_tire.mtl"]:
    rewrite_mtl(os.path.join(OUT_DIR, mtl))

print("\n=== Done ===")
