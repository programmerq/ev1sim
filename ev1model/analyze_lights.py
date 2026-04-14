#!/usr/bin/env python3
"""Analyze and render diagnostic images of the EV1 chassis light assignments.

Reads ev1_chassis_vis.obj and produces:
  - ev1_lights_top.png    — Top-down view with light faces colour-coded
  - ev1_lights_side.png   — Side view
  - ev1_lights_rear.png   — Rear view

Each lamp group is drawn in a distinct colour so you can verify which mesh
faces have been assigned to which light.

Usage:
  python3 analyze_lights.py [path/to/ev1_chassis_vis.obj]
"""

import sys
import os
from collections import defaultdict
from PIL import Image, ImageDraw, ImageFont

OBJ_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "data", "vehicle", "ev1", "ev1_chassis_vis.obj")
if len(sys.argv) > 1:
    OBJ_PATH = sys.argv[1]

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Colours (R, G, B) for each lamp material
LAMP_COLORS = {
    "headlamp_left":             (255, 255, 100),  # Bright yellow (unsplit)
    "headlamp_right":            (200, 200,  50),  # Darker yellow (unsplit)
    "headlamp_hi_left":          (220, 220, 255),  # Cool white (hi beam)
    "headlamp_hi_right":         (180, 180, 220),  # Darker cool white
    "headlamp_lo_left":          (255, 240, 140),  # Warm white (lo beam)
    "headlamp_lo_right":         (220, 200, 100),  # Darker warm white
    "signal_front_turn_left":    (255, 165,   0),  # Orange
    "signal_front_turn_right":   (200, 130,   0),  # Darker orange
    "signal_front_marker_left":  (255, 200,  80),  # Light orange
    "signal_front_marker_right": (200, 160,  60),  # Darker light orange
    "signal_front_left":         (255, 165,   0),  # Orange (legacy, pre-split)
    "signal_front_right":        (200, 130,   0),  # Darker orange (legacy)
    "stoplamp_left":             (255,  30,  30),  # Red
    "stoplamp_right":            (200,  20,  20),  # Darker red
    "signal_rear_left":          (255, 140,   0),  # Dark orange
    "signal_rear_right":         (200, 110,   0),  # Darker dark orange
    "side_marker_rear_left":     (200,  50,  50),  # Dark red (side marker)
    "side_marker_rear_right":    (150,  30,  30),  # Darker red
    "backup_left":               (180, 180, 255),  # Light blue
    "backup_right":              (130, 130, 200),  # Darker light blue
    "chmsl":                     (255,   0, 255),  # Magenta
    # Non-light materials get grey tones
}
BODY_COLOR = (60, 60, 60)
REFLECTOR_COLOR = (100, 50, 50)  # Dim red for reflectors


def parse_obj(path):
    """Parse OBJ file, return vertices and material-grouped face centroids."""
    vertices = []
    material_faces = defaultdict(list)  # material -> [(cx, cy, cz), ...]
    current_mtl = "unknown"

    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'v' and len(parts) >= 4:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == 'usemtl':
                current_mtl = parts[1]
            elif parts[0] == 'f':
                vindices = [int(p.split('/')[0]) - 1 for p in parts[1:]]
                n = len(vindices)
                cx = sum(vertices[i][0] for i in vindices) / n
                cy = sum(vertices[i][1] for i in vindices) / n
                cz = sum(vertices[i][2] for i in vindices) / n
                material_faces[current_mtl].append((cx, cy, cz))

    return vertices, material_faces


def render_view(material_faces, title, x_axis, y_axis, x_label, y_label,
                img_width=1400, img_height=800, filename="view.png"):
    """Render a 2D scatter plot of face centroids from the given axes.

    x_axis/y_axis: 0=X(fwd), 1=Y(left), 2=Z(up) — which OBJ coordinate
    maps to the horizontal / vertical image axis.
    """
    # Gather all points
    all_pts = []
    for faces in material_faces.values():
        all_pts.extend(faces)
    if not all_pts:
        return

    xs = [p[x_axis] for p in all_pts]
    ys = [p[y_axis] for p in all_pts]

    margin = 80
    plot_w = img_width - 2 * margin
    plot_h = img_height - 2 * margin - 40  # Extra space for legend

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_range = x_max - x_min or 1
    y_range = y_max - y_min or 1

    # Scale to fit, preserving aspect ratio
    scale = min(plot_w / x_range, plot_h / y_range)

    def to_pixel(val_x, val_y):
        px = margin + int((val_x - x_min) * scale)
        py = margin + plot_h - int((val_y - y_min) * scale)  # Flip Y
        return (px, py)

    img = Image.new('RGB', (img_width, img_height), (20, 20, 30))
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 14)
        font_small = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 11)
        font_title = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 18)
    except Exception:
        font = ImageFont.load_default()
        font_small = font
        font_title = font

    # Title
    draw.text((margin, 10), title, fill=(255, 255, 255), font=font_title)

    # Axis labels
    draw.text((img_width // 2, img_height - 25), x_label,
              fill=(180, 180, 180), font=font_small, anchor="mm")
    # Vertical label (just put it on the left)
    draw.text((15, img_height // 2), y_label,
              fill=(180, 180, 180), font=font_small, anchor="mm")

    # Draw body faces first (background), then light faces on top
    for mtl_name, faces in sorted(material_faces.items()):
        if mtl_name in LAMP_COLORS:
            continue  # Draw lights later
        color = REFLECTOR_COLOR if mtl_name == 'redglass' else BODY_COLOR
        for pt in faces:
            px, py = to_pixel(pt[x_axis], pt[y_axis])
            draw.point((px, py), fill=color)

    # Draw light faces on top
    for mtl_name in sorted(LAMP_COLORS.keys()):
        if mtl_name not in material_faces:
            continue
        color = LAMP_COLORS[mtl_name]
        faces = material_faces[mtl_name]
        for pt in faces:
            px, py = to_pixel(pt[x_axis], pt[y_axis])
            # Draw 2x2 pixel for visibility
            draw.rectangle([px-1, py-1, px+1, py+1], fill=color)

    # Legend
    legend_x = img_width - 250
    legend_y = 45
    draw.rectangle([legend_x - 10, legend_y - 5, img_width - 10, legend_y + len(LAMP_COLORS) * 18 + 5],
                    fill=(30, 30, 40), outline=(80, 80, 80))
    for i, (name, color) in enumerate(sorted(LAMP_COLORS.items())):
        y_pos = legend_y + i * 18
        count = len(material_faces.get(name, []))
        draw.rectangle([legend_x, y_pos, legend_x + 12, y_pos + 12], fill=color)
        draw.text((legend_x + 18, y_pos - 1), f"{name} ({count})",
                  fill=(200, 200, 200), font=font_small)

    # Add reflector entry
    y_pos = legend_y + len(LAMP_COLORS) * 18
    count = len(material_faces.get('redglass', []))
    draw.rectangle([legend_x, y_pos, legend_x + 12, y_pos + 12], fill=REFLECTOR_COLOR)
    draw.text((legend_x + 18, y_pos - 1), f"reflector ({count})",
              fill=(200, 200, 200), font=font_small)

    filepath = os.path.join(OUT_DIR, filename)
    img.save(filepath)
    print(f"  Saved: {filepath}")


def main():
    print(f"Analyzing: {OBJ_PATH}")
    vertices, material_faces = parse_obj(OBJ_PATH)
    print(f"  {len(vertices)} vertices, {sum(len(f) for f in material_faces.values())} faces")

    # Print stats per material
    print(f"\n  Material face counts:")
    for mtl in sorted(material_faces.keys()):
        marker = " ← LAMP" if mtl in LAMP_COLORS else ""
        print(f"    {mtl:25s}: {len(material_faces[mtl]):6d}{marker}")

    # Auto-detect coordinate convention.
    # Correct (Chrono): X=forward (longest, ~4.3m), Y=left (~2.0m), Z=up
    # Broken (Blender): Y=forward (longest, ~3.9m), X=lateral (~1.8m), Z=up
    all_pts = [p for faces in material_faces.values() for p in faces]
    x_range = max(p[0] for p in all_pts) - min(p[0] for p in all_pts)
    y_range = max(p[1] for p in all_pts) - min(p[1] for p in all_pts)

    if x_range > y_range:
        fwd_axis, lat_axis = 0, 1
        fwd_label, lat_label = "X", "Y"
        print(f"  Detected Chrono convention: X=forward ({x_range:.2f}m), Y=lateral ({y_range:.2f}m)")
    else:
        fwd_axis, lat_axis = 1, 0
        fwd_label, lat_label = "Y", "X"
        print(f"  Detected Blender convention: Y=forward ({y_range:.2f}m), X=lateral ({x_range:.2f}m)")
        print(f"  NOTE: Re-export with fixed export_meshes.py to get correct Chrono coordinates")

    # Top-down view: forward horizontal, lateral vertical
    print("\nRendering views...")
    render_view(material_faces,
                f"EV1 Lights — Top View ({fwd_label}=forward →, {lat_label}=left ↑)",
                x_axis=fwd_axis, y_axis=lat_axis,
                x_label=f"{fwd_label} (forward →)", y_label=f"{lat_label} (← right | left →)",
                filename="ev1_lights_top.png")

    # Side view: forward horizontal, Z(up) vertical
    render_view(material_faces,
                f"EV1 Lights — Side View ({fwd_label}=forward →, Z=up ↑)",
                x_axis=fwd_axis, y_axis=2,
                x_label=f"{fwd_label} (forward →)", y_label="Z (up ↑)",
                filename="ev1_lights_side.png")

    # Rear view: lateral horizontal, Z(up) vertical — only rear faces
    fwd_min = min(p[fwd_axis] for p in all_pts)
    fwd_max = max(p[fwd_axis] for p in all_pts)
    fwd_quarter = fwd_min + (fwd_max - fwd_min) * 0.25
    rear_faces = {}
    for mtl, faces in material_faces.items():
        rear = [f for f in faces if f[fwd_axis] < fwd_quarter]
        if rear:
            rear_faces[mtl] = rear

    render_view(rear_faces,
                f"EV1 Lights — Rear View ({lat_label}=left →, Z=up ↑) [rear 25%]",
                x_axis=lat_axis, y_axis=2,
                x_label=f"{lat_label} (← right | left →)", y_label="Z (up ↑)",
                img_width=1000, img_height=600,
                filename="ev1_lights_rear.png")

    # Front view: lateral horizontal, Z(up) vertical — only front faces
    fwd_three_quarter = fwd_min + (fwd_max - fwd_min) * 0.75
    front_faces = {}
    for mtl, faces in material_faces.items():
        front = [f for f in faces if f[fwd_axis] > fwd_three_quarter]
        if front:
            front_faces[mtl] = front

    render_view(front_faces,
                f"EV1 Lights — Front View ({lat_label}=left →, Z=up ↑) [front 25%]",
                x_axis=lat_axis, y_axis=2,
                x_label=f"{lat_label} (← right | left →)", y_label="Z (up ↑)",
                img_width=1000, img_height=600,
                filename="ev1_lights_front.png")

    print("\nDone!")


if __name__ == '__main__':
    main()
