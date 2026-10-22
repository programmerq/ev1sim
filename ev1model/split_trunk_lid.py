#!/usr/bin/env python3
"""
Move the trunk lid out of the chassis visual mesh into its own OBJ, so the
sim can swing it open.

The chassis OBJ is one Blender export with every body object joined, so the
lid is still its own connected shell in it (no vertices shared with the body).
This script finds that shell, plus the small parts mounted on it (badges, the
lock cylinder, the inner liner: every other shell whose vertices all lie within
ATTACH_M of the lid), writes them to ev1_trunk_lid.obj/.mtl, and drops their
faces from ev1_chassis_vis.obj.  Vertex, UV and normal lines stay in the
chassis file (unreferenced ones are harmless), so the change there is face
lines only and every remaining index is unchanged.

Run once on a chassis OBJ that still contains the lid:

    python3 ev1model/split_trunk_lid.py [ev1model_dir]

then copy ev1_chassis_vis.obj and ev1_trunk_lid.{obj,mtl} to data/vehicle/ev1/.
Run on an already-split mesh it finds no lid and exits non-zero.
"""

import os
import sys

# The lid shell: the largest carpaint shell lying wholly behind the rear
# glass (chassis x < LID_MAX_X) and above the bumper (z > LID_MIN_Z).
LID_MAX_X = -1.2
LID_MIN_Z = 0.55
# Parts within this distance of the lid, for every vertex, ride on it.
ATTACH_M = 0.035


def read_obj(path):
    v, vt, vn, faces, header = [], [], [], [], []
    mat = None
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                v.append(line)
            elif line.startswith('vt '):
                vt.append(line)
            elif line.startswith('vn '):
                vn.append(line)
            elif line.startswith('usemtl '):
                mat = line.split()[1]
            elif line.startswith('f '):
                corners = [tuple(int(x) - 1 for x in c.split('/')) for c in line.split()[1:]]
                faces.append((corners, mat))
            elif line.startswith('mtllib') or line.startswith('#'):
                header.append(line)
    pos = [tuple(map(float, l.split()[1:4])) for l in v]
    return pos, v, vt, vn, faces


def shells(n_verts, faces):
    parent = list(range(n_verts))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for corners, _ in faces:
        r = find(corners[0][0])
        for c in corners[1:]:
            s = find(c[0])
            if s != r:
                parent[s] = r
    by_shell = {}
    for i, (corners, _) in enumerate(faces):
        by_shell.setdefault(find(corners[0][0]), []).append(i)
    return by_shell


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    chassis_obj = os.path.join(d, 'ev1_chassis_vis.obj')
    pos, v_lines, vt_lines, vn_lines, faces = read_obj(chassis_obj)
    groups = shells(len(pos), faces)

    def verts_of(face_ids):
        return {c[0] for fi in face_ids for c in faces[fi][0]}

    lid_key, lid_size = None, 0
    for key, fids in groups.items():
        if any(faces[fi][1] != 'carpaint' for fi in fids):
            continue
        vs = verts_of(fids)
        if all(pos[i][0] < LID_MAX_X and pos[i][2] > LID_MIN_Z for i in vs) and len(fids) > lid_size:
            lid_key, lid_size = key, len(fids)
    if lid_key is None or lid_size < 1000:
        sys.exit('no trunk lid shell in %s (already split?)' % chassis_obj)

    lid_verts = verts_of(groups[lid_key])
    cell = ATTACH_M
    grid = {}
    for i in lid_verts:
        p = pos[i]
        grid.setdefault((int(p[0] // cell), int(p[1] // cell), int(p[2] // cell)), []).append(p)

    def near_lid(p):
        k = (int(p[0] // cell), int(p[1] // cell), int(p[2] // cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for q in grid.get((k[0] + dx, k[1] + dy, k[2] + dz), ()):
                        if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 <= ATTACH_M ** 2:
                            return True
        return False

    moved = set(groups[lid_key])
    for key, fids in groups.items():
        if key != lid_key and all(near_lid(pos[i]) for i in verts_of(fids)):
            moved.update(fids)

    # Lid OBJ: re-indexed, faces grouped by material.
    vmap, tmap, nmap = {}, {}, {}
    out_v, out_t, out_n = [], [], []
    by_mat = {}
    for fi in sorted(moved):
        corners, mat = faces[fi]
        refs = []
        for c in corners:
            if c[0] not in vmap:
                vmap[c[0]] = len(out_v) + 1
                out_v.append(v_lines[c[0]])
            if c[1] not in tmap:
                tmap[c[1]] = len(out_t) + 1
                out_t.append(vt_lines[c[1]])
            if c[2] not in nmap:
                nmap[c[2]] = len(out_n) + 1
                out_n.append(vn_lines[c[2]])
            refs.append('%d/%d/%d' % (vmap[c[0]], tmap[c[1]], nmap[c[2]]))
        by_mat.setdefault(mat, []).append('f ' + ' '.join(refs) + '\n')

    with open(os.path.join(d, 'ev1_trunk_lid.obj'), 'w') as f:
        f.write('# Trunk lid, split from ev1_chassis_vis.obj by split_trunk_lid.py\n')
        f.write('mtllib ev1_trunk_lid.mtl\no ev1_trunk_lid\n')
        f.writelines(out_v)
        f.writelines(out_t)
        f.writelines(out_n)
        f.write('s 1\n')
        for mat in sorted(by_mat):
            f.write('usemtl %s\n' % mat)
            f.writelines(by_mat[mat])

    # Lid MTL: the same material blocks as the chassis.
    blocks, cur = {}, None
    with open(os.path.join(d, 'ev1_chassis_vis.mtl')) as f:
        for line in f:
            if line.startswith('newmtl '):
                cur = line.split()[1]
                blocks[cur] = [line]
            elif cur is not None and line.strip():
                blocks[cur].append(line)
    with open(os.path.join(d, 'ev1_trunk_lid.mtl'), 'w') as f:
        for mat in sorted(by_mat):
            f.writelines(blocks[mat])
            f.write('\n')

    # Chassis OBJ: drop the moved faces' lines and nothing else.
    face_no = 0
    kept = []
    with open(chassis_obj) as f:
        for line in f:
            if line.startswith('f '):
                face_no += 1
                if face_no - 1 in moved:
                    continue
            kept.append(line)
    with open(chassis_obj, 'w') as f:
        f.writelines(kept)

    print('trunk lid: %d faces (%d shells) moved, %s' % (
        len(moved), sum(1 for k, fids in groups.items() if fids[0] in moved),
        ', '.join('%s %d' % (m, len(by_mat[m])) for m in sorted(by_mat))))


if __name__ == '__main__':
    main()
