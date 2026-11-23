#!/usr/bin/env python3
"""Find LM triangles that can be seen from outside but are wound inward.

The LM is drawn with back faces culled, so such a triangle is a hole: the interior
shows through and coplanar inner faces flicker against each other. Rasterises the
model orthographically from 26 directions and reports visible back-facing triangles.

Usage:
  AGC_DUMP_LM=/tmp/lm.txt build/linux/agc-lander --shot /tmp/x.png --after 0.5
  python3 tools/winding_check.py /tmp/lm.txt
A clean model reports only a few slivers (< ~250 px in total).
"""
import sys
import numpy as np

d = np.loadtxt(sys.argv[1])
st = d[:, 0].astype(int); mat = d[:, 1].astype(int); P = d[:, 5:].reshape(-1, 3, 3)
n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
area = np.linalg.norm(n, axis=1) / 2
n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
dirs = []
for a in np.linspace(0, 2 * np.pi, 8, endpoint=False):
    for e in (-0.6, 0.0, 0.6):
        dirs.append((np.cos(e) * np.cos(a), np.sin(e), np.cos(e) * np.sin(a)))
dirs += [(0, 1, 0), (0, -1, 0)]
# Model axes: x is up (stage axis). Rotate the generated directions so "up" (y above) maps to +x.
dirs = [np.array([v[1], v[0], v[2]]) for v in dirs]
RES = 0.015
bad_px = {}
for vdir in dirs:
    vdir = vdir / np.linalg.norm(vdir)          # direction the viewer looks along
    up = np.array([1.0, 0, 0]) if abs(vdir[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(up, vdir); u /= np.linalg.norm(u); v = np.cross(vdir, u)
    U = P @ u; V = P @ v; Z = P @ vdir           # depth along the view direction (smaller = nearer)
    u0, v0 = U.min(), V.min()
    W = int((U.max() - u0) / RES) + 2; H = int((V.max() - v0) / RES) + 2
    zbuf = np.full((H, W), np.inf); ibuf = np.full((H, W), -1)
    facing = n @ vdir                             # > 0: normal points away from the viewer (back face)
    for i in np.nonzero(area > 1e-7)[0]:
        pu = (U[i] - u0) / RES; pv = (V[i] - v0) / RES
        x0, x1 = int(np.floor(pu.min())), int(np.ceil(pu.max())); y0, y1 = int(np.floor(pv.min())), int(np.ceil(pv.max()))
        if x1 - x0 > 2000 or y1 - y0 > 2000: continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        (ax, bx, cx), (ay, by, cy) = pu, pv
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-9: continue
        l1 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / den
        l2 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / den
        l3 = 1 - l1 - l2
        ins = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
        if not ins.any(): continue
        z = l1 * Z[i, 0] + l2 * Z[i, 1] + l3 * Z[i, 2]
        # ties within 1 mm: a front face wins over a back face (double-sided glass, dish front/back)
        z = z + (0.001 if facing[i] > 0 else 0.0)
        sub_z = zbuf[y0:y1 + 1, x0:x1 + 1]; sub_i = ibuf[y0:y1 + 1, x0:x1 + 1]
        win = ins & (z < sub_z)
        sub_z[win] = z[win]; sub_i[win] = i
    vis = ibuf[ibuf >= 0]
    back = vis[facing[vis] > 0.05]
    for i in back:
        bad_px[i] = bad_px.get(i, 0) + 1
rows = sorted(((c, i) for i, c in bad_px.items() if c >= 15), reverse=True)
print(f"{len(rows)} inward-wound triangles visible from outside (>= 15 px of 1.5 cm over 26 views)")
groups = {}
for c, i in rows:
    key = (st[i], mat[i], tuple(np.round(P[i].mean(0), 1)))
    groups.setdefault(key, 0); groups[key] += c
for (s, m, cen), c in sorted(groups.items(), key=lambda kv: -kv[1])[:40]:
    print(f"stage {s} mat {m:2d} near {cen}: {c} px")
