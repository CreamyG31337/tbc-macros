"""Simulate click-based selection with a local rock background, on labelled data.

Anchor (one click): points within R of the click that stand more than h off a plane fitted to the
ring R..2.5R around it. Hose (clicks along it every ~1 m): points within r of the click polyline
that stand more than h off a plane fitted per segment to points r..2.5r from the line.
Clicks are simulated on the labelled object's own points. Reports selected share of the object
and wall points dragged in. Usage (MACRO = macro name, e.g. CleanCloud, or path): local_select_eval.py MACRO R_mm r_mm h_mm[,h...] LABEL=PATH [...]"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ns = macrocode.load(sys.argv[1])
R, r = float(sys.argv[2]) / 1000, float(sys.argv[3]) / 1000
hs = [float(v) / 1000 for v in sys.argv[4].split(",")]
pts, label = [], []
for spec in sys.argv[5:]:
    name, path = spec.split("=", 1)
    p = las_io.load_xyz(path)
    pts += p
    label += [name] * len(p)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
N = len(rel)
G = 0.05
grid = {}
for i, p in enumerate(rel):
    grid.setdefault(ns["cell_key"](p, G), []).append(i)


def near(c, rad):
    """Indices within rad of point c."""
    k = ns["cell_key"](c, G)
    reach = int(math.ceil(rad / G))
    r2 = rad * rad
    out = []
    for a in range(-reach, reach + 1):
        for b in range(-reach, reach + 1):
            for d in range(-reach, reach + 1):
                for j in grid.get((k[0] + a, k[1] + b, k[2] + d), ()):
                    q = rel[j]
                    if (q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2 + (q[2] - c[2]) ** 2 <= r2:
                        out.append(j)
    return out


def fit_plane(idx):
    """Least-squares plane through rel[idx], refitted twice without the worst 20%."""
    for _ in range(3):
        n = len(idx)
        if n < 10:
            return None
        m = [sum(rel[i][k] for i in idx) / n for k in range(3)]
        c = [0.0] * 6
        for i in idx:
            dx, dy, dz = rel[i][0] - m[0], rel[i][1] - m[1], rel[i][2] - m[2]
            c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
        _, nv = ns["sym3_eig"]([v / n for v in c])
        pl = (tuple(m), nv, True)
        d = sorted((abs(ns["off_plane"](rel[i], pl)), i) for i in idx)
        idx = [i for _, i in d[:max(10, int(0.8 * len(d)))]]
    return pl


def seg_dist(p, a, b):
    """Distance from p to segment ab, and the position along it (0..1)."""
    ab = [b[k] - a[k] for k in range(3)]
    L2 = sum(v * v for v in ab) or 1e-12
    t = max(0.0, min(1.0, sum((p[k] - a[k]) * ab[k] for k in range(3)) / L2))
    q = [a[k] + t * ab[k] for k in range(3)]
    return math.sqrt(sum((p[k] - q[k]) ** 2 for k in range(3))), t


# labelled objects = connected pieces of each non-wall label (link 3 cm)
objects, seen = [], set()
for i in range(N):
    if label[i] == "wall" or i in seen:
        continue
    comp, stack = [], [i]
    seen.add(i)
    while stack:
        a = stack.pop()
        comp.append(a)
        for j in near(rel[a], 0.03):
            if j not in seen and label[j] == label[i]:
                seen.add(j)
                stack.append(j)
    if len(comp) >= 15:
        objects.append((label[i], comp))


def clicks_along(comp):
    """Simulated clicks: object points nearest to positions every ~1 m along its main axis."""
    n = len(comp)
    m = [sum(rel[i][k] for i in comp) / n for k in range(3)]
    c = [0.0] * 6
    for i in comp:
        dx, dy, dz = rel[i][0] - m[0], rel[i][1] - m[1], rel[i][2] - m[2]
        c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
    cov = [v / n for v in c]
    (l1, _, _), _ = ns["sym3_eig"](cov)
    # main axis from the largest eigenvalue (same cross-product trick as sym3_eig)
    a, b, d, e, f, g = cov
    M = [[a - l1, e, f], [e, b - l1, g], [f, g, d - l1]]
    best, bn = (0, 0, 1), -1
    for u, v in ((M[0], M[1]), (M[0], M[2]), (M[1], M[2])):
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        nn = sum(x * x for x in cr)
        if nn > bn:
            best, bn = cr, nn
    ax = [x / (math.sqrt(bn) or 1) for x in best]
    proj = sorted((sum((rel[i][k] - m[k]) * ax[k] for k in range(3)), i) for i in comp)
    lo, hi = proj[0][0], proj[-1][0]
    k = max(2, int(round((hi - lo) / 1.0)) + 1)
    return [rel[min(proj, key=lambda t: abs(t[0] - (lo + (hi - lo) * j / (k - 1))))[1]] for j in range(k)], hi - lo


for h in hs:
    res = {}
    for lab, comp in objects:
        cs = set(comp)
        clicks, length = clicks_along(comp)
        if lab == "anchors" or length < 0.3:  # one click, sphere
            n = len(comp)
            cen = tuple(sum(rel[i][k] for i in comp) / n for k in range(3))
            click = rel[min(comp, key=lambda i: sum((rel[i][k] - cen[k]) ** 2 for k in range(3)))]
            inner = near(click, R)
            ring = [j for j in near(click, 2.5 * R) if j not in set(inner)]
            pl = fit_plane(ring)
            sel = set(j for j in inner if pl is None or abs(ns["off_plane"](rel[j], pl)) > h)
        else:  # clicks along a hose, tube per segment
            sel = set()
            for a, b in zip(clicks, clicks[1:]):
                mid = tuple((a[k] + b[k]) / 2 for k in range(3))
                half = math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3))) / 2
                cand = near(mid, half + 2.5 * r)
                inner, ring = [], []
                for j in cand:
                    dist, t = seg_dist(rel[j], a, b)
                    if dist <= r:
                        inner.append(j)
                    elif dist <= 2.5 * r and 0 < t < 1:
                        ring.append(j)
                pl = fit_plane(ring)
                sel |= set(j for j in inner if pl is None or abs(ns["off_plane"](rel[j], pl)) > h)
        hit = len(sel & cs)
        wall = sum(1 for j in sel if label[j] == "wall")
        res.setdefault(lab, []).append((hit / len(cs), wall, len(cs)))
    line = f"  R {R * 1000:.0f} r {r * 1000:.0f} h {h * 1000:3.0f} mm:"
    for lab, rr in sorted(res.items()):
        rec = sorted(x[0] for x in rr)
        walls = sorted(x[1] for x in rr)
        tot = sum(x[2] for x in rr)
        line += (f" | {lab}: median {100 * rec[len(rec) // 2]:.0f}% selected, {sum(1 for x in rr if x[0] >= 0.8)}/{len(rr)} >= 80%,"
                 f" wall dragged in: total {sum(walls):,} vs {tot:,} object points (median {walls[len(walls) // 2]})")
    print(line)
