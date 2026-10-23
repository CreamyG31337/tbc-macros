"""Run CGSeparateObjects' own select_round / select_tube on the labelled objects, with simulated clicks
as in local_select_eval.py (anchor: point nearest the object's centre; hose: object points every
~1 m along its main axis). Usage (MACRO = macro name, e.g. CleanCloud, or path): test_separate.py MACRO LABEL=PATH [...]"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ns = macrocode.load(sys.argv[1])
pts, label = [], []
for spec in sys.argv[2:]:
    name, path = spec.split("=", 1)
    p = las_io.load_xyz(path)
    pts += p
    label += [name] * len(p)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
N = len(rel)
grid = ns["build_grid"](rel)
link = ns["link_distance"](rel, grid)
print(f"link {link * 1000:.0f} mm")

objects, seen = [], set()
for i in range(N):
    if label[i] == "wall" or i in seen:
        continue
    comp, stack = [], [i]
    seen.add(i)
    while stack:
        a = stack.pop()
        comp.append(a)
        for j in ns["near"](rel, grid, rel[a], 0.03):
            if j not in seen and label[j] == label[i]:
                seen.add(j)
                stack.append(j)
    if len(comp) >= 15:
        objects.append((label[i], comp))


def main_axis(comp):
    n = len(comp)
    m = [sum(rel[i][k] for i in comp) / n for k in range(3)]
    c = [0.0] * 6
    for i in comp:
        dx, dy, dz = rel[i][0] - m[0], rel[i][1] - m[1], rel[i][2] - m[2]
        c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
    cov = [v / n for v in c]
    (l1, _, _), _ = ns["sym3_eig"](cov)
    a, b, d, e, f, g = cov
    M = [[a - l1, e, f], [e, b - l1, g], [f, g, d - l1]]
    best, bn = (0, 0, 1), -1
    for u, v in ((M[0], M[1]), (M[0], M[2]), (M[1], M[2])):
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        nn = sum(x * x for x in cr)
        if nn > bn:
            best, bn = cr, nn
    return m, [x / (math.sqrt(bn) or 1) for x in best]


res = {}
for lab, comp in objects:
    m, ax = main_axis(comp)
    proj = sorted((sum((rel[i][k] - m[k]) * ax[k] for k in range(3)), i) for i in comp)
    length = proj[-1][0] - proj[0][0]
    if lab == "anchors" or length < 0.3:
        click = rel[min(comp, key=lambda i: sum((rel[i][k] - m[k]) ** 2 for k in range(3)))]
        sel, _ = ns["select_round"](rel, grid, click, 0.10, 0.008, link)
    else:
        k = max(2, int(round(length / 1.0)) + 1)
        clicks = [rel[min(proj, key=lambda t: abs(t[0] - (proj[0][0] + length * j / (k - 1))))[1]] for j in range(k)]
        sel, _ = ns["select_tube"](rel, grid, clicks, 0.05, 0.008, link)
    cs = set(comp)
    res.setdefault(lab, []).append((len(sel & cs) / len(cs), sum(1 for j in sel if label[j] == "wall"), len(cs)))
for lab, rr in sorted(res.items()):
    rec = sorted(x[0] for x in rr)
    walls = sorted(x[1] for x in rr)
    print(f"{lab}: {len(rr)} objects, median {100 * rec[len(rec) // 2]:.0f}% selected, {sum(1 for x in rr if x[0] >= 0.8)} >= 80%, "
          f"wall points dragged in total {sum(walls):,} vs {sum(x[2] for x in rr):,} object points (median {walls[len(walls) // 2]})")


