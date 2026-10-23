"""Simulate click-to-select on labelled data: for each labelled object (connected piece of a
non-wall label), 'click' its most prominent point and grow through neighbours standing more than h
off a trimmed rock surface. Reports how much of the object is selected and how much wall comes
with it. Usage (MACRO = macro name, e.g. CleanCloud, or path): grow_eval.py MACRO surf_cell_mm trim_mm h_mm[,h_mm...] LABEL=PATH [...]"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402
import objfeat  # noqa: E402

ns = macrocode.load(sys.argv[1])
scell, trim = float(sys.argv[2]) / 1000, float(sys.argv[3]) / 1000
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
t0 = time.time()
planes = objfeat.trimmed_surface(rel, scell, trim, ns)
off = []
for p in rel:
    pl = planes.get(ns["cell_key"](p, scell))
    off.append(abs(ns["off_plane"](p, pl)) if pl is not None else 1e9)
s50, s95 = ns["point_spacing"](rel)
g = max(1.5 * s50, 0.012)
grid = {}
for i, p in enumerate(rel):
    grid.setdefault(ns["cell_key"](p, g), []).append(i)
print(f"{N:,} points; surface {scell * 1000:.0f} mm trim {trim * 1000:.0f} mm; link {g * 1000:.0f} mm ({time.time() - t0:.0f}s)")


def neighbours(i, r):
    x, y, z = rel[i]
    k = ns["cell_key"](rel[i], g)
    r2 = r * r
    for a, b, c in ns["NEIGHBOURS"]:
        for j in grid.get((k[0] + a, k[1] + b, k[2] + c), ()):
            q = rel[j]
            if (q[0] - x) ** 2 + (q[1] - y) ** 2 + (q[2] - z) ** 2 <= r2:
                yield j


# labelled objects = connected pieces of each non-wall label (link 3 cm)
objects = []
seen = set()
for i in range(N):
    if label[i] == "wall" or i in seen:
        continue
    comp, stack = [], [i]
    seen.add(i)
    while stack:
        a = stack.pop()
        comp.append(a)
        x, y, z = rel[a]
        k = ns["cell_key"](rel[a], g)
        for da in range(-2, 3):
            for db in range(-2, 3):
                for dc in range(-2, 3):
                    for j in grid.get((k[0] + da, k[1] + db, k[2] + dc), ()):
                        if j not in seen and label[j] == label[i]:
                            q = rel[j]
                            if (q[0] - x) ** 2 + (q[1] - y) ** 2 + (q[2] - z) ** 2 <= 0.03 ** 2:
                                seen.add(j)
                                stack.append(j)
    if len(comp) >= 15:
        objects.append((label[i], comp))
print(f"{len(objects)} labelled objects: " + ", ".join(f"{c} {sum(1 for l, _ in objects if l == c)}" for c in sorted(set(l for l, _ in objects))))

CAP = 30000
for h in hs:
    res = {}
    for lab, comp in objects:
        seed = max((i for i in comp if off[i] < 1e9), key=lambda i: off[i], default=comp[0])
        sel, stack = {seed}, [seed]
        while stack and len(sel) < CAP:
            a = stack.pop()
            for j in neighbours(a, g):
                if j not in sel and off[j] > h:
                    sel.add(j)
                    stack.append(j)
        cs = set(comp)
        hit = len(sel & cs)
        wall = sum(1 for j in sel if label[j] == "wall")
        res.setdefault(lab, []).append((hit / len(cs), wall, len(sel) >= CAP))
    line = f"  h {h * 1000:4.0f} mm:"
    for lab, r in sorted(res.items()):
        rec = sorted(x[0] for x in r)
        walls = sorted(x[1] for x in r)
        line += (f" | {lab}: median {100 * rec[len(rec) // 2]:.0f}% of object selected, "
                 f"{sum(1 for x in r if x[0] >= 0.8)}/{len(r)} objects >= 80%, wall points dragged in median {walls[len(walls) // 2]:,}"
                 f" (worst {walls[-1]:,}), ran away {sum(1 for x in r if x[2])}")
    print(line)
