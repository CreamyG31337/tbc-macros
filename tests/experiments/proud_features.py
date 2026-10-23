"""How far each labelled class stands off a broad rock surface. The surface is fitted with planes
at a coarse cell, then refitted leaving out points more than `trim` off (so hoses and anchors don't
pull it). Prints, per label, the share of points beyond several distances.
Usage (MACRO = macro name, e.g. CleanCloud, or path): proud_features.py MACRO cell_mm trim_mm LABEL=PATH [...]"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ns = macrocode.load(sys.argv[1])
cell, trim = float(sys.argv[2]) / 1000, float(sys.argv[3]) / 1000
pts, label = [], []
for spec in sys.argv[4:]:
    name, path = spec.split("=", 1)
    p = las_io.load_xyz(path)
    pts += p
    label += [name] * len(p)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
N = len(rel)
key = ns["cell_key"]
planes = ns["local_planes"](rel, cell)
for _ in range(2):  # trim and refit: the rock surface without things standing off it
    use = bytearray(N)
    for i, p in enumerate(rel):
        pl = planes.get(key(p, cell))
        use[i] = 1 if pl is not None and abs(ns["off_plane"](p, pl)) <= trim else 0
    planes = ns["local_planes"](rel, cell, None, use)
d = []
for p in rel:
    pl = planes.get(key(p, cell))
    d.append(abs(ns["off_plane"](p, pl)) if pl is not None else float("inf"))
ths = (0.02, 0.03, 0.05, 0.08, 0.12)
print(f"cell {cell * 1000:.0f} mm (planes over {3 * cell * 1000:.0f} mm), trim {trim * 1000:.0f} mm")
print("  label      points   no plane  " + "  ".join(f">{t * 1000:3.0f}mm" for t in ths))
for name in sorted(set(label)):
    idx = [i for i in range(N) if label[i] == name]
    nop = sum(1 for i in idx if d[i] == float("inf"))
    print(f"  {name:8s} {len(idx):9,}  {100.0 * nop / len(idx):6.1f}%  " +
          "  ".join(f"{100.0 * sum(1 for i in idx if d[i] > t) / len(idx):6.1f}%" for t in ths))
