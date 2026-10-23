"""Run the macro's filter exactly as run_filter() will in automatic mode, on several clouds.
Usage (MACRO = macro name, e.g. CleanCloud, or path): eval_v3.py MACRO LAZ[@inject] [...]   (@inject adds known flyers and clumps, as inject_test.py)"""
import math
import random
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ns = macrocode.load(sys.argv[1])


def auto_settings(rel):
    s50, s95 = ns["point_spacing"](rel)
    cell = min(ns["MAX_CELL"], max(ns["MIN_CELL"], round(ns["AUTO_CELL"] * s50 / 0.005) * 0.005))
    return s50, s95, cell, max(1.2 * s50, s95)


variant = {}
while sys.argv[2].startswith("--"):
    variant["_" + sys.argv.pop(2)[2:].replace("-", "_")] = True
print("variant:", variant or "default")
for spec in sys.argv[2:]:
    path, inject = (spec[:-7], True) if spec.endswith("@inject") else (spec, False)
    pts = las_io.load_xyz(path)
    o = pts[0]
    rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
    N0 = len(rel)
    kind = []
    if inject:
        planes0 = ns["local_planes"](rel, 0.05)
        random.seed(7)
        seeds = iter([i for i in random.sample(range(N0), 60000)
                      if planes0.get(ns["cell_key"](rel[i], 0.05)) and planes0[ns["cell_key"](rel[i], 0.05)][2]])
        extra = []
        for _ in range(3000):
            i = next(seeds); nv = planes0[ns["cell_key"](rel[i], 0.05)][1]; off = random.choice((1, 1, -1)) * random.uniform(0.03, 0.30)
            extra.append(tuple(rel[i][k] + nv[k] * off for k in range(3))); kind.append("flyer")
        for _ in range(300):
            i = next(seeds); nv = planes0[ns["cell_key"](rel[i], 0.05)][1]; off = random.choice((1, 1, -1)) * random.uniform(0.03, 0.15)
            for _ in range(random.randint(8, 25)):
                extra.append(tuple(rel[i][k] + nv[k] * off + random.gauss(0, 0.01) for k in range(3))); kind.append("clump")
        rel = rel + extra
    t0 = time.time()
    s50, s95, cell, gap = auto_settings(rel)
    out, counts, info = ns["clean"](rel, cell, None, False, True, True, None, gap, ns["MIN_PIECE"], **variant)
    kept = bytearray(len(rel))
    for i, _ in out:
        kept[i] = 1
    real_removed = sum(1 for i in range(N0) if not kept[i])
    print(f"\n=== {path.split(chr(92))[-1]}{' + injected noise' if inject else ''}: {N0:,} points, "
          f"spacing {s50 * 1000:.1f} / p95 {s95 * 1000:.1f} mm -> cell {cell * 1000:.0f} mm, gap {gap * 1000:.1f} mm ({time.time() - t0:.0f}s)")
    for c, tol, sc, npl in info:
        print(f"  level cell {c * 1000:4.0f} mm: scatter {('%.1f' % (sc * 1000)) if sc else '-':>5} mm, tolerance {tol * 1000:.1f} mm, {npl:,} planes")
    print(f"  real points removed: {100.0 * real_removed / N0:.2f}%  | " +
          ", ".join(f"{k} {100.0 * v / len(rel):.2f}%" for k, v in sorted(counts.items()) if v))
    if inject:
        for kname in ("flyer", "clump"):
            idx = [N0 + j for j, k in enumerate(kind) if k == kname]
            print(f"  injected {kname}s removed: {100.0 * sum(1 for i in idx if not kept[i]) / len(idx):.0f}%")
