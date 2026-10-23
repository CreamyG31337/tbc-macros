"""Crop LAS/LAZ files to a box with las2las (TBC's copy).
Usage: crop.py OUT_DIR minx miny minz maxx maxy maxz NAME=PATH [NAME=PATH ...]"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402

out_dir = sys.argv[1]
box = [float(v) for v in sys.argv[2:8]]
for spec in sys.argv[8:]:
    name, path = spec.split("=", 1)
    out = os.path.join(out_dir, name + ".laz")
    t0 = time.time()
    las_io.run_las2las(["-i", path, "-keep_xyz"] + [f"{v:.3f}" for v in box] + ["-o", out])
    h = las_io.las_header(out)
    print(f"{name}: {h['points']:,} points in {time.time() - t0:.0f}s, "
          f"extent {h['max'][0] - h['min'][0]:.1f} x {h['max'][1] - h['min'][1]:.1f} x {h['max'][2] - h['min'][2]:.1f} m -> {out}")
