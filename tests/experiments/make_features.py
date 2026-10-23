"""Compute objfeat features for labelled LAZ files and save them with labels as .npz.
Usage (training venv python): make_features.py MACRO OUT.npz LABEL=PATH [...]"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402
import objfeat  # noqa: E402

ns = macrocode.load(sys.argv[1])
pts, label = [], []
classes = []
for spec in sys.argv[3:]:
    name, path = spec.split("=", 1)
    classes.append(name)
    p = las_io.load_xyz(path)
    pts += p
    label += [len(classes) - 1] * len(p)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
t0 = time.time()
names, rows = objfeat.features(rel, ns)
X = np.array(rows, dtype=np.float32)
np.savez_compressed(sys.argv[2], X=X, y=np.array(label, dtype=np.int8), xyz=np.array(rel, dtype=np.float32),
                    names=np.array(names), classes=np.array(classes))
print(f"{len(rel):,} points x {X.shape[1]} features in {time.time() - t0:.0f}s -> {sys.argv[2]}")
