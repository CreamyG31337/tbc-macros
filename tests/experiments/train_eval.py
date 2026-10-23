"""Train on one labelled half, test on the other: per-class precision / recall at point level.
Usage: train_eval.py TRAIN.npz TEST.npz"""
import sys
import time

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.tree import DecisionTreeClassifier

tr, te = np.load(sys.argv[1]), np.load(sys.argv[2])
classes = list(tr["classes"])
rng = np.random.default_rng(0)
Xtr, ytr = tr["X"], tr["y"]
# all object points, a sample of wall points (the wall outnumbers them ~80 to 1)
wall = np.flatnonzero(ytr == 0)
idx = np.concatenate([rng.choice(wall, size=min(len(wall), 300000), replace=False), np.flatnonzero(ytr != 0)])
Xtr, ytr = Xtr[idx], ytr[idx]
Xte, yte = te["X"], te["y"]
print(f"train {len(ytr):,} points ({', '.join(f'{c} {np.sum(ytr == k):,}' for k, c in enumerate(classes))}); "
      f"test {len(yte):,} ({', '.join(f'{c} {np.sum(yte == k):,}' for k, c in enumerate(classes))})")


def report(name, model):
    t0 = time.time()
    model.fit(Xtr, ytr)
    p = model.predict(Xte)
    parts = []
    for k, c in enumerate(classes):
        if k == 0:
            continue
        tp = np.sum((p == k) & (yte == k)); fp = np.sum((p == k) & (yte != k)); fn = np.sum((p != k) & (yte == k))
        prec = tp / max(1, tp + fp); rec = tp / max(1, tp + fn)
        parts.append(f"{c}: found {100 * rec:.0f}% of them, {100 * prec:.0f}% of flagged are right ({fp:,} false)")
    wall_lost = np.sum((p != 0) & (yte == 0)) / np.sum(yte == 0)
    print(f"{name:28s} ({time.time() - t0:.0f}s) | " + " | ".join(parts) + f" | wall flagged {100 * wall_lost:.2f}%")
    return model


for depth in (6, 10, 14):
    report(f"decision tree depth {depth}", DecisionTreeClassifier(max_depth=depth, class_weight="balanced", random_state=0))
rf = report("random forest 200 x depth 16",
            RandomForestClassifier(n_estimators=200, max_depth=16, class_weight="balanced_subsample", n_jobs=-1, random_state=0))
imp = sorted(zip(rf.feature_importances_, tr["names"]), reverse=True)[:10]
print("most useful features: " + ", ".join(f"{n} {v:.2f}" for v, n in imp))
