"""Run the macro's filter (automatic settings, as run_filter() does) on labelled files combined, and
report what share of each label it removes, why, and where. Also writes a face image of the
removed 'keep' points (red) and caught 'remove' points (blue).
Usage (MACRO = macro name, e.g. CleanCloud, or path): eval_labelled.py MACRO OUT_PREFIX LABEL=PATH [LABEL=PATH ...]"""
import math
import struct
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ns = macrocode.load(sys.argv[1])
prefix = sys.argv[2]
pts, label = [], []
for spec in sys.argv[3:]:
    name, path = spec.split("=", 1)
    p = las_io.load_xyz(path)
    pts += p
    label += [name] * len(p)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
N = len(rel)
t0 = time.time()
s50, s95 = ns["point_spacing"](rel)
cell = min(ns["MAX_CELL"], max(ns["MIN_CELL"], round(ns["AUTO_CELL"] * s50 / 0.005) * 0.005))
gap = max(1.2 * s50, s95)
out, counts, info = ns["clean"](rel, cell, None, False, True, True, None, gap, ns["MIN_PIECE"])
kept = bytearray(N)
for i, _ in out:
    kept[i] = 1
print(f"{N:,} points, spacing {s50 * 1000:.1f} / p95 {s95 * 1000:.1f} mm -> cell {cell * 1000:.0f} mm, "
      f"tol {info[0][1] * 1000:.1f} mm, gap {gap * 1000:.1f} mm ({time.time() - t0:.0f}s)")
print("  " + ", ".join(f"{k} {100.0 * v / N:.2f}%" for k, v in sorted(counts.items()) if v))
for name in sorted(set(label)):
    idx = [i for i in range(N) if label[i] == name]
    rem = sum(1 for i in idx if not kept[i])
    print(f"  {name:8s}: {len(idx):,} points, removed {rem:,} ({100.0 * rem / len(idx):.2f}%)")

# why 'keep' points were removed: re-run the per-point decision summary by category
small = ns["small_pieces"](rel, gap, ns["MIN_PIECE"])
cats = {"small piece": 0, "plane test": 0}
for i in range(N):
    if label[i] == "keep" and not kept[i]:
        cats["small piece" if small[i] else "plane test"] += 1
print("  keep points removed by: " + ", ".join(f"{k} {v:,}" for k, v in cats.items()))

# face image: PCA frame of the 'keep' points
kidx = [i for i in range(0, N, max(1, N // 200000)) if label[i] == "keep"]
n = len(kidx)
m = [sum(rel[i][k] for i in kidx) / n for k in range(3)]
c = [0.0] * 6
for i in kidx:
    dx, dy, dz = rel[i][0] - m[0], rel[i][1] - m[1], rel[i][2] - m[2]
    c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
_, w = ns["sym3_eig"]([x / n for x in c])
d = w[2]; v = [-d * w[0], -d * w[1], 1 - d * w[2]]; vn = math.sqrt(sum(x * x for x in v)); v = [x / vn for x in v]
u = [v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0]]
uvw = [(sum((p[k] - m[k]) * u[k] for k in range(3)), sum((p[k] - m[k]) * v[k] for k in range(3)),
        sum((p[k] - m[k]) * w[k] for k in range(3))) for p in rel]
us = [a for a, _, _ in uvw]; vs = [b for _, b, _ in uvw]; ws = sorted(cc for _, _, cc in uvw)
u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
wlo, whi = ws[N // 50], ws[-N // 50]
W = 1600; H = int(W * (v1 - v0) / (u1 - u0)) + 1; px = (u1 - u0) / W
zbuf = [-1e9] * (W * H); col = bytearray(b"\xff" * (W * H * 3)); marks = []
for i in range(N):
    a, b, cc = uvw[i]
    x = min(W - 1, int((a - u0) / px)); y = min(H - 1, int((v1 - b) / px)); k = y * W + x
    if not kept[i]:
        marks.append((x, y, b"\xe0\x20\x20" if label[i] == "keep" else b"\xd0\x20\xd0"))
    elif label[i] != "keep":
        marks.append((x, y, b"\x20\x60\xe0" if label[i] == "anchors" else b"\xf0\xa0\x20"))
    if cc > zbuf[k]:
        zbuf[k] = cc
        g = int(60 + 170 * min(1, max(0, (cc - wlo) / max(1e-9, whi - wlo))))
        col[3 * k:3 * k + 3] = bytes((g, g, g))
for x, y, rgb in marks:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if 0 <= x + dx < W and 0 <= y + dy < H:
                k = 3 * ((y + dy) * W + x + dx)
                col[k:k + 3] = rgb
raw = b"".join(b"\x00" + bytes(col[y * W * 3:(y + 1) * W * 3]) for y in range(H))
chunk = lambda t, dd: struct.pack(">I", len(dd)) + t + dd + struct.pack(">I", zlib.crc32(t + dd) & 0xFFFFFFFF)
with open(prefix + ".png", "wb") as f:
    f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
print(f"wrote {prefix}.png ({W}x{H}, {px * 1000:.0f} mm/px): grey keep, blue anchors, orange other labels "
      f"(kept by the filter); red = keep points removed, magenta = labelled points removed")
