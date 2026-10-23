"""Run CGCleanCloud's filter on a LAS/LAZ file and render the result.

  analyze_laz.py MACRO LAZ OUT_PREFIX --cell 0.05 --tol 0.01 [--face] [--section V U0 U1] [--extra k=v ...]

--face: front view of the face, kept points grey (shaded by depth), removed red.
--section V U0 U1: profile of the slice |v - V| < 2 cm between u = U0..U1 (u along the face,
  v up the face, w out of the face), 2 mm per pixel; kept black, removed red.
--extra passes keyword arguments through to clean() (for trying new options).
"""
import argparse
import math
import struct
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lasread as las_io  # noqa: E402
import macrocode  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("macro"); ap.add_argument("laz"); ap.add_argument("prefix")
ap.add_argument("--cell", type=float, default=0.05); ap.add_argument("--tol", type=float, default=0.01)
ap.add_argument("--face", action="store_true")
ap.add_argument("--section", type=float, nargs=3, action="append", default=[])
ap.add_argument("--extra", nargs="*", default=[])
ap.add_argument("--side", type=float, nargs=2, action="append", default=[],
                help="U0 U1: side view (depth across, up the face down the page) of the slab u0..u1; "
                     "black = kept as flat, blue = kept as rough, red = removed")
ap.add_argument("--vrange", type=float, nargs=2)
ap.add_argument("--plan", action="store_true")
ap.add_argument("--halfw", type=float, default=0.02, help="half width of --section slices (m)")
ap.add_argument("--px", type=float, default=0.005)
args = ap.parse_args()

ns = macrocode.load(args.macro)
extra = {}
for kv in args.extra:
    k, val = kv.split("=")
    extra[k] = int(val) if val.isdigit() else float(val) if val.replace(".", "", 1).replace("-", "", 1).isdigit() else (val == "True")

t0 = time.time()
pts = las_io.load_xyz(args.laz)
o = pts[0]
rel = [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts]
N = len(rel)
print(f"loaded {N:,} points in {time.time() - t0:.1f}s")

# wall frame: w = normal of a global plane fit, v = world up within the face, u = along the face
step = max(1, N // 200000)
smp = rel[::step]
n = len(smp)
m = [sum(p[i] for p in smp) / n for i in range(3)]
c = [0.0] * 6
for p in smp:
    dx, dy, dz = p[0] - m[0], p[1] - m[1], p[2] - m[2]
    c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
_, w = ns["sym3_eig"]([x / n for x in c])
d = w[2]
v = [-d * w[0], -d * w[1], 1 - d * w[2]]
vn = math.sqrt(sum(x * x for x in v)); v = [x / vn for x in v]
u = [v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0]]
if args.plan:  # mostly-ground clouds: view from above (u = east, v = north, w = up)
    u, v, w = [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]


def proj(p):
    q = (p[0] - m[0], p[1] - m[1], p[2] - m[2])
    return (q[0] * u[0] + q[1] * u[1] + q[2] * u[2], q[0] * v[0] + q[1] * v[1] + q[2] * v[2],
            q[0] * w[0] + q[1] * w[1] + q[2] * w[2])


uvw = [proj(p) for p in rel]

t0 = time.time()
out, counts, info = ns["clean"](rel, args.cell, args.tol, False, True, True, **extra)
print(f"cell {args.cell * 1000:.0f} mm, tol {args.tol * 1000:.1f} mm {extra or ''}: {time.time() - t0:.1f}s, levels "
      + "; ".join(f"{c * 1000:.0f} mm tol {t * 1000:.1f}" for c, t, _, _ in info))
for k in sorted(counts):
    if counts[k]:
        print(f"  {k}: {counts[k]:,} ({100.0 * counts[k] / N:.2f}%)")
kept = bytearray(N)
for i, _ in out:
    kept[i] = 1
print(f"  removed total: {N - len(out):,} ({100.0 * (N - len(out)) / N:.2f}%)")


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    print("wrote", path)


if args.face:
    us = [a for a, _, _ in uvw]; vs = [b for _, b, _ in uvw]; ws = sorted(cc for _, _, cc in uvw)
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    wlo, whi = ws[N // 50], ws[-N // 50]
    W = 1600; H = int(W * (v1 - v0) / (u1 - u0)) + 1; px = (u1 - u0) / W
    zbuf = [-1e9] * (W * H); col = bytearray(b"\xff" * (W * H * 3)); red = bytearray(W * H)
    for i in range(N):
        a, b, cc = uvw[i]
        x = min(W - 1, int((a - u0) / px)); y = min(H - 1, int((v1 - b) / px)); k = y * W + x
        if not kept[i]:
            red[k] = 1
        if cc > zbuf[k]:
            zbuf[k] = cc
            g = int(60 + 170 * min(1, max(0, (cc - wlo) / max(1e-9, whi - wlo))))
            col[3 * k:3 * k + 3] = bytes((g, g, g))
    for k in range(W * H):
        if red[k]:
            col[3 * k:3 * k + 3] = b"\xe0\x20\x20"
    print(f"face: u {u0:.1f}..{u1:.1f} m, v {v0:.1f}..{v1:.1f} m, {px * 1000:.0f} mm/px")
    write_png(args.prefix + "_face.png", W, H, col)

if args.side:
    planes = ns["local_planes"](rel, args.cell)
    rough = bytearray(N)
    for i in range(N):
        pl = planes.get(ns["cell_key"](rel[i], args.cell))
        if pl is not None and not pl[2]:
            rough[i] = 1
for si, (ua, ub) in enumerate(args.side):
    sel = [i for i in range(N) if ua <= uvw[i][0] <= ub
           and (not args.vrange or args.vrange[0] <= uvw[i][1] <= args.vrange[1])]
    ws = sorted(uvw[i][2] for i in sel); vs = [uvw[i][1] for i in sel]
    wa, wb = ws[0] - 0.05, ws[-1] + 0.05
    va, vb = min(vs), max(vs)
    px = args.px
    W = int((wb - wa) / px) + 1; H = int((vb - va) / px) + 1
    col = bytearray(b"\xff" * (W * H * 3))
    order = sorted(sel, key=lambda i: (0 if kept[i] and not rough[i] else 1 if kept[i] else 2))
    for i in order:  # removed drawn last
        x = int((uvw[i][2] - wa) / px); y = int((vb - uvw[i][1]) / px)
        k = 3 * (y * W + x)
        col[k:k + 3] = (b"\x30\x30\x30" if not rough[i] else b"\x20\x60\xe0") if kept[i] else b"\xe0\x20\x20"
    nr = sum(1 for i in sel if kept[i] and rough[i]); nrem = sum(1 for i in sel if not kept[i])
    print(f"side u {ua}..{ub}: {len(sel):,} points, rough kept {nr:,}, removed {nrem:,}; {W}x{H} px at {px * 1000:.0f} mm/px "
          f"(left = into the rock, right = out of the face)")
    write_png(f"{args.prefix}_side{si}.png", W, H, col)

for si, (vlev, ua, ub) in enumerate(args.section):
    sel = [i for i in range(N) if abs(uvw[i][1] - vlev) < args.halfw and ua <= uvw[i][0] <= ub]
    ws = [uvw[i][2] for i in sel]
    wa, wb = min(ws) - 0.02, max(ws) + 0.02
    px = args.px
    W = int((ub - ua) / px) + 1; H = int((wb - wa) / px) + 1
    col = bytearray(b"\xff" * (W * H * 3))
    for i in sorted(sel, key=lambda i: kept[i], reverse=True):  # removed drawn last, on top
        x = int((uvw[i][0] - ua) / px); y = int((wb - uvw[i][2]) / px)
        k = 3 * (y * W + x)
        for dx in ((-1, 0, 1) if args.px > 0.01 else (0,)):  # bigger dots at coarse scales
            for dy in ((-1, 0, 1) if args.px > 0.01 else (0,)):
                if 0 <= x + dx < W and 0 <= y + dy < H:
                    k = 3 * ((y + dy) * W + x + dx)
                    col[k:k + 3] = b"\x20\x20\x20" if kept[i] else b"\xe0\x20\x20"
    nrem = sum(1 for i in sel if not kept[i])
    print(f"section v={vlev} u {ua}..{ub}: {len(sel):,} points, {nrem:,} removed; {W}x{H} px at {px * 1000:.0f} mm/px (up = out of face)")
    write_png(f"{args.prefix}_section{si}.png", W, H, col)
