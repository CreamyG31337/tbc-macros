"""Per-point shape features for separating hoses / anchors from rock, in plain Python so the same
code can run in TBC's IronPython. Uses the macro's helpers (sym3_eig, cell_key, NEIGHBOURS...).

features(pts, ns) -> (names, rows): one list of floats per point; -1 where a neighbourhood has too
few points."""
import math

SCALES = (0.025, 0.05, 0.10, 0.20)   # cell sizes; neighbourhoods are 3 x 3 x 3 cells
SURFACE_SCALES = ((0.10, 0.04), (0.20, 0.06))  # (cell, trim) for the trimmed rock surface


def eigvec(c, lam):
    """Unit eigenvector of the symmetric 3x3 c = (xx, yy, zz, xy, xz, yz) for eigenvalue lam."""
    a, b, d, e, f, g = c
    M = [[a - lam, e, f], [e, b - lam, g], [f, g, d - lam]]
    best, bn = (0.0, 0.0, 1.0), -1.0
    for i, j in ((0, 1), (0, 2), (1, 2)):
        u, v = M[i], M[j]
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        n = cr[0] ** 2 + cr[1] ** 2 + cr[2] ** 2
        if n > bn:
            best, bn = cr, n
    s = math.sqrt(bn) or 1.0
    return (best[0] / s, best[1] / s, best[2] / s)


def cell_shapes(pts, cell, ns):
    """Per occupied cell: (centre, normal, lin, pla, sph, |normal z|, |main axis z|, log n)."""
    sums = {}
    for x, y, z in pts:
        k = (int(math.floor(x / cell)), int(math.floor(y / cell)), int(math.floor(z / cell)))
        s = sums.get(k)
        if s is None:
            s = sums[k] = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        s[0] += 1; s[1] += x; s[2] += y; s[3] += z
        s[4] += x * x; s[5] += y * y; s[6] += z * z; s[7] += x * y; s[8] += x * z; s[9] += y * z
    out = {}
    for k in sums:
        t = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for a, b, d in ns["NEIGHBOURS"]:
            s = sums.get((k[0] + a, k[1] + b, k[2] + d))
            if s:
                for i in range(10):
                    t[i] += s[i]
        n = t[0]
        if n < ns["MIN_NEIGHBOURS"]:
            continue
        mx, my, mz = t[1] / n, t[2] / n, t[3] / n
        cov = (t[4] / n - mx * mx, t[5] / n - my * my, t[6] / n - mz * mz,
               t[7] / n - mx * my, t[8] / n - mx * mz, t[9] / n - my * mz)
        (l1, l2, l3), nv = ns["sym3_eig"](cov)
        if l1 <= 0:
            continue
        l3 = max(l3, 0.0)
        e1 = eigvec(cov, l1)
        out[k] = ((mx, my, mz), nv, (l1 - l2) / l1, (l2 - l3) / l1, l3 / l1, abs(nv[2]), abs(e1[2]), math.log(n))
    return out


def trimmed_surface(pts, cell, trim, ns):
    """Planes fitted at cell, refitted twice leaving out points further than trim."""
    planes = ns["local_planes"](pts, cell)
    for _ in range(2):
        use = bytearray(len(pts))
        for i, p in enumerate(pts):
            pl = planes.get(ns["cell_key"](p, cell))
            use[i] = 1 if pl is not None and abs(ns["off_plane"](p, pl)) <= trim else 0
        planes = ns["local_planes"](pts, cell, None, use)
    return planes


def features(pts, ns, log=print):
    names = []
    for sc in SCALES:
        mm = int(round(sc * 1000))
        names += [f"lin{mm}", f"pla{mm}", f"sph{mm}", f"nz{mm}", f"az{mm}", f"logn{mm}", f"off{mm}"]
    for sc, _ in SURFACE_SCALES:
        names.append(f"proud{int(round(sc * 1000))}")
    rows = [[] for _ in pts]
    for sc in SCALES:
        shapes = cell_shapes(pts, sc, ns)
        log(f"  scale {sc * 1000:.0f} mm: {len(shapes):,} cells with shapes")
        for i, p in enumerate(pts):
            s = shapes.get(ns["cell_key"](p, sc))
            if s is None:
                rows[i] += [-1.0] * 7
            else:
                c, nv = s[0], s[1]
                off = abs(nv[0] * (p[0] - c[0]) + nv[1] * (p[1] - c[1]) + nv[2] * (p[2] - c[2]))
                rows[i] += [s[2], s[3], s[4], s[5], s[6], s[7], off]
    for sc, trim in SURFACE_SCALES:
        planes = trimmed_surface(pts, sc, trim, ns)
        log(f"  rock surface {sc * 1000:.0f} mm: {len(planes):,} planes")
        for i, p in enumerate(pts):
            pl = planes.get(ns["cell_key"](p, sc))
            rows[i].append(abs(ns["off_plane"](p, pl)) if pl is not None else -1.0)
    return names, rows
