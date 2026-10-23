"""
Point Cloud Tools - Clean Point Cloud 3D (EXPERIMENTAL: test on a copy of a project).

Removes noise from point cloud regions in 3D, so walls, ceilings and slopes are cleaned as well
as ground. Two tests:
- small floating pieces: clumps of points not connected to anything big (the gap is set from
  the measured point spacing) are dropped unless they lie on the surface;
- local plane: each point is compared with the best-fit plane through its neighbourhood (the
  idea behind CloudCompare's Noise filter); points further than the tolerance are dropped. Areas
  that aren't flat (edges, corners) are kept as they are. Sparse parts of a scan are judged with
  larger cells.
Optionally the kept points are smoothed: moved onto curved (quadratic) local surfaces fitted in
overlapping blocks, a moving least-squares smooth like PCL's, with larger blocks where the noise
is high.
Cell size and tolerance are picked from the measured point spacing and noise by default, and a
run that would remove more than 20% of the points is refused.
The result is written as a new scan in a new region (and optionally the removed points in a
second one) inside one transaction, committed only if the new scans read back as expected. The
selected regions and their scans are never changed.
"""
import math
import time

import clr
clr.AddReference("IronPython.Wpf")
import wpf

from System import Action, Int32
from System.Collections.Generic import List
from System.IO import StreamReader, Path
from System.Windows.Controls import StackPanel
from System.Windows.Threading import DispatcherPriority
from System.Windows.Media import Color as MediaColor
try:
    clr.AddReference("System.Drawing")
    from System.Drawing import Color as DrawingColor
except Exception:
    DrawingColor = None

from Trimble.Vce.Core import TransactMethodCall
from Trimble.Vce.Interfaces.Client import CommandGranularity, MandatoryContainers
from Trimble.Vce.Geometry import Point3D
clr.AddReference("Trimble.Vce.UI.Controls")
for name in ("Trimble.Sdk.Scanning", "Trimble.Sdk.Interfaces"):
    try:
        clr.AddReference(name)
    except Exception:
        pass  # already loaded by TBC under another name
from Trimble.Vce.Data.Scanning import PointCloudRegion, PointCloudDatabaseContainer, ExposedPointCloudRegionContainer
from Trimble.Vce.Interfaces.PointCloudIntegration import PointInfo, ColorMode

MAX_POINTS = 12000000   # refuse larger selections: the filter runs in pure Python on TBC's UI thread and
LARGE_POINTS = 3000000  # needs roughly 0.5 GB per million points; above LARGE_POINTS the report warns
MIN_NEIGHBOURS = 10     # points needed in the 3 x 3 x 3 cells to fit a plane
FLAT_RATIO = 0.1        # smallest / middle eigenvalue below this counts as a flat surface
MIN_PIECE = 30          # floating pieces with fewer points than this are removed as noise
THICK_K = 3.0           # ... unless within THICK_K x the thickness (RMS) of the surface around them
LEVELS = 3              # plane fits at cell, 2 x cell, 4 x cell; sparse areas use the coarser ones
# automatic settings, from tests on NavVis VLX3, Trimble SX12 and drone photogrammetry clouds (docs/testing.md):
AUTO_CELL = 3.0         # cell = 3 x the median point spacing (about 25-30 points per plane)
AUTO_TOL = 4.0          # tolerance = 4 x the measured scatter about the planes
MIN_CELL, MAX_CELL = 0.02, 0.5
MAX_REMOVED = 0.2       # refuse to write if more than this share would be removed
NEIGHBOURS = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
# smoothing ("move kept points onto the local surface"): curved surfaces in overlapping blocks
SMOOTH_BLOCK = 1.2      # block spacing = 1.2 x cell (about 3.5 x the point spacing)
SMOOTH_MIN_POINTS = 15  # points a block needs for a fit
SMOOTH_RIDGE = 0.01     # damping of the curvature terms
SMOOTH_MAX_MOVE = 4.0   # a block only moves points within this x its residual RMS
SMOOTH_NOISE_K = 8.0    # 2 x blocks where the residual RMS x this exceeds the block spacing...
SMOOTH_RELIEF = 1.25    # ...and the 2 x blocks' RMS is less than this x the small ones'


def Setup(cmdData, macroFileFolder):
    cmdData.Key = "PointCloudTools_CleanCloud"
    cmdData.CommandName = "PointCloudTools_CleanCloud"
    cmdData.Caption = "_Clean Point Cloud 3D"
    cmdData.UIForm = "CleanCloud"
    try:
        cmdData.DefaultTabKey = "Macros"
        cmdData.DefaultTabGroupKey = "Point Cloud Tools"
        cmdData.ShortCaption = "Clean Cloud 3D"
        cmdData.ToolTipTitle = "Clean Point Cloud 3D"
        cmdData.ToolTipTextFormatted = "Removes points that sit off the local surface (walls included) and writes the rest to a new scan. Experimental."
        cmdData.Version = 0.2
        cmdData.MacroAuthor = "tbc-macros"
    except Exception:
        pass


# ---------------------------------------------------------------- filter (pure Python)

def sym3_eig(c):
    """Eigenvalues (descending) and smallest eigenvector of a symmetric 3x3 (xx, yy, zz, xy, xz, yz).
    Same as BestFitScans.sym3_eig (macros are loaded separately, so it is copied)."""
    a, b, d, e, f, g = c
    p1 = e * e + f * f + g * g
    q = (a + b + d) / 3
    p2 = (a - q) ** 2 + (b - q) ** 2 + (d - q) ** 2 + 2 * p1
    p = math.sqrt(p2 / 6) if p2 > 0 else 0.0
    if p < 1e-15:
        return (q, q, q), (0.0, 0.0, 1.0)
    B = [[(a - q) / p, e / p, f / p], [e / p, (b - q) / p, g / p], [f / p, g / p, (d - q) / p]]
    detB = (B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1])
            - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0])
            + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0]))
    r = max(-1.0, min(1.0, detB / 2))
    phi = math.acos(r) / 3
    l1 = q + 2 * p * math.cos(phi)
    l3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    l2 = 3 * q - l1 - l3
    M = [[a - l3, e, f], [e, b - l3, g], [f, g, d - l3]]
    best, bn = None, -1
    for i, j in ((0, 1), (0, 2), (1, 2)):
        u, v = M[i], M[j]
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        n = cr[0] ** 2 + cr[1] ** 2 + cr[2] ** 2
        if n > bn:
            best, bn = cr, n
    s = math.sqrt(bn) or 1.0
    return (l1, l2, l3), (best[0] / s, best[1] / s, best[2] / s)


def cell_key(p, cell):
    return (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)), int(math.floor(p[2] / cell)))


def cell_sums(pts, cell, tick=None, use=None, index=None):
    """Point count and coordinate sums per cell, keyed by cell_key(p, cell). use: optional
    bytearray; points with use[i] == 0 are left out. index: optional list of the points to sum
    (instead of all). tick(fraction) gets progress from 0 to 1."""
    sums = {}
    n_pts = float(len(pts))
    for j in (index if index is not None else range(len(pts))):
        if tick and j % 10000 == 0:
            tick(j / n_pts)
        if use is not None and not use[j]:
            continue
        x, y, z = pts[j]
        k = (int(math.floor(x / cell)), int(math.floor(y / cell)), int(math.floor(z / cell)))
        s = sums.get(k)
        if s is None:
            s = sums[k] = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        s[0] += 1; s[1] += x; s[2] += y; s[3] += z
        s[4] += x * x; s[5] += y * y; s[6] += z * z; s[7] += x * y; s[8] += x * z; s[9] += y * z
    return sums


def fit_plane(sums, k, minus=None):
    """Plane (centre, normal, flat, thickness) through the points in the 3 x 3 x 3 cells around
    cell k, leaving out the points summed in minus, or None if there are too few. thickness is the
    RMS distance of those points from the plane."""
    t = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for a, b, d in NEIGHBOURS:
        q = (k[0] + a, k[1] + b, k[2] + d)
        s = sums.get(q)
        if s:
            for i in range(10):
                t[i] += s[i]
            if minus:
                s = minus.get(q)
                if s:
                    for i in range(10):
                        t[i] -= s[i]
    n = t[0]
    if n < MIN_NEIGHBOURS:
        return None
    mx, my, mz = t[1] / n, t[2] / n, t[3] / n
    cov = (t[4] / n - mx * mx, t[5] / n - my * my, t[6] / n - mz * mz,
           t[7] / n - mx * my, t[8] / n - mx * mz, t[9] / n - my * mz)
    (l1, l2, l3), nv = sym3_eig(cov)
    return ((mx, my, mz), nv, l2 > 0 and l3 <= FLAT_RATIO * l2, math.sqrt(max(0.0, l3)))


def planes_from_sums(sums, tick=None):
    planes = {}
    n_cells = float(len(sums))
    for j, k in enumerate(sums):
        if tick and j % 2000 == 0:
            tick(j / n_cells)
        pl = fit_plane(sums, k)
        if pl is not None:
            planes[k] = pl
    return planes


def local_planes(pts, cell, tick=None, use=None):
    """A plane (centre, normal, flat, thickness) per occupied cell, keyed by cell_key(p, cell),
    fitted to the points in the surrounding 3 x 3 x 3 cells. Uses per-cell sums so the cost is
    linear. use: optional bytearray; points with use[i] == 0 are left out of the fits.
    tick(fraction) gets progress from 0 to 1."""
    sums = cell_sums(pts, cell, (lambda f: tick(0.4 * f)) if tick else None, use)
    return planes_from_sums(sums, (lambda f: tick(0.4 + 0.6 * f)) if tick else None)


def off_plane(p, pl):
    """Signed distance from p to the plane pl."""
    c, nv = pl[0], pl[1]
    return nv[0] * (p[0] - c[0]) + nv[1] * (p[1] - c[1]) + nv[2] * (p[2] - c[2])


def scatter(pts, planes, cell, use=None, samples=50000):
    """Robust scatter of points about the flat planes (1.4826 x median distance) over an even
    sample of the points, or None if no sampled point is in a flat cell."""
    d = []
    for i in range(0, len(pts), max(1, len(pts) // samples)):
        if use is not None and not use[i]:
            continue
        pl = planes.get(cell_key(pts[i], cell))
        if pl is not None and pl[2]:
            d.append(abs(off_plane(pts[i], pl)))
    if not d:
        return None
    d.sort()
    return 1.4826 * d[len(d) // 2]


def point_spacing(pts, samples=3000):
    """(median, 95th percentile) distance from a point to its nearest neighbour, over an even
    sample of the points (only neighbours within 10 cm count). Static scanners are much sparser far
    away, so the 95th percentile is often 2-3 x the median. None if no neighbour is that close."""
    g = 0.05
    grid = {}
    for i, (x, y, z) in enumerate(pts):
        grid.setdefault((int(math.floor(x / g)), int(math.floor(y / g)), int(math.floor(z / g))), []).append(i)
    reach = [(a, b, c) for a in (-2, -1, 0, 1, 2) for b in (-2, -1, 0, 1, 2) for c in (-2, -1, 0, 1, 2)]
    nearest = []
    for i in range(0, len(pts), max(1, len(pts) // samples)):
        x, y, z = pts[i]
        k = (int(math.floor(x / g)), int(math.floor(y / g)), int(math.floor(z / g)))
        best = 0.01  # 10 cm squared
        for a, b, c in reach:
            for j in grid.get((k[0] + a, k[1] + b, k[2] + c), ()):
                if j != i:
                    q = pts[j]
                    d2 = (q[0] - x) ** 2 + (q[1] - y) ** 2 + (q[2] - z) ** 2
                    if d2 < best:
                        best = d2
        if best < 0.01:
            nearest.append(math.sqrt(best))
    if not nearest:
        return None
    nearest.sort()
    return nearest[len(nearest) // 2], nearest[int(0.95 * (len(nearest) - 1))]


def small_pieces(pts, gap, min_points, tick=None):
    """bytearray with 1 for points in connected pieces of fewer than min_points. Points in the same
    or touching cells of size gap are connected, so pieces separated by more than about 2 x gap
    (gaps up to ~3.5 x gap diagonally) are separate. tick(fraction) gets progress from 0 to 1."""
    keys = [(int(math.floor(x / gap)), int(math.floor(y / gap)), int(math.floor(z / gap))) for x, y, z in pts]
    if tick:
        tick(0.3)
    count = {}
    for k in keys:
        count[k] = count.get(k, 0) + 1
    parent = dict((k, k) for k in count)

    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    half = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) > (0, 0, 0)]
    n_cells = float(len(count))
    for j, k in enumerate(count):
        if tick and j % 20000 == 0:
            tick(0.3 + 0.6 * j / n_cells)
        for a, b, c in half:
            q = (k[0] + a, k[1] + b, k[2] + c)
            if q in parent:
                ra, rb = find(k), find(q)
                if ra != rb:
                    parent[ra] = rb
    size = {}
    for k, n in count.items():
        r = find(k)
        size[r] = size.get(r, 0) + n
    small = set(k for k in count if size[find(k)] < min_points)
    return bytearray(1 if k in small else 0 for k in keys)


def solve(A, b):
    """x with A x = b (Gaussian elimination, partial pivoting), or None if A is singular."""
    n = len(b)
    M = [A[i][:] + [b[i]] for i in range(n)]
    for k in range(n):
        p = max(range(k, n), key=lambda i: abs(M[i][k]))
        if abs(M[p][k]) < 1e-18:
            return None
        M[k], M[p] = M[p], M[k]
        for i in range(k + 1, n):
            f = M[i][k] / M[k][k]
            if f:
                for j in range(k, n + 1):
                    M[i][j] -= f * M[k][j]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][j] * x[j] for j in range(i + 1, n))) / M[i][i]
    return x


def corners(p, c):
    """The 8 block vertices around p (blocks are c apart and reach c around their vertex) with
    tent weights that sum to 1, so a point's blend of block surfaces has no seams."""
    fx, fy, fz = p[0] / c, p[1] / c, p[2] / c
    ix, iy, iz = int(math.floor(fx)), int(math.floor(fy)), int(math.floor(fz))
    rx, ry, rz = fx - ix, fy - iy, fz - iz
    out = []
    for dx, wx in ((0, 1 - rx), (1, rx)):
        for dy, wy in ((0, 1 - ry), (1, ry)):
            for dz, wz in ((0, 1 - rz), (1, rz)):
                out.append(((ix + dx, iy + dy, iz + dz), wx * wy * wz))
    return out


def block_surfaces(pts, c, index, tick=None):
    """Curved local surfaces: {vertex: (centre, a, b, normal, coef, sigma)} with the height along
    the normal h = coef . (1, u, v, u2, uv, v2), u and v along a and b in units of c, fitted
    (tent-weighted) to the points index lists. sigma is the RMS of the fit's residuals."""
    n_idx = float(len(index)) or 1.0
    sums = {}
    for j, i in enumerate(index):
        if tick and j % 10000 == 0:
            tick(0.4 * j / n_idx)
        x, y, z = pts[i]
        for k, w in corners(pts[i], c):
            s = sums.get(k)
            if s is None:
                s = sums[k] = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            s[0] += 1; s[1] += w; s[2] += w * x; s[3] += w * y; s[4] += w * z
            s[5] += w * x * x; s[6] += w * y * y; s[7] += w * z * z; s[8] += w * x * y; s[9] += w * x * z; s[10] += w * y * z
    frames = {}
    for k, s in sums.items():
        if s[0] < SMOOTH_MIN_POINTS:
            continue
        W = s[1]
        mx, my, mz = s[2] / W, s[3] / W, s[4] / W
        cov = (s[5] / W - mx * mx, s[6] / W - my * my, s[7] / W - mz * mz,
               s[8] / W - mx * my, s[9] / W - mx * mz, s[10] / W - my * mz)
        nv = sym3_eig(cov)[1]
        t = (1.0, 0.0, 0.0) if abs(nv[0]) < 0.9 else (0.0, 1.0, 0.0)
        a = (nv[1] * t[2] - nv[2] * t[1], nv[2] * t[0] - nv[0] * t[2], nv[0] * t[1] - nv[1] * t[0])
        an = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        a = (a[0] / an, a[1] / an, a[2] / an)
        b = (nv[1] * a[2] - nv[2] * a[1], nv[2] * a[0] - nv[0] * a[2], nv[0] * a[1] - nv[1] * a[0])
        frames[k] = ((mx, my, mz), a, b, nv, [0.0] * 28)
    for j, i in enumerate(index):
        if tick and j % 10000 == 0:
            tick(0.4 + 0.6 * j / n_idx)
        x, y, z = pts[i]
        for k, w in corners(pts[i], c):
            f = frames.get(k)
            if f is None:
                continue
            (cx, cy, cz), a, b, nv, acc = f
            px, py, pz = x - cx, y - cy, z - cz
            u = (px * a[0] + py * a[1] + pz * a[2]) / c
            v = (px * b[0] + py * b[1] + pz * b[2]) / c
            h = px * nv[0] + py * nv[1] + pz * nv[2]
            # weighted normal equations for (1, u, v, uu, uv, vv), upper triangle row by row, then
            # the right-hand side and h2 (written out: this is the slowest loop in the macro)
            wu, wv = w * u, w * v
            uu, uv, vv = u * u, u * v, v * v
            wuu, wuv, wvv = wu * u, wu * v, wv * v
            acc[0] += w; acc[1] += wu; acc[2] += wv; acc[3] += wuu; acc[4] += wuv; acc[5] += wvv
            acc[6] += wuu; acc[7] += wuv; acc[8] += wuu * u; acc[9] += wuu * v; acc[10] += wuv * v
            acc[11] += wvv; acc[12] += wuv * u; acc[13] += wuv * v; acc[14] += wvv * v
            acc[15] += wuu * uu; acc[16] += wuu * uv; acc[17] += wuu * vv
            acc[18] += wuv * uv; acc[19] += wuv * vv
            acc[20] += wvv * vv
            wh = w * h
            acc[21] += wh; acc[22] += wu * h; acc[23] += wv * h; acc[24] += wuu * h; acc[25] += wuv * h; acc[26] += wvv * h
            acc[27] += wh * h
    blocks = {}
    for k, (centre, a, b, nv, acc) in frames.items():
        A = [[0.0] * 6 for _ in range(6)]
        j2 = 0
        for p in range(6):
            for q in range(p, 6):
                A[p][q] = A[q][p] = acc[j2]
                j2 += 1
        for p in range(3, 6):  # damp the curvature terms so sparse or lopsided blocks stay near-flat
            A[p][p] += SMOOTH_RIDGE * acc[0]
        rhs = acc[21:27]
        coef = solve(A, rhs) or [acc[21] / acc[0], 0.0, 0.0, 0.0, 0.0, 0.0]
        ss = acc[27] - sum(coef[p] * rhs[p] for p in range(6))
        blocks[k] = (centre, a, b, nv, coef, math.sqrt(max(0.0, ss / acc[0])))
    return blocks


def on_surface(p, blocks, c):
    """(p moved onto the blend of the block surfaces around it, their blended sigma), or
    (p, None) if no block has a say. A block whose surface is more than SMOOTH_MAX_MOVE x its sigma
    away has none: the point is off it, or the fit doesn't reach there."""
    x, y, z = p
    sx = sy = sz = sw = ss = 0.0
    for k, w in corners(p, c):
        bl = blocks.get(k)
        if bl is None:
            continue
        (cx, cy, cz), a, b, nv, f, sigma = bl
        px, py, pz = x - cx, y - cy, z - cz
        u = (px * a[0] + py * a[1] + pz * a[2]) / c
        v = (px * b[0] + py * b[1] + pz * b[2]) / c
        d = f[0] + f[1] * u + f[2] * v + f[3] * u * u + f[4] * u * v + f[5] * v * v - (px * nv[0] + py * nv[1] + pz * nv[2])
        if abs(d) > SMOOTH_MAX_MOVE * max(sigma, 0.001):
            continue
        sx += w * (x + d * nv[0]); sy += w * (y + d * nv[1]); sz += w * (z + d * nv[2])
        ss += w * sigma; sw += w
    if sw < 1e-9:
        return p, None
    return (sx / sw, sy / sw, sz / sw), ss / sw


def smooth(pts, index, c, tick=None):
    """Moves the points index lists onto curved local surfaces (a moving least-squares smooth);
    returns ({index: new point}, points smoothed with 2 x c blocks). Blocks of c where the noise is
    low; 2 x c where it is high (sigma x SMOOTH_NOISE_K > c) and the larger blocks fit about as
    well (so relief at edges and corners isn't averaged away), or the small blocks are too sparse."""
    sub = (lambda a, b: (lambda f: tick(a + (b - a) * f))) if tick else (lambda a, b: None)
    fine = block_surfaces(pts, c, index, sub(0.0, 0.45))
    coarse = block_surfaces(pts, 2 * c, index[::2], sub(0.45, 0.65))  # 4 x the points per block: half do
    moved, n2 = {}, 0
    n_idx = float(len(index)) or 1.0
    for j, i in enumerate(index):
        if tick and j % 10000 == 0:
            tick(0.65 + 0.35 * j / n_idx)
        q, s1 = on_surface(pts[i], fine, c)
        if s1 is None or SMOOTH_NOISE_K * s1 > c:
            q2, s2 = on_surface(pts[i], coarse, 2 * c)
            if s2 is not None and (s1 is None or s2 < SMOOTH_RELIEF * s1):
                q = q2
                n2 += 1
        moved[i] = q
    return moved, n2


def clean(pts, cell, tol, flatten, keep_rough, drop_isolated, tick=None, gap=0.0, min_piece=0, levels=LEVELS):
    """Returns (out, counts, info): out = [(index, (x, y, z))] of the points to keep (moved onto
    curved local surfaces fitted to the kept points if flatten; see smooth), counts by outcome, and
    info = [(cell, tolerance, scatter, planes)] per level.
    Planes are fitted at cell, 2 x cell, 4 x cell (levels); each point is judged at the finest level
    with enough points around it, so sparse parts of a scan get larger cells. tol is the
    tolerance at the finest level (None: AUTO_TOL x the measured scatter); coarser levels scale it
    by how much the scatter grows, since a larger plane follows real relief less closely.
    gap, min_piece: pieces of fewer than min_piece points separated from everything else by more
    than about 2 x gap are left out of the finest plane fit, and dropped unless they lie on the
    surface around them (a sparse patch of real surface, rather than a floating clump): within the
    finest tolerance of a flat finest plane, or where there is none, within THICK_K x the
    thickness of a coarser one fitted without the small pieces.
    tick(fraction) is called now and then with progress from 0 to 1."""
    stages = levels + 0.5 + (1 if min_piece > 0 else 0) + (3 if flatten else 0)
    done = [0]

    def sub():
        k = done[0]
        done[0] += 1
        return (lambda f: tick((k + f) / stages)) if tick else None

    small = small_pieces(pts, gap, min_piece, sub()) if min_piece > 0 else None
    use = bytearray(0 if s else 1 for s in small) if small is not None else None
    small_index = [i for i in range(len(pts)) if small[i]] if small is not None else []
    fits = []
    surface = []  # per level: (cell, sums of all points, sums of small pieces, planes without them)
    for level in range(levels):
        c = cell * 2 ** level
        # small pieces are left out of the finest fit only: coarser planes need them where a sparse
        # area is made entirely of small pieces, or it has no plane to show it is real surface
        u = None if level else use
        tk = sub()
        sums = cell_sums(pts, c, (lambda f: tk(0.4 * f)) if tk else None, u)
        planes = planes_from_sums(sums, (lambda f: tk(0.4 + 0.6 * f)) if tk else None)
        fits.append((c, planes, scatter(pts, planes, c, u)))
        if small_index:
            # the surface without the small pieces, fitted only where one is asked for
            surface.append((c, sums, cell_sums(pts, c, index=small_index) if level else None, {} if level else planes))
    s0 = next((f[2] for f in fits if f[2]), None)
    if tol is None:
        if not s0:
            raise ValueError("no flat surfaces to measure the noise on; untick automatic settings and set them by hand")
        tol = round(AUTO_TOL * s0 / 0.0005) * 0.0005
    tols = [max(tol, tol * f[2] / s0) if f[2] and s0 else tol for f in fits]
    counts = {"small piece": 0, "small piece on surface": 0, "flat kept": 0, "off surface": 0,
              "rough kept": 0, "rough dropped": 0, "isolated kept": 0, "isolated dropped": 0,
              "smoothed": 0, "smoothed with larger blocks": 0}
    for level in range(1, levels):
        counts["judged at level %d" % level] = 0
    out = []
    n_pts = float(len(pts))
    base = done[0]
    for i, p in enumerate(pts):
        if tick and i % 10000 == 0:
            tick((base + 0.5 * i / n_pts) / stages)
        pl, level = None, 0
        for level, (c, planes, _) in enumerate(fits):
            pl = planes.get(cell_key(p, c))
            if pl is not None:
                break
        if pl is not None and level:
            counts["judged at level %d" % level] += 1
        if small is not None and small[i]:
            # kept if on the surface around it, fitted without the small pieces so a clump can't
            # pull it. Close to dense surface (a finest plane): flat and within the tolerance. Where
            # the scan is sparse, a thick band of real surface breaks into small pieces and dropping
            # them opens holes, so there: within THICK_K x the coarser surface's thickness
            sp, slev = None, 0
            for slev, (c, sums, minus, cache) in enumerate(surface):
                k = cell_key(p, c)
                if k in cache:
                    sp = cache[k]
                elif minus is not None:
                    sp = cache[k] = fit_plane(sums, k, minus)
                if sp is not None:
                    break
            if sp is None:  # nothing but small pieces nearby: only a flat patch within the tolerance counts
                sp, slev = pl, 0
            on = sp is not None and abs(off_plane(p, sp)) <= (max(tol, THICK_K * sp[3]) if slev else tol) \
                and (slev > 0 or sp[2])
            if not on:
                counts["small piece"] += 1
                continue
            counts["small piece on surface"] += 1
        if pl is None:
            if drop_isolated:
                counts["isolated dropped"] += 1
            else:
                counts["isolated kept"] += 1
                out.append((i, p))
            continue
        if not pl[2]:
            if keep_rough:
                counts["rough kept"] += 1
                out.append((i, p))
            else:
                counts["rough dropped"] += 1
            continue
        d = off_plane(p, pl)
        if abs(d) > tols[level]:
            counts["off surface"] += 1
            continue
        counts["flat kept"] += 1
        out.append((i, p))
    if flatten and out:
        moved, n2 = smooth(pts, [i for i, _ in out], SMOOTH_BLOCK * cell, sub_range(tick, (base + 0.5) / stages, 1.0))
        out = [(i, moved[i]) for i, _ in out]
        counts["smoothed"] = sum(1 for i, p in out if p is not pts[i])
        counts["smoothed with larger blocks"] = n2
    return out, counts, [(f[0], tols[k], f[2], len(f[1])) for k, f in enumerate(fits)]


def sub_range(tick, a, b):
    return (lambda f: tick(a + (b - a) * f)) if tick else None


# ---------------------------------------------------------------- TBC side

def region_name(region):
    return str(getattr(region, "Name", None) or "(unnamed region)")


def centroid(pts):
    n = float(len(pts))
    return tuple(sum(p[i] for p in pts) / n for i in range(3))


class Cancelled(Exception):
    pass


class CleanCloud(StackPanel):  # TBC hosts this panel because cmdData.UIForm names this class
    def __init__(self, currentProject, macroFileFolder):
        with StreamReader(Path.Combine(macroFileFolder, "CleanCloud.xaml")) as s:
            wpf.LoadComponent(self, s)
        self.currentProject = currentProject
        self.result = None
        self.busy = False
        self.cancel_requested = False
        self.last_pump = 0.0

    def OnLoad(self, cmd, buttons, event):
        self.Caption = cmd.Command.Caption
        self.closeBtn = buttons[1]  # TBC's Cancel button; it only cancels while a run is going
        self.closeBtn.Content = "Close"
        self.regionType = clr.GetClrType(PointCloudRegion)
        self.regionPicker.IsEntityValidCallback = self.IsValid
        self.cellSize.Distance = 0.05  # used when automatic settings are off; NavVis wall scans suit 50 / 20 mm
        self.tolerance.Distance = 0.02
        self.AutoChanged(None, None)
        self.import_range = (0.0, 100.0)
        self.progressStatus.Content = "Ready"

    def IsValid(self, serial):
        return isinstance(self.currentProject.Concordance.Lookup(serial), self.regionType)

    def CancelClicked(self, cmd, args):
        if self.busy:
            self.cancel_requested = True  # picked up at the next progress update
            self.progressStatus.Content = "Cancelling..."
            return
        cmd.CloseUICommand()

    def log(self, text):
        self.resultText.Text = (self.resultText.Text + "\n" + text).lstrip("\n")

    def progress(self, value, status=None, can_cancel=True):
        self.progressBar.Value = max(0.0, min(100.0, value))
        if status is not None:
            self.progressStatus.Content = status
        now = time.time()
        if now - self.last_pump >= 0.1:
            self.last_pump = now
            # the work runs on TBC's UI thread: let WPF repaint and handle clicks (Cancel) now and then
            self.Dispatcher.Invoke(Action(lambda: None), DispatcherPriority.Background)
        if can_cancel and self.cancel_requested:
            raise Cancelled()

    def import_progress(self, pct):
        if self.Dispatcher.CheckAccess():  # TBC may report from another thread; only update from ours
            lo, hi = self.import_range
            self.progress(lo + (hi - lo) * pct / 100.0, f"Writing points ({pct}%)", can_cancel=False)

    def begin_work(self):
        self.busy = True
        self.cancel_requested = False
        self.previewBtn.IsEnabled = False
        self.closeBtn.Content = "Cancel"

    def end_work(self):
        self.busy = False
        self.cancel_requested = False
        self.previewBtn.IsEnabled = True
        self.closeBtn.Content = "Close"

    def stored_coordinates(self):
        """Context in which points are read and written as TBC stores them, without the project's
        scale factor. Points read with the scale factor and written back get scaled twice."""
        return PointCloudDatabaseContainer.GetInstance(self.currentProject).TemporarilySetGroundScaleFactorToOne()

    def AutoChanged(self, sender, e):
        manual = not self.autoCB.IsChecked
        self.cellSize.IsEnabled = manual
        self.tolerance.IsEnabled = manual

    def settings(self, regions):
        """What the filter result depends on; cell and tolerance are None when picked automatically."""
        auto = bool(self.autoCB.IsChecked)
        return (tuple(sorted(int(getattr(r, "SerialNumber", id(r))) for r in regions)),
                None if auto else float(self.cellSize.Distance), None if auto else float(self.tolerance.Distance),
                bool(self.flattenCB.IsChecked), bool(self.keepRoughCB.IsChecked), bool(self.dropIsolatedCB.IsChecked),
                bool(self.removeSmallCB.IsChecked), auto)

    def run_filter(self):
        """Reads the selected regions and filters them; stores and returns the result, or None."""
        self.result = None
        regions = [o for o in self.regionPicker.SelectedMembers(self.currentProject) if isinstance(o, self.regionType)]
        if not regions:
            self.log("Select one or more point cloud regions first.")
            return None
        settings = self.settings(regions)
        cell, tol, auto = settings[1], settings[2], settings[7]
        if not auto and (cell <= 0 or tol <= 0):
            self.log("Cell size and tolerance must be greater than zero.")
            return None
        total = sum(int(r.Integration.PointCount) for r in regions)
        self.log("Regions: " + "; ".join("%s (%s points)" % (region_name(r), int(r.Integration.PointCount)) for r in regions))
        if total > MAX_POINTS:
            self.log(f"{total:,} points is more than this macro handles ({MAX_POINTS:,}); use smaller regions.")
            return None
        # about 20-30 s per 1.7 M points in TBC in testing; smoothing adds about 75 s per million
        minutes = (total / 1.7e6 * 25 + (total / 1e6 * 75 if settings[3] else 0)) / 60
        if total > LARGE_POINTS or minutes > 2:
            self.log(f"Large job: expect about {max(1, round(minutes))} min and "
                     f"{total / 2e6:.1f} GB of memory. Cancel stops it.")
        noop = Action[Int32](lambda pct: None)
        self.progress(0, "Reading points")
        infos, pts, origin = [], [], None
        with self.stored_coordinates():
            for r in regions:
                for pi in r.Integration.GetPointInfos(noop):
                    if len(pts) % 20000 == 0:
                        self.progress(40.0 * len(pts) / total, f"Reading points {len(pts):,} / {total:,}")
                    c = pi.Coordinates
                    if origin is None:
                        origin = (c.X, c.Y, c.Z)  # work relative to one point so the sums keep their precision
                    infos.append(pi)
                    pts.append((c.X - origin[0], c.Y - origin[1], c.Z - origin[2]))
        if not pts:
            self.log("The selected regions have no points.")
            return None
        self.progress(40, "Measuring point spacing")
        spacing = point_spacing(pts)
        if spacing:
            s50, s95 = spacing
            self.log(f"Point spacing: typically {s50 * 1000:.1f} mm, {s95 * 1000:.1f} mm in the sparsest 5%")
        elif auto:
            self.log("Points are too sparse to measure their spacing; untick automatic settings and set them by hand.")
            return None
        if auto:
            cell = min(MAX_CELL, max(MIN_CELL, round(AUTO_CELL * s50 / 0.005) * 0.005))
            self.cellSize.Distance = cell
        gap, min_piece = 0.0, 0
        if settings[6]:
            if spacing:
                # pieces are separate when more than about 2 x gap apart. At 0.9 x the typical
                # spacing real rock broke into pieces; static scans are much sparser far away, so the
                # sparse end of the spacing sets the gap there
                gap, min_piece = max(1.2 * s50, s95), MIN_PIECE
                self.log(f"Floating pieces: under {MIN_PIECE} points, more than about {2 * gap * 1000:.0f} mm from "
                         f"anything else, and off the surface")
            else:
                self.log("Points are too sparse to measure their spacing; not removing small pieces")
        steps = [(1.0, "Finding floating pieces")] if min_piece else []
        steps += [(1.0, "Fitting planes, %d mm cells" % round(cell * 2 ** k * 1000)) for k in range(LEVELS)]
        steps += [(0.5, "Sorting points")]
        if settings[3]:
            steps += [(3.0, "Smoothing onto %d mm curved surfaces" % round(SMOOTH_BLOCK * cell * 1000))]
        total_w = sum(w for w, _ in steps)

        def tick(f):
            acc = 0.0
            for w, label in steps:
                acc += w / total_w
                if f <= acc:
                    break
            self.progress(42 + 58 * f, label)
        out, counts, info = clean(pts, cell, tol, settings[3], settings[4], settings[5], tick, gap, min_piece)
        self.progress(100, "Filter done")
        if auto:
            self.tolerance.Distance = info[0][1]
        n = len(pts)
        scatter0 = next((f[2] for f in info if f[2]), 0.0)
        self.log(f"Cell {cell * 1000:.0f} mm, tolerance {info[0][1] * 1000:.1f} mm"
                 + (f" ({AUTO_CELL:g} x spacing, {AUTO_TOL:g} x the {scatter0 * 1000:.1f} mm scatter)" if auto else ""))
        self.log("  sparse areas use larger cells: " + "; ".join(
            f"{c * 1000:.0f} mm, tolerance {t * 1000:.1f} mm" for c, t, _, _ in info[1:]))
        labels = ["small piece", "small piece on surface", "flat kept", "off surface", "rough kept",
                  "rough dropped", "isolated kept", "isolated dropped", "smoothed", "smoothed with larger blocks"]
        labels += ["judged at level %d" % k for k in range(1, LEVELS)]
        for label in labels:  # IronPython 3.4 dicts are unordered
            value = counts[label]
            if value:
                if label.startswith("judged at level"):
                    label = "judged with %d mm cells" % round(info[int(label[-1])][0] * 1000)
                self.log(f"  {label}: {value:,} ({100.0 * value / n:.1f}%)")
        removed = n - len(out)
        self.log(f"Keeps {len(out):,} of {n:,} points ({100.0 * len(out) / n:.1f}%).")
        if removed > MAX_REMOVED * n:
            self.log(f"STOPPED: this would remove {100.0 * removed / n:.0f}% of the points, more than the "
                     f"{100 * MAX_REMOVED:.0f}% safety limit, so OK won't write it. The settings probably don't suit "
                     "this cloud: try automatic settings, or a larger cell / tolerance.")
            return None
        if counts["rough kept"] > 0.3 * n:
            if auto:  # the automatic cell is already as small as the point spacing allows
                self.log("Many points are in areas that aren't flat at this cell size (rough rock or a thick "
                         "cloud); they are kept as they are.")
            else:
                self.log("Many points are in areas that aren't flat at this cell size: try a smaller cell.")
        self.result = (settings, regions, infos, origin, out)
        return self.result

    def PreviewClicked(self, sender, e):
        if self.busy:
            return
        self.resultText.Text = ""
        self.begin_work()
        try:
            if self.run_filter():
                self.progressStatus.Content = "Preview ready"
                self.log("Press OK to write the kept points to a new region and scan.")
        except Cancelled:
            self.result = None
            self.progressStatus.Content = "Cancelled"
            self.log("Cancelled.")
        except Exception as ex:
            self.log(f"Preview failed: {ex}")
        finally:
            self.end_work()

    def OkClicked(self, cmd, args):
        if self.busy:
            return False
        self.begin_work()
        try:
            self.write()
        except Cancelled:
            self.result = None
            self.progressStatus.Content = "Cancelled"
            self.log("Cancelled; nothing was written.")
        except Exception as ex:
            self.log(f"Failed: {ex}")
        finally:
            self.end_work()
        return False  # keep the panel open so the report stays visible

    def write(self):
        regions = [o for o in self.regionPicker.SelectedMembers(self.currentProject) if isinstance(o, self.regionType)]
        if self.result is None or not regions or self.result[0] != self.settings(regions):
            self.resultText.Text = ""
            if not self.run_filter():
                return
        settings, regions, infos, origin, out = self.result
        if not out:
            self.log("No points would be kept; nothing written.")
            return
        ox, oy, oz = origin
        points = List[PointInfo]()
        expected = []
        keep = bytearray(len(infos))
        n_out = float(len(out))
        for j, (i, p) in enumerate(out):
            if j % 50000 == 0:
                self.progress(12.0 * j / n_out, f"Preparing points {j:,} / {len(out):,}")
            keep[i] = 1
            pi = infos[i]
            if settings[3]:
                pi = PointInfo(pi.Color, Point3D(p[0] + ox, p[1] + oy, p[2] + oz), pi.Luminance, pi.Normal, pi.ScanId)
            points.Add(pi)
            expected.append(p)
        removed, removed_expected = List[PointInfo](), []
        if self.writeRemovedCB.IsChecked:
            for i, pi in enumerate(infos):
                if not keep[i]:
                    c = pi.Coordinates
                    removed.Add(pi)
                    removed_expected.append((c.X - ox, c.Y - oy, c.Z - oz))
        split = 75.0 if removed_expected else 100.0
        base = region_name(regions[0])
        committed = False
        manager = self.currentProject.TransactionManager
        manager.AddBeginMark(CommandGranularity.Command, "Clean Point Cloud 3D")
        try:
            # changes made inside the "with" block are rolled back unless Commit() is called,
            # including when Cancel raises Cancelled out of a progress update
            with TransactMethodCall(self.currentProject.TransactionCollector) as failGuard:
                ok, cleaned_region = self.write_scan(base + " - cleaned", points, expected, origin, 15.0, split)
                removed_region = None
                if removed_expected and (ok or not self.verifyCB.IsChecked):
                    ok2, removed_region = self.write_scan(base + " - removed", removed, removed_expected, origin,
                                                          split, 100.0)
                    ok = ok and ok2
                if self.verifyCB.IsChecked and not ok:
                    self.log("A new scan doesn't match the points written; rolled back. If a new region is still "
                             "listed, delete it, and send this report.")
                    return
                if self.showResultCB.IsChecked:
                    self.show_result(regions, cleaned_region, removed_region)
                failGuard.Commit()
                committed = True
        finally:
            manager.AddEndMark(CommandGranularity.Command)
            self.progress(100, "Done" if committed else "Not written", can_cancel=False)
        if committed:
            self.log("Written. Hide the source regions to compare; use Undo to remove the new regions.")
            self.result = None

    def show_result(self, sources, cleaned, removed):
        """Inside the caller's transaction: hide the source regions in the view filters, show the
        cleaned region in true colour and the removed region in red. Each step is cosmetic, so a
        failure is reported and the rest carries on."""
        project = self.currentProject
        try:
            cleaned.ColorMode = ColorMode.PointRGB
        except Exception as ex:
            self.log(f"  could not set true colour on the cleaned region: {ex}")
        if removed is not None:
            # the region's Color type isn't documented: try the two colour types TBC uses
            reds = [MediaColor.FromRgb(220, 30, 30)]
            if DrawingColor is not None:
                reds.insert(0, DrawingColor.FromArgb(255, 220, 30, 30))
            errors = []
            for red in reds:
                try:
                    removed.Color = red
                    break
                except Exception as ex:
                    errors.append(str(ex))
            else:
                self.log("  could not colour the removed region red: " + "; ".join(errors))
            try:
                removed.ColorMode = ColorMode.Model  # "region colour" in TBC's colour mode list
            except Exception as ex:
                self.log(f"  could not set region colour on the removed region: {ex}")
        try:
            types = project.GetMemberContainer(MandatoryContainers.ImitationTypeCollection)
            serial = lambda region: types.Lookup(region.ImitationTypeGuid).SerialNumber
            hide = [serial(r) for r in sources]
            show = [serial(r) for r in (cleaned, removed) if r is not None]
            filters = self.view_filters()
            changed = []
            for f in filters:
                if f.IsReadOnly:
                    continue
                overrides = f.ImitationTypeOverrides
                for s, visible in [(s, False) for s in hide] + [(s, True) for s in show]:
                    rendering = overrides[s] if overrides.Contains(s) else overrides.Add(s)
                    rendering.Visible = visible
                try:
                    f.ResetFilterCache()
                except Exception:
                    pass  # only the concrete ViewFilter has it; TBC refreshes on commit anyway
                changed.append(str(f.Name))
            self.log("  view filters updated (source regions hidden): " + (", ".join(changed) or "none writable"))
        except Exception as ex:
            self.log(f"  could not update the view filters: {ex}")

    def view_filters(self):
        collection = self.currentProject.GetMemberContainer(MandatoryContainers.ViewFilterCollection)
        filters = []
        try:
            for item in collection:
                f = item
                if not hasattr(f, "ImitationTypeOverrides"):  # the collection may yield serial numbers
                    try:
                        f = self.currentProject.Concordance.Lookup(item)
                    except Exception:
                        continue
                if hasattr(f, "ImitationTypeOverrides"):
                    filters.append(f)
        except Exception:
            pass  # not enumerable: fall back to the default filter
        return filters or [collection.DefaultViewFilter]

    def write_scan(self, name, points, expected, origin, lo, hi):
        """Writes points as a new scan in a new region called name, inside the caller's transaction,
        and checks that it reads back with the expected count and position. Returns (ok, region)."""
        project = self.currentProject
        database = PointCloudDatabaseContainer.GetInstance(project).PrimaryDatabase
        region_container = ExposedPointCloudRegionContainer.GetInstance(project)
        noop = Action[Int32](lambda pct: None)
        ox, oy, oz = origin
        # compare averages of all points: TBC returns points in its own order, so averages of
        # every n-th point differ by several mm even when nothing moved
        expected_centre = centroid(expected)
        self.progress(lo, "Creating region and station")
        region = region_container.Add()
        region.Name = region_container.MakeUniqueRegionName(name)
        station = database.Add()
        try:
            station.Name = database.MakeUniqueStationName(name)
        except Exception:
            pass  # the station keeps TBC's default name
        check_from = lo + 0.7 * (hi - lo)
        self.import_range = (lo, check_from)
        with self.stored_coordinates():
            scan = database.Integration.ImportPoints(points, station, region, False, Action[Int32](self.import_progress))
            if scan is None:
                self.log(f"TBC did not create a scan for '{name}'.")
                return False, region
            written = int(region.Integration.PointCount)
            sx = sy = sz = 0.0
            i = 0
            for q in region.Integration.GetPoints(noop):
                if i % 50000 == 0:
                    self.progress(check_from + (hi - check_from) * i / max(1, written),
                                  f"Checking '{region.Name}' {i:,} / {written:,}")
                sx += q.X - ox
                sy += q.Y - oy
                sz += q.Z - oz
                i += 1
        self.log(f"New region '{region.Name}': {written:,} points (expected {len(expected):,})")
        if not i:
            return False, region
        centre = (sx / i, sy / i, sz / i)
        off = [centre[k] - expected_centre[k] for k in range(3)]
        self.log(f"  centre offset dE {off[0] * 1000:+.1f}  dN {off[1] * 1000:+.1f}  dZ {off[2] * 1000:+.1f} mm")
        ok = abs(written - len(expected)) <= 0.01 * len(expected) and max(abs(v) for v in off) <= 0.005
        if not ok:
            # an offset proportional to the coordinates means a scale factor was applied
            absolute = [expected_centre[k] + origin[k] for k in range(3)]
            self.log("  implied scale per axis: " + "  ".join(
                "%.8f" % ((absolute[k] + off[k]) / absolute[k]) if absolute[k] else "-" for k in range(3)))
        return ok, region
