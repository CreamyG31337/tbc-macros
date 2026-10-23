"""
Point Cloud Tools - Best-Fit Scans (EXPERIMENTAL: test on a copy of a project).

Fits "moving" point cloud regions onto "reference" regions with a point-to-plane,
translation-only fit on the surfaces both clouds contain, then shifts the scans behind
the moving regions by the result. The shift runs inside one transaction: the points are
re-read and the change is committed only if they moved by the computed amount; otherwise
it is rolled back and the project is unchanged. Undo also reverses a committed shift.
A typed vertical shift (up or down) can be applied instead of the fit, the same way.

The fit is the same method as bestfit_clouds.py in the check-georef repository.
"""
import math
import random
import time

import clr
clr.AddReference("IronPython.Wpf")
import wpf

from System import Action, Int32
from System.IO import StreamReader, Path
from System.Windows.Controls import StackPanel
from System.Windows.Threading import DispatcherPriority

from Trimble.Vce.Core import TransactMethodCall
from Trimble.Vce.Interfaces.Client import CommandGranularity
from Trimble.Vce.Geometry import Matrix4D, Spinor3D, Vector3D
clr.AddReference("Trimble.Vce.UI.Controls")
try:
    clr.AddReference("Trimble.Sdk.Scanning")
except Exception:
    pass  # already loaded by TBC under another name
from Trimble.Vce.Data.Scanning import PointCloudRegion

REFERENCE_MAX_POINTS = 1000000  # cap on points read from the reference regions
MOVING_MAX_POINTS = 200000      # cap on points read from the moving regions
FIT_SAMPLES = 40000             # moving points used in the fit
SHARED_VOXEL = 0.20             # m; surfaces seen by both clouds
VERIFY_POINTS = 200000          # points re-read to confirm the shift


def Setup(cmdData, macroFileFolder):
    cmdData.Key = "PointCloudTools_BestFitScans"
    cmdData.CommandName = "PointCloudTools_BestFitScans"
    cmdData.Caption = "_Best-Fit Scans"
    cmdData.UIForm = "BestFitScans"
    try:
        cmdData.DefaultTabKey = "Macros"
        cmdData.DefaultTabGroupKey = "Point Cloud Tools"
        cmdData.ShortCaption = "Best-Fit Scans"
        cmdData.ToolTipTitle = "Best-Fit Scans"
        cmdData.ToolTipTextFormatted = "Computes the offset between overlapping point cloud regions and shifts the whole scans behind the moving regions. Experimental."
        cmdData.Version = 0.2
        cmdData.MacroAuthor = "tbc-macros"
    except Exception:
        pass


# ---------------------------------------------------------------- fit (pure Python)

def voxel(p, size):
    return (math.floor(p[0] / size), math.floor(p[1] / size), math.floor(p[2] / size))


def shared_zone(ref, mov):
    nb = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
    rv = set(voxel(p, SHARED_VOXEL) for p in ref)
    counts = {}
    for p in mov:
        key = voxel(p, SHARED_VOXEL)
        counts[key] = counts.get(key, 0) + 1
    sm = set(k for k, c in counts.items() if c >= 5 and any((k[0] + a, k[1] + b, k[2] + d) in rv for a, b, d in nb))
    sr = set((k[0] + a, k[1] + b, k[2] + d) for k in sm for a, b, d in nb)
    return [p for p in ref if voxel(p, SHARED_VOXEL) in sr], [p for p in mov if voxel(p, SHARED_VOXEL) in sm]


def sym3_eig(c):
    """Eigenvalues (descending) and smallest eigenvector of a symmetric 3x3 (xx, yy, zz, xy, xz, yz)."""
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


def fit_planes(ref, cell):
    cells = {}
    for p in ref:
        cells.setdefault(voxel(p, cell), []).append(p)
    planes = {}
    for k, cp in cells.items():
        if len(cp) < 10:
            continue
        n = len(cp)
        mx = sum(p[0] for p in cp) / n
        my = sum(p[1] for p in cp) / n
        mz = sum(p[2] for p in cp) / n
        xx = yy = zz = xy = xz = yz = 0.0
        for x, y, z in cp:
            dx, dy, dz = x - mx, y - my, z - mz
            xx += dx * dx; yy += dy * dy; zz += dz * dz; xy += dx * dy; xz += dx * dz; yz += dy * dz
        (l1, l2, l3), nv = sym3_eig((xx / n, yy / n, zz / n, xy / n, xz / n, yz / n))
        if l1 <= 0 or l3 / (l1 + l2 + l3) > 0.01 or l2 / l1 < 0.1 or math.sqrt(max(l3, 0)) > 0.006:
            continue  # not a clean plane
        planes[k] = ((mx, my, mz), nv)
    return planes


def correspond(src, planes, t, maxres, cell, search):
    reach = max(1, int(math.ceil(search / cell)))
    out = []
    for p in src:
        q = (p[0] + t[0], p[1] + t[1], p[2] + t[2])
        ci, cj, ck = voxel(q, cell)
        best, bd = None, search * search
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                for dk in range(-reach, reach + 1):
                    pl = planes.get((ci + di, cj + dj, ck + dk))
                    if pl:
                        c = pl[0]
                        d = (q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2 + (q[2] - c[2]) ** 2
                        if d < bd:
                            best, bd = pl, d
        if best:
            c, n = best
            r = n[0] * (q[0] - c[0]) + n[1] * (q[1] - c[1]) + n[2] * (q[2] - c[2])
            if abs(r) < maxres and bd - r * r < (cell * 0.75) ** 2:
                out.append((n, r))
    return out


def solve3(A, b):
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[piv] = M[piv], M[i]
        if abs(M[i][i]) < 1e-12:
            raise ValueError("the overlap does not constrain all three axes")
        for r in range(3):
            if r != i:
                f = M[r][i] / M[i][i]
                for c in range(i, 4):
                    M[r][c] -= f * M[i][c]
    return [M[i][3] / M[i][i] for i in range(3)]


def median(v):
    s = sorted(v)
    m = len(s) // 2
    return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2


def icp(src, planes, cell, search, start, tick=None):
    t = list(start)
    maxres = search
    for it in range(30):
        if tick:
            tick(it / 30.0)
        cor = correspond(src, planes, t, maxres, cell, search)
        if len(cor) < 50:
            raise ValueError(f"only {len(cor)} matching points; is there enough overlap?")
        res = [r for _, r in cor]
        med = median(res)
        mad = median([abs(r - med) for r in res]) * 1.4826
        if it >= 2:
            maxres = max(0.01, 3 * mad)
        AtA = [[0.0] * 3 for _ in range(3)]
        Atb = [0.0] * 3
        for n, r in cor:
            if abs(r) > maxres:
                continue
            for i in range(3):
                Atb[i] -= n[i] * r
                for j in range(3):
                    AtA[i][j] += n[i] * n[j]
        dt = solve3(AtA, Atb)
        t = [t[i] + dt[i] for i in range(3)]
        if math.sqrt(sum(d * d for d in dt)) < 5e-5 and it >= 3:
            break
    inl = [(n, r) for n, r in correspond(src, planes, t, maxres, cell, search) if abs(r) <= maxres]
    rms = math.sqrt(sum(r * r for _, r in inl) / len(inl))
    diag = [sum(n[i] * n[i] for n, _ in inl) for i in range(3)]
    return t, len(inl), rms, diag


def best_fit(ref, mov, search, report, progress):
    progress(20, "Finding shared surfaces")
    report(f"read {len(ref):,} reference and {len(mov):,} moving points")
    ref, mov = shared_zone(ref, mov)
    progress(35, "Fitting reference planes")
    report(f"shared surfaces: {len(ref):,} reference, {len(mov):,} moving points")
    if len(mov) < 500:
        raise ValueError("too little surface seen by both clouds")
    cell = 0.05
    planes = fit_planes(ref, cell)
    if len(planes) < 200:
        cell = 0.10
        planes = fit_planes(ref, cell)
    progress(50, "Matching moving surfaces")
    flat = sum(1 for _, n in planes.values() if abs(n[2]) > 0.9)
    report(f"reference planes ({int(cell * 100)} cm): {len(planes)} ({flat} flat)")
    random.seed(1)
    src = random.sample(mov, min(FIT_SAMPLES, len(mov)))
    fits = []
    starts = ((0.0, 0.0, 0.0), (search * 0.4, search * 0.4, search * 0.8), (-search * 0.4, -search * 0.4, -search * 0.8))
    for si, start in enumerate(starts):
        tick = lambda f, si=si: progress(50 + 15 * (si + f), "Fit start %d of 3" % (si + 1))
        try:
            fits.append(icp(src, planes, cell, search, start, tick))
        except ValueError as ex:
            report(f"  start {start}: {ex}")
    if not fits:
        raise ValueError("no fit converged")
    t, n, rms, diag = min(fits, key=lambda f: f[2])
    spread = max(math.sqrt(sum((f[0][i] - t[i]) ** 2 for i in range(3))) for f in fits)
    total = sum(diag)
    report(f"moving clouds sit dE {-t[0] * 1000:+.1f}  dN {-t[1] * 1000:+.1f}  dZ {-t[2] * 1000:+.1f} mm from the reference")
    report(f"{n:,} matches, rms {rms * 1000:.1f} mm, start-to-start spread {spread * 1000:.1f} mm")
    for i, ax in enumerate("ENZ"):
        weak = "  <-- weakly constrained" if diag[i] / total < 0.02 else ""
        report(f"  {ax}: {100 * diag[i] / total:.1f}% of constraint{weak}")
    if spread > 0.002 or rms > 0.008:
        report("WARNING: fits disagree or residuals are large")
    progress(100, "Fit complete")
    return t  # shift that moves the moving clouds onto the reference


# ---------------------------------------------------------------- TBC side

def region_points(regions, max_points, tick=None):
    total = sum(int(r.Integration.PointCount) for r in regions)
    stride = max(1, total // max_points)
    noop = Action[Int32](lambda pct: None)
    pts, i = [], 0
    for r in regions:
        for p in r.Integration.GetPoints(noop):
            if tick and i % 50000 == 0:
                tick(i / float(max(1, total)))
            if i % stride == 0:
                pts.append((p.X, p.Y, p.Z))
            i += 1
    return pts


def centroid(pts):
    n = float(len(pts))
    return tuple(sum(p[i] for p in pts) / n for i in range(3))


class Cancelled(Exception):
    pass


def scans_of(regions):
    scans = {}
    for r in regions:
        for s in r.Scans:
            scans[str(s.ExternalObjectId)] = s
    return scans


class BestFitScans(StackPanel):  # TBC hosts this panel because cmdData.UIForm names this class
    def __init__(self, currentProject, macroFileFolder):
        with StreamReader(Path.Combine(macroFileFolder, "BestFitScans.xaml")) as s:
            wpf.LoadComponent(self, s)
        self.currentProject = currentProject
        self.shift = None
        self.ref_regions = []
        self.mov_regions = []
        self.busy = False
        self.cancel_requested = False
        self.last_pump = 0.0

    def OnLoad(self, cmd, buttons, event):
        self.Caption = cmd.Command.Caption
        self.closeBtn = buttons[1]  # TBC's Cancel button; it only cancels while a run is going
        self.closeBtn.Content = "Close"
        self.regionType = clr.GetClrType(PointCloudRegion)
        self.regionPicker.IsEntityValidCallback = self.IsValid
        self.searchDist.Distance = 0.15
        self.manualDist.Distance = 0.02
        self.progressBar.Value = 0
        self.progressStatus.Content = "Ready"

    def IsValid(self, serial):
        return isinstance(self.currentProject.Concordance.Lookup(serial), self.regionType)

    def CancelClicked(self, cmd, args):
        if self.busy:
            self.cancel_requested = True  # picked up at the next progress update
            self.progressStatus.Content = "Cancelling..."
            return
        cmd.CloseUICommand()

    def current_selection(self):
        return [o for o in self.regionPicker.SelectedMembers(self.currentProject) if isinstance(o, self.regionType)]

    def region_names(self, regions):
        return ", ".join(str(getattr(r, "Name", "(unnamed region)")) for r in regions)

    def region_details(self, regions):
        return "; ".join("%s (%s points)" % (getattr(r, "Name", "(unnamed region)"),
                                                int(r.Integration.PointCount)) for r in regions)

    def SetReferenceClicked(self, sender, e):
        self.ref_regions = self.current_selection()
        self.referenceStatus.Content = "Reference: " + (self.region_names(self.ref_regions) if self.ref_regions else "none")

    def scan_names(self, regions):
        return ", ".join(str(getattr(s, "Name", "(unnamed scan)")) for s in scans_of(regions).values())

    def log_scans_to_move(self):
        self.log("Scans that will move (all of their points, in every region): " + self.scan_names(self.mov_regions))

    def SetMovingClicked(self, sender, e):
        self.mov_regions = self.current_selection()
        self.movingStatus.Content = ("Moving: " + self.region_names(self.mov_regions) + "  ->  scans: "
                                     + self.scan_names(self.mov_regions)) if self.mov_regions else "Moving: none"

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

    def begin_work(self):
        self.busy = True
        self.cancel_requested = False
        for b in (self.computeBtn, self.manualUpBtn, self.manualDownBtn, self.setReferenceBtn, self.setMovingBtn):
            b.IsEnabled = False
        self.closeBtn.Content = "Cancel"

    def end_work(self):
        self.busy = False
        self.cancel_requested = False
        for b in (self.computeBtn, self.manualUpBtn, self.manualDownBtn, self.setReferenceBtn, self.setMovingBtn):
            b.IsEnabled = True
        self.closeBtn.Content = "Close"

    def ComputeClicked(self, sender, e):
        if self.busy:
            return
        self.begin_work()
        try:
            self.compute()
        except Cancelled:
            self.shift = None
            self.progressStatus.Content = "Cancelled"
            self.log("Cancelled.")
        finally:
            self.end_work()

    def compute(self):
        self.resultText.Text = ""
        self.shift = None
        ref, mov = self.ref_regions, self.mov_regions
        if not ref or not mov:
            self.log("Select regions in the TBC view and click both capture buttons first.")
            return
        self.log("Reference: " + self.region_details(ref))
        self.log("Moving: " + self.region_details(mov))
        self.log_scans_to_move()
        self.log("Search distance: %.3f m" % float(self.searchDist.Distance))
        shared = set(scans_of(ref)) & set(scans_of(mov))
        if shared:
            self.log(f"The reference and moving regions share {len(shared)} scan(s); shifting them would move the reference too.")
            return
        try:
            ref_points = region_points(ref, REFERENCE_MAX_POINTS,
                                       lambda f: self.progress(15 * f, "Reading reference points"))
            mov_points = region_points(mov, MOVING_MAX_POINTS,
                                       lambda f: self.progress(15 + 5 * f, "Reading moving points"))
            self.shift = best_fit(ref_points, mov_points, float(self.searchDist.Distance), self.log, self.progress)
            self.log("Press OK to shift the scans listed above by this amount.")
        except Cancelled:
            raise
        except Exception as ex:
            self.log(f"Fit failed: {ex}")

    def ManualUpClicked(self, sender, e):
        self.set_manual_shift(1)

    def ManualDownClicked(self, sender, e):
        self.set_manual_shift(-1)

    def set_manual_shift(self, sign):
        """Use a typed vertical offset instead of a computed fit; only the Moving regions are needed."""
        self.resultText.Text = ""
        self.shift = None
        if not self.mov_regions:
            self.log("Select regions in the TBC view and click the Moving capture button first.")
            return
        dz = sign * abs(float(self.manualDist.Distance))
        if dz == 0:
            self.log("Enter a non-zero amount.")
            return
        self.log("Moving: " + self.region_details(self.mov_regions))
        self.log_scans_to_move()
        self.log(f"Manual shift: dZ {dz * 1000:+.1f} mm ({'up' if dz > 0 else 'down'})")
        self.shift = (0.0, 0.0, dz)
        self.progress(100, "Manual shift set")
        self.log("Press OK to shift the scans listed above by this amount.")

    def OkClicked(self, cmd, args):
        if self.busy:
            return False
        self.begin_work()
        try:
            self.apply_shift()
        except Cancelled:
            self.progressStatus.Content = "Cancelled"
            self.log("Cancelled; nothing was changed.")
        finally:
            self.end_work()
        return False  # keep the panel open so the report stays visible

    def apply_shift(self):
        if self.shift is None:
            self.log("Compute the fit first.")
            return
        mov = self.mov_regions
        scans = scans_of(mov)
        t = self.shift
        method_name = "Scan.Transform" if self.methodCombo.SelectedIndex == 0 else "Scan.ApplyTransformation"
        self.log("Applying with " + method_name + ".")
        self.log("Moving scans: " + ", ".join(str(getattr(s, "Name", "(unnamed scan)")) for s in scans.values()))
        before = centroid(region_points(mov, VERIFY_POINTS, lambda f: self.progress(40 * f, "Reading points before the shift")))
        committed = False
        manager = self.currentProject.TransactionManager
        manager.AddBeginMark(CommandGranularity.Command, "Best-Fit Scans")
        try:
            # changes made inside the "with" block are rolled back unless Commit() is called,
            # including when Cancel raises Cancelled out of a progress update
            with TransactMethodCall(self.currentProject.TransactionCollector) as failGuard:
                self.progress(40, "Moving scans")
                for scan in scans.values():
                    if self.methodCombo.SelectedIndex == 0:
                        ok = scan.Transform(Matrix4D(Vector3D(t[0], t[1], t[2])))
                    else:
                        frame = scan.Integration.Frame
                        pos = frame.Position
                        ok = scan.ApplyTransformation(Vector3D(pos.X + t[0], pos.Y + t[1], pos.Z + t[2]),
                                                      Spinor3D(frame.RotationMatrix))
                    if not ok:
                        self.log(f"TBC refused to move scan {scan.Name}; nothing was changed.")
                        return
                after = centroid(region_points(mov, VERIFY_POINTS,
                                               lambda f: self.progress(50 + 50 * f, "Checking the points moved")))
                moved = [after[i] - before[i] for i in range(3)]
                self.log(f"points moved dE {moved[0] * 1000:+.1f}  dN {moved[1] * 1000:+.1f}  dZ {moved[2] * 1000:+.1f} mm "
                         f"(expected {t[0] * 1000:+.1f} {t[1] * 1000:+.1f} {t[2] * 1000:+.1f})")
                worst = max(abs(moved[i] - t[i]) for i in range(3))
                if self.verifyCB.IsChecked and worst > max(0.003, 0.1 * math.sqrt(sum(v * v for v in t))):
                    self.log("Points did not move as expected; rolled back. (If they did not move at all, TBC may only "
                             "update point positions after commit: untick verification to test that on a copy.)")
                    return
                failGuard.Commit()
                committed = True
        finally:
            manager.AddEndMark(CommandGranularity.Command)
            self.progress(100, "Done" if committed else "Not changed", can_cancel=False)
        if committed:
            self.log(f"Shifted {len(scans)} scan(s). Use Undo to reverse.")
            self.shift = None
