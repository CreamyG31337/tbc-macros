"""
Point Cloud Tools - Separate Objects (EXPERIMENTAL: test on a copy of a project).

Click-to-select anchors / dowels, hoses / cables and other junk on a rock face, and write them to
their own regions. Each object is found by comparing it with the rock right next to it:
- one click (anchor, junk): points within the object radius of the click that stand more than the
  off-rock distance off a plane fitted to the ring of points just outside that radius;
- clicks along it (hose): the same along a tube through the clicks, one plane per segment, fitted
  to the points just outside the tube.
Only the selected points connected to the click (or the clicked line) are kept, so rock bumps beside
the object drop out. Simulated on a labelled SX12 channel wall: a median 84-90% of each anchor and
83% of each hose selected, with a median 0-14 rock points dragged in per object (docs/testing.md).
OK writes '- anchors', '- hoses', '- junk' and '- rest' regions as new scans inside one
transaction, each checked by reading it back. The selected region and its scans are not changed.
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
clr.AddReference("Trimble.Vce.UI.Controls")
for name in ("Trimble.Sdk.Scanning", "Trimble.Sdk.Interfaces", "Trimble.Sdk.UI"):
    try:
        clr.AddReference(name)
    except Exception:
        pass  # already loaded by TBC under another name
from Trimble.Sdk.UI import InputMethod
from Trimble.Vce.Data.Scanning import PointCloudRegion, PointCloudDatabaseContainer, ExposedPointCloudRegionContainer
from Trimble.Vce.Interfaces.PointCloudIntegration import PointInfo, ColorMode

MAX_POINTS = 12000000
GRID = 0.05            # cell size of the lookup grid for "points near the click"
RING = 2.5             # the rock plane is fitted to points between 1 x and RING x the object radius
MIN_RING_POINTS = 10
CLICK_START = 0.02     # selected points this close to a click (or the hose line) start the object
LINK = 2.5             # object points are linked through steps of LINK x the median point spacing
CLICK_REACH = 0.10     # a click further than this from the region's points is ignored
TYPES = ("anchors", "hoses", "junk")
COLOURS = {"anchors": (40, 90, 230), "hoses": (240, 150, 30), "junk": (210, 40, 210)}


def Setup(cmdData, macroFileFolder):
    cmdData.Key = "PointCloudTools_SeparateObjects"
    cmdData.CommandName = "PointCloudTools_SeparateObjects"
    cmdData.Caption = "_Separate Objects"
    cmdData.UIForm = "SeparateObjects"
    try:
        cmdData.DefaultTabKey = "Macros"
        cmdData.DefaultTabGroupKey = "Point Cloud Tools"
        cmdData.ShortCaption = "Separate Objects"
        cmdData.ToolTipTitle = "Separate Objects"
        cmdData.ToolTipTextFormatted = "Click anchors, hoses and junk on a rock face to move them to their own regions. Experimental."
        cmdData.Version = 0.2
        cmdData.MacroAuthor = "tbc-macros"
    except Exception:
        pass


# ---------------------------------------------------------------- geometry (pure Python)

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


def build_grid(pts):
    grid = {}
    for i, p in enumerate(pts):
        k = cell_key(p, GRID)
        cell = grid.get(k)
        if cell is None:
            grid[k] = [i]
        else:
            cell.append(i)
    return grid


def near(pts, grid, c, radius):
    """Indices of points within radius of c."""
    k = cell_key(c, GRID)
    reach = int(math.ceil(radius / GRID))
    r2 = radius * radius
    out = []
    for a in range(-reach, reach + 1):
        for b in range(-reach, reach + 1):
            for d in range(-reach, reach + 1):
                for j in grid.get((k[0] + a, k[1] + b, k[2] + d), ()):
                    q = pts[j]
                    if (q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2 + (q[2] - c[2]) ** 2 <= r2:
                        out.append(j)
    return out


def link_distance(pts, grid, samples=2000):
    """LINK x the median nearest-neighbour spacing over an even sample, within 1-5 cm."""
    nearest = []
    for i in range(0, len(pts), max(1, len(pts) // samples)):
        p = pts[i]
        d2 = [(pts[j][0] - p[0]) ** 2 + (pts[j][1] - p[1]) ** 2 + (pts[j][2] - p[2]) ** 2
              for j in near(pts, grid, p, GRID) if j != i]
        if d2:
            nearest.append(math.sqrt(min(d2)))
    if not nearest:
        return 0.05
    nearest.sort()
    return min(0.05, max(0.01, LINK * nearest[len(nearest) // 2]))


def off_plane(p, pl):
    c, nv = pl
    return nv[0] * (p[0] - c[0]) + nv[1] * (p[1] - c[1]) + nv[2] * (p[2] - c[2])


def fit_plane(pts, idx):
    """Least-squares plane (centre, normal) through pts[idx], refitted twice without the worst 20%
    so a bit of the object reaching into the ring doesn't tilt it. None if too few points."""
    pl = None
    for _ in range(3):
        n = len(idx)
        if n < MIN_RING_POINTS:
            return None
        m = [sum(pts[i][k] for i in idx) / float(n) for k in range(3)]
        c = [0.0] * 6
        for i in idx:
            dx, dy, dz = pts[i][0] - m[0], pts[i][1] - m[1], pts[i][2] - m[2]
            c[0] += dx * dx; c[1] += dy * dy; c[2] += dz * dz; c[3] += dx * dy; c[4] += dx * dz; c[5] += dy * dz
        _, nv = sym3_eig([v / n for v in c])
        pl = (tuple(m), nv)
        d = sorted((abs(off_plane(pts[i], pl)), i) for i in idx)
        idx = [i for _, i in d[:max(MIN_RING_POINTS, int(0.8 * len(d)))]]
    return pl


def connected_to(pts, grid, sel, start, link):
    """The part of sel linked to the start points through steps of at most link. Rock bumps that
    pass the off-rock test sit beside the object, not attached to it, so they drop out."""
    keep = set(start)
    stack = list(start)
    while stack:
        a = stack.pop()
        for j in near(pts, grid, pts[a], link):
            if j in sel and j not in keep:
                keep.add(j)
                stack.append(j)
    return keep


def select_round(pts, grid, click, radius, off_rock, link):
    """One-click object: points within radius of click standing more than off_rock off the rock ring
    around it, connected to the click."""
    inner = near(pts, grid, click, radius)
    inner_set = set(inner)
    ring = [j for j in near(pts, grid, click, RING * radius) if j not in inner_set]
    pl = fit_plane(pts, ring)
    if pl is None:
        return set(inner), "no rock around it to compare with: took everything within the radius"
    sel = set(j for j in inner if abs(off_plane(pts[j], pl)) > off_rock)
    reach = max(CLICK_START, link)
    start = [j for j in sel if sum((pts[j][k] - click[k]) ** 2 for k in range(3)) <= reach * reach]
    if not start and sel:
        start = [min(sel, key=lambda j: sum((pts[j][k] - click[k]) ** 2 for k in range(3)))]
    return connected_to(pts, grid, sel, start, link), None


def seg_dist(p, a, b):
    """Distance from p to segment ab, and the position along it (0..1)."""
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    L2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2 or 1e-12
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / L2))
    return math.sqrt((p[0] - a[0] - t * ab[0]) ** 2 + (p[1] - a[1] - t * ab[1]) ** 2 + (p[2] - a[2] - t * ab[2]) ** 2), t


def select_tube(pts, grid, clicks, radius, off_rock, link):
    """Clicks along a hose: per segment, points within radius of it standing off the rock beside it,
    connected to points near the clicked line."""
    sel, notes, start = set(), [], []
    reach = max(CLICK_START, link)
    for a, b in zip(clicks, clicks[1:]):
        mid = tuple((a[k] + b[k]) / 2 for k in range(3))
        half = math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3))) / 2
        inner, ring = [], []
        for j in near(pts, grid, mid, half + RING * radius):
            dist, t = seg_dist(pts[j], a, b)
            if dist <= radius:
                inner.append(j)
            elif dist <= RING * radius and 0 < t < 1:
                ring.append(j)
        pl = fit_plane(pts, ring)
        if pl is None:
            notes.append("a segment had no rock beside it: took everything in the tube there")
            part = set(inner)
        else:
            part = set(j for j in inner if abs(off_plane(pts[j], pl)) > off_rock)
        sel |= part
        start += [j for j in part if seg_dist(pts[j], a, b)[0] <= reach]
    return connected_to(pts, grid, sel, start, link), "; ".join(notes) or None


# ---------------------------------------------------------------- TBC side

def region_name(region):
    return str(getattr(region, "Name", None) or "(unnamed region)")


class Cancelled(Exception):
    pass


class SeparateObjects(StackPanel):  # TBC hosts this panel because cmdData.UIForm names this class
    def __init__(self, currentProject, macroFileFolder):
        with StreamReader(Path.Combine(macroFileFolder, "SeparateObjects.xaml")) as s:
            wpf.LoadComponent(self, s)
        self.currentProject = currentProject
        self.busy = False
        self.cancel_requested = False
        self.last_pump = 0.0
        self.loaded = None      # (region serials, regions, infos, pts, grid, origin, scale)
        self.objects = []       # [(type, set of indices, description)]
        self.hose_clicks = []

    def OnLoad(self, cmd, buttons, event):
        self.Caption = cmd.Command.Caption
        self.closeBtn = buttons[1]  # TBC's Cancel button; it only cancels while a run is going
        self.closeBtn.Content = "Close"
        self.regionType = clr.GetClrType(PointCloudRegion)
        self.regionPicker.IsEntityValidCallback = self.IsValid
        self.objectRadius.Distance = 0.10
        self.hoseRadius.Distance = 0.05
        self.offRock.Distance = 0.008
        self.import_range = (0.0, 100.0)
        self.coordCtl.ValueChanged += self.CoordChanged
        self.progressStatus.Content = "Select the region, then click objects"

    def IsValid(self, serial):
        return isinstance(self.currentProject.Concordance.Lookup(serial), self.regionType)

    def CancelClicked(self, cmd, args):
        if self.busy:
            self.cancel_requested = True
            self.progressStatus.Content = "Cancelling..."
            return
        cmd.CloseUICommand()

    def log(self, text):
        self.resultText.Text = (self.resultText.Text + "\n" + text).lstrip("\n")
        self.resultText.ScrollToEnd()

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
        self.closeBtn.Content = "Cancel"

    def end_work(self):
        self.busy = False
        self.cancel_requested = False
        self.closeBtn.Content = "Close"

    def stored_coordinates(self):
        """Context in which points are read and written as TBC stores them, without the project's
        scale factor. Points read with the scale factor and written back get scaled twice."""
        return PointCloudDatabaseContainer.GetInstance(self.currentProject).TemporarilySetGroundScaleFactorToOne()

    # ------------------------------------------------------------ loading and clicking

    def selected_regions(self):
        return [o for o in self.regionPicker.SelectedMembers(self.currentProject) if isinstance(o, self.regionType)]

    def ensure_loaded(self):
        """Reads the selected regions once (the first click); returns False if there's nothing to use."""
        regions = self.selected_regions()
        if not regions:
            self.log("Select the point cloud region first.")
            return False
        serials = tuple(sorted(int(getattr(r, "SerialNumber", id(r))) for r in regions))
        if self.loaded is not None and self.loaded[0] == serials:
            return True
        if self.objects or self.hose_clicks:
            self.log("The region changed: objects picked so far were cleared.")
        self.objects, self.hose_clicks = [], []
        total = sum(int(r.Integration.PointCount) for r in regions)
        if total > MAX_POINTS:
            self.log(f"{total:,} points is more than this macro handles ({MAX_POINTS:,}); use smaller regions.")
            return False
        self.begin_work()
        try:
            noop = Action[Int32](lambda pct: None)
            # clicks come in display coordinates (with the project scale factor); points are kept as
            # stored. The ratio of the regions' box centres in the two converts between them.
            scale = self.box_scale(regions)
            infos, pts, origin = [], [], None
            with self.stored_coordinates():
                for r in regions:
                    for pi in r.Integration.GetPointInfos(noop):
                        if len(pts) % 20000 == 0:
                            self.progress(80.0 * len(pts) / total, f"Reading points {len(pts):,} / {total:,}")
                        c = pi.Coordinates
                        if origin is None:
                            origin = (c.X, c.Y, c.Z)
                        infos.append(pi)
                        pts.append((c.X - origin[0], c.Y - origin[1], c.Z - origin[2]))
            if not pts:
                self.log("The selected regions have no points.")
                return False
            self.progress(85, "Indexing points")
            grid = build_grid(pts)
            self.progress(95, "Measuring point spacing")
            self.link = link_distance(pts, grid)
            self.loaded = (serials, regions, infos, pts, grid, origin, scale)
            self.progress(100, "Ready: click objects")
            self.log(f"Loaded {len(pts):,} points from " + ", ".join(region_name(r) for r in regions) +
                     f"; object points link within {self.link * 1000:.0f} mm.")
            return True
        except Cancelled:
            self.log("Cancelled.")
            return False
        finally:
            self.end_work()

    def box_scale(self, regions):
        """Per-axis display / stored coordinate ratio from the first region's bounding box."""
        def centre(box):
            o = box.Origin
            return (o.X + box.SizeX / 2, o.Y + box.SizeY / 2, o.Z + box.SizeZ / 2)
        try:
            shown = centre(regions[0].Integration.BoundingBox)
            with self.stored_coordinates():
                stored = centre(regions[0].Integration.BoundingBox)
            return tuple(shown[k] / stored[k] if stored[k] else 1.0 for k in range(3))
        except Exception as ex:
            self.log(f"Couldn't compare coordinates with and without the scale factor ({ex}); assuming none.")
            return (1.0, 1.0, 1.0)

    def click_point(self, ctrl):
        """The clicked coordinate in the loaded points' frame, or None if it's not on them."""
        _, _, _, pts, grid, origin, scale = self.loaded
        c = ctrl.Coordinate
        p = (c.X / scale[0] - origin[0], c.Y / scale[1] - origin[1], c.Z / scale[2] - origin[2])
        if not near(pts, grid, p, CLICK_REACH):
            self.log(f"That click isn't on the selected region's points (nothing within {CLICK_REACH * 100:.0f} cm); "
                     "turn on snapping to point clouds and click on the object.")
            return None
        return p

    def CoordChanged(self, ctrl, e):
        if self.busy or e.Cause not in (InputMethod.Mouse, InputMethod.Snap):
            return
        if not self.ensure_loaded():
            return
        p = self.click_point(ctrl)
        if p is None:
            return
        kind = TYPES[self.typeCombo.SelectedIndex]
        if kind == "hoses":
            self.hose_clicks.append(p)
            self.log(f"Hose point {len(self.hose_clicks)}" + (": click along it, then Finish hose" if len(self.hose_clicks) == 1 else ""))
            return
        _, _, _, pts, grid, _, _ = self.loaded
        sel, note = select_round(pts, grid, p, float(self.objectRadius.Distance), float(self.offRock.Distance), self.link)
        self.add_object(kind, sel, note)

    def FinishHoseClicked(self, sender, e):
        if len(self.hose_clicks) < 2:
            self.log("Click at least two points along the hose first.")
            return
        _, _, _, pts, grid, _, _ = self.loaded
        sel, note = select_tube(pts, grid, self.hose_clicks, float(self.hoseRadius.Distance), float(self.offRock.Distance),
                                self.link)
        length = sum(math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3))) for a, b in zip(self.hose_clicks, self.hose_clicks[1:]))
        self.hose_clicks = []
        self.add_object("hoses", sel, note, f"{length:.1f} m")

    def add_object(self, kind, sel, note, extra=""):
        taken = set()
        for _, s, _ in self.objects:
            taken |= s
        new = sel - taken
        if not new:
            self.log(f"Nothing new selected there (no points stand more than {float(self.offRock.Distance) * 1000:.0f} mm "
                     "off the rock around the click, or they were already picked).")
            return
        desc = f"{kind[:-1]} {sum(1 for k, _, _ in self.objects if k == kind) + 1}: {len(new):,} points" + (f", {extra}" if extra else "")
        self.objects.append((kind, new, desc))
        self.log(desc + (f" ({note})" if note else ""))
        self.progressStatus.Content = self.summary()

    def summary(self):
        return ", ".join(f"{sum(1 for k, _, _ in self.objects if k == t)} {t}" for t in TYPES)

    def UndoClicked(self, sender, e):
        if self.hose_clicks:
            self.hose_clicks.pop()
            self.log(f"Removed the last hose point ({len(self.hose_clicks)} left).")
        elif self.objects:
            self.log(f"Removed {self.objects.pop()[2]}.")
            self.progressStatus.Content = self.summary()
        else:
            self.log("Nothing to undo.")

    # ------------------------------------------------------------ writing

    def OkClicked(self, cmd, args):
        if self.busy:
            return False
        if self.loaded is None or not self.objects:
            self.log("Click some objects first.")
            return False
        if self.hose_clicks:
            self.log("Finish or undo the hose being clicked first.")
            return False
        self.begin_work()
        try:
            self.write()
        except Cancelled:
            self.progressStatus.Content = "Cancelled"
            self.log("Cancelled; nothing was written.")
        except Exception as ex:
            self.log(f"Failed: {ex}")
        finally:
            self.end_work()
        return False  # keep the panel open so the report stays visible

    def write(self):
        _, regions, infos, pts, _, origin, _ = self.loaded
        owner = {}
        for kind, sel, _ in self.objects:
            for i in sel:
                owner.setdefault(i, kind)
        groups = dict((t, (List[PointInfo](), [])) for t in TYPES + ("rest",))
        for i, pi in enumerate(infos):
            if i % 50000 == 0:
                self.progress(10.0 * i / len(infos), "Sorting points")
            g = groups[owner.get(i, "rest")]
            g[0].Add(pi)
            g[1].append(pts[i])
        order = [t for t in TYPES + ("rest",) if groups[t][1]]
        base = region_name(regions[0])
        committed = False
        manager = self.currentProject.TransactionManager
        manager.AddBeginMark(CommandGranularity.Command, "Separate Objects")
        try:
            # changes made inside the "with" block are rolled back unless Commit() is called
            with TransactMethodCall(self.currentProject.TransactionCollector) as failGuard:
                written = {}
                step = 90.0 / len(order)
                for k, t in enumerate(order):
                    ok, region = self.write_scan(f"{base} - {t}", groups[t][0], groups[t][1], origin,
                                                 10 + k * step, 10 + (k + 1) * step)
                    if not ok:
                        self.log("A new scan doesn't match the points written; rolled back. If a new region is "
                                 "still listed, delete it, and send this report.")
                        return
                    written[t] = region
                if self.showResultCB.IsChecked:
                    self.show_result(regions, written)
                failGuard.Commit()
                committed = True
        finally:
            manager.AddEndMark(CommandGranularity.Command)
            self.progress(100, "Done" if committed else "Not written", can_cancel=False)
        if committed:
            self.log("Written. Use Undo to remove the new regions.")
            self.objects, self.loaded = [], None

    def write_scan(self, name, points, expected, origin, lo, hi):
        """Writes points as a new scan in a new region called name, inside the caller's transaction,
        and checks that it reads back with the expected count and position. Returns (ok, region).
        Same as CleanCloud.write_scan."""
        project = self.currentProject
        database = PointCloudDatabaseContainer.GetInstance(project).PrimaryDatabase
        region_container = ExposedPointCloudRegionContainer.GetInstance(project)
        noop = Action[Int32](lambda pct: None)
        ox, oy, oz = origin
        n = float(len(expected))
        expected_centre = tuple(sum(p[k] for p in expected) / n for k in range(3))
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
        off = [(sx / i, sy / i, sz / i)[k] - expected_centre[k] for k in range(3)]
        ok = abs(written - len(expected)) <= 0.01 * len(expected) and max(abs(v) for v in off) <= 0.005
        if not ok:
            self.log(f"  centre offset dE {off[0] * 1000:+.1f}  dN {off[1] * 1000:+.1f}  dZ {off[2] * 1000:+.1f} mm")
        return ok, region

    def show_result(self, sources, written):
        """Hide the source regions in the view filters, show 'rest' in true colour and each object
        type in its own colour. Cosmetic: failures are reported and the rest carries on."""
        for t, region in written.items():
            try:
                if t == "rest":
                    region.ColorMode = ColorMode.PointRGB
                    continue
                r, g, b = COLOURS[t]
                for colour in ([DrawingColor.FromArgb(255, r, g, b)] if DrawingColor is not None else []) + [MediaColor.FromRgb(r, g, b)]:
                    try:
                        region.Color = colour
                        break
                    except Exception:
                        continue
                region.ColorMode = ColorMode.Model  # "region colour" in TBC's colour mode list
            except Exception as ex:
                self.log(f"  could not colour '{region_name(region)}': {ex}")
        try:
            types = self.currentProject.GetMemberContainer(MandatoryContainers.ImitationTypeCollection)
            serial = lambda region: types.Lookup(region.ImitationTypeGuid).SerialNumber
            changes = [(serial(r), False) for r in sources] + [(serial(r), True) for r in written.values()]
            collection = self.currentProject.GetMemberContainer(MandatoryContainers.ViewFilterCollection)
            filters = []
            try:
                for item in collection:
                    f = item if hasattr(item, "ImitationTypeOverrides") else self.currentProject.Concordance.Lookup(item)
                    if hasattr(f, "ImitationTypeOverrides"):
                        filters.append(f)
            except Exception:
                pass
            for f in filters or [collection.DefaultViewFilter]:
                if f.IsReadOnly:
                    continue
                overrides = f.ImitationTypeOverrides
                for s, visible in changes:
                    rendering = overrides[s] if overrides.Contains(s) else overrides.Add(s)
                    rendering.Visible = visible
        except Exception as ex:
            self.log(f"  could not update the view filters: {ex}")
