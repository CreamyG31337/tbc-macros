"""Synthetic tests of the macros' pure-Python code (no TBC, no scan data). Run with any Python 3:

    python tests/test_synthetic.py

Each test prints its numbers and fails with AssertionError if a check doesn't hold."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import macrocode  # noqa: E402


def wall_and_floor(noise, outliers):
    """3 x 3 m floor (z = 0) and wall (x = 3) with gaussian noise, plus outliers 3-30 cm off them."""
    pts, kind = [], []
    for _ in range(60000):
        pts.append((random.uniform(0, 3), random.uniform(0, 3), random.gauss(0, noise))); kind.append("floor")
    for _ in range(60000):
        pts.append((3 + random.gauss(0, noise), random.uniform(0, 3), random.uniform(0, 2.5))); kind.append("wall")
    for _ in range(outliers):
        if random.random() < 0.5:
            pts.append((random.uniform(0, 2.9), random.uniform(0, 3), random.choice((-1, 1)) * random.uniform(0.03, 0.3)))
        else:
            pts.append((3 + random.choice((-1, 1)) * random.uniform(0.03, 0.3), random.uniform(0, 3), random.uniform(0.1, 2.5)))
        kind.append("outlier")
    return pts, kind


def test_clean_cloud():
    random.seed(0)
    ns = macrocode.load("CleanCloud")
    pts, kind = wall_and_floor(0.004, 1500)
    s50, s95 = ns["point_spacing"](pts)
    cell = min(ns["MAX_CELL"], max(ns["MIN_CELL"], round(ns["AUTO_CELL"] * s50 / 0.005) * 0.005))
    gap = max(1.2 * s50, s95)
    ticks = []
    for flatten in (False, True):
        out, counts, info = ns["clean"](pts, cell, None, flatten, True, True, ticks.append, gap, ns["MIN_PIECE"])
        kept = {"floor": 0, "wall": 0, "outlier": 0}
        dev = {"floor": [], "wall": []}
        for i, p in out:
            kept[kind[i]] += 1
            if kind[i] == "wall":
                dev["wall"].append(p[0] - 3)
            elif kind[i] == "floor":
                dev["floor"].append(p[2])
        rms = dict((k, 1000 * math.sqrt(sum(d * d for d in v) / len(v))) for k, v in dev.items())
        print(f"clean (flatten={flatten}): cell {cell * 1000:.0f} mm, tol {info[0][1] * 1000:.1f} mm; kept floor "
              f"{kept['floor'] / 600:.1f}%, wall {kept['wall'] / 600:.1f}%, outliers {kept['outlier']}; "
              f"rms floor {rms['floor']:.2f} wall {rms['wall']:.2f} mm")
        assert kept["floor"] >= 0.98 * 60000 and kept["wall"] >= 0.98 * 60000, "real surface lost"
        assert kept["outlier"] <= 15, "outliers kept"
        if flatten:  # 4 mm noise; planes over 3 x 3 x 3 small cells only average it down so far
            assert rms["floor"] < 0.6 * 4 and rms["wall"] < 0.6 * 4, "smoothing didn't reduce the noise"
    assert all(b >= a for a, b in zip(ticks[:len(ticks) // 2], ticks[1:len(ticks) // 2])), "progress went backwards"
    assert all(0 <= t <= 1 for t in ticks)


def test_separate_objects():
    random.seed(1)
    ns = macrocode.load("SeparateObjects")
    pts, kind = [], []
    for _ in range(40000):  # 2 x 2 m wall in the x-z plane at y = 0, ~1 cm spacing, 2 mm noise
        pts.append((random.uniform(0, 2), random.gauss(0, 0.002), random.uniform(0, 2))); kind.append("wall")
    for _ in range(600):  # anchor: 3 cm diameter bar sticking 10 cm out of the wall at (0.5, 1.5)
        a, h = random.uniform(0, 2 * math.pi), random.uniform(0, 0.10)
        pts.append((0.5 + 0.015 * math.cos(a), h, 1.5 + 0.015 * math.sin(a))); kind.append("anchor")
    for _ in range(3000):  # hose: 4 cm diameter lying on the wall from (1.5, 0.1) to (1.5, 1.6), visible half
        a, z = random.uniform(0, math.pi), random.uniform(0.1, 1.6)
        pts.append((1.5 + 0.02 * math.cos(a), 0.02 + 0.02 * math.sin(a), z)); kind.append("hose")
    grid = ns["build_grid"](pts)
    link = ns["link_distance"](pts, grid)
    anchor, _ = ns["select_round"](pts, grid, (0.5, 0.10, 1.5), 0.10, 0.008, link)
    hose, _ = ns["select_tube"](pts, grid, [(1.5, 0.04, z) for z in (0.15, 0.85, 1.55)], 0.05, 0.008, link)
    for name, sel, label in (("anchor", anchor, "anchor"), ("hose", hose, "hose")):
        got = sum(1 for i in sel if kind[i] == label)
        total = kind.count(label)
        wall = sum(1 for i in sel if kind[i] == "wall")
        print(f"separate {name}: {100.0 * got / total:.0f}% of it selected, {wall} wall points (link {link * 1000:.0f} mm)")
        assert got >= 0.8 * total, f"{name}: too little selected"
        assert wall <= 0.1 * total, f"{name}: too much wall"


def test_best_fit():
    random.seed(2)
    ns = macrocode.load("BestFitScans")

    def corner(shift, n):
        pts = []
        for _ in range(n):  # floor, and two walls, 1 x 1 m each, 2 mm noise
            u, v, w = random.uniform(0, 1), random.uniform(0, 1), random.gauss(0, 0.002)
            pts.append(random.choice(((u, v, w), (w, u, v), (u, w, v))))
        return [(x + shift[0], y + shift[1], z + shift[2]) for x, y, z in pts]

    true = (0.012, -0.021, 0.007)  # moving cloud sits this far from the reference
    ref, mov = corner((0, 0, 0), 60000), corner(true, 30000)
    t = ns["best_fit"](ref, mov, 0.15, lambda s: None, lambda v, s: None)
    err = max(abs(t[k] + true[k]) for k in range(3))
    print(f"best fit: shift {[round(1000 * v, 1) for v in t]} mm, expected {[-1000 * v for v in true]}, error {1000 * err:.2f} mm")
    assert err < 0.001, "best fit off by more than 1 mm"


if __name__ == "__main__":
    for test in (test_clean_cloud, test_separate_objects, test_best_fit):
        test()
    print("all passed")
