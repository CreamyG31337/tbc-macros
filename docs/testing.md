# Testing and tuning notes

How the filters in Clean Point Cloud 3D and Separate Objects were tuned, and what they did on
test data. The scripts are in `tests/`; the test scans themselves aren't in the repository.

Test data used (all of rock walls and trenches):

| Data | Points | Spacing (median / 95th %) | Notes |
|---|---|---|---|
| NavVis VLX3 SLAM scan, 40 m of trench wall (two sides) | 1.7-1.8 M per section | 17 / 21 mm | Even density; noisy upper walls with wire mesh rolls |
| Trimble SX12 scans (footings; a trench wall, also hand-labelled) | 2.7-3.4 M per section | 3.7-9.5 / 8-15 mm | Very clean, density varies a lot with range |
| Drone photogrammetry over the same trench | 0.77 M | 71 / 88 mm | Vertical walls poorly captured |

## Clean Point Cloud 3D

### How it decides

1. **Floating pieces.** Points are linked if within about 2 x a gap; pieces under 30 points are
   left out of the finest plane fit and removed unless they lie on the surface: near dense
   surface, on a flat finest plane within tolerance; where only coarser planes reach (sparse
   areas), within 3 x the RMS thickness of a coarser plane fitted without the small pieces.
   Gap = max(1.2 x median spacing, 95th-percentile spacing).
2. **Local planes.** Planes are fitted to the 3 x 3 x 3 cells around each cell, at the cell size
   and at 2 x and 4 x; each point is judged at the finest level with enough points (10) around
   it. Areas that aren't flat (smallest / middle eigenvalue > 0.1) are kept as they are; on flat
   ones, points further than the tolerance from the plane are removed. Coarser levels' tolerance
   grows with the scatter measured there.
3. **Automatic settings.** Cell = 3 x the median spacing (20-500 mm); tolerance = 4 x the robust
   scatter (1.4826 x median distance) about the finest planes.
4. **Safety.** A run that would remove more than 20% is refused.

### What the tests showed

- **Fixed settings fail across scanners.** On the NavVis wall, 50 mm cells with a 10 mm tolerance
  removed 10.7%, mostly real rock along ridges: the "flat" test accepts neighbourhoods scattering
  up to ~14 mm, beyond a 10 mm tolerance. 50 / 20 mm didn't erode rock; tighter tolerances
  erode fast (15 mm: 4.7%, 12 mm: 9.8%). On the drone data 50 / 20 mm would have removed 99.99%
  (no 5 cm cell holds enough points); hence automatic settings and the 20% stop.
- **Clumps hide from the plane test.** With known noise injected into the NavVis wall, the plane
  test alone removed 84% of single flyers but only 2% of floating clumps (a clump isn't flat, so
  it's kept as "rough") and 38% of haze. Floating-piece removal brought that to 97-99% of
  flyers, 64-72% of clumps and 92% of haze.
- **Static scanners need the sparse end of the spacing.** On the SX12 footings scan a gap from
  the median spacing removed 15.7% (real surface broken into pieces); from the 95th percentile,
  1.8%; also requiring pieces to be off the surface, with multi-level planes, 0.84%.
- **Results with automatic settings:** NavVis 1.0-2.6% removed, drone 1.3%, SX12 0.84%,
  synthetic floor + wall 98.6-98.9% kept with all 1,500 outliers removed.
- **Hand-labelled SX12 wall** (rock surface, and debris / cables / anchors, split by hand in
  TBC): 0.46% of the rock removed. The plane test's removals were a median 14 mm off the surface
  (strays the manual clean missed); of the floating pieces, about half were slivers along the
  region's cut edges and ~0.06% of the rock was real surface lost. 7.3% of the debris was caught;
  anchors were mostly kept.
- **Sparse, thick areas lost surface as "floating pieces".** A NavVis wall section the scanner
  couldn't get close to (a puddle in front) was sparse and ~10 cm thick, and the clean widened its
  gaps: viewed face-on, 1.8% of the 5 cm cells that had points were emptied, almost all by the
  floating-piece test (cross-sections showed the pieces inside the band, not in front of it).
  Keeping small pieces that are within 3 x the thickness of a coarser surface fitted without
  them cut that to 0.54% with the same share of injected flyers (87%) and clumps (28%) caught.
  Applying the thickness test near dense surface too, or measuring thickness with the small
  pieces included (clumps widen it), let 3-11% more injected noise through. On the other test
  scans the change kept within 1-3% of the injected noise caught before.
- **Tried and dropped:** refitting planes without outliers (little gain, slower), a local density
  test (no gain on real data), keeping small pieces in rough areas (lost more noise than it
  saved rock).
- **Smoothing: curved surfaces instead of the cell planes.** The first "move onto the local
  surface" moved points onto the cell planes. On a synthetic rough wall (17 mm spacing) it made
  6 mm noise slightly worse at the automatic cell (6.0 -> 6.7 mm from the true surface: it flattens
  relief) and left 20 mm noise untouched (all "rough"). Quadratic surfaces fitted in overlapping,
  tent-weighted blocks and blended (moving least squares, as in CloudCompare's PCL plugin) took
  6 mm noise to 1.8-2.0 mm and 20 mm to 5.8 mm, with 2 x blocks where a block's residual RMS x 8
  exceeds its size. On real scans, fitted to half the points and scored on the other half (lower
  = follows the surface better; includes the noise): NavVis problem area 8.2 -> 5.5 mm, NavVis
  wall 7.5 -> 3.8 mm, SX12 3.2 -> 1.7 mm. Best block size about 3.5 x the spacing. Using 2 x
  blocks wherever the residual is high made real scans worse (the residual there is relief, not
  noise); only switching where the 2 x blocks fit nearly as well (RMS < 1.25 x) is safe. Sparse or
  lopsided blocks need their curvature damped, or their surfaces swing far off near the edges
  (a test point moved 0.38 m before the damping and the "no move beyond 4 x the block's RMS" rule).
  A thick NavVis area turned out to be one noisy layer, not two misaligned passes (1 vs 2
  Gaussian layers per 15 cm patch: two in 2-3% of patches, as on a good wall).
  Cost: about 75 s per million points in CPython, roughly 5 x the filter.
- **Not solved:** noise far enough off to look like relief at the block size stays.
- Ropes, hoses and anchors are long or attached pieces and mostly stay; separate them first.
  Hoses are sometimes partly removed.
- With floating-piece removal off, isolated flyers are judged by coarser planes and some near
  corners survive as "rough".

### Writing the result

- The write check compares the average of all points written with all points read back. TBC
  returns points in its own order, so averaging every n-th point gave false alarms (8.7 mm on an
  unshifted scan; re-sorting a scan moved a 1-in-8 sample's average by up to 7 mm).
- Whether `ImportPoints` rolls back with the transaction is untested: if a new region is left
  after a failed run, delete it.
- Up to 12 M points: roughly 0.5 GB of memory per million points at peak and ~25 s per 1.7 M
  points in TBC. Untested above 3.4 M.

## Separate Objects

Tested by simulating clicks on the hand-labelled SX12 channel wall (rock, anchors, and junk that
is mostly hoses), one half used for tuning and the other kept unseen.

- **Automatic classification didn't generalise.** Decision trees and a random forest on 30
  multi-scale shape features per point (line / plate / blob measures, verticality, density,
  distance off the local and a smoothed rock surface) either missed most anchors or flagged
  hundreds of thousands of rock points on the other half. Distance off a smoothed rock surface
  alone doesn't separate either: blasted rock has as much relief at 10-20 cm as a hose is thick.
  The scripts are in `tests/experiments/`.
- **Growing from a click** through points off a coarse rock surface took almost no rock but only
  ~50% of each anchor and ~6% of each hose (hoses lie on the rock).
- **Local background (what the macro does):** a plane fitted to the ring 1-2.5 x the radius
  around the click (anchors, 10 cm) or beside each tube segment (hoses, 5 cm), points more than
  8 mm off it, kept only if connected (2.5 x spacing) to the click or the clicked line.
  Tuning half: 90% of each anchor and 83% of each hose selected, median 0 rock points per
  object. Unseen half: 84% / 83%, median 14 / 0 rock points (a few anchors drag in a few hundred).
  Without the connectivity step the unseen half dragged in a median ~120 rock points per object;
  a threshold scaled to the rock's local scatter cut that but lost much of the hoses.
- Simulated clicks were ideal (object centre; every metre along a hose), so real use may be
  slightly worse. The display-to-stored coordinate conversion is untested on a project with a
  scale factor.

## Best-Fit Scans

- Same method as `bestfit_clouds.py` in the check-georef repository: point-to-plane ICP on
  planes fitted to 5-10 cm reference cells, translation only, from three starts; the report
  gives how strongly each axis is constrained.
- The fit runs in pure Python on TBC's UI thread: keep regions to the overlap (a few hundred
  thousand points). The plane-fitting stage (35-50% on the progress bar) has no updates inside.
- Moving a scan moves it in every region that uses it, so it refuses to run when the reference
  and moving regions share scans.
- Ideas: preview with `TemporarilyShift`; station-level registration
  (`ApplyIncrementalRegistration`) like TBC's own workflow; a rotation (6-DOF) solve; per-scan fits.
