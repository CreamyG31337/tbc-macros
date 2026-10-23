# TBC Macros: Point Cloud Tools

Python macros for **Trimble Business Center** (TBC) that work on point cloud regions: inspect
scans, best-fit scans onto each other, remove noise in 3D (walls and ceilings as well as ground),
and separate objects such as rock anchors and hoses from a rock face.

**Status: experimental.** Each macro writes its results as new regions or inside one undoable
transaction and checks its own work, but always try them on a copy of a project first.

| Macro | What it does |
|---|---|
| **Scan Info** (`ScanInfo.py`) | Read-only. Lists the selected point cloud regions and their scans (point counts, scan positions, adjusted or not) and writes sampled points to `ScanInfo.csv` in the TBC export folder. |
| **Best-Fit Scans** (`BestFitScans.py`) | Fits "moving" regions onto "reference" regions with a point-to-plane, translation-only fit on the surfaces both contain, then shifts the moving regions' scans. Can also apply a typed vertical shift. By default first copies the moving regions, unmoved, into hidden "- before shift" regions. Committed only if the copies read back and the points moved by the computed amount. |
| **Clean Point Cloud 3D** (`CleanCloud.py`) | Removes noise by comparing each point with a plane fitted to its neighbourhood, and removes small floating pieces that aren't on the surface. Cell size and tolerance are picked from the measured point spacing and noise. Can also smooth the kept points onto curved local surfaces. Writes "- cleaned" and "- removed" regions; refuses runs that would remove over 20%. |
| **Separate Objects** (`SeparateObjects.py`) | Click anchors / dowels and junk (one click each) or hoses / cables (clicks along them). Each object is the points standing off the rock right around it, connected to the click. Writes "- anchors", "- hoses", "- junk" and "- rest" regions. |

None of the macros change the selected regions or their scans except Best-Fit Scans, which moves
the scans behind the moving regions (in every region that uses them). Its "- before shift"
copies cover the selected moving regions only, not other regions that share those scans; they
are written in the same undo step as the shift, so Undo removes them too.

## Requirements

- TBC 5.90 or later, which runs macros with its bundled IronPython 3.4 (developed on TBC 2026.10).
- A licence that allows running your own macros (**Survey Advanced** or higher).
- Write access to a subfolder of `C:\ProgramData\Trimble\MacroCommands3\`.

## Install

TBC loads macros from `C:\ProgramData\Trimble\MacroCommands3\` and every subfolder in it. On TBC
5.90+ that folder needs administrator rights to write to, so either:

- have an administrator create `MacroCommands3\PointCloudTools` once and give your user modify
  rights on it, or
- use any existing subfolder of `MacroCommands3` that you can already write to.

Then copy the contents of `PointCloudTools\` there (or run `tools\deploy.ps1 -Target <folder>`),
restart TBC, and the commands appear on the **Macros** tab in the **Point Cloud Tools** group.
To update, close TBC, copy the changed `.py` / `.xaml` files over and restart. TBC keeps a
`<folder>.dict` cache of the registered commands next to them and rebuilds it.

## Using them

- **Scan Info:** select regions, press OK, read the report; the CSV path is listed at the end.
- **Best-Fit Scans:** capture the reference regions and the moving regions with the two buttons,
  press Compute fit, read the offset and how well each axis is constrained, then OK to shift.
  For a manual vertical nudge, capture only the moving regions, type an amount and use Set shift
  Up / Down instead. If the shift is rolled back, the report says whether the points didn't move
  or moved by the wrong amount; try the other "Apply shift with" method.
- **Clean Point Cloud 3D:** separate anything you want to keep (anchors, ropes, people) first,
  select the rock region and press Preview: the report gives the settings picked and what each
  test would remove. OK writes the "- cleaned" and "- removed" regions, hides the source and
  colours the result (cleaned in true colour, removed in red) so it can be checked at a glance.
  Tick **Smooth** to also pull the remaining fuzz onto the surface. It keeps the rock's shape
  much better than flat planes would, but it is slow (about 75 s per million points on top of
  the clean) and moves points: typically a few mm, up to several cm in very noisy areas. Check
  sharp edges after using it.
- **Separate Objects:** select the rock region, pick the object type, turn on snapping to point
  clouds and click the object (the first click reads the region). For a hose, click along it
  every metre or so, then Finish hose. Each pick reports its point count; Undo last removes it.

All panels show progress, stay responsive and can be cancelled; a cancelled or failed write is
rolled back.

## Repository layout

| Path | Contents |
|---|---|
| `PointCloudTools/` | The macros: one `.py` and one `.xaml` (the panel) each. This folder is what gets installed. |
| `docs/tbc-api-notes.md` | How TBC macros work, and the point cloud, region, view filter and transaction APIs these use. TBC's macro API is undocumented; these were read from the assemblies' metadata. |
| `docs/testing.md` | How the filters were tuned and what they did on test data (laser scanners, SLAM, photogrammetry, hand-labelled scans). |
| `tests/` | Offline tests that run the macros' own filter code in regular Python, on synthetic data or your own LAS/LAZ files. |
| `tools/dotnet_meta.py` | Lists the public types and signatures of a .NET assembly by reading its metadata (never loads it), for finding TBC APIs. |
| `tools/deploy.ps1` | Copies the macros into a TBC macro folder and checks the copies. |

## References

- TBC help, "Macro Community": <https://help.fieldsystems.trimble.com/tbc/23627.htm>
- TBC Macros and Extensions community: <https://community.trimble.com/communities/publiccommunity?CommunityKey=8a262af4-a35e-4e9a-9dd3-191cc785899a>
- TbcMacroKit, a C# + Python shim framework with Visual Studio debugging: <https://github.com/dmbrubac/TbcMacroKit>
- SCR Macros, a large public IronPython macro collection: <https://github.com/RonnySchneider/SCR_Macros_Public>

## Credits

The code here is written from scratch for TBC's IronPython (no third-party code is copied or
bundled), but several of the methods are based on other people's work:

- **Clean Point Cloud 3D, the plane test:** based on the Noise filter in
  [CloudCompare](https://www.cloudcompare.org) (`CloudSamplingTools::noiseFilter` in CCCoreLib):
  fit a plane to each point's neighbours and drop points too far from it. This version uses grid
  cells at several sizes instead of per-point neighbourhoods, and picks the tolerance from the
  measured noise.
- **Clean Point Cloud 3D, smoothing:** based on moving least-squares (MLS) smoothing as in
  the [Point Cloud Library](https://pointclouds.org)'s `MovingLeastSquares`, which CloudCompare
  offers through its PCL plugin: move each point onto a quadratic surface fitted to its
  neighbourhood. This version fits the surfaces in overlapping grid blocks and blends them,
  which is cheaper in pure Python, and uses larger blocks where the noise is high.
- **Clean Point Cloud 3D, floating pieces:** grouping points into connected pieces on a grid is
  the same idea as CloudCompare's "Label Connected Components".
- **Best-Fit Scans:** a translation-only form of point-to-plane ICP (Chen and Medioni, 1992).

Thanks to the CloudCompare and PCL developers for publishing their work as open source.

## Licence

MIT; see `LICENSE`.
