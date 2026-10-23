# TBC macro API notes

TBC's macro API is undocumented. These notes were read from TBC 2026.10's assembly metadata
(with `tools/dotnet_meta.py`) and TBC's own sample macros, then confirmed by the macros in this
repository where noted. Names can change between TBC versions.

## How TBC macros work

TBC 5.90+ runs Python macros with its bundled **IronPython 3.4.2**: f-strings work, but newer
modules like `tomllib` don't, and dicts don't keep insertion order (iterate over a fixed list of
keys when order matters). Running your own macros needs a **Survey Advanced** licence or higher.

- **Location.** TBC loads macros from `C:\ProgramData\Trimble\MacroCommands3\` and its
  subfolders, recursively.
  - Each folder gets a `<folder>.dict` cache of the registered commands (name, caption, icons,
    ribbon tab, script path and modify time).
  - TBC's own samples are readable `.py` files in `MacroCommands3\Trimble\`.
  - Commercial packs ship compiled IronPython (`.pydll`).
  - Each macro file is loaded on its own; don't rely on importing a sibling macro. Shared helpers
    are copied into each file here.
- **Installing.** On TBC 5.90+ `MacroCommands3` is write-protected for normal users: an
  administrator can create a subfolder and grant modify rights once, and any subfolder you can
  already write to also works. TBC also stores the macro folder as the per-user option
  `Environment.FileLocations.Macro3CommandsFiles` (a base64-encoded JSON string in
  `%APPDATA%\Trimble\Trimble Business Center\<version>\User.options`); it isn't on the Options
  dialog and changing it is untested.
- **Registration.** A macro file defines `Setup(cmdData, macroFileFolder)`, which sets `Key`,
  `CommandName`, `Caption`, `UIForm`, `DefaultTabKey`, `DefaultTabGroupKey`, `ShortCaption`,
  `ToolTipTitle`, `ToolTipTextFormatted`, `ImageSmall` (a `System.Drawing.Bitmap`),
  `HelpFile`/`HelpTopic`, `Version` and `MacroAuthor`. Keys are global across all installed
  macros, so these use `PointCloudTools_<name>`.
- **UI, option 1: a command pane.** Set `cmdData.UIForm = "ClassName"` and define a class of that
  name that inherits `System.Windows.Controls.StackPanel`:
  - `__init__(self, currentProject, macroFileFolder)` loads the XAML with `wpf.LoadComponent`.
    Named XAML elements become attributes, and `Click="Method"` handlers bind to methods.
  - `OnLoad(self, cmd, buttons, event)` runs when the pane opens. `buttons` are the pane's own
    buttons: `buttons[0]` OK, `buttons[1]` Cancel, `buttons[2]` an extra hidden one; set
    `.Content` to rename them and `.Visibility` to show the third.
  - `OkClicked(self, cmd, e)` runs on OK. Returning `False` keeps the pane open.
  - `CancelClicked(self, cmd, e)` should call `cmd.CloseUICommand()`.
- **UI, option 2: a command without a pane.** Define `Execute(cmd, currentProject, macroFileFolder, parameters)`.
- **TBC controls in XAML.** Use `xmlns:Wpf="clr-namespace:Trimble.Vce.UI.Controls.Wpf;assembly=Trimble.Vce.UI.Controls"`.
  - `MemberSelection`: `SelectedMembers(project)`, `IsEntityValidCallback`, `StatusMessage`.
  - `CoordinateEdit`: picks a point in the view (with snapping). `.Coordinate` is a `Point3D` in
    display (grid) coordinates; `ValueChanged += handler(ctrl, e)`, where `e.Cause` is an
    `InputMethod` (`Trimble.Sdk.UI`): `Mouse` or `Snap` for a pick. Declared as
    `<Wpf:CoordinateEdit x:Name="coordCtl" AutoTab="False"/>`.
  - `DistanceEdit` (`.Distance`, metres), `ComboBoxEntityPicker` (`FilterByEntityTypes`,
    `SelectedSerial`), `SelectEntity`, `LayerPicker`.
- **Long work and progress.** Macros run on TBC's UI thread, so WPF doesn't repaint while they
  work. Calling `self.Dispatcher.Invoke(Action(lambda: None), DispatcherPriority.Background)`
  every ~0.1 s lets the progress bar move and Cancel be clicked; guard against re-entry with a
  busy flag. TBC may call progress callbacks (`Action<int>`) from another thread: check
  `self.Dispatcher.CheckAccess()` before touching the UI.
- **Editing the project.** Wrap every change like this:
  ```python
  project.TransactionManager.AddBeginMark(CommandGranularity.Command, caption)
  with TransactMethodCall(project.TransactionCollector) as failGuard:
      ...                    # changes
      failGuard.Commit()     # without this, everything in the block is rolled back
  project.TransactionManager.AddEndMark(CommandGranularity.Command)
  ```
  An exception (for example a "cancelled" exception raised from a progress update) leaves the
  block without `Commit()`, so it rolls back.

## Point clouds

- **Regions:** `Trimble.Vce.Data.Scanning.PointCloudRegion` (named regions derive from
  `ExposedPointCloudRegionBase`, which has a settable `.Name`).
  - `.Scans` returns `IEnumerable<IPointCloudScan>`; `.Stations` returns the stations.
  - `.Integration` returns `IPointCloudRegionIntegration`:
    - `PointCount`, `BoundingBox` (`Rectangle3D`: `Origin`, `SizeX`, `SizeY`, `SizeZ`)
    - `GetPoints(Action<int> progress)` → `IEnumerable<Point3D>` (project grid, metres)
    - `GetPointInfos(Action<int>)` → `PointInfo`, with `Coordinates`, `Color`, `Luminance`, `Normal` and `ScanId`
    - `TemporarilyShift(Vector3D)` → `IDisposable` (preview a shift without saving it)
  - Points come back in TBC's own order: compare whole-set statistics, not every n-th point.
- **Scans:** `IPointCloudScan`.
  - `Transform(Matrix4D)`, `IncrementalTransform(IFrame)`, `ApplyTransformation(Vector3D position, Spinor3D rotation)`.
    Best-Fit Scans offers the first and last, and its verification shows which one works.
  - `AdjustedPosition`, `IsAdjusted`, `PointCloudStation`.
  - `.Integration` returns `IPointCloudScanIntegration`, with `Position`, `Frame` (`IFrame`: `Position`, `RotationMatrix`), `PointCount` and `GetPointInfos`.
  - Moving a scan moves its points in every region that uses it.
- **Stations:** `IPointCloudStation`.
  - `ForceRegistration(IFrame, CoordQuality)` and `ApplyIncrementalRegistration(IFrame, ref bool, CoordQuality)`, which is how TBC's own registration moves stations.
- **Database:** `PointCloudDatabaseContainer.GetInstance(project)`.
  - `.PrimaryDatabase.Integration` exposes `GetScanIds(cloudId)`, `GetPosition(scanId)`,
    `SetPosition(scanId, Point3D)`, `GetPoints(cloudId, ...)`, `CreateSpatiallySampledCloud(...)`,
    `ExportLas(...)`, and importers for LAS, E57, PTS, PTX, PLY and others.
  - `.PrimaryDatabase.Add()` creates a station; `MakeUniqueStationName(name)`.
  - `.PrimaryDatabase.Integration.ImportPoints(IEnumerable<PointInfo>, station, region, bool applyScaleFactor, Action<int>)`
    writes points as a new scan in `region` (used by Clean Point Cloud 3D and Separate Objects).
    `ImportPts(file, station, region, unit, progress)` does the same from a `.pts` file.
  - `PointInfo(Color, Point3D, double luminance, Vector3D normal, Guid scanId)` (`Trimble.Vce.Interfaces.PointCloudIntegration`).
  - **Scale factor.** Scans are stored without the project's ground scale factor, and
    `GetPoints` / `GetPointInfos` apply it. Points read normally and written back are scaled
    twice: on a real project that put the new scan ~62 m off in Northing (and
    `applyScaleFactor=True` didn't help). Read and write inside
    `PointCloudDatabaseContainer.GetInstance(project).TemporarilySetGroundScaleFactorToOne()`.
    Coordinates picked in the view are in display coordinates: Separate Objects converts them
    with the ratio of a region's bounding-box centres read with and without the scale factor.
- **New regions:** `ExposedPointCloudRegionContainer.GetInstance(project).Add()`; `MakeUniqueRegionName(name)`.
- **Region display:** `region.Color`, `region.ColorMode` (`Trimble.Vce.Interfaces.PointCloudIntegration.ColorMode`:
  `View`, `Model`, `Scan`, `PointLuminance`, `ColorCodedIntensity`, `PointRGB`, `Height`, ...). By the order of TBC's
  list, `Model` is region colour and `PointRGB` true colour (unconfirmed).
- **Classification:** `Trimble.Vce.Scanning.Classification.IGenericClassifier.Classify(regions, factory, mergeRegionsPerClass)`
  with `CreateClassificationFactoryWithAllClassesEnabled(eAlgoClassificationType)`: `eOutdoor`, `eIndoor`, `eTunnel`,
  `eOutdoorDLBased`. Classes: outdoor (ground, building, poles / signs, vegetation, power lines, steps, noise,
  dividers), tunnel (ground, envelope), indoor (floor, wall, duckboard, ceiling). Not used here.

## View filters

- Each point cloud region has its own imitation type, which is what gives it a checkbox in the
  View Filter Manager:
  `project.GetMemberContainer(MandatoryContainers.ImitationTypeCollection).Lookup(region.ImitationTypeGuid).SerialNumber`.
- Visibility per filter: `project.GetMemberContainer(MandatoryContainers.ViewFilterCollection)`;
  for each filter that isn't `IsReadOnly`, `filter.ImitationTypeOverrides.Add(serial).Visible = False`
  (or `overrides[serial]` if `overrides.Contains(serial)`). Which filter a view uses isn't
  exposed, so the macros change every writable filter.

## Geometry (`Trimble.Vce.Geometry`)

- `Point3D(x, y, z)` and `Vector3D`.
- `Matrix4D(Vector3D shift)` is a translation matrix; `Matrix4D.TransformPoint`.
- `Spinor3D(double[,] rotationMatrix)`.

## Finding more

`tools/dotnet_meta.py` lists public types and typed signatures of any .NET assembly by reading
its metadata from the file. It never loads or runs the assembly:

```
python tools/dotnet_meta.py "C:\Program Files\Trimble\Trimble Business Center\Trimble.Sdk.Scanning.dll" --type PointCloudScan
python tools/dotnet_meta.py "C:\Program Files\Trimble\Trimble Business Center\Trimble.Vce.Scanning.dll" --match "Import|Export"
```

Most scanning types are in `Trimble.Sdk.Scanning.dll`, `Trimble.Vce.Scanning.dll` and
`Trimble.Sdk.Interfaces.dll`; geometry in `Trimble.Sdk.Geometry.dll`; core and view filters in
`Trimble.Sdk.Core.dll`; WPF controls in `Trimble.Vce.UI.Controls.dll`.
