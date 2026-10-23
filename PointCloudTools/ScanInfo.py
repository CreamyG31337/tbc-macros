"""
Point Cloud Tools - Scan Info (read-only).

Lists the selected point cloud regions and their scans (point counts, scan positions,
adjustment state) and writes a sampled CSV of each region's points to the TBC export
folder. Makes no changes to the project: run it first to confirm the macro API behaves
as expected on your TBC version.
"""
import clr
clr.AddReference("IronPython.Wpf")
import wpf

from System import Action, Int32
from System.IO import StreamReader, Path
from System.Windows.Controls import StackPanel

clr.AddReference("Trimble.Vce.UI.Controls")
try:
    clr.AddReference("Trimble.Sdk.DiskIO")
except Exception:
    clr.AddReference("Trimble.DiskIO")
from Trimble.DiskIO import SearchPath
try:
    clr.AddReference("Trimble.Sdk.Scanning")
except Exception:
    pass  # already loaded by TBC under another name
from Trimble.Vce.Data.Scanning import PointCloudRegion

MAX_EXPORT_POINTS = 200000


def Setup(cmdData, macroFileFolder):
    cmdData.Key = "PointCloudTools_ScanInfo"
    cmdData.CommandName = "PointCloudTools_ScanInfo"
    cmdData.Caption = "_Scan Info"
    cmdData.UIForm = "ScanInfo"
    try:
        cmdData.DefaultTabKey = "Macros"
        cmdData.DefaultTabGroupKey = "Point Cloud Tools"
        cmdData.ShortCaption = "Scan Info"
        cmdData.ToolTipTitle = "Scan Info"
        cmdData.ToolTipTextFormatted = "Lists selected point cloud regions and their scans and exports sampled points to CSV. Makes no changes."
        cmdData.Version = 0.2
        cmdData.MacroAuthor = "tbc-macros"
    except Exception:
        pass


def region_name(region):
    return getattr(region, "Name", None) or region.GetType().Name


class ScanInfo(StackPanel):  # TBC hosts this panel because cmdData.UIForm names this class
    def __init__(self, currentProject, macroFileFolder):
        with StreamReader(Path.Combine(macroFileFolder, "ScanInfo.xaml")) as s:
            wpf.LoadComponent(self, s)
        self.currentProject = currentProject

    def OnLoad(self, cmd, buttons, event):
        self.Caption = cmd.Command.Caption
        buttons[1].Content = "Close"  # TBC's Cancel button
        self.regionType = clr.GetClrType(PointCloudRegion)
        self.regions.IsEntityValidCallback = self.IsValid

    def IsValid(self, serial):
        return isinstance(self.currentProject.Concordance.Lookup(serial), self.regionType)

    def CancelClicked(self, cmd, args):
        cmd.CloseUICommand()

    def OkClicked(self, cmd, args):
        regions = [o for o in self.regions.SelectedMembers(self.currentProject) if isinstance(o, self.regionType)]
        if not regions:
            self.regions.StatusMessage = "Select one or more point cloud regions"
            return False
        noop = Action[Int32](lambda pct: None)
        csv_path = Path.Combine(SearchPath.ExportAppDataPath, "ScanInfo.csv")
        lines = []
        try:
            with open(csv_path, "w") as f:
                f.write("region,x,y,z\n")
                for region in regions:
                    integration = region.Integration
                    count = int(integration.PointCount)
                    name = region_name(region)
                    lines.append(f"Region '{name}' ({region.GetType().Name}): {count:,} points")
                    for scan in region.Scans:
                        si = scan.Integration
                        pos = si.Position
                        station = scan.PointCloudStation.Name if scan.PointCloudStation else "?"
                        lines.append(f"    scan {scan.Name}  station {station}  points {int(si.PointCount):,}  "
                                     f"position E {pos.X:.3f} N {pos.Y:.3f} Z {pos.Z:.3f}  adjusted {scan.IsAdjusted}")
                    stride = max(1, count // MAX_EXPORT_POINTS)
                    written = 0
                    for i, p in enumerate(integration.GetPoints(noop)):
                        if i % stride == 0:
                            f.write(f"{name},{p.X:.4f},{p.Y:.4f},{p.Z:.4f}\n")
                            written += 1
                    lines.append(f"    exported {written:,} sampled points (every {stride})")
            lines.append(f"CSV: {csv_path}")
        except Exception as ex:
            lines.append(f"ERROR: {ex}")
        self.resultText.Text = "\n".join(lines)
        return False  # keep the panel open so the results stay visible
