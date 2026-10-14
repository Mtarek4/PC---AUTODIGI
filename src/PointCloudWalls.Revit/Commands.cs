using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using PointCloudWalls.Core;
using PointCloudWalls.Cad;

namespace PointCloudWalls.Revit
{
    internal static class Ctx
    {
        public const double MPerFt = 0.3048;

        public static double CutZ(Document doc, ViewPlan view, Level lvl, Settings s)
        {
            if (!double.IsNaN(s.SliceHeightAboveLevelM)) return lvl.Elevation + s.SliceHeightAboveLevelM / MPerFt;
            PlanViewRange vr = view.GetViewRange();
            Level cl = doc.GetElement(vr.GetLevelId(PlanViewPlane.CutPlane)) as Level ?? lvl;
            return cl.Elevation + vr.GetOffset(PlanViewPlane.CutPlane);
        }

        public static List<CloudSource> CloudsInView(Document doc, View view) =>
            new FilteredElementCollector(doc, view.Id).OfClass(typeof(PointCloudInstance))
                .Cast<PointCloudInstance>().Select(pc => new CloudSource(pc)).ToList();

        public static string EndText(EndType t)
        {
            switch (t)
            {
                case EndType.Cap: return "end face";
                case EndType.Junction: return "meets wall";
                case EndType.Gap: return "faces stop";
                case EndType.Click: return "your click";
                default: return "may continue";
            }
        }
    }

    /// <summary>Feeds the solver with slice points from the Revit point clouds.</summary>
    internal sealed class RevitPointSource : IWallPointSource
    {
        private readonly List<CloudSource> _clouds;
        private readonly double _zMin, _zMax;
        private readonly Settings _s;

        public RevitPointSource(List<CloudSource> clouds, double zMinFt, double zMaxFt, Settings s)
        {
            _clouds = clouds; _zMin = zMinFt; _zMax = zMaxFt; _s = s;
        }

        public List<P2> Query(P2 center, P2 axis, double halfLength, double halfWidth)
        {
            const double M = Ctx.MPerFt;
            var c = new XYZ(center.X / M, center.Y / M, 0);
            var ax = new XYZ(axis.X, axis.Y, 0);
            var res = new List<P2>();
            foreach (var cs in _clouds)
                foreach (XYZ p in cs.Query(c, ax, halfLength / M, halfWidth / M, _zMin, _zMax, _s.SpacingM / M, _s.MaxPoints))
                    res.Add(new P2(p.X * M, p.Y * M));
            return res;
        }

        /// <summary>Same query, but at a DIFFERENT Z-band than this instance was built with - used only to count points
        /// when searching for the best slice height, so it stays cheap (small radius, no need for the full point set).</summary>
        public int CountAt(double zMinFt, double zMaxFt, P2 center, double radius)
        {
            const double M = Ctx.MPerFt;
            var c = new XYZ(center.X / M, center.Y / M, 0);
            var ax = new XYZ(1, 0, 0);
            int n = 0;
            foreach (var cs in _clouds)
                n += cs.Query(c, ax, radius / M, radius / M, zMinFt, zMaxFt, _s.SpacingM / M, 2000).Count;
            return n;
        }

        /// <summary>Rebuilds this source at a different Z-band (used once the best slice height has been found).</summary>
        public RevitPointSource AtHeight(double zMinFt, double zMaxFt) => new RevitPointSource(_clouds, zMinFt, zMaxFt, _s);
    }

    // =====================================================================================================
    internal enum PickMode { BothFaces, Centerline, Diagonal }

    [Transaction(TransactionMode.Manual)]
    public class PcWallCommand : IExternalCommand          // "PC Wall - both sides"
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => WallPicker.Run(data, PickMode.BothFaces);
    }

    [Transaction(TransactionMode.Manual)]
    public class PcWallCenterlineCommand : IExternalCommand // "PC Wall - centerline"
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => WallPicker.Run(data, PickMode.Centerline);
    }

    [Transaction(TransactionMode.Manual)]
    public class PcWallDiagonalCommand : IExternalCommand   // "PC Wall - diagonal"
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => WallPicker.Run(data, PickMode.Diagonal);
    }

    internal static class WallPicker
    {
        // walls created in this Revit session: the only ones the joiner is allowed to move (besides those tagged PCW)
        private static readonly HashSet<string> Session = new HashSet<string>();

        /// <summary>What every wall tool needs before it starts: a plan view with a visible point cloud, and the wall setup
        /// (level, height, wall type - asked in the popup unless that is switched off). false = cancelled, the user was told why.</summary>
        internal static bool Prepare(ExternalCommandData data, out UIDocument uidoc, out ViewPlan view, out Level lvl, out List<CloudSource> clouds, out WallSetup setup)
        {
            uidoc = data.Application.ActiveUIDocument;
            Document doc = uidoc.Document;
            view = doc.ActiveView as ViewPlan; lvl = null; clouds = null; setup = null;
            if (view == null || view.GenLevel == null)
            {
                TaskDialog.Show("PC Wall", "Open a floor plan view (with the point cloud visible) first.");
                return false;
            }
            lvl = view.GenLevel;
            clouds = Ctx.CloudsInView(doc, view);
            if (clouds.Count == 0)
            {
                TaskDialog.Show("PC Wall", "No point cloud is visible in this view.\n\nCheck Visibility/Graphics > Point Clouds and that the cloud is loaded.");
                return false;
            }

            // ---- the popup: level, height and the wall type new types are duplicated from ---------------------------------------------
            Settings s0 = Settings.Load();
            bool ask = s0.AskWallSetup;
            setup = WallSetup.FromSettings(doc, view, s0, preferView: ask);
            if (setup.BaseLevelId == null || setup.TemplateTypeId == null)
            {
                TaskDialog.Show("PC Wall", "The project needs at least one level and one basic wall type.");
                return false;
            }
            if (ask)
            {
                var win = new WallSetupWindow(doc, setup, true, true);
                if (win.ShowDialog() != true) return false;
                setup = win.Result;
                setup.CopyTo(s0); s0.AskWallSetup = win.AskEachTime; s0.Save();      // remembered for next time
            }
            return true;
        }

        public static Result Run(ExternalCommandData data, PickMode mode)
        {
            if (!Prepare(data, out UIDocument uidoc, out ViewPlan view, out Level lvl, out List<CloudSource> clouds, out WallSetup setup)) return Result.Cancelled;
            Document doc = uidoc.Document;

            string first, second;
            switch (mode)
            {
                case PickMode.BothFaces:
                    first = "Click a point ON THE FIRST FACE of the wall (Esc = finish)";
                    second = "Now click the matching point on the OPPOSITE face";
                    break;
                case PickMode.Centerline:
                    first = "Click the START of the wall on its CENTER LINE, roughly. Only ONE face scanned? Use PC Wall - diagonal instead (Esc = finish)";
                    second = "Now click the END of the wall, also on its center line";
                    break;
                default:
                    first = "DIAGONAL: click the START of the wall on ONE SIDE of it, near the wall end (Esc = finish)";
                    second = "Now click the END of the wall on the OTHER side of the wall - even if that surface was not scanned";
                    break;
            }

            string status = "";
            while (true)
            {
                XYZ a, b;
                try
                {
                    a = uidoc.Selection.PickPoint((status.Length > 0 ? status + "   |   " : "") + first);
                    b = uidoc.Selection.PickPoint(second);
                }
                catch (Autodesk.Revit.Exceptions.OperationCanceledException) { break; }

                try { status = Process(doc, view, lvl, clouds, Settings.Load(), setup, a, b, mode); }
                catch (Exception ex) { status = "ERROR: " + ex.Message; }
            }
            return Result.Succeeded;
        }

        private static string Process(Document doc, ViewPlan view, Level lvl, List<CloudSource> clouds, Settings s, WallSetup setup, XYZ a, XYZ b, PickMode mode)
        {
            const double M = Ctx.MPerFt;
            double cutZ = Ctx.CutZ(doc, view, lvl, s);
            var src = new RevitPointSource(clouds, cutZ - s.SlabHalfM / M, cutZ + s.SlabHalfM / M, s);

            var pa = new P2(a.X * M, a.Y * M); var pb = new P2(b.X * M, b.Y * M);
            WallFitResult r = mode == PickMode.BothFaces ? WallFitter.Fit(src, pa, pb, s.ToOptions())
                            : mode == PickMode.Centerline ? WallFitter.FitAlong(src, pa, pb, s.ToOptions())
                            : WallFitter.FitDiagonal(src, pa, pb, s.ToOptions());
            if (!r.Success) return "NOT FITTED: " + r.Message;

            string modeText = mode == PickMode.Centerline ? "centerline mode" : mode == PickMode.Diagonal ? "diagonal mode" : "both-sides mode";
            using (var t = new Transaction(doc, "PC wall from 2 points"))
            {
                t.Start();
                string status = Build(doc, view, lvl, s, setup, r, modeText, out Wall wall);
                if (wall == null) { t.RollBack(); return status; }
                t.Commit();
                return status;
            }
        }

        /// <summary>Creates the Revit wall for one fitted wall: wall type for the measured thickness, join/trim, quality note, check marks.
        /// Shared by the click tools and the automatic tool, so both build walls in exactly the same way. Needs an open transaction;
        /// wall = null means nothing was created (the text says why).</summary>
        internal static string Build(Document doc, ViewPlan view, Level lvl, Settings s, WallSetup setup, WallFitResult r, string modeText, out Wall wall)
        {
            const double M = Ctx.MPerFt;
            wall = null;
            Level baseLvl = doc.GetElement(setup.BaseLevelId) as Level ?? lvl;
            var template = doc.GetElement(setup.TemplateTypeId) as WallType;
            if (template == null) return "The wall type chosen in the Wall setup no longer exists - run Wall Setup again.";
            double z = baseLvl.Elevation;
            var p0 = new XYZ(r.Start.X / M, r.Start.Y / M, z);
            var p1 = new XYZ(r.End.X / M, r.End.Y / M, z);
            double zv = lvl.Elevation;                                                          // check marks are drawn in the view the tool runs in
            var v0 = new XYZ(p0.X, p0.Y, zv); var v1 = new XYZ(p1.X, p1.Y, zv);
            if (p0.DistanceTo(p1) < 0.05 / M) return "Fitted wall too short (" + (r.Length * 1000).ToString("0") + " mm) - the ends are less than 5 cm apart";

            string note, joinInfo = "";
            double finalLenM = r.Length;
            string summary;
            WallType wt = WallBuilder.GetOrCreateType(doc, template, r.Thickness / M, s.TypeSnapM / M, out note);
            wall = WallBuilder.CreateWall(doc, setup, baseLvl, wt, p0, p1);
            if (wall == null) return "Revit could not create the wall.";
            Session.Add(wall.UniqueId);

            if (s.AutoJoin)
            {
                try
                {
                    joinInfo = WallJoiner.JoinAndTrim(doc, view, baseLvl, wall, s, Session, out double lenFt);
                    if (lenFt > 0) finalLenM = lenFt * M;
                }
                catch (Exception ex) { joinInfo = "join skipped: " + ex.Message; }
            }

            string thickNote = !r.OneFace ? ""
                : r.ThicknessSource == "your clicks" ? " (from your two clicks - the other face was not scanned)"
                : r.ThicknessSource == "sparse opposite face" ? " (from a sparse opposite face)"
                : r.ThicknessSource == "same wall, measured nearby" ? " (measured on the next piece of this wall - one face here)"
                : " (ASSUMED - one face only)";
            string thick = (r.Thickness * 1000).ToString("0") + " mm" + thickNote;
            string sideNote = r.OneFace ? "body side: " + r.SideSource : "";
            string rough = r.OneFace ? (r.SpreadA * 1000).ToString("0.0") : (r.SpreadA * 1000).ToString("0.0") + "/" + (r.SpreadB * 1000).ToString("0.0");
            summary = string.Format("{0} x {1:0.00} m @ {2:0.00} deg | surface roughness {3} mm | coverage {4:0}% | ends: {5} / {6} | {7}",
                thick, finalLenM, r.AngleDeg, rough, r.Coverage * 100,
                Ctx.EndText(r.AtStart), Ctx.EndText(r.AtEnd), r.Grade.ToUpperInvariant());
            if (s.QualityNote)
            {
                Parameter cm = wall.get_Parameter(BuiltInParameter.ALL_MODEL_INSTANCE_COMMENTS);
                if (cm != null && !cm.IsReadOnly)
                    cm.Set("PCW " + summary + " | " + (r.OneFace && r.ThicknessSource == "your clicks" ? "from clicks " : r.OneFace && r.ThicknessAssumed ? "assumed " : "measured ") + (r.Thickness * 1000).ToString("0.0") + " mm, angle +/- " + r.AngleSigmaDeg.ToString("0.000") + " deg"
                           + " | " + modeText
                           + (sideNote.Length > 0 ? " | " + sideNote : "") + (joinInfo.Length > 0 ? " | " + joinInfo : ""));
            }

            if (s.DebugLines)
            {
                XYZ nrm = new XYZ(r.Normal.X, r.Normal.Y, 0);
                XYZ off = nrm * (r.Thickness / M / 2);
                XYZ tick = nrm * (r.Thickness / M / 2 + 0.10 / M);
                var red = new Color(255, 0, 0);
                WallBuilder.Detail(doc, view, v0 - off, v1 - off, red, 3);                                        // fitted face A (real)
                WallBuilder.Detail(doc, view, v0 + off, v1 + off, r.OneFace ? new Color(255, 140, 0) : red, 3);  // face B (orange = ASSUMED)
                WallBuilder.Detail(doc, view, v0, v1, new Color(0, 160, 0), 1);                                   // centerline (as scanned)
                var blue = new Color(0, 90, 255);
                WallBuilder.Detail(doc, view, v0 - tick, v0 + tick, blue, 3);     // where the wall STARTS
                WallBuilder.Detail(doc, view, v1 - tick, v1 + tick, blue, 3);     // where the wall ENDS
            }
            return "[" + r.Grade.ToUpperInvariant() + "] " + summary.Substring(0, summary.LastIndexOf('|')).Trim()
                   + (r.OneFace ? " | body side: " + r.SideSource : "") + "  (" + note + ")"
                   + (joinInfo.Length > 0 ? "  [" + joinInfo + "]" : "");
        }
    }

    // =====================================================================================================
    /// <summary>Change level / height / wall type without starting a wall tool.</summary>
    [Transaction(TransactionMode.ReadOnly)]
    public class PcWallSetupCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            Document doc = data.Application.ActiveUIDocument.Document;
            Settings s = Settings.Load();
            WallSetup setup = WallSetup.FromSettings(doc, doc.ActiveView as ViewPlan, s, preferView: false);
            if (setup.BaseLevelId == null || setup.TemplateTypeId == null)
            {
                TaskDialog.Show("Wall setup", "The project needs at least one level and one basic wall type.");
                return Result.Cancelled;
            }
            var win = new WallSetupWindow(doc, setup, s.AskWallSetup, false);
            if (win.ShowDialog() != true) return Result.Cancelled;
            win.Result.CopyTo(s);
            s.AskWallSetup = win.AskEachTime;
            s.Save();
            return Result.Succeeded;
        }
    }

    // =====================================================================================================
    /// <summary>Change level / height / column type without starting a column tool.</summary>
    [Transaction(TransactionMode.ReadOnly)]
    public class PcColumnSetupCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            Document doc = data.Application.ActiveUIDocument.Document;
            Settings s = Settings.Load();
            WallSetup setup = WallSetup.FromSettings(doc, doc.ActiveView as ViewPlan, s, preferView: false, kind: SetupKind.Column);
            if (setup.BaseLevelId == null || setup.TemplateTypeId == null)
            {
                TaskDialog.Show("Column setup", "The project needs at least one level and one loaded structural column family.");
                return Result.Cancelled;
            }
            var win = new WallSetupWindow(doc, setup, s.AskColumnSetup, false);
            if (win.ShowDialog() != true) return Result.Cancelled;
            win.Result.CopyTo(s);
            s.AskColumnSetup = win.AskEachTime;
            s.Save();
            return Result.Succeeded;
        }
    }

    // =====================================================================================================
    /// <summary>Sends the walls to the AutoCAD session that is listening (PointCloudWalls CAD bridge).</summary>
    [Transaction(TransactionMode.ReadOnly)]
    public class PcPublishCadCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            UIDocument uidoc = data.Application.ActiveUIDocument;
            Document doc = uidoc.Document;
            View view = doc.ActiveView;
            if (!(view is ViewPlan))
            {
                TaskDialog.Show("Publish to CAD", "Open a floor plan view first. Walls and columns are published for the active view, or for what you have selected.");
                return Result.Cancelled;
            }

            Settings s = Settings.Load();
            string pipe = CadPublish.PipeName(s);
            string drawing = CadClient.Ping(pipe);
            if (drawing == null)
            {
                TaskDialog.Show("Publish to CAD",
                    "AutoCAD is not listening.\n\n" +
                    "1. Start AutoCAD 2024 - the PointCloudWalls CAD bridge loads with it.\n" +
                    "   (or type PCWLISTEN in AutoCAD)\n" +
                    "2. Open the drawing you want the walls in.\n" +
                    "3. Click PC Publish to CAD again.\n\n" +
                    "If AutoCAD is running: type PCWSTATUS in AutoCAD to see whether it is listening.");
                return Result.Cancelled;
            }
            if (drawing.Length == 0)
            {
                TaskDialog.Show("Publish to CAD", "AutoCAD is running but no drawing is open. Open or create a drawing, then publish again.");
                return Result.Cancelled;
            }

            string what;
            PublishBatch batch = CadPublish.Build(doc, view, uidoc.Selection.GetElementIds(), s, out what);
            if (batch.Elements.Count == 0)
            {
                TaskDialog.Show("Publish to CAD", "No walls or columns to publish in this view.");
                return Result.Cancelled;
            }

            PublishResult r;
            try { r = CadClient.Publish(pipe, batch, 5000); }
            catch (Exception ex)
            {
                TaskDialog.Show("Publish to CAD", "Could not publish: " + ex.Message);
                return Result.Failed;
            }

            var td = new TaskDialog("Publish to CAD");
            td.MainContent = r.Ok
                ? "Published " + batch.Elements.Count + " " + what + " to '" + r.Drawing + "'  (" + CadPublish.UnitsName(s.CadUnitsIndex) + ", " +
                  (s.CadCoordinatesIndex == 1 ? "shared coordinates" : "Revit origin") + ")\n\n" +
                  "  new: " + r.Created + "     updated: " + r.Updated + "     unchanged: " + r.Unchanged + "     removed: " + r.Deleted +
                  (r.Errors > 0 ? "\n\n" + r.Errors + " element(s) failed: " + r.Message : "")
                : "AutoCAD reported a problem:\n\n" + r.Message;
            td.Show();
            return r.Ok ? Result.Succeeded : Result.Failed;
        }
    }

    // =====================================================================================================
    [Transaction(TransactionMode.ReadOnly)]
    public class PcSettingsCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            var win = new SettingsWindow(Settings.Load());
            win.ShowDialog();
            return Result.Succeeded;
        }
    }

    // =====================================================================================================
    [Transaction(TransactionMode.ReadOnly)]
    public class PcDiagCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            UIDocument uidoc = data.Application.ActiveUIDocument;
            Document doc = uidoc.Document;
            var view = doc.ActiveView as ViewPlan;
            if (view == null || view.GenLevel == null)
            {
                TaskDialog.Show("PC Diagnostics", "Open a floor plan view first.");
                return Result.Cancelled;
            }
            Settings s = Settings.Load();
            Level lvl = view.GenLevel;
            double cutZ = Ctx.CutZ(doc, view, lvl, s);
            const double M = Ctx.MPerFt;

            XYZ pick;
            try { pick = uidoc.Selection.PickPoint("Click anywhere ON a wall in the point cloud"); }
            catch (Autodesk.Revit.Exceptions.OperationCanceledException) { return Result.Cancelled; }

            var sb = new StringBuilder();
            sb.AppendLine("View: " + view.Name + "   Level: " + lvl.Name);
            sb.AppendLine("Slice: cut Z = " + (cutZ * M).ToString("0.000") + " m  +/- " + s.SlabHalfM.ToString("0.00") + " m");
            sb.AppendLine("Settings file: " + Settings.FilePath);
            sb.AppendLine();

            var all = new FilteredElementCollector(doc).OfClass(typeof(PointCloudInstance)).Cast<PointCloudInstance>().ToList();
            var visible = new HashSet<ElementId>(new FilteredElementCollector(doc, view.Id).OfClass(typeof(PointCloudInstance)).ToElementIds());
            sb.AppendLine("Point clouds in project: " + all.Count + "   visible in this view: " + visible.Count);

            double r1 = 1.0 / M, sl = s.SlabHalfM / M;
            var min = new XYZ(pick.X - r1, pick.Y - r1, cutZ - sl);
            var max = new XYZ(pick.X + r1, pick.Y + r1, cutZ + sl);
            foreach (var pc in all)
            {
                var src = new CloudSource(pc);
                sb.AppendLine();
                sb.AppendLine("- " + pc.Name + (visible.Contains(pc.Id) ? "" : "   [NOT visible in this view]"));
                XYZ o = src.ToRevitTransform.Origin;
                sb.AppendLine("  transform: " + (src.IsIdentity ? "identity" : "origin (" + (o.X * M).ToString("0.00") + ", " + (o.Y * M).ToString("0.00") + ", " + (o.Z * M).ToString("0.00") + ") m, non-identity"));
                BoundingBoxXYZ bb = pc.get_BoundingBox(null);
                if (bb != null)
                    sb.AppendLine("  bbox (m): (" + (bb.Min.X * M).ToString("0.0") + ", " + (bb.Min.Y * M).ToString("0.0") + ", " + (bb.Min.Z * M).ToString("0.0") + ") .. (" +
                                  (bb.Max.X * M).ToString("0.0") + ", " + (bb.Max.Y * M).ToString("0.0") + ", " + (bb.Max.Z * M).ToString("0.0") + ")");
                try
                {
                    List<XYZ> pts = src.QueryBox(min, max, s.SpacingM / M, s.MaxPoints);
                    sb.AppendLine("  points in 2 m x 2 m slice around click: " + pts.Count);
                    sb.AppendLine("  coordinate mode: " + src.ModeText);
                    if (pts.Count > 0)
                        sb.AppendLine("  Z range (m): " + (pts.Min(p => p.Z) * M).ToString("0.000") + " .. " + (pts.Max(p => p.Z) * M).ToString("0.000"));
                    else
                        sb.AppendLine("  -> 0 points: is the click on the cloud? is the cloud loaded (not unloaded/missing file)? is the cut plane through the walls?");
                }
                catch (Exception ex) { sb.AppendLine("  GetPoints failed: " + ex.Message); }
            }

            var td = new TaskDialog("PC Diagnostics") { MainContent = sb.ToString() };
            td.Show();
            return Result.Succeeded;
        }
    }
}
