using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using PointCloudWalls.Core;

namespace PointCloudWalls.Revit
{
    internal enum ColumnMode { Center, Diagonal, Sides }

    [Transaction(TransactionMode.Manual)]
    public class PcColumnCenterCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => ColumnPicker.Run(data, ColumnMode.Center);
    }

    [Transaction(TransactionMode.Manual)]
    public class PcColumnDiagonalCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => ColumnPicker.Run(data, ColumnMode.Diagonal);
    }

    [Transaction(TransactionMode.Manual)]
    public class PcColumnSidesCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => ColumnPicker.Run(data, ColumnMode.Sides);
    }

    internal static class ColumnPicker
    {
        public static Result Run(ExternalCommandData data, ColumnMode mode)
        {
            UIDocument uidoc = data.Application.ActiveUIDocument;
            Document doc = uidoc.Document;
            var view = doc.ActiveView as ViewPlan;
            if (view == null || view.GenLevel == null)
            {
                TaskDialog.Show("PC Column", "Open a floor plan view (with the point cloud visible) first.");
                return Result.Cancelled;
            }
            Level lvl = view.GenLevel;
            var clouds = Ctx.CloudsInView(doc, view);
            if (clouds.Count == 0)
            {
                TaskDialog.Show("PC Column", "No point cloud is visible in this view.\n\nCheck Visibility/Graphics > Point Clouds and that the cloud is loaded.");
                return Result.Cancelled;
            }

            Settings s0 = Settings.Load();
            bool ask = s0.AskColumnSetup;
            WallSetup setup = WallSetup.FromSettings(doc, view, s0, preferView: ask, kind: SetupKind.Column);
            if (setup.BaseLevelId == null || setup.TemplateTypeId == null)
            {
                TaskDialog.Show("PC Column", "The project needs at least one level and one loaded STRUCTURAL column family " +
                    "(with a Width & Depth parameter for rectangular, or a Diameter parameter for round - e.g. Revit's Concrete-Rectangular-Column / Concrete-Round-Column). " +
                    "Architectural column families (Insert > Load Family, Structural Columns folder) are not listed here.");
                return Result.Cancelled;
            }
            if (ask)
            {
                var win = new WallSetupWindow(doc, setup, true, true);
                if (win.ShowDialog() != true) return Result.Cancelled;
                setup = win.Result;
                setup.CopyTo(s0); s0.AskColumnSetup = win.AskEachTime; s0.Save();
            }

            string first, second;
            switch (mode)
            {
                case ColumnMode.Center:
                    first = "Click near the CENTER of the column (Esc = finish)"; second = null; break;
                case ColumnMode.Diagonal:
                    first = "Click one CORNER of the column (Esc = finish)"; second = "Now click the OPPOSITE corner"; break;
                default:
                    first = "Click a point on ONE FACE of the column (Esc = finish)"; second = "Now click the matching point on the OPPOSITE face"; break;
            }

            string status = "";
            while (true)
            {
                XYZ a, b = null;
                try
                {
                    a = uidoc.Selection.PickPoint((status.Length > 0 ? status + "   |   " : "") + first);
                    if (second != null) b = uidoc.Selection.PickPoint(second);
                }
                catch (Autodesk.Revit.Exceptions.OperationCanceledException) { break; }

                try { status = Process(doc, view, lvl, clouds, Settings.Load(), setup, a, b, mode); }
                catch (Exception ex) { status = "ERROR: " + ex.Message; }
            }
            return Result.Succeeded;
        }

        private static string Process(Document doc, ViewPlan view, Level lvl, List<CloudSource> clouds, Settings s, WallSetup setup, XYZ a, XYZ b, ColumnMode mode)
        {
            const double M = Ctx.MPerFt;
            double cutZ = Ctx.CutZ(doc, view, lvl, s);
            var src = new RevitPointSource(clouds, cutZ - s.SlabHalfM / M, cutZ + s.SlabHalfM / M, s);
            // A separate source, deliberately NEVER narrowed to whatever single height ends up best for the column
            // itself (below) - it spans the whole band that auto-height search ever looks at. A wall's own direction
            // has nothing to do with which slice shows the column most clearly, and confining the diagonal-mode wall
            // search to that narrowed slice was hiding wall lines that were perfectly visible a few cm away, which is
            // why "no usable wall direction found nearby" kept showing up even with a wall plainly visible in the view.
            double wideHalfM = (s.ColAutoHeight ? s.ColHeightSearchM : 0) + s.SlabHalfM;
            var wideSrc = new RevitPointSource(clouds, cutZ - wideHalfM / M, cutZ + wideHalfM / M, s);

            string heightNote = "";
            if (s.ColAutoHeight)
            {
                var probeAt = new P2(a.X * M, a.Y * M);           // the FIRST click is always where you meant to point
                double probeRadius = Math.Max(0.3, s.ColSearchRadiusM * 0.4);

                // Raw point count at a single height picks out whatever is DENSEST there, even if that is a beam, bracket
                // or capital crossing the column at just that one elevation - not the column's own, consistent surface.
                // A real column footprint holds roughly steady over a run of heights; a beam/bracket does not. So each
                // height is scored by the WORST (not the best) of itself and its two immediate neighbours - a height only
                // wins if good data is SUSTAINED around it, which a narrow one-elevation feature cannot fake.
                var dzList = new List<double>();
                for (double dz = -s.ColHeightSearchM; dz <= s.ColHeightSearchM + 1e-9; dz += s.ColHeightStepM) dzList.Add(dz);
                var counts = new int[dzList.Count];
                for (int i = 0; i < dzList.Count; i++)
                {
                    double z = cutZ + dzList[i] / M;
                    counts[i] = src.CountAt(z - s.SlabHalfM / M, z + s.SlabHalfM / M, probeAt, probeRadius);
                }
                int origIdx = 0; for (int i = 0; i < dzList.Count; i++) if (Math.Abs(dzList[i]) < Math.Abs(dzList[origIdx])) origIdx = i;
                int bestIdx = HeightSelector.SelectSustained(counts, origIdx);

                if (bestIdx != origIdx)
                {
                    double bestZ = cutZ + dzList[bestIdx] / M;
                    src = src.AtHeight(bestZ - s.SlabHalfM / M, bestZ + s.SlabHalfM / M);
                    heightNote = " | slice height auto-adjusted " + (dzList[bestIdx] * 1000).ToString("+0;-0") + " mm (more sustained data there)";
                }
            }
            var o = s.ToColumnOptions();
            // Diagonal mode's own gesture - click two OPPOSITE CORNERS - only makes sense for a rectangle; a round
            // column has no corners to click, and is placed with Center-click mode instead. Letting Round stay a
            // candidate here bought nothing but repeated, hard-to-fully-close misclassifications against wall-corner
            // data (RANSAC occasionally stitching points from two straight walls into a spuriously tight "circle") -
            // no amount of extra heuristics fully closed that gap without also costing real round-column accuracy
            // elsewhere. Ruling Round out for THIS tool, rather than continuing to out-tune the circle fit against
            // it, removes the ambiguity at its source. Left alone only if the person has explicitly set PC Settings
            // to round-only (ColShapeMode == 2) - an unusual choice, but their own.
            if (mode == ColumnMode.Diagonal && s.ColShapeMode != 2) o.AllowRound = false;

            var pa = new P2(a.X * M, a.Y * M);
            string wallHintNote = "";
            ColumnFitResult r;
            if (mode == ColumnMode.Diagonal)
            {
                // A column embedded at a wall corner (very common in real construction) often has 2-3 of its faces
                // flush with the walls it sits in, leaving no distinguishing scan signature there at all - no amount of
                // point-cloud analysis can find a face that isn't really there. When a wall is found near the click, its
                // own direction (something Revit knows and the scan alone cannot) is offered as a fallback: if ordinary
                // detection genuinely measures the column, that measurement is still used untouched; the hint only
                // takes over when detection is not confident, building the column from the two clicks and the wall's
                // real direction instead of guessing or giving up.
                P2 pb = new P2(b.X * M, b.Y * M);
                double? angleHint = NearbyWallAngle(doc, view, pa, pb);
                // wideSrc (not the possibly height-narrowed src) is used for finding the wall direction and for
                // snapping the fallback rectangle's edges - see its declaration above for why that separation matters.
                r = ColumnFitter.FitDiagonal(src, pa, pb, o, angleHint, wideSrc);
                // The two clicks always place a column now; the only question left is where its ORIENTATION came from.
                // r.OrientationSource says exactly that, set by the core fitter itself rather than inferred here from
                // other fields (an earlier version inferred "did the fallback run" from Coverage/Spread happening to
                // both be near zero, which quietly broke once Coverage was given a real meaning on that path).
                if (r.Success && r.OrientationSource != "measured")
                {
                    string sideNote = r.Coverage >= 0.999
                        ? " (all 4 sides snapped to nearby scan data)"
                        : $" ({(int)Math.Round(r.Coverage * 4)} of 4 sides snapped to nearby scan data, the rest kept at click precision)";
                    switch (r.OrientationSource)
                    {
                        case "wall-hint":
                            wallHintNote = " | built from the clicks and the nearby wall's direction (scan alone could not separate it from the wall)" + sideNote;
                            break;
                        case "point-cloud":
                            wallHintNote = " | built from the clicks and a wall direction found directly in the point cloud (no wall needed to be modeled first)" + sideNote;
                            break;
                        case "unoriented":
                            wallHintNote = " | built from the clicks alone - no wall direction was found nearby (from either a modeled wall or the point cloud), so this column is placed at world-axis rotation and NOT yet oriented from the scan; check/rotate it by hand, or click nearer to where the wall's own lines are visible in the scan" + sideNote;
                            break;
                    }
                }
            }
            else
            {
                r = mode == ColumnMode.Center ? ColumnFitter.FitCenter(src, pa, o) : ColumnFitter.FitSides(src, pa, new P2(b.X * M, b.Y * M), o);
            }
            if (!r.Success) return "NOT FITTED: " + r.Message;

            using (var t = new Transaction(doc, "PC column"))
            {
                t.Start();
                string status = Build(doc, lvl, s, setup, r, mode.ToString().ToLowerInvariant() + " mode", mode == ColumnMode.Center, heightNote + wallHintNote, out FamilyInstance fi);
                if (fi == null) { t.RollBack(); return status; }
                t.Commit();
                return status;
            }
        }

        /// <summary>Creates the Revit column for one fitted column: the family type for the measured size, the instance, the quality note.
        /// Shared by the click tools and the automatic tool. Needs an open transaction; fi = null means nothing was created (the text says why).</summary>
        internal static string Build(Document doc, Level lvl, Settings s, WallSetup setup, ColumnFitResult r, string modeText, bool maySwitchFamily, string moreNotes, out FamilyInstance fi)
        {
            const double M = Ctx.MPerFt;
            fi = null;
            Level baseLvl = doc.GetElement(setup.BaseLevelId) as Level ?? lvl;
            var template = doc.GetElement(setup.TemplateTypeId) as FamilySymbol;
            if (template == null) return "The column type chosen in the Column Setup no longer exists - run Column Setup again.";

            bool shapeOk = r.Shape == ColumnShape.Round ? ColumnBuilder.IsRound(template) : ColumnBuilder.IsRectangular(template);
            string autoSwitchNote = "";
            if (!shapeOk && maySwitchFamily)
            {
                // No shape promise was made up front - if the scan turns out round (or rectangular) but the family chosen in
                // Column Setup is the other shape, look for a loaded family that matches instead of failing.
                FamilySymbol alt = FindAlternateTemplate(doc, s, r.Shape);
                if (alt != null)
                {
                    template = alt;
                    autoSwitchNote = "auto-switched to " + (r.Shape == ColumnShape.Round ? "round" : "rectangular") + " family '" + alt.Family.Name + " : " + alt.Name + "'";
                    s.ColAltTypeName = alt.Family.Name + " : " + alt.Name;
                    s.Save();
                }
                shapeOk = r.Shape == ColumnShape.Round ? ColumnBuilder.IsRound(template) : ColumnBuilder.IsRectangular(template);
            }
            if (!shapeOk)
                return "Found a " + (r.Shape == ColumnShape.Round ? "ROUND" : "RECTANGULAR") + " column, but the chosen type '" + template.Family.Name + " : " + template.Name +
                       "' is " + (r.Shape == ColumnShape.Round ? "not round (no Diameter/D/d parameter)" : "not rectangular (no Width/Depth or b/h parameters)") +
                       " - and no other loaded structural column family of that shape was found. Load one, then run Column Setup.";

            var loc = new XYZ(r.Center.X / M, r.Center.Y / M, baseLvl.Elevation);
            double snapFt = s.TypeSnapM / M;
            FamilySymbol sym = ColumnBuilder.GetOrCreateType(doc, template, r, snapFt, out string note);
            if (sym == null) return "Could not build a matching column type.";
            double wFt = r.Shape == ColumnShape.Round ? 0 : r.Width / M, dFt = r.Shape == ColumnShape.Round ? 0 : r.Depth / M;
            fi = ColumnBuilder.CreateColumn(doc, setup, baseLvl, sym, loc, r.Shape == ColumnShape.Round ? 0 : r.Angle, wFt, dFt);
            if (fi == null) return "Revit could not create the column.";

            string sizeText = r.Shape == ColumnShape.Round ? (r.Diameter * 1000).ToString("0") + " mm dia" : (r.Width * 1000).ToString("0") + "x" + (r.Depth * 1000).ToString("0") + " mm";
            string summary = string.Format("{0} | surface roughness {1:0.0} mm | coverage {2:0}% | {3}", sizeText, r.Spread * 1000, r.Coverage * 100, r.Grade.ToUpperInvariant());

            // The SAME extra notes (auto-switched family, auto-adjusted slice height, wall-angle fallback) that appear in the live
            // status bar are saved in the Comments too, so anyone reading a placed column's Comments later sees them.
            string extraNotes = (autoSwitchNote.Length > 0 ? "  [" + autoSwitchNote + "]" : "") + moreNotes;
            if (s.QualityNote)
            {
                Parameter cm = fi.get_Parameter(BuiltInParameter.ALL_MODEL_INSTANCE_COMMENTS);
                if (cm != null && !cm.IsReadOnly) cm.Set("PCW " + summary + " | " + modeText + " | " + note + extraNotes);
            }
            return "[" + r.Grade.ToUpperInvariant() + "] " + summary + "  (" + note + ")" + extraNotes;
        }

        /// <summary>The direction (radians) of whichever wall's location line is closest to the click, if one is within a
        /// plausible distance - used only as a fallback hint when a column cannot be separated from the wall data at all
        /// (see the comment at its call site). Either wall at a corner gives the same usable hint, since a column embedded
        /// there is virtually always aligned to both.</summary>
        private static double? NearbyWallAngle(Document doc, ViewPlan view, P2 clickA, P2 clickB)
        {
            const double M = Ctx.MPerFt;
            XYZ a = new XYZ(clickA.X / M, clickA.Y / M, 0), b = new XYZ(clickB.X / M, clickB.Y / M, 0);
            Wall best = null; double bestD = double.MaxValue;
            foreach (Wall w in new FilteredElementCollector(doc, view.Id).OfClass(typeof(Wall)).Cast<Wall>())
            {
                var lc = w.Location as LocationCurve;
                if (lc == null || lc.Curve == null) continue;
                // Only ONE of the two clicked corners is typically the one actually embedded in a wall (the other is the
                // column's free, room-facing corner, which can sit well clear of any wall centerline) - checking both and
                // keeping whichever is closer is what makes this reliably find the right wall regardless of click order.
                double da = lc.Curve.Project(a).XYZPoint.DistanceTo(a);
                double db = lc.Curve.Project(b).XYZPoint.DistanceTo(b);
                double d = Math.Min(da, db);
                // A generous allowance - a column can be a fair size, so its embedded corner may sit well off the wall's
                // own centerline. This only ever feeds a FALLBACK that already requires ordinary detection to have failed
                // to be confident, so a generous search here has little downside even if it occasionally finds a wall
                // that turns out not to be the relevant one.
                double allowance = w.Width / 2 + 1.2 / M;
                if (d <= allowance && d < bestD) { bestD = d; best = w; }
            }
            if (best == null) return null;
            var curve = ((LocationCurve)best.Location).Curve;
            XYZ dir = (curve.GetEndPoint(1) - curve.GetEndPoint(0)).Normalize();
            return Math.Atan2(dir.Y, dir.X);
        }

        /// <summary>A loaded structural column family matching the given shape - the remembered one from a previous
        /// auto-switch if it still matches, otherwise the first matching family found.</summary>
        internal static FamilySymbol FindAlternateTemplate(Document doc, Settings s, ColumnShape shape)
        {
            var types = WallSetup.ColumnTypes(doc);
            bool IsMatch(FamilySymbol t) => shape == ColumnShape.Round ? ColumnBuilder.IsRound(t) : ColumnBuilder.IsRectangular(t);
            if (!string.IsNullOrEmpty(s.ColAltTypeName))
            {
                var remembered = types.FirstOrDefault(t => t.Family.Name + " : " + t.Name == s.ColAltTypeName);
                if (remembered != null && IsMatch(remembered)) return remembered;
            }
            return types.FirstOrDefault(IsMatch);
        }
    }
}
