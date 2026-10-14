using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using Autodesk.Revit.UI.Selection;
using PointCloudWalls.Core;

namespace PointCloudWalls.Revit
{
    [Transaction(TransactionMode.Manual)]
    public class PcWallAutoCommand : IExternalCommand       // "Wall Auto"
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => AutoWalls.Run(data);
    }

    /// <summary>What one automatic search found.</summary>
    internal sealed class AutoFound
    {
        public AutoWallResult Result;
        /// <summary>Found, but a wall is already modeled there: never modeled twice.</summary>
        public List<AutoWall> AlreadyModeled = new List<AutoWall>();
        public int ColumnsAlreadyModeled;
        /// <summary>Slice points left out because neither floor nor ceiling was scanned there (mirror images in glass, outside).</summary>
        public int OutsidePoints;
        /// <summary>Area (m2) where floor or ceiling was scanned; 0 = not known.</summary>
        public double InteriorM2;
        public int SlicePoints;
        public double CutZFt;
    }

    /// <summary>
    /// "Wall Auto": finds the walls in the whole slice of the open plan view without clicks (PointCloudWalls.Core.AutoWallFinder) and
    /// builds each one through the SAME code as the click tools (WallPicker.Build): wall type per thickness, trim/join, quality note.
    /// </summary>
    internal static class AutoWalls
    {
        private const double M = Ctx.MPerFt;
        private const string Title = "Wall Auto";
        /// <summary>Diagnostics: set to see what the 3D checks counted (one line per check).</summary>
        internal static Action<string> Log { get; set; }

        public static Result Run(ExternalCommandData data)
        {
            if (!WallPicker.Prepare(data, out UIDocument uidoc, out ViewPlan view, out Level lvl, out List<CloudSource> clouds, out WallSetup setup)) return Result.Cancelled;
            Document doc = uidoc.Document;
            Settings s = Settings.Load();

            // ---- where --------------------------------------------------------------------------------------------------------
            var ask = new TaskDialog(Title)
            {
                MainInstruction = "Find the walls in this view's point cloud",
                MainContent = "Walls are found without clicking and measured the same way the click tools measure them. " +
                              "Nothing is created yet: you see what was found first.",
                CommonButtons = TaskDialogCommonButtons.Cancel,
            };
            ask.AddCommandLink(TaskDialogCommandLinkId.CommandLink1, "Whole view",
                "Everything of the point cloud visible in this view" + (view.CropBoxActive ? " (inside the crop region)." : "."));
            ask.AddCommandLink(TaskDialogCommandLinkId.CommandLink2, "Pick an area", "Drag a rectangle; only walls inside it are modeled.");
            TaskDialogResult where = ask.Show();
            double[] box;
            if (where == TaskDialogResult.CommandLink1) box = WholeView(view, clouds);
            else if (where == TaskDialogResult.CommandLink2)
            {
                try
                {
                    PickedBox pb = uidoc.Selection.PickBox(PickBoxStyle.Enclosing, "Drag a rectangle around the walls to model (Esc = cancel)");
                    box = new[] { Math.Min(pb.Min.X, pb.Max.X), Math.Min(pb.Min.Y, pb.Max.Y), Math.Max(pb.Min.X, pb.Max.X), Math.Max(pb.Min.Y, pb.Max.Y) };
                }
                catch (Autodesk.Revit.Exceptions.OperationCanceledException) { return Result.Cancelled; }
            }
            else return Result.Cancelled;
            if (box == null || box[2] - box[0] < 0.5 / M || box[3] - box[1] < 0.5 / M)
            {
                TaskDialog.Show(Title, "That area is too small to hold a wall.");
                return Result.Cancelled;
            }

            // ---- find ---------------------------------------------------------------------------------------------------------
            AutoFound found;
            try { found = Find(doc, view, lvl, clouds, s, setup, box); }
            catch (Exception ex)
            {
                TaskDialog.Show(Title, "The point cloud could not be searched:\n\n" + ex.Message);
                return Result.Failed;
            }
            List<AutoWall> walls = found.Result.Walls;
            List<AutoColumn> columns = found.Result.Columns;
            if (walls.Count + columns.Count == 0)
            {
                TaskDialog.Show(Title, "No new walls found.\n\n" + Describe(found, s) +
                    (found.SlicePoints < 500 ? "\n\nThe slice holds almost no points - check that the view's cut plane goes through the walls (Diagnostics shows what is read)." : ""));
                return Result.Cancelled;
            }

            var confirm = new TaskDialog(Title)
            {
                MainInstruction = walls.Count + (walls.Count == 1 ? " wall" : " walls") + (columns.Count > 0 ? " and " + columns.Count + (columns.Count == 1 ? " column" : " columns") : "") + " found",
                MainContent = Describe(found, s),
                CommonButtons = TaskDialogCommonButtons.Cancel,
            };
            confirm.AddCommandLink(TaskDialogCommandLinkId.CommandLink1, "Create them",
                "On " + setup.BaseLevelName + ", from '" + setup.TemplateName + "'. One Undo removes them all.");
            if (confirm.Show() != TaskDialogResult.CommandLink1) return Result.Cancelled;

            // ---- create -------------------------------------------------------------------------------------------------------
            string report = Create(doc, view, lvl, s, setup, walls, columns, out int created);
            TaskDialog.Show(Title, report);
            return created > 0 ? Result.Succeeded : Result.Failed;
        }

        // =====================================================================================================
        /// <summary>The area "Whole view" means, as [x0, y0, x1, y1] in feet: the visible clouds, inside the crop region when one is on.</summary>
        internal static double[] WholeView(ViewPlan view, List<CloudSource> clouds)
        {
            double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
            foreach (CloudSource cs in clouds)
            {
                BoundingBoxXYZ bb = cs.Pc.get_BoundingBox(null);
                if (bb == null) continue;
                x0 = Math.Min(x0, bb.Min.X); y0 = Math.Min(y0, bb.Min.Y); x1 = Math.Max(x1, bb.Max.X); y1 = Math.Max(y1, bb.Max.Y);
            }
            if (x0 > x1) return null;
            if (view.CropBoxActive)
            {
                BoundingBoxXYZ cb = view.CropBox;                       // in the view's own frame: a rotated view has a rotated crop region
                double cx0 = double.MaxValue, cy0 = double.MaxValue, cx1 = double.MinValue, cy1 = double.MinValue;
                foreach (double x in new[] { cb.Min.X, cb.Max.X })
                    foreach (double y in new[] { cb.Min.Y, cb.Max.Y })
                    {
                        XYZ p = cb.Transform.OfPoint(new XYZ(x, y, 0));
                        cx0 = Math.Min(cx0, p.X); cy0 = Math.Min(cy0, p.Y); cx1 = Math.Max(cx1, p.X); cy1 = Math.Max(cy1, p.Y);
                    }
                x0 = Math.Max(x0, cx0); y0 = Math.Max(y0, cy0); x1 = Math.Min(x1, cx1); y1 = Math.Min(y1, cy1);
            }
            return x0 < x1 && y0 < y1 ? new[] { x0, y0, x1, y1 } : null;
        }

        /// <summary>The slice of the area, read in tiles so no single read runs into the point limit (a tile that does is split).</summary>
        private static List<P2> ReadSlice(List<CloudSource> clouds, double[] box, double zMin, double zMax, double spacingM, int maxPoints)
        {
            var pts = new List<P2>();
            double tile = 8.0 / M;
            int nx = Math.Max(1, (int)Math.Ceiling((box[2] - box[0]) / tile)), ny = Math.Max(1, (int)Math.Ceiling((box[3] - box[1]) / tile));
            double tx = (box[2] - box[0]) / nx, ty = (box[3] - box[1]) / ny;
            foreach (CloudSource cs in clouds)
                for (int i = 0; i < nx; i++)
                    for (int j = 0; j < ny; j++)
                        ReadTile(cs, box[0] + (i + 0.5) * tx, box[1] + (j + 0.5) * ty, tx / 2, ty / 2, zMin, zMax, spacingM, maxPoints, pts, 0);
            return pts;
        }

        private static void ReadTile(CloudSource cs, double cx, double cy, double hx, double hy, double zMin, double zMax, double spacingM, int maxPoints, List<P2> into, int depth)
        {
            List<XYZ> got = cs.Query(new XYZ(cx, cy, 0), XYZ.BasisX, hx, hy, zMin, zMax, spacingM / M, maxPoints);
            if (got.Count >= maxPoints && depth < 4)
            {
                foreach (int sx in new[] { -1, 1 })
                    foreach (int sy in new[] { -1, 1 })
                        ReadTile(cs, cx + sx * hx / 2, cy + sy * hy / 2, hx / 2, hy / 2, zMin, zMax, spacingM, maxPoints, into, depth + 1);
                return;
            }
            foreach (XYZ p in got) into.Add(new P2(p.X * M, p.Y * M));
        }

        // =====================================================================================================
        /// <summary>Reads the slice of the area and finds the walls in it. Changes nothing in the model.</summary>
        internal static AutoFound Find(Document doc, ViewPlan view, Level lvl, List<CloudSource> clouds, Settings s, WallSetup setup, double[] box, Func<Element, bool> ignore = null)
        {
            double cutZ = Ctx.CutZ(doc, view, lvl, s);
            double zMin = cutZ - s.SlabHalfM / M, zMax = cutZ + s.SlabHalfM / M;
            List<P2> slice = ReadSlice(clouds, box, zMin, zMax, s.SpacingM, s.MaxPoints);
            IWallPointSource src = new RevitPointSource(clouds, zMin, zMax, s);     // each wall is measured from the cloud exactly like a clicked one

            // the wall's own base and top: where the 3D checks look
            Level baseLvl = doc.GetElement(setup.BaseLevelId) as Level ?? lvl;
            double baseZ = baseLvl.Elevation + setup.BaseOffsetFt;
            double topZ = Math.Max(baseZ + WallBuilder.HeightFt(doc, setup, baseLvl), cutZ + 1.0 / M);

            List<XYZ> Box3D(P2 center, P2 axis, double halfLen, double halfWid, double z0, double z1, int max)
            {
                var res = new List<XYZ>();
                foreach (CloudSource cs in clouds)
                    res.AddRange(cs.Query(new XYZ(center.X / M, center.Y / M, 0), new XYZ(axis.X, axis.Y, 0), halfLen / M, halfWid / M, z0, z1, s.SpacingM / M, max));
                return res;
            }

            // Where is the building really? Where the floor or the ceiling was scanned. The slice also catches mirror images of rooms
            // in window glass and whatever stands outside - walls as sharp as real ones, but with no floor and no ceiling of their own.
            List<P2> floorPts = ReadSlice(clouds, box, baseZ - 0.15 / M, baseZ + 0.15 / M, 0.03, s.MaxPoints);
            List<P2> upperPts = topZ - zMax > 0.20 / M ? ReadSlice(clouds, box, zMax + 0.05 / M, topZ, 0.03, s.MaxPoints) : new List<P2>();
            var interior = new InteriorMap(floorPts, upperPts);
            bool knowInterior = interior.Area >= 4.0;
            int allPoints = slice.Count;
            if (knowInterior)
            {
                slice = slice.Where(q => interior.Near(q, 0.40)).ToList();
                src = new InteriorPointSource(src, interior, 0.40);
            }

            WallDetectOptions d = s.ToDetectOptions();
            if (knowInterior) d.FloorShare = interior.Share;
            d.Log = Log;

            // A door or a window, or the end of the wall? Looked at in 30 cm bands from the slice up to the top of the wall, in the
            // footprint of the wall across the gap:
            //   - the wall carries on above the opening: its faces are scanned there, so the points lie on a few LINES along the wall
            //     (whatever its thickness up there)                                                    => an opening IN the wall
            //   - points spread all over the footprint: a ceiling, open space                          => the wall ends here
            //   - nothing scanned above at all: it cannot be told; a door-sized gap between two walls is most probably a door
            d.OpeningIsInWall = (a, b, thick) =>
            {
                P2 ax = (b - a).Normalized(), n = ax.Perp, mid = (a + b) * 0.5;
                double gap = (b - a).Length, half = gap / 2 - 0.05;
                if (half < 0.05) return true;
                double z0 = zMax + 0.05 / M, band = 0.30 / M, wide = thick / 2 + 0.03;
                int bands = (int)Math.Floor((topZ - z0) / band);
                if (bands < 1) return true;
                int bins = (int)Math.Ceiling(2 * wide / 0.02);
                var hist = new int[bands, bins]; var total = new int[bands];
                foreach (XYZ p in Box3D(mid, ax, half, wide, z0, z0 + bands * band, 20000))
                {
                    int k = Math.Min(bands - 1, (int)((p.Z - z0) / band));
                    double c = (new P2(p.X * M, p.Y * M) - mid).Dot(n);
                    hist[k, Math.Max(0, Math.Min(bins - 1, (int)((c + wide) / 0.02)))]++;
                    total[k]++;
                }
                bool lines = false, spread = false;
                var top = new List<int>();
                for (int k = 0; k < bands; k++)
                {
                    if (total[k] < 20 * gap) continue;                       // too little in this band to say anything
                    top.Clear();
                    for (int q = 0; q < bins; q++) top.Add(hist[k, q]);
                    top.Sort(); top.Reverse();
                    int onLines = top.Take(4).Sum();                         // two faces, each a line one or two bins wide
                    if (onLines >= 0.65 * total[k]) lines = true; else spread = true;
                }
                bool inWall = lines || !spread;
                Log?.Invoke("opening " + a + " -> " + b + " t=" + thick.ToString("0.000") + " points per band: " + string.Join(" ", Enumerable.Range(0, bands).Select(k => total[k])) +
                            (lines ? "  => wall above" : spread ? "  => open above" : "  => nothing scanned above, taken for a door"));
                return inWall;
            };

            // Which side of a single surface is the wall? On the room side the FLOOR is scanned right up to the surface; inside the wall
            // there is no floor. Looked at in a 5 cm strip just off the surface, on both sides, at the wall's base. (Only the floor:
            // above a wall that stops at a suspended ceiling the scan often shows the slab and services - that says nothing.)
            if (s.AutoOneFace)
                d.BodySide = (a, b) =>
                {
                    P2 ax = (b - a).Normalized(), n = ax.Perp, mid = (a + b) * 0.5;
                    double half = Math.Max(0.25, (b - a).Length / 2 - 0.15);
                    int left = Box3D(mid + n * 0.055, ax, half, 0.025, baseZ - 0.15 / M, baseZ + 0.15 / M, 4000).Count;
                    int right = Box3D(mid - n * 0.055, ax, half, 0.025, baseZ - 0.15 / M, baseZ + 0.15 / M, 4000).Count;
                    int room = Math.Max(left, right), wall = Math.Min(left, right);
                    int side = room < Math.Max(8, 6 * half) || wall > Math.Max(1, 0.12 * room) ? 0      // no floor seen, or floor on both sides
                             : left < right ? 1 : -1;
                    Log?.Invoke("side " + a + " -> " + b + " floor points left=" + left + " right=" + right + " => " + side);
                    return side;
                };

            var found = new AutoFound { SlicePoints = slice.Count, OutsidePoints = allPoints - slice.Count, InteriorM2 = knowInterior ? interior.Area : 0, CutZFt = cutZ, Result = AutoWallFinder.Run(slice, src, d, s.ToOptions(), s.ToColumnOptions()) };

            // never model a wall twice - and never drop what is still missing: of a wall that is partly in the model, the rest is built
            var existing = new List<(P2 a, P2 b, double t)>();
            foreach (Wall w in new FilteredElementCollector(doc).OfClass(typeof(Wall)).Cast<Wall>())
            {
                if (!w.LevelId.Equals(baseLvl.Id) || (ignore != null && ignore(w))) continue;
                var ln = (w.Location as LocationCurve)?.Curve as Line;
                if (ln == null) continue;
                XYZ p0 = ln.GetEndPoint(0), p1 = ln.GetEndPoint(1);
                existing.Add((new P2(p0.X * M, p0.Y * M), new P2(p1.X * M, p1.Y * M), w.Width * M));
            }
            foreach (AutoWall w in found.Result.Walls.ToList())
            {
                var parts = AutoWallFinder.UnmodeledParts(w.Fit, existing, s.AutoMinLengthM);
                if (parts.Count == 1 && (parts[0].start - w.Fit.Start).Length < 1e-6 && (parts[0].end - w.Fit.End).Length < 1e-6) continue;   // nothing of it is modeled
                found.Result.Walls.Remove(w);
                if (parts.Count == 0) { found.AlreadyModeled.Add(w); continue; }
                foreach (var part in parts)
                {
                    WallFitResult fit = w.Fit.Clone();
                    fit.Start = part.start; fit.End = part.end;
                    found.Result.Walls.Add(new AutoWall { Detected = w.Detected, Fit = fit, OneFace = w.OneFace, Extended = w.Extended });
                }
            }
            var columnsThere = new List<P2>();
            foreach (Element e in new FilteredElementCollector(doc).OfCategory(BuiltInCategory.OST_StructuralColumns).WhereElementIsNotElementType())
                if ((ignore == null || !ignore(e)) && e.Location is LocationPoint lp) columnsThere.Add(new P2(lp.Point.X * M, lp.Point.Y * M));
            found.ColumnsAlreadyModeled = found.Result.Columns.RemoveAll(c => columnsThere.Any(q => (q - c.Fit.Center).Length < 0.30));
            return found;
        }

        // =====================================================================================================
        /// <summary>Builds the walls, each exactly like a clicked wall, inside ONE undo step.</summary>
        internal static string Create(Document doc, ViewPlan view, Level lvl, Settings s, WallSetup setup, List<AutoWall> walls, List<AutoColumn> columns, out int created)
        {
            created = 0;
            int failed = 0, twoFace = 0, oneFace = 0, columnsMade = 0;
            string columnNote = "";
            var columnSizes = new SortedDictionary<string, int>();
            double length = 0;
            string firstError = null;
            var byType = new SortedDictionary<int, int>();
            var grades = new Dictionary<string, int>();
            var failures = new BatchFailures();
            using (var tg = new TransactionGroup(doc, "PC walls - automatic"))
            {
                tg.Start();
                foreach (AutoWall a in walls)
                {
                    using (var t = new Transaction(doc, "PC wall - automatic"))
                    {
                        t.Start();
                        FailureHandlingOptions fho = t.GetFailureHandlingOptions();
                        fho.SetFailuresPreprocessor(failures);                       // no dialog per wall: warnings are counted, an error drops that wall
                        fho.SetClearAfterRollback(true);
                        t.SetFailureHandlingOptions(fho);

                        Wall wall = null; string status;
                        try { status = WallPicker.Build(doc, view, lvl, s, setup, a.Fit, a.OneFace ? "automatic mode, one face" : "automatic mode", out wall); }
                        catch (Exception ex) { status = ex.Message; wall = null; }
                        if (wall == null)
                        {
                            if (t.GetStatus() == TransactionStatus.Started) t.RollBack();
                            failed++; firstError = firstError ?? status;
                            continue;
                        }
                        if (t.Commit() != TransactionStatus.Committed) { failed++; firstError = firstError ?? "Revit rejected the wall."; continue; }
                        created++;
                        if (a.OneFace) oneFace++; else twoFace++;
                        length += a.Fit.Length;
                        int mm = (int)Math.Round(wall.Width * 304.8);
                        byType[mm] = (byType.TryGetValue(mm, out int c) ? c : 0) + 1;
                        grades[a.Fit.Grade] = (grades.TryGetValue(a.Fit.Grade, out int g) ? g : 0) + 1;
                    }
                }

                // ---- the columns, each exactly like a Column Diagonal column, with the Column Setup choices
                if (columns.Count > 0)
                {
                    WallSetup colSetup = WallSetup.FromSettings(doc, view, s, preferView: false, kind: SetupKind.Column);
                    if (colSetup.BaseLevelId == null || colSetup.TemplateTypeId == null)
                        columnNote = columns.Count + " columns were found but not built: no structural column family is loaded (see Column Setup).";
                    else
                        foreach (AutoColumn c in columns)
                            using (var t = new Transaction(doc, "PC column - automatic"))
                            {
                                t.Start();
                                FailureHandlingOptions fho = t.GetFailureHandlingOptions();
                                fho.SetFailuresPreprocessor(failures);
                                fho.SetClearAfterRollback(true);
                                t.SetFailureHandlingOptions(fho);

                                FamilyInstance fi = null; string status;
                                try { status = ColumnPicker.Build(doc, lvl, s, colSetup, c.Fit, c.Detected.FreeStanding ? "automatic mode" : "automatic mode, pier on a wall (depth = the part standing proud of the wall)", true, "", out fi); }
                                catch (Exception ex) { status = ex.Message; fi = null; }
                                if (fi == null)
                                {
                                    if (t.GetStatus() == TransactionStatus.Started) t.RollBack();
                                    failed++; firstError = firstError ?? status;
                                    continue;
                                }
                                if (t.Commit() != TransactionStatus.Committed) { failed++; firstError = firstError ?? "Revit rejected the column."; continue; }
                                columnsMade++;
                                string size = (Math.Round(Math.Max(c.Fit.Width, c.Fit.Depth) * 100) * 10).ToString("0") + " x " + (Math.Round(Math.Min(c.Fit.Width, c.Fit.Depth) * 100) * 10).ToString("0") + " mm";
                                columnSizes[size] = (columnSizes.TryGetValue(size, out int n) ? n : 0) + 1;
                            }
                }
                if (created + columnsMade > 0) tg.Assimilate(); else tg.RollBack();
            }

            var sb = new StringBuilder();
            sb.AppendLine(created + (created == 1 ? " wall created" : " walls created") + ", " + length.ToString("0.0") + " m in total" +
                          (columnsMade > 0 ? ", and " + columnsMade + (columnsMade == 1 ? " column" : " columns") : "") + ". One Undo removes them all.");
            if (created > 0)
            {
                sb.AppendLine();
                sb.AppendLine("Measured from both faces: " + twoFace + (oneFace > 0 ? "     scanned from one side only: " + oneFace : ""));
                sb.AppendLine("Quality:  " + string.Join("   ", new[] { "Good", "OK", "Check" }.Where(grades.ContainsKey).Select(k => k.ToUpperInvariant() + " " + grades[k])));
                sb.AppendLine("Wall types:  " + string.Join("   ", byType.Select(k => k.Key + " mm x " + k.Value)));
                if (s.QualityNote) sb.AppendLine().AppendLine("Each wall's Comments hold its measurements; filter a schedule on \"CHECK\" or \"assumed\" to review the weaker ones.");
            }
            if (columnsMade > 0) sb.AppendLine().AppendLine("Columns:  " + string.Join("   ", columnSizes.Select(k => k.Key + " x " + k.Value)));
            if (columnNote.Length > 0) sb.AppendLine().AppendLine(columnNote);
            created += columnsMade;
            if (failed > 0) sb.AppendLine().AppendLine(failed + " could not be created" + (firstError != null ? ": " + firstError : "."));
            if (failures.Warnings.Count > 0)
            {
                sb.AppendLine().AppendLine("Revit warnings (not shown one by one, they would stop the run at every wall):");
                foreach (var w in failures.Warnings.OrderByDescending(k => k.Value)) sb.AppendLine("   " + w.Value + " x  " + w.Key);
            }
            return sb.ToString();
        }

        private sealed class BatchFailures : IFailuresPreprocessor
        {
            public readonly Dictionary<string, int> Warnings = new Dictionary<string, int>();

            public FailureProcessingResult PreprocessFailures(FailuresAccessor a)
            {
                bool error = false;
                foreach (FailureMessageAccessor m in a.GetFailureMessages())
                {
                    if (m.GetSeverity() != FailureSeverity.Warning) { error = true; continue; }
                    string text = m.GetDescriptionText();
                    Warnings[text] = (Warnings.TryGetValue(text, out int c) ? c : 0) + 1;
                    a.DeleteWarning(m);
                }
                return error ? FailureProcessingResult.ProceedWithRollBack : FailureProcessingResult.Continue;
            }
        }

        /// <summary>What was found, in plain words (shown before anything is created).</summary>
        internal static string Describe(AutoFound f, Settings s)
        {
            AutoWallResult r = f.Result;
            var sb = new StringBuilder();
            var two = r.Walls.Where(w => !w.OneFace).ToList(); var one = r.Walls.Where(w => w.OneFace).ToList();
            if (two.Count > 0)
                sb.AppendLine(two.Count + " measured from both faces  (" + two.Sum(w => w.Fit.Length).ToString("0") + " m):  " +
                              string.Join(",  ", two.GroupBy(w => (int)(Math.Round(w.Fit.Thickness * 1000 / 10.0) * 10)).OrderBy(g => g.Key).Select(g => g.Key + " mm x " + g.Count())));
            if (one.Count > 0)
            {
                int same = one.Count(w => !w.Fit.ThicknessAssumed);
                var how = new List<string>();
                if (same > 0) how.Add(same + " carry on from a measured wall and take its thickness");
                if (one.Count > same) how.Add((one.Count - same) + " get the ASSUMED thickness of " + (s.AssumedThicknessM * 1000).ToString("0") + " mm, with the body on the side where no floor was scanned");
                sb.AppendLine(one.Count + " scanned from one side only  (" + one.Sum(w => w.Fit.Length).ToString("0") + " m):  " + string.Join(";  ", how) + ".");
            }
            if (r.Columns.Count > 0)
                sb.AppendLine(r.Columns.Count + (r.Columns.Count == 1 ? " column" : " columns") + ":  " + r.Columns.Count(c => !c.Detected.FreeStanding) + " piers on a wall, " +
                              r.Columns.Count(c => c.Detected.FreeStanding) + " free-standing  (built with the Column Setup choices).");
            if (f.AlreadyModeled.Count + f.ColumnsAlreadyModeled > 0)
                sb.AppendLine(f.AlreadyModeled.Count + " walls" + (f.ColumnsAlreadyModeled > 0 ? " and " + f.ColumnsAlreadyModeled + " columns" : "") + " already modeled - left alone.");
            int unsure = r.Skipped.Count(w => w.Reason != "same wall found twice" && w.Reason != "lies inside a wall that is already found");
            if (unsure + r.SingleFaces.Count > 0)
            {
                sb.AppendLine();
                sb.AppendLine("Left for the click tools:");
                if (unsure > 0) sb.AppendLine("   " + unsure + " short or unclear pieces");
                if (r.SingleFaces.Count > 0)
                    sb.AppendLine("   " + r.SingleFaces.Count + " single surfaces  (" + r.SingleFaces.Sum(x => x.Length).ToString("0") + " m)" +
                                  (s.AutoOneFace ? " whose wall side is unclear" : " - one-sided walls are switched off in Settings"));
            }
            sb.AppendLine();
            if (f.OutsidePoints > 0)
                sb.AppendLine(f.OutsidePoints.ToString("N0") + " points lie where neither floor nor ceiling was scanned (mirror images in window glass, things outside) - ignored.");
            sb.Append("Slice: " + f.SlicePoints.ToString("N0") + " points at " + ZText(f.CutZFt) + (f.InteriorM2 > 0 ? ", " + f.InteriorM2.ToString("0") + " m2 of floor or ceiling scanned" : "") + ", wall directions " +
                      (r.DirectionsDeg.Count == 0 ? "none" : string.Join(" / ", r.DirectionsDeg.Select(x => x.ToString("0.0") + " deg"))) + ".");
            return sb.ToString();
        }

        private static string ZText(double zFt) => "Z " + (zFt * M).ToString("0.00") + " m";
    }
}
