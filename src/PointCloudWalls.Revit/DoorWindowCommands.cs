using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Autodesk.Revit.Attributes;
using Autodesk.Revit.DB;
using Autodesk.Revit.UI;
using PointCloudWalls.Core;

namespace PointCloudWalls.Revit
{
    [Transaction(TransactionMode.Manual)]
    public class PcDoorWindowCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements) => DoorWindowPicker.Run(data);
    }

    [Transaction(TransactionMode.Manual)]
    public class PcDoorWindowSetupCommand : IExternalCommand
    {
        public Result Execute(ExternalCommandData data, ref string message, ElementSet elements)
        {
            Document doc = data.Application.ActiveUIDocument.Document;
            Settings s = Settings.Load();
            DoorWindowSetup setup = DoorWindowSetup.FromSettings(doc, s);
            if (setup.DoorTypeId == null && setup.WindowTypeId == null)
            {
                TaskDialog.Show("Door/Window setup", "The project needs at least one loaded door family and one loaded window family.");
                return Result.Cancelled;
            }
            var win = new DoorWindowSetupWindow(doc, setup, s.AskDoorWindowSetup, false);
            if (win.ShowDialog() != true) return Result.Cancelled;
            win.Result.SaveToSettings(s);
            s.AskDoorWindowSetup = win.AskEachTime;
            s.Save();
            return Result.Succeeded;
        }
    }

    internal static class DoorWindowPicker
    {
        public static Result Run(ExternalCommandData data)
        {
            UIDocument uidoc = data.Application.ActiveUIDocument;
            Document doc = uidoc.Document;
            var view = doc.ActiveView as ViewPlan;
            if (view == null || view.GenLevel == null)
            {
                TaskDialog.Show("PC Door/Window", "Open a floor plan view (with the point cloud visible) first.");
                return Result.Cancelled;
            }
            var clouds = Ctx.CloudsInView(doc, view);
            if (clouds.Count == 0)
            {
                TaskDialog.Show("PC Door/Window", "No point cloud is visible in this view.\n\nCheck Visibility/Graphics > Point Clouds and that the cloud is loaded.");
                return Result.Cancelled;
            }
            List<Wall> walls = new FilteredElementCollector(doc, view.Id).OfClass(typeof(Wall)).Cast<Wall>().ToList();
            if (walls.Count == 0)
            {
                TaskDialog.Show("PC Door/Window", "No walls are modeled in this view yet - a door or window needs a wall to host it. Model the walls first (PC Wall tools), then run this.");
                return Result.Cancelled;
            }

            Settings s0 = Settings.Load();
            bool ask = s0.AskDoorWindowSetup;
            DoorWindowSetup setup = DoorWindowSetup.FromSettings(doc, s0);
            if (setup.DoorTypeId == null && setup.WindowTypeId == null)
            {
                TaskDialog.Show("PC Door/Window", "The project needs at least one loaded door family and one loaded window family " +
                    "(Insert > Load Family) before this tool can place anything.");
                return Result.Cancelled;
            }
            if (ask)
            {
                var win = new DoorWindowSetupWindow(doc, setup, true, true);
                if (win.ShowDialog() != true) return Result.Cancelled;
                setup = win.Result;
                setup.SaveToSettings(s0); s0.AskDoorWindowSetup = win.AskEachTime; s0.Save();
            }

            string status = "";
            while (true)
            {
                XYZ a;
                try { a = uidoc.Selection.PickPoint((status.Length > 0 ? status + "   |   " : "") + "Click inside a door or window opening (Esc = finish)"); }
                catch (Autodesk.Revit.Exceptions.OperationCanceledException) { break; }

                try { status = Process(doc, walls, clouds, Settings.Load(), setup, a); }
                catch (Exception ex) { status = "ERROR: " + ex.Message; }
            }
            return Result.Succeeded;
        }

        /// <summary>The wall whose location line is closest to the click, within a plausible distance of its own face.</summary>
        private static Wall NearestWall(List<Wall> walls, XYZ click)
        {
            Wall best = null; double bestD = double.MaxValue;
            foreach (Wall w in walls)
            {
                var lc = w.Location as LocationCurve;
                if (lc == null || lc.Curve == null) continue;
                XYZ p = lc.Curve.Project(click).XYZPoint;
                double d = p.DistanceTo(click);
                double allowance = w.Width / 2 + 1.0 / Ctx.MPerFt;      // half the wall's thickness, plus ~1 m of click slack
                if (d <= allowance && d < bestD) { bestD = d; best = w; }
            }
            return best;
        }

        private static string Process(Document doc, List<Wall> walls, List<CloudSource> clouds, Settings s, DoorWindowSetup setup, XYZ click)
        {
            const double M = Ctx.MPerFt;
            Wall host = NearestWall(walls, click);
            if (host == null) return "No wall found near that click - click closer to (or inside) a modeled wall.";
            var lc = host.Location as LocationCurve;
            Curve axis = lc.Curve;
            XYZ p0 = axis.GetEndPoint(0), p1 = axis.GetEndPoint(1);
            XYZ dir = (p1 - p0).Normalize();

            double wallBaseZ = host.get_BoundingBox(null)?.Min.Z ?? p0.Z;
            double wallTopZ = host.get_BoundingBox(null)?.Max.Z ?? (wallBaseZ + 8.0);
            double alongClick = (click - p0).DotProduct(dir);

            // Temporary troubleshooting aid: every PC Door/Window click writes what OpeningFitter actually did - the
            // per-column floor-vs-leaf density readings, the flood-fill extents before and after edge refinement -
            // to a plain text file, overwritten each click, so a real scan that still doesn't behave right can be
            // diagnosed from the actual data instead of guesswork. Safe to remove once the leaf detector is dialed in.
            var diagLog = new StringBuilder();
            var o = new OpeningFitOptions
            {
                InlierTol = s.InlierTolM, SearchHalfWidth = s.OpenSearchHalfWidthM,
                MinWidth = s.OpenMinWidthM, MaxWidth = s.OpenMaxWidthM,
                MinHeight = s.OpenMinHeightM, MaxHeight = s.OpenMaxHeightM,
                FloorTolerance = s.OpenFloorTolM, MinJambCoverage = s.OpenMinJambCoverage, BridgeMaxWidth = s.OpenBridgeMaxWidthM,
                Log = line => diagLog.AppendLine(line),
            };

            double queryHalfLen = (o.SearchHalfWidth + 0.3) / M;
            // step55: confirmed against TWO real doors this session (not just synthetic data) that a query band tight
            // enough to exclude a swung-open door leaf is also too tight to ever see the true far jamb when that leaf
            // happens to occlude it, or leaves the leaf's own material indistinguishable from real wall once flattened
            // to (along-wall, height) - a leaf touches the floor and spans nearly the full door height just like a
            // real jamb does, so neither of the existing floor/height tests catches it. So the query band is widened
            // back out (wide enough to comfortably catch a fully open ~900mm leaf) and each point's DEPTH - its
            // distance from the wall's own face, perpendicular to it - is computed and handed to OpeningFitter, which
            // now excludes anything farther than the wall's own thickness (plus a small tolerance) from the occupancy
            // grid before the flood fill runs. Real wall material never leaves its own face by more than that; an
            // open leaf reliably does everywhere except right at its own hinge (see MaxWallDepth's doc comment on
            // OpeningFitOptions for the one real, physical limit this still has).
            double wallFaceHalfWidth = host.Width / 2 * M;                  // meters, from the wall's centerline to each real face
            double perpHalfWidth = (wallFaceHalfWidth + 0.9) / M;          // ~0.9m past each face - a fully open leaf plus margin
            XYZ queryCenter = p0 + dir * alongClick;
            XYZ normal = new XYZ(-dir.Y, dir.X, 0).Normalize();            // horizontal, perpendicular to the wall's own length
            var points3d = new List<XYZ>();
            foreach (var cs in clouds)
                points3d.AddRange(cs.Query(queryCenter, dir, queryHalfLen, perpHalfWidth, wallBaseZ, wallTopZ + 0.4 / M, s.SpacingM / M, s.MaxPoints));
            if (points3d.Count < 20) return "Too few scan points on this wall near the click.";

            var pts2d = points3d.Select(p => new P2((p - p0).DotProduct(dir) * M, (p.Z - wallBaseZ) * M)).ToList();
            var depths2d = points3d.Select(p => (p - p0).DotProduct(normal) * M).ToList();
            P2 click2d = new P2(alongClick * M, (click.Z - wallBaseZ) * M);

            // The Revit wall element's own modeled base does not always match where its surface actually starts in the
            // real scan (e.g. a wall commonly modeled a little below finish floor to tie into a slab) - classifying a
            // door vs a window by comparing against the WRONG one of these can misjudge a genuine floor-reaching door.
            // The scan's own floor is used instead, wherever it turns out to actually be.
            double floorOffsetM = OpeningFitter.EstimateFloorOffset(pts2d);
            // 60mm past the wall's own face: comfortably beyond ordinary scan/registration noise on a real wall
            // surface, tight enough to still exclude most of a swung leaf's own face (confirmed against real data
            // from two doors this session).
            o.MaxWallDepth = wallFaceHalfWidth + 0.06;
            diagLog.Insert(0, $"floorOffsetM (EstimateFloorOffset)={floorOffsetM:F3}  host.Width={host.Width * M:F3}m  wallFaceHalfWidth={wallFaceHalfWidth:F3}m  MaxWallDepth={o.MaxWallDepth:F3}m  points3d={points3d.Count}\r\n");
            var r = OpeningFitter.Fit(pts2d, click2d, floorOffsetM, (wallTopZ - wallBaseZ) * M, o, depths2d);

            // Diagnostics: the same points, with depth, so a scan that still doesn't behave right can be inspected
            // directly rather than by guesswork. Does not feed back into r or the command's outcome.
            var wide3 = points3d.Select((p, i) => (X: pts2d[i].X, Y: pts2d[i].Y, Depth: depths2d[i])).ToList();
            WriteDiag(diagLog.ToString(), pts2d, click2d, r, wide3);
            if (!r.Success) return r.Message + "  [diagnostics: %TEMP%\\PointCloudWalls_door_diag.txt, PointCloudWalls_door_points_wide_depth.csv]";

            bool wantsDoor = r.Kind == OpeningKind.Door;
            ElementId typeId = wantsDoor ? setup.DoorTypeId : setup.WindowTypeId;
            string autoSwitchNote = "";
            if (typeId == null)
            {
                // the setup only has a family for the OTHER kind - fall back to it rather than failing outright, same
                // spirit as a column auto-switching shape, though here it is a straight "use what's loaded" fallback
                typeId = wantsDoor ? setup.WindowTypeId : setup.DoorTypeId;
                wantsDoor = !wantsDoor;
                autoSwitchNote = "  [no " + (r.Kind == OpeningKind.Door ? "door" : "window") + " family set up - used the " + (wantsDoor ? "door" : "window") + " family instead]";
            }
            var cat = wantsDoor ? OpeningCategory.Door : OpeningCategory.Window;
            FamilySymbol template = doc.GetElement(typeId) as FamilySymbol;
            if (template == null) return "The chosen door/window family type no longer exists - run Door/Window Setup again.";

            double widthFt = r.Width / M, heightFt = r.Height / M;
            // r.SillHeight is measured above the SCAN's real floor (floorOffsetM), but Revit's own Sill Height parameter
            // (and the instance's Z) are measured relative to the wall's LEVEL (~wallBaseZ) - so the scan-detected floor
            // offset is added back in here to convert between the two references.
            double sillFt = (floorOffsetM + r.SillHeight) / M;
            string note;
            using (var t = new Transaction(doc, "PC Door/Window"))
            {
                t.Start();
                string widthParamName = wantsDoor ? s.DoorWidthParamName : s.WindowWidthParamName;
                string heightParamName = wantsDoor ? s.DoorHeightParamName : s.WindowHeightParamName;
                FamilySymbol sym = DoorWindowBuilder.GetOrCreateType(doc, template, widthFt, heightFt, s.TypeSnapM / M, widthParamName, heightParamName, out note);
                // Placed at the DETECTED opening's center (r.AlongCenter, from the point cloud), not at the raw click -
                // the click only tells OpeningFitter roughly where to look; once it has found the real opening, that
                // measured position is what should be used, not wherever the click happened to land.
                // For a wall-hosted instance, Revit places it along the host's own location curve from X/Y; the vertical
                // position is governed by the Sill Height parameter (set for windows above) rather than this point's Z,
                // so the wall's base elevation is a safe, neutral Z to pass here.
                XYZ insertAt = p0 + dir * (r.AlongCenter / M);
                insertAt = new XYZ(insertAt.X, insertAt.Y, wallBaseZ + floorOffsetM / M);
                FamilyInstance fi = DoorWindowBuilder.CreateInstance(doc, sym, insertAt, host, sillFt, cat);
                if (fi != null && s.QualityNote)
                {
                    Parameter cm = fi.get_Parameter(BuiltInParameter.ALL_MODEL_INSTANCE_COMMENTS);
                    if (cm != null && !cm.IsReadOnly)
                        cm.Set("PCW " + (wantsDoor ? "door" : "window") + " " + Math.Round(r.Width * 1000) + "x" + Math.Round(r.Height * 1000) + "mm" +
                               (wantsDoor ? "" : " sill=" + Math.Round(r.SillHeight * 1000) + "mm") +
                               " | surface roughness " + (r.Spread * 1000).ToString("0.0") + " mm | coverage " + (r.Coverage * 100).ToString("0") + "% | " + r.Grade.ToUpperInvariant());
                }
                t.Commit();
            }

            string summary = $"{(wantsDoor ? "Door" : "Window")} {r.Width * 1000:F0}x{r.Height * 1000:F0}mm" +
                              (wantsDoor ? "" : $" sill={r.SillHeight * 1000:F0}mm") +
                              $" | roughness {r.Spread * 1000:F1}mm | coverage {r.Coverage * 100:F0}%";
            return "[" + r.Grade.ToUpperInvariant() + "] " + summary + "  (" + note + ")" + autoSwitchNote + "  [diagnostics: %TEMP%\\PointCloudWalls_door_diag.txt, PointCloudWalls_door_points_wide_depth.csv]";
        }

        /// <summary>Temporary troubleshooting aid (see the comment where diagLog is created above): writes the fit's
        /// internal reasoning plus the raw 2D wall-surface points it worked from to a fixed, overwritten-each-click
        /// pair of files in %TEMP%, so a specific real scan's behaviour can be inspected after the fact.</summary>
        private static void WriteDiag(string log, List<P2> pts2d, P2 click2d, OpeningFitResult r,
            List<(double X, double Y, double Depth)> wide3 = null)
        {
            try
            {
                string dir = Path.GetTempPath();
                var sb = new StringBuilder();
                sb.AppendLine("PointCloudWalls door/window diagnostic - " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
                sb.AppendLine("click2d=(" + click2d.X.ToString("F3") + "," + click2d.Y.ToString("F3") + ")");
                sb.AppendLine("result: " + r);
                sb.AppendLine();
                sb.Append(log);
                File.WriteAllText(Path.Combine(dir, "PointCloudWalls_door_diag.txt"), sb.ToString());

                var csv = new StringBuilder();
                csv.AppendLine("x,y");
                foreach (var p in pts2d) csv.AppendLine(p.X.ToString("F4") + "," + p.Y.ToString("F4"));
                File.WriteAllText(Path.Combine(dir, "PointCloudWalls_door_points.csv"), csv.ToString());

                // Diagnostics-only wide capture with depth (distance from the wall's own face) - see the comment at
                // the call site. Depth=0 is the wall centerline; the two real faces sit at roughly ±host.Width/2.
                if (wide3 != null)
                {
                    var csvW = new StringBuilder();
                    csvW.AppendLine("x,y,depth");
                    foreach (var p in wide3) csvW.AppendLine(p.X.ToString("F4") + "," + p.Y.ToString("F4") + "," + p.Depth.ToString("F4"));
                    File.WriteAllText(Path.Combine(dir, "PointCloudWalls_door_points_wide_depth.csv"), csvW.ToString());
                }
            }
            catch { /* diagnostics are best-effort only - never let a write failure affect the actual command */ }
        }
    }
}
