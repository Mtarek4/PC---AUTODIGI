using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.Revit.DB;
using PointCloudWalls.Cad;

namespace PointCloudWalls.Revit
{
    /// <summary>Turns Revit walls into the CAD publish batch: footprint outline, location line, units, coordinates.</summary>
    internal static class CadPublish
    {
        public static double UnitsPerMeter(int index) => index == 2 ? 1.0 : index == 1 ? 100.0 : 1000.0;
        public static string UnitsName(int index) => index == 2 ? "m" : index == 1 ? "cm" : "mm";

        public static string PipeName(Settings s) => string.IsNullOrWhiteSpace(s.CadPipeName) ? CadPipe.DefaultName : s.CadPipeName.Trim();

        /// <summary>
        /// Selected walls if any are selected, otherwise every wall in the active view. With "merge" on, walls that touch or overlap
        /// are drawn as ONE outline; a selection is then extended to the walls it touches, so a merged outline is never half-published.
        /// </summary>
        public static PublishBatch Build(Document doc, View view, ICollection<ElementId> selection, Settings s, out string what)
        {
            var selectedElements = selection.Select(id => doc.GetElement(id)).ToList();
            var selected = selectedElements.OfType<Wall>().ToList();
            bool anyColumnSelected = selectedElements.OfType<FamilyInstance>()
                .Any(fi => fi.Category != null && fi.Category.Id.Value == (long)BuiltInCategory.OST_StructuralColumns);
            // nothing relevant selected at all (no wall, no column) -> publish everything; otherwise just the selection
            bool full = selected.Count == 0 && !anyColumnSelected;
            List<Wall> viewWalls = new FilteredElementCollector(doc, view.Id).OfClass(typeof(Wall)).Cast<Wall>().ToList();

            string levelName = (view as ViewPlan)?.GenLevel?.Name ?? view.Name;
            var batch = new PublishBatch
            {
                Source = doc.Title,
                Scope = doc.Title + "|" + levelName,
                FullScope = full,
                RemoveMissing = s.CadRemoveDeleted,
                IncludeCenterline = s.CadCenterline,
                OutlineLayer = s.CadOutlineLayer, CenterLayer = s.CadCenterLayer,
                Zoom = s.CadZoom,
            };

            Transform toShared = s.CadCoordinatesIndex == 1 ? doc.ActiveProjectLocation.GetTotalTransform() : Transform.Identity;
            double k = Ctx.MPerFt * UnitsPerMeter(s.CadUnitsIndex);           // Revit feet -> drawing units
            string salt = s.CadUnitsIndex + "|" + s.CadCoordinatesIndex;
            double tol = 0.0005 * UnitsPerMeter(s.CadUnitsIndex);             // 0.5 mm: points closer than this are the same point

            // ---- footprints and location lines of every wall we might publish ------------------------------------------------------
            var items = new List<Item>();
            foreach (Wall w in s.CadMergeWalls || full ? viewWalls : selected)
            {
                List<XYZ> foot = Footprint(w) ?? RectangleFromLocation(w);
                double[] poly = foot != null && foot.Count >= 3 ? Flatten(foot, toShared, k, ensureCounterClockwise: true) : new double[0];
                var lc = w.Location as LocationCurve;
                double[] center = lc != null && lc.Curve != null ? Flatten(lc.Curve.Tessellate().ToList(), toShared, k, ensureCounterClockwise: false) : new double[0];
                if (poly.Length < 6 && center.Length < 4) continue;
                items.Add(new Item { W = w, Poly = poly, Center = center });
            }

            // ---- which walls form one group ---------------------------------------------------------------------------------------------
            int[] group = new int[items.Count];
            if (s.CadMergeWalls)
            {
                var withPoly = Enumerable.Range(0, items.Count).Where(i => items[i].Poly.Length >= 6).ToList();
                int[] g = PolygonUnion.Cluster(withPoly.Select(i => items[i].Poly).ToList(), tol);
                int next = g.Length == 0 ? 0 : g.Max() + 1;
                for (int i = 0; i < items.Count; i++) group[i] = -1;
                for (int n = 0; n < withPoly.Count; n++) group[withPoly[n]] = g[n];
                for (int i = 0; i < items.Count; i++) if (group[i] < 0) group[i] = next++;
            }
            else for (int i = 0; i < items.Count; i++) group[i] = i;

            HashSet<int> wanted = null;                                              // groups to publish (selection: the ones it touches)
            var selIds = new HashSet<string>(selected.Select(w => w.UniqueId));
            if (!full) wanted = new HashSet<int>(Enumerable.Range(0, items.Count).Where(i => selIds.Contains(items[i].W.UniqueId)).Select(i => group[i]));

            int groups = 0, walls = 0, merged = 0;
            foreach (var members in Enumerable.Range(0, items.Count).GroupBy(i => group[i]))
            {
                if (wanted != null && !wanted.Contains(members.Key)) continue;
                var idx = members.ToList();
                walls += idx.Count; groups++;
                List<double[]> loops = null;
                if (idx.Count > 1 && s.CadMergeWalls)
                {
                    var polys = idx.Where(i => items[i].Poly.Length >= 6).Select(i => items[i].Poly).ToList();
                    loops = polys.Count > 1 ? PolygonUnion.Union(polys, tol) : null;
                    if (loops != null && loops.Count == 0) loops = null;
                }

                if (loops != null)
                {
                    merged++;
                    string gid = "grp:" + idx.Select(i => items[i].W.UniqueId).OrderBy(x => x, StringComparer.Ordinal).First();
                    batch.Elements.Add(new CadElement
                    {
                        Id = gid, Kind = "wallgroup", Name = idx.Count + " joined walls", Level = levelName,
                        Outline = loops[0], Loops = loops,
                        Hash = CadHash.Compute(salt + "|grp", loops.ToArray()),
                    });
                    if (s.CadCenterline)
                        foreach (int i in idx)
                            if (items[i].Center.Length >= 4)
                                batch.Elements.Add(new CadElement { Id = items[i].W.UniqueId, Kind = "wall", Name = items[i].W.Name, Level = levelName,
                                                                    Center = items[i].Center, Hash = CadHash.Compute(salt + "|c", items[i].Center) });
                }
                else
                {
                    foreach (int i in idx)
                    {
                        var it = items[i];
                        batch.Elements.Add(new CadElement
                        {
                            Id = it.W.UniqueId, Kind = "wall", Name = it.W.Name, Level = levelName, Thickness = it.W.Width * k,
                            Outline = it.Poly, Center = it.Center, Hash = CadHash.Compute(salt, it.Poly, it.Center),
                        });
                    }
                }
            }

            var selectedColumns = selectedElements.OfType<FamilyInstance>()
                .Where(fi => fi.Category != null && fi.Category.Id.Value == (long)BuiltInCategory.OST_StructuralColumns).ToList();
            AddColumns(doc, view, selectedColumns, s, batch, toShared, k, salt, levelName, out int colCount);

            string wallsText = "wall" + (walls != 1 ? "s" : "") + (full ? " in the active view" : " (the selection and the walls it touches)") +
                                (merged > 0 ? ", " + merged + " merged group" + (merged != 1 ? "s" : "") : "");
            what = colCount > 0 ? wallsText + ", " + colCount + " column" + (colCount != 1 ? "s" : "") : wallsText;
            return batch;
        }

        private sealed class Item { public Wall W; public double[] Poly, Center; }

        private static double[] Flatten(List<XYZ> pts, Transform t, double k, bool ensureCounterClockwise)
        {
            var xy = new List<double[]>();
            foreach (XYZ p in pts)
            {
                XYZ q = t.OfPoint(p);
                var pt = new[] { q.X * k, q.Y * k };
                if (xy.Count > 0 && Math.Abs(xy[xy.Count - 1][0] - pt[0]) < 1e-6 && Math.Abs(xy[xy.Count - 1][1] - pt[1]) < 1e-6) continue;   // repeated point
                xy.Add(pt);
            }
            if (ensureCounterClockwise && xy.Count > 2)
            {
                if (Math.Abs(xy[0][0] - xy[xy.Count - 1][0]) < 1e-6 && Math.Abs(xy[0][1] - xy[xy.Count - 1][1]) < 1e-6) xy.RemoveAt(xy.Count - 1);   // closing point
                double area = 0;
                for (int i = 0; i < xy.Count; i++) { var a = xy[i]; var b = xy[(i + 1) % xy.Count]; area += a[0] * b[1] - b[0] * a[1]; }
                if (area < 0) xy.Reverse();
            }
            var flat = new double[xy.Count * 2];
            for (int i = 0; i < xy.Count; i++) { flat[2 * i] = xy[i][0]; flat[2 * i + 1] = xy[i][1]; }
            return flat;
        }

        // ---- the wall's real plan footprint: the outer loop of its lowest downward-facing planar face ------------------------------
        private static List<XYZ> Footprint(Element w)
        {
            try
            {
                var opt = new Options { DetailLevel = ViewDetailLevel.Medium, ComputeReferences = false };
                GeometryElement ge = w.get_Geometry(opt);
                if (ge == null) return null;
                var solids = new List<Solid>();
                CollectSolids(ge, solids);

                PlanarFace bottom = null; double bestZ = double.MaxValue, bestArea = 0;
                foreach (Solid so in solids)
                    foreach (Face f in so.Faces)
                    {
                        var pf = f as PlanarFace;
                        if (pf == null || pf.FaceNormal.Z > -0.9) continue;
                        double z = pf.Origin.Z, area = pf.Area;
                        if (z < bestZ - 1e-6 || (Math.Abs(z - bestZ) <= 1e-6 && area > bestArea)) { bottom = pf; bestZ = z; bestArea = area; }
                    }
                if (bottom == null) return null;

                // the outer boundary = the loop enclosing the largest area (the others are holes)
                List<XYZ> bestLoop = null; double bestLoopArea = -1;
                foreach (CurveLoop loop in bottom.GetEdgesAsCurveLoops())
                {
                    var pts = new List<XYZ>();
                    foreach (Curve c in loop)
                        foreach (XYZ p in c.Tessellate())
                            if (pts.Count == 0 || pts[pts.Count - 1].DistanceTo(p) > 1e-9) pts.Add(p);
                    if (pts.Count < 3) continue;
                    double a = 0;
                    for (int i = 0; i < pts.Count; i++) { XYZ p0 = pts[i], p1 = pts[(i + 1) % pts.Count]; a += p0.X * p1.Y - p1.X * p0.Y; }
                    a = Math.Abs(a) / 2;
                    if (a > bestLoopArea) { bestLoopArea = a; bestLoop = pts; }
                }
                return bestLoop;
            }
            catch { return null; }
        }

        private static void CollectSolids(GeometryElement ge, List<Solid> into)
        {
            foreach (GeometryObject go in ge)
            {
                var solid = go as Solid;
                if (solid != null) { if (solid.Volume > 1e-9 && solid.Faces.Size > 0) into.Add(solid); continue; }
                var inst = go as GeometryInstance;
                if (inst != null) CollectSolids(inst.GetInstanceGeometry(), into);
            }
        }

        /// <summary>Fallback when the geometry is unavailable: a rectangle from the location line and the wall width.</summary>
        private static List<XYZ> RectangleFromLocation(Wall w)
        {
            var lc = w.Location as LocationCurve;
            if (lc == null) return null;
            XYZ a = lc.Curve.GetEndPoint(0), b = lc.Curve.GetEndPoint(1);
            double dx = b.X - a.X, dy = b.Y - a.Y, len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-9) return null;
            double nx = -dy / len * w.Width / 2, ny = dx / len * w.Width / 2;
            return new List<XYZ> { new XYZ(a.X - nx, a.Y - ny, a.Z), new XYZ(b.X - nx, b.Y - ny, b.Z), new XYZ(b.X + nx, b.Y + ny, b.Z), new XYZ(a.X + nx, a.Y + ny, a.Z) };
        }

        /// <summary>Fallback when the geometry is unavailable: a rectangle (or many-sided circle) reconstructed from the
        /// column's own Width/Depth or Diameter parameters and its placement point and rotation.</summary>
        private static List<XYZ> RectangleFromColumn(FamilyInstance fi)
        {
            var lp = fi.Location as LocationPoint;
            if (lp == null) return null;
            XYZ p = lp.Point; double rot = lp.Rotation;
            FamilySymbol sym = fi.Symbol;
            Parameter dia = ColumnBuilder.DiameterParam(sym);
            if (dia != null)
            {
                double r = dia.AsDouble() / 2;
                var pts = new List<XYZ>();
                for (int i = 0; i < 32; i++) { double a = 2 * Math.PI * i / 32; pts.Add(new XYZ(p.X + r * Math.Cos(a), p.Y + r * Math.Sin(a), p.Z)); }
                return pts;
            }
            Parameter wp = ColumnBuilder.WidthParam(sym), dp = ColumnBuilder.DepthParam(sym);
            if (wp == null || dp == null) return null;
            double w2 = wp.AsDouble() / 2, d2 = dp.AsDouble() / 2, c = Math.Cos(rot), s2 = Math.Sin(rot);
            XYZ U = new XYZ(c, s2, 0), V = new XYZ(-s2, c, 0);
            return new List<XYZ> { p - U * w2 - V * d2, p + U * w2 - V * d2, p + U * w2 + V * d2, p - U * w2 + V * d2 };
        }

        /// <summary>Structural columns in the view (or the selection). Each is published on its own - columns are not merged
        /// with each other or with walls, matching how they are normally drawn in CAD (a distinct symbol, not fused linework).</summary>
        public static void AddColumns(Document doc, View view, List<FamilyInstance> selectedColumns, Settings s, PublishBatch batch,
                                      Transform toShared, double k, string salt, string levelName, out int count)
        {
            count = 0;
            if (!s.CadIncludeColumns) return;
            List<FamilyInstance> cols = batch.FullScope
                ? new FilteredElementCollector(doc, view.Id).OfCategory(BuiltInCategory.OST_StructuralColumns)
                      .OfClass(typeof(FamilyInstance)).Cast<FamilyInstance>().ToList()
                : selectedColumns;

            foreach (FamilyInstance fi in cols)
            {
                List<XYZ> foot = Footprint(fi) ?? RectangleFromColumn(fi);
                if (foot == null || foot.Count < 3) continue;
                double[] poly = Flatten(foot, toShared, k, ensureCounterClockwise: true);
                if (poly.Length < 6) continue;
                batch.Elements.Add(new CadElement
                {
                    Id = fi.UniqueId, Kind = "column", Name = fi.Symbol != null ? fi.Symbol.Family.Name + " : " + fi.Symbol.Name : fi.Name,
                    Level = levelName, Layer = s.CadColumnLayer, Outline = poly, Hash = CadHash.Compute(salt + "|col", poly),
                });
                count++;
            }
        }
    }
}
