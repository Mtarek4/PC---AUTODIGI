using System;
using System.Linq;
using Autodesk.Revit.DB;

namespace PointCloudWalls.Revit
{
    internal static class WallBuilder
    {
        /// <summary>
        /// The wall type for a measured thickness, made from the type the user chose. An earlier copy of that type (or the type itself)
        /// within snapFt of the thickness is reused; otherwise the chosen type is DUPLICATED and its main structural layer is resized so the
        /// total thickness matches.
        /// </summary>
        public static WallType GetOrCreateType(Document doc, WallType template, double widthFt, double snapFt, out string note)
        {
            string prefix = template.Name + " PCW ";
            var family = new FilteredElementCollector(doc).OfClass(typeof(WallType)).Cast<WallType>()
                             .Where(t => t.Kind == WallKind.Basic && (t.Id.Equals(template.Id) || t.Name.StartsWith(prefix, StringComparison.Ordinal))).ToList();

            WallType best = null; double bd = double.MaxValue;
            foreach (var t in family)
            {
                double dd = Math.Abs(t.Width - widthFt);
                if (dd < bd) { bd = dd; best = t; }
            }
            if (best != null && bd <= snapFt) { note = "existing type '" + best.Name + "'"; return best; }

            double mm = Math.Round(widthFt * 304.8 / 5.0) * 5.0;              // round to 5 mm
            string name = prefix + mm.ToString("0") + "mm";
            var byName = new FilteredElementCollector(doc).OfClass(typeof(WallType)).Cast<WallType>().FirstOrDefault(t => t.Name == name);
            if (byName != null) { note = "type '" + name + "'"; return byName; }

            var nt = (WallType)template.Duplicate(name);
            double achieved;
            bool exact = SetTotalWidth(nt, mm / 304.8, out achieved);
            note = "NEW type '" + name + "' (duplicated from '" + template.Name + "'" +
                   (exact ? "" : "; the type could only be set to " + (achieved * 304.8).ToString("0") + " mm") + ")";
            return nt;
        }

        /// <summary>Resize the main structural layer (the widest structural one) so the wall type totals targetFt.</summary>
        private static bool SetTotalWidth(WallType t, double targetFt, out double achievedFt)
        {
            CompoundStructure cs = t.GetCompoundStructure();
            if (cs == null || cs.LayerCount == 0) { achievedFt = t.Width; return Math.Abs(t.Width - targetFt) < 1e-6; }
            double total = cs.GetWidth();
            int idx = 0; double bestScore = double.MinValue;
            for (int i = 0; i < cs.LayerCount; i++)
            {
                double score = cs.GetLayerWidth(i) + (cs.GetLayerFunction(i) == MaterialFunctionAssignment.Structure ? 1000 : 0);
                if (score > bestScore) { bestScore = score; idx = i; }
            }
            double minLayer = 1.0 / 304.8;                                     // a layer cannot be thinner than 1 mm
            double newWidth = Math.Max(minLayer, cs.GetLayerWidth(idx) + (targetFt - total));
            cs.SetLayerWidth(idx, newWidth);
            t.SetCompoundStructure(cs);
            achievedFt = t.GetCompoundStructure().GetWidth();
            return Math.Abs(achievedFt - targetFt) < 1e-5;
        }

        /// <summary>The height (feet) the setup gives a wall on this base level: fixed, or up to the chosen top level.</summary>
        public static double HeightFt(Document doc, WallSetup setup, Level baseLevel)
        {
            Level top = setup.HeightMode == 1 && setup.TopLevelId != null ? doc.GetElement(setup.TopLevelId) as Level : null;
            double height = top != null ? top.Elevation + setup.TopOffsetFt - baseLevel.Elevation - setup.BaseOffsetFt : setup.HeightFt;
            if (height < 0.1) height = Math.Max(0.1, setup.HeightFt);
            return height;
        }

        /// <summary>Creates the wall on the chosen base level with the chosen height (fixed, or up to a level).</summary>
        public static Wall CreateWall(Document doc, WallSetup setup, Level baseLevel, WallType wt, XYZ a, XYZ b)
        {
            double z = baseLevel.Elevation;
            Line line = Line.CreateBound(new XYZ(a.X, a.Y, z), new XYZ(b.X, b.Y, z));
            Level top = setup.HeightMode == 1 && setup.TopLevelId != null ? doc.GetElement(setup.TopLevelId) as Level : null;
            double height = HeightFt(doc, setup, baseLevel);
            Wall w = Wall.Create(doc, line, wt.Id, baseLevel.Id, height, setup.BaseOffsetFt, false, false);
            if (w != null && top != null)
            {
                Parameter tc = w.get_Parameter(BuiltInParameter.WALL_HEIGHT_TYPE);         // Top Constraint
                if (tc != null && !tc.IsReadOnly) tc.Set(top.Id);
                Parameter to = w.get_Parameter(BuiltInParameter.WALL_TOP_OFFSET);
                if (to != null && !to.IsReadOnly) to.Set(setup.TopOffsetFt);
            }
            return w;
        }

        /// <summary>Detail curve in the view, coloured via a per-view override. Tries the level Z, then Z=0.</summary>
        public static void Detail(Document doc, View view, XYZ a, XYZ b, Color color, int weight)
        {
            foreach (double z in new[] { a.Z, 0.0 })
            {
                try
                {
                    Line ln = Line.CreateBound(new XYZ(a.X, a.Y, z), new XYZ(b.X, b.Y, z));
                    DetailCurve dc = doc.Create.NewDetailCurve(view, ln);
                    var ogs = new OverrideGraphicSettings();
                    ogs.SetProjectionLineColor(color);
                    ogs.SetProjectionLineWeight(weight);
                    view.SetElementOverrides(dc.Id, ogs);
                    return;
                }
                catch { /* try the other Z */ }
            }
        }
    }
}
