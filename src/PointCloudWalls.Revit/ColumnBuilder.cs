using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Structure;
using PointCloudWalls.Core;

namespace PointCloudWalls.Revit
{
    internal static class ColumnBuilder
    {
        public static List<FamilySymbol> ColumnTypes(Document doc) =>
            new FilteredElementCollector(doc).OfClass(typeof(FamilySymbol))
                .OfCategory(BuiltInCategory.OST_StructuralColumns).Cast<FamilySymbol>()
                .OrderBy(t => t.Family.Name + " : " + t.Name, StringComparer.CurrentCultureIgnoreCase).ToList();

        // Structural column families name their cross-section dimensions inconsistently between vendors and regions:
        // Revit's US metric/imperial families use "Width"/"Depth", but many European/international ones (following
        // engineering notation) use "b"/"h" (breadth/height), and round families vary between "Diameter", "D", "d" or,
        // in some, "b" reused as the single dimension. Every name in each list below is tried, in order, so whichever
        // one the loaded family actually uses is found.
        private static readonly string[] WidthNames = { "Width", "b", "B", "bf", "Bf", "BF" };
        private static readonly string[] DepthNames = { "Depth", "Height", "h", "H", "hf", "Hf", "HF" };
        private static readonly string[] DiameterNames = { "Diameter", "Dia", "Diam", "D", "d", "b", "B" };

        public static Parameter WidthParam(FamilySymbol sym) => FindParam(sym, WidthNames);
        public static Parameter DepthParam(FamilySymbol sym) => FindParam(sym, DepthNames);
        public static Parameter DiameterParam(FamilySymbol sym) => FindParam(sym, DiameterNames);

        // A family with distinct width AND depth parameters is treated as rectangular even if one of them (e.g. "b")
        // also appears in the diameter alias list; a round family only has ONE of that pair, if any.
        public static bool IsRectangular(FamilySymbol sym) => WidthParam(sym) != null && DepthParam(sym) != null && !ReferenceEquals(WidthParam(sym), DepthParam(sym));
        public static bool IsRound(FamilySymbol sym) => !IsRectangular(sym) && DiameterParam(sym) != null;

        private static Parameter FindParam(FamilySymbol sym, string[] names)
        {
            foreach (var n in names) { var p = Param(sym, n); if (p != null) return p; }
            return null;
        }

        private static Parameter Param(FamilySymbol sym, string name) =>
            sym.Parameters.Cast<Parameter>().FirstOrDefault(p => p.Definition != null && string.Equals(p.Definition.Name, name, StringComparison.OrdinalIgnoreCase));

        /// <summary>
        /// The column type for a fitted shape, built from the type the user chose. An earlier copy of that type (or the type
        /// itself) within snapM of the measured size is reused; otherwise the chosen type is DUPLICATED and its Width/Depth
        /// (or Diameter) parameter is set to the measured size.
        /// </summary>
        public static FamilySymbol GetOrCreateType(Document doc, FamilySymbol template, ColumnFitResult r, double snapFt, out string note)
        {
            bool round = r.Shape == ColumnShape.Round;
            if (round && !IsRound(template)) { note = ""; return null; }
            if (!round && !IsRectangular(template)) { note = ""; return null; }

            string prefix = template.Family.Name + " " + template.Name + " PCW ";
            var family = ColumnTypes(doc).Where(t => t.Id.Equals(template.Id) || t.Name.StartsWith(prefix, StringComparison.Ordinal)).ToList();

            FamilySymbol best = null; double bd = double.MaxValue;
            foreach (var t in family)
            {
                double dd = round ? Math.Abs(GetLen(DiameterParam(t)) - r.Diameter / Ctx.MPerFt)
                                   : Math.Max(Math.Abs(GetLen(WidthParam(t)) - r.Width / Ctx.MPerFt), Math.Abs(GetLen(DepthParam(t)) - r.Depth / Ctx.MPerFt));
                if (dd < bd) { bd = dd; best = t; }
            }
            if (best != null && bd <= snapFt) { note = "existing type '" + best.Name + "'"; Activate(best); return best; }

            string name = round
                ? prefix + Math.Round(r.Diameter * 1000 / 5) * 5 + "mm dia"
                : prefix + Math.Round(r.Width * 1000 / 5) * 5 + "x" + Math.Round(r.Depth * 1000 / 5) * 5 + "mm";
            var byName = ColumnTypes(doc).FirstOrDefault(t => t.Name == name);
            if (byName != null) { note = "type '" + name + "'"; Activate(byName); return byName; }

            var nt = (FamilySymbol)template.Duplicate(name);
            Activate(nt);
            if (round) SetLen(DiameterParam(nt), r.Diameter / Ctx.MPerFt);
            else { SetLen(WidthParam(nt), r.Width / Ctx.MPerFt); SetLen(DepthParam(nt), r.Depth / Ctx.MPerFt); }
            note = "NEW type '" + name + "' (duplicated from '" + template.Family.Name + " : " + template.Name + "')";
            return nt;
        }

        private static void Activate(FamilySymbol s) { if (!s.IsActive) s.Activate(); }
        private static double GetLen(Parameter p) => p != null ? p.AsDouble() : 0;
        private static void SetLen(Parameter p, double ft) { if (p != null && !p.IsReadOnly) p.Set(ft); }

        /// <summary>Places the column instance and applies the base/top level, offsets and rotation.</summary>
        public static FamilyInstance CreateColumn(Document doc, WallSetup setup, Level baseLevel, FamilySymbol sym, XYZ location, double angle, double widthFt, double depthFt)
        {
            FamilyInstance fi = doc.Create.NewFamilyInstance(location, sym, baseLevel, StructuralType.Column);
            if (fi == null) return null;

            Parameter bo = fi.get_Parameter(BuiltInParameter.FAMILY_BASE_LEVEL_OFFSET_PARAM);
            if (bo != null && !bo.IsReadOnly) bo.Set(setup.BaseOffsetFt);

            if (setup.HeightMode == 1 && setup.TopLevelId != null)
            {
                Parameter tl = fi.get_Parameter(BuiltInParameter.FAMILY_TOP_LEVEL_PARAM);
                if (tl != null && !tl.IsReadOnly) tl.Set(setup.TopLevelId);
                Parameter to = fi.get_Parameter(BuiltInParameter.FAMILY_TOP_LEVEL_OFFSET_PARAM);
                if (to != null && !to.IsReadOnly) to.Set(setup.TopOffsetFt);
            }
            else
            {
                // no top level chosen: leave it unconnected and set an explicit height if the family exposes one
                Parameter h = fi.LookupParameter("Column Height") ?? fi.LookupParameter("Height");
                if (h != null && !h.IsReadOnly) h.Set(setup.HeightFt);
            }

            // Revit families do not agree on which of their own local axes (at zero rotation) the "Width" parameter
            // actually drives - some author it along local X, some along local Y. Rotating by the fitted angle as if
            // Width were always local X puts the column 90 degrees off for any family that does it the other way round.
            // Instead of assuming, check the instance's OWN placed geometry (at zero rotation, right after creation) to
            // see which of its axes actually measures out to the fitted Width, and rotate accordingly.
            if (Math.Abs(angle) > 1e-9 && widthFt > 0 && depthFt > 0)
            {
                double rot = angle;
                try
                {
                    // a just-created element's geometry is not guaranteed to be available until the document regenerates -
                    // without this, get_BoundingBox can return null (or stale data), which silently skips the axis check
                    // below and leaves every column exactly as un-corrected as before this check was added
                    doc.Regenerate();
                    BoundingBoxXYZ bb = fi.get_BoundingBox(null);
                    if (bb != null)
                    {
                        double ex = bb.Max.X - bb.Min.X, ey = bb.Max.Y - bb.Min.Y;
                        bool xIsWidth = Math.Abs(ex - widthFt) <= Math.Abs(ex - depthFt);
                        if (!xIsWidth) rot += Math.PI / 2;
                    }
                }
                catch { /* fall back to the un-adjusted angle rather than fail the whole placement */ }
                Line axis = Line.CreateBound(location, location + XYZ.BasisZ);
                ElementTransformUtils.RotateElement(doc, fi.Id, axis, rot);
            }

            DisableAnalyticalModel(fi);
            return fi;
        }

        /// <summary>Structural columns show an "analytical model" stick line through their length by default; this looks like
        /// an unwanted reference line next to the (already visible) physical column. Looked up by NAME, not BuiltInParameter,
        /// since the exact parameter is not guaranteed the same across every Revit version - a missing parameter is a silent
        /// no-op here, never a build-time risk.</summary>
        private static void DisableAnalyticalModel(FamilyInstance fi)
        {
            Parameter p = fi.LookupParameter("Enable Analytical Model") ?? fi.LookupParameter("Analytical Model");
            if (p != null && !p.IsReadOnly) { try { p.Set(0); } catch { } }
        }
    }
}
