using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Structure;

namespace PointCloudWalls.Revit
{
    internal enum OpeningCategory { Door, Window }

    /// <summary>Family/type duplication and placement for doors and windows - mirrors ColumnBuilder's approach.</summary>
    internal static class DoorWindowBuilder
    {
        public static List<FamilySymbol> Types(Document doc, OpeningCategory cat)
        {
            var bic = cat == OpeningCategory.Door ? BuiltInCategory.OST_Doors : BuiltInCategory.OST_Windows;
            return new FilteredElementCollector(doc).OfClass(typeof(FamilySymbol)).OfCategory(bic).Cast<FamilySymbol>()
                       .OrderBy(t => t.Family.Name + " : " + t.Name, StringComparer.CurrentCultureIgnoreCase).ToList();
        }

        // Width/Height are far more consistently named in door/window families than in structural columns, but a small
        // alias list costs nothing and protects against the odd family that doesn't follow convention. An explicit
        // override (chosen in Door/Window Setup) always wins over the alias list when one is set.
        private static readonly string[] WidthNames = { "Width", "W" };
        private static readonly string[] HeightNames = { "Height", "H" };

        public static Parameter WidthParam(FamilySymbol s, string overrideName = null) => FindParam(s, WidthNames, overrideName);
        public static Parameter HeightParam(FamilySymbol s, string overrideName = null) => FindParam(s, HeightNames, overrideName);
        private static Parameter FindParam(FamilySymbol s, string[] names, string overrideName)
        {
            if (!string.IsNullOrEmpty(overrideName)) { var op = Param(s, overrideName); if (op != null) return op; }
            foreach (var n in names) { var p = Param(s, n); if (p != null) return p; }
            return null;
        }
        private static Parameter Param(FamilySymbol s, string name) =>
            s.Parameters.Cast<Parameter>().FirstOrDefault(p => p.Definition != null && string.Equals(p.Definition.Name, name, StringComparison.OrdinalIgnoreCase));

        /// <summary>Every numeric (length-like) parameter this family/type exposes, for the Door/Window Setup UI to let
        /// someone pick which one is really Width and which is really Height on their specific family.</summary>
        public static List<string> NumericParamNames(FamilySymbol s) =>
            s.Parameters.Cast<Parameter>().Where(p => p.Definition != null && p.StorageType == StorageType.Double)
                .Select(p => p.Definition.Name).Distinct().OrderBy(n => n, StringComparer.CurrentCultureIgnoreCase).ToList();

        /// <summary>Finds an existing type within <paramref name="snapFt"/> of the measured size, or duplicates a new one.</summary>
        public static FamilySymbol GetOrCreateType(Document doc, FamilySymbol template, double widthFt, double heightFt, double snapFt,
                                                    string widthParamName, string heightParamName, out string note)
        {
            var family = Types(doc, template.Category.Id.Value == (long)BuiltInCategory.OST_Doors ? OpeningCategory.Door : OpeningCategory.Window)
                             .Where(t => t.Family.Id == template.Family.Id).ToList();
            if (family.Count == 0) family.Add(template);

            FamilySymbol best = null; double bd = double.MaxValue;
            foreach (var t in family)
            {
                var wp = WidthParam(t, widthParamName); var hp = HeightParam(t, heightParamName);
                if (wp == null || hp == null) continue;
                double dd = Math.Max(Math.Abs(wp.AsDouble() - widthFt), Math.Abs(hp.AsDouble() - heightFt));
                if (dd < bd) { bd = dd; best = t; }
            }
            if (best != null && bd <= snapFt) { note = "existing type '" + best.Family.Name + " : " + best.Name + "'"; return best; }

            string mm(double ft) => Math.Round(ft * Ctx.MPerFt * 1000 / 5) * 5 + "";
            string name = template.Family.Name + " PCW " + mm(widthFt) + "x" + mm(heightFt) + "mm";
            var existing = family.FirstOrDefault(t => t.Name == name);
            if (existing != null) { note = "existing type '" + existing.Family.Name + " : " + existing.Name + "'"; return existing; }

            var nt = (FamilySymbol)template.Duplicate(name);
            if (!nt.IsActive) nt.Activate();
            var w2 = WidthParam(nt, widthParamName); var h2 = HeightParam(nt, heightParamName);
            if (w2 != null && !w2.IsReadOnly) w2.Set(widthFt);
            if (h2 != null && !h2.IsReadOnly) h2.Set(heightFt);
            note = "NEW type '" + name + "' (duplicated from '" + template.Family.Name + " : " + template.Name + "')";
            return nt;
        }

        /// <summary>Places the instance hosted on the wall, at the given point along it, with the sill height set for windows.</summary>
        public static FamilyInstance CreateInstance(Document doc, FamilySymbol sym, XYZ location, Wall host, double sillFt, OpeningCategory cat)
        {
            if (!sym.IsActive) sym.Activate();
            // The host-only overload does NOT reliably associate the instance with the WALL'S OWN level - it can default
            // to some other level in the project (e.g. whichever Level 1 the template happens to have), which is only
            // correct by coincidence. If that level's elevation differs from the host wall's real level (a very real
            // possibility - Level 1 is not always at 0), the door or window appears at the wrong height by exactly that
            // difference, even though it may still report as "hosted". Passing the host's OWN level explicitly removes
            // that ambiguity entirely.
            Level hostLevel = doc.GetElement(host.LevelId) as Level;
            FamilyInstance fi = hostLevel != null
                ? doc.Create.NewFamilyInstance(location, sym, host, hostLevel, StructuralType.NonStructural)
                : doc.Create.NewFamilyInstance(location, sym, host, StructuralType.NonStructural);
            if (fi == null) return null;

            // Revit does not always finish establishing a hosted instance's relationship with its host (specifically,
            // the boolean cut that actually opens the wall) purely from the create call - without a regenerate here,
            // the instance can visually sit in the right place while the wall around it stays solid, uncut, exactly as
            // if it were never really hosted at all. The same category of issue showed up earlier in this project with
            // a column's geometry not being ready until an explicit Regenerate() - the fix is the same idea here.
            doc.Regenerate();

            if (cat == OpeningCategory.Window)
            {
                Parameter sh = fi.get_Parameter(BuiltInParameter.INSTANCE_SILL_HEIGHT_PARAM) ?? fi.LookupParameter("Sill Height");
                if (sh != null && !sh.IsReadOnly) sh.Set(sillFt);
            }
            return fi;
        }
    }
}
