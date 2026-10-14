using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace PointCloudWalls.Cad
{
    /// <summary>One Revit element as it is drawn in CAD. Coordinates are already in DRAWING UNITS (mm / cm / m as chosen in Revit).</summary>
    public sealed class CadElement
    {
        public string Id = "";              // Revit UniqueId: the key that lets a re-publish UPDATE instead of duplicating
        public string Kind = "wall";
        public string Name = "";            // e.g. wall type name
        public string Level = "";
        public double Thickness, Height, Elevation;
        public double[] Outline = new double[0];   // x0,y0,x1,y1,... closed polygon (the first loop)
        public List<double[]> Loops = new List<double[]>();   // ALL closed loops of the outline: outer boundary first, then holes / separate pieces
        public double[] Center = new double[0];    // x0,y0,x1,y1,... open polyline
        public string Layer = "";           // per-element layer override; empty = use the batch's OutlineLayer (walls' normal case)
        // door/window ("block") elements only - a point insertion with rotation and size, not an outline (Height above is reused for these too)
        public double InsertX, InsertY, Rotation, Width;
        public string Hash = "";            // changes when the geometry changes

        internal List<double[]> AllLoops() => Loops.Count > 0 ? Loops : (Outline.Length >= 6 ? new List<double[]> { Outline } : new List<double[]>());

        internal Dictionary<string, object> ToDict() => new Dictionary<string, object>
        {
            ["id"] = Id, ["kind"] = Kind, ["name"] = Name, ["level"] = Level,
            ["thickness"] = Thickness, ["height"] = Height, ["elevation"] = Elevation,
            ["outline"] = Outline, ["loops"] = Loops, ["center"] = Center, ["layer"] = Layer,
            ["insertX"] = InsertX, ["insertY"] = InsertY, ["rotation"] = Rotation, ["width"] = Width,
            ["hash"] = Hash,
        };

        internal static CadElement FromDict(IDictionary<string, object> d)
        {
            var e = new CadElement
            {
                Id = Json.Str(d, "id"), Kind = Json.Str(d, "kind", "wall"), Name = Json.Str(d, "name"), Level = Json.Str(d, "level"),
                Thickness = Json.Num(d, "thickness"), Height = Json.Num(d, "height"), Elevation = Json.Num(d, "elevation"),
                Outline = Json.Doubles(d, "outline"), Center = Json.Doubles(d, "center"), Layer = Json.Str(d, "layer"),
                InsertX = Json.Num(d, "insertX"), InsertY = Json.Num(d, "insertY"), Rotation = Json.Num(d, "rotation"),
                Width = Json.Num(d, "width"),
                Hash = Json.Str(d, "hash"),
            };
            foreach (var o in Json.List(d, "loops"))
                if (o is List<object> lo)
                {
                    var arr = new double[lo.Count];
                    for (int i = 0; i < arr.Length; i++) arr[i] = lo[i] is double x ? x : 0;
                    e.Loops.Add(arr);
                }
            return e;
        }
    }

    /// <summary>What Revit sends when you click "Publish to CAD".</summary>
    public sealed class PublishBatch
    {
        public int Version = 1;
        public string Source = "";           // Revit document name
        public string Scope = "";            // what this batch replaces on re-publish (document + level)
        public bool FullScope;               // true = the batch is EVERYTHING in the scope, so walls missing from it were deleted in Revit
        public bool RemoveMissing = true;    // remove CAD entities of walls that no longer exist in Revit (only if FullScope)
        public bool IncludeCenterline = true;
        public string OutlineLayer = "PCW-WALL", CenterLayer = "PCW-WALL-CENTER";
        public short OutlineColor = 7, CenterColor = 4;
        public bool Zoom = true;             // zoom to extents the first time something is drawn
        public List<CadElement> Elements = new List<CadElement>();

        public string ToJson()
        {
            var els = new List<object>(Elements.Count);
            foreach (var e in Elements) els.Add(e.ToDict());
            return Json.Write(new Dictionary<string, object>
            {
                ["type"] = CadMessage.Publish, ["version"] = Version, ["source"] = Source, ["scope"] = Scope,
                ["fullScope"] = FullScope, ["removeMissing"] = RemoveMissing, ["includeCenterline"] = IncludeCenterline,
                ["outlineLayer"] = OutlineLayer, ["centerLayer"] = CenterLayer,
                ["outlineColor"] = (int)OutlineColor, ["centerColor"] = (int)CenterColor, ["zoom"] = Zoom, ["elements"] = els,
            });
        }

        public static PublishBatch FromJson(string json)
        {
            var d = Json.Parse(json) as Dictionary<string, object> ?? throw new FormatException("Not a JSON object");
            var b = new PublishBatch
            {
                Version = Json.Int(d, "version", 1), Source = Json.Str(d, "source"), Scope = Json.Str(d, "scope"),
                FullScope = Json.Bool(d, "fullScope"), RemoveMissing = Json.Bool(d, "removeMissing", true),
                IncludeCenterline = Json.Bool(d, "includeCenterline", true),
                OutlineLayer = Json.Str(d, "outlineLayer", "PCW-WALL"), CenterLayer = Json.Str(d, "centerLayer", "PCW-WALL-CENTER"),
                OutlineColor = (short)Json.Int(d, "outlineColor", 7), CenterColor = (short)Json.Int(d, "centerColor", 4),
                Zoom = Json.Bool(d, "zoom", true),
            };
            foreach (var o in Json.List(d, "elements"))
                if (o is Dictionary<string, object> ed) b.Elements.Add(CadElement.FromDict(ed));
            return b;
        }
    }

    /// <summary>What the AutoCAD side answers.</summary>
    public sealed class PublishResult
    {
        public bool Ok;
        public string Message = "";
        public string Drawing = "";          // name of the AutoCAD drawing that received the walls
        public int Created, Updated, Unchanged, Deleted, Errors;

        public string ToJson() => Json.Write(new Dictionary<string, object>
        {
            ["ok"] = Ok, ["message"] = Message, ["drawing"] = Drawing,
            ["created"] = Created, ["updated"] = Updated, ["unchanged"] = Unchanged, ["deleted"] = Deleted, ["errors"] = Errors,
        });

        public static PublishResult FromJson(string json)
        {
            var d = Json.Parse(json) as Dictionary<string, object> ?? throw new FormatException("Not a JSON object");
            return new PublishResult
            {
                Ok = Json.Bool(d, "ok"), Message = Json.Str(d, "message"), Drawing = Json.Str(d, "drawing"),
                Created = Json.Int(d, "created"), Updated = Json.Int(d, "updated"), Unchanged = Json.Int(d, "unchanged"),
                Deleted = Json.Int(d, "deleted"), Errors = Json.Int(d, "errors"),
            };
        }

        public string Summary() =>
            (Ok ? "" : "FAILED: ") + Message + (Ok ? "  created " + Created + ", updated " + Updated + ", unchanged " + Unchanged + ", deleted " + Deleted : "");
    }

    public static class CadMessage
    {
        public const string Publish = "publish";
        public const string Ping = "ping";

        public static string TypeOf(string json)
        {
            var d = Json.Parse(json) as Dictionary<string, object>;
            return d == null ? "" : Json.Str(d, "type");
        }

        public static string PingJson() => Json.Write(new Dictionary<string, object> { ["type"] = Ping });
    }

    public static class CadHash
    {
        /// <summary>Stable 64-bit FNV-1a hash of a list of coordinate arrays (rounded to 3 decimals) plus a salt.</summary>
        public static string Compute(string salt, params double[][] coords)
        {
            var sb = new StringBuilder(salt);
            foreach (var arr in coords)
            {
                sb.Append('|');
                foreach (double v in arr) sb.Append(Math.Round(v, 3).ToString("0.###", CultureInfo.InvariantCulture)).Append(',');
            }
            ulong h = 14695981039346656037UL;
            foreach (char c in sb.ToString()) { h ^= c; h *= 1099511628211UL; }
            return h.ToString("x16", CultureInfo.InvariantCulture);
        }
    }
}
