using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Cad
{
    public struct CadEntityRef
    {
        public string Handle, RevitId, Role, Hash;
    }

    /// <summary>What the sync logic needs from a drawing. The AutoCAD plugin implements this on a real Database; tests use an in-memory one.</summary>
    public interface ICadDrawing
    {
        string Name { get; }
        void EnsureLayer(string name, short colorIndex);
        /// <summary>Only entities THIS connector created, in this scope. Anything the user drew is never returned.</summary>
        List<CadEntityRef> FindByScope(string scope);
        string Create(string scope, string revitId, string role, string hash, string layer, double elevation, double[] xy, bool closed);
        void Update(string handle, string hash, string layer, double elevation, double[] xy, bool closed);
        /// <summary>Inserts a block reference for a door or window ("blockKind" is "door" or "window" - which actual block
        /// definition that maps to is entirely the drawing's own concern, e.g. a template the person picked in AutoCAD).</summary>
        string CreateBlock(string scope, string revitId, string role, string hash, string layer, double elevation,
                            double insertX, double insertY, double rotation, string blockKind, double width, double height);
        void UpdateBlock(string handle, string hash, string layer, double elevation, double insertX, double insertY, double rotation, double width, double height);
        void Delete(string handle);
        void ZoomExtents();
    }

    public static class CadSyncEngine
    {
        public const string RoleOutline = "outline", RoleCenter = "center", RoleBlock = "block";

        /// <summary>Create what is new, update what changed, leave the rest, remove what was deleted in Revit.</summary>
        public static PublishResult Apply(PublishBatch b, ICadDrawing d)
        {
            var res = new PublishResult { Ok = true, Drawing = d.Name };
            var errors = new List<string>();

            d.EnsureLayer(b.OutlineLayer, b.OutlineColor);
            foreach (var layer in b.Elements.Select(e => e.Layer).Where(l => !string.IsNullOrEmpty(l)).Distinct())
                d.EnsureLayer(layer, b.OutlineColor);              // any per-element layer override (e.g. columns) also gets created
            if (b.IncludeCenterline) d.EnsureLayer(b.CenterLayer, b.CenterColor);

            var existing = new Dictionary<string, CadEntityRef>();
            var extras = new List<CadEntityRef>();              // duplicates of the same key (should not happen, but clean up)
            foreach (var e in d.FindByScope(b.Scope))
            {
                string key = e.RevitId + "|" + e.Role;
                if (existing.ContainsKey(key)) extras.Add(e); else existing[key] = e;
            }
            int hadBefore = existing.Count;
            var wanted = new HashSet<string>();
            var idsInBatch = new HashSet<string>(b.Elements.Select(x => x.Id));

            void Upsert(CadElement el, string role, string layer, double[] xy, bool closed)
            {
                string key = el.Id + "|" + role;
                wanted.Add(key);
                string hash = el.Hash + "|" + role + "|" + layer;
                try
                {
                    if (existing.TryGetValue(key, out var old))
                    {
                        if (old.Hash == hash) res.Unchanged++;
                        else { d.Update(old.Handle, hash, layer, el.Elevation, xy, closed); res.Updated++; }
                    }
                    else { d.Create(b.Scope, el.Id, role, hash, layer, el.Elevation, xy, closed); res.Created++; }
                }
                catch (Exception ex) { res.Errors++; if (errors.Count < 3) errors.Add(ex.Message); }
            }

            void UpsertBlock(CadElement el)
            {
                string key = el.Id + "|" + RoleBlock;
                wanted.Add(key);
                string layer = string.IsNullOrEmpty(el.Layer) ? b.OutlineLayer : el.Layer;
                string hash = el.Hash + "|" + RoleBlock + "|" + layer;
                try
                {
                    if (existing.TryGetValue(key, out var old))
                    {
                        if (old.Hash == hash) res.Unchanged++;
                        else { d.UpdateBlock(old.Handle, hash, layer, el.Elevation, el.InsertX, el.InsertY, el.Rotation, el.Width, el.Height); res.Updated++; }
                    }
                    else { d.CreateBlock(b.Scope, el.Id, RoleBlock, hash, layer, el.Elevation, el.InsertX, el.InsertY, el.Rotation, el.Kind, el.Width, el.Height); res.Created++; }
                }
                catch (Exception ex) { res.Errors++; if (errors.Count < 3) errors.Add(ex.Message); }
            }

            foreach (var el in b.Elements)
            {
                if (el.Kind == "door" || el.Kind == "window") { UpsertBlock(el); continue; }
                // an outline can have several closed loops (the outer boundary, then holes such as a room enclosed by walls)
                var loops = el.AllLoops();
                for (int i = 0; i < loops.Count; i++)
                {
                    string layer = string.IsNullOrEmpty(el.Layer) ? b.OutlineLayer : el.Layer;
                    if (loops[i].Length >= 6) Upsert(el, i == 0 ? RoleOutline : "loop" + i, layer, loops[i], true);
                }
                if (b.IncludeCenterline && el.Center.Length >= 4) Upsert(el, RoleCenter, b.CenterLayer, el.Center, false);
            }

            foreach (var e in extras) TryDelete(d, e, res, errors);
            foreach (var kv in existing)
            {
                if (wanted.Contains(kv.Key)) continue;
                bool inBatch = idsInBatch.Contains(kv.Value.RevitId);
                // a wall that IS in the batch but whose centerline is no longer wanted: remove that centerline
                // a wall that is NOT in the batch: only remove if the batch is the complete set for the scope
                if (inBatch || (b.FullScope && b.RemoveMissing)) TryDelete(d, kv.Value, res, errors);
            }

            if (b.Zoom && hadBefore == 0 && res.Created > 0)
            {
                try { d.ZoomExtents(); } catch { }
            }

            res.Message = "Drawing '" + d.Name + "'";
            if (res.Errors > 0) { res.Message += " - " + res.Errors + " element(s) failed: " + string.Join("; ", errors); }
            return res;
        }

        private static void TryDelete(ICadDrawing d, CadEntityRef e, PublishResult res, List<string> errors)
        {
            try { d.Delete(e.Handle); res.Deleted++; }
            catch (Exception ex) { res.Errors++; if (errors.Count < 3) errors.Add(ex.Message); }
        }
    }
}
