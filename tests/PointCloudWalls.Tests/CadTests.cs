using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using PointCloudWalls.Cad;

/// <summary>Tests of the Revit -> AutoCAD connector logic: JSON, the pipe, and the create/update/delete sync. No Revit or AutoCAD needed.</summary>
public static class CadTests
{
    private static int _fail;
    private static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    /// <summary>In-memory stand-in for an AutoCAD drawing, including entities the user drew that we must never touch.</summary>
    private sealed class FakeDrawing : ICadDrawing
    {
        public sealed class Ent
        {
            public string Handle, Scope, RevitId, Role, Hash, Layer; public double Elev; public double[] Xy; public bool Closed; public bool Foreign;
            public bool IsBlock; public double InsertX, InsertY, Rotation, Width, Height; public string BlockKind;
        }
        public readonly Dictionary<string, Ent> Ents = new Dictionary<string, Ent>();
        public readonly Dictionary<string, short> Layers = new Dictionary<string, short>();
        public int Zooms; private int _n;
        public string Name => "Plan-01.dwg";
        public void EnsureLayer(string name, short color) { if (!Layers.ContainsKey(name)) Layers[name] = color; }
        public List<CadEntityRef> FindByScope(string scope) => Ents.Values.Where(e => !e.Foreign && e.Scope == scope)
            .Select(e => new CadEntityRef { Handle = e.Handle, RevitId = e.RevitId, Role = e.Role, Hash = e.Hash }).ToList();
        public string Create(string scope, string id, string role, string hash, string layer, double elev, double[] xy, bool closed)
        { var h = (++_n).ToString("X"); Ents[h] = new Ent { Handle = h, Scope = scope, RevitId = id, Role = role, Hash = hash, Layer = layer, Elev = elev, Xy = xy, Closed = closed }; return h; }
        public void Update(string h, string hash, string layer, double elev, double[] xy, bool closed) { var e = Ents[h]; e.Hash = hash; e.Layer = layer; e.Elev = elev; e.Xy = xy; e.Closed = closed; }
        public string CreateBlock(string scope, string id, string role, string hash, string layer, double elev, double ix, double iy, double rot, string kind, double w, double h2)
        { var h = (++_n).ToString("X"); Ents[h] = new Ent { Handle = h, Scope = scope, RevitId = id, Role = role, Hash = hash, Layer = layer, Elev = elev, IsBlock = true, InsertX = ix, InsertY = iy, Rotation = rot, BlockKind = kind, Width = w, Height = h2 }; return h; }
        public void UpdateBlock(string h, string hash, string layer, double elev, double ix, double iy, double rot, double w, double h2)
        { var e = Ents[h]; e.Hash = hash; e.Layer = layer; e.Elev = elev; e.InsertX = ix; e.InsertY = iy; e.Rotation = rot; e.Width = w; e.Height = h2; }
        public void Delete(string h) { if (!Ents.Remove(h)) throw new InvalidOperationException("no such entity " + h); }
        public void ZoomExtents() => Zooms++;
        public void AddForeign(string layer) { var h = (++_n).ToString("X"); Ents[h] = new Ent { Handle = h, Layer = layer, Foreign = true, Xy = new double[] { 0, 0, 1, 1 } }; }
    }

    private static CadElement Wall(string id, double x, double y, double len, double thk, string hashSalt = "")
    {
        var outline = new[] { x, y, x + len, y, x + len, y + thk, x, y + thk };
        var center = new[] { x, y + thk / 2, x + len, y + thk / 2 };
        return new CadElement { Id = id, Name = "Generic - " + thk, Level = "L1", Thickness = thk, Height = 3000, Outline = outline, Center = center,
                                Hash = CadHash.Compute(hashSalt, outline, center) };
    }

    public static int Run()
    {
        Console.WriteLine("=== JSON");
        var tricky = new Dictionary<string, object>
        {
            ["s"] = "quote \" backslash \\ newline \n tab \t unicode \u00e9\u4e2d\u6587 ctrl \u0001",
            ["n"] = new List<object> { 1.0, -2.5, 1e-7, 123456789.123456, 0.0 }, ["b"] = true, ["z"] = null,
            ["nested"] = new Dictionary<string, object> { ["a"] = new List<object> { new Dictionary<string, object> { ["k"] = "v" } } },
        };
        var back = (Dictionary<string, object>)Json.Parse(Json.Write(tricky));
        Check((string)back["s"] == (string)tricky["s"], "strings survive (quotes, backslash, newline, unicode, control chars)");
        var nums = (List<object>)back["n"];
        Check(nums.Count == 5 && (double)nums[2] == 1e-7 && (double)nums[3] == 123456789.123456, "numbers survive exactly (1e-7, 123456789.123456)");
        Check((bool)back["b"] && back["z"] == null, "bool and null survive");
        Check(Json.Parse("  { \"a\" : [ 1 , 2 ] }  ") is Dictionary<string, object>, "tolerates whitespace");
        bool threw = false; try { Json.Parse("{\"a\":"); } catch (FormatException) { threw = true; }
        Check(threw, "malformed JSON is rejected with an exception, not a crash");

        Console.WriteLine("=== Batch round trip");
        var b0 = new PublishBatch { Source = "Project1.rvt", Scope = "Project1|L1", FullScope = true, Elements = { Wall("w1", 0, 0, 5000, 200), Wall("w2", 0, 3000, 5000, 150) } };
        var b1 = PublishBatch.FromJson(b0.ToJson());
        Check(b1.Elements.Count == 2 && b1.Scope == "Project1|L1" && b1.FullScope && b1.Elements[1].Thickness == 150 && b1.Elements[0].Outline.Length == 8, "publish batch survives JSON");

        Console.WriteLine("=== Sync engine (create / update / leave / delete)");
        var dwg = new FakeDrawing(); dwg.AddForeign("USER-LAYER");
        var batch = new PublishBatch { Scope = "P|L1", FullScope = true, Elements = { Wall("w1", 0, 0, 5000, 200), Wall("w2", 0, 3000, 5000, 150), Wall("w3", 6000, 0, 3000, 200) } };
        var r = CadSyncEngine.Apply(batch, dwg);
        Check(r.Ok && r.Created == 6 && r.Updated == 0 && r.Deleted == 0, "first publish: 3 walls -> 3 outlines + 3 centerlines created (" + r.Created + ")");
        Check(dwg.Layers.ContainsKey("PCW-WALL") && dwg.Layers.ContainsKey("PCW-WALL-CENTER"), "layers created");
        Check(dwg.Zooms == 1, "zoomed to extents once on the first publish");
        var r2 = CadSyncEngine.Apply(batch, dwg);
        Check(r2.Created == 0 && r2.Updated == 0 && r2.Unchanged == 6 && dwg.Ents.Count == 7, "re-publish of the same walls changes NOTHING (no duplicates): unchanged=" + r2.Unchanged);
        Check(dwg.Zooms == 1, "no zoom on re-publish");
        batch.Elements[1] = Wall("w2", 0, 3000, 6500, 150);                       // wall 2 was made longer in Revit
        var r3 = CadSyncEngine.Apply(batch, dwg);
        Check(r3.Updated == 2 && r3.Created == 0 && r3.Unchanged == 4, "a wall changed in Revit -> only that wall is updated (updated=" + r3.Updated + ")");
        Check(dwg.Ents.Values.First(e => !e.Foreign && e.RevitId == "w2" && e.Role == "outline").Xy[2] == 6500, "the CAD geometry really moved");
        var handlesBefore = dwg.Ents.Keys.ToHashSet();
        Apply2(batch, dwg);
        Check(handlesBefore.SetEquals(dwg.Ents.Keys), "updates keep the same entities (handles) - nothing erased and re-drawn");

        batch.Elements.RemoveAt(2);                                                // wall 3 deleted in Revit
        var r4 = CadSyncEngine.Apply(batch, dwg);
        Check(r4.Deleted == 2 && dwg.Ents.Values.Count(e => !e.Foreign) == 4, "wall deleted in Revit (full publish) -> its CAD entities removed (deleted=" + r4.Deleted + ")");
        Check(dwg.Ents.Values.Any(e => e.Foreign), "the user's own CAD entity was NEVER touched");

        var partial = new PublishBatch { Scope = "P|L1", FullScope = false, Elements = { Wall("w1", 0, 0, 5000, 200) } };   // publishing a selection of one wall
        var r5 = CadSyncEngine.Apply(partial, dwg);
        Check(r5.Deleted == 0 && dwg.Ents.Values.Count(e => !e.Foreign) == 4, "publishing a SELECTION never deletes the walls you did not select");

        var other = new PublishBatch { Scope = "P|L2", FullScope = true, Elements = { Wall("x1", 0, 0, 1000, 200) } };
        CadSyncEngine.Apply(other, dwg);
        batch.Elements.Clear(); batch.Elements.Add(Wall("w1", 0, 0, 5000, 200));
        CadSyncEngine.Apply(batch, dwg);
        Check(dwg.Ents.Values.Any(e => e.RevitId == "x1"), "another level's walls are untouched by a publish of this level");

        batch.IncludeCenterline = false;
        var r6 = CadSyncEngine.Apply(batch, dwg);
        Check(r6.Deleted == 1 && !dwg.Ents.Values.Any(e => e.RevitId == "w1" && e.Role == "center"), "switching centerlines off removes the old centerlines");

        Console.WriteLine("=== Merged outlines (several loops per element)");
        var room = new CadElement { Id = "grp:room", Kind = "wallgroup", Loops = new List<double[]> { new double[] { 0, 0, 5, 0, 5, 4, 0, 4 }, new double[] { 0.2, 0.2, 0.2, 3.8, 4.8, 3.8, 4.8, 0.2 } }, Hash = "hRoom" };
        room.Outline = room.Loops[0];
        var dr = new FakeDrawing();
        var rb = new PublishBatch { Scope = "M|L1", FullScope = true, IncludeCenterline = false, Elements = { room } };
        var rr = CadSyncEngine.Apply(rb, dr);
        Check(rr.Created == 2 && dr.Ents.Values.Count(e => e.Role == "outline") == 1 && dr.Ents.Values.Count(e => e.Role == "loop1") == 1, "a room (outer + inner outline) is drawn as 2 closed polylines (created=" + rr.Created + ")");
        Check(PublishBatch.FromJson(rb.ToJson()).Elements[0].Loops.Count == 2, "loops survive JSON");
        Check(CadSyncEngine.Apply(rb, dr).Unchanged == 2, "re-publish: both loops unchanged");
        room.Loops.RemoveAt(1); room.Hash = "hRoom2";
        var rr2 = CadSyncEngine.Apply(rb, dr);
        Check(rr2.Deleted == 1 && rr2.Updated == 1 && dr.Ents.Count == 1, "the inner loop disappears in Revit -> removed in CAD (deleted=" + rr2.Deleted + ")");

        // walls that were separate get joined (grp outline appears, single outlines go), and back
        var dj = new FakeDrawing();
        Func<string, CadElement> centerOnly = id => { var w0 = Wall(id, 0, 0, 3000, 200); return new CadElement { Id = id, Center = w0.Center, Hash = CadHash.Compute("c", w0.Center) }; };
        var singles = new PublishBatch { Scope = "J|L1", FullScope = true, Elements = { Wall("wA", 0, 0, 3000, 200), Wall("wB", 0, 500, 3000, 200) } };
        CadSyncEngine.Apply(singles, dj);
        var joined = new PublishBatch { Scope = "J|L1", FullScope = true, Elements = { new CadElement { Id = "grp:wA", Kind = "wallgroup", Outline = new double[] { 0, 0, 3000, 0, 3000, 700, 0, 700 }, Hash = "hg" }, centerOnly("wA"), centerOnly("wB") } };
        var jr = CadSyncEngine.Apply(joined, dj);
        Check(dj.Ents.Values.Count(e => e.Role == "outline") == 1 && dj.Ents.Values.Any(e => e.RevitId == "grp:wA" && e.Role == "outline") && !dj.Ents.Values.Any(e => e.RevitId == "wA" && e.Role == "outline"),
              "two separate walls become a merged outline: the two single outlines are removed, one merged outline created");
        var jr2 = CadSyncEngine.Apply(singles, dj);
        Check(dj.Ents.Values.Count(e => e.Role == "outline") == 2 && !dj.Ents.Values.Any(e => e.RevitId == "grp:wA"), "and back: separate again -> the merged outline is removed, single outlines return");

        Console.WriteLine("=== Per-element layer override (columns on their own layer, walls unaffected)");
        var dl = new FakeDrawing();
        var mixedBatch = new PublishBatch { Scope = "MX|L1", FullScope = true, IncludeCenterline = false, OutlineLayer = "PCW-WALL" };
        mixedBatch.Elements.Add(new CadElement { Id = "w1", Kind = "wall", Outline = new double[] { 0, 0, 1000, 0, 1000, 200, 0, 200 }, Hash = "hw" });
        mixedBatch.Elements.Add(new CadElement { Id = "c1", Kind = "column", Layer = "PCW-COLUMN", Outline = new double[] { 0, 0, 400, 0, 400, 400, 0, 400 }, Hash = "hc" });
        var mr = CadSyncEngine.Apply(mixedBatch, dl);
        Check(mr.Created == 2, "both wall and column created (created=" + mr.Created + ")");
        Check(dl.Ents.Values.First(e => e.RevitId == "w1").Layer == "PCW-WALL", "wall lands on the batch's default outline layer");
        Check(dl.Ents.Values.First(e => e.RevitId == "c1").Layer == "PCW-COLUMN", "column lands on its OWN per-element layer, not the wall layer");
        Check(dl.Layers.ContainsKey("PCW-COLUMN"), "the column's layer was created automatically");
        var mr2 = CadSyncEngine.Apply(mixedBatch, dl);
        Check(mr2.Unchanged == 2, "re-publish: both stay unchanged on their respective layers");

        Console.WriteLine("=== Door/window block references (create, update, unchanged, delete)");
        var dwl = new FakeDrawing();
        var door1 = new CadElement { Id = "d1", Kind = "door", Layer = "PCW-DOOR", InsertX = 1000, InsertY = 2000, Rotation = 0, Width = 900, Height = 2100, Hash = "hdoor1" };
        var win1 = new CadElement { Id = "w1b", Kind = "window", Layer = "PCW-WINDOW", InsertX = 4000, InsertY = 2000, Rotation = Math.PI / 2, Width = 1200, Height = 1400, Hash = "hwin1" };
        var dwBatch = new PublishBatch { Scope = "DW|L1", FullScope = true, IncludeCenterline = false, OutlineLayer = "PCW-WALL" };
        dwBatch.Elements.Add(door1); dwBatch.Elements.Add(win1);
        var dwr1 = CadSyncEngine.Apply(dwBatch, dwl);
        Check(dwr1.Created == 2, "door and window block references both created");
        var doorEnt = dwl.Ents.Values.First(e => e.RevitId == "d1");
        Check(doorEnt.IsBlock && doorEnt.BlockKind == "door" && doorEnt.Layer == "PCW-DOOR" && Math.Abs(doorEnt.InsertX - 1000) < 1e-9, "door block has correct kind, layer and insertion point");
        var winEnt = dwl.Ents.Values.First(e => e.RevitId == "w1b");
        Check(winEnt.IsBlock && winEnt.BlockKind == "window" && Math.Abs(winEnt.Rotation - Math.PI / 2) < 1e-9, "window block has correct kind and rotation");

        var dwr2 = CadSyncEngine.Apply(dwBatch, dwl);
        Check(dwr2.Unchanged == 2, "re-publish with no change: both stay unchanged");

        door1.Width = 1000; door1.Hash = "hdoor1v2";   // the door was widened - re-detected with a new size
        var dwr3 = CadSyncEngine.Apply(dwBatch, dwl);
        Check(dwr3.Updated == 1 && dwr3.Unchanged == 1, "changing the door's width updates only the door, leaves the window alone");
        Check(Math.Abs(dwl.Ents.Values.First(e => e.RevitId == "d1").Width - 1000) < 1e-9, "the door block's width was actually updated");

        dwBatch.Elements.Remove(door1);
        var dwr4 = CadSyncEngine.Apply(dwBatch, dwl);
        Check(dwr4.Deleted == 1, "removing the door from the batch (deleted in Revit) deletes its block reference");
        Check(!dwl.Ents.Values.Any(e => e.RevitId == "d1") && dwl.Ents.Values.Any(e => e.RevitId == "w1b"), "the window block is untouched by the door's deletion");

        var bad = new PublishBatch { Scope = "P|L3", Elements = { new CadElement { Id = "bad", Outline = new double[] { 0, 0, 1 } , Hash = "h"}, Wall("ok", 0, 0, 1000, 100) } };
        var r7 = CadSyncEngine.Apply(bad, dwg);
        Check(r7.Ok && r7.Created == 2 && r7.Errors == 0, "a degenerate element (fewer than 3 points) is skipped, the rest is drawn");

        Console.WriteLine("=== Named pipe: Revit side <-> AutoCAD side");
        string pipe = "pcw-test-" + Guid.NewGuid().ToString("N").Substring(0, 8);
        var live = new FakeDrawing();
        var server = new CadServer(pipe, req =>
        {
            string type = CadMessage.TypeOf(req);
            if (type == CadMessage.Ping) return new PublishResult { Ok = true, Drawing = live.Name }.ToJson();
            if (type == CadMessage.Publish) return CadSyncEngine.Apply(PublishBatch.FromJson(req), live).ToJson();
            return new PublishResult { Ok = false, Message = "unknown message" }.ToJson();
        });
        server.Start();
        try
        {
            Check(CadClient.Ping(pipe) == "Plan-01.dwg", "ping finds the listening AutoCAD and its drawing name");
            var pb = new PublishBatch { Scope = "Live|L1", FullScope = true, Elements = { Wall("a", 0, 0, 4000, 200), Wall("b", 0, 2000, 4000, 200) } };
            var pr = CadClient.Publish(pipe, pb);
            Check(pr.Ok && pr.Created == 4 && pr.Drawing == "Plan-01.dwg", "publish over the pipe creates the walls (created=" + pr.Created + ")");
            pb.Elements[0] = Wall("a", 0, 0, 4500, 200);
            var pr2 = CadClient.Publish(pipe, pb);
            Check(pr2.Updated == 2 && pr2.Unchanged == 2, "live re-publish updates only the changed wall");

            bool srvSurvivedGarbage;
            var g = PublishResult.FromJson(CadClient.Request(pipe, "this is not json"));
            srvSurvivedGarbage = !g.Ok;
            Check(srvSurvivedGarbage && CadClient.Ping(pipe) != null, "garbage message: answered with an error, listener keeps running");

            using (var c = new System.IO.Pipes.NamedPipeClientStream(".", pipe)) { c.Connect(2000); }   // connect and vanish
            Check(CadClient.Ping(pipe) != null, "a client that disconnects mid-way does not break the listener");

            for (int i = 0; i < 50; i++) CadClient.Publish(pipe, pb);
            Check(server.Handled >= 55, "50 rapid publishes handled in a row (" + server.Handled + " requests handled)");

            var big = new PublishBatch { Scope = "Big|L1", FullScope = true };
            for (int i = 0; i < 5000; i++) big.Elements.Add(Wall("big" + i, (i % 100) * 5000, (i / 100) * 4000, 4800, 200));
            var sw = Stopwatch.StartNew();
            var br = CadClient.Publish(pipe, big, 5000);
            long ms1 = sw.ElapsedMilliseconds;
            sw.Restart();
            var br2 = CadClient.Publish(pipe, big, 5000);
            Check(br.Created == 10000 && br2.Unchanged == 10000, "5000 walls: created " + br.Created + " in " + ms1 + " ms; re-publish all-unchanged in " + sw.ElapsedMilliseconds + " ms");
            Check(big.ToJson().Length < 8 * 1024 * 1024, "5000-wall message is " + (big.ToJson().Length / 1024) + " KB");
        }
        finally { server.Stop(); }
        Check(!server.Running, "listener stops cleanly");

        Console.WriteLine("=== Nothing listening");
        var sw2 = Stopwatch.StartNew();
        string msg = "";
        try { CadClient.Request("pcw-nobody-" + Guid.NewGuid().ToString("N"), "{}", 700); } catch (TimeoutException ex) { msg = ex.Message; }
        Check(msg.Contains("AutoCAD is not listening") && sw2.ElapsedMilliseconds < 3000, "no AutoCAD: clear error after " + sw2.ElapsedMilliseconds + " ms, no hang");

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }

    private static void Apply2(PublishBatch b, FakeDrawing d) => CadSyncEngine.Apply(b, d);
}

