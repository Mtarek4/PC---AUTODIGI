using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Cad;

public static class PolyTests
{
    private static int _fail;
    private static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    /// <summary>A wall footprint: centre, length, thickness, angle (radians), as a counter-clockwise rectangle.</summary>
    private static double[] Rect(double cx, double cy, double len, double thk, double ang)
    {
        double c = Math.Cos(ang), s = Math.Sin(ang);
        var pts = new[] { (-len / 2, -thk / 2), (len / 2, -thk / 2), (len / 2, thk / 2), (-len / 2, thk / 2) };
        var r = new double[8];
        for (int i = 0; i < 4; i++) { r[2 * i] = cx + pts[i].Item1 * c - pts[i].Item2 * s; r[2 * i + 1] = cy + pts[i].Item1 * s + pts[i].Item2 * c; }
        return r;
    }

    private static bool InEvenOdd(double x, double y, List<double[]> loops)
    {
        bool inside = false;
        foreach (var l in loops)
        {
            int n = l.Length / 2; bool ins = false;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                double xi = l[2 * i], yi = l[2 * i + 1], xj = l[2 * j], yj = l[2 * j + 1];
                if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) ins = !ins;
            }
            if (ins) inside = !inside;
        }
        return inside;
    }

    private static double DistToBoundary(double x, double y, IEnumerable<double[]> polys)
    {
        double best = double.MaxValue;
        foreach (var l in polys)
        {
            int n = l.Length / 2;
            for (int i = 0; i < n; i++)
            {
                double ax = l[2 * i], ay = l[2 * i + 1], bx = l[2 * ((i + 1) % n)], by = l[2 * ((i + 1) % n) + 1];
                double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy, u = l2 < 1e-24 ? 0 : Math.Max(0, Math.Min(1, ((x - ax) * dx + (y - ay) * dy) / l2));
                best = Math.Min(best, Math.Sqrt((ax + u * dx - x) * (ax + u * dx - x) + (ay + u * dy - y) * (ay + u * dy - y)));
            }
        }
        return best;
    }

    private static string Desc(List<double[]> loops) => loops.Count + " loop(s), vertices: " + string.Join("+", loops.Select(l => l.Length / 2));

    public static int Run()
    {
        const double tol = 0.0005;
        Console.WriteLine("=== Polygon union: the junctions from your Revit / AutoCAD screenshots");

        // T: a bar and a stem whose top edge lies ON the bar's lower edge (abutting - Revit-trimmed walls)
        var bar = Rect(0, 0, 4, 0.2, 0);
        var stem = Rect(0, -2.1, 4, 0.2, Math.PI / 2);     // its top end is exactly at y = -0.1
        var t = PolygonUnion.Union(new List<double[]> { bar, stem }, tol);
        Check(t.Count == 1 && t[0].Length / 2 == 8, "T-junction (abutting): one 8-corner outline, the line across the junction is gone  [" + Desc(t) + "]");
        double tArea = Math.Abs(PolygonUnion.SignedArea(t[0]));
        Check(Math.Abs(tArea - (0.8 + 0.8)) < 1e-6, "T area = bar + stem (" + tArea.ToString("0.000") + ")");

        // the same T rotated 12 degrees, like your scan
        double a = 12 * Math.PI / 180, ca = Math.Cos(a), sa = Math.Sin(a);
        Func<double[], double[]> rot = p => { var q = new double[p.Length]; for (int i = 0; i < p.Length; i += 2) { q[i] = p[i] * ca - p[i + 1] * sa + 7; q[i + 1] = p[i] * sa + p[i + 1] * ca + 3; } return q; };
        var tr = PolygonUnion.Union(new List<double[]> { rot(bar), rot(stem) }, tol);
        Check(tr.Count == 1 && tr[0].Length / 2 == 8, "T rotated 12 degrees: still one 8-corner outline  [" + Desc(tr) + "]");

        // T where the stem OVERLAPS the bar (Revit walls that were not trimmed)
        var stemOv = Rect(0, -1.9, 4, 0.2, Math.PI / 2);   // reaches up to y = +0.1: through the bar
        var tOv = PolygonUnion.Union(new List<double[]> { bar, stemOv }, tol);
        Check(tOv.Count == 1 && tOv[0].Length / 2 == 12 || tOv.Count == 1 && tOv[0].Length / 2 == 8, "stem overlapping the bar -> one clean outline  [" + Desc(tOv) + "]");

        // L corner: two walls overlapping at the corner
        var l1 = Rect(0, 0, 3, 0.2, 0); var l2 = Rect(-1.4, -1.4, 3, 0.2, Math.PI / 2);
        var l = PolygonUnion.Union(new List<double[]> { l1, l2 }, tol);
        Check(l.Count == 1 && l[0].Length / 2 == 6, "L corner (overlapping): one 6-corner outline  [" + Desc(l) + "]");

        // X crossing
        var x = PolygonUnion.Union(new List<double[]> { Rect(0, 0, 4, 0.2, 0), Rect(0, 0, 4, 0.2, Math.PI / 2) }, tol);
        Check(x.Count == 1 && x[0].Length / 2 == 12, "X crossing: one 12-corner outline  [" + Desc(x) + "]");

        // two collinear walls end to end (same thickness): one plain rectangle
        var cl = PolygonUnion.Union(new List<double[]> { Rect(-1, 0, 2, 0.2, 0), Rect(1, 0, 2, 0.2, 0) }, tol);
        Check(cl.Count == 1 && cl[0].Length / 2 == 4, "two collinear walls end to end: merged into ONE rectangle with 4 corners  [" + Desc(cl) + "]");

        // a room: four walls closing a ring -> outer outline + inner outline (the room)
        var ring = PolygonUnion.Union(new List<double[]> { Rect(0, 2, 4.2, 0.2, 0), Rect(0, -2, 4.2, 0.2, 0), Rect(-2, 0, 4.2, 0.2, Math.PI / 2), Rect(2, 0, 4.2, 0.2, Math.PI / 2) }, tol);
        bool ringOk = ring.Count == 2 && ring.Count(r => PolygonUnion.SignedArea(r) > 0) == 1 && ring.Count(r => PolygonUnion.SignedArea(r) < 0) == 1;
        Check(ringOk, "a closed room of 4 walls: outer outline + inner room outline  [" + Desc(ring) + "]");

        // disjoint walls stay separate
        var dj = PolygonUnion.Union(new List<double[]> { Rect(0, 0, 2, 0.2, 0), Rect(0, 3, 2, 0.2, 0) }, tol);
        Check(dj.Count == 2, "two walls that do not touch stay two outlines  [" + Desc(dj) + "]");

        // wall with a mitred corner (pentagon footprints from a Revit join) meeting a straight wall
        var m1 = new[] { 0.0, 0, 3, 0, 3.1, 0.2, 0, 0.2 };     // mitred end
        var m2 = new[] { 3.0, 0, 3.1, 0.2, 3.1, 3, 2.9, 3, 2.9, 0.0 };   // vertical wall meeting it
        var mit = PolygonUnion.Union(new List<double[]> { m1, m2 }, tol);
        Check(mit.Count == 1, "mitred L join from Revit: one outline  [" + Desc(mit) + "]");

        Console.WriteLine("=== Polygon union: 400 random wall layouts checked against brute force");
        var rng = new Random(42);
        int bad = 0, badPts = 0, totalPts = 0;
        for (int trial = 0; trial < 400; trial++)
        {
            int n = 2 + rng.Next(6);
            var polys = new List<double[]>();
            for (int i = 0; i < n; i++)
            {
                double ang = rng.NextDouble() < 0.6 ? rng.Next(24) * Math.PI / 12 : rng.NextDouble() * Math.PI;
                polys.Add(Rect(rng.NextDouble() * 6, rng.NextDouble() * 6, 1 + rng.NextDouble() * 5, 0.1 + rng.NextDouble() * 0.25, ang));
            }
            var u = PolygonUnion.Union(polys, tol);
            int mism = 0;
            for (int k = 0; k < 400; k++)
            {
                double px = rng.NextDouble() * 12 - 3, py = rng.NextDouble() * 12 - 3;
                if (DistToBoundary(px, py, polys) < 0.01 || DistToBoundary(px, py, u) < 0.01) continue;
                bool inAny = polys.Any(p => InEvenOdd(px, py, new List<double[]> { p }));
                totalPts++;
                if (inAny != InEvenOdd(px, py, u)) { mism++; badPts++; }
            }
            if (mism > 0) bad++;
        }
        Check(bad == 0, "union matches brute force on " + totalPts + " sample points in 400 random layouts (" + badPts + " mismatches, " + bad + " bad layouts)");

        Console.WriteLine("=== Touch test (which walls form one cluster)");
        Check(PolygonUnion.Touch(bar, stem, tol), "walls that abut are one cluster");
        Check(PolygonUnion.Touch(bar, stemOv, tol), "walls that overlap are one cluster");
        Check(!PolygonUnion.Touch(bar, Rect(0, 3, 2, 0.2, 0), tol), "walls 3 m apart are not");
        Check(PolygonUnion.Touch(Rect(0, 0, 4, 4, 0), Rect(0, 0, 1, 1, 0), tol), "a wall completely inside another counts as touching");
        Console.WriteLine("=== Clustering");
        var many = new List<double[]> { bar, stem, Rect(0, 5, 2, 0.2, 0), Rect(1.0, 5, 2, 0.2, Math.PI / 2), Rect(20, 20, 2, 0.2, 0), Rect(0, 1.3, 4, 0.2, 0) };
        int[] cl2 = PolygonUnion.Cluster(many, tol);
        Check(cl2[0] == cl2[1] && cl2[2] == cl2[3] && cl2[0] != cl2[2] && cl2[4] != cl2[0] && cl2[4] != cl2[2] && cl2[5] != cl2[0],
              "clusters: T (bar+stem), a separate L, a far wall alone, a nearby-but-not-touching wall alone (groups " + string.Join(",", cl2) + ")");
        var rn = new Random(7); var lots = new List<double[]>();
        for (int i = 0; i < 5000; i++) lots.Add(Rect((i % 100) * 6.0, (i / 100) * 5.0, 4.8, 0.2, 0));      // a grid of 5000 separate walls
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int[] cc = PolygonUnion.Cluster(lots, tol);
        Check(cc.Distinct().Count() == 5000 && sw.ElapsedMilliseconds < 3000, "5000 walls clustered in " + sw.ElapsedMilliseconds + " ms");
        var chain = new List<double[]>(); for (int i = 0; i < 300; i++) chain.Add(Rect(i * 2.0 + 1, 0, 2.0, 0.2, 0));   // 300 collinear walls end to end
        sw.Restart();
        var chainU = PolygonUnion.Union(chain, tol);
        Check(chainU.Count == 1 && chainU[0].Length / 2 == 4 && sw.ElapsedMilliseconds < 5000, "300 collinear walls merge into ONE rectangle in " + sw.ElapsedMilliseconds + " ms");
        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
