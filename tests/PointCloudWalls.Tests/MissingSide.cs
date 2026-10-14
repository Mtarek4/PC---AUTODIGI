using System; using System.Collections.Generic; using System.Linq; using PointCloudWalls.Core;
internal static class MissingSide
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    // a slender column (like the screenshot) with ONE short end partly occluded: only a sparse cluster of points
    // clustered SHORT of the true corner - simulating a scanner that just missed the very top/bottom of that face
    static List<P2> SlenderPartial(P2 c, double w, double d, double ang, double missingFrac)
    {
        var pts = new List<P2>();
        double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double a0, double a1, double off, bool alongU, double density)
        {
            int n = (int)((a1 - a0) * density);
            for (int i = 0; i < n; i++) { double t = a0 + rng.NextDouble() * (a1 - a0), o = off + Gauss(0.004); pts.Add(alongU ? c + U * t + V * o : c + V * t + U * o); }
        }
        Face(-w / 2, w / 2, -d / 2, true, 700); Face(-w / 2, w / 2, d / 2, true, 700);     // the two long sides: fully scanned
        Face(-d / 2, d / 2 - d * missingFrac, -w / 2, false, 700);                          // one short end: fully scanned
        // the OTHER short end: only a sparse cluster, clustered SHORT of the true corner (occlusion, not just noise)
        for (int i = 0; i < 15; i++) pts.Add(c + V * (d / 2 - d * missingFrac * 0.5 + Gauss(0.01)) + U * (Gauss(0.03)));
        for (int i = 0; i < 100; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2, (rng.NextDouble() - 0.5) * 2));
        return pts;
    }

    public static int Run()
    {
        Console.WriteLine("=== Diagonal clicks at the TRUE corners, one short end only partly (sparsely) scanned");
        int ok = 0, trials = 150; var lenErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(40000 + t);
            double w = 0.25 + rng.NextDouble() * 0.15, d = 0.7 + rng.NextDouble() * 0.4, ang = rng.NextDouble() * Math.PI;
            var c = new P2(5, 5);
            var pts = SlenderPartial(c, w, d, ang, 0.12);
            double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
            P2 cornerA = c - U * (w / 2) - V * (d / 2) + new P2(Gauss(0.01), Gauss(0.01));
            P2 cornerB = c + U * (w / 2) + V * (d / 2) + new P2(Gauss(0.01), Gauss(0.01));
            var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
            if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
            ok++;
            double fittedLen = Math.Max(r.Width, r.Depth);
            lenErr.Add(fittedLen - d);
        }
        Check(ok >= 140, $"{ok}/{trials} fitted");
        if (lenErr.Count > 0) Console.WriteLine($"   length err  mean {lenErr.Average() * 1000:+0.0;-0.0}mm  rms {Math.Sqrt(lenErr.Select(x => x * x).Average()) * 1000:0.0}mm  (negative = still too short)");
        Check(lenErr.Count > 0 && Math.Abs(lenErr.Average()) < 0.010, "no longer systematically short - extended to the clicked corner");

        Console.WriteLine("\n=== Control: all 4 sides fully scanned, sloppy diagonal clicks must NOT override good data");
        int ok2 = 0, trials2 = 150; var err2 = new List<double>();
        for (int t = 0; t < trials2; t++)
        {
            rng = new Random(41000 + t);
            double w = 0.25 + rng.NextDouble() * 0.15, d = 0.7 + rng.NextDouble() * 0.4, ang = rng.NextDouble() * Math.PI;
            var c = new P2(5, 5);
            var pts = SlenderPartial(c, w, d, ang, 0.0);   // missingFrac=0 -> both ends fully scanned (the sparse-cluster line still runs, but at full extent this time)
            double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
            // clicks sloppy by up to 8 cm - should NOT drag a well-measured edge along with them
            P2 cornerA = c - U * (w / 2) - V * (d / 2) + new P2(Gauss(0.04), Gauss(0.04));
            P2 cornerB = c + U * (w / 2) + V * (d / 2) + new P2(Gauss(0.04), Gauss(0.04));
            var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
            if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
            ok2++;
            double fittedLen = Math.Max(r.Width, r.Depth);
            err2.Add(fittedLen - d);
        }
        Check(ok2 >= 140, $"{ok2}/{trials2} fitted");
        if (err2.Count > 0) Console.WriteLine($"   length err  rms {Math.Sqrt(err2.Select(x => x * x).Average()) * 1000:0.0}mm");
        Check(err2.Count > 0 && Math.Sqrt(err2.Select(x => x * x).Average()) < 0.010, "stays accurate (not pulled by sloppy clicks) when the scan already has good data");

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
