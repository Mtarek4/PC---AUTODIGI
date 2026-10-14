using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

internal static class SnapTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }
    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }
    static ColumnFitOptions RectOnly() => new ColumnFitOptions { AllowRound = false };

    static List<P2> PartiallyExposedColumn(double wallThick, double cw, double cd, double density, int clutter)
    {
        var pts = new List<P2>();
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, 0 + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, wallThick + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(0 + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(wallThick + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < (int)(cw * density); i++) pts.Add(new P2(rng.NextDouble() * cw, cd + Gauss(0.004)));
        for (int i = 0; i < (int)(cd * density); i++) pts.Add(new P2(cw + Gauss(0.004), rng.NextDouble() * cd));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * 3 - 0.5, rng.NextDouble() * 3 - 0.5));
        return pts;
    }

    static List<P2> FullyFlushColumn(double wallThick, double density, int clutter)
    {
        var pts = new List<P2>();
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, 0 + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, wallThick + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(0 + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(wallThick + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * 3 - 0.5, rng.NextDouble() * 3 - 0.5));
        return pts;
    }

    public static int Run()
    {
        Console.WriteLine("=== Partially exposed embedded column (300 trials): with the click-diagonal sanity check in place");
        {
            int ok = 0, trials = 300; var errs = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(5000 + t);
                double wallThick = 0.20 + rng.NextDouble() * 0.10;
                double cw = wallThick + 0.05 + rng.NextDouble() * 0.05, cd = wallThick + 0.05 + rng.NextDouble() * 0.05;
                var pts = PartiallyExposedColumn(wallThick, cw, cd, 500, 100);
                P2 cornerOut = new P2(-0.02, -0.02) + new P2(Gauss(0.02), Gauss(0.02));
                P2 cornerIn = new P2(cw, cd) + new P2(Gauss(0.02), Gauss(0.02));
                var r = ColumnFitter.FitDiagonal(pts, cornerOut, cornerIn, RectOnly());
                if (!r.Success) continue;
                ok++;
                errs.Add(Math.Max(Math.Abs(r.Width - cw), Math.Abs(r.Depth - cd)));
            }
            Check(ok >= 285, $"{ok}/{trials} fitted");
            var sorted = errs.OrderByDescending(x => x).ToList();
            Console.WriteLine($"   median {sorted[trials / 2] * 1000:0.0}mm   worst 10: {string.Join(", ", sorted.Take(10).Select(x => (x * 1000).ToString("0")))}mm");
            int over100 = errs.Count(e => e > 0.100);
            Console.WriteLine($"   over 100mm: {over100}/{trials}");
            Check(over100 == 0, "no severe (>100mm) outliers");
        }

        Console.WriteLine("\n=== Fully flush column (no real column signal at all) - must still succeed, no severe outliers");
        {
            int ok = 0, trials = 200; var errs = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(1000 + t);
                double wallThick = 0.20 + rng.NextDouble() * 0.10;
                var pts = FullyFlushColumn(wallThick, 500, 100);
                P2 cornerOut = new P2(-0.02, -0.02) + new P2(Gauss(0.02), Gauss(0.02));
                P2 cornerIn = new P2(wallThick, wallThick) + new P2(Gauss(0.02), Gauss(0.02));
                var r = ColumnFitter.FitDiagonal(pts, cornerOut, cornerIn, RectOnly());
                if (!r.Success) continue;
                ok++;
                errs.Add(Math.Max(Math.Abs(r.Width - wallThick), Math.Abs(r.Depth - wallThick)));
            }
            Check(ok >= 190, $"{ok}/{trials} fitted");
            Console.WriteLine($"   worst: {errs.Max() * 1000:0.0}mm");
            Check(errs.Max() < 0.10, "no severe outliers");
        }

        Console.WriteLine("\n=== Regression: normal freestanding column, far from any wall - completely unaffected");
        {
            int okNorm = 0; var wErr2 = new List<double>();
            for (int t = 0; t < 150; t++)
            {
                rng = new Random(2000 + t);
                double w = 0.3 + rng.NextDouble() * 0.5, d = 0.3 + rng.NextDouble() * 0.5, ang = rng.NextDouble() * Math.PI;
                var c = new P2(5, 5);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                var pts = new List<P2>();
                void Face(double a0, double a1, double off, bool alongU) { int n = (int)((a1 - a0) * 700); for (int i = 0; i < n; i++) { double tt = a0 + rng.NextDouble() * (a1 - a0), o2 = off + Gauss(0.004); pts.Add(alongU ? c + U * tt + V * o2 : c + V * tt + U * o2); } }
                Face(-w / 2, w / 2, -d / 2, true); Face(-w / 2, w / 2, d / 2, true); Face(-d / 2, d / 2, -w / 2, false); Face(-d / 2, d / 2, w / 2, false);
                for (int i = 0; i < 100; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2, (rng.NextDouble() - 0.5) * 2));
                P2 cA = c - U * (w / 2) - V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                P2 cB = c + U * (w / 2) + V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                var r = ColumnFitter.FitDiagonal(pts, cA, cB, RectOnly());
                if (!r.Success) continue;
                okNorm++;
                double a1 = Math.Abs(Math.IEEERemainder(r.Angle - ang, Math.PI)) * 180 / Math.PI;
                double a2 = Math.Abs(Math.IEEERemainder(r.Angle - ang - Math.PI / 2, Math.PI)) * 180 / Math.PI;
                wErr2.Add(Math.Min(a1, a2));
            }
            Check(okNorm >= 145, $"{okNorm}/150 still fitted normally");
            Check(wErr2.Count > 0 && wErr2.Average() < 1.0, $"angle matches TRUE column - mean err {(wErr2.Count > 0 ? wErr2.Average() : -1):0.00}deg");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
