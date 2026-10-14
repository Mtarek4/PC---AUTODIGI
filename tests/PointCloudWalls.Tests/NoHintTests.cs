using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

internal static class NoHintTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> EmbeddedCornerColumn(double wallThick, double cw, double cd, double density, int clutter)
    {
        var pts = new List<P2>();
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, 0 + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, wallThick + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(0 + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(wallThick + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * 3 - 0.5, rng.NextDouble() * 3 - 0.5));
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }
    static ColumnFitOptions RectOnly() => new ColumnFitOptions { AllowRound = false };

    public static int Run()
    {
        Console.WriteLine("=== Embedded corner column, NO Revit wall modeled, NO external hint - angle found purely from the point cloud");
        int ok = 0, trials = 150; var wErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(7000 + t);
            double wallThick = 0.20 + rng.NextDouble() * 0.10, cw = 0.35 + rng.NextDouble() * 0.15, cd = 0.35 + rng.NextDouble() * 0.15;
            double trueAngle = rng.NextDouble() * (Math.PI / 2);
            var pts = EmbeddedCornerColumn(wallThick, cw, cd, 500, 100);
            double ca = Math.Cos(trueAngle), sa = Math.Sin(trueAngle);
            P2 Rot(P2 p) => new P2(p.X * ca - p.Y * sa, p.X * sa + p.Y * ca);
            var rotPts = pts.Select(Rot).ToList();
            P2 cornerOut = Rot(new P2(-0.02, -0.02) + new P2(Gauss(0.02), Gauss(0.02)));
            P2 cornerIn = Rot(new P2(cw, cd) + new P2(Gauss(0.02), Gauss(0.02)));
            var r = ColumnFitter.FitDiagonal(rotPts, cornerOut, cornerIn, RectOnly());
            if (!r.Success) continue;
            ok++;
            double e1 = Math.Abs(r.Width - cw) + Math.Abs(r.Depth - cd), e2 = Math.Abs(r.Width - cd) + Math.Abs(r.Depth - cw);
            wErr.Add(Math.Min(e1, e2) / 2);   // per-dimension average, not summed
        }
        Check(ok >= 140, $"{ok}/{trials} fitted with NO external hint at all");
        if (wErr.Count > 0)
        {
            double rms = Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000;
            Console.WriteLine($"   per-dimension error rms {rms:0.0}mm (click precision expected - clicks were ~20mm sloppy)");
            Check(rms < 50, "consistent with click precision, no wild outliers");
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
