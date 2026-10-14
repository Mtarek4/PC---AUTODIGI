using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

/// <summary>
/// Targets a specific failure reported live: clicking a plain wall corner (no real column at all) in Diagonal mode,
/// varying exactly WHERE on/near the corner each click lands and WHICH corner is clicked first, could occasionally
/// have the fitter confidently declare a ROUND column - skipping the reliable click+wall-direction rectangular
/// fallback entirely and offering a shape that does not match the user's rectangular family type. The two clicks
/// themselves are not the cause (the whole pipeline up to the round/rect decision is provably symmetric in click
/// order - see ColumnFitter.FitDiagonal), but sparse, ambiguous wall-corner data can spuriously satisfy a circle
/// hypothesis depending on exactly which points a given click position happens to pull into range.
/// </summary>
internal static class RoundVsRectTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    // Same corner generator as EmbeddedColumnTests: two 6m walls meeting at a right angle, each modeled as two
    // parallel face lines `wallThick` apart - i.e. a plain wall corner with no column at all when proud=0.
    static List<P2> WallCorner(double wallThick, double density, int clutter)
    {
        var pts = new List<P2>();
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, 0 + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(rng.NextDouble() * 6.0, wallThick + Gauss(0.004)));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(0 + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < (int)(6.0 * density); i++) pts.Add(new P2(wallThick + Gauss(0.004), rng.NextDouble() * 6.0));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * 3 - 0.5, rng.NextDouble() * 3 - 0.5));
        return pts;
    }

    static List<P2> RoundCol(P2 c, double dia, double density, double sigma)
    {
        var pts = new List<P2>();
        double r = dia / 2;
        int n = (int)(2 * Math.PI * r * density);
        for (int i = 0; i < n; i++)
        {
            double a = rng.NextDouble() * 2 * Math.PI;
            double rr = r + Gauss(sigma);
            pts.Add(c + new P2(Math.Cos(a), Math.Sin(a)) * rr);
        }
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== Plain wall corner (no column), Diagonal mode, Round allowed - varying click position AND order");
        {
            int confidentlyRound = 0, trials = 300;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(9000 + t);
                double wallThick = 0.20 + rng.NextDouble() * 0.10;
                var pts = WallCorner(wallThick, 500, 100);
                // Click positions deliberately vary a lot - not always the same offset from the corner, matching the
                // reported "if I don't click on specific points" symptom - and the two clicks are randomly ordered.
                P2 p1 = new P2(-0.02 + Gauss(0.05), -0.02 + Gauss(0.05));
                P2 p2 = new P2(0.35 + Gauss(0.08), 0.35 + Gauss(0.08));
                bool swap = rng.NextDouble() < 0.5;
                P2 cornerA = swap ? p2 : p1, cornerB = swap ? p1 : p2;
                var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
                if (r.Success && r.Shape == ColumnShape.Round && r.Grade != "Check")
                    confidentlyRound++;
            }
            Check(confidentlyRound == 0, $"{confidentlyRound}/{trials} wrongly, confidently classified a plain wall corner as ROUND");
        }

        Console.WriteLine("\n=== Regression: a genuinely round column right next to ONE wall - must still be found as round");
        {
            int ok = 0, trials = 150;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(9500 + t);
                double dia = 0.35 + rng.NextDouble() * 0.25;
                var wallPts = new List<P2>();
                for (int i = 0; i < 3000; i++) wallPts.Add(new P2(rng.NextDouble() * 6.0 - 1.0, Gauss(0.004))); // wall along Y=0
                var c = new P2(2.5 + rng.NextDouble() * 0.5, dia / 2 + 0.15); // column sitting just off the wall face
                var pts = RoundCol(c, dia, 1200, 0.003);
                pts.AddRange(wallPts);
                P2 U = new P2(1, 0), V = new P2(0, 1);
                P2 cornerA = c - U * (dia / 2 + 0.02) - V * (dia / 2 + 0.02) + new P2(Gauss(0.03), Gauss(0.03));
                P2 cornerB = c + U * (dia / 2 + 0.02) + V * (dia / 2 + 0.02) + new P2(Gauss(0.03), Gauss(0.03));
                var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
                if (r.Success && r.Shape == ColumnShape.Round) ok++;
            }
            Check(ok >= 140, $"{ok}/{trials} still correctly found as round despite the nearby straight wall");
        }

        Console.WriteLine("\n=== Regression: a genuinely round, free-standing column (no wall nearby) via Diagonal clicks");
        {
            int ok = 0, trials = 150;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(9800 + t);
                double dia = 0.35 + rng.NextDouble() * 0.25;
                var c = new P2(20 + rng.NextDouble() * 5, 8 + rng.NextDouble() * 5);
                var pts = RoundCol(c, dia, 1500, 0.003);
                for (int i = 0; i < 150; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2.0, (rng.NextDouble() - 0.5) * 2.0));
                P2 U = new P2(1, 0), V = new P2(0, 1);
                P2 cornerA = c - U * (dia / 2 + 0.02) - V * (dia / 2 + 0.02) + new P2(Gauss(0.03), Gauss(0.03));
                P2 cornerB = c + U * (dia / 2 + 0.02) + V * (dia / 2 + 0.02) + new P2(Gauss(0.03), Gauss(0.03));
                var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
                if (r.Success && r.Shape == ColumnShape.Round) ok++;
            }
            Check(ok >= 140, $"{ok}/{trials} still correctly found as round, free-standing");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
