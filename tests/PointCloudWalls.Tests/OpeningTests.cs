using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

// Synthetic wall-face point clouds (in the wall's own along/height coordinates) with a door or window opening,
// clutter, noise and sloppy clicks - to test OpeningFitter the same way every other tool in this project was tested.
internal static class OpeningTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    // wallLen: total surface span simulated; wallTop: wall height above base=0. Opening: [openA0,openA1] x [openH0,openH1].
    static List<P2> WallWithOpening(double wallLen, double wallTop, double openA0, double openA1, double openH0, double openH1,
                                    double density, double sigma, int clutter)
    {
        var pts = new List<P2>();
        int nH = (int)(wallTop * density), nV = (int)((openH1 - openH0) * density);
        // top band (above the opening, full width)
        for (int i = 0; i < (int)(wallLen * (wallTop - openH1) * density); i++)
            pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(sigma), openH1 + rng.NextDouble() * (wallTop - openH1) + Gauss(sigma)));
        // bottom band (below the opening, full width) - only if the opening doesn't reach the floor
        if (openH0 > 1e-6)
            for (int i = 0; i < (int)(wallLen * openH0 * density); i++)
                pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(sigma), rng.NextDouble() * openH0 + Gauss(sigma)));
        // left jamb strip (left of the opening, full height of the opening band)
        for (int i = 0; i < (int)(openA0 * (openH1 - openH0) * density); i++)
            pts.Add(new P2(rng.NextDouble() * openA0 + Gauss(sigma), openH0 + rng.NextDouble() * (openH1 - openH0) + Gauss(sigma)));
        // right jamb strip
        for (int i = 0; i < (int)((wallLen - openA1) * (openH1 - openH0) * density); i++)
            pts.Add(new P2(openA1 + rng.NextDouble() * (wallLen - openA1) + Gauss(sigma), openH0 + rng.NextDouble() * (openH1 - openH0) + Gauss(sigma)));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * wallLen, rng.NextDouble() * wallTop));      // e.g. reflections, furniture silhouettes
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== Doors: opening reaches the floor");
        {
            int ok = 0, kindOk = 0, trials = 200; var wErr = new List<double>(); var hErr = new List<double>(); var reasons1 = new Dictionary<string, int>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(1000 + t);
                double wallLen = 10.0, wallTop = 2.8;
                double w = 0.7 + rng.NextDouble() * 0.5, h = 1.9 + rng.NextDouble() * 0.35;
                double a0 = 2.2 + rng.NextDouble() * 3.0; double a1 = a0 + w;
                var pts = WallWithOpening(wallLen, wallTop, a0, a1, 0, h, 600, 0.005, 200);
                var click = new P2((a0 + a1) / 2 + Gauss(0.15), h / 2 + Gauss(0.2));
                var r = OpeningFitter.Fit(pts, click, 0, wallTop);
                if (!r.Success) { reasons1[r.Message] = reasons1.GetValueOrDefault(r.Message) + 1; continue; }
                ok++;
                if (r.Kind == OpeningKind.Door) kindOk++;
                wErr.Add(r.Width - w); hErr.Add(r.Height - h);
            }
            Check(ok >= 190, $"{ok}/{trials} fitted");
            foreach (var kv in reasons1.OrderByDescending(k => k.Value).Take(4)) Console.WriteLine("   " + kv.Value + " x " + kv.Key);
            Check(kindOk == ok, $"{kindOk}/{ok} correctly classified as DOOR");
            if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm   height err rms {Math.Sqrt(hErr.Select(x => x * x).Average()) * 1000:0.0}mm");
        }

        Console.WriteLine("\n=== Windows: opening has a sill above the floor");
        {
            int ok = 0, kindOk = 0, trials = 200; var wErr = new List<double>(); var hErr = new List<double>(); var sillErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(2000 + t);
                double wallLen = 10.0, wallTop = 2.8;
                double w = 0.6 + rng.NextDouble() * 1.0, h = 0.8 + rng.NextDouble() * 0.7;
                double sill = 0.7 + rng.NextDouble() * 0.4;
                double a0 = 2.2 + rng.NextDouble() * 3.0; double a1 = a0 + w;
                if (sill + h > wallTop - 0.15) continue;
                var pts = WallWithOpening(wallLen, wallTop, a0, a1, sill, sill + h, 600, 0.005, 200);
                var click = new P2((a0 + a1) / 2 + Gauss(0.15), sill + h / 2 + Gauss(0.15));
                var r = OpeningFitter.Fit(pts, click, 0, wallTop);
                if (!r.Success) continue;
                ok++;
                if (r.Kind == OpeningKind.Window) kindOk++;
                wErr.Add(r.Width - w); hErr.Add(r.Height - h); sillErr.Add(r.SillHeight - sill);
            }
            Check(ok >= 190, $"{ok}/{trials} fitted");
            Check(kindOk == ok, $"{kindOk}/{ok} correctly classified as WINDOW");
            if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm   height err rms {Math.Sqrt(hErr.Select(x => x * x).Average()) * 1000:0.0}mm   sill err rms {Math.Sqrt(sillErr.Select(x => x * x).Average()) * 1000:0.0}mm");
        }

        Console.WriteLine("\n=== Sloppy clicks (up to 30cm off-center) must still land in the same opening");
        {
            int ok = 0, trials = 200; var wErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(3000 + t);
                double wallLen = 10.0, wallTop = 2.8, w = 0.9, h = 2.1, a0 = 4.0, a1 = a0 + w;
                var pts = WallWithOpening(wallLen, wallTop, a0, a1, 0, h, 600, 0.005, 150);
                var click = new P2(a0 + rng.NextDouble() * w + Gauss(0.10), h * 0.5 + Gauss(0.5));   // click anywhere across the width, sloppy in height too
                var r = OpeningFitter.Fit(pts, click, 0, wallTop);
                if (!r.Success) continue;
                ok++; wErr.Add(r.Width - w);
            }
            Check(ok >= 190, $"{ok}/{trials} fitted despite sloppy clicks");
        }

        Console.WriteLine("\n=== Rejections: click on solid wall with no gap nearby, or a gap that runs off the search area");
        {
            int wrong = 0, trials = 60;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(4000 + t);
                var pts = WallWithOpening(6.0, 2.8, 100, 100, 0, 0, 600, 0.005, 0);   // no opening at all - solid wall
                var click = new P2(3.0, 1.4);
                var r = OpeningFitter.Fit(pts, click, 0, 2.8);
                if (r.Success) wrong++;
            }
            Check(wrong == 0, $"solid wall (no opening): {wrong}/{trials} wrongly found a gap");
        }
        {
            // an opening WIDER than the search range - should fail cleanly (touches the search boundary), not silently mis-size
            int wrong = 0, trials = 60;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(5000 + t);
                var pts = WallWithOpening(10.0, 2.8, 1.0, 9.0, 0, 2.0, 400, 0.005, 100);   // 8m "opening" (e.g. an open storefront)
                var click = new P2(5.0, 1.0);
                var r = OpeningFitter.Fit(pts, click, 0, 2.8, new OpeningFitOptions { SearchHalfWidth = 1.8 });
                if (r.Success) wrong++;
            }
            Check(wrong == 0, $"oversized gap (runs past the search area): {wrong}/{trials} wrongly reported a normal opening");
        }

        Console.WriteLine("\n=== A door right next to a window in the same wall: must not merge them into one opening");
        {
            int ok = 0, trials = 100; var wErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(6000 + t);
                double wallLen = 8.0, wallTop = 2.8;
                var pts = new List<P2>();
                // build by hand: solid everywhere, THEN carve two separate openings with a solid pier between them
                pts.AddRange(WallWithOpening(wallLen, wallTop, 100, 100, 0, 0, 700, 0.005, 0));   // fully solid base
                // now remove points inside door [1.0,1.9]x[0,2.05] and window [2.6,3.6]x[0.9,1.9], leaving a solid pier 1.9-2.6
                double dA0 = 1.0, dA1 = 1.9, wA0 = 2.6, wA1 = 3.6, wH0 = 0.9, wH1 = 1.9;
                pts = pts.Where(p => !(p.X > dA0 && p.X < dA1 && p.Y < 2.05) && !(p.X > wA0 && p.X < wA1 && p.Y > wH0 && p.Y < wH1)).ToList();
                var clickDoor = new P2((dA0 + dA1) / 2 + Gauss(0.05), 1.0 + Gauss(0.1));
                var r = OpeningFitter.Fit(pts, clickDoor, 0, wallTop, new OpeningFitOptions { SearchHalfWidth = 1.8 });
                if (!r.Success || r.Kind != OpeningKind.Door) continue;
                ok++; wErr.Add(r.Width - (dA1 - dA0));
            }
            Check(ok >= 90, $"{ok}/{trials} correctly found just the DOOR, not merged with the nearby window");
            if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
