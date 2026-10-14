using System; using System.Collections.Generic; using System.Linq; using PointCloudWalls.Core;
internal static class UnevenTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> RectColUneven(P2 c, double w, double d, double ang, double sigma, double denseDensity, double sparseFraction)
    {
        // simulates self-occlusion: the two faces nearer an imaginary scanner (u=+w/2, v=+d/2) are densely scanned,
        // the two far faces (u=-w/2, v=-d/2) are scanned at only 'sparseFraction' of that density
        var pts = new List<P2>();
        double ca = Math.Cos(ang), sa = Math.Sin(ang);
        P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double u0, double u1, double v, bool alongU, double density)
        {
            int n = (int)((u1 - u0) * density);
            for (int i = 0; i < n; i++)
            {
                double t = u0 + rng.NextDouble() * (u1 - u0);
                double vv = v + Gauss(sigma);
                pts.Add(alongU ? c + U * t + V * vv : c + V * t + U * vv);
            }
        }
        Face(-w / 2, w / 2, -d / 2, true, denseDensity * sparseFraction);    // far face (sparse - self-occluded)
        Face(-w / 2, w / 2, d / 2, true, denseDensity);                      // near face (dense)
        Face(-d / 2, d / 2, -w / 2, false, denseDensity * sparseFraction);   // far face (sparse)
        Face(-d / 2, d / 2, w / 2, false, denseDensity);                     // near face (dense)
        return pts;
    }

    static void Clutter(List<P2> pts, P2 c, int n, double range) { for (int i = 0; i < n; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2 * range, (rng.NextDouble() - 0.5) * 2 * range)); }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    internal static int Run()
    {
        Console.WriteLine("=== Rectangular column, UNEVEN face density (self-occlusion: far faces at 25% of near-face density) - like a real scan");
        {
            int ok = 0, trials = 200; var wErr = new List<double>(); var dErr = new List<double>(); var aErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(20000 + t);
                double w = 0.3 + rng.NextDouble() * 0.6, d = 0.3 + rng.NextDouble() * 0.6, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(10 + rng.NextDouble() * 5, 5 + rng.NextDouble() * 5);
                var pts = RectColUneven(c, w, d, ang, 0.006, 700, 0.25);
                Clutter(pts, c, 150, 1.0);
                var click = c + new P2(Gauss(0.05), Gauss(0.05));
                var r = ColumnFitter.FitCenter(pts, click);
                if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
                ok++;
                double e1 = Math.Abs(r.Width - w) + Math.Abs(r.Depth - d);
                double e2 = Math.Abs(r.Width - d) + Math.Abs(r.Depth - w);
                if (e1 <= e2) { wErr.Add(r.Width - w); dErr.Add(r.Depth - d); } else { wErr.Add(r.Width - d); dErr.Add(r.Depth - w); }
            }
            Check(ok >= 180, $"{ok}/{trials} fitted despite very uneven (4x) face density");
            if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.00}mm   depth err rms {Math.Sqrt(dErr.Select(x => x * x).Average()) * 1000:0.00}mm");
        }

        Console.WriteLine("\n=== Diagonal clicks, uneven density");
        {
            int ok = 0, trials = 150;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(21000 + t);
                double w = 0.3 + rng.NextDouble() * 0.5, d = 0.3 + rng.NextDouble() * 0.5, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(30 + rng.NextDouble() * 5, 12 + rng.NextDouble() * 5);
                var pts = RectColUneven(c, w, d, ang, 0.006, 700, 0.25);
                Clutter(pts, c, 150, 1.0);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                P2 cornerA = c - U * (w / 2) - V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                P2 cornerB = c + U * (w / 2) + V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
                if (r.Success && r.Shape == ColumnShape.Rectangular) ok++;
            }
            Check(ok >= 130, $"{ok}/{trials} fitted via diagonal clicks despite uneven density");
        }

        Console.WriteLine("\n=== Sides clicks (on the DENSE near faces), uneven density - other axis auto-found from the sparser far/near pair");
        {
            int ok = 0, trials = 150; var aErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(22000 + t);
                double w = 0.3 + rng.NextDouble() * 0.5, d = 0.3 + rng.NextDouble() * 0.5, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(40 + rng.NextDouble() * 5, 15 + rng.NextDouble() * 5);
                var pts = RectColUneven(c, w, d, ang, 0.006, 700, 0.25);
                Clutter(pts, c, 150, 1.0);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                // click the two faces ALONG U (one dense at v=+d/2 side... wait U faces are at u=+-w/2): click both U faces
                P2 clickA = c - U * (w / 2) + new P2(Gauss(0.02), Gauss(0.02));
                P2 clickB = c + U * (w / 2) + new P2(Gauss(0.02), Gauss(0.02));
                var r = ColumnFitter.FitSides(pts, clickA, clickB);
                if (!r.Success) continue;
                ok++;
                double da1 = Math.Abs(Math.IEEERemainder(r.Angle - ang, Math.PI / 2)) * 180 / Math.PI;
                aErr.Add(da1);
            }
            Check(ok >= 130, $"{ok}/{trials} fitted via sides clicks despite uneven density");
            if (aErr.Count > 0) Console.WriteLine($"   angle err rms {Math.Sqrt(aErr.Select(x => x * x).Average()):0.00}deg");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
