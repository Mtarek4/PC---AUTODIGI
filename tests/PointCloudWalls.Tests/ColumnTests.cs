using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

internal static class ColumnTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> RectCol(P2 c, double w, double d, double ang, double density, double sigma, double coverageFrac = 1.0)
    {
        var pts = new List<P2>();
        double ca = Math.Cos(ang), sa = Math.Sin(ang);
        P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double u0, double u1, double v, bool alongU)
        {
            int n = (int)((u1 - u0) * density);
            for (int i = 0; i < n; i++)
            {
                if (rng.NextDouble() > coverageFrac) continue;
                double t = u0 + rng.NextDouble() * (u1 - u0);
                double vv = v + Gauss(sigma);
                P2 p = alongU ? c + U * t + V * vv : c + V * t + U * vv;
                pts.Add(p);
            }
        }
        Face(-w / 2, w / 2, -d / 2, true); Face(-w / 2, w / 2, d / 2, true);
        Face(-d / 2, d / 2, -w / 2, false); Face(-d / 2, d / 2, w / 2, false);
        return pts;
    }

    static List<P2> RoundCol(P2 c, double dia, double density, double sigma, double coverageFrac = 1.0)
    {
        var pts = new List<P2>();
        double r = dia / 2;
        int n = (int)(2 * Math.PI * r * density);
        for (int i = 0; i < n; i++)
        {
            if (rng.NextDouble() > coverageFrac) continue;
            double a = rng.NextDouble() * 2 * Math.PI;
            double rr = r + Gauss(sigma);
            pts.Add(c + new P2(Math.Cos(a), Math.Sin(a)) * rr);
        }
        return pts;
    }

    static void Clutter(List<P2> pts, P2 c, int n, double range)
    {
        for (int i = 0; i < n; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2 * range, (rng.NextDouble() - 0.5) * 2 * range));
    }


    // a WxD rectangle at angle A is the identical physical shape as a DxW rectangle at angle A+90: pick whichever labeling
    // the fitter's output matches more closely, so the test grades the real column shape, not an arbitrary axis label
    static (double wErr, double dErr, double aErrDeg) BestMatch(double fw, double fd, double fang, double tw, double td, double tang)
    {
        double a1 = Math.Abs(Math.IEEERemainder(fang - tang, Math.PI / 2)) * 180 / Math.PI;
        double e1 = Math.Abs(fw - tw) + Math.Abs(fd - td);
        double a2 = Math.Abs(Math.IEEERemainder(fang - tang - Math.PI / 2, Math.PI / 2)) * 180 / Math.PI;
        double e2 = Math.Abs(fw - td) + Math.Abs(fd - tw);
        return e1 <= e2 ? (fw - tw, fd - td, a1) : (fw - td, fd - tw, a2);
    }
    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    internal static int Run()
    {
        Console.WriteLine("=== Center click: rectangular columns");
        {
            int ok = 0, trials = 200; var wErr = new List<double>(); var dErr = new List<double>(); var aErr = new List<double>(); var cErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(1000 + t);
                double w = 0.3 + rng.NextDouble() * 0.5, d = 0.3 + rng.NextDouble() * 0.5, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(10 + rng.NextDouble() * 5, 5 + rng.NextDouble() * 5);
                var pts = RectCol(c, w, d, ang, 800, 0.003);
                Clutter(pts, c, 150, 1.0);
                var click = c + new P2(Gauss(0.08), Gauss(0.08));
                var r = ColumnFitter.FitCenter(pts, click);
                if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
                ok++;
                var m = BestMatch(r.Width, r.Depth, r.Angle, w, d, ang);
                wErr.Add(m.wErr); dErr.Add(m.dErr); aErr.Add(m.aErrDeg);
                cErr.Add((r.Center - c).Length);
            }
            Check(ok >= 190, $"{ok}/{trials} fitted as rectangular");
            Console.WriteLine($"   width err  mean {wErr.Average() * 1000:+0.00;-0.00}mm rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.00}mm");
            Console.WriteLine($"   depth err  mean {dErr.Average() * 1000:+0.00;-0.00}mm rms {Math.Sqrt(dErr.Select(x => x * x).Average()) * 1000:0.00}mm");
            Console.WriteLine($"   angle err  rms {Math.Sqrt(aErr.Select(x => x * x).Average()):0.00}deg");
            Console.WriteLine($"   center err rms {Math.Sqrt(cErr.Select(x => x * x).Average()) * 1000:0.00}mm");
        }

        Console.WriteLine("\n=== Center click: round columns");
        {
            int ok = 0, trials = 200; var dErr = new List<double>(); var cErr = new List<double>();
            var reasons = new Dictionary<string, int>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(2000 + t);
                double dia = 0.3 + rng.NextDouble() * 0.5;
                var c = new P2(20 + rng.NextDouble() * 5, 8 + rng.NextDouble() * 5);
                var pts = RoundCol(c, dia, 1500, 0.003);
                Clutter(pts, c, 150, 1.0);
                var click = c + new P2(Gauss(0.06), Gauss(0.06));
                var r = ColumnFitter.FitCenter(pts, click);
                if (!r.Success || r.Shape != ColumnShape.Round)
                {
                    if (Environment.GetEnvironmentVariable("PCW_DBG2") == "1" && reasons.Count < 5)
                        Console.WriteLine($"     [t={t}] dia={dia:0.000} click_off=({(click-c).X*1000:0.0},{(click-c).Y*1000:0.0})mm -> {r}");
                    string reason = !r.Success ? "FAIL: " + r.Message : "chose " + r.Shape + " over round (" + r + ")";
                    reasons[reason] = reasons.GetValueOrDefault(reason) + 1;
                    continue;
                }
                ok++; dErr.Add(r.Diameter - dia); cErr.Add((r.Center - c).Length);
            }
            Check(ok >= 190, $"{ok}/{trials} fitted as round");
            foreach (var kv in reasons.OrderByDescending(k => k.Value).Take(6)) Console.WriteLine("   " + kv.Value + " x " + kv.Key);
            Console.WriteLine($"   diameter err  mean {dErr.Average() * 1000:+0.00;-0.00}mm rms {Math.Sqrt(dErr.Select(x => x * x).Average()) * 1000:0.00}mm");
            Console.WriteLine($"   center err    rms {Math.Sqrt(cErr.Select(x => x * x).Average()) * 1000:0.00}mm");
        }

        Console.WriteLine("\n=== Diagonal clicks (two opposite corners)");
        {
            int ok = 0, trials = 150; var wErr = new List<double>(); var dErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(3000 + t);
                double w = 0.3 + rng.NextDouble() * 0.4, d = 0.3 + rng.NextDouble() * 0.4, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(30 + rng.NextDouble() * 5, 12 + rng.NextDouble() * 5);
                var pts = RectCol(c, w, d, ang, 800, 0.003);
                Clutter(pts, c, 150, 1.0);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                P2 cornerA = c - U * (w / 2) - V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                P2 cornerB = c + U * (w / 2) + V * (d / 2) + new P2(Gauss(0.05), Gauss(0.05));
                var r = ColumnFitter.FitDiagonal(pts, cornerA, cornerB);
                if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
                ok++;
                var m = BestMatch(r.Width, r.Depth, r.Angle, w, d, ang);
                wErr.Add(m.wErr); dErr.Add(m.dErr);
            }
            Check(ok >= 140, $"{ok}/{trials} fitted from two opposite-corner clicks");
            Console.WriteLine($"   width err  rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.00}mm   depth err rms {Math.Sqrt(dErr.Select(x => x * x).Average()) * 1000:0.00}mm");
        }

        Console.WriteLine("\n=== Two opposite-side clicks (only 2 faces guaranteed dense; other 2 must be auto-found)");
        {
            int ok = 0, trials = 150; var wErr = new List<double>(); var dErr = new List<double>(); var aErr = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(4000 + t);
                double w = 0.3 + rng.NextDouble() * 0.4, d = 0.3 + rng.NextDouble() * 0.4, ang = rng.NextDouble() * Math.PI / 2;
                var c = new P2(40 + rng.NextDouble() * 5, 15 + rng.NextDouble() * 5);
                var pts = RectCol(c, w, d, ang, 800, 0.003);
                Clutter(pts, c, 150, 1.0);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                P2 clickA = c - U * (w / 2) + new P2(Gauss(0.02), Gauss(0.02));
                P2 clickB = c + U * (w / 2) + new P2(Gauss(0.02), Gauss(0.02));
                var r = ColumnFitter.FitSides(pts, clickA, clickB);
                if (!r.Success) continue;
                ok++;
                var m = BestMatch(r.Width, r.Depth, r.Angle, w, d, ang);
                wErr.Add(m.wErr); dErr.Add(m.dErr); aErr.Add(m.aErrDeg);
            }
            Check(ok >= 130, $"{ok}/{trials} fitted from two opposite-side clicks");
            Console.WriteLine($"   width err  rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.00}mm   depth err rms {Math.Sqrt(dErr.Select(x => x * x).Average()) * 1000:0.00}mm   angle rms {Math.Sqrt(aErr.Select(x => x * x).Average()):0.00}deg");
        }

        Console.WriteLine("\n=== Rejections: clicking empty space / a wall corner must not invent a column");
        {
            int wrong = 0, trials = 60;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(5000 + t);
                var c = new P2(50, 20);
                var pts = new List<P2>(); Clutter(pts, c, 200, 1.2);
                var r = ColumnFitter.FitCenter(pts, c);
                if (r.Success) wrong++;
            }
            Check(wrong == 0, $"empty/clutter-only region: {wrong}/{trials} wrongly produced a column");
        }
        {
            // an L-corner of two walls should not be mistaken for a column
            int wrong = 0, trials = 60;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(6000 + t);
                var c = new P2(60, 25);
                var pts = new List<P2>();
                for (int i = 0; i < 800; i++) pts.Add(c + new P2(rng.NextDouble() * 3, Gauss(0.003)));
                for (int i = 0; i < 800; i++) pts.Add(c + new P2(Gauss(0.003), rng.NextDouble() * 3));
                var r = ColumnFitter.FitCenter(pts, c, new ColumnFitOptions { SearchRadius = 0.6 });
                if (r.Success && r.Width < 1.0 && r.Depth < 1.0) wrong++;
            }
            Check(wrong <= 3, $"wall L-corner: {wrong}/{trials} wrongly produced a small column");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
