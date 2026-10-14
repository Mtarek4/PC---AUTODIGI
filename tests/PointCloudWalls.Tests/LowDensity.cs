using System; using System.Collections.Generic; using System.Linq; using PointCloudWalls.Core;
internal static class LowDensity
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> Rect(P2 c, double w, double d, double ang, double density, double sigma)
    {
        var pts = new List<P2>(); double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double a0, double a1, double off, bool alongU)
        {
            int n = (int)((a1 - a0) * density);
            for (int i = 0; i < n; i++) { double t = a0 + rng.NextDouble() * (a1 - a0), o = off + Gauss(sigma); pts.Add(alongU ? c + U * t + V * o : c + V * t + U * o); }
        }
        Face(-w / 2, w / 2, -d / 2, true); Face(-w / 2, w / 2, d / 2, true); Face(-d / 2, d / 2, -w / 2, false); Face(-d / 2, d / 2, w / 2, false);
        for (int i = 0; i < 40; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2, (rng.NextDouble() - 0.5) * 2));
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        foreach (var mode in new[] { "center", "sides" })
        {
            foreach (double density in new[] { 15.0, 25.0, 40.0, 60.0 })    // points per metre of edge - baseline tests used 400-800
            {
                int ok = 0, trials = 150; var lenErr = new List<double>(); var reasons = new Dictionary<string, int>();
                for (int t = 0; t < trials; t++)
                {
                    rng = new Random((int)(60000 + density * 10) + t);
                    double w = 0.3 + rng.NextDouble() * 0.15, d = 0.8 + rng.NextDouble() * 0.4, ang = rng.NextDouble() * Math.PI;
                    var c = new P2(5, 5);
                    var pts = Rect(c, w, d, ang, density, 0.005);
                    double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                    ColumnFitResult r = mode == "center" ? ColumnFitter.FitCenter(pts, c + new P2(Gauss(0.03), Gauss(0.03)))
                                                          : ColumnFitter.FitSides(pts, c - U * (w / 2), c + U * (w / 2));
                    if (!r.Success || r.Shape != ColumnShape.Rectangular)
                    {
                        string k = !r.Success ? r.Message : "chose " + r.Shape;
                        reasons[k] = reasons.GetValueOrDefault(k) + 1;
                        continue;
                    }
                    ok++;
                    double fittedLen = Math.Max(r.Width, r.Depth);
                    lenErr.Add(fittedLen - d);
                }
                double meanErr = lenErr.Count > 0 ? lenErr.Average() : double.NaN;
                double rms = lenErr.Count > 0 ? Math.Sqrt(lenErr.Select(x => x * x).Average()) : double.NaN;
                Console.WriteLine($"  {mode,-7} density={density,5:0}pts/m: {ok,3}/{trials} fitted   length err mean {meanErr * 1000,6:+0.0;-0.0}mm  rms {rms * 1000,5:0.0}mm");
                // 15 pts/m is at the edge of what is statistically detectable at all (a short edge may carry only 3-5 points
                // total) - informational only below 20 pts/m, a hard pass/fail requirement from there up.
                bool extreme = density < 20;
                if (ok < trials * 0.85) { if (!extreme) _fail++; Console.WriteLine("    " + (extreme ? "(informational)" : "FAIL") + ": too many outright failures"); foreach (var kv in reasons.OrderByDescending(k => k.Value).Take(4)) Console.WriteLine("      " + kv.Value + " x " + kv.Key); }
                if (lenErr.Count > 0 && meanErr < -0.015) { if (!extreme) _fail++; Console.WriteLine("    " + (extreme ? "(informational)" : "FAIL") + ": still systematically short by >15mm"); }
            }
        }
        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " CHECKS FAILED");
        return _fail;
    }
}
