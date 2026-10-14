using System; using System.Collections.Generic; using System.Linq; using PointCloudWalls.Core;
internal static class UnevenAngle
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    // one edge scanned MUCH more densely (overlapping passes / debris), like the screenshot's thick bottom band
    static List<P2> RectOneThickEdge(P2 c, double w, double d, double ang, double thickEdgeMult, double thickEdgeSigma)
    {
        var pts = new List<P2>(); double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double a0, double a1, double off, bool alongU, double density, double sigma)
        {
            int n = (int)((a1 - a0) * density);
            for (int i = 0; i < n; i++) { double t = a0 + rng.NextDouble() * (a1 - a0), o = off + Gauss(sigma); pts.Add(alongU ? c + U * t + V * o : c + V * t + U * o); }
        }
        Face(-w / 2, w / 2, -d / 2, true, 400, 0.004);                               // top: normal, clean
        Face(-w / 2, w / 2, d / 2, true, 400 * thickEdgeMult, thickEdgeSigma);        // bottom: MUCH denser, noisier band
        Face(-d / 2, d / 2, -w / 2, false, 400, 0.004);                              // left: normal
        Face(-d / 2, d / 2, w / 2, false, 400, 0.004);                               // right: normal
        for (int i = 0; i < 100; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2.2, (rng.NextDouble() - 0.5) * 2.2));
        return pts;
    }

    public static int Run()
    {
        Console.WriteLine("=== Rectangle with ONE much denser/noisier edge (4x points, 2x spread) - like the screenshot's bottom band");
        int ok = 0, trials = 200; var aErr = new List<double>(); var reasons = new Dictionary<string, int>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(50000 + t);
            double w = 0.9 + rng.NextDouble() * 0.4, d = 0.4 + rng.NextDouble() * 0.3, ang = rng.NextDouble() * Math.PI;
            var c = new P2(5, 5);
            var pts = RectOneThickEdge(c, w, d, ang, 4.0, 0.008);
            var r = ColumnFitter.FitCenter(pts, c + new P2(Gauss(0.03), Gauss(0.03)));
            if (!r.Success || r.Shape != ColumnShape.Rectangular)
            {
                string k = !r.Success ? r.Message : "chose " + r.Shape;
                reasons[k] = reasons.GetValueOrDefault(k) + 1;
                continue;
            }
            ok++;
            double a1 = Math.Abs(Math.IEEERemainder(r.Angle - ang, Math.PI)) * 180 / Math.PI;
            double a2 = Math.Abs(Math.IEEERemainder(r.Angle - ang - Math.PI / 2, Math.PI)) * 180 / Math.PI;   // width/depth may be swapped in labeling
            aErr.Add(Math.Min(a1, a2));
        }
        Check(ok >= 190, $"{ok}/{trials} fitted");
        foreach (var kv in reasons) Console.WriteLine("   " + kv.Value + " x " + kv.Key);
        if (aErr.Count > 0)
        {
            Console.WriteLine($"   angle err  mean {aErr.Average():0.00}deg  max {aErr.Max():0.00}deg  rms {Math.Sqrt(aErr.Select(x => x * x).Average()):0.00}deg");
            Check(aErr.Average() < 0.5, "angle no longer dragged by the dense/noisy edge (mean well under 0.5 deg)");
        }
        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
