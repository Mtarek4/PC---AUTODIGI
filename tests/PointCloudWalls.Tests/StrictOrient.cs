using System; using System.Collections.Generic; using System.Linq; using PointCloudWalls.Core;
// STRICT: the reported Width must be the true extent measured ALONG the reported Angle direction.
// (The old tests accepted a swapped labeling as "the same shape", which is exactly what hid this bug.)
internal static class StrictOrient
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }
    static List<P2> Rect(P2 c, double w, double d, double ang)
    {
        var pts = new List<P2>(); double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
        void Face(double a0, double a1, double off, bool alongU) { int n = (int)((a1 - a0) * 700); for (int i = 0; i < n; i++) { double t = a0 + rng.NextDouble() * (a1 - a0), o = off + Gauss(0.004); pts.Add(alongU ? c + U * t + V * o : c + V * t + U * o); } }
        Face(-w / 2, w / 2, -d / 2, true); Face(-w / 2, w / 2, d / 2, true); Face(-d / 2, d / 2, -w / 2, false); Face(-d / 2, d / 2, w / 2, false);
        for (int i = 0; i < 100; i++) pts.Add(c + new P2((rng.NextDouble() - 0.5) * 2, (rng.NextDouble() - 0.5) * 2));
        return pts;
    }
    // extent of the TRUE rectangle measured along direction theta
    static double TrueExtentAlong(double w, double d, double ang, double theta) =>
        Math.Abs(w * Math.Cos(theta - ang)) + Math.Abs(d * Math.Sin(theta - ang));

    public static int Run()
    {
        int fail = 0;
        foreach (var mode in new[] { "center", "diagonal", "sides" })
        {
            int ok = 0, wrong = 0, trials = 200;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(30000 + t);
                double w = 0.25 + rng.NextDouble() * 0.2, d = 0.6 + rng.NextDouble() * 0.3;     // clearly elongated, like yours
                double ang = rng.NextDouble() * 2 * Math.PI;                                     // ANY angle
                var c = new P2(5, 5);
                var pts = Rect(c, w, d, ang);
                double ca = Math.Cos(ang), sa = Math.Sin(ang); P2 U = new P2(ca, sa), V = new P2(-sa, ca);
                ColumnFitResult r = mode == "center" ? ColumnFitter.FitCenter(pts, c + new P2(Gauss(0.03), Gauss(0.03)))
                    : mode == "diagonal" ? ColumnFitter.FitDiagonal(pts, c - U * (w / 2) - V * (d / 2), c + U * (w / 2) + V * (d / 2))
                    : ColumnFitter.FitSides(pts, c - U * (w / 2), c + U * (w / 2));
                if (!r.Success || r.Shape != ColumnShape.Rectangular) continue;
                double alongAngle = TrueExtentAlong(w, d, ang, r.Angle);
                if (Math.Abs(alongAngle - r.Width) < 0.02) ok++; else wrong++;
            }
            bool pass = wrong == 0;
            if (!pass) fail++;
            Console.WriteLine($"  {(pass ? "PASS" : "FAIL")}  {mode,-8}: Width matches the extent along Angle in {ok} fits, WRONG (90 deg off) in {wrong}");
        }
        return fail;
    }
}
