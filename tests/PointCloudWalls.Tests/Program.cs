using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

// Synthetic "scanned floor-plan slices" with KNOWN truth, to measure alignment accuracy of the solver.
static class Program
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    sealed class Scene
    {
        public List<P2> Pts = new List<P2>();
        public P2 C, E, Nu;
        public double Ang;
        public Scene(P2 c, double ang) { C = c; Ang = ang; E = new P2(Math.Cos(ang), Math.Sin(ang)); Nu = E.Perp; }
        public P2 W(double a, double c) => C + E * a + Nu * c;
        public void Face(double a0, double a1, double c, double density, double sigma = 0.003, double skewProb = 0, double skewScale = 0, int skewSign = 1)
        {
            int n = (int)((a1 - a0) * density);
            for (int i = 0; i < n; i++)
            {
                double cc = c + Gauss(sigma);
                if (skewProb > 0 && rng.NextDouble() < skewProb) cc += skewSign * (-skewScale * Math.Log(1 - rng.NextDouble()));
                Pts.Add(W(a0 + rng.NextDouble() * (a1 - a0), cc));
            }
        }
        public void Cap(double a, double c0, double c1, double density, double sigma = 0.003)
        {
            int n = (int)(Math.Abs(c1 - c0) * density);
            for (int i = 0; i < n; i++) Pts.Add(W(a + Gauss(sigma), c0 + rng.NextDouble() * (c1 - c0)));
        }
        public void Clutter(int n, double aRange, double cRange) { for (int i = 0; i < n; i++) Pts.Add(W((rng.NextDouble() - 0.5) * 2 * aRange, (rng.NextDouble() - 0.5) * 2 * cRange)); }
    }

    sealed class Acc
    {
        public string Name; public int Trials, Ok;
        public List<double> Th = new List<double>(), Ang = new List<double>(), EndA = new List<double>(), EndB = new List<double>(), Lat = new List<double>(), Cen = new List<double>();
        public Dictionary<string, int> Fails = new Dictionary<string, int>();
        public Dictionary<string, int> Kinds = new Dictionary<string, int>();
    }

    static string S(List<double> v, double k, string unit)
    {
        if (v.Count == 0) return "n/a";
        return $"mean {v.Average() * k,7:0.00}{unit}  rms {Math.Sqrt(v.Select(x => x * x).Average()) * k,7:0.00}{unit}  max {v.Max(Math.Abs) * k,7:0.00}{unit}";
    }

    static void Report(Acc a)
    {
        Console.WriteLine($"\n=== {a.Name}   trials={a.Trials} ok={a.Ok}");
        foreach (var kv in a.Fails) Console.WriteLine($"   FAIL x{kv.Value}: {kv.Key}");
        if (a.Kinds.Count > 0) Console.WriteLine("   end kinds: " + string.Join(", ", a.Kinds.Select(k => $"{k.Key}={k.Value}")));
        Console.WriteLine("   thickness  : " + S(a.Th, 1000, " mm"));
        Console.WriteLine("   angle      : " + S(a.Ang, 1, " deg").Replace("deg", "deg"));
        if (a.EndA.Count > 0) Console.WriteLine("   end A pos  : " + S(a.EndA, 1000, " mm"));
        if (a.EndB.Count > 0) Console.WriteLine("   end B pos  : " + S(a.EndB, 1000, " mm"));
        Console.WriteLine("   lateral@end: " + S(a.Lat, 1000, " mm") + "   (centerline offset from truth at the wall ends)");
        Console.WriteLine("   centre off : " + S(a.Cen, 1000, " mm"));
    }

    // truthLo/truthHi: expected along positions of the wall ends (double.NaN = don't score)
    static void Score(Acc acc, WallFitResult r, Scene sc, double T, double truthLo, double truthHi)
    {
        acc.Trials++;
        if (!r.Success) { acc.Fails[r.Message] = acc.Fails.GetValueOrDefault(r.Message) + 1; return; }
        acc.Ok++;
        acc.Th.Add(r.Thickness - T);
        acc.Ang.Add(Math.IEEERemainder(Math.Atan2(r.Direction.Y, r.Direction.X) - sc.Ang, Math.PI) * 180 / Math.PI);
        double s1 = (r.Start - sc.C).Dot(sc.E), s2 = (r.End - sc.C).Dot(sc.E);
        P2 lo = s1 < s2 ? r.Start : r.End, hi = s1 < s2 ? r.End : r.Start;
        double alo = Math.Min(s1, s2), ahi = Math.Max(s1, s2);
        if (!double.IsNaN(truthLo)) acc.EndA.Add(alo - truthLo);
        if (!double.IsNaN(truthHi)) acc.EndB.Add(ahi - truthHi);
        acc.Lat.Add((lo - sc.C).Dot(sc.Nu)); acc.Lat.Add((hi - sc.C).Dot(sc.Nu));
        acc.Cen.Add(0.5 * ((lo - sc.C).Dot(sc.Nu) + (hi - sc.C).Dot(sc.Nu)));
        string k = (s1 < s2 ? r.AtStart : r.AtEnd) + "/" + (s1 < s2 ? r.AtEnd : r.AtStart);
        acc.Kinds[k] = acc.Kinds.GetValueOrDefault(k) + 1;
    }

    // ---------------------------------------------------------------------------------------
    static int Main()
    {
        int N = 100;
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "cad") return CadTests.Run() + PolyTests.Run();
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "auto") return AutoWallTests.Run();
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "side") { SideTests(N); return 0; }
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "short") { ShortTests(N); return 0; }
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "diag") { DiagTests(N); return 0; }
        if (Environment.GetEnvironmentVariable("PCW_ONLY") == "ends") { EndTests(N); return 0; }

        // S1: free ends with end surfaces, door with jambs, crossing wall on one face, parallel wall, clutter
        {
            var acc = new Acc { Name = "S1 free end + door jamb + crossing wall + parallel wall + clutter" };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(100 + t);
                double T = new[] { 0.10, 0.15, 0.20, 0.25, 0.30 }[rng.Next(5)];
                var sc = new Scene(new P2(10 + rng.NextDouble() * 5, 5 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                double gap0 = 0.4, gap1 = 1.3;
                foreach (double c in new[] { -T / 2, T / 2 })
                {
                    Face(sc, -2.5, gap0, c, c < 0);      // left face interrupted by the crossing wall on c<0
                    sc.Face(gap1, 2.5, c, 600);
                }
                sc.Cap(-2.5, -T / 2, T / 2, 600); sc.Cap(2.5, -T / 2, T / 2, 600);
                sc.Cap(gap0, -T / 2, T / 2, 600); sc.Cap(gap1, -T / 2, T / 2, 600);
                foreach (double a in new[] { -1.275, -1.125 }) sc.Cap(a, -T / 2, -T / 2 - 2.0, 600);      // crossing wall faces
                foreach (double c in new[] { -0.775 - T / 2, -0.625 - T / 2 }) sc.Face(-2.5, 2.5, c, 500);
                sc.Clutter(400, 3, 1.5);
                double aSeed = -1.0 + (rng.NextDouble() - 0.5) * 1.0;
                var A = sc.W(aSeed, -T / 2 + Gauss(0.015)); var B = sc.W(aSeed + Gauss(0.02), T / 2 + Gauss(0.015));
                Score(acc, WallFitter.Fit(sc.Pts, A, B), sc, T, -2.5, gap0);
            }
            Report(acc);
        }

        // S2: stem ending at a through wall (T junction). Stem body hides the through wall's near face.
        foreach (bool centerline in new[] { false, true })
        {
            var acc = new Acc { Name = "S2 T-junction, junction end = " + (centerline ? "CENTERLINE of crossing wall" : "FACE of crossing wall") };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(500 + t);
                double T = new[] { 0.10, 0.15, 0.20, 0.25 }[rng.Next(4)];
                double crossT = 0.25, aNear = 2.875;
                var sc = new Scene(new P2(20 + rng.NextDouble() * 5, 8 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-3.0, aNear, c, 600);
                sc.Cap(-3.0, -T / 2, T / 2, 600);
                // through wall: perpendicular, faces at a=aNear and aNear+crossT, spanning c in [-3,3]; near face hidden behind the stem
                sc.Cap(aNear, -3, -T / 2, 600); sc.Cap(aNear, T / 2, 3, 600);
                sc.Cap(aNear + crossT, -3, 3, 600);
                sc.Clutter(200, 3, 2);
                double aSeed = -1.5 + rng.NextDouble() * 3.0;
                var A = sc.W(aSeed, -T / 2 + Gauss(0.012)); var B = sc.W(aSeed + Gauss(0.02), T / 2 + Gauss(0.012));
                var o = new WallFitOptions { JunctionToCenterline = centerline };
                Score(acc, WallFitter.Fit(sc.Pts, A, B, o), sc, T, -3.0, centerline ? aNear + crossT / 2 : aNear);
            }
            Report(acc);
        }

        // S3: long wall (24 m) - alignment accuracy over distance, three configurations
        foreach (var cfg in new[] { ("refine+follow (new default)", true, true), ("refine only, no follow", true, false), ("neither (old behaviour)", false, false) })
        {
            var acc = new Acc { Name = "S3 long 24 m wall, noise 4 mm : " + cfg.Item1 };
            for (int t = 0; t < 40; t++)
            {
                rng = new Random(900 + t);
                double T = 0.2;
                var sc = new Scene(new P2(30 + rng.NextDouble() * 5, 12 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-12, 12, c, 500, 0.004);
                sc.Cap(-12, -T / 2, T / 2, 500); sc.Cap(12, -T / 2, T / 2, 500);
                sc.Clutter(300, 12, 1.5);
                double aSeed = (rng.NextDouble() - 0.5) * 6;
                var A = sc.W(aSeed, -T / 2 + Gauss(0.01)); var B = sc.W(aSeed + Gauss(0.02), T / 2 + Gauss(0.01));
                var o = new WallFitOptions { RefineAngle = cfg.Item2, FollowWall = cfg.Item3 };
                double lo = cfg.Item3 ? -12 : double.NaN, hi = cfg.Item3 ? 12 : double.NaN;
                Score(acc, WallFitter.Fit(sc.Pts, A, B, o), sc, T, lo, hi);
            }
            Report(acc);
        }

        // S4: fuzzy one-sided face
        foreach (bool refine in new[] { true, false })
        {
            var acc = new Acc { Name = "S4 fuzzy face B (one-sided scatter), RefineAngle=" + refine };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(1300 + t);
                double T = 0.2;
                var sc = new Scene(new P2(40 + rng.NextDouble() * 5, 15 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                sc.Face(-3, 3, -T / 2, 600, 0.003);
                sc.Face(-3, 3, T / 2, 600, 0.004, 0.45, 0.012, +1);
                sc.Cap(-3, -T / 2, T / 2, 600); sc.Cap(3, -T / 2, T / 2, 600);
                double aSeed = (rng.NextDouble() - 0.5) * 3;
                var A = sc.W(aSeed, -T / 2 + Gauss(0.01)); var B = sc.W(aSeed + Gauss(0.02), T / 2 + Gauss(0.01));
                var o = new WallFitOptions { RefineAngle = refine };
                Score(acc, WallFitter.Fit(sc.Pts, A, B, o), sc, T, -3, 3);
            }
            Report(acc);
        }
        CenterlineTests(N);
        SideTests(N);
        ShortTests(N);
        DiagTests(N);
        return CadTests.Run() + PolyTests.Run() + ColumnTests.Run() + UnevenTests.Run() + StrictOrient.Run() + MissingSide.Run() + UnevenAngle.Run() + LowDensity.Run() + HeightSelectTests.Run() + OpeningTests.Run() + PersonInDoor.Run() + FloorOffsetTests.Run() + EmbeddedColumnTests.Run() + NoHintTests.Run() + SnapTests.Run() + RoundVsRectTests.Run() + DoorLeafTests.Run() + DoorLeafDepthTests.Run() + AutoWallTests.Run();
    }

    // ------------------------------------------------------------------------------------------------------------
    //  Centerline mode: two clicks along the wall's centre line
    // ------------------------------------------------------------------------------------------------------------
    static void CenterlineTests(int N)
    {
        // CL1: both faces, sloppy clicks (5 cm lateral, +-15 cm along), parallel wall 0.7 m away, clutter
        {
            var acc = new Acc { Name = "CL1 centerline clicks, both faces scanned, sloppy clicks + parallel wall 0.7 m away" };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(2100 + t);
                double T = new[] { 0.10, 0.15, 0.20, 0.25, 0.30 }[rng.Next(5)];
                var sc = new Scene(new P2(10 + rng.NextDouble() * 5, 5 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-3, 3, c, 600);
                sc.Cap(-3, -T / 2, T / 2, 600); sc.Cap(3, -T / 2, T / 2, 600);
                foreach (double c in new[] { 0.7 - T / 2, 0.7 + T / 2 }) sc.Face(-3, 3, c, 600);
                sc.Clutter(300, 3.3, 1.5);
                var A = sc.W(-3 + Gauss(0.15), Gauss(0.05)); var B = sc.W(3 + Gauss(0.15), Gauss(0.05));
                Score(acc, WallFitter.FitAlong(sc.Pts, A, B), sc, T, -3, 3);
            }
            Report(acc);
        }

        // CL2 / CL3: ONE face only (face at c=+T/2 not scanned), with / without an end face
        foreach (bool caps in new[] { true, false })
        {
            var acc = new Acc { Name = "CL2 ONE face only (thickness assumed = true), end faces " + (caps ? "visible" : "NOT visible") };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(2500 + t + (caps ? 0 : 1000));
                double T = new[] { 0.10, 0.15, 0.20, 0.25, 0.30 }[rng.Next(5)];
                var sc = new Scene(new P2(20 + rng.NextDouble() * 5, 8 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                sc.Face(-3, 3, -T / 2, 600);
                if (caps) { sc.Cap(-3, -T / 2, T / 2, 600); sc.Cap(3, -T / 2, T / 2, 600); }
                sc.Clutter(300, 3.3, 1.5);
                var A = sc.W(-3 + Gauss(0.15), Gauss(0.04)); var B = sc.W(3 + Gauss(0.15), Gauss(0.04));
                var o = new WallFitOptions { AssumedThickness = T };
                Score(acc, WallFitter.FitAlong(sc.Pts, A, B, o), sc, T, -3, 3);
            }
            Report(acc);
        }

        // CL4: door opening BETWEEN the clicks, clicks 15 cm beyond the real ends
        {
            var acc = new Acc { Name = "CL4 door opening between the clicks (wall must bridge it), ends snap to the real end faces" };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(3100 + t);
                double T = new[] { 0.15, 0.20, 0.25 }[rng.Next(3)];
                var sc = new Scene(new P2(30 + rng.NextDouble() * 5, 12 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) { sc.Face(-3, -0.5, c, 600); sc.Face(0.5, 3, c, 600); }
                sc.Cap(-3, -T / 2, T / 2, 600); sc.Cap(3, -T / 2, T / 2, 600); sc.Cap(-0.5, -T / 2, T / 2, 600); sc.Cap(0.5, -T / 2, T / 2, 600);
                sc.Clutter(200, 3.3, 1.5);
                var A = sc.W(-3.15 + Gauss(0.05), Gauss(0.04)); var B = sc.W(3.15 + Gauss(0.05), Gauss(0.04));
                Score(acc, WallFitter.FitAlong(sc.Pts, A, B), sc, T, -3, 3);
            }
            Report(acc);
        }

        // CL5: stem meets a through wall; the end click is 10 cm INSIDE the through wall
        {
            var acc = new Acc { Name = "CL5 end click lands inside a crossing wall (end must stop at its face)" };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(3700 + t);
                double T = new[] { 0.10, 0.15, 0.20 }[rng.Next(3)];
                double aNear = 2.875, crossT = 0.25;
                var sc = new Scene(new P2(40 + rng.NextDouble() * 5, 15 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-3.0, aNear, c, 600);
                sc.Cap(-3.0, -T / 2, T / 2, 600);
                sc.Cap(aNear, -3, -T / 2, 600); sc.Cap(aNear, T / 2, 3, 600); sc.Cap(aNear + crossT, -3, 3, 600);
                var A = sc.W(-3 + Gauss(0.1), Gauss(0.04)); var B = sc.W(aNear + 0.10 + Gauss(0.03), Gauss(0.04));
                Score(acc, WallFitter.FitAlong(sc.Pts, A, B), sc, T, -3, aNear);
            }
            Report(acc);
        }

        // CL6: partial wall - the user deliberately clicks 1 m short of the ends
        {
            var acc = new Acc { Name = "CL6 clicks deliberately 1 m short of the real ends (ends must sit at the clicks)" };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(4300 + t);
                double T = 0.2;
                var sc = new Scene(new P2(50 + rng.NextDouble() * 5, 18 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-3, 3, c, 600);
                sc.Cap(-3, -T / 2, T / 2, 600); sc.Cap(3, -T / 2, T / 2, 600);
                double a0 = -2.0 + Gauss(0.01), a1 = 2.0 + Gauss(0.01);
                var A = sc.W(a0, Gauss(0.04)); var B = sc.W(a1, Gauss(0.04));
                // truth = the along positions of the clicks
                Score(acc, WallFitter.FitAlong(sc.Pts, A, B), sc, T, a0, a1);
            }
            Report(acc);
        }

        // CL7: clicks nowhere near a wall -> must FAIL with a message, never invent a wall
        {
            var acc = new Acc { Name = "CL7 clicks 1.2 m away from any wall (must fail, no invented wall)" };
            for (int t = 0; t < 40; t++)
            {
                rng = new Random(5000 + t);
                double T = 0.2;
                var sc = new Scene(new P2(60 + rng.NextDouble() * 5, 20 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-3, 3, c, 600);
                sc.Clutter(300, 3.3, 2.5);
                var A = sc.W(-2, 1.2); var B = sc.W(2, 1.2);
                var r = WallFitter.FitAlong(sc.Pts, A, B);
                acc.Trials++;
                if (!r.Success) { acc.Fails[r.Message] = acc.Fails.GetValueOrDefault(r.Message) + 1; } else { acc.Ok++; }
            }
            Console.WriteLine($"\n=== {acc.Name}   trials={acc.Trials}  wrongly-fitted={acc.Ok}");
            foreach (var kv in acc.Fails) Console.WriteLine($"   FAIL x{kv.Value}: {kv.Key}");
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    //  Which side is the wall body on? (ONE dense face; the other face sparse or missing)
    // ------------------------------------------------------------------------------------------------------------
    static void SideTests(int N)
    {
        // flags: sparse opposite face | crossing wall (far side) | end caps | room clutter | FAR room walls (3-4 m away on the room side)
        //        | wall attached to the scanned face on the room side (interior corner) | click offset toward body (m)
        var scen = new (string name, bool sparse, bool crossing, bool caps, bool clutter, bool farWalls, bool attached, double clickOff, bool stemMid, bool outerCorner)[]
        {
            ("SD1 ONLY a sparse opposite face, clicks ON the dots",                          true,  false, false, false, false, false, 0.0, false, false),
            ("SD2 ONLY a crossing wall starting at the far face, clicks ON the dots",        false, true,  false, false, false, false, 0.0, false, false),
            ("SD3 ONLY end faces (caps), clicks ON the dots",                                false, false, true,  false, false, false, 0.0, false, false),
            ("SD4 your screenshot: sparse face + crossing wall + caps, clicks ON the dots",  true,  true,  true,  false, false, false, 0.0, false, false),
            ("SD5 NO wall cues, room clutter within 1 m of the visible side, clicks ON dots",false, false, false, true,  false, false, 0.0, false, false),
            ("SD6 NO wall cues, clicks 8 cm on the body side (click decides)",               false, false, false, false, false, false, 0.08, false, false),
            ("SD7 sparse face present but clicks 8 cm on the WRONG side (scan must win)",    true,  true,  true,  false, false, false, -0.08, false, false),
            ("SD8 NO wall cues, but the FAR room walls exist 3-4 m away (empty space behind)",false, false, false, false, true,  false, 0.0, false, false),
            ("SD9 interior CORNER: another wall attached to the scanned face on the room side",false, false, false, false, false, true,  0.0, false, false),
            ("SD10 NOTHING to go on at all, clicks ON the dots (must REFUSE, never guess)",  false, false, false, false, false, false, 0.0, false, false),
            ("SD11 wall meeting the scanned face from the room, in the MIDDLE of the wall",      false, false, false, false, false, false, 0.0, true,  false),
            ("SD12 OUTSIDE corner: neighbouring wall at the face end runs toward the BODY side",  false, false, false, false, false, false, 0.0, false, true),
        };
        foreach (var sd in scen)
        {
            int correct = 0, wrong = 0, refused = 0, trials = 0; var kinds = new Dictionary<string, int>(); var th = new List<double>(); var cen = new List<double>();
            for (int t = 0; t < N; t++)
            {
                rng = new Random(7000 + t);
                double T = new[] { 0.15, 0.18, 0.20, 0.25 }[rng.Next(4)];
                double body = rng.Next(2) == 0 ? 1 : -1;                           // which side of the visible face the body is on
                var sc = new Scene(new P2(70 + rng.NextDouble() * 5, 25 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                sc.Face(-3, 3, 0, 600, 0.004);                                     // dense visible face at c = 0
                if (sd.sparse) for (int i = 0; i < 40; i++) sc.Pts.Add(sc.W(0.4 + rng.NextDouble() * 1.4, body * T + Gauss(0.004)));
                if (sd.crossing) foreach (double a in new[] { 1.0, 1.25 }) sc.Cap(a, body * T, body * (T + 2.5), 600);
                if (sd.caps) { sc.Cap(-3, 0, body * T, 300); sc.Cap(3, 0, body * T, 300); }
                if (sd.clutter) for (int i = 0; i < 1500; i++) sc.Pts.Add(sc.W((rng.NextDouble() - 0.5) * 6, -body * (0.05 + rng.NextDouble() * 0.95)));   // room side only
                if (sd.farWalls)
                {
                    // the opposite wall of the room, 3.5 m away on the room side, both faces, plus a side wall
                    sc.Face(-3, 3, -body * 3.5, 600, 0.004); sc.Face(-3, 3, -body * (3.5 + 0.2), 600, 0.004);
                    sc.Cap(-3, -body * 0.0, -body * 3.5, 200);
                }
                if (sd.attached) sc.Cap(3.0, 0, -body * 2.5, 600);
                if (sd.stemMid) sc.Cap(0.3, 0, -body * 2.5, 600);                  // a wall meeting our scanned face from the room, mid-wall
                if (sd.outerCorner) sc.Cap(3.0, 0, body * 2.5, 600);               // neighbouring wall's outer face runs toward the BODY side                 // neighbouring wall's scanned face, on the ROOM side, touching our face
                double lat = body * sd.clickOff;
                var A = sc.W(-2.6 + Gauss(0.05), lat + Gauss(0.008)); var B = sc.W(2.6 + Gauss(0.05), lat + Gauss(0.008));
                if (rng.Next(2) == 0) { var tmp = A; A = B; B = tmp; }             // click direction must not matter
                var o = new WallFitOptions { AssumedThickness = T };
                var r = WallFitter.FitAlong(sc.Pts, A, B, o);
                trials++;
                if (!r.Success) { refused++; kinds["REFUSED"] = kinds.GetValueOrDefault("REFUSED") + 1; continue; }
                double c0 = ((r.Start + r.End) * 0.5 - sc.C).Dot(sc.Nu);
                double err = c0 - body * T / 2;
                if (Math.Abs(err) < 0.02) correct++; else wrong++;
                string k = (r.OneFace ? r.SideSource + (r.ThicknessAssumed ? " / thickness assumed" : " / thickness measured") : "two faces");
                kinds[k] = kinds.GetValueOrDefault(k) + 1;
                th.Add(r.Thickness - T); cen.Add(err);
            }
            Console.WriteLine($"\n=== {sd.name}\n   trials={trials}   CORRECT side: {correct}   WRONG side: {wrong}   refused (asked user to click on the body side): {refused}");
            foreach (var kv in kinds) Console.WriteLine($"   {kv.Value,4} x {kv.Key}");
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    //  SHORT walls in centerline mode (click line too short to give the direction)
    // ------------------------------------------------------------------------------------------------------------
    static void ShortTests(int N)
    {
        var scen = new (string name, double length, int mode)[]
        {
            ("SH1 isolated short wall, 0.25 m long, both faces + end faces", 0.25, 0),
            ("SH2 short wall 0.40 m long between two long parallel walls (0.6 m away)", 0.40, 1),
            ("SH3 short wall 0.30 m long, only ONE face scanned (+ end faces)", 0.30, 2),
            ("SH4 short stub 0.35 m long attached to a long perpendicular wall", 0.35, 3),
            ("SH5 small piece 0.20 m long, both faces", 0.20, 0),
        };
        foreach (var sd in scen)
        {
            var acc = new Acc { Name = sd.name };
            for (int t = 0; t < N; t++)
            {
                rng = new Random(9100 + t);
                double T = new[] { 0.15, 0.20, 0.25 }[rng.Next(3)];
                double L2 = sd.length / 2;
                var sc = new Scene(new P2(80 + rng.NextDouble() * 5, 30 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                double dens = 400;
                if (sd.mode == 2) { sc.Face(-L2, L2, -T / 2, dens); }
                else { foreach (double c in new[] { -T / 2, T / 2 }) sc.Face(-L2, L2, c, dens); }
                sc.Cap(-L2, -T / 2, T / 2, dens); sc.Cap(L2, -T / 2, T / 2, dens);
                if (sd.mode == 1) foreach (double c in new[] { -0.6 - T / 2, -0.6 + T / 2, 0.6 - T / 2, 0.6 + T / 2 }) sc.Face(-2.5, 2.5, c, 400);
                if (sd.mode == 3)
                {
                    // through wall perpendicular, its near face at the stub's far end; the stub ends where that face is
                    double aNear = L2; foreach (double c in new[] { 0.0 }) { }
                    sc.Cap(aNear, -2, -T / 2, dens); sc.Cap(aNear, T / 2, 2, dens); sc.Cap(aNear + 0.25, -2, 2, dens);
                    sc.Pts.RemoveAll(p => Math.Abs((p - sc.C).Dot(sc.E) - L2) < 0.02 && Math.Abs((p - sc.C).Dot(sc.Nu)) < T / 2 - 0.02);
                }
                sc.Clutter(60, 1.0, 1.0);
                double j = 0.03;
                var A = sc.W(-L2 + Gauss(j), Gauss(j)); var B = sc.W(L2 + Gauss(j), Gauss(j));
                if (rng.Next(2) == 0) { var tmp = A; A = B; B = tmp; }
                var o = new WallFitOptions { AssumedThickness = T };
                var rr = WallFitter.FitAlong(sc.Pts, A, B, o);
                Score(acc, rr, sc, T, -L2, L2);
                if (rr.Success) { string kk = "[" + (rr.OneFace ? rr.SideSource + (rr.ThicknessAssumed ? "/assumed" : "/measured") : "two faces") + "]"; acc.Kinds[kk] = acc.Kinds.GetValueOrDefault(kk) + 1; }
            }
            Report(acc);
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    //  DIAGONAL mode: start click on one side of the wall, end click on the other side
    // ------------------------------------------------------------------------------------------------------------
    static void DiagTests(int N)
    {
        // name, wall length, both faces scanned?, fuzzy (sigma m), busy neighbour on body side?, clicks on same line (not a diagonal)?
        var scen = new (string name, double length, bool both, double sigma, bool busyBody, bool sameLine)[]
        {
            ("DG1 both faces scanned, diagonal clicks",                                   6.0, true,  0.004, false, false),
            ("DG2 ONE face scanned; 2nd click where the missing face should be",         6.0, false, 0.004, false, false),
            ("DG3 like DG2 but the FUZZY real-scan way (sigma 1.5 cm)",                    6.0, false, 0.015, false, false),
            ("DG4 ONE face + BUSY neighbouring wall on the body side (fooled 'empty space')", 6.0, false, 0.004, true,  false),
            ("DG5 short wall 0.30 m, one face, clicks corner to corner",                  0.30, false, 0.004, false, false),
            ("DG6 clicks on the same line, one face, no cues (must NOT guess)",            6.0, false, 0.004, false, true),
        };
        foreach (var sd in scen)
        {
            int correct = 0, wrong = 0, refused = 0; var thErr = new List<double>(); var cen = new List<double>(); var kinds = new Dictionary<string, int>();
            for (int t = 0; t < N; t++)
            {
                rng = new Random(11000 + t);
                double T = new[] { 0.15, 0.18, 0.20, 0.25 }[rng.Next(4)];
                double L2 = sd.length / 2;
                var sc = new Scene(new P2(90 + rng.NextDouble() * 5, 35 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                double dens = sd.length < 1 ? 400 : 600;
                double vis = rng.Next(2) == 0 ? -1 : 1;                      // which face (c = -T/2 or +T/2) is the scanned one
                sc.Face(-L2, L2, vis * T / 2, dens, sd.sigma);
                if (sd.both) sc.Face(-L2, L2, -vis * T / 2, dens, sd.sigma);
                sc.Cap(-L2, -T / 2, T / 2, dens); sc.Cap(L2, -T / 2, T / 2, dens);
                if (sd.busyBody)
                {
                    // the other room's walls: a crossing wall (both faces) and a long parallel wall, all on the body side
                    double b = -vis;
                    foreach (double a in new[] { 1.0, 1.25 }) sc.Cap(a, b * T / 2 + b * 0.25, b * (T / 2 + 2.5), 600);
                    sc.Face(-L2, L2, b * 1.5, 600, sd.sigma); sc.Face(-L2, L2, b * 1.7, 600, sd.sigma);
                }
                // clicks
                double j = 0.02;
                P2 A, B;
                if (sd.sameLine) { A = sc.W(-L2 + Gauss(0.05), vis * T / 2 + Gauss(j)); B = sc.W(L2 + Gauss(0.05), vis * T / 2 + Gauss(j)); }
                else
                {
                    // start on the scanned side, end on the opposite side (where the missing face should be); or reversed
                    A = sc.W(-L2 + Gauss(0.03), vis * T / 2 + Gauss(j)); B = sc.W(L2 + Gauss(0.03), -vis * T / 2 + Gauss(j));
                    if (rng.Next(2) == 0) { var tmp = A; A = B; B = tmp; }
                }
                var o = new WallFitOptions();
                var r = WallFitter.FitDiagonal(sc.Pts, A, B, o);
                if (!r.Success) { refused++; string k0 = "REFUSED: " + (r.Message.Length > 60 ? r.Message.Substring(0, 60) : r.Message); kinds[k0] = kinds.GetValueOrDefault(k0) + 1; continue; }
                double c0 = ((r.Start + r.End) * 0.5 - sc.C).Dot(sc.Nu);
                double err = c0;                                             // truth: wall centre is at c = 0
                if (Math.Abs(err) < 0.03) correct++; else wrong++;
                string k = (r.OneFace ? r.SideSource + " / thickness: " + r.ThicknessSource : "two faces");
                kinds[k] = kinds.GetValueOrDefault(k) + 1;
                thErr.Add(r.Thickness - T); cen.Add(err);
            }
            Console.WriteLine($"\n=== {sd.name}\n   trials={N}   CORRECT: {correct}   WRONG: {wrong}   refused: {refused}");
            foreach (var kv in kinds) Console.WriteLine($"   {kv.Value,4} x {kv.Key}");
            Console.WriteLine("   thickness : " + S(thErr, 1000, " mm") + "\n   centre off: " + S(cen, 1000, " mm"));
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    //  Where do the wall ENDS go, in centerline vs diagonal mode?
    // ------------------------------------------------------------------------------------------------------------
    static void EndTests(int N)
    {
        foreach (bool oneFace in new[] { false, true })
            foreach (double off in new[] { 0.30, 0.60, 1.20 })
                foreach (var mode in new[] { EndMode.SnapToScan, EndMode.Automatic, EndMode.ExactClicks })
                    foreach (int pick in new[] { 1, 2 })
                    {
                        var eLo = new List<double>(); var eHi = new List<double>(); var kinds = new Dictionary<string, int>(); int ok = 0;
                        for (int t = 0; t < 40; t++)
                        {
                            rng = new Random(15000 + t);
                            double T = 0.2, L2 = 3.0;
                            var sc = new Scene(new P2(100 + rng.NextDouble() * 5, 40 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                            sc.Face(-L2, L2, -T / 2, 600);
                            if (!oneFace) sc.Face(-L2, L2, T / 2, 600);
                            sc.Cap(-L2, -T / 2, T / 2, 600); sc.Cap(L2, -T / 2, T / 2, 600);
                            double a0 = -L2 + off, a1 = L2 - off;
                            P2 A, B;
                            if (pick == 1) { A = sc.W(a0, 0.0); B = sc.W(a1, 0.0); }
                            else { A = sc.W(a0, -T / 2); B = sc.W(a1, T / 2); }
                            var o = new WallFitOptions { Ends = mode };
                            var r = pick == 1 ? WallFitter.FitAlong(sc.Pts, A, B, o) : WallFitter.FitDiagonal(sc.Pts, A, B, o);
                            if (!r.Success) continue;
                            ok++;
                            double s1 = (r.Start - sc.C).Dot(sc.E), s2 = (r.End - sc.C).Dot(sc.E);
                            eLo.Add(Math.Min(s1, s2) - (-L2)); eHi.Add(Math.Max(s1, s2) - L2);
                            string k = r.AtStart + "/" + r.AtEnd; kinds[k] = kinds.GetValueOrDefault(k) + 1;
                        }
                        Console.WriteLine($"{(oneFace ? "ONE face " : "TWO faces")} clicks {off * 100,4:0} cm short of the ends | {mode,-12} | {(pick == 1 ? "CENTERLINE" : "DIAGONAL  ")} ok={ok}  error at the two real ends (mm): {(eLo.Count == 0 ? "n/a" : (eLo.Average(Math.Abs) * 1000).ToString("0"))} / {(eHi.Count == 0 ? "n/a" : (eHi.Average(Math.Abs) * 1000).ToString("0"))}   {string.Join(", ", kinds.Select(k => k.Key + "=" + k.Value))}");
                    }

        // a LONG wall: 14 m, clicks only 2 m apart in the middle: automatic must follow it to both real ends
        foreach (int pick in new[] { 1, 2 })
        {
            var eLo = new List<double>(); var eHi = new List<double>(); int ok = 0;
            for (int t = 0; t < 30; t++)
            {
                rng = new Random(16000 + t);
                double T = 0.2, L2 = 7.0;
                var sc = new Scene(new P2(100 + rng.NextDouble() * 5, 40 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                sc.Face(-L2, L2, -T / 2, 500, 0.004);                     // one face only
                sc.Cap(-L2, -T / 2, T / 2, 500); sc.Cap(L2, -T / 2, T / 2, 500);
                double c0 = (rng.NextDouble() - 0.5) * 4;
                P2 A = pick == 1 ? sc.W(c0 - 1.0, 0.0) : sc.W(c0 - 1.0, -T / 2), B = pick == 1 ? sc.W(c0 + 1.0, 0.0) : sc.W(c0 + 1.0, T / 2);
                var r = pick == 1 ? WallFitter.FitAlong(sc.Pts, A, B, new WallFitOptions { Ends = EndMode.Automatic })
                                  : WallFitter.FitDiagonal(sc.Pts, A, B, new WallFitOptions { Ends = EndMode.Automatic });
                if (!r.Success) continue;
                ok++;
                double s1 = (r.Start - sc.C).Dot(sc.E), s2 = (r.End - sc.C).Dot(sc.E);
                eLo.Add(Math.Min(s1, s2) + L2); eHi.Add(Math.Max(s1, s2) - L2);
            }
            Console.WriteLine($"LONG 14 m wall, clicks 2 m apart in the middle | Automatic | {(pick == 1 ? "CENTERLINE" : "DIAGONAL  ")} ok={ok}  error at the two real ends (mm): {(eLo.Count == 0 ? "n/a" : (eLo.Average(Math.Abs) * 1000).ToString("0"))} / {(eHi.Count == 0 ? "n/a" : (eHi.Average(Math.Abs) * 1000).ToString("0"))}");
        }

        // door opening in the middle: automatic stops at the door jamb (a real end face); clicks on the left part only
        {
            var eLo = new List<double>(); var eHi = new List<double>(); int ok = 0;
            for (int t = 0; t < 30; t++)
            {
                rng = new Random(17000 + t);
                double T = 0.2, L2 = 3.0;
                var sc = new Scene(new P2(100 + rng.NextDouble() * 5, 40 + rng.NextDouble() * 5), rng.NextDouble() * Math.PI);
                foreach (double c in new[] { -T / 2, T / 2 }) { sc.Face(-L2, -0.45, c, 600); sc.Face(0.45, L2, c, 600); }
                sc.Cap(-L2, -T / 2, T / 2, 600); sc.Cap(L2, -T / 2, T / 2, 600); sc.Cap(-0.45, -T / 2, T / 2, 600); sc.Cap(0.45, -T / 2, T / 2, 600);
                var r = WallFitter.FitDiagonal(sc.Pts, sc.W(-2.0, -T / 2), sc.W(-1.0, T / 2), new WallFitOptions { Ends = EndMode.Automatic });
                if (!r.Success) continue;
                ok++;
                double s1 = (r.Start - sc.C).Dot(sc.E), s2 = (r.End - sc.C).Dot(sc.E);
                eLo.Add(Math.Min(s1, s2) + L2); eHi.Add(Math.Max(s1, s2) - (-0.45));
            }
            Console.WriteLine($"DOOR opening at 0.9 m wide, clicks on the left part | Automatic | DIAGONAL ok={ok}  error at (wall end, door jamb) (mm): {(eLo.Count == 0 ? "n/a" : (eLo.Average(Math.Abs) * 1000).ToString("0"))} / {(eHi.Count == 0 ? "n/a" : (eHi.Average(Math.Abs) * 1000).ToString("0"))}");
        }
    }

    // face segment helper for S1: `skipCrossing` interrupts the face where the crossing wall meets it
    static void Face(Scene sc, double a0, double a1, double c, bool skipCrossing)
    {
        if (!skipCrossing) { sc.Face(a0, a1, c, 600); return; }
        sc.Face(a0, -1.275, c, 600);
        sc.Face(-1.125, a1, c, 600);
    }
}
