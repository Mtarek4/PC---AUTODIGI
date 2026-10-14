using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

// The automatic wall search (no clicks): a small synthetic floor plan with KNOWN walls, rotated and shifted at random.
internal static class AutoWallTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    sealed class Plan
    {
        public List<P2> Pts = new List<P2>();
        public double Ang; public P2 Shift;
        public P2 W(double x, double y) { double c = Math.Cos(Ang), s = Math.Sin(Ang); return new P2(x * c - y * s + Shift.X, x * s + y * c + Shift.Y); }
        // a straight scanned surface from (x0, y0) to (x1, y1), plan coordinates
        public void Face(double x0, double y0, double x1, double y1, double density = 400, double sigma = 0.003)
        {
            double len = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            double nx = -(y1 - y0) / len, ny = (x1 - x0) / len;
            int n = (int)(len * density);
            for (int i = 0; i < n; i++)
            {
                double t = rng.NextDouble(), g = Gauss(sigma);
                Pts.Add(W(x0 + (x1 - x0) * t + nx * g, y0 + (y1 - y0) * t + ny * g));
            }
        }
    }

    // Truth (plan coordinates, before rotation):
    //   A  y = 0,    x 0..8,   200 mm, with a 0.9 m door opening at x 3..3.9          (both faces, jambs)
    //   B  x = 8,    y 0..5,   150 mm, L-corner with A                                (both faces)
    //   C  x = 4.5,  y 0.1..5, 100 mm, T-junction on A                                (both faces)
    //   D  y = 5,    x 0..8,   only the room-side face (y = 4.9) is scanned, door at x 5.5..6.4   (one face; body towards +y)
    //   + a door leaf swung open, furniture clutter, and a doubled surface 3 cm off a face of A
    static Plan Build(int seed)
    {
        rng = new Random(seed);
        var p = new Plan { Ang = rng.NextDouble() * Math.PI / 2, Shift = new P2((rng.NextDouble() - 0.5) * 1000, (rng.NextDouble() - 0.5) * 1000) };
        // A
        p.Face(0, -0.10, 3, -0.10); p.Face(3.9, -0.10, 8.075, -0.10);
        p.Face(0, 0.10, 3, 0.10); p.Face(3.9, 0.10, 4.45, 0.10); p.Face(4.55, 0.10, 7.925, 0.10);
        p.Face(3, -0.10, 3, 0.10); p.Face(3.9, -0.10, 3.9, 0.10); p.Face(0, -0.10, 0, 0.10);            // jambs + free end
        // B
        p.Face(8.075, -0.10, 8.075, 5); p.Face(7.925, 0.10, 7.925, 4.9);
        // C
        p.Face(4.45, 0.10, 4.45, 4.9); p.Face(4.55, 0.10, 4.55, 4.9);
        // D: one face, with a door opening at x 5.5..6.4
        p.Face(0, 4.9, 4.45, 4.9); p.Face(4.55, 4.9, 5.5, 4.9); p.Face(6.4, 4.9, 7.925, 4.9);
        // door leaf (40 mm thick, 0.85 m long, swung 55 degrees into the room)
        double la = 55 * Math.PI / 180, lx = Math.Cos(la), ly = Math.Sin(la);
        p.Face(3, 0.10, 3 + 0.85 * lx, 0.10 + 0.85 * ly); p.Face(3 - 0.04 * ly, 0.10 + 0.04 * lx, 3 - 0.04 * ly + 0.85 * lx, 0.10 + 0.04 * lx + 0.85 * ly);
        // a doubled surface: the lower face of A again, 3 cm out, thinner
        p.Face(5, -0.13, 7, -0.13, 150);
        // furniture
        for (int i = 0; i < 400; i++) p.Pts.Add(p.W(0.3 + rng.NextDouble() * 7.4, 0.4 + rng.NextDouble() * 4.2));
        for (int k = 0; k < 4; k++)
        {
            double bx = 0.6 + rng.NextDouble() * 6, by = 1.0 + rng.NextDouble() * 3, r = rng.NextDouble() * Math.PI;
            p.Face(bx, by, bx + 0.6 * Math.Cos(r), by + 0.6 * Math.Sin(r), 200);
        }
        return p;
    }

    static AutoWall Find(AutoWallResult r, Plan p, double x0, double y0, double x1, double y1, double tolEnd)
    {
        P2 a = p.W(x0, y0), b = p.W(x1, y1);
        return r.Walls.FirstOrDefault(w =>
            ((w.Fit.Start - a).Length <= tolEnd && (w.Fit.End - b).Length <= tolEnd) || ((w.Fit.Start - b).Length <= tolEnd && (w.Fit.End - a).Length <= tolEnd));
    }

    public static int Run()
    {
        _fail = 0;
        Console.WriteLine("\n=== Automatic walls: a floor plan with 3 two-face walls, a door, 1 one-face wall, a door leaf and clutter");
        int trials = 40, all3 = 0, bridged = 0, noExtra = 0, singleSeen = 0, oneFaceOk = 0, split = 0, wrongSide = 0;
        var thErr = new List<double>(); var endErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            Plan p = Build(31000 + t);
            var src = new ListPointSource(p.Pts);

            // ---- slice only: the three measured walls, nothing else
            AutoWallResult r = AutoWallFinder.Run(p.Pts, src, new WallDetectOptions(), new WallFitOptions());
            AutoWall A = Find(r, p, 0, 0, 8, 0, 0.12), B = Find(r, p, 8, 0, 8, 5, 0.25), C = Find(r, p, 4.5, 0, 4.5, 5, 0.25);
            if (A != null && B != null && C != null)
            {
                all3++;
                thErr.Add(A.Fit.Thickness - 0.20); thErr.Add(B.Fit.Thickness - 0.15); thErr.Add(C.Fit.Thickness - 0.10);
                endErr.Add(Math.Min((A.Fit.Start - p.W(0, 0)).Length, (A.Fit.End - p.W(0, 0)).Length));
            }
            if (A != null && A.Detected.Openings == 1) bridged++;
            if (r.Walls.Count == 3 && r.Columns.Count == 0) noExtra++;                      // a door opening with its jambs is not a column
            if (r.SingleFaces.Any(f => f.Length > 2.5)) singleSeen++;                       // D is reported, not modeled

            // ---- with the 3D clue for the body side: D becomes a one-face wall on the correct side
            var d2 = new WallDetectOptions { OneFaceThickness = 0.20 };
            d2.BodySide = (a, b) =>
            {
                P2 mid = (a + b) * 0.5, n = (b - a).Normalized().Perp;
                // truth: the room is below D (plan y < 4.9). Only surfaces on the line y = 4.9 get an answer.
                double c = Math.Cos(-p.Ang), s = Math.Sin(-p.Ang);
                P2 Plan(P2 q) { var v = q - p.Shift; return new P2(v.X * c - v.Y * s, v.X * s + v.Y * c); }
                P2 m = Plan(mid), probe = Plan(mid + n * 0.5);
                if (Math.Abs(m.Y - 4.9) > 0.03) return 0;
                return probe.Y > m.Y ? 1 : -1;
            };
            AutoWallResult r2 = AutoWallFinder.Run(p.Pts, src, d2, new WallFitOptions());
            var one = r2.Walls.Where(w => w.OneFace).ToList();
            if (one.Count == 1 && one[0].Detected.Openings == 1 && one[0].Fit.Length > 7.5 && Math.Abs(one[0].Fit.Thickness - 0.20) < 1e-6)
            {
                // centre line 10 cm above the scanned face = body on the +y side
                double c = Math.Cos(-p.Ang), s = Math.Sin(-p.Ang);
                bool right = one.All(w => { var v = (w.Fit.Start + w.Fit.End) * 0.5 - p.Shift; double y = v.X * s + v.Y * c; return Math.Abs(y - 5.0) < 0.02; });
                if (right) oneFaceOk++; else wrongSide++;
            }

            // ---- "the wall does NOT carry on above the opening": two walls, the door is not bridged
            var d3 = new WallDetectOptions { OpeningIsInWall = (a, b, th) => false };
            AutoWallResult r3 = AutoWallFinder.Run(p.Pts, src, d3, new WallFitOptions());
            if (Find(r3, p, 0, 0, 3, 0, 0.12) != null && Find(r3, p, 3.9, 0, 8, 0, 0.12) != null && Find(r3, p, 0, 0, 8, 0, 0.12) == null) split++;
        }
        Check(all3 >= 38, $"{all3}/{trials} found all three two-face walls, ends within tolerance");
        Check(bridged >= 38, $"{bridged}/{trials} carried wall A across the door opening as ONE wall");
        Check(noExtra >= 36, $"{noExtra}/{trials} modeled nothing else (door leaf, doubled surface, furniture, the one-face wall)");
        Check(singleSeen >= 38, $"{singleSeen}/{trials} reported the one-face wall as a single surface instead of guessing its side");
        Check(oneFaceOk >= 36 && wrongSide == 0, $"{oneFaceOk}/{trials} modeled the one-face wall as ONE wall through its door, on the correct side, when the 3D clue is available ({wrongSide} on the wrong side)");
        Check(split >= 36, $"{split}/{trials} kept two separate walls when the wall does not carry on above the opening");
        if (thErr.Count > 0)
        {
            double rms = Math.Sqrt(thErr.Select(x => x * x).Average()) * 1000, worst = thErr.Max(Math.Abs) * 1000;
            Console.WriteLine($"   thickness error rms {rms:0.0} mm, worst {worst:0.0} mm;  free end of wall A: worst {endErr.Max() * 1000:0} mm");
            Check(rms < 3 && worst < 10, "thickness as accurate as the click tools");
        }

        // ---- a pier on a wall is a column, and the wall runs on behind it -------------------------------------------------------
        Console.WriteLine("\n=== A 500 x 250 mm pier standing proud of a 200 mm wall");
        {
            int colOk = 0, wallOk = 0, n3 = 30; var sizeErr = new List<double>();
            for (int t = 0; t < n3; t++)
            {
                rng = new Random(41000 + t);
                var p = new Plan { Ang = rng.NextDouble() * Math.PI / 2, Shift = new P2((rng.NextDouble() - 0.5) * 200, (rng.NextDouble() - 0.5) * 200) };
                p.Face(0, -0.10, 8, -0.10);                                                   // the far face, uninterrupted
                p.Face(0, 0.10, 3.5, 0.10); p.Face(4.0, 0.10, 8, 0.10);                       // the near face, hidden behind the pier
                p.Face(3.5, 0.10, 3.5, 0.35); p.Face(4.0, 0.10, 4.0, 0.35); p.Face(3.5, 0.35, 4.0, 0.35);   // the pier: two sides and a front
                p.Face(0, -0.10, 0, 0.10); p.Face(8, -0.10, 8, 0.10);
                p.Face(0, 3.0, 8, 3.0); p.Face(0, 3.2, 8, 3.2);                               // a second wall, so the building has two...
                p.Face(6, 0.10, 6, 3.0); p.Face(6.15, 0.10, 6.15, 3.0);                       // ...directions
                for (int i = 0; i < 150; i++) p.Pts.Add(p.W(0.3 + rng.NextDouble() * 7.4, 0.5 + rng.NextDouble() * 2.3));
                // "no wall above the opening": only the column may carry the wall across the 0.5 m gap in its near face
                var d = new WallDetectOptions { OpeningIsInWall = (a, b, th) => false };
                AutoWallResult r = AutoWallFinder.Run(p.Pts, new ListPointSource(p.Pts), d, new WallFitOptions());
                var cols = r.Columns.Where(c => (c.Fit.Center - p.W(3.75, 0.225)).Length < 0.08).ToList();
                if (r.Columns.Count == 1 && cols.Count == 1)
                {
                    colOk++;
                    sizeErr.Add(Math.Max(cols[0].Fit.Width, cols[0].Fit.Depth) - 0.50); sizeErr.Add(Math.Min(cols[0].Fit.Width, cols[0].Fit.Depth) - 0.25);
                }
                if (Find(r, p, 0, 0, 8, 0, 0.15) != null) wallOk++;
            }
            Check(colOk >= 27, $"{colOk}/{n3} found the pier as the only column");
            Check(wallOk >= 27, $"{wallOk}/{n3} modeled the wall as ONE wall running on behind the pier");
            if (sizeErr.Count > 0) Check(sizeErr.Max(Math.Abs) < 0.03, $"pier size within {sizeErr.Max(Math.Abs) * 1000:0} mm");
        }

        // ---- the scanned interior: a mirror image in window glass has no floor of its own ----------------------------------------
        Console.WriteLine("\n=== Where the building really is (floor and ceiling scanned)");
        {
            rng = new Random(77);
            var floor = new List<P2>(); var upper = new List<P2>();
            void Fill(List<P2> into, double x0, double y0, double x1, double y1, double perM2)
            { int n = (int)((x1 - x0) * (y1 - y0) * perM2); for (int i = 0; i < n; i++) into.Add(new P2(x0 + rng.NextDouble() * (x1 - x0), y0 + rng.NextDouble() * (y1 - y0))); }
            Fill(floor, 0, 0, 10, 4, 300); Fill(upper, 0, 0, 10, 4, 500);                     // the real room
            Fill(floor, 0, 4.3, 10, 8.3, 15); Fill(upper, 0, 4.3, 10, 8.3, 40);              // its mirror image beyond the glass at y = 4.15: a few percent as dense
            for (int i = 0; i < 3000; i++) floor.Add(new P2(rng.NextDouble() * 10, 8.3 + 0.005 * Gauss(1)));   // the foot of the mirrored back wall: dense, but a line
            Fill(floor, 20, 0, 23, 3, 300);                                                   // a terrace elsewhere: a floor, no ceiling
            var map = new InteriorMap(floor, upper);
            Check(map.Inside(new P2(5, 2)) && map.Inside(new P2(1, 1)) && map.Inside(new P2(9, 3)), "the real room is inside");
            Check(!map.Inside(new P2(5, 6)) && !map.Near(new P2(5, 7), 0.4), "its mirror image is not");
            Check(!map.Near(new P2(5, 8.3), 0.4), "the foot of a mirrored wall is not floor");
            Check(!map.Inside(new P2(21.5, 1.5)), "a floor without a ceiling is not a room");
            Check(map.Share(new P2(0, 4.15), new P2(10, 4.15), -0.9, -0.1) > 0.8 && map.Share(new P2(0, 4.15), new P2(10, 4.15), 0.1, 0.9) < 0.05,
                  "the facade between them has the room on one side only");
        }

        // ---- walls that are already in the model are never modeled twice ---------------------------------------------------------
        Console.WriteLine("\n=== A found wall against walls already modeled by hand");
        {
            WallFitResult Found() => new WallFitResult { Success = true, Start = new P2(0, 0), End = new P2(10, 0), Thickness = 0.20 };
            (P2 a, P2 b, double t)[] One(double x0, double y0, double x1, double y1) => new[] { (new P2(x0, y0), new P2(x1, y1), 0.20) };
            var parts = AutoWallFinder.UnmodeledParts(Found(), One(-0.5, 0.01, 3, 0.02), 0.4);
            Check(parts.Count == 1 && Math.Abs(parts[0].start.X - 3) < 1e-6 && Math.Abs(parts[0].end.X - 10) < 1e-6,
                  "its first 3 m are modeled: the rest starts where the modeled piece stops");
            parts = AutoWallFinder.UnmodeledParts(Found(), One(10.2, 0, 7.5, 0), 0.4);
            Check(parts.Count == 1 && Math.Abs(parts[0].start.X) < 1e-6 && Math.Abs(parts[0].end.X - 7.5) < 1e-6,
                  "its last 2.5 m are modeled (drawn the other way round): the rest stops where the modeled piece starts");
            parts = AutoWallFinder.UnmodeledParts(Found(), One(0, 0, 9.55, 0), 0.4);
            Check(parts.Count == 1 && Math.Abs(parts[0].start.X - 9.55) < 1e-6 && Math.Abs(parts[0].end.X - 10) < 1e-6,
                  "the user clicked 45 cm short of the end of the wall: that last piece is still built");
            parts = AutoWallFinder.UnmodeledParts(Found(), One(4, 0, 6, 0), 0.4);
            Check(parts.Count == 2 && Math.Abs(parts[0].end.X - 4) < 1e-6 && Math.Abs(parts[1].start.X - 6) < 1e-6, "a piece in its middle is modeled: both ends are still built");
            Check(AutoWallFinder.UnmodeledParts(Found(), One(-0.1, 0, 9.9, 0), 0.4).Count == 0, "all of it is modeled: nothing is built twice");
            parts = AutoWallFinder.UnmodeledParts(Found(), One(0, 0.5, 10, 0.5), 0.4);
            Check(parts.Count == 1 && Math.Abs(parts[0].start.X) < 1e-6 && Math.Abs(parts[0].end.X - 10) < 1e-6, "a parallel wall half a metre away is another wall: unchanged");
            parts = AutoWallFinder.UnmodeledParts(Found(), One(5, -2, 5, 2), 0.4);
            Check(parts.Count == 1 && Math.Abs(parts[0].start.X) < 1e-6 && Math.Abs(parts[0].end.X - 10) < 1e-6, "a crossing wall is another wall: unchanged");
        }

        // ---- a door between two DIFFERENT walls on one line: the opening must end up inside one wall ---------------------------
        Console.WriteLine("\n=== A 0.9 m door between a 240 mm wall and a 120 mm wall on the same line (flush on one face)");
        {
            int closed = 0, open = 0, n5 = 30;
            for (int t = 0; t < n5; t++)
            {
                rng = new Random(71000 + t);
                var p = new Plan { Ang = rng.NextDouble() * Math.PI / 2, Shift = new P2((rng.NextDouble() - 0.5) * 200, (rng.NextDouble() - 0.5) * 200) };
                p.Face(0, -0.12, 3, -0.12); p.Face(0, 0.12, 3, 0.12); p.Face(3, -0.12, 3, 0.12); p.Face(0, -0.12, 0, 0.12);           // 240 mm, x 0..3
                p.Face(3.9, -0.12, 7, -0.12); p.Face(3.9, 0.0, 7, 0.0); p.Face(3.9, -0.12, 3.9, 0.0); p.Face(7, -0.12, 7, 0.0);       // 120 mm, x 3.9..7
                p.Face(1.0, 0.12, 1.0, 3.0); p.Face(1.15, 0.12, 1.15, 3.0);                                                         // a second direction
                p.Face(0, 3.0, 7, 3.0); p.Face(0, 3.2, 7, 3.2);
                double c = Math.Cos(-p.Ang), s = Math.Sin(-p.Ang);
                P2 Local(P2 q) { var v = q - p.Shift; return new P2(v.X * c - v.Y * s, v.X * s + v.Y * c); }
                bool Covers(AutoWallResult r) => r.Walls.Any(w =>
                {
                    P2 a = Local(w.Fit.Start), b = Local(w.Fit.End);
                    return Math.Abs(a.Y) < 0.15 && Math.Abs(b.Y) < 0.15 && Math.Min(a.X, b.X) <= 3.05 && Math.Max(a.X, b.X) >= 3.85;
                });
                var src = new ListPointSource(p.Pts);
                if (Covers(AutoWallFinder.Run(p.Pts, src, new WallDetectOptions { Columns = false }, new WallFitOptions()))) closed++;
                if (!Covers(AutoWallFinder.Run(p.Pts, src, new WallDetectOptions { Columns = false, OpeningIsInWall = (a, b, th) => false }, new WallFitOptions()))) open++;
            }
            Check(closed >= 28, $"{closed}/{n5} carried one of the two walls across the door, so the whole opening lies in a single wall");
            Check(open >= 28, $"{open}/{n5} left the gap open when the wall does not carry on above it (the end of a wall, not a door)");
        }

        // ---- a door right beside a crossing wall ------------------------------------------------------------------------------
        Console.WriteLine("\n=== A 0.9 m door between the end of a wall and the side of a crossing wall");
        {
            int closed = 0, open = 0, n6 = 30;
            for (int t = 0; t < n6; t++)
            {
                rng = new Random(81000 + t);
                var p = new Plan { Ang = rng.NextDouble() * Math.PI / 2, Shift = new P2((rng.NextDouble() - 0.5) * 200, (rng.NextDouble() - 0.5) * 200) };
                p.Face(0, -0.10, 3, -0.10); p.Face(0, 0.10, 3, 0.10); p.Face(3, -0.10, 3, 0.10); p.Face(0, -0.10, 0, 0.10);     // the wall, x 0..3, door jamb at 3
                p.Face(3.9, -2, 3.9, 2); p.Face(4.1, -2, 4.1, 2); p.Face(3.9, -2, 4.1, -2); p.Face(3.9, 2, 4.1, 2);            // the crossing wall at x = 4
                double c = Math.Cos(-p.Ang), s = Math.Sin(-p.Ang);
                P2 Local(P2 q) { var v = q - p.Shift; return new P2(v.X * c - v.Y * s, v.X * s + v.Y * c); }
                double Reach(AutoWallResult r)
                {
                    double far = 0;
                    foreach (AutoWall w in r.Walls)
                    {
                        P2 a = Local(w.Fit.Start), b = Local(w.Fit.End);
                        if (Math.Abs(a.Y) < 0.1 && Math.Abs(b.Y) < 0.1) far = Math.Max(far, Math.Max(a.X, b.X));
                    }
                    return far;
                }
                var src = new ListPointSource(p.Pts);
                if (Math.Abs(Reach(AutoWallFinder.Run(p.Pts, src, new WallDetectOptions { Columns = false }, new WallFitOptions())) - 3.9) < 0.06) closed++;
                if (Math.Abs(Reach(AutoWallFinder.Run(p.Pts, src, new WallDetectOptions { Columns = false, OpeningIsInWall = (a, b, th) => false }, new WallFitOptions())) - 3.0) < 0.06) open++;
            }
            Check(closed >= 28, $"{closed}/{n6} carried the wall across the door to the crossing wall");
            Check(open >= 28, $"{open}/{n6} left it where it ends when the wall does not carry on above the gap");
        }

        // ---- a wall that ends in a thicker piece is followed to its real end -------------------------------------------------------
        Console.WriteLine("\n=== A 200 mm wall ending in a 560 mm wide block (a pier between two doors)");
        {
            int reach = 0, n4 = 30;
            for (int t = 0; t < n4; t++)
            {
                rng = new Random(61000 + t);
                var p = new Plan { Ang = rng.NextDouble() * Math.PI / 2, Shift = new P2((rng.NextDouble() - 0.5) * 200, (rng.NextDouble() - 0.5) * 200) };
                p.Face(0, -0.10, 4, -0.10); p.Face(0, 0.10, 4, 0.10); p.Face(0, -0.10, 0, 0.10);                 // the wall, x 0..4
                p.Face(4, 0.10, 4, 0.28); p.Face(4, -0.28, 4, -0.10);                                            // the shoulders of the block
                p.Face(4, 0.28, 4.6, 0.28); p.Face(4, -0.28, 4.6, -0.28); p.Face(4.6, -0.28, 4.6, 0.28);         // its sides and its end face
                p.Face(1.0, 0.10, 1.0, 3.0); p.Face(1.15, 0.10, 1.15, 3.0);                                      // a second direction
                p.Face(0, 3.0, 5, 3.0); p.Face(0, 3.2, 5, 3.2);
                AutoWallResult r = AutoWallFinder.Run(p.Pts, new ListPointSource(p.Pts), new WallDetectOptions { Columns = false }, new WallFitOptions());
                if (Find(r, p, 0, 0, 4.6, 0, 0.08) != null) reach++;
            }
            Check(reach >= 27, $"{reach}/{n4} carried the wall on to the end face of the block instead of stopping where it gets thicker");
        }

        // ---- scans stored on a 1 mm grid must not pull a wall onto the X/Y axis ------------------------------------------------
        Console.WriteLine("\n=== Short rough wall 0.08 - 0.35 degrees off the axis, coordinates rounded to whole millimetres (as many scan files are)");
        int ok = 0, n2 = 100; var angErr = new List<double>();
        for (int t = 0; t < n2; t++)
        {
            rng = new Random(52000 + t);
            double ang = (0.08 + 0.27 * rng.NextDouble()) * Math.PI / 180 * (rng.Next(2) == 0 ? 1 : -1), len = 1.5 + rng.NextDouble(), T = 0.12 + 0.15 * rng.NextDouble();
            var pts = new List<P2>();
            double c = Math.Cos(ang), s = Math.Sin(ang);
            foreach (double off in new[] { -T / 2, T / 2 })
                for (int i = 0; i < (int)(len * 500); i++)
                {
                    double a = (rng.NextDouble() - 0.5) * len, o = off + Gauss(0.0055);
                    pts.Add(new P2(Math.Round((a * c - o * s) * 1000) / 1000, Math.Round((a * s + o * c) * 1000) / 1000));
                }
            WallFitResult r = WallFitter.Fit(pts, new P2(0.3, -T / 2 + 0.01), new P2(0.3, T / 2 - 0.01), new WallFitOptions());
            if (!r.Success) continue;
            double err = Math.IEEERemainder(Math.Atan2(r.Direction.Y, r.Direction.X) - ang, Math.PI) * 180 / Math.PI;
            angErr.Add(err);
            if (Math.Abs(err) < 0.06) ok++;
        }
        Check(ok >= 95, $"{ok}/{n2} measured the true direction within 0.06 deg (not pulled onto the axis)");
        if (angErr.Count > 0) Console.WriteLine($"   angle error rms {Math.Sqrt(angErr.Select(x => x * x).Average()):0.000} deg");
        return _fail;
    }
}
