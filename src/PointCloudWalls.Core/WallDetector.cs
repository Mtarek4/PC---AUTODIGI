using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Core
{
    /// <summary>Settings of the automatic wall search.</summary>
    public sealed class WallDetectOptions
    {
        /// <summary>Two parallel surfaces closer than this are never a wall: a doubled surface (two scans slightly out of register, a board
        /// or a frame standing proud of the wall) looks exactly like a very thin wall.</summary>
        public double MinThickness = 0.07;
        /// <summary>Two parallel surfaces further apart than this are never taken for the two faces of ONE wall (that is a corridor, a niche, two walls).</summary>
        public double MaxThickness = 0.50;
        /// <summary>Shortest wall that is reported (m).</summary>
        public double MinLength = 0.40;
        /// <summary>Point-to-face distance (m) that still counts as "on the surface" - the same number the click tools use.</summary>
        public double InlierTol = 0.010;
        /// <summary>Gaps in a surface up to this (m) are bridged - the same number the click tools use.</summary>
        public double MaxGap = 0.30;
        /// <summary>Two pieces of the same wall (same line, same thickness) with an opening up to this wide (m) between them are ONE wall:
        /// a door or window opening, exactly like clicking the wall's two far ends by hand. 0 = never bridge.</summary>
        public double BridgeOpening = 1.20;
        /// <summary>How many wall directions are searched (a normal building has 2).</summary>
        public int MaxDirections = 4;
        /// <summary>Optional second opinion on an opening wider than MaxGap, from the full 3D cloud (the slice alone cannot tell a door from the end
        /// of a wall): given the two centre-line points where the wall pieces stop and the wall thickness, true = the wall carries on above the
        /// opening (bridge it), false = it does not (two separate walls). Not set: openings up to BridgeOpening are bridged.</summary>
        public Func<P2, P2, double, bool> OpeningIsInWall;

        /// <summary>Diagnostics: one line per decision (which side of a single surface the wall is on, and why).</summary>
        public Action<string> Log;

        // ---- columns
        /// <summary>Also find rectangular columns: free-standing ones, and piers standing proud of a wall. A wall runs on behind a
        /// column instead of stopping at it.</summary>
        public bool Columns = true;
        /// <summary>Smallest / largest column side (m). Smaller bumps on a wall (window mullions, pipes) are not columns.</summary>
        public double ColumnMinSize = 0.20, ColumnMaxSize = 1.50;

        /// <summary>From the 3D cloud: the share (0..1) of a strip beside the line A -> B in which the FLOOR is scanned. The strip lies
        /// between the two given distances from the line (m): positive = on the left of A -> B, negative = on the right.
        /// Used to tell a real room from a mirror image in window glass or from something outside the building (no floor there), and to
        /// find the room side of a wall scanned from one side. Not set: neither check is made.</summary>
        public Func<P2, P2, double, double, double> FloorShare;

        // ---- walls scanned from ONE side only (used by AutoWallFinder)
        /// <summary>Model walls of which only one face is scanned (when their body side can be decided).</summary>
        public bool OneFaceWalls = true;
        /// <summary>Which side of a single scanned surface (A -> B) the wall body is on, from the full 3D cloud: +1 = on the left of A -> B,
        /// -1 = on the right, 0 = cannot tell. Not set: such walls are never modeled automatically - the slice alone cannot tell the room
        /// from the wall, and a wall on the wrong side is worse than no wall.</summary>
        public Func<P2, P2, int> BodySide;
        /// <summary>Thickness (m) given to a wall scanned from one side only.</summary>
        public double OneFaceThickness = 0.20;
        /// <summary>Shortest single surface (m) that is modeled as a one-face wall.</summary>
        public double OneFaceMinLength = 1.0;
    }

    /// <summary>One wall found by the automatic search: where to "click" it. The precise fit is still done by WallFitter.</summary>
    public sealed class DetectedWall
    {
        public P2 Start, End;               // centre line, world meters
        public double Thickness;
        /// <summary>Share of the length where BOTH faces are scanned (door openings count as missing).</summary>
        public double Coverage;
        /// <summary>Door/window openings bridged inside this wall.</summary>
        public int Openings;
        public int Points;
        public double Length => (End - Start).Length;
    }

    /// <summary>A rectangular column found by the automatic search: two opposite corners, as a user would click them in Column Diagonal.</summary>
    public sealed class DetectedColumn
    {
        public P2 CornerA, CornerB;         // world meters
        /// <summary>Size along the wall it stands on / out of that wall (for a pier: the part that stands proud of the wall).</summary>
        public double Width, Depth;
        /// <summary>Direction (radians) of the Width side.</summary>
        public double AngleRad;
        /// <summary>All four sides scanned. false = a pier on a wall: its back is inside the wall.</summary>
        public bool FreeStanding;
        public P2 Center => (CornerA + CornerB) * 0.5;

        /// <summary>Is the point inside the column (grown by a margin)?</summary>
        public bool Contains(P2 p, double margin)
        {
            var e = new P2(Math.Cos(AngleRad), Math.Sin(AngleRad));
            P2 v = p - Center;
            return Math.Abs(v.Dot(e)) <= Width / 2 + margin && Math.Abs(v.Dot(e.Perp)) <= Depth / 2 + margin;
        }
    }

    /// <summary>A straight scanned surface (one face of something).</summary>
    public sealed class DetectedFace
    {
        public P2 A, B;
        public int Points;
        public double Length => (B - A).Length;
    }

    public sealed class WallDetectResult
    {
        public List<DetectedWall> Walls = new List<DetectedWall>();
        /// <summary>Every straight surface found (diagnostics).</summary>
        public List<DetectedFace> Faces = new List<DetectedFace>();
        /// <summary>Stretches of surface (30 cm and longer) with no matching opposite face: walls scanned from one side only, or the
        /// continuation of a wall whose second face is hidden further on.</summary>
        public List<DetectedFace> SingleFaces = new List<DetectedFace>();
        public List<DetectedColumn> Columns = new List<DetectedColumn>();
        /// <summary>The short faces the column search worked with (diagnostics).</summary>
        public List<DetectedFace> ShortFaces = new List<DetectedFace>();
        public List<double> DirectionsDeg = new List<double>();
    }

    /// <summary>
    /// Finds the walls in a whole 2D slice of a point cloud WITHOUT any clicks: the wall directions of the building, then every straight
    /// surface along them, then pairs of surfaces a wall-thickness apart with nothing in between. Each result is only a rough centre line
    /// and thickness - the same thing a user gives the click tools - so the exact position, ends and quality still come from WallFitter.
    /// Pure math, no Revit dependency. Units: meters.
    /// </summary>
    public static class WallDetector
    {
        private const double WallStep = 0.10;  // step along a surface when looking for walls
        private const double ColumnStep = 0.025;   // ... and for the short faces of columns
        private const double CBin = 0.01;      // step across it

        // one wall direction: every point in its (along, across) frame, bucketed in steps of ABin along it
        private sealed class Fam
        {
            public P2 O, E, N;
            public double[] A, C;
            public double AMin, ABin;
            public int NCols;
            public int[] Order, ColStart;
            public double MaxSlope;
            public P2 World(double a, double c) => O + E * a + N * c;
            public int Col(double a) => Math.Max(0, Math.Min(NCols - 1, (int)Math.Floor((a - AMin) / ABin)));
        }

        private sealed class Face
        {
            public int Fam;
            public double A0, A1;
            public double ARef, CRef, K;                  // c(a) = CRef + K * (a - ARef)
            public double[] Alongs;                       // sorted along-positions of its points
            public List<(double lo, double hi)> Used = new List<(double lo, double hi)>();
            public double At(double a) => CRef + K * (a - ARef);
            public int CountIn(double lo, double hi) => Lower(Alongs, hi) - Lower(Alongs, lo);
            public double FirstAtOrAfter(double x) { int i = Lower(Alongs, x); return i < Alongs.Length ? Alongs[i] : double.NaN; }
            public double LastAtOrBefore(double x) { int i = Lower(Alongs, x + 1e-9) - 1; return i >= 0 ? Alongs[i] : double.NaN; }
        }

        private sealed class Piece
        {
            public int Fam;
            public Face Lo, Hi;
            public double X0, X1, Thickness;
            public int Points;
            public double Center(double a) => 0.5 * (Lo.At(a) + Hi.At(a));
            public double Slope => 0.5 * (Lo.K + Hi.K);
        }

        private struct Peak { public int Col; public double C; public int Cnt; }

        private sealed class Chain
        {
            public List<Peak> P = new List<Peak>();
            public int LastCol; public double LastC, Slope;
        }

        private static int Lower(double[] s, double x)
        {
            int lo = 0, hi = s.Length;
            while (lo < hi) { int m = (lo + hi) / 2; if (s[m] < x) lo = m + 1; else hi = m; }
            return lo;
        }

        // =====================================================================================
        public static WallDetectResult Detect(IReadOnlyList<P2> pts, WallDetectOptions o = null)
        {
            o = o ?? new WallDetectOptions();
            var res = new WallDetectResult();
            if (pts == null || pts.Count < 100) return res;

            double ox = 0, oy = 0;
            foreach (var p in pts) { ox += p.X; oy += p.Y; }
            var origin = new P2(ox / pts.Count, oy / pts.Count);

            List<double> dirs = FindDirections(pts, origin, o);
            foreach (double th in dirs) res.DirectionsDeg.Add(th * 180 / Math.PI);

            var fams = new List<Fam>();
            var faces = new List<Face>();
            for (int f = 0; f < dirs.Count; f++)
            {
                // a surface belongs to the direction it is closest to
                double nearest = 90;
                for (int g = 0; g < dirs.Count; g++)
                    if (g != f) nearest = Math.Min(nearest, Math.Abs(Math.IEEERemainder(dirs[f] - dirs[g], Math.PI)) * 180 / Math.PI);
                Fam fam = BuildFamily(pts, origin, dirs[f], WallStep);
                fam.MaxSlope = Math.Tan(Math.Min(7.0, 0.45 * nearest) * Math.PI / 180);
                fams.Add(fam);
                faces.AddRange(TraceFaces(fam, f, o, o.MaxGap, Math.Min(0.20, o.MinLength), 8));
            }
            foreach (Face fc in faces)
                res.Faces.Add(new DetectedFace { A = fams[fc.Fam].World(fc.A0, fc.At(fc.A0)), B = fams[fc.Fam].World(fc.A1, fc.At(fc.A1)), Points = fc.Alongs.Length });

            // columns first: a wall runs on behind a column, and the two sides of a column are not the two faces of a wall
            if (o.Columns)
                for (int g = 1; g < dirs.Count; g++)
                    if (Math.Abs(Math.Abs(Math.IEEERemainder(dirs[g] - dirs[0], Math.PI)) - Math.PI / 2) < 5 * Math.PI / 180)
                    {
                        res.Columns = FindColumns(pts, origin, dirs[0], dirs[g], o, res.ShortFaces);
                        break;
                    }

            List<Piece> pieces = PairFaces(fams, faces, o);
            pieces.RemoveAll(pc =>
            {
                double mid = 0.5 * (pc.X0 + pc.X1);
                P2 c = fams[pc.Fam].World(mid, pc.Center(mid));
                return pc.X1 - pc.X0 <= o.ColumnMaxSize && res.Columns.Any(col => col.Contains(c, 0.03));
            });
            Bridge(fams, pieces, o, res);

            // what is left of each surface: scanned from one side only
            foreach (Face fc in faces)
                foreach (var iv in Subtract(fc.A0, fc.A1, fc.Used))
                {
                    double x0 = fc.FirstAtOrAfter(iv.lo), x1 = fc.LastAtOrBefore(iv.hi);
                    if (double.IsNaN(x0) || double.IsNaN(x1) || x1 - x0 < 0.30) continue;
                    res.SingleFaces.Add(new DetectedFace { A = fams[fc.Fam].World(x0, fc.At(x0)), B = fams[fc.Fam].World(x1, fc.At(x1)), Points = fc.CountIn(x0, x1 + 1e-9) });
                }
            return res;
        }

        // =====================================================================================
        //  1. The wall directions of the building: the angles at which the slice collapses into the sharpest set of lines
        // =====================================================================================
        private static List<double> FindDirections(IReadOnlyList<P2> pts, P2 origin, WallDetectOptions o)
        {
            int stride = Math.Max(1, pts.Count / 40000);
            int n = (pts.Count + stride - 1) / stride;
            var xs = new double[n]; var ys = new double[n];
            for (int i = 0, k = 0; i < pts.Count && k < n; i += stride, k++) { xs[k] = pts[i].X - origin.X; ys[k] = pts[i].Y - origin.Y; }

            const double stepDeg = 0.25;
            int steps = (int)Math.Round(180 / stepDeg);
            var score = new double[steps];
            for (int i = 0; i < steps; i++) score[i] = Sharp(xs, ys, i * stepDeg * Math.PI / 180, 0.02);

            var cands = new List<(double strength, int i)>();
            int d0 = (int)(2 / stepDeg), d1 = (int)(12 / stepDeg);
            var around = new List<double>();
            for (int i = 0; i < steps; i++)
            {
                if (score[i] < score[(i + 1) % steps] || score[i] <= score[(i - 1 + steps) % steps]) continue;
                around.Clear();
                for (int d = d0; d <= d1; d++) { around.Add(score[(i + d) % steps]); around.Add(score[((i - d) % steps + steps) % steps]); }
                double baseline = Stats.Median(around);
                if (score[i] >= 1.5 * baseline) cands.Add((score[i] - baseline, i));
            }
            cands.Sort((a, b) => b.strength.CompareTo(a.strength));

            var found = new List<double>();
            foreach (var c in cands)
            {
                if (found.Count >= o.MaxDirections || c.strength < 0.05 * cands[0].strength) break;
                double th = c.i * stepDeg * Math.PI / 180;
                if (found.Any(f => Math.Abs(Math.IEEERemainder(f - th, Math.PI)) < 10 * Math.PI / 180)) continue;
                // fine search
                double best = th, bestScore = double.MinValue;
                for (double d = -0.4; d <= 0.4 + 1e-9; d += 0.02)
                {
                    double t = th + d * Math.PI / 180, sc = Sharp(xs, ys, t, 0.01);
                    if (sc > bestScore) { bestScore = sc; best = t; }
                }
                found.Add(best);
            }
            return found;
        }

        private static double Sharp(double[] xs, double[] ys, double th, double bin)
        {
            double c = Math.Cos(th), s = Math.Sin(th);
            var v = new double[xs.Length];
            double mn = double.MaxValue, mx = double.MinValue;
            for (int i = 0; i < v.Length; i++)
            {
                v[i] = -s * xs[i] + c * ys[i];
                if (v[i] < mn) mn = v[i]; if (v[i] > mx) mx = v[i];
            }
            var h = new int[(int)((mx - mn) / bin) + 1];
            for (int i = 0; i < v.Length; i++) h[(int)((v[i] - mn) / bin)]++;
            double sum = 0;
            for (int k = 0; k < h.Length; k++) sum += (double)h[k] * h[k];
            return sum;
        }

        private static Fam BuildFamily(IReadOnlyList<P2> pts, P2 origin, double th, double step)
        {
            var fam = new Fam { O = origin, E = new P2(Math.Cos(th), Math.Sin(th)), ABin = step };
            fam.N = fam.E.Perp;
            int n = pts.Count;
            fam.A = new double[n]; fam.C = new double[n];
            double amin = double.MaxValue, amax = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                var v = pts[i] - origin;
                fam.A[i] = v.Dot(fam.E); fam.C[i] = v.Dot(fam.N);
                if (fam.A[i] < amin) amin = fam.A[i]; if (fam.A[i] > amax) amax = fam.A[i];
            }
            fam.AMin = amin;
            fam.NCols = (int)Math.Floor((amax - amin) / step) + 1;
            var cnt = new int[fam.NCols + 1];
            var col = new int[n];
            for (int i = 0; i < n; i++) { col[i] = fam.Col(fam.A[i]); cnt[col[i] + 1]++; }
            for (int k = 0; k < fam.NCols; k++) cnt[k + 1] += cnt[k];
            fam.ColStart = (int[])cnt.Clone();
            fam.Order = new int[n];
            var next = (int[])cnt.Clone();
            for (int i = 0; i < n; i++) fam.Order[next[col[i]]++] = i;
            return fam;
        }

        // =====================================================================================
        //  2. Straight surfaces along one direction: in every 10 cm step along, the across-positions where points pile up;
        //     linked from step to step into lines (a surface a few degrees off the direction drifts sideways, and is followed)
        // =====================================================================================
        private static List<Face> TraceFaces(Fam fam, int famIndex, WallDetectOptions o, double maxGap, double minFace, int minPts)
        {
            double ABin = fam.ABin;
            var result = new List<Face>();
            var peaks = new List<Peak>[fam.NCols];
            var allCnt = new List<double>();
            int[] h = new int[0];
            for (int col = 0; col < fam.NCols; col++)
            {
                int s = fam.ColStart[col], e = fam.ColStart[col + 1];
                if (e - s < 2) continue;
                double cmin = double.MaxValue, cmax = double.MinValue;
                for (int q = s; q < e; q++) { double c = fam.C[fam.Order[q]]; if (c < cmin) cmin = c; if (c > cmax) cmax = c; }
                const int pad = 12;
                int nb = (int)((cmax - cmin) / CBin) + 1 + 2 * pad;
                if (h.Length < nb) h = new int[nb]; else Array.Clear(h, 0, nb);
                var sum = new double[nb];
                for (int q = s; q < e; q++)
                {
                    double c = fam.C[fam.Order[q]];
                    int b = (int)((c - cmin) / CBin) + pad;
                    h[b]++; sum[b] += c;
                }
                int S3(int b) => h[b - 1] + h[b] + h[b + 1];
                for (int b = pad; b < nb - pad; b++)
                {
                    if (h[b] == 0) continue;
                    int s3 = S3(b);
                    if (s3 < 2) continue;
                    // the middle of the pile. Equal neighbours (all the points in ONE bin make three equal sums): the bin that holds
                    // the most points itself wins, then the lower one - never an empty bin
                    bool top = true;
                    for (int k = b - 2; k <= b + 2 && top; k++)
                    {
                        if (k == b) continue;
                        int sk = S3(k);
                        if (sk > s3 || (sk == s3 && (h[k] > h[b] || (h[k] == h[b] && k < b)))) top = false;
                    }
                    if (!top) continue;
                    // a surface seen END-ON (a wall of the other direction) fills this step solidly across: not a line along our direction
                    int occ = 0;
                    for (int k = b - 10; k <= b + 10; k++) if (h[k] > 0) occ++;
                    if (occ >= 12) continue;
                    double cm = (sum[b - 1] + sum[b] + sum[b + 1]) / s3;
                    if (peaks[col] == null) peaks[col] = new List<Peak>();
                    peaks[col].Add(new Peak { Col = col, C = cm, Cnt = s3 });
                    allCnt.Add(s3);
                }
            }
            if (allCnt.Count == 0) return result;
            int minCnt = Math.Max(2, (int)Math.Round(0.08 * Stats.Median(allCnt)));       // far below a normal surface, above stray dots

            // ---- link the piles from step to step ---------------------------------------------------------------------------
            int gapCols = (int)Math.Ceiling(maxGap / ABin);
            var active = new List<Chain>(); var done = new List<Chain>();
            var cand = new List<(double d, int pi, int ci)>();
            for (int col = 0; col < fam.NCols; col++)
            {
                for (int k = active.Count - 1; k >= 0; k--)
                    if (col - active[k].LastCol > gapCols + 1) { done.Add(active[k]); active.RemoveAt(k); }
                var pk = peaks[col];
                if (pk == null) continue;
                cand.Clear();
                for (int pi = 0; pi < pk.Count; pi++)
                {
                    if (pk[pi].Cnt < minCnt) continue;
                    for (int ci = 0; ci < active.Count; ci++)
                    {
                        Chain ch = active[ci];
                        int dcol = col - ch.LastCol;
                        double pred = ch.LastC + ch.Slope * dcol * ABin;
                        double tol = ch.P.Count >= 3 ? 0.015 + 0.003 * dcol : Math.Min(0.015 * dcol, 0.025);
                        double d = Math.Abs(pk[pi].C - pred);
                        if (d <= tol) cand.Add((d, pi, ci));
                    }
                }
                cand.Sort((a, b) => a.d.CompareTo(b.d));
                var peakUsed = new bool[pk.Count]; var chainUsed = new bool[active.Count];
                foreach (var c in cand)
                {
                    if (peakUsed[c.pi] || chainUsed[c.ci]) continue;
                    peakUsed[c.pi] = chainUsed[c.ci] = true;
                    Chain ch = active[c.ci];
                    ch.P.Add(pk[c.pi]); ch.LastCol = col; ch.LastC = pk[c.pi].C;
                    if (ch.P.Count >= 3)
                    {
                        int from = Math.Max(0, ch.P.Count - 10);
                        double ma = 0, mc = 0; int m = ch.P.Count - from;
                        for (int q = from; q < ch.P.Count; q++) { ma += ch.P[q].Col * ABin; mc += ch.P[q].C; }
                        ma /= m; mc /= m;
                        double num = 0, den = 0;
                        for (int q = from; q < ch.P.Count; q++) { double da = ch.P[q].Col * ABin - ma; num += da * (ch.P[q].C - mc); den += da * da; }
                        ch.Slope = den > 1e-9 ? Math.Max(-0.15, Math.Min(0.15, num / den)) : 0;
                    }
                }
                for (int pi = 0; pi < pk.Count; pi++)
                    if (!peakUsed[pi] && pk[pi].Cnt >= minCnt)
                    {
                        var ch = new Chain { LastCol = col, LastC = pk[pi].C };
                        ch.P.Add(pk[pi]);
                        active.Add(ch);
                    }
            }
            done.AddRange(active);

            // ---- every chain: a straight line through its points, then the stretches of that line that really carry points --------------
            var la = new List<double>(); var lc = new List<double>();
            foreach (Chain ch in done)
            {
                if (ch.P.Count < 3) continue;
                la.Clear(); lc.Clear();
                foreach (Peak p in ch.P)
                    for (int q = fam.ColStart[p.Col]; q < fam.ColStart[p.Col + 1]; q++)
                    {
                        int i = fam.Order[q];
                        if (Math.Abs(fam.C[i] - p.C) <= 0.015) { la.Add(fam.A[i]); lc.Add(fam.C[i]); }
                    }
                if (la.Count < minPts) continue;
                if (!FitLine(la, lc, null, out double aRef, out double cRef, out double k)) continue;
                var resid = new List<double>(la.Count);
                for (int i = 0; i < la.Count; i++) resid.Add(lc[i] - (cRef + k * (la[i] - aRef)));
                double sigma = Stats2.MadSigma(resid);
                double keep = Math.Max(2.5 * sigma, 0.008);
                if (!FitLine(la, lc, i => Math.Abs(resid[i]) <= keep, out aRef, out cRef, out k)) continue;
                resid.Clear();
                for (int i = 0; i < la.Count; i++) resid.Add(lc[i] - (cRef + k * (la[i] - aRef)));
                sigma = Stats2.MadSigma(resid);
                if (sigma > Math.Max(0.012, 1.5 * o.InlierTol)) continue;          // not one straight surface
                if (Math.Abs(k) > fam.MaxSlope) continue;                          // belongs to another direction (or to none: a door leaf)

                double tolF = Math.Min(0.02, Math.Max(o.InlierTol, 2.5 * sigma));
                double aFrom = ch.P[0].Col * ABin + fam.AMin - 0.02, aTo = (ch.P[ch.P.Count - 1].Col + 1) * ABin + fam.AMin + 0.02;
                var al = new List<double>();
                for (int col = fam.Col(aFrom); col <= fam.Col(aTo); col++)
                    for (int q = fam.ColStart[col]; q < fam.ColStart[col + 1]; q++)
                    {
                        int i = fam.Order[q];
                        if (fam.A[i] < aFrom || fam.A[i] > aTo) continue;
                        if (Math.Abs(fam.C[i] - (cRef + k * (fam.A[i] - aRef))) <= tolF) al.Add(fam.A[i]);
                    }
                al.Sort();
                int rs = 0;
                for (int q = 1; q <= al.Count; q++)
                {
                    if (q < al.Count && al[q] - al[q - 1] <= maxGap) continue;
                    int cnt = q - rs;
                    if (cnt >= minPts && al[q - 1] - al[rs] >= minFace)
                        result.Add(new Face { Fam = famIndex, A0 = al[rs], A1 = al[q - 1], ARef = aRef, CRef = cRef, K = k, Alongs = al.GetRange(rs, cnt).ToArray() });
                    rs = q;
                }
            }

            // the same surface traced twice: keep the one with more points
            result.Sort((x, y) => y.Alongs.Length.CompareTo(x.Alongs.Length));
            var kept = new List<Face>();
            foreach (Face f in result)
            {
                bool dup = false;
                foreach (Face g in kept)
                {
                    double lo = Math.Max(f.A0, g.A0), hi = Math.Min(f.A1, g.A1);
                    if (hi - lo < 0.6 * (f.A1 - f.A0)) continue;
                    double mid = 0.5 * (lo + hi);
                    if (Math.Abs(f.At(mid) - g.At(mid)) <= 0.012) { dup = true; break; }
                }
                if (!dup) kept.Add(f);
            }
            return kept;
        }

        /// <summary>Least-squares line c = cRef + k (a - aRef) through the (selected) points.</summary>
        private static bool FitLine(List<double> a, List<double> c, Func<int, bool> use, out double aRef, out double cRef, out double k)
        {
            aRef = cRef = k = 0;
            double ma = 0, mc = 0; int n = 0;
            for (int i = 0; i < a.Count; i++) if (use == null || use(i)) { ma += a[i]; mc += c[i]; n++; }
            if (n < 6) return false;
            ma /= n; mc /= n;
            double num = 0, den = 0;
            for (int i = 0; i < a.Count; i++) if (use == null || use(i)) { double da = a[i] - ma; num += da * (c[i] - mc); den += da * da; }
            if (den < 1e-6) return false;
            aRef = ma; cRef = mc; k = num / den;
            return true;
        }

        // =====================================================================================
        //  3. Two parallel surfaces a wall-thickness apart, side by side, with NOTHING between them = a wall.
        //     Long stretches first, thin before thick: each stretch of a surface can be the face of one wall only.
        // =====================================================================================
        private static List<Piece> PairFaces(List<Fam> fams, List<Face> faces, WallDetectOptions o)
        {
            var pieces = new List<Piece>();
            double minOv = Math.Min(0.20, o.MinLength);
            var pairs = new List<(double score, Face lo, Face hi, double x0, double x1)>();
            foreach (var grp in faces.GroupBy(f => f.Fam))
            {
                var list = grp.OrderBy(f => f.CRef).ToList();
                for (int i = 0; i < list.Count; i++)
                    for (int j = i + 1; j < list.Count; j++)
                    {
                        Face fi = list[i], fj = list[j];
                        double x0 = Math.Max(fi.A0, fj.A0), x1 = Math.Min(fi.A1, fj.A1);
                        if (x1 - x0 < minOv) continue;
                        double mid = 0.5 * (x0 + x1), sep = fj.At(mid) - fi.At(mid);
                        Face lo = fi, hi = fj;
                        if (sep < 0) { sep = -sep; lo = fj; hi = fi; }
                        if (sep < o.MinThickness || sep > o.MaxThickness) continue;
                        double taper = Math.Abs((hi.At(x0) - lo.At(x0)) - (hi.At(x1) - lo.At(x1)));
                        if (taper > 0.02 + 0.004 * (x1 - x0)) continue;              // not parallel: not one wall
                        pairs.Add(((x1 - x0) / (1 + sep / 0.5), lo, hi, x0, x1));
                    }
            }
            pairs.Sort((a, b) => b.score.CompareTo(a.score));

            foreach (var pr in pairs)
            {
                Fam fam = fams[pr.lo.Fam];
                var taken = new List<(double lo, double hi)>(pr.lo.Used);
                taken.AddRange(pr.hi.Used);
                foreach (var iv in Subtract(pr.x0, pr.x1, taken))
                {
                    // only where BOTH surfaces really carry points
                    double x0 = Math.Max(pr.lo.FirstAtOrAfter(iv.lo), pr.hi.FirstAtOrAfter(iv.lo));
                    double x1 = Math.Min(pr.lo.LastAtOrBefore(iv.hi), pr.hi.LastAtOrBefore(iv.hi));
                    if (double.IsNaN(x0) || double.IsNaN(x1) || x1 - x0 < minOv) continue;
                    int nLo = pr.lo.CountIn(x0, x1 + 1e-9), nHi = pr.hi.CountIn(x0, x1 + 1e-9);
                    if (Math.Min(nLo, nHi) < 6) continue;
                    double mid = 0.5 * (x0 + x1), sep = pr.hi.At(mid) - pr.lo.At(mid);

                    // a wall has nothing inside it. The two ends are left out: an end face (door jamb) lies exactly there.
                    double m = Math.Max(0.02, 2 * o.InlierTol);
                    if (sep - 2 * m >= 0.015)
                    {
                        double endZone = Math.Min(0.06, 0.2 * (x1 - x0));
                        double i0 = x0 + endZone, i1 = x1 - endZone;
                        int inside = 0;
                        for (int col = fam.Col(i0); col <= fam.Col(i1); col++)
                            for (int q = fam.ColStart[col]; q < fam.ColStart[col + 1]; q++)
                            {
                                int i = fam.Order[q];
                                double a = fam.A[i];
                                if (a < i0 || a > i1) continue;
                                if (fam.C[i] > pr.lo.At(a) + m && fam.C[i] < pr.hi.At(a) - m) inside++;
                            }
                        if (inside > Math.Max(6, 0.15 * Math.Min(nLo, nHi))) continue;
                    }

                    pr.lo.Used.Add((x0, x1)); pr.hi.Used.Add((x0, x1));
                    pieces.Add(new Piece { Fam = pr.lo.Fam, Lo = pr.lo, Hi = pr.hi, X0 = x0, X1 = x1, Thickness = sep, Points = nLo + nHi });
                }
            }
            return pieces;
        }

        /// <summary>[lo, hi] minus the taken stretches.</summary>
        private static List<(double lo, double hi)> Subtract(double lo, double hi, List<(double lo, double hi)> taken)
        {
            var free = new List<(double lo, double hi)>();
            double cur = lo;
            foreach (var t in taken.OrderBy(t => t.lo))
            {
                if (t.hi <= cur) continue;
                if (t.lo >= hi) break;
                if (t.lo > cur) free.Add((cur, Math.Min(t.lo, hi)));
                cur = Math.Max(cur, t.hi);
                if (cur >= hi) break;
            }
            if (cur < hi) free.Add((cur, hi));
            return free;
        }

        // =====================================================================================
        //  Columns. A rectangular column shows as short faces: two parallel SIDES, a FRONT joining them, and either a back
        //  (free-standing) or the wall it stands on carrying on to the left and right of it (a pier). The same shape the other
        //  way round - the wall carrying on from the FRONT corners - is a door, a window bay or a niche, not a column.
        // =====================================================================================
        /// <summary>Does a column stand in the gap between two points on a wall line (so the wall runs on behind it)?</summary>
        public static bool ColumnInGap(IEnumerable<DetectedColumn> columns, P2 g0, P2 g1)
        {
            if (columns == null) return false;
            P2 mid = (g0 + g1) * 0.5;
            double gap = (g1 - g0).Length;
            foreach (DetectedColumn c in columns)
                if (c.Contains(mid, 0.30) && gap <= Math.Max(c.Width, c.Depth) + 0.15) return true;      // the wall's centre line lies half a wall behind a pier
            return false;
        }

        private static List<DetectedColumn> FindColumns(IReadOnlyList<P2> pts, P2 origin, double th0, double th1, WallDetectOptions o, List<DetectedFace> shortFaces)
        {
            Fam f0 = BuildFamily(pts, origin, th0, ColumnStep), f1 = BuildFamily(pts, origin, th1, ColumnStep);
            f0.MaxSlope = f1.MaxSlope = Math.Tan(5 * Math.PI / 180);
            List<Face> h = TraceFaces(f0, 0, o, 0.05, 0.06, 5), v = TraceFaces(f1, 1, o, 0.05, 0.06, 5);
            foreach (Face x in h) if (x.A1 - x.A0 <= o.ColumnMaxSize) shortFaces.Add(new DetectedFace { A = f0.World(x.A0, x.At(x.A0)), B = f0.World(x.A1, x.At(x.A1)), Points = x.Alongs.Length });
            foreach (Face x in v) if (x.A1 - x.A0 <= o.ColumnMaxSize) shortFaces.Add(new DetectedFace { A = f1.World(x.A0, x.At(x.A0)), B = f1.World(x.A1, x.At(x.A1)), Points = x.Alongs.Length });
            var found = new List<DetectedColumn>();
            Assemble(f0, f1, v, o, found);          // sides across direction 0, front along it
            Assemble(f1, f0, h, o, found);          // and the other way round
            // the same column found from both directions: keep the first
            var cols = new List<DetectedColumn>();
            foreach (DetectedColumn c in found.OrderByDescending(x => x.FreeStanding).ThenByDescending(x => x.Width * x.Depth))
                if (!cols.Any(k => k.Contains(c.Center, 0.02) || c.Contains(k.Center, 0.02))) cols.Add(c);
            return cols;
        }

        /// <summary>Share of the stretch a0..a1 of the line "across = c" that carries points (2 cm steps).</summary>
        private static double Cover(Fam fam, double a0, double a1, double c)
        {
            int n = Math.Max(1, (int)Math.Ceiling((a1 - a0) / 0.02));
            var hit = new bool[n];
            for (int col = fam.Col(a0); col <= fam.Col(a1); col++)
                for (int q = fam.ColStart[col]; q < fam.ColStart[col + 1]; q++)
                {
                    int i = fam.Order[q];
                    if (fam.A[i] < a0 || fam.A[i] >= a1 || Math.Abs(fam.C[i] - c) > 0.02) continue;
                    hit[Math.Min(n - 1, (int)((fam.A[i] - a0) / 0.02))] = true;
                }
            return hit.Count(x => x) / (double)n;
        }

        private static void Assemble(Fam fam, Fam other, List<Face> sideFaces, WallDetectOptions o, List<DetectedColumn> found)
        {
            // the short faces of the other direction, in this direction's frame: each stands at one along-position and spans cLo..cHi across
            var sides = new List<(double a, double cLo, double cHi, int n)>();
            foreach (Face s in sideFaces)
            {
                if (s.A1 - s.A0 > o.ColumnMaxSize + 0.05) continue;
                P2 w0 = other.World(s.A0, s.At(s.A0)) - fam.O, w1 = other.World(s.A1, s.At(s.A1)) - fam.O;
                double a0 = w0.Dot(fam.E), a1 = w1.Dot(fam.E), c0 = w0.Dot(fam.N), c1 = w1.Dot(fam.N);
                if (Math.Abs(a0 - a1) > 0.03 + 0.1 * Math.Abs(c1 - c0)) continue;
                sides.Add((0.5 * (a0 + a1), Math.Min(c0, c1), Math.Max(c0, c1), s.Alongs.Length));
            }
            sides.Sort((x, y) => x.a.CompareTo(y.a));
            double ang = Math.Atan2(fam.E.Y, fam.E.X);
            for (int i = 0; i < sides.Count; i++)
                for (int j = i + 1; j < sides.Count; j++)
                {
                    var L = sides[i]; var R = sides[j];
                    double w = R.a - L.a;
                    if (w > o.ColumnMaxSize) break;
                    if (w < o.ColumnMinSize) continue;
                    double d = Math.Min(L.cHi - L.cLo, R.cHi - R.cLo);
                    if (d < 0.08) continue;
                    foreach (int sgn in new[] { 1, -1 })                // the front is at the low ends of the sides, or at the high ends
                    {
                        double eL = sgn > 0 ? L.cLo : L.cHi, eR = sgn > 0 ? R.cLo : R.cHi;
                        if (Math.Abs(eL - eR) > 0.03) continue;
                        double cF = 0.5 * (eL + eR);
                        if (Cover(fam, L.a, R.a, cF) < 0.40) continue;                                   // no front
                        // the wall carries on from the FRONT corners: this is a recess in a wall (door, window bay, niche), not a column
                        if (Math.Max(Cover(fam, L.a - 0.27, L.a - 0.04, cF), Cover(fam, R.a + 0.04, R.a + 0.27, cF)) >= 0.40) continue;
                        // where is the back? The traced sides stop a little short of it (the corner belongs to the wall), so look for
                        // the line itself: the wall carrying on to BOTH sides of the column, or a face closing it
                        double bestBack = -1, depth = d;
                        bool onWall = false, closed = false;
                        for (double dd = Math.Max(0.08, d - 0.03); dd <= d + 0.12 + 1e-9; dd += 0.01)
                        {
                            double c = cF + sgn * dd;
                            double left = Cover(fam, L.a - 0.27, L.a - 0.04, c), right = Cover(fam, R.a + 0.04, R.a + 0.27, c), mid = Cover(fam, L.a, R.a, c);
                            bool wall = Math.Min(left, right) >= 0.50, shut = mid >= 0.40;
                            if (!wall && !shut) continue;
                            double score = left + right + mid;
                            if (score > bestBack) { bestBack = score; depth = dd; onWall = wall; closed = shut; }
                        }
                        double cB = cF + sgn * depth;
                        if (!onWall && !closed) continue;                                               // open at the back: not a clear column
                        bool free = closed && !onWall;
                        if (!free && (depth > 1.5 * w || w > 3 * depth)) continue;                              // deeper than wide: the end of a wall; long and flat: a piece of wall
                        if (free && (depth < o.ColumnMinSize || depth > 3 * w || w > 3 * depth)) continue;
                        // nothing inside a column
                        if (w > 0.14 && depth > 0.14)
                        {
                            int inside = 0;
                            double i0 = L.a + 0.05, i1 = R.a - 0.05, k0 = Math.Min(cF, cB) + 0.05, k1 = Math.Max(cF, cB) - 0.05;
                            for (int col = fam.Col(i0); col <= fam.Col(i1); col++)
                                for (int q = fam.ColStart[col]; q < fam.ColStart[col + 1]; q++)
                                {
                                    int k = fam.Order[q];
                                    if (fam.A[k] > i0 && fam.A[k] < i1 && fam.C[k] > k0 && fam.C[k] < k1) inside++;
                                }
                            if (inside > 0.15 * (L.n + R.n)) continue;
                        }
                        found.Add(new DetectedColumn
                        {
                            CornerA = fam.World(L.a, cF), CornerB = fam.World(R.a, cB), Width = w, Depth = depth, AngleRad = ang, FreeStanding = free,
                        });
                    }
                }
        }

        // =====================================================================================
        //  4. Pieces of the same wall on both sides of a door or window opening become ONE wall
        // =====================================================================================
        private static void Bridge(List<Fam> fams, List<Piece> pieces, WallDetectOptions o, WallDetectResult res)
        {
            foreach (var grp in pieces.GroupBy(p => p.Fam))
            {
                Fam fam = fams[grp.Key];
                var list = grp.OrderBy(p => p.X0).ToList();
                var nextOf = new int[list.Count]; var hasPrev = new bool[list.Count];
                for (int i = 0; i < list.Count; i++) nextOf[i] = -1;
                for (int i = 0; i < list.Count; i++)
                {
                    Piece u = list[i];
                    int best = -1; double bestGap = double.MaxValue;
                    for (int j = 0; j < list.Count; j++)
                    {
                        if (j == i || hasPrev[j]) continue;
                        Piece v = list[j];
                        double gap = v.X0 - u.X1;
                        if (gap < -0.05 || gap > Math.Max(o.BridgeOpening, o.MaxGap) || gap >= bestGap) continue;
                        if (Math.Abs(u.Thickness - v.Thickness) > 0.02) continue;
                        if (Math.Abs(u.Slope - v.Slope) > 0.01) continue;
                        double gm = 0.5 * (u.X1 + v.X0);
                        if (Math.Abs(u.Center(gm) - v.Center(gm)) > 0.02) continue;      // not on the same line
                        P2 g0 = fam.World(u.X1, u.Center(u.X1)), g1 = fam.World(v.X0, v.Center(v.X0));
                        if (gap > o.MaxGap && !ColumnInGap(res.Columns, g0, g1) && o.OpeningIsInWall != null &&
                            !o.OpeningIsInWall(g0, g1, 0.5 * (u.Thickness + v.Thickness))) continue;
                        best = j; bestGap = gap;
                    }
                    if (best >= 0) { nextOf[i] = best; hasPrev[best] = true; }
                }
                for (int i = 0; i < list.Count; i++)
                {
                    if (hasPrev[i]) continue;
                    Piece first = list[i], last = first;
                    double covered = 0, thick = 0; int n = 0, points = 0;
                    for (int k = i; k >= 0; k = nextOf[k])
                    {
                        last = list[k];
                        double len = last.X1 - last.X0;
                        covered += len; thick += last.Thickness * len; points += last.Points; n++;
                    }
                    double total = last.X1 - first.X0;
                    if (total < o.MinLength) continue;
                    res.Walls.Add(new DetectedWall
                    {
                        Start = fam.World(first.X0, first.Center(first.X0)), End = fam.World(last.X1, last.Center(last.X1)),
                        Thickness = thick / covered, Coverage = Math.Min(1.0, covered / total), Openings = n - 1, Points = points,
                    });
                }
            }
            res.Walls.Sort((a, b) => b.Length.CompareTo(a.Length));
        }
    }
}
