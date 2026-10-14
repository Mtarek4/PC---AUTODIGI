using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Cad
{
    /// <summary>
    /// 2D union of simple polygons (wall footprints) - so walls that touch or overlap become ONE clean outline, the way Revit
    /// shows joined walls. Handles: abutting walls (T, L), overlapping walls (X, corners), collinear end-to-end walls, rings of
    /// walls (which leave a hole), and disjoint walls (returned as separate loops).
    ///
    /// Method: cut every boundary segment where the other shape's boundary touches or crosses it, keep the pieces that lie
    /// OUTSIDE the other shape, drop pieces inside, and for pieces that lie ON the other boundary keep one copy when both shapes are on
    /// the same side and none when they are on opposite sides (that is the shared edge between two abutting walls); then chain the kept
    /// pieces into loops. Outer loops come out counter-clockwise, holes clockwise.
    /// </summary>
    public static class PolygonUnion
    {
        private struct Pt
        {
            public double X, Y;
            public Pt(double x, double y) { X = x; Y = y; }
        }

        private struct Sg
        {
            public Pt A, B;
            public Sg(Pt a, Pt b) { A = a; B = b; }
        }

        // ------------------------------------------------------------------------------------------------ public API
        /// <summary>Union of the polygons (each x0,y0,x1,y1,...). Returns loops as x,y arrays. <paramref name="tol"/> = distance below which two points are the same.</summary>
        public static List<double[]> Union(IList<double[]> polygons, double tol)
        {
            var parts = new List<List<List<Pt>>>();
            foreach (var arr in polygons)
            {
                List<Pt> p = Clean(ToPts(arr), tol);
                if (p.Count < 3) continue;
                if (Area(p) < 0) p.Reverse();
                if (Math.Abs(Area(p)) < tol * tol) continue;
                parts.Add(new List<List<Pt>> { p });
            }
            // neighbours first: sort along X, then merge halves recursively. Merging one polygon at a time into an ever larger region is
            // quadratic; a balanced tree keeps every merge small (a 1,800-wall building takes well under a second).
            parts.Sort((u, v) => u.Min(l => l.Min(q => q.X)).CompareTo(v.Min(l => l.Min(q => q.X))));
            List<List<Pt>> region = parts.Count == 0 ? new List<List<Pt>>() : Reduce(parts, 0, parts.Count, tol);

            var result = new List<double[]>();
            foreach (var loop in region)
            {
                List<Pt> sLoop = Simplify(loop, tol);
                if (sLoop.Count < 3 || Math.Abs(Area(sLoop)) < tol * tol) continue;
                var flat = new double[sLoop.Count * 2];
                for (int i = 0; i < sLoop.Count; i++) { flat[2 * i] = sLoop[i].X; flat[2 * i + 1] = sLoop[i].Y; }
                result.Add(flat);
            }
            return result;
        }

        private static List<List<Pt>> Reduce(List<List<List<Pt>>> parts, int lo, int hi, double tol)
        {
            if (hi - lo == 1) return parts[lo];
            int mid = (lo + hi) / 2;
            return Merge(Reduce(parts, lo, mid, tol), Reduce(parts, mid, hi, tol), tol);
        }

        /// <summary>Do the two polygons touch or overlap (boundaries within tol, or one inside the other)?</summary>
        public static bool Touch(double[] a, double[] b, double tol)
        {
            List<Pt> pa = ToPts(a), pb = ToPts(b);
            if (pa.Count < 3 || pb.Count < 3) return false;
            if (!BoxesOverlap(pa, pb, tol)) return false;
            for (int i = 0; i < pa.Count; i++)
                for (int j = 0; j < pb.Count; j++)
                    if (SegSegDist(pa[i], pa[(i + 1) % pa.Count], pb[j], pb[(j + 1) % pb.Count]) <= tol) return true;
            return PointInPoly(pa[0], pb) || PointInPoly(pb[0], pa);
        }

        /// <summary>Groups polygons that touch or overlap (directly or through others). Returns a group number per polygon.</summary>
        public static int[] Cluster(IList<double[]> polys, double tol)
        {
            int n = polys.Count;
            var parent = Enumerable.Range(0, n).ToArray();
            int Find(int i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
            var minX = new double[n]; var maxX = new double[n]; var minY = new double[n]; var maxY = new double[n];
            for (int i = 0; i < n; i++)
            {
                double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
                for (int k = 0; k + 1 < polys[i].Length; k += 2) { x0 = Math.Min(x0, polys[i][k]); x1 = Math.Max(x1, polys[i][k]); y0 = Math.Min(y0, polys[i][k + 1]); y1 = Math.Max(y1, polys[i][k + 1]); }
                minX[i] = x0; maxX[i] = x1; minY[i] = y0; maxY[i] = y1;
            }
            var order = Enumerable.Range(0, n).OrderBy(i => minX[i]).ToArray();      // sweep along X: only nearby polygons are compared
            for (int a = 0; a < n; a++)
            {
                int i = order[a];
                for (int b = a + 1; b < n; b++)
                {
                    int j = order[b];
                    if (minX[j] > maxX[i] + tol) break;
                    if (minY[j] > maxY[i] + tol || maxY[j] < minY[i] - tol) continue;
                    if (Find(i) == Find(j)) continue;
                    if (Touch(polys[i], polys[j], tol)) parent[Find(i)] = Find(j);
                }
            }
            var ids = new Dictionary<int, int>(); var res = new int[n];
            for (int i = 0; i < n; i++) { int r = Find(i); if (!ids.ContainsKey(r)) ids[r] = ids.Count; res[i] = ids[r]; }
            return res;
        }

        /// <summary>Signed area of a loop (counter-clockwise positive).</summary>
        public static double SignedArea(double[] loop) => Area(ToPts(loop));

        // ------------------------------------------------------------------------------------------------ one merge step
        private static List<List<Pt>> Merge(List<List<Pt>> region, List<List<Pt>> other, double tol)
        {
            double[] bb1 = Box(region), bb2 = Box(other);
            if (bb1[0] > bb2[2] + tol || bb2[0] > bb1[2] + tol || bb1[1] > bb2[3] + tol || bb2[1] > bb1[3] + tol)
            {
                var both = new List<List<Pt>>(region); both.AddRange(other);          // far apart: nothing to cut
                return both;
            }
            var regSegs = SegsOf(region);
            var pSegs = SegsOf(other);

            var regSplit = Split(regSegs, pSegs, bb2, tol);
            var pSplit = Split(pSegs, regSegs, bb1, tol);

            var kept = new List<Sg>();
            foreach (var s in regSplit)
            {
                if (!SegNearBox(s, bb2, tol)) { kept.Add(s); continue; }                // cannot touch the other shape
                Rel r = Classify(s, pSegs, other, tol);
                if (r == Rel.Outside || r == Rel.OnSame) kept.Add(s);
            }
            foreach (var s in pSplit)
            {
                if (!SegNearBox(s, bb1, tol)) { kept.Add(s); continue; }
                Rel r = Classify(s, regSegs, region, tol);
                if (r == Rel.Outside) kept.Add(s);          // OnSame is already kept from the region side; OnOpposite is the shared edge
            }
            return Chain(kept, tol);
        }

        private static double[] Box(List<List<Pt>> loops)
        {
            double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
            foreach (var l in loops) foreach (var q in l) { x0 = Math.Min(x0, q.X); y0 = Math.Min(y0, q.Y); x1 = Math.Max(x1, q.X); y1 = Math.Max(y1, q.Y); }
            return new[] { x0, y0, x1, y1 };
        }

        private static bool SegNearBox(Sg s, double[] b, double tol) =>
            Math.Max(s.A.X, s.B.X) >= b[0] - tol && Math.Min(s.A.X, s.B.X) <= b[2] + tol &&
            Math.Max(s.A.Y, s.B.Y) >= b[1] - tol && Math.Min(s.A.Y, s.B.Y) <= b[3] + tol;

        private enum Rel { Outside, Inside, OnSame, OnOpposite }

        private static Rel Classify(Sg s, List<Sg> other, List<List<Pt>> otherLoops, double tol)
        {
            var mid = new Pt((s.A.X + s.B.X) / 2, (s.A.Y + s.B.Y) / 2);
            double dx = s.B.X - s.A.X, dy = s.B.Y - s.A.Y;
            double len = Math.Sqrt(dx * dx + dy * dy);
            // on the boundary of the other shape? (prefer the segment that is parallel to this one)
            double bestAlign = -1; int bestSign = 0; bool on = false;
            foreach (var t in other)
            {
                if (PointSegDist(mid, t.A, t.B) > tol) continue;
                double tx = t.B.X - t.A.X, ty = t.B.Y - t.A.Y, tl = Math.Sqrt(tx * tx + ty * ty);
                if (tl < 1e-12 || len < 1e-12) continue;
                double dot = (dx * tx + dy * ty) / (len * tl);
                if (Math.Abs(dot) > bestAlign) { bestAlign = Math.Abs(dot); bestSign = dot >= 0 ? 1 : -1; on = true; }
            }
            if (on && bestAlign > 0.999) return bestSign > 0 ? Rel.OnSame : Rel.OnOpposite;
            if (on) return InsideEvenOdd(mid, otherLoops) ? Rel.Inside : Rel.Outside;         // only touches the other boundary at a point
            return InsideEvenOdd(mid, otherLoops) ? Rel.Inside : Rel.Outside;
        }

        // ------------------------------------------------------------------------------------------------ cutting segments
        private static List<Sg> Split(List<Sg> subject, List<Sg> other, double[] otherBox, double tol)
        {
            var res = new List<Sg>();
            foreach (var s in subject)
            {
                if (!SegNearBox(s, otherBox, tol)) { res.Add(s); continue; }
                double dx = s.B.X - s.A.X, dy = s.B.Y - s.A.Y, l2 = dx * dx + dy * dy, len = Math.Sqrt(l2);
                if (len < tol) continue;
                var ts = new List<double> { 0, 1 };
                double sMinX = Math.Min(s.A.X, s.B.X) - tol, sMaxX = Math.Max(s.A.X, s.B.X) + tol, sMinY = Math.Min(s.A.Y, s.B.Y) - tol, sMaxY = Math.Max(s.A.Y, s.B.Y) + tol;
                foreach (var t in other)
                {
                    if (Math.Max(t.A.X, t.B.X) < sMinX || Math.Min(t.A.X, t.B.X) > sMaxX || Math.Max(t.A.Y, t.B.Y) < sMinY || Math.Min(t.A.Y, t.B.Y) > sMaxY) continue;
                    // the other segment's end points lying ON this segment
                    foreach (var q in new[] { t.A, t.B })
                    {
                        double u = ((q.X - s.A.X) * dx + (q.Y - s.A.Y) * dy) / l2;
                        if (u <= 0 || u >= 1) continue;
                        double px = s.A.X + u * dx - q.X, py = s.A.Y + u * dy - q.Y;
                        if (Math.Sqrt(px * px + py * py) <= tol) ts.Add(u);
                    }
                    // a proper crossing
                    double ex = t.B.X - t.A.X, ey = t.B.Y - t.A.Y;
                    double den = dx * ey - dy * ex;
                    if (Math.Abs(den) < 1e-12 * len * Math.Sqrt(ex * ex + ey * ey)) continue;      // parallel
                    double wx = t.A.X - s.A.X, wy = t.A.Y - s.A.Y;
                    double uu = (wx * ey - wy * ex) / den, vv = (wx * dy - wy * dx) / den;
                    if (uu > 0 && uu < 1 && vv > 0 && vv < 1) ts.Add(uu);
                }
                ts.Sort();
                double minGap = tol / len;
                var clean = new List<double> { 0 };
                foreach (double u in ts) if (u - clean[clean.Count - 1] > minGap && 1 - u > minGap) clean.Add(u);
                clean.Add(1);
                for (int i = 0; i + 1 < clean.Count; i++)
                {
                    var a = i == 0 ? s.A : new Pt(s.A.X + clean[i] * dx, s.A.Y + clean[i] * dy);
                    var b = i + 2 == clean.Count ? s.B : new Pt(s.A.X + clean[i + 1] * dx, s.A.Y + clean[i + 1] * dy);
                    res.Add(new Sg(a, b));
                }
            }
            return res;
        }

        // ------------------------------------------------------------------------------------------------ stitching pieces into loops
        private static List<List<Pt>> Chain(List<Sg> segs, double tol)
        {
            // canonical vertices (points closer than tol are the same vertex)
            var verts = new List<Pt>();
            var grid = new Dictionary<long, List<int>>();
            long Key(long cx, long cy) => cx * 73856093L ^ cy * 19349663L;
            int Vertex(Pt p)
            {
                long cx = (long)Math.Floor(p.X / tol), cy = (long)Math.Floor(p.Y / tol);
                for (long ix = cx - 1; ix <= cx + 1; ix++)
                    for (long iy = cy - 1; iy <= cy + 1; iy++)
                        if (grid.TryGetValue(Key(ix, iy), out var lst))
                            foreach (int v in lst)
                                if (Math.Abs(verts[v].X - p.X) <= tol && Math.Abs(verts[v].Y - p.Y) <= tol) return v;
                verts.Add(p);
                long k = Key(cx, cy);
                if (!grid.TryGetValue(k, out var l2)) grid[k] = l2 = new List<int>();
                l2.Add(verts.Count - 1);
                return verts.Count - 1;
            }

            var edges = new HashSet<(int, int)>();
            foreach (var s in segs)
            {
                int a = Vertex(s.A), b = Vertex(s.B);
                if (a == b) continue;
                if (edges.Contains((b, a))) { edges.Remove((b, a)); continue; }      // the same edge twice in opposite directions: internal
                edges.Add((a, b));
            }

            var outgoing = new Dictionary<int, List<int>>();
            foreach (var e in edges) { if (!outgoing.TryGetValue(e.Item1, out var l)) outgoing[e.Item1] = l = new List<int>(); l.Add(e.Item2); }

            var loops = new List<List<Pt>>();
            while (true)
            {
                int start = -1;
                foreach (var kv in outgoing) if (kv.Value.Count > 0) { start = kv.Key; break; }
                if (start < 0) break;
                var loop = new List<Pt>();
                int cur = start;
                int guard = 0;
                while (guard++ < 1000000)
                {
                    loop.Add(verts[cur]);
                    if (!outgoing.TryGetValue(cur, out var outs) || outs.Count == 0) { loop = null; break; }     // open chain: should not happen
                    int next = outs[0];
                    if (outs.Count > 1)
                    {
                        // a pinch point: prefer the edge that turns most to the LEFT of the way we came in (keeps loops simple)
                        Pt prev = loop.Count > 1 ? loop[loop.Count - 2] : verts[cur];
                        double bestAng = double.MinValue;
                        foreach (int cand in outs)
                        {
                            double a1 = Math.Atan2(verts[cur].Y - prev.Y, verts[cur].X - prev.X), a2 = Math.Atan2(verts[cand].Y - verts[cur].Y, verts[cand].X - verts[cur].X);
                            double turn = a2 - a1; while (turn <= -Math.PI) turn += 2 * Math.PI; while (turn > Math.PI) turn -= 2 * Math.PI;
                            if (turn > bestAng) { bestAng = turn; next = cand; }
                        }
                    }
                    outs.Remove(next);
                    cur = next;
                    if (cur == start) break;
                }
                if (loop != null && loop.Count >= 3) loops.Add(loop);
                if (loop == null) { outgoing[cur].Clear(); }
            }
            return loops;
        }

        // ------------------------------------------------------------------------------------------------ geometry helpers
        private static List<Pt> ToPts(double[] a)
        {
            var l = new List<Pt>(a.Length / 2);
            for (int i = 0; i + 1 < a.Length; i += 2) l.Add(new Pt(a[i], a[i + 1]));
            return l;
        }

        private static List<Sg> SegsOf(List<List<Pt>> loops)
        {
            var r = new List<Sg>();
            foreach (var l in loops) for (int i = 0; i < l.Count; i++) r.Add(new Sg(l[i], l[(i + 1) % l.Count]));
            return r;
        }

        private static List<Pt> Clean(List<Pt> p, double tol)
        {
            var r = new List<Pt>();
            foreach (var q in p)
                if (r.Count == 0 || Math.Abs(r[r.Count - 1].X - q.X) > tol || Math.Abs(r[r.Count - 1].Y - q.Y) > tol) r.Add(q);
            if (r.Count > 1 && Math.Abs(r[0].X - r[r.Count - 1].X) <= tol && Math.Abs(r[0].Y - r[r.Count - 1].Y) <= tol) r.RemoveAt(r.Count - 1);
            return r;
        }

        /// <summary>Remove points that lie on the straight line between their neighbours.</summary>
        private static List<Pt> Simplify(List<Pt> loop, double tol)
        {
            var p = new List<Pt>(loop);
            bool changed = true;
            while (changed && p.Count > 3)
            {
                changed = false;
                for (int i = 0; i < p.Count && p.Count > 3; i++)
                {
                    Pt a = p[(i + p.Count - 1) % p.Count], b = p[i], c = p[(i + 1) % p.Count];
                    if (PointSegDist(b, a, c) <= tol) { p.RemoveAt(i); i--; changed = true; }
                }
            }
            return p;
        }

        private static double Area(List<Pt> p)
        {
            double a = 0;
            for (int i = 0; i < p.Count; i++) { var u = p[i]; var v = p[(i + 1) % p.Count]; a += u.X * v.Y - v.X * u.Y; }
            return a / 2;
        }

        private static bool InsideEvenOdd(Pt q, List<List<Pt>> loops)
        {
            bool inside = false;
            foreach (var l in loops) if (PointInPoly(q, l)) inside = !inside;
            return inside;
        }

        private static bool PointInPoly(Pt q, List<Pt> poly)
        {
            bool inside = false;
            for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
            {
                if ((poly[i].Y > q.Y) != (poly[j].Y > q.Y) &&
                    q.X < (poly[j].X - poly[i].X) * (q.Y - poly[i].Y) / (poly[j].Y - poly[i].Y) + poly[i].X) inside = !inside;
            }
            return inside;
        }

        private static double PointSegDist(Pt p, Pt a, Pt b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
            double u = l2 < 1e-24 ? 0 : Math.Max(0, Math.Min(1, ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2));
            double x = a.X + u * dx - p.X, y = a.Y + u * dy - p.Y;
            return Math.Sqrt(x * x + y * y);
        }

        private static double SegSegDist(Pt a, Pt b, Pt c, Pt d)
        {
            // proper crossing => 0
            double r1 = Cross(a, b, c), r2 = Cross(a, b, d), r3 = Cross(c, d, a), r4 = Cross(c, d, b);
            if (((r1 > 0 && r2 < 0) || (r1 < 0 && r2 > 0)) && ((r3 > 0 && r4 < 0) || (r3 < 0 && r4 > 0))) return 0;
            return Math.Min(Math.Min(PointSegDist(a, c, d), PointSegDist(b, c, d)), Math.Min(PointSegDist(c, a, b), PointSegDist(d, a, b)));
        }

        private static double Cross(Pt a, Pt b, Pt c) => (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);

        private static bool BoxesOverlap(List<Pt> a, List<Pt> b, double tol)
        {
            return a.Min(p => p.X) <= b.Max(p => p.X) + tol && b.Min(p => p.X) <= a.Max(p => p.X) + tol &&
                   a.Min(p => p.Y) <= b.Max(p => p.Y) + tol && b.Min(p => p.Y) <= a.Max(p => p.Y) + tol;
        }
    }
}
