using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.Revit.DB;

namespace PointCloudWalls.Revit
{
    /// <summary>
    /// After a wall is created: trims / extends wall ends to the CENTERLINE of the walls they meet
    /// (T-junctions and corners) and applies Revit's own joins so the junctions clean up.
    ///
    /// SAFETY: it only ever MOVES walls created by this tool (Comments start with "PCW", or created this session).
    /// Walls from your model are never moved; a new wall may still snap to them and be joined with them.
    /// </summary>
    internal static class WallJoiner
    {
        public const string Tag = "PCW";
        private const double MinSin = 0.34;            // ignore walls closer to parallel than ~20 degrees

        private struct V
        {
            public double X, Y;
            public V(double x, double y) { X = x; Y = y; }
            public static V operator +(V a, V b) => new V(a.X + b.X, a.Y + b.Y);
            public static V operator -(V a, V b) => new V(a.X - b.X, a.Y - b.Y);
            public static V operator *(V a, double k) => new V(a.X * k, a.Y * k);
            public double Len => Math.Sqrt(X * X + Y * Y);
            public V Unit() { double l = Len; return l < 1e-12 ? new V(0, 0) : new V(X / l, Y / l); }
            public static double Cross(V a, V b) => a.X * b.Y - a.Y * b.X;
        }

        private sealed class Seg
        {
            public Wall W;
            public V A, B;
            public double Z, Half;
            public bool Ours, Moved;
            public double Length => (B - A).Len;
        }

        public static bool IsOurs(Wall w, HashSet<string> session)
        {
            if (session.Contains(w.UniqueId)) return true;
            Parameter p = w.get_Parameter(BuiltInParameter.ALL_MODEL_INSTANCE_COMMENTS);
            string c = p != null ? p.AsString() : null;
            return c != null && c.StartsWith(Tag, StringComparison.Ordinal);
        }

        private static Seg MakeSeg(Wall w, HashSet<string> session)
        {
            var lc = w.Location as LocationCurve;
            var ln = lc != null ? lc.Curve as Line : null;
            if (ln == null) return null;
            XYZ p0 = ln.GetEndPoint(0), p1 = ln.GetEndPoint(1);
            return new Seg { W = w, A = new V(p0.X, p0.Y), B = new V(p1.X, p1.Y), Z = p0.Z, Half = w.Width / 2, Ours = IsOurs(w, session) };
        }

        /// <summary>Returns a short report; finalLengthFt is the new wall's length after trimming.</summary>
        public static string JoinAndTrim(Document doc, ViewPlan view, Level lvl, Wall wall, Settings s, HashSet<string> session, out double finalLengthFt)
        {
            double reach = s.JoinReachM / Ctx.MPerFt;
            Seg me = MakeSeg(wall, session);
            finalLengthFt = 0;
            if (me == null) return "join skipped (curved wall)";
            me.Ours = true;

            var others = new List<Seg>();
            foreach (Wall w in new FilteredElementCollector(doc).OfClass(typeof(Wall)).Cast<Wall>())
            {
                if (w.Id.Equals(wall.Id) || !w.LevelId.Equals(lvl.Id)) continue;
                Seg sg = MakeSeg(w, session);
                if (sg != null) others.Add(sg);
            }

            int nT = 0, nCorner = 0, nCross = 0;
            var related = new List<Seg>();

            // ---- 1. the NEW wall's two ends against every other wall ------------------------------------
            for (int end = 0; end < 2; end++)
            {
                V P = end == 0 ? me.A : me.B;
                V Q = end == 0 ? me.B : me.A;
                V dN = (P - Q).Unit();

                Seg best = null; double bestAbsU = double.MaxValue, bu = 0, bt = 0, bsin = 0;
                foreach (Seg y in others)
                {
                    if (!Reach(P, dN, y, me.Half, reach, out double u, out double t, out double sin)) continue;
                    if (Math.Abs(u) < bestAbsU) { best = y; bestAbsU = Math.Abs(u); bu = u; bt = t; bsin = sin; }
                }
                if (best == null) continue;

                V X = P + dN * bu;
                if ((X - Q).Len < 0.05 / Ctx.MPerFt) continue;            // never collapse the wall
                double lenW = best.Length;
                double dEndW = Math.Min(Math.Abs(bt), Math.Abs(bt - lenW));
                bool corner = dEndW <= reach + me.Half / bsin;
                if (end == 0) me.A = X; else me.B = X;
                me.Moved = true;
                if (corner && best.Ours)
                {
                    if (Math.Abs(bt) <= Math.Abs(bt - lenW)) best.A = X; else best.B = X;
                    best.Moved = true;
                }
                if (corner) nCorner++; else nT++;
                related.Add(best);
            }

            // ---- 2. ends of OUR earlier walls that now touch this new wall (this wall is the "through" wall) --
            foreach (Seg y in others.Where(o => o.Ours))
            {
                for (int end = 0; end < 2; end++)
                {
                    V P = end == 0 ? y.A : y.B;
                    V Q = end == 0 ? y.B : y.A;
                    V dY = (P - Q).Unit();
                    if (!Reach(P, dY, me, y.Half, reach, out double u, out double t, out double sin)) continue;
                    double dEndMe = Math.Min(Math.Abs(t), Math.Abs(t - me.Length));
                    if (dEndMe <= reach + y.Half / sin) continue;             // that is a corner: handled above
                    V X = P + dY * u;
                    if ((X - Q).Len < 0.05 / Ctx.MPerFt) continue;
                    if (end == 0) y.A = X; else y.B = X;
                    y.Moved = true; nT++;
                    if (!related.Contains(y)) related.Add(y);
                }
            }

            // ---- 3. apply the new geometry ---------------------------------------------------------------------
            foreach (Seg sg in new[] { me }.Concat(others).Where(o => o.Moved))
            {
                var lc = (LocationCurve)sg.W.Location;
                lc.Curve = Line.CreateBound(new XYZ(sg.A.X, sg.A.Y, sg.Z), new XYZ(sg.B.X, sg.B.Y, sg.Z));
                try { WallUtils.AllowWallJoinAtEnd(sg.W, 0); WallUtils.AllowWallJoinAtEnd(sg.W, 1); } catch { }
            }
            doc.Regenerate();
            finalLengthFt = me.Length;

            // ---- 4. walls that cross each other in the middle (X) -----------------------------------------------
            foreach (Seg y in others)
            {
                if (related.Contains(y)) continue;
                if (Crosses(me, y)) { nCross++; related.Add(y); }
            }

            // ---- 5. Revit's own join so the overlap / junction cleans up -----------------------------------------
            int joined = 0;
            foreach (Seg y in related)
            {
                try
                {
                    if (!JoinGeometryUtils.AreElementsJoined(doc, me.W, y.W)) { JoinGeometryUtils.JoinGeometry(doc, me.W, y.W); joined++; }
                }
                catch { /* walls that do not touch cannot be joined - fine */ }
            }

            if (nT + nCorner + nCross == 0) return "no walls to join";
            var parts = new List<string>();
            if (nT > 0) parts.Add(nT + " T-junction" + (nT > 1 ? "s" : ""));
            if (nCorner > 0) parts.Add(nCorner + " corner" + (nCorner > 1 ? "s" : ""));
            if (nCross > 0) parts.Add(nCross + " crossing" + (nCross > 1 ? "s" : ""));
            return "trimmed/joined: " + string.Join(", ", parts);
        }

        /// <summary>Does the extension of the line (P, unit direction d) meet wall y close enough to join? Returns the signed
        /// distance u along d to the centerline intersection and the position t of that intersection along y.</summary>
        private static bool Reach(V P, V d, Seg y, double ownHalf, double reach, out double u, out double t, out double sin)
        {
            u = t = sin = 0;
            V dy = (y.B - y.A).Unit();
            double denom = V.Cross(d, dy);
            sin = Math.Abs(denom);
            if (sin < MinSin) return false;
            V qp = y.A - P;
            u = V.Cross(qp, dy) / denom;
            t = V.Cross(qp, d) / denom;
            if (Math.Abs(u) > reach + y.Half / sin) return false;
            double len = y.Length;
            double slack = reach + ownHalf / sin;
            return t >= -slack && t <= len + slack;
        }

        private static bool Crosses(Seg a, Seg b)
        {
            V da = (a.B - a.A).Unit(), db = (b.B - b.A).Unit();
            double denom = V.Cross(da, db);
            if (Math.Abs(denom) < MinSin) return false;
            V qp = b.A - a.A;
            double u = V.Cross(qp, db) / denom, t = V.Cross(qp, da) / denom;
            return u > 0 && u < a.Length && t > 0 && t < b.Length;
        }
    }
}
