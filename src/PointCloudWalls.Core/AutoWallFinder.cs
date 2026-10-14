using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Core
{
    /// <summary>One wall of the automatic search: what was detected, how the solver measured it, and whether it is good enough to model.</summary>
    public sealed class AutoWall
    {
        public DetectedWall Detected;
        public WallFitResult Fit;
        /// <summary>Scanned from one side only: Detected is the surface itself.</summary>
        public bool OneFace;
        /// <summary>Measured from both faces, then followed along a stretch where only one of them is scanned.</summary>
        public bool Extended;
        /// <summary>Why it is NOT modeled (empty for the accepted ones).</summary>
        public string Reason = "";
    }

    /// <summary>One column of the automatic search: what was detected and how the column solver measured it.</summary>
    public sealed class AutoColumn
    {
        public DetectedColumn Detected;
        public ColumnFitResult Fit;
        /// <summary>Why it is NOT modeled (empty for the accepted ones).</summary>
        public string Reason = "";
    }

    public sealed class AutoWallResult
    {
        /// <summary>Rectangular columns to model (free-standing, or piers on a wall).</summary>
        public List<AutoColumn> Columns = new List<AutoColumn>();
        public List<AutoColumn> SkippedColumns = new List<AutoColumn>();
        /// <summary>Walls to model, longest first.</summary>
        public List<AutoWall> Walls = new List<AutoWall>();
        /// <summary>Found, but not sure enough: left for the click tools.</summary>
        public List<AutoWall> Skipped = new List<AutoWall>();
        /// <summary>Long surfaces with no opposite face whose wall side could not be decided: left for the click tools.</summary>
        public List<DetectedFace> SingleFaces = new List<DetectedFace>();
        public List<double> DirectionsDeg = new List<double>();
    }

    /// <summary>
    /// The whole automatic pipeline: WallDetector proposes walls from the slice, WallFitter measures each one exactly like a clicked wall,
    /// and only the ones the scan clearly supports are kept. Nobody checks an automatic wall while it is made, so a short or weak piece
    /// needs MORE evidence than the same wall would need under a user's clicks.
    /// </summary>
    public static class AutoWallFinder
    {
        public static AutoWallResult Run(IReadOnlyList<P2> slice, IWallPointSource src, WallDetectOptions d = null, WallFitOptions f = null, ColumnFitOptions c = null)
        {
            d = d ?? new WallDetectOptions();
            f = f ?? new WallFitOptions();
            var res = new AutoWallResult();
            WallDetectResult det = WallDetector.Detect(slice, d);
            res.DirectionsDeg = det.DirectionsDeg;

            // ---- columns: measured by the column solver from the two detected corners, exactly like Column Diagonal
            c = (c ?? new ColumnFitOptions()).Clone();
            c.AllowRound = false;
            foreach (DetectedColumn dc in det.Columns)
            {
                var ac = new AutoColumn { Detected = dc, Fit = ColumnFitter.FitDiagonal(src, dc.CornerA, dc.CornerB, c, dc.AngleRad) };
                if (!ac.Fit.Success) ac.Reason = "not measurable: " + ac.Fit.Message;
                else
                {
                    double big = Math.Max(ac.Fit.Width, ac.Fit.Depth), small = Math.Min(ac.Fit.Width, ac.Fit.Depth);
                    double big0 = Math.Max(dc.Width, dc.Depth), small0 = Math.Min(dc.Width, dc.Depth);
                    if (Math.Abs(big - big0) > Math.Max(0.06, 0.25 * big0) || Math.Abs(small - small0) > Math.Max(0.06, 0.25 * small0))
                        ac.Reason = "the measured size does not match the detected one";
                }
                if (ac.Reason.Length == 0 && !dc.FreeStanding && d.FloorShare != null)
                {
                    // A pier stands proud of its wall INTO the room. The same outline seen from the other side is a window bay or a
                    // niche between two piers: its "front" faces away from the room, and there is no floor in front of it.
                    var e = new P2(Math.Cos(dc.AngleRad), Math.Sin(dc.AngleRad));
                    P2 frontEnd = dc.CornerA + e * (dc.CornerB - dc.CornerA).Dot(e);
                    double body = (dc.CornerB - dc.CornerA).Dot((frontEnd - dc.CornerA).Normalized().Perp) >= 0 ? 1 : -1;
                    if (d.FloorShare(dc.CornerA, frontEnd, -body * 0.10, -body * 0.60) < 0.30) ac.Reason = "faces away from the room (a window bay or a niche, not a column)";
                }
                if (ac.Reason.Length > 0) res.SkippedColumns.Add(ac); else res.Columns.Add(ac);
            }

            var fitted = new List<AutoWall>();
            foreach (DetectedWall w in det.Walls)
            {
                var a = new AutoWall { Detected = w, Fit = WallFitter.FitDetected(src, w.Start, w.End, w.Thickness, f) };
                if (!a.Fit.Success) a.Reason = "not measurable: " + a.Fit.Message;
                else if (a.Fit.OneFace) a.Reason = "only one face";
                else if (Math.Abs(a.Fit.Thickness - w.Thickness) > Math.Max(0.02, 0.15 * w.Thickness)) a.Reason = "the measured thickness does not match the detected one";
                else if (a.Fit.Thickness < d.MinThickness || a.Fit.Thickness > d.MaxThickness) a.Reason = "thickness outside the automatic range";
                else if (a.Fit.Length < d.MinLength) a.Reason = "too short";
                else if (a.Fit.Coverage < 0.35) a.Reason = "the second face is mostly missing";
                else if (a.Fit.Grade == "Check" && a.Fit.Length < 1.5) a.Reason = "short and rough";
                if (a.Reason.Length > 0) res.Skipped.Add(a); else fitted.Add(a);
            }

            // short pieces: well covered on both faces, or as thick as a longer wall of this building
            var known = fitted.Where(a => a.Fit.Length >= 1.5).Select(a => a.Fit.Thickness).ToList();
            foreach (AutoWall a in fitted.ToList())
            {
                if (a.Fit.Length >= 1.0 || a.Fit.Coverage >= 0.85) continue;
                if (a.Fit.Coverage >= 0.50 && known.Any(t => Math.Abs(t - a.Fit.Thickness) <= 0.012)) continue;
                a.Reason = "short, and unlike any longer wall";
                fitted.Remove(a); res.Skipped.Add(a);
            }

            // the same wall found twice: keep the longer
            fitted.Sort((x, y) => y.Fit.Length.CompareTo(x.Fit.Length));
            foreach (AutoWall a in fitted)
            {
                if (res.Walls.Any(b => SameWall(a.Fit, b.Fit))) { a.Reason = "same wall found twice"; res.Skipped.Add(a); }
                else res.Walls.Add(a);
            }

            // ---- a wall often carries on with only ONE of its faces scanned (the other hidden behind furniture, or in a room that was
            //      not scanned). Along such a stretch it is still the same wall with the same thickness: follow it, like a user who
            //      clicks the far end of the wall. Not further than the measured part is long - a short piece must not decide a long wall.
            var free = det.SingleFaces.Select(x => new DetectedFace { A = x.A, B = x.B, Points = x.Points }).ToList();
            foreach (AutoWall a in res.Walls)
            {
                P2 s0 = a.Detected.Start, e0 = a.Detected.End;
                P2 u = (e0 - s0).Normalized(), n = u.Perp;
                double len = (e0 - s0).Length, t = a.Fit.Thickness, cap = Math.Max(1.0, len);
                double lo = 0, hi = len;
                foreach (DetectedFace fc in free)
                {
                    double ca = (fc.A - s0).Dot(n), cb = (fc.B - s0).Dot(n);
                    if (ca * cb <= 0 || Math.Abs(Math.Abs(ca) - t / 2) > 0.02 || Math.Abs(Math.Abs(cb) - t / 2) > 0.02) continue;    // not one of the two face lines of this wall
                    double sa = (fc.A - s0).Dot(u), sb = (fc.B - s0).Dot(u);
                    bool flip = sa > sb;
                    double f0 = Math.Min(sa, sb), f1 = Math.Max(sa, sb);
                    if (Math.Abs(f0 - len) <= 0.15 && f1 > hi)                           // carries on past the end
                    {
                        double to = Math.Min(f1, len + cap);
                        hi = Math.Max(hi, to);
                        P2 cut = fc.A + (fc.B - fc.A).Normalized() * (flip ? f1 - to : to - f0);
                        if (flip) fc.B = cut; else fc.A = cut;                           // what is left of the surface beyond the wall
                    }
                    else if (Math.Abs(f1) <= 0.15 && f0 < lo)                            // carries on before the start
                    {
                        double to = Math.Max(f0, -cap);
                        lo = Math.Min(lo, to);
                        P2 cut = fc.A + (fc.B - fc.A).Normalized() * (flip ? f1 - to : to - f0);
                        if (flip) fc.A = cut; else fc.B = cut;
                    }
                }
                if (lo > -0.10 && hi < len + 0.10) continue;
                WallFitResult ext = WallFitter.FitDetected(src, s0 + u * lo, s0 + u * hi, t, f);
                if (ext.Success && !ext.OneFace && Math.Abs(ext.Thickness - t) <= 0.01 && ext.Length > a.Fit.Length + 0.10) { a.Fit = ext; a.Extended = true; }
            }
            free.RemoveAll(x => x.Length < 0.30);
            foreach (AutoWall a in res.Walls) FollowToEndFace(src, a.Fit);

            // ---- walls scanned from one side only. Which side is the wall body on, and how thick is it?
            //      1. the surface is the continuation of a face of a measured wall: same side, same thickness
            //      2. the 3D cloud: the room side has floor right up to the surface, the wall side has none. Thickness: the assumed one.
            //      Anything else is left for the click tools - a wall on the wrong side is worse than no wall.
            double minOne = Math.Max(d.OneFaceMinLength, d.MinLength);
            var measured = res.Walls.ToList();
            var cands = new List<OneFaceCand>();
            foreach (DetectedFace fc in free.OrderByDescending(x => x.Length))
            {
                if (!d.OneFaceWalls) { if (fc.Length >= minOne) res.SingleFaces.Add(fc); continue; }
                P2 fmid = (fc.A + fc.B) * 0.5;
                if (det.Columns.Any(col => col.Contains(fmid, 0.03))) continue;                     // a face of a column, not of a wall
                P2 fu = (fc.B - fc.A).Normalized();
                int side = 0; double thick = d.OneFaceThickness; string source = "";
                foreach (AutoWall w in measured)
                {
                    P2 u = w.Fit.Direction, n = u.Perp;
                    double ca = (fc.A - w.Fit.Start).Dot(n), cb = (fc.B - w.Fit.Start).Dot(n), t = w.Fit.Thickness;
                    if (ca * cb <= 0 || Math.Abs(Math.Abs(ca) - t / 2) > 0.02 || Math.Abs(Math.Abs(cb) - t / 2) > 0.02) continue;
                    double sa = (fc.A - w.Fit.Start).Dot(u), sb = (fc.B - w.Fit.Start).Dot(u);
                    double gap = Math.Max(Math.Min(sa, sb) - w.Fit.Length, -Math.Max(sa, sb));      // > 0: apart along the line
                    if (gap > 0.40) continue;
                    side = (ca > 0 ? -1 : 1) * (n.Dot(fu.Perp) >= 0 ? 1 : -1);                      // towards the centre line of that wall
                    thick = t; source = "same wall, measured nearby";
                    break;
                }
                int floorSide = d.BodySide != null ? d.BodySide(fc.A, fc.B) : 0;
                bool disagree = side != 0 && floorSide != 0 && floorSide != side;
                if (side == 0) { side = floorSide; source = "floor scanned on the other side"; }
                if (disagree) side = 0;
                if (side == 0 && !disagree && d.FloorShare != null && fc.Length >= 0.5)
                {
                    // nothing right at the surface (a radiator, a bench, a curtain hides the floor there): look further out. The room
                    // side has floor or ceiling scanned, the other side - the wall, then outside or a room that was not scanned - none.
                    double l = d.FloorShare(fc.A, fc.B, 0.10, 0.60), r = d.FloorShare(fc.A, fc.B, -0.60, -0.10);
                    if (Math.Max(l, r) >= 0.30 && Math.Min(l, r) <= 0.05) { side = l < r ? 1 : -1; source = "room scanned on the other side only"; }
                    d.Log?.Invoke("wide " + fc.A + " -> " + fc.B + " room share left=" + l.ToString("0.00") + " right=" + r.ToString("0.00") + " => " + side);
                }
                if (side == 0) { if (fc.Length >= minOne) res.SingleFaces.Add(fc); continue; }
                if (d.FloorShare != null && fc.Length >= 0.5 && d.FloorShare(fc.A, fc.B, -side * 0.10, -side * 0.90) < 0.25)
                {
                    // nothing of the building beside it on its room side: a mirror image in window glass, or something outside
                    d.Log?.Invoke("no room beside " + fc.A + " -> " + fc.B);
                    continue;
                }
                cands.Add(new OneFaceCand { A = fc.A, B = fc.B, Side = side, Thick = thick, Source = source, Points = fc.Points });
            }

            // pieces of one surface on both sides of a door or window opening are ONE wall, exactly as for the measured walls
            var nextOf = new int[cands.Count]; var hasPrev = new bool[cands.Count];
            for (int i = 0; i < cands.Count; i++) nextOf[i] = -1;
            for (int i = 0; i < cands.Count; i++)
            {
                OneFaceCand p = cands[i];
                P2 u = (p.B - p.A).Normalized(), n = u.Perp;
                double len = (p.B - p.A).Length;
                int best = -1; double bestGap = double.MaxValue;
                for (int j = 0; j < cands.Count; j++)
                {
                    OneFaceCand q = cands[j];
                    if (j == i || hasPrev[j] || q.Side != p.Side || Math.Abs(q.Thick - p.Thick) > 0.02) continue;
                    if ((q.B - q.A).Normalized().Dot(u) < 0.9999) continue;                         // same direction (within ~0.8 degrees)
                    if (Math.Abs((q.A - p.A).Dot(n)) > 0.025 || Math.Abs((q.B - p.A).Dot(n)) > 0.025) continue;   // not the same surface line
                    double gap = (q.A - p.A).Dot(u) - len;
                    if (gap < -0.05 || gap > Math.Max(d.BridgeOpening, d.MaxGap) || gap >= bestGap) continue;
                    P2 into = n * (p.Side * p.Thick / 2);
                    if (gap > d.MaxGap && !WallDetector.ColumnInGap(det.Columns, p.B, q.A) && d.OpeningIsInWall != null && !d.OpeningIsInWall(p.B + into, q.A + into, p.Thick)) continue;
                    best = j; bestGap = gap;
                }
                if (best >= 0) { nextOf[i] = best; hasPrev[best] = true; }
            }
            for (int i = 0; i < cands.Count; i++)
            {
                if (hasPrev[i]) continue;
                OneFaceCand first = cands[i], last = first;
                int points = 0, pieces = 0;
                for (int k = i; k >= 0; k = nextOf[k]) { last = cands[k]; points += last.Points; pieces++; }
                P2 fu = (last.B - first.A).Normalized();
                var a = new AutoWall
                {
                    OneFace = true,
                    Detected = new DetectedWall { Start = first.A, End = last.B, Thickness = first.Thick, Coverage = 1, Openings = pieces - 1, Points = points },
                };
                if (a.Detected.Length < minOne) continue;
                a.Fit = WallFitter.FitDetectedOneFace(src, first.A, last.B, fu.Perp * first.Side, first.Thick, f);
                if (!a.Fit.Success) a.Reason = "not measurable: " + a.Fit.Message;
                else if (a.Fit.Length < minOne) a.Reason = "too short";
                else if (a.Fit.Grade == "Check") a.Reason = "one rough surface only";       // curtains, shelving, glazing bars: not a clean wall face
                else if (InsideOthers(a.Fit, res.Walls) > 0.30) a.Reason = "lies inside a wall that is already found";
                if (a.Reason.Length > 0) { res.Skipped.Add(a); continue; }
                a.Fit.SideSource = first.Source;
                if (first.Source == "same wall, measured nearby") { a.Fit.ThicknessAssumed = false; a.Fit.ThicknessSource = first.Source; }
                res.Walls.Add(a);
            }
            CloseDoorGaps(res.Walls, d);
            res.Walls.Sort((x, y) => (x.OneFace != y.OneFace) ? (x.OneFace ? 1 : -1) : y.Fit.Length.CompareTo(x.Fit.Length));   // measured walls first
            return res;
        }

        /// <summary>
        /// A found wall against the walls that are ALREADY in the model (centre line a -> b, thickness t; meters): the stretches of it
        /// that are not modeled yet, as start/end points. Nothing is modeled twice, and nothing that is still missing is dropped: a wall
        /// the user modeled only part of (clicked short of its end, or one room's piece of a longer wall) gets the rest - starting
        /// exactly where the modeled piece stops. Empty = all of it is modeled already.
        /// </summary>
        public static List<(P2 start, P2 end)> UnmodeledParts(WallFitResult w, IEnumerable<(P2 a, P2 b, double t)> existing, double minLength)
        {
            P2 s0 = w.Start, u = (w.End - w.Start).Normalized(), n = u.Perp;
            double len = w.Length;
            var cov = new List<(double lo, double hi)>();
            foreach (var e in existing)
            {
                P2 de = (e.b - e.a).Normalized();
                if (Math.Abs(u.X * de.Y - u.Y * de.X) > 0.05) continue;                                    // more than ~3 degrees apart
                if (Math.Max(Math.Abs((e.a - s0).Dot(n)), Math.Abs((e.b - s0).Dot(n))) > 0.5 * Math.Max(w.Thickness, e.t)) continue;   // side by side
                double a0 = (e.a - s0).Dot(u), a1 = (e.b - s0).Dot(u);
                double lo = Math.Max(0, Math.Min(a0, a1)), hi = Math.Min(len, Math.Max(a0, a1));
                if (hi - lo > 0.05) cov.Add((lo, hi));
            }
            var parts = new List<(P2 start, P2 end)>();
            if (cov.Count == 0) { parts.Add((w.Start, w.End)); return parts; }
            cov.Sort((x, y) => x.lo.CompareTo(y.lo));
            double keep = Math.Min(minLength, 0.25);          // the rest of a wall that is partly modeled may be shorter than a wall on its own
            double cur = 0;
            foreach (var c in cov)
            {
                if (c.lo - cur >= keep) parts.Add((s0 + u * cur, s0 + u * c.lo));
                cur = Math.Max(cur, c.hi);
            }
            if (len - cur >= keep) parts.Add((s0 + u * cur, s0 + u * len));
            return parts;
        }

        /// <summary>
        /// Doors. A wall is modeled THROUGH its door openings - the door is hosted in it afterwards and cuts its own opening - so two
        /// walls that stand on one line with a door-sized gap between them must not stay two walls with a hole between them.
        /// The pieces on the two sides of a door are often not "the same wall" for the detector: measured a little differently, one
        /// of them scanned from one side only, or really of a different thickness (two partition types meeting at the door frame).
        ///   - same thickness, same line, same kind: they become ONE wall;
        ///   - otherwise ONE of them is carried across the opening until it touches the other, so the whole opening lies in a
        ///     single wall that can host the door: the measured wall rather than one with an assumed thickness, else the longer.
        /// Only when the 3D cloud shows the wall carrying on above the opening (when that check is available): the end of a wall
        /// at a corridor is not a door.
        /// </summary>
        private static void CloseDoorGaps(List<AutoWall> walls, WallDetectOptions d)
        {
            if (d.BridgeOpening <= 0) return;
            double maxGap = Math.Max(d.BridgeOpening, d.MaxGap);
            var done = new HashSet<(WallFitResult, WallFitResult)>();
            while (true)
            {
                int bi = -1, bj = -1; bool bie = false, bje = false; double best = double.MaxValue;
                for (int i = 0; i < walls.Count; i++)
                    for (int j = i + 1; j < walls.Count; j++)
                    {
                        WallFitResult A = walls[i].Fit, B = walls[j].Fit;
                        if (done.Contains((A, B))) continue;
                        foreach (bool ie in new[] { false, true })
                            foreach (bool je in new[] { false, true })
                            {
                                P2 ea = ie ? A.End : A.Start, fa = ie ? A.Start : A.End;        // the end at the gap, the far end
                                P2 eb = je ? B.End : B.Start, fb = je ? B.Start : B.End;
                                P2 u = (ea - fa).Normalized();                                    // out of A, across the gap
                                if (u.Dot((fb - eb).Normalized()) < 0.9997) continue;             // one line (within ~1.4 degrees), B ahead of A
                                double gap = (eb - ea).Dot(u);
                                if (gap < 0.03 || gap > maxGap || gap >= best) continue;
                                if (Math.Abs((eb - ea).Dot(u.Perp)) > 0.5 * Math.Max(A.Thickness, B.Thickness) + 0.02) continue;   // side by side, not in line
                                bi = i; bj = j; bie = ie; bje = je; best = gap;
                            }
                    }
                if (bi < 0) break;

                AutoWall wa = walls[bi], wb = walls[bj];
                {
                    WallFitResult A = wa.Fit, B = wb.Fit;
                    done.Add((A, B));
                    P2 ea = bie ? A.End : A.Start, fa = bie ? A.Start : A.End, eb = bje ? B.End : B.Start, fb = bje ? B.Start : B.End;
                    P2 u = (ea - fa).Normalized();
                    double gap = (eb - ea).Dot(u), off = Math.Abs((eb - ea).Dot(u.Perp));
                    bool hostA = wa.OneFace != wb.OneFace ? !wa.OneFace : A.Length >= B.Length;
                    WallFitResult H = hostA ? A : B;
                    P2 h0 = hostA ? ea : eb, hu = hostA ? u : u * -1;                              // from the end of the host across the gap
                    if (gap > d.MaxGap && d.OpeningIsInWall != null && !d.OpeningIsInWall(h0, h0 + hu * gap, H.Thickness)) continue;

                    if (wa.OneFace == wb.OneFace && Math.Abs(A.Thickness - B.Thickness) <= 0.02 && off <= 0.02)
                    {
                        double la = A.Length, lb = B.Length;
                        P2 oldNormal = A.Normal;
                        A.Thickness = (A.Thickness * la + B.Thickness * lb) / (la + lb);
                        A.Coverage = (A.Coverage * la + B.Coverage * lb) / (la + lb + gap);
                        EndType far = bje ? B.AtStart : B.AtEnd;
                        if (bie) { A.End = fb; A.AtEnd = far; } else { A.Start = fb; A.AtStart = far; }
                        A.Direction = (A.End - A.Start).Normalized();
                        A.Normal = A.Direction.Perp * (A.Direction.Perp.Dot(oldNormal) >= 0 ? 1 : -1);
                        if (B.Grade == "Check" || (B.Grade == "OK" && A.Grade == "Good")) A.Grade = B.Grade;
                        wa.Detected.Openings += wb.Detected.Openings + 1;
                        walls.RemoveAt(bj);
                    }
                    else
                    {
                        if (hostA) { if (bie) A.End = ea + u * gap; else A.Start = ea + u * gap; }
                        else { if (bje) B.End = eb - u * gap; else B.Start = eb - u * gap; }
                        (hostA ? wa : wb).Detected.Openings++;
                    }
                }
            }

            // A door right beside a crossing wall: the end of a wall faces the SIDE of another wall across the opening. The wall is
            // carried on to that wall (the joiner then makes the T), again only when it is seen to carry on above the opening.
            foreach (AutoWall wa in walls)
                foreach (bool ie in new[] { false, true })
                {
                    WallFitResult A = wa.Fit;
                    P2 ea = ie ? A.End : A.Start, fa = ie ? A.Start : A.End, u = (ea - fa).Normalized();
                    double bestGap = double.MaxValue;
                    bool abuts = false;
                    foreach (AutoWall wb in walls)
                    {
                        if (ReferenceEquals(wb, wa)) continue;
                        WallFitResult B = wb.Fit;
                        P2 db = (B.End - B.Start).Normalized();
                        double cross = u.X * db.Y - u.Y * db.X;
                        if (Math.Abs(cross) < 0.94)
                        {
                            // a wall in line with this one that already starts right here: nothing to close at this end
                            if (Math.Abs(cross) > 0.03) continue;
                            foreach (P2 eb in new[] { B.Start, B.End })
                                if (Math.Abs((eb - ea).Dot(u)) < 0.06 && Math.Abs((eb - ea).Dot(u.Perp)) <= 0.5 * Math.Max(A.Thickness, B.Thickness) + 0.02) abuts = true;
                            continue;
                        }
                        P2 qp = B.Start - ea;
                        double along = (qp.X * db.Y - qp.Y * db.X) / cross;                 // from this end, along this wall, to its centre line
                        double onB = (qp.X * u.Y - qp.Y * u.X) / cross;                     // where on it
                        if (onB < -0.5 * A.Thickness || onB > B.Length + 0.5 * A.Thickness) continue;
                        double gap = along - 0.5 * B.Thickness;                             // to its near face
                        if (gap > 0.05 && gap <= maxGap && gap < bestGap) bestGap = gap;
                    }
                    if (abuts || bestGap == double.MaxValue) continue;
                    if (bestGap > d.MaxGap && d.OpeningIsInWall != null && !d.OpeningIsInWall(ea, ea + u * bestGap, A.Thickness)) continue;
                    if (ie) A.End = ea + u * bestGap; else A.Start = ea + u * bestGap;
                    wa.Detected.Openings++;
                }
        }

        /// <summary>
        /// A wall often ends in a thicker piece - a pier, a frame block between two doors - where its own two faces stop while the wall
        /// plainly goes on to an end face a little further. An end that was only decided by "the faces stop" is moved to that end
        /// face when (1) it spans the width of the wall, (2) there is nothing inside the wall on the way, and (3) there is material
        /// on BOTH sides all the way (so it is not a doorway with another wall behind it).
        /// </summary>
        private static void FollowToEndFace(IWallPointSource src, WallFitResult r)
        {
            foreach (bool atEnd in new[] { false, true })
            {
                EndType kind = atEnd ? r.AtEnd : r.AtStart;
                if (kind == EndType.Cap) continue;                                      // already at its end face
                P2 e = atEnd ? r.End : r.Start, u = atEnd ? r.Direction : r.Direction * -1, n = u.Perp;
                double t = r.Thickness;
                const double reach = 1.0;
                var pts = src.Query(e + u * (reach / 2), u, reach / 2 + 0.03, t / 2 + 0.40);
                int nA = (int)(reach / 0.01) + 4, nC = Math.Max(2, (int)Math.Ceiling((t - 0.04) / 0.02));
                var across = new bool[nA, nC];                  // 1 cm along x 2 cm across, inside the width of the wall
                var inside = new int[nA];
                var sideL = new bool[nA]; var sideR = new bool[nA];
                foreach (P2 q in pts)
                {
                    P2 v = q - e;
                    double a = v.Dot(u), c = v.Dot(n);
                    if (a < 0 || a > reach) continue;
                    int ia = (int)(a / 0.01);
                    if (Math.Abs(c) <= t / 2 - 0.02) across[ia, Math.Min(nC - 1, (int)((c + t / 2 - 0.02) / 0.02))] = true;
                    if (Math.Abs(c) < t / 2 - 0.03) inside[ia]++;
                    if (c >= t / 2 - 0.03 && c <= t / 2 + 0.40) sideL[ia] = true;
                    if (-c >= t / 2 - 0.03 && -c <= t / 2 + 0.40) sideR[ia] = true;
                }
                for (int ia = 10; ia < nA - 2; ia++)
                {
                    // an end face: a 3 cm slab that covers the width of the wall
                    int hit = 0;
                    for (int k = 0; k < nC; k++) if (across[ia - 1, k] || across[ia, k] || across[ia + 1, k]) hit++;
                    if (hit < 0.7 * nC) continue;
                    int stray = 0, left = 0, right = 0, steps = 0;
                    for (int j = 4; j < ia - 4; j++) stray += inside[j];
                    for (int j = 0; j + 5 <= ia; j += 5)
                    {
                        steps++;
                        bool l = false, rr = false;
                        for (int k = j; k < j + 5; k++) { l |= sideL[k]; rr |= sideR[k]; }
                        if (l) left++; if (rr) right++;
                    }
                    if (stray <= 4 && steps > 0 && left >= 0.6 * steps && right >= 0.6 * steps)
                    {
                        if (atEnd) { r.End = e + u * (ia * 0.01); r.AtEnd = EndType.Cap; } else { r.Start = e + u * (ia * 0.01); r.AtStart = EndType.Cap; }
                    }
                    break;                                       // the first face across the wall decides, whatever it is
                }
            }
        }

        /// <summary>Share of the length of a wall that lies inside other walls (same line, side by side closer than their bodies).</summary>
        private static double InsideOthers(WallFitResult a, List<AutoWall> others)
        {
            P2 da = a.Direction, n = da.Perp;
            double la = a.Length, sum = 0;
            if (la < 1e-6) return 0;
            foreach (AutoWall o in others)
            {
                WallFitResult b = o.Fit;
                if (Math.Abs(da.X * b.Direction.Y - da.Y * b.Direction.X) > 0.05) continue;
                double off0 = (b.Start - a.Start).Dot(n), off1 = (b.End - a.Start).Dot(n);
                if (Math.Max(Math.Abs(off0), Math.Abs(off1)) > 0.5 * (a.Thickness + b.Thickness) - 0.02) continue;
                double s0 = (b.Start - a.Start).Dot(da), s1 = (b.End - a.Start).Dot(da);
                sum += Math.Max(0, Math.Min(la, Math.Max(s0, s1)) - Math.Max(0, Math.Min(s0, s1)));
            }
            return sum / la;
        }

        private sealed class OneFaceCand
        {
            public P2 A, B;
            public int Side, Points;
            public double Thick;
            public string Source;
        }

        /// <summary>Do two fitted walls lie on top of each other (same line, overlapping along most of the shorter one)?</summary>
        public static bool SameWall(WallFitResult a, WallFitResult b) =>
            Overlaps(a.Start, a.End, a.Thickness, b.Start, b.End, b.Thickness);

        /// <summary>The same test for any two wall centre lines.</summary>
        public static bool Overlaps(P2 a0, P2 a1, double ta, P2 b0, P2 b1, double tb)
        {
            P2 da = (a1 - a0).Normalized(), db = (b1 - b0).Normalized();
            double la = (a1 - a0).Length, lb = (b1 - b0).Length;
            if (la < 1e-6 || lb < 1e-6) return false;
            if (Math.Abs(da.X * db.Y - da.Y * db.X) > 0.05) return false;                    // more than ~3 degrees apart
            P2 n = da.Perp;
            double off0 = (b0 - a0).Dot(n), off1 = (b1 - a0).Dot(n);
            if (Math.Max(Math.Abs(off0), Math.Abs(off1)) > 0.5 * Math.Max(ta, tb)) return false;   // side by side, not the same wall
            double s0 = (b0 - a0).Dot(da), s1 = (b1 - a0).Dot(da);
            double lo = Math.Max(0, Math.Min(s0, s1)), hi = Math.Min(la, Math.Max(s0, s1));
            return hi - lo > 0.5 * Math.Min(la, lb);
        }
    }
}
