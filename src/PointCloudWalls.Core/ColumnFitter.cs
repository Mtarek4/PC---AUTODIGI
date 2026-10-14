using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Core
{
    public enum ColumnShape { Rectangular, Round }

    public sealed class ColumnFitOptions
    {
        public double SearchRadius = 1.0;      // center-click mode: how far around the click to look (m)
        public double InlierTol = 0.010;        // point-to-face / point-to-circle distance that counts as "on the surface" (m)
        public double SeedBand = 0.08;          // how far a click may be from the true face/corner (m)
        public double MinSize = 0.10, MaxSize = 1.50;      // plausible column width/depth/diameter (m)
        public int MinInliers = 15;
        public int RansacIterations = 400;
        public int RandomSeed = 12345;
        /// <summary>Share of the circle's circumference (in 15-degree bins) that must have points to call it round.</summary>
        public double RoundCoverage = 0.65;
        /// <summary>Shapes Center/Diagonal mode may report. Both true by default; set one false to skip that hypothesis
        /// entirely (faster, and removes any chance of it being mistaken for the other shape).</summary>
        public bool AllowRound = true, AllowRectangular = true;

        public ColumnFitOptions Clone() => (ColumnFitOptions)MemberwiseClone();
    }

    public sealed class ColumnFitResult
    {
        public bool Success;
        public string Message = "";
        public ColumnShape Shape;
        public P2 Center;
        public double Angle;                    // radians: rotation of the Width axis (rectangular only)
        public double Width, Depth;             // rectangular
        public double Diameter;                 // round
        public double Spread;                   // robust surface roughness (m)
        public double Coverage;                 // round: share of the ring seen; rectangular: share of the 4 sides with support
        public int InliersUsed;
        public string Grade = "";
        // Where the rectangle's rotation came from. "measured": ordinary detection was confident, no fallback used -
        // this is the normal case. "wall-hint": the click-based fallback ran and got its angle from an actual modeled
        // Revit wall. "point-cloud": the fallback ran and got its angle directly from nearby scan data (no wall
        // needed to be modeled). "unoriented": the fallback ran but no confident direction was found anywhere nearby,
        // so the rectangle was built at world axes (0 deg) from the clicks alone - still a placed, editable column,
        // never a bare failure, but its rotation should be checked/fixed by hand.
        // Explicit and exhaustive on purpose: an earlier version tried to infer "did the fallback run" from Coverage
        // and Spread happening to both be near zero, which broke the moment Coverage was given a real meaning on
        // that path. Callers should always read this field rather than reconstruct the same inference.
        public string OrientationSource = "measured";

        public override string ToString() =>
            !Success ? "FAIL: " + Message :
            Shape == ColumnShape.Round
                ? $"OK  round  d={Diameter * 1000:F0}mm  spread={Spread * 1000:F1}mm  cov={Coverage * 100:F0}%  [{Grade}]"
                : $"OK  rect  {Width * 1000:F0}x{Depth * 1000:F0}mm  ang={Angle * 180 / Math.PI:F2}deg  spread={Spread * 1000:F1}mm  cov={Coverage * 100:F0}%  [{Grade}]";
    }

    /// <summary>
    /// Fits a column's cross-section (rectangular or round) to a 2D slice of a point cloud, from a rough click near the
    /// center, two clicks at opposite corners, or two clicks on opposite faces. Pure math, no Revit dependency. Meters.
    /// </summary>
    public static class ColumnFitter
    {
        // =====================================================================================
        //  Entry points
        // =====================================================================================
        public static ColumnFitResult FitCenter(IReadOnlyList<P2> points, P2 click, ColumnFitOptions o = null)
            => FitCenter(new ListPointSource(points), click, o);

        public static ColumnFitResult FitCenter(IWallPointSource src, P2 click, ColumnFitOptions o = null)
        {
            o = o ?? new ColumnFitOptions();
            double r = o.SearchRadius;
            List<P2> pts = src.Query(click, new P2(1, 0), r, r);
            return Solve(pts, click, o);
        }

        public static ColumnFitResult FitDiagonal(IReadOnlyList<P2> points, P2 cornerA, P2 cornerB, ColumnFitOptions o = null, double? wallAngleHint = null)
            => FitDiagonal(new ListPointSource(points), cornerA, cornerB, o, wallAngleHint);

        /// <summary><paramref name="wallAngleHint"/> (radians) is used ONLY as a last resort, when the column cannot be
        /// separated from the surrounding data at all - the common real case being a column embedded at a wall corner,
        /// where 2-3 of its faces coincide almost exactly with the walls' own faces and simply have no distinguishing
        /// scan signature of their own. In that situation no amount of point-cloud analysis can find those faces, because
        /// there is nothing there to find - but the two corners the person clicks, combined with the ADJACENT WALL'S OWN
        /// DIRECTION (something no amount of point-cloud analysis could know either, but Revit can supply directly), are
        /// enough to build the column correctly. This fallback only ever engages when ordinary detection fails outright;
        /// a column that can be measured normally always is.
        ///
        /// <paramref name="wideSrc"/> is a SEPARATE point source used only for finding this wall direction and for
        /// snapping the fallback rectangle's edges - deliberately kept apart from <paramref name="src"/>, which the
        /// caller may have already narrowed to whatever thin height slice best shows the COLUMN itself (e.g. an
        /// auto-picked slice that avoids a beam crossing at the original cut height). A wall's direction has nothing
        /// to do with that choice, and confining the search to the column's own slice can miss wall data that is
        /// perfectly visible a few centimetres higher or lower - so callers that do this kind of height narrowing
        /// should pass the ORIGINAL, un-narrowed (or a generously tall) source here instead. When omitted, <paramref
        /// name="src"/> is reused, which is correct whenever the caller never narrows the height in the first place.</summary>
        public static ColumnFitResult FitDiagonal(IWallPointSource src, P2 cornerA, P2 cornerB, ColumnFitOptions o = null, double? wallAngleHint = null, IWallPointSource wideSrc = null)
        {
            o = o ?? new ColumnFitOptions();
            wideSrc = wideSrc ?? src;
            P2 mid = (cornerA + cornerB) * 0.5;
            double diag = (cornerB - cornerA).Length;
            if (diag < o.MinSize * 0.5) return Fail("The two clicks are too close together to be opposite corners of a column.");
            double r = Math.Min(diag, o.MaxSize * 1.5) / 2 + o.SeedBand + 0.15;
            List<P2> pts = src.Query(mid, new P2(1, 0), r, r);
            if (pts.Count < 2 * o.MinInliers) return Fail("Too few points around the clicks.");

            P2 center = RefineCenter(pts, mid, o);

            // The wide, height-independent query (and the wall-direction estimate built from it) is computed ONCE,
            // up front - not just for the click+snap fallback further down, but also to VETO a spuriously confident
            // round classification below. A narrow, column-sized query is documented (see EstimateWallAngle's own
            // callers) as not enough to reliably see a wall's line at all; using that same narrow query for the veto
            // was tried first and left real wall corners undetected too often, letting some through as "round" -
            // exactly the still-happening case this responds to.
            double wr = Math.Max(Math.Max(o.MaxSize * 1.8, r * 1.3), 1.5);
            List<P2> widePts = wideSrc.Query(mid, new P2(1, 0), wr, wr);
            double? wallAngleFromCloud = wallAngleHint.HasValue ? null : EstimateWallAngle(widePts, mid, wr);
            bool nearStraightWallLine = wallAngleHint.HasValue || wallAngleFromCloud.HasValue;

            ColumnFitResult round = o.AllowRound ? TryRound(pts, center, o) : null;
            if (round != null && round.Grade != "Check")
            {
                // The two clicks are the person's own read on the column's rough footprint. A "confident" round
                // classification whose diameter or center disagrees substantially with what the clicks imply isn't
                // trustworthy on its own - sparse, cluttered data near a wall corner can occasionally arc 65%+ of the
                // way around a spurious circle that has nothing to do with the column actually clicked.
                bool sizeOk = round.Diameter <= diag * 1.4 && round.Diameter >= diag * 0.5;
                bool centerOk = (round.Center - mid).Length <= o.MaxSize * 0.6;
                // Independent of size: two straight wall lines meeting at (or running past) a corner are made of a
                // lot of nearly-collinear points, and with a generous size range and tolerance, RANSAC can
                // occasionally stitch a handful of points from each wall into a "circle" that clears the
                // coverage/spread bar on its own, without any real curved surface anywhere. But a round column
                // standing right next to a real wall is a completely normal, common case too, and it still produces
                // its own tight, near-complete ring of points (reaching "Good": spread <= 6mm, coverage >= 80%) -
                // that combination essentially never happens by accident from two straight lines. So the nearby-line
                // veto only distrusts the WEAKER "OK" tier (looser fit, 60-80% coverage), where a corner illusion and
                // a partly-occluded real column are genuinely hard to tell apart from the fit alone; a "Good" round
                // fit is trusted regardless of what else is nearby.
                bool suspicious = round.Grade == "OK" && nearStraightWallLine;
                if (!sizeOk || !centerOk || suspicious) round.Grade = "Check";
            }
            var rf = o.AllowRectangular ? FitRectangleCore(pts, center, o) : null;
            // The user gave us the two true corners explicitly - if the scan runs out before reaching one of them (an
            // occluded or partly-missed side), extend just that side out to the click rather than reporting the column
            // shorter than it really is. A side with solid data of its own is left exactly as measured; the click only
            // fills in for a side the scan itself could not pin down.
            ColumnFitResult rect = rf.HasValue ? ExtendToClicks(rf.Value, cornerA, cornerB, o) : null;
            var result = ChooseShape(round, rect, pts, center, o);
            // A plain two-wall corner (no column at all) can still produce a LOW-CONFIDENCE rectangle here, not a clean
            // failure - a real, full-length wall corner has SOME material in every direction near the corner, which is
            // enough to pass the ordinary per-edge support checks even with nothing genuinely rectangular there. So the
            // fallback is used whenever ordinary detection is not CONFIDENT (fails outright, or only reaches "Check"),
            // not only when it fails outright - the wall-anchored click reconstruction is more trustworthy than an
            // ambiguous fit in exactly that situation.
            if (result.Success && result.Grade != "Check") return result;
            if (!o.AllowRectangular) return result;

            // Fresh-start rule: the two clicks are ALWAYS enough to place a column - the only question is how well it
            // can be oriented and corrected against the surrounding scan. widePts (already queried above) covers both
            // jobs: finding the wall's own direction when one is visible, and providing the data the fallback
            // rectangle's edges get snapped to below.
            string orientSource;
            double angle;
            if (wallAngleHint.HasValue) { angle = wallAngleHint.Value; orientSource = "wall-hint"; }
            else if (wallAngleFromCloud.HasValue) { angle = wallAngleFromCloud.Value; orientSource = "point-cloud"; }
            else { angle = 0.0; orientSource = "unoriented"; }
            var fromClicks = BuildFromClicksWithAngle(widePts, cornerA, cornerB, angle, o);
            if (!fromClicks.Success) return result.Success ? result : fromClicks;
            fromClicks.OrientationSource = orientSource;
            // A world-axis guess (no real direction found anywhere nearby) must never be presented as fully trustworthy,
            // even if its edges happened to snap cleanly to nearby data at that assumed angle by coincidence - the
            // rotation itself is still just an assumption, not a measurement, so this is always at best a "Check".
            if (orientSource == "unoriented" && fromClicks.Grade == "OK") fromClicks.Grade = "Check";
            return fromClicks;
        }

        /// <summary>
        /// Finds the dominant straight-line direction in the surrounding point cloud, with NO dependency on any
        /// modeled Revit wall - the wall or walls forming a corner are already visible in the scan as dense straight
        /// lines whether or not anyone has modeled them yet, which matters because columns are very often placed
        /// before walls in practice. Works like finding the orientation of a line in image processing: for each
        /// candidate rotation, points are binned along the perpendicular axis, and the rotation whose bins are most
        /// SHARPLY CONCENTRATED (real wall lines, viewed at their own angle, cluster tightly; every other angle
        /// spreads the same points out) wins. A confidence check - the winning angle's concentration against the
        /// typical concentration across all candidates - rejects the case where there is no real directional signal
        /// at all (pure clutter has no preferred angle, so every candidate scores about the same); returns null there
        /// rather than confidently committing to what would just be noise.
        /// </summary>
        private static double? EstimateWallAngle(List<P2> pts, P2 center, double radius)
        {
            var nearby = pts.Where(p => (p - center).Length <= radius).ToList();
            if (nearby.Count < 30) return null;
            const int steps = 180;         // 0.5 degree resolution over the 0-90 degree range (walls repeat every 90 degrees)
            const double bin = 0.03;
            var scores = new double[steps];
            for (int i = 0; i < steps; i++)
            {
                double theta = i * (Math.PI / 2) / steps;
                double c = Math.Cos(theta), s = Math.Sin(theta);
                var counts = new Dictionary<int, int>();
                foreach (var p in nearby)
                {
                    var d = p - center;
                    int b = (int)Math.Floor((-d.X * s + d.Y * c) / bin);
                    counts[b] = counts.TryGetValue(b, out int cnt) ? cnt + 1 : 1;
                }
                double score = 0; foreach (var cnt in counts.Values) score += (double)cnt * cnt;
                scores[i] = score;
            }
            int bestIdx = 0;
            for (int i = 1; i < steps; i++) if (scores[i] > scores[bestIdx]) bestIdx = i;
            var sorted = (double[])scores.Clone(); Array.Sort(sorted);
            double median = sorted[steps / 2];
            double confidence = median > 1e-9 ? scores[bestIdx] / median : 0;
            const double minConfidence = 3.0;    // real walls test at 4.5-7.0; pure clutter tests at 1.1-1.2 - a wide, safe margin
            return confidence >= minConfidence ? bestIdx * (Math.PI / 2) / steps : (double?)null;
        }

        /// <summary>Builds a rectangle directly from the two clicked corners, oriented to the given angle rather than
        /// anything detected from the data - the deliberate fallback for a column embedded in a wall corner (see
        /// <see cref="FitDiagonal(IWallPointSource,P2,P2,ColumnFitOptions,double?)"/>).
        ///
        /// A per-edge refinement against nearby scan data (pulling the exposed side or two of a partly-embedded column
        /// to sub-cm precision, while leaving the flush sides at click precision) was tried here and worked beautifully
        /// most of the time - but in testing it occasionally locked onto the wrong nearby structure entirely and
        /// produced a column roughly DOUBLE the true size, in about one case in five, with no reliable way to tell a
        /// good refinement from a bad one from the result alone. For something as consequential as a structural
        /// column's placement, a consistent method the user can predict is worth more than one that is usually better
        /// but occasionally badly wrong - so this stays deliberately simple: trust the two clicks and the wall's real
        /// direction, nothing more.</summary>
        private static ColumnFitResult BuildFromClicksWithAngle(List<P2> pts, P2 cornerA, P2 cornerB, double angle, ColumnFitOptions o)
        {
            double ang = NormalizeAngle(angle);
            P2 U = new P2(Math.Cos(ang), Math.Sin(ang)), V = new P2(-Math.Sin(ang), Math.Cos(ang));
            P2 mid = (cornerA + cornerB) * 0.5;
            double ua = (cornerA - mid).Dot(U), va = (cornerA - mid).Dot(V);
            double ub = (cornerB - mid).Dot(U), vb = (cornerB - mid).Dot(V);
            double u0 = Math.Min(ua, ub), u1 = Math.Max(ua, ub), v0 = Math.Min(va, vb), v1 = Math.Max(va, vb);
            double width = u1 - u0, depth = v1 - v0;
            if (width < o.MinSize || width > o.MaxSize) return Fail($"Column width {width * 1000:F0} mm (from the wall-aligned clicks) is outside the plausible range.");
            if (depth < o.MinSize || depth > o.MaxSize) return Fail($"Column depth {depth * 1000:F0} mm (from the wall-aligned clicks) is outside the plausible range.");

            // This still trusts the two clicks for POSITION and SIZE, exactly as before - an embedded column's own faces
            // often have no distinguishing signal at all, so there is nothing to precisely refine against (an earlier,
            // more ambitious attempt at per-edge refinement was tried and reverted after it occasionally locked onto the
            // wrong nearby structure and produced a column at roughly double the true size). What CAN be checked safely,
            // without adjusting any position, is far simpler: is there any real scan data near this rectangle AT ALL?
            // A genuinely embedded column always has real wall material coincident with its own flush sides, so this
            // passes for a real embedded column same as it would for a fully exposed one - it only catches the case
            // where the clicks landed in open space with nothing nearby, which is not a column at all.
            var proj = pts.Select(p => { var d = p - mid; return (u: d.Dot(U), v: d.Dot(V)); }).ToList();
            double tol = Math.Max(o.InlierTol * 4, 0.05);
            int NearEdge(bool isUEdge, double at, double lo, double hi) => proj.Count(p =>
            {
                double coord = isUEdge ? p.u : p.v, alongVal = isUEdge ? p.v : p.u;
                return Math.Abs(coord - at) <= tol && alongVal >= lo - tol && alongVal <= hi + tol;
            });
            int support = NearEdge(true, u0, v0, v1) + NearEdge(true, u1, v0, v1) + NearEdge(false, v0, u0, u1) + NearEdge(false, v1, u0, u1);
            if (support < Math.Max(6, o.MinInliers / 3))
                return Fail("No scan data found near this rectangle - click closer to where the point cloud actually shows the column or its host wall.");

            // A TIGHT, single-pass snap toward real data - deliberately conservative. A wider, iterative, coarse-to-fine
            // search was tried first and reverted: it could "walk" an edge estimate toward the wrong nearby structure
            // over several passes, occasionally landing on a column roughly double the true size. A single pass with a
            // moderate tolerance still helps with an imprecise click without that risk - and separately, ANY straight
            // line of points that runs well beyond the extent of this one edge (a different, nearby wall's own face,
            // which can fall inside the search window purely by coincidence of position) is excluded by requiring the
            // supporting points to actually span most of this edge's own length, not just cluster at one along-value.
            double snapTol = Math.Max(o.InlierTol * 6, 0.06);
            double margin = Math.Max(snapTol * 1.5, 0.04);
            int snapped = 0;
            double? SnapEdge(bool isUEdge, double at, double lo, double hi)
            {
                double m = Math.Min(margin, (hi - lo) * 0.4);
                var near = new List<double>(); var alongVals = new List<double>();
                foreach (var p in proj)
                {
                    double coord = isUEdge ? p.u : p.v, alongVal = isUEdge ? p.v : p.u;
                    if (alongVal < lo + m || alongVal > hi - m) continue;
                    if (Math.Abs(coord - at) <= snapTol) { near.Add(coord); alongVals.Add(alongVal); }
                }
                if (near.Count < 10) return null;
                double alongSpan = alongVals.Max() - alongVals.Min(), alongRange = (hi - m) - (lo + m);
                if (alongRange > 1e-6 && alongSpan < alongRange * 0.5) return null;
                return Stats.Median(near);
            }
            double? nu0 = SnapEdge(true, u0, v0, v1); if (nu0.HasValue) { u0 = nu0.Value; snapped++; }
            double? nu1 = SnapEdge(true, u1, v0, v1); if (nu1.HasValue) { u1 = nu1.Value; snapped++; }
            double? nv0 = SnapEdge(false, v0, u0, u1); if (nv0.HasValue) { v0 = nv0.Value; snapped++; }
            double? nv1 = SnapEdge(false, v1, u0, u1); if (nv1.HasValue) { v1 = nv1.Value; snapped++; }
            width = u1 - u0; depth = v1 - v0;
            if (width < o.MinSize || width > o.MaxSize) return Fail($"Column width {width * 1000:F0} mm (after snapping to nearby scan data) is outside the plausible range.");
            if (depth < o.MinSize || depth > o.MaxSize) return Fail($"Column depth {depth * 1000:F0} mm (after snapping to nearby scan data) is outside the plausible range.");
            P2 center = mid + U * (0.5 * (u0 + u1)) + V * (0.5 * (v0 + v1));

            return new ColumnFitResult
            {
                Success = true, Shape = ColumnShape.Rectangular, Center = center, Angle = ang, Width = width, Depth = depth,
                Spread = 0, Coverage = snapped / 4.0, InliersUsed = support,
                // OrientationSource is left at its "measured" default here and set by the caller (FitDiagonal), which
                // is the only place that knows whether the angle passed in came from a wall hint, the point cloud, or
                // neither.
                // any side left at click precision (no nearby data confidently found) keeps this at "Check"; with all
                // four sides snapped to real data it is exactly as trustworthy as an ordinary measured fit
                Grade = snapped == 4 ? "OK" : "Check",
            };
        }

        /// <summary>Pushes out any WEAKLY-supported edge of a rectangle fit to match the corresponding clicked corner, when the
        /// click lies further out than what the scan alone established. An edge with real data of its own is never moved.</summary>
        private static ColumnFitResult ExtendToClicks(RectFit fit, P2 cornerA, P2 cornerB, ColumnFitOptions o)
        {
            double Proj(P2 p, P2 axis) => (p - fit.seed).Dot(axis);
            double auA = Proj(cornerA, fit.U), avA = Proj(cornerA, fit.V);
            double auB = Proj(cornerB, fit.U), avB = Proj(cornerB, fit.V);
            double clickU0 = Math.Min(auA, auB), clickU1 = Math.Max(auA, auB);
            double clickV0 = Math.Min(avA, avB), clickV1 = Math.Max(avA, avB);

            double u0 = fit.u0, u1 = fit.u1, v0 = fit.v0, v1 = fit.v1;
            bool WeakLo(int cntLo, int cntHi, double covLo) => covLo < 0.4 || cntLo < 0.3 * Math.Max(cntLo, cntHi);
            if (clickU0 < u0 - o.InlierTol && WeakLo(fit.cU0, fit.cU1, fit.covU0)) u0 = clickU0;
            if (clickU1 > u1 + o.InlierTol && WeakLo(fit.cU1, fit.cU0, fit.covU1)) u1 = clickU1;
            if (clickV0 < v0 - o.InlierTol && WeakLo(fit.cV0, fit.cV1, fit.covV0)) v0 = clickV0;
            if (clickV1 > v1 + o.InlierTol && WeakLo(fit.cV1, fit.cV0, fit.covV1)) v1 = clickV1;

            double width = u1 - u0, depth = v1 - v0;
            if (width < o.MinSize || width > o.MaxSize || depth < o.MinSize || depth > o.MaxSize) return null;

            bool extended = u0 != fit.u0 || u1 != fit.u1 || v0 != fit.v0 || v1 != fit.v1;
            P2 fittedCenter = fit.seed + fit.U * (0.5 * (u0 + u1)) + fit.V * (0.5 * (v0 + v1));
            string grade = extended && fit.grade == "Good" ? "OK" : fit.grade;   // a click-extended side is a little less certain than fully measured data
            // The two clicks are the user's own, explicit statement of roughly how big this column is - the internal
            // spread/coverage checks alone can still be satisfied by a rectangle assembled from a nearby wall corner's
            // OWN crossing data (which has plenty of tight, well-covered material, just not material belonging to any
            // real column), producing a confidently-graded result that is nonetheless far bigger than anything the user
            // actually indicated. A result substantially larger than the click-implied diagonal is treated as unproven
            // regardless of its own internal grade, so a genuinely ambiguous wall corner still gets a chance to be
            // reconsidered (by the diagonal-mode click+wall-angle fallback) rather than being accepted outright.
            double clickDiag = (cornerB - cornerA).Length, fitDiag = Math.Sqrt(width * width + depth * depth);
            if (fitDiag > clickDiag * 1.4) grade = "Check";

            return new ColumnFitResult
            {
                Success = true, Shape = ColumnShape.Rectangular, Center = fittedCenter,
                Angle = NormalizeAngle(fit.angleU), Width = width, Depth = depth,
                Spread = fit.spread, Coverage = fit.coverage, InliersUsed = fit.inliers, Grade = grade,
            };
        }

        public static ColumnFitResult FitSides(IReadOnlyList<P2> points, P2 clickA, P2 clickB, ColumnFitOptions o = null)
            => FitSides(new ListPointSource(points), clickA, clickB, o);

        public static ColumnFitResult FitSides(IWallPointSource src, P2 clickA, P2 clickB, ColumnFitOptions o = null)
        {
            o = o ?? new ColumnFitOptions();
            if (!o.AllowRectangular) return Fail("Rectangular columns are switched off in PC Settings.");
            var d = clickB - clickA;
            double w0 = d.Length;
            if (w0 < o.MinSize * 0.5 || w0 > o.MaxSize * 1.3)
                return Fail($"Clicks are {w0 * 1000:F0} mm apart - outside the plausible column size range.");
            P2 mid = (clickA + clickB) * 0.5;
            // generous enough to also reach the OTHER (unclicked) pair of faces, wherever they turn out to be
            double r = Math.Max(w0, o.MinSize) * 1.5 + o.SeedBand + 0.25;
            List<P2> pts = src.Query(mid, new P2(1, 0), r, r);
            if (pts.Count < 2 * o.MinInliers) return Fail("Too few points around the clicks.");

            var fit = FitRectangleCore(pts, mid, o);
            if (fit == null) return Fail("No rectangular column found around your two clicks.");

            // Which of the two found axes did the user actually click ACROSS? That one is reported as "Width" - purely a
            // labeling choice, the fitted rectangle itself does not depend on which axis is picked as which.
            double clickAng = Math.Atan2(d.Y, d.X);
            double diffToU = Math.IEEERemainder(fit.Value.angleU - clickAng, Math.PI);
            bool useU = Math.Abs(diffToU) <= Math.PI / 4;
            double angle = useU ? fit.Value.angleU : fit.Value.angleU + Math.PI / 2;
            double width = useU ? fit.Value.width : fit.Value.depth;
            double depth = useU ? fit.Value.depth : fit.Value.width;

            return new ColumnFitResult
            {
                Success = true, Shape = ColumnShape.Rectangular, Center = fit.Value.center,
                Angle = NormalizeAngle(angle), Width = width, Depth = depth,
                Spread = fit.Value.spread, Coverage = fit.Value.coverage, InliersUsed = fit.Value.inliers, Grade = fit.Value.grade,
            };
        }

        // =====================================================================================
        //  Shared solver: given a candidate center and a point cloud around it, decide round vs rectangular
        // =====================================================================================
        private static ColumnFitResult Solve(List<P2> pts, P2 seedCenter, ColumnFitOptions o, double diagHint = 0)
        {
            if (pts.Count < 2 * o.MinInliers) return Fail("Too few points around the click.");
            P2 center = RefineCenter(pts, seedCenter, o);
            ColumnFitResult round = o.AllowRound ? TryRound(pts, center, o) : null;
            ColumnFitResult rect = o.AllowRectangular ? TryRectangle(pts, center, o, diagHint) : null;
            if (round == null && rect == null && !(o.AllowRound && o.AllowRectangular))
                return Fail("No " + (o.AllowRound ? "round" : "rectangular") + " column found near the click.");
            return ChooseShape(round, rect, pts, center, o);
        }

        /// <summary>Decides between a round and a rectangular hypothesis for the same patch of points (either may be null).</summary>
        private static ColumnFitResult ChooseShape(ColumnFitResult round, ColumnFitResult rect, List<P2> pts, P2 center, ColumnFitOptions o)
        {
            double nearby = pts.Count(p => (p - center).Length <= o.MaxSize * 0.75);      // the population either hypothesis must explain

            // A clearly well-fitting hypothesis beats a poorly-fitting one outright, before any fraction/score
            // comparison: under heavy noise a loose rectangle can occasionally rack up a raw inlier or score edge over a
            // genuinely tighter circle (or vice versa) without this.
            int GradeRank(string g) => g == "Good" ? 2 : g == "OK" ? 1 : 0;
            if (round != null && rect != null)
            {
                int rr = GradeRank(round.Grade), rc = GradeRank(rect.Grade);
                if (rr > rc) return round;
                if (rc > rr) return rect;
            }

            // A genuine circle of points, projected onto ANY axis, always shows peaks near its silhouette edges (the "arcsine
            // caustic"), so a rectangle fit routinely succeeds on round data too. The two are told apart by how much of the
            // ACTUAL point cloud each hypothesis accounts for: a real circle's rectangle fit only explains the four tangent
            // clusters near its silhouette (a MINORITY of the ring), while the circle fit explains nearly all of it. A real
            // rectangle's circle fit (only plausible if it's close to square) fails the same way in reverse. Whichever hypothesis
            // covers the larger share of the nearby points wins; ties fall back to the roughness/coverage score.
            if (nearby > 0)
            {
                double roundFrac = round == null ? 0 : round.InliersUsed / nearby;
                double rectFrac = rect == null ? 0 : rect.InliersUsed / nearby;
                // the fraction shortcut only decides between two hypotheses that both fit reasonably tightly; if the
                // "winning" one is a much rougher fit than the alternative, fall through to the spread-aware Score() below
                // instead - this is what stops a rectangle loosely draped across a noisy circle's tangent points from
                // out-voting a genuinely tighter (if lower-coverage) circle fit
                if (round != null && roundFrac >= rectFrac + 0.10 && (rect == null || round.Spread <= 2.5 * rect.Spread + 0.005)) return round;
                if (rect != null && rectFrac >= roundFrac + 0.10 && (round == null || rect.Spread <= 2.5 * round.Spread + 0.005)) return rect;
            }
            if (round != null && rect != null)
                return Score(round) >= Score(rect) ? round : rect;
            if (round != null) return round;
            if (rect != null) return rect;
            return Fail("No column-shaped cluster of points found near the click (not round, not rectangular).");
        }

        private static double Score(ColumnFitResult r) => r.Coverage * 2 - r.Spread * 50;

        /// <summary>
        /// The MEDIAN of nearby points, not their bounding-box midpoint: a bounding box is set by the single most extreme point
        /// in each direction, so a handful of scattered clutter points near the edge of the search area throws it off badly.
        /// The median stays with the majority - the column's own dense boundary - as long as clutter is a minority of the points,
        /// and the search radius shrinks each pass so clutter far from the true center drops out entirely.
        /// </summary>
        private static P2 RefineCenter(List<P2> pts, P2 seed, ColumnFitOptions o)
        {
            P2 c = seed;
            double radius = o.SearchRadius;
            for (int it = 0; it < 5; it++)
            {
                var near = pts.Where(p => (p - c).Length <= radius).ToList();
                if (near.Count < o.MinInliers) break;
                var xs = near.Select(p => p.X).ToList(); var ys = near.Select(p => p.Y).ToList();
                c = new P2(Stats.Median(xs), Stats.Median(ys));
                radius = Math.Max(o.MaxSize * 0.75, radius * 0.7);
            }
            return c;
        }

        // =====================================================================================
        //  Round columns
        // =====================================================================================
        private static ColumnFitResult TryRound(List<P2> pts, P2 seedCenter, ColumnFitOptions o)
        {
            var near = pts.Where(p => (p - seedCenter).Length <= o.MaxSize * 1.3).ToList();
            if (near.Count < o.MinInliers) return null;

            // Algebraic (Kasa) circle fit: correct even from a PARTIAL arc (e.g. a column standing against a wall, seen from
            // one side only) - unlike a radius histogram taken around an already-estimated center, which is itself biased
            // toward whichever side is visible and can be off by more than the column's own radius under heavy occlusion.
            // A single least-squares (Kasa) circle fit over ALL nearby points is not robust here: points far from the true
            // ring (background clutter) exert huge algebraic leverage and can pull the fitted radius wildly off before any
            // trimming happens - especially under partial occlusion, where the correction has less real ring data to fall back
            // on. A RANSAC seed (each candidate circle built from just 3 points, so no leverage effect) finds the right circle
            // first; Kasa is then used only to POLISH that already-clean inlier set for precision.
            if (!RansacCircle(near, o, out P2 center, out double r0, out List<P2> seedInliers)) return null;
            for (int pass = 0; pass < 3; pass++)
            {
                if (!KasaCircleFit(seedInliers, out P2 c2, out double r2)) break;
                center = c2; r0 = r2;
                seedInliers = near.Where(p => Math.Abs((p - center).Length - r0) <= 2.5 * o.InlierTol).ToList();
                if (seedInliers.Count < o.MinInliers) return null;
            }
            if (r0 * 2 < o.MinSize || r0 * 2 > o.MaxSize) return null;

            var tight = pts.Where(p => Math.Abs((p - center).Length - r0) <= o.InlierTol).ToList();
            if (tight.Count < o.MinInliers) return null;
            double med = Stats.Median(tight.Select(p => (p - center).Length).ToList());

            // angular coverage: how much of the ring's circumference has points
            const int bins = 24;
            var seen = new bool[bins];
            foreach (var p in tight)
            {
                double a = Math.Atan2(p.Y - center.Y, p.X - center.X);
                int k = (int)(((a + Math.PI) / (2 * Math.PI)) * bins) % bins;
                seen[k] = true;
            }
            double coverage = seen.Count(b => b) / (double)bins;
            if (coverage < o.RoundCoverage) return null;

            double spread = Stats2.MadSigma(tight.Select(p => (p - center).Length).ToList());
            string grade = spread <= 0.006 && coverage >= 0.8 ? "Good" : spread <= 0.012 && coverage >= 0.6 ? "OK" : "Check";
            return new ColumnFitResult
            {
                Success = true, Shape = ColumnShape.Round, Center = center, Diameter = med * 2,
                Spread = spread, Coverage = coverage, InliersUsed = tight.Count, Grade = grade,
            };
        }

        /// <summary>RANSAC circle fit: robust to clutter and partial arcs because each candidate circle is built from just
        /// 3 points (no leverage effect from distant outliers), and the best-supported candidate wins by inlier count.</summary>
        private static bool RansacCircle(List<P2> pts, ColumnFitOptions o, out P2 center, out double radius, out List<P2> inliers)
        {
            center = default; radius = 0; inliers = null;
            int n = pts.Count;
            if (n < o.MinInliers) return false;
            var rng = new Random(o.RandomSeed);
            int best = 0; P2 bestC = default; double bestR = 0;
            for (int it = 0; it < o.RansacIterations; it++)
            {
                P2 p1 = pts[rng.Next(n)], p2 = pts[rng.Next(n)], p3 = pts[rng.Next(n)];
                if (!CircleFrom3Points(p1, p2, p3, out P2 c, out double r)) continue;
                if (r * 2 < o.MinSize || r * 2 > o.MaxSize) continue;
                int cnt = 0;
                foreach (var p in pts) if (Math.Abs((p - c).Length - r) < 2.5 * o.InlierTol) cnt++;
                if (cnt > best) { best = cnt; bestC = c; bestR = r; }
            }
            if (best < o.MinInliers) return false;
            center = bestC; radius = bestR;
            inliers = pts.Where(p => Math.Abs((p - bestC).Length - bestR) < 2.5 * o.InlierTol).ToList();
            return true;
        }

        private static bool CircleFrom3Points(P2 a, P2 b, P2 c, out P2 center, out double radius)
        {
            center = default; radius = 0;
            double d = 2 * (a.X * (b.Y - c.Y) + b.X * (c.Y - a.Y) + c.X * (a.Y - b.Y));
            if (Math.Abs(d) < 1e-9) return false;
            double a2 = a.X * a.X + a.Y * a.Y, b2 = b.X * b.X + b.Y * b.Y, c2 = c.X * c.X + c.Y * c.Y;
            double ux = (a2 * (b.Y - c.Y) + b2 * (c.Y - a.Y) + c2 * (a.Y - b.Y)) / d;
            double uy = (a2 * (c.X - b.X) + b2 * (a.X - c.X) + c2 * (b.X - a.X)) / d;
            center = new P2(ux, uy);
            radius = (center - a).Length;
            return true;
        }

        /// <summary>Algebraic least-squares circle fit (Kasa method): solves for the circle through the points that minimizes
        /// the sum of squared ALGEBRAIC residuals. Works from a partial arc, not just a full ring, since it uses each point's
        /// curvature information rather than requiring points spread all the way around a pre-guessed center.</summary>
        private static bool KasaCircleFit(List<P2> pts, out P2 center, out double radius)
        {
            center = default; radius = 0;
            int n = pts.Count;
            if (n < 3) return false;
            double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y);     // center the data for numerical stability
            double sxx = 0, syy = 0, sxy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
            foreach (var p in pts)
            {
                double x = p.X - mx, y = p.Y - my, z = x * x + y * y;
                sxx += x * x; syy += y * y; sxy += x * y; sx += x; sy += y; sxz += x * z; syz += y * z; sz += z;
            }
            // solve [sxx sxy sx; sxy syy sy; sx sy n] * [A;B;C] = [sxz;syz;sz]   (A=2a, B=2b, C=r^2-a^2-b^2, in centered coords)
            double m00 = sxx, m01 = sxy, m02 = sx, rr0 = sxz;
            double m10 = sxy, m11 = syy, m12 = sy, rr1 = syz;
            double m20 = sx, m21 = sy, m22 = n, rr2 = sz;
            double det = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
            if (Math.Abs(det) < 1e-12) return false;
            double detA = rr0 * (m11 * m22 - m12 * m21) - m01 * (rr1 * m22 - m12 * rr2) + m02 * (rr1 * m21 - m11 * rr2);
            double detB = m00 * (rr1 * m22 - m12 * rr2) - rr0 * (m10 * m22 - m12 * m20) + m02 * (m10 * rr2 - rr1 * m20);
            double detC = m00 * (m11 * rr2 - rr1 * m21) - m01 * (m10 * rr2 - rr1 * m20) + rr0 * (m10 * m21 - m11 * m20);
            double A = detA / det, B = detB / det, C = detC / det;
            double a = A / 2, b = B / 2, r2v = C + a * a + b * b;
            if (r2v <= 0) return false;
            center = new P2(a + mx, b + my);
            radius = Math.Sqrt(r2v);
            return true;
        }

        // =====================================================================================
        //  Rectangular columns - shared by center/diagonal (TryRectangle) and FitSides
        // =====================================================================================
        private static ColumnFitResult TryRectangle(List<P2> pts, P2 center, ColumnFitOptions o, double diagHint)
        {
            var fit = FitRectangleCore(pts, center, o);
            if (fit == null) return null;
            return new ColumnFitResult
            {
                Success = true, Shape = ColumnShape.Rectangular, Center = fit.Value.center,
                Angle = NormalizeAngle(fit.Value.angleU), Width = fit.Value.width, Depth = fit.Value.depth,
                Spread = fit.Value.spread, Coverage = fit.Value.coverage, InliersUsed = fit.Value.inliers, Grade = fit.Value.grade,
            };
        }

        private struct RectFit { public P2 center, seed, U, V; public double angleU, width, depth, spread, coverage; public int inliers; public string grade; public double u0, u1, v0, v1; public int cU0, cU1, cV0, cV1; public double covU0, covU1, covV0, covV1; }

        /// <summary>
        /// Finds the smallest-area bounding box over any rotation, after removing sparse background clutter by local point
        /// density. This works whether the column's cross-section shows as a thin hollow outline or is more densely covered,
        /// and - unlike comparing how many points each face has to the others - it does not care whether the four sides were
        /// scanned with equal density: a real column commonly has denser data on the side facing the scanner and much sparser
        /// data on the far side (self-occlusion), and minimising the bounding box uses whatever real data exists on each side
        /// without weighing one against another. A stray unrelated structure (e.g. two perpendicular walls meeting at a corner)
        /// is still rejected: its minimal box has one or two sides with essentially NO real data near them at all, which the
        /// per-edge support check below catches directly.
        /// </summary>
        private static RectFit? FitRectangleCore(List<P2> raw, P2 seedCenter, ColumnFitOptions o)
        {
            var dense = DensityFilter(raw, o);
            if (dense.Count < o.MinInliers) return null;

            // Rotation search: minimise a TRIMMED bounding-box area (middle ~92% of the projected spread on each axis,
            // not the raw min/max) so that a handful of clutter points the density filter did not catch cannot single-
            // handedly drag the search to the wrong angle - a raw min/max search is dominated by whichever single point
            // sits furthest out at each candidate angle, which is precisely what one leftover outlier exploits.
            var sample = dense.Count > 6000 ? Stride(dense, 6000) : dense;
            double baseAng = 0, range = Math.PI / 2;
            foreach (int samples in new[] { 60, 40, 40 })
            {
                double step = Math.Max(range / samples, 5e-5);
                double best = double.MaxValue, cand = baseAng;
                for (double a = -range / 2; a <= range / 2 + 1e-12; a += step)
                {
                    double th = baseAng + a;
                    double c = Math.Cos(th), s = Math.Sin(th);
                    var us = new double[sample.Count]; var vs = new double[sample.Count];
                    for (int i = 0; i < sample.Count; i++)
                    {
                        var d = sample[i] - seedCenter;
                        us[i] = d.X * c + d.Y * s; vs[i] = -d.X * s + d.Y * c;
                    }
                    Array.Sort(us); Array.Sort(vs);
                    double area = TrimmedExtent(us, 0.04) * TrimmedExtent(vs, 0.04);
                    if (area < best) { best = area; cand = th; }
                }
                baseAng = cand; range = 4 * step;
            }

            // The area search above treats every point equally, so an edge with far more points than the others (heavier
            // scanning, overlapping passes, debris) can pull the whole box's rotation toward itself, subtly tilting even
            // the sparser, cleaner edges away from their true position. This refines the angle so each of the FOUR EDGES
            // gets an equal vote regardless of how many points landed on it - the same principle already used for a wall's
            // two faces, extended to all four sides of a rectangle.
            for (int pass = 0; pass < 2; pass++)
            {
                double c0 = Math.Cos(baseAng), s0 = Math.Sin(baseAng);
                P2 U0 = new P2(c0, s0), V0 = new P2(-s0, c0);
                var uv0 = dense.Select(p => { var d = p - seedCenter; return (u: d.Dot(U0), v: d.Dot(V0)); }).ToList();
                var su1 = uv0.Select(p => p.u).OrderBy(x => x).ToList();
                var sv1 = uv0.Select(p => p.v).OrderBy(x => x).ToList();
                double eu0 = TrimmedLow(su1, 0.04), eu1 = TrimmedHigh(su1, 0.04);
                double ev0 = TrimmedLow(sv1, 0.04), ev1 = TrimmedHigh(sv1, 0.04);
                double bandTol = Math.Max(o.InlierTol * 2.5, 0.012);

                double? EdgeSlope(bool isUEdge, double edgeVal, double loOther, double hiOther)
                {
                    var along = new List<double>(); var across = new List<double>();
                    foreach (var p in uv0)
                    {
                        double coord = isUEdge ? p.u : p.v, a = isUEdge ? p.v : p.u;
                        if (Math.Abs(coord - edgeVal) <= bandTol && a >= loOther - bandTol && a <= hiOther + bandTol) { along.Add(a); across.Add(coord); }
                    }
                    if (along.Count < 8) return null;
                    double ma = along.Average(), mc = across.Average(), num = 0, den = 0;
                    for (int i = 0; i < along.Count; i++) { double da = along[i] - ma; num += da * (across[i] - mc); den += da * da; }
                    return den < 1e-9 ? (double?)null : num / den;
                }
                // for a U-edge (u roughly constant along v), a small angle error dtheta shows as du/dv = -dtheta;
                // for a V-edge (v roughly constant along u), the same error shows as dv/du = +dtheta - converted here
                // so all four edges vote on the SAME dtheta, then averaged with equal weight (one vote per edge, not per point)
                var votes = new List<double>();
                var vU0 = EdgeSlope(true, eu0, ev0, ev1); if (vU0.HasValue) votes.Add(-vU0.Value);
                var vU1 = EdgeSlope(true, eu1, ev0, ev1); if (vU1.HasValue) votes.Add(-vU1.Value);
                var vV0 = EdgeSlope(false, ev0, eu0, eu1); if (vV0.HasValue) votes.Add(vV0.Value);
                var vV1 = EdgeSlope(false, ev1, eu0, eu1); if (vV1.HasValue) votes.Add(vV1.Value);
                if (votes.Count < 2) break;                          // not enough independent edges to correct the angle safely
                double dtheta = votes.Average();
                if (Math.Abs(dtheta) < 1e-6) break;
                baseAng += dtheta;
            }

            double C = Math.Cos(baseAng), S = Math.Sin(baseAng);
            P2 U = new P2(C, S), V = new P2(-S, C);
            var proj = dense.Select(p => { var d = p - seedCenter; return (u: d.Dot(U), v: d.Dot(V)); }).ToList();

            // A point sitting just outside a real edge cannot be told apart from a genuine edge point by "is it inside the
            // box plus some margin" - it will usually have plenty of real neighbours nearby (borrowed from the dense true
            // edge right next to it) and survive density filtering anyway. The fix used throughout this project for exactly
            // this problem (wall faces, the round column's radius) is the same here: start from an approximate edge position,
            // then refine it as the MEDIAN of only the points within a TIGHT tolerance of that specific edge - not a box
            // margin - iterated a couple of times. A stray point 3 cm off the true edge then has no way to pull the median.
            var su0 = proj.Select(p => p.u).OrderBy(x => x).ToList();
            var sv0 = proj.Select(p => p.v).OrderBy(x => x).ToList();
            double u0 = TrimmedLow(su0, 0.04), u1 = TrimmedHigh(su0, 0.04);
            double v0 = TrimmedLow(sv0, 0.04), v1 = TrimmedHigh(sv0, 0.04);
            double edgeTolRefine = Math.Max(o.InlierTol * 2.5, 0.012);
            for (int pass = 0; pass < 3; pass++)
            {
                double nu0 = MedianNear(proj, p => p.u, u0, edgeTolRefine, p => p.v, v0 - edgeTolRefine, v1 + edgeTolRefine) ?? u0;
                double nu1 = MedianNear(proj, p => p.u, u1, edgeTolRefine, p => p.v, v0 - edgeTolRefine, v1 + edgeTolRefine) ?? u1;
                double nv0 = MedianNear(proj, p => p.v, v0, edgeTolRefine, p => p.u, u0 - edgeTolRefine, u1 + edgeTolRefine) ?? v0;
                double nv1 = MedianNear(proj, p => p.v, v1, edgeTolRefine, p => p.u, u0 - edgeTolRefine, u1 + edgeTolRefine) ?? v1;
                u0 = nu0; u1 = nu1; v0 = nv0; v1 = nv1;
            }

            // A LOW-DENSITY scan systematically undershoots every edge: with few points, the outermost sample is expected
            // to fall short of the true boundary, by roughly HALF the typical spacing between points near that boundary
            // (a standard property of sampling a line at random - the same reasoning used, e.g., to say a fence post every
            // 2 m is on average 1 m from the actual end of the fence unless one is placed right there). Estimated from the
            // OTHER two edges' own points near this boundary (not this edge's own points, which are too few to judge
            // spacing from) and capped modestly so it only ever nudges a boundary out, never invents a large gap of data
            // that was never scanned at all.
            double EdgeExtension(bool isUBoundary, double boundary, int dir, double loOther, double hiOther)
            {
                double win = Math.Max(0.06, 0.15 * (hiOther - loOther));
                var vals = new List<double>();
                foreach (var p in proj)
                {
                    double coordOther = isUBoundary ? p.v : p.u;                 // the OTHER axis's edges supply the density estimate
                    if (coordOther < loOther - edgeTolRefine || coordOther > hiOther + edgeTolRefine) continue;
                    double along = isUBoundary ? p.u : p.v;
                    double d = dir > 0 ? boundary - along : along - boundary;    // distance inward from the boundary
                    if (d >= 0 && d <= win) vals.Add(along);
                }
                if (vals.Count < 4) return 0;
                vals.Sort();
                var gaps = new List<double>();
                for (int i = 1; i < vals.Count; i++) gaps.Add(vals[i] - vals[i - 1]);
                double medSpacing = Stats.Median(gaps);
                return Math.Min(0.06, Math.Max(0, medSpacing / 2 - o.InlierTol * 0.5));
            }
            u0 -= EdgeExtension(true, u0, -1, v0, v1);
            u1 += EdgeExtension(true, u1, +1, v0, v1);
            v0 -= EdgeExtension(false, v0, -1, u0, u1);
            v1 += EdgeExtension(false, v1, +1, u0, u1);

            double width = u1 - u0, depth = v1 - v0;
            if (width < o.MinSize || width > o.MaxSize || depth < o.MinSize || depth > o.MaxSize) return null;
            var clean = proj;

            // Per-edge support, checked against an ABSOLUTE floor (not against the other edges' counts): a real column
            // face - however sparse - still has SOME points close to it and spread along a decent share of its length.
            double edgeTol = Math.Max(o.InlierTol * 3, Math.Min(width, depth) * 0.06);
            (int cnt, double cov, double spread) EdgeStats(bool isUEdge, double edgeVal, double loOther, double hiOther)
            {
                var along = new List<double>(); var dev = new List<double>();
                foreach (var p in clean)
                {
                    double coord = isUEdge ? p.u : p.v, a = isUEdge ? p.v : p.u;
                    if (Math.Abs(coord - edgeVal) <= edgeTol && a >= loOther - edgeTol && a <= hiOther + edgeTol) { along.Add(a); dev.Add(coord - edgeVal); }
                }
                if (along.Count == 0) return (0, 0, 0);
                along.Sort();
                double edgeLen = hiOther - loOther;
                double cov = edgeLen > 1e-9 ? Math.Min(1.0, (along[along.Count - 1] - along[0]) / edgeLen) : 1.0;
                return (along.Count, cov, Stats2.MadSigma(dev));
            }
            var eU0 = EdgeStats(true, u0, v0, v1); var eU1 = EdgeStats(true, u1, v0, v1);
            var eV0 = EdgeStats(false, v0, u0, u1); var eV1 = EdgeStats(false, v1, u0, u1);
            // an absolute floor (not compared between edges, so genuinely uneven real density between faces still passes),
            // scaled a little by how many points survived filtering so a very sparse overall scan needs proportionally less
            int minEdgeCount = Math.Max(4, Math.Min(15, clean.Count / 50));
            double minEdgeCoverage = 0.18;
            if (eU0.cnt < minEdgeCount || eU1.cnt < minEdgeCount || eV0.cnt < minEdgeCount || eV1.cnt < minEdgeCount) return null;
            if (eU0.cov < minEdgeCoverage || eU1.cov < minEdgeCoverage || eV0.cov < minEdgeCoverage || eV1.cov < minEdgeCoverage) return null;

            double coverage = (eU0.cov + eU1.cov + eV0.cov + eV1.cov) / 4;
            double spread = (eU0.spread + eU1.spread + eV0.spread + eV1.spread) / 4;
            int inliers = eU0.cnt + eU1.cnt + eV0.cnt + eV1.cnt;
            P2 fittedCenter = seedCenter + U * (0.5 * (u0 + u1)) + V * (0.5 * (v0 + v1));
            string grade = spread <= 0.006 && coverage >= 0.5 ? "Good" : spread <= 0.012 && coverage >= 0.3 ? "OK" : "Check";

            return new RectFit { center = fittedCenter, seed = seedCenter, U = U, V = V, angleU = baseAng, width = width, depth = depth, spread = spread, coverage = coverage, inliers = inliers, grade = grade,
                                 u0 = u0, u1 = u1, v0 = v0, v1 = v1, cU0 = eU0.cnt, cU1 = eU1.cnt, cV0 = eV0.cnt, cV1 = eV1.cnt, covU0 = eU0.cov, covU1 = eU1.cov, covV0 = eV0.cov, covV1 = eV1.cov };
        }

        /// <summary>Extent of a SORTED array after trimming <paramref name="frac"/> off each end (a robust stand-in for max-min).</summary>
        private static double TrimmedExtent(double[] sorted, double frac)
        {
            int n = sorted.Length;
            int cut = Math.Max(0, (int)(n * frac));
            if (2 * cut >= n) cut = 0;
            return sorted[n - 1 - cut] - sorted[cut];
        }

        /// <summary>Median of the coordinate values (via <paramref name="coord"/>) for points within <paramref name="tol"/> of
        /// <paramref name="target"/>, restricted to <paramref name="along"/> in [<paramref name="lo"/>, <paramref name="hi"/>].
        /// Null if too few points qualify - the caller then keeps the previous estimate rather than jumping to a bad one.</summary>
        private static double? MedianNear(List<(double u, double v)> pts, Func<(double u, double v), double> coord, double target, double tol,
                                          Func<(double u, double v), double> along, double lo, double hi)
        {
            var near = new List<double>();
            foreach (var p in pts)
            {
                double a = along(p);
                if (a < lo || a > hi) continue;
                double c = coord(p);
                if (Math.Abs(c - target) <= tol) near.Add(c);
            }
            return near.Count < 3 ? (double?)null : Stats.Median(near);
        }

        private static double TrimmedLow(List<double> sorted, double frac)
        {
            int n = sorted.Count, cut = Math.Max(0, (int)(n * frac));
            if (2 * cut >= n) cut = 0;
            return sorted[cut];
        }

        private static double TrimmedHigh(List<double> sorted, double frac)
        {
            int n = sorted.Count, cut = Math.Max(0, (int)(n * frac));
            if (2 * cut >= n) cut = 0;
            return sorted[n - 1 - cut];
        }

        /// <summary>Keeps only points with enough near neighbours, so isolated background clutter is dropped before fitting
        /// while a genuine (even sparse) surface, whose points cluster together, is kept. Falls back to the raw list if
        /// filtering would leave too little to work with, rather than risk discarding real but thin data.</summary>
        private static List<P2> DensityFilter(List<P2> pts, ColumnFitOptions o)
        {
            if (pts.Count < 30) return pts;
            double cell = Math.Max(o.InlierTol * 5, 0.02);
            var grid = new Dictionary<(int, int), List<int>>();
            (int, int) CellOf(P2 p) => ((int)Math.Floor(p.X / cell), (int)Math.Floor(p.Y / cell));
            for (int i = 0; i < pts.Count; i++) { var c = CellOf(pts[i]); if (!grid.TryGetValue(c, out var l)) grid[c] = l = new List<int>(); l.Add(i); }

            int minNeighbors = Math.Max(2, Math.Min(8, pts.Count / 80));
            var keep = new List<P2>(pts.Count);
            foreach (var p in pts)
            {
                var (cx, cy) = CellOf(p);
                int cnt = -1;                                    // don't count the point itself
                for (int ix = cx - 1; ix <= cx + 1; ix++)
                    for (int iy = cy - 1; iy <= cy + 1; iy++)
                        if (grid.TryGetValue((ix, iy), out var l)) cnt += l.Count;
                if (cnt >= minNeighbors) keep.Add(p);
            }
            return keep.Count >= o.MinInliers ? keep : pts;
        }

        // =====================================================================================
        //  Small helpers
        // =====================================================================================
        /// <summary>Wraps the Width-axis angle into [0, 180 deg). A rectangle turned by 180 degrees is the same rectangle with the
        /// SAME width and depth, so this is safe. (Wrapping by 90 degrees - as an earlier version did - silently swaps which side
        /// is the width while keeping the Width/Depth values unchanged, which put columns 90 degrees off.)</summary>
        private static double NormalizeAngle(double a) { while (a < 0) a += Math.PI; while (a >= Math.PI) a -= Math.PI; return a; }

        private static List<P2> Stride(List<P2> g, int max)
        {
            int step = (int)Math.Ceiling(g.Count / (double)max);
            var r = new List<P2>();
            for (int i = 0; i < g.Count; i += step) r.Add(g[i]);
            return r;
        }

        private static int CountNear(List<double> v, double x, double tol) => v.Count(q => Math.Abs(q - x) < tol);

        private static ColumnFitResult Fail(string m) => new ColumnFitResult { Success = false, Message = m };
    }
}
