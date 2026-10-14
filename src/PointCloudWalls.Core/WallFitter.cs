using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Core
{
    /// <summary>How a wall end was decided.</summary>
    public enum EndType
    {
        /// <summary>Ran into the edge of the loaded data - the wall may continue.</summary>
        Open,
        /// <summary>The wall faces simply stop (occlusion / no more points).</summary>
        Gap,
        /// <summary>A real end surface (end face, door jamb) was found and used.</summary>
        Cap,
        /// <summary>The wall stops at another wall.</summary>
        Junction,
        /// <summary>No scan feature near your click, so the wall ends exactly where you clicked.</summary>
        Click,
    }

    /// <summary>Where the wall ends go in centerline / diagonal mode.</summary>
    public enum EndMode
    {
        /// <summary>Snap to a scan feature (end face, other wall) near each click; otherwise stay at the click.</summary>
        SnapToScan,
        /// <summary>Follow the wall to its real ends, wherever the clicks were along it.</summary>
        Automatic,
        /// <summary>Exactly at the clicks.</summary>
        ExactClicks,
    }

    public sealed class WallFitOptions
    {
        // ---- finding the wall
        /// <summary>How far (m) a click may be from its true wall face (auto-limited for thin walls).</summary>
        public double SeedBand = 0.08;
        /// <summary>Point-to-face distance (m) that still counts as "on the surface". ~ 2-3x scanner noise.</summary>
        public double InlierTol = 0.010;
        /// <summary>Sanity guard: wall direction may deviate at most this much from "perpendicular to the click line".</summary>
        public double MaxClickAngleDeg = 60;
        public double MinThickness = 0.05;
        public double MaxThickness = 1.50;
        /// <summary>First look-around radius (m) around the clicks.</summary>
        public double SearchRadius = 3.0;
        public int RansacIterations = 600;
        public int MinInliersPerSide = 15;
        public int RandomSeed = 12345;

        // ---- accuracy
        /// <summary>Fine alignment: pick the angle/faces that make both surfaces sharpest.</summary>
        public bool RefineAngle = true;
        /// <summary>Re-measure along the whole wall (walls longer than the first look-around).</summary>
        public bool FollowWall = true;
        public double ExtendStep = 3.0;
        public double MaxWallLength = 40.0;
        /// <summary>Half-width (m) of the strip loaded beyond each face while following a long wall.</summary>
        public double StripMargin = 0.20;
        public int MaxExtensions = 14;

        // ---- wall ends
        /// <summary>Wall extent stops growing when the faces are missing for longer than this (m): doors, wide crossing walls.</summary>
        public double MaxGap = 0.30;
        /// <summary>Use end surfaces / crossing walls to place the ends precisely.</summary>
        public bool DetectEnds = true;
        /// <summary>When the wall meets another wall: false = stop at its face, true = continue to its centerline.</summary>
        public bool JunctionToCenterline = false;
        public double MaxCrossThickness = 0.80;
        public double BinSize = 0.05;
        /// <summary>Minimum points per 5 cm step of a face to count as "wall" (lowered automatically for short walls).</summary>
        public int MinBinPoints = 3;
        /// <summary>The two centerline clicks may be this close (m). Shorter click lines only get their direction from the scan.</summary>
        public double MinCenterlineLength = 0.05;

        public WallFitOptions Clone() => (WallFitOptions)MemberwiseClone();

        // ---- centerline picking mode (two clicks along the wall's centre line)
        /// <summary>How far (m) your clicked centre line may be from the wall's true centre.</summary>
        public double CenterlineClickTol = 0.25;
        /// <summary>A wall end snaps to a scan feature (end face, other wall) within this distance (m) of your click.</summary>
        public double EndSnap = 0.40;
        /// <summary>Centerline / diagonal modes: how the wall ends are decided.</summary>
        public EndMode Ends = EndMode.SnapToScan;
        /// <summary>Allow walls where only ONE face was scanned.</summary>
        public bool AllowOneFace = true;
        /// <summary>When the scan gives NO evidence which side of a one-face wall is the body: false = refuse (ask the user to click
        /// on the body side), true = guess. Guessing is wrong half the time, so it is off.</summary>
        public bool GuessSide = false;
        /// <summary>Thickness (m) used when only one face is visible.</summary>
        public double AssumedThickness = 0.20;
        /// <summary>Diagonal mode: how close (m) each click must be to a wall surface (or to where a missing surface should be).</summary>
        public double DiagonalClickTol = 0.10;
        /// <summary>Let the amount of empty space on each side decide an unclear one-face wall. Off: a neighbouring wall on the body side
        /// makes that side look busy and this clue points the wrong way.</summary>
        public bool UseEmptySpace = false;
    }

    public sealed class WallFitResult
    {
        public bool Success;
        public string Message = "";

        public P2 Start, End;               // centerline, world meters
        public double Thickness;
        public P2 Direction;                // unit Start->End
        public P2 Normal;                   // unit, from face A (first click) to face B

        public double SpreadA, SpreadB;     // robust surface roughness of each face (m)
        public double Coverage;             // 0..1 share of the wall length backed by points on BOTH faces
        public int InliersA, InliersB;
        public double AngleSigmaDeg;        // purely statistical angle uncertainty
        public EndType AtStart, AtEnd;
        public int Extensions;              // how many times the wall was re-measured along its length
        public int PointsUsed;
        public bool OneFace;                // only face A is densely scanned; face B is virtual, the body lies on B's side
        public bool ThicknessAssumed;       // one-face walls: true = thickness taken from settings, false = measured from a sparse opposite face
        public string SideSource = "";      // how the body side was decided
        public string ThicknessSource = ""; // scan / sparse opposite face / your clicks / assumed (settings)
        public string Grade = "";           // Good / OK / Check

        public double Length => (End - Start).Length;
        public WallFitResult Clone() => (WallFitResult)MemberwiseClone();
        public double AngleDeg => Math.Atan2(Direction.Y, Direction.X) * 180.0 / Math.PI;

        public override string ToString() =>
            Success
                ? $"OK  t={Thickness * 1000:F1}mm  L={Length:F3}m  ang={AngleDeg:F3}deg  spread={SpreadA * 1000:F1}/{SpreadB * 1000:F1}mm  cov={Coverage * 100:F0}%  ends={AtStart}/{AtEnd}  [{Grade}]"
                : "FAIL: " + Message;
    }

    /// <summary>
    /// Fits a wall (two parallel faces) to a 2D slice of a point cloud from two rough clicks, one on each face.
    /// Pure math, no Revit dependency. Units: meters.
    /// </summary>
    public static class WallFitter
    {
        private struct L { public double S, U; }   // local coordinates in a click/wall frame

        // =====================================================================================
        //  Entry points
        // =====================================================================================
        public static WallFitResult Fit(IReadOnlyList<P2> points, P2 clickA, P2 clickB, WallFitOptions o = null)
            => Fit(new ListPointSource(points), clickA, clickB, o);

        public static WallFitResult Fit(IWallPointSource src, P2 clickA, P2 clickB, WallFitOptions o = null)
        {
            o = o ?? new WallFitOptions();
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var d = clickB - clickA;
            double w0 = d.Length;
            if (w0 < o.MinThickness * 0.5 || w0 > o.MaxThickness)
                return Fail($"Clicks are {w0 * 1000:F0} mm apart - outside the plausible wall thickness range.");

            P2 n0 = d.Normalized(), t0 = n0.Perp, mid = (clickA + clickB) * 0.5;
            double sA = -w0 / 2, sB = +w0 / 2;

            // ---- pass 1: square look-around, find the two faces --------------------------------
            List<P2> world = src.Query(mid, t0, o.SearchRadius, o.SearchRadius);
            var pts = new List<L>(world.Count);
            foreach (var p in world) { var v = p - mid; pts.Add(new L { S = v.Dot(n0), U = v.Dot(t0) }); }
            if (pts.Count < 2 * o.MinInliersPerSide) return Fail("Too few points around the clicks.");

            double band = Math.Min(o.SeedBand, 0.45 * w0);
            if (!FindFacePair(pts, sA, sB, band, o, new Random(o.RandomSeed), out double theta, out double cA, out double cB))
                return Fail("No pair of parallel faces found through the two clicks.");

            WallFitResult res = Solve(world, mid, t0, n0, theta, cA, cB, mid, o, mid, t0, o.SearchRadius, o.SearchRadius);
            if (!res.Success || !o.FollowWall) return res;

            // ---- pass 2..n: follow the wall along its length in a narrow strip -----------------
            for (int it = 1; it <= o.MaxExtensions; it++)
            {
                if (res.AtStart != EndType.Open && res.AtEnd != EndType.Open) break;
                if (res.Length >= o.MaxWallLength) break;

                P2 c = (res.Start + res.End) * 0.5;
                double half = Math.Min(res.Length / 2 + o.ExtendStep, o.MaxWallLength / 2);
                double hw = res.Thickness / 2 + o.StripMargin;
                List<P2> strip = src.Query(c, res.Direction, half, hw);
                WallFitResult r2 = Solve(strip, c, res.Direction, res.Normal, 0.0, -res.Thickness / 2, res.Thickness / 2,
                                         mid, o, c, res.Direction, half, hw);
                if (!r2.Success) break;
                double growth = r2.Length - res.Length;
                r2.Extensions = it;
                res = r2;
                if (growth < 0.05 && it > 1) break;
            }
            return res;
        }

        // =====================================================================================
        //  Centerline mode: two clicks along the wall's centre line (start and end, approximately).
        //  Works when a face is missing: one visible face + an assumed thickness is accepted.
        // =====================================================================================
        public static WallFitResult FitAlong(IReadOnlyList<P2> points, P2 startClick, P2 endClick, WallFitOptions o = null)
            => FitAlong(new ListPointSource(points), startClick, endClick, o);

        public static WallFitResult FitAlong(IWallPointSource src, P2 startClick, P2 endClick, WallFitOptions o = null)
        {
            o = o ?? new WallFitOptions();
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var dv = endClick - startClick;
            double len = dv.Length;
            if (len < o.MinCenterlineLength)
                return Fail("The two clicks are only " + (len * 100).ToString("0") + " cm apart. Click the two ends of the wall (at least " + (o.MinCenterlineLength * 100).ToString("0") + " cm apart).");
            P2 t0 = dv.Normalized(), n0 = t0.Perp, mid = (startClick + endClick) * 0.5;

            // SHORT walls: the clicked line is too short to give the direction, so the scan decides (within a generous angle),
            // only the neighbourhood of the clicks is used, and fewer points are enough.
            bool shortWall = len < 1.0;
            if (shortWall)
            {
                o = o.Clone();
                o.MinInliersPerSide = Math.Max(5, (int)Math.Round(o.MinInliersPerSide * len));
                o.MinBinPoints = 2;
                o.EndSnap = Math.Min(o.EndSnap, Math.Max(0.05, len / 2));
                o.CenterlineClickTol = Math.Min(o.CenterlineClickTol, Math.Max(0.10, len));   // clicks on a short wall are close to it
            }

            bool auto = o.Ends == EndMode.Automatic;
            double halfLen = len / 2 + (auto ? o.ExtendStep : o.EndSnap + 0.3);
            double halfWid = Math.Min(o.CenterlineClickTol + o.MaxThickness / 2 + 0.1, 1.0);
            List<P2> world = src.Query(mid, t0, halfLen, halfWid);
            var pts = new List<L>(world.Count);
            foreach (var p in world) { var v = p - mid; pts.Add(new L { U = v.Dot(t0), S = v.Dot(n0) }); }
            if (pts.Count < 2 * o.MinInliersPerSide) return Fail("Too few points along your two clicks.");

            double maxAngDeg = shortWall
                ? Math.Min(90.0, Math.Atan2(0.10, len) * 180 / Math.PI + 3)              // clicks ~5 cm off => big angle error on a short line
                : Math.Min(len < 2.0 ? 45.0 : 20.0, Math.Max(2.0, Math.Atan2(2 * o.CenterlineClickTol, len) * 180 / Math.PI + 1));
            double maxAng = maxAngDeg * Math.PI / 180;

            var scoring = pts;
            if (shortWall)
            {
                double rr = len / 2 + 0.35;
                scoring = pts.Where(p => Math.Sqrt(p.U * p.U + p.S * p.S) <= rr).ToList();
                if (scoring.Count < 2 * o.MinInliersPerSide) scoring = pts;
            }
            // how many scan points lie on each side of a candidate face (5 m deep): used to tell the room side from the unscanned side
            Func<double, double, (int pos, int neg)> probe = (th, cFace) =>
            {
                P2 ax = t0 * Math.Cos(th) + n0 * Math.Sin(th);
                P2 nx = t0 * (-Math.Sin(th)) + n0 * Math.Cos(th);
                P2 fm = mid + nx * cFace;
                // measured along the MIDDLE of the wall only: a neighbouring wall at a corner (which extends toward the room from inside
                // a building, but toward the wall body from outside) must not influence the result
                double depth = 5.0, halfL = Math.Max(0.25, len / 2 - 0.4);
                int nPos = src.Query(fm + nx * (0.05 + depth / 2), ax, halfL, depth / 2).Count;
                int nNeg = src.Query(fm - nx * (0.05 + depth / 2), ax, halfL, depth / 2).Count;
                return (nPos, nNeg);
            };
            if (!FindLines(scoring, len, maxAng, o, out double theta, out double cA, out double cB, out bool oneFace, out string why,
                           out string sideSource, out bool assumed, probe))
                return Fail(why);

            var clip = new Clip { A = startClick, B = endClick, Snap = o.Ends == EndMode.ExactClicks ? 0 : o.EndSnap, NoClip = auto,
                                  SeedReach = (int)Math.Ceiling(len / 2 / o.BinSize) + 2 };
            WallFitResult r = Solve(world, mid, t0, n0, theta, cA, cB, mid, o, mid, t0, auto ? halfLen : 1e9, auto ? halfWid : 1e9, oneFace, clip);
            if (auto && r.Success && o.FollowWall) r = FollowAuto(src, r, oneFace, mid, o, clip);
            if (r.Success && oneFace)
            {
                r.SideSource = sideSource; r.ThicknessAssumed = assumed;
                r.ThicknessSource = assumed ? "assumed (PC Settings)" : "sparse opposite face";
            }
            else if (r.Success) r.ThicknessSource = "scan";
            return r;
        }

        // =====================================================================================
        //  Diagonal mode: the START click on ONE side of the wall, the END click on the OTHER side.
        //  The wall body lies BETWEEN the two clicks, so a wall with a missing face still gets the right side
        //  (and a thickness), without any guessing.
        // =====================================================================================
        public static WallFitResult FitDiagonal(IReadOnlyList<P2> points, P2 startClick, P2 endClick, WallFitOptions o = null)
            => FitDiagonal(new ListPointSource(points), startClick, endClick, o);

        public static WallFitResult FitDiagonal(IWallPointSource src, P2 startClick, P2 endClick, WallFitOptions o = null)
        {
            o = o ?? new WallFitOptions();
            var o0 = o;
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var dv = endClick - startClick;
            double len = dv.Length;
            if (len < o.MinCenterlineLength)
                return Fail("The two clicks are only " + (len * 100).ToString("0") + " cm apart. Click the two ends of the wall (at least " + (o.MinCenterlineLength * 100).ToString("0") + " cm apart).");
            P2 t0 = dv.Normalized(), n0 = t0.Perp, mid = (startClick + endClick) * 0.5;

            bool shortWall = len < 1.0;
            if (shortWall)
            {
                o = o.Clone();
                o.MinInliersPerSide = Math.Max(5, (int)Math.Round(o.MinInliersPerSide * len));
                o.MinBinPoints = 2;
                o.EndSnap = Math.Min(o.EndSnap, Math.Max(0.05, len / 2));
            }

            bool auto = o.Ends == EndMode.Automatic;
            double halfLen = len / 2 + (auto ? o.ExtendStep : o.EndSnap + 0.3);
            double halfWid = Math.Min(o.MaxThickness / 2 + o.DiagonalClickTol + 0.35, 1.0);
            List<P2> world = src.Query(mid, t0, halfLen, halfWid);
            var pts = new List<L>(world.Count);
            foreach (var p in world) { var v = p - mid; pts.Add(new L { U = v.Dot(t0), S = v.Dot(n0) }); }
            if (pts.Count < 2 * o.MinInliersPerSide) return Fail("Too few points along your two clicks.");

            // the clicked line is skewed by the wall thickness, so allow a wider direction search than the centerline mode
            double maxAng = Math.Min(60.0, Math.Atan2(0.30 + o.DiagonalClickTol, len) * 180 / Math.PI + 3) * Math.PI / 180;
            var scoring = pts;
            if (shortWall)
            {
                double rr = len / 2 + 0.35;
                scoring = pts.Where(p => Math.Sqrt(p.U * p.U + p.S * p.S) <= rr).ToList();
                if (scoring.Count < 2 * o.MinInliersPerSide) scoring = pts;
            }
            if (!Analyze(scoring, len, maxAng, o, out Analysis an, out string why)) return Fail(why);

            double th = an.Theta, C = Math.Cos(th), S = Math.Sin(th);
            double AcrossOf(P2 p) { var v = p - mid; return -S * v.Dot(t0) + C * v.Dot(n0); }
            double cs = AcrossOf(startClick), ce = AcrossOf(endClick);
            double lo = Math.Min(cs, ce), hi = Math.Max(cs, ce), sep = hi - lo;
            if (sep < Math.Max(0.8 * o.MinThickness, 0.06))
            {
                // both clicks are on one line (a few cm of sideways wobble is not a diagonal): treat as a centerline pick.
                var rc = FitAlong(src, startClick, endClick, o0);
                if (!rc.Success) rc.Message = "Your two clicks are on the same line. In diagonal mode click the START on one side of the wall and the END on the OTHER side.  (" + rc.Message + ")";
                return rc;
            }

            // ---- the dense line nearest each click ---------------------------------------------------------------------
            double tolD = o.DiagonalClickTol;
            var peaks = an.Peaks;
            int iLo = -1, iHi = -1;
            for (int i = 0; i < peaks.Count; i++)
            {
                double dLo = Math.Abs(peaks[i].c - lo), dHi = Math.Abs(peaks[i].c - hi);
                if (dLo <= tolD && dLo <= dHi) { if (iLo < 0 || peaks[i].cnt > peaks[iLo].cnt) iLo = i; }
                else if (dHi <= tolD) { if (iHi < 0 || peaks[i].cnt > peaks[iHi].cnt) iHi = i; }
            }
            double cA, cB; bool oneFace; string thickSource;
            bool bothOk = false;
            if (iLo >= 0 && iHi >= 0)
            {
                double sepP = peaks[iHi].c - peaks[iLo].c;
                double maxC = peaks.Max(p => p.cnt);
                double ov = Math.Min(peaks[iLo].a1, peaks[iHi].a1) - Math.Max(peaks[iLo].a0, peaks[iHi].a0);
                double shorter = Math.Min(peaks[iLo].a1 - peaks[iLo].a0, peaks[iHi].a1 - peaks[iHi].a0);
                bothOk = sepP >= o.MinThickness && sepP <= o.MaxThickness
                         && Math.Min(peaks[iLo].cnt, peaks[iHi].cnt) >= Math.Max(o.MinInliersPerSide, 0.3 * maxC)   // two real faces, not a face and a dash
                         && shorter > 1e-6 && ov / shorter >= 0.4;                                                    // running alongside each other
            }
            if (bothOk)
            {
                cA = peaks[iLo].c; cB = peaks[iHi].c; oneFace = false; thickSource = "scan";
            }
            else if (iLo < 0 && iHi < 0)
            {
                return Fail("No wall surface found near either of your two clicks (each must be within " + (tolD * 100).ToString("0") + " cm of a scanned surface, or of where a missing surface should be).");
            }
            else
            {
                // one dense face; the other click marks where the (missing) face should be
                int iv = iLo >= 0 && (iHi < 0 || peaks[iLo].cnt >= peaks[iHi].cnt) ? iLo : iHi;
                bool visibleIsLow = iv == iLo;
                double cVis = peaks[iv].c;
                double otherClick = visibleIsLow ? hi : lo;
                double sgn = visibleIsLow ? 1 : -1;
                double T = Math.Max(o.MinThickness, Math.Min(o.MaxThickness, Math.Abs(otherClick - cVis)));
                cA = cVis; cB = cVis + sgn * T; oneFace = true; thickSource = "your clicks";

                // if a few sparse points mark the other face near the second click, measure the thickness from them instead
                double tol = o.InlierTol;
                var sa = an.Across.OrderBy(x => x).ToArray();
                int LB(double x) { int a = 0, b = sa.Length; while (a < b) { int m = (a + b) / 2; if (sa[m] < x) a = m + 1; else b = m; } return a; }
                int Cnt(double x0, double x1) => LB(x1) - LB(x0);
                double bestX = 0; int bestC = 0;
                for (double d = o.MinThickness; d <= o.MaxThickness; d += 0.0025)
                {
                    double x = cVis + sgn * d;
                    if (Math.Abs(x - otherClick) > tolD) continue;
                    int cnt = Cnt(x - 2 * tol, x + 2 * tol);
                    if (cnt > bestC) { bestC = cnt; bestX = x; }
                }
                if (bestC > 0)
                {
                    int c1 = Cnt(bestX - 2 * tol, bestX + 2 * tol), c2 = Cnt(bestX - 5 * tol, bestX + 5 * tol);
                    if (c1 >= Math.Max(Math.Min(10, o.MinInliersPerSide), 0.005 * peaks[iv].cnt) && c1 >= 0.6 * c2)
                    { cB = bestX; thickSource = "sparse opposite face"; }
                }
            }

            var clip = new Clip { A = startClick, B = endClick, Snap = o.Ends == EndMode.ExactClicks ? 0 : o.EndSnap, NoClip = auto,
                                  SeedReach = (int)Math.Ceiling(len / 2 / o.BinSize) + 2 };
            WallFitResult r = Solve(world, mid, t0, n0, th, cA, cB, mid, o, mid, t0, auto ? halfLen : 1e9, auto ? halfWid : 1e9, oneFace, clip);
            if (auto && r.Success && o.FollowWall) r = FollowAuto(src, r, oneFace, mid, o, clip);
            if (r.Success)
            {
                r.ThicknessSource = thickSource;
                if (oneFace) { r.SideSource = "your two clicks (one on each side of the wall)"; r.ThicknessAssumed = thickSource == "your clicks"; }
            }
            return r;
        }

        // =====================================================================================
        //  Automatic mode: no clicks. WallDetector already found the wall roughly (centre line + thickness), so the
        //  search for the faces is skipped and the SAME solver as the click tools measures it. The detected ends play
        //  the role of the two clicks: each snaps to a scan feature (end face, other wall) near it, otherwise it stays.
        // =====================================================================================
        public static WallFitResult FitDetected(IReadOnlyList<P2> points, P2 start, P2 end, double thickness, WallFitOptions o = null)
            => FitDetected(new ListPointSource(points), start, end, thickness, o);

        public static WallFitResult FitDetected(IWallPointSource src, P2 start, P2 end, double thickness, WallFitOptions o = null)
        {
            o = o ?? new WallFitOptions();
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var dv = end - start;
            double len = dv.Length;
            if (len < o.MinCenterlineLength) return Fail("Detected wall is too short.");
            if (thickness < 0.5 * o.MinThickness || thickness > o.MaxThickness) return Fail("Detected thickness is outside the plausible wall thickness range.");
            P2 t0 = dv.Normalized(), n0 = t0.Perp, mid = (start + end) * 0.5;

            if (len < 1.0)
            {
                o = o.Clone();
                o.MinInliersPerSide = Math.Max(5, (int)Math.Round(o.MinInliersPerSide * len));
                o.MinBinPoints = 2;
                o.EndSnap = Math.Min(o.EndSnap, Math.Max(0.05, len / 2));
            }

            double halfLen = len / 2 + o.EndSnap + 0.3;
            double halfWid = thickness / 2 + Math.Max(o.StripMargin, 0.20);
            List<P2> world = src.Query(mid, t0, halfLen, halfWid);
            if (world.Count < 2 * o.MinInliersPerSide) return Fail("Too few points along the detected wall.");

            var clip = new Clip { A = start, B = end, Snap = o.EndSnap, SeedReach = (int)Math.Ceiling(len / 2 / o.BinSize) + 2 };
            WallFitResult r = Solve(world, mid, t0, n0, 0.0, -thickness / 2, thickness / 2, mid, o, mid, t0, 1e9, 1e9, false, clip);
            if (r.Success) r.ThicknessSource = "scan";
            return r;
        }

        /// <summary>Automatic mode, a wall scanned from ONE side: the detected surface (faceStart -> faceEnd), the unit vector from that
        /// surface into the wall body, and the thickness to assume. Same solver, same "one face" result as the click tools.</summary>
        public static WallFitResult FitDetectedOneFace(IWallPointSource src, P2 faceStart, P2 faceEnd, P2 intoBody, double thickness, WallFitOptions o = null)
        {
            o = o ?? new WallFitOptions();
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var dv = faceEnd - faceStart;
            double len = dv.Length;
            if (len < o.MinCenterlineLength) return Fail("Detected surface is too short.");
            P2 t0 = dv.Normalized(), n0 = t0.Perp, mid = (faceStart + faceEnd) * 0.5;
            double sgn = intoBody.Dot(n0) >= 0 ? 1 : -1;

            double halfLen = len / 2 + o.EndSnap + 0.3;
            double halfWid = thickness + Math.Max(o.StripMargin, 0.20);
            List<P2> world = src.Query(mid, t0, halfLen, halfWid);
            if (world.Count < o.MinInliersPerSide) return Fail("Too few points along the detected surface.");

            var clip = new Clip { A = faceStart, B = faceEnd, Snap = o.EndSnap, SeedReach = (int)Math.Ceiling(len / 2 / o.BinSize) + 2 };
            WallFitResult r = Solve(world, mid, t0, n0, 0.0, 0.0, sgn * thickness, mid, o, mid, t0, 1e9, 1e9, true, clip);
            if (r.Success) { r.ThicknessAssumed = true; r.ThicknessSource = "assumed (PC Settings)"; }
            return r;
        }

        /// <summary>Automatic ends: re-measure the wall in a narrow strip along its length until both ends are decided by the scan
        /// (end face, other wall, faces that simply stop) instead of by the edge of the loaded region.</summary>
        private static WallFitResult FollowAuto(IWallPointSource src, WallFitResult res, bool oneFace, P2 seed, WallFitOptions o, Clip clip)
        {
            for (int it = 1; it <= o.MaxExtensions; it++)
            {
                if (res.AtStart != EndType.Open && res.AtEnd != EndType.Open) break;
                if (res.Length >= o.MaxWallLength) break;
                P2 c = (res.Start + res.End) * 0.5;
                double half = Math.Min(res.Length / 2 + o.ExtendStep, o.MaxWallLength / 2);
                double hw = res.Thickness / 2 + o.StripMargin;
                List<P2> strip = src.Query(c, res.Direction, half, hw);
                // in the strip frame Normal points from the scanned face (A) to the other side (B)
                WallFitResult r2 = Solve(strip, c, res.Direction, res.Normal, 0.0, -res.Thickness / 2, res.Thickness / 2,
                                         seed, o, c, res.Direction, half, hw, oneFace, clip);
                if (!r2.Success) break;
                double growth = r2.Length - res.Length;
                r2.Extensions = it;
                res = r2;
                if (growth < 0.05 && it > 1) break;
            }
            return res;
        }

        private sealed class Analysis
        {
            public double Theta;
            public double[] Across, Along;
            public List<(double c, int cnt, double a0, double a1, double run)> Peaks;
        }

        /// <summary>Wall direction (the angle that makes the parallel lines sharpest, within +-maxAng of the clicked direction)
        /// and the dense lines (peaks) at that direction.</summary>
        private static bool Analyze(List<L> pts, double len, double maxAng, WallFitOptions o, out Analysis res, out string why)
        {
            res = null; why = "";
            var coarse = Stride(pts, 12000);
            var fine = Stride(pts, 40000);
            double maxU = Math.Max(len / 2, 0.5);
            double baseBin = Math.Max(0.0005, o.InlierTol / 4);

            double best = 0, range = maxAng;
            int stage = 0;
            foreach (double bin in new[] { baseBin * 4, baseBin, baseBin / 2 })
            {
                var g = stage++ == 0 ? coarse : fine;
                double step = Math.Max(bin / (2 * maxU), 2e-6);
                double bestScore = double.MinValue, cand = best;
                for (double d = -range; d <= range + 1e-12; d += step)
                {
                    double th = best + d;
                    double sc = Sharp(g, th, bin);
                    if (sc > bestScore) { bestScore = sc; cand = th; }
                }
                best = cand;
                range = 3 * step;
            }
            double theta = best;

            double c = Math.Cos(theta), s = Math.Sin(theta);
            int n = fine.Count;
            var across = new double[n]; var along = new double[n];
            double mn = double.MaxValue, mx = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                across[i] = -s * fine[i].U + c * fine[i].S;
                along[i] = c * fine[i].U + s * fine[i].S;
                if (across[i] < mn) mn = across[i]; if (across[i] > mx) mx = across[i];
            }
            double bin1 = baseBin;
            int nb = (int)((mx - mn) / bin1) + 1;
            var h = new int[nb];
            for (int i = 0; i < n; i++) h[(int)((across[i] - mn) / bin1)]++;
            var sm = new int[nb];
            for (int k = 0; k < nb; k++) sm[k] = h[k] + (k > 0 ? h[k - 1] : 0) + (k < nb - 1 ? h[k + 1] : 0);
            var order = Enumerable.Range(0, nb).OrderByDescending(k => sm[k]).ToList();
            int sepBins = Math.Max(1, (int)Math.Ceiling(0.03 / bin1));
            var peaks = new List<(double c, int cnt, double a0, double a1, double run)>();
            var used = new List<int>();
            int top = sm[order[0]];
            foreach (int k in order)
            {
                if (sm[k] < Math.Max(o.MinInliersPerSide * 0.4, 0.1 * top) || peaks.Count >= 8) break;
                if (used.Any(u => Math.Abs(u - k) < sepBins)) continue;
                used.Add(k);
                double c0 = mn + (k + 0.5) * bin1;
                var near = new List<double>();
                for (int i = 0; i < n; i++)
                    if (Math.Abs(across[i] - c0) <= Math.Max(2 * bin1, 0.5 * o.InlierTol)) near.Add(across[i]);
                if (near.Count < 3) continue;
                int cnt = 0; double cc = Stats.Median(near);
                var inl = new List<double>();
                for (int i = 0; i < n; i++) if (Math.Abs(across[i] - cc) < o.InlierTol) { cnt++; inl.Add(along[i]); }
                if (cnt < o.MinInliersPerSide) continue;
                inl.Sort();
                // longest contiguous stretch (gaps <= 25 cm): a real face runs ALONG the wall, an end face is a short dash at one spot
                int rs = 0, rBest = 0, rBestStart = 0;
                for (int q = 1; q <= inl.Count; q++)
                    if (q == inl.Count || inl[q] - inl[q - 1] > 0.25) { if (q - rs > rBest) { rBest = q - rs; rBestStart = rs; } rs = q; }
                double runSpan = inl[rBestStart + rBest - 1] - inl[rBestStart];
                if (rBest < 0.5 * inl.Count) runSpan = 0;                        // most points are elsewhere: not one line
                peaks.Add((cc, cnt, inl[(int)(inl.Count * 0.02)], inl[(int)(inl.Count * 0.98)], runSpan));
            }
            if (peaks.Count == 0) { why = "No wall faces found along your two clicks (nothing dense enough near that line)."; return false; }
            double minRun = Math.Min(0.15, 0.6 * len);
            peaks = peaks.Where(p => p.run >= minRun).ToList();
            if (peaks.Count == 0) { why = "No wall faces found along your two clicks (only short pieces, no line running along the wall)."; return false; }
            res = new Analysis { Theta = theta, Across = across, Along = along, Peaks = peaks };
            return true;
        }

        /// <summary>
        /// Finds the wall direction (sharpest set of parallel lines within +-maxAng of the clicked direction) and then the
        /// faces: the best PAIR of parallel peaks a plausible thickness apart that brackets the clicked line, else one face.
        /// </summary>
        private static bool FindLines(List<L> pts, double len, double maxAng, WallFitOptions o,
                                      out double theta, out double cA, out double cB, out bool oneFace, out string why,
                                      out string sideSource, out bool assumed, Func<double, double, (int pos, int neg)> probe)
        {
            theta = cA = cB = 0; oneFace = false; why = ""; sideSource = ""; assumed = false;
            if (!Analyze(pts, len, maxAng, o, out Analysis an, out why)) return false;
            theta = an.Theta;
            var across = an.Across; var along = an.Along; var peaks = an.Peaks;
            int n = across.Length;
            double baseBin = Math.Max(0.0005, o.InlierTol / 4);
            double bin1 = baseBin;
            double tolClick = o.CenterlineClickTol;
            double maxCnt = peaks.Max(p => p.cnt);

            // ---- best pair ----------------------------------------------------------------------------------
            double pairScore = 0; int bi = -1, bj = -1;
            for (int i = 0; i < peaks.Count; i++)
                for (int j = i + 1; j < peaks.Count; j++)
                {
                    double lo = Math.Min(peaks[i].c, peaks[j].c), hi = Math.Max(peaks[i].c, peaks[j].c), sep = hi - lo;
                    if (sep < o.MinThickness || sep > o.MaxThickness) continue;
                    if (Math.Min(peaks[i].cnt, peaks[j].cnt) < Math.Max(o.MinInliersPerSide, 0.3 * maxCnt)) continue;
                    double ov = Math.Min(peaks[i].a1, peaks[j].a1) - Math.Max(peaks[i].a0, peaks[j].a0);
                    double shorter = Math.Min(peaks[i].a1 - peaks[i].a0, peaks[j].a1 - peaks[j].a0);
                    double overlap = shorter <= 1e-6 ? 0 : Math.Max(0, Math.Min(1, ov / shorter));
                    if (overlap < 0.4) continue;
                    double mid = 0.5 * (lo + hi);
                    if (lo - tolClick > 0 || hi + tolClick < 0) continue;              // click line nowhere near this pair
                    // a wall has no dense line INSIDE it: a pair with another face between them is two different walls
                    bool between = false;
                    for (int q = 0; q < peaks.Count && !between; q++)
                        if (q != i && q != j && peaks[q].c > lo + 0.02 && peaks[q].c < hi - 0.02 &&
                            peaks[q].cnt >= 0.4 * Math.Min(peaks[i].cnt, peaks[j].cnt)) between = true;
                    if (between) continue;
                    double centered = 1.0 / (1.0 + (mid / tolClick) * (mid / tolClick));
                    double contains = (lo <= 0 && 0 <= hi) ? 1.5 : 1.0;
                    double thin = 1.0 / (1.0 + (sep / 0.6) * (sep / 0.6));       // the faces nearest the clicked line are the wall's
                    double score = 2.0 * Math.Min(peaks[i].cnt, peaks[j].cnt) * overlap * centered * contains * thin;
                    if (score > pairScore) { pairScore = score; bi = i; bj = j; }
                }
            if (bi >= 0)
            {
                cA = Math.Min(peaks[bi].c, peaks[bj].c); cB = Math.Max(peaks[bi].c, peaks[bj].c);
                return true;
            }

            // ---- one face only ------------------------------------------------------------------------------------
            if (!o.AllowOneFace) { why = "Could not find both wall faces near your clicks (one-face walls are switched off in PC Settings)."; return false; }
            double bs = 0; int bk = -1;
            for (int i = 0; i < peaks.Count; i++)
            {
                if (Math.Abs(peaks[i].c) > tolClick + 0.05) continue;
                double centered = 1.0 / (1.0 + (peaks[i].c / tolClick) * (peaks[i].c / tolClick));
                double score = peaks[i].cnt * centered;
                if (score > bs) { bs = score; bk = i; }
            }
            if (bk < 0) { why = "No wall face found within " + (tolClick * 100).ToString("0") + " cm of your clicked line."; return false; }
            cA = peaks[bk].c;
            double faceC = cA;                                  // copy: out parameters cannot be used inside local functions
            int mainCnt = peaks[bk].cnt;
            double mainA0 = peaks[bk].a0, mainA1 = peaks[bk].a1;
            double tol = o.InlierTol;

            // sorted view of the points by across-offset, for fast window counts
            var idx = Enumerable.Range(0, n).OrderBy(i => across[i]).ToArray();
            var sa = idx.Select(i => across[i]).ToArray();
            int LowerBound(double x) { int lo = 0, hi = n; while (lo < hi) { int m = (lo + hi) / 2; if (sa[m] < x) lo = m + 1; else hi = m; } return lo; }
            int CountIn(double x0, double x1) => LowerBound(x1) - LowerBound(x0);

            // ---- cue 1: a SPARSE opposite face (a few dots of the other face are enough to fix side AND thickness)
            double maxWeak = Math.Min(o.MaxThickness, 0.6);
            double bestWeak = 0, weakC = 0; int weakSide = 0;
            foreach (int side in new[] { 1, -1 })
            {
                double bestCnt = 0, bestX = 0;
                for (double d = o.MinThickness; d <= maxWeak; d += bin1)
                {
                    double x = cA + side * d;
                    int cnt = CountIn(x - tol, x + tol);
                    double mid2 = 0.5 * (cA + x);
                    double sc = cnt * (1.0 / (1.0 + (mid2 / tolClick) * (mid2 / tolClick)));
                    if (sc > bestCnt) { bestCnt = sc; bestX = x; }
                }
                if (bestCnt <= 0) continue;
                int c1 = CountIn(bestX - 2 * tol, bestX + 2 * tol), c2 = CountIn(bestX - 5 * tol, bestX + 5 * tol);   // a line: almost everything near it is ON it (real scans are fuzzy)
                if (c1 < Math.Max(Math.Min(10, o.MinInliersPerSide), 0.005 * mainCnt) || c1 < 0.6 * c2) continue;                 // too few / not a sharp line
                var al = new List<double>();
                for (int k = LowerBound(bestX - tol); k < n && sa[k] <= bestX + tol; k++) al.Add(along[idx[k]]);
                al.Sort();
                // the longest continuous stretch (gaps <= 25 cm): a real face runs ALONG the wall; wall end faces or a crossing
                // wall are short lines at one spot and must not be mistaken for it
                int segStart = 0, bestStart = 0, bestLen = 0;
                for (int q = 1; q <= al.Count; q++)
                    if (q == al.Count || al[q] - al[q - 1] > 0.25)
                    {
                        if (q - segStart > bestLen) { bestLen = q - segStart; bestStart = segStart; }
                        segStart = q;
                    }
                double w0 = al[bestStart], w1 = al[bestStart + bestLen - 1];
                double overlap = Math.Min(w1, mainA1) - Math.Max(w0, mainA0);
                if (w1 - w0 < Math.Min(0.15, 0.6 * len) || bestLen < 0.5 * al.Count || bestLen < Math.Max(Math.Min(10, o.MinInliersPerSide), 0.005 * mainCnt)
                    || overlap < 0.10 * Math.Max(mainA1 - mainA0, 1e-6)) continue;
                if (bestCnt > bestWeak) { bestWeak = bestCnt; weakC = bestX; weakSide = side; }
            }
            // where the user's clicks sit relative to the scanned face (the click line runs at across = 0 in this frame)
            double clickThr = Math.Max(0.02, Math.Min(0.05, 0.4 * o.AssumedThickness));
            int clickSide = Math.Abs(cA) >= clickThr ? ((0 - cA) >= 0 ? 1 : -1) : 0;
            const string conflict = "Your clicks and the scan disagree about which side of the dots the wall body is on. " +
                                    "Use PC Wall - diagonal: click the start on one side of the wall and the end on the other side.";
            if (weakSide != 0)
            {
                if (clickSide != 0 && clickSide != weakSide) { why = conflict; return false; }
                cB = weakC; oneFace = true; assumed = false; sideSource = "sparse opposite face";
                return true;
            }

            // ---- cue 2: straight lines PERPENDICULAR to our wall on one side (crossing walls, end faces, things against the wall).
            //      For each line: d0 = where it starts (distance from the scanned face), len = how far it runs.
            //        - an END FACE spans the wall thickness starting at the face          => body is on that side
            //        - a crossing wall from the OTHER room starts one thickness away       => body is on that side
            //        - something touching the face in the MIDDLE of the wall, longer than a thickness (a wall meeting it
            //          from the room)                                                      => that side is the ROOM
            //      (Things touching the face at the wall's ENDS say nothing: an interior corner points to the room,
            //       an outside corner points to the body.)
            double capMax = Math.Max(0.40, 0.5 * o.MaxThickness);
            int minLinePts = Math.Max(8, o.MinInliersPerSide / 2);
            // a wall line seen from the side is about as dense (points per metre) as the scanned face itself; scattered clutter is far sparser
            double minLineDens = Math.Max(25.0, 0.25 * mainCnt / Math.Max(0.5, mainA1 - mainA0));
            List<(double a, double d0, double len)> Lines(int side)
            {
                var lines = new List<(double a, double d0, double len)>();
                var pl = new List<(double a, double d)>();
                for (int i = 0; i < n; i++)
                {
                    double d = (across[i] - faceC) * side;
                    if (d <= 2 * tol || d >= 2.5) continue;
                    if (along[i] < mainA0 - 0.25 || along[i] > mainA1 + 0.25) continue;
                    pl.Add((along[i], d));
                }
                if (pl.Count < minLinePts) return lines;
                double aMin = pl.Min(p => p.a), aMax = pl.Max(p => p.a);
                int nbA = (int)((aMax - aMin) / 0.03) + 1;
                var cntA = new int[nbA];
                foreach (var p in pl) cntA[(int)((p.a - aMin) / 0.03)]++;
                var usedBins = new List<int>();
                foreach (int k in Enumerable.Range(0, nbA).OrderByDescending(k => cntA[k]).Take(12))
                {
                    if (cntA[k] < minLinePts) break;
                    if (usedBins.Any(u => Math.Abs(u - k) <= 2)) continue;
                    usedBins.Add(k);
                    double aStar = aMin + (k + 0.5) * 0.03;
                    var ds = pl.Where(p => Math.Abs(p.a - aStar) <= 0.015).Select(p => p.d).OrderBy(d => d).ToList();   // straight: +-2 cm
                    if (ds.Count < minLinePts) continue;
                    double prev = ds[0]; int chain = 1;
                    for (int q = 1; q < ds.Count && ds[q] - prev <= 0.08; q++) { prev = ds[q]; chain++; }
                    double lineLen = prev - ds[0] + 0.01;
                    // a real wall line is DENSE along its length; scattered clutter that happens to share an along-position is not
                    if (chain >= minLinePts && chain / Math.Max(lineLen, 0.05) >= minLineDens) lines.Add((aStar, ds[0], lineLen));
                }
                return lines;
            }
            // "touching the face" allows for a sparse start of the line
            double nearThr = Math.Max(o.MinThickness * 0.8, 0.10);
            bool BodyLine((double a, double d0, double len) l) =>
                (l.d0 < nearThr && l.len >= o.MinThickness * 0.8 && l.len <= capMax) ||                              // end face
                (l.d0 >= nearThr && l.d0 <= o.MaxThickness && l.len >= 0.10);                                       // crossing wall, one thickness away
            bool RoomLine((double a, double d0, double len) l) =>
                l.d0 < nearThr && l.len > capMax && l.a > mainA0 + 0.30 && l.a < mainA1 - 0.30;                     // attached in the middle

            var linesPos = Lines(+1); var linesNeg = Lines(-1);
            bool bodyPos = linesPos.Any(BodyLine), bodyNeg = linesNeg.Any(BodyLine);
            bool roomPos = linesPos.Any(RoomLine), roomNeg = linesNeg.Any(RoomLine);

            // ---- the evidence, best first --------------------------------------------------------------------------------
            // votes for "the wall BODY is on this side of the scanned face"
            int votePos = (bodyPos ? 1 : 0) + (roomNeg ? 1 : 0), voteNeg = (bodyNeg ? 1 : 0) + (roomPos ? 1 : 0);
            bool bodyCue = bodyPos || bodyNeg;
            double sgn = 0;
            if (votePos != voteNeg && (votePos == 0 || voteNeg == 0))
            {
                sgn = votePos > 0 ? 1 : -1;
                sideSource = bodyCue ? "crossing wall / end face" : "wall meeting the scanned face from the room";
            }

            // ---- empty space (OFF by default): a neighbouring wall on the body side makes that side look busy, so this clue can
            //      point the wrong way
            if (sgn == 0 && probe != null && o.UseEmptySpace)
            {
                var cnt = probe(theta, faceC);
                int hi = Math.Max(cnt.pos, cnt.neg), lo = Math.Min(cnt.pos, cnt.neg);
                if (hi >= 150 && lo <= 0.15 * hi) { sgn = cnt.pos < cnt.neg ? 1 : -1; sideSource = "empty space behind the wall"; }
            }

            // ---- your click. If the scan points the other way, do NOT choose: ask the user.
            if (sgn != 0 && clickSide != 0 && sgn != clickSide) { why = conflict; return false; }
            if (sgn == 0 && clickSide != 0) { sgn = clickSide; sideSource = "your click"; }

            // ---- last resort: only if switched on. Otherwise refuse - a wall on the wrong side is worse than no wall.
            if (sgn == 0 && o.GuessSide)
            {
                int nPos = CountIn(cA + 0.05, cA + 1.0), nNeg = CountIn(cA - 1.0, cA - 0.05);
                sgn = nPos <= nNeg ? 1 : -1; sideSource = "guess";
            }
            if (sgn == 0)
            {
                why = "Only one face is scanned and the side is unclear. Use PC Wall - diagonal (start on one side of the wall, end on the other), " +
                      "or click the center line on the wall's BODY side of the dots.";
                return false;
            }
            cB = cA + sgn * o.AssumedThickness;
            oneFace = true; assumed = true;
            return true;
        }

        // =====================================================================================
        //  Core solver for one set of points in a given frame
        // =====================================================================================
        private sealed class Clip { public P2 A, B; public double Snap; public int SeedReach; public bool NoClip; }

        private static WallFitResult Solve(IReadOnlyList<P2> world, P2 O, P2 T, P2 N, double theta, double cA, double cB,
                                           P2 seedWorld, WallFitOptions o, P2 rCenter, P2 rAxis, double rHalfLen, double rHalfWid,
                                           bool oneFace = false, Clip clip = null)
        {
            WallFitResult Fail(string m) => new WallFitResult { Success = false, Message = m };

            var pts = new List<L>(world.Count);
            foreach (var p in world) { var v = p - O; pts.Add(new L { U = v.Dot(T), S = v.Dot(N) }); }
            if (pts.Count < 2 * o.MinInliersPerSide) return Fail("Too few points.");

            // ---- 1. coarse joint refinement (median offsets, shared slope) ----------------------
            var iA = new List<int>(); var iB = new List<int>();
            double vt = cB - cA;                                  // signed virtual thickness (one-face walls only)
            foreach (double mult in new[] { 4.0, 3.0, 2.0, 1.5, 1.0 })
            {
                double ct = Math.Cos(theta), st = Math.Sin(theta);
                double tol = Math.Min(o.InlierTol * mult, 0.4 * Math.Abs(cB - cA));
                Collect(pts, ct, st, cA, cB, tol, iA, iB, oneFace);
                if (iA.Count < o.MinInliersPerSide || (!oneFace && iB.Count < o.MinInliersPerSide))
                    return Fail("Lost a face during refinement (faces not supported by enough points).");

                double num = 0, den = 0;
                foreach (var set in oneFace ? new[] { iA } : new[] { iA, iB })
                {
                    double mu = set.Average(i => ct * pts[i].U + st * pts[i].S);
                    double ms = set.Average(i => -st * pts[i].U + ct * pts[i].S);
                    foreach (var i in set)
                    {
                        double a = ct * pts[i].U + st * pts[i].S - mu;
                        double c = -st * pts[i].U + ct * pts[i].S - ms;
                        num += a * c; den += a * a;
                    }
                }
                if (den < 1e-9) return Fail("Degenerate geometry (inliers concentrated at one spot).");
                theta += Math.Atan(num / den);

                double ct2 = Math.Cos(theta), st2 = Math.Sin(theta);
                cA = Stats.Median(iA.Select(i => -st2 * pts[i].U + ct2 * pts[i].S).ToList());
                cB = oneFace ? cA + vt : Stats.Median(iB.Select(i => -st2 * pts[i].U + ct2 * pts[i].S).ToList());
            }

            // ---- 2. fine alignment: angle + face positions at the sharpest surfaces ---------------
            if (o.RefineAngle) PeakRefine(pts, ref theta, ref cA, ref cB, o, oneFace, vt);

            double C = Math.Cos(theta), S = Math.Sin(theta);
            double thickness = Math.Abs(cB - cA);
            if (!oneFace && (thickness < o.MinThickness || thickness > o.MaxThickness))
                return Fail($"Fitted thickness {thickness * 1000:F0} mm is outside [{o.MinThickness * 1000:F0}, {o.MaxThickness * 1000:F0}] mm.");
            double sign = cB >= cA ? 1 : -1;
            double cLo = Math.Min(cA, cB), cHi = Math.Max(cA, cB);

            // ---- 3. surface roughness of each face ------------------------------------------------
            var devA = new List<double>(); var devB = new List<double>();
            double wide = Math.Min(2.5 * o.InlierTol, 0.4 * thickness);
            foreach (var p in pts)
            {
                double c = -S * p.U + C * p.S;
                if (Math.Abs(c - cA) < wide) devA.Add(c);
                else if (!oneFace && Math.Abs(c - cB) < wide) devB.Add(c);
            }
            if (devA.Count < o.MinInliersPerSide || (!oneFace && devB.Count < o.MinInliersPerSide)) return Fail("Lost a face in the final pass.");
            double spreadA = Stats2.MadSigma(devA), spreadB = oneFace ? Stats2.MadSigma(devA) : Stats2.MadSigma(devB);
            double tolF = Math.Min(Math.Max(o.InlierTol, 2.0 * Math.Max(spreadA, spreadB)), 0.35 * thickness);

            // ---- 4. classify every point along the wall ------------------------------------------
            const double latZone = 0.15;
            double m = Math.Max(2 * tolF, 0.02);
            double interiorW = thickness - 2 * m;
            bool capOK = o.DetectEnds && interiorW >= 0.03;

            // one-face walls: the visible face is A, the body (and the unscanned side) lies towards B - so only the
            // A side can show a crossing wall
            bool latLow = true, latHigh = true;
            if (oneFace) { if (cA < cB) latHigh = false; else latLow = false; }

            int n = pts.Count;
            var al = new double[n]; var ac = new double[n];
            double amin = double.MaxValue, amax = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                al[i] = C * pts[i].U + S * pts[i].S;
                ac[i] = -S * pts[i].U + C * pts[i].S;
                if (ac[i] >= cLo - latZone && ac[i] <= cHi + latZone) { amin = Math.Min(amin, al[i]); amax = Math.Max(amax, al[i]); }
            }
            if (amin > amax) return Fail("No points along the fitted wall.");

            double bin = o.BinSize;
            amin -= 5 * bin; amax += 5 * bin;          // padding: end surfaces sit at the very edge of the data
            int nBins = (int)Math.Floor((amax - amin) / bin) + 1;
            var faceCnt = new int[nBins]; var faceA = new int[nBins]; var faceB = new int[nBins]; var intCnt = new int[nBins];
            var intAlong = new List<double>(); var latAlong = new List<double>();
            var faceAlongs = new List<double>(); var faceAlongsA = new List<double>(); var faceAlongsB = new List<double>();
            for (int i = 0; i < n; i++)
            {
                if (ac[i] < cLo - latZone || ac[i] > cHi + latZone) continue;
                int k = (int)Math.Floor((al[i] - amin) / bin);
                if (Math.Abs(ac[i] - cA) < tolF) { faceCnt[k]++; faceAlongs.Add(al[i]); faceA[k]++; faceAlongsA.Add(al[i]); }
                else if (!oneFace && Math.Abs(ac[i] - cB) < tolF) { faceCnt[k]++; faceAlongs.Add(al[i]); faceB[k]++; faceAlongsB.Add(al[i]); }
                else if (capOK && ac[i] > cLo + m && ac[i] < cHi - m) { intCnt[k]++; intAlong.Add(al[i]); }
                else if ((latLow && ac[i] < cLo - 2 * tolF) || (latHigh && ac[i] > cHi + 2 * tolF)) latAlong.Add(al[i]);
            }
            int nFace = faceAlongs.Count;
            if (nFace < (oneFace ? 1 : 2) * o.MinInliersPerSide) return Fail("Lost a face in the final pass.");
            double span = Math.Max(0.5, faceAlongs.Max() - faceAlongs.Min());
            double rho = nFace / span / (oneFace ? 1 : 2) * 2;           // "both faces together" density, whatever we have
            int faceMin = Math.Max(o.MinBinPoints, (int)Math.Round(0.25 * rho * bin));
            int capMin = Math.Max(o.MinBinPoints + 1, (int)Math.Round(0.35 * (rho / 2) * Math.Max(interiorW, 0)));
            int latMin = Math.Max(o.MinBinPoints + 1, (int)Math.Round(0.2 * (rho / 2) * latZone));
            int gapBins = (int)Math.Ceiling(o.MaxGap / bin);
            int capReach = (int)Math.Ceiling(0.12 / bin);   // a real end surface sits right where the faces stop
            intAlong.Sort(); latAlong.Sort();

            // seed position along the wall
            var sv = seedWorld - O; double su = sv.Dot(T), ss = sv.Dot(N);
            double seedAlong = C * su + S * ss;
            int seedBin = (int)Math.Floor((seedAlong - amin) / bin);
            int start = -1;
            int seedReach = clip != null ? clip.SeedReach : gapBins;
            for (int k = 0; k <= seedReach && start < 0; k++)
            {
                int l = seedBin - k, r = seedBin + k;
                if (l >= 0 && l < nBins && faceCnt[l] >= faceMin) start = l;
                else if (r >= 0 && r < nBins && faceCnt[r] >= faceMin) start = r;
            }
            if (start < 0) return Fail("Clicks are not on the wall faces (no wall points near the click position).");

            int CapScore(int k) { int s = 0; for (int j = k - 1; j <= k + 1; j++) if (j >= 0 && j < nBins) s += intCnt[j]; return s; }
            double MedianIn(List<double> sorted, double lo, double hi, out int count)
            {
                int i0 = sorted.BinarySearch(lo); if (i0 < 0) i0 = ~i0;
                var win = new List<double>();
                for (int i = i0; i < sorted.Count && sorted[i] <= hi; i++) win.Add(sorted[i]);
                count = win.Count;
                return count == 0 ? double.NaN : Stats.Median(win);
            }
            // A crossing wall's face shows as a tight line of points just outside our faces.
            double? LateralCluster(double endAlong, int dir)
            {
                double lo = dir > 0 ? endAlong - 0.06 : endAlong - 0.12, hi = dir > 0 ? endAlong + 0.12 : endAlong + 0.06;
                double med = MedianIn(latAlong, lo, hi, out int cnt);
                if (cnt < latMin) return null;
                MedianIn(latAlong, med - 0.03, med + 0.03, out int tight);
                if (tight < latMin || tight < 0.6 * cnt) return null;
                return med;
            }
            double? SecondLateral(double pos, int dir)
            {
                for (double x = 0.10; x <= o.MaxCrossThickness; x += 0.03)
                {
                    double q = pos + dir * x;
                    double med = MedianIn(latAlong, q - 0.03, q + 0.03, out int cnt);
                    if (cnt >= latMin) return med;
                }
                return null;
            }

            (double pos, EndType type) Grow(int from, int dir)
            {
                int lastFace = from, miss = 0;
                for (int k = from + dir; k >= 0 && k < nBins; k += dir)
                {
                    bool supported = faceCnt[k] >= faceMin;
                    if (supported && miss > 0)
                    {
                        // resuming after a gap: needs a short run of supported bins, otherwise it is just a perpendicular
                        // surface (e.g. the far face of a crossing wall) cutting through our strip
                        int k1 = k + dir, k2 = k + 2 * dir;
                        supported = k1 >= 0 && k1 < nBins && k2 >= 0 && k2 < nBins && faceCnt[k1] >= faceMin && faceCnt[k2] >= faceMin;
                    }
                    if (supported) { lastFace = k; miss = 0; continue; }
                    miss++;
                    if (capOK && miss <= capReach && CapScore(k) >= capMin)
                    {
                        double p = MedianIn(intAlong, amin + (k - 1) * bin, amin + (k + 2) * bin, out int cc);
                        if (cc > 0) return (p, EndType.Cap);
                    }
                    if (miss > gapBins) break;
                }
                // no end surface: the faces simply stop. Use the furthest face point of the last supported bin, plus a
                // thinly filled bin right after it (the wall rarely ends exactly on a bin edge).
                int lastB = lastFace, nb1 = lastFace + dir;
                if (nb1 >= 0 && nb1 < nBins && faceCnt[nb1] >= 2) lastB = nb1;
                double lim = amin + (dir > 0 ? (lastB + 1) * bin : lastB * bin);
                double endA = dir > 0 ? faceAlongs.Where(a => a < lim).Max() : faceAlongs.Where(a => a >= lim).Min();
                if (o.DetectEnds)
                {
                    double? lat = LateralCluster(endA, dir);
                    if (lat.HasValue)
                    {
                        double pos = lat.Value;
                        if (o.JunctionToCenterline)
                        {
                            double? far = SecondLateral(pos, dir);
                            if (far.HasValue) pos = 0.5 * (pos + far.Value);
                        }
                        return (pos, EndType.Junction);
                    }
                }
                return (endA, EndType.Gap);
            }

            var lowEnd = Grow(start, -1);
            var highEnd = Grow(start, +1);
            if (clip != null && !clip.NoClip)
            {
                // The user told us where the wall starts and ends. A scan feature near the click refines it; otherwise the click wins.
                var ca = clip.A - O; var cb = clip.B - O;
                double aa = C * ca.Dot(T) + S * ca.Dot(N), ab = C * cb.Dot(T) + S * cb.Dot(N);
                double clickLo = Math.Min(aa, ab), clickHi = Math.Max(aa, ab);
                int snapBins = (int)Math.Ceiling(clip.Snap / bin);

                // For each end: search for a real end feature (end face, other wall) AROUND ITS OWN CLICK - even when the
                // wall data between the clicks is interrupted (door opening). No feature within reach: the click itself.
                (double pos, EndType type) Resolve((double pos, EndType type) grown, double click, int dir)
                {
                    if (Math.Abs(grown.pos - click) <= clip.Snap) return grown;
                    int b0c = (int)Math.Floor((click - amin) / bin), sb = -1;
                    for (int k = 0; k <= snapBins && sb < 0; k++)
                    {
                        int l = b0c - k, r = b0c + k;
                        if (l >= 0 && l < nBins && faceCnt[l] >= faceMin) sb = l;
                        else if (r >= 0 && r < nBins && faceCnt[r] >= faceMin) sb = r;
                    }
                    if (sb >= 0)
                    {
                        var e2 = Grow(sb, dir);
                        if (Math.Abs(e2.pos - click) <= clip.Snap) return e2;
                    }
                    return (click, EndType.Click);
                }
                lowEnd = Resolve(lowEnd, clickLo, -1);
                highEnd = Resolve(highEnd, clickHi, +1);
            }
            double a0 = lowEnd.pos, a1 = highEnd.pos;
            if (a1 - a0 < 0.05) return Fail("Fitted wall is too short.");

            // ---- 5. world coordinates, open-end test -----------------------------------------------
            double cMid = 0.5 * (cA + cB);
            P2 ToWorld(double a, double c)
            {
                double u = C * a - S * c;
                double s = S * a + C * c;
                return O + T * u + N * s;
            }
            P2 dir = (ToWorld(1, 0) - ToWorld(0, 0)).Normalized();
            P2 nrm = (ToWorld(0, 1) - ToWorld(0, 0)).Normalized() * sign;
            P2 pStart = ToWorld(a0, cMid), pEnd = ToWorld(a1, cMid);

            double margin = Math.Max(2 * o.MaxGap, 0.3);
            double limit = RegionLimit(rAxis, rHalfLen, rHalfWid, dir);
            EndType tLow = lowEnd.type, tHigh = highEnd.type;
            if (tLow == EndType.Gap && Math.Abs((pStart - rCenter).Dot(dir)) >= limit - margin) tLow = EndType.Open;
            if (tHigh == EndType.Gap && Math.Abs((pEnd - rCenter).Dot(dir)) >= limit - margin) tHigh = EndType.Open;
            if (o.JunctionToCenterline)
            {
                // the far face of the crossing wall may lie beyond the loaded region: load more before deciding
                double jm = margin + o.MaxCrossThickness;
                if (tLow == EndType.Junction && Math.Abs((pStart - rCenter).Dot(dir)) >= limit - jm) tLow = EndType.Open;
                if (tHigh == EndType.Junction && Math.Abs((pEnd - rCenter).Dot(dir)) >= limit - jm) tHigh = EndType.Open;
            }

            // ---- 6. quality numbers ------------------------------------------------------------------
            int b0 = Math.Max(0, (int)Math.Floor((a0 - amin) / bin)), b1 = Math.Min(nBins - 1, (int)Math.Floor((a1 - amin) / bin));
            int supA = 0, supB = 0, tot = 0;
            for (int k = b0; k <= b1; k++) { tot++; if (faceA[k] >= 2) supA++; if (faceB[k] >= 2) supB++; }
            double coverage = tot == 0 ? 0 : (oneFace ? supA : Math.Min(supA, supB)) / (double)tot;
            double sigma = 0.5 * (spreadA + spreadB);
            double angSigma = (a1 - a0) > 0.1 ? Math.Atan(sigma / ((a1 - a0) * Math.Sqrt(nFace / 12.0))) * 180 / Math.PI : 0;
            double worst = Math.Max(spreadA, spreadB);
            string grade = (worst <= 0.005 && coverage >= 0.60) ? "Good" : (worst <= 0.010 && coverage >= 0.35) ? "OK" : "Check";
            if (oneFace && grade == "Good") grade = "OK";                // the thickness is an assumption, never "Good"

            return new WallFitResult
            {
                Success = true,
                Start = pStart, End = pEnd,
                Thickness = thickness, Direction = dir, Normal = nrm,
                SpreadA = spreadA, SpreadB = spreadB,
                Coverage = coverage, InliersA = faceAlongsA.Count, InliersB = faceAlongsB.Count,
                AngleSigmaDeg = angSigma, AtStart = tLow, AtEnd = tHigh, PointsUsed = pts.Count, Grade = grade, OneFace = oneFace,
            };
        }

        private static double RegionLimit(P2 axis, double halfLen, double halfWid, P2 dir)
        {
            double ex = Math.Abs(dir.Dot(axis)), ey = Math.Abs(dir.Dot(axis.Perp));
            double t1 = ex > 1e-9 ? halfLen / ex : double.MaxValue;
            double t2 = ey > 1e-9 ? halfWid / ey : double.MaxValue;
            return Math.Min(t1, t2);
        }

        private static void Collect(List<L> pts, double ct, double st, double cA, double cB, double tol, List<int> iA, List<int> iB, bool oneFace = false)
        {
            iA.Clear(); iB.Clear();
            for (int i = 0; i < pts.Count; i++)
            {
                double across = -st * pts[i].U + ct * pts[i].S;
                double dA = Math.Abs(across - cA), dB = Math.Abs(across - cB);
                if (dA < tol && (oneFace || dA <= dB)) iA.Add(i);
                else if (!oneFace && dB < tol) iB.Add(i);
            }
        }

        // =====================================================================================
        //  Fine alignment: rotate until BOTH surfaces are as sharp as possible (histogram peak height),
        //  then put each face at the peak of its density instead of the mean/median of a band.
        //  Coarse-to-fine so a 40 m wall is as cheap as a 3 m one.
        // =====================================================================================
        private static void PeakRefine(List<L> pts, ref double theta, ref double cA, ref double cB, WallFitOptions o,
                                       bool oneFace = false, double vt = 0)
        {
            double T = Math.Abs(cB - cA);
            double W = oneFace ? Math.Max(3 * o.InlierTol, 0.03) : Math.Min(Math.Max(3 * o.InlierTol, 0.03), 0.4 * T);
            double ct0 = Math.Cos(theta), st0 = Math.Sin(theta);
            var gA = new List<L>(); var gB = new List<L>();
            foreach (var p in pts)
            {
                double a = -st0 * p.U + ct0 * p.S;
                if (Math.Abs(a - cA) < W) gA.Add(p); else if (!oneFace && Math.Abs(a - cB) < W) gB.Add(p);
            }
            if (gA.Count < o.MinInliersPerSide || (!oneFace && gB.Count < o.MinInliersPerSide)) return;
            gA = Stride(gA, 40000); gB = Stride(gB, 40000);

            double maxU = 0.5;
            foreach (var p in gA) maxU = Math.Max(maxU, Math.Abs(p.U));
            foreach (var p in gB) maxU = Math.Max(maxU, Math.Abs(p.U));

            double baseBin = Math.Max(0.0005, o.InlierTol / 4);
            double best = theta;
            double range = 1.5 * Math.PI / 180;
            foreach (double bin in new[] { baseBin * 4, baseBin, baseBin / 2 })
            {
                double step = Math.Max(bin / (2 * maxU), 2e-6);
                double bestScore = double.MinValue, cand = best;
                for (double d = -range; d <= range + 1e-12; d += step)
                {
                    double th = best + d;
                    double sc = Sharp(gA, th, bin) + Sharp(gB, th, bin);
                    if (sc > bestScore) { bestScore = sc; cand = th; }
                }
                best = cand;
                range = 3 * step;
            }
            theta = best;
            cA = PeakCenter(gA, theta, baseBin / 2, o.InlierTol);
            cB = oneFace ? cA + vt : PeakCenter(gB, theta, baseBin / 2, o.InlierTol);
        }

        private static List<L> Stride(List<L> g, int max)
        {
            if (g.Count <= max) return g;
            int step = (int)Math.Ceiling(g.Count / (double)max);
            var r = new List<L>(g.Count / step + 1);
            for (int i = 0; i < g.Count; i += step) r.Add(g[i]);
            return r;
        }

        private static double Sharp(List<L> g, double th, double bin)
        {
            if (g.Count == 0) return 0;
            double c = Math.Cos(th), s = Math.Sin(th);
            var v = new double[g.Count];
            double mn = double.MaxValue, mx = double.MinValue;
            for (int i = 0; i < v.Length; i++)
            {
                v[i] = -s * g[i].U + c * g[i].S;
                if (v[i] < mn) mn = v[i]; if (v[i] > mx) mx = v[i];
            }
            // Each point is shared between its two nearest bins. Counting whole points per bin locks onto the scan file's own grid:
            // many scans store coordinates rounded to 1 mm, those rows fall unevenly into bins of a similar size, and that unevenness
            // scores as "sharp" at exactly 0 / 90 degrees - a wall really lying 0.3 degrees off the axis was pulled onto the axis.
            int nb = (int)((mx - mn) / bin) + 2;
            var h = new double[nb];
            for (int i = 0; i < v.Length; i++)
            {
                double x = (v[i] - mn) / bin;
                int k = (int)x;
                double f = x - k;
                h[k] += 1 - f; h[k + 1] += f;
            }
            double sum = 0;
            for (int k = 0; k < nb; k++) sum += h[k] * h[k];
            return sum;
        }

        private static double PeakCenter(List<L> g, double th, double bin, double tol)
        {
            double c = Math.Cos(th), s = Math.Sin(th);
            var v = g.Select(p => -s * p.U + c * p.S).ToList();
            double mn = v.Min(), mx = v.Max();
            int nb = (int)((mx - mn) / bin) + 1;
            var h = new int[nb];
            foreach (var x in v) h[(int)((x - mn) / bin)]++;
            int bk = 0, bv = -1;
            for (int k = 0; k < nb; k++)
            {
                int sm = h[k] + (k > 0 ? h[k - 1] : 0) + (k < nb - 1 ? h[k + 1] : 0);
                if (sm > bv) { bv = sm; bk = k; }
            }
            double c0 = mn + (bk + 0.5) * bin;
            var near = v.Where(x => Math.Abs(x - c0) <= Math.Max(2 * bin, 0.5 * tol)).ToList();
            return near.Count >= 5 ? Stats.Median(near) : Stats.Median(v);
        }

        // =====================================================================================
        //  Joint search for the two faces from the clicks (unchanged idea: RANSAC on face A, closed-form parallel face B)
        // =====================================================================================
        private static bool FindFacePair(List<L> pts, double sA, double sB, double band, WallFitOptions o, Random rng,
                                         out double theta, out double cA, out double cB)
        {
            theta = cA = cB = 0;
            var nearA = new List<int>();
            for (int i = 0; i < pts.Count; i++)
                if (Math.Sqrt(pts[i].U * pts[i].U + (pts[i].S - sA) * (pts[i].S - sA)) <= band + 0.25) nearA.Add(i);
            if (nearA.Count < 2) return false;

            double maxClick = o.MaxClickAngleDeg * Math.PI / 180;
            double tol = o.InlierTol;
            int bestScore = 0;
            var buf = new List<double>();

            for (int it = 0; it < o.RansacIterations; it++)
            {
                var p = pts[nearA[rng.Next(nearA.Count)]];
                var q = pts[rng.Next(pts.Count)];
                double du = q.U - p.U, ds = q.S - p.S, len = Math.Sqrt(du * du + ds * ds);
                if (len < 0.10) continue;
                if (du < 0 || (du == 0 && ds < 0)) { du = -du; ds = -ds; }
                double th = Math.Atan2(ds, du);
                if (Math.Abs(th) > maxClick) continue;
                double ct = Math.Cos(th), st = Math.Sin(th);

                double aAcross = -st * p.U + ct * p.S;
                if (Math.Abs(aAcross - ct * sA) > band) continue;

                int cntA = 0;
                buf.Clear();
                double bClick = ct * sB;
                for (int i = 0; i < pts.Count; i++)
                {
                    double c = -st * pts[i].U + ct * pts[i].S;
                    if (Math.Abs(c - aAcross) < tol) cntA++;
                    else if (Math.Abs(c - bClick) <= band && Math.Abs(c - aAcross) >= o.MinThickness) buf.Add(c);
                }
                if (cntA < o.MinInliersPerSide || buf.Count < o.MinInliersPerSide) continue;

                buf.Sort();
                int bestCnt = 0, bestStart = 0, j = 0;
                for (int bi = 0; bi < buf.Count; bi++)
                {
                    while (buf[bi] - buf[j] > 2 * tol) j++;
                    if (bi - j + 1 > bestCnt) { bestCnt = bi - j + 1; bestStart = j; }
                }
                if (bestCnt < o.MinInliersPerSide) continue;
                int score = cntA + bestCnt;
                if (score > bestScore)
                {
                    bestScore = score; theta = th; cA = aAcross;
                    cB = buf[bestStart + bestCnt / 2];
                }
            }
            return bestScore > 0;
        }

        // ---- snapping helpers (used by the Revit layer) ------------------------------------------
        public static double? SnapThickness(double measured, IEnumerable<double> candidates, double tol)
        {
            double? best = null; double bd = tol;
            foreach (var c in candidates) { var dd = Math.Abs(c - measured); if (dd <= bd) { bd = dd; best = c; } }
            return best;
        }

        public static double SnapAngle(double angle, IEnumerable<double> gridAngles, double tol)
        {
            double bestAng = angle, bd = tol;
            foreach (var g in gridAngles)
            {
                double diff = Math.IEEERemainder(angle - g, Math.PI);
                if (Math.Abs(diff) <= bd) { bd = Math.Abs(diff); bestAng = angle - diff; }
            }
            return bestAng;
        }
    }
}
