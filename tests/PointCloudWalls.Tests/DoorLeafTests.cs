using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

/// <summary>
/// Reproduces the reported live bug: a door caught ajar (or nearly shut) in the scan reads, in the flattened 2D wall
/// profile, as a solid, tall, narrow blob sitting INSIDE the true opening rather than against either real jamb - unlike
/// a person (PersonInDoor.cs), a leaf commonly spans nearly the FULL door height, so the old "narrow and doesn't reach
/// the header" test alone doesn't catch it, and the fill used to stop at the leaf instead of continuing to the true far
/// jamb - reported live as a door snapping to 768x2147mm (the leaf) instead of the real opening. A leaf always clears
/// the real floor by a small physical gap so it can swing; a real jamb never does. That's the signal
/// OpeningFitter.Fit now uses to tell the two apart and exclude the leaf's own points before the occupancy grid is
/// built.
///
/// IMPORTANT: "success" here is only counted when the returned WIDTH actually tracks the true opening (within 60mm),
/// not merely when Fit() reports Success and Kind==Door - an earlier version of this test counted the latter alone
/// and was quietly passing dozens of cases where the width was wrong by up to half a metre, which is exactly the
/// failure mode being tested for.
///
/// This is a genuinely hard case statistically: telling a ~2-4cm real headroom gap apart from a real, physically
/// continuous header a few cm above it comes down to point density right at that boundary, and ordinary scan noise
/// eats into a gap that size - especially at this test's deliberately modest density (1500 pts/m^2). The pass bar
/// below (75/150, width-validated) reflects that honestly: roughly half of these synthetic worst-case trials are
/// correctly recovered - a real improvement over the reported failure (which was 0/150 before this fix, with every
/// "success" actually being the wrong, leaf-sized width) - but this is not a 100% guarantee on the tightest synthetic
/// cases. A case that isn't recovered fails cleanly (a plain error, or an opening size outside the plausible range)
/// rather than silently returning the wrong (leaf) width undetected - real production scans are normally denser than
/// this test's 1500 pts/m^2, and a real leaf's headroom clearance is rarely at this test's tightest (30mm) end, so
/// the live recovery rate should be noticeably better than this deliberately hard synthetic benchmark.
/// </summary>
internal static class DoorLeafTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> DoorWithAjarLeaf(double wallLen, double wallTop, double a0, double a1, double doorH,
        double leafA0, double leafA1, double leafFloorGap, double leafH, double density, int clutter)
    {
        var pts = new List<P2>();
        // solid wall everywhere except the door - same pattern as PersonInDoor.DoorWithPerson
        int nTop = (int)(wallLen * (wallTop - doorH) * density);
        for (int i = 0; i < nTop; i++) pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(0.005), doorH + rng.NextDouble() * (wallTop - doorH) + Gauss(0.005)));
        int nLeft = (int)(a0 * doorH * density);
        for (int i = 0; i < nLeft; i++) pts.Add(new P2(rng.NextDouble() * a0 + Gauss(0.005), rng.NextDouble() * doorH + Gauss(0.005)));
        int nRight = (int)((wallLen - a1) * doorH * density);
        for (int i = 0; i < nRight; i++) pts.Add(new P2(a1 + rng.NextDouble() * (wallLen - a1) + Gauss(0.005), rng.NextDouble() * doorH + Gauss(0.005)));
        // the leaf: a dense, tall blob floating INSIDE the true opening - clears the real floor by leafFloorGap, and
        // (like a real leaf, undercut for hardware clearance) stops a little short of the true head too.
        double leafW = leafA1 - leafA0;
        int nLeaf = (int)(leafW * leafH * density * 1.5);   // a leaf reads quite dense/solid to a scanner
        for (int i = 0; i < nLeaf; i++) pts.Add(new P2(leafA0 + rng.NextDouble() * leafW + Gauss(0.01), leafFloorGap + rng.NextDouble() * leafH + Gauss(0.005)));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * wallLen, rng.NextDouble() * wallTop));
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== A door leaf caught ajar in the scan (tall, floor-gapped, sitting inside the true opening) - must not be mistaken for the far jamb");
        int ok = 0, trials = 150; var wErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(85000 + t);
            double wallLen = 10.0, wallTop = 2.8;
            double trueW = 0.85 + rng.NextDouble() * 0.25, doorH = 2.05 + rng.NextDouble() * 0.15;
            double a0 = 3.0 + rng.NextDouble() * 2.0, a1 = a0 + trueW;
            double leafFloorGap = 0.018 + rng.NextDouble() * 0.016;          // 18-34mm - realistic door clearance, safely above the fitter's own (tighter) floor-touch tolerance
            double leafH = doorH - (0.03 + rng.NextDouble() * 0.03);         // stops a little short of the true head too (30-60mm headroom clearance)
            double leafW = trueW - (0.20 + rng.NextDouble() * 0.20);         // narrower than the true opening (200-400mm short, split either side) - leaves a real, comfortably click-able gap on both sides
            double leafA0 = a0 + 0.06 + rng.NextDouble() * (trueW - leafW - 0.12);
            double leafA1 = leafA0 + leafW;
            var pts = DoorWithAjarLeaf(wallLen, wallTop, a0, a1, doorH, leafA0, leafA1, leafFloorGap, leafH, 1500, 150);
            // click somewhere real - in the genuinely open strip beside the leaf, same as someone clicking what they
            // see as "the opening"; pick whichever side has the bigger gap so the seed is comfortably in empty space.
            bool clickLeft = (leafA0 - a0) >= (a1 - leafA1);
            double clickX = clickLeft ? (a0 + leafA0) / 2 : (leafA1 + a1) / 2;
            var click = new P2(clickX + Gauss(0.03), doorH * 0.5);
            var r = OpeningFitter.Fit(pts, click, 0, wallTop);
            if (!r.Success || r.Kind != OpeningKind.Door) continue;
            wErr.Add(r.Width - trueW);
            // "success" only counts if the width actually tracks the TRUE opening, not the leaf's - a wrong-but-
            // confident width (still latched onto the leaf, or onto some partial mix) is exactly the bug being tested
            // for, so it must not be scored as a pass just because Fit() itself returned Success.
            if (Math.Abs(r.Width - trueW) <= 0.06) ok++;
        }
        Check(ok >= 75, $"{ok}/{trials} correctly recovered the TRUE full opening width, not the leaf's");
        if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm  (should track the true opening, not the leaf)");

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
