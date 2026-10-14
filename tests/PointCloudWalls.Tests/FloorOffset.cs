using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

internal static class FloorOffsetTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> WallWithDoor(double wallLen, double wallTop, double a0, double a1, double doorH, double density, int clutter, double floorOffset)
    {
        // floorOffset: the TRUE scanned floor is at Y=floorOffset in this frame (not Y=0) - simulating a Revit wall
        // element whose modeled base sits a bit below (or above) where the real scan shows the wall actually starting
        var pts = new List<P2>();
        int nTop = (int)(wallLen * (wallTop - doorH) * density);
        for (int i = 0; i < nTop; i++) pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(0.005), doorH + rng.NextDouble() * (wallTop - doorH) + Gauss(0.005)));
        int nLeft = (int)(a0 * (doorH - floorOffset) * density);
        for (int i = 0; i < nLeft; i++) pts.Add(new P2(rng.NextDouble() * a0 + Gauss(0.005), floorOffset + rng.NextDouble() * (doorH - floorOffset) + Gauss(0.005)));
        int nRight = (int)((wallLen - a1) * (doorH - floorOffset) * density);
        for (int i = 0; i < nRight; i++) pts.Add(new P2(a1 + rng.NextDouble() * (wallLen - a1) + Gauss(0.005), floorOffset + rng.NextDouble() * (doorH - floorOffset) + Gauss(0.005)));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * wallLen, rng.NextDouble() * wallTop));   // clutter doesn't know where the true floor is - can land anywhere, including below it
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== EstimateFloorOffset recovers the TRUE scanned floor, not 0, across a range of offsets");
        {
            int ok = 0, trials = 150; var errs = new List<double>();
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(90000 + t);
                double trueOffset = (rng.NextDouble() - 0.5) * 0.3;   // wall base modeled up to +/-15cm off the real scanned floor
                double wallLen = 10.0, wallTop = 2.8, w = 0.9, doorH = 2.05, a0 = 4.0, a1 = a0 + w;
                var pts = WallWithDoor(wallLen, wallTop, a0, a1, doorH, 500, 150, trueOffset);
                double est = OpeningFitter.EstimateFloorOffset(pts);
                errs.Add(est - trueOffset);
                if (Math.Abs(est - trueOffset) < 0.02) ok++;
            }
            Check(ok >= 140, $"{ok}/{trials} recovered the true floor within 20mm");
            Console.WriteLine($"   error rms {Math.Sqrt(errs.Select(x => x * x).Average()) * 1000:0.0}mm");
        }

        Console.WriteLine("\n=== Your exact scenario: wall modeled ~100mm BELOW the true scanned floor - door must NOT become a window");
        {
            int correctDoor = 0, wrongWindow = 0, trials = 150;
            for (int t = 0; t < trials; t++)
            {
                rng = new Random(91000 + t);
                double trueOffset = 0.16;   // the true floor is 160mm ABOVE the modeled wall base (Y=0) - clearly past the 100mm default tolerance
                double wallLen = 10.0, wallTop = 2.8, w = 0.9, doorH = 2.05, a0 = 4.0, a1 = a0 + w;
                var pts = WallWithDoor(wallLen, wallTop, a0, a1, doorH, 500, 150, trueOffset);
                var click = new P2((a0 + a1) / 2, doorH * 0.5);

                // WITHOUT the fix: classify against wallBase=0 (the Revit wall's own base), ignoring the real scan
                var rBad = OpeningFitter.Fit(pts, click, 0, wallTop);
                // WITH the fix: classify against the scan's own estimated floor
                double floorOffset = OpeningFitter.EstimateFloorOffset(pts);
                var rGood = OpeningFitter.Fit(pts, click, floorOffset, wallTop);

                if (rGood.Success && rGood.Kind == OpeningKind.Door) correctDoor++;
                // without the fix, the wrong floor reference either misclassifies the door as a window, or (depending on
                // exactly how the mismatch interacts with the data) makes the flood fill spill past the search area
                // entirely - both are real failures caused by the same root cause: trusting the wrong floor reference
                if (!rBad.Success || rBad.Kind == OpeningKind.Window) wrongWindow++;
            }
            Check(wrongWindow >= 100, $"CONFIRMS the bug: without the fix, {wrongWindow}/{trials} genuine doors were misread as windows");
            Check(correctDoor >= 140, $"WITH the fix: {correctDoor}/{trials} correctly classified as doors");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
