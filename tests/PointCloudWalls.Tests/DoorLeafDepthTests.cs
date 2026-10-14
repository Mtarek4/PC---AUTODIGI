using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

/// <summary>
/// Reproduces the REAL live bug, confirmed against two actual scans this session (not the ajar-leaf-with-floor-gap
/// model DoorLeafTests.cs was built around, which turned out not to match reality): a door leaf swung open to a real
/// angle reads, in the flattened (along-wall, height) profile, exactly like solid wall - it touches the floor and
/// spans nearly the full door height, so neither the "doesn't reach the header" nor the "clears the floor" tests
/// catch it. What actually tells it apart from a real jamb is DEPTH: a rigid flat leaf hinged at one jamb sweeps
/// from depth=0 at the hinge out to depth=leafWidth*sin(theta) at its free edge as you move away from the hinge -
/// real wall material never leaves the wall's own face by more than its thickness. That's the signal
/// OpeningFitter.Fit now uses (MaxWallDepth + a per-point `depths` argument) to exclude the leaf's own points before
/// the occupancy grid is built, instead of the floor-gap test (which a wide-open leaf never trips).
///
/// IMPORTANT: this is a real, large improvement, not a perfect fix, and the pass bar (115/150, within 150mm of the
/// true width) is honest about that. Right at the hinge, a leaf stays close to the wall plane for a genuine physical
/// stretch before its depth ramps up past any threshold - worse at shallow opening angles, where the ramp is slower
/// - so some hinge-side shortfall (typically ~50-150mm, confirmed against two real scans this session) is an
/// inherent geometric limit of a depth-only test, not a bug to keep chasing. What matters is that this turns a
/// ~600-900mm error (the leaf's own width, when the leaf is mistaken for the far jamb outright) into one of order
/// 100mm.
/// </summary>
internal static class DoorLeafDepthTests
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== A door leaf swung open to a real angle (touches the floor, spans full height, reads exactly like wall by density) - only depth off the wall face tells it apart from the real far jamb");
        int ok = 0, trials = 150; var wErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(91000 + t);
            double wallLen = 10.0, wallTop = 2.8;
            double trueW = 0.75 + rng.NextDouble() * 0.3, doorH = 2.05 + rng.NextDouble() * 0.15;
            double a0 = 3.0 + rng.NextDouble() * 2.0, a1 = a0 + trueW;
            bool hingeLeft = rng.NextDouble() < 0.5;
            double hinge = hingeLeft ? a0 : a1;
            double leafW = trueW - (0.02 + rng.NextDouble() * 0.06);         // a real door is a little narrower than its rough opening
            double thetaDeg = 25 + rng.NextDouble() * 65;                    // 25-90 degrees open - genuinely wide open, not just ajar
            double theta = thetaDeg * Math.PI / 180.0;
            double density = 1500;

            var pts = new List<P2>(); var deps = new List<double>();
            void Wall(double xlo, double xhi, double ylo, double yhi, int n) { for (int i = 0; i < n; i++) { pts.Add(new P2(xlo + rng.NextDouble() * (xhi - xlo) + Gauss(0.005), ylo + rng.NextDouble() * (yhi - ylo) + Gauss(0.005))); deps.Add(Gauss(0.01)); } }
            Wall(0, a0, 0, wallTop, (int)(a0 * wallTop * density));
            Wall(a1, wallLen, 0, wallTop, (int)((wallLen - a1) * wallTop * density));
            Wall(0, wallLen, doorH, wallTop, (int)(wallLen * (wallTop - doorH) * density));   // header, same density as the rest of the wall

            // the leaf: full floor-to-(near)head height, but at increasing DEPTH as you move from the hinge
            int nLeaf = (int)(leafW * doorH * density * 1.3);
            double dir = hingeLeft ? 1.0 : -1.0;
            for (int i = 0; i < nLeaf; i++)
            {
                double f = rng.NextDouble();                      // 0 = at the hinge, 1 = free edge
                double xx = hinge + dir * f * leafW * Math.Cos(theta) + Gauss(0.01);
                double yy = rng.NextDouble() * (doorH - 0.02) + Gauss(0.005);
                double dd = f * leafW * Math.Sin(theta) + Gauss(0.01);
                pts.Add(new P2(xx, yy)); deps.Add(dd);
            }
            for (int i = 0; i < 150; i++) { pts.Add(new P2(rng.NextDouble() * wallLen, rng.NextDouble() * wallTop)); deps.Add(Gauss(0.01)); }

            // click in the genuinely open strip on the side AWAY from the hinge (the side a person would actually
            // see as "the opening" - same spirit as the real diagnostic clicks this session)
            double clickX = hingeLeft ? (hinge + leafW * Math.Cos(theta) * dir + a1) / 2 : (a0 + hinge + leafW * Math.Cos(theta) * dir) / 2;
            var click = new P2(clickX + Gauss(0.03), doorH * 0.5);

            var o = new OpeningFitOptions { MaxWallDepth = 0.10 };
            var r = OpeningFitter.Fit(pts, click, 0, wallTop, o, deps);
            if (!r.Success || r.Kind != OpeningKind.Door) continue;
            wErr.Add(r.Width - trueW);
            // 150mm, not the usual 60mm: a depth filter cannot fully recover the last stretch right at the hinge-side
            // jamb, whatever the threshold - a rigid leaf hinged at any real angle stays close to the wall plane for
            // a genuine physical distance before its depth ramps up past ANY tolerance (worse at shallow opening
            // angles, where the ramp is slower), so some hinge-side shortfall is an inherent geometric limit of this
            // approach, not a bug. What matters, and what this checks, is that it is now a shortfall of order 100mm,
            // not the 600-900mm the leaf's own full width used to cost before depth was available at all.
            if (Math.Abs(r.Width - trueW) <= 0.15) ok++;
        }
        Check(ok >= 115, $"{ok}/{trials} recovered within 150mm of the TRUE full opening width, not the leaf's (depth-based exclusion)");
        if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm");

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
