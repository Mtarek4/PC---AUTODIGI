using System;
using System.Collections.Generic;
using System.Linq;
using PointCloudWalls.Core;

internal static class PersonInDoor
{
    static Random rng;
    static double Gauss(double s) { double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

    static List<P2> DoorWithPerson(double wallLen, double wallTop, double a0, double a1, double doorH, double personA0, double personA1, double personH, double density, int clutter)
    {
        var pts = new List<P2>();
        // solid wall everywhere except the door
        int nTop = (int)(wallLen * (wallTop - doorH) * density);
        for (int i = 0; i < nTop; i++) pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(0.005), doorH + rng.NextDouble() * (wallTop - doorH) + Gauss(0.005)));
        int nLeft = (int)(a0 * doorH * density);
        for (int i = 0; i < nLeft; i++) pts.Add(new P2(rng.NextDouble() * a0 + Gauss(0.005), rng.NextDouble() * doorH + Gauss(0.005)));
        int nRight = (int)((wallLen - a1) * doorH * density);
        for (int i = 0; i < nRight; i++) pts.Add(new P2(a1 + rng.NextDouble() * (wallLen - a1) + Gauss(0.005), rng.NextDouble() * doorH + Gauss(0.005)));
        // the person: a dense vertical blob INSIDE the door, narrower than the door, NOT reaching the header
        int nPerson = (int)((personA1 - personA0) * personH * density * 1.5);   // a person is quite dense/solid-looking to a scanner
        for (int i = 0; i < nPerson; i++) pts.Add(new P2(personA0 + rng.NextDouble() * (personA1 - personA0) + Gauss(0.01), rng.NextDouble() * personH + Gauss(0.01)));
        for (int i = 0; i < clutter; i++) pts.Add(new P2(rng.NextDouble() * wallLen, rng.NextDouble() * wallTop));
        return pts;
    }

    static int _fail;
    static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== A person standing in the doorway (matches the reported screenshot) - must not fragment the opening");
        int ok = 0, trials = 150; var wErr = new List<double>();
        for (int t = 0; t < trials; t++)
        {
            rng = new Random(80000 + t);
            double wallLen = 10.0, wallTop = 2.8;
            double w = 0.8 + rng.NextDouble() * 0.4, doorH = 2.0 + rng.NextDouble() * 0.2;
            double a0 = 3.0 + rng.NextDouble() * 2.0, a1 = a0 + w;
            double personH = 1.5 + rng.NextDouble() * 0.3;                          // shorter than the door - realistic
            double personW = 0.35 + rng.NextDouble() * 0.25;                         // a person's shoulder width, roughly
            double personA0 = a0 + rng.NextDouble() * (w - personW - 0.1) + 0.05;    // somewhere inside the doorway
            double personA1 = personA0 + personW;
            var pts = DoorWithPerson(wallLen, wallTop, a0, a1, doorH, personA0, personA1, personH, 500, 150);
            var click = new P2((a0 + a1) / 2 + Gauss(0.1), doorH * 0.6);            // clicking roughly in the doorway, near/above the person's head height
            var r = OpeningFitter.Fit(pts, click, 0, wallTop);
            if (!r.Success || r.Kind != OpeningKind.Door) continue;
            ok++;
            wErr.Add(r.Width - w);
        }
        Check(ok >= 130, $"{ok}/{trials} still correctly found the FULL door width despite the person");
        if (wErr.Count > 0) Console.WriteLine($"   width err rms {Math.Sqrt(wErr.Select(x => x * x).Average()) * 1000:0.0}mm");

        Console.WriteLine("\n=== Regression: a REAL solid pier between a door and a window must still NOT be bridged over");
        {
            int ok2 = 0, trials2 = 100;
            for (int t = 0; t < trials2; t++)
            {
                rng = new Random(81000 + t);
                double wallLen = 8.0, wallTop = 2.8;
                var pts = new List<P2>();
                int n = (int)(wallLen * wallTop * 700);
                for (int i = 0; i < n; i++) pts.Add(new P2(rng.NextDouble() * wallLen + Gauss(0.005), rng.NextDouble() * wallTop + Gauss(0.005)));
                double dA0 = 1.0, dA1 = 1.9, wA0 = 2.6, wA1 = 3.6, wH0 = 0.9, wH1 = 1.9;   // a 70cm solid pier between them (2.6-1.9)
                pts = pts.Where(p => !(p.X > dA0 && p.X < dA1 && p.Y < 2.05) && !(p.X > wA0 && p.X < wA1 && p.Y > wH0 && p.Y < wH1)).ToList();
                var clickDoor = new P2((dA0 + dA1) / 2 + Gauss(0.05), 1.0 + Gauss(0.1));
                var r = OpeningFitter.Fit(pts, clickDoor, 0, wallTop, new OpeningFitOptions { SearchHalfWidth = 1.8, BridgeMaxWidth = 0.75 });
                if (!r.Success || r.Kind != OpeningKind.Door) continue;
                ok2++;
                double gotW = r.Width;
                // a bridged pier would merge door+window into something ~1600mm+ wide - ordinary fitting noise stays well under 200mm
                if (Math.Abs(gotW - (dA1 - dA0)) > 0.20) { ok2--; }
            }
            Check(ok2 >= 90, $"{ok2}/{trials2} still correctly kept the door and window separate (pier not bridged)");
        }

        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
