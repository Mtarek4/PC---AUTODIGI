using System;
using System.Collections.Generic;
using PointCloudWalls.Core;

internal static class HeightSelectTests
{
    private static int _fail;
    private static void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what); if (!ok) _fail++; }

    public static int Run()
    {
        Console.WriteLine("=== Auto slice height: reject an isolated density spike (a beam/bracket), prefer the column's own sustained band");
        {
            // step 0.10m, range -0.6..0.6 -> 13 candidates, index 6 = the original height
            var counts = new int[13];
            counts[6] = 15;                                         // original: sparse
            counts[1] = 90; counts[2] = 95; counts[3] = 88;          // the real column surface: sustained band
            counts[10] = 500;                                       // a bracket/beam: huge but ISOLATED
            int best = HeightSelector.SelectSustained(counts, 6);
            Check(best != 10, "does not jump to the isolated 500-point spike");
            Check(best == 1 || best == 2 || best == 3, "picks a height inside the real, sustained band (got " + best + ")");
        }
        Console.WriteLine("=== A genuinely better sustained band nearby is still taken");
        {
            var counts = new int[13];
            counts[6] = 12;
            counts[8] = 80; counts[9] = 85; counts[10] = 82;
            Check(HeightSelector.SelectSustained(counts, 6) == 9, "moves to the sustained band");
        }
        Console.WriteLine("=== Marginal noise must not cause a switch");
        {
            var counts = new int[13];
            for (int i = 0; i < 13; i++) counts[i] = 60;
            counts[6] = 58; counts[7] = 63;
            Check(HeightSelector.SelectSustained(counts, 6) == 6, "stays at the original height");
        }
        Console.WriteLine("=== No sustained band anywhere (only a lone spike at the search edge): must not pick a phantom");
        {
            var counts = new int[13];
            counts[6] = 4;
            counts[0] = 200;
            Check(HeightSelector.SelectSustained(counts, 6) == 6, "stays put rather than grabbing the edge spike");
        }
        Console.WriteLine("=== A 2-wide feature (not a genuine 3-wide sustained band) is still rejected");
        {
            var counts = new int[13];
            counts[6] = 15;
            counts[1] = 90; counts[2] = 95; counts[3] = 88;
            counts[9] = 300; counts[10] = 280;
            int best = HeightSelector.SelectSustained(counts, 6);
            Check(best != 9 && best != 10, "the 2-wide spike is rejected (got " + best + ")");
        }
        Console.WriteLine(_fail == 0 ? "\nALL PASSED" : "\n" + _fail + " FAILED");
        return _fail;
    }
}
