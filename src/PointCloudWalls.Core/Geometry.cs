using System;
using System.Collections.Generic;

namespace PointCloudWalls.Core
{
    /// <summary>2D point/vector. ALL Core math is in METERS. Convert from Revit feet at the adapter boundary.</summary>
    public readonly struct P2
    {
        public readonly double X, Y;
        public P2(double x, double y) { X = x; Y = y; }

        public static P2 operator +(P2 a, P2 b) => new P2(a.X + b.X, a.Y + b.Y);
        public static P2 operator -(P2 a, P2 b) => new P2(a.X - b.X, a.Y - b.Y);
        public static P2 operator *(P2 a, double k) => new P2(a.X * k, a.Y * k);

        public double Dot(P2 o) => X * o.X + Y * o.Y;
        public double Length => Math.Sqrt(X * X + Y * Y);
        public P2 Perp => new P2(-Y, X);                       // +90 deg
        public P2 Normalized() { var l = Length; return l < 1e-12 ? new P2(0, 0) : new P2(X / l, Y / l); }
        public override string ToString() => $"({X:F4}, {Y:F4})";
    }

    internal static class Stats
    {
        public static double Median(List<double> v)
        {
            if (v.Count == 0) return double.NaN;
            var a = v.ToArray();
            Array.Sort(a);
            int m = a.Length / 2;
            return a.Length % 2 == 1 ? a[m] : 0.5 * (a[m - 1] + a[m]);
        }
    }
}

namespace PointCloudWalls.Core
{
    internal static class Stats2
    {
        /// <summary>Robust spread (sigma-equivalent) from the median absolute deviation.</summary>
        public static double MadSigma(List<double> v)
        {
            if (v.Count < 3) return 0;
            double med = Stats.Median(v);
            var dev = new List<double>(v.Count);
            foreach (var x in v) dev.Add(Math.Abs(x - med));
            return 1.4826 * Stats.Median(dev);
        }
    }
}
