using System;
using System.Collections.Generic;

namespace PointCloudWalls.Core
{
    /// <summary>
    /// Where the solver gets its 2D points from. The Revit layer implements this on top of the point cloud API;
    /// tests (and later a LAZ reader) implement it on top of a plain list. World coordinates, meters.
    /// </summary>
    public interface IWallPointSource
    {
        /// <summary>Points inside an oriented rectangle: centre, unit axis, half length along the axis, half width across it.</summary>
        List<P2> Query(P2 center, P2 axis, double halfLength, double halfWidth);
    }

    public sealed class ListPointSource : IWallPointSource
    {
        private readonly IReadOnlyList<P2> _pts;
        public ListPointSource(IReadOnlyList<P2> pts) { _pts = pts; }

        public List<P2> Query(P2 center, P2 axis, double halfLength, double halfWidth)
        {
            P2 n = axis.Perp;
            var res = new List<P2>();
            foreach (var p in _pts)
            {
                var d = p - center;
                if (Math.Abs(d.Dot(axis)) <= halfLength && Math.Abs(d.Dot(n)) <= halfWidth) res.Add(p);
            }
            return res;
        }
    }
}
