using System;
using System.Collections.Generic;

namespace PointCloudWalls.Core
{
    /// <summary>
    /// Where the building really is: the plan cells in which the FLOOR or the CEILING was scanned.
    /// A horizontal slice at wall height also catches things that are not rooms of this floor at all - above all the mirror image of
    /// a room in window glass (a complete "room" beyond the facade, with walls as sharp as real ones), and whatever stands outside.
    /// Those have no floor and no ceiling of their own (a mirror image has a faint ghost of both, a few percent as dense as the real
    /// ones), so "floor or ceiling scanned here" separates them cleanly. Furniture hides the floor but not the ceiling, hence both.
    /// Pure math, meters.
    /// </summary>
    public sealed class InteriorMap
    {
        private readonly double _cell, _x0, _y0;
        private readonly int _nx, _ny;
        private readonly bool[] _inside;

        public int Cells { get; private set; }
        /// <summary>Scanned area in square meters.</summary>
        public double Area => Cells * _cell * _cell;

        /// <param name="floor">points of the floor band (around the wall base)</param>
        /// <param name="upper">points above the slice, up to the top of the walls (ceiling); may be empty</param>
        public InteriorMap(IReadOnlyList<P2> floor, IReadOnlyList<P2> upper, double cell = 0.40)
        {
            _cell = cell;
            double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
            foreach (var list in new[] { floor, upper })
                foreach (P2 p in list) { x0 = Math.Min(x0, p.X); y0 = Math.Min(y0, p.Y); x1 = Math.Max(x1, p.X); y1 = Math.Max(y1, p.Y); }
            if (x0 > x1) { _nx = _ny = 0; _inside = new bool[0]; return; }
            _x0 = x0 - 2 * cell; _y0 = y0 - 2 * cell;
            _nx = (int)((x1 - _x0) / cell) + 3; _ny = (int)((y1 - _y0) / cell) + 3;
            _inside = Dense(floor);
            var floorDense = (bool[])_inside.Clone();
            // The ceiling only fills in where furniture hides the floor - within reach of scanned floor. On its own it proves
            // nothing: a balcony soffit or a canopy outside the glass is a "ceiling" too.
            bool[] ceiling = Dense(upper);
            int reach = (int)Math.Ceiling(1.6 / cell);                        // 1.6 m: the depth of a bed, a desk, a row of cabinets
            var nearFloor = new bool[_nx * _ny];
            for (int ix = 0; ix < _nx; ix++)
                for (int iy = 0; iy < _ny; iy++)
                {
                    if (!_inside[ix * _ny + iy]) continue;
                    for (int jx = Math.Max(0, ix - reach); jx <= Math.Min(_nx - 1, ix + reach); jx++)
                        for (int jy = Math.Max(0, iy - reach); jy <= Math.Min(_ny - 1, iy + reach); jy++) nearFloor[jx * _ny + jy] = true;
                }
            for (int k = 0; k < _inside.Length; k++) if (ceiling[k] && nearFloor[k]) _inside[k] = true;

            // A room has a floor AND a ceiling. A patch of floor on its own - a terrace or a balcony seen through the glass, an
            // unusually strong reflection - is not a room; neither is a speck of a few cells.
            bool anyCeiling = false;
            foreach (bool cdense in ceiling) if (cdense) { anyCeiling = true; break; }
            var label = new int[_nx * _ny];
            var sizes = new List<int> { 0 }; var roofed = new List<int> { 0 }; var floored = new List<int> { 0 };
            var stack = new Stack<int>();
            for (int k0 = 0; k0 < _inside.Length; k0++)
            {
                if (!_inside[k0] || label[k0] != 0) continue;
                int id = sizes.Count, size = 0, under = 0, on = 0;
                label[k0] = id; stack.Push(k0);
                while (stack.Count > 0)
                {
                    int k = stack.Pop(); size++;
                    if (ceiling[k]) under++;
                    if (floorDense[k]) on++;
                    int ix = k / _ny, iy = k % _ny;
                    for (int dx = -1; dx <= 1; dx++)
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            int jx = ix + dx, jy = iy + dy;
                            if (jx < 0 || jy < 0 || jx >= _nx || jy >= _ny) continue;
                            int j = jx * _ny + jy;
                            if (_inside[j] && label[j] == 0) { label[j] = id; stack.Push(j); }
                        }
                }
                sizes.Add(size); roofed.Add(under); floored.Add(on);
            }
            for (int k = 0; k < _inside.Length; k++)
                if (_inside[k] && (sizes[label[k]] * cell * cell < 2.0 || floored[label[k]] < 0.3 * sizes[label[k]] || (anyCeiling && roofed[label[k]] < 0.3 * sizes[label[k]]))) _inside[k] = false;
            foreach (bool b in _inside) if (b) Cells++;
        }

        /// <summary>The cells where this layer of points is a properly scanned SURFACE.</summary>
        private bool[] Dense(IReadOnlyList<P2> list)
        {
            var dense = new bool[_nx * _ny];
            if (list.Count == 0) return dense;
            var cnt = new int[_nx * _ny];
            foreach (P2 p in list) cnt[Index(p)]++;
            var seen = new List<int>();
            foreach (int c in cnt) if (c > 0) seen.Add(c);
            if (seen.Count == 0) return dense;
            seen.Sort();
            // a properly scanned cell: the DENSE end of what was seen (most of the cells that hold a few points may well be ghosts
            // and strays outside the building, so the middle of the range says nothing)
            double need = Math.Max(4, 0.15 * seen[(int)(0.9 * (seen.Count - 1))]);
            // ... and a surface fills its neighbourhood: at least 5 of a cell and its 8 neighbours are that dense. The foot of a wall -
            // real or mirrored - is dense too, but only along a line; the faint ghost of a mirrored floor is nowhere dense.
            for (int ix = 1; ix < _nx - 1; ix++)
                for (int iy = 1; iy < _ny - 1; iy++)
                {
                    int good = 0;
                    for (int dx = -1; dx <= 1; dx++)
                        for (int dy = -1; dy <= 1; dy++)
                            if (cnt[(ix + dx) * _ny + iy + dy] >= need) good++;
                    dense[ix * _ny + iy] = good >= 5;
                }
            return dense;
        }

        private int Index(P2 p) => (int)((p.X - _x0) / _cell) * _ny + (int)((p.Y - _y0) / _cell);

        /// <summary>Floor or ceiling scanned at this spot?</summary>
        public bool Inside(P2 p)
        {
            int ix = (int)Math.Floor((p.X - _x0) / _cell), iy = (int)Math.Floor((p.Y - _y0) / _cell);
            return ix >= 0 && iy >= 0 && ix < _nx && iy < _ny && _inside[ix * _ny + iy];
        }

        /// <summary>Floor or ceiling scanned within this distance (m) of the spot? Walls stand at the edge of the scanned floor, so the
        /// slice is kept a little beyond it.</summary>
        public bool Near(P2 p, double margin)
        {
            int r = (int)Math.Ceiling(margin / _cell);
            int ix = (int)Math.Floor((p.X - _x0) / _cell), iy = (int)Math.Floor((p.Y - _y0) / _cell);
            for (int dx = -r; dx <= r; dx++)
                for (int dy = -r; dy <= r; dy++)
                {
                    int jx = ix + dx, jy = iy + dy;
                    if (jx >= 0 && jy >= 0 && jx < _nx && jy < _ny && _inside[jx * _ny + jy]) return true;
                }
            return false;
        }

        /// <summary>Share (0..1) of the strip beside the line a -> b, between the distances d0 and d1 from it (positive = on the left of
        /// a -> b, negative = on the right), where floor or ceiling was scanned.</summary>
        public double Share(P2 a, P2 b, double d0, double d1) => ShareOf(_inside, a, b, d0, d1);

        private double ShareOf(bool[] cells, P2 a, P2 b, double d0, double d1)
        {
            P2 u = (b - a).Normalized(), n = u.Perp;
            double len = (b - a).Length, lo = Math.Min(d0, d1), hi = Math.Max(d0, d1);
            int hit = 0, all = 0;
            for (double s = Math.Min(0.1, len / 2); s <= len - Math.Min(0.1, len / 2) + 1e-9; s += 0.1)
                for (double c = lo; c <= hi + 1e-9; c += 0.1)
                {
                    all++;
                    P2 p = a + u * s + n * c;
                    int ix = (int)Math.Floor((p.X - _x0) / _cell), iy = (int)Math.Floor((p.Y - _y0) / _cell);
                    if (ix >= 0 && iy >= 0 && ix < _nx && iy < _ny && ix * _ny + iy < cells.Length && cells[ix * _ny + iy]) hit++;
                }
            return all == 0 ? 0 : hit / (double)all;
        }
    }

    /// <summary>A point source without the points that lie outside the scanned interior (mirror images, things outside the building).</summary>
    public sealed class InteriorPointSource : IWallPointSource
    {
        private readonly IWallPointSource _inner;
        private readonly InteriorMap _map;
        private readonly double _margin;

        public InteriorPointSource(IWallPointSource inner, InteriorMap map, double margin) { _inner = inner; _map = map; _margin = margin; }

        public List<P2> Query(P2 center, P2 axis, double halfLength, double halfWidth)
        {
            List<P2> all = _inner.Query(center, axis, halfLength, halfWidth);
            var res = new List<P2>(all.Count);
            foreach (P2 q in all) if (_map.Near(q, _margin)) res.Add(q);
            return res;
        }
    }
}
